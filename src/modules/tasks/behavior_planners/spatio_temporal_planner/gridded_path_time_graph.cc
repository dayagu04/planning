#include "gridded_path_time_graph.h"
#include <math.h>
#include "behavior_planners/lateral_offset_decider/lateral_offset_decider_utils.h"
#include "behavior_planners/spatio_temporal_planner/slt_graph_point.h"
#include "behavior_planners/spatio_temporal_planner/slt_point.h"
#include "behavior_planners/spatio_temporal_planner/speed_data.h"
#include "config/basic_type.h"
#include "math/box2d.h"
#include "src/modules/common/math/line_segment2d.h"
#include "src/modules/common/math/curve1d/cubic_polynomial_curve1d.h"
#include "src/modules/common/math/math_utils.h"
#include "src/modules/tasks/task_interface/lane_change_decider_output.h"
#include "src/modules/common/trajectory1d/variable_coordinate_time_optimal_trajectory.h"
#include "src/modules/common/trajectory1d/trajectory1d.h"
#include "src/library/advanced_ctrl_lib/include/spline.h"
#include "src/modules/common/math/linear_interpolation.h"
#include "src/modules/context/planning_context.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>
#include "log.h"
#include "vec2d.h"



namespace planning {
using namespace planning_math;


namespace {
constexpr double kDoubleEpsilon = 1.0e-6;
constexpr double kMaxLateralDistance = 20.0;
constexpr double kDynamicObsWeight = 1e-4;
constexpr double kDynamicObsConsiderTime = 3.0;
constexpr double kPathCostComputeSampleTime = 0.2;
constexpr double kDynamicObstacleCostSampleTime = 0.2;
constexpr double kPlanningUpperSpeedLimit = 12.5;
constexpr double kHighVel = 100 / 3.6;
constexpr double kVirtualAgentBuffer = 3.6;
constexpr double kJerkMin = -2.0;
constexpr double kJerkMax = 2.0;
constexpr double kPPrecision = 0.1;
constexpr double kDeltaTime = 0.2;
constexpr double kDefaultLateralDistanceBuffer = 0.4;
constexpr double kDefaultTtc = 3.0;
constexpr double kDefaultStaticObstacleLateralDis = 0.5;
constexpr int kDefaultPlanningDuration = 5.0;

}
// namespace

GriddedPathTimeGraph::GriddedPathTimeGraph(
    const EgoPlanningConfigBuilder *config_builder,
    framework::Session *session)
    : session_(session) {
  gridded_path_time_graph_config_ =
      config_builder->cast<DpStSpeedOptimizerConfig>();
  dp_poly_path_config_ =
      config_builder->cast<DpPolyPathConfig>();
  speed_limit_config_ = config_builder->cast<SpeedLimitConfig>();
  const auto& vehicle_param =
    VehicleConfigurationContext::Instance()->get_vehicle_param();
  // Safety approach preventing unreachable acceleration/deceleration
  max_acceleration_ =
      std::min(std::abs(vehicle_param.max_acceleration),
               std::abs(gridded_path_time_graph_config_.max_acceleration));
  max_deceleration_ =
      -1.0 *
      std::min(std::abs(vehicle_param.max_deceleration),
               std::abs(gridded_path_time_graph_config_.max_deceleration));
  half_lateral_sample_nums_ = dp_poly_path_config_.sample_points_num_each_level;
}

bool GriddedPathTimeGraph::Search(
    TrajectoryPoints &traj_points,
    const std::vector<AgentFrenetSpatioTemporalInFo> &agent_trajs,
    const std::vector<VirtualAgentSpatioTemporalInFo> &virtual_agents_info,
    const bool &last_enable_using_st_plan) {
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& virtual_lane_mgr =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  std::shared_ptr<ReferencePathManager> reference_path_mgr =
      session_->mutable_environmental_model()->get_reference_path_manager();
  const auto& vehicle_param =
    VehicleConfigurationContext::Instance()->get_vehicle_param();

  const auto& current_lane = virtual_lane_mgr->get_current_lane();
  const auto &origin_lane_points = current_lane->lane_points();
  planning_init_point_ =
      ego_state_manager->planning_init_point();
  std::shared_ptr<ReferencePath> current_refline_ =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(current_lane->get_virtual_id(), false);
  current_lane_coord_ = current_refline_->get_frenet_coord();
  const auto &route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();
  const bool active = session_->environmental_model().GetVehicleDbwStatus();
  const auto& agent_manager =
      session_->environmental_model().get_agent_manager();
  const auto& cipv_info = session_->planning_context().cipv_decider_output();
  const auto& lead_agent = agent_manager->GetAgent(cipv_info.cipv_id());
  double longit_risk_distance_between_obstacle = dp_poly_path_config_.obstacle_longit_risk_distance;
  Point2D ego_cart_pose{planning_init_point_.x,
                        planning_init_point_.y};
  if (!active) {
    ego_cart_pose.x = ego_state_manager->ego_pose().x;
    ego_cart_pose.y = ego_state_manager->ego_pose().y;
  }
  const double ego_cart_heading = ego_state_manager->heading_angle();
  if (!current_lane_coord_->XYToSL(ego_cart_pose, ego_frenet_pose_)) {
    ILOG_DEBUG << "GriddedPathTimeGraph::Search() find ego pose in current lane failed!";
  }

  total_length_s_ =
      ego_frenet_pose_.x + ego_state_manager->ego_v_cruise() * kDefaultPlanningDuration;
  double target_s = total_length_s_;
  if (lead_agent != nullptr) {
    longit_risk_distance_between_obstacle =
        (ego_state_manager->ego_v() - lead_agent->speed()) * kDefaultTtc;
    // if (lead_agent->is_static()) {
    //   for (const auto& agent : agent_trajs) {
    //     if (agent.agent_id == lead_agent->agent_id() &&
    //         (agent.max_agent_box.min_y() < kDefaultStaticObstacleLateralDis &&
    //           agent.max_agent_box.max_y() > - kDefaultStaticObstacleLateralDis)) {
    //       target_s = std::min(target_s, agent.max_agent_box.min_x());
    //       break;
    //     }
    //   }
    // }
  }
  longit_risk_distance_between_obstacle =
      std::max(longit_risk_distance_between_obstacle, dp_poly_path_config_.obstacle_longit_risk_distance);

  // 考虑虚拟障碍物
  if (!virtual_agents_info.empty()) {
    for (const auto& virtual_agent : virtual_agents_info) {
      if (virtual_agent.frenet_slt_info.s() < target_s) {
        target_s = virtual_agent.frenet_slt_info.s();
      }
    }
  }
  target_s += kVirtualAgentBuffer;

  auto spatio_temporal_union_plan_input = DebugInfoManager::GetInstance()
                                 .GetDebugInfoPb()
                                 ->mutable_spatio_temporal_union_plan_input();
  auto ego_init_state =
      spatio_temporal_union_plan_input->mutable_init_state();
  ego_init_state->Clear();
  ego_init_state->set_s(ego_frenet_pose_.x);
  ego_init_state->set_l(ego_frenet_pose_.y);
  ego_init_state->set_v0(planning_init_point_.v);
  ego_init_state->set_v_cruise(ego_state_manager->ego_v_cruise());
  ego_init_state->set_a(planning_init_point_.a);
  ego_init_state->set_steer_angle(ego_state_manager->ego_steer_angle());
  ego_init_state->set_heading_angle(ego_state_manager->heading_angle());

  auto dp_search_paramms =
      spatio_temporal_union_plan_input->mutable_dp_search_paramms();
  dp_search_paramms->Clear();
  dp_search_paramms->set_total_length_t(gridded_path_time_graph_config_.total_length_t);
  dp_search_paramms->set_unit_t(gridded_path_time_graph_config_.default_unit_t);
  dp_search_paramms->set_dense_unit_s(gridded_path_time_graph_config_.dense_unit_s);
  dp_search_paramms->set_sparse_unit_s(gridded_path_time_graph_config_.sparse_unit_s);
  dp_search_paramms->set_unit_l(dp_poly_path_config_.lateral_sample_offset);
  dp_search_paramms->set_dimension_l(dp_poly_path_config_.sample_points_num_each_level);
  dp_search_paramms->set_dense_dimension_s(gridded_path_time_graph_config_.dense_dimension_s);
  dp_search_paramms->set_max_acceleration(max_acceleration_);
  dp_search_paramms->set_max_deceleration(max_deceleration_);
  dp_search_paramms->set_enable_use_parallel_calculate_cost(gridded_path_time_graph_config_.enable_use_parallel_calculate_cost);

  auto lat_path_weight_params = spatio_temporal_union_plan_input->mutable_lat_path_weight_params();
  lat_path_weight_params->Clear();
  lat_path_weight_params->set_path_l_cost_param_l0(dp_poly_path_config_.path_l_cost_param_l0);
  lat_path_weight_params->set_path_l_cost_param_k(dp_poly_path_config_.path_l_cost_param_k);
  lat_path_weight_params->set_path_l_cost_param_b(dp_poly_path_config_.path_l_cost_param_b);
  lat_path_weight_params->set_path_l_cost(dp_poly_path_config_.path_l_cost);
  lat_path_weight_params->set_path_dl_cost(dp_poly_path_config_.path_dl_cost);
  lat_path_weight_params->set_path_ddl_cost(dp_poly_path_config_.path_ddl_cost);
  lat_path_weight_params->set_path_end_l_cost(dp_poly_path_config_.path_end_l_cost);

  auto stitching_cost_params = spatio_temporal_union_plan_input->mutable_stitching_cost_params();
  stitching_cost_params->set_path_l_stitching_cost_param(dp_poly_path_config_.path_l_stitching_cost_param);
  stitching_cost_params->set_stitching_cost_time_decay_factor(dp_poly_path_config_.stitching_cost_time_decay_factor);
  // lat_path_weight_params->set_path_l_stitching_cost_param(dp_poly_path_config_.path_l_stitching_cost_param);
  // lat_path_weight_params->set_stitching_cost_time_decay_factor(dp_poly_path_config_.stitching_cost_time_decay_factor);

  auto dp_dynamic_agent_weight_params = spatio_temporal_union_plan_input->mutable_dp_dynamic_agent_weight_params();
  dp_dynamic_agent_weight_params->Clear();
  dp_dynamic_agent_weight_params->set_obstacle_ignore_distance(dp_poly_path_config_.obstacle_ignore_distance);
  dp_dynamic_agent_weight_params->set_obstacle_longit_collision_distance(dp_poly_path_config_.obstacle_longit_collision_distance);
  dp_dynamic_agent_weight_params->set_obstacle_longit_risk_distance(longit_risk_distance_between_obstacle);
  dp_dynamic_agent_weight_params->set_obstacle_collision_cost(dp_poly_path_config_.obstacle_collision_cost);
  dp_dynamic_agent_weight_params->set_dynamic_obstacle_weight(dp_poly_path_config_.dynamic_obstacle_weight);
  dp_dynamic_agent_weight_params->set_default_obstacle_cost_weight(dp_poly_path_config_.default_obstacle_cost_weight);
  dp_dynamic_agent_weight_params->set_obstacle_lateral_risk_distance(
      dp_poly_path_config_.obstacle_lateral_risk_distance);
  dp_dynamic_agent_weight_params->set_obstacle_lateral_collision_distance(dp_poly_path_config_.obstacle_lateral_collision_distance);
  dp_dynamic_agent_weight_params->set_obstacle_collision_cost_without_lateral_overlap(
      dp_poly_path_config_.obstacle_collision_cost_without_lateral_overlap);

  auto long_weight_params = spatio_temporal_union_plan_input->mutable_long_weight_params();
  long_weight_params->Clear();
  long_weight_params->set_spatial_potential_penalty(gridded_path_time_graph_config_.spatial_potential_penalty);
  long_weight_params->set_keep_clear_low_speed_penalty(gridded_path_time_graph_config_.keep_clear_low_speed_penalty);
  long_weight_params->set_exceed_speed_penalty(gridded_path_time_graph_config_.exceed_speed_penalty);
  long_weight_params->set_low_speed_penalty(gridded_path_time_graph_config_.low_speed_penalty);
  long_weight_params->set_reference_speed_penalty(gridded_path_time_graph_config_.reference_speed_penalty);
  long_weight_params->set_default_speed_cost(gridded_path_time_graph_config_.default_speed_cost);
  long_weight_params->set_accel_penalty(gridded_path_time_graph_config_.accel_penalty);
  long_weight_params->set_decel_penalty(gridded_path_time_graph_config_.decel_penalty);
  long_weight_params->set_positive_jerk_coeff(gridded_path_time_graph_config_.positive_jerk_coeff);
  long_weight_params->set_negative_jerk_coeff(gridded_path_time_graph_config_.negative_jerk_coeff);

  auto speed_limit_params = spatio_temporal_union_plan_input->mutable_speed_limit_params();
  speed_limit_params->Clear();
  speed_limit_params->set_straight_ramp_v_limit(speed_limit_config_.straight_ramp_v_limit);
  speed_limit_params->set_v_limit_near_ramp_zone(speed_limit_config_.v_limit_near_ramp_zone);
  speed_limit_params->set_brake_dis_near_ramp_zone(speed_limit_config_.brake_dis_near_ramp_zone);
  speed_limit_params->set_dis_near_ramp_zone(speed_limit_config_.dis_near_ramp_zone);
  speed_limit_params->set_v_limit_ramp(speed_limit_config_.v_limit_ramp);
  speed_limit_params->set_acc_to_ramp(speed_limit_config_.acc_to_ramp);
  speed_limit_params->set_dis_curv(speed_limit_config_.dis_curv);
  speed_limit_params->set_v_intersection_min_limit(speed_limit_config_.v_intersection_min_limit);
  speed_limit_params->set_pre_accelerate_distance_for_merge(speed_limit_config_.pre_accelerate_distance_for_merge);
  speed_limit_params->set_preview_x(speed_limit_config_.preview_x);
  speed_limit_params->set_t_curv(speed_limit_config_.t_curv);
  speed_limit_params->set_dis_to_merge(route_info_output.distance_to_first_road_merge);
  speed_limit_params->set_dis_to_ramp(route_info_output.dis_to_ramp);
  speed_limit_params->set_is_continuous_ramp(virtual_lane_mgr->is_continuous_ramp());
  speed_limit_params->set_is_on_ramp(route_info_output.is_on_ramp);

  auto ref_points_vec =
      spatio_temporal_union_plan_input->mutable_ref_points_vec();
  ref_points_vec->Clear();
  for (const auto &point : origin_lane_points) {
    planning::common::Point2d *Point = ref_points_vec->Add();
    Point->set_x(point.local_point.x);
    Point->set_y(point.local_point.y);
  }

  if (!spatio_temporal_union_plan_dp_.Update(
      traj_points, agent_trajs, *spatio_temporal_union_plan_input, target_s,
      *current_lane_coord_, half_lateral_sample_nums_, last_enable_using_st_plan)) {
    return false;
  }

  return true;
}

}  // namespace planning