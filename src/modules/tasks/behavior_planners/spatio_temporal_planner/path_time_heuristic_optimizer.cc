#include "path_time_heuristic_optimizer.h"
#include <cmath>
#include "define/geometry.h"
#include "environmental_model.h"
#include "debug_info_log.h"
#include "ego_planning_config.h"
#include "ego_state_manager.h"
#include "frenet_ego_state.h"
#include "history_obstacle_manager.h"
#include "lateral_obstacle.h"
#include "obstacle_manager.h"
#include "reference_path_manager.h"
#include "traffic_light_decision_manager.h"
#include "trajectory/trajectory_stitcher.h"
#include "virtual_lane_manager.h"
#include "planning_context.h"

#include "log.h"

namespace planning {
using namespace planning_math;

PathTimeHeuristicOptimizer::PathTimeHeuristicOptimizer(
    const EgoPlanningConfigBuilder *config_builder,
    framework::Session *session)
    : slt_graph_(config_builder, session),
      session_(session) {
  config_ = config_builder->cast<DpStSpeedOptimizerConfig>();

}

bool PathTimeHeuristicOptimizer::SearchPathTimeGraph(
    TrajectoryPoints &traj_points,
    const std::vector<AgentFrenetSpatioTemporalInFo> &agent_trajs,
    const std::vector<VirtualAgentSpatioTemporalInFo> &virtual_agents_info,
    const bool &last_enable_using_st_plan) {

  if (!slt_graph_.Search(traj_points, agent_trajs, virtual_agents_info, last_enable_using_st_plan)) {
    ILOG_DEBUG << "failed to search graph with dynamic programming";
    return false;
  }
  return true;
}

bool PathTimeHeuristicOptimizer::Process(
    TrajectoryPoints &traj_points,
    const std::vector<AgentFrenetSpatioTemporalInFo> &agent_trajs,
    const std::vector<VirtualAgentSpatioTemporalInFo> &virtual_agents_info,
    const bool &last_enable_using_st_plan) {
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& virtual_lane_mgr =
      // session_->mutable_planning_context()->virtual_lane_manager();
      session_->mutable_environmental_model()->get_virtual_lane_manager();

  const auto& current_lane = virtual_lane_mgr->get_current_lane();
  planning_init_point_ =
      ego_state_manager->planning_init_point();
  std::shared_ptr<ReferencePath> base_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(virtual_lane_mgr->current_lane_virtual_id(), false);
  base_frenet_coord_ = base_refline->get_frenet_coord();
  st_dp_is_sucess_ = true;

  if (!SearchPathTimeGraph(traj_points, agent_trajs, virtual_agents_info, last_enable_using_st_plan)) {
    ILOG_DEBUG << "PathTimeHeuristicOptimizer::Process() SearchPathTimeGraph failed!!";
    st_dp_is_sucess_ = false;
    // FallbackFunction(traj_points);
  }

  GenerateEgoBoxSet(traj_points);

  return true;
}

void PathTimeHeuristicOptimizer::FallbackFunction(TrajectoryPoints &traj_points) {
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  double ego_v= ego_state_manager->ego_v();
  const double k_max_deceleration = -3.5;
  const double acc_coeff = 0.5;
  const double ego_min_vs = 0.1;
  Point2D ego_frenet_point;
  Point2D ego_cart_point{planning_init_point_.lat_init_state.x(),
                         planning_init_point_.lat_init_state.y()};
  double ego_s = 0.0;
  double ego_l = 0.0;
  if (!base_frenet_coord_->XYToSL(ego_cart_point, ego_frenet_point)) {
    ILOG_ERROR << "PathTimeHeuristicOptimizer::FallbackFunction: Cart Point -> Frenet Point Failed!!!";
  } else {
    ego_s = ego_frenet_point.x;
    ego_l = ego_frenet_point.y;
  }

  const double ego_v_angle = ego_state_manager->ego_v_angle();
  const double ref_theta =
      base_frenet_coord_->GetPathCurveHeading(ego_s);
  const double theta_diff = NormalizeAngle(ref_theta - ego_v_angle);
  double v_s = ego_v * std::cos(std::fabs(theta_diff));
  const double ego_deceleration_time = 0.0;

  // const auto& vehicle_param =
  //   VehicleConfigurationContext::Instance()->get_vehicle_param();
  // const double max_deceleration =
  //     -1.0 *
  //     std::min(std::abs(vehicle_param.max_deceleration),
  //              std::abs(config_.max_deceleration));
  // const double maximum_deceleration_duration = v_s / max_deceleration;

  TrajectoryPoint point;
  int target_count = 0;
  for (size_t i = 0; i < traj_points.size(); i++) {
    if (i == 0) {
      point.s = ego_s;
      point.l = ego_l;
      point.heading_angle = ref_theta;
      point.t = traj_points[i].t;
      point.x = ego_cart_point.x;
      point.y = ego_cart_point.y;
      traj_points[i] = point;
      target_count = i;
    } else {
      double delta_time = traj_points[i].t - traj_points[0].t;
      if (delta_time <= ego_deceleration_time) {
        double dis_to_init_point = v_s * delta_time + acc_coeff * k_max_deceleration * delta_time * delta_time;
        Point2D cur_frenet_point;
        cur_frenet_point.x = ego_s + dis_to_init_point;
        cur_frenet_point.y = ego_l;

        // point.x = sample_point.x();
        // point.y = sample_point.y();
        // point.heading_angle = lane_change_quintic_path.heading(traj_points[i].t);

        Point2D cart_point;
        if (!base_frenet_coord_->SLToXY(cur_frenet_point, cart_point)) {
          ILOG_ERROR << "ERROR! Frenet Point -> Cart Point Failed!!!";
        }
        point.s = cur_frenet_point.x;
        point.l = cur_frenet_point.y;
        point.t = traj_points[i].t;
        point.x = cart_point.x;
        point.y = cart_point.y;
        planning_math::PathPoint cur_s_nearest_point =
            base_frenet_coord_->GetPathPointByS(cur_frenet_point.x);
        point.heading_angle =
            base_frenet_coord_->GetPathCurveHeading(cur_s_nearest_point.s());

        traj_points[i] = point;
        target_count = i;
      } else {
        double constant_speed_time = delta_time - ego_deceleration_time;
        Point2D cur_frenet_point;
        cur_frenet_point.x = traj_points[target_count].s + constant_speed_time * v_s;
        cur_frenet_point.y = ego_l;

        // point.x = sample_point.x();
        // point.y = sample_point.y();
        // point.heading_angle = lane_change_quintic_path.heading(traj_points[i].t);

        Point2D cart_point;
        if (!base_frenet_coord_->SLToXY(cur_frenet_point, cart_point)) {
          ILOG_ERROR << "ERROR! Frenet Point -> Cart Point Failed!!!";
        }
        point.s = cur_frenet_point.x;
        point.l = cur_frenet_point.y;
        point.t = traj_points[i].t;
        point.x = cart_point.x;
        point.y = cart_point.y;
        planning_math::PathPoint cur_s_nearest_point =
            base_frenet_coord_->GetPathPointByS(cur_frenet_point.x);
        point.heading_angle =
            base_frenet_coord_->GetPathCurveHeading(cur_s_nearest_point.s());

        traj_points[i] = point;
      }
    }
  }

  return;
}

void PathTimeHeuristicOptimizer::GenerateEgoBoxSet(TrajectoryPoints &traj_points) {
  if (!traj_points.empty()) {
    TrajectoryPoint traj_point;
    std::vector<planning_math::Vec2d> vertices;
    for (int i = 0; i < traj_points.size(); i++) {
      vertices.clear();
      traj_point = traj_points[i];
      GetVehicleVertices(traj_point, vertices);
      AABox2d ego_box(vertices);
      ego_box_set_[i] = ego_box;
    }
  }
}

void PathTimeHeuristicOptimizer::GetVehicleVertices(
    const TrajectoryPoint &traj_point,
    std::vector<planning_math::Vec2d>& vertices) {
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double angle = traj_point.heading_angle;

  double cos_theta = cos(angle);
  double sin_theta = sin(angle);

  double c_x = traj_point.s + vehicle_param.rear_axle_to_center * cos_theta;
  double c_y = traj_point.l + vehicle_param.rear_axle_to_center * sin_theta;

  double d_wx = vehicle_param.width * 0.5 * sin_theta;
  double d_wy = vehicle_param.width * 0.5 * cos_theta;
  double d_lx = vehicle_param.length * 0.5 * cos_theta;
  double d_ly = vehicle_param.length * 0.5 * sin_theta;

  // Counterclockwise from left-front vertex
  vertices.emplace_back(Vec2d(c_x - d_wx + d_lx, c_y + d_wy + d_ly));
    // vertices.emplace_back(Vec2d(c_x - d_wx, c_y + d_wy));
  vertices.emplace_back(Vec2d(c_x - d_wx - d_lx, c_y - d_ly + d_wy));
    // vertices.emplace_back(Vec2d(c_x - d_lx, c_y - d_ly));
  vertices.emplace_back(Vec2d(c_x + d_wx - d_lx, c_y - d_wy - d_ly));
    // vertices.emplace_back(Vec2d(c_x + d_wx, c_y - d_wy));
  vertices.emplace_back(Vec2d(c_x + d_wx + d_lx, c_y + d_ly - d_wy));
    // vertices.emplace_back(Vec2d(c_x + d_lx, c_y + d_ly));

  return;
}

void PathTimeHeuristicOptimizer::UpdateLateralObstacleDecision(
    const std::vector<AgentFrenetSpatioTemporalInFo> &agent_trajs) {
  const int k_ego_traj_points_nums = 16;
  auto& lateral_obstacle_decision =
      session_->mutable_planning_context()
      ->mutable_lateral_obstacle_decider_output().lat_obstacle_decision;
  const auto& lateral_obstacle_history_info =
      session_->mutable_planning_context()
      ->mutable_lateral_obstacle_decider_output().lateral_obstacle_history_info;
  const double longit_overlap_Threshold = 0.1;
  double longit_dis = 100.0;
  AABox2d agent_box;
  for (const auto& agent : agent_trajs) {
    auto iter = lateral_obstacle_decision.find(agent.agent_id);
    auto iter_history = lateral_obstacle_history_info.find(agent.agent_id);
    if (iter_history == lateral_obstacle_history_info.end() ||
        iter_history->second.cut_in_or_cross) {
      continue;
    }
    if (iter != lateral_obstacle_decision.end()) {
      for (int i = 0; i < k_ego_traj_points_nums; i++) {
        auto it = agent.agent_boxs_set.find(i);
        if (it != agent.agent_boxs_set.end()) {
          agent_box = it->second;
          longit_dis = agent_box.LongitDistanceTo(ego_box_set_[i]);
          if (longit_dis < longit_overlap_Threshold) {
            if (ego_box_set_[i].center_y() < agent_box.center_y()) {
              lateral_obstacle_decision[agent.agent_id] = LatObstacleDecisionType::RIGHT;
            } else if (ego_box_set_[i].center_y() > agent_box.center_y()) {
              lateral_obstacle_decision[agent.agent_id] = LatObstacleDecisionType::LEFT;
            } else {
              lateral_obstacle_decision[agent.agent_id] = LatObstacleDecisionType::IGNORE;
            }
            break;
          } else {
            lateral_obstacle_decision[agent.agent_id] = LatObstacleDecisionType::IGNORE;
          }
        }
      }
    }
  }

  return;
}

}