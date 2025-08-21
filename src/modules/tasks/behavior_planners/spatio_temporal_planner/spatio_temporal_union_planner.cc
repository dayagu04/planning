// #include <fastrtps/config.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

#include "basic_types.pb.h"
#include "config/basic_type.h"
#include "define/geometry.h"
#include "ego_planning_config.h"
#include "ego_state_manager.h"
#include "environmental_model.h"
#include "refline.h"
#include "debug_info_log.h"
#include "ifly_time.h"
#include "log.h"
#include "math/linear_interpolation.h"
#include "math_lib.h"
#include "quintic_poly_path.h"
#include "reference_path_manager.h"
#include "speed/st_point.h"
// #include "trajectory1d/second_order_time_optimal_trajectory.h"
// #include "trajectory1d/trajectory1d.h"

#include "planning_context.h"
#include "utils/frenet_coordinate_system.h"
#include "utils/kd_path.h"
#include "utils/spline.h"
#include "virtual_lane.h"
#include "virtual_lane_manager.h"
#include "spatio_temporal_union_planner.h"
#include "path_time_heuristic_optimizer.h"
#include "speed_data.h"
namespace planning {

using namespace planning_math;
using namespace pnc::spline;
using namespace pnc::mathlib;

namespace {
constexpr double kDistanceThresholdApproachToStopline = 10.0;
constexpr double kDistanceThresholdApproachToCrosswalk = 12.0;
constexpr int kEgoInIntersectionCount = 3;
constexpr int kDefaultDistanceAwayFromIntersection = 500.0;


}  // namespace

SpatioTemporalPlanner::SpatioTemporalPlanner(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session),
      session_(session),
      slt_grid_map_(config_builder, session),
      path_time_heuristic_optimizer_(config_builder, session) {
  config_ = config_builder->cast<EgoPlanningConfig>();
  name_ = "SpatioTemporalPlanner";
};

bool SpatioTemporalPlanner::Execute() {
  ILOG_INFO << "=======SpatioTemporalPlanner=======";
  if (!PreCheck()) {
    ILOG_DEBUG << "SpatioTemporalPlanner::PreCheck failed";
    return false;
  }

  auto spatio_temporal_union_plan = DebugInfoManager::GetInstance()
                                 .GetDebugInfoPb()
                                 ->mutable_spatio_temporal_union_plan();
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto& coarse_planning_info =
      lane_change_decider_output.coarse_planning_info;
  const auto lc_state = coarse_planning_info.target_state;
  const auto& virtual_lane_manager =
        session_->environmental_model().get_virtual_lane_manager();
  const auto& intersection_state = virtual_lane_manager->GetIntersectionState();
  spatio_temporal_union_plan->set_st_dp_is_sucess(false);
  spatio_temporal_union_plan->set_cost_time(0.0);
  spatio_temporal_union_plan->set_enable_using_st_plan(false);

  // 过滤自车处于非路口中的状态
  UpdateIntersection();
  if (!ego_in_intersection_state_) {
    ILOG_DEBUG << "SpatioTemporalPlanner::ego not in intersection!";
    last_enable_using_st_plan_ = false;
    return true;
  }

  // if (intersection_state != planning::common::IN_INTERSECTION) {
  //   return true;
  // }

  if (lc_state != kLaneKeeping) {
    ILOG_DEBUG << "SpatioTemporalPlanner::ego not in kLaneKeeping!";
    last_enable_using_st_plan_ = false;
    return true;
  }

  if (!config_.enable_use_spatio_temporal_planning) {
    ILOG_DEBUG << "SpatioTemporalPlanner::can not use spatio temporal union planning!";
    last_enable_using_st_plan_ = false;
    return true;
  }

  slt_grid_map_.RunOnce();

  auto &traj_points = session_->mutable_planning_context()
                          ->mutable_lane_change_decider_output()
                          .coarse_planning_info.trajectory_points;
  // TrajectoryPoints traj_points;
  const auto& agent_trajs_state =
      slt_grid_map_.SurroundForwardAgentsTrajs();
  const auto& virtual_agents_st_info =
      slt_grid_map_.VirtualAgentSTInFo();

  const double st_pre_time = IflyTime::Now_ms();
  path_time_heuristic_optimizer_.Process( traj_points, agent_trajs_state, virtual_agents_st_info, last_enable_using_st_plan_);

  // 更新障碍物决策
  path_time_heuristic_optimizer_.UpdateLateralObstacleDecision(agent_trajs_state);
  const double st_end_time = IflyTime::Now_ms();

  spatio_temporal_union_plan->set_cost_time(st_end_time - st_pre_time);
  spatio_temporal_union_plan->set_enable_using_st_plan(true);
  last_enable_using_st_plan_ = true;

  LogDebugInfo(traj_points, agent_trajs_state);

  return true;
}

void SpatioTemporalPlanner::LogDebugInfo(
    const TrajectoryPoints &traj_points,
    const std::vector<AgentFrenetSpatioTemporalInFo> &agents_state) {
  auto& lateral_obstacle_decision = session_->mutable_planning_context()
      ->mutable_lateral_obstacle_decider_output().lat_obstacle_decision;
  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  common::EnvironmentModelInfo *environment_model_debug_info =
      planning_debug_data->mutable_environment_model_info();
  auto spatio_temporal_union_plan = DebugInfoManager::GetInstance()
                                 .GetDebugInfoPb()
                                 ->mutable_spatio_temporal_union_plan();
  const auto& ego_state_mgr =
      session_->mutable_environmental_model()->get_ego_state_manager();

  const auto& virtual_lane_mgr =
      // session_->mutable_planning_context()->virtual_lane_manager();
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  const auto& current_lane = virtual_lane_mgr->get_current_lane();
  const double lateral_offset = session_->mutable_planning_context()
                                    ->lateral_offset_decider_output()
                                    .lateral_offset;
  std::shared_ptr<ReferencePath> origin_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(virtual_lane_mgr->current_lane_virtual_id(), false);
  if (origin_refline == nullptr) {
    return;
  }
  const auto& base_frenet_coord = origin_refline->get_frenet_coord();
  if (base_frenet_coord == nullptr) {
    return;
  }
  Point2D ego_cart_point(
      ego_state_mgr->ego_pose().x, ego_state_mgr->ego_pose().y);
  Point2D ego_frenet_point;
  if (!base_frenet_coord->XYToSL(ego_cart_point, ego_frenet_point)) {
    ILOG_ERROR << "SpatioTemporalPlanner::LogDebugInfo: Cart Point -> Frenet Point Failed!!";
  }

  auto origin_refline_points =
      spatio_temporal_union_plan->mutable_origin_refline_points();
  origin_refline_points->Clear();

  const auto &origin_lane_points = current_lane->lane_points();
  for (const auto &point : origin_lane_points) {
    Point2D ref_frenet_point(point.s, lateral_offset);
    Point2D origin_point;
    if (!base_frenet_coord->SLToXY(ref_frenet_point, origin_point)) {
      continue;
    }
    if (origin_point.x < ego_frenet_point.x) {
      continue;
    }

    planning::common::Point2d *Point = origin_refline_points->Add();
    Point->set_x(origin_point.x);
    Point->set_y(origin_point.y);
  }

  spatio_temporal_union_plan->set_st_dp_is_sucess(
        path_time_heuristic_optimizer_.GetStDpIsSuccess());

  // trajectory points
  auto mutable_traj_points = spatio_temporal_union_plan->mutable_trajectory_points();
  mutable_traj_points->Clear();
  for (size_t i = 0; i < traj_points.size(); i++) {
    planning::common::TrajectoryPoint *traj_point = mutable_traj_points->Add();
    traj_point->set_x(traj_points[i].x);
    traj_point->set_y(traj_points[i].y);
    traj_point->set_s(traj_points[i].s);
    traj_point->set_l(traj_points[i].l);
    traj_point->set_a(traj_points[i].a);
    traj_point->set_t(traj_points[i].t);
    traj_point->set_v(traj_points[i].v);
    traj_point->set_curvature(traj_points[i].curvature);
    traj_point->set_heading_angle(traj_points[i].heading_angle);
    traj_point->set_frenet_valid(traj_points[i].frenet_valid);
  }

  // 更新障碍物横向决策proto
  if (!agents_state.empty()) {
    for (const auto& agent_state : agents_state) {
      auto obstacles = environment_model_debug_info->mutable_obstacle();
      for (size_t i = 0; i < environment_model_debug_info->obstacle_size(); ++i) {
        auto obs = obstacles->Mutable(i);
        if (obs->id() == agent_state.agent_id) {
          obs->clear_lat_decision();
          obs->set_lat_decision(
              static_cast<uint32_t>(lateral_obstacle_decision[obs->id()]));
          break;
        }
      }
    }
  }

  return;
}

void SpatioTemporalPlanner::UpdateIntersection() {
  const auto &tfl_decider = session_->mutable_planning_context()
                          ->mutable_traffic_light_decider_output();
  const auto intersection_state = session_->environmental_model()
                              .get_virtual_lane_manager()
                              ->GetIntersectionState();
  const double distance_to_stopline = session_->environmental_model()
                                    .get_virtual_lane_manager()
                                    ->GetEgoDistanceToStopline();
  const double distance_to_crosswalk = session_->environmental_model()
                                    .get_virtual_lane_manager()
                                    ->GetEgoDistanceToCrosswalk();
  bool current_intersection_state =
      intersection_state == common::IntersectionState::IN_INTERSECTION ||
      distance_to_stopline <= kDistanceThresholdApproachToStopline;
      // distance_to_crosswalk <= kDistanceThresholdApproachToCrosswalk;
  if (current_intersection_state) {
    intersection_count_ = kEgoInIntersectionCount;
  } else {
    intersection_count_ = std::max(intersection_count_ - 1, 0);
  }

  ego_in_intersection_state_ =
      intersection_count_ > 0 && !tfl_decider.is_small_front_intersection;
  return;
}

}
