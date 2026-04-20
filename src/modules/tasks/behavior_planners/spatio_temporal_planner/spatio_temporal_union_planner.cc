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
#include "debug_info_log.h"
#include "define/geometry.h"
#include "ego_planning_config.h"
#include "ego_state_manager.h"
#include "environmental_model.h"
#include "ifly_time.h"
#include "log.h"
#include "math/linear_interpolation.h"
#include "math_lib.h"
#include "quintic_poly_path.h"
#include "reference_path_manager.h"
#include "refline.h"
#include "speed/st_point.h"
// #include "trajectory1d/second_order_time_optimal_trajectory.h"
// #include "trajectory1d/trajectory1d.h"

#include "path_time_heuristic_optimizer.h"
#include "planning_context.h"
#include "spatio_temporal_union_planner.h"
#include "speed_data.h"
#include "utils/frenet_coordinate_system.h"
#include "utils/kd_path.h"
#include "utils/spline.h"
#include "virtual_lane.h"
#include "virtual_lane_manager.h"
namespace planning {

using namespace planning_math;
using namespace pnc::spline;
using namespace pnc::mathlib;

namespace {
constexpr double kDistanceThresholdApproachToStopline = 10.0;
constexpr double kDistanceThresholdApproachToCrosswalk = 12.0;
constexpr int kEgoInIntersectionCount = 3;
constexpr int kDefaultDistanceAwayFromIntersection = 500.0;
constexpr int kVirtualAreaContinuousThreshold = 5;

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
  spatio_temporal_union_plan_.Clear();
  spatio_temporal_union_plan_input_.Clear();
  if (!PreCheck()) {
    ILOG_DEBUG << "SpatioTemporalPlanner::PreCheck failed";
    LogDebugInfo();
    return false;
  }
  // auto spatio_temporal_union_plan = DebugInfoManager::GetInstance()
  //                                .GetDebugInfoPb()
  //                                ->mutable_spatio_temporal_union_plan();
  const auto &lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto &coarse_planning_info =
      lane_change_decider_output.coarse_planning_info;
  auto &spatio_temporal_union_plan_output =
      session_->mutable_planning_context()
          ->mutable_spatio_temporal_union_plan_output();
  const auto lc_state = coarse_planning_info.target_state;
  bool is_dynamic_agent_emergence_lc = coarse_planning_info.lane_change_request_source == RequestSource::DYNAMIC_AGENT_EMERGENCE_AVOID_REQUEST;
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto &intersection_state = virtual_lane_manager->GetIntersectionState();
  const auto& construction_scene_output =
      session_->environmental_model().get_construction_scene_manager()->get_construction_scene_output();
  const bool is_construction_scene_ref_path =
      coarse_planning_info.reference_path->GetIsConstructionScene();
  spatio_temporal_union_plan_.set_st_dp_is_sucess(false);
  spatio_temporal_union_plan_.set_cost_time(0.0);
  spatio_temporal_union_plan_.set_enable_using_st_plan(false);
  spatio_temporal_union_plan_output.st_dp_is_sucess = false;
  spatio_temporal_union_plan_output.cost_time = 0.0;
  spatio_temporal_union_plan_output.enable_using_st_plan = false;

  // 过滤自车处于非路口中的状态
  UpdateIntersection();
  if (!ego_in_intersection_state_ && !construction_scene_output.enable_construction_passage) {
    ILOG_DEBUG << "SpatioTemporalPlanner::ego not in intersection!";
    last_enable_using_st_plan_ = false;
    LogDebugInfo();
    return true;
  }

  // if (intersection_state != planning::common::IN_INTERSECTION) {
  //   return true;
  // }
  
  if (is_dynamic_agent_emergence_lc && (lc_state == kLaneChangePropose || lc_state == kLaneChangeComplete)) {
    // 紧急变道场景允许在 Propose/Complete 状态使用时空搜索
    // pass
  } else if (lc_state != kLaneKeeping) {
    ILOG_DEBUG << "SpatioTemporalPlanner::ego not in kLaneKeeping!";
    last_enable_using_st_plan_ = false;
    LogDebugInfo();
    return true;
  }

  if (!config_.enable_use_spatio_temporal_planning) {
    ILOG_DEBUG
        << "SpatioTemporalPlanner::can not use spatio temporal union planning!";
    last_enable_using_st_plan_ = false;
    LogDebugInfo();
    return true;
  }

  slt_grid_map_.RunOnce(spatio_temporal_union_plan_input_);

  auto &traj_points = session_->mutable_planning_context()
                          ->mutable_lane_change_decider_output()
                          .coarse_planning_info.trajectory_points;
  // TrajectoryPoints traj_points;
  const auto &agent_trajs_state = slt_grid_map_.SurroundForwardAgentsTrajs();
  const auto &virtual_agents_st_info = slt_grid_map_.VirtualAgentSTInFo();

  const double st_pre_time = IflyTime::Now_ms();
  path_time_heuristic_optimizer_.Process(
      traj_points, agent_trajs_state, virtual_agents_st_info,
      last_enable_using_st_plan_, spatio_temporal_union_plan_input_,
      ego_in_intersection_state_, &spatio_temporal_union_plan_);

  // 更新障碍物决策
  path_time_heuristic_optimizer_.UpdateLateralObstacleDecision(
      agent_trajs_state);
  const double st_end_time = IflyTime::Now_ms();

  spatio_temporal_union_plan_.set_cost_time(st_end_time - st_pre_time);
  spatio_temporal_union_plan_.set_enable_using_st_plan(true);
  spatio_temporal_union_plan_output.cost_time = st_end_time - st_pre_time;
  spatio_temporal_union_plan_output.enable_using_st_plan = true;
  last_enable_using_st_plan_ = true;

  PostProcessing(traj_points, agent_trajs_state);

  LogDebugInfo();

  return true;
}

void SpatioTemporalPlanner::PostProcessing(
    const TrajectoryPoints &traj_points,
    const std::vector<AgentFrenetSpatioTemporalInFo> &agents_state) {
  auto &lateral_obstacle_decision =
      session_->mutable_planning_context()
          ->mutable_lateral_obstacle_decider_output()
          .lat_obstacle_decision;
  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  common::EnvironmentModelInfo *environment_model_debug_info =
      planning_debug_data->mutable_environment_model_info();

  auto &spatio_temporal_union_plan_output =
      session_->mutable_planning_context()
          ->mutable_spatio_temporal_union_plan_output();

  spatio_temporal_union_plan_.set_st_dp_is_sucess(
      path_time_heuristic_optimizer_.GetStDpIsSuccess());
  spatio_temporal_union_plan_output.st_dp_is_sucess =
      path_time_heuristic_optimizer_.GetStDpIsSuccess();

  // trajectory points
  auto mutable_traj_points =
      spatio_temporal_union_plan_.mutable_trajectory_points();
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
    for (const auto &agent_state : agents_state) {
      auto obstacles = environment_model_debug_info->mutable_obstacle();
      for (size_t i = 0; i < environment_model_debug_info->obstacle_size();
           ++i) {
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

void SpatioTemporalPlanner::LogDebugInfo() {
#ifdef ENABLE_PROTO_LOG
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_spatio_temporal_union_plan_input()
      ->CopyFrom(spatio_temporal_union_plan_input_);
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_spatio_temporal_union_plan()
      ->CopyFrom(spatio_temporal_union_plan_);
#endif
  return;
}

void SpatioTemporalPlanner::UpdateIntersection() {
  const auto& tfl_decider = session_->mutable_planning_context()
                                    ->mutable_traffic_light_decider_output();
  const auto& reference_path = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  double init_vel = reference_path->get_frenet_ego_state().planning_init_point().v;
  std::vector<double> xp_vel{4.167, 16.667};
  std::vector<double> fp_length{15.0, 25.0};
  double min_virtual_length = interp(init_vel, xp_vel, fp_length);
  double preview_length = std::max(std::min(init_vel * 5.0, 90.0), 20.0);
  double virtual_length = 0.;
  double dist_to_virtual_start = 100.;
  bool is_in_virtual_area = reference_path->IsExistValidVirtualLaneAheadEgo(preview_length, min_virtual_length, virtual_length, dist_to_virtual_start);
  if (is_in_virtual_area) {
    virtual_area_count_ = 1;
  } else if (virtual_area_count_ > 0 && virtual_area_count_ < kVirtualAreaContinuousThreshold) {
    ++virtual_area_count_;
  } else {
    virtual_area_count_ = 0;
  }
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto intersection_state = virtual_lane_manager->GetIntersectionState();
  const double distance_to_stopline = virtual_lane_manager->GetEgoDistanceToStopline();
  const double distance_to_crosswalk = virtual_lane_manager->GetEgoDistanceToCrosswalk();
  bool current_intersection_state =
      (intersection_state == common::IntersectionState::IN_INTERSECTION ||
       distance_to_stopline <= kDistanceThresholdApproachToStopline) && (virtual_area_count_ > 0);
  bool is_small_intersection = false;
  // bool is_small_intersection = tfl_decider.is_small_front_intersection;
  // distance_to_crosswalk <= kDistanceThresholdApproachToCrosswalk;
  if (current_intersection_state) {
    intersection_count_ = kEgoInIntersectionCount;
  } else {
    intersection_count_ = std::max(intersection_count_ - 1, 0);
  }

  ego_in_intersection_state_ =
      intersection_count_ > 0 && !is_small_intersection;
  return;
}

}  // namespace planning
