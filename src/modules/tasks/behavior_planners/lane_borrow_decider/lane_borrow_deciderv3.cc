#include "lane_borrow_deciderv3.h"

#include <Eigen/src/Core/Matrix.h>
#include <math.h>

#include <algorithm>
#include <cmath>

#include "agent/agent_manager.h"
#include "basic_types.pb.h"
#include "behavior_planners/dp_path_decider/dp_road_graph.h"
#include "behavior_planners/lateral_offset_decider/lateral_offset_decider_utils.h"
#include "behavior_planners/traffic_light_decider/traffic_light_decider.h"
#include "common_c.h"
#include "config/basic_type.h"
#include "config/message_type.h"
#include "define/geometry.h"
#include "dp_road_graph.pb.h"
#include "environmental_model.h"
#include "frenet_obstacle.h"
#include "lane_borrow_decider.pb.h"
#include "lateral_obstacle.h"
#include "library/lc_idm_lib/include/basic_intelligent_driver_model.h"
#include "log.h"
#include "math/polygon2d.h"
#include "obstacle_manager.h"
#include "planning_context.h"
#include "planning_hmi_c.h"
#include "pose2d.h"
#include "session.h"
#include "task_interface/lane_borrow_decider_output.h"
#include "tracked_object.h"
#include "utils/cartesian_coordinate_system.h"
#include "utils/path_point.h"
#include "utils/pose2d_utils.h"

namespace {
constexpr double kMinDisToStopLine = 20.0;
constexpr double kMinDisToCrossWalk = 20.0;
constexpr double kMinDisToTrafficLight = 60.0;
constexpr double kInfDisToTrafficLight = 10000.0;
constexpr double kLatPassableBuffer = 0.8;
constexpr double kObsLatBuffer = 0.3;
constexpr double kObsSpeedRatio = 3.5;
constexpr double kForwardOtherObsDistance = 20.0;
constexpr double kObsLonDisBuffer = 2.0;
constexpr double kMaxCentricOffset = 0.75;  // 静止车绕行能力No增强
constexpr double kBackNeededDistance = 5.0;
constexpr double kPreCentricOffsetHigh = 0.75;  // CUTIN 标准仍然保守
constexpr double kPreCentricOffsetLow = 0.45;
constexpr double kStaticEdgeDistance = 0.25;
constexpr double kDynamicEdgeDistance = 0.3;
constexpr double kDynamicEdgeDistanceWithCrossing = 0.5;
constexpr double kMaxLateralRange = 5.0;
constexpr double kTotalLaneBorrowTime =
    8;  // 借道的时间阈值，如果借道时间过长，不触发借道
constexpr double kMaxLongitRange = 70.0;
constexpr double kMinLongitRange = 25.0;
constexpr double kTimeStep = 0.2;
constexpr double kOverTakeLonDisBuffer = 1.0;
constexpr double kOverTakeHysteresis = 3.0;
constexpr double kLeadClearCheckHorizon = 5.0;   // s
constexpr double kLeadClearAheadThreshold = 20;  // m
constexpr double kLaneLineSegmentLength = 3.0;
constexpr double kLaneBorrowBackNeededDistance = 10.0;
constexpr double kLatBufferToLaneBoundary = 0.1;
constexpr double kMaxNudgingSpeed = 4.2;  // 15 kph
// 5s预测轨迹内相对于中心线的最大允许横向位移，超过则认为存在cut_in/cut_out风险，不借道
constexpr double kMaxLateralMovementIn5sForBorrow =
    0.6;  // 低速障碍物5s内横向移动阈值
constexpr double kMaxRelativeHeadingForBorrow = 0.087;  // 约5度
constexpr double kLatOverlapBuffer = 0.3;
constexpr double kCutOutThresholdScale = 1.15;
constexpr double kCutOutHeadingScale = 1.15;
constexpr double kLaneBorrowThresholdScale = 1.15;
constexpr double kSpeedHysteresisScale = 1.33;
constexpr double kLonDistanceHysteresis = 2.0;   // m
constexpr double kLatDistanceHysteresis = 0.15;  // m
constexpr double kLatMovementHysteresisScale = 1.66;
constexpr double kHeadingHysteresisScale = 1.5;
constexpr double kLeadClearAheadHysteresis = 5.0;  // m
constexpr double kCentricDirectionHysteresisScale = 2.5;
constexpr double kCrossingDirectionHysteresisScale = 3.0;
constexpr double kChangeLaneLonDistanceThr = 40;
constexpr double kLaneBorrowMaxSpeed = 22.2;

};  // namespace

namespace planning {
namespace lane_borrow_deciderV3 {

bool LaneBorrowDecider::Execute() {
  const auto& state_machine = session_->environmental_model()
                                  .get_local_view()
                                  .function_state_machine_info;
  if (state_machine.current_state == iflyauto::FunctionalState_MRC) {
    spatio_temporal_planner_intersection_count_ = 0;
    virtual_area_count_ = 0;
    last_lane_borrow_failed_reason_ = NONE_FAILED_REASON;
    return true;
  }
  Update();
  path_decider_->LogDebugInfo();
  rule_path_decider_->LogDebugInfo();
  LogDebugInfo();
  SendHMIData();
  last_lane_borrow_failed_reason_ =
      lane_borrow_decider_output_.lane_borrow_failed_reason;
  return true;
}

void LaneBorrowDecider::Update() {
  if (!ProcessEnvInfos()) {
    lane_borrow_decider_output_.is_in_lane_borrow_status = false;
    static_blocked_obj_id_vec_.clear();
    nearest_no_borrow_obstacle_ = nullptr;
    lane_borrow_decider_output_.blocked_obs_id = static_blocked_obj_id_vec_;
    lane_borrow_decider_output_.borrow_direction = NO_BORROW;
    lane_borrow_decider_output_.borrow_direction_map.clear();
    lane_borrow_status_ = LaneBorrowStatus::kNoLaneBorrow;
    lane_borrow_decider_output_.lane_borrow_state = lane_borrow_status_;
    session_->mutable_planning_context()->mutable_lane_borrow_decider_output() =
        lane_borrow_decider_output_;  // 输出赋值
    obs_observe_frame_map_.clear();
    observe_frame_num_ = 0;
    lane_borrow_decider_output_.lat_flag_map.clear();
    dp_lat_decision_hysteresis_map_.clear();
    rule_path_decider_->ClearInfo();
    is_hold_reset_path_ = false;
    last_static_blocked_obj_id_vec_.clear();
    return;
  }

  switch (lane_borrow_status_) {
    case LaneBorrowStatus::kNoLaneBorrow: {
      if (CheckIfNoBorrowToLaneBorrowDriving()) {
        if (RunPathPlanning()) {
          lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowDriving;
          is_first_frame_to_lane_borrow_ = true;
        }
      }
      break;
    }

    case LaneBorrowStatus::kLaneBorrowDriving: {
      if (!CheckLaneBorrowCondition()) {
        lane_borrow_status_ = LaneBorrowStatus::kNoLaneBorrow;
      } else if (CheckIfLaneBorrowToLaneBorrowCrossing()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowCrossing;
      }
      break;
    }

    case LaneBorrowStatus::kLaneBorrowCrossing: {
      if (CheckIfLaneBorrowCrossingToNoBorrow()) {
        lane_borrow_status_ = LaneBorrowStatus::kNoLaneBorrow;
      } else if (IsSafeForBackOriginLane()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowBackOriginLane;
      }
      break;
    }

    case LaneBorrowStatus::kLaneBorrowBackOriginLane: {
      if (CheckIfBackOriginLaneToNoBorrow()) {
        lane_borrow_status_ = LaneBorrowStatus::kNoLaneBorrow;
      } else if (CheckIfBackOriginLaneToLaneBorrowDriving()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowDriving;
      } else if (CheckIfBackOriginLaneToLaneBorrowCrossing()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowCrossing;
      }
      break;
    }
  }
  last_static_blocked_obj_id_vec_ = static_blocked_obj_id_vec_;
  lane_borrow_decider_output_.lane_borrow_state = lane_borrow_status_;
  if (lane_borrow_status_ != LaneBorrowStatus::kNoLaneBorrow) {
    lane_borrow_decider_output_.is_in_lane_borrow_status = true;
    lane_borrow_decider_output_.blocked_obs_id = static_blocked_obj_id_vec_;
    if (lane_borrow_decider_output_.lane_borrow_failed_reason ==
            NEARBY_OBSTACLE_TOO_CLOSE ||
        lane_borrow_decider_output_.lane_borrow_failed_reason ==
            BACKWARD_OBSTACLE_TOO_CLOSE ||
        lane_borrow_decider_output_.lane_borrow_failed_reason ==
            AHEAD_COMING_OBS ||
        is_hold_reset_path_) {
      lane_borrow_decider_output_.lane_borrow_state = kLaneBorrowWaitting;
      const auto& vehicle_param =
          VehicleConfigurationContext::Instance()->get_vehicle_param();
      if (static_blocked_obstacles_.empty()) {
        UpdateBorrowDirectionMap();
        session_->mutable_planning_context()
            ->mutable_lane_borrow_decider_output() =
            lane_borrow_decider_output_;
        return;
      }
      double virtual_v = static_blocked_obstacles_[0]->velocity();
      auto borrow_id = static_blocked_obstacles_[0]->id();
      bool is_reverse = static_blocked_obstacles_[0]->obstacle()->is_reverse();
      if (static_blocked_obstacles_[0]->obstacle()->is_static()) {
        is_reverse = false;
      }
      virtual_v = std::max(0.0, virtual_v);
      double lateral_dist = current_lane_ptr_->width() * 0.5 -
                            vehicle_param.max_width * 0.5 -
                            kLatBufferToLaneBoundary;
      double inner_l =
          (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW)
              ? lateral_dist
              : -lateral_dist;
      if (config_.use_dp_path_planning) {
        path_decider_->AddLaneBorrowVirtualObstacle(
            inner_l, obs_start_s_, virtual_v, is_reverse, borrow_id);
        path_decider_->CartSpline(&lane_borrow_decider_output_);
      } else {
        rule_path_decider_->AddLaneBorrowVirtualObstacle(
            inner_l, obs_start_s_, virtual_v, is_reverse, borrow_id,
            is_hold_reset_path_);
        rule_path_decider_->CartSpline(&lane_borrow_decider_output_);
      }
      lane_borrow_status_ =
          LaneBorrowStatus::kLaneBorrowDriving;  // lock status
    } else {
      is_hold_reset_path_ = false;
      if (lane_borrow_decider_output_.lane_borrow_failed_reason !=
          CHANGE_TARGET_LANE) {
        lane_borrow_decider_output_.lane_borrow_failed_reason =
            NONE_FAILED_REASON;
      }
    }
  } else {
    is_hold_reset_path_ = false;
    path_decider_->ClearDPInfo();
    rule_path_decider_->ClearInfo();
    lane_borrow_decider_output_.is_in_lane_borrow_status = false;
    static_blocked_obj_id_vec_.clear();
    nearest_no_borrow_obstacle_ = nullptr;
    lane_borrow_decider_output_.blocked_obs_id = static_blocked_obj_id_vec_;
    lane_borrow_decider_output_.borrow_direction = NO_BORROW;
    // if(lane_borrow_decider_output_.lane_borrow_failed_reason !=
    // OBSERVE_TIME_CHECK_FAILED){
    //   ClearLaneBorrowStatus();
    // }
    if (IsNeedResetObserve(
            lane_borrow_decider_output_.lane_borrow_failed_reason)) {
      obs_observe_frame_map_.clear();
      observe_frame_num_ = 0;
    }
  }
  UpdateBorrowDirectionMap();
  session_->mutable_planning_context()->mutable_lane_borrow_decider_output() =
      lane_borrow_decider_output_;  // 输出赋值

  return;
}

bool LaneBorrowDecider::ProcessEnvInfos() {
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto& lat_obstacle_decision = session_->planning_context()
                                          .lateral_obstacle_decider_output()
                                          .lat_obstacle_decision;
  current_lane_ptr_ = virtual_lane_manager->get_current_lane();
  current_reference_path_ptr_ = current_lane_ptr_->get_reference_path();
  left_lane_ptr_ = virtual_lane_manager->get_left_lane();
  right_lane_ptr_ = virtual_lane_manager->get_right_lane();
  const auto& obstacle_map = current_reference_path_ptr_->get_obstacles_map();

  lane_borrow_decider_output_.lane_borrow_failed_reason =
      planning::LaneBorrowFailedReason::NONE_FAILED_REASON;
  const auto traffic_light_manager =
      session_->environmental_model().get_traffic_light_decision_manager();
  const auto all_traffic_lights = traffic_light_manager->GetTrafficLightsInfo();
  dis_to_traffic_lights_ = kInfDisToTrafficLight;
  for (int i = 0; i < all_traffic_lights.size(); i++) {
    if (all_traffic_lights[i].traffic_light_x > 0 &&
        all_traffic_lights[i].traffic_light_x < dis_to_traffic_lights_) {
      dis_to_traffic_lights_ = all_traffic_lights[i].traffic_light_x;
    }
  }
  auto lane_borrow_pb_info = DebugInfoManager::GetInstance()
                                 .GetDebugInfoPb()
                                 ->mutable_lane_borrow_decider_info();
  if (CheckSpatioTemporalPlanner()) {
    // 后续上游统一替代
    lane_borrow_decider_output_.lane_borrow_failed_reason = IN_SP;
    return false;
  }

  if (current_lane_ptr_ == nullptr || current_reference_path_ptr_ == nullptr) {
    ILOG_ERROR << "No current_lane_ptr_ or current_reference_path_ptr!";
    return false;
  };

  ego_speed_ = session_->environmental_model().get_ego_state_manager()->ego_v();
  ego_frenet_boundary_ = session_->environmental_model()
                             .get_reference_path_manager()
                             ->get_reference_path_by_current_lane()
                             ->get_ego_frenet_boundary();
  ego_sl_state_ = session_->environmental_model()
                      .get_reference_path_manager()
                      ->get_reference_path_by_current_lane()
                      ->get_frenet_ego_state();
  heading_angle_ =
      session_->environmental_model().get_ego_state_manager()->ego_pose().theta;
  ego_xy_ = session_->environmental_model().get_ego_state_manager()->ego_pose();
  lane_change_state_ = session_->planning_context()
                           .lane_change_decider_output()
                           .coarse_planning_info.target_state;
  const auto fix_lane_virtual_id = session_->planning_context()
                                       .lane_change_decider_output()
                                       .fix_lane_virtual_id;
  const auto& target_lane_nodes =
      session_->environmental_model().get_dynamic_world()->GetNodesByLaneId(
          fix_lane_virtual_id);
  double front_s = 150.0;
  front_id_ = 0;
  // 找到第一个横向非避让障碍物
  for (const auto* target_lane_node : target_lane_nodes) {
    auto agent_id = target_lane_node->node_agent_id();
    const auto lat_obs_iter = lat_obstacle_decision.find(agent_id);
    if (lat_obs_iter != lat_obstacle_decision.end() &&
        (lat_obs_iter->second != LatObstacleDecisionType::IGNORE &&
         lat_obs_iter->second != LatObstacleDecisionType::FOLLOW)) {
      // 找到第一个横向非避让障碍物
      continue;
    }
    auto it = obstacle_map.find(agent_id);
    if (it == obstacle_map.end()) {
      continue;
    }
    // double agent_s = target_lane_node->node_s();
    double agent_s = it->second->frenet_obstacle_boundary().s_start;
    if (agent_s < ego_sl_state_.boundary().s_end) {
      continue;
    }
    if (agent_s < front_s) {
      front_id_ = agent_id;
      front_s = agent_s;
    }
  }

  // for (const auto* target_lane_node : target_lane_nodes) {
  //   double agent_s = target_lane_node->node_s();
  //   if (agent_s < ego_sl_state_.s() + 1.0) {
  //     continue;
  //   }
  //   if (agent_s < front_s) {
  //     front_id_ = target_lane_node->node_agent_id();
  //     front_s = agent_s;
  //   }
  // }

  if (lane_change_state_ != kLaneKeeping) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = LANE_CHANGE_STATE;
    ILOG_INFO << "It has lane change state!";
    return false;
  }
  const bool dbw_status = session_->environmental_model().GetVehicleDbwStatus();
  if (!dbw_status) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = NOT_DBW_STATUS;
    return false;
  }
  const auto& function_info = session_->environmental_model().function_info();
  if (function_info.function_mode() != common::DrivingFunctionInfo::SCC) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = NO_LCC_MODE;
    return false;
  }
  intersection_state_ = virtual_lane_manager->GetIntersectionState();
  distance_to_stop_line_ = virtual_lane_manager->GetEgoDistanceToStopline();
  distance_to_cross_walk_ = virtual_lane_manager->GetEgoDistanceToCrosswalk();
  lane_borrow_pb_info->set_dis_to_traffic_lights(dis_to_traffic_lights_);
  lane_borrow_pb_info->set_distance_to_stop_line(distance_to_stop_line_);
  lane_borrow_pb_info->set_distance_to_cross_walk(distance_to_cross_walk_);
  if (intersection_state_ ==
          planning::common::IntersectionState::IN_INTERSECTION &&
      lane_borrow_status_ == kNoLaneBorrow) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = CLOSE_TO_JUNCTION;
    return false;
  }

  if (std::fabs(ego_speed_) > kLaneBorrowMaxSpeed) {  // 80 kph
    lane_borrow_decider_output_.lane_borrow_failed_reason = SPEED_TOO_HIGH;
    return false;
  }
  return true;
}

void LaneBorrowDecider::UpdateBorrowDirectionMap() {
  lane_borrow_decider_output_.borrow_direction_map.clear();
  if (lane_borrow_decider_output_.borrow_direction == NO_BORROW) {
    return;
  }
  const auto& dp_path = path_decider_->refined_paths();
  const auto& rule_path = rule_path_decider_->refined_paths();
  const auto& path = config_.use_dp_path_planning ? dp_path : rule_path;
  if (path.empty()) {
    return;
  }
  for (const auto& obs_id : static_blocked_obj_id_vec_) {
    // 找到对应障碍物的frenet boundary
    const FrenetObstacleBoundary* obs_sl_ptr = nullptr;
    for (const auto& obstacle : static_blocked_obstacles_) {
      if (obstacle->obstacle()->id() == obs_id) {
        obs_sl_ptr = &obstacle->frenet_obstacle_boundary();
        break;
      }
    }
    if (obs_sl_ptr == nullptr) {
      continue;
    }
    double obs_center_l = (obs_sl_ptr->l_start + obs_sl_ptr->l_end) / 2;
    double obs_center_s = (obs_sl_ptr->s_start + obs_sl_ptr->s_end) / 2;

    auto obs_it = std::lower_bound(
        path.begin(), path.end(), obs_center_s,
        [](const PathPoint& point, double s) { return point.s() < s; });

    double obs_dp_path_l = 0.0;
    bool obs_is_endpoint = false;
    if (obs_it == path.begin()) {
      obs_is_endpoint = true;
      obs_dp_path_l = path.front().l();
    } else if (obs_it == path.end() || obs_it == std::prev(path.end())) {
      obs_is_endpoint = true;
      obs_dp_path_l = path.back().l();
    }

    if (!obs_is_endpoint) {
      const auto& obs_prev_pt = *(obs_it - 1);
      const auto& obs_next_pt = *obs_it;
      if (std::abs(obs_prev_pt.s() - obs_center_s) <
          std::abs(obs_next_pt.s() - obs_center_s)) {
        obs_dp_path_l = obs_prev_pt.l();
      } else {
        obs_dp_path_l = obs_next_pt.l();
      }
    }

    if (obs_dp_path_l > obs_center_l) {
      lane_borrow_decider_output_.borrow_direction_map[obs_id] = LEFT_BORROW;
    } else if (obs_dp_path_l < obs_center_l) {
      lane_borrow_decider_output_.borrow_direction_map[obs_id] = RIGHT_BORROW;
    } else {
      lane_borrow_decider_output_.borrow_direction_map[obs_id] = NO_BORROW;
    }
  }
}

bool LaneBorrowDecider::RunPathPlanning(bool is_need_stabilize) {
  if (config_.use_dp_path_planning) {
    return RunPathPlanningBaseDP();
  } else {
    return RunPathPlanningBaseRules(is_need_stabilize);
  }
}

bool LaneBorrowDecider::RunPathPlanningBaseRules(bool is_need_stabilize) {
  // 基于选择的障碍物，基于规则给一条ref/path
  if (!rule_path_decider_->ProcessEnvInfos(&lane_borrow_decider_output_,
                                           lane_borrow_status_,
                                           static_blocked_obj_id_vec_)) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = CURRENT_LANE_LOSS;
    lane_borrow_decider_output_.is_in_lane_borrow_status = false;
    return false;
  }
  if (!rule_path_decider_->GenerateOriginRulePath(obs_direction_map_)) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        RULE_PATH_GENERATION_FAILED;
    lane_borrow_decider_output_.is_in_lane_borrow_status = false;
    return false;
  }
  if (is_need_stabilize) {
    rule_path_decider_->StabilizeRefinedPaths();
  }
  rule_path_decider_->CartSpline(&lane_borrow_decider_output_);
  // 延迟写入横向决策，校验通过后才 apply
  std::unordered_map<int32_t, LatObstacleDecisionType> pending_lat_decisions;
  std::shared_ptr<FrenetObstacle> nearest_static_non_borrow_obstacle = nullptr;
  std::shared_ptr<FrenetObstacle> nearest_dynamic_non_borrow_obstacle = nullptr;
  rule_path_decider_->UpdateObstacleLateralDecisionBaseRulePath(
      lane_borrow_decider_output_, nearest_static_non_borrow_obstacle,
      nearest_dynamic_non_borrow_obstacle, &pending_lat_decisions);
  // idm检测前方是否会被堵塞，动静态分开检测
  if (lane_borrow_status_ != kLaneBorrowCrossing) {
    if ((CheckLaneBorrowByTrajectory(nearest_static_non_borrow_obstacle))) {
      lane_borrow_decider_output_.lane_borrow_failed_reason =
          NO_LON_SPACE_TO_OVERTAKE_OBSTACLE;
      return false;
    }
  }
  // 校验通过，应用横向决策
  auto& lat_obstacle_decision = session_->mutable_planning_context()
                                    ->mutable_lateral_obstacle_decider_output()
                                    .lat_obstacle_decision;
  for (const auto& [obs_id, decision] : pending_lat_decisions) {
    lat_obstacle_decision[obs_id] = decision;
  }
  // rule_path_decider_->CartSpline(&lane_borrow_decider_output_);
  return true;
}

bool LaneBorrowDecider::RunPathPlanningBaseDP() {
  // path_decider_->Execute();
  if (!path_decider_->ProcessEnvInfos(&lane_borrow_decider_output_,
                                      lane_borrow_status_,
                                      static_blocked_obj_id_vec_)) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = CURRENT_LANE_LOSS;
    lane_borrow_decider_output_.is_in_lane_borrow_status = false;
    return false;
  }
  path_decider_->SetSampleParams(lane_borrow_status_);
  path_decider_->SetDPCostParams(lane_borrow_status_);
  path_decider_->SampleLanes(&lane_borrow_decider_output_);
  path_decider_->DPSearchPath(lane_borrow_status_);
  if (!path_decider_->FinedReferencePath() &&
      lane_borrow_status_ != LaneBorrowStatus::kNoLaneBorrow) {
    observe_path_frame_num_++;
    if (observe_path_frame_num_ < 5) {
      path_decider_->LastFramePath();
      // lane_borrow_decider_output_.lane_borrow_failed_reason =
      // TMP_DP_SEARCH_FAILED;
    } else {
      lane_borrow_decider_output_.lane_borrow_failed_reason =
          TRIGGER_BUT_DP_SEARCH_FAILED;
      lane_borrow_decider_output_.is_in_lane_borrow_status = false;
      return false;
    }
  } else if (!path_decider_->FinedReferencePath() &&
             lane_borrow_status_ == LaneBorrowStatus::kNoLaneBorrow) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        TRIGGER_BUT_DP_SEARCH_FAILED;
    lane_borrow_decider_output_.is_in_lane_borrow_status = false;
    return false;
  } else {
    observe_path_frame_num_ = 0;
  }
  path_decider_->CartSpline(&lane_borrow_decider_output_);
  UpdateObstacleLateralDecisionBaseDPPath();
  return true;
}

void LaneBorrowDecider::UpdateObstacleLateralDecisionBaseDPPath() {
  // 基于dp path做除借道障碍物之外动态障碍物的横向决策
  if (lane_borrow_decider_output_.dp_path_coord == nullptr ||
      lane_borrow_decider_output_.lane_borrow_failed_reason ==
          NEARBY_OBSTACLE_TOO_CLOSE ||
      lane_borrow_decider_output_.lane_borrow_failed_reason ==
          BACKWARD_OBSTACLE_TOO_CLOSE ||
      lane_borrow_decider_output_.lane_borrow_failed_reason ==
          AHEAD_COMING_OBS) {
    dp_lat_decision_hysteresis_map_.clear();
    return;
  }

  auto& lat_obstacle_decision = session_->mutable_planning_context()
                                    ->mutable_lateral_obstacle_decider_output()
                                    .lat_obstacle_decision;

  constexpr double kDefaultLaneWidth = 1.75;
  constexpr double kPredHorizon = 5.0;
  constexpr double kPredStep = 1.0;
  constexpr int kSwitchHoldFrames = 3;
  constexpr double kLatSafeEnterIgnoreMargin = 0.2;
  constexpr double kLatSafeReleaseIgnoreMargin = 0.2;
  constexpr double dynamic_base_lat_safe_distance = 1.0;
  constexpr double static_base_lat_safe_distance = 0.6;

  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  const auto& agent_mgr = session_->environmental_model().get_agent_manager();
  const double lon_min_s = ego_frenet_boundary_.s_start;
  const double lon_max_s = ego_frenet_boundary_.s_end + 80.0;

  // purge disappeared obstacles
  for (auto it = dp_lat_decision_hysteresis_map_.begin();
       it != dp_lat_decision_hysteresis_map_.end();) {
    const int32_t obs_id = it->first;
    if (std::find_if(obstacles.begin(), obstacles.end(),
                     [obs_id](const std::shared_ptr<FrenetObstacle>& o) {
                       return o != nullptr && o->obstacle()->id() == obs_id;
                     }) == obstacles.end()) {
      it = dp_lat_decision_hysteresis_map_.erase(it);
    } else {
      ++it;
    }
  }

  for (const auto& obstacle : obstacles) {
    const int32_t obs_id = obstacle->obstacle()->id();
    if (std::find(static_blocked_obj_id_vec_.begin(),
                  static_blocked_obj_id_vec_.end(),
                  obs_id) != static_blocked_obj_id_vec_.end()) {
      continue;  // skip lane-borrow blocked obstacles
    }

    const auto& agent = agent_mgr->GetAgent(obs_id);
    if (agent == nullptr) {
      continue;
    }
    if (!(agent->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    if (agent->agent_decision().agent_decision_type() ==
        agent::AgentDecisionType::IGNORE) {
      continue;
    }

    const auto& obs_sl = obstacle->frenet_obstacle_boundary();
    if (obs_sl.s_end < lon_min_s || obs_sl.s_start > lon_max_s) {
      continue;
    }
    double lat_safe_distance = obstacle->obstacle()->is_static()
                                   ? static_base_lat_safe_distance
                                   : dynamic_base_lat_safe_distance;
    if (obstacle->obstacle()->is_VRU() ||
        obstacle->obstacle()->is_oversize_vehicle()) {
      // 大车和VRU横向安全距离额外增加0.3m
      lat_safe_distance += 0.3;
    }
    // lat_safe_distance 距离滞回：
    // - 从避让(LEFT/RIGHT) ->
    // IGNORE：进入条件更严格（阈值更小），避免临界抖动进 IGNORE
    // - 从 IGNORE ->
    // 避让(LEFT/RIGHT)：释放条件更严格（阈值更大），避免临界抖动出 IGNORE
    const auto hist_it = dp_lat_decision_hysteresis_map_.find(obs_id);
    if (hist_it != dp_lat_decision_hysteresis_map_.end()) {
      const auto stable_decision = hist_it->second.first;
      if (stable_decision == LatObstacleDecisionType::IGNORE) {
        lat_safe_distance += kLatSafeReleaseIgnoreMargin;
      } else if (stable_decision == LatObstacleDecisionType::LEFT ||
                 stable_decision == LatObstacleDecisionType::RIGHT) {
        lat_safe_distance =
            std::max(0.0, lat_safe_distance - kLatSafeEnterIgnoreMargin);
      }
    }

    LatObstacleDecisionType candidate = LatObstacleDecisionType::IGNORE;
    const double obstacle_center_l = (obs_sl.l_start + obs_sl.l_end) / 2;
    if (obstacle_center_l > 0.0) {
      candidate = LatObstacleDecisionType::RIGHT;
    } else {
      candidate = LatObstacleDecisionType::LEFT;
    }
    for (double t = 0.0; t <= kPredHorizon + 1e-6; t += kPredStep) {
      const Box2d pred_box = PredictBoxPosition(agent, t);
      const auto corners = pred_box.GetAllCorners();

      FrenetObstacleBoundary dp_sl_bd;
      dp_sl_bd.s_start = std::numeric_limits<double>::max();
      dp_sl_bd.s_end = std::numeric_limits<double>::lowest();
      dp_sl_bd.l_start = std::numeric_limits<double>::max();
      dp_sl_bd.l_end = std::numeric_limits<double>::lowest();

      for (const auto& p : corners) {
        double s = 0.0, l = 0.0;
        if (!lane_borrow_decider_output_.dp_path_coord->XYToSL(p.x(), p.y(), &s,
                                                               &l)) {
          continue;
        }
        dp_sl_bd.s_start = std::min(dp_sl_bd.s_start, s);
        dp_sl_bd.s_end = std::max(dp_sl_bd.s_end, s);
        dp_sl_bd.l_start = std::min(dp_sl_bd.l_start, l);
        dp_sl_bd.l_end = std::max(dp_sl_bd.l_end, l);
      }

      const double min_lat_dist_to_path =
          (dp_sl_bd.l_start <= 0.0 && dp_sl_bd.l_end >= 0.0)
              ? 0.0
              : std::min(std::fabs(dp_sl_bd.l_start),
                         std::fabs(dp_sl_bd.l_end));
      if (min_lat_dist_to_path < lat_safe_distance) {
        candidate = LatObstacleDecisionType::IGNORE;
        break;
      }
    }

    // Apply internal hysteresis (do not rely on previous planning_context).
    auto& hist = dp_lat_decision_hysteresis_map_[obs_id];
    if (hist.second == 0) {
      hist.first = candidate;
      hist.second = 0;
    }

    LatObstacleDecisionType stable = hist.first;
    if (candidate == stable) {  // 连续相同决策，不改变计数
      hist.second = 0;
    } else {
      hist.second += 1;
      if (hist.second >= kSwitchHoldFrames) {
        hist.first = candidate;
        hist.second = 0;
        stable = candidate;
      }
    }

    lat_obstacle_decision[obs_id] = stable;
  }
}

bool LaneBorrowDecider::CheckBlockedBorrowObstaclesByTrajectory() {
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto& lat_obstacle_decision = session_->planning_context()
                                          .lateral_obstacle_decider_output()
                                          .lat_obstacle_decision;
  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  const auto& agent_mgr = session_->environmental_model().get_agent_manager();
  const double lane_width = current_lane_ptr_->width();
  double neighbor_left_width = 1.75;
  double neighbor_right_width = 1.75;
  const double current_left_lane_width = current_lane_ptr_->width() * 0.5;
  const double current_right_lane_width = current_lane_ptr_->width() * 0.5;
  double left_risk_bound = 0.;
  double right_risk_bound = 0.;
  if (lane_borrow_decider_output_.borrow_direction ==
      LEFT_BORROW) {  // change: bound just half lane
    const double neighbor_width =
        left_lane_ptr_->width(vehicle_param.front_edge_to_rear_axle);
    neighbor_left_width = neighbor_width * 0.5;
    neighbor_right_width = neighbor_width * 0.5;
    left_risk_bound = current_left_lane_width + neighbor_right_width;
    right_risk_bound = current_left_lane_width;
  } else if (lane_borrow_decider_output_.borrow_direction == RIGHT_BORROW) {
    const double neighbor_width =
        right_lane_ptr_->width(vehicle_param.front_edge_to_rear_axle);
    neighbor_left_width = neighbor_width * 0.5;
    neighbor_right_width = neighbor_width * 0.5;
    right_risk_bound = -current_right_lane_width - neighbor_left_width;
    left_risk_bound = -current_right_lane_width;
  }

  std::shared_ptr<FrenetObstacle> lead_nearest_follow_obstacle_static = nullptr;
  std::shared_ptr<FrenetObstacle> lead_nearest_follow_obstacle_dynamic =
      nullptr;
  double lead_nearest_s_start_static = std::numeric_limits<double>::max();
  double lead_nearest_s_start_dynamic = std::numeric_limits<double>::max();
  // 记录离纵向距离最近的非借道障碍物（区分动静态）
  for (const auto& obstacle : obstacles) {
    const auto& id = obstacle->obstacle()->id();
    const auto& agent = agent_mgr->GetAgent(id);
    const auto lat_obs_iter = lat_obstacle_decision.find(id);
    if (lat_obs_iter == lat_obstacle_decision.end()) {
      continue;
    }
    //  continue
    if (agent == nullptr) {
      continue;
    }
    if (agent->agent_decision().agent_decision_type() ==
        agent::AgentDecisionType::IGNORE) {  // ignore 忽略
      continue;
    }
    if (!(agent->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {  // 非视觉忽略
      continue;
    }
    // 过滤对向车
    if (obstacle->obstacle()->is_reverse()) {
      continue;
    }
    // 过滤相对速度差较高的车
    if (obstacle->frenet_relative_velocity_s() > kMaxNudgingSpeed) {
      continue;
    }
    if (obstacle->frenet_obstacle_boundary().s_start <=
            ego_frenet_boundary_.s_end ||
        obstacle->frenet_obstacle_boundary().s_start >=
            obs_end_s_ + kLaneBorrowBackNeededDistance) {
      // 在自车车头后面的障碍物过滤
      // 借道区域之外的障碍物障碍物过滤
      continue;
    }
    const bool is_borrow_obstacle =
        std::find(static_blocked_obj_id_vec_.begin(),
                  static_blocked_obj_id_vec_.end(),
                  id) != static_blocked_obj_id_vec_.end();
    if (is_borrow_obstacle) {
      continue;
    }
    const auto& obs_sl = obstacle->frenet_obstacle_boundary();
    if (obs_sl.l_start > left_risk_bound || obs_sl.l_end < right_risk_bound) {
      continue;
    }

    if (bypass_direction_ == LEFT_BORROW &&
        lat_obs_iter->second == LatObstacleDecisionType::LEFT) {
      // 向左借道，障碍物标签为LEFT的不应该选为跟车目前
      continue;
    } else if (bypass_direction_ == RIGHT_BORROW &&
               lat_obs_iter->second == LatObstacleDecisionType::RIGHT) {
      // 向右借道，障碍物标签为RIGHT的不应该选为跟车目标
      continue;
    }

    if (obstacle->obstacle()->is_static()) {
      if (obs_sl.s_start < lead_nearest_s_start_static) {
        lead_nearest_s_start_static = obs_sl.s_start;
        lead_nearest_follow_obstacle_static = obstacle;
      }
    } else {
      if (obs_sl.s_start < lead_nearest_s_start_dynamic) {
        lead_nearest_s_start_dynamic = obs_sl.s_start;
        lead_nearest_follow_obstacle_dynamic = obstacle;
      }
    }
  }

  if (nearest_no_borrow_obstacle_ != nullptr &&
      nearest_no_borrow_obstacle_->frenet_obstacle_boundary().s_start <
          obs_end_s_ + kLaneBorrowBackNeededDistance) {
    double no_borrow_s_start =
        nearest_no_borrow_obstacle_->frenet_obstacle_boundary().s_start;
    if (nearest_no_borrow_obstacle_->obstacle()->is_static()) {
      if (lead_nearest_follow_obstacle_static == nullptr ||
          no_borrow_s_start < lead_nearest_s_start_static) {
        lead_nearest_follow_obstacle_static = nearest_no_borrow_obstacle_;
        lead_nearest_s_start_static = no_borrow_s_start;
      }
    } else {
      if (lead_nearest_follow_obstacle_dynamic == nullptr ||
          no_borrow_s_start < lead_nearest_s_start_dynamic) {
        lead_nearest_follow_obstacle_dynamic = nearest_no_borrow_obstacle_;
        lead_nearest_s_start_dynamic = no_borrow_s_start;
      }
    }
  }
  return CheckLaneBorrowByTrajectory(lead_nearest_follow_obstacle_dynamic) ||
         CheckLaneBorrowByTrajectory(lead_nearest_follow_obstacle_static);
}

bool LaneBorrowDecider::CheckLaneBorrowByTrajectory(
    const std::shared_ptr<FrenetObstacle>& lead_nearest_follow_obstacle) {
  if (lead_nearest_follow_obstacle == nullptr) {
    return false;
  }
  const auto& agent_mgr = session_->environmental_model().get_agent_manager();
  const auto& agent =
      agent_mgr->GetAgent(lead_nearest_follow_obstacle->obstacle()->id());
  if (agent == nullptr) {
    return false;
  }
  const auto& lead_trajectories = agent->trajectories_used_by_st_graph();
  const trajectory::Trajectory* lead_trajectory_ptr = nullptr;
  if (!lead_trajectories.empty() && !lead_trajectories[0].empty()) {
    lead_trajectory_ptr = &lead_trajectories[0];
  }

  // 依据lead_trajectory 和 kTotalLaneBorrowTime
  // 以及idm生成自车轨迹，时间间隔是0.2s
  const int total_steps = static_cast<int>(kTotalLaneBorrowTime / kTimeStep);
  const auto& frenet_coord = current_reference_path_ptr_->get_frenet_coord();

  // IDM 参数
  BasicIntelligentDriverModel idm;
  BasicIntelligentDriverModel::ModelParam idm_param;
  idm_param.kDesiredVelocity =
      session_->environmental_model().get_ego_state_manager()->ego_v_cruise();
  idm_param.kVehicleLength =
      VehicleConfigurationContext::Instance()->get_vehicle_param().length;

  // 自车初始状态（Frenet）
  double ego_s = ego_frenet_boundary_.s_end;  // 自车车头 s
  double ego_vel = ego_speed_;

  // lead 轨迹时间范围（若无预测轨迹，则退化为当前位置“静态 lead”）
  const double lead_t0 = (lead_trajectory_ptr != nullptr)
                             ? lead_trajectory_ptr->front().absolute_time()
                             : 0.0;
  const double lead_t_end = (lead_trajectory_ptr != nullptr)
                                ? lead_trajectory_ptr->back().absolute_time()
                                : 0.0;

  // lead 末端状态，用于超出预测范围时外推
  double last_lead_s = 0.0, last_lead_l = 0.0;
  double last_lead_vel = 0.0;
  if (lead_trajectory_ptr != nullptr) {
    frenet_coord->XYToSL(lead_trajectory_ptr->back().x(),
                         lead_trajectory_ptr->back().y(), &last_lead_s,
                         &last_lead_l);
    last_lead_vel = lead_trajectory_ptr->back().vel();
  } else {
    frenet_coord->XYToSL(agent->x(), agent->y(), &last_lead_s, &last_lead_l);
    last_lead_vel = 0.0;
  }

  // IDM 递推生成自车轨迹
  struct EgoTrajectoryPoint {
    double time;
    double s;
    double vel;
  };
  std::vector<EgoTrajectoryPoint> ego_trajectory;
  ego_trajectory.reserve(total_steps + 1);
  ego_trajectory.push_back({0.0, ego_s, ego_vel});

  for (int i = 1; i <= total_steps; ++i) {
    const double t = i * kTimeStep;
    const double query_time = lead_t0 + t;

    // 获取 lead 在当前时刻的 Frenet s 和速度
    double lead_s = 0.0;
    double lead_vel = 0.0;
    if (lead_trajectory_ptr == nullptr) {
      // No prediction: treat lead as static at current position.
      lead_s = last_lead_s;
      lead_vel = 0.0;
    } else if (query_time <= lead_t_end) {
      const auto lead_point = lead_trajectory_ptr->Evaluate(query_time);
      double lead_l = 0.0;
      frenet_coord->XYToSL(lead_point.x(), lead_point.y(), &lead_s, &lead_l);
      lead_vel = lead_point.vel();
    } else {
      // 超出预测范围，按末端速度匀速外推
      const double extra_time = query_time - lead_t_end;
      lead_s = last_lead_s + last_lead_vel * extra_time;
      lead_vel = last_lead_vel;
    }

    // IDM 计算期望加速度
    BasicIntelligentDriverModel::ModelState state(ego_s, ego_vel, lead_s,
                                                  lead_vel);
    double acc = 0.0;
    idm.GetIIdmDesiredAcceleration(idm_param, state, &acc);

    // 更新自车状态
    ego_s += ego_vel * kTimeStep + 0.5 * acc * kTimeStep * kTimeStep;
    ego_vel = std::max(0.0, ego_vel + acc * kTimeStep);

    ego_trajectory.push_back({t, ego_s, ego_vel});
  }

  // 检查ego_trajectory和static_blocked_obj_id_vec_中的预测轨迹，判断自车能否在kTotalLaneBorrowTime内超越借道障碍物
  // static_blocked_obj_id_vec_中轨迹时间如果小于kTotalLaneBorrowTime，则后端补充轨迹时间到kTotalLaneBorrowTime
  const double lead_half_length = agent->length() * 0.5;
  double lead_rear_s_at_horizon = 0.0;
  const bool has_lead_clear_check = (lead_trajectory_ptr != nullptr);
  if (has_lead_clear_check) {
    double lead_s_at_horizon = 0.0;
    double tmp_l_lead = 0.0;
    const double query_lead_time = lead_t0 + kLeadClearCheckHorizon;
    if (query_lead_time <= lead_t_end) {
      const auto lead_pt = lead_trajectory_ptr->Evaluate(query_lead_time);
      frenet_coord->XYToSL(lead_pt.x(), lead_pt.y(), &lead_s_at_horizon,
                           &tmp_l_lead);
    } else {
      const double extra_time = query_lead_time - lead_t_end;
      lead_s_at_horizon = last_lead_s + last_lead_vel * extra_time;
    }
    lead_rear_s_at_horizon = lead_s_at_horizon - lead_half_length;
  }

  std::vector<int> not_overtaken_obs_ids;
  for (const auto& obs_id : static_blocked_obj_id_vec_) {
    const auto& agent = agent_mgr->GetAgent(obs_id);
    if (agent == nullptr) {
      continue;
    }
    const double obs_half_length = agent->length() / 2.0;
    const auto& obs_trajectories = agent->trajectories_used_by_st_graph();
    const bool has_traj =
        !obs_trajectories.empty() && !obs_trajectories[0].empty();

    double obs_t0 = 0.0, obs_t_end = 0.0;
    double last_obs_s = 0.0, last_obs_vel = 0.0;

    if (has_traj) {
      const auto& obs_traj = obs_trajectories[0];
      obs_t0 = obs_traj.front().absolute_time();
      obs_t_end = obs_traj.back().absolute_time();
      double tmp_l = 0.0;
      frenet_coord->XYToSL(obs_traj.back().x(), obs_traj.back().y(),
                           &last_obs_s, &tmp_l);
      last_obs_vel = obs_traj.back().vel();
    } else {
      // 无轨迹，使用当前位置，视为静止
      double tmp_l = 0.0;
      frenet_coord->XYToSL(agent->x(), agent->y(), &last_obs_s, &tmp_l);
      last_obs_vel = 0.0;
    }

    // 比较末端时刻的s，判断是否超越
    const auto& ego_end = ego_trajectory.back();
    double obs_s = 0.0;
    if (has_traj) {
      const auto& obs_traj = obs_trajectories[0];
      const double query_time = obs_t0 + ego_end.time;
      if (query_time <= obs_t_end) {
        const auto obs_point = obs_traj.Evaluate(query_time);
        double tmp_l = 0.0;
        frenet_coord->XYToSL(obs_point.x(), obs_point.y(), &obs_s, &tmp_l);
      } else {
        // 超出预测范围，按末端速度匀速外推
        const double extra_time = query_time - obs_t_end;
        obs_s = last_obs_s + last_obs_vel * extra_time;
      }
    } else {
      obs_s = last_obs_s;
    }

    const bool was_successful =
        std::find(last_static_blocked_obj_id_vec_.begin(),
                  last_static_blocked_obj_id_vec_.end(),
                  obs_id) != last_static_blocked_obj_id_vec_.end();
    // 5s 时前车相对借道障碍物已拉开足够纵向距离：不纳入
    // not_overtaken（不从借道列表剔除）
    bool lead_far_ahead_of_borrow_at_horizon = false;
    if (has_lead_clear_check) {
      double obs_s_at_t = 0.0;
      double tmp_l_obs = 0.0;
      const double query_obs_time = obs_t0 + kLeadClearCheckHorizon;
      if (has_traj) {
        const auto& obs_traj = obs_trajectories[0];
        if (query_obs_time <= obs_t_end) {
          const auto obs_pt = obs_traj.Evaluate(query_obs_time);
          frenet_coord->XYToSL(obs_pt.x(), obs_pt.y(), &obs_s_at_t, &tmp_l_obs);
        } else {
          const double extra_time = query_obs_time - obs_t_end;
          obs_s_at_t = last_obs_s + last_obs_vel * extra_time;
        }
      } else {
        obs_s_at_t = last_obs_s;
      }

      const double obs_front_s = obs_s_at_t + obs_half_length;
      const double lon_distance_thr =
          was_successful ? kLeadClearAheadThreshold - kLeadClearAheadHysteresis
                         : kLeadClearAheadThreshold;
      if (lead_rear_s_at_horizon > obs_front_s + lon_distance_thr) {
        lead_far_ahead_of_borrow_at_horizon = true;
      }
    }

    if (lead_far_ahead_of_borrow_at_horizon) {
      continue;
    }
    // 末端时刻自车后端未超越障碍物前端 + 冗余距离，上一帧成功则加滞回
    const double hysteresis = was_successful ? kOverTakeHysteresis : 0.0;
    if (ego_end.s - idm_param.kVehicleLength <
        obs_s + obs_half_length + kOverTakeLonDisBuffer - hysteresis) {
      not_overtaken_obs_ids.emplace_back(obs_id);
    }
  }

  // 从static_blocked_obj_id_vec_中移除不能借道的障碍物
  for (const auto& id : not_overtaken_obs_ids) {
    static_blocked_obj_id_vec_.erase(
        std::remove(static_blocked_obj_id_vec_.begin(),
                    static_blocked_obj_id_vec_.end(), id),
        static_blocked_obj_id_vec_.end());
  }

  if (static_blocked_obj_id_vec_.empty()) {
    return true;
  }

  static_blocked_obstacles_.erase(
      std::remove_if(static_blocked_obstacles_.begin(),
                     static_blocked_obstacles_.end(),
                     [this](const std::shared_ptr<FrenetObstacle>& obstacle) {
                       return std::find(static_blocked_obj_id_vec_.begin(),
                                        static_blocked_obj_id_vec_.end(),
                                        obstacle->obstacle()->id()) ==
                              static_blocked_obj_id_vec_.end();
                     }),
      static_blocked_obstacles_.end());
  return false;
}

// v2   0-1
bool LaneBorrowDecider::CheckIfNoBorrowToLaneBorrowDriving() {
  if (!CheckLaneBorrowCondition()) {
    return false;
  }
  return true;
}

void LaneBorrowDecider::ClearLaneBorrowStatus() {
  observe_path_frame_num_ = 0;
  observe_frame_num_ = 0;
  obs_observe_frame_map_.clear();
  left_borrow_ = false;
  right_borrow_ = false;
  obs_direction_map_.clear();
  dp_lat_decision_hysteresis_map_.clear();
  nearest_no_borrow_obstacle_ = nullptr;
}

// v2 连续绕行反向   1-2
bool LaneBorrowDecider::CheckIfLaneBorrowToLaneBorrowCrossing() {
  if (CrossingPositionJudgment()) {
    return true;
  } else {
    return false;
  }
}

// v2 2-0
bool LaneBorrowDecider::CheckIfLaneBorrowCrossingToNoBorrow() {
  if (!CheckLaneBorrowCondition()) {  // 1-0
    return true;
  }
  return false;
}

// v2 2-3
bool LaneBorrowDecider::IsSafeForBackOriginLane() {
  // 静态区域最后障碍物的s和自车当前是s判断，如果超过阈值则说明自车正在回自车道
  if (static_blocked_obstacles_.empty()) {
    return true;
  }
  const auto& last_obstacle = static_blocked_obstacles_.back();
  const auto& stastic_last_obs = last_obstacle->frenet_obstacle_boundary();
  const auto& center_obs_s =
      (stastic_last_obs.s_start + stastic_last_obs.s_end) / 2;
  if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW) {
    if (ego_frenet_boundary_.s_start > stastic_last_obs.s_end &&
        std::abs(ego_frenet_boundary_.l_start - stastic_last_obs.l_end) < 2.0) {
      return true;
    }
  } else if (lane_borrow_decider_output_.borrow_direction == RIGHT_BORROW) {
    if (ego_frenet_boundary_.s_start > stastic_last_obs.s_end &&
        std::abs(ego_frenet_boundary_.l_end - stastic_last_obs.l_start) < 2.0) {
      return true;
    }
  }
  return false;
}

// v2  3-0
bool LaneBorrowDecider::CheckIfBackOriginLaneToNoBorrow() {
  if (config_.use_dp_path_planning && !RunPathPlanning()) {
    return true;
  }

  double left_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_start) * 0.5;
  double right_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_start) * 0.5;

  if (ego_frenet_boundary_.l_end < left_width &&
      ego_frenet_boundary_.l_start > -right_width) {
    ClearLaneBorrowStatus();
    return true;
  } else {
    return false;
  }
}

// v2  3-1
bool LaneBorrowDecider::CheckIfBackOriginLaneToLaneBorrowDriving() {
  // 返回跳转执行，需要确定一定能执行借道
  // const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  // const double left_width =
  //     current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  // const double right_width =
  //     current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  // const auto& lat_obstacle_decision = session_->planning_context()
  //                                         .lateral_obstacle_decider_output()
  //                                         .lat_obstacle_decision;
  // for (const auto& obstacle : obstacles) {
  //   const auto& id = obstacle->obstacle()->id();
  //   const auto& obs_type = obstacle->obstacle()->type();
  //   if (!obstacle->b_frenet_valid()) {
  //     continue;
  //   }

  //   // if (obs_type == iflyauto::ObjectType::OBJECT_TYPE_PEDESTRIAN) {
  //   //   continue;
  //   // }

  //   if (!(obstacle->obstacle()->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
  //     continue;
  //   }

  //   if (obstacle->frenet_obstacle_boundary().s_start >
  //       ego_frenet_boundary_.s_end) {
  //     const auto lat_obs_iter = lat_obstacle_decision.find(id);
  //     if (lat_obs_iter != lat_obstacle_decision.end() &&
  //         (lat_obs_iter->second != LatObstacleDecisionType::IGNORE &&
  //          lat_obs_iter->second != LatObstacleDecisionType::FOLLOW)) {
  //       continue;
  //     }
  //   }

  //   auto it = std::find(static_blocked_obj_id_vec_.begin(),
  //                       static_blocked_obj_id_vec_.end(), id);

  //   if (it != static_blocked_obj_id_vec_.end()) {
  //     continue;
  //   }
  //   const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
  //   if (frenet_obstacle_sl.s_start >
  //       ego_frenet_boundary_.s_end + kForwardOtherObsDistance) {
  //     continue;
  //   }
  //   if (frenet_obstacle_sl.l_start > left_width ||
  //       frenet_obstacle_sl.l_end < -right_width) {
  //     continue;
  //   }

  //   const double obs_v = obstacle->frenet_velocity_s();
  //   if (frenet_obstacle_sl.s_start > ego_frenet_boundary_.s_end) {
  //     // if (!obstacle->obstacle()->is_static()) {    //dp obs_speed selset
  //     //   continue;
  //     // }
  //     if (obs_v > kMaxNudgingSpeed) {  // dp  concern obs_v <     15kph
  //       continue;
  //     }

  //   } else {
  //     if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW) {
  //       if (frenet_obstacle_sl.l_end > ego_frenet_boundary_.l_start) {
  //         continue;
  //       }
  //       if (frenet_obstacle_sl.l_end < -right_width &&
  //           obstacle->obstacle()->is_static()) {
  //         continue;
  //       }
  //       if (frenet_obstacle_sl.l_end + kLatPassableBuffer < -right_width) {
  //         continue;
  //       }
  //       if (frenet_obstacle_sl.s_end + obs_v * kObsSpeedRatio <
  //           ego_frenet_boundary_.s_start) {
  //         continue;
  //       }

  //     } else {
  //       if (frenet_obstacle_sl.l_start < ego_frenet_boundary_.l_end) {
  //         continue;
  //       }
  //       if (frenet_obstacle_sl.l_start > left_width &&
  //           obstacle->obstacle()->is_static()) {
  //         continue;
  //       }
  //       if (frenet_obstacle_sl.l_start - kLatPassableBuffer > left_width) {
  //         continue;
  //       }
  //       if (frenet_obstacle_sl.s_end + obs_v * kObsSpeedRatio <
  //           ego_frenet_boundary_.s_start) {
  //         continue;
  //       }
  //     }
  //   }
  //   return true;
  // }
  // return false;
  if (!CheckLaneBorrowCondition()) {
    // 如果借道障碍物是空的，则按照自车当前位置和中心线的,以五次多项式生成一条返回中心的path
    if (static_blocked_obj_id_vec_.empty()) {
      if (!rule_path_decider_->ProcessEnvInfos(&lane_borrow_decider_output_,
                                               lane_borrow_status_,
                                               static_blocked_obj_id_vec_)) {
        return false;
      }
      if (!rule_path_decider_->GenerateBackOriginLaneRulePath()) {
        return false;
      }
      rule_path_decider_->CartSpline(&lane_borrow_decider_output_);
    }
    return false;
  }
  return true;
}

// v2  3-2
bool LaneBorrowDecider::CheckIfBackOriginLaneToLaneBorrowCrossing() {
  if (CrossingPositionJudgment()) {
    if (CheckIfBackOriginLaneToLaneBorrowDriving()) {
      return true;
    }
  }
  return false;
}

// v2
bool LaneBorrowDecider::CheckLaneBorrowCondition() {
  lane_borrow_decider_output_.is_change_target_lane = false;

  UpdateJunctionInfo();

  if (!SelectStaticBlockingObstcales()) {
    return false;
  }

  if (!UpdateLaneBorrowDirection()) {
    return false;
  }

  if (!UpdateDynamicBlockingObstacles()) {
    return false;
  }

  if (!ObstacleDecision()) {
    return false;
  }

  if (!EnoughSafetyDistance()) {
    // 近距离抑制借道
    return false;
  }

  double first_obs_end =
      static_blocked_obstacles_[0]->frenet_obstacle_boundary().s_end;
  if (std::fabs(distance_to_stop_line_ - (first_obs_end - ego_sl_state_.s())) <
      15.0) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = CLOSE_TO_JUNCTION;
    return false;
  }
  if (lane_borrow_status_ == kNoLaneBorrow && (!is_facility_) &&
      (left_lane_boundary_type_ == iflyauto::LaneBoundaryType_MARKING_SOLID ||
       right_lane_boundary_type_ == iflyauto::LaneBoundaryType_MARKING_SOLID)) {
    if ((distance_to_cross_walk_ < kMinDisToCrossWalk &&
         distance_to_cross_walk_ > 0.0) ||
        (distance_to_stop_line_ < kMinDisToStopLine &&
         distance_to_stop_line_ > 0.0) ||
        (dis_to_traffic_lights_ < kMinDisToTrafficLight &&
         dis_to_traffic_lights_ > 0.0) ||
        (std::fabs(distance_to_stop_line_ -
                   (first_obs_end - ego_sl_state_.s())) < 20.0) ||
        (intersection_state_ ==
         planning::common::IntersectionState::APPROACH_INTERSECTION)) {
      ILOG_DEBUG << "Ego car is near junction";
      lane_borrow_decider_output_.lane_borrow_failed_reason = CLOSE_TO_JUNCTION;
      return false;
    }
  }

  if (lane_borrow_status_ != kNoLaneBorrow) {
    if (!CheckLaneBorrowDircetion()) {
      return false;
    }
  } else {
    lane_borrow_decider_output_.borrow_direction = bypass_direction_;
  }

  // 虚拟车道抑制借道检查
  if (!CheckVirtualLaneSuppressBorrow()) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        VIRTUAL_LANE_SUPPRESS;
    return false;
  }

  // get blocking obs
  // SendObserveToLatFlag();
  observe_frame_num_++;  // 整体计数
  if (observe_frame_num_ < config_.observe_frames) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        OBSERVE_TIME_CHECK_FAILED;
    return false;  // after 8 11 22
  }

  // // 每个障碍物独立观测，更新计数并过滤未满足观测帧数的障碍物
  // if (!CheckObserveTime()) {
  //   lane_borrow_decider_output_.lane_borrow_failed_reason =
  //       OBSERVE_TIME_CHECK_FAILED;
  //   return false;
  // }

  if (lane_borrow_status_ == kNoLaneBorrow) {
    if (!CheckLeadObs()) {
      return false;
    }
  }
  if (lane_borrow_status_ != kLaneBorrowCrossing &&
      lane_borrow_status_ != kLaneBorrowBackOriginLane) {
    if (CheckBlockedBorrowObstaclesByTrajectory()) {
      lane_borrow_decider_output_.lane_borrow_failed_reason =
          NO_LON_SPACE_TO_OVERTAKE_OBSTACLE;
      return false;
    }
  }

  if (lane_borrow_status_ != kLaneBorrowCrossing &&
      lane_borrow_status_ != kLaneBorrowBackOriginLane) {
    CheckBackWardObs();
  }

  if (lane_borrow_status_ == LaneBorrowStatus::kLaneBorrowBackOriginLane) {
    // 借道返回时，如果借道障碍物与上一帧一样，则说明借道障碍物没有变化，则不跳转为借道执行
    auto normalize_ids = [](const std::vector<int>& ids) {
      std::vector<int> out = ids;
      std::sort(out.begin(), out.end());
      out.erase(std::unique(out.begin(), out.end()), out.end());
      return out;
    };
    if (normalize_ids(lane_borrow_decider_output_.blocked_obs_id) ==
        normalize_ids(static_blocked_obj_id_vec_)) {
      if (!RunPathPlanning()) {
        return false;
      }
      return false;
    }
  }

  if (lane_borrow_status_ != kNoLaneBorrow) {
    if (1) {
      bool is_back_origin_lane_to_lane_borrow_driving =
          lane_borrow_status_ == LaneBorrowStatus::kLaneBorrowBackOriginLane;
      if (!RunPathPlanning(!is_back_origin_lane_to_lane_borrow_driving)) {
        return false;
      }
    } else {
      is_first_frame_to_lane_borrow_ = false;
    }
  }

  bool is_change_lane = IfChangeTargetLane();
  if (lane_borrow_status_ == kLaneBorrowCrossing && is_change_lane) {
    // 借道应该一直维持到选道结束，不能强行终止借道进行选道，选道不一定100%选！
    lane_borrow_decider_output_.is_change_target_lane = true;
    lane_borrow_decider_output_.lane_borrow_failed_reason = CHANGE_TARGET_LANE;
    // return false;
  } else {
    lane_borrow_decider_output_.is_change_target_lane = false;
  }

  return true;
}

// v2
void LaneBorrowDecider::UpdateJunctionInfo() {
  forward_solid_start_dis_ = std::numeric_limits<double>::max();
  forward_solid_end_s_ = std::numeric_limits<double>::max();
  if (current_lane_ptr_->lane_points().empty()) {
    return;
  }

  const auto& current_lane_points = current_lane_ptr_->lane_points();
  bool found_start = false;

  for (size_t i = 0; i < current_lane_points.size(); ++i) {
    const auto& lane_point = current_lane_points[i];
    // fine start point
    if (!found_start &&
        !IsLaneTypeDashedOrMixed(lane_point.left_lane_border_type) &&
        !IsLaneTypeDashedOrMixed(lane_point.right_lane_border_type) &&
        lane_point.s > ego_frenet_boundary_.s_start) {
      forward_solid_start_dis_ = lane_point.s - ego_frenet_boundary_.s_start;
      found_start = true;
    }

    if (found_start &&
        (IsLaneTypeDashedOrMixed(lane_point.left_lane_border_type) ||
         IsLaneTypeDashedOrMixed(lane_point.right_lane_border_type)) &&
        lane_point.s > ego_frenet_boundary_.s_start) {
      forward_solid_end_s_ =
          lane_point.s - ego_frenet_boundary_.s_start;  // bounding
      break;
    }
  }

  auto lane_borrow_pb_info = DebugInfoManager::GetInstance()
                                 .GetDebugInfoPb()
                                 ->mutable_lane_borrow_decider_info();
  lane_borrow_pb_info->set_start_solid_lane_dis(forward_solid_start_dis_);
  lane_borrow_pb_info->set_end_solid_lane_dis(forward_solid_end_s_);
}

// Determine the type of lane marking.
bool LaneBorrowDecider::IsLaneTypeDashedOrMixed(
    const iflyauto::LaneBoundaryType& type) {
  return type == iflyauto::LaneBoundaryType_MARKING_DASHED ||
         type == iflyauto::LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED ||
         type == iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED ||
         type == iflyauto::LaneBoundaryType_MARKING_VIRTUAL;
}

bool LaneBorrowDecider::UpdateLaneBorrowDirection() {
  left_borrow_ = true;
  right_borrow_ = true;

  // double lane_line_length = 0.0;
  // const auto& left_lane_boundarys =
  // current_lane_ptr_->get_left_lane_boundary(); const auto&
  // right_lane_boundarys =
  //     current_lane_ptr_->get_right_lane_boundary();
  iflyauto::LaneBoundaryType left_lane_boundary_type;
  iflyauto::LaneBoundaryType right_lane_boundary_type;

  // const auto& vehicle_param =
  //     VehicleConfigurationContext::Instance()->get_vehicle_param();
  // # Accumulate lane segment lengths.
  // Record current segment type and break loop when exceeding vehicle
  // wheelbase.
  // for (int i = 0; i < left_lane_boundarys.type_segments_size; i++) {
  //   lane_line_length += left_lane_boundarys.type_segments[i].length;
  //   if (lane_line_length > ego_frenet_boundary_.s_end) {
  //     left_lane_boundary_type = left_lane_boundarys.type_segments[i].type;
  //     break;
  //   }
  // }
  // lane_line_length = 0.0;
  // for (int i = 0; i < right_lane_boundarys.type_segments_size; i++) {
  //   lane_line_length += right_lane_boundarys.type_segments[i].length;
  //   if (lane_line_length > ego_frenet_boundary_.s_end) {
  //     right_lane_boundary_type = right_lane_boundarys.type_segments[i].type;
  //     break;
  //   }
  // }
  // change
  const auto& current_lane_points = current_lane_ptr_->lane_points();
  bool found_start = false;
  for (size_t i = 0; i < current_lane_points.size(); ++i) {
    const auto& lane_point = current_lane_points[i];
    // fine start point
    if (lane_point.s > ego_sl_state_.s()) {
      left_lane_boundary_type_ = lane_point.left_lane_border_type;
      right_lane_boundary_type_ = lane_point.right_lane_border_type;
      break;
    }
  }
  lane_borrow_decider_output_.is_right_solid =
      (right_lane_boundary_type_ == iflyauto::LaneBoundaryType_MARKING_SOLID)
          ? true
          : false;
  lane_borrow_decider_output_.is_left_solid =
      (left_lane_boundary_type_ == iflyauto::LaneBoundaryType_MARKING_SOLID)
          ? true
          : false;
  // If the lane marking is not left dashed/right solid or double dashed, return
  // False.
  // if is_facility_ or in intersection ignore lane type
  if (left_lane_ptr_ == nullptr) {
    left_borrow_ = false;
  }

  // todo: if left lane is reverse, then left_boorow is false
  // if (!is_facility_ &&
  //     (intersection_state_ !=
  //      planning::common::IntersectionState::IN_INTERSECTION) &&
  //     right_lane_boundary_type != iflyauto::LaneBoundaryType_MARKING_DASHED
  //     && right_lane_boundary_type !=
  //         iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED &&
  //     right_lane_boundary_type != iflyauto::LaneBoundaryType_MARKING_VIRTUAL)
  //     {
  //   right_borrow_ = false;
  // }
  if (right_lane_ptr_ == nullptr) {
    right_borrow_ = false;
  }

  if (!left_borrow_ && !right_borrow_) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        LANE_TYPE_CHECK_FAILED;
    return false;
  }
  return true;
}

// v2
bool LaneBorrowDecider::SelectStaticBlockingObstcales() {
  const double forward_obs_s =
      std::fmin(current_reference_path_ptr_->get_frenet_coord()->Length(),
                ego_frenet_boundary_.s_end + config_.max_concern_obs_distance);
  double left_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  double right_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;

  obs_left_l_ = -left_width;
  obs_right_l_ = right_width;
  obs_start_s_ = forward_obs_s;
  obs_end_s_ = 0.0;

  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  static_blocked_obstacles_.clear();
  static_blocked_obj_id_vec_.clear();
  nearest_no_borrow_obstacle_ = nullptr;
  const auto& lat_obstacle_decision = session_->planning_context()
                                          .lateral_obstacle_decider_output()
                                          .lat_obstacle_decision;
  const auto& lat_obstacle_position = session_->planning_context()
                                          .lateral_obstacle_decider_output()
                                          .lateral_obstacle_history_info;
  for (const auto& obstacle : obstacles) {
    const auto& id = obstacle->obstacle()->id();
    const auto& obs_type = obstacle->obstacle()->type();
    if (!obstacle->b_frenet_valid()) {
      continue;
    }
    // if (obs_type == iflyauto::ObjectType::OBJECT_TYPE_PEDESTRIAN) {
    //   continue;
    // }
    if (!(obstacle->obstacle()->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    const bool was_successful =
        std::find(last_static_blocked_obj_id_vec_.begin(),
                  last_static_blocked_obj_id_vec_.end(),
                  id) != last_static_blocked_obj_id_vec_.end();
    const double speed_hysteresis =
        was_successful ? kSpeedHysteresisScale : 1.0;
    const double lon_distache_hysteresis =
        was_successful ? kLonDistanceHysteresis : 0.0;
    const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
    if (frenet_obstacle_sl.s_start > forward_obs_s + lon_distache_hysteresis ||
        frenet_obstacle_sl.s_end + kObsLonDisBuffer <
            ego_frenet_boundary_.s_start) {  // lon concern area
      continue;
    }
    const auto& vehicle_param =
        VehicleConfigurationContext::Instance()->get_vehicle_param();
    //  no lon overlap
    // if (frenet_obstacle_sl.s_start > ego_frenet_boundary_.s_end ||
    //     frenet_obstacle_sl.s_end < ego_frenet_boundary_.s_start) {
    if (frenet_obstacle_sl.s_start > ego_frenet_boundary_.s_end + 2.0) {
      const auto history_iter = lat_obstacle_position.find(id);
      if (history_iter != lat_obstacle_position.end() &&
          !history_iter->second.has_enough_space && observe_frame_num_ < 1) {
        continue;
      }
      const auto lat_obs_iter = lat_obstacle_decision.find(id);
      if (lat_obs_iter != lat_obstacle_decision.end() &&
          (lat_obs_iter->second != LatObstacleDecisionType::IGNORE &&
           lat_obs_iter->second != LatObstacleDecisionType::FOLLOW)) {
        continue;
      }
    } else {  // lon overlap
      if (!was_successful) {
        continue;
      }
      // auto it = std::find(last_static_blocked_obj_id_vec_.begin(),
      //                     last_static_blocked_obj_id_vec_.end(), id);
      // if (it == last_static_blocked_obj_id_vec_.end()) {
      //   continue;
      // }
    }
    // TODO: concern more scene
    const double lat_distache_hysteresis =
        was_successful ? kLatDistanceHysteresis : 0.0;
    if (frenet_obstacle_sl.l_end < -right_width - lat_distache_hysteresis ||
        frenet_obstacle_sl.l_start >
            left_width + lat_distache_hysteresis) {  // away from cur lane
      continue;
    } else {
      if (obstacle->velocity() > speed_hysteresis * kMaxNudgingSpeed) {
        continue;
      }
    }

    static_blocked_obstacles_.emplace_back(obstacle);  // really needed
    static_blocked_obj_id_vec_.emplace_back(id);       // tmperal used
    if (obs_direction_map_.empty() ||
        obs_direction_map_.find(id) == obs_direction_map_.end()) {  // add
      obs_direction_map_[id] = std::make_pair(BorrowDirection::NO_BORROW, 0);
    }
  }
  // delete disappear obs
  for (auto it = obs_direction_map_.begin(); it != obs_direction_map_.end();) {
    if (std::find(static_blocked_obj_id_vec_.begin(),
                  static_blocked_obj_id_vec_.end(),
                  it->first) == static_blocked_obj_id_vec_.end()) {
      it = obs_direction_map_.erase(it);
    } else {
      ++it;
    }
  }

  return true;
}

bool LaneBorrowDecider::UpdateDynamicBlockingObstacles() {
  const auto& agent_mgr = session_->environmental_model().get_agent_manager();
  if (static_blocked_obstacles_.empty() || static_blocked_obj_id_vec_.empty()) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        NO_PASSABLE_OBSTACLE;
    return false;
  }
  // 逆序 防止删除后跳过
  nearest_no_borrow_obstacle_ = nullptr;
  double min_removed_obs_s_dist = std::numeric_limits<double>::max();
  int blocked_obs_id = 0;
  for (int i = static_blocked_obstacles_.size() - 1; i >= 0; --i) {
    blocked_obs_id = static_blocked_obstacles_[i]->obstacle()->id();
    const auto& agent = agent_mgr->GetAgent(blocked_obs_id);
    if (agent == nullptr) {
      lane_borrow_decider_output_.lane_borrow_failed_reason = AGENT_MGR_FAILED;
      // return false;
      continue;
    }
    double speed = agent->speed();
    bool is_cut_in = false;
    bool is_cut_out = false;
    bool is_borrow = true;
    if (agent->is_static()) {
      continue;
    }
    double yaw =
        static_blocked_obstacles_[i]->obstacle()->relative_heading_angle();
    if (std::fabs(yaw) > 2.0) {
      continue;
    }
    // CheckKeyObstaclesIntention(agent, is_cut_in, is_cut_out);
    CheckBlockingObstaclesIntention(blocked_obs_id, is_borrow);
    if (!is_borrow) {
      double obs_s_dist =
          static_blocked_obstacles_[i]->frenet_obstacle_boundary().s_start -
          ego_frenet_boundary_.s_end;
      if (obs_s_dist > 0.0 && obs_s_dist < min_removed_obs_s_dist) {
        min_removed_obs_s_dist = obs_s_dist;
        nearest_no_borrow_obstacle_ = static_blocked_obstacles_[i];
      }
      static_blocked_obstacles_.erase(static_blocked_obstacles_.begin() + i);
      static_blocked_obj_id_vec_.erase(
          std::remove(static_blocked_obj_id_vec_.begin(),
                      static_blocked_obj_id_vec_.end(), blocked_obs_id),
          static_blocked_obj_id_vec_.end());
    }
  }
  if (static_blocked_obstacles_.empty() || static_blocked_obj_id_vec_.empty()) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = CUTINOUT_RISK;
    lane_borrow_decider_output_.failed_obs_id = blocked_obs_id;
    return false;
  }
  return true;
}
void LaneBorrowDecider::CheckKeyObstaclesIntention(const agent::Agent* agent,
                                                   bool& is_cut_in,
                                                   bool& is_cut_out) {
  std::vector<planning_math::Vec2d> obs_corners;
  const auto& obs_box = agent->box();
  obs_corners = obs_box.GetAllCorners();
  std::vector<double> agent_sl_boundary(4);
  agent_sl_boundary.at(0) = std::numeric_limits<double>::lowest();
  agent_sl_boundary.at(1) = std::numeric_limits<double>::max();
  agent_sl_boundary.at(2) = std::numeric_limits<double>::lowest();
  agent_sl_boundary.at(3) = std::numeric_limits<double>::max();
  for (size_t i = 0; i < obs_corners.size(); ++i) {
    double project_s = 0.0, project_l = 0.0;
    current_reference_path_ptr_->get_frenet_coord()->XYToSL(
        obs_corners[i].x(), obs_corners[i].y(), &project_s,
        &project_l);  // 这是投影在路径上的 障碍物角点
    agent_sl_boundary.at(3) = std::fmin(agent_sl_boundary.at(3), project_l);
    agent_sl_boundary.at(2) = std::fmax(agent_sl_boundary.at(2), project_l);
    agent_sl_boundary.at(1) = std::fmin(agent_sl_boundary.at(1), project_s);
    agent_sl_boundary.at(0) = std::fmax(agent_sl_boundary.at(0), project_s);
  }
  double MinCentricOffset = 0.0;
  const double obs_center_l =
      0.5 * (agent_sl_boundary[2] + agent_sl_boundary[3]);
  // 获取障碍物的预测轨迹
  const auto& obs_trajectories = agent->trajectories();
  if (!obs_trajectories.empty() && !obs_trajectories[0].empty()) {
    for (size_t j = 0; j < obs_trajectories[0].size(); j = j + 3) {
      const auto& point = obs_trajectories[0][j];
      SLPoint obs_sl;
      current_reference_path_ptr_->get_frenet_coord()->XYToSL(
          point.x(), point.y(), &obs_sl.s, &obs_sl.l);
      if (j <= 13) {  // 0 3 6 9 12 15 18 21 24   0-5s
        MinCentricOffset = kPreCentricOffsetHigh;
      } else {
        MinCentricOffset = kPreCentricOffsetLow;
      }
      // 判断是否切入自车车道
      // 判断是否切入自车车道
      if (lane_borrow_status_ != kLaneBorrowCrossing) {
        if (std::fabs(obs_sl.l) < MinCentricOffset &&
            agent_sl_boundary[1] - ego_frenet_boundary_.s_end < 20) {
          is_cut_in = true;
          break;
        }

        // if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW &&
        //     obs_center_l - obs_sl.l > 0.5) {
        //   is_cut_out = true;
        // } else if (lane_borrow_decider_output_.borrow_direction ==
        //                RIGHT_BORROW &&
        //            obs_center_l - obs_sl.l < -0.5) {
        //   is_cut_out = true;
        // }

      } else {
        if (std::fabs(obs_center_l) < kPreCentricOffsetHigh &&
            agent_sl_boundary[1] - ego_frenet_boundary_.s_end <
                20) {  // status == crossing   center_obs
          if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW &&
              agent_sl_boundary[2] - obs_sl.l <
                  -current_lane_ptr_->width() * 0.5) {
            is_cut_in = true;
          } else if (lane_borrow_decider_output_.borrow_direction ==
                         RIGHT_BORROW &&
                     agent_sl_boundary[3] - obs_sl.l >
                         current_lane_ptr_->width() * 0.5) {
            is_cut_in = true;
          }
        }
      }
    }
  }
}
// v2
void LaneBorrowDecider::CheckBlockingObstaclesIntention(
    int32 obs_id, bool& will_keep_borrow) {
  const auto& agent_mgr = session_->environmental_model().get_agent_manager();
  const auto& agent = agent_mgr->GetAgent(obs_id);
  if (agent == nullptr) {
    will_keep_borrow = false;
    return;
  }

  // 预测障碍物在 0s / 1.5s / 3.0s 时刻的位置
  const auto box_at_0s = agent->box();
  const auto box_at_1_5s = PredictBoxPosition(agent, 1.5);
  const auto box_at_3s = PredictBoxPosition(agent, 3.0);
  const auto box_at_5s = PredictBoxPosition(agent, 5.0);

  auto sl_at_0s = GetSLboundaryFromAgent(box_at_0s);
  auto sl_at_1_5s = GetSLboundaryFromAgent(box_at_1_5s);
  auto sl_at_3s = GetSLboundaryFromAgent(box_at_3s);
  auto sl_at_5s = GetSLboundaryFromAgent(box_at_5s);

  // 各时刻的借道方向
  BorrowDirection direction_at_0s =
      GetPredBypassDirection(sl_at_0s, obs_id, 1.5);
  BorrowDirection direction_at_1_5s =
      GetPredBypassDirection(sl_at_1_5s, obs_id, 3.0);
  BorrowDirection direction_at_3s =
      GetPredBypassDirection(sl_at_3s, obs_id, 5.0);

  bool will_cut_in;
  bool will_cut_out;

  // 判断是否会切入
  if (direction_at_1_5s == direction_at_0s &&
      direction_at_3s == direction_at_0s && direction_at_0s != NO_BORROW) {
    will_cut_in = false;
  } else {
    will_cut_in = true;
  }

  // 判断是否会切出
  double left_boundary = current_lane_ptr_->width() * 0.5;
  double right_boundary = -current_lane_ptr_->width() * 0.5;
  if (sl_at_3s.l_end < right_boundary || sl_at_3s.l_start > left_boundary) {
    will_cut_out = true;
  } else {
    will_cut_out = false;
  }

  const bool was_successful =
      std::find(last_static_blocked_obj_id_vec_.begin(),
                last_static_blocked_obj_id_vec_.end(),
                obs_id) != last_static_blocked_obj_id_vec_.end();
  const double lane_borrow_status_scale =
      lane_borrow_status_ == kLaneBorrowCrossing ? kLaneBorrowThresholdScale
                                                 : 1.0;
  // 基于预测轨迹首末点相对于中心线的横向位移，判断潜在cut_in/cut_out：横向移动过大的障碍物不借道
  bool excessive_lateral_movement = false;
  const auto& pred_trajectories = agent->trajectories();
  if (!pred_trajectories.empty() && !pred_trajectories[0].empty()) {
    const auto& pred_traj = pred_trajectories[0];
    const auto& frenet_coord = current_reference_path_ptr_->get_frenet_coord();
    if (frenet_coord != nullptr) {
      double project_s = 0.0, project_l_first = 0.0, project_l_last = 0.0;
      frenet_coord->XYToSL(pred_traj.front().x(), pred_traj.front().y(),
                           &project_s, &project_l_first);
      frenet_coord->XYToSL(pred_traj.back().x(), pred_traj.back().y(),
                           &project_s, &project_l_last);
      double lateral_range = std::fabs(project_l_last - project_l_first);
      lateral_range = std::max(lateral_range,
                               std::fabs(sl_at_5s.l_start - sl_at_0s.l_start));
      double hysteresis_lat_mov_conf =
          was_successful ? kLatMovementHysteresisScale : 1.0;
      // Determine if the obstacle is moving toward ego lane (cut-in) or away
      // (cut-out) based on whether |l| is decreasing or increasing.
      const bool is_moving_toward_center =
          std::fabs(project_l_last) < std::fabs(project_l_first);
      const double direction_scale =
          is_moving_toward_center ? 1.0 : kCutOutThresholdScale;
      excessive_lateral_movement =
          lateral_range > kMaxLateralMovementIn5sForBorrow *
                              hysteresis_lat_mov_conf * direction_scale *
                              lane_borrow_status_scale;
    }
  }

  // 大角度的动态障碍物不借道：航向相对于中心线偏角过大的障碍物存在轨迹不确定性
  bool large_heading_angle = false;
  if (!agent->is_static()) {
    const auto& frenet_coord = current_reference_path_ptr_->get_frenet_coord();
    if (frenet_coord != nullptr) {
      double obs_center_s = (sl_at_0s.s_start + sl_at_0s.s_end) * 0.5;
      double obs_center_l = (sl_at_0s.l_start + sl_at_0s.l_end) * 0.5;
      double path_heading = frenet_coord->GetPathPointByS(obs_center_s).theta();
      double relative_heading =
          planning_math::NormalizeAngle(agent->theta() - path_heading);
      double hysteresis_heading_conf =
          was_successful ? kHeadingHysteresisScale : 1.0;
      // Heading pointing toward centerline is cut-in; away is cut-out.
      const bool heading_toward_center =
          (obs_center_l > 0.0 && relative_heading < 0.0) ||
          (obs_center_l < 0.0 && relative_heading > 0.0);
      const double direction_scale =
          heading_toward_center ? 1.0 : kCutOutHeadingScale;
      large_heading_angle = std::fabs(relative_heading) >
                            hysteresis_heading_conf *
                                kMaxRelativeHeadingForBorrow * direction_scale *
                                lane_borrow_status_scale;
    }
  }

  // 最终是否保持借道状态：排除横向位移过大的障碍物
  will_keep_borrow = !will_cut_out && !will_cut_in &&
                     !excessive_lateral_movement && !large_heading_angle;
}

// v2

bool LaneBorrowDecider::ObstacleDecision() {
  auto lane_borrow_pb_info = DebugInfoManager::GetInstance()
                                 .GetDebugInfoPb()
                                 ->mutable_lane_borrow_decider_info();
  static_blocked_obj_id_vec_.clear();
  bypass_direction_ = NO_BORROW;
  if (static_blocked_obstacles_.empty()) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        NO_PASSABLE_OBSTACLE;
    return false;
    // Sort by increasing longitudinal distance from the vehicle
  } else if (static_blocked_obstacles_.size() > 1) {
    std::sort(static_blocked_obstacles_.begin(),
              static_blocked_obstacles_.end(),
              [](const std::shared_ptr<FrenetObstacle>& a,
                 const std::shared_ptr<FrenetObstacle>& b) -> bool {
                return a->frenet_s() < b->frenet_s();
              });
  }
  const std::unordered_set<int> valid_obstacle_types = {
      iflyauto::ObjectType::OBJECT_TYPE_TRAFFIC_CONE,      // 锥桶
      iflyauto::ObjectType::OBJECT_TYPE_TRAFFIC_TEM_SIGN,  // 临时指示牌
      iflyauto::ObjectType::OBJECT_TYPE_WATER_SAFETY_BARRIER,  // 水马
      iflyauto::ObjectType::OBJECT_TYPE_CTASH_BARREL};
  is_facility_ = false;
  // Use the nearest non-new obstacle to determine bypass direction, so a newly
  // appeared front obstacle doesn't flip the established borrow direction.
  // Fall back to static_blocked_obstacles_[0] if all are new.
  std::shared_ptr<FrenetObstacle> front_obs = static_blocked_obstacles_[0];
  if (!last_static_blocked_obj_id_vec_.empty()) {
    for (const auto& obs : static_blocked_obstacles_) {
      const int id = obs->obstacle()->id();
      if (std::find(last_static_blocked_obj_id_vec_.begin(),
                    last_static_blocked_obj_id_vec_.end(),
                    id) != last_static_blocked_obj_id_vec_.end()) {
        front_obs = obs;
        break;
      }
    }
  }
  const auto& first_obs_type = front_obs->obstacle()->type();
  if (valid_obstacle_types.find(first_obs_type) != valid_obstacle_types.end()) {
    is_facility_ = true;
  }
  const auto& front_obstacle_sl = front_obs->frenet_obstacle_boundary();
  const auto& front_id = front_obs->obstacle()->id();
  lane_borrow_pb_info->set_front_obs_center(front_obstacle_sl.l_end);
  BorrowDirection front_obs_bypass_direction =
      GetBypassDirection(front_obstacle_sl, front_id);
  // // borrow more
  // if (front_obs_bypass_direction == LEFT_BORROW && left_borrow_) {
  //     bypass_direction_ = LEFT_BORROW;
  //   } else if (front_obs_bypass_direction == RIGHT_BORROW && right_borrow_) {
  //     bypass_direction_ = RIGHT_BORROW;
  //   }else if (lane_borrow_status_ == LaneBorrowStatus::kLaneBorrowDriving||
  //             lane_borrow_status_ == LaneBorrowStatus::kNoLaneBorrow){
  //     bypass_direction_ = NO_BORROW;
  //     lane_borrow_decider_output_.lane_borrow_failed_reason =
  //     CENTER_OBSTACLE; lane_borrow_decider_output_.failed_obs_id = front_id;
  //     return false;
  //   }

  // for (const auto& obstacle : static_blocked_obstacles_) {
  //   const auto& id = obstacle->obstacle()->id();
  //   const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
  //   //  extend  static area
  //   BorrowDirection obs_bypass_direction =
  //   GetBypassDirection(frenet_obstacle_sl, id); bool same_dir =
  //   (obs_bypass_direction == front_obs_bypass_direction ||
  //   obs_bypass_direction == BorrowDirection::NO_BORROW )?true:false;
  //   if (same_dir &&
  //       (id == static_blocked_obstacles_[0]->obstacle()->id() ||
  //       frenet_obstacle_sl.s_start - obs_end_s_ <
  //           config_.extend_obs_distance)) {
  //     obs_left_l_ = std::max(obs_left_l_, frenet_obstacle_sl.l_end);
  //     obs_right_l_ = std::min(obs_right_l_, frenet_obstacle_sl.l_start);
  //     obs_start_s_ = std::min(obs_start_s_, frenet_obstacle_sl.s_start);
  //     obs_end_s_ = std::max(obs_end_s_, frenet_obstacle_sl.s_end);
  //     static_blocked_obj_id_vec_.emplace_back(obstacle->obstacle()->id());
  //   }else{
  //     break;
  //   }
  // }

  if (false) {
    if (front_obs_bypass_direction == LEFT_BORROW && left_borrow_) {
      bypass_direction_ = LEFT_BORROW;
    } else if (front_obs_bypass_direction == RIGHT_BORROW && right_borrow_) {
      bypass_direction_ = RIGHT_BORROW;
    }
    for (const auto& obstacle : static_blocked_obstacles_) {
      const auto& id = obstacle->obstacle()->id();
      const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
      //  extend  static area
      if (id == static_blocked_obstacles_[0]->obstacle()->id() ||
          frenet_obstacle_sl.s_start - obs_end_s_ <
              config_.extend_obs_distance) {
        obs_left_l_ = std::max(obs_left_l_, frenet_obstacle_sl.l_end);
        obs_right_l_ = std::min(obs_right_l_, frenet_obstacle_sl.l_start);
        obs_start_s_ = std::min(obs_start_s_, frenet_obstacle_sl.s_start);
        obs_end_s_ = std::max(obs_end_s_, frenet_obstacle_sl.s_end);
        static_blocked_obj_id_vec_.emplace_back(obstacle->obstacle()->id());
      }
    }
  } else {
    if (front_obs_bypass_direction == LEFT_BORROW && left_borrow_) {
      bypass_direction_ = LEFT_BORROW;
      right_borrow_ = false;
    } else if (front_obs_bypass_direction == RIGHT_BORROW && right_borrow_) {
      bypass_direction_ = RIGHT_BORROW;
      left_borrow_ = false;
    } else {
      bypass_direction_ = NO_BORROW;
      lane_borrow_decider_output_.lane_borrow_failed_reason = CENTER_OBSTACLE;
      lane_borrow_decider_output_.failed_obs_id = front_id;
      return false;
    }

    for (const auto& obstacle : static_blocked_obstacles_) {
      const auto& id = obstacle->obstacle()->id();
      const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
      BorrowDirection obs_bypass_direction =
          GetBypassDirection(frenet_obstacle_sl, id);

      // Newly added obstacles (not in last frame) must match existing borrow
      // direction; skip rather than break so we don't lose further valid ones.
      const bool is_new =
          !last_static_blocked_obj_id_vec_.empty() &&
          std::find(last_static_blocked_obj_id_vec_.begin(),
                    last_static_blocked_obj_id_vec_.end(),
                    id) == last_static_blocked_obj_id_vec_.end();
      if (is_new && obs_bypass_direction != bypass_direction_) {
        continue;
      }

      if (obs_bypass_direction == bypass_direction_ &&
          (id == static_blocked_obstacles_[0]->obstacle()->id() ||
           id == front_id ||
           frenet_obstacle_sl.s_start - obs_end_s_ <
               config_.extend_obs_distance)) {
        obs_left_l_ = std::max(obs_left_l_, frenet_obstacle_sl.l_end);
        obs_right_l_ = std::min(obs_right_l_, frenet_obstacle_sl.l_start);
        obs_start_s_ = std::min(obs_start_s_, frenet_obstacle_sl.s_start);
        obs_end_s_ = std::max(obs_end_s_, frenet_obstacle_sl.s_end);
        static_blocked_obj_id_vec_.emplace_back(obstacle->obstacle()->id());
      } else {
        break;
      }
    }
  }

  double min_removed_obs_s_dist = std::numeric_limits<double>::max();
  if (nearest_no_borrow_obstacle_ != nullptr) {
    min_removed_obs_s_dist =
        nearest_no_borrow_obstacle_->frenet_obstacle_boundary().s_start -
        ego_frenet_boundary_.s_end;
  }
  for (const auto& obstacle : static_blocked_obstacles_) {
    const auto& id = obstacle->obstacle()->id();
    if (std::find(static_blocked_obj_id_vec_.begin(),
                  static_blocked_obj_id_vec_.end(),
                  id) == static_blocked_obj_id_vec_.end()) {
      if (obstacle->obstacle()->is_reverse()) {
        // 对向车暂时在这里不考虑，在后续修饰path的考虑
        continue;
      }
      double obs_s_dist = obstacle->frenet_obstacle_boundary().s_start -
                          ego_frenet_boundary_.s_end;
      if (obs_s_dist > 0.0 && obs_s_dist < min_removed_obs_s_dist) {
        min_removed_obs_s_dist = obs_s_dist;
        nearest_no_borrow_obstacle_ = obstacle;
      }
    }
  }

  static_blocked_obstacles_.erase(
      std::remove_if(static_blocked_obstacles_.begin(),
                     static_blocked_obstacles_.end(),
                     [this](const std::shared_ptr<FrenetObstacle>& obstacle) {
                       return std::find(static_blocked_obj_id_vec_.begin(),
                                        static_blocked_obj_id_vec_.end(),
                                        obstacle->obstacle()->id()) ==
                              static_blocked_obj_id_vec_.end();
                     }),
      static_blocked_obstacles_.end());

  if (obs_left_l_ <= obs_right_l_) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        NO_PASSABLE_OBSTACLE;
    return false;
  }
  // output to dp
  lane_borrow_decider_output_.area_start_s = obs_start_s_;
  lane_borrow_decider_output_.area_end_s = obs_end_s_;
  lane_borrow_decider_output_.area_start_l = obs_right_l_;
  lane_borrow_decider_output_.area_end_l = obs_left_l_;

  // if (lane_borrow_status_ == kNoLaneBorrow) {
  //   double left_width =
  //       current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  //   double right_width =
  //       current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;

  //   const auto& vehicle_param =
  //       VehicleConfigurationContext::Instance()->get_vehicle_param();
  //   if (obs_left_l_ + vehicle_param.width + kLatPassableBuffer < left_width
  //   ||
  //       obs_right_l_ - vehicle_param.width - kLatPassableBuffer >
  //           -right_width) {
  //     lane_borrow_decider_output_.lane_borrow_failed_reason =
  //     SELF_LANE_ENOUGH; return false;
  //   }
  //   obs_left_l_ += kObsLatBuffer;
  //   obs_right_l_ -= kObsLatBuffer;
  // }
  return true;
}
BorrowDirection LaneBorrowDecider::GetBypassDirection(
    const FrenetObstacleBoundary& frenet_obstacle_sl, const int obs_id) {
  const auto& agent_mgr = session_->environmental_model().get_agent_manager();
  const auto& agent = agent_mgr->GetAgent(obs_id);
  if (agent == nullptr) {
    return NO_BORROW;
  }
  double lane_width = current_lane_ptr_->width();
  bool is_static = agent->speed() < 2.0 || agent->is_static();
  bool is_tiny = agent->is_vru() || agent->width() < 0.5;  // 行人在之前就过滤了
  double scale = 1.0;                                      // 兜底感知跳动
  double margin = 0.0;
  if (lane_borrow_status_ != kNoLaneBorrow) {
    if (lane_borrow_status_ == kLaneBorrowCrossing) {
      // kLaneBorrowCrossing状态，提升借道能力，避免
      scale = 3;
      margin = -kDynamicEdgeDistanceWithCrossing;
    } else {
      scale = 2.5;
      margin = -kDynamicEdgeDistance;
    }
  }

  // 障碍物在自车侧方（纵向有重叠），为安全考虑必须给出借道方向，不能返回NO_BORROW
  bool is_obs_beside_ego =
      frenet_obstacle_sl.s_start <= ego_frenet_boundary_.s_end &&
      frenet_obstacle_sl.s_end >= ego_frenet_boundary_.s_start;

  // 先排除
  if (is_tiny || !is_static) {
    if (frenet_obstacle_sl.l_start < margin &&
        frenet_obstacle_sl.l_end > -margin) {  // 异号，压住中心线, 滞回
      if (is_obs_beside_ego) {
        // 侧方障碍物强制给方向：根据障碍物中心相对自车的横向位置决定
        double obs_center_l =
            0.5 * (frenet_obstacle_sl.l_start + frenet_obstacle_sl.l_end);
        double ego_center_l =
            0.5 * (ego_frenet_boundary_.l_start + ego_frenet_boundary_.l_end);
        if (obs_center_l < ego_center_l) {
          obs_direction_map_[obs_id].first = LEFT_BORROW;
          obs_direction_map_[obs_id].second = 0;
          return LEFT_BORROW;
        } else {
          obs_direction_map_[obs_id].first = RIGHT_BORROW;
          obs_direction_map_[obs_id].second = 0;
          return RIGHT_BORROW;
        }
      }
      if (obs_direction_map_[obs_id].second <
          config_.centric_obs_frames) {  // 滞回逻辑
        obs_direction_map_[obs_id].second += 1;
        return obs_direction_map_[obs_id].first;
      } else {
        obs_direction_map_[obs_id].first = NO_BORROW;
        return NO_BORROW;
      }
    }
  } else {  // 大 静态： 右边缘 在中心线右侧 0.25+  并且 左边缘 在中心线左侧
            // 0.25+
    if (frenet_obstacle_sl.l_start < -scale * kStaticEdgeDistance &&
        frenet_obstacle_sl.l_end > scale * kStaticEdgeDistance) {  //左侧同理
      if (is_obs_beside_ego) {
        // 侧方障碍物强制给方向
        double obs_center_l =
            0.5 * (frenet_obstacle_sl.l_start + frenet_obstacle_sl.l_end);
        double ego_center_l =
            0.5 * (ego_frenet_boundary_.l_start + ego_frenet_boundary_.l_end);
        if (obs_center_l < ego_center_l) {
          obs_direction_map_[obs_id].first = LEFT_BORROW;
          obs_direction_map_[obs_id].second = 0;
          return LEFT_BORROW;
        } else {
          obs_direction_map_[obs_id].first = RIGHT_BORROW;
          obs_direction_map_[obs_id].second = 0;
          return RIGHT_BORROW;
        }
      }
      if (obs_direction_map_[obs_id].second <
          config_.centric_obs_frames) {  //滞回逻辑
        obs_direction_map_[obs_id].second += 1;
        return obs_direction_map_[obs_id].first;
      } else {
        obs_direction_map_[obs_id].first = NO_BORROW;
        return NO_BORROW;
      }
    }
  }
  // 未被排除， 再判断左右方向
  if (-frenet_obstacle_sl.l_start >
      frenet_obstacle_sl.l_end) {  // right offset, left borrow
    obs_direction_map_[obs_id].first = LEFT_BORROW;
    obs_direction_map_[obs_id].second = 0;
    return LEFT_BORROW;
  } else {  // left offset, right borrow
    obs_direction_map_[obs_id].first = RIGHT_BORROW;
    obs_direction_map_[obs_id].second = 0;
    return RIGHT_BORROW;
  }

  // double max_central_offset = is_static ? kMaxCentricOffset : 0.95;

  // const double obs_center_l =
  //     0.5 * (frenet_obstacle_sl.l_start + frenet_obstacle_sl.l_end);
  // double scale = 1.0;
  // if (lane_borrow_status_ != kNoLaneBorrow) {
  //   scale = 0.5;
  // }
  // if (std::fabs(obs_center_l) <= scale * max_central_offset) {
  //   if (obs_direction_map_[obs_id].second < config_.centric_obs_frames) {
  //     obs_direction_map_[obs_id].second += 1;
  //     return obs_direction_map_[obs_id].first;
  //   } else {
  //     obs_direction_map_[obs_id].first = NO_BORROW;
  //     return NO_BORROW;
  //   }
  // } else if (obs_center_l < -scale * max_central_offset) {
  //   obs_direction_map_[obs_id].first = LEFT_BORROW;
  //   obs_direction_map_[obs_id].second = 0;
  //   return LEFT_BORROW;
  // } else {
  //   obs_direction_map_[obs_id].first = RIGHT_BORROW;
  //   obs_direction_map_[obs_id].second = 0;
  //   return RIGHT_BORROW;
  // }
}
BorrowDirection LaneBorrowDecider::GetPredBypassDirection(
    const FrenetObstacleBoundary& frenet_obstacle_sl, const int obs_id,
    const double delta_t) {
  const auto& agent_mgr = session_->environmental_model().get_agent_manager();
  const auto& agent = agent_mgr->GetAgent(obs_id);
  if (agent == nullptr) {
    return NO_BORROW;
  }
  double lane_width = current_lane_ptr_->width();
  bool is_static = agent->speed() < 2.0 || agent->is_static();
  bool is_tiny = agent->is_vru() || agent->width() < 0.5;  // 行人在之前就过滤了
  double scale = 1.0;
  double margin = 0.0;  // 未借道：以是否跨中心线判断
  const bool was_successful =
      std::find(last_static_blocked_obj_id_vec_.begin(),
                last_static_blocked_obj_id_vec_.end(),
                obs_id) != last_static_blocked_obj_id_vec_.end();
  if (was_successful) {
    // 滞回
    if (lane_borrow_status_ == kLaneBorrowCrossing) {
      // kLaneBorrowCrossing状态，提升借道能力，避免
      scale = kCrossingDirectionHysteresisScale;
      margin = -kDynamicEdgeDistanceWithCrossing;
    } else {
      scale = kCentricDirectionHysteresisScale;
      margin = -kDynamicEdgeDistance;
    }
    // 考虑预测时间的影响
    if (delta_t >= 4.9) {
      margin -= 0.2;
    } else if (delta_t >= 2.9) {
      margin -= 0.1;
    }
  }

  if (is_tiny || !is_static) {
    // 障碍物与中心线 ± margin 区域有重叠 → 贴近或横跨中心线
    if (frenet_obstacle_sl.l_start < margin &&
        frenet_obstacle_sl.l_end > -margin) {
      return NO_BORROW;
    }
  } else {  // 大静态：两侧边缘均超过 scale * 阈值 → 横跨中心线
    if (frenet_obstacle_sl.l_start < -scale * kStaticEdgeDistance &&
        frenet_obstacle_sl.l_end > scale * kStaticEdgeDistance) {
      return NO_BORROW;
    }
  }
  // 未被排除， 再判断左右方向
  if (-frenet_obstacle_sl.l_start >
      frenet_obstacle_sl.l_end) {  // right offset, left borrow
    return LEFT_BORROW;
  } else {  // left offset, right borrow
    return RIGHT_BORROW;
  }
}

// v2
bool LaneBorrowDecider::EnoughSafetyDistance() {
  // 后续可以改为近端path曲率判断
  if (static_blocked_obj_id_vec_.empty()) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        NO_PASSABLE_OBSTACLE;
    return false;
  }
  const auto& agent_mgr = session_->environmental_model().get_agent_manager();
  const auto& agent = agent_mgr->GetAgent(static_blocked_obj_id_vec_[0]);
  if (agent == nullptr) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        NO_PASSABLE_OBSTACLE;
    return false;
  }
  bool is_agent_static = agent->is_static() || agent->speed() < 0.3;
  bool is_close = obs_start_s_ - ego_frenet_boundary_.s_end < 4.5;
  bool is_ego_pull_over = ego_speed_ < 0.3;
  // 车速 静止 距离
  if (is_agent_static && is_close && is_ego_pull_over &&
      lane_borrow_status_ == kNoLaneBorrow) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        STATIC_AREA_TOO_CLOSE;
    return false;
  } else {
    return true;
  }
}

bool LaneBorrowDecider::CheckLaneBorrowDircetion() {
  // 拿静态区域的第一个obs_id 判断在path的左边还是右边
  //  const auto& id = static_blocked_obj_id_vec_[0];
  // const auto& front_obstacle_sl =
  //     static_blocked_obstacles_[0]->frenet_obstacle_boundary();
  // const auto& dp_path = path_decider_->refined_paths();
  // const auto& rule_path = rule_path_decider_->refined_paths();
  // const auto& path = config_.use_dp_path_planning ? dp_path : rule_path;
  // double center_obs_l =
  //     (front_obstacle_sl.l_start + front_obstacle_sl.l_end) / 2;
  // double center_obs_s =
  //     (front_obstacle_sl.s_start + front_obstacle_sl.s_end) / 2;

  // auto it = std::lower_bound(
  //     path.begin(), path.end(), center_obs_s,
  //     [](const PathPoint& point, double s) { return point.s() < s; });
  // bool isEndpoint = false;
  // double dp_path_l = 0.0;
  // if (it == path.begin()) {
  //   isEndpoint = true;
  //   dp_path_l = path.front().l();
  // } else if (it == std::prev(path.end())) {
  //   isEndpoint = true;
  //   dp_path_l = path.back().l();
  // }

  // const auto& prev_point = *(it - 1);
  // const auto& next_point = *it;
  // if (!isEndpoint) {
  //   if (std::abs(prev_point.s() - center_obs_s) <
  //       std::abs(next_point.s() - center_obs_s)) {
  //     dp_path_l = prev_point.l();
  //   } else {
  //     dp_path_l = next_point.l();
  //   }
  // }

  // if (dp_path_l > center_obs_l) {
  //   path_direction_ = LEFT_BORROW;
  // } else if (dp_path_l < center_obs_l) {
  //   path_direction_ = RIGHT_BORROW;
  // } else {
  //   lane_borrow_decider_output_.lane_borrow_failed_reason = DP_NO_DIRECTION;
  //   return false;
  // }

  // if (path_direction_ == bypass_direction_) {
  //   lane_borrow_decider_output_.borrow_direction = path_direction_;
  //   return true;
  // } else if (lane_borrow_status_ == kLaneBorrowCrossing &&
  //            path_direction_ != bypass_direction_) {
  //   lane_borrow_decider_output_.borrow_direction = path_direction_;
  //   return true;
  // } else {
  //   lane_borrow_decider_output_.borrow_direction = NO_BORROW;
  //   lane_borrow_decider_output_.lane_borrow_failed_reason =
  //       BORROWDIRECTION_DIFFERENT;
  //   return false;
  // }
  lane_borrow_decider_output_.borrow_direction = bypass_direction_;
  return true;
}

// v2
bool LaneBorrowDecider::CrossingPositionJudgment() {
  if (!ego_sl_state_.is_valid()) {
    return false;
  }
  const auto& c = ego_sl_state_.corners();
  if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW) {
    const double current_front_left_lane_l =
        current_lane_ptr_->width_by_s(c.s_front_left) * 0.5;
    const double current_rear_left_lane_l =
        current_lane_ptr_->width_by_s(c.s_rear_left) * 0.5;
    return c.l_front_left > current_front_left_lane_l; //&&
           //c.l_rear_left > current_rear_left_lane_l;
  }
  if (lane_borrow_decider_output_.borrow_direction == RIGHT_BORROW) {
    const double current_front_right_lane_l =
        current_lane_ptr_->width_by_s(c.s_front_right) * 0.5;
    const double current_rear_right_lane_l =
        current_lane_ptr_->width_by_s(c.s_rear_right) * 0.5;
    return c.l_front_right < -current_front_right_lane_l; //&&
           //c.l_rear_right < -current_rear_right_lane_l;
  }
  return false;

  // const auto& current_frenet_coord =
  // current_lane_ptr_->get_lane_frenet_coord(); const auto& vehicle_param =
  //     VehicleConfigurationContext::Instance()->get_vehicle_param();
  // double heading_angle = heading_angle_;

  // double ego_x = ego_xy_.x;
  // double ego_y = ego_xy_.y;

  // // Get the corner points' Cartesian coordinates while traveling straight
  // Point2D corner_front_left_point_xy(vehicle_param.front_edge_to_rear_axle,
  //                                    vehicle_param.width * 0.5);
  // Point2D corner_front_right_point_xy(vehicle_param.front_edge_to_rear_axle,
  //                                     -vehicle_param.width * 0.5);
  // Point2D corner_rear_left_point_xy(-vehicle_param.rear_edge_to_rear_axle,
  //                                   vehicle_param.width * 0.5);
  // Point2D corner_rear_right_point_xy(-vehicle_param.rear_edge_to_rear_axle,
  //                                    -vehicle_param.width * 0.5);

  // SLPoint corner_front_left, corner_rear_left, corner_front_right,
  //     corner_rear_right;

  // if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW) {
  //   Point2D corner_front_left_xy = CartesianRotation(
  //       corner_front_left_point_xy, heading_angle, ego_x, ego_y);
  //   Point2D corner_rear_left_xy = CartesianRotation(
  //       corner_rear_left_point_xy, heading_angle, ego_x, ego_y);

  //   // Back to the SL coordinate system and compare with the lane lines.
  //   current_frenet_coord->XYToSL(corner_front_left_xy.x,
  //   corner_front_left_xy.y,
  //                                &corner_front_left.s, &corner_front_left.l);
  //   current_frenet_coord->XYToSL(corner_rear_left_xy.x,
  //   corner_rear_left_xy.y,
  //                                &corner_rear_left.s, &corner_rear_left.l);

  //   const double current_front_left_lane_l =
  //       current_lane_ptr_->width_by_s(corner_front_left.s) * 0.5;
  //   const double current_rear_left_lane_l =
  //       current_lane_ptr_->width_by_s(corner_rear_left.s) * 0.5;
  //   if (corner_front_left.l > current_front_left_lane_l &&
  //       corner_rear_left.l > current_rear_left_lane_l) {
  //     return true;
  //   }
  //   return false;

  // } else if (lane_borrow_decider_output_.borrow_direction == RIGHT_BORROW) {
  //   Point2D corner_front_right_xy = CartesianRotation(
  //       corner_front_right_point_xy, heading_angle, ego_x, ego_y);
  //   Point2D corner_rear_right_xy = CartesianRotation(
  //       corner_rear_right_point_xy, heading_angle, ego_x, ego_y);

  //   current_frenet_coord->XYToSL(corner_front_right_xy.x,
  //                                corner_front_right_xy.y,
  //                                &corner_front_right.s,
  //                                &corner_front_right.l);
  //   current_frenet_coord->XYToSL(corner_rear_right_xy.x,
  //   corner_rear_right_xy.y,
  //                                &corner_rear_right.s, &corner_rear_right.l);

  //   const double current_front_right_lane_l =
  //       current_lane_ptr_->width_by_s(corner_front_right.s) * 0.5;
  //   const double current_rear_right_lane_l =
  //       current_lane_ptr_->width_by_s(corner_rear_right.s) * 0.5;

  //   if (corner_front_right.l < -current_front_right_lane_l &&
  //       corner_rear_right.l < -current_rear_right_lane_l) {
  //     return true;
  //   }
  //   return false;
  // }
  // return false;
}

bool LaneBorrowDecider::CheckBackWardObs() {
  double neighbor_left_width = 1.75;
  double neighbor_right_width = 1.75;

  const double current_left_lane_width = current_lane_ptr_->width() * 0.5;
  const double current_right_lane_width = current_lane_ptr_->width() * 0.5;
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  double left_risk_bound = 0.;
  double right_risk_bound = 0.;
  if (lane_borrow_decider_output_.borrow_direction ==
      LEFT_BORROW) {  // change: bound just half lane
    const double neighbor_width =
        left_lane_ptr_->width(vehicle_param.front_edge_to_rear_axle);
    // neighbor lane width
    neighbor_left_width = neighbor_width * 0.5;
    neighbor_right_width = neighbor_width * 0.5;

    // Calculate the total width that can be borrowed from the left lane
    left_risk_bound = current_left_lane_width + neighbor_right_width;
    right_risk_bound = current_left_lane_width;
  } else if (lane_borrow_decider_output_.borrow_direction == RIGHT_BORROW) {
    const double neighbor_width =
        right_lane_ptr_->width(vehicle_param.front_edge_to_rear_axle);
    neighbor_left_width = neighbor_width * 0.5;
    neighbor_right_width = neighbor_width * 0.5;
    right_risk_bound = -current_right_lane_width - neighbor_left_width;
    left_risk_bound = -current_right_lane_width;
  }
  // TTC设置
  double MaxConcernCollisionTime = 4;
  if (lane_borrow_decider_output_.lane_borrow_state == kLaneBorrowDriving) {
    MaxConcernCollisionTime = 3.0;
  } else if (lane_borrow_decider_output_.lane_borrow_state == kNoLaneBorrow ||
             lane_borrow_decider_output_.lane_borrow_state ==
                 kLaneBorrowWaitting) {
    MaxConcernCollisionTime = 5.0;
  }

  for (const auto& obstacle : obstacles) {
    const auto& id = obstacle->obstacle()->id();
    const auto& obs_type = obstacle->obstacle()->type();
    double obstacle_v = obstacle->frenet_velocity_s();
    const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
    if (!obstacle->b_frenet_valid()) {
      continue;
    }
    // if (obs_type == iflyauto::ObjectType::OBJECT_TYPE_PEDESTRIAN) {
    //   continue;
    // }
    // risk bound 左右之外的不考虑
    if (frenet_obstacle_sl.l_start > left_risk_bound ||
        frenet_obstacle_sl.l_end < right_risk_bound) {
      continue;
    }
    if ((obstacle->is_static() || std::fabs(obstacle->velocity()) < 2) &&
        obs_type != iflyauto::ObjectType::OBJECT_TYPE_PEDESTRIAN) {
      // 不走搜索的话，这里需要进一步处理
      continue;  // 静态忽略
    }
    if (!(obstacle->obstacle()->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    // 动态车 对向 同向 前方侧方后方
    if (frenet_obstacle_sl.s_start >
        ego_frenet_boundary_.s_end) {  // 完全在自车以前
      if (obstacle_v > .0) {
        continue;  // 同向车 不考虑
      }
      // 在前方 并且 对向车 TTC
      double relative_speed = ego_speed_ - obstacle_v;  // 相对和速度
      double dist =
          frenet_obstacle_sl.s_start -
          ego_frenet_boundary_.s_end;  // 后方车头未追上为正  有干涉则为负
      double TTC = dist / (0.01 + relative_speed);
      if (TTC >= MaxConcernCollisionTime) {
        continue;
      } else {
        lane_borrow_decider_output_.lane_borrow_failed_reason =
            AHEAD_COMING_OBS;
        lane_borrow_decider_output_.failed_obs_id = obstacle->obstacle()->id();
        return false;
      }
    } else if (frenet_obstacle_sl.s_end <
               ego_frenet_boundary_.s_start) {  // 完全在自车以后
      // 在后方 并且 反向车 慢速车 不考虑
      if (obstacle_v < .0 || obstacle_v < ego_speed_) {
        continue;
      }
      // 在自车后方的过滤
      const double ego_l_start = ego_frenet_boundary_.l_start;
      const double ego_l_end = ego_frenet_boundary_.l_end;
      const double obstacle_l_start =
          obstacle->frenet_obstacle_boundary().l_start;
      const double obstacle_l_end = obstacle->frenet_obstacle_boundary().l_end;
      double start_l = std::max(ego_l_start, obstacle_l_start);
      double end_l = std::min(ego_l_end, obstacle_l_end);
      bool lat_overlap_for_rear_obs = (start_l < end_l - kLatOverlapBuffer);
      bool is_care_rear_obstacle = !lat_overlap_for_rear_obs;
      if (!is_care_rear_obstacle) {
        continue;
      }
      // 在后方 并且 同向车 高速车
      double relative_speed = obstacle_v - ego_speed_;  // 相对差速度
      double dist =
          ego_frenet_boundary_.s_start -
          frenet_obstacle_sl.s_end;  // 后方车头未追上为正  有干涉则为负
      double TTC = dist / (0.01 + relative_speed);
      if (TTC >= MaxConcernCollisionTime) {
        continue;
      }
      // 比较后方车和自车到达第一个借道障碍物的时间
      if (IsRearObsSafeByArrivalTime(id, frenet_obstacle_sl, obstacle_v)) {
        continue;
      }
      lane_borrow_decider_output_.lane_borrow_failed_reason =
          BACKWARD_OBSTACLE_TOO_CLOSE;
      lane_borrow_decider_output_.failed_obs_id = obstacle->obstacle()->id();
      return false;
    } else {  // 当前位置在侧方的 动态的 都关注
      lane_borrow_decider_output_.lane_borrow_failed_reason =
          NEARBY_OBSTACLE_TOO_CLOSE;
      lane_borrow_decider_output_.failed_obs_id = obstacle->obstacle()->id();
      return false;
    }
    // if(obstacle_v < -2.0){
    //     continue; // 对向车 不能输出偏移path
    // }
    // const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
    // const auto& first_obstacle_sl =
    // static_blocked_obstacles_[0]->frenet_obstacle_boundary();
    // // 前方车过滤 dp可以避让
    // if (frenet_obstacle_sl.s_start > ego_frenet_boundary_.s_end) {
    //   continue;
    // }else if(ego_speed_ > obstacle_v && frenet_obstacle_sl.s_end <
    // ego_frenet_boundary_.s_start){//低于自车速速的忽略
    //   continue;
    // }
    // //bound 左右之外的不考虑
    // if (frenet_obstacle_sl.l_start > left_risk_bound ||
    //   frenet_obstacle_sl.l_end < right_risk_bound) {
    //   continue;
    // }
    // // TTC设置
    // double relative_speed = obstacle_v - ego_speed_; // 由于上方过滤，必为正
    // double dist = ego_frenet_boundary_.s_start - frenet_obstacle_sl.s_end;//
    // 后方车头未追上为正  有干涉则为负 if(dist < 0){
    //   // 动态车干涉
    //   lane_borrow_decider_output_.lane_borrow_failed_reason =
    //   NEARBY_OBSTACLE_TOO_CLOSE; lane_borrow_decider_output_.failed_obs_id =
    //   obstacle->obstacle()->id(); return false;
    // } else {
    //   // 后方车头未追上
    //   // TTC 计算
    //   double TTC = dist / (0.01 + relative_speed);
    //   double MaxConcernCollisionTime = 1.5;
    //   if (lane_borrow_decider_output_.lane_borrow_state ==
    //       kLaneBorrowDriving) {
    //     MaxConcernCollisionTime = 3.0;
    //   } else if (lane_borrow_decider_output_.lane_borrow_state ==
    //               kNoLaneBorrow) {
    //     MaxConcernCollisionTime = 5.0;
    //   }
    //   if (TTC >= MaxConcernCollisionTime) {
    //     continue;
    //   }else{
    //     lane_borrow_decider_output_.lane_borrow_failed_reason =
    //           BACKWARD_OBSTACLE_TOO_CLOSE;
    //       lane_borrow_decider_output_.failed_obs_id =
    //           obstacle->obstacle()->id();
    //       return false;
    //   }
  }
  return true;
}

bool LaneBorrowDecider::IsRearObsSafeByArrivalTime(
    int32_t rear_obs_id, const FrenetObstacleBoundary& rear_obs_sl,
    double rear_obs_v) {
  if (static_blocked_obstacles_.empty()) {
    return true;
  }
  constexpr double kEgoAcceleration = 0.0;
  constexpr double kSafetyMargin = 0.8;

  const double first_obs_s =
      static_blocked_obstacles_[0]->frenet_obstacle_boundary().s_start;

  // 自车到达第一个借道障碍物的时间：匀加速 s = v0*t + 0.5*a*t^2
  const double ego_dist_to_obs = first_obs_s - ego_frenet_boundary_.s_end;
  double ego_time_to_obs = std::numeric_limits<double>::max();
  if (ego_dist_to_obs > 0.0) {
    const double a = 0.5 * kEgoAcceleration;
    const double b = ego_speed_;
    const double c = -ego_dist_to_obs;
    const double delta = b * b - 4.0 * a * c;
    const double x = (-b + std::sqrt(delta));
    if (delta >= 0.0 && x > 0 && std::abs(a) > 1e-6) {
      ego_time_to_obs = x / (2.0 * a);
    } else {
      ego_time_to_obs = ego_dist_to_obs / (ego_speed_ + 0.01);
    }
  } else {
    ego_time_to_obs = 0.0;
  }

  // 后方车到达第一个借道障碍物的时间：优先使用预测轨迹
  const auto& agent_mgr = session_->environmental_model().get_agent_manager();
  const auto& rear_agent = agent_mgr->GetAgent(rear_obs_id);
  double rear_time_to_obs = std::numeric_limits<double>::max();

  if (rear_agent != nullptr) {
    const auto& pred_trajectories = rear_agent->trajectories_used_by_st_graph();
    if (!pred_trajectories.empty() && !pred_trajectories[0].empty()) {
      const auto& traj = pred_trajectories[0];
      const auto& frenet_coord =
          current_reference_path_ptr_->get_frenet_coord();
      const double base_time = traj.front().absolute_time();
      for (size_t i = 0; i < traj.size(); ++i) {
        double pred_s = 0.0, pred_l = 0.0;
        frenet_coord->XYToSL(traj[i].x(), traj[i].y(), &pred_s, &pred_l);
        if (pred_s + 0.5 * rear_agent->length() >= first_obs_s) {
          rear_time_to_obs = traj[i].absolute_time() - base_time;
          break;
        }
      }
    }
    // fallback: 预测轨迹无效时使用恒速模型
    if (rear_time_to_obs == std::numeric_limits<double>::max()) {
      const double rear_dist = first_obs_s - rear_obs_sl.s_end;
      rear_time_to_obs = rear_dist / (rear_obs_v + 0.01);
    }
  }
  double time_buffer = kSafetyMargin;
  if (last_lane_borrow_failed_reason_ != BACKWARD_OBSTACLE_TOO_CLOSE) {
    time_buffer = 0.4;
  }
  return rear_time_to_obs - ego_time_to_obs > time_buffer;
}

bool LaneBorrowDecider::IfChangeTargetLane() {
  // 左侧借道 左车道存在 并且是同向车道(暂时用边界判断，边界不可以是黄色的线)
  //借道障碍物较多  当前自车后轴靠近左车道  切换
  if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW) {
    if (left_lane_ptr_ == nullptr) {
      return false;
    }
    if (left_lane_ptr_->get_lane_type() == iflyauto::LANETYPE_OPPOSITE) {
      return false;  //对向车道不可切换
    }
  } else if (lane_borrow_decider_output_.borrow_direction == RIGHT_BORROW) {
    if (right_lane_ptr_ == nullptr) {
      return false;
    }
    if (right_lane_ptr_->get_lane_type() == iflyauto::LANETYPE_OPPOSITE) {
      return false;  //对向车道不可切换
    }
  } else {
    return false;
  }

  const auto& lat_obstacle_decision = session_->planning_context()
                                          .lateral_obstacle_decider_output()
                                          .lat_obstacle_decision;
  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  std::vector<double> obs_s_distribution;
  obs_s_distribution.reserve(obstacles.size());
  const double current_lane_width = current_lane_ptr_->width();
  const double ego_s_start = ego_frenet_boundary_.s_start;
  const double s_max = ego_s_start + 80.0;
  for (const auto& obstacle : obstacles) {
    if (!obstacle->b_frenet_valid()) {
      continue;
    }
    if (!(obstacle->obstacle()->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
    if (frenet_obstacle_sl.s_end > s_max ||
        frenet_obstacle_sl.s_end + kObsLonDisBuffer < ego_s_start) {
      continue;
    }
    if (obstacle->velocity() > kMaxNudgingSpeed) {
      continue;
    }
    if (frenet_obstacle_sl.l_end < -current_lane_width * 0.5 ||
        frenet_obstacle_sl.l_start > current_lane_width * 0.5) {
      continue;
    }
    const auto& idx = obstacle->obstacle()->id();
    const auto lat_obs_iter = lat_obstacle_decision.find(idx);
    if (lat_obs_iter != lat_obstacle_decision.end() &&
        (lat_obs_iter->second != LatObstacleDecisionType::IGNORE &&
         lat_obs_iter->second != LatObstacleDecisionType::FOLLOW)) {
      continue;
    }
    obs_s_distribution.push_back(frenet_obstacle_sl.s_start);
  }
  if (obs_s_distribution.size() <= 1) {
    return false;
  }
  // 按s值从大到小排序（从远到近）
  std::sort(obs_s_distribution.begin(), obs_s_distribution.end(),
            std::greater<double>());
  // 计算相邻障碍物之间的最大gap
  double max_gap = 0.0;
  double farest_s = s_max;
  for (const double obs_s : obs_s_distribution) {
    double gap = farest_s - obs_s;
    max_gap = std::max(gap, max_gap);
    farest_s = obs_s;
  }
  // 提前退出：若 max_gap >= 40，无需切换
  if (max_gap >= kChangeLaneLonDistanceThr) {
    return false;
  }
  // 判断自车后轴中心是否已经靠近目标车道
  // lateral_origin_lane: 自车相对于当前车道中心的横向偏移（左正右负）
  const double lateral_origin_lane =
      current_lane_ptr_->get_ego_lateral_offset();
  if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW) {
    // lateral_left_lane:
    // 自车相对于左车道中心的偏移（通常为负值，因为自车在左车道右侧）
    // 条件：自车在当前车道已经偏左，接近左车道边界
    const double lateral_left_lane = left_lane_ptr_->get_ego_lateral_offset();
    return (lateral_origin_lane > -lateral_left_lane);
  } else if (lane_borrow_decider_output_.borrow_direction == RIGHT_BORROW) {
    // lateral_right_lane:
    // 自车相对于右车道中心的偏移（通常为正值，因为自车在右车道左侧）
    // 条件：自车在当前车道已经偏右，接近右车道边界
    const double lateral_right_lane = right_lane_ptr_->get_ego_lateral_offset();
    return (lateral_origin_lane < -lateral_right_lane);
  }
  return false;
}
// v2
bool LaneBorrowDecider::CheckLeadObs() {
  const auto& lat_obstacle_decision = session_->planning_context()
                                          .lateral_obstacle_decider_output()
                                          .lat_obstacle_decision;
  const auto& obstacle_map = current_reference_path_ptr_->get_obstacles_map();
  const auto lat_obs_iter = lat_obstacle_decision.find(front_id_);
  if (lat_obs_iter != lat_obstacle_decision.end() &&
      (lat_obs_iter->second != LatObstacleDecisionType::IGNORE &&
       lat_obs_iter->second != LatObstacleDecisionType::FOLLOW)) {
    return true;
  }
  if (static_blocked_obj_id_vec_[0] == front_id_) {
    // 如果借了第一个横向不能避让的障碍物，则返回true
    return true;
  } else {
    // 如果没借第一个不能避让的障碍物，则需要加入
    // CheckBlockedBorrowObstaclesByTrajectory进行idm轨迹判断，
    // 而不是直接返回false，不能借道，因为存在借左侧跟着右侧的场景
    auto it = obstacle_map.find(front_id_);
    if (it != obstacle_map.end()) {
      const auto& obstacle = it->second;
      if (nearest_no_borrow_obstacle_ == nullptr ||
          obstacle->frenet_obstacle_boundary().s_start <
              nearest_no_borrow_obstacle_->frenet_obstacle_boundary().s_start) {
        nearest_no_borrow_obstacle_ = obstacle;
      }
    }
    return true;
    // lane_borrow_decider_output_.lane_borrow_failed_reason =
    //     FRONT_OBS_NOT_BORROWING;
    // lane_borrow_decider_output_.failed_obs_id = front_id_;
    // return false;
  }
}
bool LaneBorrowDecider::IsNeedResetObserve(LaneBorrowFailedReason reason) {
  static const std::vector<LaneBorrowFailedReason> fail_reasons = {
      NO_PASSABLE_OBSTACLE, LANE_TYPE_CHECK_FAILED, LANE_CHANGE_STATE,
      CLOSE_TO_JUNCTION,    CENTER_OBSTACLE,        CURRENT_LANE_LOSS};
  return std::find(fail_reasons.begin(), fail_reasons.end(), reason) !=
         fail_reasons.end();
}
Box2d LaneBorrowDecider::PredictBoxPosition(const agent::Agent* agent,
                                            double delta_t) {
  double speed = agent->speed();
  double heading_rad = agent->theta();
  Vec2d pos(agent->x(), agent->y());
  double dx = speed * std::cos(heading_rad) * delta_t;
  double dy = speed * std::sin(heading_rad) * delta_t;
  Vec2d pred_pos = pos + Vec2d(dx, dy);
  planning::planning_math::Box2d pred_box(pred_pos, heading_rad,
                                          agent->length(), agent->width());
  return pred_box;
}
FrenetObstacleBoundary LaneBorrowDecider::GetSLboundaryFromAgent(
    const Box2d& obs_box) {
  std::vector<planning_math::Vec2d> obs_corners;
  obs_corners = obs_box.GetAllCorners();
  std::vector<double> agent_sl_boundary(4);
  FrenetObstacleBoundary sl_bd;
  sl_bd.l_start = std::numeric_limits<double>::max();
  sl_bd.l_end = std::numeric_limits<double>::lowest();
  sl_bd.s_start = std::numeric_limits<double>::max();
  sl_bd.s_end = std::numeric_limits<double>::lowest();
  const auto& current_frenet_coord =
      current_reference_path_ptr_->get_frenet_coord();
  for (size_t i = 0; i < obs_corners.size(); ++i) {
    double project_s = 0.0, project_l = 0.0;
    current_frenet_coord->XYToSL(obs_corners[i].x(), obs_corners[i].y(),
                                 &project_s,
                                 &project_l);  // 这是投影在路径上的 障碍物角点
    sl_bd.l_start = std::fmin(sl_bd.l_start, project_l);  // l right
    sl_bd.l_end = std::fmax(sl_bd.l_end, project_l);      // l left
    sl_bd.s_start = std::fmin(sl_bd.s_start, project_s);  // s start
    sl_bd.s_end = std::fmax(sl_bd.s_end, project_s);      // s end
  }
  return sl_bd;
}
void LaneBorrowDecider::SendObserveToLatFlag() {
  if (static_blocked_obj_id_vec_.empty()) {
    lat_flag_map_.clear();
    lane_borrow_decider_output_.lat_flag_map.clear();
    return;
  }
  for (const auto& id : static_blocked_obj_id_vec_) {
    if (lat_flag_map_.empty() ||
        lat_flag_map_.find(id) == lat_flag_map_.end()) {  // add
      lat_flag_map_[id] = 0;                              // fist appear
    }
    lat_flag_map_[id] += 1;
    lat_flag_map_[id] = std::min(50, lat_flag_map_[id]);
    // delete disappear obs
    for (auto it = lat_flag_map_.begin(); it != lat_flag_map_.end();) {
      if (std::find(static_blocked_obj_id_vec_.begin(),
                    static_blocked_obj_id_vec_.end(),
                    it->first) == static_blocked_obj_id_vec_.end()) {
        it = lat_flag_map_.erase(it);
      } else {
        ++it;
      }
    }
  }
  lane_borrow_decider_output_.lat_flag_map = lat_flag_map_;
  return;
}
void LaneBorrowDecider::LogDebugInfo() {
  auto lane_borrow_pb_info = DebugInfoManager::GetInstance()
                                 .GetDebugInfoPb()
                                 ->mutable_lane_borrow_decider_info();
  lane_borrow_pb_info->set_lane_borrow_failed_reason(
      static_cast<int>(lane_borrow_decider_output_.lane_borrow_failed_reason));
  auto current_reference_path = session_->environmental_model()
                                    .get_reference_path_manager()
                                    ->get_reference_path_by_current_lane();

  const auto& current_frenet_coord = current_reference_path->get_frenet_coord();

  Point2D front_left_corner, front_right_corner, back_right_corner,
      back_left_corner;
  current_frenet_coord->SLToXY(obs_end_s_, obs_left_l_, &front_left_corner.x,
                               &front_left_corner.y);
  current_frenet_coord->SLToXY(obs_end_s_, obs_right_l_, &front_right_corner.x,
                               &front_right_corner.y);
  current_frenet_coord->SLToXY(obs_start_s_, obs_right_l_, &back_right_corner.x,
                               &back_right_corner.y);
  current_frenet_coord->SLToXY(obs_start_s_, obs_left_l_, &back_left_corner.x,
                               &back_left_corner.y);
  lane_borrow_pb_info->mutable_block_obs_area()
      ->mutable_front_left_corner()
      ->set_x(front_left_corner.x);
  lane_borrow_pb_info->mutable_block_obs_area()
      ->mutable_front_left_corner()
      ->set_y(front_left_corner.y);

  lane_borrow_pb_info->mutable_block_obs_area()
      ->mutable_front_right_corner()
      ->set_x(front_right_corner.x);
  lane_borrow_pb_info->mutable_block_obs_area()
      ->mutable_front_right_corner()
      ->set_y(front_right_corner.y);

  lane_borrow_pb_info->mutable_block_obs_area()
      ->mutable_back_left_corner()
      ->set_x(back_left_corner.x);
  lane_borrow_pb_info->mutable_block_obs_area()
      ->mutable_back_left_corner()
      ->set_y(back_left_corner.y);

  lane_borrow_pb_info->mutable_block_obs_area()
      ->mutable_back_right_corner()
      ->set_x(back_right_corner.x);
  lane_borrow_pb_info->mutable_block_obs_area()
      ->mutable_back_right_corner()
      ->set_y(back_right_corner.y);
  lane_borrow_pb_info->mutable_block_obs_area()->set_obs_left_l(obs_left_l_);
  lane_borrow_pb_info->mutable_block_obs_area()->set_obs_right_l(obs_right_l_);
  lane_borrow_pb_info->mutable_block_obs_area()->set_obs_start_s(obs_start_s_);
  lane_borrow_pb_info->mutable_block_obs_area()->set_obs_end_s(obs_end_s_);
  lane_borrow_pb_info->set_safe_left_borrow(left_borrow_);
  lane_borrow_pb_info->set_safe_right_borrow(right_borrow_);
  lane_borrow_pb_info->set_front_id(front_id_);
  lane_borrow_pb_info->set_failed_obs_id(
      lane_borrow_decider_output_.failed_obs_id);

  lane_borrow_pb_info->set_lane_borrow_decider_status(
      static_cast<int>(lane_borrow_status_));
  lane_borrow_pb_info->set_dp_observe_frame_num(observe_path_frame_num_);

  lane_borrow_pb_info->mutable_static_blocked_obj_id_vec()->Clear();
  for (auto static_obs_id : static_blocked_obj_id_vec_) {
    lane_borrow_pb_info->mutable_static_blocked_obj_id_vec()->Add(
        static_obs_id);
  }

  lane_borrow_pb_info->set_intersection_state(intersection_state_);

#ifdef ENABLE_PROTO_LOG
  const auto& lat_obstacle_decision = session_->planning_context()
                                          .lateral_obstacle_decider_output()
                                          .lat_obstacle_decision;
  auto& planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto environment_model_debug_info =
      planning_debug_data->mutable_environment_model_info();
  auto obstacles = environment_model_debug_info->mutable_obstacle();
  for (const auto& [obstacle_id, decision] : lat_obstacle_decision) {
    for (size_t i = 0; i < environment_model_debug_info->obstacle_size(); ++i) {
      auto obs = obstacles->Mutable(i);
      if (obs->id() == obstacle_id) {
        obs->clear_lat_decision();
        obs->set_lat_decision(static_cast<uint32_t>(decision));
        break;
      }
    }
  }
#endif
}

bool LaneBorrowDecider::CheckVirtualLaneSuppressBorrow() {
  // 借道已经触发（非kNoLaneBorrow），则不抑制
  if (lane_borrow_status_ != kNoLaneBorrow) {
    return true;
  }
  // 借道方向为NO_BORROW时无需检查
  if (lane_borrow_decider_output_.borrow_direction == NO_BORROW) {
    return true;
  }
  // 获取借道车道指针
  std::shared_ptr<VirtualLane> borrow_lane_ptr = nullptr;
  if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW) {
    borrow_lane_ptr = left_lane_ptr_;
  } else if (lane_borrow_decider_output_.borrow_direction == RIGHT_BORROW) {
    borrow_lane_ptr = right_lane_ptr_;
  }
  if (borrow_lane_ptr == nullptr) {
    return true;
  }
  // 模仿left_lane_boundary_type_的计算方式，获取借道车道在借道方向侧的边界线类型
  const auto& borrow_lane_points = borrow_lane_ptr->lane_points();
  for (size_t i = 0; i < borrow_lane_points.size(); ++i) {
    const auto& lane_point = borrow_lane_points[i];
    if (lane_point.s < ego_sl_state_.s() + kLaneLineSegmentLength) {
      continue;
    }
    if (lane_point.s > obs_end_s_ + kBackNeededDistance) {
      break;
    }
    iflyauto::LaneBoundaryType borrow_side_boundary_type =
        iflyauto::LaneBoundaryType_MARKING_UNKNOWN;
    if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW) {
      // 向左借道，看借道车道的右边界线（即与自车车道相邻的边界）
      borrow_side_boundary_type = lane_point.right_lane_border_type;
    } else if (lane_borrow_decider_output_.borrow_direction == RIGHT_BORROW) {
      // 向右借道，看借道车道的左边界线（即与自车车道相邻的边界）
      borrow_side_boundary_type = lane_point.left_lane_border_type;
    }
    if (borrow_side_boundary_type ==
        iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
      return false;
    }
  }
  return true;
}

bool LaneBorrowDecider::CheckObserveTime() {
  // 检测每个障碍物的观测时间，如果观测时间不足，则删除该障碍物
  // 如果观测时间足够，则保留该障碍物

  // 增量更新观测帧数，避免重建整个 map
  std::unordered_set<int> current_obs_set(static_blocked_obj_id_vec_.begin(),
                                          static_blocked_obj_id_vec_.end());
  for (auto it = obs_observe_frame_map_.begin();
       it != obs_observe_frame_map_.end();) {
    if (current_obs_set.count(it->first)) {
      ++it->second;
      ++it;
    } else {
      it = obs_observe_frame_map_.erase(it);
    }
  }
  for (const auto& obs_id : static_blocked_obj_id_vec_) {
    if (obs_observe_frame_map_.find(obs_id) == obs_observe_frame_map_.end()) {
      obs_observe_frame_map_[obs_id] = 1;
    }
  }

  // 原地过滤，避免复制
  std::vector<int> qualified_ids;
  std::vector<std::shared_ptr<FrenetObstacle>> qualified_obstacles;
  qualified_ids.reserve(static_blocked_obstacles_.size());
  qualified_obstacles.reserve(static_blocked_obstacles_.size());

  for (const auto& obstacle : static_blocked_obstacles_) {
    const int id = obstacle->obstacle()->id();
    if (obs_observe_frame_map_[id] >= config_.observe_frames) {
      qualified_ids.push_back(id);
      qualified_obstacles.push_back(obstacle);
    }
  }

  // 使用 unordered_set 加速查找，避免 O(n²) 复杂度
  std::unordered_set<int> qualified_ids_set(qualified_ids.begin(),
                                            qualified_ids.end());

  double min_removed_obs_s_dist = std::numeric_limits<double>::max();
  if (nearest_no_borrow_obstacle_ != nullptr) {
    min_removed_obs_s_dist =
        nearest_no_borrow_obstacle_->frenet_obstacle_boundary().s_start -
        ego_frenet_boundary_.s_end;
  }

  for (const auto& obstacle : static_blocked_obstacles_) {
    const int id = obstacle->obstacle()->id();
    if (qualified_ids_set.count(id) == 0) {
      double obs_s_dist = obstacle->frenet_obstacle_boundary().s_start -
                          ego_frenet_boundary_.s_end;
      if (obs_s_dist > 0.0 && obs_s_dist < min_removed_obs_s_dist) {
        min_removed_obs_s_dist = obs_s_dist;
        nearest_no_borrow_obstacle_ = obstacle;
      }
    }
  }

  static_blocked_obj_id_vec_ = std::move(qualified_ids);
  static_blocked_obstacles_ = std::move(qualified_obstacles);

  return !static_blocked_obj_id_vec_.empty();
}

bool LaneBorrowDecider::CheckSpatioTemporalPlanner() {
  constexpr double kDistanceThresholdApproachToStopline = 10.0;
  constexpr int kEgoInIntersectionCount = 3;
  constexpr int kVirtualAreaContinuousThreshold = 5;

  const auto& reference_path = session_->planning_context()
                                   .lane_change_decider_output()
                                   .coarse_planning_info.reference_path;
  double init_vel =
      reference_path->get_frenet_ego_state().planning_init_point().v;
  std::vector<double> xp_vel{4.167, 16.667};
  std::vector<double> fp_length{15.0, 25.0};
  double min_virtual_length = interp(init_vel, xp_vel, fp_length);
  double preview_length = std::max(std::min(init_vel * 5.0, 90.0), 20.0);
  double virtual_length = 0.0;
  double dist_to_virtual_start = 100.0;
  // bool is_in_virtual_area = true;
  bool is_in_virtual_area = reference_path->IsExistValidVirtualLaneAheadEgo(
      preview_length, min_virtual_length, virtual_length,
      dist_to_virtual_start);
  if (is_in_virtual_area) {
    virtual_area_count_ = 1;
  } else if (virtual_area_count_ > 0 &&
             virtual_area_count_ < kVirtualAreaContinuousThreshold) {
    ++virtual_area_count_;
  } else {
    virtual_area_count_ = 0;
  }

  const auto lc_state = session_->planning_context()
                            .lane_change_decider_output()
                            .coarse_planning_info.target_state;
  const auto& construction_scene_output = session_->environmental_model()
                                              .get_construction_scene_manager()
                                              ->get_construction_scene_output();
  const auto& tfl_decider =
      session_->planning_context().traffic_light_decider_output();
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
      (intersection_state == common::IntersectionState::IN_INTERSECTION ||
       distance_to_stopline <= kDistanceThresholdApproachToStopline) &&
      (virtual_area_count_ > 0);

  bool is_small_intersection = false;
  // bool is_small_intersection = tfl_decider.is_small_front_intersection &&
  //     distance_to_crosswalk <= kDistanceThresholdApproachToCrosswalk;
  if (current_intersection_state) {
    spatio_temporal_planner_intersection_count_ = kEgoInIntersectionCount;
  } else {
    spatio_temporal_planner_intersection_count_ =
        std::max(spatio_temporal_planner_intersection_count_ - 1, 0);
  }

  if (!config_.enable_use_spatio_temporal_planning) {
    return false;
  }

  if (lc_state != kLaneKeeping) {
    return false;
  }

  if (!(spatio_temporal_planner_intersection_count_ > 0 &&
        !is_small_intersection) &&
      !construction_scene_output.enable_construction_passage) {
    return false;
  }

  return true;
}

void LaneBorrowDecider::SendHMIData() {
  lane_borrow_hmi_speed_hysteresis_.SetIsValidByValue(ego_speed_);
  bool is_high_speed = lane_borrow_hmi_speed_hysteresis_.IsValid();
  if (!is_high_speed) {
    return;
  }
  // 计算自车到借道方向车道边界的距离
  const auto current_ref_path = session_->environmental_model()
                                    .get_reference_path_manager()
                                    ->get_reference_path_by_current_lane();
  double ego_dist_to_boundary = std::numeric_limits<double>::max();
  if (current_ref_path != nullptr) {
    const auto& ego_frenet_boundary =
        current_ref_path->get_ego_frenet_boundary();
    double ego_s = current_ref_path->get_frenet_ego_state().s();
    ReferencePathPoint ref_point;
    if (current_ref_path->get_reference_point_by_lon(ego_s, ref_point)) {
      if (lane_borrow_decider_output_.borrow_direction ==
          BorrowDirection::LEFT_BORROW) {
        ego_dist_to_boundary =
            ref_point.distance_to_left_lane_border - ego_frenet_boundary.l_end;
      } else if (lane_borrow_decider_output_.borrow_direction ==
                 BorrowDirection::RIGHT_BORROW) {
        ego_dist_to_boundary = ref_point.distance_to_right_lane_border +
                               ego_frenet_boundary.l_start;
      }
    }
  }
  lane_borrow_hmi_boundary_dist_hysteresis_.SetIsValidByValue(
      -ego_dist_to_boundary);
  bool close_to_boundary = lane_borrow_hmi_boundary_dist_hysteresis_.IsValid();
  if (!close_to_boundary) {
    return;
  }
  auto ad_info = &(session_->mutable_planning_context()
                       ->mutable_planning_hmi_info()
                       ->ad_info);
  lane_borrow_decider_output_.takeover_prompt = false;
  // 开始绕行信号
  // 连续帧判断开始绕行
  nudging_prompt_ = false;
  takeover_prompt_ = false;

  if (lane_borrow_status_ != kNoLaneBorrow) {
    start_frame_++;
    start_frame_ = std::min(150, start_frame_);
  } else {
    start_frame_ = 0;
  }

  // 结果
  if (start_frame_ > 3) {
    nudging_prompt_ = true;
    takeover_prompt_ = false;
  }
  if (lane_borrow_status_ == kNoLaneBorrow && nudging_prompt_ &&
      ego_speed_ < 0.5) {
    takeover_prompt_ = true;
    nudging_prompt_ = false;
  }
  lane_borrow_decider_output_.takeover_prompt = takeover_prompt_;
  //绕行提示
  ad_info->start_nudging = nudging_prompt_;
  //方向
  ad_info->borrow_direction =
      lane_borrow_decider_output_.borrow_direction == LEFT_BORROW
          ? iflyauto::LaneBorrowDirection::BORROW_LEFT
          : iflyauto::LaneBorrowDirection::BORROW_RIGHT;
  //车道线
  if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW) {
    ad_info->borrow_lane_type = lane_borrow_decider_output_.is_left_solid
                                    ? iflyauto::LaneBorrowLaneType::SOLID_LINE
                                    : iflyauto::LaneBorrowLaneType::DASHED_LINE;
  } else if (lane_borrow_decider_output_.borrow_direction == RIGHT_BORROW) {
    ad_info->borrow_lane_type = lane_borrow_decider_output_.is_right_solid
                                    ? iflyauto::LaneBorrowLaneType::SOLID_LINE
                                    : iflyauto::LaneBorrowLaneType::DASHED_LINE;
  } else {
    ad_info->borrow_lane_type = iflyauto::LaneBorrowLaneType::LANE_NONE;
  }
}
}  // namespace lane_borrow_deciderV3
}  // namespace planning