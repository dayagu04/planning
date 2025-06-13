#include "lane_borrow_deciderv2.h"
#include <Eigen/src/Core/Matrix.h>
#include <math.h>

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
#include "log.h"
#include "math/polygon2d.h"
#include "obstacle_manager.h"
#include "planning_context.h"
#include "pose2d.h"
#include "session.h"
#include "task_interface/lane_borrow_decider_output.h"
#include "tracked_object.h"
#include "utils/cartesian_coordinate_system.h"
#include "utils/path_point.h"
#include "utils/pose2d_utils.h"

namespace {
constexpr double kMinDisToStopLine = 50.0;
constexpr double kMinDisToCrossWalk = 50.0;
constexpr double kMinDisToTrafficLight = 120.0;
constexpr double kInfDisToTrafficLight = 10000.0;
constexpr double kLatPassableBuffer = 0.8;
constexpr double kObsLatBuffer = 0.3;
constexpr double kObsSpeedRatio = 3.5;
constexpr double kForwardOtherObsDistance = 20.0;
constexpr double kObsLonDisBuffer = 2.0;
constexpr double kMaxCentricOffset = 0.75;
constexpr double kBackNeededDistance = 5.0;
constexpr double kMaxConcernCollisionTime = 5.0;
constexpr double kPreCentricOffsetHigh = 0.75;
constexpr double kPreCentricOffsetLow = 0.45;
};  // namespace

namespace planning {
namespace lane_borrow_deciderV2 {

bool LaneBorrowDecider::Execute() {
  UpdateToDP();
  dp_path_decider_->LogDebugInfo();
  LogDebugInfo();
  return true;
}

bool LaneBorrowDecider::ProcessAllEnvInfos() {
  if (!ProcessEnvInfos()) {
    return false;
  }
  if (!dp_path_decider_->ProcessEnvInfos()) {
    return false;
  }
  LaneTypeDistanceInfo();
  return true;
}

bool LaneBorrowDecider::ProcessEnvInfos() {
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  current_lane_ptr_ = virtual_lane_manager->get_current_lane();
  current_reference_path_ptr_ = current_lane_ptr_->get_reference_path();
  left_lane_ptr_ = virtual_lane_manager->get_left_lane();
  right_lane_ptr_ = virtual_lane_manager->get_right_lane();

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
  if (current_lane_ptr_ == nullptr || current_reference_path_ptr_ == nullptr) {
    lane_borrow_pb_info->set_dis_to_traffic_lights(dis_to_traffic_lights_);
    LOG_ERROR("No current_lane_ptr_ or current_reference_path_ptr!");
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
  if (lane_change_state_ != kLaneKeeping) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = LANE_CHANGE_STATE;
    LOG_ERROR("It has lane change state!");
    return false;
  }

  intersection_state_ = virtual_lane_manager->GetIntersectionState();
  // if (intersection_state_ ==
  //     planning::common::IntersectionState::APPROACH_INTERSECTION) {
  //   return false;
  // }

  distance_to_stop_line_ = virtual_lane_manager->GetEgoDistanceToStopline();
  distance_to_cross_walk_ = virtual_lane_manager->GetEgoDistanceToCrosswalk();

  return true;
}

bool LaneBorrowDecider::LaneBorrowPreCheck() {
  if (!SelectStaticBlockingObstcales()) {
    return false;
  }

  if (!UpdateLaneBorrowDirection()) {
    return false;
  }

  // if(!CheckDynamicCutin()){
  //   return false;
  // }
  if (!ObstacleDecision()) {
    return false;
  }
  // to wrap  👇
  if (lane_borrow_status_ == kNoLaneBorrow && (!is_facility_) &&
      (intersection_state_ !=
       planning::common::IntersectionState::IN_INTERSECTION)) {
    if ((forward_solid_start_dis_ <
             obs_end_s_ - ego_frenet_boundary_.s_start + kBackNeededDistance &&
         forward_solid_start_dis_ > 0) ||
        (distance_to_cross_walk_ < kMinDisToCrossWalk &&
         distance_to_cross_walk_ > 0.0) ||
        (distance_to_stop_line_ < kMinDisToStopLine &&
         distance_to_stop_line_ > 0.0) ||
        (dis_to_traffic_lights_ < kMinDisToTrafficLight &&
         dis_to_traffic_lights_ > 0.0) ||
        (intersection_state_ ==
         planning::common::IntersectionState::APPROACH_INTERSECTION)) {
      LOG_DEBUG("Ego car is near junction");
      lane_borrow_decider_output_.lane_borrow_failed_reason = CLOSE_TO_JUNCTION;
      return false;
    }
  }
  // to  be wrapped 👆

  observe_frame_num_++;
  if (observe_frame_num_ < config_.observe_frames) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        OBSERVE_TIME_CHECK_FAILED;
    return false;
  }

  if (lane_borrow_status_ != kNoLaneBorrow) {
    if (!CheckLaneBorrowDircetion()) {
      return false;
    }
  }

  if (lane_borrow_status_ != kLaneBorrowCrossing) {
    if (!CheckBackWardObs()) {
      return false;
    }
  }

  // if (lane_borrow_status_ != kNoLaneBorrow) {
  //   if (!is_first_frame_to_lane_borrow_) {
  //     if (!RunDP()) {
  //       return false;
  //     }
  //   } else {
  //     is_first_frame_to_lane_borrow_ = false;
  //   }
  // }

  return true;
}
bool LaneBorrowDecider::DPDecision() {
  if (lane_borrow_status_ != kNoLaneBorrow) {
    if (!is_first_frame_to_lane_borrow_) {
      if (!RunDP()) {
        return false;
      }
    } else {
      is_first_frame_to_lane_borrow_ = false;
    }
  }

  return true;
}
void LaneBorrowDecider::UpdateToDP() {
  if (!ProcessEnvInfos()) {
    lane_borrow_decider_output_.is_in_lane_borrow_status = false;
    static_blocked_obj_id_vec_.clear();
    lane_borrow_decider_output_.blocked_obs_id = static_blocked_obj_id_vec_;
    lane_borrow_decider_output_.borrow_direction = NO_BORROW;
    lane_borrow_status_ = LaneBorrowStatus::kNoLaneBorrow;
    lane_borrow_decider_output_.lane_borrow_state = lane_borrow_status_;
    session_->mutable_planning_context()->mutable_lane_borrow_decider_output() =
    lane_borrow_decider_output_;  // 输出赋值
    return;
  }

  switch (lane_borrow_status_) {
    case LaneBorrowStatus::kNoLaneBorrow: {
      if (CheckIfNoBorrowToDPLaneBorrowDriving()) {
        if (RunDP()) {
          lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowDriving;
          is_first_frame_to_lane_borrow_ = true;
        }
      }
      break;
    }

    case LaneBorrowStatus::kLaneBorrowDriving: {
      if (!CheckLaneBorrowCondition()) {
        lane_borrow_status_ = LaneBorrowStatus::kNoLaneBorrow;
      } else if (CheckIfDPLaneBorrowToDPLaneBorrowCrossing()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowCrossing;
      }
      break;
    }

    case LaneBorrowStatus::kLaneBorrowCrossing: {
      if (CheckIfDPLaneBorrowCrossingToNoBorrow()) {
        lane_borrow_status_ = LaneBorrowStatus::kNoLaneBorrow;
      } else if (IsDPSafeForBackOriginLane()) {
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
    lane_borrow_decider_output_.lane_borrow_failed_reason = NONE_FAILED_REASON;
    lane_borrow_decider_output_.blocked_obs_id = static_blocked_obj_id_vec_;

  } else {
    dp_path_decider_->ClearDPInfo();
    lane_borrow_decider_output_.is_in_lane_borrow_status = false;
    static_blocked_obj_id_vec_.clear();
    lane_borrow_decider_output_.blocked_obs_id = static_blocked_obj_id_vec_;
    lane_borrow_decider_output_.borrow_direction = NO_BORROW;
    // if(lane_borrow_decider_output_.lane_borrow_failed_reason !=
    // OBSERVE_TIME_CHECK_FAILED){
    //   ClearLaneBorrowStatus();
    // }
  }

  session_->mutable_planning_context()->mutable_lane_borrow_decider_output() =
      lane_borrow_decider_output_;  // 输出赋值

  return;
}

bool LaneBorrowDecider::RunDP() {
  // dp_path_decider_->Execute();
  if (!dp_path_decider_->ProcessEnvInfos()) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = CURRENT_LANE_LOSS;
    lane_borrow_decider_output_.is_in_lane_borrow_status = false;
    return false;
  }
  dp_path_decider_->SetSampleParams(lane_borrow_status_);
  dp_path_decider_->SetDPCostParams(lane_borrow_status_);
  dp_path_decider_->SampleLanes(&lane_borrow_decider_output_);
  dp_path_decider_->DPSearchPath(lane_borrow_status_);
  if (!dp_path_decider_->FinedReferencePath() &&
      lane_borrow_status_ != LaneBorrowStatus::kNoLaneBorrow) {
    dp_observe_frame_num_++;
    if (dp_observe_frame_num_ < 5) {
      dp_path_decider_->LastFramePath();
      // lane_borrow_decider_output_.lane_borrow_failed_reason =
      // TMP_DP_SEARCH_FAILED;
    } else {
      lane_borrow_decider_output_.lane_borrow_failed_reason =
          TRIGGER_BUT_DP_SEARCH_FAILED;
      lane_borrow_decider_output_.is_in_lane_borrow_status = false;
      return false;
    }
  } else if (!dp_path_decider_->FinedReferencePath() &&
             lane_borrow_status_ == LaneBorrowStatus::kNoLaneBorrow) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        TRIGGER_BUT_DP_SEARCH_FAILED;
    lane_borrow_decider_output_.is_in_lane_borrow_status = false;
    return false;
  } else {
    dp_observe_frame_num_ = 0;
  }
  dp_path_decider_->CartSpline(&lane_borrow_decider_output_);
  return true;
}

// v2   0-1
bool LaneBorrowDecider::CheckIfNoBorrowToDPLaneBorrowDriving() {
  if (!CheckLaneBorrowCondition()) {
    return false;
  }
  return true;
}

void LaneBorrowDecider::ClearLaneBorrowStatus() {
  observe_frame_num_ = 0;
  dp_observe_frame_num_ = 0;
  left_borrow_ = false;
  right_borrow_ = false;
  obs_direction_map_.clear();
}

// v2 连续绕行反向   1-2
bool LaneBorrowDecider::CheckIfDPLaneBorrowToDPLaneBorrowCrossing() {
  if (CrossingPositionJudgment()) {
    return true;
  } else {
    return false;
  }
}

// v2 2-0
bool LaneBorrowDecider::CheckIfDPLaneBorrowCrossingToNoBorrow() {
  if (!CheckLaneBorrowCondition()) {  // 1-0
    return true;
  }
  return false;
}

// v2 2-3
bool LaneBorrowDecider::IsDPSafeForBackOriginLane() {
  //静态区域最后障碍物的s和自车当前是s判断，如果超过阈值则说明自车正在回自车道
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
  if (!RunDP()) {
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
  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  const double left_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  const double right_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  const auto& lat_obstacle_decision = session_->planning_context()
                                          .lateral_obstacle_decider_output()
                                          .lat_obstacle_decision;
  for (const auto& obstacle : obstacles) {
    const auto& id = obstacle->obstacle()->id();
    const auto& obs_type = obstacle->obstacle()->type();
    if (!obstacle->b_frenet_valid()) {
      continue;
    }

    if (obs_type == iflyauto::ObjectType::OBJECT_TYPE_PEDESTRIAN) {
      continue;
    }

    if (!(obstacle->obstacle()->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }

    if (obstacle->frenet_obstacle_boundary().s_start >
        ego_frenet_boundary_.s_end) {
      const auto lat_obs_iter = lat_obstacle_decision.find(id);
      if (lat_obs_iter != lat_obstacle_decision.end() &&
          lat_obs_iter->second != LatObstacleDecisionType::IGNORE) {
        continue;
      }
    }

    auto it = std::find(static_blocked_obj_id_vec_.begin(),
                        static_blocked_obj_id_vec_.end(), id);

    if (it != static_blocked_obj_id_vec_.end()) {
      continue;
    }
    const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
    if (frenet_obstacle_sl.s_start >
        ego_frenet_boundary_.s_end + kForwardOtherObsDistance) {
      continue;
    }
    if (frenet_obstacle_sl.l_start > left_width ||
        frenet_obstacle_sl.l_end < -right_width) {
      continue;
    }

    const double obs_v = obstacle->frenet_velocity_s();
    if (frenet_obstacle_sl.s_start > ego_frenet_boundary_.s_end) {
      // if (!obstacle->obstacle()->is_static()) {    //dp obs_speed selset
      //   continue;
      // }
      if (obs_v > 4.2) {  // dp  concern obs_v <     15kph
        continue;
      }

    } else {
      if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW) {
        if (frenet_obstacle_sl.l_end > ego_frenet_boundary_.l_start) {
          continue;
        }
        if (frenet_obstacle_sl.l_end < -right_width &&
            obstacle->obstacle()->is_static()) {
          continue;
        }
        if (frenet_obstacle_sl.l_end + kLatPassableBuffer < -right_width) {
          continue;
        }
        if (frenet_obstacle_sl.s_end + obs_v * kObsSpeedRatio <
            ego_frenet_boundary_.s_start) {
          continue;
        }

      } else {
        if (frenet_obstacle_sl.l_start < ego_frenet_boundary_.l_end) {
          continue;
        }
        if (frenet_obstacle_sl.l_start > left_width &&
            obstacle->obstacle()->is_static()) {
          continue;
        }
        if (frenet_obstacle_sl.l_start - kLatPassableBuffer > left_width) {
          continue;
        }
        if (frenet_obstacle_sl.s_end + obs_v * kObsSpeedRatio <
            ego_frenet_boundary_.s_start) {
          continue;
        }
      }
    }
    return true;
  }
  return false;
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
  if (lane_borrow_status_ != kNoLaneBorrow) {
    if (!is_first_frame_to_lane_borrow_) {
      if (!RunDP()) {
        return false;
      }
    } else {
      is_first_frame_to_lane_borrow_ = false;
    }
  }

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

  if (lane_borrow_status_ == kNoLaneBorrow && (!is_facility_) &&
      (intersection_state_ !=
       planning::common::IntersectionState::IN_INTERSECTION)) {
    if ((forward_solid_start_dis_ <
             obs_end_s_ - ego_frenet_boundary_.s_start + kBackNeededDistance &&
         forward_solid_start_dis_ > 0) ||
        (distance_to_cross_walk_ < kMinDisToCrossWalk &&
         distance_to_cross_walk_ > 0.0) ||
        (distance_to_stop_line_ < kMinDisToStopLine &&
         distance_to_stop_line_ > 0.0) ||
        (dis_to_traffic_lights_ < kMinDisToTrafficLight &&
         dis_to_traffic_lights_ > 0.0) ||
        (intersection_state_ ==
         planning::common::IntersectionState::APPROACH_INTERSECTION)) {
      LOG_DEBUG("Ego car is near junction");
      lane_borrow_decider_output_.lane_borrow_failed_reason = CLOSE_TO_JUNCTION;
      return false;
    }
  }

  observe_frame_num_++;
  if (observe_frame_num_ < config_.observe_frames) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        OBSERVE_TIME_CHECK_FAILED;
    return false;
  }

  if (lane_borrow_status_ != kNoLaneBorrow) {
    if (!CheckLaneBorrowDircetion()) {
      return false;
    }
  }else{
    lane_borrow_decider_output_.borrow_direction = bypass_direction_;
  }

  if (lane_borrow_status_ != kLaneBorrowCrossing) {
    if (!CheckBackWardObs()) {
      return false;
    }
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
void LaneBorrowDecider::LaneTypeDistanceInfo() {
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

// v2
// Determine the type of lane marking.
bool LaneBorrowDecider::IsLaneTypeDashedOrMixed(
    const iflyauto::LaneBoundaryType& type) {
  return type == iflyauto::LaneBoundaryType_MARKING_DASHED ||
         type == iflyauto::LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED ||
         type == iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED ||
         type == iflyauto::LaneBoundaryType_MARKING_VIRTUAL;
}
// v2
bool LaneBorrowDecider::UpdateLaneBorrowDirection() {
  left_borrow_ = true;
  right_borrow_ = true;

  double lane_line_length = 0.0;
  const auto& left_lane_boundarys = current_lane_ptr_->get_left_lane_boundary();
  const auto& right_lane_boundarys =
      current_lane_ptr_->get_right_lane_boundary();
  iflyauto::LaneBoundaryType left_lane_boundary_type;
  iflyauto::LaneBoundaryType right_lane_boundary_type;

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
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
      left_lane_boundary_type = lane_point.left_lane_border_type;
      right_lane_boundary_type = lane_point.right_lane_border_type;
      break;
    }
  }

  // If the lane marking is not left dashed/right solid or double dashed, return
  // False.
  // if is_facility_ or in intersection ignore lane type
  if (!is_facility_ &&
      (intersection_state_ !=
       planning::common::IntersectionState::IN_INTERSECTION) &&
      left_lane_boundary_type != iflyauto::LaneBoundaryType_MARKING_DASHED &&
      left_lane_boundary_type !=
          iflyauto::LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED &&
      left_lane_boundary_type !=
          iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED &&
      left_lane_boundary_type != iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
    left_borrow_ = false;
  }

  if (left_lane_ptr_ == nullptr) {
    left_borrow_ = false;
  }

  // todo: if left lane is reverse, then left_boorow is false
  if (!is_facility_ &&
      (intersection_state_ !=
       planning::common::IntersectionState::IN_INTERSECTION) &&
      right_lane_boundary_type != iflyauto::LaneBoundaryType_MARKING_DASHED &&
      right_lane_boundary_type !=
          iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED &&
      right_lane_boundary_type != iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
    right_borrow_ = false;
  }
  if (right_lane_ptr_ == nullptr) {
    right_borrow_ = false;
  }

  // todo: consider ego car near/in stop line or crosswalk area
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
  const auto& lat_obstacle_decision = session_->planning_context()
                                          .lateral_obstacle_decider_output()
                                          .lat_obstacle_decision;
  for (const auto& obstacle : obstacles) {
    int idx = obstacle->obstacle()->id();
    const auto& id = obstacle->obstacle()->id();
    const auto& obs_type = obstacle->obstacle()->type();
    if (!obstacle->b_frenet_valid()) {
      continue;
    }
    if (obs_type == iflyauto::ObjectType::OBJECT_TYPE_PEDESTRIAN) {
      continue;
    }
    if (!(obstacle->obstacle()->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }

    const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
    if (frenet_obstacle_sl.s_start > forward_obs_s ||
        frenet_obstacle_sl.s_end + kObsLonDisBuffer <
            ego_frenet_boundary_.s_start) {  // lon concern area
      continue;
    }
    const auto& vehicle_param =
        VehicleConfigurationContext::Instance()->get_vehicle_param();
    //  no lon overlap
    // if (frenet_obstacle_sl.s_start > ego_frenet_boundary_.s_end ||
    //     frenet_obstacle_sl.s_end < ego_frenet_boundary_.s_start) {
    if (frenet_obstacle_sl.s_start > ego_frenet_boundary_.s_end) {
      const auto lat_obs_iter = lat_obstacle_decision.find(id);
      if (lat_obs_iter != lat_obstacle_decision.end() &&
          lat_obs_iter->second != LatObstacleDecisionType::IGNORE) {
        continue;
      }
    } else {  // lon overlap
      auto it = std::find(last_static_blocked_obj_id_vec_.begin(),
                          last_static_blocked_obj_id_vec_.end(), id);
      if (it == last_static_blocked_obj_id_vec_.end()) {
        continue;
      }
    }
    // TODO: concern more scene
    if (frenet_obstacle_sl.l_end < -right_width ||
        frenet_obstacle_sl.l_start > left_width) {  // away from cur lane
      continue;
    } else {
      if (obstacle->frenet_velocity_s() > 4.2) {
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
  for(int i = static_blocked_obstacles_.size()-1;i >= 0; --i){
    // 不删除对向来车
    double yaw = static_blocked_obstacles_[i]->obstacle()->relative_heading_angle();
    if(std::fabs(yaw) > 2.0){
      continue;
    }
    int blocked_obs_id = static_blocked_obj_id_vec_[i];
    const auto& agent = agent_mgr->GetAgent(blocked_obs_id);
    if (agent == nullptr) {
      lane_borrow_decider_output_.lane_borrow_failed_reason = AGENT_MGR_FAILED;
      return false;
    }
    bool is_cut_in = false;
    bool is_cut_out = false;
    if (agent->speed() < 4.2) {
      CheckKeyObstaclesIntention(agent, is_cut_in, is_cut_out);
    }
    if (is_cut_in || is_cut_out) {
      static_blocked_obstacles_.erase(static_blocked_obstacles_.begin() + i);
      static_blocked_obj_id_vec_.erase(static_blocked_obj_id_vec_.begin() + i);
    }
  }
  if (static_blocked_obstacles_.empty() || static_blocked_obj_id_vec_.empty()) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = CUTINOUT_RISK;
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

        if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW &&
            obs_center_l - obs_sl.l > 0.5) {
          is_cut_out = true;
        } else if (lane_borrow_decider_output_.borrow_direction ==
                       RIGHT_BORROW &&
                   obs_center_l - obs_sl.l < -0.5) {
          is_cut_out = true;
        }

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
  const auto& first_obs_type = static_blocked_obstacles_[0]->obstacle()->type();
  if (valid_obstacle_types.find(first_obs_type) != valid_obstacle_types.end()) {
    is_facility_ = true;
  }

  const auto& front_obstacle_sl =
      static_blocked_obstacles_[0]->frenet_obstacle_boundary();
  const auto& id = static_blocked_obstacles_[0]->obstacle()->id();
  const double front_obs_center_l =
      0.5 * (front_obstacle_sl.l_start + front_obstacle_sl.l_end);
  lane_borrow_pb_info->set_front_obs_center(front_obs_center_l);
  BorrowDirection front_obs_bypass_direction =
      GetBypassDirection(front_obstacle_sl, id);
  if (lane_borrow_status_ == LaneBorrowStatus::kLaneBorrowCrossing) {
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
      right_borrow_ = false;  //
    } else if (front_obs_bypass_direction == RIGHT_BORROW && right_borrow_) {
      bypass_direction_ = RIGHT_BORROW;
      left_borrow_ = false;
    } else {
      bypass_direction_ = NO_BORROW;
      lane_borrow_decider_output_.lane_borrow_failed_reason = CENTER_OBSTACLE;
      return false;
    }

    for (const auto& obstacle : static_blocked_obstacles_) {
      const auto& id = obstacle->obstacle()->id();
      const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
      BorrowDirection obs_bypass_direction =
          GetBypassDirection(frenet_obstacle_sl, id);

      if (obs_bypass_direction == bypass_direction_) {
        obs_left_l_ = std::max(obs_left_l_, frenet_obstacle_sl.l_end);
        obs_right_l_ = std::min(obs_right_l_, frenet_obstacle_sl.l_start);
        obs_start_s_ = std::min(obs_start_s_, frenet_obstacle_sl.s_start);
        obs_end_s_ = std::max(obs_end_s_, frenet_obstacle_sl.s_end);
        static_blocked_obj_id_vec_.emplace_back(obstacle->obstacle()->id());
      } else {
        // too dense obstacles
        const double dist = frenet_obstacle_sl.s_start - obs_end_s_;
        if (dist < config_.dense_obstacle_dist) {
          lane_borrow_decider_output_.lane_borrow_failed_reason =
              CENTER_OBSTACLE;
          return false;
        }
        break;
      }
    }
  }

  if (obs_left_l_ <= obs_right_l_) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        NO_PASSABLE_OBSTACLE;
    return false;
  }
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
  const double obs_center_l =
      0.5 * (frenet_obstacle_sl.l_start + frenet_obstacle_sl.l_end);
  double scale = 1.0;
  if (lane_borrow_status_ != kNoLaneBorrow) {
    scale = 0.5;
  }
  if (std::fabs(obs_center_l) <= scale * kMaxCentricOffset) {
    if (obs_direction_map_[obs_id].second < config_.centric_obs_frames) {
      obs_direction_map_[obs_id].second += 1;
      return obs_direction_map_[obs_id].first;
    } else {
      obs_direction_map_[obs_id].first = NO_BORROW;
      return NO_BORROW;
    }
  } else if (obs_center_l < -scale * kMaxCentricOffset) {
    obs_direction_map_[obs_id].first = LEFT_BORROW;
    obs_direction_map_[obs_id].second = 0;
    return LEFT_BORROW;
  } else {
    obs_direction_map_[obs_id].first = RIGHT_BORROW;
    obs_direction_map_[obs_id].second = 0;
    return RIGHT_BORROW;
  }
}

// v2
bool LaneBorrowDecider::CheckLaneBorrowDircetion() {
  //拿静态区域的第一个obs_id 判断在path的左边还是右边
  // const auto& id = static_blocked_obj_id_vec_[0];
  const auto& front_obstacle_sl =
      static_blocked_obstacles_[0]->frenet_obstacle_boundary();
  const auto& dp_path = dp_path_decider_->refined_paths();
  double center_obs_l =
      (front_obstacle_sl.l_start + front_obstacle_sl.l_end) / 2;
  double center_obs_s =
      (front_obstacle_sl.s_start + front_obstacle_sl.s_end) / 2;

  auto it = std::lower_bound(
      dp_path.begin(), dp_path.end(), center_obs_s,
      [](const PathPoint& point, double s) { return point.s() < s; });
  bool isEndpoint = false;
  double dp_path_l = 0.0;
  if (it == dp_path.begin()) {
    isEndpoint = true;
    dp_path_l = dp_path.front().l();
  } else if (it == std::prev(dp_path.end())) {
    isEndpoint = true;
    dp_path_l = dp_path.back().l();
  }

  const auto& prev_point = *(it - 1);
  const auto& next_point = *it;
  if (!isEndpoint) {
    if (std::abs(prev_point.s() - center_obs_s) <
        std::abs(next_point.s() - center_obs_s)) {
      dp_path_l = prev_point.l();
    } else {
      dp_path_l = next_point.l();
    }
  }

  if (dp_path_l > center_obs_l) {
    dp_path_direction_ = LEFT_BORROW;
  } else if (dp_path_l < center_obs_l) {
    dp_path_direction_ = RIGHT_BORROW;
  } else {
    lane_borrow_decider_output_.lane_borrow_failed_reason = DP_NO_DIRECTION;
    return false;
  }

  if (dp_path_direction_ == bypass_direction_) {
    lane_borrow_decider_output_.borrow_direction = dp_path_direction_;
    return true;
  } else if (lane_borrow_status_ == kLaneBorrowCrossing &&
             dp_path_direction_ != bypass_direction_) {
    lane_borrow_decider_output_.borrow_direction = dp_path_direction_;
    return true;
  } else {
    lane_borrow_decider_output_.borrow_direction = NO_BORROW;
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        BORROWDIRECTION_DIFFERENT;
    return false;
  }
}

// v2
bool LaneBorrowDecider::CrossingPositionJudgment() {
  const auto& current_frenet_coord = current_lane_ptr_->get_lane_frenet_coord();
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double heading_angle = heading_angle_;

  double ego_x = ego_xy_.x;
  double ego_y = ego_xy_.y;

  // Get the corner points' Cartesian coordinates while traveling straight
  Point2D corner_front_left_point_xy(vehicle_param.front_edge_to_rear_axle,
                                     vehicle_param.width * 0.5);
  Point2D corner_front_right_point_xy(vehicle_param.front_edge_to_rear_axle,
                                      -vehicle_param.width * 0.5);
  Point2D corner_rear_left_point_xy(-vehicle_param.rear_edge_to_rear_axle,
                                    vehicle_param.width * 0.5);
  Point2D corner_rear_right_point_xy(-vehicle_param.rear_edge_to_rear_axle,
                                     -vehicle_param.width * 0.5);

  SLPoint corner_front_left, corner_rear_left, corner_front_right,
      corner_rear_right;

  if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW) {
    Point2D corner_front_left_xy = CartesianRotation(
        corner_front_left_point_xy, heading_angle, ego_x, ego_y);
    Point2D corner_rear_left_xy = CartesianRotation(
        corner_rear_left_point_xy, heading_angle, ego_x, ego_y);

    // Back to the SL coordinate system and compare with the lane lines.
    current_frenet_coord->XYToSL(corner_front_left_xy.x, corner_front_left_xy.y,
                                 &corner_front_left.s, &corner_front_left.l);
    current_frenet_coord->XYToSL(corner_rear_left_xy.x, corner_rear_left_xy.y,
                                 &corner_rear_left.s, &corner_rear_left.l);

    const double current_front_left_lane_l =
        current_lane_ptr_->width_by_s(corner_front_left.s) * 0.5;
    const double current_rear_left_lane_l =
        current_lane_ptr_->width_by_s(corner_rear_left.s) * 0.5;
    if (corner_front_left.l > current_front_left_lane_l &&
        corner_rear_left.l > current_rear_left_lane_l) {
      return true;
    }
    return false;

  } else if (lane_borrow_decider_output_.borrow_direction == RIGHT_BORROW) {
    Point2D corner_front_right_xy = CartesianRotation(
        corner_front_right_point_xy, heading_angle, ego_x, ego_y);
    Point2D corner_rear_right_xy = CartesianRotation(
        corner_rear_right_point_xy, heading_angle, ego_x, ego_y);

    current_frenet_coord->XYToSL(corner_front_right_xy.x,
                                 corner_front_right_xy.y, &corner_front_right.s,
                                 &corner_front_right.l);
    current_frenet_coord->XYToSL(corner_rear_right_xy.x, corner_rear_right_xy.y,
                                 &corner_rear_right.s, &corner_rear_right.l);

    const double current_front_right_lane_l =
        current_lane_ptr_->width_by_s(corner_front_right.s) * 0.5;
    const double current_rear_right_lane_l =
        current_lane_ptr_->width_by_s(corner_rear_right.s) * 0.5;

    if (corner_front_right.l < -current_front_right_lane_l &&
        corner_rear_right.l < -current_rear_right_lane_l) {
      return true;
    }
    return false;
  }
  return false;
}

Point2D LaneBorrowDecider::CartesianRotation(const Point2D& Cartesian_point,
                                             double heading_angle, double ego_x,
                                             double ego_y) {
  double cos_theta = cos(heading_angle);
  double sin_theta = sin(heading_angle);
  return {
      Cartesian_point.x * cos_theta - Cartesian_point.y * sin_theta + ego_x,
      Cartesian_point.x * sin_theta + Cartesian_point.y * cos_theta + ego_y};
};

bool LaneBorrowDecider::CheckBackWardObs() {
  double left_right_bounds_l = 0.0;
  double left_left_bounds_l = 0.0;

  double right_right_bounds_l = 0.0;
  double right_left_bounds_l = 0.0;

  double neighbor_left_width = 1.75;
  double neighbor_right_width = 1.75;

  const double current_left_lane_width = current_lane_ptr_->width() * 0.5;
  const double current_right_lane_width = current_lane_ptr_->width() * 0.5;
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW){
        left_right_bounds_l = current_left_lane_width;
        const double neighbor_width =
            left_lane_ptr_->width(vehicle_param.front_edge_to_rear_axle);
        // neighbor lane width
        neighbor_left_width = neighbor_width * 0.5;
        neighbor_right_width = neighbor_width * 0.5;

        // Calculate the total width that can be borrowed from the left lane
        left_left_bounds_l =
            current_left_lane_width + neighbor_right_width + neighbor_left_width;
      }else if (lane_borrow_decider_output_.borrow_direction == RIGHT_BORROW) {
        right_left_bounds_l = -current_right_lane_width;
        const double neighbor_width =
            right_lane_ptr_->width(vehicle_param.front_edge_to_rear_axle);
        neighbor_left_width = neighbor_width * 0.5;
        neighbor_right_width = neighbor_width * 0.5;
        right_right_bounds_l =
            -current_right_lane_width - neighbor_left_width - neighbor_right_width;
      }

  for (const auto& obstacle : obstacles) {
    int idx = obstacle->obstacle()->id();
    const auto& id = obstacle->obstacle()->id();
    const auto& obs_type = obstacle->obstacle()->type();
    if (!obstacle->b_frenet_valid()) {
      continue;
    }
    if (obs_type == iflyauto::ObjectType::OBJECT_TYPE_PEDESTRIAN) {
      continue;
    }
    if (!(obstacle->obstacle()->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }

    const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
    if (frenet_obstacle_sl.s_end < ego_frenet_boundary_.s_start) {
      if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW){
        if(frenet_obstacle_sl.l_start > left_left_bounds_l || frenet_obstacle_sl.l_end < left_right_bounds_l){
          continue;
        }
      }else if (lane_borrow_decider_output_.borrow_direction == RIGHT_BORROW) {
        if(frenet_obstacle_sl.l_end < right_right_bounds_l || frenet_obstacle_sl.l_start > right_left_bounds_l){
          continue;
        }
      }
      double obstacle_v = obstacle->frenet_velocity_s();
      double relative_speed = ego_speed_ - obstacle_v;
      double dist = ego_frenet_boundary_.s_start - frenet_obstacle_sl.s_end;
      if (std::abs(relative_speed) <= 0.3 &&
          dist > 1.5 * vehicle_param.length) {  // ego_v ≤ obs_v
        continue;
      } else if (std::abs(relative_speed) <= 0.3 &&
                 dist <= 1.5 * vehicle_param.length) {
        lane_borrow_decider_output_.lane_borrow_failed_reason =
            NEARBY_OBSTACLE_TOO_CLOSE;
        lane_borrow_decider_output_.failed_obs_id = obstacle->obstacle()->id();
        return false;
      } else if (ego_speed_ < obstacle_v) {     // ego_v < obs_v
        double TTC = dist / (-relative_speed);  // 分母取正
        if (TTC >= kMaxConcernCollisionTime) {
          continue;
        } else {
          lane_borrow_decider_output_.lane_borrow_failed_reason =
              BACKWARD_OBSTACLE_TOO_CLOSE;
          lane_borrow_decider_output_.failed_obs_id =
              obstacle->obstacle()->id();
          return false;
        }
      } else {
        continue;
      }
    } else if (frenet_obstacle_sl.s_start <
               ego_frenet_boundary_.s_end + vehicle_param.length) {
      if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW) {
        if (frenet_obstacle_sl.l_start > left_left_bounds_l ||
            frenet_obstacle_sl.l_end < left_right_bounds_l) {
          continue;
        }
      } else if (lane_borrow_decider_output_.borrow_direction == RIGHT_BORROW) {
        if (frenet_obstacle_sl.l_end < right_right_bounds_l ||
            frenet_obstacle_sl.l_start > right_left_bounds_l) {
          continue;
        }
      }
      lane_borrow_decider_output_.lane_borrow_failed_reason =
          NEARBY_OBSTACLE_TOO_CLOSE;
      lane_borrow_decider_output_.failed_obs_id = obstacle->obstacle()->id();
      return false;
    }
  }
  return true;
}

// v2
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

  lane_borrow_pb_info->set_lane_borrow_decider_status(
      static_cast<int>(lane_borrow_status_));
  lane_borrow_pb_info->set_dp_observe_frame_num(dp_observe_frame_num_);

  lane_borrow_pb_info->mutable_static_blocked_obj_id_vec()->Clear();
  for (auto static_obs_id : static_blocked_obj_id_vec_) {
    lane_borrow_pb_info->mutable_static_blocked_obj_id_vec()->Add(
        static_obs_id);
  }

  lane_borrow_pb_info->set_intersection_state(intersection_state_);
}
}  // namespace lane_borrow_deciderV2
}  // namespace planning