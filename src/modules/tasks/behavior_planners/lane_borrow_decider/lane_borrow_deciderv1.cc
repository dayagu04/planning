#include "lane_borrow_deciderv1.h"
#include <Eigen/src/Core/Matrix.h>
#include <math.h>

#include <cmath>

#include "basic_types.pb.h"
#include "behavior_planners/lateral_offset_decider/lateral_offset_decider_utils.h"
#include "behavior_planners/traffic_light_decider/traffic_light_decider.h"
#include "common_c.h"
#include "config/basic_type.h"
#include "config/message_type.h"
#include "define/geometry.h"
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
#include "utils/pose2d_utils.h"

namespace {
constexpr double kMinDisToSolidLane = 50.0;
constexpr double kMinDisToStopLine = 50.0;
constexpr double kMinDisToCrossWalk = 50.0;
constexpr double kMinDisToTrafficLight = 120.0;
constexpr double kInfDisToTrafficLight = 10000.0;
constexpr double kSafeBackDistance = 2.0;
constexpr double kDefaultStopLineAreaDistance = 5.0;
constexpr double kFilterStopObsDistance = 25.0;
constexpr double kObsSpeedLimit = 3.0;
constexpr double kLatPassableBuffer = 0.8;
constexpr double kObsLatBuffer = 0.3;
constexpr double kObsSpeedRatio = 3.5;
constexpr double kForwardOtherObsDistance = 20.0;
constexpr double kObsSpeedBuffer = 1.0;
constexpr double kObsLatExpendBuffer = 0.4;
constexpr double kObsLonDisBuffer = 2.0;
constexpr double kObsFilterVel = 2.5;
constexpr double kBlockHeading = 0.17;
constexpr double kCheckTurningDistance = 10.0;
constexpr double kMaxCentricOffset = 0.75;
constexpr double kBackNeededDistance = 5.0;
};  // namespace

namespace planning {

bool LaneBorrowDecider::Execute() {
  Update();

  LogDebugInfo();
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
  lane_borrow_pb_info->set_dis_to_traffic_lights(dis_to_traffic_lights_);
  if (current_lane_ptr_ == nullptr || current_reference_path_ptr_ == nullptr) {
    LOG_ERROR("No current_lane_ptr_ or current_reference_path_ptr!");
    return false;
  };

  ego_speed_ = session_->environmental_model().get_ego_state_manager()->ego_v();
  ego_frenet_boundary_ = session_->environmental_model()
                             .get_reference_path_manager()
                             ->get_reference_path_by_current_lane()
                             ->get_ego_frenet_boundary();
  heading_angle_ = session_->environmental_model()
                       .get_reference_path_manager()
                       ->get_reference_path_by_current_lane()
                       ->get_frenet_ego_state()
                       .heading_angle();
  lane_change_state_ = session_->planning_context()
                           .lane_change_decider_output()
                           .coarse_planning_info.target_state;
  if (lane_change_state_ != kLaneKeeping) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = LANE_CHANGE_STATE;
    LOG_ERROR("It has lane change state!");
    return false;
  }

  intersection_state_ = virtual_lane_manager->GetIntersectionState();
  if (intersection_state_ !=
      planning::common::IntersectionState::NO_INTERSECTION) {
    return false;
  }

  distance_to_stop_line_ = virtual_lane_manager->GetEgoDistanceToStopline();
  distance_to_cross_walk_ = virtual_lane_manager->GetEgoDistanceToCrosswalk();

  return true;
}

void LaneBorrowDecider::Update() {
  if (!ProcessEnvInfos()) {
    return;
  }

  switch (lane_borrow_status_) {
    case LaneBorrowStatus::kNoLaneBorrow: {
      if (CheckIfNoLaneBorrowToLaneBorrowDriving()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowDriving;
      }
      break;
    }

    case LaneBorrowStatus::kLaneBorrowDriving: {
      if (!CheckLaneBorrowCondition()) {
        lane_borrow_status_ = LaneBorrowStatus::kNoLaneBorrow;
      } else if (ChecekIfLaneBorrowToLaneBorrowCrossing()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowCrossing;
      }
      break;
    }

    case LaneBorrowStatus::kLaneBorrowCrossing: {
      if (CheckIfkLaneBorrowCrossingToNoBorrow()) {
        lane_borrow_status_ = LaneBorrowStatus::kNoLaneBorrow;
      } else if (IsSafeForBackOriginLane()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowBackOriginLane;
      }
      break;
    }

    case LaneBorrowStatus::kLaneBorrowBackOriginLane: {
      if (CheckIfLaneBorrowBackOriginToLaneBorrowCrossing()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowCrossing;
      } else if (CheckIfLaneBorrowBackOriginLaneToLaneBorrowDriving()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowDriving;
      } else if (CheckIfLaneBorrowBackOriginLaneToNoBorrow()) {
        lane_borrow_status_ = LaneBorrowStatus::kNoLaneBorrow;
      }
      break;
    }
  }
  last_static_blocked_obj_id_vec_ = static_blocked_obj_id_vec_;
  if (lane_borrow_status_ != LaneBorrowStatus::kNoLaneBorrow) {
    lane_borrow_decider_output_.is_in_lane_borrow_status = true;
    lane_borrow_decider_output_.lane_borrow_failed_reason = NONE_FAILED_REASON;
    lane_borrow_decider_output_.blocked_obs_id = static_blocked_obj_id_vec_;

  } else {
    lane_borrow_decider_output_.is_in_lane_borrow_status = false;
    static_blocked_obj_id_vec_.clear();
    lane_borrow_decider_output_.blocked_obs_id = static_blocked_obj_id_vec_;
    lane_borrow_decider_output_.borrow_direction = NO_BORROW;
  }

  session_->mutable_planning_context()->mutable_lane_borrow_decider_output() =
      lane_borrow_decider_output_;

  return;
}

bool LaneBorrowDecider::CheckIfNoLaneBorrowToLaneBorrowDriving() {
  if (!CheckLaneBorrowCondition()) {
    return false;
  }
  return true;
}

void LaneBorrowDecider::ClearLaneBorrowStatus() {
  observe_frame_num_ = 0;
  left_borrow_ = false;
  right_borrow_ = false;
  obs_direction_map_.clear();
}

bool LaneBorrowDecider::CheckIfLaneBorrowDrivingToLaneBorrowBackOriginLane() {
  if (!IsSafeForBackOriginLane()) {
    return false;
  }
  return true;
}

bool LaneBorrowDecider::CheckIfLaneBorrowBackOriginLaneToLaneBorrowDriving() {
  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  const double left_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  const double right_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  if (!last_static_blocked_obj_id_vec_.empty() &&
      last_static_blocked_obj_id_vec_[0] == static_blocked_obj_id_vec_[0]) {
    return false;
  }

  for (const auto& obstacle : obstacles) {
    const auto& id = obstacle->obstacle()->id();
    const auto& obs_type = obstacle->obstacle()->type();

    if (obs_type == iflyauto::ObjectType::OBJECT_TYPE_PEDESTRIAN) {
      continue;
    }

    if (!(obstacle->obstacle()->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
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

    const double obs_v = obstacle->obstacle()->velocity();
    if (frenet_obstacle_sl.s_start > ego_frenet_boundary_.s_end) {
      if (!obstacle->obstacle()->is_static()) {
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

bool LaneBorrowDecider::CheckIfLaneBorrowBackOriginLaneToNoBorrow() {
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

bool LaneBorrowDecider::CheckLaneBorrowCondition() {
  UpdateJunctionInfo();

  if (!UpdateLaneBorrowDirection()) {
    return false;
  }

  if (!SelectStaticBlockingObstcales()) {
    return false;
  }

  if (!ObstacleDecision()) {
    return false;
  }

  if ((forward_solid_start_dis_ <
           obs_end_s_ - ego_frenet_boundary_.s_start + kBackNeededDistance &&
       forward_solid_start_dis_ > 0) ||
      (distance_to_cross_walk_ < kMinDisToCrossWalk &&
       distance_to_cross_walk_ > 0.0) ||
      (distance_to_stop_line_ < kMinDisToStopLine &&
       distance_to_stop_line_ > 0.0) ||
      (dis_to_traffic_lights_ < kMinDisToTrafficLight &&
       dis_to_traffic_lights_ > 0.0)) {
    LOG_DEBUG("Ego car is near junction");
    lane_borrow_decider_output_.lane_borrow_failed_reason = CLOSE_TO_JUNCTION;
    return false;
  }

  observe_frame_num_++;
  if (observe_frame_num_ < config_.observe_frames) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        OBSERVE_TIME_CHECK_FAILED;
    return false;
  }

  if (!IsSafeForLaneBorrow()) {
    return false;
  }

  return true;
}

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
    if (frenet_obstacle_sl.s_start > ego_frenet_boundary_.s_end ||
        frenet_obstacle_sl.s_end < ego_frenet_boundary_.s_start) {
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
      if (!obstacle->obstacle()->is_static()) {  // part in lane
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

  if (lane_borrow_status_ == LaneBorrowStatus::kLaneBorrowCrossing) {
    for (const auto& obstacle : static_blocked_obstacles_) {
      const auto& id = obstacle->obstacle()->id();
      const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
      BorrowDirection obs_bypass_direction =
          GetBypassDirection(frenet_obstacle_sl, id);

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
    const auto& front_obstacle_sl =
        static_blocked_obstacles_[0]->frenet_obstacle_boundary();
    const auto& id = static_blocked_obstacles_[0]->obstacle()->id();
    BorrowDirection front_obs_bypass_direction =
        GetBypassDirection(front_obstacle_sl, id);
    const double front_obs_center_l =
        0.5 * (front_obstacle_sl.l_start + front_obstacle_sl.l_end);
    lane_borrow_pb_info->set_front_obs_center(front_obs_center_l);

    if (front_obs_bypass_direction == LEFT_BORROW && left_borrow_) {
      bypass_direction_ = LEFT_BORROW;
      right_borrow_ = false;
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

  double left_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  double right_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  if (obs_left_l_ + vehicle_param.width + kLatPassableBuffer < left_width ||
      obs_right_l_ - vehicle_param.width - kLatPassableBuffer > -right_width) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = SELF_LANE_ENOUGH;
    return false;
  }
  obs_left_l_ += kObsLatBuffer;
  obs_right_l_ -= kObsLatBuffer;
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
  const auto& lane_points = current_lane_ptr_->lane_points();
  for (int i = 0; i < lane_points.size(); i++) {
    lane_line_length = lane_points[i].s;
    if (lane_line_length > ego_frenet_boundary_.s_end) {
      left_lane_boundary_type = lane_points[i].left_lane_border_type;
      right_lane_boundary_type = lane_points[i].right_lane_border_type;
      break;
    }
  }
  // If the lane marking is not left dashed/right solid or double dashed, return
  // False.
  if (left_lane_boundary_type != iflyauto::LaneBoundaryType_MARKING_DASHED &&
      left_lane_boundary_type !=
          iflyauto::LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED &&
      left_lane_boundary_type !=
          iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED) {
    left_borrow_ = false;
  }

  if (left_lane_ptr_ == nullptr) {
    left_borrow_ = false;
  }

  // todo: if left lane is reverse, then left_boorow is false
  if (right_lane_boundary_type != iflyauto::LaneBoundaryType_MARKING_DASHED &&
      right_lane_boundary_type !=
          iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED) {
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

// Determine the type of lane marking.
bool LaneBorrowDecider::IsLaneTypeDashedOrMixed(
    const iflyauto::LaneBoundaryType& type) {
  return type == iflyauto::LaneBoundaryType_MARKING_DASHED ||
         type == iflyauto::LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED ||
         type == iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED ||
         type == iflyauto::LaneBoundaryType_MARKING_VIRTUAL;
}

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

bool LaneBorrowDecider::IsSafeForLaneBorrow() {
  double right_bounds_l = 0.0;
  double left_bounds_l = 0.0;

  double left_right_bounds_l = 0.0;
  double left_left_bounds_l = 0.0;

  double right_right_bounds_l = 0.0;
  double right_left_bounds_l = 0.0;

  bool safe_to_left_lane_borrow = false;
  double target_l = 0.0;
  double target_left_l = 0.0;
  double target_right_l = 0.0;
  double neighbor_left_width = 1.75;
  double neighbor_right_width = 1.75;

  auto lane_borrow_pb_info = DebugInfoManager::GetInstance()
                                 .GetDebugInfoPb()
                                 ->mutable_lane_borrow_decider_info();

  const double current_left_lane_width = current_lane_ptr_->width() * 0.5;
  const double current_right_lane_width = current_lane_ptr_->width() * 0.5;
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  if (left_borrow_) {
    left_right_bounds_l = obs_left_l_;
    const double neighbor_width =
        left_lane_ptr_->width(vehicle_param.front_edge_to_rear_axle);
    // neighbor lane width
    neighbor_left_width = neighbor_width * 0.5;
    neighbor_right_width = neighbor_width * 0.5;

    // Calculate the total width that can be borrowed from the left lane
    left_left_bounds_l =
        current_left_lane_width + neighbor_right_width + neighbor_left_width;
    if_left_turn_center_ = true;
    safe_to_left_lane_borrow =
        IsSafeForPath(left_left_bounds_l, left_right_bounds_l);

    target_left_l = std::min(
        left_left_bounds_l - kLatPassableBuffer - vehicle_param.width * 0.5,
        left_right_bounds_l + kLatPassableBuffer + vehicle_param.width * 0.5);
    target_left_l = std::max(target_left_l,
                             left_right_bounds_l + vehicle_param.width * 0.5);
    target_left_l =
        std::min(target_left_l, left_left_bounds_l - vehicle_param.width * 0.5);
  }
  bool safe_to_right_lane_borrow = false;
  if (right_borrow_) {
    left_borrow_ = false;
    right_left_bounds_l = obs_right_l_;
    if (right_lane_ptr_ == nullptr) {
      std::cout << "right lane is nullptr!" << std::endl;
      return false;
    }
    const double neighbor_width =
        right_lane_ptr_->width(vehicle_param.front_edge_to_rear_axle);
    right_right_bounds_l =
        -current_right_lane_width - neighbor_left_width - neighbor_right_width;
    if_left_turn_center_ = false;
    safe_to_right_lane_borrow =
        IsSafeForPath(right_left_bounds_l, right_right_bounds_l);
    target_right_l = std::max(
        right_right_bounds_l + kLatPassableBuffer + vehicle_param.width * 0.5,
        right_left_bounds_l - kLatPassableBuffer - vehicle_param.width * 0.5);
    target_right_l = std::min(target_right_l,
                              right_left_bounds_l - vehicle_param.width * 0.5);
  }
  double target_borrow_left = target_left_l;
  double target_borrow_right = target_right_l;
  lane_borrow_pb_info->set_target_left_l(target_borrow_left);
  lane_borrow_pb_info->set_target_right_l(target_borrow_right);
  lane_borrow_pb_info->set_safe_left_borrow(safe_to_left_lane_borrow);
  lane_borrow_pb_info->set_safe_right_borrow(safe_to_right_lane_borrow);
  double ego_state_l =
      (ego_frenet_boundary_.l_end + ego_frenet_boundary_.l_start) * 0.5;
  lane_borrow_pb_info->set_ego_l(ego_state_l);

  if (!safe_to_left_lane_borrow && !safe_to_right_lane_borrow) {
    lane_borrow_decider_output_.target_l = 0;
    lane_borrow_decider_output_.borrow_direction = NO_BORROW;
    return false;

  } else if (safe_to_left_lane_borrow && !safe_to_right_lane_borrow) {
    lane_borrow_decider_output_.target_l = target_left_l;
    lane_borrow_decider_output_.left_bounds_l = left_left_bounds_l;
    lane_borrow_decider_output_.right_bounds_l = left_right_bounds_l;
    lane_borrow_decider_output_.borrow_direction = LEFT_BORROW;

  } else if (!safe_to_left_lane_borrow && safe_to_right_lane_borrow) {
    lane_borrow_decider_output_.target_l = target_right_l;
    lane_borrow_decider_output_.left_bounds_l = right_left_bounds_l;
    lane_borrow_decider_output_.right_bounds_l = right_right_bounds_l;
    lane_borrow_decider_output_.borrow_direction = RIGHT_BORROW;

  } else {
    if (lane_borrow_decider_output_.borrow_direction == NO_BORROW) {
      if (abs(target_left_l - ego_state_l) <
          abs(target_right_l - ego_state_l)) {
        lane_borrow_decider_output_.target_l = target_left_l;
        lane_borrow_decider_output_.left_bounds_l = left_left_bounds_l;
        lane_borrow_decider_output_.right_bounds_l = left_right_bounds_l;
        lane_borrow_decider_output_.borrow_direction = LEFT_BORROW;
      } else {
        lane_borrow_decider_output_.target_l = target_right_l;
        lane_borrow_decider_output_.left_bounds_l = right_left_bounds_l;
        lane_borrow_decider_output_.right_bounds_l = right_right_bounds_l;
        lane_borrow_decider_output_.borrow_direction = RIGHT_BORROW;
      }
    }
  }
  return true;
}

bool LaneBorrowDecider::IsSafeForBackOriginLane() {
  double left_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  double right_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  //  going to overtake static area [the area is updating ]
  // ？
  if (obs_end_s_ - ego_frenet_boundary_.s_end > kSafeBackDistance) {
    return false;
  }
  for (const auto& obstacle : obstacles) {
    const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
    if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW) {
      if (frenet_obstacle_sl.l_end > ego_frenet_boundary_.l_start &&
          frenet_obstacle_sl.l_start > left_width) {
        continue;
      }
      if (frenet_obstacle_sl.l_end < -right_width) {
        continue;
      }
    } else {
      if (frenet_obstacle_sl.l_start < ego_frenet_boundary_.l_end &&
          frenet_obstacle_sl.l_end < -right_width) {
        continue;
      }
      if (frenet_obstacle_sl.l_start > left_width) {
        continue;
      }
    }

    if (frenet_obstacle_sl.s_start - ego_frenet_boundary_.s_end >
        kForwardOtherObsDistance) {
      continue;
    }

    const double obs_v = obstacle->obstacle()->velocity();
    if ((frenet_obstacle_sl.s_start - ego_frenet_boundary_.s_end >
         kSafeBackDistance) &&
        (obs_v > ego_speed_ + kObsSpeedBuffer)) {
      continue;
    }

    if (frenet_obstacle_sl.s_end > ego_frenet_boundary_.s_start) {
      return false;
    }

    if (ego_speed_ - obs_v > kObsSpeedBuffer) {
      continue;
    }
    if (frenet_obstacle_sl.l_start > ego_frenet_boundary_.l_end ||
        frenet_obstacle_sl.l_end < ego_frenet_boundary_.l_start) {
      const double dist = std::max(kSafeBackDistance, obs_v * kObsSpeedRatio);
      if (frenet_obstacle_sl.s_end + dist > ego_frenet_boundary_.s_start) {
        return false;  // fast come near ego car
      }
    }
  }

  if (lane_borrow_decider_output_.borrow_direction == LEFT_BORROW) {
    lane_borrow_decider_output_.right_bounds_l = -right_width;
  } else {
    lane_borrow_decider_output_.left_bounds_l = left_width;
  }

  lane_borrow_decider_output_.target_l = 0.0;
  return true;
}

bool LaneBorrowDecider::IsSafeForPath(const double& left_bounds_l,
                                      const double& right_bounds_l) {
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  if (left_bounds_l - right_bounds_l <
      vehicle_param.width + kObsLatExpendBuffer) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = BOUNDS_TOO_NARROW;
    return false;
  }

  double left_l = left_bounds_l;
  double right_l = right_bounds_l;

  if (left_borrow_) {
    left_l =
        std::min(left_l, right_l + vehicle_param.width + kObsLatExpendBuffer);
  } else {
    right_l =
        std::max(right_l, left_l - vehicle_param.width - kObsLatExpendBuffer);
  }

  const double left_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  const double right_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;

  //    too close to area but no borrow enough
  if (lane_borrow_status_ != LaneBorrowStatus::kLaneBorrowCrossing) {
    bool is_safe_for_turn = IsSafeForTurn();
    if (obs_start_s_ - ego_frenet_boundary_.s_end > 0 &&
        obs_start_s_ - ego_frenet_boundary_.s_end < kCheckTurningDistance &&
        is_safe_for_turn == false) {
      lane_borrow_decider_output_.lane_borrow_failed_reason =
          STATIC_AREA_TOO_CLOSE;
      return false;
    }
  }
  // turn radius calculate
  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  for (const auto& obstacle : obstacles) {
    const auto& id = obstacle->obstacle()->id();
    if (!(obstacle->obstacle()->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    if (!obstacle->b_frenet_valid()) {
      continue;
    }

    const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
    if (frenet_obstacle_sl.s_start > ego_frenet_boundary_.s_end) {
      if (obstacle->obstacle()->velocity() > kObsFilterVel) {
        continue;
      }
      if (frenet_obstacle_sl.s_start > obs_end_s_) {
        continue;
      }
      if (left_borrow_) {
        if (frenet_obstacle_sl.l_start > left_bounds_l ||
            frenet_obstacle_sl.l_end < left_width) {
          continue;
        }
        if (frenet_obstacle_sl.l_end + vehicle_param.width +
                    kLatPassableBuffer >
                left_bounds_l &&
            frenet_obstacle_sl.l_start - vehicle_param.width -
                    kLatPassableBuffer <
                obs_left_l_) {
          lane_borrow_decider_output_.lane_borrow_failed_reason =
              STATIC_OBSTACLE_BLOCKED;
          lane_borrow_decider_output_.failed_obs_id =
              obstacle->obstacle()->id();
          return false;
        }
      } else {
        if (frenet_obstacle_sl.l_start > -right_width ||
            frenet_obstacle_sl.l_end < right_bounds_l) {
          continue;
        }

        if (frenet_obstacle_sl.l_end + vehicle_param.width +
                    kLatPassableBuffer >
                obs_right_l_ &&
            frenet_obstacle_sl.l_end - vehicle_param.width -
                    kLatPassableBuffer <
                right_bounds_l) {
          lane_borrow_decider_output_.lane_borrow_failed_reason =
              STATIC_OBSTACLE_BLOCKED;
          lane_borrow_decider_output_.failed_obs_id =
              obstacle->obstacle()->id();
          return false;
        }
      }
    } else if (frenet_obstacle_sl.s_end < ego_frenet_boundary_.s_start) {
      if (left_borrow_) {
        if (frenet_obstacle_sl.l_start > left_bounds_l ||
            frenet_obstacle_sl.l_end < left_width) {
          continue;
        }

      } else {
        if (frenet_obstacle_sl.l_start > -right_width ||
            frenet_obstacle_sl.l_end < right_bounds_l) {
          continue;
        }
      }
      if (lane_borrow_status_ != LaneBorrowStatus::kLaneBorrowCrossing) {
        double dist =
            std::max(kSafeBackDistance,
                     obstacle->obstacle()->velocity() * kObsSpeedRatio);
        if (frenet_obstacle_sl.s_end + dist > ego_frenet_boundary_.s_start) {
          lane_borrow_decider_output_.lane_borrow_failed_reason =
              BACKWARD_OBSTACLE_TOO_CLOSE;
          lane_borrow_decider_output_.failed_obs_id =
              obstacle->obstacle()->id();
          return false;
        }
      }

    } else {
      if (left_borrow_) {
        if (frenet_obstacle_sl.l_start > left_bounds_l ||
            frenet_obstacle_sl.l_end < right_l) {
          continue;
        } else {
          lane_borrow_decider_output_.lane_borrow_failed_reason =
              NEARBY_OBSTACLE_TOO_CLOSE;
          lane_borrow_decider_output_.failed_obs_id =
              obstacle->obstacle()->id();
          return false;
        }

      } else {
        if (frenet_obstacle_sl.l_start > left_l ||
            frenet_obstacle_sl.l_end < right_bounds_l) {
          continue;
        } else {
          lane_borrow_decider_output_.lane_borrow_failed_reason =
              NEARBY_OBSTACLE_TOO_CLOSE;
          lane_borrow_decider_output_.failed_obs_id =
              obstacle->obstacle()->id();
          return false;
        }
      }
    }
  }
  return true;
}

bool LaneBorrowDecider::IsSafeForTurn() {
  // radius
  if (obs_left_l_ <= obs_right_l_) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        NO_PASSABLE_OBSTACLE;
    return false;
  }
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double wheel_base = vehicle_param.wheel_base;
  const double front_angle =
      vehicle_param.max_front_wheel_angle;  // relative to ego_speed_?
  const double max_current_radius = wheel_base / std::tan(front_angle);
  // turn center
  const Pose2D rear_center_pose =
      session_->environmental_model().get_ego_state_manager()->ego_pose();
  const Point2D rear_center_cart =
      session_->environmental_model().get_ego_state_manager()->ego_carte();
  const double heading_angle =
      session_->environmental_model().get_ego_state_manager()->ego_pose().theta;
  const Point2D turning_center =
      CalTurningCenter(rear_center_cart, heading_angle, max_current_radius);
  // sl static area overlap circle
  // sl to SLToXY
  auto current_reference_path = session_->environmental_model()
                                    .get_reference_path_manager()
                                    ->get_reference_path_by_current_lane();
  const auto& current_frenet_coord = current_reference_path->get_frenet_coord();

  Point2D back_right_corner, back_left_corner;
  current_frenet_coord->SLToXY(obs_start_s_, obs_right_l_, &back_right_corner.x,
                               &back_right_corner.y);
  current_frenet_coord->SLToXY(obs_start_s_, obs_left_l_, &back_left_corner.x,
                               &back_left_corner.y);
  const double corner_angle = std::tan((vehicle_param.max_width * 0.5) /
                                       vehicle_param.front_edge_to_rear_axle);
  const double corner_length =
      std::hypot(vehicle_param.max_width * 0.5,
                 vehicle_param.front_edge_to_rear_axle);  // 前保转弯半径
  const double corner_radius_square =
      corner_length * corner_length + max_current_radius * max_current_radius -
      2 * corner_length * max_current_radius * std::cos(corner_angle + M_PI_2);
  const double corner_radius = std::sqrt(corner_radius_square);

  const double distance_to_left =
      std::hypot(turning_center.x - back_left_corner.x,
                 turning_center.y - back_left_corner.y);
  const double distance_to_right =
      std::hypot(turning_center.x - back_right_corner.x,
                 turning_center.y - back_right_corner.y);
  // log
  auto lane_borrow_pb_info = DebugInfoManager::GetInstance()
                                 .GetDebugInfoPb()
                                 ->mutable_lane_borrow_decider_info();
  lane_borrow_pb_info->mutable_borrow_turn_circle()->mutable_center()->set_x(
      turning_center.x);
  lane_borrow_pb_info->mutable_borrow_turn_circle()->mutable_center()->set_y(
      turning_center.y);
  lane_borrow_pb_info->mutable_borrow_turn_circle()->set_corner_radius(
      corner_radius);
  if (distance_to_left > corner_radius && distance_to_right > corner_radius) {
    return true;
  } else {
    return false;
  }
}

// Turning center point
const Point2D LaneBorrowDecider::CalTurningCenter(const Point2D& ego_pos,
                                                  const double& theta,
                                                  const double& radius) const {
  Eigen::Vector2d ego_heading_vec(std::cos(theta), std::sin(theta));
  Eigen::Vector2d rear_pos(ego_pos.x, ego_pos.y);
  Eigen::Vector2d ego_n_vec(-ego_heading_vec.y(), ego_heading_vec.x());

  if (if_left_turn_center_ == false) {
    ego_n_vec *= -1.0;
  }
  Eigen::Vector2d center = rear_pos + ego_n_vec * radius;
  return Point2D(center.x(), center.y());
}

bool LaneBorrowDecider::ChecekIfLaneBorrowToLaneBorrowCrossing() {
  auto current_reference_path = session_->environmental_model()
                                    .get_reference_path_manager()
                                    ->get_reference_path_by_current_lane();
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  current_lane_ptr_ = virtual_lane_manager->get_current_lane();

  const auto& current_frenet_coord = current_reference_path->get_frenet_coord();
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double heading_angle =
      session_->environmental_model().get_ego_state_manager()->ego_pose().theta;

  double ego_x =
      session_->environmental_model().get_ego_state_manager()->ego_pose().x;
  double ego_y =
      session_->environmental_model().get_ego_state_manager()->ego_pose().y;

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

bool LaneBorrowDecider::CheckIfkLaneBorrowCrossingToNoBorrow() {
  if (!CheckLaneBorrowCrossingCondition()) {
    return true;
  }
  return false;
}

bool LaneBorrowDecider::CheckIfLaneBorrowCrossingToBackDriving() {
  if (!IsSafeForBackOriginLane()) {
    return false;
  }
  return true;
}

bool LaneBorrowDecider::CheckLaneBorrowCrossingCondition() {
  UpdateJunctionInfo();

  if (!UpdateLaneBorrowDirection()) {
    return false;
  }

  if (!SelectStaticBlockingObstcales()) {
    return false;
  }

  if (!ObstacleDecision()) {
    return false;
  }

  if ((forward_solid_start_dis_ <
           obs_end_s_ - ego_frenet_boundary_.s_start + kBackNeededDistance &&
       forward_solid_start_dis_ > 0) ||
      (distance_to_cross_walk_ < kMinDisToCrossWalk &&
       distance_to_cross_walk_ > 0.0) ||
      (distance_to_stop_line_ < kMinDisToStopLine &&
       distance_to_stop_line_ > 0.0) ||
      (dis_to_traffic_lights_ < kMinDisToTrafficLight &&
       dis_to_traffic_lights_ > 0.0)) {
    LOG_DEBUG("Ego car is near junction");
    lane_borrow_decider_output_.lane_borrow_failed_reason = CLOSE_TO_JUNCTION;
    return false;
  }

  observe_frame_num_++;
  if (observe_frame_num_ < config_.observe_frames) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        OBSERVE_TIME_CHECK_FAILED;
    return false;
  }

  if (!IsSafeForLaneBorrow()) {
    return false;
  }

  return true;
}

bool LaneBorrowDecider::CheckIfLaneBorrowBackOriginToLaneBorrowCrossing() {
  if (ChecekIfLaneBorrowToLaneBorrowCrossing()) {
    if (CheckIfLaneBorrowBackOriginLaneToLaneBorrowDriving()) {
      return true;
    }
  }
  return false;
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

  lane_borrow_pb_info->set_lane_borrow_decider_status(
      static_cast<int>(lane_borrow_status_));

  lane_borrow_pb_info->mutable_static_blocked_obj_id_vec()->Clear();
  for (auto static_obs_id : static_blocked_obj_id_vec_) {
    lane_borrow_pb_info->mutable_static_blocked_obj_id_vec()->Add(
        static_obs_id);
  }

  lane_borrow_pb_info->set_intersection_state(intersection_state_);
}

}  // namespace planning