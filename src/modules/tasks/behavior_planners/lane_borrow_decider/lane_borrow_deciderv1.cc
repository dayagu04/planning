#include "lane_borrow_deciderv1.h"

#include <limits>

#include "behavior_planners/lateral_offset_decider/lateral_offset_decider_utils.h"
#include "common_c.h"
#include "config/basic_type.h"
#include "define/geometry.h"
#include "environmental_model.h"
#include "lateral_obstacle.h"
#include "log.h"
#include "math/polygon2d.h"
#include "obstacle_manager.h"
#include "planning_context.h"
#include "session.h"
#include "task_interface/lane_borrow_decider_output.h"
#include "tracked_object.h"

namespace {
constexpr double kMinDisToSolidLane = 50.0;
constexpr double kMinDisToStopLine = 50.0;
constexpr double kMinDisToCrossWalk = 50.0;
constexpr double kMaxConcernObsDistance = 40.0;
constexpr double kDefaultStopLineAreaDistance = 5.0;
constexpr double kFilterStopObsDistance = 25.0;
constexpr double kObsSpeedLimit = 3.0;
constexpr double kLatPassableBuffer =
    0.8;  // todo: same with lat decider and lon decider
constexpr double kObsLatBuffer = 0.3;
constexpr int kObserveFrames = 30;
constexpr double kBackwardSafeDistance = 50.0;
constexpr double kObsSpeedRatio = 3.5;
constexpr double kForwardOtherObsDistance = 20.0;
constexpr double kObsSpeedBuffer = 1.0;
constexpr double kObsLatExpendBuffer = 0.4;
constexpr double kObsLonDisBuffer = 2.0;
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

  if (current_lane_ptr_ == nullptr || current_reference_path_ptr_ == nullptr) {
    LOG_ERROR("No current_lane_ptr_ or current_reference_path_ptr!");
    return false;
  };

  ego_speed_ = session_->environmental_model().get_ego_state_manager()->ego_v();
  ego_frenet_boundary_ = session_->environmental_model()
                             .get_reference_path_manager()
                             ->get_reference_path_by_current_lane()
                             ->get_ego_frenet_boundary();
  lane_change_state_ = session_->planning_context()
                           .lane_change_decider_output()
                           .coarse_planning_info.target_state;
  if (lane_change_state_ != kLaneKeeping) {
    LOG_ERROR("It has lane change state!");
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
      if (!CheckIfLaneBorrowDriving()) {
        lane_borrow_status_ = LaneBorrowStatus::kNoLaneBorrow;
      } else if (CheckIfLaneBorrowDrivingToBackDriving()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowBackDriving;
      }
      break;
    }
    case LaneBorrowStatus::kLaneBorrowBackDriving: {
      if (CheckIfBackDrivingToNoBorrow()) {
        lane_borrow_status_ = LaneBorrowStatus::kNoLaneBorrow;
      } else if (CheckIfBorrowAgain()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowDriving;
      }
      break;
    }
  }
}

bool LaneBorrowDecider::CheckIfNoLaneBorrowToLaneBorrowDriving() {
  UpdateJunctionInfo();

  if (forward_solid_start_s_ < kMinDisToSolidLane ||
      distance_to_cross_walk_ < kMinDisToCrossWalk ||
      distance_to_stop_line_ < kMinDisToStopLine) {
    LOG_DEBUG("Ego car is near junction");
    lane_borrow_decider_output_.lane_borrow_failed_reason = CLOSE_TO_JUNCTION;
    return false;
  }

  if (!SelectStaticBlockingArea()) {
    return false;
  }

  if (!UpdateLaneBorrowDirection()) {
    return false;
  };

  observe_frame_num_++;
  if (observe_frame_num_ < kObserveFrames) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        OBSERVE_TIME_CHECK_FAILED;
    return false;
  }

  if (!IsSafeForLaneBorrow()) {
    return false;
  }
}

bool LaneBorrowDecider::SelectStaticBlockingArea() {
  const double forward_obs_s =
      std::fmin(current_reference_path_ptr_->get_frenet_coord()->Length(),
                ego_frenet_boundary_.s_end + kMaxConcernObsDistance);
  double left_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  double right_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;

  obs_left_l_ = -left_width;
  obs_right_l_ = right_width;
  obs_start_s_ = forward_obs_s;
  obs_end_s_ = 0.0;

  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  static_blocked_obj_vec_.clear();
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

    const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
    if (frenet_obstacle_sl.s_start > forward_obs_s ||
        frenet_obstacle_sl.s_end + kObsLonDisBuffer <
            ego_frenet_boundary_.s_start) {  // lon concern area
      continue;
    }

    if (frenet_obstacle_sl.l_start > left_width ||
        frenet_obstacle_sl.l_end < -right_width) {
      // obstacle is absolutly out ego current lane
      continue;
    }

    // TODO: concern more scene
    if (frenet_obstacle_sl.l_end < left_width &&
        frenet_obstacle_sl.l_start > -right_width) {
      if (obstacle->obstacle()->velocity() > kObsSpeedLimit) {
        continue;
      }
    } else {
      if (!obstacle->obstacle()->is_static()) {
        continue;
      }
    }
    obs_left_l_ = std::max(obs_left_l_, frenet_obstacle_sl.l_end);
    obs_right_l_ = std::min(obs_right_l_, frenet_obstacle_sl.l_start);
    obs_start_s_ = std::min(obs_start_s_, frenet_obstacle_sl.s_start);
    obs_end_s_ = std::max(obs_end_s_, frenet_obstacle_sl.s_end);

    static_blocked_obj_vec_.emplace_back(obstacle->obstacle()->id());
  }

  obs_start_s_ = std::max(ego_frenet_boundary_.s_end, obs_start_s_);
  if (obs_left_l_ <= obs_right_l_) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        NO_PASSABLE_OBSTACLE;
    return false;
  }
  if (obs_left_l_ + vehicle_param_.width + kLatPassableBuffer < left_width ||
      obs_right_l_ - vehicle_param_.width - kLatPassableBuffer > -right_width) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = SELF_LANE_ENOUGH;
    return false;
  }
  obs_left_l_ += kObsLatBuffer;
  obs_right_l_ -= kObsLatBuffer;
  return true;
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

  for (int i = 0; i < left_lane_boundarys.type_segments_size; i++) {
    lane_line_length += left_lane_boundarys.type_segments[i].length;
    if (lane_line_length > vehicle_param_.front_edge_to_rear_axle) {
      left_lane_boundary_type = left_lane_boundarys.type_segments[i].type;
    }
  }
  lane_line_length = 0.0;
  for (int i = 0; i < right_lane_boundarys.type_segments_size; i++) {
    lane_line_length += right_lane_boundarys.type_segments[i].length;
    if (lane_line_length > vehicle_param_.front_edge_to_rear_axle) {
      right_lane_boundary_type = right_lane_boundarys.type_segments[i].type;
    }
  }

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

void LaneBorrowDecider::UpdateJunctionInfo() {
  forward_solid_start_s_ = std::numeric_limits<double>::infinity();
  forward_solid_end_s_ = std::numeric_limits<double>::infinity();

  double distance_from_lane_start_to_solid = 0.0;
  const int num_type_segements =
      std::min(current_lane_ptr_->get_left_lane_boundary().type_segments_size,
               current_lane_ptr_->get_right_lane_boundary().type_segments_size);

  for (size_t i = 0; i < num_type_segements; i++) {
    const auto& left_type_segment =
        current_lane_ptr_->get_left_lane_boundary().type_segments[i];
    const auto& right_type_segment =
        current_lane_ptr_->get_right_lane_boundary().type_segments[i];
    if (left_type_segment.begin <= 0 || right_type_segment.begin <= 0) {
      continue;
    }
    if (left_type_segment.type != iflyauto::LaneBoundaryType_MARKING_DASHED &&
        left_type_segment.type !=
            iflyauto::LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED &&
        left_type_segment.type !=
            iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED &&
        right_type_segment.type != iflyauto::LaneBoundaryType_MARKING_DASHED &&
        right_type_segment.type !=
            iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED) {
      forward_solid_start_s_ =
          std::fmin(forward_solid_start_s_, left_type_segment.begin);
      forward_solid_end_s_ =
          std::fmin(left_type_segment.end, right_type_segment.end);
    }

    if (forward_solid_end_s_ >
        current_reference_path_ptr_->get_frenet_coord()->Length()) {
      break;
    }
  }
  return;
}

bool LaneBorrowDecider::IsSafeForLaneBorrow() {
  double right_bounds_l = 0.0;
  double left_bounds_l = 0.0;
  bool safe_to_left_lane_borrow = false;
  double target_l = 0.0;
  double neighbor_left_width = 1.75;  // defualt init
  double neighbor_right_width = 1.75;

  const double current_left_lane_width = current_lane_ptr_->width() * 0.5;
  const double current_right_lane_width = current_lane_ptr_->width() * 0.5;

  if (left_borrow_) {
    right_bounds_l = obs_left_l_;
    const double neighbor_width =
        left_lane_ptr_->width(vehicle_param_.front_edge_to_rear_axle);
    neighbor_left_width = neighbor_width * 0.5;
    neighbor_right_width = neighbor_width * 0.5;

    left_bounds_l =
        current_left_lane_width + neighbor_right_width + neighbor_left_width;
    safe_to_left_lane_borrow = IsSafeForPath(left_bounds_l, right_bounds_l);
    target_l = std::min(
        left_bounds_l - kLatPassableBuffer - vehicle_param_.width * 0.5,
        right_bounds_l + kLatPassableBuffer + vehicle_param_.width * 0.5);
    target_l = std::max(target_l, right_bounds_l + vehicle_param_.width * 0.5);
    target_l = std::min(target_l, left_bounds_l - vehicle_param_.width * 0.5);
  }

  if(!safe_to_left_lane_borrow && right_borrow_){
    left_borrow_ = false;
    left_bounds_l = obs_right_l_;
    const auto& right_lane = virtual_lane_manager->get_right_lane();
    if (right_lane == nullptr) {
      std::cout << "right lane is nullptr!" << std::endl;
      return false;
    }
    const double neighbor_width =
        right_lane->width(  // todo: add lane ptr protect
            vehicle_param_
                .front_edge_to_rear_axle);  // use ego front bump width
    right_bounds_l =
        -current_right_lane_width - neighbor_left_width - neighbor_right_width;
    clear_to_lane_borrow =
        ClearForLaneBorrow(ego_speed_, left_bounds_l, right_bounds_l);
    target_l = std::max(
        right_bounds_l + kLatPassableBuffer + vehicle_param_.width * 0.5,
        left_bounds_l - kLatPassableBuffer - vehicle_param_.width * 0.5);
    target_l = std::min(target_l, left_bounds_l - vehicle_param_.width * 0.5);
 

  }
}

bool LaneBorrowDecider::IsSafeForPath(const double& left_bounds_l,
                                      const double& right_bounds_l) {
  if (left_bounds_l - right_bounds_l < vehicle_param_.width) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = BOUNDS_TOO_NARROW;
    return false;
  }

  double left_l = left_bounds_l;
  double right_l = right_bounds_l;

  if (left_borrow_) {
    left_l =
        std::min(left_l, right_l + vehicle_param_.width + kObsLatExpendBuffer);
  } else {
    right_l =
        std::max(right_l, left_l - vehicle_param_.width - kObsLatExpendBuffer);
  }

  const double left_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;
  const double right_width =
      current_lane_ptr_->width(ego_frenet_boundary_.s_end) * 0.5;

  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  for (const auto& obstacle : obstacles) {
    if (!(obstacle->obstacle()->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    if (!obstacle->b_frenet_valid()) {
      continue;
    }

    const auto& frenet_obstacle_sl = obstacle->frenet_obstacle_boundary();
    if (frenet_obstacle_sl.s_start > ego_frenet_boundary_.s_end) {
      if (!obstacle->obstacle()->is_static()) {
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
        if (frenet_obstacle_sl.l_end + vehicle_param_.width +
                    kLatPassableBuffer >
                left_bounds_l &&
            frenet_obstacle_sl.l_start - vehicle_param_.width -
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

        if (frenet_obstacle_sl.l_end + vehicle_param_.width +
                    kLatPassableBuffer >
                obs_right_l_ &&
            frenet_obstacle_sl.l_end - vehicle_param_.width -
                    kLatPassableBuffer <
                right_bounds_l) {
          lane_borrow_decider_output_.lane_borrow_failed_reason =
              STATIC_OBSTACLE_BLOCKED;
          lane_borrow_decider_output_.failed_obs_id =
              obstacle->obstacle()->id();
          return false;
        }
      }
    } else {
      if (frenet_obstacle_sl.l_start < left_l ||
          frenet_obstacle_sl.l_end < right_l) {
        continue;
      }

      const double l_buffer = 0.5;
      if (left_borrow_) {
        if (frenet_obstacle_sl.l_start <
            ego_frenet_boundary_.l_end - l_buffer) {
          continue;
        }
      } else {
        if (frenet_obstacle_sl.l_end <
            ego_frenet_boundary_.l_start + l_buffer) {
          continue;
        }
      }

      double dist = std::max(kBackwardSafeDistance,
                             obstacle->obstacle()->velocity() * kObsSpeedRatio);
      if (frenet_obstacle_sl.s_end + dist > ego_frenet_boundary_.s_start) {
        lane_borrow_decider_output_.lane_borrow_failed_reason =
            BACKWARD_OBSTACLE_TOO_CLOSE;
        lane_borrow_decider_output_.failed_obs_id = obstacle->obstacle()->id();
        return false;
      }
    }
  }
  return true;
}

void LaneBorrowDecider::LogDebugInfo() {
  // debug info
  auto lane_borrow_pb_info = DebugInfoManager::GetInstance()
                                 .GetDebugInfoPb()
                                 ->mutable_lane_borrow_decider_info();
  lane_borrow_pb_info->set_lane_borrow_failed_reason(
      lane_borrow_decider_output_.lane_borrow_failed_reason);
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

  lane_borrow_pb_info->set_lane_borrow_decider_status(lane_borrow_status_);

  lane_borrow_pb_info->mutable_static_blocked_obj_vec()->Clear();
  for (auto static_obs_id : static_blocked_obj_vec_) {
    lane_borrow_pb_info->mutable_static_blocked_obj_vec()->Add(static_obs_id);
  }
}

}  // namespace planning