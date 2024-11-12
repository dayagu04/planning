#include "lane_borrow_decider.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "behavior_planners/lateral_offset_decider/lateral_offset_decider_utils.h"
#include "common_c.h"
#include "config/basic_type.h"
#include "define/geometry.h"
#include "environmental_model.h"
#include "lateral_obstacle.h"
#include "math/polygon2d.h"
#include "obstacle_manager.h"
#include "planning_context.h"
#include "session.h"
#include "task_interface/lane_borrow_decider_output.h"

namespace {
constexpr double kMinDisToSolidLane = 50.0;
constexpr double kMinDisToStopLine = 50.0;
constexpr double kMinDisToCrossWalk = 50.0;
constexpr double kMaxConcernObsDistance = 40.0;
constexpr double kDefaultStopLineAreaDistance = 5.0;
constexpr double kFilterStopObsDistance = 25.0;
constexpr double kObsSpeedLimit = 3.0;
constexpr double kLatPassableBuffer =
    0.5;  // todo: same with lat decider and lon decider
constexpr double kObsLatBuffer = 0.3;
constexpr int kObserveFrames = 30;
constexpr double kBackwardSafeDistance = 50.0;
constexpr double kObsSpeedRatio = 3.5;
constexpr double kForwardOtherObsDistance = 20.0;
constexpr double kObsSpeedBuffer = 1.0;
constexpr double kObsLatExpendBuffer = 0.4;
};  // namespace

namespace planning {

bool LaneBorrowDecider::Execute() {
  ego_frenet_boundary_ = session_->environmental_model()
                             .get_reference_path_manager()
                             ->get_reference_path_by_current_lane()
                             ->get_ego_frenet_boundary();
  ego_speed_ = session_->environmental_model().get_ego_state_manager()->ego_v();

  if (!RunLaneBorrowStateMachine()) {
    std::cout << "lane borrow state machine run failed!" << std::endl;
  }

  return true;
}

bool LaneBorrowDecider::RunLaneBorrowStateMachine() {
  const auto& coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;

  if (coarse_planning_info.target_state != kLaneKeeping) {
    return true;
  }

  switch (lane_borrow_status_) {
    case LaneBorrowStatus::kNoLaneBorrow: {
      if (CheckIfLaneBorrowDriving()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowDriving;
      }

      break;
    }
    case LaneBorrowStatus::kLaneBorrowDriving: {
      if (!CheckIfLaneBorrowDriving()) {
        lane_borrow_status_ = LaneBorrowStatus::kNoLaneBorrow;
      } else if (CheckIfDrivingToPassSide()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowPassSide;
      }
      break;
    }
    case LaneBorrowStatus::kLaneBorrowPassSide: {
      if (CheckIfPassSideToBackDriving()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowBackDriving;
      }

      break;
    }
    case LaneBorrowStatus::kLaneBorrowBackDriving: {
      if (CheckIfBackDrivingToNoBorrow()) {
        lane_borrow_status_ = LaneBorrowStatus::kNoLaneBorrow;
      } else if (CheckIfBorrowAgain()) {
        lane_borrow_status_ = LaneBorrowStatus::kLaneBorrowPassSide;
      }

      break;
    }
  }

  if (lane_borrow_status_ != LaneBorrowStatus::kNoLaneBorrow) {
    lane_borrow_decider_output_.is_in_lane_borrow_status = true;
    lane_borrow_decider_output_.lane_borrow_failed_reason = NONE_FAILED_REASON;
    lane_borrow_decider_output_.blocked_obs_id = static_blocked_obj_vec_;
  } else {
    lane_borrow_decider_output_.is_in_lane_borrow_status = false;
    static_blocked_obj_vec_.clear();
    lane_borrow_decider_output_.blocked_obs_id = static_blocked_obj_vec_;
  }

  session_->mutable_planning_context()->mutable_lane_borrow_decider_output() =
      lane_borrow_decider_output_;

  // debug info
  auto lane_borrow_pb_info = DebugInfoManager::GetInstance()
                                 .GetDebugInfoPb()
                                 ->mutable_lane_borrow_decider_info();
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

  return true;
}

bool LaneBorrowDecider::CheckIfBorrowAgain() {
  auto current_reference_path = session_->environmental_model()
                                    .get_reference_path_manager()
                                    ->get_reference_path_by_current_lane();
  auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  auto lane = virtual_lane_manager->get_current_lane();

  double left_width = lane->width(ego_frenet_boundary_.s_end) * 0.5;
  double right_width = lane->width(ego_frenet_boundary_.s_end) * 0.5;

  const auto& obstacle_manager =
      session_->environmental_model().get_obstacle_manager();
  const auto& frenet_obstacles = current_reference_path->get_obstacles();

  for (const auto& frenet_obstacle : frenet_obstacles) {
    const auto& frenet_obstacle_sl =
        frenet_obstacle->frenet_obstacle_boundary();
    if (frenet_obstacle_sl.s_start >
        ego_frenet_boundary_.s_end + kForwardOtherObsDistance) {
      continue;
    }
    if (frenet_obstacle_sl.l_start > left_width ||
        frenet_obstacle_sl.l_end < -right_width) {
      continue;
    }

    const double obs_v = frenet_obstacle->obstacle()->velocity();
    if (frenet_obstacle_sl.s_start > ego_frenet_boundary_.s_end) {
      if (!frenet_obstacle->obstacle()->is_static()) {
        continue;
      }

    } else {
      if (lane_borrow_decider_output_.borrow_direction == 1) {
        if (frenet_obstacle_sl.l_end > ego_frenet_boundary_.l_start) {
          continue;
        }
        if (frenet_obstacle_sl.l_end < -right_width &&
            frenet_obstacle->obstacle()->is_static()) {
          continue;
        }
        if (frenet_obstacle_sl.l_end + kLatPassableBuffer < -right_width) {
          continue;
        }

      } else {
        if (frenet_obstacle_sl.l_start < ego_frenet_boundary_.l_end) {
          continue;
        }
        if (frenet_obstacle_sl.l_start > left_width &&
            frenet_obstacle->obstacle()->is_static()) {
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

bool LaneBorrowDecider::CheckIfBackDrivingToNoBorrow() {
  auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  auto lane = virtual_lane_manager->get_current_lane();

  double left_width = lane->width(ego_frenet_boundary_.s_start) * 0.5;
  double right_width = lane->width(ego_frenet_boundary_.s_start) * 0.5;

  if (ego_frenet_boundary_.l_end <
          left_width &&  // ego is totally in origin lane
      ego_frenet_boundary_.l_start > -right_width) {
    ClearLaneBorrowStatus();
    return true;
  } else {
    return false;
  }
}

void LaneBorrowDecider::ClearLaneBorrowStatus() {
  observe_frame_num_ = 0;
  left_borrow_ = false;
  right_borrow_ = false;
}

bool LaneBorrowDecider::CheckIfDrivingToPassSide() {
  // check if driving on side
  if (ego_frenet_boundary_.s_start < front_pass_sl_point_.first) {
    return false;
  }

  // todo: update l

  return true;
}

bool LaneBorrowDecider::CheckIfLaneBorrowDriving() {
  auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  auto ego_state_manager =
      session_->environmental_model().get_ego_state_manager();

  CalcDistanceToSolidLane();
  distance_to_stop_line_ = virtual_lane_manager->GetEgoDistanceToStopline();
  distance_to_cross_walk_ = virtual_lane_manager->GetEgoDistanceToCrosswalk();

  if (forward_solid_start_s_ < kMinDisToSolidLane ||
      distance_to_cross_walk_ < kMinDisToCrossWalk ||
      distance_to_stop_line_ < kMinDisToStopLine) {
    std::cout << "Ego car dis to solid start_s: " << forward_solid_start_s_
              << std::endl;
    std::cout << "Ego car dis to cross walk: " << distance_to_cross_walk_
              << std::endl;
    std::cout << "Ego car dis to stop line " << distance_to_stop_line_
              << std::endl;
    return false;
  }

  if (!HasBlockingObstacle()) {
    return false;
  }

  observe_frame_num_++;

  UpdateAdcInfo();

  if (!left_borrow_ && !right_borrow_) {
    return false;
  }

  if (observe_frame_num_ < kObserveFrames) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        OBSERVE_TIME_CHECK_FAILED;
    return false;
  }

  if (!IsSafeForLaneBorrow()) {
    return false;
  }

  last_ego_center_position_.first = ego_state_manager->ego_pose().x;
  last_ego_center_position_.second = ego_state_manager->ego_pose().y;
  return true;
}

bool LaneBorrowDecider::IsSafeForLaneBorrow() {
  auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  auto current_lane = virtual_lane_manager->get_current_lane();

  auto current_reference_path = session_->environmental_model()
                                    .get_reference_path_manager()
                                    ->get_reference_path_by_current_lane();
  double lane_width = current_lane->width();
  const double current_left_lane_width = lane_width * 0.5;
  const double current_right_lane_width = lane_width * 0.5;

  double neighbor_left_width = current_left_lane_width;  // defualt init
  double neighbor_right_width = current_right_lane_width;

  double right_bounds_l = 0.0;
  double left_bounds_l = 0.0;
  bool clear_to_lane_borrow = false;
  double target_l = 0.0;
  if (left_borrow_) {
    right_bounds_l = obs_left_l_;
    const auto& left_lane = virtual_lane_manager->get_left_lane();
    if (left_lane == nullptr) {
      std::cout << "left lane is nullptr!" << std::endl;
      return false;
    }
    const double neighbor_width =
        left_lane->width(  // todo: add lane ptr protect
            vehicle_param_
                .front_edge_to_rear_axle);  // use ego front bump width
    neighbor_left_width = neighbor_width * 0.5;
    neighbor_right_width = neighbor_width * 0.5;

    left_bounds_l =
        current_left_lane_width + neighbor_right_width + neighbor_left_width;
    clear_to_lane_borrow =
        ClearForLaneBorrow(ego_speed_, left_bounds_l, right_bounds_l);
    target_l = std::min(
        left_bounds_l - kLatPassableBuffer - vehicle_param_.width * 0.5,
        right_bounds_l + kLatPassableBuffer + vehicle_param_.width * 0.5);
    target_l = std::max(target_l, right_bounds_l + vehicle_param_.width * 0.5);
    target_l = std::min(target_l, left_bounds_l - vehicle_param_.width * 0.5);
  }
  if (!clear_to_lane_borrow && right_borrow_) {
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
  if (!clear_to_lane_borrow) {
    return false;
  }

  lane_borrow_decider_output_.target_l = target_l;
  lane_borrow_decider_output_.left_bounds_l = left_bounds_l;
  lane_borrow_decider_output_.right_bounds_l = right_bounds_l;
  lane_borrow_decider_output_.borrow_direction = left_borrow_ ? 1 : 2;

  front_pass_sl_point_.first = obs_start_s_;
  front_pass_sl_point_.second = 0.0;
  Point2D frenet_front_pass_point{obs_start_s_, 0.0};

  current_reference_path->get_frenet_coord()->SLToXY(
      obs_start_s_, 0.0, &front_pass_point_.first, &front_pass_point_.second);
  return true;
};

void LaneBorrowDecider::CalcDistanceToSolidLane() {
  auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  auto lane = virtual_lane_manager->get_current_lane();

  auto current_reference_path = session_->environmental_model()
                                    .get_reference_path_manager()
                                    ->get_reference_path_by_current_lane();
  forward_solid_start_s_ = std::numeric_limits<double>::infinity();
  forward_solid_end_s_ = std::numeric_limits<double>::infinity();
  double route_s = 0.0;

  if (lane == nullptr) {
    return;
  }

  double distance_from_lane_start_to_solid = 0.0;
  const int num_type_segements =
      std::min(lane->get_left_lane_boundary().type_segments_size,
               lane->get_right_lane_boundary().type_segments_size);
  for (int i = 0; i < num_type_segements; i++) {
    const auto& left_type_segment =
        lane->get_left_lane_boundary().type_segments[i];
    const auto& right_type_segment =
        lane->get_right_lane_boundary().type_segments[i];
    if (left_type_segment.begin <= 0 ||
        right_type_segment.begin < 0) {  // todo:check
      continue;
    }

    if (left_type_segment.type != iflyauto::LaneBoundaryType_MARKING_DASHED &&
        left_type_segment.type !=
            iflyauto::LaneBoundaryType_MARKING_LEFT_DASHED_RIGHT_SOLID &&
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
        current_reference_path->get_frenet_coord()->Length()) {
      break;
    }
  }

  return;
}

bool LaneBorrowDecider::HasBlockingObstacle() {
  auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  auto lane = virtual_lane_manager->get_current_lane();

  auto current_reference_path = session_->environmental_model()
                                    .get_reference_path_manager()
                                    ->get_reference_path_by_current_lane();

  const double forward_obs_s =
      std::fmin(current_reference_path->get_frenet_coord()->Length(),
                ego_frenet_boundary_.s_end + kMaxConcernObsDistance);
  double left_width = lane->width(ego_frenet_boundary_.s_end) * 0.5;
  double right_width = lane->width(ego_frenet_boundary_.s_end) * 0.5;
  double vehicle_length = vehicle_param_.length;

  obs_left_l_ = -left_width;
  obs_right_l_ = right_width;
  obs_start_s_ = forward_obs_s;
  obs_end_s_ = 0.0;

  const auto& lat_obstacle_decision = session_->environmental_model()
                                          .get_lateral_obstacle()
                                          ->lat_obstacle_decision();
  const auto& obstacle_manager =
      session_->environmental_model().get_obstacle_manager();
  const auto& frenet_obstacles = current_reference_path->get_obstacles();
  static_blocked_obj_vec_.clear();

  for (const auto& frenet_obstacle : frenet_obstacles) {
    const auto& id = frenet_obstacle->obstacle()->id();
    if (!(frenet_obstacle->obstacle()->fusion_source() &
          OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    const auto frenet_obstacle_sl = frenet_obstacle->frenet_obstacle_boundary();

    if (frenet_obstacle_sl.s_start > forward_obs_s ||
        frenet_obstacle_sl.s_end < ego_frenet_boundary_.s_end) {
      // obstacle is not in concern area
      continue;
    }

    if (frenet_obstacle_sl.l_start > left_width ||
        frenet_obstacle_sl.l_end < -right_width) {
      // obstacle is absolutly out ego current lane
      continue;
    }

    // todo: reverse obstacle nees to filter
    if ((frenet_obstacle_sl.s_start > distance_to_cross_walk_ &&
         frenet_obstacle_sl.s_start <
             distance_to_cross_walk_ + kDefaultStopLineAreaDistance) ||
        (frenet_obstacle_sl.s_start > distance_to_stop_line_ &&
         frenet_obstacle_sl.s_start <
             distance_to_stop_line_ + kDefaultStopLineAreaDistance)) {
      if (!frenet_obstacle->obstacle()->is_static()) {
        // the obstacle in stop line area will move
        continue;
      }
      if (frenet_obstacle_sl.s_start - ego_frenet_boundary_.s_end >
          kFilterStopObsDistance) {
        // the obstacle in stop line area will move
        continue;
      }
    } else if (frenet_obstacle_sl.s_end >
                   forward_solid_start_s_ - vehicle_length * 2.0 &&
               frenet_obstacle_sl.s_start > forward_solid_end_s_) {
      // the obstacle is parellel to solid lane and in center line
      if (frenet_obstacle->obstacle()->is_static() &&
          frenet_obstacle_sl.l_start < 0.0 && frenet_obstacle_sl.l_end > 0.0) {
        lane_borrow_decider_output_.lane_borrow_failed_reason =
            SOLID_LINE_BORROW_DISABLED;
        lane_borrow_decider_output_.failed_obs_id =
            frenet_obstacle->obstacle()->id();
        return false;
      }
      continue;
    } else {
      if (frenet_obstacle_sl.l_end < left_width &&
          frenet_obstacle_sl.l_start > -right_width) {
        // the obstacle in current lane is moving
        if (frenet_obstacle->obstacle()->velocity() > kObsSpeedLimit) {
          continue;
        }
      } else {
        // the obstacle may enter cur lane is moving
        if (!frenet_obstacle->obstacle()->is_static()) {
          continue;
        }
      }
    }

    obs_left_l_ = std::max(obs_left_l_, frenet_obstacle_sl.l_end);
    obs_right_l_ = std::min(obs_right_l_, frenet_obstacle_sl.l_start);
    obs_start_s_ = std::min(obs_start_s_, frenet_obstacle_sl.s_start);
    obs_end_s_ = std::max(obs_end_s_, frenet_obstacle_sl.s_end);

    static_blocked_obj_vec_.emplace_back(frenet_obstacle->obstacle()->id());
  }
  obs_start_s_ = std::max(ego_frenet_boundary_.s_end, obs_start_s_);

  if (obs_left_l_ <= obs_right_l_) {  // todo: what the case mean?
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        NO_PASSABLE_OBSTACLE;
    return false;
  }

  const double adc_width = vehicle_param_.width;

  if (obs_left_l_ + adc_width + kLatPassableBuffer < left_width ||
      obs_right_l_ - adc_width - kLatPassableBuffer > -right_width) {
    lane_borrow_decider_output_.lane_borrow_failed_reason = SELF_LANE_ENOUGH;
    return false;
  }
  obs_left_l_ += kObsLatBuffer;
  obs_right_l_ -= kObsLatBuffer;
  return true;
}

void LaneBorrowDecider::UpdateAdcInfo() {
  left_borrow_ = true;
  right_borrow_ = true;

  auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  auto current_lane = virtual_lane_manager->get_current_lane();

  double lane_line_length = 0.0;
  const auto& left_lane_boundarys = current_lane->get_left_lane_boundary();
  const auto& right_lane_boundarys = current_lane->get_right_lane_boundary();
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
          iflyauto::LaneBoundaryType_MARKING_LEFT_DASHED_RIGHT_SOLID &&
      left_lane_boundary_type !=
          iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED) {
    left_borrow_ = false;
  }

  // todo: if left lane is reverse, then left_boorow is false
  if (right_lane_boundary_type != iflyauto::LaneBoundaryType_MARKING_DASHED &&
      right_lane_boundary_type !=
          iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED) {
    right_borrow_ = false;
  }

  // todo: consider ego car near/in stop line or crosswalk area
  if (!left_borrow_ && !right_borrow_) {
    lane_borrow_decider_output_.lane_borrow_failed_reason =
        LANE_TYPE_CHECK_FAILED;
  }
  return;
}

bool LaneBorrowDecider::ClearForLaneBorrow(const double ego_speed,
                                           const double& left_bounds_l,
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

  auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  auto lane = virtual_lane_manager->get_current_lane();

  auto current_reference_path = session_->environmental_model()
                                    .get_reference_path_manager()
                                    ->get_reference_path_by_current_lane();

  double left_width = lane->width(ego_frenet_boundary_.s_end) * 0.5;
  double right_width = lane->width(ego_frenet_boundary_.s_end) * 0.5;

  const auto& obstacle_manager =
      session_->environmental_model().get_obstacle_manager();
  const auto& frenet_obstacles = current_reference_path->get_obstacles();
  for (const auto& frenet_obstacle : frenet_obstacles) {
    if (!(frenet_obstacle->obstacle()->fusion_source() &
          OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    if (!frenet_obstacle->b_frenet_valid()) {
      continue;
    }

    const auto frenet_obstacle_sl = frenet_obstacle->frenet_obstacle_boundary();

    if (frenet_obstacle_sl.s_start >
        ego_frenet_boundary_.s_end) {  // obs in front of ego
      if (!frenet_obstacle->obstacle()->is_static()) {
        continue;
      }
      if (frenet_obstacle_sl.s_start > obs_end_s_) {
        continue;
      }
      if (left_borrow_) {
        if (frenet_obstacle_sl.l_start > left_bounds_l ||
            frenet_obstacle_sl.l_end < left_width) {
          continue;  // todo: why?
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
              frenet_obstacle->obstacle()->id();
          return false;
        }
      } else {
        if (frenet_obstacle_sl.l_start > -right_width ||
            frenet_obstacle_sl.l_end < right_bounds_l) {
          continue;  // why
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
              frenet_obstacle->obstacle()->id();
          return false;
        }
      }

    } else {  // obs back of ego
      if (frenet_obstacle_sl.l_start > left_l ||
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

      double dist =
          std::max(kBackwardSafeDistance,
                   frenet_obstacle->obstacle()->velocity() * kObsSpeedRatio);
      if (frenet_obstacle_sl.s_end + dist > ego_frenet_boundary_.s_start) {
        lane_borrow_decider_output_.lane_borrow_failed_reason =
            BACKWARD_OBSTACLE_TOO_CLOSE;
        lane_borrow_decider_output_.failed_obs_id =
            frenet_obstacle->obstacle()->id();
        return false;
      }
    }
  }

  return true;
}

bool LaneBorrowDecider::CheckIfPassSideToBackDriving() {
  if (!IsSafeForBack()) {
    return false;
  }
  return true;
}

bool LaneBorrowDecider::IsSafeForBack() {
  auto current_reference_path = session_->environmental_model()
                                    .get_reference_path_manager()
                                    ->get_reference_path_by_current_lane();
  const auto& obstacle_manager =
      session_->environmental_model().get_obstacle_manager();
  const auto& frenet_obstacles = current_reference_path->get_obstacles();
  auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  auto lane = virtual_lane_manager->get_current_lane();

  double left_width = lane->width(ego_frenet_boundary_.s_end) * 0.5;
  double right_width = lane->width(ego_frenet_boundary_.s_end) * 0.5;

  for (const auto& frenet_obstacle : frenet_obstacles) {
    const auto& frenet_obstacle_sl =
        frenet_obstacle->frenet_obstacle_boundary();
    if (lane_borrow_decider_output_.borrow_direction == 1) {
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

    const double obs_v = frenet_obstacle->obstacle()->velocity();
    if (frenet_obstacle_sl.s_start - ego_frenet_boundary_.s_end >
            kBackwardSafeDistance &&
        obs_v > ego_speed_ + kObsSpeedBuffer) {
      continue;
    }

    if (frenet_obstacle_sl.s_end >
        ego_frenet_boundary_.s_start - kSafeDistance) {
      return false;
    }

    if (ego_speed_ - obs_v > kObsSpeedBuffer) {
      continue;
    }
    if (frenet_obstacle_sl.l_start > ego_frenet_boundary_.l_end ||
        frenet_obstacle_sl.l_end < ego_frenet_boundary_.l_start) {
      const double dist =
          std::max(kBackwardSafeDistance, obs_v * kObsSpeedRatio);
      if (frenet_obstacle_sl.s_end + dist > ego_frenet_boundary_.s_start) {
        return false;  // fast come near ego car
      }
    }
  }

  if (lane_borrow_decider_output_.borrow_direction == 1) {
    lane_borrow_decider_output_.right_bounds_l = -right_width;
  } else {
    lane_borrow_decider_output_.left_bounds_l = left_width;
  }

  lane_borrow_decider_output_.target_l = 0.0;
  return true;
}

}  // namespace planning