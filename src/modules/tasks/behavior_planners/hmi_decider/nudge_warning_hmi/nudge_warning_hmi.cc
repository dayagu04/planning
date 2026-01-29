#include "nudge_warning_hmi.h"

#include "environmental_model.h"
#include "planning_context.h"

namespace planning {
constexpr double kBoundBeyondCenterlineThre = 0.1;
constexpr double kBoundBeLowCenterlineThre = 0.2;
constexpr static int kStartRunningCount = 1;
constexpr static int kStopRunningCount = 3;
constexpr int kCoolDownCount = 5;

NudgeWarningHMIDecider::NudgeWarningHMIDecider(framework::Session* session) {
  session_ = session;
}

bool NudgeWarningHMIDecider::Execute() {
  if (session_ == nullptr) {
    Reset();
    return false;
  }
  reference_path_ptr_ = session_->planning_context()
                            .lane_change_decider_output()
                            .coarse_planning_info.reference_path;
  if (reference_path_ptr_ == nullptr) {
    return false;
  }
  switch (current_state_) {
    case NudgeWarningState::IDLE:
      if (IsStartRunning()) {
        current_state_ = NudgeWarningState::RUNNING;
        stop_running_count_ = 0;
      }
      break;

    case NudgeWarningState::RUNNING:
      if (IsStopRunning()) {
        current_state_ = NudgeWarningState::EXITING;
        // start_running_count_ = 0;
        stop_running_count_ = 0;
        avoid_id_ = -1;
        avoid_direction_ = 0;
      }
      break;

    case NudgeWarningState::EXITING:
      current_state_ = NudgeWarningState::COOLDOWN;
      cooldown_count_ = 0;
      break;

    case NudgeWarningState::COOLDOWN:
      if (cooldown_count_ <= kCoolDownCount) {
        cooldown_count_++;
      } else {
        current_state_ = NudgeWarningState::IDLE;
        cooldown_count_ = 0;
        stop_running_count_ = 0;
      }
      break;
  }

  GenerateHmiOutput();
  return true;
}

bool NudgeWarningHMIDecider::IsStartRunning() {
  const auto& lateral_offset_decider_output =
      session_->mutable_planning_context()
          ->mutable_lateral_offset_decider_output();
  const auto& general_lateral_decider_output =
      session_->mutable_planning_context()
          ->mutable_general_lateral_decider_output();
  const auto& first_soft_bounds_info_output =
      general_lateral_decider_output.first_soft_bounds_info;
  const auto& second_soft_bounds_info_output =
      general_lateral_decider_output.second_soft_bounds_info;
  const auto& first_soft_bounds_frenet_point_output =
      general_lateral_decider_output.first_soft_bounds_frenet_point;
  const auto& second_soft_bounds_frenet_point_output =
      general_lateral_decider_output.second_soft_bounds_frenet_point;

  const auto avoid_ids = lateral_offset_decider_output.avoid_ids;
  const double planning_init_point_l =
      reference_path_ptr_->get_frenet_ego_state()
          .planning_init_point()
          .frenet_state.r;
  const auto& avd_obstacles = lateral_offset_decider_output.avd_obstacles;
  const double lateral_offset = lateral_offset_decider_output.lateral_offset;
  const auto ego_v =
      session_->environmental_model().get_ego_state_manager()->ego_v();
  if (ego_v < 5 / 3.6) {
    return false;
  }

  // lateral_offset避让
  if (avd_obstacles[0].flag != AvoidObstacleFlag::INVALID) {
    if (lateral_offset > 0.1) {
      if (avd_obstacles[0].max_l_to_ref < 0 && avd_obstacles[0].s_to_ego > 0) {
        avoid_id_ = avd_obstacles[0].track_id;
        avoid_direction_ = 1;
        return true;
      } else {
        if (avd_obstacles[1].flag != AvoidObstacleFlag::INVALID &&
            avd_obstacles[1].max_l_to_ref < 0 &&
            avd_obstacles[1].s_to_ego > 0) {
          avoid_id_ = avd_obstacles[1].track_id;
          avoid_direction_ = 1;
          return true;
        }
      }
    } else if (lateral_offset < -0.1) {
      if (avd_obstacles[0].min_l_to_ref > 0 && avd_obstacles[0].s_to_ego > 0) {
        avoid_id_ = avd_obstacles[0].track_id;
        avoid_direction_ = 2;
        return true;
      } else {
        if (avd_obstacles[1].flag != AvoidObstacleFlag::INVALID &&
            avd_obstacles[1].min_l_to_ref > 0 &&
            avd_obstacles[1].s_to_ego > 0) {
          avoid_id_ = avd_obstacles[1].track_id;
          avoid_direction_ = 2;
          return true;
        }
      }
    }
  }

  // bound避让
  std::vector<std::pair<int, int>> obstacle_pairs;  // id - index
  bool bound_nudge = false;
  auto check_and_add_avoid_id = [&](const BoundInfo& bound_info,
                                    double bound_value, int index,
                                    bool is_upper) {
    auto it = reference_path_ptr_->get_obstacles_map().find(bound_info.id);
    if (it == reference_path_ptr_->get_obstacles_map().end()) {
      return;
    }
    auto obs = it->second;
    if (reference_path_ptr_->get_ego_frenet_boundary().s_start >
        obs->frenet_obstacle_boundary().s_end) {
      return;
    }
    if ((bound_info.type == BoundType::DYNAMIC_AGENT ||
         bound_info.type == BoundType::AGENT ||
         bound_info.type == BoundType::ADJACENT_AGENT ||
         bound_info.type == BoundType::REVERSE_AGENT) &&
        bound_info.id != -100) {
      bound_nudge = is_upper ? (bound_value < -kBoundBeyondCenterlineThre &&
                                (planning_init_point_l - bound_value) > 0.05)
                             : (bound_value > kBoundBeyondCenterlineThre &&
                                (bound_value - planning_init_point_l) > 0.05);
      if (bound_nudge) {
        obstacle_pairs.emplace_back(bound_info.id, index);
      }
    }
  };

  //(huwang5)TODO:左右bound重叠时，障碍物的释放需要进一步考虑
  bound_nudge = false;
  for (int i = 0; i < first_soft_bounds_frenet_point_output.size(); ++i) {
    check_and_add_avoid_id(first_soft_bounds_info_output[i].first,
                           first_soft_bounds_frenet_point_output[i].first, i,
                           false);  // lower
    if (bound_nudge) {
      break;
    }
  }

  bound_nudge = false;
  for (int i = 0; i < first_soft_bounds_frenet_point_output.size(); ++i) {
    check_and_add_avoid_id(first_soft_bounds_info_output[i].second,
                           first_soft_bounds_frenet_point_output[i].second, i,
                           true);  // upper
    if (bound_nudge) {
      break;
    }
  }

  bound_nudge = false;
  for (int i = 0; i < second_soft_bounds_frenet_point_output.size(); ++i) {
    check_and_add_avoid_id(second_soft_bounds_info_output[i].first,
                           second_soft_bounds_frenet_point_output[i].first, i,
                           false);  // lower
    if (bound_nudge) {
      break;
    }
  }

  bound_nudge = false;
  for (int i = 0; i < second_soft_bounds_frenet_point_output.size(); ++i) {
    check_and_add_avoid_id(second_soft_bounds_info_output[i].second,
                           second_soft_bounds_frenet_point_output[i].second, i,
                           true);  // upper
    if (bound_nudge) {
      break;
    }
  }

  int avoid_id = -1;
  double max_s = 1000;
  for (const auto& pair : obstacle_pairs) {
    if (pair.second < max_s) {
      avoid_id = pair.first;
    }
  }

  // 理论上计数需要针对同一个障碍物
  if (avoid_id != -1) {
    start_running_count_ =
        std::min(kStartRunningCount, start_running_count_ + 1);
  }

  if (start_running_count_ >= kStartRunningCount) {
    avoid_id_ = avoid_id;
    return true;
  }
  return false;
}

bool NudgeWarningHMIDecider::IsStopRunning() {
  const auto& lateral_offset_decider_output =
      session_->mutable_planning_context()
          ->mutable_lateral_offset_decider_output();
  const auto& general_lateral_decider_output =
      session_->mutable_planning_context()
          ->mutable_general_lateral_decider_output();

  const auto& avd_obstacles = lateral_offset_decider_output.avd_obstacles;
  const auto& first_soft_bounds_info_output =
      general_lateral_decider_output.first_soft_bounds_info;
  const auto& second_soft_bounds_info_output =
      general_lateral_decider_output.second_soft_bounds_info;
  const auto& first_soft_bounds_frenet_point_output =
      general_lateral_decider_output.first_soft_bounds_frenet_point;
  const auto& second_soft_bounds_frenet_point_output =
      general_lateral_decider_output.second_soft_bounds_frenet_point;
  const auto ego_v =
      session_->environmental_model().get_ego_state_manager()->ego_v();
  if (ego_v < 3 / 3.6) {
    return true;
  }

  if (avoid_id_ < 0) {
    return true;
  }

  if ((avd_obstacles[0].flag != AvoidObstacleFlag::INVALID &&
       avd_obstacles[0].track_id == avoid_id_) ||
      (avd_obstacles[1].flag != AvoidObstacleFlag::INVALID &&
       avd_obstacles[1].track_id == avoid_id_)) {
    return false;
  }

  const auto obstacles_map = reference_path_ptr_->get_obstacles_map();
  if (obstacles_map.find(avoid_id_) == obstacles_map.end()) {
    return true;
  }

  // bound避让
  bool bound_nudge = false;
  bool bound_no_nudge = false;
  bool is_found_obstacle = false;
  auto check_is_avoid = [&](const BoundInfo& bound_info, double bound_value,
                            bool is_upper) {
    if (bound_nudge) {
      return;
    }

    if (bound_info.id != avoid_id_) {
      return;
    }

    is_found_obstacle = is_found_obstacle || true;
    bound_nudge = is_upper ? bound_value < 0 : bound_value > 0;
    bound_no_nudge = bound_no_nudge ||
                     (is_upper ? bound_value > kBoundBeLowCenterlineThre
                               : bound_value < -kBoundBeyondCenterlineThre);
  };

  for (int i = 0; i < first_soft_bounds_frenet_point_output.size(); ++i) {
    check_is_avoid(first_soft_bounds_info_output[i].first,
                   first_soft_bounds_frenet_point_output[i].first,
                   false);  // lower
    check_is_avoid(first_soft_bounds_info_output[i].second,
                   first_soft_bounds_frenet_point_output[i].second,
                   true);  // upper
  }
  for (int i = 0; i < second_soft_bounds_frenet_point_output.size(); ++i) {
    check_is_avoid(second_soft_bounds_info_output[i].first,
                   second_soft_bounds_frenet_point_output[i].first,
                   false);  // lower
    check_is_avoid(second_soft_bounds_info_output[i].second,
                   second_soft_bounds_frenet_point_output[i].second,
                   true);  // upper
  }

  if (bound_nudge) {
    return false;
  }

  if (bound_no_nudge || !is_found_obstacle) {
    stop_running_count_ = std::max(stop_running_count_++, kStopRunningCount);
  }

  if (stop_running_count_ >= kStopRunningCount) {
    return true;
  }
  return false;
}

void NudgeWarningHMIDecider::Reset() {
  start_running_count_ = 0;
  stop_running_count_ = 0;
  cooldown_count_ = 0;
  current_state_ = NudgeWarningState::IDLE;
}

void NudgeWarningHMIDecider::GenerateHmiOutput() {
  auto& ad_info = session_->mutable_planning_context()
                      ->mutable_planning_hmi_info()
                      ->ad_info;
  ad_info.avoid_status = avoid_id_ > 0
                             ? iflyauto::AvoidObstacle::AVOID_HIDING
                             : iflyauto::AvoidObstacle::AVOID_NO_HIDING;
  ad_info.aovid_id = avoid_id_;
  ad_info.avoiddirect =
      static_cast<iflyauto::AvoidObstacleDirection>(avoid_direction_);
}
}  // namespace planning