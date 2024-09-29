#include "drive_distance_decider.h"
#include <cstddef>
#include "hybrid_astar_common.h"
#include "log_glog.h"

namespace planning {

void DriveDistanceDecider::Process(HybridAStarResult *history_path,
                                   const PlanningReason plan_reason) {
  next_path_info_.gear_ = AstarPathGear::none;
  next_path_info_.gear_switch_number_ = PathGearSwitchNumber::NONE;
  next_path_info_.dist_ = 0;

  ILOG_INFO << "plan reason=" << static_cast<int>(plan_reason);

  if (history_path == nullptr) {
    return;
  }

  if (history_path->x.size() <= 1) {
    return;
  }

  // 如果path被障碍物阻挡，不好推理下次path的信息
  if (plan_reason == PlanningReason::PATH_STUCKED) {
    if (history_path->gear.size() > 0) {
      if (history_path->gear[0] == AstarPathGear::drive) {
        next_path_info_.gear_ = AstarPathGear::reverse;
      } else {
        next_path_info_.gear_ = AstarPathGear::drive;
      }
    }
    next_path_info_.gear_switch_number_ = PathGearSwitchNumber::MANY_TIMES;

    return;
  }

  AstarPathGear first_point_gear = history_path->gear[0];
  AstarPathGear second_path_gear = AstarPathGear::none;

  next_path_info_.start_point_id_ = 0;
  next_path_info_.end_point_id_ = 0;
  next_path_info_.start_point_s_ = 0;
  size_t accumulated_s_size = 1;
  PathGearSwitchNumber gear_numer = PathGearSwitchNumber::NONE;

  if (history_path->x.size() > 0) {
    size_t x_size = history_path->x.size();
    size_t y_size = history_path->y.size();
    size_t phi_size = history_path->phi.size();
    size_t gear_size = history_path->gear.size();
    accumulated_s_size = history_path->accumulated_s.size();

    for (size_t i = 0; i < x_size; i++) {
      if (i >= gear_size || i >= x_size || i >= y_size || i >= phi_size ||
          i >= accumulated_s_size) {
        ILOG_ERROR << "point size " << i;
        break;
      }

      if (second_path_gear == AstarPathGear::none) {
        if (history_path->gear[i] == first_point_gear) {
          continue;
        }

        // gear is change
        second_path_gear = history_path->gear[i];

        next_path_info_.start_point_id_ = i;
        next_path_info_.start_point_s_ = history_path->accumulated_s[i];

        gear_numer = PathGearSwitchNumber::ONCE;
      } else {
        next_path_info_.end_point_id_ = i;
        next_path_info_.end_point_s_ = history_path->accumulated_s[i];

        // multi shot path
        if (second_path_gear != history_path->gear[i]) {
          gear_numer = PathGearSwitchNumber::MANY_TIMES;
          break;
        }
      }
    }
  }

  next_path_info_.gear_ = second_path_gear;

  // 下次path的挡位数量
  switch (gear_numer) {
    case PathGearSwitchNumber::NONE:
    case PathGearSwitchNumber::ONCE:
      next_path_info_.gear_switch_number_ = PathGearSwitchNumber::NONE;
      break;
    default:
      next_path_info_.gear_switch_number_ = PathGearSwitchNumber::MANY_TIMES;
      break;
  }

  next_path_info_.dist_ =
      history_path->accumulated_s[next_path_info_.end_point_id_] -
      history_path->accumulated_s[next_path_info_.start_point_id_];

  ILOG_INFO << "single shot parking decision, gear: "
            << PathGearDebugString(second_path_gear) << " ,dist "
            << next_path_info_.dist_ << " ,shot number: "
            << PathGearSwitchNumberString(next_path_info_.gear_switch_number_);

  return;
}

const AstarPathGear DriveDistanceDecider::GetNextPathGear() {
  return next_path_info_.gear_;
}

const double DriveDistanceDecider::GetNextPathLength() {
  return next_path_info_.dist_;
}

const size_t DriveDistanceDecider::GetNextPathStartPointId() {
  return next_path_info_.start_point_id_;
}

const bool DriveDistanceDecider::IsNextPathNoGearSwitch() {
  if (next_path_info_.gear_switch_number_ == PathGearSwitchNumber::NONE) {
    return true;
  }

  return false;
}

void DriveDistanceDecider::Process(const Pose2D &start, const Pose2D &end) {
  AstarDecider::Process(start, end);

  return;
}

std::string DriveDistanceDecider::PathGearSwitchNumberString(
    const PathGearSwitchNumber &gear_number) {
  switch (gear_number) {
    case PathGearSwitchNumber::NONE:
      return "none";
    case PathGearSwitchNumber::ONCE:
      return "single_shot_path";
    case PathGearSwitchNumber::MANY_TIMES:
      return "multi_shot_path";
    default:
      break;
  }

  return "error shot number";
}

}  // namespace planning