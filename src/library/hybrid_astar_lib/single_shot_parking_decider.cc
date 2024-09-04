#include "single_shot_parking_decider.h"
#include <cstddef>
#include "hybrid_astar_common.h"
#include "log_glog.h"

namespace planning {

void SingleShotParkingDecider::Process(HybridAStarResult *history_path) {
  next_shot_path_info_.gear_ = AstarPathGear::none;
  next_shot_path_info_.shot_number_ = PathShotNumber::none;
  next_shot_path_info_.dist_ = 0;

  if (history_path == nullptr) {
    return;
  }

  if (history_path->x.size() <= 1) {
    return;
  }

  AstarPathGear first_point_gear = history_path->gear[0];
  AstarPathGear second_path_gear = AstarPathGear::none;

  next_shot_path_info_.start_point_id_ = 0;
  next_shot_path_info_.end_point_id_ = 0;
  next_shot_path_info_.start_point_s_ = 0;
  size_t accumulated_s_size = 1;
  PathShotNumber shot_numer = PathShotNumber::none;

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

        next_shot_path_info_.start_point_id_ = i;
        next_shot_path_info_.start_point_s_ = history_path->accumulated_s[i];

        shot_numer = PathShotNumber::single_shot_path;
      } else {
        next_shot_path_info_.end_point_id_ = i;
        next_shot_path_info_.end_point_s_ = history_path->accumulated_s[i];

        // multi shot path
        if (second_path_gear != history_path->gear[i]) {
          shot_numer = PathShotNumber::multi_shot_path;
          break;
        }
      }
    }
  }

  next_shot_path_info_.gear_ = second_path_gear;
  next_shot_path_info_.shot_number_ = shot_numer;

  next_shot_path_info_.dist_ =
      history_path->accumulated_s[next_shot_path_info_.end_point_id_] -
      history_path->accumulated_s[next_shot_path_info_.start_point_id_];

  ILOG_INFO << "single shot parking decision, gear: "
            << PathGearDebugString(second_path_gear) << " ,dist "
            << next_shot_path_info_.dist_ << " ,shot number: "
            << PathShotNumberString(next_shot_path_info_.shot_number_);

  return;
}

const AstarPathGear SingleShotParkingDecider::GetNextShotGear() {
  return next_shot_path_info_.gear_;
}

const double SingleShotParkingDecider::GetNextShotPathLength() {
  return next_shot_path_info_.dist_;
}

const size_t SingleShotParkingDecider::GetNextShotStartPointId() {
  return next_shot_path_info_.start_point_id_;
}

const bool SingleShotParkingDecider::IsSingleShotPath() {
  if (next_shot_path_info_.shot_number_ == PathShotNumber::single_shot_path) {
    return true;
  }

  return false;
}

void SingleShotParkingDecider::Process(const Pose2D &start, const Pose2D &end) {
  AstarDecider::Process(start, end);

  return;
}

std::string SingleShotParkingDecider::PathShotNumberString(
    const PathShotNumber &shot_number) {
  switch (shot_number) {
    case PathShotNumber::none:
      return "none";
    case PathShotNumber::single_shot_path:
      return "single_shot_path";
    case PathShotNumber::multi_shot_path:
      return "multi_shot_path";

    default:

      break;
  }

  return "error shot number";
}

}  // namespace planning