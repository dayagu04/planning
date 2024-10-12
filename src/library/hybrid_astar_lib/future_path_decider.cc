#include "future_path_decider.h"

#include <cmath>
#include <cstddef>

#include "hybrid_astar_common.h"
#include "log_glog.h"

namespace planning {

void FuturePathDecider::Process(const HybridAStarResult *history_path,
                                const PlanningReason plan_reason,
                                const Pose2D &ego_pose,
                                EulerDistanceTransform *edt,
                                const ParkReferenceLine *ref_line,
                                ParkFirstActionRequest *future_path_request) {
  ILOG_INFO << "plan reason=" << static_cast<int>(plan_reason);

  CalcDriveDistByLineModel(ego_pose, edt, ref_line);

  CalcDriveDistByHistoryPath(history_path, plan_reason);

  UpdateFuturePathRequest(future_path_request);

  return;
}

const AstarPathGear FuturePathDecider::GetNextPathGearByHistory() {
  return history_path_info_.gear_;
}

const double FuturePathDecider::GetNextPathLenByHistory() {
  return history_path_info_.dist_;
}

const size_t FuturePathDecider::GetNextPathStartPointId() {
  return history_path_info_.start_point_id_;
}

const bool FuturePathDecider::IsNextPathNoGearSwitchByHistory() {
  if (history_path_info_.gear_switch_number_ == PathGearSwitchNumber::NONE) {
    return true;
  }

  return false;
}

void FuturePathDecider::Process(const Pose2D &start, const Pose2D &end) {
  AstarDecider::Process(start, end);

  return;
}

std::string FuturePathDecider::PathGearSwitchNumberString(
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

void FuturePathDecider::CalcDriveDistByLineModel(
    const Pose2D &ego_pose, EulerDistanceTransform *edt,
    const ParkReferenceLine *ref_line) {
  // update dist to ref line
  ad_common::math::Vec2d ego_line_segment =
      ad_common::math::Vec2d(ego_pose.x, ego_pose.y) -
      ref_line->GetStartPoint();

  double vertical_dist = ref_line->UnitDirection().CrossProd(ego_line_segment);
  future_drive_dist_info_.dist_to_ref_line = std::fabs(vertical_dist);

  // update gear drive safe dist
  Pose2D global_pose;
  Transform2d tf;
  double sin_theta = std::sin(ego_pose.theta);
  double cos_theta = std::cos(ego_pose.theta);

  const Eigen::Vector2d line_seg_start =
      Eigen::Vector2d(ego_pose.x, ego_pose.y);
  const Eigen::Vector2d unit_line_vec = Eigen::Vector2d(cos_theta, sin_theta);
  double s = 0.01;
  double ds = 0.1;

  Eigen::Vector2d point;
  future_drive_dist_info_.gear_drive_has_obs = false;
  while (s < 10.0) {
    point = line_seg_start + s * unit_line_vec;

    global_pose.x = point[0];
    global_pose.y = point[1];
    global_pose.theta = ego_pose.theta;
    tf.SetBasePose(global_pose, sin_theta, cos_theta);

    // if (edt_->IsCollisionForPoint(&tf, AstarPathGear::none)) {
    if (edt->IsCollisionForPoint(&tf, AstarPathGear::drive)) {
      future_drive_dist_info_.gear_drive_has_obs = true;
      break;
    }

    s += ds;
  }

  future_drive_dist_info_.gear_drive_dist_to_obs = s - 0.1;

  // gear reverse drive dist
  s = 0.01;
  ds = 0.1;
  future_drive_dist_info_.gear_reverse_has_obs = false;
  while (s < 5.0) {
    point = line_seg_start - s * unit_line_vec;

    global_pose.x = point[0];
    global_pose.y = point[1];
    global_pose.theta = ego_pose.theta;
    tf.SetBasePose(global_pose, sin_theta, cos_theta);

    // if (edt_->IsCollisionForPoint(&tf, AstarPathGear::none)) {
    if (edt->IsCollisionForPoint(&tf, AstarPathGear::reverse)) {
      future_drive_dist_info_.gear_reverse_has_obs = true;

      break;
    }

    s += ds;
  }
  future_drive_dist_info_.gear_reverse_dist_to_obs = s - 0.1;
  future_drive_dist_info_.advised_drive_dist = 1.2;

  ILOG_INFO << "dist to ref line=" << future_drive_dist_info_.dist_to_ref_line
            << ",d, dist=" << future_drive_dist_info_.gear_drive_dist_to_obs
            << ",r dist=" << future_drive_dist_info_.gear_reverse_dist_to_obs;

  return;
}

void FuturePathDecider::CalcDriveDistByHistoryPath(
    const HybridAStarResult *history_path, const PlanningReason plan_reason) {
  history_path_info_.gear_ = AstarPathGear::none;
  history_path_info_.gear_switch_number_ = PathGearSwitchNumber::NONE;
  history_path_info_.dist_ = 0;

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
        history_path_info_.gear_ = AstarPathGear::reverse;
      } else {
        history_path_info_.gear_ = AstarPathGear::drive;
      }
    }
    history_path_info_.gear_switch_number_ = PathGearSwitchNumber::MANY_TIMES;

    return;
  }

  AstarPathGear first_point_gear = history_path->gear[0];
  AstarPathGear second_path_gear = AstarPathGear::none;

  history_path_info_.start_point_id_ = 0;
  history_path_info_.end_point_id_ = 0;
  history_path_info_.start_point_s_ = 0;
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

        history_path_info_.start_point_id_ = i;
        history_path_info_.start_point_s_ = history_path->accumulated_s[i];

        gear_numer = PathGearSwitchNumber::ONCE;
      } else {
        history_path_info_.end_point_id_ = i;
        history_path_info_.end_point_s_ = history_path->accumulated_s[i];

        // multi shot path
        if (second_path_gear != history_path->gear[i]) {
          gear_numer = PathGearSwitchNumber::MANY_TIMES;
          break;
        }
      }
    }
  }

  history_path_info_.gear_ = second_path_gear;

  // 下次path的挡位数量
  switch (gear_numer) {
    case PathGearSwitchNumber::NONE:
    case PathGearSwitchNumber::ONCE:
      history_path_info_.gear_switch_number_ = PathGearSwitchNumber::NONE;
      break;
    default:
      history_path_info_.gear_switch_number_ = PathGearSwitchNumber::MANY_TIMES;
      break;
  }

  history_path_info_.dist_ =
      history_path->accumulated_s[history_path_info_.end_point_id_] -
      history_path->accumulated_s[history_path_info_.start_point_id_];

  ILOG_INFO << "drive distance decider, gear: "
            << PathGearDebugString(second_path_gear) << " ,dist "
            << history_path_info_.dist_ << " ,shot number: "
            << PathGearSwitchNumberString(
                   history_path_info_.gear_switch_number_);

  return;
}

void FuturePathDecider::UpdateFuturePathRequest(
    ParkFirstActionRequest *future_path_request) {
  // advised path: drive to ref line.
  future_path_request->dist_request =
      std::max(future_drive_dist_info_.dist_to_ref_line,
               future_drive_dist_info_.advised_drive_dist);

  if (future_path_request->gear_request == AstarPathGear::drive) {
    if (future_drive_dist_info_.gear_drive_dist_to_obs <
        future_path_request->dist_request) {
      future_path_request->dist_request =
          future_drive_dist_info_.gear_drive_dist_to_obs;
    }
  } else if (future_path_request->gear_request == AstarPathGear::reverse) {
    if (future_drive_dist_info_.gear_reverse_dist_to_obs <
        future_path_request->dist_request) {
      future_path_request->dist_request =
          future_drive_dist_info_.gear_reverse_dist_to_obs;
    }
  }

  return;
}

}  // namespace planning