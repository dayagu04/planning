#include "future_path_decider.h"

#include <bits/stdint-intn.h>

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "collision_detect_types.h"
#include "hybrid_astar_common.h"
#include "log_glog.h"
#include "pose2d.h"
#include "transform2d.h"
#include "vecf32.h"

namespace planning {

void FuturePathDecider::Clear() {
  path_check_dist_ = 3.0;
  point_resolution_ = 0.1;
  future_drive_dist_info_.dist_to_ref_line = 0.0;
  future_drive_dist_info_.gear_drive_path.clear();

  float s = 0.0;
  int32_t size = std::ceil(path_check_dist_ / point_resolution_);
  for (int32_t i = 0; i < size; i++) {
    future_drive_dist_info_.gear_drive_path.emplace_back(
        Eigen::Vector2f(s, 10.0));
    s += point_resolution_;
  }

  future_drive_dist_info_.gear_reverse_path.clear();

  s = 0.0;
  for (int32_t i = 0; i < size; i++) {
    future_drive_dist_info_.gear_reverse_path.emplace_back(
        Eigen::Vector2f(s, 10.0));
    s += point_resolution_;
  }

  return;
}

void FuturePathDecider::Process(const HybridAStarResult *history_path,
                                const ParkReferenceLine *ref_line,
                                const float min_turn_radius,
                                const float sampling_lon_resolution,
                                EulerDistanceTransform *edt,
                                AstarRequest &request) {
  if (path_generate_type_ == AstarPathGenerateType::TRY_SEARCHING) {
    request.first_action_request.Clear();
    return;
  }

  min_turn_radius_ = min_turn_radius;
  swap_start_goal_ = request.swap_start_goal;
  path_generate_type_ = request.path_generate_method;
  gear_request_ = request.first_action_request.gear_request;
  sampling_lon_resolution_ = sampling_lon_resolution;
  path_inference_lat_buffer_ = 0.1;
  Clear();

  edt->UpdateSafeBuffer(0.01, 0.01, 0.4);

  CalcDriveDistByLineModel(request.start_, edt, ref_line);

  // If ego need zigzag path to adjust pose, use line model to estimate path
  // length.
  if (!IsNeedZigZagPathToAdjustPose(request)) {
    CalcDriveDistByCircleModel(request.start_, edt);
  }

  UpdateFuturePathRequest(&request.first_action_request);

  return;
}

void FuturePathDecider::Process(const Pose2D &start, const Pose2D &end) {
  AstarDecider::Process(start, end);

  return;
}

void FuturePathDecider::CalcDriveDistByLineModel(
    const Pose2D &ego_pose, EulerDistanceTransform *edt,
    const ParkReferenceLine *ref_line) {
  // update dist to ref line
  Vec2df32 ego_line_segment =
      Vec2df32(ego_pose.x, ego_pose.y) - ref_line->GetStartPoint();

  float vertical_dist = ref_line->UnitDirection().CrossProd(ego_line_segment);
  future_drive_dist_info_.dist_to_ref_line = std::fabs(vertical_dist);

  // update gear drive safe dist
  Pose2D global_pose;
  Transform2d tf;
  float sin_theta = std::sin(ego_pose.theta);
  float cos_theta = std::cos(ego_pose.theta);

  const Eigen::Vector2f line_seg_start =
      Eigen::Vector2f(ego_pose.x, ego_pose.y);
  const Eigen::Vector2f unit_line_vec = Eigen::Vector2f(cos_theta, sin_theta);

  float s = 0.0;
  Eigen::Vector2f point;
  float obs_dist;
  for (int32_t i = 0; i < future_drive_dist_info_.gear_drive_path.size(); i++) {
    point = line_seg_start + s * unit_line_vec;

    global_pose.x = point[0];
    global_pose.y = point[1];
    global_pose.theta = ego_pose.theta;
    tf.SetBasePose(global_pose, sin_theta, cos_theta);

    edt->DistanceCheckForPoint(&obs_dist, &tf, AstarPathGear::DRIVE);

    if (obs_dist < future_drive_dist_info_.gear_drive_path[i][1]) {
      future_drive_dist_info_.gear_drive_path[i][1] = obs_dist;
    }

    if (obs_dist < 0.001) {
      break;
    }

    s += point_resolution_;
  }

  // gear reverse drive dist
  s = 0.0;
  for (int32_t i = 0; i < future_drive_dist_info_.gear_reverse_path.size();
       i++) {
    point = line_seg_start - s * unit_line_vec;

    global_pose.x = point[0];
    global_pose.y = point[1];
    global_pose.theta = ego_pose.theta;
    tf.SetBasePose(global_pose, sin_theta, cos_theta);

    edt->DistanceCheckForPoint(&obs_dist, &tf, AstarPathGear::REVERSE);

    if (obs_dist < future_drive_dist_info_.gear_reverse_path[i][1]) {
      future_drive_dist_info_.gear_reverse_path[i][1] = obs_dist;
    }

    if (obs_dist < 0.001) {
      break;
    }
    s += point_resolution_;
  }

  ILOG_INFO << "dist to ref line=" << future_drive_dist_info_.dist_to_ref_line;

  return;
}

void FuturePathDecider::CalcDriveDistByCircleModel(
    const Pose2D &ego_pose, EulerDistanceTransform *edt) {
  // update path safe dist
  std::vector<Pose2D> path;

  // left turn
  GetPathByCircle(&ego_pose, path_check_dist_, min_turn_radius_, true, &path);
  UpdatePathDistInfo(path, AstarPathGear::DRIVE, edt,
                     future_drive_dist_info_.gear_drive_path);

  // right turn
  GetPathByCircle(&ego_pose, path_check_dist_, -min_turn_radius_, true, &path);
  UpdatePathDistInfo(path, AstarPathGear::DRIVE, edt,
                     future_drive_dist_info_.gear_drive_path);

  // gear reverse drive dist
  // left
  GetPathByCircle(&ego_pose, path_check_dist_, min_turn_radius_, false, &path);
  UpdatePathDistInfo(path, AstarPathGear::REVERSE, edt,
                     future_drive_dist_info_.gear_reverse_path);

  // right
  GetPathByCircle(&ego_pose, path_check_dist_, -min_turn_radius_, false, &path);
  UpdatePathDistInfo(path, AstarPathGear::REVERSE, edt,
                     future_drive_dist_info_.gear_reverse_path);

  return;
}

void FuturePathDecider::UpdateFuturePathRequest(
    ParkFirstActionRequest *future_path_request) {
  if (swap_start_goal_) {
    future_path_request->dist_request = 0.0;
    future_path_request->has_request = false;
    future_path_request->gear_request = AstarPathGear::NONE;
    return;
  }

  // advised path: drive to ref line.
  future_path_request->dist_request =
      std::max(future_drive_dist_info_.dist_to_ref_line, 1.2f);

  future_drive_dist_info_.advised_gear_drive_dist = SearchAdvisedDriveDist(
      path_inference_lat_buffer_, future_drive_dist_info_.gear_drive_path);
  future_drive_dist_info_.advised_gear_reverse_dist = SearchAdvisedDriveDist(
      path_inference_lat_buffer_, future_drive_dist_info_.gear_reverse_path);

  ILOG_INFO << "drive gear dist = "
            << future_drive_dist_info_.advised_gear_drive_dist
            << ", reverse gear dist = "
            << future_drive_dist_info_.advised_gear_reverse_dist;

  if (future_path_request->gear_request == AstarPathGear::DRIVE) {
    if (future_drive_dist_info_.advised_gear_drive_dist <
        future_path_request->dist_request) {
      future_path_request->dist_request =
          future_drive_dist_info_.advised_gear_drive_dist;
    }
  } else if (future_path_request->gear_request == AstarPathGear::REVERSE) {
    if (future_drive_dist_info_.advised_gear_reverse_dist <
        future_path_request->dist_request) {
      future_path_request->dist_request =
          future_drive_dist_info_.advised_gear_reverse_dist;
    }
  } else {
    if (future_drive_dist_info_.advised_gear_reverse_dist <
        future_path_request->dist_request) {
      future_path_request->dist_request =
          future_drive_dist_info_.advised_gear_reverse_dist;
    }
    if (future_drive_dist_info_.advised_gear_drive_dist <
        future_path_request->dist_request) {
      future_path_request->dist_request =
          future_drive_dist_info_.advised_gear_drive_dist;
    }
  }

  future_path_request->dist_request =
      std::floor(future_path_request->dist_request / sampling_lon_resolution_) *
      sampling_lon_resolution_;

  return;
}

// radius: if left turn, radius is positive
void FuturePathDecider::GetVehCircleByPose(const Pose2D *pose,
                                           const float radius,
                                           const AstarPathGear gear,
                                           VehicleCircle *veh_circle) {
  veh_circle->radius = radius;
  veh_circle->gear = gear;

  veh_circle->center.x = pose->x - radius * std::sin(pose->theta);
  veh_circle->center.y = pose->y + radius * std::cos(pose->theta);

  return;
}

void FuturePathDecider::GetPathByCircle(const Pose2D *start_point_pose,
                                        const float arc, const float radius,
                                        const bool is_forward,
                                        std::vector<Pose2D> *path) {
  int path_point_num = std::ceil(arc / 0.1);

  // get vehicle circle
  VehicleCircle veh_circle;
  AstarPathGear gear;
  if (is_forward) {
    gear = AstarPathGear::DRIVE;
  } else {
    gear = AstarPathGear::REVERSE;
  }

  GetVehCircleByPose(start_point_pose, radius, gear, &veh_circle);

  // interpolate
  Pose2D next_pose;
  float inv_radius = std::fabs(1.0 / radius);

  float acc_s = 0.0;

  path->clear();
  path->push_back(*start_point_pose);

  for (int i = 0; i < path_point_num; ++i) {
    acc_s += 0.1;

    InterpolateByArcOffset(&veh_circle, start_point_pose, acc_s, inv_radius,
                           &next_pose);

    path->push_back(next_pose);
  }

  return;
}

void FuturePathDecider::InterpolateByArcOffset(const VehicleCircle *veh_circle,
                                               const Pose2D *start_pose,
                                               const float arc,
                                               const float inverse_radius,
                                               Pose2D *pose) {
  float delta_theta, theta;

  delta_theta = arc * inverse_radius;

  // left turn
  if (veh_circle->radius > 0.0) {
    if (veh_circle->gear == AstarPathGear::REVERSE) {
      delta_theta = -delta_theta;
    }
  } else {
    // right turn, gear is d
    if (veh_circle->gear == AstarPathGear::DRIVE) {
      delta_theta = -delta_theta;
    }
  }

  // update next point theta
  theta = start_pose->theta + delta_theta;

  float radius = veh_circle->radius;

  pose->x = veh_circle->center.x + radius * std::sin(theta);
  pose->y = veh_circle->center.y - radius * std::cos(theta);
  pose->theta = IflyUnifyTheta(theta, M_PI);

  return;
}

void FuturePathDecider::GetStraightLinePoint(const Pose2D *start_state,
                                             const float dist_to_start,
                                             const Pose2D *unit_vector,
                                             Pose2D *goal_state) {
  goal_state->x = start_state->x + dist_to_start * unit_vector->x;
  goal_state->y = start_state->y + dist_to_start * unit_vector->y;

  goal_state->theta = start_state->theta;

  return;
}

void FuturePathDecider::GetPathByLine(const Pose2D *start_pose,
                                      const float length, const bool is_forward,
                                      std::vector<Pose2D> *path) {
  int path_point_num = std::ceil(length / 0.1);

  float inc_dist;
  if (is_forward) {
    inc_dist = 0.1;
  } else {
    inc_dist = -0.1;
  }

  float acc_s = 0.0;

  // get unit vector
  Pose2D unit_vector;
  unit_vector.x = std::cos(start_pose->theta);
  unit_vector.y = std::sin(start_pose->theta);

  Pose2D next_pose;

  path->clear();
  path->push_back(*start_pose);

  for (int j = 0; j < path_point_num; j++) {
    acc_s += inc_dist;

    GetStraightLinePoint(start_pose, acc_s, &unit_vector, &next_pose);
    path->push_back(next_pose);
  }

  return;
}

// if left, radius is positive
void FuturePathDecider::GetPathByRadius(const Pose2D *start_pose,
                                        const float length, const float radius,
                                        const bool is_forward,
                                        std::vector<Pose2D> *path) {
  if (std::fabs(radius) > 100000.0) {
    GetPathByLine(start_pose, length, is_forward, path);

  } else {
    GetPathByCircle(start_pose, length, radius, is_forward, path);
  }

  return;
}

int32_t FuturePathDecider::SearchIdByS(
    const float s, const std::vector<Eigen::Vector2f> &path) {
  if (path.empty()) {
    return -1;
  }

  int32_t path_end_id = static_cast<int32_t>(path.size()) - 1;
  int32_t guess_id = std::round(s / point_resolution_);

  int32_t search_start_id = guess_id - 3;
  search_start_id = std::max(search_start_id, 0);

  int32_t search_end_id = guess_id + 3;
  search_end_id = std::min(search_end_id, path_end_id);

  float min_dist = 1000.0;
  int32_t min_dist_id = guess_id;
  for (int32_t i = search_start_id; i <= search_end_id; i++) {
    if (std::fabs(path[i][0] - s) < min_dist) {
      min_dist = std::fabs(path[i][0] - s);
      min_dist_id = i;
    }
  }

  if (min_dist > 0.05) {
    return -1;
  }

  return min_dist_id;
}

const float FuturePathDecider::SearchAdvisedDriveDist(
    const float safe_buffer, const std::vector<Eigen::Vector2f> &path) const {
  if (path.empty()) {
    return 0.0f;
  }

  for (size_t i = 0; i < path.size(); i++) {
    if (path[i][1] < safe_buffer) {
      if (i == 0) {
        return 0.0;
      }
      return path[i - 1][0];
    }
  }

  return path[path.size() - 1][0];
}

void FuturePathDecider::UpdatePathDistInfo(
    const std::vector<Pose2D> &path, const AstarPathGear gear,
    EulerDistanceTransform *edt, std::vector<Eigen::Vector2f> &path_dist_info) {
  float obs_dist;
  Transform2d tf;
  for (size_t i = 0; i < path.size(); i++) {
    tf.SetBasePose(path[i]);
    edt->DistanceCheckForPoint(&obs_dist, &tf, gear);

    if (i >= path_dist_info.size()) {
      break;
    }

    if (obs_dist < path_dist_info[i][1]) {
      path_dist_info[i][1] = obs_dist;
    }
  }

  return;
}

}  // namespace planning