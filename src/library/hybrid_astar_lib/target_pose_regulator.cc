#include "target_pose_regulator.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "hybrid_astar_common.h"
#include "log_glog.h"
#include "src/modules/apa_function/apa_param_config.h"
#include "pose2d.h"

namespace planning {

#define DEBUG_DECIDER (0)

void TargetPoseRegulator::Process(EulerDistanceTransform *edt,
                                  const AstarRequest *request,
                                  const Pose2D &ego_pose,
                                  const Pose2D &center_line_target,
                                  const VehicleParam &veh_param) {
  Clear();
  center_line_target_ = center_line_target;

  if (request->path_generate_method ==
          AstarPathGenerateType::CUBIC_POLYNOMIAL_SAMPLING ||
      request->path_generate_method == AstarPathGenerateType::REEDS_SHEPP) {
    ILOG_INFO << "slot polynomial";
    return;
  }

  // Parking out no need regulator.
  if (!IsParkingIn(request)) {
    ILOG_INFO << "not park in";
    return;
  }

  if (request->space_type == ParkSpaceType::VERTICAL) {
    GenerateCandidatesForVerticalSlot(edt, request, veh_param);
  } else {
    GenerateCandidatesForParallelSlot(edt, request, veh_param);
  }

  return;
}

void TargetPoseRegulator::Process(const Pose2D &start, const Pose2D &end) {
  AstarDecider::Process(start, end);

  return;
}

const bool TargetPoseRegulator::IsParkingIn(const AstarRequest *request) {
  if (request->direction_request == ParkingVehDirection::TAIL_IN ||
      request->direction_request == ParkingVehDirection::HEAD_IN) {
    return true;
  }

  return false;
}

void TargetPoseRegulator::GenerateCandidatesForVerticalSlot(
    EulerDistanceTransform *edt, const AstarRequest *request,
    const VehicleParam &veh_param) {
  // 因为存在障碍物入侵情形，不管偏移范围设定多大，总会存在失败情况.
  // 目前策略:不删除任何障碍物，只会将目标增加平移.
  // 不要删除障碍物掩盖了上游问题.
  Pose2D global_pose;
  Transform2d tf;
  AstarPathGear gear = AstarPathGear::NONE;

  global_pose = center_line_target_;
  tf.SetBasePose(global_pose);

  double y_upper = request->slot_width / 2 - veh_param.width / 2;
  y_upper = std::max(0.0, y_upper);
  double y_lower = -request->slot_width / 2 + veh_param.width / 2;
  y_lower = std::min(0.0, y_lower);

  double y_step = 0.03;
  int y_sampling_num = std::ceil((y_upper - y_lower) / y_step) * 2;
  double y_offset = 0.0;
  double left_y_offset = 0.0;
  double right_y_offset = 0.0;

  x_check_upper_ = global_pose.x + 1.5;
  x_check_lower_ = global_pose.x;
  x_step_ = 0.2;
  x_sample_num_ = std::ceil(x_check_upper_ - x_check_lower_) / x_step_;

  float dist;

  dist = GetDistToObs(&global_pose, edt);
  PoseRegulateCandidate candidate;
  candidate.lat_offset = 0.0;
  candidate.dist_to_obs = dist;
  candidate.pose = center_line_target_;
  candidate_info_.emplace_back(candidate);

  if (dist > 0.25) {
#if DEBUG_DECIDER
    DebugString();
#endif
    return;
  }

  for (int i = 0; i < y_sampling_num; i++) {
    // left
    if (i % 2 == 0) {
      left_y_offset += y_step;
      left_y_offset = std::min(left_y_offset, y_upper);
      y_offset = left_y_offset;
    } else {
      right_y_offset -= y_step;
      right_y_offset = std::max(right_y_offset, y_lower);
      y_offset = right_y_offset;
    }

    global_pose = center_line_target_;
    global_pose.y = y_offset;

    dist = GetDistToObs(&global_pose, edt);
    if (dist > 0.06) {
      PoseRegulateCandidate candidate;
      candidate.lat_offset = y_offset;
      candidate.dist_to_obs = dist;
      candidate.pose = center_line_target_;
      candidate.pose.y = y_offset;
      candidate_info_.emplace_back(candidate);
    }

    if (dist > 0.25) {
      break;
    }
  }

#if DEBUG_DECIDER
  DebugString();
#endif

  // Todo: adjust x offset
  return;
}

void TargetPoseRegulator::Clear() {
  candidate_info_.clear();
  return;
}

const float TargetPoseRegulator::GetDistToObs(Pose2D *global_pose,
                                              EulerDistanceTransform *edt) {
  Transform2d tf;
  AstarPathGear gear = AstarPathGear::NONE;
  float dist;
  float min_dist = 10.0;

  for (int j = 0; j < x_sample_num_; j++) {
    global_pose->x = x_check_lower_ + x_step_ * j;
    tf.SetBasePose(*global_pose);

    edt->DistanceCheckForPoint(&dist, &tf, gear);

    min_dist = std::min(min_dist, dist);

    if (min_dist < 0.04) {
      break;
    }
  }

  return min_dist;
}

const bool TargetPoseRegulator::IsCandidatePoseSafe(
    const double lat_buffer) const {
  if (candidate_info_.size() <= 0) {
    return false;
  }

  for (auto &obj : candidate_info_) {
    if (obj.dist_to_obs > lat_buffer) {
      return true;
    }
  }

  return false;
}

const Pose2D TargetPoseRegulator::GetCandidatePose(
    const double lat_buffer) const {
  if (candidate_info_.size() <= 0) {
    return center_line_target_;
  }

  double dist;
  double min_dist;
  double extra_buffer = 0.05;

  for (auto &obj : candidate_info_) {
    dist = obj.dist_to_obs - lat_buffer - extra_buffer;
    if (dist > 0.0) {
      ILOG_INFO << "lat offset = " << obj.lat_offset
                << ",obs dist = " << obj.dist_to_obs;
      return obj.pose;
    }
  }

  return center_line_target_;
}

void TargetPoseRegulator::DebugString() {
  ILOG_INFO << "candidate size = " << candidate_info_.size();

  for (size_t i = 0; i < candidate_info_.size(); i++) {
    ILOG_INFO << "i = " << i << ",lat offset = " << candidate_info_[i].lat_offset
              << ",obs dist = " << candidate_info_[i].dist_to_obs;
  }

  return;
}

void TargetPoseRegulator::GenerateCandidatesForParallelSlot(
    EulerDistanceTransform *edt, const AstarRequest *request,
    const VehicleParam &veh_param) {
  // 因为存在障碍物入侵情形，不管偏移范围设定多大，总会存在失败情况.
  // 目前策略:不删除任何障碍物，只会将目标增加平移.
  // 不要删除障碍物掩盖了上游问题.
  Pose2D global_pose;
  Transform2d tf;
  AstarPathGear gear = AstarPathGear::NONE;

  global_pose = center_line_target_;
  tf.SetBasePose(global_pose);

  double y_upper = request->slot_width / 2 - veh_param.width / 2 + 0.2;
  y_upper = std::max(0.0, y_upper);
  double y_lower = -request->slot_width / 2 + veh_param.width / 2 - 0.2;
  y_lower = std::min(0.0, y_lower);

  double y_step = 0.03;
  int y_sampling_num = std::ceil((y_upper - y_lower) / y_step) * 2;
  double y_offset = 0.0;
  double left_y_offset = 0.0;
  double right_y_offset = 0.0;

  x_check_upper_ = global_pose.x + 0.6;
  x_check_lower_ = global_pose.x;
  x_step_ = 0.2;
  x_sample_num_ = std::ceil(x_check_upper_ - x_check_lower_) / x_step_;

  float dist;

  dist = GetDistToObs(&global_pose, edt);
  PoseRegulateCandidate candidate;
  candidate.lat_offset = 0.0;
  candidate.dist_to_obs = dist;
  candidate.pose = center_line_target_;
  candidate_info_.emplace_back(candidate);

  if (dist > 0.25) {
#if DEBUG_DECIDER
    DebugString();
#endif
    return;
  }

  for (int i = 0; i < y_sampling_num; i++) {
    // left
    if (i % 2 == 0) {
      left_y_offset += y_step;
      left_y_offset = std::min(left_y_offset, y_upper);
      y_offset = left_y_offset;
    } else {
      right_y_offset -= y_step;
      right_y_offset = std::max(right_y_offset, y_lower);
      y_offset = right_y_offset;
    }

    global_pose = center_line_target_;
    global_pose.y = y_offset;

    dist = GetDistToObs(&global_pose, edt);
    if (dist > 0.06) {
      PoseRegulateCandidate candidate;
      candidate.lat_offset = y_offset;
      candidate.dist_to_obs = dist;
      candidate.pose = center_line_target_;
      candidate.pose.y = y_offset;
      candidate_info_.emplace_back(candidate);
    }

    if (dist > 0.25) {
      break;
    }
  }

#if DEBUG_DECIDER
  DebugString();
#endif

  // Todo: adjust x offset
  return;
}

}  // namespace planning