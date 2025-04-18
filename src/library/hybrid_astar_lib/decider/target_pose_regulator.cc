#include "target_pose_regulator.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "hybrid_astar_common.h"
#include "log_glog.h"
#include "math_utils.h"
#include "pose2d.h"

namespace planning {

#define DEBUG_DECIDER (0)
#define SAFE_ENOUGH_BUFFER (0.5)

bool TargetPoseRegulator::IsDefaultPoseSafeEnough() {
  if (candidate_info_.size() > 0 &&
      candidate_info_[0].dist_to_obs > SAFE_ENOUGH_BUFFER) {
    return true;
  }

  return false;
}

void TargetPoseRegulator::UpdateDefaultPoseInfo(const AstarRequest *request,
                                                const VehicleParam &veh_param,
                                                EulerDistanceTransform *edt) {
  float min_passage_width = 2.5;
  if (request->space_type == ParkSpaceType::VERTICAL) {
    if (request->direction_request == ParkingVehDirection::TAIL_IN) {
      float veh_x_upper = min_passage_width + request->slot_length -
                       veh_param.front_edge_to_rear_axle;
      x_check_upper_ = std::max(center_line_target_.x, veh_x_upper);
    } else {
      // 对于车头入库，需要检查更大的范围. 让后视镜经过柱子.
      // todo: 使用新的方式，加速这里的计算. 采样的计算方式并不快.
      float veh_x_upper = min_passage_width + request->slot_length -
                          veh_param.rear_edge_to_rear_axle;
      x_check_upper_ = std::max(center_line_target_.x, veh_x_upper);
    }
  } else {
    x_check_upper_ = center_line_target_.x + 0.6;
  }

  x_check_lower_ = center_line_target_.x;
  x_step_ = 0.2;
  x_sample_num_ = std::ceil(x_check_upper_ - x_check_lower_) / x_step_;

  float dist = GetDistToObs(&center_line_target_, edt);
  PoseRegulateCandidate candidate;
  candidate.lat_offset = 0.0;
  candidate.dist_to_obs = dist;
  candidate.pose = center_line_target_;
  candidate_info_.emplace_back(candidate);

  // update ego dist
  Transform2d tf;
  tf.SetBasePose(request->start_);
  AstarPathGear gear = AstarPathGear::NONE;
  edt->DistanceCheckForPoint(&dist, &tf, gear);
  ego_dist_to_obs_ = static_cast<float>(dist);

#if DEBUG_DECIDER
    DebugString();

    dist = GetDistToObs(&request->start_, edt);
    ILOG_INFO << "start point obs dist = " << dist;
#endif

  return;
}

void TargetPoseRegulator::Process(EulerDistanceTransform *edt,
                                  const AstarRequest *request,
                                  const Pose2D &ego_pose,
                                  const Pose2D &center_line_target,
                                  const VehicleParam &veh_param) {
  Clear();
  center_line_target_ = center_line_target;
  request_ = request;
  edt->UpdateSafeBuffer(0.0, 0.0, 0.0);
  UpdateDefaultPoseInfo(request, veh_param, edt);

  if (request->path_generate_method ==
          AstarPathGenerateType::CUBIC_POLYNOMIAL_SAMPLING ||
      request->path_generate_method ==
          AstarPathGenerateType::REEDS_SHEPP_SAMPLING) {
    ILOG_INFO << "slot polynomial";
    return;
  }

  // Parking out no need regulator.
  if (!IsParkingIn(request)) {
    ILOG_INFO << "not park in";
    return;
  }

  if (IsDefaultPoseSafeEnough()) {
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
  global_pose = center_line_target_;

  float y_upper = request->slot_width / 2 - veh_param.width / 2;
  y_upper = std::max(0.0f, y_upper);
  float y_lower = -request->slot_width / 2 + veh_param.width / 2;
  y_lower = std::min(0.0f, y_lower);

  float y_step = 0.03;
  int y_sampling_num = std::ceil((y_upper - y_lower) / y_step) * 2;
  float y_offset = 0.0;
  float left_y_offset = 0.0;
  float right_y_offset = 0.0;

  float dist;
  PoseRegulateCandidate candidate;
  bool check_left = true;

  for (int i = 0; i < y_sampling_num; i++) {
    // left
    if (check_left) {
      check_left = false;
      if (left_y_offset < y_upper) {
        left_y_offset += y_step;
        left_y_offset = std::min(left_y_offset, y_upper);
        y_offset = left_y_offset;
      } else {
        continue;
      }
    } else {
      check_left = true;
      if (right_y_offset > y_lower) {
        right_y_offset -= y_step;
        right_y_offset = std::max(right_y_offset, y_lower);
        y_offset = right_y_offset;
      } else {
        continue;
      }
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

#if DEBUG_DECIDER
    ILOG_INFO << "offset = " << y_offset << ", dist = " << dist;
#endif

    if (dist > SAFE_ENOUGH_BUFFER) {
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

const float TargetPoseRegulator::GetDistToObs(const Pose2D *global_pose,
                                              EulerDistanceTransform *edt) {
  Transform2d tf;
  AstarPathGear gear = AstarPathGear::NONE;
  float dist;
  float min_dist = 10.0;
  Pose2D pose = *global_pose;

  for (int j = 0; j < x_sample_num_; j++) {
    pose.x = x_check_lower_ + x_step_ * j;
    tf.SetBasePose(pose);

    edt->DistanceCheckForPoint(&dist, &tf, gear);

    min_dist = std::min(min_dist, dist);

    if (min_dist < 0.04) {
      break;
    }
  }

  return min_dist;
}

const bool TargetPoseRegulator::IsCandidatePoseSafe(
    const float lat_buffer) const {
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

const std::pair<Pose2D, float> TargetPoseRegulator::GetCandidatePose(
    const float lat_buffer) const {
  if (request_->direction_request == ParkingVehDirection::TAIL_IN ) {
    return GetCandidatePoseForTailIn(lat_buffer);
  }

  return GetCandidatePoseForHeadIn(lat_buffer);
}

void TargetPoseRegulator::DebugString() {
  ILOG_INFO << "candidate size = " << candidate_info_.size();

  for (size_t i = 0; i < candidate_info_.size(); i++) {
    ILOG_INFO << "i = " << i
              << ",lat offset = " << candidate_info_[i].lat_offset
              << ",obs dist = " << candidate_info_[i].dist_to_obs;

    candidate_info_[i].pose.DebugString();
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
  AstarPathGear gear = AstarPathGear::NONE;
  global_pose = center_line_target_;

  float y_upper = request->slot_width / 2 - veh_param.width / 2 + 0.2f;
  y_upper = std::max(0.0f, y_upper);
  float y_lower = -request->slot_width / 2 + veh_param.width / 2 - 0.2f;
  y_lower = std::min(0.0f, y_lower);

  float y_step = 0.03;
  int y_sampling_num = std::ceil((y_upper - y_lower) / y_step) * 2;
  float y_offset = 0.0;
  float left_y_offset = 0.0;
  float right_y_offset = 0.0;

  float dist;
  PoseRegulateCandidate candidate;

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
    if (dist > 0.06f) {
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

const std::pair<Pose2D, float> TargetPoseRegulator::GetCandidatePoseForTailIn(
    const float lat_buffer) const {
  if (candidate_info_.size() <= 0) {
    return std::make_pair(center_line_target_, 0.0);
  }

  float dist;
  float extra_buffer = 0.05;
  const PoseRegulateCandidate *best_candidate = &candidate_info_[0];

  for (auto &obj : candidate_info_) {
    // If pose is big buffer, return
    dist = obj.dist_to_obs - lat_buffer - extra_buffer;
    if (dist > 0.0) {
      ILOG_INFO << "big buffer, lat offset = " << obj.lat_offset
                << ",obs dist = " << obj.dist_to_obs;
      return std::make_pair(obj.pose, obj.dist_to_obs);
    }

    // get relative safe pose.
    if (obj.dist_to_obs > best_candidate->dist_to_obs) {
      best_candidate = &obj;
    }
  }

  return std::make_pair(best_candidate->pose, best_candidate->dist_to_obs);
}

const int TargetPoseRegulator::GenerateOffsetPreference() const {
  if (request_->direction_request != ParkingVehDirection::HEAD_IN) {
    return 0;
  }

  // 车辆在[0-170], [-170~0],说明车辆需要打方向盘调整入库，此时需要故意将目标点增加一个偏移量
  if (std::fabs(request_->start_.theta) < ifly_deg2rad(170.0)) {
    // right
    if (request_->start_.theta > 0.0) {
      return 1;
    } else {
      return -1;
    }
  }

  return 0;
}

const std::pair<Pose2D, float> TargetPoseRegulator::GetCandidatePoseForHeadIn(
    const float lat_buffer) const {
  if (candidate_info_.size() <= 0) {
    return std::make_pair(center_line_target_, 0.0);
  }

  const PoseRegulateCandidate *best_candidate;
  // int offset_preference = GenerateOffsetPreference();
  // best_candidate = GetCandidatePoseByOffset(lat_buffer, offset_preference);

  // ILOG_INFO << "offset = " << offset_preference;

  // if (best_candidate != nullptr && best_candidate->dist_to_obs > lat_buffer) {
  //   return std::make_pair(best_candidate->pose, best_candidate->dist_to_obs);
  // }

  best_candidate = &candidate_info_[0];
  for (auto &obj : candidate_info_) {
    // get relative safe pose.
    if (obj.dist_to_obs > best_candidate->dist_to_obs) {
      best_candidate = &obj;
    }
  }

  return std::make_pair(best_candidate->pose, best_candidate->dist_to_obs);
}

const PoseRegulateCandidate *TargetPoseRegulator::GetCandidatePoseByOffset(
    const float lat_buffer, const int offset) const {
  if (candidate_info_.size() <= 0) {
    return nullptr;
  }

  float dist;
  float extra_buffer = 0.05;
  const PoseRegulateCandidate *best_candidate = &candidate_info_[0];

  for (auto &obj : candidate_info_) {
    // If pose is big buffer, compare offset
    dist = obj.dist_to_obs - lat_buffer - extra_buffer;
    if (dist > 0.0) {
      switch (offset) {
        case 1:
          if (obj.lat_offset < best_candidate->lat_offset) {
            best_candidate = &obj;
          }
          break;
        case -1:
          if (obj.lat_offset > best_candidate->lat_offset) {
            best_candidate = &obj;
          }
          break;
        default:
          best_candidate = &obj;
          break;
      }
    }
  }
  ILOG_INFO << "big buffer, lat offset = " << best_candidate->lat_offset
            << ",obs dist = " << best_candidate->dist_to_obs;

  return best_candidate;
}

}  // namespace planning