#include "target_pose_regulator.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "hybrid_astar_common.h"
#include "hybrid_astar_request.h"
#include "log_glog.h"
#include "math_utils.h"
#include "pose2d.h"

namespace planning {

#define DEBUG_DECIDER (0)

bool TargetPoseRegulator::IsDefaultPoseSafeEnough() {
  if (candidate_info_.size() > 0 &&
      candidate_info_[0].dist_to_obs > max_lat_buffer_) {
    return true;
  }

  return false;
}

void TargetPoseRegulator::UpdateDefaultPoseInfo(
    const AstarRequest *request, const VehicleParam &veh_param,
    const ParkingVehDirection &direction_request, EulerDistanceTransform *edt) {
  // For vertical slot, need to check safe in path, not only a terminal point.
  if (request->space_type == ParkSpaceType::VERTICAL) {
    if (request->direction_request == ParkingVehDirection::TAIL_IN) {
      float veh_front_edge_to_slot = 2.3f;
      float veh_x_upper = veh_front_edge_to_slot + request->slot_length -
                          veh_param.front_edge_to_rear_axle;
      x_check_bounday_.upper = std::max(center_line_target_.x, veh_x_upper);
    } else {
      // 对于车头入库，需要检查更大的范围. 让后视镜经过柱子.
      float veh_back_edge_to_slot = 4.0f;
      float veh_x_upper = request->slot_length + veh_back_edge_to_slot -
                          veh_param.rear_edge_to_rear_axle;
      x_check_bounday_.upper = std::max(center_line_target_.x, veh_x_upper);
    }

    x_check_bounday_.lower = center_line_target_.x;
    x_check_bounday_.step = 0.2f;
  } else {
    x_check_bounday_.upper =
        request->slot_length - veh_param.front_edge_to_rear_axle;
    x_check_bounday_.lower = veh_param.rear_edge_to_rear_axle;
    x_check_bounday_.step = 0.1f;
  }
  x_check_bounday_.number =
      std::ceil((x_check_bounday_.upper - x_check_bounday_.lower) /
                x_check_bounday_.step);
  x_check_bounday_.number = std::max(1, x_check_bounday_.number);

  float dist = 10.0f;
  Transform2f tf;
  AstarPathGear gear = AstarPathGear::NONE;
  if (IsParkingOutRequest(request->direction_request)) {
    // todo ： 车头泊出目前只对终点位置进行碰撞检查，沿途路径没有做碰撞检查；
    dist = GetDistToObsHeadOut(center_line_target_, direction_request, edt);
    ILOG_INFO << "center_line_target_ dist : " << dist;
  } else if (request->space_type == ParkSpaceType::VERTICAL) {
    dist = GetMinDistByXRange(center_line_target_, edt);
  } else {
    tf.SetBasePose(center_line_target_);
    edt->DistanceCheckForPoint(&dist, &tf, gear);
  }

  PoseRegulateCandidate candidate;
  candidate.lat_offset = 0.0f;
  candidate.dist_to_obs = dist;
  candidate.pose = center_line_target_;
  candidate_info_.emplace_back(candidate);

  // update ego dist
  tf.SetBasePose(request->start_pose);
  edt->DistanceCheckForPoint(&dist, &tf, gear);
  ego_dist_to_obs_ = dist;

#if DEBUG_DECIDER
  DebugString();
#endif

  return;
}

void TargetPoseRegulator::Process(
    EulerDistanceTransform *edt, const AstarRequest *request,
    const Pose2f &ego_pose, const Pose2f &center_line_target,
    const VehicleParam &veh_param,
    const ParkingVehDirection &direction_request) {
  Clear();
  center_line_target_ = center_line_target;
  request_ = request;
  cross_the_slot_line_max_dist_ = 0.0;
  max_lat_buffer_ = 0.3;
  edt->UpdateSafeBuffer(0.0, 0.2, 0.0);
  UpdateDefaultPoseInfo(request, veh_param, direction_request, edt);

  if (IsSamplingBasedPlanning(request->path_generate_method)) {
    return;
  }

  if (request_->direction_request_size > 1) {
    // todo:
    // 仅垂直泊车预规划使用，尾泊出还需测试，后续与GenerateCandidatesForVerticalHeadOut(edt,
    // request, veh_param)
    // 合并为一个函数.
    GenerateCandidatesForVerticalHeadOut(edt, direction_request, veh_param);
    return;
  }

  if (!IsParkingIn(request)) {
    GenerateCandidatesForVerticalHeadOut(edt, request, veh_param);
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

void TargetPoseRegulator::Process(const Pose2f &start, const Pose2f &end) {
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
  Pose2f global_pose;
  global_pose = center_line_target_;

  float y_upper = request->slot_width / 2 + cross_the_slot_line_max_dist_ -
                  veh_param.width / 2;
  y_upper = std::max(0.0f, y_upper);
  float y_lower = -request->slot_width / 2 - cross_the_slot_line_max_dist_ +
                  veh_param.width / 2;
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

    global_pose.y = y_offset;
    dist = GetMinDistByXRange(global_pose, edt);
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

    if (dist > max_lat_buffer_) {
      break;
    }
  }

#if DEBUG_DECIDER
  DebugString();
#endif

  // Todo: adjust x offset
  return;
}

void TargetPoseRegulator::GenerateCandidatesForVerticalHeadOut(
    EulerDistanceTransform *edt, const AstarRequest *request,
    const VehicleParam &veh_param) {
  // 对于前左，前右两个方向的泊出，由于视野盲区在规划时需要对目标点进行适当偏移，目前的策略是目标点逐渐向
  // y = 0 的方向偏移，直至安全为止，偏移步长1.0；
  Pose2f global_pose;
  global_pose = center_line_target_;
  constexpr size_t kMaxCandidateNum = 10;  // 最大候选数量
  // constexpr size_t kNumberRows = 3;        // 行数
  constexpr float kYLowerMid = -0.1f;      // 中间方向Y轴起始偏移
  constexpr float kYStepMiddle = 0.02f;    // 中间方向Y轴步长
  constexpr float kXStep = 0.3f;           // X轴步长
  constexpr float kSlantingOffset = 0.5f;  // 斜列停车位X轴偏移
  constexpr float kNormalBaseX = 7.5f;     // 正常停车位基准X坐标

  const size_t number_rows =
      request_->space_type == ParkSpaceType::SLANTING ? 5 : 3;

  const float base_x = (request->space_type == ParkSpaceType::SLANTING)
                           ? (global_pose.GetX() - kSlantingOffset)
                           : kNormalBaseX;

  const bool is_middle_direction =
      request->direction_request == ParkingVehDirection::HEAD_OUT_TO_MIDDLE ||
      request->direction_request == ParkingVehDirection::TAIL_OUT_TO_MIDDLE;
  const size_t total_candidates =
      is_middle_direction ? kMaxCandidateNum : (number_rows * kMaxCandidateNum);
  candidate_info_.reserve(candidate_info_.size() + total_candidates);

  Eigen::Vector2d temp_pos(0, 0);

  PoseRegulateCandidate candidate;
  // 处理中间方向（只需要1列）
  if (is_middle_direction) {
    for (size_t i = 0; i < kMaxCandidateNum; ++i) {
      global_pose.y = kYLowerMid + i * kYStepMiddle;

      candidate.dist_to_obs =
          GetDistToObsHeadOut(global_pose, request->direction_request, edt);
      candidate.pose.SetPose(global_pose.x, global_pose.y, global_pose.theta);
      candidate_info_.emplace_back(candidate);
    }
    return;
  }

  // 处理左右方向（需要多行）
  const int8_t direction_factor =
      (request->direction_request == ParkingVehDirection::HEAD_OUT_TO_LEFT ||
       request->direction_request == ParkingVehDirection::TAIL_OUT_TO_RIGHT)
          ? -1
          : 1;

  for (size_t j = 0; j < number_rows; ++j) {
    const Eigen::Vector2d base_pose(base_x + kXStep * j,
                                    center_line_target_.GetY());

    for (size_t i = 0; i < kMaxCandidateNum; ++i) {
      const Eigen::Vector2d temp_pos =
          base_pose +
          request->x_axis_direction_coordinate_slant * i * direction_factor;

      global_pose.x = temp_pos.x();
      global_pose.y = temp_pos.y();

      candidate.dist_to_obs =
          GetDistToObsHeadOut(global_pose, request->direction_request, edt);
      candidate.pose.SetPose(global_pose.x, global_pose.y, global_pose.theta);
      candidate_info_.emplace_back(candidate);
    }
  }

#if DEBUG_DECIDER
  DebugString();
#endif

  // Todo: adjust x offset
  return;
}

void TargetPoseRegulator::GenerateCandidatesForVerticalHeadOut(
    EulerDistanceTransform *edt, const ParkingVehDirection &direction_request,
    const VehicleParam &veh_param) {
  // 对于前左，前右两个方向的泊出，由于视野盲区在规划时需要对目标点进行适当偏移，目前的策略是目标点逐渐向
  // y = 0 的方向偏移，直至安全为止，偏移步长1.0；
  if (center_line_target_.GetX() < 0.0) {
    // todo :: 现在的虚拟墙不支持 目标点小于0 的搜索，先跳过；
    return;
  }
  Pose2f global_pose;
  global_pose = center_line_target_;
  constexpr size_t kMaxCandidateNum = 10;  // 最大候选数量
  // constexpr size_t kNumberRows = 3;        // 行数
  constexpr float kYLowerMid = -0.1f;      // 中间方向Y轴起始偏移
  constexpr float kYStepMiddle = 0.02f;    // 中间方向Y轴步长
  constexpr float kXStep = 0.3f;           // X轴步长
  constexpr float kSlantingOffset = 0.5f;  // 斜列停车位X轴偏移
  constexpr float kNormalBaseX = 7.5f;     // 正常停车位基准X坐标

  const size_t number_rows =
      request_->space_type == ParkSpaceType::SLANTING ? 5 : 3;

  const float base_x = (request_->space_type == ParkSpaceType::SLANTING)
                           ? (global_pose.GetX() - kSlantingOffset)
                           : kNormalBaseX;

  PoseRegulateCandidate candidate;
  const size_t totalCandidates =
      (direction_request == ParkingVehDirection::HEAD_OUT_TO_MIDDLE)
          ? kMaxCandidateNum
          : (number_rows * kMaxCandidateNum);
  candidate_info_.reserve(candidate_info_.size() + totalCandidates);

  // 中间方向特殊处理，只需要1列
  if (direction_request == ParkingVehDirection::HEAD_OUT_TO_MIDDLE) {
    for (size_t i = 0; i < kMaxCandidateNum; ++i) {
      global_pose.y = kYLowerMid + i * kYStepMiddle;

      candidate.dist_to_obs =
          GetDistToObsHeadOut(global_pose, direction_request, edt);
      candidate.pose.SetPose(global_pose.x, global_pose.y, global_pose.theta);
      candidate_info_.emplace_back(candidate);
    }
  } else {
    // 左右方向处理
    for (size_t j = 0; j < number_rows; ++j) {
      const Eigen::Vector2d base_pose(base_x + kXStep * j,
                                      center_line_target_.GetY());

      for (size_t i = 0; i < kMaxCandidateNum; ++i) {
        Eigen::Vector2d temp_pos;
        switch (direction_request) {
          case ParkingVehDirection::HEAD_OUT_TO_LEFT:
            temp_pos =
                base_pose - request_->x_axis_direction_coordinate_slant * i;
            break;
          case ParkingVehDirection::HEAD_OUT_TO_RIGHT:
            temp_pos =
                base_pose + request_->x_axis_direction_coordinate_slant * i;
            break;
          default:
            // todo: 后续调整车尾泊出的candidate，当前仅调试了车头泊出的.
            continue;
        }

        global_pose.x = temp_pos.x();
        global_pose.y = temp_pos.y();

        candidate.dist_to_obs =
            GetDistToObsHeadOut(global_pose, direction_request, edt);
        candidate.pose.SetPose(global_pose.x, global_pose.y, global_pose.theta);
        candidate_info_.emplace_back(candidate);
      }
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

const float TargetPoseRegulator::GetMinDistByXRange(
    const Pose2f &global_pose, EulerDistanceTransform *edt) {
  Transform2f tf;
  AstarPathGear gear = AstarPathGear::NONE;
  if (request_->space_type == ParkSpaceType::VERTICAL) {
    if (request_->direction_request == ParkingVehDirection::TAIL_IN) {
      gear = AstarPathGear::REVERSE;
    } else {
      gear = AstarPathGear::DRIVE;
    }
  }

  float dist;
  float min_dist = 10.0;
  Pose2f pose = global_pose;
  pose.x = x_check_bounday_.lower;

  for (int j = 0; j < x_check_bounday_.number; j++) {
    tf.SetBasePose(pose);

    edt->DistanceCheckForPoint(&dist, &tf, gear);
    min_dist = std::min(min_dist, dist);

    if (min_dist < 0.04) {
      break;
    }
    pose.x += x_check_bounday_.step;
    pose.x = std::min(pose.x, x_check_bounday_.upper);
  }

  return min_dist;
}

const float TargetPoseRegulator::GetDistToObsHeadOut(
    const Pose2f &global_pose, const ParkingVehDirection &direction_request,
    EulerDistanceTransform *edt) {
  Transform2f tf;
  AstarPathGear gear = AstarPathGear::DRIVE;
  float dist;
  float min_dist = 10.0f;
  Pose2f current_pose = global_pose;
  constexpr size_t kNumSteps = 10;
  constexpr float kYStep = 0.5f;
  // constexpr float kMinSafetyDist = 0.5f;
  const float min_safety_dist =
      (direction_request == ParkingVehDirection::HEAD_OUT_TO_MIDDLE ||
       direction_request == ParkingVehDirection::TAIL_OUT_TO_MIDDLE)
          ? 0.04f
          : 0.5f;
  const Eigen::Vector2d base_pos(global_pose.GetX(), global_pose.GetY());
  Eigen::Vector2d temp_pos;

  const int8_t direction_factor =
      (direction_request == ParkingVehDirection::HEAD_OUT_TO_LEFT ||
       direction_request == ParkingVehDirection::TAIL_OUT_TO_RIGHT)
          ? -1
          : 1;
  if (direction_request == ParkingVehDirection::HEAD_OUT_TO_MIDDLE ||
      direction_request == ParkingVehDirection::TAIL_OUT_TO_MIDDLE) {
    tf.SetBasePose(current_pose);

    edt->DistanceCheckForPoint(&dist, &tf, gear);

    min_dist = std::min(min_dist, dist);
  } else {
    for (int j = 0; j < kNumSteps; j++) {
      temp_pos = base_pos + request_->x_axis_direction_coordinate_slant *
                                kYStep * j * direction_factor;
      current_pose.x = temp_pos.x();
      current_pose.y = temp_pos.y();

      tf.SetBasePose(current_pose);

      edt->DistanceCheckForPoint(&dist, &tf, gear);

      min_dist = std::min(min_dist, dist);

      if (min_dist < min_safety_dist) {
        break;
      }
    }
  }

  // tf.SetBasePose(pose);

  // edt->DistanceCheckForPoint(&dist, &tf, gear);

  // min_dist = std::min(min_dist, dist);

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

const std::pair<Pose2f, float> TargetPoseRegulator::GetCandidatePose(
    const float lat_buffer) const {
  if (candidate_info_.size() <= 0) {
    return std::make_pair(center_line_target_, 0.0);
  }

  float dist;
  float extra_buffer =
      (request_->direction_request == ParkingVehDirection::HEAD_OUT_TO_LEFT ||
       request_->direction_request == ParkingVehDirection::HEAD_OUT_TO_RIGHT ||
       request_->direction_request == ParkingVehDirection::TAIL_OUT_TO_LEFT ||
       request_->direction_request == ParkingVehDirection::TAIL_OUT_TO_RIGHT)
          ? 0.5f
          : 0.0f;
  const PoseRegulateCandidate *best_candidate = &candidate_info_[0];

  for (auto &obj : candidate_info_) {
    // If pose is big buffer, return
    dist = obj.dist_to_obs - lat_buffer;
    if (dist > 0.0f) {
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

void TargetPoseRegulator::DebugString() {
  ILOG_INFO << "candidate size = " << candidate_info_.size();

  for (size_t i = 0; i < candidate_info_.size(); i++) {
    ILOG_INFO << "i = " << i
              << ",lat offset = " << candidate_info_[i].lat_offset
              << ",obs dist = " << candidate_info_[i].dist_to_obs;

    candidate_info_[i].pose.DebugString();
  }

  ILOG_INFO << "x bound, upper " << x_check_bounday_.upper << ", lower "
            << x_check_bounday_.lower << ", num " << x_check_bounday_.number;

  return;
}

void TargetPoseRegulator::GenerateCandidatesForParallelSlot(
    EulerDistanceTransform *edt, const AstarRequest *request,
    const VehicleParam &veh_param) {
  Pose2f global_pose;
  AstarPathGear gear = AstarPathGear::NONE;
  global_pose = center_line_target_;

  TerminalCheckBoundary y_bounday;
  y_bounday.upper = request->slot_width / 2 - veh_param.width / 2 +
                    cross_the_slot_line_max_dist_;
  y_bounday.upper = std::max(0.0f, y_bounday.upper);
  y_bounday.lower = -request->slot_width / 2 + veh_param.width / 2 -
                    cross_the_slot_line_max_dist_;
  y_bounday.lower = std::min(0.0f, y_bounday.lower);
  y_bounday.step = 0.03;
  y_bounday.number =
      std::ceil((y_bounday.upper - y_bounday.lower) / y_bounday.step) * 2;

  float y_offset = 0.0;
  float left_y_offset = 0.0;
  float right_y_offset = 0.0;

  float dist;
  PoseRegulateCandidate candidate;

  for (int i = 0; i < y_bounday.number; i++) {
    // left
    if (i % 2 == 0) {
      left_y_offset += y_bounday.step;
      left_y_offset = std::min(left_y_offset, y_bounday.upper);
      y_offset = left_y_offset;
    } else {
      right_y_offset -= y_bounday.step;
      right_y_offset = std::max(right_y_offset, y_bounday.lower);
      y_offset = right_y_offset;
    }

    global_pose.y = y_offset;
    dist = GetMinDistByXRange(global_pose, edt);
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

}  // namespace planning