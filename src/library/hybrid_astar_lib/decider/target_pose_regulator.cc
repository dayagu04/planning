#include "target_pose_regulator.h"

#include <algorithm>
#include <array>
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

const bool TerminalGuessPath::IsPathAllPointsSafe(const float dist) {
  for (int32_t i = 0; i < size; i++) {
    if (points[i].dist_to_obs < dist) {
      return false;
    }
  }

  return true;
}

void TerminalGuessPath::Clear() {
  size = 0;
  safe_width_integral = 0.0f;
  min_dist_to_obs = 0.0f;
  return;
}

void TerminalGuessPath::AddPoint(const Pose2f &point, const float dist) {
  if (size < 0) {
    size = 0;
  }
  if (size >= TERMINAL_GUESS_PATH_MAX_POINT) {
    return;
  }

  points[size].x = point.x;
  points[size].y = point.y;
  points[size].theta = point.theta;
  points[size].dist_to_obs = dist;
  size++;

  return;
}

bool TargetPoseRegulator::IsReferenceLineSafeEnough() {
  if (!IsParkingIn(request_)) {
    if (candidate_info_.size() > 0 &&
        candidate_info_[0].dist_to_obs > max_lat_buffer_) {
      return true;
    }
  } else {
    if (paths_.size() > 0) {
      if (paths_[0].IsPathAllPointsSafe(max_lat_buffer_)) {
        return true;
      }
    }
  }

  return false;
}

void TargetPoseRegulator::GenerateXboundary(const AstarRequest *request,
                                            const VehicleParam &veh_param) {
  // x boundary
  if (request->space_type == ParkSpaceType::VERTICAL) {
    if (request->direction_request == ParkingVehDirection::TAIL_IN) {
      float veh_front_edge_to_slot = 2.3f;
      float veh_x_upper = veh_front_edge_to_slot + request->slot_length -
                          veh_param.front_edge_to_rear_axle;
      x_check_bounday_.upper = std::max(target_.x, veh_x_upper);
    } else {
      // 对于车头入库，需要检查更大的范围. 让后视镜经过柱子.
      float veh_back_edge_to_slot = 4.0f;
      float veh_x_upper = request->slot_length + veh_back_edge_to_slot -
                          veh_param.rear_edge_to_rear_axle;
      x_check_bounday_.upper = std::max(target_.x, veh_x_upper);
    }

    x_check_bounday_.lower = target_.x + low_confidence_zone_for_vertical_;
    x_check_bounday_.step = 0.2f;
  } else if (request->space_type == ParkSpaceType::SLANTING) {
    if (request->direction_request == ParkingVehDirection::TAIL_IN) {
      float veh_front_edge_to_slot = 2.3f;
      float veh_x_upper = veh_front_edge_to_slot + request->slot_length -
                          veh_param.front_edge_to_rear_axle;
      x_check_bounday_.upper = std::max(target_.x, veh_x_upper);
    } else {
      // 对于车头入库，需要检查更大的范围. 让后视镜经过柱子.
      float veh_back_edge_to_slot = 4.0f;
      float veh_x_upper = request->slot_length + veh_back_edge_to_slot -
                          veh_param.rear_edge_to_rear_axle;
      x_check_bounday_.upper = std::max(target_.x, veh_x_upper);
    }

    x_check_bounday_.lower = target_.x + low_confidence_zone_for_slant_;
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
  x_check_bounday_.number =
      std::min(TERMINAL_GUESS_PATH_MAX_POINT, x_check_bounday_.number);

  return;
}

void TargetPoseRegulator::GenerateYboundary(const AstarRequest *request,
                                            const VehicleParam &veh_param) {
  float y_upper = request->slot_width / 2 + cross_the_slot_line_max_dist_ -
                  veh_param.width / 2;
  y_upper = std::max(0.0f, y_upper);
  float y_lower = -y_upper;
  y_lower = std::min(0.0f, y_lower);

  y_check_bounday_.upper = y_upper;
  y_check_bounday_.lower = y_lower;
  y_check_bounday_.step = 0.03f;
  y_check_bounday_.number =
      std::ceil(y_upper / y_check_bounday_.step) +
      std::ceil(std::fabs(y_lower) / y_check_bounday_.step);
  y_check_bounday_.number = std::max(1, y_check_bounday_.number);

  return;
}

void TargetPoseRegulator::UpdateReferenceLinePath(
    const AstarRequest *request, const VehicleParam &veh_param,
    const ParkingVehDirection &direction_request, EulerDistanceTransform *edt) {
  // x boundary
  GenerateXboundary(request, veh_param);

  // y boundary
  GenerateYboundary(request, veh_param);

  paths_.reserve(y_check_bounday_.number);

  // update center line dist
  float dist = 0.0f;
  Transform2f tf;
  AstarPathGear gear = AstarPathGear::NONE;
  if (IsParkingOutRequest(request->direction_request)) {
    // todo ： 车头泊出目前只对终点位置进行碰撞检查，沿途路径没有做碰撞检查；
    dist = GetDistToObsHeadOut(target_, direction_request, edt);

    TerminalCandidatePoint candidate;
    candidate.lat_offset = 0.0f;
    candidate.dist_to_obs = dist;
    candidate.pose = target_;
    candidate_info_.emplace_back(candidate);
    ILOG_INFO << "target_ dist : " << dist;
  } else if (request->space_type == ParkSpaceType::VERTICAL ||
             request->space_type == ParkSpaceType::SLANTING) {
    GetCandidatePathByXRange(target_, edt);
  }

  // update ego dist
  tf.SetBasePose(request->start_pose);
  edt->DistanceCheckForPoint(&ego_dist_to_obs_, &tf, gear);

  return;
}

void TargetPoseRegulator::Process(
    EulerDistanceTransform *edt, const AstarRequest *request,
    const Pose2f &ego_pose, const Pose2f &target, const VehicleParam &veh_param,
    const ParkingVehDirection &direction_request) {
  // init
  Clear();
  target_ = target;
  request_ = request;
  cross_the_slot_line_max_dist_ = 0.0;
  max_lat_buffer_ = 0.3f;
  low_confidence_zone_for_vertical_ = 0.3f;
  low_confidence_zone_for_slant_ = 0.4f;
  edt->UpdateSafeBuffer(0.0, 0.0, 0.0);

  UpdateReferenceLinePath(request, veh_param, direction_request, edt);

  if (IsSamplingBasedPlanning(request->path_generate_method)) {
    return;
  }

  // park out
  if (request_->direction_request_size > 1) {
    GenerateCandidatesForVerticalHeadOut(edt, direction_request, veh_param);
    return;
  }

  // park out
  if (!IsParkingIn(request)) {
    GenerateCandidatesForVerticalHeadOut(edt, request, veh_param);
    return;
  }

  if (IsReferenceLineSafeEnough()) {
    return;
  }

  // park in
  // todo: parallel slot.
  GenerateCandidates(edt, request, veh_param);

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

void TargetPoseRegulator::GenerateCandidates(EulerDistanceTransform *edt,
                                             const AstarRequest *request,
                                             const VehicleParam &veh_param) {
  Pose2f global_pose;
  global_pose = target_;

  float y_offset = 0.0;
  float left_y_offset = 0.0;
  float right_y_offset = 0.0;
  float dist;

  for (int i = 0; i < y_check_bounday_.number; i++) {
    // left
    if (i % 2 == 0) {
      if (left_y_offset < y_check_bounday_.upper - 0.001f) {
        left_y_offset += y_check_bounday_.step;
        left_y_offset = std::min(left_y_offset, y_check_bounday_.upper);
        y_offset = left_y_offset;
      } else {
        continue;
      }
    } else {
      if (right_y_offset > y_check_bounday_.lower + 0.001f) {
        right_y_offset -= y_check_bounday_.step;
        right_y_offset = std::max(right_y_offset, y_check_bounday_.lower);
        y_offset = right_y_offset;
      } else {
        continue;
      }
    }

    global_pose.y = y_offset;
    dist = GetCandidatePathByXRange(global_pose, edt);
    if (dist > max_lat_buffer_) {
      // ILOG_INFO << "i " << i << ", dist " << dist << ", offset " << y_offset;
      break;
    }
  }

  return;
}

void TargetPoseRegulator::GenerateCandidatesForVerticalHeadOut(
    EulerDistanceTransform *edt, const AstarRequest *request,
    const VehicleParam &veh_param) {
  // 对于前左，前右两个方向的泊出，由于视野盲区在规划时需要对目标点进行适当偏移，目前的策略是目标点逐渐向
  // y = 0 的方向偏移，直至安全为止，偏移步长1.0；
  Pose2f global_pose;
  global_pose = target_;
  constexpr size_t kNumCandidateColumns = 5;  // 列数
  constexpr size_t kNumberRows = 5;           // 行数
  constexpr size_t kMidCandidateNum = 8;      //
  constexpr float kYLowerMid = -0.2f;         // 中间方向Y轴起始偏移
  constexpr float kYStepMiddle = 0.05f;       // 中间方向Y轴步长
  constexpr float kXStep = 0.3f;              // X轴步长
  constexpr float kSlantingOffset = 0.5f;     // 斜列停车位X轴偏移

  const bool is_middle_direction =
      request->direction_request == ParkingVehDirection::HEAD_OUT_TO_MIDDLE ||
      request->direction_request == ParkingVehDirection::TAIL_OUT_TO_MIDDLE;
  const size_t total_candidates = is_middle_direction
                                      ? kNumCandidateColumns
                                      : (kNumberRows * kNumCandidateColumns);
  candidate_info_.reserve(candidate_info_.size() + total_candidates);

  Eigen::Vector2d temp_pos(0, 0);

  TerminalCandidatePoint candidate;
  // 处理中间方向
  if (is_middle_direction) {
    Eigen::Vector2d mid_base_pose(target_.GetX(), kYLowerMid);
    for (size_t j = 0; j < kMidCandidateNum; ++j) {
      const Eigen::Vector2d temp_pos =
          mid_base_pose +
          request_->x_axis_direction_coordinate_slant * j * kYStepMiddle;

      for (size_t i = 0; i < kNumCandidateColumns; ++i) {
        Pose2f candidate_pose(temp_pos.x() - i * kSlantingOffset, temp_pos.y(),
                              target_.theta);
        candidate.dist_to_obs = GetDistToObsHeadOut(
            candidate_pose, request->direction_request, edt);
        candidate.pose.SetPose(candidate_pose.x, candidate_pose.y,
                               candidate_pose.theta);
        candidate_info_.emplace_back(candidate);
      }
    }
    return;
  }

  // 处理左右方向
  const int8_t direction_factor =
      (request->direction_request == ParkingVehDirection::HEAD_OUT_TO_LEFT ||
       request->direction_request == ParkingVehDirection::TAIL_OUT_TO_RIGHT)
          ? -1
          : 1;

  const Eigen::Vector2d base_pose(target_.GetX(), target_.GetY());

  for (size_t j = 0; j < kNumCandidateColumns; ++j) {
    const Eigen::Vector2d temp_pos =
        base_pose +
        request->x_axis_direction_coordinate_slant * j * direction_factor;

    for (size_t i = 0; i < kNumberRows; ++i) {
      global_pose.x = temp_pos.x() + kXStep * i;
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
  if (target_.GetX() < 0.0 || direction_request == ParkingVehDirection::NONE) {
    // todo :: 现在的虚拟墙不支持 目标点小于0 的搜索，先跳过；
    return;
  }

  Pose2f global_pose;
  global_pose = target_;
  constexpr size_t kNumCandidateColumns = 5;  // 列数
  constexpr size_t kNumberRows = 5;           // 行数
  constexpr size_t kMidCandidateNum = 8;      //
  constexpr float kYLowerMid = -0.2f;         // 中间方向Y轴起始偏移
  constexpr float kYStepMiddle = 0.05f;       // 中间方向Y轴步长
  constexpr float kXStep = 0.3f;              // X轴步长
  constexpr float kSlantingOffset = 0.5f;     // 斜列停车位X轴偏移

  const bool is_middle_direction =
      direction_request == ParkingVehDirection::HEAD_OUT_TO_MIDDLE ||
      direction_request == ParkingVehDirection::TAIL_OUT_TO_MIDDLE;
  const size_t total_candidates = is_middle_direction
                                      ? kNumCandidateColumns
                                      : (kNumberRows * kNumCandidateColumns);
  candidate_info_.reserve(candidate_info_.size() + total_candidates);

  Eigen::Vector2d temp_pos(0, 0);

  TerminalCandidatePoint candidate;
  // 处理中间方向
  if (is_middle_direction) {
    Eigen::Vector2d mid_base_pose(target_.GetX(), kYLowerMid);
    for (size_t j = 0; j < kMidCandidateNum; ++j) {
      const Eigen::Vector2d temp_pos =
          mid_base_pose +
          request_->x_axis_direction_coordinate_slant * j * kYStepMiddle;

      for (size_t i = 0; i < kNumCandidateColumns; ++i) {
        Pose2f candidate_pose(temp_pos.x() - i * kSlantingOffset, temp_pos.y(),
                              target_.theta);
        candidate.dist_to_obs =
            GetDistToObsHeadOut(candidate_pose, direction_request, edt);
        candidate.pose.SetPose(candidate_pose.x, candidate_pose.y,
                               candidate_pose.theta);
        candidate_info_.emplace_back(candidate);
      }
    }
    return;
  }

  // 处理左右方向
  const int8_t direction_factor =
      (direction_request == ParkingVehDirection::HEAD_OUT_TO_LEFT ||
       direction_request == ParkingVehDirection::TAIL_OUT_TO_RIGHT)
          ? -1
          : 1;

  const Eigen::Vector2d base_pose(target_.GetX(), target_.GetY());
  std::array<float, kNumberRows> dist_to_obs{};

  for (size_t j = 0; j < kNumCandidateColumns; ++j) {
    const Eigen::Vector2d temp_pos =
        base_pose +
        request_->x_axis_direction_coordinate_slant * j * direction_factor;

    for (size_t i = 0; i < kNumberRows; ++i) {
      global_pose.x = temp_pos.x() + kXStep * i;
      global_pose.y = temp_pos.y();
      candidate.dist_to_obs =
          GetDistToObsHeadOut(global_pose, direction_request, edt);
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

void TargetPoseRegulator::Clear() {
  candidate_info_.clear();
  paths_.clear();
  return;
}

const float TargetPoseRegulator::GetCandidatePathByXRange(
    const Pose2f &global_pose, EulerDistanceTransform *edt) {
  Transform2f tf;
  AstarPathGear gear = AstarPathGear::NONE;
  float dist;
  float max_dist = 0.0f;
  float min_dist = 10.0f;
  Pose2f pose = global_pose;
  pose.x = x_check_bounday_.lower;
  TerminalGuessPath path;
  path.Clear();

  for (int j = 0; j < x_check_bounday_.number; j++) {
    tf.SetBasePose(pose);

    edt->DistanceCheckForPoint(&dist, &tf, gear);
    path.AddPoint(pose, dist);

    max_dist = std::max(max_dist, dist);
    min_dist = std::min(min_dist, dist);

    pose.x += x_check_bounday_.step;
  }

  if (max_dist > 0.08f) {
    paths_.emplace_back(path);
  }

  return min_dist;
}

const float TargetPoseRegulator::GetDistToObsHeadOut(
    const Pose2f &global_pose, const ParkingVehDirection &direction_request,
    EulerDistanceTransform *edt) {
  Transform2f tf;
  AstarPathGear gear = AstarPathGear::NONE;
  float dist;
  float min_dist = 10.0f;
  Pose2f current_pose = global_pose;
  constexpr float kYStep = 0.5f;
  // constexpr float kMinSafetyDist = 0.5f;
  const float min_safety_dist = 0.04f;
  size_t sampling_size =
      static_cast<size_t>(ceil(fabs(global_pose.GetY()) / kYStep));

  const Eigen::Vector2d base_pos(global_pose.GetX(), global_pose.GetY());
  Eigen::Vector2d temp_pos;

  int8_t direction_factor = 0;
  bool is_middle_direction = false;

  switch (direction_request) {
    case ParkingVehDirection::TAIL_OUT_TO_LEFT:
      direction_factor = 1;
      gear = AstarPathGear::REVERSE;
      break;
    case ParkingVehDirection::TAIL_OUT_TO_RIGHT:
      direction_factor = -1;
      gear = AstarPathGear::REVERSE;
      break;
    case ParkingVehDirection::TAIL_OUT_TO_MIDDLE:
      gear = AstarPathGear::REVERSE;
      is_middle_direction = true;
      break;

    case ParkingVehDirection::HEAD_OUT_TO_LEFT:
      direction_factor = -1;
      gear = AstarPathGear::DRIVE;
      break;
    case ParkingVehDirection::HEAD_OUT_TO_RIGHT:
      direction_factor = 1;
      gear = AstarPathGear::DRIVE;
      break;
    case ParkingVehDirection::HEAD_OUT_TO_MIDDLE:
      gear = AstarPathGear::DRIVE;
      is_middle_direction = true;
      break;
    default:
      break;
  }

  if (is_middle_direction) {
    tf.SetBasePose(current_pose);

    edt->DistanceCheckForPoint(&dist, &tf, gear);

    min_dist = std::min(min_dist, dist);
  } else {
    for (int j = 0; j < sampling_size; j++) {
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

const TerminalCandidatePoint TargetPoseRegulator::GetCandidatePoseForParkOut(
    const float lat_buffer, const float min_lat_buffer) {
  if (candidate_info_.size() <= 0) {
    return TerminalCandidatePoint(target_, 0.0f, 0.0f);
  }

  float dist;
  const TerminalCandidatePoint *best_candidate = &candidate_info_[0];

  for (auto &obj : candidate_info_) {
    // If pose is big buffer, return
    dist = obj.dist_to_obs - lat_buffer;
    if (dist > 0.0f) {
      ILOG_INFO << "offset: " << obj.lat_offset
                << ",obs dist: " << obj.dist_to_obs;
      best_candidate = &obj;
      break;
    }

    // get relative safe pose.
    if (obj.dist_to_obs > best_candidate->dist_to_obs) {
      best_candidate = &obj;
    }
  }

  // for easy control, and easy planning, use a straight line to real end.
  TerminalCandidatePoint res(*best_candidate);

  return res;
}

void TargetPoseRegulator::DebugString() {
  ILOG_INFO << "candidate point size = " << candidate_info_.size();

  for (size_t i = 0; i < candidate_info_.size(); i++) {
    ILOG_INFO << "i = " << i
              << ",lat offset = " << candidate_info_[i].lat_offset
              << ",obs dist = " << candidate_info_[i].dist_to_obs;

    candidate_info_[i].pose.DebugString();
  }

  ILOG_INFO << "x bound, upper " << x_check_bounday_.upper << ", lower "
            << x_check_bounday_.lower << ", num " << x_check_bounday_.number;
  ILOG_INFO << "y bound, upper " << y_check_bounday_.upper << ", lower "
            << y_check_bounday_.lower << ", num " << y_check_bounday_.number;

  ILOG_INFO << "path size = " << paths_.size();
  // for (size_t i = 0; i < paths_.size(); i++) {
  //   DebugPath(paths_[i]);
  // }

  return;
}

const void TargetPoseRegulator::GetSafeIntegralForPath(
    TerminalGuessPath &path, const float buffer,
    const float min_lateral_buffer) {
  path.safe_width_integral = 0.0f;
  path.min_dist_to_obs = 0.0f;
  if (path.size <= 0) {
    return;
  }

  float width = 0.0f;
  float integral = 0.0f;
  float min_width = 10.0f;
  bool find_valid_point = false;

  // if path point is not safe, break.
  for (int32_t i = path.size - 1; i >= 0; i--) {
    width = path.points[i].dist_to_obs;
    if (width < min_lateral_buffer) {
      break;
    }

    min_width = std::min(min_width, width);
    // Normalize to [0, buffer]
    width = std::min(buffer, width);
    integral += width * x_check_bounday_.step;
    find_valid_point = true;
  }

  if (find_valid_point) {
    path.safe_width_integral = integral;
    path.min_dist_to_obs = min_width;
  }

  return;
}

const TerminalCandidatePoint TargetPoseRegulator::GetCandidatePoseForParkIn(
    const float lat_buffer, const float min_lateral_buffer) {
  if (paths_.size() <= 0) {
    return TerminalCandidatePoint(request_->goal, 0.0f, 0.0f);
  }

  // use integral strategy to compare path, and get the lateral position.
  float max_integral =
      lat_buffer * x_check_bounday_.step * x_check_bounday_.number;

  TerminalGuessPath *best_path = &paths_[0];
  for (size_t i = 0; i < paths_.size(); i++) {
    GetSafeIntegralForPath(paths_[i], lat_buffer, min_lateral_buffer);

    if (paths_[i].safe_width_integral > max_integral - 0.001f) {
      best_path = &paths_[i];
      break;
    }

    // get relative safe path, and path length is long.
    if (paths_[i].safe_width_integral > best_path->safe_width_integral) {
      best_path = &paths_[i];
    }
  }

  // for easy control, and easy planning, use a straight line to real end.
  // you have got lateral postion, then you need get lon position.
  TerminalCandidatePoint res;
  res.pose = best_path->points[0];
  res.lat_offset = res.pose.y;
  res.dist_to_obs = best_path->min_dist_to_obs;

  const float min_straight_line_len = 0.5f;
  res.pose.x =
      GetLonPosition(best_path, request_->real_goal.x + min_straight_line_len,
                     request_->goal.x, lat_buffer);

#if DEBUG_DECIDER
  DebugString();
  DebugPath(*best_path);

  ILOG_INFO << "lateral " << res.lat_offset << ", dist " << res.dist_to_obs
            << ", lon x " << res.pose.x << ", init guess lon x "
            << request_->goal.x;
#endif
  return res;
}

const TerminalCandidatePoint TargetPoseRegulator::GetCandidatePose(
    const float lat_buffer, const float min_lateral_buffer) {
  if (IsParkingIn(request_)) {
    return GetCandidatePoseForParkIn(lat_buffer, min_lateral_buffer);
  }

  return GetCandidatePoseForParkOut(lat_buffer, min_lateral_buffer);
}

void TargetPoseRegulator::DebugPath(const TerminalGuessPath &path) const {
  ILOG_INFO << "point size " << path.size << ", min dist "
            << path.min_dist_to_obs << ", score " << path.safe_width_integral
            << ",lat offset = " << path.points[0].y;

  for (size_t i = 0; i < path.size; i++) {
    path.points[i].DebugString();
    ILOG_INFO << "dist = " << path.points[i].dist_to_obs;
  }

  return;
}

float TargetPoseRegulator::GetLonPosition(const TerminalGuessPath *path,
                                          const float lon_lower,
                                          const float lon_upper,
                                          const float lat_buffer) {
  if (path->size <= 0) {
    return lon_upper;
  }

  // best lon position: find position from upper to lower
  // 1. find the position, it's distance  [1.0, +inf], continue to find lower
  // position;
  // 2. [0, 0.35], record max distance position;
  // 3. [0.35, 1.0], break;

  // search best lon position by obstacle distance.
  float best_lon = lon_upper;
  float narrow_space_max_dist = 0.0f;
  bool find_narrow_space = false;
  bool find_broad_space = false;

  for (int32_t i = path->size - 1; i >= 0; i--) {
    if (path->points[i].x > lon_upper + 1e-3) {
      continue;
    }
    if (path->points[i].x < lon_lower - 1e-3) {
      continue;
    }

    if (path->points[i].dist_to_obs > lat_buffer + 0.85f) {
      if (!find_narrow_space) {
        best_lon = path->points[i].x;
      }
      find_broad_space = true;
    } else if (path->points[i].dist_to_obs > lat_buffer + 0.2f) {
      best_lon = path->points[i].x;
      break;
    } else {
      if (!find_broad_space) {
        if (path->points[i].dist_to_obs > narrow_space_max_dist) {
          narrow_space_max_dist = path->points[i].dist_to_obs;
          best_lon = path->points[i].x;
        }
      }
      find_narrow_space = true;
    }
  }

  return best_lon;
}

}  // namespace planning