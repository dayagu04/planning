#include "target_pose_regulator.h"

#include <cmath>
#include <cstddef>
#include <vector>

#include "hybrid_astar_common.h"
#include "log_glog.h"

namespace planning {

#define DEBUG_DECIDER (0)

void TargetPoseRegulator::Process(EulerDistanceTransform *edt,
                                  const AstarRequest *request,
                                  const Pose2D &ego_pose,
                                  const Pose2D &center_line_target) {
  Clear();
  advised_safe_target_pose_ = center_line_target;

  // todo: add parallel regulator
  if (request->space_type == ParkSpaceType::PARALLEL) {
    return;
  }

  if (request->path_generate_method ==
          AstarPathGenerateType::CUBIC_POLYNOMIAL_SAMPLING ||
      request->path_generate_method == AstarPathGenerateType::REEDS_SHEPP) {
    return;
  }

  if (!IsParkingIn(request)) {
    return;
  }

  UpdatePoseBySafeChecker(edt, request);

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

void TargetPoseRegulator::UpdatePoseBySafeChecker(EulerDistanceTransform *edt,
                                                  const AstarRequest *request) {
  Pose2D global_pose;
  Transform2d tf;
  AstarPathGear gear = AstarPathGear::NONE;

  global_pose = advised_safe_target_pose_;
  tf.SetBasePose(global_pose);

  if (edt->IsCollisionForPoint(&tf, gear)) {
    is_default_pose_safe_ = false;
  }

  if (is_default_pose_safe_) {
    return;
  }

  std::vector<double> y_offset = {0.02, -0.02, 0.05, -0.05};
  for (size_t i = 0; i < y_offset.size(); i++) {
    global_pose.y = advised_safe_target_pose_.y + y_offset[i];
    tf.SetBasePose(global_pose);

    if (!edt->IsCollisionForPoint(&tf, gear)) {
      is_adjust_pose_ = true;
      advised_safe_target_pose_ = global_pose;

      ILOG_INFO << "regulator change pose = " << global_pose.y;
      return;
    }
  }

  // Todo: adjust x offset
  return;
}

void TargetPoseRegulator::Clear() {
  is_default_pose_safe_ = true;
  is_adjust_pose_ = false;
  return;
}

}  // namespace planning