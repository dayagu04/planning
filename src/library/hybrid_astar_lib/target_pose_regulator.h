#pragma once

#include <cstddef>
#include <string>

#include "astar_decider.h"
#include "euler_distance_transform.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_request.h"
#include "node3d.h"
#include "park_reference_line.h"
#include "pose2d.h"
#include "rs_path_interpolate.h"

namespace planning {

// 目标pose调节器.
// Todo: all target pose decisions should be moved to here.
// 1. Move goal to outside a few meters in vertical/parallel slot scene.
// 2. If origin goal is collided, move goal to left/right/top/bottom.
class TargetPoseRegulator : public AstarDecider {
 public:
  TargetPoseRegulator() = default;

  void Process(EulerDistanceTransform *edt, const AstarRequest *request,
               const Pose2D &ego_pose, const Pose2D &center_line_target);

  void Process(const Pose2D &start, const Pose2D &end);

  void Clear();

  const Pose2D &GetTargetPose() const { return advised_safe_target_pose_; }

 private:
  const bool IsParkingIn(const AstarRequest *request);

  void UpdatePoseBySafeChecker(EulerDistanceTransform *edt,
                               const AstarRequest *request);

 private:
  bool is_default_pose_safe_;
  bool is_adjust_pose_;
  Pose2D advised_safe_target_pose_;
};

}  // namespace planning