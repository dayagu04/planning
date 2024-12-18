#pragma once

#include "geometry_math.h"
#include "pose2d.h"
#include "trajectory/trajectory.h"

namespace planning {
// add trajectory stitcher.
// If do path/speed planning in every frame, need a stitcher.
// If navigation and parking switch in hpp, need trajectory stitcher for
// navigation trajectory and parking trajectory.
class ApaTrajectoryStitcher {
 public:
  ApaTrajectoryStitcher() = default;

  /**
   * [out]: trajectory
   */
  void Process(const Pose2D& ego_pose,
               const std::vector<pnc::geometry_lib::PathPoint>& path,
               trajectory::Trajectory* trajectory);

 private:
};
}  // namespace planning