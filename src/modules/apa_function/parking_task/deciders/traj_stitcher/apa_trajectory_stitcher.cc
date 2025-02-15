#include "apa_trajectory_stitcher.h"

namespace planning {

void ApaTrajectoryStitcher::Process(
    const Pose2D& ego_pose,
    const std::vector<pnc::geometry_lib::PathPoint>& path,
    trajectory::Trajectory* trajectory) {
  double dist_sqr_min = std::numeric_limits<double>::max();
  size_t index_min = 0;
  double dist_sqr;

  for (size_t i = 0; i < path.size(); ++i) {
    dist_sqr = ego_pose.DistanceSquareTo(path[i].pos);

    if (dist_sqr < dist_sqr_min) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }

  return;
}
}  // namespace planning