#pragma once

#include "ogm_common.h"
#include "pose2d.h"
#include "transform2d.h"

namespace planning {

struct OccupancyGridBound {
  double min_x;
  double min_y;
  double max_x;
  double max_y;
};

class OccupancyGridCoordinate {
 public:
  OccupancyGridCoordinate() = default;

  virtual void Process(const Pose2D &ogm_pose);

  const bool IsIndexValid(const OgmIndex &id);

  const double GetBoundMinX() const { return bound_.min_x; }

  const double GetBoundMinY() const { return bound_.min_y; }

  const double GetOgmResolutionInv() const { return ogm_resolution_inv_; }

  // ogm local pose to index
  void OgmPoseToIndex(OgmIndex *index, const Pose2D &point);

  // slot local pose to index
  const bool SlotPoseToIndex(OgmIndex *index, const Pose2D &point);

 protected:
  //               ^  x
  //               |
  //               |
  //               |
  //               |
  //               |
  //  y <----------|

  Transform2d ogm_tf_;

  // base pose in slot coordinate system
  Pose2D ogm_base_pose_;

  // in slot system.
  OccupancyGridBound bound_;

  double ogm_resolution_inv_;
};
}  // namespace planning