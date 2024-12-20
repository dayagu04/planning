#pragma once

#include "log_glog.h"
#include "ogm_common.h"
#include "pose2d.h"
#include "transform2d.h"

namespace planning {

struct OccupancyGridBound {
  double min_x;
  double min_y;
  double max_x;
  double max_y;

  OccupancyGridBound() = default;

  OccupancyGridBound(const double x_min, const double y_min, const double x_max,
                     const double y_max)
      : min_x(x_min), min_y(y_min), max_x(x_max), max_y(y_max){};

  void PrintInfo(const bool enable_log = true) const {
    ILOG_INFO_IF(enable_log)
        << "OccupancyGridBound min_x = " << min_x << "  min_y = " << min_y
        << "  max_x = " << max_x << "  max_y = " << max_y;
  }
};

class OccupancyGridCoordinate {
 public:
  OccupancyGridCoordinate() = default;

  virtual void Process(const Pose2D &ogm_pose,
                       const double _ogm_resolution = ogm_resolution);

  virtual void Process(const OccupancyGridBound &bound,
                       const double _ogm_resolution = ogm_resolution);

  const bool IsIndexValid(const OgmIndex &id) const;

  const double GetBoundMinX() const { return bound_.min_x; }

  const double GetBoundMinY() const { return bound_.min_y; }

  const double GetOgmResolutionInv() const { return ogm_resolution_inv_; }

  // ogm local pose to index
  void OgmPoseToIndex(OgmIndex *index, const Pose2D &point);

  // slot local pose to index
  const bool SlotPoseToIndex(OgmIndex *index, const Pose2D &point);

  const OccupancyGridBound &GetOGMBound() const { return bound_; }

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

  double ogm_resolution_ = ogm_resolution;

  double ogm_resolution_inv_ = 1 / ogm_resolution_;
};
}  // namespace planning