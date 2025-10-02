#pragma once

#include "log_glog.h"
#include "ogm_common.h"
#include "pose2d.h"
#include "transform2d.h"

namespace planning {

struct OccupancyGridBound {
  float min_x;
  float min_y;
  float max_x;
  float max_y;

  OccupancyGridBound() = default;

  OccupancyGridBound(const float x_min_, const float y_min_,
                     const float x_max_, const float y_max_)
      : min_x(x_min_), min_y(y_min_), max_x(x_max_), max_y(y_max_){};

  void PrintInfo(const bool enable_log = true) const {
    ILOG_INFO_IF(enable_log)
        << "OccupancyGridBound min_x = " << min_x << "  min_y = " << min_y
        << "  max_x = " << max_x << "  max_y = " << max_y;
  }
};

class OccupancyGridCoordinate {
 public:
  OccupancyGridCoordinate() = default;

  virtual void Process(const Pose2f &ogm_pose,
                       const float _ogm_resolution = ogm_resolution);

  virtual void Process(const OccupancyGridBound &bound,
                       const float _ogm_resolution = ogm_resolution);

  const bool IsIndexValid(const OgmIndex &id) const;

  const float GetBoundMinX() const { return bound_.min_x; }

  const float GetBoundMinY() const { return bound_.min_y; }

  const float GetOgmResolutionInv() const { return ogm_resolution_inv_; }

  // ogm local pose to index
  void OgmPoseToIndex(OgmIndex *index, const Pose2f &point);

  // slot local pose to index
  const bool SlotPoseToIndex(OgmIndex *index, const Pose2f &point);

  const OccupancyGridBound &GetOGMBound() const { return bound_; }

 protected:
  //               ^  x
  //               |
  //               |
  //               |
  //               |
  //               |
  //  y <----------|

  Transform2f ogm_tf_;

  // base pose in slot coordinate system
  Pose2f ogm_base_pose_;

  // in slot system.
  OccupancyGridBound bound_;

  float ogm_resolution_ = ogm_resolution;

  float ogm_resolution_inv_ = 1 / ogm_resolution_;
};
}  // namespace planning