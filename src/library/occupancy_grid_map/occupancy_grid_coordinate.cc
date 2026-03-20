#include "occupancy_grid_coordinate.h"
#include "pose2d.h"

#include "pose2d.h"

namespace planning {

void OccupancyGridCoordinate::Process(const Pose2f &ogm_pose,
                                      const float _ogm_resolution) {
  ogm_tf_.SetBasePose(ogm_pose);
  ogm_base_pose_ = ogm_pose;

  ogm_resolution_ = _ogm_resolution;

  ogm_resolution_inv_ = 1.0 / _ogm_resolution;

  bound_.min_x = ogm_base_pose_.x;
  bound_.max_x = ogm_base_pose_.x + ogm_grid_x_max * _ogm_resolution + 1.0;

  bound_.min_y = ogm_base_pose_.y;
  bound_.max_y = ogm_base_pose_.y + ogm_grid_y_max * _ogm_resolution + 1.0;

  return;
}

void OccupancyGridCoordinate::Process(const OccupancyGridBound &bound,
                                      const float _ogm_resolution) {
  Pose2f ogm_pose(bound.min_x, bound.min_y, 0.0);
  ogm_tf_.SetBasePose(ogm_pose);
  ogm_base_pose_ = ogm_pose;

  ogm_resolution_ = _ogm_resolution;
  ogm_resolution_inv_ = 1.0 / _ogm_resolution;
  bound_ = bound;

  return;
}

const bool OccupancyGridCoordinate::IsIndexValid(const OgmIndex &id) const {
  if (id.x >= ogm_grid_x_max || id.y >= ogm_grid_y_max) {
    return false;
  }

  if (id.x < 0 || id.y < 0) {
    return false;
  }

  return true;
}

void OccupancyGridCoordinate::OgmPoseToIndex(OgmIndex *index,
                                             const Pose2f &point) {
  index->x = std::round(point.x * ogm_resolution_inv_);
  index->y = std::round(point.y * ogm_resolution_inv_);

  return;
}

const bool OccupancyGridCoordinate::SlotPoseToIndex(OgmIndex *index,
                                                    const Pose2f &point) {
  index->x = std::round((point.x - bound_.min_x) * ogm_resolution_inv_);
  index->y = std::round((point.y - bound_.min_y) * ogm_resolution_inv_);

  if (IsIndexValid(*index)) {
    return true;
  }

  return false;
}

}  // namespace planning