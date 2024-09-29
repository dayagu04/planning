#pragma once
#include <opencv2/core/types.hpp>

#include "occupancy_grid_coordinate.h"
#include "ogm_common.h"
#include "opencv2/opencv.hpp"
#include "point_cloud_obstacle.h"
#include "pose2d.h"
#include "transform2d.h"

namespace planning {

class OccupancyGridMap : public OccupancyGridCoordinate {
 public:
  OccupancyGridMap() = default;

  void Init();

  void Clear();

  void Process(const Pose2D &ogm_pose,
               const double _ogm_resolution = ogm_resolution) override;

  void Process(const OccupancyGridBound &bound,
               const double _ogm_resolution = ogm_resolution) override;

  template <typename T>
  void AddSlotPoint(const T &point);

  void AddSlotCoordinatePoints(const std::vector<Position2D> &points);

  void AddSlotCoordinatePointCloud(
      const std::vector<PointCloudObstacle> &point_cloud_list);

  void AddParkingObs(const ParkObstacleList &obs);

  template <typename T>
  void AddLineSegment(const T &start, const T &end);

  void TransformToMatrix(cv::Mat *mat) const;

 private:
  // if occupacy, true
  bool ogm[ogm_grid_x_max][ogm_grid_y_max];
};
}  // namespace planning