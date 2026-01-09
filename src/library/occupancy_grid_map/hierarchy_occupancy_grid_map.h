#pragma once
#include <array>
#include <opencv2/core/types.hpp>

#include "occupancy_grid_coordinate.h"
#include "occupancy_grid_map.h"
#include "ogm_common.h"
#include "opencv2/opencv.hpp"
#include "point_cloud_obstacle.h"
#include "pose2d.h"
#include "transform2d.h"

namespace planning {

class HierarchyOccupancyGridMap {
 public:
  HierarchyOccupancyGridMap() = default;

  void Init();
  void Clear();

  void Process(const Pose2f &ogm_pose,
               const float _ogm_resolution = ogm_resolution);

  void Process(const OccupancyGridBound &bound,
               const float _ogm_resolution = ogm_resolution);

  void AddPointCloudObstacleInLayers(const PointCloudObstacle &obs);

  template <typename T>
  void AddLineSegment(const T &start, const T &end);

  void AddParkingObs(const ParkObstacleList &obs,
                     const bool use_hright_info = false);

  const OccupancyGridMap &GetLayer(
      const apa_planner::ApaObsHeightType type) const;

  const size_t ClassificationOfObsHeightType(
      const apa_planner::ApaObsHeightType &type) const;

  const std::array<OccupancyGridMap, 2> &GetHierarchyOGM() const {
    return hierarchy_ogm_;
  };

  const size_t GetHierarchyOGMSize() const { return hierarchy_ogm_size_; };

  void TransformToMatrix(cv::Mat *mat, const size_t layer_idx) const;

  void SethHierarchyOGMSize(const size_t i);

 private:
  // 各高度层的占据状态
  std::array<OccupancyGridMap, 2> hierarchy_ogm_;
  size_t hierarchy_ogm_size_ = 2;
  bool low_obs_flag_ = false;
};

}  // namespace planning