
#include "hierarchy_occupancy_grid_map.h"

#include <cstdint>

#include "ogm_common.h"
#include "pose2d.h"

namespace planning {

void HierarchyOccupancyGridMap::Init() {
  for (auto &layer : hierarchy_ogm_) {
    layer.Init();
  }
}

void HierarchyOccupancyGridMap::Clear() {
  for (auto &layer : hierarchy_ogm_) {
    layer.Clear();
  }

  low_obs_flag_ = false;
}

void HierarchyOccupancyGridMap::Process(const Pose2f &ogm_pose,
                                        const float _ogm_resolution) {
  for (auto &layer : hierarchy_ogm_) {
    layer.Process(ogm_pose, _ogm_resolution);
  }
}

void HierarchyOccupancyGridMap::Process(const OccupancyGridBound &bound,
                                        const float _ogm_resolution) {
  for (auto &layer : hierarchy_ogm_) {
    layer.Process(bound, _ogm_resolution);
  }
}

void HierarchyOccupancyGridMap::AddPointCloudObstacleInLayers(
    const PointCloudObstacle &obs) {
  const size_t layer_idx = ClassificationOfObsHeightType(obs.height_type);
  if (layer_idx >= 0 && layer_idx < hierarchy_ogm_.size()) {
    hierarchy_ogm_[layer_idx].AddSlotCoordinatePoints(obs.points);
  }
}

void HierarchyOccupancyGridMap::AddParkingObs(const ParkObstacleList &obs,
                                              const bool use_hright_info) {
  hierarchy_ogm_[0].AddSlotCoordinatePoints(obs.virtual_obs);  // 默认HIGH层
  if (use_hright_info) {
    for (const auto &pc_obs : obs.point_cloud_list) {
      if (pc_obs.height_type == apa_planner::ApaObsHeightType::LOW ||
          pc_obs.height_type == apa_planner::ApaObsHeightType::RUN_OVER) {
        // todo: 远处障碍物也应视为low；
        low_obs_flag_ = true;
      }

      AddPointCloudObstacleInLayers(pc_obs);
    }
    hierarchy_ogm_size_ = 2;
  } else {
    for (const auto &pc_obs : obs.point_cloud_list) {
      hierarchy_ogm_[0].AddSlotCoordinatePoints(pc_obs.points);
    }
    hierarchy_ogm_size_ = 1;
  }
}

void HierarchyOccupancyGridMap::TransformToMatrix(
    cv::Mat *mat, const size_t layer_idx) const {
  if (layer_idx < 0 || layer_idx >= hierarchy_ogm_.size()) return;
  hierarchy_ogm_[layer_idx].TransformToMatrix(mat);
}

const size_t HierarchyOccupancyGridMap::ClassificationOfObsHeightType(
    const apa_planner::ApaObsHeightType &type) const {
  size_t height_idx = 0;
  switch (type) {
    case apa_planner::ApaObsHeightType::RUN_OVER:
    case apa_planner::ApaObsHeightType::LOW:
      height_idx = 1;
      break;
    case apa_planner::ApaObsHeightType::MID:
    case apa_planner::ApaObsHeightType::HIGH:
    default:
      height_idx = 0;
      break;
  }
  return height_idx;
}

const OccupancyGridMap &HierarchyOccupancyGridMap::GetLayer(
    const apa_planner::ApaObsHeightType type) const {
  size_t layer_idx = 0;
  switch (type) {
    case apa_planner::ApaObsHeightType::RUN_OVER:
    case apa_planner::ApaObsHeightType::LOW:
      layer_idx = 1;
      break;
    case apa_planner::ApaObsHeightType::MID:
    case apa_planner::ApaObsHeightType::HIGH:
    default:
      layer_idx = 0;
      break;
  }
  return hierarchy_ogm_[layer_idx];
};

void HierarchyOccupancyGridMap::SethHierarchyOGMSize(const size_t i) {
  hierarchy_ogm_size_ = i;
}

}  // namespace planning