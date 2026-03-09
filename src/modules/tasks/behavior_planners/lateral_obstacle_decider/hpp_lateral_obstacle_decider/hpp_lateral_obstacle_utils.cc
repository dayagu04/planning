#include "hpp_lateral_obstacle_utils.h"

namespace planning {
bool HppLateralObstacleUtils::GenerateObstaclesToBeConsidered(
    ConstObstacleManagerPtr obstacle_manager_ptr,
    ConstReferencePathPtr reference_path_ptr, ObstacleItemMap& obs_item_map) {
  return true;
}

bool HppLateralObstacleUtils::MergeObstaclesBaseOnPos(
    const ObstacleItemMap& obs_item_map,
    MergedObstacleContainer& merged_obs_constainer) {
  return true;
}

bool HppLateralObstacleUtils::ClassifyObstacles(
    const MergedObstacleContainer& merged_obs_constainer,
    ObstacleClassificationResult& obs_classification_result) {
  return true;
}

bool HppLateralObstacleUtils::MakeDecisionForMovingObstacles(
    const MergedObstacleContainer& merged_obs_constainer,
    const ObstacleClassificationResult&
        obs_classification_result /*TODO: 输出数据结构待定*/) {
  return true;
}

bool HppLateralObstacleUtils::SortMergedObstacles(
    MergedObstacleContainer& merged_obs_contanier) {
  return true;
}
}  // namespace planning