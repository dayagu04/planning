#pragma once

#include <memory>
#include <unordered_map>
#include <vector>
#include <unordered_set>
#include <limits>
#include "tasks/task.h"
#include "modules/context/obstacle.h"
#include "modules/context/frenet_obstacle.h"
#include "modules/context/reference_path.h"
#include "modules/context/obstacle_manager.h"
#include "modules//tasks/task_interface/hpp_obstacle_lateral_preprocess_decider_output.h"

namespace planning {
// FrenetObstacle 中包含 Obstacle 的指针
using ObstacleItemMap = std::unordered_map<int, FrenetObstaclePtr>;

/************** 障碍物处理操作类定义 ************ */
class HppObstacleLateralPreprocessDecider : public Task{
 public:
  HppObstacleLateralPreprocessDecider(
      const EgoPlanningConfigBuilder* config_builder,
      framework::Session* session);
  virtual ~HppObstacleLateralPreprocessDecider() = default;

  bool Execute() override;

 private:
  // 1. 障碍物过滤
  bool GenerateObstaclesToBeConsidered(
      ConstReferencePathPtr reference_path_ptr, ObstacleItemMap& obs_item_map);

  // 2. 障碍物分类
  bool ClassifyObstacles(
      const ObstacleItemMap& obs_item_map, const FrenetEgoState& ego_state,
      ObstacleClassificationResult& classification_result);

  // 3: 聚类 (动静分离 + 规则聚类 + 凸包生成)
  bool ClusterObstacles(
      const ObstacleItemMap& obs_item_map,
      const ObstacleClassificationResult& classification_result,
      ObstacleClusterContainer& obstacle_cluster_constainer);

  /************** 障碍物分类相关私有函数定义 ************ */
  ObstacleRelPosType ClassifyObstaclesByRelPos(
      const FrenetEgoState& ego_state, const FrenetObstaclePtr& obs_ptr);

  ObstacleMotionType ClassifyObstaclesByMotion(
      const FrenetObstaclePtr& obs_ptr);

  /************** 障碍物合并相关私有函数定义 ************ */
  bool GenerateClusterCandicates(
      const ObstacleItemMap& obs_item_map,
      const ObstacleClassificationResult& classification_result,
      std::vector<ObstacleClusterCandicate>& cluster_candidates);
  bool CalculateCandidateClusterGraph(
      const ObstacleItemMap& obs_item_map,
      const std::vector<ObstacleClusterCandicate>& cluster_candidates,
      ObstacleClusterGraph& cluster_graph);

  bool DFSGenerateObstacleClusters(
      const std::vector<ObstacleClusterCandicate>& cluster_candidates,
      const ObstacleClusterGraph& cluster_graph, const int curr_idx,
      const ObstacleClusterType cluster_type,
      std::unordered_set<int>& visited_candidate_idxs,
      ObstacleCluster& obstacle_cluster);

  bool BuildObstacleClusterConvexHull(
      const ObstacleItemMap& obs_item_map, ObstacleCluster& obstacle_cluster);

private:
  HppObstacleLateralPreprocessDeciderConfig config_;
};

}  // namespace planning