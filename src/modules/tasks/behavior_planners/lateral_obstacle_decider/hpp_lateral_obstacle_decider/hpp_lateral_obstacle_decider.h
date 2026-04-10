#pragma once

#include <memory>
#include <unordered_map>
#include <vector>
#include "frenet_obstacle.h"
#include "obstacle.h"
#include "session.h"
#include "tasks/behavior_planners/lateral_obstacle_decider/ARAStar/hybrid_ara_star.h"
#include "tasks/behavior_planners/lateral_obstacle_decider/base_lateral_obstacle_decider.h"
#include "tasks/task.h"
#include "tasks/task_interface/lateral_obstacle_decider_output.h"
#include "utils/kd_path.h"
#include "hpp_lateral_obstacle_decider_result.pb.h"
#include "hpp_parameter_util.h"

namespace planning {
struct ObstacleConsistencyInfo {
  int count = 0;
  LatObstacleDecisionType last_decision = LatObstacleDecisionType::IGNORE;
  double last_seen_timestamp = 0.0;
};
using ObstacleConsistencyMap =
    std::unordered_map<uint32_t, ObstacleConsistencyInfo>;
using ObstacleLateralDecisionMap =
    std::unordered_map<uint32_t, LatObstacleDecisionType>;

class HppLateralObstacleDecider : public BaseLateralObstacleDecider {
 public:
  HppLateralObstacleDecider(const EgoPlanningConfigBuilder* config_builder,
                            framework::Session* session);
  virtual ~HppLateralObstacleDecider() = default;

  bool Execute() override;
  bool ExecuteTest(bool pipeline_test);

 private:
  bool CheckEnableSearch(
      const std::shared_ptr<ReferencePath>& reference_path_ptr,
      SearchResult search_result);
  bool ARAStar();
  bool CheckARAStarPath(const ara_star::HybridARAStarResult& result);

  void UpdateLatDecision(
      const std::shared_ptr<ReferencePath>& reference_path_ptr,
      const ObstacleConsistencyMap& obstacle_consistency_map,
      const ObstacleClusterContainer& obs_cluster_container,
      const ObstacleClassificationResult& obs_classification_result);

  //辅助函数1：处理单个 Cluster 的决策逻辑
  // 该函数被下面的函数替代
  void MakeDecisionForSingleDynamicObs(
      const std::shared_ptr<ReferencePath>& reference_path_ptr,
      const std::shared_ptr<FrenetObstacle>& obstacle,
      LatObstacleDecisionType& decision);
  void MakeDecisionForStaticCluster(
      const ObstacleCluster& cluster,
      const ObstacleConsistencyMap& obstacle_consistency_map,
      const ObstacleClassificationResult& obs_classification_result,
      LatObstacleDecisionType& decision);

  void MakeDecisionBasedPassageWidth(const ObstacleCluster& cluster,
                                     LatObstacleDecisionInfo& decision_info);
  LatObstacleDecisionType GetLastDecisionInfo(
      const ObstacleCluster& cluster,
      const ObstacleConsistencyMap& obstacle_consistency_map);
  void MakeDecisionBasedRelativePos(
      const ObstacleCluster& cluster,
      const LatObstacleDecisionInfo& previous_decision_info,
      LatObstacleDecisionInfo& decision_info);

  void MakeDecisionBasedReferPath(
      const ObstacleCluster& cluster,
      const ObstacleConsistencyMap& obstacle_consistency_map,
      const std::shared_ptr<ReferencePath>& reference_path_ptr,
      const std::vector<PathPoint>& refer_path,
      LatObstacleDecisionInfo& decision_info);

  void MakeFinalDecision(const ObstacleCluster& cluster,
                         const ObstacleConsistencyMap& obstacle_consistency_map,
                         LatObstacleDecisionInfo& passage_width_info,
                         LatObstacleDecisionInfo& relative_pos_info,
                         LatObstacleDecisionInfo& last_path_info,
                         LatObstacleDecisionType& decision);
  void AnalyzeNudgeLevelBaseCurve(
      const ObstacleCluster& cluster,
      const LatObstacleDecisionInfo& previous_decision_info,
      LatObstacleDecisionInfo& decision_info);
  bool JudgeObsAndEgoInSameStraightLane(
      const std::shared_ptr<ReferencePath>& reference_path_ptr,
      const ObstacleCluster& cluster);
  void SerializeStaticObsDecideResultToDebugInfo(
      const ObstacleCluster& cluster,
      const LatObstacleDecisionInfo& passage_width_info,
      const LatObstacleDecisionInfo& relative_pos_info,
      const LatObstacleDecisionInfo& last_path_info,
      const ObstacleClassificationResult& obs_classification_result,
      const LatObstacleDecisionType& decision);
  void SerializeDynamicObsDecideResultToDebugInfo(
      const std::shared_ptr<FrenetObstacle>& obstacle,
      const LatObstacleDecisionType& decision);
  //辅助函数2：更新历史记录状态机
  void UpdateObstacleConsistencyMap(
      const ObstacleLateralDecisionMap& lat_obstacle_decision,
      ObstacleConsistencyMap& obs_consistency_map);

  void UpdateLatDecisionWithARAStar(
      const std::shared_ptr<ReferencePath>& reference_path_ptr);

  void SaveObstaleToEnvironmentModelDebug(
      const std::shared_ptr<ReferencePath>& reference_path_ptr,
      const ObstacleLateralDecisionMap& lat_obstacle_decision);

 private:
  std::unique_ptr<HybridARAStar> hybrid_ara_star_ = nullptr;
  SearchResult search_result_;
  ObstacleConsistencyMap obstacle_consistency_map_;
  HppLateralObstacleDeciderConfig hpp_general_lateral_decider_config_;
};

}  // namespace planning