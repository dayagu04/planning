#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "frenet_obstacle.h"
#include "hpp_lateral_obstacle_decider_result.pb.h"
#include "hpp_parameter_util.h"
#include "obstacle.h"
#include "session.h"
#include "tasks/behavior_planners/lateral_obstacle_decider/ARAStar/hybrid_ara_star.h"
#include "tasks/behavior_planners/lateral_obstacle_decider/base_lateral_obstacle_decider.h"
#include "tasks/task.h"
#include "tasks/task_interface/lateral_obstacle_decider_output.h"
#include "utils/kd_path.h"

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
  // ========== 搜索决策（ARA*） ==========
  bool CheckEnableSearch(
      const std::shared_ptr<ReferencePath>& reference_path_ptr,
      SearchResult search_result);
  bool ARAStar();
  bool CheckARAStarPath(const ara_star::HybridARAStarResult& result);
  void UpdateLatDecisionWithARAStar(
      const std::shared_ptr<ReferencePath>& reference_path_ptr);

  // ========== 决策更新与一致性维护 ==========
  void UpdateLatDecision(
      const std::shared_ptr<ReferencePath>& reference_path_ptr,
      const ObstacleConsistencyMap& obstacle_consistency_map,
      const ObstacleClusterContainer& obs_cluster_container,
      const ObstacleClassificationResult& obs_classification_result);
  void UpdateObstacleConsistencyMap(
      const ObstacleLateralDecisionMap& lat_obstacle_decision,
      ObstacleConsistencyMap& obs_consistency_map);
  void SaveObstaleToEnvironmentModelDebug(
      const std::shared_ptr<ReferencePath>& reference_path_ptr,
      const ObstacleLateralDecisionMap& lat_obstacle_decision);

  // ========== 共用辅助函数 ==========
  bool JudgeObsAndEgoInSameStraightLane(
      const std::shared_ptr<ReferencePath>& reference_path_ptr,
      const double obstacle_center_s);
  void AnalyzeNudgeLevelBaseCurve(
      const FrenetObstacleBoundary& obs_boundary,
      const LatObstacleDecisionInfo& previous_decision_info,
      LatObstacleDecisionInfo& decision_info);
  double CalcBufferBasedOnEnv(const FrenetObstacleBoundary& frenet_boundary,
                              const std::vector<int>& original_ids,
                              ConstReferencePathPtr reference_path_ptr);
  void DecideBasedRelativePos(
      const FrenetObstacleBoundary& obs_boundary,
      const FrenetBoundary& ego_boundary,
      const LatObstacleDecisionInfo& previous_decision_info,
      LatObstacleDecisionInfo& decision_info);
  void DecideBasedPassageWidth(const FrenetObstacleBoundary& frenet_boundary,
                               const std::vector<int>& original_ids,
                               LatObstacleDecisionInfo& decision_info);
  // 基于分层决策结果融合产生最终决策
  void FuseLayeredDecisions(const LatObstacleDecisionInfo& passage_width_info,
                            const LatObstacleDecisionInfo& relative_pos_info,
                            const LatObstacleDecisionInfo& last_path_info,
                            LatObstacleDecisionInfo& decision_info);

  // ========== 动态障碍物决策 ==========
  void MakeDecisionForSingleDynamicObs(
      const std::shared_ptr<ReferencePath>& reference_path_ptr,
      const std::shared_ptr<FrenetObstacle>& obstacle,
      const ObstacleClassificationResult& obs_classification_result,
      LatObstacleDecisionType& decision);
  void DecideNonCrossingDynamicObs(
      const std::shared_ptr<FrenetObstacle>& obstacle,
      LatObstacleDecisionType& decision);
  void DecideCrossingDynamicObs(
      const std::shared_ptr<ReferencePath>& reference_path_ptr,
      const TrajectoryPoints& last_traj_points,
      const std::shared_ptr<FrenetObstacle>& obstacle,
      LatObstacleDecisionType& decision);
  void AggregateTrajPointDecisions(
      const std::vector<LatObstacleDecisionInfo>& per_point_decision_vec,
      const double min_obs_2left_road_boundary_mindis,
      const double min_obs_2right_road_boundary_mindis,
      LatObstacleDecisionInfo& curr_frame_decision_info);
  void SerializeDynamicObsDecideResultToDebugInfo(
      const std::shared_ptr<FrenetObstacle>& obstacle,
      const LatObstacleDecisionType& decision);

  // ========== 静态障碍物（Cluster）决策 ==========
  void MakeDecisionForStaticCluster(
      const ObstacleCluster& cluster,
      const ObstacleConsistencyMap& obstacle_consistency_map,
      const ObstacleClassificationResult& obs_classification_result,
      LatObstacleDecisionType& decision);
  void GetLastDecisionInfo(
      const std::vector<int>& original_ids,
      const ObstacleConsistencyMap& obstacle_consistency_map,
      LatObstacleDecisionInfo& decision_info);
  void DecideBasedReferPath(
      const ObstacleCluster& cluster,
      const ObstacleConsistencyMap& obstacle_consistency_map,
      const std::shared_ptr<ReferencePath>& reference_path_ptr,
      const std::vector<PathPoint>& refer_path,
      LatObstacleDecisionInfo& decision_info);
  void SerializeStaticObsDecideResultToDebugInfo(
      const ObstacleCluster& cluster,
      const LatObstacleDecisionInfo& passage_width_info,
      const LatObstacleDecisionInfo& relative_pos_info,
      const LatObstacleDecisionInfo& last_path_info,
      const ObstacleClassificationResult& obs_classification_result,
      const LatObstacleDecisionType& decision);

  // ========== 闸机决策 ==========
  void MakeDecisionForTurnstile(
      const std::shared_ptr<ReferencePath>& reference_path_ptr,
      const std::shared_ptr<FrenetObstacle>& obstacle,
      const std::unordered_map<int, TurnstileInfo>& id_2_turnstile_info,
      LatObstacleDecisionType& decision);

 private:
  std::unique_ptr<HybridARAStar> hybrid_ara_star_ = nullptr;
  SearchResult search_result_;
  ObstacleConsistencyMap obstacle_consistency_map_;
  HppLateralObstacleDeciderConfig hpp_general_lateral_decider_config_;
};

}  // namespace planning