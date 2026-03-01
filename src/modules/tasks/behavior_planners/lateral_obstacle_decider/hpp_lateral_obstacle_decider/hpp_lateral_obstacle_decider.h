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
#include "modules/tasks/behavior_planners/lateral_obstacle_decider/hpp_lateral_obstacle_decider/hpp_lateral_obstacle_utils.h"
#include "utils/kd_path.h"

namespace planning {
  struct ObstacleConsistencyInfo {
    int count = 0;
    LatObstacleDecisionType last_decision = LatObstacleDecisionType::IGNORE;
    double last_seen_timestamp = 0.0;
  };

class HppLateralObstacleDecider : public BaseLateralObstacleDecider {
 public:
  HppLateralObstacleDecider(const EgoPlanningConfigBuilder *config_builder,
                            framework::Session *session);
  virtual ~HppLateralObstacleDecider() = default;

  bool Execute() override;
  bool ExecuteTest(bool pipeline_test);

 private:
  bool PreProcessObstacle(
      ConstReferencePathPtr reference_path_ptr,
      MergedObstacleContainer &merged_obs_constainer,
      ObstacleClassificationResult &obs_classification_result);

  bool CheckEnableSearch(
      const std::shared_ptr<ReferencePath> &reference_path_ptr,
      SearchResult search_result);
  bool ARAStar();

  bool CheckARAStarPath(const ara_star::HybridARAStarResult& result);

  void UpdateLatDecision(
      const std::shared_ptr<ReferencePath> &reference_path_ptr);
  void ClearOldConsistencyInfo(
      const std::unordered_set<uint32_t>& current_frame_ids,
      double current_timestamp);
  void UpdateLatDecisionWithARAStar(
      const std::shared_ptr<ReferencePath> &reference_path_ptr);
  void Log(const std::shared_ptr<ReferencePath> &reference_path_ptr);

 private:
  std::unordered_map<uint32_t, LatObstacleDecisionType> output_;
  std::unique_ptr<HybridARAStar> hybrid_ara_star_ = nullptr;
  SearchResult search_result_;
  std::unordered_map<uint32_t, ObstacleConsistencyInfo> obstacle_consistency_map_;
};

}  // namespace planning