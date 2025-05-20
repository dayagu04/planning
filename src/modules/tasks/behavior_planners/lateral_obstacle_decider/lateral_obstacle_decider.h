#pragma once

#include <memory>
#include <unordered_map>

#include "frenet_obstacle.h"
#include "obstacle.h"
#include "session.h"
#include "src/library/arastar_lib/hybrid_ara_star.h"
#include "tasks/task.h"
#include "tasks/task_interface/lateral_obstacle_decider_output.h"
#include "utils/kd_path.h"

namespace planning {

class LateralObstacleDecider : public Task {
 public:
  LateralObstacleDecider(const EgoPlanningConfigBuilder *config_builder,
                         framework::Session *session);
  virtual ~LateralObstacleDecider() = default;

  bool Execute() override;
  bool ExecuteTest(bool pipeline_test);

 private:
  bool IsPotentialAvoidingCar(FrenetObstacle &frenet_obstacle,
                              double lane_width, bool rightest_lane,
                              double farthest_distance, bool can_left_borrow,
                              bool can_right_borrow);
  void LateralObstacleDecision(
      FrenetObstacle &frenet_obstacle, double lane_width);
  bool CheckEnableSearch(
      const std::shared_ptr<ReferencePath> &reference_path_ptr,
      SearchResult search_result);
  bool ARAStar();
  void UpdateLatDecision(
      const std::shared_ptr<ReferencePath> &reference_path_ptr);
  void UpdateLatDecisionWithARAStar(
      const std::shared_ptr<ReferencePath> &reference_path_ptr);
  void Log(const std::shared_ptr<ReferencePath> &reference_path_ptr);
  bool CalculateCutInAndCross(FrenetObstacle &frenet_obstacle,
                             std::shared_ptr<ReferencePath> reference_path,
                             double lane_width);
  void UpdateLaneBorrowDirection();
  void UpdateIntersection();

 private:
  planning::framework::Session *session_;
  LateralObstacleDeciderConfig config_;
  std::unordered_map<uint32_t, LateralObstacleHistoryInfo>
      &lateral_obstacle_history_info_;
  std::unordered_map<uint32_t, LatObstacleDecisionType> &output_;
  std::unordered_map<uint32_t, LatObstacleDecisionType> last_output_;
  std::unique_ptr<HybridARAStar> hybrid_ara_star_ = nullptr;
  SearchResult &search_result_;
  // ego info
  double ego_rear_axis_to_front_edge_;
  double ego_length_;
  double ego_width_;
  double ego_head_s_ = 0;
  double ego_head_l_ = 0;
  double ego_v_ = 0;
  double ego_v_s_ = 0;
  double ego_v_l_ = 0;
  double ego_rear_edge_to_rear_axle_ = 0;
  bool &in_intersection_;
  int intersection_count_ = 0;
  bool &left_borrow_;
  bool &right_borrow_;
};

}  // namespace planning