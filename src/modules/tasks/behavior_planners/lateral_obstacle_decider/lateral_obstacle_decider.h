#pragma once

#include <unordered_map>

#include "ego_state_manager.h"
#include "frenet_obstacle.h"
#include "obstacle.h"
#include "session.h"
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

 private:
  bool IsPotentialAvoidingCar(FrenetObstacle &frenet_obstacle,
                              double lane_width, bool rightest_lane,
                              double farthest_distance);
  void LateralObstacleDecision(FrenetObstacle &frenet_obstacle,
                               double lane_width, double expand_length);

  planning::framework::Session *session_;
  LateralObstacleDeciderConfig config_;

  std::unordered_map<uint32_t, LateralObstacleHistoryInfo>
      &lateral_obstacle_history_info_;
  std::unordered_map<uint32_t, LatObstacleDecisionType> &output_;

  // ego info
  double ego_rear_axis_to_front_edge_;
  double ego_length_;
  double ego_width_;
  double ego_head_s_ = 0;
  double ego_head_l_ = 0;
  double ego_v_ = 0;
  double ego_v_s_ = 0;
  double ego_v_l_ = 0;
};

}  // namespace planning