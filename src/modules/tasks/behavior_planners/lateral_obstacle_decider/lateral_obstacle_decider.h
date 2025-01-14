#pragma once

#include <unordered_map>

#include "ego_state_manager.h"
#include "session.h"
#include "utils/kd_path.h"
#include "obstacle.h"
#include "frenet_obstacle.h"
#include "tasks/task.h"

namespace planning{

struct LateralObstacleHistoryInfo {
  bool lane_borrow = false;
  bool can_not_avoid = false;
  double ncar_count = 0;
  bool ncar_count_in = false;
  bool is_avd_car = false;
  double close_time = 0;
  double last_recv_time = 0;
  bool front_car = false;
  bool side_car = false;
  bool rear_car = false;
};

class LateralObstacleDecider : public Task {
  public:
  LateralObstacleDecider(const EgoPlanningConfigBuilder *config_builder,
                                            framework::Session *session);
  virtual ~LateralObstacleDecider() = default;

  bool Execute() override;

  private:
  bool IsPotentialAvoidingCar(FrenetObstacle &frenet_obstacle, double lane_width,
                              bool rightest_lane, double farthest_distance);
  void LateralObstacleDecision(FrenetObstacle &frenet_obstacle, double lane_width,
                              double expand_length);

  planning::framework::Session *session_;
  PotentialAvoidDeciderConfig config_;

  std::unordered_map<int, LateralObstacleHistoryInfo> lateral_obstacle_history_info_;
  std::unordered_map<uint16_t, LatObstacleDecisionType>& output_;

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