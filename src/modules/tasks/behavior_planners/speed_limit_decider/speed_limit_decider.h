#pragma once

#include <cstdint>

#include "agent/agent.h"
#include "ego_planning_config.h"
#include "speed_limit_decider_output.h"
#include "tasks/task.h"
#include "traffic_light_decision_manager.h"
#include "virtual_lane_manager.h"

namespace planning {
struct CurvInfo {
  int curv_sign;
  double s;
  double curv;
};

struct VRURoundInfo {
  int32_t id = -1;
  double distance_to_ego = -1.0;
  bool is_satisfied_round = false;
  bool last_is_satisfied_round = false;
  double ttc = 100;
  int32_t enter_counter = 0;
  bool is_trigger = false;
  bool is_lost = false;
  bool last_is_lost = false;
  bool is_distance_exit = false;
  bool last_is_distance_exit = false;
};
class SpeedLimitDecider : public Task {
 public:
  SpeedLimitDecider(const EgoPlanningConfigBuilder *config_builder,
                    framework::Session *session);
  virtual ~SpeedLimitDecider() = default;

  bool Execute() override;

 private:
  void CalculateMapSpeedLimit();

  void CalculateCurveSpeedLimit();

  void CalculateStaticAgentLimit();

  void CalculateIntersectionSpeedLimit();

  void CalculatePerceptVisibSpeedLimit();

  void CalculatePOISpeedLimit();

  void CalculateLaneBorrowSpeedLimit();

  void CalculateSpeedLimitFromTFLDis();

  void CalculateSpeedLimitForDangerousObstacle();

  void CalculateAvoidAgentSpeedLimit();

  void CalculateFunctionFadingAwaySpeedLimit();
  void CalculateVRURoundSpeedLimit();

  bool IsSSharpBend(const std::vector<CurvInfo> &preview_curv_info_vec);

  bool HasTriggeredVRU(const std::map<int32_t, VRURoundInfo> &vru_round_map);
  double JudgeCurvBySDProMap();

  // used in curv speed limit
  const std::vector<double> _A_TOTAL_MAX_BP{0., 20., 40.};
  const std::vector<double> _A_TOTAL_MAX_V{1.5, 1.9, 3.2};
  const std::vector<double> _AY_MAX_ABS_BP{5.0, 10.0, 15.0, 30.0};
  const std::vector<double> _AY_MAX_STEERS{2.0, 1.8, 1.6, 1.6};
  const std::vector<double> _AY_MAX_CURV_BP{50, 100, 200, 300, 400};
  const std::vector<double> _AY_MAX_CURV_V{2.2, 1.6, 1.4, 1.2, 1.1};

  SpeedLimitConfig speed_limit_config_;  // all configs
  double v_target_;                      // final v target
  SpeedLimitType v_target_type_;         // final v target type
  double v_avoid_hold_ = 0.0;
  pnc::filters::SlopeFilter vel_slope_filter_function_fading_away_;
  double last_vel_function_fading_away_;
  double v_cruise_limit_;  // kph

  // used in intersection speed limit
  planning::common::IntersectionState last_intersection_state_ =
      planning::common::UNKNOWN;
  planning::common::IntersectionState current_intersection_state_ =
      planning::common::UNKNOWN;
  double v_limit_with_intersection_ = 0.0;

  double v_limit_for_dangerous_obstacle_ = 0.0;
  bool is_function_fading_away_ = false;
  iflyauto::RequestReason request_reason_ =
      iflyauto::RequestReason::REQUEST_REASON_NO_REASON;

  std::map<int32_t, VRURoundInfo> vru_round_map_;
  std::map<int32_t, VRURoundInfo> historical_vru_round_map_;
  VRURoundInfo triggered_vru_;
  bool vru_round_triggered_ = false;
};

}  // namespace planning
