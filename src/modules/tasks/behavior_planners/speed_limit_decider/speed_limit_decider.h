#pragma once

#include <cstdint>

#include "agent/agent.h"
#include "ego_planning_config.h"
#include "speed_limit_decider_output.h"
#include "tasks/task.h"
#include "traffic_light_decision_manager.h"
#include "virtual_lane_manager.h"
#include "tasks/task_interface/crossing_agent_decider_output.h"

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

  void CalculateConstructionZoneSpeedLimit();

  bool IsSSharpBend(const std::vector<CurvInfo> &preview_curv_info_vec);

  bool HasTriggeredVRU(const std::map<int32_t, VRURoundInfo> &vru_round_map);

  double JudgeCurvBySDProMap(double search_dis);

  double GetRampVelLimit();

  bool IsNearMergeCancelRampVelLimit();

  // Collect and process ramp curvature data from SDProMap
  // Outputs: k_raw (signed), s_vec (with offset), map_speed_limits, k_smooth
  void CollectRampCurvatureData(std::vector<double>* k_raw = nullptr,
                                std::vector<double>* s_vec = nullptr,
                                std::vector<double>* map_speed_limits = nullptr,
                                std::vector<double>* k_smooth = nullptr);

  // Helper functions for ramp processing
  uint64_t FindRampLinkId(
      double dist_to_ramp,
      const std::vector<NOASplitRegionInfo> &split_region_info_list);

  bool CollectPointsFromLink(
      const iflymapdata::sdpro::LinkInfo_Link *link,
      std::vector<ad_common::math::Vec2d> &enu_points, double &total_len,
      size_t start_idx = 0);

  void CollectRampPointsFromLinks(
      const iflymapdata::sdpro::LinkInfo_Link *start_link,
      const std::function<bool(const iflymapdata::sdpro::LinkInfo_Link &)> &is_ramp,
      const std::function<const iflymapdata::sdpro::LinkInfo_Link *(uint64_t)> &get_next_link,
      std::vector<ad_common::math::Vec2d> &enu_points, double &total_len,
      std::vector<double> *map_speed_limits = nullptr);

  // used in curv speed limit
  const std::vector<double> _A_TOTAL_MAX_BP{0., 20., 40.};
  const std::vector<double> _A_TOTAL_MAX_V{1.5, 1.9, 3.2};
  const std::vector<double> _AY_MAX_ABS_BP{5.0, 10.0, 15.0, 30.0};
  const std::vector<double> _AY_MAX_STEERS{2.0, 1.8, 1.6, 1.6};
  const std::vector<double> _AY_MAX_CURV_BP{50, 100, 200, 300, 400};
  const std::vector<double> _AY_MAX_CURV_V{2.2, 1.6, 1.4, 1.35, 1.3};

  SpeedLimitConfig speed_limit_config_;  // all configs
  double v_target_;                      // final v target
  SpeedLimitType v_target_type_;         // final v target type
  double v_avoid_hold_ = 0.0;
  pnc::filters::SlopeFilter vel_slope_filter_function_fading_away_;
  double last_vel_function_fading_away_;
  double v_cruise_limit_;  // kph

  bool ramp_v_limit_set_ = false;
  bool ramp_manual_intervention_detected_ = false;
  double last_v_cruise_fsm_ramp_ = 40.0;

  std::deque<double> dis_to_merge_window_ = {NL_NMAX, NL_NMAX, NL_NMAX};
  double distance_to_merge_ = NL_NMAX;
  int pass_merge_counter_ = 0;
  bool pass_merge_counter_has_set_ = false;

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

  bool poi_v_limit_set_ = false;

  bool construction_strong_deceleration_mode_ = false;
  int construction_strong_mode_frame_count_ = 0;
  bool construction_lat_dist_flag_ = false;
  bool construction_v_limit_set_ = false;
  bool construction_manual_intervention_detected_ = false;
  double last_v_cruise_fsm_ = 0.0;
  double raw_curv_spline_ = 0.0;
  double v_limit_in_turns_filtered_ = 0.0;  // Filtered v_limit_in_turns for EWMA
  bool last_is_sharp_curve_ = false;
  bool last_is_sharp_curve_by_decel_ = false;  // Previous sharp curve state based on deceleration
  int sharp_curve_frame_count_ = 0;  // Frame count counter for maintaining sharp curve state
  bool last_is_map_sharp_curve_ramp_ = false;  // Previous map sharp curve state (for hysteresis, ramp related)
  bool last_condition_ramp_raw_count_ = false;  // Previous k_raw point count condition state (for hysteresis)
  bool last_is_map_sharp_curve_ = false;  // Previous complete is_map_sharp_curve state (for detecting entering state)
  bool last_use_avg_radius_for_ewma_ = false;  // Previous use_avg_radius_for_ewma state (for hysteresis)
  double last_road_radius_origin_ = 10000.0;  // Previous road_radius_origin
  double road_radius_origin_ = 10000.0;
  bool roundabout_quit_flag_ = false;
  int roundabout_recover_counter_ = 0;
};

}  // namespace planning
