#pragma once

#include <cstdint>
#include <map>
#include <memory>

#include "agent/agent.h"
#include "common/config/basic_type.h"
#include "construction_scene_manager.h"
#include "ego_planning_config.h"
#include "filters.h"
#include "planning_context.h"
#include "session.h"
#include "traffic_light_decision_manager.h"
#include "utils/kd_path.h"
#include "virtual_lane_manager.h"

namespace planning {
namespace lane_change_joint_decision {

struct JointDecisionCurvInfo {
  int curv_sign;
  double s;
  double curv;
};

class JointDecisionSpeedLimit {
 public:
  JointDecisionSpeedLimit(const EgoPlanningConfigBuilder* config_builder,
                          framework::Session* session);
  ~JointDecisionSpeedLimit() = default;

  struct Result {
    double speed_limit;
    int type;
  };
  Result CalculateSpeedLimit();

 private:
  void CalculateMapSpeedLimit();

  void CalculateCurveSpeedLimit();

  void CalculateIntersectionSpeedLimit();

  void CalculatePOISpeedLimit();

  void CalculateSpeedLimitFromTFLDis();

  void CalculateConstructionZoneSpeedLimit();

  void CalculateFunctionFadingAwaySpeedLimit();

  bool IsSSharpBend(
      const std::vector<JointDecisionCurvInfo>& preview_curv_info_vec) const;

  double JudgeCurvBySDProMap() const;

  bool CheckClustersConsecutiveDiffSlidingWindow(
      const std::map<int, ConstructionAgentClusterArea>& cluster_map,
      const std::shared_ptr<planning_math::KDPath>& planned_kd_path,
      bool entering) const;

  // used in curv speed limit
  const std::vector<double> _A_TOTAL_MAX_BP{0., 20., 40.};
  const std::vector<double> _A_TOTAL_MAX_V{1.5, 1.9, 3.2};
  const std::vector<double> _AY_MAX_ABS_BP{5.0, 10.0, 15.0, 30.0};
  const std::vector<double> _AY_MAX_STEERS{2.0, 1.8, 1.6, 1.6};
  const std::vector<double> _AY_MAX_CURV_BP{50, 100, 200, 300, 400};
  const std::vector<double> _AY_MAX_CURV_V{2.2, 1.6, 1.4, 1.2, 1.1};

  framework::Session* session_;
  SpeedLimitConfig speed_limit_config_;  // all configs
  double v_target_;                      // final v target
  int v_target_type_;                    // final v target type
  double v_cruise_limit_;                // kph

  // used in intersection speed limit
  planning::common::IntersectionState last_intersection_state_ =
      planning::common::UNKNOWN;
  planning::common::IntersectionState current_intersection_state_ =
      planning::common::UNKNOWN;
  double v_limit_with_intersection_ = 0.0;

  bool poi_v_limit_set_ = false;

  bool construction_strong_deceleration_mode = false;
  int construction_strong_mode_frame_count_ = 0;

  // Function fading away
  bool is_function_fading_away_ = false;
  iflyauto::RequestReason request_reason_ =
      iflyauto::RequestReason::REQUEST_REASON_NO_REASON;
  pnc::filters::SlopeFilter vel_slope_filter_function_fading_away_;
  double last_vel_function_fading_away_;

 public:
  // Make constants public for use in anonymous namespace functions
  static constexpr double kEpsilon = 1e-6;
  static constexpr double kHighVel = 60.0 / 3.6;
  static constexpr double kSSharpBendRadius = 300.0;
  static constexpr double kSSharpBendCurvDis = 30.0;
  static constexpr double kSSharpBendSpeedScaleRatio = 0.8;
  static constexpr double kTFLSpeedLimitDis = 160.0;
  static constexpr double kMergePointDetectedDistance = 20.0;
  static constexpr double kTunnelVelLimitDisOffset = 50.0;
  static constexpr double kCAInvadeLatDisDiffThr = 0.25;
  static constexpr double kCAInvadeVaildLonDis = 80.0;
  static constexpr double kCAInvadeLatMaxDis = 2.5;
  static constexpr double kCAInvadeLatMinDis = 2.0;
  static constexpr int kConstructionStrongMinHoldFrames = 100;
  static constexpr int kConstructionStrongMaxHoldFrames = 600;
};

}  // namespace lane_change_joint_decision
}  // namespace planning
