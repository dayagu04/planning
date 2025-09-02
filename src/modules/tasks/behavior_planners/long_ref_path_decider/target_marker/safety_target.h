#pragma once

#include <bits/stdint-intn.h>
#include "behavior_planners/speed_limit_decider/speed_limit_decider_output.h"
#include "common/trajectory1d/piecewise_jerk_acceleration_trajectory1d.h"
#include "lon_target_maker.pb.h"
#include "session.h"
#include "target.h"

namespace planning {

class SafetyTarget : public Target {
  enum class FollowAgentSource { kLatObstacleDecision = 0, kCutinAgentIds = 1 };
  struct UpperBoundInfo {
    double s = 0.0;
    double t = 0.0;
    double v = 0.0;
    TargetType target_type = TargetType::kNotSet;
    int32_t agent_id = -1;
    int64_t st_boundary_id = -1;
    double a = 0;
  };
  struct FollowAgentWithSource {
    const agent::Agent* agent;
    FollowAgentSource source;
  };

 public:
  SafetyTarget(const SpeedPlannerConfig& config, framework::Session* session);
  ~SafetyTarget() = default;

  struct IdmParameters {
    double v0 = 33.5;
    double s0 = 3.5;
    double T = 1.0;
    double a = 1.5;
    double b = 1.0;
    double b_max = 2.0;
    double b_hard = 4.0;
    double delta = 4.0;
    double max_a_jerk = 5.0;
    double max_b_jerk = 1.0;
    double max_deceleration_jerk_lat_follow = 2.0;
    double max_deceleration_jerk_lon_cutin = 4.0;
    double virtual_front_s = 200.0;
    double cool_factor = 0.99;
    double over_speed_factor = 0.3;
    double follow_consider_distance = 10.0;
    double follow_consider_time_headway = 1.5;
  };

  const std::vector<double> _L_SLOPE_BP{0.0, 40.0};
  const std::vector<double> _L_SLOPE_V{0.35, 0.08};
  const std::vector<double> _P_SLOPE_BP{0., 40.0};
  const std::vector<double> _P_SLOPE_V{0.8, 0.2};

  struct FollowAgentInfo {
    int32_t agent_id = 899999;
    double s = 210.0;
    double v = 33.5;
    FollowAgentSource source = FollowAgentSource::kLatObstacleDecision;
  };

 private:
  void GenerateUpperBoundInfo();

  void GenerateSafetyTarget();

  double CalculateSafetyAcceleration(const double current_acc,
                                     const double current_vel,
                                     const double current_s,
                                     const double front_vel,
                                     const double front_s,
                                     const double tau) const;

  double CalcDesiredVelocity(const double d_rel, const double d_des,
                             const double v_lead, const double v_ego) const;

  void AddSafetyTargetDataToProto();

 private:
  bool is_lat_follow_ = false;
  bool is_lon_cutin_ = false;
  IdmParameters idm_params_;

  std::vector<UpperBoundInfo> upper_bound_infos_;

  common::SafetyTarget safety_target_pb_;

  std::vector<double> acc_values_;
  std::vector<int32_t> follow_agent_ids_;
};

}  // namespace planning