#pragma once

#include <cstdint>

#include "lon_target_maker.pb.h"
#include "target.h"

namespace planning {

class ComfortTarget : public Target {
  enum class FollowAgentSource { kLatObstacleDecision = 0, kCutinAgentIds = 1 };
  struct UpperBoundInfo {
    double s = 0.0;
    double t = 0.0;
    double v = 0.0;
    TargetType target_type = TargetType::kNotSet;
    int32_t agent_id = -1;
    int64_t st_boundary_id = -1;
    double a = 0;
    bool is_follow = false;
    bool is_cut_in = false;
  };
  struct FollowAgentWithSource {
    const agent::Agent* agent;
    FollowAgentSource source;
  };

 public:
  ComfortTarget(const SpeedPlannerConfig& config, framework::Session* session);
  ~ComfortTarget() = default;

  struct ComfortParameters {
    double v0;
    double s0;
    double T;
    double a;
    double b_max;
    double b;
    double b_hard;
    double delta;
    double max_accel_jerk;
    double min_decel_jerk;
    double max_decel_jerk;
    double virtual_front_s;
    double cool_factor;
    double follow_consider_distance;
    double follow_consider_time_headway;
    double delay_time_buffer;
    double w_speed_low;
    double w_speed_high;
    double w_gap_low;
    double w_gap_high;
    double eps;
  };

  const std::vector<double> _L_SLOPE_BP{0.0, 40.0};
  const std::vector<double> _L_SLOPE_V{0.35, 0.08};
  const std::vector<double> _P_SLOPE_BP{0., 40.0};
  const std::vector<double> _P_SLOPE_V{0.8, 0.2};

  struct FollowAgentInfo {
    int32_t agent_id = 899999;
    double s = 210.0;
    double v = 33.5;
    double a = 0.0;
    int64_t st_boundary_id = 899999;
    FollowAgentSource source = FollowAgentSource::kLatObstacleDecision;
  };

 private:
  void GenerateUpperBoundInfo();

  void GenerateComfortTarget();

  double CalculateComfortAcceleration(
      const double current_acc, const double current_vel,
      const double current_s, const double front_vel, const double front_s,
      const double tau, const double decel_jerk, double& v_target) const;

  double CalcDesiredVelocity(const double d_rel, const double d_des,
                             const double v_lead, const double v_ego) const;

  void AddComfortTargetDataToProto();

 private:
  ComfortParameters comfort_params_;
  std::vector<UpperBoundInfo> upper_bound_infos_;
  common::ComfortTarget comfort_target_pb_;
  std::vector<double> acc_values_;
  std::vector<int32_t> follow_agent_ids_;
  bool is_lat_follow_ = false;
  bool is_lon_cut_in_ = false;
  std::vector<double> comfort_jerk_min_vec_;
  std::vector<double> comfort_v_target_vec_;
  std::vector<double> zero_acc_vel_vec_;
  std::vector<double> zero_acc_acc_vec_;
};

}  // namespace planning