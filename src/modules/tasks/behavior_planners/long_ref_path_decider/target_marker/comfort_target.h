#pragma once

#include <cstdint>

#include "lon_target_maker.pb.h"
#include "target.h"

namespace planning {

class ComfortTarget : public Target {
  enum class FollowAgentSource {
    kLatObstacleDecision = 0,
    kLonCutinAgentIds = 1,
    kJointDangerAgentIds = 2
  };
  struct UpperBoundInfo {
    double s = 0.0;
    double t = 0.0;
    double v = 0.0;
    TargetType target_type = TargetType::kNotSet;
    int32_t agent_id = -1;
    int64_t st_boundary_id = -1;
    double a = 0;
    bool is_lat_follow = false;
    bool is_lon_cut_in = false;
    bool is_joint_danger = false;
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
    double emergency_decel_jerk;
    double virtual_front_s;
    double cool_factor;
    double delay_time_buffer;
    double static_speed_threshold;
    double eps;
    double emergency_ttc_threshold;
    double follow_max_st_boundary_t = 3.0;
  };

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

  void ProcessCutinAgents(const std::vector<int32_t>& agent_ids,
                          FollowAgentSource source,
                          const std::unordered_set<int32_t>& forbidden_ids,
                          std::unordered_set<int32_t>& added_agent_ids,
                          int32_t parallel_overtake_agent_id,
                          bool is_confluence_area,
                          std::vector<FollowAgentWithSource>& follow_agents);

  void GenerateComfortTarget();

  double CalculateComfortAcceleration(
      const double current_acc, const double current_vel,
      const double current_s, const double front_vel, const double front_s,
      const double tau, const double decel_jerk, double& v_target) const;

  void AddComfortTargetDataToProto();

 private:
  ComfortParameters comfort_params_;
  std::vector<UpperBoundInfo> upper_bound_infos_;
  common::ComfortTarget comfort_target_pb_;
  std::vector<double> acc_values_;
  std::vector<int32_t> follow_agent_ids_;
  std::vector<double> rule_base_cutin_agent_ids_;
  std::vector<int32_t> joint_danger_agent_ids_;
  std::unordered_set<int32_t> upper_bound_agent_ids_;
  bool is_lat_follow_ = false;
  bool is_lon_cut_in_ = false;
  bool is_lon_emergency_stop_ = false;
  std::vector<double> comfort_jerk_min_vec_;
  std::vector<double> comfort_v_target_vec_;
};

}  // namespace planning