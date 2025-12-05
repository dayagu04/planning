#pragma once

#include <vector>

#include "lon_target_maker.pb.h"
#include "session.h"
#include "target.h"

namespace planning {

constexpr double kInvalid = -1.0;

struct CrossVRUProfileParams {
  double v0;
  double s0;
  double T;
  double a;
  double b;
  double b_max;
  double delta;
  double b_hard;
  double max_a_jerk;
  double max_b_jerk;
  double default_front_s;
  double cool_factor;
  double end_time_buffer;
  double kEps = 1e-6;
};

struct CrossVRUAgentInfo {
  int32_t agent_id;
  double crossing_start_time = kInvalid;
  double crossing_end_time = kInvalid;
  double headway_time = kInvalid;
  std::vector<double> agent_traj_s;
  std::vector<double> agent_traj_v;
};

class CrossVRUTarget : public Target {
 public:
  CrossVRUTarget(const SpeedPlannerConfig& config, framework::Session* session);
  ~CrossVRUTarget() = default;

  const std::vector<double> _L_SLOPE_BP{0.0, 40.0};
  const std::vector<double> _L_SLOPE_V{0.35, 0.08};
  const std::vector<double> _P_SLOPE_BP{0., 40.0};
  const std::vector<double> _P_SLOPE_V{0.8, 0.2};

  void GenerateCrossVRUTarget();

  double CalcDesiredVelocity(const double d_rel, const double d_des,
                             const double v_lead, const double v_ego) const;

  void AnalyzeCrossVRUAgentsAndInitialize();

  double CalculateVRUDeceleration(
      const double current_vel, const double current_s,
      const double current_acc, const int32_t index,
      const std::vector<CrossVRUAgentInfo>& agent_infos) const;

  double CalculateVRUDecelerationCore(const double current_vel,
                                      const double current_s,
                                      const double front_s,
                                      const double front_vel,
                                      const double headway_time) const;

  void AddCrossVRUTargetDataToProto();

 private:
  CrossVRUProfileParams params_;
  planning::common::CrossVRUTarget cross_vru_target_pb_;
  std::vector<double> cross_vru_agent_ids_;
  std::vector<CrossVRUAgentInfo> agent_infos_;
  bool is_pre_handle_cross_vru_ = false;
};

}  // namespace planning
