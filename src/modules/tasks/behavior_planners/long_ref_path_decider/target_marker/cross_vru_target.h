#pragma once

#include <vector>

#include "behavior_planners/speed_limit_decider/speed_limit_decider_output.h"
#include "common/st_graph/st_graph_utils.h"
#include "common/trajectory1d/piecewise_jerk_acceleration_trajectory1d.h"
#include "lon_target_maker.pb.h"
#include "session.h"
#include "target.h"

namespace planning {

constexpr double kInvalid = -1.0;

struct CrossVRUProfileParams {
  double v0 = 11.35;
  double s0 = 3.5;
  double T = 0.5;
  double a = 2.0;
  double b = 3.0;
  double delta = 4.0;
  double b_hard = 4.5;
  double max_a_jerk = 4.0;
  double max_b_jerk = 1.5;
  double default_front_s = 200;
  double cool_factor = 0.99;
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

  void GenerateCrossVRUTarget();
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
  void MakeYieldOrOvertakeDecision(
      const agent::Agent* agent, const double interaction_t,
      const std::shared_ptr<planning_math::KDPath>& ego_lane_coord,
      const double rear_edge_to_rear_axle,
      const PlanningInitPoint& planning_init_point, bool* is_need_overtake);
  SecondOrderTimeOptimalTrajectory GenerateOvertakeTrajByJLT(
      const double v_target, const PlanningInitPoint& planning_init_point);
  void AddCrossVRUTargetDataToProto();

 private:
  CrossVRUProfileParams params_;
  planning::common::CrossVRUTarget cross_vru_target_pb_;
  std::vector<double> cross_vru_agent_ids_;
  std::vector<CrossVRUAgentInfo> agent_infos_;
};

}  // namespace planning
