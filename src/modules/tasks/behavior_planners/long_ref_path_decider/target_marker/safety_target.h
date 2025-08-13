#pragma once

#include "behavior_planners/speed_limit_decider/speed_limit_decider_output.h"
#include "common/trajectory1d/piecewise_jerk_acceleration_trajectory1d.h"
#include "lon_target_maker.pb.h"
#include "session.h"
#include "target.h"

namespace planning {

const std::vector<double> _L_SLOPE_BP{0.0, 40.0};
const std::vector<double> _L_SLOPE_V{0.35, 0.08};
const std::vector<double> _P_SLOPE_BP{0., 40.0};
const std::vector<double> _P_SLOPE_V{0.8, 0.2};

class SafetyTarget : public Target {
  struct UpperBoundInfo {
    double s = 0.0;
    double t = 0.0;
    double v = 0.0;
    TargetType target_type = TargetType::kNotSet;
    int32_t agent_id = -1;
    int64_t st_boundary_id = -1;
  };

 public:
  SafetyTarget(const SpeedPlannerConfig& config, framework::Session* session);
  ~SafetyTarget() = default;

  struct IdmParameters {
    double v0 = 30.0;
    double s0 = 3.5;
    double T = 1.0;
    double a = 1.5;
    double b_max = 2.0;
    double b = 1.0;
    double delta = 4.0;
    double b_hard = 2.0;
    double front_b_hard = 5.0;
    double max_jerk = 1.0;
    double virtual_front_s = 200.0;
    double min_distance = 0.5;
  };

 private:
  void GenerateUpperBoundInfo();

  void GenerateSafetyTarget();

  double CalculateSafetyAcceleration(const double current_vel, const double front_s,
                                     const double front_vel,
                                     const double current_s,
                                     const double current_acc,
                                     const double tau) const;

  double CalcDesiredVelocity(const double d_rel, const double d_des,
                             const double v_lead, const double v_ego) const;

  void AddSafetyTargetDataToProto();

 private:
  IdmParameters idm_params_;

  std::vector<UpperBoundInfo> upper_bound_infos_;

  common::SafetyTarget safety_target_pb_;

  double min_safety_distance_m_ = 4.0;
};

}  // namespace planning