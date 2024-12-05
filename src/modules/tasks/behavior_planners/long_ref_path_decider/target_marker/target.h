#pragma once

#include "agent/agent.h"
#include "ego_planning_config.h"
#include "math/acc_curve_maker/target_follow_curve.h"
#include "session.h"

namespace planning {

enum class TargetType {
  kFollow,
  kOvertake,
  kCruiseSpeed,
  kNeighbor,
  kNeighborYeild,
  kNeighborOvertake,
  kNotSet,
  kFollowCurve,
  kCautionYield,
};

class TargetValue {
 public:
  TargetValue(const double relative_t, const double has_target,
              const double s_target_val, const double v_target_val,
              const TargetType type)
      : relative_t_(relative_t),
        has_target_(has_target),
        s_target_val_(s_target_val),
        v_target_val_(v_target_val),
        target_type_(type) {}

  void set_relative_t(const double relative_t) { relative_t_ = relative_t; }

  void set_has_target(const bool has_target) { has_target_ = has_target; }

  void set_s_target_val(const double s_target_val) {
    s_target_val_ = s_target_val;
  }

  void set_v_target_val(const double v_target_val) {
    v_target_val_ = v_target_val;
  }

  void set_target_type(const TargetType target_type) {
    target_type_ = target_type;
  }

  double relative_t() const { return relative_t_; }

  bool has_target() const { return has_target_; }

  double s_target_val() const { return s_target_val_; }

  double v_target_val() const { return v_target_val_; }

  TargetType target_type() const { return target_type_; }

 private:
  double relative_t_ = 0.0;
  bool has_target_ = false;
  double s_target_val_ = 0.0;
  double v_target_val_ = 0.0;
  TargetType target_type_ = TargetType::kNotSet;
};

class Target {
 public:
  Target(const SpeedPlannerConfig& config, framework::Session* session);

  virtual ~Target() = default;

  struct CipvInfo {
    int32_t agent_id = -1;
    double vel = 0.0;
    double upper_bound_s = 0.0;
    bool is_large_vehicle = false;
    agent::AgentType type = agent::AgentType::UNKNOWN;
    bool is_tfl_virtual_obs = false;
  };

  virtual TargetValue target_value(const double t) const;

  virtual bool has_target(const double t) const;

  static TargetValue TargetMax(const TargetValue& v1, const TargetValue& v2) {
    return v1.s_target_val() > v2.s_target_val() ? v1 : v2;
  }

  static TargetValue TargetMin(const TargetValue& v1, const TargetValue& v2) {
    return v1.s_target_val() > v2.s_target_val() ? v2 : v1;
  }

  static std::string TargetValueType(const TargetType& type) {
    if (type == TargetType::kCruiseSpeed) {
      return "kCruiseSpeed";
    } else if (type == TargetType::kFollow) {
      return "kFollow";
    } else if (type == TargetType::kOvertake) {
      return "kOvertake";
    } else if (type == TargetType::kNeighbor) {
      return "kNeighbor";
    } else if (type == TargetType::kNeighborYeild) {
      return "kNeighborYeild";
    } else if (type == TargetType::kNeighborOvertake) {
      return "kNeighborOvertake";
    } else {
      return "kNotSet";
    }
  }

  std::unique_ptr<TargetFollowCurve> MakeTargetFollowCurve();

  std::unique_ptr<Trajectory1d> MakeVirtualZeroAccCurve();

 private:
  std::unique_ptr<Trajectory1d> MakeMaxSpeedLimitCurve();

 protected:
  framework::Session* session_;
  const SpeedPlannerConfig& config_;
  std::vector<TargetValue> target_values_;
  std::array<double, 3> init_lon_state_;
  int32_t plan_points_num_ = 0;
  double planning_time_ = 0.0;
  double dt_ = 0.0;

  std::unique_ptr<Trajectory1d> virtual_zero_acc_curve_;
  std::unique_ptr<Trajectory1d> max_speed_limit_curve_;

  // for target follow curve
  std::unique_ptr<TargetFollowCurve> target_follow_curve_;
  bool has_follow_target_curve_ = false;
  CipvInfo cipv_info_;
};

}  // namespace planning