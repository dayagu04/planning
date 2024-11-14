#pragma once
#include "ego_planning_config.h"
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
  Target(const EgoPlanningConfigBuilder *config_builder,
         framework::Session *session);

  virtual ~Target() = default;
};

}  // namespace planning