#pragma once

namespace planning {

struct KinematicsBound {
  double acc_positive_mps2 = 0.0;
  double acc_negative_mps2 = 0.0;
  double jerk_positive_mps3 = 0.0;
  double jerk_negative_mps3 = 0.0;
};

class LongitudinalDecisionDeciderOutput {
 public:
  LongitudinalDecisionDeciderOutput() = default;
  ~LongitudinalDecisionDeciderOutput() = default;

  void Reset();

  const KinematicsBound& determined_cruise_bound() const;
  void set_determined_cruise_bound(
      const KinematicsBound& determined_cruise_bound);

 private:
  KinematicsBound determined_cruise_bound_;
};

}  // namespace planning
