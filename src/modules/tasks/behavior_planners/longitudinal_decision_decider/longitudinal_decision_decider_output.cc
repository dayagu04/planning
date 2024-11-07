#include "longitudinal_decision_decider_output.h"

namespace planning {

void LongitudinalDecisionDeciderOutput::Reset() {}

const KinematicsBound&
LongitudinalDecisionDeciderOutput::determined_cruise_bound() const {
  return determined_cruise_bound_;
}

void LongitudinalDecisionDeciderOutput::set_determined_cruise_bound(
    const KinematicsBound& determined_cruise_bound) {
  determined_cruise_bound_ = determined_cruise_bound;
}

}  // namespace planning
