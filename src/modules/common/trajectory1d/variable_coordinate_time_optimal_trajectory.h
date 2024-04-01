#pragma once

#include "third_order_time_optimal_trajectory.h"

namespace planning {

// compute a time optimal trajectory to reach end state in a relative coordinate

// can be used to generate time_optimal_trajectory to reach a front car by a
// distance buffer

class VariableCoordinateTimeOptimalTrajectory
    : public ThirdOrderTimeOptimalTrajectory {
 public:
  // @init_state: init state in absolute coordinate
  // @state_limit: state limit in absolute coordinate
  // @base_coordinate: relative coordinate param
  // @p_precision: position precision for end_position
  static VariableCoordinateTimeOptimalTrajectory ConstructInstance(
      const LonState& init_state, const StateLimit& state_limit,
      const CoordinateParam& base_coordinate, const double p_precision);

  // @relative_coordinate_param: relative coordinate param
  // @relative_init_state: init state in relative coordinate
  // @relative_state_limit: state_limit in relative coordinate
  // p_precision: position precision for end_position
  VariableCoordinateTimeOptimalTrajectory(
      const CoordinateParam& relative_coordinate_param,
      const LonState& relative_init_state,
      const StateLimit& relative_state_limit, const double p_precision);

  virtual double Evaluate(const int32_t order,
                          const double param) const override;

 private:
  const CoordinateParam relative_coordinate_param_;
};

}  // namespace planning
