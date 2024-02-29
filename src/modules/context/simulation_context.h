#ifndef MODULES_SIMULATION_CONTEXT_
#define MODULES_SIMULATION_CONTEXT_

#include "macro.h"

namespace planning {

class SimulationContext {
 public:
  const double planning_loop_dt() const { return planning_loop_dt_; }
  void set_planning_loop_dt(double planning_loop_dt) {
    planning_loop_dt_ = planning_loop_dt;
  }

  const double prediction_relative_time() const {
    return prediction_relative_time_;
  }
  void set_prediction_relative_time(double prediction_relative_time) {
    prediction_relative_time_ = prediction_relative_time;
  }

 private:
  // this is a singleton class
  DECLARE_SINGLETON(SimulationContext);

  double planning_loop_dt_ = 0.1;
  double prediction_relative_time_ = 0;
};

}  // namespace planning

#endif