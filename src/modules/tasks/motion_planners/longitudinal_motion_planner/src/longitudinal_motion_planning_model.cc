#include "longitudinal_motion_planning_model.h"
#include "longitudinal_motion_planning_cost.h"
#include "math_lib.h"

using namespace pnc::mathlib;

static const double kOneSix = 1.0 / 6.0;
static const double kOneTwentyFour = 1.0 / 24.0;
namespace pnc {
namespace longitudinal_planning {

State LongitudinalMotionPlanningModel::UpdateDynamicsOneStep(
    const State &x, const Control &u, const size_t &) const {
  const double &dt = solver_config_ptr_->model_dt;
  State x1 = x;

  x1 << x[POS] +
            dt * (x[VEL] + 0.5 * x[ACC] * dt + kOneSix * x[JERK] * dt * dt +
                  kOneTwentyFour * u[SNAP] * dt * dt * dt),
      x[VEL] + dt * (x[ACC] + 0.5 * x[JERK] * dt + kOneSix * u[SNAP] * dt * dt),
      x[ACC] + dt * (x[JERK] + 0.5 * u[SNAP] * dt), x[JERK] + dt * u[SNAP];
  return x1;
}

void LongitudinalMotionPlanningModel::GetDynamicsDerivatives(
    const State &, const Control &, FxMT &f_x, FuMT &f_u,
    const size_t &) const {
  const double dt = solver_config_ptr_->model_dt;

  f_x << 1.0, dt, dt * dt * 0.5, 0.25 * dt * dt * dt, 0.0, 1.0, dt,
      0.5 * dt * dt, 0.0, 0.0, 1.0, dt, 0.0, 0.0, 0.0, 1.0;
  f_u << 0.125 * dt * dt * dt * dt, 0.25 * dt * dt * dt, 0.5 * dt * dt, dt;
}

} // namespace longitudinal_planning
} // namespace pnc
