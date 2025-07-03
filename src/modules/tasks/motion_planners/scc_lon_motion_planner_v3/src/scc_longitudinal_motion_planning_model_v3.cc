#include "scc_longitudinal_motion_planning_model_v3.h"

#include "math_lib.h"
#include "scc_longitudinal_motion_planning_cost_v3.h"

using namespace pnc::mathlib;

static const double kOneSix = 1.0 / 6.0;
static const double kOneTwentyFour = 1.0 / 24.0;
namespace pnc {
namespace scc_longitudinal_planning_v3 {

ilqr_solver::State SccLongitudinalMotionPlanningModelV3::UpdateDynamicsOneStep(
    const ilqr_solver::State &x, const ilqr_solver::Control &u,
    const size_t &) const {
  const double &dt = solver_config_ptr_->model_dt;
  ilqr_solver::State x1 = x;

  const auto &s = x[POS];
  const auto &v = x[VEL];
  const auto &a = x[ACC];
  const auto &j = x[JERK];
  const auto &djerk = u[DJERK];

  const auto dt2 = dt * dt;
  const auto dt3 = dt2 * dt;
  const auto dt4 = dt3 * dt;

  x1 << (djerk * dt4) * kOneTwentyFour + (j * dt3) * kOneSix + (a * dt2) * 0.5 + v * dt + s,

      v + a * dt + (j * dt2) * 0.5 + djerk  * dt3 * kOneSix,

      a + dt * j + djerk * dt2 * 0.5,

      j + dt * djerk;

  return x1;
}

void SccLongitudinalMotionPlanningModelV3::GetDynamicsDerivatives(
    const ilqr_solver::State &, const ilqr_solver::Control &,
    ilqr_solver::FxMT &f_x, ilqr_solver::FuMT &f_u, const size_t &) const {
  const double &dt = solver_config_ptr_->model_dt;
  const auto dt2 = dt * dt;
  const auto dt3 = dt2 * dt;
  const auto dt4 = dt3 * dt;

  f_x << 1.0, dt, dt2 * 0.5, dt3 * kOneSix,

      0.0, 1.0, dt, dt2 * 0.5,

      0.0, 0.0, 1.0, dt,

      0.0, 0.0, 0.0, 1.0;

  f_u << dt4 * kOneTwentyFour,

      dt3 * kOneSix,

      dt2 * 0.5,

      dt;
}

}  // namespace scc_longitudinal_planning_v3
}  // namespace pnc
