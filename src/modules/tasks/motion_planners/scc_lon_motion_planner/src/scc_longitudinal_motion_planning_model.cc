#include "scc_longitudinal_motion_planning_model.h"
#include "math_lib.h"
#include "scc_longitudinal_motion_planning_cost.h"

using namespace pnc::mathlib;

static const double kOneSix = 1.0 / 6.0;
namespace pnc {
namespace scc_longitudinal_planning {

ilqr_solver::State SccLongitudinalMotionPlanningModel::UpdateDynamicsOneStep(
    const ilqr_solver::State &x, const ilqr_solver::Control &u, const size_t &) const {
  const double &dt = solver_config_ptr_->model_dt;
  ilqr_solver::State x1 = x;

  const auto &s = x[POS];
  const auto &v = x[VEL];
  const auto &a = x[ACC];
  const auto &j = u[JERK];

  const auto dt2 = dt * dt;
  const auto dt3 = dt2 * dt;

  x1 << (j * dt3) * kOneSix + (a * dt2) * 0.5 + v * dt + s,

      v + dt * (a + (dt * j) * 0.5),

      a + dt * j;

  return x1;
}

void SccLongitudinalMotionPlanningModel::GetDynamicsDerivatives(
    const ilqr_solver::State &, const ilqr_solver::Control &, ilqr_solver::FxMT &f_x, ilqr_solver::FuMT &f_u,
    const size_t &) const {
  const double &dt = solver_config_ptr_->model_dt;
  const auto dt2 = dt * dt;
  const auto dt3 = dt2 * dt;

  f_x << 1.0, dt, dt2 * 0.5,

      0.0, 1.0, dt,

      0.0, 0.0, 1.0;

  f_u << dt3 * kOneSix,

      dt2 * 0.5,

      dt;
}

}  // namespace scc_longitudinal_planning
}  // namespace pnc
