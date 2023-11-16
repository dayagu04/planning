#include "realtime_lateral_motion_planning_model.h"
#include "math_lib.h"
#include "realtime_lateral_motion_planning_cost.h"

using namespace pnc::mathlib;

static const double kOneSix = 1.0 / 6.0;
static const double kOneThree = 1.0 / 3.0;

namespace pnc {
namespace realtime_lateral_planning {

State RealtimeLateralMotionPlanningModel::UpdateDynamicsOneStep(
    const State &x, const Control &u, const size_t &step) const {
  // state = [x y theta delta]
  // control = [omega]

  const IlqrCostConfig &cost_config = cost_config_vec_ptr_->at(step);

  const double &dt = solver_config_ptr_->model_dt;
  const double &theta = x[StateId::THETA];
  const double &delta = x[StateId::DELTA];
  const double &omega = u[ControlId::OMEGA];

  const double &v = cost_config[REF_VEL];
  const double &k = cost_config[CURV_FACTOR];

  const double dt2 = dt * dt;
  const double dtv = dt * v;

  const double theta_tmp = theta + (delta * dt * k * v) * 0.5;
  const double theta_tmp2 =
      (k * omega * v * dt2) * 0.5 + delta * k * v * dt + theta;
  const double theta_tmp3 =
      (k * omega * v * dt2) * 0.25 + (delta * k * v * dt) * 0.5 + theta;

  State x1 = x;
  x1 << x[StateId::X] + (dtv * cos(theta)) * kOneSix +
            (dtv * cos(theta_tmp)) * kOneThree +
            (dtv * cos(theta_tmp2)) * kOneSix +
            (dtv * cos(theta_tmp3)) * kOneThree,

      x[StateId::Y] + (dtv * sin(theta)) * kOneSix +
          (dtv * sin(theta_tmp)) * kOneThree +
          (dtv * sin(theta_tmp2)) * kOneSix +
          (dtv * sin(theta_tmp3)) * kOneThree,

      theta_tmp2,

      delta + dt * omega;

  return x1;
}

void RealtimeLateralMotionPlanningModel::GetDynamicsDerivatives(
    const State &x, const Control &u, FxMT &f_x, FuMT &f_u,
    const size_t &step) const {
  const IlqrCostConfig &cost_config = cost_config_vec_ptr_->at(step);
  const double &dt = solver_config_ptr_->model_dt;
  const double &theta = x[StateId::THETA];

  const double &v = cost_config[REF_VEL];
  const double &k = cost_config[CURV_FACTOR];

  const double kv2 = k * v * v;
  const double sin_theta = std::sin(theta);
  const double cos_theta = std::cos(theta);
  const double dt2 = dt * dt;

  f_x << 1.0, 0.0, -dt * v * sin_theta, -(dt2 * kv2 * sin_theta) * 0.5,

      0.0, 1.0, dt * v * cos_theta, (dt2 * kv2 * cos_theta) * 0.5,

      0.0, 0.0, 1.0, dt * k * v,

      0.0, 0.0, 0.0, 1.0;

  f_u << 0.0, 0.0, 0.0, dt;
}

}  // namespace realtime_lateral_planning
}  // namespace pnc
