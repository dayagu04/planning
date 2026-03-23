#include "dynamic_model.h"

#include "../problem_solver/solver_define.h"
#include "math_lib.h"

using namespace pnc::mathlib;

static const double kOneSix = 1.0 / 6.0;
static const double kOneThree = 1.0 / 3.0;

namespace pnc {
namespace lateral_planning {

DynamicModel::DynamicModel() {}

ilqr_solver::State DynamicModel::UpdateDynamicsOneStep(
    const ilqr_solver::State& x, const ilqr_solver::Control& u,
    const size_t& step) const {
  // RK4
  // state = [x y theta delta vel acc]
  // control = [omega jerk]
  const ilqr_solver::IlqrCostConfig& cost_config =
      cost_config_vec_ptr_->at(step);

  const double dt = solver_config_ptr_->model_dt;
  const double theta = x[StateID::THETA];
  const double delta = x[StateID::DELTA];
  const double omega = u[ControlID::OMEGA];
  // const double v = x[StateID::VEL];
  // const double a = x[StateID::ACC];
  // const double j = u[ControlID::JERK];
  const double v = cost_config[REF_VEL];
  const double k = cost_config[CURV_FACTOR];

  const double dt2 = dt * dt;
  const double dtv = dt * v;
  const double theta_tmp = theta + (delta * dt * k * v) * 0.5;
  const double theta_tmp2 =
      (k * omega * v * dt2) * 0.5 + delta * k * v * dt + theta;
  const double theta_tmp3 =
      (k * omega * v * dt2) * 0.25 + (delta * k * v * dt) * 0.5 + theta;

  ilqr_solver::State x1 = x;
  x1 << x[StateID::X] + (dtv * cos(theta)) * kOneSix +
                        (dtv * cos(theta_tmp)) * kOneThree +
                        (dtv * cos(theta_tmp2)) * kOneSix +
                        (dtv * cos(theta_tmp3)) * kOneThree,

        x[StateID::Y] + (dtv * sin(theta)) * kOneSix +
                        (dtv * sin(theta_tmp)) * kOneThree +
                        (dtv * sin(theta_tmp2)) * kOneSix +
                        (dtv * sin(theta_tmp3)) * kOneThree,

        theta_tmp2,

        delta + dt * omega;

  return x1;
}

void DynamicModel::GetDynamicsDerivatives(
    const ilqr_solver::State& x, const ilqr_solver::Control& u,
    ilqr_solver::FxMT& f_x, ilqr_solver::FuMT& f_u, const size_t& step) const {
  const ilqr_solver::IlqrCostConfig& cost_config = cost_config_vec_ptr_->at(step);
  const double& dt = solver_config_ptr_->model_dt;
  const double& theta = x[StateID::THETA];
  // const double& delta = x[StateID::DELTA];
  // const double& omega = u[ControlID::OMEGA];
  const double v = cost_config[REF_VEL];
  const double k = cost_config[CURV_FACTOR];
  const double kv2 = k * v * v;
  const double sin_theta = std::sin(theta);
  const double cos_theta = std::cos(theta);
  const double dt2 = dt * dt;
  // A Matrix
  //   x_1/d_x      x_1/d_y      x_1/d_theta      x_1/d_delta
  //   y_1/d_x      y_1/d_y      y_1/d_theta      y_1/d_delta
  // theta_1/d_x  theta_1/d_y  theta_1/d_theta  theta_1/d_delta
  // delta_1/d_x  delta_1/d_y  delta_1/d_theta  delta_1/d_delta
  f_x << 1.0, 0.0, -dt * v * sin_theta, -(dt2 * kv2 * sin_theta) * 0.5,

         0.0, 1.0, dt * v * cos_theta, (dt2 * kv2 * cos_theta) * 0.5,

         0.0, 0.0, 1.0, dt * k * v,

         0.0, 0.0, 0.0, 1.0;
  // B Matrix
  // x_1/d_omega  y_1/d_omega  theta_1/d_omega  delta_1/d_omega
  f_u << 0.0, 0.0, 0.0, dt;
}

}  // namespace lateral_planning
}  // namespace pnc