#include "lateral_motion_planning_model.h"
#include "lateral_motion_planning_cost.h"
#include "math_lib.h"

using namespace pnc::mathlib;

static const double kOneSix = 1.0 / 6.0;

namespace pnc {
namespace lateral_planning {

State LateralMotionPlanningModel::UpdateDynamicsOneStep(const State &x, const Control &u, const size_t &step) const {
  const IlqrCostConfig &cost_config = cost_config_vec_ptr_->at(step);
  std::array<double, STATE_SIZE> k1{}, k2{}, k3{}, k4{};

  const double dt = solver_config_ptr_->model_dt;
  const double theta = x[StateId::THETA];
  const double delta = x[StateId::DELTA];
  const double omega = x[StateId::OMEGA];
  const double omega_dot = u[ControlId::OMEGA_DOT];

  const double v = cost_config[REF_VEL];
  const double k = cost_config[CURV_FACTOR];

  // state = [ x y theta delta omega]

  k1[0] = v * cos(theta);
  k1[1] = v * sin(theta);
  k1[2] = k * v * FastTan(delta);
  k1[3] = omega;
  k1[4] = omega_dot;

  const double theta1 = theta + k1[2] * dt * 0.5;
  const double delta1 = delta + k1[3] * dt * 0.5;
  const double omega1 = omega + k1[4] * dt * 0.5;
  k2[0] = v * cos(theta1);
  k2[1] = v * sin(theta1);
  k2[2] = k * v * FastTan(delta1);
  k2[3] = omega1;
  k2[4] = omega_dot;

  const double theta2 = theta + k2[2] * dt * 0.5;
  const double delta2 = delta + k2[3] * dt * 0.5;
  const double omega2 = omega + k2[4] * dt * 0.5;
  k3[0] = v * cos(theta2);
  k3[1] = v * sin(theta2);
  k3[2] = k * v * FastTan(delta2);
  k3[3] = omega2;
  k4[4] = omega_dot;

  const double theta3 = theta + k3[2] * dt;
  const double delta3 = delta + k3[3] * dt;
  const double omega3 = omega + k3[4] * dt;
  k4[0] = v * cos(theta3);
  k4[1] = v * sin(theta3);
  k4[2] = k * v * FastTan(delta3);
  k4[3] = omega3;
  k4[4] = omega_dot;

  State x1 = x;
  for (size_t i = 0; i < STATE_SIZE; i++) {
    x1[i] += (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) * dt * kOneSix;
  }
  return x1;
}

void LateralMotionPlanningModel::GetDynamicsDerivatives(const State &x, const Control & /*u*/, FxMT &f_x, FuMT &f_u,
                                                        const size_t &step) const {
  const IlqrCostConfig &cost_config = cost_config_vec_ptr_->at(step);
  const double dt = solver_config_ptr_->model_dt;
  const double theta = x[StateId::THETA];
  const double delta = x[StateId::DELTA];

  const double v = cost_config[REF_VEL];
  const double k = cost_config[CURV_FACTOR];

  f_x << 1.0, 0.0, -dt * v * sin(theta), 0.0, 0.0, 0.0, 1.0, dt * v * cos(theta), 0.0, 0.0, 0.0, 0.0, 1.0,
      dt * k * v * (delta * delta + 1.0), 0.0, 0.0, 0.0, 0.0, 1.0, dt, 0.0, 0.0, 0.0, 0.0, 1.0;
  f_u << 0.0, 0.0, 0.0, 0.0, dt;
}
}  // namespace lateral_planning
}  // namespace pnc
