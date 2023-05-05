#include "ilqr_lat_model.h"
#include "ilqr_lat_cost.h"
#include "math_lib.h"

using namespace std::placeholders;

State iLqrLatModel::UpdateDynamicsOneStep(const State &x, const Control &u,
                                          const size_t &step) const {
  const IlqrCostConfig &cost_config = cost_config_vec_ptr_->at(step);
  std::array<double, STATE_SIZE> k1{}, k2{}, k3{}, k4{};

  const double dt = solver_config_ptr_->model_dt;
  const double theta = x[StateId::THETA];
  const double delta = x[StateId::DELTA];
  const double delta_dot = u[ControlId::DELTA_DOT];
  k1[0] = cost_config[REF_VEL] * sin(theta);
  k1[1] = cost_config[CURV_FACTOR] * cost_config[REF_VEL] * tan(delta);
  k1[2] = delta_dot;

  const double theta1 = theta + k1[1] * dt / 2.0;
  const double delta1 = delta + k1[2] * dt / 2.0;
  k2[0] = cost_config[REF_VEL] * sin(theta1);
  k2[1] = cost_config[CURV_FACTOR] * cost_config[REF_VEL] * tan(delta1);
  k2[2] = delta_dot;

  const double theta2 = theta + k2[1] * dt / 2.0;
  const double delta2 = delta + k2[2] * dt / 2.0;
  k3[0] = cost_config[REF_VEL] * sin(theta2);
  k3[1] = cost_config[CURV_FACTOR] * cost_config[REF_VEL] * tan(delta2);
  k3[2] = delta_dot;

  const double theta3 = theta + k3[1] * dt;
  const double delta3 = delta + k3[2] * dt;
  k4[0] = cost_config[REF_VEL] * sin(theta3);
  k4[1] = cost_config[CURV_FACTOR] * cost_config[REF_VEL] * tan(delta3);
  k4[2] = delta_dot;
  State x1 = x;
  for (size_t i = 0; i < STATE_SIZE; i++) {
    x1[i] += (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) * dt / 6;
  }
  return x1;
}

void iLqrLatModel::GetDynamicsDerivatives(const State &x, const Control & /*u*/,
                                          FxMT &f_x, FuMT &f_u,
                                          const size_t &step) const {
  const IlqrCostConfig &cost_config = cost_config_vec_ptr_->at(step);
  const double dt = solver_config_ptr_->model_dt;

  const double theta = x[StateId::THETA];
  const double delta = x[StateId::DELTA];

  const double v = cost_config[REF_VEL];
  const double k = cost_config[CURV_FACTOR];

  f_x << 1.0, dt * v * cos(theta), 0.0, 0.0, 1.0,
      dt * k * v * (delta * delta + 1.0), 0.0, 0.0, 1.0;
  f_u << 0.0, 0.0, dt;
}
