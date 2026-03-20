#include "vehicle_model_simulation.h"
namespace pnc {

namespace steerModel {

void VehicleSimulation::CalStateDot(Eigen::VectorXd &state_dot,
                                    Eigen::VectorXd &state) {
  state_dot[0] = control_.v_ * cos(state[2] + param_.phi_bias_ / 57.3);
  state_dot[1] = control_.v_ * sin(state[2] + param_.phi_bias_ / 57.3);
  state_dot[2] = param_.c1_ / (1 + param_.c2_ * control_.v_ * control_.v_) *
                 control_.v_ * tan(control_.delta_ + param_.bias_);
}

void VehicleSimulation::Rk4update(Eigen::VectorXd &state_dot,
                                  Eigen::VectorXd &state) {
  Eigen::VectorXd K1 = state_dot;
  Eigen::VectorXd K2 = state_dot;
  Eigen::VectorXd K3 = state_dot;
  Eigen::VectorXd K4 = state_dot;

  double dt = dt_;
  CalStateDot(K1, state);

  Eigen::VectorXd X12 = state + K1 * dt / 2.0;
  CalStateDot(K2, X12);

  Eigen::VectorXd X23 = state + K2 * dt / 2.0;
  CalStateDot(K3, X23);

  Eigen::VectorXd X34 = state + K3 * dt;
  CalStateDot(K4, X34);

  state_dot = (K1 + 2.0 * K2 + 2.0 * K3 + K4) / 6.0;
  state += state_dot * dt;
}

void VehicleSimulation::Update(const VehicleControl &control,
                               const VehicleParameter &param) {
  Eigen::VectorXd state(3);
  state << state_.x_, state_.y_, state_.phi_;
  Eigen::VectorXd state_dot = state;
  control_ = control;
  param_ = param;

  Rk4update(state_dot, state);

  state_.x_ = state[0];
  state_.y_ = state[1];
  state_.phi_ = state[2];
}

void VehicleSimulation::Init(const VehicleState &state) { state_ = state; }

}  // namespace steerModel

}  // namespace pnc