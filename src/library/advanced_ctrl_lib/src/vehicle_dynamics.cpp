/***************************************************************
 * @file       vehicle_dynamics.cpp
 * @brief      for vehicle_dynamics
 * @author     xiaoliang.wang
 * @version    v0.0.0
 * @date       Sep-17-2021
 **************************************************************/

#include "vehicle_dynamics.h"
#include "math_lib.h"
namespace pnc {
namespace vehicle_dynamics {

void SimpleKinematicsModel::SetInitState(const SimpleKinematicsState &state) {
  state_ = state;
  Gacc_.SwitchBuf(state_.a_, state_.a_);
  Gv_.SwitchBuf(state_.a_, state_.v_);
  Gdelta_.SwitchBuf(state_.delta_, state_.delta_);
}

void SimpleKinematicsModel::SetInitInput(const SimpleKinematicsInput &input) {
  input_ = input;
  Gacc_.SwitchBuf(input_.acc_cmd_, state_.a_);
  Gv_.SwitchBuf(state_.a_, state_.v_);
  Gdelta_.SwitchBuf(input_.delta_cmd_, state_.delta_);
}

void SimpleKinematicsModel::SetInitStateInput(const SimpleKinematicsState &state, const SimpleKinematicsInput &input) {
  state_ = state;
  input_ = input;
  Gacc_.SwitchBuf(input_.acc_cmd_, state_.a_);
  Gv_.SwitchBuf(state_.a_, state_.v_);
  Gdelta_.SwitchBuf(input_.delta_cmd_, state_.delta_);
}

void SimpleKinematicsModel::SetParam(const SimpleKinematicsParameters &param) {
  param_ = param;
  InitDynamicSys();
}

void SimpleKinematicsModel::GetState(SimpleKinematicsState &state) { state = state_; }

void SimpleKinematicsModel::InitDynamicSys() {
  double num_Gacc[] = {param_.tau_acc_, 1.0};
  double den_Gacc[] = {0.0, param_.k_acc_};
  Gacc_.InitTFcontinuous(num_Gacc, den_Gacc, param_.fs_);
  Gacc_.SwitchBuf(state_.a_, state_.a_);

  double num_Gv[] = {1.0, 0.0};
  double den_Gv[] = {0.0, 1.0};
  Gv_.InitTFcontinuous(num_Gv, den_Gv, param_.fs_);
  Gv_.SwitchBuf(state_.a_, state_.v_);

  double num_Gdelta[] = {param_.tau_delta_, 1.0};
  double den_Gdelta[] = {0.0, param_.curv_factor_};
  Gdelta_.InitTFcontinuous(num_Gdelta, den_Gdelta, param_.fs_);
  Gdelta_.SwitchBuf(state_.delta_, state_.delta_);
}

void SimpleKinematicsModel::CalStateDot(Eigen::VectorXd &state_dot, Eigen::VectorXd &state) {
  state_dot[0] = state_.v_ * cos(state[2] + state_.beta_);
  state_dot[1] = state_.v_ * sin(state[2] + state_.beta_);
  state_dot[2] = state_.v_ / param_.L_ * tan(state_.delta_);
}

void SimpleKinematicsModel::Rk4update(Eigen::VectorXd &state_dot, Eigen::VectorXd &state) {
  Eigen::VectorXd K1 = state_dot;
  Eigen::VectorXd K2 = state_dot;
  Eigen::VectorXd K3 = state_dot;
  Eigen::VectorXd K4 = state_dot;

  double dt = 1.0 / param_.fs_;
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

double SimpleKinematicsModel::get_max_inc_acc(double v) {
  v = v / param_.vel_max_;
  double acc_inc_max = -8.8732 * (v * v * v * v) + 22.4502 * (v * v * v) - 18.6281 * (v * v) + 4.4017 * (v) + 0.6630;
  return acc_inc_max * (param_.acc_inc_max_);
}

void SimpleKinematicsModel::Update(const SimpleKinematicsInput &input) {
  input_ = input;

  // update vel first
  Gv_.Update(state_.a_);

  if (fabs(state_.v_) >= param_.vel_max_) {
    state_.v_ = mathlib::DoubleConstrain(state_.v_, -param_.vel_max_, param_.vel_max_);
    Gv_.SwitchBuf(state_.a_, state_.v_);
  } else {
    state_.v_ = Gv_.GetOutput();
  }

  // update acc
  double acc_inc_max = get_max_inc_acc(state_.v_);
  input_.acc_cmd_ = mathlib::DoubleConstrain(input_.acc_cmd_, param_.acc_dec_max_, acc_inc_max);
  Gacc_.Update(input_.acc_cmd_);
  if (state_.a_ > acc_inc_max) {
    state_.a_ = acc_inc_max;
    Gacc_.SwitchBuf(state_.a_, state_.a_);
  } else {
    state_.a_ = Gacc_.GetOutput();
  }

  // updat delta
  input_.delta_cmd_ = mathlib::DoubleConstrain(input_.delta_cmd_, -param_.delta_max_, param_.delta_max_);
  Gdelta_.Update(input_.delta_cmd_);
  state_.delta_ = Gdelta_.GetOutput() + param_.delta_bias_;

  // rk4 update
  Eigen::VectorXd state = Eigen::Vector3d::Zero();
  state << state_.X_, state_.Y_, state_.phi_;

  Eigen::VectorXd state_dot = state;
  Rk4update(state_dot, state);

  // update other state
  state_.X_ = state[0];
  state_.Y_ = state[1];
  state_.phi_ = state[2];
  state_.beta_ = 0.5 * state_.delta_;
  state_.omega_ = state_.v_ / param_.L_ * tan(state_.delta_);
}

void SimpleKinematicsModel::SetDeltaBias(double bias) { param_.delta_bias_ = bias; }
}  // namespace vehicle_dynamics

}  // namespace pnc
