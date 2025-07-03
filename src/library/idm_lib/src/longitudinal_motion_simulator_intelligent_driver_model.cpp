#include "../include/longitudinal_motion_simulator_intelligent_driver_model.h"

#include <cmath>
#include <cstddef>
#include <iostream>

#include "src/common/log.h"
#include "src/library/idm_lib/include/basic_intelligent_driver_model.h"

namespace planning {
namespace simulator {

LonMotionSimulatorIntelligentDriverModel::
    LonMotionSimulatorIntelligentDriverModel() {
  already_set_state_ = false;
  already_set_param_ = false;
  UpdateInternalState();
}

LonMotionSimulatorIntelligentDriverModel::
    LonMotionSimulatorIntelligentDriverModel(const ModelParam &param)
    : param_(param) {
  already_set_state_ = false;
  already_set_param_ = true;
  UpdateInternalState();
}

void LonMotionSimulatorIntelligentDriverModel::Step_Forward(const double dt) {
  // boost::numeric::odeint::integrate(boost::ref(*this), internal_state_, 0.0,
  // dt,
  //                                   dt);
  if (state_.vel < 1e-6) {
    return;
  }
  RK4Integrate(internal_state_, dt, &internal_state_);

  // Linear(internal_state_, dt, &internal_state_);

  // printf("[Internal]%lf, %lf, %lf, %lf\n", internal_state_[0],
  //        internal_state_[1], internal_state_[2], internal_state_[3]);

  state_.s = internal_state_[0];
  state_.vel = internal_state_[1];
  state_.s_front = internal_state_[2];
  state_.vel_front = internal_state_[3];
  UpdateInternalState();
}

ErrorType LonMotionSimulatorIntelligentDriverModel::Simulate(const double t) {
  if (!already_set_state_) {
    LOG_ERROR("idm simulates failed, init state not set");
    return ErrorType::kWrongStatus;
  }
  if (t < dt_resolution_) {
    LOG_ERROR("idm simulates failed, t %f is less than kStepForwarddt %f \n", t,
              dt_resolution_);
    return ErrorType::kIllegalInput;
  }
  const size_t step_nums = std::round(t / dt_resolution_);
  Reset_Simulation_Result(step_nums);
  simulation_result_.s_vec.resize(step_nums + 1);
  std::vector<double> s_vec, vel_vec, acc_vec, t_vec;
  vel_vec.resize(step_nums + 1, 0.0);
  acc_vec.resize(step_nums + 1, 0.0);
  s_vec.resize(step_nums + 1, 0.0);
  t_vec.resize(step_nums + 1, 0.0);
  bool stand_still = false;
  for (size_t i = 0; i < step_nums + 1; ++i) {
    if (i == 0) {
      t_vec[i] = 0.0;
      s_vec[i] = state_.s;
      vel_vec[i] = state_.vel;
      double init_state_acc = 0.0;
      GetAccDesiredAcceleration(param_, state_, &init_state_acc);
      acc_vec[i] = init_state_acc;
    } else {
      Step_Forward(dt_resolution_);
      t_vec[i] = t_vec[i - 1] + dt_resolution_;
      stand_still = state_.vel < 1e-6;
      vel_vec[i] = stand_still ? 0.0 : state_.vel;
      acc_vec[i] = stand_still ? 0.0 : acc_;
      s_vec[i] = stand_still ? s_vec[i - 1] : state_.s;
    }
  }
  simulation_result_.t_s_spline.set_points(t_vec, s_vec);
  simulation_result_.t_v_spline.set_points(t_vec, vel_vec);
  simulation_result_.t_a_spline.set_points(t_vec, acc_vec);
  simulation_result_.s_vec.swap(s_vec);
  simulation_result_.vel_vec.swap(vel_vec);
  simulation_result_.acc_vec.swap(acc_vec);
  simulation_result_.t_vec.swap(t_vec);

  return ErrorType::kSuccess;
}

ErrorType LonMotionSimulatorIntelligentDriverModel::Simulate(
    const ModelState &init_state, const double t) {
  set_model_state(init_state);
  const auto status = Simulate(t);
  if (status == ErrorType::kIllegalInput) {
    LOG_ERROR("idm simulates failed, t %f is less than kStepForwarddt %f \n", t,
              dt_resolution_);
    return ErrorType::kIllegalInput;
  } else if (status == ErrorType::kWrongStatus) {
    LOG_ERROR("idm simulates failed, init state not set");
    return ErrorType::kWrongStatus;
  }
  return ErrorType::kSuccess;
}

void LonMotionSimulatorIntelligentDriverModel::Reset_Simulation_Result(
    size_t n) {
  simulation_result_.s_vec.resize(n, 0);
  simulation_result_.vel_vec.resize(n, 0);
  simulation_result_.acc_vec.resize(n, 0);
  simulation_result_.t_vec.resize(n, 0);
}

void LonMotionSimulatorIntelligentDriverModel::Linear(const InternalState &x,
                                                      const double dt,
                                                      InternalState *x_out) {
  ModelState cur_state;
  cur_state.s = x[0];
  cur_state.vel = x[1];
  cur_state.s_front = x[2];
  cur_state.vel_front = x[3];

  double acc = 0.0;
  GetIIdmDesiredAcceleration(param_, cur_state, &acc);

  double average_acc = dt == 0.0 ? 0.0 : cur_state.vel / dt;
  acc = std::max(
      acc, -std::min(param_.kHardBrakingDeceleration, cur_state.vel / dt));

  (*x_out)[0] = x[0] + cur_state.vel * dt + 0.5 * acc * dt * dt;
  (*x_out)[1] = cur_state.vel + acc * dt;
  (*x_out)[2] = x[2] + x[3] * dt;
  (*x_out)[3] = x[3];
}

void LonMotionSimulatorIntelligentDriverModel::operator()(
    const InternalState &x, InternalState &dxdt, const double dt) {
  // x: current state [ego s, ego vel, cipv s, cipv vel]
  // dxdt: state dot

  // 1. get current state
  ModelState cur_state;
  cur_state.s = x[0];
  cur_state.vel = x[1];
  cur_state.s_front = x[2];
  cur_state.vel_front = x[3];

  // 2. get acc by iidm
  double acc = 0.0;
  GetAccDesiredAcceleration(param_, cur_state, &acc);
  double average_acc = dt == 0.0 ? 0.0 : cur_state.vel / dt;
  acc = std::max(
      acc, -std::min(param_.kHardBrakingDeceleration, cur_state.vel / dt));

  // 3. get state dot
  dxdt[0] = cur_state.vel;
  dxdt[1] = acc;
  dxdt[2] = cur_state.vel_front;
  dxdt[3] = 0.0;  // assume other vehicle keep the current velocity
}

void LonMotionSimulatorIntelligentDriverModel::RK4Integrate(
    const InternalState &x, const double dt, InternalState *x_out) {
  // RK4 integration
  InternalState X = x;
  InternalState K1, K2, K3, K4, X_tmp;

  // k1 = f(x, t)
  (*this)(X, K1, 0.0);

  // k2 = f(x + dt/2 * k1, t + dt/2)
  for (size_t i = 0; i < x.size(); ++i) {
    X_tmp[i] = X[i] + dt / 2.0 * K1[i];
  }
  (*this)(X_tmp, K2, dt / 2.0);

  // k3 = f(x + dt/2 * k2, t + dt/2)
  for (size_t i = 0; i < x.size(); ++i) {
    X_tmp[i] = X[i] + dt / 2.0 * K2[i];
  }
  (*this)(X_tmp, K3, dt / 2.0);

  // k4 = f(x + dt * k3, t + dt)
  for (size_t i = 0; i < x.size(); ++i) {
    X_tmp[i] = X[i] + dt * K3[i];
  }
  (*this)(X_tmp, K4, dt);

  // get acc at final dt
  acc_ = K4[1];

  // x_{n+1} = x_n + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
  for (size_t i = 0; i < x.size(); ++i) {
    (*x_out)[i] = X[i] + dt / 6.0 * (K1[i] + 2.0 * K2[i] + 2.0 * K3[i] + K4[i]);
  }
}

void LonMotionSimulatorIntelligentDriverModel::set_model_state(
    const ModelState &state) {
  state_ = state;
  already_set_state_ = true;
  UpdateInternalState();
}
void LonMotionSimulatorIntelligentDriverModel::set_model_param(
    const ModelParam &param) {
  param_ = param;
  already_set_param_ = true;
}

void LonMotionSimulatorIntelligentDriverModel::UpdateInternalState() {
  internal_state_[0] = state_.s;
  internal_state_[1] = state_.vel;
  internal_state_[2] = state_.s_front;
  internal_state_[3] = state_.vel_front;
}

}  // namespace simulator
}  // namespace planning