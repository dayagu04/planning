/***************************************************************
 * @file       vehicle_dynamics.h
 * @brief      for vehicle_dynamics
 * @author     xiaoliang.wang
 * @version    v0.0
 * @date       Sep-17-2021
 **************************************************************/

#ifndef __VEHICLE_DYNAMICS_H__
#define __VEHICLE_DYNAMICS_H__

#include <vector>
#include "math_lib.h"
#include "statespace_sys.h"

namespace pnc {
namespace vehicle_dynamics {

struct SimpleKinematicsState {
  double X_;
  double Y_;
  double v_;
  double a_;
  double phi_;
  double delta_;
  double beta_;
  double omega_;

  void Reset() {
    X_ = 0.0;
    Y_ = 0.0;
    v_ = 0.0;
    a_ = 0.0;
    phi_ = 0.0;
    delta_ = 0.0;
    beta_ = delta_ * 0.5;
    omega_ = 0.0;
  }

  SimpleKinematicsState() { Reset(); };

  SimpleKinematicsState(double X, double Y, double v, double a, double phi, double delta)
      : X_(X), Y_(Y), v_(v), a_(a), phi_(phi), delta_(delta) {}

  void SetValue(double X, double Y, double v, double a, double phi, double delta) {
    X_ = X;
    Y_ = Y;
    v_ = v;
    a_ = a;
    phi_ = phi;
    delta_ = delta;
    beta_ = delta_ * 0.5;
  }
};

struct SimpleKinematicsInput {
  double acc_cmd_;
  double delta_cmd_;
  SimpleKinematicsInput() {
    acc_cmd_ = 0.0;
    delta_cmd_ = 0.0;
  }
  SimpleKinematicsInput(double acc_cmd, double delta_cmd) : acc_cmd_(acc_cmd), delta_cmd_(delta_cmd) {}
  void SetValue(double acc_cmd, double delta_cmd) {
    acc_cmd_ = acc_cmd;
    delta_cmd_ = delta_cmd;
  }
};

struct SimpleKinematicsParameters {
  double L_;
  double tau_acc_;
  double k_acc_;
  double tau_delta_;
  double curv_factor_;
  double fs_;
  double delta_bias_;
  double vel_max_;
  double acc_inc_max_;
  double acc_dec_max_;
  double delta_max_;

  SimpleKinematicsParameters() {
    L_ = 2.85;
    tau_delta_ = 0.1;
    curv_factor_ = 1.0;
    tau_acc_ = 0.5;
    k_acc_ = 0.8;
    fs_ = 100.0;
    delta_bias_ = 0.0;
    vel_max_ = 33.0;
    acc_inc_max_ = 4.0;
    acc_dec_max_ = -6.0;
    delta_max_ = mathlib::Deg2Rad(120.0);
  }

  SimpleKinematicsParameters(double L, double tau_acc, double k_acc, double tau_delta, double curv_factor,
                             double delta_bias, double vel_max, double acc_inc_max, double acc_dec_max,
                             double delta_max)
      : L_(L),
        tau_acc_(tau_acc),
        k_acc_(k_acc),
        tau_delta_(tau_delta),
        curv_factor_(curv_factor),
        delta_bias_(delta_bias),
        vel_max_(vel_max),
        acc_inc_max_(acc_inc_max),
        acc_dec_max_(acc_dec_max),
        delta_max_(delta_max) {}
};

class SimpleKinematicsModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SimpleKinematicsModel() { InitDynamicSys(); }

  double GetModelFreq(void) { return param_.fs_; };
  void SetParam(const SimpleKinematicsParameters &param);
  void Update(const SimpleKinematicsInput &input);
  void GetState(SimpleKinematicsState &state);
  void SetInitStateInput(const SimpleKinematicsState &state, const SimpleKinematicsInput &input);
  void SetDeltaBias(double bias);

 private:
  void InitDynamicSys();
  void CalStateDot(Eigen::VectorXd &state_dot, Eigen::VectorXd &state);
  void Rk4update(Eigen::VectorXd &state_dot, Eigen::VectorXd &state);
  void SetInitState(const SimpleKinematicsState &state);
  void SetInitInput(const SimpleKinematicsInput &input);
  double get_max_inc_acc(double v);

  SimpleKinematicsState state_;
  SimpleKinematicsInput input_;
  SimpleKinematicsParameters param_;
  statespace_sys::StatespaceSISOSys1st Gacc_;
  statespace_sys::StatespaceSISOSys1st Gv_;
  statespace_sys::StatespaceSISOSys1st Gdelta_;
};

}  // namespace vehicle_dynamics
}  // namespace pnc

#endif
