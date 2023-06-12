#ifndef __VEHICLE_MODEL_SIMULATION_H__
#define __VEHICLE_MODEL_SIMULATION_H__

#include <iostream>
#include "vehicle_dynamics.h"

namespace pnc {

namespace steerModel {

struct VehicleState {
  double x_ = 0.0;
  double y_ = 0.0;
  double phi_ = 0.0;
};

struct VehicleControl {
  double v_ = 1.0;
  double delta_ = 0.0;
};

struct VehicleParameter {
  double c1_ = 0.325;
  double c2_ = 0.002;
  double bias_ = 0.0;
  double phi_bias_ = 0.0;
};

class VehicleSimulation {
 public:
  void Init(const VehicleState &state);
  void Update(const VehicleControl &control, const VehicleParameter &param);
  VehicleState GetState() { return state_; }

 private:
  void CalStateDot(Eigen::VectorXd &state_dot, Eigen::VectorXd &state);
  void Rk4update(Eigen::VectorXd &state_dot, Eigen::VectorXd &state);

  VehicleState state_;
  VehicleControl control_;
  VehicleParameter param_;
};

}  // namespace steerModel

}  // namespace pnc

#endif