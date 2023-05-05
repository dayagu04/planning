#ifndef __ILQR_LAT_MPC_H__
#define __ILQR_LAT_MPC_H__

#include "ilqr_core.h"

namespace pnc {
namespace control {

struct LatMpcState {
  double dx_ = 0.0;
  double dy_ = 0.0;
  double dphi_ = 0.0;
  double delta_ = 0.0;
};

enum LatMpcStatus {
  MPC_SOLVE_SUCCESS = 0,
  MPC_ERROR_LARGE,
  MPC_SOLVE_FAIL,
  MPC_FAULT_SIZE,
};

struct LatMpcInput {
  // parameters
  double q_y_ = 1.0;
  double q_phi_ = 1.0;
  double q_omega_ = 1.0;
  double q_phi_WN_ = 1.0;
  double q_y_WN_ = 1.0;
  double omega_limit_ = 0.5;
  double delta_limit_ = 0.5;

  double curv_factor_ = 0.2;

  // init state
  LatMpcState init_state_;

  // reference
  std::vector<double> dx_ref_mpc_vec_;
  std::vector<double> dy_ref_mpc_vec_;
  std::vector<double> dphi_ref_mpc_vec_;
  std::vector<double> vel_lat_ctrl_mpc_vec_;
  std::vector<double> curv_ref_factor_;
};

struct LatMpcOutput {
  std::vector<double> time_vec_mpc_;
  std::vector<double> dx_vec_mpc_;
  std::vector<double> dy_vec_mpc_;
  std::vector<double> dphi_vec_mpc_;
  std::vector<double> delta_vec_mpc_;
  std::vector<double> omega_vec_mpc_;

  int status_ = 0;
  int iteration_ = 0;
};

class latProblem {
public:
  virtual void Init() = 0;
  virtual uint8_t Update(LatMpcInput &mpc_input) = 0;
  virtual void Reset() = 0;
  virtual LatMpcOutput GetMpcOutput() = 0;
};

class LatMpcIlqr : public latProblem {
public:
  void Init();
  uint8_t Update(LatMpcInput &mpc_input);
  void Reset();
  LatMpcOutput GetMpcOutput() { return mpc_out_; }

private:
  LatMpcOutput mpc_out_;
  std::shared_ptr<ilqr_solver::iLqr> ilqr_core_ptr_;
};

} // namespace control
} // namespace pnc
#endif
