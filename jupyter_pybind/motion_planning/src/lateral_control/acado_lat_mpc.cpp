#include "acado_lat_mpc.h"
#include "acado_qpoases_interface.hpp"
#include "math_lib.h"
#include <stdio.h>

namespace pnc {
namespace control {

// #define MPC_DEBUG

constexpr auto NX = ACADO_NX;   /* Number of differential state variables.  */
constexpr auto NU = ACADO_NU;   /* Number of control inputs. */
constexpr auto NOD = ACADO_NOD; /* Number of online data values. */
constexpr auto NY =
    ACADO_NY; /* Number of measurements/references on nodes 0..N - 1. */
constexpr auto NYN =
    ACADO_NYN;              /* Number of measurements/references on node N. */
constexpr auto N = ACADO_N; /* Number of intervals in the horizon. */

void LatMpc::Init() {
  auto out = acado_initializeSolver();
  UNUSED(out);
  int i;

  /* Initialize the states and controls. */
  for (i = 0; i < NX * (N + 1); ++i)
    acadoVariables.x[i] = 0.0;
  for (i = 0; i < NU * N; ++i)
    acadoVariables.u[i] = 0.0;

  /* Initialize the measurements/reference. */
  for (i = 0; i < NY * N; ++i)
    acadoVariables.y[i] = 0.0;
  for (i = 0; i < NYN; ++i)
    acadoVariables.yN[i] = 0.0;

  /* MPC: initialize the current state feedback. */
  for (i = 0; i < NX; ++i)
    acadoVariables.x0[i] = 0.0;
}

uint8_t LatMpc::Update(LatMpcInput &mpc_input) {
  uint8_t ret = MPC_SOLVE_FAIL;
  ret = UpdateOnce(mpc_input);

  if (ret == MPC_SOLVE_FAIL || ret == MPC_FAULT_SIZE) {
    Init();
    ret = UpdateOnce(mpc_input);
  }

  return ret;
}

uint8_t LatMpc::UpdateOnce(LatMpcInput &mpc_input) {
  if (mpc_input.dy_ref_mpc_vec_.size() != N + 1 ||
      mpc_input.dphi_ref_mpc_vec_.size() != N + 1 ||
      mpc_input.vel_lat_ctrl_mpc_vec_.size() != N + 1) {
    mpc_out_.status_ = MPC_FAULT_SIZE;
    return MPC_FAULT_SIZE;
  }

  // set parameters and reference by online data
  for (int i = 0; i <= NOD * N; i += NOD) {
    if (mpc_input.curv_ref_factor_.empty()) {
      acadoVariables.od[i] = mpc_input.curv_factor_;
    } else {
      acadoVariables.od[i] = mpc_input.curv_ref_factor_[i / NOD];
    }
    acadoVariables.od[i + 1] = mpc_input.vel_lat_ctrl_mpc_vec_[i / NOD];
    acadoVariables.od[i + 2] = mpc_input.dy_ref_mpc_vec_[i / NOD];
    acadoVariables.od[i + 3] = mpc_input.dphi_ref_mpc_vec_[i / NOD];
    acadoVariables.od[i + 4] = mpc_input.omega_limit_;
    acadoVariables.od[i + 5] = mpc_input.delta_limit_;
  }

  // set weights
  // cost for dy, phi, omega
  for (int i = 0; i < N; ++i) {
    acadoVariables.W[NY * NY * i + 0 * (NY + 1)] = mpc_input.q_y_;
    acadoVariables.W[NY * NY * i + 1 * (NY + 1)] = mpc_input.q_phi_;
    acadoVariables.W[NY * NY * i + 2 * (NY + 1)] = mpc_input.q_omega_;
  }

  // terminal cost for dy, phi
  acadoVariables.WN[0 * NYN] = mpc_input.q_y_WN_;
  acadoVariables.WN[1 * NYN + 1] = mpc_input.q_phi_WN_;

  // set init state
  acadoVariables.x0[0] = mpc_input.init_state_.dx_;
  acadoVariables.x0[1] = mpc_input.init_state_.dy_;
  acadoVariables.x0[2] = mpc_input.init_state_.dphi_;
  acadoVariables.x0[3] = mpc_input.init_state_.delta_;

  // solve the problem
  int retval_pre = acado_preparationStep();
  int retval_fb = acado_feedbackStep();

  if ((!retval_pre) && (!retval_fb)) {
#ifdef MPC_DEBUG
    acado_printDifferentialVariables();
    acado_printControlVariables();
#endif
  } else {
#ifdef MPC_DEBUG
  // printf("retval_pre = %d: %s\n", retval_pre,
  //        acado_getErrorString(retval_pre));
  // printf("retval_fb = %d: %s\n", retval_fb,
  // acado_getErrorString(retval_fb));
#endif
    auto out = acado_initializeSolver();
    UNUSED(out);
    mpc_out_.status_ = MPC_SOLVE_FAIL;
    return MPC_SOLVE_FAIL;
  }

  // get solution
  ClearMpcOutput();

  // get state： size = N + 1
  for (int i = 0; i < N + 1; ++i) {
    mpc_out_.time_vec_mpc_.push_back(static_cast<double>(i) * 0.1);
    mpc_out_.dx_vec_mpc_.push_back(acadoVariables.x[i * NX]);
    mpc_out_.dy_vec_mpc_.push_back(acadoVariables.x[i * NX + 1]);
    mpc_out_.dphi_vec_mpc_.push_back(acadoVariables.x[i * NX + 2]);
    mpc_out_.delta_vec_mpc_.push_back(acadoVariables.x[i * NX + 3]);
  }

  // get input: size = N
  for (int i = 0; i < N; ++i) {
    mpc_out_.omega_vec_mpc_.push_back(acadoVariables.u[i * NU]);
  }
  mpc_out_.omega_vec_mpc_.push_back(mpc_out_.omega_vec_mpc_.back());

  acado_shiftStates(2, 0, 0);
  acado_shiftControls(0);

  mpc_out_.iteration_ = acado_getNWSR();
  mpc_out_.status_ = MPC_SOLVE_SUCCESS;

  return MPC_SOLVE_SUCCESS;
}

void LatMpc::ClearMpcOutput() {
  mpc_out_.time_vec_mpc_.reserve(N + 1);
  mpc_out_.time_vec_mpc_.clear();

  // state
  mpc_out_.dx_vec_mpc_.reserve(N + 1);
  mpc_out_.dx_vec_mpc_.clear();

  mpc_out_.dy_vec_mpc_.reserve(N + 1);
  mpc_out_.dy_vec_mpc_.clear();

  mpc_out_.dphi_vec_mpc_.reserve(N + 1);
  mpc_out_.dphi_vec_mpc_.clear();

  mpc_out_.delta_vec_mpc_.reserve(N + 1);
  mpc_out_.delta_vec_mpc_.clear();

  // input
  mpc_out_.omega_vec_mpc_.reserve(N + 1);
  mpc_out_.omega_vec_mpc_.clear();
}

void LatMpc::Reset() {
  Init();
  init_flag_ = false;
}

} // namespace control
} // namespace pnc
