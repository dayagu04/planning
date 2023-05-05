#include "ilqr_lat_mpc.h"
#include "ilqr_lat_cost.h"
#include "ilqr_lat_model.h"
#include "math_lib.h"

using namespace pnc::mathlib;

namespace pnc {
namespace control {

StateVec x0;
ControlVec u0;

void LatMpcIlqr::Init() {
  ilqr_core_ptr_ = std::make_shared<ilqr_solver::iLqr>();
  ilqr_core_ptr_->Init(std::make_shared<iLqrLatModel>());

  // set solver config parmeters
  ilqr_solver::iLqrSolverConfig ilqr_solver_config;
  ilqr_solver_config.horizion = 25;
  ilqr_solver_config.input_size = 1;
  ilqr_solver_config.state_size = 3;
  ilqr_solver_config.model_dt = 0.1;
  ilqr_solver_config.warm_start_enable = true;

  ilqr_core_ptr_->SetSolverConfig(ilqr_solver_config);

  // init when first set solver config
  ilqr_core_ptr_->InitSolverConfig();

  ilqr_core_ptr_->AddCost(std::make_shared<yRefCostTerm>());
  ilqr_core_ptr_->AddCost(std::make_shared<thetaRefCostTerm>());
  ilqr_core_ptr_->AddCost(std::make_shared<steeringRateBoundCostTerm>());
  ilqr_core_ptr_->AddCost(std::make_shared<steeringRateCostTerm>());
  ilqr_core_ptr_->AddCost(std::make_shared<steeringBoundCostTerm>());

  // should run after add cost
  ilqr_core_ptr_->InitAdvancedInfo();

  x0.resize(ilqr_core_ptr_->GetSolverConfigPtr()->horizion + 1);
  ResizeVectorElemFromEigenVec(
      x0, ilqr_core_ptr_->GetSolverConfigPtr()->state_size);

  u0.resize(ilqr_core_ptr_->GetSolverConfigPtr()->horizion);
  ResizeVectorElemFromEigenVec(
      u0, ilqr_core_ptr_->GetSolverConfigPtr()->input_size);
}

uint8_t LatMpcIlqr::Update(LatMpcInput &mpc_input) {

  std::vector<IlqrCostConfig> ilqr_cost_config_vec;
  IlqrCostConfig ilqr_cost_config;
  ilqr_cost_config_vec.resize(
      ilqr_core_ptr_->GetSolverConfigPtr()->horizion + 1);
  for (size_t i = 0; i < ilqr_core_ptr_->GetSolverConfigPtr()->horizion + 1;
       i++) {
    ilqr_cost_config[REF_Y] = mpc_input.dy_ref_mpc_vec_[i];
    ilqr_cost_config[REF_VEL] = mpc_input.vel_lat_ctrl_mpc_vec_[i];
    ilqr_cost_config[REF_THETA] = mpc_input.dphi_ref_mpc_vec_[i];
    if (mpc_input.curv_ref_factor_.empty()) {
      ilqr_cost_config[CURV_FACTOR] = mpc_input.curv_factor_;
    } else {
      ilqr_cost_config[CURV_FACTOR] = mpc_input.curv_ref_factor_[i];
    }
    if (i == ilqr_core_ptr_->GetSolverConfigPtr()->horizion) {
      ilqr_cost_config[W_REF_Y] = mpc_input.q_y_WN_;
      ilqr_cost_config[W_REF_THETA] = mpc_input.q_phi_WN_;
    } else {
      ilqr_cost_config[W_REF_Y] = mpc_input.q_y_;
      ilqr_cost_config[W_REF_THETA] = mpc_input.q_phi_;
    }
    ilqr_cost_config[W_LAT_INPUT] = mpc_input.q_omega_;

    ilqr_cost_config[W_WHEEL_ANGLE_RATE_BOUND] = 100.0;
    ilqr_cost_config[WHEEL_RATE_MIN] = -mpc_input.delta_limit_;
    ilqr_cost_config[WHEEL_RATE_MAX] = mpc_input.delta_limit_;

    ilqr_cost_config[W_WHEEL_ANGLE_BOUND] = 100.0;
    ilqr_cost_config[WHEEL_ANGLE_MIN] = -mpc_input.omega_limit_;
    ilqr_cost_config[WHEEL_ANGLE_MAX] = mpc_input.omega_limit_;
    ilqr_cost_config[TERMINAL_FLAG] = 0;
    ilqr_cost_config_vec[i] = ilqr_cost_config;
  }
  // terminal settings
  ilqr_cost_config_vec.back()[TERMINAL_FLAG] = 1;

  ilqr_core_ptr_->SetCostConfig(ilqr_cost_config_vec);

  StateVec result_xk;
  ControlVec result_uk;

  x0[0] << mpc_input.init_state_.dy_, mpc_input.init_state_.dphi_,
      mpc_input.init_state_.delta_;

  // only need init state to update
  ilqr_core_ptr_->Solve(x0[0]);

  ilqr_core_ptr_->GetOutput(result_xk, result_uk);
  //   std::vector<debugInfo> debug_info;
  //   ilqr_core_ptr_->GetDebugInfo(debug_info);
  mpc_out_.dx_vec_mpc_ = mpc_input.dx_ref_mpc_vec_;

  mpc_out_.time_vec_mpc_.clear();
  mpc_out_.dy_vec_mpc_.clear();
  mpc_out_.dphi_vec_mpc_.clear();
  mpc_out_.delta_vec_mpc_.clear();
  for (size_t i = 0; i < ilqr_core_ptr_->GetSolverConfigPtr()->horizion + 1;
       ++i) {
    mpc_out_.time_vec_mpc_.push_back(static_cast<double>(i) * 0.1);
    mpc_out_.dy_vec_mpc_.push_back(result_xk[i][Y]);
    mpc_out_.dphi_vec_mpc_.push_back(result_xk[i][THETA]);
    mpc_out_.delta_vec_mpc_.push_back(result_xk[i][DELTA]);
  }
  mpc_out_.omega_vec_mpc_.clear();
  for (size_t i = 0; i < ilqr_core_ptr_->GetSolverConfigPtr()->horizion;
       ++i) {
    mpc_out_.omega_vec_mpc_.push_back(result_uk[i][DELTA_DOT]);
  }
  mpc_out_.omega_vec_mpc_.push_back(mpc_out_.omega_vec_mpc_.back());

  return MPC_SOLVE_SUCCESS;
}

void LatMpcIlqr::Reset() {}

} // namespace control
} // namespace pnc