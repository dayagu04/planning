#include "al_ilqr_core.h"

#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <numeric>

#include "Eigen/Core"
#include "al_ilqr_define.h"
#include "math_lib.h"

using namespace pnc::mathlib;
namespace al_ilqr_solver {

// Initialize solver with model and config.
// Shares config_ptr between core and model, then allocates internal buffers.
// @param al_ilqr_model Pointer to the dynamics model
// @param al_ilqr_solver_config Solver configuration parameters
void AliLqr::Init(const std::shared_ptr<AliLqrModel> al_ilqr_model,
                   const AliLqrSolverConfig &al_ilqr_solver_config) {
  // init first to make core share config_ptr with model
  Init(al_ilqr_model);

  // set config
  *solver_config_ptr_ = al_ilqr_solver_config;

  // init config
  InitSolverConfig();
}

// Initialize solver with model only (config must be set separately).
// Links core and model to share the same config, cost, and constraint ptrs.
// @param al_ilqr_model Pointer to the dynamics model
void AliLqr::Init(std::shared_ptr<AliLqrModel> al_ilqr_model) {
  al_ilqr_model_ptr_ = al_ilqr_model;

  // solver config will also be used by model
  solver_config_ptr_ = al_ilqr_model->GetSolverConfigPtr();
  constraint_config_vec_ptr_ = al_ilqr_model->GetConstraintConfigPtr();
  cost_config_vec_ptr_ = al_ilqr_model->GetCostConfigPtr();
}

// Allocate and zero-initialize all internal vectors/matrices
// based on solver_config_ptr_ (horizon, state_size, input_size).
void AliLqr::InitSolverConfig() {
  const auto horizon = solver_config_ptr_->horizon;
  const auto input_size = solver_config_ptr_->input_size;
  const auto state_size = solver_config_ptr_->state_size;

  xk_vec_.resize(horizon + 1);
  ResetVectorElemFromEigen(xk_vec_);

  uk_vec_.resize(horizon + 1);
  ResizeVectorElemFromEigenVec(uk_vec_, input_size);
  ResetVectorElemFromEigen(uk_vec_);

  lx_vec_.resize(horizon + 1);
  ResizeVectorElemFromEigenVec(lx_vec_, state_size);
  ResetVectorElemFromEigen(lx_vec_);

  lu_vec_.resize(horizon + 1);
  ResizeVectorElemFromEigenVec(lu_vec_, input_size);
  ResetVectorElemFromEigen(lu_vec_);

  lxx_vec_.resize(horizon + 1);
  ResizeVectorElemFromEigenMat(lxx_vec_, state_size, state_size);
  ResetVectorElemFromEigen(lxx_vec_);

  lxu_vec_.resize(horizon + 1);
  ResizeVectorElemFromEigenMat(lxu_vec_, state_size, input_size);
  ResetVectorElemFromEigen(lxu_vec_);

  luu_vec_.resize(horizon + 1);
  ResizeVectorElemFromEigenMat(luu_vec_, input_size, input_size);
  ResetVectorElemFromEigen(luu_vec_);

  fx_vec_.resize(horizon);
  ResizeVectorElemFromEigenMat(fx_vec_, state_size, state_size);
  ResetVectorElemFromEigen(fx_vec_);

  fu_vec_.resize(horizon);
  ResizeVectorElemFromEigenMat(fu_vec_, state_size, input_size);
  ResetVectorElemFromEigen(fu_vec_);

  k_vec_.resize(horizon);
  ResizeVectorElemFromEigenVec(k_vec_, input_size);
  ResetVectorElemFromEigen(k_vec_);

  K_vec_.resize(horizon);
  ResizeVectorElemFromEigenMat(K_vec_, state_size, state_size);
  ResetVectorElemFromEigen(K_vec_);

  Quu_eye_ = Eigen::MatrixXd::Identity(input_size, input_size);

  // du norm debug
  du_norm_vec_.resize(horizon);
  // solver info init
  const auto max_iter = solver_config_ptr_->max_iter;
  solver_info_.solver_condition = AliLqrSolveCondition::INIT;
  solver_info_.iteration_info_vec.resize(max_iter + 1);

  // init advanced info
#ifdef __AL_ILQR_DEBUG__
  // +1 means considering init cost
  solver_info_.cost_map_vec.resize(max_iter + 1);

  // +1 means considering init cost
  solver_info_.cost_iter_vec.resize(max_iter + 1);

  // record al param
  const auto max_outer_iter = solver_config_ptr_->max_al_iter;
  const int constraint_num = solver_config_ptr_->constraint_num;
  solver_info_.al_iteration_info.mu_k_iteration.resize(max_outer_iter,
                                                       horizon + 1);
  solver_info_.al_iteration_info.rho_k_iteration.resize(max_outer_iter,
                                                        horizon + 1);

  solver_info_.al_iteration_info.mu_u_iteration.resize(max_outer_iter,
                                                       horizon + 1);

  solver_info_.al_iteration_info.rho_u_iteration.resize(max_outer_iter,
                                                        horizon + 1);

  solver_info_.al_iteration_info.constraint_data_iteration.resize(
      constraint_num, horizon + 1);

  solver_info_.lat_al_iteration_info.mu_hard_acc_iteration.resize(
      max_outer_iter, horizon + 1);
  solver_info_.lat_al_iteration_info.rho_hard_acc_iteration.resize(
      max_outer_iter, horizon + 1);
  solver_info_.lat_al_iteration_info.mu_hard_jerk_iteration.resize(
      max_outer_iter, horizon + 1);
  solver_info_.lat_al_iteration_info.rho_hard_jerk_iteration.resize(
      max_outer_iter, horizon + 1);
  solver_info_.lat_al_iteration_info.mu_hard_upper_pos_iteration.resize(
      max_outer_iter, horizon + 1);
  solver_info_.lat_al_iteration_info.rho_hard_upper_pos_iteration.resize(
      max_outer_iter, horizon + 1);
  solver_info_.lat_al_iteration_info.mu_hard_lower_pos_iteration.resize(
      max_outer_iter, horizon + 1);
  solver_info_.lat_al_iteration_info.rho_hard_lower_pos_iteration.resize(
      max_outer_iter, horizon + 1);

  solver_info_.lat_al_iteration_info.constraint_data_iteration.resize(
      constraint_num, horizon + 1);
#endif

  // init cost size
  solver_info_.cost_size = al_ilqr_model_ptr_->GetCostStackPtr()->size();

  // for al_ilqr_model
  al_ilqr_model_ptr_->InitControlVar();
}

// Allocate internal buffers with explicit state_size and input_size.
// Used when state/input dimensions change at runtime.
// @param state_size Dimension of state vector
// @param input_size Dimension of control vector
void AliLqr::InitSolverConfig(size_t state_size, size_t input_size) {
  const auto horizon = solver_config_ptr_->horizon;

  xk_vec_.resize(horizon + 1);
  ResetVectorElemFromEigen(xk_vec_);

  uk_vec_.resize(horizon + 1);
  ResizeVectorElemFromEigenVec(uk_vec_, input_size);
  ResetVectorElemFromEigen(uk_vec_);

  lx_vec_.resize(horizon + 1);
  ResizeVectorElemFromEigenVec(lx_vec_, state_size);
  ResetVectorElemFromEigen(lx_vec_);

  lu_vec_.resize(horizon + 1);
  ResizeVectorElemFromEigenVec(lu_vec_, input_size);
  ResetVectorElemFromEigen(lu_vec_);

  lxx_vec_.resize(horizon + 1);
  ResizeVectorElemFromEigenMat(lxx_vec_, state_size, state_size);
  ResetVectorElemFromEigen(lxx_vec_);

  lxu_vec_.resize(horizon + 1);
  ResizeVectorElemFromEigenMat(lxu_vec_, state_size, input_size);
  ResetVectorElemFromEigen(lxu_vec_);

  luu_vec_.resize(horizon + 1);
  ResizeVectorElemFromEigenMat(luu_vec_, input_size, input_size);
  ResetVectorElemFromEigen(luu_vec_);

  fx_vec_.resize(horizon);
  ResizeVectorElemFromEigenMat(fx_vec_, state_size, state_size);
  ResetVectorElemFromEigen(fx_vec_);

  fu_vec_.resize(horizon);
  ResizeVectorElemFromEigenMat(fu_vec_, state_size, input_size);
  ResetVectorElemFromEigen(fu_vec_);

  k_vec_.resize(horizon);
  ResizeVectorElemFromEigenVec(k_vec_, input_size);
  ResetVectorElemFromEigen(k_vec_);

  K_vec_.resize(horizon);
  ResizeVectorElemFromEigenMat(K_vec_, state_size, state_size);
  ResetVectorElemFromEigen(K_vec_);

  Quu_eye_ = Eigen::MatrixXd::Identity(input_size, input_size);

  // for al_ilqr_model
  al_ilqr_model_ptr_->InitControlVar();
}

// Add a cost term to the solver's cost stack.
// @param cost_term Shared pointer to the cost term
void AliLqr::AddCost(std::shared_ptr<AliLqrBaseCostTerm> cost_term) {
  al_ilqr_model_ptr_->AddCost(cost_term);
  solver_info_.cost_size++;
}

// Remove all cost terms from the solver.
void AliLqr::ClearCost() {
  al_ilqr_model_ptr_->ClearCost();
  solver_info_.cost_size = 0;
}

// Forward-simulate dynamics and record trajectory (no optimization).
// @param x0 Initial state
// @param u_vec Control sequence
void AliLqr::Simulation(const State &x0, const ControlVec &u_vec) {
  // set init x
  xk_vec_[0] = x0;

  // use last warm start result
  uk_vec_ = u_vec;
  al_ilqr_model_ptr_->UpateDynamics(xk_vec_, uk_vec_);
}

// Solve unconstrained iLQR with given initial control sequence.
// @param x0 Initial state
// @param u_vec Initial control sequence
void AliLqr::Solve(const State &x0, const ControlVec &u_vec) {
  uk_vec_ = u_vec;
  Solve(x0);
}

// Solve unconstrained iLQR from initial state x0.
// Uses warm start or zero-input as initial guess, then runs iLQR iteration.
// @param x0 Initial state
void AliLqr::Solve(const State &x0) {
#ifdef __AL_ILQR_TIMER__
  // init time info
  time_info_.Reset();
  time_info_.UpdateAllStart();
#endif

  // set init x
  xk_vec_[0] = x0;

  // set first cost_map before init guess
#ifdef __AL_ILQR_DEBUG__
  al_ilqr_model_ptr_->SetCostMapPtr(&(solver_info_.cost_map_vec[0]));
#endif

  // init guess
  al_ilqr_model_ptr_->InitGuess(xk_vec_, uk_vec_, cost_);

  // calculate t_init_guess_ms
#ifdef __AL_ILQR_TIMER__
  time_info_.t_init_guess_ms +=
      time_info_.GetElapsed(time_info_.all_start, false);
#endif

  // update advanced info after init guess
#ifdef __AL_ILQR_DEBUG__
  UpdateAdvancedInfo(0);
#endif

  // ilqr main iteration
  iLqrIteration();

  // t_one_step_ms
#ifdef __AL_ILQR_TIMER__
  time_info_.t_one_step_ms = time_info_.GetElapsed(time_info_.all_start, false);
#endif

#ifdef __AL_ILQR_PRINT__
  PrintCostInfo();
  PrintSolverInfo();
#endif
}

// Reset solver state by resetting the underlying model.
void AliLqr::Reset() { al_ilqr_model_ptr_->Reset(); }

// Check positive semi-definiteness of matrix Q.
// For scalar input (1x1), checks Quu(0,0) > lambda_min.
// For multi-input, uses Cholesky decomposition.
// @param Q Matrix to check
// @return true if Q is positive definite
bool AliLqr::PSDCheck(Eigen::MatrixXd &Q) {
  if (solver_config_ptr_->input_size == 1) {
    // For scalar input, Quu positive definite check is trivial.
    // lambda_min must less than 1e-5
    if (Quu_(0, 0) <= std::min(solver_config_ptr_->lambda_min, 1e-5)) {
      lambda_ = solver_config_ptr_->lambda_fix;
      return false;
    } else {
      return true;
    }
  } else {
    Eigen::LLT<Eigen::MatrixXd> lltOfA(Q);
    if (lltOfA.info() == Eigen::NumericalIssue) {
      return false;
    } else {
      return true;
    }
  }
}

// Backward pass: compute feedback gains k, K by dynamic programming.
//
// Bellman recursion from terminal cost-to-go:
//   Qx  = lx  + fx^T * Vx
//   Qu  = lu  + fu^T * Vx
//   Qxx = lxx + fx^T * Vxx * fx
//   Quu = luu + fu^T * Vxx * fu
//   Qux = lxu^T + fu^T * Vxx * fx
//
// Regularized feedback gains (Tassa 2012, Eq. 10):
//   Quuf = Quu + lambda * I
//   k = -Quuf^{-1} * Qu       (feedforward)
//   K = -Quuf^{-1} * Qux      (feedback)
//
// Value function update (Tassa 2012, Eq. 11):
//   dV[0] += k^T * Qu
//   dV[1] += 0.5 * k^T * Quu * k
//   Vx  = Qx  + (K^T * Quu + Qux^T) * k + K^T * Qu
//   Vxx = Qxx + (K^T * Quu + Qux^T) * K + K^T * Qux
//
// @return true if backward pass succeeds (Quuf is PD at all steps)
bool AliLqr::BackwardPass() {
  // cost-to-go at end
  Eigen::MatrixXd Vx = lx_vec_[solver_config_ptr_->horizon];
  Eigen::MatrixXd Vxx = lxx_vec_[solver_config_ptr_->horizon];

  dV_.fill(0);

  for (int i = (static_cast<int>(solver_config_ptr_->horizon) - 1); i >= 0;
       i--) {  // back up from end of trajectory

    const Eigen::MatrixXd fut = fu_vec_[i].transpose();
    const Eigen::MatrixXd fxt = fx_vec_[i].transpose();
    const Eigen::MatrixXd fu_Vxx = fut * Vxx;

    Qx_ = lx_vec_[i] + fxt * Vx;
    Qu_ = lu_vec_[i] + fut * Vx;
    Qxx_ = lxx_vec_[i] + fxt * Vxx * fx_vec_[i];
    Quu_ = luu_vec_[i] + fu_Vxx * fu_vec_[i];
    Qux_ = lxu_vec_[i].transpose() + fu_Vxx * fx_vec_[i];

    // Regularization: Quuf = Quu + lambda * I (Tassa 2012)
    Quuf_ = Quu_ + lambda_ * Quu_eye_;
    if (!PSDCheck(Quuf_)) {
      return false;
    }

    const Eigen::MatrixXd QuuF_inv = Quuf_.inverse();
    k_i_ = -QuuF_inv * Qu_;
    K_i_ = -QuuF_inv * Qux_;

    const Eigen::MatrixXd Kt = K_i_.transpose();
    const Eigen::MatrixXd Kt_Quu = Kt * Quu_;
    const Eigen::MatrixXd Quxt = Qux_.transpose();
    const Eigen::MatrixXd Kt_Quu_Quxt = Kt_Quu + Quxt;

    // Update cost-to-go approximation (Tassa 2012, Eq. 11)
    dV_[0] += k_i_.transpose() * Qu_;
    dV_[1] += 0.5 * k_i_.transpose() * Quu_ * k_i_;

    Vx = Qx_ + Kt_Quu_Quxt * k_i_ + Kt * Qu_;
    Vxx = Qxx_ + Kt_Quu_Quxt * K_i_ + Kt * Qux_;

    // save controls/gains
    k_vec_[i] = k_i_;
    K_vec_[i] = K_i_;
  }
  return true;
}

// Forward pass: linesearch along the computed direction (k, K).
//
// For each linesearch step alpha:
//   du = alpha * k[i] + K[i] * (x_new[i] - x[i])
//   u_new[i] = u[i] + du
//   x_new[i+1] = f(x_new[i], u_new[i])
//
// Accept if expected improvement > 0 and actual/expected ratio > z_min.
//
// @param new_cost [out] Cost of accepted trajectory
// @param expected [out] Expected cost reduction from quadratic model
// @param iter Current iteration index
// @return true if linesearch finds an acceptable step
bool AliLqr::ForwardPass(double &new_cost, double &expected,
                          const size_t &iter) {
  double alpha;
  xk_new_vec_ = xk_vec_;
  uk_new_vec_ = uk_vec_;

  size_t linsearch_start_index = 0;
  if (iter > 0 &&
      solver_info_.iteration_info_vec[iter - 1].linesearch_success == false) {
    linsearch_start_index = solver_config_ptr_->alpha_vec.size() - 2;
  }

  solver_info_.iteration_info_vec[iter].linesearch_count = 0;
  // linesearch process
  for (size_t i = linsearch_start_index;
       i < solver_config_ptr_->alpha_vec.size(); ++i) {
    // set linesearch count for info
    solver_info_.iteration_info_vec[iter].linesearch_count++;

    alpha = solver_config_ptr_->alpha_vec[i];
    new_cost = 0.0;

    for (auto &each_cost : *(al_ilqr_model_ptr_->GetCostStackPtr())) {
      each_cost->ResetCostValue();
    }
    for (size_t i = 0; i < solver_config_ptr_->horizon; ++i) {
      const auto du =
          alpha * k_vec_[i] + K_vec_[i] * (xk_new_vec_[i] - xk_vec_[i]);

      uk_new_vec_[i] = uk_vec_[i] + du;
      du_norm_vec_[i] = du.norm();

      new_cost +=
          al_ilqr_model_ptr_->GetCost(xk_new_vec_[i], uk_new_vec_[i], i);

      xk_new_vec_[i + 1] = al_ilqr_model_ptr_->UpdateDynamicsOneStep(
          xk_new_vec_[i], uk_new_vec_[i], i);
    }

    const double du_norm =
        std::accumulate(du_norm_vec_.begin(), du_norm_vec_.end(), 0.0) /
        du_norm_vec_.size();
    solver_info_.iteration_info_vec[iter].du_norm = du_norm;

    new_cost += al_ilqr_model_ptr_->GetTerminalCost(
        xk_new_vec_[solver_config_ptr_->horizon]);

    // Expected improvement from quadratic model:
    //   E[dJ] = -alpha * (dV[0] + alpha * dV[1])
    expected = -alpha * (dV_[0] + alpha * dV_[1]);

    // note that (expected >= 0.0) does not mean reduced cost
    if (expected >= 0.0) {
      if ((cost_ - new_cost) / expected > solver_config_ptr_->z_min) {
        // line search success
        xk_vec_ = xk_new_vec_;
        uk_vec_ = uk_new_vec_;
        return true;
      } else if (du_norm < solver_config_ptr_->du_tol) {
        solver_info_.solver_condition = CONST_CONTROL_TERMINATE;
        return false;
      }
    } else {
      // expected < 0 should not occur
      solver_info_.solver_condition = NON_POSITIVE_EXPECT;
      return false;
    }
  }
  solver_info_.solver_condition = MAX_ITER_TERMINATE;
  return false;
}

// Update max inner iteration count and reallocate solver info buffers.
// @param max_iter New maximum inner iteration count
void AliLqr::SetMaxIter(size_t max_iter) {
  solver_config_ptr_->max_iter = max_iter;

  solver_info_.iteration_info_vec.resize(max_iter + 1);

  InitAdvancedInfo();
}

// Initialize debug cost tracking structures.
// Allocates cost_map_vec and cost_iter_vec for each inner iteration.
void AliLqr::InitAdvancedInfo() {
#ifdef __AL_ILQR_DEBUG__
  // +1 means considering init cost
  solver_info_.cost_map_vec.resize(solver_config_ptr_->max_iter + 1);

  // +1 means considering init cost
  solver_info_.cost_iter_vec.resize(solver_config_ptr_->max_iter + 1);

  std::vector<double> tmp_vec;
  tmp_vec.resize(solver_config_ptr_->horizon + 1, 0.0);

  for (size_t iter = 0; iter < solver_config_ptr_->max_iter + 1; ++iter) {
    // init cost map after adding cost
    auto &cost_map = solver_info_.cost_map_vec[iter];
    cost_map.clear();
    for (auto &each_cost : *(al_ilqr_model_ptr_->GetCostStackPtr())) {
      cost_map.insert(std::make_pair(each_cost->GetCostId(), tmp_vec));
    }

    // init cost_iter_vec after adding cost
    solver_info_.cost_iter_vec[iter].resize(solver_info_.cost_size, 0);
  }
#endif
}

// Increase regularization lambda when backward pass fails.
// lambda_gain = max(lambda_factor, lambda_gain * lambda_factor)
// lambda      = max(lambda_start, lambda * lambda_gain)
void AliLqr::IncreaseLambda() {
  lambda_gain_ = std::max(solver_config_ptr_->lambda_factor,
                          lambda_gain_ * solver_config_ptr_->lambda_factor);
  lambda_ = std::max(solver_config_ptr_->lambda_start, lambda_ * lambda_gain_);
}

// Decrease regularization lambda after successful forward pass.
// lambda_gain = min(1/lambda_factor, lambda_gain / lambda_factor)
// lambda      = lambda * lambda_gain * (lambda > lambda_min)
void AliLqr::DecreaseLambda() {
  lambda_gain_ = std::min(1.0 / solver_config_ptr_->lambda_factor,
                          lambda_gain_ / solver_config_ptr_->lambda_factor);
  lambda_ = lambda_ * lambda_gain_ * (lambda_ > solver_config_ptr_->lambda_min);
}

// Compute dynamics Jacobians (fx, fu) and cost derivatives (lx, lu, lxx, lxu, luu)
// at each step of the current trajectory (xk_vec_, uk_vec_).
// Terminal step uses GetTerminalGradientHessian (no dynamics Jacobian needed).
void AliLqr::UpdateDynamicsDerivatives() {
  for (size_t i = 0; i < solver_config_ptr_->horizon + 1; ++i) {
    // reset Derivatives before updating
    lx_vec_[i].setZero();
    lu_vec_[i].setZero();
    lxx_vec_[i].setZero();
    lxu_vec_[i].setZero();
    luu_vec_[i].setZero();

    if (i < solver_config_ptr_->horizon) {
      al_ilqr_model_ptr_->GetDynamicsDerivatives(xk_vec_[i], uk_vec_[i],
                                                  fx_vec_[i], fu_vec_[i], i);

      al_ilqr_model_ptr_->GetGradientHessian(xk_vec_[i], uk_vec_[i], i,
                                              lx_vec_[i], lu_vec_[i],
                                              lxx_vec_[i], lxu_vec_[i],
                                              luu_vec_[i]);
    } else {
      al_ilqr_model_ptr_->GetTerminalGradientHessian(xk_vec_[i], lx_vec_[i],
                                                      lu_vec_[i], lxx_vec_[i],
                                                      lxu_vec_[i], luu_vec_[i]);
    }
  }
}

// Inner iLQR iteration loop.
// Alternates backward pass (compute gains) and forward pass (linesearch)
// until cost convergence, control stationarity, or max iterations.
//
// @return true if solver converges successfully
bool AliLqr::iLqrIteration() {
  if (xk_vec_.empty() || uk_vec_.empty()) {
    solver_info_.solver_condition = AliLqrSolveCondition::FAULT_INPUT_SIZE;
    return false;
  }

  solver_info_.init_cost = cost_;

  // init some states
  solver_info_.solver_condition = AliLqrSolveCondition::INIT;
  lambda_ = solver_config_ptr_->lambda_start;
  lambda_gain_ = 1.0;

  // iteration loop
  bool solver_success = false;
  bool update_success = true;
  for (size_t iter = 0; iter < solver_config_ptr_->max_iter; ++iter) {
    // timer tag for iteration start
#ifdef __AL_ILQR_TIMER__
    time_info_.UpdateStart();
#endif

    // set iter count for info
    solver_info_.iter_count = iter + 1;

    // set cost_map before iteration
#ifdef __AL_ILQR_DEBUG__
    al_ilqr_model_ptr_->SetCostMapPtr(&(solver_info_.cost_map_vec[iter + 1]));
#endif

    // update dynamics and derivatives when iter success
    if (update_success) {
      // Differentiate dynamics and cost along new trajectory before iteration
      UpdateDynamicsDerivatives();
      update_success = false;
    }

    // calculate t_compute_deriv_ms
#ifdef __AL_ILQR_TIMER__
    time_info_.t_compute_deriv_ms +=
        time_info_.GetElapsed(time_info_.start, true);
#endif

    // STEP 1: backward pass
    size_t backward_pass_count = 0;
    while (true) {
      backward_pass_count++;

      // set backward pass count for info
      solver_info_.iteration_info_vec[iter].backward_pass_count =
          backward_pass_count;

      // Update Vx, Vxx, l, L, dV with back_pass
      const bool is_converged = BackwardPass();
      if (is_converged) {
        break;
      } else {
        if (lambda_ > solver_config_ptr_->lambda_max ||
            backward_pass_count >=
                solver_config_ptr_->max_backward_pass_count) {
          // backward pass failed, this should not happen when input_size = 1
          solver_info_.solver_condition =
              AliLqrSolveCondition::BACKWARD_PASS_FAIL;
          return false;
        }
        IncreaseLambda();
      }
    }

    // calculate t_backward_pass_ms
#ifdef __AL_ILQR_TIMER__
    time_info_.t_backward_pass_ms +=
        time_info_.GetElapsed(time_info_.start, true);
#endif

    // STEP 2: forward pass
    double expected = 0.0;
    double new_cost = 0.0;
    const bool forward_pass_success = ForwardPass(new_cost, expected, iter);
    const double dcost = cost_ - new_cost;

    // solver info recording
    solver_info_.iteration_info_vec[iter].linesearch_success =
        forward_pass_success;

    solver_info_.iteration_info_vec[iter].cost = new_cost;
    solver_info_.iteration_info_vec[iter].dcost = dcost;
    solver_info_.iteration_info_vec[iter].lambda = lambda_;
    solver_info_.iteration_info_vec[iter].expect = expected;
    solver_info_.iteration_info_vec[iter].cost_vec.clear();
    for (auto &each_cost : *(al_ilqr_model_ptr_->GetCostStackPtr())) {
      CostInfo each_cost_info(each_cost->GetCostId(),
                              each_cost->GetCostValue());
      solver_info_.iteration_info_vec[iter].cost_vec.emplace_back(
          std::move(each_cost_info));
    }

    // calculate t_forward_pass_ms
#ifdef __AL_ILQR_TIMER__
    time_info_.t_forward_pass_ms +=
        time_info_.GetElapsed(time_info_.start, true);
#endif

    // update advanced info after forward pass
#ifdef __AL_ILQR_DEBUG__
    // +1 means considering init
    UpdateAdvancedInfo(iter + 1);
#endif

    // STEP 3: check if terminate
    if (forward_pass_success) {
      // decrease lambda
      DecreaseLambda();

      // accept changes
      cost_ = new_cost;

      // when accept change, dynamics and derivatives should be updated
      update_success = true;

      // terminate check
      if (dcost < solver_config_ptr_->cost_tol) {
        solver_success = true;
        solver_info_.solver_condition = AliLqrSolveCondition::NORMAL_TERMINATE;
        break;
      }
    } else {
      // linesearch failed, increase lambda
      IncreaseLambda();

      // terminate?
      if (lambda_ > solver_config_ptr_->lambda_max) {
        solver_success = true;
        if (iter > 0) {
          solver_info_.solver_condition =
              AliLqrSolveCondition::LINESEARCH_TERMINATE;
        } else {
          solver_info_.solver_condition = AliLqrSolveCondition::INIT_TERMINATE;
        }
        break;
      } else if (solver_info_.solver_condition ==
                 AliLqrSolveCondition::NON_POSITIVE_EXPECT) {
        solver_success = false;
      }
    }

    if (solver_info_.iteration_info_vec[iter].du_norm <
        solver_config_ptr_->du_tol) {
      solver_success = true;
      if (iter > 0) {
        solver_info_.solver_condition =
            AliLqrSolveCondition::CONST_CONTROL_TERMINATE;
      } else {
        solver_info_.solver_condition = AliLqrSolveCondition::INIT_TERMINATE;
      }
      break;
    }

    if (iter == solver_config_ptr_->max_iter - 1) {
      solver_info_.solver_condition = AliLqrSolveCondition::MAX_ITER_TERMINATE;
      solver_success = true;
    }
  }  // end of iteration loop

  al_ilqr_model_ptr_->UpdateWarmStart(solver_success, uk_vec_);

  return solver_success;
}

// Record trajectory and per-cost-term cost values at iteration iter.
// @param iter Iteration index (0 = init guess)
void AliLqr::UpdateAdvancedInfo(size_t iter) {
  // record x_vec and u_vec for each iteration
  solver_info_.iteration_info_vec[iter].x_vec = xk_vec_;
  solver_info_.iteration_info_vec[iter].u_vec = uk_vec_;

  // record every cost term for each iteration
  auto &cost_map = solver_info_.cost_map_vec[iter];

  size_t i = 0;
  for (auto &each_cost : *(al_ilqr_model_ptr_->GetCostStackPtr())) {
    auto const &vec = cost_map[each_cost->GetCostId()];
    solver_info_.cost_iter_vec[iter].at(i++) =
        std::accumulate(vec.begin(), vec.end(), 0.0);
  }
}

// Print inner iLQR solver iteration info table.
void AliLqr::PrintSolverInfo() {
  std::cout << "--------------------------------------------------- 2. AliLqr "
               "solver info "
               "--------------------------------------------------- "
            << std::endl;
  // init info
  std::cout << "init state = " << xk_vec_[0].transpose() << std::endl;

  // iteration info
  std::cout << "cost size = " << solver_info_.cost_size
            << ", init cost = " << solver_info_.init_cost
            << ", iteration count = " << solver_info_.iter_count
            << ", solver condition = "
            << static_cast<size_t>(solver_info_.solver_condition) << std::endl;

  std::cout << std::left << std::setw(12) << "iteration" << std::setw(12)
            << "cost" << std::setw(12) << "reduction" << std::setw(12)
            << "expect" << std::setw(12) << "lambda" << std::setw(12)
            << "LS_count" << std::setw(12) << "LS_success" << std::setw(12)
            << "du_norm" << "\n";

  for (size_t iter = 0; iter < solver_info_.iter_count; ++iter) {
    const IterationInfo &info = solver_info_.iteration_info_vec[iter];

    std::cout << std::left << std::setw(12) << iter << std::setw(12)
              << info.cost << std::setw(12) << info.dcost << std::setw(12)
              << info.expect << std::setw(12) << info.lambda << std::setw(12)
              << info.linesearch_count << std::setw(12)
              << info.linesearch_success << std::setw(12) << info.du_norm
              << std::endl;
  }
}

// Print AL-iLQR solver info including outer loop statistics.
void AliLqr::PrintAlSolverInfo() {
  std::cout << "-------------------------------- 2. AliLqr "
               "solver info "
               "-------------------------------- "
            << std::endl;
  // init info
  std::cout << "init state = " << xk_vec_[0].transpose() << std::endl;

  // iteration info
  std::cout << "cost size = " << solver_info_.cost_size
            << ", init cost = " << solver_info_.init_cost
            << ", outer iteration count = " << solver_info_.outer_iter_count
            << ", cost_derivation_value = " << solver_info_.dcost_outer
            << ", constraint_violation = " << solver_info_.constraint_violation
            << ", iteration count = " << solver_info_.iter_count
            << ", solver condition = "
            << static_cast<size_t>(solver_info_.solver_condition) << std::endl;

  std::cout << std::left << std::setw(12) << "iteration" << std::setw(12)
            << "cost" << std::setw(12) << "reduction" << std::setw(12)
            << "expect" << std::setw(12) << "lambda" << std::setw(12)
            << "LS_count" << std::setw(12) << "LS_success" << std::setw(12)
            << "du_norm" << "\n";

  for (size_t iter = 0; iter < solver_info_.iter_count; ++iter) {
    const IterationInfo &info = solver_info_.iteration_info_vec[iter];

    std::cout << std::left << std::setw(12) << iter << std::setw(12)
              << info.cost << std::setw(12) << info.dcost << std::setw(12)
              << info.expect << std::setw(12) << info.lambda << std::setw(12)
              << info.linesearch_count << std::setw(12)
              << info.linesearch_success << std::setw(12) << info.du_norm
              << std::endl;
  }
}

// Virtual hook for printing AL parameter state (mu, rho) before update.
// Derived classes override to log constraint-specific parameters.
void AliLqr::PrintAlParamInfo() {}

// Virtual hook for printing AL parameter state after update.
void AliLqr::PrintAlParamInfoAfter() {}

// Print per-cost-term cost breakdown for all iterations.
void AliLqr::PrintCostInfo() {
#ifdef __AL_ILQR_DEBUG__
  // cost info
  std::cout << "\n-------------------------------- 1. cost vec info "
               "--------------------------------"
            << std::endl;
  std::cout << "cost list: [";
  for (size_t i = 0; i < al_ilqr_model_ptr_->GetCostStackPtr()->size(); ++i) {
    if (i < al_ilqr_model_ptr_->GetCostStackPtr()->size() - 1) {
      std::cout
          << al_ilqr_model_ptr_->GetCostStackPtr()->at(i)->GetCostString()
          << ", ";
    } else {
      std::cout
          << al_ilqr_model_ptr_->GetCostStackPtr()->at(i)->GetCostString()
          << "]" << std::endl;
    }
  }
  for (size_t iter = 0; iter < solver_info_.iter_count + 1; ++iter) {
    if (iter == 0) {
      std::cout << "cost_vec[init] = [ ";
    } else {
      std::cout << "cost_vec[" << iter - 1 << "]    = [ ";
    }

    for (size_t i = 0; i < solver_info_.cost_size; ++i) {
      std::cout << std::fixed << std::setprecision(5)
                << solver_info_.cost_iter_vec[iter].at(i) << " ";
    }

    std::cout << "]" << std::endl;
  }
#endif
}

// Print timing breakdown for iLQR stages.
void AliLqr::PrintTimeInfo() {
#ifdef __AL_ILQR_TIMER__
  // time info
  std::cout << "\n-----time info:" << std::endl;
  std::cout << "Total time: " << time_info_.t_one_step_ms << std::endl;
  std::cout << "compute_derivatives: " << time_info_.t_compute_deriv_ms
            << std::endl;
  std::cout << "backward pass: " << time_info_.t_backward_pass_ms << std::endl;
  std::cout << "forward pass: " << time_info_.t_forward_pass_ms << std::endl;
  std::cout << "other stuff: "
            << time_info_.t_one_step_ms - (time_info_.t_compute_deriv_ms +
                                           time_info_.t_backward_pass_ms +
                                           time_info_.t_forward_pass_ms)
            << std::endl;
#endif
}

// ==================== AL-iLQR Specific Methods ====================

// Main entry point for constrained AL-iLQR solve.
// Sets up initial guess with warm start, then calls AliLqrIteration().
// @param x0 Initial state
void AliLqr::SolveForAliLqr(const State &x0) {
#ifdef __AL_ILQR_TIMER__
  // init time info
  time_info_.Reset();
  time_info_.UpdateAllStart();
#endif

  // set init x
  xk_vec_[0] = x0;

  // set first cost_map before init guess
#ifdef __AL_ILQR_DEBUG__
  al_ilqr_model_ptr_->SetCostMapPtr(&(solver_info_.cost_map_vec[0]));
#endif

  // init guess: AL-iLQR always uses warm start from previous solve
  solver_config_ptr_->warm_start_enable = true;
  al_ilqr_model_ptr_->InitGuess(xk_vec_, uk_vec_, cost_);

  // calculate t_init_guess_ms
#ifdef __AL_ILQR_TIMER__
  time_info_.t_init_guess_ms +=
      time_info_.GetElapsed(time_info_.all_start, false);
#endif

  // update advanced info after init guess
#ifdef __AL_ILQR_DEBUG__
  UpdateAdvancedInfo(0);
#endif

  // AL-iLQR outer loop
  AliLqrIteration();

  // t_one_step_ms
#ifdef __AL_ILQR_TIMER__
  time_info_.t_one_step_ms = time_info_.GetElapsed(time_info_.all_start, false);
#endif
}

// Reset inner solver state for each outer AL iteration.
// Re-initializes iteration info and cost tracking while preserving trajectory.
void AliLqr::InitAliLqrSolverConfig() {
  // solver info init
  const auto max_iter = solver_config_ptr_->max_iter;
  solver_info_.solver_condition = AliLqrSolveCondition::INIT;
  solver_info_.iteration_info_vec.resize(max_iter + 1);

  // init advanced info
#ifdef __AL_ILQR_DEBUG__
  // +1 means considering init cost
  solver_info_.cost_map_vec.resize(max_iter + 1);

  // +1 means considering init cost
  solver_info_.cost_iter_vec.resize(max_iter + 1);
#endif
  // init cost size
  solver_info_.iter_count = 0;
  solver_info_.init_cost = 0.0;
}

// Augmented Lagrangian outer iteration loop.
//
// Algorithm (Howell et al. 2019, ALTRO):
//   for j = 1, ..., max_al_iter:
//     1. Reset inner solver config
//     2. Solve augmented unconstrained problem via iLQR inner loop
//        min J(x,u) + sum_k [ I_AL(c_k, mu_k, rho_k) ]
//        where I_AL is the AL penalty for constraint c at step k
//     3. Evaluate constraint violations:  max|c(x,u)|
//     4. Check KKT convergence:
//        constraint_violation < tol  AND  |dcost| < tol
//     5. If not converged, update AL parameters:
//        mu_new = max(0, mu + rho * c)     (dual variable update)
//        rho_new = phi * rho               (penalty escalation)
//     6. Tighten inner cost tolerance: cost_tol *= cost_scale
//
// The augmented cost gradient/Hessian contributions (for c(x,u) <= 0):
//   When constraint is active (c + mu/rho > 0):
//     cost += mu * c + (rho/2) * c^2
//     dl/dx += (mu + rho*c) * dc/dx
//     d2l/dx2 += rho * (dc/dx)(dc/dx)^T + (mu+rho*c) * d2c/dx2
//   When constraint is inactive (c + mu/rho <= 0):
//     cost += -mu^2 / (2*rho)   (constant, no gradient contribution)
void AliLqr::AliLqrIteration() {
  // Record cost before outer loop begins
  prev_outer_cost_ = cost_;

  // Timestamp for wall-clock time limit enforcement
  auto al_start_time = std::chrono::system_clock::now();

  // Save original inner cost tolerance for progressive tightening
  const double original_cost_tol = solver_config_ptr_->cost_tol;

  // Clear accumulated outer iteration records
  solver_info_.outer_iter_records.clear();

  for (size_t outer_iter = 0;
       outer_iter < solver_config_ptr_->max_al_iter;
       ++outer_iter) {
    // Record outer iteration count
    solver_info_.outer_iter_count = outer_iter + 1;

    // STEP 1: Reset inner solver state for this AL iteration.
    // Preserves trajectory (xk_vec_, uk_vec_) as warm start for inner solve.
    InitAliLqrSolverConfig();

    // STEP 2: Solve augmented unconstrained problem via standard iLQR.
    // Cost terms include AL penalty: J + sum[ mu*c + (rho/2)*c^2 ].
    // The penalty is embedded in GetCost() / GetGradientHessian() of
    // derived cost terms that read (mu, rho) from constraint_config.
    iLqrIteration();

    // STEP 3: Evaluate constraint violations after inner solve.
    //   constraint_violation = max_k max_i |max(0, c_i(x_k, u_k))|
    const double constraint_violation = MaxConstraintViolation();
    solver_info_.constraint_violation = constraint_violation;

    // Evaluate cost change between successive outer iterations.
    //   dcost_outer = |J_{j} - J_{j-1}|
    const double dcost_outer = MaxDerivationValue();
    solver_info_.dcost_outer = dcost_outer;

    // Record this outer iteration's full inner solve snapshot
    OuterIterRecord record;
    record.inner_iter_count = solver_info_.iter_count;
    record.init_cost = solver_info_.init_cost;
    record.final_cost = cost_;
    record.constraint_violation = constraint_violation;
    record.dcost_outer = dcost_outer;
    record.inner_iter_info_vec.assign(
        solver_info_.iteration_info_vec.begin(),
        solver_info_.iteration_info_vec.begin() +
            solver_info_.iter_count);
#ifdef __AL_ILQR_DEBUG__
    // Include init cost (iter 0) + all inner iterations (iter 1...N)
    // solver_info_.cost_iter_vec[0] = cost breakdown at warm start
    // solver_info_.cost_iter_vec[1...N] = cost breakdown after each iteration
    record.inner_cost_iter_vec.assign(
        solver_info_.cost_iter_vec.begin(),
        solver_info_.cost_iter_vec.begin() +
            solver_info_.iter_count + 1);  // +1 to include init
#endif
    solver_info_.outer_iter_records.emplace_back(std::move(record));

    // Debug output
#ifdef __AL_ILQR_PRINT__
    PrintAlSolverInfo();
    PrintAlParamInfo();
#endif

    // STEP 4: KKT convergence check.
    // Primal feasibility: constraint_violation < constraint_tolerance_tol
    // Stationarity:       dcost_outer < cost_tolerance_tol
    // (Dual feasibility and complementary slackness are maintained
    //  by the max(0, ...) update rule for inequality constraints)
    if (constraint_violation <
            solver_config_ptr_->constraint_tolerance_tol &&
        dcost_outer <
            solver_config_ptr_->cost_tolerance_tol) {
      solver_info_.solver_condition = AliLqrSolveCondition::KKT_TERMINATE;
#ifdef __AL_ILQR_PRINT__
      PrintAlParamInfoAfter();
#endif
      break;
    }

    // Wall-clock time limit check (real-time safety constraint)
    auto now = std::chrono::system_clock::now();
    const double elapsed_ms =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            now - al_start_time)
            .count() * 1e-6;
    if (elapsed_ms > solver_config_ptr_->max_al_solve_time) {
      solver_info_.solver_condition =
          AliLqrSolveCondition::TIME_LIMIT_TERMINATE;
#ifdef __AL_ILQR_PRINT__
      PrintAlParamInfoAfter();
#endif
      break;
    }

    // STEP 5: Update Augmented Lagrangian parameters.
    // Dual variable update:   mu_{j+1} = max(0, mu_j + rho_j * c_j)
    // Penalty escalation:     rho_{j+1} = phi * rho_j
    //   where phi = penalty_scaling (typically 10)
    // This ensures monotone penalty growth -> constraint satisfaction.
    UpdateAugmentedLagragian();

    // STEP 5b: Recompute cost_ under updated AL parameters (mu, rho).
    // After UpdateAugmentedLagragian(), the cost landscape has changed.
    // Without re-evaluation, the next inner iLQR's ForwardPass would
    // compare new_cost (computed with updated mu/rho) against stale
    // cost_ (computed with old mu/rho), causing spurious cost increase
    // and linesearch failure.
    cost_ = 0.0;
    for (auto &each_cost : *(al_ilqr_model_ptr_->GetCostStackPtr())) {
      each_cost->ResetCostValue();
    }
    for (size_t i = 0; i < solver_config_ptr_->horizon; ++i) {
      cost_ +=
          al_ilqr_model_ptr_->GetCost(xk_vec_[i], uk_vec_[i], i);
    }
    cost_ += al_ilqr_model_ptr_->GetTerminalCost(
        xk_vec_[solver_config_ptr_->horizon]);

#ifdef __AL_ILQR_PRINT__
    PrintAlParamInfoAfter();
#endif

    // Record cost for next outer iteration convergence check
    prev_outer_cost_ = cost_;

    // STEP 6: Progressive inner tolerance tightening.
    // cost_tol *= cost_scale (e.g., 0.95)
    // This allows coarse inner solves early on, tightening as constraints
    // approach satisfaction, improving overall computational efficiency.
    solver_config_ptr_->cost_tol *= solver_config_ptr_->cost_scale;

    // Check max outer iteration reached
    if (outer_iter == solver_config_ptr_->max_al_iter - 1) {
      solver_info_.solver_condition =
          AliLqrSolveCondition::MAX_OUTERITER_TERMINATE;
    }
  }

  // Restore original inner cost tolerance
  solver_config_ptr_->cost_tol = original_cost_tol;

  // Update warm start with final result
  al_ilqr_model_ptr_->UpdateWarmStart(true, uk_vec_);
}

// Default implementation: no constraint violations (base class has no constraints).
// Derived classes override to compute max |max(0, c_i(x_k, u_k))| over all
// constraint types and horizon steps.
// @return Maximum constraint violation value (0 = all satisfied)
double AliLqr::MaxConstraintViolation() {
  const double max_violation = 0.0;
  return max_violation;
}

// Default implementation: compute absolute cost change between outer iterations.
// Derived classes may override with problem-specific stationarity metrics.
// @return Cost deviation |J_current - J_previous|
double AliLqr::MaxDerivationValue() {
  const double derivation_error = std::abs(cost_ - prev_outer_cost_);
  return derivation_error;
}

// Default implementation: no-op.
// Derived classes override to update (mu, rho) in constraint_config_vec_ptr_
// based on current constraint violations.
//
// Standard update rule for inequality constraint c(x,u) <= 0:
//   mu_new  = max(0, mu + rho * c)    (project to non-negative for dual feasibility)
//   rho_new = phi * rho               (increase penalty when constraint violated)
//   rho_new = min(rho_new, rho_max)   (cap penalty to avoid ill-conditioning)
void AliLqr::UpdateAugmentedLagragian() {
  // Derived classes implement problem-specific AL parameter updates
  // by modifying values in constraint_config_vec_ptr_
}

}  // namespace al_ilqr_solver
