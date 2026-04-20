#include "al_ilqr_model.h"

#include "math_lib.h"

using namespace pnc::mathlib;

namespace al_ilqr_solver {

// Initialize control variable buffers.
// Allocates u_ (single control) and uk_vec_ (control sequence) with correct size.
void AliLqrModel::InitControlVar() {
  // u init
  u_.resize(solver_config_ptr_->input_size);
  u_.setZero();

  // uk_vec init
  uk_vec_.resize(solver_config_ptr_->horizon + 1);
  ResizeVectorElemFromEigenVec(uk_vec_, solver_config_ptr_->input_size);
  ResetVectorElemFromEigen(uk_vec_);
}

// Forward-simulate dynamics and accumulate total cost over horizon.
// x0[t+1] = f(x0[t], u0[t]) for t = 0..horizon-1
// cost = sum_{t=0}^{horizon-1} l(x_t, u_t) + l_f(x_horizon)
// @param x0 [in/out] State trajectory (x0[0] must be set)
// @param u0 Control sequence
// @return Total cost including terminal cost
double AliLqrModel::UpateDynamics(StateVec &x0, const ControlVec &u0) {
  double cost = 0.0;
  for (auto &each_cost : cost_stack_) {
    each_cost->ResetCostValue();
  }
  for (size_t t = 0; t < solver_config_ptr_->horizon; ++t) {
    cost += GetCost(x0[t], u0[t], t);
    x0[t + 1] = UpdateDynamicsOneStep(x0[t], u0[t], t);
  }
  cost += GetTerminalCost(x0[solver_config_ptr_->horizon]);
  return cost;
}

// Add a cost term to the model's cost stack.
// @param cost_term Shared pointer to cost term
void AliLqrModel::AddCost(std::shared_ptr<AliLqrBaseCostTerm> cost_term) {
  cost_stack_.emplace_back(std::move(cost_term));
}

// Remove all cost terms from the model's cost stack.
void AliLqrModel::ClearCost() {
  cost_stack_.clear();
}

// Compute total running cost at step.
// Iterates over all cost terms, sets per-step config, accumulates cost.
// When al_ilqr_enable is true, constraint configs (mu, rho) are also set
// so that cost terms can compute augmented Lagrangian penalty.
//
// @param x State at step
// @param u Control at step
// @param step Time step index
// @return Total scalar cost sum_i J_i(x, u)
double AliLqrModel::GetCost(const State &x, const Control &u,
                             const size_t &step) {
  double cost = 0.0;
  double result = 0.0;
  for (auto &each_cost : cost_stack_) {
    each_cost->SetConfig(&cost_config_vec_ptr_->at(step));
    if (solver_config_ptr_->al_ilqr_enable) {
      each_cost->SetConstraintConfig(&constraint_config_vec_ptr_->at(step));
    }
    result = each_cost->GetCost(x, u);
    cost += result;

    each_cost->SetCostValue(each_cost->GetCostValue() + result);
    // update cost map
#ifdef __AL_ILQR_DEBUG__
    cost_map_ptr_->at(each_cost->GetCostId())[step] = result;
#endif
  }
  return cost;
}

// Compute terminal cost at final step (horizon).
// Uses back() of config vectors for terminal step parameters.
// @param x Terminal state x_N
// @return Total terminal cost sum_i J_i(x_N, 0)
double AliLqrModel::GetTerminalCost(const State &x) {
  Control u = u_.setZero();

  double cost = 0.0;
  double result = 0.0;
  for (auto &each_cost : cost_stack_) {
    each_cost->SetConfig(&cost_config_vec_ptr_->back());
    if (solver_config_ptr_->al_ilqr_enable) {
      each_cost->SetConstraintConfig(&constraint_config_vec_ptr_->back());
    }
    result = each_cost->GetCost(x, u);
    cost += result;

    each_cost->SetCostValue(each_cost->GetCostValue() + result);
    // update cost map
#ifdef __AL_ILQR_DEBUG__
    cost_map_ptr_->at(each_cost->GetCostId())[solver_config_ptr_->horizon] =
        result;
#endif
  }
  return cost;
}

// Accumulate gradient and Hessian from all cost terms at step.
// Derivatives are accumulated (+=) into lx, lu, lxx, lxu, luu.
// When al_ilqr_enable is true, cost terms include AL penalty contributions:
//   For active inequality c + mu/rho > 0:
//     dl/dx += (mu + rho*c) * dc/dx
//     d2l/dx2 += rho * (dc/dx)(dc/dx)^T
//
// @param x State at step
// @param u Control at step
// @param step Time step index
// @param lx  [out] dJ/dx (accumulated)
// @param lu  [out] dJ/du (accumulated)
// @param lxx [out] d2J/dx2 (accumulated)
// @param lxu [out] d2J/dxdu (accumulated)
// @param luu [out] d2J/du2 (accumulated)
void AliLqrModel::GetGradientHessian(const State &x, const Control &u,
                                      const size_t &step, LxMT &lx, LuMT &lu,
                                      LxxMT &lxx, LxuMT &lxu, LuuMT &luu) {
  for (auto &each_cost : cost_stack_) {
    each_cost->SetConfig(&cost_config_vec_ptr_->at(step));
    if (solver_config_ptr_->al_ilqr_enable) {
      each_cost->SetConstraintConfig(&constraint_config_vec_ptr_->at(step));
    }
    each_cost->GetGradientHessian(x, u, lx, lu, lxx, lxu, luu);
  }
}

// Accumulate terminal gradient and Hessian from all cost terms.
// @param x Terminal state x_N
// @param lx  [out] dJ/dx (accumulated)
// @param lu  [out] dJ/du (accumulated, typically zero for terminal)
// @param lxx [out] d2J/dx2 (accumulated)
// @param lxu [out] d2J/dxdu (accumulated)
// @param luu [out] d2J/du2 (accumulated)
void AliLqrModel::GetTerminalGradientHessian(const State &x, LxMT &lx,
                                              LuMT &lu, LxxMT &lxx,
                                              LxuMT &lxu, LuuMT &luu) {
  Control u = u_.setZero();

  for (auto &each_cost : cost_stack_) {
    each_cost->SetConfig(&cost_config_vec_ptr_->back());
    if (solver_config_ptr_->al_ilqr_enable) {
      each_cost->SetConstraintConfig(&constraint_config_vec_ptr_->back());
    }
    each_cost->GetGradientHessian(x, u, lx, lu, lxx, lxu, luu);
  }
}

// Generate initial trajectory guess.
// If warm start enabled and previous solve succeeded, uses last solution.
// Otherwise, uses zero-input rollout.
// @param x0_vec [in/out] State trajectory (x0_vec[0] must be set)
// @param u0_vec [in/out] Control sequence
// @param init_cost [out] Cost of initial trajectory
void AliLqrModel::InitGuess(StateVec &x0_vec, ControlVec &u0_vec,
                             double &init_cost) {
  if (solver_config_ptr_->warm_start_enable && solver_success_) {
    // use last warm start result
    u0_vec = uk_vec_;
    init_cost = UpateDynamics(x0_vec, u0_vec);
  } else {
    // set zero
    for (size_t i = 0; i < solver_config_ptr_->horizon; i++) {
      u0_vec[i].setZero();
    }

    init_cost = UpateDynamics(x0_vec, u0_vec);
  }
}
}  // namespace al_ilqr_solver
