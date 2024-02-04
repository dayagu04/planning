#include "ilqr_model.h"

#include "math_lib.h"

using namespace pnc::mathlib;

namespace ilqr_solver {
void iLqrModel::InitControlVar() {
  // u init
  u_.resize(solver_config_ptr_->input_size);
  u_.setZero();

  // uk_vec init
  uk_vec_.resize(solver_config_ptr_->horizon + 1);
  ResizeVectorElemFromEigenVec(uk_vec_, solver_config_ptr_->input_size);
  ResetVectorElemFromEigen(uk_vec_);
}

double iLqrModel::UpateDynamics(StateVec &x0, const ControlVec &u0) {
  double cost = 0.0;
  for (size_t t = 0; t < solver_config_ptr_->horizon; ++t) {
    cost += GetCost(x0[t], u0[t], t);
    x0[t + 1] = UpdateDynamicsOneStep(x0[t], u0[t], t);
  }
  cost += GetTerminalCost(x0[solver_config_ptr_->horizon]);
  return cost;
}

void iLqrModel::AddCost(std::shared_ptr<BaseCostTerm> cost_term) {
  cost_stack_.emplace_back(std::move(cost_term));
}

double iLqrModel::GetCost(const State &x, const Control &u,
                          const size_t &step) {
  double cost = 0.0;
  double result = 0.0;
  for (auto &each_cost : cost_stack_) {
    each_cost->SetConfig(&cost_config_vec_ptr_->at(step));
    each_cost->SetAliLqrConfig(&alilqr_config_vec_ptr_->at(step));
    result = each_cost->GetCost(x, u);
    cost += result;

    // update cost map
#ifdef __ILQR_DEBUG__
    cost_map_ptr_->at(each_cost->GetCostId())[step] = result;
#endif
  }
  return cost;
}

double iLqrModel::GetTerminalCost(const State &x) {
  Control u = u_.setZero();

  double cost = 0.0;
  double result = 0.0;
  for (auto &each_cost : cost_stack_) {
    each_cost->SetConfig(&cost_config_vec_ptr_->back());
    each_cost->SetAliLqrConfig(&alilqr_config_vec_ptr_->back());
    result = each_cost->GetCost(x, u);
    cost += result;

    // update cost map
#ifdef __ILQR_DEBUG__
    cost_map_ptr_->at(each_cost->GetCostId())[solver_config_ptr_->horizon] =
        result;
#endif
  }
  return cost;
}

// Updates cx, cu
void iLqrModel::GetGradientHessian(const State &x, const Control &u,
                                   const size_t &step, LxMT &lx, LuMT &lu,
                                   LxxMT &lxx, LxuMT &lxu, LuuMT &luu) {
  for (auto &each_cost : cost_stack_) {
    each_cost->SetConfig(&cost_config_vec_ptr_->at(step));
    each_cost->SetAliLqrConfig(&alilqr_config_vec_ptr_->at(step));
    each_cost->GetGradientHessian(x, u, lx, lu, lxx, lxu, luu);
  }
}

void iLqrModel::GetTerminalGradientHessian(const State &x, LxMT &lx, LuMT &lu,
                                           LxxMT &lxx, LxuMT &lxu, LuuMT &luu) {
  Control u = u_.setZero();

  for (auto &each_cost : cost_stack_) {
    each_cost->SetConfig(&cost_config_vec_ptr_->back());
    each_cost->SetAliLqrConfig(&alilqr_config_vec_ptr_->back());
    each_cost->GetGradientHessian(x, u, lx, lu, lxx, lxu, luu);
  }
}

void iLqrModel::InitGuess(StateVec &x0_vec, ControlVec &u0_vec,
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

    // TODO: UpateDynamicsZeroInput
    init_cost = UpateDynamics(x0_vec, u0_vec);
  }
}
}  // namespace ilqr_solver
