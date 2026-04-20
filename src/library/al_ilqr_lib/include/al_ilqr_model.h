#ifndef __AL_ILQR_MODEL_H_
#define __AL_ILQR_MODEL_H_

#include <memory>

#include "al_ilqr_cost.h"

namespace al_ilqr_solver {
class AliLqrModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AliLqrModel() {
    // instantiation for shared ptrs
    cost_config_vec_ptr_ = std::make_shared<std::vector<AliLqrCostConfig>>();
    solver_config_ptr_ = std::make_shared<AliLqrSolverConfig>();
    // constraint config for each horizon
    constraint_config_vec_ptr_ =
        std::make_shared<std::vector<AliLqrConstraintConfig>>();
    // cost stack init
    cost_stack_.clear();
    cost_stack_.reserve(MAX_COST_SIZE);
  }

  virtual ~AliLqrModel() = default;

  // Compute one-step dynamics: x_{k+1} = f(x_k, u_k).
  // @param x Current state
  // @param u Current control
  // @param step Current time step index
  // @return Next state x_{k+1}
  virtual State UpdateDynamicsOneStep(const State &x, const Control &u,
                                      const size_t &step) const = 0;

  // Compute linearized dynamics Jacobians: f_x = df/dx, f_u = df/du.
  // @param x Current state
  // @param u Current control
  // @param f_x [out] State Jacobian  (n x n)
  // @param f_u [out] Control Jacobian (n x m)
  // @param step Current time step index
  virtual void GetDynamicsDerivatives(const State &x, const Control & /*u*/,
                                      FxMT &f_x, FuMT &f_u,
                                      const size_t &step) const = 0;

  void InitGuess(StateVec &x0, ControlVec &u0, double &init_cost);

  // Reset model warm start when switching scenario
  void Reset() { solver_success_ = false; }

  double GetCost(const State &x, const Control &u, const size_t &step);

  double GetTerminalCost(const State &x);

  void GetGradientHessian(const State &x, const Control &u, const size_t &step,
                          LxMT &lx, LuMT &lu, LxxMT &lxx, LxuMT &lxu,
                          LuuMT &luu);

  void GetTerminalGradientHessian(const State &x, LxMT &lx, LuMT &lu,
                                  LxxMT &lxx, LxuMT &lxu, LuuMT &luu);

  void AddCost(std::shared_ptr<AliLqrBaseCostTerm> cost_term);

  void ClearCost();

  double UpateDynamics(StateVec &x0, const ControlVec &u0);

  // init control u and u_vec
  void InitControlVar();

  void UpdateWarmStart(const bool solver_success, const ControlVec &uk) {
    solver_success_ = solver_success;
    uk_vec_ = uk;
  }

  void SetCostConfig(const std::vector<AliLqrCostConfig> &cost_config) {
    *cost_config_vec_ptr_ = cost_config;
  }

  void SetConstraintConfig(
      const std::vector<AliLqrConstraintConfig> &constraint_config) {
    *constraint_config_vec_ptr_ = constraint_config;
  }

  std::shared_ptr<std::vector<AliLqrCostConfig>> GetCostConfigPtr() {
    return cost_config_vec_ptr_;
  }

  std::shared_ptr<std::vector<AliLqrConstraintConfig>>
  GetConstraintConfigPtr() {
    return constraint_config_vec_ptr_;
  }

  std::shared_ptr<AliLqrSolverConfig> GetSolverConfigPtr() {
    return solver_config_ptr_;
  }

  void SetCostMapPtr(AliLqrCostMap *cost_map_ptr) {
    cost_map_ptr_ = cost_map_ptr;
  }

  std::vector<std::shared_ptr<AliLqrBaseCostTerm>> *GetCostStackPtr() {
    return &cost_stack_;
  }

 protected:
  bool solver_success_ = false;

  // should be inited by core
  ControlVec uk_vec_;
  Control u_;

  // al-ilqr cost stack to restore all the costs
  std::vector<std::shared_ptr<AliLqrBaseCostTerm>> cost_stack_;

  // cost config for each horizon
  std::shared_ptr<std::vector<AliLqrCostConfig>> cost_config_vec_ptr_;

  // constraint config for each horizon:
  //   stores Lagrange multiplier (mu) and penalty (rho) per constraint
  std::shared_ptr<std::vector<AliLqrConstraintConfig>>
      constraint_config_vec_ptr_;
  // solver config
  std::shared_ptr<AliLqrSolverConfig> solver_config_ptr_;

  // cost map: total cost of each cost term
  AliLqrCostMap *cost_map_ptr_;
};
}  // namespace al_ilqr_solver

#endif  // __AL_ILQR_MODEL_H_
