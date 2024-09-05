#ifndef __ILQR_MODEL_H_
#define __ILQR_MODEL_H_

#include <memory>

#include "ilqr_cost.h"

namespace ilqr_solver {
class iLqrModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  iLqrModel() {
    // instantiation for shared ptrs
    cost_config_vec_ptr_ = std::make_shared<std::vector<IlqrCostConfig>>();
    solver_config_ptr_ = std::make_shared<iLqrSolverConfig>();
    // alilqr config for each horizon
    alilqr_config_vec_ptr_ = std::make_shared<std::vector<AliLqrConfig>>();
    // cost stack init
    cost_stack_.clear();
    cost_stack_.reserve(MAX_COST_SIZE);
  }

  virtual ~iLqrModel() = default;

  virtual State UpdateDynamicsOneStep(const State &x, const Control &u,
                                      const size_t &step) const = 0;

  virtual void GetDynamicsDerivatives(const State &x, const Control & /*u*/,
                                      FxMT &f_x, FuMT &f_u,
                                      const size_t &step) const = 0;

  void InitGuess(StateVec &x0, ControlVec &u0, double &init_cost);

  // reset model warm start when switching
  void Reset() { solver_success_ = false; }

  double GetCost(const State &x, const Control &u, const size_t &step);
  double GetTerminalCost(const State &x);

  void GetGradientHessian(const State &x, const Control &u, const size_t &step,
                          LxMT &lx, LuMT &lu, LxxMT &lxx, LxuMT &lxu,
                          LuuMT &luu);

  void GetTerminalGradientHessian(const State &x, LxMT &lx, LuMT &lu,
                                  LxxMT &lxx, LxuMT &lxu, LuuMT &luu);

  void AddCost(std::shared_ptr<BaseCostTerm> cost_term);
  double UpateDynamics(StateVec &x0, const ControlVec &u0);

  // init control u and u_vec
  void InitControlVar();

  void UpdateWarmStart(const bool &solver_success, const ControlVec &uk) {
    solver_success_ = solver_success;
    uk_vec_ = uk;
  }

  void SetCostConfig(const std::vector<IlqrCostConfig> &cost_config) {
    *cost_config_vec_ptr_ = cost_config;
  }

  void SetAliLqrConfig(const std::vector<AliLqrConfig> &alilqr_config) {
    *alilqr_config_vec_ptr_ = alilqr_config;
  }

  std::shared_ptr<std::vector<IlqrCostConfig>> GetiLqrCostConfigPtr() {
    return cost_config_vec_ptr_;
  }

  std::shared_ptr<std::vector<AliLqrConfig>> GetAliLqrConfigPtr() {
    return alilqr_config_vec_ptr_;
  }

  std::shared_ptr<iLqrSolverConfig> GetSolverConfigPtr() {
    return solver_config_ptr_;
  }

  void SetCostMapPtr(ILqrCostMap *cost_map_ptr) {
    cost_map_ptr_ = cost_map_ptr;
  }

  std::vector<std::shared_ptr<BaseCostTerm>> *GetCostStackPtr() {
    return &cost_stack_;
  }

 protected:
  bool solver_success_ = false;

  // should be inited by core
  ControlVec uk_vec_;
  Control u_;

  // ilqr cost stack to restore all the costs
  std::vector<std::shared_ptr<BaseCostTerm>> cost_stack_;

  // cost config for each horizon
  std::shared_ptr<std::vector<IlqrCostConfig>> cost_config_vec_ptr_;

  // alilqr config for each horizon
  std::shared_ptr<std::vector<AliLqrConfig>> alilqr_config_vec_ptr_;
  // solver config
  std::shared_ptr<iLqrSolverConfig> solver_config_ptr_;

  // cost map: total cost of each cost term
  ILqrCostMap *cost_map_ptr_;
};
}  // namespace ilqr_solver

#endif
