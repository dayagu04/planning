#ifndef __ILQR_COST_H_
#define __ILQR_COST_H_

#include <memory>
#include "ilqr_define.h"

namespace ilqr_solver {
class BaseCostTerm {
 public:
  virtual ~BaseCostTerm() = default;
  virtual double GetCost(const State &x, const Control &u) = 0;
  virtual void GetGradientHessian(const State &x, const Control &u, LxMT &lx,
                                  LuMT &lu, LxxMT &lxx, LxuMT &lxu,
                                  LuuMT &luu) = 0;
  virtual void GetDiffGradientHessian(const State &x, const Control &u,
                                      LxMT &lx, LuMT &lu, LxxMT &lxx,
                                      LxuMT &lxu, LuuMT &luu);
  virtual std::string GetCostString() = 0;
  virtual uint8_t GetCostId() = 0;
  void SetConfig(IlqrCostConfig *cost_config_ptr) {
    cost_config_ptr_ = cost_config_ptr;
  }
  IlqrCostConfig *cost_config_ptr_;

  std::shared_ptr<std::vector<IlqrCostConfig>> cost_config_vec_ptr_;
  std::shared_ptr<iLqrSolverConfig> solver_config_ptr_;
};
}  // namespace ilqr_solver

#endif
