#ifndef __AL_ILQR_COST_H_
#define __AL_ILQR_COST_H_

#include <memory>

#include "al_ilqr_define.h"

namespace al_ilqr_solver {
class AliLqrBaseCostTerm {
 public:
  virtual ~AliLqrBaseCostTerm() = default;

  // Compute scalar cost value at state x with control u.
  // @param x Current state vector
  // @param u Current control vector
  // @return Scalar cost value J(x, u)
  virtual double GetCost(const State &x, const Control &u) = 0;

  // Compute analytical gradient and Hessian of the cost.
  // Results are accumulated (+=) into the output matrices.
  // @param x Current state vector
  // @param u Current control vector
  // @param lx  [out] Gradient w.r.t. state:   dJ/dx
  // @param lu  [out] Gradient w.r.t. control:  dJ/du
  // @param lxx [out] Hessian  w.r.t. state:    d²J/dx²
  // @param lxu [out] Cross Hessian:            d²J/(dx du)
  // @param luu [out] Hessian  w.r.t. control:  d²J/du²
  virtual void GetGradientHessian(const State &x, const Control &u, LxMT &lx,
                                  LuMT &lu, LxxMT &lxx, LxuMT &lxu,
                                  LuuMT &luu) = 0;

  // Compute numerical gradient and Hessian via central finite difference.
  // Used for verification or when analytical derivatives are unavailable.
  virtual void GetDiffGradientHessian(const State &x, const Control &u,
                                      LxMT &lx, LuMT &lu, LxxMT &lxx,
                                      LxuMT &lxu, LuuMT &luu);

  // Return human-readable name of this cost term for logging.
  virtual std::string GetCostString() = 0;

  // Return unique integer id of this cost term for indexing cost maps.
  virtual uint8_t GetCostId() = 0;

  void ResetCostValue() { cost_value_ = 0.0; }

  void SetCostValue(double cost_value) {
    cost_value_ = cost_value;
  }

  const double GetCostValue() const { return cost_value_; }

  void SetConfig(AliLqrCostConfig *cost_config_ptr) {
    cost_config_ptr_ = cost_config_ptr;
  }

  void SetConstraintConfig(AliLqrConstraintConfig *constraint_config_ptr) {
    constraint_config_ptr_ = constraint_config_ptr;
  }

  AliLqrCostConfig *cost_config_ptr_;
  std::shared_ptr<std::vector<AliLqrCostConfig>> cost_config_vec_ptr_;

  AliLqrConstraintConfig *constraint_config_ptr_;
  std::shared_ptr<std::vector<AliLqrConstraintConfig>> constraint_config_vec_ptr_;

  std::shared_ptr<AliLqrSolverConfig> solver_config_ptr_;

 protected:
  double cost_value_ = 0.0;
};
}  // namespace al_ilqr_solver

#endif  // __AL_ILQR_COST_H_
