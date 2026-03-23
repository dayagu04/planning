#pragma once

#include "ilqr_cost.h"
#include "../problem_solver/solver_define.h"

namespace pnc {
namespace lateral_planning {

class LateralAccBoundCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  LateralAccBoundCostTerm() = default;

  double GetCost(const ilqr_solver::State & /*x*/,
                 const ilqr_solver::Control &u) override;

  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/,
                          ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/,
                          ilqr_solver::LxxMT &lxx, ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT & /*luu*/) override;

  std::string GetCostString() override { return typeid(this).name(); }

  uint8_t GetCostId() override { return cost_id_; }

 private:
  uint8_t cost_id_ = LATERAL_ACC_BOUND_COST;
};

class LateralAccHardBoundCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  LateralAccHardBoundCostTerm() = default;
  double GetCost(const ilqr_solver::State & /*x*/,
                 const ilqr_solver::Control &u) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/,
                          ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/,
                          ilqr_solver::LxxMT &lxx, ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return cost_id_; }

 private:
  uint8_t cost_id_ = LATERAL_ACC_HARD_BOUND_COST;
};

}  // namespace lateral_planning
}  // namespace pnc