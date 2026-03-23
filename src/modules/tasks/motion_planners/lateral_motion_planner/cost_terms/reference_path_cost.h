#pragma once

#include "ilqr_cost.h"
#include "../problem_solver/solver_define.h"

namespace pnc {
namespace lateral_planning {

class ReferencePathCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  ReferencePathCostTerm() = default;

  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control & /*u*/) override;

  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/,
                          ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/,
                          ilqr_solver::LxxMT &lxx, ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }

  uint8_t GetCostId() override { return cost_id_; }

 private:
  uint8_t cost_id_ = REFERENCE_PATH_COST;
};

class FrontReferencePathCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  FrontReferencePathCostTerm() = default;

  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control & /*u*/) override;

  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/,
                          ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/,
                          ilqr_solver::LxxMT &lxx, ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT & /*luu*/) override;

  std::string GetCostString() override { return typeid(this).name(); }

  uint8_t GetCostId() override { return cost_id_; }

 private:
  uint8_t cost_id_ = FRONT_REFERENCE_PATH_COST;
};

class VirtualReferencePathCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  VirtualReferencePathCostTerm() = default;

  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control & /*u*/) override;

  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/,
                          ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/,
                          ilqr_solver::LxxMT &lxx, ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT & /*luu*/) override;

  std::string GetCostString() override { return typeid(this).name(); }

  uint8_t GetCostId() override { return cost_id_; }

 private:
  uint8_t cost_id_ = VIRTUAL_REFERENCE_PATH_COST;
};

}  // namespace lateral_planning
}  // namespace pnc