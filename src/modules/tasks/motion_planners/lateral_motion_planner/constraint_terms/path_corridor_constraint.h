#pragma once

#include "vec2d.h"

#include "ilqr_cost.h"
#include "../problem_solver/solver_define.h"

namespace pnc {
namespace lateral_planning {

class PathFirstSoftCorridorCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  PathFirstSoftCorridorCostTerm() = default;

  double GetCost(const ilqr_solver::State & /*x*/,
                 const ilqr_solver::Control &u) override;

  void GetGradientHessian(const ilqr_solver::State & /*x*/,
                          const ilqr_solver::Control &u,
                          ilqr_solver::LxMT & /*lx*/, ilqr_solver::LuMT &lu,
                          ilqr_solver::LxxMT & /*lxx*/,
                          ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT &luu) override;

  std::string GetCostString() override { return typeid(this).name(); }

  uint8_t GetCostId() override { return cost_id_; }

 private:
  uint8_t cost_id_ = PATH_FIRST_SOFT_CORRIDOR_COST;
  planning::planning_math::Vec2d ubound_direction_;
  planning::planning_math::Vec2d lbound_direction_;
};

class PathSecondSoftCorridorCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  PathSecondSoftCorridorCostTerm() = default;

  double GetCost(const ilqr_solver::State & /*x*/,
                 const ilqr_solver::Control &u) override;

  void GetGradientHessian(const ilqr_solver::State & /*x*/,
                          const ilqr_solver::Control &u,
                          ilqr_solver::LxMT & /*lx*/, ilqr_solver::LuMT &lu,
                          ilqr_solver::LxxMT & /*lxx*/,
                          ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT &luu) override;

  std::string GetCostString() override { return typeid(this).name(); }

  uint8_t GetCostId() override { return cost_id_; }

 private:
  uint8_t cost_id_ = PATH_SECOND_SOFT_CORRIDOR_COST;
  planning::planning_math::Vec2d ubound_direction_;
  planning::planning_math::Vec2d lbound_direction_;
};

class PathHardCorridorCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  PathHardCorridorCostTerm() = default;

  double GetCost(const ilqr_solver::State & /*x*/,
                 const ilqr_solver::Control &u) override;

  void GetGradientHessian(const ilqr_solver::State & /*x*/,
                          const ilqr_solver::Control &u,
                          ilqr_solver::LxMT & /*lx*/, ilqr_solver::LuMT &lu,
                          ilqr_solver::LxxMT & /*lxx*/,
                          ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT &luu) override;

  std::string GetCostString() override { return typeid(this).name(); }

  uint8_t GetCostId() override { return cost_id_; }

 private:
  uint8_t cost_id_ = PATH_HARD_CORRIDOR_COST;
  planning::planning_math::Vec2d ubound_direction_;
  planning::planning_math::Vec2d lbound_direction_;
};

class PathCorridorCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  PathCorridorCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control &) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/,
                          ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/,
                          ilqr_solver::LxxMT &lxx, ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return cost_id_; };

 private:
  uint8_t cost_id_ = PATH_CORRIDOR_COST;
  planning::planning_math::Vec2d ubound_direction_;
  planning::planning_math::Vec2d lbound_direction_;
};

}  // namespace lateral_planning
}  // namespace pnc