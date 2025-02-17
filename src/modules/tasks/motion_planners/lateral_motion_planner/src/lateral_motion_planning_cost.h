#ifndef __LATERAL_MOTION_PLANNING_COST_H__
#define __LATERAL_MOTION_PLANNING_COST_H__

#include "ilqr_cost.h"
#include "vec2d.h"

namespace pnc {
namespace lateral_planning {
enum iLqrCostconfigId {
  REF_X,
  REF_Y,
  REF_THETA,
  REF_VEL,
  EGO_VEL,
  CURV_FACTOR,
  TERMINAL_FLAG,
  CONTINUITY_X,
  CONTINUITY_Y,
  CONTINUITY_THETA,
  DELTA_SOFT_BOUND,
  DELTA_HARD_BOUND,
  OMEGA_SOFT_BOUND,
  OMEGA_HARD_BOUND,
  SOFT_UPPER_BOUND_X0,
  SOFT_UPPER_BOUND_Y0,
  SOFT_UPPER_BOUND_X1,
  SOFT_UPPER_BOUND_Y1,
  SOFT_LOWER_BOUND_X0,
  SOFT_LOWER_BOUND_Y0,
  SOFT_LOWER_BOUND_X1,
  SOFT_LOWER_BOUND_Y1,
  HARD_UPPER_BOUND_X0,
  HARD_UPPER_BOUND_Y0,
  HARD_UPPER_BOUND_X1,
  HARD_UPPER_BOUND_Y1,
  HARD_LOWER_BOUND_X0,
  HARD_LOWER_BOUND_Y0,
  HARD_LOWER_BOUND_X1,
  HARD_LOWER_BOUND_Y1,
  W_REF_X,
  W_REF_Y,
  W_REF_THETA,
  W_CONTINUITY_X,
  W_CONTINUITY_Y,
  W_CONTINUITY_THETA,
  W_ACC,
  W_JERK,
  W_ACC_SOFT_BOUND,
  W_ACC_HARD_BOUND,
  W_JERK_SOFT_BOUND,
  W_JERK_HARD_BOUND,
  W_SOFT_CORRIDOR,
  W_HARD_CORRIDOR,
  W_SNAP,
  COST_CONFIG_SIZE,
};

enum iLqrCostId {
  REFERENCE_COST,
  CONTINUITY_COST,
  LAT_ACC_COST,
  LAT_JERK_COST,
  LAT_ACC_BOUND_COST,
  LAT_JERK_SOFT_BOUND_COST,
  LAT_JERK_HARD_BOUND_COST,
  PATH_SOFT_CORRIDOR_COST,
  PATH_HARD_CORRIDOR_COST,
  LAT_SNAP_COST,
  COST_SIZE,
};

enum StateId { X = 0, Y = 1, THETA = 2, DELTA = 3, STATE_SIZE };
enum ControlId { OMEGA = 0, INPUT_SIZE };

class ReferenceCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  ReferenceCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/,
                          ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/,
                          ilqr_solver::LxxMT &lxx, ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return REFERENCE_COST; }
};

class ContinuityCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  ContinuityCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/,
                          ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/,
                          ilqr_solver::LxxMT &lxx, ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return CONTINUITY_COST; }
};

class LatAccCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  LatAccCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/,
                          ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/,
                          ilqr_solver::LxxMT &lxx, ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LAT_ACC_COST; }
};

class LatJerkCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  LatJerkCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control &u) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control &u, ilqr_solver::LxMT &lx,
                          ilqr_solver::LuMT &lu, ilqr_solver::LxxMT &lxx,
                          ilqr_solver::LxuMT &lxu,
                          ilqr_solver::LuuMT &luu) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LAT_JERK_COST; }
};

class LatAccBoundCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  LatAccBoundCostTerm() = default;
  double GetCost(const ilqr_solver::State & /*x*/,
                 const ilqr_solver::Control &u) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/,
                          ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/,
                          ilqr_solver::LxxMT &lxx, ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LAT_ACC_BOUND_COST; }
};

class LatJerkSoftBoundCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  LatJerkSoftBoundCostTerm() = default;
  double GetCost(const ilqr_solver::State & /*x*/,
                 const ilqr_solver::Control &u) override;
  void GetGradientHessian(const ilqr_solver::State & /*x*/,
                          const ilqr_solver::Control &u,
                          ilqr_solver::LxMT & /*lx*/, ilqr_solver::LuMT &lu,
                          ilqr_solver::LxxMT & /*lxx*/,
                          ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT &luu) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LAT_JERK_SOFT_BOUND_COST; }
};

class LatJerkHardBoundCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  LatJerkHardBoundCostTerm() = default;
  double GetCost(const ilqr_solver::State & /*x*/,
                 const ilqr_solver::Control &u) override;
  void GetGradientHessian(const ilqr_solver::State & /*x*/,
                          const ilqr_solver::Control &u,
                          ilqr_solver::LxMT & /*lx*/, ilqr_solver::LuMT &lu,
                          ilqr_solver::LxxMT & /*lxx*/,
                          ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT &luu) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LAT_JERK_HARD_BOUND_COST; }
};

class PathSoftCorridorCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  PathSoftCorridorCostTerm() = default;
  double GetCost(const ilqr_solver::State & /*x*/,
                 const ilqr_solver::Control &u) override;
  void GetGradientHessian(const ilqr_solver::State & /*x*/,
                          const ilqr_solver::Control &u,
                          ilqr_solver::LxMT & /*lx*/, ilqr_solver::LuMT &lu,
                          ilqr_solver::LxxMT & /*lxx*/,
                          ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT &luu) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return PATH_SOFT_CORRIDOR_COST; }

 private:
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
  uint8_t GetCostId() override { return PATH_HARD_CORRIDOR_COST; }

 private:
  planning::planning_math::Vec2d ubound_direction_;
  planning::planning_math::Vec2d lbound_direction_;
};
}  // namespace lateral_planning
}  // namespace pnc
#endif
