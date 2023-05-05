#ifndef __LATERAL_MOTION_PLANNING_COST_H__
#define __LATERAL_MOTION_PLANNING_COST_H__

#include "ilqr_cost.h"

namespace pnc {
namespace lateral_planning {
enum iLqrCostconfigId {
  REF_X,
  REF_Y,
  REF_THETA,
  REF_VEL,
  CURV_FACTOR,
  TERMINAL_FLAG,
  CONTINUITY_X,
  CONTINUITY_Y,
  CONTINUITY_THETA,
  DELTA_BOUND,
  OMEGA_BOUND,
  SOFT_UPPER_BOUND_X0,
  SOFT_UPPER_BOUND_Y0,
  SOFT_UPPER_BOUND_X1,
  SOFT_UPPER_BOUND_Y1,
  SOFT_LOWER_BOUND_X0,
  SOFT_LOWER_BOUND_Y0,
  SOFT_LOWER_BOUND_X1,
  SOFT_LOWER_BOUND_Y1,
  W_REF_X,
  W_REF_Y,
  W_REF_THETA,
  W_CONTINUITY_X,
  W_CONTINUITY_Y,
  W_CONTINUITY_THETA,
  W_ACC,
  W_JERK,
  W_ACC_BOUND,
  W_JERK_BOUND,
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
  LAT_JERK_BOUND_COST,
  PATH_SOFT_CORRIDOR_COST,
  LAT_SNAP_COST,
  COST_SIZE,
};

enum StateId { X = 0, Y = 1, THETA = 2, DELTA = 3, OMEGA = 4, STATE_SIZE };
enum ControlId { OMEGA_DOT = 0, INPUT_SIZE };

class ReferenceCostTerm : public ilqr_solver::BaseCostTerm {
public:
  ReferenceCostTerm() = default;
  double GetCost(const State &x, const Control & /*u*/) override;
  void GetGradientHessian(const State &x, const Control & /*u*/, LxMT &lx,
                          LuMT & /*lu*/, LxxMT &lxx, LxuMT & /*lxu*/,
                          LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return REFERENCE_COST; }
};

class ContinuityCostTerm : public ilqr_solver::BaseCostTerm {
public:
  ContinuityCostTerm() = default;
  double GetCost(const State &x, const Control & /*u*/) override;
  void GetGradientHessian(const State &x, const Control & /*u*/, LxMT &lx,
                          LuMT & /*lu*/, LxxMT &lxx, LxuMT & /*lxu*/,
                          LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return CONTINUITY_COST; }
};

class LatAccCostTerm : public ilqr_solver::BaseCostTerm {
public:
  LatAccCostTerm() = default;
  double GetCost(const State &x, const Control & /*u*/) override;
  void GetGradientHessian(const State &x, const Control & /*u*/, LxMT &lx,
                          LuMT & /*lu*/, LxxMT &lxx, LxuMT & /*lxu*/,
                          LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LAT_ACC_COST; }
};

class LatJerkCostTerm : public ilqr_solver::BaseCostTerm {
public:
  LatJerkCostTerm() = default;
  double GetCost(const State &x, const Control &u) override;
  void GetGradientHessian(const State &x, const Control &u, LxMT &lx, LuMT &lu,
                          LxxMT &lxx, LxuMT &lxu, LuuMT &luu) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LAT_JERK_COST; }
};

class LatAccBoundCostTerm : public ilqr_solver::BaseCostTerm {
public:
  LatAccBoundCostTerm() = default;
  double GetCost(const State & /*x*/, const Control &u) override;
  void GetGradientHessian(const State &x, const Control & /*u*/, LxMT &lx,
                          LuMT & /*lu*/, LxxMT &lxx, LxuMT & /*lxu*/,
                          LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LAT_ACC_BOUND_COST; }
};

class LatJerkBoundCostTerm : public ilqr_solver::BaseCostTerm {
public:
  LatJerkBoundCostTerm() = default;
  double GetCost(const State & /*x*/, const Control &u) override;
  void GetGradientHessian(const State & /*x*/, const Control &u, LxMT & /*lx*/,
                          LuMT &lu, LxxMT & /*lxx*/, LxuMT & /*lxu*/,
                          LuuMT &luu) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LAT_JERK_BOUND_COST; }
};

class PathSoftCorridorCostTerm : public ilqr_solver::BaseCostTerm {
public:
  PathSoftCorridorCostTerm() = default;
  double GetCost(const State & /*x*/, const Control &u) override;
  void GetGradientHessian(const State & /*x*/, const Control &u, LxMT & /*lx*/,
                          LuMT &lu, LxxMT & /*lxx*/, LxuMT & /*lxu*/,
                          LuuMT &luu) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return PATH_SOFT_CORRIDOR_COST; }
};

class LatSnapCostTerm : public ilqr_solver::BaseCostTerm {
public:
  LatSnapCostTerm() = default;
  double GetCost(const State & /*x*/, const Control &u) override;
  void GetGradientHessian(const State & /*x*/, const Control &u, LxMT & /*lx*/,
                          LuMT &lu, LxxMT & /*lxx*/, LxuMT & /*lxu*/,
                          LuuMT &luu) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LAT_SNAP_COST; }
};
} // namespace lateral_planning
} // namespace pnc
#endif
