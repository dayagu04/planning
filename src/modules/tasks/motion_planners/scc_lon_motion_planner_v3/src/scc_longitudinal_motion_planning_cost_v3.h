#ifndef __SCC_LONGITUDINAL_MOTION_PLANNING_COST_V3_H__
#define __SCC_LONGITUDINAL_MOTION_PLANNING_COST_V3_H__

#include "al_ilqr_cost.h"

namespace pnc {
namespace scc_longitudinal_planning_v3 {

enum iLqrCostconfigId {
  REF_POS,
  REF_VEL,
  REF_ACC,
  SOFT_POS_MAX,
  SOFT_POS_MIN,
  HARD_POS_MAX,
  HARD_POS_MIN,
  VEL_MAX,
  VEL_MIN,
  ACC_MAX,
  ACC_MIN,
  JERK_MAX,
  JERK_MIN,
  S_STOP,
  W_REF_POS,
  W_REF_VEL,
  W_REF_ACC,
  W_ACC,
  W_JERK,
  W_SNAP,
  W_POS_BOUND,
  W_HARD_POS_BOUND,
  W_VEL_BOUND,
  W_ACC_BOUND,
  W_JERK_BOUND,
  W_S_STOP,
  TERMINAL_FLAG,
  SAFE_DISTANCE,
  W_EMERGENCY_STOP,
  EXTEND_POS_MAX,
  EXTEND_POS_MIN,
  W_EXTEND_POS_BOUND,
  COST_CONFIG_SIZE,
};

enum ConstraintConfigId {
  MU_HARD_POS_UPPER = 0,
  RHO_HARD_POS_UPPER = 1,
  MU_HARD_POS_LOWER = 2,
  RHO_HARD_POS_LOWER = 3,
  MU_HARD_VEL_LOWER = 4,
  RHO_HARD_VEL_LOWER = 5,
  CONSTRAINT_CONFIG_SIZE,
};

enum iLqrCostId {
  REFERENCE_COST,
  LON_ACC_COST,
  LON_JERK_COST,
  LON_POS_SOFT_BOUND_COST,
  LON_POS_HARD_BOUND_COST,
  LON_POS_EXTEND_BOUND_COST,
  LON_ACC_BOUND_COST,
  LON_VEL_BOUND_COST,
  LON_JERK_BOUND_COST,
  LON_NON_NEGATIVE_VEL_COST,
  LON_EMERGENCY_STOP_COST,
  COST_SIZE,
};

enum StateId { POS = 0, VEL = 1, ACC = 2, STATE_SIZE };
enum ControlId { JERK = 0, INPUT_SIZE };

// reference cost for s and v
class ReferenceCostTerm : public al_ilqr_solver::AliLqrBaseCostTerm {
 public:
  ReferenceCostTerm() = default;
  double GetCost(const al_ilqr_solver::State &x,
                 const al_ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const al_ilqr_solver::State &x,
                          const al_ilqr_solver::Control & /*u*/,
                          al_ilqr_solver::LxMT &lx,
                          al_ilqr_solver::LuMT & /*lu*/,
                          al_ilqr_solver::LxxMT &lxx,
                          al_ilqr_solver::LxuMT & /*lxu*/,
                          al_ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return REFERENCE_COST; }
};

// longitudinal acc cost
class LonAccCostTerm : public al_ilqr_solver::AliLqrBaseCostTerm {
 public:
  LonAccCostTerm() = default;
  double GetCost(const al_ilqr_solver::State &x,
                 const al_ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const al_ilqr_solver::State &x,
                          const al_ilqr_solver::Control & /*u*/,
                          al_ilqr_solver::LxMT &lx,
                          al_ilqr_solver::LuMT & /*lu*/,
                          al_ilqr_solver::LxxMT &lxx,
                          al_ilqr_solver::LxuMT & /*lxu*/,
                          al_ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LON_ACC_COST; }
};

// longitudinal jerk cost
class LonJerkCostTerm : public al_ilqr_solver::AliLqrBaseCostTerm {
 public:
  LonJerkCostTerm() = default;
  double GetCost(const al_ilqr_solver::State & /*x*/,
                 const al_ilqr_solver::Control &u) override;
  void GetGradientHessian(const al_ilqr_solver::State & /*x*/,
                          const al_ilqr_solver::Control &u,
                          al_ilqr_solver::LxMT & /*lx*/,
                          al_ilqr_solver::LuMT &lu,
                          al_ilqr_solver::LxxMT & /*lxx*/,
                          al_ilqr_solver::LxuMT & /*lxu*/,
                          al_ilqr_solver::LuuMT &luu) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LON_JERK_COST; }
};

// longitudinal pos bound cost
class LonSoftPosBoundCostTerm : public al_ilqr_solver::AliLqrBaseCostTerm {
 public:
  LonSoftPosBoundCostTerm() = default;
  double GetCost(const al_ilqr_solver::State &x,
                 const al_ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const al_ilqr_solver::State &x,
                          const al_ilqr_solver::Control & /*u*/,
                          al_ilqr_solver::LxMT &lx,
                          al_ilqr_solver::LuMT & /*lu*/,
                          al_ilqr_solver::LxxMT &lxx,
                          al_ilqr_solver::LxuMT & /*lxu*/,
                          al_ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LON_POS_SOFT_BOUND_COST; }
};

class LonHardPosBoundCostTerm : public al_ilqr_solver::AliLqrBaseCostTerm {
 public:
  LonHardPosBoundCostTerm() = default;

  double GetCost(const al_ilqr_solver::State &x,
                 const al_ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const al_ilqr_solver::State &x,
                          const al_ilqr_solver::Control & /*u*/,
                          al_ilqr_solver::LxMT &lx,
                          al_ilqr_solver::LuMT & /*lu*/,
                          al_ilqr_solver::LxxMT &lxx,
                          al_ilqr_solver::LxuMT & /*lxu*/,
                          al_ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LON_POS_HARD_BOUND_COST; }
};

class LonExtendPosBoundCostTerm : public al_ilqr_solver::AliLqrBaseCostTerm {
 public:
  LonExtendPosBoundCostTerm() = default;

  double GetCost(const al_ilqr_solver::State &x,
                 const al_ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const al_ilqr_solver::State &x,
                          const al_ilqr_solver::Control & /*u*/,
                          al_ilqr_solver::LxMT &lx,
                          al_ilqr_solver::LuMT & /*lu*/,
                          al_ilqr_solver::LxxMT &lxx,
                          al_ilqr_solver::LxuMT & /*lxu*/,
                          al_ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LON_POS_EXTEND_BOUND_COST; }
};

// longitudinal vel bound cost
class LonVelBoundCostTerm : public al_ilqr_solver::AliLqrBaseCostTerm {
 public:
  LonVelBoundCostTerm() = default;
  double GetCost(const al_ilqr_solver::State &x,
                 const al_ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const al_ilqr_solver::State &x,
                          const al_ilqr_solver::Control & /*u*/,
                          al_ilqr_solver::LxMT &lx,
                          al_ilqr_solver::LuMT & /*lu*/,
                          al_ilqr_solver::LxxMT &lxx,
                          al_ilqr_solver::LxuMT & /*lxu*/,
                          al_ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LON_VEL_BOUND_COST; }
};

// Non-negative vel hard constraint
class LonHardNonNegativeVelCostTerm
    : public al_ilqr_solver::AliLqrBaseCostTerm {
 public:
  LonHardNonNegativeVelCostTerm() = default;
  double GetCost(const al_ilqr_solver::State &x,
                 const al_ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const al_ilqr_solver::State &x,
                          const al_ilqr_solver::Control & /*u*/,
                          al_ilqr_solver::LxMT &lx,
                          al_ilqr_solver::LuMT & /*lu*/,
                          al_ilqr_solver::LxxMT &lxx,
                          al_ilqr_solver::LxuMT & /*lxu*/,
                          al_ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LON_NON_NEGATIVE_VEL_COST; }
};

// Longitudinal acc bound cost
class LonAccBoundCostTerm : public al_ilqr_solver::AliLqrBaseCostTerm {
 public:
  LonAccBoundCostTerm() = default;
  double GetCost(const al_ilqr_solver::State &x,
                 const al_ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const al_ilqr_solver::State &x,
                          const al_ilqr_solver::Control & /*u*/,
                          al_ilqr_solver::LxMT &lx,
                          al_ilqr_solver::LuMT & /*lu*/,
                          al_ilqr_solver::LxxMT &lxx,
                          al_ilqr_solver::LxuMT & /*lxu*/,
                          al_ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LON_ACC_BOUND_COST; }
};

// longitudinal jerk bound cost
class LonJerkBoundCostTerm : public al_ilqr_solver::AliLqrBaseCostTerm {
 public:
  LonJerkBoundCostTerm() = default;
  double GetCost(const al_ilqr_solver::State & /*x*/,
                 const al_ilqr_solver::Control &u) override;
  void GetGradientHessian(const al_ilqr_solver::State & /*x*/,
                          const al_ilqr_solver::Control &u,
                          al_ilqr_solver::LxMT & /*lx*/,
                          al_ilqr_solver::LuMT &lu,
                          al_ilqr_solver::LxxMT & /*lxx*/,
                          al_ilqr_solver::LxuMT & /*lxu*/,
                          al_ilqr_solver::LuuMT &luu) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LON_JERK_BOUND_COST; }
};

class LonEmergencyStopCostTerm : public al_ilqr_solver::AliLqrBaseCostTerm {
 public:
  LonEmergencyStopCostTerm() = default;
  double GetCost(const al_ilqr_solver::State &x,
                 const al_ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const al_ilqr_solver::State &x,
                          const al_ilqr_solver::Control & /*u*/,
                          al_ilqr_solver::LxMT &lx,
                          al_ilqr_solver::LuMT & /*lu*/,
                          al_ilqr_solver::LxxMT &lxx,
                          al_ilqr_solver::LxuMT & /*lxu*/,
                          al_ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LON_EMERGENCY_STOP_COST; }
};

}  // namespace scc_longitudinal_planning_v3
}  // namespace pnc
#endif
