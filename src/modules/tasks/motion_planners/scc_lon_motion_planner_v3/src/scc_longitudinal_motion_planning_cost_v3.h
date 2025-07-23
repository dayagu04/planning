#ifndef __SCC_LONGITUDINAL_MOTION_PLANNING_COST_V3_H__
#define __SCC_LONGITUDINAL_MOTION_PLANNING_COST_V3_H__

#include "ilqr_cost.h"

namespace pnc {
namespace scc_longitudinal_planning_v3 {

enum iLqrCostconfigId {
  REF_POS,
  REF_VEL,
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
  W_NON_NEGATIVE_VEL,
  COST_CONFIG_SIZE,
};

enum iLqrCostId {
  REFERENCE_COST,
  LON_ACC_COST,
  LON_JERK_COST,
  LON_POS_SOFT_BOUND_COST,
  LON_POS_HARD_BOUND_COST,
  LON_VEL_BOUND_COST,
  LON_ACC_BOUND_COST,
  LON_JERK_BOUND_COST,
  LON_STOP_POINT_COST,
  LON_NON_NEGATIVE_VEL_COST,
  COST_SIZE,
};

enum StateId { POS = 0, VEL = 1, ACC = 2, STATE_SIZE };
enum ControlId { JERK = 0, INPUT_SIZE };

// reference cost for s and v
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

// longitudinal acc cost
class LonAccCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  LonAccCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/,
                          ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/,
                          ilqr_solver::LxxMT &lxx, ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LON_ACC_COST; }
};

// longitudinal jerk cost
class LonJerkCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  LonJerkCostTerm() = default;
  double GetCost(const ilqr_solver::State & /*x*/,
                 const ilqr_solver::Control &u) override;
  void GetGradientHessian(const ilqr_solver::State & /*x*/,
                          const ilqr_solver::Control &u,
                          ilqr_solver::LxMT & /*lx*/, ilqr_solver::LuMT &lu,
                          ilqr_solver::LxxMT & /*lxx*/,
                          ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT &luu) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LON_JERK_COST; }
};

// longitudinal pos bound cost
class LonSoftPosBoundCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  LonSoftPosBoundCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/,
                          ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/,
                          ilqr_solver::LxxMT &lxx, ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LON_POS_SOFT_BOUND_COST; }
};

class LonHardPosBoundCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  LonHardPosBoundCostTerm() = default;

  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/,
                          ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/,
                          ilqr_solver::LxxMT &lxx, ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LON_POS_HARD_BOUND_COST; }
};

// longitudinal vel bound cost
class LonVelBoundCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  LonVelBoundCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/,
                          ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/,
                          ilqr_solver::LxxMT &lxx, ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LON_VEL_BOUND_COST; }
};

// non-negative vel cost
class NonNegativeVelCost : public ilqr_solver::BaseCostTerm {
 public:
  NonNegativeVelCost() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/,
                          ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/,
                          ilqr_solver::LxxMT &lxx, ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LON_NON_NEGATIVE_VEL_COST; }
};

// longitudinal acc bound cost
class LonAccBoundCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  LonAccBoundCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/,
                          ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/,
                          ilqr_solver::LxxMT &lxx, ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LON_ACC_BOUND_COST; }
};

// longitudinal jerk bound cost
class LonJerkBoundCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  LonJerkBoundCostTerm() = default;
  double GetCost(const ilqr_solver::State & /*x*/,
                 const ilqr_solver::Control &u) override;
  void GetGradientHessian(const ilqr_solver::State & /*x*/,
                          const ilqr_solver::Control &u,
                          ilqr_solver::LxMT & /*lx*/, ilqr_solver::LuMT &lu,
                          ilqr_solver::LxxMT & /*lxx*/,
                          ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT &luu) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LON_JERK_BOUND_COST; }
};

// longitudinal stop point cost
class LonStopPointCost : public ilqr_solver::BaseCostTerm {
 public:
  LonStopPointCost() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/,
                          ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/,
                          ilqr_solver::LxxMT &lxx, ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return LON_STOP_POINT_COST; }
};

}  // namespace scc_longitudinal_planning_v3
}  // namespace pnc
#endif
