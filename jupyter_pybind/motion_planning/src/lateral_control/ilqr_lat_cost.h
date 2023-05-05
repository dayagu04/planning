#ifndef __LAT_COST_H__
#define __LAT_COST_H__

#include "ilqr_cost.h"

enum iLqrCostconfigId {
  REF_Y,
  REF_VEL,
  REF_THETA,
  CURV_FACTOR,
  W_REF_Y,
  W_REF_THETA,
  W_LAT_INPUT,
  W_WHEEL_ANGLE_RATE_BOUND,
  WHEEL_RATE_MIN,
  WHEEL_RATE_MAX,
  W_WHEEL_ANGLE_BOUND,
  WHEEL_ANGLE_MIN,
  WHEEL_ANGLE_MAX,
  TERMINAL_FLAG,
  COST_CONFIG_SIZE,
};

enum iLqrCostId : uint8_t {
  Y_REF_COST,
  THETA_REF_COST,
  DELTA_BOUND_COST,
  OMEGA_BOUND_COST,
  OMEGA_COST,
  COST_SIZE,
};

enum StateId { Y = 0, THETA = 1, DELTA = 2, STATE_SIZE };
enum ControlId { DELTA_DOT = 0, INPUT_SIZE };

// y reference cost
class yRefCostTerm : public ilqr_solver::BaseCostTerm {
public:
  yRefCostTerm() = default;
  double GetCost(const State &x, const Control & /*u*/) override;
  void GetGradientHessian(const State &x, const Control & /*u*/, LxMT &lx,
                          LuMT & /*lu*/, LxxMT &lxx, LxuMT & /*lxu*/,
                          LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return Y_REF_COST; }
};

// theta reference cost
class thetaRefCostTerm : public ilqr_solver::BaseCostTerm {
public:
  thetaRefCostTerm() = default;
  double GetCost(const State &x, const Control & /*u*/) override;
  void GetGradientHessian(const State &x, const Control & /*u*/, LxMT &lx,
                          LuMT & /*lu*/, LxxMT &lxx, LxuMT & /*lxu*/,
                          LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return THETA_REF_COST; }
};

// wheel angle bound cost
class steeringBoundCostTerm : public ilqr_solver::BaseCostTerm {
public:
  steeringBoundCostTerm() = default;
  double GetCost(const State &x, const Control & /*u*/) override;
  void GetGradientHessian(const State &x, const Control & /*u*/, LxMT &lx,
                          LuMT & /*lu*/, LxxMT &lxx, LxuMT & /*lxu*/,
                          LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return DELTA_BOUND_COST; }
};

// wheel angle rate bound cost
class steeringRateBoundCostTerm : public ilqr_solver::BaseCostTerm {
public:
  steeringRateBoundCostTerm() = default;
  double GetCost(const State & /*x*/, const Control &u) override;
  void GetGradientHessian(const State & /*x*/, const Control &u, LxMT & /*lx*/,
                          LuMT &lu, LxxMT & /*lxx*/, LxuMT & /*lxu*/,
                          LuuMT &luu) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return OMEGA_BOUND_COST; }
};

// wheel angle rate cost
class steeringRateCostTerm : public ilqr_solver::BaseCostTerm {
public:
  steeringRateCostTerm() = default;
  double GetCost(const State & /*x*/, const Control &u) override;
  void GetGradientHessian(const State & /*x*/, const Control &u, LxMT & /*lx*/,
                          LuMT &lu, LxxMT & /*lxx*/, LxuMT & /*lxu*/,
                          LuuMT &luu) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return OMEGA_COST; }
};
#endif