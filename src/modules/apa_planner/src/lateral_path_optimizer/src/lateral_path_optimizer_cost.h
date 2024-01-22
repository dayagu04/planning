#ifndef __LATERAL_PATH_OPTIMIZER_COST_H__
#define __LATERAL_PATH_OPTIMIZER_COST_H__

#include "ilqr_cost.h"
#include "ilqr_define.h"
namespace planning {
namespace apa_planner {

enum iLqrCostConfigId {
  REF_X,
  REF_Y,
  REF_THETA,
  TERMINAL_THETA,
  TERMINAL_Y,
  TERMINAL_X,
  K_MAX,
  U_MAX,
  W_REF_X,
  W_REF_Y,
  W_REF_THETA,
  W_TERMINAL_THETA,
  W_TERMINAL_Y,
  W_TERMINAL_X,
  W_K,
  W_U,
  W_K_BOUND,
  W_U_BOUND,
  TERMINAL_FLAG,
  MODEL_SIGN_GAIN,
  COST_CONFIG_SIZE
};

enum iLqrCostId {
  REFERENCE_COST,
  TERMINAL_COST,
  U_COST,
  K_COST,
  K_BOUND_COST,
  U_BOUND_COST,
  COST_SIZE
};

enum StateId { X = 0, Y = 1, THETA = 2, K = 3, STATE_SIZE };
enum ControlId { U = 0, INPUT_SIZE };

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

class TerminalCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  TerminalCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/,
                          ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/,
                          ilqr_solver::LxxMT &lxx, ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return TERMINAL_COST; }
};

class KCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  KCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/,
                          ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/,
                          ilqr_solver::LxxMT &lxx, ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return K_COST; }
};

class UCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  UCostTerm() = default;
  double GetCost(const ilqr_solver::State & /*x*/,
                 const ilqr_solver::Control &u) override;
  void GetGradientHessian(const ilqr_solver::State & /*x*/,
                          const ilqr_solver::Control &u,
                          ilqr_solver::LxMT & /*lx*/, ilqr_solver::LuMT &lu,
                          ilqr_solver::LxxMT & /*lxx*/,
                          ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT &luu) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return U_COST; }
};

class KBoundCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  KBoundCostTerm() = default;
  double GetCost(const ilqr_solver::State &x,
                 const ilqr_solver::Control & /*u*/) override;
  void GetGradientHessian(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/,
                          ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/,
                          ilqr_solver::LxxMT &lxx, ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT & /*luu*/) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return K_BOUND_COST; }
};

class UBoundCostTerm : public ilqr_solver::BaseCostTerm {
 public:
  UBoundCostTerm() = default;
  double GetCost(const ilqr_solver::State & /*x*/,
                 const ilqr_solver::Control &u) override;
  void GetGradientHessian(const ilqr_solver::State & /*x*/,
                          const ilqr_solver::Control &u,
                          ilqr_solver::LxMT & /*lx*/, ilqr_solver::LuMT &lu,
                          ilqr_solver::LxxMT & /*lxx*/,
                          ilqr_solver::LxuMT & /*lxu*/,
                          ilqr_solver::LuuMT &luu) override;
  std::string GetCostString() override { return typeid(this).name(); }
  uint8_t GetCostId() override { return U_BOUND_COST; }
};
}  // namespace apa_planner
}  // namespace planning

#endif