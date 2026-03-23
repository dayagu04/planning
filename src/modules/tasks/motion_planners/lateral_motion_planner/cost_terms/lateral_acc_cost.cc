#include "lateral_acc_cost.h"

#include "math_lib.h"

using namespace pnc::mathlib;
namespace pnc {
namespace lateral_planning {

double LateralAccCostTerm::GetCost(const ilqr_solver::State &x,
                                   const ilqr_solver::Control & /*u*/) {
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR) *
                             Square(cost_config_ptr_->at(REF_VEL)));
  const double delta = x[DELTA] - cost_config_ptr_->at(EXPECTEDE_DELTA);

  const double cost =
      0.5 * cost_config_ptr_->at(W_ACC) * k2v4 * (delta * delta);
  cost_value_ += cost;
  return cost;
}

void LateralAccCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT & /*lxu*/, ilqr_solver::LuuMT & /*luu*/) {
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR) *
                             Square(cost_config_ptr_->at(REF_VEL)));
  const double delta = x[DELTA] - cost_config_ptr_->at(EXPECTEDE_DELTA);

  lx(DELTA) += cost_config_ptr_->at(W_ACC) * k2v4 * delta;
  lxx(DELTA, DELTA) += cost_config_ptr_->at(W_ACC) * k2v4;
}


}  // namespace lateral_planning
}  // namespace pnc