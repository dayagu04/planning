#include "continuity_cost.h"

#include "math_lib.h"

using namespace pnc::mathlib;
namespace pnc {
namespace lateral_planning {

double ContinuityCostTerm::GetCost(const ilqr_solver::State &x,
                                   const ilqr_solver::Control & /*u*/) {
  const double cost =
      0.5 * (cost_config_ptr_->at(W_CONTINUITY_X) *
                 Square(x[X] - cost_config_ptr_->at(CONTINUITY_X)) +
             cost_config_ptr_->at(W_CONTINUITY_Y) *
                 Square(x[Y] - cost_config_ptr_->at(CONTINUITY_Y)) +
             cost_config_ptr_->at(W_CONTINUITY_THETA) *
                 Square(x[THETA] - cost_config_ptr_->at(CONTINUITY_THETA)));
  cost_value_ += cost;
  return cost;
}

void ContinuityCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT & /*lxu*/, ilqr_solver::LuuMT & /*luu*/) {
  lx(X) += -cost_config_ptr_->at(W_CONTINUITY_X) *
           (cost_config_ptr_->at(CONTINUITY_X) - x[X]);
  lx(Y) += -cost_config_ptr_->at(W_CONTINUITY_Y) *
           (cost_config_ptr_->at(CONTINUITY_Y) - x[Y]);
  lx(THETA) += -cost_config_ptr_->at(W_CONTINUITY_THETA) *
               (cost_config_ptr_->at(CONTINUITY_THETA) - x[THETA]);

  lxx(X, X) += cost_config_ptr_->at(W_CONTINUITY_X);
  lxx(Y, Y) += cost_config_ptr_->at(W_CONTINUITY_Y);
  lxx(THETA, THETA) += cost_config_ptr_->at(W_CONTINUITY_THETA);
}

}  // namespace lateral_planning
}  // namespace pnc