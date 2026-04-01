#include "lateral_acc_constraint.h"

#include "math_lib.h"

using namespace pnc::mathlib;
namespace pnc {
namespace lateral_planning {

double LateralAccBoundCostTerm::GetCost(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/) {
  double cost = 0.;
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR)) *
                      SSquare(cost_config_ptr_->at(REF_VEL));

  if (x[DELTA] > cost_config_ptr_->at(DELTA_UPPER_BOUND)) {
    cost = 0.5 * cost_config_ptr_->at(W_ACC_BOUND) * k2v4 *
           Square(x[DELTA] - cost_config_ptr_->at(DELTA_UPPER_BOUND));
  } else if (x[DELTA] < cost_config_ptr_->at(DELTA_LOWER_BOUND)) {
    cost = 0.5 * cost_config_ptr_->at(W_ACC_BOUND) * k2v4 *
           Square(x[DELTA] - cost_config_ptr_->at(DELTA_LOWER_BOUND));
  }
  cost_value_ += cost;
  return cost;
}

void LateralAccBoundCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT & /*lxu*/, ilqr_solver::LuuMT & /*luu*/) {
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR)) *
                      SSquare(cost_config_ptr_->at(REF_VEL));

  if (x[DELTA] > cost_config_ptr_->at(DELTA_UPPER_BOUND)) {
    lx(DELTA) += cost_config_ptr_->at(W_ACC_BOUND) * k2v4 *
                 (x[DELTA] - cost_config_ptr_->at(DELTA_UPPER_BOUND));
    lxx(DELTA, DELTA) += cost_config_ptr_->at(W_ACC_BOUND) * k2v4;
  } else if (x[DELTA] < cost_config_ptr_->at(DELTA_LOWER_BOUND)) {
    lx(DELTA) += cost_config_ptr_->at(W_ACC_BOUND) * k2v4 *
                 (x[DELTA] - cost_config_ptr_->at(DELTA_LOWER_BOUND));
    lxx(DELTA, DELTA) += cost_config_ptr_->at(W_ACC_BOUND) * k2v4;
  }
}

double LateralAccHardBoundCostTerm::GetCost(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/) {
  double cost = 0.;
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR)) *
                      SSquare(cost_config_ptr_->at(REF_VEL));
  const double cost_var =
    k2v4 * (Square(x[DELTA]) - Square(cost_config_ptr_->at(DELTA_UPPER_BOUND))) +
    alilqr_config_ptr_->at(L_ACC_HARDBOUND) /
    alilqr_config_ptr_->at(W_ACC_HARDBOUND);
  if (cost_var > 0.) {
    cost += 0.5 * alilqr_config_ptr_->at(W_ACC_HARDBOUND) * Square(cost_var);
  }
  cost_value_ += cost;
  return cost;
}

void LateralAccHardBoundCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT & /*lxu*/, ilqr_solver::LuuMT & /*luu*/) {
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR)) *
                      SSquare(cost_config_ptr_->at(REF_VEL));
  const double cost_var =
    k2v4 * (Square(x[DELTA]) - Square(cost_config_ptr_->at(DELTA_UPPER_BOUND))) +
    alilqr_config_ptr_->at(L_ACC_HARDBOUND) /
    alilqr_config_ptr_->at(W_ACC_HARDBOUND);
  if (cost_var > 0.) {
    lx(DELTA) += alilqr_config_ptr_->at(W_ACC_HARDBOUND) * cost_var * 2.0 * k2v4 * x[DELTA];
    lxx(DELTA, DELTA) += alilqr_config_ptr_->at(W_ACC_HARDBOUND) * (Square(2.0 * k2v4 * x[DELTA]) + cost_var * 2.0 * k2v4);
  }
}

}  // namespace lateral_planning
}  // namespace pnc