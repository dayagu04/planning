#include "lateral_jerk_constraint.h"

#include "math_lib.h"

using namespace pnc::mathlib;
namespace pnc {
namespace lateral_planning {

double LateralJerkBoundCostTerm::GetCost(
    const ilqr_solver::State &x, const ilqr_solver::Control &u) {
  double cost = 0.;
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR)) *
                      SSquare(cost_config_ptr_->at(REF_VEL));
  if (u[OMEGA] > cost_config_ptr_->at(OMEGA_UPPER_BOUND)) {
    cost = 0.5 * cost_config_ptr_->at(W_JERK_BOUND) * k2v4 *
           Square(u[OMEGA] - cost_config_ptr_->at(OMEGA_UPPER_BOUND));
  } else if (u[OMEGA] < cost_config_ptr_->at(OMEGA_LOWER_BOUND)) {
    cost = 0.5 * cost_config_ptr_->at(W_JERK_BOUND) * k2v4 *
           Square(u[OMEGA] - cost_config_ptr_->at(OMEGA_LOWER_BOUND));
  }
  cost_value_ += cost;
  return cost;
}

void LateralJerkBoundCostTerm::GetGradientHessian(const ilqr_solver::State &x,
                                              const ilqr_solver::Control &u,
                                              ilqr_solver::LxMT & /*lx*/,
                                              ilqr_solver::LuMT &lu,
                                              ilqr_solver::LxxMT & /*lxx*/,
                                              ilqr_solver::LxuMT & /*lxu*/,
                                              ilqr_solver::LuuMT &luu) {
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR)) *
                      SSquare(cost_config_ptr_->at(REF_VEL));
  if (u[OMEGA] > cost_config_ptr_->at(OMEGA_UPPER_BOUND)) {
    lu(OMEGA) += cost_config_ptr_->at(W_JERK_BOUND) * k2v4 *
                 (u[OMEGA] - cost_config_ptr_->at(OMEGA_UPPER_BOUND));
    luu(OMEGA, OMEGA) += cost_config_ptr_->at(W_JERK_BOUND) * k2v4;
  } else if (u[OMEGA] < cost_config_ptr_->at(OMEGA_LOWER_BOUND)) {
    lu(OMEGA) += cost_config_ptr_->at(W_JERK_BOUND) * k2v4 *
                 (u[OMEGA] - cost_config_ptr_->at(OMEGA_LOWER_BOUND));
    luu(OMEGA, OMEGA) += cost_config_ptr_->at(W_JERK_BOUND) * k2v4;
  }
}

double LateralJerkHardBoundCostTerm::GetCost(
    const ilqr_solver::State &x, const ilqr_solver::Control &u) {
  double cost = 0.;
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR)) *
                      SSquare(cost_config_ptr_->at(REF_VEL));
  const double cost_var =
    k2v4 * (Square(u[OMEGA]) - Square(cost_config_ptr_->at(OMEGA_UPPER_BOUND))) +
    alilqr_config_ptr_->at(L_JERK_HARDBOUND) /
    alilqr_config_ptr_->at(W_JERK_HARDBOUND);
  if (cost_var > 0.) {
    cost += 0.5 * alilqr_config_ptr_->at(W_JERK_HARDBOUND) * Square(cost_var);
  }
  return cost;
}

void LateralJerkHardBoundCostTerm::GetGradientHessian(const ilqr_solver::State &x,
                                              const ilqr_solver::Control &u,
                                              ilqr_solver::LxMT & /*lx*/,
                                              ilqr_solver::LuMT &lu,
                                              ilqr_solver::LxxMT & /*lxx*/,
                                              ilqr_solver::LxuMT & /*lxu*/,
                                              ilqr_solver::LuuMT &luu) {
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR)) *
                      SSquare(cost_config_ptr_->at(REF_VEL));
  const double cost_var =
    k2v4 * (Square(u[OMEGA]) - Square(cost_config_ptr_->at(OMEGA_UPPER_BOUND))) +
    alilqr_config_ptr_->at(L_JERK_HARDBOUND) /
    alilqr_config_ptr_->at(W_JERK_HARDBOUND);
  if (cost_var > 0.) {
    lu(OMEGA) += alilqr_config_ptr_->at(W_JERK_HARDBOUND) * cost_var * 2.0 * k2v4 * u[OMEGA];
    luu(OMEGA, OMEGA) += alilqr_config_ptr_->at(W_JERK_HARDBOUND) * (Square(2.0 * k2v4 * u[OMEGA]) + cost_var * 2.0 * k2v4);
  }
}

}  // namespace lateral_planning
}  // namespace pnc