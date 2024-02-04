#include "lateral_path_optimizer_cost.h"

#include <cmath>
#include <iostream>

#include "geometry_math.h"
#include "ilqr_define.h"
#include "math_lib.h"

using namespace pnc::mathlib;

namespace planning {
namespace apa_planner {

double ReferenceCostTerm::GetCost(const ilqr_solver::State &x,
                                  const ilqr_solver::Control & /*u*/) {
  const double cost_x = 0.5 * cost_config_ptr_->at(W_REF_X) *
                        Square(x[X] - cost_config_ptr_->at(REF_X));
  const double cost_y = 0.5 * cost_config_ptr_->at(W_REF_Y) *
                        Square(x[Y] - cost_config_ptr_->at(REF_Y));
  const double cost_theta = 0.5 * cost_config_ptr_->at(W_REF_THETA) *
                            Square(pnc::geometry_lib::NormalizeAngle(
                                x[THETA] - cost_config_ptr_->at(REF_THETA)));

  return cost_x + cost_y + cost_theta;
}

void ReferenceCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT & /*lxu*/, ilqr_solver::LuuMT & /*luu*/) {
  lx(X) += cost_config_ptr_->at(W_REF_X) * (x[X] - cost_config_ptr_->at(REF_X));

  lx(Y) += cost_config_ptr_->at(W_REF_Y) * (x[Y] - cost_config_ptr_->at(REF_Y));

  lx(THETA) += cost_config_ptr_->at(W_REF_THETA) *
               (pnc::geometry_lib::NormalizeAngle(
                   x[THETA] - cost_config_ptr_->at(REF_THETA)));

  lxx(X, X) += cost_config_ptr_->at(W_REF_X);
  lxx(Y, Y) += cost_config_ptr_->at(W_REF_Y);
  lxx(THETA, THETA) += cost_config_ptr_->at(W_REF_THETA);
}

double TerminalCostTerm::GetCost(const ilqr_solver::State &x,
                                 const ilqr_solver::Control & /*u*/) {
  const double cost_theta =
      0.5 * cost_config_ptr_->at(W_TERMINAL_THETA) *
      Square(x[THETA] - cost_config_ptr_->at(TERMINAL_THETA));
  const double cost_y = 0.5 * cost_config_ptr_->at(W_TERMINAL_Y) *
                        Square(x[Y] - cost_config_ptr_->at(TERMINAL_Y));

  const double cost_x = 0.5 * cost_config_ptr_->at(W_TERMINAL_X) *
                        Square(x[X] - cost_config_ptr_->at(TERMINAL_X));
  return cost_theta + cost_y + cost_x;
}

void TerminalCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT & /*lxu*/, ilqr_solver::LuuMT & /*luu*/) {
  lx(THETA) += cost_config_ptr_->at(W_TERMINAL_THETA) *
               (x[THETA] - cost_config_ptr_->at(TERMINAL_THETA));
  lx(Y) += cost_config_ptr_->at(W_TERMINAL_Y) *
           (x[Y] - cost_config_ptr_->at(TERMINAL_Y));
  lx(X) += cost_config_ptr_->at(W_TERMINAL_X) *
           (x[X] - cost_config_ptr_->at(TERMINAL_X));

  lxx(THETA, THETA) += cost_config_ptr_->at(W_TERMINAL_THETA);
  lxx(Y, Y) += cost_config_ptr_->at(W_TERMINAL_Y);
  lxx(X, X) += cost_config_ptr_->at(W_TERMINAL_X);
}

double KCostTerm::GetCost(const ilqr_solver::State &x,
                          const ilqr_solver::Control & /*u*/) {
  const double cost = 0.5 * cost_config_ptr_->at(W_K) * Square(x[K]);
  return cost;
}

void KCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT & /*lxu*/, ilqr_solver::LuuMT & /*luu*/) {
  lx(K) += cost_config_ptr_->at(W_K) * x[K];
  lxx(K, K) += cost_config_ptr_->at(W_K);
}

double UCostTerm::GetCost(const ilqr_solver::State & /*x*/,
                          const ilqr_solver::Control &u) {
  const double cost = 0.5 * cost_config_ptr_->at(W_U) * Square(u[U]);
  return cost;
}

void UCostTerm::GetGradientHessian(const ilqr_solver::State & /*x*/,
                                   const ilqr_solver::Control &u,
                                   ilqr_solver::LxMT & /*lx*/,
                                   ilqr_solver::LuMT &lu,
                                   ilqr_solver::LxxMT & /*lxx*/,
                                   ilqr_solver::LxuMT & /*lxu*/,
                                   ilqr_solver::LuuMT &luu) {
  lu(U) += cost_config_ptr_->at(W_U) * u[U];
  luu(U, U) += cost_config_ptr_->at(W_U);
}

double KBoundCostTerm::GetCost(const ilqr_solver::State &x,
                               const ilqr_solver::Control & /*u*/) {
  double cost = 0.0;
  if (Square(x[K]) - Square(cost_config_ptr_->at(K_MAX)) +
          alilqr_config_ptr_->at(ilqr_solver::L_K_HARDBOUND) /
              alilqr_config_ptr_->at(ilqr_solver::W_K_HARDBOUND) >
      0.0) {
    cost += 0.5 * alilqr_config_ptr_->at(ilqr_solver::W_K_HARDBOUND) *
            Square(Square(x[K]) - Square(cost_config_ptr_->at(K_MAX)) +
                   alilqr_config_ptr_->at(ilqr_solver::L_K_HARDBOUND) /
                       alilqr_config_ptr_->at(ilqr_solver::W_K_HARDBOUND));
  }
  return cost;
}

void KBoundCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT & /*lxu*/, ilqr_solver::LuuMT & /*luu*/) {
  if (Square(x[K]) - Square(cost_config_ptr_->at(K_MAX)) +
          alilqr_config_ptr_->at(ilqr_solver::L_K_HARDBOUND) /
              alilqr_config_ptr_->at(ilqr_solver::W_K_HARDBOUND) >
      0.0) {
    lx(K) += (alilqr_config_ptr_->at(ilqr_solver::W_K_HARDBOUND) *
                  (Square(x[K]) - Square(cost_config_ptr_->at(K_MAX))) +
              alilqr_config_ptr_->at(ilqr_solver::L_K_HARDBOUND)) *
             (2.0 * x[K]);

    lxx(K, K) +=
        4.0 * Square(x[K]) *
            alilqr_config_ptr_->at(ilqr_solver::W_K_HARDBOUND) +
        2.0 * (alilqr_config_ptr_->at(ilqr_solver::W_K_HARDBOUND) *
                   (Square(x[K]) - Square(cost_config_ptr_->at(K_MAX))) +
               alilqr_config_ptr_->at(ilqr_solver::L_K_HARDBOUND));
  }
}

double UBoundCostTerm::GetCost(const ilqr_solver::State & /*x*/,
                               const ilqr_solver::Control &u) {
  double cost = 0.0;
  if (Square(u[U]) - Square(cost_config_ptr_->at(U_MAX)) +
          alilqr_config_ptr_->at(ilqr_solver::L_U_HARDBOUND) /
              alilqr_config_ptr_->at(ilqr_solver::W_U_HARDBOUND) >
      0.0) {
    cost += 0.5 * alilqr_config_ptr_->at(ilqr_solver::W_U_HARDBOUND) *
            Square(Square(u[U]) - Square(cost_config_ptr_->at(U_MAX)) +
                   alilqr_config_ptr_->at(ilqr_solver::L_U_HARDBOUND) /
                       alilqr_config_ptr_->at(ilqr_solver::W_U_HARDBOUND));
  }
  return cost;
}

void UBoundCostTerm::GetGradientHessian(const ilqr_solver::State & /*x*/,
                                        const ilqr_solver::Control &u,
                                        ilqr_solver::LxMT & /*lx*/,
                                        ilqr_solver::LuMT &lu,
                                        ilqr_solver::LxxMT & /*lxx*/,
                                        ilqr_solver::LxuMT & /*lxu*/,
                                        ilqr_solver::LuuMT &luu) {
  if (Square(u[U]) - Square(cost_config_ptr_->at(U_MAX)) +
          alilqr_config_ptr_->at(ilqr_solver::L_U_HARDBOUND) /
              alilqr_config_ptr_->at(ilqr_solver::W_U_HARDBOUND) >
      0.0) {
    lu(U) += (alilqr_config_ptr_->at(ilqr_solver::W_U_HARDBOUND) *
                  (Square(u[U]) - Square(cost_config_ptr_->at(U_MAX))) +
              alilqr_config_ptr_->at(ilqr_solver::L_U_HARDBOUND)) *
             (2.0 * u[U]);

    luu(U, U) +=
        4.0 * Square(u[U]) *
            alilqr_config_ptr_->at(ilqr_solver::W_U_HARDBOUND) +
        2.0 * (alilqr_config_ptr_->at(ilqr_solver::W_U_HARDBOUND) *
                   (Square(u[U]) - Square(cost_config_ptr_->at(U_MAX))) +
               alilqr_config_ptr_->at(ilqr_solver::L_U_HARDBOUND));
  }
}

double KSoftBoundCostTerm::GetCost(const ilqr_solver::State &x,
                                   const ilqr_solver::Control & /*u*/) {
  double cost = 0.0;
  if (x(K) > cost_config_ptr_->at(K_MAX)) {
    cost = 0.5 * cost_config_ptr_->at(W_K_BOUND) *
           Square(x[K] - cost_config_ptr_->at(K_MAX));
  } else if (x(K) < -cost_config_ptr_->at(K_MAX)) {
    cost = 0.5 * cost_config_ptr_->at(W_K_BOUND) *
           Square(x[K] + cost_config_ptr_->at(K_MAX));
  }
  return cost;
}

void KSoftBoundCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT & /*lxu*/, ilqr_solver::LuuMT & /*luu*/) {
  if (x(K) > cost_config_ptr_->at(K_MAX)) {
    lx(K) +=
        cost_config_ptr_->at(W_K_BOUND) * (x[K] - cost_config_ptr_->at(K_MAX));

    lxx(K, K) += cost_config_ptr_->at(W_K_BOUND);
  } else if (x(K) < -cost_config_ptr_->at(K_MAX)) {
    lx(K) +=
        cost_config_ptr_->at(W_K_BOUND) * (x[K] + cost_config_ptr_->at(K_MAX));
    lxx(K, K) += cost_config_ptr_->at(W_K_BOUND);
  }
}

double USoftBoundCostTerm::GetCost(const ilqr_solver::State & /*x*/,
                                   const ilqr_solver::Control &u) {
  double cost = 0.0;
  if (u(U) > cost_config_ptr_->at(U_MAX)) {
    cost = 0.5 * cost_config_ptr_->at(W_U_BOUND) *
           Square(u(U) - cost_config_ptr_->at(U_MAX));

  } else if (u(U) < -cost_config_ptr_->at(U_MAX)) {
    cost = 0.5 * cost_config_ptr_->at(W_U_BOUND) *
           Square(u(U) + cost_config_ptr_->at(U_MAX));
  }
  return cost;
}

void USoftBoundCostTerm::GetGradientHessian(const ilqr_solver::State & /*x*/,
                                            const ilqr_solver::Control &u,
                                            ilqr_solver::LxMT & /*lx*/,
                                            ilqr_solver::LuMT &lu,
                                            ilqr_solver::LxxMT & /*lxx*/,
                                            ilqr_solver::LxuMT & /*lxu*/,
                                            ilqr_solver::LuuMT &luu) {
  if (u(U) > cost_config_ptr_->at(U_MAX)) {
    lu(U) +=
        cost_config_ptr_->at(W_U_BOUND) * (u(U) - cost_config_ptr_->at(U_MAX));

    luu(U, U) += cost_config_ptr_->at(W_U_BOUND);

  } else if (u(U) < -cost_config_ptr_->at(U_MAX)) {
    lu(U) +=
        cost_config_ptr_->at(W_U_BOUND) * (u(U) + cost_config_ptr_->at(U_MAX));
    luu(U, U) += cost_config_ptr_->at(W_U_BOUND);
  }
}
}  // namespace apa_planner
}  // namespace planning