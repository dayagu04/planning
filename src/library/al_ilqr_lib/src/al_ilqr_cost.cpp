#include "al_ilqr_cost.h"

namespace al_ilqr_solver {
static const double eps_value = 1e-4;

// Compute gradient and Hessian of cost via central finite difference.
//
// Numerical derivatives for verification or when analytical form is unavailable.
// Uses central difference with step size eps = 1e-4:
//   df/dx_i ≈ [f(x + e_i*eps) - f(x - e_i*eps)] / (2*eps)
//   d2f/(dx_i dx_j) ≈ [f(x+e_i+e_j) - f(x-e_i+e_j) - f(x+e_i-e_j) + f(x-e_i-e_j)]
//                      / (4*eps^2)
//
// @param x Current state vector (n-dim)
// @param u Current control vector (m-dim)
// @param lx  [out] dJ/dx    (n-dim)
// @param lu  [out] dJ/du    (m-dim)
// @param lxx [out] d2J/dx2  (n x n)
// @param lxu [out] d2J/dxdu (n x m)
// @param luu [out] d2J/du2  (m x m)
void AliLqrBaseCostTerm::GetDiffGradientHessian(const State &x,
                                                 const Control &u,
                                                 LxMT &lx, LuMT &lu,
                                                 LxxMT &lxx, LxuMT &lxu,
                                                 LuuMT &luu) {
  size_t x_size = x.size();
  size_t u_size = u.size();

  // calculate lx: dJ/dx via central difference
  Eigen::VectorXd dxi = Eigen::VectorXd::Zero(x_size);
  for (size_t i = 0; i < x_size; ++i) {
    dxi(i) = eps_value;
    lx(i) = (GetCost(x + dxi, u) - GetCost(x - dxi, u)) / (2.0 * eps_value);
    dxi(i) = 0.0;
  }

  // calculate lu: dJ/du via central difference
  Eigen::VectorXd dui = Eigen::VectorXd::Zero(u_size);
  for (size_t i = 0; i < u_size; ++i) {
    dui(i) = eps_value;
    lu(i) = (GetCost(x, u + dui) - GetCost(x, u - dui)) / (2.0 * eps_value);
    dui(i) = 0.0;
  }

  // calculate lxx: d2J/dx2 via central difference
  dxi.setZero();
  Eigen::VectorXd dxj = Eigen::VectorXd::Zero(x_size);

  for (size_t i = 0; i < x_size; ++i) {
    for (size_t j = 0; j < x_size; ++j) {
      dxi(i) = eps_value;
      dxj(j) = eps_value;
      lxx(i, j) = (GetCost(x + dxi + dxj, u) - GetCost(x - dxi + dxj, u) -
                   GetCost(x + dxi - dxj, u) + GetCost(x - dxi - dxj, u)) /
                  (4.0 * eps_value * eps_value);
      dxi(i) = 0.0;
      dxj(j) = 0.0;
    }
  }

  // calculate luu: d2J/du2 via central difference
  dui.setZero();
  Eigen::VectorXd duj = Eigen::VectorXd::Zero(u_size);
  for (size_t i = 0; i < u_size; ++i) {
    for (size_t j = 0; j < u_size; ++j) {
      dui(i) = eps_value;
      duj(j) = eps_value;
      luu(i, j) = (GetCost(x, u + dui + duj) - GetCost(x, u - dui + duj) -
                   GetCost(x, u + dui - duj) + GetCost(x, u - dui - duj)) /
                  (4.0 * eps_value * eps_value);
      dui(i) = 0.0;
      duj(j) = 0.0;
    }
  }

  // calculate lxu: d2J/(dx du) via central difference
  duj.setZero();
  dxi.setZero();
  for (size_t i = 0; i < x_size; ++i) {
    for (size_t j = 0; j < u_size; ++j) {
      dxi(i) = eps_value;
      duj(j) = eps_value;
      lxu(i, j) = (GetCost(x + dxi, u + duj) - GetCost(x + dxi, u - duj) -
                   GetCost(x - dxi, u + duj) + GetCost(x - dxi, u - duj)) /
                  (4.0 * eps_value * eps_value);
      dxi(i) = 0.0;
      duj(j) = 0.0;
    }
  }
}
}  // namespace al_ilqr_solver
