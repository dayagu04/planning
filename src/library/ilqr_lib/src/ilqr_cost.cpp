#include "ilqr_cost.h"

namespace ilqr_solver {
static const double eps_value = 1e-4;
void BaseCostTerm::GetDiffGradientHessian(const State &x, const Control &u,
                                          LxMT &lx, LuMT &lu, LxxMT &lxx,
                                          LxuMT &lxu, LuuMT &luu) {
  size_t x_size = x.size();
  size_t u_size = u.size();

  // calculate lx
  Eigen::VectorXd dxi = Eigen::VectorXd::Zero(x_size);
  for (size_t i = 0; i < x_size; ++i) {
    dxi(i) = eps_value;
    lx(i) = (GetCost(x + dxi, u) - GetCost(x - dxi, u)) / (2.0 * eps_value);
    dxi(i) = 0.0;
  }

  // calculate lu
  Eigen::VectorXd dui = Eigen::VectorXd::Zero(u_size);
  for (size_t i = 0; i < u_size; ++i) {
    dui(i) = eps_value;
    lu(i) = (GetCost(x, u + dui) - GetCost(x, u - dui)) / (2.0 * eps_value);
    dui(i) = 0.0;
  }

  // calculate lxx
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

  // calculate luu
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

  // calculate lxu
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
}  // namespace ilqr_solver
