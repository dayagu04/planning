#include "scc_longitudinal_motion_planning_cost_v3.h"

#include <algorithm>
#include <iostream>

#include "math_lib.h"

using namespace pnc::mathlib;
namespace pnc {
namespace scc_longitudinal_planning_v3 {

// reference cost for s and v
double ReferenceCostTerm::GetCost(const al_ilqr_solver::State &x,
                                  const al_ilqr_solver::Control &) {
  return 0.5 * (cost_config_ptr_->at(W_REF_POS) *
                    Square(x[POS] - cost_config_ptr_->at(REF_POS)) +
                cost_config_ptr_->at(W_REF_VEL) *
                    Square(x[VEL] - cost_config_ptr_->at(REF_VEL)) +
                cost_config_ptr_->at(W_REF_ACC) *
                    Square(x[ACC] - cost_config_ptr_->at(REF_ACC)));
}

void ReferenceCostTerm::GetGradientHessian(const al_ilqr_solver::State &x,
                                           const al_ilqr_solver::Control &,
                                           al_ilqr_solver::LxMT &lx,
                                           al_ilqr_solver::LuMT &,
                                           al_ilqr_solver::LxxMT &lxx,
                                           al_ilqr_solver::LxuMT &,
                                           al_ilqr_solver::LuuMT &) {
  lx(POS) += -cost_config_ptr_->at(W_REF_POS) *
             (cost_config_ptr_->at(REF_POS) - x[POS]);
  lx(VEL) += -cost_config_ptr_->at(W_REF_VEL) *
             (cost_config_ptr_->at(REF_VEL) - x[VEL]);
  lx(ACC) += -cost_config_ptr_->at(W_REF_ACC) *
             (cost_config_ptr_->at(REF_ACC) - x[ACC]);
  lxx(POS, POS) += cost_config_ptr_->at(W_REF_POS);
  lxx(VEL, VEL) += cost_config_ptr_->at(W_REF_VEL);
  lxx(ACC, ACC) += cost_config_ptr_->at(W_REF_ACC);
}

// longitudinal acc cost
double LonAccCostTerm::GetCost(const al_ilqr_solver::State &x,
                               const al_ilqr_solver::Control &) {
  return 0.5 * cost_config_ptr_->at(W_ACC) * Square(x[ACC]);
}

void LonAccCostTerm::GetGradientHessian(const al_ilqr_solver::State &x,
                                        const al_ilqr_solver::Control &,
                                        al_ilqr_solver::LxMT &lx,
                                        al_ilqr_solver::LuMT &,
                                        al_ilqr_solver::LxxMT &lxx,
                                        al_ilqr_solver::LxuMT &,
                                        al_ilqr_solver::LuuMT &) {
  lx(ACC) += cost_config_ptr_->at(W_ACC) * x[ACC];
  lxx(ACC, ACC) += cost_config_ptr_->at(W_ACC);
}

// longitudinal jerk cost
double LonJerkCostTerm::GetCost(const al_ilqr_solver::State & /*x*/,
                                const al_ilqr_solver::Control &u) {
  return 0.5 * cost_config_ptr_->at(W_JERK) * Square(u[JERK]);
}

void LonJerkCostTerm::GetGradientHessian(const al_ilqr_solver::State & /*x*/,
                                         const al_ilqr_solver::Control &u,
                                         al_ilqr_solver::LxMT & /*lx*/,
                                         al_ilqr_solver::LuMT &lu,
                                         al_ilqr_solver::LxxMT & /*lxx*/,
                                         al_ilqr_solver::LxuMT & /*lxu*/,
                                         al_ilqr_solver::LuuMT &luu) {
  lu(JERK) += cost_config_ptr_->at(W_JERK) * u[JERK];
  luu(JERK, JERK) += cost_config_ptr_->at(W_JERK);
}

// longitudinal pos soft bound cost
double LonSoftPosBoundCostTerm::GetCost(const al_ilqr_solver::State &x,
                                        const al_ilqr_solver::Control &) {
  double cost = 0.0;
  if (x[POS] > cost_config_ptr_->at(SOFT_POS_MAX)) {
    cost = 0.5 * cost_config_ptr_->at(W_POS_BOUND) *
           Square(x[POS] - cost_config_ptr_->at(SOFT_POS_MAX));
  } else if (x[POS] < cost_config_ptr_->at(SOFT_POS_MIN)) {
    cost = 0.5 * cost_config_ptr_->at(W_POS_BOUND) *
           Square(x[POS] - cost_config_ptr_->at(SOFT_POS_MIN));
  }

  return cost;
}

void LonSoftPosBoundCostTerm::GetGradientHessian(
    const al_ilqr_solver::State &x, const al_ilqr_solver::Control &,
    al_ilqr_solver::LxMT &lx, al_ilqr_solver::LuMT &,
    al_ilqr_solver::LxxMT &lxx, al_ilqr_solver::LxuMT &,
    al_ilqr_solver::LuuMT &) {
  if (x[POS] > cost_config_ptr_->at(SOFT_POS_MAX)) {
    lx(POS) += cost_config_ptr_->at(W_POS_BOUND) *
               (x[POS] - cost_config_ptr_->at(SOFT_POS_MAX));

    lxx(POS, POS) += cost_config_ptr_->at(W_POS_BOUND);
  } else if (x[POS] < cost_config_ptr_->at(SOFT_POS_MIN)) {
    lx(POS) += cost_config_ptr_->at(W_POS_BOUND) *
               (x[POS] - cost_config_ptr_->at(SOFT_POS_MIN));

    lxx(POS, POS) += cost_config_ptr_->at(W_POS_BOUND);
  }
}

double LonHardPosBoundCostTerm::GetCost(const al_ilqr_solver::State &x,
                                        const al_ilqr_solver::Control &) {
  double cost = 0.0;
  const double eps_rho = 1e-10;

  const double mu_upper = constraint_config_ptr_->at(MU_HARD_POS_UPPER);
  const double rho_upper = constraint_config_ptr_->at(RHO_HARD_POS_UPPER);
  const double mu_lower = constraint_config_ptr_->at(MU_HARD_POS_LOWER);
  const double rho_lower = constraint_config_ptr_->at(RHO_HARD_POS_LOWER);

  const double safe_distance = cost_config_ptr_->at(SAFE_DISTANCE);
  const double hard_pos_max_safe =
      std::max(cost_config_ptr_->at(HARD_POS_MAX) - safe_distance, 0.0);
  const double c_upper = x[POS] - hard_pos_max_safe;
  if (c_upper + mu_upper / std::max(rho_upper, eps_rho) > 0.0) {
    cost += mu_upper * c_upper + 0.5 * rho_upper * c_upper * c_upper;
  } else {
    cost += -mu_upper * mu_upper / (2.0 * std::max(rho_upper, eps_rho));
  }

  const double c_lower = cost_config_ptr_->at(HARD_POS_MIN) - x[POS];
  if (c_lower + mu_lower / std::max(rho_lower, eps_rho) > 0.0) {
    cost += mu_lower * c_lower + 0.5 * rho_lower * c_lower * c_lower;
  } else {
    cost += -mu_lower * mu_lower / (2.0 * std::max(rho_lower, eps_rho));
  }

  return cost;
}

void LonHardPosBoundCostTerm::GetGradientHessian(
    const al_ilqr_solver::State &x, const al_ilqr_solver::Control &,
    al_ilqr_solver::LxMT &lx, al_ilqr_solver::LuMT &,
    al_ilqr_solver::LxxMT &lxx, al_ilqr_solver::LxuMT &,
    al_ilqr_solver::LuuMT &) {
  const double eps_rho = 1e-10;

  const double mu_upper = constraint_config_ptr_->at(MU_HARD_POS_UPPER);
  const double rho_upper = constraint_config_ptr_->at(RHO_HARD_POS_UPPER);
  const double mu_lower = constraint_config_ptr_->at(MU_HARD_POS_LOWER);
  const double rho_lower = constraint_config_ptr_->at(RHO_HARD_POS_LOWER);

  const double safe_distance = cost_config_ptr_->at(SAFE_DISTANCE);
  const double hard_pos_max_safe =
      std::max(cost_config_ptr_->at(HARD_POS_MAX) - safe_distance, 0.0);
  const double c_upper = x[POS] - hard_pos_max_safe;
  if (c_upper + mu_upper / std::max(rho_upper, eps_rho) > 0.0) {
    // dl/dx = (mu + rho*c) * dc/dx = (mu + rho*c)
    lx(POS) += (mu_upper + rho_upper * c_upper);
    // d2l/dx2 = rho * (dc/dx)^2 = rho
    lxx(POS, POS) += rho_upper;
  }

  const double c_lower = cost_config_ptr_->at(HARD_POS_MIN) - x[POS];
  if (c_lower + mu_lower / std::max(rho_lower, eps_rho) > 0.0) {
    // dl/dx = (mu + rho*c) * dc/dx = -(mu + rho*c)
    lx(POS) += -(mu_lower + rho_lower * c_lower);
    // d2l/dx2 = rho * (-1)^2 = rho
    lxx(POS, POS) += rho_lower;
  }
}

// longitudinal pos extend bound cost
double LonExtendPosBoundCostTerm::GetCost(const al_ilqr_solver::State &x,
                                          const al_ilqr_solver::Control &) {
  double cost = 0.0;
  if (x[POS] > cost_config_ptr_->at(EXTEND_POS_MAX)) {
    cost = 0.5 * cost_config_ptr_->at(W_EXTEND_POS_BOUND) *
           Square(x[POS] - cost_config_ptr_->at(EXTEND_POS_MAX));
  } else if (x[POS] < cost_config_ptr_->at(EXTEND_POS_MIN)) {
    cost = 0.5 * cost_config_ptr_->at(W_EXTEND_POS_BOUND) *
           Square(x[POS] - cost_config_ptr_->at(EXTEND_POS_MIN));
  }
  return cost;
}

void LonExtendPosBoundCostTerm::GetGradientHessian(
    const al_ilqr_solver::State &x, const al_ilqr_solver::Control &,
    al_ilqr_solver::LxMT &lx, al_ilqr_solver::LuMT &,
    al_ilqr_solver::LxxMT &lxx, al_ilqr_solver::LxuMT &,
    al_ilqr_solver::LuuMT &) {
  if (x[POS] > cost_config_ptr_->at(EXTEND_POS_MAX)) {
    lx(POS) += cost_config_ptr_->at(W_EXTEND_POS_BOUND) *
               (x[POS] - cost_config_ptr_->at(EXTEND_POS_MAX));
    lxx(POS, POS) += cost_config_ptr_->at(W_EXTEND_POS_BOUND);
  } else if (x[POS] < cost_config_ptr_->at(EXTEND_POS_MIN)) {
    lx(POS) += cost_config_ptr_->at(W_EXTEND_POS_BOUND) *
               (x[POS] - cost_config_ptr_->at(EXTEND_POS_MIN));
    lxx(POS, POS) += cost_config_ptr_->at(W_EXTEND_POS_BOUND);
  }
}

// longitudinal vel bound cost
double LonVelBoundCostTerm::GetCost(const al_ilqr_solver::State &x,
                                    const al_ilqr_solver::Control &) {
  double cost = 0.0;
  if (x[VEL] > cost_config_ptr_->at(VEL_MAX)) {
    cost = 0.5 * cost_config_ptr_->at(W_VEL_BOUND) *
           Square(x[VEL] - cost_config_ptr_->at(VEL_MAX));
  } else if (x[VEL] < cost_config_ptr_->at(VEL_MIN)) {
    cost = 0.5 * cost_config_ptr_->at(W_VEL_BOUND) *
           Square(x[VEL] - cost_config_ptr_->at(VEL_MIN));
  }
  return cost;
}

void LonVelBoundCostTerm::GetGradientHessian(const al_ilqr_solver::State &x,
                                             const al_ilqr_solver::Control &,
                                             al_ilqr_solver::LxMT &lx,
                                             al_ilqr_solver::LuMT &,
                                             al_ilqr_solver::LxxMT &lxx,
                                             al_ilqr_solver::LxuMT &,
                                             al_ilqr_solver::LuuMT &) {
  if (x[VEL] > cost_config_ptr_->at(VEL_MAX)) {
    lx(VEL) += cost_config_ptr_->at(W_VEL_BOUND) *
               (x[VEL] - cost_config_ptr_->at(VEL_MAX));

    lxx(VEL, VEL) += cost_config_ptr_->at(W_VEL_BOUND);
  } else if (x[VEL] < cost_config_ptr_->at(VEL_MIN)) {
    lx(VEL) += cost_config_ptr_->at(W_VEL_BOUND) *
               (x[VEL] - cost_config_ptr_->at(VEL_MIN));

    lxx(VEL, VEL) += cost_config_ptr_->at(W_VEL_BOUND);
  }
}

double LonHardNonNegativeVelCostTerm::GetCost(const al_ilqr_solver::State &x,
                                              const al_ilqr_solver::Control &) {
  const double eps_rho = 1e-10;
  const double mu = constraint_config_ptr_->at(MU_HARD_VEL_LOWER);
  const double rho = constraint_config_ptr_->at(RHO_HARD_VEL_LOWER);

  const double c = -x[VEL];
  if (c + mu / std::max(rho, eps_rho) > 0.0) {
    return mu * c + 0.5 * rho * c * c;
  }
  return -mu * mu / (2.0 * std::max(rho, eps_rho));
}

void LonHardNonNegativeVelCostTerm::GetGradientHessian(
    const al_ilqr_solver::State &x, const al_ilqr_solver::Control &,
    al_ilqr_solver::LxMT &lx, al_ilqr_solver::LuMT &,
    al_ilqr_solver::LxxMT &lxx, al_ilqr_solver::LxuMT &,
    al_ilqr_solver::LuuMT &) {
  const double eps_rho = 1e-10;
  const double mu = constraint_config_ptr_->at(MU_HARD_VEL_LOWER);
  const double rho = constraint_config_ptr_->at(RHO_HARD_VEL_LOWER);

  const double c = -x[VEL];
  if (c + mu / std::max(rho, eps_rho) > 0.0) {
    // dl/dx = (mu + rho*c) * dc/dx = -(mu + rho*c)
    lx(VEL) += -(mu + rho * c);
    // d2l/dx2 = rho * (-1)^2 = rho
    lxx(VEL, VEL) += rho;
  }
}

// longitudinal acc bound cost
double LonAccBoundCostTerm::GetCost(const al_ilqr_solver::State &x,
                                    const al_ilqr_solver::Control &) {
  double cost = 0.0;
  if (x[ACC] > cost_config_ptr_->at(ACC_MAX)) {
    cost = 0.5 * cost_config_ptr_->at(W_ACC_BOUND) *
           Square(x[ACC] - cost_config_ptr_->at(ACC_MAX));
  } else if (x[ACC] < cost_config_ptr_->at(ACC_MIN)) {
    cost = 0.5 * cost_config_ptr_->at(W_ACC_BOUND) *
           Square(x[ACC] - cost_config_ptr_->at(ACC_MIN));
  }
  return cost;
}

void LonAccBoundCostTerm::GetGradientHessian(const al_ilqr_solver::State &x,
                                             const al_ilqr_solver::Control &,
                                             al_ilqr_solver::LxMT &lx,
                                             al_ilqr_solver::LuMT &,
                                             al_ilqr_solver::LxxMT &lxx,
                                             al_ilqr_solver::LxuMT &,
                                             al_ilqr_solver::LuuMT &) {
  if (x[ACC] > cost_config_ptr_->at(ACC_MAX)) {
    lx(ACC) += cost_config_ptr_->at(W_ACC_BOUND) *
               (x[ACC] - cost_config_ptr_->at(ACC_MAX));

    lxx(ACC, ACC) += cost_config_ptr_->at(W_ACC_BOUND);
  } else if (x[ACC] < cost_config_ptr_->at(ACC_MIN)) {
    lx(ACC) += cost_config_ptr_->at(W_ACC_BOUND) *
               (x[ACC] - cost_config_ptr_->at(ACC_MIN));
    lxx(ACC, ACC) += cost_config_ptr_->at(W_ACC_BOUND);
  }
}

// longitudinal jerk bound cost
double LonJerkBoundCostTerm::GetCost(const al_ilqr_solver::State & /*x*/,
                                     const al_ilqr_solver::Control &u) {
  double cost = 0.0;
  if (u[JERK] > cost_config_ptr_->at(JERK_MAX)) {
    cost = 0.5 * cost_config_ptr_->at(W_JERK_BOUND) *
           Square(u[JERK] - cost_config_ptr_->at(JERK_MAX));
  } else if (u[JERK] < cost_config_ptr_->at(JERK_MIN)) {
    cost = 0.5 * cost_config_ptr_->at(W_JERK_BOUND) *
           Square(u[JERK] - cost_config_ptr_->at(JERK_MIN));
  }

  return cost;
}

void LonJerkBoundCostTerm::GetGradientHessian(
    const al_ilqr_solver::State & /*x*/, const al_ilqr_solver::Control &u,
    al_ilqr_solver::LxMT & /*lx*/, al_ilqr_solver::LuMT &lu,
    al_ilqr_solver::LxxMT & /*lxx*/, al_ilqr_solver::LxuMT & /*lxu*/,
    al_ilqr_solver::LuuMT &luu) {
  if (u[JERK] > cost_config_ptr_->at(JERK_MAX)) {
    lu(JERK) += cost_config_ptr_->at(W_JERK_BOUND) *
                (u[JERK] - cost_config_ptr_->at(JERK_MAX));

    luu(JERK, JERK) += cost_config_ptr_->at(W_JERK_BOUND);
  } else if (u[JERK] < cost_config_ptr_->at(JERK_MIN)) {
    lu(JERK) += cost_config_ptr_->at(W_JERK_BOUND) *
                (u[JERK] - cost_config_ptr_->at(JERK_MIN));

    luu(JERK, JERK) += cost_config_ptr_->at(W_JERK_BOUND);
  }
}

// Longitudinal emergency stop cost
double LonEmergencyStopCostTerm::GetCost(const al_ilqr_solver::State &x,
                                         const al_ilqr_solver::Control &) {
  return 0.5 * cost_config_ptr_->at(W_EMERGENCY_STOP) * Square(x[VEL]);
}

void LonEmergencyStopCostTerm::GetGradientHessian(
    const al_ilqr_solver::State &x, const al_ilqr_solver::Control &,
    al_ilqr_solver::LxMT &lx, al_ilqr_solver::LuMT &,
    al_ilqr_solver::LxxMT &lxx, al_ilqr_solver::LxuMT &,
    al_ilqr_solver::LuuMT &) {
  lx(VEL) += cost_config_ptr_->at(W_EMERGENCY_STOP) * x[VEL];
  lxx(VEL, VEL) += cost_config_ptr_->at(W_EMERGENCY_STOP);
}

}  // namespace scc_longitudinal_planning_v3
}  // namespace pnc
