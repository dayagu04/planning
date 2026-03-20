#include "longitudinal_motion_planning_cost.h"

#include <iostream>

#include "math_lib.h"

using namespace pnc::mathlib;
namespace pnc {
namespace longitudinal_planning {
// reference cost for s and v
double ReferenceCostTerm::GetCost(const ilqr_solver::State &x,
                                  const ilqr_solver::Control &) {
  return 0.5 * (cost_config_ptr_->at(W_REF_POS) *
                    Square(x[POS] - cost_config_ptr_->at(REF_POS)) +
                cost_config_ptr_->at(W_REF_VEL) *
                    Square(x[VEL] - cost_config_ptr_->at(REF_VEL)));
}

void ReferenceCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control &,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT &, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) {
  lx(POS) += -cost_config_ptr_->at(W_REF_POS) *
             (cost_config_ptr_->at(REF_POS) - x[POS]);
  lx(VEL) += -cost_config_ptr_->at(W_REF_VEL) *
             (cost_config_ptr_->at(REF_VEL) - x[VEL]);

  lxx(POS, POS) += cost_config_ptr_->at(W_REF_POS);
  lxx(VEL, VEL) += cost_config_ptr_->at(W_REF_VEL);
}

// longitudinal acc cost
double LonAccCostTerm::GetCost(const ilqr_solver::State &x,
                               const ilqr_solver::Control &) {
  return 0.5 * cost_config_ptr_->at(W_ACC) * Square(x[ACC]);
}

void LonAccCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control &,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT &, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) {
  lx(ACC) += cost_config_ptr_->at(W_ACC) * x[ACC];
  lxx(ACC, ACC) += cost_config_ptr_->at(W_ACC);
}

// longitudinal jerk cost
double LonJerkCostTerm::GetCost(const ilqr_solver::State & /*x*/,
                                const ilqr_solver::Control &u) {
  return 0.5 * cost_config_ptr_->at(W_JERK) * Square(u[JERK]);
}

void LonJerkCostTerm::GetGradientHessian(const ilqr_solver::State & /*x*/,
                                         const ilqr_solver::Control &u,
                                         ilqr_solver::LxMT & /*lx*/,
                                         ilqr_solver::LuMT &lu,
                                         ilqr_solver::LxxMT & /*lxx*/,
                                         ilqr_solver::LxuMT & /*lxu*/,
                                         ilqr_solver::LuuMT &luu) {
  lu(JERK) += cost_config_ptr_->at(W_JERK) * u[JERK];
  luu(JERK, JERK) += cost_config_ptr_->at(W_JERK);
}

// longitudinal pos soft bound cost
double LonSoftPosBoundCostTerm::GetCost(const ilqr_solver::State &x,
                                        const ilqr_solver::Control &) {
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
    const ilqr_solver::State &x, const ilqr_solver::Control &,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT &, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) {
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

// longitudinal pos hard bound cost
double LonHardPosBoundCostTerm::GetCost(const ilqr_solver::State &x,
                                        const ilqr_solver::Control &) {
  double cost = 0.0;
  if (x[POS] > cost_config_ptr_->at(HARD_POS_MAX)) {
    cost = 0.5 * cost_config_ptr_->at(W_HARD_POS_BOUND) *
           Square(x[POS] - cost_config_ptr_->at(HARD_POS_MAX));
  } else if (x[POS] < cost_config_ptr_->at(HARD_POS_MIN)) {
    cost = 0.5 * cost_config_ptr_->at(W_HARD_POS_BOUND) *
           Square(x[POS] - cost_config_ptr_->at(HARD_POS_MIN));
  }
  return cost;
}

void LonHardPosBoundCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control &,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT &, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) {
  if (x[POS] > cost_config_ptr_->at(HARD_POS_MAX)) {
    lx(POS) += cost_config_ptr_->at(W_HARD_POS_BOUND) *
               (x[POS] - cost_config_ptr_->at(HARD_POS_MAX));

    lxx(POS, POS) += cost_config_ptr_->at(W_HARD_POS_BOUND);
  } else if (x[POS] < cost_config_ptr_->at(HARD_POS_MIN)) {
    lx(POS) += cost_config_ptr_->at(W_HARD_POS_BOUND) *
               (x[POS] - cost_config_ptr_->at(HARD_POS_MIN));

    lxx(POS, POS) += cost_config_ptr_->at(W_HARD_POS_BOUND);
  }
}
// longitudinal vel bound cost
double LonVelBoundCostTerm::GetCost(const ilqr_solver::State &x,
                                    const ilqr_solver::Control &) {
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

void LonVelBoundCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control &,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT &, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) {
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

// non-negative vel cost
double NonNegativeVelCost::GetCost(const ilqr_solver::State &x,
                                   const ilqr_solver::Control &) {
  double cost = 0.0;
  if (x[VEL] < 0.0) {
    cost = 0.5 * cost_config_ptr_->at(W_NON_NEGATIVE_VEL) * Square(x[VEL]);
  }

  return cost;
}

void NonNegativeVelCost::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control &,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT &, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) {
  if (x[VEL] < 0.0) {
    lx(VEL) += cost_config_ptr_->at(W_NON_NEGATIVE_VEL) * (x[VEL]);

    lxx(VEL, VEL) += cost_config_ptr_->at(W_NON_NEGATIVE_VEL);
  }
}

// longitudinal acc bound cost
double LonAccBoundCostTerm::GetCost(const ilqr_solver::State &x,
                                    const ilqr_solver::Control &) {
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

void LonAccBoundCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control &,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT &, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) {
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
double LonJerkBoundCostTerm::GetCost(const ilqr_solver::State & /*x*/,
                                     const ilqr_solver::Control &u) {
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

void LonJerkBoundCostTerm::GetGradientHessian(const ilqr_solver::State & /*x*/,
                                              const ilqr_solver::Control &u,
                                              ilqr_solver::LxMT & /*lx*/,
                                              ilqr_solver::LuMT &lu,
                                              ilqr_solver::LxxMT & /*lxx*/,
                                              ilqr_solver::LxuMT & /*lxu*/,
                                              ilqr_solver::LuuMT &luu) {
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

// longitudinal stop point cost
double LonStopPointCost::GetCost(const ilqr_solver::State &x,
                                 const ilqr_solver::Control &) {
  double cost = 0.0;
  if ((x[POS] <= cost_config_ptr_->at(S_STOP) && x[VEL] <= 0.0) ||
      (x[POS] > cost_config_ptr_->at(S_STOP) && x[VEL] >= 0.0)) {
    cost = 0.5 * cost_config_ptr_->at(W_S_STOP) * Square(x[VEL]);
  }

  return cost;
}

void LonStopPointCost::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control &,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT &, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) {
  if ((x[POS] <= cost_config_ptr_->at(S_STOP) && x[VEL] <= 0.0) ||
      (x[POS] > cost_config_ptr_->at(S_STOP) && x[VEL] >= 0.0)) {
    lx(VEL) = cost_config_ptr_->at(W_S_STOP) * x[VEL];
    lxx(VEL, VEL) = cost_config_ptr_->at(W_S_STOP);
  }
}

}  // namespace longitudinal_planning
}  // namespace pnc
