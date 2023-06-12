#include "lateral_motion_planning_cost.h"
#include "math_lib.h"

using namespace pnc::mathlib;
namespace pnc {
namespace lateral_planning {
double ReferenceCostTerm::GetCost(const State &x, const Control & /*u*/) {
  return 0.5 * (cost_config_ptr_->at(W_REF_X) * Square(x[X] - cost_config_ptr_->at(REF_X)) +
                cost_config_ptr_->at(W_REF_Y) * Square(x[Y] - cost_config_ptr_->at(REF_Y)) +
                cost_config_ptr_->at(W_REF_THETA) * Square(x[THETA] - cost_config_ptr_->at(REF_THETA)));
}

void ReferenceCostTerm::GetGradientHessian(const State &x, const Control & /*u*/, LxMT &lx, LuMT & /*lu*/, LxxMT &lxx,
                                           LxuMT & /*lxu*/, LuuMT & /*luu*/) {
  lx(0) += -cost_config_ptr_->at(W_REF_X) * (cost_config_ptr_->at(REF_X) - x[X]);
  lx(1) += -cost_config_ptr_->at(W_REF_Y) * (cost_config_ptr_->at(REF_Y) - x[Y]);
  lx(2) += -cost_config_ptr_->at(W_REF_THETA) * (cost_config_ptr_->at(REF_THETA) - x[THETA]);

  lxx(0, 0) += cost_config_ptr_->at(W_REF_X);
  lxx(1, 1) += cost_config_ptr_->at(W_REF_Y);
  lxx(2, 2) += cost_config_ptr_->at(W_REF_THETA);
}

double ContinuityCostTerm::GetCost(const State &x, const Control & /*u*/) {
  return 0.5 * (cost_config_ptr_->at(W_CONTINUITY_X) * Square(x[X] - cost_config_ptr_->at(CONTINUITY_X)) +
                cost_config_ptr_->at(W_CONTINUITY_Y) * Square(x[Y] - cost_config_ptr_->at(CONTINUITY_Y)) +
                cost_config_ptr_->at(W_CONTINUITY_THETA) * Square(x[THETA] - cost_config_ptr_->at(CONTINUITY_THETA)));
}

void ContinuityCostTerm::GetGradientHessian(const State &x, const Control & /*u*/, LxMT &lx, LuMT & /*lu*/, LxxMT &lxx,
                                            LxuMT & /*lxu*/, LuuMT & /*luu*/) {
  lx(0) += -cost_config_ptr_->at(W_CONTINUITY_X) * (cost_config_ptr_->at(CONTINUITY_X) - x[X]);
  lx(1) += -cost_config_ptr_->at(W_CONTINUITY_Y) * (cost_config_ptr_->at(CONTINUITY_Y) - x[Y]);
  lx(2) += -cost_config_ptr_->at(W_CONTINUITY_THETA) * (cost_config_ptr_->at(CONTINUITY_THETA) - x[THETA]);

  lxx(0, 0) += cost_config_ptr_->at(W_CONTINUITY_X);
  lxx(1, 1) += cost_config_ptr_->at(W_CONTINUITY_Y);
  lxx(2, 2) += cost_config_ptr_->at(W_CONTINUITY_THETA);
}

double LatAccCostTerm::GetCost(const State &x, const Control & /*u*/) {
  return 0.5 * cost_config_ptr_->at(W_ACC) *
         Square(cost_config_ptr_->at(CURV_FACTOR) * Square(cost_config_ptr_->at(REF_VEL)) *
                (x[DELTA] + pnc::mathlib::Cube(x[DELTA]) / 3.));
}

void LatAccCostTerm::GetGradientHessian(const State &x, const Control & /*u*/, LxMT &lx, LuMT & /*lu*/, LxxMT &lxx,
                                        LxuMT & /*lxu*/, LuuMT & /*luu*/) {
  lx(3) += cost_config_ptr_->at(W_ACC) *
           (Square(cost_config_ptr_->at(CURV_FACTOR) * Square(cost_config_ptr_->at(REF_VEL)))) *
           (1 + Square(x[DELTA])) * (x[DELTA] + pnc::mathlib::Cube(x[DELTA]) / 3.);
  lxx(3, 3) += cost_config_ptr_->at(W_ACC) *
               Square(cost_config_ptr_->at(CURV_FACTOR) * Square(cost_config_ptr_->at(REF_VEL))) *
               (1. + 4. * Square(x[DELTA]) + 5. / 3. * Square(Square(x[DELTA])));
}

double LatJerkCostTerm::GetCost(const State &x, const Control & /*u*/) {
  return 0.5 * cost_config_ptr_->at(W_JERK) *
         Square(cost_config_ptr_->at(CURV_FACTOR) * Square(cost_config_ptr_->at(REF_VEL)) *
                (x[OMEGA] + Square(x[DELTA]) * x[OMEGA]));
}

void LatJerkCostTerm::GetGradientHessian(const State &x, const Control & /*u*/, LxMT &lx, LuMT & /*lu*/, LxxMT &lxx,
                                         LxuMT & /*lxu*/, LuuMT & /*luu*/) {
  lx(3) += 2. * cost_config_ptr_->at(W_JERK) * x[DELTA] *
           Square(cost_config_ptr_->at(CURV_FACTOR) * x[OMEGA] * Square(cost_config_ptr_->at(REF_VEL))) *
           (1. + Square(x[DELTA]));
  lx(4) += cost_config_ptr_->at(W_JERK) * x[OMEGA] *
           Square(cost_config_ptr_->at(CURV_FACTOR) * Square(cost_config_ptr_->at(REF_VEL))) *
           Square(1 + Square(x[DELTA]));
  lxx(3, 3) += 2. * cost_config_ptr_->at(W_JERK) *
               Square(cost_config_ptr_->at(CURV_FACTOR) * x[OMEGA] * Square(cost_config_ptr_->at(REF_VEL))) *
               (1. + 3. * Square(x[DELTA]));
  const double lxx_delta_omega = 4. * cost_config_ptr_->at(W_JERK) * x[DELTA] * x[OMEGA] *
                                 Square(cost_config_ptr_->at(CURV_FACTOR) * Square(cost_config_ptr_->at(REF_VEL))) *
                                 (1. + Square(x[DELTA]));
  lxx(3, 4) += lxx_delta_omega;
  lxx(4, 3) += lxx_delta_omega;
  lxx(4, 4) += cost_config_ptr_->at(W_JERK) *
               Square(cost_config_ptr_->at(CURV_FACTOR) * Square(cost_config_ptr_->at(REF_VEL))) *
               Square(1 + Square(x[DELTA]));
}

double LatAccBoundCostTerm::GetCost(const State &x, const Control & /*u*/) {
  double cost = 0.;
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR)) * SSquare(cost_config_ptr_->at(REF_VEL));

  if (x[DELTA] > cost_config_ptr_->at(DELTA_BOUND)) {
    cost = 0.5 * cost_config_ptr_->at(W_ACC_BOUND) * k2v4 * Square(x[DELTA] - cost_config_ptr_->at(DELTA_BOUND));
  } else if (x[DELTA] < -cost_config_ptr_->at(DELTA_BOUND)) {
    cost = 0.5 * cost_config_ptr_->at(W_ACC_BOUND) * k2v4 * Square(-x[DELTA] - cost_config_ptr_->at(DELTA_BOUND));
  }
  return cost;
}

void LatAccBoundCostTerm::GetGradientHessian(const State &x, const Control & /*u*/, LxMT &lx, LuMT & /*lu*/, LxxMT &lxx,
                                             LxuMT & /*lxu*/, LuuMT & /*luu*/) {
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR)) * SSquare(cost_config_ptr_->at(REF_VEL));

  if (x[DELTA] > cost_config_ptr_->at(DELTA_BOUND)) {
    lx(3) += cost_config_ptr_->at(W_ACC_BOUND) * k2v4 * (x[DELTA] - cost_config_ptr_->at(DELTA_BOUND));
    lxx(3, 3) += cost_config_ptr_->at(W_ACC_BOUND) * k2v4;
  } else if (x[DELTA] < -cost_config_ptr_->at(DELTA_BOUND)) {
    lx(3) += -cost_config_ptr_->at(W_ACC_BOUND) * k2v4 * (-x[DELTA] - cost_config_ptr_->at(DELTA_BOUND));
    lxx(3, 3) += cost_config_ptr_->at(W_ACC_BOUND) * k2v4;
  }
}

double LatJerkBoundCostTerm::GetCost(const State &x, const Control & /*u*/) {
  double cost = 0.;
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR)) * SSquare(cost_config_ptr_->at(REF_VEL));
  if (x[OMEGA] > cost_config_ptr_->at(OMEGA_BOUND)) {
    cost = 0.5 * cost_config_ptr_->at(W_JERK_BOUND) * k2v4 * Square(x[OMEGA] - cost_config_ptr_->at(OMEGA_BOUND));
  } else if (x[OMEGA] < -cost_config_ptr_->at(OMEGA_BOUND)) {
    cost = 0.5 * cost_config_ptr_->at(W_JERK_BOUND) * k2v4 * Square(-x[OMEGA] - cost_config_ptr_->at(OMEGA_BOUND));
  }
  return cost;
}

void LatJerkBoundCostTerm::GetGradientHessian(const State &x, const Control & /*u*/, LxMT &lx, LuMT & /*lu*/,
                                              LxxMT &lxx, LxuMT & /*lxu*/, LuuMT & /*luu*/) {
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR)) * SSquare(cost_config_ptr_->at(REF_VEL));
  if (x[OMEGA] > cost_config_ptr_->at(OMEGA_BOUND)) {
    lx(4) += cost_config_ptr_->at(W_JERK_BOUND) * k2v4 * (x[OMEGA] - cost_config_ptr_->at(OMEGA_BOUND));
    lxx(4, 4) += cost_config_ptr_->at(W_JERK_BOUND) * k2v4;
  } else if (x[OMEGA] < -cost_config_ptr_->at(OMEGA_BOUND)) {
    lx(4) += -cost_config_ptr_->at(W_JERK_BOUND) * k2v4 * (-x[OMEGA] - cost_config_ptr_->at(OMEGA_BOUND));
    lxx(4, 4) += cost_config_ptr_->at(W_JERK_BOUND) * k2v4;
  }
}

double PathSoftCorridorCostTerm::GetCost(const State &x, const Control & /*u*/) {
  double cost = 0.0;
  // upper bound
  const double a1 = cost_config_ptr_->at(SOFT_UPPER_BOUND_Y1) - cost_config_ptr_->at(SOFT_UPPER_BOUND_Y0);
  const double b1 = cost_config_ptr_->at(SOFT_UPPER_BOUND_X0) - cost_config_ptr_->at(SOFT_UPPER_BOUND_X1);
  const double c1 = cost_config_ptr_->at(SOFT_UPPER_BOUND_Y0) * cost_config_ptr_->at(SOFT_UPPER_BOUND_X1) -
                    cost_config_ptr_->at(SOFT_UPPER_BOUND_X0) * cost_config_ptr_->at(SOFT_UPPER_BOUND_Y1);
  const double d1 = Square(cost_config_ptr_->at(SOFT_UPPER_BOUND_Y1) - cost_config_ptr_->at(SOFT_UPPER_BOUND_Y0)) +
                    Square(cost_config_ptr_->at(SOFT_UPPER_BOUND_X1) - cost_config_ptr_->at(SOFT_UPPER_BOUND_X0));

  if (a1 * x[X] + b1 * x[Y] + c1 < 0.) {
    cost = 0.5 * cost_config_ptr_->at(W_SOFT_CORRIDOR) * Square(a1 * x[X] + b1 * x[Y] + c1) / d1;
  }

  // lower bound
  const double a2 = cost_config_ptr_->at(SOFT_LOWER_BOUND_Y1) - cost_config_ptr_->at(SOFT_LOWER_BOUND_Y0);
  const double b2 = cost_config_ptr_->at(SOFT_LOWER_BOUND_X0) - cost_config_ptr_->at(SOFT_LOWER_BOUND_X1);
  const double c2 = cost_config_ptr_->at(SOFT_LOWER_BOUND_Y0) * cost_config_ptr_->at(SOFT_LOWER_BOUND_X1) -
                    cost_config_ptr_->at(SOFT_LOWER_BOUND_X0) * cost_config_ptr_->at(SOFT_LOWER_BOUND_Y1);
  const double d2 = Square(cost_config_ptr_->at(SOFT_LOWER_BOUND_Y1) - cost_config_ptr_->at(SOFT_LOWER_BOUND_Y0)) +
                    Square(cost_config_ptr_->at(SOFT_LOWER_BOUND_X1) - cost_config_ptr_->at(SOFT_LOWER_BOUND_X0));

  if (a2 * x[X] + b2 * x[Y] + c2 > 0.) {
    cost += 0.5 * cost_config_ptr_->at(W_SOFT_CORRIDOR) * Square(a2 * x[X] + b2 * x[Y] + c2) / d2;
  }

  return cost;
}

void PathSoftCorridorCostTerm::GetGradientHessian(const State &x, const Control & /*u*/, LxMT &lx, LuMT & /*lu*/,
                                                  LxxMT &lxx, LxuMT & /*lxu*/, LuuMT & /*luu*/) {
  // upper bound
  const double a1 = cost_config_ptr_->at(SOFT_UPPER_BOUND_Y1) - cost_config_ptr_->at(SOFT_UPPER_BOUND_Y0);
  const double b1 = cost_config_ptr_->at(SOFT_UPPER_BOUND_X0) - cost_config_ptr_->at(SOFT_UPPER_BOUND_X1);
  const double c1 = cost_config_ptr_->at(SOFT_UPPER_BOUND_Y0) * cost_config_ptr_->at(SOFT_UPPER_BOUND_X1) -
                    cost_config_ptr_->at(SOFT_UPPER_BOUND_X0) * cost_config_ptr_->at(SOFT_UPPER_BOUND_Y1);
  const double d1 = Square(cost_config_ptr_->at(SOFT_UPPER_BOUND_Y1) - cost_config_ptr_->at(SOFT_UPPER_BOUND_Y0)) +
                    Square(cost_config_ptr_->at(SOFT_UPPER_BOUND_X1) - cost_config_ptr_->at(SOFT_UPPER_BOUND_X0));

  if (a1 * x[X] + b1 * x[Y] + c1 < 0.) {
    lx(0) += cost_config_ptr_->at(W_SOFT_CORRIDOR) * a1 * (a1 * x[X] + b1 * x[Y] + c1) / d1;
    lx(1) += cost_config_ptr_->at(W_SOFT_CORRIDOR) * b1 * (a1 * x[X] + b1 * x[Y] + c1) / d1;
    lxx(0, 0) += cost_config_ptr_->at(W_SOFT_CORRIDOR) * Square(a1) / d1;
    lxx(1, 1) += cost_config_ptr_->at(W_SOFT_CORRIDOR) * Square(b1) / d1;
  }

  // lower bound
  const double a2 = cost_config_ptr_->at(SOFT_LOWER_BOUND_Y1) - cost_config_ptr_->at(SOFT_LOWER_BOUND_Y0);
  const double b2 = cost_config_ptr_->at(SOFT_LOWER_BOUND_X0) - cost_config_ptr_->at(SOFT_LOWER_BOUND_X1);
  const double c2 = cost_config_ptr_->at(SOFT_LOWER_BOUND_Y0) * cost_config_ptr_->at(SOFT_LOWER_BOUND_X1) -
                    cost_config_ptr_->at(SOFT_LOWER_BOUND_X0) * cost_config_ptr_->at(SOFT_LOWER_BOUND_Y1);
  const double d2 = Square(cost_config_ptr_->at(SOFT_LOWER_BOUND_Y1) - cost_config_ptr_->at(SOFT_LOWER_BOUND_Y0)) +
                    Square(cost_config_ptr_->at(SOFT_LOWER_BOUND_X1) - cost_config_ptr_->at(SOFT_LOWER_BOUND_X0));

  if (a2 * x[X] + b2 * x[Y] + c2 > 0.) {
    lx(0) += cost_config_ptr_->at(W_SOFT_CORRIDOR) * a2 * (a2 * x[X] + b2 * x[Y] + c2) / d2;
    lx(1) += cost_config_ptr_->at(W_SOFT_CORRIDOR) * b2 * (a2 * x[X] + b2 * x[Y] + c2) / d2;
    lxx(0, 0) += cost_config_ptr_->at(W_SOFT_CORRIDOR) * Square(a2) / d2;
    lxx(1, 1) += cost_config_ptr_->at(W_SOFT_CORRIDOR) * Square(b2) / d2;
  }
}

double LatSnapCostTerm::GetCost(const State & /*x*/, const Control &u) {
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR)) * SSquare(cost_config_ptr_->at(REF_VEL));
  return 0.5 * k2v4 * (cost_config_ptr_->at(W_SNAP) * Square(u[ControlId::OMEGA_DOT]));
}

void LatSnapCostTerm::GetGradientHessian(const State & /*x*/, const Control &u, LxMT & /*lx*/, LuMT &lu,
                                         LxxMT & /*lxx*/, LxuMT & /*lxu*/, LuuMT &luu) {
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR)) * SSquare(cost_config_ptr_->at(REF_VEL));
  lu(0) += cost_config_ptr_->at(W_SNAP) * k2v4 * u[ControlId::OMEGA_DOT];
  luu(0, 0) += cost_config_ptr_->at(W_SNAP) * k2v4;
}

}  // namespace lateral_planning
}  // namespace pnc
