#include "lateral_motion_planning_cost.h"

#include "math_lib.h"

using namespace pnc::mathlib;
namespace pnc {
namespace lateral_planning {
static const double kEps = 1e-6;

double ReferenceCostTerm::GetCost(const ilqr_solver::State &x,
                                  const ilqr_solver::Control & /*u*/) {
  const double cost_x = 0.5 * cost_config_ptr_->at(W_REF_X) *
                        Square(x[X] - cost_config_ptr_->at(REF_X));
  const double cost_y = 0.5 * cost_config_ptr_->at(W_REF_Y) *
                        Square(x[Y] - cost_config_ptr_->at(REF_Y));
  const double cost_theta = 0.5 * cost_config_ptr_->at(W_REF_THETA) *
                            Square(x[THETA] - cost_config_ptr_->at(REF_THETA));

  return cost_x + cost_y + cost_theta;
}

void ReferenceCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT & /*lxu*/, ilqr_solver::LuuMT & /*luu*/) {
  lx(X) +=
      -cost_config_ptr_->at(W_REF_X) * (cost_config_ptr_->at(REF_X) - x[X]);
  lx(Y) +=
      -cost_config_ptr_->at(W_REF_Y) * (cost_config_ptr_->at(REF_Y) - x[Y]);
  lx(THETA) += -cost_config_ptr_->at(W_REF_THETA) *
               (cost_config_ptr_->at(REF_THETA) - x[THETA]);

  lxx(X, X) += cost_config_ptr_->at(W_REF_X);
  lxx(Y, Y) += cost_config_ptr_->at(W_REF_Y);
  lxx(THETA, THETA) += cost_config_ptr_->at(W_REF_THETA);
}

double ContinuityCostTerm::GetCost(const ilqr_solver::State &x,
                                   const ilqr_solver::Control & /*u*/) {
  const double cost =
      0.5 * (cost_config_ptr_->at(W_CONTINUITY_X) *
                 Square(x[X] - cost_config_ptr_->at(CONTINUITY_X)) +
             cost_config_ptr_->at(W_CONTINUITY_Y) *
                 Square(x[Y] - cost_config_ptr_->at(CONTINUITY_Y)) +
             cost_config_ptr_->at(W_CONTINUITY_THETA) *
                 Square(x[THETA] - cost_config_ptr_->at(CONTINUITY_THETA)));

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

double LatAccCostTerm::GetCost(const ilqr_solver::State &x,
                               const ilqr_solver::Control & /*u*/) {
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR) *
                             Square(cost_config_ptr_->at(EGO_VEL)));
  const double &delta = x[DELTA] - cost_config_ptr_->at(EXPECTEDE_DELTA);

  const double cost =
      0.5 * cost_config_ptr_->at(W_ACC) * k2v4 * (delta * delta);

  return cost;
}

void LatAccCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT & /*lxu*/, ilqr_solver::LuuMT & /*luu*/) {
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR) *
                             Square(cost_config_ptr_->at(EGO_VEL)));
  const double &delta = x[DELTA] - cost_config_ptr_->at(EXPECTEDE_DELTA);

  lx(DELTA) += cost_config_ptr_->at(W_ACC) * k2v4 * delta;
  lxx(DELTA, DELTA) += cost_config_ptr_->at(W_ACC) * k2v4;
}

double LatJerkCostTerm::GetCost(const ilqr_solver::State &x,
                                const ilqr_solver::Control &u) {
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR) *
                             Square(cost_config_ptr_->at(EGO_VEL)));
  const double &omega = u[OMEGA];

  const double cost =
      0.5 * cost_config_ptr_->at(W_JERK) * k2v4 * (omega * omega);

  return cost;
}

void LatJerkCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control &u,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT &lu, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT & /*lxu*/, ilqr_solver::LuuMT &luu) {
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR) *
                             Square(cost_config_ptr_->at(EGO_VEL)));
  const double &omega = u[OMEGA];

  lu(OMEGA) = cost_config_ptr_->at(W_JERK) * k2v4 * omega;
  luu(OMEGA, OMEGA) = cost_config_ptr_->at(W_JERK) * k2v4;
}

double LatAccBoundCostTerm::GetCost(const ilqr_solver::State &x,
                                    const ilqr_solver::Control & /*u*/) {
  double cost = 0.;
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR)) *
                      SSquare(cost_config_ptr_->at(EGO_VEL));

  if (x[DELTA] > cost_config_ptr_->at(DELTA_UPPER_BOUND)) {
    cost = 0.5 * cost_config_ptr_->at(W_ACC_BOUND) * k2v4 *
           Square(x[DELTA] - cost_config_ptr_->at(DELTA_UPPER_BOUND));
  } else if (x[DELTA] < cost_config_ptr_->at(DELTA_LOWER_BOUND)) {
    cost = 0.5 * cost_config_ptr_->at(W_ACC_BOUND) * k2v4 *
           Square(x[DELTA] - cost_config_ptr_->at(DELTA_LOWER_BOUND));
  }

  return cost;
}

void LatAccBoundCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT & /*lxu*/, ilqr_solver::LuuMT & /*luu*/) {
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR)) *
                      SSquare(cost_config_ptr_->at(EGO_VEL));

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

double LatJerkBoundCostTerm::GetCost(const ilqr_solver::State &x,
                                     const ilqr_solver::Control &u) {
  double cost = 0.;
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR)) *
                      SSquare(cost_config_ptr_->at(EGO_VEL));
  if (u[OMEGA] > cost_config_ptr_->at(OMEGA_UPPER_BOUND)) {
    cost = 0.5 * cost_config_ptr_->at(W_JERK_BOUND) * k2v4 *
           Square(u[OMEGA] - cost_config_ptr_->at(OMEGA_UPPER_BOUND));
  } else if (u[OMEGA] < cost_config_ptr_->at(OMEGA_LOWER_BOUND)) {
    cost = 0.5 * cost_config_ptr_->at(W_JERK_BOUND) * k2v4 *
           Square(u[OMEGA] - cost_config_ptr_->at(OMEGA_LOWER_BOUND));
  }
  return cost;
}

void LatJerkBoundCostTerm::GetGradientHessian(const ilqr_solver::State &x,
                                              const ilqr_solver::Control &u,
                                              ilqr_solver::LxMT & /*lx*/,
                                              ilqr_solver::LuMT &lu,
                                              ilqr_solver::LxxMT & /*lxx*/,
                                              ilqr_solver::LxuMT & /*lxu*/,
                                              ilqr_solver::LuuMT &luu) {
  const double k2v4 = Square(cost_config_ptr_->at(CURV_FACTOR)) *
                      SSquare(cost_config_ptr_->at(EGO_VEL));
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

double PathSoftCorridorCostTerm::GetCost(const ilqr_solver::State &x,
                                         const ilqr_solver::Control & /*u*/) {
  double cost = 0.0;
  // upper bound
  const double a1 = cost_config_ptr_->at(SOFT_UPPER_BOUND_Y1) -
                    cost_config_ptr_->at(SOFT_UPPER_BOUND_Y0);
  const double b1 = cost_config_ptr_->at(SOFT_UPPER_BOUND_X0) -
                    cost_config_ptr_->at(SOFT_UPPER_BOUND_X1);
  const double c1 = cost_config_ptr_->at(SOFT_UPPER_BOUND_Y0) *
                        cost_config_ptr_->at(SOFT_UPPER_BOUND_X1) -
                    cost_config_ptr_->at(SOFT_UPPER_BOUND_X0) *
                        cost_config_ptr_->at(SOFT_UPPER_BOUND_Y1);
  const double d1 = Square(a1) + Square(b1);
  // check direction continuity
  double ubound_result = 0;
  planning::planning_math::Vec2d cur_ubound_direction{
    cost_config_ptr_->at(SOFT_UPPER_BOUND_X1) -
    cost_config_ptr_->at(SOFT_UPPER_BOUND_X0),
    cost_config_ptr_->at(SOFT_UPPER_BOUND_Y1) -
    cost_config_ptr_->at(SOFT_UPPER_BOUND_Y0)};
  if (ubound_direction_.Length() > 0) {
    // ubound_result = cur_ubound_direction.InnerProd(ubound_direction_);
  }

  if (d1 > kEps && ubound_result >= 0) {
    ubound_direction_ = cur_ubound_direction;
    const double numerator1 = a1 * x[X] + b1 * x[Y] + c1;
    const double distance_to_soft_upper_bound =
        std::abs(numerator1) / std::sqrt(d1);

    if (numerator1 < 0. && distance_to_soft_upper_bound > kEps) {
      cost = 0.5 * cost_config_ptr_->at(W_SOFT_CORRIDOR) *
             Square(distance_to_soft_upper_bound);
    }
  }

  // lower bound
  const double a2 = cost_config_ptr_->at(SOFT_LOWER_BOUND_Y1) -
                    cost_config_ptr_->at(SOFT_LOWER_BOUND_Y0);
  const double b2 = cost_config_ptr_->at(SOFT_LOWER_BOUND_X0) -
                    cost_config_ptr_->at(SOFT_LOWER_BOUND_X1);
  const double c2 = cost_config_ptr_->at(SOFT_LOWER_BOUND_Y0) *
                        cost_config_ptr_->at(SOFT_LOWER_BOUND_X1) -
                    cost_config_ptr_->at(SOFT_LOWER_BOUND_X0) *
                        cost_config_ptr_->at(SOFT_LOWER_BOUND_Y1);
  const double d2 = Square(a2) + Square(b2);
  // check direction continuity
  double lbound_result = 0;
  planning::planning_math::Vec2d cur_lbound_direction{
    cost_config_ptr_->at(SOFT_LOWER_BOUND_X1) -
    cost_config_ptr_->at(SOFT_LOWER_BOUND_X0),
    cost_config_ptr_->at(SOFT_LOWER_BOUND_Y1) -
    cost_config_ptr_->at(SOFT_LOWER_BOUND_Y0)};
  if (lbound_direction_.Length() > 0) {
    // lbound_result = cur_lbound_direction.InnerProd(lbound_direction_);
  }

  if (d2 > kEps && lbound_result >= 0) {
    lbound_direction_ = cur_lbound_direction;
    const double numerator2 = a2 * x[X] + b2 * x[Y] + c2;
    const double distance_to_soft_lower_bound =
        std::abs(numerator2) / std::sqrt(d2);

    if (numerator2 > 0. && distance_to_soft_lower_bound > kEps) {
      cost += 0.5 * cost_config_ptr_->at(W_SOFT_CORRIDOR) *
              Square(distance_to_soft_lower_bound);
    }
  }

  return cost;
}

void PathSoftCorridorCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT & /*lxu*/, ilqr_solver::LuuMT & /*luu*/) {
  // upper bound
  const double a1 = cost_config_ptr_->at(SOFT_UPPER_BOUND_Y1) -
                    cost_config_ptr_->at(SOFT_UPPER_BOUND_Y0);
  const double b1 = cost_config_ptr_->at(SOFT_UPPER_BOUND_X0) -
                    cost_config_ptr_->at(SOFT_UPPER_BOUND_X1);
  const double c1 = cost_config_ptr_->at(SOFT_UPPER_BOUND_Y0) *
                        cost_config_ptr_->at(SOFT_UPPER_BOUND_X1) -
                    cost_config_ptr_->at(SOFT_UPPER_BOUND_X0) *
                        cost_config_ptr_->at(SOFT_UPPER_BOUND_Y1);
  const double d1 = Square(a1) + Square(b1);

  // check direction continuity
  double ubound_result = 0;
  planning::planning_math::Vec2d cur_ubound_direction{
    cost_config_ptr_->at(SOFT_UPPER_BOUND_X1) -
    cost_config_ptr_->at(SOFT_UPPER_BOUND_X0),
    cost_config_ptr_->at(SOFT_UPPER_BOUND_Y1) -
    cost_config_ptr_->at(SOFT_UPPER_BOUND_Y0)};
  if (ubound_direction_.Length() > 0) {
    // ubound_result = cur_ubound_direction.InnerProd(ubound_direction_);
  }

  if (d1 > kEps && ubound_result >= 0) {
    ubound_direction_ = cur_ubound_direction;
    const double numerator1 = a1 * x[X] + b1 * x[Y] + c1;
    const double distance_to_soft_upper_bound =
        std::abs(numerator1) / std::sqrt(d1);
    if (numerator1 < 0. && distance_to_soft_upper_bound > kEps) {
      lx(X) += cost_config_ptr_->at(W_SOFT_CORRIDOR) * a1 *
               (a1 * x[X] + b1 * x[Y] + c1) / d1;
      lx(Y) += cost_config_ptr_->at(W_SOFT_CORRIDOR) * b1 *
               (a1 * x[X] + b1 * x[Y] + c1) / d1;
      lxx(X, X) += cost_config_ptr_->at(W_SOFT_CORRIDOR) * Square(a1) / d1;
      lxx(Y, Y) += cost_config_ptr_->at(W_SOFT_CORRIDOR) * Square(b1) / d1;
    }
  }

  // lower bound
  const double a2 = cost_config_ptr_->at(SOFT_LOWER_BOUND_Y1) -
                    cost_config_ptr_->at(SOFT_LOWER_BOUND_Y0);
  const double b2 = cost_config_ptr_->at(SOFT_LOWER_BOUND_X0) -
                    cost_config_ptr_->at(SOFT_LOWER_BOUND_X1);
  const double c2 = cost_config_ptr_->at(SOFT_LOWER_BOUND_Y0) *
                        cost_config_ptr_->at(SOFT_LOWER_BOUND_X1) -
                    cost_config_ptr_->at(SOFT_LOWER_BOUND_X0) *
                        cost_config_ptr_->at(SOFT_LOWER_BOUND_Y1);
  const double d2 = Square(a2) + Square(b2);
  // check direction continuity
  double lbound_result = 0;
  planning::planning_math::Vec2d cur_lbound_direction{
    cost_config_ptr_->at(SOFT_LOWER_BOUND_X1) -
    cost_config_ptr_->at(SOFT_LOWER_BOUND_X0),
    cost_config_ptr_->at(SOFT_LOWER_BOUND_Y1) -
    cost_config_ptr_->at(SOFT_LOWER_BOUND_Y0)};
  if (lbound_direction_.Length() > 0) {
    // lbound_result = cur_lbound_direction.InnerProd(lbound_direction_);
  }

  if (d2 > kEps && lbound_result >= 0) {
    lbound_direction_ = cur_lbound_direction;
    const double numerator2 = a2 * x[X] + b2 * x[Y] + c2;
    const double distance_to_soft_lower_bound =
        std::abs(numerator2) / std::sqrt(d2);
    if (numerator2 > 0. && distance_to_soft_lower_bound > kEps) {
      lx(X) += cost_config_ptr_->at(W_SOFT_CORRIDOR) * a2 *
               (a2 * x[X] + b2 * x[Y] + c2) / d2;
      lx(Y) += cost_config_ptr_->at(W_SOFT_CORRIDOR) * b2 *
               (a2 * x[X] + b2 * x[Y] + c2) / d2;
      lxx(X, X) += cost_config_ptr_->at(W_SOFT_CORRIDOR) * Square(a2) / d2;
      lxx(Y, Y) += cost_config_ptr_->at(W_SOFT_CORRIDOR) * Square(b2) / d2;
    }
  }
}

double PathHardCorridorCostTerm::GetCost(const ilqr_solver::State &x,
                                         const ilqr_solver::Control & /*u*/) {
  double cost = 0.0;
  // upper bound
  const double a1 = cost_config_ptr_->at(HARD_UPPER_BOUND_Y1) -
                    cost_config_ptr_->at(HARD_UPPER_BOUND_Y0);
  const double b1 = cost_config_ptr_->at(HARD_UPPER_BOUND_X0) -
                    cost_config_ptr_->at(HARD_UPPER_BOUND_X1);
  const double c1 = cost_config_ptr_->at(HARD_UPPER_BOUND_Y0) *
                        cost_config_ptr_->at(HARD_UPPER_BOUND_X1) -
                    cost_config_ptr_->at(HARD_UPPER_BOUND_X0) *
                        cost_config_ptr_->at(HARD_UPPER_BOUND_Y1);
  const double d1 = Square(a1) + Square(b1);
  // check direction continuity
  double ubound_result = 0;
  planning::planning_math::Vec2d cur_ubound_direction{
    cost_config_ptr_->at(HARD_UPPER_BOUND_X1) -
    cost_config_ptr_->at(HARD_UPPER_BOUND_X0),
    cost_config_ptr_->at(HARD_UPPER_BOUND_Y1) -
    cost_config_ptr_->at(HARD_UPPER_BOUND_Y0)};
  if (ubound_direction_.Length() > 0) {
    // ubound_result = cur_ubound_direction.InnerProd(ubound_direction_);
  }

  if (d1 > kEps && ubound_result >= 0) {
    ubound_direction_ = cur_ubound_direction;
    const double numerator1 = a1 * x[X] + b1 * x[Y] + c1;
    const double distance_to_hard_upper_bound =
        std::abs(numerator1) / std::sqrt(d1);
    if (numerator1 < 0. && distance_to_hard_upper_bound > kEps) {
      cost = 0.5 * cost_config_ptr_->at(W_HARD_CORRIDOR) *
             Square(a1 * x[X] + b1 * x[Y] + c1) / d1;
    }
  }

  // lower bound
  const double a2 = cost_config_ptr_->at(HARD_LOWER_BOUND_Y1) -
                    cost_config_ptr_->at(HARD_LOWER_BOUND_Y0);
  const double b2 = cost_config_ptr_->at(HARD_LOWER_BOUND_X0) -
                    cost_config_ptr_->at(HARD_LOWER_BOUND_X1);
  const double c2 = cost_config_ptr_->at(HARD_LOWER_BOUND_Y0) *
                        cost_config_ptr_->at(HARD_LOWER_BOUND_X1) -
                    cost_config_ptr_->at(HARD_LOWER_BOUND_X0) *
                        cost_config_ptr_->at(HARD_LOWER_BOUND_Y1);
  const double d2 = Square(a2) + Square(b2);
  // check direction continuity
  double lbound_result = 0;
  planning::planning_math::Vec2d cur_lbound_direction{
    cost_config_ptr_->at(HARD_LOWER_BOUND_X1) -
    cost_config_ptr_->at(HARD_LOWER_BOUND_X0),
    cost_config_ptr_->at(HARD_LOWER_BOUND_Y1) -
    cost_config_ptr_->at(HARD_LOWER_BOUND_Y0)};
  if (lbound_direction_.Length() > 0) {
    // lbound_result = cur_lbound_direction.InnerProd(lbound_direction_);
  }

  if (d2 > kEps && lbound_result >= 0) {
    lbound_direction_ = cur_lbound_direction;
    const double numerator2 = a2 * x[X] + b2 * x[Y] + c2;
    const double distance_to_hard_lower_bound =
        std::abs(numerator2) / std::sqrt(d2);
    if (numerator2 > 0. && distance_to_hard_lower_bound > kEps) {
      cost += 0.5 * cost_config_ptr_->at(W_HARD_CORRIDOR) *
              Square(a2 * x[X] + b2 * x[Y] + c2) / d2;
    }
  }

  return cost;
}

void PathHardCorridorCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT & /*lxu*/, ilqr_solver::LuuMT & /*luu*/) {
  // upper bound
  const double a1 = cost_config_ptr_->at(HARD_UPPER_BOUND_Y1) -
                    cost_config_ptr_->at(HARD_UPPER_BOUND_Y0);
  const double b1 = cost_config_ptr_->at(HARD_UPPER_BOUND_X0) -
                    cost_config_ptr_->at(HARD_UPPER_BOUND_X1);
  const double c1 = cost_config_ptr_->at(HARD_UPPER_BOUND_Y0) *
                        cost_config_ptr_->at(HARD_UPPER_BOUND_X1) -
                    cost_config_ptr_->at(HARD_UPPER_BOUND_X0) *
                        cost_config_ptr_->at(HARD_UPPER_BOUND_Y1);
  const double d1 = Square(a1) + Square(b1);
  // check direction continuity
  double ubound_result = 0;
  planning::planning_math::Vec2d cur_ubound_direction{
    cost_config_ptr_->at(HARD_UPPER_BOUND_X1) -
    cost_config_ptr_->at(HARD_UPPER_BOUND_X0),
    cost_config_ptr_->at(HARD_UPPER_BOUND_Y1) -
    cost_config_ptr_->at(HARD_UPPER_BOUND_Y0)};
  if (ubound_direction_.Length() > 0) {
    // ubound_result = cur_ubound_direction.InnerProd(ubound_direction_);
  }

  if (d1 > kEps && ubound_result >= 0) {
    ubound_direction_ = cur_ubound_direction;
    const double numerator1 = a1 * x[X] + b1 * x[Y] + c1;
    const double distance_to_hard_upper_bound =
        std::abs(numerator1) / std::sqrt(d1);
    if (numerator1 < 0. && numerator1 > kEps) {
      lx(X) += cost_config_ptr_->at(W_HARD_CORRIDOR) * a1 *
               (a1 * x[X] + b1 * x[Y] + c1) / d1;
      lx(Y) += cost_config_ptr_->at(W_HARD_CORRIDOR) * b1 *
               (a1 * x[X] + b1 * x[Y] + c1) / d1;
      lxx(X, X) += cost_config_ptr_->at(W_HARD_CORRIDOR) * Square(a1) / d1;
      lxx(Y, Y) += cost_config_ptr_->at(W_HARD_CORRIDOR) * Square(b1) / d1;
    }
  }

  // lower bound
  const double a2 = cost_config_ptr_->at(HARD_LOWER_BOUND_Y1) -
                    cost_config_ptr_->at(HARD_LOWER_BOUND_Y0);
  const double b2 = cost_config_ptr_->at(HARD_LOWER_BOUND_X0) -
                    cost_config_ptr_->at(HARD_LOWER_BOUND_X1);
  const double c2 = cost_config_ptr_->at(HARD_LOWER_BOUND_Y0) *
                        cost_config_ptr_->at(HARD_LOWER_BOUND_X1) -
                    cost_config_ptr_->at(HARD_LOWER_BOUND_X0) *
                        cost_config_ptr_->at(HARD_LOWER_BOUND_Y1);
  const double d2 = Square(a2) + Square(b2);
  // check direction continuity
  double lbound_result = 0;
  planning::planning_math::Vec2d cur_lbound_direction{
    cost_config_ptr_->at(HARD_LOWER_BOUND_X1) -
    cost_config_ptr_->at(HARD_LOWER_BOUND_X0),
    cost_config_ptr_->at(HARD_LOWER_BOUND_Y1) -
    cost_config_ptr_->at(HARD_LOWER_BOUND_Y0)};
  if (lbound_direction_.Length() > 0) {
    // lbound_result = cur_lbound_direction.InnerProd(lbound_direction_);
  }

  if (d2 > kEps && lbound_result >= 0) {
    lbound_direction_ = cur_lbound_direction;
    const double numerator2 = a2 * x[X] + b2 * x[Y] + c2;
    const double distance_to_hard_lower_bound =
        std::abs(numerator2) / std::sqrt(d2);
    if (numerator2 > 0. && distance_to_hard_lower_bound > kEps) {
      lx(X) += cost_config_ptr_->at(W_HARD_CORRIDOR) * a2 *
              (a2 * x[X] + b2 * x[Y] + c2) / d2;
      lx(Y) += cost_config_ptr_->at(W_HARD_CORRIDOR) * b2 *
              (a2 * x[X] + b2 * x[Y] + c2) / d2;
      lxx(X, X) += cost_config_ptr_->at(W_HARD_CORRIDOR) * Square(a2) / d2;
      lxx(Y, Y) += cost_config_ptr_->at(W_HARD_CORRIDOR) * Square(b2) / d2;
    }
  }
}

}  // namespace lateral_planning
}  // namespace pnc
