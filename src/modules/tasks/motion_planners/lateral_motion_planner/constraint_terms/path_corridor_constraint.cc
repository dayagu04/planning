#include "path_corridor_constraint.h"

#include "math_lib.h"

using namespace pnc::mathlib;
namespace pnc {
namespace lateral_planning {

constexpr double kEps = 1e-6;

double PathFirstSoftCorridorCostTerm::GetCost(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/) {
  double cost = 0.0;
  // upper bound
  const double a1 = cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_Y1) -
                    cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_Y0);
  const double b1 = cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_X0) -
                    cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_X1);
  const double c1 = cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_Y0) *
                        cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_X1) -
                    cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_X0) *
                        cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_Y1);
  const double d1 = Square(a1) + Square(b1);
  // check direction continuity
  double ubound_result = 0;
  planning::planning_math::Vec2d cur_ubound_direction{
      cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_X1) -
          cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_X0),
      cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_Y1) -
          cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_Y0)};
  if (ubound_direction_.Length() > 0) {
    // ubound_result = cur_ubound_direction.InnerProd(ubound_direction_);
  }

  if (d1 > kEps && ubound_result >= 0) {
    ubound_direction_ = cur_ubound_direction;
    const double numerator1 = a1 * x[X] + b1 * x[Y] + c1;
    const double distance_to_soft_upper_bound =
        std::abs(numerator1) / std::sqrt(d1);

    if (numerator1 < 0. && distance_to_soft_upper_bound > kEps) {
      cost = 0.5 * cost_config_ptr_->at(W_FIRST_SOFT_CORRIDOR) *
             Square(distance_to_soft_upper_bound);
    }
  }

  // lower bound
  const double a2 = cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_Y1) -
                    cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_Y0);
  const double b2 = cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_X0) -
                    cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_X1);
  const double c2 = cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_Y0) *
                        cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_X1) -
                    cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_X0) *
                        cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_Y1);
  const double d2 = Square(a2) + Square(b2);
  // check direction continuity
  double lbound_result = 0;
  planning::planning_math::Vec2d cur_lbound_direction{
      cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_X1) -
          cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_X0),
      cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_Y1) -
          cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_Y0)};
  if (lbound_direction_.Length() > 0) {
    // lbound_result = cur_lbound_direction.InnerProd(lbound_direction_);
  }

  if (d2 > kEps && lbound_result >= 0) {
    lbound_direction_ = cur_lbound_direction;
    const double numerator2 = a2 * x[X] + b2 * x[Y] + c2;
    const double distance_to_soft_lower_bound =
        std::abs(numerator2) / std::sqrt(d2);

    if (numerator2 > 0. && distance_to_soft_lower_bound > kEps) {
      cost += 0.5 * cost_config_ptr_->at(W_FIRST_SOFT_CORRIDOR) *
              Square(distance_to_soft_lower_bound);
    }
  }
  cost_value_ += cost;
  return cost;
}

void PathFirstSoftCorridorCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT & /*lxu*/, ilqr_solver::LuuMT & /*luu*/) {
  // upper bound
  const double a1 = cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_Y1) -
                    cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_Y0);
  const double b1 = cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_X0) -
                    cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_X1);
  const double c1 = cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_Y0) *
                        cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_X1) -
                    cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_X0) *
                        cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_Y1);
  const double d1 = Square(a1) + Square(b1);

  // check direction continuity
  double ubound_result = 0;
  planning::planning_math::Vec2d cur_ubound_direction{
      cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_X1) -
          cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_X0),
      cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_Y1) -
          cost_config_ptr_->at(FIRST_SOFT_UPPER_BOUND_Y0)};
  if (ubound_direction_.Length() > 0) {
    // ubound_result = cur_ubound_direction.InnerProd(ubound_direction_);
  }

  if (d1 > kEps && ubound_result >= 0) {
    ubound_direction_ = cur_ubound_direction;
    const double numerator1 = a1 * x[X] + b1 * x[Y] + c1;
    const double distance_to_soft_upper_bound =
        std::abs(numerator1) / std::sqrt(d1);
    if (numerator1 < 0. && distance_to_soft_upper_bound > kEps) {
      lx(X) += cost_config_ptr_->at(W_FIRST_SOFT_CORRIDOR) * a1 *
               (a1 * x[X] + b1 * x[Y] + c1) / d1;
      lx(Y) += cost_config_ptr_->at(W_FIRST_SOFT_CORRIDOR) * b1 *
               (a1 * x[X] + b1 * x[Y] + c1) / d1;
      lxx(X, X) +=
          cost_config_ptr_->at(W_FIRST_SOFT_CORRIDOR) * Square(a1) / d1;
      lxx(Y, Y) +=
          cost_config_ptr_->at(W_FIRST_SOFT_CORRIDOR) * Square(b1) / d1;
    }
  }

  // lower bound
  const double a2 = cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_Y1) -
                    cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_Y0);
  const double b2 = cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_X0) -
                    cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_X1);
  const double c2 = cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_Y0) *
                        cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_X1) -
                    cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_X0) *
                        cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_Y1);
  const double d2 = Square(a2) + Square(b2);
  // check direction continuity
  double lbound_result = 0;
  planning::planning_math::Vec2d cur_lbound_direction{
      cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_X1) -
          cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_X0),
      cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_Y1) -
          cost_config_ptr_->at(FIRST_SOFT_LOWER_BOUND_Y0)};
  if (lbound_direction_.Length() > 0) {
    // lbound_result = cur_lbound_direction.InnerProd(lbound_direction_);
  }

  if (d2 > kEps && lbound_result >= 0) {
    lbound_direction_ = cur_lbound_direction;
    const double numerator2 = a2 * x[X] + b2 * x[Y] + c2;
    const double distance_to_soft_lower_bound =
        std::abs(numerator2) / std::sqrt(d2);
    if (numerator2 > 0. && distance_to_soft_lower_bound > kEps) {
      lx(X) += cost_config_ptr_->at(W_FIRST_SOFT_CORRIDOR) * a2 *
               (a2 * x[X] + b2 * x[Y] + c2) / d2;
      lx(Y) += cost_config_ptr_->at(W_FIRST_SOFT_CORRIDOR) * b2 *
               (a2 * x[X] + b2 * x[Y] + c2) / d2;
      lxx(X, X) +=
          cost_config_ptr_->at(W_FIRST_SOFT_CORRIDOR) * Square(a2) / d2;
      lxx(Y, Y) +=
          cost_config_ptr_->at(W_FIRST_SOFT_CORRIDOR) * Square(b2) / d2;
    }
  }
}

double PathSecondSoftCorridorCostTerm::GetCost(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/) {
  double cost = 0.0;
  // upper bound
  const double a1 = cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_Y1) -
                    cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_Y0);
  const double b1 = cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_X0) -
                    cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_X1);
  const double c1 = cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_Y0) *
                        cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_X1) -
                    cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_X0) *
                        cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_Y1);
  const double d1 = Square(a1) + Square(b1);
  // check direction continuity
  double ubound_result = 0;
  planning::planning_math::Vec2d cur_ubound_direction{
      cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_X1) -
          cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_X0),
      cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_Y1) -
          cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_Y0)};
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
  const double a2 = cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_Y1) -
                    cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_Y0);
  const double b2 = cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_X0) -
                    cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_X1);
  const double c2 = cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_Y0) *
                        cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_X1) -
                    cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_X0) *
                        cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_Y1);
  const double d2 = Square(a2) + Square(b2);
  // check direction continuity
  double lbound_result = 0;
  planning::planning_math::Vec2d cur_lbound_direction{
      cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_X1) -
          cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_X0),
      cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_Y1) -
          cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_Y0)};
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
  cost_value_ += cost;
  return cost;
}

void PathSecondSoftCorridorCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT & /*lxu*/, ilqr_solver::LuuMT & /*luu*/) {
  // upper bound
  const double a1 = cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_Y1) -
                    cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_Y0);
  const double b1 = cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_X0) -
                    cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_X1);
  const double c1 = cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_Y0) *
                        cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_X1) -
                    cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_X0) *
                        cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_Y1);
  const double d1 = Square(a1) + Square(b1);

  // check direction continuity
  double ubound_result = 0;
  planning::planning_math::Vec2d cur_ubound_direction{
      cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_X1) -
          cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_X0),
      cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_Y1) -
          cost_config_ptr_->at(SECOND_SOFT_UPPER_BOUND_Y0)};
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
  const double a2 = cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_Y1) -
                    cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_Y0);
  const double b2 = cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_X0) -
                    cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_X1);
  const double c2 = cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_Y0) *
                        cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_X1) -
                    cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_X0) *
                        cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_Y1);
  const double d2 = Square(a2) + Square(b2);
  // check direction continuity
  double lbound_result = 0;
  planning::planning_math::Vec2d cur_lbound_direction{
      cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_X1) -
          cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_X0),
      cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_Y1) -
          cost_config_ptr_->at(SECOND_SOFT_LOWER_BOUND_Y0)};
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

double PathHardCorridorCostTerm::GetCost(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/) {
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
             Square(distance_to_hard_upper_bound);
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
              Square(distance_to_hard_lower_bound);
    }
  }
  cost_value_ += cost;
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
    if (numerator1 < 0. && distance_to_hard_upper_bound > kEps) {
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

double PathCorridorCostTerm::GetCost(const ilqr_solver::State &x,
                                     const ilqr_solver::Control &) {
  double cost = 0.;
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
    const double upper_cost_var =
        -(a1 * x[X] + b1 * x[Y] + c1) +
        alilqr_config_ptr_->at(L_POS_UPPER_HARDBOUND) /
        alilqr_config_ptr_->at(W_POS_UPPER_HARDBOUND);
    if (upper_cost_var < 0.) {
      cost += 0.5 * alilqr_config_ptr_->at(W_POS_UPPER_HARDBOUND) *
              Square(upper_cost_var);
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
    const double lower_cost_var =
        a2 * x[X] + b2 * x[Y] + c2 +
        alilqr_config_ptr_->at(L_POS_LOWER_HARDBOUND) /
        alilqr_config_ptr_->at(W_POS_LOWER_HARDBOUND);
    if (lower_cost_var < 0.) {
      cost += 0.5 * cost_config_ptr_->at(W_POS_LOWER_HARDBOUND) *
              Square(lower_cost_var);
    }
  }

  return cost;
}

void PathCorridorCostTerm::GetGradientHessian(
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
    const double upper_cost_var =
        -(a1 * x[X] + b1 * x[Y] + c1) +
        alilqr_config_ptr_->at(L_POS_UPPER_HARDBOUND) /
        alilqr_config_ptr_->at(W_POS_UPPER_HARDBOUND);
    if (upper_cost_var < 0.) {
      lx(X) += -alilqr_config_ptr_->at(W_POS_UPPER_HARDBOUND) *
               upper_cost_var * a1;
      lx(Y) += -alilqr_config_ptr_->at(W_POS_UPPER_HARDBOUND) *
               upper_cost_var * b1;
      lxx(X, X) +=
          alilqr_config_ptr_->at(W_POS_UPPER_HARDBOUND) * Square(a1);
      lxx(Y, Y) +=
          alilqr_config_ptr_->at(W_POS_UPPER_HARDBOUND) * Square(b1);
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
    const double lower_cost_var =
        a2 * x[X] + b2 * x[Y] + c2 +
        alilqr_config_ptr_->at(L_POS_LOWER_HARDBOUND) /
        alilqr_config_ptr_->at(W_POS_LOWER_HARDBOUND);
    if (lower_cost_var < 0.) {
      lx(X) += alilqr_config_ptr_->at(W_POS_LOWER_HARDBOUND) *
               lower_cost_var * a2;
      lx(Y) += alilqr_config_ptr_->at(W_POS_LOWER_HARDBOUND) *
               lower_cost_var * b2;
      lxx(X, X) +=
          alilqr_config_ptr_->at(W_POS_LOWER_HARDBOUND) * Square(a2);
      lxx(Y, Y) +=
          alilqr_config_ptr_->at(W_POS_LOWER_HARDBOUND) * Square(b1);
    }
  }

}

}  // namespace lateral_planning
}  // namespace pnc