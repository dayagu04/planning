#include "reference_path_cost.h"

#include "math_lib.h"

using namespace pnc::mathlib;
namespace pnc {
namespace lateral_planning {

double ReferencePathCostTerm::GetCost(const ilqr_solver::State &x,
                                      const ilqr_solver::Control & /*u*/) {
  double cost = 0.;
  const double cost_x = 0.5 * cost_config_ptr_->at(W_REF_X) *
                        Square(x[X] - cost_config_ptr_->at(REF_X));
  const double cost_y = 0.5 * cost_config_ptr_->at(W_REF_Y) *
                        Square(x[Y] - cost_config_ptr_->at(REF_Y));
  const double cost_theta = 0.5 * cost_config_ptr_->at(W_REF_THETA) *
                            Square(x[THETA] - cost_config_ptr_->at(REF_THETA));
  cost = cost_x + cost_y + cost_theta;
  cost_value_ += cost;
  return cost;
}

void ReferencePathCostTerm::GetGradientHessian(
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

double FrontReferencePathCostTerm::GetCost(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/) {
  double cost = 0.;
  double head_x = x[X] + cost_config_ptr_->at(WHEEL_BASE) * std::cos(x[THETA]);
  double head_y = x[Y] + cost_config_ptr_->at(WHEEL_BASE) * std::sin(x[THETA]);
  const double cost_x = 0.5 * cost_config_ptr_->at(W_FRONT_REF_X) *
                        Square(head_x - cost_config_ptr_->at(FRONT_REF_X));
  const double cost_y = 0.5 * cost_config_ptr_->at(W_FRONT_REF_Y) *
                        Square(head_y - cost_config_ptr_->at(FRONT_REF_Y));
  // const double cost_theta = 0.5 * cost_config_ptr_->at(W_REF_THETA) *
  //                           Square(x[THETA] -
  //                           cost_config_ptr_->at(HEAD_REF_THETA));
  cost = cost_x + cost_y;
  cost_value_ += cost;
  return cost;
}

void FrontReferencePathCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT & /*lxu*/, ilqr_solver::LuuMT & /*luu*/) {
  double head_x = x[X] + cost_config_ptr_->at(WHEEL_BASE) * std::cos(x[THETA]);
  double head_y = x[Y] + cost_config_ptr_->at(WHEEL_BASE) * std::sin(x[THETA]);
  lx(X) += -cost_config_ptr_->at(W_FRONT_REF_X) *
           (cost_config_ptr_->at(FRONT_REF_X) - head_x);
  lx(Y) += -cost_config_ptr_->at(W_FRONT_REF_Y) *
           (cost_config_ptr_->at(FRONT_REF_Y) - head_y);
  // lx(THETA) += -cost_config_ptr_->at(W_REF_THETA) *
  //              (cost_config_ptr_->at(REF_THETA) - x[THETA]);

  lxx(X, X) += cost_config_ptr_->at(W_FRONT_REF_X);
  lxx(Y, Y) += cost_config_ptr_->at(W_FRONT_REF_Y);
  // lxx(THETA, THETA) += cost_config_ptr_->at(W_REF_THETA);
}

double VirtualReferencePathCostTerm::GetCost(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/) {
  double cost = 0.;
  const double cost_x = 0.5 * cost_config_ptr_->at(W_VIRTUAL_REF_X) *
                        Square(x[X] - cost_config_ptr_->at(VIRTUAL_REF_X));
  const double cost_y = 0.5 * cost_config_ptr_->at(W_VIRTUAL_REF_Y) *
                        Square(x[Y] - cost_config_ptr_->at(VIRTUAL_REF_Y));
  const double cost_theta =
      0.5 * cost_config_ptr_->at(W_VIRTUAL_REF_THETA) *
      Square(x[THETA] - cost_config_ptr_->at(VIRTUAL_REF_THETA));
  cost = cost_x + cost_y + cost_theta;
  cost_value_ += cost;
  return cost;
}

void VirtualReferencePathCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT & /*lxu*/, ilqr_solver::LuuMT & /*luu*/) {
  lx(X) += -cost_config_ptr_->at(W_VIRTUAL_REF_X) *
           (cost_config_ptr_->at(VIRTUAL_REF_X) - x[X]);
  lx(Y) += -cost_config_ptr_->at(W_VIRTUAL_REF_Y) *
           (cost_config_ptr_->at(VIRTUAL_REF_Y) - x[Y]);
  lx(THETA) += -cost_config_ptr_->at(W_VIRTUAL_REF_THETA) *
               (cost_config_ptr_->at(VIRTUAL_REF_THETA) - x[THETA]);

  lxx(X, X) += cost_config_ptr_->at(W_VIRTUAL_REF_X);
  lxx(Y, Y) += cost_config_ptr_->at(W_VIRTUAL_REF_Y);
  lxx(THETA, THETA) += cost_config_ptr_->at(W_VIRTUAL_REF_THETA);
}

}  // namespace lateral_planning
}  // namespace pnc