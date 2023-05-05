#include "ilqr_lat_cost.h"

double yRefCostTerm::GetCost(const State &x, const Control & /*u*/) {
  double cost = cost_config_ptr_->at(W_REF_Y) *
                std::pow(cost_config_ptr_->at(REF_Y) - x[StateId::Y], 2) / 2;
  return cost;
}
void yRefCostTerm::GetGradientHessian(const State &x, const Control & /*u*/,
                                      LxMT &lx, LuMT & /*lu*/, LxxMT &lxx,
                                      LxuMT & /*lxu*/, LuuMT & /*luu*/) {
  lx(0) += -cost_config_ptr_->at(W_REF_Y) *
           (cost_config_ptr_->at(REF_Y) - x[StateId::Y]);
  lxx(0, 0) += cost_config_ptr_->at(W_REF_Y);
}

double thetaRefCostTerm::GetCost(const State &x, const Control & /*u*/) {
  double cost =
      cost_config_ptr_->at(W_REF_THETA) *
      std::pow(cost_config_ptr_->at(REF_THETA) - x[StateId::THETA], 2) / 2;
  return cost;
}
void thetaRefCostTerm::GetGradientHessian(const State &x, const Control & /*u*/,
                                          LxMT &lx, LuMT & /*lu*/, LxxMT &lxx,
                                          LxuMT & /*lxu*/, LuuMT & /*luu*/) {
  lx(1) += -cost_config_ptr_->at(W_REF_THETA) *
           (cost_config_ptr_->at(REF_THETA) - x[StateId::THETA]);

  lxx(1, 1) += cost_config_ptr_->at(W_REF_THETA);
}

double steeringBoundCostTerm::GetCost(const State &x, const Control & /*u*/) {
  double cost = 0.0;
  if (x[StateId::DELTA] < cost_config_ptr_->at(WHEEL_ANGLE_MIN)) {
    cost = cost_config_ptr_->at(W_WHEEL_ANGLE_BOUND) *
           pow(cost_config_ptr_->at(WHEEL_ANGLE_MIN) - x[StateId::DELTA], 2) /
           2;
  } else if (x[StateId::DELTA] > cost_config_ptr_->at(WHEEL_ANGLE_MAX)) {
    cost = cost_config_ptr_->at(WHEEL_ANGLE_MAX) *
           pow(cost_config_ptr_->at(WHEEL_ANGLE_MAX) - x[StateId::DELTA], 2) /
           2;
  }
  return cost;
}
void steeringBoundCostTerm::GetGradientHessian(const State &x,
                                               const Control & /*u*/, LxMT &lx,
                                               LuMT & /*lu*/, LxxMT &lxx,
                                               LxuMT & /*lxu*/,
                                               LuuMT & /*luu*/) {
  if (x[StateId::DELTA] < cost_config_ptr_->at(WHEEL_ANGLE_MIN)) {
    lx(2) += cost_config_ptr_->at(W_WHEEL_ANGLE_BOUND) *
             (x[StateId::DELTA] - cost_config_ptr_->at(WHEEL_ANGLE_MIN));
    lxx(2, 2) += cost_config_ptr_->at(W_WHEEL_ANGLE_BOUND);
  } else if (x[StateId::DELTA] > cost_config_ptr_->at(WHEEL_ANGLE_MAX)) {
    lx(2) += cost_config_ptr_->at(W_WHEEL_ANGLE_BOUND) *
             (x[StateId::DELTA] - cost_config_ptr_->at(WHEEL_ANGLE_MAX));
    lxx(2, 2) += cost_config_ptr_->at(W_WHEEL_ANGLE_BOUND);
  }
}

double steeringRateBoundCostTerm::GetCost(const State & /*x*/,
                                          const Control &u) {
  double cost = 0.0;
  if (cost_config_ptr_->at(TERMINAL_FLAG) > 0.) {
    return 0.0;
  }
  if (u[ControlId::DELTA_DOT] < cost_config_ptr_->at(WHEEL_RATE_MIN)) {
    cost =
        cost_config_ptr_->at(W_WHEEL_ANGLE_RATE_BOUND) *
        pow(cost_config_ptr_->at(WHEEL_RATE_MIN) - u[ControlId::DELTA_DOT], 2) /
        2;
  } else if (u[ControlId::DELTA_DOT] > cost_config_ptr_->at(WHEEL_RATE_MAX)) {
    cost =
        cost_config_ptr_->at(W_WHEEL_ANGLE_RATE_BOUND) *
        pow(cost_config_ptr_->at(WHEEL_RATE_MAX) - u[ControlId::DELTA_DOT], 2) /
        2;
  }
  return cost;
}
void steeringRateBoundCostTerm::GetGradientHessian(
    const State & /*x*/, const Control &u, LxMT & /*lx*/, LuMT &lu,
    LxxMT & /*lxx*/, LxuMT & /*lxu*/, LuuMT &luu) {
  if (cost_config_ptr_->at(TERMINAL_FLAG) > 0.) {
    return;
  }
  if (u[ControlId::DELTA_DOT] < cost_config_ptr_->at(WHEEL_RATE_MIN)) {
    lu(0) += cost_config_ptr_->at(W_WHEEL_ANGLE_RATE_BOUND) *
             (u[ControlId::DELTA_DOT] - cost_config_ptr_->at(WHEEL_RATE_MIN));
    luu(0, 0) += cost_config_ptr_->at(W_WHEEL_ANGLE_RATE_BOUND);
  } else if (u[ControlId::DELTA_DOT] > cost_config_ptr_->at(WHEEL_RATE_MAX)) {
    lu(0) += cost_config_ptr_->at(W_WHEEL_ANGLE_RATE_BOUND) *
             (u[ControlId::DELTA_DOT] - cost_config_ptr_->at(WHEEL_RATE_MAX));
    luu(0, 0) += cost_config_ptr_->at(W_WHEEL_ANGLE_RATE_BOUND);
  }
}

double steeringRateCostTerm::GetCost(const State & /*x*/, const Control &u) {
  if (cost_config_ptr_->at(TERMINAL_FLAG) > 0.) {
    return 0.0;
  }
  double cost = u[ControlId::DELTA_DOT] * u[ControlId::DELTA_DOT] *
                cost_config_ptr_->at(W_LAT_INPUT) / 2.0;
  return cost;
}
void steeringRateCostTerm::GetGradientHessian(const State & /*x*/,
                                              const Control &u, LxMT & /*lx*/,
                                              LuMT &lu, LxxMT & /*lxx*/,
                                              LxuMT & /*lxu*/, LuuMT &luu) {
  if (cost_config_ptr_->at(TERMINAL_FLAG) > 0.) {
    return;
  }
  lu(0) += u[ControlId::DELTA_DOT] * cost_config_ptr_->at(W_LAT_INPUT);
  luu(0, 0) += cost_config_ptr_->at(W_LAT_INPUT);
}