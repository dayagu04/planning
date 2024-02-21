#include "lateral_path_optimizer_model.h"

#include "geometry_math.h"
#include "ilqr_define.h"
#include "lateral_path_optimizer_cost.h"
#include "math_lib.h"

using namespace pnc::mathlib;

static const double kOneSix = 1.0 / 6.0;
static const double kOneThree = 1.0 / 3.0;

namespace planning {

namespace apa_planner {
ilqr_solver::State LateralPathOptimizerModel::UpdateDynamicsOneStep(
    const ilqr_solver::State &x, const ilqr_solver::Control &u,
    const size_t &step) const {
  // state = [x y theta k]
  // control = [u]
  double model_sgin_gain = 1.0;
  if (cost_config_vec_ptr_) {
    if (cost_config_vec_ptr_->size() > 0) {
      model_sgin_gain = cost_config_vec_ptr_->at(0)[MODEL_SIGN_GAIN];
    }
  }
  const double &ds = solver_config_ptr_->model_dt;
  const double &theta = x[StateId::THETA];
  const double &k = x[StateId::K];
  const double &u_ = u[ControlId::U];

  // ds * k
  const double ds_k = ds * k;
  const double ds_u = ds * u_;
  const double ds2_u = ds_u * ds;

  // theta
  const double theta0 = theta + ds_k * 0.5;
  const double theta1 = theta + ds_k * 0.5 + ds2_u * 0.25;
  const double theta2 = theta + ds_k + ds2_u * 0.5;
  // const double theta3 = pnc::geometry_lib::NormalizeAngle(theta2);
  // x + (ds*(2*cos(theta0) + cos(theta) + cos(theta2) + 2*cos(theta1)))/6;
  // y + (ds*(sin(theta2) + 2*sin(theta1) + 2*sin(theta0) + sin(theta)))/6;
  // cos theta
  const double cos_theta = std::cos(theta);
  const double cos_theta_0 = ds * cos_theta;
  const double cos_theta_1 = ds * std::cos(theta0);
  const double cos_theta_2 = ds * std::cos(theta1);
  const double cos_theta_3 = ds * std::cos(theta2);
  // sin theta
  const double sin_theta = std::sin(theta);
  const double sin_theta_0 = ds * sin_theta;
  const double sin_theta_1 = ds * std::sin(theta0);
  const double sin_theta_2 = ds * std::sin(theta1);
  const double sin_theta_3 = ds * std::sin(theta2);

  ilqr_solver::State x1 = x;
  x1 << x[X] +
            model_sgin_gain * (kOneSix * cos_theta_0 + kOneThree * cos_theta_1 +
                               kOneThree * cos_theta_2 + kOneSix * cos_theta_3),

      x[Y] +
          model_sgin_gain * (kOneSix * sin_theta_0 + kOneThree * sin_theta_1 +
                             kOneThree * sin_theta_2 + kOneSix * sin_theta_3),

      theta2,

      k + ds_u;
  return x1;
}

void LateralPathOptimizerModel::GetDynamicsDerivatives(
    const ilqr_solver::State &x, const ilqr_solver::Control &u,
    ilqr_solver::FxMT &f_x, ilqr_solver::FuMT &f_u, const size_t &step) const {
  double model_sgin_gain = 1.0;
  if (cost_config_vec_ptr_) {
    if (cost_config_vec_ptr_->size() > 0) {
      model_sgin_gain = cost_config_vec_ptr_->at(0)[MODEL_SIGN_GAIN];
    }
  }
  const double &ds = solver_config_ptr_->model_dt;
  const double &theta = x[StateId::THETA];
  const double sin_theta = std::sin(theta);
  const double cos_theta = std::cos(theta);
  const double ds2 = ds * ds;

  f_x << 1.0, 0.0, -model_sgin_gain * ds * sin_theta,
      -model_sgin_gain * (ds2 * sin_theta) * 0.5,

      0.0, 1.0, model_sgin_gain * ds * cos_theta,
      model_sgin_gain * (ds2 * cos_theta) * 0.5,

      0.0, 0.0, 1.0, ds,

      0.0, 0.0, 0.0, 1.0;

  f_u << 0.0, 0.0, 0.0, ds;
}

}  // namespace apa_planner

}  // namespace planning