#include "joint_motion_planning_model.h"

#include "joint_motion_planning_cost.h"

using namespace pnc::mathlib;
static constexpr double kOneSix = 1.0 / 6.0;
static constexpr double kOneTwentyFour = 1.0 / 24.0;
static constexpr double kHalf = 1.0 / 2.0;
static constexpr double kOneEighth = 1.0 / 8.0;
namespace pnc {
namespace joint_motion_planning {
ilqr_solver::State JointMotionPlanningModel::UpdateDynamicsOneStep(
    const ilqr_solver::State &x0, const ilqr_solver::Control &u,
    const size_t &step) const {
  const double dt = solver_config_ptr_->model_dt;
  const ilqr_solver::IlqrCostConfig &cost_config =
      cost_config_vec_ptr_->at(step);
  ilqr_solver::State x1 = x0;
  x1.setZero();
  const auto dt2 = dt * dt;
  const auto dt3 = dt2 * dt;
  const auto &x0_x = x0[EGO_X];
  const auto &x0_y = x0[EGO_Y];
  const auto &x0_theta = x0[EGO_THETA];
  const auto &x0_delta = x0[EGO_DELTA];
  const auto &x0_vel = x0[EGO_VEL];
  const auto &x0_acc = x0[EGO_ACC];
  const double curv_factor = cost_config[CURV_FACTOR];
  const auto &u_omega = u[EGO_OMEGA];
  const auto &u_jerk = u[EGO_JERK];
  const double theta_mid = x0_theta +
                           kHalf * curv_factor * x0_delta * x0_vel * dt +
                           kOneEighth * curv_factor * u_omega * x0_vel * dt2;
  const double cos_theta = std::cos(theta_mid);
  const double sin_theta = std::sin(theta_mid);
  const double d_s = std::max(
      0.0, x0_vel * dt + kHalf * x0_acc * dt2 + kOneSix * u_jerk * dt3);
  x1[EGO_X] = x0_x + d_s * cos_theta;
  x1[EGO_Y] = x0_y + d_s * sin_theta;
  x1[EGO_THETA] = x0_theta + curv_factor * x0_delta * x0_vel * dt +
                  kHalf * curv_factor * u_omega * x0_vel * dt2;
  x1[EGO_DELTA] = x0_delta + u_omega * dt;
  x1[EGO_VEL] = std::max(0.0, x0_vel + x0_acc * dt + kHalf * u_jerk * dt2);
  x1[EGO_ACC] = x0_acc + u_jerk * dt;
  // Obstacles use reference trajectories, no need to propagate dynamics.
  return x1;
}

void JointMotionPlanningModel::GetDynamicsDerivatives(
    const ilqr_solver::State &x0, const ilqr_solver::Control &u,
    ilqr_solver::FxMT &f_x, ilqr_solver::FuMT &f_u, const size_t &step) const {
  const double dt = solver_config_ptr_->model_dt;
  const ilqr_solver::IlqrCostConfig &cost_config =
      cost_config_vec_ptr_->at(step);
  // Only optimize ego vehicle trajectory.
  const int total_state_size = EGO_STATE_SIZE;
  const int total_control_size = EGO_CONTROL_SIZE;
  f_x.setZero(total_state_size, total_state_size);
  f_u.setZero(total_state_size, total_control_size);
  const auto dt2 = dt * dt;
  const auto dt3 = dt2 * dt;
  const auto &x0_x = x0[EGO_X];
  const auto &x0_y = x0[EGO_Y];
  const auto &x0_theta = x0[EGO_THETA];
  const auto &x0_delta = x0[EGO_DELTA];
  const auto &x0_vel = x0[EGO_VEL];
  const auto &x0_acc = x0[EGO_ACC];
  const double k = cost_config[CURV_FACTOR];
  const auto &u_omega = u[EGO_OMEGA];
  const auto &u_jerk = u[EGO_JERK];
  const double theta_mid = x0_theta + kHalf * k * x0_delta * x0_vel * dt +
                           kOneEighth * k * u_omega * x0_vel * dt2;
  const double cos_theta = std::cos(theta_mid);
  const double sin_theta = std::sin(theta_mid);
  const double d_s = std::max(
      0.0, x0_vel * dt + kHalf * x0_acc * dt2 + kOneSix * u_jerk * dt3);
  const double k_delta_t = kHalf * k * x0_delta * dt;
  const double k_omega_t = kOneEighth * k * x0_vel * dt2;
  // df/dx: 状态雅可比矩阵 (6x6)
  f_x.block(0, 0, EGO_STATE_SIZE, EGO_STATE_SIZE) <<
      // dx/d[x,y,θ,δ,v,a]
      1.0,
      0.0, -d_s * sin_theta, -d_s * sin_theta * kHalf * k * x0_vel * dt,
      dt * cos_theta - dt * x0_vel * k_delta_t * sin_theta,
      kHalf * dt2 * cos_theta,
      // dy/d[x,y,θ,δ,v,a]
      0.0, 1.0, d_s * cos_theta, d_s * cos_theta * kHalf * k * x0_vel * dt,
      dt * sin_theta + dt * x0_vel * k_delta_t * cos_theta,
      kHalf * dt2 * sin_theta,
      // dθ/d[x,y,θ,δ,v,a]
      0.0, 0.0, 1.0, k * x0_vel * dt, k * x0_delta * dt, 0.0,
      // dδ/d[x,y,θ,δ,v,a]
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      // dv/d[x,y,θ,δ,v,a]
      0.0, 0.0, 0.0, 0.0, 1.0, dt,
      // da/d[x,y,θ,δ,v,a]
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  // df/du: 输入雅可比矩阵 (6x2)
  f_u.block(0, 0, EGO_STATE_SIZE, EGO_CONTROL_SIZE)
      << -d_s * sin_theta * k_omega_t,
      kOneSix * dt3 * cos_theta, d_s * cos_theta * k_omega_t,
      kOneSix * dt3 * sin_theta, kHalf * k * x0_vel * dt2, 0.0, dt, 0.0, 0.0,
      kHalf * dt2, 0.0, dt;
  // Obstacles use reference trajectories, no need to compute dynamics jacobian.
  return;
}
}  // namespace joint_motion_planning
}  // namespace pnc