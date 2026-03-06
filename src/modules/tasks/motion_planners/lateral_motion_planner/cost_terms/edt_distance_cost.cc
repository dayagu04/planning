#include "edt_distance_cost.h"

#include "math_lib.h"
#include "pose2d.h"
#include "transform2d.h"

using namespace pnc::mathlib;
namespace pnc {
namespace lateral_planning {

EdtDistanceCostTerm::EdtDistanceCostTerm(
    planning::Transform2f* ego_base_ptr, planning::EulerDistanceTransform* edt_ptr) {
  ego_base_ptr_ = ego_base_ptr;
  edt_ptr_ = edt_ptr;
}

double EdtDistanceCostTerm::GetCost(const ilqr_solver::State &x,
                                   const ilqr_solver::Control & /*u*/) {
  double cost = 0.0;
  if (edt_ptr_ == nullptr || ego_base_ptr_ == nullptr) {
    cost_value_ += cost;
    return cost;
  }

  // Create PathPoint from state x and y and theta
  // pnc::geometry_lib::PathPoint path_point;
  // path_point.pos.x() = x[X];
  // path_point.pos.y() = x[Y];
  // path_point.heading = x[THETA];

  // Get distance to nearest obstacle from EDT
  float min_dist = 0.0f;
  planning::Pose2f local;
  ego_base_ptr_->GlobalPointToULFLocal(&local, planning::Pose2f(x[X], x[Y], x[THETA]));
  planning::Transform2f tf(local);
  bool is_valid =
      edt_ptr_->DistanceCheckForPoint(&min_dist, &tf, planning::AstarPathGear::REVERSE);
  // bool is_valid = edt_ptr_->DistanceCheckForPoint(
  //     &min_dist, path_point, pnc::geometry_lib::SEG_GEAR_INVALID);

  // If EDT is valid and has positive distance, add cost
  // Cost is quadratic penalty for being too close to obstacles
  // We use inverse of distance to penalize closeness
  if ((!is_valid && min_dist > 0.0f) ||
      (is_valid && min_dist < -1e-3)) {
    const double safe_distance = 0.2; // 0.2m safe buffer
    if (min_dist < safe_distance) {
      const double distance_error = safe_distance - min_dist;
      cost = 0.5 * cost_config_ptr_->at(W_EDT_DISTANCE) * Square(distance_error);
    }
  }

  cost_value_ += cost;
  return cost;
}

void EdtDistanceCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control & /*u*/,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT & /*lu*/, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT & /*lxu*/, ilqr_solver::LuuMT & /*luu*/) {
  if (edt_ptr_ == nullptr || ego_base_ptr_ == nullptr) {
    return;
  }

  // Create PathPoint from state
  // pnc::geometry_lib::PathPoint path_point;
  // path_point.pos.x() = x[X];
  // path_point.pos.y() = x[Y];
  // path_point.heading = x[THETA];

  // Get distance to nearest obstacle
  float min_dist = 0.0f;
  planning::Pose2f local;
  ego_base_ptr_->GlobalPointToULFLocal(&local, planning::Pose2f(x[X], x[Y], x[THETA]));
  planning::Transform2f tf(local);
  bool is_valid =
      edt_ptr_->DistanceCheckForPoint(&min_dist, &tf, planning::AstarPathGear::REVERSE);
  // bool is_valid = edt_ptr_->DistanceCheckForPoint(
  //     &min_dist, path_point, pnc::geometry_lib::SEG_GEAR_INVALID);

  if ((!is_valid && min_dist > 0.0f) ||
      (is_valid && min_dist < -1e-3)) {
    const double safe_distance = 0.2; // 0.2m safe buffer
    if (min_dist < safe_distance) {
      const double distance_error = safe_distance - min_dist;

      // For simplicity, we approximate the gradient
      // In practice, we would need to compute the gradient of distance
      // w.r.t x and y, which requires EDT gradient information
      // This is a simplified version that treats distance as constant
      // for gradient computation (conservative approximation)

      // Add regularization to Hessian to encourage staying away
      lx(X) += cost_config_ptr_->at(W_EDT_DISTANCE) * distance_error;
      lx(Y) += cost_config_ptr_->at(W_EDT_DISTANCE) * distance_error;
      lxx(X, X) += cost_config_ptr_->at(W_EDT_DISTANCE);
      lxx(Y, Y) += cost_config_ptr_->at(W_EDT_DISTANCE);
    }
  }
}

}  // namespace lateral_planning
}  // namespace pnc