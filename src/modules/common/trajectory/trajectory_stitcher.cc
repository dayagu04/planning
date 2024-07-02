#include "trajectory/trajectory_stitcher.h"
#include <algorithm>
#include "math/math_utils.h"
#include "vehicle_model/vehicle_model.h"

namespace planning {

PncTrajectoryPoint TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
    const VehicleState& vehicle_state) {
  PncTrajectoryPoint point;
  point.path_point.s = 0.0;
  point.path_point.x = vehicle_state.x;
  point.path_point.y = vehicle_state.y;
  point.path_point.z = vehicle_state.z;
  point.path_point.theta = vehicle_state.heading;
  point.path_point.kappa = vehicle_state.kappa;
  point.delta = vehicle_state.delta;
  point.v = vehicle_state.linear_velocity;
  point.a = vehicle_state.linear_acceleration;
  point.jerk = vehicle_state.jerk;
  point.relative_time = 0.0;
  return point;
}

PncTrajectoryPoint
TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const double planning_cycle_time, const VehicleState& vehicle_state,
    const bool enable_vehi_state_predict) {
  PncTrajectoryPoint reinit_point;
  // static constexpr double kEpsilon_v = 0.1;
  // static constexpr double kEpsilon_a = 0.4;
  // if (std::abs(vehicle_state.linear_velocity) < kEpsilon_v &&
  //     std::abs(vehicle_state.linear_acceleration) < kEpsilon_a) {
  //   reinit_point = ComputeTrajectoryPointFromVehicleState(vehicle_state);
  // } else {
  VehicleState predicted_vehicle_state;
  if (enable_vehi_state_predict) {
    predicted_vehicle_state =
        common::VehicleModel::Predict(planning_cycle_time, vehicle_state);
    reinit_point =
        ComputeTrajectoryPointFromVehicleState(predicted_vehicle_state);
    reinit_point.relative_time += planning_cycle_time;
  } else {
    reinit_point = ComputeTrajectoryPointFromVehicleState(vehicle_state);
  }

  return reinit_point;
}

}  // namespace planning