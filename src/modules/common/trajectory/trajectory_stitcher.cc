#include "trajectory/trajectory_stitcher.h"
#include <algorithm>
#include "math/math_utils.h"
#include "vehicle_model/vehicle_model.h"

namespace planning {

PncTrajectoryPoint TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(const VehicleState& vehicle_state) {
  PncTrajectoryPoint point;
  point.path_point.s = 0.0;
  point.path_point.x = vehicle_state.x;
  point.path_point.y = vehicle_state.y;
  point.path_point.z = vehicle_state.z;
  point.path_point.theta = vehicle_state.heading;
  point.path_point.kappa = vehicle_state.kappa;
  point.v = vehicle_state.linear_velocity;
  point.a = vehicle_state.linear_acceleration;
  point.relative_time = 0.0;
  return point;
}

std::vector<PncTrajectoryPoint> TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const double planning_cycle_time, const VehicleState& vehicle_state) {
  PncTrajectoryPoint reinit_point;
  static constexpr double kEpsilon_v = 0.1;
  static constexpr double kEpsilon_a = 0.4;
  if (std::abs(vehicle_state.linear_velocity) < kEpsilon_v &&
      std::abs(vehicle_state.linear_acceleration) < kEpsilon_a) {
    reinit_point = ComputeTrajectoryPointFromVehicleState(vehicle_state);
  } else {
    VehicleState predicted_vehicle_state;
    predicted_vehicle_state = common::VehicleModel::Predict(planning_cycle_time, vehicle_state);
    reinit_point = ComputeTrajectoryPointFromVehicleState(predicted_vehicle_state);
    reinit_point.relative_time += planning_cycle_time;
  }

  return std::vector<PncTrajectoryPoint>(1, reinit_point);
}

/* Planning from current vehicle state if:
   1. the auto-driving mode is off
   (or) 2. we don't have the trajectory from last planning cycle
   (or) 3. the position deviation from actual and target is too high
*/
// std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(
//     const VehicleState& vehicle_state, const double current_timestamp,
//     const double planning_cycle_time, const size_t preserved_points_num,
//     const bool replan_by_offset, PlanningResult* prev_planning_result,
//     std::string* replan_reason) {

//   DiscretizedTrajectory prev_trajectory(*prev_planning_result);
//   if (!prev_trajectory.NumOfPoints()) {
//     *replan_reason = "replan for no previous trajectory.";
//     MSD_LOG(INFO, "trajectory stitcher: replan for no previous trajectory.");
//     return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
//   }

//   if (vehicle_state.driving_mode != DrivingMode::AUTO) {
//     *replan_reason = "replan for manual mode.";
//     MSD_LOG(INFO, "trajectory stitcher: replan for manual mode.");
//     return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
//   }

//   if (std::abs(vehicle_state.linear_velocity - prev_planning_result->v_target) >
//       FLAGS_replan_velocity_threshold) {
//     MSD_LOG(INFO, "trajectory stitcher: vehicle-vel control-error is bigger than: %f ",
//         FLAGS_replan_velocity_threshold);
//     *replan_reason = "replan for empty previous trajectory.";
//     return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
//   }

//   size_t prev_trajectory_size = prev_trajectory.size();

//   if (prev_trajectory_size == 0) {
//     MSD_LOG(INFO, "trajectory stitcher: Projected trajectory at time [%f] size is zero! Previous planning not exist
//     or failed. Use "
//               "origin car status instead.", prev_trajectory.header_time());
//     *replan_reason = "replan for empty previous trajectory.";
//     return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
//   }

//   const double veh_rel_time =
//       current_timestamp - prev_trajectory.header_time();

//   size_t time_matched_index =
//       prev_trajectory.QueryLowerBoundPoint(veh_rel_time);

//   if (time_matched_index == 0 &&
//       veh_rel_time + 1.e-2 < prev_trajectory.StartPoint().relative_time - planning_cycle_time) {
//     MSD_LOG(INFO, "trajectory stitcher: current time smaller than the previous trajectory's first time");
//     *replan_reason =
//         "replan for current time smaller than the previous trajectory's first time.";
//     return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
//   }
//   if (time_matched_index + 1 >= prev_trajectory_size) {
//     MSD_LOG(INFO, "trajectory stitcher: current time beyond the previous trajectory's last time");
//     *replan_reason =
//         "replan for current time beyond the previous trajectory's last time";
//     return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
//   }

//   // auto time_matched_point = prev_trajectory.TrajectoryPointAt(
//   //     static_cast<uint32_t>(time_matched_index));
//   auto time_matched_point = prev_trajectory.Evaluate(veh_rel_time);

//   size_t position_matched_index = prev_trajectory.QueryNearestPointWithBuffer(
//       {vehicle_state.x, vehicle_state.y}, 1.0e-6);
//   auto position_matched_point = prev_trajectory.TrajectoryPointAt(
//       static_cast<uint32_t>(position_matched_index));

//   auto frenet_sd = ComputePositionProjection(
//       vehicle_state.x, vehicle_state.y,
//       prev_trajectory.TrajectoryPointAt(
//           static_cast<uint32_t>(position_matched_index)));
//   auto position_project_matched_index = prev_trajectory.QueryNearestPointWithBuffer(frenet_sd.first, 1.0e-6);
//   auto position_project_matched_point = prev_trajectory.TrajectoryPointAt(
//       static_cast<uint32_t>(position_project_matched_index));

//   double forward_rel_time = std::max(veh_rel_time, planning_cycle_time);

//   size_t forward_time_index =
//       prev_trajectory.QueryLowerBoundPoint(forward_rel_time);
//   // auto forward_time_trajectory_point = prev_trajectory.at(forward_time_index);
//   auto forward_time_trajectory_point = prev_trajectory.Evaluate(forward_rel_time);

//   if (replan_by_offset) {
//     auto lon_diff = time_matched_point.path_point.s - frenet_sd.first;
//     auto lat_diff = frenet_sd.second;
//     auto lon_time_diff = time_matched_point.relative_time - position_project_matched_point.relative_time;
//     prev_planning_result->lon_error = lon_diff;
//     prev_planning_result->lat_error = lat_diff;

//     MSD_LOG(INFO, "trajectory stitcher: Control lateral diff: %f, longitudinal diff: %f longitudinal time diff: %f",
//         lat_diff, lon_diff, lon_time_diff);

//     if (std::fabs(lat_diff) > FLAGS_replan_lateral_distance_threshold) {
//       std::string msg(
//           "the distance between matched point and actual position is too "
//           "large. Replan is triggered. lat_diff = " +
//           std::to_string(lat_diff));
//       MSD_LOG(INFO, "trajectory stitcher: %s", msg.c_str());
//       *replan_reason = msg;
//       auto reinit_traj =  ComputeReinitStitchingTrajectory(planning_cycle_time,
//                                               vehicle_state);
//       if (FLAGS_enable_s_update_mechanism) {
//         reinit_traj.at(0).v = forward_time_trajectory_point.v;
//         reinit_traj.at(0).a = forward_time_trajectory_point.a;
//       }
//       return reinit_traj;
//     }

//     if (std::fabs(lon_diff) > FLAGS_replan_longitudinal_distance_threshold ||
//         (std::fabs(lon_time_diff) > FLAGS_replan_longitudinal_time_threshold &&
//         std::fabs(lon_diff) > FLAGS_min_replan_longitudinal_distance_threshold)) {
//       std::string msg(
//           "the distance between matched point and actual position is too "
//           "large. Replan is triggered. lon_diff = " +
//           std::to_string(lon_diff) + " lon_time_diff = " + std::to_string(lon_time_diff));
//       MSD_LOG(INFO, "trajectory stitcher: %s", msg.c_str());
//       *replan_reason = msg;
//       auto reinit_traj =  ComputeReinitStitchingTrajectory(planning_cycle_time,
//                                               vehicle_state);
//       if (FLAGS_enable_s_update_mechanism) {
//         reinit_traj.at(0).v = forward_time_trajectory_point.v;
//         reinit_traj.at(0).a = forward_time_trajectory_point.a;
//       }
//       return reinit_traj;
//     }
//   } else {
//     MSD_LOG(INFO, "trajectory stitcher: replan according to certain amount of lat and lon offset is "
//               "disabled");
//   }

//   MSD_LOG(INFO, "trajectory stitcher: Position matched index: %d", position_matched_index);
//   MSD_LOG(INFO, "trajectory stitcher: Time matched index: %d", time_matched_index);

//   std::vector<TrajectoryPoint> stitching_trajectory = std::vector<TrajectoryPoint>(1, forward_time_trajectory_point);

//   // auto matched_index = std::min(time_matched_index, position_matched_index);

//   // std::vector<TrajectoryPoint> stitching_trajectory(
//   //     prev_trajectory.begin() +
//   //         std::max(0, static_cast<int>(matched_index - preserved_points_num)),
//   //     prev_trajectory.begin() + forward_time_index + 1);
//   // MSD_LOG(INFO, "trajectory stitcher: stitching_trajectory size: %d", stitching_trajectory.size());

//   const double zero_s = stitching_trajectory.back().path_point.s;
//   for (auto& tp : stitching_trajectory) {
//     tp.relative_time = tp.relative_time + prev_trajectory.header_time() -
//                          current_timestamp;
//     tp.path_point.s = tp.path_point.s - zero_s;
//   }
//   return stitching_trajectory;
// }

}  // namespace planning