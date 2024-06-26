#ifndef MODULES_PLANNING_COMMON_TRAJECTORY_STITCHER_H_
#define MODULES_PLANNING_COMMON_TRAJECTORY_STITCHER_H_

#include <vector>

#include "config/message_type.h"
#include "math/linear_interpolation.h"

namespace planning {

class TrajectoryStitcher {
 public:
  TrajectoryStitcher() = delete;

  // static void TransformLastPublishedTrajectory(
  //     const double x_diff, const double y_diff, const double theta_diff,
  //     DiscretizedTrajectory* prev_trajectory);

  // static DiscretizedTrajectory ConstructTrajFromPlanningResult(
  //     const PlanningResult &planning_result);

  // static std::vector<TrajectoryPoint> ComputeStitchingTrajectory(
  //     const VehicleState& vehicle_state, const double current_timestamp,
  //     const double planning_cycle_time, const size_t preserved_points_num,
  //     const bool replan_by_offset, PlanningResult* prev_planning_result,
  //     std::string* replan_reason);

  static std::vector<PncTrajectoryPoint> ComputeReinitStitchingTrajectory(
      const double planning_cycle_time, const VehicleState& vehicle_state,
      const bool enable_vehi_state_predict = false);

  //   static PublishedTrajectory TransformToPublishedTraj(
  //       std::vector<TrajectoryPoint> trajectory);

  //  private:
  //   static std::pair<double, double> ComputePositionProjection(
  //       const double x, const double y,
  //       const TrajectoryPoint& matched_trajectory_point);

  static PncTrajectoryPoint ComputeTrajectoryPointFromVehicleState(
      const VehicleState& vehicle_state);
};

}  // namespace planning

#endif /* MODULES_PLANNING_COMMON_TRAJECTORY_STITCHER_H_ */
