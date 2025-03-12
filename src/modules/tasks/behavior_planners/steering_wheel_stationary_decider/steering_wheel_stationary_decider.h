#pragma once

#include <cstddef>
#include "ego_planning_config.h"
#include "planning_context.h"
#include "tasks/task.h"

namespace planning {

class SteeringWheelStationaryDecider : public Task {
 public:
  explicit SteeringWheelStationaryDecider(
      const EgoPlanningConfigBuilder* config_builder,
      framework::Session* session);

  virtual ~SteeringWheelStationaryDecider() = default;

  bool Execute() override;

 private:
  bool Init();

  void CalculateTurningRadius();

  bool CalculateSteeringWheelAngleForHPP();

  bool CalculateSteeringWheelAngle();

  double CalculateMinTurningRadius(
      const std::shared_ptr<FrenetObstacle>& target_obstacle);

  void GetTurningDirectionByLane(
      const std::shared_ptr<planning::FrenetObstacle> &obstacle);

  void GetTurningDirectionByRef();

  void GetTurningDirectionByPassInterval();

  bool CheckLaneLineTypeByTurningDirection();

  bool CheckTurningSafety(
      const std::shared_ptr<FrenetObstacle>& target_obstacle);

  std::shared_ptr<planning::FrenetObstacle> GetFrontNearestObstacle();

  std::shared_ptr<planning::FrenetObstacle> GetFrontNearestObstacleByDecision();

  int GetFrontNearestObstacleInLaneChangeState();

  int GetFrontNearestObstacleInLaneBorrowState();

  bool UpdateOutput();

  void Reset();

 private:
  SteeringWheelStationaryDeciderConfig config_;
  bool is_need_steering_wheel_stationary_ = false;
  double turning_direction_ = 0;
  double target_steering_angle_ = 0.0;  // rad
  size_t target_obstacle_id_ = 0;

  double ego_length_;
  double ego_width_;  // with mirror
  double front_edge_to_rear_axle_;
  double rear_edge_to_rear_axle_;
  double rear_axle_to_center_;
  double wheel_base_;
  double steer_ratio_;
  double max_steer_angle_;  // rad
  double max_front_wheel_angle_;
  double min_turn_radius_;
  double safe_buffer_;
  double steer_angle_step_;  // deg
  double min_steer_angle_;  // deg
  double steer_angle_thr_;  // deg

  double min_inner_radius_;
  double max_inner_radius_;
//   double min_outer_radius_;
//   double max_outer_radius_;
//   std::unordered_map<double, double> turning_radius_map_;
//   pnc::mathlib::spline turning_radius_spline_;
};

}  // namespace planning