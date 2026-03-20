#pragma once

#include "../common.h"
#include "actuator.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace planning {
struct PlanningMeta {
  uint64_t timestamp_us;
  uint64_t plan_timestamp_us;
  std::string plan_strategy_name;
};

enum MSDTrajectoryAvailable {
  MSD_TRAJECTORY_POLYNOMIAL_CURVE = 1 << 0,
  MSD_TRAJECTORY_PATH_POINTS = 1 << 1,
};

struct MSDCurve {
  std::vector<double> polynomial;
};

// struct MSDPoint2d {
//  double x;
//  double y;
//};

struct MSDTrajectoryPoint {
  MSDPoint2d position_enu;
  double heading_yaw;
  double curvature;
};

struct MSDTrajectory {
  uint8_t available;
  MSDCurve polynomial_curve;
  std::vector<MSDTrajectoryPoint> path_points;
};

enum MSDVelocityAvailable {
  MSD_VELOCITY_TARGET_VALUE = 1 << 0,
  MSD_VELOCITY_PATH_POINTS = 1 << 1,
};

struct MSDVelocityPoint {
  double target_velocity;  // target velocity in current point
  double relative_time;    // relative time from start point of plan to current
                           // point.
  double distance;  // trajectory distance from start point of plan to current
                    // point.
};

struct MSDVelocity {
  uint8_t available;
  double target_value;
  std::vector<MSDVelocityPoint> path_points;
};

enum MSDAccelerationAvailable {
  MSD_ACCELERATION_RANGE_LIMIT = 1 << 0,
  MSD_ACCELERATION_PATH_POINTS = 1 << 1,
};

struct MSDAccelerationRange {
  double max;
  double min;
};

struct MSDAcceleration {
  uint8_t available;
  MSDAccelerationRange range_limit;
  std::vector<double> path_points;
};

enum MSDGearStateAvailable {
  MSD_GEAR_STATE_CONTROL_FORMAT = 1 << 0,
};

enum MSDGear {
  MSD_GEAR_NONE = 0,
  MSD_GEAR_PARK = 1,
  MSD_GEAR_REVERSE = 2,
  MSD_GEAR_NEUTRAL = 3,
  MSD_GEAR_DRIVE = 4,
  MSD_GEAR_LOW = 5,
};

struct MSDGearState {
  uint8_t available;
  MSDGear control_format;
};

enum MSDEmergencyAvailable {
  MSD_EMERGENCY_JERK_FACTOR = 1 << 0,
  MSD_EMERGENCY_STATIONARY_OBSTACLE_CAR_INFO = 1 << 1,
};

struct MSDStationaryObstacleCarInfo {
  bool close_to_obstacle;
  double steering_wheel_rad_limit;  // rad
};

struct MSDEmergency {
  uint8_t available;
  double jerk_factor;
  MSDStationaryObstacleCarInfo stationary_obstacle_car_info;
};

enum MSDCompensationAvailable {
  MSD_COMPENSATION_STEERING_ANGLE_COMPENSATION_VALUE = 1 << 0,
  MSD_COMPENSATION_TRAJECTORY_LATITUDE_COMPENSATION_VALUE = 1 << 1,
};

struct MSDCompensation {
  uint8_t available;
  double steering_angle_compensation_value;
  double trajectory_latitude_compensation_value;
};

enum MSDPlanStatusAvailable {
  MSD_PLAN_STATUS_ALGORITHM_STATUS = 1 << 0,
};

enum MSDScene {
  MSD_SCENE_NORMAL_ROAD = 1,  // driving in normal road
  MSD_SCENE_INTERSECT = 2,    // driving in intersect
  MSD_SCENE_PARKING = 4,      // car is parking
};

enum MSDAction {
  MSD_ACTION_LANE_CHANGE_LEFT = 1,        // lane change left
  MSD_ACTION_LANE_CHANGE_RIGHT = 2,       // lane change right
  MSD_ACTION_LANE_BORROW_LEFT = 4,        // lane borrow left
  MSD_ACTION_LANE_BORROW_RIGHT = 8,       // lane borrow right
  MSD_ACTION_INTERSECT_GO_STRAIGHT = 16,  // go straight in intersect scene
  MSD_ACTION_INTERSECT_TURN_LEFT = 32,    // turn left in intersect scene
  MSD_ACTION_INTERSECT_TURN_RIGHT = 64,   // turn right in intersect scene
  MSD_ACTION_INTERSECT_U_TURN = 128,      // u-turn in intersect scene
  MSD_ACTION_LANE_BORROW_IN_NON_MOTORIZED_LANE =
      256,  // lane borrow in non-motorized-lane
};

enum MSDActionStatus {
  MSD_ACTION_STATUS_LANE_CHANGE_WAITING = 1,  //  waiting the lane changing
  MSD_ACTION_STATUS_LANE_CHANGEING = 2,       //  changing lane
  MSD_ACTION_STATUS_LANE_CHANGE_BACK = 4,     //  lane changing back
  MSD_ACTION_STATUS_LANE_BORROWING = 8,       //  borrowing lane
  MSD_ACTION_STATUS_LANE_BORROW_BACK =
      16,  //  lane borrow back to the back of target car
  MSD_ACTION_STATUS_LANE_BORROW_RETURN =
      32,  //  lane borrow back to the front of target car
  MSD_ACTION_STATUS_LANE_BORROW_SUSPEND = 64,  //  lane borrow suspend
};

struct MSDPlanAlgorithmStatus {
  MSDScene scene;
  MSDAction action;
  MSDActionStatus action_status;
};

struct MSDPlanStatus {
  uint8_t available;
  MSDPlanAlgorithmStatus algorithm_status;
};

enum MSDComfortAvailable {
  MSD_COMFORT_MANEUVER_GEAR = 1 << 0,
};

enum MSDManeuverGear {
  MSD_MANEUVER_GEAR_SLOW = 0,
  MSD_MANEUVER_GEAR_NORMAL = 1,
  MSD_MANEUVER_GEAR_FAST = 2,
  MSD_MANEUVER_GEAR_ACCIDENT = 3,
};

struct MSDComfort {
  uint8_t available;
  MSDManeuverGear maneuver_gear;
};

enum MSDPlanExtraAvailable {
  MSD_PLAN_EXTRA_VERSION = 1 << 0,
  MSD_PLAN_EXTRA_JSON = 1 << 1,
};

struct MSDPlanExtra {
  uint8_t available;
  std::string version;
  std::string json;
};

// struct MSDPlan {
struct PlanningOutput {
  PlanningMeta meta;
  MSDTrajectory trajectory;      // target trajectory.
  MSDVelocity velocity;          // target velocity.
  MSDAcceleration acceleration;  // target acceleration.
  MSDTurnState turn_state;       // turn light state.
  MSDGearState gear_state;       // gear state.
  MSDEmergency emergency;        // emergency degree.
  MSDCompensation compensation;  // steering offset.
  MSDPlanStatus plan_status;     // plan status.
  MSDComfort comfort;            // comfort.
  MSDPlanExtra extra;            // extra information.
};

}  // namespace planning
