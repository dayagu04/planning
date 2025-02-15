#ifndef MESSAGE_TYPE_H_
#define MESSAGE_TYPE_H_

#include <string>

#include "common.pb.h"
#include "utils/path_point.h"

#define VEHICLE_LENGTH 4.98

namespace planning {

struct SpeedPoint {
  double s;
  double t;
  // speed (m/s)
  double v;
  // acceleration (m/s^2)
  double a;
  // jerk (m/s^3)
  double da;
};

struct ObstacleType {
  int id;
  double vel;
  double r_frenet;
  double s_frenet;
  double s_max;
  double s_min;
  double r_abs_min;
  double yaw_relative_frenet;
  std::string intention;
  bool isFreeMoveTraj;
  bool isLowPriority;
  double probability;
  bool isAtTargetLane;
  double cutin_score;
  double vel_rotation;
  double v_lat;
  double v_lon;
};

struct PncTrajectoryPoint {
  // path point
  planning_math::PathPoint path_point;

  // linear velocity
  double v;  // in [m/s]
  // linear acceleration
  double a;
  double s;
  double jerk;
  double delta;
  // relative time from beginning of the trajectory
  double relative_time;
  // probability only for prediction trajectory point
  double prediction_prob;
  // speed direction
  double velocity_direction;
  double sigma_x;
  double sigma_y;
  double relative_ego_x;
  double relative_ego_y;
  double relative_ego_yaw;
  double relative_ego_speed;
  double relative_ego_std_dev_x{0.0};
  double relative_ego_std_dev_y{0.0};
  double relative_ego_std_dev_yaw{0.0};
  double relative_ego_std_dev_speed{0.0};
};

enum class DrivingMode {
  AUTO = 0,
  MANUAL,
};

struct SLPoint {
  SLPoint(double S, double L) : s(S), l(L) {}
  SLPoint() : s(0.0), l(0.0) {}
  double s;
  double l;
};

struct VehicleState {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double timestamp{0.0};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
  double heading{0.0};
  double kappa{0.0};
  double linear_velocity{0.0};
  double angular_velocity{0.0};
  double linear_acceleration{0.0};
  double delta{0.0};
  double jerk{0.0};
  DrivingMode driving_mode{DrivingMode::AUTO};
  int gear_position{0};
  double steering_percentage{0.0};
};

enum class ObjectType {
  NOT_KNOW = 0,
  PEDESTRIAN = 1,
  OFO = 2,
  COUPE = 3,
  TRANSPORT_TRUNK = 4,
  BUS = 5,
  ENGINEER_TRUCK = 6,
  TRICYCLE = 7,
  CONE_BUCKET = 8,
  STOP_LINE = 9,
};

}  // namespace planning

#endif
