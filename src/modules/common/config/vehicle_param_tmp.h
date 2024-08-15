#ifndef MODULES_PLANNING_OPTIMIZERS_VEHICLE_PARAM_H_
#define MODULES_PLANNING_OPTIMIZERS_VEHICLE_PARAM_H_

namespace planning {

struct vehicle_param {
  // static constexpr double front_edge_to_rear_axle = 3.846;
  // static constexpr double rear_edge_to_rear_axle = 0.94;
  // static constexpr double left_edge_to_center = 0.9675;
  // static constexpr double right_edge_to_center = 0.9675;

  static constexpr double front_edge_to_rear_axle = 3.846;
  static constexpr double rear_edge_to_rear_axle = 0.94;
  static constexpr double length = 4.786;
  static constexpr double width = 1.935;
  static constexpr double max_width = 1.229;
  static constexpr double height = 1.685;
  static constexpr double width_mirror = 0.15;
  static constexpr double max_jerk = 8.0;
  static constexpr double min_jerk = -5.0;

  static constexpr double min_turn_radius = 4.88;
  static constexpr double max_acceleration = 2.5;
  static constexpr double max_deceleration = 6.0;
  static constexpr double max_steer_angle = 8.20304748437;
  static constexpr double max_steer_angle_rate = 8.55211;
  static constexpr double min_steer_angle_rate = 0;
  static constexpr double steer_ratio = 16.5;  // AION
  static constexpr double wheel_base = 2.92;
  static constexpr double max_front_wheel_angle = 0.56;
};

}  // namespace planning

#endif /* MODULES_PLANNING_OPTIMIZERS_VEHICLE_PARAM_H_ */