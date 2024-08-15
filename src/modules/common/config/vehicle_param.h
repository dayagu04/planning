#ifndef MODULES_PLANNING_OPTIMIZERS_VEHICLE_PARAM_H_
#define MODULES_PLANNING_OPTIMIZERS_VEHICLE_PARAM_H_

#include "debug_info_log.h"
#include "utils/file.h"

namespace planning {

struct VehicleParam {
  VehicleParam() = default;
  VehicleParam(std::string config_path) { InitParameters(config_path); }

  double front_edge_to_rear_axle = 3.73;
  double rear_edge_to_rear_axle = 1.085;
  double rear_axle_to_center = 1.295;
  double length = 4.786;                   // 车长
  double width = 1.935;                    // 车宽
  double max_width = 2.229;                // 车宽 （带后视镜
  double height = 1.685;                   // 车高度
  double width_mirror = 0.15;              // 车宽 （带后视镜
  double min_turn_radius = 4.88;           // 最小转弯半径
  double max_acceleration = 2.5;           // 最大加速度
  double max_deceleration = -6.0;          // 最大减速度
  double max_steer_angle = 8.20304748437;  // 方向盘最大转角（rad
  double max_steer_angle_rate = 8.55211;  // 方向盘最大转角速度 （rad/s
  double min_steer_angle_rate = 0;
  double steer_ratio = 16.5;  // 传动比
  double wheel_base = 2.92;   // 轴距
  double max_jerk = 8.0;
  double min_jerk = -5.0;
  double max_front_wheel_angle = 0.56;
  std::string car_type = "";

  void InitParameters(std::string config_path) {
    std::string path = config_path;

    std::string config_file = planning::common::util::ReadFile(path);
    auto config = mjson::Reader(config_file);

    JSON_READ_VALUE(front_edge_to_rear_axle, double, "front_edge_to_rear_axle");
    JSON_READ_VALUE(rear_edge_to_rear_axle, double, "rear_edge_to_rear_axle");
    JSON_READ_VALUE(length, double, "length");
    JSON_READ_VALUE(width, double, "width");
    JSON_READ_VALUE(max_width, double, "max_width");
    JSON_READ_VALUE(height, double, "height");
    JSON_READ_VALUE(width_mirror, double, "width_mirror");
    JSON_READ_VALUE(min_turn_radius, double, "min_turn_radius");
    JSON_READ_VALUE(max_acceleration, double, "max_acceleration");
    JSON_READ_VALUE(max_deceleration, double, "max_deceleration");
    JSON_READ_VALUE(max_jerk, double, "max_jerk");
    JSON_READ_VALUE(min_jerk, double, "min_jerk");
    JSON_READ_VALUE(max_steer_angle, double, "max_steer_angle");
    JSON_READ_VALUE(max_steer_angle_rate, double, "max_steer_angle_rate");
    JSON_READ_VALUE(min_steer_angle_rate, double, "min_steer_angle_rate");
    JSON_READ_VALUE(steer_ratio, double, "steer_ratio");
    JSON_READ_VALUE(wheel_base, double, "wheel_base");
    JSON_READ_VALUE(max_front_wheel_angle, double, "max_front_wheel_angle");
    JSON_READ_VALUE(rear_axle_to_center, double, "rear_axle_to_center");
    JSON_READ_VALUE(car_type, std::string, "car_type");
  }
};

}  // namespace planning

#endif /* MODULES_PLANNING_OPTIMIZERS_VEHICLE_PARAM_H_ */