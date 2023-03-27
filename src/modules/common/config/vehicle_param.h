#ifndef MODULES_PLANNING_OPTIMIZERS_VEHICLE_PARAM_H_
#define MODULES_PLANNING_OPTIMIZERS_VEHICLE_PARAM_H_

namespace planning {

struct VehicleParam {
    double front_edge_to_center = 3.846;
    double back_edge_to_center = 0.94;
    double left_edge_to_center = 0.9675;
    double right_edge_to_center = 0.9675;

    double length = 4.786;
    double width = 1.935;
    double height = 1.685;
    double min_turn_radius = 4.88;
    double max_acceleration = 2.5;
    double max_deceleration = -6.0;
    double max_steer_angle = 8.20304748437;
    double max_steer_angle_rate = 8.55211;
    double min_steer_angle_rate = 0;
    double steer_ratio = 16.5; // AION
    double wheel_base = 2.92;
    double wheel_rolling_radius = 0.3765;
    double max_abs_speed_when_stopped = 0.2;

    double brake_deadzone = 15.5;
    double throttle_deadzone = 18.0;

    double max_front_wheel_angle = 0.56;
};

} // namespace planning

#endif /* MODULES_PLANNING_OPTIMIZERS_VEHICLE_PARAM_H_ */