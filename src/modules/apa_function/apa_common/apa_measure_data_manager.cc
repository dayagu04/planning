#include "apa_measure_data_manager.h"

#include "apa_param_config.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "log_glog.h"
#include "vehicle_service_c.h"

namespace planning {
namespace apa_planner {

void ApaMeasureDataManager::Update(const LocalView* local_view_ptr) {
  if (local_view_ptr == nullptr) {
    ILOG_ERROR << "Update ApaMeasureDataManager, local_view_ptr is nullptr";
    return;
  }

  ILOG_INFO << "Update ApaMeasureDataManager";

  const iflyauto::IFLYLocalization& localization_info =
      local_view_ptr->localization;

  const iflyauto::VehicleServiceOutputInfo& vehicle_service_output_info =
      local_view_ptr->vehicle_service_output_info;

  const auto& param = apa_param.GetParam();

  // rad
  steer_wheel_angle_ = vehicle_service_output_info.steering_wheel_angle;

  front_wheel_angle_ = steer_wheel_angle_ / param.steer_ratio;

  if (vehicle_service_output_info.brake_pedal_pressed_available &&
      vehicle_service_output_info.brake_pedal_pressed) {
    brake_flag_ = true;
  } else {
    brake_flag_ = false;
  }

  const auto& pose = localization_info.position.position_boot;

  const Eigen::Vector2d current_pos(pose.x, pose.y);

  // calculate standstill time by pos
  const double local_move_dist = (pos_ - current_pos).norm();

  if (local_move_dist < param.car_static_pos_err_strict) {
    car_static_timer_by_pos_strict_ += param.plan_time;
  } else {
    car_static_timer_by_pos_strict_ = 0.0;
  }
  if (local_move_dist < param.car_static_pos_err_normal) {
    car_static_timer_by_pos_normal_ += param.plan_time;
  } else {
    car_static_timer_by_pos_normal_ = 0.0;
  }

  pos_ = current_pos;
  heading_ = localization_info.orientation.euler_boot.yaw;
  heading_vec_ << std::cos(heading_), std::sin(heading_);

  const Eigen::Vector2d heading_vec_turn_right(heading_vec_.y(),
                                               -heading_vec_.x());

  const Eigen::Vector2d heading_vec_turn_left(-heading_vec_.y(),
                                              heading_vec_.x());

  right_mirror_pos_ = pos_ + param.lon_dist_mirror_to_rear_axle * heading_vec_ +
                      param.lat_dist_mirror_to_center * heading_vec_turn_right;

  left_mirror_pos_ = pos_ + param.lon_dist_mirror_to_rear_axle * heading_vec_ +
                     param.lat_dist_mirror_to_center * heading_vec_turn_left;

  vel_ = localization_info.velocity.velocity_body.vx;

  // calculate standstill time by velocity
  if (std::fabs(vel_) < param.car_static_velocity_strict) {
    car_static_timer_by_vel_strict_ += param.plan_time;
  } else {
    car_static_timer_by_vel_strict_ = 0.0;
  }
  if (std::fabs(vel_) < param.car_static_velocity_normal) {
    car_static_timer_by_vel_normal_ += param.plan_time;
  } else {
    car_static_timer_by_vel_normal_ = 0.0;
  }

  // static flag
  static_flag_ = (car_static_timer_by_pos_strict_ >
                      param.car_static_keep_time_by_pos_strict ||
                  car_static_timer_by_pos_normal_ >
                      param.car_static_keep_time_by_pos_normal) &&
                 (car_static_timer_by_vel_strict_ >
                      param.car_static_keep_time_by_vel_strict ||
                  car_static_timer_by_vel_normal_ >
                      param.car_static_keep_time_by_vel_normal);

  acceleration_ = localization_info.acceleration.acceleration_body.ax;

  // todo: need to get signal from vehicle service
  fold_mirror_flag_ = apa_param.GetParam().force_fold_mirror;

  ILOG_INFO << "local_move_dist = " << local_move_dist << " m";
  ILOG_INFO << "pos = " << pos_.transpose()
            << "  heading = " << heading_ * kRad2Deg << "  vel = " << vel_
            << " m/s"
            << ", acc = " << acceleration_;
  ILOG_INFO << "car_static_timer_by_pos_strict = "
            << car_static_timer_by_pos_strict_ << " s";
  ILOG_INFO << "car_static_timer_by_pos_normal = "
            << car_static_timer_by_pos_normal_ << " s";
  ILOG_INFO << "car_static_timer_by_vel_strict_ = "
            << car_static_timer_by_vel_strict_ << " s";
  ILOG_INFO << "car_static_timer_by_vel_normal_ = "
            << car_static_timer_by_vel_normal_ << " s";
  ILOG_INFO << "static_flag = " << static_flag_
            << "  fold_mirror_flag = " << fold_mirror_flag_
            << "  brake_flag = " << brake_flag_;

  JSON_DEBUG_VALUE("local_move_dist", local_move_dist)
  JSON_DEBUG_VALUE("car_static_timer_by_pos_strict",
                   car_static_timer_by_pos_strict_)
  JSON_DEBUG_VALUE("car_static_timer_by_pos_normal",
                   car_static_timer_by_pos_normal_)
  JSON_DEBUG_VALUE("car_static_timer_by_vel_strict",
                   car_static_timer_by_vel_strict_)
  JSON_DEBUG_VALUE("car_static_timer_by_vel_normal",
                   car_static_timer_by_vel_normal_)
  JSON_DEBUG_VALUE("static_flag", static_flag_)
  JSON_DEBUG_VALUE("fold_mirror_flag", fold_mirror_flag_)
}

}  // namespace apa_planner
}  // namespace planning