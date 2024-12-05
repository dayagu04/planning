#pragma once
#include "Eigen/Core"
#include "local_view.h"

namespace planning {
namespace apa_planner {
class ApaMeasureDataManager final {
 public:
  ApaMeasureDataManager() {}
  ~ApaMeasureDataManager() {}

  void Update(const LocalView* local_view_ptr);

  void Reset() {
    vel_ = 0.0;
    pos_.setZero();
    heading_ = 0.0;
    heading_vec_.setZero();
    car_static_timer_by_pos_strict_ = 0.0;
    car_static_timer_by_pos_normal_ = 0.0;
    car_static_timer_by_vel_strict_ = 0.0;
    car_static_timer_by_vel_normal_ = 0.0;
    static_flag_ = true;
    steer_wheel_angle_ = 0.0;
    front_wheel_angle_ = 0.0;
    brake_flag_ = false;
  }

  const double GetVel() const { return vel_; }
  void SetPose(const Eigen::Vector2d pos, const double heading) {
    pos_ = pos;
    heading_ = heading;
    heading_vec_ << std::cos(heading), std::sin(heading);
  }
  const Eigen::Vector2d GetPos() const { return pos_; }
  const double GetHeading() const { return heading_; }
  const Eigen::Vector2d GetHeadingVec() const { return heading_vec_; }
  const Eigen::Vector2d GetRightMirrorPos() const { return right_mirror_pos_; }
  const Eigen::Vector2d GetLeftMirrorPos() const { return left_mirror_pos_; }
  const double GetCarStaticTimerByPosStrict() const {
    return car_static_timer_by_pos_strict_;
  }
  const double GetCarStaticTimerByPosNormal() const {
    return car_static_timer_by_pos_normal_;
  }
  const double GetCarStaticTimerByVelStrict() const {
    return car_static_timer_by_vel_strict_;
  }
  const double GetCarStaticTimerByVelNormal() const {
    return car_static_timer_by_vel_normal_;
  }
  const bool GetStaticFlag() const { return static_flag_; }
  const double GetSteerWheelAngle() const { return steer_wheel_angle_; }
  const double GetFrontWheelAngle() const { return front_wheel_angle_; }
  const bool GetBrakeFlag() const { return brake_flag_; }

 private:
  double vel_ = 0.0;
  Eigen::Vector2d pos_ = Eigen::Vector2d::Zero();
  double heading_ = 0.0;
  Eigen::Vector2d heading_vec_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d right_mirror_pos_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d left_mirror_pos_ = Eigen::Vector2d::Zero();

  double car_static_timer_by_pos_strict_ = 0.0;
  double car_static_timer_by_pos_normal_ = 0.0;
  double car_static_timer_by_vel_strict_ = 0.0;
  double car_static_timer_by_vel_normal_ = 0.0;
  bool static_flag_ = true;

  double steer_wheel_angle_ = 0.0;
  double front_wheel_angle_ = 0.0;

  bool brake_flag_ = false;
};
}  // namespace apa_planner
}  // namespace planning