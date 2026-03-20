#include "virtual_chassis.h"
#include <cmath>
#include "log_glog.h"
#include "pose2d.h"
#include "src/common/quaternion.h"
#include "tools/virtual_chassis/virtual_chassis_state.h"

namespace planning {
#define debug_virtual_chassis (1)

VirtualChassis::VirtualChassis(/* args */) {}

VirtualChassis::~VirtualChassis() {}

int VirtualChassis::PerfectResponse() {
  /* update steer */
  state_.steer_ = cmd_.steer_;
  state_.gear_ = cmd_.gear_;

  /* update velocity */
  double abs_expect_vel = std::fabs(state_.v_);
  abs_expect_vel += cmd_.acc_ * cmd_interval_;
  abs_expect_vel = std::max(0.0, abs_expect_vel);

  if (state_.gear_ == ChassisGearMode::reverse) {
    state_.v_ = -abs_expect_vel;
  } else {
    state_.v_ = abs_expect_vel;
  }

  state_.acc_ = cmd_.acc_;

  return 0;
}

int VirtualChassis::ChassisResponseByDelay() {
  state_.gear_ = cmd_.gear_;

  /* update steer */
  double delay_time = 0.2;

  double delta_steer = cmd_.steer_ - history_state_.steer_;
  delta_steer = delta_steer * cmd_interval_ / delay_time;

  state_.steer_ = history_state_.steer_ + delta_steer;

  /* update velocity */
  delay_time = 0.1;
  double abs_expect_vel = std::fabs(state_.v_);
  abs_expect_vel += cmd_.acc_ * cmd_interval_;
  abs_expect_vel = std::max(0.0, abs_expect_vel);

  double delta_speed = abs_expect_vel - std::fabs(history_state_.v_);

  if (0) {
    delta_speed = delta_speed * cmd_interval_ / delay_time;
  }

  if (state_.gear_ == ChassisGearMode::reverse) {
    state_.v_ = history_state_.v_ - delta_speed;
  } else {
    state_.v_ = history_state_.v_ + delta_speed;
  }

  state_.acc_ = cmd_.acc_;

  return 0;
}

int VirtualChassis::ChassisResponseBySpeed() {
  state_.gear_ = cmd_.gear_;

  /* update steer */
  double delay_time = 0.2;

  double delta_steer = cmd_.steer_ - history_state_.steer_;
  delta_steer = delta_steer * cmd_interval_ / delay_time;

  state_.steer_ = history_state_.steer_ + delta_steer;

  /* update velocity */
  delay_time = 0.1;
  double abs_expect_vel = std::fabs(cmd_.v_);
  abs_expect_vel = std::max(0.0, abs_expect_vel);

  if (state_.gear_ == ChassisGearMode::reverse) {
    state_.v_ = -abs_expect_vel;
  } else {
    state_.v_ = abs_expect_vel;
  }

  state_.acc_ = cmd_.acc_;

  return 0;
}

int VirtualChassis::ChassisResponseByDelayAndOvershoot() {
  double delay_time = 0.2;

  /* update steer */

  double overshoot = 2.0 * M_PI / 180.0;

  // 超调2度
  double cmd_steer = cmd_.steer_;
  if (cmd_steer > 0.0) {
    cmd_steer += overshoot;
  } else if (cmd_steer < 0.0) {
    cmd_steer -= overshoot;
  }

  double delta_steer = cmd_steer - history_state_.steer_;
  delta_steer = delta_steer * cmd_interval_ / delay_time;

  state_.steer_ = history_state_.steer_ + delta_steer;

  /* update velocity */
  delay_time = 0.1;
  double expect_v = std::fabs(state_.v_);
  expect_v += cmd_.acc_ * cmd_interval_;
  expect_v = std::max(0.0, expect_v);

  double delta_speed = expect_v - std::fabs(history_state_.v_);
  delta_speed = delta_speed * cmd_interval_ / delay_time;

  if (state_.gear_ == ChassisGearMode::reverse) {
    state_.v_ = history_state_.v_ - delta_speed;
  } else {
    state_.v_ = history_state_.v_ + delta_speed;
  }

  state_.acc_ = cmd_.acc_;

  return 0;
}

inline double VirtualChassis::CalcTurnRadius(const double lfr,
                                             const double steering) const {
  return lfr / std::tan(steering);
}

int VirtualChassis::UpdateChassisPose(Pose2D *veh_pose,
                                      const double steering_angle,
                                      const double move_dist,
                                      const VehicleParam *veh_params) {
  double radius;
  Position2D center_pos;

  if (std::fabs(steering_angle) > 0.00001) {
    radius = CalcTurnRadius(veh_params->wheel_base, steering_angle);

    // get circle center
    center_pos.x = veh_pose->x - radius * std::sin(veh_pose->theta);
    center_pos.y = veh_pose->y + radius * std::cos(veh_pose->theta);

    veh_pose->theta += move_dist / radius;
    veh_pose->x = center_pos.x + radius * std::sin(veh_pose->theta);
    veh_pose->y = center_pos.y - radius * std::cos(veh_pose->theta);
  } else {
    veh_pose->x += move_dist * std::cos(veh_pose->theta);
    veh_pose->y += move_dist * std::sin(veh_pose->theta);
  }

  return 0;
}

int VirtualChassis::Process(const ChassisCommand &cmd) {
  cmd_ = cmd;
  history_state_ = state_;

  if (cmd.control_mode_ == ChassisControlMode::acc) {
    ChassisErrorMode type = ChassisErrorMode::time_delay;
    switch (type) {
      case ChassisErrorMode::perfect_response:
        PerfectResponse();
        break;
      case ChassisErrorMode::time_delay:
        ChassisResponseByDelay();
        break;
      default:
        ChassisResponseByDelayAndOvershoot();
        break;
    }
  } else if (cmd.control_mode_ == ChassisControlMode::speed) {
    if (1) {
      ChassisResponseBySpeed();
    } else {
      PerfectResponse();
    }
  }

  Pose2D cur_pose;
  cur_pose.x = state_.x_;
  cur_pose.y = state_.y_;
  cur_pose.theta = state_.heading_;

  double move_dist = state_.v_ * cmd_interval_;
  UpdateChassisPose(&cur_pose, state_.steer_, move_dist, &veh_params_);

  state_.x_ = cur_pose.x;
  state_.y_ = cur_pose.y;
  double theta = IflyUnifyTheta(cur_pose.theta, M_PI);
  state_.heading_ = theta;

#if debug_virtual_chassis
  ILOG_INFO << state_.steer_ * 180 / M_PI << " " << state_.v_ << " "
            << state_.acc_ << " " << state_.x_ << " " << state_.y_ << " "
            << state_.heading_ << " " << cmd_interval_;

#endif

  return 0;
}

int VirtualChassis::Init(const VehicleParam &veh_param,
                         const double interval_second) {
  cmd_interval_ = 0.01;
  cmd_.acc_ = 0.0;
  cmd_.v_ = 0.0;
  cmd_.steer_ = 0.0;

  veh_params_ = veh_param;
  cmd_interval_ = interval_second;

  return 0;
}

}  // namespace planning