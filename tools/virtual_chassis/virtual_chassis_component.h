#pragma once

#include <cmath>
#include <memory>
#include <string>

#include "chassis_local_view.h"
#include "cyber/class_loader/class_loader.h"
#include "cyber/component/timer_component.h"
#include "cyber/time/time.h"
#include "log_glog.h"
#include "pose2d.h"
#include "src/common/quaternion.h"
#include "tools/virtual_chassis/virtual_chassis_state.h"
#include "virtual_chassis.h"

namespace planning {

//虚拟底盘
class VirtualChassisComponent {
 private:
  // publish msg
  std::shared_ptr<LocalizationOutput::LocalizationEstimate>
      latest_localization_;

  std::shared_ptr<
      apollo::cyber::Writer<LocalizationOutput::LocalizationEstimate>>
      localization_writer_;

  // pub
  std::shared_ptr<VehicleService::VehicleServiceOutputInfo> latest_chassis_;

  // pub
  std::shared_ptr<
      apollo::cyber::Writer<VehicleService::VehicleServiceOutputInfo>>
      chassis_writer_;

  // reader
  std::shared_ptr<apollo::cyber::Reader<ControlCommand::ControlOutput>>
      control_reader_;
  ControlCommand::ControlOutput control_command_;

  // reader
  std::shared_ptr<apollo::cyber::Reader<PlanningOutput::PlanningOutput>>
      planning_reader;
  PlanningOutput::PlanningOutput last_planning_;

  std::shared_ptr<apollo::cyber::Reader<FuncStateMachine::FuncStateMachine>>
      fsm_reader;
  FuncStateMachine::FuncStateMachine fsm_;

  // node
  std::shared_ptr<apollo::cyber::Node> node_;

  std::mutex mutex_;

  VirtualChassis chassis_;

  // control cmd for virtual chassis
  ChassisCommand virtual_chassis_cmd_;

  VirtualChassisLocalView local_view_;

  VehicleParam veh_param_;

  double loop_time_s_;

 public:
  VirtualChassisComponent(/* args */);
  ~VirtualChassisComponent(){};

  int Init(const VehicleParam &veh_param, const Pose2D &start_point,
           double cycle_time);

  int PublishMessages();

  int Process();

  int GetVirtualChassisCommandByControl(
      ChassisCommand *virtual_cmd,
      const ControlCommand::ControlOutput &control_command);

  int GetVirtualChassisCommandByTraj(
      ChassisCommand *virtual_cmd,
      const ControlCommand::ControlOutput &control_command);

 private:
  int CalcRadiusByThreePoint(double *radius, const Pose2D *p1, const Pose2D *p2,
                             const Pose2D *p3) {
    double delta_theta, delta_dist;

    delta_theta = p3->theta - p1->theta;
    delta_theta = IflyUnifyTheta(delta_theta, M_PI);
    delta_dist = p1->DistanceTo(p2) + p2->DistanceTo(p3);

    *radius = ifly_fequal(delta_theta, 0.0)
                  ? 1000000000
                  : delta_dist / std::fabs(delta_theta);

    if (delta_theta > 0.0) *radius = -(*radius);

    return 1;
  };

  void GenerateLocalizationMsg(
      std::shared_ptr<LocalizationOutput::LocalizationEstimate> &localization,
      const double curr_time_stamp, const ChassisState &state) {
    localization->mutable_header()->set_timestamp(curr_time_stamp);
    auto *pose = localization->mutable_pose();

    // Set position
    pose->mutable_local_position()->set_x(state.x_);
    pose->mutable_local_position()->set_y(state.y_);

    // Set orientation and heading
    double cur_theta = state.heading_;

    Eigen::Quaternion<double> cur_orientation =
        planning::planning_math::HeadingToQuaternion<double>(cur_theta);

    pose->mutable_orientation()->set_qw(cur_orientation.w());
    pose->mutable_orientation()->set_qx(cur_orientation.x());
    pose->mutable_orientation()->set_qy(cur_orientation.y());
    pose->mutable_orientation()->set_qz(cur_orientation.z());

    pose->set_heading(cur_theta);

    // Set linear_velocity
    pose->set_linear_velocity_from_wheel(state.v_);

    // Set angular_velocity in both map reference frame and vehicle
    // reference frame
    double kappa;
    kappa = std::tan(state.steer_) / veh_param_.wheel_base;

    pose->mutable_angular_velocity()->set_x(0);
    pose->mutable_angular_velocity()->set_y(0);
    if (state.v_ > 0.0) {
      pose->mutable_angular_velocity()->set_z(state.v_ * kappa);
    } else {
      pose->mutable_angular_velocity()->set_z(0);
    }

    // Set linear_acceleration in both map reference frame and vehicle
    // reference frame
    auto *linear_acceleration = pose->mutable_linear_acceleration();
    double acc = 0.0;

    if (state.v_ > 0.0) {
      acc = state.acc_;
    }
    linear_acceleration->set_x(std::cos(cur_theta) * acc);
    linear_acceleration->set_y(std::sin(cur_theta) * acc);
    linear_acceleration->set_z(0);

    // euler_angles
    planning::planning_math::EulerAnglesZXYd euler_angles(
        cur_orientation.w(), cur_orientation.x(), cur_orientation.y(),
        cur_orientation.z());
    pose->mutable_euler_angles()->set_roll(euler_angles.roll());
    pose->mutable_euler_angles()->set_pitch(euler_angles.pitch());
    pose->mutable_euler_angles()->set_yaw(euler_angles.yaw());

    return;
  }

  int GenerateChassisMsg(
      const double timestamp_us,
      std::shared_ptr<VehicleService::VehicleServiceOutputInfo> &ptr_chassis,
      const ChassisState &state) {
    if (ptr_chassis == nullptr) {
      ILOG_ERROR << "nullptr";
      return -1;
    }

    // set
    ptr_chassis->mutable_header()->set_timestamp(timestamp_us);

    ptr_chassis->set_turn_switch_state(0);
    ptr_chassis->set_turn_switch_state_available(true);

    float speed_mps = std::fabs(state.v_);
    float brake_percentage = -1.0;
    double front_wheel_angle = state.steer_;

    double steering_wheel_angle = front_wheel_angle * veh_param_.steer_ratio;

    ptr_chassis->set_steering_wheel_angle(steering_wheel_angle);

    ptr_chassis->set_vehicle_speed(speed_mps);

    return 0;
  }
};

}  // namespace planning