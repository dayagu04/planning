
#include "virtual_chassis_component.h"

#include "basic_types.pb.h"
#include "cyber/cyber.h"
#include "func_state_machine.pb.h"
#include "log_glog.h"
#include "pose2d.h"
#include "src/common/ifly_time.h"
#include "src/modules/common/config/vehicle_param_tmp.h"
#include "tools/virtual_chassis/virtual_chassis_state.h"
#include "transform2d.h"
#include "vehicle_service.pb.h"

namespace planning {

// 用来测试底盘不响应的case，观察规控.
// 0: 底盘不响应
// 1： 底盘响应
#define chassis_response_debug (1)

VirtualChassisComponent::VirtualChassisComponent(/* args */) {
  node_ = apollo::cyber::CreateNode(FilePath::GetName());
}

int VirtualChassisComponent::Init(const VehicleParam &veh_param,
                                  const Pose2D &start_point,
                                  double cycle_time) {
  chassis_writer_ =
      node_->CreateWriter<VehicleService::VehicleServiceOutputInfo>(
          "/iflytek/vehicle_service");

  localization_writer_ =
      node_->CreateWriter<LocalizationOutput::LocalizationEstimate>(
          "/iflytek/localization/ego_pose");

  control_reader_ = node_->CreateReader<ControlCommand::ControlOutput>(
      "/iflytek/control/control_command",
      [this](const std::shared_ptr<ControlCommand::ControlOutput>
                 &control_command) {
        ILOG_DEBUG << "Received planning data: run planning callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        control_command_.CopyFrom(*control_command);
      });

  planning_reader = node_->CreateReader<PlanningOutput::PlanningOutput>(
      "/iflytek/planning/plan",
      [this](const std::shared_ptr<PlanningOutput::PlanningOutput> &obj) {
        ILOG_DEBUG << "Received planning data: run planning callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        last_planning_.CopyFrom(*obj);
      });

  fsm_reader = node_->CreateReader<FuncStateMachine::FuncStateMachine>(
      "/iflytek/system_state/soc_state",
      [this](const std::shared_ptr<FuncStateMachine::FuncStateMachine> &obj) {
        ILOG_DEBUG << "Received planning data: run planning callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        fsm_.CopyFrom(*obj);
      });

  latest_localization_ =
      std::make_shared<LocalizationOutput::LocalizationEstimate>();
  latest_chassis_ =
      std::make_shared<VehicleService::VehicleServiceOutputInfo>();

  loop_time_s_ = cycle_time;

  chassis_.Init(veh_param, cycle_time);

  chassis_.SetInitChassisState(start_point.x, start_point.y, 0.0,
                               start_point.theta);

  veh_param_ = veh_param;

  // set initial vehicle state by cmd
  // need to sleep, because advertised channel is not ready immediately
  // simple test shows a short delay of 80 ms or so
  ILOG_INFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  return 0;
}

int VirtualChassisComponent::PublishMessages() {
  double timestamp_us = IflyTime::Now_us();

  ChassisState state;

  chassis_.GetState(&state);

  GenerateChassisMsg(timestamp_us, latest_chassis_, state);

  latest_chassis_->set_brake_pedal_pos(0);
  latest_chassis_->set_brake_pedal_pos_available(true);

  latest_chassis_->set_brake_pedal_pressed(false);
  latest_chassis_->set_brake_pedal_pressed_available(true);

  latest_chassis_->set_shift_lever_state_available(true);
  latest_chassis_->set_gear_lever_state_available(true);

  switch (local_view_.control_.gear_command_value()) {
    case Common::GearCommandValue::GEAR_COMMAND_VALUE_REVERSE:
      latest_chassis_->set_shift_lever_state(1);
      latest_chassis_->set_gear_lever_state(1);
      break;
    case Common::GearCommandValue::GEAR_COMMAND_VALUE_DRIVE:
      latest_chassis_->set_shift_lever_state(3);
      latest_chassis_->set_gear_lever_state(3);
      break;
    case Common::GearCommandValue::GEAR_COMMAND_VALUE_PARKING:
      latest_chassis_->set_shift_lever_state(0);
      latest_chassis_->set_gear_lever_state(0);
      break;

    default:
      latest_chassis_->set_shift_lever_state(2);
      latest_chassis_->set_gear_lever_state(2);
      break;
  }

  latest_chassis_->set_door_lock_state(true);
  latest_chassis_->set_door_lock_state_available(true);

  latest_chassis_->set_parking_long_control_actuator_status_available(true);
  latest_chassis_->set_parking_long_control_actuator_status(2);

  latest_chassis_->set_parking_lat_control_actuator_status_available(true);
  latest_chassis_->set_parking_lat_control_actuator_status(2);

  latest_chassis_->set_aeb_actuator_status_available(true);
  latest_chassis_->set_aeb_actuator_status(2);

  latest_chassis_->set_auto_hold_state_available(true);
  latest_chassis_->set_auto_hold_state(0);

  latest_chassis_->set_epb_state_available(true);
  latest_chassis_->set_epb_state(0);

  GenerateLocalizationMsg(latest_localization_, timestamp_us, state);

  // latest_chassis_->set_speed_mps(20.0);

  chassis_writer_->Write(latest_chassis_);

  localization_writer_->Write(latest_localization_);

  // debug
#if 0
  ILOG_INFO << latest_localization_->DebugString();
#endif

  return 0;
}

int VirtualChassisComponent::Process() {
  {
    std::lock_guard<std::mutex> lock(mutex_);

    local_view_.control_.Clear();
    local_view_.control_.CopyFrom(control_command_);

    local_view_.planning_.Clear();
    local_view_.planning_.CopyFrom(last_planning_);

    local_view_.fsm_.Clear();
    local_view_.fsm_.CopyFrom(fsm_);
  }

  bool is_control_cmd_valid = true;
  if (!local_view_.control_.has_header() ||
      !local_view_.control_.has_acceleration() ||
      !local_view_.control_.has_steering()) {
    ILOG_ERROR << "Control msg invalid";
    is_control_cmd_valid = false;
  }

  // use control command to update vehicle pose, has a bug. todo: fix it.
  bool use_control_update_veh = false;

  if (is_control_cmd_valid && use_control_update_veh) {
    GetVirtualChassisCommandByControl(&virtual_chassis_cmd_,
                                      local_view_.control_);

    // ILOG_INFO << local_view_.control_.DebugString();

    chassis_.Process(virtual_chassis_cmd_);
  } else if (local_view_.planning_.trajectory().trajectory_points_size() > 0 &&
             local_view_.fsm_.current_state() ==
                 FuncStateMachine::PARK_IN_ACTIVATE_CONTROL) {
    // use traj to update vehicle pose
    ChassisState state;

    chassis_.GetState(&state);

    double min_dist = 100000.0;
    double tmp_dist;
    int min_dist_index = 0;

    double x_diff;
    double y_diff;

    for (int i = 0;
         i < local_view_.planning_.trajectory().trajectory_points_size(); i++) {
      x_diff = state.x_ -
               local_view_.planning_.trajectory().trajectory_points(i).x();
      y_diff = state.y_ -
               local_view_.planning_.trajectory().trajectory_points(i).y();

      tmp_dist = std::sqrt(x_diff * x_diff + y_diff * y_diff);

      if (tmp_dist < min_dist) {
        min_dist = tmp_dist;
        min_dist_index = i;
      }
    }

    switch (local_view_.planning_.gear_command().gear_command_value()) {
      case Common::GEAR_COMMAND_VALUE_DRIVE:
        state.v_ = 0.5;
        break;
      case Common::GEAR_COMMAND_VALUE_REVERSE:
        state.v_ = -0.5;
        break;
      default:
        state.v_ = 0.0;
        break;
    }

    double move_dist = state.v_ * loop_time_s_;

    // 车辆预测位置，直线预测
    Pose2D cur_pose;

    cur_pose.x = state.x_ + move_dist * std::cos(state.heading_);
    cur_pose.y = state.y_ + move_dist * std::sin(state.heading_);
    cur_pose.theta = state.heading_;

    if (min_dist_index + 1 <
        local_view_.planning_.trajectory().trajectory_points_size()) {
      Pose2D traj_min_pose;

      auto &point =
          local_view_.planning_.trajectory().trajectory_points(min_dist_index);

      traj_min_pose.x = point.x();
      traj_min_pose.y = point.y();
      traj_min_pose.theta = point.heading_yaw();

      // get projection point
      Pose2D local_pos;

      CvtPoseGlobalToLocal(&local_pos, &cur_pose, &traj_min_pose);

      // 新点在y轴方向上
      local_pos.x = 0.0;
      Pose2D global_pos;

      Transform2d tf;
      tf.SetBasePose(traj_min_pose);
      tf.RUFLocalPoseToGlobal(&global_pos, local_pos);

      state.x_ = global_pos.x;
      state.y_ = global_pos.y;
      state.heading_ = traj_min_pose.theta;

    } else {
      min_dist_index =
          local_view_.planning_.trajectory().trajectory_points_size() - 1;

      auto &point =
          local_view_.planning_.trajectory().trajectory_points(min_dist_index);

      state.x_ = point.x();
      state.y_ = point.y();
      state.heading_ = point.heading_yaw();

      state.v_ = 0.0;
    }

    switch (local_view_.planning_.gear_command().gear_command_value()) {
      case Common::GEAR_COMMAND_VALUE_DRIVE:
        state.gear_ = ChassisGearMode::drive;
        break;
      case Common::GEAR_COMMAND_VALUE_REVERSE:
        state.gear_ = ChassisGearMode::reverse;
        break;
      default:
        state.gear_ = ChassisGearMode::park;
        break;
    }

    chassis_.SetState(state);
  }

  return 0;
}

int VirtualChassisComponent::GetVirtualChassisCommandByControl(
    ChassisCommand *virtual_cmd,
    const ControlCommand::ControlOutput &control_command) {
  double steering_wheel_angle;
  steering_wheel_angle = control_command.steering() * M_PI / 180.0;

  double front_wheel_angle;
  front_wheel_angle = steering_wheel_angle / veh_param_.steer_ratio;

  // set virtual cmd
  virtual_cmd->steer_ = front_wheel_angle;

  virtual_cmd->acc_ = control_command.acceleration();

  // for, control vehicle by speed, not acc
  virtual_cmd->control_mode_ = ChassisControlMode::speed;

  ILOG_INFO << control_command.control_status().DebugString();
  ILOG_INFO << Common::GearCommandValue_Name(
      control_command.gear_command_value());
  ILOG_INFO << " wheel  "
            << control_command.steering() / veh_param_.steer_ratio;

  if (control_command.control_status().control_status_type() == 0 ||
      control_command.control_status().control_status_type() == 3) {
    virtual_cmd->v_ = 0.0;
  } else {
    switch (control_command.gear_command_value()) {
      case Common::GearCommandValue::GEAR_COMMAND_VALUE_REVERSE:
        virtual_cmd->v_ = 0.5;
        virtual_cmd->gear_ = ChassisGearMode::reverse;
        break;
      case Common::GearCommandValue::GEAR_COMMAND_VALUE_DRIVE:
        virtual_cmd->v_ = 0.5;
        virtual_cmd->gear_ = ChassisGearMode::drive;
        break;

      default:
        virtual_cmd->v_ = 0.0;
        virtual_cmd->gear_ = ChassisGearMode::park;
        break;
    }
  }

  return 0;
}

int VirtualChassisComponent::GetVirtualChassisCommandByTraj(
    ChassisCommand *virtual_cmd,
    const ControlCommand::ControlOutput &control_command) {
  double steering_wheel_angle;
  steering_wheel_angle = control_command.steering() * M_PI / 180.0;

  double front_wheel_angle;
  front_wheel_angle = steering_wheel_angle / veh_param_.steer_ratio;

  Pose2D left;
  Pose2D middle;
  Pose2D right;
  double r;

  CalcRadiusByThreePoint(&r, &left, &middle, &right);

  front_wheel_angle = std::atan(veh_param_.wheel_base / r);

  // set virtual cmd
  virtual_cmd->steer_ = front_wheel_angle;

  virtual_cmd->acc_ = control_command.acceleration();

  // for, control vehicle by speed, not acc
  virtual_cmd->control_mode_ = ChassisControlMode::speed;

  ILOG_INFO << control_command.control_status().DebugString();
  ILOG_INFO << Common::GearCommandValue_Name(
      control_command.gear_command_value());

  if (control_command.control_status().control_status_type() == 0 ||
      control_command.control_status().control_status_type() == 3) {
    virtual_cmd->v_ = 0.0;
  } else {
    switch (control_command.gear_command_value()) {
      case Common::GearCommandValue::GEAR_COMMAND_VALUE_REVERSE:
        virtual_cmd->v_ = 0.5;
        virtual_cmd->gear_ = ChassisGearMode::reverse;
        break;
      case Common::GearCommandValue::GEAR_COMMAND_VALUE_DRIVE:
        virtual_cmd->v_ = 0.5;
        virtual_cmd->gear_ = ChassisGearMode::drive;
        break;

      default:
        virtual_cmd->v_ = 0.0;
        virtual_cmd->gear_ = ChassisGearMode::park;
        break;
    }
  }

  return 0;
}
}  // namespace planning