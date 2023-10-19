#include "general_planning.h"

#include <cmath>

#include "apa_planner/common/apa_utils.h"
#include "config/vehicle_param.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "ifly_time.h"
#include "log.h"
#include "math/math_utils.h"
#include "planning_context.h"
#include "planning_debug_info.pb.h"
#include "planning_output_context.h"
#include "scene_type_config.pb.h"
#include "utils/lateral_utils.h"
#include "vehicle_config_context.h"

namespace planning {

using ::FuncStateMachine::FunctionalState;

GeneralPlanning::GeneralPlanning() { Init(); }

GeneralPlanning::~GeneralPlanning() {}

void GeneralPlanning::Init() {
  session_.Init();
  scheduler_.Init(&session_);
  VehicleParam vehicle_param;
  // session->mutable_vehicle_config_context()->load_vehicle_param();
  session_.mutable_vehicle_config_context()->set_vehicle_param(vehicle_param);
  EnvironmentalModel *environmental_model =
      session_.mutable_environmental_model();
  planning::common::SceneType scene_type = session_.get_scene_type();
  auto config_builder =
      session_.environmental_model().config_builder(scene_type);
  config_ = config_builder->cast<GeneralPlanningConfig>();
  environmental_model->set_vehicle_param(
      session_.vehicle_config_context().get_vehicle_param());
}

bool GeneralPlanning::RunOnce(
    const LocalView &local_view,
    PlanningOutput::PlanningOutput *const planning_output,
    common::PlanningDebugInfo &debug_info,
    PlanningHMI::PlanningHMIOutputInfoStr *const planning_hmi_info) {
  LOG_ERROR("GeneralPlanning::RunOnce \n");
  frame_num_++;
  local_view_ = local_view;
  session_.mutable_planning_output_context()->feed_planning_hmi_info(
      planning_hmi_info);

  double start_timestamp = IflyTime::Now_ms();
  EnvironmentalModel *environmental_model =
      session_.mutable_environmental_model();

  environmental_model->feed_local_view(local_view);  // todo

  const auto &state_machine = local_view.function_state_machine_info;
  if (state_machine.has_current_state()) {
    if (IsUndefinedScene(state_machine.current_state())) {
      session_.set_scene_type(planning::common::SceneType::HIGHWAY);
      ClearParkingInfo(planning_output);
    } else if (IsValidParkingState(state_machine.current_state())) {
      session_.set_scene_type(planning::common::SceneType::PARKING_APA);
    } else {
      session_.set_scene_type(planning::common::SceneType::HIGHWAY);
      ClearParkingInfo(planning_output);
    }
  }

  if (session_.is_parking_scene()) {
    scheduler_.RunOnce();
    *planning_output = session_.planning_output_context()
                           .planning_status()
                           .planning_result.planning_output;
    return true;
  }

  auto pre_planning_status = session_.mutable_planning_output_context()
                                 ->mutable_prev_planning_status();
  *pre_planning_status =
      session_.mutable_planning_output_context()->planning_status();
  auto *planning_status =
      session_.mutable_planning_output_context()->mutable_planning_status();
  planning_status->pre_planning_result = planning_status->planning_result;
  planning_status->planning_result.next_timestamp = start_timestamp;

  printf("VERSION: 2023-03-31 \n");
  // 1.校验输入 TBD

  if (reset_pnc_) {
    // reset dbw when hdmap_valid is changed
    environmental_model->UpdateVehicleDbwStatus(false);
    reset_pnc_ = false;
  }

  // 开始执行规划部分
  scheduler_.RunOnce();

  bool planning_success = session_.planning_context().planning_success();

  planning_status->planning_success = planning_success;
  auto end_timestamp = IflyTime::Now_ms();
  planning_status->time_consumption = end_timestamp - start_timestamp;
  LOG_DEBUG("general planning: planning time cost %f\n",
            planning_status->time_consumption);
  planning_status->planning_result.timestamp =
      planning_status->planning_result.next_timestamp;

  // when planning succeed, update the planning_output
  if (planning_status->planning_success) {
    FillPlanningTrajectory(start_timestamp, planning_output);
    FillPlanningHmiInfo(start_timestamp, planning_hmi_info);
    std::cout << "The RunOnce is successed !!!!:" << std::endl;
  } else {
    LOG_DEBUG("RunOnce failed !!!! \n");
  }

  int64_t frame_duration = IflyTime::Now_ms() - start_timestamp;
  LOG_DEBUG("The time cost of RunOnce is: %d\n", (int)frame_duration);

  auto frame_info = debug_info.mutable_frame_info();
  frame_info->set_frame_num(frame_num_);
  frame_info->set_scene_type(common::SceneType_Name(session_.get_scene_type()));
  frame_info->set_frame_duration_ms(frame_duration);
  frame_info->set_planning_succ(planning_success);
  std::cout << "frame_info=" << frame_info->ShortDebugString() << std::endl;

  return planning_status->planning_success;
}

void GeneralPlanning::FillPlanningTrajectory(
    double start_time, PlanningOutput::PlanningOutput *const planning_output) {
  // 获取计算结果
  const auto &lateral_output =
      session_.planning_context().lateral_behavior_planner_output();
  const auto &vision_only_longitudinal_outputs =
      session_.planning_context().vision_longitudinal_behavior_planner_output();
  auto &planning_result = session_.planning_context().planning_result();
  auto &planning_context = session_.planning_context();
  auto &ego_state = session_.environmental_model().get_ego_state_manager();
  auto function_info = session_.environmental_model().function_info();
  const bool active = session_.environmental_model().GetVehicleDbwStatus();
  auto virtual_lane_manager =
      session_.environmental_model().get_virtual_lane_manager();

  // 更新输出
  auto time_stamp_us = IflyTime::Now_us();

  planning_output->mutable_meta()->set_plan_timestamp_us(time_stamp_us);
  planning_output->mutable_meta()->set_plan_strategy_name("Test");

  // 2.Trajectory
  auto trajectory = planning_output->mutable_trajectory();
  // set trajectory false when dbw is false
  trajectory->set_available(active);

  // 根据定位有效性决定实时、长时
  auto location_valid = session_.environmental_model().location_valid();
  if (location_valid) {
    trajectory->set_trajectory_type(
        Common::TrajectoryType::TRAJECTORY_TYPE_TRAJECTORY_POINTS);
    trajectory->mutable_trajectory_points()->Clear();
    trajectory->mutable_target_reference()->Clear();
    for (size_t i = 0; i < planning_result.traj_points.size(); i++) {
      auto path_point = trajectory->add_trajectory_points();
      path_point->set_x(planning_result.traj_points[i].x);
      path_point->set_y(planning_result.traj_points[i].y);
      path_point->set_heading_yaw(planning_result.traj_points[i].heading_angle);
      path_point->set_curvature(planning_result.traj_points[i].curvature);
      path_point->set_t(planning_result.traj_points[i].t);
      path_point->set_v(planning_result.traj_points[i].v);
      path_point->set_a(planning_result.traj_points[i].a);
      path_point->set_distance(planning_result.traj_points[i].s);
      path_point->set_jerk(0.0);  // TBD
    }
    // 设置参考线为default
    auto target_ref = trajectory->mutable_target_reference();
    target_ref->add_polynomial(0.0);
    target_ref->set_target_velocity(0.0);
    auto acceleration_range_limit =
        target_ref->mutable_acceleration_range_limit();
    acceleration_range_limit->set_min_a(-4.0);
    acceleration_range_limit->set_max_a(4.0);
    target_ref->set_lateral_maneuver_gear(
        Common::LateralManeuverGear::LATERAL_MANEUVER_GEAR_NORMAL);
  } else {
    // set vision_only_longitudinal_outputs if hdmpa valid is false
    trajectory->set_trajectory_type(
        Common::TrajectoryType::TRAJECTORY_TYPE_TARGET_REFERENCE);
    trajectory->mutable_trajectory_points()->Clear();
    trajectory->mutable_target_reference()->Clear();
    // 设置轨迹为default
    auto path_point = trajectory->add_trajectory_points();
    path_point->set_x(0.0);
    path_point->set_y(0.0);
    path_point->set_heading_yaw(0.0);
    path_point->set_curvature(0.0);
    path_point->set_t(0.0);
    path_point->set_v(0.0);
    path_point->set_a(0.0);
    path_point->set_distance(0.0);
    path_point->set_jerk(0.0);

    auto target_ref = trajectory->mutable_target_reference();
    // add polynomial
    // clip the polynomial C_3
    // const double lat_offset_rate = config.lat_offset_rate();
    // const double max_lat_offset = config.max_lat_offset();
    const double lat_offset_rate = 0.2;
    const double max_lat_offset = 2;
    const double presee_x_dist = 50.;
    static double limited_polynomial_3 = 0.0;
    const auto &d_polynomial = lateral_output.d_poly;

    bool enable_presee = virtual_lane_manager->dis_to_ramp() < 1000. &&
                         lateral_output.lc_status == "right_lane_change";

    double presee_y_dist =
        enable_presee ? calc_poly1d(d_polynomial, presee_x_dist) : 0.;
    LOG_DEBUG("presee_y_dist: [%f]: \n", presee_y_dist);

    if (d_polynomial.size() == 4) {
      if (std::fabs(d_polynomial[3] + presee_y_dist) * 0.5 > max_lat_offset) {
        limited_polynomial_3 += planning_math::Clamp(
            d_polynomial[3], -lat_offset_rate, lat_offset_rate);
      } else {
        if ((lateral_output.lc_status == "left_lane_change_back" ||
             lateral_output.lc_status == "right_lane_change_back") &&
            std::fabs(d_polynomial[3]) >
                config_.lc_back_consider_smooth_dpoly_thr) {
          limited_polynomial_3 =
              planning_math::Clamp(d_polynomial[3], -config_.lc_back_smooth_thr,
                                   config_.lc_back_smooth_thr);
        } else {
          limited_polynomial_3 = planning_math::Clamp(
              d_polynomial[3], -max_lat_offset, max_lat_offset);
        }
      }
      limited_polynomial_3 = planning_math::Clamp(
          limited_polynomial_3, -max_lat_offset, max_lat_offset);
    }
    std::cout << "smooth dpoly enable_none_smooth: "
              << config_.enable_none_smooth
              << "   config_.none_consider_slope_thr:   "
              << config_.none_consider_slope_thr << std::endl;
    LOG_DEBUG("limited_polynomial_3: [%f]: \n", limited_polynomial_3);
    LOG_DEBUG("lateral_output.d_poly C0: [%f] C1: [%f] C2: [%f] C3: [%f] \n",
              lateral_output.d_poly[0], lateral_output.d_poly[1],
              lateral_output.d_poly[2], lateral_output.d_poly[3]);
    std::vector<double> polynomial_limited(4);
    polynomial_limited[0] = d_polynomial[0];
    polynomial_limited[1] = d_polynomial[1];
    polynomial_limited[2] = d_polynomial[2];
    polynomial_limited[3] = limited_polynomial_3;
    LOG_DEBUG("limited_polynomial C0: [%f] C1: [%f] C2: [%f] C3: [%f] \n",
              polynomial_limited[0], polynomial_limited[1],
              polynomial_limited[2], polynomial_limited[3]);
    for (size_t i = 0; i < polynomial_limited.size(); i++) {
      target_ref->add_polynomial(polynomial_limited[i]);
    }

    target_ref->set_target_velocity(
        vision_only_longitudinal_outputs.velocity_target);

    auto acceleration_range_limit =
        target_ref->mutable_acceleration_range_limit();
    acceleration_range_limit->set_min_a(
        vision_only_longitudinal_outputs.a_target_min);
    acceleration_range_limit->set_max_a(
        vision_only_longitudinal_outputs.a_target_max);
    target_ref->set_lateral_maneuver_gear(
        Common::LateralManeuverGear::LATERAL_MANEUVER_GEAR_NORMAL);
  }
  // 3.Turn signal
  auto turn_signal = planning_output->mutable_turn_signal_command();
  turn_signal->set_available(true);
  if (planning_result.turn_signal == NO_CHANGE) {
    turn_signal->set_turn_signal_value(
        Common::TurnSignalType::TURN_SIGNAL_TYPE_NONE);
  } else if (planning_result.turn_signal == LEFT_CHANGE) {
    turn_signal->set_turn_signal_value(
        Common::TurnSignalType::TURN_SIGNAL_TYPE_LEFT);
  } else {
    turn_signal->set_turn_signal_value(
        Common::TurnSignalType::TURN_SIGNAL_TYPE_RIGHT);
  }
  // WB start:--------临时hack以下信号--------
  // 4.Light signal
  auto light_signal = planning_output->mutable_light_signal_command();
  light_signal->set_available(true);
  light_signal->set_light_signal_value(
      Common::LightSignalType::LIGHT_SIGNAL_TYPE_NONE);

  // 5.Horn signal
  auto horn_signal_command = planning_output->mutable_horn_signal_command();
  horn_signal_command->set_available(true);
  horn_signal_command->set_horn_signal_value(
      Common::HornSignalType::HORN_SIGNAL_TYPE_NONE);

  // 6.Gear signal
  auto gear_command = planning_output->mutable_gear_command();
  gear_command->set_available(true);
  // 需要获取目标挡位值
  auto gear = Common::GearCommandValue::GEAR_COMMAND_VALUE_DRIVE;
  gear_command->set_gear_command_value(gear);

  // 7.Open loop steering command
  auto open_loop_steering_command =
      planning_output->mutable_open_loop_steering_command();
  open_loop_steering_command->set_available(true);
  open_loop_steering_command->set_jerk_factor(70.0);  // hack
  open_loop_steering_command->set_need_steering_wheel_stationary(false);
  open_loop_steering_command->set_steering_wheel_rad_limit(0.1);

  // 8.Planning status
  auto planning_status = planning_output->mutable_planning_status();
  planning_status->set_standstill(false);
  if (function_info.function_mode == DrivingFunctionMode::ACC ||
      function_info.function_mode == DrivingFunctionMode::SCC) {
    planning_status->set_standstill(std::fabs(ego_state->ego_v()) < 0.1);
  }
  // 启停状态机
  planning_status->set_ready_to_go(planning_context.start_stop_result().state !=
                                   StartStopInfo::STOP);
  planning_status->set_apa_planning_status(
      PlanningOutput::ApaPlanningStatus::NONE);
  // WB end:--------临时hack以上信号--------
}

void GeneralPlanning::GenerateStopTrajectory(
    double start_time, PlanningOutput::PlanningOutput *const planning_output) {
  // 更新输出
  planning_output->mutable_meta()->set_plan_timestamp_us(IflyTime::Now_ms());

  auto trajectory = planning_output->mutable_trajectory();
  // Hack: 长时规划
  trajectory->set_trajectory_type(
      Common::TrajectoryType::TRAJECTORY_TYPE_TRAJECTORY_POINTS);
  trajectory->mutable_trajectory_points()->Clear();
  trajectory->mutable_target_reference()->Clear();
  double t = 0.0;
  for (size_t i = 0; i < 21; i++) {
    t = 0.1 * i;
    auto path_point = trajectory->add_trajectory_points();
    path_point->set_x(0.0);
    path_point->set_y(0.0);
    path_point->set_heading_yaw(0.0);
    path_point->set_curvature(0.0);
    path_point->set_t(t);
    path_point->set_v(0.0);
    path_point->set_a(0.0);
    path_point->set_distance(0.0);
    path_point->set_jerk(0.0);  // TBD
  }
}

void GeneralPlanning::FillPlanningHmiInfo(
    double start_timestamp,
    PlanningHMI::PlanningHMIOutputInfoStr *const planning_hmi_info) {
  const auto &lateral_output =
      session_.planning_context().lateral_behavior_planner_output();
  const auto &state_machine_output =
      session_.mutable_planning_context()
          ->mutable_lat_behavior_state_machine_output();

  planning_hmi_info->mutable_header()->set_timestamp(IflyTime::Now_us());
  // HMI for alc
  auto alc_output_pb = planning_hmi_info->mutable_alc_output_info();
  alc_output_pb->set_lc_request(lateral_output.lc_request);
  alc_output_pb->set_lc_status(lateral_output.lc_status);
  alc_output_pb->set_lc_invalid_reason(state_machine_output.lc_invalid_reason);
  alc_output_pb->set_lc_back_reason(
      state_machine_output.lc_back_invalid_reason);
  // HMI for ldw
  auto lkas_info =
      session_.mutable_planning_context()->lane_keep_assit_function();
  planning_hmi_info->mutable_ldw_output_info()->set_ldw_state(
      lkas_info->get_ldw_state_info());
  planning_hmi_info->mutable_ldw_output_info()->set_ldw_left_warning(
      lkas_info->get_ldw_left_warning_info());
  planning_hmi_info->mutable_ldw_output_info()->set_ldw_right_warning(
      lkas_info->get_ldw_right_warning_info());
  // HMI for ldp
  planning_hmi_info->mutable_ldp_output_info()->set_ldp_state(
      lkas_info->get_ldp_state_info());
  planning_hmi_info->mutable_ldp_output_info()->set_ldp_left_intervention_flag(
      lkas_info->get_ldp_left_intervention_flag_info());
  planning_hmi_info->mutable_ldp_output_info()->set_ldp_right_intervention_flag(
      lkas_info->get_ldp_right_intervention_flag_info());
  // HMI for elk
  planning_hmi_info->mutable_elk_output_info()->set_elk_state(
      lkas_info->get_elk_state_info());
  planning_hmi_info->mutable_elk_output_info()->set_elk_left_intervention_flag(
      lkas_info->get_elk_left_intervention_flag_info());
  planning_hmi_info->mutable_elk_output_info()->set_elk_right_intervention_flag(
      lkas_info->get_elk_right_intervention_flag_info());
  // HMI for ihc
  auto ihc_info = session_.mutable_planning_context()
                      ->intelligent_headlight_control_function();
  planning_hmi_info->mutable_ihc_output_info()->set_ihc_state(
      ihc_info->get_ihc_state_info());
  planning_hmi_info->mutable_ihc_output_info()->set_ihc_request(
      ihc_info->get_ihc_request_info());
  planning_hmi_info->mutable_ihc_output_info()->set_ihc_request_status(
      ihc_info->get_ihc_request_status_info());
  // HMI for tsr
  auto tsr_info =
      session_.mutable_planning_context()->traffic_sign_recognition_function();
  planning_hmi_info->mutable_tsr_output_info()->set_tsr_state(
      tsr_info->get_tsr_state_info());
  planning_hmi_info->mutable_tsr_output_info()->set_tsr_warning(
      tsr_info->get_tsr_warning_info());
  planning_hmi_info->mutable_tsr_output_info()->set_tsr_speed_limit(
      tsr_info->get_tsr_speed_limit_info());
  // HMI for CIPV
  // TBD: 后续需要丰富障碍物的信息，后车、侧方车辆等
  auto cipv_info =
      session_.planning_output_context().mutable_planning_hmi_info()->mutable_cipv_info();
  planning_hmi_info->mutable_cipv_info()->set_has_cipv(cipv_info->has_cipv());
  planning_hmi_info->mutable_cipv_info()->set_cipv_id(cipv_info->cipv_id());

  auto &planning_result = session_.planning_context().planning_result();
  auto ad_info = session_.mutable_planning_output_context()
                     ->mutable_planning_hmi_info()
                     ->mutable_ad_info();

  ad_info->set_cruise_speed(
      session_.environmental_model().get_ego_state_manager()->ego_v_cruise());

  // HMI for NOA
  auto virtual_lane_manager =
      session_.environmental_model().get_virtual_lane_manager();
  ad_info->set_distance_to_ramp(virtual_lane_manager->dis_to_ramp());
  ad_info->set_distance_to_split(
      virtual_lane_manager->distance_to_first_road_split());
  ad_info->set_distance_to_merge(
      virtual_lane_manager->distance_to_first_road_merge());
  ad_info->set_distance_to_toll_station(
      (uint)virtual_lane_manager
          ->ramp_direction());  // 临时将toll_station改为ramp_direction
}

void GeneralPlanning::ClearParkingInfo(
    PlanningOutput::PlanningOutput *const planning_output) {
  session_.planning_output_context()
      .planning_status()
      .planning_result.planning_output.mutable_planning_status()
      ->set_apa_planning_status(PlanningOutput::ApaPlanningStatus::NONE);
  session_.planning_output_context()
      .planning_status()
      .planning_result.planning_output.mutable_successful_slot_info_list()
      ->Clear();
  planning_output->mutable_planning_status()->set_apa_planning_status(
      PlanningOutput::ApaPlanningStatus::NONE);
}

bool GeneralPlanning::IsUndefinedScene(
    const ::FuncStateMachine::FunctionalState &current_state) {
  return current_state == FunctionalState::INIT ||
         current_state == FunctionalState::STANDBY ||
         current_state == FunctionalState::ERROR;
}

}  // namespace planning