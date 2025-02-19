#include "planning_scheduler.h"

#include <common/config/basic_type.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <memory>

#include "adas_function/adaptive_cruise_control.h"
#include "adas_function/ihc_function/intelligent_headlight_control.h"
#include "adas_function/lkas_function/lane_keep_assist_manager.h"
#include "adas_function/mrc_condition.h"
#include "adas_function/start_stop_enable.h"
#include "adas_function/tsr_function/traffic_sign_recognition.h"
#include "apa_function/util/apa_utils.h"
#include "basic_types.pb.h"
#include "common/config_context.h"
#include "config/basic_type.h"
#include "config/vehicle_param.h"
#include "debug_info_log.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "func_state_machine_c.h"
#include "ifly_time.h"
#include "log.h"
#include "log_glog.h"
#include "math/math_utils.h"
#include "planning_context.h"
#include "planning_debug_info.pb.h"
#include "planning_hmi_c.h"
#include "scene_type_config.pb.h"
#include "struct_container.hpp"
#include "utils/file.h"
#include "utils/lateral_utils.h"
#include "vehicle_config_context.h"
#include "vehicle_status.pb.h"
#include "virtual_lane.h"

namespace planning {

PlanningScheduler::PlanningScheduler(
    const LocalView *const local_view,
    const common::EngineConfiguration *const engine_config_ptr)
    : local_view_(local_view) {
  Init(engine_config_ptr);
}

PlanningScheduler::~PlanningScheduler() {}

void PlanningScheduler::Init(
    const common::EngineConfiguration *const engine_config_ptr) {
  session_.Init();
  environmental_model_manager_.Init(&session_);
  EnvironmentalModel *environmental_model =
      session_.mutable_environmental_model();
  environmental_model->feed_local_view(local_view_);
  // TODO: 车辆配置文件从文件读取

  VehicleConfigurationContext::Instance()->set_vehicle_param(
      engine_config_ptr->vehicle_cfg_dir);
  std::cout
      << "load vehicle param success! car type: "
      << VehicleConfigurationContext::Instance()->get_vehicle_param().car_type
      << ", the car wheel base is: "
      << VehicleConfigurationContext::Instance()->get_vehicle_param().wheel_base
      << std::endl;

  // TODO：配置文件改成和场景/功能有关，不能使用默认场景
  planning::common::SceneType scene_type = session_.get_scene_type();
  auto config_builder =
      session_.environmental_model().config_builder(scene_type);
  config_ = config_builder->cast<GeneralPlanningConfig>();

  // TODO：移到Scc_Function
  InitSccFunction();

  hpp_function_ = std::make_unique<HppFunction>(&session_);
  noa_function_ = std::make_unique<NoaFunction>(&session_);
  scc_function_ = std::make_unique<SccFunction>(&session_);
  apa_function_ = std::make_unique<ApaFunction>(&session_);
}

void PlanningScheduler::SyncParameters(planning::common::SceneType scene_type) {
  std::string path;
  auto engine_config =
      common::ConfigurationContext::Instance()->engine_config();
  switch (scene_type) {
    case planning::common::SceneType::HIGHWAY:
      path =
          engine_config.module_cfg_dir + "/general_planner_module_highway.json";
      break;
    case planning::common::SceneType::PARKING_APA:
      path =
          engine_config.module_cfg_dir + "/general_planner_module_parking.json";
      break;
    case planning::common::SceneType::HPP:
      path = engine_config.module_cfg_dir + "/general_planner_module_hpp.json";
      break;
    default:
      path =
          engine_config.module_cfg_dir + "/general_planner_module_highway.json";
  }

  std::string config_file = common::util::ReadFile(path);
  auto config = mjson::Reader(config_file);

  auto config_builder =
      session_.environmental_model().config_builder(scene_type);
  config_ = config_builder->cast<GeneralPlanningConfig>();
  environmental_model_manager_.SetConfig(scene_type);
  // all parameters can be changed here
  JSON_READ_VALUE(GENERAL_PLANNING_CONTEXT.MutablePram().planner_type, int,
                  "planner_type");
}

planning::common::SceneType PlanningScheduler::DetermineSceneType(
    const iflyauto::FuncStateMachine &func_state_machine) {
  auto scene_type = planning::common::SceneType::HIGHWAY;

  if (IsUndefinedScene(func_state_machine.current_state)) {
    scene_type = planning::common::SceneType::HIGHWAY;
  } else if (IsSwitchApaState(func_state_machine.current_state)) {
    scene_type = planning::common::SceneType::PARKING_APA;
  } else if (IsValidHppState(func_state_machine.current_state)) {
    scene_type = planning::common::SceneType::HPP;
  } else {
    scene_type = planning::common::SceneType::HIGHWAY;
  }
  session_.set_scene_type(scene_type);

  auto frame_info =
      DebugInfoManager::GetInstance().GetDebugInfoPb()->mutable_frame_info();
  frame_info->set_scene_type(common::SceneType_Name(scene_type));

  ILOG_INFO << "scene_type " << scene_type;
  return scene_type;
}

bool PlanningScheduler::RunOnce(
    iflyauto::PlanningOutput *const planning_output,
    iflyauto::PlanningHMIOutputInfoStr *const planning_hmi_info) {
  LOG_ERROR("PlanningScheduler::RunOnce \n");
  auto &planning_result =
      session_.mutable_planning_context()->mutable_planning_result();
  const double start_timestamp = IflyTime::Now_ms();
  bool planning_success = false;
  session_.mutable_planning_context()->Clear();
  session_.mutable_planning_context()->feed_planning_hmi_info(
      planning_hmi_info);

  const auto &state_machine = local_view_->function_state_machine_info;
  auto function_type = DetermineSceneType(state_machine);
  planning_result.scene_type = function_type;
  planning_result.timestamp = start_timestamp;
  planning_output->successful_slot_info_list_size = 0;

  // reset
  if (function_type == common::PARKING_APA || function_type == common::HPP) {
    const auto &state_machine = local_view_->function_state_machine_info;
    if (GENERAL_PLANNING_CONTEXT.GetStatemachine().apa_reset_flag &&
        state_machine.current_state !=
            iflyauto::FunctionalState_PARK_GUIDANCE) {
      apa_function_->Reset();
      ILOG_INFO << "reset parking";

      ResetGLogFile();
    }
  }

  bool is_hpp_slot_searching = IsHppSlotSearchingByDistance();
  if (function_type == common::PARKING_APA || is_hpp_slot_searching) {
    planning_success = ExcuteParkingFunction(function_type, planning_output);
  }

  if (function_type == common::HIGHWAY || function_type == common::HPP) {
    planning_success = ExcuteNavigationFunction(
        function_type, start_timestamp, planning_output, planning_hmi_info);
  }

  int64_t frame_duration = IflyTime::Now_ms() - start_timestamp;
  LOG_DEBUG("The time cost of RunOnce is: %d\n", (int)frame_duration);

  return planning_success;
}

uint64_t PlanningScheduler::FaultCode() {
  return environmental_model_manager_.getFaultcode();
}
void PlanningScheduler::FillPlanningTrajectory(
    double start_time, iflyauto::PlanningOutput *const planning_output) {
  // 获取LDP&&ELK功能干预状态
  auto lkas_info =
      session_.mutable_planning_context()->lane_keep_assit_function();
  bool lkas_intervention_flag;
  if ((lkas_info->get_ldp_left_intervention_flag_info() == true) ||
      (lkas_info->get_ldp_right_intervention_flag_info() == true) ||
      (lkas_info->get_elk_left_intervention_flag_info() == true) ||
      (lkas_info->get_elk_right_intervention_flag_info() == true)) {
    lkas_intervention_flag = true;
  } else {
    lkas_intervention_flag = false;
  }

  // 获取计算结果
  const auto &lateral_output =
      session_.planning_context().lateral_behavior_planner_output();
  const auto &vision_only_longitudinal_outputs =
      session_.planning_context().vision_longitudinal_behavior_planner_output();
  const auto &planning_context = session_.planning_context();
  const auto &planning_result = planning_context.planning_result();
  const auto &ego_state =
      session_.environmental_model().get_ego_state_manager();
  const auto &function_info = session_.environmental_model().function_info();
  const bool active = session_.environmental_model().GetVehicleDbwStatus();
  auto virtual_lane_manager =
      session_.environmental_model().get_virtual_lane_manager();

  // 更新输出
  iflyauto::strcpy_array(planning_output->meta.plan_strategy_name,
                         "Real Time Planning");

  // 2.Trajectory
  auto trajectory = &planning_output->trajectory;
  trajectory->trajectory_points_size = 0;
  // set trajectory false when dbw is false
  if (lkas_intervention_flag) {
    trajectory->available = true;
  } else {
    trajectory->available = active;
  }

  // 根据定位有效性决定实时、长时
  auto location_valid = session_.environmental_model().location_valid();

  if (location_valid) {
    trajectory->trajectory_type = iflyauto::TRAJECTORY_TYPE_TRAJECTORY_POINTS;
    for (size_t i = 0; i < planning_result.traj_points.size(); i++) {
      auto path_point = &trajectory->trajectory_points[i];
      path_point->x = planning_result.traj_points[i].x;
      path_point->y = planning_result.traj_points[i].y;
      path_point->heading_yaw = planning_result.traj_points[i].heading_angle;
      path_point->curvature = planning_result.traj_points[i].curvature;
      path_point->t = planning_result.traj_points[i].t;
      path_point->v = planning_result.traj_points[i].v;
      path_point->a = planning_result.traj_points[i].a;
      path_point->distance = planning_result.traj_points[i].s;
      path_point->jerk = planning_result.traj_points[i].jerk;
      ++(trajectory->trajectory_points_size);
    }
    // 设置参考线为default
    auto target_ref = &trajectory->target_reference;
    // add polynomial
    const auto &d_polynomial = lateral_output.d_poly;
    for (size_t i = 0; i < d_polynomial.size(); i++) {
      target_ref->polynomial[i] = d_polynomial[i];
    }
    target_ref->target_velocity =
        vision_only_longitudinal_outputs.velocity_target;
    auto acceleration_range_limit = &target_ref->acceleration_range_limit;
    acceleration_range_limit->min_a = -4.0;
    acceleration_range_limit->max_a = 4.0;

    target_ref->lateral_maneuver_gear = iflyauto::LATERAL_MANEUVER_GEAR_NORMAL;
  } else {
    // set vision_only_longitudinal_outputs if hdmpa valid is false
    trajectory->trajectory_type = iflyauto::TRAJECTORY_TYPE_TARGET_REFERENCE;
    // trajectory->mutable_trajectory_points()->Clear();
    // trajectory->mutable_target_reference()->Clear();
    // 设置轨迹为default
    auto path_point = &trajectory->trajectory_points[0];
    path_point->x = 0.0;
    path_point->y = 0.0;
    path_point->heading_yaw = 0.0;
    path_point->curvature = 0.0;
    path_point->t = 0.0;
    path_point->v = 0.0;
    path_point->a = 0.0;
    path_point->distance = 0.0;
    path_point->jerk = 0.0;
    ++(trajectory->trajectory_points_size);

    auto target_ref = &trajectory->target_reference;
    // add polynomial
    // clip the polynomial C_3
    const double lat_offset_rate = config_.d_poly_lat_offset_rate;
    const double max_lat_offset = config_.d_poly_max_lat_offset;
    const auto &d_polynomial = lateral_output.d_poly;
    double lat_offset_bound = max_lat_offset;
    static double limited_polynomial_3 = 0.0;

    if (d_polynomial.size() == 4) {
      if (lateral_output.lc_status == "left_lane_change" ||
          lateral_output.lc_status == "right_lane_change") {
        lat_offset_bound = ComputeBoundOfReferenceIntercept();

        if (std::fabs(d_polynomial[3]) > lat_offset_bound) {
          limited_polynomial_3 += planning_math::Clamp(
              d_polynomial[3], -lat_offset_rate, lat_offset_rate);
        } else {
          limited_polynomial_3 = planning_math::Clamp(
              d_polynomial[3], -lat_offset_bound, lat_offset_bound);
        }
      } else if ((lateral_output.lc_status == "left_lane_change_back" ||
                  lateral_output.lc_status == "right_lane_change_back") &&
                 std::fabs(d_polynomial[3]) >
                     config_.lc_back_consider_smooth_dpoly_thr) {
        limited_polynomial_3 =
            planning_math::Clamp(d_polynomial[3], -config_.lc_back_smooth_thr,
                                 config_.lc_back_smooth_thr);
      } else if (lateral_output.lc_status == "left_lane_change_wait" ||
                 lateral_output.lc_status == "right_lane_change_wait") {
        limited_polynomial_3 = 0.0;
      } else {
        limited_polynomial_3 = planning_math::Clamp(
            d_polynomial[3], -lat_offset_bound, lat_offset_bound);
      }
      limited_polynomial_3 = planning_math::Clamp(
          limited_polynomial_3, -lat_offset_bound, lat_offset_bound);
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

    const auto &current_lane = session_.environmental_model()
                                   .get_virtual_lane_manager()
                                   ->get_current_lane();
    if ((lkas_intervention_flag == true) && (current_lane != nullptr)) {
      polynomial_limited[0] = current_lane->get_center_line()[3];
      polynomial_limited[1] = current_lane->get_center_line()[2];
      polynomial_limited[2] = current_lane->get_center_line()[1];
      polynomial_limited[3] = current_lane->get_center_line()[0];
    } else {
      polynomial_limited[0] = d_polynomial[0];
      polynomial_limited[1] = d_polynomial[1];
      polynomial_limited[2] = d_polynomial[2];
      polynomial_limited[3] = limited_polynomial_3;
    }
    LOG_DEBUG("limited_polynomial C0: [%f] C1: [%f] C2: [%f] C3: [%f] \n",
              polynomial_limited[0], polynomial_limited[1],
              polynomial_limited[2], polynomial_limited[3]);
    for (size_t i = 0; i < polynomial_limited.size(); i++) {
      target_ref->polynomial[i] = polynomial_limited[i];
    }

    target_ref->target_velocity =
        vision_only_longitudinal_outputs.velocity_target;

    auto acceleration_range_limit = &(target_ref->acceleration_range_limit);
    acceleration_range_limit->min_a =
        vision_only_longitudinal_outputs.a_target_min;
    acceleration_range_limit->max_a =
        vision_only_longitudinal_outputs.a_target_max;
    target_ref->lateral_maneuver_gear = iflyauto::LATERAL_MANEUVER_GEAR_NORMAL;
  }
  // 3.Turn signal
  auto turn_signal = &(planning_output->turn_signal_command);
  turn_signal->available = true;
  if (planning_result.turn_signal == NO_CHANGE) {
    turn_signal->turn_signal_value = iflyauto::TURN_SIGNAL_TYPE_NONE;
  } else if (planning_result.turn_signal == LEFT_CHANGE) {
    turn_signal->turn_signal_value = iflyauto::TURN_SIGNAL_TYPE_LEFT;
  } else {
    turn_signal->turn_signal_value = iflyauto::TURN_SIGNAL_TYPE_RIGHT;
  }
  // WB start:--------临时hack以下信号--------
  // 4.Light signal
  auto light_signal = &(planning_output->light_signal_command);
  light_signal->available = true;
  light_signal->light_signal_value = iflyauto::LIGHT_SIGNAL_TYPE_NONE;

  // 5.Horn signal
  auto horn_signal_command = &(planning_output->horn_signal_command);
  horn_signal_command->available = true;
  horn_signal_command->horn_signal_value = iflyauto::HORN_SIGNAL_TYPE_NONE;

  // 6.Gear signal
  auto gear_command = &(planning_output->gear_command);
  gear_command->available = true;
  // 需要获取目标挡位值
  gear_command->gear_command_value = iflyauto::GEAR_COMMAND_VALUE_DRIVE;

  // 7.Open loop steering command
  auto open_loop_steering_command =
      &(planning_output->open_loop_steering_command);
  open_loop_steering_command->available = true;
  open_loop_steering_command->jerk_factor = 70.0;  // hack
  open_loop_steering_command->need_steering_wheel_stationary = false;
  open_loop_steering_command->steering_wheel_rad_limit = 0.1;

  // 8.Planning status
  auto planning_status = &(planning_output->planning_status);
  planning_status->standstill = false;
  if (function_info.function_mode() == common::DrivingFunctionInfo::ACC ||
      function_info.function_mode() == common::DrivingFunctionInfo::SCC) {
    planning_status->standstill = std::fabs(ego_state->ego_v()) < 0.1;
  }
  // 启停状态机
  planning_status->ready_to_go = planning_context.start_stop_result().state() !=
                                 common::StartStopInfo::STOP;
  planning_status->apa_planning_status = iflyauto::APA_NONE;
  // WB end:--------临时hack以上信号--------
  const bool planning_success = planning_context.planning_success();
  const bool planning_completed = planning_context.planning_completed();
  if (planning_completed) {
    planning_status->hpp_planning_status = iflyauto::HPP_COMPLETED;
  } else if (planning_success) {
    planning_status->hpp_planning_status = iflyauto::HPP_RUNNING;
  } else {
    planning_status->hpp_planning_status = iflyauto::HPP_RUNNING_FAILED;
  }
}

void PlanningScheduler::GenerateStopTrajectory(
    double start_time, iflyauto::PlanningOutput *const planning_output) {
  // 更新输出
  // planning_output->msg_header.stamp = IflyTime::Now_ms();

  auto trajectory = &(planning_output->trajectory);
  // Hack: 长时规划
  trajectory->trajectory_type = iflyauto::TRAJECTORY_TYPE_TRAJECTORY_POINTS;
  trajectory->trajectory_points_size = 0;
  double t = 0.0;
  for (size_t i = 0; i < 21; i++) {
    t = 0.1 * i;
    auto path_point =
        &(trajectory->trajectory_points[trajectory->trajectory_points_size++]);
    path_point->x = 0.0;
    path_point->y = 0.0;
    path_point->heading_yaw = 0.0;
    path_point->curvature = 0.0;
    path_point->t = t;
    path_point->v = 0.0;
    path_point->a = 0.0;
    path_point->distance = 0.0;
    path_point->jerk = 0.0;  // TBD
  }
}

void PlanningScheduler::FillPlanningHmiInfo(
    double start_timestamp,
    iflyauto::PlanningHMIOutputInfoStr *const planning_hmi_info) {
  const auto &lateral_output =
      session_.planning_context().lateral_behavior_planner_output();
  const auto &lane_change_decider_output =
      session_.planning_context().lane_change_decider_output();
  const auto &lat_offset_decider_output =
      session_.planning_context().lateral_offset_decider_output();

  planning_hmi_info->msg_header.stamp = IflyTime::Now_us();
  // HMI for ldw
  const auto &lkas_info =
      session_.mutable_planning_context()->lane_keep_assit_function();
  planning_hmi_info->ldw_output_info.ldw_state =
      lkas_info->get_ldw_state_info();
  planning_hmi_info->ldw_output_info.ldw_left_warning =
      lkas_info->get_ldw_left_warning_info();
  planning_hmi_info->ldw_output_info.ldw_right_warning =
      lkas_info->get_ldw_right_warning_info();
  // HMI for ldp
  planning_hmi_info->ldp_output_info.ldp_state =
      lkas_info->get_ldp_state_info();
  planning_hmi_info->ldp_output_info.ldp_left_intervention_flag =
      lkas_info->get_ldp_left_intervention_flag_info();
  planning_hmi_info->ldp_output_info.ldp_right_intervention_flag =
      lkas_info->get_ldp_right_intervention_flag_info();
  // HMI for elk
  planning_hmi_info->elk_output_info.elk_state =
      lkas_info->get_elk_state_info();
  planning_hmi_info->elk_output_info.elk_left_intervention_flag =
      lkas_info->get_elk_left_intervention_flag_info();
  planning_hmi_info->elk_output_info.elk_right_intervention_flag =
      lkas_info->get_elk_right_intervention_flag_info();
  // HMI for tsr
  const auto &tsr_info =
      session_.mutable_planning_context()->traffic_sign_recognition_function();
  planning_hmi_info->tsr_output_info.tsr_state = tsr_info->get_tsr_state_info();
  planning_hmi_info->tsr_output_info.tsr_warning =
      tsr_info->get_tsr_warning_info();
  planning_hmi_info->tsr_output_info.tsr_speed_limit =
      tsr_info->get_tsr_speed_limit_info();
  // HMI for ihc
  const auto &ihc_info = session_.mutable_planning_context()
                             ->intelligent_headlight_control_function();
  planning_hmi_info->ihc_output_info.ihc_state = ihc_info->get_ihc_state_info();
  planning_hmi_info->ihc_output_info.ihc_request =
      ihc_info->get_ihc_request_info();
  planning_hmi_info->ihc_output_info.ihc_request_status =
      ihc_info->get_ihc_request_status_info();

  // HMI for alc
  auto alc_output_pb = &(planning_hmi_info->alc_output_info);
  iflyauto::strcpy_array(alc_output_pb->lc_request,
                         lateral_output.lc_request.c_str());

  iflyauto::strcpy_array(alc_output_pb->lc_status,
                         lateral_output.lc_status.c_str());

  iflyauto::strcpy_array(alc_output_pb->lc_invalid_reason,
                         lane_change_decider_output.lc_invalid_reason.c_str());
  iflyauto::strcpy_array(
      alc_output_pb->lc_back_reason,
      lane_change_decider_output.lc_back_invalid_reason.c_str());

  // HMI for CIPV
  // TBD: 后续需要丰富障碍物的信息，后车、侧方车辆等
  const auto &cipv_info =
      session_.planning_context().planning_hmi_info().cipv_info;
  planning_hmi_info->cipv_info.has_cipv = cipv_info.has_cipv;
  planning_hmi_info->cipv_info.cipv_id = cipv_info.cipv_id;

  // HMI for ad_info
  const auto &ad_info =
      session_.planning_context().planning_hmi_info().ad_info;
  planning_hmi_info->ad_info.cruise_speed = ad_info.cruise_speed;
  planning_hmi_info->ad_info.lane_change_direction =
      ad_info.lane_change_direction;
  planning_hmi_info->ad_info.lane_change_status = ad_info.lane_change_status;
  planning_hmi_info->ad_info.status_update_reason =
      ad_info.status_update_reason;
  planning_hmi_info->ad_info.obstacle_info[0] = ad_info.obstacle_info[0];
  planning_hmi_info->ad_info.lane_change_reason = ad_info.lane_change_reason;

  planning_hmi_info->ad_info.distance_to_ramp = ad_info.distance_to_ramp;
  planning_hmi_info->ad_info.distance_to_split = ad_info.distance_to_split;
  planning_hmi_info->ad_info.distance_to_merge = ad_info.distance_to_merge;
  planning_hmi_info->ad_info.distance_to_toll_station =
      ad_info.distance_to_toll_station;
  planning_hmi_info->ad_info.noa_exit_warning_level_distance =
      ad_info.noa_exit_warning_level_distance;
  // planning_hmi_info->ad_info.distance_to_tunnel = ;
  // planning_hmi_info->ad_info.is_within_hdmap = ;
  planning_hmi_info->ad_info.ramp_direction = ad_info.ramp_direction;
  planning_hmi_info->ad_info.dis_to_reference_line =
      ad_info.dis_to_reference_line;
  planning_hmi_info->ad_info.angle_to_roaddirection =
      ad_info.angle_to_roaddirection;
  planning_hmi_info->ad_info.is_in_sdmaproad = ad_info.is_in_sdmaproad;
  planning_hmi_info->ad_info.road_type = ad_info.road_type;
  planning_hmi_info->ad_info.ramp_pass_sts = ad_info.ramp_pass_sts;
  planning_hmi_info->ad_info.landing_point = ad_info.landing_point;

  planning_hmi_info->ad_info.avoid_status =
      lat_offset_decider_output.avoid_id > 0
          ? iflyauto::AvoidObstacle::AVOID_HIDING
          : iflyauto::AvoidObstacle::AVOID_NO_HIDING;
  planning_hmi_info->ad_info.aovid_id =
      lat_offset_decider_output.avoid_id;
  planning_hmi_info->ad_info.avoiddirect =
      static_cast<iflyauto::AvoidObstacleDirection>(
          lat_offset_decider_output.avoid_direction);

  // HMI for hpp
  const auto &ego_state_manager =
      session_.environmental_model().get_ego_state_manager();
  const auto &route_info_output =
      session_.environmental_model().get_route_info()->get_route_info_output();
  auto hpp_info = &(session_.mutable_planning_context()
                        ->mutable_planning_hmi_info()
                        ->hpp_info);
  hpp_info->is_avaliable = route_info_output.is_on_hpp_lane;
  hpp_info->distance_to_parking_space =
      route_info_output.distance_to_target_slot;
  hpp_info->is_on_hpp_lane = route_info_output.is_on_hpp_lane;
  // hpp_info->is_on_hpp_lane = true;  // hack
  hpp_info->is_reached_hpp_trace_start =
      route_info_output.is_reached_hpp_start_point;
  hpp_info->accumulated_driving_distance =
      route_info_output.sum_distance_driving;

  hpp_info->is_approaching_intersection = false;
  hpp_info->is_approaching_turn = false;
  hpp_info->is_parking_space_occupied = false;
  hpp_info->is_new_parking_space_found = false;
  hpp_info->hpp_state_switch = iflyauto::HPPStateSwitch::HPP_NONE;
  auto reference_path_manager =
      session_.environmental_model().get_reference_path_manager();
  auto current_reference_path =
      reference_path_manager->get_reference_path_by_current_lane();
  const double kCheckTurnDistance = 15.0;
  const double kEgoIsOnTurnDistance1 = -3.0;
  const double kEgoIsOnTurnDistance2 = 5.0;
  auto &frenet_ego_state = current_reference_path->get_frenet_ego_state();
  auto &points = current_reference_path->get_points();
  double ego_s = frenet_ego_state.s();
  if (current_reference_path != nullptr) {
    for (auto &point : points) {
      double distance = point.path_point.s() - ego_s;
      if (distance > kEgoIsOnTurnDistance1 &&
          distance < kEgoIsOnTurnDistance2 &&
          point.path_point.kappa() > 0.08) {  // ego is on the curve
        break;
      }
      if (distance > kEgoIsOnTurnDistance2 && distance <= kCheckTurnDistance) {
        if (point.path_point.kappa() >
            0.1) {  // 关注实际曲率的连续性，考虑多点还是单点
          // hpp_info->set_is_approaching_intersection(true);
          hpp_info->is_approaching_turn = true;
          break;
        }
      } else if (distance > kCheckTurnDistance) {
        break;
      }
    }
  }
  if (route_info_output.distance_to_target_slot < 10.0) {
    hpp_info->distance_to_parking_space =
        std::min(std::fabs(points.back().path_point.s() - ego_s),
                 route_info_output.distance_to_target_slot);
  }
  // hpp状态切park_in状态
  if (session_.is_hpp_scene()) {
    const auto &parking_switch_info = session_.planning_context()
                                          .parking_switch_decider_output()
                                          .parking_switch_info;
    if (parking_switch_info.is_memory_slot_allowed_to_park) {
      hpp_info->hpp_state_switch =
          iflyauto::HPPStateSwitch::HPP_CRUISING_TO_PARKING;
    } else if (parking_switch_info.is_memory_slot_occupied) {
      hpp_info->is_parking_space_occupied = true;
    } else if (parking_switch_info.is_selected_slot_allowed_to_park) {
      hpp_info->hpp_state_switch =
          iflyauto::HPPStateSwitch::HPP_CRUISING_TO_PARKING;
    }

    // todo: is_new_parking_space_found is unused.
    if (parking_switch_info.has_parking_slot_in_hpp_searching) {
      hpp_info->is_new_parking_space_found = true;
    }
  }

  return;
}

void PlanningScheduler::FillPlanningRequest(
    iflyauto::RequestLevel request,
    iflyauto::PlanningOutput *const planning_output) {
  planning_output->planning_request.take_over_req_level = request;
  planning_output->planning_request.request_reason =
      iflyauto::REQUEST_REASON_NO_REASON;
}

void PlanningScheduler::ClearParkingInfo(
    iflyauto::PlanningOutput *const planning_output) {
  session_.mutable_planning_context()
      ->mutable_planning_output()
      .planning_status.apa_planning_status = iflyauto::APA_NONE;

  session_.mutable_planning_context()
      ->mutable_planning_output()
      .successful_slot_info_list_size = 0;

  planning_output->planning_status.apa_planning_status = iflyauto::APA_NONE;
}

bool PlanningScheduler::IsUndefinedScene(
    const iflyauto::FunctionalState &current_state) {
  return current_state == iflyauto::FunctionalState_MANUAL ||
         current_state == iflyauto::FunctionalState_ERROR ||
         current_state == iflyauto::FunctionalState_MRC;
}

bool PlanningScheduler::IsValidHppState(
    const iflyauto::FunctionalState &current_state) {
  return current_state >= iflyauto::FunctionalState_HPP_STANDBY &&
         current_state <= iflyauto::FunctionalState_HPP_ERROR;
}

void PlanningScheduler::InitSccFunction() {
  // TODO：配置文件改成和场景/功能有关，不能使用默认场景
  planning::common::SceneType scene_type = session_.get_scene_type();
  auto config_builder =
      session_.environmental_model().config_builder(scene_type);

  // SCC Function
  auto adaptive_cruise_control =
      std::make_shared<AdaptiveCruiseControl>(config_builder, &session_);
  session_.mutable_planning_context()->set_adaptive_cruise_control_function(
      adaptive_cruise_control);

  auto start_stop =
      std::make_shared<StartStopEnable>(config_builder, &session_);
  session_.mutable_planning_context()->set_start_stop_enable(start_stop);

  auto mrc_condition =
      std::make_shared<MrcCondition>(config_builder, &session_);
  session_.mutable_planning_context()->set_mrc_condition(mrc_condition);

  auto lane_keep_assit = std::make_shared<LaneKeepAssistManager>(&session_);
  session_.mutable_planning_context()->set_lane_keep_assit_function(
      lane_keep_assit);

  auto intelligent_headlight_control =
      std::make_shared<IntelligentHeadlightControl>(&session_);
  session_.mutable_planning_context()
      ->set_intelligent_headlight_control_function(
          intelligent_headlight_control);

  auto traffic_sign_recognition =
      std::make_shared<TrafficSignRecognition>(&session_);
  session_.mutable_planning_context()->set_traffic_sign_recognition_function(
      traffic_sign_recognition);
}

void PlanningScheduler::interpolate_with_last_trajectory_points() {
  const auto &last_planning_result =
      session_.planning_context().last_planning_result();

  // interpolate traj points
  // todo @xbliu config

  auto &planning_result =
      session_.mutable_planning_context()->mutable_planning_result();
  auto start_time =
      (planning_result.timestamp - last_planning_result.timestamp) / 1000.0;
  assert(start_time >= 0);
  planning_result.traj_points.clear();
  auto backup_num_points = 201;
  auto delta_time = 0.025;
  auto &last_traj_points = last_planning_result.traj_points;
  assert(last_traj_points.size() >= 2);

  size_t idx = 0;
  for (int j = 0; j < backup_num_points; ++j) {
    TrajectoryPoint traj_pt;
    auto t = j * delta_time;
    auto interpolate_t = t + start_time;
    for (; idx < last_traj_points.size() - 1; idx++) {
      if (last_traj_points[idx].t <= interpolate_t &&
          interpolate_t <= last_traj_points[idx + 1].t) {
        break;
      }
    }

    auto pre_idx = std::min(idx, last_traj_points.size() - 2);
    auto &pre_pt = last_traj_points[pre_idx];
    auto &next_pt = last_traj_points[pre_idx + 1];

    traj_pt.t = t;
    traj_pt.x = planning_math::Interpolate(pre_pt.t, pre_pt.x, next_pt.t,
                                           next_pt.x, interpolate_t);
    traj_pt.y = planning_math::Interpolate(pre_pt.t, pre_pt.y, next_pt.t,
                                           next_pt.y, interpolate_t);
    traj_pt.heading_angle = planning_math::InterpolateAngle(
        pre_pt.t, pre_pt.heading_angle, next_pt.t, next_pt.heading_angle,
        interpolate_t);
    traj_pt.curvature =
        planning_math::Interpolate(pre_pt.t, pre_pt.curvature, next_pt.t,
                                   next_pt.curvature, interpolate_t);
    traj_pt.v = planning_math::Interpolate(pre_pt.t, pre_pt.v, next_pt.t,
                                           next_pt.v, interpolate_t);
    traj_pt.a = planning_math::Interpolate(pre_pt.t, pre_pt.a, next_pt.t,
                                           next_pt.a, interpolate_t);
    traj_pt.s = planning_math::Interpolate(pre_pt.t, pre_pt.s, next_pt.t,
                                           next_pt.s, interpolate_t);
    traj_pt.l = planning_math::Interpolate(pre_pt.t, pre_pt.l, next_pt.t,
                                           next_pt.l, interpolate_t);
    traj_pt.frenet_valid = pre_pt.frenet_valid && next_pt.frenet_valid;
    planning_result.traj_points.emplace_back(traj_pt);
  }
  planning_result.raw_traj_points.clear();
}

bool PlanningScheduler::UpdateFailedPlanningResult() {
  const auto &coarse_planning_info = session_.planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  const auto &last_planning_result =
      session_.planning_context().last_planning_result();
  auto &planning_result =
      session_.mutable_planning_context()->mutable_planning_result();
  if (last_planning_result.scene_type != planning_result.scene_type) {
    return false;
  }

  if (last_planning_result.target_lane_id ==
          coarse_planning_info.target_lane_id &&
      last_planning_result.use_backup_cnt <= config_.failure_counter_thrshld) {
    auto delta_time =
        (planning_result.timestamp - last_planning_result.timestamp) / 1000.0;
    if (0.0 < delta_time && delta_time < 1.0) {
      interpolate_with_last_trajectory_points();
      planning_result.use_backup_cnt = last_planning_result.use_backup_cnt + 1;
      return true;
    }
  }

  return false;
}

bool PlanningScheduler::UpdateSuccessfulPlanningResult() {
  const auto &coarse_planning_info = session_.planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  auto &planning_result =
      session_.mutable_planning_context()->mutable_planning_result();
  planning_result.use_backup_cnt = 0;
  planning_result.target_lane_id = coarse_planning_info.target_lane_id;
  session_.mutable_planning_context()->mutable_last_planning_result() =
      planning_result;

  return true;
}

double PlanningScheduler::ComputeBoundOfReferenceIntercept() {
  int origin_lane_virtual_id = session_.planning_context()
                                   .lane_change_decider_output()
                                   .origin_lane_virtual_id;
  int target_lane_virtual_id = session_.planning_context()
                                   .lane_change_decider_output()
                                   .target_lane_virtual_id;
  auto target_reference =
      session_.environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(target_lane_virtual_id, false);
  auto origin_reference =
      session_.environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(origin_lane_virtual_id, false);
  const double max_lat_offset = config_.d_poly_max_lat_offset;
  const double min_lat_offset = config_.d_poly_min_lat_offset;
  double intercept_cart_point = 0.0;
  double intercept_presee_cart_point = 0.0;
  double presee_dist = 25;
  double reference_intercept_bound = max_lat_offset;
  LOG_DEBUG("start compute bound of reference intercept!! \n");

  if (origin_reference != nullptr && target_reference != nullptr) {
    Point2D frenet_of_cart_point_in_target;
    Point2D cart_point_in_target;
    Point2D presee_cart_point_in_target;
    Point2D frenet_of_presee_cart_point_in_target;

    if (target_reference->get_frenet_coord()->SLToXY(
            Point2D(target_reference->get_frenet_ego_state().s(), 0),
            cart_point_in_target)) {
      if (origin_reference->get_frenet_coord()->XYToSL(
              cart_point_in_target, frenet_of_cart_point_in_target)) {
        intercept_cart_point = frenet_of_cart_point_in_target.y;
      }
    }

    double s_end =
        std::min(target_reference->get_frenet_coord()->Length(),
            target_reference->get_frenet_ego_state().s() + presee_dist);
    if (target_reference->get_frenet_coord()->SLToXY(
            Point2D(s_end, 0), presee_cart_point_in_target)) {
      if (origin_reference->get_frenet_coord()->XYToSL(
              presee_cart_point_in_target,
              frenet_of_presee_cart_point_in_target)) {
        intercept_presee_cart_point = frenet_of_presee_cart_point_in_target.y;
      }
    }
  }
  std::cout << "intercept_presee_cart_point: " << intercept_presee_cart_point
            << "intercept_cart_point: " << intercept_cart_point << std::endl;

  if (std::fabs(intercept_presee_cart_point) >
      std::fabs(intercept_cart_point)) {
    reference_intercept_bound =
        max_lat_offset -
        std::fabs(intercept_presee_cart_point - intercept_cart_point);
    reference_intercept_bound = planning_math::Clamp(
        reference_intercept_bound, min_lat_offset, max_lat_offset);
    LOG_DEBUG("lat_offset_bound: [%f]: \n", reference_intercept_bound);
  }

  return reference_intercept_bound;
}

bool PlanningScheduler::IsHppSlotSearchingByDistance() {
  // check state
  const auto &state_machine = local_view_->function_state_machine_info;
  if (!IsHppSlotSearchingStage(state_machine.current_state)) {
    return false;
  }

  // check dist
  if (state_machine.current_state ==
      iflyauto::FunctionalState_HPP_CRUISE_ROUTING) {
    double dist = session_.environmental_model()
                      .get_parking_slot_manager()
                      ->GetDistanceToTargetSlot();
    const double kdistance_thresh = 10.0;
    if (dist > kdistance_thresh) {
      return false;
    }
  }

  //  check speed
  const auto &ego_state =
      session_.environmental_model().get_ego_state_manager();
  const double kspeed_thresh = 5.0;
  if (ego_state->ego_v() > kspeed_thresh) {
    return false;
  }

  return true;
}

const bool PlanningScheduler::ExcuteParkingFunction(
    const common::SceneType function_type,
    iflyauto::PlanningOutput *const planning_output) {
  // 泊车规划部分
  bool planning_success = apa_function_->Plan();

  *planning_output = session_.planning_context().planning_output();

  return planning_success;
}

const bool PlanningScheduler::ExcuteNavigationFunction(
    const common::SceneType function_type, const double start_timestamp,
    iflyauto::PlanningOutput *const planning_output,
    iflyauto::PlanningHMIOutputInfoStr *const planning_hmi_info) {
  // 行车规划部分
  // TODO(xjli32): 功能切换时，reset
  // ClearParkingInfo(planning_output);

  // sync parameters only if scene_type or dbw_status changes
  const bool dbw_status = session_.environmental_model().GetVehicleDbwStatus();
  if ((function_type != GENERAL_PLANNING_CONTEXT.GetStatemachine().scene_type ||
       (dbw_status != GENERAL_PLANNING_CONTEXT.GetStatemachine().dbw_status))) {
    session_.mutable_planning_context()->ResetTaskOutput();
    SyncParameters(function_type);
  }
  GENERAL_PLANNING_CONTEXT.MutableStatemachine().dbw_status = dbw_status;
  GENERAL_PLANNING_CONTEXT.MutableStatemachine().scene_type = function_type;

  // update environment model
  if (!environmental_model_manager_.Run()) {
    session_.mutable_planning_context()->Clear();
    return false;
  }

  bool planning_success;
  if (function_type == planning::common::SceneType::HIGHWAY) {
    planning_success = scc_function_->Plan();
  } else if (function_type == planning::common::SceneType::HPP) {
    planning_success = hpp_function_->Plan();
  } else {
    planning_success = scc_function_->Plan();
  }

  JSON_DEBUG_VALUE("current planning_success", planning_success);
  if (!planning_success) {
    LOG_DEBUG("Planning failed !!!! \n");
    if (!UpdateFailedPlanningResult()) {
      LOG_DEBUG("RunOnce failed !!!! \n");
      FillPlanningRequest(iflyauto::REQUEST_LEVEL_MIDDLE, planning_output);
      return false;
    }
  } else {
    UpdateSuccessfulPlanningResult();
  }

  std::cout << "The RunOnce is successed !!!!:" << std::endl;
  // 存在问题
  session_.mutable_planning_context()->mutable_last_planning_success() = planning_success;
  session_.mutable_planning_context()->mutable_planning_success() = true;

  const auto end_timestamp = IflyTime::Now_ms();
  const double time_consumption = end_timestamp - start_timestamp;
  LOG_DEBUG("general planning: planning time cost %f\n", time_consumption);
  JSON_DEBUG_VALUE("planning_time_cost", time_consumption);
  FillPlanningTrajectory(start_timestamp, planning_output);
  FillPlanningHmiInfo(start_timestamp, planning_hmi_info);
  ClearParkingInfo(planning_output);

  return true;
}

}  // namespace planning