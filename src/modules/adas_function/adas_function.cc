#include "adas_function.h"

#include <iostream>

#include "adas_function_lib.h"
#include "debug_info_log.h"
#include "general_planning_context.h"
#include "planning_context.h"

namespace planning {

AdasFunction::AdasFunction(framework::Session *session)
    : BaseFunction(session) {
  Init();
}

bool AdasFunction::Reset() {
  task_pipeline_.reset(nullptr);

  return true;
}

void AdasFunction::Init(void) {
  // Preprocess
  preprocess_ptr_ = std::make_shared<adas_function::preprocess::Preprocess>();
  preprocess_ptr_->Init();

  // LdwCore
  ldw_core_ptr_ = std::make_shared<adas_function::ldw_core::LdwCore>();
  // LdpCore
  ldp_core_ptr_ = std::make_shared<adas_function::ldp_core::LdpCore>();
  // ElkCore
  elk_core_ptr_ = std::make_shared<adas_function::elk_core::ElkCore>();
  // srCore
  tsr_core_ptr_ = std::make_shared<adas_function::tsr_core::TsrCore>();
  // IhcCore
  ihc_core_ptr_ = std::make_shared<adas_function::ihc_core::IntelligentHeadlightControl>();
}

void AdasFunction::StoreInfoForNextCycle(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  GetContext.mutable_last_cycle_info()->left_turn_light_state =
      GetContext.mutable_session()
          ->mutable_environmental_model()
          ->get_local_view()
          .vehicle_service_output_info.left_turn_light_state;

  GetContext.mutable_last_cycle_info()->right_turn_light_state =
      GetContext.mutable_session()
          ->mutable_environmental_model()
          ->get_local_view()
          .vehicle_service_output_info.right_turn_light_state;

  GetContext.mutable_last_cycle_info()->yaw_rad =
      GetContext.mutable_session()
          ->mutable_environmental_model()
          ->get_local_view()
          .localization_estimate.pose.euler_angles.yaw;

  GetContext.mutable_last_cycle_info()->accelerator_pedal_pos =
      GetContext.mutable_session()
          ->mutable_environmental_model()
          ->get_local_view()
          .vehicle_service_output_info.accelerator_pedal_pos;
}

bool AdasFunction::Plan() {
  /*
   *1.过路口，掉头数据：/data_cold/abu_zone/autoparse/jac_s811_35kw2/trigger/20230101/20230101-18-21-33/
   *2.过没有红绿灯的斑马线，能识别，不稳定：/data_cold/abu_zone/autoparse/jac_s811_35kw2/trigger/20230101/20230101-18-19-38/
   *3.速度满足，触发又问题/data_cold/abu_zone/autoparse/chery_e0y_04228/trigger/20241029/20241029-11-45-40/data_collection_CHERY_E0Y_04228_EVENT_FILTER_2024-10-29-11-45-40_no_camera.bag
   */
  double start_time = IflyTime::Now_ms();

  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  // 获取外部session数据
  GetContext.set_session_mutable(session_);
  if (GetContext.mutable_session() == nullptr) {
    ILOG_ERROR << "adas_function get session failed!!!";
    return false;
  }

  // Preprocess
  double start_time_preprocess = IflyTime::Now_ms();
  preprocess_ptr_->RunOnce();
  double end_time_preprocess = IflyTime::Now_ms();
  ILOG_DEBUG << "adas_preprocess_cost_time is [" << end_time_preprocess - start_time_preprocess << "]ms";
  JSON_DEBUG_VALUE("adas_preprocess_cost_time",
                   (end_time_preprocess - start_time_preprocess));

  // LdwCore
  ldw_core_ptr_->RunOnce();
  // LdwCore
  ldp_core_ptr_->RunOnce();
  // ElkCore
  elk_core_ptr_->RunOnce();
  // TsrCore
  tsr_core_ptr_->RunOnce();

  // IhcCore
  ihc_core_ptr_->RunOnce();

  // run lkas_function
  double start_time_lkas = IflyTime::Now_ms();
  if (GetContext.get_param()->hmi_test_switch == true) {
    TestLkasForHmi();
  }
  SetLkaTrajectory();
  double end_time_lkas = IflyTime::Now_ms();
  ILOG_DEBUG << "lkas_function cost is [" << end_time_lkas - start_time_lkas << "]ms";
  JSON_DEBUG_VALUE("lkas_function_cost_time_ms",
                   (end_time_lkas - start_time_lkas));

  // ihc_function test
  // double start_time_ihc = IflyTime::Now_ms();
  // auto ihc_function_ptr = session_->mutable_planning_context()
  //                             ->intelligent_headlight_control_function();
  // ihc_function_ptr->RunOnce();
  // double end_time_ihc = IflyTime::Now_ms();

  // tsr_function test
  // double start_time_tsr = IflyTime::Now_ms();
  // auto tsr_function_ptr =
  //     session_->mutable_planning_context()->traffic_sign_recognition_function();
  // tsr_function_ptr->RunOnce();
  // double end_time_tsr = IflyTime::Now_ms();

  Log();
  // 存储数据用于下一次循环
  StoreInfoForNextCycle();

  return true;
}
void AdasFunction::SetLkaTrajectory() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  // 设置目标轨迹
  double plan_traj_dt = 0.025;
  Eigen::Vector2d pos_proj;
  Eigen::Vector2d current_pos = Eigen::Vector2d::Zero();
  double s_proj = 0.0;
  if (GetContext.get_output_info()
          ->ldp_output_info_.ldp_left_intervention_flag_ ||
      GetContext.get_output_info()
          ->elk_output_info_.elk_left_intervention_flag_) {
    if (GetContext.mutable_road_info()->current_lane.left_line.valid) {
      adas_function::CalProjectionPointForLong(
          GetContext.get_road_info()->current_lane.left_line.dx_s_spline_,
          GetContext.get_road_info()->current_lane.left_line.dy_s_spline_, 0,
          GetContext.get_road_info()->current_lane.left_line.end_s, current_pos,
          s_proj);
    } else if (GetContext.mutable_road_info()
                   ->current_lane.left_roadedge.valid) {
      adas_function::CalProjectionPointForLong(
          GetContext.get_road_info()->current_lane.left_roadedge.dx_s_spline_,
          GetContext.get_road_info()->current_lane.left_roadedge.dy_s_spline_,
          0, GetContext.get_road_info()->current_lane.left_roadedge.end_s,
          current_pos, s_proj);
    } else {
      s_proj = 0.0;
    }
  } else if (GetContext.get_output_info()
                 ->ldp_output_info_.ldp_right_intervention_flag_ ||
             GetContext.get_output_info()
                 ->elk_output_info_.elk_right_intervention_flag_) {
    if (GetContext.mutable_road_info()->current_lane.right_line.valid) {
      adas_function::CalProjectionPointForLong(
          GetContext.get_road_info()->current_lane.right_line.dx_s_spline_,
          GetContext.get_road_info()->current_lane.right_line.dy_s_spline_, 0,
          GetContext.get_road_info()->current_lane.right_line.end_s,
          current_pos, s_proj);
    } else if (GetContext.mutable_road_info()
                   ->current_lane.right_roadedge.valid) {
      adas_function::CalProjectionPointForLong(
          GetContext.get_road_info()->current_lane.right_roadedge.dx_s_spline_,
          GetContext.get_road_info()->current_lane.right_roadedge.dy_s_spline_,
          0, GetContext.get_road_info()->current_lane.right_roadedge.end_s,
          current_pos, s_proj);
    } else {
      s_proj = 0.0;
    }
  } else {
    s_proj = 0.0;
  }

  auto &car2local =
      session_->environmental_model().get_ego_state_manager()->get_car2enu();
  auto lkas_trajectory = GetContext.mutable_lka_trajectory_info();
  lkas_trajectory->trajectory_points_size = 0;
  lkas_trajectory->trajectory_type =
      iflyauto::TRAJECTORY_TYPE_TRAJECTORY_POINTS;
  for (size_t i = 0; i < PLANNING_TRAJ_POINTS_MAX_NUM; i++) {
    auto path_point = &lkas_trajectory->trajectory_points[i];
    double s_ref = std::max(1.0, GetContext.get_state_info()->vehicle_speed) *
                       plan_traj_dt * i +
                   s_proj;
    Eigen::Vector3d car_point, local_point;
    if (GetContext.get_output_info()
            ->ldp_output_info_.ldp_left_intervention_flag_ ||
        GetContext.get_output_info()
            ->elk_output_info_.elk_left_intervention_flag_) {
      lkas_trajectory->available = true;
      if (GetContext.mutable_road_info()->current_lane.left_line.valid) {
        car_point.x() =
            GetContext.get_road_info()->current_lane.left_line.dx_s_spline_(
                s_ref);
        car_point.y() =
            GetContext.get_road_info()->current_lane.left_line.dy_s_spline_(
                s_ref) -
            GetContext.get_param()->ldp_center_line_offset -
            0.5 * GetContext.get_param()->ego_width;
        car_point.z() = 0.0;
      } else if (GetContext.mutable_road_info()
                     ->current_lane.left_roadedge.valid) {
        car_point.x() =
            GetContext.get_road_info()->current_lane.left_roadedge.dx_s_spline_(
                s_ref);
        car_point.y() =
            GetContext.get_road_info()->current_lane.left_roadedge.dy_s_spline_(
                s_ref) -
            GetContext.get_param()->ldp_center_roadedge_offset -
            0.5 * GetContext.get_param()->ego_width;
        car_point.z() = 0.0;
      } else {
        lkas_trajectory->available = false;
        car_point.x() = s_ref;
        car_point.y() = 0.0;
        car_point.z() = 0.0;
      }
    } else if (GetContext.get_output_info()
                   ->ldp_output_info_.ldp_right_intervention_flag_ ||
               GetContext.get_output_info()
                   ->elk_output_info_.elk_right_intervention_flag_) {
      lkas_trajectory->available = true;
      if (GetContext.get_road_info()->current_lane.right_line.valid) {
        car_point.x() =
            GetContext.get_road_info()->current_lane.right_line.dx_s_spline_(
                s_ref);
        car_point.y() =
            GetContext.get_road_info()->current_lane.right_line.dy_s_spline_(
                s_ref) +
            GetContext.get_param()->ldp_center_line_offset +
            0.5 * GetContext.get_param()->ego_width;
        car_point.z() = 0.0;
      } else if (GetContext.get_road_info()
                     ->current_lane.right_roadedge.valid) {
        car_point.x() = GetContext.get_road_info()
                            ->current_lane.right_roadedge.dx_s_spline_(s_ref);
        car_point.y() = GetContext.get_road_info()
                            ->current_lane.right_roadedge.dy_s_spline_(s_ref) +
                        GetContext.get_param()->ldp_center_roadedge_offset +
                        0.5 * GetContext.get_param()->ego_width;
        car_point.z() = 0.0;
      } else {
        lkas_trajectory->available = false;
        car_point.x() = s_ref;
        car_point.y() = 0.0;
        car_point.z() = 0.0;
      }
    } else {
      lkas_trajectory->available = false;
      car_point.x() = s_ref;
      car_point.y() = 0.0;
      car_point.z() = 0.0;
    }
    local_point = car2local * car_point;
    path_point->x = local_point.x();
    path_point->y = local_point.y();
    path_point->heading_yaw = 0.0;
    path_point->curvature = 0.0;
    path_point->t = plan_traj_dt * i;
    path_point->v = GetContext.get_state_info()->vehicle_speed;
    path_point->a = 0.0;
    path_point->distance =
        GetContext.get_state_info()->vehicle_speed * plan_traj_dt * i;
    path_point->jerk = 0.0;
    ++(lkas_trajectory->trajectory_points_size);
  }
  ILOG_DEBUG << "lks_trajectory_.trajectory_points_size = "
             << (int)lkas_trajectory->trajectory_points_size;
}
void AdasFunction::TestLkasForHmi(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  // test for ldw
  if (GetContext.get_param()->hmi_ldw_state == 1) {
    GetContext.mutable_output_info()->ldw_output_info_.ldw_state_ =
        iflyauto::LDWFunctionFSMWorkState::LDW_FUNCTION_FSM_WORK_STATE_OFF;
  } else if (GetContext.get_param()->hmi_ldw_state == 2) {
    GetContext.mutable_output_info()->ldw_output_info_.ldw_state_ =
        iflyauto::LDWFunctionFSMWorkState::LDW_FUNCTION_FSM_WORK_STATE_STANDBY;
  } else if (GetContext.get_param()->hmi_ldw_state == 3) {
    GetContext.mutable_output_info()->ldw_output_info_.ldw_state_ =
        iflyauto::LDWFunctionFSMWorkState::
            LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION;
  } else if (GetContext.get_param()->hmi_ldw_state == 4) {
    GetContext.mutable_output_info()->ldw_output_info_.ldw_state_ =
        iflyauto::LDWFunctionFSMWorkState::
            LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION;
  } else if (GetContext.get_param()->hmi_ldw_state == 5) {
    GetContext.mutable_output_info()->ldw_output_info_.ldw_state_ =
        iflyauto::LDWFunctionFSMWorkState::
            LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION;
  } else {
    GetContext.mutable_output_info()->ldw_output_info_.ldw_state_ = iflyauto::
        LDWFunctionFSMWorkState::LDW_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
  }

  if (GetContext.mutable_output_info()->ldw_output_info_.ldw_state_ ==
      iflyauto::LDWFunctionFSMWorkState::
          LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) {
    GetContext.mutable_output_info()->ldw_output_info_.ldw_left_warning_ = true;
  } else {
    GetContext.mutable_output_info()->ldw_output_info_.ldw_left_warning_ =
        false;
  }
  if (GetContext.mutable_output_info()->ldw_output_info_.ldw_state_ ==
      iflyauto::LDWFunctionFSMWorkState::
          LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    GetContext.mutable_output_info()->ldw_output_info_.ldw_right_warning_ =
        true;
  } else {
    GetContext.mutable_output_info()->ldw_output_info_.ldw_right_warning_ =
        false;
  }
  // test for ldp
  if (GetContext.get_param()->hmi_ldp_state == 1) {
    GetContext.mutable_output_info()->ldp_output_info_.ldp_state_ =
        iflyauto::LDPFunctionFSMWorkState::LDP_FUNCTION_FSM_WORK_STATE_OFF;
  } else if (GetContext.get_param()->hmi_ldp_state == 2) {
    GetContext.mutable_output_info()->ldp_output_info_.ldp_state_ =
        iflyauto::LDPFunctionFSMWorkState::LDP_FUNCTION_FSM_WORK_STATE_STANDBY;
  } else if (GetContext.get_param()->hmi_ldp_state == 3) {
    GetContext.mutable_output_info()->ldp_output_info_.ldp_state_ =
        iflyauto::LDPFunctionFSMWorkState::
            LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION;
  } else if (GetContext.get_param()->hmi_ldp_state == 4) {
    GetContext.mutable_output_info()->ldp_output_info_.ldp_state_ =
        iflyauto::LDPFunctionFSMWorkState::
            LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION;
  } else if (GetContext.get_param()->hmi_ldp_state == 5) {
    GetContext.mutable_output_info()->ldp_output_info_.ldp_state_ =
        iflyauto::LDPFunctionFSMWorkState::
            LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION;
  } else {
    GetContext.mutable_output_info()->ldp_output_info_.ldp_state_ = iflyauto::
        LDPFunctionFSMWorkState::LDP_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
  }

  if (GetContext.get_output_info()->ldp_output_info_.ldp_state_ ==
      iflyauto::LDPFunctionFSMWorkState::
          LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) {
    GetContext.mutable_output_info()
        ->ldp_output_info_.ldp_left_intervention_flag_ = true;
  } else {
    GetContext.mutable_output_info()
        ->ldp_output_info_.ldp_left_intervention_flag_ = false;
  }
  if (GetContext.get_output_info()->ldp_output_info_.ldp_state_ ==
      iflyauto::LDPFunctionFSMWorkState::
          LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    GetContext.mutable_output_info()
        ->ldp_output_info_.ldp_right_intervention_flag_ = true;
  } else {
    GetContext.mutable_output_info()
        ->ldp_output_info_.ldp_right_intervention_flag_ = false;
  }
  // test for elk
  if (GetContext.get_param()->hmi_elk_state == 1) {
    GetContext.mutable_output_info()->elk_output_info_.elk_state_ =
        iflyauto::ELKFunctionFSMWorkState::ELK_FUNCTION_FSM_WORK_STATE_OFF;
  } else if (GetContext.get_param()->hmi_elk_state == 2) {
    GetContext.mutable_output_info()->elk_output_info_.elk_state_ =
        iflyauto::ELKFunctionFSMWorkState::ELK_FUNCTION_FSM_WORK_STATE_STANDBY;
  } else if (GetContext.get_param()->hmi_elk_state == 3) {
    GetContext.mutable_output_info()->elk_output_info_.elk_state_ =
        iflyauto::ELKFunctionFSMWorkState::
            ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION;
  } else if (GetContext.get_param()->hmi_elk_state == 4) {
    GetContext.mutable_output_info()->elk_output_info_.elk_state_ =
        iflyauto::ELKFunctionFSMWorkState::
            ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION;
  } else if (GetContext.get_param()->hmi_elk_state == 5) {
    GetContext.mutable_output_info()->elk_output_info_.elk_state_ =
        iflyauto::ELKFunctionFSMWorkState::
            ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION;
  } else {
    GetContext.mutable_output_info()->elk_output_info_.elk_state_ = iflyauto::
        ELKFunctionFSMWorkState::ELK_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
  }
  if (GetContext.get_output_info()->elk_output_info_.elk_state_ ==
      iflyauto::ELKFunctionFSMWorkState::
          ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) {
    GetContext.mutable_output_info()
        ->elk_output_info_.elk_left_intervention_flag_ = true;
  } else {
    GetContext.mutable_output_info()
        ->elk_output_info_.elk_left_intervention_flag_ = false;
  }
  if (GetContext.get_output_info()->elk_output_info_.elk_state_ ==
      iflyauto::ELKFunctionFSMWorkState::
          ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    GetContext.mutable_output_info()
        ->elk_output_info_.elk_right_intervention_flag_ = true;
  } else {
    GetContext.mutable_output_info()
        ->elk_output_info_.elk_right_intervention_flag_ = false;
  }
  // tsr
  if (GetContext.get_param()->hmi_tsr_state == 1) {
    GetContext.mutable_output_info()->tsr_output_info_.tsr_state_ =
        iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_OFF;
  } else if (GetContext.get_param()->hmi_tsr_state == 2) {
    GetContext.mutable_output_info()->tsr_output_info_.tsr_state_ =
        iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_STANDBY;
  } else if (GetContext.get_param()->hmi_tsr_state == 3) {
    GetContext.mutable_output_info()->tsr_output_info_.tsr_state_ =
        iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_ACTIVE;
  } else {
    GetContext.mutable_output_info()->tsr_output_info_.tsr_state_ = iflyauto::
        TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
  }
  GetContext.mutable_output_info()->tsr_output_info_.tsr_warning_ = false;
  GetContext.mutable_output_info()->tsr_output_info_.tsr_speed_limit_ = 0;
  if (GetContext.get_param()->hmi_tsr_state == 4) {
    GetContext.mutable_output_info()->tsr_output_info_.tsr_state_ =
        iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_ACTIVE;
    GetContext.mutable_output_info()->tsr_output_info_.tsr_warning_ = false;
    GetContext.mutable_output_info()->tsr_output_info_.tsr_speed_limit_ =
        GetContext.get_param()->hmi_tsr_speed_limit;
  } else if (GetContext.get_param()->hmi_tsr_state == 5) {
    GetContext.mutable_output_info()->tsr_output_info_.tsr_state_ =
        iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_ACTIVE;
    GetContext.mutable_output_info()->tsr_output_info_.tsr_warning_ = true;
    GetContext.mutable_output_info()->tsr_output_info_.tsr_speed_limit_ =
        GetContext.get_param()->hmi_tsr_speed_limit;
  }
  return;
}
void AdasFunction::Log(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  // Parameters
  JSON_DEBUG_VALUE("params_dt", GetContext.get_param()->dt);
  JSON_DEBUG_VALUE("params_ego_length", GetContext.get_param()->ego_length);
  JSON_DEBUG_VALUE("params_ego_width", GetContext.get_param()->ego_width);
  JSON_DEBUG_VALUE("params_origin_2_front_bumper",
                   GetContext.get_param()->origin_2_front_bumper);
  JSON_DEBUG_VALUE("params_origin_2_rear_bumper",
                   GetContext.get_param()->origin_2_rear_bumper);
  JSON_DEBUG_VALUE("params_steer_ratio", GetContext.get_param()->steer_ratio);
  JSON_DEBUG_VALUE("params_wheel_base", GetContext.get_param()->wheel_base);
  JSON_DEBUG_VALUE("params_ldp_c0_right_offset",
                   GetContext.get_param()->ldp_c0_right_offset);
  JSON_DEBUG_VALUE("params_ldp_center_line_offset",
                   GetContext.get_param()->ldp_center_line_offset);
  JSON_DEBUG_VALUE("params_ldp_ttlc_right_hack",
                   GetContext.get_param()->ldp_ttlc_right_hack);
  JSON_DEBUG_VALUE("params_ldp_tlc_thrd", GetContext.get_param()->ldp_tlc_thrd);
  JSON_DEBUG_VALUE("params_ldw_enable_speed",
                   GetContext.get_param()->ldw_enable_speed);
  // vehicle state info
  JSON_DEBUG_VALUE("state_ctrl_output_steering_angle",
                   GetContext.get_state_info()->ctrl_output_steering_angle);
  JSON_DEBUG_VALUE("state_left_turn_light_off_time",
                   GetContext.get_state_info()->left_turn_light_off_time);
  JSON_DEBUG_VALUE("state_right_turn_light_off_time",
                   GetContext.get_state_info()->right_turn_light_off_time);
  JSON_DEBUG_VALUE("state_driver_hand_trq",
                   GetContext.get_state_info()->driver_hand_trq);
  JSON_DEBUG_VALUE("state_ego_curvature",
                   GetContext.get_state_info()->ego_curvature);
  JSON_DEBUG_VALUE("state_fl_wheel_distance_to_line",
                   GetContext.get_state_info()->fl_wheel_distance_to_line);
  JSON_DEBUG_VALUE("state_fr_wheel_distance_to_line",
                   GetContext.get_state_info()->fr_wheel_distance_to_line);
  JSON_DEBUG_VALUE("state_fl_wheel_distance_to_roadedge",
                   GetContext.get_state_info()->fl_wheel_distance_to_roadedge);
  JSON_DEBUG_VALUE("state_fr_wheel_distance_to_roadedge",
                   GetContext.get_state_info()->fr_wheel_distance_to_roadedge);
  JSON_DEBUG_VALUE("state_right_turn_light_off_time",
                   GetContext.get_state_info()->right_turn_light_off_time);
  JSON_DEBUG_VALUE("state_vehicle_speed",
                   GetContext.get_state_info()->vehicle_speed);
  JSON_DEBUG_VALUE("state_dispaly_vehicle_speed",
                   GetContext.get_state_info()->display_vehicle_speed);
  JSON_DEBUG_VALUE("state_yaw_rate", GetContext.get_state_info()->yaw_rate);
  JSON_DEBUG_VALUE("state_yaw_rate_observer",
                   GetContext.get_state_info()->yaw_rate_observer);
  JSON_DEBUG_VALUE("state_yaw_rate_loc",
                   GetContext.get_state_info()->yaw_rate_loc);
  JSON_DEBUG_VALUE("state_left_departure_speed",
                   GetContext.get_state_info()->veh_left_departure_speed);
  JSON_DEBUG_VALUE("state_right_departure_speed",
                   GetContext.get_state_info()->veh_right_departure_speed);
  JSON_DEBUG_VALUE("state_steer_wheel_angle_degree",
                   GetContext.get_state_info()->steer_wheel_angle_degree);
  JSON_DEBUG_VALUE("state_lat_departure_acc",
                   GetContext.get_state_info()->lat_departure_acc);
  // road info
  JSON_DEBUG_VALUE("road_lane_changed_flag",
                   GetContext.get_road_info()->current_lane.lane_changed_flag);
  JSON_DEBUG_VALUE("road_lane_width",
                   GetContext.get_road_info()->current_lane.lane_width);
  JSON_DEBUG_VALUE("road_lane_width_valid",
                   GetContext.get_road_info()->current_lane.lane_width_valid);
  JSON_DEBUG_VALUE(
      "road_left_sideway_exist_flag",
      GetContext.get_road_info()->current_lane.left_sideway_exist_flag);
  JSON_DEBUG_VALUE(
      "road_right_sideway_exist_flag",
      GetContext.get_road_info()->current_lane.right_sideway_exist_flag);
//   JSON_DEBUG_VALUE("road_left_departure_permission_flag",
//                    GetContext.get_road_info()
//                        ->current_lane.left_safe_departure_permission_flag);
//   JSON_DEBUG_VALUE("road_right_departure_permission_flag",
//                    GetContext.get_road_info()
//                        ->current_lane.right_safe_departure_permission_flag);
  JSON_DEBUG_VALUE(
      "road_left_parallel_car_flag",
      GetContext.get_road_info()->current_lane.left_parallel_car_flag);
  JSON_DEBUG_VALUE(
      "road_right_parallel_car_flag",
      GetContext.get_road_info()->current_lane.right_parallel_car_flag);
  JSON_DEBUG_VALUE(
      "road_right_front_car_flag",
      GetContext.get_road_info()->current_lane.right_front_car_flag);
  JSON_DEBUG_VALUE(
      "road_left_front_car_flag",
      GetContext.get_road_info()->current_lane.left_front_car_flag);

  /*left*/
  JSON_DEBUG_VALUE(
      "road_left_line_boundary_type",
      int(GetContext.get_road_info()->current_lane.left_line.boundary_type));
  JSON_DEBUG_VALUE(
      "road_left_line_line_type",
      int(GetContext.get_road_info()->current_lane.left_line.line_type));
  JSON_DEBUG_VALUE("road_left_line_begin",
                   GetContext.get_road_info()->current_lane.left_line.begin);
  JSON_DEBUG_VALUE("road_left_line_end",
                   GetContext.get_road_info()->current_lane.left_line.end);
  JSON_DEBUG_VALUE("road_left_line_c0",
                   GetContext.get_road_info()->current_lane.left_line.c0);
  JSON_DEBUG_VALUE("road_left_line_c1",
                   GetContext.get_road_info()->current_lane.left_line.c1);
  JSON_DEBUG_VALUE("road_left_line_c2",
                   GetContext.get_road_info()->current_lane.left_line.c2);
  JSON_DEBUG_VALUE("road_left_line_c3",
                   GetContext.get_road_info()->current_lane.left_line.c3);
  JSON_DEBUG_VALUE("road_left_line_valid",
                   GetContext.get_road_info()->current_lane.left_line.valid);
  JSON_DEBUG_VALUE(
      "road_left_roadedge_valid",
      GetContext.get_road_info()->current_lane.left_roadedge.valid);
  JSON_DEBUG_VALUE(
      "road_left_roadedge_begin_x",
      GetContext.get_road_info()->current_lane.left_roadedge.begin_x);
  JSON_DEBUG_VALUE(
      "road_left_roadedge_end_x",
      GetContext.get_road_info()->current_lane.left_roadedge.end_x);
  // road segments
  JSON_DEBUG_VALUE(
      "road_left_line_segement0_length",
      GetContext.get_road_info()->current_lane.left_line.segment0_length);
  JSON_DEBUG_VALUE(
      "road_left_line_segement0_type",
      int(GetContext.get_road_info()->current_lane.left_line.segment0_type));
  JSON_DEBUG_VALUE(
      "road_left_line_segement1_length",
      GetContext.get_road_info()->current_lane.left_line.segment1_length);
  JSON_DEBUG_VALUE(
      "road_left_line_segement1_type",
      int(GetContext.get_road_info()->current_lane.left_line.segment1_type));
  JSON_DEBUG_VALUE(
      "road_left_line_segement2_length",
      GetContext.get_road_info()->current_lane.left_line.segment2_length);
  JSON_DEBUG_VALUE(
      "road_left_line_segement2_type",
      int(GetContext.get_road_info()->current_lane.left_line.segment2_type));
  JSON_DEBUG_VALUE(
      "road_left_line_segement3_length",
      GetContext.get_road_info()->current_lane.left_line.segment3_length);
  JSON_DEBUG_VALUE(
      "road_left_line_segement3_type",
      int(GetContext.get_road_info()->current_lane.left_line.segment3_type));
  /*right*/
  JSON_DEBUG_VALUE(
      "road_right_line_boundary_type",
      int(GetContext.get_road_info()->current_lane.right_line.boundary_type));
  JSON_DEBUG_VALUE(
      "road_right_line_line_type",
      int(GetContext.get_road_info()->current_lane.right_line.line_type));
  JSON_DEBUG_VALUE("road_right_line_begin",
                   GetContext.get_road_info()->current_lane.right_line.begin);
  JSON_DEBUG_VALUE("road_right_line_end",
                   GetContext.get_road_info()->current_lane.right_line.end);
  JSON_DEBUG_VALUE("road_right_line_c0",
                   GetContext.get_road_info()->current_lane.right_line.c0);
  JSON_DEBUG_VALUE("road_right_line_c1",
                   GetContext.get_road_info()->current_lane.right_line.c1);
  JSON_DEBUG_VALUE("road_right_line_c2",
                   GetContext.get_road_info()->current_lane.right_line.c2);
  JSON_DEBUG_VALUE("road_right_line_c3",
                   GetContext.get_road_info()->current_lane.right_line.c3);
  JSON_DEBUG_VALUE("road_right_line_valid",
                   GetContext.get_road_info()->current_lane.right_line.valid);
  JSON_DEBUG_VALUE(
      "road_right_roadedge_valid",
      GetContext.get_road_info()->current_lane.right_roadedge.valid);
  JSON_DEBUG_VALUE(
      "road_right_roadedge_begin_x",
      GetContext.get_road_info()->current_lane.right_roadedge.begin_x);
  JSON_DEBUG_VALUE(
      "road_right_roadedge_end_x",
      GetContext.get_road_info()->current_lane.right_roadedge.end_x);
  // road calculate
  // road segments
  JSON_DEBUG_VALUE(
      "road_right_line_segement0_length",
      GetContext.get_road_info()->current_lane.right_line.segment0_length);
  JSON_DEBUG_VALUE(
      "road_right_line_segement0_type",
      int(GetContext.get_road_info()->current_lane.right_line.segment0_type));
  JSON_DEBUG_VALUE(
      "road_right_line_segement1_length",
      GetContext.get_road_info()->current_lane.right_line.segment1_length);
  JSON_DEBUG_VALUE(
      "road_right_line_segement1_type",
      int(GetContext.get_road_info()->current_lane.right_line.segment1_type));
  JSON_DEBUG_VALUE(
      "road_right_line_segement2_length",
      GetContext.get_road_info()->current_lane.right_line.segment2_length);
  JSON_DEBUG_VALUE(
      "road_right_line_segement2_type",
      int(GetContext.get_road_info()->current_lane.right_line.segment2_type));
  JSON_DEBUG_VALUE(
      "road_right_line_segement3_length",
      GetContext.get_road_info()->current_lane.right_line.segment3_length);
  JSON_DEBUG_VALUE(
      "road_right_line_segement3_type",
      int(GetContext.get_road_info()->current_lane.right_line.segment3_type));
  if (GetContext.get_param()->adas_sim_switch) {
    JSON_DEBUG_VECTOR(
        "road_left_line_all_dx_vec_",
        GetContext.get_road_info()->current_lane.left_line.dx_vec_, 2);
    JSON_DEBUG_VECTOR(
        "road_left_line_all_dy_vec_",
        GetContext.get_road_info()->current_lane.left_line.dy_vec_, 2);
    JSON_DEBUG_VECTOR(
        "road_right_line_all_dx_vec_",
        GetContext.get_road_info()->current_lane.right_line.dx_vec_, 2);
    JSON_DEBUG_VECTOR(
        "road_right_line_all_dy_vec_",
        GetContext.get_road_info()->current_lane.right_line.dy_vec_, 2);

    JSON_DEBUG_VECTOR(
        "road_left_roadedge_all_dx_vec_",
        GetContext.get_road_info()->current_lane.left_roadedge.all_dx_vec_, 2);
    JSON_DEBUG_VECTOR(
        "road_left_roadedge_all_dy_vec_",
        GetContext.get_road_info()->current_lane.left_roadedge.all_dy_vec_, 2);
    JSON_DEBUG_VECTOR(
        "road_right_roadedge_all_dx_vec_",
        GetContext.get_road_info()->current_lane.right_roadedge.all_dx_vec_, 2);
    JSON_DEBUG_VECTOR(
        "road_right_roadedge_all_dy_vec_",
        GetContext.get_road_info()->current_lane.right_roadedge.all_dy_vec_, 2);
    // OBJ_info
    /*obj loc*/
    std::vector<double> obj_corner_vector;
    obj_corner_vector.clear();
    obj_corner_vector.resize(8, 0.0);
    if (GetContext.get_objs_info()->objs_selected.fl_objs.vehicle_info_valid) {
      obj_corner_vector = adas_function::ObjCornersCalculate(
          GetContext.get_objs_info()->objs_selected.fl_objs.vehicle_info);
    }
    JSON_DEBUG_VECTOR("obj_fl_obj_loc_vec", obj_corner_vector, 2);
    obj_corner_vector.clear();
    obj_corner_vector.resize(8, 0.0);
    if (GetContext.get_objs_info()->objs_selected.fm_objs.vehicle_info_valid) {
      obj_corner_vector = adas_function::ObjCornersCalculate(
          GetContext.get_objs_info()->objs_selected.fm_objs.vehicle_info);
    }
    JSON_DEBUG_VECTOR("obj_fm_obj_loc_vec", obj_corner_vector, 2);
    obj_corner_vector.clear();
    obj_corner_vector.resize(8, 0.0);
    if (GetContext.get_objs_info()->objs_selected.fr_objs.vehicle_info_valid) {
      obj_corner_vector = adas_function::ObjCornersCalculate(
          GetContext.get_objs_info()->objs_selected.fr_objs.vehicle_info);
    }
    JSON_DEBUG_VECTOR("obj_fr_obj_loc_vec", obj_corner_vector, 2);
    obj_corner_vector.clear();
    obj_corner_vector.resize(8, 0.0);
    if (GetContext.get_objs_info()->objs_selected.ml_objs.vehicle_info_valid) {
      obj_corner_vector = adas_function::ObjCornersCalculate(
          GetContext.get_objs_info()->objs_selected.ml_objs.vehicle_info);
    }
    JSON_DEBUG_VECTOR("obj_ml_obj_loc_vec", obj_corner_vector, 2);
    obj_corner_vector.clear();
    obj_corner_vector.resize(8, 0.0);
    if (GetContext.get_objs_info()->objs_selected.mr_objs.vehicle_info_valid) {
      obj_corner_vector = adas_function::ObjCornersCalculate(
          GetContext.get_objs_info()->objs_selected.mr_objs.vehicle_info);
    }
    JSON_DEBUG_VECTOR("obj_mr_obj_loc_vec", obj_corner_vector, 2);
    obj_corner_vector.clear();
    obj_corner_vector.resize(8, 0.0);
    if (GetContext.get_objs_info()->objs_selected.rl_objs.vehicle_info_valid) {
      obj_corner_vector = adas_function::ObjCornersCalculate(
          GetContext.get_objs_info()->objs_selected.rl_objs.vehicle_info);
    }
    JSON_DEBUG_VECTOR("obj_rl_obj_loc_vec", obj_corner_vector, 2);
    obj_corner_vector.clear();
    obj_corner_vector.resize(8, 0.0);
    if (GetContext.get_objs_info()->objs_selected.rm_objs.vehicle_info_valid) {
      obj_corner_vector = adas_function::ObjCornersCalculate(
          GetContext.get_objs_info()->objs_selected.rm_objs.vehicle_info);
    }
    JSON_DEBUG_VECTOR("obj_rm_obj_loc_vec", obj_corner_vector, 2);
    obj_corner_vector.clear();
    obj_corner_vector.resize(8, 0.0);
    if (GetContext.get_objs_info()->objs_selected.rr_objs.vehicle_info_valid) {
      obj_corner_vector = adas_function::ObjCornersCalculate(
          GetContext.get_objs_info()->objs_selected.rr_objs.vehicle_info);
    }
    JSON_DEBUG_VECTOR("obj_rr_obj_loc_vec", obj_corner_vector, 2);
  }
  // adas debug info

  // lkas debug info
}

}  // namespace planning