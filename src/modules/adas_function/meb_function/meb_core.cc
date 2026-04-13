#include "meb_core.h"

#include <algorithm>
#include <iostream>

#include "adas_function_lib.h"
#include "geometry_math.h"
#include "polygon_base.h"

using namespace planning;
namespace adas_function {
namespace meb_core {

void MebCore::InitOnce(void) {
  od_straight_scenario_ptr_ =
      std::make_shared<adas_function::OdStraightScenario>();
  od_crossing_scenario_ptr_ =
      std::make_shared<adas_function::OdCrossingScenario>();
  occ_straight_scenario_ptr_ =
      std::make_shared<adas_function::OccStraightScenario>();
  uss_straight_scenario_ptr_ =
      std::make_shared<adas_function::UssStraightScenario>();
  meb_brake_duration_ = 0.0;
  meb_no_response_duration_ = 0.0;
  meb_hold_duration_ = 0.0;
  meb_cooling_time_remain_ = 0.0;
  meb_state_info_.first_state = OFF;
  meb_state_info_.second_state = NoIntervertion;
  meb_index_ = 0;
}

void MebCore::UpdateMebEnableCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto meb_param = adas_function::MebPreprocess::GetInstance().GetMebParam();
  auto meb_input = adas_function::MebPreprocess::GetInstance().GetMebInput();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint32 enable_code = 0;
  // bit 0
  // 判断车速是否处于工作车速范围内
  uint8 vehicle_speed_display_kph =
      vehicle_service_output_info_ptr->vehicle_speed_display * 3.6 + 0.5;
  if (vehicle_speed_display_kph < meb_param.enable_vehspd_display_kph_min) {
    enable_code += uint16_bit[0];
  } else if (vehicle_speed_display_kph >
             meb_param.enable_vehspd_display_kph_max) {
    enable_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // bit 1
  // 判断挡位是否处于P档
  if (GetContext.get_state_info()->shift_lever_state ==
          iflyauto::ShiftLeverStateEnum::ShiftLeverState_P ||
      GetContext.get_state_info()->shift_lever_state ==
          iflyauto::ShiftLeverStateEnum::ShiftLeverState_N) {
    enable_code += uint16_bit[1];
  } else {
    /*do nothing*/
  }

  if (!vehicle_service_output_info_ptr->aeb_actuator_status_available) {
    enable_code += uint16_bit[2];
  }

  if ((vehicle_service_output_info_ptr->fl_door_state == true) ||
      (vehicle_service_output_info_ptr->fr_door_state == true) ||
      (vehicle_service_output_info_ptr->rl_door_state == true) ||
      (vehicle_service_output_info_ptr->rr_door_state == true) ||
      (vehicle_service_output_info_ptr->fl_door_state_available == false) ||
      (vehicle_service_output_info_ptr->fr_door_state_available == false) ||
      (vehicle_service_output_info_ptr->rl_door_state_available == false) ||
      (vehicle_service_output_info_ptr->rr_door_state_available == false)) {
    enable_code += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  if ((vehicle_service_output_info_ptr->hood_state == true) ||
      (vehicle_service_output_info_ptr->hood_state_available == false)) {
    enable_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  //
  if ((vehicle_service_output_info_ptr->trunk_door_state == true) ||
      (vehicle_service_output_info_ptr->trunk_door_state_available == false)) {
    enable_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }

  if (GetContext.get_param()->meb_function_state_switch) {
    if (vehicle_service_output_info_ptr->fl_seat_belt_state == false ||
        vehicle_service_output_info_ptr->fl_seat_belt_state_available ==
            false) {
      enable_code += uint16_bit[1];
    } else {
    }
  }
  meb_state_info_.enable_code = enable_code;
}

void MebCore::UpdateMebDisableCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto meb_param = adas_function::MebPreprocess::GetInstance().GetMebParam();
  auto meb_input = adas_function::MebPreprocess::GetInstance().GetMebInput();
  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;
  uint32 disable_code = 0;
  // bit 0
  // 判断车速是否处于工作车速范围内
  uint8 vehicle_speed_display_kph =
      vehicle_service_output_info_ptr->vehicle_speed_display * 3.6 + 0.5;
  if (meb_state_info_.second_state != Intervertion) {
    if (vehicle_speed_display_kph < meb_param.disable_vehspd_display_kph_min) {
      disable_code += uint16_bit[0];
    } else if (vehicle_speed_display_kph >
               meb_param.disable_vehspd_display_kph_max) {
      disable_code += uint16_bit[0];
    } else {
      /*do nothing*/
    }
  }

  // bit 1
  // 判断挡位是否处于D档
  if (GetContext.get_state_info()->shift_lever_state ==
          iflyauto::ShiftLeverStateEnum::ShiftLeverState_P ||
      GetContext.get_state_info()->shift_lever_state ==
          iflyauto::ShiftLeverStateEnum::ShiftLeverState_N) {
    disable_code += uint16_bit[1];
  } else {
    /*do nothing*/
  }

  if (GetContext.get_param()->meb_function_state_switch) {
    if ((!vehicle_service_output_info_ptr->aeb_actuator_status_available)) {
      disable_code += uint16_bit[2];
    }

    if ((vehicle_service_output_info_ptr->fl_door_state == true) ||
        (vehicle_service_output_info_ptr->fr_door_state == true) ||
        (vehicle_service_output_info_ptr->rl_door_state == true) ||
        (vehicle_service_output_info_ptr->rr_door_state == true) ||
        (vehicle_service_output_info_ptr->hood_state == true) ||
        (vehicle_service_output_info_ptr->trunk_door_state == true) ||
        (vehicle_service_output_info_ptr->fl_door_state_available == false) ||
        (vehicle_service_output_info_ptr->fr_door_state_available == false) ||
        (vehicle_service_output_info_ptr->rl_door_state_available == false) ||
        (vehicle_service_output_info_ptr->rr_door_state_available == false) ||
        (vehicle_service_output_info_ptr->hood_state_available == false) ||
        (vehicle_service_output_info_ptr->trunk_door_state_available ==
         false)) {
      disable_code += uint16_bit[3];
    } else {
      /*do nothing*/
    }

    if (vehicle_service_output_info_ptr->fl_seat_belt_state == false ||
        vehicle_service_output_info_ptr->fl_seat_belt_state_available ==
            false) {
      disable_code += uint16_bit[4];
    } else {
    }
  }
  meb_state_info_.disable_code = disable_code;
}

void MebCore::UpdateMebFaultCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  uint32 fault_code = 0;
  auto degraded_driving_function_info_ptr =
      &GetContext.mutable_session()
           ->mutable_environmental_model()
           ->get_local_view()
           .degraded_driving_function_info;

  // bit 13
  // 故障降级
  if ((degraded_driving_function_info_ptr->meb.degraded == iflyauto::INHIBIT) ||
      (degraded_driving_function_info_ptr->meb.degraded ==
       iflyauto::ERROR_DEGRADED) ||
      (degraded_driving_function_info_ptr->meb.degraded ==
       iflyauto::ERROR_SAFE_STOP) ||
      (degraded_driving_function_info_ptr->meb.degraded ==
       iflyauto::MCU_COMM_SHUTDOWN)) {
    fault_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }
  meb_state_info_.fault_code = fault_code;
}

void MebCore::UpdateMebKickdownCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;
  auto veh_speed_ms = GetContext.get_state_info()->vehicle_speed;
  auto shift_level = GetContext.get_state_info()->shift_lever_state;
  auto epb_state = vehicle_service_output_info_ptr->epb_state;
  auto brake_pedal_pos = vehicle_service_output_info_ptr->brake_pedal_pos;
  auto accelerator_pedal_pos =
      vehicle_service_output_info_ptr->accelerator_pedal_pos;
  auto brake_pedal_pos_rate = adas_function::MebPreprocess::GetInstance()
                                  .GetMebInput()
                                  .brake_pedal_pos_rate;

  uint32 kickdown_code = 0;

  if (meb_hold_duration_ > 2.0) {
    kickdown_code += uint16_bit[0];
  }

  if (meb_hold_duration_ > 0.1 && veh_speed_ms < 0.1) {
    if (shift_level == iflyauto::ShiftLeverStateEnum::ShiftLeverState_P) {
      kickdown_code += uint16_bit[1];
    } else if (epb_state == iflyauto::EPBStateEnum::EPBState_FullyApplied ||
               epb_state == iflyauto::EPBStateEnum::EPBState_Applying) {
      kickdown_code += uint16_bit[1];
    } else {
    }
  }

  if (GetContext.get_param()->meb_function_state_switch) {
    if (brake_pedal_pos > 80.0) {
      kickdown_code += uint16_bit[2];
    }

    if (accelerator_pedal_pos > 80.0) {
      kickdown_code += uint16_bit[3];
    }

    if (meb_no_response_duration_ > 0.6) {
      kickdown_code += uint16_bit[4];
    }

    if (meb_brake_duration_ > 6.0) {
      kickdown_code += uint16_bit[5];
    }

    // if (brake_pedal_pos_rate > 600.0) {
    //   kickdown_code += uint16_bit[6];
    // }
  }
  meb_state_info_.kickdown_code = kickdown_code;
}

void MebCore::UpdateMebSuppCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;
  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;
  auto meb_input = adas_function::MebPreprocess::GetInstance().GetMebInput();

  uint32 supp_code = 0;
  if (meb_cooling_time_remain_ > 0.1) {
    supp_code += uint16_bit[0];
  }

  if (vehicle_service_output_info_ptr->vs_car_mode !=
      iflyauto::VsCarMode::VS_CAR_MODE_NORMAL_MODE) {
    supp_code += uint16_bit[1];
  } else {
  }

  if (vehicle_service_output_info_ptr->trailer_state == true) {
    supp_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  if ((function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_PARK_GUIDANCE) ||
      (function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_HPP_PARKING_IN) ||
      (function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_HPP_PARKING_OUT) ||
      (function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_RADS_TRACING) ||
      (function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_HPP_CRUISE_ROUTING) ||
      (function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_SCC_ACTIVATE) ||
      (function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_ACC_ACTIVATE) ||
      (function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_NOA_ACTIVATE)) {
    supp_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  if (vehicle_service_output_info_ptr->aeb_actuator_status == 2) {
    supp_code += uint16_bit[5];
  }
  meb_state_info_.supp_code = supp_code;
}

void MebCore::MebStateMachine(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto main_switch =
      adas_function::MebPreprocess::GetInstance().GetMebInput().meb_main_switch;
  auto vehicle_service_ptr = GetContext.mutable_session()
                                 ->mutable_environmental_model()
                                 ->get_local_view()
                                 .vehicle_service_output_info;
  auto aeb_actuator = vehicle_service_ptr.aeb_actuator_status;
  auto veh_speed_ms = GetContext.get_state_info()->vehicle_speed;
  auto &first_state = meb_state_info_.first_state;
  auto &second_state = meb_state_info_.second_state;
  auto &fault_code =
      meb_state_info_.fault_code;  // 故障码: 0: 无故障, 其他: 有故障
  auto &disable_code =
      meb_state_info_.disable_code;  // 0 无disable 其他 存在disable
  auto &enable_code = meb_state_info_.enable_code;      // 0 使能
  auto &kickdown_code = meb_state_info_.kickdown_code;  // 0 无kick down
  auto &supp_code =
      meb_state_info_.supp_code;  // 0 无intervention 其他 存在intervention
  auto &init_flag = meb_state_info_.state_machine_init_flag;

  if (init_flag == false) {
    // 状态机处于初始化状态 根据开关状态,决定第一个周期是输出OFF还是STANDBY
    if (main_switch == false) {
      first_state = OFF;
    } else {
      first_state = STANDBY;
    }
    init_flag = true;
    return;
  }

  // 状态机处于完成过初始化的状态
  switch (first_state) {
    case OFF:
      if (main_switch) {
        first_state = STANDBY;
      } else {
        first_state = OFF;
      }
      break;
    case FAULT:
      if (main_switch == false) {
        first_state = OFF;
      } else if (fault_code == 0) {
        first_state = STANDBY;
      } else {
      }
      break;
    case STANDBY:
      if (main_switch == false) {
        first_state = OFF;
      } else if (fault_code != 0) {
        first_state = FAULT;
      } else if (enable_code == 0) {
        first_state = ACTIVE;
      } else {
      }
      break;
    case ACTIVE:
      if (main_switch == false) {
        first_state = OFF;
      } else if (fault_code != 0) {
        first_state = FAULT;
      } else if (disable_code != 0) {
        first_state = STANDBY;
      } else {
        first_state = ACTIVE;
      }
      break;
    default:
      first_state = OFF;
      break;
  }

  switch (second_state) {
    case Intervertion:
      if (kickdown_code != 0) {
        second_state = Suppression;
      } else {
      }
      break;
    case NoIntervertion:
      if ((supp_code != 0)) {
        second_state = Suppression;
      } else if (meb_intervention_flag_) {
        second_state = Intervertion;
      }
      break;
    case Suppression:
      if (supp_code == 0 && meb_intervention_flag_ == false) {
        second_state = NoIntervertion;
      } else {
        second_state = Suppression;
      }
      break;
    default:
      second_state = NoIntervertion;
      break;
  }

  if (first_state != ACTIVE) {
    second_state = NoIntervertion;
    meb_hold_duration_ = 0;
    meb_no_response_duration_ = 0;
    meb_brake_duration_ = 0;
    meb_cooling_time_remain_ -= MEB_CYCLE_TIME_SEC;
    if (meb_cooling_time_remain_ < 0.0) {
      meb_cooling_time_remain_ = 0.0;
    }
  } else if (second_state != Intervertion) {
    meb_hold_duration_ = 0;
    meb_no_response_duration_ = 0;
    meb_brake_duration_ = 0;
    meb_cooling_time_remain_ -= MEB_CYCLE_TIME_SEC;
    if (meb_cooling_time_remain_ < 0.0) {
      meb_cooling_time_remain_ = 0.0;
    }
  } else if (second_state == Intervertion) {
    meb_brake_duration_ += MEB_CYCLE_TIME_SEC;

    if (aeb_actuator == 2) {
      meb_no_response_duration_ = 0.0;
    } else {
      meb_no_response_duration_ += MEB_CYCLE_TIME_SEC;
    }

    if (veh_speed_ms < 0.1) {
      meb_hold_duration_ += MEB_CYCLE_TIME_SEC;
    }

    meb_cooling_time_remain_ = 20.0;
  }
}

void MebCore::SetMebOutputInfo(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  // 状态机映射
  if (meb_state_info_.first_state == OFF) {
    meb_state_ =
        iflyauto::MEBFunctionFSMWorkState::MEB_FUNCTION_FSM_WORK_STATE_OFF;
  } else if (meb_state_info_.first_state == FAULT) {
    meb_state_ =
        iflyauto::MEBFunctionFSMWorkState::MEB_FUNCTION_FSM_WORK_STATE_FAULT;
  } else if (meb_state_info_.first_state == STANDBY) {
    meb_state_ =
        iflyauto::MEBFunctionFSMWorkState::MEB_FUNCTION_FSM_WORK_STATE_STANDBY;
  } else if (meb_state_info_.first_state == ACTIVE) {
    meb_state_ =
        iflyauto::MEBFunctionFSMWorkState::MEB_FUNCTION_FSM_WORK_STATE_ACTIVE;
  } else {
    meb_state_ =
        iflyauto::MEBFunctionFSMWorkState::MEB_FUNCTION_FSM_WORK_STATE_OFF;
  }

  if (GetContext.get_param()->meb_hmi_test_switch == true) {
    // 标定文件测试 仪表显示
    GetContext.mutable_output_info()->meb_output_info_.meb_state =
        (iflyauto::MEBFunctionFSMWorkState)GetContext.get_param()
            ->meb_hmi_state;
    GetContext.mutable_output_info()->meb_output_info_.meb_request_status =
        GetContext.get_param()->meb_hmi_status;
    GetContext.mutable_output_info()->meb_output_info_.meb_request_value =
        GetContext.get_param()->meb_hmi_vaule;
    GetContext.mutable_output_info()->meb_output_info_.meb_request_direction =
        (iflyauto::MEBInterventionDirection)GetContext.get_param()
            ->meb_hmi_direction;
  } else {
    GetContext.mutable_output_info()->meb_output_info_.meb_state = meb_state_;
    if (meb_state_ == iflyauto::MEBFunctionFSMWorkState::
                          MEB_FUNCTION_FSM_WORK_STATE_ACTIVE &&
        meb_state_info_.second_state == Intervertion) {
      if (GetContext.get_state_info()->shift_lever_state ==
          iflyauto::ShiftLeverStateEnum::ShiftLeverState_R) {
        GetContext.mutable_output_info()->meb_output_info_.meb_request_value =
            GetContext.get_param()->meb_request_acc *
            GetContext.get_param()->meb_reverse_acc_gain;
        GetContext.mutable_output_info()
            ->meb_output_info_.meb_request_direction =
            iflyauto::MEBInterventionDirection::MEB_INTERVENTION_DIRECTION_REAR;
      } else {
        GetContext.mutable_output_info()->meb_output_info_.meb_request_value =
            GetContext.get_param()->meb_request_acc;
        GetContext.mutable_output_info()
            ->meb_output_info_.meb_request_direction = iflyauto::
            MEBInterventionDirection::MEB_INTERVENTION_DIRECTION_FRONT;
      }
    } else {
      GetContext.mutable_output_info()->meb_output_info_.meb_request_value =
          0.0;
      GetContext.mutable_output_info()->meb_output_info_.meb_request_direction =
          iflyauto::MEBInterventionDirection::MEB_INTERVENTION_DIRECTION_NONE;
    }

#if defined(ADAS_IN_SIMULATION)

    if (meb_state_ == iflyauto::MEBFunctionFSMWorkState::
                          MEB_FUNCTION_FSM_WORK_STATE_ACTIVE &&
        meb_state_info_.second_state == Intervertion) {
      GetContext.mutable_output_info()->meb_output_info_.meb_request_status = 1;
    } else if (meb_intervention_flag_) {
      GetContext.mutable_output_info()->meb_output_info_.meb_request_status =
          100;
    } else {
      GetContext.mutable_output_info()->meb_output_info_.meb_request_status = 0;
    }
#else
    if (meb_state_ == iflyauto::MEBFunctionFSMWorkState::
                          MEB_FUNCTION_FSM_WORK_STATE_ACTIVE &&
        meb_state_info_.second_state == Intervertion &&
        GetContext.get_param()->meb_call_switch) {
      // 认为开启制动 meb_request_status = 1
      GetContext.mutable_output_info()->meb_output_info_.meb_request_status = 1;
    } else if (meb_intervention_flag_) {
      // meb 制动不开启 meb_request_status = 100
      GetContext.mutable_output_info()->meb_output_info_.meb_request_status =
          100;
      GetContext.mutable_output_info()->meb_output_info_.meb_request_value =
          0.0;
    } else {
      GetContext.mutable_output_info()->meb_output_info_.meb_request_status = 0;
    }

    if (GetContext.get_param()->meb_request_status_const_switch == true) {
      GetContext.mutable_output_info()->meb_output_info_.meb_request_status =
          GetContext.get_param()->meb_request_status_const;
    }
#endif
  }
}  // namespace meb_core

void MebCore::Log(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto meb_input = adas_function::MebPreprocess::GetInstance().GetMebInput();

  auto &MebInputInstacne = adas_function::MebPreprocess::GetInstance();
  JSON_DEBUG_VALUE("meb_version", (double)260409);
  JSON_DEBUG_VALUE("meb_first_state", (int)meb_state_info_.first_state);
  JSON_DEBUG_VALUE("meb_second_state", (int)meb_state_info_.second_state);
  JSON_DEBUG_VALUE("meb_enable_code", meb_state_info_.enable_code);
  JSON_DEBUG_VALUE("meb_disable_code", meb_state_info_.disable_code);
  JSON_DEBUG_VALUE("meb_fault_code", meb_state_info_.fault_code);
  JSON_DEBUG_VALUE("meb_kickdown_code", meb_state_info_.kickdown_code);
  JSON_DEBUG_VALUE("meb_supp_code", meb_state_info_.supp_code);
  JSON_DEBUG_VALUE("meb_intervention_flag", (int)meb_intervention_flag_);
  JSON_DEBUG_VALUE("meb_brake_duration", meb_brake_duration_);
  JSON_DEBUG_VALUE("meb_no_response_duration", meb_no_response_duration_);
  JSON_DEBUG_VALUE("meb_hold_duration", meb_hold_duration_);
  JSON_DEBUG_VALUE("meb_cooling_time_remain", meb_cooling_time_remain_);
  JSON_DEBUG_VALUE("meb_occ_obs_switch",
                   GetContext.get_param()->meb_occ_obs_switch);
  JSON_DEBUG_VALUE("meb_uss_obs_switch",
                   GetContext.get_param()->meb_uss_obs_switch);
  JSON_DEBUG_VALUE("meb_false_trigger_switch",
                   GetContext.get_param()->meb_false_trigger_switch);
  JSON_DEBUG_VALUE("meb_call_switch", GetContext.get_param()->meb_call_switch);

  JSON_DEBUG_VALUE(
      "meb_state",
      (int)GetContext.get_output_info()->meb_output_info_.meb_state);
  JSON_DEBUG_VALUE(
      "meb_request_status",
      GetContext.mutable_output_info()->meb_output_info_.meb_request_status);
  JSON_DEBUG_VALUE(
      "meb_request_value",
      GetContext.mutable_output_info()->meb_output_info_.meb_request_value);
  JSON_DEBUG_VALUE("meb_request_direction",
                   (int)GetContext.mutable_output_info()
                       ->meb_output_info_.meb_request_direction);

  if (od_straight_scenario_ptr_->final_collision_obj_info_.valid_num > 0) {
    JSON_DEBUG_VALUE("meb_od_obs_collision_flag", (int)true);
  } else {
    JSON_DEBUG_VALUE("meb_od_obs_collision_flag", (int)false);
  }

  if (od_crossing_scenario_ptr_->final_collision_obj_info_.valid_num > 0) {
    JSON_DEBUG_VALUE("meb_od_cross_obs_collision_flag", (int)true);
  } else {
    JSON_DEBUG_VALUE("meb_od_cross_obs_collision_flag", (int)false);
  }

  if (occ_straight_scenario_ptr_->final_collision_obj_info_.valid_num > 0) {
    JSON_DEBUG_VALUE("meb_occ_obs_collision_flag", (int)true);
  } else {
    JSON_DEBUG_VALUE("meb_occ_obs_collision_flag", (int)false);
  }

  if (uss_straight_scenario_ptr_->final_collision_obj_info_.valid_num > 0) {
    JSON_DEBUG_VALUE("meb_uss_obs_collision_flag", (int)true);
  } else {
    JSON_DEBUG_VALUE("meb_uss_obs_collision_flag", (int)false);
  }

  auto all_obs_point_num =
      od_straight_scenario_ptr_->final_collision_obj_info_.valid_num +
      od_crossing_scenario_ptr_->final_collision_obj_info_.valid_num +
      occ_straight_scenario_ptr_->final_collision_obj_info_.valid_num +
      uss_straight_scenario_ptr_->final_collision_obj_info_.valid_num;

  JSON_DEBUG_VALUE("meb_all_obs_point_num", all_obs_point_num);
  JSON_DEBUG_VALUE(
      "meb_fusion_od_obs_size",
      od_straight_scenario_ptr_->final_collision_obj_info_.valid_num);
  JSON_DEBUG_VALUE(
      "meb_fusion_od_cross_obs_size",
      od_crossing_scenario_ptr_->final_collision_obj_info_.valid_num);
  JSON_DEBUG_VALUE(
      "meb_fusion_occ_obs_size",
      occ_straight_scenario_ptr_->final_collision_obj_info_.valid_num);
  JSON_DEBUG_VALUE(
      "meb_fusion_uss_obs_size",
      uss_straight_scenario_ptr_->final_collision_obj_info_.valid_num);

  std::vector<double> meb_od_obs_x_vector = {};
  std::vector<double> meb_od_obs_y_vector = {};
  std::vector<double> meb_od_obs_rel_vx_vector = {};
  std::vector<double> meb_od_obs_rel_vy_vector = {};
  std::vector<double> meb_od_obs_stop_distance_buffer_vector = {};
  std::vector<double> meb_od_obs_track_id = {};
  std::vector<double> meb_od_obs_suppe_code = {};
  std::vector<double> meb_od_obs_type = {};
  std::vector<double> meb_od_an_avoid_by_steering = {};
  for (auto &obs :
       od_straight_scenario_ptr_->collision_obj_info_.interest_obj_vec_) {
    meb_od_obs_x_vector.push_back(obs.rel_x);
    meb_od_obs_y_vector.push_back(obs.rel_y);
    meb_od_obs_stop_distance_buffer_vector.push_back(obs.stop_distance_buffer);
    meb_od_obs_track_id.push_back(obs.track_id);
    meb_od_obs_suppe_code.push_back(obs.suppe_code);
    meb_od_obs_rel_vx_vector.push_back(obs.rel_vx);
    meb_od_obs_rel_vy_vector.push_back(obs.rel_vy);
    meb_od_obs_type.push_back(obs.type);
    meb_od_an_avoid_by_steering.push_back(obs.an_avoid_by_steering);
  }

  JSON_DEBUG_VECTOR("meb_od_obs_x_vector", meb_od_obs_x_vector, 2);
  JSON_DEBUG_VECTOR("meb_od_obs_y_vector", meb_od_obs_y_vector, 2);
  JSON_DEBUG_VECTOR("meb_od_obs_stop_distance_buffer_vector",
                    meb_od_obs_stop_distance_buffer_vector, 2);
  JSON_DEBUG_VECTOR("meb_od_obs_track_id", meb_od_obs_track_id, 2);
  JSON_DEBUG_VECTOR("meb_od_obs_suppe_code", meb_od_obs_suppe_code, 2);
  JSON_DEBUG_VECTOR("meb_od_obs_rel_vx_vector", meb_od_obs_rel_vx_vector, 2);
  JSON_DEBUG_VECTOR("meb_od_obs_rel_vy_vector", meb_od_obs_rel_vy_vector, 2);
  JSON_DEBUG_VECTOR("meb_od_obs_type", meb_od_obs_type, 2);
  JSON_DEBUG_VECTOR("meb_od_an_avoid_by_steering", meb_od_an_avoid_by_steering,
                    2);

  std::vector<double> meb_od_cross_obs_x_vector = {};
  std::vector<double> meb_od_cross_obs_y_vector = {};
  std::vector<double> meb_od_cross_obs_stop_distance_buffer_vector = {};
  std::vector<double> meb_od_cross_obs_track_id = {};
  std::vector<double> meb_od_cross_obs_suppe_code = {};
  std::vector<double> meb_od_cross_obs_rel_vx_vector = {};
  std::vector<double> meb_od_cross_obs_rel_vy_vector = {};
  std::vector<double> meb_od_cross_obs_type = {};
  std::vector<double> meb_od_cross_obs_an_avoid_by_steering = {};
  std::vector<double> meb_od_cross_obs_ay_avoid_by_accelerating = {};
  std::vector<double> meb_od_cross_obs_collision_point_y = {};
  for (auto &obs :
       od_crossing_scenario_ptr_->collision_obj_info_.interest_obj_vec_) {
    meb_od_cross_obs_x_vector.push_back(obs.rel_x);
    meb_od_cross_obs_y_vector.push_back(obs.rel_y);
    meb_od_cross_obs_stop_distance_buffer_vector.push_back(
        obs.stop_distance_buffer);
    meb_od_cross_obs_track_id.push_back(obs.track_id);
    meb_od_cross_obs_suppe_code.push_back(obs.suppe_code);
    meb_od_cross_obs_rel_vx_vector.push_back(obs.rel_vx);
    meb_od_cross_obs_rel_vy_vector.push_back(obs.rel_vy);
    meb_od_cross_obs_type.push_back(obs.type);
    meb_od_cross_obs_an_avoid_by_steering.push_back(obs.an_avoid_by_steering);
    meb_od_cross_obs_ay_avoid_by_accelerating.push_back(
        obs.ay_avoid_by_accelerating);
    meb_od_cross_obs_collision_point_y.push_back(obs.collision_point_y);
  }

  JSON_DEBUG_VECTOR("meb_od_cross_obs_x_vector", meb_od_cross_obs_x_vector, 2);
  JSON_DEBUG_VECTOR("meb_od_cross_obs_y_vector", meb_od_cross_obs_y_vector, 2);
  JSON_DEBUG_VECTOR("meb_od_cross_obs_stop_distance_buffer_vector",
                    meb_od_cross_obs_stop_distance_buffer_vector, 2);
  JSON_DEBUG_VECTOR("meb_od_cross_obs_track_id", meb_od_cross_obs_track_id, 2);
  JSON_DEBUG_VECTOR("meb_od_cross_obs_suppe_code", meb_od_cross_obs_suppe_code,
                    2);
  JSON_DEBUG_VECTOR("meb_od_cross_obs_rel_vx_vector",
                    meb_od_cross_obs_rel_vx_vector, 2);
  JSON_DEBUG_VECTOR("meb_od_cross_obs_rel_vy_vector",
                    meb_od_cross_obs_rel_vy_vector, 2);
  JSON_DEBUG_VECTOR("meb_od_cross_obs_type", meb_od_cross_obs_type, 2);
  JSON_DEBUG_VECTOR("meb_od_cross_obs_an_avoid_by_steering",
                    meb_od_cross_obs_an_avoid_by_steering, 2);
  JSON_DEBUG_VECTOR("meb_od_cross_obs_ay_avoid_by_accelerating",
                    meb_od_cross_obs_ay_avoid_by_accelerating, 2);
  JSON_DEBUG_VECTOR("meb_od_cross_obs_collision_point_y",
                    meb_od_cross_obs_collision_point_y, 2);

  std::vector<double> meb_uss_obs_x_vector = {};
  std::vector<double> meb_uss_obs_y_vector = {};
  std::vector<double> meb_uss_obs_stop_distance_buffer_vector = {};
  for (auto &obs :
       uss_straight_scenario_ptr_->collision_obj_info_.interest_obj_vec_) {
    meb_uss_obs_x_vector.push_back(obs.rel_x);
    meb_uss_obs_y_vector.push_back(obs.rel_y);
    meb_uss_obs_stop_distance_buffer_vector.push_back(obs.stop_distance_buffer);
  }

  JSON_DEBUG_VECTOR("meb_uss_obs_x_vector", meb_uss_obs_x_vector, 2);
  JSON_DEBUG_VECTOR("meb_uss_obs_y_vector", meb_uss_obs_y_vector, 2);
  JSON_DEBUG_VECTOR("meb_uss_obs_stop_distance_buffer_vector",
                    meb_uss_obs_stop_distance_buffer_vector, 2);

  std::vector<double> meb_occ_obs_x_vector = {};
  std::vector<double> meb_occ_obs_y_vector = {};
  std::vector<double> meb_occ_obs_stop_distance_buffer_vector = {};
  for (auto &obs :
       occ_straight_scenario_ptr_->collision_obj_info_.interest_obj_vec_) {
    meb_occ_obs_x_vector.push_back(obs.rel_x);
    meb_occ_obs_y_vector.push_back(obs.rel_y);
    meb_occ_obs_stop_distance_buffer_vector.push_back(obs.stop_distance_buffer);
  }

  JSON_DEBUG_VECTOR("meb_occ_obs_x_vector", meb_occ_obs_x_vector, 2);
  JSON_DEBUG_VECTOR("meb_occ_obs_y_vector", meb_occ_obs_y_vector, 2);
  JSON_DEBUG_VECTOR("meb_occ_obs_stop_distance_buffer_vector",
                    meb_occ_obs_stop_distance_buffer_vector, 2);
  JSON_DEBUG_VALUE("meb_od_straight_scene_code",
                   od_straight_scenario_ptr_->scene_code_);
  JSON_DEBUG_VALUE("meb_occ_straight_scene_code",
                   occ_straight_scenario_ptr_->scene_code_);
  JSON_DEBUG_VALUE("meb_uss_straight_scene_code",
                   uss_straight_scenario_ptr_->scene_code_);
  JSON_DEBUG_VALUE("meb_od_crossing_scene_code",
                   od_crossing_scenario_ptr_->scene_code_);
  JSON_DEBUG_VALUE("yaw_rate", MebInputInstacne.GetHistoryFrame(0)->yaw_rate);
  JSON_DEBUG_VALUE("yaw_rate_jerk",
                   MebInputInstacne.GetHistoryFrame(0)->yaw_rate_jerk);
  JSON_DEBUG_VALUE("driver_linear_state",
                   MebInputInstacne.GetHistoryFrame(0)->driver_linear_state);
  JSON_DEBUG_VALUE(
      "driver_linear_state_available",
      MebInputInstacne.GetHistoryFrame(0)->driver_linear_state_available);
  JSON_DEBUG_VALUE("drive_slow_down",
                   MebInputInstacne.GetHistoryFrame(0)->drive_slow_down);
  JSON_DEBUG_VALUE("start_turning_index",
                   MebInputInstacne.GetHistoryFrame(0)->start_turning_index);
  JSON_DEBUG_VALUE("turning_count",
                   MebInputInstacne.GetHistoryFrame(0)->turning_count);
  JSON_DEBUG_VALUE("straight_count",
                   MebInputInstacne.GetHistoryFrame(0)->straight_count);
  JSON_DEBUG_VALUE("need_update_backright",
                   MebInputInstacne.GetHistoryFrame(0)->need_update_backright);
  JSON_DEBUG_VALUE("meb_index_", (float64)meb_index_);
  JSON_DEBUG_VALUE(
      "state_steer_wheel_angle_degree_speed",
      GetContext.get_session()
              ->environmental_model()
              .get_local_view()
              .vehicle_service_output_info.steering_wheel_angle_speed *
          57.3);
  JSON_DEBUG_VALUE("function_state_machine",
                   (int)GetContext.get_session()
                       ->environmental_model()
                       .get_local_view()
                       .function_state_machine_info.current_state);
}

void MebCore::RunOnce(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  meb_index_++;
  // 每次更新输入
  adas_function::MebPreprocess::GetInstance().UpdateMebInput();

#if defined(ADAS_IN_SIMULATION)
  GetContext.mutable_param()->meb_false_trigger_switch = true;
#else
#endif
  // lane_follow_straight_scene--od
  od_straight_scenario_ptr_->Process();
  // straight_crossing_scene--od
  od_crossing_scenario_ptr_->Process();

  // 暂时不用
  if (GetContext.get_param()->meb_occ_obs_switch == true) {
    occ_straight_scenario_ptr_->Process();
  } else {
    occ_straight_scenario_ptr_->Init();
  }

  // 暂时不用
  if (GetContext.get_param()->meb_uss_obs_switch == true) {
    uss_straight_scenario_ptr_->Process();
  } else {
    uss_straight_scenario_ptr_->Init();
  }

  // for straight scene
  if ((od_straight_scenario_ptr_->final_collision_obj_info_.valid_num > 0) ||
      (occ_straight_scenario_ptr_->final_collision_obj_info_.valid_num > 0) ||
      uss_straight_scenario_ptr_->final_collision_obj_info_.valid_num > 0 ||
      od_crossing_scenario_ptr_->final_collision_obj_info_.valid_num > 0) {
    meb_intervention_flag_ = true;
  } else {
    meb_intervention_flag_ = false;
  }

  UpdateMebEnableCode();
  UpdateMebDisableCode();
  UpdateMebFaultCode();
  UpdateMebKickdownCode();
  UpdateMebSuppCode();
  MebStateMachine();
  SetMebOutputInfo();
  Log();
}

}  // namespace meb_core
}  // namespace adas_function