#include "tsr_core.h"

#include <iostream>

#include "adas_function_lib.h"
#include "planning_hmi_c.h"

namespace adas_function {
namespace tsr_core {

iflyauto::NotificationMainSwitch TsrCore::UpdateTsrMainSwitch(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                     ->mutable_environmental_model()
                                     ->get_local_view()
                                     .function_state_machine_info;
  if (GetContext.get_param()->tsr_main_switch == 2) {
    return iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_VISUAL_ONLY;
  } else if (GetContext.get_param()->tsr_main_switch == 3) {
    return iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_VISUAL_AND_AUDIO;
  } else if (GetContext.get_param()->tsr_main_switch == 1) {
    return iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_OFF;
  }
  return function_state_machine_info_ptr->switch_sts.tsr_main_switch;
}

uint16 TsrCore::UpdateTsrEnableCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint16 enable_code = 0;

  // bit 0
  // 判断是否为前进挡
  if (vehicle_service_output_info_ptr->shift_lever_state !=
      iflyauto::ShiftLeverStateEnum::ShiftLeverState_D) {
    enable_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // if (tsr_speed_limit_valid_ == false) {
  //   enable_code += uint16_bit[1];
  // } else {
  //   /*do nothing*/
  // }

  return enable_code;
}

uint16 TsrCore::UpdateTsrDisableCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint16 disable_code = 0;

  // bit 0
  // 判断是否为前进挡
  if (vehicle_service_output_info_ptr->shift_lever_state !=
      iflyauto::ShiftLeverStateEnum::ShiftLeverState_D) {
    disable_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // if (tsr_speed_limit_valid_ == false) {
  //   disable_code += uint16_bit[1];
  // } else {
  //   /*do nothing*/
  // }

  return disable_code;
}

uint16 TsrCore::UpdateTsrFaultCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint16 fault_code = 0;

  // bit 0
  // 判断挡位信号有效性
  if (vehicle_service_output_info_ptr->shift_lever_state_available == false) {
    fault_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // bit 1
  // 判断仪表车速信号有效性
  if (vehicle_service_output_info_ptr->vehicle_speed_display_available ==
      false) {
    fault_code += uint16_bit[1];
  } else {
    /*do nothing*/
  }

  return fault_code;
}

iflyauto::TSRFunctionFSMWorkState TsrCore::TsrStateMachine(void) {
  iflyauto::TSRFunctionFSMWorkState tsr_state;
  iflyauto::TSRFunctionFSMWorkState tsr_state_delay = tsr_state_;

  if (tsr_state_machine_init_flag_ == false) {
    // 状态机处于初始化状态 根据开关状态,决定第一个周期是输出OFF还是STANDBY
    tsr_state_machine_init_flag_ = true;
    if (tsr_main_switch_ == iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_OFF) {
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_OFF;
    } else {
      tsr_state = iflyauto::TSRFunctionFSMWorkState::
          TSR_FUNCTION_FSM_WORK_STATE_STANDBY;
    }
    return tsr_state;
  }

  // 状态机处于完成过初始化的状态
  if (tsr_state_delay == iflyauto::TSRFunctionFSMWorkState::
                             TSR_FUNCTION_FSM_WORK_STATE_UNAVAILABLE) {
    // 上一时刻处于TSR_FUNCTION_FSM_WORK_STATE_UNAVAILABLE状态
    if (tsr_main_switch_ == iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_OFF ||
        tsr_main_switch_ == iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_NONE) {
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_OFF;
    } else if (tsr_fault_code_ == 0) {
      tsr_state = iflyauto::TSRFunctionFSMWorkState::
          TSR_FUNCTION_FSM_WORK_STATE_STANDBY;
    } else {
      tsr_state = iflyauto::TSRFunctionFSMWorkState::
          TSR_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
    }
  } else if (tsr_state_delay == iflyauto::TSRFunctionFSMWorkState::
                                    TSR_FUNCTION_FSM_WORK_STATE_OFF) {
    // 上一时刻处于TSR_FUNCTION_FSM_WORK_STATE_OFF状态
    if (tsr_main_switch_ == iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_VISUAL_ONLY ||
        tsr_main_switch_ == iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_VISUAL_AND_AUDIO) {
      tsr_state = iflyauto::TSRFunctionFSMWorkState::
          TSR_FUNCTION_FSM_WORK_STATE_STANDBY;
    } else {
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_OFF;
    }
  } else if (tsr_state_delay == iflyauto::TSRFunctionFSMWorkState::
                                    TSR_FUNCTION_FSM_WORK_STATE_STANDBY) {
    // 上一时刻处于TSR_FUNCTION_FSM_WORK_STATE_STANDBY状态
    if (tsr_main_switch_ == iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_OFF ||
        tsr_main_switch_ == iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_NONE) {
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_OFF;
    } else if (tsr_fault_code_) {
      tsr_state = iflyauto::TSRFunctionFSMWorkState::
          TSR_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
    } else if (tsr_enable_code_ == 0) {
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_ACTIVE;
    } else {
      tsr_state = iflyauto::TSRFunctionFSMWorkState::
          TSR_FUNCTION_FSM_WORK_STATE_STANDBY;
    }
  } else if (tsr_state_delay == iflyauto::TSRFunctionFSMWorkState::
                                    TSR_FUNCTION_FSM_WORK_STATE_ACTIVE) {
    // 上一时刻处于TSR_FUNCTION_FSM_WORK_STATE_ACTIVE状态
    if (tsr_main_switch_ == iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_OFF ||
        tsr_main_switch_ == iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_NONE) {
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_OFF;
    } else if (tsr_fault_code_) {
      tsr_state = iflyauto::TSRFunctionFSMWorkState::
          TSR_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
    } else if (tsr_disable_code_) {
      tsr_state = iflyauto::TSRFunctionFSMWorkState::
          TSR_FUNCTION_FSM_WORK_STATE_STANDBY;
    } else {
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_ACTIVE;
    }
  } else {
    // 处于异常状态
    tsr_state =
        iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_OFF;
  }

  return tsr_state;
}

// 辅助标识牌优先级选择函数
adas_function::context::SuppSignInfo TsrCore::selectHighestPrioritySign(const std::vector<adas_function::context::SuppSignInfo>& signs) {
    adas_function::context::SuppSignInfo supplementary_sign_info;
    if (signs.empty()) {
        return supplementary_sign_info;
    }
    
    return *std::min_element(signs.begin(), signs.end(), 
        [](adas_function::context::SuppSignInfo a, adas_function::context::SuppSignInfo b) {
            return static_cast<int>(a.supp_sign_type) < static_cast<int>(b.supp_sign_type);
        }
    );
}

// 从 adas_function::context::SpeedSignType 到 iflyauto::SuppSignType 的转换函数
iflyauto::SuppSignType TsrCore::convertToIflySpeedSign(adas_function::context::SpeedSignType sign) {
    using Ifly = iflyauto::SuppSignType;
    using Ctx = adas_function::context::SpeedSignType;
    
    static const std::unordered_map<Ctx, Ifly> mapping = {
        {Ctx::SPEED_SIGN_TYPE_MAXIMUM_SPEED, Ifly::SUPP_SIGN_TYPE_MAXIMUM_SPEED},
        {Ctx::SPEED_SIGN_TYPE_MINIMUM_SPEED, Ifly::SUPP_SIGN_TYPE_MINIMUM_SPEED},
        {Ctx::SPEED_SIGN_TYPE_END_OF_SPEED_LIMIT, Ifly::SUPP_SIGN_TYPE_END_OF_SPEED_LIMIT},
        {Ctx::SPEED_SIGN_TYPE_UNKNOWN, Ifly::SUPP_SIGN_TYPE_UNKNOWN},
    };
    
    auto it = mapping.find(sign);
    return it != mapping.end() ? it->second : Ifly::SUPP_SIGN_TYPE_UNKNOWN;
}

// 从 adas_function::context 到 iflyauto 的转换函数
iflyauto::SuppSignType TsrCore::convertToIflySuppSign(adas_function::context::SuppSignType sign) {
    using Ifly = iflyauto::SuppSignType;
    using Ctx = adas_function::context::SuppSignType;
    
    // 反向映射
    static const std::unordered_map<Ctx, Ifly> mapping = {
        {Ctx::SUPP_SIGN_TYPE_YIELD_SIGN, Ifly::SUPP_SIGN_TYPE_YIELD_SIGN},
        {Ctx::SUPP_SIGN_TYPE_STOP_SIGN, Ifly::SUPP_SIGN_TYPE_STOP_SIGN},
        {Ctx::SUPP_SIGN_TYPE_PROHIBIT_PROLONGED_PARKING, Ifly::SUPP_SIGN_TYPE_PROHIBIT_PROLONGED_PARKING},
        {Ctx::SUPP_SIGN_TYPE_NO_PARKING, Ifly::SUPP_SIGN_TYPE_NO_PARKING},
        {Ctx::SUPP_SIGN_TYPE_NO_OVERTAKING, Ifly::SUPP_SIGN_TYPE_NO_OVERTAKING},
        {Ctx::SUPP_SIGN_TYPE_CANCEL_NO_OVERTAKING, Ifly::SUPP_SIGN_TYPE_CANCEL_NO_OVERTAKING},
        {Ctx::SUPP_SIGN_TYPE_NO_ENTRY, Ifly::SUPP_SIGN_TYPE_NO_ENTRY},
        {Ctx::SUPP_SIGN_TYPE_PROHIBIT_MOTOR_ENTERING, Ifly::SUPP_SIGN_TYPE_PROHIBIT_MOTOR_ENTERING},
        {Ctx::SUPP_SIGN_TYPE_PROHIBIT_TURN_U, Ifly::SUPP_SIGN_TYPE_PROHIBIT_TURN_U},
        {Ctx::SUPP_SIGN_TYPE_PROHIBIT_TURN_RIGHT, Ifly::SUPP_SIGN_TYPE_PROHIBIT_TURN_RIGHT},
        {Ctx::SUPP_SIGN_TYPE_PROHIBIT_TURN_LEFT, Ifly::SUPP_SIGN_TYPE_PROHIBIT_TURN_LEFT},
        {Ctx::SUPP_SIGN_TYPE_NO_PASSING, Ifly::SUPP_SIGN_TYPE_NO_PASSING},
        {Ctx::SUPP_SIGN_TYPE_UNKNOWN, Ifly::SUPP_SIGN_TYPE_UNKNOWN},
    };
    
    auto it = mapping.find(sign);
    return it != mapping.end() ? it->second : Ifly::SUPP_SIGN_TYPE_UNKNOWN;
}

// 更新辅助标识牌信息, 不包含限速标识牌
void TsrCore::UpdateTsrSuppInfo(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  // 车辆状态信息
  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;
  // 判断辅助标识牌是否需要抑制显示
  // TODO: thzhang5 0630 yawrate可以写为配置参数
  if (supp_sign_in_suppression_flag_ == false && std::abs(vehicle_service_output_info_ptr->yaw_rate) > 6.5 * M_PI / 180.0) {
    supp_sign_in_suppression_flag_ = true;
  } else if (supp_sign_in_suppression_flag_ == true && std::abs(vehicle_service_output_info_ptr->yaw_rate) < 5 * M_PI / 180.0) {
    supp_sign_in_suppression_flag_ = false;
  }

  // 重置实时辅助标识牌信息
  realtime_supp_sign_info_ = iflyauto::SuppSignType::SUPP_SIGN_TYPE_UNKNOWN;

  if (!supp_sign_in_suppression_flag_) {
    // 道路标识信息
    auto tsr_info_ptr = GetContext.get_tsr_info();
    auto &supp_sign_info_vector = tsr_info_ptr->supp_sign_info_vector;
    for (auto &supp_sign_info : supp_sign_info_vector) {
      // 判断是否有辅助标识牌在范围内(前15,左10)
      if (supp_sign_info.supp_sign_y <= 0.0 && supp_sign_info.supp_sign_y >= -10.0 && supp_sign_info.supp_sign_x >= 0.0 && supp_sign_info.supp_sign_x <= 15.0) {
        supp_sign_info_vector_.emplace_back(supp_sign_info);
      }
    }
  }
  // 候选辅助标识牌信息排序
  if (supp_sign_info_vector_.size() > 0) {
    auto highest_priority_sign = selectHighestPrioritySign(supp_sign_info_vector_);
    if (highest_priority_sign.supp_sign_type != adas_function::context::SuppSignType::SUPP_SIGN_TYPE_UNKNOWN) {
      // 实时辅助标识牌类型
      realtime_supp_sign_info_ = convertToIflySuppSign(highest_priority_sign.supp_sign_type);
    }
  }
  
  // 更新输出辅助标识牌信息
  if (realtime_supp_sign_info_ != iflyauto::SuppSignType::SUPP_SIGN_TYPE_UNKNOWN) {
    // 检测到辅助标识牌
    if (realtime_supp_sign_info_ != output_supp_sign_info_) {
      // 检测到新的辅助标识牌类型
      output_supp_sign_info_ = realtime_supp_sign_info_;
      supp_sign_change_flag_ = true;
      supp_sign_valid_flag_ = true;
    } else if (supp_sign_valid_flag_ == false) {
      // 相同标识牌重新出现，重置计时器
      supp_sign_change_flag_ = true;
      supp_sign_valid_flag_ = true;
    }
  }
  return;
}

void TsrCore::UpdateTsrSpeedLimit(void) {
  // 需要从规划拿数据
  // 奇瑞要求功能关闭时也显示限速值,只是不会报警
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto peception_tsr_info = GetContext.get_session()
                                ->environmental_model()
                                .get_local_view()
                                .perception_tsr_info;
  auto supp_signs_array = peception_tsr_info.supp_signs;
  int supp_signs_array_size = peception_tsr_info.supp_signs_size;
  bool sign_vald = false;
  uint32 lane_speed_limit_value = tsr_speed_limit_;
  uint32 tsr_speed_limit_last = tsr_speed_limit_;
  for (int i = 0; i < supp_signs_array_size; i++) {
    auto single_sign = supp_signs_array[i];
    if (single_sign.supp_sign_type != iflyauto::SuppSignType::SUPP_SIGN_TYPE_MAXIMUM_SPEED) {
      continue;
    } else {
      if (sign_vald == false) {
        lane_speed_limit_value = single_sign.speed_limit;
        sign_vald = true;
        continue;
      } else {
        if (single_sign.speed_limit > lane_speed_limit_value) {
          lane_speed_limit_value = single_sign.speed_limit;
        }
      }
    }
  }
  bool speed_limit_exist_in_view_flag_last = speed_limit_exist_in_view_flag_;
  uint32 speed_limit_exist_in_view_last = speed_limit_exist_in_view_;
  if (sign_vald == true) {
    speed_limit_exist_in_view_flag_ = true;
    speed_limit_exist_in_view_ = lane_speed_limit_value;
    // tsr_speed_limit_ = lane_speed_limit_value;
    // tsr_speed_limit_valid_ = true;
  } else {
    speed_limit_exist_in_view_flag_ = false;
    speed_limit_exist_in_view_ = 0;
  }

  if (speed_limit_exist_in_view_flag_last == true &&
      speed_limit_exist_in_view_flag_ == false) {
    // 通过限速标志消失与否判断车辆是否驶入限速路段
    if (tsr_speed_limit_valid_ == false) {
      tsr_speed_limit_ = speed_limit_exist_in_view_last;
    } else {
      if (speed_limit_exist_in_view_last != tsr_speed_limit_) {
        tsr_speed_limit_ = speed_limit_exist_in_view_last;
      }
    }
    tsr_speed_limit_valid_ = true;
  } else if (speed_limit_exist_in_view_flag_ == true) {
    // 当标志牌一直存在时，如果限速值存在变化时，判断进入了限速路段
    if ((speed_limit_exist_in_view_last != speed_limit_exist_in_view_) &&
        (speed_limit_exist_in_view_flag_last == true)) {
      tsr_speed_limit_ = speed_limit_exist_in_view_last;
      tsr_speed_limit_valid_ = true;
    }
  } else {
    /*do nothing*/
  }

  if (tsr_speed_limit_valid_ == false) {
    tsr_speed_limit_ = 0;
  }
  if (tsr_speed_limit_valid_ == true) {
    if (tsr_speed_limit_last != tsr_speed_limit_) {
      tsr_speed_limit_change_flag_ = true;
    }
  }
  if (accumulated_path_length_ >
      GetContext.get_param()->tsr_reset_path_length) {
    tsr_speed_limit_valid_ = false;
    tsr_speed_limit_ = 0;
  }
  return;
}

// 获取限速标识牌中的最高限速值
uint32 TsrCore::GetHighestSpeedLimit(void) {
  uint32 highest_speed_limit = 0;
  
  if (speed_limit_sign_info_vector_.empty()) {
    return highest_speed_limit;
  }
  
  for (const auto& speed_sign_info : speed_limit_sign_info_vector_) {
    if (speed_sign_info.speed_limit > highest_speed_limit) {
      highest_speed_limit = speed_sign_info.speed_limit;
    }
  }
  
  return highest_speed_limit;
}

// 采用tsr的限速标识牌信息更新限速信息
void TsrCore::UpdateTsrSpeedLimitNew(void) {
  // 完成限速更新标志位
  bool speed_limit_update_flag = false;
  // 上一次限速信息
  uint32 last_tsr_speed_limit = tsr_speed_limit_;
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  // 车辆状态信息
  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;
  // 判断限速标识牌是否需要抑制显示
  // TODO: thzhang5 0630 yawrate可以写为配置参数
  if (speed_limit_suppression_flag_ == false && std::abs(vehicle_service_output_info_ptr->yaw_rate) > 10 * M_PI / 180.0) {
    speed_limit_suppression_flag_ = true;
  } else if (speed_limit_suppression_flag_ == true && std::abs(vehicle_service_output_info_ptr->yaw_rate) < 1 * M_PI / 180.0) {
    speed_limit_suppression_flag_ = false;
    // 完成转弯, 则进入新道路
    new_road_flag_ = true;
  }
  if (new_road_flag_ == true) {
    // 进入新道路, 则清掉视觉限速信息
    speed_limit_exist_in_view_flag_ = false;
    speed_limit_exist_in_view_ = 0;
    new_road_flag_ = false;
    speed_limit_update_flag = true;
  }
  // 未完成限速更新, 则更新限速信息
  if (!speed_limit_update_flag) {
    // 自车实际车速, m/s->km/h
    double vehicle_speed_display = vehicle_service_output_info_ptr->vehicle_speed_display * 3.6;
    speed_limit_sign_info_vector_.clear();
    end_of_speed_sign_info_vector_.clear();
    auto tsr_info_ptr = GetContext.get_tsr_info();
    auto &speed_sign_info_vector = tsr_info_ptr->speed_sign_info_vector;
    // 限速和解除限速信息分别存储
    for (auto &speed_sign_info : speed_sign_info_vector) {
      if (speed_sign_info.speed_sign_type == adas_function::context::SpeedSignType::SPEED_SIGN_TYPE_MAXIMUM_SPEED) {
        // 远处限速感知不一定准确, 只取前方50m内的限速信息
        if (speed_sign_info.supp_sign_x >= 0.0 && speed_sign_info.supp_sign_x <= 50.0) {
          // 判断是否是匝道限速牌, 速度与实车速度差距过大, 或者车辆打右转灯, 且横向距离小于8m
          if (vehicle_speed_display - speed_sign_info.speed_limit > 20.0) {
            // 速度与实车速度差距过大, 不更新限速信息
            continue;
          }
          else if (vehicle_service_output_info_ptr->right_turn_light_state == true) {
            // 车辆打右转灯, 且横向距离小于8m, 不更新限速信息
            if (speed_sign_info.supp_sign_y < 0.0 && speed_sign_info.supp_sign_y > -8.0) {
              continue;
            }
          }
          if (speed_sign_info.ramp_flag == true) {
            // 匝道限速牌, 不更新限速信息 (如果感知有的话)
            continue;
          }
          speed_limit_sign_info_vector_.emplace_back(speed_sign_info);
        }
      }
      if (speed_sign_info.speed_sign_type == adas_function::context::SpeedSignType::SPEED_SIGN_TYPE_END_OF_SPEED_LIMIT) {
        // 解除限速信息, 只取前方50m内的解除限速信息
        if (speed_sign_info.supp_sign_x >= 0.0 && speed_sign_info.supp_sign_x <= 15.0) {
          end_of_speed_sign_info_vector_.emplace_back(speed_sign_info);
        }
      }
    }
    // 如果当前限速和解除限速速度一致, 且道路限速信息有效,则取道路限速信息
    for (auto &end_of_speed_sign_info : end_of_speed_sign_info_vector_) {
      if (current_map_speed_limit_valid_ == true && speed_limit_exist_in_view_ == end_of_speed_sign_info.speed_limit) {
        tsr_speed_limit_ = current_map_speed_limit_;
      }
    }
    // 获取限速标识牌中的最高限速值
    uint32 current_tsr_speed_limit = GetHighestSpeedLimit();
    if (current_tsr_speed_limit > 0) {
      // 限速信息有效
      tsr_speed_limit_valid_ = true;
      tsr_speed_limit_ = current_tsr_speed_limit;
    } else {
      // 限速信息无效
      tsr_speed_limit_valid_ = false;
      tsr_speed_limit_ = 0;
    }
    if (last_tsr_speed_limit != tsr_speed_limit_) {
      tsr_speed_limit_change_flag_ = true;
    }
  }
  if (accumulated_path_length_ > GetContext.get_param()->tsr_reset_path_length) {
    tsr_speed_limit_valid_ = false;
    tsr_speed_limit_ = 0;
  }
  return;
}

void TsrCore::UpdateTsrWarning(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  // 更新overspeed_status_
  if (((vehicle_service_output_info_ptr->vehicle_speed_display * 3.6) >
       tsr_speed_limit_) &&
      (tsr_speed_limit_valid_ == true)) {
    overspeed_status_ = true;
  } else {
    overspeed_status_ = false;
  }

  // 更新overspeed_duration_time_
  if (overspeed_status_) {
    overspeed_duration_time_ += GetContext.get_param()->dt;
  } else {
    overspeed_duration_time_ = 0.0;
  }

  // 更新tsr_warning_image_
  if (overspeed_duration_time_ >= 1.5) {
    tsr_warning_image_ = true;
  } else {
    tsr_warning_image_ = false;
  }

  // 发起声音报警需要的确认时长
  double image2voice_confirm_time;
  if ((vehicle_service_output_info_ptr->vehicle_speed_display * 3.6) >=
      (tsr_speed_limit_ * 1.3)) {
    image2voice_confirm_time = 3.0;
  } else if ((vehicle_service_output_info_ptr->vehicle_speed_display * 3.6) >=
             (tsr_speed_limit_ * 1.2)) {
    image2voice_confirm_time = 4.0;
  } else if ((vehicle_service_output_info_ptr->vehicle_speed_display * 3.6) >=
             (tsr_speed_limit_ * 1.1)) {
    image2voice_confirm_time = 5.0;
  } else if ((vehicle_service_output_info_ptr->vehicle_speed_display * 3.6) >=
             (tsr_speed_limit_ * 1.0)) {
    image2voice_confirm_time = 6.0;
  } else {
    image2voice_confirm_time = 6.0;
  }

  // 更新tsr_warning_voice_
  if (tsr_warning_voice_ == false) {
    // 上一时刻未发起声音报警
    if (overspeed_duration_time_ >= image2voice_confirm_time) {
      tsr_warning_voice_ = true;
    } else {
      tsr_warning_voice_ = false;
    }
  } else {
    // do nothing
  }
  // 功能未激活时取消声音报警
  if (tsr_state_ !=
      iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_ACTIVE) {
    tsr_warning_voice_ = false;
  } else {
    // do nothing
  }
  // 未超速时取消声音报警
  if (overspeed_status_ == false) {
    tsr_warning_voice_ = false;
  } else {
    // do nothing
  }
  // 声音报警超过最大时长时取消声音报警
  if (overspeed_duration_time_ >= (image2voice_confirm_time + 5.0)) {
    tsr_warning_voice_ = false;
  } else {
    // do nothing
  }
  // 驾驶员未踩下油门踏板时取消声音报警
  if ((vehicle_service_output_info_ptr->accelerator_pedal_pos < 1.0) &&
      vehicle_service_output_info_ptr->accelerator_pedal_pos_available) {
    tsr_warning_voice_ = false;
  } else {
    // do nothing
  }
  // 驾驶员踩下刹车踏板时取消声音报警
  if (vehicle_service_output_info_ptr->brake_pedal_pressed_available &&
      vehicle_service_output_info_ptr->brake_pedal_pressed) {
    tsr_warning_voice_ = false;
  } else {
    // do nothing
  }
  if (tsr_speed_limit_change_flag_ == true) {
    tsr_speed_limit_change_flag_ = false;
  }
  return;
}

void TsrCore::CalculatePathLengthAccumulated() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  if (tsr_speed_limit_change_flag_ == true || tsr_speed_limit_valid_ == false) {
    accumulated_path_length_ = 0.0;
  } else {
    accumulated_path_length_ =
        accumulated_path_length_ +
        GetContext.get_state_info()->vehicle_speed * GetContext.get_param()->dt;
  }
  // accumulated_path_length_ = accumulated_path_length_;
  return;
}

// 计算当前显示的辅助标识牌的时间是否大于2s
void TsrCore::CalculateDurationTime(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  // 如果输出辅助路标发生变化, 则重置辅助标识牌保持时间计数器
  if (supp_sign_change_flag_) {
    supp_sign_hold_time_ = 0.0;
    supp_sign_change_flag_ = false;
  } else {
    supp_sign_hold_time_ += GetContext.get_param()->dt;
  }
  return;
}

void TsrCore::SetTsrOutputInfo() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  GetContext.mutable_output_info()->tsr_output_info_.tsr_state_ = tsr_state_;
  if (tsr_state_ == iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_ACTIVE) 
  // if (1)
  {
    // 如果没有有效的限速信息, 则不输出限速信息
    if (tsr_speed_limit_valid_ == false && current_map_speed_limit_ > 0) {
      GetContext.mutable_output_info()->tsr_output_info_.tsr_speed_limit_ = current_map_speed_limit_;
    } else {
      // 输出感知限速信息
      GetContext.mutable_output_info()->tsr_output_info_.tsr_speed_limit_ = tsr_speed_limit_;
    }
    GetContext.mutable_output_info()->tsr_output_info_.tsr_warning_ =
        tsr_warning_image_;
    // 辅助标识牌抑制显示时，不输出辅助标识牌
    if (supp_sign_in_suppression_flag_ == false) {
      GetContext.mutable_output_info()->tsr_output_info_.supp_sign_type = output_supp_sign_info_;
      std::cout << "output_supp_sign_info_ ========== " << (int)output_supp_sign_info_ << std::endl;
    } else {
      GetContext.mutable_output_info()->tsr_output_info_.supp_sign_type = iflyauto::SuppSignType::SUPP_SIGN_TYPE_UNKNOWN;
    }
  } else {
    GetContext.mutable_output_info()->tsr_output_info_.tsr_warning_ = false;
    GetContext.mutable_output_info()->tsr_output_info_.tsr_speed_limit_ = 0;
    GetContext.mutable_output_info()->tsr_output_info_.supp_sign_type = iflyauto::SuppSignType::SUPP_SIGN_TYPE_UNKNOWN;
  }
  return;
}

void TsrCore::ResetRealTimeTsrInfo(void) {
  supp_sign_info_vector_.clear();
  
  // 如果没有检测到实时辅助标识牌，且当前有输出，且持续时间超过2s，则清空输出
  if (realtime_supp_sign_info_ == iflyauto::SuppSignType::SUPP_SIGN_TYPE_UNKNOWN && 
      supp_sign_valid_flag_ == true && 
      supp_sign_hold_time_ > 2.0) {
    output_supp_sign_info_ = iflyauto::SuppSignType::SUPP_SIGN_TYPE_UNKNOWN;
    supp_sign_valid_flag_ = false;
    supp_sign_hold_time_ = 0.0;
  }
  return;
}

void TsrCore::RunOnce(void) {
  // 更新tsr开关状态
  tsr_main_switch_ = UpdateTsrMainSwitch();

  // 更新tsr_enable_code_
  tsr_enable_code_ = UpdateTsrEnableCode();

  // 更新tsr_disable_code_
  tsr_disable_code_ = UpdateTsrDisableCode();

  // 更新tsr_fault_code_
  tsr_fault_code_ = UpdateTsrFaultCode();

  // 更新tsr_state_
  tsr_state_ = TsrStateMachine();

  // 更新实时辅助标识牌信息
  UpdateTsrSuppInfo();

  // 更新tsr_speed_limit_
  UpdateTsrSpeedLimit();

  // 计算accumulated_path_length_
  CalculatePathLengthAccumulated();

  // 计算辅助标识牌保持时间
  CalculateDurationTime();

  // 更新tsr_warning_image_&&tsr_warning_voice_
  UpdateTsrWarning();

  //output
  SetTsrOutputInfo();

  // log
  JSON_DEBUG_VALUE("tsr_main_switch_", (int)tsr_main_switch_);
  JSON_DEBUG_VALUE("tsr_enable_code_", tsr_enable_code_);
  JSON_DEBUG_VALUE("tsr_disable_code_", tsr_disable_code_);
  JSON_DEBUG_VALUE("tsr_fault_code_", tsr_fault_code_);
  JSON_DEBUG_VALUE("tsr_state_", (int)tsr_state_);
  JSON_DEBUG_VALUE("tsr_speed_limit_", tsr_speed_limit_);

  JSON_DEBUG_VALUE("tsr_speed_limit_valid_", tsr_speed_limit_valid_);
  JSON_DEBUG_VALUE("tsr_warning_image_", tsr_warning_image_);
  JSON_DEBUG_VALUE("tsr_warning_voice_", tsr_warning_voice_);
  JSON_DEBUG_VALUE("tsr_overspeed_status_", overspeed_status_);
  JSON_DEBUG_VALUE("tsr_overspeed_duration_time_", overspeed_duration_time_);
  JSON_DEBUG_VALUE("tsr_speed_limit_change_flag_",
                   tsr_speed_limit_change_flag_);
  JSON_DEBUG_VALUE("tsr_speed_limit_exist_in_view_flag_",
                   speed_limit_exist_in_view_flag_);
  JSON_DEBUG_VALUE("tsr_speed_limit_exist_in_view_",
                   speed_limit_exist_in_view_);
  JSON_DEBUG_VALUE("tsr_accumulated_path_length_", accumulated_path_length_);
  JSON_DEBUG_VALUE("tsr_output_supp_sign_info_", (int)output_supp_sign_info_);

  // reset info
  ResetRealTimeTsrInfo();
}

}  // namespace tsr_core
}  // namespace adas_function