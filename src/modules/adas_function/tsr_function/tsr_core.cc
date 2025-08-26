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
  if (GetContext.get_param()->tsr_main_switch == iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_VISUAL_ONLY) {
    return iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_VISUAL_ONLY;
  } else if (GetContext.get_param()->tsr_main_switch == iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_VISUAL_AND_AUDIO) {
    return iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_VISUAL_AND_AUDIO;
  } else if (GetContext.get_param()->tsr_main_switch == iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_OFF) {
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
  // if (vehicle_service_output_info_ptr->shift_lever_state !=
  //     iflyauto::ShiftLeverStateEnum::ShiftLeverState_D) {
  //   enable_code += uint16_bit[0];
  // } else {
  //   /*do nothing*/
  // }

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
  // if (vehicle_service_output_info_ptr->shift_lever_state !=
  //     iflyauto::ShiftLeverStateEnum::ShiftLeverState_D) {
  //   disable_code += uint16_bit[0];
  // } else {
  //   /*do nothing*/
  // }

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
    // 重置辅助标识牌代码
    supp_sign_code_ = 0;
    
    for (auto &supp_sign_info : supp_sign_info_vector) {
      // 判断是否有辅助标识牌在范围内(前15,左10)
      if (supp_sign_info.supp_sign_y <= 0.0 && supp_sign_info.supp_sign_y >= -10.0 && supp_sign_info.supp_sign_x >= 0.0 && supp_sign_info.supp_sign_x <= 15.0) {
        // 根据标识牌类型设置对应的位
        switch (supp_sign_info.supp_sign_type) {
          case iflyauto::SuppSignType::SUPP_SIGN_TYPE_YIELD_SIGN:
            supp_sign_code_ |= (1 << 0);  // bit 0
            break;
          case iflyauto::SuppSignType::SUPP_SIGN_TYPE_STOP_SIGN:
            supp_sign_code_ |= (1 << 1);  // bit 1
            break;
          case iflyauto::SuppSignType::SUPP_SIGN_TYPE_NO_STOPPING:
            supp_sign_code_ |= (1 << 2);  // bit 2
            break;
          case iflyauto::SuppSignType::SUPP_SIGN_TYPE_PROHIBIT_PROLONGED_PARKING:
            supp_sign_code_ |= (1 << 3);  // bit 3
            break;
          case iflyauto::SuppSignType::SUPP_SIGN_TYPE_NO_PARKING:
            supp_sign_code_ |= (1 << 4);  // bit 4
            break;
          case iflyauto::SuppSignType::SUPP_SIGN_TYPE_NO_OVERTAKING:
            supp_sign_code_ |= (1 << 5);  // bit 5
            break;
          case iflyauto::SuppSignType::SUPP_SIGN_TYPE_CANCEL_NO_OVERTAKING:
            supp_sign_code_ |= (1 << 6);  // bit 6
            break;
          case iflyauto::SuppSignType::SUPP_SIGN_TYPE_NO_ENTRY:
            supp_sign_code_ |= (1 << 7);  // bit 7
            break;
          case iflyauto::SuppSignType::SUPP_SIGN_TYPE_PROHIBIT_MOTOR_ENTERING:
            supp_sign_code_ |= (1 << 8);  // bit 8
            break;
          case iflyauto::SuppSignType::SUPP_SIGN_TYPE_PROHIBIT_TURN_U:
            supp_sign_code_ |= (1 << 9);  // bit 9
            break;
          case iflyauto::SuppSignType::SUPP_SIGN_TYPE_PROHIBIT_TURN_RIGHT:
            supp_sign_code_ |= (1 << 10);  // bit 10
            break;
          case iflyauto::SuppSignType::SUPP_SIGN_TYPE_PROHIBIT_TURN_LEFT:
            supp_sign_code_ |= (1 << 11); // bit 11
            break;
          case iflyauto::SuppSignType::SUPP_SIGN_TYPE_NO_PASSING:
            supp_sign_code_ |= (1 << 12); // bit 12
            break;
          default:
            // 其他类型不处理
            break;
        }
      }
    }
  }
  // 候选辅助标识牌信息排序
  if (supp_sign_code_ > 0) {
    // 根据优先级定义标识牌类型对应的位数组（从低位到高位）
    static const iflyauto::SuppSignType supp_sign_priority_array[] = {
      iflyauto::SuppSignType::SUPP_SIGN_TYPE_YIELD_SIGN,                 // bit 0
      iflyauto::SuppSignType::SUPP_SIGN_TYPE_STOP_SIGN,                  // bit 1
      iflyauto::SuppSignType::SUPP_SIGN_TYPE_NO_STOPPING,                // bit 2
      iflyauto::SuppSignType::SUPP_SIGN_TYPE_PROHIBIT_PROLONGED_PARKING,  // bit 3
      iflyauto::SuppSignType::SUPP_SIGN_TYPE_NO_PARKING,                 // bit 4
      iflyauto::SuppSignType::SUPP_SIGN_TYPE_NO_OVERTAKING,              // bit 5
      iflyauto::SuppSignType::SUPP_SIGN_TYPE_CANCEL_NO_OVERTAKING,       // bit 6
      iflyauto::SuppSignType::SUPP_SIGN_TYPE_NO_ENTRY,                   // bit 7
      iflyauto::SuppSignType::SUPP_SIGN_TYPE_PROHIBIT_MOTOR_ENTERING,     // bit 8
      iflyauto::SuppSignType::SUPP_SIGN_TYPE_PROHIBIT_TURN_U,            // bit 9
      iflyauto::SuppSignType::SUPP_SIGN_TYPE_PROHIBIT_TURN_RIGHT,        // bit 10
      iflyauto::SuppSignType::SUPP_SIGN_TYPE_PROHIBIT_TURN_LEFT,         // bit 11
      iflyauto::SuppSignType::SUPP_SIGN_TYPE_NO_PASSING,                 // bit 12
    };
    
    // 查找第一个从低到高位数为1的位子所对应的标识牌类型
    for (int i = 0; i < 13; i++) {
      if (supp_sign_code_ & (1 << i)) {
        realtime_supp_sign_info_ = supp_sign_priority_array[i];
        break;
      }
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

// 更新限速信息
void TsrCore::UpdateTsrSpeedLimit(void) {
  // 需要从规划拿数据
  // 奇瑞要求功能关闭时也显示限速值,只是不会报警
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto peception_tsr_info = GetContext.get_session()
                                ->environmental_model()
                                .get_local_view()
                                .perception_tsr_info;
  // 车辆状态信息
  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;
  // sd_map信息，获取限速
  if (GetContext.get_session()->environmental_model()
                                .get_route_info()
                                ->get_sdpromap_valid()) {
    const auto &sd_pro_map_info_ptr = GetContext.get_session()
                                  ->environmental_model()
                                  .get_route_info()
                                  ->get_sdpro_map();
    
    // 获取当前道路限速值
    // ego_motion信息
    auto localization_info = GetContext.mutable_session()->mutable_environmental_model()->
                              get_ego_state_manager(); // enu实际上是boot
    ad_common::math::Vec2d current_point;
    current_point.set_x(localization_info->location_enu().position.x);
    current_point.set_y(localization_info->location_enu().position.y); // enu实际上是boot
    const double search_distance = 50.0;
    const double max_heading_diff = PI / 4;
    double temp_nearest_s = 0;
    double nearest_l = 0;
    const double ego_heading_angle = localization_info->heading_angle();
    const iflymapdata::sdpro::LinkInfo_Link* current_link = sd_pro_map_info_ptr.GetNearestLinkWithHeading(
        current_point, search_distance, ego_heading_angle, max_heading_diff,
        temp_nearest_s, nearest_l);
    if (!current_link) {
      current_map_speed_limit_valid_ = false;
      current_map_speed_limit_ = 0;
    } else {
      current_map_speed_limit_ = current_link->speed_limit();
      current_map_speed_limit_valid_ = true;
    }
  } else {
    current_map_speed_limit_valid_ = false;
    current_map_speed_limit_ = 0;
  }

  // 判断限速标识牌是否需要抑制显示
  if (speed_limit_suppression_flag_ == false && std::abs(vehicle_service_output_info_ptr->yaw_rate) > 10 * M_PI / 180.0) {
    speed_limit_suppression_flag_ = true;
  } else if (speed_limit_suppression_flag_ == true && std::abs(vehicle_service_output_info_ptr->yaw_rate) < 3 * M_PI / 180.0) {
    speed_limit_suppression_flag_ = false;
    // 完成转弯, 则进入新道路
    new_road_flag_ = true;
  }
  
  // 处理新道路标志 - 移到提前返回之前
  if (new_road_flag_ == true) {
    // 进入新道路, 则清掉视觉限速信息与视觉解除限速信息, 采用地图限速信息
    speed_limit_exist_in_view_flag_ = false;
    speed_limit_exist_in_view_ = 0;
    tsr_speed_limit_valid_ = false;
    tsr_speed_limit_ = current_map_speed_limit_;
    end_of_speed_sign_value_ = 0;
    end_of_speed_sign_display_time_ = 0.0;
    end_of_speed_sign_display_flag_ = false;
    new_road_flag_ = false;
  }

  if (speed_limit_suppression_flag_ == true) {
    // 抑制限速结果变更
    return;
  }

  auto supp_signs_array = peception_tsr_info.supp_signs;
  int supp_signs_array_size = peception_tsr_info.supp_signs_size;
  bool sign_vald = false; // 是否实时感知到限速牌
  uint32 lane_speed_limit_value = tsr_speed_limit_;
  uint32 tsr_speed_limit_last = tsr_speed_limit_;
  bool end_of_speed_sign_vald = false; // 是否实时感知到解除限速牌
  // 获取自车表显速度 (km/h)
  double ego_speed_kph = vehicle_service_output_info_ptr->vehicle_speed_display * 3.6;
  
  for (int i = 0; i < supp_signs_array_size; i++) {
    auto single_sign = supp_signs_array[i];
    // 只取限速相关标识牌
    if (single_sign.supp_sign_type != iflyauto::SuppSignType::SUPP_SIGN_TYPE_MAXIMUM_SPEED && 
        single_sign.supp_sign_type != iflyauto::SuppSignType::SUPP_SIGN_TYPE_END_OF_SPEED_LIMIT) {
      continue;
    } else {
      if (single_sign.supp_sign_type == iflyauto::SuppSignType::SUPP_SIGN_TYPE_MAXIMUM_SPEED) {
        // 使用自车表显速度判断感知限速牌真实性
        bool speed_limit_valid = false;
        if (ego_speed_kph < 30.0) {
          // 自车车速＜30kph，任意限速标志都可以接受
          speed_limit_valid = true;
        } else if (ego_speed_kph >= 30.0 && ego_speed_kph < 40.0) {
          // 30kph≤自车车速＜40kph，限速标志的限速值>10kph才可以接受
          speed_limit_valid = (single_sign.speed_limit > 10);
        } else if (ego_speed_kph >= 40.0 && ego_speed_kph <= 80.0) {
          // 40kph≤自车车速≤80kph，限速标志的限速值>10kph才可以接受
          speed_limit_valid = (single_sign.speed_limit > 10);
        } else {
          // 80kph＜自车车速，限速标志的限速值>40kph才可以接受
          speed_limit_valid = (single_sign.speed_limit > 40);
        }
        
        if (!speed_limit_valid) {
          // 限速标志不符合真实性判断，跳过此标志
          continue;
        }
        
        // 限速标识,取最大限速值
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
      if (single_sign.supp_sign_type == iflyauto::SuppSignType::SUPP_SIGN_TYPE_END_OF_SPEED_LIMIT) {
        // 解除限速标识, 取最大解除限速值
        if (end_of_speed_sign_vald == false) {
          end_of_speed_sign_value_ = single_sign.speed_limit;
          end_of_speed_sign_vald = true;
          continue;
        } else {
          if (single_sign.speed_limit > end_of_speed_sign_value_) {
            end_of_speed_sign_value_ = single_sign.speed_limit;
          }
        }
      }
    }
  }
  bool speed_limit_exist_in_view_flag_last = speed_limit_exist_in_view_flag_;
  uint32 speed_limit_exist_in_view_last = speed_limit_exist_in_view_;
  if (sign_vald == true) {
    // 还未驶过限速标识
    speed_limit_exist_in_view_flag_ = true;
    speed_limit_exist_in_view_ = lane_speed_limit_value;
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
      // 如果视觉限速信息还存在，且有新的限速感知，则采用新的限速感知
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
    // 没有视觉限速有效值, 采用地图限速信息
    tsr_speed_limit_ = current_map_speed_limit_;
  } else {
    if (tsr_speed_limit_last != tsr_speed_limit_) {
      tsr_speed_limit_change_flag_ = true;
    }
  }

  if (end_of_speed_sign_vald == false && end_of_speed_sign_value_ > 0.0) {
    // 驶过解除限速标识，
    // 1. 清除小于解除限速牌的限速值
    // 2. 显示解除显示标识
    if (tsr_speed_limit_valid_ == true && tsr_speed_limit_ <= end_of_speed_sign_value_) {
      tsr_speed_limit_valid_ = false;
      tsr_speed_limit_ = current_map_speed_limit_;
      end_of_speed_sign_display_flag_ = true;
      end_of_speed_sign_display_time_ = 0.0;
    }
  }

  if (accumulated_path_length_ >
      GetContext.get_param()->tsr_reset_path_length) {
    // 行驶距离过长，则清掉视觉限速信息, 采用地图限速信息
    tsr_speed_limit_valid_ = false;
    tsr_speed_limit_ = current_map_speed_limit_;
  }

  if (tsr_speed_limit_valid_ == false && end_of_speed_sign_display_flag_ == false) {
    // 没有视觉限速有效值且不需要显示解除限速牌, 采用地图限速信息
    if (current_map_speed_limit_valid_ == true) {
      tsr_speed_limit_ = current_map_speed_limit_;
    }
  }
  return;
}

// 更新限速信息 - 只使用地图限速
void TsrCore::UpdateTsrSpeedLimitOnlyByMap(void) {
  // 只透传地图限速信息
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  
  // sd_map信息，获取限速
  if (GetContext.get_session()->environmental_model()
                                .get_route_info()
                                ->get_sdpromap_valid()) {
    const auto &sd_pro_map_info_ptr = GetContext.get_session()
                                  ->environmental_model()
                                  .get_route_info()
                                  ->get_sdpro_map();
    
    // 获取当前道路限速值
    auto localization_info = GetContext.mutable_session()->mutable_environmental_model()->
                              get_ego_state_manager();
    ad_common::math::Vec2d current_point;
    current_point.set_x(localization_info->location_enu().position.x);
    current_point.set_y(localization_info->location_enu().position.y);
    const double search_distance = 50.0;
    const double max_heading_diff = PI / 4;
    double temp_nearest_s = 0;
    double nearest_l = 0;
    const double ego_heading_angle = localization_info->heading_angle();
    const iflymapdata::sdpro::LinkInfo_Link* current_link = sd_pro_map_info_ptr.GetNearestLinkWithHeading(
        current_point, search_distance, ego_heading_angle, max_heading_diff,
        temp_nearest_s, nearest_l);
    
    if (!current_link) {
      tsr_speed_limit_ = 0;
      tsr_speed_limit_valid_ = false;
      current_map_speed_limit_valid_ = false;
      current_map_speed_limit_ = 0;
    } else {
      tsr_speed_limit_ = current_link->speed_limit();
      tsr_speed_limit_valid_ = true;
      current_map_speed_limit_valid_ = true;
      current_map_speed_limit_ = current_link->speed_limit();
    }
  } else {
    tsr_speed_limit_ = 0;
    tsr_speed_limit_valid_ = false;
    current_map_speed_limit_valid_ = false;
    current_map_speed_limit_ = 0;
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
  } else if (speed_limit_suppression_flag_ == true && std::abs(vehicle_service_output_info_ptr->yaw_rate) < 3 * M_PI / 180.0) {
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
    end_of_speed_sign_info_vector_.clear(); // 解除限速标识
    auto tsr_info_ptr = GetContext.get_tsr_info();
    auto &speed_sign_info_vector = tsr_info_ptr->speed_sign_info_vector;
    // 限速和解除限速信息分别存储
    for (auto &speed_sign_info : speed_sign_info_vector) {
      if (speed_sign_info.speed_sign_type == iflyauto::SuppSignType::SUPP_SIGN_TYPE_MAXIMUM_SPEED) {
        // 远处限速感知不一定准确, 只取前方50m内的限速信息
        if (speed_sign_info.supp_sign_x >= 0.0 && speed_sign_info.supp_sign_x <= 50.0) {
          // 判断是否是匝道限速牌
          if (speed_sign_info.ramp_flag == true) {
            // 速度与实车速度差距过大, 或者车辆打右转灯, 且横向距离小于8m, 不更新限速信息
            if (std::abs(vehicle_speed_display - speed_sign_info.speed_limit) > 20.0) {
              continue;
            }
            else if (vehicle_service_output_info_ptr->right_turn_light_state == true) {
              // 车辆打右转灯, 且横向距离小于8m, 不更新限速信息
              if (speed_sign_info.supp_sign_y < 0.0 && speed_sign_info.supp_sign_y > -8.0) {
                continue;
              }
            }
          } else {
            // 非匝道限速牌, 更新限速信息
          }
          speed_limit_sign_info_vector_.emplace_back(speed_sign_info);
        }
      }
      if (speed_sign_info.speed_sign_type == iflyauto::SuppSignType::SUPP_SIGN_TYPE_END_OF_SPEED_LIMIT) {
        // 解除限速信息, 只取前方50m内的解除限速信息
        if (speed_sign_info.supp_sign_x >= 0.0 && speed_sign_info.supp_sign_x <= 15.0) {
          end_of_speed_sign_info_vector_.emplace_back(speed_sign_info);
        }
      }
    }
    // 如果当前限速和解除限速速度一致, 且地图限速信息有效,则取地图限速信息
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
  // 车辆状态信息
  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  // 功能状态信息
  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                     ->mutable_environmental_model()
                                     ->get_local_view()
                                     .function_state_machine_info;

  // 更新overspeed_status_
  if (tsr_speed_limit_ != 0 && (vehicle_service_output_info_ptr->vehicle_speed_display * 3.6) >
       tsr_speed_limit_ + 1) {
    // +1是为了避免巡航速度正好与限速值一致, 导致超速报警
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

  // 更新tsr_warning_image_, tsr_warning_voice_, 图像和声音同时提示, 延迟0.5s
  if (overspeed_duration_time_ >= 0.5) {
    // 延迟0.5s后显示超速报警
    tsr_warning_image_ = true;
    tsr_warning_voice_ = true;
  } else {
    tsr_warning_image_ = false;
    tsr_warning_voice_ = false;
  }

  // 功能未激活时取消声音与图像报警
  if (tsr_state_ !=
      iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_ACTIVE) {
    tsr_warning_image_ = false;
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
  if (overspeed_duration_time_ >= (5.5)) {
    tsr_warning_image_ = false;
    tsr_warning_voice_ = false;
  } else {
    // do nothing
  }

  // // 按照功能状态更新tsr_warning_image_和tsr_warning_voice_
  // if (function_state_machine_info_ptr->switch_sts.tsr_main_switch == iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_VISUAL_ONLY) {
  //   tsr_warning_voice_ = false;
  // } else if (function_state_machine_info_ptr->switch_sts.tsr_main_switch == iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_OFF ||
  //            function_state_machine_info_ptr->switch_sts.tsr_main_switch == iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_NONE) {
  //   tsr_warning_voice_ = false;
  //   tsr_warning_image_ = false;
  // } else {
  //   // do nothing
  // }

  // HNOA激活，取消所有报警
  if (function_state_machine_info_ptr->current_state == iflyauto::FunctionalState::FunctionalState_NOA_ACTIVATE || 
      function_state_machine_info_ptr->current_state == iflyauto::FunctionalState::FunctionalState_NOA_OVERRIDE) {
    tsr_warning_voice_ = false;
    tsr_warning_image_ = false;
  } else {
    // do nothing
  }

  // 限速值为0时, 取消声音与图像报警
  if (tsr_speed_limit_ == 0) {
    tsr_warning_image_ = false;
    tsr_warning_voice_ = false;
  }
  return;
}

void TsrCore::CalculatePathLengthAccumulated() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  
  // 获取功能状态信息
  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                     ->mutable_environmental_model()
                                     ->get_local_view()
                                     .function_state_machine_info;
  
  // NOA激活模式下，只使用地图限速，不需要计算累积距离
  if (function_state_machine_info_ptr->current_state == iflyauto::FunctionalState::FunctionalState_NOA_ACTIVATE || 
      function_state_machine_info_ptr->current_state == iflyauto::FunctionalState::FunctionalState_NOA_OVERRIDE) {
    // NOA模式下保持距离为0，避免影响其他逻辑
    accumulated_path_length_ = 0.0;
    tsr_speed_limit_change_flag_ = false;  // 重置标志
    return;
  }
  
  // 非NOA模式下，正常计算累积距离用于感知限速的距离衰减
  if (tsr_speed_limit_change_flag_ == true || tsr_speed_limit_valid_ == false) {
    accumulated_path_length_ = 0.0;
    // 重置限速变化标志，避免标志一直为true
    tsr_speed_limit_change_flag_ = false;
  } else {
    accumulated_path_length_ =
        accumulated_path_length_ +
        GetContext.get_state_info()->vehicle_speed * GetContext.get_param()->dt;
  }
  return;
}

// 计算当前显示的辅助标识牌与解除限速牌的时间是否大于2s
void TsrCore::CalculateDurationTime(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  // 如果输出辅助路标发生变化, 则重置辅助标识牌保持时间计数器
  if (supp_sign_change_flag_) {
    supp_sign_hold_time_ = 0.0;
    supp_sign_change_flag_ = false;
  } else {
    supp_sign_hold_time_ += GetContext.get_param()->dt;
  }
  if (end_of_speed_sign_display_flag_ == true) {
    end_of_speed_sign_display_time_ += GetContext.get_param()->dt;
  } else {
    end_of_speed_sign_display_time_ = 0.0;
  }
  return;
}

void TsrCore::SetTsrOutputInfo() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  GetContext.mutable_output_info()->tsr_output_info_.tsr_state_ = tsr_state_;
  // 辅助标识牌抑制显示时，不输出辅助标识牌
  if (supp_sign_in_suppression_flag_ == false) {
    GetContext.mutable_output_info()->tsr_output_info_.supp_sign_type = output_supp_sign_info_;
  } else {
    GetContext.mutable_output_info()->tsr_output_info_.supp_sign_type = iflyauto::SuppSignType::SUPP_SIGN_TYPE_UNKNOWN;
  }
  if (tsr_state_ == iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_ACTIVE) {
    if (end_of_speed_sign_display_flag_) {
      GetContext.mutable_output_info()->tsr_output_info_.isli_display_type_ = true; // 显示解除限速
      GetContext.mutable_output_info()->tsr_output_info_.tsr_speed_limit_ = end_of_speed_sign_value_;
    } else {
      GetContext.mutable_output_info()->tsr_output_info_.isli_display_type_ = false; // 显示限速
      GetContext.mutable_output_info()->tsr_output_info_.tsr_speed_limit_ = tsr_speed_limit_;
    }
    GetContext.mutable_output_info()->tsr_output_info_.tsr_warning_ =
        tsr_warning_image_;
  } else {
    GetContext.mutable_output_info()->tsr_output_info_.tsr_warning_ = false;
    GetContext.mutable_output_info()->tsr_output_info_.tsr_speed_limit_ = 0;
  }
  return;
}

void TsrCore::ResetRealTimeTsrInfo(void) {
  // 如果没有检测到实时辅助标识牌，且当前有输出，且持续时间超过2s，则清空输出
  if (realtime_supp_sign_info_ == iflyauto::SuppSignType::SUPP_SIGN_TYPE_UNKNOWN && 
      supp_sign_valid_flag_ == true && 
      supp_sign_hold_time_ > 2.0) {
    output_supp_sign_info_ = iflyauto::SuppSignType::SUPP_SIGN_TYPE_UNKNOWN;
    supp_sign_valid_flag_ = false;
    supp_sign_hold_time_ = 0.0;
  }
  if (end_of_speed_sign_display_flag_ == true && end_of_speed_sign_display_time_ > 2.0) {
    end_of_speed_sign_display_flag_ = false;
    end_of_speed_sign_display_time_ = 0.0;
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
  // 获取功能状态信息来判断是否为NOA激活模式
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                     ->mutable_environmental_model()
                                     ->get_local_view()
                                     .function_state_machine_info;
  
  // NOA激活模式下，只使用地图限速信息，避免感知干扰
  if (function_state_machine_info_ptr->current_state == iflyauto::FunctionalState::FunctionalState_NOA_ACTIVATE || 
      function_state_machine_info_ptr->current_state == iflyauto::FunctionalState::FunctionalState_NOA_OVERRIDE) {
    UpdateTsrSpeedLimitOnlyByMap();
  } else {
    // 非NOA模式下，使用感知+地图的综合限速信息
    UpdateTsrSpeedLimit();
    // UpdateTsrSpeedLimitNew();
  }


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
  JSON_DEBUG_VALUE("end_of_speed_sign_display_flag_", end_of_speed_sign_display_flag_);
  JSON_DEBUG_VALUE("current_map_speed_limit_", current_map_speed_limit_);
  JSON_DEBUG_VALUE("current_map_speed_limit_valid_", current_map_speed_limit_valid_);
  JSON_DEBUG_VALUE("speed_limit_suppression_flag_", speed_limit_suppression_flag_);
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
  JSON_DEBUG_VALUE("supp_sign_in_suppression_flag_", supp_sign_in_suppression_flag_);

  // reset info
  ResetRealTimeTsrInfo();
}

}  // namespace tsr_core
}  // namespace adas_function