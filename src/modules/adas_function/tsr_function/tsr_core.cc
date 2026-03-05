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
  if (GetContext.get_param()->tsr_main_switch ==
      iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_VISUAL_ONLY) {
    return iflyauto::NotificationMainSwitch::
        NOTIFICATION_MAIN_SWITCH_VISUAL_ONLY;
  } else if (GetContext.get_param()->tsr_main_switch ==
             iflyauto::NotificationMainSwitch::
                 NOTIFICATION_MAIN_SWITCH_VISUAL_AND_AUDIO) {
    return iflyauto::NotificationMainSwitch::
        NOTIFICATION_MAIN_SWITCH_VISUAL_AND_AUDIO;
  } else if (GetContext.get_param()->tsr_main_switch ==
             iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_OFF) {
    return iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_OFF;
  }
  return function_state_machine_info_ptr->switch_sts.tsr_main_switch;
}

// Disable Code
// 0: 正常
// >0: 异常
uint16 TsrCore::UpdateTsrDisableCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;
  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  uint16 disable_code = 0;

  // bit 0
  // TSR感知模块节点通讯丢失，持续0.5s
  if (GetContext.mutable_state_info()->tsr_info_node_valid == false) {
    disable_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // bit 1
  // 判断vehicle_service模块节点通讯丢失,持续0.5s
  if (GetContext.mutable_state_info()->vehicle_service_node_valid == false) {
    disable_code += uint16_bit[1];
  } else {
    /*do nothing*/
  }

  // bit 2
  // 判断仪表车速信号有效性
  if ((vehicle_service_output_info_ptr->vehicle_speed_available == false)) {
    disable_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // bit 3
  // 判断仪表车速信号有效性
  if ((vehicle_service_output_info_ptr->vehicle_speed_display_available ==
       false)) {
    disable_code += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // bit 4
  // 判断横摆角速度信号有效性
  if ((vehicle_service_output_info_ptr->yaw_rate_available == false)) {
    disable_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }
  // bit 5
  //表显车速小于4kph-奔腾车型
  if (GetContext.get_param()->car_type == "bestune_e541" &&
      vehicle_service_output_info_ptr->vehicle_speed_display * 3.6 < 3.8) {
    disable_code += uint16_bit[5];
  }
  // bit 6
  //在APA及HPP状态下，抑制tsr
  if (((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_PARK_STANDBY) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_PARK_PRE_ACTIVE)) ||
      ((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_HPP_STANDBY) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_HPP_ERROR))) {
    disable_code += uint16_bit[6];
  }

  return disable_code & GetContext.get_param()->tsr_disable_code_maskcode;
}

// Fault Code
// 0: 正常
// >0: 异常
uint16 TsrCore::UpdateTsrFaultCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint16 fault_code = 0;

  // bit 0
  // 故障降级
  auto degraded_driving_function_info_ptr =
      &GetContext.mutable_session()
           ->mutable_environmental_model()
           ->get_local_view()
           .degraded_driving_function_info;

  if ((degraded_driving_function_info_ptr->tsr.degraded == iflyauto::INHIBIT) ||
       (degraded_driving_function_info_ptr->tsr.degraded ==
           iflyauto::ERROR_DEGRADED) ||
       (degraded_driving_function_info_ptr->tsr.degraded ==
           iflyauto::ERROR_SAFE_STOP) ||
       (degraded_driving_function_info_ptr->tsr.degraded ==
           iflyauto::MCU_COMM_SHUTDOWN)) {
    fault_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  return fault_code & GetContext.get_param()->tsr_fault_code_maskcode;
}

iflyauto::TSRFunctionFSMWorkState TsrCore::TsrStateMachine(void) {
  iflyauto::TSRFunctionFSMWorkState tsr_state;
  iflyauto::TSRFunctionFSMWorkState tsr_state_delay = tsr_state_;

  if (tsr_state_machine_init_flag_ == false) {
    // 状态机处于初始化状态 根据开关状态,决定第一个周期是输出OFF还是STANDBY
    tsr_state_machine_init_flag_ = true;
    if (tsr_main_switch_ ==
        iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_OFF) {
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
                             TSR_FUNCTION_FSM_WORK_STATE_OFF) {
    // 上一时刻处于TSR_FUNCTION_FSM_WORK_STATE_OFF状态
    if (tsr_main_switch_ == iflyauto::NotificationMainSwitch::
                                       NOTIFICATION_MAIN_SWITCH_VISUAL_ONLY ||
               tsr_main_switch_ ==
                   iflyauto::NotificationMainSwitch::
                       NOTIFICATION_MAIN_SWITCH_VISUAL_AND_AUDIO) {
      // 1. 开关打开
      if (tsr_fault_code_) {  // 1.1 有故障 -> FAULT
        tsr_state =
            iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_FAULT;
      } else {  // 1.2 无故障 -> STANDBY
        tsr_state = iflyauto::TSRFunctionFSMWorkState::
            TSR_FUNCTION_FSM_WORK_STATE_STANDBY;
      }
    } else {  // 2. 开关关闭，维持OFF
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_OFF;
    }
  } else if (tsr_state_delay == iflyauto::TSRFunctionFSMWorkState::
                                    TSR_FUNCTION_FSM_WORK_STATE_STANDBY) {
    // 上一时刻处于TSR_FUNCTION_FSM_WORK_STATE_STANDBY状态
    if (tsr_main_switch_ ==
            iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_OFF ||
        tsr_main_switch_ ==
            iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_NONE) {
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_OFF;
    } else if (tsr_fault_code_) {
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_FAULT;
    } else if (!tsr_disable_code_) {
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_ACTIVE;
    } else {
      tsr_state = iflyauto::TSRFunctionFSMWorkState::
          TSR_FUNCTION_FSM_WORK_STATE_STANDBY;
    }
  } else if (tsr_state_delay == iflyauto::TSRFunctionFSMWorkState::
                                    TSR_FUNCTION_FSM_WORK_STATE_ACTIVE) {
    // 上一时刻处于TSR_FUNCTION_FSM_WORK_STATE_ACTIVE状态
    if (tsr_main_switch_ ==
            iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_OFF ||
        tsr_main_switch_ ==
            iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_NONE) {
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_OFF;
    } else if (tsr_fault_code_) {
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_FAULT;
    } else if (tsr_disable_code_) {
      tsr_state = iflyauto::TSRFunctionFSMWorkState::
          TSR_FUNCTION_FSM_WORK_STATE_STANDBY;
    } else {
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_ACTIVE;
    }
  } else {
    // 处于FAULT状态
    if (tsr_main_switch_ ==
            iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_OFF ||
        tsr_main_switch_ ==
            iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_NONE) {
      // 1. 优先级最高：开关关闭 -> OFF
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_OFF;
    } else if (tsr_fault_code_ == 0) {
      // 2. 其次：全部系统故障解除 -> STANDBY
      tsr_state = iflyauto::TSRFunctionFSMWorkState::
          TSR_FUNCTION_FSM_WORK_STATE_STANDBY;
    } else {
      // 3. 维持FAULT状态
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_FAULT;
    }
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
  if (supp_sign_in_suppression_flag_ == false &&
      std::abs(vehicle_service_output_info_ptr->yaw_rate) >
          6.5 * M_PI / 180.0) {
    supp_sign_in_suppression_flag_ = true;
  } else if (supp_sign_in_suppression_flag_ == true &&
             std::abs(vehicle_service_output_info_ptr->yaw_rate) <
                 5 * M_PI / 180.0) {
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
      if (supp_sign_info.supp_sign_y <= 10.0 &&
          supp_sign_info.supp_sign_y >= -10.0 &&
          supp_sign_info.supp_sign_x >= -100.0 &&
          supp_sign_info.supp_sign_x <= 200.0) {
        // 根据标识牌类型设置对应的位
        switch (supp_sign_info.supp_sign_type) {
          case iflyauto::SuppSignType::SUPP_SIGN_TYPE_STOP_SIGN:
            supp_sign_code_ |= (1 << 0);  // bit 0
            break;
          case iflyauto::SuppSignType::SUPP_SIGN_TYPE_NO_STOPPING:
            supp_sign_code_ |= (1 << 1);  // bit 1
            break;
          case iflyauto::SuppSignType::SUPP_SIGN_TYPE_PROHIBIT_PROLONGED_PARKING:
            supp_sign_code_ |= (1 << 2);  // bit 2
            break;
          case iflyauto::SuppSignType::SUPP_SIGN_TYPE_NO_PARKING:
            supp_sign_code_ |= (1 << 3);  // bit 3
            break;
          case iflyauto::SuppSignType::SUPP_SIGN_TYPE_TUNNEL:
            supp_sign_code_ |= (1 << 4);  // bit 4
            break;
          case iflyauto::SuppSignType::SUPP_SIGN_TYPE_ROAD_CONSTRUCTION_SIGN:
            supp_sign_code_ |= (1 << 5);  // bit 5
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
      iflyauto::SuppSignType::SUPP_SIGN_TYPE_STOP_SIGN,    // bit 0
      iflyauto::SuppSignType::SUPP_SIGN_TYPE_NO_STOPPING,  // bit 1
      iflyauto::SuppSignType::
            SUPP_SIGN_TYPE_PROHIBIT_PROLONGED_PARKING,                // bit 2
      iflyauto::SuppSignType::SUPP_SIGN_TYPE_NO_PARKING,   // bit 3
      iflyauto::SuppSignType::SUPP_SIGN_TYPE_TUNNEL,       // bit 4
      iflyauto::SuppSignType::SUPP_SIGN_TYPE_ROAD_CONSTRUCTION_SIGN, // bit 5
    };

    // 查找第一个从低到高位数为1的位子所对应的标识牌类型
    for (int i = 0; i < sizeof(supp_sign_priority_array) / sizeof(supp_sign_priority_array[0]); i++) {
      if (supp_sign_code_ & (1 << i)) {
        realtime_supp_sign_info_ = supp_sign_priority_array[i];
        break;
      }
    }
  }

  // 更新输出辅助标识牌信息
  if (realtime_supp_sign_info_ !=
      iflyauto::SuppSignType::SUPP_SIGN_TYPE_UNKNOWN) {
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

// 获取限速标识牌中的最高限速值
uint32 TsrCore::GetHighestFromSet(const std::unordered_set<uint8_t>& input_set) {
  uint32 out_num = 0;
  if (input_set.empty()) {
    return out_num;
  }
  for (const auto &each_num : input_set) {
    if (each_num > out_num) {
      out_num = each_num;
    }
  }
  return out_num;
}

// 更新限速信息
void TsrCore::UpdateTsrSpeedLimit(void) {
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
  
  // 道路信息
  auto road_info = GetContext.get_road_info();

  // 地图限速获取逻辑：优先使用sd_map，如果sd_map无效或限速为0，则使用sd_pro_map
  bool sd_map_speed_limit_valid = false;
  uint32 sd_map_speed_limit = 0;
  current_road_type_ = iflyauto::DrivingRoadType::DRIVING_ROAD_TYPE_NONE;

  // 先尝试获取sd_map限速信息
  if (road_info->sdmap_info.valid_flag) {
    sd_map_speed_limit_valid = true;
    sd_map_speed_limit = road_info->sdmap_info.speed_limit;
    current_road_type_ = road_info->sdmap_info.road_type;
  }

  // 如果sd_map限速有效且大于0，则使用sd_map限速
  if (sd_map_speed_limit_valid) {
    current_map_speed_limit_valid_ = true;
    current_map_speed_limit_ = sd_map_speed_limit;
    current_map_type_ = 1;  // sd_map
  }
  // 否则尝试获取sd_pro_map限速信息
  else if (road_info->sdpromap_info.valid_flag) {
    current_map_speed_limit_valid_ = true;
    current_map_speed_limit_ = road_info->sdpromap_info.speed_limit;
    current_map_type_ = 2;  // sd_pro_map
    current_road_type_ = road_info->sdpromap_info.road_type;
  } else {
    current_map_speed_limit_valid_ = false;
    current_map_speed_limit_ = 0;
    current_map_type_ = 0;  // 无地图
    current_road_type_ = iflyauto::DrivingRoadType::DRIVING_ROAD_TYPE_NONE;
  }

  // 判断限速标识牌是否需要抑制显示
  if (speed_limit_suppression_flag_ == false &&
      std::abs(vehicle_service_output_info_ptr->yaw_rate) > 10 * M_PI / 180.0) {
    speed_limit_suppression_flag_ = true;
  } else if (speed_limit_suppression_flag_ == true &&
             std::abs(vehicle_service_output_info_ptr->yaw_rate) <
                 3 * M_PI / 180.0) {
    speed_limit_suppression_flag_ = false;
    // 完成转弯, 则进入新道路
    new_road_flag_ = true;
  }

  // 处理新道路标志
  if (new_road_flag_ == true) {
    // 进入新道路, 则清掉视觉限速信息与视觉解除限速信息, 采用地图限速信息
    tsr_speed_limit_ = current_map_speed_limit_;
    end_of_speed_sign_value_ = 0;
    end_of_speed_sign_display_time_ = 0.0;
    // 重置限速牌和解除限速牌相关标志
    speed_limit_set_.clear();
    end_of_speed_limit_set_.clear();
    speed_limit_out_flag_ = false;
    end_of_speed_limit_out_flag_ = false;
    speed_limit_ever_appeared_ = false;  // 重置曾经出现标志
    end_of_speed_limit_ever_appeared_ = false;  // 重置曾经出现标志
    new_road_flag_ = false;
  }

  if (speed_limit_suppression_flag_ == true) {
    // 抑制限速结果变更
    return;
  }

  auto supp_signs_array = peception_tsr_info.supp_signs;
  int supp_signs_array_size = peception_tsr_info.supp_signs_size;
  uint32 tsr_speed_limit_last = tsr_speed_limit_;
  // 此帧是否有视觉限速标识牌
  has_perception_speed_limit_ = false;
  // 此帧是否有视觉解除限速标识牌
  has_perception_end_of_speed_limit_ = false;
  // 获取自车表显速度 (km/h)
  double ego_speed_kph =
      vehicle_service_output_info_ptr->vehicle_speed_display * 3.6;
  // 遍历所有的标识牌, 获取限速标识牌, 通过set存储起来
  for (int i = 0; i < supp_signs_array_size; i++) {
    auto single_sign = supp_signs_array[i];
    // 只取限速相关标识牌
    if (single_sign.supp_sign_type !=
            iflyauto::SuppSignType::SUPP_SIGN_TYPE_MAXIMUM_SPEED &&
        single_sign.supp_sign_type !=
            iflyauto::SuppSignType::SUPP_SIGN_TYPE_END_OF_SPEED_LIMIT) {
      continue;
    } else {
      // 插入限速标识牌值集合
      if (single_sign.supp_sign_type ==
          iflyauto::SuppSignType::SUPP_SIGN_TYPE_MAXIMUM_SPEED) {
        // 高速公路场景下，过滤限速70或90的限速牌
        if (current_road_type_ == iflyauto::DrivingRoadType::DRIVING_ROAD_TYPE_HIGHWAY) {
          if (single_sign.speed_limit == 70 || single_sign.speed_limit == 90) {
            // 高速公路上的限速70或90标识牌不加入候选列表
            continue;
          }
        }

        speed_limit_set_.insert(single_sign.speed_limit);
        has_perception_speed_limit_ = true;
        speed_limit_ever_appeared_ = true;
      }
      // 插入解除限速标识牌值集合
      if (single_sign.supp_sign_type ==
          iflyauto::SuppSignType::SUPP_SIGN_TYPE_END_OF_SPEED_LIMIT && has_perception_speed_limit_ == false) {
        end_of_speed_limit_set_.insert(single_sign.speed_limit);
        end_of_speed_limit_ever_appeared_ = true;
      }
    }
  }

  // 如果限速牌set有内容, 则清空解除限速牌set, 用于提高限速牌优先级
  if (speed_limit_set_.empty() == false) {
    end_of_speed_limit_set_.clear();
    end_of_speed_limit_ever_appeared_ = false;
  }

  // 计算未感知到任何限速标识持续时间（只有曾经出现过后，才在消失时开始计时）
  if (speed_limit_ever_appeared_ == true && has_perception_speed_limit_ == false) {
    no_speed_limit_duration_time_ += GetContext.get_param()->dt;
    if (no_speed_limit_duration_time_ >= 5.0) {
      no_speed_limit_duration_time_ = 5.0;
    }
  } else if (has_perception_speed_limit_ == true) {
    no_speed_limit_duration_time_ = 0.0;
  }
  // 计算未感知到任何解除限速标识牌持续时间（只有曾经出现过后，才在消失时开始计时）
  if (end_of_speed_limit_ever_appeared_ == true && has_perception_end_of_speed_limit_ == false) {
    no_end_of_speed_limit_duration_time_ += GetContext.get_param()->dt;
    if (no_end_of_speed_limit_duration_time_ >= 5.0) {
      no_end_of_speed_limit_duration_time_ = 5.0;
    }
  } else if (has_perception_end_of_speed_limit_ == true) {
    no_end_of_speed_limit_duration_time_ = 0.0;
  }

  speed_limit_renew_flag_ = false; // 还未更新限速牌
  // 感知没有检测到任何限速牌ns后 允许输出
  if (no_speed_limit_duration_time_ >= GetContext.get_param()->tsr_out_flag_need_last_time) {
    if (speed_limit_set_.size() > 0) {
      // 获取限速标识牌中的最高限速值
      uint32 hightest_perception_speed_limit = GetHighestFromSet(speed_limit_set_);
      if (hightest_perception_speed_limit > 0) {
        // 使用自车表显速度判断感知限速牌真实性
        if (ego_speed_kph < 40.0 ||
           (ego_speed_kph >= 40.0 && ego_speed_kph <= 80.0 && hightest_perception_speed_limit > 10) ||
           (ego_speed_kph > 80.0 && hightest_perception_speed_limit > 40)) {
          tsr_speed_limit_ = hightest_perception_speed_limit;
          speed_limit_out_flag_ = true;
          speed_limit_renew_flag_ = true;
        }
        speed_limit_set_.clear();
      }
    }
  }

  // 感知没有检测到任何解除限速牌ns后
  if (no_end_of_speed_limit_duration_time_ >= GetContext.get_param()->tsr_out_flag_need_last_time) {
    if (end_of_speed_limit_set_.size() > 0) {
      // 获取解除限速标识牌中的最高限速值
      uint32 hightest_perception_end_of_speed_limit = GetHighestFromSet(end_of_speed_limit_set_);
      if (hightest_perception_end_of_speed_limit > 0) {
        if (tsr_speed_limit_last <= hightest_perception_end_of_speed_limit) {
          end_of_speed_sign_value_ = hightest_perception_end_of_speed_limit;
          end_of_speed_limit_out_flag_ = true;
          tsr_speed_limit_ = 0;
          speed_limit_out_flag_ = false;
          speed_limit_renew_flag_ = true;
          speed_limit_set_.clear();
        }
        end_of_speed_limit_set_.clear();
      }
    }

  }

  // 根据当前感知限速速度，更改tsr_reset_path_length
  tsr_reset_path_length_ = GetContext.get_param()->tsr_reset_path_length;
  if (speed_limit_renew_flag_ == true) {
    // bestune_e541
    if (GetContext.get_param()->car_type == "bestune_e541") {
      if (tsr_speed_limit_ >= 5.0 && tsr_speed_limit_ < 20.0) {
        tsr_reset_path_length_ = 150.0;
      } else if (tsr_speed_limit_ >= 20.0 && tsr_speed_limit_ < 30.0) {
        tsr_reset_path_length_ = 200.0;
      } else if (tsr_speed_limit_ >= 30.0 && tsr_speed_limit_ < 40.0) {
        tsr_reset_path_length_ = 350.0;
      } else if (tsr_speed_limit_ >= 40.0 && tsr_speed_limit_ < 50.0) {
        tsr_reset_path_length_ = 400.0;
      } else if (tsr_speed_limit_ >= 50.0 && tsr_speed_limit_ < 70.0) {
        tsr_reset_path_length_ = 700.0;
      } else if (tsr_speed_limit_ >= 70.0 && tsr_speed_limit_ < 80.0) {
        tsr_reset_path_length_ = 900.0;
      } else {
        tsr_reset_path_length_ = 1100.0;
      }
    }
    else {
      if (tsr_speed_limit_ >= 5.0 && tsr_speed_limit_ <= 50.0) {
        tsr_reset_path_length_ = 1500.0;
      } else if (tsr_speed_limit_ >= 51.0 && tsr_speed_limit_ <= 80.0) {
        tsr_reset_path_length_ = 2000.0;
      } else {
        tsr_reset_path_length_ = 4000.0;
      }
    }
  }
  // 行驶距离过长，则清掉视觉限速信息, 采用地图限速信息
  if (accumulated_path_length_ > tsr_reset_path_length_) {
    // 行驶距离过长，则清掉视觉限速信息, 采用地图限速信息
    tsr_speed_limit_ = current_map_speed_limit_;
    // 重置限速牌和解除限速牌相关标志
    speed_limit_set_.clear();
    end_of_speed_limit_set_.clear();
    speed_limit_out_flag_ = false;
    end_of_speed_limit_out_flag_ = false;
    speed_limit_ever_appeared_ = false;  // 重置曾经出现标志
    end_of_speed_limit_ever_appeared_ = false;  // 重置曾经出现标志
    speed_limit_renew_flag_ = true;
  }

  if (road_info->sdpromap_info.NaviMode == 1) {
    tsr_navi_flag_ = true;
  } else {
    tsr_navi_flag_ = false;
  }

  // 只要有有效的sdmap限速信息就采用sdmap的限速信息
  if (sd_map_speed_limit_valid && 
      GetContext.get_param()->sd_map_speed_sw && 
      tsr_navi_flag_) {
    // sd_map限速有效时，直接采用sd_map限速，忽略视觉限速
    tsr_speed_limit_ = current_map_speed_limit_;
  } else if (speed_limit_out_flag_ == false &&
             end_of_speed_limit_out_flag_ == false) {
    // 没有视觉限速有效值且不需要显示解除限速牌, 采用地图限速信息
    if (current_map_speed_limit_valid_ == true) {
      tsr_speed_limit_ = current_map_speed_limit_;
    } else {
      // 地图限速无效时，清空限速
      tsr_speed_limit_ = 0;
    }
  }

  // // bestune_e541车型：仪表车速小于4km/h时不显示限速
  // if (GetContext.get_param()->car_type == "bestune_e541" &&
  //     vehicle_service_output_info_ptr->vehicle_speed_display * 3.6 < 3.8) {
  //   tsr_speed_limit_ = 0;
  // }

  return;
}

// 更新限速信息 - 只使用地图限速
void TsrCore::UpdateTsrSpeedLimitOnlyByMap(void) {
  // 只透传地图限速信息，直接从 SetNavMapInfo 处理后的 road_info 中获取
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto road_info = GetContext.get_road_info();
  
  speed_limit_out_flag_ = false;
  end_of_speed_limit_out_flag_ = false;
  
  // 如果车型是m32t，则使用 sd_map 限速信息
  if (road_info->sdmap_info.valid_flag && GetContext.get_param()->car_type == "chery_m32t") {
    tsr_speed_limit_ = road_info->sdmap_info.speed_limit;
    current_map_speed_limit_valid_ = true;
    current_map_speed_limit_ = road_info->sdmap_info.speed_limit;
    current_map_type_ = road_info->sdmap_info.map_source;  // sd_map
    current_road_type_ = road_info->sdmap_info.road_type;
  }
  // 否则使用 sd_pro_map 限速信息（其他车型只有sdpromap）
  else if (road_info->sdpromap_info.valid_flag) {
    tsr_speed_limit_ = road_info->sdpromap_info.speed_limit;
    current_map_speed_limit_valid_ = true;
    current_map_speed_limit_ = road_info->sdpromap_info.speed_limit;
    current_map_type_ = road_info->sdpromap_info.map_source;  // sd_pro_map
    current_road_type_ = road_info->sdpromap_info.road_type;
  } else {
    tsr_speed_limit_ = 0;
    current_map_speed_limit_valid_ = false;
    current_map_speed_limit_ = 0;
    current_map_type_ = 0;  // 无地图
    current_road_type_ = iflyauto::DrivingRoadType::DRIVING_ROAD_TYPE_NONE;
  }

  return;
}

// 更新超速报警信息
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
  if (tsr_speed_limit_ != 0 &&
      (vehicle_service_output_info_ptr->vehicle_speed_display * 3.6) >
          tsr_speed_limit_ + GetContext.get_param()->tsr_speed_limit_offset) {
    // tsr_speed_limit_offset限速偏移，超过限速偏移才超速报警
    overspeed_status_ = true;
  } else {
    overspeed_status_ = false;
  }

  // 更新overspeed_duration_time_
  if (overspeed_status_) {
    overspeed_duration_time_ += GetContext.get_param()->dt;
    // 限制最大值，防止越界
    if (overspeed_duration_time_ > 100.0) {
      overspeed_duration_time_ = 100.0;
    }
  } else {
    overspeed_duration_time_ = 0.0;
  }

  // 更新tsr_warning, 延迟0.1s
  if (overspeed_duration_time_ >= 0.1) {
    // 延迟0.1s后显示超速报警
    tsr_warning_flag_ = true;
  } else {
    tsr_warning_flag_ = false;
  }

  // 功能未激活时取消声音与图像报警
  if (tsr_state_ !=
      iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_ACTIVE) {
        tsr_warning_flag_ = false;
  } else {
    // do nothing
  }
  // 未超速时取消声音报警
  if (overspeed_status_ == false) {
    tsr_warning_flag_ = false;
  } else {
    // do nothing
  }
  // 声音报警超过最大时长时取消声音报警, 配置参数>100为永远报警
  if (overspeed_duration_time_ >= GetContext.get_param()->tsr_warning_max_duration) {
    tsr_warning_flag_ = false;
  } else {
    // do nothing
  }

  // HNOA激活，取消所有报警
  if (function_state_machine_info_ptr->current_state ==
          iflyauto::FunctionalState::FunctionalState_NOA_ACTIVATE ||
      function_state_machine_info_ptr->current_state ==
          iflyauto::FunctionalState::FunctionalState_NOA_OVERRIDE) {
    tsr_warning_flag_ = false;
  } else {
    // do nothing
  }

  // 限速值为0时, 取消声音与图像报警
  if (tsr_speed_limit_ == 0) {
    tsr_warning_flag_ = false;
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
  if (function_state_machine_info_ptr->current_state ==
          iflyauto::FunctionalState::FunctionalState_NOA_ACTIVATE ||
      function_state_machine_info_ptr->current_state ==
          iflyauto::FunctionalState::FunctionalState_NOA_OVERRIDE) {
    // NOA模式下保持距离为0，避免影响其他逻辑
    accumulated_path_length_ = 0.0;
    return;
  }

  // 非NOA模式下，正常计算累积距离用于感知限速的距离衰减
  if (speed_limit_renew_flag_ == true) {
    accumulated_path_length_ = 0.0;
  } else {
    if (tsr_speed_limit_ != 0) {
      accumulated_path_length_ =
            accumulated_path_length_ +
            GetContext.get_state_info()->vehicle_speed * GetContext.get_param()->dt;
    } else {
      accumulated_path_length_ = 0.0;
    }
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
  // 计算解除限速牌显示持续时间
  if (end_of_speed_limit_out_flag_ == true) {
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
    GetContext.mutable_output_info()->tsr_output_info_.supp_sign_type =
        output_supp_sign_info_;
  } else {
    GetContext.mutable_output_info()->tsr_output_info_.supp_sign_type =
        iflyauto::SuppSignType::SUPP_SIGN_TYPE_UNKNOWN;
  }
  if (tsr_state_ ==
      iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_ACTIVE) {
    if (end_of_speed_limit_out_flag_ == true && speed_limit_out_flag_ == false) {
      GetContext.mutable_output_info()->tsr_output_info_.isli_display_type_ =
          true;  // 显示解除限速
      GetContext.mutable_output_info()->tsr_output_info_.tsr_speed_limit_ =
          end_of_speed_sign_value_;
    } else {
      GetContext.mutable_output_info()->tsr_output_info_.isli_display_type_ =
          false;  // 显示限速
      GetContext.mutable_output_info()->tsr_output_info_.tsr_speed_limit_ =
          tsr_speed_limit_;
    }
    GetContext.mutable_output_info()->tsr_output_info_.tsr_warning_ =
      tsr_warning_flag_;
  } else {
    GetContext.mutable_output_info()->tsr_output_info_.tsr_warning_ = false;
    GetContext.mutable_output_info()->tsr_output_info_.tsr_speed_limit_ = 0;
  }
  return;
}

void TsrCore::ResetRealTimeTsrInfo(void) {
  // 当状态切换到非active时，清除限速信息、辅助标识牌信息和累计距离
  if (tsr_state_prev_ != iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_ACTIVE) {
    // 清除限速信息
    tsr_speed_limit_ = 0;
    speed_limit_set_.clear();
    end_of_speed_limit_set_.clear();
    speed_limit_out_flag_ = false;
    end_of_speed_limit_out_flag_ = false;
    end_of_speed_sign_value_ = 0;
    end_of_speed_sign_display_time_ = 0.0;
    speed_limit_ever_appeared_ = false;
    end_of_speed_limit_ever_appeared_ = false;
    has_perception_speed_limit_ = false;
    has_perception_end_of_speed_limit_ = false;
    no_speed_limit_duration_time_ = 0.0;
    no_end_of_speed_limit_duration_time_ = 0.0;
    speed_limit_renew_flag_ = false;
    
    // 清除辅助标识牌信息
    output_supp_sign_info_ = iflyauto::SuppSignType::SUPP_SIGN_TYPE_UNKNOWN;
    realtime_supp_sign_info_ = iflyauto::SuppSignType::SUPP_SIGN_TYPE_UNKNOWN;
    supp_sign_valid_flag_ = false;
    supp_sign_change_flag_ = false;
    supp_sign_hold_time_ = 0.0;
    supp_sign_code_ = 0;
    
    // 清除累计距离
    accumulated_path_length_ = 0.0;
  }
  
  // 如果没有检测到实时辅助标识牌，且当前有输出，且持续时间超过2s，则清空输出
  if (realtime_supp_sign_info_ ==
          iflyauto::SuppSignType::SUPP_SIGN_TYPE_UNKNOWN &&
      supp_sign_valid_flag_ == true && supp_sign_hold_time_ > 2.0) {
    output_supp_sign_info_ = iflyauto::SuppSignType::SUPP_SIGN_TYPE_UNKNOWN;
    supp_sign_valid_flag_ = false;
    supp_sign_hold_time_ = 0.0;
  }
  // 如果没有检测到解除限速标识牌，且当前有输出，且持续时间超过2s，则清空输出
  if (end_of_speed_limit_out_flag_ == true &&
      end_of_speed_sign_display_time_ > 2.0) {
    end_of_speed_limit_out_flag_ = false;
    end_of_speed_sign_display_time_ = 0.0;
  }
  return;
}

void TsrCore::RunOnce(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  // 更新tsr开关状态
  tsr_main_switch_ = UpdateTsrMainSwitch();

  // 更新tsr_disable_code_
  tsr_disable_code_ = UpdateTsrDisableCode();

  // 更新tsr_fault_code_
  tsr_fault_code_ = UpdateTsrFaultCode();

  // 保存上一时刻的状态
  tsr_state_prev_ = tsr_state_;

  // 更新tsr_state_
  tsr_state_ = TsrStateMachine();

  // 更新实时辅助标识牌信息
  UpdateTsrSuppInfo();

  // 更新tsr_speed_limit_
  // 获取功能状态信息来判断是否为NOA激活模式
  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  auto road_info = GetContext.get_road_info();
  if (road_info->sdpromap_info.NaviMode == 1) {
    tsr_navi_flag_ = true;
  } else {
    tsr_navi_flag_ = false;
  }

  // NOA激活模式下，只使用地图限速信息，避免感知干扰
  if (function_state_machine_info_ptr->current_state ==
          iflyauto::FunctionalState::FunctionalState_NOA_ACTIVATE ||
      function_state_machine_info_ptr->current_state ==
          iflyauto::FunctionalState::FunctionalState_NOA_OVERRIDE || 
          tsr_navi_flag_) {
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

  // 更新tsr_warning
  UpdateTsrWarning();

  // 测试功能
  if (GetContext.get_param()->tsr_function_test_switch) {
    TsrTestFunction();
  }
  //移到此处，解决flag不能清0的问题
  ResetRealTimeTsrInfo();

  // output
  SetTsrOutputInfo();

  // log
  JSON_DEBUG_VALUE("tsr_main_switch_", (int)tsr_main_switch_);
  JSON_DEBUG_VALUE("tsr_disable_code_", tsr_disable_code_);
  JSON_DEBUG_VALUE("tsr_fault_code_", tsr_fault_code_);
  JSON_DEBUG_VALUE("tsr_state_", (int)tsr_state_);
  JSON_DEBUG_VALUE("tsr_speed_limit_", tsr_speed_limit_);
  JSON_DEBUG_VALUE("end_of_speed_sign_value_", end_of_speed_sign_value_);
  JSON_DEBUG_VALUE("has_perception_speed_limit_", has_perception_speed_limit_)
  JSON_DEBUG_VALUE("has_perception_end_of_speed_limit_", has_perception_end_of_speed_limit_)
  JSON_DEBUG_VALUE("end_of_speed_limit_out_flag_", end_of_speed_limit_out_flag_);
  JSON_DEBUG_VALUE("speed_limit_out_flag_", speed_limit_out_flag_);
  JSON_DEBUG_VALUE("speed_limit_renew_flag_", speed_limit_renew_flag_);
  JSON_DEBUG_VALUE("current_map_speed_limit_", current_map_speed_limit_);
  JSON_DEBUG_VALUE("current_map_speed_limit_valid_",
                   current_map_speed_limit_valid_);
  JSON_DEBUG_VALUE("current_map_type_", current_map_type_);
  JSON_DEBUG_VALUE("current_road_type_", (int)current_road_type_);
  JSON_DEBUG_VALUE("speed_limit_suppression_flag_",
                   speed_limit_suppression_flag_);
  JSON_DEBUG_VALUE("tsr_warning_flag_", tsr_warning_flag_);
  JSON_DEBUG_VALUE("tsr_overspeed_status_", overspeed_status_);
  JSON_DEBUG_VALUE("tsr_accumulated_path_length_", accumulated_path_length_);
  JSON_DEBUG_VALUE("tsr_output_supp_sign_info_", (int)output_supp_sign_info_);  
  JSON_DEBUG_VALUE("supp_sign_in_suppression_flag_",
                   supp_sign_in_suppression_flag_);
  JSON_DEBUG_VALUE("tsr_navi_flag_",
                   tsr_navi_flag_);

  // reset info
 // ResetRealTimeTsrInfo();
}

// 测试函数：根据不同的测试模式执行不同的测试场景
// 模式0: 关闭测试
// 模式1: 输出限速80，不报警
// 模式2: 输出限速80，报警
// 模式3: 输出解除限速80
void TsrCore::TsrTestFunction(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  tsr_state_ = iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_ACTIVE;
  int test_mode = GetContext.get_param()->tsr_function_test_switch;
  int supp_sign_type_test = GetContext.get_param()->tsr_supp_sign_type_test;
  // 辅助标识牌输出
  if (supp_sign_type_test != 0) {
    output_supp_sign_info_ = static_cast<iflyauto::SuppSignType>(supp_sign_type_test);
    supp_sign_in_suppression_flag_ = false;
  } else {
    output_supp_sign_info_ = iflyauto::SuppSignType::SUPP_SIGN_TYPE_UNKNOWN;
    supp_sign_in_suppression_flag_ = true;
  }
  // 模式1：输出限速80，不报警
  if (test_mode == 1) {
    tsr_speed_limit_ = 80;
    speed_limit_out_flag_ = true;
    end_of_speed_limit_out_flag_ = false;
    end_of_speed_sign_value_ = 0;
    tsr_warning_flag_ = false;  // 不报警
    overspeed_status_ = false;
  }
  // 模式2：输出限速80，报警
  else if (test_mode == 2) {
    tsr_speed_limit_ = 80;
    speed_limit_out_flag_ = true;
    end_of_speed_limit_out_flag_ = false;
    end_of_speed_sign_value_ = 0;
    tsr_warning_flag_ = true;  // 报警
    overspeed_status_ = true;
  }
  // 模式3：输出解除限速80
  else if (test_mode == 3) {
    end_of_speed_sign_value_ = 80;
    end_of_speed_limit_out_flag_ = true;
    speed_limit_out_flag_ = false;
    tsr_speed_limit_ = 0;
    tsr_warning_flag_ = false;
    overspeed_status_ = false;
  } else {
      // 错误模式：关闭测试，不做任何操作
  }
}

}  // namespace tsr_core
}  // namespace adas_function