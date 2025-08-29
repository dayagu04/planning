#include "ihc_core.h"

using namespace planning;
namespace adas_function {
namespace ihc_core {

void IhcCore::RunOnce(void) {
  // 更新输入信息
  GetInputInfo();

  // 根据输入信息，更新使能码、禁用码、故障码、状态机跳转
  ihc_sys_.state.ihc_enable_code = UpdateIhcEnableCode();
  ihc_sys_.state.ihc_disable_code = UpdateIhcDisableCode();
  ihc_sys_.state.ihc_fault_code = UpdateIhcFaultCode();

  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  if (GetContext.get_param()->ihc_use_json_code) {
    // 如果使用json，则使用配置文件中的使能码、禁用码、故障码
    ihc_sys_.input.ihc_main_switch = GetContext.get_param()->ihc_main_switch;
    ihc_sys_.state.ihc_enable_code = GetContext.get_param()->ihc_enable_code_maskcode;
    ihc_sys_.state.ihc_fault_code = GetContext.get_param()->ihc_fault_code_maskcode;
  }

  ihc_sys_.state.ihc_state = IHCStateMachine();

  // IHC功能处于激活状态
  if (ihc_sys_.state.ihc_state == iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE) {
    ihc_sys_.state.ihc_request_status = true;
    ihc_sys_.state.ihc_request = IHCRequest();
  } else {
    ihc_sys_.state.ihc_request_status = false;
    ihc_sys_.state.ihc_request = false;
  }
  SetIhcOutputInfo();

  // 从配置文件中读取ihc_main_switch值, 测试用, 后续需要移动到XXXcode中
  if (GetContext.get_param()->ihc_use_json_switch) {
    JsonSwitchIhcMainSwitch();
  } else {
    // do nothing
  }

  JSON_DEBUG_VALUE("ihc_function::ihc_enable_code",
                   ihc_sys_.state.ihc_enable_code);
  JSON_DEBUG_VALUE("ihc_function::ihc_disable_code",
                   ihc_sys_.state.ihc_disable_code);
  JSON_DEBUG_VALUE("ihc_function::ihc_fault_code",
                   ihc_sys_.state.ihc_fault_code);
  JSON_DEBUG_VALUE("ihc_function::ihc_state", int(ihc_sys_.state.ihc_state));
  JSON_DEBUG_VALUE("ihc_function::ihc_request_status",
                   ihc_sys_.state.ihc_request_status);
  JSON_DEBUG_VALUE("ihc_function::ihc_request", ihc_sys_.state.ihc_request);
  JSON_DEBUG_VALUE("ihc_function::ihc_main_switch",
                   ihc_sys_.input.ihc_main_switch);
  JSON_DEBUG_VALUE("ihc_function::auto_light_state",
                   ihc_sys_.input.auto_light_state);
  JSON_DEBUG_VALUE("ihc_function::low_beam_due_to_same_dir_vehicle",
                   ihc_sys_.state.low_beam_due_to_same_dir_vehicle);
  JSON_DEBUG_VALUE("ihc_function::low_beam_due_to_oncomming_vehicle",
                   ihc_sys_.state.low_beam_due_to_oncomming_vehicle);
  JSON_DEBUG_VALUE("ihc_function::low_beam_due_to_oncomming_cycle",
                   ihc_sys_.state.low_beam_due_to_oncomming_cycle);
  
  // 输出环境光线条件（已在IHCRequest中设置）
  JSON_DEBUG_VALUE("ihc_function::lighting_condition",
                   int(ihc_sys_.input.lighting_condition));
}

void IhcCore::GetInputInfo() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  // 获取功能状态机信息
  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                     ->mutable_environmental_model()
                                     ->get_local_view()
                                     .function_state_machine_info;

  // 获取车辆服务输出信息
  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                            ->mutable_environmental_model()
                                            ->get_local_view()
                                            .vehicle_service_output_info;

  // 获取环境光线条件记录信息
  const auto scene_info_ptr = &GetContext.mutable_session()
                                     ->mutable_environmental_model()
                                     ->get_local_view()
                                     .perception_scene_info;

  ihc_sys_.input.lighting_condition = scene_info_ptr->lighting_condition;

  // 获取IHC开关状态
  ihc_sys_.input.ihc_main_switch =
      function_state_machine_info_ptr->switch_sts.ihc_main_switch;

  // 获取当前仪表车速
  ihc_sys_.input.vehicle_speed_display_kph =
      vehicle_service_output_info_ptr->vehicle_speed_display * 3.6F;  // 当前车速 单位:kph

  // 获取换档杆状态
  ihc_sys_.input.shift_lever_state =
      static_cast<iflyauto::ShiftLeverStateEnum>(vehicle_service_output_info_ptr->shift_lever_state);

  // 获取前雾灯状态
  ihc_sys_.input.front_fog_light_state =
      vehicle_service_output_info_ptr->front_fog_light_state;
  
  // 后雾灯状态
  ihc_sys_.input.rear_fog_light_state =
      vehicle_service_output_info_ptr->rear_fog_light_state;
  
  // 雨刮运行速度是否达到快速档位
  // ihc_sys_.input.wiper_speed_fast = vehicle_service_output_info_ptr->wiper_speed_fast;


  // 获取自动灯光控制状态
  ihc_sys_.input.auto_light_state = vehicle_service_output_info_ptr->auto_light_state;
}

uint16 IhcCore::UpdateIhcEnableCode() {
  uint16 ihc_enable_code_temp = 0;

  // condition0: 灯光挡位处于auto
  if (ihc_sys_.input.ihc_main_switch == true) {
    ihc_enable_code_temp += uint16_bit[0];
  } else {
    // do nothing
  }

  // condition1: 车速是否大于等于40kph
  if (ihc_sys_.input.vehicle_speed_display_kph >= 40.0F) {
    ihc_enable_code_temp += uint16_bit[1];
  } else {
    // do nothing
  }

  // codition2：车速是否小于等于150kph
  if (ihc_sys_.input.vehicle_speed_display_kph <= 150.0F) {
    ihc_enable_code_temp += uint16_bit[2];
  } else {
    // do nothing
  }

  // condition3：后雾灯状态为false
  if (ihc_sys_.input.rear_fog_light_state == false) {
    ihc_enable_code_temp += uint16_bit[3];
  } else {
    // do nothing
  }

  // condition4: 前雾灯状态是否为false
  if (ihc_sys_.input.front_fog_light_state == false) {
    ihc_enable_code_temp += uint16_bit[4];
  } else {
    // do nothing
  }

  // condition5: 雨刮运行速度没有达到快速档位
  // if (ihc_sys_.input.wiper_speed_fast == false) {
  //   ihc_enable_code_temp += uint16_bit[5];
  // } else {
  //   // do nothing
  // }

  return ihc_enable_code_temp;
}
uint16 IhcCore::UpdateIhcDisableCode() {
  uint16 ihc_disable_code_temp = 0;

  // condition0：灯光挡位不是auto
  if (ihc_sys_.input.ihc_main_switch == false) {
    ihc_disable_code_temp += uint16_bit[0];
  } else {
    // do nothing
  }

  // condition1: 车速小于等于30kph
  if (ihc_sys_.input.vehicle_speed_display_kph < 30.0F) {
    ihc_disable_code_temp += uint16_bit[1];
  } else {
    // do nothing
  }

  // condition2：车速是否大于155kph
  if (ihc_sys_.input.vehicle_speed_display_kph > 155.0F) {
    ihc_disable_code_temp += uint16_bit[2];
  } else {
    // do nothing
  }

  // condition3：后雾灯状态为true
  if (ihc_sys_.input.rear_fog_light_state == true) {
    ihc_disable_code_temp += uint16_bit[3];
  } else {
    // do nothing
  }

  // condition4：前雾灯状态为true
  if (ihc_sys_.input.front_fog_light_state == true) {
    ihc_disable_code_temp += uint16_bit[4];
  } else {
    // do nothing
  }

  // condition5：雨刮运行速度达到快速档位
  // if (ihc_sys_.input.wiper_speed_fast == true) {
  //   ihc_disable_code_temp += uint16_bit[5];
  // } else {
  //   // do nothing
  // }

  return ihc_disable_code_temp;
}

// TODO: thzhang5 0714 需要根据文档需求更改
uint16 IhcCore::UpdateIhcFaultCode() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint16 fault_code = 0;

  // bit 0
  // 前视摄像头有故障(信号没有给出)
  // if (vehicle_service_output_info_ptr->... == false) {
  //   fault_code += uint16_bit[0];
  // } else {
  //   /*do nothing*/
  // }

  // bit 1
  // IHC感知模块节点通讯丢失，持续5s
  if (GetContext.mutable_state_info()->ihc_info_node_valid == false) {
    fault_code += uint16_bit[1];
  } else {
    /*do nothing*/
  }

  // bit 2
  // 障碍物融合模块节点通讯丢失，持续0.5s
  if (GetContext.mutable_state_info()->obstacle_fusion_info_node_valid == false) {
    fault_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // bit 3
  // vehicle_service模块节点通讯丢失，持续0.5s
  if (GetContext.mutable_state_info()->vehicle_service_node_valid == false) {
    fault_code += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // bit 4
  // 实际车速信号无效
  if (vehicle_service_output_info_ptr->vehicle_speed_available == false) {
    fault_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  // bit 5
  // 仪表车速信号无效
  if (vehicle_service_output_info_ptr->vehicle_speed_display_available == false) {
    fault_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }

  // bit 6
  // 雾灯信号无效
  if (vehicle_service_output_info_ptr->front_fog_light_state_available == false ||
      vehicle_service_output_info_ptr->front_fog_light_state_available == false) {
    fault_code += uint16_bit[6];
  } else {
    /*do nothing*/
  }

  // bit 7
  // 雨刮信号无效
  if (vehicle_service_output_info_ptr->wiper_state_available == false) {
    fault_code += uint16_bit[7];
  } else {
    /*do nothing*/
  }

  return fault_code;
}

iflyauto::IHCFunctionFSMWorkState IhcCore::IHCStateMachine() {
  bool main_switch = ihc_sys_.input.ihc_main_switch;
  uint16 fault_code = ihc_sys_.state.ihc_fault_code;
  uint16 enable_code = ihc_sys_.state.ihc_enable_code;
  uint16 disable_code = ihc_sys_.state.ihc_disable_code;

  static uint8 ihc_state_machine_init_flag =
      0;  // IHC状态机初始化状态 0:未初始化过 1:已完成过初始化
  static uint8 ihc_state_fault_off_standby_active =
      0;                 // IHC一级主状态 FAULT OFF STANDBY ACTIVE
  iflyauto::IHCFunctionFSMWorkState ihc_state_temp;  // 用于存储状态机跳转完状态的临时变量

  if (ihc_state_machine_init_flag == 0) {
    // 状态机处于初始化状态 根据开关状态,决定第一个周期是输出OFF还是STANDBY
    ihc_state_machine_init_flag = 1;
    if (!main_switch) {
      ihc_state_fault_off_standby_active = IHC_StateMachine_IN_OFF;
      ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
    } else {
      ihc_state_fault_off_standby_active = IHC_StateMachine_IN_STANDBY;
      ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY;
    }
  } else {
    // 状态机处于完成过初始化的状态
    switch (ihc_state_fault_off_standby_active) {
      case IHC_StateMachine_IN_ACTIVE:
        if (!main_switch) {  // ACTIVE->OFF
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_OFF;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
        } else if (fault_code) {  // ACTIVE->FAULT
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_FAULT;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_FAULT;
        } else if (enable_code != 31) {  // ACTIVE->STANDBY (所有条件不满足时切换)
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_STANDBY;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY;
        } else {  // 继续维持在ACTIVE状态
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE;
        }
        break;
      case IHC_StateMachine_IN_FAULT:
        if (!main_switch) {  // FAULT->OFF
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_OFF;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
        } else if (!fault_code) {  // FAULT->STANDBY
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_STANDBY;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY;
        } else {  // 继续维持在FAULT状态
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_FAULT;
        }
        break;
      case IHC_StateMachine_IN_OFF:
        // OFF
        if (main_switch) {
          if (fault_code) {
            // OFF->FAULT
            ihc_state_fault_off_standby_active = IHC_StateMachine_IN_FAULT;
            ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_FAULT;
          } else {
            // OFF->STANDBY
            ihc_state_fault_off_standby_active = IHC_StateMachine_IN_STANDBY;
            ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY;
          }
        } else {  // 继续维持在OFF状态
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
        }
        break;
      default:
        // standby
        if (!main_switch) {  // STANDBY->OFF
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_OFF;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
        } else if (fault_code) {  // STANDBY->FAULT
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_FAULT;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_FAULT;
        } else if (enable_code == 31) {  // STANDBY->ACTIVE
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_ACTIVE;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE;
        } else {  // 继续维持在STANDBY状态
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY;
        }
        break;
    }
  }
  return ihc_state_temp;
}

bool IhcCore::IHCRequestLightingFilter(bool ihc_request_lighting, uint8_t window_size, float ratio_threshold, uint8_t max_trasition) {
  bool ihc_request_lighting_filter_temp = false; // 默认近光灯
  if (ihc_request_lighting_buffer_.size() < window_size) {
    ihc_request_lighting_buffer_.push_back(ihc_request_lighting);
    ihc_request_lighting_filter_temp = false;
  } else {
    ihc_request_lighting_buffer_.erase(ihc_request_lighting_buffer_.begin());
    ihc_request_lighting_buffer_.push_back(ihc_request_lighting);
    // 计算信号是否稳定
    uint8_t transitions = 0;
    for (uint8_t i = 0; i < ihc_request_lighting_buffer_.size(); ++i) {
      if (ihc_request_lighting_buffer_[i] != ihc_request_lighting_buffer_[i + 1]) {
        transitions++;
      }
    }
    if (transitions <= max_trasition) {
      // 选择输出结果
      // 统计true和false的数量，选择出现最多的结果
      uint8_t true_count = 0;
      uint8_t false_count = 0;
      
      for (uint8_t i = 0; i < ihc_request_lighting_buffer_.size(); ++i) {
        if (ihc_request_lighting_buffer_[i]) {
          true_count++;
        } else {
          false_count++;
        }
      }
      
      // 选择出现次数最多的结果，如果相等则默认选择false
      ihc_request_lighting_filter_temp = (true_count > false_count);
      
    } else {
      ihc_request_lighting_filter_temp = false;
    }
  }
  return ihc_request_lighting_filter_temp;
}

/*
  动态障碍物消息处理, 返回是否需要切远光
  1. 同向有车(含非机动车)100m内, 切近光
  2. 对向有车, 机动车200m内,切近光, 非机动车75m内, 切近光
  使用滞回控制避免临界距离抖动：在滞回区间内保持当前状态不变
*/
bool IhcCore::IHCRequestDynamicObstacle(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  bool last_high_beam_request = GetContext.get_output_info()->ihc_output_info_.ihc_request_;
  bool high_beam_request_temp = true;  // 默认远光
  bool found_obstacle_in_hysteresis = false;  // 是否发现滞回区间内的障碍物
  
  // 初始化debug变量
  ihc_sys_.state.low_beam_due_to_same_dir_vehicle = false;
  ihc_sys_.state.low_beam_due_to_oncomming_vehicle = false;
  ihc_sys_.state.low_beam_due_to_oncomming_cycle = false;
  
  // 获取动态障碍物消息
  // 1. 同向有车(含非机动车)100m内, 切近光
  // 2. 对向有车, 机动车400m内,切近光, 非机动车75m内, 切近光
  const auto &fusion_objs = GetContext.get_session()
                                ->environmental_model()
                                .get_local_view()
                                .fusion_objects_info.fusion_object;
  const int fusion_objs_num = GetContext.get_session()
                                ->environmental_model()
                                .get_local_view()
                                .fusion_objects_info.fusion_object_size;
  
  for (int i = 0; i < fusion_objs_num; i++) {
    float distance = fusion_objs[i].common_info.relative_center_position.x;
    
    // 筛选前方的车辆动态障碍物，使用滞回控制
    if (distance > 0 && distance < 220.0F) {  // 扩大检测范围
      // 判断障碍物是否为机动车
      if (fusion_objs[i].common_info.type >= iflyauto::ObjectType::OBJECT_TYPE_COUPE && 
          fusion_objs[i].common_info.type <= iflyauto::ObjectType::OBJECT_TYPE_TRAILER) {
        
        // 判断障碍物是否为对向车辆
        if (fusion_objs[i].additional_info.motion_pattern_current == iflyauto::ObjectMotionType::OBJECT_MOTION_TYPE_ONCOME) {
          // 对向机动车：滞回控制，190m~210m为滞回区间
          if (distance < 190.0f) {
            // 明确进入近光区域
            high_beam_request_temp = false;
            ihc_sys_.state.low_beam_due_to_oncomming_vehicle = true;
            break;
          } else if (distance <= 210.0f) {
            // 190m~210m滞回区间，保持当前状态
            found_obstacle_in_hysteresis = true;
            if (!last_high_beam_request) {
              high_beam_request_temp = false;
              ihc_sys_.state.low_beam_due_to_oncomming_vehicle = true;
              break;
            }
          }
          // distance > 210.0f 时继续检查其他障碍物
        } else {
          // 同向机动车：滞回控制，90m~110m为滞回区间
          if (distance < 90.0f) {
            // 明确进入近光区域
            high_beam_request_temp = false;
            ihc_sys_.state.low_beam_due_to_same_dir_vehicle = true;
            break;
          } else if (distance <= 110.0f) {
            // 90m~110m滞回区间，保持当前状态
            found_obstacle_in_hysteresis = true;
            if (!last_high_beam_request) {
              high_beam_request_temp = false;
              ihc_sys_.state.low_beam_due_to_same_dir_vehicle = true;
              break;
            }
          }
          // distance > 110.0f 时继续检查其他障碍物
        }
      } else if (fusion_objs[i].common_info.type >= iflyauto::ObjectType::OBJECT_TYPE_CYCLE_RIDING && 
                 fusion_objs[i].common_info.type <= iflyauto::ObjectType::OBJECT_TYPE_TRICYCLE_RIDING) {
        // 对向非机动车：滞回控制，65m~85m为滞回区间
        if (fusion_objs[i].additional_info.motion_pattern_current == iflyauto::ObjectMotionType::OBJECT_MOTION_TYPE_ONCOME) {
          if (distance < 65.0f) {
            // 明确进入近光区域
            high_beam_request_temp = false;
            ihc_sys_.state.low_beam_due_to_oncomming_cycle = true;
            break;
          } else if (distance <= 85.0f) {
            // 65m~85m滞回区间，保持当前状态
            found_obstacle_in_hysteresis = true;
            if (!last_high_beam_request) {
              high_beam_request_temp = false;
              ihc_sys_.state.low_beam_due_to_oncomming_cycle = true;
              break;
            }
          }
          // distance > 85.0f 时继续检查其他障碍物
        }
      }
    }
  }
  
  return high_beam_request_temp;
}

bool IhcCore::IHCRequest() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  bool ihc_request_temp = GetContext.get_output_info()->ihc_output_info_.ihc_request_;

  // 环境亮度条件
  if (ihc_sys_.input.lighting_condition == iflyauto::CameraPerceptionLightingCondition::CAMERA_PERCEPTION_LIGHTING_CONDITION_DARK) {
    // 昏暗环境, 根据障碍物信息判断是否需要切远光
    ihc_request_temp = IHCRequestDynamicObstacle();
  } else if (ihc_sys_.input.lighting_condition == iflyauto::CameraPerceptionLightingCondition::CAMERA_PERCEPTION_LIGHTING_CONDITION_BRIGHT) {
    // 明亮环境
    ihc_request_temp = false;
  } else if (ihc_sys_.input.lighting_condition == iflyauto::CameraPerceptionLightingCondition::CAMERA_PERCEPTION_LIGHTING_CONDITION_MEDIUM) {
    // 中等亮度环境，根据当前车灯判断是否需要切远光
    if (ihc_request_temp == false) {
      // 当前为近光灯，保持
    } else {
      // 当前为远光灯，判断是否要切近光
      ihc_request_temp = IHCRequestDynamicObstacle();
    }
  } else {
    // 中等亮度环境，保持
    // do nothing
  }
  
  return ihc_request_temp;
}



void IhcCore::SetIhcOutputInfo(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  
  // 状态安全检查：只有在ACTIVE状态下才能输出有效的请求
  if (ihc_sys_.state.ihc_state == iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE) {
    GetContext.mutable_output_info()->ihc_output_info_.ihc_request_status_ =
      ihc_sys_.state.ihc_request_status;
    GetContext.mutable_output_info()->ihc_output_info_.ihc_request_ =
        ihc_sys_.state.ihc_request;
  } else {
    // 非ACTIVE状态下，强制设置为无请求状态
    GetContext.mutable_output_info()->ihc_output_info_.ihc_request_status_ = false;
    GetContext.mutable_output_info()->ihc_output_info_.ihc_request_ = false;
  }
  
  // 输出状态信息
  switch (ihc_sys_.state.ihc_state) {
    case iflyauto::IHC_FUNCTION_FSM_WORK_STATE_FAULT:
      GetContext.mutable_output_info()->ihc_output_info_.ihc_state_ = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_FAULT;
      break;
    case iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF:
      GetContext.mutable_output_info()->ihc_output_info_.ihc_state_ = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
      break;
    case iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY:
      GetContext.mutable_output_info()->ihc_output_info_.ihc_state_ = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY;
      break;
    case iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE:
      GetContext.mutable_output_info()->ihc_output_info_.ihc_state_ = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE;
      break;
    default:
      // 不可用状态，设置为 OFF
      GetContext.mutable_output_info()->ihc_output_info_.ihc_state_ = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
      break;
  }
}

bool IhcCore::JsonSwitchIhcMainSwitch() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  // 读取配置文件中的 ihc_main_switch 值
  bool ihc_switch = GetContext.get_param()->ihc_main_switch;
  
  if (ihc_switch) {
    // 如果为1，设置为true
    GetContext.mutable_output_info()->ihc_output_info_.ihc_state_ = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE;
    GetContext.mutable_output_info()->ihc_output_info_.ihc_request_status_ = true;
    GetContext.mutable_output_info()->ihc_output_info_.ihc_request_ = true;
  } else {
    // 如果为0，设置为false 
    GetContext.mutable_output_info()->ihc_output_info_.ihc_state_ = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE;
    GetContext.mutable_output_info()->ihc_output_info_.ihc_request_status_ = true;
    GetContext.mutable_output_info()->ihc_output_info_.ihc_request_ = false;
  }
  
  return ihc_switch;
}

}  // namespace ihc_core
}  // namespace adas_function