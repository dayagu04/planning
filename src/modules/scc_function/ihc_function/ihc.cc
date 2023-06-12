#include "ihc.h"
// #include <stdio.h>

IHCSys ihc_sys;
#define IHC_StateMachine_IN_ACTIVE 1   // IHC一级主状态
#define IHC_StateMachine_IN_FAULT 2    // IHC一级主状态
#define IHC_StateMachine_IN_OFF 3      // IHC一级主状态
#define IHC_StateMachine_IN_STANDBY 4  // IHC一级主状态

// 更新输入信息
void IHCUpdateInput(planning::framework::Session *session, IHCSys *sys) {
  // 获取IHC开关状态
  sys->input.ihc_main_switch = session->mutable_environmental_model()->get_hmi_info().ihc_main_switch();

  // 获取当前仪表车速
  auto ptr_ego_state_manager = session->mutable_environmental_model()->get_ego_state_manager();
  sys->input.vehicle_speed_display_kph = ptr_ego_state_manager->ego_hmi_v() * 3.6F;  // 当前车速 单位:m/s

  // 获取自动灯光控制状态
  sys->input.auto_light_state = FALSE;
}

uint16 IHCEnableCode(IHCSys *sys) {
  uint16 uint16_bit[16] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
  uint16 ihc_enable_code_temp = 0;

  // condition0
  if (sys->input.vehicle_speed_display_kph < 40.0F) {
    ihc_enable_code_temp += uint16_bit[0];
  } else {
    // do nothing
  }

  // condition1
  if (sys->input.auto_light_state == FALSE) {
    ihc_enable_code_temp += uint16_bit[1];
  } else {
    // do nothing
  }

  return ihc_enable_code_temp;
}

uint16 IHCDisableCode(IHCSys *sys) {
  uint16 uint16_bit[16] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
  uint16 ihc_disable_code_temp = 0;

  // condition0
  if (sys->input.vehicle_speed_display_kph < 25.0F) {
    ihc_disable_code_temp += uint16_bit[0];
  } else {
    // do nothing
  }

  // condition1
  if (sys->input.auto_light_state == FALSE) {
    ihc_disable_code_temp += uint16_bit[1];
  } else {
    // do nothing
  }

  return ihc_disable_code_temp;
}

uint16 IHCFaultCode(IHCSys *sys) {
  uint16 uint16_bit[16] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
  uint16 ihc_fault_code_temp = 0;

  return ihc_fault_code_temp;
}

// IHC状态机
uint8 IHCStateMachine(IHCSys *sys) {
  boolean main_switch = sys->input.ihc_main_switch;
  uint16 fault_code = sys->state.ihc_fault_code;
  uint16 enable_code = sys->state.ihc_enable_code;
  uint16 disable_code = sys->state.ihc_disable_code;

  static uint8 ihc_state_machine_init_flag = 0;  // IHC状态机初始化状态 0:未初始化过 1:已完成过初始化
  static uint8 ihc_state_fault_off_standby_active = 0;  // IHC一级主状态 FAULT OFF STANDBY ACTIVE
  uint8 ihc_state_temp;                                 // 用于存储状态机跳转完状态的临时变量

  if (ihc_state_machine_init_flag == 0) {
    // 状态机处于初始化状态 根据开关状态,决定第一个周期是输出OFF还是STANDBY
    ihc_state_machine_init_flag = 1;
    if (!main_switch) {
      ihc_state_fault_off_standby_active = IHC_StateMachine_IN_OFF;
      ihc_state_temp = 1;
    } else {
      ihc_state_fault_off_standby_active = IHC_StateMachine_IN_STANDBY;
      ihc_state_temp = 2;
    }
  } else {
    // 状态机处于完成过初始化的状态
    switch (ihc_state_fault_off_standby_active) {
      case IHC_StateMachine_IN_ACTIVE:
        if (!main_switch) {  // ACTIVE->OFF
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_OFF;
          ihc_state_temp = 1;
        } else if (fault_code) {  // ACTIVE->FAULT
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_FAULT;
          ihc_state_temp = 0;
        } else if (disable_code) {  // ACTIVE->STANDBY
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_STANDBY;
          ihc_state_temp = 2;
        } else {  // 继续维持在ACTIVE状态
          ihc_state_temp = 3;
        }
        break;
      case IHC_StateMachine_IN_FAULT:
        if (!main_switch) {  // FAULT->OFF
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_OFF;
          ihc_state_temp = 1;
        } else if (!fault_code) {  // FAULT->STANDBY
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_STANDBY;
          ihc_state_temp = 2;
        } else {  // 继续维持在FAULT状态
          ihc_state_temp = 0;
        }
        break;
      case IHC_StateMachine_IN_OFF:
        if (main_switch) {  // OFF->STANDBY
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_STANDBY;
          ihc_state_temp = 2;
        } else {  // 继续维持在OFF状态
          ihc_state_temp = 1;
        }
        break;
      default:
        if (!main_switch) {  // STANDBY->OFF
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_OFF;
          ihc_state_temp = 1;
        } else if (fault_code) {  // STANDBY->FAULT
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_FAULT;
          ihc_state_temp = 0;
        } else if (enable_code == 0) {  // STANDBY->ACTIVE
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_ACTIVE;
          ihc_state_temp = 3;
        } else {  // 继续维持在STANDBY状态
          ihc_state_temp = 2;
        }
        break;
    }
  }
  return ihc_state_temp;
}

// 更新输出信息
void IHCUpdateOutput(planning::framework::Session *session, IHCSys *sys) {}

boolean IHCRequest(planning::framework::Session *session, IHCSys *sys) {
  boolean ihc_request_temp = TRUE;

  // 如果本车道、左车道、右车道检测到移动物体,则禁止请求远光灯
  // 坐标系:本车后轴中心 左正右负
  float32 x_min = 0.0F;    // 检测区域:x最小值 单位:m
  float32 x_max = 100.0F;  // 检测区域:x最大值 单位:m
  float32 y_min = -5.0F;   // 检测区域:y最小值 单位:m
  float32 y_max = 5.0F;    // 检测区域:y最大值 单位:m
  auto ptr_obstacles = session->mutable_environmental_model()->get_obstacle_manager()->get_obstacles();
  float32 obstacle_x = 255.0F;                                                 // 目标物中心的x坐标 单位:m
  float32 obstacle_y = 255.0F;                                                 // 目标物中心的y坐标 单位:m
  float32 obstacle_v = 255.0F;                                                 // 目标物的速度 单位:m/s
  Common::ObjectType obstacle_type = Common::ObjectType::OBJECT_TYPE_UNKNOWN;  // 目标物的类型
  uint16 uint16_bit[16] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
  uint16 obstacle_code = 0;
  // 遍历所有obstacle
  for (const planning::Obstacle *obstacle_ptr : ptr_obstacles.Items()) {
    obstacle_code = 0;

    // condition0
    obstacle_x = obstacle_ptr->x_center();
    if ((obstacle_x < x_min) || (obstacle_x > x_max)) {
      obstacle_code += uint16_bit[0];
    } else {
      // do nothing
    }

    // condition1
    obstacle_y = obstacle_ptr->y_center();
    if ((obstacle_y < y_min) || (obstacle_y > y_max)) {
      obstacle_code += uint16_bit[1];
    } else {
      // do nothing
    }

    // condition2
    obstacle_v = obstacle_ptr->velocity();
    if (std::fabs(obstacle_v) < 1.0F) {
      obstacle_code += uint16_bit[2];
    } else {
      // do nothing
    }

    // condition3
    obstacle_type = obstacle_ptr->type();
    if ((obstacle_type == Common::ObjectType::OBJECT_TYPE_COUPE) ||
        (obstacle_type == Common::ObjectType::OBJECT_TYPE_MINIBUS) ||
        (obstacle_type == Common::ObjectType::OBJECT_TYPE_VAN) ||
        (obstacle_type == Common::ObjectType::OBJECT_TYPE_BUS) ||
        (obstacle_type == Common::ObjectType::OBJECT_TYPE_TRUCK) ||
        (obstacle_type == Common::ObjectType::OBJECT_TYPE_TRAILER) ||
        (obstacle_type == Common::ObjectType::OBJECT_TYPE_BICYCLE) ||
        (obstacle_type == Common::ObjectType::OBJECT_TYPE_MOTORCYCLE) ||
        (obstacle_type == Common::ObjectType::OBJECT_TYPE_TRICYCLE) ||
        (obstacle_type == Common::ObjectType::OBJECT_TYPE_PEDESTRIAN)) {
      obstacle_code += uint16_bit[3];
    } else {
      // do nothing
    }

    if (obstacle_code == 0) {
      ihc_request_temp = FALSE;
      break;
    } else {
      // do nothing
    }
  }

  return ihc_request_temp;
}

void IHCStep(planning::framework::Session *session) {
  // 更新输入信息
  IHCUpdateInput(session, &ihc_sys);

  // 状态机跳转
  ihc_sys.state.ihc_enable_code = IHCEnableCode(&ihc_sys);
  ihc_sys.state.ihc_disable_code = IHCDisableCode(&ihc_sys);
  ihc_sys.state.ihc_fault_code = IHCFaultCode(&ihc_sys);
  ihc_sys.state.ihc_state = IHCStateMachine(&ihc_sys);

  // IHC功能处于激活状态
  if (ihc_sys.state.ihc_state == 3) {
    ihc_sys.state.ihc_request_status = TRUE;
    ihc_sys.state.ihc_request = IHCRequest(session, &ihc_sys);
  } else {
    ihc_sys.state.ihc_request_status = FALSE;
    ihc_sys.state.ihc_request = FALSE;
  }

  // 更新输出信息
  IHCUpdateOutput(session, &ihc_sys);
}