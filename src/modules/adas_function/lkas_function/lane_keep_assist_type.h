#ifndef _LANE_KEEP_ASSIST_TYPE_H_
#define _LANE_KEEP_ASSIST_TYPE_H_
#include "Platform_Types.h"
// #include "context/virtual_lane_manager.h"
#include <cmath>

#include "debug_info_log.h"
#include "func_state_machine_c.h"
#include "planning_hmi_c.h"

namespace planning {
#define LKA_StateMachine_IN_ACTIVE 1             // LKA一级主状态
#define LKA_StateMachine_IN_FAULT 2              // LKA一级主状态
#define LKA_StateMachine_IN_OFF 3                // LKA一级主状态
#define LKA_StateMachine_IN_STANDBY 4            // LKA一级主状态
#define LKA_StateMachine_IN_NO_ACTIVE_CHILD 0    // LKA ACTIVE子状态
#define LKA_StateMachine_IN_LeftIntervention 1   // LKA ACTIVE子状态
#define LKA_StateMachine_IN_NoIntervention 2     // LKA ACTIVE子状态
#define LKA_StateMachine_IN_RightIntervention 3  // LKA ACTIVE子状态
#define Common_Cycle_Time 100                    // 系统循环周期计数
#define Common_FrontCamera_PosX \
  1.817  // 前视摄像头光轴距前保的水平距离，单位：m
#define Common_FrontCamera_PosYL \
  0.967  // 前视摄像头光轴距左侧轮胎外边缘的横向距离，单位：m
#define Common_FrontCamera_PosYR \
  0.967  // 前视摄像头光轴距右侧轮胎外边缘的横向距离，单位：m
#define k_lka_enable_lane_width_max 4.5F  // LDW激活的最大车道宽度，单位：m
#define k_lka_enable_lane_width_min 2.8F  // LDW激活的最小车道宽度，单位：m
#define k_lka_disable_lane_width_max 4.8F  // LDW退出的最大车道宽度，单位：m
#define k_lka_disable_lane_width_min 2.5F  // LDW退出的最小车道宽度，单位：m
// for bsd&lca
#define max_objs_num 10
#define width_distance_min 0.5F;  // F线离车身距离
#define width_distance_max 3.5F;  // G线离车身距离
extern uint16 uint16_bit[16];

struct MeasurementPoint {
  uint8 state;
  bool main_switch;  // LDW功能开关状态 0:Off  1:On
  uint16 enable_code;
  uint16 disable_code;
  uint16 fault_code;
  uint16 left_suppression_code;
  uint16 left_kickdown_code;
  uint16 right_suppression_code;
  uint16 right_kickdown_code;
  bool left_intervention;
  bool right_intervention;
  double tlc_line_threshold;
  uint8 left_bsd_lca_code;   // 0,无风险，1有风险
  uint8 right_bsd_lca_code;  // 0,无风险，1有风险
};

struct CalibrationParameter {
  bool enable_roadedge_switch;  // 是否使能功能的路沿场景开关 0:不使能 1:使能
  double enable_vehspd_display_min;  // 激活的最小仪表车速，单位：m/s
  double enable_vehspd_display_max;  // 激活的最大仪表车速，单位：m/s
  double disable_vehspd_display_min;  // 退出的最小仪表车速，单位：m/s
  double disable_vehspd_display_max;  // 退出的最大仪表车速，单位：m/s
  uint16 supp_turn_light_recovery_time;  // 转向灯抑制恢复时长，单位：ms
  double earliest_warning_line;  // 触发的最早报警线，单位：m
  double latest_warning_line;    // 触发的最晚报警线，单位：m
  double reset_warning_line;     // 触发的报警重置线，单位：m
  uint16 warning_time_max;       // 最大报警时长，单位：ms
  double tlc_line_far;  // 针对道线触发报警的高灵敏度阈值，单位：s
  double tlc_line_medium;  // 针对道线触发报警的中灵敏度阈值，单位：s
  double tlc_line_near;  // 针对道线触发报警的低灵敏度阈值，单位：s
  double
      suppression_driver_hand_trq;  // 抑制报警的驾驶员手力矩(绝对值)阈值，单位：Nm
  double
      kickdown_driver_hand_trq;  // 打断报警的驾驶员手力矩(绝对值)阈值，单位：Nm
};

// 道线和路缘信息结构体定义
struct RoadInfo {
  bool left_line_existence;  // 本车道左侧道线存在性
  bool left_line_valid;      // 本车道左侧道线有效性 0:Invalid 1:Valid
  uint8 left_line_type;  // 本车道左侧车道线类型 0：虚线  1：实线
  double left_line_c0;   // 本车道左侧道线方程系数c0
  double left_line_c1;   // 本车道左侧道线方程系数c1
  double left_line_c2;   // 本车道左侧道线方程系数c3
  double left_line_c3;   // 本车道左侧道线方程系数c3
  bool right_line_existence;  // 本车道右侧道线存在性
  bool right_line_valid;  // 本车道右侧道线有效性 0:Invalid 1:Valid
  uint8 right_line_type;  // 本车道右侧车道线类型 0：虚线  1：实线
  double right_line_c0;          // 本车道右侧道线方程系数c0
  double right_line_c1;          // 本车道右侧道线方程系数c1
  double right_line_c2;          // 本车道右侧道线方程系数c2
  double right_line_c3;          // 本车道右侧道线方程系数c3
  bool left_roadedge_existence;  // 本车道左侧路缘存在性
  bool left_roadedge_valid;  // 本车道左侧路缘有效性 0:Invalid 1:Valid
  double left_roadedge_c0;   // 本车道左侧路缘方程系数c0
  double left_roadedge_c1;   // 本车道左侧路缘方程系数c1
  double left_roadedge_c2;   // 本车道左侧路缘方程系数c3
  double left_roadedge_c3;   // 本车道左侧路缘方程系数c3
  bool right_roadedge_existence;  // 本车道右侧路缘存在性
  bool right_roadedge_valid;  // 本车道右侧路缘有效性 0:Invalid 1:Valid
  double right_roadedge_c0;   // 本车道右侧路缘方程系数c0
  double right_roadedge_c1;   // 本车道右侧路缘方程系数c1
  double right_roadedge_c2;   // 本车道右侧路缘方程系数c2
  double right_roadedge_c3;   // 本车道右侧路缘方程系数c3
  bool lane_width_valid;  // 当前车道宽度信息有效性 0:Invalid 1:Valid
  double lane_width;      // 当前车道宽度,单位:m
};

struct VehInfo {
  double veh_left_departure_speed;
  double veh_right_departure_speed;
  double veh_display_speed;
  double veh_actual_speed;
  double veh_yaw_rate;
  double driver_hand_torque;
  bool left_turn_light_state;
  bool right_turn_light_state;
  bool ldw_main_switch;
  bool ldp_main_switch;
  bool elk_main_switch;
  uint8 ldw_tlc_level;
  double common_front_over;
  double common_rear_over;
  double common_wheel_base;
  double common_veh_width;
};

struct WheelToLine {
  double fl_wheel_distance_to_line;
  double fl_wheel_distance_to_roadedge;
  double fr_wheel_distance_to_line;
  double fr_wheel_distance_to_roadedge;
};

struct Param {
  double ldp_tlc_thrd = 1.0;
  double ldp_c0_right_offset = 0.0;
  double ldp_ttlc_right_hack = 1.1;
};

struct LkasInput {
  VehInfo vehicle_info;
  RoadInfo road_info;
  WheelToLine wheel_to_line;
  iflyauto::FunctionalState function_state;
  Param param;
};

/*
0:Unavailable
1:Off
2:Standby
3:Active(No Intervention)
4:Active(Left Intervention)
5:Active(Right Intervention)*/
struct LkasState {
  uint8 ldw_state;
  uint8 ldp_state;
  uint8 elk_state;
};

// for bsd&lca
struct RadarObjData {
  uint8 pos;  // 0代表左，1代表右
  uint8 obj_class;
  double obj_x;
  double obj_y;
  double obj_vx;
  double obj_vy;
  double obj_length;
  double obj_width;
};

struct AeraVel {
  double area_length_distance;
  double b_x;
  double c_x;
  double f_y;
  double g_y;
  double l_y;
  double k_y;
  double obstacle_velocity_limit;
};

bool LKALineLeftIntervention(double tlc_to_line_threshold,
                             planning::LkasInput *lkas_input);
bool LKARoadEdgeLeftIntervention(double tlc_to_roadedge_threshold,
                                 planning::LkasInput *lkas_input);
bool LKALineRightIntervention(double tlc_to_line_threshold,
                              planning::LkasInput *lkas_input);
bool LKARoadEdgeRightIntervention(double tlc_to_roadedge_threshold,
                                  planning::LkasInput *lkas_input);

}  // namespace planning

#endif