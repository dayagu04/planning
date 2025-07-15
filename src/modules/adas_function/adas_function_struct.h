#ifndef __ADAS_FUNCTION_STRUCT_H__
#define __ADAS_FUNCTION_STRUCT_H__

#include <string>

#include "camera_perception_tsr_c.h"
#include "common_c.h"
#include "func_state_machine_c.h"
#include "fusion_road_c.h"
#include "planning_hmi_c.h"
#include "spline.h"
#include "transform_lib.h"
#include "vehicle_service_c.h"

// using namespace iflyauto;
namespace adas_function {
namespace context {

struct Parameters {
  double dt = 0.1;  // adas_function运行周期 单位:s

  // 车辆参数
  std::string car_type = "Unknow";
  double wheel_base = 2.72;    // 轴距，单位：m
  double steer_ratio = 15.04;  // 转向传动比，即方向盘转角/前轮转角
  double ego_length = 4.605;   // 本车车长，单位：m
  double ego_width = 1.89;     // 本车车宽，单位：m
  double origin_2_front_bumper = 0.950 + 2.72;  // 后轴中心到前保的距离 单位：m
  double origin_2_rear_bumper = 0.950;          // 后轴中心到后保的距离 单位：m

  // LKAS_Function 参数
  std::vector<double> lka_vel_vector = {40.0, 60.0, 80.0, 100.0, 120.0, 140.0};
  std::vector<double> lka_tlc_vector = {1.0, 1.0, 0.9, 0.8, 0.7, 0.7};

  // 根据曲率查表tlc的参数
  std::vector<double> lka_c2_vector = {0.00025, 0.000333, 0.0005, 0.001,
                                       0.0025};  // R2000,R1500,R1000,R500,R200
  std::vector<double> lka_dec_tlc_by_c2_vector = {0.0, 0.05, 0.15, 0.25, 0.35};

  // 根据道宽查表tlc的参数
  std::vector<double> lka_lane_width_vector = {2.50, 2.75, 3.00, 3.25, 3.50};
  std::vector<double> lka_tlc_dec_by_lane_width_vector = {0.80, 0.60, 0.40,
                                                          0.10, 0.00};
  double safe_departure_ttc = 3.0;
  double ldw_enable_speed = 27.555;
  double ldw_tlc_thrd = 0.0;
  double ldp_tlc_thrd = 1.0;
  double ldp_roadedge_tlc_thrd = 1.0;
  double ldp_roadedge_offset = 0.15;
  double ldp_roadedge_distance_limit = 3.0;

  double ldp_c0_right_offset = 0.0;
  double ldp_ttlc_right_hack = 1.1;
  double ldp_center_line_offset = 0.4;
  double ldp_center_roadedge_offset = 0.7;

  bool ldw_main_switch = false;
  bool ldp_main_switch = false;
  bool elk_main_switch = false;
  iflyauto::NotificationMainSwitch tsr_main_switch =
      iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_NONE;
  bool ihc_main_switch = false;
  double elk_tlc_thrd = 1.0;
  double elk_roadedge_tlc_thrd = 1.0;
  double elk_roadedge_offset = 0.15;
  double lon_distance_buffer0 = 3.0;
  double lon_distance_buffer1 = 60.0;
  double lat_buffer_to_line = 4.0;
  double tsr_reset_path_length = 10000.0;
  double lane_boundary_vaild_length_set = 15.0;
  double sideway_exist_gap_thrd = 0.5;
  double lane_line_width = 0.15;
  // sim params
  bool adas_sim_switch = false;
  bool force_no_sideway_switch = true;
  bool force_no_safe_departure_switch = false;
  // test params
  bool hmi_test_switch = false;
  int hmi_ldw_state = 0;
  int hmi_ldp_state = 0;
  int hmi_elk_state = 0;
  int hmi_tsr_state = 0;
  int hmi_tsr_speed_limit = 0;
  uint32 ldp_fault_code_maskcode = 0;
  uint32 ldp_enable_code_maskcode = 0;
  uint32 ldp_disable_code_maskcode = 0;
  uint32 ldp_left_suppression_code_maskcode = 0;
  uint32 ldp_left_kickdown_code_maskcode = 0;
  uint32 ldp_right_suppression_code_maskcode = 0;
  uint32 ldp_right_kickdown_code_maskcode = 0;
  uint32 ldw_enable_code_maskcode = 0;
  uint32 ldw_disable_code_maskcode = 0;
  uint32 ldw_fault_code_maskcode = 0;
  uint32 ldw_left_suppression_code_maskcode = 0;
  uint32 ldw_left_kickdown_code_maskcode = 0;
  uint32 ldw_right_suppression_code_maskcode = 0;
  uint32 ldw_right_kickdown_code_maskcode = 0;
  uint32 elk_fault_code_maskcode = 0;
  uint32 elk_enable_code_maskcode = 0;
  uint32 elk_disable_code_maskcode = 0;
  uint32 elk_left_suppression_code_maskcode = 0;
  uint32 elk_left_kickdown_code_maskcode = 0;
  uint32 elk_right_suppression_code_maskcode = 0;
  uint32 elk_right_kickdown_code_maskcode = 0;
  // 打断纠偏的横向速度持续时间，单位：S
  double ldp_kickdown_lat_v_dur = 3.0;
  // 抑制报警的驾驶员手力矩(绝对值)阈值，单位：Nm
  double LDP_suppression_driver_hand_trq = 2.0;
  // 打断报警的驾驶员手力矩(绝对值)阈值，单位：Nm
  double LDP_kickdown_samedir_hand_trq = 2.3;
  // 打断报警的驾驶员手力矩(绝对值)阈值，单位：Nm
  double LDP_kickdown_oppodir_hand_trq = 2.5;
  // 打断报警的驾驶员手力矩(绝对值)阈值，单位：Nm
  double LDP_kickdown_abs_hand_trq = 1.5;
  // 打断纠偏的手力矩绝对值持续时间，单位：S
  double LDP_kickdown_hand_trq_dur = 0.5;
  // 抑制报警的驾驶员手力矩(绝对值)阈值，单位：Nm
  double ELK_suppression_driver_hand_trq = 2.0;
  // 打断报警的驾驶员手力矩(绝对值)阈值，单位：Nm
  double ELK_kickdown_samedir_hand_trq = 2.3;
  // 打断报警的驾驶员手力矩(绝对值)阈值，单位：Nm
  double ELK_kickdown_oppodir_hand_trq = 2.5;
  // 打断报警的驾驶员手力矩(绝对值)阈值，单位：Nm
  double ELK_kickdown_abs_hand_trq = 1.5;
  // 打断纠偏的手力矩绝对值持续时间，单位：S
  double ELK_kickdown_hand_trq_dur = 0.5;
};

struct StateInfo {
  double current_time_us = 0.0;  // 当前时间

  double vehicle_speed = 0.0;          // 本车实际车速 单位:m/s
  double display_vehicle_speed = 0.0;  // 本车实际车速 单位:m/s
  double yaw_rate = 0.0;               // 本车横摆角速度 单位:rad/s
  double yaw_rate_observer = 0.0;      // 本车横摆角速度 单位:rad/s
  double yaw_rate_loc = 0.0;

  // 根据本车yaw_rate和方向盘转角,计算得到的本车当前行驶曲率
  double ego_curvature = 0.0;

  // 左转向灯处于关闭状态的时长(最大值为60) 单位:s
  double left_turn_light_off_time = 0.0;

  // 右转向灯处于关闭状态的时长(最大值为60) 单位:s
  double right_turn_light_off_time = 0.0;

  double fl_wheel_distance_to_line = 0.0;

  double fr_wheel_distance_to_line = 0.0;

  double fl_wheel_distance_to_roadedge = 0.0;

  double fr_wheel_distance_to_roadedge = 0.0;

  // 驾驶员手力矩，单位 Nm
  double driver_hand_trq = 0.0;

  // 左、右偏离车道线速度 单位 m/s 左正右负
  double veh_left_departure_speed = 0.0;
  double veh_right_departure_speed = 0.0;

  // 方向盘转角 左正右负 单位 degree
  double ctrl_output_steering_angle = 0.0;
  double steer_wheel_angle_degree = 0.0;
  // 实际加速踏板开度百分比 范围:[0-100]
  double accelerator_pedal_pos = 0.0;
  // 实际制动踏板开度百分比 范围:[0-100] 不是所有车均有此信号
  double brake_pedal_pos = 0.0;
  bool brake_pedal_pressed = false;
  // enu2car 变换矩阵
  Eigen::Vector2d current_pos_i;
  Eigen::Matrix2d rotm2d = Eigen::Matrix2d::Identity();
  // 偏离加速度
  double lat_departure_acc = 0.0;
  double accelerator_pedal_pos_rate = 0.0;  // 油门踏板速率 %/s

  // 定义当前挡位值
  iflyauto::ShiftLeverStateEnum shift_lever_state =
      iflyauto::ShiftLeverStateEnum::ShiftLeverState_P;

  // vehicle_service模块节点通讯丢失
  // 1：模块节点通讯未丢失 ， 0：模块节点通讯丢失
  bool vehicle_service_node_valid = true;
  // 车道线融合模块节点通讯丢失
  //  1：模块节点通讯未丢失 ， 0：模块节点通讯丢失
  bool road_info_node_valid = true;
  // 定位模块节点通讯丢失
  //  1：模块节点通讯未丢失 ， 0：模块节点通讯丢失
  bool localization_info_node_valid = true;
};

enum Enum_LineType {
  Enum_LineType_Virtual = 0,  // 虚拟的
  Enum_LineType_Dashed = 1,   // 虚线
  Enum_LineType_Solid = 2,    // 实线
  // Enum_LineType_RoadEdge = 3,  // 路沿
  Enum_LineType_SelfSet = 3,  // 自定
  Enum_LineType_Other = 4,    // 其他
};

// enum Enum_DistanceToBordeType {
//   Enum_Y_ToLeftBorder = 0,   // 虚拟的
//   Enum_Y_ToLeftLine = 1,     // 虚线
//   Enum_Y_ToRightBorder = 2,  // 实线
//   Enum_Y_ToRightLine = 3,    // 路沿
// };

struct LineInfo {
  Enum_LineType line_type;

  // 0:未知 1:虚线 2:实线 3~10:其他类型
  iflyauto::LaneBoundaryType boundary_type;  // 感知提供的结果

  double c0;                    // 车道线方程系数c0 y=c0+c1*x+c2*x*x +c3*x*x*x
  double c1;                    // 车道线方程系数c1
  double c2;                    // 车道线方程系数c3
  double c3;                    // 车道线方程系数c3
  std::vector<double> dx_vec_;  // 存储车道线散点 x坐标值
  std::vector<double> dy_vec_;  // 存储车道线散点 y坐标值
  std::vector<double> s_vec_;   // 存储车道线散点 起始点s值为0
  pnc::mathlib::spline dx_s_spline_;
  pnc::mathlib::spline dy_s_spline_;
  double begin;    // 车道线起始点 x坐标值
  double end;      // 车道线终点 x坐标值
  double begin_s;  // 车道起始点 s值
  double end_s;    // 车道终点 s值
  bool valid;
  double segment0_length = 0.0;
  double segment1_length = 0.0;
  double segment2_length = 0.0;
  double segment3_length = 0.0;
  iflyauto::LaneBoundaryType segment0_type;  // 感知提供的结果
  iflyauto::LaneBoundaryType segment1_type;  // 感知提供的结果
  iflyauto::LaneBoundaryType segment2_type;  // 感知提供的结果
  iflyauto::LaneBoundaryType segment3_type;  // 感知提供的结果
};

struct RoadedgeInfo {
  std::vector<double> dx_vec_;  // 存储路沿线有效散点 x坐标值
  std::vector<double> dy_vec_;  // 存储路沿线有效散点 y坐标值
  std::vector<double> s_vec_;   // 存储路沿线有效散点 起始点s值为0
  pnc::mathlib::spline dx_s_spline_;
  pnc::mathlib::spline dy_s_spline_;
  double begin_x;  // 车道路沿起始点 x坐标值
  double end_x;    // 车道路沿终点 x坐标值
  double begin_s;  // 车道路沿起始点 s值
  double end_s;    // 车道路沿终点 s值
  // double distance_to_border;
  bool valid;
  std::vector<double> all_dx_vec_;  // 左侧路沿所有点x坐标
  std::vector<double> all_dy_vec_;  // 左侧路沿所有点y坐标
};

struct LaneInfo {
  LineInfo left_line;
  LineInfo right_line;
  RoadedgeInfo left_roadedge;
  RoadedgeInfo right_roadedge;
  double lane_width;
  bool lane_width_valid;
  bool lane_changed_flag;
  bool left_sideway_exist_flag;
  bool right_sideway_exist_flag;
  // bool left_safe_departure_permission_flag = false;
  // bool right_safe_departure_permission_flag = false;
  bool left_parallel_car_flag = false;
  bool right_parallel_car_flag = false;
  bool right_front_car_flag = false;
  bool left_front_car_flag = false;
};

struct RoadInfo {
  LaneInfo current_lane;
};

struct LastCycleInfo {
  bool left_turn_light_state = false;   // 左转向灯状态  false:关闭 true:开启
  bool right_turn_light_state = false;  // 右转向灯状态  false:关闭 true:开启
  double yaw_rad = 0.0;                 // 定位yaw角
  double accelerator_pedal_pos = 0.0;   // 实际加速踏板开度百分比 范围:[0-100]
};

typedef enum {
  SPEED_SIGN_TYPE_MAXIMUM_SPEED = 0,
  SPEED_SIGN_TYPE_MINIMUM_SPEED = 1,
  SPEED_SIGN_TYPE_END_OF_SPEED_LIMIT = 2,
  SPEED_SIGN_TYPE_UNKNOWN = 3,
} _ENUM_PACKED_ SpeedSignType;

// 限速标识牌信息，包括限速，解除限速
struct SpeedSignInfo {
  uint64 isp_timestamp;           // 图像曝光中间时刻时间戳    (微秒)
  uint8_t id;                     // 跟踪id号
  SpeedSignType speed_sign_type;  // 限速标志牌类型
  // 是否为匝道限速牌
  bool ramp_flag = false;
  double supp_sign_x;  // 限速标志牌纵向距离 (m)
  double supp_sign_y;  // 限速标志牌横向距离 (m)
  double supp_sign_z;  // 限速标志牌高度     (m)
  double
      speed_limit;  // 限速速度值(km/h)
                    // 仅在标识牌类型为【MAXIMUM_SPEED】、【MINIMUM_SPEED】、【END_OF_SPEED_LIMIT】时对该字段赋值,否则该字段默认赋值为0
};

// 按照优先级定义
typedef enum {
  SUPP_SIGN_TYPE_YIELD_SIGN = 0,                  // 让行标识
  SUPP_SIGN_TYPE_STOP_SIGN = 1,                   // 停车标识
  SUPP_SIGN_TYPE_PROHIBIT_PROLONGED_PARKING = 2,  // 禁止长时间停车
  SUPP_SIGN_TYPE_NO_PARKING = 3,                  // 禁止停车
  SUPP_SIGN_TYPE_NO_OVERTAKING = 4,               // 禁止超车
  SUPP_SIGN_TYPE_CANCEL_NO_OVERTAKING = 5,        // 解除禁止超车
  SUPP_SIGN_TYPE_NO_ENTRY = 6,                    // 禁止驶入
  SUPP_SIGN_TYPE_PROHIBIT_MOTOR_ENTERING = 7,     // 禁止机动车驶入
  SUPP_SIGN_TYPE_PROHIBIT_TURN_U = 8,             // 禁止掉头
  SUPP_SIGN_TYPE_PROHIBIT_TURN_RIGHT = 9,         // 禁止右转
  SUPP_SIGN_TYPE_PROHIBIT_TURN_LEFT = 10,         // 禁止左转
  SUPP_SIGN_TYPE_NO_PASSING = 11,                 // 禁止通行
  SUPP_SIGN_TYPE_UNKNOWN = 12,                    // 未知类型
} _ENUM_PACKED_ SuppSignType;

// 辅助标识牌信息
struct SuppSignInfo {
  uint64 isp_timestamp;         // 图像曝光中间时刻时间戳    (微秒)
  uint8_t id;                   // 跟踪id号
  SuppSignType supp_sign_type;  // 辅助标志牌类型
  double supp_sign_x;           // 辅助标志牌纵向距离 (m)
  double supp_sign_y;           // 辅助标志牌横向距离 (m)
  double supp_sign_z;           // 辅助标志牌高度     (m)
};

// 道路标识信息, 包含多个限速标识牌和多个辅助标识牌
struct TsrInfo {
  std::vector<SpeedSignInfo> speed_sign_info_vector;
  std::vector<SuppSignInfo> supp_sign_info_vector;
};

struct LdwOutputInfo {
  iflyauto::LDWFunctionFSMWorkState ldw_state_{
      iflyauto::LDW_FUNCTION_FSM_WORK_STATE_OFF}; /* LDW功能状态
0:Unavailable 1:Off 2:Standby 3:Active(No Intervention) 4:Active(Left
Intervention) 5:Active(Right Intervention) */
  bool ldw_left_warning_{
      false}; /* LDW功能触发左侧报警标志位 0:No Warning 1:Left Warning */
  bool ldw_right_warning_{
      false}; /* LDW功能触发右侧报警标志位 0:No Warning 1:Rirht Warning */
};

struct LdpOutputInfo {
  iflyauto::LDPFunctionFSMWorkState ldp_state_{
      iflyauto::LDP_FUNCTION_FSM_WORK_STATE_OFF}; /* LDP功能状态
0:Unavailable 1:Off 2:Standby 3:Active(No Intervention) 4:Active(Left
Intervention) 5:Active(Right Intervention) */
  bool ldp_left_intervention_flag_{
      false}; /* LDP功能触发左侧报警标志位 0:No
                 Intervention 1:Left Intervention */
  bool ldp_right_intervention_flag_{
      false}; /* LDP功能触发右侧报警标志位 0:No Intervention 1:Rirht
                 Intervention */
};

struct ElkOutputInfo {
  iflyauto::ELKFunctionFSMWorkState elk_state_{
      iflyauto::ELK_FUNCTION_FSM_WORK_STATE_OFF}; /* ELK功能状态
0:Unavailable 1:Off 2:Standby 3:Active(No Intervention) 4:Active(Left
Intervention) 5:Active(Right Intervention) */
  bool elk_left_intervention_flag_{
      false}; /* ELK功能触发左侧报警标志位 0:No
                 Intervention 1:Left Intervention */
  bool elk_right_intervention_flag_{
      false}; /* ELK功能触发右侧报警标志位 0:No Intervention 1:Rirht*/
};

struct TSROutputInfo {
  iflyauto::TSRFunctionFSMWorkState tsr_state_;  // TSR功能状态
  uint32 tsr_speed_limit_;  // TSR识别到的限速标识牌    (公里/小时)
  boolean tsr_warning_;  // TSR超速报警标志位 (true:Warning / false:No Warning)

  iflyauto::SuppSignType
      supp_sign_type;  // 辅助标志牌类型,
                       // 和限速牌区分，辅助标识牌是让行，停止之类的标识牌
};

struct IHCOutputInfo {
  iflyauto::IHCFunctionFSMWorkState ihc_state_;  // IHC功能状态
  bool ihc_request_status_;  // IHC请求状态 (true:Request / false:No Request)
  bool ihc_request_;         // IHC请求 (true:HighBeam / false:LowBeam)
};

struct AdasOutputInfo {
  LdwOutputInfo ldw_output_info_;
  LdpOutputInfo ldp_output_info_;
  ElkOutputInfo elk_output_info_;
  TSROutputInfo tsr_output_info_;
  IHCOutputInfo ihc_output_info_;
};

enum Enum_LaneLocType {
  Enum_Left_Lane = 0,     // 障碍物在左侧车道
  Enum_Current_Lane = 1,  // 障碍物在右侧车道
  Enum_Right_Lane = 2,    // 障碍物在当前车道
  Enum_Other = 3,         // 障碍物在其他车道
};

struct FusionObjExtractInfo {
  Enum_LaneLocType obj_loc_in_lane = Enum_LaneLocType::Enum_Other;
  uint8 id;
  uint8 type;
  uint8 fusion_source;
  double timestamp_us;
  double delay_time{0.0};
  // ObstacleIntentType intention;
  bool b_backup_freemove{false};
  double cutin_score;
  double position_x;
  double position_y;
  double length;
  double width;
  double speed;
  double yaw;    // for obj
  double theta;  // for velocity
  double acc;
  uint8_t confidence;
  uint16_t track_age;
  uint32_t track_status;  // 0-illegal, 1-new,2-measure, 3-predict, 4-fusion
  // add relative info for highway
  double relative_position_x;
  double relative_position_y;
  double relative_position_x_up;
  double relative_position_y_left;
  double relative_position_x_down;
  double relative_position_y_right;
  double relative_speed_x;
  double relative_speed_y;
  double relative_acceleration_x;
  double relative_acceleration_y;
  double acceleration_relative_to_ground_x;
  double acceleration_relative_to_ground_y;
  double relative_theta;
};
struct SingleAreaObjs {
  bool vehicle_info_valid = false;
  FusionObjExtractInfo vehicle_info;
  bool person_info_valid = false;
  FusionObjExtractInfo person_info;
};

struct SelectedAreasObjs {
  SingleAreaObjs fl_objs;
  SingleAreaObjs fm_objs;
  SingleAreaObjs fr_objs;
  SingleAreaObjs ml_objs;
  SingleAreaObjs mr_objs;
  SingleAreaObjs rl_objs;
  SingleAreaObjs rm_objs;
  SingleAreaObjs rr_objs;
};

struct ObjectsInfo {
  std::vector<FusionObjExtractInfo> all_objs_vector;
  SelectedAreasObjs objs_selected;
  std::vector<double> ego_around_area_x_vector;
};

}  // namespace context
}  // namespace adas_function

#endif