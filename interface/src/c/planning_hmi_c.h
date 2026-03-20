// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_PLANNING_HMI_H_
#define _IFLYAUTO_PLANNING_HMI_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"
#include "camera_perception_tsr_c.h"
#include "fusion_road_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#define PLANNING_HMI_AD2HMI_OBSTACLE_MAX_NUM 16
#define PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM 64

#pragma pack(4)

//  LDW功能输出的结构体定义
typedef enum {
  LDW_FUNCTION_FSM_WORK_STATE_UNAVAILABLE = 0,                // Unavailable
  LDW_FUNCTION_FSM_WORK_STATE_OFF = 1,                        // Off
  LDW_FUNCTION_FSM_WORK_STATE_STANDBY = 2,                    // Standby
  LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION = 3,     // Active(No Intervention)
  LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION = 4,   // Active(Left Intervention)
  LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION = 5,  // Active(Right Intervention)
  LDW_FUNCTION_FSM_WORK_STATE_FAULT = 6,                      // Fault
} _ENUM_PACKED_ LDWFunctionFSMWorkState;

typedef struct {
  LDWFunctionFSMWorkState ldw_state;  // LDW功能状态机状态
  boolean ldw_left_warning;           // LDW功能触发左侧报警标志位    (true:Left Warning / false:No Warning)
  boolean ldw_right_warning;          // LDW功能触发右侧报警标志位    (true:Rirht Warning / false:No Warning)
} _STRUCT_ALIGNED_ LDWOutputInfoStr;

//  LDP功能输出的结构体定义
typedef enum {
  LDP_FUNCTION_FSM_WORK_STATE_UNAVAILABLE = 0,                // Unavailable
  LDP_FUNCTION_FSM_WORK_STATE_OFF = 1,                        // Off
  LDP_FUNCTION_FSM_WORK_STATE_STANDBY = 2,                    // Standby
  LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION = 3,     // Active(No Intervention)
  LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION = 4,   // Active(Left Intervention)
  LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION = 5,  // Active(Right Intervention)
  LDP_FUNCTION_FSM_WORK_STATE_FAULT = 6,                      // Fault
} _ENUM_PACKED_ LDPFunctionFSMWorkState;
typedef enum {
  LDP_DRIVER_HANDSOFF_WARNING_OFF = 0,
  LDP_DRIVER_HANDSOFF_WARNING_STAGE1 = 1,
  LDP_DRIVER_HANDSOFF_WARNING_STAGE2 = 2,
  LDP_DRIVER_HANDSOFF_WARNING_STAGE3 = 3,
} _ENUM_PACKED_ LDPDriverhandsoffWarning;
typedef struct {
  LDPFunctionFSMWorkState ldp_state;  // LDP功能状态机状态
  boolean ldp_left_intervention_flag;  // LDP功能触发左侧报警标志位    (true:Left Intervention / false:No Intervention)
  boolean
      ldp_right_intervention_flag;  // LDP功能触发右侧报警标志位    (true:Rirht Intervention / false:No Intervention)
  boolean ldp_warning_audio_flag;   // LDP功能触发语音播报标志位    (true:Warning / false:No Warning)
  LDPDriverhandsoffWarning ldp_driver_handsoff_warning;  // LDP功能触发驾驶员分心警告标志位
} _STRUCT_ALIGNED_ LDPOutputInfoStr;

// ELK功能输出结构体定义
typedef enum {
  ELK_FUNCTION_FSM_WORK_STATE_UNAVAILABLE = 0,                // Unavailable
  ELK_FUNCTION_FSM_WORK_STATE_OFF = 1,                        // Off
  ELK_FUNCTION_FSM_WORK_STATE_STANDBY = 2,                    // Standby
  ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION = 3,     // Active(No Intervention)
  ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION = 4,   // Active(Left Intervention)
  ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION = 5,  // Active(Right Intervention)
  ELK_FUNCTION_FSM_WORK_STATE_FAULT = 6,                      // Fault
} _ENUM_PACKED_ ELKFunctionFSMWorkState;

typedef struct {
  boolean obj_valid;           // false:invalid true:valid
  uint32 id;                   // 障碍物id
  ObjectType type;             // 障碍物类型
  ObjectLightType light_type;  // 障碍物灯光状态
  Shape3d shape;               // 障碍物尺寸
  // Point3f relative_position;         // 障碍物近侧距离自车后轴中心的距离     (米)
  Point3f relative_velocity;         // 障碍物相对自车坐标系相对速度         (米/秒)
  Point3f relative_acceleration;     // 障碍物相对自车坐标系相对加速度 (米/秒^2)
  Point3f relative_center_position;  // 障碍物几何中心距离自车后轴中心的距离 (米)

  /* 障碍物与自车的相对朝向角
     单位：弧度rad
     备注：沿自车坐标系x轴正向逆时针旋转为正（0~Π），顺时针为负（0~-Π）
  */
  float32 relative_heading_angle;
  float32 relative_heading_angle_rate;  // 障碍物与自车的相对朝向角变化率
                                        // (弧度rad/秒)
  // Point3f position;                     // 障碍物距自车近侧在绝对坐标系中的位置 (米)
  // Point3f velocity;                     // 障碍物在绝对坐标系中的速度           (米/秒)
  // Point3f acceleration;                 // 障碍物在绝对坐标系中的加速度         (米/秒^2)
  // Point3f center_position;              // 障碍物几何中心在绝对坐标系的位置     (米)
  // float32 heading_angle;                // 障碍物在绝对坐标系中的朝向角         (弧度rad)
  // float32 heading_angle_rate;           // 障碍物在绝对坐标系中的朝向角变化率 (弧度rad/秒)
} _STRUCT_ALIGNED_ ELKRiskObjInfo;

typedef struct {
  ELKFunctionFSMWorkState elk_state;  // ELK功能状态机状态
  boolean elk_left_intervention_flag;  // ELK功能触发左侧报警标志位    (true:Left Intervention / false:No Intervention)
  boolean
      elk_right_intervention_flag;  // ELK功能触发右侧报警标志位    (true:Rirht Intervention / false:No Intervention)
  ELKRiskObjInfo elk_risk_obj;
} _STRUCT_ALIGNED_ ELKOutputInfoStr;

// TSR功能输出结构体定义
typedef enum {
  TSR_FUNCTION_FSM_WORK_STATE_UNAVAILABLE = 0,  // Unavailable
  TSR_FUNCTION_FSM_WORK_STATE_OFF = 1,          // Off
  TSR_FUNCTION_FSM_WORK_STATE_STANDBY = 2,      // Standby
  TSR_FUNCTION_FSM_WORK_STATE_ACTIVE = 3,       // Active
  TSR_FUNCTION_FSM_WORK_STATE_FAULT = 4,        // Fault
} _ENUM_PACKED_ TSRFunctionFSMWorkState;

typedef struct {
  TSRFunctionFSMWorkState tsr_state;  // TSR功能状态
  uint32 tsr_speed_limit;             // TSR识别到的限速标识牌    (公里/小时)
  boolean tsr_warning;                // TSR超速报警标志位 (true:Warning / false:No Warning)
  SuppSignType tsr_supp_sign_type;    // TSR辅助标志牌类型
  boolean tsr_speed_unlimit_warning;  // TSR取消限速提醒标志位 (true:Warning / false:No Warning)
} _STRUCT_ALIGNED_ TSROutputInfoStr;

// IHC功能输出结构体定义
typedef enum {
  IHC_FUNCTION_FSM_WORK_STATE_UNAVAILABLE = 0,  // Unavailable
  IHC_FUNCTION_FSM_WORK_STATE_OFF = 1,          // Off
  IHC_FUNCTION_FSM_WORK_STATE_STANDBY = 2,      // Standby
  IHC_FUNCTION_FSM_WORK_STATE_ACTIVE = 3,       // Active
  IHC_FUNCTION_FSM_WORK_STATE_FAULT = 4,        // Fault
} _ENUM_PACKED_ IHCFunctionFSMWorkState;

typedef struct {
  IHCFunctionFSMWorkState ihc_state;  // IHC功能状态
  boolean ihc_request_status;         // IHC请求状态  (true:Request / false:No Request)
  boolean ihc_request;                // IHC请求      (true:HighBeam / false:LowBeam)
} _STRUCT_ALIGNED_ IHCOutputInfoStr;

typedef enum {
  AMAP_FUNCTION_FSM_WORK_STATE_INIT = 0,     // Init
  AMAP_FUNCTION_FSM_WORK_STATE_OFF = 1,      // Off
  AMAP_FUNCTION_FSM_WORK_STATE_STANDBY = 2,  // Standby
  AMAP_FUNCTION_FSM_WORK_STATE_ACTIVE = 3,   // Active
  AMAP_FUNCTION_FSM_WORK_STATE_FAULT = 4,    // Fault
} _ENUM_PACKED_ AMAPFunctionFSMWorkState;

typedef struct {
  AMAPFunctionFSMWorkState amap_state;  // AMAP功能状态
  boolean amap_request_flag;            // 0:no request  1:request
  float64 amap_trq_limit_max;           // 单位 : Nm
} _STRUCT_ALIGNED_ AMAPOutputInfoStr;

typedef enum {
  MEB_FUNCTION_FSM_WORK_STATE_INIT = 0,     // Init
  MEB_FUNCTION_FSM_WORK_STATE_OFF = 1,      // Off
  MEB_FUNCTION_FSM_WORK_STATE_STANDBY = 2,  // Standby
  MEB_FUNCTION_FSM_WORK_STATE_ACTIVE = 3,   // Active
  MEB_FUNCTION_FSM_WORK_STATE_FAULT = 4,    // Fault
} _ENUM_PACKED_ MEBFunctionFSMWorkState;

typedef enum {
  MEB_INTERVENTION_DIRECTION_NONE = 0,     //none
  MEB_INTERVENTION_DIRECTION_FRONT = 1,      // front
  MEB_INTERVENTION_DIRECTION_REAR = 2,  // back
} _ENUM_PACKED_ MEBInterventionDirection;

typedef struct {
  MEBFunctionFSMWorkState meb_state;  // MEB功能状态
  uint32 meb_request_status;
  float64 meb_request_value;  // 单位:m/ss
  MEBInterventionDirection meb_request_direction; //触发方向
} _STRUCT_ALIGNED_ MEBOutputInfoStr;

/*  ALC功能输出结构体定义
 *  发送给工控机的可视化工具，做换道可视化
 *  字段含义解释链接
 * http://wiki.iflytek.com/pages/viewpage.action?pageId=517586574
 *  字段稳定后，再改成具体的结构体
 */
typedef struct {
  uint8 lc_request_size;
  uint8 lc_request[PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM];  // 换道请求
  uint8 lc_status_size;
  uint8 lc_status[PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM];  // 换道状态
  uint8 lc_invalid_reason_size;
  uint8 lc_invalid_reason[PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM];  // 无法换道原因
  uint8 lc_back_reason_size;
  uint8 lc_back_reason[PLANNING_HMI_ALC_OUTPUT_INFO_MAX_NUM];  // 换道返回原因
} _STRUCT_ALIGNED_ ALCOutputInfoStr;

typedef struct {
  boolean has_cipv;  // 是否存在cipv (true:yes / false:no)
  int32 cipv_id;     // cipv的id，对应融合障碍物id
                     // <FusionObjectsInfo.FusionObject.FusionAdditional.track_id>
} _STRUCT_ALIGNED_ CIPVInfoStr;

typedef enum {
  TRAFFIC_LIGHT_REMINDER_NONE = 0,
  TRAFFIC_LIGHT_REMINDER_RED_LIGHT_STOP = 1,     // 闯红灯风险提醒
  TRAFFIC_LIGHT_REMINDER_GREEN_LIGHT_START = 2,  // 绿灯未起步提醒
} _ENUM_PACKED_ TrafficLightReminder;

typedef struct {
  TrafficLightReminder traffic_light_reminder;  // 红绿灯提醒：闯红灯风险提醒、绿灯未起步提醒
} _STRUCT_ALIGNED_ TLAOutputInfoStr;

// Lane change direction
typedef enum {
  LC_DIR_NO_CHANGE = 0,  // NO_CHANGE
  LC_DIR_LEFT = 1,       // LEFT_CHANGE
  LC_DIR_RIGHT = 2,      // RIGHT_CHANGE
} _ENUM_PACKED_ LaneChangeDirection;

// Lane change direction
typedef enum {
  RAMP_NONE = 0,
  RAMP_LEFT = 1,
  RAMP_RIGHT = 2,
} _ENUM_PACKED_ RampDirection;

// 变道状态
typedef enum {
  LC_STATE_NO_CHANGE = 0,  // No change
  LC_STATE_WAITING = 1,    // 等待
  LC_STATE_STARTING = 2,   // 变道中
  LC_STATE_CANCELLED = 3,  // 取消
  LC_STATE_COMPLETE = 4,   // 完成
} _ENUM_PACKED_ LaneChangeStatus;

// 变道状态改变的原因
typedef enum {
  STATUS_UPDATE_REASON_NONE = 0,
  STATUS_UPDATE_REASON_SOLID_LINE = 1,     // 实线抑制
  STATUS_UPDATE_REASON_SIDE_VEH = 2,       // 旁车抑制
  STATUS_UPDATE_REASON_MANUAL_CANCEL = 3,  // 手动取消
  STATUS_UPDATE_REASON_TIMEOUT = 4,        // 超时取消
} _ENUM_PACKED_ StatusUpdateReason;

// 发起变道的原因
typedef enum {
  LC_REASON_NONE = 0,
  LC_REASON_MANUAL = 1,       // 手动打灯变道
  LC_REASON_SPLIT = 2,        // 下匝道
  LC_REASON_MERGE = 3,        // 汇入主路
  LC_REASON_SLOWING_VEH = 4,  // 前方车辆慢行
  LC_REASON_NAVIGATION = 5,   // 导航变道
  LC_REASON_CONE = 6,         // 锥桶变道
} _ENUM_PACKED_ LaneChangeReason;

// Obstacle status
typedef enum {
  OBSTACLE_STATUS_NORMAL = 0,     // Normal
  OBSTACLE_STATUS_FOLLOWING = 1,  // Following
  OBSTACLE_STATUS_TOO_CLOSE = 2,  // Too close
  OBSTACLE_STATUS_DANGER = 3,     // Danger
  OBSTACLE_STATUS_OTHER = 4,      // Other
} _ENUM_PACKED_ ObstacleLonStatus;

// Avoid obstacle
typedef enum {
  AVOID_NO_HIDING = 0,       // No hiding
  AVOID_HIDING = 1,          // Hiding
  AVOID_OBSTACLE_OTHER = 2,  // Other
} _ENUM_PACKED_ AvoidObstacle;

//lane borrow
typedef enum {
  NONE_FAILED_REASON = 0,
  CENTER_OBSTACLE,      // 障碍物过于居中，无法绕行
  BACKWARD_CARS,        // 后方来车，绕行暂停
  COMING_CARS,          // 前方来车
  NEIGHBOR_LANE_NULL,   // 旁边无可借车道
  NO_PASSABLE_SPACE,    // 空间不足无法借道
  FRONT_CAR_CUTING_IN,  // 前车切入
  FRONT_CAR_CUTING_OUT  // 前车切出
} _ENUM_PACKED_ LaneBorrowFailedReason;

typedef enum {
  TARGET_NONE = 0,
  WORKING_AREA,//施工区域
  CONES,       //锥桶
  CARS,        //汽车
  VRU,         // VRU
  CRASH_BARREL,// 防撞桶
  WATER_NERREL,//水马
} _ENUM_PACKED_ LaneBorrowTarget;

typedef enum {
  BORROW_NONE = 0,
  BORROW_LEFT = 1,
  BORROW_RIGHT = 2,
} _ENUM_PACKED_ LaneBorrowDirection;

typedef enum {
  LANE_NONE = 0,
  SOLID_LINE,
  DASHED_LINE
} _ENUM_PACKED_ LaneBorrowLaneType;

// Landing point
typedef struct {
  boolean is_avaliable;
  Point3d relative_pos;
  float32 heading;
} _STRUCT_ALIGNED_ LandingPoint;

// Lccs trajectory polyline
typedef struct {
  float32 c0;
  float32 c1;
  float32 c2;
  float32 c3;
  float32 c4;
  float32 c5;
  float32 start_x;
  float32 end_x;
} _STRUCT_ALIGNED_ LCCS_TrajPoly;

// Trajectory point
typedef struct {
  LCCS_TrajPoly start;
  LCCS_TrajPoly middle;
  LCCS_TrajPoly end;
} _STRUCT_ALIGNED_ LCCSTrajectoryPoint;

typedef enum {
  FRONT_LEFT = 0,
  FRONT_MIDDLE = 1,
  FRONT_RIGHT = 2,
  RIGHT_MIDDLE = 3,
  RIGHT_BACK = 4,
  BACK_MIDDLE = 5,
  BACK_LEFT = 6,
  LEFT_MIDDLE = 7,
  MIDDLE = 8,
} _ENUM_PACKED_ AnchorPoint;

// Obstacle info
typedef struct {
  uint32 id;
  float32 speed_x;               // 车体坐标系下, 障碍物X方向速度
  float32 speed_y;               // 车体坐标系下, 障碍物Y方向速度
  float32 heading;               // 障碍物朝向
  ObjectType type;               // 障碍物类型: 骑车人, 卡车, 小车, 公交车, 锥桶
  float32 center_x;              // 车体坐标系下, 障碍物中心X坐标
  float32 center_y;              // 车体坐标系下, 障碍物中心Y坐标
  Shape3f size;                  // 障碍物大小
  ObstacleLonStatus lon_status;  // 障碍物状态
  AnchorPoint anchor;            // 参考点
} _STRUCT_ALIGNED_ ObstacleInfo;

typedef enum {
  NO_BRAKE = 0,
  BRAKE = 1,
} _ENUM_PACKED_ BrakeState;

typedef enum {
  AVOID_NONE = 0,
  AVOID_LEFT = 1,
  AVOID_RIGHT = 2,
} _ENUM_PACKED_ AvoidObstacleDirection;

typedef enum {
  PARKING_PAUSE_OTHER = 0,           // 泊车暂停原因：其他
  PARKING_PAUSE_FOR_STATIC_OBS = 1,  // 轨迹上有水马、锥桶，导致卡停, 其他场景待补充
  PARKING_PAUSE_FOR_BREAK = 2,       // 泊车暂停原因：刹车
  PARKING_PAUSE_BY_DYNAMIC_OBS = 3,  // 动态障碍物卡停，且车辆静止
  PARKING_PAUSE_NONE = 4,            // 无暂停原因
} _ENUM_PACKED_ APAPauseReason;

typedef enum {
  AVOID_OTHER = 0,       // 避让其他类型障碍物
  AVOID_PEDESTRIAN = 2,  // 避让行人
  AVOID_VEHICLE = 3,     // 避让车辆
} _ENUM_PACKED_ AvoidObstacleType;

typedef enum {
  EMERGRENCY_LEVEL_NORMAL = 0,  // 出现正常紧急情况，无法继续
  EMERGRENCY_LEVEL_HIGH = 1,    // 出现危险紧急情况，无法继续
} _ENUM_PACKED_ EmergencyLevel;

// 匝道通过状态
typedef enum {
  RAMP_PASS_STS_NONE = 0,         // 无
  RAMP_PASS_STS_PASS = 1,         // 通过
  RAMP_PASS_STS_READYTOMISS = 2,  // 即将错过最晚下匝道位置，降级并请求接管
  RAMP_PASS_STS_TIMEOUT = 3,      // 下匝道超时,车辆已在最右车道，由于拥堵或其他原因导致下匝道超时
} _ENUM_PACKED_ RampPassSts;

// 路口通过状态
typedef enum {
  INTERSECTION_STRAIGHT = 0,         //路口直行
  INTERSECTION_RED_LIGHT_STOP = 1,   //路口红灯刹停
  INTERSECTION_GREEN_LIGHT_GO = 2,   //路口绿灯起步
} _ENUM_PACKED_ IntersectionPassSts;

//路口位置状态
typedef enum {
  NO_INTERSECTION = 0,            // 非路口
  APPROACH_INTERSECTION = 1,      // 接近路口
  IN_INTERSECTION = 2,            // 在路口内
  OFF_INTERSECTION = 3,           // 离开路口
  INTERSECTIONSTATE_UNKNOWN = 4,  // 未知状态
} _ENUM_PACKED_ IntersectionState;

// 道路类型
typedef enum {
  DRIVING_ROAD_TYPE_NONE = 0,      // 无
  DRIVING_ROAD_TYPE_URBAN = 1,     // 城区
  DRIVING_ROAD_TYPE_HIGHWAY = 2,   // 高速
  DRIVING_ROAD_TYPE_OVERPASS = 3,  // 高架
  DRIVING_ROAD_TYPE_BRIDGE = 4,    // 桥梁
  DRIVING_ROAD_TYPE_COUNTRY = 5,   // 乡道
  DRIVING_ROAD_TYPE_OHTER = 6,     // 其他
} _ENUM_PACKED_ DrivingRoadType;

// 点击车位后，发送泊车预规划状态
typedef enum {
  PREPARE_PLANNING_NONE = 0,       // 空值，正式泊车按钮不能点击
  PREPARE_PLANNING_COMPUTING = 1,  // 计算中，正式泊车按钮不能点击
  PREPARE_PLANNING_SUCCESS = 2,    // 计算成功，泊车按钮可以点击
  PREPARE_PLANNING_FAILED = 3,     // 计算失败，车位变成灰色，播报请选择其他车位
} _ENUM_PACKED_ PreparePlanningState;

typedef enum {
  HPP_NONE = 0,                 // 无
  HPP_CRUISING_TO_PARKING = 1,  // 巡航切泊车
} _ENUM_PACKED_ HPPStateSwitch;

typedef enum {
  CONSTRUCTION_NONE = 0,       // 空值
  CONSTRUCTION_APPROACH = 1,   // 接近施工区域
  CONSTRUCTION_IN = 2,         // 正在施工区域中
  CONSTRUCTION_OFF = 3,        // 离开施工区域
} _ENUM_PACKED_ ConstructionState;

typedef struct {
  ConstructionState construction_state;
  uint32 distance_to_construction;           // 单位是米 离施工区域的距离
} _STRUCT_ALIGNED_ ConstructionInfo;

typedef enum {
  CONEWARNINGACTION_NONE = 0,            // 空值
  CONEWARNINGACTION_NUDGE = 1,           // 避让
  CONEWARNINGACTION_YIELD = 2,           // 减速
  CONEWARNINGACTION_NUDGE_AND_YIELD = 3, // 减速避让
  CONEWARNINGACTION_LANE_BORROW = 4,     // 借道
  CONEWARNINGACTION_LANE_CHANGE = 5      // 变道
} _ENUM_PACKED_ ConeWarningAction;

typedef struct {
  boolean cone_warning;                   // 锥桶提醒
  ConeWarningAction cone_warning_action;  // 锥桶提醒时，自车行为; 预留
} _STRUCT_ALIGNED_ ConeWarningInfo;

typedef enum {
  NSA_DISABLE_REASON_NONE = 0,      // 无
  NSA_DISABLE_REASON_HEADING = 1,   // 角度不满足要求
  NSA_DISABLE_REASON_WIDE = 2,      // 通行空间太宽
  NSA_DISABLE_REASON_NARROW = 3,    // 通行空间太窄
  NSA_DISABLE_REASON_NOT_ON_NARROW = 4,    // 自车未在窄路上
} _ENUM_PACKED_ NSADisableReason;

typedef enum {
  RADS_PAUSE_REASON_NONE = 0,      // 无
  RADS_PAUSE_REASON_BRAKE_BY_OBSTACLE = 1,   //障碍物阻挡刹停
} _ENUM_PACKED_ RADSPauseReason;

typedef enum {
  NSA_COMPLETE_REASON_NONE = 0,       // 无
  NSA_COMPLETE_REASON_NO_NARROW = 1,  // 前方无窄路
  NSA_COMPLETE_REASON_NO_SPACE = 2,   // 前方无通行空间
  NSA_COMPLETE_REASON_DISTANCE_SATISFY = 3,   // 行驶距离满足
} _ENUM_PACKED_ NSACompleteReason;

typedef enum {
  NSA_PAUSE_REASON_UNKNOWN = 0,       // 无
  NSA_PAUSE_REASON_BLOCK = 1,         // 前方遇障碍物
} _ENUM_PACKED_ NSAPauseReason;


typedef enum {
  NO_SPLIT_SPLIT = 0,         // 无分流
  SPLIT_SELECT_TO_LEFT = 1,   // 分流向左行驶
  SPLIT_SELECT_TO_RIGHT = 2,  // 分流向右行驶
  SPLIT_INTERACTIVE_TO_LEFT = 3, // 分流拨杆向左行驶
  SPLIT_INTERACTIVE_TO_RIGHT = 4, // 分流拨杆向左行驶
} _ENUM_PACKED_ SplitSelectDirection;

// Ad to Hmi data
typedef struct {
  boolean is_avaliable;  // 是否可进入lcc        (0:否/1:是)
  uint64 timestamp;      // ms
  LaneChangeDirection lane_change_direction;
  LaneChangeStatus lane_change_status;
  LaneChangeReason lane_change_reason;
  StatusUpdateReason status_update_reason;
  uint32 noa_exit_warning_level_distance;  // 单位是米 驶出地图剩余距离 修改为 导航重点剩余距离
  AvoidObstacle avoid_status;              // 是否在避让状态
  int32 aovid_id;                          // 躲避障碍物id
  boolean is_curva;                        // 弯道限速
  LandingPoint landing_point;              // 变道结束位置点
  LCCSTrajectoryPoint tp;                  // 暂无
  uint8 obstacle_info_size;
  ObstacleInfo obstacle_info[PLANNING_HMI_AD2HMI_OBSTACLE_MAX_NUM];
  int32 cutin_track_id;                // cutin目标物id
  float32 cutin_ttc;                   // 与cutin目标物的碰撞时间
  float32 cruise_speed;                // 最高巡航速度，单位是km/h
  AvoidObstacleDirection avoiddirect;  // 躲避方向，当前正在躲避的障碍物(纵向有重叠且正在躲避)
  uint32 distance_to_ramp;             // 单位是米 离匝道的距离
  uint32 distance_to_split;            // 单位是米 离前方分流的距离
  uint32 distance_to_merge;            // 单位是米 离前方汇流的距离
  uint32 distance_to_toll_station;     // 单位是米 离收费站的距离
  uint32 distance_to_tunnel;           // 单位是米 离隧道的距离
  boolean is_within_hdmap;             // 能否启用NOA功能
  RampDirection ramp_direction;        // 匝道方向
  RampPassSts ramp_pass_sts;           // 匝道通过状态
  IntersectionPassSts intersection_pass_sts;           // 路口通过状态
  uint32 dis_to_reference_line;        // 单位是厘米，自车距车道中心线距离
  float32 angle_to_roaddirection;      // 单位是rad，自车航向与道路方向夹角
  boolean is_in_sdmaproad;             // 是否处于导航路线上
  DrivingRoadType road_type;           // 当前行车所处道路类型
  ReferenceLineMsg reference_line_msg; // 可变车道，变道过程中为目标车道，lane_keeping为自车当前车道
  LaneBorrowLaneType borrow_lane_type; //借道压线类型
  LaneBorrowDirection borrow_direction; //借道方向
  LaneBorrowFailedReason borrow_failed_reason; //借道失败原因
  LaneBorrowTarget borrow_target;       //借道目标
  boolean start_nudging;                //开始借道提示
  ConeWarningInfo cone_warning_info;    // 锥桶提醒信息
  ConstructionInfo construction_info;   // 施工区域信息
  IntersectionState intersection_state; // 路口位置状态
  boolean traffic_light_reminder;       // 红绿灯提醒
  SplitSelectDirection split_select_direction; // 选道方向
} _STRUCT_ALIGNED_ AD2HMIData;

//HPP Planning失败的原因
typedef enum {
  HPP_PLANNING_FAILED_REASON_NONE = 0,            // 无
  HPP_PLANNING_FAILED_REASON_TARGET_PARKING_SPACE_OCCUPIED = 1, // 目标停车位已被占用
} _ENUM_PACKED_ HPPPlanningFailedReason;

typedef struct {
  boolean is_avaliable;                   // 是否可进入hpp        (0:否/1:是)
  float32 distance_to_parking_space;      // 离泊车位的距离       (米)
  AvoidObstacle avoid_status;             // 避让状态
  AvoidObstacleType avoid_obstacle_type;  // 避让类型
  int32 aovid_id;                         // 躲避障碍物id
  boolean is_approaching_turn;            // 是否即将进入弯道     (0:否/1:是)
  boolean is_left_turn;  // 是否为左弯道         (0:否/1:是) （仅当is_approaching_turn为1时有效）
  boolean is_approaching_intersection;   // 是否即将进入路口     (0:否/1:是)
  boolean is_approaching_speed_bumps;    // 是否存在减速带       (0:否/1:是)
  EmergencyLevel emergency_level;        // 紧急情况的等级
  boolean is_target_parking_space_occupied;     // 记忆泊车的目标停车位是否已被占用 (0:否/1:是)
  boolean is_new_parking_space_found;    // 是否发现新的可用车位         (0:否/1:是)
  boolean is_on_hpp_lane;                // 是否在路线上          (0:否/1:是)
  boolean is_reached_hpp_trace_start;    // 是否到达记忆泊车起点   (0:否/1:是)
  float32 accumulated_driving_distance;  // 已行驶距离             (米)
  float32 estimated_remaining_time;      // 到达终点的预估时间     (秒)
  HPPStateSwitch hpp_state_switch;       // 通知状态机巡航切泊车状态
  HPPPlanningFailedReason hpp_planning_failed_reason; // HPP Planning失败的原因
} _STRUCT_ALIGNED_ HPPHMIData;

typedef struct {
  float32 remain_dist;                      // 当前路径可以行驶的距离 (米)
  boolean is_parking_pause;                 // 泊车是否暂停     (0:不暂停/1:暂停)
  APAPauseReason parking_pause_reason;      // 泊车暂停原因
  PreparePlanningState prepare_plan_state;  // 预规划状态

  /* planning计算泊车方向
   * 按位划分，每位上1表示可泊，0表示不可泊
   * bit 0: 校验位，1有效,0无效
   * 1-8:出库方向
   * bit 1: 垂直前左
   * bit 2: 水平前左
   * bit 3: 前
   * bit 4: 垂直前右
   * bi5 5: 水平前右
   * bit 6: 后
   * bit 7: 垂直后左
   * bit 8: 垂直后右
   * 11-14：入库方向
   * bit 11: 垂直车位车头入库
   * bit 12: 垂直车位车尾入库
   * bit 13: 平行车位，同向入库 (预留)
   * bit 14: 平行车位，掉头入库 (预留)
   */
  uint16 planning_park_dir;
  uint16 planning_recommend_park_dir;

  boolean recommend_park_out;            // 后台泊出推荐     (0:不泊出/1:泊出)
  boolean recommend_park_in;             // 后台泊入推荐     (0:不泊入/1:泊入)

  float32 pa_remain_distance;               // 贴边全部剩余距离
  float32 remain_distance_percentage;       // 贴边全部剩余距离百分比
  /* planning预规划计算的可贴边方向
   * 按位划分，每位上1表示可泊，0表示不可泊
   * bit 0: 校验位，1有效,0无效
   * bit 1: 左贴边
   * bit 2: 右贴边
   */
  uint16 planning_park_pa_dir;
  /* planning推荐的可贴边方向
   * 按位划分，每位上1表示可泊，0表示不可泊
   * bit 0: 校验位，1有效,0无效
   * bit 1: 左贴边
   * bit 2: 右贴边
   */
  uint16 planning_recommend_pa_dir;
} _STRUCT_ALIGNED_ APAHMIData;


typedef struct {
  boolean is_avaliable;                   // 是否可进入nsa        (0:否/1:是)
  NSADisableReason nsa_disable_reason;    // 无法进入nsa原因
  boolean is_complete;                    // nsa是否完成          (0:未完成 /1:完成) (弃用)
  NSACompleteReason nsa_complete_reason;  // nsa完成原因          (0:未完成 /1:完成)
  NSAPauseReason nsa_pause_reason;        // 自车停止原因
} _STRUCT_ALIGNED_ NSAHMIData;

typedef struct {
  boolean is_avaliable;               // 是否可进入rads        (0:否/1:是)
  RADSPauseReason rads_pause_reason;  // rads中途停止原因
  AvoidObstacle avoid_status;         // 避让状态
  int32 aovid_id;                     // 躲避障碍物id
} _STRUCT_ALIGNED_ RADSHMIData;

// PlanningHMI模块输出结构体定义
typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  LDWOutputInfoStr ldw_output_info;
  LDPOutputInfoStr ldp_output_info;
  ELKOutputInfoStr elk_output_info;
  TSROutputInfoStr tsr_output_info;
  IHCOutputInfoStr ihc_output_info;
  AMAPOutputInfoStr amap_output_info;
  MEBOutputInfoStr meb_output_info;
  ALCOutputInfoStr alc_output_info;
  CIPVInfoStr cipv_info;  // cipv信息
  TLAOutputInfoStr tla_output_info;
  AD2HMIData ad_info;
  HPPHMIData hpp_info;
  APAHMIData apa_info;
  NSAHMIData nsa_info;
  RADSHMIData rads_info;
} _STRUCT_ALIGNED_ PlanningHMIOutputInfoStr;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_PLANNING_HMI_H_