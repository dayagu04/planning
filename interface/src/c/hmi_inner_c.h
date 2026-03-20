// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_HMI_INNER_H_
#define _IFLYAUTO_HMI_INNER_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"
#include "vehicle_service_c.h"
#include "camera_perception_parking_slot_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#define FREE_SLOT_REGION_NUM 6

#pragma pack(4)

// noa语音提示开关反馈
typedef enum {
  PROMPT_INVALID = 0,
  PROMPT_SAFE = 1,     // 安全
  PROMPT_CONCISE = 2,  // 简洁
  PROMPT_CLOSE = 3,    // 贴心
  PROMPT_OFF = 4,      // 关闭
} _ENUM_PACKED_ NoaVoicePromptSwitch;

typedef enum {
  SENSITIVITY_LEVEL_INVALID = 0,  // 无效
  SENSITIVITY_LEVEL_LOW = 1,      // 低
  SENSITIVITY_LEVEL_MIDDLE = 2,   // 中
  SENSITIVITY_LEVEL_HIGH = 3,     // 高
} _ENUM_PACKED_ SensitivityLevel;

typedef enum {
  PARKING_DIRECTION_INVALID = 0,
  BACK_END_PARKING_DIRECTION = 1,    // 车尾泊入
  FRONT_END_PARKING_DIRECTION = 2,   // 车头泊入
} _ENUM_PACKED_ ApaParkingDirection;

typedef enum {
  PRK_OUT_DIRECTION_INVALID = 0,
  PRK_OUT_TO_FRONT_LEFT_CROSS = 1,    // 垂直前左
  PRK_OUT_TO_FRONT_LEFT_PARALLEL = 2, // 水平前左
  PRK_OUT_TO_FRONT_OUT = 3,           // 前
  PRK_OUT_TO_FRONT_RIGHT_CROSS = 4,   // 垂直前右
  PRK_OUT_TO_FRONT_RIGHT_PARALLEL = 5,// 水平前右
  PRK_OUT_TO_BACK_OUT = 6,            // 后
  PRK_OUT_TO_BACK_LEFT_CROSS = 7,     // 垂直后左
  PRK_OUT_TO_BACK_RIGHT_CROSS = 8,    // 垂直后右
} _ENUM_PACKED_ ApaParkOutDirection;

typedef enum {
  APA_WORK_MODE_INVALID = 0,
  APA_WORK_MODE_PARKING_IN = 1,    // 泊入
  APA_WORK_MODE_PARKING_OUT = 2,   // 泊出
  APA_WORK_MODE_RCP_IN = 3,        // RCP泊入
  APA_WORK_MODE_RCP_OUT = 4,       // RCP泊出
  APA_WORK_MODE_RCP_STRAIGHT = 5,  // 直入直出
  APA_WORK_MODEE_EPA = 6,          // 离车泊入
  APA_WORK_MODE_PA = 7,            // 一键贴边
  APA_WORK_MODE_NRA = 8,           // 窄路通行
  APA_WORK_MODE_FTBA = 9,          // 循迹倒车
  APA_WORK_MODE_HPA = 10,          // 记忆泊车
  APA_WORK_MODEE_SAPA = 11,        // 自选车位
} _ENUM_PACKED_ ApaWorkMode;

typedef enum {
  SELECT_SOURCE_INVALID = 0, 
  SELECT_SOURCE_SWITCH = 1,     //软开关                           
  SELECT_SOURCE_VOICE = 2,      //语音                              
  SELECT_SOURCE_RESERVED = 3,   //预留
} _ENUM_PACKED_ SelectSource; 

typedef enum {
  HORIZONTAL_PREFERENCE_INVALID = 0,
  HORIZONTAL_PREFERENCE_SLOT_MID = 1,     //居中                           
  HORIZONTAL_PREFERENCE_SLOT_LEFT = 2,    //偏左                              
  HORIZONTAL_PREFERENCE_SLOT_RIGHT = 3,   //偏右
} _ENUM_PACKED_ HorizontalPreferenceSlot; 

typedef enum {
  VERTICAL_PREFERENCE_INVALID = 0,
  VERTICAL_PREFERENCE_SLOT_MID = 1,     //居中                           
  VERTICAL_PREFERENCE_SLOT_FRONT = 2,   //偏前                          
  VERTICAL_PREFERENCE_SLOT_BACK = 3,    //偏后
} _ENUM_PACKED_ VerticalPreferenceSlot; 

typedef enum {
    FREE_SLOT_SELECTED_STATUS_DEFAULT = 0,   // 默认状态
    FREE_SLOT_SELECTED_STATUS_DRAGING = 1,   // 拖动中
    FREE_SLOT_SELECTED_STATUS_FINISHED = 2,  // 拖动完成
} _ENUM_PACKED_ FreeSlotSelectedStatus;

typedef struct {
  SelectSource select_source; // 信号来源
  HorizontalPreferenceSlot horizontal_preference_slot; // 停车位置偏好(左右)
  VerticalPreferenceSlot vertical_preference_slot;     // 停车位置偏好(前后）
} _STRUCT_ALIGNED_ ApaUserPreference;

typedef struct {
    FreeSlotSelectedStatus is_free_slot_selected;                             // 车位的拖动状态
    boolean free_slot_activate;                                               // 是否激活自定义泊车状态
    ParkingSlotType type;                                                     // 车位类型
    Point2f corner_points[CAMERA_PERCEPTION_PARKING_SLOT_CORNER_POINTS_NUM];  // 车位角点信息     (米)  <固定4个>
    boolean collision_dete[FREE_SLOT_REGION_NUM];                             // 车位区域占用状态 
} _STRUCT_ALIGNED_ ApaFreeSlotInfo;

typedef enum {
  SIMPLE_MAIN_SWITCH_NONE = 0, // 无输出
  SIMPLE_MAIN_SWITCH_OFF = 1,  // 关闭
  SIMPLE_MAIN_SWITCH_ON = 2,   // 打开
} _ENUM_PACKED_ SimpleMainSwitch;

// 提醒功能开关设置
typedef enum {
  NOTIFICATION_MAIN_SWITCH_NONE = 0,              // 无输出
  NOTIFICATION_MAIN_SWITCH_OFF = 1,               // 关闭
  NOTIFICATION_MAIN_SWITCH_VISUAL_ONLY = 2,       // 仅显示
  NOTIFICATION_MAIN_SWITCH_VISUAL_AND_AUDIO = 3,  // 显示+声音提醒
} _ENUM_PACKED_ NotificationMainSwitch;

// 泊车速度设置
typedef enum {
  PARKING_SPEED_SET_NONE = 0,   // 无输出
  PARKING_SPEED_SET_SLOW = 1,   // 慢
  PARKING_SPEED_SET_NORMAL = 2, // 正常
  PARKING_SPEED_SET_FAST = 3,   // 快速 
} _ENUM_PACKED_ ParkingSpeedSet;

typedef struct {
  SimpleMainSwitch apa_main_switch;            // apa功能软开关    (true:on / false:off)
  boolean apa_active_switch;                   // apa激活信号      (true:open acc / false:do nothing)
  ApaWorkMode apa_work_mode;                   // 泊车模式
  boolean apa_cancel_switch;                   // apa取消信号      (true:open acc / false:do nothing)
  uint32 apa_select_slot_id;                   // 选择车位id
  boolean apa_start;                           // apa开始信号      (true:open acc / false:do nothing)
  boolean apa_resume;                          // apa恢复信号      (true:open acc / false:do nothing)
  ApaParkingDirection apa_parking_direction;   // 泊入方向
  ApaParkOutDirection apa_park_out_direction;  // 泊出方向
  SimpleMainSwitch apa_avm_main_switch;        // 泊车avm开关      (ture: on / false: no request)
  ApaUserPreference apa_user_preference;       // 泊车用户偏好
  ApaFreeSlotInfo apa_free_slot_info;          // 自定义泊车
  boolean stop_parking;                        // 停止泊车
  boolean outside_parking;                     // 离车泊入
  boolean change_slot;                         // 更换车位
  boolean sapa_start;                          // 激活自选车位
  boolean sapa_cancel;                         // 退出自选车位
  SimpleMainSwitch apa_park_out_active_switch; // apa 泊出激活信号 (true:open / false:do nothing)
  boolean switch_parking_direction;            // 切换泊入方向     (ture: on / false: no request)
  ParkingSpeedSet parking_speed_set;           // 泊车速度设置
} _STRUCT_ALIGNED_ APAIn;

typedef enum {
  RPA_DEVICE_NO_FAILURE = 0,     // 无故障
  RPA_DEVICE_OFFLINE = 1,        // 离线
  RPA_DEVICE_LOWBATT = 2,        // 低电量
  RPA_DEVICE_OTHER_FAILURES = 3, // 其他故障
  RPA_DEVICE_RESERVED1 = 4,      // 4-7保留
  RPA_DEVICE_RESERVED2 = 5,
  RPA_DEVICE_RESERVED3 = 6,
  RPA_DEVICE_RESERVED4 = 7,
} _ENUM_PACKED_ RpaDevicefailSts; 

typedef enum {
  RPA_MOD_NOT_SELECT = 0,     // 未选择
  RPA_MOD_IN_ON = 1,          // 遥控泊入
  RPA_MOD_OUT_ON = 2,         // 遥控拨出
  RPA_MOD_BAFA_ON = 3,        // 直入直出
  RPA_MOD_HPA_RPA_IN_ON = 4,  // 记忆泊车遥控泊入
  RPA_MOD_HPA_RPA_OUT_ON = 5, // 记忆泊车遥控泊出
  RPA_MOD_RESERVED1 = 6,      // 保留
  RPA_MOD_RESERVED2 = 7,      // 保留
} _ENUM_PACKED_ RpaModselect;   

typedef enum {
  RPA_DEVICE_DIS_STS_NOT_VALID = 0,    // 无效状态
  RPA_DEVICE_DIS_STS_IN_THE_CAR = 1,   // 在车内状态
  RPA_DEVICE_DIS_STS_NORMAL_RANGE = 2, // 正常范围状态
  RPA_DEVICE_DIS_STS_OUT_OF_RANGE = 3, // 超出范围状态
} _ENUM_PACKED_ RpaDeviceDisSts; 

typedef enum {
  BLE_CONN_STS_DEFAULT = 0,        // 默认状态
  BLE_CONN_STS_NOT_CONNECTION = 1, // 未连接状态
  BLE_CONN_STS_CONNECTION = 2,     // 已连接状态
  BLE_CONN_STS_RESERVED = 3,       // 保留状态
} _ENUM_PACKED_ BLEConnSts; 

typedef enum {
  RPA_SWITCH_NONE = 0,                    // NONE状态
  RPA_SWITCH_ON = 1,                      // ON状态
  RPA_SWITCH_RPA = 2,                     // RPA状态
  RPA_SWITCH_RPA_WITHOUT_SUPERVISON = 3,  // RPA WITHOUT SUPERVISON状态
  RPA_SWITCH_CANCEL = 4                   // CANCLE状态
} _ENUM_PACKED_ RpaSwitchReq; 

typedef struct {
  boolean bncm_chanllenge_result;       // ADCC功能安全校验反馈
  boolean bncm_heartbeat_result;        // ADCC功能心跳校验反馈
  RpaDevicefailSts rpa_device_fail_sts; // 蓝牙故障状态反馈
  boolean rpa_mod_req;                  // 进入遥控泊车模式请求 （true: request; false: no request）
  RpaModselect rpa_mod_select;          // 遥控泊车模式
  boolean rpa_btn_sts;                  // RPA开关控制状态 (true: pressed on; false: no pressed)
  boolean straight_in_btn_sts;          // 直入功能开关控制状态 (true: pressed on; false: no pressed)
  boolean straight_out_btn_sts;         // 直出功能开关控制状态 (true: pressed on; false: no pressed)
  boolean rpa_btn_cancel_sts;           // RPA功能取消开关控制状态 (true: pressed on; false: no pressed)
  ApaParkOutDirection prk_out_mod_sel_rmt;  // 泊出方向选择
  uint32 select_slot_id;                // 选择的车位id
  boolean heart_beat_signal;            // 蓝牙心跳信号 (true: active; false: not active)
  RpaDeviceDisSts rpa_device_dis_sts;   // 蓝牙设备距离状态反馈 
  BLEConnSts ble_conn_sts;              // 蓝牙连接状态
  boolean ble_err_status;               // 蓝牙故障状态
  boolean addc_fusa_sts;                // BNCM 反馈addc challenge 结果状态， 0:成功 1：失败
  SimpleMainSwitch rpa_main_switch;     // rpa开关控制状态
  boolean rpa_pause_switch;             // 对应用户按下暂停泊车按钮
  boolean rpa_resume_switch;            // 对应用户按下恢复泊车按钮
  RpaSwitchReq rpa_switch_req;          // RPA状态位
} _STRUCT_ALIGNED_ RPAIn;

typedef enum {
  DEFAUTL_CRUISE_MODE = 0,        // 不切换
  SIDEROAD_PARKING_MODE = 1,      // 切换为沿途泊车模式
  FIXED_ROUTE_PARKING_MODE = 2,   // 切换为固定路线泊车模式
} _ENUM_PACKED_ CruiseModeSwitch; 

typedef struct {
  SimpleMainSwitch hpp_main_switch; // HPP主开关
  boolean hpp_cancel_switch;        // HPP取消开关
  boolean hpp_active_switch;        // HPP激活按键
  boolean start_memory_parking;     // 开始记忆巡航
  boolean continue_memory_parking;  // 恢复记忆巡航
  boolean stop_memory_parking;      // 停止记忆巡航
  boolean start_route_learning;     // 开始路径学习
  boolean complete_route_learning;  // 完成路径学习
  boolean stop_route_learning;      // 停止路径学习
  boolean resume_location;          // 重定位
  boolean save_route;               // 保存路线
  boolean cancel_route;             // 取消保存
  CruiseModeSwitch cruise_mode_switch;    // 巡航模式切换
  uint8 need_gnss_loss_signal;            // 是否需要消星，用于研测可视化模拟消星点触发 0:默认值 1:不需要 2:需要 
  uint32 target_prk_space_id;       // （奔腾预留）目标车位ID
  boolean try_it_now_request;         // 三步引导开始命令
} _STRUCT_ALIGNED_ HPPIn;

typedef enum {
  EHP_NONE_ACTION = 0,             // 无
  EHP_QUERY_MAP = 1,               // 查询地图列表
  EHP_DELETE_MAP = 2,              // 删除地图
  EHP_UPDATE_MAP = 3,              // 更新地图
  EHP_QUERY_SPECIFIC_MAP = 4,      // 查询指定地图
} _ENUM_PACKED_ EhpAction; 

typedef struct {
  uint8 map_file_id;                // 删除地图ID
  char filename[PROTO_STR_LEN];     // 更新地图名
} _STRUCT_ALIGNED_ EhpParam;

typedef struct {
  EhpAction action;
  EhpParam param;
} _STRUCT_ALIGNED_ EHPIn;

typedef struct {
  SimpleMainSwitch fcw_main_switch;           // fcw软开关 (true:on / false:off)
  SensitivityLevel fcw_set_sensitivity_level; // fcw敏感度等级
  SimpleMainSwitch aeb_main_switch;           // aeb软开关
  SimpleMainSwitch meb_main_switch;           // meb软开关
  NotificationMainSwitch tsr_main_switch;     // tsr软开关
  SimpleMainSwitch ihc_main_switch;           // ihc软开关
  SimpleMainSwitch ldw_main_switch;           // ldw软开关
  SensitivityLevel ldw_set_sensitivity_level; // ldw敏感度等级
  SimpleMainSwitch elk_main_switch;           // elk软开关
  SimpleMainSwitch bsd_main_switch;           // bsd软开关
  SimpleMainSwitch lca_main_switch;           // lca软开关
  SimpleMainSwitch dow_main_switch;           // dow软开关
  SimpleMainSwitch fcta_main_switch;          // fcta软开关
  SimpleMainSwitch fctb_main_switch;          // fctb软开关
  SimpleMainSwitch rcta_main_switch;          // rcta软开关
  SimpleMainSwitch rctb_main_switch;          // rctb软开关
  SimpleMainSwitch rcw_main_switch;           // rcw软开关
  SimpleMainSwitch ldp_main_switch;           // ldp软开关
  SimpleMainSwitch amap_main_switch;          // amap软开关
  SimpleMainSwitch dai_main_switch;           // dai软开关
  SimpleMainSwitch dow_secondary_alert_main_switch; // dow 二级警示语音软开关
} _STRUCT_ALIGNED_ ADASIn;

typedef enum {
  SPEED_UNCHANGED = 0,       // 速度不变
  SPEED_INCREASED_1KPH = 1,  // 速度+1kph
  SPEED_INCREASED_5KPH = 2,  // 速度+5kph
  SPEED_INCREASED_10KPH = 3, // 速度+10kph
  SPEED_DECREASED_1KPH = 4,  // 速度-1kph
  SPEED_DECREASED_5KPH = 5,  // 速度-5kph
  SPEED_DECREASED_10KPH = 6, // 速度-10kph
} _ENUM_PACKED_ AccSetSpdPm; 

typedef enum {
  TIME_GAP_UNCHANGED = 0,      // 时距不变
  TIME_GAP_INCREASE = 1,       // 时距+
  TIME_GAP_DECREASE = 2,       // 时距-
  TIME_GAP_SMARTFOLLOWING = 3, // 智慧跟车
} _ENUM_PACKED_ AccSetTimeGap; 

typedef enum {
  DRIVE_MODE_NONE = 0,        // 无输入
  DRIVE_MODE_NORMAL = 1,      // 正常模式                           
  DRIVE_MODE_COMFORTABLE = 2, // 舒适模式                          
  DRIVE_MODE_SNOW = 3,        // 雪地模式
  DRIVE_MODE_MUD = 4,         // 泥泞模式
  DRIVE_MODE_OFFROAD = 5,     // 越野模式
  DRIVE_MODE_SAND = 6,        // 沙地模式
  DRIVE_MODE_SPORT = 7,       // 运动模式                              
} _ENUM_PACKED_ DriveMode; 

typedef enum {
  HORIZONTAL_PREFERENCE_LINE_INVALID = 0,
  HORIZONTAL_PREFERENCE_LINE_MID = 1,     //居中                           
  HORIZONTAL_PREFERENCE_LINE_LEFT = 2,    //偏左                              
  HORIZONTAL_PREFERENCE_LINE_RIGHT = 3,   //偏右
} _ENUM_PACKED_ PreferenceLine; 

typedef enum {
  QUICK_SPEED_SET_NONE = 0,         // 未调速        
  QUICK_SPEED_SET_OVERRIDE = 1,     // OVERRIDE下快捷调速                            
  QUICK_SPEED_SET_NOT_OVERRIDE = 2, // 非OVERRIDE下快捷调速
} _ENUM_PACKED_ QuickSpeedSet; 

typedef struct {
  SelectSource select_source;      // 信号来源
  PreferenceLine preference_line;  // 车道偏好
  DriveMode drive_mode;            // 驾驶模式
} _STRUCT_ALIGNED_ PilotUserPreference; 

// 偏移模式
typedef enum {
  SPEED_OFFSET_MODE_VALUE = 0,      // 固定值偏移
  SPEED_OFFSET_MODE_PERCENTAGE = 1, // 百分比偏移
} _ENUM_PACKED_ SpeedOffsetMode; 

// 速度偏移设置
typedef enum {
  SPEED_OFFSET_SET_UNCHANGED = 0,      // 不变
  SPEED_OFFSET_SET_INCREASE = 1,       // +1
  SPEED_OFFSET_SET_DECREASE = 2,       // -1
} _ENUM_PACKED_ SpeedOffsetSet; 

// 速度偏移设置
typedef struct {
  SpeedOffsetMode offset_mode;
  SpeedOffsetSet speed_offset_set;
  int32 speed_offset_value;
} _STRUCT_ALIGNED_ SpeedOffset;

// 脱手检测模式设置
typedef enum {
  HANDS_OFF_DETECTION_NONE = 0,        // 无输入
  HANDS_OFF_DETECTION_STANDARD = 1,    // 标准
  HANDS_OFF_DETECTION_COMFORTABLE = 2, // 舒适
} _ENUM_PACKED_ HandsOffDetection;

// 变道风格
typedef enum {
  LANE_CHANGE_STYLE_NONE = 0,      // 无输入
  LANE_CHANGE_STYLE_ASSISTIVE = 1, // 温和
  LANE_CHANGE_STYLE_STANDARD = 2,  // 标准
  LANE_CHANGE_STYLE_AGILE = 3,     // 敏捷
} _ENUM_PACKED_ LaneChangeStyle;

// 巡航车速模式设置
typedef enum {
  ACC_SPD_MODE_SET_INACTIVE = 0,             //无效值
  ACC_SPD_MODE_SET_CURREENT_SPEED_MODE = 1,  //当前速度：自适应巡航或车道居中辅助激活时，巡航速度不随限速值自动调整。
  ACC_SPD_MODE_SET_LSLI_MODE = 2,            //道路限速：自适应巡航或车道居中辅助激活时，巡航速度跟随道路限速自动调整。
  ACC_SPD_MODE_SET_RESERVED = 3,             //保留值
} _ENUM_PACKED_ ACCCuriseSpdModeSet;

typedef struct {
  SimpleMainSwitch acc_main_switch;            // acc功能软开关
  boolean acc_active_switch;                   // acc激活信号
  boolean acc_cancel_switch;                   // acc关闭信号
  float32 acc_set_disp_speed;                  // acc跟车表显时速
  AccSetSpdPm acc_set_spd_pm;                  // acc跟车表显时速加减
  AccSetTimeGap acc_set_time_gap;              // acc跟车时距设定
  float32 acc_set_time_interval;               // acc跟车时距
  boolean acc_stop2go;                         // acc跟车起步确认
  SimpleMainSwitch scc_main_switch;            // scc功能软开关
  boolean scc_active_switch;                   // scc激活信号
  boolean scc_cancel_switch;                   // scc关闭信号
  SimpleMainSwitch noa_main_switch;            // noa激活软开关
  boolean noa_active_switch;                   // noa激活信号
  boolean noa_downgrade_scc_switch;            // noa降级scc信号
  boolean scc_upgrade_noa_switch;              // scc升级noa信号
  NoaVoicePromptSwitch noa_voice_promp_switch; // noa语音提示开关
  SimpleMainSwitch noa_cruise_dclc_switch;     // noa变道确认提醒开关
  SimpleMainSwitch mnp_main_switch;            // mnp功能软开关
  boolean noa_cancel_switch;                   // noa功能关闭 (true:close noa / false:do nothing)
  PilotUserPreference pilot_user_preference;   // 行车用户偏好
  QuickSpeedSet quick_speed_set;               // 快捷速度设定
  SpeedOffset speed_offset;                    // 速度偏移设定
  HandsOffDetection hands_off_detection;       // 脱手检测设置
  LaneChangeStyle lane_change_style;           // 变道风格
  SimpleMainSwitch traffic_light_stop_go;      // 直行红绿灯启停
  SimpleMainSwitch ObstacleBypass;             // 借道绕行
  TurnSwitchStateEnum voice_turn_switch;       // 语音变道状态
  ACCCuriseSpdModeSet acc_spd_mode_set;        // 巡航车速模式
} _STRUCT_ALIGNED_ PilotIn;

typedef struct {
  SimpleMainSwitch rads_main_switch; // RADS主开关
  boolean rads_cancel_switch;        // RADS取消开关
  boolean rads_active_switch;        // RADS激活按键
  boolean rads_start_switch;         // 开始倒车
} _STRUCT_ALIGNED_ RADSIn;

typedef enum {
  PA_DIRECTION_INVALID = 0,
  PA_DIRECTION_LEFT = 1,    // 向左贴边
  PA_DIRECTION_RIGHT = 2,   // 向右贴边
} _ENUM_PACKED_ PaDirection;

typedef struct {
  SimpleMainSwitch pa_main_switch;    // PA主开关
  boolean pa_cancel_switch;           // PA取消开关
  boolean pa_active_switch;           // PA激活按键
  boolean pa_start_switch;            // 开始贴边
  PaDirection pa_direction;           // 选择的贴边方向
} _STRUCT_ALIGNED_ PAIn;

typedef struct {
  SimpleMainSwitch nra_main_switch;   // NRA主开关
  boolean nra_cancel_switch;          // NRA取消开关
  boolean nra_active_switch;          // NRA激活按键
  boolean nra_start_switch;           // 开始
  boolean nra_resume_switch;          // 继续
} _STRUCT_ALIGNED_ NRAIn;

typedef struct {
  SimpleMainSwitch blue_light_main_switch;   // 小蓝灯主开关
} _STRUCT_ALIGNED_ CommonIn;

typedef enum {
  SR_SWITCH_INVALID = 0,
  SR_SWITCH_ACTIVE = 1,    // 进入SR
  SR_SWITCH_CANCEL = 2,    // 退出SR
} _ENUM_PACKED_ SRSwitch;

typedef enum {
  DATA_UPLOAD_NONE = 0,       // 无效值  
  DATA_UPLOAD_TRIGGER = 1,    // 触发
  DATA_UPLOAD_RESERVED = 2,   // 保存
} _ENUM_PACKED_ DataUpLoadBtn;

typedef enum {
  RECORD_NONE = 0,          // 无效值  
  RECORD_UPLOAD = 1,        // 上传
  RECORD_NOT_UPLOAD = 2,    // 不上传
  RECORD_RESERVED = 3,      // 保留
} _ENUM_PACKED_ RecordUpLoadBtn;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  boolean is_valid;                   //hmi消息是否有效
  APAIn apa_in;                       // 泊车信号输入         
  RPAIn rpa_in;                       // 遥控泊车信号
  HPPIn hpp_in;                       // 记忆泊车信号
  EHPIn ehp_in;                       // EHP地图管理输入
  ADASIn adas_in;                     // ADAS开关信号
  PilotIn pilot_in;                   // 行车信号输入
  boolean calib_active_switch;        // 标定激活按键
  boolean data_record_switch;         // 数据录制按键
  RADSIn rads_in;                     // 循迹倒车输入信号
  PAIn pa_in;                         // 一键贴边输入信号
  NRAIn nra_in;                       // 窄路通行输入信号
  CommonIn common_in;                 // 通用信号输入(不区分行泊车功能)
  DataUpLoadBtn data_upload_btn;      // 吐槽/一键数据回传
  RecordUpLoadBtn record_upload_btn;  // 吐槽/录音数据回传
  SRSwitch sr_switch;                 // SR应用进入/退出接口
} _STRUCT_ALIGNED_ HmiInner;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_HMI_INNER_H_
