/**
 * @file hmi_service_c.h
 * @brief
 * @author chenzhang20 (chenzhang20@iflytek.com)
 * @version v0.1
 * @date 2024-08-08
 *
 * @copyright Copyright (c) 2024  iflytek
 */

#ifndef _IFLYAUTO_HMI_SERVICE_H_
#define _IFLYAUTO_HMI_SERVICE_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"
#include "hmi_mcu_out_c.h"
#include "hmi_soc_outer_c.h"
#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

typedef enum {
  HMI_SERVICE_VCU_GEAR_NO_REQUEST = 0,  // 无挡位信息
  HMI_SERVICE_VCU_GEAR_VALUE_P = 1,     // 驻车
  HMI_SERVICE_VCU_GEAR_VALUE_R = 2,     // 倒车
  HMI_SERVICE_VCU_GEAR_VALUE_N = 3,     // 空挡
  HMI_SERVICE_VCU_GEAR_VALUE_D = 4,     // 前进挡
  HMI_SERVICE_VCU_GEAR_RESERVED1 = 5,   // 保留
  HMI_SERVICE_VCU_GEAR_RESERVED2 = 6,   // 保留
  HMI_SERVICE_VCU_GEAR_INVALID = 7,     // 无效
} _ENUM_PACKED_ HmiServiceVcuGear;

typedef struct {
  float32               pilot_disp_acc_set_speed; 
  int32                 pilot_disp_acc_set_headway;
  AccStatus             pilot_disp_acc_sts;
  AccNotify             pilot_disp_acc_notify;
  SccStatus             pilot_disp_scc_sts;
  SccNotify             pilot_disp_scc_notify;
  NoaStatus             pilot_disp_noa_sts;
  NoaNotify             pilot_disp_noa_notify;
  boolean               pilot_disp_adas_takeover_req;
  ADASTakeoverReason    pilot_disp_adas_takeover_reason;
  LC_REASON             pilot_disp_lc_reason;
  SccHandsOffWarn       pilot_disp_scc_handoff_warning;
  uint32                pilot_disp_dist_to_dest;
  uint32                pilot_disp_dist_to_station;
} _STRUCT_ALIGNED_ PilotHmiDisplay;   //行车HMI显示  

typedef struct {
  ApaStatus      park_disp_apa_status;
  ApaNotifyReq   park_disp_apa_notify_req;
  ApaWorkMode    park_disp_apa_mode_status;
  RadsStatus     park_disp_rads_status;
  RadsNotifyReq  park_disp_rads_notify_req;
  NraStatus      park_disp_nra_status;
  NraNotify      park_disp_nra_notify;
  PaStatus       park_disp_pa_status;
  PaNotify       park_disp_pa_notify; 
  HppStatus      park_disp_hpp_status;
  HppNotifyReq   park_disp_hpp_notify_req;
  ApaParkingDirection select_park_in_dir;         //用户选择泊入方向
  float32        apa_notify_remain_distance;      //距离泊入成功还有多少米
  ApaParkOutDirection select_park_out_dir;        //用户选择泊出方向
  uint16         planning_park_dir;               //规划泊车方向
  ApaWorkMode    apa_mode_status;                 //泊车模式
} _STRUCT_ALIGNED_ ParkHmiDisplay;  //泊车HMI显示

typedef struct {
  boolean noa_switch_response_display;
  boolean mnp_switch_response_display;
  boolean apa_switch_response_display;
  boolean rpa_switch_response_display;
  boolean hpp_switch_response_display;
  boolean ldw_switch_response_display;
  boolean ldp_switch_response_display;
  SensitivityLevel ldw_level_response_display;
  boolean aeb_switch_response_display;
  boolean fcw_switch_response_display;
  SensitivityLevel fcw_level_response_display;
  NotificationMainSwitch tsr_switch_response_display;
  boolean elk_switch_response_display;
  boolean meb_switch_response_display;
  boolean amap_switch_response_display;
  boolean fcta_switch_response_display;
  boolean fctb_switch_response_display;
  boolean ihc_switch_response_display;
  boolean dai_switch_response_display;
  boolean lcc_switch_response_display;
  boolean bsd_switch_response_display;
  boolean lca_switch_response_display;
  boolean dow_switch_response_display;
  boolean rcta_switch_response_display;
  boolean rctb_switch_response_display;
  boolean rcw_switch_response_display;
  boolean dow_secondary_alert_switch_response_display;
  boolean blue_light_switch_response_display;
  boolean function_degrade_switch_response_display;
}_STRUCT_ALIGNED_ SwitchHmiDisplay;     //开关状态反馈

typedef struct {
  boolean tsr_warning_display;
  uint32 tsr_speed_limit_display;
  boolean tsr_speed_unlimit_warning_display;
  TSRFunctionFSMWorkState tsr_state_display;
  boolean ldw_left_warning_display;
  boolean ldw_right_warning_display;
  boolean ldp_left_intervention_flag_display;
  boolean ldp_right_intervention_flag_display;
  LDWFunctionFSMWorkState ldw_state_display;
  LDPFunctionFSMWorkState ldp_state_display;
  boolean elk_left_intervention_flag_display;
  boolean elk_right_intervention_flag_display;
  ELKFunctionFSMWorkState elk_state_display;
  MEBFunctionFSMWorkState meb_state_display;
  MEBInterventionDirection meb_request_direction_display;
}_STRUCT_ALIGNED_ AdasHmiDisplay;   //adas信息

typedef struct {
  SpeedOffsetMode offset_mode_display;
  int32 speed_offset_value_display;
  int32 speed_offset_percentage_display;
  ACCCruiseSpdModeInd acc_spd_mode_ind_display;
}_STRUCT_ALIGNED_ AccHmiDisplay;    //acc信息

typedef struct {
  HandsOffDetection hands_off_detection_display;
  boolean traffic_light_stop_go_display;
  boolean obstacle_bypass_display;
}_STRUCT_ALIGNED_ SccHmiDisPlay;    //scc信息

typedef struct {
  boolean noa_cruise_dclc_resp_display;
  LaneChangeStyle lane_change_style_display;
}_STRUCT_ALIGNED_ NoaHmiDisplay;    //noa信息

typedef struct {
  PilotHmiDisplay   pilot_hmi_display;      //行车HMI显示    
  ParkHmiDisplay    park_hmi_display;       //泊车HMI显示
  SwitchHmiDisplay  hmi_switch_display;     //开关状态反馈 
  AdasHmiDisplay    adas_hmi_display;       //adas信息
  AccHmiDisplay     acc_hmi_display;        //acc信息
  SccHmiDisPlay     scc_hmi_display;        //scc信息
  NoaHmiDisplay     noa_hmi_display;        //noa信息
} _STRUCT_ALIGNED_ HmiDisplayInfo;

typedef struct {
  RADSIn            rads_in_display;        //RADS信息
  PAIn              pa_in_display;          //一键贴边输入信息
  NRAIn             nra_in_display;         //窄路通行输入信息
  APAIn             apa_in_display;         //泊车信息输入
  ADASIn            adas_in_display;        //adas开关信息
  PilotIn           pilot_in_display;       //行车信号输入
  RPAIn             rpa_in_display;         //遥控泊车
  HPPIn             hpp_in_display;         //记忆泊车
  CommonIn          common_in;              // 通用信号输入(不区分行泊车功能)
} _STRUCT_ALIGNED_ HmiSetInfo;

typedef struct {
  MsgHeader msg_header;
  // MsgMeta msg_meta;
  boolean active_btn_press;               // 进入自动拨杆触发，SOC做单击激活（ACC）和双击激活（SCC）判断
  boolean cancel_btn_press;               // 退出自动拨杆触发
  HmiServiceVcuGear vcu_actual_gear;      // 当前挡位信号
  HmiServiceVcuGear vcu_target_gear;      // 目标挡位信号
  int8 cruise_speed_value;                // 巡航速度滚轮，0:无操作, -1:减速, 1:加速
  int8 time_gap_value;                    // 时距滚轮，0:无操作, -1:减小时距, 1:增加时距
  boolean break_btn_press;                // 刹车踩下触发
  boolean record_btn_press;               // 录制按键触发
  ADASFunctionOutputInfo adas_out_info;   // ADAS输出信息
  HmiPdcInfo hmi_pdc_info;                // PDC报警信息
  RPAIn rpa_info;                         // rpa info
  uint8 hmi_car_mode;                     // 车辆模式
  uint8 auto_driving_btn_active;          // 按键激活功能信号(0:no active / 1: lcc active /2: acc active)
  uint8 auto_driving_btn_cancel;          // 按键退出功能信号(0:no active / 1: cancel)
  uint8 cruise_speed_sync;                // 同步巡航车速信号(0:no active / 1:向上同步车速 /2：向下同步车速)
  uint8 btn_reboot_adcc_req;              // 上报"再按一次时距+，重启域控"文言(0:no active / 1：请求显示文言"再按一次时距+，重启域控")
  uint8 reboot_adcc_standby;              // 上报"即将开始重启域控"文言(0:no active / 1：请求显示文言"即将开始重启域控"  /2：请求显示文言"辅助驾驶激活状态，不能重启")
  uint8 cruise_speed_cont;                // 巡航车速长按按键(0:no active / 1:长按车速+ /2：长按车速-)
  HmiSetInfo hmi_set_info;                // hmi设置项
  uint8 reserved[64];                     // 保留
} _STRUCT_ALIGNED_ HmiMcuToSocBsw_CHERY_E0Y_MDC510;

typedef enum {
  HMI_SERVICE_RPA_STATE_OFF = 0, // 
  HMI_SERVICE_RPA_STATE_STANDBY , // 
  HMI_SERVICE_RPA_STATE_SEARCHING , // 
  HMI_SERVICE_RPA_STATE_PARKING_ACTIVE , // 
  HMI_SERVICE_RPA_STATE_COMPLETED , // 
  HMI_SERVICE_RPA_STATE_FAILURE , // 
  HMI_SERVICE_RPA_STATE_TERMINATE , // 
  HMI_SERVICE_RPA_STATE_PAUSE , // 
  HMI_SERVICE_RPA_STATE_UNDO , // 
} _ENUM_PACKED_ RPAStatus_CHERY_E0Y_MDC510;

typedef enum {
    HMI_SERVICE_RPA_REQ_PATH_SAVING              = 0x2E,  // 46
    HMI_SERVICE_RPA_REQ_SPEED_OVER_LIMIT_APA_CANT_ACTIVE = 0x2D,  // 45
    HMI_SERVICE_RPA_REQ_PARKING_RESUME           = 0x2C,  // 44
    HMI_SERVICE_RPA_REQ_CAUTION_REAR_SPACE       = 0x2B,  // 43
    HMI_SERVICE_RPA_REQ_TARGET_SLOT_OCCUPIED     = 0x2A,  // 42
    HMI_SERVICE_RPA_REQ_TRAINING_PATH_IS_MAPPING = 0x29,  // 41
    HMI_SERVICE_RPA_REQ_PATH_LENGTH_READY_EXCEED_LIMIT = 0x28,  // 40
    HMI_SERVICE_RPA_REQ_SYSTEM_AUTO_SAVE_PATH    = 0x27,  // 39
    HMI_SERVICE_RPA_REQ_STOP_PARKING_SLOT_RELEASED = 0x26,  // 38
    HMI_SERVICE_RPA_REQ_CONFIRM_SAVE_TRAINING_PATH = 0x25,  // 37
    HMI_SERVICE_RPA_REQ_PRESS_BRAKE_AND_RECOVERY = 0x24,  // 36
    HMI_SERVICE_RPA_REQ_PS_REPEAT_INPUT          = 0x23,  // 35
    HMI_SERVICE_RPA_REQ_UNDO_SELECT              = 0x22,  // 34
    HMI_SERVICE_RPA_REQ_STEERING_WHEEL_HANDOFF   = 0x21,  // 33
    HMI_SERVICE_RPA_REQ_PS_INPUT                 = 0x20,  // 32
    HMI_SERVICE_RPA_REQ_CONFIRM_PRESS_UNDO_SWITCH= 0x1F,  // 31
    HMI_SERVICE_RPA_REQ_FOWARD_MOVE_A_BIT        = 0x1E,  // 30
    HMI_SERVICE_RPA_REQ_OPEN_OUTSIDE_REAR_VIEW_MIRROR = 0x1D,  // 29
    HMI_SERVICE_RPA_REQ_MRA_START_CONFIRM        = 0x1C,  // 28
    HMI_SERVICE_RPA_RESERVED_27                  = 0x1B,
    HMI_SERVICE_RPA_RESERVED_26                  = 0x1A,
    HMI_SERVICE_RPA_RESERVED_25                  = 0x19,
    HMI_SERVICE_RPA_RESERVED_24                  = 0x18,
    HMI_SERVICE_RPA_REQ_REPRESS_PARKING_SWITCH   = 0x17,  // 23
    HMI_SERVICE_RPA_REQ_POC_DIRECTION_SELECT     = 0x16,  // 22
    HMI_SERVICE_RPA_REQ_FUNCTION_OFF             = 0x15,  // 21
    HMI_SERVICE_RPA_REQ_PROCESS_BAR              = 0x14,  // 20
    HMI_SERVICE_RPA_REQ_RELEASE_BRAKE            = 0x13,  // 19
    HMI_SERVICE_RPA_REQ_CONFIRM_PRESS_DM_SWITCH  = 0x12,  // 18
    HMI_SERVICE_RPA_REQ_PRESS_BRAKE_PEDAL        = 0x11,  // 17
    HMI_SERVICE_RPA_REQ_SURROUND_VIEW            = 0x10,  // 16
    HMI_SERVICE_RPA_REQ_BUCKLE_SEAT_BELT         = 0x0F,  // 15
    HMI_SERVICE_RPA_REQ_CLOSE_DOOR_ALL           = 0x0E,  // 14
    HMI_SERVICE_RPA_REQ_CLOSE_ENGINE_FRONT_DOOR  = 0x0D,  // 13
    HMI_SERVICE_RPA_REQ_CLOSE_TRUNK              = 0x0C,  // 12
    HMI_SERVICE_RPA_REQ_LEAVE_CAR                = 0x0B,  // 11
    HMI_SERVICE_RPA_REQ_EPB_APPLIED              = 0x0A,  // 10
    HMI_SERVICE_RPA_REQ_CONNECT_PHONE            = 0x09,  // 9
    HMI_SERVICE_RPA_REQ_FUNCTION_SELECT_ARPA     = 0x08,  // 8
    HMI_SERVICE_RPA_REQ_TURN_LEVER               = 0x07,  // 7
    HMI_SERVICE_RPA_REQ_PS_ID_SELECTION          = 0x06,  // 6
    HMI_SERVICE_RPA_REQ_STOP                     = 0x05,  // 5
    HMI_SERVICE_RPA_REQ_SEARCHING_PROCESS        = 0x04,  // 4
    HMI_SERVICE_RPA_REQ_SLOW_DOWN                = 0x03,  // 3
    HMI_SERVICE_RPA_REQ_GEAR_D                   = 0x02,  // 2
    HMI_SERVICE_RPA_REQ_FUNCTION_SELECT          = 0x01,  // 1
    HMI_SERVICE_RPA_REQ_DriverOperateInd_NO_REQUEST                   = 0x00   // 0
} _ENUM_PACKED_ RPADriverOperateInd_CHERY_E0Y_MDC510;

typedef enum {
    HMI_SERVICE_RPA_REQ_RECOVER_CAMERA_BLOCKED            = 0x0B,  // 11
    HMI_SERVICE_RPA_REQ_RECOVER_SEAT_BELT_UNBUCKLED       = 0x0A,  // 10
    HMI_SERVICE_RPA_REQ_RECOVER_TRUNK_OPEN                = 0x09,  // 9
    HMI_SERVICE_RPA_REQ_RECOVER_EXTERNAL_MIRROR_OPEN      = 0x08,  // 8
    HMI_SERVICE_RPA_REQ_RECOVER_VEHICLE_STANDSTILL_OVERTIME = 0x07,// 7
    HMI_SERVICE_RPA_REQ_RECOVER_PEDESTRIAN_IN_TRAJECTORY  = 0x06,  // 6 (修正拼写)
    HMI_SERVICE_RPA_REQ_RECOVER_KEY_OUT_OF_RANGE          = 0x05,  // 5
    HMI_SERVICE_RPA_REQ_RECOVER_PHONE_OUT_OF_RANGE        = 0x04,  // 4
    HMI_SERVICE_RPA_REQ_RECOVER_FOUND_OBSTACLE_IN_TRAJECTORY = 0x03, //3
    HMI_SERVICE_RPA_REQ_RECOVER_PASSENGER_DOOR_OPEN       = 0x02,  // 2
    HMI_SERVICE_RPA_REQ_RECOVER_PRESS_RESUME_SWITCH       = 0x01,  // 1 (修正空格)
    HMI_SERVICE_RPA_REQ_ParkingPauseInd_NO_REQUEST        = 0x00   // 0
} _ENUM_PACKED_ RPAParkingPauseInd_CHERY_E0Y_MDC510;

typedef enum {
    HMI_SERVICE_RPA_REQ_QUIT_RESPONSE_TIMEOUT                  = 0x31,  // 49
    HMI_SERVICE_RPA_REQ_QUIT_NO_AVAILABLE_SLOT                 = 0x30,  // 48
    HMI_SERVICE_RPA_REQ_QUIT_PASSENGER_DOOR                    = 0x2F,  // 47
    HMI_SERVICE_RPA_REQ_QUIT_FREE_SPACE_LIMIT                  = 0x2E,  // 46
    HMI_SERVICE_RPA_REQ_QUIT_LOC_UNAVAILABLE                   = 0x2D,  // 45
    HMI_SERVICE_RPA_REQ_QUIT_NOT_ALLOWED_AREA                  = 0x2C,  // 44
    HMI_SERVICE_RPA_REQ_QUIT_PITCH_OVER_LIMIT                  = 0x2B,  // 43
    HMI_SERVICE_RPA_REQ_QUIT_SLOPE_OVER_LIMIT                  = 0x2A,  // 42
    HMI_SERVICE_RPA_REQ_QUIT_PATH_LONG_OVER                    = 0x29,  // 41
    HMI_SERVICE_RPA_REQ_QUIT_FOLDED_MIRROR                     = 0x28,  // 40
    HMI_SERVICE_RPA_REQ_QUIT_CHARGE_PLUG_CONNECTED             = 0x27,  // 39
    HMI_SERVICE_RPA_REQ_QUIT_LIMITATION_PER_FUSION_MAP         = 0x26,  // 38
    HMI_SERVICE_RPA_REQ_QUIT_LIMITED_SENSORS                   = 0x25,  // 37
    HMI_SERVICE_RPA_REQ_QUIT_STILL_MOVING_IN_RECOVERY_STAGES   = 0x24,  // 36
    HMI_SERVICE_RPA_REQ_QUIT_PATH_UPLOAD_TIMEOUT               = 0x23,  // 35
    HMI_SERVICE_RPA_REQ_QUIT_TRAINING_PROCESS_OVERFLOW         = 0x22,  // 34
    HMI_SERVICE_RPA_REQ_QUIT_CORRESPONDING_REMIND_TIMEOUT      = 0x21,  // 33
    HMI_SERVICE_RPA_REQ_QUIT_BACKWARD_MOVE_LONG                = 0x20,  // 32
    HMI_SERVICE_RPA_REQ_QUIT_INTERNAL_COMPONENTS_ABNORMAL      = 0x1F,  // 31
    HMI_SERVICE_RPA_REQ_QUIT_FUNCTION_SAFETY                   = 0x1E,  // 30
    HMI_SERVICE_RPA_REQ_QUIT_ENGINE_FRONT_DOOR_OPEN            = 0x1D,  // 29
    HMI_SERVICE_RPA_REQ_QUIT_REMOTE_DEVICE                     = 0x1C,  // 28
    HMI_SERVICE_RPA_REQ_QUIT_DOOR_LOCK                         = 0x1B,  // 27
    HMI_SERVICE_RPA_REQ_QUIT_UNSAFE_BEHAVIOR                   = 0x1A,  // 26
    HMI_SERVICE_RPA_REQ_QUIT_NO_POC_SLOT_AVAILABLE             = 0x19,  // 25
    HMI_SERVICE_RPA_REQ_QUIT_PHONE_LOW_BATTERY                 = 0x18,  // 24
    HMI_SERVICE_RPA_REQ_QUIT_PHONE_DISCONNECTED                = 0x17,  // 23
    HMI_SERVICE_RPA_REQ_QUIT_OTHER_REASON                      = 0x16,  // 22
    HMI_SERVICE_RPA_REQ_QUIT_PATH_DEVIATION                    = 0x15,  // 21
    HMI_SERVICE_RPA_REQ_QUIT_STEERING_WHEEL_INTERVENTION       = 0x14,  // 20
    HMI_SERVICE_RPA_REQ_QUIT_EXTERNAL_ECU_ACTIVE               = 0x13,  // 19
    HMI_SERVICE_RPA_REQ_QUIT_VEHICLE_BLOCK                     = 0x12,  // 18
    HMI_SERVICE_RPA_REQ_QUIT_EPB_APPLY                         = 0x11,  // 17
    HMI_SERVICE_RPA_REQ_QUIT_GEAR_INTERRUPT                    = 0x10,  // 16
    HMI_SERVICE_RPA_REQ_QUIT_GEAR_INTERVENTION                 = 0x0F,  // 15
    HMI_SERVICE_RPA_REQ_QUIT_TERMINATE_BUTTON_PRESSED          = 0x0E,  // 14
    HMI_SERVICE_RPA_REQ_QUIT_SURROUND_VIEW                     = 0x0D,  // 13
    HMI_SERVICE_RPA_REQ_QUIT_LUGGAGE_DOOR_OPEN                 = 0x0C,  // 12
    HMI_SERVICE_RPA_REQ_QUIT_DRIVER_DOOR_ALL                   = 0x0B,  // 11
    HMI_SERVICE_RPA_REQ_QUIT_SEAT_BELT_UNBUCKLE                = 0x0A,  // 10
    HMI_SERVICE_RPA_REQ_QUIT_TJA                               = 0x09,  // 9
    HMI_SERVICE_RPA_REQ_QUIT_ACC                               = 0x08,  // 8
    HMI_SERVICE_RPA_REQ_QUIT_GAS_PEDAL                         = 0x07,  // 7
    HMI_SERVICE_RPA_REQ_QUIT_SPEED_HIGH                        = 0x06,  // 6
    HMI_SERVICE_RPA_REQ_QUIT_SPACE_LIMIT                       = 0x05,  // 5
    HMI_SERVICE_RPA_REQ_QUIT_TRAJECTORY                        = 0x04,  // 4
    HMI_SERVICE_RPA_REQ_QUIT_RECOVER_INTERRUPT_TIMES_OVERFLOW  = 0x03,  // 3
    HMI_SERVICE_RPA_REQ_QUIT_MOVE_TIMES_OVERFLOW               = 0x02,  // 2
    HMI_SERVICE_RPA_REQ_QUIT_TIMING_OVERFLOW                   = 0x01,  // 1
    HMI_SERVICE_RPA_REQ_QuitRequestCode_NO_REQUEST             = 0x00   // 0
} _ENUM_PACKED_ RPAQuitRequestCode_CHERY_E0Y_MDC510;

typedef enum {
    HMI_SERVICE_RPA_RESERVED_7 = 0x07,  // 7
    HMI_SERVICE_RPA_RESERVED_6 = 0x06,  // 6
    HMI_SERVICE_RPA_RESERVED_5 = 0x05,  // 5
    HMI_SERVICE_RPA_RESERVED_4 = 0x04,  // 4
    HMI_SERVICE_RPA_RESERVED_3 = 0x03,  // 3
    HMI_SERVICE_RPA_OUT_ON = 0x02,  // 2
    HMI_SERVICE_RPA_INT_ON = 0x01,  // 1
    HMI_SERVICE_RPA_ParkingActiveSubFuncSts_NO_REQUEST = 0x00   // 0
} _ENUM_PACKED_ RPAParkingActiveSubFuncSts_CHERY_E0Y_MDC510;

typedef enum {
    HMI_SERVICE_RPA_PARK_OUT_DIRECTION_RESERVED_15 = 0x0F,  // 15
    HMI_SERVICE_RPA_PARK_OUT_DIRECTION_RESERVED_14 = 0x0E,  // 14
    HMI_SERVICE_RPA_PARK_OUT_DIRECTION_RESERVED_13 = 0x0D,  // 13
    HMI_SERVICE_RPA_PARK_OUT_DIRECTION_RESERVED_12 = 0x0C,  // 12
    HMI_SERVICE_RPA_PARK_OUT_DIRECTION_RESERVED_11 = 0x0B,  // 11
    HMI_SERVICE_RPA_PARK_OUT_DIRECTION_RESERVED_10 = 0x0A,  // 10
    HMI_SERVICE_RPA_PARK_OUT_DIRECTION_RESERVED_9  = 0x09,  // 9
    HMI_SERVICE_RPA_PARK_OUT_DIRECTION_BACK_RIGHT_CROSS = 0x08,  // 8
    HMI_SERVICE_RPA_PARK_OUT_DIRECTION_BACK_LEFT_CROSS  = 0x07,  // 7
    HMI_SERVICE_RPA_PARK_OUT_DIRECTION_BACK_OUT         = 0x06,  // 6
    HMI_SERVICE_RPA_PARK_OUT_DIRECTION_FRONT_RIGHT_PARALLEL = 0x05,  // 5
    HMI_SERVICE_RPA_PARK_OUT_DIRECTION_FRONT_RIGHT_CROSS = 0x04,  // 4
    HMI_SERVICE_RPA_PARK_OUT_DIRECTION_FRONT_OUT        = 0x03,  // 3
    HMI_SERVICE_RPA_PARK_OUT_DIRECTION_FRONT_LEFT_PARALLEL = 0x02,  // 2
    HMI_SERVICE_RPA_PARK_OUT_DIRECTION_FRONT_LEFT_CROSS = 0x01,  // 1
    HMI_SERVICE_RPA_PARK_OUT_DIRECTION_NONE             = 0x00   // 0
} _ENUM_PACKED_ RPAParkOutDirectionSt_CHERY_E0Y_MDC510;

typedef struct {
  RPAStatus_CHERY_E0Y_MDC510           rpa_status;               // RPA 功能状态
  RPAParkingPauseInd_CHERY_E0Y_MDC510  rpa_parking_pause_ind;    // 泊车暂停的原因
  RPADriverOperateInd_CHERY_E0Y_MDC510 rpa_driver_operate_ind;   // 驾驶员操作提示
  RPAQuitRequestCode_CHERY_E0Y_MDC510  rpa_quit_request_code;    // 泊车退出的原因
  uint32  challenge1 ;                                             // MCU 底软实现，无需关注
  uint32  challenge2 ;                                             // MCU 底软实现，无需关注
  RPAParkingActiveSubFuncSts_CHERY_E0Y_MDC510  rpa_parking_active_sub_func_sts;  // RPA 功能选择状态，直入直出
  boolean       ads_rpa_int_fun_sts;                                             // 遥控泊车功能可用状态 0: 不可用， 1：可用
  boolean       back_left_out_sts;     // 车尾左向泊出可用状态
  boolean       back_right_out_sts;    // 车尾右向泊出可用状态
  boolean       back_stright_out_sts;  // 车尾直出泊出可用状态
  boolean       front_left_out_sts ;   // 车头向左泊出可用状态
  boolean       front_right_out_sts;   // 车头右向泊出可用状态
  boolean       front_stright_out_sts; // 车头直出泊出可用状态
  boolean       front_left_parallel_out_sts ;   // 车头向左水平泊出可用状态
  boolean       front_right_parallel_out_sts;   // 车头向右水平泊出可用状态
  RPAParkOutDirectionSt_CHERY_E0Y_MDC510 rpa_park_out_direction_st; // 泊出方向状态（实际泊出方向）
  float32       parking_stop_dist;      // 泊车剩余距离单位 m 
  uint32        parking_complete_time ; // 泊车用时
  boolean       rpa_out_fun_sts ;       // 遥控泊出功能可用状态
  boolean       rpa_start_but_sts ;     // rpa开启开关可用状态
  boolean       prk_in_sts;             // 遥控泊入方向显示  0：back,  1: front
} _STRUCT_ALIGNED_ RPASoc2Mcu_CHERY_E0Y_MDC510;

typedef enum {
  TURN_LIGHT_REQ_NO_ACTIVE = 0,         // 无动作
  TURN_LIGHT_REQ_OFF = 1,               // 关灯
  TURN_LIGHT_REQ_LEFT = 2,              // 左转灯
  TURN_LIGHT_REQ_RIGHT = 3,             // 右转灯
  TURN_LIGHT_REQ_HAZARD = 4,            // 双闪
  TURN_LIGHT_REQ_HAZARD_THREE = 5       // 双闪闪三下
} _ENUM_PACKED_ TurnLightReq;

typedef enum {
  BEAM_LIGHT_REQ_NO_ACTIVE = 0,         // 无动作
  BEAM_LIGHT_REQ_LOW_BEAM = 1,          // 开近光，关远光
  BEAM_LIGHT_REQ_HIGHT_BEAM = 2,        // 开远光，关近光
  BEAM_LIGHT_REQ_OFF = 3                // 故障，关闭远近光
} _ENUM_PACKED_ BeamLightReq;

typedef struct {
  MsgHeader msg_header;
  // MsgMeta msg_meta;
  // RPASoc2Mcu_CHERY_E0Y_MDC510 rpa_soc_to_mcu;  // 添加 rpa 相关信息 根据dbc
  TurnLightReq turn_light_req;           // (0:no active /1:关转向灯 /2:开左转灯 /3:开右转灯 /4: 开双闪灯 /5:双闪闪三下)
  BeamLightReq beam_light_req;           // (0:no active /1:开近光关远光 /2:开远光关近光 /3:故障态，关闭远近光)
  uint8 reboot_adcc_active;              // (0:no active /1:完成显示)
  uint8 icc_reboot_adcc_req;             // (0:no active /1:请求重启ADCC)
  uint8 turn_light_state_machine;        // 状态机转向灯状态
  HmiDisplayInfo hmi_display_info;       // hmi显示信息
  boolean blue_light_req;                // 小蓝灯开关请求(true：开启/false：关闭)
  HmiParkingVoice hmi_parking_voice;     // 泊车提示音请求
  HmiRpaInfo hmi_rap_info;               // 遥控泊车信号
  DoorLockReq door_lock_req;             // 车门锁请求
  uint8 reserved[64];                    // 保留 位置-> 0 ： apa_status  3 : IHC_state 4 : IHC_request 5 : IHC_request_status
} _STRUCT_ALIGNED_ HmiSocToMcuBsw_CHERY_E0Y_MDC510;

/**
 * @brief T26 HMI Soc2Mcu BSW
 */
typedef struct {
  MsgHeader msg_header;
  // MsgMeta msg_meta; //MCU BSW不需要
  HmiCommon hmi_common;  // HMI 通用信息显示
  // uint8 hmi_line_info_size;
  // HmiLineInfo hmi_line_info[HMI_LINE_NUM];  // 车道线信息
  // uint8 hmi_obj_info_size;
  // HmiObjInfo hmi_obj_info[HMI_OBJECT_NUM];  // 障碍物信息
  // HmiObjInfo hmi_cipv_info;                 // CIPV信息
  // HmiApaInfo hmi_apa_info;                  // 泊车信息
  HmiAccInfo hmi_acc_info;                     // acc信息
  HmiSccInfo hmi_scc_info;                     // scc信息
  HmiNoaInfo hmi_noa_info;                     // noa信息
  HmiAdasInfo hmi_adas_info;                   // adas信息
  HmiCalibInfo calib_info;                     // 标定信息
  HmiSensorInfo sensor_info;                   // 传感器故障/遮挡信息
  uint8 reserved[64];  // 保留
} _STRUCT_ALIGNED_ HmiSocToMcuBsw_CHERY_T26;

/**
 * @brief T26 HMI Mcu2Soc BSW
 */
typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  boolean active_btn_press;              // 进入自动拨杆触发，SOC做单击激活（ACC）和双击激活（SCC）判断
  boolean cancel_btn_press;              // 退出自动拨杆触发
  HmiServiceVcuGear vcu_actual_gear;     // 当前挡位信号
  HmiServiceVcuGear vcu_target_gear;     // 目标挡位信号
  int8 cruise_speed_value;               // 巡航速度滚轮，0:无操作, -1:减速, 1:加速
  int8 time_gap_value;                   // 时距滚轮，0:无操作, -1:减小时距, 1:增加时距
  boolean break_btn_press;               // 刹车踩下触发
  boolean record_btn_press;              // 录制按键触发
  ADASFunctionOutputInfo adas_out_info;  // ADAS输出信息
  HmiPdcInfo hmi_pdc_info;               // PDC报警信息
  ADASIn adas_in;                        // ADAS开关信号
  PilotIn pilot_in;                      // 行车信号输入
  boolean calib_active_switch;           // 标定激活按键 (true:response / false:no response)
  uint8 reserved[64];                    // 保留
} _STRUCT_ALIGNED_ HmiMcuToSocBsw_CHERY_T26;


#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_HMI_SERVICE_H_
