/**
 * \file        adas_function.h
 * \brief       Definition of the current version of the adas_function
 *struct. \update      2023/03/23
 **/

#ifndef _IFLYAUTO_ADAS_FUNCTION_H_
#define _IFLYAUTO_ADAS_FUNCTION_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

/*******************************************************************************
 * adas结构体
 * topic：---
 * 频率：---
 *******************************************************************************/

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
} _STRUCT_ALIGNED_ RiskObjInfo;

typedef struct {
  /*  FCW功能状态
   *  0:Unavailable
   *  1:Off
   *  2:Standby
   *  3:Active
   *  <TODO:改成枚举>
   **/
  uint8 fcw_state;

  /*  FCW功能报警等级
   *  0:No Warning
   *  1:Warning Level1
   *  2:Warning Level2
   *  <TODO:改成枚举>
   **/
  uint8 fcw_warning_level;
  boolean fcw_Prefill_request_status;  // Prefill请求状态  (true:Request/false:No Request)
  boolean fcw_jerk_request_status;     // Jerk请求状态     (true:Request/false:No Request)
  RiskObjInfo fcw_risk_obj;
  boolean fcw_headway_warning_request;  // 安全距离请求状态(true:Request/false:No Request)
} _STRUCT_ALIGNED_ FCWOutputInfo;

typedef struct {
  /** PCW功能状态
   *  0:Unavailable
   *  1:Off
   *  2:Standby
   *  3:Active
   *  <TODO:改成枚举>
   **/
  uint8 pcw_state;

  /*  PCW功能报警等级
   *  0:No Warning
   *  1:Warning Level1
   *  2:Warning Level2
   *  <TODO:改成枚举>
   **/
  uint8 pcw_warning_level;
  boolean pcw_prefill_request_status;  // Prefill请求状态  (true:Request/false:No Request)
  boolean pcw_jerk_request_status;     // Jerk请求状态     (true:Request/false:No Request)
  RiskObjInfo pcw_risk_obj;
} _STRUCT_ALIGNED_ PCWOutputInfo;

typedef struct {
  /** AEB功能状态
   *  0:Unavailable
   *  1:Off
   *  2:Standby
   *  3:Active
   *  <TODO:改成枚举>
   **/
  uint8 aeb_state;
  boolean aeb_deceleration_request_status;  // AEB减速请求状态     (true:Request/false:No Request)

  /** AEB减速请求值
   *  单位：米/秒^2   (m/s^2)
   *  备注：AEB激活时，请求值为负值
   **/
  float32 aeb_deceleration_request_value;
  boolean aeb_hold_request_status;  // AEB停车保压请求状态 (true:Request/false:No Request)
  RiskObjInfo aeb_risk_obj;
} _STRUCT_ALIGNED_ AEBOutputInfo;

typedef struct {
  /** AEBP功能状态
   *  0:Unavailable
   *  1:Off
   *  2:Standby
   *  3:Active
   *  <TODO:改成枚举>
   **/
  uint8 aebp_state;
  boolean aebp_deceleration_requestStatus;  // AEBP减速请求状态        (true:Request/false:No Request)

  /** AEBP减速请求值
   *  单位：米/秒^2   (m/s^2)
   *  备注：AEB激活时，请求值为负值
   **/
  float32 aebp_deceleration_requestValue;
  boolean aebp_hold_request_status;  // AEBP停车保压请求状态    (true:Request/false:No Request)
  RiskObjInfo aebp_risk_obj;
} _STRUCT_ALIGNED_ AEBPOutputInfo;

typedef struct {
  /** BSD功能状态
   *  0:Unavailable
   *  1:Off
   *  2:Standby
   *  3:Active
   *  <TODO:改成枚举>
   **/
  uint8 bsd_state;

  /*  BSD左侧报警状态
   *  0:No Warning
   *  1:Warning Level1
   *  2:Warning Level2
   *  <TODO:改成枚举>
   **/
  uint8 bsd_left_warning_level;

  /*  BSD右侧报警状态
   *  0:No Warning
   *  1:Warning Level1
   *  2:Warning Level2
   *  <TODO:改成枚举>
   **/
  uint8 bsd_right_warning_level;
} _STRUCT_ALIGNED_ BSDOutputInfo;

typedef struct {
  /** LCA功能状态
   *  0:Unavailable
   *  1:Off
   *  2:Standby
   *  3:Active
   *  <TODO:改成枚举>
   **/
  uint8 lca_state;

  /*  LCA左侧报警状态
   *  0:No Warning
   *  1:Warning Level1
   *  2:Warning Level2
   *  <TODO:改成枚举>
   **/
  uint8 lca_left_warning_level;

  /*  LCA右侧报警状态
   *  0:No Warning
   *  1:Warning Level1
   *  2:Warning Level2
   *  <TODO:改成枚举>
   **/
  uint8 lca_right_warning_level;
} _STRUCT_ALIGNED_ LCAOutputInfo;

typedef struct {
  /** DOW功能状态
   *  0:Unavailable
   *  1:Off
   *  2:Standby
   *  3:Active
   *  <TODO:改成枚举>
   **/
  uint8 dow_state;

  /*  DOW前左侧报警状态
   *  0:No Warning
   *  1:Warning Level1
   *  2:Warning Level2
   *  <TODO:改成枚举>
   **/
  uint8 dow_front_left_warning_level;

  /*  DOW前右侧报警状态
   *  0:No Warning
   *  1:Warning Level1
   *  2:Warning Level2
   *  <TODO:改成枚举>
   **/
  uint8 dow_front_right_warning_level;

  /*  DOW后左侧报警状态
   *  0:No Warning
   *  1:Warning Level1
   *  2:Warning Level2
   *  <TODO:改成枚举>
   **/
  uint8 dow_rear_left_warning_level;

  /*  DOW后右侧报警状态
   *  0:No Warning
   *  1:Warning Level1
   *  2:Warning Level2
   *  <TODO:改成枚举>
   **/
  uint8 dow_rear_right_warning_level;
} _STRUCT_ALIGNED_ DOWOutputInfo;

typedef struct {
  /** RCW功能状态
   *  0:Unavailable
   *  1:Off
   *  2:Standby
   *  3:Active
   *  <TODO:改成枚举>
   **/
  uint8 rcw_state;

  /*  RCW报警状态
   *  0:No Warning
   *  1:Warning Level1
   *  2:Warning Level2
   *  <TODO:改成枚举>
   **/
  uint8 rcw_warning_level;
} _STRUCT_ALIGNED_ RCWOutputInfo;

typedef struct {
  /** FCTA功能状态
   *  0:Unavailable
   *  1:Off
   *  2:Standby
   *  3:Active
   *  <TODO:改成枚举>
   **/
  uint8 fcta_state;

  /*  FCTA左侧报警状态
   *  0:No Warning
   *  1:Warning Level1
   *  2:Warning Level2
   *  <TODO:改成枚举>
   **/
  uint8 fcta_left_warning_level;

  /*  FCTA右侧报警状态
   *  0:No Warning
   *  1:Warning Level1
   *  2:Warning Level2
   *  <TODO:改成枚举>
   **/
  uint8 fcta_right_warning_level;
  RiskObjInfo fcta_risk_obj;
} _STRUCT_ALIGNED_ FCTAOutputInfo;

typedef struct {
  /** RCTA功能状态
   *  0:Unavailable
   *  1:Off
   *  2:Standby
   *  3:Active
   *  <TODO:改成枚举>
   **/
  uint8 rcta_state;

  /*  RCTA左侧报警状态
   *  0:No Warning
   *  1:Warning Level1
   *  2:Warning Level2
   *  <TODO:改成枚举>
   **/
  uint8 rcta_left_warning_level;

  /*  RCTA右侧报警状态
   *  0:No Warning
   *  1:Warning Level1
   *  2:Warning Level2
   *  <TODO:改成枚举>
   **/
  uint8 rcta_right_warning_level;
  RiskObjInfo rcta_risk_obj;
} _STRUCT_ALIGNED_ RCTAOutputInfo;

typedef enum {
  FCTBActiveSource_NoActive = 0,
  FCTBActiveSource_Left = 1,
  FCTBActiveSource_Right = 2,
} _ENUM_PACKED_ FCTBActiveSourceEnum;

typedef struct {
  /** FCTB功能状态
   *  0:Unavailable
   *  1:Off
   *  2:Standby
   *  3:Active
   *  <TODO:改成枚举>
   **/
  uint8 fctb_state;
  boolean fctb_deceleration_request_status;  // FCTB减速请求状态       (true:Request/false:No Request)

  /** FCTB减速请求值
   *  单位：米/秒^2   (m/s^2)
   *  备注：AEB激活时，请求值为负值
   **/
  float32 fctb_deceleration_request_value;
  boolean fctb_hold_request_status;  // FCTB停车保压请求状态   (true:Request/false:No Request)
  RiskObjInfo fctb_risk_obj;
  FCTBActiveSourceEnum fctb_active_source;
} _STRUCT_ALIGNED_ FCTBOutputInfo;

typedef enum {
  RCTBActiveSource_NoActive = 0,
  RCTBActiveSource_Left = 1,
  RCTBActiveSource_Right = 2,
} _ENUM_PACKED_ RCTBActiveSourceEnum;
typedef struct {
  /** RCTB功能状态
   *  0:Unavailable
   *  1:Off
   *  2:Standby
   *  3:Active
   *  <TODO:改成枚举>
   **/
  uint8 rctb_state;
  boolean rctb_deceleration_request_status;  // RCTB减速请求状态       (true:Request/false:No Request)

  /** RCTB减速请求值
   *  单位：m/s^2
   *  备注：AEB激活时，请求值为负值
   **/
  float32 rctb_deceleration_request_value;
  boolean rctb_hold_request_status;  // RCTB停车保压请求状态   (true:Request/false:No Request)
  RiskObjInfo rctb_risk_obj;
  RCTBActiveSourceEnum rctb_active_source;
} _STRUCT_ALIGNED_ RCTBOutputInfo;

typedef enum {
  DAIState_Init = 0,
  DAIState_Off = 1,
  DAIState_Standby = 2,
  DAIState_Active = 3,
  DAIState_Unavailable = 4,
} _ENUM_PACKED_ DAIStateEnum;  // dai_state枚举值

typedef struct {
  DAIStateEnum dai_state;
  boolean dai_warning_request;  // 驶离提醒请求状态(true:Request/false:No Request)
} _STRUCT_ALIGNED_ DAIOutputInfo;

// 汇总debug的结构体
typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  FCWOutputInfo fcw_output_info;
  PCWOutputInfo pcw_output_info;
  AEBOutputInfo aeb_output_info;
  AEBPOutputInfo aebp_output_info;
  BSDOutputInfo bsd_output_info;
  LCAOutputInfo lca_output_info;
  DOWOutputInfo dow_output_info;
  RCWOutputInfo rcw_output_info;
  FCTAOutputInfo fcta_output_info;
  RCTAOutputInfo rcta_output_info;
  FCTBOutputInfo fctb_output_info;
  RCTBOutputInfo rctb_output_info;
  DAIOutputInfo dai_output_info;
} _STRUCT_ALIGNED_ ADASFunctionOutputInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_ADAS_FUNCTION_H_
