// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_USS_WAVE_INFO_H_
#define _IFLYAUTO_USS_WAVE_INFO_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#define USS_WAVE_SNS_DIS_BUF_NUM 5
#define USS_WAVE_DIS_INFO_VALUE_NUM 5
#define USS_WAVE_DIS_INFO_VALUE_TYPE_NUM 12
#define USS_WAVE_NUM 12
#define USS_WAVE_UPA_ORI_DIS_NUM 8
#define USS_WAVE_APA_ORI_DIS_NUM 4
#define USS_WAVE_APA_DIS_NUM 4
#define USS_WAVE_UPA_DIS_NUM 2

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

typedef struct {
  float32 apa_sns_dtd_obj_first;    // 障碍物1距离  (米)
  uint8 obj1_type;                  // 障碍物1类型  <TODO:当前未使用，使用时需要改成枚举>
  float32 apa_sns_dtd_obj_sencond;  // 障碍物2距离  (米)
  uint8 obj2_type;                  // 障碍物2类型  <TODO:当前未使用，使用时需要改成枚举>
  float32 apa_sns_dtd_obj_three;    // 障碍物3距离  (米)
  uint8 obj3_type;                  // 障碍物3类型  <TODO:当前未使用，使用时需要改成枚举>
} _STRUCT_ALIGNED_ APASnsDtdObjDisInfoType;

typedef struct {
  APASnsDtdObjDisInfoType apa_sns_dis_buf[USS_WAVE_SNS_DIS_BUF_NUM];  // 障碍物缓存buff    <固定5个>
  uint8 apa_sns_dtd_obj_dis_info_buf_read_index;             // 障碍物buff读索引
  uint8 apa_sns_dtd_obj_dis_info_buf_write_index;            // 障碍物buff写索引
  uint32 apa_sns_dtd_obj_dis_info_counter;                   // 障碍物数量
} _STRUCT_ALIGNED_ APASnsDisProcessType;

typedef struct {
  float32 wdis_value[USS_WAVE_DIS_INFO_VALUE_NUM];
} _STRUCT_ALIGNED_ DisValueType;

typedef struct {
  uint32 wtype_value[USS_WAVE_DIS_INFO_VALUE_NUM];
} _STRUCT_ALIGNED_ TypeValueType;

typedef struct {
  float32 wx_value[USS_WAVE_DIS_INFO_VALUE_NUM];
} _STRUCT_ALIGNED_ XValueType;

typedef struct {
  float32 wy_value[USS_WAVE_DIS_INFO_VALUE_NUM];
} _STRUCT_ALIGNED_ YValueType;

typedef struct {
  DisValueType wdis[USS_WAVE_DIS_INFO_VALUE_TYPE_NUM];    // 障碍物距离   (米)    <固定12*5个>
  TypeValueType wtype[USS_WAVE_DIS_INFO_VALUE_TYPE_NUM];  // 障碍物类型   (米)    <固定12*5个>
  XValueType wx[USS_WAVE_DIS_INFO_VALUE_TYPE_NUM];        // 障碍物X坐标  (米)    <固定12*5个>
  YValueType wy[USS_WAVE_DIS_INFO_VALUE_TYPE_NUM];        // 障碍物Y坐标  (米)    <固定12*5个>
} _STRUCT_ALIGNED_ DtObjUPASnsDtObjDisInfoType;

typedef struct {
  MsgHeader msg_header;                                       // SOC消息发送信息
  SensorMeta sensor_meta;                                     // CAN报文到达MCU信息
  /** 超声波雷达状态
   *  true:valid / false:invalid
   *  备注：固定12个雷达，<TODO:顺序待补充>
   **/
  boolean uss_state[USS_WAVE_NUM];
  float32 upa_ori_dis_buffer[USS_WAVE_UPA_ORI_DIS_NUM];                // upa原始距离信号  <固定8组>
  float32 apa_ori_dis_buffer[USS_WAVE_APA_ORI_DIS_NUM];                // apa原始距离信号  <固定4组>
  APASnsDisProcessType apa_dis_info_buf[USS_WAVE_APA_DIS_NUM];         // apa障碍物缓存    <固定4组>
  DtObjUPASnsDtObjDisInfoType upa_dis_info_buf[USS_WAVE_UPA_DIS_NUM];  // apa障碍物缓存    <固定2组>
} _STRUCT_ALIGNED_ UssWaveInfo;


/* ======================================================= */
/* =                     valeo                           = */
/* ======================================================= */
#define USS_WAVE_PDC_SONAR_NUM 12
#define USS_WAVE_PDC_PRIV_POINT_NUM 150
/* POINT_PRIV_TIMESTAMP_NUM - CAN_RECV_PRIV_0_6_POINT_0X290_POS = 22*/
#define USS_WAVE_POINT_PRIV_TIMESTAMP_NUM 22

// Pdc距离信息
typedef struct {
 float32 pas_sonarx_distance; /*x号雷达距离，单位:m*/
 uint8 pas_sonarx_blind;/*x号雷达盲区，0：False,1:True*/
 float32 pas_sonarx_ditance_time;/*x号雷达发波时间戳，单位：ms*/
 uint16 pas_sonarx_cross_distance_left;/*x号雷达发、x-1号雷达收的间接回波的距离，单位:cm*/
 uint16 pas_sonarx_cross_distance_right;/*x号雷达发、x+1号雷达收的间接回波的距离，单位:cm*/
 float32 pas_sonarx_cross_ditance_left_time;/*x号雷达发、x-1号雷达收的间接回波的距离时间戳——发波时间戳,单位:ms。0号和6号雷达没有这个值*/
 float32 pas_sonarx_cross_ditance_right_time;/*x号雷达发、x+1号雷达收的间接回波的距离时间戳——发波时间戳,单位:ms。5号和11号雷达没有这个值*/
 uint8 pas_sonarx_confidence; /*x号雷达置信度，单位:%*/
 uint8 pas_sonarx_counter;/*x号雷达探头发波计数器*/
 float32 pas_sonarx_tof1_distance;/*x号雷达一次回波距离*/
 float32 pas_sonarx_tof2_distance;/*x号雷达二次回波距离*/
 float32 pas_sonarx_ringing_time;/*x号雷达余震时间，单位:us*/
 uint16 pas_sonarx_echo_tof1;/*x号雷达一次回波时间，单位:us*/
 uint16 pas_sonarx_echo_width1;/*x号雷达一次回波宽度，单位:us*/
 uint16 pas_sonarx_echo_tof2;/*x号雷达二次回波时间，单位:us*/
 uint16 pas_sonarx_echo_width2;/*x号雷达二次回波宽度，单位:us*/
 uint8 pas_sonarx_echo_peak1;/*x号雷达一次回波高度*/
 uint8 pas_sonarx_echo_peak2;/*x号雷达二次回波高度*/
 uint8 pas_sonarx_emit;/*对应探头是否发波,0x0:off;0x1:receive only;0x2:emit and receive*/
} _STRUCT_ALIGNED_ UssPdcPasSonarDistanceType;

// 点云数据
typedef struct {
  uint8 point_id;/*点的ID*/
  boolean point_valid;/*点的ID是否有效，0：invalid,1:valid*/
  int16 point_x;/*点X的坐标,单位:cm*/ 
  int16 point_y;/*点Y的坐标,单位:cm*/ 
  uint8 point_error_a;/*点的偏差（椭圆长轴）,单位:cm*/
  uint8 point_error_b;/*点的偏差（椭圆短轴）,单位:cm*/
  float32 point_error_theta;/*点的偏差（椭圆角度）,-180~+180°*/
  uint8 point_high;/*点的高低，0：Low,1:High,2:Unkown*/
  uint8 point_exist_probability;/*点的存在概率，单位:%*/
} _STRUCT_ALIGNED_ UssPdcPrivPointDataType;


typedef struct {
  float32 point_priv_timestamp[USS_WAVE_POINT_PRIV_TIMESTAMP_NUM];/*点的timestamp,单位:ms*/
  UssPdcPrivPointDataType priv_point_data_prop[USS_WAVE_PDC_PRIV_POINT_NUM];
} _STRUCT_ALIGNED_ UssPdcPrivPointType;

/*MCU发送给SOC的数据类型
  Pdc上行数据
*/
typedef struct {
  MsgHeader msg_header;                                       // SOC消息发送信息
  SensorMeta sensor_meta;     
  UssPdcPasSonarDistanceType sonar_distance_data[USS_WAVE_PDC_SONAR_NUM];
  UssPdcPrivPointType priv_point_data;/*2024-03-29,为了减少通用版本的proto的适配工作量，暂时不发这个数据*/
} _STRUCT_ALIGNED_ UssPdcIccSendDataType;

// Pdc需要的定位信息，预留
typedef struct {
  float32 upc_vehicle_position_x; /*车辆的X坐标,单位:m*/
  float32 upc_vehicle_position_y; /*车辆的Y坐标,单位:m*/
  float32 upc_vehicle_position_update_time;/*车辆坐标更新的时间戳,单位:ms*/
  float32 upc_vehicle_position_curvature;/*车辆的行驶曲率半径,单位:1/m*/
  float32 upc_vechile_position_yaw_angle;/*车辆的横摆角,单位:rad*/
  float32 upc_vechile_position_drive_distance;/*车辆的行驶距离,单位:m*/
} _STRUCT_ALIGNED_ UssPdcUpcVehiclePosDataType;

/*MCU接收SOC的数据类型
 Pdc下行数据 ，预留
*/
typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  UssPdcUpcVehiclePosDataType upc_vehicle_pos_data;
} _STRUCT_ALIGNED_ UssPdcIccRecvDataType;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_USS_WAVE_INFO_H_