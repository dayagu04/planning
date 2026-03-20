// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2024/05/30

#ifndef _IFLYAUTO_IFLY_SENSOR_IMU_H_
#define _IFLYAUTO_IFLY_SENSOR_IMU_H_

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

typedef struct {
  MsgHeader msg_header;          // SOC消息发送信息
  SensorMeta sensor_meta;        // CAN报文到达MCU信息
  uint8 imu_name[PROTO_STR_LEN]; // 传感器名
  int8 temperature_val;          // 温度(摄氏度)
  boolean gyro_selftest_result;  // 陀螺仪自检结果(true:有效 / fasle:无效)
  boolean acc_selftest_result;   // 加速度计自检结果(true:有效 / fasle:无效)
  Point3f acc_val;               // 加速度值(m/s2)

  /** 角速度
   *  单位:rad/s
   *  备注:x滚转角速度，y俯仰角速度，z航向角速度；延坐标轴正方向看，顺时针为正
   **/
  Point3f angular_rate_val;
} _STRUCT_ALIGNED_ IFLYIMU;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_IFLY_SENSOR_IMU_H_