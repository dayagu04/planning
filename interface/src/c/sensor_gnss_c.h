// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2024/05/30

#ifndef _IFLYAUTO_IFLY_SNESOR_GNSS_H_
#define _IFLYAUTO_IFLY_SNESOR_GNSS_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#define GNSS_SATELLITE_NUM 20

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

typedef enum {
  IFLY_GNSS_QUALITY_INVALID = 0,          // Invalid
  IFLY_GNSS_QUALITY_SINGLE_POSITION = 1,  // Single Position
  IFLY_GNSS_QUALITY_DGPS = 2,             // DGPS
  IFLY_GNSS_QUALITY_RTK_FIXED = 4,        // RTK Fixed
  IFLY_GNSS_QUALITY_RTK_FLOAT = 5,        // RTK Float
  IFLY_GNSS_QUALITY_DEAD_RECKONING = 6,   // Dead Reckoning
} _ENUM_PACKED_ IFLY_GNSS_QUALITY;

typedef enum {
  IFLY_GNSS_SOURCE_UNKNOWN = 0,   // unknown
  IFLY_GNSS_SOURCE_TBOX = 1,      // Tbox
  IFLY_GNSS_SOURCE_RTK = 2,       // RTK
  IFLY_GNSS_SOURCE_RESERVED = 3,  // reserved
} _ENUM_PACKED_ IFLY_GNSS_SOURCE;

typedef enum {
  IFLY_GNSS_POS_STATUS_INVALID = 0,          // Invalid
  IFLY_GNSS_POS_STATUS_DR = 1,               // DR
  IFLY_GNSS_POS_STATUS_2D_Fixed = 2,         // 2D
  IFLY_GNSS_POS_STATUS_3D_Fixed = 3,         // 3D
  IFLY_GNSS_POS_STATUS_GNSS_DR = 4,          // GNSS+DR
  IFLY_GNSS_POS_STATUS_ONLY_TIME_VALID = 5,  // ONLY Time Valid
} _ENUM_PACKED_ IFLY_GNSS_POS_STATUS;

// 卫星星历信号
typedef enum {
  IFLY_GNSS_TYPE_GPS = 0,      // gps
  IFLY_GNSS_TYPE_BEIDOU = 1,   // 北斗
  IFLY_GNSS_TYPE_GLONASS = 2,  // glonass
  IFLY_GNSS_TYPE_GALILEO = 3,  // galileo
  IFLY_GNSS_TYPE_QZSS = 4,     // qzss
  IFLY_GNSS_TYPE_SBAS = 5,     // SBAS
  IFLY_GNSS_TYPE_UNKNOWN = 6,  // unknown
} _ENUM_PACKED_ IFLY_GNSS_TYPE;

typedef struct {
  IFLY_GNSS_TYPE gnss_type;       // 定位系统
  uint32 gnss_num_in_view;        // 可见卫星总数
  uint32 gnss_sat_prn;            // 卫星编号
  uint32 gnss_sat_snr;            // 信噪比                [0-99]
  uint32 gnss_sat_elev;           // 仰角          (度)    [0-90]
  uint32 gnss_sat_azimuth;        // 方位角        (度)    [0-359]
} _STRUCT_ALIGNED_ IFLYGnssSatelliteInfo;

typedef struct {
  float64 gnss_altitude;   // 海拔高度       (米)
  float64 gnss_ellipsoid;  // GnssEllipsoid

  IFLY_GNSS_SOURCE gnss_source;               // gnss数据来源
  LLHType gnss_llh_type;                      // 坐标系类型
  IFLY_GNSS_QUALITY gnss_quality;             // gnss定位质量
  IFLY_GNSS_POS_STATUS gnss_pos_status;       // gnss定位状态
  uint32 gnss_num_satellites;                 // 卫星数目
  float32 gnss_tdop;                          // GNSS钟差精度因子(缺省)
  float32 gnss_hdop;                          // GNSS水平精度因子
  float32 gnss_vdop;                          // GNSS垂直精度因子

  float32 gnss_heading;      // GNSS载体航向角   (度)
  float32 gnss_course;       // GNSS运动航迹角   (度)
  float32 gnss_heading_err;  // GNSS航向角误差   (度)
  float32 gnss_course_err;   // GNSS航迹角误差   (度)

  float64 gnss_lat;  // GNSS定位纬度    (度)
  float64 gnss_lon;  // GNSS定位经度    (度)

  float32 gnss_horipos_err;  // GNSS水平位置精度 (米)
  float32 gnss_vertpos_err;  // GNSS垂向位置精度 (米)
  float32 gnss_horivel_err;  // GNSS水平速度精度 (米/秒)
  float32 gnss_vertvel_err;  // GNSS垂向速度精度 (米/秒)

  float32 gnss_vel_north;  // GNSS北东地速度-北  (米/秒)
  float32 gnss_vel_east;   // GNSS北东地速度-东  (米/秒)
  float32 gnss_vel_down;   // GNSS北东地速度-地  (米/秒)
} _STRUCT_ALIGNED_ IFLYGnssMsg;

typedef struct {
  uint32 angle_heading;  // 航向角   (度)
  float32 angle_pitch;   // 俯仰角   (度)
  float32 angle_roll;    // 横滚角   (度)
} _STRUCT_ALIGNED_ IFLYInsInfo;

typedef struct {
  MsgHeader msg_header;                                           // SOC消息发送信息
  SensorMeta sensor_meta;                                         // CAN报文到达MCU信息
  uint8 gnss_name[PROTO_STR_LEN];                                 // 传感器名
  IFLYGnssMsg gnss_msg;                                           // gnss报文信息
  IFLYGnssSatelliteInfo gnss_satellite_info[GNSS_SATELLITE_NUM];  // GNSS satellite info
  IFLYInsInfo ins_info;                                           // INS姿态角
} _STRUCT_ALIGNED_ IFLYGnss;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_IFLY_SNESOR_GNSS_H_