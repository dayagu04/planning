// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2024/03/29

#ifndef _IFLYAUTO_SENTIENL_MODE_H_
#define _IFLYAUTO_SENTIENL_MODE_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"
#include "adas_function_c.h"

#define HMI_PDC_RADAER_NUM 12
#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

// 哨兵模式开启关闭状态
typedef enum{
  SENTINEL_INVALID,
  SENTINEL_OFF,
  SENTINEL_ON,
  SENTINEL_RESERVERED
}_ENUM_PACKED_ SENTINEL_Mode_State;

// 哨兵模式进入， 退出请求
typedef enum{
  SENTINEL_REQUEST_NOT_ACTIVE,
  SENTINEL_REQUEST_ENTER,
  SENTINEL_REQUEST_EXIT,
  SENTINEL_REQUEST_OFF
} _ENUM_PACKED_ SENTINEL_Request;

typedef enum{
  SENTINEL_ADCC_CAMERA_STREAM_RESET = 0x0,         // 刚上电，未初始化状态
  SENTINEL_ADCC_CAMERA_STREAM_INITED = 0x1,        // ADCC侧96722 camera均device detect成功
  SENTINEL_ADCC_CAMERA_STREAM_WORKING = 0x2,       // 和ICC侧通信链路建立成功，正常通信
  SENTINEL_ADCC_CAMERA_STREAM_DEINIT_FINISH = 0x3, // ADCC侧链路去初始化完成
  SENTINEL_ADCC_CAMERA_STREAM_ERROR = 0x4,         // ADCC侧链路出现故障
  SENTINEL_ADCC_CAMERA_STREAM_RECOVERYING = 0x5,   // ADCC侧链路正在恢复中（预留）
  SENTINEL_ADCC_CAMERA_STREAM_RESERVED_6 = 0x6,    // Reserved
  SENTINEL_ADCC_CAMERA_STREAM_RESERVED_7 = 0x7     // Reserved
}_ENUM_PACKED_ SENTINEL_ADCC_CameraStreamSts;

typedef enum{
  SENTINEL_ADCC_CAMERA_NO_ERROR = 0x0,
  SENTINEL_ADCC_CAMERA_ERROR_CODE1 = 0x1,
  SENTINEL_ADCC_CAMERA_ERROR_CODE2 = 0x2,
  SENTINEL_ADCC_CAMERA_ERROR_CODE3 = 0x3,
  SENTINEL_ADCC_CAMERA_ERROR_RESERVED = 0x4
}_ENUM_PACKED_ SENTINEL_ADCC_CameraErrorCode;


typedef enum{
  SENTINEL_VCU_HV_LmpOFF = 0x0,
  SENTINEL_VCU_HV_LmpON = 0x1
}_ENUM_PACKED_ SENTINEL_VCU_HVReady;

typedef struct {
  MsgHeader msg_header;
  // MsgMeta msg_meta;
  uint8 Mode_State;
  uint8 Request;
  uint8 VCU_HVReady;
} _STRUCT_ALIGNED_ SENTINEL_Mcu2Soc;


typedef struct {
  MsgHeader msg_header;
  // MsgMeta msg_meta;
  uint8 CameraStreamSts;
  uint8 CameraErrorCode;
} _STRUCT_ALIGNED_ SENTINEL_Soc2Mcu;


#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_SENTIENL_MODE_H_