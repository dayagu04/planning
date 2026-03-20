// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_IFLY_MAPPING_STATUS_H_
#define _IFLYAUTO_IFLY_MAPPING_STATUS_H_

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

// 建图运行状态
typedef enum {
    RUNNING_STATUS_WAITING = 0,           // 等待建图
    RUNNING_STATUS_MAPPING,               // 建图进行中
    RUNNING_STATUS_WAITING_FOR_MAP_INFO,  // 建图等待存储信息中
    RUNNING_STATUS_SAVING_MAP,            // 建图保存中
    RUNNING_STATUS_FINISHED,              // 建图完成
    RUNNING_STATUS_ERROR,                 // 建图失败，对应原因在FailedReason
} _ENUM_PACKED_ MappingStatusInfoRunningStatus;

// 建图失败原因，待产品补充需要展示原因信息
typedef enum {
    FAILED_REASON_NONE = 0,                             // 默认值，NONE
    FAILED_REASON_SAVE_FAILED_GPS_LOSS_POINT_NO_READY,  // 建图保存失败, 未收到消星点
    FAILED_REASON_SAVE_FAILED_OPEN_FILE_FAILED,         // 建图保存失败, 打开文件失败
    FAILED_REASON_NO_READY,                             // 建图条件不满足
} _ENUM_PACKED_ MappingStatusInfoFailedReason;

// 自车是否在车位内
typedef enum {
    IN_PARKING_SLOT_STATUS_NONE = 0,  // 默认值，NONE
    IS_IN_PARKING_SLOT,               // 在车位内
    IS_NOT_IN_PARKING_SLOT,           // 不在车位内
} _ENUM_PACKED_ MappingStatusInParkingSlot;

typedef struct _MappingStatusInfo {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  float driving_distance;   // 建图过程的路程长度 (单位：m)
  float backward_distance;  // 建图过程的后退路程长度 (单位：m)
  uint32 saved_progress;    // 建图保存进度 [0, 100]

  MappingStatusInfoRunningStatus running_status;  // 建图状态
  MappingStatusInfoFailedReason failed_reason;    // 建图失败原因
  MappingStatusInParkingSlot is_in_parking_slot;  // 自车是否在车位内
  int32 in_slot_id;                                 // 自车在车位内的车位ID
} _STRUCT_ALIGNED_ MappingStatusInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_IFLY_MAPPING_STATUS_H_
