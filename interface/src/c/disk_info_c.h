// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef DISK_INFO_H
#define DISK_INFO_H

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif
#define DISK_INFO_MAX_NUM 2
#pragma pack(4)

typedef enum {
    OBD_UPDATE_NOT_OCCURRED = 0,        // 未发生近端升级
    OBD_UPDATE_OCCURRED = 1             // 发生近端升级
} _ENUM_PACKED_ ObdUpdateStatus;        // 近端升级状态

typedef struct {
    MsgHeader msg_header;
    ObdUpdateStatus obd_update_status;      // 近端升级状态
} _STRUCT_ALIGNED_ ObdUpdateEvent;          // 近端升级信息

typedef enum{
    PLATFORM_ID_SOC_A = 0,          //A面
    PLATFORM_ID_SOC_B = 1           //B面
} _ENUM_PACKED_ PlatformIDType;    //SocID

typedef enum{
    STORAGE_TYPE_UFS = 0,
    STORAGE_TYPE_SSD = 1,
    STORAGE_TYPE_EMMC = 2
} _ENUM_PACKED_ StorageType;     //存储器类型

typedef struct {
    MsgHeader      msg_header;
    PlatformIDType platform_id;  //SocA or SocA
    uint8          life;         //存储器寿命
    uint8          temperature;  //温度
    StorageType    storage_type; //存储器类型
    uint64         capacity;    // 容量，Bytes
} _STRUCT_ALIGNED_ StorageInfo;    //PHM_A->FDL  PHM_B ->FDL

typedef struct {
    MsgHeader      msg_header;
    PlatformIDType platform_id;      // SocA or SocB
    uint64         free_space;       //空余空间大小，Bytes
} _STRUCT_ALIGNED_ DiskSpaceInfo;     //磁盘空间状态  PHMA->OTA  PHMB->OTA

typedef enum {
    DISK_CLEAN_FLAG_DISENABLE = 0,
    DISK_CLEAN_FLAG_ENABLE = 1
} _ENUM_PACKED_ DiskCleanFlagType;    //执行 or 不执行 磁盘清理

typedef struct {
    PlatformIDType         platform_id;       //SocA or SocB
    DiskCleanFlagType      disk_clean_flag;   //是否执行磁盘清理
    uint64                 reserve_space;    //保证磁盘空余空间大小，Bytes
    uint64                 clean_space;       //磁盘清理大小，Bytes
} _STRUCT_ALIGNED_ DiskCleanRequest;

typedef struct {
    MsgHeader           msg_header;
    DiskCleanRequest    disk_clean_request[DISK_INFO_MAX_NUM];
} _STRUCT_ALIGNED_ DiskCleanRequestList;      //OTA->PHM_A OTA->PHM_B  PHM_A->FDL  PHM_B->FDL

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // DISK_INFO_H