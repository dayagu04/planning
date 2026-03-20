// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_ENGINEER_MODE_H_
#define _IFLYAUTO_ENGINEER_MODE_H_

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

#define PROCESS_NAME_SIZE 64
#define MDC_VERSION_SIZE 64
#define MDC_VIN_SIZE 32
#define PROCESS_LIST_MAX_SIZE 128
#define SFTP_USR_NAME_SIZE 64
#define SFTP_PASSWORD_SIZE 64
#define SFTP_FTP_PATH_SIZE 128
#define FAULT_INFO_SIZE 256

typedef enum {
  MDC = 0,      // MDC日志
  IFLY = 1      // 讯飞日志
} _ENUM_PACKED_ Log_Mode;

typedef enum {
  EM_NO_Request = 0,   // 未请求
  EM_ON         = 1,   // 开
  EM_OFF        = 2    // 关
} _ENUM_PACKED_ EM_Swt;


typedef struct {
  int64_t start_time; // 日志开始时间
  int64_t stop_time;  // 日志结束时间
  Log_Mode logmode;   // 日志模块
} _STRUCT_ALIGNED_ Log_Req;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  EM_Swt engineer_mode;       // 工程模式状态
  EM_Swt fault_degraded_swt;  // 故障降级开关
  EM_Swt seatbelt_swt;        // 安全带检测开
  EM_Swt lever_swt;           // 拨杆激活开关
  EM_Swt str_swt;             // 浅休眠开关
  Log_Req log_req;            // 日志打包请求
} _STRUCT_ALIGNED_ Engineer_Inner;



typedef enum {
  IDLE        = 0,  // 未运行
  STARTING    = 1,  // 正在拉起中
  RUNNING     = 2,  // 正在运行中
  TERMINATING = 3,  // 正在正常关闭中
  TERMINATED  = 4,  // 正常关闭
  ABORTED     = 5   // 非正常关闭 包括 coredump、异常abort、进程启动时却库、权限访问等...
} _ENUM_PACKED_ EM_Status;

typedef struct { 
  char process_name[PROCESS_NAME_SIZE];   // 进程名
  EM_Status em_status;                    // EM状态
} _STRUCT_ALIGNED_ Process_Status;

typedef struct {
  char version[MDC_VERSION_SIZE];                           // MDC版本
  bool factory_sts;                                         // 下线状态
  bool calibration_sts;                                     // 标定状态
  char veh_vin[MDC_VIN_SIZE];                               // 车辆VIN码
  uint16 process_num;                                       // 进程实际个数
  Process_Status process_status[PROCESS_LIST_MAX_SIZE];     // 进程状态
} _STRUCT_ALIGNED_ MDC_Status;

typedef struct {
  char usr_name[SFTP_USR_NAME_SIZE];
  char password[SFTP_PASSWORD_SIZE];
  char ftppath[SFTP_FTP_PATH_SIZE];
} _STRUCT_ALIGNED_ Sftp_Info;

typedef struct {
  uint32_t alarmId;
  uint32_t alarmObj;
} _STRUCT_ALIGNED_ Engineer_FaultInfo;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  MDC_Status mdc_status;
  uint64_t fault_info_size;
  Engineer_FaultInfo fault_info[FAULT_INFO_SIZE];
  Sftp_Info sftp_info;
} _STRUCT_ALIGNED_ Engineer_Outer;

#pragma pack()

#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_ENGINEER_MODE_H_