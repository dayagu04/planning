// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_POST_SALE_CALIB_H_
#define _IFLYAUTO_POST_SALE_CALIB_H_

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

#define POST_SALE_CALIB_SENSOR_NUM 12

typedef enum {
    POST_SALE_CALIB_SENSOR_FRONT_WIDE = 0, // 前视广角相机
    POST_SALE_CALIB_SENSOR_FRONT_NARROW, // 前视长焦相机
    POST_SALE_CALIB_SENSOR_FRONT_LEFT, // 左前相机
    POST_SALE_CALIB_SENSOR_FRONT_RIGHT, // 右前相机
    POST_SALE_CALIB_SENSOR_REAR_LEFT, // 后左相机
    POST_SALE_CALIB_SENSOR_REAR_RIGHT, // 后右相机
    POST_SALE_CALIB_SENSOR_REAR_NARROW, // 后视相机
    POST_SALE_CALIB_SENSOR_SVC_FRONT, // 前环视相机
    POST_SALE_CALIB_SENSOR_SVC_LEFT, // 左环视相机
    POST_SALE_CALIB_SENSOR_SVC_RIGHT, // 右环视相机
    POST_SALE_CALIB_SENSOR_SVC_REAR, // 后环视相机
    POST_SALE_CALIB_SENSOR_IMU, // 惯性测量单元
    POST_SALE_CALIB_SENSOR_SENSOR_NAME_NUM
} _ENUM_PACKED_ PostSaleCalibFSCSensorNameEnum;

/* 激活模式
 * topic: IflytekPostSaleCalibrationActivation
 * c->s: PostSaleCalibActiveInfo
 * s->c: PostSaleCalibActiveInfoResult
 */
typedef struct { 
    MsgHeader msg_header;
    MsgMeta msg_meta;
    uint8 calib_active; // 标定激活信息 0: ignore 1: active
    /*
    * FW: front_wide
    * FL: front_left
    * FR: front_right
    * FN: front_narrow
    * RN: rear_narrow
    * RL: rear_left
    * RR: rear_right
    * SFW: svc_front
    * SLW: svc_left
    * SRW: svc_right
    * SRCW: svc_rear
    * imu: imu
    */
    char calib_sensors[100]; // 需要标定的传感器，用/隔开 e.g. FW/FL/FR/RL/RR/SFW/SRW/SRCW/SLW
} _STRUCT_ALIGNED_ PostSaleCalibActiveInfo;

typedef struct { 
    MsgHeader msg_header;
    MsgMeta msg_meta;
    uint8 result; // 激活应答 0: fail 1: success
} _STRUCT_ALIGNED_ PostSaleCalibActiveInfoResult;

/* 停止售后标定
 * topic: IflytekPostSaleCalibrationStop
 * c->s: PostSaleCalibStopReqInfo
 * s->c: PostSaleCalibStopResult
 */
typedef struct { 
    MsgHeader msg_header;
    MsgMeta msg_meta;
    uint8 stop_calib; // 停止售后标定信息 0: ignore 1: active
} _STRUCT_ALIGNED_ PostSaleCalibStopReqInfo;

typedef struct { 
    MsgHeader msg_header;
    MsgMeta msg_meta;
    uint8 result; // 停止售后标定应答 0: fail 1: success
} _STRUCT_ALIGNED_ PostSaleCalibStopResult;

/* 标定结果传递
 * topic: IflytekPostSaleCalibrationResult
 * c->s: PostSaleCalibGetCalibResReqInfo
 * s->c: PostSaleCalibResults
 */
typedef struct { 
    MsgHeader msg_header;
    MsgMeta msg_meta;
    uint8 req_calib_res; // 标定结果获取信息 0: ignore 1: active
} _STRUCT_ALIGNED_ PostSaleCalibGetCalibResReqInfo;

typedef enum {
    POST_SALE_CALIB_RESULT_OK = 0, // 标定成功
    POST_SALE_CALIB_RESULT_UNCONFIRMED, // 未收敛
    POST_SALE_CALIB_RESULT_FAILED_WITH_BIG_INSTALLATION_ERROR, // 超差
    POST_SALE_CALIB_RESULT_INIT_ERROR, // 初始化失败
    POST_SALE_CALIB_RESULT_BAD_CONFIG, // 配置错误
    POST_SALE_CALIB_RESULT_BAD_DATA, // 数据错误
    POST_SALE_CALIB_RESULT_DETECT_FAILED, // 检测失败
    POST_SALE_CALIB_RESULT_CALIB_FAILED, // 标定失败
    POST_SALE_CALIB_RESULT_IGNORED, // 忽略
    POST_SALE_CALIB_RESULT_UNKNOWN_ERROR, // 未知错误
    POST_SALE_CALIB_RESULT_RESULT_NUM
} _ENUM_PACKED_ PostSaleCalibFSCResultEnum;

typedef struct { 
    uint8 status;               // 0:标定完成 1:标定中 2:未配置不需要标定
    uint8 result;               // PostSaleCalibFSCResultEnum
    char sensor_name[32];  // 传感器名称：e.g "front_wide" 
    float delta_x;
    float delta_y;
    float delta_z;
    float delta_roll;
    float delta_yaw;
    float delta_pitch;
} _STRUCT_ALIGNED_ PostSaleSensorCalibResult;

typedef struct { 
    MsgHeader msg_header;
    MsgMeta msg_meta;
    uint8 status;               // 整体标定状态 0:complete 1:running 2:idle
    uint8 result;               // 整体标定结果 0:success 1: running 2:failed
    PostSaleSensorCalibResult calib_results[POST_SALE_CALIB_SENSOR_NUM];
} _STRUCT_ALIGNED_ PostSaleCalibResults;

#pragma pack()

#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_POST_SALE_CALIB_H_