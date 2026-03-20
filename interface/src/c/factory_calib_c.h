// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_FACTORY_CALIB_H_
#define _IFLYAUTO_FACTORY_CALIB_H_

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

#define FACTORY_CALIB_SVC_CAMERA_NUM 4
typedef enum { 
    FACTORY_CALIB_SVC_CAMERA_INDEX_SVC_FRONT = 0x0,
    FACTORY_CALIB_SVC_CAMERA_INDEX_SVC_REAR = 0x1,       
    FACTORY_CALIB_SVC_CAMERA_INDEX_SVC_LEFT = 0x2, 
    FACTORY_CALIB_SVC_CAMERA_INDEX_SVC_RIGHT = 0x3,
    FACTORY_CALIB_SVC_CAMERA_INDEX_SVC_CAMERA_NUM = 0x4
} _ENUM_PACKED_ FactoryCalibSVCCameraIndex;

#define FACTORY_CALIB_PANORAMA_CAMERA_NUM 7
typedef enum {
    FACTORY_CALIB_PANORAMA_CAMERA_INDEX_PANORAMA_FRONT120 = 0x0, 
    FACTORY_CALIB_PANORAMA_CAMERA_INDEX_PANORAMA_FRONT30 = 0x1,
    FACTORY_CALIB_PANORAMA_CAMERA_INDEX_PANORAMA_REAR = 0x2,       
    FACTORY_CALIB_PANORAMA_CAMERA_INDEX_PANORAMA_SIDE_FRONTLEFT = 0X3, 
    FACTORY_CALIB_PANORAMA_CAMERA_INDEX_PANORAMA_SIDE_FRONTRIGHT = 0X4, 
    FACTORY_CALIB_PANORAMA_CAMERA_INDEX_PANORAMA_SIDE_REARLEFT = 0X5, 
    FACTORY_CALIB_PANORAMA_CAMERA_INDEX_PANORAMA_SIDE_REARRIGHT = 0X6,
    FACTORY_CALIB_PANORAMA_CAMERA_INDEX_PANORAMA_CAMERA_NUM = 0X7 
} _ENUM_PACKED_ FactoryCalibPanoramaCameraIndex;

typedef enum { 
    FACTORY_CALIB_CAR_MODE_NORMAL_MODE = 0X0,     // 普通模式
    FACTORY_CALIB_CAR_MODE_FACTORY_MODE = 0X1,    // 工厂模式    
    FACTORY_CALIB_CAR_MODE_TRANSPORT_MODE = 0X2,  // 运输模式
    FACTORY_CALIB_CAR_MODE_RESERVED1 = 0X3, // 预留
    FACTORY_CALIB_CAR_MODE_RESERVED2 = 0X4, // 预留
    FACTORY_CALIB_CAR_MODE_RESERVED3 = 0X5, // 预留
    FACTORY_CALIB_CAR_MODE_RESERVED4 = 0X6, // 预留
    FACTORY_CALIB_CAR_MODE_RESERVED5 = 0X7  // 预留
} _ENUM_PACKED_ FactoryCalibCarModeEnum;

#define FACTORY_CALIB_FSC_SENSOR_NUM 12
typedef enum {
    FACTORY_CALIB_FSC_SENSOR_FRONT_WIDE = 0, // 前视广角相机
    FACTORY_CALIB_FSC_SENSOR_FRONT_NARROW, // 前视长焦相机
    FACTORY_CALIB_FSC_SENSOR_FRONT_LEFT, // 左前相机
    FACTORY_CALIB_FSC_SENSOR_FRONT_RIGHT, // 右前相机
    FACTORY_CALIB_FSC_SENSOR_REAR_LEFT, // 后左相机
    FACTORY_CALIB_FSC_SENSOR_REAR_RIGHT, // 后右相机
    FACTORY_CALIB_FSC_SENSOR_REAR_NARROW, // 后视相机
    FACTORY_CALIB_FSC_SENSOR_SVC_FRONT, // 前环视相机
    FACTORY_CALIB_FSC_SENSOR_SVC_LEFT, // 左环视相机
    FACTORY_CALIB_FSC_SENSOR_SVC_RIGHT, // 右环视相机
    FACTORY_CALIB_FSC_SENSOR_SVC_REAR, // 后环视相机
    FACTORY_CALIB_FSC_SENSOR_IMU, // 惯性测量单元
    FACTORY_CALIB_FSC_SENSOR_SENSOR_NAME_NUM
}_ENUM_PACKED_ FactoryCalibFSCSensorNameEnum;

/* 车辆模式信息
 * topic: IflytekHmiFactoryMode
 * 用于判定是否处于标定模式
 */
typedef struct { 
    uint8 car_mode; // 车辆模式，枚举FactoryCalibCarModeEnum
} _STRUCT_ALIGNED_ FactoryCalibCarModeInfo;

/* 激活模式
 * topic: IflytekFactoryCalibrationActivation
 * c->s: FactoryCalibActiveInfo
 * s->c: FactoryCalibActiveInfoResult
 */
typedef struct { 
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
} _STRUCT_ALIGNED_ FactoryCalibActiveInfo;

typedef struct { 
    uint8 result; // 激活应答 0: fail 1: success
} _STRUCT_ALIGNED_ FactoryCalibActiveInfoResult;

/* 标定结果传递
 * topic: IflytekFactoryCalibrationResult
 * c->s: FactoryCalibGetCalibResReqInfo
 * s->c: FactoryCalibResults
 */
typedef struct { 
    uint8 req_calib_res; // 标定结果获取信息 0: ignore 1: active
}_STRUCT_ALIGNED_ FactoryCalibGetCalibResReqInfo;

typedef enum {
    FACTORY_CALIB_FSC_RESULT_OK = 0, // 标定成功
    FACTORY_CALIB_FSC_RESULT_BAD_REF_POINTS_INFO, // 错误的参考点信息
    FACTORY_CALIB_FSC_RESULT_WRONG_SENSOR_NUM, // 标定传感器数量错误
    FACTORY_CALIB_FSC_RESULT_BAD_REF_POINT_NUM, // 错误的参考点数量
    FACTORY_CALIB_FSC_RESULT_BAD_REF_EXTRINSIC, // 传感器设计值错误
    FACTORY_CALIB_FSC_RESULT_BAD_INTRINSIC, // 相机内参错误
    FACTORY_CALIB_FSC_RESULT_BAD_SENSOR_DATA, // 传感器数据错误
    FACTORY_CALIB_FSC_RESULT_BAD_SENSOR_DATA_NUM, // 传感器数据数量错误
    FACTORY_CALIB_FSC_RESULT_PATTERN_NOT_FOUND, // 靶标检测失败
    FACTORY_CALIB_FSC_RESULT_CALIBRATION_FAILED, // 标定失败
    FACTORY_CALIB_FSC_RESULT_SINGLE_TURB_VALIDATION_FAILED, // 单扰动验证失败
    FACTORY_CALIB_FSC_RESULT_OVERLAP_TURB_VALIDATION_FAILED, // 共视扰动验证失败
    FACTORY_CALIB_FSC_RESULT_REPROJECT_VALIDATION_FAILED, // 投影验证失败
    FACTORY_CALIB_FSC_RESULT_POINT_TO_PLANE_VALIDATION_FAILED, // 点面验证失败
    FACTORY_CALIB_FSC_RESULT_POINT_TO_POINT_VALIDATION_FAILED, // 点点验证失败
    FACTORY_CALIB_FSC_RESULT_POINT_TO_PLANE_TURB_FAILED, // 点面扰动验证失败
    FACTORY_CALIB_FSC_RESULT_LIDAR_ROI_CHECK_VALIDATION_FAILED, // 激光雷达区域检查失败
    FACTORY_CALIB_FSC_RESULT_IMU_VALIDATION_FAILED, // IMU验证失败
    FACTORY_CALIB_FSC_RESULT_ERROR_BAD_ROLL_RESULT, // roll角超差
    FACTORY_CALIB_FSC_RESULT_ERROR_BAD_PITCH_RESULT, // pitch角超差
    FACTORY_CALIB_FSC_RESULT_ERROR_BAD_YAW_RESULT, // yaw角超差
    FACTORY_CALIB_FSC_RESULT_ERROR_BAD_X_RESULT, // x平移超差
    FACTORY_CALIB_FSC_RESULT_ERROR_BAD_Y_RESULT, // y平移超差
    FACTORY_CALIB_FSC_RESULT_ERROR_BAD_Z_RESULT, // z平移超差
    FACTORY_CALIB_FSC_RESULT_ERROR_VEHICLE_NOT_STATIONARY, // 车辆未静止
    FACTORY_CALIB_FSC_RESULT_TIMEOUT, // 标定超时
    FACTORY_CALIB_FSC_RESULT_TERMINATE_ABNORMALLY, // 终端异常
    FACTORY_CALIB_FSC_RESULT_UNKNOWN_ERROR, // 未知错误
    FACTORY_CALIB_FSC_RESULT_INIT_ERROR, // 初始化失败
    FACTORY_CALIB_FSC_RESULT_IGNORED, // 忽略
    FACTORY_CALIB_FSC_RESULT_RESULT_NUM
}_ENUM_PACKED_ FactoryCalibFSCResultEnum;

typedef struct { 
    uint8 status;               // 0:标定完成 1:标定中 2:未配置不需要标定
    uint8 result;               // 对应FactoryCalibFSCResultEnum 当前是0-29，错误码详细含义详见工厂标定车端集成需求 章节3.3 
    char sensor_name[32];  // 传感器名称：e.g "front_wide" 
    float install_error_rx;     // x轴旋转角度 
    float install_error_ry;     // y轴旋转角度 
    float install_error_rz;     // z轴旋转角度 
    float install_error_tx;     // x轴平移米 
    float install_error_ty;     // y轴平移米 
    float install_error_tz;     // z轴平移米
}_STRUCT_ALIGNED_ FactorySensorCalibResult;

typedef struct { 
    uint8 status;               // 整体标定状态 0:complete 1:running 2:idle
    uint8 result;               // 整体标定结果 
    FactorySensorCalibResult calib_results[FACTORY_CALIB_FSC_SENSOR_NUM];
}_STRUCT_ALIGNED_ FactoryCalibResults;

typedef struct { 
    uint8 result; // 0: fail 1:success
    uint8 name; // 名称，枚举FactoryCalibFSCSensorNameEnum
    uint8 model; // 1 代表针孔(pinhole model)，2 代表KB (fisheye model)
    char hardware_info[64]; // 硬件版本号 
    char sn[64]; // 唯一硬件识别码 
    float64 fx; // 焦距fx 
    float64 fy; // 焦距fy 
    float64 cx; // 主点cx 
    float64 cy; // 主点cy 
    uint32 width; // 图像宽 
    uint32 height; // 图像高 
    uint8 distortion_param_size; // 畸变参数数组大小，distortion_param_array有效值
    // 可以固定大小20个，具体参数一般是5-8个数字，多余填0，具体看供应商内参
    // pinhole model (针孔): k1 k2 p1 p2 k3 k4 k5 k6（参数数量不定，以供应商为准）
    // 针孔畸变参数特别举例说明
    // 若供应商标出来的针孔内参是 k1 k2 p1 p2 k3，则distortion_param_size为5, distortion_param_array为[k1,k2,p1,p2,k3, 0, 0, 0]
    // 若供应商标出来的针孔内参是 k1 k2 p1 p2 k3 k4 k5 k6的话，则distortion_param_size为8, distortion_param_array为[k1, k2, p1, p2, k3, k4, k5, k6]
    // 若供应商标出来的针孔内参是 k1 k2 k3（特殊情况），则distortion_param_size为5, distortion_param_array为[k1,k2,0,0,k3, 0, 0, 0]
    // fisheye model (KB): k1 k2 k3 k4
    float64 distortion_param_array[20]; 
}_STRUCT_ALIGNED_ FactoryCalibIntriParam;

/* 环视相机内参请求
 * topic: IflytekSensorGetSurroundCameraIntriParam
 * c->s: FactoryCalibSvcIntriParamReqInfo
 * s->c: FactoryCalibSvcIntriParamArray
 */
typedef struct { 
    uint8 svc_intri_param_req; //环视相机内参请求 0: ignore 1: active
}_STRUCT_ALIGNED_ FactoryCalibSvcIntriParamReqInfo;

typedef struct {
    uint8 size;  // 有效相机数
    FactoryCalibIntriParam parameter[FACTORY_CALIB_SVC_CAMERA_NUM];  // 相机内参，排布顺序FactoryCalibSVCCameraIndex
}_STRUCT_ALIGNED_ FactoryCalibSvcIntriParamArray;

/* 周视相机内参请求
 * topic: IflytekSensorGetPanoramaCameraIntriParam
 * c->s: FactoryCalibPanoramaIntriParamReqInfo
 * s->c: FactoryCalibPanoramaIntriParamArray
 */
typedef struct { 
    uint8 panorama_intri_param_req; //周视相机内参请求 0: ignore 1: active
}_STRUCT_ALIGNED_ FactoryCalibPanoramaIntriParamReqInfo;

typedef struct {
    uint8 size;  // 有效相机数
    FactoryCalibIntriParam parameter[FACTORY_CALIB_PANORAMA_CAMERA_NUM];  // 相机内参，排布顺序FactoryCalibPanoramaCameraIndex
}_STRUCT_ALIGNED_ FactoryCalibPanoramaIntriParamArray;

/* 写标定文件到B面（如果存在AB分面）
 * topic: IflytekSensorWriteCalibrationFile
 * 无需定义结构体
 */

#pragma pack()

#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_FACTORY_CALIB_H_