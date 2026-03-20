// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_IFLY_LOCALIZATION_H_
#define _IFLYAUTO_IFLY_LOCALIZATION_H_

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

#define LOCALIZATION_RESERVED_DATA_SIZE 16

typedef struct {
  uint32 available;                          // 预留数据标志
  uint64 reserved_data[LOCALIZATION_RESERVED_DATA_SIZE];  // 存储预留数据
} _STRUCT_ALIGNED_ IFLYReserved;

typedef struct {
  uint64 id;         // 定位消息id
  uint64 timestamp;  // pose时间戳(微秒)
} _STRUCT_ALIGNED_ IFLYLocalizationMeta;

// 加速度(ENU坐标系)，单位(g)
typedef struct {
  float64 ae;  // 东向(East)方向上的加速度
  float64 an;  // 北向(North)方向上的加速度
  float64 au;  // 天向(Up)方向上的加速度
} _STRUCT_ALIGNED_ IFLYAccGlobal;

// 加速度(开机坐标系)，单位(g)
typedef struct {
  float64 ax;  // x 轴方向上的加速度
  float64 ay;  // y 轴方向上的加速度
  float64 az;  // z 轴方向上的加速度
} _STRUCT_ALIGNED_ IFLYAccLocal;

// 加速度(ENU坐标系，开机坐标系)
typedef struct {
  IFLYAccGlobal acceleration_enu;  // ENU坐标系下的加速度(不包含重力分量)
  IFLYAccLocal acceleration_boot;  // 开机坐标系下的加速度(不包含重力分量)
  IFLYAccLocal acceleration_body;  // body坐标系下的加速度(不包含重力分量)
} _STRUCT_ALIGNED_ IFLYAcceleration;

// 角速度(开机坐标系)，单位(rad/s)
typedef struct {
  float64 vx;  // x 轴方向上的角速度
  float64 vy;  // y 轴方向上的角速度
  float64 vz;  // z 轴方向上的角速度
} _STRUCT_ALIGNED_ IFLYAngVelLocal;

// 角速度(开机坐标系)
typedef struct {
  IFLYAngVelLocal angvelocity_boot;  // 开机坐标系下的角速度    (rad/s)
  IFLYAngVelLocal angvelocity_body;  // body坐标系下的角速度    (rad/s)
} _STRUCT_ALIGNED_ IFLYAngularVelocity;

// 速度(ENU坐标系)，单位(m/s)
typedef struct {
  boolean available; // 是否可用
  float64 ve;      // 东向(East)方向上的速度
  float64 vn;      // 北向(North)方向上的速度
  float64 vu;      // 天向(Up)方向上的速度
} _STRUCT_ALIGNED_ IFLYVelGlobal;

// 速度(开机坐标系)，单位(m/s)
typedef struct {
  float64 vx;  // x 轴方向上的速度
  float64 vy;  // y 轴方向上的速度
  float64 vz;  // z 轴方向上的速度
} _STRUCT_ALIGNED_ IFLYVelLocal;

// 速度(ENU坐标系，开机坐标系)
typedef struct {
  IFLYVelGlobal velocity_enu;  // ENU坐标系下的速度信息
  IFLYVelLocal velocity_boot;  // 开机坐标系下的速度信息
  IFLYVelLocal velocity_body;  // body坐标系下的速度信息
} _STRUCT_ALIGNED_ IFLYVelocity;

/** 旋转欧拉角
 *  单位：弧度
 *  备注：ZYX，表示先绕Z轴旋转(yaw)，然后绕Y轴旋转(pitch)，最后绕X轴旋转(roll)
 */
typedef struct {
  float64 yaw;    // 绕Z轴的旋转角度     (弧度)    [-pi, pi)
  float64 pitch;  // 绕Y轴的旋转角度     (弧度)    [-pi/2, pi/2]
  float64 roll;   // 绕X轴的旋转角度     (弧度)    [-pi, pi)
} _STRUCT_ALIGNED_ IFLYEuler;

// 旋转四元数
typedef struct {
  float64 w;  // 四元数的实部，标量部分
  float64 x;  // 四元数的虚部 i 分量
  float64 y;  // 四元数的虚部 j 分量
  float64 z;  // 四元数的虚部 k 分量
} _STRUCT_ALIGNED_ IFLYQuaternion;

// 旋转(欧拉角，四元数)(ENU坐标系，开机坐标系)
typedef struct {
  IFLYEuler euler_enu;             // ENU坐标系下的欧拉角
  IFLYQuaternion quaternion_enu;   // ENU坐标系下的四元数
  IFLYEuler euler_boot;            // 开机坐标系下的欧拉角
  IFLYQuaternion quaternion_boot;  // 开机坐标系下的四元数
} _STRUCT_ALIGNED_ IFLYOrientation;

// 位置(LLH坐标系)
typedef struct {
  boolean available;    // 是否可用
  LLHType type;       // 坐标系类型
  float64 longitude;  // 经度     (度)    [-180°,180°)
  float64 latitude;   // 纬度     (度)    [-90,90]
  float64 height;     // 高度     (米)
} _STRUCT_ALIGNED_ IFLYPosGlobal;

// 位置(开机坐标系)，单位(米)
typedef struct {
  boolean available; // 是否可用 
  float64 x;       // x 轴方向上的位置
  float64 y;       // y 轴方向上的位置
  float64 z;       // z 轴方向上的位置
} _STRUCT_ALIGNED_ IFLYPosLocal;

typedef struct {
  boolean available; // 是否可用 
  float64 e;       // e 轴方向上的位置
  float64 n;       // n 轴方向上的位置
  float64 u;       // u 轴方向上的位置
} _STRUCT_ALIGNED_ IFLYPosEnu;

// 位置(LLH坐标系，开机坐标系)
typedef struct {
  IFLYPosGlobal position_llh;  // LLH坐标系下的位置信息
  IFLYPosLocal position_boot;  // 开机坐标系下的位置信息
  IFLYPosEnu position_enu;     // body坐标系下的位置信息
} _STRUCT_ALIGNED_ IFLYPosition;

/*用法：
 * transform_llh_to_boot:
 *   --1.point_llh -> point_enu
 * 利用参考点transform_center的llh坐标作为原点，将要转化的点的llh坐标转换为ecef坐标，再将ecef坐标转换为enu坐标
 *     具体可参考: http://wiki.gis.com/wiki/index.php/Geodetic_system
 *   --2.point_boot = transform_q * point_enu + transform_t
 * transform_ego_motion_to_boot:
 *    point_boot = transform_q * point_origin + transform_t
 */
typedef struct {
  boolean available;              // 该变换是否有效。当GNSS信号缺失或者地图缺失时，该标志位为false
  IFLYQuaternion transform_q;  // 变换的旋转部分(四元数)
                               // transform_llh_to_boot：enu -> boot的旋转
                               // transform_ego_motion_to_boot：ego_motion -> boot的旋转
  IFLYPosLocal transform_t;    // 变换的平移部分(开机坐标系)
                               // transform_llh_to_boot：enu -> boot 的平移
                               // transform_ego_motion_to_boot：ego_motion -> boot 的平移
  IFLYPosGlobal
      transform_center;  // 变换的参考点(LLH坐标系)
                         // 仅用于transform_llh_to_boot中，利用参考点作为原点，将LLH坐标系转换为ENU坐标系(llh->ecef->enu)，再利用transform_q和transform_t转换到boot坐标系
} _STRUCT_ALIGNED_ IFLYTransformInfo;

typedef struct {
  IFLYTransformInfo transform_llh_to_boot;         // LLH坐标系-开机坐标系的转换
  IFLYTransformInfo transform_ego_motion_to_boot;  // egomotion坐标系-开机坐标系的转换
} _STRUCT_ALIGNED_ IFLYTransform;

typedef enum {
  IFLY_STATUS_INFO_MODE_ERROR = 0,  // 0:错误，位姿无效
  IFLY_STATUS_INFO_MODE_EGOMOTION,  // 1:egomotion模式，无地图定位
  IFLY_STATUS_INFO_MODE_MAPLOC,     // 2:地图定位模式，高精地图可用
} _ENUM_PACKED_ IFLYStatusInfoMode;

typedef struct {
  /** 当前车辆运动模式的标识符(下游只需要判断该字段) **/
  IFLYStatusInfoMode mode;
  uint64 common;    // 通用定位状态的标识符(主要供定位使用)
  uint64 extended;  // 扩展状态的标识符(主要供定位使用)
} _STRUCT_ALIGNED_ IFLYStatusInfo;

typedef struct {
  IFLYStatusInfo status_info;  // 定位状态信息
} _STRUCT_ALIGNED_ IFLYStatus;

typedef enum {
  IFLY_POSE_DETAIL_INFO_ESTIMATION_UNKNOWN,     // 未知的位姿估计类型
  IFLY_POSE_DETAIL_INFO_ESTIMATION_PER_FRAME,   // 每帧位姿估计
  IFLY_POSE_DETAIL_INFO_ESTIMATION_WINDOW_OPT,  // 窗口优化位姿估计
  IFLY_POSE_DETAIL_INFO_ESTIMATION_DR,          // DR(惯导)位姿估计
} _ENUM_PACKED_ IFLYPoseDetailInfoType;

typedef struct {
  IFLYPoseDetailInfoType type;  // 详细位姿信息标志
  boolean is_keyframe;          // 是否为关键帧的标志
  boolean has_scale;            // 是否包含尺度信息的标志
  uint32 pose_type;             // 位姿估计的类型
  int32 map_floor_id;           // 地图定位楼层，未确定时为-1
} _STRUCT_ALIGNED_ IFLYPoseDetailInfo;

typedef struct {
  IFLYPoseDetailInfo pose_detail_info;  // 详细位姿信息
} _STRUCT_ALIGNED_ IFLYPoseDetail;

typedef enum {
    IFLY_LOCALIZATION_NO_EXCEPTION = 0,           // 无异常
    IFLY_LOCALIZATION_LOCAL_FILTER_ABNORMAL = 1,  // 内部滤波器状态异常
    IFLY_LOCALIZATION_MAP_FILTER_ABNORMAL   = 2,  // 地图定位滤波器异常
    IFLY_LOCALIZATION_GNSS_FILTER_ABNORMAL  = 4,  // GNSS滤波器异常
} _ENUM_PACKED_ IFLYLocalizationExceptionType;

typedef struct {
  IFLYLocalizationExceptionType type;
  uint64 reserved_data[LOCALIZATION_RESERVED_DATA_SIZE];
} _STRUCT_ALIGNED_ IFLYLocalizationException;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  IFLYReserved reserved;                 // 保留数据
  IFLYLocalizationMeta meta;             // 定位数据
  IFLYAcceleration acceleration;         // 加速度(ENU坐标系，开机坐标系, body坐标系)
  IFLYAngularVelocity angular_velocity;  // 角速度(开机坐标系，body坐标系)
  IFLYVelocity velocity;                 // 速度(ENU坐标系，开机坐标系，body坐标系)
  IFLYOrientation orientation;           // 旋转(欧拉角，四元数)(ENU坐标系，开机坐标系)
  IFLYPosition position;                 // 位置(LLH坐标系，开机坐标系，body坐标系)
  IFLYTransform transform;               // 变换
  IFLYStatus status;                     // 定位状态
  IFLYPoseDetail pose_detail;            // 详细位姿
  IFLYLocalizationException exception;   // 定位异常结构体
} _STRUCT_ALIGNED_ IFLYLocalization;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_IFLY_LOCALIZATION_H_
