// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_EHR_H_
#define _IFLYAUTO_EHR_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

// #define LANE_POINT_NUM 200
// #define LANE_ID_NUM 10
// #define LANE_OVERLAP_NUM 1
// #define ATTR_POINT_NUM 100
// #define ATTR_TYPE_NUM 5
// #define ATTR_COLOR_NUM 5
// #define BOUNDARY_ATTR_NUM 10
#define EHR_LANE_GROUPS_MAX_NUM 10
#define EHR_LANE_GROUP_ID_MAX_NUM 10
#define EHR_POSITION_MAX_NUM 2
// #define ROAD_LANE_NUM 400
// #define ROAD_LANE_BOUNDARY_NUM 800
// #define ROAD_ROAD_BOUNDARY_NUM 200
// #define ROAD_LANE_GROUP_NUM 100
// #define ROUTE_LANE_GROUP_NUM 50
// #define ORIGIN_DATA_NUM 5
// #define LANE_DATA_NUM 1

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

typedef struct {
  uint32 path_id;  // 0: offroad(未匹配到道路上)  1~2^32-1: 当前匹配的path id
  uint32 offset;   // 当前自车距离初始点的纵向长度
  uint32 accuracy;
  uint32 lateral_accuracy;
  uint32 longitudinal_accuracy;
  float64 original_lon;  // 原始经纬度,GCJ02格式
  float64 original_lat;
  int32 deviation;           // cm
  float32 vehicle_speed;     // m/s
  float32 relative_heading;  // °度
  float32 probability;       // %
  uint32 current_lane;
  uint32 preferred_path;
} _STRUCT_ALIGNED_ PostionData;

typedef struct {
  uint64 road_id;   // 定位到高精地图里的道路ID，“0"示缺少高精地图，无法输出ID
  uint64 lane_id;   // 定位到高精地图里的车道ID，“0"示缺少高精地图，无法输出ID
  uint32 lane_seq;  // 从右至左分别是"1、2、3…",“0”表示缺少高精地图，无法输出车道的序号
  uint32 dis_left;  // 车辆原点距离左车道线的距离（车辆原点为后轴中心）     (厘米)
  uint32 dis_right;       // 车辆原点距离右车道线的距离（车辆原点为后轴中心）     (厘米)
  float32 head_left;      // (弧度)
  float32 head_right;     // (弧度)
  uint32 confi_lane_loc;  // 车道级定位置信度
} _STRUCT_ALIGNED_ RelativePostion;

typedef struct {
  uint64 original_loc_timestamp;  // 定位信号原始时间戳   (微秒)
  float64 lon;                    // 绝对位置经度,GCJ02格式
  float64 lat;                    // 绝对位置纬度,GCJ02格式
  float32 heading;     // 绝对航向,以正北方向为开始边，顺时针为正，旋转到车辆姿态正前
  float32 velocity_x;  // x方向绝度速度        (米/秒)
  float32 velocity_y;  // y方向绝度速度        (米/秒)
  float32 velocity_z;  // z方向绝度速度        (米/秒)
  float32 x_acc;       // x方向绝度加速度      (米/秒)
  float32 y_acc;       // y方向绝度加速度      (米/秒)
  float32 z_acc;       // z方向绝度加速度      (米/秒)
  float32 angular_velocity_x;  // 横滚角速度           (弧度/秒)
  float32 angular_velocity_y;  // 俯仰角速度           (弧度/秒)
  float32 angular_velocity_z;  // 横摆角速度           (弧度/秒)
} _STRUCT_ALIGNED_ AbsolutePostion;

typedef enum {
  POSITION_WARING_UNKNOWN = 0,
  POSITION_WARING_OUT_AREA = 1,
  POSITION_WARING_RIGHT = 2,
  POSITION_WARING_TUNNEL = 3,
  POSITION_WARING_TOLL_STATION = 4,
  POSITION_WARING_CONSTRUCTION = 5,
  POSITION_WARING_TRAFFICSIGN = 6,
  POSITION_WARING_WITHOUT_GUARDRAIL = 7,
  POSITION_WARING_CUTED_ROAD = 8,
  POSITION_WARING_ALL_ROAD_CHANGED = 9,
  POSITION_WARING_CURVATURE_OUT_LIMIT = 10,
  POSITION_WARING_SLOPE_OUT_LIMIT = 11,
  POSITION_WARING_RMAP_OR_JCT = 12,
  POSITION_WARING_EXTINCTION_OR_RENEWAL_LANE = 13,
  POSITION_WARING_LANE_WIDTH = 14,
} _ENUM_PACKED_ PositionWarningType;

typedef struct {
  int32 warning_judge_status;
  /** 报警类型
   *  1: 不确定
   *  2: 非区域内(百度原始信号)
   *  3: 隧道
   *  4: 收费站
   *  5: 施工
   *  6: 交通信号灯
   *  7: 道路无护栏
   *  8: 断头路
   *  9: 所有车道发生变化(百度原始信号)
   *  10: 曲率超限
   *  11: 坡度超限
   *  12: 匝道或JCT
   *  13: 消亡/新生车道
   *  14: 车道宽度
   **/
  uint32 warning_judge_type;
  uint32 warning_bounding_distance;
} _STRUCT_ALIGNED_ PositionWarning;

typedef enum {
  GEO_FENCE_INVALID = 0,
  GEO_FENCE_INDETERMINACY = 1,
  GEO_FENCE_RESERVED = 2,
  GEO_FENCE_EMERGENCY = 3,
  GEO_FENCE_TOLL_STATION = 4,
  GEO_FENCE_CONSTRUCTION = 5,
  GEO_FENCE_TRAFFICSIGN = 6,
  GEO_FENCE_CUTED_ROAD = 7,
} _ENUM_PACKED_ PositionGeofenceType;

typedef struct {
  int32 geofence_judge_status;
  /** 地理围栏类型
   *  0:Invalid
   *  1:不确定
   *  2:预留字段
   *  3:应急车道
   *  4:隧道??
   *  5:收费站
   *  6:施工
   *  7:交通信号灯
   *  8:断头路
   **/
  uint32 geofence_judge_type;
  uint32 geofence_bounding_distance;
} _STRUCT_ALIGNED_ PositionGeofence;

typedef enum {
  LOCATION_AVALIABLE = 0,
  LOCATION_CACULATING = 1,
  LOCATION_FAILED = 2,
} _ENUM_PACKED_ FailLocStatus;

typedef enum {
  GNSS_DATA_LOST = 0,
  GNSS_RESERVED = 1,
  GNSS_FREQ_ERR = 2,
  GNSS_DATA_INVALID = 3,
} _ENUM_PACKED_ FailGnssStatus;

typedef enum {
  CAMERA_DATA_LOST = 0,
  CAMERA_RESERVED = 1,
  CAMERA_FREQ_ERR = 2,
} _ENUM_PACKED_ FailCameraStatus;

typedef enum {
  HDMAP_NOLINK = 0,
  HDMAP_RESERVED = 1,
  HDMAP_LICENSE_FAILED = 2,
} _ENUM_PACKED_ FailHdmapStatus;

typedef enum {
  VEHICLE_DATA_LOST = 0,
  VEHICLE_RESERVED = 1,
  VEHICLE_FREQ_ERR = 2,
} _ENUM_PACKED_ FailVehicleStatus;

typedef enum {
  IMU_DATA_LOST = 0,
  IMU_RESERVED = 1,
  IMU_FREQ_ERR = 2,
  IMU_ACC_FAIL = 3,
  IMU_ANG_FAIL = 4,
  IMU_CALIB_FAIL = 5,
  IMU_TIME_ERR = 6,
} _ENUM_PACKED_ FailImuStatus;

typedef struct {
  int32 failsafe_loc_status;
  int32 failsafe_gnss_status;
  int32 failsafe_camera_status;
  int32 failsafe_hdmap_status;
  int32 failsafe_vehicle_status;
  int32 failsafe_imu_status;
} _STRUCT_ALIGNED_ PostionFailSafe;

typedef struct {
  uint64 lane_id;
  uint8 entry_lane_ids_size;
  uint64 entry_lane_ids[EHR_LANE_GROUPS_MAX_NUM];
  uint8 exit_lane_ids_size;
  uint64 exit_lane_ids[EHR_LANE_GROUPS_MAX_NUM];
} _STRUCT_ALIGNED_ LaneInRoute;

typedef struct {
  uint64 lane_group_id;
  float64 length;  // 可以由该lane group包含的所有的lane长度的平均值计算得到
  uint8 lanes_in_route_size;
  LaneInRoute lanes_in_route[EHR_LANE_GROUP_ID_MAX_NUM];
  Point3d start_point;  // reserve,暂时不用
  Point3d end_point;    // reserve,暂时不用
} _STRUCT_ALIGNED_ LaneGroupInRoute;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint64 position_age;
  uint8 positions_size;
  PostionData positions[EHR_POSITION_MAX_NUM];
  uint8 relative_pos_size;
  RelativePostion relative_pos[EHR_POSITION_MAX_NUM];
  uint8 absolute_pos_size;
  AbsolutePostion absolute_pos[EHR_POSITION_MAX_NUM];
  uint8 position_warning_size;
  PositionWarning position_warning[EHR_POSITION_MAX_NUM];
  uint8 position_geofence_size;
  PositionGeofence position_geofence[EHR_POSITION_MAX_NUM];
  uint8 fail_safe_size;
  PostionFailSafe fail_safe[EHR_POSITION_MAX_NUM];
} _STRUCT_ALIGNED_ Position;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_EHR_H_
