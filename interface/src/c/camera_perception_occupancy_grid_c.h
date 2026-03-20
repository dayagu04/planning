// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2024/07/03

#ifndef _IFLYAUTO_CAMERA_PERCEPTION_OCCUPANCY_GRID_H_
#define _IFLYAUTO_CAMERA_PERCEPTION_OCCUPANCY_GRID_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#define CAMERA_PERCEPTION_OCC_GRID_MAX_NUM  (300 * 150)
#define CAMERA_PERCEPTION_DRIVABLE_SPACE_GRID_MAX_NUM  (760 * 50)


#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

// occ_grid描述信息
typedef struct {
    float32 grid_resolution_x;                             // x轴分辨率
    float32 grid_resolution_y;                             // y轴分辨率
    float32 grid_resolution_z;                             // z轴分辨率
    uint32 grid_size_positive_x;                           // x轴正方向grid个数
    uint32 grid_size_negative_x;                           // x轴负方向grid个数
    uint32 grid_size_positive_y;                           // y轴正方向grid个数
    uint32 grid_size_negative_y;                           // y轴负方向grid个数
    uint32 grid_size_positive_z;                           // z轴正方向grid个数
    uint32 grid_size_negative_z;                           // z轴负方向grid个数
} _STRUCT_ALIGNED_ CameraPerceptionOccGridMeta;

// 通用occ_grid信息
typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint64 isp_timestamp;                                                            // ISP出图时间戳    (微秒)
  double isp_ego_pose[16];                                                         // ISP时刻自车位姿
  CameraPerceptionOccGridMeta occ_grid_meta;                                       // occ grid描述信息
  uint32 occ_grid_num;                                                             // occ grid个数
  int8 occ_grid[CAMERA_PERCEPTION_OCC_GRID_MAX_NUM];                               // occ grid信息
} _STRUCT_ALIGNED_ CameraPerceptionOccGridInfo;


// 可行驶区域
typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint64 isp_timestamp;                                                            // ISP出图时间戳    (微秒)
  double isp_ego_pose[16];                                                         // ISP时刻自车位姿
  CameraPerceptionOccGridMeta drivable_space_grid_meta;                            // 可行驶区域描述
  uint32 drivable_space_grid_num;                                                  // 可行驶区域grid个数
  int8 drivable_space_grid[CAMERA_PERCEPTION_DRIVABLE_SPACE_GRID_MAX_NUM];         // 可行驶区域grid信息, 输出（0， 1）两类，按位存储
} _STRUCT_ALIGNED_ CameraPerceptionDrivableSpaceGridInfo;


#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_CAMERA_PERCEPTION_OCCUPANCY_GRID_H_