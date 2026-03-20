// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

/*
 * 待4D毫米波电云接口稳定后补充注释
 */

#ifndef _IFLYAUTO_SENSOR_RADAR_H_
#define _IFLYAUTO_SENSOR_RADAR_H_

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

#define SENSOR_RADAR_POINT_CLOUD_MAX_NUN 2048

typedef struct {
    uint16 matched_obj_id;
    float32 rng_m;
    float32 azi_deg;
    float32 ele_deg;
    float32 x;
    float32 y;
    float32 z;

    uint8 motion_status;
    float32 motion_speed;

    float32 dpl_raw_mps;
    float32 dpl_unambi_mps;

    uint8 bandwidth_mode;  // 1：短距；2：长距
    float32 snr_db;
    float32 rcs_dbsm;

    uint8 is_peek;
    uint8 is_ghost;
    uint8 azimuth_order;
} _STRUCT_ALIGNED_ RadarPointCloudUnit;

typedef struct {
    MsgHeader msg_header;
    SensorMeta sensor_meta;
    float32 short_dpl_unambi_scale_mps;
    float32 long_dpl_unambi_scale_mps;
    uint8 is_valid;
    uint16 pointcloud_size;
    RadarPointCloudUnit pointcloud_unit[SENSOR_RADAR_POINT_CLOUD_MAX_NUN];
} _STRUCT_ALIGNED_ RadarPointCloud;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_SENSOR_RADAR_H_
