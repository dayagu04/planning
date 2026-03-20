// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_SENSOR_IMAGE_H_
#define _IFLYAUTO_SENSOR_IMAGE_H_

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

#define CAMERA_FRAME_ID_STR_LEN 8
#define CAMERA_FORMAT_STR_LEN 8
#define CAMERA_VIEW_ANGLE_STR_LEN 8
#define CAMERA_RESOLUTION_STR_LEN 16

// 基础相机结构
typedef struct {
  MsgHeader msg_header;                                 // SOC消息发送信息
  SensorMeta sensor_meta;                               // ISP曝光信息
  uint8 frame_id[CAMERA_FRAME_ID_STR_LEN];              // 图片帧数
  uint8 format[CAMERA_FORMAT_STR_LEN];                  // 图片的编码格式 nv12 jpeg h264
  uint8 camera_view_angle[CAMERA_VIEW_ANGLE_STR_LEN];   // 相机视角
  uint8 camera_resolution[CAMERA_RESOLUTION_STR_LEN];   // 相机分辨率信息
  uint32 height;                                        // 图片的高
  uint32 width;                                         // 图片的宽
  int32 dmabuf_fd_index;                                // 图片在dma ringbuffer里的序号
} _STRUCT_ALIGNED_ Camera_Image_Info;

typedef struct {
  Camera_Image_Info front_camera_data;  // 前视相机数据
} _STRUCT_ALIGNED_ Front_Camera_Image_Info;

typedef struct {
  Camera_Image_Info right_camera_data;  // 右环视视相机数据
  Camera_Image_Info front_camera_data;  // 前环视视相机数据
  Camera_Image_Info left_camera_data;   // 左环视视相机数据
  Camera_Image_Info rear_camera_data;   // 后环视视相机数据
} _STRUCT_ALIGNED_ Surround_Camera_Image_Info;

typedef struct {
  Camera_Image_Info front120_camera_data;  // 前视120度相机数据
  Camera_Image_Info rear_camera_data;      // 后视相机数据
  Camera_Image_Info side_fl_camera_data;   // 侧视前左相机数据
  Camera_Image_Info side_fr_camera_data;   // 侧视前右相机数据
  Camera_Image_Info side_rl_camera_data;   // 侧视后左相机数据
  Camera_Image_Info side_rr_camera_data;   // 侧视后右相机数据
} _STRUCT_ALIGNED_ Panorama_Camera_Image_Info;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_SENSOR_IMAGE_H_