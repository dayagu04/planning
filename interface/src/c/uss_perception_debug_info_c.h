// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_USS_PERCEPTION_DEBUG_INFO_H_
#define _IFLYAUTO_USS_PERCEPTION_DEBUG_INFO_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#define USS_PERCEPTION_DEBUG_BORDER_TYPE_NUM 4
#define USS_PERCEPTION_DEBUG_CORNER_POINT_NUM 4
#define USS_PERCEPTION_DEBUG_CORNER_SENSOR_NUM 4
#define USS_PERCEPTION_DEBUG_USS_SLOT_MAX_NUM 6
#define USS_PERCEPTION_DEBUG_SLOT_PARAM_EACH_SIDE_NUM 6
#define USS_PERCEPTION_DEBUG_SLOT_SIDE_NUM 2
#define USS_PERCEPTION_DEBUG_CORNER_OBJ_MAX_NUM 200

#pragma pack(4)

typedef struct {
  uint32 id;                                  // 库位id
  float32 sita;                               // 车位角度(度)
  ParkingSlotPositionType slot_side;          // 寻库方向
  uint32 border_type[USS_PERCEPTION_DEBUG_BORDER_TYPE_NUM];     // 车位边界类型(0:真实探测到的点 / 1:虚拟的点)
  Point2d corner_point[USS_PERCEPTION_DEBUG_CORNER_POINT_NUM];  // 四个角点坐标<固定4个>
  ParkingSlotType slot_type;                  // 车位类型
} _STRUCT_ALIGNED_ AusspsUss;

typedef struct {
  uint32 timestamp;                          // 时间戳
  uint8 parking_slot_size;                   // 库位数量 uint8
  AusspsUss parking_slots[USS_PERCEPTION_DEBUG_USS_SLOT_MAX_NUM]; // 库位信息 AUSSPS_USS[6]
} _STRUCT_ALIGNED_ AusspsListUss;

typedef struct {
  ParkingSlotType slot_type;
  ParkingSlotType slot_type_confirm;
  int32 slot_start_index;                           // int16 车位起点索引
  int32 slot_end_index;                             // int16 车位终点索引
  int32 slot_cal_dis_car_to_obj1;                   // int16
  int32 slot_cal_dis_car_to_obj2;                   // int16
  int32 slot_length_detected1;                      // int16
  int32 slot_length_detected2;                      // int16
  int32 slot_length_detected3;                      // int16
  int32 slot_length_total;                          // int16
  int32 slot_depth_detected1;                       // int16
  int32 slot_depth_detected2;                       // int16
  int32 slot_depth_detected3;                       // int16
  int32 slot_length;                                // int16 车位长度
  int32 slot_depth_parallel;                        // int16
  int32 slot_depth;                                 // int16 车位深度
  int32 sub_slot_depth;                             // int16
  int32 dis_car_to_obj1_by_passing_slot;            // int16
  int32 dis_car_to_obj2_by_passing_slot;            // int16
  int32 car_pass_the_slot_end_pt_distance;          // int16
  int32 car_pass_the_slot_start_pt_distance;        // int16
  int32 apa_slot_detection_compensate_length_head;  // int16 补偿值
  int32 apa_slot_detection_compensate_length_tail;  // int16 补偿值
  int32 obj1_width;                                 // int16 障碍物1宽度
  int32 obj2_width;                                 // int16 障碍物2宽度
  int32 obj1_width_confirm;                         // int16
  int32 obj2_width_confirm;                         // int16
  int32 obj1_type;                                  // int16 障碍物1类型
  int32 obj2_type;                                  // int16 障碍物2类型
  int32 obj1_start_pt_index;                        // int16 障碍物1起始索引
  int32 obj1_end_pt_index;                          // int16 障碍物1终点索引
  int32 obj2_start_pt_index;                        // int16 障碍物2起始索引
  int32 obj2_end_pt_index;                          // int16 障碍物2终点索引
  int32 slot_id;                                    // int8  车位ID
  uint32 curb_exist;                                // uint8
  uint32 slot_proc_slot_obj1_exist;                 // uint8
  uint32 slot_proc_slot_obj2_exist;                 // uint8
} _STRUCT_ALIGNED_ SlotParameterType;

typedef struct {
  uint32 slot_num;                                          // uint8
  uint32 slot_confirm_seq;                                  // uint8
  uint32 slot_prev_id;                                      // uint16
  SlotParameterType slot_par[USS_PERCEPTION_DEBUG_SLOT_PARAM_EACH_SIDE_NUM];  // SlotParameterType[3]
} _STRUCT_ALIGNED_ SlotInfoDataType;

typedef struct {
  uint32 wr_index;                                // 数组写索引
  uint32 obj_pt_cnt;                              // 障碍物数量
  Point2d obj_pt[USS_PERCEPTION_DEBUG_CORNER_OBJ_MAX_NUM];              // 障碍物坐标 (毫米)   <最大200个>
  uint32 dis_from_car_to_obj[USS_PERCEPTION_DEBUG_CORNER_OBJ_MAX_NUM];  // 障碍物到车身的距离 (毫米)   <最大200个>
} _STRUCT_ALIGNED_ CornerPoints;

typedef struct {
  MsgHeader msg_header;                               // SOC消息发送信息
  SensorMeta sensor_meta;                             // CAN报文到达MCU信息
  uint32 parking_slot_num;                            // 超声波车位数量
  AusspsListUss uss_slot_list;                        // 超声波车位信息
  SlotInfoDataType uss_slot_param[USS_PERCEPTION_DEBUG_SLOT_SIDE_NUM];  // 车位参数 2
  CornerPoints corner_points[USS_PERCEPTION_DEBUG_CORNER_SENSOR_NUM];   // 超声波角雷达障碍物点云信息 4
} _STRUCT_ALIGNED_ UssPerceptDebugInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_USS_PERCEPTION_INFO_H_