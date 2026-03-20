// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_CAMERA_PERCEPTION_LANE_LINES_H_
#define _IFLYAUTO_CAMERA_PERCEPTION_LANE_LINES_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#define CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM 8
#define CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM 8
#define CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM 32
#define CAMERA_PERCEPTION_LANE_LINE_SET_SEED_MAX_NUM 16
#define CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM 10
#define CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM 10
#define CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM 10
#define CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM 256
#define CAMERA_PERCEPTION_LANE_MAX_NUM 12
#define CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM 12
#define CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM 32

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

typedef enum {
  SPEED_INFO_SOURCE_TYPE_UNKNOWN = 0,            // 未知
  SPEED_INFO_SOURCE_TYPE_CAMERA_PERCEPTION = 1,  // 相机感知
  SPEED_INFO_SOURCE_TYPE_MAP = 2,                // 地图
} _ENUM_PACKED_ SpeedInfoSourceType;

typedef enum {
  LANE_ASSIGNMENT_THIRD_LEFT = -3,     // 左侧第三条车道
  LANE_ASSIGNMENT_ADJACENT_LEFT = -2,  // 左侧第二条车道
  LANE_ASSIGNMENT_EGO_LEFT = -1,       // 左侧第一条车道
  LANE_ASSIGNMENT_EGO_CENTER = 0,      // 自车当前的车道
  LANE_ASSIGNMENT_EGO_RIGHT = 1,       // 右侧第一条车道
  LANE_ASSIGNMENT_ADJACENT_RIGHT = 2,  // 右侧第二条车道
  LANE_ASSIGNMENT_THIRD_RIGHT = 3,     // 右侧第三条车道
} _ENUM_PACKED_ LaneAssignment;

// 各个车道限速信息
typedef struct {
  float32 speed_limit;             // 限速值       (公里/小时)     <待定，可能改成枚举>
  LaneAssignment lane_assignment;  // 车道位置信息
  SpeedInfoSourceType source;      // 限速信息来源
} _STRUCT_ALIGNED_ SpeedInfo;

typedef struct {
  float64 a0;          // 车道线方程常数项
  float64 a1;          // 车道线方程一次项
  float64 a2;          // 车道线方程二次项
  float64 a3;          // 车道线方程三次项
  float32 begin;       // 起点             (米)
  float32 end;         // 终点             (米)
  float32 confidence;  // 车道线置信度     [0.0-1.0]
} _STRUCT_ALIGNED_ LineSegment;

typedef struct {
  LaneLineColor color;  // 车道线颜色
  float32 begin;        // 起点     (米)
  float32 end;          // 终点     (米)
} _STRUCT_ALIGNED_ ColorSegment;

typedef struct {
  LaneBoundaryType marking;  // 线型
  float32 begin;             // 起点     (米)
  float32 end;               // 终点     (米)
} _STRUCT_ALIGNED_ MarkingSegment;

typedef enum {
  LANE_LINE_SOURCE_TYPE_UNKNOWN = 0,           // 未知的
  LANE_LINE_SOURCE_TYPE_OBSTACLE_BLOCKED = 1,  // 基于拥堵跟车预测，如果被前车近距离遮挡，则会预测
  LANE_LINE_SOURCE_TYPE_HISTORY_BASED = 2,     // 基于历史观测进行预测
  LANE_LINE_SOURCE_TYPE_EXTRAPOLATION = 3,     // 基于车辆运动和车道线情况，即自车行驶状态和车道线趋势一致进行预测
  LANE_LINE_SOURCE_TYPE_NEW = 4,               // 新检测得到的车道线
  LANE_LINE_SOURCE_TYPE_MEASURED = 5,          // 更新出的车道线
  LANE_LINE_SOURCE_TYPE_PREDICTED = 6,         // 预测出的车道线
} _ENUM_PACKED_ LaneLineSourceType;

typedef enum {
  POS_TYPE_CURB_LEFT = -5,      // 左侧道路边沿
  POS_TYPE_FOURTH_LEFT = -4,    // 左侧第四条线
  POS_TYPE_THIRD_LEFT = -3,     // 左侧第三条线
  POS_TYPE_ADJACENT_LEFT = -2,  // 左侧第二条线
  POS_TYPE_EGO_LEFT = -1,       // 左侧第一条线
  POS_TYPE_EGO_CENTER = 0,      // 自车压住的线    <感知可能给不出>
  POS_TYPE_EGO_RIGHT = 1,       // 右侧第一条线
  POS_TYPE_ADJACENT_RIGHT = 2,  // 右侧第二条线
  POS_TYPE_THIRD_RIGHT = 3,     // 右侧第三条线
  POS_TYPE_FOURTH_RIGHT = 4,    // 右侧第四条线
  POS_TYPE_CURB_RIGHT = 5,      // 右侧道路边沿
  POS_TYPE_OTHER = 6,           // 其他
  POS_TYPE_UNKNOWN = 7,         // 未知
} _ENUM_PACKED_ PosType;

// 车道线点的信息
typedef struct {
  LaneBoundaryType  lane_point_marking;           // 车道线点的线型
  LaneLineColor     lane_point_color;             // 车道线点的颜色
  float32           lane_point_confidence;        // 车道线点的置信度  [0.0-1.0]  大于0.5表示该点可见，小于0.5表示该点不可见(被遮挡或超出视线)
  Point2f           lane_point_coordinate;        // 车道线点的坐标    (米)
} _STRUCT_ALIGNED_ LanePointAttr;

// 车道线信息
typedef struct {
  uint32 id;                  // 车道线id
  uint32 life_time;           // 生命周期     (毫秒)
  LaneLineSourceType source;  // 数据来源
  LineType type;              // 车道线类型
  PosType pos_type;           // 车道线位置

  /** 车道线散点数据
   *  单位：米
   *  备注：点的顺序是沿自车前进方向，由近到远
   **/
  uint32 lane_points_attr_set_size;                           // 车道线点的数量
  LanePointAttr lane_points_attr_set[CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM];    // 车道线点的属性信息
  uint8 line_segments_size;                                   // 车道线分段数量
  LineSegment line_segments[CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM];           // 车道线每段信息
  uint8 marking_segments_size;                                // 车道线线型分段数
  MarkingSegment marking_segments[CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM];      // 线型信息
  uint8 color_segments_size;                                  // 车道线颜色分段数
  ColorSegment color_segments[CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM];            // 颜色信息
  float32 confidence;
} _STRUCT_ALIGNED_ LaneLine;

// 车道位置信息
typedef struct {
  float32 begin;             // 起点     (米) 
  float32 end;               // 终点     (米)
  uint32 left_to_right_id;   // 车道id（从左到右，id从1开始）
  uint32 right_to_left_id;   // 车道id（从右到左，id从1开始）
} _STRUCT_ALIGNED_ LanePositionSeg;

// 车道匹配信息
typedef struct {
  float32 begin;                  // 起点     (米) 
  float32 end;                    // 终点     (米)
  uint32 match_current_lane;      // 是否匹配当前车道（0代表不匹配，1代表匹配）
} _STRUCT_ALIGNED_ LaneMatchFlag;

// 车道信息
typedef struct {
  uint32 id;                                             // 车道id
  uint32 order_id;                                       // 车道顺序id: 0,1,2,3,4...
  LaneLine central_line;                                 // 车道中心线
  uint8 lane_types_size;                                 // 车道类型数量                            
  LaneTypeMsg lane_types[CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM];         // 车道类型列表
  uint8 turn_types_size;                                                    // 车道通行方向数量
  LaneMarkMsg turn_types[CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM];         // 车道通行方向列表
  uint8 position_segs_size;                                                 // 车道位置数量
  LanePositionSeg position_segs[CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM];  // 车道位置列表
  uint8 match_flags_size;                                                   // 车道匹配信息数量
  LaneMatchFlag match_flags[CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM];      // 车道匹配信息列表
  uint32 left_lane_boundary_id;                          // 左车道线id
  uint32 right_lane_boundary_id;                         // 右车道线id
  uint32 left_road_boundary_id;                          // 左道路边线id
  uint32 right_road_boundary_id;                         // 右道路边线id
  uint32 stop_line_id;                                   // 绑定的停止线id
  uint8 merge_split_points_size;                                                            // 车道分合流数量
  LaneMergeSplitPointData merge_split_points[CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM];     // 车道分合流信息
} _STRUCT_ALIGNED_ LaneData;

// 车道地面标识信息
typedef struct {
  uint32 ground_marking_points_set_size;                    // 地面标识点集中点的数量
  Point2f ground_marking_points_set[CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM];  // 地面标识点集
  float32 orientation_angle;                                // 地面标识的朝向角度
  LaneDrivableDirection turn_type;                          // 地面标识对应的车道通行方向
  uint32 track_id;                                          // 地面标识跟踪id
} _STRUCT_ALIGNED_ LaneGroundMarking;

// 车道线信息集合
typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint64 isp_timestamp;                                                     // ISP出图时间戳    (微秒)
  uint8 lanes_size;                                                         // 车道数量
  LaneData lanes[CAMERA_PERCEPTION_LANE_MAX_NUM];                           // 车道信息
  uint8 lane_line_size;                                                     // 车道线数量
  LaneLine lane_line[CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM];         // 车道线信息
  uint8 lane_ground_markings_size;                                          // 地面标识数量
  LaneGroundMarking lane_ground_markings[CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM];          // 地面标识信息
  uint8 stop_line_size;                                                     // 停止线数量
  LaneLine stop_line[CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM];             // 停止线信息
  uint8 inhibit_line_size;                                                  // 禁止线数量
  LaneLine inhibit_line[CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM];       // 禁止线信息
  uint8 speed_info_size;                                                    // 车道限速信息数量
  SpeedInfo speed_info[CAMERA_PERCEPTION_LANE_LINE_SET_SEED_MAX_NUM];       // 车道限速信息
  CommonCameraPerceptionInputTimestamp camera_perception_input_timestamp;   // 所有相机的ISP出图时间戳    (微秒)
} _STRUCT_ALIGNED_ LaneLineSet;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_CAMERA_PERCEPTION_LANE_LINES_H_