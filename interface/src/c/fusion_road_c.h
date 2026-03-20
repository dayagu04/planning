// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_FUSION_ROAD_H_
#define _IFLYAUTO_FUSION_ROAD_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#define FUSION_ROAD_LINE_POLYNOMIAL_NUM 4
#define FUSION_ROAD_REFLINE_POINT_MAX_NUM 125
#define FUSION_ROAD_LANE_BOUNDARY_MAX_NUM 10
#define FUSION_ROAD_LANE_SPLIT_POINT_MAX_NUM 4
#define FUSION_ROAD_REFLANE_MSG_MAX_NUM 8
#define FUSION_ROAD_REFLINE_MAX_NUM 7
#define FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM 100
#define FUSION_ROAD_LANE_GROUND_MARKINGS_MAX_NUM 16
#define FUSION_ROAD_POLYLINE_MAX_NUM 32
#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

typedef enum {
  LaneSource_SOURCE_UNKNOWN = 0,                    // 未知来源
  LaneSource_SOURCE_CAMERA = 1,                     // 仅相机
  LaneSource_SOURCE_HDMAP = 2,                      // 仅高精地图
  LaneSource_SOURCE_CAMERA_HDMAP_FUSION = 3,        // 相机和高精地图融合
  LaneSource_SOURCE_PARKING_MAP = 4,                // 仅自建地图
  LaneSource_SOURCE_CAMERA_PARKING_MAP_FUSION = 5,  // 相机和自建地图融合
} _ENUM_PACKED_ LaneSource;

typedef struct {
  uint64 track_id;                          // 该点处lane_id
  Point2d car_point;                        // 参考点在自车坐标系下坐标     (米)
  Point3d enu_point;                        // 参考点在ENU坐标系下坐标      (米)
  Point3d local_point;                      // 参考点在局部坐标系下坐标     (米)
  float32 curvature;                        // 参考点处曲率
  float32 car_heading;                      // 参考点处自车坐标系下航向角   (弧度rad)
  float32 enu_heading;                      // 参考点处ENU坐标系下航向角    (弧度rad)
  float32 local_heading;                    // 参考点处LOCAL坐标系下航向角  (弧度rad)
  float32 distance_to_left_road_border;     // 参考点到道路左边缘距离       (米)
  float32 distance_to_right_road_border;    // 参考点到道路右边缘距离       (米)
  float32 distance_to_left_lane_border;     // 参考点到车道左车道线距离     (米)
  float32 distance_to_right_lane_border;    // 参考点到车道右车道线距离     (米)
  float32 lane_width;                       // 参考点处车道宽度             (米)
  float32 speed_limit_max;                  // 参考点处最高限速             (米/秒)
  float32 speed_limit_min;                  // 参考点处最低限速             (米/秒)
  LaneBoundaryType left_road_border_type;   // 参考点处左道路边缘类型
  LaneBoundaryType right_road_border_type;  // 参考点处右道路边缘类型
  LaneBoundaryType left_lane_border_type;   // 参考点处左车道线类型
  LaneBoundaryType right_lane_border_type;  // 参考点处右车道线类型
  boolean is_in_intersection;               // 参考点是否在路口 (true:路口 // false:不在路口)
  LaneType lane_type;                       // 参考点处车道类型
  float64 s;                                // 参考点距离起点的累计距离
  float32 confidence;                       // 参考点置信度   [0.0-1.0]
} _STRUCT_ALIGNED_ ReferencePoint;

typedef struct {
  uint8 virtual_lane_refline_points_size;
  ReferencePoint virtual_lane_refline_points[FUSION_ROAD_REFLINE_POINT_MAX_NUM];  // 车道中心线的参考点
  /** 自车坐标系下中心线方程系数
   *  备注:固定4个，依次为常数项、一次项、二次项、三次项
   **/
  float64 poly_coefficient_car[FUSION_ROAD_LINE_POLYNOMIAL_NUM];
} _STRUCT_ALIGNED_ LaneReferenceLine;

typedef struct {
  float64 a0;          // 车道线方程常数项
  float64 a1;          // 车道线方程一次项
  float64 a2;          // 车道线方程二次项
  float64 a3;          // 车道线方程三次项
  float32 begin;       // 起点（以自车投影点为0点）             (米)
  float32 end;         // 终点（以自车投影点为0点）             (米)
  float32 confidence;  // 车道线置信度     [0.0-1.0]
} _STRUCT_ALIGNED_ FusionLineSegment;

typedef struct {
  float32 length;       // 该颜色剩余车道线长度     (米)
  float32 begin;        // 起点（以自车投影点为0点）                   （米）
  float32 end;          // 终点（以自车投影点为0点）                   （米）
  LaneLineColor color;  // 车道线类型
} _STRUCT_ALIGNED_ LaneBoundaryColorSegment;

typedef struct {
  float32 length;         // 该颜色剩余车道线长度     (米)
  float32 begin;          // 起点（以自车投影点为0点）                   （米）
  float32 end;            // 终点（以自车投影点为0点）                   （米）
  LaneBoundaryType type;  // 车道线类型
} _STRUCT_ALIGNED_ LaneBoundaryTypeSegment;

typedef struct {
  boolean existence;  // 数据有效性           (true:valid / false:invalid)
  uint32 life_time;   // 生命周期
  uint64 track_id;    // 车道id
  LineType type;      // 车道线类型

  /** 车道线方程系数
   *  备注:固定4个，依次为常数项、一次项、二次项、三次项
   **/
  float64 poly_coefficient[FUSION_ROAD_LINE_POLYNOMIAL_NUM];
  float32 begin;                                                       // 车道线起始点         (米)
  float32 end;                                                         // 车道线终止点         (米)
  uint8 line_segments_size;                                            // 车道线分段数量
  FusionLineSegment line_segments[FUSION_ROAD_LANE_BOUNDARY_MAX_NUM];  // 车道线每段信息
  uint8 type_segments_size;
  LaneBoundaryTypeSegment type_segments[FUSION_ROAD_LANE_BOUNDARY_MAX_NUM];  // 车道线类型分段信息
  uint8 color_segments_size;
  LaneBoundaryColorSegment color_segments[FUSION_ROAD_LANE_BOUNDARY_MAX_NUM];  // 车道线颜色分段信息
  uint8 car_points_size;
  Point2f car_points[FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM];  // 自车坐标系车道线点集  (米)
  uint8 enu_points_size;
  Point3f enu_points[FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM];  // ENU坐标系下车道线点集 (米)
  uint8 local_points_size;
  Point3f local_point[FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM];  // local坐标系下车道线点集 (米)
  float32
      point_confidence_list[FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM];  // 车道线点的置信度，数量和car_points_size相同
  float32 line_width;                                                  // 车道线宽度
} _STRUCT_ALIGNED_ LaneBoundary;

typedef struct {
  boolean existence;  // 数据有效性   (true:valid / false:invalid)
  uint8 merge_split_point_data_size;
  LaneMergeSplitPointData merge_split_point_data[FUSION_ROAD_LANE_SPLIT_POINT_MAX_NUM];  // 分流点信息
} _STRUCT_ALIGNED_ LaneMergeSplitPoint;

typedef struct {
  float64 begin;      // 起点（以自车投影点为0点）                   （米）
  float64 end;        // 终点（以自车投影点为0点）                   （米）
  LaneSource source;  // 数据源
} _STRUCT_ALIGNED_ LaneSourceMsg;

typedef struct {
  float64 begin;          // 起点（以自车投影点为0点）                   （米）
  float64 end;            // 终点（以自车投影点为0点）                   （米）
  uint32 left_lane_num;   // 左侧车道数量
  uint32 right_lane_num;  // 右侧车道数量
} _STRUCT_ALIGNED_ LaneNumMsg;

typedef struct {
  /** 该车道所属的顺序id
   *  备注:从左到右顺序，最左车道ID为0
   **/
  int32 order_id;
  /** 该车道的相对id
   *  备注:当前车道id为0，向左依次-1，向右依次+1
   **/
  int32 relative_id;
  uint8 lane_types_size;
  LaneTypeMsg lane_types[FUSION_ROAD_REFLANE_MSG_MAX_NUM];  // 车道类型
  uint8 lane_marks_size;
  LaneMarkMsg lane_marks[FUSION_ROAD_REFLANE_MSG_MAX_NUM];  // 该车道可行驶方向
  uint8 lane_sources_size;
  LaneSourceMsg lane_sources[FUSION_ROAD_REFLANE_MSG_MAX_NUM];  // 车道信息来源
  uint8 lane_num_size;
  LaneNumMsg lane_num[FUSION_ROAD_REFLANE_MSG_MAX_NUM];  // 车道数量
  LaneReferenceLine lane_reference_line;                 // 该车道绑定的中心线信息
  LaneMergeSplitPoint lane_merge_split_point;            // 该车道绑定的分合流信息
  LaneBoundary left_lane_boundary;                       // 该车道的左车道线
  LaneBoundary right_lane_boundary;                      // 该车道的右车道线
  LaneBoundary stop_line;                                // 该车道绑定的停止线
} _STRUCT_ALIGNED_ ReferenceLineMsg;

typedef struct {
  uint32 ground_marking_points_set_size;                                             // 地面标识车体坐标系点集中点的数量
  Point2f ground_marking_points_set[FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM];        // 地面标识车体坐标系点集
  uint32 local_ground_marking_points_set_size;                                       // 地面标识开机坐标系点集中点的数量
  Point2f local_ground_marking_points_set[FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM];  // 地面标识开机坐标系点集
  float32 orientation_angle;                                                         // 地面标识的车体坐标系朝向角度
  LaneDrivableDirection turn_type;                                                   // 地面标识对应的车道通行方向
} _STRUCT_ALIGNED_ FusionLaneGroundMarking;

typedef struct {
  LineType type;                                                  // polyline类型
  uint8 local_points_size;                                        // 局部坐标系下点集数量
  Point3f local_points[FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM];  // 局部坐标系下点集
} _STRUCT_ALIGNED_ FusionPolyLine;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint64 isp_timestamp;  // ISP出图时间戳        (微秒)
  uint8 reference_line_msg_size;
  ReferenceLineMsg reference_line_msg[FUSION_ROAD_REFLINE_MAX_NUM];  // 参考线信息           <最大7个>
  uint8 lane_ground_markings_size;                                   // 地面标识数量
  FusionLaneGroundMarking
      lane_ground_markings[FUSION_ROAD_LANE_GROUND_MARKINGS_MAX_NUM];  // 地面标识列表，当前只输出人行横道
  boolean local_point_valid;  // 绝对坐标有效性       (true:valid / false:invalid)
  uint8 fusion_polyline_size;
  FusionPolyLine fusion_polyline[FUSION_ROAD_POLYLINE_MAX_NUM];
} _STRUCT_ALIGNED_ RoadInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  //_IFLYAUTO_FUSION_ROAD_H_
