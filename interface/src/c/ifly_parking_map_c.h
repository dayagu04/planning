// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_IFLY_PARKING_MAP_H_
#define _IFLYAUTO_IFLY_PARKING_MAP_H_

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

#define PARKING_MAP_POLYLINE_POINT_MAX_NUN 1000
#define PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN 20
#define PARKING_MAP_LANE_SEGMENT_SEGS_MAX_NUN 100
#define PARKING_MAP_DIRECTION_MAX_NUN 10
#define PARKING_MAP_LANE_MAX_NUN 20
#define PARKING_MAP_LANE_BOUNDARY_GROUP_MAX_NUN 20
#define PARKING_MAP_LANE_GROUP_MAX_NUN 20
#define PARKING_MAP_ROAD_STRUCTURE_MAX_NUN 10
#define PARKING_MAP_REFERENCE_MAX_NUN 10
#define PARKING_MAP_ROADMARK_POINT_MAX_NUN 10
#define PARKING_MAP_POLYGON_CORNER_MAX_NUN 10
#define PARKING_MAP_ROAD_OBSTACLE_POINT_MAX_NUN 60
#define PARKING_MAP_ROAD_MAX_NUN 1
#define PARKING_MAP_LANE_SEGMENT_MAX_NUN 1
#define PARKING_MAP_ROAD_MARK_MAX_NUN 20
#define PARKING_MAP_ROAD_OBSTACLE_MAX_NUN 40
#define PARKING_MAP_PARKING_SPACE_MAX_NUN 48
#define PARKING_MAP_TARGET_PRK_POS_NUN 4
#define PARKING_MAP_LIMITER_POINT_NUM 2
#define PARKING_MAP_LIMITER_MAX_NUN 2
#define PARKING_MAP_RAMP_MAX_NUN 10
#define PARKING_MAP_POLYGON_OBSTACLE_MAX_NUN 40
#define PARKING_MAP_EHP_MAP_MAX_NUN 100
#define PARKING_MAP_EHP_MESSAGE_DESCRIBE_STR_LEN 128
#define PARKING_MAP_FREE_SLOT_REGION_NUM 6


// 坐标系类型
typedef enum {
  COORDINATE_LLH = 1,  // 经纬高
  COORDINATE_ENU = 2,  // 东北天
  COORDINATE_BOOT = 3,  // 开机系
} _ENUM_PACKED_ CoordinateType;
typedef struct {
  CoordinateType coordinate_type;
  union {
    PointLLH llh;
    PointENU enu;
    Point3d boot;
  };
} _STRUCT_ALIGNED_ Coordinate;
typedef enum {
  ATTRIBUTE_ID_ATTRIBUTE_TYPE_UNKNOWN = 0,  // 未知类型

  // 用于定位的地图属性字段 (1 - 50)
  ATTRIBUTE_ID_ATTRIBUTE_TYPE_POLE = 1,                  // 电线杆
  ATTRIBUTE_ID_ATTRIBUTE_TYPE_TRAFFIC_SIGN = 2,          // 交通标志
  ATTRIBUTE_ID_ATTRIBUTE_TYPE_DASHED_LINE_SEGEMENT = 3,  // 虚线段

  // 用于导航的地图属性字段 (51 - 100)
  ATTRIBUTE_ID_ATTRIBUTE_TYPE_TRAFFIC_LIGHT = 51,     // 交通信号灯
  ATTRIBUTE_ID_ATTRIBUTE_TYPE_ROAD_MARK = 52,         // 道路标志物
  ATTRIBUTE_ID_ATTRIBUTE_TYPE_ROAD_OBSTACLE = 53,     // 道路障碍物
  ATTRIBUTE_ID_ATTRIBUTE_TYPE_ROAD_FACILITY = 54,     // 道路设施
  ATTRIBUTE_ID_ATTRIBUTE_TYPE_LANE_MERGE_SPLIT = 55,  // 车道合并/分流
  ATTRIBUTE_ID_ATTRIBUTE_TYPE_SPEED_LIMIT = 56,       // 速度限制标志
  ATTRIBUTE_ID_ATTRIBUTE_TYPE_DIVERGENT_POINT = 57,   // 分歧点
  ATTRIBUTE_ID_ATTRIBUTE_TYPE_MULTI_DIGITIZED = 58,   // 数字化分段
  ATTRIBUTE_ID_ATTRIBUTE_TYPE_BUS_LANE = 59,          // 公交车道
  ATTRIBUTE_ID_ATTRIBUTE_TYPE_ROAD_PRIORITY = 60,     // 道路优先权
  ATTRIBUTE_ID_ATTRIBUTE_TYPE_POLYGON_OBSTACLE = 61,  // BOX障碍物

  // 用于特定点的地图属性字段 (101 - 200)
  ATTRIBUTE_ID_ATTRIBUTE_TYPE_POI = 101,               // 特定点
  ATTRIBUTE_ID_ATTRIBUTE_TYPE_PARKING_SPACE = 102,     // 停车位
  ATTRIBUTE_ID_ATTRIBUTE_TYPE_PARKING_LOT = 103,       // 停车场
  ATTRIBUTE_ID_ATTRIBUTE_TYPE_PARKING_FACILITY = 104,  // 停车设施

  // 用于指南的地图属性字段 experience (201 - 250)
  ATTRIBUTE_ID_ATTRIBUTE_TYPE_EXPERIENCE_GUIDE_LINE = 201,  // 体验指南线
  ATTRIBUTE_ID_ATTRIBUTE_TYPE_CROSS_LAYER_GROUP = 202,      // 交叉路口层组
} _ENUM_PACKED_ AttributeIdAttributeType;

// 属性标识 (描述对象属性标识)
typedef struct _AttributeId {
  AttributeIdAttributeType type;  // 属性类型（描述对象属性）
  TileId tile_id;                 // 地图瓦片标识id
  uint32 count;                   // 对象id
  FloorId floor_id;               // 楼层id
} _STRUCT_ALIGNED_ AttributeId;

// 道路标识id信息
typedef struct {
  TileId tile_id;    // 地图瓦片标识id
  uint32 count;      // 道路id
  UrId ur_id;        // 城市标识id
  FloorId floor_id;  // 楼层id
} _STRUCT_ALIGNED_ RoadId;

// 道路端口标识id
typedef struct {
  TileId tile_id;  // 地图瓦片标识id
  uint32 count;    // 道路口id
} _STRUCT_ALIGNED_ RoadPortId;

// 车道组id信息
typedef struct {
  RoadId road_id;  // 车道组所在的道路id信息
  boolean
      dir_reversed;  // 保留字段，车道组方向是否被反转，true表示车道组的方向与正常方向相反，否则表示方向与正常方向一致。
  uint32 index;      // 车道祖index，车道组在道路上的索引顺序
} _STRUCT_ALIGNED_ LaneGroupId;

// 车道边界组id信息
typedef struct {
  LaneGroupId lane_group_id;  // 车道组id信息
  uint32 index;               // 车道组index，车道边界组在所属车道组中的索引顺序
} _STRUCT_ALIGNED_ LaneBoundaryGroupId;

// 车道标识id信息
typedef struct {
  LaneGroupId lane_group_id;  // 车道组id信息
  uint32 index;               // 车道index，车道在车道组中的索引顺序
} _STRUCT_ALIGNED_ LaneId;

// 车道端口标识id
typedef struct {
  TileId tile_id;  // 地图瓦片标识id
  uint32 count;    // 车道id
} _STRUCT_ALIGNED_ LanePortId;

// 平行车道边界id信息
typedef struct {
  LaneBoundaryGroupId lane_boundary_group_id;  // 车道边界组id信息
  uint32 index;  // 平行车道边界index，平行车道边界在车道边界组中的索引
} _STRUCT_ALIGNED_ ParallelLaneBoundaryId;

// 车道边界id信息
typedef struct {
  ParallelLaneBoundaryId parallel_lane_boundary_id;  // 平行车道边界id信息
  uint32 index;  // 车道边界index，车道边界在平行车道边界中的索引
} _STRUCT_ALIGNED_ LaneBoundaryId;

// 交叉口id信息
typedef struct {
  TileId tile_id;  // 地图瓦片标识id信息
  uint32 count;    // 交叉口索引id（交叉口在所属瓦片内的索引）
  UrId ur_id;      // 城市标识id信息
} _STRUCT_ALIGNED_ IntersectionId;

typedef struct {
  uint16 points_size;
  Coordinate points[PARKING_MAP_POLYLINE_POINT_MAX_NUN];  // 点的坐标
  float length;                                // 长度       (米)
  uint8 buffer[PROTO_STR_LEN];                 // 保留字段，用于提高序列化/反序列化的速度
} _STRUCT_ALIGNED_ Polyline;

// 车道线类型
typedef enum {
  LANE_BOUNDARY_LINE_UNKNOWN = 0,  // 未知
  LANE_BOUNDARY_LINE_SOLID_LINE,   // 实线
  LANE_BOUNDARY_LINE_DASHED_LINE,  // 虚线
  LANE_BOUNDARY_LINE_VIRTUAL,      // 虚拟线
  LANE_BOUNDARY_LINE_PHYSICAL,     // 实体线
  LANE_BOUNDARY_LINE_LOGICAL,      // 逻辑线
} _ENUM_PACKED_ LaneBoundaryLineType;

// 车道边界类型
typedef enum {
  LANE_BOUNDARY_UNKNOWN_TYPE = 0,  // 未知类型
  LANE_BOUNDARY_LANE_BORDER,       // 车道边界
  LANE_BOUNDARY_ROAD_BORDER,       // 道路边界
} _ENUM_PACKED_ LaneBoundaryBorderType;
// 颜色
typedef enum {
  LANE_BOUNDARY_UNKNOWN_COLOR = 0,  // 未知颜色
  LANE_BOUNDARY_WHITE,              // 白色
  LANE_BOUNDARY_YELLOW,             // 黄色
} _ENUM_PACKED_ LaneBoundaryColor;
// 车道边界信息来源
typedef enum {
  LANE_BOUNDARY_SOURCE_AUTO_MAPPING = 0,  // 自动映射
  LANE_BOUNDARY_SOURCE_MANUAL_VALID,      // 手动验证
  LANE_BOUNDARY_SOURCE_MANUAL_GUESS,      // 手动猜测
  LANE_BOUNDARY_SOURCE_OTHER,             // 未知
} _ENUM_PACKED_ LaneBoundarySource;

// 车道边界
typedef struct _LaneBoundary {
  LaneBoundaryId id;                          // 车道边界id信息
  LaneBoundaryLineType line_type;             // 车道边界线型类型
  LaneBoundaryBorderType lane_boundary_type;  // 车道边界类型
  Polyline geometry;                          // 多线段
  LaneBoundaryColor color;                    // 车道边界颜色
  LaneBoundarySource source;                  // 车道边界信息来源
} _STRUCT_ALIGNED_ ParkingLaneBoundary;

// 平行车道边界
typedef struct {
  uint8 sequential_lane_boundaries_size;
  ParkingLaneBoundary sequential_lane_boundaries[PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN];  // 多个平行车道边界
  ParallelLaneBoundaryId id;                                                       // 平行车道边界id信息
} _STRUCT_ALIGNED_ ParallelLaneBoundary;

// 车道边界组
typedef struct {
  LaneBoundaryGroupId id;  // 车道边界组id信息
  uint8 parallel_lane_boundaries_size;
  ParallelLaneBoundary parallel_lane_boundaries[PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN];  // 平行车道边界
} _STRUCT_ALIGNED_ LaneBoundaryGroup;

// 分段信息
typedef struct {
  uint32 start_index;  // 起始索引
  uint32 end_index;    // 结束索引
  float length;        // 分段长度
} _STRUCT_ALIGNED_ Segment;

// 车道分段信息
typedef struct {
  LaneId lane_id;  // 车道标识id信息
  uint32 segs_size;
  Segment segs[PARKING_MAP_LANE_SEGMENT_SEGS_MAX_NUN];  // 多个分段车道信息
} _STRUCT_ALIGNED_ LaneSegment;

// 车道类型
typedef enum {
  MAPPING_LANE_TYPE_NORMAL = 0,              // 普通车道
  MAPPING_LANE_TYPE_ACCELERATE,              // 加速车道
  MAPPING_LANE_TYPE_DECELERATE,              // 减速车道
  MAPPING_LANE_TYPE_BUS,                     // 公交车道
  MAPPING_LANE_TYPE_EMERGENCY,               // 紧急车道
  MAPPING_LANE_TYPE_ACC_DEC,                 // 加减速车道
  MAPPING_LANE_TYPE_LEFT_TURN_WAITING_AREA,  // 左转等待区
  MAPPING_LANE_TYPE_PARKING,                 // 停车区
  MAPPING_LANE_TYPE_TIDAL,                   // 潮汐车道（根据交通流量变化而设定的车道）
  MAPPING_LANE_TYPE_NON_MOTOR,               // 非机动车道
  MAPPING_LANE_TYPE_TOLL_ETC,                // ETC 收费车道
  MAPPING_LANE_TYPE_TOLL_MANUAL,             // 人工收费车道
  MAPPING_LANE_TYPE_TOLL,                    // 收费车道
  MAPPING_LANE_TYPE_VARIABLE,                // 可变车道
  MAPPING_LANE_TYPE_HOV,                     // 用于高乘客数车辆的车道
} _ENUM_PACKED_ MappingLaneType;

// 车道信息
typedef struct _Lane {
  LaneId id;                                 // 车道标识id信息
  MappingLaneType lane_type;                 // 车道类型
  Polyline center_line;                      // 车道中心线（也会用来存储车道线）
  LaneBoundaryGroupId left_boundary_group;   // 车道左边界组id信息
  LaneBoundaryGroupId right_boundary_group;  // 车道右边界组id信息
  LanePortId head_id;                        // 车道起始端标识id信息
  LanePortId tail_id;                        // 车道终止端标识id信息
  LaneDrivableDirection direction;           // 车道行驶箭头方向
} _STRUCT_ALIGNED_ Lane;

// 车道组信息
typedef struct {
  LaneGroupId id;             // 车道组id信息
  float length;               // 车道组长度
  uint8 lanes_size;           // 车道组包含的车道数量
  Lane lanes[PARKING_MAP_LANE_MAX_NUN];  // 车道组中的所有车道，每个车道有自己的属性信息
  uint8 lane_boundary_groups_size;
  LaneBoundaryGroup lane_boundary_groups[PARKING_MAP_LANE_BOUNDARY_GROUP_MAX_NUN];  // 车道组的车道边界信息
} _STRUCT_ALIGNED_ LaneGroup;

// 道路类别
typedef enum {
  ROAD_CLASS_UNKNOWN = 0,  // 未知类别
  HIGHWAY,                 // 高速公路
  URBAN,                   // 城市道路
  CITY_EXPRESSWAY,         // 城市快速路
} _ENUM_PACKED_ RoadClass;
// 道路结构信息
typedef enum {
  ROAD_STRUCTURE_UNKNOWN = 0,  // 未知结构
  TUNNEL,                      // 隧道
  TOLL,                        // 收费站
  BRIDGE,                      // 桥梁
  ROUNDABOUT,                  // 环岛
} _ENUM_PACKED_ RoadStructure;
// 道路类型
typedef enum {
  ROAD_TYPE_UNKNOWN = 0,   // 未知类型
  U_TURN,                  // U形转弯
  INTERSECTION_LEFT_TURN,  // 交叉口左转
  SEPARATE_TURN_RIGHT,     // 右转道
  SEPARATE_TURN_LEFT,      // 左转道
} _ENUM_PACKED_ RoadType;
// 道路行驶方向
typedef enum {
  ROAD_TRAVEL_DIRECTION_UNKNOWN = 0,  // 未知方向
  ROAD_TRAVEL_DIRECTION_NONE,         // 无特定行驶方向
  ROAD_TRAVEL_DIRECTION_A_B,          // 正向行驶方向
  ROAD_TRAVEL_DIRECTION_B_A,          // 反向行驶方向
  ROAD_TRAVEL_DIRECTION_BOTH,         // 双向行驶方向
} _ENUM_PACKED_ RoadTravelDirection;
// 匝道类型
typedef enum {
  ROAD_RAMP_NOT = 0,       // 非匝道（没有匝道）
  ROAD_RAMP_IC,            // 互通式匝道
  ROAD_RAMP_JCT,           // 交叉式匝道
  ROAD_RAMP_SAPA,          // 上下匝道
  ROAD_RAMP_PARKING_RAMP,  // 停车坡道
} _ENUM_PACKED_ RoadRamp;

// 道路信息
typedef struct _Road {
  RoadId id;                             // 道路标识id信息
  RoadPortId head_id;                    // 道路起始端标识id信息
  RoadPortId tail_id;                    // 道路终止端标识id信息
  float length;                          // 道路长度
  RoadType road_type;                    // 道路类型
  Polyline road_center;                  // 道路中心线（也会复用这个字段用来存储轨迹信息）
  RoadTravelDirection travel_direction;  // 道路行驶方向
  RoadRamp ramp;                                 // 匝道类型
  uint8 name[PROTO_STR_LEN];                     // 道路名称
  RoadClass road_class;                          // 道路类别
  uint8 road_structure_size;
  RoadStructure road_structure[PARKING_MAP_ROAD_STRUCTURE_MAX_NUN]; // 道路结构信息
  boolean intersection_internal;   // 描述内部是否有交叉 (true：有交叉/false:不相交)
  IntersectionId intersection_id;  // 道路的交叉口标识id
} _STRUCT_ALIGNED_ Road;

// 地图元素的方位朝向
typedef enum {
  REFERENCE_ORIENTATION_UNKNOWN = 0,  // 未知
  REFERENCE_ORIENTATION_LEFT,         // 左侧
  REFERENCE_ORIENTATION_RIGHT,        // 右侧
  REFERENCE_ORIENTATION_INTERSECT,    // 相交
} _ENUM_PACKED_ ReferenceOrientation;

// 坡道类型
typedef enum {
  RAMP_TYPE_UNKNOWN = 0,  // 未知
  RAMP_TYPE_UP,           // 上坡
  RAMP_TYPE_DOWN,         // 下坡
} _ENUM_PACKED_ RampType;

// 绑定的id类型
typedef enum {
  REFERENCE_ID_UNKNOWN = 0,  // 未知
  REFERENCE_ID_ROAD,         // 道路id
  REFERENCE_ID_LANE,         // 车道id
  REFERENCE_ID_ATTRIBUTE,    // 标识属性id
  REFERENCE_ID_ROAD_PORT,    // 路口id
  REFERENCE_ID_LANE_PORT,    // 车道口id
} _ENUM_PACKED_ ReferenceIdType;

// 绑定关系（存储绑定的地图道路信息）
typedef struct _Reference {
  union {
      RoadId road_id;            // 道路id
      LaneId lane_id;            // 车道id
      AttributeId attribute_id;  // 标识属性id
      RoadPortId road_port;      // 路口id
      LanePortId lane_port;      // 车道口id
  };
  float32 offset;                    // 车位中心到道路上某一个点的距离
  float32 length;                    // 长度，保留字段
  ReferenceOrientation orientation;  // 方位
  ReferenceIdType id_type;           // id类型
  boolean fully_covered;             // 地图元素中是否有完全重叠 (true：有重叠，则offset和length字段不需要使用/false:无重叠)
} _STRUCT_ALIGNED_ Reference;

// 道路标线类型
typedef enum {
  ROAD_MARK_TYPE_UNKNOWN = 0,                   // 未知类型
  ROAD_MARK_TYPE_SPEED_BUMP,                    // 减速带
  ROAD_MARK_TYPE_STOP_LINE,                     // 停车线
  ROAD_MARK_TYPE_GUIDE_ARROW,                   // 指示箭头
  ROAD_MARK_TYPE_CROSSWALK,                     // 斑马线，人行横道
  ROAD_MARK_TYPE_CENTRAL_CIRCLE,                // 中央圆圈（通常表示道路上的交叉口或环岛）
  ROAD_MARK_TYPE_NO_PARKING_ZONE,               // 禁停区
  ROAD_MARK_TYPE_INDICATED_LINE,                // 指示线
  ROAD_MARK_TYPE_LATERAL_DECELERATION_MARKING,  // 侧向减速标线
  ROAD_MARK_TYPE_SYMBOL,                        // 符号（包括道路上的各种标志、符号、图标）
  ROAD_MARK_TYPE_TEXT,  // 文本（用于显示道路上的文本信息，如路名、方向指示等）
} _ENUM_PACKED_ RoadMarkType;
// 道路标线的涂装类型
typedef enum {
  ROAD_MARK_PAINTINGTYPE_UNKNOWN = 0,  // 未知类型
  ROAD_MARK_PAINTINGTYPE_NORMAL,       // 常规类型
  ROAD_MARK_PAINTINGTYPE_SPECIAL,      // 特殊类型
} _ENUM_PACKED_ RoadMarkPaintingType;

// 道路标线信息
typedef struct _RoadMark {
  AttributeId id;     // 属性标识id信息
  RoadMarkType type;  // 道路标线类型
  uint8 ref_size;
  Reference ref[PARKING_MAP_REFERENCE_MAX_NUN];  // 绑定关系（绑定的地图道路信息）
  uint8 shape_size;
  Coordinate shape[PARKING_MAP_ROADMARK_POINT_MAX_NUN];  // 道路标线的几何位置信息
  float32 angle;  // 标识朝向角度，只对某些特定类型的道路标线（如指示箭头）有效。
  LaneDrivableDirection direction;  // 标识方向类型，只对某些特定类型的道路标线（如指示箭头）有效。
  RoadMarkPaintingType painting_type;  // 道路标线的涂装类型，用于满足特殊斑马线的渲染需求
} _STRUCT_ALIGNED_ RoadMark;

// 道路障碍物信息
typedef struct _RoadObstacle {
  AttributeId id;         // 属性标识id信息
  GroundLineType type;    // 道路障碍物类型
  uint8 ref_size;
  Reference ref[PARKING_MAP_REFERENCE_MAX_NUN];  // 绑定关系（绑定的地图道路信息）
  uint8 shape_size;
  Coordinate shape[PARKING_MAP_ROAD_OBSTACLE_POINT_MAX_NUN];  // 道路障碍物的几何位置信息
} _STRUCT_ALIGNED_ RoadObstacle;

// 单个限位器信息
typedef struct {
  AttributeId id;       // 属性标识id信息
  Coordinate end_points[PARKING_MAP_LIMITER_POINT_NUM]; // 车体限位器端点位置 （米） 限两个, boot坐标系下
} _STRUCT_ALIGNED_ ParkingMapLimiter;

// 车位信息
typedef struct _ParkingSpace {
    AttributeId id;                                           // 属性标识id信息
    ParkingSlotType parking_space_type;                       // 车位类型
    uint8 ref_size;                                           // 绑定关系的数量
    Reference ref[PARKING_MAP_REFERENCE_MAX_NUN];             // 绑定关系（绑定的地图道路信息）
    Coordinate shape[PARKING_MAP_TARGET_PRK_POS_NUN];         // 车位角点  <固定4个>
    uint8 floor_name[PROTO_STR_LEN];                          // 楼层
    boolean allow_parking;                                    // 是否被占用
    boolean is_turn_corner;                                   // 车位角点是否翻转
    SlotSourceType slot_source;                               // 车位来源
    uint8 limiters_size;                                      // 此库位限位器个数
    ParkingMapLimiter limiters[PARKING_MAP_LIMITER_MAX_NUN];  // 单个库位最多2个限位器
    int32 empty_votes;                                        // 保留字段
} _STRUCT_ALIGNED_ ParkingSpace;

// 目标停车位id信息
typedef struct {
  AttributeId attribute_id;  // 属性标识id信息
  Reference ref;             // 绑定关系（存储绑定的地图道路信息）
} _STRUCT_ALIGNED_ TargetParkingId;

// 通过3D BOX感知融合得到的障碍物类型
typedef enum {
  POLYGON_OBJECT_TYPE_UNKNOWN = 0,                   // 未知类型
  POLYGON_OBJECT_TYPE_COLUMN = 1,                    // 立柱
} _ENUM_PACKED_ PolygonObjectType;

typedef struct _PolygonObject {
    AttributeId id;                       // 属性标识id信息
    PolygonObjectType type;               // 障碍物类型
    uint8 ref_size;                       // 绑定关系的数量
    Reference ref[PARKING_MAP_REFERENCE_MAX_NUN];    // 绑定关系（绑定的地图道路信息）
    uint8 corner_size;
    Coordinate corners[PARKING_MAP_POLYGON_CORNER_MAX_NUN]; // 障碍物角点  <最大10个>
} _STRUCT_ALIGNED_ PolygonObject;

typedef struct {
  TileId id;  // 地图瓦片标识id

  // 基础信息
  uint8 road_size;
  Road road[PARKING_MAP_ROAD_MAX_NUN];  // 道路信息        <最大200个>

  // 路由信息
  uint8 lane_segment_size;
  LaneSegment lane_segment[PARKING_MAP_LANE_SEGMENT_MAX_NUN];  // 分段信息        <最大200个>

  // 导航信息
  uint8 road_mark_size;
  RoadMark road_mark[PARKING_MAP_ROAD_MARK_MAX_NUN];  // 道路标线信息     <最大200个>
  uint8 road_obstacle_size;
  RoadObstacle road_obstacle[PARKING_MAP_ROAD_OBSTACLE_MAX_NUN];  // 道路障碍物信息   <最大1000个>
  uint8 parking_space_size;
  ParkingSpace parking_space[PARKING_MAP_PARKING_SPACE_MAX_NUN];  // 停车位信息       <最大1000个>
  uint8 polygon_obstacle_size;
  PolygonObject polygon_obstacle[PARKING_MAP_POLYGON_OBSTACLE_MAX_NUN];       // 通过3D BOX感知融合得到的障碍物信息 <最大1000个>
} _STRUCT_ALIGNED_ RoadTile;

typedef struct {
  boolean available;       // 信息是否可用标志
  uint64 detection_time;   // 检测时间 (微秒)
  PointLLH global_pos;     // 点的位置（global坐标系）
  Point3d local_pos;       // 点的位置（local坐标系）
  Quaternion global_quat;  // 点的方位（global坐标系）
  Quaternion local_quat;   // 点的方位（local坐标系）
} _STRUCT_ALIGNED_ FeaturePoint;

// 坡道信息
typedef struct {
  uint32 id;                           // 坡道id
  RampType ramp_type;                  // 坡道类型
  FeaturePoint entrance_ramp_point;    // 入坡点信息
  FeaturePoint exit_ramp_point;        // 出坡点信息
} _STRUCT_ALIGNED_ RampInfo;

// 语义信息
typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  FeaturePoint gps_signal_loss_point;  // 消星点信息
  float32 gps_snr;                     // 信号强度
  boolean gps_signal_available;        // 卫星信号是否可用标志 (0: 不可用, 信号强度通常低于16. 1: 可用, 信号强度通常高于16)
  uint8 ramp_info_size;
  RampInfo ramp_info[PARKING_MAP_RAMP_MAX_NUN];  // 坡道信息
} _STRUCT_ALIGNED_ SemanticInfo;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint64 timestamp;                                           // localmap 的时间戳 (微妙)
  uint64 perception_isp_timestamp;                            // 当前帧使用的感知isp时间戳（微妙）
  PointLLH trajectory_ref_point;                              // 参考点（LLH），用于将local坐标系(ENU)-global坐标系(LLH)坐标进行相互转换的参考点
  Coordinate trace_start;                                     // 轨迹起点
  Coordinate trace_dest;                                      // 轨迹终点
  Coordinate target_prk_pos[PARKING_MAP_TARGET_PRK_POS_NUN];  // 目标停车位四个角点信息   <固定4个>
  TargetParkingId target_prk_id;                              // 目标停车位id信息
  RoadTile road_tile_info;                                    // 道路相关信息
  SemanticInfo semantic_info;                                 // 语义信息
  uint32 map_id;                                              // ehp每次更新地图消息，值加1
} _STRUCT_ALIGNED_ ParkingInfo;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint8 file_name[PROTO_STR_LEN];    // 地图文件名
  uint8 file_path[PROTO_STR_LEN];    // 地图文件路径
  uint8 vehicle_vin[PROTO_STR_LEN];  // 车辆识别代码
} _STRUCT_ALIGNED_ ParkingMapFileInfo;

// Ehp输出信息to ivi
typedef enum {
  EHP_NOTIFY_NO_REQ,                      // 无消息         
  EHP_QUERY_SUCCESS = 1,            // 地图查询成功 
  EHP_QUERY_FAILED = 2,            // 地图查询失败
  EHP_DELETE_SUCCESS = 3,         // 删除成功
  EHP_DELETE_FAILED = 4,         // 删除失败
  EHP_UPDATE_SUCCESS = 5,        // 更新成功
  EHP_UPDATE_FAILED = 6,         // 更新失败
  EHP_SAVE_MAP_SUCCESS = 7,         // 地图保存成功
  EHP_QUERY_SPECIFIC_MAP_SUCCESS = 8,            // 指定地图查询成功
  EHP_QUERY_SPECIFIC_MAP_FAILED = 9,            // 指定地图查询失败 
} _ENUM_PACKED_ EhpStatus;

// ehp输出信息 to fsm 表征静默建图是否存在老地图
typedef enum {
  EHP_MAP_DEFAULT = 0,         // 默认状态
  EHP_MAP_EXIST_OLD_MAP = 1,         // 存在老地图
} _ENUM_PACKED_ EhpMapStatus;

// ehp输出信息to fsm
typedef enum {
  EHP_MATCHING_WITH_GPS_SIGNAL_LOSS_POINT_DEFAULT = 0 ,        // 默认状态
  EHP_MATCHING_WITH_GPS_SIGNAL_LOSS_POINT_SUCCESS = 1,         // hpp消星点与地图匹配成功
  EHP_MATCHING_WITH_GPS_SIGNAL_LOSS_POINT_FAILED = 2,          // hpp消星点与地图匹配失败
} _ENUM_PACKED_ EhpMatchingStatus;

typedef struct {
  uint8_t map_file_id;                // 地图文件ID
  char map_file_name[PROTO_STR_LEN];  // 地图文件名称
} _STRUCT_ALIGNED_ EhpfileInfo;

//EHP输出topic /iflytek/ehp/map_manager
typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint8_t map_file_size;                                      // 地图文件总个数
  EhpfileInfo map_file[PARKING_MAP_EHP_MAP_MAX_NUN];          // 地图文件列表 上限100个
  EhpStatus ehp_status;                                       // 地图管理处理状态
  EhpMatchingStatus ehp_matching_status;                      // 消星点匹配状态
  EhpMapStatus ehp_map_status;                                // 静默地图状态
  char failed_reason[PARKING_MAP_EHP_MESSAGE_DESCRIBE_STR_LEN];   // 失败原因
} _STRUCT_ALIGNED_ EhpOutput;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_IFLY_PARKING_MAP_H_
