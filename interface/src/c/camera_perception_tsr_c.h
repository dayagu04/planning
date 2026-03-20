// Copyright (C) iflyauto Technologies (2024). All rights reserved.
// Modified: 2024/02/21

#ifndef _IFLYAUTO_CAMERA_PERCEPTION_TSR_H_
#define _IFLYAUTO_CAMERA_PERCEPTION_TSR_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#define CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM 10
#define CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM 20 

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

// 辅助标志牌类型枚举
typedef enum {
  SUPP_SIGN_TYPE_UNKNOWN = 0,                                     // 未知类型
  SUPP_SIGN_TYPE_NO_ENTRY = 1,                                    // 禁止驶入
  SUPP_SIGN_TYPE_PROHIBIT_MOTOR_ENTERING = 2,                     // 禁止机动车驶入
  SUPP_SIGN_TYPE_NO_STOPPING = 3,                                 // 禁止车辆停放
  SUPP_SIGN_TYPE_NO_PARKING = 4,                                  // 禁止停车
  SUPP_SIGN_TYPE_PROHIBIT_PROLONGED_PARKING = 5,                  // 禁止长时间停车
  SUPP_SIGN_TYPE_NO_OVERTAKING = 6,                               // 禁止超车
  SUPP_SIGN_TYPE_CANCEL_NO_OVERTAKING = 7,                        // 解除禁止超车
  SUPP_SIGN_TYPE_PROHIBIT_TURN_LEFT = 8,                          // 禁止左转弯
  SUPP_SIGN_TYPE_PROHIBIT_TURN_RIGHT = 9,                         // 禁止右转弯
  SUPP_SIGN_TYPE_PROHIBIT_TURN_U = 10,                            // 禁止掉头
  SUPP_SIGN_TYPE_STOP_SIGN = 11,                                  // 停车标识
  SUPP_SIGN_TYPE_YIELD_SIGN = 12,                                 // 让行标识
  SUPP_SIGN_TYPE_NO_PASSING = 13,                                 // 禁止通行
  SUPP_SIGN_TYPE_NO_HONKING = 14,                                 // 禁止鸣喇叭
  SUPP_SIGN_TYPE_HONK = 15,                                       // 鸣喇叭
  SUPP_SIGN_TYPE_MAXIMUM_SPEED = 16,                              // 最高车速，包括电子限速
  SUPP_SIGN_TYPE_MINIMUM_SPEED = 17,                              // 最低车速，包括电子限速
  SUPP_SIGN_TYPE_END_OF_SPEED_LIMIT = 18,                         // 解除限速
  SUPP_SIGN_TYPE_ATTENTION_CHILDREN = 19,                         // 注意儿童
  SUPP_SIGN_TYPE_ATTENTION_PEDESTRIANS = 20,                      // 注意行人
  SUPP_SIGN_TYPE_ROAD_CONSTRUCTION_SIGN = 21,                     // 道路施工标识
  SUPP_SIGN_TYPE_SHARP_TURN_LEFT = 22,                            // 左转急弯
  SUPP_SIGN_TYPE_SHARP_TURN_RIGHT = 23,                           // 右转急弯
  SUPP_SIGN_TYPE_VEHICLE_MERGE_LEFT = 24,                         // 左侧车辆汇入
  SUPP_SIGN_TYPE_VEHICLE_MERGE_RIGHT = 25,                        // 右侧车辆汇入
  SUPP_SIGN_TYPE_SINGLE_WAY = 26,                                 // 单行路
  SUPP_SIGN_TYPE_CROSSROADS = 27,                                 // 交叉路口
  SUPP_SIGN_TYPE_CURVES_AHEAD = 28,                               // 连续弯路
  SUPP_SIGN_TYPE_UP_STEEP_SLOPE = 29,                             // 上陡坡
  SUPP_SIGN_TYPE_DOWN_STEEP_SLOPE = 30,                           // 下陡坡
  SUPP_SIGN_TYPE_NARROW_ROAD = 31,                                // 窄路
  SUPP_SIGN_TYPE_TUNNEL = 32,                                     // 隧道
  SUPP_SIGN_TYPE_MOTORWAY_START_SIGN = 33,                        // 高速公路启动标志
  SUPP_SIGN_TYPE_MOTORWAY_END_SIGN = 34,                          // 高速公路结束标志
} _ENUM_PACKED_ SuppSignType;

// 交通信号灯类型枚举
typedef enum {
  TRAFFIC_LIGHT_TYPE_UNKNOWN = 0,             // 未知
  TRAFFIC_LIGHT_TYPE_CIRCULAR = 1,            // 圆饼型
  TRAFFIC_LIGHT_TYPE_ARROW = 2,               // 箭头型
  TRAFFIC_LIGHT_TYPE_NUM = 3,                 // 数字型
  TRAFFIC_LIGHT_TYPE_OTHER = 4,               // 其他类型
  TRAFFIC_LIGHT_TYPE_LEFT_TURN = 5,           // 左转灯
  TRAFFIC_LIGHT_TYPE_STRAIGHT = 6,            // 直行灯
  TRAFFIC_LIGHT_TYPE_RIGHT_TURN = 7,          // 右转灯
  TRAFFIC_LIGHT_TYPE_U_TURN = 8,              // 掉头灯
} _ENUM_PACKED_ TrafficLightType;

// 交通信号灯颜色枚举
typedef enum {
  TRAFFIC_LIGHT_COLOR_UNKNOWN = 0,                                    // 未知
  TRAFFIC_LIGHT_COLOR_RED = 1,                                        // 红色交通灯
  TRAFFIC_LIGHT_COLOR_YELLOW = 2,                                     // 黄色交通灯
  TRAFFIC_LIGHT_COLOR_GREEN =3,                                       // 绿色交通灯
  TRAFFIC_LIGHT_COLOR_OFF = 4,                                        // 关闭
  TRAFFIC_LIGHT_COLOR_FLASHING_RED = 5,                               // 闪烁的红色交通灯
  TRAFFIC_LIGHT_COLOR_FLASHING_YELLOW = 6,                            // 闪烁的黄色交通灯
  TRAFFIC_LIGHT_COLOR_FLASHING_GREEN = 7,                             // 闪烁的绿色交通灯
} _ENUM_PACKED_ TrafficLightColor;

// 交通信号灯排布类型枚举
typedef enum {
  TRAFFIC_LIGHT_ARRANGE_TYPE_UNKNOWN = 0,                                    // 未知
  TRAFFIC_LIGHT_ARRANGE_TYPE_VERTICAL = 1,                                   // 纵向排布
  TRAFFIC_LIGHT_ARRANGE_TYPE_HORIZONTAL = 2,                                 // 横向排布
  TRAFFIC_LIGHT_ARRANGE_TYPE_OTHER = 3,                                      // 其他类型
} _ENUM_PACKED_ TrafficLightArrangeType;

// 交通标识的boundingbox信息(像素坐标系，可视化用)
typedef struct {
    float32 x;                              // 锚框左上角顶点的x坐标 (pixel)
    float32 y;                              // 锚框右上角顶点的y坐标 (pixel)
    float32 width;                          // 锚框的宽度 (pixel)
    float32 height;                         // 锚框的高度 (pixel)
} _STRUCT_ALIGNED_ TrafficSignBoundingbox;

// 辅助标志牌信息
typedef struct {
  uint8 id;                                     // 跟踪id号
  SuppSignType supp_sign_type;                  // 辅助标志牌类型
  float32 supp_sign_x;                          // 辅助标志牌纵向距离 (m)
  float32 supp_sign_y;                          // 辅助标志牌横向距离 (m)
  float32 supp_sign_z;                          // 辅助标志牌高度     (m)
  uint8 speed_limit;                            // 限速速度值(km/h)  仅在标识牌类型为【MAXIMUM_SPEED】、【MINIMUM_SPEED】、【END_OF_SPEED_LIMIT】时对该字段赋值,否则该字段默认赋值为0
  TrafficSignBoundingbox bbox;                  // 辅助标志牌的boundingbox
} _STRUCT_ALIGNED_ CameraPerceptionSuppSign;

// 交通信号灯信息
typedef struct {
  uint8 id;                                             // 跟踪id号
  TrafficLightType traffic_light_type;                  // 交通信号灯类型
  TrafficLightColor traffic_light_color;                // 交通信号灯颜色
  TrafficLightArrangeType traffic_light_arrange_type;   // 交通信号灯排布类型
  float32 traffic_light_x;                              // 交通信号灯纵向距离 (m)
  float32 traffic_light_y;                              // 交通信号灯横向距离 (m)
  float32 traffic_light_z;                              // 交通信号灯高度     (m)
  TrafficSignBoundingbox bbox;                          // 交通信号灯的boundingbox
  int32 countdown_number;                               // 交通灯倒计时时间  仅在交通信号灯类型为【TRAFFIC_LIGHT_TYPE_NUM】时对其赋值
} _STRUCT_ALIGNED_ CameraPerceptionTrafficLight;

// 通行状态信息   
/* 个位表示通行状态颜色：0 未知  1 红(禁止)  2 黄(等待)  3 绿(允许)  4 关闭
   十位表示闪烁状态：    0 未知  1 红灯闪烁  2 黄灯闪烁  3 绿灯闪烁  4 非闪烁状态(常亮或常灭)*/                
typedef struct {
  int8                    go_left;                       // 左转状态
  int8                    go_straight;                   // 直行状态
  int8                    go_right;                      // 右转状态
  int8                    go_uturn;                      // 调头状态 
} _STRUCT_ALIGNED_ CameraPerceptionTrafficStatus;

// TSR信息集合
typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint64 isp_timestamp;                                                                   // 图像曝光中间时刻时间戳    (微秒)
  uint8 supp_signs_size;                                                                  // 辅助标志牌数量
  CameraPerceptionSuppSign supp_signs[CAMERA_PERCEPTION_SUPP_SIGNS_MAX_NUM];              // 辅助标志牌列表
  uint8 traffic_lights_size;                                                              // 交通信号灯数量
  CameraPerceptionTrafficLight traffic_lights[CAMERA_PERCEPTION_TRAFFIC_LIGHTS_MAX_NUM];  // 交通信号灯列表
  CameraPerceptionTrafficStatus traffic_status;                                           // 通行状态
} _STRUCT_ALIGNED_ CameraPerceptionTsrInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_CAMERA_PERCEPTION_TSR_H_