// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_PREDICTION_H_
#define _IFLYAUTO_PREDICTION_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"
#include "fusion_objects_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#define PREDICTION_OBSTACLE_MAX_NUM 128
#define PREDICTION_TRAJ_POINT_NUM 26

#pragma pack(4)

typedef struct {
  float32 sigma_x;           // x方向标准差
  float32 sigma_y;           // y方向标准差
  float32 correlation;       // 相关系数
  float32 area_probability;  // 置信概率
  float32 ellipse_a;         // 置信椭圆长轴
  float32 ellipse_b;         // 置信椭圆短轴
  float32 theta_a;           // 椭圆倾斜角
} _STRUCT_ALIGNED_ GaussianInfo;

typedef struct {
  float32 confidence;  // 轨迹点置信度     [0.0-1.0]
  Point2f position;    // 轨迹点障碍物几何中心在x-y坐标系下的位置信息  (米)

  /** 轨迹点障碍物在x-y坐标系下的朝向角
   *  单位：弧度   (rad)
   *  备注：朝x轴正向绕z轴逆时针旋转为正（0~Π），顺时针旋转为负（0~-Π）
   **/
  float32 yaw;

  /** 轨迹点障碍物在绝对坐标系下的速度方向角
   *  单位：弧度   (rad)
   *  备注：朝x轴正向绕z轴逆时针旋转为正（0~Π），顺时针旋转为负（0~-Π）
   **/
  float32 theta_vel;
  float32 velocity;            // 轨迹点障碍物在x-y坐标系下的线速度           (米/秒)
  Point2f relative_position;   // 轨迹点障碍物几何中心到自车后轴中心的相对距离 (米)
  Point2f relative_velocity;   // 轨迹点障碍物相对于自车后轴中心的相对速度 (米/秒)
  float32 relative_yaw;        // 轨迹点障碍物在自车坐标系中朝向角            (弧度rad)
  float32 relative_theta_vel;  // 轨迹点障碍物在自车坐标系中速度方向角 (弧度rad)
  GaussianInfo gaussian_info;  // 轨迹点障碍物的高斯信息
} _STRUCT_ALIGNED_ PredictionTrajectoryPoint;

typedef struct {
  float32 confidence;             // 障碍物轨迹置信度             [0.0-1.0]
  uint64 time_stamp;              // 障碍物轨迹点起始生成时刻     (微秒)
  float64 relative_time;          // 生成障碍物轨迹点的时间间隔   (秒)
  uint8 trajectory_point_size;    // 障碍物轨迹点个数
  PredictionTrajectoryPoint trajectory_point[PREDICTION_TRAJ_POINT_NUM];  // 障碍物轨迹点
} _STRUCT_ALIGNED_ PredictionTrajectory;

typedef enum {
  OBSTACLE_INTENT_COMMON = 0,  // 正常行驶
  OBSTACLE_INTENT_CUT_IN = 1,  // 有切入意图
} _ENUM_PACKED_ OstacleIntentType;

typedef struct {
  OstacleIntentType type;  // 障碍物意图类型
} _STRUCT_ALIGNED_ ObstacleIntent;

typedef struct {
  FusionObject fusion_obstacle;  // 融合障碍物结果
  PredictionTrajectory trajectory;  // 障碍物轨迹，预留多条轨迹
  ObstacleIntent obstacle_intent;   // 障碍物意图
} _STRUCT_ALIGNED_ PredictionObject;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint8 prediction_obstacle_list_size;
  PredictionObject prediction_obstacle_list[PREDICTION_OBSTACLE_MAX_NUM];  // 预测障碍物信息
} _STRUCT_ALIGNED_ PredictionResult;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_PREDICTION_H_