#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "modules/common/config/basic_type.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/polygon2d.h"
#include "common/vec2d.h"

namespace planning {

/************** 障碍物分类相关定义 ************ */
// 障碍物当前时刻相对于自车的位置关系
enum class ObstacleRelPosType : uint8_t {
  FAR_AWAY = 0,  // 远离自车障碍物
  MID_FRONT,     // 自车正前方
  LEFT_FRONT,    // 自车左前方
  RIGHT_FRONT,   // 自车右前方
  LEFT_SIDE,     // 自车左侧并排
  RIGHT_SIDE,    // 自车右侧并排
  MID_BACK,      // 自车正后方
  LEFT_BACK,     // 自车左后方
  RIGHT_BACK,    // 自车右后方
};

enum class ObstacleMotionType : uint8_t {
  STATIC = 0,           // 静止车辆
  OPPOSITE_DIR_MOVING,  // 对向运动
  SAME_DIR_MOVING,      // 同向运动
  CROSSING_MOVING,      // 横穿运动
};

struct ObstacleClassificationResult {
  void Clear() {
    rel_pos_type_to_ids.clear();
    motion_type_to_ids.clear();
    id_to_rel_pos_type.clear();
    id_to_motion_type.clear();
  }

  std::unordered_map<ObstacleRelPosType, std::vector<int>> rel_pos_type_to_ids;
  std::unordered_map<ObstacleMotionType, std::vector<int>> motion_type_to_ids;
  std::unordered_map<int, ObstacleRelPosType> id_to_rel_pos_type;
  std::unordered_map<int, ObstacleMotionType> id_to_motion_type;
};

/************** 障碍物合并相关定义 ************ */
enum class ObstacleClusterType : uint8_t {
  NO_MERGE = 0,   // 不合并
  ABS_MERGE = 1,  // 绝对距离太近合并
  LAT_MERGE = 2,  // 横向距离太近合并
  LON_MERGE = 3   // 纵向距离太近合并
};
using ObstacleClusterMap = std::unordered_map<int, ObstacleClusterType>;
using ObstacleClusterGraph = std::unordered_map<int, ObstacleClusterMap>;

struct ObstacleClusterCandicate {
  int origin_id;
  ObstacleRelPosType rel_pos_types;  // 合并后的障碍物位置类型
  ObstacleMotionType motion_type;  // 合并后的障碍物运动类型
};

struct ObstacleCluster {
  // 【结构体初始化规范】：在此统一初始化极大极小值防合并错误
  ObstacleCluster() {
    frenet_boundary.l_start = std::numeric_limits<double>::max();
    frenet_boundary.l_end = std::numeric_limits<double>::lowest();
    frenet_boundary.s_start = std::numeric_limits<double>::max();
    frenet_boundary.s_end = std::numeric_limits<double>::lowest();
  }

  int cluster_id;
  std::vector<int> original_ids;
  std::unordered_set<ObstacleRelPosType> rel_pos_types;
  std::unordered_set<ObstacleMotionType> motion_types;
  std::unordered_set<ObstacleClusterType> cluster_types;
  FrenetObstacleBoundary frenet_boundary;
  planning_math::Box2d bounding_box;
  planning_math::Polygon2d polygon;
  std::vector<planning_math::Vec2d> perception_points;
};

struct ObstacleClusterContainer {
  std::unordered_map<int, int> obs_id_to_cluster_id;  // 原始ID -> 聚类ID
  std::vector<ObstacleCluster> obstacle_clusters;     // 聚类列表
};

struct HppObstacleLateralPreprocessDeciderOutput {
  ObstacleClassificationResult obs_classification_result;
  ObstacleClusterContainer obs_cluster_container;
};

}  // namespace planning