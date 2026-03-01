#pragma once

#include <memory>
#include <unordered_map>
#include <vector>
#include "modules/context/obstacle.h"
#include "modules/context/frenet_obstacle.h"
#include "modules/context/reference_path.h"
#include "modules/context/obstacle_manager.h"

namespace planning {
  // FrenetObstacle 中包含 Obstacle 的指针
using ObstacleItemMap = std::unordered_map<int, FrenetObstaclePtr>;

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

// 障碍物当前时刻运动关系
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
enum class ObstacleMergeType : uint8_t {
  NO_MERGE = 0,   // 不合并
  ABS_MERGE = 1,  // 绝对距离太近合并
  LAT_MERGE = 2,  // 横向距离太近合并
  LON_MERGE = 3   // 纵向距离太近合并
};
using ObstacleMergeMap = std::unordered_map<int, ObstacleMergeType>;
using ObstacleMergeGraph = std::unordered_map<int, ObstacleMergeMap>;

struct MergedObstacleCandicate {
  int origin_id;
  ObstacleRelPosType rel_pos_types;  // 合并后的障碍物位置类型
  ObstacleMotionType motion_type;    // 合并后的障碍物运动类型(默认只合并静止障碍物)
};

struct MergedObstacleResult {
  int merged_id;                                  // 聚类Cluster的ID
  std::vector<int> original_ids;                  // 原始障碍物 ID 列表
  std::unordered_set<ObstacleRelPosType> rel_pos_types;  // 合并后的障碍物位置类型
  std::unordered_set<ObstacleMotionType> motion_types;   // 合并后的障碍物运动类型
  std::unordered_set<ObstacleMergeType> merge_types;   // 合并后的障碍物运动类型
  FrenetObstacleBoundary frenet_boundary;     // SL boundary
  planning_math::Box2d bounding_box;         // XY Box
  planning_math::Polygon2d polygon;           // XY Polygon
  std::vector<planning_math::Vec2d> perception_points; // XY Point
};

struct MergedObstacleContainer {
  std::unordered_map<int, int> obs_id_to_merged_id;  // 原始ID -> 聚类ID
  std::vector<MergedObstacleResult> merged_obstacles;  // 聚类列表
};

/************** 障碍物处理操作类定义 ************ */
class HppLateralObstacleUtils {
 public:
  HppLateralObstacleUtils() = delete;

  // 1. 障碍物过滤
  static bool GenerateObstaclesToBeConsidered(
      ConstReferencePathPtr reference_path_ptr, ObstacleItemMap& obs_item_map);

  // 2. 障碍物分类
  static bool ClassifyObstacles(
      const ObstacleItemMap& obs_item_map, const FrenetEgoState& ego_state,
      ObstacleClassificationResult& classification_result);

  // 3: 聚类 (动静分离 + 规则聚类 + 凸包生成)
  static bool MergeObstaclesBaseOnPos(
      const ObstacleItemMap& obs_item_map,
      const ObstacleClassificationResult& classification_result,
      MergedObstacleContainer& merged_obs_container);

 private:
  /************** 障碍物分类相关私有函数定义 ************ */
  static ObstacleRelPosType ClassifyObstaclesByRelPos(
      const FrenetEgoState& ego_state, const FrenetObstaclePtr& obs_ptr);

  static ObstacleMotionType ClassifyObstaclesByMotion(
      const FrenetObstaclePtr& obs_ptr);

  /************** 障碍物合并相关私有函数定义 ************ */
  static bool GenerateMergeCandicates(
      const ObstacleItemMap& obs_item_map,
      const ObstacleClassificationResult& classification_result,
      std::vector<MergedObstacleCandicate>& merge_candidates);
  static bool CalculateCandidateMergeGraph(
      const ObstacleItemMap& obs_item_map,
      const std::vector<MergedObstacleCandicate>& merge_candidates,
      ObstacleMergeGraph& merge_graph);

  static bool DFSGenerateMergedObstacles(
      const std::vector<MergedObstacleCandicate>& merge_candidates,
      const ObstacleMergeGraph& merge_graph,
      const int curr_idx,
      const ObstacleMergeType merge_type,
      std::unordered_set<int>& visited_candidate_idxs,
      MergedObstacleResult& merged_result);

  static bool BuildMergedObstacleConvexHull(
      const ObstacleItemMap& obs_item_map, MergedObstacleResult& merged_result);
};

}  // namespace planning