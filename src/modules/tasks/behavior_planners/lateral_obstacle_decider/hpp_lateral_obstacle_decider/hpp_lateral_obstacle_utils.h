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
  using ConstReferencePathPtr = std::shared_ptr<const ReferencePath>;

  enum class ObstacleTypeBeforeCluster : uint8_t {
    IGNORE = 0,             // 需过滤的障碍物 (如后方对向、同向高速等)
    FRONT_STATIC,           // 前方静止
    SIDE_REAR_STATIC,       // 并排/后方静止
    OPPOSITE_MOVING,        // 对向行驶 (前方)
    FRONT_SAME_MOVING,      // 同向前方 (正常速度)
    BACK_SAME_MOVING        // 同向后方 (正常/快速)
  };

  struct MergedObstacleInfo {
    int merged_id;                  // 聚类Cluster的ID
    std::vector<int> original_ids;  // 原始障碍物 ID 列表
    ObstacleTypeBeforeCluster type = ObstacleTypeBeforeCluster::IGNORE; // 障碍物类型

    // 几何边界
    double s_start = std::numeric_limits<double>::infinity();
    double s_end = -std::numeric_limits<double>::infinity();
    double l_start = std::numeric_limits<double>::infinity();
    double l_end = -std::numeric_limits<double>::infinity();

    // 凸包多边形
    planning_math::Polygon2d polygon;
    std::vector<planning_math::Vec2d> raw_points;

    double center_l() const { return (l_start + l_end) / 2.0; }
    double center_s() const { return (s_start + s_end) / 2.0; }
  };

  struct MergedObstacleContainer {
    std::unordered_map<int, int> obs_id_to_merged_id; // 原始ID -> 聚类ID
    std::vector<MergedObstacleInfo> merged_obstacles; // 聚类列表
  };

  class HppLateralObstacleUtils {
   public:
    HppLateralObstacleUtils() = delete;

    // 1. 障碍物过滤
    static bool GenerateObstaclesToBeConsidered(
          ConstReferencePathPtr reference_path_ptr,
          ObstacleItemMap& obs_item_map);

    // 2. 障碍物分类
    static std::unordered_map<int, ObstacleTypeBeforeCluster> ClassifyObstacles(
          const ObstacleItemMap& obs_item_map,
          const double ego_s,
          const double ego_v);

    // 3: 聚类 (动静分离 + 规则聚类 + 凸包生成)
    static bool MergeObstaclesBaseOnPos(
        const ObstacleItemMap& obs_item_map,
        const std::unordered_map<int, ObstacleTypeBeforeCluster>& classification_map,
        MergedObstacleContainer& merged_obs_container);

   private:
    // 内部辅助：构建凸包
    static void BuildConvexHull(MergedObstacleInfo& cluster);
  };

} // namespace planning