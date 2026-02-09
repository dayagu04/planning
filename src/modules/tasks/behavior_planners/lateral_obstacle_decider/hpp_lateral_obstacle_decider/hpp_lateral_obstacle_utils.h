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
  using ConstObstacleManagerPtr = std::shared_ptr<const ObstacleManager>;
  using ConstReferencePathPtr = std::shared_ptr<const ReferencePath>;

  struct MergedObstacleInfo {
    int merged_id;                  // 聚类 ID (从0开始迭代)
    std::vector<int> original_ids;  // 原始障碍物 ID 列表
    bool is_static = false;

    // 如果两个 Cluster 需要保持决策一致，它们的 consistency_group_id 将会被设为相同。
    int consistency_group_id = -1;

    // 几何边界
    double s_start = std::numeric_limits<double>::infinity();
    double s_end = -std::numeric_limits<double>::infinity();
    double l_start = std::numeric_limits<double>::infinity();
    double l_end = -std::numeric_limits<double>::infinity();

    // 运动属性
    double v_s = 0.0;
    double v_l = 0.0;

    double center_l() const { return (l_start + l_end) / 2.0; }
    double center_s() const { return (s_start + s_end) / 2.0; }
  };

  enum class ObstacleClassificationType : uint8_t {
    FRONT_STATIC_OBS = 0,       // 前方静止
    SIDE_STATIC_OBS = 1,        // 并排/后方静止
    OPPOSITE_MOVING_OBS = 2,    // 对向行驶
    FRONT_SAME_MOVING_OBS = 3,  // 同向前方
    BACK_SAME_MOVING_OBS = 4,   // 同向后方
    UNKNOWN = 5
  };

  struct MergedObstacleContainer {
    // 原始 ID -> 聚类 ID 的映射
    std::unordered_map<int, int> obs_id_to_merged_id;
    // 聚类列表 (存储具体的 Cluster 信息)
    std::vector<MergedObstacleInfo> merged_obstacles;
  };

  struct ObstacleClassificationResult {
    std::vector<const MergedObstacleInfo*> front_static_obs;      // 前方静止
    std::vector<const MergedObstacleInfo*> side_static_obs;       // 侧方/后方静止
    std::vector<const MergedObstacleInfo*> opposite_moving_obs;   // 对向
    std::vector<const MergedObstacleInfo*> front_same_moving_obs; // 同向前方
    std::vector<const MergedObstacleInfo*> back_same_moving_obs;  // 同向后方
    void Clear() {
      front_static_obs.clear();
      side_static_obs.clear();
      opposite_moving_obs.clear();
      front_same_moving_obs.clear();
      back_same_moving_obs.clear();
  }
  };

  class HppLateralObstacleUtils {
   public:
    HppLateralObstacleUtils() = delete;

    // 1. 障碍物过滤
    static bool GenerateObstaclesToBeConsidered(
        ConstReferencePathPtr reference_path_ptr,
        ObstacleItemMap& obs_item_map);

    // 2. 物理聚类 (Merge并生成 0,1,2... 编号)
    static bool MergeObstaclesBaseOnPos(
        const ObstacleItemMap& obs_item_map,
        MergedObstacleContainer& merged_obs_container);

    // 3. 排序
    static bool SortMergedObstacles(
        MergedObstacleContainer& merged_obs_container);

    // 4. 分类
    static bool ClassifyObstacles(
      const MergedObstacleContainer& merged_obs_container,
      const double ego_s,
      const double ego_v,
      ObstacleClassificationResult& obs_classification_result);

    // 5. 静态障碍物一致性处理
    static void UpdateStaticObsConsistency(
        MergedObstacleContainer& merged_obs_container,
        const ObstacleClassificationResult& classification_result,
        const double ego_head_l);

    // 6. 动态障碍物处理 (预留)
    static bool MakeDecisionForMovingObstacles(
        const MergedObstacleContainer& merged_obs_container,
        const ObstacleClassificationResult& obs_classification_result);
  };

} // namespace planning