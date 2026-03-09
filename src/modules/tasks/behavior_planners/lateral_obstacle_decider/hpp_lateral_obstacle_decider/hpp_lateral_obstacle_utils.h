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

  struct MergedObstacleInfo {
    int merged_obs_id;
    std::vector<int> obstacle_ids;
    planning_math::Box2d box;
    planning_math::Polygon2d polygon;
    FrenetObstacleBoundary sl_boundary;
    std::vector<planning_math::Vec2d> sl_corner_points;
  };

  enum class ObstacleClassificationType:uint8_t {
    FRONT_STATIC_OBS = 0,       //前方静止障碍物
    SIDE_STATIC_OBS = 1,        //侧方静止障碍物
    OPPOSITE_MOVING_OBS = 2,    //对向行驶障碍物
    FRONT_SAME_MOVING_OBS = 3,  //前方同向行驶障碍物
    BACK_SAME_MOVING_OBS = 4,   //后方同向行驶障碍物
  };

  struct MergedObstacleContainer {
    std::unordered_map<int, int> obs_id_to_merged_obs_id;
    std::vector<MergedObstacleInfo> merged_obstacles;     // 合并后的障碍物尺寸信息

  };

  struct ObstacleClassificationResult {
    std::unordered_map<ObstacleClassificationType, std::vector<int>> type_to_merged_obs_list;
    std::unordered_map<int, ObstacleClassificationType> merged_obs_id_to_type;
  };

  class HppLateralObstacleUtils {
   public:
    // 障碍物过滤，并转换成 ObstacleItemMap 格式
    static bool GenerateObstaclesToBeConsidered(
        ConstObstacleManagerPtr obstacle_manager_ptr,
        ConstReferencePathPtr reference_path_ptr,
        ObstacleItemMap& obs_item_map);

    // 基于障碍物位置关系合并障碍物
    static bool MergeObstaclesBaseOnPos(
        const ObstacleItemMap& obs_item_map,
        MergedObstacleContainer& merged_obs_constainer);

    // 基于障碍物位置和运动属性，对障碍物分类
    static bool ClassifyObstacles(
        const MergedObstacleContainer& merged_obs_constainer,
        ObstacleClassificationResult& obs_classification_result);

    // 对运动障碍物进行避让（会车/被超车）、跟车、超车
    static bool MakeDecisionForMovingObstacles(
        const MergedObstacleContainer& merged_obs_constainer,
        const ObstacleClassificationResult&
            obs_classification_result /*TODO: 输出数据结构待定*/);

    // 基于障碍物在 frenet 坐标系下的 start_s 进行增序排序
    static bool SortMergedObstacles(
        MergedObstacleContainer& merged_obs_contanier);
  };
}  // namespace planning