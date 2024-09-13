
#pragma once

#include <bits/stdint-intn.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "./../collision_detection/gjk2d_interface.h"
#include "./../occupancy_grid_map/point_cloud_obstacle.h"
#include "ad_common/math/line_segment2d.h"
#include "hybrid_astar_config.h"
#include "log_glog.h"
#include "node2d.h"
#include "node3d.h"
#include "polygon_base.h"
#include "pose2d.h"

namespace planning {

class GridSearch {
 public:
  explicit GridSearch(const PlannerOpenSpaceConfig& open_space_conf);

  ~GridSearch() = default;

  bool GenerateDpMap(const double ex, const double ey, const MapBound& XYbounds,
                     const ParkObstacleList* obstacles,
                     const double veh_half_width_with_safe_dist);

  double CheckDpMap(const double sx, const double sy);

  void Init();

 private:
  double EuclidDistance(const double x1, const double y1, const double x2,
                        const double y2);

  void GenerateNextNodes(Node2dChildSet* next_nodes, Node2d* node);

  bool CheckConstraints(Node2d* node);

  int32_t GenerateGlobalId();

  const bool IsPointInMapBound(const double x, const double y);

  void ResetNodePool();

  double GetNodeGCost(const size_t x_id, const size_t y_id) {
    return node_pool_[x_id][y_id].GetCost();
  }

  bool NodeIndexValid(const Node2dIndex& id);

  Node2d* GetNodeFromPool(const Node2dIndex& id);

  void NodePoolPush(Node2d* node);

  void DebugNodePool();

  bool CheckCollisionByObsProjection(Node2d* node);

  void ProjectObstacleToNodeMap();

 private:
  double xy_grid_resolution_ = 0.0;
  double inv_xy_resolution_ = 0.0;
  double xy_grid_resolution_half_ = 0.0;
  double safe_width_ = 0.0;
  double veh_half_width_with_safe_dist_ = 0.0;

  MapBound XYbounds_;

  int32_t max_grid_x_ = 0.0;
  int32_t max_grid_y_ = 0.0;

  Node2d start_node_;
  Node2d end_node_;
  Node2d final_node_;
  const ParkObstacleList* obstacles_;

  Polygon2D grid_box_;

  int32_t max_x_search_size_;
  int32_t max_y_search_size_;

  Node2dMatrix node_pool_;

  std::multimap<double, Node2d*> open_set_;

  int32_t global_idx_;

  GJK2DInterface gjk_interface_;
};

}  // namespace planning
