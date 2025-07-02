
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "src/library/convex_collision_detection/gjk2d_interface.h"
#include "src/library/occupancy_grid_map/point_cloud_obstacle.h"
#include "ad_common/math/line_segment2d.h"
#include "hybrid_astar_config.h"
#include "log_glog.h"
#include "node2d.h"
#include "node3d.h"
#include "polygon_base.h"
#include "pose2d.h"

namespace planning {

#define DP_MAX_X_SEARCH_SIZE (100)
#define DP_MAX_Y_SEARCH_SIZE (150)
#define DP_NODE_LAYER_MAX_NUMBER (600)

enum class NodeLayerTag {
  NONE,
  PARENT_LAYER,
  CHILD_LAYER,
  MAX_NUMBER,
};

// 在搜索的过程中，为了减少删除或者拷贝操作，通过标签来确认搜索层是父节点层还是子节点层
struct NodeLayer {
  Node2d* node_layer[DP_NODE_LAYER_MAX_NUMBER];
  int32_t size;
  NodeLayerTag tag;

  void AddNode(Node2d* node) {
    node_layer[size] = node;
    size++;

    return;
  }

  void Clear() {
    size = 0;

    return;
  }

  const int32_t GetSize() const { return size; }
};

class GridSearch {
 public:
  GridSearch() = default;
  GridSearch(const PlannerOpenSpaceConfig& open_space_conf);

  ~GridSearch() = default;

  bool GenerateDpMap(const float ex, const float ey, const MapBound& XYbounds,
                     const ParkObstacleList* obstacles);

  float CheckDpMap(const float sx, const float sy);

  void Init(const PlannerOpenSpaceConfig& open_space_conf);

 private:
  float EuclidDistance(const float x1, const float y1, const float x2,
                        const float y2);

  void GenerateNextNodes(Node2dChildSet* next_nodes, Node2d* node);

  bool CheckConstraints(Node2d* node);

  int32_t GenerateGlobalId();

  const bool IsPointInMapBound(const float x, const float y);

  void ResetNodePool();

  float GetNodeGCost(const size_t x_id, const size_t y_id) {
    return node_pool_[x_id][y_id].GetCost();
  }

  bool NodeIndexValid(const Node2dIndex& id);

  Node2d* GetNodeFromPool(const Node2dIndex& id);

  void NodePoolPush(Node2d* node);

  void DebugNodePool();

  bool CheckCollisionByObsProjection(Node2d* node);

  void ProjectObstacleToNodeMap();

  bool NodePositionValid(const float x, const float y);

 private:
  float heuristic_grid_resolution_ = 0.0;
  float inv_xy_resolution_ = 0.0;
  float xy_grid_resolution_half_ = 0.0;
  float safe_width_ = 0.0;

  MapBound XYbounds_;

  int32_t max_grid_x_ = 0.0;
  int32_t max_grid_y_ = 0.0;

  Node2d start_node_;
  Node2d end_node_;
  const ParkObstacleList* obstacles_;

  Polygon2D grid_box_;

  Node2d node_pool_[DP_MAX_X_SEARCH_SIZE][DP_MAX_Y_SEARCH_SIZE];

  GJK2DInterface gjk_interface_;
};

}  // namespace planning
