#pragma once

#include <bits/stdint-uintn.h>

#include <memory>

#include "collision_detector_interface.h"
#include "hybrid_astar_config.h"
#include "node2d.h"
namespace planning {
namespace apa_planner {

#define GS_MAX_X_SEARCH_SIZE (100)
#define GS_MAX_Y_SEARCH_SIZE (150)
#define GS_NODE_LAYER_MAX_NUMBER (600)

extern const float kMaxNodeCost;

enum class NodeLayerTag : uint8_t {
  NONE,
  PARENT_LAYER,
  CHILD_LAYER,
  MAX_NUMBER,
};

struct GridMotion {
  int move_x;
  int move_y;
  float move_dist;
  GridMotion() = default;
  GridMotion(const int x, const int y, const float dist)
      : move_x(x), move_y(y), move_dist(dist) {}
  ~GridMotion() = default;
};

struct NodeLayer {
  Node2d* node_layer[GS_NODE_LAYER_MAX_NUMBER];
  int32_t size = 0;
  NodeLayerTag tag;

  void AddNode(Node2d* node) {
    if (size >= GS_NODE_LAYER_MAX_NUMBER) {
      return;
    }
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
  GridSearch(
      const PlannerOpenSpaceConfig& config,
      const std::shared_ptr<CollisionDetectorInterface>& col_det_interface_ptr);

  ~GridSearch() = default;

  void Init();

  void UpdateConfig(const PlannerOpenSpaceConfig& config) {
    config_ = config;
    Init();
  }

  void SetCollisionDetectorIntefacePtr(
      const std::shared_ptr<CollisionDetectorInterface>&
          col_det_interface_ptr) {
    col_det_interface_ptr_ = col_det_interface_ptr;
  }

  const bool GenerateDpMap(const float ex, const float ey,
                           const MapBound& XYbounds);

  const float CheckDpMap(const float sx, const float sy);

 private:
  const float EuclidDistance(const float x1, const float y1, const float x2,
                             const float y2);

  void GenerateNextNodes(Node2dChildSet* next_nodes, Node2d* current_node);

  const bool CheckNodeIndexValid(const Node2dIndex& id);

  const bool CheckNodePosValid(const float x, const float y);

  void ResetNodePool();

  void ProjectObstacleToNodeMap();

  Node2d* GetNodeFromPool(const Node2dIndex& id);

  const bool NodePoolPush(Node2d* node);

  void DebugNodePool();

 private:
  PlannerOpenSpaceConfig config_;
  std::shared_ptr<CollisionDetectorInterface> col_det_interface_ptr_;

  float heuristic_grid_resolution_ = 0.0;
  float inv_xy_resolution_ = 0.0;
  float xy_grid_resolution_half_ = 0.0;

  MapBound XYbounds_;

  int32_t max_grid_x_ = 0.0;
  int32_t max_grid_y_ = 0.0;

  Node2d start_node_;
  Node2d end_node_;

  Node2d node_pool_[GS_MAX_X_SEARCH_SIZE][GS_MAX_Y_SEARCH_SIZE];
};

}  // namespace apa_planner
}  // namespace planning