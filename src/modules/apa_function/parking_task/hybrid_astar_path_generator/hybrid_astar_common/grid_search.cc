#include "grid_search.h"

#include <cstddef>
#include <utility>
#include <vector>

namespace planning {
namespace apa_planner {

const float kMaxNodeCost = 100000.0;

#define DEBUG_NODE_COST (0)

GridSearch::GridSearch(
    const PlannerOpenSpaceConfig& config,
    const std::shared_ptr<CollisionDetectorInterface>& col_det_interface_ptr) {
  config_ = config;
  col_det_interface_ptr_ = col_det_interface_ptr;
}

void GridSearch::Init() {
  heuristic_grid_resolution_ = config_.heuristic_grid_resolution;
  inv_xy_resolution_ = 1.0 / heuristic_grid_resolution_;
  xy_grid_resolution_half_ = heuristic_grid_resolution_ * 0.5;

  return;
}

const float GridSearch::EuclidDistance(const float x1, const float y1,
                                       const float x2, const float y2) {
  return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

const bool GridSearch::CheckNodeIndexValid(const Node2dIndex& id) {
  if (id.x < 0 || id.x >= GS_MAX_X_SEARCH_SIZE || id.y < 0 ||
      id.y >= GS_MAX_Y_SEARCH_SIZE) {
    return false;
  }

  return true;
}

const bool GridSearch::CheckNodePosValid(const float x, const float y) {
  if (x < XYbounds_.x_min || x > XYbounds_.x_max || y < XYbounds_.y_min ||
      y > XYbounds_.y_max) {
    return false;
  }

  return true;
}

void GridSearch::GenerateNextNodes(Node2dChildSet* next_nodes,
                                   Node2d* current_node) {
  next_nodes->size = 0;
  const int current_node_x = current_node->GetGridX();
  const int current_node_y = current_node->GetGridY();

  const float current_node_path_cost = current_node->GetCost();
  const float diagonal_distance = 1.414;
  const float edge_distance = 1.0;

  std::vector<GridMotion> motion_vec;
  motion_vec.reserve(8);
  motion_vec.emplace_back(GridMotion(0, 1, edge_distance));
  motion_vec.emplace_back(GridMotion(1, 1, diagonal_distance));
  motion_vec.emplace_back(GridMotion(1, 0, edge_distance));
  motion_vec.emplace_back(GridMotion(1, -1, diagonal_distance));
  motion_vec.emplace_back(GridMotion(0, -1, edge_distance));
  motion_vec.emplace_back(GridMotion(-1, -1, diagonal_distance));
  motion_vec.emplace_back(GridMotion(-1, 0, edge_distance));
  motion_vec.emplace_back(GridMotion(-1, 1, diagonal_distance));

  // 这里的cost指到起点的距离
  for (const GridMotion& motion : motion_vec) {
    Node2d node(current_node_x + motion.move_x, current_node_y + motion.move_y,
                XYbounds_, heuristic_grid_resolution_);
    node.SetCost(current_node_path_cost +
                 motion.move_dist * heuristic_grid_resolution_);

    if (CheckNodeIndexValid(node.GetGridIndex()) &&
        CheckNodePosValid(node.GetX(), node.GetY())) {
      next_nodes->nodes[next_nodes->size] = node;
      next_nodes->size++;
    }
  }
}

Node2d* GridSearch::GetNodeFromPool(const Node2dIndex& id) {
  if (!CheckNodeIndexValid(id)) {
    ILOG_INFO << "get node from pool fail" << id.x << " " << id.y;
    return nullptr;
  }

  return &node_pool_[id.x][id.y];
}

const bool GridSearch::NodePoolPush(Node2d* node) {
  if (!CheckNodeIndexValid(node->GetGridIndex())) {
    ILOG_INFO << "size too big";
    return false;
  }

  node_pool_[node->GetGridX()][node->GetGridY()] = *node;

  return true;
}

void GridSearch::DebugNodePool() {
  for (int32_t i = 0; i < GS_MAX_X_SEARCH_SIZE; i++) {
    for (int32_t k = 0; k < GS_MAX_Y_SEARCH_SIZE; k++) {
      node_pool_[i][k].DebugNodeString();
    }
  }

  cv::Mat map_matrix(GS_MAX_X_SEARCH_SIZE, GS_MAX_Y_SEARCH_SIZE, CV_8UC1,
                     cv::Scalar(200));

  int row_num = map_matrix.rows;
  int column_num = map_matrix.cols;

  float value = 0;

  for (int32_t i = 0; i < row_num; i++) {
    uchar* data = map_matrix.ptr<uchar>(i);

    for (int32_t j = 0; j < column_num; j++) {
      value = node_pool_[i][j].GetCost();
      value = value / 50.0 * 255;

      if (value < 0.0) {
        value = 0.0;
      }

      if (value > 255.0) {
        value = 255.0;
      }

      data[j] = std::round(value);
    }
  }

  cv::imwrite("/asw/planning/glog/dp_cost.png", map_matrix);

  return;
}

void GridSearch::ProjectObstacleToNodeMap() {
  Node2d* node = nullptr;
  Node2dIndex index;

  if (col_det_interface_ptr_ == nullptr) {
    return;
  }

  const std::unordered_map<size_t, ApaObstacle>& obs =
      col_det_interface_ptr_->GetObsManagerPtr()->GetObstacles();

  for (const auto& pair : obs) {
    for (const Eigen::Vector2d& pt : pair.second.GetPtClout2dLocal()) {
      if (!CheckNodePosValid(pt.x(), pt.y())) {
        continue;
      }

      index.x = std::round((pt.x() - XYbounds_.x_min) * inv_xy_resolution_);
      index.y = std::round((pt.y() - XYbounds_.y_min) * inv_xy_resolution_);

      node = GetNodeFromPool(index);

      if (node == nullptr) {
        continue;
      }

      node->SetCollision(true);
    }
  }
}

void GridSearch::ResetNodePool() {
  int32_t x_bound_size = max_grid_x_ + 1;
  int32_t y_bound_size = max_grid_y_ + 1;

  x_bound_size = std::min(GS_MAX_X_SEARCH_SIZE, x_bound_size);
  y_bound_size = std::min(GS_MAX_Y_SEARCH_SIZE, y_bound_size);

  // ILOG_INFO << "x size = " << x_bound_size << ", y size = " << y_bound_size;

  for (int32_t i = 0; i < x_bound_size; i++) {
    for (int32_t j = 0; j < y_bound_size; j++) {
      node_pool_[i][j].Clear();
    }
  }

  return;
}

const float GridSearch::CheckDpMap(const float sx, const float sy) {
  Node2dIndex index = Node2d::CalcIndex(sx, sy, inv_xy_resolution_, XYbounds_);

  Node2d* node = GetNodeFromPool(index);

  if (node == nullptr) {
    return kMaxNodeCost;
  }

  return node->GetCost();
}

const bool GridSearch::GenerateDpMap(const float ex, const float ey,
                                     const MapBound& XYbounds) {
  XYbounds_ = XYbounds;
  max_grid_y_ =
      std::round((XYbounds_.y_max - XYbounds_.y_min) * inv_xy_resolution_);
  max_grid_x_ =
      std::round((XYbounds_.x_max - XYbounds_.x_min) * inv_xy_resolution_);

  ResetNodePool();

  NodeLayer node_layer1, node_layer2;
  NodeLayer *parent_layer = nullptr, *child_layer = nullptr;

  ProjectObstacleToNodeMap();

  end_node_.Set(ex, ey, inv_xy_resolution_, XYbounds_);
  if (!NodePoolPush(&end_node_)) {
    return false;
  }

  node_layer1.AddNode(&end_node_);
  node_layer1.tag = NodeLayerTag::PARENT_LAYER;
  parent_layer = &node_layer1;

  node_layer2.tag = NodeLayerTag::CHILD_LAYER;
  child_layer = &node_layer2;

  Node2d *current_node = nullptr, *node_in_pool = nullptr, *next_node = nullptr;
  Node2dChildSet next_nodes;
  std::unordered_set<int> child_layer_hash_table;

  int explored_node_num = 0;

  while (true) {
    if (parent_layer->tag == NodeLayerTag::PARENT_LAYER &&
        parent_layer->GetSize() <= 0) {
      break;
    }

#if DEBUG_NODE_COST
    ILOG_INFO << "parent layer size=" << parent_layer->GetSize()
              << ",child layer size=" << child_layer->GetSize();
#endif

    child_layer_hash_table.clear();

    for (int32_t i = 0; i < parent_layer->GetSize(); i++) {
      current_node = parent_layer->node_layer[i];

      GenerateNextNodes(&next_nodes, current_node);

      for (int j = 0; j < next_nodes.size; j++) {
        explored_node_num++;

        next_node = &next_nodes.nodes[j];

        node_in_pool = GetNodeFromPool(next_node->GetGridIndex());

        if (node_in_pool->IsCollision()) {
          continue;
        }

        if (next_node->GetCost() >= node_in_pool->GetCost()) {
          continue;
        }

        // next node cost is smaller than node in pool
        node_in_pool->CopyFrom(*next_node);

        if (child_layer_hash_table.find(node_in_pool->GetGlobalID()) ==
            child_layer_hash_table.end()) {
          child_layer->AddNode(node_in_pool);
          child_layer_hash_table.insert(node_in_pool->GetGlobalID());
        }
      }
    }

    if (node_layer1.tag == NodeLayerTag::PARENT_LAYER) {
      parent_layer = &node_layer2;
      parent_layer->tag = NodeLayerTag::PARENT_LAYER;
      child_layer = &node_layer1;
      child_layer->tag = NodeLayerTag::CHILD_LAYER;
    } else {
      parent_layer = &node_layer1;
      parent_layer->tag = NodeLayerTag::PARENT_LAYER;
      child_layer = &node_layer2;
      child_layer->tag = NodeLayerTag::CHILD_LAYER;
    }

    child_layer->Clear();
  }

#if DEBUG_NODE_COST
  end_timestamp = IflyTime::Now_ms();
  time_consumption = end_timestamp - start_timestamp;

  ILOG_INFO << "search time = " << time_consumption;
  ILOG_INFO << "heuristic search explored node num is " << explored_node_num;

  DebugNodePool();

#endif

  return true;
}

}  // namespace apa_planner
}  // namespace planning