
#include "dynamic_programing_cost.h"

#include <opencv2/imgproc/types_c.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "log_glog.h"
#include "node2d.h"
#include "polygon_base.h"
#include "src/common/ifly_time.h"

namespace planning {

#define DEBUG_NODE_COST (0)

GridSearch::GridSearch(const PlannerOpenSpaceConfig& open_space_conf) {
  heuristic_grid_resolution_ = open_space_conf.heuristic_grid_resolution;
  inv_xy_resolution_ = 1.0 / heuristic_grid_resolution_;
  xy_grid_resolution_half_ = heuristic_grid_resolution_ / 2.0;
  safe_width_ = open_space_conf.heuristic_safe_dist;
}

float GridSearch::EuclidDistance(const float x1, const float y1,
                                  const float x2, const float y2) {
  return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

bool GridSearch::CheckConstraints(Node2d* node) {
  const int node_grid_x = node->GetGridX();
  const int node_grid_y = node->GetGridY();

  if (node_grid_x > max_grid_x_ || node_grid_x < 0 ||
      node_grid_y > max_grid_y_ || node_grid_y < 0) {
    return false;
  }

  if (obstacles_->IsEmpty()) {
    return true;
  }

  const float x = node->GetX();
  const float y = node->GetY();
  bool is_collision;

  GenerateRectPolygon(
      &grid_box_, x - xy_grid_resolution_half_, y - xy_grid_resolution_half_,
      x + xy_grid_resolution_half_, y + xy_grid_resolution_half_);

  for (const auto& obstacle : obstacles_->virtual_obs) {
    gjk_interface_.PolygonPointCollisionDetect(&is_collision, &grid_box_,
                                               obstacle);

    if (is_collision) {
      return false;
    }
  }

  for (const auto& obstacle : obstacles_->point_cloud_list) {
    // envelop box check
    gjk_interface_.PolygonCollisionByCircleCheck(
        &is_collision, &obstacle.envelop_polygon, &grid_box_, 0.01);

    if (!is_collision) {
      continue;
    }

    // internal points
    for (size_t j = 0; j < obstacle.points.size(); j++) {
      gjk_interface_.PolygonPointCollisionDetect(&is_collision, &grid_box_,
                                                 obstacle.points[j]);

      if (is_collision) {
        return false;
      }
    }
  }

  return true;
}

bool GridSearch::CheckCollisionByObsProjection(Node2d* node) {
  const int node_grid_x = node->GetGridX();
  const int node_grid_y = node->GetGridY();

  if (node_grid_x > max_grid_x_ || node_grid_x < 0 ||
      node_grid_y > max_grid_y_ || node_grid_y < 0) {
    return false;
  }

  return node->IsCollision();
}

void GridSearch::GenerateNextNodes(Node2dChildSet* next_nodes,
                                   Node2d* current_node) {
  int current_node_x = current_node->GetGridX();
  int current_node_y = current_node->GetGridY();

  float current_node_path_cost = current_node->GetCost();
  float diagonal_distance = 1.414;
  float edge_distance = 1.0;

  Node2d up = Node2d(current_node_x, current_node_y + 1, XYbounds_,
                     heuristic_grid_resolution_);
  up.SetCost(current_node_path_cost + edge_distance);

  Node2d up_right = Node2d(current_node_x + 1, current_node_y + 1, XYbounds_,
                           heuristic_grid_resolution_);
  up_right.SetCost(current_node_path_cost + diagonal_distance);

  Node2d right = Node2d(current_node_x + 1, current_node_y, XYbounds_,
                        heuristic_grid_resolution_);
  right.SetCost(current_node_path_cost + edge_distance);

  Node2d down_right = Node2d(current_node_x + 1, current_node_y - 1, XYbounds_,
                             heuristic_grid_resolution_);
  down_right.SetCost(current_node_path_cost + diagonal_distance);

  Node2d down = Node2d(current_node_x, current_node_y - 1, XYbounds_,
                       heuristic_grid_resolution_);
  down.SetCost(current_node_path_cost + edge_distance);

  Node2d down_left = Node2d(current_node_x - 1, current_node_y - 1, XYbounds_,
                            heuristic_grid_resolution_);
  down_left.SetCost(current_node_path_cost + diagonal_distance);

  Node2d left = Node2d(current_node_x - 1, current_node_y, XYbounds_,
                       heuristic_grid_resolution_);
  left.SetCost(current_node_path_cost + edge_distance);

  Node2d up_left = Node2d(current_node_x - 1, current_node_y + 1, XYbounds_,
                          heuristic_grid_resolution_);
  up_left.SetCost(current_node_path_cost + diagonal_distance);

  if (NodeIndexValid(up.GetGridIndex()) &&
      NodePositionValid(up.GetX(), up.GetY())) {
    next_nodes->nodes[next_nodes->size] = up;
    next_nodes->size++;
  }

  if (NodeIndexValid(up_right.GetGridIndex()) &&
      NodePositionValid(up_right.GetX(), up_right.GetY())) {
    next_nodes->nodes[next_nodes->size] = up_right;
    next_nodes->size++;
  }

  if (NodeIndexValid(right.GetGridIndex()) &&
      NodePositionValid(right.GetX(), right.GetY())) {
    next_nodes->nodes[next_nodes->size] = right;
    next_nodes->size++;
  }

  if (NodeIndexValid(down_right.GetGridIndex()) &&
      NodePositionValid(down_right.GetX(), down_right.GetY())) {
    next_nodes->nodes[next_nodes->size] = down_right;
    next_nodes->size++;
  }

  if (NodeIndexValid(down.GetGridIndex()) &&
      NodePositionValid(down.GetX(), down.GetY())) {
    next_nodes->nodes[next_nodes->size] = down;
    next_nodes->size++;
  }

  if (NodeIndexValid(down_left.GetGridIndex()) &&
      NodePositionValid(down_left.GetX(), down_left.GetY())) {
    next_nodes->nodes[next_nodes->size] = down_left;
    next_nodes->size++;
  }

  if (NodeIndexValid(left.GetGridIndex()) &&
      NodePositionValid(left.GetX(), left.GetY())) {
    next_nodes->nodes[next_nodes->size] = left;
    next_nodes->size++;
  }

  if (NodeIndexValid(up_left.GetGridIndex()) &&
      NodePositionValid(up_left.GetX(), up_left.GetY())) {
    next_nodes->nodes[next_nodes->size] = up_left;
    next_nodes->size++;
  }

  return;
}

bool GridSearch::GenerateDpMap(const float ex, const float ey,
                               const MapBound& XYbounds,
                               const ParkObstacleList* obstacles,
                               const float veh_half_width_with_safe_dist) {
// init
#if DEBUG_NODE_COST
  double start_timestamp = IflyTime::Now_ms();
#endif

  // XYbounds with xmin, xmax, ymin, ymax
  XYbounds_ = XYbounds;
  veh_half_width_with_safe_dist_ = veh_half_width_with_safe_dist;
  max_grid_y_ =
      std::round((XYbounds_.y_max - XYbounds_.y_min) * inv_xy_resolution_);
  max_grid_x_ =
      std::round((XYbounds_.x_max - XYbounds_.x_min) * inv_xy_resolution_);

  ResetNodePool();
  NodeLayer node_layer1;
  NodeLayer node_layer2;
  node_layer1.Clear();
  node_layer2.Clear();
  NodeLayer* parent_layer = nullptr;
  NodeLayer* child_layer = nullptr;

  // projection
  obstacles_ = obstacles;
  ProjectObstacleToNodeMap();

#if DEBUG_NODE_COST
  double end_timestamp = IflyTime::Now_ms();
  double time_consumption = end_timestamp - start_timestamp;
  ILOG_INFO << "obs process time = " << time_consumption;
#endif

  // backward search in end node
  end_node_.Set(ex, ey, inv_xy_resolution_, XYbounds_);
  NodePoolPush(&end_node_);

#if DEBUG_NODE_COST
  end_node_.DebugNodeString(xy_grid_resolution_);
#endif

  // start_timestamp = IflyTime::Now_ms();
  node_layer1.AddNode(&end_node_);
  node_layer1.tag = NodeLayerTag::PARENT_LAYER;
  node_layer2.tag = NodeLayerTag::CHILD_LAYER;
  parent_layer = &node_layer1;
  child_layer = &node_layer2;

  // Grid a star begins
  Node2d* current_node = nullptr;
  Node2d* node_in_pool = nullptr;
  Node2d* next_node = nullptr;
  Node2dChildSet next_nodes;
  std::unordered_set<int> child_layer_hash_table;

  int explored_node_num = 0;

  // update dp cost
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

      next_nodes.size = 0;
      GenerateNextNodes(&next_nodes, current_node);

      // check next nodes
      for (int j = 0; j < next_nodes.size; j++) {
        explored_node_num++;

        next_node = &next_nodes.nodes[j];
        node_in_pool = GetNodeFromPool(next_node->GetGridIndex());

        if (node_in_pool == nullptr) {
          // ILOG_INFO << " next node is null";
          continue;
        }

        if (node_in_pool->IsCollision()) {
          // node_in_pool->SetCost(10000.0);
          continue;
        }

        if (next_node->GetCost() >= node_in_pool->GetCost()) {
          continue;
        }

        node_in_pool->CopyFrom(*next_node);

        if (child_layer_hash_table.find(next_node->GetGlobalID()) ==
            child_layer_hash_table.end()) {
          child_layer->AddNode(node_in_pool);

          child_layer_hash_table.insert(next_node->GetGlobalID());
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

float GridSearch::CheckDpMap(const float sx, const float sy) {
  Node2dIndex index = Node2d::CalcIndex(sx, sy, inv_xy_resolution_, XYbounds_);

  Node2d* node = GetNodeFromPool(index);

  float max_cost = 100000.0;

  if (node == nullptr) {
    return max_cost;
  }

  return node->GetCost() * heuristic_grid_resolution_;
}

void GridSearch::DebugNodePool() {
  for (int32_t i = 0; i < DP_MAX_X_SEARCH_SIZE; i++) {
    for (int32_t k = 0; k < DP_MAX_Y_SEARCH_SIZE; k++) {
      node_pool_[i][k].DebugNodeString(heuristic_grid_resolution_);
    }
  }

  cv::Mat map_matrix(DP_MAX_X_SEARCH_SIZE, DP_MAX_Y_SEARCH_SIZE, CV_8UC1,
                     cv::Scalar(200));

  int row_num = map_matrix.rows;
  int column_num = map_matrix.cols;

  float value = 0;

  for (int32_t i = 0; i < row_num; i++) {
    uchar* data = map_matrix.ptr<uchar>(i);

    for (int32_t j = 0; j < column_num; j++) {
      value = node_pool_[i][j].GetCost() * heuristic_grid_resolution_;
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

void GridSearch::NodePoolPush(Node2d* node) {
  if (!NodeIndexValid(node->GetGridIndex())) {
    ILOG_INFO << "size too big";
    return;
  }

  node_pool_[node->GetGridX()][node->GetGridY()] = *node;

  return;
}

Node2d* GridSearch::GetNodeFromPool(const Node2dIndex& id) {
  if (!NodeIndexValid(id)) {
    ILOG_INFO << "get node from pool fail" << id.x << " " << id.y;
    return nullptr;
  }

  return &node_pool_[id.x][id.y];
}

bool GridSearch::NodeIndexValid(const Node2dIndex& id) {
  if (id.x < 0 || id.x >= DP_MAX_X_SEARCH_SIZE || id.y < 0 ||
      id.y >= DP_MAX_Y_SEARCH_SIZE) {
    return false;
  }

  return true;
}

bool GridSearch::NodePositionValid(const float x, const float y) {
  if (x < XYbounds_.x_min || x > XYbounds_.x_max || y < XYbounds_.y_min ||
      y > XYbounds_.y_max) {
    return false;
  }

  return true;
}

void GridSearch::ResetNodePool() {
  int32_t x_bound_size = max_grid_x_ + 1;
  int32_t y_bound_size = max_grid_y_ + 1;

  x_bound_size = std::min(DP_MAX_X_SEARCH_SIZE, x_bound_size);
  y_bound_size = std::min(DP_MAX_Y_SEARCH_SIZE, y_bound_size);

  // ILOG_INFO << "x size = " << x_bound_size << ", y size = " << y_bound_size;

  for (int32_t i = 0; i < x_bound_size; i++) {
    for (int32_t k = 0; k < y_bound_size; k++) {
      node_pool_[i][k].Clear();
    }
  }

  return;
}

const bool GridSearch::IsPointInMapBound(const float x, const float y) {
  if (x >= XYbounds_.x_max || x <= XYbounds_.x_min) {
    return false;
  }
  if (y >= XYbounds_.y_max || y <= XYbounds_.y_min) {
    return false;
  }

  return true;
}

void GridSearch::Init() { return; }

void GridSearch::ProjectObstacleToNodeMap() {
  Node2d* node = nullptr;
  Node2dIndex index;

  for (size_t i = 0; i < obstacles_->virtual_obs.size(); i++) {
    const Position2D& obs = obstacles_->virtual_obs[i];

    // point
    if (!IsPointInMapBound(obs.x, obs.y)) {
      continue;
    }

    index.x = std::round((obs.x - XYbounds_.x_min) * inv_xy_resolution_);
    index.y = std::round((obs.y - XYbounds_.y_min) * inv_xy_resolution_);

    if (!NodeIndexValid(index)) {
      continue;
    }

    node = GetNodeFromPool(index);

    if (node == nullptr) {
      continue;
    }

    // ILOG_INFO << "x y " << position->x << " " << position->y;
    node->SetCollision(true);
  }

  for (size_t i = 0; i < obstacles_->point_cloud_list.size(); i++) {
    const PointCloudObstacle& obs = obstacles_->point_cloud_list[i];

    // fusion object check

    for (size_t j = 0; j < obs.points.size(); j++) {
      const Position2D& position = obs.points[j];

      if (!IsPointInMapBound(position.x, position.y)) {
        continue;
      }

      index.x = std::round((position.x - XYbounds_.x_min) * inv_xy_resolution_);
      index.y = std::round((position.y - XYbounds_.y_min) * inv_xy_resolution_);

      if (!NodeIndexValid(index)) {
        continue;
      }

      node = GetNodeFromPool(index);

      if (node == nullptr) {
        continue;
      }

      // ILOG_INFO << "x y " << position->x << " " << position->y;
      node->SetCollision(true);

      // todo, check box or other polygon
    }
  }

  return;
}

}  // namespace planning
