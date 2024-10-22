#pragma once

#include <bits/stdint-intn.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ad_common/math/line_segment2d.h"
#include "hybrid_astar_common.h"
#include "log_glog.h"

namespace planning {

struct Node2dIndex {
  int x;
  int y;
};

enum class Node2dDirection {
  none = 0,
  left,
  left_upper,
  upper,
  right_upper,
  right,
  right_lower,
  lower,
  left_lower,
  max_num
};

class Node2d {
 public:
  Node2d() {}

  Node2d(const double x, const double y, const double inv_xy_resolution,
         const MapBound& XYbounds) {
    // XYbounds with xmin, xmax, ymin, ymax

    x_ = x;
    y_ = y;

    grid_index_.x = std::round((x - XYbounds.x_min) * inv_xy_resolution);
    grid_index_.y = std::round((y - XYbounds.y_min) * inv_xy_resolution);

    if (IsNode2dIndexValid()) {
      id_ = IDTransform(grid_index_);
    } else {
      id_ = -1;
    }

  }

  Node2d(const int grid_x, const int grid_y, const MapBound& XYbounds,
         const double xy_resolution) {
    x_ = grid_x * xy_resolution + XYbounds.x_min;
    y_ = grid_y * xy_resolution + XYbounds.y_min;

    grid_index_.x = grid_x;
    grid_index_.y = grid_y;

    if (IsNode2dIndexValid()) {
      id_ = IDTransform(grid_index_);
    } else {
      id_ = -1;
    }
  }

  void Set(const double x, const double y, const double inv_xy_resolution,
           const MapBound& XYbounds) {
    x_ = x;
    y_ = y;

    grid_index_.x = std::round((x - XYbounds.x_min) * inv_xy_resolution);
    grid_index_.y = std::round((y - XYbounds.y_min) * inv_xy_resolution);

    if (IsNode2dIndexValid()) {
      id_ = IDTransform(grid_index_);
    } else {
      id_ = -1;
    }

    cost_ = 0.0;
  }

  void CalcRealPositionByIndex(const int32_t grid_x, const int32_t grid_y,
                               const std::vector<double>& XYbounds,
                               const double xy_resolution) {
    x_ = grid_x * xy_resolution + XYbounds[0];
    y_ = grid_y * xy_resolution + XYbounds[2];

    return;
  }

  void SetCost(const double cost) { cost_ = cost; }

  // x index
  int GetGridX() const { return grid_index_.x; }

  double GetX() const { return x_; }

  int GetGridY() const { return grid_index_.y; }

  double GetY() const { return y_; }

  double GetCost() const { return cost_; }

  const Node2dIndex& GetGridIndex() const { return grid_index_; }

  const int GetGlobalID() const { return id_; }

  static Node2dIndex CalcIndex(const double x, const double y,
                               const double inv_xy_resolution,
                               const MapBound& XYbounds) {
    // XYbounds with xmin, xmax, ymin, ymax
    Node2dIndex grid_id;
    grid_id.x = std::round((x - XYbounds.x_min) * inv_xy_resolution);
    grid_id.y = std::round((y - XYbounds.y_min) * inv_xy_resolution);

    return grid_id;
  }

  static std::string CalcStringIndex(const double x, const double y,
                                     const double xy_resolution,
                                     const MapBound& XYbounds) {
    // XYbounds with xmin, xmax, ymin, ymax
    int grid_x = std::round((x - XYbounds.x_min) / xy_resolution);
    int grid_y = std::round((y - XYbounds.y_min) / xy_resolution);

    return ComputeStringIndex(grid_x, grid_y);
  }

  bool operator==(const Node2d& right) const {
    if (right.GetGridX() != x_ || right.GetGridY() != y_) {
      return false;
    }

    return true;
  }

  void Clear() {
    // grid_index_.x = grid_x;
    // grid_index_.y = grid_y;

    // x_ = 0.0;
    // y_ = 0.0;

    is_collision_ = false;
    cost_ = 10000.0;

    return;
  }

  void CopyFrom(const Node2d& node) {
    x_ = node.GetX();
    y_ = node.GetY();

    cost_ = node.GetCost();
    id_ = node.GetGlobalID();
    grid_index_ = node.GetGridIndex();

    return;
  }

  void DebugNodeString(const double xy_resolution) {
    ILOG_INFO << "id, x " << grid_index_.x << ", id y " << grid_index_.y
              << ", x " << x_ << ", y " << y_ << ", dist "
              << cost_ * xy_resolution << ", is collision "
              << static_cast<int>(is_collision_);

    return;
  }

  void SetCollision(const bool is_collision) {
    is_collision_ = is_collision;
    return;
  }

  const bool IsCollision() const { return is_collision_; }

  static std::string ComputeStringIndex(int x_grid, int y_grid) {
    std::string line = "_";

    std::string tmp = std::to_string(x_grid) + line + std::to_string(y_grid);

    return tmp;
  }

  int IDTransform(const Node2dIndex& id) {
    int x_id = id.x;
    int y_id = id.y << 10;

    return x_id + y_id;
  }

  const bool IsNode2dIndexValid() {
    if (grid_index_.x < 0 || grid_index_.x > 1000 || grid_index_.y < 0 ||
        grid_index_.y > 1000) {
      return false;
    }

    return true;
  }

  const bool IsNodeValid() {
    if (id_ < 0) {
      return false;
    }

    return true;
  }

 private:
  double x_;
  double y_;

  // cost is not real path length, is grid num
  double cost_;

  // unique id
  int id_;

  Node2dIndex grid_index_;

  bool is_collision_;
};

struct Node2dChildSet {
  int size;
  Node2d nodes[8];
};

}  // namespace planning