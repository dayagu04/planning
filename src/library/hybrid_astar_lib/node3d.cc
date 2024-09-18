#include "node3d.h"

#include <cstddef>
#include <string>

#include "log_glog.h"
#include "pose2d.h"
#include "utils_math.h"

namespace planning {
using ad_common::math::Box2d;

#define DEBUG_NODE3D (0)

Node3d::Node3d(const double x, const double y, const double phi) {
  x_ = x;
  y_ = y;
  phi_ = phi;

  path_.path_dist = 0;
  path_.point_size = 1;
  path_.points[0] = Pose2D(x, y, phi);

  visited_type_ = AstarNodeVisitedType::not_visited;

  is_start_node_ = false;

  dist_to_start_ = 0.0;
  global_id_ = 0;
  dist_to_obs_ = 100.0f;
}

Node3d::Node3d(double x, double y, double phi, const MapBound& XYbounds,
               const PlannerOpenSpaceConfig& open_space_conf) {
  x_ = x;
  y_ = y;
  phi_ = phi;

  grid_index_.x = std::round((x_ - XYbounds.x_min) *
                             open_space_conf.xy_grid_resolution_inv);

  grid_index_.y = std::round((y_ - XYbounds.y_min) *
                             open_space_conf.xy_grid_resolution_inv);
  grid_index_.phi =
      std::round((phi_ - (-M_PI)) * open_space_conf.phi_grid_resolution_inv);

  path_.path_dist = 0;
  path_.point_size = 1;
  path_.points[0].x = x;
  path_.points[0].y = y;
  path_.points[0].theta = phi;

  visited_type_ = AstarNodeVisitedType::not_visited;

  is_start_node_ = false;

  dist_to_start_ = 0.0;
  global_id_ = 0;
  dist_to_obs_ = 100.0f;
}

Node3d::Node3d(const NodePath& path, const MapBound& XYbounds,
               const PlannerOpenSpaceConfig& open_space_conf) {
  if (path.point_size > 0) {
    x_ = path.points[path.point_size - 1].x;
    y_ = path.points[path.point_size - 1].y;
    phi_ = IflyUnifyTheta(path.points[path.point_size - 1].theta, M_PI);
  }

  path_ = path;

  // XYbounds in xmin, xmax, ymin, ymax
  grid_index_.x = std::round((x_ - XYbounds.x_min) *
                             open_space_conf.xy_grid_resolution_inv);
  grid_index_.y = std::round((y_ - XYbounds.y_min) *
                             open_space_conf.xy_grid_resolution_inv);
  grid_index_.phi =
      std::round((phi_ - (-M_PI)) * open_space_conf.phi_grid_resolution_inv);

  visited_type_ = AstarNodeVisitedType::not_visited;
  is_start_node_ = false;
  dist_to_start_ = 0.0;

  global_id_ = 0;
  dist_to_obs_ = 100.0f;

#if DEBUG_NODE3D
  ILOG_INFO << "new index " << index_;
#endif
}

int Node3d::Set(const NodePath& path, const MapBound& XYbounds,
                const PlannerOpenSpaceConfig& open_space_conf,
                const double node_path_dist) {
  if (path.point_size > 0) {
    x_ = path.points[path.point_size - 1].x;
    y_ = path.points[path.point_size - 1].y;
    phi_ = IflyUnifyTheta(path.points[path.point_size - 1].theta, M_PI);
  }

  path_ = path;
  path_.path_dist = node_path_dist;

  // XYbounds in xmin, xmax, ymin, ymax
  grid_index_.x = std::round((x_ - XYbounds.x_min) *
                             open_space_conf.xy_grid_resolution_inv);
  grid_index_.y = std::round((y_ - XYbounds.y_min) *
                             open_space_conf.xy_grid_resolution_inv);
  grid_index_.phi =
      std::round((phi_ - (-M_PI)) * open_space_conf.phi_grid_resolution_inv);

  visited_type_ = AstarNodeVisitedType::not_visited;

  ResetCost();

  is_start_node_ = false;
  dist_to_obs_ = 100.0f;

  if (NodeIndexValid(grid_index_)) {
    global_id_ = IDTransform(grid_index_);
  } else {
    ILOG_INFO << "invalid node id";

    // invalid node
    path_.Clear();
  }

  collision_type_ = NodeCollisionType::none;

#if DEBUG_NODE3D
  ILOG_INFO << "new index " << index_;
#endif

  return 0;
}

void Node3d::ShrinkPathByCollisionID(const PlannerOpenSpaceConfig& conf) {
  if (collision_id_ >= 3) {
    path_.point_size = collision_id_;
    x_ = path_.points[path_.point_size - 1].x;
    y_ = path_.points[path_.point_size - 1].y;
    phi_ = IflyUnifyTheta(path_.points[path_.point_size - 1].theta, M_PI);
  }

  double shink_dist =
      path_.path_dist - conf.node_path_dist_resolution * (path_.point_size - 1);

  path_.path_dist -= shink_dist;
  dist_to_start_ -= shink_dist;

  return;
}

Box2d Node3d::GetBoundingBox(const VehicleParam& vehicle_param,
                             const double rear_overhanging, const double x,
                             const double y, const double phi) {
  double ego_length = vehicle_param.length;
  double ego_width = vehicle_param.width;
  double shift_distance = ego_length / 2.0 - rear_overhanging;
  Box2d ego_box(
      {x + shift_distance * std::cos(phi), y + shift_distance * std::sin(phi)},
      phi, ego_length, ego_width);
  return ego_box;
}

bool Node3d::operator==(const Node3d& right) const {
  const NodeGridIndex& id = right.GetIndex();
  if (grid_index_.x != id.x || grid_index_.y != id.y ||
      grid_index_.phi != id.phi) {
    return false;
  }

  return true;
}

const std::string Node3d::ComputeStringIndex(int x_grid, int y_grid,
                                             int phi_grid) const {
  std::string line = "_";

  std::string tmp = std::to_string(x_grid) + line + std::to_string(y_grid) +
                    line + std::to_string(phi_grid);

  return tmp;
}

void Node3d::DebugString() const {
  DebugPoseString();

  ILOG_INFO << "index " << grid_index_.x << ", " << grid_index_.y << " "
            << grid_index_.phi << " point size " << path_.point_size
            << " steering_ " << steering_ * 57.3 << " gear "
            << PathGearDebugString(gear_type_) << " g ,h, f " << traj_cost_
            << ", " << heuristic_cost_ << "," << f_cost_ << ", "
            << static_cast<int>(collision_type_) << ", dist to start "
            << dist_to_start_ << ", id " << global_id_ << " , size "
            << path_.point_size << " ,safe dist " << dist_to_obs_
            << ", gear switch num: " << gear_switch_num_;

  // for (size_t i = 0; i < path_.point_size; i++) {
  //   ILOG_INFO << "x,y,theta: " << path_.points[i].x << " , "
  //             << path_.points[i].y << " , " << path_.points[i].theta * 57.3;
  // }

  return;
}

void Node3d::DebugPoseString() const {
  ILOG_INFO << "x,y,theta: " << x_ << " , " << y_ << " , " << phi_ * 57.3;

  return;
}

void Node3d::DebugCost() const {
  ILOG_INFO << "g: " << traj_cost_ << " ,safe dist " << dist_to_obs_
            << " ,h: " << heuristic_cost_ << ", f:" << f_cost_ << " astar dist "
            << h_cost_debug_.astar_dist << " rs_path_dist "
            << h_cost_debug_.rs_path_dist << " rs_path_gear "
            << h_cost_debug_.rs_path_gear << " euler_dist "
            << h_cost_debug_.euler_dist << " ref_line_heading "
            << h_cost_debug_.ref_line_heading_cost << " rs_path_steer "
            << h_cost_debug_.rs_path_steer
            << " ,gear cost: " << h_cost_debug_.expected_gear;
  return;
}

void Node3d::CopyNode(const Node3d* node) {
  x_ = node->GetX();
  y_ = node->GetY();
  phi_ = node->GetPhi();

  path_ = node->GetNodePath();

  traj_cost_ = node->GetGCost();
  heuristic_cost_ = node->GetHeuCost();
  f_cost_ = node->GetFCost();

  pre_node_ = node->GetPreNode();

  steering_ = node->GetSteer();

  path_type_ = node->GetPathType();

  gear_type_ = node->GetGearType();

  grid_index_ = node->GetIndex();

  visited_type_ = node->GetVisitedType();

  global_id_ = node->GetGlobalID();

  next_node_ = node->GetNextNode();

  dist_to_start_ = node->GetDistToStart();

  is_start_node_ = node->IsStartNode();

  h_cost_debug_ = node->GetHeuCostDebug();

  collision_type_ = node->GetConstCollisionType();

  dist_to_obs_ = node->GetDistToObs();

  // nodeMapIt = node->nodeMapIt;
  return;
}

bool Node3d::IsPathGearChange(const AstarPathGear type) {
  if (gear_type_ == AstarPathGear::drive && type == AstarPathGear::reverse) {
    return true;
  }

  if (gear_type_ == AstarPathGear::reverse && type == AstarPathGear::drive) {
    return true;
  }

  return false;
}

const bool Node3d::IsRsPath() const {
  if (path_type_ == AstarPathType::Reeds_Shepp) {
    return true;
  }
  return false;
}

void Node3d::SetVisitedType(const AstarNodeVisitedType type) {
  visited_type_ = type;
  return;
}

const bool Node3d::IsNodeValid() {
  if (path_.point_size > 0) {
    return true;
  }
  return false;
}

bool Node3d::NodeIndexValid(const NodeGridIndex& id) {
  if (id.x < 0 || id.x >= PER_DIMENSION_MAX_NODE || id.y < 0 ||
      id.y >= PER_DIMENSION_MAX_NODE || id.phi < 0 ||
      id.phi >= PER_DIMENSION_MAX_NODE) {
    return false;
  }

  return true;
}

void Node3d::ClearPath() {
  path_.point_size = 0;
  return;
}

void Node3d::Clear() {
  path_.Clear();

  pre_node_ = nullptr;
  next_node_ = nullptr;

  is_start_node_ = false;

  visited_type_ = AstarNodeVisitedType::not_visited;

  path_type_ = AstarPathType::none;
  global_id_ = 0;
  dist_to_obs_ = 100.0f;

  return;
}

const AstarNodeVisitedType Node3d::GetVisitedType() const {
  return visited_type_;
}

size_t Node3d::IDTransform(const NodeGridIndex& id) {
  size_t x_id = id.x;
  size_t y_id = id.y << 10;
  size_t theta_id = id.phi << 20;

  return x_id + y_id + theta_id;
}

void Node3d::SetGlobalID(const size_t id) {
  global_id_ = id;

  return;
}

void Node3d::ResetCost() {
  traj_cost_ = 0.0;
  heuristic_cost_ = 0.0;
  f_cost_ = 0.0;

  return;
}

void Node3d::SetCollisionType(const NodeCollisionType type) {
  collision_type_ = type;
}

void Node3d::CoordinateToGridIndex(const double x, const double y,
                                   const double phi, NodeGridIndex* index,
                                   const MapBound& XYbounds,
                                   const PlannerOpenSpaceConfig& conf) {
  index->x = std::round((x - XYbounds.x_min) * conf.xy_grid_resolution_inv);
  index->y = std::round((y - XYbounds.y_min) * conf.xy_grid_resolution_inv);
  index->phi = std::round((phi - (-M_PI)) * conf.phi_grid_resolution_inv);
}

const Pose2D Node3d::GetPose() const { return Pose2D(x_, y_, phi_); }

const double Node3d::GetEulerDist(const Node3d* end) const {
  double dist;

  dist = std::pow(x_ - end->GetX(), 2) + std::pow(y_ - end->GetY(), 2);

  return std::sqrt(dist);
}

void Node3d::SetDistToObs(const float dist) { dist_to_obs_ = dist; }

}  // namespace planning
