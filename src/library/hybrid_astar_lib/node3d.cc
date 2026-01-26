#include "node3d.h"

#include <cmath>
#include <cstddef>
#include <string>

#include "log_glog.h"
#include "pose2d.h"
#include "utils_math.h"

namespace planning {
using ad_common::math::Box2d;

#define DEBUG_NODE3D (0)
#define DEBUG_NODE_HCOST (0)

Node3d::Node3d(const float x, const float y, const float phi) {
  path_.path_dist = 0;
  path_.point_size = 1;
  float theta = IflyUnifyTheta(phi, M_PIf32);
  path_.points[0] = Pose2f(x, y, theta);

  visited_type_ = AstarNodeVisitedType::NOT_VISITED;

  is_start_node_ = false;

  dist_to_start_ = 0.0f;
  global_id_ = 0;
  dist_to_obs_ = 26.8f;
  gear_switch_num_ = 0;
  s_curve_num_ = 0;
  gear_switch_node_ = nullptr;
}

Node3d::Node3d(float x, float y, float phi, const MapBound& XYbounds,
               const PlannerOpenSpaceConfig& open_space_conf) {
  grid_index_.x =
      std::round((x - XYbounds.x_min) * open_space_conf.xy_grid_resolution_inv);

  grid_index_.y =
      std::round((y - XYbounds.y_min) * open_space_conf.xy_grid_resolution_inv);

  float theta = IflyUnifyTheta(phi, M_PIf32);
  grid_index_.phi =
      std::round((theta - (-M_PI)) * open_space_conf.phi_grid_resolution_inv);

  path_.path_dist = 0.0f;
  path_.point_size = 1;
  path_.points[0].x = x;
  path_.points[0].y = y;
  path_.points[0].theta = theta;

  visited_type_ = AstarNodeVisitedType::NOT_VISITED;

  is_start_node_ = false;

  dist_to_start_ = 0.0f;
  global_id_ = 0;
  dist_to_obs_ = 26.8f;
  gear_switch_num_ = 0;
  s_curve_num_ = 0;
  gear_switch_node_ = nullptr;
}

Node3d::Node3d(const NodePath& path, const MapBound& XYbounds,
               const PlannerOpenSpaceConfig& open_space_conf) {
  path_ = path;

  // XYbounds in xmin, xmax, ymin, ymax
  grid_index_.x = std::round((path_.GetEndPoint().x - XYbounds.x_min) *
                             open_space_conf.xy_grid_resolution_inv);
  grid_index_.y = std::round((path_.GetEndPoint().y - XYbounds.y_min) *
                             open_space_conf.xy_grid_resolution_inv);

  float theta = IflyUnifyTheta(path_.GetEndPoint().theta, M_PIf32);
  grid_index_.phi =
      std::round((theta - (-M_PI)) * open_space_conf.phi_grid_resolution_inv);

  visited_type_ = AstarNodeVisitedType::NOT_VISITED;
  is_start_node_ = false;
  dist_to_start_ = 0.0f;

  global_id_ = 0;
  dist_to_obs_ = 26.8f;
  gear_switch_num_ = 0;
  s_curve_num_ = 0;
  gear_switch_node_ = nullptr;

#if DEBUG_NODE3D
  ILOG_INFO << "new index " << index_;
#endif
}

void Node3d::Set(const NodePath& path, const MapBound& XYbounds,
                 const PlannerOpenSpaceConfig& open_space_conf,
                 const float node_path_dist) {
  path_ = path;
  path_.path_dist = node_path_dist;

  // XYbounds in xmin, xmax, ymin, ymax
  grid_index_.x = std::round((path_.GetEndPoint().x - XYbounds.x_min) *
                             open_space_conf.xy_grid_resolution_inv);
  grid_index_.y = std::round((path_.GetEndPoint().y - XYbounds.y_min) *
                             open_space_conf.xy_grid_resolution_inv);

  float theta = IflyUnifyTheta(path_.GetEndPoint().theta, M_PIf32);
  grid_index_.phi = std::round((theta - (-M_PIf32)) *
                               open_space_conf.phi_grid_resolution_inv);

  visited_type_ = AstarNodeVisitedType::NOT_VISITED;

  ResetCost();

  is_start_node_ = false;
  dist_to_obs_ = 26.8f;

  if (NodeIndexValid(grid_index_)) {
    global_id_ = IDTransform(grid_index_);
  } else {
    ILOG_INFO << "invalid node id";

    // invalid node
    path_.Clear();
  }

  collision_type_ = NodeCollisionType::NONE;
  gear_switch_num_ = 0;
  s_curve_num_ = 0;
  gear_switch_node_ = nullptr;

  radius_ = 100000.0f;
  steering_ = 0.0f;
  kappa_ = 0.0f;
  gear_type_ = AstarPathGear::NONE;
  path_type_ = AstarPathType::NONE;
  straight_dist_to_goal_ = 0.0;

#if DEBUG_NODE3D
  ILOG_INFO << "new index " << index_;
#endif

  return;
}

int Node3d::UpdatePath(const NodePath& path, const MapBound& XYbounds,
                       const PlannerOpenSpaceConfig& open_space_conf,
                       const float node_path_dist) {
  path_ = path;
  path_.path_dist = node_path_dist;

  // XYbounds in xmin, xmax, ymin, ymax
  grid_index_.x = std::round((path_.GetEndPoint().x - XYbounds.x_min) *
                             open_space_conf.xy_grid_resolution_inv);
  grid_index_.y = std::round((path_.GetEndPoint().y - XYbounds.y_min) *
                             open_space_conf.xy_grid_resolution_inv);

  float theta = IflyUnifyTheta(path_.GetEndPoint().theta, M_PIf32);
  grid_index_.phi =
      std::round((theta - (-M_PI)) * open_space_conf.phi_grid_resolution_inv);

  if (NodeIndexValid(grid_index_)) {
    global_id_ = IDTransform(grid_index_);
  } else {
    ILOG_INFO << "invalid node id";

    // invalid node
    path_.Clear();
  }
  return 1;
}

void Node3d::ShrinkPathByCollisionID(const PlannerOpenSpaceConfig& conf) {
  if (collision_id_ >= 3) {
    path_.point_size = collision_id_;
  }

  float shink_dist =
      path_.path_dist - conf.node_path_dist_resolution * (path_.point_size - 1);

  path_.path_dist -= shink_dist;
  dist_to_start_ -= shink_dist;

  return;
}

Box2d Node3d::GetBoundingBox(const VehicleParam& vehicle_param,
                             const float rear_overhanging, const float x,
                             const float y, const float phi) {
  float ego_length = vehicle_param.length;
  float ego_width = vehicle_param.width;
  float shift_distance = ego_length / 2.0 - rear_overhanging;
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
            << " steering_ " << steering_ * 57.2957795 << " gear "
            << PathGearDebugString(gear_type_) << " (g, h, f) = (" << traj_cost_
            << ", " << heuristic_cost_ << ", " << f_cost_
            << "), collision_type " << static_cast<int>(collision_type_)
            << ", dist to start " << dist_to_start_ << ", id " << global_id_
            << " , size " << path_.point_size << " ,safe dist " << dist_to_obs_
            << ", gear switch num: " << gear_switch_num_ << ", s curve num "
            << s_curve_num_ << ", path type "
            << GetNodeCurveDebugString(path_type_) << ", striaght dist "
            << straight_dist_to_goal_;

  if (gear_switch_node_ != nullptr) {
    ILOG_INFO << "gear switch node dist to start = "
              << gear_switch_node_->GetDistToStart();

    ILOG_INFO << "x = " << gear_switch_node_->GetPose().x
              << ",y = " << gear_switch_node_->GetPose().y
              << ",theta = " << gear_switch_node_->GetPose().theta * 57.2957795;
  }

  // for (size_t i = 0; i < path_.point_size; i++) {
  //   ILOG_INFO << "x,y,theta: " << path_.points[i].x << " , "
  //             << path_.points[i].y << " , " << path_.points[i].theta * 57.3;
  // }

  return;
}

void Node3d::DebugPoseString() const {
  ILOG_INFO << "x,y,theta(degree): " << path_.GetEndPoint().x << " , "
            << path_.GetEndPoint().y << " , "
            << path_.GetEndPoint().theta * 57.2957795;

  return;
}

void Node3d::DebugCost() const {
  ILOG_INFO << "g: " << traj_cost_ << " ,safe dist " << dist_to_obs_
            << " ,h: " << heuristic_cost_ << ", f:" << f_cost_ << " astar dist "
            << "  dp_cost = " << heuristic_cost_debug_.dp_cost
            << "  ref_line_heading_cost = "
            << heuristic_cost_debug_.ref_line_heading_cost
            << "  curve_path_cost = " << heuristic_cost_debug_.curve_path_cost
            << "  euler_cost = " << heuristic_cost_debug_.euler_dist;

  return;
}

bool Node3d::IsPathGearChange(const AstarPathGear type) const {
  if (gear_type_ == AstarPathGear::DRIVE && type == AstarPathGear::REVERSE) {
    return true;
  }

  if (gear_type_ == AstarPathGear::REVERSE && type == AstarPathGear::DRIVE) {
    return true;
  }

  return false;
}

const bool Node3d::IsRsPath() const {
  if (path_type_ == AstarPathType::REEDS_SHEPP) {
    return true;
  }
  return false;
}

const bool Node3d::IsQunticPolynomialPath() const {
  if (path_type_ == AstarPathType::QUNTIC_POLYNOMIAL) {
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

  visited_type_ = AstarNodeVisitedType::NOT_VISITED;

  path_type_ = AstarPathType::NONE;
  global_id_ = 0;
  dist_to_obs_ = 26.8f;

  gear_switch_node_ = nullptr;

  return;
}

const AstarNodeVisitedType Node3d::GetVisitedType() const {
  return visited_type_;
}

int Node3d::IDTransform(const NodeGridIndex& id) {
  int x_id = id.x;
  int y_id = id.y << 10;
  int theta_id = id.phi << 20;

  return x_id + y_id + theta_id;
}

void Node3d::SetGlobalID(const int id) {
  global_id_ = id;

  return;
}

void Node3d::ResetCost() {
  traj_cost_ = 0.0f;
  heuristic_cost_ = 0.0f;
  f_cost_ = 0.0f;

  return;
}

void Node3d::SetCollisionType(const NodeCollisionType type) {
  collision_type_ = type;
}

void Node3d::CoordinateToGridIndex(const float x, const float y,
                                   const float phi, NodeGridIndex* index,
                                   const MapBound& XYbounds,
                                   const PlannerOpenSpaceConfig& conf) {
  index->x = std::round((x - XYbounds.x_min) * conf.xy_grid_resolution_inv);
  index->y = std::round((y - XYbounds.y_min) * conf.xy_grid_resolution_inv);
  index->phi = std::round((phi - (-M_PI)) * conf.phi_grid_resolution_inv);
}

const Pose2f& Node3d::GetPose() const { return path_.GetEndPoint(); }

const float Node3d::GetEulerDist(const Node3d* end) const {
  return std::hypot(path_.GetEndPoint().x - end->GetX(),
                    path_.GetEndPoint().y - end->GetY());
}

const float Node3d::GetPhiErr(const Node3d* end) const {
  return IflyUnifyTheta(std::fabs(path_.GetEndPoint().theta - end->GetPhi()),
                        M_PIf32);
}

void Node3d::SetDistToObs(const float dist) { dist_to_obs_ = dist; }

float Node3d::DistToPose(const Pose2f& pose) {
  float dist = path_.GetEndPoint().DistanceTo(pose);

  return dist;
}

const bool Node3d::IsSteerOpposite(const float next) const {
  if (steering_ > 0.01f && next < -0.01f) {
    return true;
  }

  if (steering_ < -0.01f && next > 0.01f) {
    return true;
  }

  return false;
}

const bool Node3d::IsSteerOpposite(const AstarPathSteer steer) const {
  if (steer_type_ == AstarPathSteer::LEFT && steer == AstarPathSteer::RIGHT) {
    return true;
  }

  if (steer_type_ == AstarPathSteer::RIGHT && steer == AstarPathSteer::LEFT) {
    return true;
  }

  return false;
}

const bool Node3d::IsSteerOppositeWithParent() const {
  if (pre_node_ == nullptr) {
    return false;
  }

  return IsSteerOpposite(pre_node_->GetSteer());
}

const bool Node3d::IsScurveWithParent() const {
  if (pre_node_ == nullptr) {
    return false;
  }

  if (IsPathGearChange(pre_node_->GetGearType())) {
    return false;
  }

  return IsSteerOppositeWithParent();
}

const bool Node3d::IsScurveWithParentSteer() const {
  if (pre_node_ == nullptr) {
    return false;
  }

  if (IsPathGearChange(pre_node_->GetGearType())) {
    return false;
  }

  return IsSteerOpposite(pre_node_->GetSteerType());
}

const bool Node3d::IsScurve(const AstarPathGear type,
                            const AstarPathSteer steer) const {
  if (IsPathGearChange(type)) {
    return false;
  }

  // define s curve:
  // left to right, is s curve;
  // straight to left, is not s curve;
  // straight to right, is not s curve;
  // left to straight, is not s curve;
  // right to straight, is not s curve;
  return IsSteerOpposite(steer);
}

const bool Node3d::IsSameSteerDir(const float next) const {
  // left turn, left turn is same turn direction
  if (steering_ > 0.01f && next > 0.01f) {
    return true;
  }

  if (steering_ < -0.01f && next < -0.01f) {
    return true;
  }

  if (std::fabs(steering_ - next) < 0.01f) {
    return true;
  }

  return false;
}

const bool Node3d::IsSameSteerDir(const AstarPathSteer next) const {
  // left turn, left turn is same turn direction
  if (steer_type_ == AstarPathSteer::LEFT && next == AstarPathSteer::LEFT) {
    return true;
  }

  if (steer_type_ == AstarPathSteer::RIGHT && next == AstarPathSteer::RIGHT) {
    return true;
  }

  return false;
}

const bool Node3d::IsReturnPath(const bool is_gear_switch, const float steer) {
  if (is_gear_switch) {
    return IsSameSteerDir(steer);
  }

  return false;
}

const bool Node3d::IsReturnPath(const bool is_gear_switch,
                                const AstarPathSteer steer) {
  if (is_gear_switch) {
    return IsSameSteerDir(steer);
  }

  return false;
}

const bool Node3d::IsSameGearSwitchNode(const Node3d* other) const {
  if (gear_switch_node_ == nullptr || other == nullptr) {
    return false;
  }

  if (gear_switch_node_->GetGlobalID() == other->GetGlobalID()) {
    return true;
  }

  return false;
}

}  // namespace planning
