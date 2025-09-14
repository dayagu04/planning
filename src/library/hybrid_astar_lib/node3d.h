#pragma once

#include <bits/stdint-intn.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ad_common/math/box2d.h"

#include "./../../modules/common/config/vehicle_param.h"
#include "cost/g_cost.h"
#include "cost/h_cost.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_config.h"
#include "log_glog.h"
#include "node2d.h"
#include "pose2d.h"

namespace planning {

#define NODE_PATH_MAX_POINT (8)
#define PER_DIMENSION_MAX_NODE (1000)

struct NodeGridIndex {
  int x;
  int y;
  int phi;

  void PrintInfo(const bool enable_log = true) {
    ILOG_INFO << "x: " << x << ", y: " << y << ", phi: " << phi;
  }
};

struct NodePath {
  int point_size = 0;
  // node points max number is 8. If this node is rs, please restore path in rs
  // path.
  Pose2f points[NODE_PATH_MAX_POINT];
  float path_dist = 0.0f;

  void Clear() {
    point_size = 0;
    path_dist = 0.0f;
    return;
  }

  NodePath() = default;

  NodePath(const float x, const float y, const float theta) {
    point_size = 1;
    points[0].x = x;
    points[0].y = y;
    points[0].theta = theta;
  }

  NodePath(const Pose2f& pose) {
    point_size = 1;
    points[0].x = pose.x;
    points[0].y = pose.y;
    points[0].theta = pose.theta;
  }

  const Pose2f& GetEndPoint() const {
    if (point_size > 0) {
      return points[point_size - 1];
    } else {
      return points[0];
    }
  }
};

enum class NodeShrinkType {
  NONE = 0,
  OUT_OF_BOUNDARY = 1,
  COLLISION = 2,
  BACK_TO_START_NODE = 3,
  BACK_TO_PARENT_NODE = 4,
  UNEXPECTED_HEADING = 5,
  UNEXPECTED_GEAR = 6,
  UNEXPECTED_STEERING_WHEEL = 7,
  UNEXPECTED_DRIVE_DIST = 8,
  FAIL_TO_ALLOCATE_NODE = 9,
  UNEXPECTED_POS = 10,
  MAX_NUMBER,
};

class Node3d {
 public:
  Node3d() = default;

  Node3d(const float x, const float y, const float phi);

  Node3d(const float x, const float y, const float phi,
         const MapBound& XYbounds,
         const PlannerOpenSpaceConfig& open_space_conf);

  Node3d(const NodePath& path, const MapBound& XYbounds,
         const PlannerOpenSpaceConfig& open_space_conf);

  void Set(const NodePath& path, const MapBound& XYbounds,
           const PlannerOpenSpaceConfig& open_space_conf,
           const float node_path_dist);

  int UpdatePath(const NodePath& path, const MapBound& XYbounds,
                 const PlannerOpenSpaceConfig& open_space_conf,
                 const float node_path_dist);

  void SetGlobalID(const int id);

  static ad_common::math::Box2d GetBoundingBox(
      const VehicleParam& vehicle_param, const float rear_overhanging,
      const float x, const float y, const float phi);

  float GetFCost() const { return f_cost_; }

  float GetGCost() const { return traj_cost_; }

  float GetHeuCost() const { return heuristic_cost_; }

  int GetGridX() const { return grid_index_.x; }

  int GetGridY() const { return grid_index_.y; }

  int GetGridPhi() const { return grid_index_.phi; }

  const float GetX() const { return path_.GetEndPoint().x; }

  const float GetY() const { return path_.GetEndPoint().y; }

  const float GetPhi() const { return path_.GetEndPoint().theta; }

  // Get end pose
  const Pose2f& GetPose() const;

  bool operator==(const Node3d& right) const;

  const NodeGridIndex& GetIndex() const { return grid_index_; }

  int GetStepSize() const { return path_.point_size; }

  const bool IsRsPath() const;

  const bool IsQunticPolynomialPath() const;

  float GetSteer() const { return steering_; }

  float GetSteerChange(const float next_steer) const {
    return steering_ - next_steer;
  }

  const bool IsSteerOpposite(const float next) const;

  const float GetRadius() const { return radius_; }

  const float GetKappa() const { return kappa_; }

  Node3d* GetPreNode() const { return pre_node_; }

  const NodePath& GetNodePath() const { return path_; }

  void SetPre(Node3d* pre_node) { pre_node_ = pre_node; }

  const Node3d* GetConstNextNode() const { return next_node_; }

  Node3d* GetNextNode() const { return next_node_; }

  Node3d* GetMutableNextNode() { return next_node_; }

  void SetNext(Node3d* next_node) { next_node_ = next_node; }

  void SetGCost(float cost) { traj_cost_ = cost; }

  void SetFCost() { f_cost_ = traj_cost_ + heuristic_cost_; }

  void SetFCost(const float v) { f_cost_ = v; }

  void SetHeuCost(float cost) { heuristic_cost_ = cost; }

  const float GetNodePathDistance() const { return path_.path_dist; }

  // void SetHeuCostDebug(const NodeHeuristicCost& cost) { h_cost_debug_ = cost;
  // }

  // const NodeHeuristicCost& GetHeuCostDebug() const { return h_cost_debug_; }

  void SetGearType(const AstarPathGear type) { gear_type_ = type; }

  void SetDistToStart(const float dist) { dist_to_start_ = dist; }

  const float GetDistToStart() const { return dist_to_start_; }

  const AstarPathGear& GetGearType() const { return gear_type_; }

  void SetSteer(float steering) { steering_ = steering; }

  void SetRadius(const float radius) { radius_ = radius; }

  void SetKappa(const float kappa) { kappa_ = kappa; }

  void SetSingleGearLength(const float length) { single_gear_length_ = length; }

  const float GetSingleGearLength() const { return single_gear_length_; }

  const AstarPathType GetPathType() const { return path_type_; }

  void SetPathType(const AstarPathType type) { path_type_ = type; }

  bool IsPathGearChange(const AstarPathGear type) const;

  const bool IsForward() const { return gear_type_ == AstarPathGear::DRIVE; }

  void DebugString() const;

  void DebugPoseString() const;

  void DebugCost() const;

  void ResetCost();

  // must check NodeGridIndex before use IDTransform.
  int IDTransform(const NodeGridIndex& id);

  void SetIsStartNode(const bool is_start) { is_start_node_ = is_start; }

  const bool IsStartNode() const { return is_start_node_; }

  void SetCollisionType(const NodeCollisionType type);

  const NodeCollisionType GetConstCollisionType() const {
    return collision_type_;
  }

  const std::string ComputeStringIndex(int x_grid, int y_grid,
                                       int phi_grid) const;

  void Clear();

  bool NodeIndexValid(const NodeGridIndex& id);

  void ClearPath();

  const bool IsNodeValid();

  const int GetGlobalID() const { return global_id_; }

  const AstarNodeVisitedType GetVisitedType() const;

  void SetVisitedType(const AstarNodeVisitedType type);

  static void CoordinateToGridIndex(const float x, const float y,
                                    const float phi, NodeGridIndex* index,
                                    const MapBound& XYbounds,
                                    const PlannerOpenSpaceConfig& conf);

  // todo: move all heuristic_cost to this file
  const float GetEulerDist(const Node3d* end) const;

  const float GetPhiErr(const Node3d* end) const;

  void SetDistToObs(const float dist);

  float GetDistToObs() const { return dist_to_obs_; }

  void SetObsDistRelativeSlot(
      const ObsToPathDistRelativeSlot& dist_to_obs_relative_slot) {
    dist_to_obs_relative_slot_ = dist_to_obs_relative_slot;
  }

  const ObsToPathDistRelativeSlot& GetObsDistRelativeSlot() const {
    return dist_to_obs_relative_slot_;
  }

  void SetGearSwitchNum(const int number) {
    gear_switch_num_ = number;
    return;
  }

  void AddGearSwitchNumber() { gear_switch_num_++; }

  void AddGearSwitchNumber(const int number) { gear_switch_num_ += number; }

  int GetGearSwitchNum() const { return gear_switch_num_; }

  void SetCollisionID(const int collision_id) {
    collision_id_ = collision_id;
    return;
  }

  const int GetCollisionID() const { return collision_id_; }

  void ShrinkPathByCollisionID(const PlannerOpenSpaceConfig& conf);

  void SetMultiMapIter(std::multimap<float, Node3d*>::iterator iter) {
    multimap_iter_ = iter;
    return;
  }

  std::multimap<float, Node3d*>::iterator GetMultiMapIter() {
    return multimap_iter_;
  }

  float DistToPose(const Pose2f& pose);

  Node3d* GearSwitchNode() const { return gear_switch_node_; }

  void SetGearSwitchNode(Node3d* node) {
    gear_switch_node_ = node;
    return;
  }

  Node3d* NextGearSwitchNode() const { return next_gear_switch_node_; }

  void SetNextGearSwitchNode(Node3d* node) {
    next_gear_switch_node_ = node;
    return;
  }

  const bool IsSteerOppositeWithParent() const;

  const bool IsScurveWithParent() const;

  const bool IsScurve(const AstarPathGear type, const float steer) const;

  const bool IsSameSteerDir(const float next) const;

  const bool IsReturnPath(const bool is_gear_switch, const float steer);

  void SetScurveNum(const int number) {
    s_curve_num_ = number;
    return;
  }

  void AddScurveNumber() {
    s_curve_num_++;
    return;
  }

  const int GetScurveNum() const { return s_curve_num_; }

 protected:
  // path point size
  NodePath path_;

  // node index, unique value.
  NodeGridIndex grid_index_;

  // todo, move to private
  AstarNodeVisitedType visited_type_;

  // generate global id by node position
  int global_id_;

  // weight: 15
  // [0-0.15], cost: 1000;
  // [0.15-0.5],cost: (1/dist -2) * weight;
  // [0.5-1000], cost:0;
  float dist_to_obs_;
  ObsToPathDistRelativeSlot dist_to_obs_relative_slot_;

  // g cost
  float traj_cost_ = 0.0;
  // h cost
  float heuristic_cost_ = 0.0;

  // f cost
  float f_cost_ = 0.0;
  // backward pass
  Node3d* pre_node_ = nullptr;

  // forward pass
  Node3d* next_node_ = nullptr;

  // front wheel angle, [-pi, +pi]
  // left is positive
  float steering_ = 0.0;
  float radius_;
  float kappa_;
  float single_gear_length_ = 0.0;

  // if is rs path, record rs first path gear.
  AstarPathGear gear_type_;

  // gear numer is 0: only one gear path;
  // gear numer is 1: two different gear path;
  int gear_switch_num_;

  int s_curve_num_;

  // is positive
  float dist_to_start_;

  AstarPathType path_type_;

  bool is_start_node_;

  // for debug
  // NodeHeuristicCost h_cost_debug_;

  NodeCollisionType collision_type_;
  int collision_id_;

  std::multimap<float, Node3d*>::iterator multimap_iter_;

  // 第一次换档点.
  // 如果换档点在搜索节点，需要记录. 这里如果搜索节点和rs曲线连接处换档，也记录.
  // 如果换档点在rs曲线上，不需要记录.
  Node3d* gear_switch_node_ = nullptr;
  // 下一个换挡点.
  Node3d* next_gear_switch_node_ = nullptr;
};

}  // namespace planning
