#include "hybrid_a_star.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>

#include "ad_common/math/math_utils.h"
#include "config/vehicle_param.h"
#include "h_cost.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_request.h"
#include "log_glog.h"
#include "node3d.h"
#include "path_comparator.h"
#include "rs_sampling.h"
#include "spiral_sampling.h"
#include "src/common/ifly_time.h"
#include "transform2d.h"
#include "utils_math.h"

namespace planning {

#define PLOT_RS_COST_PATH (0)
#define PLOT_RS_EXNTEND_TO_END (0)
#define PLOT_CHILD_NODE (0)
#define PLOT_SEARCH_SEQUENCE (0)
#define PLOT_DELETE_NODE (0)
#define RS_H_COST_MAX_NUM (32)

#define DEBUG_SEARCH_RESULT (0)
#define DEBUG_CHILD_NODE (0)
#define DEBUG_REF_LINE_COST (0)
#define DEBUG_EDT (0)

#define DEBUG_NODE_MAX_NUM (10000)
#define DEBUG_NODE_GEAR_SWITCH_NUMBER (0)

#define LOG_TIME_PROFILE (0)

#define DEBUG_ONE_SHOT_PATH (0)
#define DEBUG_ONE_SHOT_PATH_MAX_NODE (10000)
#define ENABLE_OBS_DIST_G_COST (0)

HybridAStar::HybridAStar(const PlannerOpenSpaceConfig& open_space_conf,
                         const VehicleParam& veh_param,
                         const ParkObstacleList* obstacles,
                         EulerDistanceTransform* edt,
                         const ObstacleClearZone* clear_zone,
                         ParkReferenceLine* ref_line,
                         std::shared_ptr<GridSearch> dp_map)
    : config_(open_space_conf),
      vehicle_param_(veh_param),
      obstacles_(obstacles),
      edt_(edt),
      ref_line_(ref_line),
      clear_zone_(clear_zone),
      dp_heuristic_generator_(dp_map) {
  collision_detect_ = std::make_shared<NodeCollisionDetect>(
      obstacles_, edt_, clear_zone_, &grid_map_bound_, &request_);

  polynomial_sampling_ = std::make_shared<PolynomialCurveSampling>(
      &grid_map_bound_, obstacles_, &request_, edt_, clear_zone_, ref_line_,
      &config_, vehicle_param_.min_turn_radius, collision_detect_);
  rs_sampling_ = std::make_shared<RSSampling>(
      &grid_map_bound_, obstacles_, &request_, edt_, clear_zone_, ref_line_,
      &config_, vehicle_param_.min_turn_radius, collision_detect_);
  spiral_sampling_ = std::make_shared<SpiralSampling>(
      &grid_map_bound_, obstacles_, &request_, edt_, clear_zone_, ref_line_,
      &config_, vehicle_param_.min_turn_radius, collision_detect_);
}

bool HybridAStar::CalcRSPathToGoal(Node3d* current_node,
                                   const bool need_rs_dense_point,
                                   const bool need_anchor_point,
                                   const RSPathRequestType rs_request,
                                   const float rs_radius) {
#if LOG_TIME_PROFILE
  double rs_start_time = IflyTime::Now_ms();
#endif

  const Pose2f& start_pose = current_node->GetPose();
  const Pose2f& end_pose = rs_expansion_decider_.GetRSEndPose();

  bool is_connected_to_goal;
  rs_path_interface_.GeneShortestRSPath(
      &rs_path_, &is_connected_to_goal, &start_pose, &end_pose, rs_radius,
      need_rs_dense_point, need_anchor_point, rs_request);

#if LOG_TIME_PROFILE
  float rs_end_time = IflyTime::Now_ms();
  rs_time_ms_ += rs_end_time - rs_start_time;
#endif

  if (rs_path_.total_length < 0.01 || !is_connected_to_goal) {
    ILOG_INFO << " path is short";
    return false;
  }

  return true;
}

bool HybridAStar::AnalyticExpansionByRS(Node3d* current_node,
                                        const AstarPathGear node_gear_request,
                                        Node3d* rs_node_to_goal) {
  // check gear and steering wheel
  if (!rs_expansion_decider_.IsNeedRsExpansion(current_node, &request_)) {
    // ILOG_INFO << "no need rs path link";
    return false;
  }

  const float rs_radius = vehicle_param_.min_turn_radius + 0.2;

  RSPathRequestType rs_request = RSPathRequestType::GEAR_SWITCH_LESS_THAN_TWICE;
  if (!CalcRSPathToGoal(current_node, false, false, rs_request, rs_radius)) {
    ILOG_INFO << " generate rs fail";

    return false;
  }

  // set node by rs path
  if (rs_path_.size < 1) {
    return false;
  }

  // todo: move to rs link decision
  // request check
  if (request_.first_action_request.has_request) {
    // start node rs gear is not expectation
    if (current_node->IsStartNode()) {
      if (IsGearDifferent(request_.first_action_request.gear_request,
                          rs_path_.paths[0].gear)) {
        rs_node_to_goal->ClearPath();

        // ILOG_INFO << "gear is not expectation";
        return false;
      }
    }

    // search node rs gear is not expectation
    if (current_node->GetGearType() ==
            request_.first_action_request.gear_request &&
        current_node->GetDistToStart() <
            request_.first_action_request.dist_request) {
      if (IsGearDifferent(request_.first_action_request.gear_request,
                          rs_path_.paths[0].gear)) {
        rs_node_to_goal->ClearPath();

        // ILOG_INFO << "gear is not expectation";
        return false;
      } else {
        if ((current_node->GetDistToStart() + rs_path_.GetFirstGearLength()) <
            request_.first_action_request.dist_request) {
          // ILOG_INFO << "dist is not expectation";
          return false;
        }
      }
    }

    // search node gear is not expectation
    if (current_node->GetGearSwitchNum() == 0 &&
        current_node->IsPathGearChange(
            request_.first_action_request.gear_request)) {
      rs_node_to_goal->ClearPath();

      // ILOG_INFO << "gear is not expectation";
      return false;
    }
  }

  // todo, need to get backward pass dist in all nodes.
  float parent_node_path_dist = current_node->GetNodePathDistance();
  if (current_node->GetGearType() != rs_path_.paths[0].gear) {
    parent_node_path_dist = 0;
  }

  // length check
  if (!IsRsPathFirstSegmentLongEnough(&rs_path_, parent_node_path_dist)) {
    // ILOG_INFO << "length is not expectation";
    return false;
  }

  // last segment gear check
  if (!RsLastSegmentSatisfyRequest(&rs_path_)) {
    // ILOG_INFO << "gear is not expectation";
    return false;
  }

  // gear check
  if (!CheckRSPathGear(&rs_path_, node_gear_request)) {
    // ILOG_INFO << "gear is not expectation";
    return false;
  }

  // interpolation
#if LOG_TIME_PROFILE
  double rs_start_time = IflyTime::Now_ms();
#endif

  const Pose2f& start_pose = current_node->GetPose();
  rs_path_interface_.RSPathInterpolate(&rs_path_, &start_pose, rs_radius);

#if LOG_TIME_PROFILE
  double rs_end_time = IflyTime::Now_ms();
  rs_interpolate_time_ms_ += rs_end_time - rs_start_time;
#endif

  NodePath path;
  path.path_dist = std::fabs(rs_path_.total_length);
  path.point_size = 1;
  // use first path end point to fill astar node
  RSPoint rs_end_point;
  rs_path_.FirstPathEndPoint(&rs_end_point);

  path.points[0].x = rs_end_point.x;
  path.points[0].y = rs_end_point.y;
  path.points[0].theta = IflyUnifyTheta(rs_end_point.theta, M_PIf32);

  rs_node_to_goal->Set(path, grid_map_bound_, config_, path.path_dist);
  if (!NodeInSearchBound(rs_node_to_goal->GetIndex())) {
    // ILOG_INFO << "node positiong is not expectation";
    rs_node_to_goal->ClearPath();
    return false;
  }

  rs_node_to_goal->SetPathType(AstarPathType::REEDS_SHEPP);
  rs_node_to_goal->SetGearType(rs_path_.paths[0].gear);

  // collision check
  if (!collision_detect_->RSPathCollisionCheck(&rs_path_, rs_node_to_goal)) {
    rs_node_to_goal->ClearPath();

    // ILOG_INFO << "rs collision";
    return false;
  }

  // ILOG_INFO << "Reach the end configuration with Reeds Shepp";

  rs_node_to_goal->SetPre(current_node);

  if (!rs_node_to_goal->IsNodeValid()) {
    // ILOG_INFO << "node invalid";
    return false;
  }

  float gcost =
      CalcRSGCostToParentNode(current_node, rs_node_to_goal, &rs_path_);
  rs_node_to_goal->SetGCost(current_node->GetGCost() + gcost);
  rs_node_to_goal->SetHeuCost(0.0);
  rs_node_to_goal->SetFCost();

#if PLOT_RS_EXNTEND_TO_END
  if (rs_path_h_cost_debug_.size() < RS_H_COST_MAX_NUM &&
      current_node->GetGearSwitchNum() <= 1 &&
      current_node->GetGearType() == AstarPathGear::DRIVE) {
    rs_path_h_cost_debug_.emplace_back(rs_path_);
  }
#endif

  // DebugRSPath(&rs_path_);

  return true;
}

bool HybridAStar::IsAllPathSegmentLongEnough(const RSPath* reeds_shepp_to_end,
                                             const float father_node_dist) {
  float same_gear_path_min_dist;

  // 双指针搜索
  int rs_path_seg_size = reeds_shepp_to_end->size;
  int left_pointer_id = 0;
  int right_pointer_id = 1;

  while (left_pointer_id < rs_path_seg_size) {
    AstarPathGear gear = reeds_shepp_to_end->paths[left_pointer_id].gear;

    same_gear_path_min_dist =
        std::fabs(reeds_shepp_to_end->paths[left_pointer_id].length);
    if (left_pointer_id == 0) {
      same_gear_path_min_dist += father_node_dist;
    }

    // ILOG_INFO << "left_pointer_id " << left_pointer_id;

    // search same gear path
    for (right_pointer_id = left_pointer_id + 1;
         right_pointer_id < rs_path_seg_size; right_pointer_id++) {
      // same gear
      if (gear == reeds_shepp_to_end->paths[right_pointer_id].gear) {
        same_gear_path_min_dist +=
            std::fabs(reeds_shepp_to_end->paths[right_pointer_id].length);

      } else {
        break;
      }
    }

    left_pointer_id = right_pointer_id;

    if (same_gear_path_min_dist < config_.rs_path_seg_advised_dist) {
      ILOG_INFO << " rs path seg len " << same_gear_path_min_dist;
      return false;
    }
  }

  return true;
}

bool HybridAStar::IsRsPathFirstSegmentLongEnough(
    const RSPath* reeds_shepp_to_end, const float father_node_dist) {
  float len = reeds_shepp_to_end->GetFirstGearLength() + father_node_dist;

  if (len < config_.rs_path_seg_advised_dist) {
    // ILOG_INFO << " rs path first seg len " << len;
    return false;
  }

  return true;
}

bool HybridAStar::RsLastSegmentSatisfyRequest(
    const RSPath* reeds_shepp_to_end) {
  int rs_path_seg_size = reeds_shepp_to_end->size;
  if (rs_path_seg_size < 1) {
    return true;
  }

  AstarPathGear first_gear = reeds_shepp_to_end->paths[0].gear;
  const AstarPathGear last_gear =
      reeds_shepp_to_end->paths[rs_path_seg_size - 1].gear;

  if (request_.space_type == ParkSpaceType::VERTICAL) {
    if (request_.direction_request == ParkingVehDirection::TAIL_IN &&
        request_.rs_request == RSPathRequestType::LAST_PATH_FORBID_FORWARD) {
      if (last_gear == AstarPathGear::DRIVE) {
        // ILOG_INFO << " rs path last seg len is drive gear ";

        return false;
      }
    } else if (request_.direction_request == ParkingVehDirection::HEAD_IN) {
      if (request_.rs_request == RSPathRequestType::LAST_PATH_FORBID_REVERSE &&
          last_gear == AstarPathGear::REVERSE) {
        // ILOG_INFO << " rs path last seg len is reverse gear ";
        return false;
      } else if (first_gear == AstarPathGear::DRIVE) {
        int i = 1;
        while (i < rs_path_seg_size && first_gear == AstarPathGear::DRIVE) {
          first_gear = reeds_shepp_to_end->paths[i].gear;
          i++;
        }
        const RSPoint first_drive_path_end_pos =
            rs_path_interface_.GetAnchorPoint().points[i];
        // ILOG_INFO << "first drive end pos = " << first_drive_path_end_pos.x
        //           << ", " << first_drive_path_end_pos.y;

        if (first_drive_path_end_pos.x < astar_end_node_->GetX() - 0.15 &&
            std::fabs(first_drive_path_end_pos.y) <
                config_.headin_limit_y_shrink) {
          return false;
        }
      }
    }
  }

  return true;
}

bool HybridAStar::CheckRSPathGear(const RSPath* reeds_shepp_to_end,
                                  const AstarPathGear gear_request_info) {
  int rs_path_seg_size = reeds_shepp_to_end->size;
  if (rs_path_seg_size < 1) {
    return true;
  }

  // delete gear switch bigger than 1.
  if (reeds_shepp_to_end->gear_change_number > 1) {
    return false;
  }

  AstarPathGear gear;

  for (int i = 0; i < rs_path_seg_size; i++) {
    gear = reeds_shepp_to_end->paths[i].gear;

    if (request_.space_type == ParkSpaceType::VERTICAL) {
      if (request_.rs_request == RSPathRequestType::ALL_PATH_FORBID_FORWARD &&
          gear == AstarPathGear::DRIVE) {
        // ILOG_INFO << " rs path seg need single shot by reverse gear ";
        return false;

      } else if (request_.rs_request ==
                     RSPathRequestType::ALL_PATH_FORBID_REVERSE &&
                 gear == AstarPathGear::REVERSE) {
        // ILOG_INFO << " rs path seg need single shot by drive gear ";
        return false;
      }

      if (gear_request_info == AstarPathGear::REVERSE &&
          gear == AstarPathGear::DRIVE) {
        // ILOG_INFO << "gear is not expectation";
        return false;
      } else if (gear_request_info == AstarPathGear::DRIVE &&
                 gear == AstarPathGear::REVERSE) {
        // ILOG_INFO << "gear is not expectation";
        return false;
      }
    }
  }

  return true;
}

void HybridAStar::GetPathByBicycleModel(NodePath* path, const float arc,
                                        const float radius,
                                        const bool is_forward) {
  int path_point_num = std::ceil(arc / node_path_dist_resolution_);

  int kinetics_model_number =
      std::round(node_path_dist_resolution_ / kinetics_model_step_);

  path->path_dist = 0;

  Pose2f* old_point;
  old_point = &path->points[0];

  for (int i = 0; i < path_point_num; ++i) {
    UpdatePoseBySamplingNumber(old_point, radius, kinetics_model_number,
                               &path->points[path->point_size], is_forward);

    old_point = &path->points[path->point_size];

    path->point_size++;
    path->path_dist += node_path_dist_resolution_;
    if (path->point_size >= NODE_PATH_MAX_POINT) {
      ILOG_INFO << "size too much";
      break;
    }
  }

  return;
}

void HybridAStar::GetPathByCircle(NodePath* path, const float arc,
                                  const float radius, const bool is_forward) {
  int path_point_num = std::ceil(arc / node_path_dist_resolution_);

  // get vehicle circle
  VehicleCircle veh_circle;
  AstarPathGear gear;
  if (is_forward) {
    gear = AstarPathGear::DRIVE;
  } else {
    gear = AstarPathGear::REVERSE;
  }

  GetVehCircleByPose(&veh_circle, &path->points[0], radius, gear);

  // interpolate
  path->path_dist = 0;

  Pose2f* start_pose;
  Pose2f* next_pose;
  start_pose = &path->points[0];

  float acc_s = 0.0;

  for (int i = 0; i < path_point_num; ++i) {
    next_pose = &path->points[path->point_size];
    acc_s += node_path_dist_resolution_;

    InterpolateByArcOffset(next_pose, &veh_circle, start_pose, acc_s,
                           inv_radius_);

    path->point_size++;
    path->path_dist += node_path_dist_resolution_;

    if (path->point_size >= NODE_PATH_MAX_POINT) {
      ILOG_INFO << "size too much";
      break;
    }
  }

  return;
}

void HybridAStar::GetPathByLine(NodePath* path, const float arc,
                                const bool is_forward) {
  int path_point_num = std::ceil(arc / node_path_dist_resolution_);

  float inc_dist;
  if (is_forward) {
    inc_dist = node_path_dist_resolution_;
  } else {
    inc_dist = -node_path_dist_resolution_;
  }

  path->path_dist = 0;
  float acc_s = 0.0;

  // get unit vector
  Pose2f unit_vector;
  unit_vector.x = std::cos(path->points[0].GetPhi());
  unit_vector.y = std::sin(path->points[0].GetPhi());

  Pose2f* start_pose = &path->points[0];
  Pose2f* next_pose;

  for (int j = 0; j < path_point_num; j++) {
    next_pose = &path->points[path->point_size];
    acc_s += inc_dist;

    GetStraightLinePoint(next_pose, start_pose, acc_s, &unit_vector);

    // ILOG_INFO << "start " << state_next->x << " " << state_next->y << " acc_s
    // "
    //           << acc_s;

    path->point_size++;
    path->path_dist += node_path_dist_resolution_;

    if (path->point_size >= NODE_PATH_MAX_POINT) {
      ILOG_INFO << "size too much";
      break;
    }
  }

  return;
}

const NodeShrinkType HybridAStar::NextNodeGenerator(
    Node3d* new_node, Node3d* parent_node, size_t next_node_index,
    const AstarPathGear gear_request_info) {
  float front_wheel_angle = 0.0;
  float radius = 0.0;
  float traveled_distance = 0.0;

  // clear
  new_node->ClearPath();

  // update angle
  if (next_node_index < next_node_angles_.size) {
    front_wheel_angle = next_node_angles_.angles[next_node_index];
    radius = next_node_angles_.radius[next_node_index];
    traveled_distance = node_path_dist_resolution_;
  } else if (next_node_index < next_node_angles_.size * 2) {
    size_t index = next_node_index - next_node_angles_.size;
    front_wheel_angle = next_node_angles_.angles[index];
    radius = next_node_angles_.radius[index];
    traveled_distance = -node_path_dist_resolution_;
  } else {
    return NodeShrinkType::NONE;
  }

  // gear check
  if (gear_request_info == AstarPathGear::REVERSE && traveled_distance > 0.0) {
    return NodeShrinkType::UNEXPECTED_GEAR;
  } else if (gear_request_info == AstarPathGear::DRIVE &&
             traveled_distance < 0.0) {
    return NodeShrinkType::UNEXPECTED_GEAR;
  }

  if (parent_node->IsStartNode()) {
    if (request_.first_action_request.gear_request == AstarPathGear::DRIVE &&
        traveled_distance < 0.0) {
      return NodeShrinkType::UNEXPECTED_GEAR;
    } else if (request_.first_action_request.gear_request ==
                   AstarPathGear::REVERSE &&
               traveled_distance > 0.0) {
      return NodeShrinkType::UNEXPECTED_GEAR;
    }
  }

  // take above motion primitive to generate a curve driving the car to a
  // different grid
  // float node_step = std::sqrt(2) * xy_grid_resolution_;
  float node_step = config_.node_step;

  NodePath path;
  path.point_size = 1;
  path.points[0].x = parent_node->GetX();
  path.points[0].y = parent_node->GetY();
  path.points[0].theta = parent_node->GetPhi();

  bool is_forward = traveled_distance > 0.0 ? true : false;

  // generate path by bycicle model
  if (std::fabs(front_wheel_angle) > 0.0001) {
    GetPathByBicycleModel(&path, node_step, radius, is_forward);
  } else {
    GetPathByLine(&path, node_step, is_forward);
  }

  // check if the vehicle runs outside of XY boundary
  const Pose2f& end_point = path.GetEndPoint();
  if (collision_detect_->IsPointBeyondBound(end_point.x, end_point.y)) {
    return NodeShrinkType::OUT_OF_BOUNDARY;
  }

  new_node->Set(path, grid_map_bound_, config_, path.path_dist);

  // check search bound
  if (!NodeInSearchBound(new_node->GetIndex())) {
    new_node->ClearPath();
    return NodeShrinkType::OUT_OF_BOUNDARY;
  }

  bool heading_legal = false;
  heading_legal = node_shrink_decider_.IsLegalForHeading(new_node->GetPhi());
  if (!heading_legal) {
#if PLOT_DELETE_NODE
    delete_queue_path_debug_.emplace_back(
        Vec2f(new_node->GetX(), new_node->GetY()));
#endif
    // ILOG_INFO << "heading is illegal";
    new_node->ClearPath();
    return NodeShrinkType::UNEXPECTED_HEADING;
  }

  // headin shrink limit pose
  if (!node_shrink_decider_.IsLegalByXBound(new_node->GetX())) {
#if PLOT_DELETE_NODE
    delete_queue_path_debug_.emplace_back(
        Vec2f(new_node->GetX(), new_node->GetY()));
#endif
    // ILOG_INFO << "pos is illegal";
    new_node->ClearPath();
    return NodeShrinkType::UNEXPECTED_POS;
  }

  new_node->SetPre(parent_node);

  AstarPathGear gear;
  if (traveled_distance > 0.0) {
    gear = AstarPathGear::DRIVE;
  } else {
    gear = AstarPathGear::REVERSE;
  }
  new_node->SetGearType(gear);
  if (parent_node->IsPathGearChange(gear)) {
    new_node->SetGearSwitchNum(parent_node->GetGearSwitchNum() + 1);
  } else {
    new_node->SetGearSwitchNum(parent_node->GetGearSwitchNum());
  }

  new_node->SetSteer(front_wheel_angle);
  new_node->SetPathType(AstarPathType::NODE_SEARCHING);
  new_node->SetDistToStart(path.path_dist + parent_node->GetDistToStart());
  new_node->SetRadius(radius);

  if (new_node->GetGearSwitchNum() == 0) {
    new_node->SetGearSwitchNode(nullptr);
  } else if (new_node->GetGearSwitchNum() == 1 &&
             parent_node->IsPathGearChange(gear)) {
    new_node->SetGearSwitchNode(parent_node);
  } else {
    new_node->SetGearSwitchNode(parent_node->GearSwitchNode());
  }

  // ILOG_INFO << "next node end";
  // new_node->GetPose().DebugString();

  return NodeShrinkType::NONE;
}

void HybridAStar::CalculateNodeFCost(Node3d* current_node, Node3d* next_node) {
  CalculateNodeGCost(current_node, next_node);

  CalculateNodeHeuristicCost(current_node, next_node);

  return;
}

void HybridAStar::CalculateNodeHeuristicCost(Node3d* father_node,
                                             Node3d* next_node) {
#if LOG_TIME_PROFILE
  const double start_time = IflyTime::Now_ms();
#endif

  NodeHeuristicCost cost;
  // evaluate heuristic cost
  float optimal_path_cost = 0.0;
  float dp_path_dist = 0.0;

  float dp_path_cost = 0.0;
  dp_path_dist = ObstacleHeuristicWithHolonomic(next_node);
  dp_path_cost = dp_path_dist * config_.traj_forward_penalty;
  cost.astar_dist = dp_path_cost;

  float rs_path_cost = 0.0;
  rs_path_cost = GenerateHeuristicCostByRsPath(next_node, &cost);
  optimal_path_cost = std::max(dp_path_cost, rs_path_cost);

  // heading cost
  float ref_line_heading_cost = 0.0;
  ref_line_heading_cost = GenerateRefLineHeuristicCost(next_node, dp_path_dist);
  cost.ref_line_heading_cost = ref_line_heading_cost;

  optimal_path_cost = std::max(optimal_path_cost, ref_line_heading_cost);
  // optimal_path_cost += ref_line_heading_cost;

  // euler cost
  float euler_dist_cost = 0.0;
  euler_dist_cost = next_node->GetEulerDist(astar_end_node_);
  cost.euler_dist = euler_dist_cost;

  optimal_path_cost = std::max(euler_dist_cost, optimal_path_cost);

  next_node->SetHeuCost(optimal_path_cost);
  // next_node->SetHeuCostDebug(cost);

#if LOG_TIME_PROFILE
  const double end_time = IflyTime::Now_ms();
  heuristic_time_ += end_time - start_time;
#endif

  return;
}

void HybridAStar::GetSingleShotNodeHeuCost(const Node3d* father_node,
                                           Node3d* next_node) {
  NodeHeuristicCost cost;
  float optimal_path_cost = 0.0;

  // euler cost
  float euler_dist_cost = 0.0;
  euler_dist_cost = next_node->GetEulerDist(astar_end_node_);
  cost.euler_dist = euler_dist_cost;

  optimal_path_cost = std::max(euler_dist_cost, optimal_path_cost);

  next_node->SetHeuCost(optimal_path_cost);
  // next_node->SetHeuCostDebug(cost);

  return;
}

void HybridAStar::CalculateNodeGCost(Node3d* current_node, Node3d* next_node) {
  next_node->SetGCost(current_node->GetGCost() +
                      CalcGCostToParentNode(current_node, next_node));

  return;
}

float HybridAStar::GenerateHeuristicCostByRsPath(Node3d* next_node,
                                                 NodeHeuristicCost* cost) {
  RSPathRequestType rs_request = RSPathRequestType::NONE;
  if (!CalcRSPathToGoal(next_node, false, false, rs_request,
                        vehicle_param_.min_turn_radius)) {
    ILOG_INFO << "ShortestRSP failed";
    return 100.0;
  }

  float path_dist = std::fabs(rs_path_.total_length);

  float dist_cost = path_dist * config_.traj_forward_penalty;
  cost->rs_path_dist = dist_cost;

#if PLOT_RS_COST_PATH
  if (rs_path_h_cost_debug_.size() < RS_H_COST_MAX_NUM) {
    const Pose2f& rs_start_pose = next_node->GetPose();
    rs_path_interface_.RSPathInterpolate(&rs_path_, &rs_start_pose,
                                         vehicle_param_.min_turn_radius);
    rs_path_h_cost_debug_.emplace_back(rs_path_);
  }
#endif

  return dist_cost;
}

float HybridAStar::CalcGCostToParentNode(Node3d* current_node,
                                         Node3d* next_node) {
  // 1. evaluate cost on the trajectory and add current cost
  float piecewise_cost = 0.0f;
  float path_dist = next_node->GetNodePathDistance();

  if (next_node->IsForward()) {
    piecewise_cost += path_dist * config_.traj_forward_penalty;
  } else {
    piecewise_cost += path_dist * config_.traj_reverse_penalty;
  }

  // 2. gear punish
  if (current_node->IsPathGearChange(next_node->GetGearType())) {
    piecewise_cost += config_.gear_switch_penalty;
  }

  // 3. Turning cost (direction size + change amount + zigzag), todo::
  // currently, it is only targeted at parking out
  if (!current_node->IsStartNode() &&
      IsParkingOutRequest(request_.direction_request)) {
    const float steer_now = next_node->GetSteer();
    const float steer_last = current_node->GetSteer();
    const float steer_delta = steer_now - steer_last;
    const float steer_change = std::fabs(steer_delta);
    const bool steer_sign_changed = (steer_now * steer_last < 0.0f);
    const float steer_abs = std::fabs(steer_now);
    const bool gear_change =
        (current_node->IsPathGearChange(next_node->GetGearType()));

    // 3.1 Steering angle penalty (large steering is discouraged, but not
    // restricted)
    piecewise_cost += config_.traj_steer_penalty * steer_abs;

    // 3.2 Steering change penalty:
    // Strong penalty for small changes to control
    // zigzag; large changes are allowed to avoid penalizing paths with large
    // curvature and increasing the number of gear shifts
    if (!gear_change) {
      constexpr float steer_change_threshold = 0.2f;  // 允许突变的最大变化
      float effective_change = std::min(steer_change, steer_change_threshold);

      piecewise_cost += config_.traj_steer_change_penalty * effective_change;
    }

    // 3.3 Zigzag penalty: directional jump change with an angle exceeding a
    // certain value
    if (steer_sign_changed && steer_change > 0.1f && !gear_change) {
      piecewise_cost += config_.zigzag_penalty;
    }
  }

  // 4. request dist and gear cost
  if (request_.plan_reason == PlanningReason::FIRST_PLAN) {
    if (current_node->GetDistToStart() <
        request_.first_action_request.dist_request) {
      if (current_node->IsPathGearChange(next_node->GetGearType())) {
        piecewise_cost += config_.expect_dist_penalty;
      }
    }
  } else {
    if (next_node->GetDistToStart() <
            request_.first_action_request.dist_request ||
        current_node->GetDistToStart() <
            request_.first_action_request.dist_request) {
      // gear is different
      if (next_node->IsPathGearChange(
              request_.first_action_request.gear_request)) {
        piecewise_cost += config_.expect_dist_penalty;
      }
    }
  }

  // safe dist cost
#if ENABLE_OBS_DIST_G_COST
  float safe_punish = 0.0;
  safe_punish = CalcSafeDistCost(next_node);
  piecewise_cost += safe_punish;
#endif

  // box cost
  if (!collision_detect_->IsContainByRecommendBox(next_node->GetPose())) {
    piecewise_cost += config_.recommend_box_penalty;
  }

  return piecewise_cost;
}

float HybridAStar::CalcRSGCostToParentNode(Node3d* current_node,
                                           Node3d* rs_node,
                                           const RSPath* rs_path) {
  rs_node->SetGearSwitchNum(current_node->GetGearSwitchNum());
  // evaluate cost on the trajectory and add current cost
  float piecewise_cost = 0.0;
  float path_dist = rs_path->total_length;
  piecewise_cost += path_dist * config_.traj_forward_penalty;

  float gear_cost = 0.0;
  float steer_change_cost = 0.0;
  float box_cost = 0.0;
  for (int i = 0; i < rs_path_.size - 1; i++) {
    // gear cost
    if (rs_path_.paths[i].gear != rs_path_.paths[i + 1].gear) {
      gear_cost += config_.gear_switch_penalty;

      rs_node->AddGearSwitchNumber();
    }

    // steer change cost
    if (rs_path_.paths[i].steer != rs_path_.paths[i + 1].steer) {
      if (rs_path_.paths[i].steer == RS_STRAIGHT ||
          rs_path_.paths[i + 1].steer == RS_STRAIGHT) {
        steer_change_cost +=
            config_.traj_steer_change_penalty * max_steer_angle_;
      } else {
        steer_change_cost +=
            config_.traj_steer_change_penalty * max_steer_angle_ * 2;
      }
    }

    // box cost
    if (rs_path_.paths[i].size > 0 &&
        !collision_detect_->IsContainByRecommendBox(
            rs_path_.paths[i].points[0])) {
      box_cost += config_.recommend_box_penalty;
    }
  }

  piecewise_cost += box_cost;

  // gear cost
  bool gear_switch = current_node->IsPathGearChange(rs_path_.paths[0].gear);
  if (gear_switch) {
    gear_cost += config_.gear_switch_penalty;
    rs_node->AddGearSwitchNumber();
  }
  piecewise_cost += gear_cost;

  // steer cost
  if (current_node->GetSteer() > 0.0 && rs_path_.paths[0].steer != RS_LEFT) {
    steer_change_cost += config_.traj_steer_change_penalty * max_steer_angle_;
  } else if (current_node->GetSteer() < 0.0 &&
             rs_path_.paths[0].steer != RS_RIGHT) {
    steer_change_cost += config_.traj_steer_change_penalty * max_steer_angle_;
  }
  piecewise_cost += steer_change_cost;

  // request dist and gear cost
  if (current_node->GetDistToStart() <
      request_.first_action_request.dist_request) {
    if (current_node->IsPathGearChange(rs_path_.paths[0].gear)) {
      piecewise_cost += config_.expect_dist_penalty;
    } else if (current_node->GetDistToStart() +
                   std::fabs(rs_path_.paths[0].length) <
               request_.first_action_request.dist_request) {
      piecewise_cost += config_.expect_dist_penalty;
    }
  }

  rs_node->SetDistToStart(path_dist + current_node->GetDistToStart());

  if (current_node->GetGearSwitchNum() > 0) {
    rs_node->SetGearSwitchNode(current_node->GearSwitchNode());
  } else if (rs_node->GetGearSwitchNum() == 0) {
    rs_node->SetGearSwitchNode(nullptr);
  } else if (gear_switch) {
    // 正常节点没有换档，rs起点换档
    rs_node->SetGearSwitchNode(current_node);
  } else {
    // rs 起点没有换档
    rs_node->SetGearSwitchNode(nullptr);
  }

  return piecewise_cost;
}

void HybridAStar::GetSingleShotNodeGCost(Node3d* current_node,
                                         Node3d* next_node) {
  // evaluate cost on the trajectory and add current cost
  float piecewise_cost = 0.0;
  float path_dist = 0.0;
  path_dist = next_node->GetNodePathDistance();

  if (next_node->IsForward()) {
    piecewise_cost += path_dist * config_.traj_forward_penalty;
  } else {
    piecewise_cost += path_dist * config_.traj_reverse_penalty;
  }

  // steering wheel angle cost
  // for start node, steering angle can be any value
  if (!current_node->IsStartNode()) {
    piecewise_cost += 0.0 * std::fabs(next_node->GetSteer());

    // steering wheel change
    piecewise_cost +=
        1.0 * std::fabs(next_node->GetSteer() - current_node->GetSteer());
  }

  // box cost
  if (!collision_detect_->IsContainByRecommendBox(next_node->GetPose())) {
    piecewise_cost += config_.recommend_box_penalty;
  }

  next_node->SetGCost(current_node->GetGCost() + piecewise_cost);

  return;
}

float HybridAStar::ObstacleHeuristicWithHolonomic(Node3d* next_node) {
  return dp_heuristic_generator_->CheckDpMap(next_node->GetX(),
                                             next_node->GetY());
}

float HybridAStar::GenerateHeuristicCost(Node3d* next_node) {
  float h_cost = 0.0;

  h_cost += ObstacleHeuristicWithHolonomic(next_node);

  return h_cost;
}

float HybridAStar::GenerateRefLineHeuristicCost(Node3d* next_node,
                                                const float dist_to_go) {
  // heading cost
  float theta1 = next_node->GetPhi();
  float theta2 = ref_line_->GetHeading();

  float heading_cost_weight = 2.0;
  float heading_cost = dist_to_go + std::fabs(Getf32ThetaDiff(theta1, theta2)) *
                                        heading_cost_weight;

#if DEBUG_REF_LINE_COST
  ILOG_INFO << "node heading = " << next_node->GetPhi() * 57.3
            << " heading cost " << heading_cost
            << ", ref line heading =" << theta2 * 57.3;
#endif

  return heading_cost;
}

const bool HybridAStar::BackwardPassByNode(
    HybridAStarResult* result, Node3d* best_node, const RSPath* rs_path,
    const std::vector<AStarPathPoint>& poly_path) {
  if (best_node == nullptr) {
    result->Clear();
    return false;
  }

  Node3d* parent_node = nullptr;
  Node3d* child_node = best_node;
  best_node->SetNext(nullptr);

  result->base_pose = request_.base_pose;
  result->gear_change_num = 0;

  // debug all nodes
  std::vector<Node3d*> node_list;

  // backward pass
  while (child_node->GetPreNode() != nullptr) {
    parent_node = child_node->GetPreNode();

    parent_node->SetNext(child_node);
    child_node = parent_node;
  }

  size_t point_size;
  float kappa;

  AstarPathGear last_gear_type = AstarPathGear::NONE;
  AstarPathGear cur_gear_type;
  while (child_node != nullptr) {
    // break
    if (child_node->GetPathType() == AstarPathType::REEDS_SHEPP ||
        child_node->GetPathType() == AstarPathType::SPIRAL ||
        child_node->GetPathType() == AstarPathType::CUBIC_POLYNOMIAL ||
        child_node->GetPathType() == AstarPathType::QUNTIC_POLYNOMIAL) {
      break;
    }

    cur_gear_type = child_node->GetGearType();
    CopyNodePath(child_node, result);

    // check gear switch number
    if (last_gear_type != AstarPathGear::NONE) {
      if (last_gear_type != cur_gear_type) {
        result->gear_change_num++;
      }
    }

    node_list.push_back(child_node);

    last_gear_type = cur_gear_type;
    parent_node = child_node;
    child_node = child_node->GetMutableNextNode();
  }

  // get rs path
  AstarPathType path_type = AstarPathType::NONE;
  if (child_node != nullptr && child_node->IsRsPath() && rs_path != nullptr) {
    path_type = child_node->GetPathType();
    for (int seg_id = 0; seg_id < rs_path->size; seg_id++) {
      const RSPathSegment* segment = &rs_path->paths[seg_id];

      if (segment->size < 1) {
        ILOG_ERROR << "result size check failed";
        continue;
      }

      cur_gear_type = segment->gear;
      kappa = segment->kappa;
      for (int k = 0; k < segment->size; k++) {
        result->x.emplace_back(segment->points[k].x);
        result->y.emplace_back(segment->points[k].y);
        result->phi.emplace_back(segment->points[k].theta);
        result->type.emplace_back(path_type);
        result->gear.emplace_back(cur_gear_type);
        result->kappa.emplace_back(kappa);
      }

      // check gear switch number
      if (last_gear_type != AstarPathGear::NONE) {
        if (last_gear_type != cur_gear_type) {
          result->gear_change_num++;
        }
      }

      last_gear_type = cur_gear_type;
    }

    ILOG_INFO << "get result start backward pass by rs";
  }
  // get path
  else if (child_node != nullptr && child_node->IsQunticPolynomialPath()) {
    if (poly_path.size() < 1) {
      return false;
    }

    path_type = child_node->GetPathType();

    for (size_t k = 0; k < poly_path.size(); k++) {
      result->x.emplace_back(poly_path[k].x);
      result->y.emplace_back(poly_path[k].y);
      result->phi.emplace_back(poly_path[k].phi);
      result->type.emplace_back(path_type);
      result->gear.emplace_back(poly_path[k].gear);
      result->kappa.emplace_back(poly_path[k].kappa);
    }

    ILOG_INFO << "get result start backward pass by polynomial";
  }

  ReversePathBySwapStartGoal(result);

  // get path lengh
  UpdatePathS(result);
  result->fail_type = AstarFailType::SUCCESS;

  // DebugPathString(result);
  ILOG_INFO << "get result finish, path point size " << result->x.size();

#if DEBUG_SEARCH_RESULT
  DebugNodeList(node_list);
#endif

  return true;
}

void HybridAStar::OneShotPathAttempt(const MapBound& XYbounds,
                                     const Pose2f& start, const Pose2f& target,
                                     HybridAStarResult* result) {
  result->Clear();
  if (request_.first_action_request.gear_request == AstarPathGear::DRIVE &&
      !IsNeedGearDriveSearch(start)) {
    return;
  }

  if (request_.first_action_request.gear_request == AstarPathGear::REVERSE &&
      !IsNeedGearReverseSearch(start)) {
    return;
  }

  AstarPathGear full_path_gear_request;
  if (request_.direction_request == ParkingVehDirection::HEAD_IN ||
      request_.path_generate_method ==
          AstarPathGenerateType::GEAR_DRIVE_SEARCHING) {
    full_path_gear_request = AstarPathGear::DRIVE;
  } else {
    full_path_gear_request = AstarPathGear::REVERSE;
  }

  // clear containers
  ResetNodePool();
  open_pq_.clear();
  node_set_.clear();

  // debug
  child_node_debug_.clear();
  queue_path_debug_.clear();
  delete_queue_path_debug_.clear();
  rs_path_h_cost_debug_.clear();
  rs_path_.Clear();

  collision_check_time_ms_ = 0.0;
  rs_time_ms_ = 0.0;
  rs_interpolate_time_ms_ = 0.0;
  ILOG_INFO << "one shot searching begin";

  // load XYbounds
  grid_map_bound_ = XYbounds;
  // check bound
  UpdateMaxGridIndex();

  // start
  start_node_ = node_pool_.AllocateNode();

  // load nodes and obstacles
  if (start_node_ == nullptr) {
    ILOG_ERROR << "start node nullptr";

    result->fail_type = AstarFailType::ALLOCATE_NODE_FAIL;
    return;
  }

  start_node_->Set(NodePath(start), grid_map_bound_, config_, 0.0);
  if (!start_node_->IsNodeValid()) {
    ILOG_ERROR << "start_node invalid";

    result->fail_type = AstarFailType::OUT_OF_BOUND;
    return;
  }

  // in searching, start node gear is forward or backward which will be ok.
  start_node_->SetGearType(AstarPathGear::NONE);
  start_node_->SetSteer(0.0);
  start_node_->SetIsStartNode(true);
  start_node_->SetPathType(AstarPathType::START_NODE);
  start_node_->SetDistToStart(0.0);
  start_node_->SetGearSwitchNum(0);
  start_node_->DebugString();
  // start node gcost is 0

  // check start
  double check_start_time = IflyTime::Now_ms();
  if (!collision_detect_->ValidityCheckByEDT(start_node_)) {
    // second check
    if (collision_detect_->IsFootPrintCollision(
            Transform2d(start_node_->GetPose()))) {
      ILOG_ERROR << "start_node in collision with obstacles "
                 << static_cast<int>(start_node_->GetConstCollisionType());

      // start_node_->DebugString();
      result->fail_type = AstarFailType::START_COLLISION;
      return;
    }
  }

  double check_end_time = IflyTime::Now_ms();
  collision_check_time_ms_ += check_end_time - check_start_time;

  // allocate end
  astar_end_node_ = node_pool_.AllocateNode();
  if (astar_end_node_ == nullptr) {
    ILOG_ERROR << "end node nullptr";

    result->fail_type = AstarFailType::OUT_OF_BOUND;
    return;
  }
  astar_end_node_->Set(NodePath(target), grid_map_bound_, config_, 0.0);
  astar_end_node_->SetGearType(AstarPathGear::NONE);
  astar_end_node_->SetPathType(AstarPathType::END_NODE);
  astar_end_node_->DebugString();

  if (!astar_end_node_->IsNodeValid()) {
    ILOG_ERROR << "end_node invalid";

    result->fail_type = AstarFailType::OUT_OF_BOUND;
    return;
  }

  // check end
  check_start_time = IflyTime::Now_ms();
  if (!collision_detect_->ValidityCheckByEDT(astar_end_node_)) {
    ILOG_INFO << "end_node in collision with obstacles "
              << static_cast<int>(astar_end_node_->GetConstCollisionType());

    astar_end_node_->DebugString();
    result->fail_type = AstarFailType::GOAL_COLLISION;
    return;
  }

  check_end_time = IflyTime::Now_ms();
  collision_check_time_ms_ += check_end_time - check_start_time;

  // node shrink related
  node_shrink_decider_.Process(start, target, request_.direction_request,
                               request_.real_goal, grid_map_bound_);
  rs_expansion_decider_.Process(
      vehicle_param_.min_turn_radius, request_.slot_width, request_.slot_length,
      start, target, vehicle_param_.width, request_.space_type,
      request_.direction_request);

  SetSamplingTarget(target);

  // load open set, pq
  start_node_->SetMultiMapIter(
      open_pq_.insert(std::make_pair(0.0, start_node_)));

  node_set_.emplace(start_node_->GetGlobalID(), start_node_);

  // a star searching
  size_t explored_node_num = 0;
  size_t explored_rs_path_num = 0;
  size_t h_cost_rs_path_num = 0;
  double astar_search_start_time = IflyTime::Now_ms();
  double astar_search_time;
  heuristic_time_ = 0.0;

  Node3d* current_node = nullptr;
  Node3d* next_node_in_pool = nullptr;
  Node3d* best_node = nullptr;
  Node3d new_node;
  Node3d rs_node_to_goal;
  rs_node_to_goal.Clear();

  AstarNodeVisitedType vis_type;

  bool is_safe = false;
  float child_node_dist;
  float father_node_dist;
  std::vector<AStarPathPoint> poly_path;
  Node3d polynomial_node;
  PolynomialPathErrorCode poly_path_fail_type;
  polynomial_node.Clear();

  PathComparator path_comparator;
  path_comparator.SetHeuristicPose(request_);

  while (!open_pq_.empty()) {
    // take out the lowest cost neighboring node

    current_node = open_pq_.begin()->second;
    open_pq_.erase(open_pq_.begin());
    if (current_node == nullptr) {
      ILOG_INFO << "pq is null node";
      continue;
    }

    current_node->SetVisitedType(AstarNodeVisitedType::IN_CLOSE);

    // generate best node
    if (best_node == nullptr) {
      best_node = current_node;
    } else if (path_comparator.NodeCompare(request_.real_goal, best_node,
                                           current_node)) {
      best_node = current_node;
    }

#if DEBUG_ONE_SHOT_PATH
    if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
      ILOG_INFO << "****************************";
      ILOG_INFO << "cycle, explored_node_num " << explored_node_num
                << " open set size for now " << open_pq_.size();
      current_node->DebugString();
    }
#endif

#if PLOT_SEARCH_SEQUENCE
    queue_path_debug_.emplace_back(
        Vec2f(current_node->GetX(), current_node->GetY()));
#endif

    // check if an analystic curve could be connected from current
    // configuration to the end configuration without collision. if so,
    // search ends.
    double current_time = IflyTime::Now_ms();

    // if bigger than 100 ms，break
    astar_search_time = current_time - astar_search_start_time;
    if (astar_search_time > config_.max_search_time_ms_for_no_gear_switch) {
      ILOG_INFO << "time out " << astar_search_time;
      break;
    }

    if (polynomial_sampling_->SamplingByQunticPolynomial(
            current_node, poly_path, &polynomial_node, &poly_path_fail_type)) {
      ILOG_INFO << "polynomial success";

      current_node->DebugPoseString();

      ILOG_INFO << "path " << poly_path[0].x << ",y " << poly_path[0].y
                << ",heading = " << poly_path[0].phi * 57.4
                << ",radius = " << current_node->GetRadius();

      break;
    } else if (rs_sampling_->SamplingByRSPath(current_node, &rs_node_to_goal)) {
      rs_path_ = rs_sampling_->GetConstRsPath();
      ILOG_INFO << "RS success";
      break;
    }

    explored_rs_path_num++;

    for (size_t i = 0; i < next_node_num_; ++i) {
      NextNodeGenerator(&new_node, current_node, i, full_path_gear_request);
      explored_node_num++;

      if (!new_node.IsNodeValid()) {
        continue;
      }

      child_node_dist = new_node.DistToPose(request_.real_goal);
      father_node_dist = current_node->DistToPose(request_.real_goal);

      // dist is bigger
      if (child_node_dist > father_node_dist - 0.001) {
        continue;
      }

      if (node_shrink_decider_.IsShrinkByStartNode(start_node_->GetGlobalID(),
                                                   &new_node)) {
        continue;
      }

      if (node_shrink_decider_.IsShrinkByParent(current_node, &new_node)) {
        continue;
      }

      // collision check
#if LOG_TIME_PROFILE
      check_start_time = IflyTime::Now_ms();
#endif

      is_safe = collision_detect_->ValidityCheckByEDT(&new_node);

#if LOG_TIME_PROFILE
      check_end_time = IflyTime::Now_ms();
      collision_check_time_ms_ += check_end_time - check_start_time;
#endif

      if (!is_safe) {
#if PLOT_CHILD_NODE
        // if node is unsafe, plot it also.
        if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
          child_node_debug_.emplace_back(
              DebugAstarSearchPoint(new_node.GetX(), new_node.GetY(), false));
        }
#endif

        continue;
      }

#if DEBUG_ONE_SHOT_PATH
      if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
        ILOG_INFO << "  ======================== ";
        ILOG_INFO << "  search child node cycle, open set size "
                  << open_pq_.size()
                  << " ,gear change num:" << current_node->GetGearSwitchNum();
        current_node->DebugString();
        new_node.DebugString();
      }
#endif

      // allocate new node
      if (node_set_.find(new_node.GetGlobalID()) == node_set_.end()) {
        next_node_in_pool = node_pool_.AllocateNode();

        vis_type = AstarNodeVisitedType::NOT_VISITED;
      } else {
        next_node_in_pool = node_set_[new_node.GetGlobalID()];

        vis_type = next_node_in_pool->GetVisitedType();
      }

      // check null
      if (next_node_in_pool == nullptr) {
        continue;
      }

      GetSingleShotNodeGCost(current_node, &new_node);

#if DEBUG_ONE_SHOT_PATH
      if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
        ILOG_INFO << "  vis type " << static_cast<int>(vis_type)
                  << " new node g " << new_node.GetGCost() << " pool node g "
                  << next_node_in_pool->GetGCost()
                  << " ,safe dist: " << new_node.GetDistToObs();
      }
#endif

      // find a new node
      if (vis_type == AstarNodeVisitedType::NOT_VISITED) {
        GetSingleShotNodeHeuCost(current_node, &new_node);

        h_cost_rs_path_num++;

        // next_node_in_pool->CopyNode(&new_node);
        *next_node_in_pool = new_node;
        next_node_in_pool->SetFCost();
        next_node_in_pool->SetVisitedType(AstarNodeVisitedType::IN_OPEN);

        next_node_in_pool->SetMultiMapIter(open_pq_.insert(
            std::make_pair(next_node_in_pool->GetFCost(), next_node_in_pool)));

        node_set_.emplace(next_node_in_pool->GetGlobalID(), next_node_in_pool);

#if DEBUG_ONE_SHOT_PATH
        if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
          ILOG_INFO << "new point";
          next_node_in_pool->DebugCost();
        }
#endif

      } else if (vis_type == AstarNodeVisitedType::IN_OPEN) {
        // in open set and need update
        if (new_node.GetGCost() < next_node_in_pool->GetGCost()) {
          GetSingleShotNodeHeuCost(current_node, &new_node);

          h_cost_rs_path_num++;

          open_pq_.erase(next_node_in_pool->GetMultiMapIter());

          *next_node_in_pool = new_node;
          next_node_in_pool->SetFCost();
          next_node_in_pool->SetVisitedType(AstarNodeVisitedType::IN_OPEN);

          // put node in open set again and record it.
          next_node_in_pool->SetMultiMapIter(open_pq_.insert(std::make_pair(
              next_node_in_pool->GetFCost(), next_node_in_pool)));

#if DEBUG_ONE_SHOT_PATH
          if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
            ILOG_INFO << "node has in open, open set size= " << open_pq_.size();
            next_node_in_pool->DebugCost();
          }
#endif
        }
      } else {
        // in close set and need update
        if (new_node.GetGCost() < next_node_in_pool->GetGCost() + 1e-1) {
          if (node_shrink_decider_.IsLoopBackNode(&new_node,
                                                  next_node_in_pool)) {
            continue;
          }

          if (!node_shrink_decider_.IsSameGridNodeContinuous(
                  &new_node, next_node_in_pool)) {
            continue;
          }

          GetSingleShotNodeHeuCost(current_node, &new_node);

          h_cost_rs_path_num++;

          *next_node_in_pool = new_node;
          next_node_in_pool->SetFCost();
          next_node_in_pool->SetVisitedType(AstarNodeVisitedType::IN_OPEN);

          // put node in open set again and record it.
          next_node_in_pool->SetMultiMapIter(open_pq_.insert(std::make_pair(
              next_node_in_pool->GetFCost(), next_node_in_pool)));

#if DEBUG_ONE_SHOT_PATH
          if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
            ILOG_INFO << "node has in close";
            next_node_in_pool->DebugCost();
          }
#endif
        }
      }

#if PLOT_CHILD_NODE
      if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
        child_node_debug_.emplace_back(
            DebugAstarSearchPoint(new_node.GetX(), new_node.GetY(), true));
      }
#endif
    }

    // search neighbor nodes  over
  }

  // todo, use all astar node, maybe no need use rs path.
  if (polynomial_node.IsNodeValid()) {
    BackwardPassByNode(result, &polynomial_node, nullptr, poly_path);
  } else if (rs_node_to_goal.IsNodeValid()) {
    BackwardPassByNode(result, &rs_node_to_goal, &rs_path_, poly_path);
  } else if (BestNodeIsNice(best_node)) {
    BackwardPassByNode(result, best_node, nullptr, poly_path);
  }
  result->fail_type = AstarFailType::TIME_OUT;
  if (open_pq_.size() <= 0) {
    result->fail_type = AstarFailType::NODE_POOL_IS_NULL;
  }

  ILOG_INFO << "explored node num is " << explored_node_num
            << " ,rs path size is: " << explored_rs_path_num
            << " ,h cost rs num " << h_cost_rs_path_num
            << " ,node pool size:" << node_pool_.PoolSize()
            << " ,open_pq_.size=" << open_pq_.size();

  ILOG_INFO << "heuristic time " << heuristic_time_ << " ,rs params time "
            << rs_time_ms_ << ",rs interpolate time:" << rs_interpolate_time_ms_
            << " ,collision time " << collision_check_time_ms_
            << ", gear switch num = " << result->gear_change_num
            << ", hybrid astar search time (ms)= " << astar_search_time;

  double astar_end_time = IflyTime::Now_ms();
  result->time_ms = astar_end_time - check_start_time;
  ILOG_INFO << "hybrid astar total time (ms) = " << result->time_ms;

  return;
}

bool HybridAStar::AstarSearch(const Pose2f& start, const Pose2f& end,
                              const MapBound& XYbounds,
                              HybridAStarResult* result) {
  double astar_start_time = IflyTime::Now_ms();

  result->Clear();

  // clear containers
  ResetNodePool();
  open_pq_.clear();
  node_set_.clear();

  // debug
  child_node_debug_.clear();
  queue_path_debug_.clear();
  delete_queue_path_debug_.clear();
  rs_path_h_cost_debug_.clear();
  rs_path_.Clear();

  collision_check_time_ms_ = 0.0;
  rs_time_ms_ = 0.0;
  rs_interpolate_time_ms_ = 0.0;

  ILOG_INFO << "hybrid astar begin";

  // load XYbounds
  grid_map_bound_ = XYbounds;
  // ILOG_INFO << "map bound, xmin " << grid_map_bound_.x_min << " , ymin "
  //           << grid_map_bound_.y_min << " ,xmax " << grid_map_bound_.x_max <<
  //           " , ymax "
  //           << grid_map_bound_.y_max;

  // check bound
  UpdateMaxGridIndex();

  // start
  start_node_ = node_pool_.AllocateNode();

  // load nodes and obstacles
  if (start_node_ == nullptr) {
    ILOG_ERROR << "start node nullptr";

    result->fail_type = AstarFailType::ALLOCATE_NODE_FAIL;
    return false;
  }

  start_node_->Set(NodePath(start), grid_map_bound_, config_, 0.0);
  if (!start_node_->IsNodeValid()) {
    ILOG_ERROR << "start_node invalid";

    result->fail_type = AstarFailType::OUT_OF_BOUND;
    return false;
  }

  // in searching, start node gear is forward or backward which will be ok.
  start_node_->SetGearType(AstarPathGear::NONE);
  start_node_->SetSteer(0.0);
  start_node_->SetIsStartNode(true);
  start_node_->SetPathType(AstarPathType::START_NODE);
  start_node_->SetDistToStart(0.0);
  start_node_->SetGearSwitchNum(0);
  start_node_->DebugString();
  // start node gcost is 0

  // check start
  double check_start_time = IflyTime::Now_ms();
  if (!collision_detect_->ValidityCheckByEDT(start_node_)) {
    // second check
    if (collision_detect_->IsFootPrintCollision(
            Transform2d(start_node_->GetPose()))) {
      ILOG_ERROR << "start_node in collision with obstacles "
                 << static_cast<int>(start_node_->GetConstCollisionType());

      // start_node_->DebugString();

      result->fail_type = AstarFailType::START_COLLISION;
      return false;
    }
  }
  double check_end_time = IflyTime::Now_ms();
  collision_check_time_ms_ += check_end_time - check_start_time;

  // allocate end
  astar_end_node_ = node_pool_.AllocateNode();

  if (astar_end_node_ == nullptr) {
    ILOG_ERROR << "end node nullptr";

    result->fail_type = AstarFailType::OUT_OF_BOUND;
    return false;
  }

  astar_end_node_->Set(NodePath(end), grid_map_bound_, config_, 0.0);
  astar_end_node_->SetGearType(AstarPathGear::NONE);
  astar_end_node_->SetPathType(AstarPathType::END_NODE);
  astar_end_node_->DebugString();

  if (!astar_end_node_->IsNodeValid()) {
    ILOG_ERROR << "end_node invalid";

    result->fail_type = AstarFailType::OUT_OF_BOUND;
    return false;
  }

  // check end
  check_start_time = IflyTime::Now_ms();
  if (!collision_detect_->ValidityCheckByEDT(astar_end_node_)) {
    ILOG_INFO << "end_node in collision with obstacles "
              << static_cast<int>(astar_end_node_->GetConstCollisionType());

    check_end_time = IflyTime::Now_ms();
    collision_check_time_ms_ += check_end_time - check_start_time;
  }

  // node shrink related
  node_shrink_decider_.Process(start, end, request_.direction_request,
                               request_.real_goal, grid_map_bound_);

  rs_expansion_decider_.Process(
      vehicle_param_.min_turn_radius, request_.slot_width, request_.slot_length,
      start, end, vehicle_param_.width, request_.space_type,
      request_.direction_request);

  SetSamplingTarget(end);

  PathComparator path_comparator;
  path_comparator.SetHeuristicPose(request_);

  // load open set, pq
  start_node_->SetMultiMapIter(
      open_pq_.insert(std::make_pair(0.0, start_node_)));

  node_set_.emplace(start_node_->GetGlobalID(), start_node_);

  // Hybrid A*
  size_t explored_node_num = 0;
  size_t explored_rs_path_num = 0;
  size_t h_cost_rs_path_num = 0;
  double astar_search_start_time = IflyTime::Now_ms();
  double astar_search_time;
  heuristic_time_ = 0.0;

  Node3d* current_node = nullptr;
  Node3d* next_node_in_pool = nullptr;
  Node3d new_node;
  Node3d rs_node_to_goal;
  rs_node_to_goal.Clear();

  AstarNodeVisitedType vis_type;
  bool is_safe = false;
  AstarPathGear full_path_gear_request = AstarPathGear::NONE;
  double current_time;

  int rs_path_success_num = 0;
  RSPath best_rs_path;
  Node3d best_rs_node;
  best_rs_node.ClearPath();
  best_rs_node.SetFCost(1000000.0);

  std::vector<AStarPathPoint> poly_path;

  while (!open_pq_.empty()) {
    // take out the lowest cost neighboring node

    current_node = open_pq_.begin()->second;
    open_pq_.erase(open_pq_.begin());
    if (current_node == nullptr) {
      ILOG_INFO << "pq is null node";
      continue;
    }

    current_node->SetVisitedType(AstarNodeVisitedType::IN_CLOSE);

#if DEBUG_CHILD_NODE
    if (explored_node_num < DEBUG_NODE_MAX_NUM) {
      ILOG_INFO << "============== main cycle =========== ";
      ILOG_INFO << "explored_node_num " << explored_node_num
                << " open set size for now " << open_pq_.size();
      current_node->DebugString();
    }
#endif

#if PLOT_SEARCH_SEQUENCE
    queue_path_debug_.emplace_back(
        Vec2f(current_node->GetX(), current_node->GetY()));
#endif

    current_time = IflyTime::Now_ms();
    // if bigger than 5.0 s，break
    astar_search_time = current_time - astar_search_start_time;
    if (astar_search_time > config_.max_search_time_ms) {
      // ILOG_INFO << "time out " << astar_search_time;
      break;
    }

    // check if an analystic curve could be connected from current
    // configuration to the end configuration without collision. if so,
    // search ends.
    if (AnalyticExpansionByRS(current_node, full_path_gear_request,
                              &rs_node_to_goal)) {
      rs_path_success_num++;

      if (path_comparator.Compare(&request_, &best_rs_node, &rs_node_to_goal)) {
        best_rs_node = rs_node_to_goal;
        best_rs_path = rs_path_;
      }

#if PLOT_CHILD_NODE
      // if node is gear switch point, plot it also.
      if (rs_node_to_goal.GearSwitchNode() != nullptr) {
        child_node_debug_.emplace_back(DebugAstarSearchPoint(
            rs_node_to_goal.GearSwitchNode()->GetX(),
            rs_node_to_goal.GearSwitchNode()->GetY(), true, true));
      }
#endif

      // in try searching, no need optimal path.
      if (request_.path_generate_method ==
              AstarPathGenerateType::TRY_SEARCHING &&
          astar_search_time > config_.search_time_ms_scenario_try) {
        break;
      }

      // total gear switch number is 0, break;
      if (rs_node_to_goal.GetGearSwitchNum() < 1) {
        break;
      }

      /** If got some solutions, and search time bigger than 300.0, searching
       *  can break. If not satisfy break condition, continue to do searching.
       */
      if (rs_path_success_num > 5 && astar_search_time > 300.0) {
        break;
      }

      // If search time bigger than 1000 ms, break
      if (rs_path_success_num > 0 && astar_search_time > 1000.0) {
        break;
      }
    }

    explored_rs_path_num++;

    for (size_t i = 0; i < next_node_num_; ++i) {
      NextNodeGenerator(&new_node, current_node, i, full_path_gear_request);
      explored_node_num++;

#if DEBUG_CHILD_NODE
      if (explored_node_num < DEBUG_NODE_MAX_NUM) {
        ILOG_INFO << "~~~~~~~~~~ child node cycle ~~~~~~~~~";
        ILOG_INFO << "open set size " << open_pq_.size()
                  << ", gear change num:" << current_node->GetGearSwitchNum();
        new_node.DebugString();
      }
#endif

      // boundary check failure handle
      if (!new_node.IsNodeValid()) {
        continue;
      }

      if (node_shrink_decider_.IsShrinkByHeadOutDirection(request_,
                                                          &new_node)) {
        continue;
      }

      if (node_shrink_decider_.IsShrinkByStartNode(start_node_->GetGlobalID(),
                                                   &new_node)) {
        continue;
      }

      if (node_shrink_decider_.IsShrinkByParent(current_node, &new_node)) {
        continue;
      }

      if (node_shrink_decider_.IsShrinkByGearSwitchNumber(&new_node)) {
        continue;
      }

      // collision check
#if LOG_TIME_PROFILE
      check_start_time = IflyTime::Now_ms();
#endif

      is_safe = collision_detect_->ValidityCheckByEDT(&new_node);

#if LOG_TIME_PROFILE
      check_end_time = IflyTime::Now_ms();
      collision_check_time_ms_ += check_end_time - check_start_time;
#endif

      if (!is_safe) {
#if PLOT_CHILD_NODE
        // if node is unsafe, plot it also.
        int gear_switch_num = new_node.GetGearSwitchNum();
        if (explored_node_num < DEBUG_NODE_MAX_NUM &&
            gear_switch_num <= DEBUG_NODE_GEAR_SWITCH_NUMBER) {
          child_node_debug_.emplace_back(
              DebugAstarSearchPoint(new_node.GetX(), new_node.GetY(), false));
        }
#endif
        continue;
      }

      // allocate new node
      if (node_set_.find(new_node.GetGlobalID()) == node_set_.end()) {
        next_node_in_pool = node_pool_.AllocateNode();

        vis_type = AstarNodeVisitedType::NOT_VISITED;
      } else {
        next_node_in_pool = node_set_[new_node.GetGlobalID()];

        vis_type = next_node_in_pool->GetVisitedType();
      }

      // check null
      if (next_node_in_pool == nullptr) {
        // ILOG_INFO << " next node is null";
        continue;
      }

      CalculateNodeGCost(current_node, &new_node);

#if DEBUG_CHILD_NODE
      if (explored_node_num < DEBUG_NODE_MAX_NUM) {
        ILOG_INFO << "  vis type " << static_cast<int>(vis_type)
                  << " new node g " << new_node.GetGCost() << " pool node g "
                  << next_node_in_pool->GetGCost()
                  << " ,safe dist: " << new_node.GetDistToObs();
      }
#endif

      // find a new node
      if (vis_type == AstarNodeVisitedType::NOT_VISITED) {
        CalculateNodeHeuristicCost(current_node, &new_node);

        h_cost_rs_path_num++;

        *next_node_in_pool = new_node;
        next_node_in_pool->SetFCost();
        next_node_in_pool->SetVisitedType(AstarNodeVisitedType::IN_OPEN);

        next_node_in_pool->SetMultiMapIter(open_pq_.insert(
            std::make_pair(next_node_in_pool->GetFCost(), next_node_in_pool)));

        node_set_.emplace(next_node_in_pool->GetGlobalID(), next_node_in_pool);

#if DEBUG_CHILD_NODE
        if (explored_node_num < DEBUG_NODE_MAX_NUM) {
          ILOG_INFO << "new point";
          next_node_in_pool->DebugCost();
        }
#endif

      } else if (vis_type == AstarNodeVisitedType::IN_OPEN) {
        // in open set and need update
        if (new_node.GetGCost() < next_node_in_pool->GetGCost()) {
          CalculateNodeHeuristicCost(current_node, &new_node);

          h_cost_rs_path_num++;

          open_pq_.erase(next_node_in_pool->GetMultiMapIter());

          *next_node_in_pool = new_node;
          next_node_in_pool->SetFCost();
          next_node_in_pool->SetVisitedType(AstarNodeVisitedType::IN_OPEN);

          // put node in open set again and record it.
          next_node_in_pool->SetMultiMapIter(open_pq_.insert(std::make_pair(
              next_node_in_pool->GetFCost(), next_node_in_pool)));

#if DEBUG_CHILD_NODE
          if (explored_node_num < DEBUG_NODE_MAX_NUM) {
            ILOG_INFO << "node has in open";
            next_node_in_pool->DebugCost();
          }
#endif
        }
      } else {
        // in close set and need update
        if (new_node.GetGCost() < next_node_in_pool->GetGCost() + 1e-1) {
          if (node_shrink_decider_.IsLoopBackNode(&new_node,
                                                  next_node_in_pool)) {
            continue;
          }

          if (!node_shrink_decider_.IsSameGridNodeContinuous(
                  &new_node, next_node_in_pool)) {
            continue;
          }

          CalculateNodeHeuristicCost(current_node, &new_node);

          h_cost_rs_path_num++;

          *next_node_in_pool = new_node;
          next_node_in_pool->SetFCost();
          next_node_in_pool->SetVisitedType(AstarNodeVisitedType::IN_OPEN);

          // put node in open set again and record it.
          next_node_in_pool->SetMultiMapIter(open_pq_.insert(std::make_pair(
              next_node_in_pool->GetFCost(), next_node_in_pool)));

#if DEBUG_CHILD_NODE
          if (explored_node_num < DEBUG_NODE_MAX_NUM) {
            ILOG_INFO << "node has in close";
            next_node_in_pool->DebugCost();
          }
#endif
        }
      }

#if PLOT_CHILD_NODE
      int gear_switch_num = new_node.GetGearSwitchNum();
      if (explored_node_num < DEBUG_NODE_MAX_NUM &&
          gear_switch_num <= DEBUG_NODE_GEAR_SWITCH_NUMBER) {
        child_node_debug_.emplace_back(
            DebugAstarSearchPoint(new_node.GetX(), new_node.GetY(), true));
      }
#endif
    }

    // search neighbor nodes  over
  }

  double search_end_time = IflyTime::Now_ms();
  ILOG_INFO << "hybrid astar search time (ms) is "
            << search_end_time - astar_search_start_time;

  // todo, use all astar node, maybe no need use rs path.
  if (best_rs_node.IsNodeValid()) {
    BackwardPassByNode(result, &best_rs_node, &best_rs_path, poly_path);
  }

  result->fail_type = AstarFailType::TIME_OUT;
  if (open_pq_.size() <= 0) {
    result->fail_type = AstarFailType::NODE_POOL_IS_NULL;
  }

  ILOG_INFO << "explored node num is " << explored_node_num
            << " , rs path size is : " << explored_rs_path_num
            << " , h cost rs num: " << h_cost_rs_path_num
            << " , node pool size: " << node_pool_.PoolSize()
            << " , open_pq.size: " << open_pq_.size()
            << " , RS success number = " << rs_path_success_num;

  ILOG_INFO << "heuristic time " << heuristic_time_ << " ,rs params time "
            << rs_time_ms_ << ",rs interpolate time:" << rs_interpolate_time_ms_
            << " ,collision time " << collision_check_time_ms_
            << ", gear switch num = " << result->gear_change_num;

  double astar_end_time = IflyTime::Now_ms();
  result->time_ms = astar_end_time - astar_start_time;
  ILOG_INFO << "hybrid astar total time (ms) = " << result->time_ms;

  // ILOG_INFO << "hybrid astar finished";
  // ILOG_INFO << "child node size" << child_node_debug_.size();

#if DEBUG_EDT
  DebugEDTCheck(result);
#endif

  return true;
}

void HybridAStar::Init() {
  next_node_num_ = config_.next_node_num;
  max_steer_angle_ =
      vehicle_param_.max_steer_angle / vehicle_param_.steer_ratio;

  // min_radius_ = vehicle_param_.wheel_base / std::tan(max_steer_angle_);
  min_radius_ = vehicle_param_.min_turn_radius;
  inv_radius_ = 1.0 / min_radius_;

  // front sampling number is 5, back sampling number is 5.
  next_node_angles_.angles[0] = max_steer_angle_;
  next_node_angles_.radius[0] = min_radius_;

  next_node_angles_.angles[1] = -max_steer_angle_;
  next_node_angles_.radius[1] = -next_node_angles_.radius[0];

  next_node_angles_.angles[2] = 0.0;
  next_node_angles_.radius[2] = 100000.0;

  next_node_angles_.angles[3] = max_steer_angle_ * 0.5;
  next_node_angles_.radius[3] =
      vehicle_param_.wheel_base / std::tan(max_steer_angle_ * 0.5);

  next_node_angles_.angles[4] = -max_steer_angle_ * 0.5;
  next_node_angles_.radius[4] = -next_node_angles_.radius[3];
  next_node_angles_.size = 5;

  node_path_dist_resolution_ = config_.node_path_dist_resolution;
  xy_grid_resolution_ = config_.xy_grid_resolution;
  phi_grid_resolution_ = config_.phi_grid_resolution;

  car_half_width_ = vehicle_param_.width / 2 + config_.heuristic_safe_dist;
  kinetics_model_step_ = 0.05;

  NodePoolInit();

  rs_path_h_cost_debug_.clear();
  rs_path_.Clear();

  ILOG_INFO << "astar init finish";

  return;
}

void HybridAStar::GetRSPathForDebug(std::vector<float>& x,
                                    std::vector<float>& y,
                                    std::vector<float>& phi) {
  x.clear();
  y.clear();
  phi.clear();

  for (int i = 0; i < std::min(MAX_RS_PATH_NUM, rs_path_.size); i++) {
    RSPathSegment* segment = &rs_path_.paths[i];

    for (int j = 0; j < std::min(RS_PATH_MAX_POINT, segment->size); j++) {
      x.emplace_back(segment->points[j].x);
      y.emplace_back(segment->points[j].y);
      phi.emplace_back(segment->points[j].theta);
    }
  }

  ILOG_INFO << "rs size " << x.size();

  return;
}

void HybridAStar::DebugLineSegment(
    const ad_common::math::LineSegment2d& line) const {
  ILOG_INFO << "start " << line.start().x() << " " << line.start().y();
  ILOG_INFO << "end " << line.end().x() << " " << line.end().y();

  return;
}

void HybridAStar::GetNodeListMessage(planning::common::AstarNodeList* list) {
  planning::common::TrajectoryPoint point;

  for (auto i = node_set_.begin(); i != node_set_.end(); i++) {
    if (i->second->GetStepSize() < 2 || i->second->IsRsPath()) {
      continue;
    }

    const NodePath& path = i->second->GetNodePath();

    planning::common::AstarNode* tmp_node = list->add_nodes();

    for (size_t m = 0; m < path.point_size; m++) {
      point.set_x(path.points[m].x);
      point.set_y(path.points[m].y);
      point.set_heading_angle(path.points[m].theta);

      planning::common::TrajectoryPoint* tmp_point = tmp_node->add_path_point();

      tmp_point->CopyFrom(point);
    }
  }

  return;
}

void HybridAStar::GetNodeListMessage(
    std::vector<std::vector<Eigen::Vector2d>>& list) {
  for (auto i = node_set_.begin(); i != node_set_.end(); i++) {
    if (i->second->GetStepSize() < 2 || i->second->IsRsPath()) {
      continue;
    }

    const NodePath& path = i->second->GetNodePath();

    std::vector<Eigen::Vector2d> node;
    for (size_t m = 0; m < path.point_size; m++) {
      node.emplace_back(Eigen::Vector2d(path.points[m].x, path.points[m].y));
    }

    list.emplace_back(node);
  }

  return;
}

void HybridAStar::ResetNodePool() {
  node_pool_.Clear();

  return;
}

void HybridAStar::NodePoolInit() {
  node_pool_.Init();

  return;
}

bool HybridAStar::NodeInSearchBound(Node3d* node) {
  if (!NodeInSearchBound(node->GetIndex())) {
    return false;
  }

  return true;
}

bool HybridAStar::NodeInSearchBound(const NodeGridIndex& id) {
  if (id.x < 0 || id.x >= max_grid_map_index_.x || id.y < 0 ||
      id.y >= max_grid_map_index_.y || id.phi < 0 ||
      id.phi >= max_grid_map_index_.phi) {
    return false;
  }

  return true;
}

const std::vector<DebugAstarSearchPoint>& HybridAStar::GetChildNodeForDebug() {
  // ILOG_INFO << "child node size" << child_node_debug_.size();
  return child_node_debug_;
}

const std::vector<Vec2f>& HybridAStar::GetQueuePathForDebug() {
  return queue_path_debug_;
}

const std::vector<Vec2f>& HybridAStar::GetDelQueuePathForDebug() {
  return delete_queue_path_debug_;
}

const std::vector<RSPath>& HybridAStar::GetRSPathHeuristic() {
  return rs_path_h_cost_debug_;
}

void HybridAStar::KineticsModel(const Pose2f* old_pose, const float radius,
                                Pose2f* pose, const bool is_forward) {
  float dist = is_forward ? kinetics_model_step_ : -kinetics_model_step_;
  pose->x = old_pose->x + dist * std::cos(old_pose->theta);
  pose->y = old_pose->y + dist * std::sin(old_pose->theta);

  float new_phi;
  new_phi = old_pose->theta + dist / radius;
  pose->theta = ad_common::math::NormalizeAngle(new_phi);

  return;
}

void HybridAStar::UpdatePoseByPathPointInterval(const Pose2f* old_pose,
                                                const float radius,
                                                const float interval,
                                                Pose2f* pose,
                                                const bool is_forward) {
  int number = std::ceil(interval / kinetics_model_step_);
  Pose2f start = *old_pose;
  Pose2f end;

  for (int i = 0; i < number; i++) {
    KineticsModel(&start, radius, &end, is_forward);

    start = end;
  }

  *pose = end;

  return;
}

void HybridAStar::UpdatePoseBySamplingNumber(const Pose2f* old_pose,
                                             const float radius,
                                             const int number, Pose2f* pose,
                                             const bool is_forward) {
  Pose2f start = *old_pose;

  for (int i = 0; i < number; i++) {
    KineticsModel(&start, radius, pose, is_forward);

    start = *pose;
  }

  return;
}

float HybridAStar::CalcSafeDistCost(Node3d* node) {
  // weight: 15
  // [0-0.15], cost: 1000;
  // [0.15-0.5],cost: (1/dist -2) * weight;
  // [0.5-1000], cost:0;

  float dist = node->GetDistToObs();
  float weight = 10.0f;

  if (dist > 0.4f) {
    return 0.0;
  } else if (dist > 0.15f) {
    return (1.0f / dist - 2.5f) * weight;
  }

  return 100.0;
}

const ParkReferenceLine* HybridAStar::GetConstRefLine() const {
  return ref_line_;
}

int HybridAStar::InterpolateByArcOffset(Pose2f* pose,
                                        const VehicleCircle* veh_circle,
                                        const Pose2f* start_pose,
                                        const float arc,
                                        const float inverse_radius) {
  float delta_theta, theta;

  delta_theta = arc * inverse_radius;

  // left turn
  if (veh_circle->radius > 0.0) {
    if (veh_circle->gear == AstarPathGear::REVERSE) {
      delta_theta = -delta_theta;
    }
  } else {
    // right turn, gear is d
    if (veh_circle->gear == AstarPathGear::DRIVE) {
      delta_theta = -delta_theta;
    }
  }

  // update next point theta
  theta = start_pose->theta + delta_theta;

  float radius = veh_circle->radius;

  pose->x = veh_circle->center.x + radius * std::sin(theta);
  pose->y = veh_circle->center.y - radius * std::cos(theta);
  pose->theta = IflyUnifyTheta(theta, M_PIf32);

  return 1;
}

// radius: if left turn, radius is positive
int HybridAStar::GetVehCircleByPose(VehicleCircle* veh_circle,
                                    const Pose2f* pose, const float radius,
                                    const AstarPathGear gear) {
  veh_circle->radius = radius;
  veh_circle->gear = gear;

  veh_circle->center.x = pose->x - radius * std::sin(pose->theta);
  veh_circle->center.y = pose->y + radius * std::cos(pose->theta);

  return 1;
}

int HybridAStar::GetStraightLinePoint(Pose2f* goal_state,
                                      const Pose2f* start_state,
                                      const float dist_to_start,
                                      const Pose2f* unit_vector) {
  goal_state->x = start_state->x + dist_to_start * unit_vector->x;
  goal_state->y = start_state->y + dist_to_start * unit_vector->y;

  goal_state->theta = start_state->theta;

  return 1;
}

void HybridAStar::UpdateCarBoxBySafeBuffer(const float lat_buffer_outside,
                                           const float lat_buffer_inside,
                                           const float lon_buffer) {
  collision_detect_->UpdateFootPrintBySafeBuffer(lat_buffer_outside,
                                                 lat_buffer_inside, lon_buffer,
                                                 vehicle_param_, config_);

  return;
}

void HybridAStar::Clear() {
  fallback_path_.Clear();
  return;
}

void HybridAStar::CopyFallbackPath(HybridAStarResult* path) {
  *path = fallback_path_;
  return;
}

void HybridAStar::SetRequest(const AstarRequest& request) {
  request_ = request;
  if (request.space_type == ParkSpaceType::PARALLEL) {
    config_.node_step = config_.parallel_slot_node_step;
  } else {
    config_.node_step = config_.perpendicular_slot_node_step;
    if (request.direction_request == ParkingVehDirection::HEAD_OUT_TO_LEFT ||
        request.direction_request == ParkingVehDirection::HEAD_OUT_TO_RIGHT ||
        request.direction_request == ParkingVehDirection::HEAD_OUT_TO_MIDDLE ||
        request.direction_request == ParkingVehDirection::TAIL_OUT_TO_LEFT ||
        request.direction_request == ParkingVehDirection::TAIL_OUT_TO_RIGHT ||
        request.direction_request == ParkingVehDirection::TAIL_OUT_TO_MIDDLE) {
      config_.node_step = config_.perpendicular_slot_head_out_node_step;
    }
  }

  SetSamplingTarget(request.goal);

  return;
}

void HybridAStar::ReversePathBySwapStartGoal(HybridAStarResult* result) {
  if (!request_.swap_start_goal) {
    return;
  }

  if (result->x.size() < 2) {
    return;
  }

  std::reverse(result->x.begin(), result->x.end());
  std::reverse(result->y.begin(), result->y.end());
  std::reverse(result->phi.begin(), result->phi.end());
  std::reverse(result->gear.begin(), result->gear.end());
  std::reverse(result->type.begin(), result->type.end());
  std::reverse(result->kappa.begin(), result->kappa.end());

  for (size_t i = 0; i < result->gear.size(); i++) {
    if (result->gear[i] == AstarPathGear::DRIVE) {
      result->gear[i] = AstarPathGear::REVERSE;
    } else {
      result->gear[i] = AstarPathGear::DRIVE;
    }
  }

  return;
}

const bool HybridAStar::BestNodeIsNice(const Node3d* node) {
  bool node_is_good = false;
  if (node == nullptr) {
    return false;
  }

  // ILOG_INFO <<"check best node";
  // node->DebugPoseString();

  if (std::fabs((node->GetPose().y - request_.real_goal.y)) > 0.03) {
    return false;
  }

  if (std::fabs(ad_common::math::NormalizeAngle(
          node->GetPose().theta - request_.real_goal.theta)) > 0.02) {
    return false;
  }

  return true;
}

void HybridAStar::CopyNodePath(const Node3d* node,
                               HybridAStarResult* astar_path) {
  const NodePath& path = node->GetNodePath();
  AstarPathType path_type = node->GetPathType();
  AstarPathGear cur_gear_type = node->GetGearType();

  size_t point_size;

  // todo
  if (node->GetConstNextNode() == nullptr) {
    point_size = path.point_size;
  }
  // gear change
  else if (node->IsPathGearChange(node->GetConstNextNode()->GetGearType())) {
    point_size = path.point_size;
  }
  // same gear
  else {
    // delete same point
    point_size = path.point_size - 1;
  }

  float kappa = std::tan(node->GetSteer()) / vehicle_param_.wheel_base;

  for (size_t k = 0; k < point_size; k++) {
    astar_path->x.emplace_back(path.points[k].x);
    astar_path->y.emplace_back(path.points[k].y);
    astar_path->phi.emplace_back(path.points[k].theta);
    astar_path->type.emplace_back(path_type);
    astar_path->gear.emplace_back(cur_gear_type);
    astar_path->kappa.emplace_back(kappa);
  }

  return;
}

FootPrintCircleModel* HybridAStar::GetSlotOutsideCircleFootPrint() {
  if (collision_detect_ == nullptr) {
    return nullptr;
  }

  return collision_detect_->GetSlotOutsideCircleFootPrint();
}

void HybridAStar::UpdatePathS(HybridAStarResult* path) {
  if (path->x.empty()) {
    return;
  }

  // get path lengh
  float accumulated_s = 0.0;
  path->accumulated_s.clear();
  float last_x = path->x.front();
  float last_y = path->y.front();
  float x_diff;
  float y_diff;
  size_t path_points_size = path->x.size();

  for (size_t i = 0; i < path_points_size; ++i) {
    x_diff = path->x[i] - last_x;
    y_diff = path->y[i] - last_y;
    accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
    path->accumulated_s.emplace_back(accumulated_s);
    last_x = path->x[i];
    last_y = path->y[i];
  }
  return;
}

const bool HybridAStar::IsNeedGearDriveSearch(const Pose2f& start) {
  if (request_.direction_request != ParkingVehDirection::HEAD_IN) {
    return false;
  }

  float heading = IflyUnifyTheta(start.GetPhi(), M_PIf32);
  if (std::fabs(heading) < (M_PI_2 - 0.001)) {
    ILOG_INFO << "start.GetPhi() =" << heading * 57.4;
    return false;
  }

  if (start.GetX() < 4.0) {
    ILOG_INFO << "start.GetX() =" << start.GetX();
    return false;
  }

  if (start.GetY() < -9.0 || start.GetY() > 9.0) {
    ILOG_INFO << "start.GetY() =" << start.GetY();
    return false;
  }

  return true;
}

const bool HybridAStar::IsNeedGearReverseSearch(const Pose2f& start) {
  if (request_.direction_request != ParkingVehDirection::TAIL_IN) {
    return false;
  }

  if (start.GetX() < 1.0) {
    ILOG_INFO << "start.GetX() =" << start.GetX();
    return false;
  }

  if (start.GetY() < -8.0 || start.GetY() > 8.0) {
    ILOG_INFO << "start.GetY() =" << start.GetY();
    return false;
  }

  float heading = IflyUnifyTheta(start.GetPhi(), M_PIf32);
  if (std::fabs(heading) > (M_PI_2 + 0.001)) {
    ILOG_INFO << "start.GetPhi() =" << heading * 57.4;
    return false;
  }

  return true;
}

void HybridAStar::DebugNodeList(const std::vector<Node3d*>& node_list) {
  ILOG_INFO << "path node num " << node_list.size();
  for (size_t i = 0; i < node_list.size(); i++) {
    ILOG_INFO << "node id " << i << " node steer "
              << node_list[i]->GetSteer() * 57.3 << ", gear "
              << PathGearDebugString(node_list[i]->GetGearType())
              << ", node type "
              << GetNodeCurveDebugString(node_list[i]->GetPathType())
              << ", length: "
              << node_path_dist_resolution_ * node_list[i]->GetStepSize();
  }

  return;
}

void HybridAStar::SetSamplingTarget(const Pose2f& pose) {
  polynomial_sampling_->SetSearchGoal(pose);
  spiral_sampling_->SetSearchGoal(pose);
  rs_sampling_->SetSearchGoal(pose);

  return;
}

void HybridAStar::UpdateMaxGridIndex() {
  max_grid_map_index_.x =
      std::ceil((grid_map_bound_.x_max - grid_map_bound_.x_min) *
                config_.xy_grid_resolution_inv);
  max_grid_map_index_.y =
      std::ceil((grid_map_bound_.y_max - grid_map_bound_.y_min) *
                config_.xy_grid_resolution_inv);
  max_grid_map_index_.phi =
      std::ceil((M_PIf32 * 2.0f) * config_.phi_grid_resolution_inv);

  return;
}

}  // namespace planning
