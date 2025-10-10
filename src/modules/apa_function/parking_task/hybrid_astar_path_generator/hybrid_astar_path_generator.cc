#include "hybrid_astar_path_generator.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "aabb2d.h"
#include "apa_context.h"
#include "apa_param_config.h"
#include "common_math.h"
#include "curve_node.h"
#include "geometry_math.h"
#include "grid_search.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_context.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "node3d.h"
#include "pose2d.h"
#include "reeds_shepp.h"

namespace planning {
namespace apa_planner {

const float kMaxTurnRadius = 100000.0;

#define LOG_TIME_PROFILE (1)
#define PLOT_DELETE_NODE (0)
#define DEBUG_FINAL_PATH (0)

const bool HybridAStarPathGenerator::Init() {
  config_.InitConfig();

  InitNodePool();

  GenerateCarMotionVec();

  grid_search_.UpdateConfig(config_);

  if (col_det_interface_ptr_ != nullptr) {
    node_delete_decider_.SetCollisionDetectorIntefacePtr(
        col_det_interface_ptr_);
  }

  ILOG_INFO << "astar init finish";

  return true;
}

void HybridAStarPathGenerator::SetCollisionDetectorIntefacePtr(
    const std::shared_ptr<CollisionDetectorInterface>& col_det_interface_ptr) {
  col_det_interface_ptr_ = col_det_interface_ptr;
  grid_search_.SetCollisionDetectorIntefacePtr(col_det_interface_ptr);
  node_delete_decider_.SetCollisionDetectorIntefacePtr(col_det_interface_ptr_);
}

void HybridAStarPathGenerator::UpdateConfig(
    const PlannerOpenSpaceConfig& config) {
  config_ = config;
  grid_search_.UpdateConfig(config_);
  GenerateCarMotionVec();
}

void HybridAStarPathGenerator::GenerateCarMotionVec() {
  car_motion_vec.clear();
  car_motion_vec.reserve(config_.next_node_num);

  min_radius_ = std::max(apa_param.GetParam().min_turn_radius, 3.0);
  max_front_wheel_angle_ =
      std::atan2(apa_param.GetParam().wheel_base, min_radius_);

  max_kappa_ = 1.0f / min_radius_;
  max_kappa_change_ = 2.0f * max_kappa_;

  int radius_count = (config_.next_node_num / 2) / 2;
  std::vector<float> radius_vec;
  for (int i = 1; i <= radius_count; ++i) {
    radius_vec.emplace_back(i * min_radius_);
  }
  radius_vec.emplace_back(kMaxTurnRadius);
  for (int i = radius_count; i >= 1; --i) {
    radius_vec.emplace_back(-i * min_radius_);
  }
  for (const float& radius : radius_vec) {
    car_motion_vec.emplace_back(
        GenerateCarMotion(radius, AstarPathGear::DRIVE));
    car_motion_vec.emplace_back(
        GenerateCarMotion(radius, AstarPathGear::REVERSE));
  }
  for (const CarMotion& car_motion : car_motion_vec) {
    ILOG_INFO << "front_wheel_angle = " << car_motion.front_wheel_angle
              << "rad, radius = " << car_motion.radius
              << "m, kappa = " << car_motion.kappa;
  }
}

const CarMotion HybridAStarPathGenerator::GenerateCarMotion(
    const float radius, const AstarPathGear gear) {
  CarMotion car_motion;
  car_motion.radius = radius;
  car_motion.gear = gear;
  car_motion.traj_length = config_.node_step;
  // 前轮转角和转弯半径并不是真的对应，只是象征性的表示一下
  if (std::fabs(radius) > 1000.0f) {
    car_motion.front_wheel_angle = 0.0f;
    car_motion.kappa = 0.0f;
    car_motion.type = AstarPathSegType::LINE;
  } else {
    car_motion.front_wheel_angle =
        std::atan(apa_param.GetParam().wheel_base / radius);
    car_motion.kappa = 1.0f / radius;
    car_motion.type = AstarPathSegType::ARC;
  }
  return car_motion;
}

const NodePath HybridAStarPathGenerator::GetNodePathByCarMotion(
    const Pose2f& pose, const CarMotion& car_motion) {
  NodePath path;

  const int path_point_num = int(std::ceil((car_motion.traj_length - 1e-5f) /
                                           config_.node_path_dist_resolution)) +
                             1;

  // ILOG_INFO << "path_point_num = " << path_point_num;

  path.point_size = 1;
  path.points[0] = pose;

  const common_math::Pos<float> dir = common_math::CalDirFromTheta(pose.theta);
  Eigen::Vector2f move_vec;
  Eigen::Matrix2f rot_mat;
  Eigen::Vector2f center;
  float rot_angle = 0.0f;

  if (car_motion.type == AstarPathSegType::LINE) {
    move_vec = dir * config_.node_path_dist_resolution;
    if (car_motion.gear == AstarPathGear::REVERSE) {
      move_vec *= -1.0f;
    }
  } else {
    rot_angle = 90.0f * common_math::kDeg2RadF;
    if (car_motion.radius < 0.0f) {
      rot_angle *= -1.0f;
    }

    rot_mat = common_math::CalRotMatFromTheta(rot_angle);
    center = Eigen::Vector2f(pose.x, pose.y) +
             rot_mat * dir * std::fabs(car_motion.radius);

    rot_angle =
        config_.node_path_dist_resolution / std::fabs(car_motion.radius);

    if ((car_motion.gear == AstarPathGear::DRIVE && car_motion.radius < 0.0f) ||
        (car_motion.gear == AstarPathGear::REVERSE &&
         car_motion.radius > 0.0f)) {
      rot_angle *= -1.0f;
    }
    rot_mat = common_math::CalRotMatFromTheta(rot_angle);
  }

  common_math::PathPt<float> new_pose(Eigen::Vector2f(pose.x, pose.y),
                                      pose.theta);

  for (int i = 1; i < path_point_num; ++i) {
    if (car_motion.type == AstarPathSegType::LINE) {
      new_pose.pos += move_vec;
    } else {
      new_pose.pos = rot_mat * (new_pose.pos - center) + center;
      new_pose.theta = common_math::UnifyAngleSum(new_pose.theta, rot_angle);
    }
    path.points[path.point_size++] =
        Pose2f(new_pose.pos.x(), new_pose.pos.y(), new_pose.theta);
    path.path_dist += config_.node_path_dist_resolution;

    if (path.point_size >= NODE_PATH_MAX_POINT) {
      ILOG_INFO << "size too much";
      break;
    }
  }

  // DebugNodePath(path, car_motion.gear);

  return path;
}

void HybridAStarPathGenerator::CalcNodeGCost(Node3d* current_node,
                                             Node3d* next_node) {
  return;
}

void HybridAStarPathGenerator::CalcNodeHCost(
    Node3d* current_node, Node3d* next_node,
    const AnalyticExpansionRequest& analytic_expansion_request) {
  return;
}

const bool HybridAStarPathGenerator::AnalyticExpansion(
    const AnalyticExpansionRequest& request) {
  if (request.type == AnalyticExpansionType::REEDS_SHEEP) {
    return AnalyticExpansionByRS(request.current_node,
                                 request.curve_node_to_goal, request.rs_radius,
                                 request.need_rs_dense_point,
                                 request.need_anchor_point, request.rs_request);
  }

  if (request.type == AnalyticExpansionType::LINK_POSE_LINE) {
    return AnalyticExpansionByLPL(
        request.current_node, request.curve_node_to_goal, *request.lpl_input);
  }

  return false;
}

const bool HybridAStarPathGenerator::AnalyticExpansionByRS(
    Node3d* current_node, CurveNode* curve_node_to_goal, const float rs_radius,
    const bool need_rs_dense_point, const bool need_anchor_point,
    const RSPathRequestType rs_request) {
  curve_node_to_goal->Clear();
  if (std::fabs(current_node->GetPhi()) > M_PI_2f32) {
    return false;
  }

  if (!CalcRSPathToGoal(current_node, need_rs_dense_point, need_anchor_point,
                        rs_request, rs_radius)) {
    ILOG_INFO << "rs path connect fail";
    return false;
  }

  // const double rs_start_time = IflyTime::Now_ms();

  if (request_.search_mode == SearchMode::FORMAL) {
    // 是否需要计算rs每段挡位路径的长度
    float rs_seg_length[6];
    int rs_seg_num = 0;
    rs_seg_length[rs_seg_num] = rs_path_.paths[0].length;
    for (int i = 1; i < rs_path_.size; i++) {
      if (rs_path_.paths[i].gear == rs_path_.paths[i - 1].gear) {
        rs_seg_length[rs_seg_num] += rs_path_.paths[i].length;
      } else {
        rs_seg_num++;
        rs_seg_length[rs_seg_num] = rs_path_.paths[i].length;
      }
    }

    // request check
    if (current_node->GetPathType() == AstarPathType::START_NODE) {
      // ILOG_INFO << "the cur node is start node";
      if (IsGearDifferent(rs_path_.GetFirstGear(),
                          request_.inital_action_request.ref_gear)) {
        // ILOG_INFO << "rs first gear is not expectation";
        return false;
      }
      if (rs_path_.gear_change_number > request_.max_gear_shift_number) {
        // ILOG_INFO << "rs gear shift number is not expectation";
        return false;
      }
      if (rs_path_.GetFirstGearLength() <
          request_.inital_action_request.ref_length) {
        // ILOG_INFO << "rs first gear length is not expectation";
        return false;
      }
      for (int i = 1; i < rs_seg_num; i++) {
        if (rs_seg_length[i] < request_.every_gear_length) {
          // ILOG_INFO << "rs seg every gear length is not expectation";
          return false;
        }
      }
    }

    else if (current_node->GetGearSwitchNum() == 0) {
      // ILOG_INFO << "the cur node is not start node, but from start node and
      // cur node donot shift gear";
      if (IsGearDifferent(current_node->GetGearType(),
                          rs_path_.GetFirstGear())) {
        if (rs_path_.gear_change_number + 1 > request_.max_gear_shift_number) {
          // ILOG_INFO << "rs gear shift number is not expectation";
          return false;
        }
        if (current_node->GetDistToStart() <
            request_.inital_action_request.ref_length) {
          // ILOG_INFO << "rs first gear length is not expectation";
          return false;
        }
        for (int i = 0; i < rs_seg_num; i++) {
          if (rs_seg_length[i] < request_.every_gear_length) {
            // ILOG_INFO << "rs seg every gear length is not expectation";
            return false;
          }
        }
      } else {
        if (current_node->GetDistToStart() + rs_seg_length[0] <
            request_.inital_action_request.ref_length) {
          // ILOG_INFO << "rs first gear length is not expectation";
          return false;
        }
        for (int i = 1; i < rs_seg_num; i++) {
          if (rs_seg_length[i] < request_.every_gear_length) {
            // ILOG_INFO << "rs seg every gear length is not expectation";
            return false;
          }
        }
      }
    }

    else {
      // ILOG_INFO << "the cur node is not start node, and from start node and
      // cur node shift gear";
      int shift_num =
          current_node->GetGearSwitchNum() + rs_path_.gear_change_number;
      if (IsGearDifferent(current_node->GetGearType(),
                          rs_path_.GetFirstGear())) {
        shift_num++;
      }
      if (shift_num > request_.max_gear_shift_number) {
        // ILOG_INFO << "rs gear shift number is not expectation";
        return false;
      }
    }

    if (request_.scenario_type ==
            ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN &&
        rs_path_.GetLastGear() != AstarPathGear::REVERSE) {
      // ILOG_INFO << "rs last gear is not expectation";
      curve_node_to_goal->Clear();
      return false;
    }
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

#if PLOT_RS_EXNTEND_TO_END
  if (rs_path_h_cost_debug_.size() < RS_H_COST_MAX_NUM) {
    rs_path_h_cost_debug_.emplace_back(rs_path_);
  }
#endif

#if LOG_TIME_PROFILE
  const double set_curve_path_start_time = IflyTime::Now_ms();
#endif

  CurvePath& path = curve_node_to_goal->GetMutableCurvePath();
  path.path_dist = rs_path_.total_length;
  path.segment_size = rs_path_.size;
  path.ptss.resize(rs_path_.size);
  path.gear_change_number = rs_path_.gear_change_number;
  path.kappa_change = lpl_path_.kappa_change;
  for (int i = 0; i < rs_path_.size; ++i) {
    const RSPathSegment& single_rs_path = rs_path_.paths[i];
    path.dists[i] = std::fabs(single_rs_path.length);
    path.gears[i] = single_rs_path.gear;
    path.steers[i] = GetAstarSteerFromRsSteer(single_rs_path.steer);
    path.kappas[i] = single_rs_path.kappa;
    path.point_sizes[i] = single_rs_path.size;
    path.ptss[i].resize(single_rs_path.size);
    for (int j = 0; j < single_rs_path.size; j++) {
      path.ptss[i][j].SetPos(single_rs_path.points[j].x,
                             single_rs_path.points[j].y);
      path.ptss[i][j].SetTheta(single_rs_path.points[j].theta);
    }
  }

  // curve_node_to_goal->Set(path, search_map_boundary_, config_);
  curve_node_to_goal->Set(search_map_boundary_, config_);

#if LOG_TIME_PROFILE
  set_curve_path_time_ += (IflyTime::Now_ms() - set_curve_path_start_time);
#endif

  // ILOG_INFO << "rs path gear change number = " <<
  // rs_path_.gear_change_number;

  // Check curve_node_to_goal is valid
  NodeDeleteRequest node_del_request;
  node_del_request.cur_gear = rs_path_.paths[0].gear;
  node_del_request.curve_node = curve_node_to_goal;
  node_del_request.parent_node = current_node;

  if (CheckNodeShouldDelete(node_del_request)) {
    // ILOG_INFO << "rs col consume time = " << IflyTime::Now_ms() -
    // rs_start_time
    //           << "  ms";
    curve_node_to_goal->Clear();
    return false;
  }

  curve_node_to_goal->SetPathType(AstarPathType::NODE_CURVE);
  curve_node_to_goal->SetGearType(rs_path_.paths[0].gear);
  curve_node_to_goal->SetPre(current_node);

  // ILOG_INFO << "Reach the end configuration with Reeds Shepp";

  curve_node_to_goal->SetGCost(
      current_node->GetGCost() +
      CalcCurveNodeGCostToParentNode(current_node, curve_node_to_goal));
  curve_node_to_goal->SetHeuCost(0.0);
  curve_node_to_goal->SetFCost();

  AstarPathGear cur_gear;
  float cur_gear_length;
#if USE_LINK_PT_LINE
  common_math::PathPt<float> gear_switch_pose;
  common_math::PathPt<float> next_gear_switch_pose;
#else
  geometry_lib::PathPoint gear_switch_pose;
  geometry_lib::PathPoint next_gear_switch_pose;
#endif
  if (curve_node_to_goal->GearSwitchNode() != nullptr) {
    const Pose2f& pose = curve_node_to_goal->GearSwitchNode()->GetPose();
    gear_switch_pose.SetPos(pose.x, pose.y);
    gear_switch_pose.SetTheta(pose.theta);
    cur_gear = curve_node_to_goal->GearSwitchNode()->GetGearType();
    cur_gear_length = curve_node_to_goal->GearSwitchNode()->GetDistToStart();
  } else {
    double seg_length[6];
    int seg_num = 0;
    seg_length[seg_num] = path.dists[0];
    gear_switch_pose = path.ptss[0][0];
    for (int i = 1; i < path.segment_size; i++) {
      if (path.gears[i] == path.gears[i - 1]) {
        seg_length[seg_num] += path.dists[i];
      } else {
        seg_length[++seg_num] = path.dists[i];
        if (seg_num == 1) {
          gear_switch_pose = path.ptss[i][0];
        }
      }
    }
    cur_gear = path.gears[0];
    cur_gear_length = seg_length[0] + current_node->GetDistToStart();
  }

  if (curve_node_to_goal->NextGearSwitchNode() != nullptr) {
    const Pose2f& pose = curve_node_to_goal->NextGearSwitchNode()->GetPose();
    next_gear_switch_pose.SetPos(pose.x, pose.y);
    next_gear_switch_pose.SetTheta(pose.theta);
  } else if (curve_node_to_goal->GetGearSwitchNum() > 1) {
    if (curve_node_to_goal->GearSwitchNode() != nullptr) {
      // node和curve_path 换挡 找curve_path路径的第一个换挡点
      if (path.ptss.size() > 1) {
        next_gear_switch_pose = path.ptss[1][0];
      }
    } else {
      // node和curve_path 不换挡 找curve_path路径的第二个换挡点
      if (path.ptss.size() > 2) {
        next_gear_switch_pose = path.ptss[2][0];
      }
    }
  }

  curve_node_to_goal->SetCurGear(cur_gear);
  curve_node_to_goal->SetCurGearLength(cur_gear_length);
  curve_node_to_goal->SetGearSwitchPose(gear_switch_pose);
  curve_node_to_goal->SetNextGearSwitchPose(next_gear_switch_pose);

  // DebugCurvePath(path);
  //  ILOG_INFO << "rs success consume time = "
  //            << IflyTime::Now_ms() - rs_start_time << "  ms";
  return true;
}

const bool HybridAStarPathGenerator::AnalyticExpansionByLPL(
    Node3d* current_node, CurveNode* curve_node_to_goal,
#if USE_LINK_PT_LINE
    const link_pt_line::LinkPtLineInput<float>& input
#else
    const LinkPoseLineInput& input
#endif
) {
  curve_node_to_goal->Clear();
  if (std::fabs(current_node->GetPhi()) > M_PI_2f32) {
    return false;
  }

  if (!CalcLPLPathToGoal(current_node, input)) {
    return false;
  }

  // request check
  if (current_node->GetPathType() == AstarPathType::START_NODE) {
    // ILOG_INFO << "the cur node is start node";
    if (IsGearDifferent(request_.inital_action_request.ref_gear,
                        lpl_path_.cur_gear)) {
      // ILOG_INFO << "lpl first gear is not expectation";
      return false;
    }
    if (lpl_path_.gear_change_num > request_.max_gear_shift_number) {
      // ILOG_INFO << "lpl gear shift number is not expectation";
      return false;
    }
    if (lpl_path_.cur_gear_length < request_.inital_action_request.ref_length) {
      // ILOG_INFO << "lpl first gear length is not expectation";
      return false;
    }
    for (int i = 1; i < lpl_path_.gear_num; i++) {
      if (lpl_path_.single_gear_lengths[i] < request_.every_gear_length) {
        // ILOG_INFO << "rs seg every gear length is not expectation";
        return false;
      }
    }
  }

  else if (current_node->GetGearSwitchNum() == 0) {
    // ILOG_INFO << "the cur node is not start node, but from start node and cur
    // node donot shift gear";
    if (IsGearDifferent(current_node->GetGearType(), lpl_path_.cur_gear)) {
      if (lpl_path_.gear_change_num + 1 > request_.max_gear_shift_number) {
        // ILOG_INFO << "lpl gear shift number is not expectation";
        return false;
      }
      if (current_node->GetDistToStart() <
          request_.inital_action_request.ref_length) {
        // ILOG_INFO << "lpl first gear length is not expectation";
        return false;
      }
      for (int i = 0; i < lpl_path_.gear_num; i++) {
        if (lpl_path_.single_gear_lengths[i] < request_.every_gear_length) {
          // ILOG_INFO << "rs seg every gear length is not expectation";
          return false;
        }
      }
    } else {
      if (current_node->GetDistToStart() + lpl_path_.cur_gear_length <
          request_.inital_action_request.ref_length) {
        // ILOG_INFO << "lpl first gear length is not expectation";
        return false;
      }
      for (int i = 1; i < lpl_path_.gear_num; i++) {
        if (lpl_path_.single_gear_lengths[i] < request_.every_gear_length) {
          // ILOG_INFO << "lpl seg every gear length is not expectation";
          return false;
        }
      }
    }
  }

  else {
    // ILOG_INFO << "the cur node is not start node, and from start node and cur
    // node shift gear";
    int shift_num =
        current_node->GetGearSwitchNum() + rs_path_.gear_change_number;
    if (IsGearDifferent(current_node->GetGearType(), lpl_path_.cur_gear)) {
      shift_num++;
    }
    if (shift_num > request_.max_gear_shift_number) {
      // ILOG_INFO << "lpl gear shift number is not expectation";
      return false;
    }
  }

  if (IsGearDifferent(input.ref_last_line_gear, lpl_path_.last_gear)) {
    // ILOG_INFO << "lpl last gear is not expectation";
    curve_node_to_goal->Clear();
    return false;
  }

#if LOG_TIME_PROFILE
  const double lpl_interpolate_start_time = IflyTime::Now_ms();
#endif

  lpl_path_.SamplePath(request_.sample_ds);

#if LOG_TIME_PROFILE
  lpl_interpolate_time_ms_ += (IflyTime::Now_ms() - lpl_interpolate_start_time);
#endif

#if LOG_TIME_PROFILE
  const double set_curve_path_start_time = IflyTime::Now_ms();
#endif

  if (lpl_path_.ptss[0][0].GetX() > 3000.0f) {
    lpl_path_.PrintInfo();
  }

  CurvePath& path = curve_node_to_goal->GetMutableCurvePath();
  path.path_dist = lpl_path_.total_length;
  path.segment_size = lpl_path_.seg_num;
  path.ptss.resize(lpl_path_.seg_num);
  path.gear_change_number = lpl_path_.gear_change_num;
  path.kappa_change = lpl_path_.kappa_change;
  for (int i = 0; i < lpl_path_.seg_num; ++i) {
    const auto& pt_vec = lpl_path_.ptss[i];
    path.dists[i] = lpl_path_.lengths[i];
    path.kappas[i] = lpl_path_.kappas[i];
#if USE_LINK_PT_LINE
    path.gears[i] = lpl_path_.gears[i];
    path.steers[i] = lpl_path_.steers[i];
#else
    path.gears[i] = GetAstarGearFromSegGear(lpl_path_.gears[i]);
    path.steers[i] = GetAstarSteerFromSegSteer(lpl_path_.steers[i]);
#endif
    path.point_sizes[i] = lpl_path_.ptss[i].size();
    path.ptss[i] = std::move(lpl_path_.ptss[i]);
  }

  // curve_node_to_goal->Set(path, search_map_boundary_, config_);
  curve_node_to_goal->Set(search_map_boundary_, config_);

#if LOG_TIME_PROFILE
  set_curve_path_time_ += (IflyTime::Now_ms() - set_curve_path_start_time);
#endif

  // Check curve_node_to_goal is valid
  NodeDeleteRequest node_del_request;
  node_del_request.cur_gear = rs_path_.paths[0].gear;
  node_del_request.curve_node = curve_node_to_goal;
  node_del_request.parent_node = current_node;

  if (CheckNodeShouldDelete(node_del_request)) {
    // ILOG_INFO << "lpl col consume time = " << IflyTime::Now_ms() -
    // lpl_start_time << "  ms";
    curve_node_to_goal->Clear();
    return false;
  }

  curve_node_to_goal->SetPathType(AstarPathType::NODE_CURVE);
  curve_node_to_goal->SetGearType(rs_path_.paths[0].gear);
  curve_node_to_goal->SetPre(current_node);

  // ILOG_INFO << "Reach the end configuration with Link Pose Line Path";

  curve_node_to_goal->SetGCost(
      current_node->GetGCost() +
      CalcCurveNodeGCostToParentNode(current_node, curve_node_to_goal));
  curve_node_to_goal->SetHeuCost(0.0);
  curve_node_to_goal->SetFCost();

  AstarPathGear cur_gear;
  float cur_gear_length;
#if USE_LINK_PT_LINE
  common_math::PathPt<float> gear_switch_pose;
  common_math::PathPt<float> next_gear_switch_pose;
#else
  geometry_lib::PathPoint gear_switch_pose;
  geometry_lib::PathPoint next_gear_switch_pose;
#endif
  if (curve_node_to_goal->GearSwitchNode() != nullptr) {
    const Pose2f& pose = curve_node_to_goal->GearSwitchNode()->GetPose();
    gear_switch_pose.SetPos(pose.x, pose.y);
    gear_switch_pose.SetTheta(pose.theta);
    cur_gear = curve_node_to_goal->GearSwitchNode()->GetGearType();
    cur_gear_length = curve_node_to_goal->GearSwitchNode()->GetDistToStart();
  } else {
    double seg_length[6];
    int seg_num = 0;
    seg_length[seg_num] = path.dists[0];
    gear_switch_pose = path.ptss[0][0];
    for (int i = 1; i < path.segment_size; i++) {
      if (path.gears[i] == path.gears[i - 1]) {
        seg_length[seg_num] += path.dists[i];
      } else {
        seg_length[++seg_num] = path.dists[i];
        if (seg_num == 1) {
          gear_switch_pose = path.ptss[i][0];
        }
      }
    }
    cur_gear = path.gears[0];
    cur_gear_length = seg_length[0] + current_node->GetDistToStart();
  }

  if (curve_node_to_goal->NextGearSwitchNode() != nullptr) {
    const Pose2f& pose = curve_node_to_goal->NextGearSwitchNode()->GetPose();
    next_gear_switch_pose.SetPos(pose.x, pose.y);
    next_gear_switch_pose.SetTheta(pose.theta);
  } else if (curve_node_to_goal->GetGearSwitchNum() > 1) {
    if (curve_node_to_goal->GearSwitchNode() != nullptr) {
      // node和curve_path 换挡 找curve_path路径的第一个换挡点
      if (path.ptss.size() > 1) {
        next_gear_switch_pose = path.ptss[1][0];
      }
    } else {
      // node和curve_path 不换挡 找curve_path路径的第二个换挡点
      if (path.ptss.size() > 2) {
        next_gear_switch_pose = path.ptss[2][0];
      }
    }
  }

  curve_node_to_goal->SetCurGear(cur_gear);
  curve_node_to_goal->SetCurGearLength(cur_gear_length);
  curve_node_to_goal->SetGearSwitchPose(gear_switch_pose);
  curve_node_to_goal->SetNextGearSwitchPose(next_gear_switch_pose);

  // lpl_path_.PrintInfo();

  // DebugCurvePath(path);
  //  ILOG_INFO << "lpl success consume time = "
  //            << IflyTime::Now_ms() - lpl_start_time << "  ms";
  return true;
}

const float HybridAStarPathGenerator::CalcCurveNodeGCostToParentNode(
    Node3d* current_node, CurveNode* curve_node) {
  // evaluate cost on the trajectory and add current cost
  float length_cost = 0.0, gear_change_cost = 0.0, kappa_change_cost = 0.0;
  const CurvePath& path = curve_node->GetCurvePath();

  if (path.segment_size == 0) {
    return kMaxNodeCost;
  }

  length_cost = path.path_dist * config_.traj_forward_penalty;
  gear_change_cost = config_.gear_switch_penalty * path.gear_change_number;
  kappa_change_cost = config_.traj_kappa_change_penalty * path.kappa_change;

  curve_node->SetGearSwitchNum(current_node->GetGearSwitchNum());
  curve_node->AddGearSwitchNumber(path.gear_change_number);

  // gear cost
  if (current_node->IsPathGearChange(path.gears[0])) {
    gear_change_cost += config_.gear_switch_penalty;
    curve_node->AddGearSwitchNumber();
  }

  const bool gear_switch = current_node->IsPathGearChange(path.gears[0]) ||
                           (path.gear_change_number > 0);

  // steer change cost
  if (!current_node->IsPathGearChange(path.gears[0])) {
    kappa_change_cost += config_.traj_kappa_change_penalty *
                         std::fabs(current_node->GetKappa() - path.kappas[0]);
  }

  curve_node->SetDistToStart(path.path_dist + current_node->GetDistToStart());

  if (current_node->GetGearSwitchNum() > 0) {
    curve_node->SetGearSwitchNode(current_node->GearSwitchNode());
  } else if (curve_node->GetGearSwitchNum() == 0) {
    curve_node->SetGearSwitchNode(nullptr);
  } else if (gear_switch) {
    curve_node->SetGearSwitchNode(current_node);
  } else {
    curve_node->SetGearSwitchNode(nullptr);
  }

  if (current_node->GetGearSwitchNum() > 1) {
    curve_node->SetNextGearSwitchNode(current_node->NextGearSwitchNode());
  } else if (current_node->GetGearSwitchNum() == 1) {
    if (curve_node->GetGearSwitchNum() == 1) {
      curve_node->SetNextGearSwitchNode(nullptr);
    } else {
      curve_node->SetNextGearSwitchNode(current_node);
    }
  } else {
    if (curve_node->GetGearSwitchNum() < 2) {
      curve_node->SetNextGearSwitchNode(nullptr);
    } else {
      curve_node->SetNextGearSwitchNode(current_node);
    }
  }

  // if (current_node->GetGearSwitchNum() > 1) {
  //   curve_node->SetNextGearSwitchNode(current_node->NextGearSwitchNode());
  // } else if (curve_node->GetGearSwitchNum() < 2) {
  //   curve_node->SetNextGearSwitchNode(nullptr);
  // } else {
  //   if (current_node->GetGearSwitchNum() == 0) {
  //     curve_node->SetNextGearSwitchNode(nullptr);
  //   } else if (current_node->GetGearSwitchNum() == 1) {
  //     if (gear_switch) {
  //       curve_node->SetNextGearSwitchNode(current_node);
  //     } else {
  //       curve_node->SetNextGearSwitchNode(nullptr);
  //     }
  //   }
  // }

  return length_cost + gear_change_cost + kappa_change_cost;
}

const bool HybridAStarPathGenerator::CalcRSPathToGoal(
    Node3d* current_node, const bool need_rs_dense_point,
    const bool need_anchor_point, const RSPathRequestType rs_request,
    const float rs_radius, const bool cal_h_cost) {
#if LOG_TIME_PROFILE
  const double rs_start_time = IflyTime::Now_ms();
#endif

  // rs_path_.Clear();

  const Pose2f& start_pose = current_node->GetPose();
  Pose2f end_pose = end_node_->GetPose();

  if (request_.scenario_type ==
          ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN &&
      request_.search_mode == SearchMode::FORMAL && !cal_h_cost) {
    end_pose.x += 1.68;
  }

  bool is_connected_to_goal;
  rs_path_interface_.GeneShortestRSPath(
      &rs_path_, &is_connected_to_goal, &start_pose, &end_pose, rs_radius,
      need_rs_dense_point, need_anchor_point, rs_request);

#if LOG_TIME_PROFILE
  rs_time_ms_ += (IflyTime::Now_ms() - rs_start_time);
  rs_try_num_++;
#endif

  if (!is_connected_to_goal || rs_path_.total_length < 0.01 ||
      rs_path_.size < 1) {
    ILOG_INFO << "REEDS_SHEEP path failed";
    return false;
  }

  if (request_.scenario_type ==
          ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN &&
      request_.search_mode == SearchMode::FORMAL && !cal_h_cost) {
    RSPathSegment rs_seg;
    rs_seg.gear = AstarPathGear::REVERSE;
    rs_seg.steer = RSPathSteer::RS_STRAIGHT;
    rs_seg.kappa = 0.0;
    rs_seg.length = end_node_->GetPose().x - end_pose.x;
    rs_seg.size = 1;
    rs_seg.points[0].x = end_pose.x;
    rs_seg.points[0].y = end_pose.y;
    rs_seg.points[0].theta = end_pose.theta;
    rs_seg.points[0].kappa = 0.0;
    rs_seg.points[0].dir = AstarPathGear::REVERSE;

    if (rs_path_.paths[rs_path_.size - 1].gear == AstarPathGear::DRIVE) {
      rs_path_.gear_change_number += 1;
    }
    rs_path_.total_length += rs_seg.length;
    rs_path_.paths[rs_path_.size++] = rs_seg;
  }

  return true;
}

const bool HybridAStarPathGenerator::CalcLPLPathToGoal(
    Node3d* current_node,
#if USE_LINK_PT_LINE
    const link_pt_line::LinkPtLineInput<float>& input,
#else
    const LinkPoseLineInput& input,
#endif
    const bool cal_h_cost) {
#if LOG_TIME_PROFILE
  const double lpl_start_time = IflyTime::Now_ms();
#endif

  // lpl_path_.Reset();

  const bool success = lpl_interface_.CalLPLPath(input);

#if LOG_TIME_PROFILE
  lpl_time_ms_ += (IflyTime::Now_ms() - lpl_start_time);
  lpl_try_num_++;
#endif

  if (!success) {
    return false;
  }

  const auto& lpl_output = lpl_interface_.GetLPLOutput();

  const float gear_change_cost = 10.0f, length_cost = 1.0f;
  const float kappa_change_cost =
      request_.adjust_pose ? 0.0f : (2.5f * length_cost / max_kappa_change_);
  float min_cost = std::numeric_limits<float>::infinity();
  uint8_t min_index = 0;
  for (uint8_t i = 0; i < lpl_output.lpl_path_num; i++) {
    float cost = 0.0;
    const auto& lpl_path = lpl_output.lpl_paths[i];
    cost = length_cost * lpl_path.total_length;
    if (!cal_h_cost) {
      cost += (gear_change_cost * lpl_path.gear_change_num +
               kappa_change_cost * lpl_path.kappa_change);
    }
    if (cost < min_cost) {
      min_index = i;
      min_cost = cost;
    }
  }

  lpl_path_ = lpl_output.lpl_paths[min_index];

  return true;
}

const float HybridAStarPathGenerator::GenerateHeuristicCostByRsPath(
    Node3d* next_node, NodeHeuristicCost* cost) {
  RSPathRequestType rs_request = RSPathRequestType::NONE;
  if (!CalcRSPathToGoal(next_node, false, false, rs_request, min_radius_,
                        true)) {
    return 100.0;
  }

  return rs_path_.total_length * config_.traj_forward_penalty;

  float length_cost = 0.0, gear_change_cost = 0.0, steer_cost = 0.0,
        steer_change_cost = 0.0;
  length_cost = rs_path_.total_length * config_.traj_forward_penalty;

  // for (int i = 0; i < rs_path_.size - 1; i++) {
  //   // gear cost
  //   if (rs_path_.paths[i].gear != rs_path_.paths[i + 1].gear) {
  //     gear_change_cost += config_.gear_switch_penalty_heu;
  //   }

  //   // steer cost
  //   if (rs_path_.paths[i].steer != RS_STRAIGHT) {
  //     steer_cost += config_.traj_steer_penalty * max_front_wheel_angle_;
  //   }

  //   // steer change cost
  //   if (rs_path_.paths[i].steer != rs_path_.paths[i + 1].steer) {
  //     if (rs_path_.paths[i].steer == RS_STRAIGHT ||
  //         rs_path_.paths[i + 1].steer == RS_STRAIGHT) {
  //       steer_change_cost +=
  //           config_.traj_steer_change_penalty * max_front_wheel_angle_;
  //     } else {
  //       steer_change_cost +=
  //           config_.traj_steer_change_penalty * max_front_wheel_angle_ * 2;
  //     }
  //   }
  // }

  // // gear cost
  // if (next_node->GetGearType() != rs_path_.paths[0].gear) {
  //   gear_change_cost += config_.gear_switch_penalty_heu;
  // }

  // // steer cost, need modify
  // if (next_node->GetSteer() > 0.0 && rs_path_.paths[0].steer != RS_LEFT) {
  //   steer_change_cost +=
  //       config_.traj_steer_change_penalty * max_front_wheel_angle_;
  // } else if (next_node->GetSteer() < 0.0 &&
  //            rs_path_.paths[0].steer != RS_RIGHT) {
  //   steer_change_cost +=
  //       config_.traj_steer_change_penalty * max_front_wheel_angle_;
  // }

  // cost->rs_path_dist = length_cost;
  // cost->rs_path_gear = gear_change_cost;
  // cost->rs_path_steer = steer_change_cost;

  float collision_cost = 0.0;

#if PLOT_RS_COST_PATH
  if (rs_path_h_cost_debug_.size() < RS_H_COST_MAX_NUM) {
    const Pose2D& rs_start_pose = next_node->GetPose();
    rs_path_interface_.RSPathInterpolate(&rs_path_, &rs_start_pose,
                                         min_radius_);
    rs_path_h_cost_debug_.emplace_back(rs_path_);
  }
#endif

  return length_cost + gear_change_cost + steer_cost + steer_change_cost +
         collision_cost;
}

const float HybridAStarPathGenerator::GenerateHeuristicCostByLPLPath(
    Node3d* next_node,
#if USE_LINK_PT_LINE
    const link_pt_line::LinkPtLineInput<float>& input,
#else
    const LinkPoseLineInput& input,
#endif
    NodeHeuristicCost* cost) {
  if (!CalcLPLPathToGoal(next_node, input, true)) {
    ILOG_INFO << "cal LPL path failed";
    input.pose.PrintInfo();
    // input.ref_line.PrintInfo();
    return 100.0f;
  }

  return std::min(float(lpl_path_.total_length), 36.8f) *
         config_.traj_forward_penalty;

  float length_cost = 0.0f, gear_change_cost = 0.0f, steer_cost = 0.0f,
        steer_change_cost = 0.0f;
  length_cost = lpl_path_.total_length * config_.traj_forward_penalty;

  float collision_cost = 0.0f;

  return length_cost + gear_change_cost + steer_cost + steer_change_cost +
         collision_cost;
}

void HybridAStarPathGenerator::InitNodePool() { node_pool_.Init(); }

void HybridAStarPathGenerator::ResetNodePool() { node_pool_.Clear(); }

const bool HybridAStarPathGenerator::CheckOutOfGridBound(
    const NodeGridIndex& id) const {
  if (id.x < 0 || id.x >= search_map_grid_boundary_.x || id.y < 0 ||
      id.y >= search_map_grid_boundary_.y || id.phi < 0 ||
      id.phi >= search_map_grid_boundary_.phi) {
    return false;
  }

  return true;
}

const bool HybridAStarPathGenerator::CheckOutOfPoseBound(
    const Pose2D& pose) const {
  if (pose.x > search_map_boundary_.x_max ||
      pose.x < search_map_boundary_.x_min ||
      pose.y > search_map_boundary_.y_max ||
      pose.y < search_map_boundary_.y_min ||
      pose.theta > search_map_boundary_.phi_max ||
      pose.theta < search_map_boundary_.phi_min) {
    return true;
  }
  return false;
}

void HybridAStarPathGenerator::GenerateNextNode(Node3d* new_node,
                                                Node3d* parent_node,
                                                const CarMotion& car_motion) {
#if LOG_TIME_PROFILE
  const double generate_next_node_start_time = IflyTime::Now_ms();
#endif

  new_node->ClearPath();

  const NodePath path =
      GetNodePathByCarMotion(parent_node->GetPose(), car_motion);

  new_node->Set(path, search_map_boundary_, config_, path.path_dist);

#if LOG_TIME_PROFILE
  generate_next_node_time_ +=
      (IflyTime::Now_ms() - generate_next_node_start_time);
#endif
}

const bool HybridAStarPathGenerator::CheckNodeShouldDelete(
    const NodeDeleteRequest& request) {
#if LOG_TIME_PROFILE
  const double node_del_start_time = IflyTime::Now_ms();
#endif

  const bool del_flag = node_delete_decider_.CheckShouldBeDeleted(request);

#if LOG_TIME_PROFILE
  check_node_should_del_time_ += (IflyTime::Now_ms() - node_del_start_time);
#endif

  return del_flag;
}

void HybridAStarPathGenerator::CalcObsDistRelativeSlot(
    const CurveNode& curve_node,
    ObsToPathDistRelativeSlot& obs_dist_relative_slot) {
  obs_dist_relative_slot.SetSmallerDist(curve_node.GetObsDistRelativeSlot());
  const Node3d* pre_node = curve_node.GetPreNode();
  while (pre_node != nullptr) {
    obs_dist_relative_slot.SetSmallerDist(pre_node->GetObsDistRelativeSlot());
    pre_node = pre_node->GetPreNode();
  }
}

const float HybridAStarPathGenerator::CalcGearChangePoseCost(
#if USE_LINK_PT_LINE
    const common_math::PathPt<float>& gear_switch_pose,
#else
    const geometry_lib::PathPoint& gear_switch_pose,
#endif
    AstarPathGear gear, const float gear_switch_penalty,
    const float length_penalty) {
  return 0.0f;
}

const bool HybridAStarPathGenerator::BackwardPassByCurveNode(
    const CurveNode* curve_node_to_goal) {
  const CurvePath& path = curve_node_to_goal->GetCurvePath();
  if (path.segment_size < 1) {
    ILOG_ERROR << "there are no path";
    return false;
  }

  // if (request_.is_searching_stage) {
  //   result_.path_plan_success = true;
  //   return true;
  // }

  const double backward_start_time = IflyTime::Now_ms();
  if (request_.analytic_expansion_type ==
      AnalyticExpansionType::LINK_POSE_LINE) {
    curve_node_to_goal->GetLPLPath().PrintInfo();
  }
  ILOG_INFO << "curve_node_to_goal gear = "
            << PathGearDebugString(curve_node_to_goal->GetCurGear())
            << "  cur_gear_length = " << curve_node_to_goal->GetCurGearLength();
  curve_node_to_goal->GetGearSwitchPose().PrintInfo();
  // DebugCurvePath(path);

  std::vector<Node3d*> node_list;
  Node3d* parent_node = nullptr;
  Node3d* child_node = curve_node_to_goal->GetPreNode();
  if (child_node == nullptr) {
    ILOG_ERROR << "curve_node_to_goal pre node is nullptr";
    return false;
  }
  while (child_node->GetPreNode() != nullptr) {
    parent_node = child_node->GetPreNode();
    parent_node->SetNext(child_node);
    child_node = parent_node;
  }

  // the chile node is start node
  AstarPathGear cur_gear_type = AstarPathGear::NONE;
  AstarPathGear next_gear_type = AstarPathGear::NONE;
  bool gear_change_flag = false;
  std::vector<float> x_vec, y_vec, phi_vec, kappa_vec;
  std::vector<AstarPathType> type_vec;
  child_node = child_node->GetNextNode();
  // if the child node is not nullptr, that is first search node
  // otherwise is curve node
  size_t search_node_num = 0;
  while (child_node != nullptr) {
    search_node_num++;
    const NodePath& path = child_node->GetNodePath();
    const size_t start_index = x_vec.empty() ? 0 : 1;
    for (size_t i = start_index; i < path.point_size; i++) {
      x_vec.emplace_back(path.points[i].x);
      y_vec.emplace_back(path.points[i].y);
      phi_vec.emplace_back(path.points[i].theta);
      kappa_vec.emplace_back(child_node->GetKappa());
      type_vec.emplace_back(AstarPathType::NODE_SEARCHING);
    }

    cur_gear_type = child_node->GetGearType();

    Node3d* next_node = child_node->GetNextNode();
    if (next_node == nullptr) {
      // the child node is the last search node
      break;
    }
    next_gear_type = next_node->GetGearType();

    gear_change_flag = IsGearDifferent(cur_gear_type, next_gear_type);

    if (gear_change_flag) {
      result_.gear_vec.emplace_back(cur_gear_type);
      result_.x_vec_vec.emplace_back(x_vec);
      result_.y_vec_vec.emplace_back(y_vec);
      result_.phi_vec_vec.emplace_back(phi_vec);
      result_.kappa_vec_vec.emplace_back(kappa_vec);
      result_.type_vec_vec.emplace_back(type_vec);
      x_vec.clear();
      y_vec.clear();
      phi_vec.clear();
      kappa_vec.clear();
      type_vec.clear();
    }

    child_node = next_node;
  }

  ILOG_INFO << "there are " << search_node_num << " search node";
  ILOG_INFO << " curve path start pose";
  path.ptss.front().front().PrintInfo();

  if (IsGearDifferent(cur_gear_type, path.gears[0])) {
    result_.gear_vec.emplace_back(cur_gear_type);
    result_.x_vec_vec.emplace_back(x_vec);
    result_.y_vec_vec.emplace_back(y_vec);
    result_.phi_vec_vec.emplace_back(phi_vec);
    result_.kappa_vec_vec.emplace_back(kappa_vec);
    result_.type_vec_vec.emplace_back(type_vec);
    x_vec.clear();
    y_vec.clear();
    phi_vec.clear();
    kappa_vec.clear();
    type_vec.clear();
  }

  for (size_t i = 0; i < path.segment_size; ++i) {
    const size_t start_index = x_vec.empty() ? 0 : 1;
    for (size_t j = start_index; j < path.point_sizes[i]; j++) {
      x_vec.emplace_back(path.ptss[i][j].GetX());
      y_vec.emplace_back(path.ptss[i][j].GetY());
      phi_vec.emplace_back(path.ptss[i][j].GetTheta());
      kappa_vec.emplace_back(path.ptss[i][j].kappa);
      type_vec.emplace_back(AstarPathType::NODE_CURVE);
    }
    cur_gear_type = path.gears[i];
    if (i < path.segment_size - 1) {
      next_gear_type = path.gears[i + 1];
      gear_change_flag = IsGearDifferent(cur_gear_type, next_gear_type);
    } else {
      gear_change_flag = true;
    }
    if (gear_change_flag) {
      result_.gear_vec.emplace_back(cur_gear_type);
      result_.x_vec_vec.emplace_back(x_vec);
      result_.y_vec_vec.emplace_back(y_vec);
      result_.phi_vec_vec.emplace_back(phi_vec);
      result_.kappa_vec_vec.emplace_back(kappa_vec);
      result_.type_vec_vec.emplace_back(type_vec);
      x_vec.clear();
      y_vec.clear();
      phi_vec.clear();
      kappa_vec.clear();
      type_vec.clear();
    }
  }

  result_.path_plan_success = result_.gear_vec.size() > 0;

  if (!result_.path_plan_success) {
    ILOG_INFO << "path_plan_success failed";
    result_.Clear();
    return false;
  }

  std::vector<float> s_vec;
  s_vec.reserve(result_.x_vec_vec[0].size());
  float s = 0.0;
  for (size_t i = 0; i < result_.x_vec_vec.size(); i++) {
    const std::vector<float>& x_vec = result_.x_vec_vec[i];
    const std::vector<float>& y_vec = result_.y_vec_vec[i];
    s_vec.emplace_back(s);
    for (size_t j = 1; j < x_vec.size(); ++j) {
      s += std::hypot(x_vec[j] - x_vec[j - 1], y_vec[j] - y_vec[j - 1]);
      s_vec.emplace_back(s);
    }
    result_.accumulated_s_vec_vec.emplace_back(s_vec);
    s_vec.clear();
    s_vec.reserve(x_vec.size());
  }

  result_.cur_gear = result_.gear_vec.front();
  result_.gear_change_num = result_.gear_vec.size() - 1;

  for (size_t i = 0; i < result_.gear_vec.size(); i++) {
    result_.length_vec.emplace_back(
        std::max(0.1 * (result_.x_vec_vec[i].size() - 1), 0.1));
  }

#if DEBUG_FINAL_PATH
  for (size_t i = 0; i < result_.x_vec_vec.size(); i++) {
    for (size_t j = 0; j < result_.x_vec_vec[i].size(); j++) {
      ILOG_INFO << "x = " << result_.x_vec_vec[i][j]
                << "  y = " << result_.y_vec_vec[i][j] << "  phi = "
                << result_.phi_vec_vec[i][j] * common_math::kRad2DegF
                << "  kappa = " << result_.kappa_vec_vec[i][j]
                << "  type = " << static_cast<int>(result_.type_vec_vec[i][j])
                << "  gear = " << static_cast<int>(result_.gear_vec[i]);
    }
  }
#endif

  ILOG_INFO << "backward pass time = "
            << IflyTime::Now_ms() - backward_start_time << " ms";

  return true;
}

void HybridAStarPathGenerator::ChooseBestCurveNode(
    const std::vector<CurveNode>& curve_node_vec,
    const AnalyticExpansionType analytic_expansion_type,
    const bool consider_obs_dist, const PathColDetBuffer& safe_buffer,
    CurveNode& best_curve_node) {
  return;
}

void HybridAStarPathGenerator::DebugNodePath(const NodePath& path,
                                             const AstarPathGear gear) {
  ILOG_INFO << "debug node path, path dist = " << path.path_dist
            << "  point size = " << path.point_size
            << " gear = " << PathGearDebugString(gear);
  for (size_t i = 0; i < path.point_size; i++) {
    path.points[i].DebugString();
  }
}

void HybridAStarPathGenerator::DebugCurvePath(const CurvePath& path) {
  ILOG_INFO << "debug curve path,  path dist = " << path.path_dist
            << "  segment size = " << path.segment_size
            << "  gear_change_number = " << path.gear_change_number;
  for (size_t i = 0; i < path.segment_size; i++) {
    ILOG_INFO
        << "segment " << i << "  dist = " << path.dists[i]
        << "  gear = " << static_cast<int>(path.gears[i]) << "  steer = "
        << static_cast<int>(path.steers[i])
        // << "  front wheel angle = " << path.segment_front_wheel_angles[i]
        << "  kappa = " << path.kappas[i]
        << "  point size = " << path.point_sizes[i];
    const int jump_number =
        std::max(static_cast<int>(path.point_sizes[i] / 5), 1);
    for (size_t j = 0; j < path.point_sizes[i]; j += 1) {
      path.ptss[i][j].PrintInfo();
    }
  }
}

const std::vector<DebugAstarSearchPoint>&
HybridAStarPathGenerator::GetChildNodeForDebug() {
  return child_node_debug_;
}

const std::vector<Eigen::Vector2d>&
HybridAStarPathGenerator::GetQueuePathForDebug() {
  return queue_path_debug_;
}

const std::vector<Eigen::Vector2d>&
HybridAStarPathGenerator::GetDeleteQueuePathForDebug() {
  return delete_queue_path_debug_;
}

const std::vector<std::vector<Eigen::Vector2d>>&
HybridAStarPathGenerator::GetSearchNodeListMessage() {
  all_search_node_list_.clear();
  all_search_node_list_.reserve(node_set_.size());
  for (auto i = node_set_.begin(); i != node_set_.end(); i++) {
    if (i->second->GetStepSize() < 2 ||
        i->second->GetPathType() != AstarPathType::NODE_SEARCHING) {
      continue;
    }

    const NodePath& path = i->second->GetNodePath();

    std::vector<Eigen::Vector2d> node;
    for (size_t m = 0; m < path.point_size; m++) {
      node.emplace_back(Eigen::Vector2d(path.points[m].x, path.points[m].y));
    }

    all_search_node_list_.emplace_back(node);
  }

  return all_search_node_list_;
}

const std::vector<std::vector<Eigen::Vector2d>>&
HybridAStarPathGenerator::GetCurveNodeListMessage() {
  all_curve_node_list_.clear();
  return all_curve_node_list_;
}

const std::vector<CurvePath>&
HybridAStarPathGenerator::GetAllSuccessCurvePathForDebug() {
  return all_success_curve_path_debug_;
}

const std::vector<geometry_lib::PathPoint>
HybridAStarPathGenerator::GetAllSuccessCurvePathFirstGearSwitchPoseForDebug() {
#if USE_LINK_PT_LINE
  std::vector<geometry_lib::PathPoint> poses(
      all_success_path_first_gear_switch_pose_debug_.size());
  for (size_t i = 0; i < all_success_path_first_gear_switch_pose_debug_.size();
       i++) {
    const auto& pose = all_success_path_first_gear_switch_pose_debug_[i];
    poses[i].SetPos(pose.GetX(), pose.GetY());
    poses[i].SetTheta(pose.GetTheta());
  }
  return poses;
#else
  return all_success_path_first_gear_switch_pose_debug_;
#endif
}

const cdl::AABB& HybridAStarPathGenerator::GetPreSearchABBoxForDebug() {
  return pre_search_abbox_;
}

CompactNodePool HybridAStarPathGenerator::node_pool_;
GridSearch HybridAStarPathGenerator::grid_search_;

}  // namespace apa_planner
}  // namespace planning