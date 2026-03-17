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
  bool ret = false;
  Node3d* current_node = request.current_node;
  CurveNode* curve_node = request.curve_node_to_goal;
  curve_node->Clear();

  if (request.type == AnalyticExpansionType::REEDS_SHEEP) {
    ret = AnalyticExpansionByRS(current_node, curve_node, request.rs_radius,
                                request.need_rs_dense_point,
                                request.need_anchor_point, request.rs_request);
  } else if (request.type == AnalyticExpansionType::LINK_POSE_LINE) {
    ret = AnalyticExpansionByLPL(current_node, curve_node, *request.lpl_input);
  }

  if (request_.search_mode == SearchMode::FORMAL) {
    const CurvePath& path = curve_node->GetCurvePath();
    // request check
    const AstarPathGear ref_gear = request_.inital_action_request.ref_gear;
    const float ref_length = request_.inital_action_request.ref_length;
    const float min_single_gear_length = request_.every_gear_length;
    AstarPathGear cur_gear = current_node->GetGearType();
    const bool gear_change = IsGearDifferent(path.gears[0], cur_gear);
    float cur_length = current_node->GetSingleGearLength();
    int index = 0;
    int gear_change_num = path.gear_change_number;
    if (current_node->GetPathType() == AstarPathType::START_NODE) {
      cur_gear = path.gears[0];
      cur_length = path.single_gear_lengths[0];
      index = 1;
    } else if (current_node->GetGearSwitchNum() == 0) {
      if (gear_change) {
        gear_change_num++;
        cur_length = current_node->GetSingleGearLength();
      } else {
        cur_length =
            current_node->GetSingleGearLength() + path.single_gear_lengths[0];
        index = 1;
      }
    } else {
      gear_change_num += current_node->GetGearSwitchNum();
      cur_length = current_node->GearSwitchNode()->GetSingleGearLength();
      if (gear_change) {
        gear_change_num++;
      } else {
        index = 1;
      }
    }
    if (ret && (IsGearDifferent(ref_gear, cur_gear) ||
                gear_change_num > request_.max_gear_shift_number ||
                cur_length < ref_length)) {
      ret = false;
    }
    for (int i = index; ret && i < path.gear_number; i++) {
      if (path.single_gear_lengths[i] < min_single_gear_length) {
        ret = false;
      }
    }

    if (ret) {
      if (request_.scenario_type ==
              ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN &&
          IsGearDifferent(path.gears[path.segment_size - 1],
                          AstarPathGear::REVERSE)) {
        ret = false;
      }
      if (request_.scenario_type ==
              ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN &&
          IsGearDifferent(path.gears[path.segment_size - 1],
                          AstarPathGear::DRIVE)) {
        ret = false;
      }
    }
  }

  NodeDeleteRequest node_del_request;
  node_del_request.curve_node = curve_node;
  node_del_request.parent_node = current_node;

  if (ret && CheckNodeShouldDelete(node_del_request)) {
    return false;
  }

  if (!ret) {
    curve_node->Clear();
    return false;
  }

  curve_node->SetGCost(
      current_node->GetGCost() +
      CalcCurveNodeGCostToParentNode(current_node, curve_node));
  curve_node->SetHeuCost(0.0);
  curve_node->SetFCost();

  return true;
}

const bool HybridAStarPathGenerator::AnalyticExpansionByRS(
    Node3d* current_node, CurveNode* curve_node_to_goal, const float rs_radius,
    const bool need_rs_dense_point, const bool need_anchor_point,
    const RSPathRequestType rs_request) {
  if (request_.search_mode == SearchMode::FORMAL) {
    if (request_.scenario_type ==
        ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN) {
      if (std::fabs(current_node->GetPhi()) > M_PI_2f32) {
        return false;
      }
    } else if (request_.scenario_type ==
               ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
      if (std::fabs(current_node->GetPhi()) < M_PI_2f32) {
        return false;
      }
    }
  }

  if (!CalcRSPathToGoal(current_node, need_rs_dense_point, need_anchor_point,
                        rs_request, rs_radius)) {
    return false;
  }

  // interpolation
#if LOG_TIME_PROFILE
  const double rs_start_time = IflyTime::Now_ms();
#endif

  const Pose2f& start_pose = current_node->GetPose();
  rs_path_interface_.RSPathInterpolate(&rs_path_, &start_pose, rs_radius);

#if LOG_TIME_PROFILE
  rs_interpolate_time_ms_ += (IflyTime::Now_ms() - rs_start_time);
  const double set_curve_path_start_time = IflyTime::Now_ms();
#endif

  CurvePath& path = curve_node_to_goal->GetMutableCurvePath();
  path.path_dist = rs_path_.total_length;
  path.segment_size = rs_path_.size;
  path.ptss.resize(rs_path_.size);
  path.gear_change_number = rs_path_.gear_change_number;
  path.kappa_change = 0.0f;
  path.last_path_kappa_change = 0.0f;
  path.gear_number = 1;
  path.single_gear_lengths[path.gear_number - 1] =
      std::fabs(rs_path_.paths[0].length);
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
    if (i > 0) {
      if (IsGearDifferent(path.gears[i], path.gears[i - 1])) {
        path.single_gear_lengths[(++path.gear_number) - 1] = path.dists[i];
        path.last_path_kappa_change = 0.0f;
      } else {
        path.single_gear_lengths[path.gear_number - 1] += path.dists[i];
        path.kappa_change += std::fabs(path.kappas[i] - path.kappas[i - 1]);
        path.last_path_kappa_change +=
            std::fabs(path.kappas[i] - path.kappas[i - 1]);
      }
    }
  }

  curve_node_to_goal->Set(search_map_boundary_, config_);

#if LOG_TIME_PROFILE
  set_curve_path_time_ += (IflyTime::Now_ms() - set_curve_path_start_time);
#endif

  return true;
}

const bool HybridAStarPathGenerator::AnalyticExpansionByLPL(
    Node3d* current_node, CurveNode* curve_node_to_goal,
    const link_pt_line::LinkPtLineInput<float>& input) {
  if (request_.search_mode == SearchMode::FORMAL) {
    const float x = current_node->GetX();
    const float y = current_node->GetY();
    const float abs_y = std::fabs(y);
    const float phi = current_node->GetPhi() * common_math::kRad2DegF;
    const float abs_phi = std::fabs(phi);
    if (request_.scenario_type ==
        ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN) {
      if (cul_de_sac_info_.is_cul_de_sac) {
        if (cul_de_sac_info_.type == CulDeSacType::RIGHT &&
            phi < cul_de_sac_info_.limit_phi) {
          return false;
        } else if (cul_de_sac_info_.type == CulDeSacType::LEFT &&
                   phi > cul_de_sac_info_.limit_phi) {
          return false;
        }
      }

      if (abs_phi > 90.0f) {
        return false;
      }
      if (y * phi < 0.0f && abs_y > 5.0f) {
        return false;
      }
      if (x < 3.0f && abs_phi > 68.0f && abs_y > 3.0f) {
        return false;
      }
      const std::vector<float> y_t{1.368f, 2.368f, 3.368f};
      const std::vector<float> phi_t{18.6f, 28.6f, 38.6f};
      for (int i = 0; i < y_t.size(); i++) {
        if (abs_y > y_t[i] && abs_phi < phi_t[i]) {
          return false;
        }
      }
    } else if (request_.scenario_type ==
               ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
      if (abs_phi < 90.0f) {
        return false;
      }
      if (y * phi > 0.0f && abs_y > 5.0f) {
        return false;
      }
      if (x < 5.0f && abs_phi < 132.0f && abs_y > 3.6f) {
        return false;
      }
      const std::vector<float> y_t{1.368f, 2.368f, 3.368f};
      const std::vector<float> phi_t{161.4f, 151.4f, 141.4f};
      for (int i = 0; i < y_t.size(); i++) {
        if (abs_y > y_t[i] && abs_phi > phi_t[i]) {
          return false;
        }
      }
    }
  }
  if (!CalcLPLPathToGoal(current_node, input)) {
    return false;
  }

#if LOG_TIME_PROFILE
  const double lpl_interpolate_start_time = IflyTime::Now_ms();
#endif

  lpl_path_.SamplePath(request_.sample_ds);

#if LOG_TIME_PROFILE
  lpl_interpolate_time_ms_ += (IflyTime::Now_ms() - lpl_interpolate_start_time);
  const double set_curve_path_start_time = IflyTime::Now_ms();
#endif

  CurvePath& path = curve_node_to_goal->GetMutableCurvePath();
  path.path_dist = lpl_path_.total_length;
  path.segment_size = lpl_path_.seg_num;
  path.ptss.resize(lpl_path_.seg_num);
  path.gear_change_number = lpl_path_.gear_change_num;
  path.kappa_change = lpl_path_.kappa_change;
  path.last_path_kappa_change = 0.0f;
  path.gear_number = 1;
  path.single_gear_lengths[path.gear_number - 1] = lpl_path_.lengths[0];
  for (int i = 0; i < lpl_path_.seg_num; ++i) {
    const auto& pt_vec = lpl_path_.ptss[i];
    path.dists[i] = lpl_path_.lengths[i];
    path.kappas[i] = lpl_path_.kappas[i];
    path.gears[i] = lpl_path_.gears[i];
    path.steers[i] = lpl_path_.steers[i];
    path.point_sizes[i] = lpl_path_.ptss[i].size();
    path.ptss[i] = std::move(lpl_path_.ptss[i]);
    if (i > 0) {
      if (IsGearDifferent(path.gears[i], path.gears[i - 1])) {
        path.single_gear_lengths[(++path.gear_number) - 1] = path.dists[i];
        path.last_path_kappa_change = 0.0f;
      } else {
        path.single_gear_lengths[path.gear_number - 1] += path.dists[i];
        path.last_path_kappa_change +=
            std::fabs(path.kappas[i] - path.kappas[i - 1]);
      }
    }
  }

  curve_node_to_goal->Set(search_map_boundary_, config_);

#if LOG_TIME_PROFILE
  set_curve_path_time_ += (IflyTime::Now_ms() - set_curve_path_start_time);
#endif

  return true;
}

const float HybridAStarPathGenerator::CalcCurveNodeGCostToParentNode(
    Node3d* current_node, CurveNode* curve_node) {
  // evaluate cost on the trajectory and add current cost
  float length_cost = 0.0f, gear_change_cost = 0.0f, kappa_change_cost = 0.0f;
  const CurvePath& path = curve_node->GetCurvePath();

  if (path.segment_size == 0) {
    return kMaxNodeCost;
  }

  const bool curve_search_node_gear_change =
      current_node->IsPathGearChange(path.gears[0]);

  const float curve_node_kappa_change =
      curve_search_node_gear_change
          ? path.kappa_change
          : path.kappa_change +
                std::fabs(current_node->GetKappa() - path.kappas[0]);

  const int curve_node_gear_change = curve_search_node_gear_change
                                         ? path.gear_change_number + 1
                                         : path.gear_change_number;

  length_cost = path.path_dist * config_.traj_forward_penalty;
  gear_change_cost = curve_node_gear_change * config_.gear_switch_penalty;
  kappa_change_cost =
      curve_node_kappa_change * config_.traj_kappa_change_penalty;

  curve_node->SetPre(current_node);
  curve_node->SetPathType(AstarPathType::NODE_CURVE);
  if (request_.search_mode == SearchMode::FORMAL) {
    curve_node->SetGearType(path.gears[0]);
    curve_node->SetKappa(path.kappas[0]);
    curve_node->SetGearSwitchNum(current_node->GetGearSwitchNum() +
                                 curve_node_gear_change);
    curve_node->SetDistToStart(path.path_dist + current_node->GetDistToStart());

    if (curve_node->GetGearSwitchNum() == 0) {
      curve_node->SetGearSwitchNode(nullptr);
      curve_node->SetNextGearSwitchNode(nullptr);
    } else if (curve_node->GetGearSwitchNum() == 1) {
      if (current_node->GetGearSwitchNum() == 0) {
        curve_node->SetGearSwitchNode(current_node);
      } else {
        curve_node->SetGearSwitchNode(current_node->GearSwitchNode());
      }
      curve_node->SetNextGearSwitchNode(nullptr);
    } else {
      if (current_node->GetGearSwitchNum() == 0) {
        curve_node->SetGearSwitchNode(current_node);
        curve_node->SetNextGearSwitchNode(current_node);
      } else if (current_node->GetGearSwitchNum() == 1) {
        curve_node->SetGearSwitchNode(current_node->GearSwitchNode());
        curve_node->SetNextGearSwitchNode(current_node);
      } else {
        curve_node->SetGearSwitchNode(current_node->GearSwitchNode());
        curve_node->SetNextGearSwitchNode(current_node->NextGearSwitchNode());
      }
    }

    // seg_num > 0
    double seg_length[MAX_CURVE_PATH_SEG_NUM];
    int seg_num = 0, first_gear_switch_index = 0, second_gear_switch_index = 0;
    seg_length[seg_num] = path.dists[0];
    for (int i = 1; i < path.segment_size; i++) {
      if (path.gears[i] == path.gears[i - 1]) {
        seg_length[seg_num] += path.dists[i];
      } else {
        seg_length[++seg_num] = path.dists[i];
        if (seg_num == 1) {
          first_gear_switch_index = i;
        } else if (seg_num == 2) {
          second_gear_switch_index = i;
        }
      }
    }

    Node3d* gear_switch_node = curve_node->GearSwitchNode();
    Node3d* next_gear_switch_node = curve_node->NextGearSwitchNode();

    AstarPathGear cur_gear;
    float cur_kappa, cur_gear_length;

    common_math::PathPt<float> gear_switch_pose;
    common_math::PathPt<float> next_gear_switch_pose;

    if (gear_switch_node != nullptr) {
      // gear switch num > 0
      if (gear_switch_node->GetPathType() == AstarPathType::START_NODE) {
        // the complete path is only curve path
        cur_gear = path.gears[0];
        cur_kappa = path.kappas[0];
        cur_gear_length = seg_length[0];
        gear_switch_pose = path.ptss[first_gear_switch_index][0];
      } else {
        // the complete path is node + curve path
        cur_gear = gear_switch_node->GetGearType();
        if (current_node->GetGearSwitchNum() > 0 ||
            curve_search_node_gear_change) {
          cur_gear_length = gear_switch_node->GetDistToStart();
          gear_switch_pose.SetX(gear_switch_node->GetX());
          gear_switch_pose.SetY(gear_switch_node->GetY());
          gear_switch_pose.SetTheta(gear_switch_node->GetPhi());
        } else {
          cur_gear_length = gear_switch_node->GetDistToStart() + seg_length[0];
          gear_switch_pose = path.ptss[first_gear_switch_index][0];
        }
        while (gear_switch_node != nullptr &&
               gear_switch_node->GetPathType() != AstarPathType::START_NODE) {
          cur_kappa = gear_switch_node->GetKappa();
          gear_switch_node = gear_switch_node->GetPreNode();
        }
      }
    }

    if (next_gear_switch_node != nullptr) {
      // gear switch num > 1
      if (next_gear_switch_node->GetPathType() == AstarPathType::START_NODE) {
        // the complete path is only curve path
        next_gear_switch_pose = path.ptss[second_gear_switch_index][0];
      } else {
        // the complete path is node + curve path
        if (current_node->GetGearSwitchNum() == 0) {
          if (curve_search_node_gear_change) {
            next_gear_switch_pose = path.ptss[first_gear_switch_index][0];
          } else {
            next_gear_switch_pose = path.ptss[second_gear_switch_index][0];
          }
        } else if (current_node->GetGearSwitchNum() == 1) {
          if (curve_search_node_gear_change) {
            next_gear_switch_pose = path.ptss[0][0];
          } else {
            next_gear_switch_pose = path.ptss[first_gear_switch_index][0];
          }
        } else {
          next_gear_switch_pose.SetX(next_gear_switch_node->GetX());
          next_gear_switch_pose.SetY(next_gear_switch_node->GetY());
          next_gear_switch_pose.SetTheta(next_gear_switch_node->GetPhi());
        }
      }
    }

    float last_path_kappa_change = path.last_path_kappa_change;
    if (path.gear_change_number == 0) {
      if (!curve_search_node_gear_change) {
        last_path_kappa_change +=
            std::fabs(path.kappas[0] - current_node->GetKappa());
        auto pre_node = current_node;
        while (pre_node->GetPreNode() != nullptr) {
          auto pre_pre_node = pre_node->GetPreNode();
          if (IsGearDifferent(pre_pre_node->GetGearType(),
                              pre_node->GetGearType())) {
            break;
          }
          if (pre_node->GetPhi() * pre_pre_node->GetPhi() < -1e-6f) {
            last_path_kappa_change += max_kappa_change_;
          }
          last_path_kappa_change +=
              std::fabs(pre_pre_node->GetKappa() - pre_node->GetKappa());
          pre_node = pre_pre_node;
        }
      }

      for (size_t i = 1; i < path.segment_size; ++i) {
        if (path.ptss[i].back().GetTheta() *
                path.ptss[i - 1].front().GetTheta() <
            -1e-6f) {
          last_path_kappa_change += max_kappa_change_;
        }
      }
    }

    curve_node->SetCurGear(cur_gear);
    curve_node->SetCurKappa(cur_kappa);
    curve_node->SetCurGearLength(cur_gear_length);
    curve_node->SetGearSwitchPose(gear_switch_pose);
    curve_node->SetNextGearSwitchPose(next_gear_switch_pose);
    curve_node->SetLatErr(std::fabs(path.ptss.back().back().GetY()));
    curve_node->SetThetaErr(std::fabs(path.ptss.back().back().GetTheta()));
    curve_node->SetLastPathKappaChange(last_path_kappa_change);
  }

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

  if (request_.search_mode == SearchMode::FORMAL && !cal_h_cost) {
    if (request_.scenario_type ==
        ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN) {
      end_pose.x += 1.68;
    } else if (request_.scenario_type ==
               ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
      end_pose.x += 2.86;
    }
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
    return false;
  }

  if (request_.search_mode == SearchMode::FORMAL && !cal_h_cost) {
    if (request_.scenario_type ==
            ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN ||
        request_.scenario_type ==
            ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
      RSPathSegment rs_seg;
      rs_seg.steer = RSPathSteer::RS_STRAIGHT;
      rs_seg.kappa = 0.0;
      rs_seg.size = 1;
      rs_seg.points[0].x = end_pose.x;
      rs_seg.points[0].y = end_pose.y;
      rs_seg.points[0].theta = end_pose.theta;
      rs_seg.points[0].kappa = 0.0;
      if (request_.scenario_type ==
          ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN) {
        rs_seg.gear = AstarPathGear::REVERSE;
        rs_seg.length = end_node_->GetX() - end_pose.GetX();
        rs_seg.points[0].dir = AstarPathGear::REVERSE;
        if (rs_path_.paths[rs_path_.size - 1].gear == AstarPathGear::DRIVE) {
          rs_path_.gear_change_number += 1;
        }
      } else if (request_.scenario_type ==
                 ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
        rs_seg.gear = AstarPathGear::DRIVE;
        rs_seg.length = end_pose.GetX() - end_node_->GetX();
        rs_seg.points[0].dir = AstarPathGear::DRIVE;
        if (rs_path_.paths[rs_path_.size - 1].gear == AstarPathGear::REVERSE) {
          rs_path_.gear_change_number += 1;
        }
      }
      rs_path_.total_length += std::fabs(rs_seg.length);
      rs_path_.paths[rs_path_.size++] = rs_seg;
    }
  }

  if (request_.search_mode == SearchMode::DECIDE_CUL_DE_SAC &&
      request_.scenario_type ==
          ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN) {
    if (rs_path_.GetFirstGear() != AstarPathGear::DRIVE ||
        rs_path_.gear_change_number > 1) {
      return false;
    }
  }

  return true;
}

const bool HybridAStarPathGenerator::CalcLPLPathToGoal(
    Node3d* current_node, const link_pt_line::LinkPtLineInput<float>& input,
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
  if (!CalcRSPathToGoal(next_node, false, false, RSPathRequestType::NONE,
                        min_radius_, true)) {
    return 100.0f;
  }

  return rs_path_.total_length * config_.traj_forward_penalty;
}

const float HybridAStarPathGenerator::GenerateHeuristicCostByLPLPath(
    Node3d* next_node, const link_pt_line::LinkPtLineInput<float>& input,
    NodeHeuristicCost* cost) {
  if (!CalcLPLPathToGoal(next_node, input, true)) {
    return 100.0f;
  }

  return std::min(float(lpl_path_.total_length), 36.8f) *
         config_.traj_forward_penalty;
}

void HybridAStarPathGenerator::InitNodePool() { node_pool_.Init(); }

void HybridAStarPathGenerator::ResetNodePool() { node_pool_.Clear(); }

void HybridAStarPathGenerator::ResetSearchState() {
  ResetNodePool();
  open_pq_.clear();
  node_set_.clear();
  node_set_.reserve(NODE_POOL_MAX_NUM);
  result_.Clear();
  child_node_debug_.clear();
  queue_path_debug_.clear();
  delete_queue_path_debug_.clear();
  all_success_curve_path_debug_.clear();
  all_success_path_first_gear_switch_pose_debug_.clear();
  all_search_node_list_.clear();
  all_curve_node_list_.clear();
  collision_check_time_ms_ = 0.0;
  rs_try_num_ = 0;
  rs_interpolate_time_ms_ = 0.0;
  rs_time_ms_ = 0.0;
  lpl_try_num_ = 0;
  lpl_interpolate_time_ms_ = 0.0;
  lpl_time_ms_ = 0.0;
  set_curve_path_time_ = 0.0;
  check_node_should_del_time_ = 0.0;
  generate_next_node_time_ = 0.0;
}

void HybridAStarPathGenerator::InitSearchPhaseTimeCost() {
  ILOG_INFO << "hybrid astar " << GetScenarioPrefix() << " update start";
  search_phase_time_cost_ = SearchPhaseTimeCost();
}

void HybridAStarPathGenerator::SyncSearchPhaseTimeCostToResult() {
  result_.decide_cul_de_sac_consume_time_ms =
      search_phase_time_cost_.decide_cul_de_sac_consume_time_ms;
  result_.pre_search_consume_time_ms =
      search_phase_time_cost_.pre_search_consume_time_ms;
  result_.formal_search_consume_time_ms =
      search_phase_time_cost_.formal_search_consume_time_ms;
  result_.search_consume_time_ms = result_.decide_cul_de_sac_consume_time_ms +
                                   result_.pre_search_consume_time_ms +
                                   result_.formal_search_consume_time_ms;
}

const bool HybridAStarPathGenerator::FinalizeUpdate(const bool search_success) {
  SyncSearchPhaseTimeCostToResult();
  result_.search_node_num = node_set_.size();
  LogUpdateSummary();
  return search_success;
}

void HybridAStarPathGenerator::LogUpdateSummary() const {
  ILOG_INFO << "hybrid astar " << GetScenarioPrefix() << " update end"
            << ", total time(ms): " << result_.search_consume_time_ms
            << ", decide cul-de-sac time(ms): "
            << result_.decide_cul_de_sac_consume_time_ms
            << ", pre search time(ms): " << result_.pre_search_consume_time_ms
            << ", formal search time(ms): "
            << result_.formal_search_consume_time_ms
            << ", node num: " << result_.search_node_num
            << ", solve number: " << result_.solve_number;
}

const bool HybridAStarPathGenerator::RunFormalSearch(
    const SearchConfigSnapshot& snapshot) {
  const double formal_search_start_time = IflyTime::Now_ms();
  const bool formal_search_success =
      UpdateOnce(BuildFormalSearchPathColDetBuffer());
  search_phase_time_cost_.formal_search_consume_time_ms =
      IflyTime::Now_ms() - formal_search_start_time;
  return formal_search_success;
}

PathColDetBuffer HybridAStarPathGenerator::BuildFormalSearchPathColDetBuffer()
    const {
  const ParkingLatLonPathBuffer& lat_lon_path_buffer =
      apa_param.GetParam().lat_lon_path_buffer;

  PathColDetBuffer path_col_det_buffer;
  path_col_det_buffer.need_distinguish_outinslot = true;
  path_col_det_buffer.lon_buffer = lat_lon_path_buffer.lon_buffer;
  path_col_det_buffer.out_slot_body_lat_buffer =
      lat_lon_path_buffer.out_slot_body_lat_buffer;
  path_col_det_buffer.out_slot_mirror_lat_buffer =
      lat_lon_path_buffer.out_slot_mirror_lat_buffer;
  path_col_det_buffer.out_slot_extra_turn_lat_buffer =
      lat_lon_path_buffer.out_slot_extra_turn_lat_buffer;
  path_col_det_buffer.entrance_slot_body_lat_buffer =
      lat_lon_path_buffer.entrance_slot_body_lat_buffer;
  path_col_det_buffer.entrance_slot_mirror_lat_buffer =
      lat_lon_path_buffer.entrance_slot_mirror_lat_buffer;
  path_col_det_buffer.entrance_slot_extra_turn_lat_buffer =
      lat_lon_path_buffer.entrance_slot_extra_turn_lat_buffer;
  path_col_det_buffer.in_slot_body_lat_buffer =
      lat_lon_path_buffer.in_slot_body_lat_buffer;
  path_col_det_buffer.in_slot_mirror_lat_buffer =
      lat_lon_path_buffer.in_slot_mirror_lat_buffer;
  path_col_det_buffer.in_slot_extra_turn_lat_buffer =
      lat_lon_path_buffer.in_slot_extra_turn_lat_buffer;
  return path_col_det_buffer;
}

const std::string HybridAStarPathGenerator::GetScenarioPrefix() const {
  switch (request_.scenario_type) {
    case ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN:
      return "perpendicular tail in";
    case ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN:
      return "perpendicular head in";
    default:
      return "";
  }
}

const geometry_lib::PathPoint&
HybridAStarPathGenerator::GetStartPoseForCurrentSearch() const {
  if (request_.swap_start_goal) {
    return request_.ego_info_under_slot.target_pose;
  }
  return request_.ego_info_under_slot.cur_pose;
}

const geometry_lib::PathPoint&
HybridAStarPathGenerator::GetEndPoseForCurrentSearch() const {
  if (request_.swap_start_goal) {
    return request_.ego_info_under_slot.cur_pose;
  }
  return request_.ego_info_under_slot.target_pose;
}

const bool HybridAStarPathGenerator::InitStartAndEndNodes() {
  start_node_ = node_pool_.AllocateNode();
  if (start_node_ == nullptr) {
    ILOG_ERROR << "start node nullptr";
    result_.fail_type = AstarFailType::ALLOCATE_NODE_FAIL;
    return false;
  }

  const auto& start_pose = GetStartPoseForCurrentSearch();
  start_node_->Set(
      NodePath(start_pose.pos.x(), start_pose.pos.y(), start_pose.heading),
      search_map_boundary_, config_, 0.0);
  start_node_->SetPathType(AstarPathType::START_NODE);
  start_node_->DebugString();

  end_node_ = node_pool_.AllocateNode();
  if (end_node_ == nullptr) {
    ILOG_ERROR << "end node nullptr";
    result_.fail_type = AstarFailType::ALLOCATE_NODE_FAIL;
    return false;
  }

  const auto& end_pose = GetEndPoseForCurrentSearch();
  end_node_->Set(NodePath(end_pose.pos.x(), end_pose.pos.y(), end_pose.heading),
                 search_map_boundary_, config_, 0.0);
  end_node_->SetPathType(AstarPathType::END_NODE);
  end_node_->DebugString();

  return true;
}

void HybridAStarPathGenerator::GenerateDpMapForCurrentSearch() {
  const double dp_start_time = IflyTime::Now_ms();
  const auto& end_pose = GetEndPoseForCurrentSearch();
  grid_search_.GenerateDpMap(end_pose.GetX(), end_pose.GetY(),
                             search_map_boundary_);
  ILOG_INFO << "generate dp map consume time = "
            << IflyTime::Now_ms() - dp_start_time << "  ms";
}

const NodeDeleteInput HybridAStarPathGenerator::BuildNodeDeleteInput(
    const PathColDetBuffer& path_col_det_buffer) const {
  NodeDeleteInput node_del_input;
  node_del_input.ego_info_under_slot = request_.ego_info_under_slot;
  node_del_input.map_bound = search_map_boundary_;
  node_del_input.config = config_;
  node_del_input.map_grid_bound = search_map_grid_boundary_;
  node_del_input.scenario_type = request_.scenario_type;
  node_del_input.start_id = start_node_->GetGlobalID();
  node_del_input.max_gear_shift_number = request_.max_gear_shift_number;
  node_del_input.path_col_det_buffer = path_col_det_buffer;
  node_del_input.sample_ds = request_.sample_ds;
  node_del_input.swap_start_goal = request_.swap_start_goal;
  if (request_.search_mode == SearchMode::FORMAL) {
    node_del_input.need_cal_obs_dist = true;
    node_del_input.enable_smart_fold_mirror = request_.enable_smart_fold_mirror;
  } else {
    node_del_input.need_cal_obs_dist = false;
    node_del_input.enable_smart_fold_mirror = false;
  }
  return node_del_input;
}

const bool HybridAStarPathGenerator::ValidateStartAndEndNodes() {
  NodeDeleteRequest node_del_request;

  node_del_request.current_node = start_node_;
  if (CheckNodeShouldDelete(node_del_request)) {
    ILOG_ERROR << "start node is deleted, reason = "
               << GetNodeDeleteReasonString(
                      node_delete_decider_.GetNodeDeleteReason());
    result_.fail_type = AstarFailType::START_COLLISION;
    return false;
  }

  node_del_request.current_node = end_node_;
  if (CheckNodeShouldDelete(node_del_request)) {
    ILOG_ERROR << "end node is deleted, reason = "
               << GetNodeDeleteReasonString(
                      node_delete_decider_.GetNodeDeleteReason());
    result_.fail_type = AstarFailType::GOAL_COLLISION;
    return false;
  }

  return true;
}

void HybridAStarPathGenerator::PopulateChildNodeState(
    Node3d* new_node, Node3d* current_node, const CarMotion& car_motion) const {
  new_node->SetPathType(AstarPathType::NODE_SEARCHING);
  new_node->SetGearType(car_motion.gear);
  new_node->SetKappa(car_motion.kappa);
  new_node->SetPre(current_node);
  new_node->SetDistToStart(new_node->GetNodePathDistance() +
                           current_node->GetDistToStart());

  if (current_node->IsPathGearChange(car_motion.gear)) {
    new_node->SetGearSwitchNum(current_node->GetGearSwitchNum() + 1);
    if (new_node->GetGearSwitchNum() == 1) {
      new_node->SetGearSwitchNode(current_node);
      new_node->SetNextGearSwitchNode(nullptr);
    } else if (new_node->GetGearSwitchNum() == 2) {
      new_node->SetGearSwitchNode(current_node->GearSwitchNode());
      new_node->SetNextGearSwitchNode(current_node);
    } else {
      new_node->SetGearSwitchNode(current_node->GearSwitchNode());
      new_node->SetNextGearSwitchNode(current_node->NextGearSwitchNode());
    }
    new_node->SetScurveNum(current_node->GetScurveNum());
    new_node->SetSingleGearLength(new_node->GetNodePathDistance());
    new_node->SetSingleGearMinLength(
        std::min(new_node->GetSingleGearLength(),
                 current_node->GetSingleGearMinLength()));
  } else {
    new_node->SetGearSwitchNum(current_node->GetGearSwitchNum());
    new_node->SetGearSwitchNode(current_node->GearSwitchNode());
    new_node->SetNextGearSwitchNode(current_node->NextGearSwitchNode());
    if (IsSteerOpposite(current_node->GetKappa(), new_node->GetKappa())) {
      new_node->SetScurveNum(current_node->GetScurveNum() + 1);
    } else {
      new_node->SetScurveNum(current_node->GetScurveNum());
    }
    new_node->SetSingleGearLength(new_node->GetNodePathDistance() +
                                  current_node->GetSingleGearLength());
    if (new_node->GetGearSwitchNum() == 0) {
      new_node->SetSingleGearMinLength(new_node->GetSingleGearLength());
    } else {
      new_node->SetSingleGearMinLength(
          std::min(new_node->GetSingleGearLength(),
                   current_node->GetSingleGearMinLength()));
    }
  }
}

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
    const common_math::PathPt<float>& gear_switch_pose, AstarPathGear gear,
    const float gear_switch_penalty, const float length_penalty) {
  return 0.0f;
}

const bool HybridAStarPathGenerator::BackwardPassByCurveNode(
    const CurveNode* curve_node_to_goal) {
  const CurvePath& path = curve_node_to_goal->GetCurvePath();
  if (path.segment_size < 1) {
    ILOG_ERROR << "there are no path";
    return false;
  }

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
    result_.length_vec.emplace_back(std::max(
        config_.node_path_dist_resolution * (result_.x_vec_vec[i].size() - 1),
        config_.node_path_dist_resolution));
  }

  const float cur_gear_last_pt_kappa = result_.kappa_vec_vec.front().back();
  if (cur_gear_last_pt_kappa > 1e-5f) {
    result_.cur_steer = AstarPathSteer::LEFT;
  } else if (cur_gear_last_pt_kappa < -1e-5f) {
    result_.cur_steer = AstarPathSteer::RIGHT;
  } else {
    result_.cur_steer = AstarPathSteer::NONE;
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
    ILOG_INFO << "segment " << i << "  dist = " << path.dists[i]
              << "  gear = " << static_cast<int>(path.gears[i])
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
  std::vector<geometry_lib::PathPoint> poses(
      all_success_path_first_gear_switch_pose_debug_.size());
  for (size_t i = 0; i < all_success_path_first_gear_switch_pose_debug_.size();
       i++) {
    const auto& pose = all_success_path_first_gear_switch_pose_debug_[i];
    poses[i].SetPos(pose.GetX(), pose.GetY());
    poses[i].SetTheta(pose.GetTheta());
  }
  return poses;
}

const cdl::AABB& HybridAStarPathGenerator::GetIntersetingAreaForDebug() {
  return interesting_area_;
}

CompactNodePool HybridAStarPathGenerator::node_pool_;
GridSearch HybridAStarPathGenerator::grid_search_;

}  // namespace apa_planner
}  // namespace planning