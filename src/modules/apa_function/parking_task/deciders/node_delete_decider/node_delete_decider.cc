#include "node_delete_decider.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

#include "aabb2d.h"
#include "apa_context.h"
#include "apa_param_config.h"
#include "apa_slot_manager.h"
#include "base_collision_detector.h"
#include "common_math.h"
#include "common_platform_type_soc.h"
#include "curve_node.h"
#include "geometry_math.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_request.h"
#include "log_glog.h"
#include "pose2d.h"

namespace planning {
namespace apa_planner {

static bool g_debug = false;

void NodeDeleteDecider::Process(const NodeDeleteInput input) {
  input_ = input;
  const EgoInfoUnderSlot& ego_info_under_slot = input.ego_info_under_slot;
  const ApaSlot& slot = ego_info_under_slot.slot;
  search_pts_.reserve(7);
  curve_ptss_.reserve(MAX_CURVE_PATH_SEG_NUM);
  curve_gears_.reserve(MAX_CURVE_PATH_SEG_NUM);
  grade_segment_pts_.reserve(100);

  if (input.scenario_type ==
          ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN ||
      input.scenario_type ==
          ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
    const float bound = ego_info_under_slot.slot_type == SlotType::SLANT
                            ? SLANT_SLOT_EXTEND_BOUND
                            : 0.0f;

    const double rearaxle_frontoverhang_length =
        apa_param.GetParam().car_length -
        apa_param.GetParam().rear_overhanging - 0.1;

    const geometry_lib::RectangleBound& ogm_bound =
        col_det_interface_ptr_->GetEDTColDetPtr()->GetOgmBound();

    pose_bound_.y_down_bound =
        std::max(input.map_bound.y_min,
                 float(ogm_bound.min_y + rearaxle_frontoverhang_length));
    pose_bound_.y_up_bound =
        std::min(input.map_bound.y_max,
                 float(ogm_bound.max_y - rearaxle_frontoverhang_length));
    pose_bound_.x_up_bound =
        std::min(input.map_bound.x_max,
                 float(ogm_bound.max_x - rearaxle_frontoverhang_length));

    double min_x = 2.68, min_curve_x = 2.68;
    if (input.scenario_type ==
        ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
      min_x += apa_param.GetParam().wheel_base;
      min_curve_x += apa_param.GetParam().wheel_base;
    }

    pose_bound_.x_down_bound =
        std::min({ego_info_under_slot.cur_pose.pos.x() - 0.4,
                  ego_info_under_slot.target_pose.pos.x() + 0.4, min_x}) -
        bound;

    if (input.swap_start_goal) {
      pose_bound_.x_down_bound =
          std::min({ego_info_under_slot.cur_pose.pos.x() - 0.4,
                    ego_info_under_slot.target_pose.pos.x() - 0.1, min_x}) -
          bound;
    }

    pose_bound_.curve_x_down_bound =
        std::min({ego_info_under_slot.cur_pose.pos.x() - 0.1,
                  ego_info_under_slot.target_pose.pos.x() - 0.02,
                  min_curve_x}) -
        bound;

    pose_bound_.heading_down_bound = -1e-6;

    const double heading_check_bound = ifly_deg2rad(150.0);
    const double heading_buffer = ifly_deg2rad(20.0);

    pose_bound_.heading_up_bound = std::max(
        heading_check_bound,
        std::fabs(ego_info_under_slot.cur_pose.heading) + heading_buffer);

    pose_bound_.heading_up_bound =
        std::min(pose_bound_.heading_up_bound, M_PIf32);

    pose_bound_.heading_up_bound =
        std::fabs(geometry_lib::NormalizeAngle(pose_bound_.heading_up_bound));

    if (input.scenario_type ==
        ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
      pose_bound_.heading_up_bound = M_PIf32 + 1e-5f;
      if (ego_info_under_slot.cur_pose.heading > heading_buffer + 1e-5f) {
        pose_bound_.heading_down_bound =
            ego_info_under_slot.cur_pose.heading - heading_buffer;
      } else if (ego_info_under_slot.cur_pose.heading <
                 -heading_buffer - 1e-5f) {
        pose_bound_.heading_down_bound =
            std::fabs(ego_info_under_slot.cur_pose.heading + heading_buffer);
      }
    }

    const float slot_x =
        ego_info_under_slot.slot.origin_corner_coord_local_.pt_01_mid.x();

    const float half_slot_width = slot.slot_width_ * 0.5 + 0.068;

    slot_box_ = cdl::AABB2f(
        Eigen::Vector2f(0.0f, -half_slot_width),
        Eigen::Vector2f(slot_x + 1.25f + +apa_param.GetParam().wheel_base,
                        half_slot_width));

    slot_entrance_box_ = cdl::AABB2f(
        Eigen::Vector2f(slot_x - 0.68f, -half_slot_width),
        Eigen::Vector2f(slot_x + 1.25f + apa_param.GetParam().wheel_base,
                        half_slot_width));
  }

  slot_box_.DebugString();
  slot_entrance_box_.DebugString();

  ILOG_INFO << "pose_bound x_up_bound = " << pose_bound_.x_up_bound
            << " x_down_bound = " << pose_bound_.x_down_bound
            << " curve_x_down_bound = " << pose_bound_.curve_x_down_bound
            << " y_up_bound = " << pose_bound_.y_up_bound
            << " y_down_bound = " << pose_bound_.y_down_bound
            << " heading_up_bound = " << pose_bound_.heading_up_bound * kRad2Deg
            << " heading_down_bound = "
            << pose_bound_.heading_down_bound * kRad2Deg;
}

const bool NodeDeleteDecider::CheckShouldBeDeleted(NodeDeleteRequest request) {
  request_ = request;
  reason_ = NodeDeleteReason::UNKNOWN;

  // current_node and curve_node should not be both nullptr, but one of them
  // should be nullptr.
  if (request.current_node == nullptr && request.curve_node == nullptr) {
    return true;
  }

  if (request.current_node != nullptr && request.curve_node != nullptr) {
    return true;
  }

  if (request.current_node != nullptr &&
      request.current_node->GetNodePath().point_size < 1) {
    return true;
  }

  if (request.curve_node != nullptr &&
      request.curve_node->GetCurvePath().segment_size < 1) {
    return true;
  }

  if (input_.scenario_type ==
          ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN ||
      input_.scenario_type ==
          ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
    return CheckShouldBeDeletedForPerpendicularIn();
  }

  return true;
}

const bool NodeDeleteDecider::CheckShouldBeDeletedForPerpendicularIn() {
  if (request_.old_node == nullptr) {
    if (CheckUnexpectedGear()) {
      reason_ = NodeDeleteReason::UNEXPECTED_GEAR;
      return true;
    }

    if (CheckExceedGearShiftNumber()) {
      reason_ = NodeDeleteReason::EXCEED_GEAR_SHIFT_NUMBER;
      return true;
    }

    if (CheckExceedScurveNum()) {
      reason_ = NodeDeleteReason::EXCEED_SCURVE_NUMBER;
      return true;
    }

    if (CheckDelByStartNode()) {
      reason_ = NodeDeleteReason::START_NODE;
      return true;
    }

    if (CheckZigzagPath()) {
      reason_ = NodeDeleteReason::ZIGZAG_PATH;
      return true;
    }

    if (CheckBacktrackPath()) {
      reason_ = NodeDeleteReason::BACKTRACK_PATH;
      return true;
    }

    if (CheckDelByParentNode()) {
      reason_ = NodeDeleteReason::PARENT_NODE;
      return true;
    }

    if (CheckOutOfGridBound()) {
      reason_ = NodeDeleteReason::OUT_OF_GRID_BOUND;
      return true;
    }

    if (CheckOutOfPoseBound()) {
      reason_ = NodeDeleteReason::OUT_OF_POSE_BOUND;
      return true;
    }

    if (CheckOutOfPoseStandard()) {
      reason_ = NodeDeleteReason::OUT_OF_POSE_STANDARD;
      return true;
    }

    if (CheckCollision()) {
      reason_ = NodeDeleteReason::COLLISION;
      return true;
    }

  } else {
    if (CheckDelBySameGridNodeContinuous()) {
      reason_ = NodeDeleteReason::SAME_GRID_NODE_CONTINUOUS;
      return true;
    }

    if (CheckDelByLoopBackNode()) {
      reason_ = NodeDeleteReason::LOOP_BACK_NODE;
      return true;
    }
  }

  return false;
}

const bool NodeDeleteDecider::UpdateObsDistRelativeSlot(
    NodeDeleteRequest request) {
  request_ = request;
  input_.need_cal_obs_dist = true;
  return !CheckCollision();
}

const bool NodeDeleteDecider::CheckCollision() {
  if (col_det_interface_ptr_->GetObsManagerPtr()->GetObstacles().empty()) {
    // ILOG_INFO << "OBS IS EMPTY";
    return false;
  }

  search_pts_.clear();
  curve_ptss_.clear();
  curve_gears_.clear();

  const float ds = input_.sample_ds;

  if (request_.current_node != nullptr) {
    Node3d* current_node = request_.current_node;

    // do col det for every pt in node
    const NodePath& path = current_node->GetNodePath();

    // The first {x, y, phi} is collision free unless they are start and end
    // configuration of search problem
    const size_t check_start_index = path.point_size == 1 ? 0 : 1;
    float s = check_start_index * ds;

    common_math::PathPt<float> pt;
    pt.kappa = current_node->GetKappa();
    pt.gear = current_node->GetGearType();
    for (size_t i = check_start_index; i < path.point_size; ++i) {
      pt.SetPose(path.points[i].x, path.points[i].y, path.points[i].theta);
      pt.s = s;
      s += ds;
      search_pts_.emplace_back(pt);
    }

    double safe_remain_dist = 0.0;
    ObsToPathDistRelativeSlot obs_dist;

    const ParkingLatLonSpeedBuffer& speed_buffer =
        apa_param.GetParam().lat_lon_speed_buffer;

    const bool is_special_node =
        speed_buffer.enable_leave_initial_place &&
        (current_node->GetDistToStart() <
             speed_buffer.leave_initial_place_dist ||
         current_node->GetPathType() == AstarPathType::START_NODE ||
         current_node->GetPathType() == AstarPathType::END_NODE);

    const bool col_flag = CheckPtsCollision(search_pts_, is_special_node,
                                            &obs_dist, &safe_remain_dist);

    if (input_.need_cal_obs_dist) {
      current_node->SetObsDistRelativeSlot(obs_dist);
    }
    const size_t min_save_pt_number = 2;
    if (col_flag && safe_remain_dist > min_save_pt_number * input_.sample_ds &&
        safe_remain_dist < path.path_dist) {
      // i must be bigger than 1
      size_t i = min_save_pt_number;
      for (; i < path.point_size; ++i) {
        if (i * input_.sample_ds > safe_remain_dist) {
          break;
        }
      }
      NodePath trimmed_path = path;
      trimmed_path.point_size = i;

      current_node->UpdatePath(trimmed_path, input_.map_bound, input_.config,
                               (i - 1) * ds);

      return false;
    }

    return col_flag;
  }

  else {
    CurveNode* curve_node = request_.curve_node;
    // do col det for every pt in curve
    const CurvePath& path = curve_node->GetCurvePath();
    // The first {x, y, phi} is collision free unless they are start and end
    float s = 0.0f;
    for (size_t i = 0; i < path.segment_size; ++i) {
      const auto& src_pts = path.ptss[i];
      if (src_pts.size() <= 1) {
        continue;
      }

      if (curve_ptss_.empty() ||
          IsGearDifferent(curve_gears_.back(), path.gears[i])) {
        curve_ptss_.emplace_back();
        curve_gears_.emplace_back(path.gears[i]);
        curve_ptss_.back().reserve(src_pts.size() - 1);
      } else {
        curve_ptss_.back().reserve(curve_ptss_.back().size() + src_pts.size() -
                                   1);
      }

      auto& dst_pts = curve_ptss_.back();
      for (size_t j = 1; j < src_pts.size(); ++j) {
        common_math::PathPt<float> pt = src_pts[j];
        s += ds;
        pt.s = s;
        dst_pts.emplace_back(pt);
      }
    }

    bool enable_smart_fold_mirror = false;
    while (input_.enable_smart_fold_mirror &&
           (input_.scenario_type ==
                ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN ||
            input_.scenario_type ==
                ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN)) {
      if (curve_ptss_.empty() || curve_ptss_.back().empty()) {
        break;
      }
      const AstarPathGear last_gear = curve_gears_.back();
      const auto& pts = curve_ptss_.back();

      const int last_path_pt_count = static_cast<int>(pts.size());

      int last_line_pt_index = last_path_pt_count;
      for (int i = last_path_pt_count - 1; i >= 0; --i) {
        if (std::fabs(pts[i].kappa) > 1e-3f) {
          break;
        }
        last_line_pt_index = i;
      }

      if (last_line_pt_index == last_path_pt_count) {
        // no lint pt found, no need to fold mirror
        break;
      }

      const float last_lint_pt_x = pts[last_line_pt_index].GetX();

      const ApaParameters& param = apa_param.GetParam();
      const float slot_x = input_.ego_info_under_slot.slot
                               .processed_corner_coord_local_.pt_01_mid.x();

      float mirror_x = last_lint_pt_x + param.lon_dist_mirror_to_rear_axle;

      if (input_.scenario_type ==
          ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
        mirror_x = last_lint_pt_x - param.lon_dist_mirror_to_rear_axle;
      }

      const float lower_x =
          slot_x + param.smart_fold_mirror_params.x_down_offset;

      const float upper_x = slot_x + param.smart_fold_mirror_params.x_up_offset;

      const float redundant_x = 0.1f;

      if (mirror_x < lower_x) {
        break;
      }

      // No need to insert in reverse, just insert directly, saving subsequent
      // reversals
      // lath_path_line_pts_.insert(lath_path_line_pts_.end(),
      //     std::make_reverse_iterator(pts.begin() + last_path_pt_count),
      //     std::make_reverse_iterator(pts.begin() + last_lint_pt_index));
      const bool exist_arc_pts = last_line_pt_index > 0;
      std::vector<common_math::PathPt<float>> last_pts =
          std::move(curve_ptss_.back());
      curve_ptss_.pop_back();
      curve_gears_.pop_back();

      const auto line_begin_it = last_pts.begin() + last_line_pt_index;
      const auto line_end_it = last_pts.end();

      if (exist_arc_pts) {
        curve_ptss_.emplace_back(last_pts.begin(), line_begin_it);
        curve_gears_.emplace_back(last_gear);
      }

      if (mirror_x < upper_x) {
        curve_ptss_.emplace_back(line_begin_it, line_end_it);
        curve_gears_.emplace_back(last_gear);
      } else {
        const float surplus_x = mirror_x - upper_x + redundant_x;
        const auto split_it =
            line_begin_it +
            std::clamp(static_cast<int>(surplus_x / ds), 0,
                       static_cast<int>(line_end_it - line_begin_it));
        curve_ptss_.emplace_back(line_begin_it, split_it);
        curve_ptss_.emplace_back(split_it, line_end_it);
        curve_gears_.emplace_back(last_gear);
        curve_gears_.emplace_back(last_gear);
      }

      enable_smart_fold_mirror = true;
      break;
    }

    ObsToPathDistRelativeSlot small_obs_dist;
    double safe_remain_dist = 0.0;
    for (size_t i = 0; i < curve_ptss_.size(); ++i) {
      ObsToPathDistRelativeSlot obs_dist;
      const bool use_fold_mirror_col_det =
          enable_smart_fold_mirror && i == curve_ptss_.size() - 1;
      if (use_fold_mirror_col_det) {
        col_det_interface_ptr_->Init(true);
      }
      const bool col_flag = CheckPtsCollision(curve_ptss_[i], false, &obs_dist,
                                              &safe_remain_dist);
      if (use_fold_mirror_col_det) {
        col_det_interface_ptr_->Init(false);
      }
      if (input_.need_cal_obs_dist) {
        small_obs_dist.SetSmallerDist(obs_dist);
      }

      if (col_flag) {
        if (input_.need_cal_obs_dist) {
          curve_node->SetObsDistRelativeSlot(small_obs_dist);
        }
        if (i == curve_ptss_.size() - 1 && curve_ptss_.back().size() > 0 &&
            safe_remain_dist > curve_ptss_.back().back().s - ds - 0.001f) {
          return false;
        }
        return true;
      }
    }
    if (input_.need_cal_obs_dist) {
      curve_node->SetObsDistRelativeSlot(small_obs_dist);
    }
    return false;
  }

  return false;
}

const bool NodeDeleteDecider::CheckOutOfPoseBound() {
  if (request_.current_node != nullptr) {
    if (request_.current_node->GetPathType() == AstarPathType::END_NODE) {
      return false;
    }
    const NodePath& path = request_.current_node->GetNodePath();
    for (size_t i = 0; i < path.point_size; ++i) {
      if (i > 0 && i < path.point_size - 1) {
        continue;
      }
      const Pose2f& pose = path.points[i];
      if (pose.x < pose_bound_.x_down_bound ||
          pose.x > pose_bound_.x_up_bound ||
          pose.y < pose_bound_.y_down_bound ||
          pose.y > pose_bound_.y_up_bound ||
          std::fabs(pose.theta) < pose_bound_.heading_down_bound ||
          std::fabs(pose.theta) > pose_bound_.heading_up_bound) {
        return true;
      }
    }
  } else {
    const CurvePath& curve_path = request_.curve_node->GetCurvePath();
    for (const auto& points : curve_path.ptss) {
      for (size_t i = 0; i < points.size(); ++i) {
        if (i > 0 && i < points.size() - 1) {
          continue;
        }
        const auto& pose = points[i];
        if (pose.GetX() < pose_bound_.curve_x_down_bound ||
            pose.GetX() > pose_bound_.x_up_bound ||
            pose.GetY() < pose_bound_.y_down_bound ||
            pose.GetY() > pose_bound_.y_up_bound ||
            std::fabs(pose.GetTheta()) < pose_bound_.heading_down_bound ||
            std::fabs(pose.GetTheta()) > pose_bound_.heading_up_bound) {
          return true;
        }
      }
    }
  }

  return false;
}

const bool NodeDeleteDecider::CheckOutOfPoseStandard() {
  if (request_.current_node == nullptr) {
    return false;
  }
  if (input_.swap_start_goal) {
    return false;
  }
  if (input_.scenario_type !=
          ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN &&
      input_.scenario_type !=
          ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN) {
    return false;
  }
  const NodePath& path = request_.current_node->GetNodePath();
  const float target_heading =
      input_.ego_info_under_slot.target_pose.GetTheta();
  const float x_err = 0.36f;
  const float heading_err = 30.0f * common_math::kDeg2RadF;
  const float y_err = 3.68f;
  const float target_y = input_.ego_info_under_slot.target_pose.GetY();
  for (size_t i = 0; i < path.point_size; ++i) {
    const Pose2f& pose = path.points[i];
    if (pose.GetX() - pose_bound_.x_down_bound < x_err &&
        std::fabs(pose.GetY() - target_y) < y_err &&
        std::fabs(common_math::UnifyAngleDiff(pose.GetPhi(), target_heading)) >
            heading_err) {
      return true;
    }
  }
  return false;
}

const bool NodeDeleteDecider::CheckOutOfGridBound() {
  if (request_.current_node == nullptr) {
    return false;
  }
  const NodeGridIndex id = request_.current_node->GetIndex();

  if (id.x < 0 || id.x >= input_.map_grid_bound.x || id.y < 0 ||
      id.y >= input_.map_grid_bound.y || id.phi < 0 ||
      id.phi >= input_.map_grid_bound.phi) {
    return true;
  }

  return false;
}

const bool NodeDeleteDecider::CheckZigzagPath() {
  const Node3d* parent_node = request_.parent_node;
  const Node3d* cur_node = request_.current_node;
  if (parent_node == nullptr || cur_node == nullptr) {
    return false;
  }
  const Node3d* grandpa_node = parent_node->GetPreNode();
  if (grandpa_node == nullptr) {
    return false;
  }

  if (IsGearDifferent(parent_node->GetGearType(), cur_node->GetGearType()) ||
      IsGearDifferent(grandpa_node->GetGearType(),
                      parent_node->GetGearType())) {
    return false;
  }

  if (IsSteerOpposite(parent_node->GetKappa(), cur_node->GetKappa()) &&
      IsSteerOpposite(grandpa_node->GetKappa(), parent_node->GetKappa())) {
    return true;
  }

  return false;
}

const bool NodeDeleteDecider::CheckBacktrackPath() {
  const Node3d* parent_node = request_.parent_node;
  const Node3d* cur_node = request_.current_node;

  if (parent_node == nullptr || cur_node == nullptr) {
    return false;
  }

  if (IsSteerOpposite(parent_node->GetKappa(), cur_node->GetKappa())) {
    return false;
  }

  if (IsGearDifferent(parent_node->GetGearType(), cur_node->GetGearType()) &&
      std::fabs(parent_node->GetKappa() - cur_node->GetKappa()) < 0.14f) {
    return true;
  }

  return false;
}

const bool NodeDeleteDecider::CheckUnexpectedGear() {
  if (IsGearDifferent(request_.gear_request, request_.cur_gear)) {
    return true;
  }
  return false;
}

const bool NodeDeleteDecider::CheckExceedGearShiftNumber() {
  if (request_.current_node != nullptr &&
      request_.current_node->GetGearSwitchNum() >
          input_.max_gear_shift_number) {
    return true;
  }

  return false;
}

const bool NodeDeleteDecider::CheckExceedScurveNum() {
  if (request_.current_node != nullptr &&
      request_.current_node->GetScurveNum() > input_.max_scurve_number) {
    return true;
  }

  return false;
}

const bool NodeDeleteDecider::CheckDelByParentNode() {
  if (request_.current_node == nullptr || request_.parent_node == nullptr) {
    return false;
  }

  if (request_.current_node->GetGlobalID() ==
      request_.parent_node->GetGlobalID()) {
    return true;
  }

  const Node3d* grandpa = request_.parent_node->GetPreNode();

  if (grandpa != nullptr) {
    if (grandpa->GetGlobalID() == request_.current_node->GetGlobalID()) {
      return true;
    }
  }

  return false;
}

const bool NodeDeleteDecider::CheckDelByStartNode() {
  if (request_.current_node == nullptr) {
    return false;
  }

  if (request_.current_node->GetPathType() == AstarPathType::START_NODE ||
      request_.current_node->GetPathType() == AstarPathType::END_NODE) {
    return false;
  }

  if (request_.current_node->GetGlobalID() == input_.start_id) {
    return true;
  }

  return false;
}

const bool NodeDeleteDecider::CheckDelByLoopBackNode() {
  if (request_.current_node == nullptr || request_.parent_node == nullptr) {
    return false;
  }

  const size_t new_node_id = request_.current_node->GetGlobalID();

  const Node3d* parent = request_.parent_node;

  for (size_t i = 0; i < request_.explored_node_num; i++) {
    if (parent->GetGlobalID() == new_node_id) {
      return true;
    }

    parent = parent->GetPreNode();
    if (parent == nullptr) {
      return false;
    }
  }

  return false;
}

const bool NodeDeleteDecider::CheckDelBySameGridNodeContinuous() {
  if (request_.current_node == nullptr) {
    return false;
  }

  if (request_.current_node->GetEulerDist(request_.old_node) > 0.01 ||
      request_.current_node->GetPhiErr(request_.old_node) > 0.0087f) {
    return true;
  }

  return false;
}

void NodeDeleteDecider::SplitPathPtsUsingGradeBuffer(
    const std::vector<common_math::PathPt<float>>& origin_pts,
    std::vector<GradeBufferPathPts>& grade_buffer_pts_vec) {
  grade_buffer_pts_vec.clear();
  if (origin_pts.empty()) {
    return;
  }

  GradeColDetBufferType last_type = GetGradeBufferType(origin_pts[0]);
  std::vector<common_math::PathPt<float>> pts;

  for (size_t i = 0; i < origin_pts.size(); ++i) {
    const auto& pt = origin_pts[i];
    const GradeColDetBufferType now_type = GetGradeBufferType(pt);

    if (now_type != last_type) {
      grade_buffer_pts_vec.emplace_back();
      grade_buffer_pts_vec.back().type = last_type;
      grade_buffer_pts_vec.back().pts = std::move(pts);
      pts.clear();
    }

    pts.emplace_back(pt);

    if (i == origin_pts.size() - 1) {
      grade_buffer_pts_vec.emplace_back();
      grade_buffer_pts_vec.back().type = now_type;
      grade_buffer_pts_vec.back().pts = std::move(pts);
    } else {
      last_type = now_type;
    }
  }
}

const GradeColDetBufferType NodeDeleteDecider::GetGradeBufferType(
    const common_math::PathPt<float>& pt) {
  const bool is_turn = std::fabs(pt.kappa) >= 1e-3f;
  const float abs_theta_deg = std::fabs(pt.GetTheta()) * common_math::kRad2DegF;
  constexpr float kSlotHeadingThresholdDeg = 23.86f;
  constexpr float kHeadInSlotHeadingThresholdDeg =
      180.0f - kSlotHeadingThresholdDeg;

  // 0->out_slot  1->entrance_slot 2->in_slot
  int slot_type = 0;
  if (input_.scenario_type ==
      ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN) {
    if (slot_entrance_box_.IsContain(pt.pos) &&
        abs_theta_deg > kSlotHeadingThresholdDeg) {
      slot_type = 1;
    } else if (slot_box_.IsContain(pt.pos)) {
      slot_type = abs_theta_deg < kSlotHeadingThresholdDeg ? 2 : 1;
    }
  } else if (input_.scenario_type ==
             ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
    if (slot_entrance_box_.IsContain(pt.pos) &&
        abs_theta_deg < kHeadInSlotHeadingThresholdDeg) {
      slot_type = 1;
    } else if (slot_box_.IsContain(pt.pos)) {
      slot_type = abs_theta_deg > kHeadInSlotHeadingThresholdDeg ? 2 : 1;
    }
  }

  if (is_turn) {
    switch (slot_type) {
      case 0:
        return GradeColDetBufferType::TURN_PATH_OUT_SLOT_BUFFER;
      case 1:
        return GradeColDetBufferType::TURN_PATH_ENTRANCE_SLOT_BUFFER;
      case 2:
        return GradeColDetBufferType::TURN_PATH_IN_SLOT_BUFFER;
      default:
        return GradeColDetBufferType::TURN_PATH_OUT_SLOT_BUFFER;
    }
  }

  switch (slot_type) {
    case 0:
      return GradeColDetBufferType::STRAIGHT_PATH_OUT_SLOT_BUFFER;
    case 1:
      return GradeColDetBufferType::STRAIGHT_PATH_ENTRANCE_SLOT_BUFFER;
    case 2:
      return GradeColDetBufferType::STRAIGHT_PATH_IN_SLOT_BUFFER;
    default:
      return GradeColDetBufferType::STRAIGHT_PATH_OUT_SLOT_BUFFER;
  }
}

const bool NodeDeleteDecider::CheckPtsCollision(
    const std::vector<common_math::PathPt<float>>& pts, bool is_special_node,
    ObsToPathDistRelativeSlot* obs_dist, double* safe_remain_dist) {
  if (pts.empty()) {
    return false;
  }

  const std::shared_ptr<EDTCollisionDetector>& edt_col_det_ptr =
      col_det_interface_ptr_->GetEDTColDetPtr();
  const bool need_cal_obs_dist = input_.need_cal_obs_dist;
  const PathColDetBuffer& path_col_det_buffer = input_.path_col_det_buffer;
  const ParkingLatLonSpeedBuffer& speed_buffer =
      apa_param.GetParam().lat_lon_speed_buffer;

  const auto set_special_buffers = [&](float* lon_buffer,
                                       float* body_lat_buffer,
                                       float* mirror_lat_buffer) {
    *body_lat_buffer = speed_buffer.leave_initial_place_body_lat_buffer + 1e-2f;
    *mirror_lat_buffer =
        speed_buffer.leave_initial_place_mirror_lat_buffer + 1e-2f;
    *lon_buffer = speed_buffer.leave_initial_place_lon_buffer;
  };

  const auto set_buffers_by_type = [&](GradeColDetBufferType type,
                                       float* lon_buffer,
                                       float* body_lat_buffer,
                                       float* mirror_lat_buffer) {
    *lon_buffer = path_col_det_buffer.lon_buffer;
    *body_lat_buffer = path_col_det_buffer.body_lat_buffer;
    *mirror_lat_buffer = path_col_det_buffer.mirror_lat_buffer;
    switch (type) {
      case GradeColDetBufferType::STRAIGHT_PATH_IN_SLOT_BUFFER:
        *body_lat_buffer = path_col_det_buffer.in_slot_body_lat_buffer;
        *mirror_lat_buffer = path_col_det_buffer.in_slot_mirror_lat_buffer;
        break;
      case GradeColDetBufferType::STRAIGHT_PATH_ENTRANCE_SLOT_BUFFER:
        *body_lat_buffer = path_col_det_buffer.entrance_slot_body_lat_buffer;
        *mirror_lat_buffer =
            path_col_det_buffer.entrance_slot_mirror_lat_buffer;
        break;
      case GradeColDetBufferType::STRAIGHT_PATH_OUT_SLOT_BUFFER:
        *body_lat_buffer = path_col_det_buffer.out_slot_body_lat_buffer;
        *mirror_lat_buffer = path_col_det_buffer.out_slot_mirror_lat_buffer;
        break;
      case GradeColDetBufferType::TURN_PATH_IN_SLOT_BUFFER:
        *body_lat_buffer = path_col_det_buffer.in_slot_body_lat_buffer +
                           path_col_det_buffer.in_slot_extra_turn_lat_buffer;
        *mirror_lat_buffer = path_col_det_buffer.in_slot_mirror_lat_buffer +
                             path_col_det_buffer.in_slot_extra_turn_lat_buffer;
        break;
      case GradeColDetBufferType::TURN_PATH_ENTRANCE_SLOT_BUFFER:
        *body_lat_buffer =
            path_col_det_buffer.entrance_slot_body_lat_buffer +
            path_col_det_buffer.entrance_slot_extra_turn_lat_buffer;
        *mirror_lat_buffer =
            path_col_det_buffer.entrance_slot_mirror_lat_buffer +
            path_col_det_buffer.entrance_slot_extra_turn_lat_buffer;
        break;
      case GradeColDetBufferType::TURN_PATH_OUT_SLOT_BUFFER:
        *body_lat_buffer = path_col_det_buffer.out_slot_body_lat_buffer +
                           path_col_det_buffer.out_slot_extra_turn_lat_buffer;
        *mirror_lat_buffer = path_col_det_buffer.out_slot_mirror_lat_buffer +
                             path_col_det_buffer.out_slot_extra_turn_lat_buffer;
        break;
      default:
        break;
    }
  };

  const auto set_obs_dist = [&](GradeColDetBufferType type,
                                float min_obs_dist) {
    switch (type) {
      case GradeColDetBufferType::STRAIGHT_PATH_IN_SLOT_BUFFER:
        obs_dist->obs_dist_in_slot_straight = min_obs_dist;
        break;
      case GradeColDetBufferType::STRAIGHT_PATH_ENTRANCE_SLOT_BUFFER:
        obs_dist->obs_dist_entrance_slot_straight = min_obs_dist;
        break;
      case GradeColDetBufferType::STRAIGHT_PATH_OUT_SLOT_BUFFER:
        obs_dist->obs_dist_out_slot_straight = min_obs_dist;
        break;
      case GradeColDetBufferType::TURN_PATH_IN_SLOT_BUFFER:
        obs_dist->obs_dist_in_slot_turn = min_obs_dist;
        break;
      case GradeColDetBufferType::TURN_PATH_ENTRANCE_SLOT_BUFFER:
        obs_dist->obs_dist_entrance_slot_turn = min_obs_dist;
        break;
      case GradeColDetBufferType::TURN_PATH_OUT_SLOT_BUFFER:
        obs_dist->obs_dist_out_slot_turn = min_obs_dist;
        break;
      default:
        break;
    }
  };

  float body_lat_buffer = path_col_det_buffer.body_lat_buffer;
  float mirror_lat_buffer = path_col_det_buffer.mirror_lat_buffer;
  float lon_buffer = path_col_det_buffer.lon_buffer;

  if (!path_col_det_buffer.need_distinguish_outinslot) {
    if (is_special_node) {
      set_special_buffers(&lon_buffer, &body_lat_buffer, &mirror_lat_buffer);
    }

    const auto& res = edt_col_det_ptr->Update(
        pts, lon_buffer, body_lat_buffer, mirror_lat_buffer, need_cal_obs_dist);

    if (need_cal_obs_dist) {
      obs_dist->SetDist(res.min_obs_dist);
    }

    *safe_remain_dist = res.remain_dist;
    return res.col_flag;
  }

  const auto update_collision =
      [&](const std::vector<common_math::PathPt<float>>& check_pts,
          GradeColDetBufferType type) {
        if (is_special_node) {
          set_special_buffers(&lon_buffer, &body_lat_buffer,
                              &mirror_lat_buffer);
        } else {
          set_buffers_by_type(type, &lon_buffer, &body_lat_buffer,
                              &mirror_lat_buffer);
        }

        const auto& res =
            edt_col_det_ptr->Update(check_pts, lon_buffer, body_lat_buffer,
                                    mirror_lat_buffer, need_cal_obs_dist);

        if (need_cal_obs_dist) {
          set_obs_dist(type, res.min_obs_dist);
        }

        *safe_remain_dist = res.remain_dist;
        return res.col_flag;
      };

  grade_segment_pts_.clear();
  grade_segment_pts_.reserve(pts.size());

  GradeColDetBufferType segment_type = GetGradeBufferType(pts.front());
  size_t segment_begin_index = 0;

  for (size_t i = 1; i <= pts.size(); ++i) {
    const bool reach_segment_end = i == pts.size();
    GradeColDetBufferType next_type = segment_type;
    if (!reach_segment_end) {
      next_type = GetGradeBufferType(pts[i]);
    }

    if (!reach_segment_end && next_type == segment_type) {
      continue;
    }

    const bool is_same_grade_path =
        segment_begin_index == 0 && reach_segment_end;
    if (is_same_grade_path) {
      return update_collision(pts, segment_type);
    }

    grade_segment_pts_.clear();
    grade_segment_pts_.insert(grade_segment_pts_.end(),
                              pts.begin() + segment_begin_index,
                              pts.begin() + i);
    if (update_collision(grade_segment_pts_, segment_type)) {
      return true;
    }

    if (!reach_segment_end) {
      segment_begin_index = i;
      segment_type = next_type;
    }
  }

  return false;
}

const std::string GetNodeDeleteReasonString(const NodeDeleteReason reason) {
  switch (reason) {
    case NodeDeleteReason::UNKNOWN:
      return "UNKNOWN";
    case NodeDeleteReason::UNEXPECTED_GEAR:
      return "UNEXPECTED_GEAR";
    case NodeDeleteReason::EXCEED_GEAR_SHIFT_NUMBER:
      return "EXCEED_GEAR_SHIFT_NUMBER";
    case NodeDeleteReason::START_NODE:
      return "START_NODE";
    case NodeDeleteReason::PARENT_NODE:
      return "PARENT_NODE";
    case NodeDeleteReason::LOOP_BACK_NODE:
      return "LOOP_BACK_NODE";
    case NodeDeleteReason::SAME_GRID_NODE_CONTINUOUS:
      return "SAME_GRID_NODE_CONTINUOUS";
    case NodeDeleteReason::OUT_OF_GRID_BOUND:
      return "OUT_OF_GRID_BOUND";
    case NodeDeleteReason::OUT_OF_POSE_BOUND:
      return "OUT_OF_POSE_BOUND";
    case NodeDeleteReason::COLLISION:
      return "COLLISION";
    case NodeDeleteReason::ZIGZAG_PATH:
      return "ZIGZAG_PATH";
    case NodeDeleteReason::EXCEED_SCURVE_NUMBER:
      return "EXCEED_SCURVE_NUMBER";
    case NodeDeleteReason::BACKTRACK_PATH:
      return "BACKTRACK_PATH";
    default:
      return "UNKNOWN";
  }
}

void PrintNodeDeleteReason(const NodeDeleteReason reason,
                           const bool enable_log) {
  ILOG_INFO_IF(enable_log) << "node delete reason: "
                           << GetNodeDeleteReasonString(reason);
}

const float NodeDeleteDecider::CalcObsDistCost(const float obs_dist,
                                               const float gear_change_penalty,
                                               const float length_cost,
                                               const float critical_dist,
                                               const float safe_dist,
                                               const float adsolute_safe_dist) {
  // four zone: very critical(z1), relative critical(z2), relative safe(z3),
  // absolute safe(z4)

  // obs_dist unit is cm

  const float target_c_crit = 2.0f * gear_change_penalty + 8.0 * length_cost;
  const float target_c_safe = 1.0f * gear_change_penalty + 4.0 * length_cost;

  // zone 3  f(x) = z3_k * (adsolute_safe_dist - x)^n_z3
  const float n_z3 = 1.05f;
  const float dist_z3 = adsolute_safe_dist - safe_dist;
  const float z3_k = target_c_safe / std::pow(dist_z3, n_z3);
  const float slope_safe = -1.0f * n_z3 * z3_k * std::pow(dist_z3, n_z3 - 1.0f);

  // zone 2 trapezoidal area method
  const float dist_z2 = safe_dist - critical_dist;
  const float cost_z2 = target_c_safe - target_c_crit;
  const float slope_crit = 2.0f * cost_z2 / dist_z2 - slope_safe;

  // zone 1
  const float dist_z1 = critical_dist - 0.0f;
  const float slope_0 = -8.8f;
  const float cost_0 = target_c_crit - 0.5f * (slope_crit + slope_0) * dist_z1;

  float cost = 0.0f, dk = 0.0f, dx = 0.0f;

  if (obs_dist < 0.0f) {
    // zone 0, but it is impossible to exist
    // f(x) = y0 + k * x
    cost = cost_0 + slope_0 * obs_dist;
  } else if (obs_dist < critical_dist) {
    // zone 1
    // f(x) = y0 + v * x + 0.5 * a * x^2
    dx = obs_dist - 0.0f;
    dk = (slope_crit - slope_0) / dist_z1;
    cost = cost_0 + slope_0 * dx + 0.5f * dk * std::pow(dx, 2.0f);
  } else if (obs_dist < safe_dist) {
    // zone 2
    // f(x) = y0 + v * x + 0.5 * a * x^2
    dx = obs_dist - critical_dist;
    dk = (slope_safe - slope_crit) / dist_z2;
    cost = target_c_crit + slope_crit * dx + 0.5f * dk * std::pow(dx, 2.0f);
  } else if (obs_dist < adsolute_safe_dist) {
    // zone 3
    // f(x) = z3_k * (adsolute_safe_dist - x)^n_z3
    dx = adsolute_safe_dist - obs_dist;
    cost = z3_k * std::pow(dx, n_z3);
  } else {
    // zone 4
    cost = 0.0f;
  }

  return cost;
}

}  // namespace apa_planner
}  // namespace planning