#include "node_delete_decider.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

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

    pose_bound_.x_down_bound =
        std::min({ego_info_under_slot.cur_pose.pos.x() - 0.4,
                  ego_info_under_slot.target_pose.pos.x() + 0.4, 2.68}) -
        bound;

    if (input.swap_start_goal) {
      pose_bound_.x_down_bound =
          std::min({ego_info_under_slot.cur_pose.pos.x() - 0.4,
                    ego_info_under_slot.target_pose.pos.x() - 0.1, 2.68}) -
          bound;
    }

    pose_bound_.curve_x_down_bound =
        std::min({ego_info_under_slot.cur_pose.pos.x() - 0.1,
                  ego_info_under_slot.target_pose.pos.x() - 0.02, 1.68}) -
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

    slot_box_ = cdl::AABB2f(Eigen::Vector2f(0.0f, -half_slot_width),
                            Eigen::Vector2f(slot_x + 1.25f, half_slot_width));

    slot_entrance_box_ =
        cdl::AABB2f(Eigen::Vector2f(slot_x - 0.68f, -half_slot_width),
                    Eigen::Vector2f(slot_x + 1.25f, half_slot_width));
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

const bool NodeDeleteDecider::CheckCollision() {
  if (col_det_interface_ptr_->GetObsManagerPtr()->GetObstacles().empty()) {
    // ILOG_INFO << "OBS IS EMPTY";
    return false;
  }

  if (request_.current_node != nullptr) {
    Node3d* current_node = request_.current_node;

    // do col det for every pt in node
    NodePath path = current_node->GetNodePath();

    // The first {x, y, phi} is collision free unless they are start and end
    // configuration of search problem
    size_t check_start_index = 0;
    if (path.point_size == 1) {
      check_start_index = 0;
    } else {
      check_start_index = 1;
    }

#if USE_LINK_PT_LINE
    std::vector<common_math::PathPt<float>> pt_vec;
    common_math::PathPt<float> pt;
#else
    std::vector<geometry_lib::PathPoint> pt_vec;
    geometry_lib::PathPoint pt;
#endif

    pt_vec.reserve(path.point_size - check_start_index + 2);

    for (size_t i = check_start_index; i < path.point_size; ++i) {
      pt.SetPos(path.points[i].x, path.points[i].y);
      pt.SetTheta(path.points[i].theta);
      pt.kappa = current_node->GetKappa();
      pt.s = i * input_.sample_ds;
      pt_vec.emplace_back(pt);
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

    const bool col_flag =
        CheckPtsCollision(pt_vec, current_node->GetGearType(), is_special_node,
                          &obs_dist, &safe_remain_dist);

    if (input_.need_cal_obs_dist) {
      current_node->SetObsDistRelativeSlot(obs_dist);
    }
    size_t min_save_pt_number = 2;
    if (col_flag && safe_remain_dist > min_save_pt_number * input_.sample_ds &&
        safe_remain_dist < path.path_dist) {
      // i must be bigger than 1
      size_t i = min_save_pt_number;
      for (; i < path.point_size; ++i) {
        if (i * input_.sample_ds > safe_remain_dist) {
          break;
        }
      }
      path.point_size = i;

      current_node->UpdatePath(path, input_.map_bound, input_.config,
                               (i - 1) * input_.sample_ds);

      return false;
    }

    return col_flag;
  }

  else {
    CurveNode* curve_node = request_.curve_node;

    // do col det for every pt in curve
    const CurvePath& path = curve_node->GetCurvePath();
// The first {x, y, phi} is collision free unless they are start and end
#if USE_LINK_PT_LINE
    std::vector<std::vector<common_math::PathPt<float>>> pt_vec_vec;
    std::vector<common_math::PathPt<float>> pt_vec;
#else
    std::vector<std::vector<geometry_lib::PathPoint>> pt_vec_vec;
    std::vector<geometry_lib::PathPoint> pt_vec;
#endif

    std::vector<AstarPathGear> gear_vec;
    AstarPathGear cur_gear, next_gear;
    bool gear_change_flag = false;
    pt_vec_vec.reserve(path.gear_change_number + 2);
    gear_vec.reserve(path.gear_change_number + 2);
    for (int i = 0; i < path.segment_size; ++i) {
      for (int j = 1; j < path.ptss[i].size(); ++j) {
        pt_vec.emplace_back(path.ptss[i][j]);
      }
      cur_gear = path.gears[i];
      if (i < path.segment_size - 1) {
        next_gear = path.gears[i + 1];
        gear_change_flag = IsGearDifferent(cur_gear, next_gear);
      } else {
        gear_change_flag = true;
      }
      if (gear_change_flag) {
        pt_vec_vec.emplace_back(pt_vec);
        gear_vec.emplace_back(cur_gear);
        pt_vec.clear();
      }
    }

    double s = 0.0;
    for (size_t i = 0; i < pt_vec_vec.size(); ++i) {
      for (size_t j = 0; j < pt_vec_vec[i].size(); ++j) {
        s += input_.sample_ds;
        pt_vec_vec[i][j].s = s;
      }
    }

    bool enable_smart_fold_mirror = false;
    while ((input_.scenario_type ==
                ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN ||
            input_.scenario_type ==
                ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) &&
           input_.enable_smart_fold_mirror) {
      if (pt_vec_vec.empty()) {
        break;
      }
      const AstarPathGear last_gear = gear_vec.back();
      const auto& pts = pt_vec_vec.back();
#if USE_LINK_PT_LINE
      std::vector<common_math::PathPt<float>> pt_line, pt_arc;
#else
      std::vector<geometry_lib::PathPoint> pt_line, pt_arc;
#endif

      int i = pts.size() - 1;
      for (; i >= 0; --i) {
        if (std::fabs(pts[i].kappa) > 1e-3) {
          break;
        }
        pt_line.emplace_back(pts[i]);
      }

      if (pt_line.empty()) {
        break;
      }

      if (pt_line.size() < pts.size()) {
        for (int j = i; j >= 0; --j) {
          pt_arc.emplace_back(pts[j]);
        }
      }

      const ApaParameters& param = apa_param.GetParam();
      const float slot_x = input_.ego_info_under_slot.slot
                               .processed_corner_coord_local_.pt_01_mid.x();

      float mirror_x =
          pt_line.back().GetX() + param.lon_dist_mirror_to_rear_axle;

      if (input_.scenario_type ==
          ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
        mirror_x = pt_line.back().GetX() - param.lon_dist_mirror_to_rear_axle;
      }

      const float lower_x =
          slot_x + param.smart_fold_mirror_params.x_down_offset;

      const float upper_x = slot_x + param.smart_fold_mirror_params.x_up_offset;

      const float redundant_x = 0.1f;

      if (mirror_x < lower_x) {
        break;
      }

      pt_vec_vec.pop_back();
      gear_vec.pop_back();

      if (pt_arc.size() > 0) {
        std::reverse(pt_arc.begin(), pt_arc.end());
        pt_vec_vec.emplace_back(pt_arc);
        gear_vec.emplace_back(last_gear);
      }

      if (mirror_x < upper_x) {
        std::reverse(pt_line.begin(), pt_line.end());
        pt_vec_vec.emplace_back(pt_line);
        gear_vec.emplace_back(last_gear);
      } else {
#if USE_LINK_PT_LINE
        std::vector<common_math::PathPt<float>> pt_line_up, pt_line_down;
#else
        std::vector<geometry_lib::PathPoint> pt_line_up, pt_line_down;
#endif
        i = 0;
        for (; i < pt_line.size(); ++i) {
          if (pt_line[i].GetX() > upper_x - redundant_x) {
            break;
          }
          pt_line_down.emplace_back(pt_line[i]);
        }
        for (int j = i; j < pt_line.size(); ++j) {
          pt_line_up.emplace_back(pt_line[j]);
        }

        std::reverse(pt_line_down.begin(), pt_line_down.end());
        std::reverse(pt_line_up.begin(), pt_line_up.end());
        pt_vec_vec.emplace_back(pt_line_up);
        pt_vec_vec.emplace_back(pt_line_down);
        gear_vec.emplace_back(last_gear);
        gear_vec.emplace_back(last_gear);
      }

      enable_smart_fold_mirror = true;
      break;
    }

    ObsToPathDistRelativeSlot small_obs_dist;
    double safe_remain_dist = 0.0;
    for (size_t i = 0; i < pt_vec_vec.size(); ++i) {
      ObsToPathDistRelativeSlot obs_dist;
      if (enable_smart_fold_mirror && i == pt_vec_vec.size() - 1) {
        col_det_interface_ptr_->Init(true);
      }
      const bool col_flag = CheckPtsCollision(pt_vec_vec[i], gear_vec[i], false,
                                              &obs_dist, &safe_remain_dist);
      if (enable_smart_fold_mirror) {
        col_det_interface_ptr_->Init(false);
      }
      if (input_.need_cal_obs_dist) {
        small_obs_dist.SetSmallerDist(obs_dist);
        curve_node->SetObsDistRelativeSlot(small_obs_dist);
      }

      if (col_flag) {
        if (i == pt_vec_vec.size() - 1 && pt_vec_vec.back().size() > 0 &&
            safe_remain_dist >
                pt_vec_vec.back().back().s - input_.sample_ds - 0.001) {
          return false;
        }
        return true;
      }
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
#if USE_LINK_PT_LINE
    const std::vector<common_math::PathPt<float>>& origin_pts,
#else
    const std::vector<geometry_lib::PathPoint>& origin_pts,
#endif
    std::vector<GradeBufferPathPts>& grade_buffer_pts_vec) {
  grade_buffer_pts_vec.clear();
  if (origin_pts.size() < 1) {
    return;
  }
  GradeBufferPathPts grade_buffer_pts;
  GradeColDetBufferType last_type = GetGradeBufferType(origin_pts[0]);
  GradeColDetBufferType now_type;
#if USE_LINK_PT_LINE
  std::vector<common_math::PathPt<float>> pts;
#else
  std::vector<geometry_lib::PathPoint> pts;
#endif

  for (size_t i = 0; i < origin_pts.size(); ++i) {
    const auto& pt = origin_pts[i];
    now_type = GetGradeBufferType(pt);

    if (now_type != last_type) {
      grade_buffer_pts.type = last_type;
      grade_buffer_pts.pts = pts;
      grade_buffer_pts_vec.emplace_back(grade_buffer_pts);
      pts.clear();
    }

    if (i == origin_pts.size() - 1) {
      pts.emplace_back(pt);
      grade_buffer_pts.type = now_type;
      grade_buffer_pts.pts = pts;
      grade_buffer_pts_vec.emplace_back(grade_buffer_pts);
    } else {
      pts.emplace_back(pt);
      last_type = now_type;
    }
  }
}

const GradeColDetBufferType NodeDeleteDecider::GetGradeBufferType(
#if USE_LINK_PT_LINE
    const common_math::PathPt<float>& pt
#else
    const geometry_lib::PathPoint& pt
#endif
) {
  bool is_turn = true;
  if (std::fabs(pt.kappa) < 1e-3f) {
    is_turn = false;
  }
  // 0->out_slot  1->entrance_slot 2->in_slot
  int slot_type = 0;
  if (input_.scenario_type ==
      ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN) {
    if (slot_entrance_box_.IsContain(pt.pos) &&
        std::fabs(pt.GetTheta()) * common_math::kRad2DegF > 23.86f) {
      slot_type = 1;
    } else if (slot_box_.IsContain(pt.pos)) {
      if (std::fabs(pt.GetTheta()) * common_math::kRad2DegF < 23.86f) {
        slot_type = 2;
      } else {
        slot_type = 1;
      }
    }
  } else if (input_.scenario_type ==
             ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN) {
    if (slot_entrance_box_.IsContain(pt.pos) &&
        std::fabs(pt.GetTheta()) * common_math::kRad2DegF < 180.0f - 23.86f) {
      slot_type = 1;
    } else if (slot_box_.IsContain(pt.pos)) {
      if (std::fabs(pt.GetTheta()) * common_math::kRad2DegF > 180.0f - 23.86f) {
        slot_type = 2;
      } else {
        slot_type = 1;
      }
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
  } else {
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

  return GradeColDetBufferType::TURN_PATH_OUT_SLOT_BUFFER;
}

const bool NodeDeleteDecider::CheckPtsCollision(
#if USE_LINK_PT_LINE
    const std::vector<common_math::PathPt<float>>& pts,
#else
    const std::vector<geometry_lib::PathPoint>& pts,
#endif
    AstarPathGear gear, bool is_special_node,
    ObsToPathDistRelativeSlot* obs_dist, double* safe_remain_dist) {
  const std::shared_ptr<EDTCollisionDetector>& edt_col_det_ptr =
      col_det_interface_ptr_->GetEDTColDetPtr();

  if (pts.empty()) {
    return false;
  }

  const bool need_cal_obs_dist = input_.need_cal_obs_dist;

  const ParkingLatLonSpeedBuffer& speed_buffer =
      apa_param.GetParam().lat_lon_speed_buffer;

  if (!input_.path_col_det_buffer.need_distinguish_outinslot) {
    float body_lat_buffer = input_.path_col_det_buffer.body_lat_buffer;
    float mirror_lat_buffer = input_.path_col_det_buffer.mirror_lat_buffer;
    float lon_buffer = input_.path_col_det_buffer.lon_buffer;
    float min_obs_dist = 26.8f;
    if (is_special_node) {
      // only to get rid of special case
      body_lat_buffer = speed_buffer.leave_initial_place_body_lat_buffer + 1e-2;
      mirror_lat_buffer =
          speed_buffer.leave_initial_place_mirror_lat_buffer + 1e-2;
      lon_buffer = speed_buffer.leave_initial_place_lon_buffer;
    }

#if USE_LINK_PT_LINE
    const auto res =
        edt_col_det_ptr->Update(pts, lon_buffer, body_lat_buffer,
                                mirror_lat_buffer, gear, need_cal_obs_dist);
    min_obs_dist = res.min_obs_dist;
#else
    const auto res = edt_col_det_ptr->Update(pts, body_lat_buffer, lon_buffer,
                                             input_.need_cal_obs_dist, 0.5,
                                             true, mirror_lat_buffer, gear);
    min_obs_dist = res.pt_closest2obs.first;
#endif

    if (need_cal_obs_dist) {
      obs_dist->SetDist(min_obs_dist);
    }

    *safe_remain_dist = res.remain_dist;
    return res.col_flag;
  }

  std::vector<GradeBufferPathPts> grade_buffer_pts_vec;
  SplitPathPtsUsingGradeBuffer(pts, grade_buffer_pts_vec);
  for (GradeBufferPathPts& grade_buffer_pts : grade_buffer_pts_vec) {
    GradeColDetBufferType temp_type = grade_buffer_pts.type;
    const auto& temp_pts = grade_buffer_pts.pts;
    float lon_buffer = input_.path_col_det_buffer.lon_buffer;
    float body_lat_buffer = input_.path_col_det_buffer.body_lat_buffer;
    float mirror_lat_buffer = input_.path_col_det_buffer.mirror_lat_buffer;
    switch (temp_type) {
      case GradeColDetBufferType::STRAIGHT_PATH_IN_SLOT_BUFFER:
        body_lat_buffer = input_.path_col_det_buffer.in_slot_body_lat_buffer;
        mirror_lat_buffer =
            input_.path_col_det_buffer.in_slot_mirror_lat_buffer;
        break;
      case GradeColDetBufferType::STRAIGHT_PATH_ENTRANCE_SLOT_BUFFER:
        body_lat_buffer =
            input_.path_col_det_buffer.entrance_slot_body_lat_buffer;
        mirror_lat_buffer =
            input_.path_col_det_buffer.entrance_slot_mirror_lat_buffer;
        break;
      case GradeColDetBufferType::STRAIGHT_PATH_OUT_SLOT_BUFFER:
        body_lat_buffer = input_.path_col_det_buffer.out_slot_body_lat_buffer;
        mirror_lat_buffer =
            input_.path_col_det_buffer.out_slot_mirror_lat_buffer;
        break;
      case GradeColDetBufferType::TURN_PATH_IN_SLOT_BUFFER:
        body_lat_buffer =
            input_.path_col_det_buffer.in_slot_body_lat_buffer +
            input_.path_col_det_buffer.in_slot_extra_turn_lat_buffer;
        mirror_lat_buffer =
            input_.path_col_det_buffer.in_slot_mirror_lat_buffer +
            input_.path_col_det_buffer.in_slot_extra_turn_lat_buffer;
        break;
      case GradeColDetBufferType::TURN_PATH_ENTRANCE_SLOT_BUFFER:
        body_lat_buffer =
            input_.path_col_det_buffer.entrance_slot_body_lat_buffer +
            input_.path_col_det_buffer.entrance_slot_extra_turn_lat_buffer;
        mirror_lat_buffer =
            input_.path_col_det_buffer.entrance_slot_mirror_lat_buffer +
            input_.path_col_det_buffer.entrance_slot_extra_turn_lat_buffer;
      case GradeColDetBufferType::TURN_PATH_OUT_SLOT_BUFFER:
        body_lat_buffer =
            input_.path_col_det_buffer.out_slot_body_lat_buffer +
            input_.path_col_det_buffer.out_slot_extra_turn_lat_buffer;
        mirror_lat_buffer =
            input_.path_col_det_buffer.out_slot_mirror_lat_buffer +
            input_.path_col_det_buffer.out_slot_extra_turn_lat_buffer;
        break;
      default:
        break;
    }

    if (is_special_node) {
      // only to get rid of special case
      body_lat_buffer = speed_buffer.leave_initial_place_body_lat_buffer + 1e-2;
      mirror_lat_buffer =
          speed_buffer.leave_initial_place_mirror_lat_buffer + 1e-2;
      lon_buffer = speed_buffer.leave_initial_place_lon_buffer;
    }

    float min_obs_dist = 26.8f;
#if USE_LINK_PT_LINE
    const auto res =
        edt_col_det_ptr->Update(temp_pts, lon_buffer, body_lat_buffer,
                                mirror_lat_buffer, gear, need_cal_obs_dist);
    min_obs_dist = res.min_obs_dist;
#else
    const auto res = edt_col_det_ptr->Update(temp_pts, body_lat_buffer,
                                             lon_buffer, need_cal_obs_dist, 0.5,
                                             true, mirror_lat_buffer, gear);
    min_obs_dist = res.pt_closest2obs.first;
#endif

    if (input_.need_cal_obs_dist) {
      switch (temp_type) {
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
    }

    *safe_remain_dist = res.remain_dist;
    if (res.col_flag) {
      return true;
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