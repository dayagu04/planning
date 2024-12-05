#include "ego_lane_road_right_decider.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <numeric>
#include <type_traits>

#include "../../common/planning_gflags.h"
#include "behavior_planners/lateral_offset_decider/lateral_offset_decider_utils.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "ego_state_manager.h"
#include "planning_context.h"
#include "reference_path.h"
#include "reference_path_manager.h"
#include "spline_projection.h"
#include "tasks/behavior_planners/vision_only_lateral_behavior_planner/vision_lateral_behavior_planner.h"
#include "tracked_object.h"
#include "utils/pose2d_utils.h"
#include "vehicle_config_context.h"
#include "vehicle_status.pb.h"

namespace planning {

namespace {
constexpr double kEps = 1e-6;
constexpr double kEgoPreviewTimeMinThd = 3.0;
constexpr double kEgoPreviewTimeMaxThd = 5.0;
constexpr double kExistSplitLateralDisThd = 1.5;
constexpr double kCenterLineLateralDisThd = 0.8;
constexpr double kNearPreviewDistanceThd = 20.0;
constexpr double kDefaultIntersectionSpeedLimit = 19.5;
}  // namespace

EgoLaneRoadRightDecider::EgoLaneRoadRightDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session), session_(session) {
  name_ = "EgoLaneRoadRightDecider";
  config_ = config_builder->cast<ScenarioStateMachineConfig>();
  virtual_lane_mgr_ =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  ref_path_mgr_ = session_->environmental_model().get_reference_path_manager();

  Init();
}

void EgoLaneRoadRightDecider::Init() {
  is_merge_region_ = false;
  merge_lane_virtual_id_ = 0;
  cur_lane_is_continue_ = true;
  boundary_merge_point_valid_ = false;
  merge_direction_ = NONE_LANE_MERGE;
}

bool EgoLaneRoadRightDecider::Execute() {
  merge_lane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
  const bool active = session_->environmental_model().GetVehicleDbwStatus();
  const int current_lc_status =
      session_->planning_context().lane_change_decider_output().curr_state;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  Point2D ego_point = {ego_state->planning_init_point().x,
                       ego_state->planning_init_point().y};
  is_merge_region_ = false;
  is_split_region_ = false;
  cur_lane_is_continue_ = true;
  boundary_merge_point_valid_ = false;
  merge_point_ = ego_point;
  boundary_merge_point_ = ego_point;

  ComputeIsSplitRegion();

  ComputeIsMergeRegion();
  JSON_DEBUG_VALUE("is_merge_region", is_merge_region_);
  JSON_DEBUG_VALUE("is_split_region", is_split_region_);
  JSON_DEBUG_VALUE("merge_lane_virtual_id", merge_lane_virtual_id_);
  JSON_DEBUG_VALUE("ego_lane_boundary_exist_virtual_line",
                   ego_lane_boundary_exist_virtual_line_);
  JSON_DEBUG_VALUE("target_lane_boundary_exist_virtual_line",
                   target_lane_boundary_exist_virtual_line_);

  if (is_merge_region_) {
    std::vector<Point2D> merge_point_list;
    merge_point_list.resize(2);
    cur_lane_is_continue_ = true;
    int calculate_nums = -1;
    CalculateMergePoint(merge_point_list, &calculate_nums);
    CalculateRoadRight(calculate_nums);
    merge_point_ = merge_point_list[0];
    boundary_merge_point_ = merge_point_list[1];
    boundary_merge_point_valid_ = true;
  } else {
    merge_point_ = ego_point;
    boundary_merge_point_ = ego_point;
    cur_lane_is_continue_ = true;
    boundary_merge_point_valid_ = false;
  }
  JSON_DEBUG_VALUE("macroeconomic_decider_merge_point_x", merge_point_.x);
  JSON_DEBUG_VALUE("macroeconomic_decider_merge_point_y", merge_point_.y);
  JSON_DEBUG_VALUE("boundary_line_merge_point_x", boundary_merge_point_.x);
  JSON_DEBUG_VALUE("boundary_line_merge_point_y", boundary_merge_point_.y);
  JSON_DEBUG_VALUE("cur_lane_is_continue", cur_lane_is_continue_);
  JSON_DEBUG_VALUE("is_left_merge_direction", is_left_merge_direction_);
  JSON_DEBUG_VALUE("is_right_merge_direction", is_right_merge_direction_);

  auto& road_right_decider = session_->mutable_planning_context()
                                 ->mutable_ego_lane_road_right_decider_output();
  road_right_decider.cur_lane_is_continue = cur_lane_is_continue_;
  road_right_decider.merge_lane_virtual_id = merge_lane_virtual_id_;
  road_right_decider.boundary_merge_point_valid = boundary_merge_point_valid_;
  road_right_decider.is_merge_region = is_merge_region_;
  road_right_decider.is_split_region = is_split_region_;
  road_right_decider.boundary_merge_point.x = boundary_merge_point_.x;
  road_right_decider.boundary_merge_point.y = boundary_merge_point_.y;
  return true;
}

void EgoLaneRoadRightDecider::ComputeIsMergeRegion() {
  const auto& function_info = session_->environmental_model().function_info();
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  double ego_vel = ego_state->ego_v();
  const auto& left_lane = virtual_lane_mgr_->get_left_lane();
  const auto& right_lane = virtual_lane_mgr_->get_right_lane();
  std::shared_ptr<ReferencePath> left_reference_path;
  std::shared_ptr<ReferencePath> right_reference_path;
  ego_lane_boundary_exist_virtual_line_ = false;
  target_lane_boundary_exist_virtual_line_ = false;

  //判断与左车道是否merge
  if (left_lane != nullptr) {
    left_reference_path =
        ref_path_mgr_->get_reference_path_by_lane(left_lane->get_virtual_id());
  }
  if (left_reference_path != nullptr) {
    if (ego_vel > kDefaultIntersectionSpeedLimit &&
        function_info.function_mode() == common::DrivingFunctionInfo::NOA) {
      CheckIfMergeWithLeftLane();
      if (((ego_lane_boundary_exist_virtual_line_ &&
            !target_lane_boundary_exist_virtual_line_) ||
           (!ego_lane_boundary_exist_virtual_line_ &&
            target_lane_boundary_exist_virtual_line_)) &&
          !is_split_region_) {
        merge_lane_virtual_id_ = left_lane->get_virtual_id();
        is_merge_region_ = true;
        return;
      }
    }
    if (IsOverlapWithOtherLaneOnEndRegion(left_reference_path,
                                          LEFT_DIRECTION)) {
      merge_lane_virtual_id_ = left_lane->get_virtual_id();
      is_merge_region_ = true;
      return;
    }
  }

  //判断与右车道是否merge
  if (right_lane != nullptr) {
    right_reference_path =
        ref_path_mgr_->get_reference_path_by_lane(right_lane->get_virtual_id());
  }
  if (right_reference_path != nullptr) {
    // if (ego_vel > kDefaultIntersectionSpeedLimit &&
    //     function_info.function_mode() == common::DrivingFunctionInfo::NOA) {
    //   CheckIfMergeWithRightLane();
    //   if (((ego_lane_boundary_exist_virtual_line_ &&
    //   !target_lane_boundary_exist_virtual_line_) ||
    //         (!ego_lane_boundary_exist_virtual_line_ &&
    //         target_lane_boundary_exist_virtual_line_)) &&
    //           !is_split_region_) {
    //     merge_lane_virtual_id_ = right_lane->get_virtual_id();
    //     is_merge_region_ = true;
    //     return;
    //   }
    // }
    if (IsOverlapWithOtherLaneOnEndRegion(right_reference_path,
                                          RIGHT_DIRECTION)) {
      merge_lane_virtual_id_ = right_lane->get_virtual_id();
      is_merge_region_ = true;
      return;
    }
  }
  return;
}

bool EgoLaneRoadRightDecider::IsOverlapWithOtherLaneOnEndRegion(
    const std::shared_ptr<ReferencePath> reference_path,
    const RelativeDirection rel_dir) {
  const auto& current_lane = virtual_lane_mgr_->get_current_lane();
  bool lane_end_satisfied_merge_condition = false;
  const double standard_lane_width = 3.8;
  const double default_lane_buffer = 1.0;
  const double cur_lane_lat_diff_threshold = standard_lane_width - 0.8;
  const double end_lane_lat_diff_threshold = standard_lane_width / 4;
  double cur_to_other_lane_end_lat_diff = NL_NMAX;
  double ref_ego_l = NL_NMAX;
  double ref_ego_s = NL_NMAX;
  double cur_ego_l = NL_NMAX;
  double cur_ego_s = NL_NMAX;
  double boundary_point_s = NL_NMAX;
  double abs_lat_diff = 0.0;
  const auto ref_frenet_ego_state = reference_path->get_frenet_ego_state();
  ref_ego_l = ref_frenet_ego_state.l();
  ref_ego_s = ref_frenet_ego_state.s();
  const auto ref_lane_coord = reference_path->get_frenet_coord();
  const auto cur_reference_path =
      ref_path_mgr_->get_reference_path_by_current_lane();
  if (cur_reference_path == nullptr) {
    return false;
  }
  //计算当前位置的abs_lat_diff
  cur_ego_l = cur_reference_path->get_frenet_ego_state().l();
  cur_ego_s = cur_reference_path->get_frenet_ego_state().s();
  ReferencePathPoint cur_ref_path_point{};
  cur_reference_path->get_reference_point_by_lon(cur_ego_s, cur_ref_path_point);
  Point2D cur_ref_point = {cur_ref_path_point.path_point.x,
                           cur_ref_path_point.path_point.y};
  Point2D frenet_ref_point;
  if (ref_lane_coord->XYToSL(cur_ref_point, frenet_ref_point)) {
    abs_lat_diff = std::abs(frenet_ref_point.y);
  } else {
    abs_lat_diff = std::abs(ref_ego_l - cur_ego_l);
  }
  const auto& cur_lane_coord = cur_reference_path->get_frenet_coord();
  const auto cur_ref_path_end_point = cur_reference_path->get_points().back();

  //计算自车前方车道线的长度
  if (current_lane == nullptr || cur_lane_coord == nullptr) {
    return false;
  }
  const auto& cur_lane_left_boundary = current_lane->get_left_lane_boundary();
  std::shared_ptr<planning_math::KDPath> target_boundary_path =
      virtual_lane_mgr_->MakeBoundaryPath(cur_lane_left_boundary);

  if (target_boundary_path == nullptr) {
    return false;
  }
  Point2D frenet_boundary_point;
  if (target_boundary_path->XYToSL(cur_ref_point, frenet_boundary_point)) {
    boundary_point_s = frenet_boundary_point.x;
  }
  double ego_front_length = target_boundary_path->Length() - boundary_point_s;
  ego_front_length = 
      std::min(ego_front_length, cur_lane_coord->Length() - cur_ego_s);
  if (ego_front_length < kEps) {
    return false;
  }

  //计算两条车道起始点的lat_diff
  double start_abs_lat_diff = 0.0;
  Point2D frenet_point_start;
  Point2D cur_ref_path_start_point_temp = {
      cur_reference_path->get_points().begin()->path_point.x,
      cur_reference_path->get_points().begin()->path_point.y};
  Point2D ref_path_start_point_temp = {
      reference_path->get_points().begin()->path_point.x,
      reference_path->get_points().begin()->path_point.y};
  if (ref_lane_coord->XYToSL(cur_ref_path_start_point_temp,
                             frenet_point_start)) {
    start_abs_lat_diff = std::abs(frenet_point_start.y);
  } else {
    if (cur_lane_coord->XYToSL(ref_path_start_point_temp, frenet_point_start)) {
      start_abs_lat_diff = std::abs(frenet_point_start.y);
    }
  }

  //计算两条车道终点的lat_diff
  Point2D cur_ref_path_end_point_temp;
  ReferencePathPoint cur_ref_path_point_temp{};
  double target_s = cur_ego_s + ego_front_length - default_lane_buffer;
  if (cur_reference_path->get_reference_point_by_lon(target_s,
                                                     cur_ref_path_point_temp)) {
    cur_ref_path_end_point_temp = {cur_ref_path_point_temp.path_point.x,
                                   cur_ref_path_point_temp.path_point.y};
  } else {
    cur_ref_path_end_point_temp = {cur_ref_path_end_point.path_point.x,
                                   cur_ref_path_end_point.path_point.y};
  }
  Point2D ref_path_end_point_temp;
  ReferencePathPoint ref_path_point_temp{};
  target_s = std::min(target_s, reference_path->get_frenet_coord()->Length());
  if (reference_path->get_reference_point_by_lon(target_s,
                                                 ref_path_point_temp)) {
    ref_path_end_point_temp = {ref_path_point_temp.path_point.x,
                               ref_path_point_temp.path_point.y};
  } else {
    ref_path_end_point_temp = {
        reference_path->get_points().back().path_point.x,
        reference_path->get_points().back().path_point.y};
  }

  Point2D frenet_point;
  if (ref_lane_coord->XYToSL(cur_ref_path_end_point_temp, frenet_point)) {
    cur_to_other_lane_end_lat_diff = frenet_point.y;
    if (std::abs(cur_to_other_lane_end_lat_diff) <
        end_lane_lat_diff_threshold) {
      lane_end_satisfied_merge_condition = true;
    }
  } else {
    if (cur_lane_coord->XYToSL(ref_path_end_point_temp, frenet_point)) {
      cur_to_other_lane_end_lat_diff = frenet_point.y;
      if (std::abs(cur_to_other_lane_end_lat_diff) <
          end_lane_lat_diff_threshold) {
        lane_end_satisfied_merge_condition = true;
      }
    }
  }
  if ((abs_lat_diff > cur_lane_lat_diff_threshold ||
       start_abs_lat_diff > cur_lane_lat_diff_threshold) &&
      lane_end_satisfied_merge_condition) {
    std::cout << "is merge region!!!" << std::endl;
    return true;
  }
  // 遍历自车向前的点，是否有overlap情况
  const double step_length = 5.0;
  const double buffer = 1.0;
  const int calculate_nums = (int)(ego_front_length / step_length - buffer);
  for (int i = 1; i <= calculate_nums; i++) {
    ReferencePathPoint cur_ref_path_point_temp{};
    Point2D frenet_point_temp;
    if (!cur_reference_path->get_reference_point_by_lon(
            cur_ego_s + i * 5, cur_ref_path_point_temp)) {
      continue;
    }
    Point2D cur_point_temp = {cur_ref_path_point_temp.path_point.x,
                              cur_ref_path_point_temp.path_point.y};
    if (ref_lane_coord->XYToSL(cur_point_temp, frenet_point_temp)) {
      double cur_point_to_other_lane_lat_diff = frenet_point_temp.y;
      if (std::abs(cur_point_to_other_lane_lat_diff) <
          end_lane_lat_diff_threshold) {
        lane_end_satisfied_merge_condition = true;
        break;
      }
    }
  }
  if ((abs_lat_diff > cur_lane_lat_diff_threshold ||
       start_abs_lat_diff > cur_lane_lat_diff_threshold) &&
      lane_end_satisfied_merge_condition) {
    std::cout << "is merge region!!!" << std::endl;
    return true;
  }
  return false;
}

void EgoLaneRoadRightDecider::CalculateMergePoint(
    std::vector<Point2D>& merge_point_list, int* calculate_nums) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  Point2D merge_point = {ego_state->planning_init_point().x,
                         ego_state->planning_init_point().y};
  Point2D boundary_line_merge_point = {ego_state->planning_init_point().x,
                                       ego_state->planning_init_point().y};
  (merge_point_list)[0] = merge_point;
  (merge_point_list)[1] = boundary_line_merge_point;
  const auto& overlap_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(merge_lane_virtual_id_);
  if (!overlap_lane) {
    return;
  }
  const auto& cur_path = ref_path_mgr_->get_reference_path_by_current_lane();
  if (!cur_path) {
    return;
  }
  const auto& overlap_path =
      ref_path_mgr_->get_reference_path_by_lane(merge_lane_virtual_id_);
  if (!overlap_path) {
    return;
  }
  const auto& overlap_path_coordinate = overlap_path->get_frenet_coord();
  const double ego_front_line_length = CalculateEgoFrontLineLength();
  if (ego_front_line_length < kEps) {
    return;
  }
  const double cur_ego_s = cur_path->get_frenet_ego_state().s();
  const double ego_front_center_line_length =
      cur_path->get_frenet_coord()->Length() - cur_ego_s;
  const double buffer = 1.0;
  const double need_judgement_length = std::max(
      0.0,
      std::min(ego_front_line_length, ego_front_center_line_length) - buffer);
  const double step_length = 1.0;
  const double num = std::max(need_judgement_length / step_length, 0.0);
  bool is_find_merge_point = false;
  bool is_find_boundary_merge_point = false;
  const double standard_lane_width = 3.8;
  const double lat_err = standard_lane_width / 4;
  for (double i = 0; i < num; i++) {
    ReferencePathPoint cur_ref_path_point_temp{};
    if (!cur_path->get_reference_point_by_lon(cur_ego_s + i * step_length,
                                              cur_ref_path_point_temp)) {
      continue;
    }
    Point2D frenet_point;
    Point2D projection_point = {cur_ref_path_point_temp.path_point.x,
                                cur_ref_path_point_temp.path_point.y};
    if (!overlap_path_coordinate->XYToSL(projection_point, frenet_point)) {
      continue;
    }
    double lat_diff = std::abs(frenet_point.y);
    const double overlap_lane_width =
        overlap_lane->width_by_s(cur_ego_s + i * step_length);
    if (lat_diff < overlap_lane_width / 2 && !is_find_boundary_merge_point) {
      boundary_line_merge_point = projection_point;
      is_find_boundary_merge_point = true;
    }
    if (lat_diff < lat_err) {
      // last_point_lat_diff = lat_diff;
      merge_point = projection_point;
      is_find_merge_point = true;
      *calculate_nums = i;
      break;
    }
    //处理遍历完，没有找到merge point的情况
    if (i + 1 > num && !is_find_merge_point) {
      ReferencePathPoint cur_ref_path_point_temp{};
      if (cur_path->get_reference_point_by_lon(
              cur_ego_s + need_judgement_length, cur_ref_path_point_temp)) {
        merge_point.x = cur_ref_path_point_temp.path_point.x;
        merge_point.y = cur_ref_path_point_temp.path_point.y;
      }
      *calculate_nums = i;
    }
  }
  (merge_point_list)[0] = merge_point;
  (merge_point_list)[1] = boundary_line_merge_point;
}

void EgoLaneRoadRightDecider::CalculateRoadRight(const int calculate_nums) {
  const auto& left_lane = virtual_lane_mgr_->get_left_lane();
  const auto& right_lane = virtual_lane_mgr_->get_right_lane();
  const auto& overlap_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(merge_lane_virtual_id_);
  if (!overlap_lane) {
    return;
  }
  const auto& cur_path = ref_path_mgr_->get_reference_path_by_current_lane();
  if (!cur_path) {
    return;
  }
  const auto& overlap_path =
      ref_path_mgr_->get_reference_path_by_lane(merge_lane_virtual_id_);
  if (!overlap_path) {
    return;
  }

  // 利用地面标识判断路权
  ComputeRoadRightFromLaneMark();
  if (is_right_merge_direction_ || is_left_merge_direction_) {
    cur_lane_is_continue_ = false;
    return;
  }

  // 利用虚拟车道判断路权
  if (ego_lane_boundary_exist_virtual_line_ &&
      !target_lane_boundary_exist_virtual_line_ && !is_split_region_) {
    cur_lane_is_continue_ = false;
    return;
  }

  //针对向左侧汇流 利用曲率判断路权
  if (left_lane != nullptr &&
      merge_lane_virtual_id_ == left_lane->get_virtual_id()) {
    const double cur_ego_s = cur_path->get_frenet_ego_state().s();
    const double overlap_ego_s = overlap_path->get_frenet_ego_state().s();
    const double step_length = 1.0;
    std::vector<planning_math::PathPoint> cur_path_points;
    std::vector<planning_math::PathPoint> overlap_path_points;
    for (int i = 0; i <= calculate_nums; i++) {
      ReferencePathPoint cur_ref_path_point_temp{};
      if (!cur_path->get_reference_point_by_lon(cur_ego_s + i * step_length,
                                                cur_ref_path_point_temp)) {
        continue;
      }
      ReferencePathPoint over_ref_path_point_temp{};
      if (!overlap_path->get_reference_point_by_lon(
              overlap_ego_s + i * step_length, over_ref_path_point_temp)) {
        continue;
      }
      auto overlap_pt =
          planning_math::PathPoint(over_ref_path_point_temp.path_point.x,
                                   over_ref_path_point_temp.path_point.y);
      auto cur_pt =
          planning_math::PathPoint(cur_ref_path_point_temp.path_point.x,
                                   cur_ref_path_point_temp.path_point.y);
      overlap_path_points.emplace_back(overlap_pt);
      cur_path_points.emplace_back(cur_pt);
    }
    if (cur_path_points.size() < 3 || overlap_path_points.size() < 3) {
      return;
    }

    std::shared_ptr<planning_math::KDPath> cur_kd_path =
        std::make_shared<planning_math::KDPath>(std::move(cur_path_points));
    std::shared_ptr<planning_math::KDPath> over_kd_path =
        std::make_shared<planning_math::KDPath>(std::move(overlap_path_points));
    const double cur_average_kappa = CalculateAverageKappa(cur_kd_path);
    const double overlap_average_kappa = CalculateAverageKappa(over_kd_path);

    if (cur_average_kappa > overlap_average_kappa) {
      cur_lane_is_continue_ = false;
    } else {
      cur_lane_is_continue_ = true;
    }
  }
  return;
}

bool EgoLaneRoadRightDecider::IsVirtualLaneLine(const int lane_virtual_id) {
  const auto base_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(lane_virtual_id);
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();
  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  std::shared_ptr<planning_math::KDPath> left_base_boundary_path;
  std::shared_ptr<planning_math::KDPath> right_base_boundary_path;
  if (!base_lane) {
    return false;
  }

  double left_lane_line_length = 0.0;
  int left_current_segment_count = 0;
  double left_ego_s = 0.0, left_ego_l = 0.0;
  // 判断左侧车道线类型
  auto left_lane_boundarys = base_lane->get_left_lane_boundary();
  left_base_boundary_path =
      virtual_lane_mgr_->MakeBoundaryPath(left_lane_boundarys);
  if (left_base_boundary_path != nullptr) {
    if (!left_base_boundary_path->XYToSL(ego_x, ego_y, &left_ego_s,
                                         &left_ego_l)) {
      return false;
    }
  } else {
    return false;
  }
  for (int i = 0; i < left_lane_boundarys.type_segments_size; i++) {
    left_lane_line_length += left_lane_boundarys.type_segments[i].length;
    if (left_lane_line_length > left_ego_s) {
      left_current_segment_count = i;
      break;
    }
  }
  for (int i = left_current_segment_count;
       i < left_lane_boundarys.type_segments_size; i++) {
    if (left_lane_boundarys.type_segments[i].type ==
        iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
      return true;
    }
  }

  // 判断右侧车道线类型
  double right_lane_line_length = 0.0;
  int right_current_segment_count = 0;
  double right_ego_s = 0.0, right_ego_l = 0.0;
  auto right_lane_boundarys = base_lane->get_right_lane_boundary();
  right_base_boundary_path =
      virtual_lane_mgr_->MakeBoundaryPath(right_lane_boundarys);
  if (right_base_boundary_path != nullptr) {
    if (!right_base_boundary_path->XYToSL(ego_x, ego_y, &right_ego_s,
                                          &right_ego_l)) {
      return false;
    }
  } else {
    return false;
  }
  for (int i = 0; i < right_lane_boundarys.type_segments_size; i++) {
    right_lane_line_length += right_lane_boundarys.type_segments[i].length;
    if (right_lane_line_length > right_ego_s) {
      right_current_segment_count = i;
      break;
    }
  }
  for (int i = right_current_segment_count;
       i < right_lane_boundarys.type_segments_size; i++) {
    if (right_lane_boundarys.type_segments[i].type ==
        iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
      return true;
    }
  }
  return false;
}

const double EgoLaneRoadRightDecider::CalculateEgoFrontLineLength() {
  const double default_lane_line_length = -1.0;
  const auto& cur_lane = virtual_lane_mgr_->get_current_lane();
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();
  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  double ego_s = 0.0, ego_l = 0.0;
  const auto& llane = virtual_lane_mgr_->get_left_lane();
  const auto& rlane = virtual_lane_mgr_->get_right_lane();
  std::shared_ptr<planning_math::KDPath> target_boundary_path;

  if (llane != nullptr && merge_lane_virtual_id_ == llane->get_virtual_id()) {
    const auto& cur_lane_left_boundary = cur_lane->get_left_lane_boundary();
    target_boundary_path =
        virtual_lane_mgr_->MakeBoundaryPath(cur_lane_left_boundary);
  } else if (rlane != nullptr &&
             merge_lane_virtual_id_ == rlane->get_virtual_id()) {
    const auto& cur_lane_right_boundary = cur_lane->get_right_lane_boundary();
    target_boundary_path =
        virtual_lane_mgr_->MakeBoundaryPath(cur_lane_right_boundary);
  } else {
    return default_lane_line_length;
  }

  if (target_boundary_path != nullptr) {
    if (!target_boundary_path->XYToSL(ego_x, ego_y, &ego_s, &ego_l)) {
      return default_lane_line_length;
    }
  } else {
    return default_lane_line_length;
  }
  double ego_front_length = target_boundary_path->Length() - ego_s;
  return ego_front_length;
}

const double EgoLaneRoadRightDecider::CalculateAverageKappa(
    const std::shared_ptr<planning_math::KDPath> kd_path) {
  if (!kd_path) {
    return -1;
  }
  const int size_num = kd_path->path_points().size();
  if (size_num == 0) {
    return 0.0;
  }
  double sum_kappa = 0.0;
  for (int i = 0; i < size_num; i++) {
    sum_kappa = sum_kappa + std::abs(kd_path->path_points()[i].kappa());
  }
  return sum_kappa / size_num;
}

void EgoLaneRoadRightDecider::ComputeIsSplitRegion() {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  double ego_vel = ego_state->ego_v();
  // if (ego_vel < kApprochingRampSpeedThd) {
  //   return;
  // }
  Point2D ego_point = {ego_state->planning_init_point().x,
                       ego_state->planning_init_point().y};
  const auto& llane = virtual_lane_mgr_->get_left_lane();
  const auto& rlane = virtual_lane_mgr_->get_right_lane();
  const auto& current_lane = virtual_lane_mgr_->get_current_lane();
  bool has_left_lane = false;
  bool has_ego_lane = false;
  bool has_right_lane = false;
  if (current_lane != nullptr) {
    has_ego_lane = true;
  }
  const auto& current_lane_points = current_lane->lane_points();

  if (llane != nullptr) {
    if (llane->get_lane_frenet_coord() != nullptr) {
      has_left_lane = true;
    }
  }
  if (rlane != nullptr) {
    if (rlane->get_lane_frenet_coord() != nullptr) {
      has_right_lane = true;
    }
  }

  if (!has_ego_lane || (!has_left_lane && !has_right_lane) ||
      current_lane_points.size() < 3) {
    return;
  }

  if (has_left_lane) {
    const auto& left_lane_frenet_crd = llane->get_lane_frenet_coord();
    double ego_s_base_left = 0.0;
    double ego_l_base_left = 0.0;
    left_lane_frenet_crd->XYToSL(ego_point.x, ego_point.y, &ego_s_base_left,
                                 &ego_l_base_left);
    double near_average_l = 0.0;
    double far_average_l = 0.0;
    int32_t near_pt_count = 0;
    int32_t far_pt_count = 0;
    double near_pt_sum_l = 0.0;
    double far_pt_sum_l = 0.0;
    for (const auto point : current_lane_points) {
      if (std::isnan(point.local_point.x) || std::isnan(point.local_point.y)) {
        LOG_ERROR("update_lane_points: skip NaN point");
        continue;
      }
      double pt_s = 0.0;
      double pt_l = 0.0;
      left_lane_frenet_crd->XYToSL(point.local_point.x, point.local_point.y,
                                   &pt_s, &pt_l);
      const double pt_distance_to_ego = pt_s - ego_s_base_left;
      if (pt_l > 0.0 || pt_s > left_lane_frenet_crd->Length()) {
        continue;
      }
      if (pt_distance_to_ego > -kNearPreviewDistanceThd &&
          pt_distance_to_ego < 0.0) {
        ++near_pt_count;
        near_pt_sum_l += pt_l;
      }
      if (pt_distance_to_ego > kEgoPreviewTimeMinThd * ego_vel &&
          pt_distance_to_ego < kEgoPreviewTimeMaxThd * ego_vel) {
        ++far_pt_count;
        far_pt_sum_l += pt_l;
      }
    }
    near_pt_count = std::max(near_pt_count, 1);
    far_pt_count = std::max(far_pt_count, 1);

    near_average_l = std::fabs(near_pt_sum_l / near_pt_count);
    far_average_l = std::fabs(far_pt_sum_l / far_pt_count);

    if ((far_average_l - kExistSplitLateralDisThd > near_average_l) ||
        near_average_l < kCenterLineLateralDisThd) {
      is_split_region_ = true;
      return;
    }
  }

  if (has_right_lane) {
    const auto& right_lane_frenet_crd = rlane->get_lane_frenet_coord();
    double ego_s_base_right = 0.0;
    double ego_l_base_right = 0.0;
    right_lane_frenet_crd->XYToSL(ego_point.x, ego_point.y, &ego_s_base_right,
                                  &ego_l_base_right);
    double near_average_l = 0.0;
    double far_average_l = 0.0;
    int32_t near_pt_count = 0;
    int32_t far_pt_count = 0;
    double near_pt_sum_l = 0.0;
    double far_pt_sum_l = 0.0;
    for (const auto point : current_lane_points) {
      if (std::isnan(point.local_point.x) || std::isnan(point.local_point.y)) {
        LOG_ERROR("update_lane_points: skip NaN point");
        continue;
      }
      double pt_s = 0.0;
      double pt_l = 0.0;
      right_lane_frenet_crd->XYToSL(point.local_point.x, point.local_point.y,
                                    &pt_s, &pt_l);
      const double pt_distance_to_ego = pt_s - ego_s_base_right;
      if (pt_l < 0.0 || pt_s > right_lane_frenet_crd->Length()) {
        continue;
      }
      if (pt_distance_to_ego > -kNearPreviewDistanceThd &&
          pt_distance_to_ego < 0.0) {
        ++near_pt_count;
        near_pt_sum_l += pt_l;
      }
      if (pt_distance_to_ego > kEgoPreviewTimeMinThd * ego_vel &&
          pt_distance_to_ego < kEgoPreviewTimeMaxThd * ego_vel) {
        ++far_pt_count;
        far_pt_sum_l += pt_l;
      }
    }
    near_pt_count = std::max(near_pt_count, 1);
    far_pt_count = std::max(far_pt_count, 1);

    near_average_l = std::fabs(near_pt_sum_l / near_pt_count);
    far_average_l = std::fabs(far_pt_sum_l / far_pt_count);

    if ((far_average_l - kExistSplitLateralDisThd > near_average_l) ||
        near_average_l < kCenterLineLateralDisThd) {
      is_split_region_ = true;
      return;
    }
  }
  return;
}

void EgoLaneRoadRightDecider::CheckIfMergeWithLeftLane() {
  const auto base_lane = virtual_lane_mgr_->get_current_lane();
  const double default_consider_lane_marks_length = 80.0;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();
  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  Point2D ego_cart_point{plannig_init_point.lat_init_state.x(),
                         plannig_init_point.lat_init_state.y()};

  bool left_boundary_exist_virtual_type = false;
  bool right_boundary_exist_virtual_type = false;
  bool target_left_boundary_exist_virtual_type = false;
  bool target_right_boundary_exist_virtual_type = false;
  const auto& llane = virtual_lane_mgr_->get_left_lane();
  const auto& rlane = virtual_lane_mgr_->get_right_lane();
  const int lane_nums = virtual_lane_mgr_->get_lane_num();
  const int current_lane_order_id = base_lane->get_order_id();
  bool is_left_edge_side_lane = current_lane_order_id == 0;
  bool is_right_edge_side_lane = current_lane_order_id == lane_nums - 1;
  bool exist_left_direction_merge = false;
  bool exist_right_direction_merge = false;

  std::shared_ptr<planning_math::KDPath> left_base_boundary_path;
  std::shared_ptr<planning_math::KDPath> right_base_boundary_path;

  if (base_lane != nullptr) {
    double left_lane_line_length = 0.0;
    int left_current_segment_count = 0;
    double left_ego_s = 0.0, left_ego_l = 0.0;
    // 判断左侧车道线类型
    auto left_lane_boundarys = base_lane->get_left_lane_boundary();
    left_base_boundary_path =
        virtual_lane_mgr_->MakeBoundaryPath(left_lane_boundarys);
    if (left_base_boundary_path != nullptr) {
      if (!left_base_boundary_path->XYToSL(ego_x, ego_y, &left_ego_s,
                                           &left_ego_l)) {
        return;
      }
    } else {
      return;
    }
    for (int i = 0; i < left_lane_boundarys.type_segments_size; i++) {
      left_lane_line_length += left_lane_boundarys.type_segments[i].length;
      if (left_lane_line_length > left_ego_s) {
        left_current_segment_count = i;
        break;
      }
    }
    for (int i = left_current_segment_count;
         i < left_lane_boundarys.type_segments_size; i++) {
      if (left_lane_boundarys.type_segments[i].type ==
          iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
        left_boundary_exist_virtual_type = true;
        break;
      } else {
        continue;
      }
    }

    if (llane != nullptr) {
      double target_right_lane_line_length = 0.0;
      int target_right_current_segment_count = 0;
      double right_ego_s = 0.0, right_ego_l = 0.0;
      auto right_lane_boundarys = llane->get_right_lane_boundary();
      right_base_boundary_path =
          virtual_lane_mgr_->MakeBoundaryPath(right_lane_boundarys);
      if (right_base_boundary_path != nullptr) {
        if (!right_base_boundary_path->XYToSL(ego_x, ego_y, &right_ego_s,
                                              &right_ego_l)) {
          right_ego_s = 50.0;
        }

        for (int i = 0; i < right_lane_boundarys.type_segments_size; i++) {
          target_right_lane_line_length +=
              right_lane_boundarys.type_segments[i].length;
          if (target_right_lane_line_length > right_ego_s) {
            target_right_current_segment_count = i;
            break;
          }
        }
        for (int i = target_right_current_segment_count;
             i < right_lane_boundarys.type_segments_size; i++) {
          if (right_lane_boundarys.type_segments[i].type ==
              iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
            target_right_boundary_exist_virtual_type = true;
            break;
          } else {
            continue;
          }
        }
      }
      if (left_boundary_exist_virtual_type &&
          !target_right_boundary_exist_virtual_type) {
        ego_lane_boundary_exist_virtual_line_ = true;
        target_lane_boundary_exist_virtual_line_ = false;
      } else if (!left_boundary_exist_virtual_type &&
                 target_right_boundary_exist_virtual_type) {
        ego_lane_boundary_exist_virtual_line_ = false;
        target_lane_boundary_exist_virtual_line_ = true;
      } else {
        return;
      }
    }
  }
}

void EgoLaneRoadRightDecider::CheckIfMergeWithRightLane() {
  const auto base_lane = virtual_lane_mgr_->get_current_lane();
  const double default_consider_lane_marks_length = 80.0;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();
  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  Point2D ego_cart_point{plannig_init_point.lat_init_state.x(),
                         plannig_init_point.lat_init_state.y()};

  bool left_boundary_exist_virtual_type = false;
  bool right_boundary_exist_virtual_type = false;
  bool target_left_boundary_exist_virtual_type = false;
  bool target_right_boundary_exist_virtual_type = false;
  const auto& llane = virtual_lane_mgr_->get_left_lane();
  const auto& rlane = virtual_lane_mgr_->get_right_lane();
  const int lane_nums = virtual_lane_mgr_->get_lane_num();
  const int current_lane_order_id = base_lane->get_order_id();
  bool is_left_edge_side_lane = current_lane_order_id == 0;
  bool is_right_edge_side_lane = current_lane_order_id == lane_nums - 1;
  bool exist_left_direction_merge = false;
  bool exist_right_direction_merge = false;

  std::shared_ptr<planning_math::KDPath> left_base_boundary_path;
  std::shared_ptr<planning_math::KDPath> right_base_boundary_path;

  if (base_lane != nullptr) {
    double right_lane_line_length = 0.0;
    int right_current_segment_count = 0;
    double right_ego_s = 0.0, right_ego_l = 0.0;
    auto right_lane_boundarys = base_lane->get_right_lane_boundary();
    right_base_boundary_path =
        virtual_lane_mgr_->MakeBoundaryPath(right_lane_boundarys);
    if (right_base_boundary_path != nullptr) {
      if (!right_base_boundary_path->XYToSL(ego_x, ego_y, &right_ego_s,
                                            &right_ego_l)) {
        return;
      }
    } else {
      return;
    }
    for (int i = 0; i < right_lane_boundarys.type_segments_size; i++) {
      right_lane_line_length += right_lane_boundarys.type_segments[i].length;
      if (right_lane_line_length > right_ego_s) {
        right_current_segment_count = i;
        break;
      }
    }
    for (int i = right_current_segment_count;
         i < right_lane_boundarys.type_segments_size; i++) {
      if (right_lane_boundarys.type_segments[i].type ==
          iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
        right_boundary_exist_virtual_type = true;
        break;
      } else {
        continue;
      }
    }

    if (rlane != nullptr) {
      double target_left_lane_line_length = 0.0;
      int target_left_current_segment_count = 0;
      double left_ego_s = 0.0, left_ego_l = 0.0;
      // 判断左侧车道线类型
      auto left_lane_boundarys = rlane->get_left_lane_boundary();
      left_base_boundary_path =
          virtual_lane_mgr_->MakeBoundaryPath(left_lane_boundarys);
      if (left_base_boundary_path != nullptr) {
        if (!left_base_boundary_path->XYToSL(ego_x, ego_y, &left_ego_s,
                                             &left_ego_l)) {
          left_ego_s = 50.0;
        }

        for (int i = 0; i < left_lane_boundarys.type_segments_size; i++) {
          target_left_lane_line_length +=
              left_lane_boundarys.type_segments[i].length;
          if (target_left_lane_line_length > left_ego_s) {
            target_left_current_segment_count = i;
            break;
          }
        }
        for (int i = target_left_current_segment_count;
             i < left_lane_boundarys.type_segments_size; i++) {
          if (left_lane_boundarys.type_segments[i].type ==
              iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
            target_left_boundary_exist_virtual_type = true;
            break;
          } else {
            continue;
          }
        }
      }
      if (right_boundary_exist_virtual_type &&
          !target_left_boundary_exist_virtual_type) {
        ego_lane_boundary_exist_virtual_line_ = true;
        target_lane_boundary_exist_virtual_line_ = false;
      } else if (!right_boundary_exist_virtual_type &&
                 target_left_boundary_exist_virtual_type) {
        ego_lane_boundary_exist_virtual_line_ = false;
        target_lane_boundary_exist_virtual_line_ = true;
      } else {
        return;
      }
    }
  }
}

void EgoLaneRoadRightDecider::ComputeRoadRightFromLaneMark() {
  const auto base_lane = virtual_lane_mgr_->get_current_lane();
  const double default_consider_lane_marks_length = 80.0;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();
  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  Point2D ego_cart_point{plannig_init_point.lat_init_state.x(),
                         plannig_init_point.lat_init_state.y()};
  is_left_merge_direction_ = false;
  is_right_merge_direction_ = false;

  const auto& llane = virtual_lane_mgr_->get_left_lane();
  const auto& rlane = virtual_lane_mgr_->get_right_lane();
  const int lane_nums = virtual_lane_mgr_->get_lane_num();
  const int current_lane_order_id = base_lane->get_order_id();
  bool is_left_edge_side_lane = current_lane_order_id == 0;
  bool is_right_edge_side_lane = current_lane_order_id == lane_nums - 1;
  bool exist_left_direction_merge = false;
  bool exist_right_direction_merge = false;

  std::shared_ptr<planning_math::KDPath> left_base_boundary_path;
  std::shared_ptr<planning_math::KDPath> right_base_boundary_path;

  if (base_lane != nullptr) {
    std::shared_ptr<KDPath> base_lane_frenet_crd =
        base_lane->get_lane_frenet_coord();
    int segment = -1;
    double ego_s = 0.0;
    if (base_lane_frenet_crd != nullptr) {
      Point2D ego_cart_frenet_point;
      if (!base_lane_frenet_crd->XYToSL(ego_cart_point,
                                        ego_cart_frenet_point)) {
        ego_s = 0.0;
      } else {
        ego_s = ego_cart_frenet_point.x;
      }
      std::vector<iflyauto::LaneMarkMsg> lane_marks = base_lane->lane_marks();
      double lane_line_length = 0.0;
      for (int i = 0; i < lane_marks.size(); i++) {
        lane_line_length = lane_marks[i].end;
        if (lane_line_length > ego_s && lane_marks[i].begin <= ego_s) {
          segment = i;
          break;
        } else {
          continue;
        }
      }
    }

    // 根据地面标识判断是否向左汇流
    if (segment >= 0 && ego_s != 0.0) {
      std::vector<iflyauto::LaneMarkMsg> lane_marks = base_lane->lane_marks();

      for (int i = segment; i < lane_marks.size(); i++) {
        if (lane_marks[i].begin > ego_s + default_consider_lane_marks_length) {
          break;
        }
        if (lane_marks[i].lane_mark ==
            iflyauto::LaneDrivableDirection_DIRECTION_LEFT_MERGE) {
          exist_left_direction_merge = true;
          break;
        }
      }
    }

    //根据地面标识判断是否向右汇流
    if (segment >= 0 && ego_s != 0.0) {
      std::vector<iflyauto::LaneMarkMsg> lane_marks = base_lane->lane_marks();

      for (int i = segment; i < lane_marks.size(); i++) {
        if (lane_marks[i].begin > ego_s + default_consider_lane_marks_length) {
          break;
        }
        if (lane_marks[i].lane_mark ==
            iflyauto::LaneDrivableDirection_DIRECTION_RIGHT_MERGE) {
          exist_right_direction_merge = true;
          break;
        }
      }
    }

    if (is_right_edge_side_lane && !is_split_region_) {
      if (exist_left_direction_merge) {
        is_left_merge_direction_ = true;
      }
    }
    if (is_left_edge_side_lane && !is_split_region_) {
      if (exist_right_direction_merge) {
        is_right_merge_direction_ = true;
      }
    }
  } else {
    return;
  }
}

}  // namespace planning
