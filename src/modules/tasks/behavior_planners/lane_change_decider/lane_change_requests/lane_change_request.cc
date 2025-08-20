#include "lane_change_request.h"

#include <sys/param.h>

#include <algorithm>
#include <cmath>

#include "common_c.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "lateral_obstacle.h"
#include "obstacle_manager.h"
#include "planning_context.h"
#include "task_interface/lane_change_utils.h"
#include "utils/lateral_utils.h"

namespace planning {
namespace {
constexpr double kLargeAgentLengthM = 8.0;
constexpr double kInputBoundaryLenLimit = 145.;
constexpr double kDefaultBoundaryLen = 5000.;
// https://yf2ljykclb.xfchat.iflytek.com/wiki/MXjXwlCjni6g7nkjgKGrfwGwzPb
constexpr double kLaneChangeSolidLineTTC = 1.75;  // todo(ldh): 从配置中读取
constexpr double kIgnoreLineTypeThreshold = 0.33333333333;
}  // namespace
LaneChangeRequest::LaneChangeRequest(
    framework::Session *session,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : session_(session),
      virtual_lane_mgr_(virtual_lane_mgr),
      lane_change_lane_mgr_(lane_change_lane_mgr) {}

void LaneChangeRequest::GenerateRequest(RequestType direction) {
  if (direction != LEFT_CHANGE && direction != RIGHT_CHANGE) {
    ILOG_DEBUG << "[LaneChangeRequest::GenerateRequest] Illgeal direction[" << direction << "]";
  }
  if (request_type_ == direction) {
    ILOG_DEBUG << "[LaneChangeRequest::GenerateRequest] duplicated request, direction :" << direction;
    return;
  }
  request_type_ = direction;
  turn_signal_ = direction;
  tstart_ = IflyTime::Now_s();
}

void LaneChangeRequest::Finish() {
  if (request_type_ == NO_CHANGE) {
    ILOG_DEBUG << "[LaneChangeRequest::Finish] No request to finish";
    turn_signal_ = NO_CHANGE;
    return;
  }

  request_type_ = NO_CHANGE;
  turn_signal_ = NO_CHANGE;
  tfinish_ = IflyTime::Now_s();
}

bool LaneChangeRequest::AggressiveChange() const {
  auto origin_lane = lane_change_lane_mgr_->has_origin_lane()
                         ? lane_change_lane_mgr_->olane()
                         : virtual_lane_mgr_->get_current_lane();
  auto lc_map_decision =
      0;  // hack
          // origin_lane != nullptr ? origin_lane->lc_map_decision() : 0;
  auto aggressive_change_distance = 200.0;  // WB: hack
  // virtual_lane_mgr_->is_on_highway()
  //     ? aggressive_lane_change_distance_highway_
  //     : aggressive_lane_change_distance_urban_;

  // auto aggressive_change =
  //     origin_lane != nullptr
  //         ? origin_lane->must_change_lane(aggressive_change_distance *
  //                                         std::fabs(lc_map_decision))
  //         : false;
  auto aggressive_change =
      origin_lane != nullptr ? virtual_lane_mgr_->must_change_lane(
                                   origin_lane, aggressive_change_distance *
                                                    std::fabs(lc_map_decision))
                             : false;
  return aggressive_change && (request_type_ != NO_CHANGE);
}

double LaneChangeRequest::CalculatePressLineRatio(
    const int origin_lane_id, const RequestType &lc_request) const {
  double press_line_ratio = 0.0;

  // press_line_ratio = target_lane->press_line_ratio();
  auto origin_reference_path =
      session_->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(origin_lane_id, false);
  auto origin_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(origin_lane_id);

  if (nullptr == origin_lane || origin_reference_path == nullptr) {
    return 0.0;
  }
  const auto &ego_frenent_boundary =
      origin_reference_path->get_ego_frenet_boundary();
  const double ego_center_s =
      ego_frenent_boundary.s_start +
      (ego_frenent_boundary.s_end - ego_frenent_boundary.s_start) / 2.0;
  const double frenent_ego_width =
      ego_frenent_boundary.l_end - ego_frenent_boundary.l_start;
  double lane_width = origin_lane->width_by_s(ego_center_s);
  double half_lane_width = lane_width * 0.5;
  if (lc_request == LEFT_CHANGE) {
    if (ego_frenent_boundary.l_end < half_lane_width) {
      press_line_ratio = 0.0;
    } else if (ego_frenent_boundary.l_end > half_lane_width) {
      press_line_ratio =
          (ego_frenent_boundary.l_end - half_lane_width) / frenent_ego_width;
    } else {
      press_line_ratio = 0.0;
    }
  } else if (lc_request == RIGHT_CHANGE) {
    if (ego_frenent_boundary.l_start > -half_lane_width) {
      press_line_ratio = 0.0;
    } else if (ego_frenent_boundary.l_start < -half_lane_width) {
      press_line_ratio =
          (-ego_frenent_boundary.l_start + half_lane_width) / frenent_ego_width;
    } else {
      press_line_ratio = 0.0;
    }
  } else {
    press_line_ratio = 0.0;
  }
  return press_line_ratio;
}

bool LaneChangeRequest::IsDashedLineEnough(
    RequestType direction, const double ego_vel,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    const StateMachineLaneChangeStatus &lc_status) {
  ILOG_DEBUG << "dashed_enough: direction:" << direction;
  ILOG_DEBUG << "dashed_enough: vel: " << ego_vel;

  double dash_length = 80;

  double right_dash_line_len = virtual_lane_mgr->get_distance_to_dash_line(
      RIGHT_CHANGE, origin_lane_virtual_id_);
  double left_dash_line_len = virtual_lane_mgr->get_distance_to_dash_line(
      LEFT_CHANGE, origin_lane_virtual_id_);
  ILOG_DEBUG << "dashed_enough: right_dash_line_len:" << right_dash_line_len;
  ILOG_DEBUG << "dashed_enough: left_dash_line_len:" << left_dash_line_len;
  std::cout << "origin_lane_virtual_id_: " << origin_lane_virtual_id_
            << "origin_lane_order_id_: " << origin_lane_virtual_id_
            << std::endl;
  const auto &route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();
  double press_line_ratio =
      CalculatePressLineRatio(origin_lane_virtual_id_, direction);
  // HACK RUI

  if (route_info_output.dis_to_ramp < 500.) return true;
  if (direction == LEFT_CHANGE) {
    if (left_dash_line_len > ego_vel * kLaneChangeSolidLineTTC) {
      return true;
    } else {
      if (lc_status == StateMachineLaneChangeStatus::kLaneChangeExecution &&
          press_line_ratio >= kIgnoreLineTypeThreshold) {
        return true;
      } else {
        dash_length = right_dash_line_len;
      }
    }
  } else if (direction == RIGHT_CHANGE) {
    if (right_dash_line_len > ego_vel * kLaneChangeSolidLineTTC) {
      return true;
    } else {
      if (lc_status == StateMachineLaneChangeStatus::kLaneChangeExecution &&
          press_line_ratio >= kIgnoreLineTypeThreshold) {
        return true;
      } else {
        dash_length = right_dash_line_len;
      }
    }
  }

  dash_length = (dash_length > kInputBoundaryLenLimit) ? kDefaultBoundaryLen
                                                       : dash_length;
  double error_buffer = std::fmin(ego_vel * 0.5, 5);
  dash_length -= error_buffer;

  double v_target =
      session_->environmental_model().get_ego_state_manager()->ego_v_cruise();

  double distance_thld = std::max(v_target, ego_vel) * 4.0;
  // bool must_change_lane =
  //     virtual_lane_mgr->get_current_lane()->must_change_lane(distance_thld);
  auto current_lane = virtual_lane_mgr_->get_current_lane();
  auto must_change_lane =
      current_lane != nullptr
          ? virtual_lane_mgr_->must_change_lane(current_lane, distance_thld)
          : false;
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double max_front_wheel_angle = vehicle_param.max_front_wheel_angle;
  const double wheel_base = vehicle_param.wheel_base;
  if (!must_change_lane &&
      cal_lat_offset(ego_vel, dash_length, max_front_wheel_angle, wheel_base) <
          3.6) {
    ILOG_ERROR << "!dashed_enough";
    return false;
  }

  return true;
}
bool LaneChangeRequest::ComputeLcValid(RequestType direction) {
  // 根据方向获取目标车道的障碍物
  const auto &dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const planning_data::DynamicAgentNode *target_lane_front_node = nullptr;
  const planning_data::DynamicAgentNode *target_lane_middle_node = nullptr;
  const planning_data::DynamicAgentNode *target_lane_rear_node = nullptr;
  int64_t target_lane_front_node_id = planning_data::kInvalidId;
  int64_t target_lane_middle_node_id = planning_data::kInvalidId;
  int64_t target_lane_rear_node_id = planning_data::kInvalidId;
  if (direction == LEFT_CHANGE) {
    target_lane_front_node_id = dynamic_world->ego_left_front_node_id();
    target_lane_middle_node_id = dynamic_world->ego_left_node_id();
    target_lane_rear_node_id = dynamic_world->ego_left_rear_node_id();
  } else if (direction == RIGHT_CHANGE) {
    target_lane_front_node_id = dynamic_world->ego_right_front_node_id();
    target_lane_middle_node_id = dynamic_world->ego_right_node_id();
    target_lane_rear_node_id = dynamic_world->ego_right_rear_node_id();
  }
  target_lane_front_node = dynamic_world->GetNode(target_lane_front_node_id);
  target_lane_middle_node = dynamic_world->GetNode(target_lane_middle_node_id);
  target_lane_rear_node = dynamic_world->GetNode(target_lane_rear_node_id);
  bool is_large_car_in_side_ = false;
  if (target_lane_rear_node) {
    is_large_car_in_side_ =
        agent::AgentType::BUS == target_lane_rear_node->type() ||
        agent::AgentType::TRUCK == target_lane_rear_node->type() ||
        agent::AgentType::TRAILER == target_lane_rear_node->type() ||
        target_lane_rear_node->node_length() > kLargeAgentLengthM;
  }
  if (target_lane_middle_node) {
    // 如果在纵向上有与自车overlap的障碍物车辆，直接返回false
    return false;
  }
  const double v_ego =
      session_->environmental_model().get_ego_state_manager()->ego_v();
  if (target_lane_front_node) {
    const double distance_rel =
        target_lane_front_node->node_back_edge_to_ego_front_edge_distance();
    const double node_v = target_lane_front_node->node_speed();
    const double node_a = target_lane_front_node->node_accel();
    const double target_lane_need_safety_dist =
        planning::CalcGapObjSafeDistance(v_ego, node_v, node_a, false, true);
    if (distance_rel < target_lane_need_safety_dist) {
      // 前方障碍物纵向上需要的空间不够
      return false;
    }
  }
  if (target_lane_rear_node) {
    const double node_v = target_lane_rear_node->node_speed();
    const double node_a = target_lane_rear_node->node_accel();
    const double distance_rel =
        target_lane_rear_node->node_front_edge_to_ego_back_edge_distance();

    const double need_safety_dist = planning::CalcGapObjSafeDistance(
        v_ego, node_v, node_a, is_large_car_in_side_, false);
    if (distance_rel < need_safety_dist) {
      // 后方障碍物纵向上需要的空间不够
      return false;
    }
  }
  return true;
}
bool LaneChangeRequest::IsDashEnoughForRepeatSegments(
    const RequestType& lc_request, const int origin_lane_id,
    const StateMachineLaneChangeStatus &lc_status) const {
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();
  const double ego_v = ego_state->ego_v();
  double dash_length = 0.0;
  double lane_line_length = 0.0;
  double current_segment_already_pass_length = 0.0;
  int current_segment_count = 0;
  bool all_lane_boundary_types_are_dashed = true;
  double default_lc_boundary_length = 100.0;
  std::shared_ptr<planning_math::KDPath> target_boundary_path;
  const std::shared_ptr<VirtualLane> current_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(origin_lane_id);

  if (current_lane == nullptr) {
    return false;
  }
  const auto &route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();

  const auto &plannig_init_point = ego_state->planning_init_point();
  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  double ego_s = 0.0, ego_l = 0.0;
  if (lc_request == LEFT_CHANGE) {
    const auto &left_lane_boundarys = current_lane->get_left_lane_boundary();
    target_boundary_path =
        virtual_lane_mgr_->MakeBoundaryPath(left_lane_boundarys);
    if (target_boundary_path != nullptr) {
      if (!target_boundary_path->XYToSL(ego_x, ego_y, &ego_s, &ego_l)) {
        return false;
      }
    } else {
      return false;
    }

    for (int i = 0; i < left_lane_boundarys.type_segments_size; i++) {
      lane_line_length += left_lane_boundarys.type_segments[i].length;
      if (lane_line_length > ego_s) {
        current_segment_count = i;
        break;
      }
    }
    current_segment_already_pass_length =
        left_lane_boundarys.type_segments[current_segment_count].length -
        (lane_line_length - ego_s);

    for (int iter = current_segment_count;
         iter < left_lane_boundarys.type_segments_size; iter++) {
      if (left_lane_boundarys.type_segments[iter].type ==
              iflyauto::LaneBoundaryType_MARKING_DASHED ||
          left_lane_boundarys.type_segments[iter].type ==
              iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
        dash_length += left_lane_boundarys.type_segments[iter].length;
      } else {
        all_lane_boundary_types_are_dashed = false;
        break;
      }
    }
    dash_length -= current_segment_already_pass_length;
    dash_length = std::max(0.0, dash_length);
  } else if (lc_request == RIGHT_CHANGE) {
    const auto &right_lane_boundarys = current_lane->get_right_lane_boundary();
    target_boundary_path =
        virtual_lane_mgr_->MakeBoundaryPath(right_lane_boundarys);
    if (target_boundary_path != nullptr) {
      if (!target_boundary_path->XYToSL(ego_x, ego_y, &ego_s, &ego_l)) {
        return false;
      }
    } else {
      return false;
    }

    for (int i = 0; i < right_lane_boundarys.type_segments_size; i++) {
      lane_line_length += right_lane_boundarys.type_segments[i].length;
      if (lane_line_length > ego_s) {
        current_segment_count = i;
        break;
      }
    }
    current_segment_already_pass_length =
        right_lane_boundarys.type_segments[current_segment_count].length -
        (lane_line_length - ego_s);

    for (int iter = current_segment_count;
         iter < right_lane_boundarys.type_segments_size; iter++) {
      if (right_lane_boundarys.type_segments[iter].type ==
              iflyauto::LaneBoundaryType_MARKING_DASHED ||
          right_lane_boundarys.type_segments[iter].type ==
              iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
        dash_length += right_lane_boundarys.type_segments[iter].length;
      } else {
        all_lane_boundary_types_are_dashed = false;
        break;
      }
    }
    dash_length -= current_segment_already_pass_length;
    dash_length = std::max(0.0, dash_length);
  }

  double lc_response_dist = ego_v * kLaneChangeSolidLineTTC;  // hack
  JSON_DEBUG_VALUE("dash_line_len", dash_length);
  std::cout << "dash_length:" << dash_length
            << ",lc_response_dist:" << lc_response_dist << std::endl;
  if (dash_length > default_lc_boundary_length ||
      all_lane_boundary_types_are_dashed || dash_length > lc_response_dist) {
    return true;
  }
  const auto &cur_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();
  if (route_info_output.dis_to_ramp < 100 ||
      (cur_lane->is_nearing_split_mlc_task() &&
       route_info_output.distance_to_first_road_split < 100)) {
    if (lc_request == LEFT_CHANGE) {
      iflyauto::LaneBoundaryType left_boundary_type =
          MakesureCurrentBoundaryType(LEFT_CHANGE, origin_lane_id);
      if (left_boundary_type ==
              iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_DASHED ||
          left_boundary_type == iflyauto::LaneBoundaryType_MARKING_VIRTUAL ||
          left_boundary_type == iflyauto::LaneBoundaryType_MARKING_SOLID) {
        return true;
      }
    } else if (lc_request == RIGHT_CHANGE) {
      iflyauto::LaneBoundaryType right_boundary_type =
          MakesureCurrentBoundaryType(RIGHT_CHANGE, origin_lane_id);
      if (right_boundary_type ==
              iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_DASHED ||
          right_boundary_type == iflyauto::LaneBoundaryType_MARKING_VIRTUAL ||
          right_boundary_type == iflyauto::LaneBoundaryType_MARKING_SOLID) {
        return true;
      }
    }
  }
  //
  if (lc_status == StateMachineLaneChangeStatus::kLaneChangeComplete ||
      lc_status == StateMachineLaneChangeStatus::kLaneKeeping) {
    return true;
  }
  double press_line_ratio = CalculatePressLineRatio(origin_lane_id, lc_request);
  if (lc_status == StateMachineLaneChangeStatus::kLaneChangeExecution) {
    if (press_line_ratio > kIgnoreLineTypeThreshold) {
      return true;
    }
  }
  // std::cout << "dash lengh less than lc response dist!!!!" << std::endl;
  return false;
}

iflyauto::LaneBoundaryType LaneChangeRequest::MakesureCurrentBoundaryType(
    const RequestType lc_request, const int origin_lane_id) const {
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();
  double lane_line_length = 0.0;
  std::shared_ptr<planning_math::KDPath> target_boundary_path;
  const auto &plannig_init_point = ego_state->planning_init_point();
  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  double ego_s = 0.0, ego_l = 0.0;
  const std::shared_ptr<VirtualLane> current_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(origin_lane_id);

  if (lc_request == LEFT_CHANGE) {
    const auto &left_lane_boundarys = current_lane->get_left_lane_boundary();
    target_boundary_path =
        virtual_lane_mgr_->MakeBoundaryPath(left_lane_boundarys);
    if (target_boundary_path != nullptr) {
      if (!target_boundary_path->XYToSL(ego_x, ego_y, &ego_s, &ego_l)) {
        return iflyauto::LaneBoundaryType_MARKING_SOLID;
      }
    } else {
      return iflyauto::LaneBoundaryType_MARKING_SOLID;
    }
    for (int i = 0; i < left_lane_boundarys.type_segments_size; i++) {
      lane_line_length += left_lane_boundarys.type_segments[i].length;
      if (lane_line_length > ego_s) {
        return left_lane_boundarys.type_segments[i].type;
      }
    }
  } else if (lc_request == RIGHT_CHANGE) {
    const auto &right_lane_boundarys = current_lane->get_right_lane_boundary();
    target_boundary_path =
        virtual_lane_mgr_->MakeBoundaryPath(right_lane_boundarys);
    if (target_boundary_path != nullptr) {
      if (!target_boundary_path->XYToSL(ego_x, ego_y, &ego_s, &ego_l)) {
        return iflyauto::LaneBoundaryType_MARKING_SOLID;
      }
    } else {
      return iflyauto::LaneBoundaryType_MARKING_SOLID;
    }
    for (int i = 0; i < right_lane_boundarys.type_segments_size; i++) {
      lane_line_length += right_lane_boundarys.type_segments[i].length;
      if (lane_line_length > ego_s) {
        return right_lane_boundarys.type_segments[i].type;
      }
    }
  }
  return iflyauto::LaneBoundaryType_MARKING_SOLID;
}

bool LaneChangeRequest::IsRoadBorderSurpressLaneChange(
    const RequestType lc_request, const int origin_lane_id,
    const int target_lane_id) {
  ReferencePathPoint sample_path_point{};
  const double cut_length = 1.4;
  const double sample_forward_distance = 1.0;
  ReferencePathPoint refpath_pt{};
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  std::shared_ptr<ReferencePathManager> reference_path_mgr =
      session_->mutable_environmental_model()->get_reference_path_manager();

  std::shared_ptr<ReferencePath> reference_path_ptr =
      reference_path_mgr->get_reference_path_by_lane(origin_lane_id, false);
  const std::shared_ptr<VirtualLane> target_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(target_lane_id);
  const double ego_lateral_offset_in_target_lane =
      std::fabs(target_lane->get_ego_lateral_offset());

  const std::shared_ptr<planning_math::KDPath> base_frenet_coord =
      reference_path_ptr->get_frenet_coord();
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();
  const PlanningInitPoint planning_init_point =
      ego_state->planning_init_point();
  Point2D ego_frenet_point;
  Point2D ego_cart_point{planning_init_point.lat_init_state.x(),
                         planning_init_point.lat_init_state.y()};
  if (!base_frenet_coord->XYToSL(ego_cart_point, ego_frenet_point)) {
    ILOG_DEBUG << "IsRoadBorderSurpressLaneChange::fail to get ego position on base lane";
    return true;
  }
  if (!reference_path_ptr->get_reference_point_by_lon(ego_frenet_point.x,
                                                      sample_path_point)) {
    return true;
  }
  for (double s = ego_frenet_point.x - vehicle_param.rear_edge_to_rear_axle;
       s < ego_frenet_point.x + vehicle_param.front_edge_to_rear_axle +
               sample_forward_distance;
       s += cut_length) {
    if (reference_path_ptr->get_reference_point_by_lon(s, refpath_pt)) {
      sample_path_point.distance_to_left_lane_border =
          std::fmin(refpath_pt.distance_to_left_lane_border,
                    sample_path_point.distance_to_left_lane_border);
      sample_path_point.distance_to_right_lane_border =
          std::fmin(refpath_pt.distance_to_right_lane_border,
                    sample_path_point.distance_to_right_lane_border);
      sample_path_point.distance_to_left_road_border =
          std::fmin(refpath_pt.distance_to_left_road_border,
                    sample_path_point.distance_to_left_road_border);
      sample_path_point.distance_to_right_road_border =
          std::fmin(refpath_pt.distance_to_right_road_border,
                    sample_path_point.distance_to_right_road_border);
    }
  }
  if (lc_request == LEFT_CHANGE) {
    if (sample_path_point.distance_to_left_road_border <
        ego_lateral_offset_in_target_lane) {
      return true;
    } else {
      return false;
    }
  } else if (lc_request == RIGHT_CHANGE) {
    if (sample_path_point.distance_to_right_road_border <
        ego_lateral_offset_in_target_lane) {
      return true;
    } else {
      return false;
    }
  } else {
    return true;
  }
}

bool LaneChangeRequest::IsRoadBorderSurpressDuringLaneChange(
    const RequestType lc_direction, const int origin_lane_id,
    const int target_lane_id) {
  ReferencePathPoint sample_path_point{};
  const double cut_length = 1.4;
  const double sample_forward_distance = 1.0;
  const double predict_time_horizon = 8.0;

  ReferencePathPoint refpath_pt{};
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  std::shared_ptr<ReferencePathManager> reference_path_mgr =
      session_->mutable_environmental_model()->get_reference_path_manager();

  std::shared_ptr<ReferencePath> reference_path_ptr =
      reference_path_mgr->get_reference_path_by_lane(origin_lane_id, false);
  const std::shared_ptr<VirtualLane> target_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(target_lane_id);

  if (!reference_path_ptr) {
    ILOG_ERROR << "IsRoadBorderSurpressDuringLaneChange: invalid reference path";
    return true;
  }
  if (!target_lane) {
    ILOG_ERROR << "IsRoadBorderSurpressDuringLaneChange: invalid target lane";
    return true;
  }

  const double ego_lateral_offset_in_target_lane =
      std::fabs(target_lane->get_ego_lateral_offset());
  const std::shared_ptr<planning_math::KDPath> base_frenet_coord =
      reference_path_ptr->get_frenet_coord();
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();
  const PlanningInitPoint planning_init_point =
      ego_state->planning_init_point();
  double ego_vel = ego_state->ego_v();
  Point2D ego_frenet_point;
  Point2D ego_cart_point{planning_init_point.lat_init_state.x(),
                         planning_init_point.lat_init_state.y()};
  if (!base_frenet_coord->XYToSL(ego_cart_point, ego_frenet_point)) {
    ILOG_DEBUG << "IsRoadBorderSurpressLaneChange::fail to get ego position on base lane";
    return true;
  }
  if (!reference_path_ptr->get_reference_point_by_lon(ego_frenet_point.x,
                                                      sample_path_point)) {
    return true;
  }
  double RoadBorderConsiderDistance = MAX(ego_vel * predict_time_horizon, 50.0);
  for (double s = ego_frenet_point.x - vehicle_param.rear_edge_to_rear_axle;
       s < RoadBorderConsiderDistance; s += cut_length) {
    if (reference_path_ptr->get_reference_point_by_lon(s, refpath_pt)) {
      sample_path_point.distance_to_left_lane_border =
          std::fmin(refpath_pt.distance_to_left_lane_border,
                    sample_path_point.distance_to_left_lane_border);
      sample_path_point.distance_to_right_lane_border =
          std::fmin(refpath_pt.distance_to_right_lane_border,
                    sample_path_point.distance_to_right_lane_border);
      sample_path_point.distance_to_left_road_border =
          std::fmin(refpath_pt.distance_to_left_road_border,
                    sample_path_point.distance_to_left_road_border);
      sample_path_point.distance_to_right_road_border =
          std::fmin(refpath_pt.distance_to_right_road_border,
                    sample_path_point.distance_to_right_road_border);
    }
  }
  if (lc_direction == LEFT_CHANGE) {
    if (sample_path_point.distance_to_left_road_border <
        ego_lateral_offset_in_target_lane) {
      return true;
    } else {
      return false;
    }
  } else if (lc_direction == RIGHT_CHANGE) {
    if (sample_path_point.distance_to_right_road_border <
        ego_lateral_offset_in_target_lane) {
      return true;
    } else {
      return false;
    }
  } else {
    return true;
  }
}
}  // namespace planning