#include "lane_change_request.h"

#include <sys/param.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "common_c.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "lateral_obstacle.h"
#include "obstacle_manager.h"
#include "planning_context.h"
#include "task_interface/lane_change_utils.h"
#include "utils/kd_path.h"
#include "utils/lateral_utils.h"

namespace planning {
namespace {
constexpr double kLargeAgentLengthM = 8.0;
constexpr double kInputBoundaryLenLimit = 145.;
constexpr double kDefaultBoundaryLen = 5000.;
// https://yf2ljykclb.xfchat.iflytek.com/wiki/MXjXwlCjni6g7nkjgKGrfwGwzPb
constexpr double kLaneChangeSolidLineTTC = 3.0;  // todo(ldh): 从配置中读取
constexpr double kLaneChangeFrontSolidLineTTC = 1.5;  // todo(ldh): 从配置中读取
constexpr double kLaneChangeMinDistance = 9;
constexpr double kIgnoreLineTypeThreshold = 0.33333333333;
constexpr double kStandardLaneWidth = 3.8;
constexpr double kEps = 1e-6;
constexpr int kEgoInIntersectionCount = 3;
constexpr double kLongClusterCoeff = 10.0;
constexpr double kLatClusterThre = 0.3;
constexpr double kLatPassThre = 1.2;
constexpr double kLatPassThreBuffer = 0.25;
constexpr double kConeCrossingLaneLineBuffer = 0.15;
constexpr double kDefaultLaneWidth = 4.5;
constexpr double kMinDefaultLaneWidth = 2.65;
constexpr int kInvalidAgentId = -1;
constexpr double kHysteresisCoefficient = 1.5;
constexpr double kConeLaneChangelateralDistancethre = 2.25;
constexpr double kMinRampSampleLength = 150.0;
constexpr double kCurveRadiusThreshold = 150.0;

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
    ILOG_DEBUG << "[LaneChangeRequest::GenerateRequest] Illgeal direction["
               << direction << "]";
  }
  if (request_type_ == direction) {
    ILOG_DEBUG << "[LaneChangeRequest::GenerateRequest] duplicated request, "
                  "direction :"
               << direction;
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
    const int target_lane_id, const RequestType &lc_request) const {
  double press_line_ratio = 0.0;

  // press_line_ratio = target_lane->press_line_ratio();
  auto target_reference_path =
      session_->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(target_lane_id, false);
  auto target_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(target_lane_id);

  if (nullptr == target_lane || target_reference_path == nullptr) {
    return 0.0;
  }
  const auto &ego_frenent_boundary =
      target_reference_path->get_ego_frenet_boundary();
  const double ego_center_s =
      ego_frenent_boundary.s_start +
      (ego_frenent_boundary.s_end - ego_frenent_boundary.s_start) / 2.0;
  const double frenent_ego_width =
      ego_frenent_boundary.l_end - ego_frenent_boundary.l_start;
  double lane_width = target_lane->width_by_s(ego_center_s);
  double half_lane_width = lane_width * 0.5;
  if (lc_request == LEFT_CHANGE) {
    if (ego_frenent_boundary.l_end < -half_lane_width) {
      press_line_ratio = 0.0;
    } else if (ego_frenent_boundary.l_end > -half_lane_width) {
      press_line_ratio =
          (ego_frenent_boundary.l_end + half_lane_width) / frenent_ego_width;
    } else {
      press_line_ratio = 0.0;
    }
  } else if (lc_request == RIGHT_CHANGE) {
    if (ego_frenent_boundary.l_start > half_lane_width) {
      press_line_ratio = 0.0;
    } else if (ego_frenent_boundary.l_start < half_lane_width) {
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
double LaneChangeRequest::CalculatePressLineRatioByTwoLanes(
    const int origin_lane_id, const int target_lane_id,
    const RequestType &lc_request) const {
  double press_line_ratio = 0.0;
  // auto origin_lane =
  //     virtual_lane_mgr_->get_lane_with_virtual_id(origin_lane_id);
  // auto target_lane =
  //     virtual_lane_mgr_->get_lane_with_virtual_id(target_lane_id);
  // if(origin_lane != nullptr){
  //   press_line_ratio = CalculatePressLineRatioByOrigin(origin_lane_id,
  //   lc_request);
  // }else if(target_lane != nullptr){
  //   press_line_ratio = CalculatePressLineRatioByTarget(target_lane_id,
  //   lc_request);
  // }else{
  //   press_line_ratio = 0.0;
  // }
  // 内部有非空保护了
  double press_origin_line_ratio =
      CalculatePressLineRatioByOrigin(origin_lane_id, lc_request);
  double press_target_line_ratio =
      CalculatePressLineRatioByTarget(target_lane_id, lc_request);
  press_line_ratio = std::max(press_origin_line_ratio, press_target_line_ratio);
  return press_line_ratio;
}
double LaneChangeRequest::CalculatePressLineRatioByOrigin(
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
          (-ego_frenent_boundary.l_start - half_lane_width) / frenent_ego_width;
    } else {
      press_line_ratio = 0.0;
    }
  } else {
    press_line_ratio = 0.0;
  }
  return press_line_ratio;
}
double LaneChangeRequest::CalculatePressLineRatioByTarget(
    const int target_lane_id, const RequestType &lc_request) const {
  double press_line_ratio = 0.0;

  // press_line_ratio = target_lane->press_line_ratio();
  auto target_reference_path =
      session_->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(target_lane_id, false);
  auto target_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(target_lane_id);

  if (nullptr == target_lane || target_reference_path == nullptr) {
    return 0.0;
  }
  const auto &ego_frenent_boundary =
      target_reference_path->get_ego_frenet_boundary();
  const double ego_center_s =
      ego_frenent_boundary.s_start +
      (ego_frenent_boundary.s_end - ego_frenent_boundary.s_start) / 2.0;
  const double frenent_ego_width =
      ego_frenent_boundary.l_end - ego_frenent_boundary.l_start;
  double lane_width = target_lane->width_by_s(ego_center_s);
  double half_lane_width = lane_width * 0.5;
  if (lc_request == LEFT_CHANGE) {
    if (ego_frenent_boundary.l_end < -half_lane_width) {
      press_line_ratio = 0.0;
    } else if (ego_frenent_boundary.l_end > -half_lane_width) {
      press_line_ratio =
          (ego_frenent_boundary.l_end + half_lane_width) / frenent_ego_width;
    } else {
      press_line_ratio = 0.0;
    }
  } else if (lc_request == RIGHT_CHANGE) {
    if (ego_frenent_boundary.l_start > half_lane_width) {
      press_line_ratio = 0.0;
    } else if (ego_frenent_boundary.l_start < half_lane_width) {
      press_line_ratio =
          (half_lane_width - ego_frenent_boundary.l_start) / frenent_ego_width;
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
  ILOG_DEBUG << "origin_lane_virtual_id_: " << origin_lane_virtual_id_
            << "origin_lane_order_id_: " << origin_lane_virtual_id_;
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
    ILOG_INFO << "!dashed_enough";
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
    const RequestType &lc_request, const RequestSource &lc_request_source,
    int origin_lane_id, const StateMachineLaneChangeStatus &lc_status) const {
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

  std::shared_ptr<VirtualLane> target_lane = nullptr;
  if (lc_request == LEFT_CHANGE) {
    target_lane = virtual_lane_mgr_->get_left_lane();
  } else {
    target_lane = virtual_lane_mgr_->get_right_lane();
  }
  // 增加对于前方路沿的判断
  if (target_lane != nullptr &&
      IsRoadBorderSurpressDuringLaneChange(lc_request, origin_lane_id,
                                     target_lane->get_virtual_id())) {
    return false;
  }

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

    if (left_lane_boundarys.type_segments[current_segment_count].type ==
            iflyauto::LaneBoundaryType_MARKING_SOLID ||
        left_lane_boundarys.type_segments[current_segment_count].type ==
            iflyauto::LaneBoundaryType_MARKING_DOUBLE_SOLID ||
        left_lane_boundarys.type_segments[current_segment_count].type ==
            iflyauto::LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED) {
      all_lane_boundary_types_are_dashed = false;
      // 计算前方实线的距离
      const double front_ignore_solid_dis =
          ego_v * kLaneChangeFrontSolidLineTTC;
      double front_sum_solid_dis = 0.0;
      double front_sum_dash_dis = 0.0;

      bool is_continue_solid_lane = true;
      for (int iter = current_segment_count;
           iter < left_lane_boundarys.type_segments_size; iter++) {
        if (left_lane_boundarys.type_segments[iter].type ==
                iflyauto::LaneBoundaryType_MARKING_SOLID ||
            left_lane_boundarys.type_segments[iter].type ==
                iflyauto::LaneBoundaryType_MARKING_DOUBLE_SOLID ||
            left_lane_boundarys.type_segments[iter].type ==
                iflyauto::LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED) {
          if (!is_continue_solid_lane) {
            break;
          }
          front_sum_solid_dis = front_sum_solid_dis +
                                left_lane_boundarys.type_segments[iter].length;
          if (front_sum_solid_dis - current_segment_already_pass_length >
              front_ignore_solid_dis) {
            break;
          }

        } else if (left_lane_boundarys.type_segments[iter].type ==
                       iflyauto::LaneBoundaryType_MARKING_DASHED ||
                   left_lane_boundarys.type_segments[iter].type ==
                       iflyauto::LaneBoundaryType_MARKING_DECELERATION_DASHED ||
                   left_lane_boundarys.type_segments[iter].type ==
                       iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
          is_continue_solid_lane = false;
          front_sum_dash_dis = front_sum_dash_dis +
                               left_lane_boundarys.type_segments[iter].length;
          double front_need_gap_dash_len = ego_v * 2.0;
          if (front_sum_dash_dis > front_need_gap_dash_len) {
            return true;
          }
        }
      }
    } else {
      for (int iter = current_segment_count;
           iter < left_lane_boundarys.type_segments_size; iter++) {
        if (left_lane_boundarys.type_segments[iter].type ==
                iflyauto::LaneBoundaryType_MARKING_DASHED ||
            left_lane_boundarys.type_segments[iter].type ==
                iflyauto::LaneBoundaryType_MARKING_DECELERATION_DASHED ||
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
    }

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

    if (right_lane_boundarys.type_segments[current_segment_count].type ==
            iflyauto::LaneBoundaryType_MARKING_SOLID ||
        right_lane_boundarys.type_segments[current_segment_count].type ==
            iflyauto::LaneBoundaryType_MARKING_DOUBLE_SOLID ||
        right_lane_boundarys.type_segments[current_segment_count].type ==
            iflyauto::LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED) {
      all_lane_boundary_types_are_dashed = false;
      // 计算前方实线的距离
      const double front_ignore_solid_dis =
          ego_v * kLaneChangeFrontSolidLineTTC;
      double front_sum_solid_dis = 0.0;
      double front_sum_dash_dis = 0.0;

      bool is_continue_solid_lane = true;
      for (int iter = current_segment_count;
           iter < right_lane_boundarys.type_segments_size; iter++) {
        if (right_lane_boundarys.type_segments[iter].type ==
                iflyauto::LaneBoundaryType_MARKING_SOLID ||
            right_lane_boundarys.type_segments[iter].type ==
                iflyauto::LaneBoundaryType_MARKING_DOUBLE_SOLID ||
            right_lane_boundarys.type_segments[iter].type ==
                iflyauto::LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED) {
          if (!is_continue_solid_lane) {
            break;
          }
          front_sum_solid_dis = front_sum_solid_dis +
                                right_lane_boundarys.type_segments[iter].length;
          if (front_sum_solid_dis - current_segment_already_pass_length >
              front_ignore_solid_dis) {
            break;
          }

        } else if (right_lane_boundarys.type_segments[iter].type ==
                       iflyauto::LaneBoundaryType_MARKING_DASHED ||
                   right_lane_boundarys.type_segments[iter].type ==
                       iflyauto::LaneBoundaryType_MARKING_DECELERATION_DASHED ||
                   right_lane_boundarys.type_segments[iter].type ==
                       iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
          is_continue_solid_lane = false;
          front_sum_dash_dis = front_sum_dash_dis +
                               right_lane_boundarys.type_segments[iter].length;
          double front_need_gap_dash_len = ego_v * 2.0;
          if (front_sum_dash_dis > front_need_gap_dash_len) {
            return true;
          }
        }
      }
    } else {
      for (int iter = current_segment_count;
          iter < right_lane_boundarys.type_segments_size; iter++) {
        if (right_lane_boundarys.type_segments[iter].type ==
                iflyauto::LaneBoundaryType_MARKING_DASHED ||
            right_lane_boundarys.type_segments[iter].type ==
                iflyauto::LaneBoundaryType_MARKING_DECELERATION_DASHED ||
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

  }

  const double lc_response_dist = std::max(ego_v * kLaneChangeSolidLineTTC,
                                           kLaneChangeMinDistance);  // hack
  JSON_DEBUG_VALUE("dash_line_len", dash_length);
  ILOG_DEBUG << "dash_length:" << dash_length
            << ",lc_response_dist:" << lc_response_dist;

  if (dash_length > default_lc_boundary_length ||
      all_lane_boundary_types_are_dashed || dash_length > lc_response_dist) {
    return true;
  }

  const auto &cur_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();
  bool is_mlc_ignore_solid_lane_check =
      IsMLCIgnoreSolidLaneCheck(lc_request, lc_request_source);
  if (is_mlc_ignore_solid_lane_check) {
    if (lc_request == LEFT_CHANGE) {
      iflyauto::LaneBoundaryType left_boundary_type =
          MakesureCurrentBoundaryType(LEFT_CHANGE, origin_lane_id);
      if (left_boundary_type ==
              iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_DASHED ||
          left_boundary_type ==
              iflyauto::LaneBoundaryType::
                  LaneBoundaryType_MARKING_DECELERATION_DASHED ||
          left_boundary_type == iflyauto::LaneBoundaryType_MARKING_VIRTUAL ||
          left_boundary_type == iflyauto::LaneBoundaryType_MARKING_SOLID ||
          left_boundary_type ==
              iflyauto::LaneBoundaryType_MARKING_DECELERATION_SOLID) {
        return true;
      }
    } else if (lc_request == RIGHT_CHANGE) {
      iflyauto::LaneBoundaryType right_boundary_type =
          MakesureCurrentBoundaryType(RIGHT_CHANGE, origin_lane_id);
      if (right_boundary_type ==
              iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_DASHED ||
          right_boundary_type ==
              iflyauto::LaneBoundaryType::
                  LaneBoundaryType_MARKING_DECELERATION_DASHED ||
          right_boundary_type == iflyauto::LaneBoundaryType_MARKING_VIRTUAL ||
          right_boundary_type == iflyauto::LaneBoundaryType_MARKING_SOLID ||
          right_boundary_type ==
              iflyauto::LaneBoundaryType_MARKING_DECELERATION_SOLID) {
        return true;
      }
    }
  }
  //
  // if (lc_status == StateMachineLaneChangeStatus::kLaneChangeComplete ||
  //     lc_status == StateMachineLaneChangeStatus::kLaneKeeping) {
  //   return true;
  // }
  if (lc_status == StateMachineLaneChangeStatus::kLaneChangeComplete) {
    return true;
  }
  double press_line_ratio = CalculatePressLineRatio(origin_lane_id, lc_request);
  if (lc_status == StateMachineLaneChangeStatus::kLaneChangeExecution) {
    if (press_line_ratio > kIgnoreLineTypeThreshold) {
      return true;
    }
  }
  // ILOG_DEBUG << "dash lengh less than lc response dist!!!!" << std::endl;
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
    ILOG_DEBUG << "IsRoadBorderSurpressLaneChange::fail to get ego position on "
                  "base lane";
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
    const int target_lane_id) const {
  ReferencePathPoint sample_path_point{};
  const double cut_length = 1.4;
  const double sample_forward_distance = 1.0;
  const double predict_time_horizon = 3.5;

  ReferencePathPoint refpath_pt{};
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  std::shared_ptr<ReferencePathManager> reference_path_mgr =
      session_->mutable_environmental_model()->get_reference_path_manager();

  std::shared_ptr<ReferencePath> reference_path_ptr =
      reference_path_mgr->get_reference_path_by_lane(origin_lane_id, false);
  const std::shared_ptr<VirtualLane> target_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(target_lane_id);
  const std::shared_ptr<VirtualLane> origin_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(origin_lane_id);
  if (!reference_path_ptr) {
    ILOG_ERROR
        << "IsRoadBorderSurpressDuringLaneChange: invalid reference path";
    return true;
  }
  if (!target_lane) {
    ILOG_ERROR << "IsRoadBorderSurpressDuringLaneChange: invalid target lane";
    return true;
  }
  if(!origin_lane){
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
    ILOG_DEBUG << "IsRoadBorderSurpressLaneChange::fail to get ego position on "
                  "base lane";
    return true;
  }
  if (!reference_path_ptr->get_reference_point_by_lon(ego_frenet_point.x,
                                                      sample_path_point)) {
    return true;
  }
  double half_origin_lane_width = origin_lane->width() * 0.5;
  double half_target_lane_width = target_lane->width() * 0.5;
  double RoadBorderConsiderDistance = ego_vel * predict_time_horizon;
  double ego_lateral_offset_to_origin_boundary = std::max(ego_lateral_offset_in_target_lane - half_target_lane_width, 0.0);
  double lateral_ratio = ego_lateral_offset_to_origin_boundary / half_origin_lane_width;
  //根据横向距离衰减
  RoadBorderConsiderDistance = std::max(lateral_ratio * RoadBorderConsiderDistance, 5.0);
  for (double s = ego_frenet_point.x - vehicle_param.rear_edge_to_rear_axle;
       s < ego_frenet_point.x - vehicle_param.rear_edge_to_rear_axle +
               RoadBorderConsiderDistance;
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
  if (lc_direction == LEFT_CHANGE) {
    if (sample_path_point.distance_to_left_road_border <
        ego_lateral_offset_to_origin_boundary) {
      return true;
    } else {
      return false;
    }
  } else if (lc_direction == RIGHT_CHANGE) {
    if (sample_path_point.distance_to_right_road_border <
        ego_lateral_offset_to_origin_boundary) {
      return true;
    } else {
      return false;
    }
  } else {
    return true;
  }
}

double LaneChangeRequest::CalculateDynamicTTCtime(
    const int origin_lane_id, const RequestType &lc_request) const {
  double ttc_time = kLaneChangeSolidLineTTC;

  const auto origin_reference_path =
      session_->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(origin_lane_id, false);
  const auto origin_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(origin_lane_id);

  if (nullptr == origin_lane || origin_reference_path == nullptr) {
    return ttc_time;
  }

  const auto origin_frenet_coor = origin_reference_path->get_frenet_coord();

  if (origin_frenet_coor == nullptr) {
    return ttc_time;
  }

  const double origin_lane_half_width =
      origin_lane->width_by_s(
          origin_reference_path->get_frenet_ego_state().s()) *
      0.5;

  // 车辆参数
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double kEgoWidth = vehicle_param.width;

  //六分之一的自车宽度
  const double l_err = 0.16666666666667 * kEgoWidth;
  const double ego_l = origin_reference_path->get_frenet_ego_state().l();

  double lat_offset = 0.0;
  if (lc_request == LEFT_CHANGE) {
    lat_offset = origin_lane_half_width - ego_l - l_err;
  } else if (lc_request == RIGHT_CHANGE) {
    lat_offset = origin_lane_half_width + ego_l - l_err;
  }
  const double base_lat_offset = kStandardLaneWidth * 0.5 - l_err;

  if (base_lat_offset < kEps) {
    return 0.0;
  }

  ttc_time = kLaneChangeSolidLineTTC * lat_offset / base_lat_offset;

  return ttc_time;
}

bool LaneChangeRequest::EgoInIntersection() {
  const auto &tfl_decider = session_->mutable_planning_context()
                                ->mutable_traffic_light_decider_output();
  const auto intersection_state = session_->environmental_model()
                                      .get_virtual_lane_manager()
                                      ->GetIntersectionState();
  const double distance_to_stopline = session_->environmental_model()
                                          .get_virtual_lane_manager()
                                          ->GetEgoDistanceToStopline();
  const double distance_to_crosswalk = session_->environmental_model()
                                           .get_virtual_lane_manager()
                                           ->GetEgoDistanceToCrosswalk();
  bool current_intersection_state =
      intersection_state == common::IntersectionState::IN_INTERSECTION ||
      distance_to_stopline <= 5.0;
  if (current_intersection_state) {
    intersection_count_ = kEgoInIntersectionCount;
  } else {
    intersection_count_ = std::max(intersection_count_ - 1, 0);
  }

  bool ego_in_intersection_state = intersection_count_ > 0;
  return ego_in_intersection_state;
}

bool LaneChangeRequest::ConeSituationJudgement(
    const std::shared_ptr<VirtualLane> &target_lane) {
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto &function_info = session_->environmental_model().function_info();
  const auto &lateral_obstacle =
      session_->environmental_model().get_lateral_obstacle();
  const auto &ref_path_mgr =
      session_->environmental_model().get_reference_path_manager();
  const auto cur_reference_path =
      ref_path_mgr->get_reference_path_by_current_lane();
  double k_left_cone_occ_lane_line_buffer =
      kConeCrossingLaneLineBuffer * kHysteresisCoefficient;
  double k_right_cone_occ_lane_line_buffer =
      kConeCrossingLaneLineBuffer * kHysteresisCoefficient;
  double k_default_ego_pass_buffer = kLatPassThre;
  int left_lane_nums = 0;
  int right_lane_nums = 0;

  if (target_lane == nullptr) {
    ILOG_DEBUG
        << "LaneChangeRequest::ConeSituationJudgement() base lane not exist";
    return false;
  }
  const auto &rlane = virtual_lane_mgr_->get_lane_with_virtual_id(
      target_lane->get_virtual_id() + 1);
  const auto &llane = virtual_lane_mgr_->get_lane_with_virtual_id(
      target_lane->get_virtual_id() - 1);

  std::shared_ptr<ReferencePath> origin_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(target_lane->get_virtual_id(), false);

  if (origin_refline == nullptr) {
    return false;
  }
  const auto &frenet_obstacles_map = origin_refline->get_obstacles_map();
  const auto &base_frenet_coord = origin_refline->get_frenet_coord();
  if (!base_frenet_coord) {
    return false;
  }
  Point2D ego_frenet_point;
  Point2D ego_cart_point{ego_state->planning_init_point().lat_init_state.x(),
                         ego_state->planning_init_point().lat_init_state.y()};
  if (!base_frenet_coord->XYToSL(ego_cart_point, ego_frenet_point)) {
    return false;
  }

  auto lane_nums_msg = target_lane->get_lane_nums();
  auto iter =
      std::find_if(lane_nums_msg.begin(), lane_nums_msg.end(),
                   [&ego_frenet_point](const iflyauto::LaneNumMsg &lane_num) {
                     return lane_num.begin <= ego_frenet_point.x &&
                            lane_num.end > ego_frenet_point.x;
                   });
  if (iter != lane_nums_msg.end()) {
    left_lane_nums = iter->left_lane_num;
    right_lane_nums = iter->right_lane_num;
  }

  if (llane == nullptr) {
    k_right_cone_occ_lane_line_buffer += 0.25 * kHysteresisCoefficient;
  }
  if (rlane == nullptr) {
    k_left_cone_occ_lane_line_buffer += 0.25 * kHysteresisCoefficient;
  }
  if (function_info.function_mode() == common::DrivingFunctionInfo::NOA) {
    k_default_ego_pass_buffer = kLatPassThre + kLatPassThreBuffer;
  }

  const double ego_rear_edge = vehicle_param.rear_edge_to_rear_axle;
  double eps_s = vehicle_param.length * kLongClusterCoeff;
  double eps_l = vehicle_param.width + kLatClusterThre;
  int cone_nums_of_front_objects = 0;
  int minPts = 1;
  target_lane_cone_points_.clear();
  target_lane_cone_cluster_attribute_set_.clear();

  const auto &tracks_map = lateral_obstacle->tracks_map();
  const auto &front_obstacles_array = lateral_obstacle->front_tracks();
  std::vector<std::pair<double, double>> target_lane_s_width;
  for (const auto front_obstacle : front_obstacles_array) {
    int obstacle_id = front_obstacle->id();
    if (obstacle_id == kInvalidAgentId) {
      continue;
    }
    auto front_vehicle_iter = tracks_map.find(obstacle_id);
    if (front_vehicle_iter != tracks_map.end()) {
      if (front_vehicle_iter->second == nullptr) {
        continue;
      }
      if (front_vehicle_iter->second->type() ==
              iflyauto::OBJECT_TYPE_TRAFFIC_CONE ||
          front_vehicle_iter->second->type() ==
              iflyauto::OBJECT_TYPE_CTASH_BARREL ||
          front_vehicle_iter->second->type() ==
              iflyauto::OBJECT_TYPE_CYLINDER_BARRIER ||
          front_vehicle_iter->second->type() ==
              iflyauto::OBJECT_TYPE_WATER_SAFETY_BARRIER) {
        if (front_vehicle_iter->second->d_s_rel() < -ego_rear_edge ||
            front_vehicle_iter->second->d_s_rel() >
                base_frenet_coord->Length() - ego_frenet_point.x) {
          continue;
        }
        cone_nums_of_front_objects++;
        Point2D obs_cart_point{0.0, 0.0};
        Point2D obs_frenet_point;
        obs_cart_point.x = front_vehicle_iter->second->obstacle()->x_center();
        obs_cart_point.y = front_vehicle_iter->second->obstacle()->y_center();
        if (!base_frenet_coord->XYToSL(obs_cart_point, obs_frenet_point)) {
          continue;
        }

        if (frenet_obstacles_map.find(obstacle_id) ==
                frenet_obstacles_map.end() ||
            !frenet_obstacles_map.at(obstacle_id)->b_frenet_valid()) {
          continue;
        }
        double cone_s = obs_frenet_point.x;
        double cone_l = obs_frenet_point.y;
        double dist_to_left_boundary;
        if (!GetLaneWidthByCone(target_lane, cone_s, cone_l, true,
                                &dist_to_left_boundary, target_lane_s_width)) {
          return false;
        }
        double dist_to_right_boundary;
        if (!GetLaneWidthByCone(target_lane, cone_s, cone_l, false,
                                &dist_to_right_boundary, target_lane_s_width)) {
          return false;
        }

        auto point = ConePoint(front_vehicle_iter->first, obs_cart_point.x,
                               obs_cart_point.y, cone_s, cone_l,
                               dist_to_left_boundary, dist_to_right_boundary);
        target_lane_cone_points_.push_back(point);
      }
    } else {
      continue;
    }
  }
  JSON_DEBUG_VALUE("cone_nums_of_front_objects", cone_nums_of_front_objects);

  if (target_lane_cone_points_.empty()) {
    return true;
  } else {
    // 检测类型为cone的障碍物赋予cluster属性
    DbScan(target_lane_cone_points_, eps_s, eps_l, minPts);
  }

  for (const auto &p : target_lane_cone_points_) {
    // 构建相同cluster属性所包含cones的map
    target_lane_cone_cluster_attribute_set_[p.cluster].push_back(p);
  }

  double lane_width = QueryLaneMinWidth(
      target_lane_cone_points_, target_lane_s_width, ego_frenet_point.x);
  double pass_threshold_left =
      std::max(vehicle_param.width + k_default_ego_pass_buffer,
               lane_width + k_left_cone_occ_lane_line_buffer);
  pass_threshold_left = std::min(pass_threshold_left, kDefaultLaneWidth - 0.1);
  double pass_threshold_right =
      std::max(vehicle_param.width + k_default_ego_pass_buffer,
               lane_width + k_right_cone_occ_lane_line_buffer);
  pass_threshold_right =
      std::min(pass_threshold_right, kDefaultLaneWidth - 0.1);
  for (const auto &cluster_attribute_iter :
       target_lane_cone_cluster_attribute_set_) {
    int cluster = cluster_attribute_iter.first;
    const std::vector<ConePoint> &points = cluster_attribute_iter.second;
    double min_left_l, min_right_l;
    min_left_l = CalcClusterToBoundaryDist(points, LEFT_CHANGE);
    min_right_l = CalcClusterToBoundaryDist(points, RIGHT_CHANGE);
    double total_l = 0.0;
    // 过滤横向远离车道中心线的锥桶簇
    double min_l_to_center_line = 10.0;
    if (points.size() <= 0) {
      continue;
    }

    for (const auto &p : points) {
      min_l_to_center_line = std::min(std::abs(p.l), min_l_to_center_line);
      total_l += p.l;
    }
    if (min_l_to_center_line >
        kConeLaneChangelateralDistancethre * kHysteresisCoefficient) {
      continue;
    }
    double average_l = total_l / points.size();
    ILOG_DEBUG << "min_left_l is:" << min_left_l
               << ", min_right_l is: is:" << min_right_l
               << ", pass_threshold_left is:" << pass_threshold_left
               << ", pass_threshold_right is:" << pass_threshold_right;

    // judge if to trigger cone lc
    if ((min_left_l < pass_threshold_left &&
         min_right_l < pass_threshold_right) ||
        (!llane && min_right_l < pass_threshold_right && points.size() >= 3 &&
         average_l > 0.0) ||
        (!rlane && min_left_l < pass_threshold_left && points.size() >= 3 &&
         average_l < 0.0) ||
        (target_lane->get_ego_lateral_offset() > average_l && average_l > 0.0 && points.size() >= 3) ||
        (target_lane->get_ego_lateral_offset() < average_l && average_l < 0.0 && points.size() >= 3)) {
      return false;
    }
  }

  return true;
}

bool LaneChangeRequest::GetLaneWidthByCone(
    const std::shared_ptr<VirtualLane> base_lane, const double cone_s,
    const double cone_l, bool is_left, double *dist,
    std::vector<std::pair<double, double>> &lane_s_width) {
  if (base_lane == nullptr) {
    return false;
  }

  std::shared_ptr<ReferencePath> origin_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(base_lane->get_virtual_id(), false);

  double origin_lane_width = kDefaultLaneWidth;
  if (origin_refline != nullptr) {
    lane_s_width.clear();
    lane_s_width.reserve(origin_refline->get_points().size());
    for (auto i = 0; i < origin_refline->get_points().size(); i++) {
      const ReferencePathPoint &ref_path_point =
          origin_refline->get_points()[i];
      lane_s_width.emplace_back(std::make_pair(ref_path_point.path_point.s(),
                                               ref_path_point.lane_width));
    }
    origin_lane_width = QueryLaneWidth(cone_s, lane_s_width);
  }

  if (is_left) {
    double left_width = 0.5 * origin_lane_width;
    if (cone_l > left_width) {
      *dist = kDefaultLaneWidth;
    } else {
      *dist = left_width - cone_l;
    }
  } else {
    double right_width = 0.5 * origin_lane_width;
    if (cone_l < -right_width) {
      *dist = kDefaultLaneWidth;
    } else {
      *dist = right_width + cone_l;
    }
  }
  return true;
}

double LaneChangeRequest::QueryLaneWidth(
    const double s0,
    const std::vector<std::pair<double, double>> &lane_s_width) {
  auto comp = [](const std::pair<double, double> &s_width, const double s) {
    return s_width.first < s;
  };
  double lane_width;
  const auto &first_pair_on_lane =
      std::lower_bound(lane_s_width.begin(), lane_s_width.end(), s0, comp);

  if (first_pair_on_lane == lane_s_width.begin()) {
    lane_width = lane_s_width.front().second;
  } else if (first_pair_on_lane == lane_s_width.end()) {
    lane_width = lane_s_width.back().second;
  } else {
    lane_width = planning_math::lerp(
        (first_pair_on_lane - 1)->second, (first_pair_on_lane - 1)->first,
        first_pair_on_lane->second, first_pair_on_lane->first, s0);
  }
  return std::fmax(lane_width, 3.0);
}

double LaneChangeRequest::QueryLaneMinWidth(
    std::vector<ConePoint> &cone_points,
    const std::vector<std::pair<double, double>> &lane_s_width,
    const double target_s) {
  const auto &function_info = session_->environmental_model().function_info();
  double max_lane_width = 2.65;
  double lane_width = 2.5;
  const double k_default_lane_width = 2.5;
  int cone_nums = 0;
  if (!cone_points.empty()) {
    for (const auto &cone : cone_points) {
      if (cone.s < target_s) {
        continue;
      }
      lane_width = QueryLaneWidth(cone.s, lane_s_width);
      cone_nums++;
      if (lane_width > max_lane_width) {
        max_lane_width = lane_width;
      }
    }

    if (cone_nums >= 3 &&
        function_info.function_mode() == common::DrivingFunctionInfo::NOA) {
      return max_lane_width;
    } else {
      return k_default_lane_width;
    }
  }

  return lane_width;
}

void LaneChangeRequest::DbScan(std::vector<ConePoint> &cone_points,
                               double eps_s, double eps_l, int minPts) {
  int c = 0;  // cluster index

  for (size_t index = 0; index < cone_points.size(); ++index) {
    if (!cone_points[index].visited) {
      // 根据锥桶间的距离聚类
      ExpandCluster(cone_points, index, c, eps_s, eps_l, minPts);
      c++;
    }
  }
}

bool LaneChangeRequest::ConeDistance(const ConePoint &a, const ConePoint &b,
                                     double eps_s, double eps_l) {
  return std::abs(a.s - b.s) < eps_s && std::abs(a.l - b.l) < eps_l;
}

void LaneChangeRequest::ExpandCluster(std::vector<ConePoint> &cone_points,
                                      int index, int c, double eps_s,
                                      double eps_l, int minPts) {
  std::vector<int> neighborPts;

  for (size_t i = 0; i < cone_points.size(); ++i) {
    if (ConeDistance(cone_points[index], cone_points[i], eps_s, eps_l)) {
      neighborPts.push_back(i);
    }
  }

  if (neighborPts.size() < minPts) {
    // The point is noise
    cone_points[index].cluster = -1;
    return;
  }

  // Assign the cluster to initial point
  cone_points[index].visited = true;
  cone_points[index].cluster = c;

  // Check all neighbours for being part of the cluster
  for (auto &neighborPt : neighborPts) {
    ConePoint &p_neighbor = cone_points[neighborPt];
    if (!p_neighbor.visited) {
      // Recursively expand the cluster
      ExpandCluster(cone_points, neighborPt, c, eps_s, eps_l, minPts);
    }
  }
}

double LaneChangeRequest::CalcClusterToBoundaryDist(
    const std::vector<ConePoint> &points, RequestType direction) {
  double left_l = std::abs(points[0].left_dist);
  double right_l = std::abs(points[0].right_dist);
  for (const auto &p : points) {
    left_l = std::min(std::abs(p.left_dist), left_l);
    right_l = std::min(std::abs(p.right_dist), right_l);
  }
  if (direction == LEFT_CHANGE) {
    return left_l;
  } else if (direction == RIGHT_CHANGE) {
    return right_l;
  } else {
    return std::max(left_l, right_l);
  }
}

bool LaneChangeRequest::IsCurveSurpressLaneChange() const {
  if (session_ == nullptr) {
    return false;
  }

  const auto &route_info = session_->environmental_model().get_route_info();
  if (route_info == nullptr) {
    return false;
  }

  const auto &ego_state = session_->environmental_model().get_ego_state_manager();
  const auto &pose = ego_state->location_enu();

  // 配置：向前搜索距离（动态车速），曲率平滑窗口
  const double kSearchDistanceAhead = ego_state->ego_v() * 6;
  const double kWindowLen = 20.0;

  // 获取自车当前所在道路link
  const auto *start_link = route_info->get_current_link();
  if (start_link == nullptr) {
    return false;
  }

  const auto &cur_pts = start_link->points().boot().points();
  if (cur_pts.size() < 2) {
    return false;
  }

  // 步骤1：找到自车在当前link上的最近点索引
  double min_dist = std::numeric_limits<double>::max();
  size_t nearest_idx = 0;
  ad_common::math::Vec2d current_point(pose.position.x, pose.position.y);
  for (size_t i = 0; i < cur_pts.size(); ++i) {
    double dist = std::hypot(cur_pts[i].x() - current_point.x(),
                             cur_pts[i].y() - current_point.y());
    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = i;
    }
  }

  // 步骤2：计算自车在当前link上的s坐标（link起点到自车的距离）
  double ego_s_on_cur_link = 0.0;
  for (size_t i = 1; i <= nearest_idx; ++i) {
    ego_s_on_cur_link += std::hypot(cur_pts[i].x() - cur_pts[i - 1].x(),
                                    cur_pts[i].y() - cur_pts[i - 1].y());
  }

  // 遍历起点：从自车所在点开始，只向前计算，不遍历车后方
  const size_t start_traverse_idx = nearest_idx;

  std::vector<double> valid_curves;
  double accumulated_s_from_ego = 0.0;
  const auto *search_link = start_link;
  const auto &sdpro_map = route_info->get_sdpro_map();

  // 遍历当前link及后续link，直到搜索距离耗尽
  while (search_link != nullptr) {
    const auto &pts = search_link->points().boot().points();
    if (pts.size() < 2) {
      search_link = sdpro_map.GetNextLinkOnRoute(search_link->id());
      continue;
    }

    // 构建点集用于曲率计算
    std::vector<ad_common::math::Vec2d> pts_vec;
    pts_vec.reserve(pts.size());
    for (const auto &p : pts) {
      pts_vec.emplace_back(p.x(), p.y());
    }

    // 预计算当前link每个点的绝对s值（从link起点开始）
    std::vector<double> link_point_abs_s(pts.size(), 0.0);
    for (size_t i = 1; i < pts.size(); ++i) {
      link_point_abs_s[i] = link_point_abs_s[i - 1] +
                           std::hypot(pts[i].x() - pts[i-1].x(),
                                      pts[i].y() - pts[i-1].y());
    }
    double link_total_length = link_point_abs_s.back();

    // 计算曲率
    const auto &curve_vec = CalculateAndSmoothCurvature(pts_vec, kWindowLen);
    if (curve_vec.empty() || curve_vec.size() != pts.size()) {
      search_link = sdpro_map.GetNextLinkOnRoute(search_link->id());
      continue;
    }

    // 自车所在link：从 nearest_idx 开始遍历（只向前）
    // 后续link：从 0 开始遍历
    size_t begin_idx = 0;
    if (search_link->id() == start_link->id()) {
      begin_idx = start_traverse_idx;
      // 边界保护：如果起点超出点集，直接跳过当前link
      if (begin_idx >= pts.size()) {
        accumulated_s_from_ego += link_total_length - ego_s_on_cur_link;
        search_link = sdpro_map.GetNextLinkOnRoute(search_link->id());
        continue;
      }
    }

    // 只遍历前方有效点，不处理车后方
    for (size_t i = begin_idx; i < pts.size(); ++i) {
      double point_s_from_ego = 0.0;
      if (search_link->id() == start_link->id()) {
        point_s_from_ego = link_point_abs_s[i] - ego_s_on_cur_link;
      } else {
        point_s_from_ego = accumulated_s_from_ego + link_point_abs_s[i];
      }

      // 超过搜索范围，直接终止
      if (point_s_from_ego > kSearchDistanceAhead) {
        break;
      }

      valid_curves.push_back(std::abs(curve_vec[i]));
    }

    // 更新累计向前距离
    if (search_link->id() == start_link->id()) {
      accumulated_s_from_ego += link_total_length - ego_s_on_cur_link;
    } else {
      accumulated_s_from_ego += link_total_length;
    }

    // 达到最大搜索距离，退出所有循环
    if (accumulated_s_from_ego >= kSearchDistanceAhead - 1e-6) {
      break;
    }

    // 切换到下一个link继续搜索
    search_link = sdpro_map.GetNextLinkOnRoute(search_link->id());
  }

  // 无有效曲率，不抑制变道
  if (valid_curves.empty()) {
    return false;
  }

  double total_curve = 0.0;
  for (const auto &curve : valid_curves) {
    total_curve += curve;
  }
  double average_curve = total_curve / valid_curves.size();

  // 最终大曲率判断（连续点阈值）
  const double curve_threshold = 1.0 / kCurveRadiusThreshold;
  const int min_continuous_points = 4;
  return IsLargeCurvatureByQuantile(valid_curves, curve_threshold,
                                    min_continuous_points);
}

// 输入：150m内的曲率列表（已平滑）、大曲率阈值、最小连续点数
bool LaneChangeRequest::IsLargeCurvatureByQuantile(
    const std::vector<double> &curvature_list, double large_curv_thresh,
    int min_continuous_points) const {
  if (curvature_list.empty()) return false;

  // 步骤1：计算90分位数（先排序）
  std::vector<double> curv_sorted = curvature_list;
  std::sort(curv_sorted.begin(), curv_sorted.end());
  size_t quantile_idx = static_cast<size_t>(curv_sorted.size() * 0.9);
  double curv_90 = curv_sorted[quantile_idx];
  JSON_DEBUG_VALUE("average_curve", curv_90);

  // 步骤2：90分位数未超阈值，直接返回false
  if (curv_90 < large_curv_thresh) return false;

  // 步骤3：验证是否有连续段超阈值（避免单点高曲率）
  int continuous_count = 0;
  for (double curv : curvature_list) {
    if (std::abs(curv) > large_curv_thresh) {
      continuous_count++;
      if (continuous_count >= min_continuous_points) {
        return true;
      }
    } else {
      continuous_count = 0;
    }
  }

  return false;
}
bool LaneChangeRequest::CollectConsiderPoints(std::vector<ad_common::math::Vec2d>& consider_points) const {
  if (session_ == nullptr) {
    return false;
  }

  const auto& route_info = session_->environmental_model().get_route_info();
  if (route_info == nullptr) {
    return false;
  }

  const auto &start_link = route_info->get_current_link();
  if (start_link == nullptr) {
    return false;
  }

  const iflymapdata::sdpro::LinkInfo_Link* cur_link = start_link;
  const auto& pts = cur_link->points().boot().points();
  if (pts.size() < 2) {
    return false;
  }

  // Find nearest point to vehicle
  double min_dist = std::numeric_limits<double>::max();
  size_t nearest_idx = 0;

  ad_common::math::Vec2d current_point;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& pose = ego_state->location_enu();
  current_point.set_x(pose.position.x);
  current_point.set_y(pose.position.y);

  for (size_t i = 0; i < pts.size(); ++i) {
    double dist = std::hypot(pts[i].x() - current_point.x(),
                              pts[i].y() - current_point.y());
    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = i;
    }
  }

  // Collect ENU points on ramp until total length >= 150m or leaving ramp
  consider_points.reserve(50);
  double total_len = 0.0;

  // Collect from nearest point (inclusive)
  const double front_judge_dis = std::max(ego_state->ego_v() * 8, 50.0);
  double cur_link_speed_limit = cur_link->speed_limit();
  size_t points_before = consider_points.size();
  for (size_t i = nearest_idx; i < pts.size(); ++i) {
    ad_common::math::Vec2d p(pts[i].x(), pts[i].y());
    if (!consider_points.empty()) {
      const auto &last = consider_points.back();
      double ds = std::hypot(p.x() - last.x(), p.y() - last.y());
      if (ds < 0.1) continue;
      total_len += ds;
    }
    consider_points.emplace_back(p);
    if (total_len >= front_judge_dis) {
      return true;
    }
  }

  // Continue collecting points from subsequent links
  const auto &sdpro_map = route_info->get_sdpro_map();
  const iflymapdata::sdpro::LinkInfo_Link *next_link =
      sdpro_map.GetNextLinkOnRoute(cur_link->id());
  while (total_len < front_judge_dis) {
    if (next_link == nullptr) {
      return false;
    }

    if (CollectPointsFromLink(next_link, front_judge_dis, consider_points, total_len,
                              0)) {
      return true;
    }

    next_link =
        sdpro_map.GetNextLinkOnRoute(next_link->id());
  }
  return false;
}

bool LaneChangeRequest::CollectPointsFromLink(
    const iflymapdata::sdpro::LinkInfo_Link *link, const double front_judge_dis,
    std::vector<ad_common::math::Vec2d> &enu_points, double &total_len,
    size_t start_idx) const {
  if (link == nullptr) {
    return false;
  }

  const auto &pts = link->points().boot().points();
  if (pts.size() < 2 || start_idx >= pts.size()) {
    return false;
  }

  constexpr double kMinPointSpacing = 0.1;
  for (size_t i = start_idx; i < pts.size(); ++i) {
    ad_common::math::Vec2d p(pts[i].x(), pts[i].y());
    if (!enu_points.empty()) {
      const auto &last = enu_points.back();
      double ds = std::hypot(p.x() - last.x(), p.y() - last.y());
      if (ds < kMinPointSpacing) {
        continue;
      }
      total_len += ds;
    }
    enu_points.emplace_back(p);
    if (total_len >= front_judge_dis) {
      return true;  // Reached target length
    }
  }
  return false;  // Not reached target length
}

bool LaneChangeRequest::IsCurveSurpressLaneChange(int target_lane_virtual_id) const {
  if (session_ == nullptr) {
    return true;
  }
  const auto& virtual_lane_manager = session_->environmental_model().get_virtual_lane_manager();
  const auto& target_lane = virtual_lane_manager->get_lane_with_virtual_id(target_lane_virtual_id);
  if (target_lane == nullptr) {
    return true;
  }
  const auto& target_ref = target_lane->get_reference_path();
  if (target_ref == nullptr) {
    return true;
  }
  const double ego_s = target_ref->get_frenet_ego_state().s();
  // 遍历自车前方 80m 参考点平均曲率
  double curv_sum = 0.0;
  int count = 0;
  constexpr int kMinValidPointCount = 5;
  for (int idx = 0; idx * 2.0 < 80.0; idx++) {
    ReferencePathPoint refpath_pt;
    if (target_ref->get_reference_point_by_lon(ego_s + idx * 2.0, refpath_pt)) {
      double kappa = refpath_pt.path_point.kappa();
      // 检查曲率值有效性
      if (std::isnan(kappa) || std::isinf(kappa)) {
        continue;  // 跳过无效的曲率值
      }
      curv_sum += std::abs(kappa);
      count++;
    }
  }
  // 如果有效点数不足，采用保守策略：抑制变道
  if (count < kMinValidPointCount) {
    return true;
  }
  double average_curve = curv_sum / count;  // 计算平均曲率
  const double curve_condition = 1.0 / kCurveRadiusThreshold;
  return average_curve > curve_condition;
}
bool LaneChangeRequest::IsMLCIgnoreSolidLaneCheck(
    const RequestType& lc_request,
    const RequestSource& lc_request_source) const {
  if (lc_request_source != RequestSource::MAP_REQUEST) {
    return false;
  }
  const auto& route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();
  bool is_process_split =
      route_info_output.mlc_decider_scene_type_info.mlc_scene_type ==
      SPLIT_SCENE;

  bool is_mlc_avoidance =
      route_info_output.mlc_decider_scene_type_info.mlc_scene_type ==
          AVOID_SPLIT ||
      route_info_output.mlc_decider_scene_type_info.mlc_scene_type ==
          AVOID_MERGE;

  int minVal_seq = 1000;
  int maxVal_seq = -1000;
  for (const auto& seq : route_info_output.feasible_lane_sequence) {
    if (seq < minVal_seq) {
      minVal_seq = seq;
    }

    if (seq > maxVal_seq) {
      maxVal_seq = seq;
    }
  }
  // bool is_one_lane_gap = lc_request == RIGHT_CHANGE
  //                            ? (minVal_seq - route_info_output.ego_seq) == 1
  //                            : (route_info_output.ego_seq - maxVal_seq) == 1;

  const bool is_satisfy_dis_condition =
      route_info_output.mlc_decider_scene_type_info
          .dis_to_link_topo_change_point < 100.0;
  if (is_process_split && !is_mlc_avoidance && is_satisfy_dis_condition) {
    return true;
  }
  return false;
}

bool LaneChangeRequest::IsPathCollisionWithRoadEdge(
    const std::shared_ptr<VirtualLaneManager>& virtual_lane_mgr,
    int origin_lane_id, int target_lane_id,
    const TrajectoryPoints& path_points) {
  const double road_edge_buffer = 0.1;
  const auto origin_lane =
      virtual_lane_mgr->get_lane_with_virtual_id(origin_lane_id);
  if (origin_lane == nullptr) {
    return true;
  }
  const auto& origin_lane_points = origin_lane->lane_points();
  if (origin_lane_points.empty()) {
    return true;
  }

  const auto target_lane =
      virtual_lane_mgr->get_lane_with_virtual_id(target_lane_id);
  const bool has_target_lane = target_lane != nullptr;
  const std::vector<iflyauto::ReferencePoint>* target_lane_points_ptr =
      has_target_lane ? &target_lane->lane_points() : nullptr;

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double half_vehicle_width = vehicle_param.width * 0.5;

  for (size_t i = 0; i < path_points.size(); ++i) {
    const double pt_x = path_points[i].x;
    const double pt_y = path_points[i].y;

    // 在 origin_lane 上找最近点
    size_t origin_nearest_idx = 0;
    double origin_min_dist_sq = std::numeric_limits<double>::max();
    for (size_t j = 0; j < origin_lane_points.size(); ++j) {
      const double dx = origin_lane_points[j].enu_point.x - pt_x;
      const double dy = origin_lane_points[j].enu_point.y - pt_y;
      const double dist_sq = dx * dx + dy * dy;
      if (dist_sq < origin_min_dist_sq) {
        origin_min_dist_sq = dist_sq;
        origin_nearest_idx = j;
      }
    }

    // 在 target_lane 上找最近点
    size_t target_nearest_idx = 0;
    double target_min_dist_sq = std::numeric_limits<double>::max();
    const bool target_valid =
        has_target_lane && target_lane_points_ptr != nullptr &&
        !target_lane_points_ptr->empty();
    if (target_valid) {
      for (size_t j = 0; j < target_lane_points_ptr->size(); ++j) {
        const double dx = (*target_lane_points_ptr)[j].enu_point.x - pt_x;
        const double dy = (*target_lane_points_ptr)[j].enu_point.y - pt_y;
        const double dist_sq = dx * dx + dy * dy;
        if (dist_sq < target_min_dist_sq) {
          target_min_dist_sq = dist_sq;
          target_nearest_idx = j;
        }
      }
    }

    // 选择距离更近的 lane
    const bool use_target =
        target_valid && target_min_dist_sq < origin_min_dist_sq;
    const auto& nearest_pt =
        use_target ? (*target_lane_points_ptr)[target_nearest_idx]
                   : origin_lane_points[origin_nearest_idx];
    const double ref_x = nearest_pt.enu_point.x;
    const double ref_y = nearest_pt.enu_point.y;
    const double ref_heading = nearest_pt.enu_heading;

    const double dx = pt_x - ref_x;
    const double dy = pt_y - ref_y;
    const double l = -dx * std::sin(ref_heading) + dy * std::cos(ref_heading);

    const double left_vehicle_edge = l + half_vehicle_width;
    const double right_vehicle_edge = l - half_vehicle_width;
    if (left_vehicle_edge >
        nearest_pt.distance_to_left_road_border - road_edge_buffer) {
      return true;
    }
    if (right_vehicle_edge <
        -nearest_pt.distance_to_right_road_border + road_edge_buffer) {
      return true;
    }
  }
  return false;
}

}  // namespace planning