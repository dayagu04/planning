#include "modules/tasks/behavior_planners/hmi_decider/lane_change_hmi/lane_change_hmi_decider.h"

#include "common/ifly_time.h"
#include "modules/context/environmental_model_manager.h"
#include "modules/context/planning_context.h"
namespace planning {
namespace {
static constexpr int kHmiSendMsgCntThreshold = 5;
}
LaneChangeHmiDecider::LaneChangeHmiDecider(framework::Session* session)
    : session_(session) {}

bool LaneChangeHmiDecider::Execute() {
  if (!session_) {
    return false;
  }
  UpdateHMIInfo();
  UpdateTurnSignal();
  return true;
}

void LaneChangeHmiDecider::UpdateTurnSignal() {
  auto& planning_result =
      session_->mutable_planning_context()->mutable_planning_result();
  planning_result.turn_signal = RequestType::NO_CHANGE;
  bool active = session_->environmental_model().GetVehicleDbwStatus();
  if (!active) {
    // planning_result.turn_signal = RequestType::NO_CHANGE;
    return;
  }
  const auto& lane_borrow_decider_output =
      session_->planning_context().lane_borrow_decider_output();
  bool turn_signal_on_from_lane_borrow =
      lane_borrow_decider_output.lane_borrow_state ==
          LaneBorrowStatus::kLaneBorrowDriving ||
      lane_borrow_decider_output.lane_borrow_state ==
          LaneBorrowStatus::kLaneBorrowCrossing;
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  bool turn_signal_from_ramp_direction =
      lane_change_decider_output.dir_turn_signal_road_to_ramp !=
      RampDirection::RAMP_NONE;
  bool turn_signal_from_lane_change =
      lane_change_decider_output.lc_request != 0;
  bool is_dynamic_agent_emergency_lane_change =
      lane_change_decider_output.lc_request_source ==
      DYNAMIC_AGENT_EMERGENCE_AVOID_REQUEST;
  bool is_overtake_lane_change =
      lane_change_decider_output.lc_request_source ==
      OVERTAKE_REQUEST;
  bool is_pre_avoid_link_merge_mlc = false;
  const auto& route_info = session_->environmental_model().get_route_info();
  if (route_info != nullptr) {
    is_pre_avoid_link_merge_mlc =
        route_info->get_route_info_output()
                .mlc_decider_scene_type_info.mlc_scene_type == AVOID_MERGE &&
        lane_change_decider_output.lc_request_source == MAP_REQUEST;
  }
  const auto curr_state = lane_change_decider_output.curr_state;

  const bool is_normal_lane_change_scenario =
      turn_signal_from_lane_change && !is_dynamic_agent_emergency_lane_change &&
      !is_pre_avoid_link_merge_mlc && !is_overtake_lane_change;

  // 针对紧急变道、躲避link_merge的mlc、超车变道，只有进入到execution了才会打灯，其余不变
  const bool is_special_lane_change_scenario =
      (is_dynamic_agent_emergency_lane_change || is_pre_avoid_link_merge_mlc ||
       is_overtake_lane_change) &&
      turn_signal_from_lane_change &&
      (curr_state == kLaneChangeExecution ||
       curr_state == kLaneChangeComplete || curr_state == kLaneChangeHold ||
       curr_state == kLaneChangeCancel);

  if (is_normal_lane_change_scenario || is_special_lane_change_scenario) {
    planning_result.turn_signal = lane_change_decider_output.lc_request == 1
                                      ? RequestType::LEFT_CHANGE
                                      : RequestType::RIGHT_CHANGE;
    return;
  }
  if (turn_signal_from_ramp_direction) {
    planning_result.turn_signal =
        lane_change_decider_output.dir_turn_signal_road_to_ramp ==
                RampDirection::RAMP_ON_LEFT
            ? RequestType::LEFT_CHANGE
            : RequestType::RIGHT_CHANGE;
    return;
  }

  if (turn_signal_on_from_lane_borrow) {
    planning_result.turn_signal = lane_borrow_decider_output.borrow_direction ==
                                          BorrowDirection::LEFT_BORROW
                                      ? RequestType::LEFT_CHANGE
                                      : RequestType::RIGHT_CHANGE;
    return;
  }
}

void LaneChangeHmiDecider::UpdateHMIInfo() {
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto& route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();
  auto& ad_info = session_->mutable_planning_context()
                      ->mutable_planning_hmi_info()
                      ->ad_info;
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  // ad_info.cruise_speed = ego_state_manager->ego_v_cruise();
  ad_info.lane_change_direction =
      (iflyauto::LaneChangeDirection)lane_change_decider_output.lc_request;
  // update LaneChangeStatus
  const auto curr_state = lane_change_decider_output.curr_state;
  const auto lasr_frame_state = session_->planning_context()
                                    .lane_change_decider_output()
                                    .coarse_planning_info.source_state;
  static double lc_complete_to_lk_time = 0.0;
  const auto dir_turn_signal_road_to_ramp =
      lane_change_decider_output.dir_turn_signal_road_to_ramp;
  static int lane_change_complete_cnt = kHmiSendMsgCntThreshold;
  if (curr_state == kLaneKeeping) {
    if (lane_change_complete_cnt < kHmiSendMsgCntThreshold) {
      ad_info.lane_change_status =
          iflyauto::LaneChangeStatus::LC_STATE_COMPLETE;
      lane_change_complete_cnt++;
    } else if (lasr_frame_state == kLaneChangeComplete) {
      lane_change_complete_cnt = 1;
      lc_complete_to_lk_time = IflyTime::Now_ms();
      ad_info.lane_change_status =
          iflyauto::LaneChangeStatus::LC_STATE_COMPLETE;

    } else {
      ad_info.lane_change_status =
          iflyauto::LaneChangeStatus::LC_STATE_NO_CHANGE;

      if (session_->is_hpp_scene()) {
        // for HPP turn signal road to ramp
        const auto hpp_turn_signal = lane_change_decider_output.hpp_turn_signal;
        if (hpp_turn_signal == NO_CHANGE) {
          ad_info.lane_change_status =
              iflyauto::LaneChangeStatus::LC_STATE_NO_CHANGE;
        } else if (hpp_turn_signal == LEFT_CHANGE) {
          ad_info.lane_change_direction =
              iflyauto::LaneChangeDirection::LC_DIR_LEFT;
        } else if (hpp_turn_signal == RIGHT_CHANGE) {
          ad_info.lane_change_direction =
              iflyauto::LaneChangeDirection::LC_DIR_RIGHT;
        }
      } else {
        // for NOA turn signal road to ramp
        // 自车在滑入匝道时，需要更新ad_info.lane_change_reason，已供hmi模块使用

        if (dir_turn_signal_road_to_ramp == RAMP_NONE) {
          if (last_frame_dir_turn_signal_road_to_ramp_ != RAMP_NONE) {
            ad_info.lane_change_status =
                iflyauto::LaneChangeStatus::LC_STATE_COMPLETE;
            ad_info.lane_change_reason =
                iflyauto::LaneChangeReason::LC_REASON_SPLIT;
            lc_state_complete_frame_nums_ = 1;
          }
          // 需要给hmi模块连续发3s的complete状态
          if (lc_state_complete_frame_nums_ <= 30) {
            ad_info.lane_change_status =
                iflyauto::LaneChangeStatus::LC_STATE_COMPLETE;
            ad_info.lane_change_reason =
                iflyauto::LaneChangeReason::LC_REASON_SPLIT;
            lc_state_complete_frame_nums_++;
          } else {
            ad_info.lane_change_status =
                iflyauto::LaneChangeStatus::LC_STATE_NO_CHANGE;
            lc_state_complete_frame_nums_ = 31;
          }

        } else if (dir_turn_signal_road_to_ramp == RAMP_ON_LEFT) {
          lc_state_complete_frame_nums_ = 31;
          ad_info.lane_change_direction =
              iflyauto::LaneChangeDirection::LC_DIR_LEFT;
          ad_info.lane_change_status =
              iflyauto::LaneChangeStatus::LC_STATE_STARTING;
          ad_info.lane_change_reason =
              iflyauto::LaneChangeReason::LC_REASON_SPLIT;
        } else if (dir_turn_signal_road_to_ramp == RAMP_ON_RIGHT) {
          lc_state_complete_frame_nums_ = 31;
          ad_info.lane_change_direction =
              iflyauto::LaneChangeDirection::LC_DIR_RIGHT;
          ad_info.lane_change_status =
              iflyauto::LaneChangeStatus::LC_STATE_STARTING;
          ad_info.lane_change_reason =
              iflyauto::LaneChangeReason::LC_REASON_SPLIT;
        }
      }
    }
  } else if (curr_state == kLaneChangePropose) {
    ad_info.lane_change_status = iflyauto::LaneChangeStatus::LC_STATE_WAITING;
  } else if (curr_state == kLaneChangeExecution) {
    ad_info.lane_change_status = iflyauto::LaneChangeStatus::LC_STATE_STARTING;
  } else if (curr_state == kLaneChangeComplete) {
    ad_info.lane_change_status = iflyauto::LaneChangeStatus::LC_STATE_STARTING;
  } else if (curr_state == kLaneChangeCancel || curr_state == kLaneChangeHold) {
    ad_info.lane_change_status = iflyauto::LaneChangeStatus::LC_STATE_CANCELLED;
  }
  last_frame_dir_turn_signal_road_to_ramp_ = dir_turn_signal_road_to_ramp;

  // update StatusUpdateReason
  const auto int_request_cancel_reason =
      lane_change_decider_output.int_request_cancel_reason;
  const auto lc_invalid_reason = lane_change_decider_output.lc_invalid_reason;
  const auto lc_back_reason = lane_change_decider_output.lc_back_reason;
  static int proposal_time_out_cnt = kHmiSendMsgCntThreshold;
  static int manual_cancle_cnt = kHmiSendMsgCntThreshold;
  if (int_request_cancel_reason == MANUAL_CANCEL) {
    manual_cancle_cnt = 1;
  }
  if (lane_change_decider_output.lc_invalid_reason == "propose time out") {
    proposal_time_out_cnt = 1;
  }

  if (proposal_time_out_cnt < kHmiSendMsgCntThreshold) {
    ad_info.status_update_reason =
        iflyauto::StatusUpdateReason::STATUS_UPDATE_REASON_TIMEOUT;
    proposal_time_out_cnt++;
  } else if (lc_back_reason == "dash line length not satisfy" ||
             lane_change_decider_output.lc_invalid_reason ==
                 "dash not enough") {
    ad_info.status_update_reason =
        iflyauto::StatusUpdateReason::STATUS_UPDATE_REASON_SOLID_LINE;
  } else if (lc_invalid_reason == "side view invalid" ||
             lc_invalid_reason == "front view invalid" ||
             lc_back_reason == "side view back" ||
             lc_back_reason == "front view back" ||
             lc_back_reason == "but back cnt below threshold") {
    ad_info.status_update_reason =
        iflyauto::StatusUpdateReason::STATUS_UPDATE_REASON_SIDE_VEH;
    iflyauto::ObstacleInfo obstacle;
    obstacle.id = lane_change_decider_output.lc_invalid_track.track_id;
    ad_info.obstacle_info[0] = obstacle;
    ad_info.obstacle_info_size = 1;
  } else if (int_request_cancel_reason == SOLID_LC) {
    ad_info.status_update_reason =
        iflyauto::StatusUpdateReason::STATUS_UPDATE_REASON_SOLID_LINE;
    // 暂时为了满足实线变道时打灯合planing_hmi的提示需求
    // 在此更新变道状态和变道方向的值！！！！！！！
    //  TODO(fengwang31):在变道过程中，遇到实线取消了，是否需要发出方向？
    ad_info.lane_change_direction =
        (iflyauto::LaneChangeDirection)
            lane_change_decider_output.lc_request;  // LC_DIR_LEFT/RIGHT
    ad_info.lane_change_status = iflyauto::LaneChangeStatus::LC_STATE_NO_CHANGE;
  } else if (manual_cancle_cnt < kHmiSendMsgCntThreshold) {
    ad_info.status_update_reason =
        iflyauto::StatusUpdateReason::STATUS_UPDATE_REASON_MANUAL_CANCEL;
    manual_cancle_cnt++;
  } else {
    ad_info.status_update_reason =
        iflyauto::StatusUpdateReason::STATUS_UPDATE_REASON_NONE;
  }

  // update LaneChangeReason
  const auto& ego_lane_road_right_decider_output =
      session_->planning_context().ego_lane_road_right_decider_output();
  const bool is_merge_region =
      ego_lane_road_right_decider_output.is_merge_region;
  const auto& function_info = session_->environmental_model().function_info();
  const double dis_to_merge =
      route_info_output.map_merge_region_info_list.empty()
          ? NL_NMAX
          : route_info_output.map_merge_region_info_list[0].distance_to_merge_point;
  const double dis_to_split =
      route_info_output.map_split_region_info_list.empty()
          ? NL_NMAX
          : route_info_output.map_split_region_info_list[0].distance_to_split_point;
  const auto lc_request_source = lane_change_decider_output.lc_request_source;

  if (lc_request_source == NO_REQUEST &&
      lane_change_decider_output.dir_turn_signal_road_to_ramp == RAMP_NONE) {
    ad_info.lane_change_reason = iflyauto::LaneChangeReason::LC_REASON_NONE;
  } else if (lc_request_source == INT_REQUEST) {
    ad_info.lane_change_reason = iflyauto::LaneChangeReason::LC_REASON_MANUAL;
  } else if (lc_request_source == OVERTAKE_REQUEST) {
    ad_info.lane_change_reason =
        iflyauto::LaneChangeReason::LC_REASON_SLOWING_VEH;
  } else if (lc_request_source == MAP_REQUEST) {
    const double dis_to_merge =
        route_info_output.map_merge_region_info_list.empty()
            ? NL_NMAX
            : route_info_output.map_merge_region_info_list[0]
                  .distance_to_merge_point;
    // if (route_info_output.dis_to_ramp < 100.0 &&
    //     route_info_output.dis_to_ramp < dis_to_merge) {
    //   ad_info.lane_change_reason =
    //   iflyauto::LaneChangeReason::LC_REASON_SPLIT;
    // } else
    if (route_info_output.mlc_decider_scene_type_info.mlc_scene_type == MERGE_SCENE) {
      ad_info.lane_change_reason = iflyauto::LaneChangeReason::LC_REASON_MERGE;
    } else {
      ad_info.lane_change_reason =
          iflyauto::LaneChangeReason::LC_REASON_NAVIGATION;
    }
  } else if (lc_request_source == MERGE_REQUEST) {
    if (route_info_output.mlc_decider_scene_type_info.mlc_scene_type == MERGE_SCENE) {
      ad_info.lane_change_reason = iflyauto::LaneChangeReason::LC_REASON_MERGE;
    } else {
      ad_info.lane_change_reason =
          iflyauto::LaneChangeReason::LC_REASON_NAVIGATION;
    }
    // ad_info.lane_change_reason = iflyauto::LaneChangeReason::LC_REASON_MERGE;
  }

  // update route info
  if (!route_info_output.is_on_ramp) {
    ad_info.distance_to_ramp = route_info_output.dis_to_ramp;
  } else {
    ad_info.distance_to_ramp = NL_NMAX;
  }
  ad_info.distance_to_split = dis_to_split;
  bool is_ramp_merge_to_road_on_expressway = false;
  const auto& route_info_sdpro_map =
      session_->environmental_model().get_route_info()->get_sdpro_map();
  if (!route_info_output.map_merge_region_info_list.empty()) {
    const auto& merge_seg = route_info_sdpro_map.GetLinkOnRoute(
        route_info_output.map_merge_region_info_list[0].merge_link_id);
    const auto& merge_seg_last_seg =
        route_info_sdpro_map.GetPreviousLinkOnRoute(
            route_info_output.map_merge_region_info_list[0].merge_link_id);
    if (merge_seg != nullptr && merge_seg_last_seg != nullptr) {
      if ((route_info_sdpro_map.isRamp(merge_seg_last_seg->link_type()) ||
           route_info_sdpro_map.isSaPa(merge_seg_last_seg->link_type())) &&
          !route_info_sdpro_map.isRamp(merge_seg->link_type()) &&
          (route_info_sdpro_map.isSaPa(merge_seg_last_seg->link_type()) ||
           route_info_output.is_on_ramp)) {
        is_ramp_merge_to_road_on_expressway = true;
      }
    }
  }
  if ((route_info_output.mlc_decider_scene_type_info.mlc_scene_type ==
           MERGE_SCENE ||
       is_ramp_merge_to_road_on_expressway) &&
      route_info_output.is_on_ramp) {
    ad_info.distance_to_merge = dis_to_merge;
  } else {
    if (is_merge_region) {
      ad_info.distance_to_merge = ego_lane_road_right_decider_output.merge_point_distance;
    } else {
      ad_info.distance_to_merge = NL_NMAX;
    }
  }
  ad_info.distance_to_toll_station = route_info_output.distance_to_toll_station;
  ad_info.noa_exit_warning_level_distance =
      route_info_output.distance_to_route_end;
  // ad_info.distance_to_tunnel = ;  //
  // ad_info.is_within_hdmap = ;     //
  const int ramp_direction = route_info_output.ramp_direction;
  ad_info.ramp_direction = (iflyauto::RampDirection)ramp_direction;

  const auto& fix_reference_path =
      lane_change_decider_output.coarse_planning_info.reference_path;
  if (fix_reference_path != nullptr) {
    ad_info.dis_to_reference_line =
        std::abs(fix_reference_path->get_frenet_ego_state().l() * 100);
    ad_info.angle_to_roaddirection =
        fix_reference_path->get_frenet_ego_state().heading_angle();
  }

  ad_info.is_in_sdmaproad = route_info_output.is_in_sdmaproad;
  if (route_info_output.is_ego_on_expressway_hmi) {
    ad_info.road_type = iflyauto::DrivingRoadType::DRIVING_ROAD_TYPE_HIGHWAY;
    // update RampPassSts
    // 高速主路上距离匝道距离小于200m，且不在一分二车道的场景
    if (route_info_output.dis_to_ramp < 200 &&
        lane_change_decider_output.dir_turn_signal_road_to_ramp == RAMP_NONE &&
        !route_info_output.is_on_ramp) {
      if (ramp_direction == iflyauto::RAMP_LEFT &&
          !lane_change_decider_output.is_ego_on_leftmost_lane) {
        ad_info.ramp_pass_sts = iflyauto::RAMP_PASS_STS_READYTOMISS;
      } else if (ramp_direction == iflyauto::RAMP_RIGHT &&
                 !lane_change_decider_output.is_ego_on_rightmost_lane) {
        ad_info.ramp_pass_sts = iflyauto::RAMP_PASS_STS_READYTOMISS;
      } else {
        ad_info.ramp_pass_sts = iflyauto::RAMP_PASS_STS_NONE;
      }
    } else {
      ad_info.ramp_pass_sts = iflyauto::RAMP_PASS_STS_NONE;
    }
  } else if (route_info_output.is_ego_on_city_expressway_hmi) {
    ad_info.road_type = iflyauto::DrivingRoadType::DRIVING_ROAD_TYPE_OVERPASS;
    // update RampPassSts
    // 城区主路上距离匝道距离小于50m，且不在一分二车道的场景
    if (route_info_output.dis_to_ramp < 50 &&
        lane_change_decider_output.dir_turn_signal_road_to_ramp == RAMP_NONE &&
        !route_info_output.is_on_ramp) {
      if (ramp_direction == iflyauto::RAMP_LEFT &&
          !lane_change_decider_output.is_ego_on_leftmost_lane) {
        ad_info.ramp_pass_sts = iflyauto::RAMP_PASS_STS_READYTOMISS;
      } else if (ramp_direction == iflyauto::RAMP_RIGHT &&
                 !lane_change_decider_output.is_ego_on_rightmost_lane) {
        ad_info.ramp_pass_sts = iflyauto::RAMP_PASS_STS_READYTOMISS;
      } else {
        ad_info.ramp_pass_sts = iflyauto::RAMP_PASS_STS_NONE;
      }
    } else {
      ad_info.ramp_pass_sts = iflyauto::RAMP_PASS_STS_NONE;
    }
  } else {
    ad_info.road_type = iflyauto::DrivingRoadType::DRIVING_ROAD_TYPE_NONE;
    ad_info.ramp_pass_sts = iflyauto::RAMP_PASS_STS_NONE;
  }
  ad_info.reference_line_msg = session_->environmental_model()
                                   .get_virtual_lane_manager()
                                   ->get_current_lane()
                                   ->get_reference_line_msg();
  ad_info.landing_point.is_avaliable = false;
  ad_info.landing_point.heading = 0.0;
  ad_info.landing_point.relative_pos.x = 0.0;
  ad_info.landing_point.relative_pos.y = 0.0;
  ad_info.landing_point.relative_pos.z = 0.0;
  JSON_DEBUG_VALUE("ramp_pass_sts", (int)ad_info.ramp_pass_sts)
  double lc_complete_time_period = IflyTime::Now_ms() - lc_complete_to_lk_time;
  static constexpr double kMaxTimeToCompleteLaneChange = 7000.0;  // ms
  bool is_lane_keeping = curr_state == kLaneKeeping;
  bool is_time_out = lc_complete_time_period > kMaxTimeToCompleteLaneChange;
  iflyauto::LandingPoint landing_point;
  if (!is_time_out && is_lane_keeping) {
    landing_point = CalculateLandingPoint(true, lane_change_decider_output);
    landing_point.is_avaliable = true;
  } else if (curr_state == kLaneChangePropose ||
             curr_state == kLaneChangeExecution ||
             curr_state == kLaneChangeComplete ||
             curr_state == kLaneChangeCancel || curr_state == kLaneChangeHold) {
    landing_point = CalculateLandingPoint(false, lane_change_decider_output);
    landing_point.is_avaliable = true;
    // }
  } else {
    landing_point = CalculateLandingPoint(true, lane_change_decider_output);
    landing_point.is_avaliable = false;
  }
  ad_info.landing_point = landing_point;
  JSON_DEBUG_VALUE("lane_change_reason",
                   static_cast<int>(ad_info.lane_change_reason))
  JSON_DEBUG_VALUE("status_update_reason",
                   static_cast<int>(ad_info.status_update_reason))
  JSON_DEBUG_VALUE("lane_change_status",
                   static_cast<int>(ad_info.lane_change_status))
  JSON_DEBUG_VALUE("lane_change_direction",
                   static_cast<int>(ad_info.lane_change_direction))

  return;
}

iflyauto::LandingPoint LaneChangeHmiDecider::CalculateLandingPoint(
    bool is_lane_keeping,
    const LaneChangeDeciderOutput& lane_change_decider_output) {
  iflyauto::LandingPoint landing_point;
  landing_point.relative_pos.x = 0.0;
  landing_point.relative_pos.y = 0.0;
  landing_point.relative_pos.z = 0.0;
  landing_point.heading = 0.0;
  std::shared_ptr<ReferencePath> target_reference = nullptr;
  double l_velocity = 1.4;
  auto ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  double ego_v = std::fmax(1.0, ego_state_manager->ego_v_cruise() * 0.2);
  double l_min = 0.08;
  if (is_lane_keeping) {
    target_reference = session_->environmental_model()
                           .get_reference_path_manager()
                           ->get_reference_path_by_current_lane();

  } else {
    int target_reference_virtual_id =
        lane_change_decider_output.target_lane_virtual_id;
    target_reference =
        session_->environmental_model()
            .get_reference_path_manager()
            ->get_reference_path_by_lane(target_reference_virtual_id, false);
  }
  if (target_reference != nullptr) {
    double l = std::fabs(target_reference->get_frenet_ego_state().l());
    // double t_ratio = std::fabs(l) / std::fabs(l_max);
    double t = std::fmax(0.0, l - l_min) / l_velocity;

    if (l < l_min) {
      t = 0.0;
    }

    Point2D cart_point;
    if (!target_reference->get_frenet_coord()->SLToXY(
            Point2D(target_reference->get_frenet_ego_state().s() + t * ego_v,
                    0),
            cart_point)) {
      return landing_point;
    }
    const auto& ego_pose = ego_state_manager->ego_pose();
    const double theta_ori = ego_pose.theta;
    double landing_point_theta_global = 0;
    ReferencePathPoint reference_path_point{};
    if (target_reference->get_reference_point_by_lon(
            target_reference->get_frenet_ego_state().s() + t * ego_v,
            reference_path_point)) {
      landing_point_theta_global = reference_path_point.path_point.theta();
    }
    Eigen::Vector2d pos_n_ori(ego_pose.x, ego_pose.y);
    // pnc::geometry_lib::GlobalToLocalTf global_to_local_tf(pos_n_ori,
    // theta_ori); Eigen::Vector2d p_n(cart_point.x, cart_point.y);
    // Eigen::Vector2d landing_point_body = global_to_local_tf.GetPos(p_n);
    // const double landing_point_theta_local =
    //     global_to_local_tf.GetHeading(landing_point_theta_global);

    landing_point.relative_pos.x = cart_point.x;
    landing_point.relative_pos.y = cart_point.y;
    landing_point.relative_pos.z = 0;
    landing_point.heading = landing_point_theta_global;
  }
  return landing_point;
}
}  // namespace planning