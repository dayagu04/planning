#include "lane_change_requests/active_lane_change_request.h"

#include "active_lane_change_request.h"
#include "lateral_behavior_object_selector.h"
#include "planning_context.h"
#include "virtual_lane_manager.h"

namespace planning {
// class: ActRequest
ActRequest::ActRequest(
    planning::framework::Session *session,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : LaneChangeRequest(session, virtual_lane_mgr, lane_change_lane_mgr) {
  auto config_builder = session_->mutable_environmental_model()->config_builder(
      planning::common::SceneType::HIGHWAY);
  act_request_config_ = config_builder->cast<ActRequestConfig>();
}

void ActRequest::Update(int lc_status, double start_move_distolane,
                        double lc_int_tfinish, double lc_map_tfinish,
                        bool accident_ahead, bool not_accident) {
  std::cout << "coming active lane change request" << std::endl;
  double default_int_delay = 2;
  double default_ma_delay = 1.5;
  double diff_int = IflyTime::Now_s() - lc_int_tfinish;
  double diff_map = IflyTime::Now_s() - lc_map_tfinish;

  auto &ego_state = session_->environmental_model().get_ego_state_manager();
  // 获取相邻车道障碍物信息
  auto &object_selector = session_->planning_context().object_selector();
  double v_rel_l = object_selector->v_rel_l();
  double v_rel_r = object_selector->v_rel_r();
  double v_rel_f = object_selector->v_rel_f();
  bool neg_left_alc_car = object_selector->neg_left_alc_car();
  bool neg_right_alc_car = object_selector->neg_right_alc_car();
  auto &left_alc_car = object_selector->left_alc_car();
  auto &right_alc_car = object_selector->right_alc_car();
  int front_tracks_l_cnt = object_selector->front_tracks_l_cnt();
  int front_tracks_r_cnt = object_selector->front_tracks_r_cnt();

  auto flane = lane_change_lane_mgr_->flane();
  auto olane = lane_change_lane_mgr_->olane();
  auto tlane = lane_change_lane_mgr_->tlane();

  double olane_width = flane->width();
  double v_ego = ego_state->ego_v();
  double dt_delay = v_ego < 5.0 ? 0.0 : 0.03;
  int left_lane_index = 0;
  int right_lane_index = virtual_lane_mgr_->get_lane_num() - 1;

  auto current_lane = virtual_lane_mgr_->get_current_lane();
  auto current_lane_index = virtual_lane_mgr_->get_lane_index(current_lane);
  if (virtual_lane_mgr_->lc_map_decision(current_lane) == -1 &&
      virtual_lane_mgr_->lc_map_decision_offset(current_lane) < 150) {
    default_int_delay = 0.;
    default_ma_delay = 0.;
  }

  auto current_lane_virtual_id = virtual_lane_mgr_->current_lane_virtual_id();
  if (lane_change_lane_mgr_->has_origin_lane()) {
    auto origin_lane = lane_change_lane_mgr_->olane();
    origin_lane_virtual_id_ = origin_lane->get_virtual_id();
  } else {
    origin_lane_virtual_id_ = current_lane_virtual_id;
  }
  int target_lane_virtual_id_tmp{current_lane_virtual_id};

  LOG_DEBUG(
      "[ActRequest::update] current_lane_virtual_id: %d, "
      "origin_lane_virtual_id_: %d, target_lane_virtual_id_: %d \n",
      current_lane_virtual_id, origin_lane_virtual_id_,
      target_lane_virtual_id_);
  // if (olane.has_master() && !map_info.is_in_intersection()) {
  //   olane_width = olane.width();
  // } else if (map_info.is_in_intersection()) {
  //   olane_width = 3.6;
  // }
  olane_width = 3.6;  // hack
  double act_cancel_thr = std::max(olane_width / 2 - 0.6, 0.);
  bool l_change_cond = (lane_change_lane_mgr_->has_target_lane() &&
                        flane->get_ego_lateral_offset() + v_ego * dt_delay <
                            start_move_distolane + act_cancel_thr);

  bool r_change_cond = (lane_change_lane_mgr_->has_target_lane() &&
                        flane->get_ego_lateral_offset() - v_ego * dt_delay >
                            start_move_distolane - act_cancel_thr);

  // auto traffic_light_direction = map_info.traffic_light_direction();
  // bool curr_direct_exist =
  //     (current_lane->get_lane_marks() == DIRECTION_UNKNOWN &&
  //      current_lane->get_lane_type() ==
  //      FusionRoad::LaneType::LANETYPE_NORMAL);
  // hack curr_direct_exist until intersection info ready
  bool curr_direct_exist =
      (current_lane->get_lane_marks() ==
       FusionRoad::LaneDrivableDirection::DIRECTION_STRAIGHT);
  // 获取左右车道的index 换成获取id
  if (current_lane_index > 0) {
    left_lane_index = current_lane_index - 1;
  }
  if (current_lane_index <= virtual_lane_mgr_->get_lane_num() - 1) {
    right_lane_index = current_lane_index + 1;
  }

  // 判断道路条件lane_change_condition是否可以进行换道
  const bool enable_act_request =
      act_request_config_.enable_act_request_function;
  const double act_request_speed_limit =
      act_request_config_.enable_speed_threshold / 3.6;
  const int kDistanceBuffer = 1000;
  // 道路限速
  auto v_limit_map = current_lane->velocity_limit();
  bool is_on_highway = session_->environmental_model().is_on_highway();
  bool is_not_on_ramp = true;  // hack !map_info.is_on_ramp();
  // 判断是否为最左侧车道
  bool is_not_on_most_left_lane = current_lane_index != 0;
  bool is_normal_lane =
      current_lane->get_lane_type() == FusionRoad::LaneType::LANETYPE_NORMAL;

  auto distance_to_merge_point =
      current_lane->get_lane_merge_split_point().merge_split_point_data_size() >
              0
          ? current_lane->get_lane_merge_split_point()
                .merge_split_point_data(0)
                .distance()
          : 5000.;  // hack
  // WB hack: distance_to_y_point计算需要更新
  auto distance_to_y_point =
      current_lane->get_lane_merge_split_point().merge_split_point_data_size() >
              0
          ? current_lane->get_lane_merge_split_point()
                .merge_split_point_data(0)
                .distance()
          : 5000.;  // hack
  // 判断分汇流情况目前不完善，TBD
  bool is_nearby_right_merge_point =
      current_lane->get_lane_merge_split_point().merge_split_point_data_size() >
          0 &&
      current_lane->get_lane_merge_split_point()
              .merge_split_point_data(0)
              .orientation() ==
          FusionRoad::LaneOrientation::ORIENTATION_RIGHT &&
      distance_to_merge_point > 0 &&
      distance_to_merge_point < kDistanceBuffer &&
      distance_to_merge_point + kDistanceBuffer <
          virtual_lane_mgr_->dis_to_ramp();

  bool is_nearby_right_merge_point_lc_valid =
      (virtual_lane_mgr_->lc_map_decision(current_lane) >= 0 &&
       distance_to_merge_point + kDistanceBuffer <
           virtual_lane_mgr_->lc_map_decision_offset(current_lane)) ||
      virtual_lane_mgr_->lc_map_decision(current_lane) < 0;

  // WB TBD： 汇流判断目前为hack，需要更新逻辑
  bool is_nearby_right_y_point =
      current_lane->get_lane_merge_split_point().merge_split_point_data_size() >
          0 &&
      current_lane->get_lane_merge_split_point()
              .merge_split_point_data(0)
              .orientation() ==
          FusionRoad::LaneOrientation::ORIENTATION_RIGHT &&
      distance_to_y_point > 0 && distance_to_y_point < kDistanceBuffer &&
      distance_to_y_point + kDistanceBuffer < virtual_lane_mgr_->dis_to_ramp();
  bool is_from_right_y_point_lc_decision_valid =
      (virtual_lane_mgr_->lc_map_decision(current_lane) >= 0 &&
       distance_to_y_point + kDistanceBuffer <
           virtual_lane_mgr_->lc_map_decision_offset(current_lane)) ||
      virtual_lane_mgr_->lc_map_decision(current_lane) < 0;
  // 整合分汇流附近换道条件：
  bool lc_condition_nearby_merge =
      enable_act_request && v_limit_map >= act_request_speed_limit &&
      is_on_highway && is_not_on_ramp &&
      ((is_nearby_right_merge_point && is_nearby_right_merge_point_lc_valid &&
        front_tracks_r_cnt > 0) ||
       (is_nearby_right_y_point && is_from_right_y_point_lc_decision_valid)) &&
      is_not_on_most_left_lane && is_normal_lane;

  if (lc_condition_nearby_merge) {
    if (request_type_ == NO_CHANGE) {
      if ((((v_rel_l - v_rel_f > 0. || distance_to_y_point < 1000) &&
            v_rel_l > 0.) ||
           v_rel_l + v_ego > 75. / 3.6 ||
           (v_rel_f - v_rel_l < 5. && v_rel_l > -2.5 &&
            v_rel_l + v_ego > 60. / 3.6) ||
           (v_rel_f <= -2.5 && v_rel_l + v_ego > 30. / 3.6)) &&
          (v_rel_r < 15.0 ||
           current_lane_index == virtual_lane_mgr_->get_lane_num() - 1) &&
          v_ego > 30 / 3.6 && (v_rel_f <= 6. || distance_to_y_point < 1000)) {
        pos_cnt_l_ += 1;
        neg_cnt_l_ = 0;
        if (pos_cnt_l_ > 20) {
          left_faster_ = true;
          enable_l_ = true;
          LOG_DEBUG("[ActRequest::update] %s:%d enable_l_ set", __FUNCTION__,
                    __LINE__);
        }
      } else {
        LOG_DEBUG(
            "[ActRequest::update] %s:%d v_rel not satisified for avoid merge/y "
            "point",
            __FUNCTION__, __LINE__);
        neg_cnt_l_ += 1;
        if (neg_cnt_l_ > 20) {
          pos_cnt_l_ = 0;
          left_faster_ = false;
          enable_l_ = false;
          LOG_DEBUG("[ActRequest::update] %s:%d enable_l_ cleared",
                    __FUNCTION__, __LINE__);
        }
      }
    } else if (enable_l_ && request_type_ == LEFT_CHANGE &&
               (lc_status == ROAD_NONE || lc_status == ROAD_LC_LWAIT) &&
               ((v_rel_l < -3. && v_rel_l + v_ego <= 50. / 3.6) ||
                v_rel_l + v_ego < 20. / 3.6 || v_ego < 20. / 3.6 ||
                (v_rel_f > 6.0 && !(distance_to_y_point < 1000)))) {
      LOG_DEBUG(
          "[ActRequest::update] v_rel not satisified for avoid merge/y point "
          "when request generated");
      neg_cnt_l_ += 1;
      if (neg_cnt_l_ > 20) {
        pos_cnt_l_ = 0;
        left_faster_ = false;
        enable_l_ = false;
        LOG_DEBUG("[ActRequest::update] %s:%d enable_l_ cleared", __FUNCTION__,
                  __LINE__);
        Finish();
        Reset();
        LOG_DEBUG(
            "[ActRequest::update] %s:%d finish request, v_rel not satisified "
            "not satistied for merge/Y",
            __FUNCTION__, __LINE__);
      }
    }
  } else {
    if (lc_status == ROAD_NONE || lc_status == ROAD_LC_LWAIT) {
      if (enable_l_ && request_type_ == LEFT_CHANGE) {
        Finish();
        Reset();
        LOG_DEBUG(
            "[ActRequest::update] %s:%d finish request, merge/Y not satistied "
            "\n",
            __FUNCTION__, __LINE__);
      }
      pos_cnt_l_ = 0;
      neg_cnt_l_ = 0;
      left_faster_ = false;
      enable_l_ = false;
      LOG_DEBUG("[ActRequest::update] %s:%d enable_l_ cleared \n", __FUNCTION__,
                __LINE__);
    }
  }

  if ((enforced_l_ || enable_l_) && diff_map > default_ma_delay &&
      diff_int > default_int_delay && !current_lane->is_solid_line(0)) {
    if (request_type_ != LEFT_CHANGE) {
      GenerateRequest(LEFT_CHANGE);
      LOG_DEBUG("[ActRequest::update] Ask for active changing lane to left \n");
      // if (map_info.lanes_y_point_type() == MSD_MERGE_TYPE_MERGE_FROM_RIGHT) {
      // 需要区分 merge_split_point, y_point的差异和类型
      if (current_lane->get_lane_merge_split_point()
                  .merge_split_point_data_size() > 0 &&
          current_lane->get_lane_merge_split_point()
                  .merge_split_point_data(0)
                  .orientation() ==
              FusionRoad::LaneOrientation::ORIENTATION_RIGHT) {
        act_request_source_ = "avd_y_from_right";
      }
      // else if (map_info.lanes_merge_type() ==
      //            MSD_MERGE_TYPE_MERGE_FROM_RIGHT) {
      //   act_request_source_ = "avd_merge_from_right";
      // }
    }
    if (!IsDashedLineEnough(LEFT_CHANGE, v_ego, virtual_lane_mgr_) &&
        curr_direct_exist && request_type_ != NO_CHANGE &&
        (lc_status == ROAD_NONE || lc_status == ROAD_LC_LWAIT ||
         (lc_status == ROAD_LC_LBACK &&
          (lane_change_lane_mgr_->has_origin_lane() &&
           lane_change_lane_mgr_->is_ego_on(olane))))) {
      Finish();
      Reset();
      LOG_DEBUG("[ActRequest::update] %s:%d finish request, dash not enough \n",
                __FUNCTION__, __LINE__);
    }
  } else if (!enforced_l_) {
    bool status_cond =
        ((not_accident && lane_change_lane_mgr_->has_origin_lane() &&
          lane_change_lane_mgr_->is_ego_on(olane)) ||
         (!(lc_status == ROAD_LC_LCHANGE || lc_status == ROAD_LC_RCHANGE ||
            lc_status == INTER_GS_LC_LCHANGE ||
            lc_status == INTER_GS_LC_RCHANGE ||
            lc_status == INTER_TR_LC_LCHANGE ||
            lc_status == INTER_TR_LC_RCHANGE ||
            lc_status == INTER_TL_LC_LCHANGE ||
            lc_status == INTER_TL_LC_RCHANGE) ||
          ((lc_status == ROAD_LC_LCHANGE) || (lc_status == ROAD_LC_RCHANGE) ||
           (lc_status == INTER_GS_LC_LCHANGE && l_change_cond) ||
           (lc_status == INTER_GS_LC_RCHANGE && r_change_cond) ||
           (lc_status == INTER_TR_LC_LCHANGE && l_change_cond) ||
           (lc_status == INTER_TR_LC_RCHANGE && r_change_cond) ||
           (lc_status == INTER_TL_LC_LCHANGE && l_change_cond) ||
           (lc_status == INTER_TL_LC_RCHANGE && r_change_cond))));

    if (left_alc_car.size() > 0) {
      // 存在需要从左避让的车
      if (request_type_ != LEFT_CHANGE) {
        // if (is_on_highway &&
        //     ((map_info.lanes_merge_type(left_lane_index) ==
        //           MSD_MERGE_TYPE_MERGE_FROM_LEFT &&
        //       map_info.distance_to_lanes_merge(left_lane_index) < 1200) ||
        //      (map_info.lanes_y_point_type(left_lane_index) ==
        //           MSD_MERGE_TYPE_MERGE_FROM_LEFT &&
        //       map_info.distance_to_lanes_y_point(left_lane_index) < 1200))) {
        target_lane_virtual_id_tmp = origin_lane_virtual_id_ - 1;
        if (is_on_highway &&
            (is_nearby_right_merge_point || is_nearby_right_y_point)) {
        } else {
          GenerateRequest(LEFT_CHANGE);
          set_target_lane_virtual_id(target_lane_virtual_id_tmp);
          act_request_source_ = "act";
          LOG_DEBUG(
              "[ActRequest::update] Ask for active changing lane to left \n");
        }
      }
      if (!IsDashedLineEnough(LEFT_CHANGE, v_ego, virtual_lane_mgr_) &&
          curr_direct_exist && request_type_ != NO_CHANGE &&
          (lc_status == ROAD_NONE || lc_status == ROAD_LC_LWAIT ||
           (lc_status == ROAD_LC_LBACK &&
            (lane_change_lane_mgr_->has_origin_lane() &&
             lane_change_lane_mgr_->is_ego_on(olane))))) {
        Finish();
        Reset();
        set_target_lane_virtual_id(current_lane_virtual_id);
        LOG_DEBUG(
            "[ActRequest::update] %s:%d finish request, dash not enough \n",
            __FUNCTION__, __LINE__);
      }
    } else if (right_alc_car.size() > 0) {
      // 存在需要从右避让的车
      if (request_type_ != RIGHT_CHANGE) {
        // if (is_on_highway &&
        //     ((map_info.lanes_merge_type(right_lane_index) ==
        //           MSD_MERGE_TYPE_MERGE_FROM_RIGHT &&
        //       map_info.distance_to_lanes_merge(right_lane_index) < 1200) ||
        //      (map_info.lanes_y_point_type(right_lane_index) ==
        //           MSD_MERGE_TYPE_MERGE_FROM_RIGHT &&
        //       map_info.distance_to_lanes_y_point(right_lane_index) < 1200)))
        //       {
        target_lane_virtual_id_tmp = origin_lane_virtual_id_ + 1;
        if (is_on_highway &&
            (is_nearby_right_merge_point || is_nearby_right_y_point)) {
        } else {
          GenerateRequest(RIGHT_CHANGE);
          set_target_lane_virtual_id(target_lane_virtual_id_tmp);
          act_request_source_ = "act";
          LOG_DEBUG(
              "[ActRequest::update] Ask for active changing lane to right \n");
        }
      }
      if (!IsDashedLineEnough(RIGHT_CHANGE, v_ego, virtual_lane_mgr_) &&
          curr_direct_exist && request_type_ != NO_CHANGE &&
          (lc_status == ROAD_NONE || lc_status == ROAD_LC_RWAIT ||
           (lc_status == ROAD_LC_RBACK &&
            (lane_change_lane_mgr_->has_origin_lane() &&
             lane_change_lane_mgr_->is_ego_on(olane))))) {
        Finish();
        Reset();
        set_target_lane_virtual_id(current_lane_virtual_id);
        LOG_DEBUG(
            "[ActRequest::update] %s:%d finish request, dash not enough \n",
            __FUNCTION__, __LINE__);
      }
    } else if (request_type_ != NO_CHANGE && status_cond) {
      Finish();
      set_target_lane_virtual_id(current_lane_virtual_id);
      act_request_source_ = "none";
      LOG_DEBUG("[ActRequest::update] %s:%d finish request, no alc cars \n",
                __FUNCTION__, __LINE__);
    }

    if (neg_left_alc_car && status_cond) {
      if (request_type_ == LEFT_CHANGE) {
        Finish();
        set_target_lane_virtual_id(current_lane_virtual_id);
        act_request_source_ = "none";
        LOG_DEBUG(
            "[ActRequest::update] %s:%d finish request, neg_left_alc_car \n",
            __FUNCTION__, __LINE__);
      }
    }

    if (neg_right_alc_car && status_cond) {
      if (request_type_ == RIGHT_CHANGE) {
        Finish();
        set_target_lane_virtual_id(current_lane_virtual_id);
        act_request_source_ = "none";
        LOG_DEBUG(
            "[ActRequest::update] %s:%d finish request, neg_right_alc_car \n",
            __FUNCTION__, __LINE__);
      }
    }
  }
}

void ActRequest::Reset(int direction) {
  if (direction == NO_CHANGE) {
    pos_cnt_l_ = 0;
    pos_cnt_r_ = 0;
    neg_cnt_l_ = 0;
    neg_cnt_r_ = 0;
    left_faster_ = false;
    right_faster_ = false;
  } else if (direction == LEFT_CHANGE) {
    pos_cnt_l_ = 0;
    neg_cnt_l_ = 0;
    left_faster_ = false;
  } else if (direction == RIGHT_CHANGE) {
    pos_cnt_r_ = 0;
    neg_cnt_r_ = 0;
    right_faster_ = false;
  } else {
    LOG_ERROR("[ActRequest::reset] Illegal direction \n");
  }
  act_request_source_ = "none";
}
}  // namespace planning