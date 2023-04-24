#include "src/modules/scenario/lane_change_requests/active_lane_change_request.h"

#include "src/modules/scenario/lateral_behavior_object_selector.h"

namespace planning {
// class: ActRequest
ActRequest::ActRequest(
    planning::framework::Session *session,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : LaneChangeRequest(session, virtual_lane_mgr, lane_change_lane_mgr) {}

void ActRequest::Update(int lc_status, double start_move_distolane,
                        double lc_int_tfinish, double lc_map_tfinish,
                        bool accident_ahead, bool not_accident) {
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
  int right_lane_index = virtual_lane_mgr_.get_lane_num() - 1;

  auto current_lane = virtual_lane_mgr_->get_current_lane();
  auto current_lane_index = virtual_lane_mgr_->get_lane_index(current_lane);
  if (virtual_lane_mgr_->lc_map_decision(current_lane) == -1 &&
      current_lane->lc_map_decision_offset() < 150) {
    default_int_delay = 0.;
    default_ma_delay = 0.;
  }
  // if (olane.has_master() && !map_info.is_in_intersection()) {
  //   olane_width = olane.width();
  // } else if (map_info.is_in_intersection()) {
  //   olane_width = 3.6;
  // }
  olane_width = 3.6;  // hack
  double act_cancel_thr = std::max(olane_width / 2 - 0.6, 0.);
  bool l_change_cond = (tlane.has_master() &&
                        flane->get_ego_lateral_offset() + v_ego * dt_delay <
                            start_move_distolane + act_cancel_thr);

  bool r_change_cond = (tlane.has_master() &&
                        flane->get_ego_lateral_offset() - v_ego * dt_delay >
                            start_move_distolane - act_cancel_thr);

  // auto traffic_light_direction = map_info.traffic_light_direction();
  bool curr_direct_exist =
      (current_lane->get_lane_marks() == DIRECTION_UNKNOWN &&
       current_lane->get_lane_type() == FusionRoad::LaneType::LANE_TYPE_NORMAL);
  // 获取左右车道的index 换成获取id
  if (current_lane_index > 0) {
    left_lane_index = current_lane_index - 1;
  }
  if (current_lane_index <= map_info.lanes_num() - 1) {
    right_lane_index = current_lane_index + 1;
  }

  // 判断道路条件lane_change_condition是否可以进行换道
  const double kSpeedLimit = 60.0 / 3.6;
  const int kDistanceBuffer = 1000;
  // 道路限速
  auto v_limit_map = session_->planning_output_context()
                         .planning_status()
                         .planning_result.v_limit_map;
  bool is_on_highway = session_->environmental_model().is_on_highway();
  bool is_not_on_ramp = true;  // hack !map_info.is_on_ramp();
  // 判断是否为最左侧车道
  bool is_not_on_most_left_lane = current_lane_index != 0;
  bool is_normal_lane =
      current_lane->get_lane_type() == FusionRoad::LaneType::LANE_TYPE_NORMAL;

  auto distance_to_merge_point =
      current_lane->get_lane_merge_split_point()->distance();
  bool is_nearby_right_merge_point =
      map_info.lanes_merge_type() ==
          FusionRoad::LaneOrientation::ORIENTATION_RIGHT &&
      distance_to_merge_point > 0 &&
      distance_to_merge_point < kDistanceBuffer &&
      distance_to_merge_point + kDistanceBuffer < map_info.dis_to_ramp();

  bool is_nearby_right_merge_point_lc_valid =
      (virtual_lane_mgr_->lc_map_decision(current_lane) >= 0 &&
       distance_to_merge_point + kDistanceBuffer < map_info.lc_end_dis()) ||
      virtual_lane_mgr_->lc_map_decision(current_lane) < 0;

  bool is_nearby_right_y_point =
      map_info.lanes_y_point_type() ==
          FusionRoad::LaneOrientation::ORIENTATION_RIGHT &&
      map_info.distance_to_lanes_y_point() > 0 &&
      map_info.distance_to_lanes_y_point() < kDistanceBuffer &&
      map_info.distance_to_lanes_y_point() + kDistanceBuffer <
          map_info.dis_to_ramp();
  bool is_from_right_y_point_lc_decision_valid =
      (virtual_lane_mgr_->lc_map_decision(current_lane) >= 0 &&
       map_info.distance_to_lanes_y_point() + kDistanceBuffer <
           map_info.lc_end_dis()) ||
      virtual_lane_mgr_->lc_map_decision(current_lane) < 0;

  bool lane_change_condition =
      v_limit_map >= kSpeedLimit && is_on_highway && is_not_on_ramp &&
      ((is_nearby_right_merge_point && is_nearby_right_merge_point_lc_valid &&
        front_tracks_r_cnt > 0) ||
       (is_nearby_right_y_point && is_from_right_y_point_lc_decision_valid)) &&
      is_not_on_most_left_lane && is_normal_lane;

  if (lane_change_condition) {
    if (request_ == NO_CHANGE) {
      if ((((v_rel_l - v_rel_f > 0. ||
             map_info.distance_to_lanes_y_point() < 1000) &&
            v_rel_l > 0.) ||
           v_rel_l + v_ego > 75. / 3.6 ||
           (v_rel_f - v_rel_l < 5. && v_rel_l > -2.5 &&
            v_rel_l + v_ego > 60. / 3.6) ||
           (v_rel_f <= -2.5 && v_rel_l + v_ego > 30. / 3.6)) &&
          (v_rel_r < 15.0 ||
           map_info.current_lane_index() == map_info.lanes_num() - 1) &&
          v_ego > 30 / 3.6 &&
          (v_rel_f <= 6. || map_info.distance_to_lanes_y_point() < 1000)) {
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
    } else if (enable_l_ && request_ == LEFT_CHANGE &&
               (lc_status == ROAD_NONE || lc_status == ROAD_LC_LWAIT) &&
               ((v_rel_l < -3. && v_rel_l + v_ego <= 50. / 3.6) ||
                v_rel_l + v_ego < 20. / 3.6 || v_ego < 20. / 3.6 ||
                (v_rel_f > 6.0 &&
                 !(map_info.distance_to_lanes_y_point() < 1000)))) {
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
        finish();
        reset();
        LOG_DEBUG(
            "[ActRequest::update] %s:%d finish request, v_rel not satisified "
            "not satistied for merge/Y",
            __FUNCTION__, __LINE__);
      }
    }
  } else {
    if (lc_status == ROAD_NONE || lc_status == ROAD_LC_LWAIT) {
      if (enable_l_ && request_ == LEFT_CHANGE) {
        finish();
        reset();
        LOG_DEBUG(
            "[ActRequest::update] %s:%d finish request, merge/Y not satistied",
            __FUNCTION__, __LINE__);
      }
      pos_cnt_l_ = 0;
      neg_cnt_l_ = 0;
      left_faster_ = false;
      enable_l_ = false;
      LOG_DEBUG("[ActRequest::update] %s:%d enable_l_ cleared", __FUNCTION__,
                __LINE__);
    }
  }
}

}  // namespace planning