// #include "lateral_behavior_object_selector.h"
// #include <algorithm>
// #include "../../common/planning_gflags.h"
// #include "config/basic_type.h"
// #include "planning_context.h"

// namespace planning {

// void calc_desired_gap(double v_ego, const TrackedObject &first_object,
//                       const TrackedObject &second_object, double t,
//                       double d_offset, double t_gap, double safety_dist,
//                       double &diff_dn_car, double &desired_gap) {
//   double v_lead = v_ego + first_object.v_rel;
//   double diff_vn_car = first_object.v_rel - second_object.v_rel;
//   diff_dn_car = first_object.d_rel - second_object.d_rel + diff_vn_car * t;
//   desired_gap = d_offset + v_lead * t_gap + 5.0 + safety_dist;
// }

// void remove_car(std::vector<int> &car_list, int id) {
//   auto iter = std::find(car_list.begin(), car_list.end(), id);
//   if (iter != car_list.end()) {
//     car_list.erase(iter);
//   }
// }

// ObjectSelector::ObjectSelector(const EgoPlanningConfigBuilder *config_builder,
//                                planning::framework::Session *session)
//     : session_(session) {
//   config_ = config_builder->cast<EgoPlanningObjectSelectorManagerConfig>();
//   left_lb_car_cnt_.insert(std::make_pair(1000, CarCount(0, 0)));
//   right_lb_car_cnt_.insert(std::make_pair(1000, CarCount(0, 0)));
//   left_alc_car_cnt_.insert(std::make_pair(1000, CarCount(0, 0)));
//   right_alc_car_cnt_.insert(std::make_pair(1000, CarCount(0, 0)));
// }

// bool ObjectSelector::in_alc_range() {
//   // TODO(Rui):wait for map ready
//   // const auto &map_info = map_info_mgr_.get_map_info();
//   // if (map_info.dist_to_intsect() < 50 || (is_on_highway && lc_end_dis <=
//   // 350)) {
//   //   return false;
//   // }
//   const auto &route_info_output =
//       session_->environmental_model().get_route_info()->get_route_info_output();
//   bool is_on_ramp = route_info_output.is_on_ramp;
//   if (is_on_ramp) return false;

//   return true;
// }

// double ObjectSelector::get_vrel_close(int side, int status) {
//   double v_rel_close = 15.;
//   double fvf_drel_confident = 120.;
//   std::vector<TrackedObject> front_tracks;

//   const auto &virtual_lane_mgr =
//       session_->environmental_model().get_virtual_lane_manager();
//   const auto &lane_change_decider_output =
//       session_->planning_context().lane_change_decider_output();
//   const auto &lateral_obstacle =
//       session_->environmental_model().get_lateral_obstacle();
//   int fix_lane_virtual_id = lane_change_decider_output.fix_lane_virtual_id;
//   int olane_virtual_id = lane_change_decider_output.origin_lane_virtual_id;
//   int target_lane_virtual_id =
//       lane_change_decider_output.target_lane_virtual_id;
//   std::shared_ptr<ReferencePathManager> reference_path_mgr =
//       session_->mutable_environmental_model()->get_reference_path_manager();
//   auto flane = virtual_lane_mgr->get_lane_with_virtual_id(fix_lane_virtual_id);
//   auto olane = virtual_lane_mgr->get_lane_with_virtual_id(olane_virtual_id);
//   auto tlane =
//       virtual_lane_mgr->get_lane_with_virtual_id(target_lane_virtual_id);
//   auto clane = virtual_lane_mgr->get_current_lane();
//   auto llane = virtual_lane_mgr->get_left_lane();
//   auto rlane = virtual_lane_mgr->get_right_lane();

//   auto &lead_cars = lateral_obstacle->get_lead_cars();

//   if ((side == -1 && llane == nullptr) || (side == 1 && rlane == nullptr) ||
//       side > 1 || side < -1) {
//     LOG_ERROR(
//         "[LaneTracksManager::get_vrel_close] Illegal side[%d] argument \n",
//         side);
//     return v_rel_close;
//   }

//   if (clane->get_virtual_id() == flane->get_virtual_id()) {
//     if (lead_cars.lead_one != nullptr) {
//       fvf_drel_confident = lead_cars.lead_one->d_rel +
//                            std::max(30., lead_cars.lead_one->v_lead * 5.);
//     } else if (lead_cars.temp_lead_one != nullptr) {
//       fvf_drel_confident = lead_cars.temp_lead_one->d_rel +
//                            std::max(30., lead_cars.temp_lead_one->v_lead * 5.);
//     }
//   } else {
//     fvf_drel_confident = 30;
//     if (lead_cars.temp_lead_one != nullptr) {
//       fvf_drel_confident = lead_cars.temp_lead_one->d_rel + 30;
//     }
//   }

//   auto lane = clane;
//   if (side == -1) {
//     lane = llane;
//   } else if (side == 1) {
//     lane = rlane;
//   }

//   int leadone_id = lane->get_reference_path()->get_lane_leadone_obstacle();
//   for (TrackedObject obstacle : lateral_obstacle->front_tracks()) {
//     auto &obj_tmp = obstacle;
//     if (obj_tmp.track_id == leadone_id) {
//       front_tracks.push_back(obj_tmp);
//       break;
//     }
//   }

//   if (front_tracks.size() == 0) {
//     LOG_ERROR("[LaneTracksManager::get_vrel_close] %d tracks is null \n", side);
//     return v_rel_close;
//   }

//   for (auto &tr : front_tracks) {
//     if (tr.d_rel < fvf_drel_confident) {
//       v_rel_close = tr.v_rel;
//       break;
//     }
//   }

//   return v_rel_close;
// }

// bool ObjectSelector::in_alc_status(int status, double start_move_distolane) {
//   auto &virtual_lane_mgr =
//       session_->environmental_model().get_virtual_lane_manager();
//   const auto &lane_change_decider_output =
//       session_->planning_context().lane_change_decider_output();
//   int fix_lane_virtual_id = lane_change_decider_output.fix_lane_virtual_id;
//   int olane_virtual_id = lane_change_decider_output.origin_lane_virtual_id;
//   int target_lane_virtual_id =
//       lane_change_decider_output.target_lane_virtual_id;
//   std::shared_ptr<ReferencePathManager> reference_path_mgr =
//       session_->mutable_environmental_model()->get_reference_path_manager();
//   auto fix_reference_path =
//       reference_path_mgr->get_reference_path_by_lane(fix_lane_virtual_id);
//   auto frenet_ego_state = fix_reference_path->get_frenet_ego_state();
//   auto flane = virtual_lane_mgr->get_lane_with_virtual_id(fix_lane_virtual_id);
//   auto olane = virtual_lane_mgr->get_lane_with_virtual_id(olane_virtual_id);
//   auto tlane =
//       virtual_lane_mgr->get_lane_with_virtual_id(target_lane_virtual_id);

//   double olane_width = flane->width();
//   if (olane != nullptr) {
//     olane_width = olane->width();
//   }

//   const auto state = lane_change_decider_output.curr_state;
//   const auto lc_request_direction = lane_change_decider_output.lc_request;
//   bool is_LC_LWAIT =
//       (state == kLaneChangePropose) && (lc_request_direction == 1);
//   bool is_LC_RWAIT =
//       (state == kLaneChangePropose) && (lc_request_direction == 2);
//   bool is_LC_LBACK =
//       (state == kLaneChangeCancel) && (lc_request_direction == 1);
//   bool is_LC_RBACK =
//       (state == kLaneChangeCancel) && (lc_request_direction == 2);
//   return (status == kLaneKeeping || is_LC_LWAIT || is_LC_RWAIT ||
//           (is_LC_LBACK && lane_change_decider_output.lc_back_reason != "" &&
//            (lane_change_decider_output.lc_back_reason == "front view back" ||
//             lane_change_decider_output.lc_back_reason == "side view back")) ||
//           (is_LC_RBACK && lane_change_decider_output.lc_back_reason != "" &&
//            (lane_change_decider_output.lc_back_reason == "front view back" ||
//             lane_change_decider_output.lc_back_reason == "side view back")));
// }

// bool ObjectSelector::check_map_alc_enable(int direction, bool accident_ahead) {
//   // TODO(Rui):wait for map
//   return true;
// }

// bool ObjectSelector::update(int status, double start_move_distolane,
//                             bool accident_ahead, double perception_range,
//                             bool disable_l, bool disable_r,
//                             bool upstream_enable_l, bool upstream_enable_r,
//                             bool upstream_enable_lb, int upstream_enable_id) {
//   double d_offset = 3.5;
//   bool l_enable = true;
//   bool r_enable = true;
//   bool need_lb = false;
//   enable_l_ = true;
//   enable_r_ = true;
//   neg_left_lb_car_ = false;
//   neg_right_lb_car_ = false;
//   neg_left_alc_car_ = false;
//   neg_right_alc_car_ = false;
//   jam_cancel_ = false;
//   left_close_objs_.clear();
//   right_close_objs_.clear();
//   current_close_objs_.clear();

//   auto &virtual_lane_mgr =
//       session_->environmental_model().get_virtual_lane_manager();
//   const auto &lane_change_decider_output =
//       session_->planning_context().lane_change_decider_output();
//   const auto &route_info_output =
//       session_->environmental_model().get_route_info()->get_route_info_output();
//   auto &ego_state = session_->environmental_model().get_ego_state_manager();
//   auto &lateral_obstacle =
//       session_->environmental_model().get_lateral_obstacle();
//   int fix_lane_virtual_id = lane_change_decider_output.fix_lane_virtual_id;
//   int olane_virtual_id = lane_change_decider_output.origin_lane_virtual_id;
//   int target_lane_virtual_id =
//       lane_change_decider_output.target_lane_virtual_id;
//   std::shared_ptr<ReferencePathManager> reference_path_mgr =
//       session_->mutable_environmental_model()->get_reference_path_manager();
//   auto fix_reference_path =
//       reference_path_mgr->get_reference_path_by_lane(fix_lane_virtual_id);
//   if (!fix_reference_path) {
//     LOG_ERROR("fix_reference_path is null\n");
//     return false;
//   }
//   auto frenet_ego_state = fix_reference_path->get_frenet_ego_state();
//   auto flane = virtual_lane_mgr->get_lane_with_virtual_id(fix_lane_virtual_id);
//   auto olane = virtual_lane_mgr->get_lane_with_virtual_id(olane_virtual_id);
//   auto tlane =
//       virtual_lane_mgr->get_lane_with_virtual_id(target_lane_virtual_id);
//   auto clane = virtual_lane_mgr->get_current_lane();
//   auto llane = virtual_lane_mgr->get_left_lane();
//   auto rlane = virtual_lane_mgr->get_right_lane();
//   bool is_on_highway = session_->environmental_model().is_on_highway();

//   int request_source = lane_change_decider_output.lc_request_source;
//   int request = lane_change_decider_output.lc_request;

//   double coefficient = FLAGS_planning_loop_rate / 15.;

//   std::array<double, 5> lead_confidence_v{1 * coefficient, 2 * coefficient,
//                                           4 * coefficient, 20 * coefficient,
//                                           50 * coefficient};
//   std::array<double, 5> lead_confidence_bp{0, 30, 60, 90, 120};
//   std::array<double, 3> t_gap_vego_v{1.35, 1.55, 2.0};
//   std::array<double, 3> t_gap_vego_bp{5.0, 15.0, 30.0};

//   double v_ego = ego_state->ego_v();
//   double l_ego = frenet_ego_state.l();
//   double safety_dist = v_ego * v_ego / 6.0 + 2;
//   double min_visible_gap = 10.0;
//   double max_visible_gap = 25.0;
//   double max_perception_range = 90.0;
//   // 暂时将速度低于10km/h的目标物看作静止目标物
//   double active_lane_change_min_object_speed_threshold = 2.778;
//   const int active_lane_change_min_duration_threshold =
//       config_.active_lane_change_min_duration_threshold;

//   const bool use_lateral_distance_to_judge_cutout_in_active_lane_change =
//       config_.use_lateral_distance_to_judge_cutout_in_active_lane_change;

//   const double half_car_width = 1.75;

//   const std::vector<TrackedObject> &front_tracks =
//       lateral_obstacle->front_tracks();
//   auto &lane_tracks_mgr =
//       session_->environmental_model().get_lane_tracks_manager();
//   const std::vector<TrackedObject> &front_tracks_l =
//       lane_tracks_mgr->front_tracks_l();
//   const std::vector<TrackedObject> &front_tracks_r =
//       lane_tracks_mgr->front_tracks_r();
//   const std::vector<TrackedObject> &front_tracks_c =
//       lane_tracks_mgr->front_tracks_c();
//   front_tracks_l_cnt_ = front_tracks_l.size();
//   front_tracks_r_cnt_ = front_tracks_r.size();

//   TrackedObject *lead_one = lateral_obstacle->leadone();
//   TrackedObject *lead_two = lateral_obstacle->leadtwo();
//   TrackedObject *temp_leadone = lateral_obstacle->tleadone();
//   TrackedObject *temp_leadtwo = lateral_obstacle->tleadtwo();

//   auto current_lane_index = virtual_lane_mgr->get_lane_index(clane);
//   auto &left_boundary_info = clane->get_left_lane_boundary();
//   auto &right_boundary_info = clane->get_right_lane_boundary();
//   auto &lane_merge_split_point = clane->get_lane_merge_split_point();
//   // auto traffic_light_direction = virtual_lane_mgr->traffic_light_direction();
//   auto intersection_info = virtual_lane_mgr->get_intersection_info();
//   auto dist_to_last_intsect = intersection_info.dist_to_last_intsect();
//   auto dist_to_intsect = intersection_info.dist_to_intsect();
//   bool is_in_intersection = intersection_info.is_in_intersection();
//   double dis_to_ramp = route_info_output.dis_to_ramp;
//   double distance_to_first_road_split =
//       route_info_output.distance_to_first_road_split;
//   double intersect_length = intersection_info.intsect_length();
//   bool is_on_ramp = false;  // hack map_info.is_on_ramp()
//   double lc_end_dis = virtual_lane_mgr->lc_map_decision_offset(clane);
//   int current_lane_tasks_id = virtual_lane_mgr->get_tasks(clane);
//   int left_lane_tasks_id = virtual_lane_mgr->get_tasks(llane);
//   int right_lane_tasks_id = virtual_lane_mgr->get_tasks(rlane);
//   std::cout << "current_lane_tasks_id: " << current_lane_tasks_id
//             << " left_lane_tasks_id: " << left_lane_tasks_id
//             << " right_lane_tasks_id: " << right_lane_tasks_id << std::endl;

//   double car_width = 2.2;
//   double lb_width_l = 0.4;
//   double lb_width_r = 0.0;
//   double press_thr = 0.3;
//   double lane_width = flane->width();
//   double t_gap = interp(v_ego, t_gap_vego_bp, t_gap_vego_v);
//   const auto state = lane_change_decider_output.curr_state;
//   const auto lc_request_direction = lane_change_decider_output.lc_request;
//   bool LCHANGE =
//       ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
//       (lc_request_direction == LEFT_CHANGE);
//   bool RCHANGE =
//       ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
//       (lc_request_direction == RIGHT_CHANGE);
//   bool is_LC_LWAIT =
//       (state == kLaneChangePropose) && (lc_request_direction == LEFT_CHANGE);
//   bool is_LC_RWAIT =
//       (state == kLaneChangePropose) && (lc_request_direction == RIGHT_CHANGE);
//   bool is_LC_LBACK =
//       (state == kLaneChangeCancel) && (lc_request_direction == LEFT_CHANGE);
//   bool is_LC_RBACK =
//       (state == kLaneChangeCancel) && (lc_request_direction == RIGHT_CHANGE);
//   bool isRedLightStop = false;

//   if ((llane == nullptr || (left_boundary_info.type_segments_size > 0 &&
//                             left_boundary_info.type_segments[0].type ==
//                                 iflyauto::LaneBoundaryType_MARKING_SOLID)) &&
//       (status == kLaneKeeping ||
//        (olane != nullptr &&
//         olane->get_virtual_id() == clane->get_virtual_id() &&
//         (!LCHANGE || l_ego < -lane_width / 2 - 0.2)))) {
//     l_enable = false;
//   }

//   if ((rlane == nullptr || (right_boundary_info.type_segments_size > 0 &&
//                             right_boundary_info.type_segments[0].type ==
//                                 iflyauto::LaneBoundaryType_MARKING_SOLID)) &&
//       (status == kLaneKeeping ||
//        (olane != nullptr &&
//         olane->get_virtual_id() == clane->get_virtual_id() &&
//         (!RCHANGE || l_ego > lane_width / 2 + 0.2)))) {
//     r_enable = false;
//   }

//   if (disable_l) {
//     l_enable = false;
//   }
//   if (disable_r) {
//     r_enable = false;
//   }

//   enable_l_ = l_enable;
//   enable_r_ = r_enable;

//   bool lb_leadone_disable = false;

//   bool curr_direct_exist = true;

//   bool left_direct_exist = true;

//   bool right_direct_exist = true;

//   bool left_laneout_direct_exist = true;
//   bool right_laneout_direct_exist = true;

//   int faster_cnt = dist_to_intsect > 0 ? 50 : 10;

//   if (l_enable && llane != nullptr) {
//     v_rel_l_ = get_vrel_close(-1, status);
//     if (tlane != nullptr &&
//         tlane->get_virtual_id() == clane->get_virtual_id() &&
//         request == LEFT_CHANGE) {
//       v_rel_l_ = std::max(get_vrel_close(0, status) + 2.,
//                           get_vrel_close(0, status) + v_ego);
//     }

//     if (v_rel_l_ >= 5.0 ||
//         (temp_leadone != nullptr && v_rel_l_ - temp_leadone->v_rel >= 5.0)) {
//       left_is_faster_cnt_++;
//       if (left_is_faster_cnt_ >= faster_cnt * coefficient) {
//         left_is_faster_ = true;
//       }
//     } else {
//       if (left_is_faster_cnt_ > 100) {
//         left_is_faster_cnt_ = 100;
//       }
//       left_is_faster_cnt_ = std::max(left_is_faster_cnt_ - 10, 0);
//       if (left_is_faster_cnt_ == 0) {
//         left_is_faster_ = false;
//       }
//     }
//   } else {
//     v_rel_l_ = 0.0;
//     left_is_faster_ = false;
//   }

//   if (r_enable && rlane != nullptr) {
//     v_rel_r_ = get_vrel_close(1, status);
//     if (tlane != nullptr &&
//         tlane->get_virtual_id() == clane->get_virtual_id() &&
//         request == RIGHT_CHANGE) {
//       v_rel_r_ = std::max(get_vrel_close(0, status) + 2.,
//                           get_vrel_close(0, status) + v_ego);
//     }

//     if (v_rel_r_ >= 5.0 ||
//         (temp_leadone != nullptr && v_rel_r_ - temp_leadone->v_rel >= 5.0)) {
//       right_is_faster_cnt_ += 1;
//       if (right_is_faster_cnt_ >= faster_cnt * coefficient) {
//         right_is_faster_ = true;
//       }
//     } else {
//       if (right_is_faster_cnt_ > 100) {
//         right_is_faster_cnt_ = 100;
//       }
//       right_is_faster_cnt_ = std::max(right_is_faster_cnt_ - 10, 0);
//       if (right_is_faster_cnt_ == 0) {
//         right_is_faster_ = false;
//       }
//     }
//   } else {
//     v_rel_r_ = 0.0;
//     right_is_faster_ = false;
//   }

//   double rout_l_dash_length = 0.;
//   double lout_r_dash_length = 0.;
//   double l_dash_length = 0;
//   double r_dash_length = 0;

//   for (int i = 0; i < left_boundary_info.type_segments_size; i++) {
//     if (left_boundary_info.type_segments[i].type !=
//         iflyauto::LaneBoundaryType_MARKING_SOLID) {
//       l_dash_length += left_boundary_info.type_segments[i].length;
//     } else {
//       break;
//     }
//   }
//   for (int i = 0; i < right_boundary_info.type_segments_size; i++) {
//     if (right_boundary_info.type_segments[i].type !=
//         iflyauto::LaneBoundaryType_MARKING_SOLID) {
//       r_dash_length += right_boundary_info.type_segments[i].length;
//     } else {
//       break;
//     }
//   }

//   if ((in_alc_range() ||
//        (left_boundary_info.type_segments_size > 0 &&
//         left_boundary_info.type_segments[0].type ==
//             iflyauto::LaneBoundaryType_MARKING_DASHED &&
//         dist_to_intsect > 0 &&
//         (left_boundary_info.type_segments[0].length - dist_to_intsect > -10 ||
//          (LCHANGE) || (RCHANGE))) ||
//        dist_to_intsect < -5 || accident_ahead) &&
//       lateral_obstacle->sensors_okay()) {
//     std::map<int, bool> front_tracks_ids;
//     std::map<int, bool> front_tracks_l_ids;
//     std::map<int, bool> front_tracks_r_ids;
//     std::map<int, bool> front_tracks_c_ids;
//     std::map<int, bool> front_tracks_cone_ids;
//     double d_max = -100.;
//     double d_min = 100.;
//     double max_d_rel = 0.;
//     double min_d_rel = 0.;
//     double accident_drel = 1000.;
//     double c_final_drel = 0.;
//     const double kInputBoundaryLenLimit = 145.;
//     const double kDefaultBoundaryLen = 5000.;
//     bool accident_front = false;

//     for (auto &tr : front_tracks_l) {
//       front_tracks_l_ids.insert(std::make_pair(tr.track_id, true));
//       if (tr.d_rel <= 30 && tr.type != 20001 && tr.v_lead < 1.0) {
//         left_close_objs_.push_back(tr.track_id);
//         if (tr.type == 2 || tr.type == 3 || tr.type == 4) {
//           left_close_objs_.push_back(tr.track_id + 100000);
//         }
//       }
//     }
//     for (auto &tr : front_tracks_r) {
//       front_tracks_r_ids.insert(std::make_pair(tr.track_id, true));
//       if (tr.d_rel <= 30 && tr.type != 20001 && tr.v_lead < 1.0) {
//         right_close_objs_.push_back(tr.track_id);
//         if (tr.type == 2 || tr.type == 3 || tr.type == 4) {
//           right_close_objs_.push_back(tr.track_id + 100000);
//         }
//       }
//     }
//     v_rel_f_ = 15.0;
//     for (auto &tr : front_tracks_c) {
//       front_tracks_c_ids.insert(std::make_pair(tr.track_id, true));
//       if (tr.d_rel > c_final_drel) {
//         c_final_drel = tr.d_rel;
//       }

//       v_rel_f_ = std::min(v_rel_f_, tr.v_rel);
//       if (tr.is_accident_car &&
//           (accident_drel == 1000. || tr.d_rel > accident_drel)) {
//         accident_drel = tr.d_rel;
//       }
//       if (tr.d_rel <= 30 && tr.type != 20001 && tr.v_lead < 1.0) {
//         current_close_objs_.push_back(tr.track_id);
//         if (tr.is_accident_car) {
//           accident_front = true;
//         }
//         if (tr.type == 2 || tr.type == 3 || tr.type == 4) {
//           current_close_objs_.push_back(tr.track_id + 100000);
//         }
//       } else if (tr.type == 20001) {
//         if (tr.d_min_cpath < d_min) {
//           d_min = tr.d_min_cpath;
//           min_d_rel = tr.d_rel;
//         }
//         if (tr.d_max_cpath > d_max) {
//           d_max = tr.d_max_cpath;
//           max_d_rel = tr.d_rel;
//         }
//         front_tracks_cone_ids.insert(std::make_pair(tr.track_id, true));
//       }
//     }
//     if (front_tracks_c.size() == 0) {
//       v_rel_f_ = 15.;
//     }

//     for (auto &tr : front_tracks) {
//       front_tracks_ids.insert(std::make_pair(tr.track_id, true));
//       if (!LCHANGE && !RCHANGE && accident_drel < 1000. &&
//           front_tracks_l_ids.find(tr.track_id) == front_tracks_l_ids.end() &&
//           front_tracks_r_ids.find(tr.track_id) == front_tracks_r_ids.end() &&
//           accident_drel - tr.d_rel > 2 && accident_drel - tr.d_rel < 15 &&
//           !tr.is_lead && !tr.is_temp_lead) {
//         if (tr.d_max_cpath < lane_width && tr.d_min_cpath > 0 &&
//             ((curr_direct_exist &&
//               (right_direct_exist ||
//                (right_boundary_info.type_segments_size > 0 &&
//                 right_boundary_info.type_segments[0].type ==
//                     iflyauto::LaneBoundaryType_MARKING_DASHED &&
//                 right_boundary_info.type_segments[0].length - tr.d_rel >
//                     60))) ||
//              (!curr_direct_exist &&
//               (right_direct_exist || dist_to_intsect - tr.d_rel > 200)))) {
//           std::array<double, 4> d_expect_bp{0., tr.d_min_cpath};
//           std::array<double, 4> d_expect_v{5., std::max(v_ego * 3, 20.)};
//           double d_expect =
//               interp(tr.d_min_cpath - l_ego, d_expect_bp, d_expect_v);
//           if (tr.d_rel < d_expect) {
//             l_enable = false;
//             enable_l_ = false;
//           }
//         } else if (tr.d_min_cpath > -lane_width && tr.d_max_cpath < 0 &&
//                    ((curr_direct_exist &&
//                      (left_direct_exist ||
//                       (left_boundary_info.type_segments_size > 0 &&
//                        left_boundary_info.type_segments[0].type ==
//                            iflyauto::LaneBoundaryType_MARKING_DASHED &&
//                        left_boundary_info.type_segments[0].length - tr.d_rel >
//                            60))) ||
//                     (!curr_direct_exist &&
//                      (left_direct_exist ||
//                       dist_to_intsect - tr.d_rel > 200)))) {
//           std::array<double, 4> d_expect_bp{0., -tr.d_max_cpath};
//           std::array<double, 4> d_expect_v{5., std::max(v_ego * 3, 20.)};
//           double d_expect =
//               interp(-tr.d_max_cpath + l_ego, d_expect_bp, d_expect_v);
//           if (tr.d_rel < d_expect) {
//             r_enable = false;
//             enable_r_ = false;
//           }
//         }
//       }
//     }

//     if (!check_map_alc_enable(LEFT_CHANGE, accident_ahead) && is_on_highway &&
//         l_enable && !LCHANGE) {
//       l_enable = false;
//       enable_l_ = false;
//       LOG_ERROR("check_map_alc_enable] left alc disable");
//     }
//     if (!check_map_alc_enable(RIGHT_CHANGE, accident_ahead) && is_on_highway &&
//         r_enable && !RCHANGE) {
//       r_enable = false;
//       enable_r_ = false;
//       LOG_ERROR("check_map_alc_enable] right alc disable");
//     }

//     if (in_alc_status(status, start_move_distolane) || accident_ahead) {
//       for (auto &tr : front_tracks) {
//         if (!l_enable && !r_enable) {
//           left_lb_car_.clear();
//           left_alc_car_.clear();
//           right_lb_car_.clear();
//           right_alc_car_.clear();

//           enable_l_ = false;
//           enable_r_ = false;
//           return true;
//         } else if (!l_enable) {
//           if (left_alc_car_.size() > 0 &&
//               front_tracks_r_ids.find(left_alc_car_[0]) !=
//                   front_tracks_r_ids.end() &&
//               !disable_l) {
//           } else {
//             left_lb_car_.clear();
//             left_alc_car_.clear();
//             enable_l_ = false;
//           }
//         } else if (!r_enable) {
//           if (right_alc_car_.size() > 0 &&
//               front_tracks_l_ids.find(right_alc_car_[0]) !=
//                   front_tracks_l_ids.end() &&
//               !disable_r) {
//           } else {
//             right_lb_car_.clear();
//             right_alc_car_.clear();
//             enable_r_ = false;
//           }
//         }

//         if (l_enable || r_enable) {
//           double v_target = session_->environmental_model()
//                                 .get_ego_state_manager()
//                                 ->ego_v_cruise();
//           // 临时hack 过滤由静止障碍物引发的ALC
//           if (tr.v > active_lane_change_min_object_speed_threshold &&
//               ((l_enable && llane != nullptr) ||
//                (tlane != nullptr &&
//                 tlane->get_virtual_id() == clane->get_virtual_id()))) {
//             if (accident_ahead && left_is_faster_ && tr.is_accident_car &&
//                 ((lead_one == nullptr ||
//                   ((lead_two == nullptr) ||
//                    (tr.track_id != lead_two->track_id)))) &&
//                 (status == kLaneKeeping || is_LC_LWAIT || LCHANGE ||
//                  is_LC_LBACK ||
//                  (RCHANGE && (request_source == MAP_REQUEST ||
//                               request_source == INT_REQUEST))) &&
//                 (r_accident_cnt_ != 1 || rlane == nullptr) &&
//                 (dist_to_intsect - tr.d_rel >= 35 ||
//                  (left_boundary_info.type_segments_size > 0 &&
//                   left_boundary_info.type_segments[0].type ==
//                       iflyauto::LaneBoundaryType_MARKING_DASHED &&
//                   dist_to_intsect > 0 &&
//                   left_boundary_info.type_segments[0].length - dist_to_intsect >
//                       -10) ||
//                  dist_to_intsect < -5)) {
//               if (dist_to_intsect > 0 || dist_to_intsect < -5) {
//                 if (((left_boundary_info.type_segments_size == 2 ||
//                       left_boundary_info.type_segments_size == 1) &&
//                      left_boundary_info.type_segments[0].type ==
//                          iflyauto::LaneBoundaryType_MARKING_DASHED &&
//                      ((!left_direct_exist &&
//                        ((left_boundary_info.type_segments[0].length - tr.d_rel >
//                              80 &&
//                          !right_direct_exist &&
//                          (olane == nullptr ||
//                           (olane != nullptr &&
//                            olane->get_virtual_id() ==
//                                clane->get_virtual_id()))))) ||
//                       (left_direct_exist &&
//                        left_boundary_info.type_segments[0].length >
//                            tr.d_rel))) ||
//                     left_boundary_info.type_segments_size > 3 ||
//                     dist_to_intsect < -5 ||
//                     (tlane != nullptr &&
//                      tlane->get_virtual_id() == clane->get_virtual_id())) {
//                   l_accident_cnt_ = 1;
//                   left_lb_car_.clear();
//                   left_alc_car_.clear();
//                   left_lb_car_.push_back(tr.track_id);
//                   left_alc_car_.push_back(tr.track_id);
//                   neg_left_lb_car_ = false;
//                   neg_left_alc_car_ = false;
//                   if (((d_min > -lane_width / 2 + car_width + 0.3 &&
//                         !is_in_intersection &&
//                         front_tracks_cone_ids.size() > 1) ||
//                        (is_in_intersection &&
//                         (d_min > -0.5 &&
//                          dist_to_last_intsect - min_d_rel > 15))) &&
//                       !accident_front) {
//                     if (front_tracks_cone_ids.find(left_alc_car_[0]) !=
//                         front_tracks_cone_ids.end()) {
//                       left_lb_car_.clear();
//                       left_alc_car_.clear();
//                       l_accident_cnt_ = 0;
//                     }
//                   }

//                   if (((lead_two != nullptr && (!LCHANGE && !RCHANGE) &&
//                         lead_two->d_rel - lead_one->d_rel < 20 &&
//                         lead_two->d_rel - lead_one->d_rel > 5 &&
//                         lead_two->type != 20001 && lead_one->type != 20001) ||
//                        ((temp_leadtwo != nullptr &&
//                          temp_leadtwo->d_rel - temp_leadone->d_rel < 20 &&
//                          temp_leadtwo->d_rel - temp_leadone->d_rel > 5 &&
//                          temp_leadtwo->type != 20001 &&
//                          temp_leadone->type != 20001) ||
//                         (right_close_objs_.size() > 2 && tlane != nullptr &&
//                          tlane->get_virtual_id() == clane->get_virtual_id()) ||
//                         (current_close_objs_.size() > 2 && olane != nullptr &&
//                          olane->get_virtual_id() ==
//                              clane->get_virtual_id()))) &&
//                       (((olane == nullptr ||
//                          (olane != nullptr && olane->get_virtual_id() ==
//                                                   clane->get_virtual_id())) &&
//                         !left_direct_exist) ||
//                        is_in_intersection ||
//                        (tlane != nullptr &&
//                         tlane->get_virtual_id() == clane->get_virtual_id() &&
//                         !curr_direct_exist)) &&
//                       dist_to_intsect < 170) {
//                     left_lb_car_.clear();
//                     left_alc_car_.clear();
//                     neg_left_alc_car_ = true;
//                     jam_cancel_ = true;
//                   } else {
//                     if (left_alc_car_cnt_.find(tr.track_id) !=
//                         left_alc_car_cnt_.end()) {
//                       left_alc_car_cnt_[tr.track_id].neg = 0;
//                     }
//                   }
//                 } else if (left_boundary_info.type_segments_size > 0 &&
//                            left_boundary_info.type_segments[0].type ==
//                                iflyauto::LaneBoundaryType_MARKING_SOLID) {
//                   left_lb_car_.clear();
//                   left_alc_car_.clear();
//                   l_accident_cnt_ = 0;
//                 }
//               } else {
//                 left_lb_car_.clear();
//                 left_alc_car_.clear();
//                 l_accident_cnt_ = 0;
//               }
//             } else {
//               l_accident_cnt_ = 0;

//               if (accident_ahead) {
//                 remove_car(left_lb_car_, tr.track_id);
//                 if (left_alc_car_.size() > 0 &&
//                     front_tracks_c_ids.find(left_alc_car_[0]) !=
//                         front_tracks_c_ids.end())
//                   remove_car(left_alc_car_, tr.track_id);
//               }
//             }

//             if ((tr.d_max_cpath != 100 &&
//                  tr.d_max_cpath <= lane_width / 2 + car_width / 5 &&
//                  tr.d_max_cpath > -(car_width / 2 + lane_width * 0.05)) &&
//                 tr.v_rel + v_ego < v_target && (v_ego > 1)) {
//               v_rel_l_ = get_vrel_close(-1, status);
//               double v_left_front = v_rel_l_ + v_ego;
//               double v_front_lb = tr.v_rel + v_ego;

//               std::array<double, 4> xp_pos_l{0, 5, 10, 20};
//               std::array<double, 4> xp_pos_lb{0, 5, 10, 20};
//               std::array<double, 4> fp_pos_l{150 * coefficient,
//                                              80 * coefficient, 40 * coefficient,
//                                              10 * coefficient};
//               std::array<double, 4> fp_pos_lb{
//                   100 * coefficient, 50 * coefficient, 15 * coefficient,
//                   10 * coefficient};
//               std::array<double, 3> xp_neg{-22, -10, 0};
//               std::array<double, 3> fp_neg{20 * coefficient, 35 * coefficient,
//                                            80 * coefficient};

//               int temp =
//                   int(interp(tr.d_rel, lead_confidence_bp, lead_confidence_v));
//               int pos_thr_l = std::max(
//                   int(interp(v_target - v_front_lb, xp_pos_l, fp_pos_l)), temp);
//               int pos_thr_lb_l = std::max(
//                   int(interp(v_target - v_front_lb, xp_pos_lb, fp_pos_lb)),
//                   temp);

//               int neg_thr_l = int(interp(v_rel_l_, xp_neg, fp_neg));
//               int neg_thr_lb_l = neg_thr_l;

//               if (left_lb_car_cnt_.find(tr.track_id) ==
//                   left_lb_car_cnt_.end()) {
//                 left_lb_car_cnt_.insert(
//                     std::make_pair(tr.track_id, CarCount(0, 0)));
//               }

//               if (left_alc_car_cnt_.find(tr.track_id) ==
//                   left_alc_car_cnt_.end()) {
//                 left_alc_car_cnt_.insert(
//                     std::make_pair(tr.track_id, CarCount(0, 0)));
//               }

//               if (right_lb_car_cnt_.find(tr.track_id) ==
//                   right_lb_car_cnt_.end()) {
//                 right_lb_car_cnt_.insert(
//                     std::make_pair(tr.track_id, CarCount(0, 0)));
//               }

//               if (right_alc_car_cnt_.find(tr.track_id) ==
//                   right_alc_car_cnt_.end()) {
//                 right_alc_car_cnt_.insert(
//                     std::make_pair(tr.track_id, CarCount(0, 0)));
//               }

//               if (lead_one != nullptr &&
//                   left_lb_car_cnt_.find(lead_one->track_id) ==
//                       left_lb_car_cnt_.end()) {
//                 left_lb_car_cnt_.insert(
//                     std::make_pair(lead_one->track_id, CarCount(0, 0)));
//               }

//               if (temp_leadone != nullptr &&
//                   left_lb_car_cnt_.find(temp_leadone->track_id) ==
//                       left_lb_car_cnt_.end()) {
//                 left_lb_car_cnt_.insert(
//                     std::make_pair(temp_leadone->track_id, CarCount(0, 0)));
//               }

//               if (accident_ahead && v_rel_l_ >= 5 && r_accident_cnt_ != 1) {
//               } else {
//                 l_accident_cnt_ = 0;

//                 if (!accident_ahead) {
//                   if (!LCHANGE) {
//                     left_lb_car_.clear();
//                   } else {
//                     for (auto &tr : front_tracks) {
//                       auto iter = std::find(left_alc_car_.begin(),
//                                             left_alc_car_.end(), tr.track_id);
//                       if (iter != left_alc_car_.end()) {
//                         if (tr.v_rel > 2.5) {
//                           left_lb_car_.clear();
//                           left_alc_car_.clear();
//                           left_alc_car_cnt_[tr.track_id].pos = 0;
//                           left_lb_car_cnt_[tr.track_id].pos = 0;
//                         }
//                         break;
//                       }
//                     }
//                   }
//                 }

//                 std::array<double, 3> xp{0, 30 / 3.6, 67 / 3.6};
//                 std::array<double, 3> fp{20 / 3.6, 18 / 3.6, 10 / 3.6};

//                 if ((std::min(v_left_front, v_target) >
//                      v_front_lb + interp(v_front_lb, xp, fp)) &&
//                     tr.d_rel < 80 && v_left_front > v_ego + 3 &&
//                     (left_direct_exist ||
//                      (!left_direct_exist &&
//                       (!r_enable || !right_direct_exist) &&
//                       (dist_to_intsect - tr.d_rel > 40 ||
//                        tr.d_max_cpath < lb_width_l || tr.v_lead < 1)) ||
//                      dist_to_intsect < -5) &&
//                     (dist_to_intsect - tr.d_rel > 40 ||
//                      (std::fabs(tr.v_lead) < 1 && dist_to_intsect < -5))) {
//                   double d_stop = 0;
//                   if (left_boundary_info.type_segments_size > 0 &&
//                       left_boundary_info.type_segments[0].type ==
//                           iflyauto::LaneBoundaryType_MARKING_DASHED) {
//                     if (!is_on_highway) {
//                       if (lane_merge_split_point.existence == 0 ||
//                           (lane_merge_split_point.merge_split_point_data_size >
//                                0 &&
//                            lane_merge_split_point.merge_split_point_data[0]
//                                    .distance < 0 &&
//                            lane_merge_split_point.merge_split_point_data[0]
//                                        .distance +
//                                    lane_merge_split_point
//                                        .merge_split_point_data[0]
//                                        .length <
//                                0) ||
//                           (lane_merge_split_point.merge_split_point_data_size >
//                                0 &&
//                            lane_merge_split_point.merge_split_point_data[0]
//                                .is_split &&
//                            !lane_merge_split_point.merge_split_point_data[0]
//                                 .is_continue)) {
//                         if (left_boundary_info.type_segments_size > 0) {
//                           d_stop = left_boundary_info.type_segments[0].length;
//                         }

//                       } else if (lane_merge_split_point
//                                          .merge_split_point_data_size > 0 &&
//                                  lane_merge_split_point
//                                          .merge_split_point_data[0]
//                                          .distance > 0 &&
//                                  left_boundary_info.type_segments_size > 0) {
//                         d_stop = std::min(
//                             lane_merge_split_point.merge_split_point_data[0]
//                                 .distance,
//                             left_boundary_info.type_segments[0].length);
//                       } else {
//                         d_stop = -10000;
//                       }
//                     } else {
//                       if (left_boundary_info.type_segments_size > 0) {
//                         d_stop = std::min(
//                             (double)left_boundary_info.type_segments[0].length >
//                                     kInputBoundaryLenLimit
//                                 ? kDefaultBoundaryLen
//                                 : left_boundary_info.type_segments[0].length,
//                             dis_to_ramp - 200.);
//                       }
//                     }
//                   } else if (dist_to_intsect < -5) {
//                     d_stop = dist_to_last_intsect;
//                   } else {
//                     d_stop = -10000;
//                   }

//                   d_stop_l_ = d_stop;

//                   double d_lb_car = tr.d_rel + tr.length + safety_dist;
//                   d_lb_car_l_ = d_lb_car;

//                   double t = 0;
//                   double acc_t = 0.0;
//                   double acc_delta_x = 0.0;
//                   double v_aver = v_target;
//                   if (tr.v_rel != 0) {
//                     t = d_lb_car /
//                         std::max(
//                             ((std::min(v_left_front, v_target) + v_ego) / 2 -
//                              v_front_lb),
//                             0.1);
//                     v_aver = (std::min(v_left_front, v_target) + v_ego) / 2;
//                     std::array<double, 2> xp3{40 / 3.6, 80 / 3.6};
//                     std::array<double, 2> fp3{0.7, 0.4};
//                     double acc = interp(v_target, xp3, fp3);
//                     if (v_ego < (std::min(v_left_front, v_target) - 4.) &&
//                         v_left_front - v_ego > 5) {
//                       acc_t = (std::min(v_left_front, v_target) - v_ego) / acc;
//                       acc_delta_x = acc_t * acc_t / 2 * acc +
//                                     (v_ego - v_front_lb) * acc_t;
//                       if (acc_delta_x > d_lb_car) {
//                         t = std::sqrt(std::pow(v_ego - v_front_lb, 2) * 2.25 +
//                                       3 * d_lb_car) -
//                             (v_ego - v_front_lb) * 1.5;
//                         v_aver = v_ego + acc * t / 2;
//                       } else {
//                         t = acc_t +
//                             (d_lb_car - acc_delta_x) /
//                                 std::max((std::min(v_left_front, v_target) -
//                                           v_front_lb),
//                                          0.001);
//                         v_aver = (std::min(v_left_front, v_target) + v_ego) / 2;
//                       }
//                     }
//                   } else if (v_ego >= 1) {
//                     t = d_lb_car / 2.0;
//                     v_aver = v_ego;
//                   } else {
//                     t = d_lb_car;
//                     v_aver = v_ego;
//                   }

//                   t_surpass_l_ = t;

//                   double diff_dn_car = 0;
//                   double desired_gap = 0;
//                   double diff_thre = std::min(v_left_front, v_target) * 2;

//                   left_lb_car_cnt_[tr.track_id].neg = 0;
//                   left_alc_car_cnt_[tr.track_id].neg = 0;

//                   if (left_lane_tasks_id == 1 && current_lane_tasks_id == 0 &&
//                       (dis_to_ramp >= 1500. || !is_on_highway)) {
//                     std::array<double, 4> xp{0, 30, 100, 200};
//                     std::array<double, 4> fp{40 * coefficient, 20 * coefficient,
//                                              -10 * coefficient,
//                                              -20 * coefficient};
//                     std::array<double, 4> xp_lat{-1.0, -0.5, 0.5, 1.5};
//                     std::array<double, 4> fp_lat{1.0, 0.7, 0.2, 0.1};
//                     pos_thr_l =
//                         std::max((int)(10 * coefficient),
//                                  pos_thr_l + (int)interp(d_stop, xp, fp));
//                     pos_thr_lb_l = std::max(
//                         (int)(10 * coefficient),
//                         (int)(pos_thr_lb_l *
//                               interp(l_ego - tr.d_max_cpath, xp_lat, fp_lat)) +
//                             (int)interp(d_stop, xp, fp));
//                     LOG_DEBUG("objselector pos_thr_l[%d], pos_thr_lb_l[%d]",
//                               pos_thr_l, pos_thr_lb_l);

//                     if (d_stop > d_lb_car + 10) {
//                       if ((d_stop > v_aver * (t + 5) && v_ego >= 1) ||
//                           (v_ego < 1 &&
//                            d_stop > std::max(v_ego, 0.1) * (t + 5)) ||
//                           tr.v_lead < 1) {
//                         if (lead_one != nullptr) {
//                           if (tr.track_id == lead_one->track_id) {
//                             if (lead_two != nullptr) {
//                               calc_desired_gap(v_ego, *lead_two, tr, t,
//                                                d_offset, t_gap, safety_dist,
//                                                diff_dn_car, desired_gap);

//                               if (diff_dn_car >
//                                   std::max(desired_gap, diff_thre)) {
//                                 if (tr.d_max_cpath < lb_width_l || need_lb) {
//                                   if (perception_range > lead_one->d_rel &&
//                                       lead_two->d_rel - perception_range >
//                                           std::max(desired_gap, diff_thre)) {
//                                     left_lb_car_cnt_[tr.track_id].pos += 3;
//                                   } else {
//                                     left_lb_car_cnt_[tr.track_id].pos += 1;
//                                   }
//                                 } else {
//                                   if (perception_range > lead_one->d_rel &&
//                                       lead_two->d_rel - perception_range >
//                                           std::max(desired_gap, diff_thre)) {
//                                     left_alc_car_cnt_[tr.track_id].pos += 2;
//                                   } else if (perception_range >
//                                                  lead_one->d_rel &&
//                                              lead_two->d_rel -
//                                                      perception_range <
//                                                  min_visible_gap &&
//                                              lead_two->v_rel + v_ego < 1.0) {
//                                     left_alc_car_cnt_[tr.track_id].pos += 0;
//                                   } else {
//                                     left_alc_car_cnt_[tr.track_id].pos += 1;
//                                   }
//                                   if (!premover_ &&
//                                       left_alc_car_cnt_[tr.track_id].pos >=
//                                           pos_thr_l * 0.4) {
//                                     if (!premovel_ &&
//                                         (tr.track_id == neg_premoved_id_)) {
//                                       left_alc_car_cnt_[tr.track_id].pos = 0;
//                                       neg_premoved_id_ = -1000;
//                                     } else if (premoved_id_ != tr.track_id &&
//                                                v_ego < 30 / 3.6) {
//                                       premoved_id_ = tr.track_id;
//                                       premovel_ = true;
//                                       premove_dist_ =
//                                           std::max((flane->width() / 2 -
//                                                     car_width / 2 + press_thr),
//                                                    0.0);
//                                     }
//                                   }
//                                   if (premovel_ &&
//                                       std::fabs(
//                                           premove_dist_ -
//                                           std::max((flane->width() / 2 -
//                                                     car_width / 2 + press_thr),
//                                                    0.0)) > 0.2) {
//                                     premove_dist_ =
//                                         std::max((flane->width() / 2 -
//                                                   car_width / 2 + press_thr),
//                                                  0.0);
//                                   }
//                                 }
//                               } else {
//                                 left_lb_car_cnt_[tr.track_id].pos = 0;
//                                 left_lb_car_cnt_[tr.track_id].neg =
//                                     neg_thr_lb_l + 1;
//                                 neg_left_lb_car_ = true;
//                                 lb_leadone_disable = true;
//                                 // if ((current_lane_index == lanes_num - 1 ||
//                                 //      (current_lane_index + 1 < lanes_num &&
//                                 //       (map_info.lane_type(current_lane_index
//                                 //       + 1) ==
//                                 //             MSD_LANE_TYPE_PARKING ||
//                                 //        map_info.lane_type(current_lane_index
//                                 //        + 1) ==
//                                 //             MSD_LANE_TYPE_NON_MOTOR))) &&
//                                 //     (map_info.lane_marks_size(
//                                 //          current_lane_index) == 1 ||
//                                 //      (map_info.lane_marks_size(
//                                 //           current_lane_index) > 1 &&
//                                 //       left_direct_has_straight)) &&
//                                 //     traffic_light_direction ==
//                                 //         MSD_DIRECTION_TURN_RIGHT &&
//                                 //     curr_direct_exist) {
//                                 //   left_alc_car_cnt_[tr.track_id].pos += 1;
//                                 //   pos_thr_l = (int)(10 * coefficient);
//                                 // }
//                               }
//                             } else {
//                               if (tr.d_max_cpath < lb_width_l || need_lb) {
//                                 if (perception_range > lead_one->d_rel &&
//                                     perception_range <
//                                         lead_one->d_rel + max_visible_gap &&
//                                     lead_one->d_rel + max_visible_gap <
//                                         max_perception_range) {
//                                   left_lb_car_cnt_[tr.track_id].pos += 5;
//                                 } else {
//                                   left_lb_car_cnt_[tr.track_id].pos += 1;
//                                 }
//                               } else {
//                                 if (perception_range > lead_one->d_rel &&
//                                     perception_range <
//                                         lead_one->d_rel + max_visible_gap &&
//                                     lead_one->d_rel + max_visible_gap <
//                                         max_perception_range &&
//                                     (v_ego > 40 / 3.6 || !is_on_highway)) {
//                                   left_alc_car_cnt_[tr.track_id].pos += 3;
//                                 } else {
//                                   left_alc_car_cnt_[tr.track_id].pos += 1;
//                                 }
//                                 if (!premover_ &&
//                                     left_alc_car_cnt_[tr.track_id].pos >=
//                                         pos_thr_l * 0.4) {
//                                   if (!premovel_ &&
//                                       (tr.track_id == neg_premoved_id_)) {
//                                     left_alc_car_cnt_[tr.track_id].pos = 0;
//                                     neg_premoved_id_ = -1000;
//                                   } else if (premoved_id_ != tr.track_id &&
//                                              v_ego < 30 / 3.6) {
//                                     premoved_id_ = tr.track_id;
//                                     premovel_ = true;
//                                     premove_dist_ =
//                                         std::max((flane->width() / 2 -
//                                                   car_width / 2 + press_thr),
//                                                  0.0);
//                                   }
//                                 }
//                                 if (premovel_ &&
//                                     std::fabs(
//                                         premove_dist_ -
//                                         std::max((flane->width() / 2 -
//                                                   car_width / 2 + press_thr),
//                                                  0.0)) > 0.2) {
//                                   premove_dist_ =
//                                       std::max((flane->width() / 2 -
//                                                 car_width / 2 + press_thr),
//                                                0.0);
//                                 }
//                               }
//                             }
//                           } else {
//                             calc_desired_gap(v_ego, tr, *lead_one, t, d_offset,
//                                              t_gap, safety_dist, diff_dn_car,
//                                              desired_gap);

//                             if (std::fabs(tr.d_center_cpath -
//                                           lead_one->d_center_cpath) < 1.0 &&
//                                 tr.d_rel > lead_one->d_rel &&
//                                 diff_dn_car <
//                                     std::max(desired_gap, diff_thre)) {
//                               left_lb_car_cnt_[lead_one->track_id].pos = 0;
//                               left_lb_car_cnt_[lead_one->track_id].neg =
//                                   neg_thr_lb_l + 1;

//                               neg_left_lb_car_ = true;
//                               lb_leadone_disable = true;
//                             }

//                             calc_desired_gap(v_ego, *lead_one, tr, t, d_offset,
//                                              t_gap, safety_dist, diff_dn_car,
//                                              desired_gap);

//                             if (diff_dn_car > desired_gap) {
//                               if (tr.d_max_cpath < lb_width_l || need_lb) {
//                                 left_lb_car_cnt_[tr.track_id].pos += 1;
//                               } else {
//                                 left_alc_car_cnt_[tr.track_id].pos += 1;
//                               }
//                             }
//                           }
//                         } else {
//                           if (temp_leadone != nullptr) {
//                             if (tr.track_id == temp_leadone->track_id) {
//                               if (temp_leadtwo != nullptr) {
//                                 calc_desired_gap(v_ego, *temp_leadtwo, tr, t,
//                                                  d_offset, t_gap, safety_dist,
//                                                  diff_dn_car, desired_gap);

//                                 if (diff_dn_car <
//                                     std::max(desired_gap, diff_thre)) {
//                                   left_lb_car_cnt_[tr.track_id].pos = 0;
//                                   left_lb_car_cnt_[tr.track_id].neg =
//                                       neg_thr_lb_l + 1;
//                                   neg_left_lb_car_ = true;
//                                   lb_leadone_disable = true;
//                                   // if (traffic_light_direction ==
//                                   //         MSD_DIRECTION_TURN_RIGHT &&
//                                   //     (current_lane_index == lanes_num - 1 ||
//                                   //      (current_lane_index + 1 < lanes_num &&
//                                   //       (map_info.lane_type(current_lane_index
//                                   //       + 1) ==
//                                   //             MSD_LANE_TYPE_PARKING ||
//                                   //        map_info.lane_type(current_lane_index
//                                   //        + 1) ==
//                                   //             MSD_LANE_TYPE_NON_MOTOR)))) {
//                                   //   left_alc_car_cnt_[tr.track_id].pos += 1;
//                                   // }
//                                 } else {
//                                   if (tr.d_max_cpath < lb_width_l || need_lb) {
//                                     left_lb_car_cnt_[tr.track_id].pos += 1;
//                                   } else {
//                                     left_alc_car_cnt_[tr.track_id].pos += 1;
//                                   }
//                                 }
//                               }
//                             } else {
//                               calc_desired_gap(v_ego, tr, *temp_leadone, t,
//                                                d_offset, t_gap, safety_dist,
//                                                diff_dn_car, desired_gap);

//                               if (std::fabs(tr.d_center_cpath -
//                                             temp_leadone->d_center_cpath) <
//                                       1.0 &&
//                                   tr.d_rel > temp_leadone->d_rel &&
//                                   diff_dn_car <
//                                       std::max(desired_gap, diff_thre)) {
//                                 left_lb_car_cnt_[temp_leadone->track_id].pos =
//                                     0;
//                                 left_lb_car_cnt_[temp_leadone->track_id].neg =
//                                     neg_thr_lb_l + 1;

//                                 neg_left_lb_car_ = true;
//                                 lb_leadone_disable = true;
//                               }
//                             }
//                           } else {
//                             if (tr.d_max_cpath < lb_width_l || need_lb) {
//                               left_lb_car_cnt_[tr.track_id].pos += 1;
//                             } else {
//                               left_alc_car_cnt_[tr.track_id].pos += 1;
//                             }
//                           }
//                         }
//                       } else if ((d_stop > v_aver * (t + 1) && v_ego >= 1) ||
//                                  (v_ego < 1 &&
//                                   d_stop > std::max(v_ego, 0.1) * (t + 2))) {
//                         left_alc_car_cnt_[tr.track_id].pos = 0;
//                         remove_car(left_alc_car_, tr.track_id);

//                         if (lead_one != nullptr) {
//                           if (tr.track_id == lead_one->track_id) {
//                             if (lead_two != nullptr) {
//                               calc_desired_gap(v_ego, *lead_two, tr, t,
//                                                d_offset, t_gap, safety_dist,
//                                                diff_dn_car, desired_gap);

//                               if (diff_dn_car >
//                                       std::max(desired_gap, diff_thre) &&
//                                   tr.d_max_cpath < lb_width_l) {
//                                 if (perception_range > lead_one->d_rel &&
//                                     lead_two->d_rel - perception_range >
//                                         std::max(desired_gap, diff_thre)) {
//                                   left_lb_car_cnt_[tr.track_id].pos += 3;
//                                 } else {
//                                   left_lb_car_cnt_[tr.track_id].pos += 1;
//                                 }
//                               } else {
//                                 left_lb_car_cnt_[tr.track_id].pos = 0;
//                                 left_lb_car_cnt_[tr.track_id].neg =
//                                     neg_thr_lb_l + 1;
//                                 neg_left_lb_car_ = true;
//                                 lb_leadone_disable = true;
//                               }
//                             } else if (tr.d_max_cpath < lb_width_l || need_lb) {
//                               if (perception_range > lead_one->d_rel &&
//                                   perception_range <
//                                       lead_one->d_rel + max_visible_gap &&
//                                   lead_one->d_rel + max_visible_gap <
//                                       max_perception_range) {
//                                 left_lb_car_cnt_[tr.track_id].pos += 5;
//                               } else {
//                                 left_lb_car_cnt_[tr.track_id].pos += 1;
//                               }
//                             }
//                           } else {
//                             calc_desired_gap(v_ego, tr, *lead_one, t, d_offset,
//                                              t_gap, safety_dist, diff_dn_car,
//                                              desired_gap);

//                             if (std::fabs(tr.d_center_cpath -
//                                           lead_one->d_center_cpath) < 1.0 &&
//                                 tr.d_rel > lead_one->d_rel &&
//                                 diff_dn_car <
//                                     std::max(desired_gap, diff_thre)) {
//                               left_lb_car_cnt_[lead_one->track_id].pos = 0;
//                               left_lb_car_cnt_[lead_one->track_id].neg =
//                                   neg_thr_lb_l + 1;

//                               neg_left_lb_car_ = true;
//                               lb_leadone_disable = true;
//                             }

//                             calc_desired_gap(v_ego, *lead_one, tr, t, d_offset,
//                                              t_gap, safety_dist, diff_dn_car,
//                                              desired_gap);

//                             if (diff_dn_car > desired_gap &&
//                                 tr.d_max_cpath < lb_width_l) {
//                               left_lb_car_cnt_[tr.track_id].pos += 1;
//                             }
//                           }
//                         } else {
//                           if (temp_leadone != nullptr) {
//                             if (tr.track_id == temp_leadone->track_id) {
//                               if (temp_leadtwo != nullptr) {
//                                 calc_desired_gap(v_ego, *temp_leadtwo, tr, t,
//                                                  d_offset, t_gap, safety_dist,
//                                                  diff_dn_car, desired_gap);

//                                 if (diff_dn_car <
//                                     std::max(desired_gap, diff_thre)) {
//                                   left_lb_car_cnt_[tr.track_id].pos = 0;
//                                   left_lb_car_cnt_[tr.track_id].neg =
//                                       neg_thr_lb_l + 1;
//                                   neg_left_lb_car_ = true;
//                                   lb_leadone_disable = true;
//                                 } else if (tr.d_max_cpath < lb_width_l ||
//                                            need_lb) {
//                                   left_lb_car_cnt_[tr.track_id].pos += 1;
//                                 }
//                               }
//                             } else {
//                               calc_desired_gap(v_ego, tr, *temp_leadone, t,
//                                                d_offset, t_gap, safety_dist,
//                                                diff_dn_car, desired_gap);

//                               if (std::fabs(tr.d_center_cpath -
//                                             temp_leadone->d_center_cpath) <
//                                       1.0 &&
//                                   tr.d_rel > temp_leadone->d_rel &&
//                                   diff_dn_car <
//                                       std::max(desired_gap, diff_thre)) {
//                                 left_lb_car_cnt_[temp_leadone->track_id].pos =
//                                     0;
//                                 left_lb_car_cnt_[temp_leadone->track_id].neg =
//                                     neg_thr_lb_l + 1;

//                                 neg_left_lb_car_ = true;
//                                 lb_leadone_disable = true;
//                               }
//                             }
//                           } else if (tr.d_max_cpath < lb_width_l || need_lb) {
//                             left_lb_car_cnt_[tr.track_id].pos += 1;
//                           }
//                         }
//                       } else if (d_stop > v_aver * 4 && left_direct_exist &&
//                                  !isRedLightStop) {
//                         left_alc_car_cnt_[tr.track_id].pos += 1;
//                         left_lb_car_cnt_[tr.track_id].pos = 0;

//                         remove_car(left_lb_car_, tr.track_id);
//                       } else {
//                         left_alc_car_cnt_[tr.track_id].pos = 0;
//                         left_lb_car_cnt_[tr.track_id].pos = 0;

//                         remove_car(left_alc_car_, tr.track_id);
//                         remove_car(left_lb_car_, tr.track_id);
//                       }
//                     } else {
//                       if (d_stop > v_aver * 3 && left_direct_exist) {
//                         if ((!isRedLightStop &&
//                              (left_laneout_direct_exist ||
//                               (dist_to_intsect + intersect_length +
//                                    lout_r_dash_length >
//                                d_lb_car + 30)) &&
//                              !is_on_highway) ||
//                             (std::min(dis_to_ramp, lc_end_dis) >
//                                  d_lb_car + std::max(v_target * 2, 30.0) &&
//                              is_on_highway)) {
//                           left_alc_car_cnt_[tr.track_id].pos += 1;
//                           left_lb_car_cnt_[tr.track_id].pos = 0;

//                           remove_car(left_lb_car_, tr.track_id);
//                         } else if (status != LCHANGE) {
//                           left_alc_car_cnt_[tr.track_id].pos = 0;
//                           left_lb_car_cnt_[tr.track_id].pos = 0;

//                           remove_car(left_alc_car_, tr.track_id);
//                           remove_car(left_lb_car_, tr.track_id);
//                         }
//                       }
//                     }
//                   } else if ((left_lane_tasks_id == 0 &&
//                               current_lane_tasks_id == 0) ||
//                              (is_on_highway && ((left_lane_tasks_id == 0 &&
//                                                  current_lane_tasks_id == -1) ||
//                                                 (left_lane_tasks_id == -1 &&
//                                                  current_lane_tasks_id == -2) ||
//                                                 (dis_to_ramp > 4500.)))) {
//                     std::array<double, 4> xp_pos_l{4, 7, 10, 20};
//                     std::array<double, 4> xp_pos_lb{4, 7, 10, 20};
//                     std::array<double, 4> fp_pos_l{1.0, 0.6, 0.3, 0.1};
//                     std::array<double, 4> fp_pos_lb{1.0, 0.5, 0.2, 0.1};
//                     std::array<double, 4> xp_pos{0, 50, 100, 200};
//                     std::array<double, 4> fp_pos{20 * coefficient, 0,
//                                                  -10 * coefficient,
//                                                  -20 * coefficient};
//                     std::array<double, 4> xp_lat{-1.0, -0.5, 0.5, 1.5};
//                     std::array<double, 4> fp_lat{1.0, 0.7, 0.2, 0.1};

//                     pos_thr_l = std::max(
//                         (int)(10 * coefficient),
//                         (int)(pos_thr_l * interp(v_left_front - v_front_lb,
//                                                  xp_pos_l, fp_pos_l)));

//                     pos_thr_lb_l = std::max(
//                         (int)(10 * coefficient),
//                         (int)(pos_thr_lb_l *
//                               interp(v_left_front - v_front_lb, xp_pos_lb,
//                                      fp_pos_lb) *
//                               interp(l_ego - tr.d_max_cpath, xp_lat, fp_lat)));

//                     if (!is_on_highway) {
//                       pos_thr_l = std::max(
//                           (int)(10 * coefficient),
//                           pos_thr_l + (int)interp(d_stop, xp_pos, fp_pos));

//                       pos_thr_lb_l = std::max(
//                           (int)(10 * coefficient),
//                           pos_thr_lb_l + (int)interp(d_stop, xp_pos, fp_pos));
//                     }

//                     if ((d_stop > v_aver * (t + 2) && v_ego >= 1) ||
//                         (v_ego < 1 &&
//                          d_stop > std::max(v_ego, 0.1) * (t + 2))) {
//                       left_alc_car_cnt_[tr.track_id].pos += 1;

//                       if (lead_one != nullptr) {
//                         if (tr.track_id == lead_one->track_id) {
//                           if (lead_two != nullptr) {
//                             calc_desired_gap(v_ego, *lead_two, tr, t, d_offset,
//                                              t_gap, safety_dist, diff_dn_car,
//                                              desired_gap);

//                             if (diff_dn_car >
//                                 std::max(desired_gap, diff_thre)) {
//                               if (tr.d_max_cpath < lb_width_l) {
//                                 if (perception_range > lead_one->d_rel &&
//                                     lead_two->d_rel - perception_range >
//                                         std::max(desired_gap, diff_thre)) {
//                                   left_lb_car_cnt_[tr.track_id].pos += 3;
//                                 } else {
//                                   left_lb_car_cnt_[tr.track_id].pos += 1;
//                                 }
//                               }
//                             } else {
//                               left_lb_car_cnt_[tr.track_id].pos = 0;
//                               left_lb_car_cnt_[tr.track_id].neg =
//                                   neg_thr_lb_l + 1;

//                               neg_left_lb_car_ = true;
//                               lb_leadone_disable = true;
//                             }
//                             if (perception_range > lead_one->d_rel &&
//                                 lead_two->d_rel - perception_range >
//                                     std::max(desired_gap, diff_thre)) {
//                               left_alc_car_cnt_[tr.track_id].pos += 2;
//                             }
//                           } else {
//                             if (tr.d_max_cpath < lb_width_l) {
//                               if (perception_range > lead_one->d_rel &&
//                                   perception_range <
//                                       lead_one->d_rel + max_visible_gap &&
//                                   lead_one->d_rel + max_visible_gap <
//                                       max_perception_range) {
//                                 left_lb_car_cnt_[tr.track_id].pos += 5;
//                               } else {
//                                 left_lb_car_cnt_[tr.track_id].pos += 1;
//                               }
//                             }
//                             if (perception_range > lead_one->d_rel &&
//                                 perception_range <
//                                     lead_one->d_rel + max_visible_gap &&
//                                 lead_one->d_rel + max_visible_gap <
//                                     max_perception_range &&
//                                 (v_ego > 40 / 3.6 || !is_on_highway)) {
//                               left_alc_car_cnt_[tr.track_id].pos += 3;
//                             }
//                           }
//                         } else {
//                           calc_desired_gap(v_ego, tr, *lead_one, t, d_offset,
//                                            t_gap, safety_dist, diff_dn_car,
//                                            desired_gap);

//                           if (std::fabs(tr.d_center_cpath -
//                                         lead_one->d_center_cpath) < 1.0 &&
//                               tr.d_rel > lead_one->d_rel &&
//                               diff_dn_car < std::max(desired_gap, diff_thre)) {
//                             left_lb_car_cnt_[lead_one->track_id].pos = 0;
//                             left_lb_car_cnt_[lead_one->track_id].neg =
//                                 neg_thr_lb_l + 1;

//                             neg_left_lb_car_ = true;
//                             lb_leadone_disable = true;
//                           }

//                           calc_desired_gap(v_ego, *lead_one, tr, t, d_offset,
//                                            t_gap, safety_dist, diff_dn_car,
//                                            desired_gap);

//                           if (diff_dn_car > desired_gap &&
//                               tr.d_max_cpath < lb_width_l) {
//                             left_lb_car_cnt_[tr.track_id].pos += 1;
//                           }
//                         }
//                       } else {
//                         if (temp_leadone != nullptr) {
//                           if (tr.track_id == temp_leadone->track_id) {
//                             if (temp_leadtwo != nullptr) {
//                               calc_desired_gap(v_ego, *temp_leadtwo, tr, t,
//                                                d_offset, t_gap, safety_dist,
//                                                diff_dn_car, desired_gap);

//                               if (diff_dn_car <
//                                   std::max(desired_gap, diff_thre)) {
//                                 left_lb_car_cnt_[tr.track_id].pos = 0;
//                                 left_lb_car_cnt_[tr.track_id].neg =
//                                     neg_thr_lb_l + 1;

//                                 neg_left_lb_car_ = true;
//                                 lb_leadone_disable = true;
//                               } else {
//                                 if (tr.d_max_cpath < lb_width_l) {
//                                   left_lb_car_cnt_[tr.track_id].pos += 1;
//                                 }
//                               }
//                             }
//                           } else {
//                             calc_desired_gap(v_ego, tr, *temp_leadone, t,
//                                              d_offset, t_gap, safety_dist,
//                                              diff_dn_car, desired_gap);

//                             if (std::fabs(tr.d_center_cpath -
//                                           temp_leadone->d_center_cpath) < 1.0 &&
//                                 tr.d_rel > temp_leadone->d_rel &&
//                                 diff_dn_car <
//                                     std::max(desired_gap, diff_thre)) {
//                               left_lb_car_cnt_[temp_leadone->track_id].pos = 0;
//                               left_lb_car_cnt_[temp_leadone->track_id].neg =
//                                   neg_thr_lb_l + 1;

//                               neg_left_lb_car_ = true;
//                               lb_leadone_disable = true;
//                             }
//                           }
//                         } else if (tr.d_max_cpath < lb_width_l) {
//                           left_lb_car_cnt_[tr.track_id].pos += 1;
//                         }
//                       }
//                     } else if (d_stop > v_ego * 2) {
//                       left_alc_car_cnt_[tr.track_id].pos += 1;
//                       left_lb_car_cnt_[tr.track_id].pos = 0;

//                       remove_car(left_lb_car_, tr.track_id);
//                     } else {
//                       left_alc_car_cnt_[tr.track_id].pos = 0;
//                       left_lb_car_cnt_[tr.track_id].pos = 0;

//                       remove_car(left_alc_car_, tr.track_id);
//                       remove_car(left_lb_car_, tr.track_id);
//                     }
//                   } else if (left_lane_tasks_id >= 2 &&
//                              current_lane_tasks_id >= 1 &&
//                              dis_to_ramp > 2000.) {
//                     std::array<double, 4> xp{0, 50, 100, 200};
//                     std::array<double, 4> fp{40 * coefficient, 20 * coefficient,
//                                              0, -20 * coefficient};
//                     std::array<double, 4> xp_lat{-1.0, -0.5, 0.5, 1.5};
//                     std::array<double, 4> fp_lat{1.0, 0.7, 0.2, 0.1};
//                     if (!is_on_highway) {
//                       pos_thr_l =
//                           std::max((int)(10 * coefficient),
//                                    pos_thr_l + (int)interp(d_stop, xp, fp));
//                       pos_thr_lb_l = std::max(
//                           (int)(10 * coefficient),
//                           (int)(pos_thr_lb_l * interp(l_ego - tr.d_max_cpath,
//                                                       xp_lat, fp_lat)) +
//                               (int)interp(d_stop, xp, fp));
//                     }
//                     double d_stop_r = lc_end_dis;
//                     double d_map = d_stop;

//                     if (d_stop > d_lb_car + 30) {
//                       if ((d_stop_r > v_aver * (t + 5 * left_lane_tasks_id) &&
//                            d_map > v_aver * (t + 5) &&
//                            d_stop > v_aver * (t + 5) && v_ego >= 1) ||
//                           (v_ego < 1 &&
//                            d_stop_r > std::max(v_ego, 0.1) * (t + 15) &&
//                            d_map > std::max(v_ego, 0.1) * (t + 5) &&
//                            d_stop > std::max(v_ego, 0.1) * (t + 5))) {
//                         if (lead_one != nullptr) {
//                           if (tr.track_id == lead_one->track_id) {
//                             if (lead_two != nullptr) {
//                               calc_desired_gap(v_ego, *lead_two, tr, t,
//                                                d_offset, t_gap, safety_dist,
//                                                diff_dn_car, desired_gap);

//                               if (diff_dn_car >
//                                   std::max(desired_gap, diff_thre)) {
//                                 if (tr.d_max_cpath < lb_width_l) {
//                                   if (perception_range > lead_one->d_rel &&
//                                       lead_two->d_rel - perception_range >
//                                           std::max(desired_gap, diff_thre)) {
//                                     left_lb_car_cnt_[tr.track_id].pos += 3;
//                                   } else {
//                                     left_lb_car_cnt_[tr.track_id].pos += 1;
//                                   }
//                                 } else {
//                                   if (perception_range > lead_one->d_rel &&
//                                       lead_two->d_rel - perception_range >
//                                           std::max(desired_gap, diff_thre)) {
//                                     left_alc_car_cnt_[tr.track_id].pos += 2;
//                                   } else if (perception_range >
//                                                  lead_one->d_rel &&
//                                              lead_two->d_rel -
//                                                      perception_range <
//                                                  min_visible_gap &&
//                                              lead_two->v_rel + v_ego < 1.0) {
//                                     left_alc_car_cnt_[tr.track_id].pos += 0;
//                                   } else {
//                                     left_alc_car_cnt_[tr.track_id].pos += 1;
//                                   }
//                                 }
//                               } else {
//                                 left_lb_car_cnt_[tr.track_id].pos = 0;
//                                 left_lb_car_cnt_[tr.track_id].neg =
//                                     neg_thr_lb_l + 1;

//                                 neg_left_lb_car_ = true;
//                                 lb_leadone_disable = true;
//                               }
//                             } else {
//                               if (tr.d_max_cpath < lb_width_l) {
//                                 if (perception_range > lead_one->d_rel &&
//                                     perception_range <
//                                         lead_one->d_rel + max_visible_gap &&
//                                     lead_one->d_rel + max_visible_gap <
//                                         max_perception_range) {
//                                   left_lb_car_cnt_[tr.track_id].pos += 5;
//                                 } else {
//                                   left_lb_car_cnt_[tr.track_id].pos += 1;
//                                 }
//                               } else {
//                                 if (perception_range > lead_one->d_rel &&
//                                     perception_range <
//                                         lead_one->d_rel + max_visible_gap &&
//                                     lead_one->d_rel + max_visible_gap <
//                                         max_perception_range &&
//                                     (v_ego > 40 / 3.6 || !is_on_highway)) {
//                                   left_alc_car_cnt_[tr.track_id].pos += 3;
//                                 } else {
//                                   left_alc_car_cnt_[tr.track_id].pos += 1;
//                                 }
//                               }
//                             }
//                           } else {
//                             calc_desired_gap(v_ego, tr, *lead_one, t, d_offset,
//                                              t_gap, safety_dist, diff_dn_car,
//                                              desired_gap);

//                             if (std::fabs(tr.d_center_cpath -
//                                           lead_one->d_center_cpath) < 1.0 &&
//                                 tr.d_rel > lead_one->d_rel &&
//                                 diff_dn_car <
//                                     std::max(desired_gap, diff_thre)) {
//                               left_lb_car_cnt_[lead_one->track_id].pos = 0;
//                               left_lb_car_cnt_[lead_one->track_id].neg =
//                                   neg_thr_lb_l + 1;

//                               neg_left_lb_car_ = true;
//                               lb_leadone_disable = true;
//                             }

//                             calc_desired_gap(v_ego, *lead_one, tr, t, d_offset,
//                                              t_gap, safety_dist, diff_dn_car,
//                                              desired_gap);

//                             if (diff_dn_car > desired_gap) {
//                               if (tr.d_max_cpath < lb_width_l) {
//                                 left_lb_car_cnt_[tr.track_id].pos += 1;
//                               } else {
//                                 left_alc_car_cnt_[tr.track_id].pos += 1;
//                               }
//                             }
//                           }
//                         } else {
//                           if (temp_leadone != nullptr) {
//                             if (tr.track_id == temp_leadone->track_id) {
//                               if (temp_leadtwo != nullptr) {
//                                 calc_desired_gap(v_ego, *temp_leadtwo, tr, t,
//                                                  d_offset, t_gap, safety_dist,
//                                                  diff_dn_car, desired_gap);

//                                 if (diff_dn_car <
//                                     std::max(desired_gap, diff_thre)) {
//                                   left_lb_car_cnt_[tr.track_id].pos = 0;
//                                   left_lb_car_cnt_[tr.track_id].neg =
//                                       neg_thr_lb_l + 1;

//                                   neg_left_lb_car_ = true;
//                                   lb_leadone_disable = true;
//                                 } else {
//                                   if (tr.d_max_cpath < lb_width_l) {
//                                     left_lb_car_cnt_[tr.track_id].pos += 1;
//                                   } else {
//                                     left_alc_car_cnt_[tr.track_id].pos += 1;
//                                   }
//                                 }
//                               }
//                             } else {
//                               calc_desired_gap(v_ego, tr, *temp_leadone, t,
//                                                d_offset, t_gap, safety_dist,
//                                                diff_dn_car, desired_gap);

//                               if (std::fabs(tr.d_center_cpath -
//                                             temp_leadone->d_center_cpath) <
//                                       1.0 &&
//                                   tr.d_rel > temp_leadone->d_rel &&
//                                   diff_dn_car <
//                                       std::max(desired_gap, diff_thre)) {
//                                 left_lb_car_cnt_[temp_leadone->track_id].pos =
//                                     0;
//                                 left_lb_car_cnt_[temp_leadone->track_id].neg =
//                                     neg_thr_lb_l + 1;

//                                 neg_left_lb_car_ = true;
//                                 lb_leadone_disable = true;
//                               }
//                             }
//                           } else {
//                             if (tr.d_max_cpath < lb_width_l) {
//                               left_lb_car_cnt_[tr.track_id].pos += 1;
//                             } else {
//                               left_alc_car_cnt_[tr.track_id].pos += 1;
//                             }
//                           }
//                         }
//                       } else if ((d_stop_r >
//                                       v_aver * (t + 4 * left_lane_tasks_id) &&
//                                   d_map > v_aver * (t + 2) &&
//                                   d_stop > v_aver * (t + 2) && v_ego >= 1) ||
//                                  (v_ego < 1 &&
//                                   d_stop_r > std::max(v_ego, 0.1) * (t + 10) &&
//                                   d_map > std::max(v_ego, 0.1) * (t + 2) &&
//                                   d_stop > std::max(v_ego, 0.1) * (t + 2))) {
//                         left_alc_car_cnt_[tr.track_id].pos = 0;
//                         remove_car(left_alc_car_, tr.track_id);

//                         if (lead_one != nullptr) {
//                           if (tr.track_id == lead_one->track_id) {
//                             if (lead_two != nullptr) {
//                               calc_desired_gap(v_ego, *lead_two, tr, t,
//                                                d_offset, t_gap, safety_dist,
//                                                diff_dn_car, desired_gap);

//                               if (diff_dn_car >
//                                       std::max(desired_gap, diff_thre) &&
//                                   tr.d_max_cpath < lb_width_l) {
//                                 if (perception_range > lead_one->d_rel &&
//                                     lead_two->d_rel - perception_range >
//                                         std::max(desired_gap, diff_thre)) {
//                                   left_lb_car_cnt_[tr.track_id].pos += 3;
//                                 } else {
//                                   left_lb_car_cnt_[tr.track_id].pos += 1;
//                                 }
//                               } else {
//                                 left_lb_car_cnt_[tr.track_id].pos = 0;
//                                 left_lb_car_cnt_[tr.track_id].neg =
//                                     neg_thr_lb_l + 1;

//                                 neg_left_lb_car_ = true;
//                                 lb_leadone_disable = true;
//                               }
//                             } else if (tr.d_max_cpath < lb_width_l) {
//                               if (perception_range > lead_one->d_rel &&
//                                   perception_range <
//                                       lead_one->d_rel + max_visible_gap &&
//                                   lead_one->d_rel + max_visible_gap <
//                                       max_perception_range) {
//                                 left_lb_car_cnt_[tr.track_id].pos += 5;
//                               } else {
//                                 left_lb_car_cnt_[tr.track_id].pos += 1;
//                               }
//                             }
//                           } else {
//                             calc_desired_gap(v_ego, tr, *lead_one, t, d_offset,
//                                              t_gap, safety_dist, diff_dn_car,
//                                              desired_gap);

//                             if (std::fabs(tr.d_center_cpath -
//                                           lead_one->d_center_cpath) < 1.0 &&
//                                 tr.d_rel > lead_one->d_rel &&
//                                 diff_dn_car <
//                                     std::max(desired_gap, diff_thre)) {
//                               left_lb_car_cnt_[lead_one->track_id].pos = 0;
//                               left_lb_car_cnt_[lead_one->track_id].neg =
//                                   neg_thr_lb_l + 1;

//                               neg_left_lb_car_ = true;
//                               lb_leadone_disable = true;
//                             }

//                             calc_desired_gap(v_ego, *lead_one, tr, t, d_offset,
//                                              t_gap, safety_dist, diff_dn_car,
//                                              desired_gap);

//                             if (diff_dn_car > desired_gap &&
//                                 tr.d_max_cpath < lb_width_l) {
//                               left_lb_car_cnt_[tr.track_id].pos += 1;
//                             }
//                           }
//                         } else {
//                           if (temp_leadone != nullptr) {
//                             if (tr.track_id == temp_leadone->track_id) {
//                               if (temp_leadtwo != nullptr) {
//                                 calc_desired_gap(v_ego, *temp_leadtwo, tr, t,
//                                                  d_offset, t_gap, safety_dist,
//                                                  diff_dn_car, desired_gap);

//                                 if (diff_dn_car <
//                                     std::max(desired_gap, diff_thre)) {
//                                   left_lb_car_cnt_[tr.track_id].pos = 0;
//                                   left_lb_car_cnt_[tr.track_id].neg =
//                                       neg_thr_lb_l + 1;

//                                   neg_left_lb_car_ = true;
//                                   lb_leadone_disable = true;
//                                 } else if (tr.d_max_cpath < lb_width_l) {
//                                   left_lb_car_cnt_[tr.track_id].pos += 1;
//                                 }
//                               }
//                             } else {
//                               calc_desired_gap(v_ego, tr, *temp_leadone, t,
//                                                d_offset, t_gap, safety_dist,
//                                                diff_dn_car, desired_gap);

//                               if (std::fabs(tr.d_center_cpath -
//                                             temp_leadone->d_center_cpath) <
//                                       1.0 &&
//                                   tr.d_rel > temp_leadone->d_rel &&
//                                   diff_dn_car <
//                                       std::max(desired_gap, diff_thre)) {
//                                 left_lb_car_cnt_[temp_leadone->track_id].pos =
//                                     0;
//                                 left_lb_car_cnt_[temp_leadone->track_id].neg =
//                                     neg_thr_lb_l + 1;

//                                 neg_left_lb_car_ = true;
//                                 lb_leadone_disable = true;
//                               }
//                             }
//                           } else if (tr.d_max_cpath < lb_width_l) {
//                             left_lb_car_cnt_[tr.track_id].pos += 1;
//                           }
//                         }
//                       } else {
//                         left_alc_car_cnt_[tr.track_id].pos = 0;
//                         left_lb_car_cnt_[tr.track_id].pos = 0;

//                         remove_car(left_alc_car_, tr.track_id);
//                         remove_car(left_lb_car_, tr.track_id);
//                       }
//                     }
//                   } else if ((left_lane_tasks_id == 0 &&
//                               current_lane_tasks_id == -1) ||
//                              (left_lane_tasks_id == -1 &&
//                               current_lane_tasks_id == -2)) {
//                   }

//                   auto iter = std::find(left_alc_car_.begin(),
//                                         left_alc_car_.end(), tr.track_id);

//                   if (left_alc_car_cnt_[tr.track_id].pos >
//                           pos_thr_l +
//                               active_lane_change_min_duration_threshold &&
//                       iter == left_alc_car_.end() &&
//                       left_lb_car_cnt_[tr.track_id].pos < pos_thr_lb_l) {
//                     left_alc_car_.push_back(tr.track_id);
//                     left_alc_car_cnt_[tr.track_id].neg = 0;
//                     neg_left_alc_car_ = false;
//                   }

//                   iter = std::find(left_lb_car_.begin(), left_lb_car_.end(),
//                                    tr.track_id);

//                   if (left_lb_car_cnt_[tr.track_id].pos > pos_thr_lb_l &&
//                       iter == left_lb_car_.end()) {
//                     left_lb_car_.push_back(tr.track_id);
//                     left_lb_car_cnt_[tr.track_id].neg = 0;
//                     neg_left_lb_car_ = false;
//                   }
//                 }

//                 auto iter = std::find(left_alc_car_.begin(),
//                                       left_alc_car_.end(), tr.track_id);

//                 if (left_alc_car_cnt_[tr.track_id].pos >
//                         pos_thr_l + active_lane_change_min_duration_threshold &&
//                     iter == left_alc_car_.end() &&
//                     left_lb_car_cnt_[tr.track_id].pos < pos_thr_lb_l) {
//                   left_alc_car_.push_back(tr.track_id);
//                   neg_left_alc_car_ = false;
//                 }

//                 iter = std::find(left_lb_car_.begin(), left_lb_car_.end(),
//                                  tr.track_id);

//                 if (left_lb_car_cnt_[tr.track_id].pos > pos_thr_lb_l &&
//                     iter == left_lb_car_.end()) {
//                   left_lb_car_.push_back(tr.track_id);
//                   neg_left_lb_car_ = false;
//                 }

//                 LOG_DEBUG("WR: pos_thr_l[%d] pos_thr_lb_l[%d]", pos_thr_l,
//                           pos_thr_lb_l);

//                 double neg_d_stop = 0.0;
//                 if (left_boundary_info.type_segments_size > 0 &&
//                     left_boundary_info.type_segments[0].type ==
//                         iflyauto::LaneBoundaryType_MARKING_DASHED) {
//                   neg_d_stop = left_boundary_info.type_segments[0].length;
//                 } else if (dist_to_intsect < -5) {
//                   neg_d_stop = dist_to_last_intsect;
//                 } else {
//                   neg_d_stop = -10000;
//                 }

//                 std::array<double, 3> fp1{13 / 3.6, 11 / 3.6, 9 / 3.6};
//                 // std::array<double, 3> xp2{0, 40, 70};
//                 // std::array<double, 3> fp2{0.5, 1.5, 2.5};
//                 std::array<double, 4> xp3{0., 3., 5., 10.};
//                 std::array<double, 4> fp3{-2., 0.6, 2., 4.};
//                 // double temp_attenuation = interp(neg_d_stop, xp2, fp2);
//                 double temp_attenuation =
//                     interp(std::max(v_target - v_front_lb, 0.), xp3, fp3);
//                 if (std::min(v_left_front, v_target) <
//                                 v_front_lb + interp(v_front_lb, xp, fp1) ||
//                             v_left_front < v_ego + 2 ||
//                             tr.v_rel > temp_attenuation ||
//                             use_lateral_distance_to_judge_cutout_in_active_lane_change
//                         ? tr.l > half_car_width
//                         : std::fabs(tr.v_lat) > 0.3 ||
//                               ((!LCHANGE && !RCHANGE) && lead_one != nullptr &&
//                                lead_two != nullptr &&
//                                lead_two->d_rel - lead_one->d_rel <
//                                    std::min(v_left_front, v_target) * 2 &&
//                                (lead_two->v_rel < 5. &&
//                                 std::fabs(lead_two->v_lat) < 0.3 &&
//                                 lead_two->d_min_cpath < 1.1 &&
//                                 lead_two->d_min_cpath + lead_two->width >
//                                     1.1))) {
//                   left_lb_car_cnt_[tr.track_id].neg += 1;
//                   left_alc_car_cnt_[tr.track_id].neg += 1;

//                   int lb_pos = left_lb_car_cnt_[tr.track_id].pos;
//                   int alc_pos = left_alc_car_cnt_[tr.track_id].pos;
//                   left_lb_car_cnt_[tr.track_id].pos = std::max(lb_pos - 3, 0);
//                   left_alc_car_cnt_[tr.track_id].pos = std::max(alc_pos - 3, 0);

//                   if (premovel_ &&
//                       left_alc_car_cnt_[tr.track_id].pos < pos_thr_l * 0.1) {
//                     premovel_ = false;
//                   }

//                   if (left_alc_car_cnt_[tr.track_id].neg > neg_thr_l ||
//                       ((!LCHANGE && !RCHANGE) && lead_one != nullptr &&
//                        lead_two != nullptr &&
//                        lead_two->d_rel - lead_one->d_rel <
//                            std::min(v_left_front, v_target) * 1.5 &&
//                        ((lead_one->v_lead > 3)) &&
//                        (lead_two->v_rel < 5. &&
//                         std::fabs(lead_two->v_lat) < 0.3 &&
//                         lead_two->d_min_cpath < 1.1 &&
//                         lead_two->d_min_cpath + lead_two->width > 1.1))) {
//                     if (left_alc_car_.size() > 0 &&
//                         std::find(left_alc_car_.begin(), left_alc_car_.end(),
//                                   tr.track_id) != left_alc_car_.end()) {
//                       neg_premoved_id_ = tr.track_id;
//                       premoved_id_ = -1000;
//                       premovel_ = false;
//                       left_alc_car_.clear();
//                       left_alc_car_cnt_[tr.track_id].pos = 0;
//                       neg_left_alc_car_ = true;
//                     }
//                   }

//                   if (left_lb_car_cnt_[tr.track_id].neg > neg_thr_lb_l ||
//                       ((lead_one != nullptr && lead_two != nullptr &&
//                         lead_two->d_rel - lead_one->d_rel <
//                             std::min(v_left_front, v_target) * 1.5 &&
//                         lead_one->d_max_cpath > lb_width_l / 2.0 &&
//                         ((lead_one->v_lead > 3))) ||
//                        (temp_leadone != nullptr && temp_leadtwo != nullptr &&
//                         left_lb_car_.size() > 0 &&
//                         std::find(left_lb_car_.begin(), left_lb_car_.end(),
//                                   temp_leadone->track_id) !=
//                             left_lb_car_.end() &&
//                         temp_leadtwo->d_rel - temp_leadone->d_rel <
//                             v_target * 1.5 &&
//                         temp_leadone->d_max_cpath > lb_width_l / 2.0 &&
//                         ((temp_leadone->v_lead > 3))))) {
//                     if (left_lb_car_.size() > 0 &&
//                         std::find(left_lb_car_.begin(), left_lb_car_.end(),
//                                   tr.track_id) != left_lb_car_.end()) {
//                       left_lb_car_.clear();

//                       if (true) {
//                         neg_left_lb_car_ = true;
//                       }

//                       left_lb_car_cnt_[tr.track_id].pos = 0;
//                     }
//                   }
//                 }
//               }
//             } else if (LCHANGE &&
//                        (left_alc_car_.size() > 0 || left_lb_car_.size() > 0 ||
//                         v_ego >= 1.0) &&
//                        tr.d_max_cpath <= -lane_width / 2 + car_width / 5 &&
//                        tr.d_max_cpath >
//                            -(lane_width + car_width / 2 + lane_width * 0.05) &&
//                        dist_to_intsect < 170) {
//               v_rel_l_ = get_vrel_close(-1, status);
//               double v_left_front = v_rel_l_ + v_ego;
//               double v_front_lb = tr.v_rel + v_ego;

//               std::array<double, 3> xp1{-22, -10, 0};
//               std::array<double, 3> fp1{20 * coefficient, 35 * coefficient,
//                                         80 * coefficient};

//               std::array<double, 2> xp2{0, 67 / 3.6};
//               std::array<double, 2> fp2{12 / 3.6, 8 / 3.6};

//               int neg_thr_l = int(interp(v_rel_l_, xp1, fp1));
//               int neg_thr_lb_l = neg_thr_l;

//               if (std::min(v_left_front, v_target) <
//                               v_front_lb + interp(v_front_lb, xp2, fp2) ||
//                           v_left_front < v_ego + 2 || tr.v_rel > 2.5 ||
//                           use_lateral_distance_to_judge_cutout_in_active_lane_change
//                       ? tr.l > half_car_width
//                       : std::fabs(tr.v_lat) > 0.3 ||
//                             ((!LCHANGE && !RCHANGE) && lead_one != nullptr &&
//                              lead_two != nullptr &&
//                              lead_two->d_rel - lead_one->d_rel <
//                                  std::min(v_left_front, v_target) * 2 &&
//                              (lead_two->v_rel < 5. &&
//                               std::fabs(lead_two->v_lat) < 0.3 &&
//                               lead_two->d_min_cpath < 1.1 &&
//                               lead_two->d_min_cpath + lead_two->width > 1.1))) {
//                 if (left_lb_car_cnt_.find(tr.track_id) !=
//                     left_lb_car_cnt_.end()) {
//                   left_lb_car_cnt_[tr.track_id].neg += 1;
//                   int lb_pos = left_lb_car_cnt_[tr.track_id].pos;
//                   left_lb_car_cnt_[tr.track_id].pos = std::max(lb_pos - 3, 0);

//                   if (left_lb_car_cnt_[tr.track_id].neg > neg_thr_lb_l ||
//                       ((lead_one != nullptr && lead_two != nullptr &&
//                         lead_two->d_rel - lead_one->d_rel <
//                             std::min(v_left_front, v_target) * 1.5 &&
//                         lead_one->d_max_cpath > lb_width_l / 2.0 &&
//                         ((lead_one->v_lead > 3))) ||
//                        (temp_leadone != nullptr && temp_leadtwo != nullptr &&
//                         left_lb_car_.size() > 0 &&
//                         std::find(left_lb_car_.begin(), left_lb_car_.end(),
//                                   temp_leadone->track_id) !=
//                             left_lb_car_.end() &&
//                         temp_leadtwo->d_rel - temp_leadone->d_rel <
//                             v_target * 1.5 &&
//                         temp_leadone->d_max_cpath > lb_width_l / 2.0 &&
//                         ((temp_leadone->v_lead > 3))))) {
//                     if (left_lb_car_.size() > 0 &&
//                         std::find(left_lb_car_.begin(), left_lb_car_.end(),
//                                   tr.track_id) != left_lb_car_.end()) {
//                       left_lb_car_.clear();

//                       if (true) {
//                         neg_left_lb_car_ = true;
//                       }

//                       left_lb_car_cnt_[tr.track_id].pos = 0;
//                     }
//                   }
//                 }

//                 if (left_alc_car_cnt_.find(tr.track_id) !=
//                     left_alc_car_cnt_.end()) {
//                   left_alc_car_cnt_[tr.track_id].neg += 1;
//                   int alc_pos = left_alc_car_cnt_[tr.track_id].pos;
//                   left_alc_car_cnt_[tr.track_id].pos = std::max(alc_pos - 3, 0);

//                   if (left_alc_car_cnt_[tr.track_id].neg > neg_thr_l ||
//                       ((!LCHANGE && !RCHANGE) && lead_one != nullptr &&
//                        lead_two != nullptr &&
//                        lead_two->d_rel - lead_one->d_rel <
//                            std::min(v_left_front, v_target) * 1.5 &&
//                        ((lead_one->v_lead > 3)) &&
//                        (lead_two->v_rel < 5. &&
//                         std::fabs(lead_two->v_lat) < 0.3 &&
//                         lead_two->d_min_cpath < 1.1 &&
//                         lead_two->d_min_cpath + lead_two->width > 1.1))) {
//                     if (left_alc_car_.size() > 0 &&
//                         std::find(left_alc_car_.begin(), left_alc_car_.end(),
//                                   tr.track_id) != left_alc_car_.end()) {
//                       neg_premoved_id_ = tr.track_id;
//                       premoved_id_ = -1000;
//                       premovel_ = false;
//                       left_alc_car_.clear();
//                       left_alc_car_cnt_[tr.track_id].pos = 0;
//                       neg_left_alc_car_ = true;
//                     }
//                   }
//                 }
//               }
//             }
//           }
//           // 临时hack 过滤由静止障碍物引发的ALC
//           if (tr.v > active_lane_change_min_object_speed_threshold &&
//               ((r_enable && rlane != nullptr) ||
//                (tlane != nullptr &&
//                 tlane->get_virtual_id() == clane->get_virtual_id()))) {
//             if (accident_ahead && right_is_faster_ && tr.is_accident_car &&
//                 ((lead_one == nullptr ||
//                   ((lead_two == nullptr) ||
//                    (tr.track_id != lead_two->track_id)))) &&
//                 (status == kLaneKeeping || is_LC_RWAIT || RCHANGE ||
//                  is_LC_RBACK ||
//                  (LCHANGE && (request_source == MAP_REQUEST ||
//                               request_source == INT_REQUEST))) &&
//                 (l_accident_cnt_ != 1 || llane == nullptr) &&
//                 (dist_to_intsect - tr.d_rel >= 35 || dist_to_intsect < -5)) {
//               if (dist_to_intsect > 0 || dist_to_intsect < -5) {
//                 if (((right_boundary_info.type_segments_size == 2 ||
//                       right_boundary_info.type_segments_size == 1) &&
//                      right_boundary_info.type_segments[0].type ==
//                          iflyauto::LaneBoundaryType_MARKING_DASHED &&
//                      ((!right_direct_exist &&
//                        right_boundary_info.type_segments[0].length - tr.d_rel >
//                            80 &&
//                        !left_direct_exist &&
//                        (olane == nullptr ||
//                         (olane != nullptr && olane->get_virtual_id() ==
//                                                  clane->get_virtual_id()))) ||
//                       (right_direct_exist &&
//                        right_boundary_info.type_segments[0].length >
//                            tr.d_rel))) ||
//                     right_boundary_info.type_segments_size > 3 ||
//                     dist_to_intsect < -5 ||
//                     (tlane != nullptr &&
//                      tlane->get_virtual_id() == clane->get_virtual_id())) {
//                   r_accident_cnt_ = 1;
//                   right_lb_car_.clear();
//                   right_alc_car_.clear();

//                   right_lb_car_.push_back(tr.track_id);
//                   right_alc_car_.push_back(tr.track_id);

//                   neg_right_lb_car_ = false;
//                   neg_right_alc_car_ = false;
//                   if (((d_max < lane_width / 2 - car_width - 0.3 &&
//                         !is_in_intersection &&
//                         front_tracks_cone_ids.size() > 1) ||
//                        (is_in_intersection &&
//                         (d_max < 0.5 &&
//                          dist_to_last_intsect - max_d_rel > 15))) &&
//                       !accident_front) {
//                     if (front_tracks_cone_ids.find(right_alc_car_[0]) !=
//                         front_tracks_cone_ids.end()) {
//                       right_lb_car_.clear();
//                       right_alc_car_.clear();
//                       r_accident_cnt_ = 0;
//                     }
//                   }
//                   if (((lead_two != nullptr && (!LCHANGE && !RCHANGE) &&
//                         lead_two->d_rel - lead_one->d_rel < 20 &&
//                         lead_two->d_rel - lead_one->d_rel > 5 &&
//                         lead_two->type != 20001 && lead_one->type != 20001) ||
//                        ((temp_leadtwo != nullptr &&
//                          temp_leadtwo->d_rel - temp_leadone->d_rel < 20 &&
//                          temp_leadtwo->d_rel - temp_leadone->d_rel > 5 &&
//                          temp_leadtwo->type != 20001 &&
//                          temp_leadone->type != 20001) ||
//                         (left_close_objs_.size() > 2 && tlane != nullptr &&
//                          tlane->get_virtual_id() == clane->get_virtual_id()) ||
//                         (current_close_objs_.size() > 2 && olane != nullptr &&
//                          olane->get_virtual_id() ==
//                              clane->get_virtual_id()))) &&
//                       (((olane == nullptr ||
//                          (olane != nullptr && olane->get_virtual_id() ==
//                                                   clane->get_virtual_id())) &&
//                         !right_direct_exist) ||
//                        is_in_intersection ||
//                        (tlane != nullptr &&
//                         tlane->get_virtual_id() == clane->get_virtual_id() &&
//                         !curr_direct_exist)) &&
//                       dist_to_intsect < 240) {
//                     right_lb_car_.clear();
//                     right_alc_car_.clear();
//                     neg_right_alc_car_ = true;
//                     jam_cancel_ = true;
//                   } else {
//                     if (right_alc_car_cnt_.find(tr.track_id) !=
//                         right_alc_car_cnt_.end()) {
//                       right_alc_car_cnt_[tr.track_id].neg = 0;
//                     }
//                   }
//                 } else if (right_boundary_info.type_segments_size > 0 &&
//                            right_boundary_info.type_segments[0].type ==
//                                iflyauto::LaneBoundaryType_MARKING_SOLID) {
//                   right_lb_car_.clear();
//                   right_alc_car_.clear();
//                   r_accident_cnt_ = 0;
//                 }
//               } else {
//                 right_lb_car_.clear();
//                 right_alc_car_.clear();
//                 r_accident_cnt_ = 0;
//               }
//             } else {
//               r_accident_cnt_ = 0;

//               if (accident_ahead) {
//                 remove_car(right_lb_car_, tr.track_id);
//                 if (right_alc_car_.size() > 0 &&
//                     front_tracks_c_ids.find(right_alc_car_[0]) !=
//                         front_tracks_c_ids.end())
//                   remove_car(right_alc_car_, tr.track_id);
//               }
//             }
//             iflyauto::LaneType right_lane_type = iflyauto::LANETYPE_NORMAL;
//             if (rlane != nullptr) right_lane_type = rlane->get_lane_type();

//             if (tr.d_min_cpath != 100 &&
//                 tr.d_min_cpath >= -(lane_width / 2 + car_width / 5) &&
//                 tr.d_min_cpath < (car_width / 2 + lane_width * 0.05) &&
//                 tr.v_rel + v_ego < v_target && v_ego > 1) {
//               v_rel_r_ = get_vrel_close(1, status);
//               double v_right_front = v_rel_r_ + v_ego;
//               double v_front_lb = tr.v_rel + v_ego;

//               std::array<double, 4> xp_pos_r{0, 5, 10, 20};
//               std::array<double, 4> xp_pos_lb{0, 5, 10, 20};
//               std::array<double, 4> fp_pos_r{150 * coefficient,
//                                              80 * coefficient, 40 * coefficient,
//                                              10 * coefficient};
//               std::array<double, 4> fp_pos_lb{
//                   100 * coefficient, 50 * coefficient, 15 * coefficient,
//                   10 * coefficient};
//               std::array<double, 3> xp_neg{-22, -10, 0};
//               std::array<double, 3> fp_neg{20 * coefficient, 35 * coefficient,
//                                            80 * coefficient};

//               int temp =
//                   int(interp(tr.d_rel, lead_confidence_bp, lead_confidence_v));
//               int pos_thr_r = std::max(
//                   int(interp(v_target - v_front_lb, xp_pos_r, fp_pos_r)), temp);
//               int pos_thr_lb_r = std::max(
//                   int(interp(v_target - v_front_lb, xp_pos_lb, fp_pos_lb)),
//                   temp);

//               int neg_thr_r = int(interp(v_rel_r_, xp_neg, fp_neg));
//               int neg_thr_lb_r = neg_thr_r;

//               if (left_lb_car_cnt_.find(tr.track_id) ==
//                   left_lb_car_cnt_.end()) {
//                 left_lb_car_cnt_.insert(
//                     std::make_pair(tr.track_id, CarCount(0, 0)));
//               }

//               if (left_alc_car_cnt_.find(tr.track_id) ==
//                   left_alc_car_cnt_.end()) {
//                 left_alc_car_cnt_.insert(
//                     std::make_pair(tr.track_id, CarCount(0, 0)));
//               }

//               if (right_lb_car_cnt_.find(tr.track_id) ==
//                   right_lb_car_cnt_.end()) {
//                 right_lb_car_cnt_.insert(
//                     std::make_pair(tr.track_id, CarCount(0, 0)));
//               }

//               if (right_alc_car_cnt_.find(tr.track_id) ==
//                   right_alc_car_cnt_.end()) {
//                 right_alc_car_cnt_.insert(
//                     std::make_pair(tr.track_id, CarCount(0, 0)));
//               }

//               if (lead_one != nullptr &&
//                   right_lb_car_cnt_.find(lead_one->track_id) ==
//                       right_lb_car_cnt_.end()) {
//                 right_lb_car_cnt_.insert(
//                     std::make_pair(lead_one->track_id, CarCount(0, 0)));
//               }

//               if (temp_leadone != nullptr &&
//                   right_lb_car_cnt_.find(temp_leadone->track_id) ==
//                       right_lb_car_cnt_.end()) {
//                 right_lb_car_cnt_.insert(
//                     std::make_pair(temp_leadone->track_id, CarCount(0, 0)));
//               }

//               if (accident_ahead && v_rel_r_ >= 5 && tr.d_rel > -2 &&
//                   l_accident_cnt_ != 1) {
//               } else {
//                 r_accident_cnt_ = 0;

//                 if (!accident_ahead) {
//                   if (!RCHANGE) {
//                     right_lb_car_.clear();
//                     // right_alc_car_.clear();
//                   } else {
//                     for (auto &tr : front_tracks) {
//                       auto iter = std::find(right_alc_car_.begin(),
//                                             right_alc_car_.end(), tr.track_id);
//                       if (iter != right_alc_car_.end()) {
//                         if (tr.v_rel > 2.5) {
//                           right_lb_car_.clear();
//                           right_alc_car_.clear();
//                           right_alc_car_cnt_[tr.track_id].pos = 0;
//                           right_lb_car_cnt_[tr.track_id].pos = 0;
//                         }
//                         break;
//                       }
//                     }
//                   }
//                 }

//                 std::array<double, 3> xp{0, 30 / 3.6, 67 / 3.6};
//                 std::array<double, 3> fp{20 / 3.6, 18 / 3.6, 10 / 3.6};

//                 if (std::min(v_right_front, v_target) >
//                         v_front_lb + interp(v_front_lb, xp, fp) &&
//                     tr.d_rel < std::max(70., 70. - tr.v_lead * 2) &&
//                     v_right_front > std::min(v_ego + 3., tr.v_lead + 6.) &&
//                     (right_direct_exist ||
//                      (!right_direct_exist &&
//                       (!l_enable || !left_direct_exist) &&
//                       (dist_to_intsect - tr.d_rel > 60 ||
//                        tr.d_min_cpath > -lb_width_r)) ||
//                      dist_to_intsect < -5) &&
//                     (dist_to_intsect - tr.d_rel > 50 ||
//                      (std::fabs(tr.v_lead) < 1 && dist_to_intsect < -5))) {
//                   double d_stop = 0;
//                   if (right_boundary_info.type_segments_size > 0 &&
//                       right_boundary_info.type_segments[0].type ==
//                           iflyauto::LaneBoundaryType_MARKING_DASHED) {
//                     if (!is_on_highway) {
//                       if (lane_merge_split_point.existence == 0 ||
//                           (lane_merge_split_point.merge_split_point_data_size >
//                                0 &&
//                            lane_merge_split_point.merge_split_point_data[0]
//                                    .distance < 0 &&
//                            lane_merge_split_point.merge_split_point_data[0]
//                                        .distance +
//                                    lane_merge_split_point
//                                        .merge_split_point_data[0]
//                                        .length <
//                                0) ||
//                           (lane_merge_split_point.merge_split_point_data_size >
//                                0 &&
//                            lane_merge_split_point.merge_split_point_data[0]
//                                .is_split &&
//                            !lane_merge_split_point.merge_split_point_data[0]
//                                 .is_continue)) {
//                         if (right_boundary_info.type_segments_size > 0) {
//                           d_stop = right_boundary_info.type_segments[0].length;
//                         }

//                       } else if (right_boundary_info.type_segments_size > 0 &&
//                                  lane_merge_split_point
//                                          .merge_split_point_data_size > 0 &&
//                                  lane_merge_split_point
//                                          .merge_split_point_data[0]
//                                          .distance > 0) {
//                         d_stop = std::min(
//                             lane_merge_split_point.merge_split_point_data[0]
//                                 .distance,
//                             right_boundary_info.type_segments[0].length);
//                       } else {
//                         d_stop = -10000;
//                       }
//                     } else {
//                       if (right_boundary_info.type_segments_size > 0) {
//                         d_stop = std::min(
//                             (double)right_boundary_info.type_segments[0]
//                                         .length > kInputBoundaryLenLimit
//                                 ? kDefaultBoundaryLen
//                                 : right_boundary_info.type_segments[0].length,
//                             dis_to_ramp - 200.);
//                       }
//                       if (right_lane_tasks_id == -1) {
//                         d_stop = std::min(d_stop,
//                                           distance_to_first_road_split - 200.);
//                       }
//                       if (!is_on_ramp &&
//                           lane_merge_split_point.merge_split_point_data_size >
//                               0 &&
//                           !lane_merge_split_point.merge_split_point_data[0]
//                                .is_split &&
//                           lane_merge_split_point.merge_split_point_data[0]
//                               .is_continue) {
//                         d_stop = std::min(d_stop, (double)lane_merge_split_point
//                                                       .merge_split_point_data[0]
//                                                       .distance);
//                       }
//                     }
//                   } else if (dist_to_intsect < -5) {
//                     d_stop = dist_to_last_intsect;
//                   } else {
//                     d_stop = -10000;
//                   }

//                   if (!right_direct_exist && tr.d_min_cpath > -lb_width_r &&
//                       tr.v_lead < 1 && std::fabs(tr.v_lat) < 0.3) {
//                     pos_thr_r = 1000;
//                     pos_thr_lb_r = 10 * coefficient;
//                     d_stop += 10.0;
//                   }

//                   d_stop_r_ = d_stop;

//                   double d_lb_car = tr.d_rel + tr.length + safety_dist;
//                   d_lb_car_r_ = d_lb_car;

//                   double t = 0;
//                   double acc_t = 0.0;
//                   double acc_delta_x = 0.0;
//                   double v_aver = v_target;

//                   if (tr.v_rel != 0) {
//                     t = d_lb_car /
//                         std::max(
//                             ((std::min(v_right_front, v_target) + v_ego) / 2 -
//                              v_front_lb),
//                             0.1);
//                     v_aver = (std::min(v_right_front, v_target) + v_ego) / 2;
//                     std::array<double, 2> xp3{40 / 3.6, 80 / 3.6};
//                     std::array<double, 2> fp3{0.7, 0.4};
//                     double acc = interp(v_target, xp3, fp3);

//                     if (v_ego < std::min(v_right_front, v_target) - 4. &&
//                         v_right_front - v_ego > 5) {
//                       acc_t = (std::min(v_right_front, v_target) - v_ego) / acc;
//                       acc_delta_x = acc_t * acc_t / 2 * acc +
//                                     (v_ego - v_front_lb) * acc_t;
//                       if (acc_delta_x > d_lb_car) {
//                         t = std::sqrt(std::pow(v_ego - v_front_lb, 2) * 2.25 +
//                                       3 * d_lb_car) -
//                             (v_ego - v_front_lb) * 1.5;
//                         v_aver = v_ego + acc * t / 2;
//                       } else {
//                         t = acc_t +
//                             (d_lb_car - acc_delta_x) /
//                                 std::max((std::min(v_right_front, v_target) -
//                                           v_front_lb),
//                                          0.001);
//                         v_aver =
//                             (std::min(v_right_front, v_target) + v_ego) / 2;
//                       }
//                     }
//                   } else {
//                     if (v_ego >= 1) {
//                       t = d_lb_car / 2.0;
//                     } else {
//                       t = d_lb_car;
//                     }
//                     v_aver = v_ego;
//                   }

//                   t_surpass_r_ = t;

//                   double diff_dn_car = 0;
//                   double desired_gap = 0;
//                   double diff_thre = std::min(v_right_front, v_target) * 2;

//                   right_lb_car_cnt_[tr.track_id].neg = 0;
//                   right_alc_car_cnt_[tr.track_id].neg = 0;

//                   if (right_lane_tasks_id == -1 && current_lane_tasks_id == 0 &&
//                       current_lane_index < 2) {
//                     std::array<double, 4> xp{0, 50, 100, 200};
//                     std::array<double, 4> fp{40 * coefficient, 20 * coefficient,
//                                              -10 * coefficient,
//                                              -20 * coefficient};
//                     std::array<double, 4> xp_lat{-1.0, -0.5, 0.5, 1.5};
//                     std::array<double, 4> fp_lat{1.0, 0.7, 0.2, 0.1};
//                     pos_thr_r =
//                         std::max((int)(10 * coefficient),
//                                  pos_thr_r + (int)interp(d_stop, xp, fp));
//                     pos_thr_lb_r = std::max(
//                         (int)(10 * coefficient),
//                         (int)(pos_thr_lb_r *
//                               interp(tr.d_min_cpath - l_ego, xp_lat, fp_lat)) +
//                             (int)interp(d_stop, xp, fp));

//                     if (d_stop > d_lb_car + 20) {
//                       if ((d_stop > v_aver * (t + 5) && v_ego >= 1) ||
//                           (v_ego < 1 &&
//                            d_stop > std::max(v_ego, 0.1) * (t + 5))) {
//                         if (lead_one != nullptr) {
//                           if (tr.track_id == lead_one->track_id) {
//                             if (lead_two != nullptr &&
//                                 lead_two->v_lead > -0.5) {
//                               calc_desired_gap(v_ego, *lead_two, tr, t,
//                                                d_offset, t_gap, safety_dist,
//                                                diff_dn_car, desired_gap);

//                               if (diff_dn_car >
//                                   std::max(desired_gap, diff_thre)) {
//                                 if (tr.d_min_cpath > -lb_width_r) {
//                                   if (perception_range > lead_one->d_rel &&
//                                       lead_two->d_rel - perception_range >
//                                           std::max(desired_gap, diff_thre)) {
//                                     right_lb_car_cnt_[tr.track_id].pos += 3;
//                                   } else {
//                                     right_lb_car_cnt_[tr.track_id].pos += 1;
//                                   }
//                                 } else {
//                                   if (perception_range > lead_one->d_rel &&
//                                       lead_two->d_rel - perception_range >
//                                           std::max(desired_gap, diff_thre)) {
//                                     right_alc_car_cnt_[tr.track_id].pos += 2;
//                                   } else if (perception_range >
//                                                  lead_one->d_rel &&
//                                              lead_two->d_rel -
//                                                      perception_range <
//                                                  min_visible_gap &&
//                                              lead_two->v_rel + v_ego < 1.0) {
//                                     right_alc_car_cnt_[tr.track_id].pos += 0;
//                                   } else {
//                                     right_alc_car_cnt_[tr.track_id].pos += 1;
//                                   }
//                                   if (!premovel_ &&
//                                       right_alc_car_cnt_[tr.track_id].pos >=
//                                           pos_thr_r * 0.4) {
//                                     if (!premover_ &&
//                                         (tr.track_id == neg_premoved_id_)) {
//                                       right_alc_car_cnt_[tr.track_id].pos = 0;
//                                       neg_premoved_id_ = -1000;
//                                     } else if (premoved_id_ != tr.track_id &&
//                                                v_ego < 30 / 3.6) {
//                                       premoved_id_ = tr.track_id;
//                                       premover_ = true;
//                                       premove_dist_ =
//                                           std::min(-(flane->width() / 2 -
//                                                      car_width / 2 + press_thr),
//                                                    0.0);
//                                     }
//                                   }
//                                   if (premover_ &&
//                                       std::fabs(
//                                           premove_dist_ -
//                                           std::min(-(flane->width() / 2 -
//                                                      car_width / 2 + press_thr),
//                                                    0.0)) > 0.2) {
//                                     premove_dist_ =
//                                         std::min((flane->width() / 2 -
//                                                   car_width / 2 + press_thr),
//                                                  0.0);
//                                   }
//                                 }
//                               } else {
//                                 right_lb_car_cnt_[tr.track_id].pos = 0;
//                                 right_lb_car_cnt_[tr.track_id].neg =
//                                     neg_thr_lb_r + 1;

//                                 neg_right_lb_car_ = true;
//                                 lb_leadone_disable = true;
//                               }
//                             } else {
//                               if (tr.d_min_cpath > -lb_width_r) {
//                                 if (perception_range > lead_one->d_rel &&
//                                     perception_range <
//                                         lead_one->d_rel + max_visible_gap &&
//                                     lead_one->d_rel + max_visible_gap <
//                                         max_perception_range) {
//                                   right_lb_car_cnt_[tr.track_id].pos += 5;
//                                 } else {
//                                   right_lb_car_cnt_[tr.track_id].pos += 1;
//                                 }
//                               } else {
//                                 if (perception_range > lead_one->d_rel &&
//                                     perception_range <
//                                         lead_one->d_rel + max_visible_gap &&
//                                     lead_one->d_rel + max_visible_gap <
//                                         max_perception_range &&
//                                     (v_ego > 40 / 3.6 || !is_on_highway)) {
//                                   right_alc_car_cnt_[tr.track_id].pos += 3;
//                                 } else {
//                                   right_alc_car_cnt_[tr.track_id].pos += 1;
//                                 }
//                                 if (!premovel_ &&
//                                     right_alc_car_cnt_[tr.track_id].pos >=
//                                         pos_thr_r * 0.4) {
//                                   if (!premover_ &&
//                                       (tr.track_id == neg_premoved_id_)) {
//                                     right_alc_car_cnt_[tr.track_id].pos = 0;
//                                     neg_premoved_id_ = -1000;
//                                   } else if (premoved_id_ != tr.track_id &&
//                                              v_ego < 30 / 3.6) {
//                                     premoved_id_ = tr.track_id;
//                                     premover_ = true;
//                                     premove_dist_ =
//                                         std::min(-(flane->width() / 2 -
//                                                    car_width / 2 + press_thr),
//                                                  0.0);
//                                   }
//                                 }
//                                 if (premover_ &&
//                                     std::fabs(
//                                         premove_dist_ -
//                                         std::min(-(flane->width() / 2 -
//                                                    car_width / 2 + press_thr),
//                                                  0.0)) > 0.2) {
//                                   premove_dist_ =
//                                       std::min((flane->width() / 2 -
//                                                 car_width / 2 + press_thr),
//                                                0.0);
//                                 }
//                               }
//                             }
//                           } else {
//                             calc_desired_gap(v_ego, tr, *lead_one, t, d_offset,
//                                              t_gap, safety_dist, diff_dn_car,
//                                              desired_gap);

//                             if (std::fabs(tr.d_center_cpath -
//                                           lead_one->d_center_cpath) < 1.0 &&
//                                 tr.d_rel > lead_one->d_rel &&
//                                 diff_dn_car <
//                                     std::max(desired_gap, diff_thre)) {
//                               right_lb_car_cnt_[lead_one->track_id].pos = 0;
//                               right_lb_car_cnt_[lead_one->track_id].neg =
//                                   neg_thr_lb_r + 1;

//                               neg_right_lb_car_ = true;
//                               lb_leadone_disable = true;
//                             }

//                             calc_desired_gap(v_ego, *lead_one, tr, t, d_offset,
//                                              t_gap, safety_dist, diff_dn_car,
//                                              desired_gap);

//                             if (diff_dn_car > desired_gap) {
//                               if (tr.d_min_cpath > -lb_width_r) {
//                                 right_lb_car_cnt_[tr.track_id].pos += 1;
//                               } else {
//                                 right_alc_car_cnt_[tr.track_id].pos += 1;
//                               }
//                             }
//                           }
//                         } else {
//                           if (temp_leadone != nullptr) {
//                             if (tr.track_id == temp_leadone->track_id) {
//                               if (temp_leadtwo != nullptr) {
//                                 calc_desired_gap(v_ego, *temp_leadtwo, tr, t,
//                                                  d_offset, t_gap, safety_dist,
//                                                  diff_dn_car, desired_gap);

//                                 if (diff_dn_car <
//                                     std::max(desired_gap, diff_thre)) {
//                                   right_lb_car_cnt_[tr.track_id].pos = 0;
//                                   right_lb_car_cnt_[tr.track_id].neg =
//                                       neg_thr_lb_r + 1;

//                                   neg_right_lb_car_ = true;
//                                   lb_leadone_disable = true;
//                                 } else {
//                                   if (tr.d_min_cpath > -lb_width_r) {
//                                     right_lb_car_cnt_[tr.track_id].pos += 1;
//                                   } else {
//                                     right_alc_car_cnt_[tr.track_id].pos += 1;
//                                   }
//                                 }
//                               }
//                             } else {
//                               calc_desired_gap(v_ego, tr, *temp_leadone, t,
//                                                d_offset, t_gap, safety_dist,
//                                                diff_dn_car, desired_gap);

//                               if (std::fabs(tr.d_center_cpath -
//                                             temp_leadone->d_center_cpath) <
//                                       1.0 &&
//                                   tr.d_rel > temp_leadone->d_rel &&
//                                   diff_dn_car <
//                                       std::max(desired_gap, diff_thre)) {
//                                 right_lb_car_cnt_[temp_leadone->track_id].pos =
//                                     0;
//                                 right_lb_car_cnt_[temp_leadone->track_id].neg =
//                                     neg_thr_lb_r + 1;

//                                 neg_right_lb_car_ = true;
//                                 lb_leadone_disable = true;
//                               }
//                             }
//                           } else {
//                             if (tr.d_min_cpath > -lb_width_r) {
//                               right_lb_car_cnt_[tr.track_id].pos += 1;
//                             } else {
//                               right_alc_car_cnt_[tr.track_id].pos += 1;
//                             }
//                           }
//                         }
//                       } else if ((d_stop > v_aver * (t + 2) && v_ego >= 1) ||
//                                  (v_ego < 1 &&
//                                   d_stop > std::max(v_ego, 0.1) * (t + 2))) {
//                         right_alc_car_cnt_[tr.track_id].pos = 0;
//                         remove_car(right_alc_car_, tr.track_id);

//                         if (lead_one != nullptr) {
//                           if (tr.track_id == lead_one->track_id) {
//                             if (lead_two != nullptr) {
//                               calc_desired_gap(v_ego, *lead_two, tr, t,
//                                                d_offset, t_gap, safety_dist,
//                                                diff_dn_car, desired_gap);

//                               if (diff_dn_car >
//                                       std::max(desired_gap, diff_thre) &&
//                                   tr.d_min_cpath > -lb_width_r) {
//                                 if (perception_range > lead_one->d_rel &&
//                                     lead_two->d_rel - perception_range >
//                                         std::max(desired_gap, diff_thre)) {
//                                   right_lb_car_cnt_[tr.track_id].pos += 3;
//                                 } else {
//                                   right_lb_car_cnt_[tr.track_id].pos += 1;
//                                 }
//                               } else {
//                                 right_lb_car_cnt_[tr.track_id].pos = 0;
//                                 right_lb_car_cnt_[tr.track_id].neg =
//                                     neg_thr_lb_r + 1;

//                                 neg_right_lb_car_ = true;
//                                 lb_leadone_disable = true;
//                               }
//                             } else if (tr.d_min_cpath > -lb_width_r) {
//                               if (perception_range > lead_one->d_rel &&
//                                   perception_range <
//                                       lead_one->d_rel + max_visible_gap &&
//                                   lead_one->d_rel + max_visible_gap <
//                                       max_perception_range) {
//                                 right_lb_car_cnt_[tr.track_id].pos += 5;
//                               } else {
//                                 right_lb_car_cnt_[tr.track_id].pos += 1;
//                               }
//                             }
//                           } else {
//                             calc_desired_gap(v_ego, tr, *lead_one, t, d_offset,
//                                              t_gap, safety_dist, diff_dn_car,
//                                              desired_gap);

//                             if (std::fabs(tr.d_center_cpath -
//                                           lead_one->d_center_cpath) < 1.0 &&
//                                 tr.d_rel > lead_one->d_rel &&
//                                 diff_dn_car <
//                                     std::max(desired_gap, diff_thre)) {
//                               right_lb_car_cnt_[lead_one->track_id].pos = 0;
//                               right_lb_car_cnt_[lead_one->track_id].neg =
//                                   neg_thr_lb_r + 1;

//                               neg_right_lb_car_ = true;
//                               lb_leadone_disable = true;
//                             }

//                             calc_desired_gap(v_ego, *lead_one, tr, t, d_offset,
//                                              t_gap, safety_dist, diff_dn_car,
//                                              desired_gap);

//                             if (diff_dn_car > desired_gap &&
//                                 tr.d_min_cpath > -lb_width_r) {
//                               right_lb_car_cnt_[tr.track_id].pos += 1;
//                             }
//                           }
//                         } else {
//                           if (temp_leadone != nullptr) {
//                             if (tr.track_id == temp_leadone->track_id) {
//                               if (temp_leadtwo != nullptr) {
//                                 calc_desired_gap(v_ego, *temp_leadtwo, tr, t,
//                                                  d_offset, t_gap, safety_dist,
//                                                  diff_dn_car, desired_gap);

//                                 if (diff_dn_car <
//                                     std::max(desired_gap, diff_thre)) {
//                                   right_lb_car_cnt_[tr.track_id].pos = 0;
//                                   right_lb_car_cnt_[tr.track_id].neg =
//                                       neg_thr_lb_r + 1;

//                                   neg_right_lb_car_ = true;
//                                   lb_leadone_disable = true;
//                                 } else if (tr.d_min_cpath > -lb_width_r) {
//                                   right_lb_car_cnt_[tr.track_id].pos += 1;
//                                 }
//                               }
//                             } else {
//                               calc_desired_gap(v_ego, tr, *temp_leadone, t,
//                                                d_offset, t_gap, safety_dist,
//                                                diff_dn_car, desired_gap);

//                               if (std::fabs(tr.d_center_cpath -
//                                             temp_leadone->d_center_cpath) <
//                                       1.0 &&
//                                   tr.d_rel > temp_leadone->d_rel &&
//                                   diff_dn_car <
//                                       std::max(desired_gap, diff_thre)) {
//                                 right_lb_car_cnt_[temp_leadone->track_id].pos =
//                                     0;
//                                 right_lb_car_cnt_[temp_leadone->track_id].neg =
//                                     neg_thr_lb_r + 1;

//                                 neg_right_lb_car_ = true;
//                                 lb_leadone_disable = true;
//                               }
//                             }
//                           } else if (tr.d_min_cpath > -lb_width_r) {
//                             right_lb_car_cnt_[tr.track_id].pos += 1;
//                           }
//                         }
//                       } else if (d_stop > v_ego * 8 && right_direct_exist &&
//                                  (current_lane_index == 0 ||
//                                   !left_direct_exist) &&
//                                  !isRedLightStop) {
//                         right_alc_car_cnt_[tr.track_id].pos += 1;
//                         right_lb_car_cnt_[tr.track_id].pos = 0;

//                         remove_car(right_lb_car_, tr.track_id);
//                       } else {
//                         right_alc_car_cnt_[tr.track_id].pos = 0;
//                         right_lb_car_cnt_[tr.track_id].pos = 0;

//                         remove_car(right_alc_car_, tr.track_id);
//                         remove_car(right_lb_car_, tr.track_id);
//                       }
//                     } else {
//                       if (d_stop > v_aver * 8 && right_direct_exist) {
//                         if ((!isRedLightStop &&
//                              (right_laneout_direct_exist ||
//                               (dist_to_intsect + intersect_length +
//                                    rout_l_dash_length >
//                                d_lb_car + 60)) &&
//                              !is_on_highway) ||
//                             (std::min(dis_to_ramp, lc_end_dis) >
//                                  d_lb_car + std::max(v_target * 4, 60.0) &&
//                              is_on_highway)) {
//                           right_alc_car_cnt_[tr.track_id].pos += 1;
//                           right_lb_car_cnt_[tr.track_id].pos = 0;

//                           remove_car(right_lb_car_, tr.track_id);
//                         } else if (status != RCHANGE) {
//                           right_alc_car_cnt_[tr.track_id].pos = 0;
//                           right_lb_car_cnt_[tr.track_id].pos = 0;

//                           remove_car(right_alc_car_, tr.track_id);
//                           remove_car(right_lb_car_, tr.track_id);
//                         }
//                       }
//                     }
//                   } else if ((right_lane_tasks_id == 0 &&
//                               current_lane_tasks_id == 0 &&
//                               current_lane_index < 2 &&
//                               right_lane_type == iflyauto::LANETYPE_NORMAL) ||
//                              (is_on_highway && right_lane_tasks_id >= 1 &&
//                               current_lane_tasks_id >= 2)) {
//                     std::array<double, 4> xp_pos_r{4, 7, 10, 20};
//                     std::array<double, 4> xp_pos_lb{4, 7, 10, 20};
//                     std::array<double, 4> fp_pos_r{1.0, 0.6, 0.3, 0.1};
//                     std::array<double, 4> fp_pos_lb{1.0, 0.5, 0.2, 0.1};
//                     std::array<double, 4> xp_pos{0, 50, 100, 200};
//                     std::array<double, 4> fp_pos{20 * coefficient, 0,
//                                                  -10 * coefficient,
//                                                  -20 * coefficient};
//                     std::array<double, 4> xp_lat{-1.0, -0.5, 0.5, 1.5};
//                     std::array<double, 4> fp_lat{1.0, 0.7, 0.2, 0.1};

//                     pos_thr_r = std::max(
//                         (int)(10 * coefficient),
//                         (int)(pos_thr_r * interp(v_right_front - v_front_lb,
//                                                  xp_pos_r, fp_pos_r)));

//                     pos_thr_lb_r = std::max(
//                         (int)(10 * coefficient),
//                         (int)(pos_thr_lb_r *
//                               interp(v_right_front - v_front_lb, xp_pos_lb,
//                                      fp_pos_lb) *
//                               interp(tr.d_min_cpath - l_ego, xp_lat, fp_lat)));

//                     if (!is_on_highway) {
//                       pos_thr_r = std::max(
//                           (int)(10 * coefficient),
//                           pos_thr_r + (int)interp(d_stop, xp_pos, fp_pos));

//                       pos_thr_lb_r = std::max(
//                           (int)(10 * coefficient),
//                           pos_thr_lb_r + (int)interp(d_stop, xp_pos, fp_pos));
//                     }

//                     if ((d_stop > v_aver * (t + 2) && v_ego >= 1) ||
//                         (v_ego < 1 &&
//                          d_stop > std::max(v_ego, 0.1) * (t + 2))) {
//                       right_alc_car_cnt_[tr.track_id].pos += 1;

//                       if (lead_one != nullptr) {
//                         if (tr.track_id == lead_one->track_id) {
//                           if (lead_two != nullptr && lead_two->v_lead > -0.5) {
//                             calc_desired_gap(v_ego, *lead_two, tr, t, d_offset,
//                                              t_gap, safety_dist, diff_dn_car,
//                                              desired_gap);

//                             if (diff_dn_car >
//                                 std::max(desired_gap, diff_thre)) {
//                               if (tr.d_min_cpath > -lb_width_r) {
//                                 if (perception_range > lead_one->d_rel &&
//                                     lead_two->d_rel - perception_range >
//                                         std::max(desired_gap, diff_thre)) {
//                                   right_lb_car_cnt_[tr.track_id].pos += 3;
//                                 } else {
//                                   right_lb_car_cnt_[tr.track_id].pos += 1;
//                                 }
//                               }
//                             } else {
//                               right_lb_car_cnt_[tr.track_id].pos = 0;
//                               right_lb_car_cnt_[tr.track_id].neg =
//                                   neg_thr_lb_r + 1;

//                               neg_right_lb_car_ = true;
//                               lb_leadone_disable = true;
//                             }
//                             if (perception_range > lead_one->d_rel &&
//                                 lead_two->d_rel - perception_range >
//                                     std::max(desired_gap, diff_thre)) {
//                               right_alc_car_cnt_[tr.track_id].pos += 2;
//                             }
//                           } else {
//                             if (tr.d_min_cpath > -lb_width_r) {
//                               if (perception_range > lead_one->d_rel &&
//                                   perception_range <
//                                       lead_one->d_rel + max_visible_gap &&
//                                   lead_one->d_rel + max_visible_gap <
//                                       max_perception_range) {
//                                 right_lb_car_cnt_[tr.track_id].pos += 5;
//                               } else {
//                                 right_lb_car_cnt_[tr.track_id].pos += 1;
//                               }
//                             }
//                             if (perception_range > lead_one->d_rel &&
//                                 perception_range <
//                                     lead_one->d_rel + max_visible_gap &&
//                                 lead_one->d_rel + max_visible_gap <
//                                     max_perception_range &&
//                                 (v_ego > 40 / 3.6 || !is_on_highway)) {
//                               right_alc_car_cnt_[tr.track_id].pos += 3;
//                             }
//                           }
//                         } else {
//                           calc_desired_gap(v_ego, tr, *lead_one, t, d_offset,
//                                            t_gap, safety_dist, diff_dn_car,
//                                            desired_gap);

//                           if (std::fabs(tr.d_center_cpath -
//                                         lead_one->d_center_cpath) < 1.0 &&
//                               tr.d_rel > lead_one->d_rel &&
//                               diff_dn_car < std::max(desired_gap, diff_thre)) {
//                             right_lb_car_cnt_[lead_one->track_id].pos = 0;
//                             right_lb_car_cnt_[lead_one->track_id].neg =
//                                 neg_thr_lb_r + 1;

//                             neg_right_lb_car_ = true;
//                             lb_leadone_disable = true;
//                           }

//                           calc_desired_gap(v_ego, *lead_one, tr, t, d_offset,
//                                            t_gap, safety_dist, diff_dn_car,
//                                            desired_gap);

//                           if (diff_dn_car > desired_gap &&
//                               tr.d_min_cpath > -lb_width_r) {
//                             right_lb_car_cnt_[tr.track_id].pos += 1;
//                           }
//                         }
//                       } else {
//                         if (temp_leadone != nullptr) {
//                           if (tr.track_id == temp_leadone->track_id) {
//                             if (temp_leadtwo != nullptr) {
//                               calc_desired_gap(v_ego, *temp_leadtwo, tr, t,
//                                                d_offset, t_gap, safety_dist,
//                                                diff_dn_car, desired_gap);

//                               if (diff_dn_car <
//                                   std::max(desired_gap, diff_thre)) {
//                                 right_lb_car_cnt_[tr.track_id].pos = 0;
//                                 right_lb_car_cnt_[tr.track_id].neg =
//                                     neg_thr_lb_r + 1;

//                                 neg_right_lb_car_ = true;
//                                 lb_leadone_disable = true;
//                               } else {
//                                 if (tr.d_min_cpath > -lb_width_r) {
//                                   right_lb_car_cnt_[tr.track_id].pos += 1;
//                                 }
//                               }
//                             }
//                           } else {
//                             calc_desired_gap(v_ego, tr, *temp_leadone, t,
//                                              d_offset, t_gap, safety_dist,
//                                              diff_dn_car, desired_gap);

//                             if (std::fabs(tr.d_center_cpath -
//                                           temp_leadone->d_center_cpath) < 1.0 &&
//                                 tr.d_rel > temp_leadone->d_rel &&
//                                 diff_dn_car <
//                                     std::max(desired_gap, diff_thre)) {
//                               right_lb_car_cnt_[temp_leadone->track_id].pos = 0;
//                               right_lb_car_cnt_[temp_leadone->track_id].neg =
//                                   neg_thr_lb_r + 1;

//                               neg_right_lb_car_ = true;
//                               lb_leadone_disable = true;
//                             }
//                           }
//                         } else if (tr.d_min_cpath > -lb_width_r) {
//                           right_lb_car_cnt_[tr.track_id].pos += 1;
//                         }
//                       }
//                     } else if (d_stop > v_ego * 2) {
//                       right_alc_car_cnt_[tr.track_id].pos += 1;
//                       right_lb_car_cnt_[tr.track_id].pos = 0;

//                       remove_car(right_lb_car_, tr.track_id);
//                     } else {
//                       right_alc_car_cnt_[tr.track_id].pos = 0;
//                       right_lb_car_cnt_[tr.track_id].pos = 0;

//                       remove_car(right_alc_car_, tr.track_id);
//                       remove_car(right_lb_car_, tr.track_id);
//                     }
//                   } else if ((is_on_highway && !is_on_ramp &&
//                               right_lane_tasks_id == 0 &&
//                               current_lane_tasks_id == 1 &&
//                               right_lane_type == iflyauto::LANETYPE_NORMAL)) {
//                     std::array<double, 4> xp{0, 50, 100, 200};
//                     std::array<double, 4> fp{40 * coefficient, 20 * coefficient,
//                                              0, -20 * coefficient};
//                     std::array<double, 4> xp_lat{-1.0, -0.5, 0.5, 1.5};
//                     std::array<double, 4> fp_lat{1.0, 0.7, 0.2, 0.1};

//                     if (!is_on_highway) {
//                       pos_thr_r =
//                           std::max((int)(10 * coefficient),
//                                    pos_thr_r + (int)interp(d_stop, xp, fp));
//                       pos_thr_lb_r = std::max(
//                           (int)(10 * coefficient),
//                           (int)(pos_thr_lb_r * interp(tr.d_min_cpath - l_ego,
//                                                       xp_lat, fp_lat)) +
//                               (int)interp(d_stop, xp, fp));
//                     }

//                     double d_stop_l = lc_end_dis;
//                     double d_map = d_stop;

//                     if (d_stop > d_lb_car + 30) {
//                       if ((d_stop_l > v_aver * (t + 10) &&
//                            d_map > v_aver * (t + 5) &&
//                            d_stop > v_aver * (t + 5) && v_ego >= 1) ||
//                           (v_ego < 1 &&
//                            d_stop_l > std::max(v_ego, 0.1) * (t + 15) &&
//                            d_map > std::max(v_ego, 0.1) * (t + 5) &&
//                            d_stop > std::max(v_ego, 0.1) * (t + 5))) {
//                         if (lead_one != nullptr) {
//                           if (tr.track_id == lead_one->track_id) {
//                             if (lead_two != nullptr &&
//                                 lead_two->v_lead > -0.5) {
//                               calc_desired_gap(v_ego, *lead_two, tr, t,
//                                                d_offset, t_gap, safety_dist,
//                                                diff_dn_car, desired_gap);

//                               if (diff_dn_car >
//                                   std::max(desired_gap, diff_thre)) {
//                                 if (tr.d_min_cpath > -lb_width_r) {
//                                   if (perception_range > lead_one->d_rel &&
//                                       lead_two->d_rel - perception_range >
//                                           std::max(desired_gap, diff_thre)) {
//                                     right_lb_car_cnt_[tr.track_id].pos += 3;
//                                   } else {
//                                     right_lb_car_cnt_[tr.track_id].pos += 1;
//                                   }
//                                 } else {
//                                   if (perception_range > lead_one->d_rel &&
//                                       lead_two->d_rel - perception_range >
//                                           std::max(desired_gap, diff_thre)) {
//                                     right_alc_car_cnt_[tr.track_id].pos += 2;
//                                   } else if (perception_range >
//                                                  lead_one->d_rel &&
//                                              lead_two->d_rel -
//                                                      perception_range <
//                                                  min_visible_gap &&
//                                              lead_two->v_rel + v_ego < 1.0) {
//                                     right_alc_car_cnt_[tr.track_id].pos += 0;
//                                   } else {
//                                     right_alc_car_cnt_[tr.track_id].pos += 1;
//                                   }
//                                 }
//                               } else {
//                                 right_lb_car_cnt_[tr.track_id].pos = 0;
//                                 right_lb_car_cnt_[tr.track_id].neg =
//                                     neg_thr_lb_r + 1;

//                                 neg_right_lb_car_ = true;
//                                 lb_leadone_disable = true;
//                               }
//                             } else {
//                               if (tr.d_min_cpath > -lb_width_r) {
//                                 if (perception_range > lead_one->d_rel &&
//                                     perception_range <
//                                         lead_one->d_rel + max_visible_gap &&
//                                     lead_one->d_rel + max_visible_gap <
//                                         max_perception_range) {
//                                   right_lb_car_cnt_[tr.track_id].pos += 5;
//                                 } else {
//                                   right_lb_car_cnt_[tr.track_id].pos += 1;
//                                 }
//                               } else {
//                                 if (perception_range > lead_one->d_rel &&
//                                     perception_range <
//                                         lead_one->d_rel + max_visible_gap &&
//                                     lead_one->d_rel + max_visible_gap <
//                                         max_perception_range &&
//                                     (v_ego > 40 / 3.6 || !is_on_highway)) {
//                                   right_alc_car_cnt_[tr.track_id].pos += 3;
//                                 } else {
//                                   right_alc_car_cnt_[tr.track_id].pos += 1;
//                                 }
//                               }
//                             }
//                           } else {
//                             calc_desired_gap(v_ego, tr, *lead_one, t, d_offset,
//                                              t_gap, safety_dist, diff_dn_car,
//                                              desired_gap);

//                             if (std::fabs(tr.d_center_cpath -
//                                           lead_one->d_center_cpath) < 1.0 &&
//                                 tr.d_rel > lead_one->d_rel &&
//                                 diff_dn_car <
//                                     std::max(desired_gap, diff_thre)) {
//                               right_lb_car_cnt_[lead_one->track_id].pos = 0;
//                               right_lb_car_cnt_[lead_one->track_id].neg =
//                                   neg_thr_lb_r + 1;

//                               neg_right_lb_car_ = true;
//                               lb_leadone_disable = true;
//                             }

//                             calc_desired_gap(v_ego, *lead_one, tr, t, d_offset,
//                                              t_gap, safety_dist, diff_dn_car,
//                                              desired_gap);

//                             if (diff_dn_car > desired_gap) {
//                               if (tr.d_min_cpath > -lb_width_r) {
//                                 right_lb_car_cnt_[tr.track_id].pos += 1;
//                               } else {
//                                 right_alc_car_cnt_[tr.track_id].pos += 1;
//                               }
//                             }
//                           }
//                         } else {
//                           if (temp_leadone != nullptr) {
//                             if (tr.track_id == temp_leadone->track_id) {
//                               if (temp_leadtwo != nullptr) {
//                                 calc_desired_gap(v_ego, *temp_leadtwo, tr, t,
//                                                  d_offset, t_gap, safety_dist,
//                                                  diff_dn_car, desired_gap);

//                                 if (diff_dn_car <
//                                     std::max(desired_gap, diff_thre)) {
//                                   right_lb_car_cnt_[tr.track_id].pos = 0;
//                                   right_lb_car_cnt_[tr.track_id].neg =
//                                       neg_thr_lb_r + 1;

//                                   neg_right_lb_car_ = true;
//                                   lb_leadone_disable = true;
//                                 } else {
//                                   if (tr.d_min_cpath > -lb_width_r) {
//                                     right_lb_car_cnt_[tr.track_id].pos += 1;
//                                   } else {
//                                     right_alc_car_cnt_[tr.track_id].pos += 1;
//                                   }
//                                 }
//                               }
//                             } else {
//                               calc_desired_gap(v_ego, tr, *temp_leadone, t,
//                                                d_offset, t_gap, safety_dist,
//                                                diff_dn_car, desired_gap);

//                               if (std::fabs(tr.d_center_cpath -
//                                             temp_leadone->d_center_cpath) <
//                                       1.0 &&
//                                   tr.d_rel > temp_leadone->d_rel &&
//                                   diff_dn_car <
//                                       std::max(desired_gap, diff_thre)) {
//                                 right_lb_car_cnt_[temp_leadone->track_id].pos =
//                                     0;
//                                 right_lb_car_cnt_[temp_leadone->track_id].neg =
//                                     neg_thr_lb_r + 1;

//                                 neg_right_lb_car_ = true;
//                                 lb_leadone_disable = true;
//                               }
//                             }
//                           } else {
//                             if (tr.d_min_cpath > -lb_width_r) {
//                               right_lb_car_cnt_[tr.track_id].pos += 1;
//                             } else {
//                               right_alc_car_cnt_[tr.track_id].pos += 1;
//                             }
//                           }
//                         }
//                       } else if ((d_stop_l > v_aver * (t + 8) &&
//                                   d_map > v_aver * (t + 2) &&
//                                   d_stop > v_aver * (t + 2) && v_ego >= 1) ||
//                                  (v_ego < 1 &&
//                                   d_stop_l > std::max(v_ego, 0.1) * (t + 10) &&
//                                   d_map > std::max(v_ego, 0.1) * (t + 2) &&
//                                   d_stop > std::max(v_ego, 0.1) * (t + 2))) {
//                         right_alc_car_cnt_[tr.track_id].pos = 0;
//                         remove_car(right_alc_car_, tr.track_id);

//                         if (lead_one != nullptr) {
//                           if (tr.track_id == lead_one->track_id) {
//                             if (lead_two != nullptr) {
//                               calc_desired_gap(v_ego, *lead_two, tr, t,
//                                                d_offset, t_gap, safety_dist,
//                                                diff_dn_car, desired_gap);

//                               if (diff_dn_car >
//                                       std::max(desired_gap, diff_thre) &&
//                                   tr.d_min_cpath > -lb_width_r) {
//                                 if (perception_range > lead_one->d_rel &&
//                                     lead_two->d_rel - perception_range >
//                                         std::max(desired_gap, diff_thre)) {
//                                   right_lb_car_cnt_[tr.track_id].pos += 3;
//                                 } else {
//                                   right_lb_car_cnt_[tr.track_id].pos += 1;
//                                 }
//                               } else {
//                                 right_lb_car_cnt_[tr.track_id].pos = 0;
//                                 right_lb_car_cnt_[tr.track_id].neg =
//                                     neg_thr_lb_r + 1;

//                                 neg_right_lb_car_ = true;
//                                 lb_leadone_disable = true;
//                               }
//                             } else if (tr.d_min_cpath > -lb_width_r) {
//                               if (perception_range > lead_one->d_rel &&
//                                   perception_range <
//                                       lead_one->d_rel + max_visible_gap) {
//                                 right_lb_car_cnt_[tr.track_id].pos += 5;
//                               } else {
//                                 right_lb_car_cnt_[tr.track_id].pos += 1;
//                               }
//                             }
//                           } else {
//                             calc_desired_gap(v_ego, tr, *lead_one, t, d_offset,
//                                              t_gap, safety_dist, diff_dn_car,
//                                              desired_gap);

//                             if (std::fabs(tr.d_center_cpath -
//                                           lead_one->d_center_cpath) < 1.0 &&
//                                 tr.d_rel > lead_one->d_rel &&
//                                 diff_dn_car <
//                                     std::max(desired_gap, diff_thre)) {
//                               right_lb_car_cnt_[lead_one->track_id].pos = 0;
//                               right_lb_car_cnt_[lead_one->track_id].neg =
//                                   neg_thr_lb_r + 1;

//                               neg_right_lb_car_ = true;
//                               lb_leadone_disable = true;
//                             }

//                             calc_desired_gap(v_ego, *lead_one, tr, t, d_offset,
//                                              t_gap, safety_dist, diff_dn_car,
//                                              desired_gap);

//                             if (diff_dn_car > desired_gap &&
//                                 tr.d_min_cpath > -lb_width_r) {
//                               right_lb_car_cnt_[tr.track_id].pos += 1;
//                             }
//                           }
//                         } else {
//                           if (temp_leadone != nullptr) {
//                             if (tr.track_id == temp_leadone->track_id) {
//                               if (temp_leadtwo != nullptr) {
//                                 calc_desired_gap(v_ego, *temp_leadtwo, tr, t,
//                                                  d_offset, t_gap, safety_dist,
//                                                  diff_dn_car, desired_gap);

//                                 if (diff_dn_car <
//                                     std::max(desired_gap, diff_thre)) {
//                                   right_lb_car_cnt_[tr.track_id].pos = 0;
//                                   right_lb_car_cnt_[tr.track_id].neg =
//                                       neg_thr_lb_r + 1;

//                                   neg_right_lb_car_ = true;
//                                   lb_leadone_disable = true;
//                                 } else if (tr.d_min_cpath > -lb_width_r) {
//                                   right_lb_car_cnt_[tr.track_id].pos += 1;
//                                 }
//                               }
//                             } else {
//                               calc_desired_gap(v_ego, tr, *temp_leadone, t,
//                                                d_offset, t_gap, safety_dist,
//                                                diff_dn_car, desired_gap);

//                               if (std::fabs(tr.d_center_cpath -
//                                             temp_leadone->d_center_cpath) <
//                                       1.0 &&
//                                   tr.d_rel > temp_leadone->d_rel &&
//                                   diff_dn_car <
//                                       std::max(desired_gap, diff_thre)) {
//                                 right_lb_car_cnt_[temp_leadone->track_id].pos =
//                                     0;
//                                 right_lb_car_cnt_[temp_leadone->track_id].neg =
//                                     neg_thr_lb_r + 1;

//                                 neg_right_lb_car_ = true;
//                                 lb_leadone_disable = true;
//                               }
//                             }
//                           } else if (tr.d_min_cpath > -lb_width_r) {
//                             right_lb_car_cnt_[tr.track_id].pos += 1;
//                           }
//                         }
//                       } else {
//                         right_alc_car_cnt_[tr.track_id].pos = 0;
//                         right_lb_car_cnt_[tr.track_id].pos = 0;

//                         remove_car(right_alc_car_, tr.track_id);
//                         remove_car(right_lb_car_, tr.track_id);
//                       }
//                     }
//                   } else if ((right_lane_tasks_id == 0 &&
//                               current_lane_tasks_id == 1) ||
//                              (right_lane_tasks_id == 1 &&
//                               current_lane_tasks_id == 2)) {
//                   }

//                   auto iter = std::find(right_alc_car_.begin(),
//                                         right_alc_car_.end(), tr.track_id);

//                   if (right_alc_car_cnt_[tr.track_id].pos >
//                           pos_thr_r +
//                               active_lane_change_min_duration_threshold &&
//                       iter == right_alc_car_.end() &&
//                       right_lb_car_cnt_[tr.track_id].pos < pos_thr_lb_r) {
//                     right_alc_car_.push_back(tr.track_id);
//                     right_alc_car_cnt_[tr.track_id].neg = 0;
//                     neg_right_alc_car_ = false;
//                   }

//                   iter = std::find(right_lb_car_.begin(), right_lb_car_.end(),
//                                    tr.track_id);

//                   if (right_lb_car_cnt_[tr.track_id].pos > pos_thr_lb_r &&
//                       iter == right_lb_car_.end()) {
//                     right_lb_car_.push_back(tr.track_id);
//                     right_lb_car_cnt_[tr.track_id].neg = 0;
//                     neg_right_lb_car_ = false;
//                   }
//                 }

//                 auto iter = std::find(right_alc_car_.begin(),
//                                       right_alc_car_.end(), tr.track_id);

//                 // if (is_on_highway && !is_on_ramp &&
//                 //    ((map_info.lanes_merge_type() ==
//                 //    MSD_MERGE_TYPE_MERGE_FROM_RIGHT &&
//                 //     map_info.distance_to_lanes_merge() < 500))) {
//                 //       right_lb_car_.clear();
//                 //       right_alc_car_.clear();
//                 //       right_lb_car_cnt_.clear();
//                 //       right_alc_car_cnt_.clear();
//                 // }

//                 if (right_alc_car_cnt_[tr.track_id].pos >
//                         pos_thr_r + active_lane_change_min_duration_threshold &&
//                     iter == right_alc_car_.end() &&
//                     right_lb_car_cnt_[tr.track_id].pos < pos_thr_lb_r) {
//                   right_alc_car_.push_back(tr.track_id);
//                   neg_right_alc_car_ = false;
//                 }

//                 iter = std::find(right_lb_car_.begin(), right_lb_car_.end(),
//                                  tr.track_id);

//                 if (right_lb_car_cnt_[tr.track_id].pos > pos_thr_lb_r &&
//                     iter == right_lb_car_.end()) {
//                   right_lb_car_.push_back(tr.track_id);
//                   neg_right_lb_car_ = false;
//                 }

//                 LOG_DEBUG("WR: pos_thr_r[%d] pos_thr_lb_r[%d]", pos_thr_r,
//                           pos_thr_lb_r);

//                 double neg_d_stop = 0.0;
//                 if (right_boundary_info.type_segments_size > 0 &&
//                     right_boundary_info.type_segments[0].type ==
//                         iflyauto::LaneBoundaryType_MARKING_DASHED) {
//                   neg_d_stop = right_boundary_info.type_segments[0].length;
//                 } else if (dist_to_intsect < -5) {
//                   neg_d_stop = dist_to_last_intsect;
//                 } else {
//                   neg_d_stop = -10000;
//                 }

//                 std::array<double, 2> xp1{0, 67 / 3.6};
//                 std::array<double, 2> fp1{12 / 3.6, 8 / 3.6};
//                 // std::array<double, 3> xp2{0, 40, 70};
//                 // std::array<double, 3> fp2{0.5, 1.5, 0.5};
//                 double temp = interp(v_front_lb, xp1, fp1);
//                 std::array<double, 4> xp3{0., 3., 5., 10.};
//                 std::array<double, 4> fp3{-2., 0.6, 2., 4.};
//                 // double temp_attenuation = interp(neg_d_stop, xp2, fp2);
//                 double temp_attenuation =
//                     interp(std::max(v_target - v_front_lb, 0.), xp3, fp3);

//                 if (std::min(v_right_front, v_target) < v_front_lb + temp ||
//                             v_right_front <
//                                 std::min(v_ego + 2, tr.v_lead + 3) ||
//                             tr.v_rel > temp_attenuation ||
//                             use_lateral_distance_to_judge_cutout_in_active_lane_change
//                         ? tr.l < -half_car_width
//                         : std::fabs(tr.v_lat) > 0.3 ||
//                               (((!LCHANGE && !RCHANGE) && lead_one != nullptr &&
//                                 lead_two != nullptr &&
//                                 lead_two->v_lead > -0.5 &&
//                                 lead_two->d_rel - lead_one->d_rel <
//                                     std::min(v_right_front, v_target) * 2 &&
//                                 (lead_two->v_rel < 5. &&
//                                  std::fabs(lead_two->v_lat) < 0.3 &&
//                                  lead_two->d_min_cpath < 1.1 &&
//                                  lead_two->d_min_cpath + lead_two->width >
//                                      1.1)) ||
//                                (temp_leadone != nullptr &&
//                                 temp_leadtwo != nullptr &&
//                                 temp_leadtwo->v_lead > -0.5 &&
//                                 temp_leadtwo->d_rel - temp_leadone->d_rel <
//                                     v_target * 2 &&
//                                 (temp_leadtwo->v_rel < 5. ||
//                                  std::fabs(temp_leadtwo->v_lat) < 0.2)))) {
//                   right_lb_car_cnt_[tr.track_id].neg += 1;
//                   right_alc_car_cnt_[tr.track_id].neg += 1;

//                   int lb_pos = right_lb_car_cnt_[tr.track_id].pos;
//                   int alc_pos = right_alc_car_cnt_[tr.track_id].pos;

//                   right_lb_car_cnt_[tr.track_id].pos = std::max(lb_pos - 3, 0);
//                   right_alc_car_cnt_[tr.track_id].pos =
//                       std::max(alc_pos - 3, 0);

//                   if (premover_ &&
//                       right_alc_car_cnt_[tr.track_id].pos < pos_thr_r * 0.1) {
//                     premover_ = false;
//                   }

//                   if (right_alc_car_cnt_[tr.track_id].neg > neg_thr_r ||
//                       (((!LCHANGE && !RCHANGE) && lead_one != nullptr &&
//                         lead_two != nullptr &&
//                         lead_two->d_rel - lead_one->d_rel <
//                             std::min(v_right_front, v_target) * 1.5 &&
//                         (lead_two->v_rel < 5. &&
//                          std::fabs(lead_two->v_lat) < 0.3 &&
//                          lead_two->d_min_cpath < 1.1 &&
//                          lead_two->d_min_cpath + lead_two->width > 1.1)))) {
//                     if (right_alc_car_.size() > 0 &&
//                         std::find(right_alc_car_.begin(), right_alc_car_.end(),
//                                   tr.track_id) != right_alc_car_.end()) {
//                       neg_premoved_id_ = tr.track_id;
//                       premoved_id_ = -1000;
//                       premover_ = false;
//                       right_alc_car_.clear();
//                       right_alc_car_cnt_[tr.track_id].pos = 0;
//                       neg_right_alc_car_ = true;
//                     }
//                   }

//                   if (right_lb_car_cnt_[tr.track_id].neg > neg_thr_lb_r ||
//                       ((lead_one != nullptr && lead_two != nullptr &&
//                         lead_two->d_rel - lead_one->d_rel <
//                             std::min(v_right_front, v_target) * 1.5 &&
//                         lead_one->d_min_cpath < -lb_width_r / 2.0) ||
//                        (temp_leadone != nullptr && temp_leadtwo != nullptr &&
//                         right_lb_car_.size() > 0 &&
//                         std::find(right_lb_car_.begin(), right_lb_car_.end(),
//                                   temp_leadone->track_id) !=
//                             right_lb_car_.end() &&
//                         temp_leadtwo->d_rel - temp_leadone->d_rel <
//                             v_target * 1.5 &&
//                         temp_leadone->d_min_cpath < -lb_width_r / 2.0))) {
//                     if (right_lb_car_.size() > 0 &&
//                         std::find(right_lb_car_.begin(), right_lb_car_.end(),
//                                   tr.track_id) != right_lb_car_.end()) {
//                       right_lb_car_.clear();

//                       if (true) {
//                         neg_right_lb_car_ = true;
//                       }

//                       right_lb_car_cnt_[tr.track_id].pos = 0;
//                     }
//                   }
//                 }
//               }
//             } else if (RCHANGE &&
//                        (right_alc_car_.size() > 0 || right_lb_car_.size() > 0 ||
//                         v_ego >= 1.0) &&
//                        tr.d_min_cpath >= lane_width / 2 - car_width / 5 &&
//                        tr.d_min_cpath <
//                            lane_width + car_width / 2 + lane_width * 0.05 &&
//                        dist_to_intsect < 240) {
//               v_rel_r_ = get_vrel_close(1, status);
//               double v_right_front = v_rel_r_ + v_ego;
//               double v_front_lb = tr.v_rel + v_ego;

//               std::array<double, 3> xp1{-22, -10, 0};
//               std::array<double, 3> fp1{20 * coefficient, 35 * coefficient,
//                                         80 * coefficient};

//               std::array<double, 2> xp2{0, 67 / 3.6};
//               std::array<double, 2> fp2{12 / 3.6, 8 / 3.6};

//               int neg_thr_r = int(interp(v_rel_r_, xp1, fp1));
//               int neg_thr_lb_r = neg_thr_r;

//               if (std::min(v_right_front, v_target) <
//                               v_front_lb + interp(v_front_lb, xp2, fp2) ||
//                           v_right_front < v_ego + 2 || tr.v_rel > 2.5 ||
//                           use_lateral_distance_to_judge_cutout_in_active_lane_change
//                       ? tr.l > half_car_width
//                       : std::fabs(tr.v_lat) > 0.3 ||
//                             ((!LCHANGE && !RCHANGE) &&
//                              (lead_one != nullptr && lead_two != nullptr &&
//                               lead_two->d_rel - lead_one->d_rel <
//                                   std::min(v_right_front, v_target) * 2 &&
//                               (lead_two->v_rel < 5. &&
//                                std::fabs(lead_two->v_lat) < 0.3 &&
//                                lead_two->d_min_cpath < 1.1 &&
//                                lead_two->d_min_cpath + lead_two->width >
//                                    1.1))) ||
//                             (temp_leadone != nullptr &&
//                              temp_leadtwo != nullptr &&
//                              temp_leadtwo->d_rel - temp_leadone->d_rel <
//                                  v_target * 2 &&
//                              (temp_leadtwo->v_rel < 5. ||
//                               std::fabs(temp_leadtwo->v_lat) < 0.2))) {
//                 if (right_lb_car_cnt_.find(tr.track_id) !=
//                     right_lb_car_cnt_.end()) {
//                   right_lb_car_cnt_[tr.track_id].neg += 1;
//                   right_lb_car_cnt_[tr.track_id].pos =
//                       std::max(right_lb_car_cnt_[tr.track_id].pos - 3, 0);

//                   if (right_lb_car_cnt_[tr.track_id].neg > neg_thr_lb_r ||
//                       ((lead_one != nullptr && lead_two != nullptr &&
//                         lead_two->d_rel - lead_one->d_rel <
//                             std::min(v_right_front, v_target) * 1.5 &&
//                         lead_one->d_min_cpath < -lb_width_r / 2.0) ||
//                        (temp_leadone != nullptr && temp_leadtwo != nullptr &&
//                         right_lb_car_.size() > 0 &&
//                         std::find(right_lb_car_.begin(), right_lb_car_.end(),
//                                   temp_leadone->track_id) !=
//                             right_lb_car_.end() &&
//                         temp_leadtwo->d_rel - temp_leadone->d_rel <
//                             v_target * 1.5 &&
//                         temp_leadone->d_min_cpath < -lb_width_r / 2.0))) {
//                     if (right_lb_car_.size() > 0 &&
//                         std::find(right_lb_car_.begin(), right_lb_car_.end(),
//                                   tr.track_id) != right_lb_car_.end()) {
//                       right_lb_car_.clear();
//                       if (true) {
//                         neg_right_lb_car_ = true;
//                       }

//                       right_lb_car_cnt_[tr.track_id].pos = 0;
//                     }
//                   }
//                 }

//                 if (right_alc_car_cnt_.find(tr.track_id) !=
//                     right_alc_car_cnt_.end()) {
//                   right_alc_car_cnt_[tr.track_id].neg += 1;
//                   right_alc_car_cnt_[tr.track_id].pos =
//                       std::max(right_alc_car_cnt_[tr.track_id].pos - 3, 0);

//                   if (right_alc_car_cnt_[tr.track_id].neg > neg_thr_r ||
//                       (((!LCHANGE && !RCHANGE) && lead_one != nullptr &&
//                         lead_two != nullptr &&
//                         lead_two->d_rel - lead_one->d_rel <
//                             std::min(v_right_front, v_target) * 1.5 &&
//                         (lead_two->v_rel < 5. &&
//                          std::fabs(lead_two->v_lat) < 0.3 &&
//                          lead_two->d_min_cpath < 1.1 &&
//                          lead_two->d_min_cpath + lead_two->width > 1.1)))) {
//                     if (right_alc_car_.size() > 0 &&
//                         std::find(right_alc_car_.begin(), right_alc_car_.end(),
//                                   tr.track_id) != right_alc_car_.end()) {
//                       neg_premoved_id_ = tr.track_id;
//                       premoved_id_ = -1000;
//                       premover_ = false;
//                       right_alc_car_.clear();
//                       right_alc_car_cnt_[tr.track_id].pos = 0;
//                       neg_right_alc_car_ = true;
//                     }
//                   }
//                 }
//               }
//             }
//           }

//           if (left_alc_car_.size() > 0) {
//             left_lb_car_.clear();
//             right_lb_car_.clear();
//             right_alc_car_.clear();
//             premover_ = false;

//             if (left_lb_car_cnt_.find(tr.track_id) != left_lb_car_cnt_.end()) {
//               left_lb_car_cnt_[tr.track_id].pos = 0;
//             }

//             if (right_lb_car_cnt_.find(tr.track_id) !=
//                 right_lb_car_cnt_.end()) {
//               right_lb_car_cnt_[tr.track_id].pos = 0;
//             }

//             if (right_alc_car_cnt_.find(tr.track_id) !=
//                 right_alc_car_cnt_.end()) {
//               right_alc_car_cnt_[tr.track_id].pos = 0;
//             }
//           } else if (right_alc_car_.size() > 0) {
//             left_lb_car_.clear();
//             right_lb_car_.clear();

//             if (left_lb_car_cnt_.find(tr.track_id) != left_lb_car_cnt_.end()) {
//               left_lb_car_cnt_[tr.track_id].pos = 0;
//             }

//             if (right_lb_car_cnt_.find(tr.track_id) !=
//                 right_lb_car_cnt_.end()) {
//               right_lb_car_cnt_[tr.track_id].pos = 0;
//             }
//           }
//           // fengwang31:新状态机中没有这几个状态，暂时注掉这部分代码
//           //   else if (left_lb_car_.size() > 0 || status == ROAD_LB_LBORROW ||
//           //              status == ROAD_LB_LRETURN || status ==
//           //              ROAD_LB_LSUSPEND) {
//           //     right_lb_car_.clear();

//           //     if (right_lb_car_cnt_.find(tr.track_id) !=
//           //         right_lb_car_cnt_.end()) {
//           //       right_lb_car_cnt_[tr.track_id].pos = 0;
//           //     }
//           //   }
//           // }

//           if (!l_enable) {
//             if (left_alc_car_.size() > 0 &&
//                 front_tracks_r_ids.find(left_alc_car_[0]) !=
//                     front_tracks_r_ids.end()) {
//             } else {
//               left_alc_car_.clear();
//               left_lb_car_.clear();
//               premovel_ = false;

//               if (left_lb_car_cnt_.find(tr.track_id) !=
//                   left_lb_car_cnt_.end()) {
//                 left_lb_car_cnt_[tr.track_id].pos = 0;
//               }

//               if (left_alc_car_cnt_.find(tr.track_id) !=
//                   left_alc_car_cnt_.end()) {
//                 left_alc_car_cnt_[tr.track_id].pos = 0;
//               }
//             }
//           }

//           if (!r_enable) {
//             if (right_alc_car_.size() > 0 &&
//                 front_tracks_l_ids.find(right_alc_car_[0]) !=
//                     front_tracks_l_ids.end()) {
//             } else {
//               right_lb_car_.clear();
//               right_alc_car_.clear();
//               premover_ = false;

//               if (right_lb_car_cnt_.find(tr.track_id) !=
//                   right_lb_car_cnt_.end()) {
//                 right_lb_car_cnt_[tr.track_id].pos = 0;
//               }

//               if (right_alc_car_cnt_.find(tr.track_id) !=
//                   right_alc_car_cnt_.end()) {
//                 right_alc_car_cnt_[tr.track_id].pos = 0;
//               }
//             }
//           }
//         }
//       }

//       if (LCHANGE) {
//         premovel_ = false;
//         premover_ = false;
//         if (!accident_ahead) {
//           right_alc_car_.clear();
//           left_lb_car_.clear();
//           right_lb_car_.clear();
//         }
//       } else if (RCHANGE) {
//         premovel_ = false;
//         premover_ = false;
//         if (!accident_ahead) {
//           left_alc_car_.clear();
//           left_lb_car_.clear();
//           right_lb_car_.clear();
//         }
//       }

//       if (left_alc_car_.size() > 0 &&
//           (front_tracks_ids.find(left_alc_car_[0]) == front_tracks_ids.end() ||
//            front_tracks_c_ids.find(left_alc_car_[0]) ==
//                front_tracks_c_ids.end()) &&
//           !LCHANGE) {
//         if (status == kLaneKeeping) {
//           left_alc_car_.clear();
//           premovel_ = false;
//           premover_ = false;
//         } else {
//           if (olane != nullptr &&
//               olane->get_virtual_id() == clane->get_virtual_id()) {
//             left_alc_car_.clear();
//             premovel_ = false;
//             premover_ = false;
//           }
//         }
//       }

//       if (right_alc_car_.size() > 0 || RCHANGE) {
//         if (!RCHANGE && (front_tracks_ids.find(right_alc_car_[0]) ==
//                              front_tracks_ids.end() ||
//                          front_tracks_c_ids.find(right_alc_car_[0]) ==
//                              front_tracks_c_ids.end())) {
//           if (status == kLaneKeeping) {
//             right_alc_car_.clear();
//             premovel_ = false;
//             premover_ = false;
//           } else {
//             if (olane != nullptr &&
//                 olane->get_virtual_id() == clane->get_virtual_id()) {
//               right_alc_car_.clear();
//               premovel_ = false;
//               premover_ = false;
//             }
//           }
//         }
//       }

//       if (left_lb_car_.size() > 0) {
//         double lat_dist_l = 0.;
//         for (auto &tr : front_tracks) {
//           if (tr.track_id == left_lb_car_[0]) {
//             lat_dist_l = tr.d_max_cpath;
//             break;
//           }
//         }
//         if (front_tracks_ids.find(left_lb_car_[0]) == front_tracks_ids.end() ||
//             lat_dist_l > lane_width * 1.5 + car_width / 8 ||
//             lat_dist_l < -lane_width) {
//           left_lb_car_.clear();
//           premovel_ = false;
//           premover_ = false;
//         }
//       }

//       if (right_lb_car_.size() > 0) {
//         double lat_dist_r = 0.;
//         for (auto &tr : front_tracks) {
//           if (tr.track_id == right_lb_car_[0]) {
//             lat_dist_r = tr.d_min_cpath;
//             break;
//           }
//         }
//         if (front_tracks_ids.find(right_lb_car_[0]) == front_tracks_ids.end() ||
//             lat_dist_r < -(lane_width * 1.5 + car_width / 8) ||
//             lat_dist_r > lane_width) {
//           right_lb_car_.clear();
//           premovel_ = false;
//           premover_ = false;
//         }
//       }

//       auto iter = left_lb_car_cnt_.begin();

//       while (iter != left_lb_car_cnt_.end()) {
//         if (front_tracks_ids.find(iter->first) == front_tracks_ids.end()) {
//           iter = left_lb_car_cnt_.erase(iter);
//         } else {
//           iter++;
//         }
//       }

//       iter = right_lb_car_cnt_.begin();

//       while (iter != right_lb_car_cnt_.end()) {
//         if (front_tracks_ids.find(iter->first) == front_tracks_ids.end()) {
//           iter = right_lb_car_cnt_.erase(iter);
//         } else {
//           iter++;
//         }
//       }

//       iter = left_alc_car_cnt_.begin();

//       while (iter != left_alc_car_cnt_.end()) {
//         if (front_tracks_ids.find(iter->first) == front_tracks_ids.end()) {
//           iter = left_alc_car_cnt_.erase(iter);
//         } else {
//           iter++;
//         }
//       }

//       iter = right_alc_car_cnt_.begin();

//       while (iter != right_alc_car_cnt_.end()) {
//         if (front_tracks_ids.find(iter->first) == front_tracks_ids.end()) {
//           iter = right_alc_car_cnt_.erase(iter);
//         } else {
//           iter++;
//         }
//       }

//       if (lb_leadone_disable) {
//         neg_right_lb_car_ = true;
//         neg_left_lb_car_ = true;
//       }

//       if (is_on_highway) {
//         left_lb_car_.clear();
//         left_lb_car_cnt_.clear();
//         right_lb_car_.clear();
//         right_lb_car_cnt_.clear();
//       }

//       double v_target_final = session_->environmental_model()
//                                   .get_ego_state_manager()
//                                   ->ego_v_cruise();
//       double ego_minium_cruise_speed =
//           config_.minimum_ego_cruise_speed_for_active_lane_change;
//       if (v_target_final <= ego_minium_cruise_speed) {
//         left_alc_car_.clear();
//         left_alc_car_cnt_.clear();
//         right_alc_car_.clear();
//         right_alc_car_cnt_.clear();
//       }

//     } else {
//       left_lb_car_.clear();
//       left_lb_car_cnt_.clear();
//       neg_right_lb_car_ = true;
//       left_alc_car_.clear();
//       right_lb_car_.clear();
//       right_alc_car_.clear();
//       left_alc_car_cnt_.clear();
//       right_lb_car_cnt_.clear();
//       right_alc_car_cnt_.clear();
//       neg_right_alc_car_ = true;
//       premovel_ = false;
//       premover_ = false;
//     }
//     LOG_DEBUG(
//         "WRDEBUG left_alc_car_.size()[%lu], left_lb_car_.size()[%lu], "
//         "right_alc_car_.size()[%lu], right_lb_car_.size()[%lu]",
//         left_alc_car_.size(), left_lb_car_.size(), right_alc_car_.size(),
//         right_lb_car_.size());

//     if (upstream_enable_r || upstream_enable_l) {
//       if (!(upstream_enable_lb)) {
//         if (left_alc_car_.size() > 0) return true;
//         if (right_alc_car_.size() > 0) return true;
//         if (left_lb_car_.size() > 0) return true;
//         if (right_lb_car_.size() > 0) return true;
//       }
//       if (upstream_enable_l) {
//         if (right_alc_car_.size() > 0 || right_lb_car_.size() > 0) {
//           if (!upstream_enable_r) {
//             if (!RCHANGE) {
//               right_alc_car_.clear();
//               right_lb_car_.clear();
//             }
//           } else {
//             return true;
//           }
//         } else if (left_alc_car_.size() > 0 || left_lb_car_.size() > 0) {
//           return true;
//         }
//         if (left_alc_car_.size() == 0 && left_lb_car_.size() == 0 &&
//             !(upstream_enable_lb)) {
//           for (auto &tr : front_tracks_c) {
//             if (tr.v_lead > 2.0)
//               break;  // assume front_tracks_c is in d_rel-ascending order
//             left_alc_car_.push_back(tr.track_id);
//           }
//           right_alc_car_.clear();
//           right_lb_car_.clear();
//           left_lb_car_.clear();
//           neg_left_alc_car_ = false;
//           neg_right_alc_car_ = false;
//           neg_left_lb_car_ = false;
//           neg_right_lb_car_ = false;
//         }

//       } else {
//         if ((left_alc_car_.size() > 0 || left_lb_car_.size() > 0)) {
//           if (!upstream_enable_l) {
//             if (!LCHANGE) {
//               left_alc_car_.clear();
//               left_lb_car_.clear();
//             }
//           } else {
//             return true;
//           }
//         } else if (right_alc_car_.size() > 0 || right_lb_car_.size() > 0) {
//           return true;
//         }
//         if (right_alc_car_.size() == 0 && right_lb_car_.size() == 0 &&
//             !(upstream_enable_lb)) {
//           for (auto &tr : front_tracks_c) {
//             if (tr.v_lead > 2.0) break;
//             right_alc_car_.push_back(tr.track_id);
//           }
//           left_alc_car_.clear();
//           right_lb_car_.clear();
//           left_lb_car_.clear();
//           neg_left_alc_car_ = false;
//           neg_right_alc_car_ = false;
//           neg_left_lb_car_ = false;
//           neg_right_lb_car_ = false;
//         }
//       }
//     }
//     return true;
//   }

//   return false;
// }  // namespace planning
// }  // namespace planning