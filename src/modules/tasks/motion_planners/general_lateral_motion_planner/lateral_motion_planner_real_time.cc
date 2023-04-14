
// #include
// "src/modules/tasks/motion_planners/lateral_motion_planner_real_time/lateral_motion_planner_real_time.h"

#include <iomanip>
#include <vector>

// #include "core/common/trace.h"
// #include "core/modules/common/ego_prediction_utils.h"
// #include "core/modules/context/ego_state.h"
// #include "core/modules/np_functions/mrc_condition.h"
#include "lateral_motion_planner_real_time.h"

namespace planning {

LateralMotionPlannerV1::LateralMotionPlannerV1(
    const EgoPlanningConfigBuilder *config_builder,
    const std::shared_ptr<TaskPipelineContext> &pipeline_context)
    : Task(config_builder, pipeline_context) {
  config_ = config_builder->cast<LateralMotionPlannerV1Config>();
  name_ = "LateralMotionPlannerV1";
}

bool LateralMotionPlannerV1::Execute(planning::framework::Frame *frame) {
  // NTRACE_CALL(7);

  // auto config_builder =
  //     frame->mutable_session()->mutable_planning_context()->config_builder(
  //         planning::common::SceneType::HIGHWAY);

  const auto &session = frame->session();
  auto &planning_context = session->planning_context();
  auto &ego_prediction_result = pipeline_context_->planning_result;
  auto &ego_prediction_info = pipeline_context_->planning_info;
  bool b_success = false;

  // obtain the session information
  const auto &state_machine_output =
      planning_context.lat_behavior_state_machine_output();
  const auto &lat_behavior_info = planning_context.lat_behavior_info();
  const auto &status = state_machine_output.curr_state;
  const auto &accident_ahead = state_machine_output.accident_ahead;
  const auto &should_premove = state_machine_output.should_premove;
  const auto &should_suspend = state_machine_output.should_suspend;
  const auto &avd_car_past = lat_behavior_info.avd_car_past;
  const auto &avd_sp_car_past = lat_behavior_info.avd_sp_car_past;
  const auto &flag_avd = lat_behavior_info.flag_avd;
  const auto &dist_rblane = lat_behavior_info.dist_rblane;

  b_success =
      update(status, flag_avd, accident_ahead, should_premove, should_suspend,
             dist_rblane, avd_car_past, avd_sp_car_past);

  if (!b_success) {
    // TBD : add logs
  }

  return b_success;
}

bool LateralMotionPlannerV1::update(
    int status, bool flag_avd, bool accident_ahead, bool should_premove,
    bool should_suspend, double dist_rblane,
    const std::array<std::vector<double>, 2> &avd_car_past,
    const std::array<std::vector<double>, 2> &avd_sp_car_past) {
  // std::shared_ptr<VirtualLane> flane = frame_->session()
  //                                          ->environmental_model()
  //                                          .virtual_lane_manager()
  //                                          ->current_lane();

  // std::reverse_copy(flane.c_poly().begin(), flane.c_poly().end(),
  //                   c_poly_.begin());  // c_poly should

  // if (flane.status() == LaneStatusEx::BOTH_MISSING) {
  //   std::reverse_copy(flane.c_poly().begin(), flane.c_poly().end(),
  //                     d_poly_.begin());  // BOTH Missing 是指？
  // }

  // l_poly_.fill(0);
  // r_poly_.fill(0);

  // update_basic_path(status);

  // if (status == ROAD_LC_LWAIT || status == ROAD_LC_RWAIT ||
  //     status == ROAD_LC_LBACK || status == ROAD_LC_RBACK ||
  //     status == INTER_GS_LC_LWAIT || status == INTER_GS_LC_RWAIT ||
  //     status == INTER_TR_LC_LWAIT || status == INTER_TR_LC_RWAIT ||
  //     status == INTER_TL_LC_LWAIT || status == INTER_TL_LC_RWAIT) {
  //   update_premove_path(status, should_premove, should_suspend,
  //   accident_ahead,
  //                       avd_car_past);
  // } else {
  //   premoving_ = false;
  // }

  // if ((status >= ROAD_NONE && status <= INTER_GS_NONE) ||
  //     status == INTER_TR_NONE || status == INTER_TL_NONE ||
  //     status == INTER_GS_LC_LCHANGE || status == INTER_GS_LC_RCHANGE ||
  //     status == INTER_TR_LC_LCHANGE || status == INTER_TR_LC_RCHANGE ||
  //     status == INTER_TL_LC_LCHANGE || status == INTER_TL_LC_RCHANGE) {
  //   update_avoidance_path(status, flag_avd, accident_ahead, should_premove,
  //                         dist_rblane, avd_car_past, avd_sp_car_past);
  // } else {
  //   lat_offset_ = 0;
  // }
}

// bool LateralMotionPlannerV1::check_premove(const int &lane_status) {
//   if (lane_status == ScenarioStateEnum::ROAD_LC_LWAIT ||
//       lane_status == ScenarioStateEnum::ROAD_LC_RWAIT ||
//       lane_status == ScenarioStateEnum::ROAD_LC_LBACK ||
//       lane_status == ScenarioStateEnum::ROAD_LC_RBACK ||
//       lane_status == ScenarioStateEnum::INTER_GS_LC_LWAIT ||
//       lane_status == ScenarioStateEnum::INTER_GS_LC_RWAIT ||
//       lane_status == ScenarioStateEnum::INTER_TR_LC_LWAIT ||
//       lane_status == ScenarioStateEnum::INTER_TR_LC_RWAIT ||
//       lane_status == ScenarioStateEnum::INTER_TL_LC_LWAIT ||
//       lane_status == ScenarioStateEnum::INTER_TL_LC_RWAIT) {
//     // TODO@cai: add log
//     return true;
//   } else {
//     return false;
//   }
// }

// bool LateralMotionPlannerV1::check_avoidance_path(const int &lane_status) {
//   if ((lane_status >= ScenarioStateEnum::ROAD_NONE &&
//        lane_status <= ScenarioStateEnum::INTER_GS_NONE) ||
//       lane_status == ScenarioStateEnum::INTER_TR_NONE ||
//       lane_status == ScenarioStateEnum::INTER_TL_NONE ||
//       lane_status == ScenarioStateEnum::INTER_GS_LC_LCHANGE ||
//       lane_status == ScenarioStateEnum::INTER_GS_LC_RCHANGE ||
//       lane_status == ScenarioStateEnum::INTER_TR_LC_LCHANGE ||
//       lane_status == ScenarioStateEnum::INTER_TR_LC_RCHANGE ||
//       lane_status == ScenarioStateEnum::INTER_TL_LC_LCHANGE ||
//       lane_status == ScenarioStateEnum::INTER_TL_LC_RCHANGE) {
//     // TODO@cai: add log
//     return true;
//   } else {
//     return false;
//   }
// }

// bool LateralMotionPlannerV1::update_basic_path(const int &lane_status) {
//   constexpr double REJETC_PROB_THRESHOLD = 0.5;
//   constexpr double SHORT_REJECT_LENGTH = 15.;

//   reject_reason_ = RejectReason::NO_REJECTION;
//   left_reject_ = false;
//   right_reject_ = false;
//   intercept_width_ = 3.8;

//   const auto &fix_lane =
//       frame_->session()
//           ->environmental_model_->virtual_lane_manager_.get_fix_lane();
//   const auto &map_info_manager =
//       frame_->session()
//           ->environmental_model_->virtual_lane_manager_
//           .get_map_info_manager();  // map info in virtual_lane_manager
//   const auto &map_info =
//       frame_->session()
//           ->environmental_model_->virtual_lane_manager_.get_map_info();

//   const auto fix_lane_status = fix_lane.status();
//   const auto fix_lane_width = fix_lane.width();
//   const auto min_width = fix_lane.min_width();
//   const auto max_width = fix_lane.max_width();

//   // left_poly_.fill(0);  // has been filled above
//   // right_poly_.fill(0);

//   double left_prob, right_prob, intercept_width;

//   if (fix_lane_status == LaneStatusEx::LEFT_AVAILABLE) {
//     if (fix_lane_width > min_width && fix_lane_width < max_width) {
//       std::reverse_copy(fix_lane.c_poly().begin(),
//                         fix_lane.c_poly().end(),  // fix_lane.c_poly：0-3
//                         desired_poly_.begin());   // desired_poly_ : 3-0
//     } else {
//       left_prob = 1.;
//       right_prob = 0.;

//       std::reverse_copy(
//           fix_lane.polys()[0].begin(),
//           fix_lane.polys()[0].end(),  // fix_lane.c_poly 和 polys的区别是啥？
//           left_poly_.begin());
//       // right_poly_.fill(0.0);// has been filled above

//       fix_lane_width = clip(fix_lane_width, max_width, min_width);
//       intercept_width =
//           fix_lane_width * std::sqrt(1 + left_poly_[2] * left_poly_[2]);

//       calc_desired_path(left_poly_, right_poly_, left_prob, right_prob,
//                         intercept_width, desired_poly_);
//     }
//   } else if (fix_lane_status == LaneStatusEx::RIGHT_AVAILABLE) {
//     if (fix_lane_width > min_width && fix_lane_width < max_width) {
//       std::reverse_copy(fix_lane.c_poly().begin(), fix_lane.c_poly().end(),
//                         desired_poly_.begin());
//     } else {
//       left_prob = 0.;
//       right_prob = 1.;

//       // left_poly_.fill(0.0);// has been filled above
//       std::reverse_copy(fix_lane.polys()[1].begin(),
//       fix_lane.polys()[1].end(),
//                         right_poly_.begin());

//       fix_lane_width = clip(fix_lane_width, max_width, min_width);
//       intercept_width =
//           fix_lane_width * std::sqrt(1 + right_poly_[2] * right_poly_[2]);

//       calc_desired_path(left_poly_, right_poly_, left_prob, right_prob,
//                         intercept_width, desired_poly_);
//     }
//   } else {
//     left_prob = 1;
//     right_prob = 1;
//     double left_intercept = fix_lane.intercepts()[0];
//     double right_intercept = fix_lane.intercepts()[1];
//     double left_length = fix_lane.heads()[0];  // 啥意思
//     double right_length = fix_lane.heads()[1];

//     bool left_reject = false;
//     bool right_reject = false;
//     bool wide_reject_enable = true;
//     bool narrow_reject_enable = true;
//     bool short_reject_enable = true;

//     double gap_distance = 1.0;

//     if (fix_lane_width > max_width) {
//       if (std::fabs(right_intercept) <
//               std::fabs(left_intercept) - gap_distance &&
//           lane_status != ScenarioStateEnum::ROAD_LC_LWAIT &&
//           lane_status != ScenarioStateEnum::ROAD_LC_LBACK) {
//         left_reject = true;
//         reject_reason_ = RejectReason::WIDE_REJECTION_L;
//       } else if (std::fabs(right_intercept) >=
//                      std::fabs(left_intercept) - gap_distance &&
//                  lane_status != ScenarioStateEnum::ROAD_LC_RWAIT &&
//                  lane_status != ScenarioStateEnum::ROAD_LC_RBACK) {
//         right_reject = true;
//         reject_reason_ = RejectReason::WIDE_REJECTION_R;
//       }
//     }

//     if ((!left_reject && !right_reject) ||
//         reject_reason_ == RejectReason::BIAS_L ||
//         reject_reason_ == RejectReason::BIAS_R) {
//       if (fix_lane_width < min_width) {
//         if (map_info.is_on_highway()) {
//           constexpr double FRONT_DISTANCE_CHECK = 30.0;
//           constexpr double REAR_DISTANCE_CHECK = -15.0;
//           constexpr double MIN_WIDTH = 2.2;

//           double front_lane_witdh = calc_lane_width_by_dist(
//               fix_lane.polys()[0], fix_lane.polys()[1],
//               FRONT_DISTANCE_CHECK);
//           double rear_lane_witdh = calc_lane_width_by_dist(
//               fix_lane.polys()[0], fix_lane.polys()[1], REAR_DISTANCE_CHECK);
//           if (fix_lane_width > MIN_WIDTH && front_lane_witdh > MIN_WIDTH &&
//               rear_lane_witdh > MIN_WIDTH) {
//             // do what in this case
//           } else {
//             if (lane_status == ScenarioStateEnum::ROAD_LC_LCHANGE) {
//               left_reject = true;
//               reject_reason_ = RejectReason::NARROW_REJECTION;
//             } else if (status == ScenarioStateEnum::ROAD_LC_RCHANGE) {
//               right_reject = true;
//               reject_reason_ = RejectReason::NARROW_REJECTION;
//             } else if (reject_reason_ == RejectReason::BIAS_L ||
//                        reject_reason_ == RejectReason::BIAS_R) {
//               left_reject = true;
//               reject_reason_ = RejectReason::NARROW_REJECTION;
//             }
//           }
//         } else {
//           left_reject = true;
//           reject_reason_ = RejectReason::NARROW_REJECTION;
//         }
//       }
//     }

//     if (!left_reject && !right_reject) {
//       if (left_length < SHORT_REJECT_LENGTH && right_length > 60.) {
//         left_reject = true;
//         reject_reason_ = RejectReason::SHORT_REJECTION;
//       } else if (right_length < SHORT_REJECT_LENGTH && right_length > 60.) {
//         right_reject = true;
//         reject_reason_ = RejectReason::SHORT_REJECTION;
//       }
//     }

//     if (fix_lane_width > min_width && fix_lane_width < max_width &&
//         !left_reject && !right_reject) {
//       std::reverse_copy(fix_lane.c_poly().begin(), fix_lane.c_poly().end(),
//                         desired_poly_.begin());
//     } else {
//       fix_lane_width = clip(fix_lane_width, max_width, min_width);

//       if (left_reject) {
//         left_prob = 0.;
//         right_prob = 1.;

//         left_poly_.fill(0.0);
//         std::reverse_copy(fix_lane.polys()[1].begin(),
//                           fix_lane.polys()[1].end(), right_poly_.begin());

//         intercept_width =
//             fix_lane_width * std::sqrt(1 + right_poly_[2] * right_poly_[2]);

//         calc_desired_path(left_poly_, right_poly_, left_prob, right_prob,
//                           intercept_width, desired_poly_);
//       } else if (right_reject) {
//         left_prob = 1.;
//         right_prob = 0.;

//         std::reverse_copy(fix_lane.polys()[0].begin(),
//                           fix_lane.polys()[0].end(), left_poly_.begin());

//         right_poly_.fill(0.0);

//         intercept_width =
//             fix_lane_width * std::sqrt(1 + left_poly_[2] * left_poly_[2]);

//         calc_desired_path(left_poly_, right_poly_, left_prob, right_prob,
//                           intercept_width, desired_poly_);
//       } else {
//         left_prob = 1.;
//         right_prob = 1.;

//         std::reverse_copy(fix_lane.polys()[0].begin(),
//                           fix_lane.polys()[0].end(), left_poly_.begin());

//         std::reverse_copy(fix_lane.polys()[1].begin(),
//                           fix_lane.polys()[1].end(), right_poly_.begin());

//         intercept_width =
//             lane_width *
//             std::sqrt(1 + std::pow(left_poly_[2] + right_poly_[2], 2) / 4);

//         calc_desired_path(left_poly_, right_poly_, left_prob, right_prob,
//                           intercept_width, desired_poly_);
//       }
//       left_reject_ = left_reject;
//       right_reject_ = right_reject;
//       intercept_width_ = intercept_width;
//     }
//   }
// }

// bool LateralMotionPlannerV1::update_premove_path(
//     int lane_status, bool execute_premove, bool should_suspend,
//     bool exist_accident_ahead,
//     const std::array<std::vector<double>, 2> &avoid_car_info) {
//   const auto &fix_lane =
//       seesion_->environmental_model_->virtual_lane_manager_.get_fix_lane();
//   const auto &map_info_manager =
//       seesion_->environmental_model_->virtual_lane_manager_
//           .get_map_info_manager();  // map info in virtual_lane_manager
//   const auto &map_info =
//       seesion_->environmental_model_->virtual_lane_manager_.get_map_info();

//   auto &ego_state =
//       seesion_->environmental_model_->get_baseline_info(fix_lane.position())
//           ->get_ego_state();

//   if (fix_lane.status() == LaneStatusEx::BOTH_MISSING) {
//     // TODO @cai: add log
//     return;
//   }

//   double car_width = 2.2;

//   double v_ego = ego_state.ego_vel;

//   double lane_width = fix_lane.width();

//   left_poly_.fill(0);
//   right_poly_.fill(0);

//   std::reverse_copy(fix_lane.polys()[0].begin(), fix_lane.polys()[0].end(),
//                     left_poly_.begin());

//   std::reverse_copy(fix_lane.polys()[1].begin(), fix_lane.polys()[1].end(),
//                     right_poly_.begin());

//   std::array<double, 3> xp1{10, 20, 30};  // 这些值是否需要定义成config
//   std::array<double, 3> fp1{0.15, 0.3, 0.45};

//   double norminal_move =
//       0.5 * (lane_width - car_width) - interp(v_ego, xp1, fp1);

//   std::array<double, 2> xp2{0.5 * lane_width, 0.5 * (lane_width +
//   car_width)}; std::array<double, 2> fp2{0, norminal_move};

//   double temp_ego = std::min(10.0 / std::max(v_ego, 0.1), 1.0);
//   double temp_poly =
//       std::sqrt(1 + std::pow(left_poly_[2] + right_poly_[2], 2) / 4);

//   if ((lane_status == ScenarioStateEnum::ROAD_LC_LWAIT &&
//        (should_premove || exist_accident_ahead)) ||
//       lane_status == ScenarioStateEnum::ROAD_LC_LBACK ||
//       lane_status == ScenarioStateEnum::INTER_GS_LC_LWAIT ||
//       lane_status == ScenarioStateEnum::INTER_TR_LC_LWAIT ||
//       lane_status == ScenarioStateEnum::INTER_TL_LC_LWAIT) {
//     premoving_ = true;

//     if (avoid_car_info[0].size() > 0 && avoid_car_info[0][5] > 0 &&
//         avoid_car_info[0][5] < lane_width) {
//       lat_offset_ =
//           interp(avoid_car_info[0][5], xp2, fp2) * temp_ego * temp_poly;
//       desired_poly_[3] = center_poly_[3] + lat_offset_;
//     } else if (avoid_car_info[1].size() > 0 && avoid_car_info[1][5] > 0 &&
//                avoid_car_info[1][5] < lane_width) {
//       lat_offset_ =
//           interp(avoid_car_info[1][5], xp2, fp2) * temp_ego * temp_poly;
//       desired_poly_[3] = center_poly_[3] + lat_offset_;

//     } else {
//       lat_offset_ = norminal_move * temp_poly;

//       if (should_suspend && lat_offset_ < -desired_poly_[3]) {
//         lat_offset_ = -desired_poly_[3];
//       }

//       desired_poly_[3] += lat_offset_;
//     }
//   } else if ((status == ScenarioStateEnum::ROAD_LC_RWAIT &&
//               (should_premove || exist_accident_ahead)) ||
//              lane_status == ScenarioStateEnum::ROAD_LC_RBACK ||
//              lane_status == ScenarioStateEnum::INTER_GS_LC_RWAIT ||
//              lane_status == ScenarioStateEnum::INTER_TR_LC_RWAIT ||
//              lane_status == ScenarioStateEnum::INTER_TL_LC_RWAIT) {
//     premoving_ = true;

//     if (avoid_car_info[0].size() > 0 && avoid_car_info[0][6] < 0 &&
//         avoid_car_info[0][6] > -lane_width) {
//       lat_offset_ =
//           -interp(avoid_car_info[0][6], xp2, fp2) * temp_ego * temp_poly;
//       desired_poly_[3] = c_poly_[3] + lat_offset_;
//     } else if (avoid_car_info[1].size() > 0 && avoid_car_info[1][6] < 0 &&
//                avoid_car_info[1][6] > -lane_width_) {
//       lat_offset_ =
//           -interp(avoid_car_info[1][6], xp2, fp2) * temp_ego * temp_poly;
//       desired_poly_[3] = c_poly_[3] + lat_offset_;
//     } else {
//       if (should_suspend && lat_offset_ > -desired_poly_[3]) {
//         lat_offset_ = -desired_poly_[3];
//       } else {
//         lat_offset_ = -norminal_move * temp_poly;
//       }
//       desired_poly_[3] += lat_offset_;
//     }
//   } else {
//     premoving_ = false;
//   }
//   LOG_NOTICE("WR: premoving_[%d]", premoving_);
// }

// bool LateralMotionPlannerV1::update_avoidance_path(
//     int lane_status, bool flag_avoid, bool exist_accident_ahead,
//     bool execute_premove, double dist_rblane,
//     const std::array<std::vector<double>, 2> &avoid_car_info,
//     const std::array<std::vector<double>, 2> &avoid_sp_car_info) {
//   const auto &fix_lane =
//       seesion_->environmental_model_->virtual_lane_manager_.get_fix_lane();
//   const auto &map_info_manager =
//       seesion_->environmental_model_->virtual_lane_manager_
//           .get_map_info_manager();  // map info in virtual_lane_manager
//   const auto &map_info =
//       seesion_->environmental_model_->virtual_lane_manager_.get_map_info();

//   const auto &fix_refline =
//       seesion_->environmental_model_->virtual_lane_manager_.get_fix_refline();
//   const auto &ego_state =
//       seesion_->environmental_model_->get_baseline_info(fix_lane.position())
//           ->get_ego_state();
//   const auto &isRedLightStop =
//       seesion_->environmental_model_->get_traffic_light_decision()
//           ->get_stop_flag();

//   const auto &v_cruise = map_info_manager.get_map_info().v_cruise();
//   double lane_width = fix_lane.width();
//   const auto &min_width = fix_lane.min_width();
//   const auto &max_width = fix_lane.max_width();
//   lane_width = clip(lane_width, max_width, min_width);

//   double entrance_lane_width = lane_width;

//   if (map_info.is_in_intersection()) {
//     for (auto &refline_point : map_info.current_refline_points()) {
//       if (refline_point.car_point.x >= 0 &&
//           refline_point.in_intersection == false &&
//           refline_point.distance_to_left_lane_border < 10. &&
//           refline_point.distance_to_right_lane_border < 10.) {
//         //@TODO @cai, add log
//         entrance_lane_width = refline_point.distance_to_left_lane_border +
//                               refline_point.distance_to_right_lane_border;
//         break;
//       }
//     }
//     // aovid car: refine lane width
//     if (entrance_lane_width < lane_width) {  // TODO @cai, so much magic
//     numbers
//       if (lane_status != ScenarioStateEnum::INTER_TL_NONE ||
//           map_info.intsect_length() == DBL_MAX) {
//         std::array<double, 2> xp{20, 30};
//         std::array<double, 2> fp{entrance_lane_width, lane_width};
//         lane_width = interp(map_info.dist_to_last_intsect(), xp, fp);
//       } else if (lane_status == ScenarioStateEnum::INTER_TL_NONE &&
//                  map_info.intsect_length() != DBL_MAX) {
//         if (map_info.intsect_length() < 70) {
//           std::array<double, 2> xp{20, 30};
//           std::array<double, 2> fp{entrance_lane_width, lane_width};
//           lane_width = interp(map_info.dist_to_last_intsect(), xp, fp);
//         } else {
//           std::array<double, 4> xp{20, 40, map_info.intsect_length() - 20,
//                                    map_info.intsect_length() - 10};
//           std::array<double, 4> fp{entrance_lane_width, 4.6, 4.6,
//           lane_width}; lane_width = interp(map_info.dist_to_last_intsect(),
//           xp, fp);
//         }
//       }
//     } else {
//       if (lane_status == ScenarioStateEnum::INTER_TL_NONE) {
//         if (map_info.intsect_length() > 70 &&
//             map_info.intsect_length() != DBL_MAX) {
//           std::array<double, 4> xp{10, 40, map_info.intsect_length() - 20,
//                                    map_info.intsect_length() - 10};
//           std::array<double, 4> fp{entrance_lane_width, 4.6, 4.6,
//           lane_width}; lane_width = interp(map_info.dist_to_last_intsect(),
//           xp, fp);
//         } else {
//           std::array<double, 2> xp{20, 40};
//           std::array<double, 2> fp{entrance_lane_width, lane_width};
//           lane_width = interp(map_info.dist_to_last_intsect(), xp, fp);
//         }
//       }
//     }
//   }

//   // calc lat avoid limit
//   double avd_limit_left = 0.15 * lane_width;
//   double avd_limit_right = 0.15 * lane_width;
//   bool special_lane_type = false;

//   for (auto &segment : map_info.current_refline_segments()) {
//     if (segment.is_in_intersection == true) {
//       break;
//     }

//     if (segment.direction ==
//             LaneConnectDirection::LANE_CONNECT_DIRECTION_SPLIT_TO_LEFT ||
//         segment.direction ==
//             LaneConnectDirection::LANE_CONNECT_DIRECTION_SPLIT_TO_RIGHT ||
//         segment.direction ==
//             LaneConnectDirection::LANE_CONNECT_DIRECTION_MERGE_FROM_LEFT ||
//         segment.direction ==
//             LaneConnectDirection::LANE_CONNECT_DIRECTION_MERGE_FROM_RIGHT) {
//       int index = segment.begin_index;
//       if (index >= 0 &&
//           (size_t)index < map_info.current_refline_points().size()) {
//         const auto &dist_x =
//             map_info.current_refline_points()[index].car_point.x;
//         if (dist_x < 60.) {
//           special_lane_type = true;  // short lane length
//           if (dist_x > -10. && lane_width < 2.6) {
//             lane_width = 5.0;  // magic number?
//           }

//           if (segment.direction ==
//                   LaneConnectDirection::LANE_CONNECT_DIRECTION_SPLIT_TO_LEFT
//                   ||
//               segment.direction ==
//                   LaneConnectDirection::
//                       LANE_CONNECT_DIRECTION_MERGE_FROM_RIGHT) {
//             avd_limit_right = 0.2;
//           } else {
//             avd_limit_left = 0.2;
//           }
//           break;
//         }
//       }
//     }
//   }

//   const auto &lane_type = fix_lane.type();

//   left_poly_.fill(0);
//   right_poly_.fill(0);
//   if (fix_lane.status() != LaneStatusEx::BOTH_MISSING) {
//     auto &polys = fix_lane.polys();
//     std::reverse_copy(polys[0].begin(), polys[0].end(), left_poly_.begin());
//     std::reverse_copy(polys[1].begin(), polys[1].end(), right_poly_.begin());
//   }

//   std::array<double, 3> near_car_vrel_v{0.06 * lane_width, 0.02 * lane_width,
//                                         0};
//   std::array<double, 3> near_car_vrel_bp{-7.5, -3.5, 1};

//   std::array<double, 3> near_car_drel_v{0.08 * lane_width, 0.05 * lane_width,
//                                         0};
//   std::array<double, 3> near_car_drel_bp{0, 20, 60};

//   std::array<double, 3> t_gap_vego_v{1.35, 1.55, 2.0};
//   std::array<double, 3> t_gap_vego_bp{5, 15, 30};
//   std::array<double, 3> avd_vRel_v = {20., 5., 1.};
//   std::array<double, 3> avd_vRel_bp = {-7.5, -2.5, 1.};

//   double lat_offset = 0;
//   double lat_offset1 = 0;
//   double lat_offsetl = 0;
//   double lat_offsetr = 0;

//   int two_nudge_car =
//       -1000;  // 这个为啥是int 类型？应该和desired_poly_[3]类型一致？
//   int one_nudge_left_car = -1000;
//   int one_nudge_right_car = -1000;

//   double v_ego = ego_state.ego_vel;
//   double l_ego = ego_state.ego_frenet.y;
//   double safety_dist = 2.0 + v_ego * 0.2;  // magic number
//   double t_gap = interp(v_ego, t_gap_vego_bp, t_gap_vego_v);

//   double dist_offset = 3.5;
//   double car_width = 2.2;
//   double avd_normal_threshold = lane_width / *0.5 - 0.3 - car_width * 0.5;
//   double pre_str_dist = 100.;

//   if (map_info.current_lane_index() != 0) {
//     avd_normal_threshold = lane_width * 0.5 - 0.1 - car_width * 0.5;
//   }

//   sb_lane_ = false;
//   sb_blane_ = false;
//   large_lat_ = false;
//   cross_left_solid_line_ = false;
//   cross_right_solid_line_ = false;
//   force_pause_ = false;
//   avd_car_info_ = avoid_car_info;

//   if (lane_status == ScenarioStateEnum::ROAD_NONE ||
//       lane_status == ScenarioStateEnum::ROAD_LC_LCHANGE ||
//       lane_status == ScenarioStateEnum::ROAD_LC_RCHANGE ||
//       lane_status == ScenarioStateEnum::ROAD_LC_LWAIT ||
//       lane_status == ScenarioStateEnum::ROAD_LC_RWAIT ||
//       lane_status == ScenarioStateEnum::ROAD_LC_LBACK ||
//       lane_status == ScenarioStateEnum::ROAD_LC_RBACK ||
//       lane_status == ScenarioStateEnum::ROAD_LB_LBACK ||
//       lane_status == ScenarioStateEnum::ROAD_LB_RBACK ||
//       lane_status == ScenarioStateEnum::ROAD_LB_LRETURN ||
//       lane_status == ScenarioStateEnum::ROAD_LB_RRETURN ||
//       lane_status == ScenarioStateEnum::INTER_GS_NONE ||
//       lane_status == ScenarioStateEnum::INTER_TR_NONE ||
//       lane_status == ScenarioStateEnum::INTER_TL_NONE ||
//       lane_status == ScenarioStateEnum::INTER_GS_LC_LCHANGE ||
//       lane_status == ScenarioStateEnum::INTER_GS_LC_RCHANGE ||
//       lane_status == ScenarioStateEnum::INTER_TR_LC_LCHANGE ||
//       lane_status == ScenarioStateEnum::INTER_TR_LC_RCHANGE ||
//       lane_status == ScenarioStateEnum::INTER_TL_LC_LCHANGE ||
//       lane_status == ScenarioStateEnum::INTER_TL_LC_RCHANGE) {
//     if (avoid_car_info[0].size() > 0) {
//       double plus1 =
//           interp(avoid_car_info[0][2], near_car_vrel_bp, near_car_vrel_v);
//       double plus1_rel =
//           interp(avoid_car_info[0][3], near_car_drel_bp, near_car_drel_v);
//       double lat_compen1 = 0.5 * plus1 + 0.5 * plus1_rel;

//       if (avoid_car_info[1].size() > 0) {
//         double v_near_car2 = v_ego + avoid_car_info[1][2];
//         double desired_dist2 = dist_offset + v_near_car2 * t_gap;
//         double diff_dist_nudge_car =
//             avoid_car_info[1][3] - avoid_car_info[0][3];
//         double diff_vel_nudge_car = avoid_car_info[1][2] -
//         avoid_car_info[0][2]; double dist_avd_nudge_car1 =
//         avoid_car_info[0][3] + 5.0 + safety_dist;

//         double t_avd_car1 = 0;
//         // if(avoid_car_info[0][2] != 0){
//         if (equal_zero(avoid_car_info[0][2]) == false) {
//           if (avoid_car_info[0][2] < -1.0e-3) {
//             t_avd_car1 = -dist_avd_nudge_car1 / avoid_car_info[0][2];
//           } else {
//             t_avd_car1 = 5.;
//           }
//         } else {
//           t_avd_car1 = (v_ego < 1) ? 5. : 0.;
//         }

//         double plus2 =
//             interp(avoid_car_info[1][2], near_car_vrel_bp, near_car_vrel_v);
//         double plus2_rel =
//             interp(avoid_car_info[1][3], near_car_drel_bp, near_car_drel_v);
//         double lat_compen2 = 0.5 * plus2 + 0.5 * plus2_rel;

//         if (t_avd_car1 > 0) {
//           diff_dist_nudge_car += diff_vel_nudge_car * t_avd_car1;
//           if (avoid_car_info[0][5] > 0 && avoid_car_info[1][5] < 0) {
//             if (diff_dist_nudge_car <
//                     desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego &&
//                 avoid_car_info[0][5] - avoid_car_info[1][6] > 2.8) {
//               lat_offset = avoid_car_info[0][5] -
//                            (avoid_car_info[0][5] - avoid_car_info[1][6]) / 2;

//               if (lat_compen1 > lat_compen2) {
//                 if (lat_offset - (lat_compen1 - lat_compen2) -
//                         avoid_car_info[1][6] >=
//                     1.4) {
//                   lat_offset -= lat_compen1 - lat_compen2;
//                 } else {
//                   lat_offset -= 0.1;
//                 }
//               } else if (lat_compen1 < lat_compen2) {
//                 if (avoid_car_info[0][5] -
//                         (lat_offset - (lat_compen1 - lat_compen2)) >=
//                     1.4) {
//                   lat_offset -= lat_compen1 - lat_compen2;
//                 } else {
//                   lat_offset += 0.1;
//                 }
//               }

//               if (lat_offset >= 0) {
//                 if (avd_normal_threshold > 0) {
//                   lat_offset = std::min(
//                       lat_offset,
//                       std::min(avd_normal_threshold, avoid_car_info[0][9]));
//                 } else {
//                   lat_offset = std::min(
//                       lat_offset,
//                       std::min(0.5 * lane_width - 0.9,
//                       avoid_car_info[0][9]));
//                 }
//               } else {
//                 if (avd_normal_threshold > 0) {
//                   lat_offset = -std::min(
//                       std::fabs(lat_offset),
//                       std::min(avd_normal_threshold, avoid_car_info[0][9]));
//                 } else {
//                   lat_offset = -std::min(
//                       std::fabs(lat_offset),
//                       std::min(0.5 * lane_width - 0.9,
//                       avoid_car_info[0][9]));
//                 }
//               }

//               curr_time_ = IflyTime::Now_s();
//             } else if (avoid_car_info[1][6] < -1.5 &&
//                        diff_dist_nudge_car >=
//                            desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego) {
//               if (IflyTime::Now_s() - curr_time_ > 2) {
//                 lat_offset = 0.5 * (lane_width - car_width / 2 -
//                                     std::fabs(avoid_car_info[0][5])) +
//                              lat_compen1;

//                 lat_offset = std::max(lat_offset, 0.0);

//                 if (avd_normal_threshold > 0) {
//                   lat_offset = -std::min(
//                       lat_offset,
//                       std::min(avd_normal_threshold, avoid_car_info[0][9]));
//                 } else {
//                   lat_offset = -std::min(
//                       lat_offset,
//                       std::min(0.5 * lane_width - 0.9,
//                       avoid_car_info[0][9]));
//                 }
//               } else {
//                 lat_offset = -desired_poly_[3];
//               }
//             } else {  // add some comments
//               if (avoid_car_info[0][2] <= avoid_car_info[1][2] ||
//                   avoid_car_info[0][2] - avoid_car_info[1][2] < 2) {
//                 if (avoid_car_info[0][5] != 100) {
//                   lat_offset = 0.9 * (lane_width - car_width / 2 -
//                                       std::fabs(avoid_car_info[0][5])) +
//                                lat_compen1;
//                 }

//                 lat_offset = std::max(lat_offset, 0.0);
//                 lat_offset = -std::min(
//                     lat_offset,
//                     std::min(0.5 * lane_width - 0.9, avoid_car_info[0][9]));

//                 if (avoid_car_info[1][6] >= 0 && avoid_car_info[1][6] != 100
//                 &&
//                     (avoid_car_info[1][2] + v_ego < 1.5 ||
//                      avoid_car_info[0][3] < 0) &&
//                     avoid_car_info[0][4] > -0.5 && avd_car_past_[0][4] < 0.3
//                     && avoid_car_info[0][1] == 0) {
//                   lat_offset =
//                       avoid_car_info[1][5] -
//                       (avoid_car_info[1][5] + 1.8 + avoid_car_info[0][9]) /
//                       2;
//                   if (((map_info.current_lane_index() ==
//                         map_info.lanes_num() - 1) ||
//                        (map_info.current_lane_index() ==
//                             map_info.lanes_num() - 2 &&
//                         map_info_manager.rlane_.exist() &&
//                         map_info_manager.rlane_.type() ==
//                             MSD_LANE_TYPE_NON_MOTOR)) &&
//                       !map_info.is_on_highway() &&
//                       map_info.dist_to_intsect() > 80. &&
//                       map_info.dist_to_intsect() - avoid_car_info[0][3] > 80.
//                       &&
//                       ((avoid_car_info[0][3] < 15 && v_ego < 5) ||
//                        (v_ego < 10 && avoid_car_info[0][2] + v_ego < -1) ||
//                        avoid_car_info[0][3] < 1)) {
//                     if (-lat_offset - avoid_car_info[0][9] > 0.4) {
//                       lat_offset =
//                           std::max(std::max(lat_offset,
//                                             -0.15 * lane_width -
//                                             dist_rblane),
//                                    std::max(-avoid_car_info[0][9],
//                                             -2.0 + avoid_car_info[0][5]));
//                     } else {
//                       lat_offset = std::max(
//                           lat_offset, std::max(-0.15 * lane_width -
//                           dist_rblane,
//                                                -2.0 + avoid_car_info[0][5]));
//                     }

//                     if (lat_offset == -avoid_car_info[0][9]) {
//                       lat_offset = 0;
//                     }

//                     if (dist_rblane > 1.0) {
//                       sb_blane_ = true;
//                     }
//                     if (std::pow(avoid_car_info[0][2] - 1.0, 2) / 4 >
//                             avoid_car_info[0][3] - 2 &&
//                         avoid_car_info[0][3] > -5 && desired_poly_[3] < -0.7)
//                         {
//                       lat_offset = -1.3 + avoid_car_info[0][5];
//                       force_pause_ = true;
//                     }
//                   } else if ((avoid_car_info[1][5] > -0.65 ||
//                               avoid_car_info[1][7] == 20001) &&
//                              (lane_status == INTER_GS_NONE ||
//                               lane_status == INTER_TR_NONE ||
//                               lane_status == INTER_TL_NONE)) {
//                     lat_offset =
//                         std::max(lat_offset, -1.6 + avoid_car_info[1][5]);
//                     sb_lane_ = true;
//                   }

//                   if (!sb_blane_ || !force_pause_ || !sb_lane_) {
//                     lat_offset = std::max(lat_offset, -0.5 * lane_width +
//                     0.9);
//                   }
//                 } else if (avoid_car_info[1][6] < 0 &&
//                            (avoid_car_info[1][2] + v_ego < 0.5 ||
//                             avoid_car_info[1][3] < 1.0) &&
//                            std::fabs(avoid_car_info[1][4]) < 0.3 &&
//                            ((map_info.current_lane_index() ==
//                                  map_info.lanes_num() - 1 &&
//                              ((map_info_manager.clane_.type() !=
//                                    MSD_LANE_TYPE_NON_MOTOR &&
//                                map_info.current_lane_index() >= 1) ||
//                               (map_info_manager.clane_.type() ==
//                                    MSD_LANE_TYPE_NON_MOTOR &&
//                                map_info.current_lane_index() >= 2))) ||
//                             (map_info.current_lane_index() ==
//                                  map_info.lanes_num() - 2 &&
//                              map_info_manager.rlane_.exist() &&
//                              map_info_manager.rlane_.type() ==
//                                  MSD_LANE_TYPE_NON_MOTOR &&
//                              map_info.current_lane_index() >= 1))) {
//                   lat_offset = std::min(1.5 + avoid_car_info[1][6],
//                                         std::min(avoid_car_info[1][9],
//                                                  avoid_car_info[0][5]
//                                                  - 1.4));
//                   if (lat_offset >= lane_width / 2 - 1.1) {
//                     large_lat_ = true;
//                   } else if (std::pow(avoid_car_info[1][2] - 1.0, 2) / 4 >
//                                  avoid_car_info[1][3] - 2 &&
//                              avoid_car_info[1][3] > -10 &&
//                              desired_poly_[3] < -0.5) {
//                     lat_offset = 1.3 + avoid_car_info[1][6];
//                     force_pause_ = true;
//                   }
//                 } else if ((avoid_car_info[0][7] == 20001 ||
//                             avoid_car_info[1][7] == 20001) &&
//                            map_info.is_in_intersection()) {
//                   if (std::fabs(avoid_car_info[1][5] - 1.0) >
//                       (avoid_car_info[0][6] + 1.3)) {  // need consider more
//                     lat_offset = avoid_car_info[0][6] + 1.3;
//                   } else {
//                     lat_offset = avoid_car_info[1][5] - 1.3;
//                   }
//                 } else {
//                   avd_car_past_[1].clear();
//                 }
//               } else {
//                 if (avoid_car_info[1][6] != 100) {
//                   lat_offset = 0.9 * (lane_width - car_width / 2 -
//                                       std::fabs(avoid_car_info[1][6])) +
//                                lat_compen1;
//                 }

//                 lat_offset = std::max(lat_offset, 0.0);
//                 lat_offset = std::min(
//                     lat_offset,
//                     std::min(0.5 * lane_width - 0.9, avoid_car_info[0][9]));
//                 avd_car_past_[0] = avoid_car_info[1];
//                 avd_car_past_[1].clear();
//               }
//             }
//             if (avoid_car_info[0][7] == 20001 &&
//                 map_info.dist_to_intsect() - avoid_car_info[0][3] >= -5 &&
//                 map_info.dist_to_intsect() - avoid_car_info[0][3] < 50 &&
//                 (avoid_car_info[0][5] <=
//                  ((car_width + 0.3) - lane_width / 2))) {
//               if (map_info.current_lane_index() != map_info.lanes_num() - 1
//               ||
//                   dist_rblane > 1.5) {
//                 cross_right_solid_line_ = true;
//               } else if (map_info.current_lane_index() != 0 ||
//                          f_refline.position() == RIGHT_POS) {
//                 cross_left_solid_line_ = true;
//               }
//               lat_offset = 0.;
//             } else if (avoid_car_info[1][7] == 20001 &&
//                        map_info.dist_to_intsect() - avoid_car_info[1][3] >=
//                            -5 &&
//                        map_info.dist_to_intsect() - avoid_car_info[1][3] < 50
//                        && (avoid_car_info[1][6] >=
//                         (lane_width / 2 - (car_width + 0.3)))) {
//               if (avoid_car_info[1][6] >= 0) {
//                 if (map_info.current_lane_index() != map_info.lanes_num() - 1
//                 ||
//                     dist_rblane > 1.5) {
//                   cross_right_solid_line_ = true;
//                 } else if (map_info.current_lane_index() != 0 ||
//                            f_refline.position() == RIGHT_POS) {
//                   cross_left_solid_line_ = true;
//                 }
//               } else {
//                 if (map_info.current_lane_index() != 0 ||
//                     f_refline.position() == RIGHT_POS) {
//                   cross_left_solid_line_ = true;
//                 } else if (map_info.current_lane_index() !=
//                                map_info.lanes_num() - 1 ||
//                            dist_rblane > 1.5) {
//                   cross_right_solid_line_ = true;
//                 }
//               }
//               lat_offset = 0.;
//             }
//           } else if (avoid_car_info[0][5] < 0 && avoid_car_info[1][5] > 0) {
//             if (diff_dist_nudge_car <
//                     desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego &&
//                 std::fabs(avoid_car_info[0][6] - avoid_car_info[1][5]) > 2.8)
//                 {
//               lat_offset = avoid_car_info[0][6] -
//                            (avoid_car_info[0][6] - avoid_car_info[1][5]) / 2;
//               if (lat_compen1 > lat_compen2) {
//                 if (avoid_car_info[1][5] -
//                         (lat_offset + (lat_compen1 - lat_compen2)) >=
//                     1.4) {
//                   lat_offset += lat_compen1 - lat_compen2;
//                 } else {
//                   lat_offset += 0.1;
//                 }
//               } else if (lat_compen1 < lat_compen2) {
//                 if (lat_offset + (lat_compen1 - lat_compen2) -
//                         avoid_car_info[0][6] >=
//                     1.4) {
//                   lat_offset += lat_compen1 - lat_compen2;
//                 } else {
//                   lat_offset -= 0.1;
//                 }
//               }

//               if (lat_offset >= 0) {
//                 if (avd_normal_threshold > 0) {
//                   lat_offset = std::min(
//                       lat_offset,
//                       std::min(avd_normal_threshold, avoid_car_info[0][9]));
//                 } else {
//                   lat_offset = std::min(
//                       lat_offset,
//                       std::min(0.5 * lane_width - 0.9,
//                       avoid_car_info[0][9]));
//                 }
//               } else {
//                 if (avd_normal_threshold > 0) {
//                   lat_offset = -std::min(
//                       std::fabs(lat_offset),
//                       std::min(avd_normal_threshold, avoid_car_info[0][9]));
//                 } else {
//                   lat_offset = -std::min(
//                       std::fabs(lat_offset),
//                       std::min(0.5 * lane_width - 0.9,
//                       avoid_car_info[0][9]));
//                 }
//               }

//               curr_time_ = IflyTime::Now_s();
//             } else if (avoid_car_info[0][6] < -1.5 &&
//                        diff_dist_nudge_car >=
//                            desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego) {
//               if (IflyTime::Now_s() - curr_time_ > 2) {
//                 lat_offset = 0.5 * (lane_width - car_width / 2 -
//                                     std::fabs(avoid_car_info[0][6])) +
//                              lat_compen1;
//                 lat_offset = std::max(lat_offset, 0.0);

//                 if (avd_normal_threshold > 0) {
//                   lat_offset = std::min(
//                       lat_offset,
//                       std::min(avd_normal_threshold, avoid_car_info[0][9]));
//                 } else {
//                   lat_offset = std::min(
//                       lat_offset,
//                       std::min(0.5 * lane_width - 0.9,
//                       avoid_car_info[0][9]));
//                 }
//               } else {
//                 lat_offset = -desired_poly_[3];
//               }
//             } else {
//               if (avoid_car_info[0][2] <= avoid_car_info[1][2] ||
//                   avoid_car_info[0][2] - avoid_car_info[1][2] < 2) {
//                 if (avoid_car_info[0][6] != 100) {
//                   lat_offset = 0.9 * (lane_width - car_width / 2 -
//                                       std::fabs(avoid_car_info[0][6])) +
//                                lat_compen1;
//                 }

//                 lat_offset = std::max(lat_offset, 0.0);
//                 lat_offset = std::min(
//                     lat_offset,
//                     std::min(0.5 * lane_width - 0.9, avoid_car_info[0][9]));

//                 if (avoid_car_info[0][6] >= 0 && avoid_car_info[0][6] != 100
//                 &&
//                     (avoid_car_info[0][2] + v_ego < 1.5 ||
//                      avoid_car_info[0][3] < 0) &&
//                     avoid_car_info[0][4] > -0.5 && avoid_car_info[0][4] < 0.3
//                     && avoid_car_info[0][1] == 0) {
//                   lat_offset =
//                       avoid_car_info[0][5] -
//                       (avoid_car_info[0][5] + 1.8 + avoid_car_info[0][9]) /
//                       2;

//                   if ((map_info.current_lane_index() ==
//                        map_info.lanes_num() - 1) ||
//                       (map_info.current_lane_index() ==
//                            map_info.lanes_num() - 2 &&
//                        map_info_manager.rlane_.exist() &&
//                        map_info_manager.rlane_.type() ==
//                            MSD_LANE_TYPE_NON_MOTOR)) {
//                     if (!map_info.is_on_highway() &&
//                         map_info.dist_to_intsect() > 80 &&
//                         map_info.dist_to_intsect() - avoid_car_info[0][3] >
//                             80 &&
//                         ((avoid_car_info[0][3] < 15 && v_ego < 5) ||
//                          (v_ego < 10 && avoid_car_info[0][2] + v_ego < -1) ||
//                          avoid_car_info[0][3] < 1)) {
//                       if (-lat_offset - avoid_car_info[0][9] > 0.4) {
//                         lat_offset =
//                             std::max(std::max(lat_offset,
//                                               -0.15 * lane_width -
//                                               dist_rblane),
//                                      std::max(-avoid_car_info[0][9],
//                                               -2.0 + avoid_car_info[0][5]));
//                       } else {
//                         lat_offset =
//                             std::max(lat_offset,
//                                      std::max(-0.15 * lane_width -
//                                      dist_rblane,
//                                               -2.0 + avoid_car_info[0][5]));
//                       }

//                       if (lat_offset == -avoid_car_info[0][9]) {
//                         lat_offset = 0;
//                       }

//                       if (dist_rblane > 1.0) {
//                         sb_blane_ = true;
//                       }
//                       if (std::pow(avoid_car_info[0][2] - 1.0, 2) / 4 >
//                               avoid_car_info[0][3] - 2 &&
//                           avoid_car_info[0][3] > -5 &&
//                           desired_poly_[3] < -0.7) {
//                         lat_offset = -1.3 + avoid_car_info[0][5];
//                         force_pause_ = true;
//                       }
//                     } else {
//                       lat_offset = 0;
//                     }
//                   } else if ((avoid_car_info[0][5] > -0.65 ||
//                               avoid_car_info[0][7] == 20001) &&
//                              (status == INTER_GS_NONE ||
//                               status == INTER_TR_NONE ||
//                               status == INTER_TL_NONE)) {
//                     lat_offset =
//                         std::max(lat_offset, -1.6 + avoid_car_info[0][5]);
//                     sb_lane_ = true;
//                   }

//                   if (!sb_blane_ || !force_pause_ || !sb_lane_) {
//                     lat_offset = std::max(lat_offset, -0.5 * lane_width +
//                     0.9);
//                   }
//                 } else if (avoid_car_info[0][6] < 0 &&
//                            (avoid_car_info[0][2] + v_ego < 0.5 ||
//                             avoid_car_info[0][3] < 1) &&
//                            std::fabs(avoid_car_info[0][4]) < 0.3 &&
//                            ((map_info.current_lane_index() ==
//                                  map_info.lanes_num() - 1 &&
//                              ((map_info_manager.clane_.type() !=
//                                    MSD_LANE_TYPE_NON_MOTOR &&
//                                map_info.current_lane_index() >= 1) ||
//                               (map_info_manager.clane_.type() ==
//                                    MSD_LANE_TYPE_NON_MOTOR &&
//                                map_info.current_lane_index() >= 2))) ||
//                             (map_info.current_lane_index() ==
//                                  map_info.lanes_num() - 2 &&
//                              map_info_manager.rlane_.exist() &&
//                              map_info_manager.rlane_.type() ==
//                                  MSD_LANE_TYPE_NON_MOTOR &&
//                              map_info.current_lane_index() >= 1))) {
//                   lat_offset = std::min(1.5 + avoid_car_info[0][6],
//                                         std::min(avoid_car_info[0][9],
//                                                  avoid_car_info[1][5]
//                                                  - 1.4));

//                   if (lat_offset >= lane_width / 2 - 1.1) {
//                     large_lat_ = true;
//                   } else if (std::pow(avoid_car_info[0][2] - 1, 2) / 4 >
//                                  avoid_car_info[0][3] - 2 &&
//                              avoid_car_info[0][3] > -10 &&
//                              desired_poly_[3] < -0.5) {
//                     lat_offset = 1.3 + avoid_car_info[0][6];
//                     force_pause_ = true;
//                   }
//                 } else if ((avoid_car_info[0][7] == 20001 ||
//                             avoid_car_info[1][7] == 20001) &&
//                            map_info.is_in_intersection()) {
//                   if (std::fabs(avoid_car_info[0][5] - 1.0) >
//                       (avoid_car_info[1][6] + 1.3)) {  // need consider more
//                     lat_offset = avoid_car_info[1][6] + 1.3;
//                   } else {
//                     lat_offset = avoid_car_info[0][5] - 1.3;
//                   }
//                 } else {
//                   avd_car_past_[1].clear();
//                 }
//               } else {
//                 if (avoid_car_info[1][5] != 100) {
//                   lat_offset = 0.9 * (lane_width - car_width / 2 -
//                                       std::fabs(avoid_car_info[1][5])) +
//                                lat_compen1;
//                 }

//                 lat_offset = std::max(lat_offset, 0.0);
//                 lat_offset = -std::min(
//                     lat_offset,
//                     std::min(0.5 * lane_width - 0.9, avoid_car_info[0][9]));

//                 avd_car_past_[0] = avoid_car_info[1];
//                 avd_car_past_[1].clear();
//               }
//             }
//             if (avoid_car_info[1][7] == 20001 &&
//                 map_info.dist_to_intsect() - avoid_car_info[1][3] >= -5 &&
//                 map_info.dist_to_intsect() - avoid_car_info[1][3] < 50 &&
//                 (avoid_car_info[1][5] <=
//                  ((car_width + 0.3) - lane_width / 2))) {
//               if (map_info.current_lane_index() != map_info.lanes_num() - 1
//               ||
//                   dist_rblane > 1.5) {
//                 cross_right_solid_line_ = true;
//               } else if (map_info.current_lane_index() != 0 ||
//                          f_refline.position() == RIGHT_POS) {
//                 cross_left_solid_line_ = true;
//               }
//               lat_offset = 0.;
//             } else if (avoid_car_info[0][7] == 20001 &&
//                        map_info.dist_to_intsect() - avoid_car_info[0][3] >=
//                            -5 &&
//                        map_info.dist_to_intsect() - avoid_car_info[0][3] < 50
//                        && (avoid_car_info[0][6] >=
//                         (lane_width / 2 - (car_width + 0.3)))) {
//               if (avoid_car_info[0][6] >= 0) {
//                 if (map_info.current_lane_index() != map_info.lanes_num() - 1
//                 ||
//                     dist_rblane > 1.5) {
//                   cross_right_solid_line_ = true;
//                 } else if (map_info.current_lane_index() != 0 ||
//                            f_refline.position() == RIGHT_POS) {
//                   cross_left_solid_line_ = true;
//                 }
//               } else {
//                 if (map_info.current_lane_index() != 0 ||
//                     f_refline.position() == RIGHT_POS) {
//                   cross_left_solid_line_ = true;
//                 } else if (map_info.current_lane_index() !=
//                                map_info.lanes_num() - 1 ||
//                            dist_rblane > 1.5) {
//                   cross_right_solid_line_ = true;
//                 }
//               }
//               lat_offset = 0.;
//             }
//           } else if (avoid_car_info[0][5] > 0 && avoid_car_info[1][5] > 0) {
//             if (diff_dist_nudge_car <
//                 desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego) {
//               if (avoid_car_info[0][5] != 100 || avoid_car_info[1][5] != 100)
//               {
//                 lat_offset = 0.5 * (lane_width - car_width / 2 -
//                                     std::fabs(std::min(avoid_car_info[0][5],
//                                                        avoid_car_info[1][5])))
//                                                        +
//                              lat_compen1;
//               }
//             } else {
//               if (avoid_car_info[0][5] != 100) {
//                 lat_offset = 0.5 * (lane_width - car_width / 2 -
//                                     std::fabs(avoid_car_info[0][5])) +
//                              lat_compen1;
//               }
//             }

//             lat_offset = std::max(lat_offset, 0.0);

//             if ((map_info.current_lane_index() != map_info.lanes_num() - 1 &&
//                  (!map_info_manager.rlane_.exist() ||
//                   (map_info_manager.rlane_.exist() &&
//                    map_info_manager.rlane_.type() !=
//                        MSD_LANE_TYPE_NON_MOTOR))) ||
//                 avd_limit_left == 0.2 ||
//                 (avoid_car_info[0][2] + v_ego >= 1.5 &&
//                  avoid_car_info[0][3] >= 0) ||
//                 map_info.dist_to_intsect() <= 80 ||
//                 map_info.dist_to_intsect() - avoid_car_info[0][3] <= 80) {
//               if (avd_normal_threshold > 0 && map_info.dist_to_intsect() > 0)
//               {
//                 lat_offset =
//                     -std::min(std::min(lat_offset, avd_normal_threshold),
//                               std::min(avoid_car_info[0][9],
//                               avd_limit_left));
//               } else {
//                 lat_offset =
//                     -std::min(std::min(lat_offset, 0.5 * lane_width - 0.9),
//                               std::min(avoid_car_info[0][9],
//                               avd_limit_left));
//               }
//               if ((map_info.current_lane_index() != map_info.lanes_num() - 1)
//               &&
//                   avoid_car_info[0][2] + v_ego < 0.5 &&
//                   map_info.dist_to_intsect() > -5 &&
//                   map_info.dist_to_intsect() - avoid_car_info[0][3] < 50 &&
//                   abs(avoid_car_info[0][4]) < 0.2 && !isRedLightStop &&
//                   ((avoid_car_info[0][5] <=
//                     ((car_width + 0.3) - lane_width / 2)) ||
//                    (avoid_car_info[1][5] <=
//                     ((car_width + 0.3) - lane_width / 2)))) {
//                 cross_right_solid_line_ = true;
//                 lat_offset = 0.;
//               }
//               if ((avoid_car_info[0][7] == 20001 &&
//                    map_info.dist_to_intsect() - avoid_car_info[0][3] >= -5 &&
//                    map_info.dist_to_intsect() - avoid_car_info[0][3] < 50 &&
//                    (avoid_car_info[0][5] <=
//                     ((car_width + 0.3) - lane_width / 2))) ||
//                   (avoid_car_info[1][7] == 20001 &&
//                    map_info.dist_to_intsect() - avoid_car_info[1][3] >= -5 &&
//                    map_info.dist_to_intsect() - avoid_car_info[1][3] < 50 &&
//                    (avoid_car_info[1][5] <=
//                     ((car_width + 0.3) - lane_width / 2)))) {
//                 if (map_info.current_lane_index() != map_info.lanes_num() - 1
//                 ||
//                     dist_rblane > 1.5) {
//                   cross_right_solid_line_ = true;
//                 } else if (map_info.current_lane_index() != 0 ||
//                            f_refline.position() == RIGHT_POS) {
//                   cross_left_solid_line_ = true;
//                 }
//                 lat_offset = 0.;
//               }

//               if ((status == INTER_GS_NONE || status == INTER_TR_NONE ||
//                    status == INTER_TL_NONE) &&
//                   avoid_car_info[0][5] < 1.1 &&
//                   (avoid_car_info[0][2] + v_ego < 1.5 ||
//                    avoid_car_info[0][3] < 0) &&
//                   avoid_car_info[0][4] > -0.5 && avoid_car_info[0][4] < 0.3
//                   && avoid_car_info[0][1] == 0) {
//                 lat_offset =
//                     avoid_car_info[0][5] -
//                     (avoid_car_info[0][5] + 2.0 + avoid_car_info[0][9]) / 2;
//                 lat_offset = std::max(lat_offset, -1.6 +
//                 avoid_car_info[0][5]);

//                 sb_lane_ = true;
//               }
//             } else if (!map_info.is_on_highway() &&
//                        abs(avoid_car_info[0][4]) < 0.45) {
//               if ((avoid_car_info[0][3] < 15 && v_ego < 5) ||
//                   (v_ego < 10 && avoid_car_info[0][2] + v_ego < -1) ||
//                   avoid_car_info[0][3] < 1) {
//                 lat_offset = -std::min(
//                     std::min(lat_offset, 0.1 * lane_width + dist_rblane),
//                     std::min(avoid_car_info[0][9], 2 -
//                     avoid_car_info[0][5]));
//               }

//               if (dist_rblane > 1.0 &&
//                   0.5 * lane_width +
//                           std::min(avoid_car_info[0][5],
//                           avoid_car_info[1][5]) <
//                       3.0 + 0.02 * std::fabs(std::min(avoid_car_info[0][2],
//                                                       avoid_car_info[1][2])))
//                                                       {
//                 sb_blane_ = true;
//               }

//               if (std::pow(avoid_car_info[0][2] - 1, 2) / 4 >
//                       avoid_car_info[0][3] - 2 &&
//                   avoid_car_info[0][3] > -5 && desired_poly_[3] < -0.7) {
//                 lat_offset = -1.3 + avoid_car_info[0][5];
//                 force_pause_ = true;
//               }
//             }

//             if (lat_offset > 0) {
//               lat_offset = -desired_poly_[3];
//             }

//             if (fix_lane.status() != LaneStatusEx::BOTH_MISSING &&
//                 right_poly_[3] < 0 && right_poly_[3] > -1.0 &&
//                 (map_info.current_lane_index() == map_info.lanes_num() - 1 &&
//                  (dist_rblane < 0.5 || avd_limit_left == 0.2)) &&
//                 !special_lane_type) {
//               lat_offset = std::min(lat_offset + right_poly_[3] + 1.0, 0.0);
//             }
//           } else if (avoid_car_info[0][5] < 0 && avoid_car_info[1][5] < 0) {
//             if (diff_dist_nudge_car <
//                 desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego) {
//               if (avoid_car_info[0][6] != 100 && avoid_car_info[1][6] != 100
//               &&
//                   avoid_car_info[0][6] < 0 && avoid_car_info[1][6] < 0) {
//                 lat_offset = 0.5 * (lane_width - car_width / 2 -
//                                     std::fabs(std::max(avoid_car_info[0][6],
//                                                        avoid_car_info[1][6])))
//                                                        +
//                              lat_compen1;
//               } else if (avoid_car_info[0][6] != 100 &&
//                          avoid_car_info[0][6] < 0) {
//                 lat_offset = 0.5 * (lane_width - car_width / 2 -
//                                     std::fabs(avoid_car_info[0][6])) +
//                              lat_compen1;
//               } else if (avoid_car_info[1][6] != 100 &&
//                          avoid_car_info[1][6] < 0) {
//                 lat_offset = 0.5 * (lane_width - car_width / 2 -
//                                     std::fabs(avoid_car_info[1][6])) +
//                              lat_compen1;
//               }

//               if ((status == INTER_GS_NONE || status == INTER_TR_NONE ||
//                    status == INTER_TL_NONE) &&
//                   (avoid_car_info[0][6] > 0 || (avoid_car_info[1][7] == 20001
//                   &&
//                                                 avoid_car_info[1][6] > 0)) &&
//                   (avoid_car_info[0][2] + v_ego < 1.5 ||
//                    avoid_car_info[0][3] < 0) &&
//                   avoid_car_info[0][4] > -0.5 && avoid_car_info[0][4] < 0.3
//                   && avoid_car_info[0][1] == 0) {
//                 if (((avoid_car_info[0][7] != 20001 ||
//                       status != INTER_TL_NONE ||
//                       map_info.dist_to_last_intsect() - avoid_car_info[0][3]
//                       <=
//                           15) &&
//                      avoid_car_info[0][5] - l_ego > -0.6) ||
//                     (avoid_car_info[0][7] == 20001 &&
//                      (status == INTER_TL_NONE &&
//                           std::fabs(std::min(avoid_car_info[0][5],
//                                              avoid_car_info[1][5])) +
//                                   0.3 <
//                               avoid_car_info[0][6] ||
//                       (avoid_car_info[1][7] != 20001 &&
//                        avoid_car_info[1][5] - l_ego > -1.)) &&
//                      map_info.dist_to_last_intsect() - avoid_car_info[0][3] >
//                          15)) {  // need a history record for
//                                  // avoid_car_info[0][5]
//                   lat_offset =
//                       std::min(avoid_car_info[0][5], avoid_car_info[1][5]) -
//                       (std::min(avoid_car_info[0][5], avoid_car_info[1][5]) +
//                        2.0 + avoid_car_info[0][9]) /
//                           2;
//                   lat_offset = std::max(lat_offset,
//                                         -1.6 + std::min(avoid_car_info[0][5],
//                                                         avoid_car_info[1][5]));

//                   sb_lane_ = true;
//                 } else if (avoid_car_info[0][6] < 0.65 ||
//                            avoid_car_info[0][7] == 20001) {
//                   lat_offset = std::min(
//                       std::max(avoid_car_info[0][6], avoid_car_info[1][6]) -
//                           (std::max(avoid_car_info[0][6],
//                                     avoid_car_info[1][6]) -
//                            1.8 - avoid_car_info[0][9]) /
//                               2,
//                       1.6 -
//                           std::max(avoid_car_info[0][6],
//                           avoid_car_info[1][6]));
//                   lat_offset = std::min(lat_offset, 1.1);
//                 }
//               }

//               if (avoid_car_info[0][6] >= 0 && avoid_car_info[0][6] != 100 &&
//                   (avoid_car_info[0][2] + v_ego < 1.5 ||
//                    avoid_car_info[0][3] < 0) &&
//                   std::fabs(avoid_car_info[0][4]) < 0.45 &&
//                   ((map_info.current_lane_index() ==
//                     map_info.lanes_num() - 1) ||
//                    (map_info.current_lane_index() == map_info.lanes_num() - 2
//                    &&
//                     map_info_manager.rlane_.exist() &&
//                     map_info_manager.rlane_.type() ==
//                         MSD_LANE_TYPE_NON_MOTOR)) &&
//                   !map_info.is_in_intersection()) {
//                 if (((!map_info.is_on_highway() &&
//                       map_info.dist_to_intsect() > 80 &&
//                       map_info.dist_to_intsect() - avoid_car_info[0][3] > 80)
//                       ||
//                      (!isRedLightStop && accident_ahead)) &&
//                     ((avoid_car_info[0][3] < 15 && v_ego < 5) ||
//                      (v_ego < 10 && avoid_car_info[0][2] + v_ego < -1) ||
//                      avoid_car_info[0][3] < 1)) {
//                   lat_offset =
//                       avoid_car_info[0][5] -
//                       (avoid_car_info[0][5] + 2.0 + avoid_car_info[0][9]) /
//                       2;

//                   if (-lat_offset - avoid_car_info[0][9] > 0.4) {
//                     lat_offset = std::max(
//                         std::max(lat_offset, -0.15 * lane_width -
//                         dist_rblane), std::max(-avoid_car_info[0][9],
//                                  -2.0 + avoid_car_info[0][5]));
//                   } else {
//                     lat_offset = std::max(
//                         std::max(lat_offset, -0.15 * lane_width -
//                         dist_rblane), -2.0 + avoid_car_info[0][5]);
//                   }

//                   if (lat_offset == -avoid_car_info[0][9]) {
//                     lat_offset = 0;
//                   }

//                   if (dist_rblane > 1.0) {
//                     sb_blane_ = true;
//                   }
//                   if (std::pow(avoid_car_info[0][2] - 1, 2) / 4 >
//                           avoid_car_info[0][3] - 2 &&
//                       avoid_car_info[0][3] > -5 && desired_poly_[3] < -0.7) {
//                     lat_offset = -1.3 + avoid_car_info[0][5];
//                     force_pause_ = true;
//                   }
//                 } else {
//                   lat_offset = 0;
//                 }
//               }
//             } else {
//               if (avoid_car_info[0][6] != 100) {
//                 lat_offset = 0.5 * (lane_width - car_width / 2 -
//                                     std::fabs(avoid_car_info[0][6])) +
//                              lat_compen1;

//                 if (avoid_car_info[0][6] >= 0 &&
//                     (avoid_car_info[0][2] + v_ego < 1.5 ||
//                      avoid_car_info[0][3] < 0) &&
//                     std::fabs(avoid_car_info[0][4]) < 0.45 &&
//                     ((map_info.current_lane_index() ==
//                       map_info.lanes_num() - 1) ||
//                      (map_info.current_lane_index() ==
//                           map_info.lanes_num() - 2 &&
//                       map_info_manager.rlane_.exist() &&
//                       map_info_manager.rlane_.type() ==
//                           MSD_LANE_TYPE_NON_MOTOR))) {
//                   if (!map_info.is_on_highway() &&
//                       map_info.dist_to_intsect() > 80 &&
//                       map_info.dist_to_intsect() - avoid_car_info[0][3] > 80
//                       &&
//                       ((avoid_car_info[0][3] < 15 && v_ego < 5) ||
//                        (v_ego < 10 && avoid_car_info[0][2] + v_ego < -1) ||
//                        avoid_car_info[0][3] < 1)) {
//                     lat_offset =
//                         avoid_car_info[0][5] -
//                         (avoid_car_info[0][5] + 2.3 + avoid_car_info[0][9]) /
//                         2;

//                     if (-lat_offset - avoid_car_info[0][9] > 0.4) {
//                       lat_offset =
//                           std::max(std::max(lat_offset,
//                                             -0.15 * lane_width -
//                                             dist_rblane),
//                                    std::max(-avoid_car_info[0][9],
//                                             -2.0 + avoid_car_info[0][5]));
//                     } else {
//                       lat_offset =
//                           std::max(std::max(lat_offset,
//                                             -0.15 * lane_width -
//                                             dist_rblane),
//                                    -2.0 + avoid_car_info[0][5]);
//                     }

//                     if (lat_offset == -avoid_car_info[0][9]) {
//                       lat_offset = 0;
//                     }

//                     if (dist_rblane > 1.0) {
//                       sb_blane_ = true;
//                     }
//                     if (std::pow(avoid_car_info[0][2] - 1, 2) / 4 >
//                             avoid_car_info[0][3] - 2 &&
//                         avoid_car_info[0][3] > -5 && desired_poly_[3] < -0.7)
//                         {
//                       lat_offset = -1.3 + avoid_car_info[0][5];
//                       force_pause_ = true;
//                     }
//                   } else {
//                     lat_offset = 0;
//                   }
//                 }
//               }
//             }

//             if (avoid_car_info[0][6] < 0 || avoid_car_info[1][6] < 0) {
//               lat_offset = std::max(lat_offset, 0.0);

//               if ((avoid_car_info[0][2] + v_ego < 0.5 ||
//                    avoid_car_info[0][3] < 1) &&
//                   std::fabs(avoid_car_info[0][4]) < 0.3 &&
//                   std::fabs(avoid_car_info[1][4]) < 0.3 &&
//                   ((map_info.current_lane_index() == map_info.lanes_num() - 1
//                   &&
//                     ((map_info_manager.clane_.type() !=
//                           MSD_LANE_TYPE_NON_MOTOR &&
//                       map_info.current_lane_index() >= 1) ||
//                      (map_info_manager.clane_.type() ==
//                           MSD_LANE_TYPE_NON_MOTOR &&
//                       map_info.current_lane_index() >= 2))) ||
//                    (map_info.current_lane_index() == map_info.lanes_num() - 2
//                    &&
//                     map_info_manager.rlane_.exist() &&
//                     map_info_manager.rlane_.type() == MSD_LANE_TYPE_NON_MOTOR
//                     && map_info.current_lane_index() >= 1))) {
//                 lat_offset = std::min(
//                     lat_offset,
//                     std::min(avoid_car_info[0][9], avoid_car_info[1][9]));
//                 if (lat_offset >= lane_width / 2 - 1.1) {
//                   large_lat_ = true;
//                 } else if (std::pow(avoid_car_info[0][2] - 1, 2) / 4 >
//                                avoid_car_info[0][3] - 2 &&
//                            avoid_car_info[0][3] > -10 &&
//                            desired_poly_[3] < -0.5) {
//                   lat_offset = 1.3 + avoid_car_info[0][6];
//                   force_pause_ = true;
//                 }
//               } else {
//                 if (avd_normal_threshold > 0) {
//                   lat_offset =
//                       std::min(std::min(lat_offset, avd_normal_threshold),
//                                std::min(avoid_car_info[0][9],
//                                avd_limit_right));
//                 } else {
//                   lat_offset =
//                       std::min(std::min(lat_offset, 0.5 * lane_width - 0.9),
//                                std::min(avoid_car_info[0][9],
//                                avd_limit_right));
//                 }
//               }
//             }

//             if ((map_info.current_lane_index() != map_info.lanes_num() - 1 &&
//                  map_info.current_lane_index() != 0) &&
//                 avoid_car_info[0][2] + v_ego < 0.5 &&
//                 map_info.dist_to_intsect() > 0 &&
//                 map_info.dist_to_intsect() - avoid_car_info[0][3] < 50 &&
//                 abs(avoid_car_info[0][4]) < 0.2 && !isRedLightStop &&
//                 ((avoid_car_info[0][6] >=
//                   (lane_width / 2 - (car_width + 0.3))) ||
//                  (avoid_car_info[1][6] >=
//                   (lane_width / 2 - (car_width + 0.3)))) &&
//                 avoid_car_info[0][6] < 0 && avoid_car_info[1][6] < 0) {
//               cross_left_solid_line_ = true;
//               lat_offset = 0.;
//             }
//             if ((avoid_car_info[0][7] == 20001 &&
//                  map_info.dist_to_intsect() - avoid_car_info[0][3] >= -5 &&
//                  map_info.dist_to_intsect() - avoid_car_info[0][3] < 50 &&
//                  (avoid_car_info[0][6] >=
//                   (lane_width / 2 - (car_width + 0.3)))) ||
//                 (avoid_car_info[1][7] == 20001 &&
//                  map_info.dist_to_intsect() - avoid_car_info[1][3] >= -5 &&
//                  map_info.dist_to_intsect() - avoid_car_info[1][3] < 50 &&
//                  (avoid_car_info[1][6] >=
//                   (lane_width / 2 - (car_width + 0.3))))) {
//               if (map_info.current_lane_index() != 0 ||
//                   f_refline.position() == RIGHT_POS) {
//                 cross_left_solid_line_ = true;
//               } else if (map_info.current_lane_index() !=
//                          map_info.lanes_num() - 1) {
//                 cross_right_solid_line_ = true;
//               }
//               lat_offset = 0.;
//             }

//             if (lat_offset < 0 && dist_rblane < 0.5 && sb_lane_ == false) {
//               lat_offset = -desired_poly_[3];
//             }

//             if (fix_lane.status() != LaneStatusEx::BOTH_MISSING &&
//                 left_poly_[3] > 0 && left_poly_[3] < 1.0 &&
//                 map_info.current_lane_index() == 0 && !special_lane_type &&
//                 !large_lat_ && !force_pause_) {
//               lat_offset = std::max(lat_offset + left_poly_[3] - 1.0, 0.0);
//             }
//           }
//         }

//         if (avoid_car_info[0][7] > 0.) {
//           pre_str_dist = interp(avoid_car_info[0][2], avd_vRel_bp,
//           avd_vRel_v); if (avoid_car_info[0][0] != -1 && avoid_car_info[0][0]
//           != -2) {
//             if (lat_offset >= 0) {
//               lat_offset = std::min((avoid_car_info[0][7] -
//                                      (avoid_car_info[0][3] - pre_str_dist)) /
//                                         avoid_car_info[0][7] * lat_offset,
//                                     lat_offset);
//             } else {
//               lat_offset = std::max((avoid_car_info[0][7] -
//                                      (avoid_car_info[0][3] - pre_str_dist)) /
//                                         avoid_car_info[0][7] * lat_offset,
//                                     lat_offset);
//             }
//           } else {
//             if (lat_offset >= 0.) {
//               lat_offset = std::min(
//                   (5. / std::max(std::fabs(avoid_car_info[0][3]), 5.)) *
//                       lat_offset,
//                   lat_offset);
//             } else {
//               lat_offset = std::max(
//                   (5. / std::max(std::fabs(avoid_car_info[0][3]), 5.)) *
//                       lat_offset,
//                   lat_offset);
//             }
//           }
//           if (lat_offset < 0. && desired_poly_[3] + lat_offset < -0.3) {
//             lat_offset = -desired_poly_[3] - 0.3;
//           } else if (lat_offset >= 0. && desired_poly_[3] + lat_offset > 0.3)
//           {
//             lat_offset = -desired_poly_[3] + 0.3;
//           }
//         }

//         if (dist_rblane == 0 && (lat_offset > 0.8 || lat_offset < -0.8)) {
//         } else {
//           if (std::fabs(lat_offset1) >= std::fabs(desired_poly_[3]) &&
//               ((desired_poly_[3] > 0 && lat_offset < 0 &&
//                 desired_poly_[3] + lat_offset > 0) ||
//                (desired_poly_[3] < 0 && lat_offset > 0 &&
//                 desired_poly_[3] + lat_offset < 0))) {
//             desired_poly_[3] = 0;
//           } else if (std::pow(avoid_car_info[0][2] - 1, 2) / 4 >
//                          avoid_car_info[0][3] - 2 &&
//                      avoid_car_info[0][3] > -5) {
//             if (avoid_car_info[0][5] > 0 &&
//                 std::fabs(lat_offset - avoid_car_info[0][5]) < 1.3) {
//               lat_offset = std::max(-1.3 + avoid_car_info[0][5],
//                                     -0.15 * lane_width - dist_rblane);
//               force_pause_ = true;
//             } else if (avoid_car_info[0][5] < 0 && avoid_car_info[0][6] < 0
//             &&
//                        std::fabs(lat_offset - avoid_car_info[0][6]) < 1.3) {
//               lat_offset = 1.3 + avoid_car_info[0][6];
//               if (map_info.current_lane_index() == 0) {
//                 lat_offset = std::min(1.3 + avoid_car_info[0][6],
//                                       0.5 * lane_width - 0.9);
//               }
//               force_pause_ = true;
//             } else {
//               desired_poly_[3] += lat_offset;
//               two_nudge_car = desired_poly_[3];  // ?
//             }
//           } else {
//             desired_poly_[3] += lat_offset;
//             two_nudge_car = desired_poly_[3];
//           }
//         }
//       } else if (avoid_car_info[0][5] > 0) {
//         if ((int)avoid_car_info[0][1] == -100) {
//           lat_offset =
//               -0.15 * lane_width * std::fabs(avoid_car_info[0][10]) / 3.2;

//           if (lane_type == 3 || std::fabs(desired_poly_[1]) > 0.0001 ||
//               (map_info.lanes_num() > 1 &&
//                map_info.current_lane_index() == map_info.lanes_num() - 1)) {
//             lat_offset = 0.8 * lat_offset;
//           }

//           if (avd_limit_left == 0.2) {
//             lat_offset = std::max(lat_offset, -avd_limit_left);
//           }

//           if (fix_lane.status() != LaneStatusEx::BOTH_MISSING &&
//               right_poly_[3] < 0 && right_poly_[3] > -1.0 &&
//               !special_lane_type) {
//             lat_offset += right_poly_[3] + 1.0;
//           }

//           if (desired_poly_[3] > 1.0) {
//             lat_offset = -desired_poly_[3] - 0.5;
//           }
//         } else if (((int)avoid_car_info[0][0] == 0 &&
//                     avoid_car_info[0][3] > -6.0) ||
//                    ((int)avoid_car_info[0][0] != 0 &&
//                     avoid_car_info[0][3] >= -3.0)) {
//           if (status != INTER_GS_NONE && status != INTER_TR_NONE &&
//               status != INTER_TL_NONE) {
//             if (avoid_car_info[0][5] != 100) {
//               lat_offset = 0.5 * (lane_width - car_width / 2 -
//                                   std::fabs(avoid_car_info[0][5])) +
//                            lat_compen1;
//             }

//             lat_offset = std::max(lat_offset, 0.0);
//             if (avd_normal_threshold > 0 && map_info.dist_to_intsect() > 0) {
//               lat_offset =
//                   -std::min(std::min(lat_offset, avd_normal_threshold),
//                             std::min(avoid_car_info[0][9], avd_limit_left));
//             } else {
//               lat_offset =
//                   -std::min(std::min(lat_offset, 0.5 * lane_width - 0.9),
//                             std::min(avoid_car_info[0][9], avd_limit_left));
//             }

//             if (avoid_car_info[0][5] < 1.5) {
//               if ((map_info.current_lane_index() != map_info.lanes_num() - 1
//               &&
//                    (!map_info_manager.rlane_.exist() ||
//                     (map_info_manager.rlane_.exist() &&
//                      map_info_manager.rlane_.type() !=
//                          MSD_LANE_TYPE_NON_MOTOR))) ||
//                   (avoid_car_info[0][2] + v_ego >= 1.5 &&
//                    avoid_car_info[0][3] >= 0) ||
//                   map_info.dist_to_intsect() <= 80 ||
//                   map_info.dist_to_intsect() - avoid_car_info[0][3] <= 80) {
//                 lat_offset =
//                     avoid_car_info[0][5] -
//                     (avoid_car_info[0][5] + 1.8 + avoid_car_info[0][9]) / 2;
//                 lat_offset = std::max(lat_offset, -0.5 * lane_width + 0.9);
//               } else if (!map_info.is_on_highway() &&
//                          abs(avoid_car_info[0][4]) < 0.45) {
//                 if ((avoid_car_info[0][3] < 15 && v_ego < 5) ||
//                     (v_ego < 10 && avoid_car_info[0][2] + v_ego < -1) ||
//                     avoid_car_info[0][3] < 1) {
//                   lat_offset =
//                       avoid_car_info[0][5] -
//                       (avoid_car_info[0][5] + 1.8 + avoid_car_info[0][9]) /
//                       2;

//                   lat_offset = std::max(
//                       lat_offset, std::max(-0.15 * lane_width - dist_rblane,
//                                            -2.0 + avoid_car_info[0][5]));
//                 }

//                 if (dist_rblane > 1.0 &&
//                     0.5 * lane_width + avoid_car_info[0][5] <
//                         3.0 + 0.02 * avoid_car_info[0][2]) {
//                   sb_blane_ = true;
//                 }
//                 if (std::pow(avoid_car_info[0][2] - 1, 2) / 4 >
//                         avoid_car_info[0][3] - 2 &&
//                     avoid_car_info[0][3] > -5 && desired_poly_[3] < -0.7) {
//                   lat_offset = -1.3 + avoid_car_info[0][5];
//                   force_pause_ = true;
//                 }
//               }

//               if (map_info.lanes_num() > 1 &&
//                   map_info.current_lane_index() == map_info.lanes_num() - 1
//                   && dist_rblane < 0.5) {
//                 lat_offset *= 0.8;
//               }
//             }

//             if (avd_limit_left == 0.2) {
//               lat_offset = std::max(lat_offset, -avd_limit_left);
//             }

//             if (fix_lane.status() != LaneStatusEx::BOTH_MISSING &&
//                 right_poly_[3] < 0 && right_poly_[3] > -1.0 &&
//                 (map_info.current_lane_index() == map_info.lanes_num() - 1 &&
//                  (dist_rblane < 0.5 || avd_limit_left == 0.2)) &&
//                 !special_lane_type) {
//               lat_offset = std::min(lat_offset + right_poly_[3] + 1.0, 0.0);
//             }
//           } else {
//             if (avoid_car_info[0][5] != 100) {
//               lat_offset = 0.5 * (lane_width - car_width / 2 -
//                                   std::fabs(avoid_car_info[0][5])) +
//                            lat_compen1;
//             }

//             lat_offset = std::max(lat_offset, 0.0);
//             lat_offset = -std::min(lat_offset, std::min(0.5 * lane_width -
//             0.9,
//                                                         avoid_car_info[0][9]));

//             if (avoid_car_info[0][5] < 1.5) {
//               lat_offset =
//                   avoid_car_info[0][5] -
//                   (avoid_car_info[0][5] + 1.8 + avoid_car_info[0][9]) / 2;

//               if (avoid_car_info[0][5] >= 1.1) {
//                 lat_offset = std::max(lat_offset, -0.5 * lane_width + 0.9);
//               } else if (lane_width <= 3.8) {
//                 if ((avoid_car_info[0][2] + v_ego < 1.5 ||
//                      avoid_car_info[0][3] < 0) &&
//                     avoid_car_info[0][4] > -0.5 && avoid_car_info[0][4] < 0.3
//                     && avoid_car_info[0][1] == 0) {
//                   lat_offset =
//                       std::max(lat_offset, -1.6 + avoid_car_info[0][5]);
//                   sb_lane_ = true;
//                 } else {
//                   if (map_info.dist_to_last_intsect() > -10) {
//                     lat_offset = std::max(lat_offset, -0.5 * lane_width +
//                     0.9);
//                   } else {
//                     if (avd_normal_threshold > 0 &&
//                         map_info.dist_to_intsect() > 0) {
//                       lat_offset = std::max(lat_offset,
//                       -avd_normal_threshold);
//                     } else {
//                       lat_offset =
//                           std::max(lat_offset, -0.5 * lane_width + 0.9);
//                     }
//                   }
//                 }
//               } else {
//                 lat_offset =
//                     std::max(lat_offset, -0.24 * lane_width / 4.4 *
//                     lane_width);
//                 if ((avoid_car_info[0][2] + v_ego < 1.5 ||
//                      avoid_car_info[0][3] < 0) &&
//                     avoid_car_info[0][4] > -0.5 && avoid_car_info[0][4] < 0.3
//                     && avoid_car_info[0][1] == 0) {
//                   lat_offset =
//                       std::max(lat_offset, -1.8 + avoid_car_info[0][5]);
//                   sb_lane_ = true;
//                 }
//               }
//             }
//           }

//           if ((map_info.current_lane_index() != map_info.lanes_num() - 1) &&
//               avoid_car_info[0][2] + v_ego < 0.5 &&
//               map_info.dist_to_intsect() > 0 &&
//               map_info.dist_to_intsect() - avoid_car_info[0][3] < 50 &&
//               abs(avoid_car_info[0][4]) < 0.2 && !isRedLightStop &&
//               (avoid_car_info[0][5] <= ((car_width + 0.3) - lane_width / 2)))
//               {
//             cross_right_solid_line_ = true;
//             lat_offset = 0.;
//           }
//           if (avoid_car_info[0][7] == 20001 &&
//               map_info.dist_to_intsect() - avoid_car_info[0][3] >= -5 &&
//               map_info.dist_to_intsect() - avoid_car_info[0][3] < 50 &&
//               (avoid_car_info[0][5] <= ((car_width + 0.3) - lane_width / 2)))
//               {
//             if (map_info.current_lane_index() != map_info.lanes_num() - 1 ||
//                 dist_rblane > 1.5) {
//               cross_right_solid_line_ = true;
//             } else if (map_info.current_lane_index() != 0 ||
//                        f_refline.position() == RIGHT_POS) {
//               cross_left_solid_line_ = true;
//             }
//             lat_offset = 0.;
//           }

//           lat_offsetl = lat_offset;

//           pre_str_dist = interp(avoid_car_info[0][2], avd_vRel_bp,
//           avd_vRel_v); if (avoid_car_info[0][0] > 0 && avoid_car_info[0][7]
//           != 0.) {
//             if (avoid_car_info[0][7] >= avoid_car_info[0][3]) {
//               lat_offset = std::max((avoid_car_info[0][7] -
//                                      (avoid_car_info[0][3] - pre_str_dist)) /
//                                         avoid_car_info[0][7] * lat_offset,
//                                     lat_offset);
//             } else if (avoid_car_info[0][3] != 0.) {
//               lat_offset = (1 - avoid_car_info[0][7] / avoid_car_info[0][3])
//               *
//                            lat_offset;
//             }
//           } else {
//             if (avoid_car_info[0][5] < 1.5) {
//               lat_offset =
//                   std::max((5. /
//                   std::max(std::abs(avoid_car_info[0][3]), 5.)) *
//                                lat_offset,
//                            lat_offset);
//             } else {
//               lat_offset =
//                   std::max((5. /
//                   std::max(std::abs(avoid_car_info[0][3]), 5.)) *
//                                lat_offset,
//                            lat_offset);
//             }
//           }
//           if (desired_poly_[3] + lat_offset < -0.3) {
//             lat_offset = -desired_poly_[3] - 0.3;
//           }
//         } else {
//           if (map_info.dist_to_intsect() > 0) {
//             lat_offset = std::min(-desired_poly_[3], 0.);
//           } else {
//             lat_offset = std::min(
//                 std::max(avoid_car_info[0][5] - 1.6,
//                          avoid_car_info[0][5] - (avoid_car_info[0][5] + 1.8 +
//                                                  avoid_car_info[0][9]) /
//                                                     2),
//                 0.0);
//           }

//           if (std::fabs(lat_offset) > 0.5 * lane_width - 0.9 &&
//               dist_rblane < 0.5) {
//             lat_offset = -0.5 * lane_width + 0.9;
//           }
//         }

//         if ((dist_rblane == 0 || sb_lane_ == false) &&
//             (lat_offset < -0.8 || lat_offset > 0.8)) {
//         } else {
//           if (std::fabs(lat_offsetl) >= std::fabs(desired_poly_[3]) &&
//               desired_poly_[3] > 0 && desired_poly_[3] + lat_offset > 0) {
//             desired_poly_[3] = 0;
//           } else if (std::pow(avoid_car_info[0][2] - 1, 2) / 4 >
//                          avoid_car_info[0][3] - 2 &&
//                      avoid_car_info[0][3] > -5) {
//             if (avoid_car_info[0][5] > 0 &&
//                 std::fabs(lat_offset - avoid_car_info[0][5]) < 1.3) {
//               lat_offset = std::max(-1.3 + avoid_car_info[0][5],
//                                     -0.15 * lane_width - dist_rblane);
//               force_pause_ = true;
//             } else {
//               desired_poly_[3] += lat_offset;
//               two_nudge_car = desired_poly_[3];
//             }
//           } else {
//             desired_poly_[3] += lat_offset;
//           }

//           one_nudge_left_car = desired_poly_[3];
//         }
//       } else {
//         if ((int)avoid_car_info[0][1] == -200) {
//           lat_offset =
//               0.15 * lane_width * std::fabs(avoid_car_info[0][10]) / 3.2;

//           if (lane_type == MSD_LANE_TYPE_PARKING ||
//               std::fabs(desired_poly_[1]) > 0.0001 ||
//               (map_info.lanes_num() > 1 &&
//                map_info.current_lane_index() == 0)) {
//             lat_offset *= 0.8;
//           }

//           if (avd_limit_right == 0.2) {
//             lat_offset = std::min(lat_offset, avd_limit_right);
//           }

//           if (fix_lane.status() != LaneStatusEx::BOTH_MISSING &&
//               left_poly_[3] > 0 && left_poly_[3] < 1.0 && !special_lane_type)
//               {
//             lat_offset += left_poly_[3] - 1.0;
//           }
//         } else if (((int)avoid_car_info[0][0] == 0 &&
//                     avoid_car_info[0][3] > -6.0) ||
//                    ((int)avoid_car_info[0][0] != 0 &&
//                     avoid_car_info[0][3] >= -3.0)) {
//           if (status != INTER_GS_NONE && status != INTER_TR_NONE &&
//               status != INTER_TL_NONE) {
//             if (avoid_car_info[0][6] != 100) {
//               lat_offset = 0.5 * (lane_width - car_width / 2 -
//                                   std::fabs(avoid_car_info[0][6])) +
//                            lat_compen1;
//             }

//             lat_offset = std::max(lat_offset, 0.0);

//             if (avd_normal_threshold > 0) {
//               lat_offset =
//                   std::min(std::min(lat_offset, avd_normal_threshold),
//                            std::min(avoid_car_info[0][9], avd_limit_right));
//             } else {
//               lat_offset =
//                   std::min(std::min(lat_offset, 0.5 * lane_width - 0.9),
//                            std::min(avoid_car_info[0][9], avd_limit_right));
//             }

//             if (avoid_car_info[0][6] <
//                     -0.9 + std::max(lane_width / 2 - 1.8, 0.0) &&
//                 avoid_car_info[0][6] > -1.5) {
//               lat_offset =
//                   avoid_car_info[0][6] -
//                   (avoid_car_info[0][6] - 1.8 - avoid_car_info[0][9]) / 2;
//               lat_offset = std::min(lat_offset, 0.15 * lane_width);

//               if (map_info.lanes_num() > 1 &&
//                   map_info.current_lane_index() == 0) {
//                 lat_offset *= 0.8;
//               }
//             } else if (avoid_car_info[0][6] >= 0 &&
//                        (avoid_car_info[0][2] + v_ego < 1.5 ||
//                         avoid_car_info[0][3] < 1) &&
//                        std::fabs(avoid_car_info[0][4]) < 0.5 &&
//                        ((map_info.current_lane_index() ==
//                          map_info.lanes_num() - 1) ||
//                         (map_info.current_lane_index() ==
//                              map_info.lanes_num() - 2 &&
//                          map_info_manager.rlane_.exist() &&
//                          map_info_manager.rlane_.type() ==
//                              MSD_LANE_TYPE_NON_MOTOR))) {
//               if (((!map_info.is_on_highway() &&
//                     map_info.dist_to_intsect() > 80 &&
//                     map_info.dist_to_intsect() - avoid_car_info[0][3] > 80)
//                     ||
//                    (desired_poly_[3] > 1 && avoid_car_info[0][3] < 3)) &&
//                   ((avoid_car_info[0][3] < 15 && v_ego < 5) ||
//                    (v_ego < 10 && avoid_car_info[0][2] + v_ego < -1) ||
//                    avoid_car_info[0][3] < 1)) {
//                 lat_offset =
//                     avoid_car_info[0][5] -
//                     (avoid_car_info[0][5] + 1.8 + avoid_car_info[0][9]) / 2;

//                 if (-lat_offset - avoid_car_info[0][9] > 0.4) {
//                   lat_offset = std::max(
//                       std::max(lat_offset, -0.15 * lane_width - dist_rblane),
//                       std::max(-avoid_car_info[0][9],
//                                -2.0 + avoid_car_info[0][5]));
//                 } else {
//                   lat_offset = std::max(
//                       lat_offset, std::max(-0.15 * lane_width - dist_rblane,
//                                            -2.0 + avoid_car_info[0][5]));
//                 }

//                 if (lat_offset == -avoid_car_info[0][9]) {
//                   lat_offset = 0;
//                 }

//                 if (dist_rblane > 1.0) {
//                   sb_blane_ = true;
//                 }
//                 if (std::pow(avoid_car_info[0][2] - 1, 2) / 4 >
//                         avoid_car_info[0][3] - 2 &&
//                     avoid_car_info[0][3] > -5 && desired_poly_[3] < -0.7) {
//                   lat_offset = -1.3 + avoid_car_info[0][5];
//                   force_pause_ = true;
//                 }
//               } else {
//                 lat_offset = 0;
//               }
//             } else if (avoid_car_info[0][6] >=
//                            -0.9 + std::max(lane_width / 2 - 1.8, 0.0) &&
//                        avoid_car_info[0][6] < 0 &&
//                        (avoid_car_info[0][2] + v_ego < 0.5 ||
//                         avoid_car_info[0][3] < 1) &&
//                        std::fabs(avoid_car_info[0][4]) < 0.5 &&
//                        ((map_info.current_lane_index() ==
//                              map_info.lanes_num() - 1 &&
//                          ((map_info_manager.clane_.type() !=
//                                MSD_LANE_TYPE_NON_MOTOR &&
//                            map_info.current_lane_index() >= 1) ||
//                           (map_info_manager.clane_.type() ==
//                                MSD_LANE_TYPE_NON_MOTOR &&
//                            map_info.current_lane_index() >= 2))) ||
//                         (map_info.current_lane_index() ==
//                              map_info.lanes_num() - 2 &&
//                          map_info_manager.rlane_.exist() &&
//                          map_info_manager.rlane_.type() ==
//                              MSD_LANE_TYPE_NON_MOTOR &&
//                          map_info.current_lane_index() >= 1))) {
//               lat_offset =
//                   std::min(1.5 + avoid_car_info[0][6], avoid_car_info[0][9]);

//               if (lat_offset >= lane_width / 2 - 1.1) {
//                 large_lat_ = true;
//               } else if (std::pow(avoid_car_info[0][2] - 1, 2) / 4 >
//                              avoid_car_info[0][3] - 2 &&
//                          avoid_car_info[0][3] > -10 &&
//                          desired_poly_[3] < -0.5) {
//                 lat_offset = 1.3 + avoid_car_info[0][6];
//                 force_pause_ = true;
//               }
//             }

//             if (avd_limit_right == 0.2) {
//               lat_offset = std::min(lat_offset, avd_limit_right);
//             }

//             if (fix_lane.status() != LaneStatusEx::BOTH_MISSING &&
//                 left_poly_[3] > 0 && left_poly_[3] < 1.0 &&
//                 map_info.current_lane_index() == 0 && !special_lane_type &&
//                 !large_lat_ && !force_pause_) {
//               lat_offset = std::max(lat_offset + left_poly_[3] - 1.0, 0.0);
//             }
//           } else {
//             if (avoid_car_info[0][6] != 100) {
//               lat_offset = 0.5 * (lane_width - car_width / 2 -
//                                   std::fabs(avoid_car_info[0][6])) +
//                            lat_compen1;
//             }

//             lat_offset = std::max(lat_offset, 0.0);
//             lat_offset = std::min(lat_offset, std::min(0.5 * lane_width -
//             0.9,
//                                                        avoid_car_info[0][9]));

//             if (avoid_car_info[0][6] > -1.5) {
//               lat_offset =
//                   avoid_car_info[0][6] -
//                   (avoid_car_info[0][6] - 1.8 - avoid_car_info[0][9]) / 2;

//               if (avoid_car_info[0][6] < -1.1) {
//                 if (map_info.left_refline_points().size() != 0 ||
//                     map_info.dist_to_last_intsect() > 20) {
//                   lat_offset = std::min(lat_offset, 0.5 * lane_width - 0.9);
//                 } else {
//                   if (avd_normal_threshold > 0) {
//                     lat_offset = std::min(lat_offset, avd_normal_threshold);
//                   } else {
//                     lat_offset = std::min(lat_offset, 0.5 * lane_width -
//                     0.9);
//                   }
//                 }
//               } else if (lane_width <= 3.8) {
//                 if (avoid_car_info[0][6] > 0 && avoid_car_info[0][5] > -0.65
//                 &&
//                     (avoid_car_info[0][2] + v_ego < 1.5 ||
//                      avoid_car_info[0][3] < 0) &&
//                     avoid_car_info[0][4] > -0.5 && avoid_car_info[0][4] < 0.3
//                     && avoid_car_info[0][1] == 0) {
//                   lat_offset =
//                       avoid_car_info[0][5] -
//                       (avoid_car_info[0][5] + 1.8 + avoid_car_info[0][9]) /
//                       2;

//                   lat_offset =
//                       std::max(lat_offset, -1.6 + avoid_car_info[0][5]);
//                   sb_lane_ = true;
//                 } else {
//                   if (map_info.dist_to_last_intsect() > -10) {
//                     lat_offset = std::min(
//                         lat_offset, std::min(0.5 * lane_width - 0.9, 0.7));
//                   } else {
//                     if (avd_normal_threshold > 0) {
//                       lat_offset = std::min(lat_offset,
//                       avd_normal_threshold);
//                     } else {
//                       lat_offset = std::min(lat_offset, 0.5 * lane_width -
//                       0.9);
//                     }
//                   }
//                 }
//               } else {
//                 lat_offset =
//                     std::min(lat_offset, 0.24 * lane_width / 4.4 *
//                     lane_width);

//                 if (avoid_car_info[0][6] > 0 && avoid_car_info[0][5] > -0.65
//                 &&
//                     (avoid_car_info[0][2] + v_ego < 1.5 ||
//                      avoid_car_info[0][3] < 0) &&
//                     avoid_car_info[0][4] > -0.5 && avoid_car_info[0][4] < 0.3
//                     && avoid_car_info[0][1] == 0) {
//                   lat_offset =
//                       avoid_car_info[0][5] -
//                       (avoid_car_info[0][5] + 1.8 + avoid_car_info[0][9]) /
//                       2;
//                   lat_offset =
//                       std::max(lat_offset, -1.8 + avoid_car_info[0][5]);
//                   sb_lane_ = true;
//                 }
//               }
//             }
//           }

//           if ((map_info.current_lane_index() != map_info.lanes_num() - 1 &&
//                map_info.current_lane_index() != 0) &&
//               avoid_car_info[0][2] + v_ego < 0.5 &&
//               map_info.dist_to_intsect() > 0 &&
//               map_info.dist_to_intsect() - avoid_car_info[0][3] < 50 &&
//               abs(avoid_car_info[0][4]) < 0.2 && !isRedLightStop &&
//               (avoid_car_info[0][6] >= (lane_width / 2 - (car_width + 0.3)))
//               && avoid_car_info[0][6] < 0) {
//             cross_left_solid_line_ = true;
//             lat_offset = 0.;
//           }
//           if (avoid_car_info[0][7] == 20001 &&
//               map_info.dist_to_intsect() - avoid_car_info[0][3] >= -5 &&
//               map_info.dist_to_intsect() - avoid_car_info[0][3] < 50 &&
//               (avoid_car_info[0][6] >= (lane_width / 2 - (car_width + 0.3))))
//               {
//             if (map_info.current_lane_index() != 0 ||
//                 f_refline.position() == RIGHT_POS) {
//               cross_left_solid_line_ = true;
//             } else if (map_info.current_lane_index() !=
//                        map_info.lanes_num() - 1) {
//               cross_right_solid_line_ = true;
//             }
//             lat_offset = 0.;
//           }

//           lat_offsetr = lat_offset;

//           pre_str_dist = interp(avoid_car_info[0][2], avd_vRel_bp,
//           avd_vRel_v); if (avoid_car_info[0][0] > 0 && avoid_car_info[0][7]
//           != 0.) {
//             if (avoid_car_info[0][7] >= avoid_car_info[0][3]) {
//               lat_offset = std::min((avoid_car_info[0][7] -
//                                      (avoid_car_info[0][3] - pre_str_dist)) /
//                                         avoid_car_info[0][7] * lat_offset,
//                                     lat_offset);
//             } else if (avoid_car_info[0][3] != 0.) {
//               lat_offset = (1 - avoid_car_info[0][7] / avoid_car_info[0][3])
//               *
//                            lat_offset;
//             }
//           } else {
//             if (avoid_car_info[0][5] < 1.5) {
//               lat_offset =
//                   std::max((5. /
//                   std::max(std::abs(avoid_car_info[0][3]), 5.)) *
//                                lat_offset,
//                            lat_offset);
//             } else {
//               lat_offset =
//                   std::max((5. /
//                   std::max(std::abs(avoid_car_info[0][3]), 5.)) *
//                                lat_offset,
//                            lat_offset);
//             }
//           }
//           if (desired_poly_[3] + lat_offset > 0.3) {
//             lat_offset = -desired_poly_[3] + 0.3;
//           }
//         } else {
//           if (map_info.dist_to_intsect() > 0) {
//             lat_offset = std::max(-desired_poly_[3], 0.);
//           } else {
//             lat_offset = std::max(
//                 std::min(1.6 + avoid_car_info[0][6],
//                          avoid_car_info[0][6] - (avoid_car_info[0][6] - 1.8 -
//                                                  avoid_car_info[0][9]) /
//                                                     2),
//                 0.0);
//           }

//           if (std::fabs(lat_offset) > 0.5 * lane_width - 0.9 &&
//               avoid_car_info[0][6] < 0) {
//             lat_offset = 0.5 * lane_width - 0.9;
//           }
//         }

//         if (dist_rblane == 0 && (lat_offset > 0.8 || lat_offset < -0.8)) {
//         } else {
//           if (std::fabs(lat_offsetr) >= std::fabs(desired_poly_[3]) &&
//               desired_poly_[3] < 0 && desired_poly_[3] + lat_offset < 0) {
//             desired_poly_[3] = 0;
//           } else if (std::pow(avoid_car_info[0][2] - 1, 2) / 4 >
//                          avoid_car_info[0][3] - 2 &&
//                      avoid_car_info[0][3] > -5) {
//             if (avoid_car_info[0][5] < 0 && avoid_car_info[0][6] < 0 &&
//                 std::fabs(lat_offset - avoid_car_info[0][6]) < 1.3) {
//               lat_offset = 1.3 + avoid_car_info[0][6];

//               if (map_info.current_lane_index() == 0) {
//                 lat_offset = std::min(1.3 + avoid_car_info[0][6],
//                                       0.5 * lane_width - 0.9);
//               }

//               force_pause_ = true;
//             } else {
//               desired_poly_[3] += lat_offset;
//               two_nudge_car = desired_poly_[3];
//             }
//           } else {
//             desired_poly_[3] += lat_offset;
//           }

//           one_nudge_right_car = desired_poly_[3];
//         }
//       }
//     } else if (flag_avoid == 1 && lane_status != ROAD_LB_LBACK &&
//                lane_status != ROAD_LB_RBACK && lane_status != ROAD_LB_LRETURN
//                && lane_status != ROAD_LB_RRETURN && lane_status !=
//                ROAD_LC_LCHANGE && lane_status != ROAD_LC_RCHANGE) {
//       double path_gap = 0.2;
//       std::array<double, 2> xp1{0, 0.02};
//       std::array<double, 2> xp2{0, 0.003};
//       std::array<double, 2> fp1{1, 5};
//       std::array<double, 2> fp2{1, 3};

//       double min_factor =
//           std::max(interp(std::fabs(desired_poly_[2]), xp1, fp1),
//                    interp(std::fabs(desired_poly_[1]), xp2, fp1));
//       double max_factor =
//           std::max(interp(std::fabs(desired_poly_[2]), xp1, fp2),
//                    interp(std::fabs(desired_poly_[1]), xp2, fp2));

//       if (desired_poly_[3] < -min_factor * path_gap) {
//         desired_poly_[3] = -min_factor * path_gap;
//       } else if (desired_poly_[3] > max_factor * path_gap) {
//         desired_poly_[3] = max_factor * path_gap;
//       }
//     }

//     lane_borrow_suspend_cnt_ = 0;
//     suspend_lat_offset_ = 0;

//   } else if (status == ROAD_LB_LBORROW || status == ROAD_LB_RBORROW) {
//     if (avoid_sp_car_info[0].size() > 0) {
//       double plus1 =
//           interp(avoid_sp_car_info[0][2], near_car_vrel_bp, near_car_vrel_v);
//       double plus1_rel =
//           interp(avoid_sp_car_info[0][3], near_car_drel_bp, near_car_drel_v);
//       double lat_compen1 = 0.5 * plus1 + 0.5 * plus1_rel;

//       if (avoid_sp_car_info[1].size() > 0) {
//         double v_near_car2 = v_ego + avoid_sp_car_info[1][2];
//         double desired_dist2 = dist_offset + v_near_car2 * t_gap;
//         double diff_dist_nudge_car =
//             avoid_sp_car_info[1][3] - avoid_sp_car_info[0][3];
//         double dist_avd_nudge_car1 =
//             avoid_sp_car_info[0][3] + 5.0 + safety_dist;

//         double t_avdcar1 = 0;
//         if (avoid_sp_car_info[0][2] != 0) {
//           if (avoid_sp_car_info[0][2] < 0) {
//             t_avdcar1 = -dist_avd_nudge_car1 / avoid_sp_car_info[0][2];
//           } else {
//             t_avdcar1 = 5;
//           }
//         } else {
//           t_avdcar1 = 0;
//         }

//         if (t_avdcar1 > 0) {
//           diff_dist_nudge_car += diff_dist_nudge_car * t_avdcar1;
//           if ((int)avoid_sp_car_info[0][9] == RIGHT_CHANGE &&
//               (int)avoid_sp_car_info[1][9] == LEFT_CHANGE) {
//             if (diff_dist_nudge_car <
//                 desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego) {
//               if (avoid_sp_car_info[0][2] <= avoid_sp_car_info[1][2] ||
//                   avoid_sp_car_info[0][2] - avoid_sp_car_info[1][2] < 2) {
//                 if (avoid_sp_car_info[0][5] != 100) {
//                   lat_offset = 0.9 * (lane_width - car_width / 2 -
//                                       std::fabs(avoid_sp_car_info[0][5])) +
//                                lat_compen1;
//                 }

//                 lat_offset = -std::max(lat_offset, 0.0);
//                 if (desired_poly_[3] < 0) {
//                   lat_offset += lane_width;
//                   desired_poly_[3] += lat_offset;
//                 }
//               } else {
//                 if (avoid_sp_car_info[1][6] != 100) {
//                   lat_offset = 0.9 * (lane_width - car_width / 2 -
//                                       std::fabs(avoid_sp_car_info[1][6])) +
//                                lat_compen1;
//                 }

//                 lat_offset = std::max(lat_offset, 0.0);
//                 if (desired_poly_[3] > 0) {
//                   lat_offset = -(lane_width - lat_offset);
//                   desired_poly_[3] += lat_offset;
//                 }
//               }
//             } else {
//               lat_offset = 0.9 * (lane_width - car_width / 2 -
//                                   std::fabs(avoid_sp_car_info[0][5])) +
//                            lat_compen1;

//               lat_offset = -std::max(lat_offset, 0.0);
//               if (lane_type == MSD_LANE_TYPE_PARKING ||
//                   std::fabs(desired_poly_[1]) > 0.0001) {
//                 lat_offset *= 0.7;
//               }
//             }
//           } else if ((int)avoid_sp_car_info[0][9] == LEFT_CHANGE &&
//                      (int)avoid_sp_car_info[1][9] == RIGHT_CHANGE) {
//             if (diff_dist_nudge_car <
//                 desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego) {
//               if (avoid_sp_car_info[0][2] <= avoid_sp_car_info[1][2] ||
//                   avoid_sp_car_info[0][2] - avoid_sp_car_info[1][2] < 2) {
//                 if (avoid_sp_car_info[0][6] != 100) {
//                   lat_offset = 0.9 * (lane_width - car_width / 2 -
//                                       std::fabs(avoid_sp_car_info[0][6])) +
//                                lat_compen1;
//                 }

//                 lat_offset = std::max(lat_offset, 0.0);

//                 if (desired_poly_[3] > 0) {
//                   lat_offset = -(lane_width - lat_offset);
//                   desired_poly_[3] += lat_offset;
//                 }
//               } else {
//                 if (avoid_sp_car_info[1][5] != 100) {
//                   lat_offset = 0.9 * (lane_width - car_width -
//                                       std::fabs(avoid_sp_car_info[1][5])) +
//                                lat_compen1;
//                 }

//                 lat_offset = -std::max(lat_offset, 0.0);

//                 if (desired_poly_[3] < 0) {
//                   lat_offset += lane_width;
//                   desired_poly_[3] += lat_offset;
//                 }
//               }
//             } else {
//               lat_offset = 0.9 * (lane_width - car_width / 2 -
//                                   std::fabs(avoid_sp_car_info[0][6])) +
//                            lat_compen1;

//               lat_offset = std::max(lat_offset, 0.0);

//               if (lane_type == MSD_LANE_TYPE_PARKING ||
//                   std::fabs(desired_poly_[1]) > 0.0001) {
//                 lat_offset *= 0.7;
//               }

//               if (desired_poly_[3] > 0) {
//                 lat_offset = -(lane_width - lat_offset);
//                 desired_poly_[3] += lane_width - lat_offset;
//               }
//             }
//           } else if ((int)avoid_sp_car_info[0][9] == RIGHT_CHANGE &&
//                      (int)avoid_sp_car_info[1][9] == RIGHT_CHANGE) {
//             if (diff_dist_nudge_car <
//                 desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego) {
//               if (avoid_sp_car_info[0][5] != 100 ||
//                   avoid_sp_car_info[1][5] != 100) {
//                 if (std::fabs(avoid_sp_car_info[0][5]) < 1.9 ||
//                     std::fabs(avoid_sp_car_info[1][5]) < 1.9) {
//                   lat_offset = 0.9 * (lane_width - car_width / 2 -
//                                       std::min(avoid_sp_car_info[0][5],
//                                                avoid_sp_car_info[1][5])) +
//                                lat_compen1;
//                 } else {
//                   if (avoid_sp_car_info[0][5] >= 1.9 ||
//                       avoid_sp_car_info[1][5] >= 1.9) {
//                     lat_offset = 0.9 * (lane_width - car_width / 2 -
//                                         std::min(avoid_sp_car_info[0][5],
//                                                  avoid_sp_car_info[1][5]) +
//                                         lane_width) +
//                                  lat_compen1;
//                   }
//                 }
//               }
//             } else {
//               if (avoid_sp_car_info[0][5] != 100) {
//                 if (std::fabs(avoid_sp_car_info[0][5]) < 1.9) {
//                   lat_offset = 0.9 * (lane_width - car_width / 2 -
//                                       avoid_sp_car_info[0][5]) +
//                                lat_compen1;
//                 } else {
//                   if (avoid_sp_car_info[0][5] >= 1.9) {
//                     lat_offset = 0.9 * (lane_width - car_width / 2 -
//                                         avoid_sp_car_info[0][5] + lane_width)
//                                         +
//                                  lat_compen1;
//                   }
//                 }
//               }
//             }

//             lat_offset = -std::max(lat_offset, 0.0);
//             lat_offset = std::max(-1.2 * lane_width, lat_offset);
//             desired_poly_[3] += lat_offset;
//           } else if ((int)avoid_sp_car_info[0][9] == LEFT_CHANGE &&
//                      (int)avoid_sp_car_info[1][9] == LEFT_CHANGE) {
//             if (diff_dist_nudge_car <
//                 desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego) {
//               if (avoid_sp_car_info[0][6] != 100 &&
//                   avoid_sp_car_info[1][6] != 100) {
//                 if (std::fabs(avoid_sp_car_info[0][6]) < 1.9 ||
//                     std::fabs(avoid_sp_car_info[1][6]) < 1.9) {
//                   lat_offset = 0.9 * (lane_width - car_width / 2 +
//                                       std::max(avoid_sp_car_info[0][6],
//                                                avoid_sp_car_info[1][6]) +
//                                       lane_width) +
//                                lat_compen1;
//                 } else {
//                   if (avoid_sp_car_info[0][6] <= -1.9 ||
//                       avoid_sp_car_info[1][6] <= -1.9) {
//                     lat_offset = 0.9 * (lane_width - car_width / 2 +
//                                         std::max(avoid_sp_car_info[0][6],
//                                                  avoid_sp_car_info[1][6]) +
//                                         lane_width) +
//                                  lat_compen1;
//                   }
//                 }
//               } else if (avoid_sp_car_info[0][6] != 100) {
//                 if (std::fabs(avoid_sp_car_info[0][6]) < 1.9) {
//                   lat_offset = 0.9 * (lane_width - car_width / 2 +
//                                       avoid_sp_car_info[0][6]) +
//                                lat_compen1;
//                 } else {
//                   if (avoid_sp_car_info[0][6] <= -1.9) {
//                     lat_offset = 0.9 * (lane_width - car_width / 2 +
//                                         avoid_sp_car_info[0][6] + lane_width)
//                                         +
//                                  lat_compen1;
//                   }
//                 }
//               } else if (avoid_sp_car_info[1][6] != 100) {
//                 if (std::fabs(avoid_sp_car_info[1][6]) < 1.9) {
//                   lat_offset = 0.9 * (lane_width - car_width / 2 +
//                                       avoid_sp_car_info[1][6]) +
//                                lat_compen1;
//                 } else {
//                   if (avoid_sp_car_info[1][6] <= -1.9) {
//                     lat_offset = 0.9 * (lane_width - car_width / 2 +
//                                         avoid_sp_car_info[1][6] + lane_width)
//                                         +
//                                  lat_compen1;
//                   }
//                 }
//               }
//             } else {
//               if (avoid_sp_car_info[0][6] != 100) {
//                 if (std::fabs(avoid_sp_car_info[0][6]) < 1.9) {
//                   lat_offset = 0.9 * (lane_width - car_width / 2 +
//                                       avoid_sp_car_info[0][6]) +
//                                lat_compen1;
//                 } else {
//                   if (avoid_sp_car_info[0][6] <= -1.9) {
//                     lat_offset = 0.9 * (lane_width - car_width / 2 +
//                                         avoid_sp_car_info[0][6] + lane_width)
//                                         +
//                                  lat_compen1;
//                   }
//                 }
//               }
//             }

//             lat_offset = std::max(lat_offset, 0.0);
//             lat_offset = std::min(lat_offset, 1.2 * lane_width);
//             desired_poly_[3] += lat_offset;
//           }
//         }
//       } else if ((int)avoid_sp_car_info[0][9] == RIGHT_CHANGE) {
//         if (avoid_sp_car_info[0][5] != 100) {
//           if (std::fabs(avoid_sp_car_info[0][5]) < 1.9) {
//             lat_offset =
//                 0.9 * (lane_width - car_width / 2 - avoid_sp_car_info[0][5])
//                 + lat_compen1;
//           } else {
//             if (avoid_sp_car_info[0][5] >= 1.9) {
//               lat_offset =
//                   0.9 * (lane_width - car_width / 2 -
//                   avoid_sp_car_info[0][5]) + lat_compen1;
//             }
//           }

//           lat_offset = -std::max(lat_offset, 0.0);
//           lat_offset = std::max(-1.2 * lane_width, lat_offset);
//         } else {
//           lat_offset = 0;
//         }

//         desired_poly_[3] += lat_offset;
//       } else {
//         if (avoid_sp_car_info[0][6] != 100) {
//           if (std::fabs(avoid_sp_car_info[0][6]) < 1.9) {
//             lat_offset =
//                 0.9 * (lane_width - car_width / 2 + avoid_sp_car_info[0][6])
//                 + lat_compen1;
//           } else {
//             if (avoid_sp_car_info[0][6] <= -1.9) {
//               lat_offset =
//                   0.9 * (lane_width - car_width / 2 +
//                   avoid_sp_car_info[0][6]) + lat_compen1;
//             }
//           }

//           lat_offset = std::max(lat_offset, 0.0);
//           lat_offset = std::min(lat_offset, 1.2 * lane_width);
//         } else {
//           lat_offset = 0;
//         }

//         desired_poly_[3] += lat_offset;
//       }
//     }

//     lane_borrow_suspend_cnt_ = 0;
//     suspend_lat_offset_ = 0;

//   } else if (lane_status == ROAD_LB_LSUSPEND ||
//              lane_status == ROAD_LB_RSUSPEND) {
//     if (lane_borrow_suspend_cnt_ == 0) {
//       suspend_lat_offset_ = desired_poly_[3];
//       lat_offset = -suspend_lat_offset_;
//       lane_borrow_suspend_cnt_ += 1;
//     }

//     lat_offset = -suspend_lat_offset_;
//     desired_poly_[3] -= suspend_lat_offset_;
//   }

//   two_nudge_car_ = two_nudge_car;  // some problem need to be solved
//   one_nudge_left_car_ = one_nudge_left_car;
//   one_nudge_right_car_ = one_nudge_right_car;
//   lane_width_ = lane_width;

//   if (premoving_) {
//     lat_offset_ += lat_offset;
//   } else {
//     lat_offset_ = lat_offset;
//   }
// }

// bool LateralMotionPlannerV1::update_planner_output() {
//   auto &map_info = world_model_->get_map_info();
//   auto &map_info_mgr = world_model_->get_map_info_manager();
//   auto &lateral_output = context_->mutable_lateral_behavior_planner_output();
//   auto &state_machine_output = context_->state_machine_output();

//   auto &flane = virtual_lane_mgr_->get_fix_lane();  //
//   auto &f_refline =
//       virtual_lane_mgr_
//           ->get_fix_refline();  // fix lane 和fix refline 的区别是？
//   auto &lateral_obstacle = world_model_->mutable_lateral_obstacle();

//   int scenario = state_machine_output.scenario;
//   int state = state_machine_output.curr_state;
//   auto &state_name = state_machine_output.state_name;
//   int turn_light = state_machine_output.turn_light;
//   int map_turn_light =
//       state_machine_output.map_turn_light;  // 取值范围，对应的含义？
//   bool accident_ahead = state_machine_output.accident_ahead;        // ?
//   bool accident_back = state_machine_output.accident_back;          // ?
//   bool close_to_accident = state_machine_output.close_to_accident;  // ?

//   bool lc_pause = state_machine_output.lc_pause;
//   int lc_pause_id =
//       state_machine_output
//           .lc_pause_id;  // id
//                          //
//                          是指车辆id还是路线id，以及id命名顺序是从左到右还是从右到左？
//   double tr_pause_l =
//       state_machine_output.tr_pause_l;  // 为啥是double 类型？ 表示的含义是？
//   double tr_pause_s = state_machine_output.tr_pause_s;

//   bool isRedLightStop =
//       world_model_->get_traffic_light_decision()->get_stop_flag();

//   update_planner_status();

//   lateral_output.timestamp = IflyTime::Now_ms();
//   lateral_output.enable = true;
//   lateral_output.track_id = ignore_track_id_;
//   lateral_output.v_limit = 40.0 / 3.6;
//   lateral_output.isRedLightStop = isRedLightStop;

//   lateral_output.disable_l = state_machine_output.disable_l;
//   lateral_output.disable_r = state_machine_output.disable_r;
//   lateral_output.enable_l =
//       state_machine_output.enable_l;  // disabled
//       ,enable的区别是啥？代表啥意思
//   lateral_output.enable_r = state_machine_output.enable_r;
//   lateral_output.enable_id =
//       state_machine_output.enable_id;  // enable车的id？ 车道线的id?

//   lateral_output.lat_offset =
//       path_planner_->lat_offset();  // 这个是啥含义？ 从哪里计算的？
//   lateral_output.lane_borrow = lane_borrow_;
//   lateral_output.lane_borrow_range = lane_borrow_range_;
//   TrackedObject *lead_one = lateral_obstacle.leadone();

//   if (f_refline.position() ==
//           LEFT_POS ||  // 判断fix refline 在自车的方位？ 左或者右？
//       f_refline.position() == LEFT_LEFT_POS) {
//     lateral_output.which_lane =
//         "left_line";  // 定义成int type类型，减少string的使用
//   } else if (f_refline.position() == RIGHT_POS) {
//     lateral_output.which_lane = "right_line";
//   } else {
//     lateral_output.which_lane = "current_line";
//   }

//   int lb_request =
//       state_machine_output.lb_request;  // 放到上面一起定义，尽量使用引用

//   if (lb_request == NO_CHANGE) {
//     lateral_output.lb_request = "none";  // 减少string的使用
//   } else if (lb_request == LEFT_CHANGE) {
//     lateral_output.lb_request = "left";
//   } else {
//     lateral_output.lb_request = "right";
//   }

//   lateral_output.lb_width = lb_width_;  // lane borrow 的宽度是指？ 画图理解

//   int lc_request = state_machine_output.lc_request;

//   if (lc_request == NO_CHANGE) {
//     lateral_output.lc_request = "none";
//   } else if (lc_request == LEFT_CHANGE) {
//     lateral_output.lc_request = "left";
//   } else {
//     lateral_output.lc_request = "right";
//   }

//   if (state == ROAD_LC_LCHANGE || state == INTER_GS_LC_LCHANGE ||
//       state == INTER_TR_LC_LCHANGE || state == INTER_TL_LC_LCHANGE) {
//     lateral_output.lc_status = "left_lane_change";
//   } else if (state == ROAD_LC_LBACK || state == INTER_GS_LC_LBACK ||
//              state == INTER_TR_LC_LBACK || state == INTER_TL_LC_LBACK) {
//     lateral_output.lc_status = "left_lane_change_back";
//   } else if (state == ROAD_LC_RCHANGE || state == INTER_GS_LC_RCHANGE ||
//              state == INTER_TR_LC_RCHANGE || state == INTER_TL_LC_RCHANGE) {
//     lateral_output.lc_status = "right_lane_change";
//   } else if (state == ROAD_LC_RBACK || state == INTER_GS_LC_RBACK ||
//              state == INTER_TR_LC_RBACK || state == INTER_TL_LC_RBACK) {
//     lateral_output.lc_status = "right_lane_change_back";
//   } else if (state == ROAD_LC_LWAIT || state == INTER_GS_LC_LWAIT ||
//              state == INTER_TR_LC_LWAIT || state == INTER_TL_LC_LWAIT) {
//     lateral_output.lc_status = "left_lane_change_wait";
//   } else if (state == ROAD_LC_RWAIT || state == INTER_GS_LC_RWAIT ||
//              state == INTER_TR_LC_RWAIT || state == INTER_TL_LC_RWAIT) {
//     lateral_output.lc_status = "right_lane_change_wait";
//   } else {
//     lateral_output.lc_status = "none";
//   }

//   if (state == ROAD_LB_LBORROW) {
//     lateral_output.lb_status = "left_lane_borrow";
//   } else if (state == ROAD_LB_RBORROW) {
//     lateral_output.lb_status = "right_lane_borrow";
//   } else if (state == ROAD_LB_LSUSPEND) {
//     lateral_output.lb_status = "left_lane_suspend";
//   } else if (state == ROAD_LB_RSUSPEND) {
//     lateral_output.lb_status = "right_lane_suspend";
//   } else {
//     lateral_output.lb_status = "none";
//   }

//   lateral_output.avd_info.clear();  // avoid info 里存的是 ？
//   for (auto it = avd_info_.begin(); it != avd_info_.end(); ++it) {
//     lateral_output.avd_info.push_back(it->second);
//   }

//   lateral_output.scenario = scenario;
//   lateral_output.flane_width = flane.width();  // fix lane width 是指？
//   画图理解

//   lateral_output.dist_rblane = dist_rblane_;  // rb lane 是指？

//   lateral_output.path_points = f_refline.path_points();

//   if (((map_info.current_lane_index() == map_info.lanes_num() - 1) ||
//        (map_info.current_lane_index() ==
//             map_info.lanes_num() - 2 &&  // 从左到右的顺序是？
//         map_info_mgr.rlane_.exist() &&   // rlane_ 是指？
//         map_info_mgr.rlane_.type() ==
//             MSD_LANE_TYPE_NON_MOTOR)) &&   // LANE_TYPE_NON_MOTOR是啥。。
//       (map_info.dist_to_intsect() > 80 ||  // dist_to 路口的距离大于80？
//        (!isRedLightStop &&
//         lateral_output
//             .accident_ahead &&  // 没有红灯？ accident_ahead的0和1分别代表？
//         lead_one != nullptr &&
//         lead_one->type == 20001))) {  // lead_one 存在， 20001 magic
//                                       // number咋理解。。。。 自行车？
//     lateral_output.borrow_bicycle_lane = true;
//   } else {
//     lateral_output.borrow_bicycle_lane = false;
//   }

//   // 进入路口逻辑判断
//   if (map_info.is_in_intersection() || map_info.dist_to_intsect() <= 20.0) {
//     lateral_output.enable_intersection_planner = true;
//   } else {
//     lateral_output.enable_intersection_planner = false;
//   }

//   bool left_direct_exist =
//       (map_info.left_lane_marks() &
//        map_info
//            .traffic_light_direction());  // lane_marks 左转路标？
//            交通路灯方向

//   bool right_direct_exist =
//       (map_info.right_lane_marks() & map_info.traffic_light_direction());

//   bool curr_direct_has_straight =
//       (map_info.current_lane_marks() & MSD_DIRECTION_GO_STRAIGHT);

//   bool curr_direct_has_right =
//       (map_info.current_lane_marks() & MSD_DIRECTION_TURN_RIGHT);

//   bool left_direct_has_straight =
//       (map_info.left_lane_marks() & MSD_DIRECTION_GO_STRAIGHT);

//   bool is_right_turn =
//       (map_info.traffic_light_direction() & MSD_DIRECTION_TURN_RIGHT);

//   if (map_info_mgr.llane_.exist() == false ||
//       left_direct_exist == false) {  // 咋理解
//     lateral_output.tleft_lane = true;
//   } else {
//     lateral_output.tleft_lane = false;
//   }

//   if (((map_info.current_lane_index() == map_info.lanes_num() - 1) ||
//        (map_info.current_lane_index() == map_info.lanes_num() - 2 &&
//         map_info_mgr.rlane_.exist() &&
//         map_info_mgr.rlane_.type() ==
//             MSD_LANE_TYPE_NON_MOTOR)) &&  // 最右车道判断？
//       map_info.current_lane_index() - 1 >= 0) {
//     lateral_output.rightest_lane = true;
//   } else {
//     lateral_output.rightest_lane = false;
//   }

//   if (map_info.dist_to_intsect() !=
//       DBL_MAX) {  // double max 越界保护，最大值1000
//                   // ，这种是不是得放在函数入口处啊；
//     lateral_output.dist_intersect = map_info.dist_to_intsect();
//   } else {
//     lateral_output.dist_intersect = 1000;
//   }

//   if (map_info.intsect_length() != DBL_MAX) {
//     lateral_output.intersect_length = map_info.intsect_length();
//   } else {
//     lateral_output.intersect_length = 1000;
//   }

//   if (map_info.lc_end_dis() != DBL_MAX) {
//     lateral_output.lc_end_dis = map_info.lc_end_dis();
//   } else {
//     lateral_output.lc_end_dis = 10000;
//   }

//   if (map_info.dis_to_ramp() != DBL_MAX) {
//     lateral_output.dis_to_ramp = map_info.dis_to_ramp();
//   } else {
//     lateral_output.dis_to_ramp = 10000;
//   }

//   lateral_output.sb_lane = path_planner_->sb_lane();
//   lateral_output.sb_blane = path_planner_->sb_blane();
//   lateral_output.force_pause =
//       path_planner_->force_pause();  // force pause强制 pause？
//       什么情况下有？
//   lateral_output.large_lat =
//       path_planner_->large_lat();  // large lat最大横向位置？
//   lateral_output.premoving = path_planner_->premoving();  // premoving 预移动
//   lateral_output.accident_ahead =
//       accident_ahead;  // 这个值直接从 state
//       machine里拿的，是否可以同时定义；
//   lateral_output.accident_back = accident_back;
//   lateral_output.lc_pause_id = lc_pause_id;
//   lateral_output.lc_pause = lc_pause;
//   lateral_output.tr_pause_l = tr_pause_l;
//   lateral_output.tr_pause_s = tr_pause_s;
//   lateral_output.must_change_lane = state_machine_output.must_change_lane;
//   lateral_output.close_to_accident = close_to_accident;
//   lateral_output.angle_steers_limit = angle_steers_limit_;
//   lateral_output.left_faster = state_machine_output.left_is_faster;
//   lateral_output.right_faster = state_machine_output.right_is_faster;
//   lateral_output.premove =
//       (state_machine_output.premovel ||
//        state_machine_output.premover);  // 一个向左 一个向右？
//   lateral_output.premove_dist =
//       state_machine_output.premove_dist;  // 这个是纵向距离还是横向距离
//   lateral_output.isFasterStaticAvd =
//       (left_direct_exist &&
//        lateral_output.left_faster) ||  // left_faster是对于自车还是他车？
//       (right_direct_exist && lateral_output.right_faster) ||
//       (curr_direct_has_right && !curr_direct_has_straight) ||
//       (is_right_turn && left_direct_has_straight &&
//       lateral_output.left_faster);
//   lateral_output.isOnHighway = map_info.is_on_highway();

//   lateral_output.c_poly.assign(
//       path_planner_->c_poly()
//           .begin(),  // c_poly 和 d_poly考虑是不是定义为成员变量
//       path_planner_->c_poly().end());

//   lateral_output.d_poly.assign(path_planner_->d_poly().begin(),
//                                path_planner_->d_poly().end());

//   lateral_output.s_v_limit.clear();
//   lateral_output.s_a_limit.clear();
//   lateral_output.s_r_offset.clear();
//   lateral_output.s_v_limit.assign(
//       s_v_limit_.begin(),
//       s_v_limit_.end());  // s_v_limit
//       和s_a_limit是在哪里计算的或者从哪里透传的
//   lateral_output.s_a_limit.assign(s_a_limit_.begin(), s_a_limit_.end());
//   lateral_output.s_r_offset.assign(s_r_offset_.begin(), s_r_offset_.end());

//   if (lead_one != nullptr) {
//     lateral_output.lead_one_drel = lead_one->d_rel;  //  lead_one距离
//     lateral_output.lead_one_vrel = lead_one->v_rel;  // lead_one 速度
//   } else {
//     lateral_output.lead_one_drel = 0.0;
//     lateral_output.lead_one_vrel = 0.0;
//   }

//   lateral_output.state_name = state_name;  // 这里也可以只做一次赋值；

//   lateral_output.scenario_name =
//       (scenario == LOCATION_ROAD) ? "Road" : "Intersect";

//   if (turn_light == 1) {
//     lateral_output.turn_light = "Left";  // 这里也可以只做一次赋值；
//   } else if (turn_light == 2) {
//     lateral_output.turn_light = "Right";
//   } else {
//     lateral_output.turn_light = "None";
//   }

//   auto request_source = state_machine_output.lc_request_source;
//   lateral_output.act_request_source = "none";  // act是嘛意思
//   if (request_source == INT_REQUEST) {
//     lateral_output.lc_request_source = "int_request";  // int 是嘛意思
//   } else if (request_source == MAP_REQUEST) {
//     lateral_output.lc_request_source = "map_request";
//   } else if (request_source == ACT_REQUEST) {
//     lateral_output.lc_request_source = "act_request";
//     lateral_output.act_request_source =
//     state_machine_output.act_request_source;
//   } else {
//     lateral_output.lc_request_source = "none";
//   }

//   if (map_turn_light > 0) {
//     lateral_output.turn_light_source = "map_turn_light";
//   } else if (state_machine_output.lc_turn_light > 0) {
//     lateral_output.turn_light_source = "lc_turn_light";
//   } else if (state_machine_output.lb_turn_light > 0) {
//     lateral_output.turn_light_source = "lb_turn_light";
//   } else {
//     lateral_output.turn_light_source = "none";
//   }

//   auto planning_status = context_->mutable_planning_status();
//   if (nullptr != planning_status) {
//     planning_status->planning_result.turn_signal_cmd.set_value(turn_light);
//   }

//   lateral_output.avd_car_past = avd_car_past_;
//   lateral_output.avd_sp_car_past = avd_sp_car_past_;
//   lateral_output.vel_sequence = vel_sequence_;

//   lateral_output.ignore_change_false = ignore_change_false_;
//   lateral_output.ignore_change_true = ignore_change_true_;

//   lateral_output.cross_lsolid_line = path_planner_->cross_lsolid_line();
//   lateral_output.cross_rsolid_line = path_planner_->cross_rsolid_line();

//   lateral_output.l_poly = path_planner_->l_poly();  // l_poly is the ？
//   lateral_output.r_poly = path_planner_->r_poly();  // r_poly is the?

//   lateral_output.behavior_suspension =
//       state_machine_output.behavior_suspend;  // lateral suspend
//   lateral_output.suspension_obs.assign(
//       state_machine_output.suspend_obs.begin(),
//       state_machine_output.suspend_obs.end());  // lateral suspend obstacles
//   if (!update_lateral_info()) {  // 这里进入 横向规划轨迹的输出部分
//     return false;
//   }
//   return true;
// }
// bool LateralMotionPlannerV1::update_lateral_info() {
//   world_model_->mutable_map_info_manager().get_lane_change_point(world_model_);

//   auto &map_info = world_model_->mutable_map_info_manager().get_map_info();

//   auto planning_status = context_->mutable_planning_status();
//   auto &lateral_output = context_->lateral_behavior_planner_output();

//   auto lc_request = lateral_output.lc_request;
//   auto lc_status = lateral_output.lc_status;
//   // auto lb_status = lateral_output.lane_borrow;
//   auto lb_request = lateral_output.lb_request;
//   auto lb_status = lateral_output.lb_status;
//   auto lb_info = lateral_output.lane_borrow_range;
//   // LOG_DEBUG("zzd arbitrator lc_status %s lc_request %s lb_status %s
//   // lb_request %s lb_info %d", lc_status.c_str(), lc_request.c_str(),
//   //   lb_status.c_str(), lb_request.c_str(), lb_info);

//   auto &state_machine_output = context_->state_machine_output();

//   int scenario = state_machine_output.scenario;
//   int state = state_machine_output.curr_state;
//   LaneStatus default_lane_status;
//   // scenario input info
//   default_lane_status.change_lane.target_gap_obs =
//       planning_status->lane_status.change_lane
//           .target_gap_obs;  // target gap obstacle 是啥？
//   planning_status->lane_status = default_lane_status;

//   if (state == ROAD_NONE || state == INTER_GS_NONE || state == INTER_TR_NONE
//   ||
//       state == INTER_TL_NONE) {
//     planning_status->lane_status.status = LaneStatus::Status::LANE_KEEP;
//   } else {
//     if (lc_request == "none") {  //  换成type类型判断
//       if (lb_request != "none") {
//         planning_status->lane_status.status =
//         LaneStatus::Status::LANE_BORROW;
//         // TODO: BORROW_LANE_KEEP state to be set accordingly
//         if (lb_request == "left") {
//           planning_status->lane_status.borrow_lane.direction = "left";
//         } else if (lb_request == "right") {
//           planning_status->lane_status.borrow_lane.direction = "right";
//         }
//         if (lb_status == "left_lane_borrow" ||
//             lb_status == "right_lane_borrow") {
//           planning_status->lane_status.borrow_lane.status =
//               BorrowLaneStatus::Status::IN_BORROW_LANE;
//         } else if (lb_status == "left_lane_suspend" ||
//                    lb_status == "right_lane_suspend") {
//           planning_status->lane_status.borrow_lane.status =
//               BorrowLaneStatus::Status::BORROW_LANE_KEEP;
//         } else {
//           planning_status->lane_status.borrow_lane.status =
//               BorrowLaneStatus::Status::BORROW_LANE_FINISHED;
//         }
//       } else {
//         planning_status->lane_status.status = LaneStatus::Status::LANE_KEEP;
//         planning_status->lane_status.change_lane.status =
//             ChangeLaneStatus::Status::CHANGE_LANE_FINISHED;
//         planning_status->lane_status.borrow_lane.status =
//             BorrowLaneStatus::Status::BORROW_LANE_FINISHED;
//       }
//     } else {
//       planning_status->lane_status.status = LaneStatus::Status::LANE_CHANGE;

//       if (map_info.current_tasks().size() > 0 &&
//           (map_info.current_tasks()[0] == -1 ||
//            map_info.current_tasks()[0] ==
//                1)) {  // map_info 现在还没接入，这些值是默认值吗？
//         planning_status->lane_status.change_lane.is_active_lane_change =
//         false;
//       } else {
//         planning_status->lane_status.change_lane.is_active_lane_change =
//         true;
//       }
//       if (lc_status == "none") {
//         // lane change preparation stage
//         planning_status->lane_status.change_lane.status =
//             ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION;
//         if (lc_request == "left") {
//           planning_status->lane_status.change_lane.direction = "left";
//         } else if (lc_request == "right") {
//           planning_status->lane_status.change_lane.direction = "right";
//         }
//       } else if (lc_status == "left_lane_change" ||
//                  lc_status == "right_lane_change") {
//         planning_status->lane_status.change_lane.status =
//             ChangeLaneStatus::Status::IN_CHANGE_LANE;
//         if (lc_status == "left_lane_change") {
//           planning_status->lane_status.change_lane.direction = "left";
//         } else if (lc_status == "right_lane_change") {
//           planning_status->lane_status.change_lane.direction = "right";
//         }
//       } else if (lc_status == "left_lane_change_back" ||
//                  lc_status == "right_lane_change_back") {
//         planning_status->lane_status.change_lane.status =
//             ChangeLaneStatus::Status::CHANGE_LANE_BACK;
//         if (lc_status == "left_lane_change_back") {
//           planning_status->lane_status.change_lane.direction = "left";
//         } else if (lc_status == "right_lane_change_back") {
//           planning_status->lane_status.change_lane.direction = "right";
//         }
//       } else if (lc_status == "left_lane_change_wait" ||
//                  lc_status == "right_lane_change_wait") {
//         planning_status->lane_status.change_lane.status =
//             ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION;
//         if (lc_status == "left_lane_change_wait") {
//           planning_status->lane_status.change_lane.direction = "left";
//         } else if (lc_status == "right_lane_change_wait") {
//           planning_status->lane_status.change_lane.direction = "right";
//         }
//       }
//     }
//   }

//   // update target lane id
//   int target_lane_id = 0;
//   if (lateral_output.which_lane == "left_line") {
//     target_lane_id = -1;
//   } else if (lateral_output.which_lane == "right_line") {
//     target_lane_id = 1;
//   } else {
//     target_lane_id = 0;
//   }
//   planning_status->lane_status.target_lane_id = target_lane_id;
//   planning_status->lane_status.change_lane.path_id = target_lane_id;
//   if (planning_status->lane_status.status == LaneStatus::Status::LANE_CHANGE
//   &&
//       (planning_status->lane_status.change_lane.status ==
//            ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION ||
//        planning_status->lane_status.change_lane.status ==
//            ChangeLaneStatus::Status::CHANGE_LANE_BACK)) {
//     if (lc_request == "left") {
//       planning_status->lane_status.change_lane.path_id = target_lane_id - 1;
//     } else if (lc_request == "right") {
//       planning_status->lane_status.change_lane.path_id = target_lane_id + 1;
//     }
//   }
//   planning_status->lane_status.target_lane_lat_offset =
//       lateral_output.lat_offset;

//   auto baseline_info = world_model_->get_baseline_info(target_lane_id);

//   // todo: add target baseline protection in lane change /lane borrow stage
//   if (baseline_info == nullptr || !baseline_info->is_valid()) {
//     LOG_ERROR("zzd arbitrator invalid target lane[%d]!", target_lane_id);
//     return false;
//   }

//   // construct_virtual_obstacles();
//   return true;
// }
// bool LateralMotionPlannerV1::update_planner_status() {
//   auto &lateral_output = context_->mutable_lateral_behavior_planner_output();
//   auto &state_machine_output = context_->state_machine_output();

//   lateral_output.planner_scene = 0;
//   lateral_output.planner_action = 0;
//   lateral_output.planner_status = 0;

//   switch (state_machine_output.curr_state) {
//     case ROAD_NONE:
//       lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
//       break;

//     case ROAD_LC_LWAIT:
//       lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
//       lateral_output.planner_action = AlgorithmAction::LANE_CHANGE_LEFT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
//       break;

//     case ROAD_LC_RWAIT:
//       lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
//       lateral_output.planner_action = AlgorithmAction::LANE_CHANGE_RIGHT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
//       break;

//     case ROAD_LC_LCHANGE:
//       lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
//       lateral_output.planner_action = AlgorithmAction::LANE_CHANGE_LEFT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
//       break;

//     case ROAD_LC_RCHANGE:
//       lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
//       lateral_output.planner_action = AlgorithmAction::LANE_CHANGE_RIGHT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
//       break;

//     case ROAD_LC_LBACK:
//       lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
//       lateral_output.planner_action = AlgorithmAction::LANE_CHANGE_LEFT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
//       break;

//     case ROAD_LC_RBACK:
//       lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
//       lateral_output.planner_action = AlgorithmAction::LANE_CHANGE_RIGHT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
//       break;

//     case ROAD_LB_LBORROW:
//       lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
//       lateral_output.planner_action = AlgorithmAction::LANE_BORROW_LEFT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_BORROWING;
//       break;

//     case ROAD_LB_RBORROW:
//       lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
//       lateral_output.planner_action = AlgorithmAction::LANE_BORROW_RIGHT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_BORROWING;
//       break;

//     case ROAD_LB_LBACK:
//       lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
//       lateral_output.planner_action = AlgorithmAction::LANE_BORROW_LEFT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_BORROW_BACK;
//       break;

//     case ROAD_LB_RBACK:
//       lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
//       lateral_output.planner_action = AlgorithmAction::LANE_BORROW_RIGHT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_BORROW_BACK;
//       break;

//     case ROAD_LB_LRETURN:
//       lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
//       lateral_output.planner_action = AlgorithmAction::LANE_BORROW_LEFT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_BORROW_RETURN;
//       break;

//     case ROAD_LB_RRETURN:
//       lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
//       lateral_output.planner_action = AlgorithmAction::LANE_BORROW_RIGHT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_BORROW_RETURN;
//       break;

//     case ROAD_LB_LSUSPEND:
//       lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
//       lateral_output.planner_action = AlgorithmAction::LANE_BORROW_LEFT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_BORROW_SUSPEND;
//       break;

//     case ROAD_LB_RSUSPEND:
//       lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
//       lateral_output.planner_action = AlgorithmAction::LANE_BORROW_RIGHT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_BORROW_SUSPEND;
//       break;

//     case INTER_GS_NONE:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_GO_STRAIGHT;
//       break;

//     case INTER_GS_LC_LWAIT:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_GO_STRAIGHT
//       |
//                                       AlgorithmAction::LANE_CHANGE_LEFT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
//       break;

//     case INTER_GS_LC_RWAIT:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_GO_STRAIGHT
//       |
//                                       AlgorithmAction::LANE_CHANGE_RIGHT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
//       break;

//     case INTER_GS_LC_LCHANGE:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_GO_STRAIGHT
//       |
//                                       AlgorithmAction::LANE_CHANGE_LEFT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
//       break;

//     case INTER_GS_LC_RCHANGE:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_GO_STRAIGHT
//       |
//                                       AlgorithmAction::LANE_CHANGE_RIGHT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
//       break;

//     case INTER_GS_LC_LBACK:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_GO_STRAIGHT
//       |
//                                       AlgorithmAction::LANE_CHANGE_LEFT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
//       break;

//     case INTER_GS_LC_RBACK:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_GO_STRAIGHT
//       |
//                                       AlgorithmAction::LANE_CHANGE_RIGHT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
//       break;

//     case INTER_TR_NONE:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_RIGHT;
//       break;

//     case INTER_TR_LC_LWAIT:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_RIGHT |
//                                       AlgorithmAction::LANE_CHANGE_LEFT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
//       break;

//     case INTER_TR_LC_RWAIT:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_RIGHT |
//                                       AlgorithmAction::LANE_CHANGE_RIGHT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
//       break;

//     case INTER_TR_LC_LCHANGE:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_RIGHT |
//                                       AlgorithmAction::LANE_CHANGE_LEFT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
//       break;

//     case INTER_TR_LC_RCHANGE:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_RIGHT |
//                                       AlgorithmAction::LANE_CHANGE_RIGHT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
//       break;

//     case INTER_TR_LC_LBACK:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_RIGHT |
//                                       AlgorithmAction::LANE_CHANGE_LEFT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
//       break;

//     case INTER_TR_LC_RBACK:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_RIGHT |
//                                       AlgorithmAction::LANE_CHANGE_RIGHT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
//       break;

//     case INTER_TL_NONE:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_LEFT;
//       break;

//     case INTER_TL_LC_LWAIT:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_LEFT |
//                                       AlgorithmAction::LANE_CHANGE_LEFT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
//       break;

//     case INTER_TL_LC_RWAIT:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_LEFT |
//                                       AlgorithmAction::LANE_CHANGE_RIGHT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
//       break;

//     case INTER_TL_LC_LCHANGE:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_LEFT |
//                                       AlgorithmAction::LANE_CHANGE_LEFT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
//       break;

//     case INTER_TL_LC_RCHANGE:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_LEFT |
//                                       AlgorithmAction::LANE_CHANGE_RIGHT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
//       break;

//     case INTER_TL_LC_LBACK:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_LEFT |
//                                       AlgorithmAction::LANE_CHANGE_LEFT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
//       break;

//     case INTER_TL_LC_RBACK:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_LEFT |
//                                       AlgorithmAction::LANE_CHANGE_RIGHT;
//       lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
//       break;

//     case INTER_UT_NONE:
//       lateral_output.planner_scene = AlgorithmScene::INTERSECT;
//       lateral_output.planner_action = AlgorithmAction::INTERSECT_U_TURN;
//       break;

//     default:
//       break;
//   }

//   if (lateral_output.sb_blane) {
//     lateral_output.planner_action |=
//         AlgorithmAction::LANE_BORROW_IN_NON_MOTORIZED_LANE;
//   }
// }
// bool LateralMotionPlannerV1::log_planner_debug_info() {
//   std::string plan_msg;
//   create_lateral_behavior_planner_msg(plan_msg);

//   auto &lateral_behavior_planner_output =
//       PlanningContext::Instance()->mutable_lateral_behavior_planner_output();

//   lateral_behavior_planner_output.plan_msg = plan_msg;
// }
// bool LateralMotionPlannerV1::create_lateral_behavior_planner_msg(
//     std::string &plan_msg) {
//   auto &lateral_output =
//       PlanningContext::Instance()->lateral_behavior_planner_output();
//   rapidjson::Document later_out_json(rapidjson::kObjectType);
//   rapidjson::Document::AllocatorType &allocator =
//   later_out_json.GetAllocator();

//   rapidjson::SetValueByPointer(later_out_json, "/timestamp",
//                                lateral_output.timestamp);
//   rapidjson::SetValueByPointer(later_out_json, "/enable",
//                                lateral_output.enable);
//   rapidjson::SetValueByPointer(later_out_json, "/track_id",
//                                lateral_output.track_id);
//   rapidjson::SetValueByPointer(later_out_json, "/v_limit",
//                                lateral_output.v_limit);
//   rapidjson::SetValueByPointer(later_out_json, "/lat_offset",
//                                lateral_output.lat_offset);
//   rapidjson::SetValueByPointer(later_out_json, "/lane_borrow",
//                                lateral_output.lane_borrow);
//   rapidjson::SetValueByPointer(later_out_json, "/lane_borrow_range",
//                                lateral_output.lane_borrow_range);

//   rapidjson::Document avd_array(rapidjson::kArrayType, &allocator);
//   for (int i = 0; i < lateral_output.avd_info.size(); i++) {
//     rapidjson::Document one_avd_obj(rapidjson::kObjectType, &allocator);
//     rapidjson::Value str_val;
//     rapidjson::SetValueByPointer(one_avd_obj, "/id",
//                                  lateral_output.avd_info[i].id);
//     str_val.SetString(lateral_output.avd_info[i].property.c_str(),
//                       lateral_output.avd_info[i].property.length(),
//                       allocator);
//     rapidjson::SetValueByPointer(one_avd_obj, "/property", str_val);
//     rapidjson::SetValueByPointer(one_avd_obj, "/ignore",
//                                  lateral_output.avd_info[i].ignore);
//     str_val.SetString(lateral_output.avd_info[i].avd_direction.c_str(),
//                       lateral_output.avd_info[i].avd_direction.length(),
//                       allocator);
//     rapidjson::SetValueByPointer(one_avd_obj, "/avd_direction", str_val);
//     rapidjson::SetValueByPointer(one_avd_obj, "/avd_priority",
//                                  lateral_output.avd_info[i].avd_priority);
//     rapidjson::SetValueByPointer(one_avd_obj, "/blocked_time_begin",
//                                  lateral_output.avd_info[i].blocked_time_begin);
//     rapidjson::SetValueByPointer(one_avd_obj, "/blocked_time_end",
//                                  lateral_output.avd_info[i].blocked_time_end);
//     avd_array.PushBack(one_avd_obj, allocator);
//   }
//   rapidjson::SetValueByPointer(later_out_json, "/avd_info", avd_array);

//   rapidjson::Value str_temp;
//   str_temp.SetString(lateral_output.which_lane.c_str(),
//                      lateral_output.which_lane.length(), allocator);
//   rapidjson::SetValueByPointer(later_out_json, "/which_lane", str_temp);
//   str_temp.SetString(lateral_output.lb_request.c_str(),
//                      lateral_output.lb_request.length(), allocator);
//   rapidjson::SetValueByPointer(later_out_json, "/lb_request", str_temp);
//   str_temp.SetString(lateral_output.lc_request.c_str(),
//                      lateral_output.lc_request.length(), allocator);
//   rapidjson::SetValueByPointer(later_out_json, "/lc_request", str_temp);
//   str_temp.SetString(lateral_output.lc_status.c_str(),
//                      lateral_output.lc_status.length(), allocator);
//   rapidjson::SetValueByPointer(later_out_json, "/lc_status", str_temp);
//   str_temp.SetString(lateral_output.lb_status.c_str(),
//                      lateral_output.lb_status.length(), allocator);
//   rapidjson::SetValueByPointer(later_out_json, "/lb_status", str_temp);

//   rapidjson::SetValueByPointer(later_out_json, "/scenario",
//                                lateral_output.scenario);
//   rapidjson::SetValueByPointer(later_out_json, "/flane_width",
//                                lateral_output.flane_width);
//   rapidjson::SetValueByPointer(later_out_json, "/dist_rblane",
//                                lateral_output.dist_rblane);
//   rapidjson::SetValueByPointer(later_out_json, "/borrow_bicycle_lane",
//                                lateral_output.borrow_bicycle_lane);
//   rapidjson::SetValueByPointer(later_out_json,
//   "/enable_intersection_planner",
//                                lateral_output.enable_intersection_planner);
//   rapidjson::SetValueByPointer(later_out_json, "/tleft_lane",
//                                lateral_output.tleft_lane);
//   rapidjson::SetValueByPointer(later_out_json, "/rightest_lane",
//                                lateral_output.rightest_lane);
//   rapidjson::SetValueByPointer(later_out_json, "/isFasterStaticAvd",
//                                lateral_output.isFasterStaticAvd);
//   rapidjson::SetValueByPointer(later_out_json, "/isOnHighway",
//                                lateral_output.isOnHighway);
//   rapidjson::SetValueByPointer(later_out_json, "/isRedLightStop",
//                                lateral_output.isRedLightStop);
//   rapidjson::SetValueByPointer(later_out_json, "/dist_intersect",
//                                lateral_output.dist_intersect);
//   rapidjson::SetValueByPointer(later_out_json, "/intersect_length",
//                                lateral_output.intersect_length);
//   rapidjson::SetValueByPointer(later_out_json, "/lc_end_dis",
//                                lateral_output.lc_end_dis);
//   rapidjson::SetValueByPointer(later_out_json, "/dis_to_ramp",
//                                lateral_output.dis_to_ramp);

//   /* rapidjson::Document path_pnt_array(rapidjson::kArrayType, &allocator);
//   for(int i = 0; i < lateral_output.path_points.size(); i++)
//   {
//     rapidjson::Document one_path_pnt(rapidjson::kObjectType, &allocator);
//     rapidjson::SetValueByPointer(one_path_pnt, "/x",
//   lateral_output.path_points[i].x);
//   rapidjson::SetValueByPointer(one_path_pnt,
//   "/y", lateral_output.path_points[i].y);
//     rapidjson::SetValueByPointer(one_path_pnt, "/z",
//   lateral_output.path_points[i].z);
//   rapidjson::SetValueByPointer(one_path_pnt,
//   "/theta", lateral_output.path_points[i].theta);
//     rapidjson::SetValueByPointer(one_path_pnt, "/kappa",
//   lateral_output.path_points[i].kappa);
//     rapidjson::SetValueByPointer(one_path_pnt, "/s",
//   lateral_output.path_points[i].s);
//   rapidjson::SetValueByPointer(one_path_pnt,
//   "/dkappa", lateral_output.path_points[i].dkappa);
//     rapidjson::SetValueByPointer(one_path_pnt, "/ddkappa",
//   lateral_output.path_points[i].ddkappa);
//     rapidjson::SetValueByPointer(one_path_pnt, "/lane_id",
//   lateral_output.path_points[i].lane_id.c_str());
//     rapidjson::SetValueByPointer(one_path_pnt, "/x_derivative",
//   lateral_output.path_points[i].x_derivative);
//     rapidjson::SetValueByPointer(one_path_pnt, "/y_derivative",
//   lateral_output.path_points[i].y_derivative);
//     path_pnt_array.PushBack(one_path_pnt, allocator);
//   }
//   rapidjson::SetValueByPointer(later_out_json, "/path_points",
//   path_pnt_array);
// */

//   rapidjson::SetValueByPointer(later_out_json, "/sb_lane",
//                                lateral_output.sb_lane);
//   rapidjson::SetValueByPointer(later_out_json, "/sb_blane",
//                                lateral_output.sb_blane);
//   rapidjson::SetValueByPointer(later_out_json, "/force_pause",
//                                lateral_output.force_pause);
//   rapidjson::SetValueByPointer(later_out_json, "/large_lat",
//                                lateral_output.large_lat);
//   rapidjson::SetValueByPointer(later_out_json, "/premoving",
//                                lateral_output.premoving);
//   rapidjson::SetValueByPointer(later_out_json, "/accident_ahead",
//                                lateral_output.accident_ahead);
//   rapidjson::SetValueByPointer(later_out_json, "/accident_back",
//                                lateral_output.accident_back);
//   rapidjson::SetValueByPointer(later_out_json, "/lc_pause_id",
//                                lateral_output.lc_pause_id);
//   rapidjson::SetValueByPointer(later_out_json, "/lc_pause",
//                                lateral_output.lc_pause);
//   rapidjson::SetValueByPointer(later_out_json, "/tr_pause_l",
//                                lateral_output.tr_pause_l);
//   rapidjson::SetValueByPointer(later_out_json, "/tr_pause_s",
//                                lateral_output.tr_pause_s);
//   rapidjson::SetValueByPointer(later_out_json, "/must_change_lane",
//                                lateral_output.must_change_lane);
//   rapidjson::SetValueByPointer(later_out_json, "/left_faster",
//                                lateral_output.left_faster);
//   rapidjson::SetValueByPointer(later_out_json, "/right_faster",
//                                lateral_output.right_faster);
//   rapidjson::SetValueByPointer(later_out_json, "/close_to_accident",
//                                lateral_output.close_to_accident);
//   rapidjson::SetValueByPointer(later_out_json, "/premove",
//                                lateral_output.premove);
//   rapidjson::SetValueByPointer(later_out_json, "/behavior_suspension",
//                                lateral_output.behavior_suspension);

//   rapidjson::Document suspension_ob_array(rapidjson::kArrayType, &allocator);
//   for (int i = 0; i < lateral_output.suspension_obs.size(); i++) {
//     suspension_ob_array.PushBack(lateral_output.suspension_obs[i],
//     allocator);
//   }
//   rapidjson::SetValueByPointer(later_out_json, "/suspension_obs",
//                                suspension_ob_array);

//   rapidjson::SetValueByPointer(later_out_json, "/angle_steers_limit",
//                                lateral_output.angle_steers_limit);
//   rapidjson::SetValueByPointer(later_out_json, "/premove_dist",
//                                lateral_output.premove_dist);
//   rapidjson::SetValueByPointer(later_out_json, "/planner_scene",
//                                lateral_output.planner_scene);
//   rapidjson::SetValueByPointer(later_out_json, "/planner_action",
//                                lateral_output.planner_action);
//   rapidjson::SetValueByPointer(later_out_json, "/planner_status",
//                                lateral_output.planner_status);

//   rapidjson::Document c_poly_array(rapidjson::kArrayType, &allocator);
//   for (int i = 0; i < lateral_output.c_poly.size(); i++) {
//     c_poly_array.PushBack(lateral_output.c_poly[i], allocator);
//   }
//   rapidjson::SetValueByPointer(later_out_json, "/c_poly", c_poly_array);

//   rapidjson::Document d_poly_array(rapidjson::kArrayType, &allocator);
//   for (int i = 0; i < lateral_output.d_poly.size(); i++) {
//     d_poly_array.PushBack(lateral_output.d_poly[i], allocator);
//   }
//   rapidjson::SetValueByPointer(later_out_json, "/d_poly", d_poly_array);

//   rapidjson::SetValueByPointer(later_out_json, "/lead_one_drel",
//                                lateral_output.lead_one_drel);
//   rapidjson::SetValueByPointer(later_out_json, "/lead_one_vrel",
//                                lateral_output.lead_one_vrel);

//   str_temp.SetString(lateral_output.state_name.c_str(),
//                      lateral_output.state_name.length(), allocator);
//   rapidjson::SetValueByPointer(later_out_json, "/state_name", str_temp);
//   str_temp.SetString(lateral_output.scenario_name.c_str(),
//                      lateral_output.scenario_name.length(), allocator);
//   rapidjson::SetValueByPointer(later_out_json, "/scenario_name", str_temp);
//   str_temp.SetString(lateral_output.turn_light.c_str(),
//                      lateral_output.turn_light.length(), allocator);
//   rapidjson::SetValueByPointer(later_out_json, "/turn_light", str_temp);
//   str_temp.SetString(lateral_output.lc_request_source.c_str(),
//                      lateral_output.lc_request_source.length(), allocator);
//   rapidjson::SetValueByPointer(later_out_json, "/lc_request_source",
//   str_temp); str_temp.SetString(lateral_output.act_request_source.c_str(),
//                      lateral_output.act_request_source.length(), allocator);
//   rapidjson::SetValueByPointer(later_out_json, "/act_request_source",
//   str_temp); str_temp.SetString(lateral_output.turn_light_source.c_str(),
//                      lateral_output.turn_light_source.length(), allocator);
//   rapidjson::SetValueByPointer(later_out_json, "/turn_light_source",
//   str_temp); rapidjson::SetValueByPointer(later_out_json, "/lb_width",
//                                lateral_output.lb_width);

//   rapidjson::Document v_seq_array(rapidjson::kArrayType, &allocator);
//   for (int i = 0; i < lateral_output.vel_sequence.size(); i++) {
//     v_seq_array.PushBack(lateral_output.vel_sequence[i], allocator);
//   }
//   rapidjson::SetValueByPointer(later_out_json, "/vel_sequence", v_seq_array);

//   rapidjson::Document ig_change_f_array(rapidjson::kArrayType, &allocator);
//   for (auto i = lateral_output.ignore_change_false.begin();
//        i != lateral_output.ignore_change_false.end(); i++) {
//     ig_change_f_array.PushBack(*i, allocator);
//   }
//   rapidjson::SetValueByPointer(later_out_json, "/ignore_change_false",
//                                ig_change_f_array);

//   rapidjson::Document ig_change_t_array(rapidjson::kArrayType, &allocator);
//   for (auto i = lateral_output.ignore_change_true.begin();
//        i != lateral_output.ignore_change_true.end(); i++) {
//     ig_change_t_array.PushBack(*i, allocator);
//   }
//   rapidjson::SetValueByPointer(later_out_json, "/ignore_change_true",
//                                ig_change_t_array);

//   rapidjson::SetValueByPointer(later_out_json, "/cross_lsolid_line",
//                                lateral_output.cross_lsolid_line);
//   rapidjson::SetValueByPointer(later_out_json, "/cross_rsolid_line",
//                                lateral_output.cross_rsolid_line);

//   rapidjson::Document l_poly_array(rapidjson::kArrayType, &allocator);
//   for (int i = 0; i < lateral_output.l_poly.size(); i++) {
//     l_poly_array.PushBack(lateral_output.l_poly[i], allocator);
//   }
//   rapidjson::SetValueByPointer(later_out_json, "/l_poly", l_poly_array);

//   rapidjson::Document r_poly_array(rapidjson::kArrayType, &allocator);
//   for (int i = 0; i < lateral_output.r_poly.size(); i++) {
//     r_poly_array.PushBack(lateral_output.r_poly[i], allocator);
//   }
//   rapidjson::SetValueByPointer(later_out_json, "/r_poly", r_poly_array);

//   rapidjson::SetValueByPointer(later_out_json, "/disable_l",
//                                lateral_output.disable_l);
//   rapidjson::SetValueByPointer(later_out_json, "/disable_r",
//                                lateral_output.disable_r);
//   rapidjson::SetValueByPointer(later_out_json, "/enable_l",
//                                lateral_output.enable_l);
//   rapidjson::SetValueByPointer(later_out_json, "/enable_r",
//                                lateral_output.enable_r);
//   rapidjson::SetValueByPointer(later_out_json, "/enable_id",
//                                lateral_output.enable_id);
//   rapidjson::SetValueByPointer(later_out_json, "/lc_warning",
//                                lateral_output.lc_warning);
//   rapidjson::SetValueByPointer(later_out_json, "/avd_in_lane",
//                                lateral_output.avd_in_lane);
//   rapidjson::SetValueByPointer(later_out_json, "/lc_exit_ramp_start",
//                                lateral_output.lc_exit_ramp_start);

//   /* rapidjson::Document s_v_limit_array(rapidjson::kArrayType, &allocator);
//   for(int i = 0; i < lateral_output.s_v_limit.size(); i++)
//   {
//     rapidjson::Document one_s_v_limit(rapidjson::kArrayType, &allocator);
//     one_s_v_limit.PushBack(std::get<0>(lateral_output.s_v_limit[i]),
//     allocator);
//     one_s_v_limit.PushBack(std::get<1>(lateral_output.s_v_limit[i]),
//     allocator);

//     s_v_limit_array.PushBack(one_s_v_limit, allocator);
//   }
//   rapidjson::SetValueByPointer(later_out_json, "/s_v_limit",
//   s_v_limit_array);

//   rapidjson::Document s_a_limit_array(rapidjson::kArrayType, &allocator);
//   for(int i = 0; i < lateral_output.s_a_limit.size(); i++)
//   {
//     rapidjson::Document one_s_a_limit(rapidjson::kArrayType, &allocator);
//     one_s_a_limit.PushBack(std::get<0>(lateral_output.s_a_limit[i]),
//     allocator);
//     one_s_a_limit.PushBack(std::get<1>(lateral_output.s_a_limit[i]),
//     allocator);

//     s_a_limit_array.PushBack(one_s_a_limit, allocator);
//   }
//   rapidjson::SetValueByPointer(later_out_json, "/s_a_limit",
//   s_a_limit_array);

//   rapidjson::Document s_r_off_array(rapidjson::kArrayType, &allocator);
//   for(int i = 0; i < lateral_output.s_r_offset.size(); i++)
//   {
//     rapidjson::Document one_s_r_off(rapidjson::kArrayType, &allocator);
//     one_s_r_off.PushBack(std::get<0>(lateral_output.s_r_offset[i]),
//     allocator);
//     one_s_r_off.PushBack(std::get<1>(lateral_output.s_r_offset[i]),
//     allocator);

//     s_r_off_array.PushBack(one_s_r_off, allocator);
//   }
//   rapidjson::SetValueByPointer(later_out_json, "/s_r_offset", s_r_off_array);
//   */

//   rapidjson::Document avd_past_array(rapidjson::kArrayType, &allocator);
//   for (int i = 0; i < lateral_output.avd_car_past.size(); i++) {
//     rapidjson::Document one_avd_past(rapidjson::kArrayType, &allocator);
//     for (int j = 0; j < lateral_output.avd_car_past[i].size(); j++) {
//       one_avd_past.PushBack(lateral_output.avd_car_past[i][j], allocator);
//     }
//     avd_past_array.PushBack(one_avd_past, allocator);
//   }
//   rapidjson::SetValueByPointer(later_out_json, "/avd_car_past",
//   avd_past_array);

//   rapidjson::Document avd_sp_past_array(rapidjson::kArrayType, &allocator);
//   for (int i = 0; i < lateral_output.avd_sp_car_past.size(); i++) {
//     rapidjson::Document one_avd_sp_past(rapidjson::kArrayType, &allocator);
//     for (int j = 0; j < lateral_output.avd_sp_car_past[i].size(); j++) {
//       one_avd_sp_past.PushBack(lateral_output.avd_sp_car_past[i][j],
//       allocator);
//     }
//     avd_sp_past_array.PushBack(one_avd_sp_past, allocator);
//   }
//   rapidjson::SetValueByPointer(later_out_json, "/avd_sp_car_past",
//                                avd_sp_past_array);

//   rapidjson::StringBuffer jsonBuffer;
//   rapidjson::Writer<rapidjson::StringBuffer> writer(jsonBuffer);
//   later_out_json.Accept(writer);
//   plan_msg = std::string(jsonBuffer.GetString(), jsonBuffer.GetSize());
// }

// bool LateralMotionPlannerV1::optimize(
//     int lane_status, bool flag_avoid, bool exist_accident_ahead,
//     bool execute_premove, bool should_suspend, double dist_rblane,
//     const std::array<std::vector<double>, 2> &avoid_car_info,
//     const std::array<std::vector<double>, 2> &avoid_sp_car_info) {
//   // init left_poly_, right_poly_, center_poly_, desired_poly_
//   left_poly_.fill(0.);
//   right_poly_.fill(0.);

//   const auto &fixed_lane =
//       frame_->session()->environmental_model().virtual_lane_manager_;

//   if (fixed_lane.status() == LaneStatusEx::BOTH_MISSING) {
//     std::reverse_copy(fixed_lane.current_lane().begin(),
//                       fixed_lane->current_lane().end(),
//                       desired_poly_.begin());
//     // TODO(@cai) : add log
//   }

//   std::reverse_copy(fixed_lane->current_lane().begin(),
//                     fixed_lane->current_lane().end(),
//                     center_poly_.begin());  // c_poly should

//   update_basic_path(lane_status);

//   // if (check_premove(lane_status)) {
//   //   if (!update_premove_path(lane_status, execute_premove, should_suspend,
//   //                            exist_accident_ahead, avoid_car_info)) {
//   //     premoving_ = false;
//   //     // TODO(@cai) : add log
//   //   }
//   // }

//   // if (check_avoidance_path(lane_status)) {
//   //   if (!update_avoidance_path(lane_status, flag_avoid,
//   exist_accident_ahead,
//   //                              execute_premove, dist_rblane,
//   avoid_car_info,
//   //                              avoid_sp_car_info)) {
//   //     lat_offset_ = 0.;
//   //     // TODO(@cai) : add log
//   //   }
//   // }
//   return true;
// }

// void LateralMotionPlannerV1::calc_desired_path(const CubicPoly &l_poly,
//                                                const CubicPoly &r_poly,
//                                                const double &left_prob,
//                                                const double &right_prob,
//                                                const double &intercept_width,
//                                                CubicPoly &desired_poly) {
//   CubicPoly half_lane_poly{0, 0, 0, intercept_width / 2};

//   if (left_prob + right_prob > 0) {
//     for (size_t i = 0; i < desired_poly.size(); i++) {
//       desired_poly[i] = ((l_poly[i] - half_lane_poly[i]) * left_prob +
//                          (r_poly[i] + half_lane_poly[i]) * right_prob) /
//                         (left_prob + right_prob);
//     }
//   } else {
//     desired_poly.fill(0);
//   }
// }

// double LateralMotionPlannerV1::calc_lane_width_by_dist(
//     const std::vector<double> &left_poly, const std::vector<double>
//     &right_poly, const double &dist_x) {
//   std::vector<double> left_poly_yx, right_poly_yx;
//   left_poly_yx.resize(left_poly.size());
//   right_poly_yx.resize(right_poly.size());
//   std::reverse_copy(left_poly.begin(), left_poly.end(),
//   left_poly_yx.begin()); std::reverse_copy(right_poly.begin(),
//   right_poly.end(),
//                     right_poly_yx.begin());

//   double left_intercept = calc_poly1d(left_poly_yx, dist_x);
//   double right_intercept = calc_poly1d(right_poly_yx, dist_x);

//   if (left_poly_yx.size() >= 2 && right_poly_yx.size() >= 2) {
//     return (left_intercept - right_intercept) /
//            std::sqrt(1 +
//                      std::pow(0.5 * (left_poly_yx[1] + right_poly_yx[1]),
//                      2));
//   } else {
//     return 3.8;
//   }
// }

}  // namespace planning
