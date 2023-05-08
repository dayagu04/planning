
// #include
// "tasks/motion_planners/lateral_motion_planner_real_time/lateral_motion_planner_real_time.h"

#include <iomanip>
#include <vector>

// #include "core/common/trace.h"
// #include "core/modules/common/ego_prediction_utils.h"
// #include "core/modules/context/ego_state.h"
// #include "core/modules/np_functions/mrc_condition.h"
#include "lateral_motion_planner_real_time.h"

namespace planning {

VisionLateralMotionPlanner::VisionLateralMotionPlanner(
    const EgoPlanningConfigBuilder *config_builder,
    const std::shared_ptr<TaskPipelineContext> &pipeline_context)
    : Task(config_builder, pipeline_context) {
  config_ = config_builder->cast<VisionLateralMotionPlannerConfig>();
  name_ = "VisionLateralMotionPlanner";
}

bool VisionLateralMotionPlanner::Execute(planning::framework::Frame *frame) {
  // NTRACE_CALL(7);
  frame_ = frame;

  if (Task::Execute(frame) == false) {
    return false;
  }

  // auto config_builder =
  //     frame->mutable_session()->mutable_planning_context()->config_builder(
  //         planning::common::SceneType::HIGHWAY);

  const auto &session = frame_->session();
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

  // init info
  flane_ = frame_->mutable_session()
               ->mutable_planning_context()
               ->mutable_scenario_state_machine()
               ->get_lane_change_lane_manager()
               ->flane();

  fix_reference_path_ =
      frame_->session()
          ->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(flane_->get_virtual_id());

  ego_frenet_state_ = fix_reference_path_->get_frenet_ego_state();
  ego_cart_state_manager_ =
      frame_->session()->environmental_model().get_ego_state_manager();

  virtual_lane_manager_ =
      frame_->session()->environmental_model().get_virtual_lane_manager();

  set_left_lane_boundary_poly();   // hack left_lane_boundary_poly
  set_right_lane_boundary_poly();  // hack right_lane_boundary_poly

  b_success =
      update(status, flag_avd, accident_ahead, should_premove, should_suspend,
             dist_rblane, avd_car_past, avd_sp_car_past);

  if (!b_success) {
    // TBD : add logs
  }

  // update planning output

  if (!update_planner_output()) {
    // TBD : add logs
  }

  return b_success;
}

bool VisionLateralMotionPlanner::update(
    int status, bool flag_avd, bool accident_ahead, bool should_premove,
    bool should_suspend, double dist_rblane,
    const std::array<std::vector<double>, 2> &avd_car_past,
    const std::array<std::vector<double>, 2> &avd_sp_car_past) {
  std::reverse_copy(flane_->c_poly().begin(), flane_->c_poly().end(),
                    c_poly_.begin());  // c_poly should

  if (flane_->status() == LaneStatusEx::BOTH_MISSING) {
    std::reverse_copy(flane_->c_poly().begin(), flane_->c_poly().end(),
                      d_poly_.begin());
  }

  l_poly_.fill(0);
  r_poly_.fill(0);

  update_basic_path(status);

  if (status == ROAD_LC_LWAIT || status == ROAD_LC_RWAIT ||
      status == ROAD_LC_LBACK || status == ROAD_LC_RBACK ||
      status == INTER_GS_LC_LWAIT || status == INTER_GS_LC_RWAIT ||
      status == INTER_TR_LC_LWAIT || status == INTER_TR_LC_RWAIT ||
      status == INTER_TL_LC_LWAIT || status == INTER_TL_LC_RWAIT) {
    update_premove_path(status, should_premove, should_suspend, accident_ahead,
                        avd_car_past);
  } else {
    premoving_ = false;
  }

  if ((status >= ROAD_NONE && status <= INTER_GS_NONE) ||
      status == INTER_TR_NONE || status == INTER_TL_NONE ||
      status == INTER_GS_LC_LCHANGE || status == INTER_GS_LC_RCHANGE ||
      status == INTER_TR_LC_LCHANGE || status == INTER_TR_LC_RCHANGE ||
      status == INTER_TL_LC_LCHANGE || status == INTER_TL_LC_RCHANGE) {
    update_avoidance_path(status, flag_avd, accident_ahead, should_premove,
                          dist_rblane, avd_car_past, avd_sp_car_past);
  } else {
    lat_offset_ = 0;
  }

  return true;
}

bool VisionLateralMotionPlanner::update_basic_path(const int &status) {
  double reject_prob_thre = 0.5;
  double short_reject_length = 15;

  reject_reason_ = NO_REJECTION;
  l_reject_ = false;
  r_reject_ = false;
  intercept_width_ = 3.8;

  int lane_status = flane_->status();
  double lane_width = flane_->width();
  double min_width = flane_->min_width();
  double max_width = flane_->max_width();

  l_poly_.fill(0);
  r_poly_.fill(0);

  double l_prob, r_prob, intercept_width;

  if (lane_status == LEFT_AVAILABLE) {
    if (lane_width > min_width && lane_width < max_width) {
      std::reverse_copy(flane_->c_poly().begin(),
                        flane_->c_poly().end(),  // flane_->c_poly：0-3
                        d_poly_.begin());        // d_poly_ : 3-0
    } else {
      l_prob = 1;
      r_prob = 0;

      std::reverse_copy(left_lane_boundary_poly().begin(),
                        left_lane_boundary_poly().end(), l_poly_.begin());
      r_poly_.fill(0.0);

      lane_width = clip(lane_width, max_width, min_width);
      intercept_width = lane_width * std::sqrt(1 + l_poly_[2] * l_poly_[2]);

      calc_desired_path(l_poly_, r_poly_, l_prob, r_prob, intercept_width,
                        d_poly_);
    }
  } else if (lane_status == RIGHT_AVAILABLE) {
    if (lane_width > min_width && lane_width < max_width) {
      std::reverse_copy(flane_->c_poly().begin(),
                        flane_->c_poly().end(),  // flane_->c_poly：0-3
                        d_poly_.begin());
    } else {
      l_prob = 0;
      r_prob = 1;

      l_poly_.fill(0.0);
      std::reverse_copy(right_lane_boundary_poly().begin(),
                        right_lane_boundary_poly().end(), r_poly_.begin());

      l_poly_.begin();
      lane_width = clip(lane_width, max_width, min_width);
      intercept_width = lane_width * std::sqrt(1 + r_poly_[2] * r_poly_[2]);

      calc_desired_path(l_poly_, r_poly_, l_prob, r_prob, intercept_width,
                        d_poly_);
    }
  } else {
    l_prob = 1;
    r_prob = 1;
    double l_intercept = flane_->get_left_lane_boundary().poly_coefficient(0);
    double r_intercept = flane_->get_right_lane_boundary().poly_coefficient(0);
    double l_length = flane_->get_left_lane_boundary().end();
    double r_length = flane_->get_right_lane_boundary().end();

    bool l_reject = false;
    bool r_reject = false;
    // bool bias_enable = true;
    bool wide_reject_enable = true;
    bool narrow_reject_enable = true;
    bool short_reject_enable = true;

    double gap_distance = 1.0;

    if (lane_width > max_width) {
      if (std::fabs(r_intercept) < std::fabs(l_intercept) - gap_distance &&
          status != ROAD_LC_LWAIT && status != ROAD_LC_LBACK) {
        l_reject = true;
        reject_reason_ = WIDE_REJECTION_L;
      } else if (std::fabs(r_intercept) >=
                     std::fabs(l_intercept) - gap_distance &&
                 status != ROAD_LC_RWAIT && status != ROAD_LC_RBACK) {
        r_reject = true;
        reject_reason_ = WIDE_REJECTION_R;
      }
    }

    if ((!l_reject && !r_reject) || reject_reason_ == BIAS_L ||
        reject_reason_ == BIAS_R) {
      if (lane_width < min_width) {
        if (frame_->session()->environmental_model().is_on_highway()) {
          double FRONT_DISTANCE_CHECK = 30.0;
          double REAR_DISTANCE_CHECK = -15.0;
          double MIN_WIDTH = 2.2;

          double front_lane_witdh = calc_lane_width_by_dist(
              left_lane_boundary_poly(), right_lane_boundary_poly(),
              FRONT_DISTANCE_CHECK);
          double rear_lane_witdh = calc_lane_width_by_dist(
              left_lane_boundary_poly(), right_lane_boundary_poly(),
              REAR_DISTANCE_CHECK);
          if (lane_width > MIN_WIDTH && front_lane_witdh > MIN_WIDTH &&
              rear_lane_witdh > MIN_WIDTH) {
          } else {
            if (status == ROAD_LC_LCHANGE) {
              l_reject = true;
              reject_reason_ = NARROW_REJECTION;
            } else if (status == ROAD_LC_RCHANGE) {
              r_reject = true;
              reject_reason_ = NARROW_REJECTION;
            } else if (reject_reason_ == BIAS_L || reject_reason_ == BIAS_R) {
              l_reject = true;
              reject_reason_ = NARROW_REJECTION;
            }
          }
        } else {
          l_reject = true;
          reject_reason_ = NARROW_REJECTION;
        }
      }
    }

    if (!l_reject && !r_reject) {
      if (l_length < short_reject_length && r_length > 60) {
        l_reject = true;
        reject_reason_ = SHORT_REJECTION;
      } else if (r_length < short_reject_length && l_length > 60) {
        r_reject = true;
        reject_reason_ = SHORT_REJECTION;
      }
    }

    if (lane_width > min_width && lane_width < max_width && !l_reject &&
        !r_reject) {
      std::reverse_copy(flane_->c_poly().begin(), flane_->c_poly().end(),
                        d_poly_.begin());
    } else {
      lane_width = clip(lane_width, max_width, min_width);

      if (l_reject) {
        l_prob = 0;
        r_prob = 1;

        l_poly_.fill(0.0);
        std::reverse_copy(right_lane_boundary_poly().begin(),
                          right_lane_boundary_poly().end(), r_poly_.begin());

        intercept_width = lane_width * std::sqrt(1 + r_poly_[2] * r_poly_[2]);

        calc_desired_path(l_poly_, r_poly_, l_prob, r_prob, intercept_width,
                          d_poly_);
      } else if (r_reject) {
        l_prob = 1;
        r_prob = 0;

        std::reverse_copy(left_lane_boundary_poly().begin(),
                          left_lane_boundary_poly().end(), l_poly_.begin());

        r_poly_.fill(0.0);

        intercept_width = lane_width * std::sqrt(1 + l_poly_[2] * l_poly_[2]);

        calc_desired_path(l_poly_, r_poly_, l_prob, r_prob, intercept_width,
                          d_poly_);
      } else {
        l_prob = 1;
        r_prob = 1;

        std::reverse_copy(left_lane_boundary_poly().begin(),
                          left_lane_boundary_poly().end(), l_poly_.begin());

        std::reverse_copy(right_lane_boundary_poly().begin(),
                          right_lane_boundary_poly().end(), r_poly_.begin());

        intercept_width =
            lane_width *
            std::sqrt(1 + std::pow(l_poly_[2] + r_poly_[2], 2) / 4);

        calc_desired_path(l_poly_, r_poly_, l_prob, r_prob, intercept_width,
                          d_poly_);
      }
      l_reject_ = l_reject;
      r_reject_ = r_reject;
      intercept_width_ = intercept_width;
    }
  }
}

void VisionLateralMotionPlanner::update_premove_path(
    int status, bool should_premove, bool should_suspend, bool accident_ahead,
    const std::array<std::vector<double>, 2> &avd_car_past) {
  if (flane_->status() == BOTH_MISSING) {
    return;
  }

  double car_width = 2.2;

  double v_ego = ego_cart_state_manager_->ego_v();

  double lane_width = flane_->width();

  l_poly_.fill(0);
  r_poly_.fill(0);

  std::reverse_copy(left_lane_boundary_poly().begin(),
                    left_lane_boundary_poly().end(), l_poly_.begin());

  std::reverse_copy(right_lane_boundary_poly().begin(),
                    right_lane_boundary_poly().end(), r_poly_.begin());

  std::array<double, 3> xp1{10, 20, 30};
  std::array<double, 3> fp1{0.15, 0.3, 0.45};

  double norminal_move =
      0.5 * (lane_width - car_width) - interp(v_ego, xp1, fp1);

  std::array<double, 2> xp2{0.5 * lane_width, 0.5 * (lane_width + car_width)};
  std::array<double, 2> fp2{0, norminal_move};

  double temp_ego = std::min(10.0 / std::max(v_ego, 0.1), 1.0);
  double temp_poly = std::sqrt(1 + std::pow(l_poly_[2] + r_poly_[2], 2) / 4);

  if ((status == ROAD_LC_LWAIT && (should_premove || accident_ahead)) ||
      status == ROAD_LC_LBACK || status == INTER_GS_LC_LWAIT ||
      status == INTER_TR_LC_LWAIT || status == INTER_TL_LC_LWAIT) {
    premoving_ = true;

    if (avd_car_past[0].size() > 0 && avd_car_past[0][5] > 0 &&
        avd_car_past[0][5] < lane_width) {
      lat_offset_ = interp(avd_car_past[0][5], xp2, fp2) * temp_ego * temp_poly;
      d_poly_[3] = c_poly_[3] + lat_offset_;
    } else if (avd_car_past[1].size() > 0 && avd_car_past[1][5] > 0 &&
               avd_car_past[1][5] < lane_width) {
      lat_offset_ = interp(avd_car_past[1][5], xp2, fp2) * temp_ego * temp_poly;
      d_poly_[3] = c_poly_[3] + lat_offset_;

    } else {
      lat_offset_ = norminal_move * temp_poly;

      if (should_suspend && lat_offset_ < -d_poly_[3]) {
        lat_offset_ = -d_poly_[3];
      }

      d_poly_[3] += lat_offset_;
    }
  } else if ((status == ROAD_LC_RWAIT && (should_premove || accident_ahead)) ||
             status == ROAD_LC_RBACK || status == INTER_GS_LC_RWAIT ||
             status == INTER_TR_LC_RWAIT || status == INTER_TL_LC_RWAIT) {
    premoving_ = true;

    if (avd_car_past[0].size() > 0 && avd_car_past[0][6] < 0 &&
        avd_car_past[0][6] > -lane_width) {
      lat_offset_ =
          -interp(avd_car_past[0][6], xp2, fp2) * temp_ego * temp_poly;
      d_poly_[3] = c_poly_[3] + lat_offset_;
    } else if (avd_car_past[1].size() > 0 && avd_car_past[1][6] < 0 &&
               avd_car_past[1][6] > -lane_width_) {
      lat_offset_ =
          -interp(avd_car_past[1][6], xp2, fp2) * temp_ego * temp_poly;
      d_poly_[3] = c_poly_[3] + lat_offset_;
    } else {
      lat_offset_ = -norminal_move * temp_poly;

      if (should_suspend && lat_offset_ > -d_poly_[3]) {
        lat_offset_ = -d_poly_[3];
      }

      d_poly_[3] += lat_offset_;
    }
  } else {
    premoving_ = false;
  }
  LOG_NOTICE("WR: premoving_[%d]", premoving_);
}

bool VisionLateralMotionPlanner::update_avoidance_path(
    int status, bool flag_avd, bool accident_ahead, bool should_premove,
    double dist_rblane, const std::array<std::vector<double>, 2> &avd_car_past,
    const std::array<std::vector<double>, 2> &avd_sp_car_past) {
  const auto &v_cruise = ego_cart_state_manager_->ego_v_cruise();
  double lane_width = flane_->width();
  const auto &min_width = flane_->min_width();
  const auto &max_width = flane_->max_width();
  lane_width = clip(lane_width, max_width, min_width);

  double entrance_lane_width = lane_width;

  // calc lat avoid limit
  double avd_limit_left = 0.15 * lane_width;
  double avd_limit_right = 0.15 * lane_width;
  bool special_lane_type = false;

  const auto &lane_type = flane_->get_lane_type();

  l_poly_.fill(0);
  r_poly_.fill(0);
  if (flane_->status() != LaneStatusEx::BOTH_MISSING) {
    std::reverse_copy(left_lane_boundary_poly().begin(),
                      left_lane_boundary_poly().end(), l_poly_.begin());
    std::reverse_copy(right_lane_boundary_poly().begin(),
                      right_lane_boundary_poly().end(), r_poly_.begin());
  }

  std::array<double, 3> near_car_vrel_v{0.06 * lane_width, 0.02 * lane_width,
                                        0};
  std::array<double, 3> near_car_vrel_bp{-7.5, -3.5, 1};

  std::array<double, 3> near_car_drel_v{0.08 * lane_width, 0.05 * lane_width,
                                        0};
  std::array<double, 3> near_car_drel_bp{0, 20, 60};

  std::array<double, 3> t_gap_vego_v{1.35, 1.55, 2.0};
  std::array<double, 3> t_gap_vego_bp{5, 15, 30};
  std::array<double, 3> avd_vRel_v = {20., 5., 1.};
  std::array<double, 3> avd_vRel_bp = {-7.5, -2.5, 1.};

  double lat_offset = 0;
  double lat_offset1 = 0;
  double lat_offsetl = 0;
  double lat_offsetr = 0;

  int two_nudge_car =
      -1000;  // 这个为啥是int 类型？应该和desired_poly_[3]类型一致？
  int one_nudge_left_car = -1000;
  int one_nudge_right_car = -1000;

  double v_ego = ego_cart_state_manager_->ego_v();

  double l_ego = ego_frenet_state_.l();    // hack ! it's l in cur ref path?
  double safety_dist = 2.0 + v_ego * 0.2;  // magic number
  double t_gap = interp(v_ego, t_gap_vego_bp, t_gap_vego_v);

  double dist_offset = 3.5;
  double car_width = 2.2;
  double avd_normal_thr = lane_width / 2 - 0.3 - car_width * 0.5;
  double pre_str_dist = 100.;

  if (flane_->get_virtual_id() != 0) {  // hack !
    avd_normal_thr = lane_width * 0.5 - 0.1 - car_width * 0.5;
  }

  sb_lane_ = false;
  sb_blane_ = false;
  large_lat_ = false;
  cross_left_solid_line_ = false;
  cross_right_solid_line_ = false;
  force_pause_ = false;
  avd_car_past_ = avd_car_past;
  avd_sp_car_past_ = avd_sp_car_past;
  if (status == ScenarioStateEnum::ROAD_NONE ||
      status == ScenarioStateEnum::ROAD_LC_LCHANGE ||
      status == ScenarioStateEnum::ROAD_LC_RCHANGE ||
      status == ScenarioStateEnum::ROAD_LC_LWAIT ||
      status == ScenarioStateEnum::ROAD_LC_RWAIT ||
      status == ScenarioStateEnum::ROAD_LC_LBACK ||
      status == ScenarioStateEnum::ROAD_LC_RBACK ||
      status == ScenarioStateEnum::ROAD_LB_LBACK ||
      status == ScenarioStateEnum::ROAD_LB_RBACK ||
      status == ScenarioStateEnum::ROAD_LB_LRETURN ||
      status == ScenarioStateEnum::ROAD_LB_RRETURN ||
      status == ScenarioStateEnum::INTER_GS_NONE ||
      status == ScenarioStateEnum::INTER_TR_NONE ||
      status == ScenarioStateEnum::INTER_TL_NONE ||
      status == ScenarioStateEnum::INTER_GS_LC_LCHANGE ||
      status == ScenarioStateEnum::INTER_GS_LC_RCHANGE ||
      status == ScenarioStateEnum::INTER_TR_LC_LCHANGE ||
      status == ScenarioStateEnum::INTER_TR_LC_RCHANGE ||
      status == ScenarioStateEnum::INTER_TL_LC_LCHANGE ||
      status == ScenarioStateEnum::INTER_TL_LC_RCHANGE) {
    if (avd_car_past[0].size() > 0) {
      double plus1 =
          interp(avd_car_past[0][2], near_car_vrel_bp, near_car_vrel_v);
      double plus1_rel =
          interp(avd_car_past[0][3], near_car_drel_bp, near_car_drel_v);
      double lat_compen1 = 0.5 * plus1 + 0.5 * plus1_rel;

      if (avd_car_past[1].size() > 0) {
        double v_near_car2 = v_ego + avd_car_past[1][2];
        double desired_dist2 = dist_offset + v_near_car2 * t_gap;
        double diff_dist_nudge_car = avd_car_past[1][3] - avd_car_past[0][3];
        double diff_vel_nudge_car = avd_car_past[1][2] - avd_car_past[0][2];
        double dist_avd_nudge_car1 = avd_car_past[0][3] + 5.0 + safety_dist;

        double t_avd_car1 = 0;
        // if(avd_car_past[0][2] != 0){
        if (equal_zero(avd_car_past[0][2]) == false) {
          if (avd_car_past[0][2] < -1.0e-3) {
            t_avd_car1 = -dist_avd_nudge_car1 / avd_car_past[0][2];
          } else {
            t_avd_car1 = 5.;
          }
        } else {
          t_avd_car1 = (v_ego < 1) ? 5. : 0.;
        }

        double plus2 =
            interp(avd_car_past[1][2], near_car_vrel_bp, near_car_vrel_v);
        double plus2_rel =
            interp(avd_car_past[1][3], near_car_drel_bp, near_car_drel_v);
        double lat_compen2 = 0.5 * plus2 + 0.5 * plus2_rel;

        if (t_avd_car1 > 0) {
          diff_dist_nudge_car += diff_vel_nudge_car * t_avd_car1;
          if (avd_car_past[0][5] > 0 && avd_car_past[1][5] < 0) {
            if (diff_dist_nudge_car <
                    desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego &&
                avd_car_past[0][5] - avd_car_past[1][6] > 2.8) {
              lat_offset = avd_car_past[0][5] -
                           (avd_car_past[0][5] - avd_car_past[1][6]) / 2;

              if (lat_compen1 > lat_compen2) {
                if (lat_offset - (lat_compen1 - lat_compen2) -
                        avd_car_past[1][6] >=
                    1.4) {
                  lat_offset -= lat_compen1 - lat_compen2;
                } else {
                  lat_offset -= 0.1;
                }
              } else if (lat_compen1 < lat_compen2) {
                if (avd_car_past[0][5] -
                        (lat_offset - (lat_compen1 - lat_compen2)) >=
                    1.4) {
                  lat_offset -= lat_compen1 - lat_compen2;
                } else {
                  lat_offset += 0.1;
                }
              }

              if (lat_offset >= 0) {
                if (avd_normal_thr > 0) {
                  lat_offset = std::min(
                      lat_offset, std::min(avd_normal_thr, avd_car_past[0][9]));
                } else {
                  lat_offset = std::min(
                      lat_offset,
                      std::min(0.5 * lane_width - 0.9, avd_car_past[0][9]));
                }
              } else {
                if (avd_normal_thr > 0) {
                  lat_offset =
                      -std::min(std::fabs(lat_offset),
                                std::min(avd_normal_thr, avd_car_past[0][9]));
                } else {
                  lat_offset = -std::min(
                      std::fabs(lat_offset),
                      std::min(0.5 * lane_width - 0.9, avd_car_past[0][9]));
                }
              }

              curr_time_ = IflyTime::Now_s();
            } else if (avd_car_past[1][6] < -1.5 &&
                       diff_dist_nudge_car >=
                           desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego) {
              if (IflyTime::Now_s() - curr_time_ > 2) {
                lat_offset = 0.5 * (lane_width - car_width / 2 -
                                    std::fabs(avd_car_past[0][5])) +
                             lat_compen1;

                lat_offset = std::max(lat_offset, 0.0);

                if (avd_normal_thr > 0) {
                  lat_offset = -std::min(
                      lat_offset, std::min(avd_normal_thr, avd_car_past[0][9]));
                } else {
                  lat_offset = -std::min(
                      lat_offset,
                      std::min(0.5 * lane_width - 0.9, avd_car_past[0][9]));
                }
              } else {
                lat_offset = -d_poly_[3];
              }
            } else {  // add some comments
              if (avd_car_past[0][2] <= avd_car_past[1][2] ||
                  avd_car_past[0][2] - avd_car_past[1][2] < 2) {
                if (avd_car_past[0][5] != 100) {
                  lat_offset = 0.9 * (lane_width - car_width / 2 -
                                      std::fabs(avd_car_past[0][5])) +
                               lat_compen1;
                }

                lat_offset = std::max(lat_offset, 0.0);
                lat_offset = -std::min(
                    lat_offset,
                    std::min(0.5 * lane_width - 0.9, avd_car_past[0][9]));

                if (avd_car_past[1][6] >= 0 && avd_car_past[1][6] != 100 &&
                    (avd_car_past[1][2] + v_ego < 1.5 ||
                     avd_car_past[0][3] < 0) &&
                    avd_car_past[0][4] > -0.5 && avd_car_past_[0][4] < 0.3 &&
                    avd_car_past[0][1] == 0) {
                  lat_offset =
                      avd_car_past[1][5] -
                      (avd_car_past[1][5] + 1.8 + avd_car_past[0][9]) / 2;
                  if (((virtual_lane_manager_->current_lane_virtual_id() ==
                        virtual_lane_manager_->get_lane_num() - 1) ||
                       (virtual_lane_manager_->current_lane_virtual_id() ==
                            virtual_lane_manager_->get_lane_num() - 2 &&
                        virtual_lane_manager_->get_right_lane() != nullptr &&
                        virtual_lane_manager_->get_right_lane()
                                ->get_lane_type() ==
                            MSD_LANE_TYPE_NON_MOTOR)) &&
                      !frame_->session()
                           ->environmental_model()
                           .is_on_highway() &&
                      ((avd_car_past[0][3] < 15 && v_ego < 5) ||
                       (v_ego < 10 && avd_car_past[0][2] + v_ego < -1) ||
                       avd_car_past[0][3] < 1)) {
                    if (-lat_offset - avd_car_past[0][9] > 0.4) {
                      lat_offset =
                          std::max(std::max(lat_offset,
                                            -0.15 * lane_width - dist_rblane),
                                   std::max(-avd_car_past[0][9],
                                            -2.0 + avd_car_past[0][5]));
                    } else {
                      lat_offset = std::max(
                          lat_offset, std::max(-0.15 * lane_width - dist_rblane,
                                               -2.0 + avd_car_past[0][5]));
                    }

                    if (lat_offset == -avd_car_past[0][9]) {
                      lat_offset = 0;
                    }

                    if (dist_rblane > 1.0) {
                      sb_blane_ = true;
                    }
                    if (std::pow(avd_car_past[0][2] - 1.0, 2) / 4 >
                            avd_car_past[0][3] - 2 &&
                        avd_car_past[0][3] > -5 && d_poly_[3] < -0.7) {
                      lat_offset = -1.3 + avd_car_past[0][5];
                      force_pause_ = true;
                    }
                  } else if ((avd_car_past[1][5] > -0.65 ||
                              avd_car_past[1][7] == 20001) &&
                             (status == INTER_GS_NONE ||
                              status == INTER_TR_NONE ||
                              status == INTER_TL_NONE)) {
                    lat_offset =
                        std::max(lat_offset, -1.6 + avd_car_past[1][5]);
                    sb_lane_ = true;
                  }

                  if (!sb_blane_ || !force_pause_ || !sb_lane_) {
                    lat_offset = std::max(lat_offset, -0.5 * lane_width + 0.9);
                  }
                } else if (avd_car_past[1][6] < 0 &&
                           (avd_car_past[1][2] + v_ego < 0.5 ||
                            avd_car_past[1][3] < 1.0) &&
                           std::fabs(avd_car_past[1][4]) < 0.3 &&
                           ((virtual_lane_manager_->current_lane_virtual_id() ==
                                 virtual_lane_manager_->get_lane_num() - 1 &&
                             ((virtual_lane_manager_->get_current_lane()
                                       ->get_lane_type() !=
                                   MSD_LANE_TYPE_NON_MOTOR &&
                               virtual_lane_manager_
                                       ->current_lane_virtual_id() >= 1) ||
                              (virtual_lane_manager_->get_current_lane()
                                       ->get_lane_type() ==
                                   MSD_LANE_TYPE_NON_MOTOR &&
                               virtual_lane_manager_
                                       ->current_lane_virtual_id() >= 2))) ||
                            (virtual_lane_manager_->current_lane_virtual_id() ==
                                 virtual_lane_manager_->get_lane_num() - 2 &&
                             virtual_lane_manager_->get_right_lane() !=
                                 nullptr &&
                             virtual_lane_manager_->get_right_lane()
                                     ->get_lane_type() ==
                                 MSD_LANE_TYPE_NON_MOTOR &&
                             virtual_lane_manager_->current_lane_virtual_id() >=
                                 1))) {
                  lat_offset = std::min(
                      1.5 + avd_car_past[1][6],
                      std::min(avd_car_past[1][9], avd_car_past[0][5] - 1.4));
                  if (lat_offset >= lane_width / 2 - 1.1) {
                    large_lat_ = true;
                  } else if (std::pow(avd_car_past[1][2] - 1.0, 2) / 4 >
                                 avd_car_past[1][3] - 2 &&
                             avd_car_past[1][3] > -10 && d_poly_[3] < -0.5) {
                    lat_offset = 1.3 + avd_car_past[1][6];
                    force_pause_ = true;
                  }
                } else if ((avd_car_past[0][7] == 20001 ||
                            avd_car_past[1][7] == 20001)) {
                  if (std::fabs(avd_car_past[1][5] - 1.0) >
                      (avd_car_past[0][6] + 1.3)) {
                    lat_offset = avd_car_past[0][6] + 1.3;
                  } else {
                    lat_offset = avd_car_past[1][5] - 1.3;
                  }
                } else {
                  avd_car_past_[1].clear();
                }
              } else {
                if (avd_car_past[1][6] != 100) {
                  lat_offset = 0.9 * (lane_width - car_width / 2 -
                                      std::fabs(avd_car_past[1][6])) +
                               lat_compen1;
                }

                lat_offset = std::max(lat_offset, 0.0);
                lat_offset = std::min(
                    lat_offset,
                    std::min(0.5 * lane_width - 0.9, avd_car_past[0][9]));
                avd_car_past_[0] = avd_car_past[1];
                avd_car_past_[1].clear();
              }
            }
            if (avd_car_past[0][7] == 20001 &&
                // map_info.dist_to_intsect() - avd_car_past[0][3] >= -5 && //
                // hack! map_info.dist_to_intsect() - avd_car_past[0][3] < 50 &&
                (avd_car_past[0][5] <= ((car_width + 0.3) - lane_width / 2))) {
              if (virtual_lane_manager_->current_lane_virtual_id() !=
                      virtual_lane_manager_->get_lane_num() - 1 ||
                  dist_rblane > 1.5) {
                cross_right_solid_line_ = true;
              } else if (virtual_lane_manager_->current_lane_virtual_id() !=
                             0 ||
                         flane_->get_relative_id() == RIGHT_POS) {
                cross_left_solid_line_ = true;
              }
              lat_offset = 0.;
            } else if (avd_car_past[1][7] == 20001 &&
                       // map_info.dist_to_intsect() - avd_car_past[1][3] >= -5
                       // // hack
                       // &&
                       // map_info.dist_to_intsect() - avd_car_past[1][3] < 50
                       // &&
                       (avd_car_past[1][6] >=
                        (lane_width / 2 - (car_width + 0.3)))) {
              if (avd_car_past[1][6] >= 0) {
                if (virtual_lane_manager_->current_lane_virtual_id() !=
                        virtual_lane_manager_->get_lane_num() - 1 ||
                    dist_rblane > 1.5) {
                  cross_right_solid_line_ = true;
                } else if (virtual_lane_manager_->current_lane_virtual_id() !=
                               0 ||
                           flane_->get_relative_id() == RIGHT_POS) {
                  cross_left_solid_line_ = true;
                }
              } else {
                if (virtual_lane_manager_->current_lane_virtual_id() != 0 ||
                    flane_->get_relative_id() == RIGHT_POS) {
                  cross_left_solid_line_ = true;
                } else if (virtual_lane_manager_->current_lane_virtual_id() !=
                               virtual_lane_manager_->get_lane_num() - 1 ||
                           dist_rblane > 1.5) {
                  cross_right_solid_line_ = true;
                }
              }
              lat_offset = 0.;
            }
          } else if (avd_car_past[0][5] < 0 && avd_car_past[1][5] > 0) {
            if (diff_dist_nudge_car <
                    desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego &&
                std::fabs(avd_car_past[0][6] - avd_car_past[1][5]) > 2.8) {
              lat_offset = avd_car_past[0][6] -
                           (avd_car_past[0][6] - avd_car_past[1][5]) / 2;
              if (lat_compen1 > lat_compen2) {
                if (avd_car_past[1][5] -
                        (lat_offset + (lat_compen1 - lat_compen2)) >=
                    1.4) {
                  lat_offset += lat_compen1 - lat_compen2;
                } else {
                  lat_offset += 0.1;
                }
              } else if (lat_compen1 < lat_compen2) {
                if (lat_offset + (lat_compen1 - lat_compen2) -
                        avd_car_past[0][6] >=
                    1.4) {
                  lat_offset += lat_compen1 - lat_compen2;
                } else {
                  lat_offset -= 0.1;
                }
              }

              if (lat_offset >= 0) {
                if (avd_normal_thr > 0) {
                  lat_offset = std::min(
                      lat_offset, std::min(avd_normal_thr, avd_car_past[0][9]));
                } else {
                  lat_offset = std::min(
                      lat_offset,
                      std::min(0.5 * lane_width - 0.9, avd_car_past[0][9]));
                }
              } else {
                if (avd_normal_thr > 0) {
                  lat_offset =
                      -std::min(std::fabs(lat_offset),
                                std::min(avd_normal_thr, avd_car_past[0][9]));
                } else {
                  lat_offset = -std::min(
                      std::fabs(lat_offset),
                      std::min(0.5 * lane_width - 0.9, avd_car_past[0][9]));
                }
              }

              curr_time_ = IflyTime::Now_s();
            } else if (avd_car_past[0][6] < -1.5 &&
                       diff_dist_nudge_car >=
                           desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego) {
              if (IflyTime::Now_s() - curr_time_ > 2) {
                lat_offset = 0.5 * (lane_width - car_width / 2 -
                                    std::fabs(avd_car_past[0][6])) +
                             lat_compen1;
                lat_offset = std::max(lat_offset, 0.0);

                if (avd_normal_thr > 0) {
                  lat_offset = std::min(
                      lat_offset, std::min(avd_normal_thr, avd_car_past[0][9]));
                } else {
                  lat_offset = std::min(
                      lat_offset,
                      std::min(0.5 * lane_width - 0.9, avd_car_past[0][9]));
                }
              } else {
                lat_offset = -d_poly_[3];
              }
            } else {
              if (avd_car_past[0][2] <= avd_car_past[1][2] ||
                  avd_car_past[0][2] - avd_car_past[1][2] < 2) {
                if (avd_car_past[0][6] != 100) {
                  lat_offset = 0.9 * (lane_width - car_width / 2 -
                                      std::fabs(avd_car_past[0][6])) +
                               lat_compen1;
                }

                lat_offset = std::max(lat_offset, 0.0);
                lat_offset = std::min(
                    lat_offset,
                    std::min(0.5 * lane_width - 0.9, avd_car_past[0][9]));

                if (avd_car_past[0][6] >= 0 && avd_car_past[0][6] != 100 &&
                    (avd_car_past[0][2] + v_ego < 1.5 ||
                     avd_car_past[0][3] < 0) &&
                    avd_car_past[0][4] > -0.5 && avd_car_past[0][4] < 0.3 &&
                    avd_car_past[0][1] == 0) {
                  lat_offset =
                      avd_car_past[0][5] -
                      (avd_car_past[0][5] + 1.8 + avd_car_past[0][9]) / 2;

                  if ((virtual_lane_manager_->current_lane_virtual_id() ==
                       virtual_lane_manager_->get_lane_num() - 1) ||
                      (virtual_lane_manager_->current_lane_virtual_id() ==
                           virtual_lane_manager_->get_lane_num() - 2 &&
                       virtual_lane_manager_->get_right_lane() != nullptr &&
                       virtual_lane_manager_->get_right_lane()
                               ->get_lane_type() == MSD_LANE_TYPE_NON_MOTOR)) {
                    if (!frame_->session()
                             ->environmental_model()
                             .is_on_highway() &&
                        // map_info.dist_to_intsect() > 80 &&  // hack
                        // map_info.dist_to_intsect() - avd_car_past[0][3] > 80
                        // &&
                        ((avd_car_past[0][3] < 15 && v_ego < 5) ||
                         (v_ego < 10 && avd_car_past[0][2] + v_ego < -1) ||
                         avd_car_past[0][3] < 1)) {
                      if (-lat_offset - avd_car_past[0][9] > 0.4) {
                        lat_offset =
                            std::max(std::max(lat_offset,
                                              -0.15 * lane_width - dist_rblane),
                                     std::max(-avd_car_past[0][9],
                                              -2.0 + avd_car_past[0][5]));
                      } else {
                        lat_offset =
                            std::max(lat_offset,
                                     std::max(-0.15 * lane_width - dist_rblane,
                                              -2.0 + avd_car_past[0][5]));
                      }

                      if (lat_offset == -avd_car_past[0][9]) {
                        lat_offset = 0;
                      }

                      if (dist_rblane > 1.0) {
                        sb_blane_ = true;
                      }
                      if (std::pow(avd_car_past[0][2] - 1.0, 2) / 4 >
                              avd_car_past[0][3] - 2 &&
                          avd_car_past[0][3] > -5 && d_poly_[3] < -0.7) {
                        lat_offset = -1.3 + avd_car_past[0][5];
                        force_pause_ = true;
                      }
                    } else {
                      lat_offset = 0;
                    }
                  } else if ((avd_car_past[0][5] > -0.65 ||
                              avd_car_past[0][7] == 20001) &&
                             (status == INTER_GS_NONE ||
                              status == INTER_TR_NONE ||
                              status == INTER_TL_NONE)) {
                    lat_offset =
                        std::max(lat_offset, -1.6 + avd_car_past[0][5]);
                    sb_lane_ = true;
                  }

                  if (!sb_blane_ || !force_pause_ || !sb_lane_) {
                    lat_offset = std::max(lat_offset, -0.5 * lane_width + 0.9);
                  }
                } else if (avd_car_past[0][6] < 0 &&
                           (avd_car_past[0][2] + v_ego < 0.5 ||
                            avd_car_past[0][3] < 1) &&
                           std::fabs(avd_car_past[0][4]) < 0.3 &&
                           ((virtual_lane_manager_->current_lane_virtual_id() ==
                                 virtual_lane_manager_->get_lane_num() - 1 &&
                             ((virtual_lane_manager_->get_current_lane()
                                       ->get_lane_type() !=
                                   MSD_LANE_TYPE_NON_MOTOR &&
                               virtual_lane_manager_
                                       ->current_lane_virtual_id() >= 1) ||
                              (virtual_lane_manager_->get_current_lane()
                                       ->get_lane_type() ==
                                   MSD_LANE_TYPE_NON_MOTOR &&
                               virtual_lane_manager_
                                       ->current_lane_virtual_id() >= 2))) ||
                            (virtual_lane_manager_->current_lane_virtual_id() ==
                                 virtual_lane_manager_->get_lane_num() - 2 &&
                             virtual_lane_manager_->get_right_lane() !=
                                 nullptr &&
                             virtual_lane_manager_->get_right_lane()
                                     ->get_lane_type() ==
                                 MSD_LANE_TYPE_NON_MOTOR &&
                             virtual_lane_manager_->current_lane_virtual_id() >=
                                 1))) {
                  lat_offset = std::min(
                      1.5 + avd_car_past[0][6],
                      std::min(avd_car_past[0][9], avd_car_past[1][5] - 1.4));

                  if (lat_offset >= lane_width / 2 - 1.1) {
                    large_lat_ = true;
                  } else if (std::pow(avd_car_past[0][2] - 1, 2) / 4 >
                                 avd_car_past[0][3] - 2 &&
                             avd_car_past[0][3] > -10 && d_poly_[3] < -0.5) {
                    lat_offset = 1.3 + avd_car_past[0][6];
                    force_pause_ = true;
                  }
                } else if ((avd_car_past[0][7] == 20001 ||
                            avd_car_past[1][7] == 20001)
                           //&&map_info.is_in_intersection() // hack
                ) {
                  if (std::fabs(avd_car_past[0][5] - 1.0) >
                      (avd_car_past[1][6] + 1.3)) {  //
                    // need consider more
                    lat_offset = avd_car_past[1][6] + 1.3;
                  } else {
                    lat_offset = avd_car_past[0][5] - 1.3;
                  }
                } else {
                  avd_car_past_[1].clear();
                }
              } else {
                if (avd_car_past[1][5] != 100) {
                  lat_offset = 0.9 * (lane_width - car_width / 2 -
                                      std::fabs(avd_car_past[1][5])) +
                               lat_compen1;
                }

                lat_offset = std::max(lat_offset, 0.0);
                lat_offset = -std::min(
                    lat_offset,
                    std::min(0.5 * lane_width - 0.9, avd_car_past[0][9]));

                avd_car_past_[0] = avd_car_past[1];
                avd_car_past_[1].clear();
              }
            }
            if (avd_car_past[1][7] == 20001 &&
                // map_info.dist_to_intsect() - avd_car_past[1][3] >= -5 && //
                // hack
                // map_info.dist_to_intsect() - avd_car_past[1][3] < 50 &&  //
                // hcak
                (avd_car_past[1][5] <= ((car_width + 0.3) - lane_width / 2))) {
              if (virtual_lane_manager_->current_lane_virtual_id() !=
                      virtual_lane_manager_->get_lane_num() - 1 ||
                  dist_rblane > 1.5) {
                cross_right_solid_line_ = true;
              } else if (virtual_lane_manager_->current_lane_virtual_id() !=
                             0 ||
                         flane_->get_relative_id() == RIGHT_POS) {
                cross_left_solid_line_ = true;
              }
              lat_offset = 0.;
            } else if (avd_car_past[0][7] == 20001 &&
                       //   map_info.dist_to_intsect() - avd_car_past[0][3] >=
                       //   -5 &&  // hcak
                       //  map_info.dist_to_intsect() - avd_car_past[0][3] < 50
                       //  &&
                       (avd_car_past[0][6] >=
                        (lane_width / 2 - (car_width + 0.3)))) {
              if (avd_car_past[0][6] >= 0) {
                if (virtual_lane_manager_->current_lane_virtual_id() !=
                        virtual_lane_manager_->get_lane_num() - 1 ||
                    dist_rblane > 1.5) {
                  cross_right_solid_line_ = true;
                } else if (virtual_lane_manager_->current_lane_virtual_id() !=
                               0 ||
                           flane_->get_relative_id() == RIGHT_POS) {
                  cross_left_solid_line_ = true;
                }
              } else {
                if (virtual_lane_manager_->current_lane_virtual_id() != 0 ||
                    flane_->get_relative_id() == RIGHT_POS) {
                  cross_left_solid_line_ = true;
                } else if (virtual_lane_manager_->current_lane_virtual_id() !=
                               virtual_lane_manager_->get_lane_num() - 1 ||
                           dist_rblane > 1.5) {
                  cross_right_solid_line_ = true;
                }
              }
              lat_offset = 0.;
            }
          } else if (avd_car_past[0][5] > 0 && avd_car_past[1][5] > 0) {
            if (diff_dist_nudge_car <
                desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego) {
              if (avd_car_past[0][5] != 100 || avd_car_past[1][5] != 100) {
                lat_offset = 0.5 * (lane_width - car_width / 2 -
                                    std::fabs(std::min(avd_car_past[0][5],
                                                       avd_car_past[1][5]))) +
                             lat_compen1;
              }
            } else {
              if (avd_car_past[0][5] != 100) {
                lat_offset = 0.5 * (lane_width - car_width / 2 -
                                    std::fabs(avd_car_past[0][5])) +
                             lat_compen1;
              }
            }

            lat_offset = std::max(lat_offset, 0.0);

            if ((virtual_lane_manager_->current_lane_virtual_id() !=
                     virtual_lane_manager_->get_lane_num() - 1 &&
                 (virtual_lane_manager_->get_right_lane() == nullptr ||
                  (virtual_lane_manager_->get_right_lane() != nullptr &&
                   virtual_lane_manager_->get_right_lane()->get_lane_type() !=
                       MSD_LANE_TYPE_NON_MOTOR))) ||
                avd_limit_left == 0.2 ||
                (avd_car_past[0][2] + v_ego >= 1.5 && avd_car_past[0][3] >= 0)
                //||
                //  map_info.dist_to_intsect() <= 80 ||  // hack
                // map_info.dist_to_intsect() - avd_car_past[0][3] <= 80 // hack
            ) {
              if (avd_normal_thr > 0) {
                lat_offset =
                    -std::min(std::min(lat_offset, avd_normal_thr),
                              std::min(avd_car_past[0][9], avd_limit_left));
              } else {
                lat_offset =
                    -std::min(std::min(lat_offset, 0.5 * lane_width - 0.9),
                              std::min(avd_car_past[0][9], avd_limit_left));
              }
              if ((virtual_lane_manager_->current_lane_virtual_id() !=
                   virtual_lane_manager_->get_lane_num() - 1) &&
                  avd_car_past[0][2] + v_ego < 0.5 &&

                  abs(avd_car_past[0][4]) < 0.2 &&
                  //! isRedLightStop && // hack!
                  ((avd_car_past[0][5] <=
                    ((car_width + 0.3) - lane_width / 2)) ||
                   (avd_car_past[1][5] <=
                    ((car_width + 0.3) - lane_width / 2)))) {
                cross_right_solid_line_ = true;
                lat_offset = 0.;
              }
              if ((avd_car_past[0][7] == 20001 &&

                   (avd_car_past[0][5] <=
                    ((car_width + 0.3) - lane_width / 2))) ||
                  (avd_car_past[1][7] == 20001 &&

                   (avd_car_past[1][5] <=
                    ((car_width + 0.3) - lane_width / 2)))) {
                if (virtual_lane_manager_->current_lane_virtual_id() !=
                        virtual_lane_manager_->get_lane_num() - 1 ||
                    dist_rblane > 1.5) {
                  cross_right_solid_line_ = true;
                } else if (virtual_lane_manager_->current_lane_virtual_id() !=
                               0 ||
                           flane_->get_relative_id() == RIGHT_POS) {
                  cross_left_solid_line_ = true;
                }
                lat_offset = 0.;
              }

              if ((status == INTER_GS_NONE || status == INTER_TR_NONE ||
                   status == INTER_TL_NONE) &&
                  avd_car_past[0][5] < 1.1 &&
                  (avd_car_past[0][2] + v_ego < 1.5 ||
                   avd_car_past[0][3] < 0) &&
                  avd_car_past[0][4] > -0.5 && avd_car_past[0][4] < 0.3 &&
                  avd_car_past[0][1] == 0) {
                lat_offset =
                    avd_car_past[0][5] -
                    (avd_car_past[0][5] + 2.0 + avd_car_past[0][9]) / 2;
                lat_offset = std::max(lat_offset, -1.6 + avd_car_past[0][5]);

                sb_lane_ = true;
              }
            } else if (!frame_->session()
                            ->environmental_model()
                            .is_on_highway() &&
                       abs(avd_car_past[0][4]) < 0.45) {
              if ((avd_car_past[0][3] < 15 && v_ego < 5) ||
                  (v_ego < 10 && avd_car_past[0][2] + v_ego < -1) ||
                  avd_car_past[0][3] < 1) {
                lat_offset = -std::min(
                    std::min(lat_offset, 0.1 * lane_width + dist_rblane),
                    std::min(avd_car_past[0][9], 2 - avd_car_past[0][5]));
              }

              if (dist_rblane > 1.0 &&
                  0.5 * lane_width +
                          std::min(avd_car_past[0][5], avd_car_past[1][5]) <
                      3.0 + 0.02 * std::fabs(std::min(avd_car_past[0][2],
                                                      avd_car_past[1][2]))) {
                sb_blane_ = true;
              }

              if (std::pow(avd_car_past[0][2] - 1, 2) / 4 >
                      avd_car_past[0][3] - 2 &&
                  avd_car_past[0][3] > -5 && d_poly_[3] < -0.7) {
                lat_offset = -1.3 + avd_car_past[0][5];
                force_pause_ = true;
              }
            }

            if (lat_offset > 0) {
              lat_offset = -d_poly_[3];
            }

            if (flane_->status() != LaneStatusEx::BOTH_MISSING &&
                r_poly_[3] < 0 && r_poly_[3] > -1.0 &&
                (virtual_lane_manager_->current_lane_virtual_id() ==
                     virtual_lane_manager_->get_lane_num() - 1 &&
                 (dist_rblane < 0.5 || avd_limit_left == 0.2)) &&
                !special_lane_type) {
              lat_offset = std::min(lat_offset + r_poly_[3] + 1.0, 0.0);
            }
          } else if (avd_car_past[0][5] < 0 && avd_car_past[1][5] < 0) {
            if (diff_dist_nudge_car <
                desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego) {
              if (avd_car_past[0][6] != 100 && avd_car_past[1][6] != 100 &&
                  avd_car_past[0][6] < 0 && avd_car_past[1][6] < 0) {
                lat_offset = 0.5 * (lane_width - car_width / 2 -
                                    std::fabs(std::max(avd_car_past[0][6],
                                                       avd_car_past[1][6]))) +
                             lat_compen1;
              } else if (avd_car_past[0][6] != 100 && avd_car_past[0][6] < 0) {
                lat_offset = 0.5 * (lane_width - car_width / 2 -
                                    std::fabs(avd_car_past[0][6])) +
                             lat_compen1;
              } else if (avd_car_past[1][6] != 100 && avd_car_past[1][6] < 0) {
                lat_offset = 0.5 * (lane_width - car_width / 2 -
                                    std::fabs(avd_car_past[1][6])) +
                             lat_compen1;
              }

              if ((status == INTER_GS_NONE || status == INTER_TR_NONE ||
                   status == INTER_TL_NONE) &&
                  (avd_car_past[0][6] > 0 ||
                   (avd_car_past[1][7] == 20001 && avd_car_past[1][6] > 0)) &&
                  (avd_car_past[0][2] + v_ego < 1.5 ||
                   avd_car_past[0][3] < 0) &&
                  avd_car_past[0][4] > -0.5 && avd_car_past[0][4] < 0.3 &&
                  avd_car_past[0][1] == 0) {
                if (((avd_car_past[0][7] != 20001 || status != INTER_TL_NONE
                      //||
                      //   map_info.dist_to_last_intsect() - avd_car_past[0][3]
                      //   <=
                      //        15// hack
                      ) &&
                     avd_car_past[0][5] - l_ego > -0.6) ||
                    (avd_car_past[0][7] == 20001 &&
                     (status == INTER_TL_NONE &&
                          std::fabs(std::min(avd_car_past[0][5],
                                             avd_car_past[1][5])) +
                                  0.3 <
                              avd_car_past[0][6] ||
                      (avd_car_past[1][7] != 20001 &&
                       avd_car_past[1][5] - l_ego >
                           -1.)))) {  // need a history record for
                                      // avd_car_past[0][5]
                  lat_offset =
                      std::min(avd_car_past[0][5], avd_car_past[1][5]) -
                      (std::min(avd_car_past[0][5], avd_car_past[1][5]) + 2.0 +
                       avd_car_past[0][9]) /
                          2;
                  lat_offset = std::max(
                      lat_offset,
                      -1.6 + std::min(avd_car_past[0][5], avd_car_past[1][5]));

                  sb_lane_ = true;
                } else if (avd_car_past[0][6] < 0.65 ||
                           avd_car_past[0][7] == 20001) {
                  lat_offset = std::min(
                      std::max(avd_car_past[0][6], avd_car_past[1][6]) -
                          (std::max(avd_car_past[0][6], avd_car_past[1][6]) -
                           1.8 - avd_car_past[0][9]) /
                              2,
                      1.6 - std::max(avd_car_past[0][6], avd_car_past[1][6]));
                  lat_offset = std::min(lat_offset, 1.1);
                }
              }

              if (avd_car_past[0][6] >= 0 && avd_car_past[0][6] != 100 &&
                  (avd_car_past[0][2] + v_ego < 1.5 ||
                   avd_car_past[0][3] < 0) &&
                  std::fabs(avd_car_past[0][4]) < 0.45 &&
                  ((virtual_lane_manager_->current_lane_virtual_id() ==
                    virtual_lane_manager_->get_lane_num() - 1) ||
                   (virtual_lane_manager_->current_lane_virtual_id() ==
                        virtual_lane_manager_->get_lane_num() - 2 &&
                    virtual_lane_manager_->get_right_lane() != nullptr &&
                    virtual_lane_manager_->get_right_lane()->get_lane_type() ==
                        MSD_LANE_TYPE_NON_MOTOR))) {
                if (((!frame_->session()->environmental_model().is_on_highway()
                      //    &&  // hack
                      //  map_info.dist_to_intsect() > 80 &&
                      //  map_info.dist_to_intsect() - avd_car_past[0][3] > 80
                      ) ||
                     accident_ahead) &&
                    ((avd_car_past[0][3] < 15 && v_ego < 5) ||
                     (v_ego < 10 && avd_car_past[0][2] + v_ego < -1) ||
                     avd_car_past[0][3] < 1)) {
                  lat_offset =
                      avd_car_past[0][5] -
                      (avd_car_past[0][5] + 2.0 + avd_car_past[0][9]) / 2;

                  if (-lat_offset - avd_car_past[0][9] > 0.4) {
                    lat_offset = std::max(
                        std::max(lat_offset, -0.15 * lane_width - dist_rblane),
                        std::max(-avd_car_past[0][9],
                                 -2.0 + avd_car_past[0][5]));
                  } else {
                    lat_offset = std::max(
                        std::max(lat_offset, -0.15 * lane_width - dist_rblane),
                        -2.0 + avd_car_past[0][5]);
                  }

                  if (lat_offset == -avd_car_past[0][9]) {
                    lat_offset = 0;
                  }

                  if (dist_rblane > 1.0) {
                    sb_blane_ = true;
                  }
                  if (std::pow(avd_car_past[0][2] - 1, 2) / 4 >
                          avd_car_past[0][3] - 2 &&
                      avd_car_past[0][3] > -5 && d_poly_[3] < -0.7) {
                    lat_offset = -1.3 + avd_car_past[0][5];
                    force_pause_ = true;
                  }
                } else {
                  lat_offset = 0;
                }
              }
            } else {
              if (avd_car_past[0][6] != 100) {
                lat_offset = 0.5 * (lane_width - car_width / 2 -
                                    std::fabs(avd_car_past[0][6])) +
                             lat_compen1;

                if (avd_car_past[0][6] >= 0 &&
                    (avd_car_past[0][2] + v_ego < 1.5 ||
                     avd_car_past[0][3] < 0) &&
                    std::fabs(avd_car_past[0][4]) < 0.45 &&
                    ((virtual_lane_manager_->current_lane_virtual_id() ==
                      virtual_lane_manager_->get_lane_num() - 1) ||
                     (virtual_lane_manager_->current_lane_virtual_id() ==
                          virtual_lane_manager_->get_lane_num() - 2 &&
                      virtual_lane_manager_->get_right_lane() != nullptr &&
                      virtual_lane_manager_->get_right_lane()
                              ->get_lane_type() == MSD_LANE_TYPE_NON_MOTOR))) {
                  if (!frame_->session()
                           ->environmental_model()
                           .is_on_highway() &&
                      // map_info.dist_to_intsect() > 80 &&  //hack
                      // map_info.dist_to_intsect() - avd_car_past[0][3] > 80 &&
                      ((avd_car_past[0][3] < 15 && v_ego < 5) ||
                       (v_ego < 10 && avd_car_past[0][2] + v_ego < -1) ||
                       avd_car_past[0][3] < 1)) {
                    lat_offset =
                        avd_car_past[0][5] -
                        (avd_car_past[0][5] + 2.3 + avd_car_past[0][9]) / 2;

                    if (-lat_offset - avd_car_past[0][9] > 0.4) {
                      lat_offset =
                          std::max(std::max(lat_offset,
                                            -0.15 * lane_width - dist_rblane),
                                   std::max(-avd_car_past[0][9],
                                            -2.0 + avd_car_past[0][5]));
                    } else {
                      lat_offset =
                          std::max(std::max(lat_offset,
                                            -0.15 * lane_width - dist_rblane),
                                   -2.0 + avd_car_past[0][5]);
                    }

                    if (lat_offset == -avd_car_past[0][9]) {
                      lat_offset = 0;
                    }

                    if (dist_rblane > 1.0) {
                      sb_blane_ = true;
                    }
                    if (std::pow(avd_car_past[0][2] - 1, 2) / 4 >
                            avd_car_past[0][3] - 2 &&
                        avd_car_past[0][3] > -5 && d_poly_[3] < -0.7) {
                      lat_offset = -1.3 + avd_car_past[0][5];
                      force_pause_ = true;
                    }
                  } else {
                    lat_offset = 0;
                  }
                }
              }
            }

            if (avd_car_past[0][6] < 0 || avd_car_past[1][6] < 0) {
              lat_offset = std::max(lat_offset, 0.0);

              if ((avd_car_past[0][2] + v_ego < 0.5 ||
                   avd_car_past[0][3] < 1) &&
                  std::fabs(avd_car_past[0][4]) < 0.3 &&
                  std::fabs(avd_car_past[1][4]) < 0.3 &&
                  ((virtual_lane_manager_->current_lane_virtual_id() ==
                        virtual_lane_manager_->get_lane_num() - 1 &&
                    ((virtual_lane_manager_->get_current_lane()
                              ->get_lane_type() != MSD_LANE_TYPE_NON_MOTOR &&
                      virtual_lane_manager_->current_lane_virtual_id() >= 1) ||
                     (virtual_lane_manager_->get_current_lane()
                              ->get_lane_type() == MSD_LANE_TYPE_NON_MOTOR &&
                      virtual_lane_manager_->current_lane_virtual_id() >=
                          2))) ||
                   (virtual_lane_manager_->current_lane_virtual_id() ==
                        virtual_lane_manager_->get_lane_num() - 2 &&
                    virtual_lane_manager_->get_right_lane() != nullptr &&
                    virtual_lane_manager_->get_right_lane()->get_lane_type() ==
                        MSD_LANE_TYPE_NON_MOTOR &&
                    virtual_lane_manager_->current_lane_virtual_id() >= 1))) {
                lat_offset = std::min(lat_offset, std::min(avd_car_past[0][9],
                                                           avd_car_past[1][9]));
                if (lat_offset >= lane_width / 2 - 1.1) {
                  large_lat_ = true;
                } else if (std::pow(avd_car_past[0][2] - 1, 2) / 4 >
                               avd_car_past[0][3] - 2 &&
                           avd_car_past[0][3] > -10 && d_poly_[3] < -0.5) {
                  lat_offset = 1.3 + avd_car_past[0][6];
                  force_pause_ = true;
                }
              } else {
                if (avd_normal_thr > 0) {
                  lat_offset =
                      std::min(std::min(lat_offset, avd_normal_thr),
                               std::min(avd_car_past[0][9], avd_limit_right));
                } else {
                  lat_offset =
                      std::min(std::min(lat_offset, 0.5 * lane_width - 0.9),
                               std::min(avd_car_past[0][9], avd_limit_right));
                }
              }
            }

            if ((virtual_lane_manager_->current_lane_virtual_id() !=
                     virtual_lane_manager_->get_lane_num() - 1 &&
                 virtual_lane_manager_->current_lane_virtual_id() != 0) &&
                avd_car_past[0][2] + v_ego < 0.5 &&
                // map_info.dist_to_intsect() > 0 &&
                // map_info.dist_to_intsect() - avd_car_past[0][3] < 50 &&
                abs(avd_car_past[0][4]) < 0.2 &&
                //! isRedLightStop && // hack
                ((avd_car_past[0][6] >= (lane_width / 2 - (car_width + 0.3))) ||
                 (avd_car_past[1][6] >=
                  (lane_width / 2 - (car_width + 0.3)))) &&
                avd_car_past[0][6] < 0 && avd_car_past[1][6] < 0) {
              cross_left_solid_line_ = true;
              lat_offset = 0.;
            }
            if ((avd_car_past[0][7] == 20001 &&
                 //   map_info.dist_to_intsect() - avd_car_past[0][3] >= -5 &&
                 //   map_info.dist_to_intsect() - avd_car_past[0][3] < 50 &&
                 (avd_car_past[0][6] >=
                  (lane_width / 2 - (car_width + 0.3)))) ||
                (avd_car_past[1][7] == 20001 &&
                 //    map_info.dist_to_intsect() - avd_car_past[1][3] >= -5 &&
                 //    map_info.dist_to_intsect() - avd_car_past[1][3] < 50 &&
                 (avd_car_past[1][6] >=
                  (lane_width / 2 - (car_width + 0.3))))) {
              if (virtual_lane_manager_->current_lane_virtual_id() != 0 ||
                  flane_->get_relative_id() == RIGHT_POS) {
                cross_left_solid_line_ = true;
              } else if (virtual_lane_manager_->current_lane_virtual_id() !=
                         virtual_lane_manager_->get_lane_num() - 1) {
                cross_right_solid_line_ = true;
              }
              lat_offset = 0.;
            }

            if (lat_offset < 0 && dist_rblane < 0.5 && sb_lane_ == false) {
              lat_offset = -d_poly_[3];
            }

            if (flane_->status() != LaneStatusEx::BOTH_MISSING &&
                l_poly_[3] > 0 && l_poly_[3] < 1.0 &&
                virtual_lane_manager_->current_lane_virtual_id() == 0 &&
                !special_lane_type && !large_lat_ && !force_pause_) {
              lat_offset = std::max(lat_offset + l_poly_[3] - 1.0, 0.0);
            }
          }
        }

        if (avd_car_past[0][7] > 0.) {
          pre_str_dist = interp(avd_car_past[0][2], avd_vRel_bp, avd_vRel_v);
          if (avd_car_past[0][0] != -1 && avd_car_past[0][0] != -2) {
            if (lat_offset >= 0) {
              lat_offset = std::min(
                  (avd_car_past[0][7] - (avd_car_past[0][3] - pre_str_dist)) /
                      avd_car_past[0][7] * lat_offset,
                  lat_offset);
            } else {
              lat_offset = std::max(
                  (avd_car_past[0][7] - (avd_car_past[0][3] - pre_str_dist)) /
                      avd_car_past[0][7] * lat_offset,
                  lat_offset);
            }
          } else {
            if (lat_offset >= 0.) {
              lat_offset =
                  std::min((5. / std::max(std::fabs(avd_car_past[0][3]), 5.)) *
                               lat_offset,
                           lat_offset);
            } else {
              lat_offset =
                  std::max((5. / std::max(std::fabs(avd_car_past[0][3]), 5.)) *
                               lat_offset,
                           lat_offset);
            }
          }
          if (lat_offset < 0. && d_poly_[3] + lat_offset < -0.3) {
            lat_offset = -d_poly_[3] - 0.3;
          } else if (lat_offset >= 0. && d_poly_[3] + lat_offset > 0.3) {
            lat_offset = -d_poly_[3] + 0.3;
          }
        }

        if (dist_rblane == 0 && (lat_offset > 0.8 || lat_offset < -0.8)) {
        } else {
          if (std::fabs(lat_offset1) >= std::fabs(d_poly_[3]) &&
              ((d_poly_[3] > 0 && lat_offset < 0 &&
                d_poly_[3] + lat_offset > 0) ||
               (d_poly_[3] < 0 && lat_offset > 0 &&
                d_poly_[3] + lat_offset < 0))) {
            d_poly_[3] = 0;
          } else if (std::pow(avd_car_past[0][2] - 1, 2) / 4 >
                         avd_car_past[0][3] - 2 &&
                     avd_car_past[0][3] > -5) {
            if (avd_car_past[0][5] > 0 &&
                std::fabs(lat_offset - avd_car_past[0][5]) < 1.3) {
              lat_offset = std::max(-1.3 + avd_car_past[0][5],
                                    -0.15 * lane_width - dist_rblane);
              force_pause_ = true;
            } else if (avd_car_past[0][5] < 0 && avd_car_past[0][6] < 0 &&
                       std::fabs(lat_offset - avd_car_past[0][6]) < 1.3) {
              lat_offset = 1.3 + avd_car_past[0][6];
              if (virtual_lane_manager_->current_lane_virtual_id() == 0) {
                lat_offset =
                    std::min(1.3 + avd_car_past[0][6], 0.5 * lane_width - 0.9);
              }
              force_pause_ = true;
            } else {
              d_poly_[3] += lat_offset;
              two_nudge_car = d_poly_[3];  // ?
            }
          } else {
            d_poly_[3] += lat_offset;
            two_nudge_car = d_poly_[3];
          }
        }
      } else if (avd_car_past[0][5] > 0) {
        if ((int)avd_car_past[0][1] == -100) {
          lat_offset =
              -0.15 * lane_width * std::fabs(avd_car_past[0][10]) / 3.2;

          if (lane_type == 3 || std::fabs(d_poly_[1]) > 0.0001 ||
              (virtual_lane_manager_->get_lane_num() > 1 &&
               virtual_lane_manager_->current_lane_virtual_id() ==
                   virtual_lane_manager_->get_lane_num() - 1)) {
            lat_offset = 0.8 * lat_offset;
          }

          if (avd_limit_left == 0.2) {
            lat_offset = std::max(lat_offset, -avd_limit_left);
          }

          if (flane_->status() != LaneStatusEx::BOTH_MISSING &&
              r_poly_[3] < 0 && r_poly_[3] > -1.0 && !special_lane_type) {
            lat_offset += r_poly_[3] + 1.0;
          }

          if (d_poly_[3] > 1.0) {
            lat_offset = -d_poly_[3] - 0.5;
          }
        } else if (((int)avd_car_past[0][0] == 0 &&
                    avd_car_past[0][3] > -6.0) ||
                   ((int)avd_car_past[0][0] != 0 &&
                    avd_car_past[0][3] >= -3.0)) {
          if (status != INTER_GS_NONE && status != INTER_TR_NONE &&
              status != INTER_TL_NONE) {
            if (avd_car_past[0][5] != 100) {
              lat_offset = 0.5 * (lane_width - car_width / 2 -
                                  std::fabs(avd_car_past[0][5])) +
                           lat_compen1;
            }

            lat_offset = std::max(lat_offset, 0.0);
            if (avd_normal_thr > 0) {
              lat_offset =
                  -std::min(std::min(lat_offset, avd_normal_thr),
                            std::min(avd_car_past[0][9], avd_limit_left));
            } else {
              lat_offset =
                  -std::min(std::min(lat_offset, 0.5 * lane_width - 0.9),
                            std::min(avd_car_past[0][9], avd_limit_left));
            }

            if (avd_car_past[0][5] < 1.5) {
              if ((virtual_lane_manager_->current_lane_virtual_id() !=
                       virtual_lane_manager_->get_lane_num() - 1 &&
                   (virtual_lane_manager_->get_right_lane() == nullptr ||
                    (virtual_lane_manager_->get_right_lane() != nullptr &&
                     virtual_lane_manager_->get_right_lane()->get_lane_type() !=
                         MSD_LANE_TYPE_NON_MOTOR))) ||
                  (avd_car_past[0][2] + v_ego >= 1.5 && avd_car_past[0][3] >= 0)
                  // ||
                  //  map_info.dist_to_intsect() <= 80 ||
                  //  map_info.dist_to_intsect() - avd_car_past[0][3] <= 80  //
                  //  hack
              ) {
                lat_offset =
                    avd_car_past[0][5] -
                    (avd_car_past[0][5] + 1.8 + avd_car_past[0][9]) / 2;
                lat_offset = std::max(lat_offset, -0.5 * lane_width + 0.9);
              } else if (!frame_->session()
                              ->environmental_model()
                              .is_on_highway() &&
                         abs(avd_car_past[0][4]) < 0.45) {
                if ((avd_car_past[0][3] < 15 && v_ego < 5) ||
                    (v_ego < 10 && avd_car_past[0][2] + v_ego < -1) ||
                    avd_car_past[0][3] < 1) {
                  lat_offset =
                      avd_car_past[0][5] -
                      (avd_car_past[0][5] + 1.8 + avd_car_past[0][9]) / 2;

                  lat_offset = std::max(
                      lat_offset, std::max(-0.15 * lane_width - dist_rblane,
                                           -2.0 + avd_car_past[0][5]));
                }

                if (dist_rblane > 1.0 && 0.5 * lane_width + avd_car_past[0][5] <
                                             3.0 + 0.02 * avd_car_past[0][2]) {
                  sb_blane_ = true;
                }
                if (std::pow(avd_car_past[0][2] - 1, 2) / 4 >
                        avd_car_past[0][3] - 2 &&
                    avd_car_past[0][3] > -5 && d_poly_[3] < -0.7) {
                  lat_offset = -1.3 + avd_car_past[0][5];
                  force_pause_ = true;
                }
              }

              if (virtual_lane_manager_->get_lane_num() > 1 &&
                  virtual_lane_manager_->current_lane_virtual_id() ==
                      virtual_lane_manager_->get_lane_num() - 1 &&
                  dist_rblane < 0.5) {
                lat_offset *= 0.8;
              }
            }

            if (avd_limit_left == 0.2) {
              lat_offset = std::max(lat_offset, -avd_limit_left);
            }

            if (flane_->status() != LaneStatusEx::BOTH_MISSING &&
                r_poly_[3] < 0 && r_poly_[3] > -1.0 &&
                (virtual_lane_manager_->current_lane_virtual_id() ==
                     virtual_lane_manager_->get_lane_num() - 1 &&
                 (dist_rblane < 0.5 || avd_limit_left == 0.2)) &&
                !special_lane_type) {
              lat_offset = std::min(lat_offset + r_poly_[3] + 1.0, 0.0);
            }
          } else {
            if (avd_car_past[0][5] != 100) {
              lat_offset = 0.5 * (lane_width - car_width / 2 -
                                  std::fabs(avd_car_past[0][5])) +
                           lat_compen1;
            }

            lat_offset = std::max(lat_offset, 0.0);
            lat_offset = -std::min(lat_offset, std::min(0.5 * lane_width - 0.9,
                                                        avd_car_past[0][9]));

            if (avd_car_past[0][5] < 1.5) {
              lat_offset = avd_car_past[0][5] -
                           (avd_car_past[0][5] + 1.8 + avd_car_past[0][9]) / 2;

              if (avd_car_past[0][5] >= 1.1) {
                lat_offset = std::max(lat_offset, -0.5 * lane_width + 0.9);
              } else if (lane_width <= 3.8) {
                if ((avd_car_past[0][2] + v_ego < 1.5 ||
                     avd_car_past[0][3] < 0) &&
                    avd_car_past[0][4] > -0.5 && avd_car_past[0][4] < 0.3 &&
                    avd_car_past[0][1] == 0) {
                  lat_offset = std::max(lat_offset, -1.6 + avd_car_past[0][5]);
                  sb_lane_ = true;
                } else {
                  if (avd_normal_thr > 0) {
                    lat_offset = std::max(lat_offset, -avd_normal_thr);
                  } else {
                    lat_offset = std::max(lat_offset, -0.5 * lane_width + 0.9);
                  }
                }
              } else {
                lat_offset =
                    std::max(lat_offset, -0.24 * lane_width / 4.4 * lane_width);
                if ((avd_car_past[0][2] + v_ego < 1.5 ||
                     avd_car_past[0][3] < 0) &&
                    avd_car_past[0][4] > -0.5 && avd_car_past[0][4] < 0.3 &&
                    avd_car_past[0][1] == 0) {
                  lat_offset = std::max(lat_offset, -1.8 + avd_car_past[0][5]);
                  sb_lane_ = true;
                }
              }
            }
          }

          if ((virtual_lane_manager_->current_lane_virtual_id() !=
               virtual_lane_manager_->get_lane_num() - 1) &&
              avd_car_past[0][2] + v_ego < 0.5 &&

              abs(avd_car_past[0][4]) < 0.2 &&
              (avd_car_past[0][5] <= ((car_width + 0.3) - lane_width / 2))) {
            cross_right_solid_line_ = true;
            lat_offset = 0.;
          }
          if (avd_car_past[0][7] == 20001 &&

              (avd_car_past[0][5] <= ((car_width + 0.3) - lane_width / 2))) {
            if (virtual_lane_manager_->current_lane_virtual_id() !=
                    virtual_lane_manager_->get_lane_num() - 1 ||
                dist_rblane > 1.5) {
              cross_right_solid_line_ = true;
            } else if (virtual_lane_manager_->current_lane_virtual_id() != 0 ||
                       flane_->get_relative_id() == RIGHT_POS) {
              cross_left_solid_line_ = true;
            }
            lat_offset = 0.;
          }

          lat_offsetl = lat_offset;

          pre_str_dist = interp(avd_car_past[0][2], avd_vRel_bp, avd_vRel_v);
          if (avd_car_past[0][0] > 0 && avd_car_past[0][7] != 0.) {
            if (avd_car_past[0][7] >= avd_car_past[0][3]) {
              lat_offset = std::max(
                  (avd_car_past[0][7] - (avd_car_past[0][3] - pre_str_dist)) /
                      avd_car_past[0][7] * lat_offset,
                  lat_offset);
            } else if (avd_car_past[0][3] != 0.) {
              lat_offset =
                  (1 - avd_car_past[0][7] / avd_car_past[0][3]) * lat_offset;
            }
          } else {
            if (avd_car_past[0][5] < 1.5) {
              lat_offset =
                  std::max((5. / std::max(std::abs(avd_car_past[0][3]), 5.)) *
                               lat_offset,
                           lat_offset);
            } else {
              lat_offset =
                  std::max((5. / std::max(std::abs(avd_car_past[0][3]), 5.)) *
                               lat_offset,
                           lat_offset);
            }
          }
          if (d_poly_[3] + lat_offset < -0.3) {
            lat_offset = -d_poly_[3] - 0.3;
          }
        } else {
          lat_offset = std::min(-d_poly_[3], 0.);
          if (std::fabs(lat_offset) > 0.5 * lane_width - 0.9 &&
              dist_rblane < 0.5) {
            lat_offset = -0.5 * lane_width + 0.9;
          }
        }
        if ((dist_rblane == 0 || sb_lane_ == false) &&
            (lat_offset < -0.8 || lat_offset > 0.8)) {
        } else {
          if (std::fabs(lat_offsetl) >= std::fabs(d_poly_[3]) &&
              d_poly_[3] > 0 && d_poly_[3] + lat_offset > 0) {
            d_poly_[3] = 0;
          } else if (std::pow(avd_car_past[0][2] - 1, 2) / 4 >
                         avd_car_past[0][3] - 2 &&
                     avd_car_past[0][3] > -5) {
            if (avd_car_past[0][5] > 0 &&
                std::fabs(lat_offset - avd_car_past[0][5]) < 1.3) {
              lat_offset = std::max(-1.3 + avd_car_past[0][5],
                                    -0.15 * lane_width - dist_rblane);
              force_pause_ = true;
            } else {
              d_poly_[3] += lat_offset;
              two_nudge_car = d_poly_[3];
            }
          } else {
            d_poly_[3] += lat_offset;
          }

          one_nudge_left_car = d_poly_[3];
        }
      } else {
        if ((int)avd_car_past[0][1] == -200) {
          lat_offset = 0.15 * lane_width * std::fabs(avd_car_past[0][10]) / 3.2;

          if (lane_type == MSD_LANE_TYPE_PARKING ||
              std::fabs(d_poly_[1]) > 0.0001 ||
              (virtual_lane_manager_->get_lane_num() > 1 &&
               virtual_lane_manager_->current_lane_virtual_id() == 0)) {
            lat_offset *= 0.8;
          }

          if (avd_limit_right == 0.2) {
            lat_offset = std::min(lat_offset, avd_limit_right);
          }

          if (flane_->status() != LaneStatusEx::BOTH_MISSING &&
              l_poly_[3] > 0 && l_poly_[3] < 1.0 && !special_lane_type) {
            lat_offset += l_poly_[3] - 1.0;
          }
        } else if (((int)avd_car_past[0][0] == 0 &&
                    avd_car_past[0][3] > -6.0) ||
                   ((int)avd_car_past[0][0] != 0 &&
                    avd_car_past[0][3] >= -3.0)) {
          if (status != INTER_GS_NONE && status != INTER_TR_NONE &&
              status != INTER_TL_NONE) {
            if (avd_car_past[0][6] != 100) {
              lat_offset = 0.5 * (lane_width - car_width / 2 -
                                  std::fabs(avd_car_past[0][6])) +
                           lat_compen1;
            }

            lat_offset = std::max(lat_offset, 0.0);

            if (avd_normal_thr > 0) {
              lat_offset =
                  std::min(std::min(lat_offset, avd_normal_thr),
                           std::min(avd_car_past[0][9], avd_limit_right));
            } else {
              lat_offset =
                  std::min(std::min(lat_offset, 0.5 * lane_width - 0.9),
                           std::min(avd_car_past[0][9], avd_limit_right));
            }

            if (avd_car_past[0][6] <
                    -0.9 + std::max(lane_width / 2 - 1.8, 0.0) &&
                avd_car_past[0][6] > -1.5) {
              lat_offset = avd_car_past[0][6] -
                           (avd_car_past[0][6] - 1.8 - avd_car_past[0][9]) / 2;
              lat_offset = std::min(lat_offset, 0.15 * lane_width);

              if (virtual_lane_manager_->get_lane_num() > 1 &&
                  virtual_lane_manager_->current_lane_virtual_id() == 0) {
                lat_offset *= 0.8;
              }
            } else if (avd_car_past[0][6] >= 0 &&
                       (avd_car_past[0][2] + v_ego < 1.5 ||
                        avd_car_past[0][3] < 1) &&
                       std::fabs(avd_car_past[0][4]) < 0.5 &&
                       ((virtual_lane_manager_->current_lane_virtual_id() ==
                         virtual_lane_manager_->get_lane_num() - 1) ||
                        (virtual_lane_manager_->current_lane_virtual_id() ==
                             virtual_lane_manager_->get_lane_num() - 2 &&
                         virtual_lane_manager_->get_right_lane() != nullptr &&
                         virtual_lane_manager_->get_right_lane()
                                 ->get_lane_type() ==
                             MSD_LANE_TYPE_NON_MOTOR))) {
              if (((!frame_->session()->environmental_model().is_on_highway()
                    //  &&map_info.dist_to_intsect() > 80 &&
                    // map_info.dist_to_intsect() - avd_car_past[0][3] > 80
                    ) ||
                   (d_poly_[3] > 1 && avd_car_past[0][3] < 3)) &&
                  ((avd_car_past[0][3] < 15 && v_ego < 5) ||
                   (v_ego < 10 && avd_car_past[0][2] + v_ego < -1) ||
                   avd_car_past[0][3] < 1)) {
                lat_offset =
                    avd_car_past[0][5] -
                    (avd_car_past[0][5] + 1.8 + avd_car_past[0][9]) / 2;

                if (-lat_offset - avd_car_past[0][9] > 0.4) {
                  lat_offset = std::max(
                      std::max(lat_offset, -0.15 * lane_width - dist_rblane),
                      std::max(-avd_car_past[0][9], -2.0 + avd_car_past[0][5]));
                } else {
                  lat_offset = std::max(
                      lat_offset, std::max(-0.15 * lane_width - dist_rblane,
                                           -2.0 + avd_car_past[0][5]));
                }

                if (lat_offset == -avd_car_past[0][9]) {
                  lat_offset = 0;
                }

                if (dist_rblane > 1.0) {
                  sb_blane_ = true;
                }
                if (std::pow(avd_car_past[0][2] - 1, 2) / 4 >
                        avd_car_past[0][3] - 2 &&
                    avd_car_past[0][3] > -5 && d_poly_[3] < -0.7) {
                  lat_offset = -1.3 + avd_car_past[0][5];
                  force_pause_ = true;
                }
              } else {
                lat_offset = 0;
              }
            } else if (avd_car_past[0][6] >=
                           -0.9 + std::max(lane_width / 2 - 1.8, 0.0) &&
                       avd_car_past[0][6] < 0 &&
                       (avd_car_past[0][2] + v_ego < 0.5 ||
                        avd_car_past[0][3] < 1) &&
                       std::fabs(avd_car_past[0][4]) < 0.5 &&
                       ((virtual_lane_manager_->current_lane_virtual_id() ==
                             virtual_lane_manager_->get_lane_num() - 1 &&
                         ((virtual_lane_manager_->get_current_lane()
                                   ->get_lane_type() !=
                               MSD_LANE_TYPE_NON_MOTOR &&
                           virtual_lane_manager_->current_lane_virtual_id() >=
                               1) ||
                          (virtual_lane_manager_->get_current_lane()
                                   ->get_lane_type() ==
                               MSD_LANE_TYPE_NON_MOTOR &&
                           virtual_lane_manager_->current_lane_virtual_id() >=
                               2))) ||
                        (virtual_lane_manager_->current_lane_virtual_id() ==
                             virtual_lane_manager_->get_lane_num() - 2 &&
                         virtual_lane_manager_->get_right_lane() != nullptr &&
                         virtual_lane_manager_->get_right_lane()
                                 ->get_lane_type() == MSD_LANE_TYPE_NON_MOTOR &&
                         virtual_lane_manager_->current_lane_virtual_id() >=
                             1))) {
              lat_offset =
                  std::min(1.5 + avd_car_past[0][6], avd_car_past[0][9]);

              if (lat_offset >= lane_width / 2 - 1.1) {
                large_lat_ = true;
              } else if (std::pow(avd_car_past[0][2] - 1, 2) / 4 >
                             avd_car_past[0][3] - 2 &&
                         avd_car_past[0][3] > -10 && d_poly_[3] < -0.5) {
                lat_offset = 1.3 + avd_car_past[0][6];
                force_pause_ = true;
              }
            }

            if (avd_limit_right == 0.2) {
              lat_offset = std::min(lat_offset, avd_limit_right);
            }

            if (flane_->status() != LaneStatusEx::BOTH_MISSING &&
                l_poly_[3] > 0 && l_poly_[3] < 1.0 &&
                virtual_lane_manager_->current_lane_virtual_id() == 0 &&
                !special_lane_type && !large_lat_ && !force_pause_) {
              lat_offset = std::max(lat_offset + l_poly_[3] - 1.0, 0.0);
            }
          } else {
            if (avd_car_past[0][6] != 100) {
              lat_offset = 0.5 * (lane_width - car_width / 2 -
                                  std::fabs(avd_car_past[0][6])) +
                           lat_compen1;
            }

            lat_offset = std::max(lat_offset, 0.0);
            lat_offset = std::min(lat_offset, std::min(0.5 * lane_width - 0.9,
                                                       avd_car_past[0][9]));

            if (avd_car_past[0][6] > -1.5) {
              lat_offset = avd_car_past[0][6] -
                           (avd_car_past[0][6] - 1.8 - avd_car_past[0][9]) / 2;

              if (avd_car_past[0][6] < -1.1) {
                // if (map_info.left_refline_points().size() != 0 || // hack
                //     map_info.dist_to_last_intsect() > 20) {
                //  lat_offset = std::min(lat_offset, 0.5 * lane_width - 0.9);
                //} else {
                if (avd_normal_thr > 0) {
                  lat_offset = std::min(lat_offset, avd_normal_thr);
                } else {
                  lat_offset = std::min(lat_offset, 0.5 * lane_width - 0.9);
                }
                //}
              } else if (lane_width <= 3.8) {
                if (avd_car_past[0][6] > 0 && avd_car_past[0][5] > -0.65 &&
                    (avd_car_past[0][2] + v_ego < 1.5 ||
                     avd_car_past[0][3] < 0) &&
                    avd_car_past[0][4] > -0.5 && avd_car_past[0][4] < 0.3 &&
                    avd_car_past[0][1] == 0) {
                  lat_offset =
                      avd_car_past[0][5] -
                      (avd_car_past[0][5] + 1.8 + avd_car_past[0][9]) / 2;

                  lat_offset = std::max(lat_offset, -1.6 + avd_car_past[0][5]);
                  sb_lane_ = true;
                } else {
                  //    if (map_info.dist_to_last_intsect() > -10) {
                  lat_offset = std::min(lat_offset,
                                        std::min(0.5 * lane_width - 0.9, 0.7));
                  // } else {
                  //   if (avd_normal_thr > 0) {
                  //     lat_offset = std::min(lat_offset, avd_normal_thr);
                  //   } else {
                  //     lat_offset =
                  //         std::min(lat_offset, 0.5 * lane_width - 0.9);
                  //   }
                  // }
                }
              } else {
                lat_offset =
                    std::min(lat_offset, 0.24 * lane_width / 4.4 * lane_width);

                if (avd_car_past[0][6] > 0 && avd_car_past[0][5] > -0.65 &&
                    (avd_car_past[0][2] + v_ego < 1.5 ||
                     avd_car_past[0][3] < 0) &&
                    avd_car_past[0][4] > -0.5 && avd_car_past[0][4] < 0.3 &&
                    avd_car_past[0][1] == 0) {
                  lat_offset =
                      avd_car_past[0][5] -
                      (avd_car_past[0][5] + 1.8 + avd_car_past[0][9]) / 2;
                  lat_offset = std::max(lat_offset, -1.8 + avd_car_past[0][5]);
                  sb_lane_ = true;
                }
              }
            }
          }

          if ((virtual_lane_manager_->current_lane_virtual_id() !=
                   virtual_lane_manager_->get_lane_num() - 1 &&
               virtual_lane_manager_->current_lane_virtual_id() != 0) &&
              avd_car_past[0][2] + v_ego < 0.5 &&
              // map_info.dist_to_intsect() > 0 &&
              // map_info.dist_to_intsect() - avd_car_past[0][3] < 50 &&
              abs(avd_car_past[0][4]) < 0.2 &&
              (avd_car_past[0][6] >= (lane_width / 2 - (car_width + 0.3))) &&
              avd_car_past[0][6] < 0) {
            cross_left_solid_line_ = true;
            lat_offset = 0.;
          }
          if (avd_car_past[0][7] == 20001 &&
              // map_info.dist_to_intsect() - avd_car_past[0][3] >= -5 &&
              //  map_info.dist_to_intsect() - avd_car_past[0][3] < 50 &&
              (avd_car_past[0][6] >= (lane_width / 2 - (car_width + 0.3)))) {
            if (virtual_lane_manager_->current_lane_virtual_id() != 0 ||
                flane_->get_relative_id() == RIGHT_POS) {
              cross_left_solid_line_ = true;
            } else if (virtual_lane_manager_->current_lane_virtual_id() !=
                       virtual_lane_manager_->get_lane_num() - 1) {
              cross_right_solid_line_ = true;
            }
            lat_offset = 0.;
          }

          lat_offsetr = lat_offset;

          pre_str_dist = interp(avd_car_past[0][2], avd_vRel_bp, avd_vRel_v);
          if (avd_car_past[0][0] > 0 && avd_car_past[0][7] != 0.) {
            if (avd_car_past[0][7] >= avd_car_past[0][3]) {
              lat_offset = std::min(
                  (avd_car_past[0][7] - (avd_car_past[0][3] - pre_str_dist)) /
                      avd_car_past[0][7] * lat_offset,
                  lat_offset);
            } else if (avd_car_past[0][3] != 0.) {
              lat_offset =
                  (1 - avd_car_past[0][7] / avd_car_past[0][3]) * lat_offset;
            }
          } else {
            if (avd_car_past[0][5] < 1.5) {
              lat_offset =
                  std::max((5. / std::max(std::abs(avd_car_past[0][3]), 5.)) *
                               lat_offset,
                           lat_offset);
            } else {
              lat_offset =
                  std::max((5. / std::max(std::abs(avd_car_past[0][3]), 5.)) *
                               lat_offset,
                           lat_offset);
            }
          }
          if (d_poly_[3] + lat_offset > 0.3) {
            lat_offset = -d_poly_[3] + 0.3;
          }
        } else {
          lat_offset = std::max(
              std::min(1.6 + avd_car_past[0][6],
                       avd_car_past[0][6] -
                           (avd_car_past[0][6] - 1.8 - avd_car_past[0][9]) / 2),
              0.0);

          if (std::fabs(lat_offset) > 0.5 * lane_width - 0.9 &&
              avd_car_past[0][6] < 0) {
            lat_offset = 0.5 * lane_width - 0.9;
          }
        }

        if (dist_rblane == 0 && (lat_offset > 0.8 || lat_offset < -0.8)) {
        } else {
          if (std::fabs(lat_offsetr) >= std::fabs(d_poly_[3]) &&
              d_poly_[3] < 0 && d_poly_[3] + lat_offset < 0) {
            d_poly_[3] = 0;
          } else if (std::pow(avd_car_past[0][2] - 1, 2) / 4 >
                         avd_car_past[0][3] - 2 &&
                     avd_car_past[0][3] > -5) {
            if (avd_car_past[0][5] < 0 && avd_car_past[0][6] < 0 &&
                std::fabs(lat_offset - avd_car_past[0][6]) < 1.3) {
              lat_offset = 1.3 + avd_car_past[0][6];

              if (virtual_lane_manager_->current_lane_virtual_id() == 0) {
                lat_offset =
                    std::min(1.3 + avd_car_past[0][6], 0.5 * lane_width - 0.9);
              }

              force_pause_ = true;
            } else {
              d_poly_[3] += lat_offset;
              two_nudge_car = d_poly_[3];
            }
          } else {
            d_poly_[3] += lat_offset;
          }

          one_nudge_right_car = d_poly_[3];
        }
      }
    } else if (flag_avd == 1 && status != ROAD_LB_LBACK &&
               status != ROAD_LB_RBACK && status != ROAD_LB_LRETURN &&
               status != ROAD_LB_RRETURN && status != ROAD_LC_LCHANGE &&
               status != ROAD_LC_RCHANGE) {
      double path_gap = 0.2;
      std::array<double, 2> xp1{0, 0.02};
      std::array<double, 2> xp2{0, 0.003};
      std::array<double, 2> fp1{1, 5};
      std::array<double, 2> fp2{1, 3};

      double min_factor = std::max(interp(std::fabs(d_poly_[2]), xp1, fp1),
                                   interp(std::fabs(d_poly_[1]), xp2, fp1));
      double max_factor = std::max(interp(std::fabs(d_poly_[2]), xp1, fp2),
                                   interp(std::fabs(d_poly_[1]), xp2, fp2));

      if (d_poly_[3] < -min_factor * path_gap) {
        d_poly_[3] = -min_factor * path_gap;
      } else if (d_poly_[3] > max_factor * path_gap) {
        d_poly_[3] = max_factor * path_gap;
      }
    }

    lane_borrow_suspend_cnt_ = 0;
    suspend_lat_offset_ = 0;

  } else if (status == ROAD_LB_LBORROW || status == ROAD_LB_RBORROW) {
    if (avd_sp_car_past[0].size() > 0) {
      double plus1 =
          interp(avd_sp_car_past[0][2], near_car_vrel_bp, near_car_vrel_v);
      double plus1_rel =
          interp(avd_sp_car_past[0][3], near_car_drel_bp, near_car_drel_v);
      double lat_compen1 = 0.5 * plus1 + 0.5 * plus1_rel;

      if (avd_sp_car_past[1].size() > 0) {
        double v_near_car2 = v_ego + avd_sp_car_past[1][2];
        double desired_dist2 = dist_offset + v_near_car2 * t_gap;
        double diff_dist_nudge_car =
            avd_sp_car_past[1][3] - avd_sp_car_past[0][3];
        double dist_avd_nudge_car1 = avd_sp_car_past[0][3] + 5.0 + safety_dist;

        double t_avdcar1 = 0;
        if (avd_sp_car_past[0][2] != 0) {
          if (avd_sp_car_past[0][2] < 0) {
            t_avdcar1 = -dist_avd_nudge_car1 / avd_sp_car_past[0][2];
          } else {
            t_avdcar1 = 5;
          }
        } else {
          t_avdcar1 = 0;
        }

        if (t_avdcar1 > 0) {
          diff_dist_nudge_car += diff_dist_nudge_car * t_avdcar1;
          if ((int)avd_sp_car_past[0][9] == RIGHT_CHANGE &&
              (int)avd_sp_car_past[1][9] == LEFT_CHANGE) {
            if (diff_dist_nudge_car <
                desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego) {
              if (avd_sp_car_past[0][2] <= avd_sp_car_past[1][2] ||
                  avd_sp_car_past[0][2] - avd_sp_car_past[1][2] < 2) {
                if (avd_sp_car_past[0][5] != 100) {
                  lat_offset = 0.9 * (lane_width - car_width / 2 -
                                      std::fabs(avd_sp_car_past[0][5])) +
                               lat_compen1;
                }

                lat_offset = -std::max(lat_offset, 0.0);
                if (d_poly_[3] < 0) {
                  lat_offset += lane_width;
                  d_poly_[3] += lat_offset;
                }
              } else {
                if (avd_sp_car_past[1][6] != 100) {
                  lat_offset = 0.9 * (lane_width - car_width / 2 -
                                      std::fabs(avd_sp_car_past[1][6])) +
                               lat_compen1;
                }

                lat_offset = std::max(lat_offset, 0.0);
                if (d_poly_[3] > 0) {
                  lat_offset = -(lane_width - lat_offset);
                  d_poly_[3] += lat_offset;
                }
              }
            } else {
              lat_offset = 0.9 * (lane_width - car_width / 2 -
                                  std::fabs(avd_sp_car_past[0][5])) +
                           lat_compen1;

              lat_offset = -std::max(lat_offset, 0.0);
              if (lane_type == MSD_LANE_TYPE_PARKING ||
                  std::fabs(d_poly_[1]) > 0.0001) {
                lat_offset *= 0.7;
              }
            }
          } else if ((int)avd_sp_car_past[0][9] == LEFT_CHANGE &&
                     (int)avd_sp_car_past[1][9] == RIGHT_CHANGE) {
            if (diff_dist_nudge_car <
                desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego) {
              if (avd_sp_car_past[0][2] <= avd_sp_car_past[1][2] ||
                  avd_sp_car_past[0][2] - avd_sp_car_past[1][2] < 2) {
                if (avd_sp_car_past[0][6] != 100) {
                  lat_offset = 0.9 * (lane_width - car_width / 2 -
                                      std::fabs(avd_sp_car_past[0][6])) +
                               lat_compen1;
                }

                lat_offset = std::max(lat_offset, 0.0);

                if (d_poly_[3] > 0) {
                  lat_offset = -(lane_width - lat_offset);
                  d_poly_[3] += lat_offset;
                }
              } else {
                if (avd_sp_car_past[1][5] != 100) {
                  lat_offset = 0.9 * (lane_width - car_width -
                                      std::fabs(avd_sp_car_past[1][5])) +
                               lat_compen1;
                }

                lat_offset = -std::max(lat_offset, 0.0);

                if (d_poly_[3] < 0) {
                  lat_offset += lane_width;
                  d_poly_[3] += lat_offset;
                }
              }
            } else {
              lat_offset = 0.9 * (lane_width - car_width / 2 -
                                  std::fabs(avd_sp_car_past[0][6])) +
                           lat_compen1;

              lat_offset = std::max(lat_offset, 0.0);

              if (lane_type == MSD_LANE_TYPE_PARKING ||
                  std::fabs(d_poly_[1]) > 0.0001) {
                lat_offset *= 0.7;
              }

              if (d_poly_[3] > 0) {
                lat_offset = -(lane_width - lat_offset);
                d_poly_[3] += lane_width - lat_offset;
              }
            }
          } else if ((int)avd_sp_car_past[0][9] == RIGHT_CHANGE &&
                     (int)avd_sp_car_past[1][9] == RIGHT_CHANGE) {
            if (diff_dist_nudge_car <
                desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego) {
              if (avd_sp_car_past[0][5] != 100 ||
                  avd_sp_car_past[1][5] != 100) {
                if (std::fabs(avd_sp_car_past[0][5]) < 1.9 ||
                    std::fabs(avd_sp_car_past[1][5]) < 1.9) {
                  lat_offset = 0.9 * (lane_width - car_width / 2 -
                                      std::min(avd_sp_car_past[0][5],
                                               avd_sp_car_past[1][5])) +
                               lat_compen1;
                } else {
                  if (avd_sp_car_past[0][5] >= 1.9 ||
                      avd_sp_car_past[1][5] >= 1.9) {
                    lat_offset = 0.9 * (lane_width - car_width / 2 -
                                        std::min(avd_sp_car_past[0][5],
                                                 avd_sp_car_past[1][5]) +
                                        lane_width) +
                                 lat_compen1;
                  }
                }
              }
            } else {
              if (avd_sp_car_past[0][5] != 100) {
                if (std::fabs(avd_sp_car_past[0][5]) < 1.9) {
                  lat_offset = 0.9 * (lane_width - car_width / 2 -
                                      avd_sp_car_past[0][5]) +
                               lat_compen1;
                } else {
                  if (avd_sp_car_past[0][5] >= 1.9) {
                    lat_offset = 0.9 * (lane_width - car_width / 2 -
                                        avd_sp_car_past[0][5] + lane_width) +
                                 lat_compen1;
                  }
                }
              }
            }

            lat_offset = -std::max(lat_offset, 0.0);
            lat_offset = std::max(-1.2 * lane_width, lat_offset);
            d_poly_[3] += lat_offset;
          } else if ((int)avd_sp_car_past[0][9] == LEFT_CHANGE &&
                     (int)avd_sp_car_past[1][9] == LEFT_CHANGE) {
            if (diff_dist_nudge_car <
                desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego) {
              if (avd_sp_car_past[0][6] != 100 &&
                  avd_sp_car_past[1][6] != 100) {
                if (std::fabs(avd_sp_car_past[0][6]) < 1.9 ||
                    std::fabs(avd_sp_car_past[1][6]) < 1.9) {
                  lat_offset = 0.9 * (lane_width - car_width / 2 +
                                      std::max(avd_sp_car_past[0][6],
                                               avd_sp_car_past[1][6]) +
                                      lane_width) +
                               lat_compen1;
                } else {
                  if (avd_sp_car_past[0][6] <= -1.9 ||
                      avd_sp_car_past[1][6] <= -1.9) {
                    lat_offset = 0.9 * (lane_width - car_width / 2 +
                                        std::max(avd_sp_car_past[0][6],
                                                 avd_sp_car_past[1][6]) +
                                        lane_width) +
                                 lat_compen1;
                  }
                }
              } else if (avd_sp_car_past[0][6] != 100) {
                if (std::fabs(avd_sp_car_past[0][6]) < 1.9) {
                  lat_offset = 0.9 * (lane_width - car_width / 2 +
                                      avd_sp_car_past[0][6]) +
                               lat_compen1;
                } else {
                  if (avd_sp_car_past[0][6] <= -1.9) {
                    lat_offset = 0.9 * (lane_width - car_width / 2 +
                                        avd_sp_car_past[0][6] + lane_width) +
                                 lat_compen1;
                  }
                }
              } else if (avd_sp_car_past[1][6] != 100) {
                if (std::fabs(avd_sp_car_past[1][6]) < 1.9) {
                  lat_offset = 0.9 * (lane_width - car_width / 2 +
                                      avd_sp_car_past[1][6]) +
                               lat_compen1;
                } else {
                  if (avd_sp_car_past[1][6] <= -1.9) {
                    lat_offset = 0.9 * (lane_width - car_width / 2 +
                                        avd_sp_car_past[1][6] + lane_width) +
                                 lat_compen1;
                  }
                }
              }
            } else {
              if (avd_sp_car_past[0][6] != 100) {
                if (std::fabs(avd_sp_car_past[0][6]) < 1.9) {
                  lat_offset = 0.9 * (lane_width - car_width / 2 +
                                      avd_sp_car_past[0][6]) +
                               lat_compen1;
                } else {
                  if (avd_sp_car_past[0][6] <= -1.9) {
                    lat_offset = 0.9 * (lane_width - car_width / 2 +
                                        avd_sp_car_past[0][6] + lane_width) +
                                 lat_compen1;
                  }
                }
              }
            }

            lat_offset = std::max(lat_offset, 0.0);
            lat_offset = std::min(lat_offset, 1.2 * lane_width);
            d_poly_[3] += lat_offset;
          }
        }
      } else if ((int)avd_sp_car_past[0][9] == RIGHT_CHANGE) {
        if (avd_sp_car_past[0][5] != 100) {
          if (std::fabs(avd_sp_car_past[0][5]) < 1.9) {
            lat_offset =
                0.9 * (lane_width - car_width / 2 - avd_sp_car_past[0][5]) +
                lat_compen1;
          } else {
            if (avd_sp_car_past[0][5] >= 1.9) {
              lat_offset =
                  0.9 * (lane_width - car_width / 2 - avd_sp_car_past[0][5]) +
                  lat_compen1;
            }
          }

          lat_offset = -std::max(lat_offset, 0.0);
          lat_offset = std::max(-1.2 * lane_width, lat_offset);
        } else {
          lat_offset = 0;
        }

        d_poly_[3] += lat_offset;
      } else {
        if (avd_sp_car_past[0][6] != 100) {
          if (std::fabs(avd_sp_car_past[0][6]) < 1.9) {
            lat_offset =
                0.9 * (lane_width - car_width / 2 + avd_sp_car_past[0][6]) +
                lat_compen1;
          } else {
            if (avd_sp_car_past[0][6] <= -1.9) {
              lat_offset =
                  0.9 * (lane_width - car_width / 2 + avd_sp_car_past[0][6]) +
                  lat_compen1;
            }
          }

          lat_offset = std::max(lat_offset, 0.0);
          lat_offset = std::min(lat_offset, 1.2 * lane_width);
        } else {
          lat_offset = 0;
        }

        d_poly_[3] += lat_offset;
      }
    }

    lane_borrow_suspend_cnt_ = 0;
    suspend_lat_offset_ = 0;

  } else if (status == ROAD_LB_LSUSPEND || status == ROAD_LB_RSUSPEND) {
    if (lane_borrow_suspend_cnt_ == 0) {
      suspend_lat_offset_ = d_poly_[3];
      lat_offset = -suspend_lat_offset_;
      lane_borrow_suspend_cnt_ += 1;
    }

    lat_offset = -suspend_lat_offset_;
    d_poly_[3] -= suspend_lat_offset_;
  }

  two_nudge_car_ = two_nudge_car;  // some problem
                                   // need to be solved
  one_nudge_left_car_ = one_nudge_left_car;
  one_nudge_right_car_ = one_nudge_right_car;
  lane_width_ = lane_width;

  if (premoving_) {
    lat_offset_ += lat_offset;
  } else {
    lat_offset_ = lat_offset;
  }
}

bool VisionLateralMotionPlanner::update_planner_output() {
  //   auto &map_info = world_model_->get_map_info();
  //   auto &map_info_mgr =
  // world_model_->get_map_info_manager();
  auto &lateral_output = frame_->mutable_session()
                             ->mutable_planning_context()
                             ->mutable_lateral_behavior_planner_output();
  // lateral_output = context_->mutable_lateral_behavior_planner_output();
  auto &state_machine_output =
      frame_->mutable_session()
          ->mutable_planning_context()
          ->mutable_lat_behavior_state_machine_output();

  //   //   auto &flane = virtual_lane_mgr_->get_fix_lane();
  //   //
  //   //   auto &f_refline =
  //   //       virtual_lane_mgr_
  //   //           ->get_fix_refline();  // fix lane 和fix
  //   refline 的区别是？
  //   //   auto &lateral_obstacle =
  //   world_model_->mutable_lateral_obstacle();

  int scenario = state_machine_output.scenario;
  int state = state_machine_output.curr_state;
  auto &state_name = state_machine_output.state_name;
  int turn_light = state_machine_output.turn_light;
  int map_turn_light = state_machine_output.map_turn_light;  //
  //   取值范围，对应的含义？
  bool accident_ahead = state_machine_output.accident_ahead;

  bool accident_back = state_machine_output.accident_back;

  bool close_to_accident = state_machine_output.close_to_accident;

  bool lc_pause = state_machine_output.lc_pause;
  int lc_pause_id = state_machine_output.lc_pause_id;  // id
  //   //                          //
  //   //
  //   是指车辆id还是路线id，以及id命名顺序是从左到右还是从右到左？
  double tr_pause_l = state_machine_output.tr_pause_l;  //
  //   为啥是double 类型？
  //   //       表示的含义是？
  double tr_pause_s = state_machine_output.tr_pause_s;

  bool isRedLightStop = false;  // hack!

  update_planner_status();

  // lateral_output.timestamp = IflyTime::Now_ms();
  lateral_output.enable = true;
  lateral_output.track_id = -20000;  // hack!
  lateral_output.v_limit = 40.0 / 3.6;
  lateral_output.isRedLightStop = isRedLightStop;

  lateral_output.disable_l = state_machine_output.disable_l;
  lateral_output.disable_r = state_machine_output.disable_r;
  lateral_output.enable_l = state_machine_output.enable_l;  // disabled
  //       ,enable的区别是啥？代表啥意思
  lateral_output.enable_r = state_machine_output.enable_r;
  lateral_output.enable_id = state_machine_output.enable_id;  //
  //   enable车的id？ 车道线的id?

  lateral_output.lat_offset = lat_offset_;

  lateral_output.lane_borrow = false;        // attention!! hack!
  lateral_output.lane_borrow_range = -1000;  // attention!! hack!
  TrackedObject *lead_one = frame_->mutable_session()
                                ->mutable_environmental_model()
                                ->get_lateral_obstacle()
                                ->leadone();

  if (flane_->get_relative_id() == LEFT_POS ||  // 判断fix refline
                                                //   在自车的方位？ 左或者右？
      flane_->get_relative_id() == LEFT_LEFT_POS) {
    lateral_output.which_lane = "left_line";  // 定义成int
    //   type类型，减少string的使用
  } else if (flane_->get_relative_id() == RIGHT_POS) {
    lateral_output.which_lane = "right_line";
  } else {
    lateral_output.which_lane = "current_line";
  }

  int lb_request = state_machine_output.lb_request;

  if (lb_request == NO_CHANGE) {
    lateral_output.lb_request = "none";
  } else if (lb_request == LEFT_CHANGE) {
    lateral_output.lb_request = "left";
  } else {
    lateral_output.lb_request = "right";
  }

  lateral_output.lb_width = -10.;  //  attention!! hack!
  int lc_request = state_machine_output.lc_request;

  if (lc_request == NO_CHANGE) {
    lateral_output.lc_request = "none";
  } else if (lc_request == LEFT_CHANGE) {
    lateral_output.lc_request = "left";
  } else {
    lateral_output.lc_request = "right";
  }

  if (state == ROAD_LC_LCHANGE || state == INTER_GS_LC_LCHANGE ||
      state == INTER_TR_LC_LCHANGE || state == INTER_TL_LC_LCHANGE) {
    lateral_output.lc_status = "left_lane_change";
  } else if (state == ROAD_LC_LBACK || state == INTER_GS_LC_LBACK ||
             state == INTER_TR_LC_LBACK || state == INTER_TL_LC_LBACK) {
    lateral_output.lc_status = "left_lane_change_back";
  } else if (state == ROAD_LC_RCHANGE || state == INTER_GS_LC_RCHANGE ||
             state == INTER_TR_LC_RCHANGE || state == INTER_TL_LC_RCHANGE) {
    lateral_output.lc_status = "right_lane_change";
  } else if (state == ROAD_LC_RBACK || state == INTER_GS_LC_RBACK ||
             state == INTER_TR_LC_RBACK || state == INTER_TL_LC_RBACK) {
    lateral_output.lc_status = "right_lane_change_back";
  } else if (state == ROAD_LC_LWAIT || state == INTER_GS_LC_LWAIT ||
             state == INTER_TR_LC_LWAIT || state == INTER_TL_LC_LWAIT) {
    lateral_output.lc_status = "left_lane_change_wait";
  } else if (state == ROAD_LC_RWAIT || state == INTER_GS_LC_RWAIT ||
             state == INTER_TR_LC_RWAIT || state == INTER_TL_LC_RWAIT) {
    lateral_output.lc_status = "right_lane_change_wait";
  } else {
    lateral_output.lc_status = "none";
  }

  if (state == ROAD_LB_LBORROW) {
    lateral_output.lb_status = "left_lane_borrow";
  } else if (state == ROAD_LB_RBORROW) {
    lateral_output.lb_status = "right_lane_borrow";
  } else if (state == ROAD_LB_LSUSPEND) {
    lateral_output.lb_status = "left_lane_suspend";
  } else if (state == ROAD_LB_RSUSPEND) {
    lateral_output.lb_status = "right_lane_suspend";
  } else {
    lateral_output.lb_status = "none";
  }

  lateral_output.avd_info.clear();

  lateral_output.scenario = scenario;
  lateral_output.flane_width = flane_->width();

  lateral_output.dist_rblane = 10.;  // attention!!hack!

  //todo:clren 后面会使用ReferencePoints代替PathPoint
  std::vector<PathPoint> path_points; 
  if (flane_ != nullptr) {
    auto &ref_path = flane_->get_reference_path();
    for (auto &ref_point : ref_path->get_points()) {
      path_points.emplace_back(ref_point.path_point);
    }  
  }
  lateral_output.path_points = path_points;
  // lateral_output.path_points =
  //     flane_->get_refined_lane_points();  // attention! transform
  //                                         // RerencePathPoint 2 PathPoint

  if (((virtual_lane_manager_->current_lane_virtual_id() ==
        virtual_lane_manager_->get_lane_num() - 1) ||
       (virtual_lane_manager_->current_lane_virtual_id() ==
            virtual_lane_manager_->get_lane_num() - 2 &&
        virtual_lane_manager_->get_right_lane() != nullptr &&
        virtual_lane_manager_->get_right_lane()->get_lane_type() ==
            MSD_LANE_TYPE_NON_MOTOR)) &&
      ((!isRedLightStop && lateral_output.accident_ahead &&
        lead_one != nullptr && lead_one->type == 20001))) {
    lateral_output.borrow_bicycle_lane = true;
  } else {
    lateral_output.borrow_bicycle_lane = false;
  }

  lateral_output.enable_intersection_planner = false;

  bool left_direct_exist = true;

  bool right_direct_exist = true;

  bool curr_direct_has_straight = true;

  bool curr_direct_has_right = false;

  bool left_direct_has_straight = true;

  bool is_right_turn = false;

  if (virtual_lane_manager_->get_left_lane() == nullptr ||
      left_direct_exist == false) {
    lateral_output.tleft_lane = true;
  } else {
    lateral_output.tleft_lane = false;
  }

  if (((virtual_lane_manager_->current_lane_virtual_id() ==
        virtual_lane_manager_->get_lane_num() - 1) ||
       (virtual_lane_manager_->current_lane_virtual_id() ==
            virtual_lane_manager_->get_lane_num() - 2 &&
        virtual_lane_manager_->get_right_lane() != nullptr &&
        virtual_lane_manager_->get_right_lane()->get_lane_type() ==
            MSD_LANE_TYPE_NON_MOTOR)) &&
      virtual_lane_manager_->current_lane_virtual_id() - 1 >= 0) {
    lateral_output.rightest_lane = true;
  } else {
    lateral_output.rightest_lane = false;
  }

  lateral_output.dist_intersect = 1000;  // attention !

  if (virtual_lane_manager_->get_intersection_info().intsect_length() !=
      DBL_MAX) {  // attention !

    lateral_output.intersect_length =
        virtual_lane_manager_->get_intersection_info().intsect_length();
  } else {
    lateral_output.intersect_length = 1000;
  }

  if (virtual_lane_manager_->lc_map_decision_offset(
          virtual_lane_manager_->get_current_lane()) !=
      DBL_MAX) {  // attention !
    lateral_output.lc_end_dis = virtual_lane_manager_->lc_map_decision_offset(
        virtual_lane_manager_->get_current_lane());
  } else {
    lateral_output.lc_end_dis = 10000;
  }

  if (virtual_lane_manager_->get_ramp().dis_to_ramp() != DBL_MAX) {  // attention !
    lateral_output.dis_to_ramp = virtual_lane_manager_->get_ramp().dis_to_ramp();
  } else {
    lateral_output.dis_to_ramp = 10000;
  }

  lateral_output.sb_lane = sb_lane_;
  lateral_output.sb_blane = sb_blane_;
  lateral_output.force_pause = force_pause_;
  lateral_output.large_lat = large_lat_;
  lateral_output.premoving = premoving_;
  lateral_output.accident_ahead = accident_ahead;
  lateral_output.accident_back = accident_back;
  lateral_output.lc_pause_id = lc_pause_id;
  lateral_output.lc_pause = lc_pause;
  lateral_output.tr_pause_l = tr_pause_l;
  lateral_output.tr_pause_s = tr_pause_s;
  lateral_output.must_change_lane = state_machine_output.must_change_lane;
  lateral_output.close_to_accident = close_to_accident;
  lateral_output.angle_steers_limit = 0.0;  // attention!
  lateral_output.left_faster = state_machine_output.left_is_faster;

  lateral_output.right_faster = state_machine_output.right_is_faster;
  lateral_output.premove =
      (state_machine_output.premovel || state_machine_output.premover);
  lateral_output.premove_dist = state_machine_output.premove_dist;
  lateral_output.isFasterStaticAvd =
      (left_direct_exist && lateral_output.left_faster) ||
      (right_direct_exist && lateral_output.right_faster) ||
      (curr_direct_has_right && !curr_direct_has_straight) ||
      (is_right_turn && left_direct_has_straight &&
       lateral_output.left_faster);  // attention!
  lateral_output.isOnHighway =
      frame_->session()->environmental_model().is_on_highway();

  lateral_output.c_poly.assign(c_poly_.begin(), c_poly_.end());

  //   //
  lateral_output.d_poly.assign(d_poly_.begin(), d_poly_.end());

  lateral_output.s_v_limit.clear();  // attention!
  lateral_output.s_a_limit.clear();
  lateral_output.s_r_offset.clear();

  if (lead_one != nullptr) {                         // attention! 同上
    lateral_output.lead_one_drel = lead_one->d_rel;  // lead_one距离
    lateral_output.lead_one_vrel = lead_one->v_rel;  // lead_one 速度
  } else {
    lateral_output.lead_one_drel = 0.0;
    lateral_output.lead_one_vrel = 0.0;
  }

  lateral_output.state_name = state_name;

  lateral_output.scenario_name =
      (scenario == LOCATION_ROAD) ? "Road" : "Intersect";

  if (turn_light == 1) {
    lateral_output.turn_light = "Left";  //
    //   这里也可以只做一次赋值；
  } else if (turn_light == 2) {
    lateral_output.turn_light = "Right";
  } else {
    lateral_output.turn_light = "None";
  }

  auto request_source = state_machine_output.lc_request_source;
  lateral_output.act_request_source = "none";
  if (request_source == INT_REQUEST) {
    lateral_output.lc_request_source = "int_request";
    //   // int 是嘛意思
  } else if (request_source == MAP_REQUEST) {
    lateral_output.lc_request_source = "map_request";
  } else if (request_source == ACT_REQUEST) {
    lateral_output.lc_request_source = "act_request";
    lateral_output.act_request_source = state_machine_output.act_request_source;
  } else {
    lateral_output.lc_request_source = "none";
  }

  if (map_turn_light > 0) {
    lateral_output.turn_light_source = "map_turn_light";
  } else if (state_machine_output.lc_turn_light > 0) {
    lateral_output.turn_light_source = "lc_turn_light";
  } else if (state_machine_output.lb_turn_light > 0) {
    lateral_output.turn_light_source = "lb_turn_light";
  } else {
    lateral_output.turn_light_source = "none";
  }

  // auto planning_status = frame_->session()
  //                            ->planning_output_context()
  //                            .planning_status();  // attention!

  //                            pipeline_context_->planning_result
  // if (nullptr != planning_status) {
  //   planning_status->planning_result.turn_signal_cmd.set_value(
  //       turn_light);  //
  //       planning_result.planning_output->turn_single_command->turn_signal_value(turn_light)
  // }

  lateral_output.avd_car_past = avd_car_past_;
  lateral_output.avd_sp_car_past = avd_sp_car_past_;

  lateral_output.vel_sequence.resize(1);
  lateral_output.vel_sequence[0] =
      ego_cart_state_manager_->ego_v_cruise();  // attention ! or ego_v() ?

  lateral_output.cross_lsolid_line = cross_lsolid_line_;
  lateral_output.cross_rsolid_line = cross_rsolid_line_;

  lateral_output.l_poly = l_poly_;

  lateral_output.r_poly = r_poly_;

  lateral_output.behavior_suspension =
      state_machine_output.behavior_suspend;  //
                                              //   lateral suspend
  lateral_output.suspension_obs.assign(
      state_machine_output.suspend_obs.begin(),
      state_machine_output.suspend_obs.end());  //
  //   lateral suspend
  //   //       obstacles
  if (!update_lateral_info()) {  // 这里进入
    //   横向规划轨迹的输出部分
    //   //     return false;
  }
  //   //   return true;
}
bool VisionLateralMotionPlanner::update_lateral_info() {
  // //
  // world_model_->mutable_map_info_manager().get_lane_change_point(world_model_);

  // //   auto &map_info =
  // world_model_->mutable_map_info_manager().get_map_info();

  auto planning_status = frame_->mutable_session()
                             ->mutable_planning_output_context()
                             ->mutable_planning_status();  // attention!
  auto &lateral_output = frame_->mutable_session()
                             ->mutable_planning_context()
                             ->mutable_lateral_behavior_planner_output();
  auto lc_request = lateral_output.lc_request;
  auto lc_status = lateral_output.lc_status;
  // auto lb_status = lateral_output.lane_borrow;
  auto lb_request = lateral_output.lb_request;
  auto lb_status = lateral_output.lb_status;
  auto lb_info = lateral_output.lane_borrow_range;
  // //   // LOG_DEBUG("zzd arbitrator lc_status %s
  // lc_request %s lb_status %s
  // //   // lb_request %s lb_info %d", lc_status.c_str(),
  // lc_request.c_str(),
  // //   //   lb_status.c_str(), lb_request.c_str(),
  // lb_info);

  auto &state_machine_output =
      frame_->mutable_session()
          ->mutable_planning_context()
          ->mutable_lat_behavior_state_machine_output();

  int scenario = state_machine_output.scenario;
  int state = state_machine_output.curr_state;
  planning::common::LaneStatus default_lane_status;
  // //   // scenario input info
  default_lane_status.change_lane.target_gap_obs =
      planning_status->lane_status.change_lane.target_gap_obs;
  planning_status->lane_status = default_lane_status;

  if (state == ROAD_NONE || state == INTER_GS_NONE || state == INTER_TR_NONE ||
      state == INTER_TL_NONE) {
    planning_status->lane_status.status =
        planning::common::LaneStatus::Status::LANE_KEEP;
  } else {
    if (lc_request == "none") {
      if (lb_request != "none") {
        planning_status->lane_status.status =
            planning::common::LaneStatus::Status::LANE_BORROW;
        // TODO: BORROW_LANE_KEEP state to be set
        // accordingly
        if (lb_request == "left") {
          planning_status->lane_status.borrow_lane.direction = "left";
        } else if (lb_request == "right") {
          planning_status->lane_status.borrow_lane.direction = "right";
        }
        if (lb_status == "left_lane_borrow" ||
            lb_status == "right_lane_borrow") {
          planning_status->lane_status.borrow_lane.status =
              planning::common::BorrowLaneStatus::Status::IN_BORROW_LANE;
        } else if (lb_status == "left_lane_suspend" ||
                   lb_status == "right_lane_suspend") {
          planning_status->lane_status.borrow_lane.status =
              planning::common::BorrowLaneStatus::Status::BORROW_LANE_KEEP;
        } else {
          planning_status->lane_status.borrow_lane.status =
              planning::common::BorrowLaneStatus::Status::BORROW_LANE_FINISHED;
        }
      } else {
        planning_status->lane_status.status =
            planning::common::LaneStatus::Status::LANE_KEEP;
        planning_status->lane_status.change_lane.status =
            planning::common::ChangeLaneStatus::Status::CHANGE_LANE_FINISHED;
        planning_status->lane_status.borrow_lane.status =
            planning::common::BorrowLaneStatus::Status::BORROW_LANE_FINISHED;
      }
    } else {
      planning_status->lane_status.status =
          planning::common::LaneStatus::Status::LANE_CHANGE;

      if (virtual_lane_manager_->get_tasks(
              virtual_lane_manager_->get_current_lane()) <
              0 ||  // get_tasks(virtual_lane_manager_->get_current_lane)
          virtual_lane_manager_->get_tasks(
              virtual_lane_manager_->get_current_lane()) > 0) {  // attention!!

        planning_status->lane_status.change_lane.is_active_lane_change = false;
      } else {
        planning_status->lane_status.change_lane.is_active_lane_change = true;
      }
      if (lc_status == "none") {
        // lane change preparation stage
        planning_status->lane_status.change_lane.status =
            planning::common::ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION;
        if (lc_request == "left") {
          planning_status->lane_status.change_lane.direction = "left";
        } else if (lc_request == "right") {
          planning_status->lane_status.change_lane.direction = "right";
        }
      } else if (lc_status == "left_lane_change" ||
                 lc_status == "right_lane_change") {
        planning_status->lane_status.change_lane.status =
            planning::common::ChangeLaneStatus::Status::IN_CHANGE_LANE;
        if (lc_status == "left_lane_change") {
          planning_status->lane_status.change_lane.direction = "left";
        } else if (lc_status == "right_lane_change") {
          planning_status->lane_status.change_lane.direction = "right";
        }
      } else if (lc_status == "left_lane_change_back" ||
                 lc_status == "right_lane_change_back") {
        planning_status->lane_status.change_lane.status =
            planning::common::ChangeLaneStatus::Status::CHANGE_LANE_BACK;
        if (lc_status == "left_lane_change_back") {
          planning_status->lane_status.change_lane.direction = "left";
        } else if (lc_status == "right_lane_change_back") {
          planning_status->lane_status.change_lane.direction = "right";
        }
      } else if (lc_status == "left_lane_change_wait" ||
                 lc_status == "right_lane_change_wait") {
        planning_status->lane_status.change_lane.status =
            planning::common::ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION;
        if (lc_status == "left_lane_change_wait") {
          planning_status->lane_status.change_lane.direction = "left";
        } else if (lc_status == "right_lane_change_wait") {
          planning_status->lane_status.change_lane.direction = "right";
        }
      }
    }
  }

  // //   // update target lane id
  int target_lane_id = 0;
  if (lateral_output.which_lane == "left_line") {
    target_lane_id = -1;
  } else if (lateral_output.which_lane == "right_line") {
    target_lane_id = 1;
  } else {
    target_lane_id = 0;
  }
  planning_status->lane_status.target_lane_id = target_lane_id;
  planning_status->lane_status.change_lane.path_id = target_lane_id;
  if (planning_status->lane_status.status ==
          planning::common::LaneStatus::Status::LANE_CHANGE &&
      (planning_status->lane_status.change_lane.status ==
           planning::common::ChangeLaneStatus::Status::
               CHANGE_LANE_PREPARATION ||
       planning_status->lane_status.change_lane.status ==
           planning::common::ChangeLaneStatus::Status::CHANGE_LANE_BACK)) {
    if (lc_request == "left") {
      planning_status->lane_status.change_lane.path_id = target_lane_id - 1;
    } else if (lc_request == "right") {
      planning_status->lane_status.change_lane.path_id = target_lane_id + 1;
    }
  }
  planning_status->lane_status.target_lane_lat_offset =
      lateral_output.lat_offset;

  // //   auto baseline_info =
  // world_model_->get_baseline_info(target_lane_id);

  // //   // todo: add target baseline protection in lane
  // change /lane borrow stage
  // //   if (baseline_info == nullptr ||
  // !baseline_info->is_valid()) {
  // //     LOG_ERROR("zzd arbitrator invalid target
  // lane[%d]!", target_lane_id);
  // //     return false;
  // //   }

  // //   // construct_virtual_obstacles();
  return true;
}
bool VisionLateralMotionPlanner::update_planner_status() {
  auto &lateral_output = frame_->mutable_session()
                             ->mutable_planning_context()
                             ->mutable_lateral_behavior_planner_output();
  auto &state_machine_output =
      frame_->mutable_session()
          ->mutable_planning_context()
          ->mutable_lat_behavior_state_machine_output();

  lateral_output.planner_scene = 0;
  lateral_output.planner_action = 0;
  lateral_output.planner_status = 0;

  switch (state_machine_output.curr_state) {
    case ROAD_NONE:
      lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
      break;

    case ROAD_LC_LWAIT:
      lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
      lateral_output.planner_action = AlgorithmAction::LANE_CHANGE_LEFT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
      break;

    case ROAD_LC_RWAIT:
      lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
      lateral_output.planner_action = AlgorithmAction::LANE_CHANGE_RIGHT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
      break;

    case ROAD_LC_LCHANGE:
      lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
      lateral_output.planner_action = AlgorithmAction::LANE_CHANGE_LEFT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
      break;

    case ROAD_LC_RCHANGE:
      lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
      lateral_output.planner_action = AlgorithmAction::LANE_CHANGE_RIGHT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
      break;

    case ROAD_LC_LBACK:
      lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
      lateral_output.planner_action = AlgorithmAction::LANE_CHANGE_LEFT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
      break;

    case ROAD_LC_RBACK:
      lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
      lateral_output.planner_action = AlgorithmAction::LANE_CHANGE_RIGHT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
      break;

    case ROAD_LB_LBORROW:
      lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
      lateral_output.planner_action = AlgorithmAction::LANE_BORROW_LEFT;
      lateral_output.planner_status = AlgorithmStatus::LANE_BORROWING;
      break;

    case ROAD_LB_RBORROW:
      lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
      lateral_output.planner_action = AlgorithmAction::LANE_BORROW_RIGHT;
      lateral_output.planner_status = AlgorithmStatus::LANE_BORROWING;
      break;

    case ROAD_LB_LBACK:
      lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
      lateral_output.planner_action = AlgorithmAction::LANE_BORROW_LEFT;
      lateral_output.planner_status = AlgorithmStatus::LANE_BORROW_BACK;
      break;

    case ROAD_LB_RBACK:
      lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
      lateral_output.planner_action = AlgorithmAction::LANE_BORROW_RIGHT;
      lateral_output.planner_status = AlgorithmStatus::LANE_BORROW_BACK;
      break;

    case ROAD_LB_LRETURN:
      lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
      lateral_output.planner_action = AlgorithmAction::LANE_BORROW_LEFT;
      lateral_output.planner_status = AlgorithmStatus::LANE_BORROW_RETURN;
      break;

    case ROAD_LB_RRETURN:
      lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
      lateral_output.planner_action = AlgorithmAction::LANE_BORROW_RIGHT;
      lateral_output.planner_status = AlgorithmStatus::LANE_BORROW_RETURN;
      break;

    case ROAD_LB_LSUSPEND:
      lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
      lateral_output.planner_action = AlgorithmAction::LANE_BORROW_LEFT;
      lateral_output.planner_status = AlgorithmStatus::LANE_BORROW_SUSPEND;
      break;

    case ROAD_LB_RSUSPEND:
      lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
      lateral_output.planner_action = AlgorithmAction::LANE_BORROW_RIGHT;
      lateral_output.planner_status = AlgorithmStatus::LANE_BORROW_SUSPEND;
      break;

    case INTER_GS_NONE:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_GO_STRAIGHT;
      break;

    case INTER_GS_LC_LWAIT:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_GO_STRAIGHT |
                                      AlgorithmAction::LANE_CHANGE_LEFT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
      break;

    case INTER_GS_LC_RWAIT:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_GO_STRAIGHT |
                                      AlgorithmAction::LANE_CHANGE_RIGHT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
      break;

    case INTER_GS_LC_LCHANGE:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_GO_STRAIGHT |
                                      AlgorithmAction::LANE_CHANGE_LEFT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
      break;

    case INTER_GS_LC_RCHANGE:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_GO_STRAIGHT |
                                      AlgorithmAction::LANE_CHANGE_RIGHT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
      break;

    case INTER_GS_LC_LBACK:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_GO_STRAIGHT |
                                      AlgorithmAction::LANE_CHANGE_LEFT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
      break;

    case INTER_GS_LC_RBACK:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_GO_STRAIGHT |
                                      AlgorithmAction::LANE_CHANGE_RIGHT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
      break;

    case INTER_TR_NONE:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_RIGHT;
      break;

    case INTER_TR_LC_LWAIT:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_RIGHT |
                                      AlgorithmAction::LANE_CHANGE_LEFT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
      break;

    case INTER_TR_LC_RWAIT:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_RIGHT |
                                      AlgorithmAction::LANE_CHANGE_RIGHT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
      break;

    case INTER_TR_LC_LCHANGE:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_RIGHT |
                                      AlgorithmAction::LANE_CHANGE_LEFT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
      break;

    case INTER_TR_LC_RCHANGE:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_RIGHT |
                                      AlgorithmAction::LANE_CHANGE_RIGHT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
      break;

    case INTER_TR_LC_LBACK:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_RIGHT |
                                      AlgorithmAction::LANE_CHANGE_LEFT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
      break;

    case INTER_TR_LC_RBACK:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_RIGHT |
                                      AlgorithmAction::LANE_CHANGE_RIGHT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
      break;
      //
    case INTER_TL_NONE:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_LEFT;
      break;

    case INTER_TL_LC_LWAIT:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_LEFT |
                                      AlgorithmAction::LANE_CHANGE_LEFT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
      break;

    case INTER_TL_LC_RWAIT:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_LEFT |
                                      AlgorithmAction::LANE_CHANGE_RIGHT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
      break;

    case INTER_TL_LC_LCHANGE:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_LEFT |
                                      AlgorithmAction::LANE_CHANGE_LEFT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
      break;

    case INTER_TL_LC_RCHANGE:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_LEFT |
                                      AlgorithmAction::LANE_CHANGE_RIGHT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
      break;

    case INTER_TL_LC_LBACK:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_LEFT |
                                      AlgorithmAction::LANE_CHANGE_LEFT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
      break;

    case INTER_TL_LC_RBACK:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_LEFT |
                                      AlgorithmAction::LANE_CHANGE_RIGHT;
      lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
      break;

    case INTER_UT_NONE:
      lateral_output.planner_scene = AlgorithmScene::INTERSECT;
      lateral_output.planner_action = AlgorithmAction::INTERSECT_U_TURN;
      break;

    default:
      break;
  }

  if (lateral_output.sb_blane) {
    lateral_output.planner_action |=

        AlgorithmAction::LANE_BORROW_IN_NON_MOTORIZED_LANE;
  }
}
//   // bool
//   VisionLateralMotionPlanner::log_planner_debug_info()
//   {
//   //   std::string plan_msg;
//   //   create_lateral_behavior_planner_msg(plan_msg);

//   //   auto &lateral_behavior_planner_output =
//   //
//   PlanningContext::Instance()->mutable_lateral_behavior_planner_output();

//   //   lateral_behavior_planner_output.plan_msg =
//   plan_msg;
//   // }
//   // bool
//   VisionLateralMotionPlanner::create_lateral_behavior_planner_msg(
//   //     std::string &plan_msg) {
//   //   auto &lateral_output =
//   //
//   PlanningContext::Instance()->lateral_behavior_planner_output();
//   //   rapidjson::Document
//   later_out_json(rapidjson::kObjectType);
//   //   rapidjson::Document::AllocatorType &allocator =
//   //   later_out_json.GetAllocator();

//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/timestamp",
//   // lateral_output.timestamp);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/enable",
//   // lateral_output.enable);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/track_id",
//   // lateral_output.track_id);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/v_limit",
//   // lateral_output.v_limit);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/lat_offset",
//   // lateral_output.lat_offset);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/lane_borrow",
//   // lateral_output.lane_borrow);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/lane_borrow_range",
//   // lateral_output.lane_borrow_range);

//   //   rapidjson::Document
//   avd_array(rapidjson::kArrayType, &allocator);
//   //   for (int i = 0; i <
//   lateral_output.avd_info.size(); i++)
//   {
//   //     rapidjson::Document
//   one_avd_obj(rapidjson::kObjectType, &allocator);
//   //     rapidjson::Value str_val;
//   //     rapidjson::SetValueByPointer(one_avd_obj,
//   "/id",
//   // lateral_output.avd_info[i].id);
//   //
//   str_val.SetString(lateral_output.avd_info[i].property.c_str(),
//   // lateral_output.avd_info[i].property.length(),
//   //                       allocator);
//   //     rapidjson::SetValueByPointer(one_avd_obj,
//   "/property", str_val);
//   //     rapidjson::SetValueByPointer(one_avd_obj,
//   "/ignore",
//   // lateral_output.avd_info[i].ignore);
//   //
//   str_val.SetString(lateral_output.avd_info[i].avd_direction.c_str(),
//   // lateral_output.avd_info[i].avd_direction.length(),
//   //                       allocator);
//   //     rapidjson::SetValueByPointer(one_avd_obj,
//   "/avd_direction", str_val);
//   //     rapidjson::SetValueByPointer(one_avd_obj,
//   "/avd_priority",
//   // lateral_output.avd_info[i].avd_priority);
//   //     rapidjson::SetValueByPointer(one_avd_obj,
//   "/blocked_time_begin",
//   // lateral_output.avd_info[i].blocked_time_begin);
//   //     rapidjson::SetValueByPointer(one_avd_obj,
//   "/blocked_time_end",
//   // lateral_output.avd_info[i].blocked_time_end);
//   //     avd_array.PushBack(one_avd_obj, allocator);
//   //   }
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/avd_info", avd_array);

//   //   rapidjson::Value str_temp;
//   //
//   str_temp.SetString(lateral_output.which_lane.c_str(),
//   // lateral_output.which_lane.length(), allocator);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/which_lane", str_temp);
//   //
//   str_temp.SetString(lateral_output.lb_request.c_str(),
//   // lateral_output.lb_request.length(), allocator);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/lb_request", str_temp);
//   //
//   str_temp.SetString(lateral_output.lc_request.c_str(),
//   // lateral_output.lc_request.length(), allocator);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/lc_request", str_temp);
//   //
//   str_temp.SetString(lateral_output.lc_status.c_str(),
//   // lateral_output.lc_status.length(), allocator);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/lc_status", str_temp);
//   //
//   str_temp.SetString(lateral_output.lb_status.c_str(),
//   // lateral_output.lb_status.length(), allocator);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/lb_status", str_temp);

//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/scenario",
//   // lateral_output.scenario);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/flane_width",
//   // lateral_output.flane_width);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/dist_rblane",
//   // lateral_output.dist_rblane);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/borrow_bicycle_lane",
//   // lateral_output.borrow_bicycle_lane);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   //   "/enable_intersection_planner",
//   // lateral_output.enable_intersection_planner);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/tleft_lane",
//   // lateral_output.tleft_lane);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/rightest_lane",
//   // lateral_output.rightest_lane);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/isFasterStaticAvd",
//   // lateral_output.isFasterStaticAvd);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/isOnHighway",
//   // lateral_output.isOnHighway);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/isRedLightStop",
//   // lateral_output.isRedLightStop);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/dist_intersect",
//   // lateral_output.dist_intersect);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/intersect_length",
//   // lateral_output.intersect_length);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/lc_end_dis",
//   // lateral_output.lc_end_dis);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/dis_to_ramp",
//   // lateral_output.dis_to_ramp);

//   //   /* rapidjson::Document
//   path_pnt_array(rapidjson::kArrayType, &allocator);
//   //   for(int i = 0; i <
//   lateral_output.path_points.size(); i++)
//   //   {
//   //     rapidjson::Document
//   one_path_pnt(rapidjson::kObjectType, &allocator);
//   //     rapidjson::SetValueByPointer(one_path_pnt,
//   "/x",
//   //   lateral_output.path_points[i].x);
//   //   rapidjson::SetValueByPointer(one_path_pnt,
//   //   "/y", lateral_output.path_points[i].y);
//   //     rapidjson::SetValueByPointer(one_path_pnt,
//   "/z",
//   //   lateral_output.path_points[i].z);
//   //   rapidjson::SetValueByPointer(one_path_pnt,
//   //   "/theta", lateral_output.path_points[i].theta);
//   //     rapidjson::SetValueByPointer(one_path_pnt,
//   "/kappa",
//   //   lateral_output.path_points[i].kappa);
//   //     rapidjson::SetValueByPointer(one_path_pnt,
//   "/s",
//   //   lateral_output.path_points[i].s);
//   //   rapidjson::SetValueByPointer(one_path_pnt,
//   //   "/dkappa",
//   lateral_output.path_points[i].dkappa);
//   //     rapidjson::SetValueByPointer(one_path_pnt,
//   "/ddkappa",
//   //   lateral_output.path_points[i].ddkappa);
//   //     rapidjson::SetValueByPointer(one_path_pnt,
//   "/lane_id",
//   //   lateral_output.path_points[i].lane_id.c_str());
//   //     rapidjson::SetValueByPointer(one_path_pnt,
//   "/x_derivative",
//   //   lateral_output.path_points[i].x_derivative);
//   //     rapidjson::SetValueByPointer(one_path_pnt,
//   "/y_derivative",
//   //   lateral_output.path_points[i].y_derivative);
//   //     path_pnt_array.PushBack(one_path_pnt,
//   allocator);
//   //   }
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/path_points",
//   //   path_pnt_array);
//   // */

//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/sb_lane",
//   // lateral_output.sb_lane);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/sb_blane",
//   // lateral_output.sb_blane);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/force_pause",
//   // lateral_output.force_pause);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/large_lat",
//   // lateral_output.large_lat);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/premoving",
//   // lateral_output.premoving);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/accident_ahead",
//   // lateral_output.accident_ahead);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/accident_back",
//   // lateral_output.accident_back);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/lc_pause_id",
//   // lateral_output.lc_pause_id);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/lc_pause",
//   // lateral_output.lc_pause);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/tr_pause_l",
//   // lateral_output.tr_pause_l);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/tr_pause_s",
//   // lateral_output.tr_pause_s);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/must_change_lane",
//   // lateral_output.must_change_lane);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/left_faster",
//   // lateral_output.left_faster);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/right_faster",
//   // lateral_output.right_faster);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/close_to_accident",
//   // lateral_output.close_to_accident);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/premove",
//   // lateral_output.premove);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/behavior_suspension",
//   // lateral_output.behavior_suspension);

//   //   rapidjson::Document
//   suspension_ob_array(rapidjson::kArrayType,
//   //   &allocator); for (int i = 0; i <
//   lateral_output.suspension_obs.size();
//   //   i++) {
//   //
//   suspension_ob_array.PushBack(lateral_output.suspension_obs[i],
//   //     allocator);
//   //   }
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/suspension_obs",
//   // suspension_ob_array);

//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/angle_steers_limit",
//   // lateral_output.angle_steers_limit);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/premove_dist",
//   // lateral_output.premove_dist);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/planner_scene",
//   // lateral_output.planner_scene);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/planner_action",
//   // lateral_output.planner_action);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/planner_status",
//   // lateral_output.planner_status);

//   //   rapidjson::Document
//   c_poly_array(rapidjson::kArrayType, &allocator);
//   //   for (int i = 0; i <
//   lateral_output.c_poly.size(); i++) {
//   // c_poly_array.PushBack(lateral_output.c_poly[i],
//   allocator);
//   //   }
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/c_poly", c_poly_array);

//   //   rapidjson::Document
//   d_poly_array(rapidjson::kArrayType, &allocator);
//   //   for (int i = 0; i <
//   lateral_output.d_poly.size(); i++) {
//   // d_poly_array.PushBack(lateral_output.d_poly[i],
//   allocator);
//   //   }
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/d_poly", d_poly_array);

//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/lead_one_drel",
//   // lateral_output.lead_one_drel);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/lead_one_vrel",
//   // lateral_output.lead_one_vrel);

//   //
//   str_temp.SetString(lateral_output.state_name.c_str(),
//   // lateral_output.state_name.length(), allocator);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/state_name", str_temp);
//   //
//   str_temp.SetString(lateral_output.scenario_name.c_str(),
//   // lateral_output.scenario_name.length(), allocator);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/scenario_name", str_temp);
//   //
//   str_temp.SetString(lateral_output.turn_light.c_str(),
//   // lateral_output.turn_light.length(), allocator);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/turn_light", str_temp);
//   //
//   str_temp.SetString(lateral_output.lc_request_source.c_str(),
//   // lateral_output.lc_request_source.length(),
//   allocator);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/lc_request_source",
//   //   str_temp);
//   str_temp.SetString(lateral_output.act_request_source.c_str(),
//   // lateral_output.act_request_source.length(),
//   //                      allocator);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/act_request_source",
//   //   str_temp);
//   str_temp.SetString(lateral_output.turn_light_source.c_str(),
//   // lateral_output.turn_light_source.length(),
//   allocator);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/turn_light_source",
//   //   str_temp);
//   rapidjson::SetValueByPointer(later_out_json,
//   "/lb_width",
//   // lateral_output.lb_width);

//   //   rapidjson::Document
//   v_seq_array(rapidjson::kArrayType, &allocator);
//   //   for (int i = 0; i <
//   lateral_output.vel_sequence.size(); i++) {
//   //
//   v_seq_array.PushBack(lateral_output.vel_sequence[i],
//   allocator);
//   //   }
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/vel_sequence",
//   //   v_seq_array);

//   //   rapidjson::Document
//   ig_change_f_array(rapidjson::kArrayType, &allocator);
//   //   for (auto i =
//   lateral_output.ignore_change_false.begin();
//   //        i !=
//   lateral_output.ignore_change_false.end(); i++)
//   {
//   //     ig_change_f_array.PushBack(*i, allocator);
//   //   }
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/ignore_change_false",
//   //                                ig_change_f_array);

//   //   rapidjson::Document
//   ig_change_t_array(rapidjson::kArrayType, &allocator);
//   //   for (auto i =
//   lateral_output.ignore_change_true.begin();
//   //        i !=
//   lateral_output.ignore_change_true.end(); i++)
//   {
//   //     ig_change_t_array.PushBack(*i, allocator);
//   //   }
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/ignore_change_true",
//   //                                ig_change_t_array);

//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/cross_lsolid_line",
//   // lateral_output.cross_lsolid_line);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/cross_rsolid_line",
//   // lateral_output.cross_rsolid_line);

//   //   rapidjson::Document
//   l_poly_array(rapidjson::kArrayType, &allocator);
//   //   for (int i = 0; i <
//   lateral_output.l_poly.size(); i++) {
//   // l_poly_array.PushBack(lateral_output.l_poly[i],
//   allocator);
//   //   }
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/l_poly", l_poly_array);

//   //   rapidjson::Document
//   r_poly_array(rapidjson::kArrayType, &allocator);
//   //   for (int i = 0; i <
//   lateral_output.r_poly.size(); i++) {
//   // r_poly_array.PushBack(lateral_output.r_poly[i],
//   allocator);
//   //   }
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/r_poly", r_poly_array);

//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/disable_l",
//   // lateral_output.disable_l);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/disable_r",
//   // lateral_output.disable_r);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/enable_l",
//   // lateral_output.enable_l);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/enable_r",
//   // lateral_output.enable_r);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/enable_id",
//   // lateral_output.enable_id);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/lc_warning",
//   // lateral_output.lc_warning);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/avd_in_lane",
//   // lateral_output.avd_in_lane);
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/lc_exit_ramp_start",
//   // lateral_output.lc_exit_ramp_start);

//   //   /* rapidjson::Document
//   s_v_limit_array(rapidjson::kArrayType,
//   //   &allocator); for(int i = 0; i <
//   lateral_output.s_v_limit.size(); i++)
//   //   {
//   //     rapidjson::Document
//   one_s_v_limit(rapidjson::kArrayType, &allocator);
//   //
//   one_s_v_limit.PushBack(std::get<0>(lateral_output.s_v_limit[i]),
//   //     allocator);
//   //
//   one_s_v_limit.PushBack(std::get<1>(lateral_output.s_v_limit[i]),
//   //     allocator);

//   //     s_v_limit_array.PushBack(one_s_v_limit,
//   allocator);
//   //   }
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/s_v_limit",
//   //   s_v_limit_array);

//   //   rapidjson::Document
//   s_a_limit_array(rapidjson::kArrayType, &allocator);
//   //   for(int i = 0; i <
//   lateral_output.s_a_limit.size(); i++)
//   //   {
//   //     rapidjson::Document
//   one_s_a_limit(rapidjson::kArrayType, &allocator);
//   //
//   one_s_a_limit.PushBack(std::get<0>(lateral_output.s_a_limit[i]),
//   //     allocator);
//   //
//   one_s_a_limit.PushBack(std::get<1>(lateral_output.s_a_limit[i]),
//   //     allocator);

//   //     s_a_limit_array.PushBack(one_s_a_limit,
//   allocator);
//   //   }
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/s_a_limit",
//   //   s_a_limit_array);

//   //   rapidjson::Document
//   s_r_off_array(rapidjson::kArrayType, &allocator);
//   //   for(int i = 0; i <
//   lateral_output.s_r_offset.size(); i++)
//   //   {
//   //     rapidjson::Document
//   one_s_r_off(rapidjson::kArrayType, &allocator);
//   //
//   one_s_r_off.PushBack(std::get<0>(lateral_output.s_r_offset[i]),
//   //     allocator);
//   //
//   one_s_r_off.PushBack(std::get<1>(lateral_output.s_r_offset[i]),
//   //     allocator);

//   //     s_r_off_array.PushBack(one_s_r_off,
//   allocator);
//   //   }
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/s_r_offset",
//   //   s_r_off_array);
//   //   */

//   //   rapidjson::Document
//   avd_past_array(rapidjson::kArrayType, &allocator);
//   //   for (int i = 0; i <
//   lateral_output.avd_car_past.size(); i++) {
//   //     rapidjson::Document
//   one_avd_past(rapidjson::kArrayType, &allocator);
//   //     for (int j = 0; j <
//   lateral_output.avd_car_past[i].size(); j++)
//   {
//   //
//   one_avd_past.PushBack(lateral_output.avd_car_past[i][j],
//   allocator);
//   //     }
//   //     avd_past_array.PushBack(one_avd_past,
//   allocator);
//   //   }
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/avd_car_past",
//   //   avd_past_array);

//   //   rapidjson::Document
//   avd_sp_past_array(rapidjson::kArrayType, &allocator);
//   //   for (int i = 0; i <
//   lateral_output.avd_sp_car_past.size(); i++)
//   {
//   //     rapidjson::Document
//   one_avd_sp_past(rapidjson::kArrayType, &allocator);
//   //     for (int j = 0; j <
//   lateral_output.avd_sp_car_past[i].size(); j++)
//   {
//   //
//   one_avd_sp_past.PushBack(lateral_output.avd_sp_car_past[i][j],
//   //       allocator);
//   //     }
//   //     avd_sp_past_array.PushBack(one_avd_sp_past,
//   allocator);
//   //   }
//   //   rapidjson::SetValueByPointer(later_out_json,
//   "/avd_sp_car_past",
//   //                                avd_sp_past_array);

//   //   rapidjson::StringBuffer jsonBuffer;
//   //   rapidjson::Writer<rapidjson::StringBuffer>
//   writer(jsonBuffer);
//   //   later_out_json.Accept(writer);
//   //   plan_msg = std::string(jsonBuffer.GetString(),
//   jsonBuffer.GetSize());

void VisionLateralMotionPlanner::calc_desired_path(const std::array<double, 4> &l_poly,
                       const std::array<double, 4> &r_poly, double l_prob,
                       double r_prob, double intercept_width,
                       std::array<double, 4> &d_poly) {
  std::array<double, 4> half_lane_poly{0, 0, 0, intercept_width / 2};

  if (l_prob + r_prob > 0) {
    for (size_t i = 0; i < d_poly.size(); i++) {
      d_poly[i] = ((l_poly[i] - half_lane_poly[i]) * l_prob +
                   (r_poly[i] + half_lane_poly[i]) * r_prob) /
                  (l_prob + r_prob);
    }
  } else {
    d_poly.fill(0);
  }
}

double VisionLateralMotionPlanner::calc_lane_width_by_dist(
    const std::array<double, 4> &left_poly,
    const std::array<double, 4> &right_poly, const double &dist_x) {
  std::vector<double> left_poly_yx, r_poly_yx;
  left_poly_yx.resize(left_poly.size());
  r_poly_yx.resize(right_poly.size());
  std::reverse_copy(left_poly.begin(), left_poly.end(), left_poly_yx.begin());
  std::reverse_copy(right_poly.begin(), right_poly.end(), r_poly_yx.begin());

  double left_intercept = calc_poly1d(left_poly_yx, dist_x);
  double right_intercept = calc_poly1d(r_poly_yx, dist_x);

  if (left_poly_yx.size() >= 2 && r_poly_yx.size() >= 2) {
    return (left_intercept - right_intercept) /
           std::sqrt(1 + std::pow(0.5 * (left_poly_yx[1] + r_poly_yx[1]), 2));
  } else {
    return 3.8;
  }
}

}  // namespace planning
