#include "general_lateral_decider.h"

#include <assert.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "agent/agent.h"
#include "agent_node_manager.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "frenet_ego_state.h"
#include "general_lateral_decider_utils.h"
#include "log.h"
#include "task_basic_types.h"
#include "utils/hysteresis_decision.h"
#include "utils/kd_path.h"
#include "vehicle_config_context.h"
#include "virtual_lane_manager.h"
#include "agent/agent.h"

namespace planning {

using namespace planning_math;
using namespace pnc::spline;

GeneralLateralDecider::GeneralLateralDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<GeneralLateralDeciderConfig>();
  name_ = "GeneralLateralDecider";
}

bool GeneralLateralDecider::InitInfo() {
  reference_path_ptr_ = session_->planning_context()
                            .lane_change_decider_output()
                            .coarse_planning_info.reference_path;
  if (reference_path_ptr_ == nullptr) {
    return false;
  }

  cruise_vel_ = session_->planning_context().v_ref_cruise();

  is_lane_change_scene_ = false;

  auto &general_lateral_decider_output =
      session_->mutable_planning_context()
          ->mutable_general_lateral_decider_output();
  general_lateral_decider_output.Clear();

  lat_lane_change_info_ = LatDeciderLaneChangeInfo::NONE;

  ego_frenet_state_ = reference_path_ptr_->get_frenet_ego_state();

  ego_cart_state_manager_ =
      session_->environmental_model().get_ego_state_manager();
  if (ego_cart_state_manager_ == nullptr) {
    // add logs
    return false;
  }
  static_obstacle_decisions_.clear();
  dynamic_obstacle_decisions_.clear();
  ref_traj_points_.clear();
  plan_history_traj_.clear();
  match_index_map_.clear();
  ref_path_points_.clear();
  soft_bounds_.clear();
  hard_bounds_.clear();
  is_blocked_obstacle_ =false;
  return true;
}

bool GeneralLateralDecider::Execute() {
  LOG_DEBUG("=======GeneralLateralDecider======= \n");

  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    last_lat_obstacle_decision_.clear();
    extra_lane_width_decrease_buffer_ = 0.0;
    return false;
  }

  auto start_time = IflyTime::Now_ms();

  if (!InitInfo()) {
    last_lat_obstacle_decision_.clear();
    extra_lane_width_decrease_buffer_ = 0.0;
    return false;
  };

  auto &traj_points = session_->mutable_planning_context()
                          ->mutable_planning_result()
                          .traj_points;

  ConstructTrajPoints(traj_points);
  ConstructReferencePathPoints(traj_points);

  GenerateRoadAndLaneBoundary();

  GenerateObstaclesBoundary();
  // UnitTest();
  std::vector<std::pair<double, double>> frenet_soft_bounds;
  std::vector<std::pair<double, double>> frenet_hard_bounds;
  std::vector<std::pair<BoundInfo, BoundInfo>> soft_bounds_info;
  std::vector<std::pair<BoundInfo, BoundInfo>> hard_bounds_info;

  ExtractBoundary(frenet_soft_bounds, frenet_hard_bounds, soft_bounds_info,
                  hard_bounds_info);
  CalculateAvoidObstacles(frenet_soft_bounds, soft_bounds_info);

  auto &general_lateral_decider_output =
      session_->mutable_planning_context()
          ->mutable_general_lateral_decider_output();
  PostProcessReferenceTrajBySoftBound(frenet_soft_bounds,
                                      general_lateral_decider_output);
  GenerateLateralDeciderOutput(frenet_soft_bounds, frenet_hard_bounds,
                               general_lateral_decider_output);

  CalcLateralBehaviorOutput();

  SaveLatDebugInfo(frenet_soft_bounds, frenet_hard_bounds, soft_bounds_info,
                   hard_bounds_info, general_lateral_decider_output);

  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("GeneralLateralDeciderCostTime", end_time - start_time);

  return true;
}

bool GeneralLateralDecider::ExecuteTest(bool pipeline_test) {
  // pipeline test
  return true;
}

void GeneralLateralDecider::UnitTest() {
  for (int i = 1; i < 12; i++) {
    std::vector<WeightedBound> bounds_input;
    std::pair<double, double> bound_output{-10., 10.};
    std::pair<BoundInfo, BoundInfo> bound_info;
    double init_l = 0;
    switch (i) {
      case 1: {
        // case 1:
        //           type
        //           upper     10    2
        //           lower               -1   -10
        // result                    2   -1
        // bounds_input.emplace_back(
        //     WeightedBound{-10, 10, config_.kPhysicalBoundWeight,
        //                   BoundInfo{-100, BoundType::ROAD_BORDER}});
        bounds_input.emplace_back(
            WeightedBound{-1, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        bounds_input.emplace_back(
            WeightedBound{-10, 2, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        init_l = 0;
      } break;
      case 2: {
        // case 2:
        //           type
        //           upper     10      -1
        //           lower          3       -10
        // result                     1
        // bounds_input.emplace_back(
        //     WeightedBound{-10, 10, config_.kPhysicalBoundWeight,
        //                   BoundInfo{-100, BoundType::ROAD_BORDER}});
        bounds_input.emplace_back(
            WeightedBound{-10, -1, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        bounds_input.emplace_back(
            WeightedBound{3, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        init_l = 0;
      } break;
      case 3: {
        // case 3:
        //           type          J   J
        //           upper         -1
        //           lower            -3
        // init                   0
        // result                 0   -3
        bounds_input.emplace_back(
            WeightedBound{-3, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::ADJACENT_AGENT}});
        bounds_input.emplace_back(
            WeightedBound{-10, -1, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::ADJACENT_AGENT}});
        init_l = 0;
      } break;
      case 4: {
        // case 4:
        //           type           J        A      A
        //           upper     10
        //           lower          3        2      1  -10
        // init                         2.5
        // result                        *
        bounds_input.emplace_back(
            WeightedBound{-10, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::ROAD_BORDER}});
        bounds_input.emplace_back(
            WeightedBound{3, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::ADJACENT_AGENT}});
        bounds_input.emplace_back(
            WeightedBound{2, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        bounds_input.emplace_back(
            WeightedBound{1, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        init_l = 2.5;
      } break;
      case 5: {
        // case 5:
        //           type           J    A      A        A
        //           upper     10
        //           lower          3    2      1       -2    -10
        // init                             1.5
        // result                        *
        bounds_input.emplace_back(
            WeightedBound{-10, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::ROAD_BORDER}});
        bounds_input.emplace_back(
            WeightedBound{3, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::ADJACENT_AGENT}});
        bounds_input.emplace_back(
            WeightedBound{2, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        bounds_input.emplace_back(
            WeightedBound{1, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        bounds_input.emplace_back(
            WeightedBound{-2, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        init_l = 1.5;
      } break;
      case 6: {
        // case 6:
        //           type           A    J      A        A
        //           upper     10
        //           lower          3    2      1       -2    -10
        // init                             1.5
        // result                   *
        bounds_input.emplace_back(
            WeightedBound{-10, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::ROAD_BORDER}});
        bounds_input.emplace_back(
            WeightedBound{3, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        bounds_input.emplace_back(
            WeightedBound{2, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::ADJACENT_AGENT}});
        bounds_input.emplace_back(
            WeightedBound{1, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        bounds_input.emplace_back(
            WeightedBound{-2, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        init_l = 1.5;
      } break;

      case 7: {
        // case 7:
        //           type           J   A    A    A   A     A
        //           upper     10      2.5        0
        //           lower          3        1       -1   -2    -10
        // init                          2
        // result                          *
        bounds_input.emplace_back(
            WeightedBound{-10, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::ROAD_BORDER}});
        bounds_input.emplace_back(
            WeightedBound{-10, 2.5, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        bounds_input.emplace_back(
            WeightedBound{-10, 0, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        bounds_input.emplace_back(
            WeightedBound{3, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::ADJACENT_AGENT}});
        bounds_input.emplace_back(
            WeightedBound{1, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        bounds_input.emplace_back(
            WeightedBound{-1, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        bounds_input.emplace_back(
            WeightedBound{-2, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        init_l = 2;
      } break;
      case 8: {
        // case 8:
        //           type           J   A      A  A   A
        //           upper     10      2.5        0
        //           lower          3         0.5     -1   -10
        // init                              1
        // result                            *
        bounds_input.emplace_back(
            WeightedBound{-10, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::ROAD_BORDER}});
        bounds_input.emplace_back(
            WeightedBound{-10, 2.5, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        bounds_input.emplace_back(
            WeightedBound{-10, 0, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        bounds_input.emplace_back(
            WeightedBound{3, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::ADJACENT_AGENT}});
        bounds_input.emplace_back(
            WeightedBound{0.5, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        bounds_input.emplace_back(
            WeightedBound{-1, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        init_l = 1;
      } break;

      case 9: {
        // case 9:
        //           type           J   A      A   A
        //           upper     10      2.5         0
        //           lower          3  2.5        -1   -10
        // init                               0.75
        // result                           *
        bounds_input.emplace_back(
            WeightedBound{-10, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::ROAD_BORDER}});
        bounds_input.emplace_back(
            WeightedBound{-10, 2.5, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        bounds_input.emplace_back(
            WeightedBound{-10, 0, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        bounds_input.emplace_back(
            WeightedBound{3, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::ADJACENT_AGENT}});
        bounds_input.emplace_back(
            WeightedBound{2.5, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        bounds_input.emplace_back(
            WeightedBound{-1, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        init_l = 0.75;
      } break;
      case 10: {
        // case 10:
        //           type           J   A      A   J
        //           upper     10      2.5         0
        //           lower          3          1      -1   -10
        // init                                1
        // result                              *
        bounds_input.emplace_back(
            WeightedBound{-10, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::ROAD_BORDER}});
        bounds_input.emplace_back(
            WeightedBound{-10, 2.5, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        bounds_input.emplace_back(
            WeightedBound{-10, 0, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::ADJACENT_AGENT}});
        bounds_input.emplace_back(
            WeightedBound{3, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::ADJACENT_AGENT}});
        bounds_input.emplace_back(
            WeightedBound{1, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        bounds_input.emplace_back(
            WeightedBound{-1, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        init_l = 1;
      } break;
      case 11: {
        // case 11:
        //           type           J   J
        //           upper              0
        //           lower          3
        // init                        1
        // result                      *
        bounds_input.emplace_back(
            WeightedBound{3, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::ADJACENT_AGENT}});
        bounds_input.emplace_back(
            WeightedBound{-10, 0, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::ADJACENT_AGENT}});
        init_l = 1;
      } break;
    }
    PostProcessBound(init_l, bounds_input, bound_output, bound_info);
    printf("case %d: %f %f\n", i, bound_output.first, bound_output.second);
  }
}

bool GeneralLateralDecider::CalCruiseVelByCurvature(
    const double ego_v, const CoarsePlanningInfo& coars_planning_info, double &cruise_v) {
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  if (virtual_lane_manager
          ->get_is_exist_ramp_on_road() ||
      virtual_lane_manager
          ->get_is_exist_split_on_ramp() ||
      virtual_lane_manager
          ->get_is_exist_intersection_split()) {
      // (virtual_lane_manager
      //         ->GetIntersectionState() >= common::APPROACH_INTERSECTION &&
      //  virtual_lane_manager
      //         ->GetIntersectionState() <= common::OFF_INTERSECTION)) {
    return false;
  }
  const auto &route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();
  if ((config_.ramp_limit_v_valid) && (route_info_output.is_on_ramp)) {
    cruise_v = std::min(std::max(config_.ramp_limit_v, ego_v), cruise_v);
  }
  const auto& reference_path = coars_planning_info.reference_path;
  const double init_s =
      reference_path->get_frenet_ego_state().planning_init_point().frenet_state.s;
  const auto& cart_ref_info =
      coars_planning_info.cart_ref_info;
  const double preview_length = 15.0;
  const double preview_step = 1.0;
  double sum_far_kappa = 0.0;
  double preview_s = std::max(3.0 * ego_v - 5.0, 20.0);
  if (cart_ref_info.k_s_spline.get_x().size() > 0) {
    for (double preview_distance = 0.0; preview_distance < preview_length;
        preview_distance += preview_step) {
      sum_far_kappa +=
          std::fabs(cart_ref_info.k_s_spline(init_s + preview_s + preview_distance));
    }
  } else {
    for (double preview_distance = 0.0; preview_distance < preview_length;
        preview_distance += preview_step) {
      ReferencePathPoint ref_ponit;
      reference_path->get_reference_point_by_lon(init_s + preview_s + preview_distance,
                                                 ref_ponit);
      sum_far_kappa += std::fabs(ref_ponit.path_point.kappa());
    }
  }
  if ((std::fabs(preview_length) > 1e-6) && (std::fabs(preview_step) > 1e-6)) {
    double aver_far_kappa =
        sum_far_kappa / std::max((preview_length / preview_step), 1.0);
    double far_kappa_radius = 1.0 / std::max(aver_far_kappa, 0.0001);
    JSON_DEBUG_VALUE("far_kappa_radius", far_kappa_radius);
    if (virtual_lane_manager
            ->GetIntersectionState() >= common::APPROACH_INTERSECTION &&
        virtual_lane_manager
            ->GetIntersectionState() <= common::OFF_INTERSECTION) {
      if (far_kappa_radius <= 50.0) {
        return true;
      }
    } else {
      if (far_kappa_radius < 750.0) {
        return true;
      }
    }

  }
  // std::vector<double> d_polys;
  // d_polys.resize(d_poly.size());
  // std::reverse_copy(d_poly.begin(), d_poly.end(), d_polys.begin());
  // for (double preview_distance = 0.0; preview_distance < preview_length;
  //      preview_distance += preview_step) {
  //   sum_far_kappa +=
  //       std::fabs(2 * d_polys[0] * (preview_distance + preview_x) +
  //                 d_polys[1]) /
  //       std::pow(std::pow(2 * d_polys[0] * (preview_distance + preview_x) +
  //                             d_polys[1],
  //                         2) +
  //                    1,
  //                1.5);
  // }
  // if ((std::fabs(preview_length) > 1e-6) && (std::fabs(preview_step) > 1e-6)) {
  //   double aver_far_kappa =
  //       sum_far_kappa / std::max((preview_length / preview_step), 1.0);
  //   double far_kappa_radius = 1.0 / std::max(aver_far_kappa, 0.0001);
  //   JSON_DEBUG_VALUE("far_kappa_radius", far_kappa_radius);
  //   if (far_kappa_radius < 750.0) {
  //     double road_radius = far_kappa_radius;
  //     std::array<double, 4> xp_radius{100.0, 200.0, 400.0, 600.0};
  //     std::array<double, 4> fp_acc{1.5, 0.9, 0.7, 0.6};
  //     double acc_max = interp(road_radius, xp_radius, fp_acc);
  //     cruise_v = std::min(
  //         std::max(std::sqrt(acc_max * road_radius) * 0.9, ego_v), cruise_v);
  //     return true;
  //   }
  // }
  return false;
}

void GeneralLateralDecider::ConstructTrajPoints(TrajectoryPoints &traj_points) {
  const auto &gap_selector_decider_output =
      session_->planning_context().gap_selector_decider_output();
  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  const std::shared_ptr<VirtualLane> flane =
      session_->environmental_model()
          .get_virtual_lane_manager()
          ->get_lane_with_virtual_id(coarse_planning_info.target_lane_id);
  const auto &frenet_coord =
      reference_path_ptr_->get_frenet_coord();
  const auto &planning_init_point =
      ego_frenet_state_.planning_init_point();

  bool limit_ref_vel_on_ramp_valid = false;
  bool is_LC_CHANGE =
      ((coarse_planning_info.target_state == kLaneChangeExecution) ||
       (coarse_planning_info.target_state == kLaneChangeComplete));
  bool is_LC_HOLD = coarse_planning_info.target_state == kLaneChangeHold;
  bool is_LC_BACK = coarse_planning_info.target_state == kLaneChangeCancel;

  const double lateral_offset = session_->mutable_planning_context()
                                    ->lateral_offset_decider_output()
                                    .lateral_offset;
  const double v_ego =
      session_->mutable_environmental_model()->get_ego_state_manager()->ego_v();
  std::vector<double> xp_v_ego{10.0, 15.0, 20.0, 25.0};
  double dynamic_ref_buffer =
      interp(v_ego, xp_v_ego, config_.dynamic_lc_ref_buffer);
  if (is_LC_BACK) {
    dynamic_ref_buffer =
        interp(v_ego, xp_v_ego, config_.dynamic_lc_finished_ref_buffer);
  }
  double init_dist_to_ref =
      std::fabs(planning_init_point.frenet_state.r - lateral_offset) - dynamic_ref_buffer;
  double dist_to_second_stage = init_dist_to_ref - config_.lc_second_dist_thr;
  if (is_LC_BACK) {
    dist_to_second_stage = init_dist_to_ref - config_.lc_finished_second_dist_thr;
  }
  if (dist_to_second_stage < -1e-6) {
    dynamic_ref_buffer =
        std::max(0.0, dist_to_second_stage + dynamic_ref_buffer);
  }
  if (planning_init_point.frenet_state.r < -1e-6) {
    dynamic_ref_buffer = -dynamic_ref_buffer;
  }

  if (config_.lateral_ref_traj_type ||
      ((is_LC_HOLD ||
      ((dist_to_second_stage >= 1e-6) && (is_LC_CHANGE || is_LC_BACK))) &&
       gap_selector_decider_output.gap_selector_trustworthy)) {
    traj_points = coarse_planning_info.trajectory_points;
  } else {
    // generate traj_points based on kMaxAcc or kMinAcc
    double kMaxAcc = 0.8;
    const double kMinAcc = -5.5;
    double cruise_v = std::max(config_.min_v_cruise,
                               session_->planning_context().v_ref_cruise());
    double ego_v = planning_init_point.v;
    // if (cruise_v < 4.167) {  // low speed cruise
    //   kMaxAcc = 0.4;
    // }
    if (is_LC_CHANGE || is_LC_BACK) {
      ego_v = std::max(ego_v, config_.min_v_cruise);
      kMaxAcc = 1e-6;
    }
    if (CalCruiseVelByCurvature(ego_v, coarse_planning_info, cruise_v)) {
      limit_ref_vel_on_ramp_valid = true;
      ego_v = std::max(ego_v, config_.min_v_cruise);
      kMaxAcc = 0.2;
    }
    double s = 0.0;
    double span_t = config_.delta_t * config_.num_step;
    if (ego_v < cruise_v) {
      double t = (cruise_v - ego_v) / kMaxAcc;
      if (t > span_t) {
        s = ego_v * span_t + 0.5 * kMaxAcc * span_t * span_t;
      } else {
        s = ego_v * t + 0.5 * kMaxAcc * t * t;
        s += (span_t - t) * cruise_v;
      }
    } else {
      double t = (cruise_v - ego_v) / kMinAcc;
      if (t > span_t) {
        s = ego_v * span_t + 0.5 * kMinAcc * span_t * span_t;
      } else {
        s = ego_v * t + 0.5 * kMinAcc * t * t;
        s += (span_t - t) * cruise_v;
      }
    }
    const auto &motion_planner_output =
        session_->planning_context().motion_planner_output();
    if (motion_planner_output.lat_enable_flag &&
        motion_planner_output.lat_init_flag) {
      double last_ref_length =
          motion_planner_output.s_lat_vec.back() - motion_planner_output.s_lat_vec[1];
      s = std::min(s, last_ref_length + 1.0);
    }
    const auto &cart_ref_info = coarse_planning_info.cart_ref_info;
    double s_ref = planning_init_point.frenet_state.s;
    const double max_ref_length = std::max(
        std::min(cart_ref_info.s_vec.back(),frenet_coord->Length()) - s_ref - 0.01,
        0.0);
    double avg_cruise_v = std::max(std::min(s, max_ref_length) / span_t, 0.0);
    double delta_s = avg_cruise_v * config_.delta_t;
    traj_points.clear();
    TrajectoryPoint point;
    constexpr double kEps = 1e-4;
    for (size_t i = 0; i < config_.num_step + 1; ++i) {
      // cart info
      if (s_ref < cart_ref_info.s_vec.back() + kEps) {
        point.x = cart_ref_info.x_s_spline(s_ref);
        point.y = cart_ref_info.y_s_spline(s_ref);
        point.heading_angle =
            std::atan2(cart_ref_info.y_s_spline.deriv(1, s_ref),
                       cart_ref_info.x_s_spline.deriv(1, s_ref));
      }

      // frenet info
      Point2D frenet_pt{0.0, 0.0};
      Point2D cart_pt(point.x, point.y);
      frenet_coord->XYToSL(cart_pt, frenet_pt);
      point.s = frenet_pt.x;
      point.l = frenet_pt.y;
      point.t = static_cast<double>(i) * config_.delta_t;

      s_ref += delta_s;
      traj_points.emplace_back(point);
    }
  }
  auto &general_lateral_decider_output =
      session_->mutable_planning_context()
          ->mutable_general_lateral_decider_output();
  if (limit_ref_vel_on_ramp_valid) {
    general_lateral_decider_output.ramp_scene = true;
  } else {
    general_lateral_decider_output.ramp_scene = false;
  }
  if ((is_LC_CHANGE || is_LC_BACK || is_LC_HOLD) &&
      ((config_.not_use_gap_flag) ||
       gap_selector_decider_output.gap_selector_trustworthy)) {
    general_lateral_decider_output.complete_follow = true;
    general_lateral_decider_output.lane_change_scene = true;
    if ((dist_to_second_stage < -1e-6 ||
        !gap_selector_decider_output.gap_selector_trustworthy) && !is_LC_HOLD) {
      HandleAvoidScene(traj_points, dynamic_ref_buffer);
    }
  } else {
    general_lateral_decider_output.complete_follow =
        false;  // fusion is unsteady, lane keep weight need decay in end of
                // ref
    general_lateral_decider_output.lane_change_scene = false;
    HandleAvoidScene(traj_points, 0.0);
  }
}

void GeneralLateralDecider::HandleAvoidScene(TrajectoryPoints &traj_points,
                                             double dynamic_ref_buffer) {
  const auto &frenet_coord =
      reference_path_ptr_->get_frenet_coord();

  const LateralOffsetDeciderOutput &lateral_offset_decider_output =
      session_->mutable_planning_context()->lateral_offset_decider_output();
  if (lateral_offset_decider_output.is_valid ||
      std::fabs(dynamic_ref_buffer) > 1e-6) {
    double lateral_offset =
        lateral_offset_decider_output.lateral_offset + dynamic_ref_buffer;
    Point2D first_offset_xy_point;
    if (frenet_coord->SLToXY(Point2D(traj_points[0].s, lateral_offset),
                             first_offset_xy_point)) {
      double frist_point_x = traj_points[0].x;
      double frist_point_y = traj_points[0].y;

      double diff_x = first_offset_xy_point.x - frist_point_x;
      double diff_y = first_offset_xy_point.y - frist_point_y;

      for (auto &traj_point : traj_points) {
        traj_point.x += diff_x;
        traj_point.y += diff_y;
        traj_point.l += lateral_offset;
      }
    } else {
      std::cout << "HandleAvoidScene frenet error!" << std::endl;
    }
  }
}

bool GeneralLateralDecider::ConstructReferencePathPoints(
    const TrajectoryPoints &traj_points) {
  ref_path_points_.reserve(traj_points.size());
  min_road_radius_ = 10.0;
  const auto &coarse_planning_info =
      session_->planning_context()
              .lane_change_decider_output()
              .coarse_planning_info;
  for (const auto &traj_point : traj_points) {
    ReferencePathPoint refpath_pt{};
    if (!reference_path_ptr_->get_reference_point_by_lon(traj_point.s,
                                                         refpath_pt)) {
      // add logs
      LOG_ERROR("Get reference point by lon failed!");
    }
    double road_radius1 =
        1 / std::max(std::fabs(refpath_pt.path_point.kappa()), 1e-6);
    double road_radius2 = 10.0;
    if (coarse_planning_info.cart_ref_info.k_s_spline.get_x().size() > 0) {
      road_radius2 =
          1 / std::max(std::fabs(coarse_planning_info.cart_ref_info.k_s_spline(traj_point.s)), 1e-6);
    }
    min_road_radius_ = std::max(std::min(std::min(road_radius1, road_radius2) - 1.0, min_road_radius_), 0.2);
    ref_path_points_.emplace_back(refpath_pt);
  }

  ref_traj_points_.resize(traj_points.size());
  std::copy(traj_points.begin(), traj_points.end(), ref_traj_points_.begin());

  const auto &frenet_coord =
      reference_path_ptr_->get_frenet_coord();
  auto &last_traj_points = session_->mutable_planning_context()
                               ->mutable_last_planning_result()
                               .raw_traj_points;

  TrajectoryPoints plan_history_traj_tmp;
  for (size_t i = 0; i < last_traj_points.size(); ++i) {
    // frenet info
    Point2D frenet_pt{0.0, 0.0};
    Point2D cart_pt(last_traj_points[i].x, last_traj_points[i].y);
    if (frenet_coord->XYToSL(cart_pt, frenet_pt)) {
      last_traj_points[i].s = frenet_pt.x;
      last_traj_points[i].l = frenet_pt.y;
      plan_history_traj_tmp.emplace_back(last_traj_points[i]);
    } else {
      LOG_DEBUG("plan_history_traj frenet error");
    }
  }
  if (plan_history_traj_tmp.empty()) {
    return false;
  }
  auto ego_s = ego_frenet_state_.planning_init_point().frenet_state.s;
  // auto ego_s = ego_frenet_state_.s();
  if (ego_s <= plan_history_traj_tmp.front().s) {
    for (double t = 0; t <= plan_history_traj_tmp.back().t;
         t += config_.delta_t) {
      TrajectoryPoint pt =
          general_lateral_decider_utils::GetTrajectoryPointAtTime(
              plan_history_traj_tmp, t);
      pt.s = pt.s - (ego_s - plan_history_traj_tmp.front().s);
      plan_history_traj_.emplace_back(std::move(pt));
    }
  } else if (ego_s >= plan_history_traj_tmp.back().s) {
    // assert(false);
  } else {
    int index = 1;
    while (index < plan_history_traj_tmp.size()) {
      if (plan_history_traj_tmp[index].s >= ego_s) {
        break;
      }
      index++;
    }
    const auto &traj_1 = plan_history_traj_tmp[index - 1];
    const auto &traj_2 = plan_history_traj_tmp[index];

    const double weight0 = (ego_s - traj_1.s) / (traj_2.s - traj_1.s);
    const double weight1 = 1.0 - weight0;
    const double base_t = weight1 * traj_1.t + weight0 * traj_2.t;
    for (double t = base_t; t <= plan_history_traj_tmp.back().t;
         t += config_.delta_t) {
      TrajectoryPoint pt =
          general_lateral_decider_utils::GetTrajectoryPointAtTime(
              plan_history_traj_tmp, t);
      plan_history_traj_.emplace_back(std::move(pt));
    }

    for (auto &traj : plan_history_traj_) {
      traj.t -= base_t;
    }

    if (plan_history_traj_.size() == 0) {
    } else {
      for (int point_num = plan_history_traj_.size();
           point_num < config_.num_step + 1; point_num++) {
        TrajectoryPoint pt = plan_history_traj_.back();
        // For now, only s and t are modified
        pt.s += pt.v * config_.delta_t;
        pt.t += config_.delta_t;
        plan_history_traj_.emplace_back(std::move(pt));
      }
    }
  }

  for (int i = 0; i < plan_history_traj_.size(); i++) {
    double plan_history_traj_point_s = plan_history_traj_[i].s;
    std::vector<int> match_indexes =
        general_lateral_decider_utils::MatchRefTrajPoints(
            plan_history_traj_point_s, ref_traj_points_);
    match_index_map_[i] = std::move(match_indexes);
  }
  return true;
}

void GeneralLateralDecider::UpdateDistanceToRoadBorder() {
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  for (size_t i = 0; i < ref_traj_points_.size(); i++) {
    SampleRoadDistanceInfo(ref_traj_points_[i].s, ref_path_points_[i]);
    // distance to lane is unaccurate near end of the ref line
    // need to avoid the intersection of lane bound
    size_t lower_truncation_idx = 0;
    size_t upper_truncation_idx = 0;
    if (ref_path_points_[i].distance_to_left_lane_border >
        0.5 * vehicle_param.max_width + config_.soft_buffer2lane) {
      upper_truncation_idx = i;
    } else {
      ref_path_points_[i].distance_to_left_lane_border =
          ref_path_points_[upper_truncation_idx].distance_to_left_lane_border;
    }

    if (ref_path_points_[i].distance_to_right_lane_border >
        0.5 * vehicle_param.max_width + config_.soft_buffer2lane) {
      lower_truncation_idx = i;
    } else {
      ref_path_points_[i].distance_to_right_lane_border =
          ref_path_points_[lower_truncation_idx].distance_to_right_lane_border;
    }
  }
}
void GeneralLateralDecider::GenerateRoadAndLaneBoundary() {
  UpdateDistanceToRoadBorder();
  GenerateRoadHardSoftBoundary();
  GenerateLaneSoftBoundary();
}

void GeneralLateralDecider::GenerateRoadHardSoftBoundary() {
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double left_road_extra_buffer, right_road_extra_buffer;
  GetDesireRoadExtraBuffer(&left_road_extra_buffer, &right_road_extra_buffer);

  const double kDefaultDistanceToRoad = 10.0;
  min_road_radius_ = std::min(kDefaultDistanceToRoad, min_road_radius_);
  hard_bounds_.resize(ref_traj_points_.size());
  soft_bounds_.resize(ref_traj_points_.size());
  for (size_t i = 0; i < ref_traj_points_.size(); i++) {
    Bound soft_bound_road{-min_road_radius_, min_road_radius_};
    Bound hard_bound_road{-min_road_radius_, min_road_radius_};
    // if (ref_path_points_[i].path_point.kappa() > 0.0) {
    //   soft_bound_road.upper = min_road_radius;
    //   hard_bound_road.upper = min_road_radius;
    // } else {
    //   soft_bound_road.lower = -min_road_radius;
    //   hard_bound_road.lower = -min_road_radius;
    // }

    MapObstaclePositionDecision map_obstacle_decision;

    map_obstacle_decision.tp.t = ref_traj_points_[i].t;
    map_obstacle_decision.tp.s = ref_traj_points_[i].s;
    map_obstacle_decision.tp.l = ref_traj_points_[i].l;
    if (map_obstacle_decision.tp.s -
            ego_frenet_state_.planning_init_point().frenet_state.s <
        config_.care_lon_area_road_border) {
      hard_bound_road.upper =
          std::fmin(std::max(config_.hard_min_distance_road2center,
                             ref_path_points_[i].distance_to_left_road_border -
                                 0.5 * vehicle_param.max_width -
                                 config_.hard_buffer2road),
                    hard_bound_road.upper);
      hard_bound_road.lower = std::fmax(
          std::min(-config_.hard_min_distance_road2center,
                   -ref_path_points_[i].distance_to_right_road_border +
                       0.5 * vehicle_param.max_width +
                       config_.hard_buffer2road),
          hard_bound_road.lower);
      soft_bound_road.upper =
          std::fmin(std::max(config_.soft_min_distance_road2center,
                             hard_bound_road.upper - left_road_extra_buffer),
                    soft_bound_road.upper);
      soft_bound_road.lower =
          std::fmax(std::min(-config_.soft_min_distance_road2center,
                             hard_bound_road.lower + right_road_extra_buffer),
                    soft_bound_road.lower);
    }

    hard_bounds_[i].emplace_back(WeightedBound{
        hard_bound_road.lower, hard_bound_road.upper, config_.kHardBoundWeight,
        BoundInfo{-100, BoundType::ROAD_BORDER}});
    soft_bounds_[i].emplace_back(WeightedBound{
        soft_bound_road.lower, soft_bound_road.upper,
        config_.kPhysicalBoundWeight, BoundInfo{-100, BoundType::ROAD_BORDER}});
  }
}

void GeneralLateralDecider::GenerateLaneSoftBoundary() {
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  const auto &lc_request_direction =
      session_->planning_context().lane_change_decider_output().lc_request;
  bool is_LC_CHANGE =
      ((coarse_planning_info.target_state == kLaneChangeExecution) ||
       (coarse_planning_info.target_state == kLaneChangeComplete));
  bool is_LC_BACK = coarse_planning_info.target_state == kLaneChangeCancel;
  bool is_lane_change = is_LC_CHANGE || is_LC_BACK;
  const double kDefaultDistanceToRoad = 10.0;
  const double half_ego_width = 0.5 * vehicle_param.max_width;
  for (size_t i = 0; i < ref_traj_points_.size(); i++) {
    Bound soft_bound_lane{-kDefaultDistanceToRoad, kDefaultDistanceToRoad};
    soft_bound_lane.upper =
        std::fmin(ref_path_points_[i].distance_to_left_lane_border -
                      half_ego_width - config_.soft_buffer2lane,
                  soft_bound_lane.upper);
    soft_bound_lane.lower =
        std::fmax(-ref_path_points_[i].distance_to_right_lane_border +
                      half_ego_width + config_.soft_buffer2lane,
                  soft_bound_lane.lower);
    const auto &ego_init_sl_info = ego_frenet_state_.ego_init_sl_info();
    if (is_lane_change && lc_request_direction == RequestType::LEFT_CHANGE) {
      if (ego_init_sl_info.min_l + half_ego_width < soft_bound_lane.lower) {
        soft_bounds_[i].emplace_back(
            WeightedBound{ego_init_sl_info.min_l + half_ego_width,
                          kDefaultDistanceToRoad, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::EGO_POSITION}});
        soft_bounds_[i].emplace_back(WeightedBound{
            -kDefaultDistanceToRoad, soft_bound_lane.upper,
            config_.kPhysicalBoundWeight, BoundInfo{-100, BoundType::LANE}});
      } else {
        soft_bounds_[i].emplace_back(WeightedBound{
            soft_bound_lane.lower, soft_bound_lane.upper,
            config_.kPhysicalBoundWeight, BoundInfo{-100, BoundType::LANE}});
      }
    } else if (is_lane_change &&
               lc_request_direction == RequestType::RIGHT_CHANGE) {
      if (ego_init_sl_info.max_l - half_ego_width > soft_bound_lane.upper) {
        soft_bounds_[i].emplace_back(WeightedBound{
            -kDefaultDistanceToRoad, ego_init_sl_info.max_l - half_ego_width,
            config_.kPhysicalBoundWeight,
            BoundInfo{-100, BoundType::EGO_POSITION}});
        soft_bounds_[i].emplace_back(WeightedBound{
            soft_bound_lane.lower, kDefaultDistanceToRoad,
            config_.kPhysicalBoundWeight, BoundInfo{-100, BoundType::LANE}});
      } else {
        soft_bounds_[i].emplace_back(WeightedBound{
            soft_bound_lane.lower, soft_bound_lane.upper,
            config_.kPhysicalBoundWeight, BoundInfo{-100, BoundType::LANE}});
      }
    } else {
      soft_bounds_[i].emplace_back(WeightedBound{
          soft_bound_lane.lower, soft_bound_lane.upper,
          config_.kPhysicalBoundWeight, BoundInfo{-100, BoundType::LANE}});
    }
  }
}

void GeneralLateralDecider::GetDesireRoadExtraBuffer(
    double *const left_road_extra_buffer,
    double *const right_road_extra_buffer) {
  const double kMinExtraBuffer = config_.extra_soft_buffer2road;
  const auto &planning_init_point =
      ego_cart_state_manager_->planning_init_point();
  const double ego_v = planning_init_point.v;
  double max_collision_t, left_collision_t, right_collision_t;
  GetLateralTTCToRoad(&max_collision_t, &left_collision_t, &right_collision_t);

  *left_road_extra_buffer =
      interp(left_collision_t, config_.lateral_road_boader_collision_ttc_bp,
             config_.extra_collision_lateral_buffer);
  *right_road_extra_buffer =
      interp(right_collision_t, config_.lateral_road_boader_collision_ttc_bp,
             config_.extra_collision_lateral_buffer);
  // *left_road_extra_buffer =
  //     std::min(0.2, (max_collision_t - left_collision_t) * 0.1);
  // *right_road_extra_buffer =
  //     std::min(0.2, (max_collision_t - right_collision_t) * 0.1);
  // 36     60         80          100          120           130     kph
  // 0.3    0.43       0.53        0.632        0.725         0.77    m
  // double extra_buffer =
  double extra_buffer = interp(ego_v * 3.6, config_.lateral_road_boader_v_bp,
                               config_.extra_lateral_buffer);
  extra_buffer = std::max(extra_buffer, kMinExtraBuffer);
  *left_road_extra_buffer += extra_buffer;
  *right_road_extra_buffer += extra_buffer;
}

void GeneralLateralDecider::GetLateralTTCToRoad(
    double *max_collision_t, double *const left_collision_t,
    double *const right_collision_t) {
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto &frenet_coord =
      session_->planning_context()
          .lane_change_decider_output()
          .coarse_planning_info.reference_path->get_frenet_coord();
  const auto &planning_init_point =
      ego_cart_state_manager_->planning_init_point();
  const double ego_v = planning_init_point.v;
  const double ego_theta = planning_init_point.heading_angle;

  const double ego_vx = ego_v * std::cos(ego_theta);
  const double ego_vy = ego_v * std::sin(ego_theta);

  const double time_diff = 0.1;

  bool is_left_overlap = false;
  bool is_right_overlap = false;
  *left_collision_t = config_.max_lateral_ttc;
  *right_collision_t = config_.max_lateral_ttc;
  *max_collision_t = config_.max_lateral_ttc;
  for (double t = 0.0; t < config_.max_lateral_ttc; t += time_diff) {
    Point2D prediction_xy(planning_init_point.x + ego_vx * t,
                          planning_init_point.y + ego_vy * t);
    Point2D prediction_sl;
    if (!frenet_coord->XYToSL(prediction_xy, prediction_sl)) {
      continue;
    }
    if (prediction_sl.x -
            ego_frenet_state_.planning_init_point().frenet_state.s >
        config_.care_lon_area_road_border) {
      *max_collision_t = t;
      break;
    }
    const double ego_s_start =
        prediction_sl.x - vehicle_param.rear_edge_to_rear_axle;
    const double ego_s_end = prediction_sl.x + vehicle_param.length -
                             vehicle_param.rear_edge_to_rear_axle;
    const auto ego_center_point =
        Vec2d((ego_s_start + ego_s_end) * 0.5, prediction_sl.y);

    const double heading_angle = planning_math::NormalizeAngle(
        ego_theta - frenet_coord->GetPathCurveHeading(ego_center_point.x()));
    const auto ego_box = Box2d(ego_center_point, heading_angle,
                               vehicle_param.length, vehicle_param.max_width);

    for (size_t i = 1; i < ref_path_points_.size(); i++) {
      if (!is_left_overlap) {
        const auto left_road_line_segment = LineSegment2d(
            Vec2d(ref_path_points_.at(i - 1).path_point.s(),
                  ref_path_points_.at(i - 1).distance_to_left_road_border),
            Vec2d(ref_path_points_.at(i).path_point.s(),
                  ref_path_points_.at(i).distance_to_left_road_border));
        if (ego_box.HasOverlap(left_road_line_segment)) {
          is_left_overlap = true;
          *left_collision_t = t;
        }
      }

      if (!is_right_overlap) {
        const auto right_road_line_segment = LineSegment2d(
            Vec2d(ref_path_points_.at(i - 1).path_point.s(),
                  -ref_path_points_.at(i - 1).distance_to_right_road_border),
            Vec2d(ref_path_points_.at(i).path_point.s(),
                  -ref_path_points_.at(i).distance_to_right_road_border));
        if (ego_box.HasOverlap(right_road_line_segment)) {
          is_right_overlap = true;
          *right_collision_t = t;
        }
      }

      // if (is_left_overlap && is_right_overlap) {
      //   break;
      // }
    }
  }

  if (*left_collision_t > *max_collision_t) {
    *left_collision_t = *max_collision_t;
  }
  if (*right_collision_t > *max_collision_t) {
    *right_collision_t = *max_collision_t;
  }
}

void GeneralLateralDecider::GenerateObstaclesBoundary() {
  const auto &general_lateral_decider_output =
      session_->mutable_planning_context()
          ->mutable_general_lateral_decider_output();
  const LateralOffsetDeciderOutput &lateral_offset_decider_output =
      session_->mutable_planning_context()->lateral_offset_decider_output();
  CalculateExtraLaneWidthDecreaseBuffer();

  if (general_lateral_decider_output.lane_change_scene) {
    last_lat_obstacle_decision_.clear();
    LOG_DEBUG("LatObstacle Decider! GS trustworthy");
    return;
  }

  if (!lateral_offset_decider_output.enable_bound) {
    last_lat_obstacle_decision_.clear();
    LOG_DEBUG("Enable_bound is invalid!");
    return;
  }
  if (ref_path_points_.empty()) {
    LOG_DEBUG("lat ref path points is empty! \n");
  }
  if (plan_history_traj_.empty()) {
    LOG_DEBUG("plan history traj is empty! \n");
  }

  if (plan_history_traj_.empty() || ref_path_points_.empty()) {
    last_lat_obstacle_decision_.clear();
    // add logs
    LOG_ERROR("Ref traj points or ref path points is null!");
    return;
  }
  const auto &obs_vec = reference_path_ptr_->get_obstacles();
  std::vector<std::shared_ptr<FrenetObstacle>> static_obstacles;
  std::vector<std::shared_ptr<FrenetObstacle>> dynamic_obstacles;
  for (const auto &obs : obs_vec) {
    if (obs->obstacle()->is_static()) {
      static_obstacles.emplace_back(obs);
    } else {
      dynamic_obstacles.emplace_back(obs);
    }
  }

  GenerateStaticObstaclesBoundary(static_obstacles, static_obstacle_decisions_);
  GenerateDynamicObstaclesBoundary(dynamic_obstacles,
                                   dynamic_obstacle_decisions_);
}

void GeneralLateralDecider::GenerateStaticObstaclesBoundary(
    const std::vector<std::shared_ptr<FrenetObstacle>> obs_vec,
    ObstacleDecisions &obstacle_decisions) {
  ObstaclePotentialDecisions obstacle_potential_decisions;
  for (auto &obstacle : obs_vec) {
    if (!IsFilterForStaticObstacle(obstacle)) {
      continue;
    }

    const auto &obstacle_id = obstacle->id();
    auto obstacle_decision = ObstacleDecision{obstacle_id, {}, {}};
    auto obstacle_potential_decision =
        ObstaclePotentialDecision{obstacle_id, {}};

    GenerateStaticObstacleDecision(obstacle, obstacle_decision, true);
    GenerateStaticObstacleDecision(obstacle, obstacle_decision, false);

    ExtractStaticObstacleBound(obstacle_decision);
    obstacle_decisions[obstacle_id] = std::move(obstacle_decision);
    if (obstacle_potential_decision.extend_decisions.extended) {
      obstacle_potential_decisions[obstacle_id] =
          std::move(obstacle_potential_decision);
    }
  }
}

void GeneralLateralDecider::GenerateStaticObstacleDecision(
    const std::shared_ptr<FrenetObstacle> obstacle,
    ObstacleDecision &obstacle_decision, bool is_update_hard_bound) {
  using namespace planning_math;
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto &lat_obstacle_decision = session_->planning_context()
                                          .lateral_obstacle_decider_output()
                                          .lat_obstacle_decision;
  const auto &lat_obstacle_position = session_->mutable_planning_context()
                                    ->mutable_lateral_obstacle_decider_output()
                                    .lateral_obstacle_history_info;
  const bool in_intersection = session_->environmental_model()
                                   .get_virtual_lane_manager()
                                   ->GetIntersectionState() ==
                               common::IntersectionState::IN_INTERSECTION;
  const auto &lane_borrow_decider_output =
      session_->planning_context().lane_borrow_decider_output();
  const bool is_in_lane_borrow_status =
      lane_borrow_decider_output.is_in_lane_borrow_status;
  bool is_care_rear_obstacle = IsRearObstacle(obstacle) &&
      IsAgentPredLonOverlapWithPlanPath(obstacle);

  // Step 1) configs
  const auto &l_care_width = config_.l_care_width;

  const auto &ego_cur_s = ref_traj_points_.front().s;

  const double rear_axle_to_front_bumper =  // TBD：define as consexpr
      vehicle_param.length - vehicle_param.rear_edge_to_rear_axle;
  const double ego_cur_s_start =
      ego_cur_s - vehicle_param.rear_edge_to_rear_axle;
  const double ego_cur_s_end = ego_cur_s + rear_axle_to_front_bumper;

  double front_lon_buf_dis = 1.0;
  double rear_lon_buf_dis = 1.0;
  if (!is_update_hard_bound) {
    front_lon_buf_dis = general_lateral_decider_utils::CalDesireLonDistance(
        ego_frenet_state_.velocity_s(), obstacle->frenet_velocity_s());
  }

  if ((is_in_lane_borrow_status) && (is_blocked_obstacle_)) {
    front_lon_buf_dis += config_.extra_front_lon_buffer2blockobstacle;
    rear_lon_buf_dis += config_.extra_rear_lon_buffer2blockobstacle;
  }

  auto pre_lateral_decision = LatObstacleDecisionType::IGNORE;

  const bool init_lon_no_overlap =
      (obstacle->frenet_obstacle_boundary().s_end < ego_cur_s_start ||
       obstacle->frenet_obstacle_boundary().s_start > ego_cur_s_end);

  bool reset_conflict_decision{false};

  bool is_nudge_left = lat_obstacle_decision.at(obstacle->id()) ==
                       LatObstacleDecisionType::RIGHT;
  bool is_side_obstacle = false;
  const auto lat_obs_position_iter = lat_obstacle_position.find(obstacle->id());
  if (lat_obs_position_iter != lat_obstacle_position.end()) {
    if (lat_obs_position_iter->second.side_car) {
      last_lat_obstacle_decision_[obstacle->id()] = lat_obstacle_decision.at(obstacle->id());
    } else {
      if (last_lat_obstacle_decision_.find(obstacle->id()) !=
          last_lat_obstacle_decision_.end()) {
        last_lat_obstacle_decision_.erase(obstacle->id());
      }
    }
    is_side_obstacle = (lat_obs_position_iter->second.side_car) &&
                       (!lat_obs_position_iter->second.front_car);
  }

  const auto borrow_direction = lane_borrow_decider_output.borrow_direction;
  if ((is_in_lane_borrow_status) && (is_blocked_obstacle_)) {
    if (borrow_direction == LEFT_BORROW) {
      // 向左借道
      is_nudge_left = false;
    } else if (borrow_direction == RIGHT_BORROW) {
      // 向右借道
      is_nudge_left = true;
    }
    is_care_rear_obstacle = false;
  }

  BoundType bound_type = BoundType::AGENT;
  if (is_care_rear_obstacle) {
    BoundType bound_type = BoundType::REAR_AGENT;
  }

  bool is_cross_obj{false};
  bool has_lat_decision{false};
  bool has_lon_decision{false};
  // Step 5) calculate soft_bound, hard_bound
  Polygon2d obstacle_sl_polygon;
  bool ok = false;
  if (config_.use_obstacle_prediction_model_in_planning) {
    ok = obstacle->get_polygon_at_time(0, reference_path_ptr_,
                                       obstacle_sl_polygon);
  } else {
    ok = obstacle->get_polygon_at_time_tmp(0, reference_path_ptr_,
                                           obstacle_sl_polygon);
  }
  if (!ok) {
    // TBD add log
    return;
  }
  double extra_lane_type_decrease_buffer =
      CalculateExtraLaneTypeDecreaseBuffer(is_nudge_left, obstacle->frenet_obstacle_boundary().s_start,
                                          obstacle->frenet_obstacle_boundary().s_end);

  for (size_t i = 0; i < ref_traj_points_.size(); i++) {
    auto &traj_point = ref_traj_points_[i];
    const auto &t = traj_point.t;
    if (t > config_.care_static_object_t_threshold) {
      continue;
    }

    const double ego_s = traj_point.s;
    const double ego_l = traj_point.l;

    const double care_area_s_start =
        ego_s - vehicle_param.rear_edge_to_rear_axle - rear_lon_buf_dis;
    const double care_area_s_end =
        ego_s + rear_axle_to_front_bumper + front_lon_buf_dis;
    const auto care_area_center =
        Vec2d((care_area_s_start + care_area_s_end) * 0.5, ego_l);
    const double care_area_length = care_area_s_end - care_area_s_start;
    const auto care_polygon =  // @cai: consider the heading
        Polygon2d(Box2d(care_area_center, 0, care_area_length, l_care_width));

    Polygon2d care_overlap_polygon;
    bool b_overlap_with_care = false;
    // default: invalid value
    double overlap_min_y = 100.0;
    double overlap_max_y = -100.0;

    b_overlap_with_care =
        obstacle_sl_polygon.ComputeOverlap(care_polygon, &care_overlap_polygon);
    if (b_overlap_with_care) {
      // TBD: add log
      overlap_min_y = care_overlap_polygon.min_y();
      overlap_max_y = care_overlap_polygon.max_y();
    } else {
      continue;
    }

    double lat_buf_dis =
        general_lateral_decider_utils::CalDesireStaticLateralDistance(
            config_.hard_buffer2static_agent, ego_cart_state_manager_->ego_v(),
            ego_frenet_state_.l(), obstacle->type(), is_update_hard_bound,
            config_);

    if ((is_in_lane_borrow_status) && (is_blocked_obstacle_)) {
      lat_buf_dis += config_.extra_hard_buffer2blockobstacle;
    }

    if (!is_in_lane_borrow_status) {
      lat_buf_dis = std::fmax(lat_buf_dis - extra_lane_width_decrease_buffer_ -
                                  extra_lane_type_decrease_buffer,
                              0.);
      if (is_side_obstacle && !session_->mutable_planning_context()
                                    ->mutable_lateral_obstacle_decider_output()
                                    .in_intersection) {
        lat_buf_dis = std::fmin(lat_buf_dis, config_.side_obstacle_lat_buffer_limit);
      }
    }

    auto lat_decision = LatObstacleDecisionType::IGNORE;
    auto lon_decision = LonObstacleDecisionType::IGNORE;

    std::pair<double, double> s_side_range{obstacle_sl_polygon.min_x(),
                                           obstacle_sl_polygon.max_x()};
    std::pair<double, double> ego_side_range{
        ego_s - vehicle_param.rear_edge_to_rear_axle,
        ego_s + rear_axle_to_front_bumper + 2.};
    bool b_overlap_side = std::max(s_side_range.first, ego_side_range.first) <
                          std::min(s_side_range.second, ego_side_range.second);

    GenerateObstaclePreliminaryDecision(
        ego_l, ref_path_points_[i].distance_to_right_lane_border,
        ref_path_points_[i].distance_to_left_lane_border, overlap_min_y,
        overlap_max_y, lat_buf_dis, b_overlap_side, init_lon_no_overlap,
        is_nudge_left, is_cross_obj, pre_lateral_decision,
        reset_conflict_decision, obstacle_decision, lat_decision, lon_decision);
    has_lat_decision =
        has_lat_decision || lat_decision != LatObstacleDecisionType::IGNORE;
    has_lon_decision =
        has_lon_decision || lon_decision != LonObstacleDecisionType::IGNORE;

    AddObstacleDecisionBound(obstacle->id(), t, bound_type, overlap_min_y,
                             overlap_max_y, lat_buf_dis, lat_decision,
                             lon_decision, obstacle_decision,
                             is_update_hard_bound);
  }
}

bool GeneralLateralDecider::IsCutoutSideObstacle(
    const std::shared_ptr<FrenetObstacle> obstacle, double &limit_overlap_min_y,
    double &limit_overlap_max_y) {
  if (plan_history_traj_.empty()) {
    return false;
  }
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double ego_s = reference_path_ptr_->get_frenet_ego_state().s();
  if (reference_path_ptr_->get_ego_frenet_boundary().s_start <=
          obstacle->frenet_obstacle_boundary().s_end &&
      reference_path_ptr_->get_ego_frenet_boundary().s_end >=
          obstacle->frenet_obstacle_boundary().s_start) {
    const double ego_l = plan_history_traj_[0].l;
    const auto &l_care_width = config_.l_care_width;
    const double care_area_s_start =
        ego_s - vehicle_param.rear_edge_to_rear_axle;
    const double care_area_s_end = ego_s + 10;
    const auto care_area_center =
        Vec2d((care_area_s_start + care_area_s_end) * 0.5, ego_l);
    const double care_area_length = care_area_s_end - care_area_s_start;
    const auto care_polygon =
        Polygon2d(Box2d(care_area_center, 0, care_area_length, l_care_width));
    Polygon2d obstacle_sl_polygon;
    bool ok = false;
    if (config_.use_obstacle_prediction_model_in_planning) {
      ok = obstacle->get_polygon_at_time(0, reference_path_ptr_,
                                         obstacle_sl_polygon);
    } else {
      ok = obstacle->get_polygon_at_time_tmp(0, reference_path_ptr_,
                                             obstacle_sl_polygon);
    }
    if (!ok) {
      // TBD add log
      return false;
    }
    Polygon2d care_overlap_polygon;

    if ((obstacle->frenet_l() * obstacle->frenet_velocity_l() > 0) &&
        fabs(obstacle->frenet_velocity_l()) > 0.3) {
      if (obstacle_sl_polygon.ComputeOverlap(care_polygon,
                                             &care_overlap_polygon)) {
        limit_overlap_min_y = care_overlap_polygon.min_y();
        limit_overlap_max_y = care_overlap_polygon.max_y();
        return true;
      } else {
        return false;
      }
    }
  }
  return false;
}

double GeneralLateralDecider::CalculateExtraDecreaseBuffer(
    const std::shared_ptr<FrenetObstacle> obstacle, bool is_nudge_left) {
  bool in_intersection = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->GetIntersectionState() ==
                         common::IntersectionState::IN_INTERSECTION;
  if (in_intersection) {
    return 0.0;
  }

  const double relative_position =
      obstacle->frenet_obstacle_boundary().s_end -
      reference_path_ptr_->get_ego_frenet_boundary().s_start;
  double extra_relative_position_decrease_buffer = 0.0;
  if ((is_nudge_left && ego_frenet_state_.heading_angle() < 0.03) ||
      (!is_nudge_left && ego_frenet_state_.heading_angle() > -0.03)) {
    extra_relative_position_decrease_buffer =
        interp(relative_position, config_._relative_positon_bp,
               config_._relative_positon_decrease_extra_buffer);
  }

  double extra_relative_v_decrease_buffer = 0.0;
  if (relative_position <= 2.5 &&
      ((is_nudge_left && ego_frenet_state_.heading_angle() < 0.03) ||
       (!is_nudge_left && ego_frenet_state_.heading_angle() > -0.03))) {
    extra_relative_v_decrease_buffer = interp(
        obstacle->frenet_velocity_s() - ego_frenet_state_.velocity_s(),
        config_._relative_v_bp, config_._relative_v_decrease_extra_buffer);
  }

  double extra_type_decrease_buffer = 0.0;
  if (general_lateral_decider_utils::IsTruck(obstacle->type())) {
    extra_type_decrease_buffer = config_.truck_decrease_extra_buffer;
  }

  if (extra_relative_position_decrease_buffer > 0.0) {
    JSON_DEBUG_VALUE("extra_relative_position_decrease_buffer",
                     extra_relative_position_decrease_buffer);
  }
  if (extra_relative_v_decrease_buffer > 0.0) {
    JSON_DEBUG_VALUE("extra_relative_v_decrease_buffer",
                     extra_relative_v_decrease_buffer);
  }

  return extra_relative_position_decrease_buffer +
         extra_relative_v_decrease_buffer + extra_type_decrease_buffer;
}

double GeneralLateralDecider::CalculateExtraLaneTypeDecreaseBuffer(
    bool is_nudge_left, const double start_s, const double end_s) {
  double lane_type_decrease_buffer = 0.0;
  if (is_nudge_left) {
    // 获取自车当前位置右车道线型
    const auto current_ego_right_boundary_type =
        CalLaneBoundaryType(RIGHT, ego_frenet_state_.s());

    // 获取start_s位置右车道线型
    const auto care_start_s_right_boundary_type =
        CalLaneBoundaryType(RIGHT, start_s);

    // 获取end_s位置右车道线型
    const auto care_end_s_right_boundary_type =
        CalLaneBoundaryType(RIGHT, end_s);

    if (current_ego_right_boundary_type ==
            iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_SOLID ||
        care_start_s_right_boundary_type ==
            iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_SOLID ||
        care_end_s_right_boundary_type ==
            iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_SOLID) {
      lane_type_decrease_buffer = config_.extra_lane_type_decrease_buffer;
    }
  } else {
    // 获取自车当前位置左车道线型
    const auto current_ego_left_boundary_type =
        CalLaneBoundaryType(LEFT, ego_frenet_state_.s());

    // 获取start_s位置左车道线型
    const auto care_start_s_left_boundary_type =
        CalLaneBoundaryType(LEFT, start_s);

    // 获取end_s位置左车道线型
    const auto care_end_s_left_boundary_type = CalLaneBoundaryType(LEFT, end_s);

    if (current_ego_left_boundary_type ==
            iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_SOLID ||
        care_start_s_left_boundary_type ==
            iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_SOLID ||
        care_end_s_left_boundary_type ==
            iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_SOLID) {
      lane_type_decrease_buffer = config_.extra_lane_type_decrease_buffer;
    }
  }

  if (lane_type_decrease_buffer > 0.0) {
    JSON_DEBUG_VALUE("lane_type_decrease_buffer", lane_type_decrease_buffer);
  }

  return lane_type_decrease_buffer;
}

double
GeneralLateralDecider::CalculateSideObstacleExtraDecreaseBufferInIntersection(
    const std::shared_ptr<FrenetObstacle> obstacle, bool is_nudge_left,
    bool in_intersection) {
  if (!in_intersection) {
    return 0;
  }
  const double relative_position =
      obstacle->frenet_obstacle_boundary().s_end -
      reference_path_ptr_->get_ego_frenet_boundary().s_end;
  double extra_relative_position_decrease_buffer = 0.0;
  if ((is_nudge_left && ego_frenet_state_.heading_angle() < 0.03) ||
      (!is_nudge_left && ego_frenet_state_.heading_angle() > -0.03)) {
    extra_relative_position_decrease_buffer =
        interp(relative_position, config_._side_obstacle_relative_position_bp,
               config_._side_obstacle_relative_position_decrease_extra_buffer);
  }
  return extra_relative_position_decrease_buffer;
}

void GeneralLateralDecider::CalculateExtraLaneWidthDecreaseBuffer() {
  const auto current_left_boundary_type =
      CalLaneBoundaryType(LEFT, ego_frenet_state_.s());

  const auto current_right_boundary_type =
      CalLaneBoundaryType(RIGHT, ego_frenet_state_.s());

  double extra_lane_width_decrease_buffer = 0.0;

  if (current_left_boundary_type !=
          iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_VIRTUAL &&
      current_right_boundary_type !=
          iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_VIRTUAL) {
    double lane_mean_width = CalLaneWidth();
    extra_lane_width_decrease_buffer =
        interp(lane_mean_width, config_.extra_buffer_for_lane_width_bp,
               config_.extra_lane_width_buffer);
  } else if (current_left_boundary_type ==
                 iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_VIRTUAL &&
             current_right_boundary_type ==
                 iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_VIRTUAL) {
    extra_lane_width_decrease_buffer = 0.0;
  } else {
    extra_lane_width_decrease_buffer = extra_lane_width_decrease_buffer_;
  }
  extra_lane_width_decrease_buffer_ = extra_lane_width_decrease_buffer;
}

double GeneralLateralDecider::CalLaneWidth() {
  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  const std::shared_ptr<VirtualLane> flane =
      session_->environmental_model()
          .get_virtual_lane_manager()
          ->get_lane_with_virtual_id(coarse_planning_info.target_lane_id);

  const double preview_s = 40 + ego_frenet_state_.s();
  const double interval_s = 5;
  const double start_s = ego_frenet_state_.s();
  double width = flane->width_by_s(ego_frenet_state_.s());
  int point_num = 1;

  for (double s = start_s + interval_s; s <= preview_s; s += interval_s) {
    auto left_boundary_type = CalLaneBoundaryType(LEFT, s);

    auto right_boundary_type = CalLaneBoundaryType(RIGHT, s);

    if (left_boundary_type !=
            iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_VIRTUAL &&
        right_boundary_type !=
            iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_VIRTUAL) {
      width += flane->width_by_s(s);
      point_num += 1;
    } else {
      break;
    }
  }
  return width / point_num;
}

iflyauto::LaneBoundaryType GeneralLateralDecider::CalLaneBoundaryType(
    const LineDirection direction, const double s) const {
  ReferencePathPoint refpath_pt{};
  if (reference_path_ptr_->get_reference_point_by_lon(s, refpath_pt)) {
    if (direction == LEFT) {
      return refpath_pt.left_lane_border_type;
    } else if (direction == RIGHT) {
      return refpath_pt.right_lane_border_type;
    }
  }
  return iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_VIRTUAL;
}

void GeneralLateralDecider::GenerateDynamicObstaclesBoundary(
    const std::vector<std::shared_ptr<FrenetObstacle>> obs_vec,
    ObstacleDecisions &obstacle_decisions) {
  ObstaclePotentialDecisions obstacle_potential_decisions;
  for (auto &obstacle : obs_vec) {
    if (!IsFilterForDynamicObstacle(obstacle)) {
      continue;
    }

    const auto &obstacle_id = obstacle->id();
    auto obstacle_decision = ObstacleDecision{obstacle_id, {}, {}};
    auto obstacle_potential_decision =
        ObstaclePotentialDecision{obstacle_id, {}};

    GenerateDynamicObstacleDecision(obstacle, obstacle_decision);
    ExtractDynamicObstacleBound(obstacle_decision);
    obstacle_decisions[obstacle_id] = std::move(obstacle_decision);
    if (obstacle_potential_decision.extend_decisions.extended) {
      obstacle_potential_decisions[obstacle_id] =
          std::move(obstacle_potential_decision);
    }
  }
}

void GeneralLateralDecider::GenerateDynamicObstacleDecision(
    const std::shared_ptr<FrenetObstacle> obstacle,
    ObstacleDecision &obstacle_decision) {
  using namespace planning_math;
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto &lat_obstacle_decision = session_->planning_context()
                                          .lateral_obstacle_decider_output()
                                          .lat_obstacle_decision;
  const auto &lat_obstacle_position = session_->mutable_planning_context()
                                    ->mutable_lateral_obstacle_decider_output()
                                      .lateral_obstacle_history_info;
  bool in_intersection = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->GetIntersectionState() ==
                         common::IntersectionState::IN_INTERSECTION;
  const auto &lane_borrow_decider_output =
      session_->planning_context().lane_borrow_decider_output();
  const bool is_in_lane_borrow_status =
      lane_borrow_decider_output.is_in_lane_borrow_status;

  // Step 1) configs
  const auto &l_care_width = config_.l_care_width;

  const auto &ego_cur_s = plan_history_traj_.front().s;

  const double rear_axle_to_front_bumper =  // TBD：define as consexpr
      vehicle_param.length - vehicle_param.rear_edge_to_rear_axle;
  const double ego_cur_s_start =
      ego_cur_s - vehicle_param.rear_edge_to_rear_axle;
  const double ego_cur_s_end = ego_cur_s + rear_axle_to_front_bumper;

  double front_lon_buf_dis =
      general_lateral_decider_utils::CalDesireLonDistance(
          ego_frenet_state_.velocity_s(), obstacle->frenet_velocity_s());
  double rear_lon_buf_dis = 1.0;

  if ((is_in_lane_borrow_status) && (is_blocked_obstacle_)) {
    front_lon_buf_dis += config_.extra_front_lon_buffer2blockobstacle;
    rear_lon_buf_dis += config_.extra_rear_lon_buffer2blockobstacle;
  }

  auto pre_lateral_decision = LatObstacleDecisionType::IGNORE;

  const bool init_lon_no_overlap =
      (obstacle->frenet_obstacle_boundary().s_end < ego_cur_s_start ||
       obstacle->frenet_obstacle_boundary().s_start > ego_cur_s_end);
  // const bool init_lat_overlap = !(obstacle->frenet_obstacle_boundary().l_end
  // <
  //                                     ego_cur_l - half_ego_width ||
  //                                 obstacle->frenet_obstacle_boundary().l_start
  //                                 >
  //                                     ego_cur_l + half_ego_width);

  // double dynamic_bound_gain_vel = std::max(config_.min_gain_vel,
  // ego_velocity); double dynamic_bound_slack_coefficient =
  //     1.0 / dynamic_bound_gain_vel / dynamic_bound_gain_vel;
  bool reset_conflict_decision{false};
  bool is_care_rear_obstacle = IsRearObstacle(obstacle) &&
      IsAgentPredLonOverlapWithPlanPath(obstacle);
  bool is_nudge_left = lat_obstacle_decision.at(obstacle->id()) ==
                       LatObstacleDecisionType::RIGHT;
  bool is_side_obstacle = false;
  BoundType bound_type = BoundType::DYNAMIC_AGENT;
  const auto lat_obs_position_iter =
      lat_obstacle_position.find(obstacle->id());
  const auto last_lat_obs_decision_iter =
      last_lat_obstacle_decision_.find(obstacle->id());
  if (lat_obs_position_iter != lat_obstacle_position.end()) {
    if (lat_obs_position_iter->second.side_car) {
      if (last_lat_obs_decision_iter != last_lat_obstacle_decision_.end()) {
        if (last_lat_obs_decision_iter->second !=
          LatObstacleDecisionType::IGNORE &&
          lat_obstacle_decision.at(obstacle->id()) ==
          LatObstacleDecisionType::IGNORE) {
          is_nudge_left = last_lat_obs_decision_iter->second ==
                            LatObstacleDecisionType::RIGHT;
          bound_type = BoundType::ADJACENT_AGENT;
        }
      } else {
        last_lat_obstacle_decision_[obstacle->id()] = lat_obstacle_decision.at(obstacle->id());
      }
    } else {
      if (last_lat_obs_decision_iter != last_lat_obstacle_decision_.end()) {
        last_lat_obstacle_decision_.erase(obstacle->id());
      }
    }
    is_side_obstacle = (lat_obs_position_iter->second.side_car) &&
                       (!lat_obs_position_iter->second.front_car);
  }

  const auto borrow_direction = lane_borrow_decider_output.borrow_direction;

  if ((is_in_lane_borrow_status) && (is_blocked_obstacle_)) {
    if (borrow_direction == LEFT_BORROW) {
      // 向左借道
      is_nudge_left = false;
    } else if (borrow_direction == RIGHT_BORROW) {
      // 向右借道
      is_nudge_left = true;
    }
    is_care_rear_obstacle = false;
  }

  bool is_cross_obj{false};
  bool has_lat_decision{false};
  bool has_lon_decision{false};

  double limit_overlap_min_y = -1000;
  double limit_overlap_max_y = 1000;
  // hack: consider that the obstacle is not completely over the car
  bool is_cut_out_side_obstacle =
      IsCutoutSideObstacle(obstacle, limit_overlap_min_y, limit_overlap_max_y);
  double hack_yaw_limit_overlap_min_y = -1000;
  double hack_yaw_limit_overlap_max_y = 1000;
  const bool is_hack_yaw =
      HackYawSideObstacle(obstacle, is_nudge_left, hack_yaw_limit_overlap_min_y,
                          hack_yaw_limit_overlap_max_y);
  if (is_cut_out_side_obstacle) {
    bound_type = BoundType::ADJACENT_AGENT;
  } else if (is_care_rear_obstacle) {
    bound_type = BoundType::REAR_AGENT;
  }

  double extra_decrease_buffer =
      CalculateExtraDecreaseBuffer(obstacle, is_nudge_left);

  double extra_lane_type_decrease_buffer = CalculateExtraLaneTypeDecreaseBuffer(
      is_nudge_left, overlap_start_s_, overlap_end_s_);

  const std::shared_ptr<agent::AgentManager> agent_manager = session_->environmental_model().get_agent_manager();
  const auto *agent = agent_manager->GetAgent(obstacle->id());

  for (size_t i = 0; i < plan_history_traj_.size(); i++) {
    auto &traj_point = plan_history_traj_[i];
    const auto &t = traj_point.t;
    if (t > config_.care_dynamic_object_t_threshold) {
      continue;
    }

    const double ego_s = traj_point.s;
    const double ego_l = traj_point.l;

    const double care_area_s_start =
        ego_s - vehicle_param.rear_edge_to_rear_axle - rear_lon_buf_dis;
    const double care_area_s_end =
        ego_s + rear_axle_to_front_bumper + front_lon_buf_dis;
    const auto care_area_center =
        Vec2d((care_area_s_start + care_area_s_end) * 0.5, ego_l);
    const double care_area_length = care_area_s_end - care_area_s_start;
    const auto care_polygon =  // @cai: consider the heading
        Polygon2d(Box2d(care_area_center, 0, care_area_length, l_care_width));

    Polygon2d obstacle_sl_polygon;
    bool ok = false;
    if (config_.use_obstacle_prediction_model_in_planning) {
      ok = obstacle->get_polygon_at_time(i * config_.delta_t, reference_path_ptr_,
                                                  obstacle_sl_polygon);
    } else {
      ok = obstacle->get_polygon_at_time_tmp(
          i * config_.delta_t, reference_path_ptr_, obstacle_sl_polygon);
    }
    if (!ok) {
      // TBD add log
      return;
    }
    Polygon2d care_overlap_polygon;
    bool b_overlap_with_care = false;
    // default: invalid value
    double overlap_min_y = 100.0;
    double overlap_max_y = -100.0;

    b_overlap_with_care =
        obstacle_sl_polygon.ComputeOverlap(care_polygon, &care_overlap_polygon);
    if (b_overlap_with_care) {
      overlap_min_y = care_overlap_polygon.min_y();
      overlap_max_y = care_overlap_polygon.max_y();
    } else {
      continue;
    }

    if (is_nudge_left) {
      overlap_min_y = std::max(overlap_min_y, limit_overlap_min_y);
    } else {
      overlap_max_y = std::min(overlap_max_y, limit_overlap_max_y);
    }

    double lat_buf_dis =
        general_lateral_decider_utils::CalDesireLateralDistance(
            ego_cart_state_manager_->ego_v(), t, 0, obstacle->type(),
            is_nudge_left, in_intersection, config_);

    if ((is_in_lane_borrow_status) && (is_blocked_obstacle_)) {
      lat_buf_dis += config_.extra_hard_buffer2blockobstacle;
    }

    if (!is_in_lane_borrow_status) {
      lat_buf_dis = std::fmax(lat_buf_dis - extra_lane_width_decrease_buffer_ -
                                  extra_lane_type_decrease_buffer,
                              0.);
    }

    // todo: high speed vehicle
    // do decision
    auto lat_decision = LatObstacleDecisionType::IGNORE;
    auto lon_decision = LonObstacleDecisionType::IGNORE;

    std::pair<double, double> s_side_range{obstacle_sl_polygon.min_x(),
                                           obstacle_sl_polygon.max_x()};
    std::pair<double, double> ego_side_range{
        ego_s - vehicle_param.rear_edge_to_rear_axle,
        ego_s + rear_axle_to_front_bumper + 2.};
    bool b_overlap_side = std::max(s_side_range.first, ego_side_range.first) <
                          std::min(s_side_range.second, ego_side_range.second);

    if (CheckObstacleCrossingCondition(obstacle, is_cross_obj)) {
      // TBD:add logs
    }

    const auto &indexes = match_index_map_[i];
    if (is_cut_out_side_obstacle || is_care_rear_obstacle ||
        lat_obstacle_decision.at(obstacle->id()) ==
        LatObstacleDecisionType::IGNORE ||
        (extra_decrease_buffer < 1e-5 && !is_hack_yaw)) {
      for (auto index : indexes) {
        GenerateObstaclePreliminaryDecision(
            ego_l, ref_path_points_[index].distance_to_right_lane_border,
            ref_path_points_[index].distance_to_left_lane_border, overlap_min_y,
            overlap_max_y, lat_buf_dis, b_overlap_side, init_lon_no_overlap,
            is_nudge_left, is_cross_obj, pre_lateral_decision,
            reset_conflict_decision, obstacle_decision, lat_decision,
            lon_decision);
        has_lat_decision =
            has_lat_decision || lat_decision != LatObstacleDecisionType::IGNORE;
        has_lon_decision =
            has_lon_decision || lon_decision != LonObstacleDecisionType::IGNORE;
      }
      lat_buf_dis = AdjustBufferForSideObstacleInIntersection(
          obstacle, overlap_min_y, overlap_max_y, lat_buf_dis, is_nudge_left,
          rear_lon_buf_dis, front_lon_buf_dis, lat_decision, i);
      AddObstacleDecisionBound(obstacle->id(), t, bound_type, overlap_min_y,
                               overlap_max_y, lat_buf_dis, lat_decision,
                               lon_decision, obstacle_decision);
    } else {
      for (int k = 0; k < 2; k++) {
        if (k == 0) {
          bound_type = BoundType::ADJACENT_AGENT;
        } else {
          overlap_min_y = std::max(overlap_min_y, hack_yaw_limit_overlap_min_y);
          overlap_max_y = std::min(overlap_max_y, hack_yaw_limit_overlap_max_y);
          lat_buf_dis = std::max(lat_buf_dis - extra_decrease_buffer, 0.0);
          bound_type = BoundType::DYNAMIC_AGENT;
          if (!is_in_lane_borrow_status) {
            if (is_side_obstacle && !session_->mutable_planning_context()
                                          ->mutable_lateral_obstacle_decider_output()
                                          .in_intersection) {
              lat_buf_dis = std::fmin(lat_buf_dis, config_.side_obstacle_lat_buffer_limit);
            }
          }
          lat_buf_dis = AdjustBufferForSideObstacleInIntersection(
              obstacle, overlap_min_y, overlap_max_y, lat_buf_dis,
              is_nudge_left, rear_lon_buf_dis, front_lon_buf_dis, lat_decision,
              i);
        }
        for (auto index : indexes) {
          GenerateObstaclePreliminaryDecision(
              ego_l, ref_path_points_[index].distance_to_right_lane_border,
              ref_path_points_[index].distance_to_left_lane_border,
              overlap_min_y, overlap_max_y, lat_buf_dis, b_overlap_side,
              init_lon_no_overlap, is_nudge_left, is_cross_obj,
              pre_lateral_decision, reset_conflict_decision, obstacle_decision,
              lat_decision, lon_decision);
          has_lat_decision = has_lat_decision ||
                             lat_decision != LatObstacleDecisionType::IGNORE;
          has_lon_decision = has_lon_decision ||
                             lon_decision != LonObstacleDecisionType::IGNORE;
        }
        AddObstacleDecisionBound(obstacle->id(), t, bound_type, overlap_min_y,
                                 overlap_max_y, lat_buf_dis, lat_decision,
                                 lon_decision, obstacle_decision);
      }
    }
  }
}

bool GeneralLateralDecider::HackYawSideObstacle(
    const std::shared_ptr<FrenetObstacle> obstacle, bool is_nudge_left,
    double &limit_overlap_min_y, double &limit_overlap_max_y) {
  if (obstacle->frenet_velocity_s() > ego_frenet_state_.velocity_s() &&
      (reference_path_ptr_->get_ego_frenet_boundary().s_end + 1.0 >=
       obstacle->frenet_obstacle_boundary().s_end)) {
    if (is_nudge_left) {
      limit_overlap_min_y =
          std::min(obstacle->frenet_obstacle_corners().l_front_right,
                   (obstacle->frenet_obstacle_corners().l_front_right +
                    obstacle->frenet_obstacle_corners().l_rear_right) *
                       0.5);
    } else {
      limit_overlap_max_y =
          std::max(obstacle->frenet_obstacle_corners().l_front_left,
                   (obstacle->frenet_obstacle_corners().l_front_left +
                    obstacle->frenet_obstacle_corners().l_rear_left) *
                       0.5);
    }
    return true;
  }
  return false;
}

double GeneralLateralDecider::AdjustBufferForSideObstacleInIntersection(
    const std::shared_ptr<FrenetObstacle> obstacle, double overlap_min_y,
    double overlap_max_y, double lat_buf_dis, bool is_nudge_left,
    double rear_lon_buf_dis, double front_lon_buf_dis,
    LatObstacleDecisionType lat_decision, int index) {
  const auto &lat_obstacle_position = session_->mutable_planning_context()
                                    ->mutable_lateral_obstacle_decider_output()
                                      .lateral_obstacle_history_info;
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double half_ego_width = vehicle_param.max_width * 0.5;
  const double planning_init_point_l =
      ego_frenet_state_.planning_init_point().frenet_state.r;
  const auto &in_intersection = session_->mutable_planning_context()
                                    ->mutable_lateral_obstacle_decider_output()
                                      .in_intersection;
  const double basic_nudge_buffer = 0.1;
  bool is_side_obstacle = false;
  const auto lat_obs_position_iter = lat_obstacle_position.find(obstacle->id());
  if (lat_obs_position_iter != lat_obstacle_position.end()) {
    is_side_obstacle = lat_obs_position_iter->second.side_car;
  }

  if (!is_side_obstacle || !in_intersection) {
    return lat_buf_dis;
  }

  const auto &l_care_width = config_.l_care_width;
  const auto &ego_cur_s = plan_history_traj_.front().s;
  const double rear_axle_to_front_bumper =  // TBD：define as consexpr
      vehicle_param.length - vehicle_param.rear_edge_to_rear_axle;
  const double ego_s = plan_history_traj_[index].s;
  const double ego_l = plan_history_traj_[index].l;
  const double care_area_s_start =
      ego_s - vehicle_param.rear_edge_to_rear_axle - rear_lon_buf_dis;
  const double care_area_s_end =
      ego_s + rear_axle_to_front_bumper + front_lon_buf_dis;
  const auto care_area_center =
      Vec2d((care_area_s_start + care_area_s_end) * 0.5, ego_l);
  const double care_area_length = care_area_s_end - care_area_s_start;
  const auto care_polygon =
      Polygon2d(Box2d(care_area_center, 0, care_area_length, l_care_width));

  Polygon2d obstacle_sl_polygon;
  auto ok = obstacle->get_polygon_at_time_tmp(
      index * config_.delta_t, reference_path_ptr_, obstacle_sl_polygon);
  if (!ok) {
    return lat_buf_dis;
  }

  Polygon2d care_overlap_polygon;
  bool b_overlap_with_care = false;
  // default: invalid value
  double overlap_min_y_temp = 100.0;
  double overlap_max_y_temp = -100.0;
  double limit_overlap_min_y_temp = -1000;
  double limit_overlap_max_y_temp = 1000;

  b_overlap_with_care =
      obstacle_sl_polygon.ComputeOverlap(care_polygon, &care_overlap_polygon);
  if (b_overlap_with_care) {
    overlap_min_y_temp = care_overlap_polygon.min_y();
    overlap_max_y_temp = care_overlap_polygon.max_y();
  } else {
    overlap_min_y_temp = obstacle_sl_polygon.min_y();
    overlap_max_y_temp = obstacle_sl_polygon.max_y();
    // return lat_buf_dis;
  }

  if (is_nudge_left) {
    overlap_min_y_temp = std::max(overlap_min_y_temp, limit_overlap_min_y_temp);
  } else {
    overlap_max_y_temp = std::min(overlap_max_y_temp, limit_overlap_max_y_temp);
  }

  if (lat_decision == LatObstacleDecisionType::LEFT) {
    double basic_nudge_position =
        overlap_max_y_temp + lat_buf_dis + half_ego_width;
    double desire_nudge_position = overlap_max_y + lat_buf_dis + half_ego_width;
    double decrease_buffer =
        CalculateSideObstacleExtraDecreaseBufferInIntersection(
            obstacle, is_nudge_left, in_intersection);
    desire_nudge_position =
        std::min(desire_nudge_position - decrease_buffer, basic_nudge_position);
    return desire_nudge_position - overlap_max_y - half_ego_width;
  } else if (lat_decision == LatObstacleDecisionType::RIGHT) {
    double basic_nudge_position =
        overlap_min_y_temp - lat_buf_dis - half_ego_width;
    double desire_nudge_position = overlap_min_y - lat_buf_dis - half_ego_width;
    double decrease_buffer =
        CalculateSideObstacleExtraDecreaseBufferInIntersection(
            obstacle, is_nudge_left, in_intersection);
    desire_nudge_position =
        std::max(desire_nudge_position + decrease_buffer, basic_nudge_position);
    return overlap_min_y - half_ego_width - desire_nudge_position;
  }
  return lat_buf_dis;
}

void GeneralLateralDecider::GenerateObstaclePreliminaryDecision(
    double ego_l, double distance_to_right_lane_border,
    double distance_to_left_lane_border, double overlap_min_y,
    double overlap_max_y, double lat_buf_dis, bool b_overlap_side,
    bool init_lon_no_overlap, bool is_nudge_left, bool is_cross_obj,
    LatObstacleDecisionType pre_lateral_decision, bool &reset_conflict_decision,
    ObstacleDecision &obstacle_decision, LatObstacleDecisionType &lat_decision,
    LonObstacleDecisionType &lon_decision) {
  if (is_nudge_left) {
    lat_decision = LatObstacleDecisionType::RIGHT;
    lon_decision = LonObstacleDecisionType::IGNORE;
  } else {
    lat_decision = LatObstacleDecisionType::LEFT;
    lon_decision = LonObstacleDecisionType::IGNORE;
  }
  // if ((overlap_min_y <= ego_l && ego_l <= overlap_max_y) || is_cross_obj) {
  //   lat_decision = LatObstacleDecisionType::IGNORE;
  //   lon_decision = LonObstacleDecisionType::YIELD;
  // } else if (overlap_min_y > ego_l) {
  //   auto avoid_right_edge = overlap_min_y - lat_buf_dis - half_ego_width;
  //   if (avoid_right_edge >
  //           std::fmax(-distance_to_right_lane_border - avoid_cross_lane,
  //                     -kMaxAvoidEdgeL) or
  //       b_overlap_side) {
  //     lat_decision = LatObstacleDecisionType::RIGHT;
  //     lon_decision = LonObstacleDecisionType::IGNORE;
  //     // lat_decision = LatObstacleDecisionType::IGNORE;
  //     // lon_decision = LonObstacleDecisionType::YIELD;
  //   } else {
  //     lat_decision = LatObstacleDecisionType::IGNORE;
  //     lon_decision = LonObstacleDecisionType::YIELD;
  //   }
  // } else {
  //   assert(overlap_max_y < ego_l);
  //   auto avoid_left_edge = overlap_max_y + lat_buf_dis + half_ego_width;
  //   if (avoid_left_edge <
  //           std::min(distance_to_left_lane_border + avoid_cross_lane,
  //                    kMaxAvoidEdgeL) or
  //       b_overlap_side) {
  //     lat_decision = LatObstacleDecisionType::LEFT;
  //     lon_decision = LonObstacleDecisionType::IGNORE;
  //     // lat_decision = LatObstacleDecisionType::IGNORE;
  //     // lon_decision = LonObstacleDecisionType::YIELD;
  //   } else {
  //     lat_decision = LatObstacleDecisionType::IGNORE;
  //     lon_decision = LonObstacleDecisionType::YIELD;
  //   }
  // }

  if (pre_lateral_decision == LatObstacleDecisionType::IGNORE) {
    pre_lateral_decision = lat_decision;
    if (lat_decision != LatObstacleDecisionType::IGNORE) {
    }
  } else if (pre_lateral_decision == LatObstacleDecisionType::LEFT) {
    if (lat_decision == LatObstacleDecisionType::RIGHT) {
      lat_decision = LatObstacleDecisionType::IGNORE;
      lon_decision = LonObstacleDecisionType::YIELD;
      if (!reset_conflict_decision && init_lon_no_overlap) {
        RefineConflictLatDecisions(ego_l, obstacle_decision);
        reset_conflict_decision = true;
        // add logs
      }
    }
  } else {
    assert(pre_lateral_decision == LatObstacleDecisionType::RIGHT);
    if (lat_decision == LatObstacleDecisionType::LEFT) {
      lat_decision = LatObstacleDecisionType::IGNORE;
      lon_decision = LonObstacleDecisionType::YIELD;
      if (!reset_conflict_decision && init_lon_no_overlap) {
        RefineConflictLatDecisions(ego_l, obstacle_decision);
        reset_conflict_decision = true;
        // add logs
      }
    }
  }
}

void GeneralLateralDecider::AddObstacleDecisionBound(
    int id, double t, BoundType bound_type, double overlap_min_y,
    double overlap_max_y, double lat_buf_dis,
    LatObstacleDecisionType lat_decision, LonObstacleDecisionType lon_decision,
    ObstacleDecision &obstacle_decision, bool is_update_hard_bound) {
  const double l_offset_limit = 10.0;
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double half_ego_width = vehicle_param.max_width * 0.5;

  BoundInfo bound_info;
  Bound bound{-l_offset_limit, l_offset_limit};

  if (lat_decision == LatObstacleDecisionType::LEFT) {
    bound.lower = overlap_max_y + lat_buf_dis + half_ego_width;
    // soft_bound.lower = is_rear_obstacle ? std::max(0.0,
    // care_overlap_polygon.max_y() + lat_buf_dis + half_ego_width) :
    //                                       care_overlap_polygon.max_y() +
    //                                       lat_buf_dis + half_ego_width;
    bound_info.type = bound_type;
    bound_info.id = id;
  } else if (lat_decision == LatObstacleDecisionType::RIGHT) {
    bound.upper = overlap_min_y - lat_buf_dis - half_ego_width;
    // soft_bound.upper = is_rear_obstacle ? std::min(0.0,
    // care_overlap_polygon.min_y() - lat_buf_dis - half_ego_width) :
    //                                       care_overlap_polygon.min_y() -
    //                                       lat_buf_dis - half_ego_width;
    bound_info.type = bound_type;
    bound_info.id = id;
  } else {
    // assert(lon_decision != LonObstacleDecisionType::IGNORE);
  }

  ObstaclePositionDecision position_decision;
  position_decision.tp.t = t;
  // position_decision.tp.s = ego_s;
  // position_decision.tp.l = ego_l;
  if (is_update_hard_bound) {
    position_decision.lat_bounds.push_back(WeightedBound{
        bound.lower, bound.upper, config_.kHardBoundWeight, bound_info});
  } else {
    position_decision.lat_bounds.push_back(WeightedBound{
        bound.lower, bound.upper,
        config_.dynamic_bound_slack_coefficient * config_.kPhysicalBoundWeight,
        bound_info});
  }

  position_decision.lat_decision = lat_decision;
  position_decision.lon_decision = lon_decision;

  obstacle_decision.position_decisions.emplace_back(
      std::move(position_decision));
}

bool GeneralLateralDecider::CheckObstacleNudgeDecision(
    const std::shared_ptr<FrenetObstacle> &obstacle) {
  const auto &lat_obstacle_decision = session_->planning_context()
                                          .lateral_obstacle_decider_output()
                                          .lat_obstacle_decision;
  const auto &lat_obstacle_position = session_->mutable_planning_context()
                                    ->mutable_lateral_obstacle_decider_output()
                                      .lateral_obstacle_history_info;
  const auto lat_obs_decision_iter =
      lat_obstacle_decision.find(obstacle->id());
  if (lat_obs_decision_iter != lat_obstacle_decision.end()) {
    if (lat_obs_decision_iter->second == LatObstacleDecisionType::LEFT ||
        lat_obs_decision_iter->second == LatObstacleDecisionType::RIGHT) {
      return true;
    } else {
      const auto lat_obs_position_iter =
          lat_obstacle_position.find(obstacle->id());
      const auto last_lat_obs_decision_iter =
          last_lat_obstacle_decision_.find(obstacle->id());
      if ((last_lat_obs_decision_iter !=
          last_lat_obstacle_decision_.end()) &&
          (lat_obs_position_iter != lat_obstacle_position.end())) {
        if ((last_lat_obs_decision_iter->second !=
            LatObstacleDecisionType::IGNORE) &&
            (lat_obs_position_iter->second.side_car)) {
          return true;
        }
      }
    }
  }
  return false;
}

bool GeneralLateralDecider::CheckObstacleCrossingCondition(
    const std::shared_ptr<FrenetObstacle> obstacle, bool &is_cross_obj) {
  return false;
}

void GeneralLateralDecider::RefineConflictLatDecisions(
    const double &ego_l, ObstacleDecision &obstacle_decision) {
  for (size_t n_pd = 0; n_pd < obstacle_decision.position_decisions.size();
       ++n_pd) {
    if (obstacle_decision.position_decisions[n_pd].lat_decision !=
        LatObstacleDecisionType::IGNORE) {
      for (size_t n_lb = 0;
           n_lb < obstacle_decision.position_decisions[n_pd].lat_bounds.size();
           ++n_lb) {
        obstacle_decision.position_decisions[n_pd].lat_bounds[n_lb].lower =
            ego_l - config_.l_offset_limit;
        obstacle_decision.position_decisions[n_pd].lat_bounds[n_lb].upper =
            ego_l + config_.l_offset_limit;
      }
      obstacle_decision.position_decisions[n_pd].lat_decision =
          LatObstacleDecisionType::IGNORE;
      obstacle_decision.position_decisions[n_pd].lon_decision =
          LonObstacleDecisionType::YIELD;
    }
  }
}

void GeneralLateralDecider::ExtractBoundary(
    std::vector<std::pair<double, double>> &frenet_soft_bounds,
    std::vector<std::pair<double, double>> &frenet_hard_bounds,
    std::vector<std::pair<BoundInfo, BoundInfo>> &soft_bounds_info,
    std::vector<std::pair<BoundInfo, BoundInfo>> &hard_bounds_info) {
  const double planning_init_point_l =
      ego_frenet_state_.planning_init_point().frenet_state.r;

  for (int i = 0; i < hard_bounds_.size(); i++) {
    std::pair<double, double> hard_bound{-10., 10.};  // <lower ,upper>
    std::pair<BoundInfo, BoundInfo> hard_bound_info;  // <lower ,upper>
    PostProcessBound(planning_init_point_l, hard_bounds_[i], hard_bound,
                     hard_bound_info);
    if (i == 0) {
      ProtectBoundByInitPoint(hard_bound, hard_bound_info);
    }
    frenet_hard_bounds.emplace_back(hard_bound);
    hard_bounds_info.emplace_back(hard_bound_info);
  }

  for (int i = 0; i < soft_bounds_.size(); i++) {
    std::pair<double, double> soft_bound{-10., 10.};  // <lower ,upper>
    std::pair<BoundInfo, BoundInfo> soft_bound_info;  // <lower ,upper>
    PostProcessBound(planning_init_point_l, soft_bounds_[i], soft_bound,
                     soft_bound_info);
    if (i == 0) {
      ProtectBoundByInitPoint(soft_bound, soft_bound_info);
    }
    // soft in hard
    if (soft_bound.first > frenet_hard_bounds[i].second) {
      soft_bound.first = frenet_hard_bounds[i].second;
    } else if (soft_bound.first < frenet_hard_bounds[i].first) {
      soft_bound.first = frenet_hard_bounds[i].first;
    }
    if (soft_bound.second > frenet_hard_bounds[i].second) {
      soft_bound.second = frenet_hard_bounds[i].second;
    } else if (soft_bound.second < frenet_hard_bounds[i].first) {
      soft_bound.second = frenet_hard_bounds[i].first;
    }
    frenet_soft_bounds.emplace_back(soft_bound);
    soft_bounds_info.emplace_back(soft_bound_info);
  }

  assert(frenet_hard_bounds.size() == ref_traj_points_.size());
  assert(frenet_soft_bounds.size() == ref_traj_points_.size());
}

void GeneralLateralDecider::ProtectBoundByInitPoint(
    std::pair<double, double> &bound,
    std::pair<BoundInfo, BoundInfo> &bound_info) {
  const double planning_init_point_l =
      ego_frenet_state_.planning_init_point().frenet_state.r;
  if (bound.first > planning_init_point_l) {
    bound.first = planning_init_point_l;
    bound_info.first.type = BoundType::EGO_POSITION;
  }
  if (bound.second < planning_init_point_l) {
    bound.second = planning_init_point_l;
    bound_info.second.type = BoundType::EGO_POSITION;
  }
}

void GeneralLateralDecider::ExtractDynamicObstacleBound(
    const ObstacleDecision &obstacle_decision) {
  if (plan_history_traj_.size() <= 0 || ref_traj_points_.size() <= 0) {
    return;
  }

  for (auto &obstacle_position_decision :
       obstacle_decision.position_decisions) {
    for (size_t i = 0; i < plan_history_traj_.size(); i++) {
      if (std::fabs(obstacle_position_decision.tp.t - plan_history_traj_[i].t) <
          1e-2) {
        auto obstacle_pos_bounds = obstacle_position_decision.lat_bounds;
        for (auto &obstacle_pos_bound : obstacle_pos_bounds) {
          if (obstacle_pos_bound.weight < 0.) {
            for (auto index : match_index_map_[i]) {
              hard_bounds_[index].emplace_back(obstacle_pos_bound);
            }
          } else {
            for (auto index : match_index_map_[i]) {
              soft_bounds_[index].emplace_back(obstacle_pos_bound);
            }
          }
        }
      }
    }
  }
}

void GeneralLateralDecider::ExtractStaticObstacleBound(
    const ObstacleDecision &obstacle_decision) {
  if (ref_traj_points_.size() <= 0) {
    return;
  }

  for (auto &obstacle_position_decision :
       obstacle_decision.position_decisions) {
    for (size_t i = 0; i < ref_traj_points_.size(); i++) {
      if (std::fabs(obstacle_position_decision.tp.t - ref_traj_points_[i].t) <
          1e-2) {
        auto obstacle_pos_bounds = obstacle_position_decision.lat_bounds;
        for (auto &obstacle_pos_bound : obstacle_pos_bounds) {
          if (obstacle_pos_bound.weight < 0.) {
            hard_bounds_[i].emplace_back(obstacle_pos_bound);
          } else {
            soft_bounds_[i].emplace_back(obstacle_pos_bound);
          }
        }
      }
    }
  }
}

void GeneralLateralDecider::PostProcessBound(
    const double planning_init_point_l,
    const std::vector<WeightedBound> &bounds_input,
    std::pair<double, double> &bound_output,
    std::pair<BoundInfo, BoundInfo> &bound_info) {
  // empty
  const size_t bounds_size = bounds_input.size();
  if (bounds_size == 0) {
    return;
  }
  // copy and sort
  auto compare_bound_upper = [&](WeightedBound bound1, WeightedBound bound2) {
    return bound1.upper < bound2.upper;
  };
  auto compare_bound_lower = [&](WeightedBound bound1, WeightedBound bound2) {
    return bound1.lower > bound2.lower;
  };
  std::vector<WeightedBound> upper_bounds;
  std::vector<WeightedBound> lower_bounds;
  upper_bounds.resize(bounds_size);
  lower_bounds.resize(bounds_size);
  std::copy(bounds_input.begin(), bounds_input.end(), upper_bounds.begin());
  std::copy(bounds_input.begin(), bounds_input.end(), lower_bounds.begin());
  std::sort(upper_bounds.begin(), upper_bounds.end(), compare_bound_upper);
  std::sort(lower_bounds.begin(), lower_bounds.end(), compare_bound_lower);
  // final bound & info
  double lower_bound = std::max(lower_bounds.back().lower, bound_output.first);
  double upper_bound = std::min(upper_bounds.back().upper, bound_output.second);
  BoundInfo lower_bound_info = lower_bounds.back().bound_info;
  BoundInfo upper_bound_info = upper_bounds.back().bound_info;
  // init
  size_t lower_index = 0;
  size_t upper_index = 0;
  bool use_lower_init_protect = false;
  bool use_upper_init_protect = false;
  // end conditin: 1.index >= size;
  while ((lower_index < bounds_size) && (upper_index < bounds_size)) {
    BoundInfo lower_info = lower_bounds[lower_index].bound_info;
    BoundInfo upper_info = upper_bounds[upper_index].bound_info;
    BoundType lower_type = lower_info.type;
    BoundType upper_type = upper_info.type;
    // hack: only road border and agent in hard bounds
    if ((upper_bounds[upper_index].weight < 0.0) &&
        ((upper_type != BoundType::ROAD_BORDER) &&
         (upper_type != BoundType::AGENT))) {
      upper_index += 1;
      continue;
    }
    if ((lower_bounds[lower_index].weight < 0.0) &&
        ((lower_type != BoundType::ROAD_BORDER) &&
         (lower_type != BoundType::AGENT))) {
      lower_index += 1;
      continue;
    }
    double lower = lower_bounds[lower_index].lower;
    double upper = upper_bounds[upper_index].upper;
    const int lower_priority =
        general_lateral_decider_utils::GetBoundTypePriority(lower_type);
    const int upper_priority =
        general_lateral_decider_utils::GetBoundTypePriority(upper_type);
    const double lower_weight = general_lateral_decider_utils::GetBoundWeight(
        lower_type, config_.map_bound_weight);
    const double upper_weight = general_lateral_decider_utils::GetBoundWeight(
        upper_type, config_.map_bound_weight);
    // start compare
    if (upper >= lower) {  // <==> (upper_bound >= lower_bound)
      // end condition 2.upper > upper bound >= lower boud > lower
      if (((upper > upper_bound) && (lower < lower_bound)) ||
          (upper_bound == lower_bound)) {
        break;
      }

      // upper_bound
      if (upper < upper_bound) {
        upper_bound_info = upper_info;
        if (use_upper_init_protect) {
          upper_bound = std::min(upper, planning_init_point_l);
        } else {
          upper_bound = upper;
        }
        if (upper_type == BoundType::ADJACENT_AGENT || upper_type == BoundType::REAR_AGENT) {
          if (upper_bound < planning_init_point_l) {
            upper_bound = planning_init_point_l;
            use_upper_init_protect = true;
          }
        }
      } else if (upper == upper_bound) {
        if ((upper_type == BoundType::ADJACENT_AGENT || upper_type == BoundType::REAR_AGENT) && (upper_index == 0)) {
          if (upper_bound < planning_init_point_l) {
            upper_bound_info = upper_info;
            upper_bound = planning_init_point_l;
            use_upper_init_protect = true;
          }
        }
      }
      // lower_bound
      if (lower > lower_bound) {
        lower_bound_info = lower_info;
        if (use_lower_init_protect) {
          lower_bound = std::max(lower, planning_init_point_l);
        } else {
          lower_bound = lower;
        }
        if (lower_type == BoundType::ADJACENT_AGENT || lower_type == BoundType::REAR_AGENT) {
          if (lower_bound > planning_init_point_l) {
            lower_bound = planning_init_point_l;
            use_lower_init_protect = true;
          }
        }
      } else if (lower == lower_bound) {
        if ((lower_type == BoundType::ADJACENT_AGENT || lower_type == BoundType::REAR_AGENT) && (lower_index == 0)) {
          if (lower_bound > planning_init_point_l) {
            lower_bound_info = lower_info;
            lower_bound = planning_init_point_l;
            use_lower_init_protect = true;
          }
        }
      }
      // continue upper & lower
      upper_index += 1;
      lower_index += 1;
    } else {
      upper_bound_info = upper_info;
      lower_bound_info = lower_info;
      if (upper_priority > lower_priority) {
        if (use_upper_init_protect) {
          upper_bound = std::min(upper, planning_init_point_l);
        } else {
          upper_bound = upper;
        }
        if (use_lower_init_protect) {
          lower_bound = std::max(lower, planning_init_point_l);
        } else {
          lower_bound = upper;
        }
        if (upper_type == BoundType::ADJACENT_AGENT || upper_type == BoundType::REAR_AGENT) {
          if (upper_bound < planning_init_point_l) {
            upper_bound = planning_init_point_l;
            lower_bound =
                std::min(planning_init_point_l, std::max(lower, lower_bound));
            use_upper_init_protect = true;
            // continue upper
            upper_index += 1;
          }
        }
        if (lower_type == BoundType::ADJACENT_AGENT || lower_type == BoundType::REAR_AGENT) {
          if (lower_bound > planning_init_point_l) {
            lower_bound = planning_init_point_l;
            use_lower_init_protect = true;
          }
        }
        // continue lower
        lower_index += 1;
      } else if (upper_priority < lower_priority) {
        if (use_upper_init_protect) {
          upper_bound = std::min(upper, planning_init_point_l);
        } else {
          upper_bound = lower;
        }
        if (use_lower_init_protect) {
          lower_bound = std::max(lower, planning_init_point_l);
        } else {
          lower_bound = lower;
        }
        if (lower_type == BoundType::ADJACENT_AGENT || lower_type == BoundType::REAR_AGENT) {
          if (lower_bound > planning_init_point_l) {
            lower_bound = planning_init_point_l;
            upper_bound =
                std::max(planning_init_point_l, std::min(upper, upper_bound));
            use_lower_init_protect = true;
            // continue lower
            lower_index += 1;
          }
        }
        if (upper_type == BoundType::ADJACENT_AGENT || upper_type == BoundType::REAR_AGENT) {
          if (upper_bound < planning_init_point_l) {
            upper_bound = planning_init_point_l;
            use_upper_init_protect = true;
          }
        }
        // continue upper
        upper_index += 1;
      } else {
        // double mid_bound = std::min(
        //     std::max(upper + (std::max(lower - upper, 0.0) *
        //                       (lower_weight / (upper_weight +
        //                       lower_weight))),
        //              lower_bound), upper_bound);
        double mid_bound =
            upper + (std::max(lower - upper, 0.0) *
                     (lower_weight / (upper_weight + lower_weight)));
        if (use_upper_init_protect) {
          mid_bound = std::min(mid_bound, planning_init_point_l);
        }
        if (use_lower_init_protect) {
          mid_bound = std::max(mid_bound, planning_init_point_l);
        }
        upper_bound = mid_bound;
        lower_bound = mid_bound;
        if (mid_bound < planning_init_point_l) {
          if (upper_type == BoundType::ADJACENT_AGENT || upper_type == BoundType::REAR_AGENT) {
            upper_bound = planning_init_point_l;
            lower_bound = std::min(lower, planning_init_point_l);
            use_upper_init_protect = true;
            upper_index += 1;
          } else {
            // end condition 3.not ADJACENT_AGENT
            break;
          }
        } else if (mid_bound > planning_init_point_l) {
          if (lower_type == BoundType::ADJACENT_AGENT || lower_type == BoundType::REAR_AGENT) {
            upper_bound = std::max(upper, planning_init_point_l);
            lower_bound = planning_init_point_l;
            use_lower_init_protect = true;
            lower_index += 1;
          } else {
            // end condition 3.not ADJACENT_AGENT
            break;
          }
        } else {
          // end condition 4.not across init point
          break;
        }
      }
    }
  }
  bound_output.first = lower_bound;
  bound_output.second = upper_bound;
  bound_info.first = lower_bound_info;
  bound_info.second = upper_bound_info;
}

void GeneralLateralDecider::SaveLatDebugInfo(
    const std::vector<std::pair<double, double>> &frenet_soft_bounds,
    const std::vector<std::pair<double, double>> &frenet_hard_bounds,
    std::vector<std::pair<BoundInfo, BoundInfo>> &soft_bounds_info,
    std::vector<std::pair<BoundInfo, BoundInfo>> &hard_bounds_info,
    GeneralLateralDeciderOutput &general_lateral_decider_output) {
  lat_debug_info_.Clear();
  lat_debug_info_.mutable_bound_s_vec()->Resize(ref_traj_points_.size(), 0.0);
  lat_debug_info_.mutable_hard_lower_bound_info_vec()->Reserve(
      ref_traj_points_.size());
  lat_debug_info_.mutable_hard_upper_bound_info_vec()->Reserve(
      ref_traj_points_.size());
  lat_debug_info_.mutable_soft_lower_bound_info_vec()->Reserve(
      ref_traj_points_.size());
  lat_debug_info_.mutable_soft_upper_bound_info_vec()->Reserve(
      ref_traj_points_.size());

  auto &hard_bounds_frenet_output =
      general_lateral_decider_output.hard_bounds_frenet_point;
  auto &soft_bounds_frenet_output =
      general_lateral_decider_output.soft_bounds_frenet_point;
  auto &hard_bounds_info_output =
      general_lateral_decider_output.hard_bounds_info;
  auto &soft_bounds_info_output =
      general_lateral_decider_output.soft_bounds_info;
  for (size_t i = 0; i < ref_traj_points_.size(); ++i) {
    lat_debug_info_.mutable_bound_s_vec()->Set(i, ref_traj_points_[i].s);

    auto hard_lower_bound_info =
        lat_debug_info_.mutable_hard_lower_bound_info_vec()->Add();
    hard_lower_bound_info->set_lower(frenet_hard_bounds[i].first);
    hard_lower_bound_info->mutable_bound_info()->set_id(
        hard_bounds_info[i].first.id);
    hard_lower_bound_info->mutable_bound_info()->set_type(
        BoundType2String(hard_bounds_info[i].first.type));

    auto hard_upper_bound_info =
        lat_debug_info_.mutable_hard_upper_bound_info_vec()->Add();
    hard_upper_bound_info->set_upper(frenet_hard_bounds[i].second);
    hard_upper_bound_info->mutable_bound_info()->set_id(
        hard_bounds_info[i].second.id);
    hard_upper_bound_info->mutable_bound_info()->set_type(
        BoundType2String(hard_bounds_info[i].second.type));

    auto soft_lower_bound_info =
        lat_debug_info_.mutable_soft_lower_bound_info_vec()->Add();
    soft_lower_bound_info->set_lower(frenet_soft_bounds[i].first);
    soft_lower_bound_info->mutable_bound_info()->set_id(
        soft_bounds_info[i].first.id);
    soft_lower_bound_info->mutable_bound_info()->set_type(
        BoundType2String(soft_bounds_info[i].first.type));

    auto soft_upper_bound_info =
        lat_debug_info_.mutable_soft_upper_bound_info_vec()->Add();
    soft_upper_bound_info->set_upper(frenet_soft_bounds[i].second);
    soft_upper_bound_info->mutable_bound_info()->set_id(
        soft_bounds_info[i].second.id);
    soft_upper_bound_info->mutable_bound_info()->set_type(
        BoundType2String(soft_bounds_info[i].second.type));

    hard_bounds_frenet_output.emplace_back(frenet_hard_bounds[i]);
    soft_bounds_frenet_output.emplace_back(frenet_soft_bounds[i]);
    hard_bounds_info_output.emplace_back(soft_bounds_info[i]);
    soft_bounds_info_output.emplace_back(hard_bounds_info[i]);
  }

  // 障碍物决策是否存在跳动
  lat_debug_info_.mutable_obstacle_ids()->Resize(dynamic_obstacle_decisions_.size() + static_obstacle_decisions_.size(), 0);
  int i = 0;
  for (const auto&dynamic_obstacle_decision : dynamic_obstacle_decisions_) {
    lat_debug_info_.mutable_obstacle_ids()->Set(i, dynamic_obstacle_decision.first);
    i++;
  }
  for (const auto&static_obstacle_decision : static_obstacle_decisions_) {
    lat_debug_info_.mutable_obstacle_ids()->Set(i, static_obstacle_decision.first);
    i++;
  }

  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_lateral_behavior_debug_info()
      ->CopyFrom(lat_debug_info_);
}

void GeneralLateralDecider::PostProcessReferenceTrajBySoftBound(
    const std::vector<std::pair<double, double>> &frenet_soft_bounds,
    GeneralLateralDeciderOutput &general_lateral_decider_output) {
  bool bound_avoid = false;
  for (size_t i = 0; i < ref_traj_points_.size(); i++) {
    if (ref_traj_points_[i].l < frenet_soft_bounds[i].first ||
        ref_traj_points_[i].l > frenet_soft_bounds[i].second) {
      bound_avoid = true;
    }
    ref_traj_points_[i].l =
        std::min(std::max(ref_traj_points_[i].l, frenet_soft_bounds[i].first),
                 frenet_soft_bounds[i].second);
  }
  general_lateral_decider_output.bound_avoid = bound_avoid;
}

void GeneralLateralDecider::GenerateLateralDeciderOutput(
    const std::vector<std::pair<double, double>> &frenet_soft_bounds,
    const std::vector<std::pair<double, double>> &frenet_hard_bounds,
    GeneralLateralDeciderOutput &general_lateral_decider_output) {
  general_lateral_decider_output.soft_bounds = std::move(soft_bounds_);
  general_lateral_decider_output.hard_bounds = std::move(hard_bounds_);

  GenerateEnuBoundaryPoints(frenet_soft_bounds, frenet_hard_bounds,
                            general_lateral_decider_output);

  GenerateEnuReferenceTheta(general_lateral_decider_output);

  GenerateEnuReferenceTraj(general_lateral_decider_output);
}

void GeneralLateralDecider::GenerateEnuBoundaryPoints(
    const std::vector<std::pair<double, double>> &frenet_soft_bounds,
    const std::vector<std::pair<double, double>> &frenet_hard_bounds,
    GeneralLateralDeciderOutput &general_lateral_decider_output) {
  auto &soft_bounds_output =
      general_lateral_decider_output.soft_bounds_cart_point;
  auto &hard_bounds_output =
      general_lateral_decider_output.hard_bounds_cart_point;

  const std::shared_ptr<planning_math::KDPath> frenet_coord =
      reference_path_ptr_->get_frenet_coord();
  Point2D tmp_soft_lower_point;
  Point2D tmp_soft_upper_point;
  Point2D tmp_hard_lower_point;
  Point2D tmp_hard_upper_point;
  for (size_t i = 0; i < ref_traj_points_.size(); ++i) {
    if (!frenet_coord->SLToXY(
            Point2D(ref_traj_points_[i].s, frenet_soft_bounds[i].first),
            tmp_soft_lower_point))  // soft lower
    {
      // TODO: add logs
    }

    if (!frenet_coord->SLToXY(
            Point2D(ref_traj_points_[i].s, frenet_soft_bounds[i].second),
            tmp_soft_upper_point))  // soft upper
    {
      // TODO: add logs
    }
    soft_bounds_output.emplace_back(std::pair<Point2D, Point2D>(
        tmp_soft_lower_point, tmp_soft_upper_point));

    if (!frenet_coord->SLToXY(
            Point2D(ref_traj_points_[i].s, frenet_hard_bounds[i].first),
            tmp_hard_lower_point))  // hard lower
    {
      // TODO: add logs
    }

    if (!frenet_coord->SLToXY(
            Point2D(ref_traj_points_[i].s, frenet_hard_bounds[i].second),
            tmp_hard_upper_point))  // hard upper
    {
      // TODO: add logs
    }

    hard_bounds_output.emplace_back(std::pair<Point2D, Point2D>(
        tmp_hard_lower_point, tmp_hard_upper_point));
  }
};

void GeneralLateralDecider::GenerateEnuReferenceTraj(
    GeneralLateralDeciderOutput &general_lateral_decider_output) {
  const std::shared_ptr<planning_math::KDPath> frenet_coord =
      reference_path_ptr_->get_frenet_coord();
  auto &enu_ref_path = general_lateral_decider_output.enu_ref_path;
  enu_ref_path.resize(ref_traj_points_.size());

  Point2D ref_point;
  for (size_t i = 0; i < ref_traj_points_.size(); i++) {
    if (!frenet_coord->SLToXY(
            Point2D(ref_traj_points_[i].s, ref_traj_points_[i].l),
            ref_point))  // soft lower
    {
      // TODO: add logs
    }
    enu_ref_path[i].first = ref_point.x;
    enu_ref_path[i].second = ref_point.y;
  }

  const auto &s_start = ref_traj_points_.front().s;
  const auto &s_end = ref_traj_points_.back().s;

  general_lateral_decider_output.v_cruise =
      (s_end - s_start) / (config_.delta_t * (ref_traj_points_.size() - 1));
}

void GeneralLateralDecider::GenerateEnuReferenceTheta(
    GeneralLateralDeciderOutput &general_lateral_decider_output) {
  auto &enu_ref_theta = general_lateral_decider_output.enu_ref_theta;

  for (size_t i = 0; i < ref_path_points_.size(); i++) {
    enu_ref_theta.emplace_back(ref_traj_points_[i].heading_angle);
  }
}

void GeneralLateralDecider::SampleRoadDistanceInfo(
    const double &s_target, ReferencePathPoint &sample_path_point) {
  const double cut_length = config_.sample_step;
  ReferencePathPoint refpath_pt{};
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();

  for (double s = s_target - vehicle_param.rear_edge_to_rear_axle;
       s < s_target + vehicle_param.front_edge_to_rear_axle +
               config_.sample_forward_distance;
       s += cut_length) {
    if (reference_path_ptr_->get_reference_point_by_lon(s, refpath_pt)) {
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
}

void GeneralLateralDecider::CalculateAvoidObstacles(
    const std::vector<std::pair<double, double>> frenet_soft_bounds,
    std::vector<std::pair<BoundInfo, BoundInfo>> soft_bounds_info) {
  auto &lateral_offset_decider_output =
      session_->mutable_planning_context()
          ->mutable_lateral_offset_decider_output();
  const double planning_init_point_l =
      ego_frenet_state_.planning_init_point().frenet_state.r;
  for (int i = 0; i < frenet_soft_bounds.size(); i++) {
    // lower bound 是否影响自车
    auto find_object = reference_path_ptr_->get_obstacles_map().find(soft_bounds_info[i].first.id);
    if (find_object == reference_path_ptr_->get_obstacles_map().end()) {
      continue;
    }
    auto frenet_obstacle = find_object->second;
    if (reference_path_ptr_->get_ego_frenet_boundary().s_start >
          frenet_obstacle->frenet_obstacle_boundary().s_end) {
      continue;
    }
    if ((soft_bounds_info[i].first.type == BoundType::DYNAMIC_AGENT ||
         soft_bounds_info[i].first.type == BoundType::AGENT) &&
         soft_bounds_info[i].first.id != -100) {
      if (frenet_soft_bounds[i].first > planning_init_point_l ||
          frenet_soft_bounds[i].first > config_.bound2center_line_distance_thr) {
        if (std::find(lateral_offset_decider_output.avoid_ids.begin(),
            lateral_offset_decider_output.avoid_ids.end(),
            soft_bounds_info[i].first.id) == lateral_offset_decider_output.avoid_ids.end()) {
          lateral_offset_decider_output.avoid_ids.emplace_back(soft_bounds_info[i].first.id);
        }
      }
    }
    // upper bound 是否影响自车
    find_object = reference_path_ptr_->get_obstacles_map().find(soft_bounds_info[i].second.id);
    if (find_object == reference_path_ptr_->get_obstacles_map().end()) {
      continue;
    }
    frenet_obstacle = find_object->second;
    if (reference_path_ptr_->get_ego_frenet_boundary().s_start >
          frenet_obstacle->frenet_obstacle_boundary().s_end) {
      continue;
    }
    if ((soft_bounds_info[i].second.type == BoundType::DYNAMIC_AGENT ||
         soft_bounds_info[i].second.type == BoundType::AGENT) &&
         soft_bounds_info[i].second.id != -100) {
      if (frenet_soft_bounds[i].second < planning_init_point_l ||
          frenet_soft_bounds[i].second < -config_.bound2center_line_distance_thr) {
        if (std::find(lateral_offset_decider_output.avoid_ids.begin(),
            lateral_offset_decider_output.avoid_ids.end(),
            soft_bounds_info[i].second.id) == lateral_offset_decider_output.avoid_ids.end()) {
          lateral_offset_decider_output.avoid_ids.emplace_back(soft_bounds_info[i].second.id);
        }
      }
    }
  }
  JSON_DEBUG_VECTOR("lateral_avoid_ids", lateral_offset_decider_output.avoid_ids, 0);
}

void GeneralLateralDecider::CalcLateralBehaviorOutput() {
  auto &lateral_output = session_->mutable_planning_context()
                             ->mutable_lateral_behavior_planner_output();
  const auto &lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto &coarse_planning_info =
      lane_change_decider_output.coarse_planning_info;

  const std::shared_ptr<VirtualLane> flane =
      session_->environmental_model()
          .get_virtual_lane_manager()
          ->get_lane_with_virtual_id(coarse_planning_info.target_lane_id);
  const std::shared_ptr<VirtualLaneManager> virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();

  // path points
  std::vector<planning_math::PathPoint> path_points;
  if (flane != nullptr && flane->get_reference_path() != nullptr) {
    auto &ref_path = flane->get_reference_path();
    for (auto &ref_point : ref_path->get_points()) {
      path_points.emplace_back(ref_point.path_point);
    }
  }
  lateral_output.scenario = lane_change_decider_output.scenario;
  // lc info
  int lc_request = lane_change_decider_output.lc_request;

  if (lc_request == NO_CHANGE) {
    lateral_output.lc_request = "none";
  } else if (lc_request == LEFT_CHANGE) {
    lateral_output.lc_request = "left";
  } else {
    lateral_output.lc_request = "right";
  }

  const auto state = lane_change_decider_output.curr_state;
  const auto lc_request_direction = lane_change_decider_output.lc_request;
  bool is_LC_LCHANGE =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == LEFT_CHANGE);
  bool is_LC_RCHANGE =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == RIGHT_CHANGE);
  bool is_LC_LWAIT =
      (state == kLaneChangePropose) && (lc_request_direction == LEFT_CHANGE);
  bool is_LC_RWAIT =
      (state == kLaneChangePropose) && (lc_request_direction == RIGHT_CHANGE);
  bool is_LC_LBACK =
      (state == kLaneChangeCancel) && (lc_request_direction == LEFT_CHANGE);
  bool is_LC_RBACK =
      (state == kLaneChangeCancel) && (lc_request_direction == RIGHT_CHANGE);
  //(fengwang31)TODO:交互式变道的的取消状态还需要考虑进去
  if (is_LC_LCHANGE) {
    lateral_output.lc_status = "left_lane_change";
  } else if (is_LC_LBACK) {
    lateral_output.lc_status = "left_lane_change_back";
  } else if (is_LC_RCHANGE) {
    lateral_output.lc_status = "right_lane_change";
  } else if (is_LC_RBACK) {
    lateral_output.lc_status = "right_lane_change_back";
  } else if (is_LC_LWAIT) {
    lateral_output.lc_status = "left_lane_change_wait";
  } else if (is_LC_RWAIT) {
    lateral_output.lc_status = "right_lane_change_wait";
  } else {
    lateral_output.lc_status = "none";
  }
  // flane width
  lateral_output.flane_width = flane->width();
  // lat offset, attention!
  const LateralOffsetDeciderOutput &lateral_offset_decider_output =
      session_->mutable_planning_context()->lateral_offset_decider_output();
  // borrow_bicycle_lane
  lateral_output.lat_offset = lateral_offset_decider_output.lateral_offset;
  bool isRedLightStop = false;  // attention again!!!
  TrackedObject *lead_one = session_->mutable_environmental_model()
                                ->get_lateral_obstacle()
                                ->leadone();

  if (((virtual_lane_manager->current_lane_virtual_id() ==
        virtual_lane_manager->get_lane_num() - 1) ||
       (virtual_lane_manager->current_lane_virtual_id() ==
            virtual_lane_manager->get_lane_num() - 2 &&
        virtual_lane_manager->get_right_lane() != nullptr &&
        virtual_lane_manager->get_right_lane()->get_lane_type() ==
            iflyauto::LANETYPE_NON_MOTOR)) &&
      ((!isRedLightStop && lead_one != nullptr && lead_one->type == 20001))) {
    lateral_output.borrow_bicycle_lane = true;
  } else {
    lateral_output.borrow_bicycle_lane = false;
  }
  // enable intersection planner
  lateral_output.enable_intersection_planner = false;  // attention again!!!
  // dist rblane
  lateral_output.dist_rblane = 10.;  // attention again!!!

  // tleft_lane
  bool left_direct_exist = true;  // attention agagin!!!
  if (virtual_lane_manager->get_left_lane() == nullptr ||
      left_direct_exist == false) {
    lateral_output.tleft_lane = true;
  } else {
    lateral_output.tleft_lane = false;
  }

  // rightest_lane
  if (((virtual_lane_manager->current_lane_virtual_id() ==
        virtual_lane_manager->get_lane_num() - 1) ||
       (virtual_lane_manager->current_lane_virtual_id() ==
            virtual_lane_manager->get_lane_num() - 2 &&
        virtual_lane_manager->get_right_lane() != nullptr &&
        virtual_lane_manager->get_right_lane()->get_lane_type() ==
            iflyauto::LANETYPE_NON_MOTOR)) &&
      virtual_lane_manager->current_lane_virtual_id() - 1 >= 0) {
    lateral_output.rightest_lane = true;
  } else {
    lateral_output.rightest_lane = false;
  }

  // dist_intersect, attention again!!!
  lateral_output.dist_intersect = 1000;

  // intersect length, attention again!!!
  if (virtual_lane_manager->get_intersection_info().intsect_length() !=
      DBL_MAX) {
    lateral_output.intersect_length =
        virtual_lane_manager->get_intersection_info().intsect_length();
  } else {
    lateral_output.intersect_length = 1000;
  }

  // is on highway
  lateral_output.isOnHighway = session_->environmental_model().is_on_highway();

  // d_poly ,c_poly
  auto &d_poly = lateral_output.d_poly;
  auto &c_poly = lateral_output.c_poly;

  d_poly.resize(flane->get_center_line().size());
  c_poly.resize(flane->get_center_line().size());

  std::reverse_copy(flane->get_center_line().begin(),
                    flane->get_center_line().end(), d_poly.begin());
  std::reverse_copy(flane->get_center_line().begin(),
                    flane->get_center_line().end(), c_poly.begin());
}

bool GeneralLateralDecider::IsAgentPredLonOverlapWithPlanPath(
    const std::shared_ptr<FrenetObstacle> obstacle) {
  // const double KDynamicLonOverlapDisBuffer = 1.0;
  const double dynamic_lon_overlap_dis_buffer =
      general_lateral_decider_utils::CalDesireLonOverlapDistance(
          ego_frenet_state_.velocity_s(), obstacle->frenet_velocity_s(),
          config_.use_obstacle_prediction_model_in_planning);
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double rear_axle_to_front_bumper =  // TBD：define as consexpr
      vehicle_param.length - vehicle_param.rear_edge_to_rear_axle;
  const double span_t = config_.delta_t * config_.num_step;
  for (size_t i = 0; i < plan_history_traj_.size(); i++) {
    auto &traj_point = plan_history_traj_[i];
    const auto &t = traj_point.t;
    if (t > span_t) {
      continue;
    }
    const double ego_s_start =
        traj_point.s - vehicle_param.rear_edge_to_rear_axle;
    const double ego_s_end = traj_point.s + rear_axle_to_front_bumper;

    Polygon2d obstacle_sl_polygon;
    bool ok = false;
    if (config_.use_obstacle_prediction_model_in_planning) {
      ok = obstacle->get_polygon_at_time(t, reference_path_ptr_,
                                         obstacle_sl_polygon);
    } else {
      ok = obstacle->get_polygon_at_time_tmp(t, reference_path_ptr_,
                                             obstacle_sl_polygon);
    }
    if (!ok) {
      continue;
    }
    const double obstacle_s_start = obstacle_sl_polygon.min_x();
    const double obstacle_s_end = obstacle_sl_polygon.max_x();

    double start_s = std::max(ego_s_start, obstacle_s_start);
    double end_s = std::min(ego_s_end, obstacle_s_end);
    if (start_s - dynamic_lon_overlap_dis_buffer < end_s) {
      overlap_start_s_ = start_s;
      overlap_end_s_ = end_s;
      return true;
    }
  }
  return false;
}

bool GeneralLateralDecider::IsLonOverlap(
    const std::shared_ptr<FrenetObstacle> obstacle) {
  const double obstacle_s_start = obstacle->frenet_obstacle_boundary().s_start;
  const double obstacle_s_end = obstacle->frenet_obstacle_boundary().s_end;

  const double ego_s_start = ego_frenet_state_.boundary().s_start;
  const double ego_s_end = ego_frenet_state_.boundary().s_end;
  double start_s = std::max(ego_s_start, obstacle_s_start);
  double end_s = std::min(ego_s_end, obstacle_s_end);
  if (start_s < end_s) {
    return true;
  }
  return false;
}

bool GeneralLateralDecider::IsFarObstacle(
    const std::shared_ptr<FrenetObstacle> obstacle) {
  const auto &care_object_lat_distance_threshold =
      config_.care_obj_lat_distance_threshold;
  const auto &care_object_lon_distance_threshold =
      config_.care_obj_lon_distance_threshold;
  const auto &ego_cur_s = plan_history_traj_.front().s;
  const auto &ego_cur_l = plan_history_traj_.front().l;
  if (std::fabs(obstacle->frenet_s() - ego_cur_s) >
          care_object_lon_distance_threshold or
      std::fabs(obstacle->frenet_l() - ego_cur_l) >
          care_object_lat_distance_threshold) {
    // TBD: add log
    return false;
  }
  return true;
}

bool GeneralLateralDecider::IsRearObstacle(
    const std::shared_ptr<FrenetObstacle> obstacle) {
  return reference_path_ptr_->get_ego_frenet_boundary().s_start >
         obstacle->frenet_obstacle_boundary().s_end;
}

bool GeneralLateralDecider::IsBlockedObstacleInLaneBorrow(
    const std::shared_ptr<FrenetObstacle> obstacle) {
  const auto &lane_borrow_decider_output =
      session_->planning_context().lane_borrow_decider_output();
  const auto &obs_type = obstacle->type();
  const auto obs_fusion_source = obstacle->obstacle()->fusion_source();
  if ((obs_fusion_source & OBSTACLE_SOURCE_CAMERA) == 0) {
    return false;
  }

  const auto &blocked_obstacles = lane_borrow_decider_output.blocked_obs_id;
  const bool is_in_lane_borrow_status =
      lane_borrow_decider_output.is_in_lane_borrow_status;

  const bool is_exceed_blocked_obstacle =
      reference_path_ptr_->get_ego_frenet_boundary().s_start >
      obstacle->frenet_obstacle_boundary().s_end +
          config_.care_exceed_distance_with_blocked_obstacle;

  if ((is_in_lane_borrow_status) && (!is_exceed_blocked_obstacle) &&
      (std::find(blocked_obstacles.begin(), blocked_obstacles.end(),
                 obstacle->id()) != blocked_obstacles.end())) {
    return true;
  } else {
    return false;
  }
}

bool GeneralLateralDecider::IsFilterForStaticObstacle(
    const std::shared_ptr<FrenetObstacle> obstacle) {
  const auto &obs_type = obstacle->type();
  const auto obs_fusion_source = obstacle->obstacle()->fusion_source();

  is_blocked_obstacle_ = IsBlockedObstacleInLaneBorrow(obstacle);
  if (is_blocked_obstacle_) {
    return true;
  }

  if ((obs_fusion_source & OBSTACLE_SOURCE_CAMERA) == 0) {
    return false;
  }

  if (obs_type == iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN or  // TBD: check
                                                                // obstacle type
      obs_type == iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN_MOVABLE or
      obs_type == iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN_IMMOVABLE or
      obs_type == iflyauto::ObjectType::OBJECT_TYPE_VAN or
      obs_type == iflyauto::ObjectType::OBJECT_TYPE_TRAILER or
      obs_type == iflyauto::ObjectType::OBJECT_TYPE_TRAFFIC_TEM_SIGN) {
    // add logs;
    return false;
  }

  // filter far away object
  if (!IsFarObstacle(obstacle)) {
    return false;
  }

  // filter rear object without care
  if (IsRearObstacle(obstacle) &&
     (!IsAgentPredLonOverlapWithPlanPath(obstacle))) {
    return false;
  }

  // no nudge decision
  if (!CheckObstacleNudgeDecision(obstacle)) {
    return false;
  }
  return true;
}

bool GeneralLateralDecider::IsFilterForDynamicObstacle(
    const std::shared_ptr<FrenetObstacle> obstacle) {
  const auto &obs_type = obstacle->type();
  const auto obs_fusion_source = obstacle->obstacle()->fusion_source();

  is_blocked_obstacle_ = IsBlockedObstacleInLaneBorrow(obstacle);
  if (is_blocked_obstacle_) {
    return true;
  }

  if ((obs_fusion_source & OBSTACLE_SOURCE_CAMERA) == 0) {
    return false;
  }

  if (obs_type == iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN or  // TBD: check
                                                                // obstacle type
      obs_type == iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN_MOVABLE or
      obs_type == iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN_IMMOVABLE or
      obs_type == iflyauto::ObjectType::OBJECT_TYPE_VAN or
      obs_type == iflyauto::ObjectType::OBJECT_TYPE_TRAILER or
      obs_type == iflyauto::ObjectType::OBJECT_TYPE_TRAFFIC_TEM_SIGN) {
    // add logs;
    return false;
  }

  // filter far away object
  if (!IsFarObstacle(obstacle)) {
    return false;
  }

  // no nudge decision
  if (!CheckObstacleNudgeDecision(obstacle)) {
    return false;
  }

  if (!IsAgentPredLonOverlapWithPlanPath(obstacle)) {
    return false;
  }

  return true;
}
}  // namespace planning
