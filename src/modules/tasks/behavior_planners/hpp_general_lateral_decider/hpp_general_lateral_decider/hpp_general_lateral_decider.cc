#include "hpp_general_lateral_decider.h"

#include <assert.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "agent_node_manager.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "frenet_ego_state.h"
#include "hpp_general_lateral_decider_utils.h"
#include "log.h"
#include "math/polygon2d.h"
#include "spline.h"
#include "task_basic_types.h"
#include "utils/kd_path.h"
#include "vehicle_config_context.h"
#include "virtual_lane_manager.h"

namespace planning {

using namespace planning_math;
using namespace pnc::spline;

HppGeneralLateralDecider::HppGeneralLateralDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : BaseGeneralLateralDecider(config_builder, session) {
  name_ = "HppGeneralLateralDecider";
  config_ = config_builder->cast<HppGeneralLateralDeciderConfig>();
  second_frenet_soft_bounds_.resize(config_.num_step + 1);
  first_frenet_soft_bounds_.resize(config_.num_step + 1);
  frenet_hard_bounds_.resize(config_.num_step + 1);
  second_soft_bounds_info_.resize(config_.num_step + 1);
  first_soft_bounds_info_.resize(config_.num_step + 1);
  hard_bounds_info_.resize(config_.num_step + 1);
}

bool HppGeneralLateralDecider::InitInfo() {
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

  is_ego_reverse_ = false;
  if (session_->is_rads_scene()) {
    is_ego_reverse_ = true;
  }
  min_road_radius_ = 10000.0;
  static_obstacle_decisions_.clear();
  dynamic_obstacle_decisions_.clear();
  ref_traj_points_.clear();
  plan_history_traj_.clear();
  match_index_map_.clear();
  ref_path_points_.clear();
  first_soft_bounds_.clear();
  second_soft_bounds_.clear();
  hard_bounds_.clear();
  second_frenet_soft_bounds_.assign(second_frenet_soft_bounds_.size(),
                             std::make_pair(0.0, 0.0));
  first_frenet_soft_bounds_.assign(first_frenet_soft_bounds_.size(),
                             std::make_pair(0.0, 0.0));
  frenet_hard_bounds_.assign(frenet_hard_bounds_.size(),
                             std::make_pair(0.0, 0.0));
  second_soft_bounds_info_.assign(second_soft_bounds_info_.size(),
                           std::make_pair(BoundInfo(), BoundInfo()));
  first_soft_bounds_info_.assign(first_soft_bounds_info_.size(),
                           std::make_pair(BoundInfo(), BoundInfo()));
  hard_bounds_info_.assign(hard_bounds_info_.size(),
                           std::make_pair(BoundInfo(), BoundInfo()));
  // vehicle_dynamic_buffer_.clear();
  return true;
}

bool HppGeneralLateralDecider::Execute() {
  LOG_DEBUG("=======HppGeneralLateralDecider======= \n");

  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }

  auto start_time = IflyTime::Now_ms();

  if (!InitInfo()) {
    return false;
  };


  auto &traj_points = session_->mutable_planning_context()
                          ->mutable_planning_result()
                          .traj_points;

  ConstructReferencePathPoints();
  GenerateRoadAndLaneBoundary();

  GenerateObstaclesBoundary();

  ExtractBoundary(second_frenet_soft_bounds_, first_frenet_soft_bounds_,
                  frenet_hard_bounds_, second_soft_bounds_info_,
                  first_soft_bounds_info_, hard_bounds_info_);

  PostProcessBoundary();

  auto &general_lateral_decider_output =
      session_->mutable_planning_context()
          ->mutable_general_lateral_decider_output();

  CalculateAvoidObstacles(first_frenet_soft_bounds_, first_soft_bounds_info_,
                          second_frenet_soft_bounds_, second_soft_bounds_info_);

  PostProcessReferenceTrajBySoftBound(second_frenet_soft_bounds_, first_frenet_soft_bounds_,
                                      general_lateral_decider_output);

  GenerateBoundCenterLine(frenet_hard_bounds_, 2.0);
  MergeReferenceTrajectories(frenet_hard_bounds_, second_frenet_soft_bounds_);

  GenerateLateralDeciderOutput(second_frenet_soft_bounds_, first_frenet_soft_bounds_,
                               frenet_hard_bounds_, second_soft_bounds_info_,
                               first_soft_bounds_info_, hard_bounds_info_,
                               general_lateral_decider_output);

  traj_points = ref_traj_points_;

  CalcLateralBehaviorOutput();

  SaveLatDebugInfo(second_frenet_soft_bounds_, first_frenet_soft_bounds_, frenet_hard_bounds_, second_soft_bounds_info_,
                   first_soft_bounds_info_, hard_bounds_info_);

  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("HppGeneralLateralDeciderCostTime", end_time - start_time);

  return true;
}

bool HppGeneralLateralDecider::ExecuteTest(bool pipeline_test) {
  // pipeline test
  return true;
}

void HppGeneralLateralDecider::UnitTest() {
  for (int i = 1; i < 10; i++) {
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
        bounds_input.emplace_back(
            WeightedBound{-10, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::ROAD_BORDER}});
        bounds_input.emplace_back(
            WeightedBound{-1, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        bounds_input.emplace_back(
            WeightedBound{-10, 2, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::AGENT}});
        init_l = 0;
      } break;
      case 2: {
        // case 1:
        //           type
        //           upper     10      -1
        //           lower          3       -10
        bounds_input.emplace_back(
            WeightedBound{-10, 10, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::ROAD_BORDER}});
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
      case 4: {
        // case 4:
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
      case 5: {
        // case 4:
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

      case 6: {
        // case 4:
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
      case 7: {
        // case 4:
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

      case 8: {
        // case 4:
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
      case 9: {
        // case 4:
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
    }
    PostProcessBound(init_l, bounds_input, bound_output, bound_info);
    ILOG_DEBUG << "case " << i << bound_output.first << bound_output.second;
  }
}

bool HppGeneralLateralDecider::CalCruiseVelByCurvature(
    const double ego_v, const std::vector<double> &d_poly, double &cruise_v) {
  if (session_->environmental_model()
          .get_virtual_lane_manager()
          ->get_is_exist_ramp_on_road() ||
      session_->environmental_model()
          .get_virtual_lane_manager()
          ->get_is_exist_split_on_ramp()) {
    return false;
  }
  if ((config_.ramp_limit_v_valid) && (session_->environmental_model()
                                           .get_route_info()
                                           ->get_route_info_output()
                                           .is_on_ramp)) {
    cruise_v = std::min(std::max(config_.ramp_limit_v, ego_v), cruise_v);
  }
  const double preview_length = 20.0;
  const double preview_step = 1.0;
  // double sum_close_kappa = 0.0;
  double sum_far_kappa = 0.0;
  double preview_x = 3.0 * ego_v - 10.0;
  std::vector<double> d_polys;
  d_polys.resize(d_poly.size());
  std::reverse_copy(d_poly.begin(), d_poly.end(), d_polys.begin());
  for (double preview_distance = 0.0; preview_distance < preview_length;
       preview_distance += preview_step) {
    // sum_close_kappa +=
    //     std::fabs(2 * d_polys[0] * preview_distance + d_polys[1]) /
    //     std::pow(
    //         std::pow(2 * d_polys[0] * preview_distance + d_polys[1], 2) + 1,
    //         1.5);
    sum_far_kappa +=
        std::fabs(2 * d_polys[0] * (preview_distance + preview_x) +
                  d_polys[1]) /
        std::pow(std::pow(2 * d_polys[0] * (preview_distance + preview_x) +
                              d_polys[1],
                          2) +
                     1,
                 1.5);
  }

  if ((std::fabs(preview_length) > 1e-6) && (std::fabs(preview_step) > 1e-6)) {
    // double aver_close_kappa = sum_close_kappa / std::max((preview_length /
    // preview_step), 1.0);
    double aver_far_kappa =
        sum_far_kappa / std::max((preview_length / preview_step), 1.0);
    // double close_kappa_radius = 1.0 / std::max(aver_close_kappa, 0.0001);
    double far_kappa_radius = 1.0 / std::max(aver_far_kappa, 0.0001);
    // JSON_DEBUG_VALUE("close_kappa_radius", close_kappa_radius);
    JSON_DEBUG_VALUE("far_kappa_radius", far_kappa_radius);
    // if ((close_kappa_radius < 750.0) || (far_kappa_radius < 750.0)) {
    //   double road_radius = close_kappa_radius < far_kappa_radius
    //                           ? close_kappa_radius
    //                           : far_kappa_radius;
    if (far_kappa_radius < 750.0) {
      double road_radius = far_kappa_radius;
      std::array<double, 4> xp_radius{100.0, 200.0, 400.0, 600.0};
      std::array<double, 4> fp_acc{1.5, 0.9, 0.7, 0.6};
      double acc_max = interp(road_radius, xp_radius, fp_acc);
      cruise_v = std::min(
          std::max(std::sqrt(acc_max * road_radius) * 0.9, ego_v), cruise_v);
      return true;
    }
  }
  return false;
}

void HppGeneralLateralDecider::CalculateLonSampleLength() {
  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  const auto &parking_slot_manager =
      session_->environmental_model().get_parking_slot_manager();
  const auto &frenet_coord = reference_path_ptr_->get_frenet_coord();
  const auto &planning_init_point =
      ego_cart_state_manager_->planning_init_point();

  Eigen::Vector2d cart_init_point(
      ego_cart_state_manager_->planning_init_point().lat_init_state.x(),
      ego_cart_state_manager_->planning_init_point().lat_init_state.y());
  Point2D frenet_init_pt{0.0, 3.0};
  Point2D cart_init_pt(cart_init_point.x(), cart_init_point.y());
  frenet_coord->XYToSL(cart_init_pt, frenet_init_pt);

  const double kMaxAcc = 0.2;
  const double kMinAcc = -5.5;
  double cruise_v = session_->planning_context().v_ref_cruise();
  double ego_v = planning_init_point.v;
  double s_ref = frenet_init_pt.x;

  const auto static_analysis_storage =
      reference_path_ptr_->get_static_analysis_storage();

  is_point_in_turning_ = false;

  constexpr double kPreviewBeforeTurn = 5.0;
  constexpr double kExitRecoverDist = 5.0;

  if (static_analysis_storage) {
    const QueryTypeInfo turn_query(CRoadType::Turn, CPassageType::Ignore,
                                    CElemType::Ignore);
    const auto &front_turn_range =
        static_analysis_storage->GetFrontSRange(turn_query, s_ref);
    const auto &back_turn_range =
        static_analysis_storage->GetBackSRange(turn_query, s_ref);

    const double origin_cruise_v = cruise_v;
    const double kMinCruiseV = 2.0;

    double factor = 1.0;

    if (front_turn_range.second > front_turn_range.first) {
      const double dist_to_turn = std::max(front_turn_range.first - s_ref, 0.0);
      if (dist_to_turn <= kPreviewBeforeTurn) {
        const double ratio = std::clamp(
            1.0 - dist_to_turn / kPreviewBeforeTurn, 0.0, 1.0);
        factor = 1.0 - 0.5 * ratio;
        is_point_in_turning_ = true;
      }
    }

    if (back_turn_range.second > back_turn_range.first &&
                back_turn_range.second < s_ref) {
      const double dist_since_turn_end =
          std::max(s_ref - back_turn_range.second, 0.0);
      if (dist_since_turn_end <= kExitRecoverDist) {
        const double ratio = std::clamp(
            dist_since_turn_end / kExitRecoverDist, 0.0, 1.0);
        factor = std::min(factor, 0.5 + 0.5 * ratio);
        is_point_in_turning_ = true;
      }
    }

    factor = std::clamp(factor, 0.5, 1.0);
    const double adjusted =
        std::clamp(origin_cruise_v * factor, kMinCruiseV, std::max(kMinCruiseV, origin_cruise_v));
    cruise_v = adjusted;
  }

  double ref_len_based_on_speed = 0.0;
  double span_t = config_.delta_t * config_.num_step;
  if (ego_v < cruise_v) {
    double t = (cruise_v - ego_v) / kMaxAcc;
    if (t > span_t) {
      ref_len_based_on_speed = ego_v * span_t + 0.5 * kMaxAcc * span_t * span_t;
    } else {
      ref_len_based_on_speed = ego_v * t + 0.5 * kMaxAcc * t * t;
      ref_len_based_on_speed += (span_t - t) * cruise_v;
    }
  } else {
    double t = (cruise_v - ego_v) / kMinAcc;
    if (t > span_t) {
      ref_len_based_on_speed = ego_v * span_t + 0.5 * kMinAcc * span_t * span_t;
    } else {
      ref_len_based_on_speed = ego_v * t + 0.5 * kMinAcc * t * t;
      ref_len_based_on_speed += (span_t - t) * cruise_v;
    }
  }

  const auto &cart_ref_info = coarse_planning_info.cart_ref_info;
  constexpr double kStraightCheckLength = 20.0;
  double ref_len_based_on_straight = 20.0;
  if (!cart_ref_info.s_vec.empty() && cart_ref_info.s_vec.back() > s_ref) {
    const double s_max =
        std::min(cart_ref_info.s_vec.back(), frenet_coord->Length());
    const double s_end = std::min(s_ref + kStraightCheckLength, s_max);
    if (s_end > s_ref + 1e-3) {
      ref_len_based_on_straight = std::max(s_end - s_ref, 0.0);

      if (static_analysis_storage) {
        const QueryTypeInfo turn_query(CRoadType::Turn, CPassageType::Ignore,
                                       CElemType::Ignore);
        const auto front_turn_range =
            static_analysis_storage->GetFrontSRange(turn_query, s_ref);

        constexpr double kTurnInnerPreview = 12.0;
        constexpr double kExitRecoverDist = 10.0;

        const double min_len_base_straight = 12.0;

        if (is_point_in_turning_) {  // 当前位于弯道内
          double ref_len_in_turn = -1.0;

          bool is_in_turn = s_ref > front_turn_range.first &&
                            s_ref < front_turn_range.second;

          if (is_in_turn) {
            ref_len_in_turn = kTurnInnerPreview;
          } else if (front_turn_range.second > front_turn_range.first &&
                      s_ref < front_turn_range.first) {
            auto t = std::clamp(
                (kPreviewBeforeTurn - front_turn_range.first + s_ref) /
                    kPreviewBeforeTurn, 0.0, 1.0);
            ref_len_in_turn = kTurnInnerPreview + (1.0 - t) *
                (kStraightCheckLength - kTurnInnerPreview);
          } else {
            const auto &back_turn_range =
                static_analysis_storage->GetBackSRange(turn_query, s_ref);
            auto t = std::clamp((s_ref - back_turn_range.second) /
                                  kExitRecoverDist, 0.0, 1.0);
            ref_len_in_turn = kTurnInnerPreview + t *
                (kStraightCheckLength - kTurnInnerPreview);
          }
          ref_len_based_on_straight =
              std::max(min_len_base_straight, ref_len_in_turn);
        } else {
          const auto back_turn_range =
              static_analysis_storage->GetBackSRange(turn_query, s_ref);
          if (back_turn_range.second > back_turn_range.first &&
              back_turn_range.second <= s_ref) {
            const double dist_since_turn_end =
                std::max(s_ref - back_turn_range.second, 0.0);
            if (dist_since_turn_end <= kExitRecoverDist) {
              const double recover_ratio = std::clamp(
                  dist_since_turn_end / kExitRecoverDist, 0.0, 1.0);
              const double ref_len_recover =
                  kTurnInnerPreview +
                  recover_ratio * (kStraightCheckLength - kTurnInnerPreview);
              ref_len_based_on_straight =
                  std::max(min_len_base_straight, ref_len_recover);
            }
          }
        }
      }
    }
  }

  double ref_len_based_on_target_slot = std::numeric_limits<double>::max();
  if (parking_slot_manager->IsExistTargetSlot()) {
    const auto target_slot_point = parking_slot_manager->GetTargetSlotCenter();
    double target_slot_s, target_slot_l;
    if(frenet_coord->XYToSL(target_slot_point.x(), target_slot_point.y(), &target_slot_s, &target_slot_l)) {
      ref_len_based_on_target_slot = target_slot_s - frenet_init_pt.x;
    }
  }

  const double ref_len_based_on_ref_info =
      std::min(cart_ref_info.s_vec.back(), frenet_coord->Length()) - s_ref;

  const double ref_len_based_on_guide_or_slot =
      std::min(ref_len_based_on_target_slot, ref_len_based_on_ref_info);

  lon_sample_length_ =
      std::min(ref_len_based_on_guide_or_slot,
               std::max(ref_len_based_on_speed, ref_len_based_on_straight));
}

void HppGeneralLateralDecider::ConstructReferencePathPoints() {
  CalculateLonSampleLength();

  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  const auto &cart_ref_info = coarse_planning_info.cart_ref_info;
  const auto &frenet_coord = reference_path_ptr_->get_frenet_coord();
  const auto &planning_init_point = ego_cart_state_manager_->planning_init_point();
  Eigen::Vector2d cart_init_point(planning_init_point.lat_init_state.x(),
                                  planning_init_point.lat_init_state.y());
  Point2D frenet_init_pt{planning_init_point.frenet_state.s, 0.0};
  Point2D cart_init_pt(cart_init_point.x(), cart_init_point.y());
  if (!frenet_coord->XYToSL(cart_init_pt, frenet_init_pt)) {
    frenet_init_pt.x = planning_init_point.frenet_state.s;
  }
  double s_ref = frenet_init_pt.x;

  const auto &cruise_v = session_->planning_context().v_ref_cruise();

  const double avg_dis = lon_sample_length_ / config_.num_step;

  const double avg_vel = lon_sample_length_ / (config_.delta_t * config_.num_step);

  const double kMaxAcc = cruise_v > avg_vel ? 0.8 : -5.5;

  std::vector<double> ref_sample_vel_vec(config_.num_step + 1, avg_vel);
  std::vector<double> delta_s_vec(config_.num_step + 1, avg_dis);

  if (!is_point_in_turning_ && avg_vel > 2.0) {
  // if (0) {
    ref_sample_vel_vec[0] = avg_vel;
    double total_s = 0.0;

    double t_reach = (cruise_v - avg_vel) / kMaxAcc;

    for (size_t i = 0; i < config_.num_step; ++i) {
      const double t_next = static_cast<double>(i + 1) * config_.delta_t;

      auto &v_curr = ref_sample_vel_vec[i];
      auto &v_next = ref_sample_vel_vec[i + 1];

      if (t_reach > t_next) {
        v_next = v_curr + kMaxAcc * config_.delta_t;
      } else {
        v_next = cruise_v;
      }

      if (t_reach < t_next && t_reach > t_next - config_.delta_t) {
        delta_s_vec[i] =
            0.5 * (v_curr + v_next) * (t_reach - t_next + config_.delta_t)
            + v_next * (t_next - t_reach);
      } else {
        delta_s_vec[i] = 0.5 * (v_curr + v_next) * config_.delta_t;
      }
      total_s += delta_s_vec[i];
    }

    const double scale = lon_sample_length_ / std::max(total_s, 1e-6);
    for (size_t i = 0; i < config_.num_step; ++i) {
      delta_s_vec[i] *= scale;
      ref_sample_vel_vec[i] *= scale;
    }
  }
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double half_ego_width = vehicle_param.max_width * 0.5;
  const auto &motion_planner_output =
      session_->planning_context().motion_planner_output();

  std::vector<double> left_outline_s;
  std::vector<double> left_outline_l;
  std::vector<double> right_outline_s;
  std::vector<double> right_outline_l;
  left_outline_s.reserve(config_.num_step + 4);
  left_outline_l.reserve(config_.num_step + 4);
  right_outline_s.reserve(config_.num_step + 4);
  right_outline_l.reserve(config_.num_step + 4);

  ref_traj_points_.clear();
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

    Point2D frenet_pt{0.0, 0.0};
    Point2D cart_pt(point.x, point.y);
    frenet_coord->XYToSL(cart_pt, frenet_pt);
    point.s = frenet_pt.x;
    point.l = frenet_pt.y;
    point.t = static_cast<double>(i) * config_.delta_t;

    point.v = ref_sample_vel_vec[i];

    if (i < config_.num_step) {
      s_ref += delta_s_vec[i];
    }
    ref_traj_points_.emplace_back(point);

    if (i == 0) {
      for (size_t j = 3; j > 0; --j) {
        left_outline_s.emplace_back(point.s - 0.3 * j);
        left_outline_l.emplace_back(half_ego_width);
        right_outline_s.emplace_back(point.s - 0.3 * j);
        right_outline_l.emplace_back(-half_ego_width);
      }
    }

    if (config_.enable_last_lat_path && !motion_planner_output.s_lat_vec.empty()) {
      const double tmp_t = std::fmin(0.1 + i * 0.2, 5.0);
      const double last_lat_path_x = motion_planner_output.lateral_x_t_spline(tmp_t);
      const double last_lat_path_y = motion_planner_output.lateral_y_t_spline(tmp_t);
      double last_lat_path_s = point.s;
      double last_lat_path_l = 0.0;
      if (!frenet_coord->XYToSL(last_lat_path_x, last_lat_path_y, &last_lat_path_s,
                                &last_lat_path_l)) {
        last_lat_path_l = 0.0;
      }

      double last_path_theta = motion_planner_output.lateral_theta_t_spline(tmp_t);
      const double theta_err = point.heading_angle - last_path_theta;
      if (theta_err > M_PI) {
        last_path_theta += 2.0 * M_PI;
      } else if (theta_err < -M_PI) {
        last_path_theta -= 2.0 * M_PI;
      }

      const double ego_center_x =
          last_lat_path_x + std::cos(last_path_theta) * vehicle_param.rear_axle_to_center;
      const double ego_center_y =
          last_lat_path_y + std::sin(last_path_theta) * vehicle_param.rear_axle_to_center;
      const Box2d ego_box({ego_center_x, ego_center_y}, last_path_theta,
                          vehicle_param.length, vehicle_param.max_width);

      std::pair<double, double> ego_lbuffer{last_lat_path_s, 0.0};
      std::pair<double, double> ego_rbuffer{last_lat_path_s, 0.0};
      for (auto &pt : ego_box.GetAllCorners()) {
        Point2D frenet_corner, cart_corner;
        cart_corner.x = pt.x();
        cart_corner.y = pt.y();
        if (frenet_coord->XYToSL(cart_corner, frenet_corner) &&
            frenet_corner.x > last_lat_path_s) {
          const double rel_corner_l = frenet_corner.y - last_lat_path_l;
          if (ego_lbuffer.second < rel_corner_l) {
            ego_lbuffer.first = frenet_corner.x;
            ego_lbuffer.second = rel_corner_l;
          }
          if (ego_rbuffer.second > rel_corner_l) {
            ego_rbuffer.first = frenet_corner.x;
            ego_rbuffer.second = rel_corner_l;
          }
        }
      }
      ego_lbuffer.second = std::max(ego_lbuffer.second, half_ego_width);
      ego_rbuffer.second = std::min(ego_rbuffer.second, -half_ego_width);

      if ((ego_lbuffer.first > left_outline_s.back()) &&
          (ego_rbuffer.first > right_outline_s.back())) {
        left_outline_s.emplace_back(ego_lbuffer.first);
        left_outline_l.emplace_back(ego_lbuffer.second);
        right_outline_s.emplace_back(ego_rbuffer.first);
        right_outline_l.emplace_back(ego_rbuffer.second);
      }
    } else {
      const double ego_yaw = point.heading_angle;
      const double ego_center_x =
          point.x + std::cos(ego_yaw) * vehicle_param.rear_axle_to_center;
      const double ego_center_y =
          point.y + std::sin(ego_yaw) * vehicle_param.rear_axle_to_center;
      const Box2d ego_box({ego_center_x, ego_center_y}, ego_yaw, vehicle_param.length,
                          vehicle_param.max_width);

      std::pair<double, double> ego_lbuffer{point.s, half_ego_width};
      std::pair<double, double> ego_rbuffer{point.s, -half_ego_width};
      for (auto &pt : ego_box.GetAllCorners()) {
        Point2D frenet_corner, cart_corner;
        cart_corner.x = pt.x();
        cart_corner.y = pt.y();
        if (frenet_coord->XYToSL(cart_corner, frenet_corner) &&
            frenet_corner.x > point.s) {
          if (frenet_corner.y > ego_lbuffer.second) {
            ego_lbuffer.first = frenet_corner.x;
            ego_lbuffer.second = frenet_corner.y;
          }
          if (frenet_corner.y < ego_rbuffer.second) {
            ego_rbuffer.first = frenet_corner.x;
            ego_rbuffer.second = frenet_corner.y;
          }
        }
      }

      if ((ego_lbuffer.first > left_outline_s.back()) &&
          (ego_rbuffer.first > right_outline_s.back())) {
        left_outline_s.emplace_back(ego_lbuffer.first);
        left_outline_l.emplace_back(ego_lbuffer.second);
        right_outline_s.emplace_back(ego_rbuffer.first);
        right_outline_l.emplace_back(ego_rbuffer.second);
      }
    }
  }

  if (left_outline_s.empty() || right_outline_s.empty()) {
    ILOG_ERROR << "no ref_traj_points!";
    return;
  }
  left_outline_s.emplace_back(left_outline_s.back() + 5.0);
  left_outline_l.emplace_back(half_ego_width);
  right_outline_s.emplace_back(right_outline_s.back() + 5.0);
  right_outline_l.emplace_back(-half_ego_width);
  lbuffer_s_spline_.set_points(left_outline_s, left_outline_l,
                               pnc::mathlib::spline::linear);
  rbuffer_s_spline_.set_points(right_outline_s, right_outline_l,
                               pnc::mathlib::spline::linear);

  auto &general_lateral_decider_output =
      session_->mutable_planning_context()
          ->mutable_general_lateral_decider_output();

  general_lateral_decider_output.ramp_scene = false;
  general_lateral_decider_output.complete_follow = false;
  general_lateral_decider_output.lane_change_scene = false;
  if (config_.enable_ara_ref) {
    general_lateral_decider_output.enable_ara_ref = HandleAraPath(ref_traj_points_);
  } else {
    general_lateral_decider_output.enable_ara_ref = false;
  }

  ref_path_points_.clear();

  const double reference_end_s = reference_path_ptr_->get_points().back().path_point.s();
  ref_path_points_.reserve(ref_traj_points_.size());
  for (const auto &traj_point : ref_traj_points_) {
    ReferencePathPoint refpath_pt{};
    const double point_s = std::min(traj_point.s, reference_end_s);
    if (!reference_path_ptr_->get_reference_point_by_lon(point_s,
                                                          refpath_pt)) {
      ILOG_ERROR
          << "ConstructReferencePathPointsFromTrajPoints: Get reference point by lon failed!";
    }
    const double road_radius =
        1 / std::max(std::fabs(refpath_pt.path_point.kappa()), 1e-6);
    min_road_radius_ = std::max(
        std::min(road_radius - 1.0, min_road_radius_), 0.2);
    ref_path_points_.emplace_back(refpath_pt);
  }

  if (!ref_traj_points_.empty()) {
    ReferencePathPoint refpath_front_pt{};
    const double front_point_s_1 = std::min(ref_traj_points_.back().s + 1.0,
                                             reference_end_s);
    if (reference_path_ptr_->get_reference_point_by_lon(front_point_s_1,
                                                         refpath_front_pt)) {
      const double road_radius =
          1 / std::max(std::fabs(refpath_front_pt.path_point.kappa()),
                       1e-6);
      min_road_radius_ = std::max(
          std::min(road_radius - 1.0, min_road_radius_), 0.2);
    }
  }

  ref_traj_points_.resize(config_.num_step + 1);
  bound_center_line_.resize(config_.num_step + 1);

  TrajectoryPoints plan_history_traj_tmp;
  const TrajectoryPoints &last_hpp_lateral_motion_traj =
      session_->planning_context().last_hpp_lateral_motion_traj();
  if (!last_hpp_lateral_motion_traj.empty()) {
    for (size_t i = 0; i < last_hpp_lateral_motion_traj.size(); ++i) {
      TrajectoryPoint pt = last_hpp_lateral_motion_traj[i];
      Point2D frenet_pt{0.0, 0.0};
      Point2D cart_pt(pt.x, pt.y);
      if (frenet_coord->XYToSL(cart_pt, frenet_pt)) {
        pt.s = frenet_pt.x;
        pt.l = frenet_pt.y;
        plan_history_traj_tmp.emplace_back(std::move(pt));
      } else {
        LOG_DEBUG("plan_history_traj frenet error");
      }
    }
  } else {
    auto &last_traj_points = session_->mutable_planning_context()
                                 ->mutable_last_planning_result()
                                 .raw_traj_points;
    for (size_t i = 0; i < last_traj_points.size(); ++i) {
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
  }
  if (plan_history_traj_tmp.empty()) {
    LOG_DEBUG("plan_history_traj_tmp is empty");
    return;
  }

  for (size_t i = 0; i < ref_traj_points_.size(); ++i) {
    TrajectoryPoint pt;
    if (!hpp_general_lateral_decider_utils::GetTrajectoryPointAtS(
            plan_history_traj_tmp, ref_traj_points_[i].s, pt)) {
      // continue;
    }
    // pt.s = pt.s - (ego_s - plan_history_traj_tmp.front().s);
    plan_history_traj_.emplace_back(std::move(pt));
  }

  auto t_delay = plan_history_traj_.front().t;

  if (t_delay < -1e-2) {
    for (size_t i = 0; i < ref_traj_points_.size(); ++i) {
      plan_history_traj_[i].t -= t_delay;
    }
  } else if (t_delay > 1e-2) {
    for (size_t i = 0; i < ref_traj_points_.size(); ++i) {
      plan_history_traj_[i].t -= t_delay;
    }
  }



  for (int i = 0; i < plan_history_traj_.size(); i++) {
    const auto &history_traj_point = plan_history_traj_[i];
    double plan_history_traj_point_s = history_traj_point.s;
    std::vector<int> match_indexes =
        hpp_general_lateral_decider_utils::MatchRefTrajPoints(
            plan_history_traj_point_s, ref_traj_points_);
    match_index_map_[i] = std::move(match_indexes);
  }
}

bool HppGeneralLateralDecider::HandleAraPath(TrajectoryPoints &traj_points) {
  auto &hybrid_ara_result = session_->mutable_planning_context()
                                ->mutable_lateral_obstacle_decider_output()
                                .hybrid_ara_result;
  const auto &search_result = session_->mutable_planning_context()
                                  ->lateral_obstacle_decider_output()
                                  .search_result;

  const auto &frenet_coord = reference_path_ptr_->get_frenet_coord();
  const PlanningInitPoint &planning_init_point = session_->environmental_model()
                                                     .get_ego_state_manager()
                                                     ->planning_init_point();
  if (search_result != SearchResult::SUCCESS ||
      hybrid_ara_result.s.back() <= traj_points.back().s) {
    return false;
  }

  double ego_s = 0.0;
  double ego_l = 0.0;
  if (!frenet_coord->XYToSL(planning_init_point.x, planning_init_point.y,
                            &ego_s, &ego_l)) {
    std::cout << "General Lateral Decider: planning_init_point frenet failed!!!"
              << std::endl;
    return false;
  }

  auto traj_size = hybrid_ara_result.x.size();
  std::vector<double> x_vec(traj_size);
  std::vector<double> y_vec(traj_size);
  std::vector<double> theta_vec(traj_size);
  std::vector<double> s_vec(traj_size);
  std::vector<double> l_vec(traj_size);
  double angle_offset = 0.0;
  bool behind_equal_l_point = true;

  double kFilterLBuffer = 0.0;
  // 直道or弯道，0表示关闭过滤细微扰动
  if (session_->planning_context()
          .lane_change_decider_output()
          .hpp_turn_signal == NO_CHANGE) {
    kFilterLBuffer = 0.0;  // 0.28
  } else {
    kFilterLBuffer = 0.0;
  }
  // 过滤细微扰动
  bool find_first_ploe = false;
  bool monotonic = true;
  if (hybrid_ara_result.l[0] < kFilterLBuffer) {
    for (size_t i = 1; i < traj_size; ++i) {
      if (hybrid_ara_result.l[i] > kFilterLBuffer) {
        find_first_ploe = true;
      }
      if (find_first_ploe && hybrid_ara_result.l[i] < kFilterLBuffer) {
        monotonic = false;
        break;
      }
    }
  } else {
    for (size_t i = 1; i < traj_size; ++i) {
      if (hybrid_ara_result.l[i] < kFilterLBuffer) {
        find_first_ploe = true;
      }
      if (find_first_ploe && hybrid_ara_result.l[i] > kFilterLBuffer) {
        monotonic = false;
        break;
      }
    }
  }
  if (monotonic) {
    for (size_t i = 0; i < traj_size; ++i) {
      if (std::fabs(hybrid_ara_result.l[i]) < kFilterLBuffer) {
        hybrid_ara_result.l[i] = 0;
      }
    }
  }

  for (size_t i = 0; i < traj_size; ++i) {
    if (!frenet_coord->SLToXY(hybrid_ara_result.s[i], hybrid_ara_result.l[i],
                              &hybrid_ara_result.x[i],
                              &hybrid_ara_result.y[i])) {
      std::cout << "General Lateral Decider: SLToXY failed!!!" << std::endl;
      return false;
    }

    x_vec[i] = hybrid_ara_result.x[i];
    y_vec[i] = hybrid_ara_result.y[i];
    s_vec[i] = hybrid_ara_result.s[i];
    l_vec[i] = hybrid_ara_result.l[i];
    if (i == 0) {
      theta_vec[i] = hybrid_ara_result.phi[i];
    } else {
      const auto delta_theta =
          hybrid_ara_result.phi[i] - hybrid_ara_result.phi[i - 1];
      if (delta_theta > 1.5 * M_PI) {
        angle_offset -= 2.0 * M_PI;
      } else if (delta_theta < -1.5 * M_PI) {
        angle_offset += 2.0 * M_PI;
      }
      theta_vec[i] = hybrid_ara_result.phi[i] + angle_offset;
    }
  }


  pnc::mathlib::spline x_s_spline;
  pnc::mathlib::spline y_s_spline;
  pnc::mathlib::spline l_s_spline;
  pnc::mathlib::spline theta_s_spline;
  x_s_spline.set_points(s_vec, x_vec, pnc::mathlib::spline::linear);
  y_s_spline.set_points(s_vec, y_vec, pnc::mathlib::spline::linear);
  theta_s_spline.set_points(s_vec, theta_vec, pnc::mathlib::spline::linear);
  l_s_spline.set_points(s_vec, l_vec, pnc::mathlib::spline::linear);

  for (auto &traj_point : traj_points) {
    traj_point.l = l_s_spline(traj_point.s);
    traj_point.x = x_s_spline(traj_point.s);
    traj_point.y = y_s_spline(traj_point.s);
    // traj_point.heading_angle = theta_s_spline(traj_point.s);
    traj_point.heading_angle = std::atan2(y_s_spline.deriv(1, traj_point.s),
                                          x_s_spline.deriv(1, traj_point.s));
  }
  return true;
}

void HppGeneralLateralDecider::HandleAvoidScene(TrajectoryPoints &traj_points,
                                                double dynamic_ref_buffer) {
  const auto &frenet_coord = reference_path_ptr_->get_frenet_coord();

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

void HppGeneralLateralDecider::UpdateDistanceToRoadBorder() {
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
void HppGeneralLateralDecider::GenerateRoadAndLaneBoundary() {
  UpdateDistanceToRoadBorder();
  GenerateRoadHardSoftBoundary();
  GenerateLaneSoftBoundary();
  if (!session_->is_nsa_scene()) {
    GenerateGroundLineAndParkingSpaceBoundary();
  }
}

void HppGeneralLateralDecider::GenerateGroundLineAndParkingSpaceBoundary() {
  const double kDefaultDistanceToRoad = 10.0;

  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double rear_axle_to_front_bumper =  // TBD：define as consexpr
      vehicle_param.length - vehicle_param.rear_edge_to_rear_axle;
  const double half_ego_width = vehicle_param.max_width * 0.5;
  const auto &l_care_width = config_.l_care_width;
  double front_lon_buf_dis = 1.0;
  double rear_lon_buf_dis = 1.0;

  std::array<std::vector<std::pair<int, Polygon2d>>, 2> groundline_polygons;
  std::array<std::vector<std::pair<int, Polygon2d>>, 2> parking_space_polygons;

  ConstructStaticObstacleTotalPolygons(groundline_polygons,
                                       parking_space_polygons);

  // 后面根据实际情况，将parking-space与groundline逻辑合并
  for (size_t i = 0; i < ref_traj_points_.size(); i++) {
    auto &traj_point = ref_traj_points_[i];
    const auto &t = traj_point.t;
    const double ego_s = traj_point.s;
    const double ego_l = traj_point.l;

    double care_area_s_start =
        ego_s - vehicle_param.rear_edge_to_rear_axle - rear_lon_buf_dis;
    double care_area_s_end =
        ego_s + rear_axle_to_front_bumper + front_lon_buf_dis;
    if (is_ego_reverse_) {
      care_area_s_start = ego_s - rear_axle_to_front_bumper - rear_lon_buf_dis;
      care_area_s_end = ego_s + vehicle_param.rear_edge_to_rear_axle + front_lon_buf_dis;
    }
    const auto care_area_center =
        Vec2d((care_area_s_start + care_area_s_end) * 0.5, ego_l);
    const double care_area_length = care_area_s_end - care_area_s_start;
    const auto care_polygon =  // @cai: consider the heading
        Polygon2d(Box2d(care_area_center, 0, care_area_length, l_care_width));

    // double extra_soft_buffer = config_.extra_soft_buffer2groundline;
    // double extra_hard_buffer =
    // std::fabs(ref_path_points_[i].path_point.kappa) /
    //     config_.max_ref_curvature * 0.2;
    double extra_soft_buffer =
        std::max(std::fabs(ref_path_points_[i].path_point.kappa()) /
                     config_.max_ref_curvature,
                 config_.extra_soft_buffer2groundline);
    double extra_hard_buffer = config_.extra_hard_buffer2groundline;

    // Bound soft_bound_groundline{-kDefaultDistanceToRoad,
    // kDefaultDistanceToRoad}; Bound
    // hard_bound_groundline{-kDefaultDistanceToRoad, kDefaultDistanceToRoad};
    Bound soft_bound_parking_space{-kDefaultDistanceToRoad,
                                   kDefaultDistanceToRoad};
    // Bound hard_bound_parking_space{-kDefaultDistanceToRoad,
    // kDefaultDistanceToRoad};
    for (int direction = 0; direction < 2; direction++) {
      bool is_left = direction == 0;
      ObstacleBorderInfo parking_space_border =
          hpp_general_lateral_decider_utils::GetNearestObstacleBorder(
              care_polygon, care_area_s_start, care_area_s_end,
              parking_space_polygons[direction], is_left, false, false, i,
              ref_traj_points_);
      if (is_left) {
        soft_bound_parking_space.upper =
            std::fmin(soft_bound_parking_space.upper,
                      parking_space_border.obstacle_border - half_ego_width -
                          extra_soft_buffer);

        second_soft_bounds_[i].emplace_back(WeightedBound{
            -kDefaultDistanceToRoad, soft_bound_parking_space.upper,
            config_.kPhysicalBoundWeight,
            BoundInfo{parking_space_border.obstacle_id,
                      BoundType::PARKING_SPACE}});
      } else {
        soft_bound_parking_space.lower =
            std::fmax(soft_bound_parking_space.lower,
                      parking_space_border.obstacle_border + half_ego_width +
                          extra_soft_buffer);

        second_soft_bounds_[i].emplace_back(
            WeightedBound{soft_bound_parking_space.lower,
                          kDefaultDistanceToRoad, config_.kPhysicalBoundWeight,
                          BoundInfo{parking_space_border.obstacle_id,
                                    BoundType::PARKING_SPACE}});
      }
    }
  }
}

void HppGeneralLateralDecider::GenerateRoadHardSoftBoundary() {
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double left_road_extra_buffer, right_road_extra_buffer;
  GetDesireRoadExtraBuffer(&left_road_extra_buffer, &right_road_extra_buffer);

  const double kDefaultDistanceToRoad = 10.0;
  min_road_radius_ = std::min(kDefaultDistanceToRoad, min_road_radius_);
  hard_bounds_.resize(ref_traj_points_.size());
  second_soft_bounds_.resize(ref_traj_points_.size());
  for (size_t i = 0; i < ref_traj_points_.size(); i++) {
    Bound soft_bound_road{-min_road_radius_, min_road_radius_};
    Bound hard_bound_road{-min_road_radius_, min_road_radius_};
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
    second_soft_bounds_[i].emplace_back(WeightedBound{
        soft_bound_road.lower, soft_bound_road.upper,
        config_.kPhysicalBoundWeight, BoundInfo{-100, BoundType::ROAD_BORDER}});
  }
}

void HppGeneralLateralDecider::GenerateLaneSoftBoundary() {
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
        second_soft_bounds_[i].emplace_back(
            WeightedBound{ego_init_sl_info.min_l + half_ego_width,
                          kDefaultDistanceToRoad, config_.kPhysicalBoundWeight,
                          BoundInfo{-100, BoundType::EGO_POSITION}});
        second_soft_bounds_[i].emplace_back(WeightedBound{
            -kDefaultDistanceToRoad, soft_bound_lane.upper,
            config_.kPhysicalBoundWeight, BoundInfo{-100, BoundType::LANE}});
      } else {
        second_soft_bounds_[i].emplace_back(WeightedBound{
            soft_bound_lane.lower, soft_bound_lane.upper,
            config_.kPhysicalBoundWeight, BoundInfo{-100, BoundType::LANE}});
      }
    } else if (is_lane_change &&
               lc_request_direction == RequestType::RIGHT_CHANGE) {
      if (ego_init_sl_info.max_l - half_ego_width > soft_bound_lane.upper) {
        second_soft_bounds_[i].emplace_back(WeightedBound{
            -kDefaultDistanceToRoad, ego_init_sl_info.max_l - half_ego_width,
            config_.kPhysicalBoundWeight,
            BoundInfo{-100, BoundType::EGO_POSITION}});
        second_soft_bounds_[i].emplace_back(WeightedBound{
            soft_bound_lane.lower, kDefaultDistanceToRoad,
            config_.kPhysicalBoundWeight, BoundInfo{-100, BoundType::LANE}});
      } else {
        second_soft_bounds_[i].emplace_back(WeightedBound{
            soft_bound_lane.lower, soft_bound_lane.upper,
            config_.kPhysicalBoundWeight, BoundInfo{-100, BoundType::LANE}});
      }
    } else {
      second_soft_bounds_[i].emplace_back(WeightedBound{
          soft_bound_lane.lower, soft_bound_lane.upper,
          config_.kPhysicalBoundWeight, BoundInfo{-100, BoundType::LANE}});
    }
  }
}

void HppGeneralLateralDecider::GetDesireRoadExtraBuffer(
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

void HppGeneralLateralDecider::GetLateralTTCToRoad(
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

void HppGeneralLateralDecider::GenerateObstaclesBoundary() {
  if (ref_traj_points_.empty() || ref_path_points_.empty()) {
    // add logs
    ILOG_INFO << "Ref traj points or ref path points is null!";
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

  if (plan_history_traj_.empty()) {
    // add logs
    ILOG_INFO << "lan history traj is null!";
    return;
  }
  GenerateDynamicObstaclesBoundary(dynamic_obstacles,
                                   dynamic_obstacle_decisions_);
}

void HppGeneralLateralDecider::GenerateStaticObstaclesBoundary(
    const std::vector<std::shared_ptr<FrenetObstacle>>& obs_vec,
    ObstacleDecisions &obstacle_decisions) {
  for (auto &obstacle : obs_vec) {
    if (!IsFilterForStaticObstacle(obstacle)) {
      continue;
    }

    Polygon2d obstacle_sl_polygon;
    if (!obstacle->get_polygon_at_time_tmp(0, reference_path_ptr_,
                                           obstacle_sl_polygon)) {
      continue;
    }

    const auto &obstacle_id = obstacle->id();
    auto obstacle_decision = ObstacleDecision{obstacle_id, {}, {}};

    GenerateStaticObstacleDecision(obstacle, obstacle_sl_polygon,
                                   obstacle_decision, true);
    GenerateStaticObstacleDecision(obstacle, obstacle_sl_polygon,
                                   obstacle_decision, false);

    ExtractStaticObstacleBound(obstacle_decision);
    obstacle_decisions[obstacle_id] = std::move(obstacle_decision);
  }
}

void HppGeneralLateralDecider::GenerateStaticObstacleDecision(
    const std::shared_ptr<FrenetObstacle> obstacle,
    const Polygon2d &obstacle_sl_polygon,
    ObstacleDecision &obstacle_decision, bool is_update_hard_bound) {
  using namespace planning_math;
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto &lat_obstacle_decision = session_->mutable_planning_context()
                                          ->lateral_obstacle_decider_output()
                                          .lat_obstacle_decision;
  // Step 1) configs
  const auto &l_care_width = config_.l_care_width;

  const auto &ego_cur_s = ref_traj_points_.front().s;

  const double rear_axle_to_front_bumper =  // TBD：define as consexpr
      vehicle_param.length - vehicle_param.rear_edge_to_rear_axle;
  const double ego_cur_s_start =
      ego_cur_s - vehicle_param.rear_edge_to_rear_axle;
  const double ego_cur_s_end = ego_cur_s + rear_axle_to_front_bumper;

  double front_lon_buf_dis = 0.0;
  double rear_lon_buf_dis = 1.0;
  if (!is_update_hard_bound) {
    front_lon_buf_dis = hpp_general_lateral_decider_utils::CalDesireLonDistance(
        ego_frenet_state_.velocity_s(), obstacle->frenet_velocity_s(), 0.0);
  }
  auto pre_lateral_decision = LatObstacleDecisionType::IGNORE;

  const bool init_lon_no_overlap =
      (obstacle->frenet_obstacle_boundary().s_end < ego_cur_s_start ||
       obstacle->frenet_obstacle_boundary().s_start > ego_cur_s_end);

  bool reset_conflict_decision{false};

  bool is_nudge_left = lat_obstacle_decision.at(obstacle->id()) ==
                       LatObstacleDecisionType::RIGHT;
  const BoundType bound_type = BoundType::AGENT;

  bool is_cross_obj{false};
  bool has_lat_decision{false};
  bool has_lon_decision{false};

  const ConstStaticAnalysisStoragePtr static_storage =
      reference_path_ptr_->get_static_analysis_storage();
  const double ego_v_for_road = ego_cart_state_manager_ != nullptr
                                    ? ego_cart_state_manager_->ego_v()
                                    : 0.0;
  double ref_s_length = ref_traj_points_.back().s;
  constexpr double kCareStaticObjectSThreshold = 25.0;

  const double obs_s_min = obstacle_sl_polygon.min_x();
  const double obs_s_max = obstacle_sl_polygon.max_x();
  const double obs_l_min = obstacle_sl_polygon.min_y();
  const double obs_l_max = obstacle_sl_polygon.max_y();
  const double half_l_care = l_care_width * 0.5;

  for (size_t i = 0; i < ref_traj_points_.size(); i++) {
    auto &traj_point = ref_traj_points_[i];
    const auto &t = traj_point.t;
    if (t > config_.care_static_object_t_threshold) {
      continue;
    }

    const double ego_s = traj_point.s;
    const double ego_l = traj_point.l;
    double care_area_s_start =
        ego_s - vehicle_param.rear_edge_to_rear_axle - rear_lon_buf_dis;
    double care_area_s_end =
        ego_s + rear_axle_to_front_bumper + front_lon_buf_dis;

    if (is_ego_reverse_) {
      care_area_s_start = ego_s - rear_axle_to_front_bumper - rear_lon_buf_dis;
      care_area_s_end = ego_s + vehicle_param.rear_edge_to_rear_axle + front_lon_buf_dis;
    }

    if (care_area_s_start > obs_s_max) {
      break;
    }
    if (care_area_s_end < obs_s_min ||
        ego_l - half_l_care > obs_l_max ||
        ego_l + half_l_care < obs_l_min) {
      continue;
    }

    const auto care_area_center =
        Vec2d((care_area_s_start + care_area_s_end) * 0.5, ego_l);
    const double care_area_length = care_area_s_end - care_area_s_start;
    const auto care_polygon =
        Polygon2d(Box2d(care_area_center, 0, care_area_length, l_care_width));

    Polygon2d care_overlap_polygon;
    bool b_overlap_with_care =
        obstacle_sl_polygon.ComputeOverlap(care_polygon, &care_overlap_polygon);
    if (!b_overlap_with_care) {
      continue;
    }

    double lat_buf_dis =
        hpp_general_lateral_decider_utils::CalDesireStaticLateralDistance(
            config_.hard_buffer2static_agent, ego_cart_state_manager_->ego_v(),
            ego_frenet_state_.l(), obstacle->type(), is_update_hard_bound, config_);

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
        ref_path_points_[i].distance_to_left_lane_border, care_overlap_polygon,
        lat_buf_dis, b_overlap_side, init_lon_no_overlap, is_nudge_left,
        is_cross_obj, pre_lateral_decision, reset_conflict_decision,
        obstacle_decision, lat_decision, lon_decision);
    has_lat_decision =
        has_lat_decision || lat_decision != LatObstacleDecisionType::IGNORE;
    has_lon_decision =
        has_lon_decision || lon_decision != LonObstacleDecisionType::IGNORE;

    AddObstacleDecisionBound(obstacle->id(), t, bound_type,
                             care_overlap_polygon, lat_buf_dis, lat_decision,
                             lon_decision, obstacle_decision, true,
                             is_update_hard_bound);
  }
}

bool HppGeneralLateralDecider::IsCutoutSideObstacle(
    const std::shared_ptr<FrenetObstacle> obstacle,
    Polygon2d &care_overlap_polygon) {
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
    auto ok = obstacle->get_polygon_at_time_tmp(0, reference_path_ptr_,
                                                obstacle_sl_polygon);
    if (!ok) {
      // TBD add log
      return false;
    }
    if (!obstacle_sl_polygon.ComputeOverlap(care_polygon,
                                            &care_overlap_polygon)) {
      return false;
    }
    if ((obstacle->frenet_l() * obstacle->frenet_velocity_l() > 0) &&
        fabs(obstacle->frenet_velocity_l()) > 0.3) {
      return true;
    }
  }
  return false;
}

double HppGeneralLateralDecider::CalculateExtraDecreaseBuffer(
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
  if (hpp_general_lateral_decider_utils::IsTruck(obstacle)) {
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

void HppGeneralLateralDecider::GenerateDynamicObstaclesBoundary(
    const std::vector<std::shared_ptr<FrenetObstacle>>& obs_vec,
    ObstacleDecisions &obstacle_decisions) {
  for (auto &obstacle : obs_vec) {
    if (!IsFilterForDynamicObstacle(obstacle)) {
      continue;
    }

    const auto &obstacle_id = obstacle->id();
    auto obstacle_decision = ObstacleDecision{obstacle_id, {}, {}};

    GenerateDynamicObstacleDecision(obstacle, obstacle_decision);
    ExtractDynamicObstacleBound(obstacle_decision);
    obstacle_decisions[obstacle_id] = std::move(obstacle_decision);
  }
}

void HppGeneralLateralDecider::GenerateDynamicObstacleDecision(
    const std::shared_ptr<FrenetObstacle> obstacle,
    ObstacleDecision &obstacle_decision) {
  using namespace planning_math;
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto &lat_obstacle_decision = session_->mutable_planning_context()
                                          ->lateral_obstacle_decider_output()
                                          .lat_obstacle_decision;
  bool in_intersection = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->GetIntersectionState() ==
                         common::IntersectionState::IN_INTERSECTION;

  // Step 1) configs
  const auto &l_care_width = config_.l_care_width;

  const auto &ego_cur_s = plan_history_traj_.front().s;

  const double rear_axle_to_front_bumper =  // TBD：define as consexpr
      vehicle_param.length - vehicle_param.rear_edge_to_rear_axle;
  const double ego_cur_s_start =
      ego_cur_s - vehicle_param.rear_edge_to_rear_axle;
  const double ego_cur_s_end = ego_cur_s + rear_axle_to_front_bumper;

  const double front_lon_buf_dis =
      hpp_general_lateral_decider_utils::CalDesireLonDistance(
          ego_frenet_state_.velocity_s(), obstacle->frenet_velocity_s(), 3.0);
  const double rear_lon_buf_dis = 1.0;
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

  bool is_nudge_left = lat_obstacle_decision.at(obstacle->id()) ==
                       LatObstacleDecisionType::RIGHT;

  bool is_cross_obj{false};
  bool has_lat_decision{false};
  bool has_lon_decision{false};

  // double limit_overlap_min_y = -1000;
  // double limit_overlap_max_y = 1000;
  // hack: consider that the obstacle is not completely over the car
  Polygon2d limit_care_overlap_polygon;
  bool is_cut_out_side_obstacle =
      IsCutoutSideObstacle(obstacle, limit_care_overlap_polygon);
  BoundType bound_type = BoundType::DYNAMIC_AGENT;
  if (is_cut_out_side_obstacle) {
    bound_type = BoundType::ADJACENT_AGENT;
  }

  double extra_decrease_buffer =
      CalculateExtraDecreaseBuffer(obstacle, is_nudge_left);

  for (size_t i = 0; i < plan_history_traj_.size(); i++) {
    auto &traj_point = plan_history_traj_[i];
    const auto &t = traj_point.t;
    if (t > config_.care_dynamic_object_t_threshold) {
      continue;
    }

    const double ego_s = traj_point.s;
    const double ego_l = traj_point.l;

    double care_area_s_start =
        ego_s - vehicle_param.rear_edge_to_rear_axle - rear_lon_buf_dis;
    double care_area_s_end =
        ego_s + rear_axle_to_front_bumper + front_lon_buf_dis;
    if (is_ego_reverse_) {
      care_area_s_start = ego_s - rear_axle_to_front_bumper - rear_lon_buf_dis;
      care_area_s_end = ego_s + vehicle_param.rear_edge_to_rear_axle + front_lon_buf_dis;
    }
    const auto care_area_center =
        Vec2d((care_area_s_start + care_area_s_end) * 0.5, ego_l);
    const double care_area_length = care_area_s_end - care_area_s_start;
    const auto care_polygon =  // @cai: consider the heading
        Polygon2d(Box2d(care_area_center, 0, care_area_length, l_care_width));

    Polygon2d obstacle_sl_polygon;
    auto ok = obstacle->get_polygon_at_time_tmp(
        i * config_.delta_t, reference_path_ptr_, obstacle_sl_polygon);
    if (!ok) {
      // TBD add log
      return;
    }
    Polygon2d care_overlap_polygon;
    bool b_overlap_with_care = false;
    // default: invalid value
    // double overlap_min_y = 100.0;
    // double overlap_max_y = -100.0;

    b_overlap_with_care =
        obstacle_sl_polygon.ComputeOverlap(care_polygon, &care_overlap_polygon);
    if (!b_overlap_with_care) {
      continue;
    }

    if (is_nudge_left) {
      if (is_cut_out_side_obstacle) {
        care_overlap_polygon =
            care_overlap_polygon.min_y() > limit_care_overlap_polygon.min_y()
                ? care_overlap_polygon
                : limit_care_overlap_polygon;
      }
    } else {
      if (is_cut_out_side_obstacle) {
        care_overlap_polygon =
            care_overlap_polygon.max_y() > limit_care_overlap_polygon.max_y()
                ? limit_care_overlap_polygon
                : care_overlap_polygon;
      }
    }

    double lat_buf_dis =
        hpp_general_lateral_decider_utils::CalDesireLateralDistance(
            ego_cart_state_manager_->ego_v(), t, 0, obstacle, is_nudge_left,
            in_intersection, config_);
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
    if (is_cut_out_side_obstacle || extra_decrease_buffer < 1e-5) {
      for (auto index : indexes) {
        GenerateObstaclePreliminaryDecision(
            ego_l, ref_path_points_[index].distance_to_right_lane_border,
            ref_path_points_[index].distance_to_left_lane_border,
            care_overlap_polygon, lat_buf_dis, b_overlap_side,
            init_lon_no_overlap, is_nudge_left, is_cross_obj,
            pre_lateral_decision, reset_conflict_decision, obstacle_decision,
            lat_decision, lon_decision);
        has_lat_decision =
            has_lat_decision || lat_decision != LatObstacleDecisionType::IGNORE;
        has_lon_decision =
            has_lon_decision || lon_decision != LonObstacleDecisionType::IGNORE;
      }
      AddObstacleDecisionBound(obstacle->id(), t, bound_type,
                               care_overlap_polygon, lat_buf_dis, lat_decision,
                               lon_decision, obstacle_decision, false);
    } else {
      for (int k = 0; k < 2; k++) {
        if (k == 0) {
          bound_type = BoundType::ADJACENT_AGENT;
        } else {
          lat_buf_dis = std::max(lat_buf_dis - extra_decrease_buffer, 0.0);
          bound_type = BoundType::DYNAMIC_AGENT;
        }
        for (auto index : indexes) {
          GenerateObstaclePreliminaryDecision(
              ego_l, ref_path_points_[index].distance_to_right_lane_border,
              ref_path_points_[index].distance_to_left_lane_border,
              care_overlap_polygon, lat_buf_dis, b_overlap_side,
              init_lon_no_overlap, is_nudge_left, is_cross_obj,
              pre_lateral_decision, reset_conflict_decision, obstacle_decision,
              lat_decision, lon_decision);
          has_lat_decision = has_lat_decision ||
                             lat_decision != LatObstacleDecisionType::IGNORE;
          has_lon_decision = has_lon_decision ||
                             lon_decision != LonObstacleDecisionType::IGNORE;
        }
        AddObstacleDecisionBound(
            obstacle->id(), t, bound_type, care_overlap_polygon, lat_buf_dis,
            lat_decision, lon_decision, obstacle_decision, false);
      }
    }
  }
}

void HppGeneralLateralDecider::GenerateObstaclePreliminaryDecision(
    double ego_l, double distance_to_right_lane_border,
    double distance_to_left_lane_border, const Polygon2d &overlap_polygon,
    double lat_buf_dis, bool b_overlap_side, bool init_lon_no_overlap,
    bool is_nudge_left, bool is_cross_obj,
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

void HppGeneralLateralDecider::AddObstacleDecisionBound(
    int id, double t, BoundType bound_type, const Polygon2d &overlap_polygon,
    double lat_buf_dis, LatObstacleDecisionType lat_decision,
    LonObstacleDecisionType lon_decision, ObstacleDecision &obstacle_decision,
    bool is_static, bool is_update_hard_bound) {
  const double l_offset_limit = 10.0;
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double half_ego_width = vehicle_param.max_width * 0.5;
  double ego_left_buffer = half_ego_width;
  double ego_right_buffer = half_ego_width;
  if (is_static) {
    if (!lbuffer_s_spline_.get_x().empty() &&
        !lbuffer_s_spline_.get_y().empty()) {
      Vec2d key_point = overlap_polygon.GetAllVertices()[0];
      for (const auto &vertice : overlap_polygon.GetAllVertices()) {
        if (vertice.y() < key_point.y()) {
          key_point = vertice;
        }
      }
      if (key_point.x() <= lbuffer_s_spline_.get_x_min()) {
        ego_left_buffer = lbuffer_s_spline_(lbuffer_s_spline_.get_x_min());
      } else if (key_point.x() >= lbuffer_s_spline_.get_x_max()) {
        ego_left_buffer = lbuffer_s_spline_(lbuffer_s_spline_.get_x_max());
      } else {
        ego_left_buffer = lbuffer_s_spline_(key_point.x());
      }
    }
    if (!rbuffer_s_spline_.get_x().empty() &&
        !rbuffer_s_spline_.get_y().empty()) {
      Vec2d key_point = overlap_polygon.GetAllVertices()[0];
      for (const auto &vertice : overlap_polygon.GetAllVertices()) {
        if (vertice.y() > key_point.y()) {
          key_point = vertice;
        }
      }
      if (key_point.x() <= rbuffer_s_spline_.get_x_min()) {
        ego_right_buffer = -rbuffer_s_spline_(rbuffer_s_spline_.get_x_min());
      } else if (key_point.x() >= rbuffer_s_spline_.get_x_max()) {
        ego_right_buffer = -rbuffer_s_spline_(rbuffer_s_spline_.get_x_max());
      } else {
        ego_right_buffer = -rbuffer_s_spline_(key_point.x());
      }
    }
  }

  BoundInfo bound_info;
  Bound bound{-l_offset_limit, l_offset_limit};

  if (lat_decision == LatObstacleDecisionType::LEFT) {
    bound.lower = overlap_polygon.max_y() + lat_buf_dis + ego_right_buffer;
    // soft_bound.lower = is_rear_obstacle ? std::max(0.0,
    // care_overlap_polygon.max_y() + lat_buf_dis + half_ego_width) :
    //                                       care_overlap_polygon.max_y() +
    //                                       lat_buf_dis + half_ego_width;
    bound_info.type = bound_type;
    bound_info.id = id;
  } else if (lat_decision == LatObstacleDecisionType::RIGHT) {
    bound.upper = overlap_polygon.min_y() - lat_buf_dis - ego_left_buffer;
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

bool HppGeneralLateralDecider::CheckObstacleNudgeDecision(
    const std::shared_ptr<FrenetObstacle> &obstacle) {
  auto &lat_obstacle_decision = session_->mutable_planning_context()
                                    ->lateral_obstacle_decider_output()
                                    .lat_obstacle_decision;
  if (lat_obstacle_decision.find(obstacle->id()) !=
      lat_obstacle_decision.end()) {
    if (lat_obstacle_decision.at(obstacle->id()) ==
            LatObstacleDecisionType::LEFT ||
        lat_obstacle_decision.at(obstacle->id()) ==
            LatObstacleDecisionType::RIGHT) {
      return true;
    }
  }
  return false;
}

bool HppGeneralLateralDecider::CheckObstacleCrossingCondition(
    const std::shared_ptr<FrenetObstacle> obstacle, bool &is_cross_obj) {
  return false;
}

void HppGeneralLateralDecider::RefineConflictLatDecisions(
    const double ego_l, ObstacleDecision& obstacle_decision) {
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

void HppGeneralLateralDecider::ExtractBoundary(
    std::vector<std::pair<double, double>> &second_frenet_soft_bounds,
    std::vector<std::pair<double, double>> &first_frenet_soft_bounds,
    std::vector<std::pair<double, double>> &frenet_hard_bounds,
    std::vector<std::pair<BoundInfo, BoundInfo>> &second_soft_bounds_info,
    std::vector<std::pair<BoundInfo, BoundInfo>> &first_soft_bounds_info,
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
    frenet_hard_bounds[i] = hard_bound;
    hard_bounds_info[i] = hard_bound_info;
  }

  for (int i = 0; i < second_soft_bounds_.size(); i++) {
    std::pair<double, double> soft_bound{-10., 10.};  // <lower ,upper>
    std::pair<BoundInfo, BoundInfo> soft_bound_info;  // <lower ,upper>
    PostProcessBound(planning_init_point_l, second_soft_bounds_[i], soft_bound,
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
    second_frenet_soft_bounds[i] = soft_bound;
    second_soft_bounds_info[i] = soft_bound_info;
  }

  for (int i = 0; i < first_soft_bounds_.size(); i++) {
    std::pair<double, double> first_soft_bound{-20., 20.};  // <lower ,upper>
    std::pair<BoundInfo, BoundInfo> first_soft_bound_info;  // <lower ,upper>
    PostProcessBound(planning_init_point_l, first_soft_bounds_[i], first_soft_bound,
                     first_soft_bound_info);
    if (i == 0) {
      ProtectBoundByInitPoint(first_soft_bound, first_soft_bound_info);
    }
    // second_soft in first_soft
    // 满足特定场景的case，不进行比大小，例如默认关闭first_soft_bound
    if (!(second_soft_bounds_info[i].first.type == BoundType :: ROAD_BORDER ||
        second_soft_bounds_info[i].first.type == BoundType :: EGO_POSITION ||
        second_soft_bounds_info[i].first.type == BoundType :: LANE)) {
      // if (first_soft_bound.first > second_frenet_soft_bounds[i].second) {
      //   first_soft_bound.first = second_frenet_soft_bounds[i].second;
      // } else if (first_soft_bound.first < second_frenet_soft_bounds[i].first) {
      //   first_soft_bound.first = second_frenet_soft_bounds[i].first;
      // }
      if (first_soft_bound.first < second_frenet_soft_bounds[i].first) {
        first_soft_bound.first = second_frenet_soft_bounds[i].first;
      }
      if (first_soft_bound.second < second_frenet_soft_bounds[i].first) {
        first_soft_bound.second = second_frenet_soft_bounds[i].first;
      }
      if (first_soft_bound.first > second_frenet_soft_bounds[i].second) {
        if (second_soft_bounds_info[i].second.type == BoundType :: LANE ||
            second_soft_bounds_info[i].second.type == BoundType :: EGO_POSITION) {
          second_frenet_soft_bounds[i].second = first_soft_bound.first;
        } else {
          first_soft_bound.first = second_frenet_soft_bounds[i].second;
        }
      }
    }
    if (!(second_soft_bounds_info[i].second.type == BoundType :: ROAD_BORDER ||
        second_soft_bounds_info[i].second.type == BoundType :: EGO_POSITION ||
        second_soft_bounds_info[i].second.type == BoundType :: LANE)) {
      // if (first_soft_bound.second > second_frenet_soft_bounds[i].second) {
      //   first_soft_bound.second = second_frenet_soft_bounds[i].second;
      // } else if (first_soft_bound.second < second_frenet_soft_bounds[i].first) {
      //   first_soft_bound.second = second_frenet_soft_bounds[i].first;
      // }
      if (first_soft_bound.second > second_frenet_soft_bounds[i].second) {
        first_soft_bound.second = second_frenet_soft_bounds[i].second;
      }
      if (first_soft_bound.first > second_frenet_soft_bounds[i].second) {
        first_soft_bound.first = second_frenet_soft_bounds[i].second;
      }
      if (first_soft_bound.second < second_frenet_soft_bounds[i].first) {
        if (second_soft_bounds_info[i].first.type == BoundType :: LANE ||
            second_soft_bounds_info[i].first.type == BoundType :: EGO_POSITION) {
          second_frenet_soft_bounds[i].first = first_soft_bound.second;
        } else {
          first_soft_bound.second = second_frenet_soft_bounds[i].first;
        }
      }
    }
    first_frenet_soft_bounds[i] = first_soft_bound;
    first_soft_bounds_info[i] = first_soft_bound_info;
  }

  assert(frenet_hard_bounds.size() == ref_traj_points_.size());
  assert(first_frenet_soft_bounds.size() == ref_traj_points_.size());
  assert(second_frenet_soft_bounds.size() == ref_traj_points_.size());
}


void HppGeneralLateralDecider::PostProcessBoundary() {
  LimitFrenetLateralSlope(first_frenet_soft_bounds_);
  LimitFrenetLateralSlope(second_frenet_soft_bounds_);
  LimitFrenetLateralSlope(frenet_hard_bounds_);

  const std::shared_ptr<KDPath> frenet_coord =
      reference_path_ptr_->get_frenet_coord();
  if (frenet_coord == nullptr) {
    return;
  }

  enu_soft_bounds_.clear();
  enu_hard_bounds_.clear();
  Point2D tmp_soft_lower_point;
  Point2D tmp_soft_upper_point;
  Point2D tmp_hard_lower_point;
  Point2D tmp_hard_upper_point;
  for (size_t i = 0; i < ref_traj_points_.size(); ++i) {
    if (!frenet_coord->SLToXY(
            Point2D(ref_traj_points_[i].s, second_frenet_soft_bounds_[i].first),
            tmp_soft_lower_point))  // soft lower
    {
      // TODO: add logs
    }

    if (!frenet_coord->SLToXY(
            Point2D(ref_traj_points_[i].s, second_frenet_soft_bounds_[i].second),
            tmp_soft_upper_point))  // soft upper
    {
      // TODO: add logs
    }

    enu_soft_bounds_.emplace_back(std::pair<Point2D, Point2D>(
        tmp_soft_lower_point, tmp_soft_upper_point));

    if (!frenet_coord->SLToXY(
            Point2D(ref_traj_points_[i].s, frenet_hard_bounds_[i].first),
            tmp_hard_lower_point))  // hard lower
    {
      // TODO: add logs
    }

    if (!frenet_coord->SLToXY(
            Point2D(ref_traj_points_[i].s, frenet_hard_bounds_[i].second),
            tmp_hard_upper_point))  // hard upper
    {
      // TODO: add logs
    }

    enu_hard_bounds_.emplace_back(std::pair<Point2D, Point2D>(
      tmp_hard_lower_point, tmp_hard_upper_point));
  }
}

bool HppGeneralLateralDecider::SmoothPointsIteratively(
    const std::vector<Point2D> &input,
    TrajectoryPoints &output,
    const int effective_radius,
    const size_t iters) {

  const size_t n = input.size();
  if (n < 3 || effective_radius <= 0 || iters == 0) {
    return false;
  }

  const auto frenet_coord = reference_path_ptr_ ? reference_path_ptr_->get_frenet_coord() : nullptr;
  if (frenet_coord == nullptr || ref_traj_points_.size() != n) {
    return false;
  }

  std::vector<double> s_ref(n, 0.0);
  std::vector<double> l_curr(n, 0.0);
  for (size_t i = 0; i < n; ++i) {

    s_ref[i] = input[i].x;
    l_curr[i] = input[i].y;
  }

  std::vector<double> l_next(n, 0.0);
  std::vector<double> t(n, 0.0);
  for (size_t i = 0; i < n; ++i) {
    t[i] = static_cast<double>(i) / static_cast<double>(n - 1);
  }

  int base_stride = std::max(1, effective_radius);
  base_stride = std::min<int>(base_stride, static_cast<int>(n - 1));

  for (size_t iter = 0; iter < iters; ++iter) {
    const int stride = std::max(1, base_stride - static_cast<int>(iter));

    std::vector<size_t> ctrl_idx;
    ctrl_idx.reserve(n / static_cast<size_t>(stride) + 4);
    ctrl_idx.push_back(0);
    ctrl_idx.push_back(n - 1);
    for (size_t i = 0; i < n; i += static_cast<size_t>(stride)) {
      ctrl_idx.push_back(i);
    }

    std::sort(ctrl_idx.begin(), ctrl_idx.end());
    ctrl_idx.erase(std::unique(ctrl_idx.begin(), ctrl_idx.end()), ctrl_idx.end());

    if (ctrl_idx.size() < 2) {
      l_next = l_curr;
      break;
    }

    std::vector<double> t_ctrl;
    std::vector<double> l_ctrl;
    t_ctrl.reserve(ctrl_idx.size());
    l_ctrl.reserve(ctrl_idx.size());

    for (size_t idx : ctrl_idx) {
      t_ctrl.push_back(t[idx]);
      l_ctrl.push_back(l_curr[idx]);
    }

    pnc::mathlib::spline sl_spline;
    const pnc::mathlib::spline::spline_type type =
        (ctrl_idx.size() >= 4) ? pnc::mathlib::spline::cspline
                               : pnc::mathlib::spline::linear;
    sl_spline.set_points(t_ctrl, l_ctrl, type);

    for (size_t i = 0; i < n; ++i) {
      l_next[i] = sl_spline(t[i]);
    }

    l_curr.swap(l_next);
  }

  for (size_t i = 0; i < n; ++i) {
    output[i].s = s_ref[i];
    output[i].l = l_curr[i];
    Point2D xy;
    if (!frenet_coord->SLToXY(Point2D(s_ref[i], l_curr[i]), xy)) {
      return false;
    } else {
      output[i].x = xy.x;
      output[i].y = xy.y;
    }
  }

  return true;
}

void HppGeneralLateralDecider::GenerateBoundCenterLine(
    const std::vector<std::pair<double, double>> &hard_bounds,
    const int window) {
  const auto frenet_coord = reference_path_ptr_->get_frenet_coord();
  if (frenet_coord == nullptr) {
    return;
  }

  const size_t n = ref_traj_points_.size();
  if (n == 0 || hard_bounds.size() != n) {
    return;
  }

  std::vector<Point2D> center_sl_candidate(n);
  for (size_t i = 0; i < n; ++i) {
    center_sl_candidate[i].x = ref_traj_points_[i].s;
    center_sl_candidate[i].y =
        0.5 * (hard_bounds[i].first + hard_bounds[i].second);
  }

  if (!SmoothPointsIteratively(center_sl_candidate,
        bound_center_line_, 2 * window, 1)) {
    bound_center_line_ = ref_traj_points_;
  }
  // ref_traj_points_ = bound_center_line_;
}

TrajectoryPoints HppGeneralLateralDecider::iterativeSmoothWithBounds(
    const TrajectoryPoints& trajectory) {
  const int iterations = 10;
  const double smooth_factor = 0.8;
  const size_t n = trajectory.size();

  TrajectoryPoints result = trajectory;
  std::vector<double> current_l(n), new_l(n);

  for (int iter = 0; iter < iterations; ++iter) {
    for (size_t i = 0; i < n; ++i) {
      current_l[i] = result[i].l;
      new_l[i] = current_l[i];
    }

    for (size_t i = 1; i < n - 1; ++i) {
      const double smoothed =
          (current_l[i - 1] + current_l[i] + current_l[i + 1]) / 3.0;
      new_l[i] =
          (1.0 - smooth_factor) * current_l[i] + smooth_factor * smoothed;
    }

    for (size_t i = 0; i < n; ++i) {
      new_l[i] = std::max(frenet_hard_bounds_[i].first,
                          std::min(frenet_hard_bounds_[i].second, new_l[i]));
      result[i].l = new_l[i];
    }
  }
  return result;
}

void HppGeneralLateralDecider::MergeReferenceTrajectories(
    const std::vector<std::pair<double, double>> &hard_bounds,
    const std::vector<std::pair<double, double>> &soft_bounds) {
  const size_t n = ref_traj_points_.size();
  if (n == 0 || bound_center_line_.size() != n || hard_bounds.size() != n ||
      soft_bounds.size() != n) {
    return;
  }

  constexpr size_t kBlendRadius = 5;
  const auto frenet_coord = reference_path_ptr_->get_frenet_coord();
  if (frenet_coord == nullptr) {
    return;
  }

  auto is_break_hard_bound = [&](size_t idx) {
    return ref_traj_points_[idx].l < hard_bounds[idx].first ||
           ref_traj_points_[idx].l > hard_bounds[idx].second;
  };

  size_t break_start = n;
  for (size_t i = 0; i < n; ++i) {
    if (is_break_hard_bound(i)) {
      break_start = std::max(i, kBlendRadius);
      break;
    }
  }

  if (break_start == n) {
    return;
  }

  constexpr double kRecoverConfirmDist = 3.0;
  std::vector<std::pair<size_t, size_t>> break_recover_ranges;
  break_recover_ranges.reserve(4);

  bool in_break = false;
  size_t cur_break_start = n;
  size_t recover_window_start = n;
  for (size_t i = 0; i < n; ++i) {
    const bool is_break = is_break_hard_bound(i);
    if (!in_break) {
      if (is_break) {
        cur_break_start = std::max(i, kBlendRadius);
        in_break = true;
        recover_window_start = n;
      }
      continue;
    }

    if (!is_break) {
      if (recover_window_start == n) {
        recover_window_start = i;
      }
      const double recover_dist =
          ref_traj_points_[i].s - ref_traj_points_[recover_window_start].s;
      if (recover_dist >= kRecoverConfirmDist) {
        break_recover_ranges.emplace_back(cur_break_start, recover_window_start);
        in_break = false;
        cur_break_start = n;
        recover_window_start = n;
      }
    } else {
      recover_window_start = n;
    }
  }

  if (in_break && cur_break_start < n) {
    break_recover_ranges.emplace_back(cur_break_start, n - 1);
  }

  if (break_recover_ranges.empty()) {
    break_recover_ranges.emplace_back(break_start, n - 1);
  }

  TrajectoryPoints merged = ref_traj_points_;
  const size_t first_break_start = break_recover_ranges.front().first;
  const double s_ref = ref_traj_points_.front().s;

  if (first_break_start < plan_history_traj_.size() &&
      (plan_history_traj_[first_break_start].s - s_ref > 1.0)) {
    for (size_t i = 0; i < first_break_start; ++i) {
      merged[i] = plan_history_traj_[i];
    }
  } else {
    for (size_t i = 0; i < n; ++i) {
      if (plan_history_traj_[i].s - s_ref > 1.0) {
        break;
      }
      merged[i] = plan_history_traj_[i];
    }
  }

  for (const auto &range : break_recover_ranges) {
    const size_t &s = range.first;
    const size_t &e = range.second;
    if (s >= n) {
      continue;
    }
    for (size_t i = s; i <= std::min(e, n - 1); ++i) {
      merged[i] = bound_center_line_[i];
    }
  }

  merged = iterativeSmoothWithBounds(merged);
  Point2D ref_point;
  for (size_t i = 0; i < merged.size(); i++) {
    if (frenet_coord->SLToXY(
            Point2D(merged[i].s, merged[i].l),
            ref_point)) {
      merged[i].x = ref_point.x;
      merged[i].y = ref_point.y;
    }
    if (i > 0) {
      const double dx = merged[i].x - merged[i - 1].x;
      const double dy = merged[i].y - merged[i - 1].y;
      merged[i - 1].heading_angle = std::atan2(dy, dx);
    }
  }
  merged.back().heading_angle =
      merged[merged.size() - 2].heading_angle;

  ref_traj_points_ = merged;
}

void HppGeneralLateralDecider::LimitFrenetLateralSlope(
    std::vector<std::pair<double, double>> &frenet_bounds) {
  // 避免dl ds 差值过大
  const int boundsize = frenet_bounds.size();
  const double dl_ds_ratio = 1.0;
  double ds = 0;
  double dl = 0;
  if (boundsize > 1) {
    for (int i = 1; i <= boundsize - 1; i++) {
      ds = ref_traj_points_[i].s - ref_traj_points_[i - 1].s;
      dl = frenet_bounds[i].second - frenet_bounds[i - 1].second;
      if (std::fabs(dl) > std::fabs(dl_ds_ratio * ds)) {
        if (dl > 0) {
          frenet_bounds[i].second =
              frenet_bounds[i - 1].second + dl_ds_ratio * ds;
        } else {
          frenet_bounds[i - 1].second =
              frenet_bounds[i].second + dl_ds_ratio * ds;
        }
      }
      dl = frenet_bounds[i].first - frenet_bounds[i - 1].first;
      if (std::fabs(dl) > std::fabs(dl_ds_ratio * ds)) {
        if (dl > 0) {
          frenet_bounds[i - 1].first =
              frenet_bounds[i].first - dl_ds_ratio * ds;
        } else {
          frenet_bounds[i].first =
              frenet_bounds[i - 1].first - dl_ds_ratio * ds;
        }
      }
    }
  }
}


void HppGeneralLateralDecider::ProtectBoundByInitPoint(
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

void HppGeneralLateralDecider::ExtractDynamicObstacleBound(
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
              second_soft_bounds_[index].emplace_back(obstacle_pos_bound);
            }
          }
        }
      }
    }
  }
}

void HppGeneralLateralDecider::ExtractStaticObstacleBound(
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
            second_soft_bounds_[i].emplace_back(obstacle_pos_bound);
          }
        }
      }
    }
  }
}

void HppGeneralLateralDecider::PostProcessBound(
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
    // hack: only road border, agent and ground line in hard bounds
    if ((upper_bounds[upper_index].weight < 0.0) &&
        ((upper_type != BoundType::ROAD_BORDER) &&
         (upper_type != BoundType::AGENT) &&
         (upper_type != BoundType::GROUNDLINE))) {
      upper_index += 1;
      continue;
    }
    if ((lower_bounds[lower_index].weight < 0.0) &&
        ((lower_type != BoundType::ROAD_BORDER) &&
         (lower_type != BoundType::AGENT) &&
         (lower_type != BoundType::GROUNDLINE))) {
      lower_index += 1;
      continue;
    }
    double lower = lower_bounds[lower_index].lower;
    double upper = upper_bounds[upper_index].upper;
    const int lower_priority =
        hpp_general_lateral_decider_utils::GetBoundTypePriority(lower_type);
    const int upper_priority =
        hpp_general_lateral_decider_utils::GetBoundTypePriority(upper_type);
    const double lower_weight =
        hpp_general_lateral_decider_utils::GetBoundWeight(
            lower_type, config_.map_bound_weight);
    const double upper_weight =
        hpp_general_lateral_decider_utils::GetBoundWeight(
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
        if (upper_type == BoundType::ADJACENT_AGENT) {
          if (upper_bound < planning_init_point_l) {
            upper_bound = planning_init_point_l;
            use_upper_init_protect = true;
          }
        }
      } else if (upper == upper_bound) {
        if ((upper_type == BoundType::ADJACENT_AGENT) && (upper_index == 0)) {
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
        if (lower_type == BoundType::ADJACENT_AGENT) {
          if (lower_bound > planning_init_point_l) {
            lower_bound = planning_init_point_l;
            use_lower_init_protect = true;
          }
        }
      } else if (lower == lower_bound) {
        if ((lower_type == BoundType::ADJACENT_AGENT) && (lower_index == 0)) {
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
        if (upper_type == BoundType::ADJACENT_AGENT) {
          if (upper_bound < planning_init_point_l) {
            upper_bound = planning_init_point_l;
            lower_bound =
                std::min(planning_init_point_l, std::max(lower, lower_bound));
            use_upper_init_protect = true;
            // continue upper
            upper_index += 1;
          }
        }
        if (lower_type == BoundType::ADJACENT_AGENT) {
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
        if (lower_type == BoundType::ADJACENT_AGENT) {
          if (lower_bound > planning_init_point_l) {
            lower_bound = planning_init_point_l;
            upper_bound =
                std::max(planning_init_point_l, std::min(upper, upper_bound));
            use_lower_init_protect = true;
            // continue lower
            lower_index += 1;
          }
        }
        if (upper_type == BoundType::ADJACENT_AGENT) {
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
          if (upper_type == BoundType::ADJACENT_AGENT) {
            upper_bound = planning_init_point_l;
            lower_bound = std::min(lower, planning_init_point_l);
            use_upper_init_protect = true;
            upper_index += 1;
          } else {
            // end condition 3.not ADJACENT_AGENT
            break;
          }
        } else if (mid_bound > planning_init_point_l) {
          if (lower_type == BoundType::ADJACENT_AGENT) {
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

void HppGeneralLateralDecider::SaveLatDebugInfo(
    const std::vector<std::pair<double, double>> &second_frenet_soft_bounds,
    const std::vector<std::pair<double, double>> &first_frenet_soft_bounds,
    const std::vector<std::pair<double, double>> &frenet_hard_bounds,
    const std::vector<std::pair<BoundInfo, BoundInfo>> &second_soft_bounds_info,
    const std::vector<std::pair<BoundInfo, BoundInfo>> &first_soft_bounds_info,
    const std::vector<std::pair<BoundInfo, BoundInfo>> &hard_bounds_info) {
  const auto &potential_dangerous_agent_decider_output =
      session_->planning_context()
               .potential_dangerous_agent_decider_output();
  if (!potential_dangerous_agent_decider_output.dangerous_agent_info.empty() &&
      potential_dangerous_agent_decider_output.dangerous_agent_info.front().
      recommended_maneuver.lateral_maneuver != LateralManeuver :: IGNORE) {
    JSON_DEBUG_VALUE("potential_dangerous_agent_id",
        potential_dangerous_agent_decider_output.dangerous_agent_info.front().id);
  } else {
    JSON_DEBUG_VALUE("potential_dangerous_agent_id", -0.01);
  }

#ifdef ENABLE_PROTO_LOG
  lat_debug_info_.Clear();
  lat_debug_info_.mutable_bound_s_vec()->Resize(ref_traj_points_.size(), 0.0);
  lat_debug_info_.mutable_hard_lower_bound_info_vec()->Reserve(
      ref_traj_points_.size());
  lat_debug_info_.mutable_hard_upper_bound_info_vec()->Reserve(
      ref_traj_points_.size());
  lat_debug_info_.mutable_second_soft_lower_bound_info_vec()->Reserve(
      ref_traj_points_.size());
  lat_debug_info_.mutable_second_soft_upper_bound_info_vec()->Reserve(
      ref_traj_points_.size());
  lat_debug_info_.mutable_first_soft_lower_bound_info_vec()->Reserve(
      ref_traj_points_.size());
  lat_debug_info_.mutable_first_soft_upper_bound_info_vec()->Reserve(
      ref_traj_points_.size());

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

    auto second_soft_lower_bound_info =
        lat_debug_info_.mutable_second_soft_lower_bound_info_vec()->Add();
    second_soft_lower_bound_info->set_lower(second_frenet_soft_bounds[i].first);
    second_soft_lower_bound_info->mutable_bound_info()->set_id(
        second_soft_bounds_info[i].first.id);
    second_soft_lower_bound_info->mutable_bound_info()->set_type(
        BoundType2String(second_soft_bounds_info[i].first.type));

    auto second_soft_upper_bound_info =
        lat_debug_info_.mutable_second_soft_upper_bound_info_vec()->Add();
    second_soft_upper_bound_info->set_upper(second_frenet_soft_bounds[i].second);
    second_soft_upper_bound_info->mutable_bound_info()->set_id(
        second_soft_bounds_info[i].second.id);
    second_soft_upper_bound_info->mutable_bound_info()->set_type(
        BoundType2String(second_soft_bounds_info[i].second.type));

    auto first_soft_lower_bound_info =
        lat_debug_info_.mutable_first_soft_lower_bound_info_vec()->Add();
    first_soft_lower_bound_info->set_lower(first_frenet_soft_bounds[i].first);
    first_soft_lower_bound_info->mutable_bound_info()->set_id(
        first_soft_bounds_info[i].first.id);
    first_soft_lower_bound_info->mutable_bound_info()->set_type(
        BoundType2String(first_soft_bounds_info[i].first.type));

    auto first_soft_upper_bound_info =
        lat_debug_info_.mutable_first_soft_upper_bound_info_vec()->Add();
    first_soft_upper_bound_info->set_upper(first_frenet_soft_bounds[i].second);
    first_soft_upper_bound_info->mutable_bound_info()->set_id(
        first_soft_bounds_info[i].second.id);
    first_soft_upper_bound_info->mutable_bound_info()->set_type(
        BoundType2String(first_soft_bounds_info[i].second.type));
  }

  // 障碍物决策是否存在跳动
  lat_debug_info_.mutable_obstacle_ids()->Resize(
      dynamic_obstacle_decisions_.size() + static_obstacle_decisions_.size(),
      0);
  int i = 0;
  for (const auto &dynamic_obstacle_decision : dynamic_obstacle_decisions_) {
    lat_debug_info_.mutable_obstacle_ids()->Set(
        i, dynamic_obstacle_decision.first);
    i++;
  }
  for (const auto &static_obstacle_decision : static_obstacle_decisions_) {
    lat_debug_info_.mutable_obstacle_ids()->Set(i,
                                                static_obstacle_decision.first);
    i++;
  }

  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_lateral_behavior_debug_info()
      ->CopyFrom(lat_debug_info_);
#endif
}

void HppGeneralLateralDecider::CalculateAvoidObstacles(
    const std::vector<std::pair<double, double>> first_frenet_soft_bounds,
    std::vector<std::pair<BoundInfo, BoundInfo>> first_soft_bounds_info,
    const std::vector<std::pair<double, double>> second_frenet_soft_bounds,
    std::vector<std::pair<BoundInfo, BoundInfo>> second_soft_bounds_info) {
  auto &hpp_general_lateral_decider_output =
      session_->mutable_planning_context()
          ->mutable_hpp_general_lateral_decider_output();
  hpp_general_lateral_decider_output.avoid_ids.clear();
  const double planning_init_point_l =
      ego_frenet_state_.planning_init_point().frenet_state.r;
  auto check_and_add_avoid_id = [&](const BoundInfo &bound_info,
                                    double bound_value, bool is_upper) {
    auto it = reference_path_ptr_->get_obstacles_map().find(bound_info.id);
    if (it == reference_path_ptr_->get_obstacles_map().end()) {
      return;
    }
    auto obs = it->second;
    if (reference_path_ptr_->get_ego_frenet_boundary().s_start >
        obs->frenet_obstacle_boundary().s_end) {
      return;
    }
    if ((bound_info.type == BoundType::DYNAMIC_AGENT ||
         bound_info.type == BoundType::AGENT ||
         bound_info.type == BoundType::ADJACENT_AGENT ||
         bound_info.type == BoundType::REVERSE_AGENT) &&
        bound_info.id != -100) {
      bool is_avoid_car =
          is_upper ? (bound_value < planning_init_point_l ||
                      bound_value < -config_.bound2center_line_distance_thr)
                   : (bound_value > planning_init_point_l ||
                      bound_value > config_.bound2center_line_distance_thr);
      if (is_avoid_car &&
          std::find(hpp_general_lateral_decider_output.avoid_ids.begin(),
                    hpp_general_lateral_decider_output.avoid_ids.end(),
                    bound_info.id) ==
              hpp_general_lateral_decider_output.avoid_ids.end()) {
        hpp_general_lateral_decider_output.avoid_ids.emplace_back(bound_info.id);
      }
    }
  };
  //(huwang5)TODO:左右bound重叠时，障碍物的释放需要进一步考虑
  for (int i = 0; i < first_frenet_soft_bounds.size(); ++i) {
    check_and_add_avoid_id(first_soft_bounds_info[i].first,
                           first_frenet_soft_bounds[i].first, false);  // lower
    check_and_add_avoid_id(first_soft_bounds_info[i].second,
                           first_frenet_soft_bounds[i].second, true);  // upper
  }
  for (int i = 0; i < second_frenet_soft_bounds.size(); ++i) {
    check_and_add_avoid_id(second_soft_bounds_info[i].first,
                           second_frenet_soft_bounds[i].first, false);  // lower
    check_and_add_avoid_id(second_soft_bounds_info[i].second,
                           second_frenet_soft_bounds[i].second, true);  // upper
  }
  std::vector<double> avoid_ids_double(
      hpp_general_lateral_decider_output.avoid_ids.begin(),
      hpp_general_lateral_decider_output.avoid_ids.end());
  JSON_DEBUG_VECTOR("hpp_lateral_avoid_ids", avoid_ids_double, 0);
}

void HppGeneralLateralDecider::PostProcessReferenceTrajBySoftBound(
    const std::vector<std::pair<double, double>> &second_frenet_soft_bounds,
    const std::vector<std::pair<double, double>> &first_frenet_soft_bounds,
    GeneralLateralDeciderOutput &general_lateral_decider_output) {
  // bool bound_avoid = false;
  for (size_t i = 0; i < ref_traj_points_.size(); i++) {
    // if (ref_traj_points_[i].l < frenet_soft_bounds[i].first ||
    //     ref_traj_points_[i].l > frenet_soft_bounds[i].second) {
    //   bound_avoid = true;
    // }
    ref_traj_points_[i].l =
        std::min(std::max(ref_traj_points_[i].l, second_frenet_soft_bounds[i].first),
                 second_frenet_soft_bounds[i].second);
    ref_traj_points_[i].l =
        std::min(std::max(ref_traj_points_[i].l, first_frenet_soft_bounds[i].first),
                 first_frenet_soft_bounds[i].second);
  }
  // general_lateral_decider_output.bound_avoid = bound_avoid;
}

void HppGeneralLateralDecider::GenerateLateralDeciderOutput(
    const std::vector<std::pair<double, double>> &second_frenet_soft_bounds,
    const std::vector<std::pair<double, double>> &first_frenet_soft_bounds,
    const std::vector<std::pair<double, double>> &frenet_hard_bounds,
    const std::vector<std::pair<BoundInfo, BoundInfo>> &second_soft_bounds_info,
    const std::vector<std::pair<BoundInfo, BoundInfo>> &first_soft_bounds_info,
    const std::vector<std::pair<BoundInfo, BoundInfo>> &hard_bounds_info,
    GeneralLateralDeciderOutput &general_lateral_decider_output) {
  general_lateral_decider_output.second_soft_bounds = std::move(second_soft_bounds_);
  general_lateral_decider_output.first_soft_bounds = std::move(first_soft_bounds_);
  general_lateral_decider_output.hard_bounds = std::move(hard_bounds_);

  GenerateEnuBoundaryPoints(second_frenet_soft_bounds, first_frenet_soft_bounds, frenet_hard_bounds,
                            general_lateral_decider_output);

  GenerateEnuReferenceTraj(general_lateral_decider_output);

  auto &hard_bounds_frenet_output =
      general_lateral_decider_output.hard_bounds_frenet_point;
  auto &first_soft_bounds_frenet_output =
      general_lateral_decider_output.first_soft_bounds_frenet_point;
  auto &second_soft_bounds_frenet_output =
      general_lateral_decider_output.second_soft_bounds_frenet_point;
  auto &hard_bounds_info_output =
      general_lateral_decider_output.hard_bounds_info;
  auto &first_soft_bounds_info_output =
      general_lateral_decider_output.first_soft_bounds_info;
  auto &second_soft_bounds_info_output =
      general_lateral_decider_output.second_soft_bounds_info;

  for (size_t i = 0; i < ref_traj_points_.size(); ++i) {
    hard_bounds_frenet_output.emplace_back(frenet_hard_bounds[i]);
    first_soft_bounds_frenet_output.emplace_back(first_frenet_soft_bounds[i]);
    second_soft_bounds_frenet_output.emplace_back(second_frenet_soft_bounds[i]);
    hard_bounds_info_output.emplace_back(hard_bounds_info[i]);
    first_soft_bounds_info_output.emplace_back(first_soft_bounds_info[i]);
    second_soft_bounds_info_output.emplace_back(second_soft_bounds_info[i]);
  }
}

void HppGeneralLateralDecider::GenerateEnuBoundaryPoints(
    const std::vector<std::pair<double, double>> &second_frenet_soft_bounds,
    const std::vector<std::pair<double, double>> &first_frenet_soft_bounds,
    const std::vector<std::pair<double, double>> &frenet_hard_bounds,
    GeneralLateralDeciderOutput &general_lateral_decider_output) {
  auto &soft_bounds_output =
      general_lateral_decider_output.second_soft_bounds_cart_point;
  auto &hard_bounds_output =
      general_lateral_decider_output.hard_bounds_cart_point;

  soft_bounds_output = enu_soft_bounds_;

  hard_bounds_output = enu_hard_bounds_;

  // 临时 hack（flli9）：避免折返约束
  static constexpr double kReverseTurnDegThreshold = 95.0 * M_PI / 180.0;
  static constexpr double kMinSegmentLength = 0.1;
  const auto smooth_reverse_points = [&](const int current_index,
                                         const bool use_lower) {
    if (current_index <= 0 ||
        current_index >= static_cast<int>(hard_bounds_output.size()) ||
        current_index >= static_cast<int>(ref_traj_points_.size())) {
      return;
    }
    int move_step_num = 0;
    int probe_index = current_index;
    while (probe_index >= 1) {
      const int pre_index = probe_index - 1;
      if (pre_index + 1 >= static_cast<int>(ref_traj_points_.size())) {
        break;
      }
      const auto &curr_point = use_lower
                                   ? hard_bounds_output[current_index].first
                                   : hard_bounds_output[current_index].second;
      const auto &pre_point = use_lower ? hard_bounds_output[pre_index].first
                                        : hard_bounds_output[pre_index].second;

      const auto prev_2_curr = planning_math::Vec2d(curr_point.x - pre_point.x,
                                                    curr_point.y - pre_point.y);

      const auto prev_2_curr_heading = prev_2_curr.Angle();

      if (prev_2_curr.Length() <= kMinSegmentLength) {
        break;
      }
      const auto &frenet_coord = reference_path_ptr_->get_frenet_coord();
      double ref_point_heading =
          frenet_coord->GetPathPointByS(ref_traj_points_[pre_index + 1].s)
              .theta();
      const auto heading_diff =
          planning_math::AngleDiff(prev_2_curr_heading, ref_point_heading);

      if (std::fabs(heading_diff) < kReverseTurnDegThreshold) {
        break;
      }
      ++move_step_num;
      --probe_index;
    }
    if (move_step_num <= 0) {
      return;
    }
    auto current_point = use_lower ? hard_bounds_output[current_index].first
                                   : hard_bounds_output[current_index].second;
    for (int i = current_index; i > current_index - move_step_num; --i) {
      if (use_lower) {
        hard_bounds_output[i].first = hard_bounds_output[i - 1].first;
      } else {
        hard_bounds_output[i].second = hard_bounds_output[i - 1].second;
      }
    }
    if (use_lower) {
      hard_bounds_output[current_index - move_step_num].first = current_point;
    } else {
      hard_bounds_output[current_index - move_step_num].second = current_point;
    }
  };

  const ConstStaticAnalysisStoragePtr static_storage =
      reference_path_ptr_->get_static_analysis_storage();
  const int max_process_index =
      static_cast<int>(std::min(hard_bounds_output.size(), ref_traj_points_.size())) - 1;
  for (int current_index = max_process_index; current_index >= 1;
       --current_index) {
    // only smooth curve type
    ResultTypeInfo type_info;
    if (static_storage) {
      type_info =
          static_storage->GetTypeInfo(ref_traj_points_[current_index].s);
    }
    if (type_info.road_type == CRoadType::NormalStraight ||
        type_info.road_type == CRoadType::SharpTurn) {
      // continue;
    }
    smooth_reverse_points(current_index, true);
    smooth_reverse_points(current_index, false);
  }
  // 临时 hack
  general_lateral_decider_output.first_soft_bounds_cart_point = general_lateral_decider_output.second_soft_bounds_cart_point;
};

void HppGeneralLateralDecider::GenerateEnuReferenceTraj(
    GeneralLateralDeciderOutput &general_lateral_decider_output) {
  const std::shared_ptr<KDPath> frenet_coord =
      reference_path_ptr_->get_frenet_coord();
  if (frenet_coord == nullptr) {
    return;
  }
  auto &enu_ref_path = general_lateral_decider_output.enu_ref_path;
  auto &enu_ref_theta = general_lateral_decider_output.enu_ref_theta;
  enu_ref_path.clear();
  enu_ref_theta.clear();
  enu_ref_path.resize(ref_traj_points_.size());

  for (size_t i = 0; i < ref_traj_points_.size(); i++) {
    enu_ref_path[i].first = ref_traj_points_[i].x;
    enu_ref_path[i].second = ref_traj_points_[i].y;
    enu_ref_theta.emplace_back(ref_traj_points_[i].heading_angle);
  }

  const auto &s_start = ref_traj_points_.front().s;
  const auto &s_end = ref_traj_points_.back().s;

  general_lateral_decider_output.v_cruise =
      (s_end - s_start) / (config_.delta_t * (ref_traj_points_.size() - 1));
}

void HppGeneralLateralDecider::SampleRoadDistanceInfo(
    const double s_target, ReferencePathPoint& sample_path_point) {
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

void HppGeneralLateralDecider::CalcLateralBehaviorOutput() {
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
  std::vector<planning::PathPoint> path_points;
  if (flane != nullptr && flane->get_reference_path() != nullptr) {
    auto &ref_path = flane->get_reference_path();
    for (auto &ref_point : ref_path->get_points()) {
      path_points.emplace_back(ref_point.path_point);
    }
  }
  // scenario, left_faster, right_is_faster
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
  // borrow_bicycle_lane
  lateral_output.lat_offset = 0.0;
  bool isRedLightStop = false;  // attention again!!!
  // TrackedObject *lead_one = session_->mutable_environmental_model()
  //                               ->get_lateral_obstacle()
  //                               ->leadone();

  // if (((virtual_lane_manager->current_lane_virtual_id() ==
  //       virtual_lane_manager->get_lane_num() - 1) ||
  //      (virtual_lane_manager->current_lane_virtual_id() ==
  //           virtual_lane_manager->get_lane_num() - 2 &&
  //       virtual_lane_manager->get_right_lane() != nullptr &&
  //       virtual_lane_manager->get_right_lane()->get_lane_type() ==
  //           iflyauto::LANETYPE_NON_MOTOR)) &&
  //     ((!isRedLightStop && lateral_output.accident_ahead &&
  //       lead_one != nullptr && lead_one->type == 20001))) {
  //   lateral_output.borrow_bicycle_lane = true;
  // } else {
  //   lateral_output.borrow_bicycle_lane = false;
  // }
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

  // isFasterStaticAvd, attention!
  bool right_direct_exist = true;
  bool curr_direct_has_right = false;
  bool curr_direct_has_straight = true;
  bool is_right_turn = false;
  bool left_direct_has_straight = true;
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

bool HppGeneralLateralDecider::IsAgentPredLonOverlapWithPlanPath(
    const std::shared_ptr<FrenetObstacle> obstacle) {
  const double KDynamicLonOverlapDisBuffer = 1.0;
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
    auto ok = obstacle->get_polygon_at_time_tmp(t, reference_path_ptr_,
                                                obstacle_sl_polygon);
    if (!ok) {
      continue;
    }
    const double obstacle_s_start = obstacle_sl_polygon.min_x();
    const double obstacle_s_end = obstacle_sl_polygon.max_x();

    double start_s = std::max(ego_s_start, obstacle_s_start);
    double end_s = std::min(ego_s_end, obstacle_s_end);
    if (start_s - KDynamicLonOverlapDisBuffer < end_s) {
      return true;
    }
  }
  return false;
}

bool HppGeneralLateralDecider::IsLonOverlap(
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

bool HppGeneralLateralDecider::IsFarObstacle(
    const std::shared_ptr<FrenetObstacle> obstacle) {
  const auto &care_object_lat_distance_threshold =
      config_.care_obj_lat_distance_threshold;
  const auto &care_object_lon_distance_threshold =
      config_.care_obj_lon_distance_threshold;
  const auto &ego_cur_s = ref_traj_points_.front().s;
  const auto &ego_cur_l = ref_traj_points_.front().l;
  if (std::fabs(obstacle->frenet_s() - ego_cur_s) >
          care_object_lon_distance_threshold or
      std::fabs(obstacle->frenet_l() - ego_cur_l) >
          care_object_lat_distance_threshold) {
    // TBD: add log
    return false;
  }
  return true;
}

bool HppGeneralLateralDecider::IsRearObstacle(
    const std::shared_ptr<FrenetObstacle> obstacle) {
  return reference_path_ptr_->get_ego_frenet_boundary().s_start >
         obstacle->frenet_obstacle_boundary().s_end;
}

bool HppGeneralLateralDecider::IsFilterForStaticObstacle(
    const std::shared_ptr<FrenetObstacle> obstacle) {
  const auto &otype = obstacle->type();
  const auto ofusion_source = obstacle->obstacle()->fusion_source();
  if ((ofusion_source & OBSTACLE_SOURCE_CAMERA) == 0) {
    return false;
  }

  if (otype == iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN or  // TBD: check
                                                             // obstacle type
      otype == iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN_MOVABLE or
      otype == iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN_IMMOVABLE or
      otype == iflyauto::ObjectType::OBJECT_TYPE_VAN or
      otype == iflyauto::ObjectType::OBJECT_TYPE_TRAILER or
      otype == iflyauto::ObjectType::OBJECT_TYPE_TRAFFIC_TEM_SIGN or
      otype == iflyauto::ObjectType::OBJECT_TYPE_SOLT) {
    // add logs;
    return false;
  }

  // filter rear object
  if (IsRearObstacle(obstacle)) {
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
  return true;
}

bool HppGeneralLateralDecider::IsFilterForDynamicObstacle(
    const std::shared_ptr<FrenetObstacle> obstacle) {
  const auto &otype = obstacle->type();
  const auto ofusion_source = obstacle->obstacle()->fusion_source();
  if ((ofusion_source & OBSTACLE_SOURCE_CAMERA) == 0) {
    return false;
  }

  if (otype == iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN or  // TBD: check
                                                             // obstacle type
      otype == iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN_MOVABLE or
      otype == iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN_IMMOVABLE or
      otype == iflyauto::ObjectType::OBJECT_TYPE_VAN or
      otype == iflyauto::ObjectType::OBJECT_TYPE_TRAILER or
      otype == iflyauto::ObjectType::OBJECT_TYPE_TRAFFIC_TEM_SIGN or
      otype == iflyauto::ObjectType::OBJECT_TYPE_SOLT) {
    // add logs;
    return false;
  }

  if (obstacle->source_type() == SourceType::OCC ||
      obstacle->source_type() == SourceType::GroundLine ||
      obstacle->source_type() == SourceType::ParkingSlot) {
    // add logs;
    return false;
  }

  // filter rear object
  if (IsRearObstacle(obstacle)) {
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

void HppGeneralLateralDecider::ConstructStaticObstacleTotalPolygons(
    std::array<std::vector<std::pair<int, Polygon2d>>, 2> &groundline_polygons,
    std::array<std::vector<std::pair<int, Polygon2d>>, 2>
        &parking_space_polygons) {
  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  const std::shared_ptr<KDPath> &frenet_coord =
      reference_path_ptr->get_frenet_coord();
  for (auto &obstacle : reference_path_ptr->get_parking_space()) {
    const planning_math::Polygon2d &polygon = obstacle->perception_polygon();
    hpp_general_lateral_decider_utils::MakePolygon(
        obstacle->id(), frenet_coord, polygon, parking_space_polygons[0],
        parking_space_polygons[1]);
  }
}
}  // namespace planning
