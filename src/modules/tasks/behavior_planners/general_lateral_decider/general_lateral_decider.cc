#include "general_lateral_decider.h"

#include <assert.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "agent_node_manager.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "frenet_ego_state.h"
#include "general_lateral_decider_utils.h"
#include "log.h"
#include "task_basic_types.h"
#include "utils/kd_path.h"
#include "vehicle_config_context.h"

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
  general_lateral_decider_output.enu_ref_path.clear();
  general_lateral_decider_output.last_enu_ref_path.clear();
  general_lateral_decider_output.soft_bounds.clear();
  general_lateral_decider_output.hard_bounds.clear();
  general_lateral_decider_output.soft_bounds_cart_point.clear();
  general_lateral_decider_output.hard_bounds_cart_point.clear();
  general_lateral_decider_output.enu_ref_theta.clear();
  general_lateral_decider_output.last_enu_ref_theta.clear();

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
  return true;
}

bool GeneralLateralDecider::Execute() {
  LOG_DEBUG("=======GeneralLateralDecider======= \n");

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

  ConstructTrajPoints(traj_points);
  ConstructReferencePathPoints(traj_points);

  GenerateRoadAndLaneBoundary();

  GenerateObstaclesBoundary();

  std::vector<std::pair<double, double>> frenet_soft_bounds;
  std::vector<std::pair<double, double>> frenet_hard_bounds;
  std::vector<std::pair<BoundInfo, BoundInfo>> soft_bounds_info;
  std::vector<std::pair<BoundInfo, BoundInfo>> hard_bounds_info;

  ExtractBoundary(frenet_soft_bounds, frenet_hard_bounds, soft_bounds_info,
                  hard_bounds_info);

  auto &general_lateral_decider_output =
      session_->mutable_planning_context()
          ->mutable_general_lateral_decider_output();
  GenerateLateralDeciderOutput(frenet_soft_bounds, frenet_hard_bounds,
                               general_lateral_decider_output);

  CalcLateralBehaviorOutput();

  SaveLatDebugInfo(frenet_soft_bounds, frenet_hard_bounds, soft_bounds_info,
                   hard_bounds_info);

  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("GeneralLateralDeciderCostTime", end_time - start_time);

  return true;
}

bool GeneralLateralDecider::ExecuteTest(bool pipeline_test) {
  // pipeline test
  return true;
}

bool GeneralLateralDecider::CalCruiseVelByCurvature(
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
                                           .get_virtual_lane_manager()
                                           ->is_on_ramp())) {
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

  bool limit_ref_vel_on_ramp_valid = false;
  bool is_LC_CHANGE =
      ((coarse_planning_info.target_state == kLaneChangeExecution) ||
       (coarse_planning_info.target_state == kLaneChangeComplete));
  bool is_LC_BACK = coarse_planning_info.target_state == kLaneChangeCancel;

  if (config_.lateral_ref_traj_type ||
      ((is_LC_CHANGE || is_LC_BACK) &&
       gap_selector_decider_output.gap_selector_trustworthy)) {
    traj_points = coarse_planning_info.trajectory_points;
  } else {
    // generate traj_points based on kMaxAcc or kMinAcc
    const auto &planning_init_point =
        ego_cart_state_manager_->planning_init_point();
    const double kMaxAcc = 2;
    const double kMinAcc = -5.5;
    double ego_v = planning_init_point.v;
    double cruise_v = session_->planning_context().v_ref_cruise();
    if (CalCruiseVelByCurvature(ego_v, flane->get_center_line(), cruise_v)) {
      limit_ref_vel_on_ramp_valid = true;
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
    const double max_ref_length =
        session_->planning_context().v_ref_cruise() * span_t;
    double avg_cruise_v = std::min(s, max_ref_length) / span_t;
    double delta_s = avg_cruise_v * config_.delta_t;
    Eigen::Vector2d cart_init_point(planning_init_point.lat_init_state.x(),
                                    planning_init_point.lat_init_state.y());
    const auto &frenet_coord =
        coarse_planning_info.reference_path->get_frenet_coord();
    const auto &cart_ref_info = coarse_planning_info.cart_ref_info;
    pnc::spline::Projection projection_spline;
    projection_spline.CalProjectionPoint(
        cart_ref_info.x_s_spline, cart_ref_info.y_s_spline,
        cart_ref_info.s_vec.front(), cart_ref_info.s_vec.back(),
        cart_init_point);

    double s_ref = projection_spline.GetOutput().s_proj;
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
  if ((is_LC_CHANGE || is_LC_BACK) &&
      gap_selector_decider_output.gap_selector_trustworthy) {
    general_lateral_decider_output.complete_follow = true;
    general_lateral_decider_output.lane_change_scene = true;
  } else {
    general_lateral_decider_output.complete_follow =
        false;  // fusion is unsteady, lane keep weight need decay in end of
                // ref
    general_lateral_decider_output.lane_change_scene = false;
    HandleAvoidScene(traj_points);
  }
}

void GeneralLateralDecider::HandleAvoidScene(TrajectoryPoints &traj_points) {
  const auto &frenet_coord =
      session_->planning_context()
          .lane_change_decider_output()
          .coarse_planning_info.reference_path->get_frenet_coord();

  const LateralOffsetDeciderOutput &lateral_offset_decider_output =
      session_->mutable_planning_context()->lateral_offset_decider_output();
  if (lateral_offset_decider_output.is_valid) {
    double lateral_offset = lateral_offset_decider_output.lateral_offset;
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
  for (const auto &traj_point : traj_points) {
    ReferencePathPoint refpath_pt{};
    if (!reference_path_ptr_->get_reference_point_by_lon(traj_point.s,
                                                         refpath_pt)) {
      // add logs
      LOG_ERROR("Get reference point by lon failed!");
    }
    ref_path_points_.emplace_back(refpath_pt);
  }

  ref_traj_points_.resize(traj_points.size());
  std::copy(traj_points.begin(), traj_points.end(), ref_traj_points_.begin());

  const auto &frenet_coord =
      session_->planning_context()
          .lane_change_decider_output()
          .coarse_planning_info.reference_path->get_frenet_coord();
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
    for (double t = 0; t <= plan_history_traj_tmp.back().t; t += config_.delta_t) {
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
  hard_bounds_.resize(ref_traj_points_.size());
  soft_bounds_.resize(ref_traj_points_.size());
  for (size_t i = 0; i < ref_traj_points_.size(); i++) {
    Bound soft_bound_road{-kDefaultDistanceToRoad, kDefaultDistanceToRoad};
    Bound hard_bound_road{-kDefaultDistanceToRoad, kDefaultDistanceToRoad};
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
      std::min(0.4, (max_collision_t - left_collision_t) * 0.1);
  *right_road_extra_buffer =
      std::min(0.4, (max_collision_t - right_collision_t) * 0.1);

  double extra_buffer =
      std::max(0.02 * std::pow(ego_v * 3.6, 0.75), kMinExtraBuffer);
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
            Vec2d(ref_path_points_.at(i - 1).path_point.s,
                  ref_path_points_.at(i - 1).distance_to_left_road_border),
            Vec2d(ref_path_points_.at(i).path_point.s,
                  ref_path_points_.at(i).distance_to_left_road_border));
        if (ego_box.HasOverlap(left_road_line_segment)) {
          is_left_overlap = true;
          *left_collision_t = t;
        }
      }

      if (!is_right_overlap) {
        const auto right_road_line_segment = LineSegment2d(
            Vec2d(ref_path_points_.at(i - 1).path_point.s,
                  -ref_path_points_.at(i - 1).distance_to_right_road_border),
            Vec2d(ref_path_points_.at(i).path_point.s,
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
  if (general_lateral_decider_output.lane_change_scene) {
    LOG_DEBUG("LatObstacle Decider! GS trustworthy");
    return;
  }

  if (plan_history_traj_.empty() || ref_path_points_.empty()) {
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
  const auto &lat_obstacle_decision = session_->environmental_model()
                                          .get_lateral_obstacle()
                                          ->lat_obstacle_decision();
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
  auto pre_lateral_decision = LatObstacleDecisionType::IGNORE;

  const bool init_lon_no_overlap =
      (obstacle->frenet_obstacle_boundary().s_end < ego_cur_s_start ||
       obstacle->frenet_obstacle_boundary().s_start > ego_cur_s_end);

  bool reset_conflict_decision{false};

  bool is_nudge_left = lat_obstacle_decision.at(obstacle->id()) ==
                       LatObstacleDecisionType::RIGHT;

  bool is_cross_obj{false};
  bool has_lat_decision{false};
  bool has_lon_decision{false};
  // Step 5) calculate soft_bound, hard_bound
  Polygon2d obstacle_sl_polygon;
  auto ok = obstacle->get_polygon_at_time_tmp(0, reference_path_ptr_,
                                              obstacle_sl_polygon);
  if (!ok) {
    // TBD add log
    return;
  }
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
            ego_frenet_state_.l(), obstacle->type(), is_update_hard_bound);

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
        overlap_max_y, lat_buf_dis, b_overlap_side, init_lon_no_overlap, is_nudge_left,
        is_cross_obj, pre_lateral_decision, reset_conflict_decision,
        obstacle_decision, lat_decision, lon_decision);
    has_lat_decision =
        has_lat_decision || lat_decision != LatObstacleDecisionType::IGNORE;
    has_lon_decision =
        has_lon_decision || lon_decision != LonObstacleDecisionType::IGNORE;

    AddObstacleDecisionBound(obstacle->id(), t, care_overlap_polygon,
                             lat_buf_dis, lat_decision, lon_decision,
                             obstacle_decision, is_update_hard_bound);
  }
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
  const auto &lat_obstacle_decision = session_->environmental_model()
                                          .get_lateral_obstacle()
                                          ->lat_obstacle_decision();
  // Step 1) configs
  const auto &l_care_width = config_.l_care_width;

  const auto &ego_cur_s = plan_history_traj_.front().s;

  const double rear_axle_to_front_bumper =  // TBD：define as consexpr
      vehicle_param.length - vehicle_param.rear_edge_to_rear_axle;
  const double ego_cur_s_start =
      ego_cur_s - vehicle_param.rear_edge_to_rear_axle;
  const double ego_cur_s_end = ego_cur_s + rear_axle_to_front_bumper;

  const double half_ego_width = vehicle_param.max_width * 0.5;
  auto collision_center_distance = half_ego_width;

  const double front_lon_buf_dis =
      general_lateral_decider_utils::CalDesireLonDistance(
          ego_frenet_state_.velocity_s(), obstacle->frenet_velocity_s());
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
    auto ok = obstacle->get_polygon_at_time_tmp(
        i * config_.delta_t, reference_path_ptr_, obstacle_sl_polygon);
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

    const double lat_buf_dis =
        general_lateral_decider_utils::CalDesireLateralDistance(
            ego_cart_state_manager_->ego_v(), t, 0, obstacle->type(),
            is_nudge_left);
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
    for (auto index : indexes) {
      GenerateObstaclePreliminaryDecision(
          ego_l, ref_path_points_[index].distance_to_right_lane_border,
          ref_path_points_[index].distance_to_left_lane_border, overlap_min_y,
          overlap_max_y, lat_buf_dis, b_overlap_side, init_lon_no_overlap, is_nudge_left,
          is_cross_obj, pre_lateral_decision, reset_conflict_decision,
          obstacle_decision, lat_decision, lon_decision);
      has_lat_decision =
          has_lat_decision || lat_decision != LatObstacleDecisionType::IGNORE;
      has_lon_decision =
          has_lon_decision || lon_decision != LonObstacleDecisionType::IGNORE;
    }
    AddObstacleDecisionBound(obstacle->id(), t, care_overlap_polygon,
                             lat_buf_dis, lat_decision, lon_decision,
                             obstacle_decision);
  }
}

void GeneralLateralDecider::GenerateObstaclePreliminaryDecision(
    double ego_l, double distance_to_right_lane_border,
    double distance_to_left_lane_border, double overlap_min_y,
    double overlap_max_y, double lat_buf_dis, bool b_overlap_side,
    bool init_lon_no_overlap, bool is_nudge_left, bool is_cross_obj,
    LatObstacleDecisionType pre_lateral_decision, bool &reset_conflict_decision,
    ObstacleDecision &obstacle_decision, LatObstacleDecisionType &lat_decision,
    LonObstacleDecisionType &lon_decision) {
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double half_ego_width = vehicle_param.max_width * 0.5;
  constexpr double kMaxAvoidEdgeL{2.0};  // m
  auto avoid_cross_lane = 0.2;
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
    int id, double t, Polygon2d &care_overlap_polygon, double lat_buf_dis,
    LatObstacleDecisionType lat_decision, LonObstacleDecisionType lon_decision,
    ObstacleDecision &obstacle_decision, bool is_update_hard_bound) {
  const double l_offset_limit = 10.0;
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double half_ego_width = vehicle_param.max_width * 0.5;

  BoundInfo bound_info;
  Bound bound{-l_offset_limit, l_offset_limit};
  BoundType type = BoundType::DYNAMIC_AGENT;
  const auto &obs_map = reference_path_ptr_->get_obstacles_map();
  if (obs_map.find(id) != obs_map.end()) {
    type = obs_map.at(id)->obstacle()->is_static() ? BoundType::AGENT
                                                   : BoundType::DYNAMIC_AGENT;
  }
  if (lat_decision == LatObstacleDecisionType::LEFT) {
    bound.lower = care_overlap_polygon.max_y() + lat_buf_dis + half_ego_width;
    // soft_bound.lower = is_rear_obstacle ? std::max(0.0,
    // care_overlap_polygon.max_y() + lat_buf_dis + half_ego_width) :
    //                                       care_overlap_polygon.max_y() +
    //                                       lat_buf_dis + half_ego_width;
    bound_info.type = type;
    bound_info.id = id;
  } else if (lat_decision == LatObstacleDecisionType::RIGHT) {
    bound.upper = care_overlap_polygon.min_y() - lat_buf_dis - half_ego_width;
    // soft_bound.upper = is_rear_obstacle ? std::min(0.0,
    // care_overlap_polygon.min_y() - lat_buf_dis - half_ego_width) :
    //                                       care_overlap_polygon.min_y() -
    //                                       lat_buf_dis - half_ego_width;
    bound_info.type = type;
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
  const auto &lat_obstacle_decision = session_->environmental_model()
                                          .get_lateral_obstacle()
                                          ->lat_obstacle_decision();
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
  for (int i = 0; i < hard_bounds_.size(); i++) {
    std::pair<double, double> hard_bound{-10., 10.};  // <lower ,upper>
    std::pair<BoundInfo, BoundInfo> hard_bound_info;  // <lower ,upper>
    PostProcessBound(hard_bounds_[i], hard_bound, hard_bound_info);
    if (i == 0) {
      ProtectBoundByInitPoint(hard_bound, hard_bound_info);
    }
    frenet_hard_bounds.emplace_back(hard_bound);
    hard_bounds_info.emplace_back(hard_bound_info);
  }

  for (int i = 0; i < hard_bounds_.size(); i++) {
    std::pair<double, double> soft_bound{-10., 10.};  // <lower ,upper>
    std::pair<BoundInfo, BoundInfo> soft_bound_info;  // <lower ,upper>
    PostProcessBound(soft_bounds_[i], soft_bound, soft_bound_info);
    if (i == 0) {
      ProtectBoundByInitPoint(soft_bound, soft_bound_info);
    }
    frenet_soft_bounds.emplace_back(soft_bound);
    soft_bounds_info.emplace_back(soft_bound_info);
  }

  assert(frenet_hard_bounds.size() == ref_traj_points_.size());
  assert(frenet_soft_bounds.size() == ref_traj_points_.size());
}

void GeneralLateralDecider::ProtectBoundByInitPoint(std::pair<double, double> &bound, std::pair<BoundInfo, BoundInfo> &bound_info) {
  const double planning_init_point_l = ego_frenet_state_.planning_init_point().frenet_state.r;
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
    std::vector<WeightedBound> &bounds_input,
    std::pair<double, double> &bound_output,
    std::pair<BoundInfo, BoundInfo> &bound_info) {
  auto compare_bound_priority = [&](WeightedBound bound1,
                                    WeightedBound bound2) {
    return general_lateral_decider_utils::GetBoundTypePriority(
               bound1.bound_info.type) >
           general_lateral_decider_utils::GetBoundTypePriority(
               bound2.bound_info.type);
  };
  BoundInfo tmp_lower_bound_info;
  BoundInfo tmp_upper_bound_info;
  std::sort(bounds_input.begin(), bounds_input.end(), compare_bound_priority);
  double min_lower = std::max(bounds_input.front().lower, bound_output.first);
  double max_upper = std::min(bounds_input.front().upper, bound_output.second);
  for (auto &bound : bounds_input) {
    if ((bound.weight < 0.0) &&
        ((bound.bound_info.type != BoundType::ROAD_BORDER) &&
         (bound.bound_info.type !=
          BoundType::AGENT))) {  // hack: only road border in hard bounds
      continue;
    }
    double lower = bound.lower;
    double upper = bound.upper;
    const int bound_priority =
        general_lateral_decider_utils::GetBoundTypePriority(
            bound.bound_info.type);
    const int lower_bound_priority =
        general_lateral_decider_utils::GetBoundTypePriority(
            tmp_lower_bound_info.type);
    const int upper_bound_priority =
        general_lateral_decider_utils::GetBoundTypePriority(
            tmp_upper_bound_info.type);
    if (bound_priority < lower_bound_priority) {
      min_lower = bound_output.first;
    }
    if (bound_priority < upper_bound_priority) {
      max_upper = bound_output.second;
    }
    // 处理同一类型bound的lower和upper
    if (lower > upper) {
      double mid_bound = 0.5 * (lower + upper);
      lower = mid_bound;
      upper = mid_bound;
    }
    // 处理不同类型bound的lower和upper
    if (lower > bound_output.second) {
      if (bound_priority < upper_bound_priority) {
        lower = bound_output.second;
      } else if (bound_priority == upper_bound_priority) {
        double mid_bound =
            std::min(std::max(0.5 * (lower + bound_output.second), min_lower),
                     max_upper);
        lower = mid_bound;
        bound_output.second = mid_bound;
      }
    }
    if (upper < bound_output.first) {
      if (bound_priority < lower_bound_priority) {
        upper = bound_output.first;
      } else if (bound_priority == lower_bound_priority) {
        double mid_bound = std::min(
            std::max(0.5 * (upper + bound_output.first), min_lower), max_upper);
        upper = mid_bound;
        bound_output.first = mid_bound;
      }
    }
    // 选取最值
    if (bound.lower > bound_output.first) {
      bound_output.first = lower;
      tmp_lower_bound_info.id = bound.bound_info.id;
      tmp_lower_bound_info.type = bound.bound_info.type;
    }
    if (bound.upper < bound_output.second) {
      bound_output.second = upper;
      tmp_upper_bound_info.id = bound.bound_info.id;
      tmp_upper_bound_info.type = bound.bound_info.type;
    }
  }
  bound_info.first = tmp_lower_bound_info;
  bound_info.second = tmp_upper_bound_info;
}

void GeneralLateralDecider::SaveLatDebugInfo(
    const std::vector<std::pair<double, double>> &frenet_soft_bounds,
    const std::vector<std::pair<double, double>> &frenet_hard_bounds,
    std::vector<std::pair<BoundInfo, BoundInfo>> &soft_bounds_info,
    std::vector<std::pair<BoundInfo, BoundInfo>> &hard_bounds_info) {
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
  }

  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_lateral_behavior_debug_info()
      ->CopyFrom(lat_debug_info_);
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

  const std::shared_ptr<KDPath> frenet_coord =
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
  auto &enu_ref_path = general_lateral_decider_output.enu_ref_path;
  enu_ref_path.resize(ref_traj_points_.size());

  for (size_t i = 0; i < ref_traj_points_.size(); i++) {
    enu_ref_path[i].first = ref_traj_points_[i].x;
    enu_ref_path[i].second = ref_traj_points_[i].y;
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
  std::vector<planning::PathPoint> path_points;
  if (flane != nullptr && flane->get_reference_path() != nullptr) {
    auto &ref_path = flane->get_reference_path();
    for (auto &ref_point : ref_path->get_points()) {
      path_points.emplace_back(ref_point.path_point);
    }
  }
  // scenario, left_faster, right_is_faster
  lateral_output.scenario = lane_change_decider_output.scenario;
  lateral_output.left_faster = lane_change_decider_output.left_is_faster;
  lateral_output.right_faster = lane_change_decider_output.right_is_faster;
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
            MSD_LANE_TYPE_NON_MOTOR)) &&
      ((!isRedLightStop && lateral_output.accident_ahead &&
        lead_one != nullptr && lead_one->type == 20001))) {
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
            MSD_LANE_TYPE_NON_MOTOR)) &&
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
  lateral_output.isFasterStaticAvd =
      (left_direct_exist && lateral_output.left_faster) ||
      (right_direct_exist && lateral_output.right_faster) ||
      (curr_direct_has_right && !curr_direct_has_straight) ||
      (is_right_turn && left_direct_has_straight && lateral_output.left_faster);
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

bool GeneralLateralDecider::IsFilterForStaticObstacle(
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
      otype == iflyauto::ObjectType::OBJECT_TYPE_TRAFFIC_TEM_SIGN) {
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

bool GeneralLateralDecider::IsFilterForDynamicObstacle(
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
      otype == iflyauto::ObjectType::OBJECT_TYPE_TRAFFIC_TEM_SIGN) {
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
}  // namespace planning
