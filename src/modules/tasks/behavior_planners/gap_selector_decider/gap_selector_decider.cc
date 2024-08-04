#include "gap_selector_decider.h"

#include <fastrtps/config.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "basic_types.pb.h"
// #include "behavior_planners/gap_selector_decider/gap_selector_debug_info.h"
#include "config/basic_type.h"
#include "define/geometry.h"
#include "ego_planning_config.h"
#include "ego_state_manager.h"
#include "environmental_model.h"
#include "refline.h"
#include "tasks/behavior_planners/gap_selector_decider/gap_selector_interface.h"
// #include "gap_selector.pb.h"
#include "debug_info_log.h"
#include "ifly_time.h"
#include "log.h"
#include "math/linear_interpolation.h"
#include "math_lib.h"
#include "quintic_poly_path.h"
#include "reference_path_manager.h"
#include "speed/st_point.h"
// #include "trajectory1d/second_order_time_optimal_trajectory.h"
// #include "trajectory1d/trajectory1d.h"

#include "planning_context.h"
#include "utils/frenet_coordinate_system.h"
#include "utils/kd_path.h"
#include "utils/spline.h"
#include "virtual_lane.h"
#include "virtual_lane_manager.h"
namespace planning {

using namespace planning_math;
using namespace pnc::spline;
using namespace pnc::mathlib;

constexpr int PointNum = 25;
constexpr double delta_t = 0.2;
constexpr double kFilterFrontCarDistace = 50.;
constexpr double kMaxNegativeJerk = -4.;
constexpr double kMaxPositiveJerk = 4.;
constexpr double kMaxDecel = -5.;
constexpr double kMaxAcc = 5.;
constexpr double kPlanEps = 0.1;
constexpr double kOvertakeSpeedThreshld = 2.;
constexpr double kCarCollisionThreshld = 1.0;
constexpr double kMaxTotalLCTime = 35;
constexpr double kHalfEgoLength = 2.55;
constexpr double kHalfEgoWidth = 1.1;
constexpr double kMaxExpectedLCTime = 8.5;
constexpr double kMinExpectedLCTime = 6.0;
constexpr double kMinSplineSampleLength = 30.0;
constexpr double kMaxSplineSampleLength = 60.0;
constexpr double kDefaultLatMovedDistance = 7.0;
constexpr double kDeg2Rad = 0.01745;
constexpr double kRad2Deg = 57.3;

GapSelectorDecider::GapSelectorDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<GapSelectorConfig>();
  name_ = "GapSelectorDecider";

  base_frenet_coord_ = std::make_shared<KDPath>();
  agent_node_mgr_ = std::make_shared<AgentNodeManager>();

  current_lane_coord_ptr_ = nullptr;
  origin_lane_coord_ptr_ = nullptr;
  target_lane_coord_ptr_ = nullptr;

  gap_selector_state_machine_info_.ego_l_buffer = {0., 0., 0.};
  gap_selector_state_machine_info_.lc_request_buffer = {0, 0, 0};

  target_lane_s_width_.clear();
  origin_lane_s_width_.clear();

  _LB_T_.emplace_back(config_.lb_t_min);
  _LB_T_.emplace_back(config_.lb_t_max);
  _LB_HEADING_ERROR_.emplace_back(config_.lb_heading_error_min);
  _LB_HEADING_ERROR_.emplace_back(config_.lb_heading_error_max);
};

bool GapSelectorDecider::Execute() {
  LOG_DEBUG("=======GapSelectorDecider======= \n");

  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }

  Update();

  //   GapSelectorDeciderOutput &gap_selector_decider_output =
  //       session_->mutable_planning_context()
  //           ->mutable_gap_selector_decider_output();

  //   // Preprocess Part:below is the upstream info, these need to be decoupled
  //   by
  //   // interface
  //   // -----------------------------------------

  //   auto start_time = IflyTime::Now_ms();

  //   Preprocessor();

  //   auto preprocessor_end_time = IflyTime::Now_ms();
  //   JSON_DEBUG_VALUE("GapSelectorPreprocessCostTime",
  //                    preprocessor_end_time - start_time);
  //   // update
  //   GapSelectorStatus gap_status = Update(gap_selector_decider_output);

  // #ifdef __COLLECT_GAP_SELECTOR_DEBUG_INFO__
  //   gap_selector_interface_.Store(session_,
  //   gap_selector_state_machine_info_);
  //   gap_selector_interface_.Store(path_spline_, *traj_points_ptr_);
  // #endif

  // #ifdef __COLLECT_GAP_SELECTOR_REPLAY_INFO__
  //   gap_selector_interface_.ReplayCollect(
  //       *traj_points_ptr_, gap_selector_state_machine_info_, path_spline_,
  //       gap_list_, nearby_gap_, front_gap_car_st_boundaries_,
  //       rear_gap_car_st_boundaries_, front_careful_car_st_boundary_,
  //       front_careful_car_id_origin_lane_, st_time_optimal_, gap_status,
  //       front_car_dynamic_dis_, rear_car_dynamic_dis_,
  //       ego_lane_car_dynamic_dis_);

  // #endif
  //   auto all_task_end_time = IflyTime::Now_ms();
  //   JSON_DEBUG_VALUE("GapSelectorTaskCostTime", all_task_end_time -
  //   start_time);
  return true;
}

void GapSelectorDecider::Preprocessor() {
  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  target_state_ =
      coarse_planning_info.target_state == ROAD_LC_LCHANGE
          ? 1
          : (coarse_planning_info.target_state == ROAD_LC_RCHANGE ? 2 : 0);
  const std::shared_ptr<EgoStateManager> ego_state_mgr =
      session_->mutable_environmental_model()->get_ego_state_manager();
  const std::shared_ptr<VirtualLaneManager> virtual_lane_mgr =
      session_->mutable_environmental_model()->get_virtual_lane_manager();

  gap_selector_state_machine_info_.current_lane_id =
      session_->environmental_model()
          .get_virtual_lane_manager()
          ->current_lane_virtual_id();

  if (target_state_ != 0) {  // @cailiu2:
    gap_selector_state_machine_info_.target_lane_id =
        session_->planning_context()
            .lane_change_decider_output()
            .target_lane_virtual_id;
    gap_selector_state_machine_info_.origin_lane_id =
        session_->planning_context()
            .lane_change_decider_output()
            .origin_lane_virtual_id;
  } else {
    gap_selector_state_machine_info_.target_lane_id =
        gap_selector_state_machine_info_.current_lane_id;
    gap_selector_state_machine_info_.origin_lane_id =
        gap_selector_state_machine_info_.current_lane_id;
  }

  std::shared_ptr<ReferencePath> target_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(
              gap_selector_state_machine_info_.target_lane_id, false);

  if (target_refline != nullptr) {
    ids_obstacle_in_target_lane_ =
        target_refline->mutable_obstacles_in_lane_map();
    target_lane_width_ =
        virtual_lane_mgr
            ->get_lane_with_virtual_id(
                gap_selector_state_machine_info_.target_lane_id)
            ->width_by_s(0.);  // near s
    target_lane_coord_ptr_ = target_refline->get_frenet_coord();
    use_query_lane_width_ = true;

    target_lane_s_width_.clear();
    target_lane_s_width_.reserve(target_refline->get_points().size());
    for (auto i = 0; i < target_refline->get_points().size(); i++) {
      const ReferencePathPoint &ref_path_point =
          target_refline->get_points()[i];
      target_lane_s_width_.emplace_back(std::make_pair(
          ref_path_point.path_point.s, ref_path_point.lane_width));
    }
  } else {
    ids_obstacle_in_target_lane_.emplace_back(-1);  // no obstacle in targe lane
    target_lane_coord_ptr_ = current_lane_coord_ptr_;
    use_query_lane_width_ = false;
    target_lane_width_ = 3.5;
  }

  std::shared_ptr<ReferencePath> origin_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(
              gap_selector_state_machine_info_.origin_lane_id, false);
  std::vector<int> origin_obstacles_in_lane_map;
  if (origin_refline != nullptr) {
    ids_obstacle_in_origin_lane_ =
        origin_refline->mutable_obstacles_in_lane_map();
    origin_lane_coord_ptr_ = origin_refline->get_frenet_coord();
    origin_lane_width_ =
        virtual_lane_mgr
            ->get_lane_with_virtual_id(
                gap_selector_state_machine_info_.origin_lane_id)
            ->width_by_s(0.);  // near s
    use_query_lane_width_ = use_query_lane_width_ ? true : false;

    origin_lane_s_width_.clear();
    origin_lane_s_width_.reserve(origin_refline->get_points().size());
    for (auto i = 0; i < origin_refline->get_points().size(); i++) {
      const ReferencePathPoint &ref_path_point =
          origin_refline->get_points()[i];
      origin_lane_s_width_.emplace_back(std::make_pair(
          ref_path_point.path_point.s, ref_path_point.lane_width));
    }
  } else {
    ids_obstacle_in_origin_lane_.emplace_back(-1);  // no obstacle in targe lane
    origin_lane_width_ = 3.5;
    origin_lane_coord_ptr_ = current_lane_coord_ptr_;
    use_query_lane_width_ = false;
  }

#ifdef __FUNCTION_CONSUMPTION_COLLECT__
  auto gs_care_copy_start = IflyTime::Now_ms();
#endif

  gs_care_obstacles_ = session_->mutable_environmental_model()
                           ->get_obstacle_manager()
                           ->get_gs_care_obstacles()
                           .Dict();
#ifdef __FUNCTION_CONSUMPTION_COLLECT__
  auto gs_care_copy_end = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("GSCareObjsCopyTime", gs_care_copy_end - gs_care_copy_start);
#endif

  cruise_vel_ = ego_state_mgr->ego_v_cruise();
  ego_l_cur_lane_ = session_->environmental_model()
                        .get_reference_path_manager()
                        ->get_reference_path_by_current_lane()
                        ->get_frenet_ego_state()
                        .l();

  // set all virtual lanes info
  current_lane_coord_ptr_ = virtual_lane_mgr->get_current_lane()
                                ->get_reference_path()
                                ->get_frenet_coord();
  ego_cart_point_ = {ego_state_mgr->ego_pose().x, ego_state_mgr->ego_pose().y};
  // transform
  transform_.SetEgoState(ego_state_mgr->heading_angle(),
                         ego_state_mgr->ego_pose().x,
                         ego_state_mgr->ego_pose().y);
  refline_transform_.SetEgoState(
      current_lane_coord_ptr_->GetPathCurveHeading(10.),
      planning_init_point_.lat_init_state.x(),
      planning_init_point_.lat_init_state.y());
  traj_points_ptr_ = &(session_->mutable_planning_context()
                           ->mutable_lane_change_decider_output()
                           .coarse_planning_info.trajectory_points);
  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  planning_init_point_ =
      reference_path_ptr->get_frenet_ego_state().planning_init_point();
  ego_planning_init_s_[1] = ego_state_mgr->ego_v();
  ego_planning_init_s_[2] = ego_state_mgr->ego_acc();
  return;
}

GapSelectorStatus GapSelectorDecider::Update() {
  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  auto &traj_points = session_->mutable_planning_context()
                          ->mutable_lane_change_decider_output()
                          .coarse_planning_info.trajectory_points;
  auto &gap_selector_decider_output =
      session_->mutable_planning_context()
          ->mutable_gap_selector_decider_output();
  bool is_lc_scene{false};
  bool is_lc_back_scene{false};
  // lat avoid scene
  const LateralOffsetDeciderOutput &lateral_offset_decider_output =
      session_->mutable_planning_context()->lateral_offset_decider_output();
  double avoid_lat_offset = lateral_offset_decider_output.is_valid
                                ? lateral_offset_decider_output.lateral_offset
                                : 0.0;
  const auto &ego_state_mgr =
      session_->mutable_environmental_model()->get_ego_state_manager();
  Point2D ego_cart_pose{ego_state_mgr->ego_carte().x,
                        ego_state_mgr->ego_carte().y};
  Point2D ego_frenet_pose;
  const double ego_cart_heading = ego_state_mgr->heading_angle();
  coarse_planning_info.reference_path->get_frenet_coord()->XYToSL(
      ego_cart_pose, ego_frenet_pose);

  if (coarse_planning_info.target_state != last_target_state_) {
    if (coarse_planning_info.target_state == ROAD_NONE ||
        coarse_planning_info.target_state == ROAD_LC_LWAIT ||
        coarse_planning_info.target_state == ROAD_LC_RWAIT) {
      lc_timer_ = 0.;
      lc_total_time_ = config_.default_lc_time;
      lc_back_timer_ = 0.;
      lc_back_vel_ = 0.;
      use_ego_v_ = false;
      return GapSelectorStatus::NO_LC_REQUSET;
    } else if (coarse_planning_info.target_state == ROAD_LC_LBACK ||
               coarse_planning_info.target_state == ROAD_LC_RBACK) {
      lc_timer_ = 0.;
      lc_total_time_ = config_.default_lc_time;
      use_ego_v_ = false;
      // cart pose and heading
      // calc lc back duration
      const double ref_line_theta =
          coarse_planning_info.reference_path->get_frenet_coord()
              ->GetPathCurveHeading(ego_frenet_pose.x);

      const double ego_theta_error =
          std::fabs(ref_line_theta - ego_cart_heading) * kRad2Deg;
      if (ego_theta_error < config_.lb_heading_error_min) {
        lc_back_total_time_ = 0.;
      } else {
        lc_back_total_time_ =
            interp(ego_theta_error, _LB_HEADING_ERROR_, _LB_T_);
        lc_back_vel_ = ego_frenet_pose.y / lc_back_total_time_;
      }
      lc_back_timer_ = 0.;
      is_lc_back_scene = true;

    } else {
      lc_timer_ += 0.1;
      is_lc_scene = true;
    }
  } else if (coarse_planning_info.target_state == ROAD_LC_LCHANGE ||
             coarse_planning_info.target_state == ROAD_LC_RCHANGE) {
    lc_timer_ += 0.1;
    is_lc_scene = true;
  } else if (coarse_planning_info.target_state == ROAD_LC_LBACK ||
             coarse_planning_info.target_state == ROAD_LC_RBACK) {
    lc_back_timer_ += 0.1;
    is_lc_back_scene = true;
  }

  gap_selector_decider_output.gap_selector_trustworthy = false;
  if (is_lc_scene) {
    double lc_end_s, remain_lc_time = lc_total_time_ - lc_timer_;
    RefineLCTime(&lc_end_s, &remain_lc_time, avoid_lat_offset);

    FixedTimeQuinticPathPlan(avoid_lat_offset, lc_end_s, remain_lc_time,
                             traj_points);

    // bool is_left =
    //     coarse_planning_info.target_state == ROAD_LC_LCHANGE ? true : false;

    // bool is_quintic_valid = RecheckQuinticValid(is_left, &traj_points);
    // if (!is_quintic_valid) {
    //   GenerateLinearRefTrajectory(is_left, traj_points);
    // }

    gap_selector_decider_output.gap_selector_trustworthy =
        remain_lc_time < 1.0 ? false : true;

  } else if (is_lc_back_scene) {
    double lb_end_s, lb_target_l,
        remain_lb_time = lc_back_total_time_ - lc_back_timer_;
    if (remain_lb_time > 1.0) {
      lb_target_l =
          lc_back_vel_ * lc_back_total_time_ - lc_back_vel_ * lc_back_timer_;

      FixedTimeQuinticPathPlan(lb_target_l, lb_end_s, remain_lb_time,
                               traj_points);
      gap_selector_decider_output.gap_selector_trustworthy = true;
    }
  }

  last_target_state_ = coarse_planning_info.target_state;
  return GapSelectorStatus::DEFAULT;
}

bool GapSelectorDecider::RecheckQuinticValid(const bool is_left,
                                             TrajectoryPoints *traj_points) {
  int max_check_idx = 15;
  double max_l_threhold = 0.2;
  bool is_quintic_valid = true;
  for (auto i = 0; i < max_check_idx; i++) {
    const auto &traj_point = traj_points->at(i);
    if (is_left) {
      is_quintic_valid = traj_point.l < max_l_threhold;
    } else {
      is_quintic_valid = traj_point.l > -max_l_threhold;
    }
    if (!is_quintic_valid) return false;
  }
  return is_quintic_valid;
}

void GapSelectorDecider::FixedTimeQuinticPathPlan(
    const double lat_avoid_offset, const double lc_end_s,
    const double remain_lc_duration, TrajectoryPoints &traj_points) {
  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  const std::shared_ptr<EgoStateManager> ego_state_mgr =
      session_->mutable_environmental_model()->get_ego_state_manager();
  if (config_.use_ego_v) {
    use_ego_v_ = true;
  }
  auto ego_v =
      use_ego_v_ ? ego_state_mgr->ego_v() : ego_state_mgr->ego_v_cruise();

  auto lat_state = ego_state_mgr->planning_init_point().lat_init_state;
  Point2D frenet_init_point;
  Point2D cart_init_point{lat_state.x(), lat_state.y()};
  const auto &coord =
      session_->planning_context()
          .lane_change_decider_output()
          .coarse_planning_info.reference_path->get_frenet_coord();
  if (!coord->XYToSL(cart_init_point, frenet_init_point)) {
    LOG_ERROR("ERROR! Frenet Point -> Cart Point Failed!!!");
  }
  // if reamining duration less than 1s, then aggresively lane change
  if (remain_lc_duration < 1.0) {
    return;
  }

  // refine velocity if frenet length is short
  const auto curvature = lat_state.curv();
  const auto heading_angle = lat_state.theta();

  const auto normal_acc = ego_v * ego_v * curvature;

  // const auto &heading_angle =
  // ego_state->planning_init_point().heading_angle; const auto normal_acc
  // = ego_v * ego_v * curvature;
  if (frenet_init_point.x + ego_v * remain_lc_duration >=
      coord->Length() - 0.5) {
    ego_v = (coord->Length() - frenet_init_point.x - 0.5) /
            std::fmax(remain_lc_duration, (traj_points.size() - 1) * 0.2);
  }
  auto lane_change_end_s = frenet_init_point.x + ego_v * remain_lc_duration;

  const auto v_x = ego_v * std::cos(heading_angle);
  const auto v_y = ego_v * std::sin(heading_angle);
  auto normal_acc_x = normal_acc * std::sin(heading_angle);
  auto normal_acc_y = normal_acc * std::cos(heading_angle);

  // set lane change end state
  Point2D frenet_end_point{lane_change_end_s, lat_avoid_offset};
  Point2D cart_end_point;
  if (!coord->SLToXY(frenet_end_point, cart_end_point)) {
    LOG_ERROR("ERROR! Frenet Point -> Cart Point Failed!!!");
  }

  const auto lane_change_end_heading_angle =
      coord->GetPathCurveHeading(lane_change_end_s);
  double lane_change_end_curvature{0.};
  coord->GetKappaByS(lane_change_end_s, &lane_change_end_curvature);

  const auto normal_acc_end = ego_v * ego_v * lane_change_end_curvature;

  const auto v_x_end = ego_v * std::cos(lane_change_end_heading_angle);
  const auto v_y_end = ego_v * std::sin(lane_change_end_heading_angle);
  auto normal_acc_x_end =
      normal_acc_end * std::sin(lane_change_end_heading_angle);
  auto normal_acc_y_end =
      normal_acc_end * std::cos(lane_change_end_heading_angle);

  // construct quintic path for lane change duration
  QuinticPolynominalPath lane_change_quintic_path;

  Eigen::Vector2d x0(ego_state_mgr->planning_init_point().x,
                     ego_state_mgr->planning_init_point().y);
  Eigen::Vector2d dx0(v_x, v_y);
  Eigen::Vector2d ddx0(normal_acc_x, normal_acc_y);
  Eigen::Vector2d xT(cart_end_point.x, cart_end_point.y);
  Eigen::Vector2d dxT(v_x_end, v_y_end);
  Eigen::Vector2d ddxT(normal_acc_x_end, normal_acc_y_end);
  lane_change_quintic_path.SetPoints(x0, xT, dx0, dxT, ddx0, ddxT,
                                     remain_lc_duration);

  // sample traj point on quintic path by t
  TrajectoryPoint point;
  Eigen::Vector2d sample_point;
  Point2D frenet_point;
  size_t truncation_idx = 0;
  for (size_t i = 0; i < traj_points.size(); i++) {
    if (traj_points[i].t < remain_lc_duration) {
      sample_point = lane_change_quintic_path(traj_points[i].t);
      point.x = sample_point.x();
      point.y = sample_point.y();
      point.heading_angle = lane_change_quintic_path.heading(traj_points[i].t);

      Point2D cart_point(point.x, point.y);
      if (!coord->XYToSL(cart_point, frenet_point)) {
        LOG_ERROR("ERROR! Frenet Point -> Cart Point Failed!!!");
      }
      point.s = frenet_point.x;
      point.l = frenet_point.y;
      point.t = traj_points[i].t;

      traj_points[i] = point;
      truncation_idx = i;
    } else {
      double s_truncation;
      if (i == truncation_idx + 1) {
        Eigen::Vector2d truncation_point(traj_points[truncation_idx].x,
                                         traj_points[truncation_idx].y);
        auto &cart_ref_info = coarse_planning_info.cart_ref_info;

        pnc::spline::Projection projection_truncation_point;
        projection_truncation_point.CalProjectionPoint(
            cart_ref_info.x_s_spline, cart_ref_info.y_s_spline,
            cart_ref_info.s_vec.front(), cart_ref_info.s_vec.back(),
            truncation_point);

        s_truncation = projection_truncation_point.GetOutput().s_proj;
      }
      point.s = std::fmin(s_truncation + ego_v * (i - truncation_idx) * 0.2,
                          coord->Length());
      point.l = lat_avoid_offset;
      point.t = traj_points[i].t;

      frenet_point.x = point.s;
      frenet_point.y = point.l;
      Point2D cart_point;
      if (!coord->SLToXY(frenet_point, cart_point)) {
        LOG_ERROR("ERROR! Cart Point -> Frenet Point Failed!!!");
      }
      point.x = cart_point.x;
      point.y = cart_point.y;

      point.heading_angle = coord->GetPathCurveHeading(point.s);
      traj_points[i] = point;
    }
  }
  return;
}

void GapSelectorDecider::RefineLCTime(double *lc_end_s, double *remain_lc_time,
                                      const double lat_avoid_offset) {
  if (*remain_lc_time > 1.0 || use_ego_v_) {
    return;
  }

  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  const auto &target_state = coarse_planning_info.target_state;
  const auto target_l = lat_avoid_offset;
  const auto &planning_init_point = session_->mutable_environmental_model()
                                        ->get_ego_state_manager()
                                        ->planning_init_point();

  Point2D frenet_init_point;
  Point2D cart_init_point{planning_init_point.lat_init_state.x(),
                          planning_init_point.lat_init_state.y()};
  const auto &coord =
      session_->planning_context()
          .lane_change_decider_output()
          .coarse_planning_info.reference_path->get_frenet_coord();

  if (!coord->XYToSL(cart_init_point, frenet_init_point)) {
    LOG_ERROR("ERROR! Frenet Point -> Cart Point Failed!!!");
  }
  // frenet_init_point_ = frenet_init_point;

  const double lat_lc_dis =
      target_state == ROAD_LC_LCHANGE
          ? std::fabs(lat_avoid_offset - frenet_init_point.y)
          : std::fabs(frenet_init_point.y - lat_avoid_offset);
  const double ego_v =
      session_->mutable_environmental_model()->get_ego_state_manager()->ego_v();
  const double v_cruise = session_->mutable_environmental_model()
                              ->get_ego_state_manager()
                              ->ego_v_cruise();
  const double default_lc_lat_v = 3.8 / config_.default_lc_time;
  if (lat_lc_dis <= 0.6) {
    return;
  }

  *remain_lc_time = lat_lc_dis / default_lc_lat_v;
  lc_total_time_ = *remain_lc_time;
  lc_timer_ = 0.0;
  use_ego_v_ = true;
  return;
}

GapSelectorStatus GapSelectorDecider::Update(
    GapSelectorDeciderOutput &gap_selector_decider_output) {
  // -----------------------------------------
  GapSelectorStatus env_preprocessor_status =
      EnvHandle(target_state_, Point2D{ego_cart_point_[0], ego_cart_point_[1]},
                gap_selector_decider_output);

  if (env_preprocessor_status == GS_SKIP ||
      env_preprocessor_status == LC_CANCEL ||
      env_preprocessor_status == BASE_FRENET_COORD_NULLPTR ||
      env_preprocessor_status == LC_FINISHED ||
      env_preprocessor_status == LC_PASS_TIME_EXCEED_THRESHOLD) {
    gap_selector_decider_output = {false, target_state_, false, false};
    LOG_DEBUG("Gap Selector finished after env preprocessor, the status is: %d",
              (int)env_preprocessor_status);
    return env_preprocessor_status;
  }
  GapSelectorStatus status = GapSelectorStatus::DEFAULT;
  // generate gap info
  ConstructGaps();

  if (gap_selector_state_machine_info_.lc_premove_time <
      config_.lc_premove_time) {  // no collision check when no gap
                                  // exist gap, then lateral premove

    status = GapSelectorStatus::LC_LATERAL_PREMOVE;
    gap_selector_state_machine_info_.lc_premove_time += 0.1;
    // GenerateLinearRefTrajectory(*traj_points_ptr_);
    return status;
  }
  // generate candidate lateral path
  ResponsivePathPlan();  // generate path plan responsive

  // get the nearby gap
  ObtainNearbyGap();

  if (nearby_gap_.front_agent_id > -1 || nearby_gap_.rear_agent_id > -1) {
    if (!PreSafetyCheck()) {
      status = GapSelectorStatus::COLLISION_CHECK_FAILED_PATH;
      path_spline_.path_spline_status = GapSelectorPathSpline::NO_VALID;
      gap_selector_decider_output = {true, target_state_, false, false};
      SecondOrderTimeOptimalTrajectory lane_hold_speed_profile =
          DriveStyleTimeOptimalTrajEvaluation(planning_init_point_.v,
                                              cruise_vel_);
      GenerateLHTrajectory(lane_hold_speed_profile, *traj_points_ptr_);

      return status;
    }
  }

  // make decision of drive style
  MakeDriveStyleDecision();

  if (gap_drive_style_ == GapDriveStyle::Free) {
    status = GapSelectorStatus::FREE_LC;
  }

  if (path_spline_.path_spline_status == GapSelectorPathSpline::LC_VALID ||
      path_spline_.path_spline_status == GapSelectorPathSpline::LC_LANE_CROSS) {
    DriveStyleTimeOptimalTrajEvaluation(gap_selector_path_spline_[0]);
    ConstructTimeOptimal(traj_time_optimal_, gap_selector_path_spline_[0]);
    int collision_id =
        gap_list_.empty() ? -1 : SafetyCheck(gap_selector_path_spline_[0]);
    if (collision_id < 0 &&
        PathSplineLengthCheck(gap_selector_path_spline_[0])) {
      GenerateEgoTrajectory(*traj_points_ptr_);
      status = GapSelectorStatus::COLLISION_CHECK_OK_PATH;
      path_spline_ = gap_selector_path_spline_[0];
      path_spline_.path_spline_status = GapSelectorPathSpline::LC_VALID;
      gap_selector_decider_output = {true, target_state_, true, true};
      // planning::common::GapSelectorPathSpline::LC_VALID;
      gap_selector_state_machine_info_.lc_pass_time += 0.1;

    } else {
      status = GapSelectorStatus::COLLISION_CHECK_FAILED_PATH;
      path_spline_.path_spline_status = GapSelectorPathSpline::NO_VALID;
    }
    gap_selector_state_machine_info_.lc_premove_time =
        config_.lc_premove_time + 0.1;  // in this case, not premove
  }

  if (path_spline_.path_spline_status == GapSelectorPathSpline::NO_VALID) {
    SecondOrderTimeOptimalTrajectory lane_hold_speed_profile =
        DriveStyleTimeOptimalTrajEvaluation(planning_init_point_.v,
                                            cruise_vel_);
    GenerateLHTrajectory(lane_hold_speed_profile, *traj_points_ptr_);
    gap_drive_style_ = GapDriveStyle::NONESTYLE;
    path_spline_.path_spline_status = GapSelectorPathSpline::LH_VALID;
    status = GapSelectorStatus::COLLISION_CHECK_FAILED_PATH;
    gap_selector_decider_output = {true, target_state_, true, false};
  }

  RestoreTrajResult(*traj_points_ptr_);

  return status;
}

GapSelectorStatus GapSelectorDecider::EnvHandle(
    const int target_state, const Point2D &ego_cart_pose,
    GapSelectorDeciderOutput &gap_selector_decider_output) {
  //
  UpdateSequenceInfos();

  CheckLaneCrossed();

  if (!SetBaseFrenetCoordAndUpdateAgentNode()) {
    LOG_ERROR("Enmergency error! Agent Node updated false!");
  }

  Point2D ego_frenet_point;
  Point2D ego_cart_point{planning_init_point_.lat_init_state.x(),
                         planning_init_point_.lat_init_state.y()};
  if (!base_frenet_coord_->XYToSL(ego_cart_point, ego_frenet_point)) {
    LOG_ERROR("Enmergency error! Ego Pose Cart2SL failed!");
  }  // upstream default reference path
  planning_init_point_.frenet_state.s = ego_frenet_point.x;
  planning_init_point_.frenet_state.r = ego_frenet_point.y;

  ego_l_ = ego_frenet_point.y;
  ego_planning_init_s_[0] = planning_init_point_.frenet_state.s;

  // no lc case
  if (target_state_ == 0) {
    path_spline_.path_spline_status = GapSelectorPathSpline::NO_VALID;
  }

  GapSelectorStatus gs_status{DEFAULT};
  if (gap_selector_state_machine_info_.gs_skip ||
      base_frenet_coord_ == nullptr) {
    ResetAllInfo();
    gap_selector_decider_output = {false, target_state_, false, false};
    LOG_DEBUG("\n Gap selector skip is %d, base_frenet_coord is ok: %d",
              gap_selector_state_machine_info_.gs_skip,
              base_frenet_coord_ == nullptr);
    gs_status = gap_selector_state_machine_info_.gs_skip
                    ? GS_SKIP
                    : BASE_FRENET_COORD_NULLPTR;
  } else if (CheckLCFinish() || gap_selector_state_machine_info_.lc_cancel) {
    ResetAllInfo();
    gap_selector_decider_output = {true, target_state_, false, false};
    LOG_DEBUG("\n Gap selector lc cancel %d",
              gap_selector_state_machine_info_.lc_cancel);
    gs_status =
        gap_selector_state_machine_info_.lc_cancel ? LC_CANCEL : LC_FINISHED;
  } else if (gap_selector_state_machine_info_.lc_pass_time > kMaxTotalLCTime &&
             !gap_selector_state_machine_info_.lane_cross) {
    GenerateLBTrajectory(*traj_points_ptr_);
    path_spline_.path_spline_status = GapSelectorPathSpline::LB_VALID;
    gap_selector_decider_output = {false, target_state_, true, false};
    LOG_DEBUG("Lc time is exceed max , LB Launch !");
    gs_status = LC_PASS_TIME_EXCEED_THRESHOLD;
  }
  return gs_status;
}

void GapSelectorDecider::ResetAllInfo() {
  gap_selector_state_machine_info_ = {false,
                                      false,
                                      false,
                                      false,
                                      false,
                                      0.,
                                      0.,
                                      0.,
                                      false,
                                      false,
                                      false,
                                      {0, 0, 0},
                                      {ego_l_, ego_l_, ego_l_}};

  st_lower_boundaries_.clear();
  st_upper_boundaries_.clear();
  st_time_optimal_.clear();
  loninfo_time_optimal_.clear();

  front_gap_car_st_boundaries_.clear();
  gs_status_ = GapSelectorPathSpline::NO_VALID;

  gap_list_.clear();
  nearby_gap_.front_agent_id = -1;
  nearby_gap_.rear_agent_id = -1;
  target_state_ = 0;
  gap_selector_path_spline_.clear();
  front_careful_car_id_origin_lane_ = -1;
  path_spline_.path_spline_status = GapSelectorPathSpline::NO_VALID;
  front_gap_car_st_boundaries_.clear();
  rear_gap_car_st_boundaries_.clear();
  front_careful_car_st_boundary_.clear();
  return;
}

void GapSelectorDecider::UpdateSequenceInfos() {
  // lc sequence
  gap_selector_state_machine_info_.lc_request_buffer[0] =
      gap_selector_state_machine_info_.lc_request_buffer[1];
  gap_selector_state_machine_info_.lc_request_buffer[1] =
      gap_selector_state_machine_info_.lc_request_buffer[2];
  gap_selector_state_machine_info_.lc_request_buffer[2] = target_state_;

  if (gap_selector_state_machine_info_.lc_request_buffer[2] ==
          gap_selector_state_machine_info_.lc_request_buffer[1] &&
      gap_selector_state_machine_info_.lc_request_buffer[1] != 0) {
    gap_selector_state_machine_info_.lc_triggered = false;
    gap_selector_state_machine_info_.lb_triggered = false;
    // gap_selector_state_machine_info_.lc_pass_time += 0.1;
  } else if (gap_selector_state_machine_info_.lc_request_buffer[2] !=
                 gap_selector_state_machine_info_.lc_request_buffer[1] &&
             gap_selector_state_machine_info_.lc_request_buffer[2] != 0) {
    gap_selector_state_machine_info_.lc_triggered = true;
    gap_selector_state_machine_info_.lc_pass_time = 0.;
    gap_selector_state_machine_info_.lc_wait_time = 0.;
    gap_selector_state_machine_info_.lc_premove_time = 0.;
  } else if (gap_selector_state_machine_info_.lc_request_buffer[2] !=
                 gap_selector_state_machine_info_.lc_request_buffer[1] &&
             gap_selector_state_machine_info_.lc_request_buffer[2] == 0) {
    gap_selector_state_machine_info_.lb_triggered = true;
    gap_selector_state_machine_info_.lc_pass_time = 0.;
    gap_selector_state_machine_info_.lc_wait_time = 0.;
    gap_selector_state_machine_info_.lc_premove_time = 0.;
  } else if (gap_selector_state_machine_info_.lc_request_buffer[2] == 0) {
    gap_selector_state_machine_info_.lc_triggered = false;
    gap_selector_state_machine_info_.lb_triggered = false;
  }

  if (gs_status_ == GapSelectorPathSpline::LH_VALID) {
    gap_selector_state_machine_info_.lc_wait_time += 0.1;
  } else {
    gap_selector_state_machine_info_.lc_wait_time = 0.;
  }
  if (gap_selector_state_machine_info_.lc_request_buffer[2] == 0 &&
      gap_selector_state_machine_info_.lc_request_buffer[1] != 0) {
    gap_selector_state_machine_info_.lc_cancel = true;
  } else {
    gap_selector_state_machine_info_.lc_cancel = false;
  }

  if (gap_selector_state_machine_info_.lc_request_buffer[2] == 0 &&
      gap_selector_state_machine_info_.lc_request_buffer[1] == 0) {
    gap_selector_state_machine_info_.gs_skip = true;
  } else {
    gap_selector_state_machine_info_.gs_skip = false;
  }
  return;
}

bool GapSelectorDecider::SetBaseFrenetCoordAndUpdateAgentNode() {
  gap_selector_state_machine_info_.path_requintic = false;
  if (!gap_selector_state_machine_info_.lane_cross) {
    base_frenet_coord_ = origin_lane_coord_ptr_;
    if (base_frenet_coord_ == nullptr) {
      gap_selector_state_machine_info_.path_requintic = true;
      base_frenet_coord_ = current_lane_coord_ptr_;
      LOG_ERROR("The original lane coord get nullptr after lane cross!");
    }
  } else {
    base_frenet_coord_ = current_lane_coord_ptr_;
  }

  bool agent_node_updated = {false};

  auto agent_node_start_time = IflyTime::Now_ms();
  if (origin_lane_coord_ptr_ != nullptr && target_lane_coord_ptr_ != nullptr &&
      !gap_selector_state_machine_info_.gs_skip) {
    // agent_node_mgr_->SetOriginLane(origin_lane_coord_ptr_);
    // agent_node_updated =
    //     agent_node_mgr_->Update(target_lane_coord_ptr_, target_state_);

    agent_node_mgr_->set_input_info(
        origin_lane_coord_ptr_, target_lane_coord_ptr_, target_state_,
        ids_obstacle_in_origin_lane_, ids_obstacle_in_target_lane_,
        gs_care_obstacles_);
    agent_node_updated = agent_node_mgr_->Update();
  }
  auto agent_node_end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("AgentNodeCostTime",
                   agent_node_end_time - agent_node_start_time);
  return agent_node_updated;
}

void GapSelectorDecider::CheckLaneCrossed() {
  for (int i = 0; i < gap_selector_state_machine_info_.ego_l_buffer.size() - 1;
       i++) {
    gap_selector_state_machine_info_.ego_l_buffer[i] =
        gap_selector_state_machine_info_.ego_l_buffer[i + 1];
  }
  gap_selector_state_machine_info_.ego_l_buffer.back() = ego_l_cur_lane_;
  const double lane_cross_lat_thres = 1.5;
  bool left_cross_lane =
      gap_selector_state_machine_info_.lc_request_buffer[2] == 1 &&
      gap_selector_state_machine_info_.ego_l_buffer[2] -
              gap_selector_state_machine_info_.ego_l_buffer[1] <
          -lane_cross_lat_thres;
  bool right_cross_lane =
      gap_selector_state_machine_info_.lc_request_buffer[2] == 2 &&
      gap_selector_state_machine_info_.ego_l_buffer[2] -
              gap_selector_state_machine_info_.ego_l_buffer[1] >
          lane_cross_lat_thres;
  bool left_cross_back =
      gap_selector_state_machine_info_.lc_request_buffer[2] == 1 &&
      gap_selector_state_machine_info_.ego_l_buffer[2] -
              gap_selector_state_machine_info_.ego_l_buffer[1] >
          lane_cross_lat_thres;
  bool right_cross_back =
      gap_selector_state_machine_info_.lc_request_buffer[2] == 2 &&
      gap_selector_state_machine_info_.ego_l_buffer[2] -
              gap_selector_state_machine_info_.ego_l_buffer[1] <
          -lane_cross_lat_thres;

  if (!gap_selector_state_machine_info_.lane_cross &&
      (left_cross_lane || right_cross_lane)) {
    gap_selector_state_machine_info_.lane_cross = true;
  } else if (gap_selector_state_machine_info_.lane_cross &&
             (gap_selector_state_machine_info_.lc_request_buffer[2] == 0 ||
              left_cross_back || right_cross_back)) {
    gap_selector_state_machine_info_.lane_cross = false;
  }
  // if (gap_selector_state_machine_info_.lane_cross == false) {
  //   gap_selector_state_machine_info_.lane_cross = BodyCheck();
  // }
  return;
}

void GapSelectorDecider::ConstructGaps() {
  gap_list_.clear();
  const std::vector<int64_t> &ids_sorted_target_lane =
      agent_node_mgr_->ids_sorted_target_lane();
  const std::unordered_map<int64_t, ObstaclePredicatedInfo>
      &agent_target_lane_map = agent_node_mgr_->agent_node_target_lane_map();

  if (ids_sorted_target_lane.empty()) {
    LOG_DEBUG("GapSelectorTask, No gap exists!");
    return;
  }

  gap_list_.reserve(ids_sorted_target_lane.size() + 1);

  for (size_t i = 0; i < ids_sorted_target_lane.size() + 1; i++) {
    const bool is_first_gap = (i == 0);
    const bool is_last_gap = (i + 1 == ids_sorted_target_lane.size() + 1);

    if (is_first_gap) {
      Gap first_gap;
      first_gap.front_agent_id = -1;
      first_gap.rear_agent_id = ids_sorted_target_lane.front();

      auto iter = agent_target_lane_map.find(first_gap.rear_agent_id);
      first_gap.gap_front_s = std::fmax(iter->second.cur_s, 0.1);

      gap_list_.emplace_back(first_gap);
    }

    if (is_last_gap) {
      Gap last_gap;
      last_gap.front_agent_id = ids_sorted_target_lane.back();
      last_gap.rear_agent_id = -1;
      auto iter = agent_target_lane_map.find(last_gap.front_agent_id);
      last_gap.gap_front_s =
          std::fmin(iter->second.cur_s, base_frenet_coord_->Length() - 0.1);
      gap_list_.emplace_back(last_gap);
    }

    if (is_first_gap || is_last_gap) {
      continue;
    }

    Gap normal_gap;
    normal_gap.front_agent_id = ids_sorted_target_lane.at(i - 1);
    normal_gap.rear_agent_id = ids_sorted_target_lane.at(i);

    auto front_iter = agent_target_lane_map.find(normal_gap.front_agent_id);
    auto rear_iter = agent_target_lane_map.find(normal_gap.rear_agent_id);

    auto diff_cur_s = rear_iter->second.cur_s - front_iter->second.cur_s;
    if (diff_cur_s < rear_iter->second.length * 2) {
      continue;  // perception false check!
    }

    if (front_iter != agent_target_lane_map.end() &&
        rear_iter != agent_target_lane_map.end()) {
      normal_gap.gap_front_s = front_iter->second.cur_s;
    }

    gap_list_.emplace_back(normal_gap);
  }
  return;
}

void GapSelectorDecider::ObtainNearbyGap() {
  if (gap_list_.empty()) {
    nearby_gap_.front_agent_id = -1;
    nearby_gap_.rear_agent_id = -1;
    return;
  }

  double ego_s = ego_planning_init_s_[0];
  bool compare_front_obj{true};
  if (gap_list_.size() == 2) {
    if (ego_s < gap_list_[0].gap_front_s) {
      compare_front_obj = false;
    }

    auto obj_id = gap_list_[0].front_agent_id == -1
                      ? gap_list_[0].rear_agent_id
                      : gap_list_[0].front_agent_id;
    nearby_gap_.front_agent_id = compare_front_obj ? obj_id : -1;
    nearby_gap_.rear_agent_id = compare_front_obj ? -1 : obj_id;
  } else {
    auto nearby_gap =
        std::min_element(gap_list_.begin(), gap_list_.end(),
                         [&](const Gap &gap1, const Gap &gap2) {
                           return std::abs(gap1.gap_front_s - ego_s) <
                                  std::abs(gap2.gap_front_s - ego_s);
                         });
    if (ego_s < nearby_gap->gap_front_s) {
      compare_front_obj = false;
    }

    for (auto i = 0; i < gap_list_.size(); i++) {
      auto compare_id = compare_front_obj ? gap_list_[i].front_agent_id
                                          : gap_list_[i].rear_agent_id;
      auto id = nearby_gap->front_agent_id == -1 ? nearby_gap->rear_agent_id
                                                 : nearby_gap->front_agent_id;

      if (compare_id == id) {
        nearby_gap_ = gap_list_[i];
        break;
      }
    }
  }

  // select
  const std::vector<int64_t> &ids_sorted_origin_lane =
      agent_node_mgr_->ids_sorted_origin_lane();
  const std::unordered_map<int64_t, ObstaclePredicatedInfo>
      &agent_node_origin_lane_map =
          agent_node_mgr_->agent_node_origin_lane_map();

  front_careful_car_id_origin_lane_ = -1;

  if (!gap_selector_state_machine_info_.lane_cross) {
    for (size_t i = 0; i < ids_sorted_origin_lane.size(); i++) {
      const auto &obj_pred_info =
          agent_node_origin_lane_map.find(ids_sorted_origin_lane.at(i));
      if (obj_pred_info->second.cur_s > ego_s) {
        front_careful_car_id_origin_lane_ = ids_sorted_origin_lane.at(i);
        break;
      }
    }
  }

  // // if()

  // // select origin lane front car
  // auto agent_on_origin_lane = agent_node_mgr_->agent_node_origin_lane_map();
  // auto nearest_element =
  //     std::min_element(agent_on_origin_lane.begin(),
  //     agent_on_origin_lane.end(),
  //                      [&](const auto &pair1, const auto &pair2) {
  //                        return std::abs(pair1.second.cur_s - ego_s) <
  //                                   std::abs(pair2.second.cur_s - ego_s) &&
  //                               pair1.second.cur_s > ego_s;
  //                      });
  // if (nearest_element != agent_on_origin_lane.end() &&
  //     nearest_element->second.cur_s - ego_s < kFilterFrontCarDistace) {
  //   front_careful_car_id_origin_lane_ = nearest_element->first;
  // } else {
  //   front_careful_car_id_origin_lane_ = -1;
  // }
  return;
}

void GapSelectorDecider::MakeDriveStyleDecision() {
  if (nearby_gap_.rear_agent_id != -1) {
    ObstaclePredicatedInfo rear_obstacle_pred_info =
        agent_node_mgr_->agent_node_target_lane_map()
            .find(nearby_gap_.rear_agent_id)
            ->second;

    if (nearby_gap_.front_agent_id == -1) {
      if (rear_obstacle_pred_info.raw_vel >=
          ego_planning_init_s_[1] + kPlanEps) {
        gap_drive_style_ = GapDriveStyle::OnlyRearFasterCar;

      } else {
        gap_drive_style_ = GapDriveStyle::OnlyRearSlowerCar;
      }
    } else if (nearby_gap_.front_agent_id != -1) {
      ObstaclePredicatedInfo front_obstacle_pred_info =
          agent_node_mgr_->agent_node_target_lane_map()
              .find(nearby_gap_.front_agent_id)
              ->second;

      if (front_obstacle_pred_info.raw_vel <= rear_obstacle_pred_info.raw_vel &&
          rear_obstacle_pred_info.raw_vel >= ego_planning_init_s_[1] &&
          front_obstacle_pred_info.raw_vel <= ego_planning_init_s_[1]) {
        gap_drive_style_ = GapDriveStyle::RearCarFaster;

      } else if (front_obstacle_pred_info.raw_vel >=
                     rear_obstacle_pred_info.raw_vel &&
                 rear_obstacle_pred_info.raw_vel < ego_planning_init_s_[1] &&
                 front_obstacle_pred_info.raw_vel > ego_planning_init_s_[1]) {
        gap_drive_style_ = GapDriveStyle::FrontCarFaster;

      } else {
        gap_drive_style_ = GapDriveStyle::NoDecisionForBothCar;
      }
    }

  } else {
    if (nearby_gap_.front_agent_id != -1) {
      ObstaclePredicatedInfo front_obstacle_pred_info =
          agent_node_mgr_->agent_node_target_lane_map()
              .find(nearby_gap_.front_agent_id)
              ->second;

      if (front_obstacle_pred_info.raw_vel + kPlanEps > ego_planning_init_s_[1]
          //&&front_obstacle_pred_info.cur_s > ego_init_s_[0]
      ) {
        gap_drive_style_ = GapDriveStyle::OnlyFrontFasterCar;

      } else if (front_obstacle_pred_info.raw_vel + kPlanEps <
                 ego_planning_init_s_[1]
                 //&&front_obstacle_pred_info.cur_s > ego_init_s_[0]
                 )  //
      {
        gap_drive_style_ = GapDriveStyle::OnlyFrontSlowerCar;
      }
    } else {
      gap_drive_style_ = GapDriveStyle::Free;
    }
  };
  return;
}

bool GapSelectorDecider::MakeInteractiveDecision(
    const ObstaclePredicatedInfo &interactive_obj_pred_info,
    const GapSelectorPathSpline &path_spline,
    SecondOrderTimeOptimalTrajectory &traj_time_optimal) {
  const Point2D &crossed_line_point =
      path_spline.crossed_line_point_info.crossed_line_point;
  Point2D crossed_line_frenet_point;
  if (!base_frenet_coord_->XYToSL(crossed_line_point,
                                  crossed_line_frenet_point)) {
    LOG_ERROR("Crossed line transform frenet failed!");
    return false;
  }

  if (!path_spline.crossed_line_point_info.valid) {
    LOG_ERROR("crossed_line_point_info is not valid");
    return false;
  }

  // get the obj arrive the interacted point time
  if (interactive_obj_pred_info.cur_s > crossed_line_frenet_point.x) {
    std::cout << "interactive_obj_pred_info.cur_s > crossed_line_frenet_point.x"
              << std::endl;
    return false;
  };

  const double interacted_time =
      (crossed_line_frenet_point.x - interactive_obj_pred_info.cur_s) /
      interactive_obj_pred_info.raw_vel;
  // default speed profile
  LonState ego_lon_state{ego_planning_init_s_[0], ego_planning_init_s_[1],
                         ego_planning_init_s_[2], 0., 0.};
  StateLimit default_state_limit{
      0.,        cruise_vel_, kPlanEps,         cruise_vel_ + kPlanEps,
      kMaxDecel, kMaxAcc,     kMaxNegativeJerk, kMaxPositiveJerk};

  SecondOrderTimeOptimalTrajectory default_traj(ego_lon_state,
                                                default_state_limit);
  LonState ego_crossed_line_state = default_traj.GetSecondOrderTrajectoryState(
      ego_lon_state, default_traj.second_order_param(), interacted_time);

  std::vector<double> vel_candidates{};
  if (std::fabs(ego_crossed_line_state.p - crossed_line_frenet_point.x) >
      kHalfEgoLength * 2 + kCarCollisionThreshld) {
    traj_time_optimal = default_traj;
    traj_time_optimal.valid_ = true;
    std::cout << "interactive decision normal ego vel: "
              << default_state_limit.v_end << std::endl;
    return true;
  } else {
    for (auto &v_candidate : vel_candidates) {
      StateLimit decel_state_limit = default_state_limit;
      decel_state_limit.v_end = std::fmax(v_candidate, 0.1);
      SecondOrderTimeOptimalTrajectory decel_time_optimal_traj(
          ego_lon_state, decel_state_limit);

      LonState decel_crossed_line_state =
          decel_time_optimal_traj.GetSecondOrderTrajectoryState(
              ego_lon_state, decel_time_optimal_traj.second_order_param(),
              interacted_time);
      if (crossed_line_frenet_point.x - decel_crossed_line_state.p >
          kHalfEgoLength * 2 + kCarCollisionThreshld) {
        traj_time_optimal = decel_time_optimal_traj;
        traj_time_optimal.valid_ = true;
        std::cout << "interactive decision decel ego vel: "
                  << default_state_limit.v_end << std::endl;
        return true;
      } else {
        std::cout << "decel cross line is too close " << std::endl;
        continue;
      }
    }
    return false;
    // check decel traj
  }
}

bool GapSelectorDecider::MakeInteractiveDecision(
    const ObstaclePredicatedInfo &front_obj_pred_info,
    const ObstaclePredicatedInfo &rear_obj_pred_info,
    const GapSelectorPathSpline &path_spline,
    SecondOrderTimeOptimalTrajectory &traj_time_optimal) {
  const Point2D &crossed_line_point =
      path_spline.crossed_line_point_info.crossed_line_point;
  Point2D crossed_line_frenet_point;
  if (!base_frenet_coord_->XYToSL(crossed_line_point,
                                  crossed_line_frenet_point)) {
    LOG_ERROR("Crossed line transform frenet failed!");
    return false;
  }
  if (!path_spline.crossed_line_point_info.valid) {
    LOG_ERROR("crossed_line_point_info is not valid");
    return false;
  }
  if (front_obj_pred_info.cur_s > crossed_line_frenet_point.x) {
    std::cout << "interactive_obj_pred_info.cur_s > crossed_line_frenet_point.x"
              << std::endl;
    return false;
  }
};

void GapSelectorDecider::DriveStyleTimeOptimalTrajEvaluation(
    const GapSelectorPathSpline &path_spline) {
  LonState ego_lon_state{ego_planning_init_s_[0], ego_planning_init_s_[1],
                         ego_planning_init_s_[2], 0., 0.};
  StateLimit state_limit{
      0.,        cruise_vel_, kPlanEps,         cruise_vel_ + kPlanEps,
      kMaxDecel, kMaxAcc,     kMaxNegativeJerk, kMaxPositiveJerk};

  bool interactive_ok{false};
  if (gap_drive_style_ == GapDriveStyle::OnlyRearFasterCar) {
    const ObstaclePredicatedInfo &rear_obstacle_info =
        agent_node_mgr_->agent_node_target_lane_map()
            .find(nearby_gap_.rear_agent_id)
            ->second;
    state_limit.v_end =
        std::fmin(cruise_vel_, rear_obstacle_info.raw_vel - kPlanEps);
    // -----apend decision
  } else if (gap_drive_style_ == GapDriveStyle::OnlyRearSlowerCar) {
    const ObstaclePredicatedInfo &rear_obstacle_info =
        agent_node_mgr_->agent_node_target_lane_map()
            .find(nearby_gap_.rear_agent_id)
            ->second;
    double suggested_vel{cruise_vel_};  // reserve
    state_limit.v_end = std::fmin(cruise_vel_, suggested_vel);
    interactive_ok = MakeInteractiveDecision(rear_obstacle_info, path_spline,
                                             traj_time_optimal_);
  } else if (gap_drive_style_ == GapDriveStyle::RearCarFaster) {
    const ObstaclePredicatedInfo &rear_obstacle_info =
        agent_node_mgr_->agent_node_target_lane_map()
            .find(nearby_gap_.rear_agent_id)
            ->second;
    state_limit.v_end =
        std::fmin(cruise_vel_, rear_obstacle_info.raw_vel - kPlanEps);
    // -----apend decision
  } else if (gap_drive_style_ == GapDriveStyle::FrontCarFaster) {
    const ObstaclePredicatedInfo &front_obstacle_info =
        agent_node_mgr_->agent_node_target_lane_map()
            .find(nearby_gap_.front_agent_id)
            ->second;
    const ObstaclePredicatedInfo &rear_obstacle_info =
        agent_node_mgr_->agent_node_target_lane_map()
            .find(nearby_gap_.rear_agent_id)
            ->second;
    state_limit.v_end =
        std::fmin(cruise_vel_, front_obstacle_info.raw_vel + kPlanEps);

    // -----apend decision
  } else if (gap_drive_style_ == GapDriveStyle::OnlyFrontFasterCar) {
    const double overtake_speed_buffer = 2.0;
    const ObstaclePredicatedInfo &front_obstacle_info =
        agent_node_mgr_->agent_node_target_lane_map()
            .find(nearby_gap_.front_agent_id)
            ->second;
    state_limit.v_end = std::fmin(
        cruise_vel_, front_obstacle_info.raw_vel + overtake_speed_buffer);

    // -----apend decision
    interactive_ok = MakeInteractiveDecision(front_obstacle_info, path_spline,
                                             traj_time_optimal_);
  } else if (gap_drive_style_ == GapDriveStyle::OnlyFrontSlowerCar) {
    const ObstaclePredicatedInfo &front_obstacle_info =
        agent_node_mgr_->agent_node_target_lane_map()
            .find(nearby_gap_.front_agent_id)
            ->second;
    state_limit.v_end = cruise_vel_;
  }

  // time optimal traj
  SecondOrderTimeOptimalTrajectory time_optimal_traj(ego_lon_state,
                                                     state_limit);
  time_optimal_traj.valid_ = true;
  if (!interactive_ok) {
    traj_time_optimal_ = time_optimal_traj;
  }

  return;
}

SecondOrderTimeOptimalTrajectory
GapSelectorDecider::DriveStyleTimeOptimalTrajEvaluation(const double ego_v,
                                                        const double target_v) {
  LonState ego_lon_state{ego_planning_init_s_[0], ego_v,
                         ego_planning_init_s_[2], 0., 0.};
  StateLimit state_limit{
      0.,        target_v, kPlanEps,         cruise_vel_,
      kMaxDecel, kMaxAcc,  kMaxNegativeJerk, kMaxPositiveJerk};
  return SecondOrderTimeOptimalTrajectory(ego_lon_state, state_limit);
}

void GapSelectorDecider::RetentivePathPlan() {
  // // generate different lateral path
  gap_selector_path_spline_.clear();
  lc_time_list_[0] = config_.default_lc_time;
  for (auto i = 0; i < 1; i++) {
    double quintic_truancation_s{200.};
    double expected_s = 100.;
    double expected_l = 2.5;
    double refine_time = lc_time_list_[i];

    CalcLatOffset(expected_s, expected_l, refine_time);

    pnc::spline::QuinticPolynominalPath lc_quintic_poly = ConstructQuinticPath(
        expected_s, expected_l, quintic_truancation_s, refine_time);

    // to stitch the quintic path and target lane path!
    GapSelectorPathSpline path_spline;
    path_spline.start_cart_point.x = planning_init_point_.lat_init_state.x();
    path_spline.start_cart_point.y = planning_init_point_.lat_init_state.y();
    path_spline.start_frenet_point.x = planning_init_point_.frenet_state.s;
    path_spline.start_frenet_point.y = planning_init_point_.frenet_state.r;
    path_spline.quintic_p0.x = lc_quintic_poly(0.).x();
    path_spline.quintic_p0.y = lc_quintic_poly(0.).y();
    path_spline.quintic_pe.x = lc_quintic_poly(refine_time).x();
    path_spline.quintic_pe.y = lc_quintic_poly(refine_time).y();

    StitchQuinticPath(quintic_truancation_s, refine_time, expected_l,
                      lc_quintic_poly, path_spline);
    QuerySplineCrossLinePoint(
        path_spline);  // TODO: need to consider the lane cross state
    gap_selector_path_spline_.emplace_back(path_spline);
  }
  return;
}

void GapSelectorDecider::ResponsivePathPlan() {
  gap_selector_path_spline_.clear();
  lc_time_list_[0] = config_.default_lc_time;

  double remaining_lc_duration =
      lc_time_list_[0] - gap_selector_state_machine_info_.lc_pass_time;
  shared_ptr<KDPath> coord = target_lane_coord_ptr_;
  if (remaining_lc_duration < 1.0) {
    const double lat_mvd_coeff = lc_time_list_[0] / kDefaultLatMovedDistance;
    remaining_lc_duration = std::fabs(ego_l_cur_lane_) * lat_mvd_coeff;
    if (gap_selector_state_machine_info_
            .lane_cross) {  // base coord is current lane coord
      // lane cross, go lc
      coord = target_lane_coord_ptr_;
    } else {
      // not lane cross, go lb
      coord = base_frenet_coord_;
    }
  }
  auto quintic_lc_path = ConstructQuinticPath(remaining_lc_duration, coord);
  GapSelectorPathSpline path_spline;
  path_spline.start_cart_point.x = planning_init_point_.lat_init_state.x();
  path_spline.start_cart_point.y = planning_init_point_.lat_init_state.y();
  path_spline.start_frenet_point.x = planning_init_point_.frenet_state.s;
  path_spline.start_frenet_point.y = planning_init_point_.frenet_state.r;
  path_spline.quintic_p0.x = quintic_lc_path(0.).x();
  path_spline.quintic_p0.y = quintic_lc_path(0.).y();
  path_spline.quintic_pe.x = quintic_lc_path(remaining_lc_duration).x();
  path_spline.quintic_pe.y = quintic_lc_path(remaining_lc_duration).y();
  DecoupleQuinticPathSpline(remaining_lc_duration, quintic_lc_path, coord,
                            path_spline);
  if (remaining_lc_duration > 1.0) {
    QuerySplineCrossLinePoint(
        path_spline);  // TODO: need to consider the lane cross state
    path_spline_.path_spline_status = GapSelectorPathSpline::LC_VALID;
    gap_selector_path_spline_.emplace_back(path_spline);
  } else {
    if (gap_selector_state_machine_info_.lane_cross) {
      path_spline_.path_spline_status = GapSelectorPathSpline::LC_LANE_CROSS;
      path_spline_.crossed_line_point_info.valid = false;
      gap_selector_path_spline_.emplace_back(path_spline);
    } else {
      path_spline.path_spline_status = GapSelectorPathSpline::LB_VALID;
      path_spline_.crossed_line_point_info.valid = false;
      gap_selector_path_spline_.emplace_back(path_spline);
    }
  }
  return;
};

void GapSelectorDecider::QuerySplineCrossLinePoint(
    GapSelectorPathSpline &path_spline) {
  if (gap_selector_state_machine_info_.lane_cross) {
    cross_line_point_info_.valid = false;
    cross_line_point_info_.crossed_line_point.x =
        planning_init_point_.lat_init_state.x();
    cross_line_point_info_.crossed_line_point.y =
        planning_init_point_.lat_init_state.y();
    return;
  }
  // generate spline local points
  double sample_total_length =
      std::fmin(std::fmax(cruise_vel_ * 5., kMinSplineSampleLength),
                path_spline.x_s_spline.get_x_max() - 2.);
  double sample_step = 4.;
  int sample_num = sample_total_length / sample_step;

  std::vector<double> local_x_vec_spline_path, local_y_vec_spline_path,
      local_x_vec_lane_boundary, local_y_vec_lane_boundary;
  local_x_vec_spline_path.reserve(sample_num);
  local_y_vec_spline_path.reserve(sample_num);
  local_x_vec_lane_boundary.reserve(sample_num);
  local_y_vec_lane_boundary.reserve(sample_num);

  double lane_direction = target_state_ == 1 ? 1 : -1;

  for (auto i = 0; i < sample_num; i++) {
    // spline local points
    double sample_s = i > 0 ? i * sample_step : 0.3;
    Point2D global_sample_point{path_spline.x_s_spline(sample_s),
                                path_spline.y_s_spline(sample_s)};
    Point2D local_sample_point;
    refline_transform_.Global2Local(global_sample_point, local_sample_point);
    local_x_vec_spline_path.emplace_back(local_sample_point.x);
    local_y_vec_spline_path.emplace_back(local_sample_point.y);

    // lane boundary local points
    double half_lane_length = 0.5 * lane_direction * origin_lane_width_;
    Point2D global_frenet_lane_point{
        planning_init_point_.frenet_state.s + sample_s, half_lane_length};
    Point2D global_cart_lane_point;
    base_frenet_coord_->SLToXY(global_frenet_lane_point,
                               global_cart_lane_point);

    Point2D local_lane_point;
    refline_transform_.Global2Local(global_cart_lane_point, local_lane_point);
    local_x_vec_lane_boundary.emplace_back(local_lane_point.x);
    local_y_vec_lane_boundary.emplace_back(local_lane_point.y);
  }

  bool is_monotic = (MonotonicCheck(local_x_vec_lane_boundary) == INCREASE) &&
                    (MonotonicCheck(local_x_vec_spline_path) == INCREASE);

  if (is_monotic) {
    // path spline & boundary spline, both x-y spline: local to protect the
    // mono
    pnc::mathlib::spline local_boundary_spline, local_path_spline;
    local_boundary_spline.set_points(local_x_vec_lane_boundary,
                                     local_y_vec_lane_boundary);
    local_path_spline.set_points(local_x_vec_spline_path,
                                 local_y_vec_spline_path);

    // Newton method
    double x0 =
        (local_path_spline.get_x_max() + local_path_spline.get_x_min()) * 0.5;
    double epsilon = 0.01;
    double x1 = x0;

    for (size_t i = 0; i < 10; ++i) {
      const double f_derv_1st =
          local_path_spline.deriv(1, x0) - local_boundary_spline.deriv(1, x0);
      x1 =
          x0 - (local_path_spline(x0) - local_boundary_spline(x0)) / f_derv_1st;
      if (std::fabs(x1 - x0) < epsilon) {
        break;
      }
      x0 = x1;
    };

    Point2D cross_local_point{x0, local_boundary_spline(x0)};
    Point2D cross_global_point;
    refline_transform_.Local2Gobal(cross_local_point, cross_global_point);
    path_spline.crossed_line_point_info.valid = true;
    path_spline.crossed_line_point_info.crossed_line_point = cross_global_point;
  } else {
    // linear calc
    const double spline_length =
        path_spline.x_s_spline.get_x_max() - path_spline.x_s_spline.get_x_min();
    const double expected_lat_movd_dist =
        lane_direction * 0.5 * (origin_lane_width_ + target_lane_width_) -
        path_spline.start_frenet_point.y;
    const double coef_dist =
        std::fabs((lane_direction * 0.5 * origin_lane_width_ -
                   path_spline.start_frenet_point.y));
    const double movd_s = spline_length * (coef_dist / expected_lat_movd_dist);

    path_spline.crossed_line_point_info.valid = true;
    path_spline.crossed_line_point_info.crossed_line_point.x =
        path_spline.x_s_spline(movd_s);
    path_spline.crossed_line_point_info.crossed_line_point.y =
        path_spline.y_s_spline(movd_s);
  }
  return;
}

MonotonicStatus GapSelectorDecider::MonotonicCheck(
    const std::vector<double> &x_vec) {
  if (x_vec.size() <= 1) {
    return NONMONOTIONIC;
  }

  bool increasing = true;
  bool decreasing = true;
  for (size_t i = 1; i < x_vec.size(); ++i) {
    if (x_vec[i] > x_vec[i - 1]) {
      decreasing = false;
    } else if (x_vec[i] < x_vec[i - 1]) {
      increasing = false;
    }
    // Early termination if both increasing and decreasing are already false
    if (!increasing && !decreasing) {
      return NONMONOTIONIC;
    }
  }

  if (increasing) {
    return INCREASE;
  } else if (decreasing) {
    return DECREASE;
  } else {
    return NONMONOTIONIC;
  }
}

void GapSelectorDecider::ConstructTimeOptimal(
    const SecondOrderTimeOptimalTrajectory &time_optimal_traj,
    const GapSelectorPathSpline &path_spline) {
  st_time_optimal_.resize(PointNum + 1);
  loninfo_time_optimal_.resize(PointNum + 1);
  // currently only check the first gap selector path spline st
  const double speed_adjust_duration = time_optimal_traj.ParamLength();
  const LonState &init_state = time_optimal_traj.init_state();
  const SecondOrderParam &second_order_param =
      time_optimal_traj.second_order_param();
  double remain_length = 0.;
  int truncation_idx = 0;
  double spline_s = 0.;
  // profile end point
  LonState profile_end_state = time_optimal_traj.GetSecondOrderTrajectoryState(
      init_state, second_order_param, time_optimal_traj.ParamLength());

  st_time_optimal_[0].set_t(0.);
  st_time_optimal_[0].set_x(planning_init_point_.lat_init_state.x());
  st_time_optimal_[0].set_y(planning_init_point_.lat_init_state.y());
  st_time_optimal_[0].set_s(ego_planning_init_s_[0]);
  loninfo_time_optimal_[0].v = ego_planning_init_s_[1];
  loninfo_time_optimal_[0].a = ego_planning_init_s_[2];
  loninfo_time_optimal_[0].j = 0.;
  loninfo_time_optimal_[0].l = ego_l_;
  loninfo_time_optimal_[0].heading = planning_init_point_.heading_angle;

  const auto &path_spline_start_cart_point = path_spline.start_cart_point;
  Point2D path_spline_start_frenet_point;
  double compensated_dis =
      std::hypot(path_spline_start_cart_point.x - ego_cart_point_[0],
                 path_spline_start_cart_point.y - ego_cart_point_[1]);
  if (base_frenet_coord_->XYToSL(path_spline_start_cart_point,
                                 path_spline_start_frenet_point)) {
    LOG_ERROR(
        "GS:Cart 2 frenet success, spline start cart point -> frenet point!");
    compensated_dis =
        ego_planning_init_s_[0] - path_spline_start_frenet_point.x;
  }
  // std::cout << "\n compensated_dis: " << compensated_dis << std::endl;
  for (auto i = 1; i < PointNum + 1; i++) {
    if (i * delta_t < speed_adjust_duration) {
      LonState profile_state = time_optimal_traj.GetSecondOrderTrajectoryState(
          init_state, second_order_param, i * delta_t);
      st_time_optimal_[i].set_s(profile_state.p);
      st_time_optimal_[i].set_t(i * delta_t);
      spline_s = profile_state.p - ego_planning_init_s_[0] + compensated_dis;
      loninfo_time_optimal_[i].v = profile_state.v;
      loninfo_time_optimal_[i].a = profile_state.a;
      loninfo_time_optimal_[i].j = profile_state.j;
      // frenet convert
      truncation_idx = i;
    } else {
      remain_length = i * delta_t - speed_adjust_duration > 0
                          ? i * delta_t - profile_end_state.t
                          : delta_t - profile_end_state.t;
      st_time_optimal_[i].set_s(profile_end_state.p +
                                remain_length * profile_end_state.v);
      st_time_optimal_[i].set_t(i * delta_t);
      spline_s = profile_end_state.p + remain_length * profile_end_state.v -
                 ego_planning_init_s_[0] + compensated_dis;
      loninfo_time_optimal_[i].v = profile_end_state.v;
      loninfo_time_optimal_[i].a = profile_end_state.a;
      loninfo_time_optimal_[i].j = profile_end_state.j;
    }
    st_time_optimal_[i].set_x(path_spline.x_s_spline(spline_s));
    st_time_optimal_[i].set_y(path_spline.y_s_spline(spline_s));

    Point2D frenet_point{-100, 0.};
    Point2D cart_point{st_time_optimal_[i].x(), st_time_optimal_[i].y()};
    if (!base_frenet_coord_->XYToSL(cart_point, frenet_point)) {
      std::cout << "GS:Cart 2 frenet failed, Generate loninfo time optimal!"
                << std::endl;
    }
    loninfo_time_optimal_[i].l = frenet_point.y;
    // std::cout << "\n loninfo_time_optimal l:" << loninfo_time_optimal_[i].l
    //           << " , i is: " << i << std::endl;
    loninfo_time_optimal_[i].heading =
        std::atan2(path_spline.y_s_spline.deriv(1, spline_s),
                   path_spline.x_s_spline.deriv(1, spline_s));
  }
  return;
}

int GapSelectorDecider::CollisionCheck() {
  int check_max_idx = 25;  // check 3s predication
  int collision_idx = -1;
  bool lat_collision{false};

  if (!front_gap_car_st_boundaries_.empty()) {
    for (auto i = 0; i < check_max_idx + 1; i++) {
      if ((st_time_optimal_[i].s() - kHalfEgoLength <
               front_gap_car_st_boundaries_[i].first.s() &&
           st_time_optimal_[i].s() - kHalfEgoLength >
               front_gap_car_st_boundaries_[i].second.s()) ||
          (st_time_optimal_[i].s() + kHalfEgoLength <
               front_gap_car_st_boundaries_[i].first.s() &&
           st_time_optimal_[i].s() + kHalfEgoLength >
               front_gap_car_st_boundaries_[i].second.s())) {
        lat_collision =
            target_state_ == 1
                ? loninfo_time_optimal_[i].l + kHalfEgoWidth > 1.85
                : loninfo_time_optimal_[i].l - kHalfEgoWidth < -1.85;
        // std::cout << "front gap car, lat collision check, time idx: " << i
        //           << ", l is: " << loninfo_time_optimal_[i].l << std::endl;
        collision_idx = lat_collision ? i : collision_idx;
        if (collision_idx > 0) {
          break;
        } else {
          continue;
        }
      } else {
        continue;
      }
    }
  }

  if (!rear_gap_car_st_boundaries_.empty() && collision_idx < 0) {
    for (auto i = 0; i < check_max_idx + 1; i++) {
      if ((st_time_optimal_[i].s() - kHalfEgoLength <
               rear_gap_car_st_boundaries_[i].first.s() &&
           st_time_optimal_[i].s() - kHalfEgoLength >
               rear_gap_car_st_boundaries_[i].second.s()) ||
          (st_time_optimal_[i].s() + kHalfEgoLength <
               rear_gap_car_st_boundaries_[i].first.s() &&
           st_time_optimal_[i].s() + kHalfEgoLength >
               rear_gap_car_st_boundaries_[i].second.s())) {
        lat_collision =
            target_state_ == 1
                ? loninfo_time_optimal_[i].l + kHalfEgoWidth > 1.85
                : loninfo_time_optimal_[i].l - kHalfEgoWidth < -1.85;
        // std::cout << "rear gap car, lat collision check, time idx: " << i
        //           << ", l is: " << loninfo_time_optimal_[i].l << std::endl;
        collision_idx = lat_collision ? i : collision_idx;
        if (collision_idx > 0) {
          break;
        } else {
          continue;
        }

      } else {
        continue;
      }
    }
  }

  if (!front_careful_car_st_boundary_.empty() && collision_idx < 0) {
    for (auto i = 0; i < check_max_idx + 1; i++) {
      if ((st_time_optimal_[i].s() - kHalfEgoLength <
               front_careful_car_st_boundary_[i].first.s() &&
           st_time_optimal_[i].s() - kHalfEgoLength >
               front_careful_car_st_boundary_[i].second.s()) ||
          (st_time_optimal_[i].s() + kHalfEgoLength <
               front_careful_car_st_boundary_[i].first.s() &&
           st_time_optimal_[i].s() + kHalfEgoLength >
               front_careful_car_st_boundary_[i].second.s())) {
        lat_collision =
            target_state_ == 1
                ? loninfo_time_optimal_[i].l + kHalfEgoWidth < 1.85
                : loninfo_time_optimal_[i].l - kHalfEgoWidth > -1.85;
        // std::cout << "careful car, lat collision check, time idx: " << i
        //           << ", l is: " << loninfo_time_optimal_[i].l << std::endl;
        collision_idx = lat_collision ? i : collision_idx;
        if (collision_idx > 0) {
          break;
        } else {
          continue;
        }
      } else {
        continue;
      }
    }
  }

  return collision_idx;
}

int GapSelectorDecider::SafetyCheck(const GapSelectorPathSpline &path_spline) {
  int collision_idx = -1;
  int64_t front_check_obj_id = -1;
  int64_t rear_check_obj_id = -1;
  front_gap_car_st_boundaries_.clear();
  rear_gap_car_st_boundaries_.clear();
  front_careful_car_st_boundary_.clear();
  if (nearby_gap_.front_agent_id < 0 && nearby_gap_.rear_agent_id < 0 &&
      front_careful_car_id_origin_lane_ < 0) {
    LOG_ERROR("ST Boundaries break check!");
    return collision_idx;
  }

  if (gap_drive_style_ == GapDriveStyle::OnlyRearSlowerCar ||
      gap_drive_style_ == GapDriveStyle::OnlyRearFasterCar) {
    rear_check_obj_id = nearby_gap_.rear_agent_id;
  } else if (gap_drive_style_ == GapDriveStyle::OnlyFrontSlowerCar ||
             gap_drive_style_ == GapDriveStyle::OnlyFrontFasterCar) {
    front_check_obj_id = nearby_gap_.front_agent_id;
  } else if (gap_drive_style_ == GapDriveStyle::RearCarFaster ||
             gap_drive_style_ == GapDriveStyle::FrontCarFaster ||
             gap_drive_style_ == GapDriveStyle::NoDecisionForBothCar) {
    front_check_obj_id = nearby_gap_.front_agent_id;
    rear_check_obj_id = nearby_gap_.rear_agent_id;
  }

  ConvertObsPredToSTBoundary(
      front_check_obj_id, agent_node_mgr_->agent_node_target_lane_map(),
      front_gap_car_st_boundaries_, front_car_dynamic_dis_);
  ConvertObsPredToSTBoundary(
      rear_check_obj_id, agent_node_mgr_->agent_node_target_lane_map(),
      rear_gap_car_st_boundaries_, rear_car_dynamic_dis_);

  if (front_careful_car_id_origin_lane_ > 0) {
    auto iter = agent_node_mgr_->agent_node_origin_lane_map().find(
        front_careful_car_id_origin_lane_);
    if (iter != agent_node_mgr_->agent_node_origin_lane_map().end()) {
      ConvertObsPredToSTBoundary(front_careful_car_id_origin_lane_,
                                 agent_node_mgr_->agent_node_origin_lane_map(),
                                 front_careful_car_st_boundary_,
                                 ego_lane_car_dynamic_dis_);
    }
  }

  return CollisionCheck();
}

bool GapSelectorDecider::PreSafetyCheck() {
  // nearby_gap_.front_agent_id > -1 || nearby_gap_.rear_agent_id > -1
  std::pair<double, double> ego_agent_s_area{
      ego_planning_init_s_[0] + kHalfEgoLength,
      ego_planning_init_s_[0] - kHalfEgoLength};

  if (nearby_gap_.front_agent_id > -1) {
    std::pair<double, double> front_agent_s_area;

    const ObstaclePredicatedInfo &car_pred_info =
        agent_node_mgr_->agent_node_target_lane_map()
            .find(nearby_gap_.front_agent_id)
            ->second;

    front_agent_s_area.first =
        car_pred_info.cur_s + 0.5 * car_pred_info.length + config_.near_car_ttc;
    front_agent_s_area.second =
        car_pred_info.cur_s - 0.5 * car_pred_info.length - config_.near_car_ttc;
    if ((ego_agent_s_area.first > front_agent_s_area.second &&
         ego_agent_s_area.first < front_agent_s_area.first) ||
        (ego_agent_s_area.second > front_agent_s_area.second &&
         ego_agent_s_area.second < front_agent_s_area.first)) {
      return false;
    }
  }
  if (nearby_gap_.rear_agent_id > -1) {
    std::pair<double, double> rear_agent_s_area;

    const ObstaclePredicatedInfo &car_pred_info =
        agent_node_mgr_->agent_node_target_lane_map()
            .find(nearby_gap_.rear_agent_id)
            ->second;

    rear_agent_s_area.first =
        car_pred_info.cur_s + 0.5 * car_pred_info.length + config_.near_car_ttc;
    rear_agent_s_area.second =
        car_pred_info.cur_s - 0.5 * car_pred_info.length - config_.near_car_ttc;
    if ((ego_agent_s_area.first > rear_agent_s_area.second &&
         ego_agent_s_area.first < rear_agent_s_area.first) ||
        (ego_agent_s_area.second > rear_agent_s_area.second &&
         ego_agent_s_area.second < rear_agent_s_area.first)) {
      return false;
    }
  }

  return true;
}

bool GapSelectorDecider::BodyCheck() {
  const auto &heading = planning_init_point_.heading_angle;
  const auto &ego_x = planning_init_point_.lat_init_state.x();
  const auto &ego_y = planning_init_point_.lat_init_state.y();

  Point2D sl_center;
  origin_lane_coord_ptr_->XYToSL({ego_x, ego_y}, sl_center);

  Transform2D ego_transform(heading, sl_center.x, sl_center.y);
  const double half_away_origin_lane_width =
      0.5 * QueryLaneWidth(ego_planning_init_s_[0], origin_lane_s_width_);
  if (target_state_ == 0) {
    return false;
  }

  const double cross_l = (target_state_ == 1) ? half_away_origin_lane_width
                                              : -half_away_origin_lane_width;

  Point2D lf_point{vehicle_param_.rear_axis_to_front_edge,
                   vehicle_param_.left_edge_to_center};
  Point2D lb_point{vehicle_param_.back_edge_to_rear_axis,
                   vehicle_param_.left_edge_to_center};
  Point2D rf_point{vehicle_param_.rear_axis_to_front_edge,
                   vehicle_param_.right_edge_to_center};
  Point2D rb_point{vehicle_param_.back_edge_to_rear_axis,
                   vehicle_param_.right_edge_to_center};
  Point2D lf_point_car, lb_point_car, rf_point_car, rb_point_car;
  ego_transform.Local2Gobal(lf_point, lf_point_car);
  ego_transform.Local2Gobal(lb_point, lb_point_car);
  ego_transform.Local2Gobal(rf_point, rf_point_car);
  ego_transform.Local2Gobal(rb_point, rb_point_car);

  if ((lf_point_car.y > cross_l && target_state_ == 1) ||
      (lf_point_car.y < cross_l && target_state_ == 2) ||
      (lb_point_car.y > cross_l && target_state_ == 1) ||
      (lb_point_car.y < cross_l && target_state_ == 2) ||
      (rf_point_car.y > cross_l && target_state_ == 1) ||
      (rf_point_car.y < cross_l && target_state_ == 2) ||
      (rb_point_car.y > cross_l && target_state_ == 1) ||
      (rb_point_car.y < cross_l && target_state_ == 2)) {
    return true;
  }
  return false;
};

bool GapSelectorDecider::PathSplineLengthCheck(
    GapSelectorPathSpline &path_spline) {
  const Point2D &spline_start_cart_point = path_spline.start_cart_point;
  /*
    @liucai:
    the frenet start point is about 50m away from ego init s
  */
  return std::hypot(ego_cart_point_[0] - spline_start_cart_point.x,
                    ego_cart_point_[1] - spline_start_cart_point.y) < 50.;
}
void GapSelectorDecider::ConvertObsPredToSTBoundary(
    const int64_t &id,
    const std::unordered_map<int64_t, ObstaclePredicatedInfo> &agent_node_map,
    std::vector<std::pair<STPoint, STPoint>> &st_boundary,
    double &dynamic_dis) {
  if (id < 0) {
    return;
  }
  st_boundary.resize(PointNum + 1);

  const ObstaclePredicatedInfo &car_pred_info = agent_node_map.find(id)->second;
  const double obj_length = car_pred_info.length;

  // calc ttc length threshold
  const double obj_vel = car_pred_info.raw_vel;
  // const double collision_threshold =
  //     std::fmax(config_.collision_check_length_threshold, );
  bool ego_front = (nearby_gap_.front_agent_id == id ||
                    front_careful_car_id_origin_lane_ == id)
                       ? true
                       : false;
  const double delta_v = std::abs(obj_vel - ego_planning_init_s_[1]);
  double dynamic_safe_distance = 0.;
  const double kMaxEndurableAcc = 2.0;
  if ((ego_front && obj_vel < ego_planning_init_s_[1]) ||
      (!ego_front && obj_vel > ego_planning_init_s_[1])) {
    dynamic_safe_distance = 0.5 * delta_v * delta_v / kMaxEndurableAcc;
  }

  for (auto i = 0; i < PointNum + 1; i++) {
    st_boundary[i].first.set_s(car_pred_info.obstacle_pred_info.at(i).s +
                               obj_length * 0.5 +
                               config_.collision_check_length_threshold +
                               (ego_front ? 0. : dynamic_safe_distance));
    st_boundary[i].first.set_t(i * delta_t);

    st_boundary[i].second.set_s(car_pred_info.obstacle_pred_info.at(i).s -
                                obj_length * 0.5 -
                                config_.collision_check_length_threshold -
                                (ego_front ? dynamic_safe_distance : 0.));
    st_boundary[i].second.set_t(i * delta_t);
  }
  dynamic_dis = dynamic_safe_distance;
  return;
}

double GapSelectorDecider::QueryLaneWidth(
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
  return std::fmax(lane_width, 2.8);
}

void GapSelectorDecider::CalcLatOffset(double &expected_s, double &expected_l,
                                       double &refine_lc_time) {
  expected_s = std::fmin(
      planning_init_point_.frenet_state.s +
          planning_init_point_.v * refine_lc_time,
      base_frenet_coord_->Length() - planning_init_point_.frenet_state.s - 0.5);
  const double &cur_l = planning_init_point_.frenet_state.r;

  if (gap_selector_state_machine_info_.lane_cross) {
    expected_l = 0.;
    return;
  }

  if (use_query_lane_width_) {
    const double away_origin_lane_width =
        QueryLaneWidth(expected_s, origin_lane_s_width_);
    const double away_target_lane_width =
        QueryLaneWidth(expected_s, target_lane_s_width_);
    expected_l = target_state_ == 1
                     ? 0.5 * (away_origin_lane_width + away_target_lane_width)
                     : -0.5 * (away_origin_lane_width + away_target_lane_width);

  } else {
    const double &cur_s = planning_init_point_.frenet_state.s;

    const double near_current_lane_width = origin_lane_width_;
    const double near_target_lane_width = target_lane_width_;

    double near_expected_l =
        0.5 * (near_current_lane_width + near_target_lane_width);
    near_expected_l = target_state_ == 1 ? near_expected_l : -near_expected_l;
    double coeff = (near_expected_l - cur_l) / near_expected_l;

    refine_lc_time =
        std::fmax(std::min(refine_lc_time * coeff, kMaxExpectedLCTime),
                  kMinExpectedLCTime);  // refine time

    const double away_current_lane_width = origin_lane_width_;
    // origin_vir_lane_ptr_->width_by_s(expected_s);
    const double away_target_lane_width = target_lane_width_;
    // target_vir_lane_ptr_->width_by_s(expected_s);

    expected_l =
        target_state_ == 1
            ? 0.5 * (away_current_lane_width + away_target_lane_width)
            : -0.5 * (away_current_lane_width + away_target_lane_width);
  }
  return;
}

pnc::spline::QuinticPolynominalPath GapSelectorDecider::ConstructQuinticPath(
    double &expected_s, double &expected_l, double &truancation_end_s,
    const double expected_lc_time) {
  double x, y, v_x, v_y, v_x_end, v_y_end, normal_acc_x, normal_acc_y,
      normal_acc_x_end, normal_acc_y_end;

  double normal_acc, heading_end, curv_end, normal_acc_end;

  x = planning_init_point_.lat_init_state.x();
  y = planning_init_point_.lat_init_state.y();
  v_x = ego_planning_init_s_[1] *
        std::cos(planning_init_point_.heading_angle);  // theta equal to heading
  v_y = ego_planning_init_s_[1] * std::sin(planning_init_point_.heading_angle);

  normal_acc =
      ego_planning_init_s_[1] * ego_planning_init_s_[1] *
      planning_init_point_
          .curvature;  // TODO: curv and heading is important for the path
  normal_acc_x = normal_acc * std::sin(planning_init_point_.heading_angle);
  normal_acc_y = normal_acc * std::cos(planning_init_point_.heading_angle);

  Point2D end_frenet_point(expected_s, expected_l);
  Point2D end_cart_point;
  if (!base_frenet_coord_->SLToXY(end_frenet_point, end_cart_point)) {
    LOG_ERROR("Quintic Path Frenet->Car Error");
  }
  // realign reference velocity
  double s_length = std::hypot(end_cart_point.y - y, end_cart_point.x - x);
  const double v_cruise_scale =
      std::min(s_length / (cruise_vel_ * expected_lc_time), 1.0);
  const double v_adjust = cruise_vel_ * v_cruise_scale;
  // Assumption: target lane has same status of the origin lane
  heading_end = base_frenet_coord_->GetPathCurveHeading(
      expected_s);  // TODO: 1.add frenet length s protection; consider target
                    // lane heading

  base_frenet_coord_->GetKappaByS(expected_s, &curv_end);

  v_x_end = v_adjust * std::cos(heading_end);
  v_y_end = v_adjust * std::sin(heading_end);
  normal_acc_end = v_adjust * v_adjust * curv_end;
  normal_acc_x_end = normal_acc_end * std::sin(heading_end);
  normal_acc_y_end = normal_acc_end * std::cos(heading_end);

  Eigen::Vector2d x0(x, y);
  Eigen::Vector2d dx0(v_x, v_y);
  Eigen::Vector2d ddx0(normal_acc_x, normal_acc_y);

  Eigen::Vector2d xT(end_cart_point.x, end_cart_point.y);
  Eigen::Vector2d dxT(v_x_end, v_y_end);
  Eigen::Vector2d ddxT(normal_acc_x_end, normal_acc_y_end);

  pnc::spline::QuinticPolynominalPath candidate_path;

  candidate_path.SetPoints(x0, xT, dx0, dxT, ddx0, ddxT, expected_lc_time);
  truancation_end_s = expected_s;
  return candidate_path;
}

pnc::spline::QuinticPolynominalPath GapSelectorDecider::ConstructQuinticPath(
    const double remaining_lc_duration, const shared_ptr<KDPath> coord) {
  double ego_v = cruise_vel_;
  const auto &lat_state = planning_init_point_.lat_init_state;

  if (planning_init_point_.frenet_state.s + ego_v * remaining_lc_duration >=
      coord->Length() - 0.5) {
    ego_v = (coord->Length() - planning_init_point_.frenet_state.s - 0.5) /
            std::fmax(remaining_lc_duration, PointNum * delta_t);
  }
  const double lane_change_end_s =
      planning_init_point_.frenet_state.s + ego_v * remaining_lc_duration;

  const double normal_acc = ego_v * ego_v * lat_state.curv();

  Point2D frenet_end_point{lane_change_end_s, 0.};
  Point2D cart_end_point;
  if (!coord->SLToXY(frenet_end_point, cart_end_point)) {
    LOG_ERROR("ERROR! TargetLaneCoordPtr Frenet Point -> Cart Point Failed!!!");
  }
  const auto lane_change_end_heading_angle =  // use target lane info!
      coord->GetPathCurveHeading(lane_change_end_s);
  const auto lane_change_end_curvature =
      coord->GetPathPointByS(lane_change_end_s).kappa();

  const auto normal_acc_end = ego_v * ego_v * lane_change_end_curvature;

  // construct quintic path for lane change duration
  QuinticPolynominalPath lane_change_quintic_path;
  QuinticPolyInput quintic_poly_input{
      Eigen::Vector2d(planning_init_point_.lat_init_state.x(),
                      planning_init_point_.lat_init_state.y()),
      Eigen::Vector2d(cart_end_point.x, cart_end_point.y),
      Eigen::Vector2d(ego_v * std::cos(lat_state.theta()),
                      ego_v * std::sin(lat_state.theta())),
      Eigen::Vector2d(ego_v * std::cos(lane_change_end_heading_angle),
                      ego_v * std::sin(lane_change_end_heading_angle)),
      Eigen::Vector2d(normal_acc * std::cos(lat_state.theta()),
                      normal_acc * std::sin(lat_state.theta())),
      Eigen::Vector2d(normal_acc_end * std::sin(lane_change_end_heading_angle),
                      normal_acc_end * std::cos(lane_change_end_heading_angle)),
      remaining_lc_duration};
  lane_change_quintic_path.SetPoints(quintic_poly_input);
  return lane_change_quintic_path;
}

void GapSelectorDecider::StitchQuinticPath(
    const double &truancation_end_s, const double expected_lc_time,
    const double expected_l, pnc::spline::QuinticPolynominalPath &quintic_path,
    GapSelectorPathSpline &path_spline) {
  double path_length =
      base_frenet_coord_->Length() - 0.5 - planning_init_point_.frenet_state.s;
  const double sample_s = 15.0;

  int quintic_point_num = expected_lc_time / delta_t;

  std::vector<double> x_path_points;
  std::vector<double> y_path_points;
  std::vector<double> s_vec;
  x_path_points.reserve(quintic_point_num + 1  //+ stitch_point_num
  );
  y_path_points.reserve(quintic_point_num + 1  //+ stitch_point_num
  );
  s_vec.reserve(quintic_point_num + 1  //+ quintic_point_num
  );

  for (auto i = 0; i < quintic_point_num; i++) {
    Eigen::Vector2d path_point = quintic_path(i * delta_t);
    x_path_points.emplace_back(path_point.x());
    y_path_points.emplace_back(path_point.y());
  }

  Point2D path_end_point{std::fmin(truancation_end_s + 70., path_length),
                         expected_l};
  Point2D path_end_pos{0., 0.};
  if (!base_frenet_coord_->SLToXY(path_end_point, path_end_pos)) {
    LOG_ERROR("GS ERROR! Frenet Point -> Cart Point Failed!!!");
  }
  x_path_points.emplace_back(path_end_pos.x);
  y_path_points.emplace_back(path_end_pos.y);

  path_spline.stitched_p.x = path_end_pos.x;
  path_spline.stitched_p.y = path_end_pos.y;
  s_vec.emplace_back(0.);
  for (auto i = 0; i < x_path_points.size() - 1; i++) {
    double delta_s = std::hypot(x_path_points[i + 1] - x_path_points[i],
                                y_path_points[i + 1] - y_path_points[i]);
    s_vec.emplace_back(s_vec[i] + delta_s + 0.1);
  }

  // // set spline
  assert(s_vec.size() > 3);
  path_spline.x_s_spline.set_points(s_vec, x_path_points);
  path_spline.y_s_spline.set_points(s_vec, y_path_points);
  return;
}

void GapSelectorDecider::DecoupleQuinticPathSpline(
    const double remaining_lc_duration,
    const pnc::spline::QuinticPolynominalPath &quintic_path,
    const std::shared_ptr<KDPath> coord, GapSelectorPathSpline &path_spline) {
  const double path_length =
      coord->Length() - 0.5 - planning_init_point_.frenet_state.s;
  const double sample_s = 7.0;

  std::vector<double> x_path_points, y_path_points, s_vec;

  int quintic_point_num = remaining_lc_duration / delta_t;
  quintic_point_num = quintic_point_num > 1 ? quintic_point_num : 1;
  for (auto i = 0; i < quintic_point_num; i++) {
    Eigen::Vector2d path_point = quintic_path(i * delta_t);
    x_path_points.emplace_back(path_point.x());
    y_path_points.emplace_back(path_point.y());
  };

  Point2D quintic_end_point{x_path_points.back(), y_path_points.back()};
  Point2D quintic_end_frenet_in_target_lane;
  if (!coord->XYToSL(quintic_end_point, quintic_end_frenet_in_target_lane)) {
    LOG_ERROR("Target lane calc end path end frenet point failed!");
  }

  int compensated_point_num =
      (coord->Length() - quintic_end_frenet_in_target_lane.x) / sample_s;

  for (auto i = 0; i < compensated_point_num; i++) {
    Point2D compensate_point{
        quintic_end_frenet_in_target_lane.x + (i + 1) * sample_s, 0.};
    Point2D compensate_cart_point;
    if (!coord->SLToXY(compensate_point, compensate_cart_point)) {
      LOG_ERROR("Target lane calc compensate_point path point failed!");
    }
    x_path_points.emplace_back(compensate_cart_point.x);
    y_path_points.emplace_back(compensate_cart_point.y);
  }

  double s = 0.;
  s_vec.emplace_back(s);
  for (auto i = 1; i < compensated_point_num + quintic_point_num; i++) {
    s += std::hypot(x_path_points[i] - x_path_points[i - 1],
                    y_path_points[i] - y_path_points[i - 1]);
    s_vec.emplace_back(s);
  }

  // set spline
  path_spline.x_s_spline.set_points(s_vec, x_path_points);
  path_spline.y_s_spline.set_points(s_vec, y_path_points);
  return;
};

void GapSelectorDecider::GenerateEgoTrajectory(TrajectoryPoints &traj_points) {
  for (auto i = 0; i < PointNum + 1; i++) {
    auto &traj_point = traj_points[i];
    traj_point.s = st_time_optimal_[i].s();
    traj_point.t = st_time_optimal_[i].t();
    traj_point.x = st_time_optimal_[i].x();
    traj_point.y = st_time_optimal_[i].y();
    traj_point.a = loninfo_time_optimal_[i].a;
    traj_point.v = loninfo_time_optimal_[i].v;
    traj_point.l = loninfo_time_optimal_[i].l;
    traj_point.heading_angle = loninfo_time_optimal_[i].heading;
  }
  return;
}

void GapSelectorDecider::GenerateLBTrajectory(TrajectoryPoints &traj_points) {
  double v_target = cruise_vel_;
  Point2D tmp_cart_point{planning_init_point_.lat_init_state.x(),
                         planning_init_point_.lat_init_state.y()};
  Point2D tmp_frenet_point{planning_init_point_.frenet_state.s,
                           planning_init_point_.frenet_state.r};

  const double l0 = planning_init_point_.frenet_state.r;
  const double le = 0.;
  const double t0 = 0.;
  const double te = 5.;

  for (auto i = 0; i < PointNum + 1; i++) {
    auto &traj_point = traj_points[i];
    traj_point.v = v_target;
    traj_point.s = ego_planning_init_s_[0] + i * v_target * delta_t;
    traj_point.l = planning_math::lerp(l0, t0, le, te, i * delta_t);

    tmp_frenet_point.x = traj_point.s;
    tmp_frenet_point.y = traj_point.l;
    base_frenet_coord_->SLToXY(tmp_frenet_point, tmp_cart_point);
    traj_point.x = tmp_cart_point.x;
    traj_point.y = tmp_cart_point.y;
    traj_point.t = i * delta_t;
    traj_point.a = 0.;
    traj_point.heading_angle =
        i == 0 ? planning_init_point_.heading_angle
               : std::atan2(traj_point.y - traj_points[i - 1].y,
                            traj_point.x - traj_points[i - 1].x);
  }
  return;
}

void GapSelectorDecider::GenerateLHTrajectory(
    const SecondOrderTimeOptimalTrajectory &lane_hold_time_speed_profile,
    TrajectoryPoints &traj_points) {
  // the car is in dynamic lc, need to consider the lat car state, use quintic
  // poly
  double expected_s = cruise_vel_ * 5;

  for (auto i = 0; i < traj_points.size(); i++) {
    auto &traj_point = traj_points[i];
    Point2D tmp_frenet_point{
        ego_planning_init_s_[0] + i * delta_t * ego_planning_init_s_[1],
        ego_l_cur_lane_};
    Point2D tmp_cart_point;
    if (base_frenet_coord_->SLToXY(tmp_frenet_point, tmp_cart_point)) {
      LOG_ERROR("Restore Traj Result failed!");
      traj_point.s = tmp_frenet_point.x;
      traj_point.l = tmp_frenet_point.y;
      traj_point.x = tmp_cart_point.x;
      traj_point.y = tmp_cart_point.y;
      traj_point.v = cruise_vel_;
      traj_point.t = i * delta_t;
      traj_point.a = 0.;
      traj_point.heading_angle =
          i == 0 ? planning_init_point_.heading_angle
                 : std::atan2(traj_point.y - traj_points[i - 1].y,
                              traj_point.x - traj_points[i - 1].x);
    }
  }

  return;

  // /*TODO: @cailiu
  //   the expected l will be in target lane, because traj points is directely
  //   generated by target lane
  //   currently, hack the expected l
  // */
  // double expected_l = ego_l_cur_lane_;  // base frenet coord l
  // double expected_lh_time = 5.;
  // double default_end_s = 200.;
  // const std::shared_ptr<KDPath> coord =
  //     gap_selector_state_machine_info_.lane_cross ? target_lane_coord_ptr_
  //                                                 : origin_lane_coord_ptr_;
  // pnc::spline::QuinticPolynominalPath lh_quintic_path =
  //     ConstructQuinticPath(expected_lh_time, coord);
  // DecoupleQuinticPathSpline(expected_lh_time, lh_quintic_path, coord,
  //                           path_spline_);

  // const double frenet_end_s =
  //     base_frenet_coord_->Length() - planning_init_point_.frenet_state.s;

  // const LonState &init_state = lane_hold_time_speed_profile.init_state();

  // const SecondOrderParam &second_order_param =
  //     lane_hold_time_speed_profile.second_order_param();
  // const double speed_adjust_duration =
  //     lane_hold_time_speed_profile.ParamLength();
  // const LonState profile_end_state =
  //     lane_hold_time_speed_profile.GetSecondOrderTrajectoryState(
  //         init_state, second_order_param, speed_adjust_duration);
  // double remain_length = 0.;
  // double spline_s = 0.;

  // const auto &path_spline_start_cart_point =
  //     Point2D{planning_init_point_.lat_init_state.x(),
  //             planning_init_point_.lat_init_state.y()};
  // Point2D path_spline_start_frenet_point{0., 0.};

  // // std::cout << "\nplanning_init_point_.frenet_state.s: "
  // //           << planning_init_point_.frenet_state.s << std::endl;
  // // std::cout << "\n path_spline_start_cart_point x: "
  // //           << path_spline_start_cart_point.x << std::endl;
  // // std::cout << "\n path_spline_start_cart_point y: "
  // //           << path_spline_start_cart_point.y << std::endl;
  // // std::cout << "\n planning_init_point.x: " << planning_init_point_.x
  // //           << std::endl;
  // // std::cout << "\n planning_init_point.y: " << planning_init_point_.y
  // //           << std::endl;
  // if (!base_frenet_coord_->XYToSL(path_spline_start_cart_point,
  //                                 path_spline_start_frenet_point)) {
  //   LOG_ERROR(
  //       "GS:Cart 2 frenet failed, spline start cart point -> frenet point!");
  // }
  // // std::cout << "\nstart_frenet_point s: " <<
  // // path_spline_start_frenet_point.x
  // //           << std::endl;
  // for (auto i = 0; i < PointNum + 1; i++) {
  //   auto &traj_point = traj_points[i];
  //   if (i * delta_t < speed_adjust_duration) {
  //     LonState profile_state =
  //         lane_hold_time_speed_profile.GetSecondOrderTrajectoryState(
  //             init_state, second_order_param, i * delta_t);

  //     traj_point.v = i > 0 ? profile_state.v : ego_planning_init_s_[1];
  //     traj_point.a = i > 0 ? profile_state.a : ego_planning_init_s_[2];
  //     spline_s = profile_state.p - path_spline_start_frenet_point.x;
  //     // std::cout << "\n profile_state.p: " << profile_state.p << std::endl;
  //     // std::cout << "\n path_spline_start_frenet_point.x: "
  //     //           << path_spline_start_frenet_point.x << std::endl;
  //   } else {
  //     remain_length = i * delta_t - speed_adjust_duration > 0
  //                         ? i * delta_t - profile_end_state.t
  //                         : delta_t - profile_end_state.t;

  //     traj_point.v = profile_end_state.v;
  //     traj_point.a = profile_end_state.a;
  //     spline_s = profile_end_state.p + remain_length * profile_end_state.v -
  //                path_spline_start_frenet_point.x;
  //   }
  //   // std::cout << "\n lh spline s[" << i << "]: " << spline_s << std::endl;
  //   traj_point.x = i > 0 ? path_spline_.x_s_spline(spline_s)
  //                        : planning_init_point_.lat_init_state.x();
  //   traj_point.y = i > 0 ? path_spline_.y_s_spline(spline_s)
  //                        : planning_init_point_.lat_init_state.y();

  //   Point2D cart_point{traj_point.x, traj_point.y};
  //   Point2D frenet_point{-100, 0.};
  //   if (!base_frenet_coord_->XYToSL(cart_point, frenet_point)) {
  //     LOG_ERROR("GS:Cart 2 frenet failed, LH,Generate loninfo time
  //     optimal!");
  //   }
  //   traj_point.s = i > 0 ? frenet_point.x : ego_planning_init_s_[0];
  //   traj_point.l = i > 0 ? frenet_point.y : ego_l_;  // keep the l
  //   traj_point.heading_angle = base_frenet_coord_->GetPathCurveHeading(
  //       std::fmin(frenet_end_s, traj_point.s));
  //   // NOTE: the steer angle may not be 0, todo:
  //   // consider lateral planning
  //   traj_point.t = i * delta_t;
  // }
  // return;
}

void GapSelectorDecider::GenerateLinearRefTrajectory(
    const bool is_left, TrajectoryPoints &traj_points) {
  // linear generate traj
  const double lane_direction = is_left ? 1 : -1;
  const double lat_movd_coef = lane_direction * 3.8 / config_.default_lc_time;

  for (auto i = 0; i < traj_points.size(); i++) {
    auto &traj_point = traj_points[i];
    Point2D tmp_frenet_point{
        ego_planning_init_s_[0] + i * delta_t * ego_planning_init_s_[1],
        ego_l_cur_lane_ + lat_movd_coef * i * delta_t};
    Point2D tmp_cart_point;
    if (base_frenet_coord_->SLToXY(tmp_frenet_point, tmp_cart_point)) {
      LOG_ERROR("Restore Traj Result failed!");
      traj_point.s = tmp_frenet_point.x;
      traj_point.l = tmp_frenet_point.y;
      traj_point.x = tmp_cart_point.x;
      traj_point.y = tmp_cart_point.y;
      traj_point.v = cruise_vel_;
      traj_point.t = i * delta_t;
      traj_point.a = 0.;
      traj_point.heading_angle =
          i == 0 ? planning_init_point_.heading_angle
                 : std::atan2(traj_point.y - traj_points[i - 1].y,
                              traj_point.x - traj_points[i - 1].x);
    }
  }

  return;
}
void GapSelectorDecider::RestoreTrajResult(TrajectoryPoints &traj_points) {
  if (gap_selector_state_machine_info_.lane_cross) {
    for (auto i = 0; i < traj_points.size(); i++) {
      Point2D tmp_cart_point{traj_points[i].x, traj_points[i].y};
      Point2D tmp_frenet_point;
      if (origin_lane_coord_ptr_->XYToSL(tmp_cart_point, tmp_frenet_point)) {
        LOG_ERROR("Restore Traj Result failed!");
        traj_points[i].s = tmp_frenet_point.x;
        traj_points[i].l = tmp_frenet_point.y;
      }
    }
  }
  return;
}

bool GapSelectorDecider::CheckLCFinish() {
  if (gap_selector_state_machine_info_.lane_cross) {
    LOG_DEBUG("lc finished! lane crossed!");
    return std::fabs(ego_l_cur_lane_) < 0.5 ? true : false;
  } else {
    LOG_DEBUG("lc finished! lane crossed!");
    return false;
  }
}

}  // namespace planning