#include "hpp_general_lateral_decider.h"

#include <assert.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "../../common/planning_gflags.h"
#include "agent_node_manager.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "history_obstacle_manager.h"
#include "planning_context.h"
#include "prediction_object.h"
#include "task_basic_types.h"
#include "utils/general_lateral_decider_utils.h"
#include "utils/kd_path.h"
#include "vehicle_config_context.h"
namespace planning {

using namespace planning_math;
using namespace pnc::spline;

static constexpr double kCareAreaSBuffer = 5.0;

HppGeneralLateralDecider::HppGeneralLateralDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<HppGeneralLateralDeciderConfig>();
  name_ = "HppGeneralLateralDecider";
}

bool HppGeneralLateralDecider::InitInfo() {
  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  ego_frenet_state_ = reference_path_ptr->get_frenet_ego_state();

  ego_cart_state_manager_ =
      session_->environmental_model().get_ego_state_manager();

  history_obstacle_manager_ =
      session_->environmental_model().get_history_obstacle_manager();

  cruise_vel_ = session_->planning_context().v_ref_cruise();

  is_lane_change_scene_ = false;

  auto &general_lateral_decider_output =
      session_->mutable_planning_context()
          ->mutable_hpp_general_lateral_decider_output();
  general_lateral_decider_output.enu_ref_path.clear();
  general_lateral_decider_output.last_enu_ref_path.clear();
  general_lateral_decider_output.path_bounds.clear();
  general_lateral_decider_output.safe_bounds.clear();
  general_lateral_decider_output.enu_ref_theta.clear();
  general_lateral_decider_output.last_enu_ref_theta.clear();

  lat_lane_change_info_ = LatDeciderLaneChangeInfo::NONE;

  if (reference_path_ptr == nullptr || ego_cart_state_manager_ == nullptr) {
    // add logs
    return false;
  }
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

  // @liucai: this agent node mgr update function can be moved to gap selector
  // task
  // ----------------------------------
  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  const auto &lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto state = coarse_planning_info.target_state;
  const auto lc_request_direction = lane_change_decider_output.lc_request;
  bool is_LC_LCHANGE =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == LEFT_CHANGE);
  bool is_LC_RCHANGE =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == RIGHT_CHANGE);
  int target_state = is_LC_LCHANGE ? 1 : (is_LC_RCHANGE ? 2 : 0);
  std::shared_ptr<AgentNodeManager> agent_node_manager =
      session_->mutable_environmental_model()->mutable_agent_node_manager();

  auto origin_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(
              lane_change_decider_output.origin_lane_virtual_id, false);
  auto origin_lane_coord_ptr = origin_refline->get_frenet_coord();
  auto target_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(
              lane_change_decider_output.target_lane_virtual_id, false);
  auto target_lane_coord_ptr = target_refline->get_frenet_coord();
  std::vector<int> ids_obstacle_in_target_lane;
  if (target_refline != nullptr) {
    ids_obstacle_in_target_lane =
        target_refline->mutable_obstacles_in_lane_map();
  } else {
    ids_obstacle_in_target_lane.emplace_back(-1);  // no obstacle in targe lane
  }

  std::vector<int> ids_obstacle_in_origin_lane;
  if (origin_refline != nullptr) {
    ids_obstacle_in_origin_lane =
        origin_refline->mutable_obstacles_in_lane_map();
  } else {
    ids_obstacle_in_origin_lane.emplace_back(-1);  // no obstacle in targe lane
  }

  auto gs_care_obstacles = session_->mutable_environmental_model()
                               ->get_obstacle_manager()
                               ->get_gs_care_obstacles()
                               .Dict();
  bool agent_node_updated = {false};
  if (origin_lane_coord_ptr != nullptr && target_lane_coord_ptr != nullptr) {
    agent_node_manager->set_input_info(
        origin_lane_coord_ptr, target_lane_coord_ptr, target_state,
        ids_obstacle_in_origin_lane, ids_obstacle_in_target_lane,
        gs_care_obstacles);
    agent_node_updated = agent_node_manager->Update();
  }

  //------------------------------------
  auto &traj_points = session_->mutable_planning_context()
                          ->mutable_planning_result()
                          .traj_points;

  traj_points = coarse_planning_info.trajectory_points;

  auto &general_lateral_decider_output =
      session_->mutable_planning_context()
          ->mutable_hpp_general_lateral_decider_output();
  general_lateral_decider_output.complete_follow =
      false;  // fusion is unsteady, lane keep weight need decay in end of ref
  general_lateral_decider_output.v_cruise = cruise_vel_;

  auto time_start = IflyTime::Now_ms();
  HandleLaneChangeScene(traj_points);  // TODO:handle the lane change info;
  auto time_end = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("HandleLaneChangeScene cost", time_end - time_start)

  time_start = IflyTime::Now_ms();
  ConstructReferencePathPoints(traj_points);
  time_end = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("ConstructReferencePathPoints cost", time_end - time_start)

  ObstacleDecisions obstacle_decisions;
  MapObstacleDecision map_obstacle_decisions;

  time_start = IflyTime::Now_ms();
  ConstructLaneAndBoundaryBounds(map_obstacle_decisions);
  time_end = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("ConstructLaneAndBoundaryBounds cost", time_end - time_start)

  time_start = IflyTime::Now_ms();
  ConstructLateralObstacleDecisions(obstacle_decisions);
  time_end = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("ConstructLateralObstacleDecisions cost",
                   time_end - time_start)

  std::vector<std::pair<double, double>> frenet_safe_bounds;
  std::vector<std::pair<double, double>> frenet_path_bounds;

  time_start = IflyTime::Now_ms();
  ExtractBoundary(map_obstacle_decisions, obstacle_decisions,
                  frenet_safe_bounds, frenet_path_bounds,
                  general_lateral_decider_output);
  time_end = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("ExtractBoundary cost", time_end - time_start)

  time_start = IflyTime::Now_ms();
  GenerateEnuBoundaryPoints(frenet_safe_bounds, frenet_path_bounds,
                            general_lateral_decider_output);
  time_end = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("GenerateEnuBoundaryPoints cost", time_end - time_start)

  time_start = IflyTime::Now_ms();
  GenerateEnuReferenceTheta(general_lateral_decider_output);
  time_end = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("GenerateEnuReferenceTheta cost", time_end - time_start)

  time_start = IflyTime::Now_ms();
  GenerateEnuReferenceTraj(general_lateral_decider_output);
  time_end = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("GenerateEnuReferenceTraj cost", time_end - time_start)

  time_start = IflyTime::Now_ms();
  CalcLateralBehaviorOutput();
  time_end = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("CalcLateralBehaviorOutput cost", time_end - time_start)

  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("HppGeneralLateralDeciderCost", end_time - start_time);

  return true;
}

bool HppGeneralLateralDecider::ExecuteTest(bool pipeline_test) {
  // pipeline test
  return true;
}

void HppGeneralLateralDecider::HandleLaneChangeScene(
    TrajectoryPoints &traj_points) {
  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();
  auto &general_lateral_decider_output =
      session_->mutable_planning_context()
          ->mutable_general_lateral_decider_output();

  const auto &timer =
      session_->planning_context().lane_change_decider_output().lc_timer;
  bool lane_change_flag{false};
  const auto &lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto state = coarse_planning_info.target_state;
  const auto lc_request_direction = lane_change_decider_output.lc_request;
  bool is_LC_LCHANGE =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == LEFT_CHANGE);
  bool is_LC_RCHANGE =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == RIGHT_CHANGE);
  if (is_LC_LCHANGE) {
    lat_lane_change_info_ = LatDeciderLaneChangeInfo::LEFT_LANE_CHANGE;
    lane_change_flag = true;
    general_lateral_decider_output.complete_follow = true;
  } else if (is_LC_RCHANGE) {
    lat_lane_change_info_ = LatDeciderLaneChangeInfo::RIGHT_LANE_CHANGE;
    lane_change_flag = true;
    general_lateral_decider_output.complete_follow = true;
  }

  // protect lane change duration, if total lc time > 8.0s then aggresively lane
  // change by motion planner
  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;

  if (lane_change_flag) {
    double remaining_lane_change_duration =
        config_.lane_change_duration - timer;
    auto ego_v = cruise_vel_;

    auto lat_state = ego_state->planning_init_point().lat_init_state;
    Point2D frenet_init_point;
    Point2D cart_init_point{lat_state.x(), lat_state.y()};
    if (!reference_path_ptr->get_frenet_coord()->XYToSL(cart_init_point,
                                                        frenet_init_point)) {
      LOG_ERROR("ERROR! Frenet Point -> Cart Point Failed!!!");
    }

    // if reamining duration less than 1s, then aggresively lane change
    if (remaining_lane_change_duration < 1.0) {
      return;
    }

    // refine velocity if frenet length is short
    const auto curvature = lat_state.curv();
    const auto heading_angle = lat_state.theta();

    const auto normal_acc = ego_v * ego_v * curvature;

    // const auto &heading_angle =
    // ego_state->planning_init_point().heading_angle; const auto normal_acc
    // = ego_v * ego_v * curvature;
    if (frenet_init_point.x + ego_v * remaining_lane_change_duration >=
        reference_path_ptr->get_frenet_coord()->Length() - 0.5) {
      ego_v = (reference_path_ptr->get_frenet_coord()->Length() -
               frenet_init_point.x - 0.5) /
              std::fmax(remaining_lane_change_duration,
                        (traj_points.size() - 1) * config_.delta_t);
    }
    auto lane_change_end_s =
        frenet_init_point.x + ego_v * remaining_lane_change_duration;

    const auto v_x = ego_v * std::cos(heading_angle);
    const auto v_y = ego_v * std::sin(heading_angle);
    auto normal_acc_x = normal_acc * std::sin(heading_angle);
    auto normal_acc_y = normal_acc * std::cos(heading_angle);

    // set lane change end state
    Point2D frenet_end_point{lane_change_end_s, 0.};
    Point2D cart_end_point;
    if (!reference_path_ptr->get_frenet_coord()->SLToXY(frenet_end_point,
                                                        cart_end_point)) {
      LOG_ERROR("ERROR! Frenet Point -> Cart Point Failed!!!");
    }

    const auto lane_change_end_heading_angle =
        reference_path_ptr->get_frenet_coord()->GetPathCurveHeading(
            lane_change_end_s);
    const auto lane_change_end_curvature =
        reference_path_ptr->get_frenet_coord()
            ->GetPathPointByS(lane_change_end_s)
            .kappa();

    const auto normal_acc_end = ego_v * ego_v * lane_change_end_curvature;

    const auto v_x_end = ego_v * std::cos(lane_change_end_heading_angle);
    const auto v_y_end = ego_v * std::sin(lane_change_end_heading_angle);
    auto normal_acc_x_end =
        normal_acc_end * std::sin(lane_change_end_heading_angle);
    auto normal_acc_y_end =
        normal_acc_end * std::cos(lane_change_end_heading_angle);

    // construct quintic path for lane change duration
    QuinticPolynominalPath lane_change_quintic_path;

    Eigen::Vector2d x0(ego_state->planning_init_point().x,
                       ego_state->planning_init_point().y);
    Eigen::Vector2d dx0(v_x, v_y);
    Eigen::Vector2d ddx0(normal_acc_x, normal_acc_y);
    Eigen::Vector2d xT(cart_end_point.x, cart_end_point.y);
    Eigen::Vector2d dxT(v_x_end, v_y_end);
    Eigen::Vector2d ddxT(normal_acc_x_end, normal_acc_y_end);
    lane_change_quintic_path.SetPoints(x0, xT, dx0, dxT, ddx0, ddxT,
                                       remaining_lane_change_duration);

    // sample traj point on quintic path by t
    TrajectoryPoint point;
    Eigen::Vector2d sample_point;
    Point2D frenet_point;
    size_t truncation_idx = 0;
    double s_truncation{0.0};
    for (size_t i = 0; i < traj_points.size(); i++) {
      if (traj_points[i].t < remaining_lane_change_duration) {
        sample_point = lane_change_quintic_path(traj_points[i].t);
        point.x = sample_point.x();
        point.y = sample_point.y();
        point.heading_angle =
            lane_change_quintic_path.heading(traj_points[i].t);

        Point2D cart_point(point.x, point.y);
        if (!reference_path_ptr->get_frenet_coord()->XYToSL(cart_point,
                                                            frenet_point)) {
          LOG_ERROR("ERROR! Frenet Point -> Cart Point Failed!!!");
        }
        point.s = frenet_point.x;
        point.l = frenet_point.y;
        point.t = traj_points[i].t;

        traj_points[i] = point;
        truncation_idx = i;
      } else {
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
        point.s = std::fmin(
            s_truncation + ego_v * (i - truncation_idx) * config_.delta_t,
            reference_path_ptr->get_frenet_coord()->Length());
        point.l = 0.;
        point.t = traj_points[i].t;

        frenet_point.x = point.s;
        frenet_point.y = point.l;
        Point2D cart_point;
        if (!reference_path_ptr->get_frenet_coord()->SLToXY(frenet_point,
                                                            cart_point)) {
          LOG_ERROR("ERROR! Cart Point -> Frenet Point Failed!!!");
        }
        point.x = cart_point.x;
        point.y = cart_point.y;

        point.heading_angle =
            reference_path_ptr->get_frenet_coord()->GetPathCurveHeading(
                point.s);
        traj_points[i] = point;
      }
    }
    return;
  } else {
    return;
  }
}

bool HppGeneralLateralDecider::ConstructReferencePathPoints(
    const TrajectoryPoints &traj_points) {
  ref_traj_points_.clear();
  ref_path_points_.clear();
  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;

  for (const auto &traj_point : traj_points) {
    ref_traj_points_.emplace_back(traj_point);
    ReferencePathPoint refpath_pt{};
    if (!reference_path_ptr->get_reference_point_by_lon(traj_point.s,
                                                        refpath_pt)) {
      // add logs
      LOG_ERROR("Get reference point by lon failed!");
    }
    ref_path_points_.emplace_back(refpath_pt);
  }
  return true;
}

void HppGeneralLateralDecider::ConstructLaneAndBoundaryBounds(
    MapObstacleDecision &map_obstacle_decisions) {
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();

  double left_border_distance{10.};
  double right_border_distance{10.};
  double l_offset_limit = 20.0;

  std::vector<std::pair<int, Polygon2d>> left_groundline_polygons,
      right_groundline_polygons, left_parking_space_polygons,
      right_parking_space_polygons;
  ConstructStaticObstacleTotalPolygons(
      left_groundline_polygons, right_groundline_polygons,
      left_parking_space_polygons, right_parking_space_polygons);
  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;

  for (size_t i = 0; i < ref_traj_points_.size(); i++) {
    Bound path_bound{-l_offset_limit, l_offset_limit};
    Bound safe_bound{-l_offset_limit, l_offset_limit};
    MapObstaclePositionDecision map_obstacle_decision;

    map_obstacle_decision.tp.t = ref_traj_points_[i].t;
    map_obstacle_decision.tp.s = ref_traj_points_[i].s;
    map_obstacle_decision.tp.l = ref_traj_points_[i].l;

    double left_lane_distance = 10.;
    double right_lane_distance = 10.;
    double left_road_distance = 10.;
    double right_road_distance = 10.;
    double ego_s = ref_traj_points_[i].s;
    double ego_length = vehicle_param.length;
    double ego_width = vehicle_param.width;
    double care_area_s_start = ego_s - ego_length / 2;
    double care_area_s_end = ego_s + ego_length / 2 + config_.care_area_s_len;
    auto care_area_center = Vec2d((care_area_s_start + care_area_s_end) / 2, 0);
    double care_area_length = care_area_s_end - care_area_s_start;
    auto care_polygon = Polygon2d(
        Box2d(care_area_center, 0, care_area_length, config_.l_care_width));

    SampleRoadDistanceInfo(map_obstacle_decision.tp.s, ref_path_points_[i]);
    // distance to lane is unaccurate near end of the ref line
    // need to avoid the intersection of lane bound
    size_t lower_truncation_idx = 0;
    size_t upper_truncation_idx = 0;
    if (ref_path_points_[i].distance_to_left_lane_border >
        0.5 * vehicle_param.width + config_.buffer2lane) {
      upper_truncation_idx = i;
    } else {
      ref_path_points_[i].distance_to_left_lane_border =
          ref_path_points_[upper_truncation_idx].distance_to_left_lane_border;
    }

    if (ref_path_points_[i].distance_to_right_lane_border >
        0.5 * vehicle_param.width + config_.buffer2lane) {
      lower_truncation_idx = i;
    } else {
      ref_path_points_[i].distance_to_right_lane_border =
          ref_path_points_[lower_truncation_idx].distance_to_right_lane_border;
    }

    ObstacleBorderInfo left_groundline_obstacle_border =
        GetNearestObstacleBorder(care_polygon, care_area_s_start,
                                 care_area_s_end, left_groundline_polygons,
                                 true, false, false, i, ref_traj_points_);
    ObstacleBorderInfo right_groundline_obstacle_border =
        GetNearestObstacleBorder(care_polygon, care_area_s_start,
                                 care_area_s_end, right_groundline_polygons,
                                 false, false, false, i, ref_traj_points_);

    ObstacleBorderInfo left_parking_space_border = GetNearestObstacleBorder(
        care_polygon, care_area_s_start, care_area_s_end,
        left_parking_space_polygons, true, false, false, i, ref_traj_points_);
    ObstacleBorderInfo right_parking_space_border = GetNearestObstacleBorder(
        care_polygon, care_area_s_start, care_area_s_end,
        right_parking_space_polygons, false, false, false, i, ref_traj_points_);

    double static_obstacle_collision_center_distance = vehicle_param.width / 2;
    double static_obstacle_safe_center_distance =
        config_.static_obj_safe_buffer + vehicle_param.width / 2;
    double static_obstacle_buffer_extra =
        std::fabs(ref_path_points_[i].path_point.kappa) /
        config_.max_ref_curvature * 0.2;
    double static_obstacle_safe_buffer_extra = 0.2;

    // Groundline obstacle borders
    Bound groundline_obstacle_bound{-l_offset_limit, l_offset_limit};
    groundline_obstacle_bound.upper =
        std::fmin(groundline_obstacle_bound.upper,
                  left_groundline_obstacle_border.obstacle_border -
                      static_obstacle_collision_center_distance -
                      static_obstacle_buffer_extra);
    groundline_obstacle_bound.lower =
        std::fmax(groundline_obstacle_bound.lower,
                  right_groundline_obstacle_border.obstacle_border +
                      static_obstacle_collision_center_distance +
                      static_obstacle_buffer_extra);
    Bound groundline_obstacle_safe_bound{-l_offset_limit, l_offset_limit};
    groundline_obstacle_safe_bound.upper =
        std::fmin(groundline_obstacle_safe_bound.upper,
                  left_groundline_obstacle_border.obstacle_border -
                      static_obstacle_safe_center_distance -
                      static_obstacle_safe_buffer_extra);
    groundline_obstacle_safe_bound.lower =
        std::fmax(groundline_obstacle_safe_bound.lower,
                  right_groundline_obstacle_border.obstacle_border +
                      static_obstacle_safe_center_distance +
                      static_obstacle_safe_buffer_extra);

    // Parking Space borders
    Bound parking_space_bound{-l_offset_limit, l_offset_limit};
    parking_space_bound.upper =
        std::fmin(parking_space_bound.upper,
                  left_parking_space_border.obstacle_border -
                      static_obstacle_collision_center_distance -
                      static_obstacle_buffer_extra);
    parking_space_bound.lower =
        std::fmax(parking_space_bound.lower,
                  right_parking_space_border.obstacle_border +
                      static_obstacle_collision_center_distance +
                      static_obstacle_buffer_extra);
    Bound parking_space_safe_bound{-l_offset_limit, l_offset_limit};
    parking_space_safe_bound.upper =
        std::fmin(parking_space_safe_bound.upper,
                  left_parking_space_border.obstacle_border -
                      static_obstacle_safe_center_distance -
                      static_obstacle_safe_buffer_extra);
    parking_space_safe_bound.lower =
        std::fmax(parking_space_safe_bound.lower,
                  right_parking_space_border.obstacle_border +
                      static_obstacle_safe_center_distance +
                      static_obstacle_safe_buffer_extra);

    if (lat_lane_change_info_ == LatDeciderLaneChangeInfo::NONE) {
      safe_bound.upper =
          std::fmin(ref_path_points_[i].distance_to_left_lane_border -
                        0.5 * vehicle_param.width - config_.buffer2lane,
                    safe_bound.upper);
      safe_bound.lower =
          std::fmax(-ref_path_points_[i].distance_to_right_lane_border +
                        0.5 * vehicle_param.width + config_.buffer2lane,
                    safe_bound.lower);
      path_bound.upper =
          std::fmin(ref_path_points_[i].distance_to_left_lane_border -
                        0.5 * vehicle_param.width - config_.buffer2border,
                    path_bound.upper);
      path_bound.lower =
          std::fmax(-ref_path_points_[i].distance_to_right_lane_border +
                        0.5 * vehicle_param.width + config_.buffer2border,
                    path_bound.lower);
    } else if (lat_lane_change_info_ ==
               LatDeciderLaneChangeInfo::LEFT_LANE_CHANGE) {
      safe_bound.upper =
          std::fmin(ref_path_points_[i].distance_to_left_lane_border -
                        0.5 * vehicle_param.width - config_.buffer2lane,
                    safe_bound.upper);
      safe_bound.lower =
          std::fmax(  // NOTE: Although the lower bound needs to be slacked, it
                      // cannot exceed right road @cai
              -ref_path_points_[i].distance_to_right_road_border +
                  0.5 * vehicle_param.width + config_.buffer2lane,
              safe_bound.lower);
      path_bound.upper =
          std::fmin(ref_path_points_[i].distance_to_left_lane_border -
                        0.5 * vehicle_param.width - config_.buffer2border,
                    path_bound.upper);
      path_bound.lower =
          std::fmax(-ref_path_points_[i].distance_to_right_road_border +
                        0.5 * vehicle_param.width + config_.buffer2border,
                    path_bound.lower);
    } else {
      safe_bound.upper =
          std::fmin(ref_path_points_[i].distance_to_left_road_border -
                        0.5 * vehicle_param.width - config_.buffer2lane,
                    safe_bound.upper);
      safe_bound.lower =
          std::fmax(-ref_path_points_[i].distance_to_right_lane_border +
                        0.5 * vehicle_param.width + config_.buffer2lane,
                    safe_bound.lower);
      path_bound.upper =
          std::fmin(ref_path_points_[i].distance_to_left_road_border -
                        0.5 * vehicle_param.width - config_.buffer2border,
                    path_bound.upper);
      path_bound.lower =
          std::fmax(-ref_path_points_[i].distance_to_right_lane_border +
                        0.5 * vehicle_param.width + config_.buffer2border,
                    path_bound.lower);
    }

    map_obstacle_decision.lat_bounds.emplace_back(WeightedBound{
        safe_bound.lower, safe_bound.upper, config_.kPhysicalBoundWeight,
        BoundInfo{-100, BoundType::LANE}});
    map_obstacle_decision.lat_bounds.emplace_back(WeightedBound{
        path_bound.lower, path_bound.upper, config_.kHardBoundWeight,
        BoundInfo{-100, BoundType::ROAD_BORDER}});
    map_obstacle_decision.lat_bounds.emplace_back(
        WeightedBound{groundline_obstacle_bound.lower, l_offset_limit,
                      config_.kHardBoundWeight,
                      BoundInfo{right_groundline_obstacle_border.obstacle_id,
                                BoundType::GROUNDLINE}});
    map_obstacle_decision.lat_bounds.emplace_back(
        WeightedBound{groundline_obstacle_safe_bound.lower, l_offset_limit,
                      config_.kPhysicalBoundWeight,
                      BoundInfo{right_groundline_obstacle_border.obstacle_id,
                                BoundType::GROUNDLINE}});
    map_obstacle_decision.lat_bounds.push_back(
        WeightedBound{parking_space_bound.lower, l_offset_limit,
                      config_.kSolidLaneBoundWeight,
                      BoundInfo{right_parking_space_border.obstacle_id,
                                BoundType::PARKING_SPACE}});
    map_obstacle_decision.lat_bounds.push_back(
        WeightedBound{parking_space_safe_bound.lower, l_offset_limit,
                      config_.kVirtualLaneBoundWeight,
                      BoundInfo{right_parking_space_border.obstacle_id,
                                BoundType::PARKING_SPACE}});
    map_obstacle_decision.lat_bounds.emplace_back(
        WeightedBound{-l_offset_limit, groundline_obstacle_bound.upper,
                      config_.kHardBoundWeight,
                      BoundInfo{left_groundline_obstacle_border.obstacle_id,
                                BoundType::GROUNDLINE}});
    map_obstacle_decision.lat_bounds.emplace_back(
        WeightedBound{-l_offset_limit, groundline_obstacle_safe_bound.upper,
                      config_.kPhysicalBoundWeight,
                      BoundInfo{left_groundline_obstacle_border.obstacle_id,
                                BoundType::GROUNDLINE}});
    map_obstacle_decision.lat_bounds.push_back(
        WeightedBound{-l_offset_limit, parking_space_bound.upper,
                      config_.kSolidLaneBoundWeight,
                      BoundInfo{left_parking_space_border.obstacle_id,
                                BoundType::PARKING_SPACE}});
    map_obstacle_decision.lat_bounds.push_back(
        WeightedBound{-l_offset_limit, parking_space_safe_bound.upper,
                      config_.kVirtualLaneBoundWeight,
                      BoundInfo{left_parking_space_border.obstacle_id,
                                BoundType::PARKING_SPACE}});

    map_obstacle_decisions.emplace_back(std::move(map_obstacle_decision));
  }
}

void HppGeneralLateralDecider::ConstructLateralObstacleDecisions(
    // const TrajectoryPoints &traj_points,
    ObstacleDecisions &obstacle_decisions) {
  ObstaclePotentialDecisions obstacle_potential_decisions;

  int32_t obj_cnt = 0;
  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;

  const auto &obs_vec = reference_path_ptr->get_obstacles();
  const auto &history_obs_vec = history_obstacle_manager_->GetOldObstacles();
  // new current obstacle. only apply to current function
  std::vector<std::shared_ptr<FrenetObstacle>> current_obstacles;
  for (std::shared_ptr<FrenetObstacle> obstacle : obs_vec) {
    bool is_use_history_static_obs = false;
    if (obstacle->b_frenet_valid()) {
      for (const Obstacle &history_obstacle : history_obs_vec) {
        if ((history_obstacle.id() == obstacle->id()) &&
            (history_obstacle.is_static())) {
          is_use_history_static_obs = true;
          break;
        }
      }
      if (is_use_history_static_obs) {
        continue;
      }
      current_obstacles.emplace_back(obstacle);
    }
  }

  auto time_start = IflyTime::Now_ms();
  if (is_deduce_near_obstacles_) {  // deduce obstacle pos & traj
    history_obstacle_manager_->AddNewDeductionObstacles(reference_path_ptr,
                                                        current_obstacles);
  }
  auto time_end = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("SelectObstacleNearEgo Cost", time_end - time_start)

  for (auto &obstacle : current_obstacles) {
    const auto &otype = obstacle->type();
    const auto ofusion_source = obstacle->obstacle()->fusion_source();
    if ((ofusion_source & OBSTACLE_SOURCE_CAMERA) == 0) {
      LOG_ERROR("The obstacle's fusion source is no camera whose id : %d \n",
                obstacle->id());
      continue;
    }
    if (otype == iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN ||  // TBD: check
                                                               // obstacle type
        otype == iflyauto::OBJECT_TYPE_UNKNOWN_MOVABLE ||
        otype == iflyauto::OBJECT_TYPE_UNKNOWN_IMMOVABLE ||
        otype == iflyauto::OBJECT_TYPE_VAN ||
        otype == iflyauto::OBJECT_TYPE_TRAILER ||
        otype == iflyauto::OBJECT_TYPE_TRAFFIC_TEM_SIGN ||
        otype == iflyauto::OBJECT_TYPE_PEDESTRIAN) {  // hpp hack
      // add logs;
      continue;
    }
    const auto &obstacle_id = obstacle->id();
    auto obstacle_decision = ObstacleDecision{obstacle_id, {}, {}};
    auto obstacle_potential_decision =
        ObstaclePotentialDecision{obstacle_id, {}};

    ConstructLateralObstacleDecision(obstacle, obstacle_decision);

    obstacle_decisions[obstacle_id] = std::move(obstacle_decision);
    if (obstacle_potential_decision.extend_decisions.extended) {
      obstacle_potential_decisions[obstacle_id] =
          std::move(obstacle_potential_decision);
    }

    obj_cnt++;
  }
  JSON_DEBUG_VALUE("LateralObstacleDecisioNum", obj_cnt)
}

void HppGeneralLateralDecider::ConstructLateralObstacleDecision(
    const std::shared_ptr<FrenetObstacle> obstacle,
    ObstacleDecision &obstacle_decision) {
  // NTRACE_CALL();
  using namespace planning_math;
  if (ref_traj_points_.empty() || ref_path_points_.empty()) {
    // add logs
    LOG_ERROR("Ref traj points or ref path points is null!");
    return;
  }

  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();

  // TBD: consider the otype in CP scenario;

  // Step 1) configs
  const auto &l_care_width = config_.l_care_width;
  const auto &care_object_lat_distance_threshold =
      config_.care_obj_lat_distance_threshold;
  const auto &care_object_lon_distance_threshold =
      config_.care_obj_lon_distance_threshold;
  const auto &ego_cur_s = ref_traj_points_.front().s;
  const auto &ego_cur_l = ref_traj_points_.front().l;
  const auto &ego_velocity = cruise_vel_;

  auto safe_center_distance =
      config_.dynamic_obj_safe_buffer + vehicle_param.width / 2;
  auto collision_center_distance = vehicle_param.width / 2;
  auto l_offset_limit = 10.0;
  auto avoid_cross_lane = 0.2;
  auto pre_lateral_decision = LatObstacleDecisionType::IGNORE;

  double rear_axle_to_front_bumper =  // TBD：define as consexpr
      vehicle_param.length - vehicle_param.rear_edge_to_rear_axle;
  const bool init_lon_no_overlap =
      (obstacle->frenet_obstacle_boundary().s_end <
           ego_cur_s - vehicle_param.rear_edge_to_rear_axle ||
       obstacle->frenet_obstacle_boundary().s_start >
           ego_cur_s + rear_axle_to_front_bumper);
  const bool init_lat_overlap = !(obstacle->frenet_obstacle_boundary().l_end <
                                      ego_cur_l - vehicle_param.width / 2 ||
                                  obstacle->frenet_obstacle_boundary().l_start >
                                      ego_cur_l + vehicle_param.width / 2);

  // Polygon2d obstacle_start_sl_polygon;
  Polygon2d obstacle_end_sl_polygon;
  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  // const bool ok_start = obstacle->get_polygon_at_time(  // TBD: no prediction
  //     0., reference_path_ptr, obstacle_start_sl_polygon);
  const bool ok_end = obstacle->get_polygon_at_time(  // TBD: no prediction
      ref_traj_points_.back().t, reference_path_ptr, obstacle_end_sl_polygon);

  double dynamic_bound_gain_vel = std::max(config_.min_gain_vel, ego_velocity);
  double dynamic_bound_slack_coefficient =
      1.0 / dynamic_bound_gain_vel / dynamic_bound_gain_vel;
  double nudge_obj_extra_buffer = 0.2;
  bool nudge_obj_flag{false};
  bool reset_conflict_decision{false};

  obstacle_decision.rel_pos_type = ObsRelPosType::NONE;

  auto care_dynamic_object_t_threshold =
      config_.care_dynamic_object_t_threshold;
  // bool is_approach_to_destination =
  //     distance_to_destination <
  //     route_extend_length +
  //     config_.approach_distance_threshold_avoid_obstacle;  // // TBD:
  // didnt
  //     prepared yet

  // Step 2) filter far away objects
  if (std::fabs(obstacle->frenet_s() - ego_cur_s) >
          care_object_lon_distance_threshold or
      std::fabs(obstacle->frenet_l() - ego_cur_l) >
          care_object_lat_distance_threshold) {
    // TBD: add log
    return;
  }

  // Step 3) filter rear objects
  if (obstacle->frenet_obstacle_boundary().s_end +
          config_.lon_rear_car_filter_buffer <
      ego_cur_s) {
    // TBD: add log
    return;
  }

  auto safe_extra_distance = 0.0;
  if (obstacle->type() == iflyauto::OBJECT_TYPE_TRAFFIC_CONE ||
      obstacle->type() == iflyauto::OBJECT_TYPE_BUS ||
      obstacle->type() == iflyauto::OBJECT_TYPE_PEDESTRIAN ||
      obstacle->type() == iflyauto::OBJECT_TYPE_TRUCK) {
    safe_extra_distance += 0.2;
  }

  if (CheckObstacleNudgeCondition(
          obstacle)) {  // TBD: execute nudge logic in this function:
                        //       1. whether to nudge?
                        //       2. calc the extra buffer
                        //       3. set the lat type as NUDGE
    // TBD: add log;
    safe_extra_distance += nudge_obj_extra_buffer;
  }

  bool is_cross_obj{false};
  bool has_lat_decision{false};
  bool has_lon_decision{false};
  bool static_obstacle_loop_flag{true};

  Polygon2d obstacle_sl_polygon;

  // Step 5) calculate safe_bound, path_bound
  for (size_t i = 0; i < ref_traj_points_.size(); i++) {
    auto &traj_point = ref_traj_points_[i];
    const auto &t = traj_point.t;
    const auto &ego_s = traj_point.s;
    const auto &ego_l = traj_point.l;

    // double kCareAreaSBuffer =
    //     std::max(std::min(5.0, 5.0 - 0.5 * (10.0 - ego_velocity)),
    //              0.0);  // @cai: currently use fixed distance

    auto care_area_s_start = ego_s - vehicle_param.rear_edge_to_rear_axle -
                             config_.care_area_s_start_buffer;
    auto care_area_s_end = ego_s + rear_axle_to_front_bumper + kCareAreaSBuffer;
    auto care_area_center =
        Vec2d((care_area_s_start + care_area_s_end) * 0.5, ego_l);
    auto care_area_length = care_area_s_end - care_area_s_start;
    auto care_polygon =  // @cai: consider the heading
        Polygon2d(Box2d(care_area_center, 0, care_area_length, l_care_width));

    bool ok = true;
    if (obstacle->obstacle()->is_static() && static_obstacle_loop_flag) {
      ok = obstacle->get_polygon_at_time(0, reference_path_ptr,
                                         obstacle_sl_polygon);
      static_obstacle_loop_flag = false;
    } else if (not obstacle->obstacle()->is_static()) {
      if (t > care_dynamic_object_t_threshold) {
        continue;
      }
      ok = obstacle->get_polygon_at_time(
          i * config_.delta_t, reference_path_ptr, obstacle_sl_polygon);
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
    // frenet_length is larger than raw length when obj is in lagre curvature
    // road

    /*@cailiu:
      ComputeOverlap heavily depends on objs predication,
      Obstacle lat avoidance and bypassing is not currently being considered
    */

    b_overlap_with_care =
        obstacle_sl_polygon.ComputeOverlap(care_polygon, &care_overlap_polygon);
    if (b_overlap_with_care) {
      // TBD: add log
      overlap_min_y = care_overlap_polygon.min_y();
      overlap_max_y = care_overlap_polygon.max_y();
      LOG_DEBUG("Obstacle[%d] decision, t is: %f, min_y: %f, max_y: %f",
                obstacle->id(), i * config_.delta_t, overlap_min_y,
                overlap_max_y);
    } else {
      // TBD: add log
      LOG_DEBUG("Obstacle[%d] no decision, t is: %f", obstacle->id(),
                i * config_.delta_t);
      continue;
    }

    // todo: high speed vehicle
    // do decision
    auto lat_decision = LatObstacleDecisionType::IGNORE;
    auto lon_decision = LonObstacleDecisionType::IGNORE;
    Bound path_bound{-l_offset_limit, l_offset_limit};
    Bound safe_bound{-l_offset_limit, l_offset_limit};
    BoundInfo path_bound_info;
    BoundInfo safe_bound_info;

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

    const auto &nudge_reference_center_l = ego_l;

    if ((overlap_min_y <= nudge_reference_center_l &&
         nudge_reference_center_l <= overlap_max_y) ||
        is_cross_obj) {
      lat_decision = LatObstacleDecisionType::IGNORE;
      lon_decision = LonObstacleDecisionType::YIELD;

    } else if (overlap_min_y > nudge_reference_center_l) {
      auto avoid_right_edge = overlap_min_y - safe_center_distance -
                              safe_extra_distance - vehicle_param.width / 2;
      if (avoid_right_edge >
              std::fmax(-ref_path_points_[i].distance_to_right_lane_border -
                            avoid_cross_lane,
                        -config_.max_avoid_edge) or
          b_overlap_side) {
        lat_decision = LatObstacleDecisionType::RIGHT;
        lon_decision = LonObstacleDecisionType::YIELD;
      } else {
        lat_decision = LatObstacleDecisionType::IGNORE;
        lon_decision = LonObstacleDecisionType::YIELD;
      }
    } else {
      assert(overlap_max_y < nudge_reference_center_l);
      auto avoid_left_edge = overlap_max_y + safe_center_distance +
                             safe_extra_distance + vehicle_param.width / 2;
      if (avoid_left_edge <
              std::min(ref_path_points_[i].distance_to_left_lane_border +
                           avoid_cross_lane,
                       config_.max_avoid_edge) or
          b_overlap_side) {
        lat_decision = LatObstacleDecisionType::LEFT;
        lon_decision = LonObstacleDecisionType::YIELD;
      } else {
        lat_decision = LatObstacleDecisionType::IGNORE;
        lon_decision = LonObstacleDecisionType::YIELD;
      }
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

    if (lat_decision == LatObstacleDecisionType::LEFT) {
      path_bound.lower =
          care_overlap_polygon.max_y() + collision_center_distance;
      safe_bound.lower = care_overlap_polygon.max_y() + safe_center_distance +
                         safe_extra_distance;
      path_bound_info.type = BoundType::AGENT;
      safe_bound_info.type = BoundType::AGENT;
      path_bound_info.id = obstacle->id();
      safe_bound_info.id = obstacle->id();
    } else if (lat_decision == LatObstacleDecisionType::RIGHT) {
      path_bound.upper =
          care_overlap_polygon.min_y() - collision_center_distance;
      safe_bound.upper = care_overlap_polygon.min_y() - safe_center_distance -
                         safe_extra_distance;
      path_bound_info.type = BoundType::AGENT;
      safe_bound_info.type = BoundType::AGENT;
      path_bound_info.id = obstacle->id();
      safe_bound_info.id = obstacle->id();
    } else {
      // assert(lon_decision != LonObstacleDecisionType::IGNORE);
    }

    // refine safe bound a.c. model traj and path bound
    // if (lat_decision == LatObstacleDecisionType::LEFT) {
    //   if (ego_l < safe_bound.lower) {
    //     if (ego_l > path_bound.lower + config_.min_obstacle_avoid_distance) {
    //       double origin_avoid_buffer = safe_bound.lower - path_bound.lower;
    //       double adjust_avoid_buffer = planning::planning_math::lerp(
    //           config_.min_obstacle_avoid_distance, 0.0, origin_avoid_buffer,
    //           1.0, (ego_l - path_bound.lower) / origin_avoid_buffer);
    //       safe_bound.lower =
    //           path_bound.lower +
    //           std::max(config_.min_obstacle_avoid_distance,
    //                                       adjust_avoid_buffer);

    //     } else {
    //       safe_bound.lower =
    //           path_bound.lower + config_.min_obstacle_avoid_distance;
    //     }
    //   }
    //   if (ego_l < safe_bound.lower) {
    //     double converge_coeff = std::min(
    //         1.0, std::pow(config_.lateral_bound_converge_speed * t, 2));
    //     safe_bound.lower = planning::planning_math::lerp(
    //         ego_l, 0, safe_bound.lower, 1.0, converge_coeff);
    //     safe_bound.lower =
    //         std::max(safe_bound.lower,
    //                  path_bound.lower + config_.min_obstacle_avoid_distance);
    //   }
    // } else if (lat_decision == LatObstacleDecisionType::RIGHT) {
    //   if (ego_l > safe_bound.upper) {
    //     if (ego_l < path_bound.upper - config_.min_obstacle_avoid_distance) {
    //       double origin_avoid_buffer = path_bound.upper - safe_bound.upper;
    //       double adjust_avoid_buffer = planning::planning_math::lerp(
    //           config_.min_obstacle_avoid_distance, 0.0, origin_avoid_buffer,
    //           1.0, (path_bound.upper - ego_l) / origin_avoid_buffer);
    //       safe_bound.upper =
    //           path_bound.upper -
    //           std::max(config_.min_obstacle_avoid_distance,
    //                                       adjust_avoid_buffer);

    //     } else {
    //       safe_bound.upper =
    //           path_bound.upper - config_.min_obstacle_avoid_distance;
    //     }
    //   }
    //   if (ego_l > safe_bound.upper) {
    //     double converge_coeff = std::min(
    //         1.0, std::pow(config_.lateral_bound_converge_speed * t, 2));
    //     safe_bound.upper = planning::planning_math::lerp(
    //         ego_l, 0.0, safe_bound.upper, 1.0, converge_coeff);
    //     safe_bound.upper =
    //         std::min(safe_bound.upper,
    //                  path_bound.upper - config_.min_obstacle_avoid_distance);
    //   }
    // }

    has_lat_decision =
        has_lat_decision || lat_decision != LatObstacleDecisionType::IGNORE;
    has_lon_decision =
        has_lon_decision || lon_decision != LonObstacleDecisionType::IGNORE;

    ObstaclePositionDecision position_decision;
    position_decision.tp.t = t;
    position_decision.tp.s = ego_s;
    position_decision.tp.l = ego_l;

    position_decision.lat_bounds.push_back(
        WeightedBound{path_bound.lower, path_bound.upper,
                      config_.kHardBoundWeight, path_bound_info});
    position_decision.lat_bounds.push_back(WeightedBound{
        safe_bound.lower, safe_bound.upper,
        config_.dynamic_bound_slack_coefficient * config_.kPhysicalBoundWeight,
        safe_bound_info});
    position_decision.lat_decision = lat_decision;
    position_decision.lon_decision = lon_decision;

    obstacle_decision.position_decisions.emplace_back(
        std::move(position_decision));
  }

  // update relative position:

  // TBD: ObsRelPosType not use yet
  // if (init_lat_overlap && obstacle->frenet_obstacle_boundary().s_start >
  //                             ego_cur_s + rear_axle_to_front_bumper) {
  //   obstacle_decision.rel_pos_type = ObsRelPosType::FRONT;
  // } else if (is_cross_obj) {
  //   obstacle_decision.rel_pos_type = ObsRelPosType::CROSSING;
  // } else if (has_lon_decision) {
  //   obstacle_decision.rel_pos_type = ObsRelPosType::CUTIN;
  // } else if (has_lat_decision) {
  //   obstacle_decision.rel_pos_type = ObsRelPosType::ADJACENT;
  // }
}

bool HppGeneralLateralDecider::CheckObstacleNudgeCondition(

    const std::shared_ptr<FrenetObstacle> &obstacle) {
  // nudge logic
  return false;
}

bool HppGeneralLateralDecider::CheckObstacleCrossingCondition(
    const std::shared_ptr<FrenetObstacle> obstacle, bool &is_cross_obj) {
  // check crossing:
  // constexpr double kCrossVelThreshold{2.0};
  // auto obstacle_point =
  //     obstacle->obstacle()->get_point_at_time(t);  // TBD: no prediction yet
  // double relative_yaw =
  //     obstacle_point.velocity_direction -
  //     reference_path_ptr_->get_frenet_coord()->GetRefCurveHeading(
  //         (obstacle_sl_polygon.min_x() + obstacle_sl_polygon.max_x()) / 2.0);
  // double obstacle_v_lat = std::fabs(obstacle_point.v *
  // std::sin(relative_yaw)); if (ok_start && ok_end) {
  //   bool left2right =
  //       obstacle_start_sl_polygon.min_y() >
  //           ego_l + vehicle_param.width
  //           / 2.0
  //           &&
  //       obstacle_end_sl_polygon.max_y() <
  //           ego_l - vehicle_param.width
  //           / 2.0;
  //   bool right2left =
  //       obstacle_start_sl_polygon.max_y() <
  //           ego_l - vehicle_param.width
  //           / 2.0
  //           &&
  //       obstacle_end_sl_polygon.min_y() >
  //           ego_l + vehicle_param.width
  //           / 2.0;
  //   is_cross_obj = is_cross_obj || (obstacle_v_lat > kCrossVelThreshold &&
  //                                   (left2right || right2left));
  // }
  return false;
}

void HppGeneralLateralDecider::RefineConflictLatDecisions(
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

void HppGeneralLateralDecider::ExtractBoundary(
    const MapObstacleDecision &map_obstacle_decision,
    const ObstacleDecisions &obstacle_decisions,
    std::vector<std::pair<double, double>> &frenet_safe_bounds,
    std::vector<std::pair<double, double>> &frenet_path_bounds,
    HppGeneralLateralDeciderOutput &general_lateral_decider_output) {
  assert(map_obstacle_decision.size() == ref_traj_points_.size());

  // auto &path_bounds = general_lateral_decider_output.path_bounds;
  // auto &safe_bounds = general_lateral_decider_output.path_bounds;
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

  std::vector<WeightedBounds> path_bounds;
  std::vector<WeightedBounds> safe_bounds;
  path_bounds.resize(ref_traj_points_.size());
  safe_bounds.resize(ref_traj_points_.size());

  for (size_t i = 0; i < map_obstacle_decision.size(); i++) {
    auto &map_obstacle_position_lat_bounds =
        map_obstacle_decision[i].lat_bounds;
    for (size_t j = 0; j < map_obstacle_position_lat_bounds.size(); j++) {
      if (map_obstacle_position_lat_bounds[j].weight < 0.) {
        path_bounds[i].emplace_back(map_obstacle_position_lat_bounds[j]);
      } else {
        safe_bounds[i].emplace_back(map_obstacle_position_lat_bounds[j]);
      }
    }
    lat_debug_info_.mutable_bound_s_vec()->Set(i, ref_traj_points_[i].s);
  }

  for (auto &obstacle_decision : obstacle_decisions) {
    for (auto &obstacle_position_decision :
         obstacle_decision.second.position_decisions) {
      for (size_t i = 0; i < ref_traj_points_.size(); i++) {
        if (std::fabs(obstacle_position_decision.tp.t - ref_traj_points_[i].t) <
            1e-2) {
          auto obstacle_pos_bounds = obstacle_position_decision.lat_bounds;
          for (auto &obstacle_pos_bound : obstacle_pos_bounds) {
            if (obstacle_pos_bound.weight < 0.) {
              path_bounds[i].emplace_back(obstacle_pos_bound);
            } else {
              safe_bounds[i].emplace_back(obstacle_pos_bound);
            }
          }
        }
      }
    }
  }

  for (auto &bounds : path_bounds) {
    std::pair<double, double> tmp_bound{-10., 10.};  // <lower ,upper >
    BoundInfo path_upper_bound_info;
    BoundInfo path_lower_bound_info;
    for (auto &bound : bounds) {
      if (bound.upper < tmp_bound.second) {
        tmp_bound.second = bound.upper;
        path_upper_bound_info.id = bound.bound_info.id;
        path_upper_bound_info.type = bound.bound_info.type;
      }
      if (bound.lower > tmp_bound.first) {
        tmp_bound.first = bound.lower;
        path_lower_bound_info.id = bound.bound_info.id;
        path_lower_bound_info.type = bound.bound_info.type;
      }
    }
    frenet_path_bounds.emplace_back(tmp_bound);
    auto hard_upper_bound_info =
        lat_debug_info_.mutable_hard_upper_bound_info_vec()->Add();
    hard_upper_bound_info->set_upper(tmp_bound.second);
    hard_upper_bound_info->mutable_bound_info()->set_id(
        path_upper_bound_info.id);
    hard_upper_bound_info->mutable_bound_info()->set_type(
        BoundType2String(path_upper_bound_info.type));
    auto hard_lower_bound_info =
        lat_debug_info_.mutable_hard_lower_bound_info_vec()->Add();
    hard_lower_bound_info->set_lower(tmp_bound.first);
    hard_lower_bound_info->mutable_bound_info()->set_id(
        path_lower_bound_info.id);
    hard_lower_bound_info->mutable_bound_info()->set_type(
        BoundType2String(path_lower_bound_info.type));
  }

  for (auto &bounds : safe_bounds) {
    std::pair<double, double> tmp_bound{-10., 10.};  // <lower ,upper >
    BoundInfo safe_upper_bound_info;
    BoundInfo safe_lower_bound_info;
    for (auto &bound : bounds) {
      if (bound.upper < tmp_bound.second) {
        tmp_bound.second = bound.upper;
        safe_upper_bound_info.id = bound.bound_info.id;
        safe_upper_bound_info.type = bound.bound_info.type;
      }
      if (bound.lower > tmp_bound.first) {
        tmp_bound.first = bound.lower;
        safe_lower_bound_info.id = bound.bound_info.id;
        safe_lower_bound_info.type = bound.bound_info.type;
      }
    }
    frenet_safe_bounds.emplace_back(tmp_bound);
    auto soft_upper_bound_info =
        lat_debug_info_.mutable_soft_upper_bound_info_vec()->Add();
    soft_upper_bound_info->set_upper(tmp_bound.second);
    soft_upper_bound_info->mutable_bound_info()->set_id(
        safe_upper_bound_info.id);
    soft_upper_bound_info->mutable_bound_info()->set_type(
        BoundType2String(safe_upper_bound_info.type));
    auto soft_lower_bound_info =
        lat_debug_info_.mutable_soft_lower_bound_info_vec()->Add();
    soft_lower_bound_info->set_lower(tmp_bound.first);
    soft_lower_bound_info->mutable_bound_info()->set_id(
        safe_lower_bound_info.id);
    soft_lower_bound_info->mutable_bound_info()->set_type(
        BoundType2String(safe_lower_bound_info.type));
  }

  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_lateral_behavior_debug_info()
      ->CopyFrom(lat_debug_info_);
  assert(frenet_path_bounds.size() == ref_traj_points_.size());
  assert(frenet_safe_bounds.size() == ref_traj_points_.size());
}

void HppGeneralLateralDecider::GenerateEnuBoundaryPoints(
    const std::vector<std::pair<double, double>> &frenet_safe_bounds,
    const std::vector<std::pair<double, double>> &frenet_path_bounds,
    HppGeneralLateralDeciderOutput &general_lateral_decider_output) {
  auto &safe_bounds_output = general_lateral_decider_output.safe_bounds;
  auto &path_bounds_output = general_lateral_decider_output.path_bounds;
  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;

  const std::shared_ptr<KDPath> frenet_coord =
      reference_path_ptr->get_frenet_coord();
  Point2D tmp_safe_lower_point;
  Point2D tmp_safe_upper_point;
  Point2D tmp_path_lower_point;
  Point2D tmp_path_upper_point;
  for (size_t i = 0; i < ref_traj_points_.size(); ++i) {
    if (!frenet_coord->SLToXY(
            Point2D(ref_traj_points_[i].s, frenet_safe_bounds[i].first),
            tmp_safe_lower_point))  // safe lower
    {
      // TODO: add logs
    }

    if (!frenet_coord->SLToXY(
            Point2D(ref_traj_points_[i].s, frenet_safe_bounds[i].second),
            tmp_safe_upper_point))  // safe upper
    {
      // TODO: add logs
    }
    safe_bounds_output.emplace_back(std::pair<Point2D, Point2D>(
        tmp_safe_lower_point, tmp_safe_upper_point));

    if (!frenet_coord->SLToXY(
            Point2D(ref_traj_points_[i].s, frenet_path_bounds[i].first),
            tmp_path_lower_point))  // path lower
    {
      // TODO: add logs
    }

    if (!frenet_coord->SLToXY(
            Point2D(ref_traj_points_[i].s, frenet_path_bounds[i].second),
            tmp_path_upper_point))  // path upper
    {
      // TODO: add logs
    }

    path_bounds_output.emplace_back(std::pair<Point2D, Point2D>(
        tmp_path_lower_point, tmp_path_upper_point));
  }
};

void HppGeneralLateralDecider::GenerateEnuReferenceTraj(
    HppGeneralLateralDeciderOutput &general_lateral_decider_output) {
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

void HppGeneralLateralDecider::GenerateEnuReferenceTheta(
    HppGeneralLateralDeciderOutput &general_lateral_decider_output) {
  auto &enu_ref_theta = general_lateral_decider_output.enu_ref_theta;

  for (size_t i = 0; i < ref_path_points_.size(); i++) {
    enu_ref_theta.emplace_back(ref_traj_points_[i].heading_angle);
  }
}

void HppGeneralLateralDecider::SampleRoadDistanceInfo(
    const double &s_target, ReferencePathPoint &sample_path_point) {
  const double cut_length = config_.sample_step;
  ReferencePathPoint refpath_pt{};
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;

  for (double s = s_target - vehicle_param.rear_edge_to_rear_axle;
       s < s_target + vehicle_param.front_edge_to_rear_axle +
               config_.sample_forward_distance;
       s += cut_length) {
    if (reference_path_ptr->get_reference_point_by_lon(s, refpath_pt)) {
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
  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  auto &lateral_output = session_->mutable_planning_context()
                             ->mutable_lateral_behavior_planner_output();
  const auto &lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();

  const std::shared_ptr<VirtualLane> flane =
      session_->environmental_model()
          .get_virtual_lane_manager()
          ->get_lane_with_virtual_id(coarse_planning_info.target_lane_id);
  const std::shared_ptr<VirtualLaneManager> virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();

  // path points
  std::vector<planning::PathPoint> path_points;
  if (flane != nullptr) {
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
  lateral_output.lat_offset = 0.0;
  // borrow_bicycle_lane
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

ObstacleBorderInfo HppGeneralLateralDecider::GetNearestObstacleBorder(
    const planning_math::Polygon2d &care_polygon, double care_area_s_start,
    double care_area_s_end,
    const std::vector<std::pair<int, planning_math::Polygon2d>>
        &obstacle_frenet_polygons,
    bool is_left, bool is_sorted, bool is_curve, int index,
    const TrajectoryPoints &traj_points) {
  static constexpr double kMaxLaneBound = 100.0;
  double l_care_width = 10.;
  double nearest_border = is_left ? kMaxLaneBound : -kMaxLaneBound;
  // planning_math::Polygon2d overlap_polygon;
  ObstacleBorderInfo nearest_obstacle_border;
  nearest_obstacle_border.obstacle_id = -100;
  nearest_obstacle_border.obstacle_border = nearest_border;

  for (auto &polygon : obstacle_frenet_polygons) {
    if (polygon.second.max_x() < care_area_s_start) {
      continue;
    }
    if (polygon.second.min_x() > care_area_s_end) {
      if (is_sorted) {
        break;
      }
      continue;
    }
    if (!is_sorted && polygon.second.max_y() * polygon.second.min_y() < 0) {
      continue;
    }
    if (std::min(std::fabs(polygon.second.min_y()),
                 std::fabs(polygon.second.max_y())) > l_care_width)
      continue;
    nearest_border = is_left
                         ? std::fmin(nearest_border, polygon.second.min_y())
                         : std::fmax(nearest_border, polygon.second.max_y());
    if (nearest_obstacle_border.obstacle_border != nearest_border) {
      nearest_obstacle_border.obstacle_id = polygon.first;
      nearest_obstacle_border.obstacle_border = nearest_border;
    }
  }
  return nearest_obstacle_border;
}

void HppGeneralLateralDecider::ConstructStaticObstacleTotalPolygons(
    vector<pair<int, Polygon2d>> &left_groundline_polygons,
    vector<pair<int, Polygon2d>> &right_groundline_polygons,
    vector<pair<int, Polygon2d>> &left_parking_space_polygons,
    vector<pair<int, Polygon2d>> &right_parking_space_polygons) {
  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  const std::shared_ptr<KDPath> &frenet_coord =
      reference_path_ptr->get_frenet_coord();
  // Step1: 主要区分 lines 类型 和 polygon 类型（slot &
  // pillar），生成所有polygons Step 1.1 : 处理 lines
  for (auto &obstacle : reference_path_ptr->get_free_space_ground_lines()) {
    const std::vector<planning_math::Vec2d> &line_vec2d_points =
        obstacle->perception_points();
    MakeLinePolygons(obstacle->id(), frenet_coord, line_vec2d_points,
                     left_groundline_polygons, right_groundline_polygons);
  }
  // Step 1.2 : 处理 polygon
  for (auto &obstacle : reference_path_ptr->get_parking_space()) {
    const planning_math::Polygon2d &polygon = obstacle->perception_polygon();
    MakePolygon(obstacle->id(), frenet_coord, polygon,
                left_parking_space_polygons, right_parking_space_polygons);
  }
}

}  // namespace planning
