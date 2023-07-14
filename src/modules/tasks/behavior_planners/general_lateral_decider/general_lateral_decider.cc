#include "general_lateral_decider.h"

#include <assert.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "task_basic_types.h"

namespace planning {

using namespace planning_math;
using namespace pnc::spline;

static constexpr double kCareAreaSBuffer = 5.0;

GeneralLateralDecider::GeneralLateralDecider(
    const EgoPlanningConfigBuilder *config_builder,
    const std::shared_ptr<TaskPipelineContext> &pipeline_context)
    : Task(config_builder, pipeline_context) {
  config_ = config_builder->cast<GeneralLateralDeciderConfig>();
  name_ = "GeneralLateralDecider";
}

bool GeneralLateralDecider::InitInfo() {
  ego_frenet_state_ = reference_path_ptr_->get_frenet_ego_state();

  ego_cart_state_manager_ =
      frame_->session()->environmental_model().get_ego_state_manager();

  cruise_vel_ = frame_->session()->planning_context().v_ref_cruise();

  is_lane_change_scene_ = false;

  LatDeciderOutput &lat_decider_output = frame_->mutable_session()
                                             ->mutable_planning_context()
                                             ->mutable_lat_decider_output();
  lat_decider_output.enu_ref_path.clear();
  lat_decider_output.last_enu_ref_path.clear();
  lat_decider_output.path_bounds.clear();
  lat_decider_output.safe_bounds.clear();
  lat_decider_output.enu_ref_theta.clear();
  lat_decider_output.last_enu_ref_theta.clear();

  lat_lane_change_info_ = LatDeciderLaneChangeInfo::NONE;

  if (reference_path_ptr_ == nullptr || ego_cart_state_manager_ == nullptr) {
    // add logs
    return false;
  }
  return true;
}

bool GeneralLateralDecider::Execute(planning::framework::Frame *frame) {
  LOG_DEBUG("=======GeneralLateralDecider======= \n");
  frame_ = frame;
  if (Task::Execute(frame) == false) {
    return false;
  }

  if (!InitInfo()) {
    return false;
  };

  auto &traj_points = pipeline_context_->planning_result.traj_points;

  traj_points = pipeline_context_->coarse_planning_info.trajectory_points;

  LatDeciderOutput &lat_decider_output = frame_->mutable_session()
                                             ->mutable_planning_context()
                                             ->mutable_lat_decider_output();
  lat_decider_output.complete_follow =
      false;  // fusion is unsteady, lane keep weight need decay in end of ref
  lat_decider_output.v_cruise = cruise_vel_;
  HandleLaneChangeScene(traj_points);  // TODO:handle the lane change info;

  ConstructReferencePathPoints(traj_points);

  auto &obstacle_decisions =
      pipeline_context_->planning_info.obstacle_decisions;
  auto &map_obstacle_decisions =
      pipeline_context_->planning_info.map_obstacle_decision;

  ConstructLaneAndBoundaryBounds(map_obstacle_decisions);

  ConstructLateralObstacleDecisions(obstacle_decisions);

  std::vector<std::pair<double, double>> frenet_safe_bounds;
  std::vector<std::pair<double, double>> frenet_path_bounds;

  ExtractBoundary(map_obstacle_decisions, obstacle_decisions,
                  frenet_safe_bounds, frenet_path_bounds);

  GenerateEnuBoundaryPoints(frenet_safe_bounds, frenet_path_bounds,
                            lat_decider_output);

  GenerateEnuReferenceTheta(lat_decider_output);

  GenerateEnuReferenceTraj(lat_decider_output);

  CalcLateralBehaviorOutput();

  return true;
}

bool GeneralLateralDecider::ExecuteTest(planning::framework::Frame *frame,
                                        bool pipeline_test) {
  // pipeline test
  return true;
}

void GeneralLateralDecider::HandleLaneChangeScene(
    TrajectoryPoints &traj_points) {
  const auto &coarse_planning_info = pipeline_context_->coarse_planning_info;
  const auto &target_state = coarse_planning_info.target_state;
  const auto &ego_state =
      frame_->session()->environmental_model().get_ego_state_manager();
  LatDeciderOutput &lat_decider_output = frame_->mutable_session()
                                             ->mutable_planning_context()
                                             ->mutable_lat_decider_output();

  auto &timer = frame_->mutable_session()
                    ->mutable_planning_context()
                    ->mutable_lat_behavior_state_machine_output()
                    .lc_timer;
  bool lane_change_flag{false};

  if (target_state == ROAD_LC_LCHANGE) {
    lat_lane_change_info_ = LatDeciderLaneChangeInfo::LEFT_LANE_CHANGE;
    lane_change_flag = true;
    lat_decider_output.complete_follow = true;
  } else if (target_state == ROAD_LC_RCHANGE) {
    lat_lane_change_info_ = LatDeciderLaneChangeInfo::RIGHT_LANE_CHANGE;
    lane_change_flag = true;
    lat_decider_output.complete_follow = true;
  }

  // protect lane change duration, if total lc time > 8.0s then aggresively lane
  // change by motion planner

  if (lane_change_flag) {
    double remaining_lane_change_duration =
        config_.lane_change_duration - timer;
    auto ego_v = cruise_vel_;

    auto lat_state = ego_state->planning_init_point().lat_init_state;
    Point2D frenet_init_point;
    Point2D cart_init_point{lat_state.x(), lat_state.y()};
    if (TRANSFORM_FAILED ==
        reference_path_ptr_->get_frenet_coord()->CartCoord2FrenetCoord(
            cart_init_point, frenet_init_point)) {
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
        reference_path_ptr_->get_frenet_coord()->GetLength() - 0.5) {
      ego_v = (reference_path_ptr_->get_frenet_coord()->GetLength() -
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
    if (TRANSFORM_FAILED ==
        reference_path_ptr_->get_frenet_coord()->FrenetCoord2CartCoord(
            frenet_end_point, cart_end_point)) {
      LOG_ERROR("ERROR! Frenet Point -> Cart Point Failed!!!");
    }

    const auto lane_change_end_heading_angle =
        reference_path_ptr_->get_frenet_coord()->GetRefCurveHeading(
            lane_change_end_s);
    const auto lane_change_end_curvature =
        reference_path_ptr_->get_frenet_coord()->GetRefCurveCurvature(
            lane_change_end_s);

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
    for (size_t i = 0; i < traj_points.size(); i++) {
      if (traj_points[i].t < remaining_lane_change_duration) {
        sample_point = lane_change_quintic_path(traj_points[i].t);
        point.x = sample_point.x();
        point.y = sample_point.y();
        point.heading_angle =
            lane_change_quintic_path.heading(traj_points[i].t);

        Point2D cart_point(point.x, point.y);
        if (TRANSFORM_FAILED ==
            reference_path_ptr_->get_frenet_coord()->CartCoord2FrenetCoord(
                cart_point, frenet_point)) {
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
        point.s = std::fmin(
            s_truncation + ego_v * (i - truncation_idx) * config_.delta_t,
            reference_path_ptr_->get_frenet_coord()->GetLength());
        point.l = 0.;
        point.t = traj_points[i].t;

        frenet_point.x = point.s;
        frenet_point.y = point.l;
        Point2D cart_point;
        if (TRANSFORM_FAILED ==
            reference_path_ptr_->get_frenet_coord()->FrenetCoord2CartCoord(
                frenet_point, cart_point)) {
          LOG_ERROR("ERROR! Cart Point -> Frenet Point Failed!!!");
        }
        point.x = cart_point.x;
        point.y = cart_point.y;

        point.heading_angle =
            reference_path_ptr_->get_frenet_coord()->GetRefCurveHeading(
                point.s);
        traj_points[i] = point;
      }
    }
    return;
  } else {
    return;
  }
}

bool GeneralLateralDecider::ConstructReferencePathPoints(
    const TrajectoryPoints &traj_points) {
  ref_traj_points_.clear();
  ref_path_points_.clear();

  for (const auto &traj_point : traj_points) {
    ref_traj_points_.emplace_back(traj_point);
    ReferencePathPoint refpath_pt{};
    if (!reference_path_ptr_->get_reference_point_by_lon(traj_point.s,
                                                         refpath_pt)) {
      // add logs
      LOG_ERROR("Get reference point by lon failed!");
    }
    ref_path_points_.emplace_back(refpath_pt);
  }
  return true;
}

void GeneralLateralDecider::ConstructLaneAndBoundaryBounds(
    MapObstacleDecision &map_obstacle_decisions) {
  const auto &vehicle_param =
      frame_->session()->vehicle_config_context().get_vehicle_param();

  double left_border_distance{10.};
  double right_border_distance{10.};

  for (size_t i = 0; i < ref_traj_points_.size(); i++) {
    Bound path_bound{-10., 10.};
    Bound safe_bound{-10., 10.};
    MapObstaclePositionDecision map_obstacle_decision;

    map_obstacle_decision.tp.t = ref_traj_points_[i].t;
    map_obstacle_decision.tp.s = ref_traj_points_[i].s;
    map_obstacle_decision.tp.l = ref_traj_points_[i].l;

    double left_lane_distance = 10.;
    double right_lane_distance = 10.;
    double left_road_distance = 10.;
    double right_road_distance = 10.;

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

    map_obstacle_decision.lat_bounds.emplace_back(
        WeightedBound{safe_bound.lower, safe_bound.upper,
                      config_.kPhysicalBoundWeight, BoundInfo{i, "lane"}});
    map_obstacle_decision.lat_bounds.emplace_back(
        WeightedBound{path_bound.lower, path_bound.upper,
                      config_.kHardBoundWeight, BoundInfo{i, "road"}});

    map_obstacle_decisions.emplace_back(std::move(map_obstacle_decision));
  }
}

void GeneralLateralDecider::ConstructLateralObstacleDecisions(
    // const TrajectoryPoints &traj_points,
    ObstacleDecisions &obstacle_decisions) {
  ObstaclePotentialDecisions obstacle_potential_decisions;

  int32_t obj_cnt = 0;

  const auto &obs_vec = reference_path_ptr_->get_obstacles();
  for (auto &obstacle : obs_vec) {
    const auto &otype = obstacle->type();
    const auto ofusion_source = obstacle->obstacle()->fusion_source();
    if ((ofusion_source != OBSTACLE_SOURCE_CAMERA) &&
        (ofusion_source != OBSTACLE_SOURCE_F_RADAR_CAMERA)) {
      LOG_ERROR("The obstacle's fusion source is no camera whose id : %d \n",
                obstacle->id());
      continue;
    }
    if (otype == Common::ObjectType::OBJECT_TYPE_UNKNOWN or  // TBD: check
                                                             // obstacle type
        otype == Common::ObjectType::OBJECT_TYPE_UNKNOWN_MOVABLE or
        otype == Common::ObjectType::OBJECT_TYPE_UNKNOWN_IMMOVABLE or
        otype == Common::ObjectType::OBJECT_TYPE_VAN or
        otype == Common::ObjectType::OBJECT_TYPE_TRAILER or
        otype == Common::ObjectType::OBJECT_TYPE_TEMPORY_SIGN) {
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
}

void GeneralLateralDecider::ConstructLateralObstacleDecision(
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
      frame_->session()->vehicle_config_context().get_vehicle_param();
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
      vehicle_param.length - vehicle_param.back_edge_to_rear_axis;
  const bool init_lon_no_overlap =
      (obstacle->frenet_obstacle_boundary().s_end <
           ego_cur_s - vehicle_param.back_edge_to_rear_axis ||
       obstacle->frenet_obstacle_boundary().s_start >
           ego_cur_s + rear_axle_to_front_bumper);
  const bool init_lat_overlap = !(obstacle->frenet_obstacle_boundary().l_end <
                                      ego_cur_l - vehicle_param.width / 2 ||
                                  obstacle->frenet_obstacle_boundary().l_start >
                                      ego_cur_l + vehicle_param.width / 2);

  // Polygon2d obstacle_start_sl_polygon;
  Polygon2d obstacle_end_sl_polygon;
  // const bool ok_start = obstacle->get_polygon_at_time(  // TBD: no prediction
  //     0., reference_path_ptr_, obstacle_start_sl_polygon);
  const bool ok_end = obstacle->get_polygon_at_time(  // TBD: no prediction
      ref_traj_points_.back().t, reference_path_ptr_, obstacle_end_sl_polygon);

  double dynamic_bound_gain_vel = std::max(config_.min_gain_vel, ego_velocity);
  double dynamic_bound_slack_coefficient =
      1.0 / dynamic_bound_gain_vel / dynamic_bound_gain_vel;
  double nudge_obj_extra_buffer = 0.2;
  bool nudge_obj_flag{false};
  bool reset_conflict_decision{false};

  obstacle_decision.rel_pos_type = ObsRelPosType::UNDEFINED;

  auto care_object_t_threshold = config_.care_object_t_threshold;
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
  if (obstacle->type() == Common::ObjectType::OBJECT_TYPE_CONE ||
      obstacle->type() == Common::ObjectType::OBJECT_TYPE_BUS ||
      obstacle->type() == Common::ObjectType::OBJECT_TYPE_PEDESTRIAN ||
      obstacle->type() == Common::ObjectType::OBJECT_TYPE_TRUCK) {
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
  // Step 5) calculate safe_bound, path_bound
  for (size_t i = 0; i < ref_traj_points_.size(); i++) {
    auto &traj_point = ref_traj_points_[i];
    const auto &t = traj_point.t;
    if (t > care_object_t_threshold) {
      continue;
    }

    const auto &ego_s = traj_point.s;
    const auto &ego_l = traj_point.l;

    // double kCareAreaSBuffer =
    //     std::max(std::min(5.0, 5.0 - 0.5 * (10.0 - ego_velocity)),
    //              0.0);  // @cai: currently use fixed distance

    auto care_area_s_start = ego_s - vehicle_param.back_edge_to_rear_axis;
    auto care_area_s_end = ego_s + rear_axle_to_front_bumper + kCareAreaSBuffer;
    auto care_area_center =
        Vec2d((care_area_s_start + care_area_s_end) * 0.5, ego_l);
    auto care_area_length = care_area_s_end - care_area_s_start;
    auto care_polygon =  // @cai: consider the heading
        Polygon2d(Box2d(care_area_center, 0, care_area_length, l_care_width));

    // auto obstacle_pos_time = 0.;  // TBD: wait for predication
    //  if (obstacle->type() == ObjectType::PEDESTRIAN ||
    //      obstacle->type() == ObjectType::OFO) {
    //    obstacle_pos_time = 0.;
    //  }

    Polygon2d obstacle_sl_polygon;
    auto ok = obstacle->get_polygon_at_time(
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
    // frenet_length is larger than raw length when obj is in lagre curvature
    // road

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

    std::pair<double, double> s_side_range{obstacle_sl_polygon.min_x(),
                                           obstacle_sl_polygon.max_x()};
    std::pair<double, double> ego_side_range{
        ego_s - vehicle_param.back_edge_to_rear_axis,
        ego_s + rear_axle_to_front_bumper + 2.};
    bool b_overlap_side = std::max(s_side_range.first, ego_side_range.first) <
                          std::min(s_side_range.second, ego_side_range.second);

    if (CheckObstacleCrossingCondition(obstacle, is_cross_obj)) {
      // TBD:add logs
    }

    const auto &nudge_reference_center_l = ego_l;
    constexpr double kMaxAvoidEdgeL{2.0};  // m

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
                        -kMaxAvoidEdgeL) or
          b_overlap_side) {
        lat_decision = LatObstacleDecisionType::RIGHT;
        lon_decision = LonObstacleDecisionType::IGNORE;
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
                       kMaxAvoidEdgeL) or
          b_overlap_side) {
        lat_decision = LatObstacleDecisionType::LEFT;
        lon_decision = LonObstacleDecisionType::IGNORE;
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

    } else if (lat_decision == LatObstacleDecisionType::RIGHT) {
      path_bound.upper =
          care_overlap_polygon.min_y() - collision_center_distance;
      safe_bound.upper = care_overlap_polygon.min_y() - safe_center_distance -
                         safe_extra_distance;

    } else {
      assert(lon_decision != LonObstacleDecisionType::IGNORE);
    }

    // refine safe bound a.c. model traj and path bound
    if (lat_decision == LatObstacleDecisionType::LEFT) {
      if (ego_l < safe_bound.lower) {
        if (ego_l > path_bound.lower + config_.min_obstacle_avoid_distance) {
          double origin_avoid_buffer = safe_bound.lower - path_bound.lower;
          double adjust_avoid_buffer = planning::planning_math::lerp(
              config_.min_obstacle_avoid_distance, 0.0, origin_avoid_buffer,
              1.0, (ego_l - path_bound.lower) / origin_avoid_buffer);
          safe_bound.lower =
              path_bound.lower + std::max(config_.min_obstacle_avoid_distance,
                                          adjust_avoid_buffer);

        } else {
          safe_bound.lower =
              path_bound.lower + config_.min_obstacle_avoid_distance;
        }
      }
      if (ego_l < safe_bound.lower) {
        double converge_coeff = std::min(
            1.0, std::pow(config_.lateral_bound_converge_speed * t, 2));
        safe_bound.lower = planning::planning_math::lerp(
            ego_l, 0, safe_bound.lower, 1.0, converge_coeff);
        safe_bound.lower =
            std::max(safe_bound.lower,
                     path_bound.lower + config_.min_obstacle_avoid_distance);
      }
    } else if (lat_decision == LatObstacleDecisionType::RIGHT) {
      if (ego_l > safe_bound.upper) {
        if (ego_l < path_bound.upper - config_.min_obstacle_avoid_distance) {
          double origin_avoid_buffer = path_bound.upper - safe_bound.upper;
          double adjust_avoid_buffer = planning::planning_math::lerp(
              config_.min_obstacle_avoid_distance, 0.0, origin_avoid_buffer,
              1.0, (path_bound.upper - ego_l) / origin_avoid_buffer);
          safe_bound.upper =
              path_bound.upper - std::max(config_.min_obstacle_avoid_distance,
                                          adjust_avoid_buffer);

        } else {
          safe_bound.upper =
              path_bound.upper - config_.min_obstacle_avoid_distance;
        }
      }
      if (ego_l > safe_bound.upper) {
        double converge_coeff = std::min(
            1.0, std::pow(config_.lateral_bound_converge_speed * t, 2));
        safe_bound.upper = planning::planning_math::lerp(
            ego_l, 0.0, safe_bound.upper, 1.0, converge_coeff);
        safe_bound.upper =
            std::min(safe_bound.upper,
                     path_bound.upper - config_.min_obstacle_avoid_distance);
      }
    }

    has_lat_decision =
        has_lat_decision || lat_decision != LatObstacleDecisionType::IGNORE;
    has_lon_decision =
        has_lon_decision || lon_decision != LonObstacleDecisionType::IGNORE;

    ObstaclePositionDecision position_decision;
    position_decision.tp.t = t;
    position_decision.tp.s = ego_s;
    position_decision.tp.l = ego_l;

    position_decision.lat_bounds.push_back(WeightedBound{
        path_bound.lower, path_bound.upper, config_.kHardBoundWeight, {}});
    position_decision.lat_bounds.push_back(WeightedBound{
        safe_bound.lower,
        safe_bound.upper,
        config_.dynamic_bound_slack_coefficient * config_.kPhysicalBoundWeight,
        {}});
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

bool GeneralLateralDecider::CheckObstacleNudgeCondition(

    const std::shared_ptr<FrenetObstacle> &obstacle) {
  // nudge logic
  return false;
}

bool GeneralLateralDecider::CheckObstacleCrossingCondition(
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
    const MapObstacleDecision &map_obstacle_decision,
    const ObstacleDecisions &obstacle_decisions,
    std::vector<std::pair<double, double>> &frenet_safe_bounds,
    std::vector<std::pair<double, double>> &frenet_path_bounds) {
  assert(map_obstacle_decision.size() == ref_traj_points_.size());

  // auto &path_bounds = lat_decider_output.path_bounds;
  // auto &safe_bounds = lat_decider_output.path_bounds;

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
  }

  for (auto &obstacle_decision : obstacle_decisions) {
    for (auto &obstacle_position_decision :
         obstacle_decision.second.position_decisions) {
      for (size_t i = 0; i < ref_traj_points_.size(); i++) {
        if (std::fabs(obstacle_position_decision.tp.t - ref_traj_points_[i].t <
                      1e-2)) {
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
    for (auto &bound : bounds) {
      if (bound.upper < tmp_bound.second) {
        tmp_bound.second = bound.upper;
      }
      if (bound.lower > tmp_bound.first) {
        tmp_bound.first = bound.lower;
      }
    }
    frenet_path_bounds.emplace_back(tmp_bound);
  }

  for (auto &bounds : safe_bounds) {
    std::pair<double, double> tmp_bound{-10., 10.};  // <lower ,upper >
    for (auto &bound : bounds) {
      if (bound.upper < tmp_bound.second) {
        tmp_bound.second = bound.upper;
      }
      if (bound.lower > tmp_bound.first) {
        tmp_bound.first = bound.lower;
      }
    }
    frenet_safe_bounds.emplace_back(tmp_bound);
  }

  assert(frenet_path_bounds.size() == ref_traj_points_.size());
  assert(frenet_safe_bounds.size() == ref_traj_points_.size());
}

void GeneralLateralDecider::GenerateEnuBoundaryPoints(
    const std::vector<std::pair<double, double>> &frenet_safe_bounds,
    const std::vector<std::pair<double, double>> &frenet_path_bounds,
    LatDeciderOutput &lat_decider_output) {
  auto &safe_bounds_output = lat_decider_output.safe_bounds;
  auto &path_bounds_output = lat_decider_output.path_bounds;

  const std::shared_ptr<FrenetCoordinateSystem> frenet_coord =
      reference_path_ptr_->get_frenet_coord();
  Point2D tmp_safe_lower_point;
  Point2D tmp_safe_upper_point;
  Point2D tmp_path_lower_point;
  Point2D tmp_path_upper_point;
  for (size_t i = 0; i < ref_traj_points_.size(); ++i) {
    if (frenet_coord->FrenetCoord2CartCoord(
            Point2D(ref_traj_points_[i].s, frenet_safe_bounds[i].first),
            tmp_safe_lower_point) !=
        TRANSFORM_STATUS::TRANSFORM_SUCCESS)  // safe lower
    {
      // TODO: add logs
    }

    if (frenet_coord->FrenetCoord2CartCoord(
            Point2D(ref_traj_points_[i].s, frenet_safe_bounds[i].second),
            tmp_safe_upper_point) !=
        TRANSFORM_STATUS::TRANSFORM_SUCCESS)  // safe upper
    {
      // TODO: add logs
    }
    safe_bounds_output.emplace_back(std::pair<Point2D, Point2D>(
        tmp_safe_lower_point, tmp_safe_upper_point));

    if (frenet_coord->FrenetCoord2CartCoord(
            Point2D(ref_traj_points_[i].s, frenet_path_bounds[i].first),
            tmp_path_lower_point) !=
        TRANSFORM_STATUS::TRANSFORM_SUCCESS)  // path lower
    {
      // TODO: add logs
    }

    if (frenet_coord->FrenetCoord2CartCoord(
            Point2D(ref_traj_points_[i].s, frenet_path_bounds[i].second),
            tmp_path_upper_point) !=
        TRANSFORM_STATUS::TRANSFORM_SUCCESS)  // path upper
    {
      // TODO: add logs
    }

    path_bounds_output.emplace_back(std::pair<Point2D, Point2D>(
        tmp_path_lower_point, tmp_path_upper_point));
  }
};

void GeneralLateralDecider::GenerateEnuReferenceTraj(
    LatDeciderOutput &lat_decider_output) {
  auto &enu_ref_path = lat_decider_output.enu_ref_path;
  enu_ref_path.resize(ref_traj_points_.size());

  for (size_t i = 0; i < ref_traj_points_.size(); i++) {
    enu_ref_path[i].first = ref_traj_points_[i].x;
    enu_ref_path[i].second = ref_traj_points_[i].y;
  }

  const auto &s_start = ref_traj_points_.front().s;
  const auto &s_end = ref_traj_points_.back().s;

  lat_decider_output.v_cruise =
      (s_end - s_start) / (config_.delta_t * (ref_traj_points_.size() - 1));
}

void GeneralLateralDecider::GenerateEnuReferenceTheta(
    LatDeciderOutput &lat_decider_output) {
  auto &enu_ref_theta = lat_decider_output.enu_ref_theta;

  for (size_t i = 0; i < ref_path_points_.size(); i++) {
    enu_ref_theta.emplace_back(ref_traj_points_[i].heading_angle);
  }
}

void GeneralLateralDecider::SampleRoadDistanceInfo(
    const double &s_target, ReferencePathPoint &sample_path_point) {
  const double cut_length = config_.sample_step;
  ReferencePathPoint refpath_pt{};

  for (double s = s_target - vehicle_param_.back_edge_to_rear_axis;
       s < s_target + vehicle_param_.rear_axis_to_front_edge +
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
  auto &coarse_planning_info = pipeline_context_->coarse_planning_info;
  auto &lateral_output = frame_->mutable_session()
                             ->mutable_planning_context()
                             ->mutable_lateral_behavior_planner_output();
  auto &state_machine_output =
      frame_->mutable_session()
          ->mutable_planning_context()
          ->mutable_lat_behavior_state_machine_output();

  const std::shared_ptr<VirtualLane> flane =
      frame_->session()
          ->environmental_model()
          .get_virtual_lane_manager()
          ->get_lane_with_virtual_id(coarse_planning_info.target_lane_id);
  const std::shared_ptr<VirtualLaneManager> virtual_lane_manager =
      frame_->session()->environmental_model().get_virtual_lane_manager();

  // path points
  std::vector<PathPoint> path_points;
  if (flane != nullptr) {
    auto &ref_path = flane->get_reference_path();
    for (auto &ref_point : ref_path->get_points()) {
      path_points.emplace_back(ref_point.path_point);
    }
  }
  // scenario, left_faster, right_is_faster
  lateral_output.scenario = state_machine_output.scenario;
  lateral_output.left_faster = state_machine_output.left_is_faster;
  lateral_output.right_faster = state_machine_output.right_is_faster;
  // flane width
  lateral_output.flane_width = flane->width();
  // lat offset, attention!
  lateral_output.lat_offset = 0.0;
  // borrow_bicycle_lane
  bool isRedLightStop = false;  // attention again!!!
  TrackedObject *lead_one = frame_->mutable_session()
                                ->mutable_environmental_model()
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
  lateral_output.isOnHighway =
      frame_->session()->environmental_model().is_on_highway();

  // d_poly ,c_poly
  auto &d_poly = lateral_output.d_poly;
  auto &c_poly = lateral_output.c_poly;

  d_poly.resize(flane->get_center_line().poly_coefficient_car().size());
  c_poly.resize(flane->get_center_line().poly_coefficient_car().size());

  std::reverse_copy(flane->get_center_line().poly_coefficient_car().begin(),
                    flane->get_center_line().poly_coefficient_car().end(),
                    d_poly.begin());
  std::reverse_copy(flane->get_center_line().poly_coefficient_car().begin(),
                    flane->get_center_line().poly_coefficient_car().end(),
                    c_poly.begin());
}

}  // namespace planning
