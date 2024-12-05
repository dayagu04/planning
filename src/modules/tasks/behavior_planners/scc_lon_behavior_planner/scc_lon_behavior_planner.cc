#include "scc_lon_behavior_planner.h"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <vector>

#include "config/basic_type.h"
#include "debug_info_log.h"
#include "ifly_time.h"
#include "planning_context.h"
#include "src/modules/common/trajectory1d/variable_coordinate_time_optimal_trajectory.h"
#include "task_basic_types.h"
#include "trajectory1d/trajectory1d.h"

namespace {

// far slow car jlt
constexpr double check_time = 1.6;
constexpr double kMinFarTimeGap = 1.8;
constexpr double kDefaultFollowMinDist = 3.0;
constexpr double kPreviewTime = 0.5;
constexpr double kSpeedBuffer = 5.0;
constexpr double kEgoSpeedThreshold = 50.0 / 3.6;
constexpr double kFarDistFollowTimeGap = 1.8;
constexpr double kNearDistFollowTimeGap = 1.2;
constexpr double kFarDistanceThreshold = 20.0;
constexpr double kNearDistanceThreshold = 10.0;
constexpr double kFarPreviewTime = 6.0;
constexpr double kNearPreviewTime = 2.0;
constexpr double kSpeedLower = 6.0;
constexpr double kSpeedUpper = 20.0;
constexpr double kAccMinLower = -0.7;
constexpr double kAccMinUpper = -0.3;
constexpr double kAccMax = 1.5;
constexpr double kJerkMin = -2.0;
constexpr double kJerkMax = 0.5;
constexpr double kPositionPrecision = 0.3;
// TODO: 后续取参考线的长度为s bound upper
constexpr double kSUpperBound = 200.0;
constexpr double kVelBoundCoeff = 1.1;
}  // namespace
namespace planning {

SccLonBehaviorPlanner::SccLonBehaviorPlanner(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<SccLonBehaviorPlannerConfig>();
  name_ = "SccLonBehaviorPlanner";

  Init();
}

bool SccLonBehaviorPlanner::Execute() {
  LOG_DEBUG("=======SccLonBehaviorPlanner======= \n");

  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }

  if (Calculate()) {
    return true;
  } else {
    return false;
  }
}

void SccLonBehaviorPlanner::Init() {
  lon_behav_plan_input_ = std::make_shared<common::RealTimeLonBehaviorInput>();
  st_graph_ = std::make_shared<scc::StGraphGenerator>(config_, session_);
  sv_graph_ = std::make_shared<scc::SvGraphGenerator>(config_);
}

bool SccLonBehaviorPlanner::Calculate() {
  LOG_DEBUG("=======SccLonBehaviorPlanner======= \n");
  auto start_time = IflyTime::Now_ms();

  ConstructLonBehavInput();
  auto construct_input_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("RealTimeLonBehaviorConstructInputCostTime",
                   construct_input_time - start_time);
  Update();
  SaveToSession();
  SaveToDebugInfo();

  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("SccLonBehaviorCostTime", end_time - start_time);
  return true;
}

void SccLonBehaviorPlanner::SetConfig(
    planning::common::RealTimeLonBehaviorTunedParams &tuned_params) {
  config_.safe_distance_base = tuned_params.safe_distance_base();
  config_.safe_distance_ttc = tuned_params.safe_distance_ttc();
  config_.t_actuator_delay = tuned_params.t_actuator_delay();
  config_.lane_keep_cutinp_threshold =
      tuned_params.lane_keep_cutinp_threshold();
  config_.lane_change_cutinp_threshold =
      tuned_params.lane_change_cutinp_threshold();
  config_.corridor_width = tuned_params.corridor_width();
  config_.preview_x = tuned_params.preview_x();
  config_.dis_zero_speed = tuned_params.dis_zero_speed();
  config_.dis_zero_speed_accident = tuned_params.dis_zero_speed_accident();
  config_.ttc_brake_hysteresis = tuned_params.ttc_brake_hysteresis();
  config_.t_curv = tuned_params.t_curv();
  config_.dis_curv = tuned_params.dis_curv();
  config_.velocity_upper_bound = tuned_params.velocity_upper_bound();
  config_.v_start = tuned_params.v_start();
  config_.distance_stop = tuned_params.distance_stop();
  config_.distance_start = tuned_params.distance_start();

  st_graph_->SetConfig(tuned_params);
  sv_graph_->SetConfig(tuned_params);
}

bool SccLonBehaviorPlanner::JudgeCurvBySDMap() {
  if (!config_.enable_sdmap_curv_v_adjust) {
    return true;
  }
  if (!session_->environmental_model().get_route_info()->get_sdmap_valid()) {
    std::cout << "sd_map is invalid!!!" << std::endl;
    return true;
  }
  ad_common::math::Vec2d current_point;
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto &pose = ego_state->location_enu();
  std::cout << "ego_pose_x_:" << pose.position.x
            << "ego_pose_y_:" << pose.position.y << std::endl;
  current_point.set_x(pose.position.x);
  current_point.set_y(pose.position.y);
  const auto &sd_map = 
      session_->environmental_model().get_route_info()->get_sd_map();
  double nearest_s = 0;
  double nearest_l = 0;
  // const double max_search_length = 7000.0;  // 搜索7km范围内得地图信息
  const double search_distance = 50.0;
  const double max_heading_diff = PI / 4;
  const double ego_heading_angle = ego_state->heading_angle();
  const auto current_segment = sd_map.GetNearestRoadWithHeading(
      current_point, search_distance, ego_heading_angle, max_heading_diff,
      nearest_s, nearest_l);
  if (!current_segment) {
    return true;  // 返回true,代表判断出弯道，不加速，有异常就不加速
  }
  std::vector<std::pair<double, double>> curv_list;
  curv_list = sd_map.GetCurvatureList(current_segment->id(), nearest_s,
                                      config_.search_sdmap_curv_dis);
  double min_curv_radius = 10000.0;
  bool has_curv = true;
  for (int i = 0; i < curv_list.size(); i++) {
    double one_curv_radius = 1.0 / (std::abs(curv_list[i].second));
    if (one_curv_radius < min_curv_radius) {
      min_curv_radius = one_curv_radius;
    }
  }
  if (min_curv_radius > config_.sdmap_curv_thred) {
    has_curv = false;
  }
  JSON_DEBUG_VALUE("sdmap_min_curv_radius", min_curv_radius)
  return has_curv;
}

void SccLonBehaviorPlanner::ConstructLonBehavInput() {
  const auto &environmental_model = session_->environmental_model();
  const bool dbw_status = environmental_model.GetVehicleDbwStatus();
  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  const auto lateral_obstacles = environmental_model.get_lateral_obstacle();
  const auto virtual_lane_manager =
      environmental_model.get_virtual_lane_manager();
  const auto lane_tracks_mgr = environmental_model.get_lane_tracks_manager();
  const auto &lateral_behavior_planner_output =
      session_->planning_context().lateral_behavior_planner_output();
  const auto &lateral_motion_planner_output =
      session_->planning_context().motion_planner_output();
  const auto start_stop_state_info =
      session_->planning_context().start_stop_result();
  auto &lon_decision_info =
      session_->mutable_planning_context()->mutable_lon_decision_result();
  auto &function_info = session_->environmental_model().function_info();
  const auto &planning_init_point = ego_state_mgr->planning_init_point();

  const auto &lane_status = session_->mutable_planning_context()->lane_status();
  const auto &agent_manager =
      session_->environmental_model().get_agent_manager();

  // 0. set dbw (Drive-by-Wire)
  lon_behav_plan_input_->set_dbw_status(dbw_status);
  // 1. set ego_info & lon_decision_info, ego_info needs to be recieved from
  // init point rather than real-time vehicle state
  auto ego_info = lon_behav_plan_input_->mutable_ego_info();
  ego_info->set_ego_v(ego_state_mgr->ego_v());
  ego_info->set_ego_cruise(std::max(ego_state_mgr->ego_v_cruise(), 0.0));
  ego_info->set_ego_acc(ego_state_mgr->ego_acc());
  ego_info->set_ego_steer_angle(ego_state_mgr->ego_steer_angle());
  ego_info->set_ego_pose_x(ego_state_mgr->ego_pose().x);
  ego_info->set_ego_pose_y(ego_state_mgr->ego_pose().y);
  lon_init_state_ = {0, planning_init_point.lon_init_state.v(),
                     planning_init_point.lon_init_state.a()};
  auto lon_decision_info_input =
      lon_behav_plan_input_->mutable_lon_decision_info();
  lon_decision_info_input->CopyFrom(lon_decision_info);

  // 2. set lat_output
  auto lat_output = lon_behav_plan_input_->mutable_lat_output();
  lat_output->set_lc_request(lateral_behavior_planner_output.lc_request);
  lat_output->set_lc_status(lateral_behavior_planner_output.lc_status);
  lat_output->set_close_to_accident(
      lateral_behavior_planner_output.close_to_accident);
  lat_output->clear_d_poly_vec();

  for (auto coeff : lateral_behavior_planner_output.d_poly) {
    lat_output->add_d_poly_vec(coeff);
  }

  const auto &s_lat_vec = lateral_motion_planner_output.s_lat_vec;
  lat_output->clear_spline_x_vec();
  lat_output->clear_spline_y_vec();
  for (int i = 1; i <= config_.lon_num_step + 1; ++i) {
    lat_output->add_spline_x_vec(
        lateral_motion_planner_output.x_s_spline(s_lat_vec[i]));
    lat_output->add_spline_y_vec(
        lateral_motion_planner_output.y_s_spline(s_lat_vec[i]));
  }

  // 3. set lateral obstacles
  auto lead_one =
      lon_behav_plan_input_->mutable_lat_obs_info()->mutable_lead_one();
  if (lateral_obstacles->leadone() != nullptr) {
    lead_one->set_track_id(lateral_obstacles->leadone()->track_id);
    lead_one->set_type(lateral_obstacles->leadone()->type);
    lead_one->set_fusion_source(lateral_obstacles->leadone()->fusion_source);
    lead_one->set_v_lead(lateral_obstacles->leadone()->v_lead);
    lead_one->set_v_rel(lateral_obstacles->leadone()->v_rel);
    lead_one->set_a_lead_k(lateral_obstacles->leadone()->a_lead_k);
    lead_one->set_d_rel(lateral_obstacles->leadone()->d_rel);
    lead_one->set_d_path(lateral_obstacles->leadone()->d_path);
    lead_one->set_d_path_self(lateral_obstacles->leadone()->d_path_self);
    lead_one->set_v_lat(lateral_obstacles->leadone()->v_lat);
    lead_one->set_is_accident_car(
        lateral_obstacles->leadone()->is_accident_car);
    lead_one->set_is_lead(lateral_obstacles->leadone()->is_lead);
    lead_one->set_is_temp_lead(lateral_obstacles->leadone()->is_temp_lead);
    lead_one->set_cutinp(lateral_obstacles->leadone()->cutinp);
    lead_one->set_is_vru(lateral_obstacles->leadone()->is_VRU);
    lead_one->set_is_car(lateral_obstacles->leadone()->is_car);
  } else {
    lead_one->set_track_id(-1);
    lead_one->set_type(0);
  }

  auto lead_two =
      lon_behav_plan_input_->mutable_lat_obs_info()->mutable_lead_two();
  if (lateral_obstacles->leadtwo() != nullptr) {
    auto lead_two =
        lon_behav_plan_input_->mutable_lat_obs_info()->mutable_lead_two();
    lead_two->set_track_id(lateral_obstacles->leadtwo()->track_id);
    lon_behav_plan_input_->mutable_lat_obs_info()->mutable_lead_two()->set_type(
        lateral_obstacles->leadtwo()->type);
    lead_two->set_fusion_source(lateral_obstacles->leadtwo()->fusion_source);
    lead_two->set_v_lead(lateral_obstacles->leadtwo()->v_lead);
    lead_two->set_v_rel(lateral_obstacles->leadtwo()->v_rel);
    lead_two->set_a_lead_k(lateral_obstacles->leadtwo()->a_lead_k);
    lead_two->set_d_rel(lateral_obstacles->leadtwo()->d_rel);
    lead_two->set_d_path(lateral_obstacles->leadtwo()->d_path);
    lead_two->set_d_path_self(lateral_obstacles->leadtwo()->d_path_self);
    lead_two->set_v_lat(lateral_obstacles->leadtwo()->v_lat);
    lead_two->set_is_accident_car(
        lateral_obstacles->leadtwo()->is_accident_car);
    lead_two->set_is_lead(lateral_obstacles->leadtwo()->is_lead);
    lead_two->set_is_temp_lead(lateral_obstacles->leadtwo()->is_temp_lead);
    lead_two->set_cutinp(lateral_obstacles->leadtwo()->cutinp);
    lead_two->set_is_vru(lateral_obstacles->leadtwo()->is_VRU);
    lead_two->set_is_car(lateral_obstacles->leadtwo()->is_car);
  } else {
    lead_two->set_track_id(-1);
    lead_two->set_type(0);
  }

  auto temp_lead_one =
      lon_behav_plan_input_->mutable_lat_obs_info()->mutable_temp_lead_one();
  if (lateral_obstacles->tleadone() != nullptr) {
    temp_lead_one->set_track_id(lateral_obstacles->tleadone()->track_id);
    temp_lead_one->set_type(lateral_obstacles->tleadone()->type);
    temp_lead_one->set_fusion_source(
        lateral_obstacles->tleadone()->fusion_source);
    temp_lead_one->set_v_lead(lateral_obstacles->tleadone()->v_lead);
    temp_lead_one->set_v_rel(lateral_obstacles->tleadone()->v_rel);
    temp_lead_one->set_a_lead_k(lateral_obstacles->tleadone()->a_lead_k);
    temp_lead_one->set_d_rel(lateral_obstacles->tleadone()->d_rel);
    temp_lead_one->set_d_path(lateral_obstacles->tleadone()->d_path);
    temp_lead_one->set_d_path_self(lateral_obstacles->tleadone()->d_path_self);
    temp_lead_one->set_v_lat(lateral_obstacles->tleadone()->v_lat);
    temp_lead_one->set_is_accident_car(
        lateral_obstacles->tleadone()->is_accident_car);
    temp_lead_one->set_is_lead(lateral_obstacles->tleadone()->is_lead);
    temp_lead_one->set_is_temp_lead(
        lateral_obstacles->tleadone()->is_temp_lead);
    temp_lead_one->set_cutinp(lateral_obstacles->tleadone()->cutinp);
    temp_lead_one->set_is_vru(lateral_obstacles->tleadone()->is_VRU);
    temp_lead_one->set_is_car(lateral_obstacles->tleadone()->is_car);
  } else {
    temp_lead_one->set_track_id(-1);
    temp_lead_one->set_type(0);
  }

  auto temp_lead_two =
      lon_behav_plan_input_->mutable_lat_obs_info()->mutable_temp_lead_two();
  if (lateral_obstacles->tleadtwo() != nullptr) {
    temp_lead_two->set_track_id(lateral_obstacles->tleadtwo()->track_id);
    temp_lead_two->set_type(lateral_obstacles->tleadtwo()->type);
    temp_lead_two->set_fusion_source(
        lateral_obstacles->tleadtwo()->fusion_source);
    temp_lead_two->set_v_lead(lateral_obstacles->tleadtwo()->v_lead);
    temp_lead_two->set_v_rel(lateral_obstacles->tleadtwo()->v_rel);
    temp_lead_two->set_a_lead_k(lateral_obstacles->tleadtwo()->a_lead_k);
    temp_lead_two->set_d_rel(lateral_obstacles->tleadtwo()->d_rel);
    temp_lead_two->set_d_path(lateral_obstacles->tleadtwo()->d_path);
    temp_lead_two->set_d_path_self(lateral_obstacles->tleadtwo()->d_path_self);
    temp_lead_two->set_v_lat(lateral_obstacles->tleadtwo()->v_lat);
    temp_lead_two->set_is_accident_car(
        lateral_obstacles->tleadtwo()->is_accident_car);
    temp_lead_two->set_is_lead(lateral_obstacles->tleadtwo()->is_lead);
    temp_lead_two->set_is_temp_lead(
        lateral_obstacles->tleadtwo()->is_temp_lead);
    temp_lead_two->set_cutinp(lateral_obstacles->tleadtwo()->cutinp);
    temp_lead_two->set_is_vru(lateral_obstacles->tleadtwo()->is_VRU);
    temp_lead_two->set_is_car(lateral_obstacles->tleadtwo()->is_car);
  } else {
    temp_lead_two->set_track_id(-1);
    temp_lead_two->set_type(0);
  }

  auto lat_obs_info = lon_behav_plan_input_->mutable_lat_obs_info();
  lat_obs_info->mutable_front_tracks()->Clear();
  lat_obs_info->mutable_front_tracks()->Reserve(
      lateral_obstacles->front_tracks().size());
  for (auto &track : lateral_obstacles->front_tracks()) {
    planning::common::TrackedObjectInfo *one_obs =
        lat_obs_info->add_front_tracks();
    one_obs->set_track_id(track.track_id);
    one_obs->set_type(track.type);
    one_obs->set_fusion_source(track.fusion_source);
    one_obs->set_v_lead(track.v_lead);
    one_obs->set_v_rel(track.v_rel);
    one_obs->set_a_lead_k(track.a_lead_k);
    one_obs->set_d_rel(track.d_rel);
    one_obs->set_d_path(track.d_path);
    one_obs->set_d_path_self(track.d_path_self);
    one_obs->set_v_lat(track.v_lat);
    one_obs->set_is_accident_car(track.is_accident_car);
    one_obs->set_is_lead(track.is_lead);
    one_obs->set_is_temp_lead(track.is_temp_lead);
    one_obs->set_cutinp(track.cutinp);
    one_obs->set_y_min(track.y_min);
    one_obs->set_y_x0(track.y_x0);
    one_obs->set_location_head(track.location_head);
    one_obs->set_location_tail(track.location_tail);
    one_obs->set_vy_rel(track.vy_rel);
    one_obs->set_y_rel(track.y_rel);
    const auto *agent = agent_manager->GetAgent(track.track_id);
    if (agent != nullptr) {
      one_obs->set_is_new_cutin(agent->is_cutin());
    } else {
      one_obs->set_is_new_cutin(false);
    }
  }

  lat_obs_info->mutable_side_tracks()->Clear();
  lat_obs_info->mutable_side_tracks()->Reserve(
      lateral_obstacles->side_tracks().size());
  for (auto &track : lateral_obstacles->side_tracks()) {
    planning::common::TrackedObjectInfo *one_obs =
        lat_obs_info->add_side_tracks();
    one_obs->set_track_id(track.track_id);
    one_obs->set_type(track.type);
    one_obs->set_fusion_source(track.fusion_source);
    one_obs->set_v_lead(track.v_lead);
    one_obs->set_v_rel(track.v_rel);
    one_obs->set_a_lead_k(track.a_lead_k);
    one_obs->set_d_rel(track.d_rel);
    one_obs->set_d_path(track.d_path);
    one_obs->set_d_path_self(track.d_path_self);
    one_obs->set_v_lat(track.v_lat);
    one_obs->set_is_accident_car(track.is_accident_car);
    one_obs->set_is_lead(track.is_lead);
    one_obs->set_is_temp_lead(track.is_temp_lead);
    one_obs->set_cutinp(track.cutinp);
    one_obs->set_y_min(track.y_min);
    one_obs->set_y_x0(track.y_x0);
    one_obs->set_location_head(track.location_head);
    one_obs->set_location_tail(track.location_tail);
    one_obs->set_vy_rel(track.vy_rel);
    one_obs->set_y_rel(track.y_rel);
  }

  // 4. set lane change info
  const auto &lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto current_lane = virtual_lane_manager->get_current_lane();
  int lc_map_decision = virtual_lane_manager->lc_map_decision(current_lane);
  double lc_end_dis = 0;
  auto lane_change_info = lon_behav_plan_input_->mutable_lc_info();
  lane_change_info->set_has_target_lane(
      lane_change_decider_output.has_target_lane);
  lane_change_info->set_tlane_virtual_id(
      lane_change_decider_output.target_lane_virtual_id);
  lane_change_info->set_v_limit(ego_state_mgr->ego_v_cruise());
  lane_change_info->set_lc_end_dis(lc_end_dis);
  lane_change_info->set_lc_map_decision(lc_map_decision);
  lane_change_info->set_current_lane_type(current_lane->get_lane_type());
  lane_change_info->set_target_gap_obs_first(
      lane_status.change_lane.target_gap_obs.first);
  lane_change_info->set_target_gap_obs_second(
      lane_status.change_lane.target_gap_obs.second);
  lane_change_info->clear_lc_cars();

  std::vector<TrackedObject> *front_target_tracks =
      lane_tracks_mgr->get_lane_tracks(
          lane_change_decider_output.target_lane_virtual_id, FRONT_TRACK);
  if (front_target_tracks != nullptr) {
    for (auto &track : *front_target_tracks) {
      // ignore obj without camera source
      if ((track.fusion_source & OBSTACLE_SOURCE_CAMERA) == 0) {
        continue;
      };
      planning::common::TrackedObjectInfo *one_obs =
          lane_change_info->add_lc_cars();
      one_obs->set_track_id(track.track_id);
      one_obs->set_type(track.type);
      one_obs->set_fusion_source(track.fusion_source);
      one_obs->set_v_lead(track.v_lead);
      one_obs->set_v_rel(track.v_rel);
      one_obs->set_a_lead_k(track.a_lead_k);
      one_obs->set_d_rel(track.d_rel);
      one_obs->set_d_path(track.d_path);
      one_obs->set_d_path_self(track.d_path_self);
      one_obs->set_v_lat(track.v_lat);
      one_obs->set_is_accident_car(track.is_accident_car);
      one_obs->set_is_lead(track.is_lead);
      one_obs->set_is_temp_lead(track.is_temp_lead);
      one_obs->set_cutinp(track.cutinp);
    }
  }
  std::vector<TrackedObject> *side_target_tracks =
      lane_tracks_mgr->get_lane_tracks(
          lane_change_decider_output.target_lane_virtual_id, SIDE_TRACK);
  if (side_target_tracks != nullptr) {
    for (auto &track : *side_target_tracks) {
      // ignore obj without camera source
      if ((track.fusion_source & OBSTACLE_SOURCE_CAMERA) == 0) {
        continue;
      };
      planning::common::TrackedObjectInfo *one_obs =
          lane_change_info->add_lc_cars();
      one_obs->set_track_id(track.track_id);
      one_obs->set_type(track.type);
      one_obs->set_fusion_source(track.fusion_source);
      one_obs->set_v_lead(track.v_lead);
      one_obs->set_v_rel(track.v_rel);
      one_obs->set_a_lead_k(track.a_lead_k);
      one_obs->set_d_rel(track.d_rel);
      one_obs->set_d_path(track.d_path);
      one_obs->set_d_path_self(track.d_path_self);
      one_obs->set_v_lat(track.v_lat);
      one_obs->set_is_accident_car(track.is_accident_car);
      one_obs->set_is_lead(track.is_lead);
      one_obs->set_is_temp_lead(track.is_temp_lead);
      one_obs->set_cutinp(track.cutinp);
    }
  }

  // 5. set fix lane info
  // TBD: 格式统一用proto后直接copy from
  const auto fix_lane = virtual_lane_manager->get_lane_with_virtual_id(
      lane_change_decider_output.fix_lane_virtual_id);
  const std::vector<ReferencePathPoint> fix_ref_points =
      fix_lane->get_reference_path()->get_points();
  auto ref_path_points = lon_behav_plan_input_->mutable_ref_path_points();
  ref_path_points->Clear();
  ref_path_points->Reserve(fix_ref_points.size());
  for (auto &point : fix_ref_points) {
    common::ReferencePathPoint *ref_path_point = ref_path_points->Add();
    auto *path_point = ref_path_point->mutable_path_point();
    path_point->set_x(point.path_point.x);
    path_point->set_y(point.path_point.y);
    path_point->set_z(point.path_point.z);
    path_point->set_theta(point.path_point.theta);
    path_point->set_kappa(point.path_point.kappa);
    path_point->set_s(point.path_point.s);
    path_point->set_dkappa(point.path_point.dkappa);
    path_point->set_ddkappa(point.path_point.ddkappa);
    path_point->set_x_derivative(point.path_point.x_derivative);
    path_point->set_y_derivative(point.path_point.y_derivative);

    ref_path_point->set_distance_to_left_lane_border(
        point.distance_to_left_lane_border);
    ref_path_point->set_distance_to_left_road_border(
        point.distance_to_left_road_border);
    ref_path_point->set_distance_to_right_lane_border(
        point.distance_to_right_lane_border);
    ref_path_point->set_distance_to_right_road_border(
        point.distance_to_right_road_border);
    ref_path_point->set_lane_width(point.lane_width);
    ref_path_point->set_max_velocity(point.max_velocity);
    ref_path_point->set_min_velocity(point.min_velocity);
    ref_path_point->set_is_in_intersection(point.is_in_intersection);
  }

  // 6. set state of start & stop
  auto start_stop_state = lon_behav_plan_input_->mutable_start_stop_info();
  start_stop_state->CopyFrom(start_stop_state_info);

  // 6. set function_info
  auto function_info_input = lon_behav_plan_input_->mutable_function_info();
  function_info_input->CopyFrom(function_info);
  const auto& route_info_output = session_->
      environmental_model().get_route_info()->get_route_info_output();
  double dis_to_ramp = route_info_output.dis_to_ramp;
  double dis_to_merge = route_info_output.distance_to_first_road_merge;
  bool is_on_ramp = route_info_output.is_on_ramp;
  bool is_continuous_ramp = virtual_lane_manager->is_continuous_ramp();
  double dis_to_stopline = virtual_lane_manager->GetEgoDistanceToStopline();
  double dis_to_crosswalk = virtual_lane_manager->GetEgoDistanceToCrosswalk();
  planning::common::IntersectionState intersection_state =
      virtual_lane_manager->GetIntersectionState();

  lon_behav_plan_input_->set_dis_to_ramp(dis_to_ramp);
  lon_behav_plan_input_->set_dis_to_merge(dis_to_merge);
  lon_behav_plan_input_->set_is_on_ramp(is_on_ramp);
  lon_behav_plan_input_->set_is_continuous_ramp(is_continuous_ramp);
  lon_behav_plan_input_->set_dis_to_stopline(dis_to_stopline);
  lon_behav_plan_input_->set_dis_to_crosswalk(dis_to_crosswalk);
  lon_behav_plan_input_->set_intersection_state(intersection_state);
  lon_behav_plan_input_->set_sdmap_has_curv(JudgeCurvBySDMap());

  // set lon_init_state
  auto lon_init_state_ptr = lon_behav_plan_input_->mutable_lon_init_state();
  lon_init_state_ptr->set_s(
      ego_state_mgr->planning_init_point().lon_init_state.s());
  lon_init_state_ptr->set_v(
      ego_state_mgr->planning_init_point().lon_init_state.v());
  lon_init_state_ptr->set_a(
      ego_state_mgr->planning_init_point().lon_init_state.a());
  lon_init_state_ptr->set_j(
      ego_state_mgr->planning_init_point().lon_init_state.j());
}

void SccLonBehaviorPlanner::SetInput(
    planning::common::RealTimeLonBehaviorInput &lon_behav_plan_input) {
  *lon_behav_plan_input_ = lon_behav_plan_input;
}

bool SccLonBehaviorPlanner::Update() {
  LOG_DEBUG("=======Entering SccLonBehaviorPlanner::Update======= \n");
  // 1.ST
  st_graph_->Update(lon_behav_plan_input_, session_);

  // 2.SV
  sv_graph_->Update(lon_behav_plan_input_);

  // 3.generate bounds
  auto st_boundaries = st_graph_->GetSTboundaries();
  auto s_refs = st_graph_->GetSTRefs();
  auto v_refs = st_graph_->GetVTRefs();
  auto sv_boundaries = sv_graph_->GetSVBoundaries();

  // 4.update bounds
  auto a_bounds = st_graph_->GetAccBound();
  auto j_bounds = st_graph_->GetJerkBound();

  ClearOutput();
  UpdateLonRefPath(s_refs, v_refs, st_boundaries, sv_boundaries, a_bounds,
                   j_bounds);
  GenerateLonRefPathPB();
  UpdateHMI();
  return true;
}

void SccLonBehaviorPlanner::UpdateLonRefPath(
    const std::vector<double> &s_refs, const std::vector<double> &v_refs,
    const scc::STboundaries &st_boundaries, const SVBoundaries &sv_boundaries,
    const std::pair<double, double> &a_bounds,
    const std::pair<double, double> &j_bounds) {
  auto v_cruise = lon_behav_plan_input_->ego_info().ego_cruise();
  auto init_state_a = lon_init_state_[2];
  lon_behav_output_.t_list.resize(config_.lon_num_step + 1);
  lon_behav_output_.s_refs.resize(config_.lon_num_step + 1);
  lon_behav_output_.ds_refs.resize(config_.lon_num_step + 1);
  lon_behav_output_.hard_bounds.resize(config_.lon_num_step + 1);
  lon_behav_output_.soft_bounds.resize(config_.lon_num_step + 1);
  lon_behav_output_.lead_bounds.resize(config_.lon_num_step + 1);
  lon_behav_output_.lon_bound_v.resize(config_.lon_num_step + 1);
  lon_behav_output_.lon_bound_a.resize(config_.lon_num_step + 1);
  lon_behav_output_.lon_bound_jerk.resize(config_.lon_num_step + 1);
  WeightedBounds s_hard_bounds;
  s_hard_bounds.emplace_back(WeightedBound{0.0 - 10.0, kSUpperBound, -1.0});
  WeightedBounds s_soft_bounds;
  s_soft_bounds.emplace_back(WeightedBound{0.0 - 10.0, kSUpperBound, -1.0});
  LonLeadBounds s_lead_bounds;
  s_lead_bounds.emplace_back(LonLeadBound{300.0, 0.0, 0.0, -1});
  Bound lon_v_bound{-0.1, std::min(v_cruise, config_.velocity_upper_bound)};
  // get lane change info
  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  if (coarse_planning_info.target_state == kLaneChangePropose ||
      coarse_planning_info.target_state == kLaneChangeExecution) {
    lon_v_bound.upper = std::min(v_cruise * kVelBoundCoeff,
                                 config_.velocity_upper_bound_in_lane_change);
  }
  Bound lon_a_bound{a_bounds.first, a_bounds.second};
  Bound lon_j_bound{j_bounds.first, j_bounds.second};

  for (unsigned int i = 0; i <= config_.lon_num_step; i++) {
    // 0. 避免init a 超出 a bound
    lon_a_bound.lower = std::fmin(a_bounds.first, init_state_a);
    lon_a_bound.upper = std::fmax(
        a_bounds.second, init_state_a + kJerkMin * config_.delta_time * i);

    // 1.update t_list
    lon_behav_output_.t_list[i] = i * config_.delta_time;
    // 2.update s_refs <s_ref, weight>
    lon_behav_output_.s_refs[i] = {s_refs.at(i), 1.0};
    // 3.update ds_refs  临时使用巡航车速，后续用s-v信息
    lon_behav_output_.ds_refs[i] = {v_refs.at(i), 0.0};
    // 4.construct default s_bounds
    // hack: 先默认自车s = 0
    lon_behav_output_.hard_bounds[i] = s_hard_bounds;
    // 5.construct default s_soft_bounds
    // hack: 先默认自车s = 0
    lon_behav_output_.soft_bounds[i] = s_soft_bounds;
    // 5*.construct default s_lead_bounds
    lon_behav_output_.lead_bounds[i] = s_lead_bounds;
    // 6.update v bounds
    lon_behav_output_.lon_bound_v[i] = lon_v_bound;
    // 7.update a bounds
    lon_behav_output_.lon_bound_a[i] = lon_a_bound;
    // 8.update jerk bounds
    lon_behav_output_.lon_bound_jerk[i] = lon_j_bound;
  }
  // 9.update bounds by obstacles: s_soft_bounds & hard_bounds
  // TBD: weight是hack的10
  for (auto &st_bound : st_boundaries) {
    for (unsigned int i = 0; i <= config_.lon_num_step; i++) {
      WeightedBound s_hard_bound;
      s_hard_bound.lower = st_bound.hard_bound.at(i).lower;
      s_hard_bound.upper = st_bound.hard_bound.at(i).upper;
      s_hard_bound.weight = 10;
      s_hard_bound.bound_info.id = st_bound.id;
      s_hard_bound.bound_info.type = BoundType::AGENT;
      WeightedBound s_soft_bound;
      s_soft_bound.lower = st_bound.soft_bound.at(i).lower;
      s_soft_bound.upper = st_bound.soft_bound.at(i).upper;
      s_soft_bound.weight = 10;
      s_soft_bound.bound_info.id = st_bound.id;
      s_soft_bound.bound_info.type = BoundType::AGENT;
      LonLeadBound s_lead_bound;
      s_lead_bound.s_lead = st_bound.hard_bound.at(i).upper;
      s_lead_bound.v_lead = st_bound.hard_bound.at(i).vel;
      s_lead_bound.a_lead = st_bound.hard_bound.at(i).acc;
      s_lead_bound.lead_id = st_bound.hard_bound.at(i).id;

      lon_behav_output_.hard_bounds[i].emplace_back(s_hard_bound);
      lon_behav_output_.soft_bounds[i].emplace_back(s_soft_bound);
      lon_behav_output_.lead_bounds[i].emplace_back(s_lead_bound);
    }
  }
  // 10. using JLT to update Sref in farslow and stable car case
  bool jlt_farslow_status = false;
  bool jlt_stable_status = false;
  GetHardBounds();
  std::vector<double> sref_farslow;
  std::vector<double> sref_stable;
  sref_farslow.resize(config_.lon_num_step + 1);
  sref_stable.resize(config_.lon_num_step + 1);
  jlt_farslow_status = GenerateFarSlowCarFollowCurve(sref_farslow);
  jlt_stable_status = GenerateStableFollowSlowCurve(sref_farslow);
  JSON_DEBUG_VALUE("jlt_status_farslow", jlt_farslow_status)
  JSON_DEBUG_VALUE("jlt_status_stable", jlt_stable_status)
  const auto &lane_change_info =
      session_->planning_context().lane_change_decider_output();
  if (jlt_farslow_status || jlt_stable_status ||
      (lane_change_info.s_search_status && config_.enable_speed_adjust)) {
    lon_j_bound.lower = -1.0;
    lon_j_bound.upper = 1.0;
  }
  for (unsigned int i = 0; i <= config_.lon_num_step; i++) {
    // update jerk bounds
    lon_behav_output_.lon_bound_jerk[i] = lon_j_bound;
  }
  if (config_.enable_jlt) {
    for (unsigned int i = 0; i <= config_.lon_num_step; i++) {
      if (jlt_farslow_status) {
        lon_behav_output_.s_refs[i].first =
            std::fmin(lon_behav_output_.s_refs[i].first, sref_farslow[i]);
      }
      if (jlt_stable_status) {
        lon_behav_output_.s_refs[i].first =
            std::fmin(lon_behav_output_.s_refs[i].first, sref_stable[i]);
      }
    }
  }

  // 11. use speed adjust s search ref
  if (lane_change_info.s_search_status && config_.enable_speed_adjust) {
    if (lane_change_info.st_search_vec.size() == config_.lon_num_step + 1) {
      for (size_t i = 0; i <= config_.lon_num_step; i++) {
        lon_behav_output_.s_refs[i].first = lane_change_info.st_search_vec[i];
      }
      std::cout << "use search path in lc wait!" << std::endl;
    } else {
      std::cout << "search path num is error:  "
                << lane_change_info.st_search_vec.size() << std::endl;
    }
  }

  // 12.update sv boundary
  SVBoundary sv_boundary_tmp = sv_boundaries.front();
  for (auto it = std::next(sv_boundaries.begin()); it != sv_boundaries.end();
       ++it) {
    for (unsigned int i = 0; i <= config_.lon_num_step; ++i) {
      sv_boundary_tmp.sv_bounds[i].s = it->sv_bounds.at(i).s;
      sv_boundary_tmp.sv_bounds[i].v_bound.upper =
          std::min(it->sv_bounds.at(i).v_bound.upper,
                   sv_boundary_tmp.sv_bounds[i].v_bound.upper);
    }
  }
  lon_behav_output_.lon_sv_boundary = std::move(sv_boundary_tmp);
}

void SccLonBehaviorPlanner::GenerateLonRefPathPB() {
  // 1.update t_list
  lon_behav_output_pb_.mutable_t_list()->Reserve(
      lon_behav_output_.t_list.size());
  for (const auto &t : lon_behav_output_.t_list) {
    lon_behav_output_pb_.add_t_list(t);
  }
  // 2.update s_refs
  lon_behav_output_pb_.mutable_s_refs()->Reserve(
      lon_behav_output_.s_refs.size());
  for (const auto &s_ref : lon_behav_output_.s_refs) {
    auto add_s_ref = lon_behav_output_pb_.add_s_refs();
    add_s_ref->set_first(s_ref.first);   // offset
    add_s_ref->set_second(s_ref.first);  // weight
  }
  // 3.update ds_refs
  lon_behav_output_pb_.mutable_ds_refs()->Reserve(
      lon_behav_output_.ds_refs.size());
  for (const auto &ds_ref : lon_behav_output_.ds_refs) {
    auto add_ds_ref = lon_behav_output_pb_.add_ds_refs();
    add_ds_ref->set_first(ds_ref.first);   // offset
    add_ds_ref->set_second(ds_ref.first);  // weight
  }
  // 4.update hard bounds
  lon_behav_output_pb_.mutable_bounds()->Reserve(
      lon_behav_output_.hard_bounds.size());
  for (const auto &bounds : lon_behav_output_.hard_bounds) {
    auto bounds_pb = lon_behav_output_pb_.add_bounds();
    for (const auto &bound : bounds) {
      auto bound_pb = bounds_pb->add_bound();
      bound_pb->set_lower(bound.lower);
      bound_pb->set_upper(bound.upper);
      bound_pb->set_weight(bound.weight);
      bound_pb->mutable_bound_info()->set_id(bound.bound_info.id);
      bound_pb->mutable_bound_info()->set_type(
          BoundType2String(bound.bound_info.type));
    }
  }
  // 5.update lon_soft_bounds
  lon_behav_output_pb_.mutable_soft_bounds()->Reserve(
      lon_behav_output_.soft_bounds.size());
  for (const auto &lon_soft_bounds : lon_behav_output_.soft_bounds) {
    auto lon_soft_bounds_pb = lon_behav_output_pb_.add_soft_bounds();
    for (const auto &lon_soft_bound : lon_soft_bounds) {
      auto lon_soft_bound_pb = lon_soft_bounds_pb->add_bound();
      lon_soft_bound_pb->set_lower(lon_soft_bound.lower);
      lon_soft_bound_pb->set_upper(lon_soft_bound.upper);
      lon_soft_bound_pb->set_weight(lon_soft_bound.weight);
      lon_soft_bound_pb->mutable_bound_info()->set_id(
          lon_soft_bound.bound_info.id);
      lon_soft_bound_pb->mutable_bound_info()->set_type(
          BoundType2String(lon_soft_bound.bound_info.type));
    }
  }
  // 6.update lon_sv_boundary
  lon_behav_output_pb_.mutable_lon_sv_boundary()->mutable_sv_bounds()->Reserve(
      lon_behav_output_.lon_sv_boundary.sv_bounds.size());
  lon_behav_output_pb_.mutable_lon_sv_boundary()->set_boundary_type(
      (common::SVBoundary::SVBoundaryType)
          lon_behav_output_.lon_sv_boundary.boundary_type);

  for (const auto &sv_bound : lon_behav_output_.lon_sv_boundary.sv_bounds) {
    auto sv_bound_pb =
        lon_behav_output_pb_.mutable_lon_sv_boundary()->add_sv_bounds();
    sv_bound_pb->set_s(sv_bound.s);
    sv_bound_pb->mutable_v_bound()->set_lower(sv_bound.v_bound.lower);
    sv_bound_pb->mutable_v_bound()->set_upper(sv_bound.v_bound.upper);
  }

  // 7.update lon_bound_v
  auto lon_bounds_v_pb = lon_behav_output_pb_.mutable_lon_bound_v();
  lon_bounds_v_pb->mutable_bound()->Reserve(
      lon_behav_output_.lon_bound_v.size());
  for (const auto &lon_bound_v : lon_behav_output_.lon_bound_v) {
    auto lon_bound_v_pb = lon_bounds_v_pb->add_bound();
    lon_bound_v_pb->set_lower(lon_bound_v.lower);
    lon_bound_v_pb->set_upper(lon_bound_v.upper);
  }
  // 8.update lon_bound_a
  auto lon_bounds_a_pb = lon_behav_output_pb_.mutable_lon_bound_a();
  lon_bounds_a_pb->mutable_bound()->Reserve(
      lon_behav_output_.lon_bound_a.size());
  for (const auto &lon_bound_a : lon_behav_output_.lon_bound_a) {
    auto lon_bound_a_pb = lon_bounds_a_pb->add_bound();
    lon_bound_a_pb->set_lower(lon_bound_a.lower);
    lon_bound_a_pb->set_upper(lon_bound_a.upper);
  }
  // 9.update lon_bound_jerk
  auto lon_bounds_jerk_pb = lon_behav_output_pb_.mutable_lon_bound_jerk();
  lon_bounds_jerk_pb->mutable_bound()->Reserve(
      lon_behav_output_.lon_bound_jerk.size());
  for (const auto &lon_bound_jerk : lon_behav_output_.lon_bound_jerk) {
    auto lon_bound_jerk_pb = lon_bounds_jerk_pb->add_bound();
    lon_bound_jerk_pb->set_lower(lon_bound_jerk.lower);
    lon_bound_jerk_pb->set_upper(lon_bound_jerk.upper);
  }
}

void SccLonBehaviorPlanner::UpdateHMI() {
  auto hmi_info =
      session_->mutable_planning_context()->mutable_planning_hmi_info();
  auto lc_status = lon_behav_plan_input_->lat_output().lc_status();
  auto lateral_obstacle = lon_behav_plan_input_->lat_obs_info();

  // check lead one
  bool is_lead_one_vehicle = IsLeadVehicle(lateral_obstacle.lead_one());
  // check lead two
  bool is_lead_two_vehicle = IsLeadVehicle(lateral_obstacle.lead_two());
  // check temp lead one
  bool is_temp_lead_one_vehicle =
      IsLeadVehicle(lateral_obstacle.temp_lead_one());
  // check temp lead two
  bool is_temp_lead_two_vehicle =
      IsLeadVehicle(lateral_obstacle.temp_lead_two());

  // 1. update CIPV
  int CIPV_id = -1;
  if ((lc_status != "left_lane_change") && (lc_status != "right_lane_change")) {
    if (is_lead_one_vehicle) {
      hmi_info->cipv_info.has_cipv = true;
      hmi_info->cipv_info.cipv_id = lateral_obstacle.lead_one().track_id();
      CIPV_id = lateral_obstacle.lead_one().track_id();
    } else if (!is_lead_one_vehicle && is_lead_two_vehicle) {
      hmi_info->cipv_info.has_cipv = true;
      hmi_info->cipv_info.cipv_id = lateral_obstacle.lead_two().track_id();
      CIPV_id = lateral_obstacle.lead_two().track_id();
    } else {
      hmi_info->cipv_info.has_cipv = false;
      hmi_info->cipv_info.cipv_id = -1;
      CIPV_id = -1;
    }
  } else {
    if (is_temp_lead_one_vehicle) {
      hmi_info->cipv_info.has_cipv = true;
      hmi_info->cipv_info.cipv_id = lateral_obstacle.temp_lead_one().track_id();
      CIPV_id = lateral_obstacle.temp_lead_one().track_id();
    } else if (!is_temp_lead_one_vehicle && is_temp_lead_two_vehicle) {
      hmi_info->cipv_info.has_cipv = true;
      hmi_info->cipv_info.cipv_id = lateral_obstacle.temp_lead_two().track_id();
      CIPV_id = lateral_obstacle.temp_lead_two().track_id();
    } else {
      hmi_info->cipv_info.has_cipv = false;
      hmi_info->cipv_info.cipv_id = -1;
      CIPV_id = -1;
    }
  }
  JSON_DEBUG_VALUE("CIPV_id", CIPV_id);
}

bool SccLonBehaviorPlanner::IsLeadVehicle(
    const planning::common::TrackedObjectInfo &lead) {
  return lead.track_id() != -1 && lead.is_lead() && lead.is_car();
}

void SccLonBehaviorPlanner::GetHardBounds() {
  const auto &s_bounds = lon_behav_output_.lead_bounds;
  size_t size = s_bounds.size();
  hard_bounds_.reserve(s_bounds.size());
  for (size_t i = 0; i < size; ++i) {
    STBound tmp_bound;
    for (auto &bound : s_bounds[i]) {
      if (bound.s_lead < tmp_bound.upper) {
        tmp_bound.upper = bound.s_lead;
        tmp_bound.vel = bound.v_lead;
        tmp_bound.acc = bound.a_lead;
        tmp_bound.id = bound.lead_id;
      }
    }
    hard_bounds_.emplace_back(tmp_bound);
  }
}

bool SccLonBehaviorPlanner::GenerateFarSlowCarFollowCurve(
    std::vector<double> &s_refs) {
  // check far slow car
  const int check_idx = 8;
  const auto &check_st = hard_bounds_[check_idx];

  constexpr double min_far_distance_threshold = 15.0;
  const double add_time_gap = 0.3;
  // const double add_time_gap = std::fmax(0.0, matched_headway - 1.2);
  const double far_time_gap = kMinFarTimeGap + add_time_gap;
  const double ego_move_dist =
      std::fmax(lon_init_state_[1] * far_time_gap + kDefaultFollowMinDist,
                min_far_distance_threshold);
  const double lead_bound_s_diff =
      check_st.upper - check_time * check_st.vel;  // + check_idx * 0.2
  // check lead bound s_diff
  if (lead_bound_s_diff < ego_move_dist) {
    return false;
  }

  if (hard_bounds_.size() < check_idx || check_st.id == -1) {
    return false;
  }

  const double preview_ego_v =
      lon_init_state_[1] + lon_init_state_[2] * kPreviewTime;

  // check preview_ego_v
  if (preview_ego_v - check_st.vel < kSpeedBuffer) {
    return false;
  }

  if (lon_init_state_[1] < kEgoSpeedThreshold) {
    return false;
  }

  // if (check_st.vel > kLowAgentSpeed) {
  //   return false;
  // }

  const double far_preview_dist = kFarPreviewTime * lon_init_state_[1];
  const double near_preview_dist = kNearPreviewTime * lon_init_state_[1];

  const double far_distance =
      std::fmax(kFarDistanceThreshold, far_preview_dist);
  const double near_distance =
      std::fmax(kNearDistanceThreshold, near_preview_dist);
  const double init_dist = hard_bounds_.front().upper;

  const double far_slow_follow_time_gap = planning_math::LerpWithLimit(
      kNearDistFollowTimeGap, near_distance, kFarDistFollowTimeGap,
      far_distance, init_dist);

  // generate relative coord
  constexpr double min_follow_distance_m = 3.0;
  CoordinateParam coordinate_param;
  const auto &end_bound = hard_bounds_.back();
  const double vel = end_bound.vel;
  const double target_distance = std::max(
      lon_init_state_[1] * far_slow_follow_time_gap + min_follow_distance_m,
      min_follow_distance_m);
  const double s_target = end_bound.upper - 5.0 * vel - target_distance;
  coordinate_param.s_start = s_target;
  coordinate_param.v = vel;

  // using jlt to generate s v curve
  double acc_min = planning_math::LerpWithLimit(
      kAccMinLower, kSpeedLower, kAccMinUpper, kSpeedUpper, lon_init_state_[1]);

  constexpr double AccMinThreshold = -3.0;

  LonState init_state;
  init_state.p = lon_init_state_[0];
  init_state.v = lon_init_state_[1];
  init_state.a = std::fmax(AccMinThreshold, lon_init_state_[2]);

  StateLimit state_limit;
  state_limit.p_end = 0.0;
  state_limit.v_min = -0.1;
  state_limit.v_max = 40.0;
  state_limit.a_min = acc_min;
  state_limit.a_max = kAccMax;
  state_limit.j_min = kJerkMin;
  state_limit.j_max = kJerkMax;

  // const int32_t kMaxLoopNum = 1;
  double acc_min_lower = -1.5;
  double acc_min_upper = state_limit.a_min;
  double a_min = 0.5 * (acc_min_upper + acc_min_lower);

  state_limit.a_min = a_min;
  auto far_slow_curve =
      VariableCoordinateTimeOptimalTrajectory::ConstructInstance(
          init_state, state_limit, coordinate_param, kPositionPrecision);

  const double delta_time = 0.2;
  s_refs[0] = 0.0;
  for (int i = 1; i <= 25; i++) {
    double time = i * delta_time;
    double s_ego = far_slow_curve.Evaluate(0, time);
    // double v_ego = far_slow_curve.Evaluate(1, time);
    s_refs[i] = s_ego;
    // safty check
    if (s_ego > hard_bounds_[i].upper) {
      return false;
    }
  }
  return true;
}

bool SccLonBehaviorPlanner::GenerateStableFollowSlowCurve(
    std::vector<double> &s_refs) {
  // for stable follow trajectory
  const double stable_vehicle_max_acc = 0.3;
  const double stable_vehicle_min_acc = -0.8;
  constexpr double v_min = -0.1;
  constexpr double v_max = 40.0;
  constexpr double a_min = -1.0;
  constexpr double a_max = 1.8;
  constexpr double j_min = -1.0;
  constexpr double j_max = 1.0;
  constexpr double p_precision = 0.1;

  // check stable agent
  const int check_idx = 15;
  if (hard_bounds_.size() <= check_idx) {
    return false;
  }

  // check lane change status
  const auto &lc_request = lon_behav_plan_input_->lat_output().lc_request();
  const auto &lc_status = lon_behav_plan_input_->lat_output().lc_status();
  const bool is_in_lane_change =
      ((lc_request != "none") && (lc_status != "none"));
  if (is_in_lane_change) {
    return false;
  }

  // check lead id
  // const double check_front_time = 2.0;
  // const double dt = 0.2;
  for (unsigned t = 0; t < config_.lon_num_step; t++) {
    const auto &current_bound_id = hard_bounds_[t].id;
    const auto &next_bound_id = hard_bounds_[t + 1].id;
    if (current_bound_id != next_bound_id) {
      return false;
    }
  }

  const auto &start_bound = hard_bounds_.front();
  const double init_s_distance = start_bound.upper - lon_init_state_[0];
  const double front_vel = start_bound.vel;
  const double front_acc = start_bound.acc;
  if (front_acc > stable_vehicle_max_acc ||
      front_acc < stable_vehicle_min_acc) {
    // front vehicle acc not stable
    return false;
  }

  const double vel_diff_threshold = std::min(3.0, lon_init_state_[1] * 0.3);
  if (std::fabs(front_vel - lon_init_state_[1]) > vel_diff_threshold) {
    // vel diff too large
    return false;
  }

  constexpr double min_follow_distance_m = 3.0;
  const double follow_time_gap = 1.5;
  const double target_s_distance =
      std::max(lon_init_state_[1] * follow_time_gap + min_follow_distance_m,
               min_follow_distance_m);
  const double distance_ratio = 0.3;

  if (target_s_distance - init_s_distance >
      target_s_distance * distance_ratio) {
    // init_s_distance too small than target_s_distance
    return false;
  }

  CoordinateParam relative_coordinate_param;
  const auto &last_bound = hard_bounds_.back();
  const double vel = last_bound.vel;
  const double target_distance =
      std::max(lon_init_state_[1] * follow_time_gap + min_follow_distance_m,
               min_follow_distance_m);
  const double s_target = last_bound.upper - 5.0 * vel - target_distance;
  relative_coordinate_param.s_start = s_target;
  relative_coordinate_param.v = vel;

  LonState init_state;
  init_state.p = lon_init_state_[0];
  init_state.v = lon_init_state_[1];
  init_state.a = lon_init_state_[2];

  StateLimit state_limit;
  state_limit.p_end = 0.0;
  state_limit.v_min = v_min;
  state_limit.v_max = v_max;
  state_limit.a_min = a_min;
  state_limit.a_max = a_max;
  state_limit.j_min = j_min;
  state_limit.j_max = j_max;

  auto follow_stable_target_trajectory =
      VariableCoordinateTimeOptimalTrajectory::ConstructInstance(
          init_state, state_limit, relative_coordinate_param, p_precision);

  const double delta_time = 0.2;
  for (int i = 0; i <= 25; i++) {
    double time = i * delta_time;
    double s_ego = follow_stable_target_trajectory.Evaluate(0, time);
    // double v_ego = follow_stable_target_trajectory.Evaluate(1, time);
    s_refs[i] = s_ego;
    // safty check
    if (s_ego > hard_bounds_[i].upper) {
      return false;
    }
  }
  return true;
}

void SccLonBehaviorPlanner::SaveToSession() {
  // 更新状态信息
  auto &start_stop_state_info =
      session_->mutable_planning_context()->mutable_start_stop_result();
  start_stop_state_info.CopyFrom(st_graph_->GetStartStopState());

  // 更新lon_decision_info
  auto &lon_decision_info =
      session_->mutable_planning_context()->mutable_lon_decision_result();
  lon_decision_info.CopyFrom(lon_behav_plan_input_->lon_decision_info());
}

void SccLonBehaviorPlanner::SaveToDebugInfo() {
  // 转存纵向决策信息至debuginfo
  auto &debug_info_pb = DebugInfoManager::GetInstance().GetDebugInfoPb();
  debug_info_pb->mutable_real_time_lon_behavior_planning_input()->CopyFrom(
      *lon_behav_plan_input_);
  auto &lon_ref_path = session_->mutable_planning_context()
                           ->mutable_longitudinal_decider_output();

  lon_ref_path = lon_behav_output_;
  auto long_ref_path_pb = debug_info_pb->mutable_long_ref_path();
  long_ref_path_pb->CopyFrom(lon_behav_output_pb_);
}

void SccLonBehaviorPlanner::ClearOutput() {
  lon_behav_output_pb_.Clear();

  lon_behav_output_.t_list.clear();
  lon_behav_output_.s_refs.clear();
  lon_behav_output_.ds_refs.clear();
  lon_behav_output_.hard_bounds.clear();
  lon_behav_output_.soft_bounds.clear();
  lon_behav_output_.lon_lead_bounds.clear();
  lon_behav_output_.lon_obstacle_overlap_info.clear();
  lon_behav_output_.lon_obstacle_yield_info.clear();
  lon_behav_output_.lon_sv_boundary.sv_bounds.clear();
  lon_behav_output_.lon_bound_v.clear();
  lon_behav_output_.lon_bound_a.clear();
  lon_behav_output_.lon_bound_jerk.clear();
  hard_bounds_.clear();
}

}  // namespace planning
