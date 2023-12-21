#include "real_time_lon_behavior_planner.h"

#include <memory>

#include "debug_info_log.h"
#include "ifly_time.h"
#include "planning_output_context.h"
#include "scenario_state_machine.h"

namespace planning {

RealTimeLonBehaviorPlanner::RealTimeLonBehaviorPlanner(
    const EgoPlanningConfigBuilder *config_builder,
    const std::shared_ptr<TaskPipelineContext> &pipeline_context)
    : Task(config_builder, pipeline_context) {
  config_ = config_builder->cast<RealTimeLonBehaviorPlannerConfig>();
  name_ = "RealTimeLonBehaviorPlanner";

  Init();
}

RealTimeLonBehaviorPlanner::RealTimeLonBehaviorPlanner(
    const EgoPlanningConfigBuilder *config_builder)
    : Task(config_builder) {
  config_ = config_builder->cast<RealTimeLonBehaviorPlannerConfig>();
  name_ = "RealTimeLonBehaviorPlanner";

  Init();
}

bool RealTimeLonBehaviorPlanner::Execute(framework::Frame *frame) {
  LOG_DEBUG("%s !! \n", name_.c_str());
  if (!Task::Execute(frame)) {
    return false;
  }

  if (Calculate()) {
    return true;
  } else {
    return false;
  }
}

void RealTimeLonBehaviorPlanner::Init() {
  lon_behav_plan_input_ = std::make_shared<common::RealTimeLonBehaviorInput>();
  st_graph_ = std::make_shared<StGraphGenerator>(config_);
  sv_graph_ = std::make_shared<SvGraphGenerator>(config_);
}

bool RealTimeLonBehaviorPlanner::Calculate() {
  LOG_DEBUG("=======RealTimeLonBehaviorPlanner======= \n");
  auto start_time = IflyTime::Now_ms();

  ConstructLonBehavInput();
  auto construct_input_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("RealTimeLonBehaviorConstructInputCostTime",
                   construct_input_time - start_time);
  Update();
  SaveToSession();
  SaveToDebugInfo();

  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("RealTimeLonBehaviorCostTime", end_time - start_time);
  return true;
}

void RealTimeLonBehaviorPlanner::SetConfig(
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

void RealTimeLonBehaviorPlanner::ConstructLonBehavInput() {
  const auto &environmental_model = frame_->session()->environmental_model();
  const bool dbw_status = environmental_model.GetVehicleDbwStatus();
  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  const auto lateral_obstacles = environmental_model.get_lateral_obstacle();
  const auto virtual_lane_manager =
      environmental_model.get_virtual_lane_manager();
  const auto lane_tracks_mgr = environmental_model.get_lane_tracks_manager();
  const auto lateral_outputs =
      frame_->session()->planning_context().lateral_behavior_planner_output();
  const auto start_stop_state_info =
      frame_->session()->planning_context().start_stop_result();
  auto &lon_decision_info = frame_->mutable_session()
                                ->mutable_planning_context()
                                ->mutable_lon_decision_result();
  auto &function_info =
      frame_->session()->environmental_model().function_info();
  const auto lane_change_lane_manager = frame_->session()
                                            ->planning_context()
                                            .scenario_state_machine()
                                            ->get_lane_change_lane_manager();

  const auto planning_status =
      frame_->session()->planning_output_context().planning_status();

  // 0. set dbw (Drive-by-Wire)
  lon_behav_plan_input_->set_dbw_status(dbw_status);
  // 1. set ego_info & lon_decision_info
  auto ego_info = lon_behav_plan_input_->mutable_ego_info();
  ego_info->set_ego_v(ego_state_mgr->ego_v());
  ego_info->set_ego_cruise(std::max(ego_state_mgr->ego_v_cruise(), 0.0));
  ego_info->set_ego_acc(ego_state_mgr->ego_acc());
  ego_info->set_ego_steer_angle(ego_state_mgr->ego_steer_angle());

  auto lon_decision_info_input =
      lon_behav_plan_input_->mutable_lon_decision_info();
  lon_decision_info_input->CopyFrom(lon_decision_info);

  // 2. set lat_output
  auto lat_output = lon_behav_plan_input_->mutable_lat_output();
  lat_output->set_lc_request(lateral_outputs.lc_request);
  lat_output->set_lc_status(lateral_outputs.lc_status);
  lat_output->set_close_to_accident(lateral_outputs.close_to_accident);
  for (auto coeff : lateral_outputs.d_poly) {
    lat_output->add_d_poly_vec(coeff);
  }

  // 3. set lateral obstacles
  if (lateral_obstacles->leadone() != nullptr) {
    auto lead_one =
        lon_behav_plan_input_->mutable_lat_obs_info()->mutable_lead_one();
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
  }

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
  }

  if (lateral_obstacles->tleadone() != nullptr) {
    auto temp_lead_one =
        lon_behav_plan_input_->mutable_lat_obs_info()->mutable_temp_lead_one();
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
  }
  if (lateral_obstacles->tleadtwo() != nullptr) {
    auto temp_lead_two =
        lon_behav_plan_input_->mutable_lat_obs_info()->mutable_temp_lead_two();
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
  }

  // 4. set lane change info
  const auto current_lane = virtual_lane_manager->get_current_lane();
  int lc_map_decision = virtual_lane_manager->lc_map_decision(current_lane);
  double lc_end_dis = 0;
  auto lane_change_info = lon_behav_plan_input_->mutable_lc_info();
  lane_change_info->set_has_target_lane(
      lane_change_lane_manager->has_target_lane());
  lane_change_info->set_tlane_virtual_id(
      lane_change_lane_manager->tlane_virtual_id());
  lane_change_info->set_v_limit(ego_state_mgr->ego_v_cruise());
  lane_change_info->set_lc_end_dis(lc_end_dis);
  lane_change_info->set_lc_map_decision(lc_map_decision);
  lane_change_info->set_current_lane_type(current_lane->get_lane_type());
  lane_change_info->set_target_gap_obs_first(
      planning_status.lane_status.change_lane.target_gap_obs.first);
  lane_change_info->set_target_gap_obs_second(
      planning_status.lane_status.change_lane.target_gap_obs.second);

  std::vector<TrackedObject> *front_target_tracks =
      lane_tracks_mgr->get_lane_tracks(
          lane_change_lane_manager->tlane_virtual_id(), FRONT_TRACK);
  if (front_target_tracks != nullptr) {
    for (auto &track : *front_target_tracks) {
      // ignore obj without camera source
      if ((track.fusion_source != OBSTACLE_SOURCE_CAMERA) &&
          (track.fusion_source != OBSTACLE_SOURCE_F_RADAR_CAMERA)) {
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
          lane_change_lane_manager->tlane_virtual_id(), SIDE_TRACK);
  if (side_target_tracks != nullptr) {
    for (auto &track : *side_target_tracks) {
      // ignore obj without camera source
      if ((track.fusion_source != OBSTACLE_SOURCE_CAMERA) &&
          (track.fusion_source != OBSTACLE_SOURCE_F_RADAR_CAMERA)) {
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
      lane_change_lane_manager->flane_virtual_id());
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

  double dis_to_ramp = virtual_lane_manager->dis_to_ramp();
  double dis_to_merge = virtual_lane_manager->distance_to_first_road_merge();
  bool is_on_ramp = virtual_lane_manager->is_on_ramp();

  lon_behav_plan_input_->set_dis_to_ramp(dis_to_ramp);
  lon_behav_plan_input_->set_dis_to_merge(dis_to_merge);
  lon_behav_plan_input_->set_is_on_ramp(is_on_ramp);
}

void RealTimeLonBehaviorPlanner::SetInput(
    planning::common::RealTimeLonBehaviorInput &lon_behav_plan_input) {
  *lon_behav_plan_input_ = lon_behav_plan_input;
}

bool RealTimeLonBehaviorPlanner::Update() {
  LOG_DEBUG("=======Entering RealTimeLonBehaviorPlanner::Update======= \n");
  // 1.ST
  st_graph_->Update(lon_behav_plan_input_);

  // 2.SV
  sv_graph_->Update(lon_behav_plan_input_);

  // 3.generate bounds
  auto st_boundaries = st_graph_->GetSTboundaries();
  auto s_refs = st_graph_->GetSTRefs();
  auto v_refs = st_graph_->GetVTRefs();
  auto sv_boundaries = sv_graph_->GetSVBoundaries();

  ClearOutput();
  UpdateLonRefPath(s_refs, v_refs, st_boundaries, sv_boundaries);
  GenerateLonRefPathPB();
  UpdateHMI();
  return true;
}

void RealTimeLonBehaviorPlanner::UpdateLonRefPath(
    const std::vector<double> &s_refs, const std::vector<double> &v_refs,
    const real_time::STboundaries &st_boundaries,
    const SVBoundaries &sv_boundaries) {
  auto v_cruise = lon_behav_plan_input_->ego_info().ego_cruise();
  lon_behav_output_.t_list.resize(config_.lon_num_step + 1);
  lon_behav_output_.s_refs.resize(config_.lon_num_step + 1);
  lon_behav_output_.ds_refs.resize(config_.lon_num_step + 1);
  lon_behav_output_.hard_bounds.resize(config_.lon_num_step + 1);
  lon_behav_output_.soft_bounds.resize(config_.lon_num_step + 1);
  lon_behav_output_.lon_bound_v.resize(config_.lon_num_step + 1);
  lon_behav_output_.lon_bound_a.resize(config_.lon_num_step + 1);
  lon_behav_output_.lon_bound_jerk.resize(config_.lon_num_step + 1);
  WeightedBounds s_hard_bounds;
  s_hard_bounds.emplace_back(WeightedBound{0.0 - 10.0, 150.0, -1.0});
  WeightedBounds s_soft_bounds;
  s_soft_bounds.emplace_back(WeightedBound{0.0 - 10.0, 150.0, -1.0});
  Bound lon_v_bound{0.0, std::min(v_cruise, config_.velocity_upper_bound)};
  Bound lon_a_bound{-7.0, 2.0};
  Bound lon_j_bound{-7.0, 7.0};

  for (unsigned int i = 0; i <= config_.lon_num_step; i++) {
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
      s_hard_bound.bound_info.type = "obstacle";
      WeightedBound s_soft_bound;
      s_soft_bound.lower = st_bound.soft_bound.at(i).lower;
      s_soft_bound.upper = st_bound.soft_bound.at(i).upper;
      s_soft_bound.weight = 10;
      s_soft_bound.bound_info.id = st_bound.id;
      s_soft_bound.bound_info.type = "obstacle";

      lon_behav_output_.hard_bounds[i].emplace_back(s_hard_bound);
      lon_behav_output_.soft_bounds[i].emplace_back(s_soft_bound);
    }
  }

  // 10.update sv boundary
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

void RealTimeLonBehaviorPlanner::GenerateLonRefPathPB() {
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
      bound_pb->mutable_bound_info()->set_type(bound.bound_info.type);
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
          lon_soft_bound.bound_info.type);
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

void RealTimeLonBehaviorPlanner::UpdateHMI() {
  auto hmi_info = frame_->mutable_session()
                      ->mutable_planning_output_context()
                      ->mutable_planning_hmi_info();
  auto lc_status = lon_behav_plan_input_->lat_output().lc_status();
  auto lateral_obstacle = lon_behav_plan_input_->lat_obs_info();

  // 1. update CIPV
  int CIPV_id = -1;
  if ((lc_status != "left_lane_change") && (lc_status != "right_lane_change")) {
    if (lateral_obstacle.has_lead_one() &&
        lateral_obstacle.lead_one().type() != 0) {
      hmi_info->mutable_cipv_info()->set_has_cipv(true);
      hmi_info->mutable_cipv_info()->set_cipv_id(
          lateral_obstacle.lead_one().track_id());
      CIPV_id = lateral_obstacle.lead_one().track_id();
    }
  } else {
    if (lateral_obstacle.has_temp_lead_one() &&
        lateral_obstacle.temp_lead_one().type() != 0) {
      hmi_info->mutable_cipv_info()->set_has_cipv(true);
      hmi_info->mutable_cipv_info()->set_cipv_id(
          lateral_obstacle.temp_lead_one().track_id());
      CIPV_id = lateral_obstacle.temp_lead_one().track_id();
    }
  }
  if (CIPV_id == -1) {
    hmi_info->mutable_cipv_info()->set_has_cipv(false);
    hmi_info->mutable_cipv_info()->set_cipv_id(CIPV_id);
  }
  JSON_DEBUG_VALUE("CIPV_id", CIPV_id);
}

void RealTimeLonBehaviorPlanner::SaveToSession() {
  // 更新状态信息
  auto &start_stop_state_info = frame_->mutable_session()
                                    ->mutable_planning_context()
                                    ->mutable_start_stop_result();
  start_stop_state_info.CopyFrom(st_graph_->GetStartStopState());

  // 更新lon_decision_info
  auto &lon_decision_info = frame_->mutable_session()
                                ->mutable_planning_context()
                                ->mutable_lon_decision_result();
  lon_decision_info.CopyFrom(lon_behav_plan_input_->lon_decision_info());
}

void RealTimeLonBehaviorPlanner::SaveToDebugInfo() {
  // 转存纵向决策信息至debuginfo
  auto &debug_info_pb = DebugInfoManager::GetInstance().GetDebugInfoPb();
  debug_info_pb->mutable_real_time_lon_behavior_planning_input()->CopyFrom(
      *lon_behav_plan_input_);
  auto &lon_ref_path = pipeline_context_->planning_info.lon_ref_path;
  lon_ref_path = lon_behav_output_;
  auto long_ref_path_pb = debug_info_pb->mutable_long_ref_path();
  long_ref_path_pb->CopyFrom(lon_behav_output_pb_);
}

void RealTimeLonBehaviorPlanner::ClearOutput() {
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
}

}  // namespace planning
