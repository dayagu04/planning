#include "scc_st_graph.h"

#include <math.h>

#include <algorithm>
#include <cmath>
#include <complex>

#include "behavior_planners/scc_lon_behavior_planner/scc_lon_behavior_types.h"
#include "debug_info_log.h"
#include "ego_planning_config.h"
#include "math/linear_interpolation.h"
#include "planning_context.h"
#include "tasks/behavior_planners/lane_change_decider/lane_change_requests/lane_change_lane_manager.h"
// #include "scenario_state_machine.h"
#include "task_basic_types.h"
#include "task_basic_types.pb.h"
#include "virtual_lane_manager.h"

namespace {

constexpr double kStaticAgentSpeedThr = 3;
constexpr double kStaticAgentPosThr = 1.4;
constexpr double kStaticAgentBuffer = 0.12;
constexpr double kStaticLeadThr = 1.1;
constexpr double kHalfLaneWidth = 1.75;
constexpr double kLeadoneThr = 1.2;
constexpr double kHalfEgoWidth = 1.1;
constexpr double kHalfEgoLength = 2.55;
constexpr double kEgoWidth = 2.2;
constexpr double kEgoLength = 5.1;

void CalculateAgentSLBoundary(const std::shared_ptr<KDPath> &planned_path,
                              const planning_math::Box2d &agent_box,
                              double *const ptr_min_s, double *const ptr_max_s,
                              double *const ptr_min_l,
                              double *const ptr_max_l) {
  if (nullptr == ptr_min_s || nullptr == ptr_max_s || nullptr == ptr_min_l ||
      nullptr == ptr_max_l) {
    return;
  }
  const auto &all_corners = agent_box.GetAllCorners();
  for (const auto &corner : all_corners) {
    double agent_s = 0.0;
    double agent_l = 0.0;
    planned_path->XYToSL(corner.x(), corner.y(), &agent_s, &agent_l);
    *ptr_min_s = std::fmin(*ptr_min_s, agent_s);
    *ptr_max_s = std::fmax(*ptr_max_s, agent_s);
    *ptr_min_l = std::fmin(*ptr_min_l, agent_l);
    *ptr_max_l = std::fmax(*ptr_max_l, agent_l);
  }
}

void CalculateAgentSLBoundary(const std::shared_ptr<KDPath> &planned_path,
                              const agent::Agent &agent,
                              double *const ptr_min_s, double *const ptr_max_s,
                              double *const ptr_min_l,
                              double *const ptr_max_l) {
  const auto &agent_box = agent.box();
  CalculateAgentSLBoundary(planned_path, agent_box, ptr_min_s, ptr_max_s,
                           ptr_min_l, ptr_max_l);
}
}  // namespace

namespace planning {
namespace scc {

StGraphGenerator::StGraphGenerator(const SccLonBehaviorPlannerConfig &config)
    : config_(config) {
  lead_desired_distance_filter_.Init(-0.2, config_.fast_lead_distance_step, 0.0,
                                     200, 0.1);
  lead_two_desired_distance_filter_.Init(-0.2, config_.fast_lead_distance_step,
                                         0.0, 200, 0.1);
  cut_in_desired_distance_filter_.Init(
      -0.2, config_.cut_in_desired_distance_step, 0.0, 200, 0.1);
  // TODO: 不同场景下的速度滤波应初始化不同的滤波器
  accel_vel_filter_.Init(-1.0, 1.0, 0.0, 42.0, 0.1);
  accel_vel_in_turns_filter_.Init(-1.5, 1.0, 0.0, 42.0, 0.1);
}

void StGraphGenerator::Update(
    std::shared_ptr<common::RealTimeLonBehaviorInput> lon_behav_input,
    const TrajectoryPoints &last_traj,
    std::shared_ptr<planning::planning_data::DynamicWorld> dynamic_world,
    std::shared_ptr<VirtualLane> current_lane) {
  lon_behav_input_ = std::move(lon_behav_input);
  LOG_DEBUG("=======Entering StGraphGenerator::Update======= \n");
  double v_ego = lon_behav_input_->ego_info().ego_v();
  double v_cruise = lon_behav_input_->ego_info().ego_cruise();
  double acc_ego = lon_behav_input_->ego_info().ego_acc();
  double steer_angle_ego = lon_behav_input_->ego_info().ego_steer_angle();
  lon_init_state_[0] = lon_behav_input_->lon_init_state().s();
  lon_init_state_[1] = lon_behav_input_->lon_init_state().v();
  lon_init_state_[2] = lon_behav_input_->lon_init_state().a();

  std::vector<double> d_polys;
  const auto &d_poly_infos = lon_behav_input_->lat_output().d_poly_vec();
  for (auto d_poly_info : d_poly_infos) {
    d_polys.push_back(d_poly_info);
  }

  if (lane_changing_decider_ == nullptr) {
    lane_changing_decider_ = std::make_unique<RealTimeLaneChangeDecider>(
        lon_behav_input_->lc_info());
  } else {
    lane_changing_decider_->update_lc_info(lon_behav_input_->mutable_lc_info());
  }

  st_refs_.clear();
  st_boundaries_.clear();
  JSON_DEBUG_VALUE("v_ego", v_ego);

  last_v_target_ = v_target_;
  accel_vel_filter_.SetState(last_v_target_);
  accel_vel_in_turns_filter_.SetState(last_v_target_);

  // 1. 初始化 acc_bound, refs
  // 1.1 acceleration limits for cruising
  CalcCruiseAccelLimits(v_ego);

  // 1.2 初始化v_refs
  v_target_ = v_cruise;
  vt_refs_.resize(config_.lon_num_step + 1);
  for (int i = 0; i < config_.lon_num_step + 1; ++i) {
    vt_refs_[i] = v_cruise;
  }

  // calc target v for noa curv and ramp
  CalcSpeedWithTurns(v_ego, steer_angle_ego, d_polys);

  double distance_to_ramp = lon_behav_input_->dis_to_ramp();
  double distance_to_merge = lon_behav_input_->dis_to_merge();
  bool is_on_ramp = lon_behav_input_->is_on_ramp();
  bool is_continuous_ramp = lon_behav_input_->is_continuous_ramp();
  double ramp_v_limit = config_.v_limit_ramp;
  double acc_to_ramp = -1.0;
  CalcSpeedWithRamp(distance_to_ramp, distance_to_merge, is_on_ramp,
                    is_continuous_ramp, ramp_v_limit, acc_to_ramp, v_ego);

  if (config_.enable_intersection_v_limit) {
    CalcSpeedInfoWithIntersection();
  }

  // 2. 计算障碍物s-t
  // 2.1 计算leads: lead one, 选择性使用lead two
  std::vector<planning::common::RealTimeLonObstacleSTInfo> leads_st_info;
  CalcSpeedInfoWithLead(lon_behav_input_->lat_obs_info().lead_one(),
                        lon_behav_input_->lat_obs_info().lead_two(),
                        lon_behav_input_->lat_output().lc_request(), v_ego,
                        leads_st_info);
  CalculateNarrowLimitSpeed(lon_behav_input_->lat_obs_info(), dynamic_world,
                            current_lane, leads_st_info);

  // 2.2 计算temp lead one, 选择性使用lead two
  std::vector<planning::common::RealTimeLonObstacleSTInfo> temp_leads_st_info;
  CalcSpeedInfoWithTempLead(lon_behav_input_->lat_obs_info().temp_lead_one(),
                            lon_behav_input_->lat_obs_info().temp_lead_two(),
                            v_ego, lon_behav_input_->lat_output(),
                            temp_leads_st_info);

  // 2.3 计算cut in
  std::vector<common::RealTimeLonObstacleSTInfo> cut_in_st_info;
  // 对障碍物进行cut in决策
  CalcSpeedInfoWithCutin(lon_behav_input_->lat_obs_info(),
                         lon_behav_input_->lat_output().lc_status(), v_cruise,
                         v_ego, cut_in_st_info);

  // 2.4 计算lane change，gap相关信息
  std::vector<planning::common::RealTimeLonObstacleSTInfo> lane_change_st_info;
  CalcSpeedInfoWithGap(lon_behav_input_->lat_obs_info().lead_one(), v_cruise,
                       v_ego, lon_behav_input_->lat_output().lc_request(),
                       lon_behav_input_->lat_output().lc_status(),
                       lane_change_st_info);

  // 2.5 计算virtual obstacle st相关信息
  std::vector<planning::common::RealTimeLonObstacleSTInfo> virtual_obs_st_info;
  CalcSpeedInfoWithVirtualObstacle(dynamic_world, virtual_obs_st_info);

  std::vector<common::RealTimeLonObstacleSTInfo> st_infos;
  int st_infos_num = leads_st_info.size() + temp_leads_st_info.size() +
                     cut_in_st_info.size() + lane_change_st_info.size() +
                     virtual_obs_st_info.size();
  st_infos.resize(st_infos_num);
  for (int i = 0; i < leads_st_info.size(); ++i) {
    st_infos[i].CopyFrom(leads_st_info[i]);
  }
  for (int i = 0; i < temp_leads_st_info.size(); ++i) {
    st_infos[leads_st_info.size() + i].CopyFrom(temp_leads_st_info[i]);
  }
  for (int i = 0; i < cut_in_st_info.size(); ++i) {
    st_infos[leads_st_info.size() + temp_leads_st_info.size() + i].CopyFrom(
        cut_in_st_info[i]);
  }
  for (int i = 0; i < lane_change_st_info.size(); ++i) {
    st_infos[leads_st_info.size() + temp_leads_st_info.size() +
             cut_in_st_info.size() + i]
        .CopyFrom(lane_change_st_info[i]);
  }
  for (int i = 0; i < virtual_obs_st_info.size(); ++i) {
    st_infos[leads_st_info.size() + temp_leads_st_info.size() +
             cut_in_st_info.size() + lane_change_st_info.size() + i]
        .CopyFrom(virtual_obs_st_info[i]);
  }

  // filter v target
  if (v_target_ > v_ego) {
    if (start_stop_info_.state() == common::StartStopInfo::START) {
      accel_vel_filter_.SetRate(-config_.acc_start, config_.acc_start);
    } else {
      accel_vel_filter_.SetRate(-1.0, 1.0);
    }
    if (v_ego > v_last_target_) {
      accel_vel_filter_.SetState(v_ego);
    }
    if (v_last_target_ > v_target_) {
      accel_vel_filter_.SetState(v_target_);
    }
    accel_vel_filter_.Update(v_target_);
    v_target_ = accel_vel_filter_.GetOutput();
  } else if (v_target_ < v_ego &&
             ((is_on_ramp && v_limit_on_ramp_ == v_target_) ||
              v_limit_lc_ == v_target_ ||
              (v_limit_with_intersection_ == v_target_ && v_limit_with_intersection_ > 0.1))) {
    if (v_ego < v_last_target_) {
      accel_vel_filter_.SetState(v_ego);
    }
    accel_vel_filter_.Update(v_target_);
    v_target_ = accel_vel_filter_.GetOutput();
  } else if (v_target_ < v_ego && v_limit_on_turns_and_road_ == v_target_) {
    if (v_ego < v_last_target_) {
      accel_vel_in_turns_filter_.SetState(v_ego);
    }
    accel_vel_in_turns_filter_.Update(v_target_);
    v_target_ = accel_vel_in_turns_filter_.GetOutput();
  }

  // 0. get start & stop state
  common::StartStopInfo::StateType stop_start_state = UpdateStartStopState(
      lon_behav_input_->lat_obs_info().lead_one(), v_ego, last_traj);
  v_target_ = stop_start_state == common::StartStopInfo::STOP ? 0.0 : v_target_;
  JSON_DEBUG_VALUE("stop_start_state", (int)stop_start_state);
  JSON_DEBUG_VALUE("v_target_start_stop", v_cruise);

  v_last_target_ = v_target_;

  // 3. update vref
  UpdateVelRefs();

  // 4. update bound
  MakeAccBound();
  MakeJerkBound();

  // 5. calculate sref by vref
  std::vector<double> sref_vec;
  sref_vec.resize(config_.lon_num_step + 1);
  CalculateSrefsByVref(v_ego, vt_refs_, acc_ego, sref_vec);
  // sref_vec.clear();
  // CalculateCruiseSrefs(v_ego, v_cruise, acc_ego, sref_vec);
  // 6. update STboundaries & sref
  UpdateSTGraphs(st_infos, sref_vec);
}

bool StGraphGenerator::CalcSpeedInfoWithLead(
    const planning::common::TrackedObjectInfo &lead_one,
    const planning::common::TrackedObjectInfo &lead_two,
    const string &lc_request, const double v_ego,
    std::vector<planning::common::RealTimeLonObstacleSTInfo> &leads_st_info) {
  double lead_one_a_processed = 0.0;
  double lead_one_desired_distance = 0.0;
  double safe_distance = 0.0;
  double lead_one_desired_velocity = 40.0;
  double lead_two_a_processed = 0.0;
  double lead_two_desired_distance = 0.0;
  double lead_two_desired_velocity = 40.0;
  double desired_distance_filtered = 0.0;
  double lead_two_desired_distance_filtered = 0.0;
  std::pair<double, double> acc_target = {-0.5, 0.5};

  // 纵向只使用融合成功障碍物
  bool lead_fusion_enable = (lead_one.fusion_source() & OBSTACLE_SOURCE_CAMERA);
  LOG_DEBUG("----compute_speed_with_leads--- \n");
  if (lead_one.track_id() != 0 &&
      lead_one.type() != iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN &&
      lead_fusion_enable) {
    LOG_DEBUG("target_lead_one's id : [%i], d_rel is : [%f], v_lead is: [%f]\n",
              lead_one.track_id(), lead_one.d_rel(), lead_one.v_lead());

    lead_one_a_processed = ProcessObstacleAcc(lead_one.a_lead_k());
    safe_distance = CalcSafeDistance(lead_one.v_lead(), v_ego);
    lead_one_desired_distance =
        CalcDesiredDistance(lead_one, v_ego, lc_request);
    lead_one_desired_velocity = CalcDesiredVelocity(
        lead_one.d_rel(), lead_one_desired_distance, lead_one.v_lead());

    desired_distance_filtered = DesiredDistanceFilter(
        lead_one, v_ego, safe_distance, lead_one_desired_distance);

    // update lead one st
    common::RealTimeLonObstacleSTInfo lead_one_st_info;
    lead_one_st_info.set_st_type(common::RealTimeLonObstacleSTInfo::LEADS);
    lead_one_st_info.set_id(lead_one.track_id());
    lead_one_st_info.set_a_lead(lead_one_a_processed);
    lead_one_st_info.set_v_lead(lead_one.v_lead());
    lead_one_st_info.set_s_lead(lead_one.d_rel());
    lead_one_st_info.set_desired_distance(desired_distance_filtered);
    lead_one_st_info.set_desired_velocity(lead_one_desired_velocity);
    lead_one_st_info.set_safe_distance(safe_distance);
    lead_one_st_info.set_start_time(0.0);  // TBD:使用可配置参数
    lead_one_st_info.set_end_time(5.0);    // TBD:使用可配置参数
    lead_one_st_info.set_start_s(lead_one.d_rel());
    leads_st_info.emplace_back(lead_one_st_info);
    v_target_ = std::min(v_target_, lead_one_desired_velocity);

    JSON_DEBUG_VALUE("lead_one_id", lead_one.track_id());
    JSON_DEBUG_VALUE("lead_one_dis", lead_one.d_rel());
    JSON_DEBUG_VALUE("lead_one_vel", lead_one.v_lead());
    JSON_DEBUG_VALUE("v_target_lead_one", lead_one_desired_velocity);
    JSON_DEBUG_VALUE("desired_distance_lead_one", desired_distance_filtered);

    // 对lead two进行类似的计算
    bool is_camera_and_lidar =
        lead_two.fusion_source() == OBSTACLE_SOURCE_F_RADAR_CAMERA;
    if (config_.enable_lead_two && lead_two.track_id() != 0 &&
        lead_two.type() != iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN) {
      LOG_DEBUG(
          "target_lead_two's id : [%i], d_rel is : [%f], v_lead is: [%f]\n",
          lead_two.track_id(), lead_two.d_rel(), lead_two.v_lead());

      lead_two_a_processed = ProcessObstacleAcc(lead_two.a_lead_k());
      safe_distance = CalcSafeDistance(lead_two.v_lead(), v_ego);
      lead_two_desired_distance =
          CalcDesiredDistance(lead_two, v_ego, lc_request);
      // leave enough space for leadOne
      lead_two_desired_distance += 7.0;  // 物理含义注明
      lead_two_desired_velocity = CalcDesiredVelocity(
          lead_two.d_rel(), lead_two_desired_distance, lead_two.v_lead());

      lead_two_desired_distance_filtered = LeadtwoDesiredDistanceFilter(
          lead_two, v_ego, safe_distance, lead_two_desired_distance);

      // update lead two st
      planning::common::RealTimeLonObstacleSTInfo lead_two_st_info;
      lead_two_st_info.set_st_type(common::RealTimeLonObstacleSTInfo::LEADS);
      lead_two_st_info.set_id(lead_two.track_id());
      lead_two_st_info.set_a_lead(lead_two_a_processed);
      lead_two_st_info.set_v_lead(lead_two.v_lead());
      lead_two_st_info.set_s_lead(lead_two.d_rel());
      lead_two_st_info.set_desired_distance(lead_two_desired_distance_filtered);
      lead_two_st_info.set_desired_velocity(lead_two_desired_velocity);
      lead_two_st_info.set_safe_distance(safe_distance);
      lead_two_st_info.set_start_time(0.0);  // TBD:使用可配置参数
      lead_two_st_info.set_end_time(5.0);    // TBD:使用可配置参数
      lead_two_st_info.set_start_s(lead_two.d_rel());
      leads_st_info.emplace_back(lead_two_st_info);

      v_target_ = std::min(v_target_, lead_two_desired_velocity);

      if (lead_two_desired_velocity < lead_one_desired_velocity) {
        CalcAccLimits(lead_two, lead_two_desired_distance,
                      lead_two_desired_velocity, v_ego, lead_two_a_processed,
                      acc_target);
        acc_target_.first = std::min(acc_target.first, acc_target.first);
        acc_target_.second = std::min(acc_target.second, acc_target.second);
      }
      JSON_DEBUG_VALUE("lead_two_id", lead_two.track_id());
      JSON_DEBUG_VALUE("lead_two_dis", lead_two.d_rel());
      JSON_DEBUG_VALUE("lead_two_vel", lead_two.v_lead());
      JSON_DEBUG_VALUE("v_target_lead_two", lead_two_desired_velocity);
    }

    // calcuate acc
    CalcAccLimits(lead_one, lead_one_desired_distance,
                  lead_one_desired_velocity, v_ego, lead_one_a_processed,
                  acc_target);
    acc_target_.first = std::min(acc_target_.first, acc_target.first);
    acc_target_.second = std::min(acc_target_.second, acc_target.second);

    JSON_DEBUG_VALUE("acc_cipv", lead_one.a_lead_k());
    JSON_DEBUG_VALUE("acc_target_high", acc_target_.second);
    JSON_DEBUG_VALUE("acc_target_low", acc_target_.first);

  } else {
    LOG_DEBUG("There is no lead \n");
    lon_behav_input_->mutable_lon_decision_info()
        ->mutable_leadone_info()
        ->set_has_leadone(false);

    JSON_DEBUG_VALUE("lead_one_id", 0);
    JSON_DEBUG_VALUE("lead_one_dis", 0);
    JSON_DEBUG_VALUE("lead_one_vel", 0);
    JSON_DEBUG_VALUE("v_target_lead_one", 0);
    JSON_DEBUG_VALUE("lead_two_id", 0);
    JSON_DEBUG_VALUE("lead_two_dis", 0);
    JSON_DEBUG_VALUE("lead_two_vel", 0);
    JSON_DEBUG_VALUE("v_target_lead_two", 0);
    JSON_DEBUG_VALUE("acc_cipv", 0.0);
    JSON_DEBUG_VALUE("acc_target_high", acc_target_.second);
    JSON_DEBUG_VALUE("acc_target_low", acc_target_.first);
  }
  return true;
}

// 需要和CalcSpeedInfoWithLead进行整合
bool StGraphGenerator::CalcSpeedInfoWithTempLead(
    const planning::common::TrackedObjectInfo &temp_lead_one,
    const planning::common::TrackedObjectInfo &temp_lead_two, double v_ego,
    const planning::common::LatOutputInfo &lateral_outputs,
    std::vector<planning::common::RealTimeLonObstacleSTInfo>
        &temp_leads_st_info) {
  double temp_lead_one_a_processed = 0.0;
  double temp_lead_one_desired_distance = 0.0;
  double temp_desired_distance_filtered = 0.0;
  double safe_distance = 0.0;
  double temp_lead_one_desired_velocity = 40.0;
  double temp_lead_two_a_processed = 0.0;
  double temp_lead_two_desired_distance = 0.0;
  double temp_lead_two_desired_velocity = 40.0;

  LOG_DEBUG("----CalcSpeedInfoWithTempLead--- \n");
  // temp leadone
  if (temp_lead_one.track_id() != 0 && !lateral_outputs.close_to_accident() &&
      (temp_lead_one.d_path_self() + std::min(temp_lead_one.v_lat(), 0.3)) <
          1.0 &&
      temp_lead_one.type() != iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN) {
    LOG_DEBUG("temp_lead_one's id : [%i], d_rel is : [%f], v_lead is: [%f]\n ",
              temp_lead_one.track_id(), temp_lead_one.d_rel(),
              temp_lead_one.v_lead());
    // process noisy a_lead signal from radar processing
    temp_lead_one_a_processed = ProcessObstacleAcc(temp_lead_one.a_lead_k());
    safe_distance = CalcSafeDistance(temp_lead_one.v_lead(), v_ego);
    // compute desired distance
    temp_lead_one_desired_distance =
        CalcDesiredDistance(temp_lead_one, v_ego, lateral_outputs.lc_request());
    // compute desired speed
    temp_lead_one_desired_velocity = CalcDesiredVelocity(
        temp_lead_one.d_rel(), temp_lead_one_desired_distance,
        temp_lead_one.v_lead());
    temp_desired_distance_filtered = TmpLeadDesiredDistanceFilter(
        temp_lead_one, v_ego, safe_distance, temp_lead_one_desired_distance);
    // update lead one st
    common::RealTimeLonObstacleSTInfo temp_lead_one_st_info;
    temp_lead_one_st_info.set_st_type(
        common::RealTimeLonObstacleSTInfo::TEMP_LEADS);
    temp_lead_one_st_info.set_decision(
        common::RealTimeLonObstacleSTInfo::YIELD);
    temp_lead_one_st_info.set_id(temp_lead_one.track_id());
    temp_lead_one_st_info.set_a_lead(temp_lead_one_a_processed);
    temp_lead_one_st_info.set_v_lead(temp_lead_one.v_lead());
    temp_lead_one_st_info.set_s_lead(temp_lead_one.d_rel());
    temp_lead_one_st_info.set_desired_distance(temp_desired_distance_filtered);
    temp_lead_one_st_info.set_desired_velocity(temp_lead_one_desired_velocity);
    temp_lead_one_st_info.set_safe_distance(safe_distance);
    temp_lead_one_st_info.set_start_time(0.0);  // TBD:使用可配置参数
    temp_lead_one_st_info.set_end_time(5.0);    // TBD:使用可配置参数
    temp_lead_one_st_info.set_start_s(temp_lead_one.d_rel());
    temp_leads_st_info.emplace_back(temp_lead_one_st_info);
    v_target_ = std::min(v_target_, temp_lead_one_desired_velocity);

    JSON_DEBUG_VALUE("temp_lead_one_id", temp_lead_one.track_id());
    JSON_DEBUG_VALUE("temp_lead_one_dis", temp_lead_one.d_rel());
    JSON_DEBUG_VALUE("temp_lead_one_vel", temp_lead_one.v_lead());
    JSON_DEBUG_VALUE("v_target_temp_lead_one", temp_lead_one_desired_velocity);

    // 对lead two进行类似的计算
    bool is_camera_and_lidar =
        temp_lead_two.fusion_source() == OBSTACLE_SOURCE_F_RADAR_CAMERA;
    if (config_.enable_lead_two && temp_lead_two.track_id() != 0 &&
        temp_lead_two.type() != iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN) {
      LOG_DEBUG(
          "target_temp_lead_two's id : [%i], d_rel is : [%f], v_lead is: "
          "[%f]\n",
          temp_lead_two.track_id(), temp_lead_two.d_rel(),
          temp_lead_two.v_lead());
      temp_lead_two_a_processed = ProcessObstacleAcc(temp_lead_two.a_lead_k());
      safe_distance = CalcSafeDistance(temp_lead_two.v_lead(), v_ego);
      temp_lead_two_desired_distance = CalcDesiredDistance(
          temp_lead_two, v_ego, lateral_outputs.lc_request());
      // leave enough space for leadOne
      temp_lead_two_desired_distance += 7.0;
      temp_lead_two_desired_velocity = CalcDesiredVelocity(
          temp_lead_two.d_rel(), temp_lead_two_desired_distance,
          temp_lead_two.v_lead());

      // update lead two st
      planning::common::RealTimeLonObstacleSTInfo temp_lead_two_st_info;
      temp_lead_two_st_info.set_st_type(
          common::RealTimeLonObstacleSTInfo::TEMP_LEADS);
      temp_lead_two_st_info.set_id(temp_lead_two.track_id());
      temp_lead_two_st_info.set_a_lead(temp_lead_two_a_processed);
      temp_lead_two_st_info.set_v_lead(temp_lead_two.v_lead());
      temp_lead_two_st_info.set_s_lead(temp_lead_two.d_rel());
      temp_lead_two_st_info.set_desired_distance(
          temp_lead_two_desired_distance);
      temp_lead_two_st_info.set_desired_velocity(
          temp_lead_two_desired_velocity);
      temp_lead_two_st_info.set_safe_distance(safe_distance);
      temp_lead_two_st_info.set_start_time(0.0);  // TBD:使用可配置参数
      temp_lead_two_st_info.set_end_time(5.0);    // TBD:使用可配置参数
      temp_lead_two_st_info.set_start_s(temp_lead_two.d_rel());
      temp_leads_st_info.emplace_back(temp_lead_two_st_info);
      v_target_ = std::min(v_target_, temp_lead_two_desired_velocity);

      JSON_DEBUG_VALUE("temp_lead_two_id", temp_lead_two.track_id());
      JSON_DEBUG_VALUE("temp_lead_two_dis", temp_lead_two.d_rel());
      JSON_DEBUG_VALUE("temp_lead_two_vel", temp_lead_two.v_lead());
      JSON_DEBUG_VALUE("v_target_temp_lead_two",
                       temp_lead_two_desired_velocity);
    }
  } else {
    LOG_DEBUG("There is no temp lead \n");

    JSON_DEBUG_VALUE("temp_lead_one_id", 0);
    JSON_DEBUG_VALUE("temp_lead_one_dis", 0);
    JSON_DEBUG_VALUE("temp_lead_one_vel", 0);
    JSON_DEBUG_VALUE("v_target_temp_lead_one", 0);
    JSON_DEBUG_VALUE("temp_lead_two_id", 0);
    JSON_DEBUG_VALUE("temp_lead_two_dis", 0);
    JSON_DEBUG_VALUE("temp_lead_two_vel", 0);
    JSON_DEBUG_VALUE("v_target_temp_lead_two", 0);
  }
  return true;
}

void StGraphGenerator::CalcSpeedInfoWithCutin(
    const planning::common::LatObsInfo &lateral_obstacles,
    const std::string &lc_request, double v_cruise, double v_ego,
    std::vector<planning::common::RealTimeLonObstacleSTInfo> &cut_in_st_info) {
  // 更新周围障碍物的cut in信息，纵向评估的cut in,不用做计算
  UpdateNearObstacles(lateral_obstacles, lc_request, v_ego);

  UpdateSpeedWithPotentialCutinCar(lateral_obstacles, lc_request, v_cruise,
                                   v_ego, cut_in_st_info);
}

bool StGraphGenerator::CalcCruiseAccelLimits(const double v_ego) {
  acc_target_.first = interp(v_ego, _A_CRUISE_MIN_BP, _A_CRUISE_MIN_V);
  acc_target_.second = interp(v_ego, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V);
  LOG_DEBUG("----CalcCruiseAccelLimits--- \n");
  LOG_DEBUG("acc_target_.first : %f ,acc_target_.first : %f\n",
            acc_target_.first, acc_target_.second);
  return true;
}

bool StGraphGenerator::CalcSpeedWithTurns(const double v_ego,
                                          const double angle_steers,
                                          const std::vector<double> &d_poly) {
  // *** this function returns a limited long acceleration allowed, depending on
  // the existing lateral acceleration
  //  this should avoid accelerating when losing the target in turns
  LOG_DEBUG("----CalcSpeedWithTurns--- \n");
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double steer_ratio = vehicle_param.steer_ratio;
  double wheel_base = vehicle_param.wheel_base;

  double acc_target_in_turns = 0.0;
  double angle_steers_deg = angle_steers * DEG_PER_RAD;

  double acc_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V);
  double acc_lat =
      std::pow(v_ego, 2) * angle_steers / (steer_ratio * wheel_base);
  double acc_lon_allowed = std::sqrt(
      std::max(std::pow(acc_total_max, 2) - std::pow(acc_lat, 2), 0.0));

  // And limit the logitudinal velocity for a safe turn
  double acc_lat_max =
      interp(std::abs(angle_steers_deg), _AY_MAX_ABS_BP, _AY_MAX_STEERS);
  double v_limit_steering = std::sqrt((acc_lat_max * steer_ratio * wheel_base) /
                                      std::max(std::abs(angle_steers), 0.001));
  double v_limit_in_turns = v_limit_steering;
  // calculate the velocity limit according to the road curvature
  if (d_poly.size() == 4) {
    double preview_x = config_.dis_curv + config_.t_curv * v_ego;
    double curv =
        std::fabs(2 * d_poly[0] * preview_x + d_poly[1]) /
        std::pow(std::pow(2 * d_poly[0] * preview_x + d_poly[1], 2) + 1, 1.5);
    double road_radius = 1 / std::max(curv, 0.0001);
    if (road_radius < 750) {
      acc_lat_max = interp(road_radius, _AY_MAX_CURV_BP, _AY_MAX_CURV_V);
    }
    double v_limit_road = std::sqrt(acc_lat_max * road_radius) * 0.9;
    v_limit_in_turns = std::min(v_limit_in_turns, v_limit_road);
    LOG_DEBUG("road_radius is : [%f], acc_lat_max: [%f]\n", road_radius,
              acc_lat_max);
    LOG_DEBUG(
        "angle_steers: [%f], angle_steers_deg: [%f], v_limit_road: [%f]\n",
        angle_steers, angle_steers_deg, v_limit_road);
    JSON_DEBUG_VALUE("v_limit_road", v_limit_road);
    JSON_DEBUG_VALUE("road_radius", road_radius);
  }

  JSON_DEBUG_VALUE("v_limit_steering", v_limit_steering);
  JSON_DEBUG_VALUE("v_limit_in_turns", v_limit_in_turns);
  JSON_DEBUG_VALUE("ego_acc_lat", acc_lat);

  if (v_limit_in_turns < v_ego - 2) {
    acc_target_in_turns = -0.2;
  }

  acc_target_.first = std::min(acc_target_.first, acc_target_in_turns);
  acc_target_.second = std::min(acc_target_.second, acc_lon_allowed);
  v_limit_on_turns_and_road_ = v_limit_in_turns;
  v_target_ = std::min(v_target_, v_limit_in_turns);

  LOG_DEBUG("v_target_ : [%f] \n", v_target_);
  return true;
}

bool StGraphGenerator::CalcSpeedWithRamp(double dis_to_ramp,
                                         double dis_to_merge, bool is_on_ramp,
                                         bool is_continuous_ramp,
                                         double ramp_v_limit,
                                         double acc_to_ramp, double v_ego) {
  LOG_DEBUG("----calc_speed_for_ramp--- \n");
  auto ref_path_points = lon_behav_input_->ref_path_points();
  double v_target_ramp = 40;
  double v_target_near_ramp_zone = 40;
  // 通过接口获取是否在匝道的信息
  if (is_on_ramp) {
    if (dis_to_merge > 50 || is_continuous_ramp) {
      v_target_ramp = ramp_v_limit;
    }
    v_target_ = std::min(v_target_ramp, v_target_);
    v_limit_on_ramp_ = v_target_ramp;
    LOG_DEBUG("v_target_ramp : [%f] \n", v_target_ramp);
    JSON_DEBUG_VALUE("v_target_ramp", v_target_ramp);
    JSON_DEBUG_VALUE("dis_to_ramp", dis_to_ramp);
    JSON_DEBUG_VALUE("dis_to_merge", dis_to_merge);
    LOG_DEBUG("v_target : [%f] \n", v_target_);
    return true;
  }
  if (dis_to_ramp <= config_.dis_near_ramp_zone) {
    double pre_brake_dis_near_ramp_zone = std::max(dis_to_ramp - config_.brake_dis_near_ramp_zone, 0.0);
    v_target_near_ramp_zone = std::pow(
      std::pow(config_.v_limit_near_ramp_zone, 2.0) - 2 * pre_brake_dis_near_ramp_zone * acc_to_ramp,
      0.5);
  }
  double pre_brake_dis_to_ramp = std::max(dis_to_ramp - 50, 0.0);
  v_target_ramp = std::pow(
      std::pow(ramp_v_limit, 2.0) - 2 * pre_brake_dis_to_ramp * acc_to_ramp,
      0.5);
  v_target_ramp = std::min(v_target_near_ramp_zone, v_target_ramp);
  v_limit_on_ramp_ = v_target_ramp;
  v_target_ = std::min(v_target_ramp, v_target_);
  LOG_DEBUG("dis_to_ramp : [%f] \n", dis_to_ramp);
  LOG_DEBUG("v_target_ramp : [%f] \n", v_target_ramp);
  JSON_DEBUG_VALUE("v_target_ramp", v_target_ramp);
  JSON_DEBUG_VALUE("dis_to_ramp", dis_to_ramp);
  JSON_DEBUG_VALUE("dis_to_merge", dis_to_merge);
  LOG_DEBUG("v_target : [%f] \n", v_target_);
  return true;
}

void StGraphGenerator::UpdateSTRefs(const std::vector<double> &sref_vec) {
  st_refs_ = sref_vec;
}

void StGraphGenerator::UpdateSTGraphs(
    const std::vector<common::RealTimeLonObstacleSTInfo> &st_infos,
    const std::vector<double> &sref_vec) {
  const double soft_bound_corridor_t = config_.soft_bound_corridor_t;
  // local variables
  double sample_time = 0;
  double t = config_.delta_time;
  double t_square = config_.delta_time * config_.delta_time;
  double t_cube = config_.delta_time * config_.delta_time * config_.delta_time;
  // TODO: 后续取参考线的长度为s bound upper
  constexpr double s_upper_bound = 200.0;
  LonBound soft_bound;
  LonBound hard_bound;
  double s_ref;
  double s_ref_update;
  std::vector<double> sref_update;
  // 稳态跟车过程中soft bound和s ref之间增加一个buffer来补偿控制误差
  double static_soft_bound_buffer = 0.5;

  // sref_update.reserve(config_.lon_num_step+1);
  sref_update = sref_vec;
  st_boundaries_.reserve(st_infos.size());
  for (auto &st : st_infos) {
    // 1.设置boundary类型，是否必要？
    scc::STBoundary st_boundary;
    st_boundary.id = st.id();
    switch (st.st_type()) {
      case common::RealTimeLonObstacleSTInfo::LEADS:
        st_boundary.boundary_type = scc::BoundaryType::YIELD;
        break;
      case common::RealTimeLonObstacleSTInfo::TEMP_LEADS:
        st_boundary.boundary_type = scc::BoundaryType::YIELD;
        break;
      case common::RealTimeLonObstacleSTInfo::CUT_IN:
        st_boundary.boundary_type = scc::BoundaryType::YIELD;
        break;
      case common::RealTimeLonObstacleSTInfo::GAP:
        st_boundary.boundary_type =
            st.decision() == common::RealTimeLonObstacleSTInfo::YIELD
                ? scc::BoundaryType::YIELD
                : scc::BoundaryType::OVERTAKE;
        break;
      default:
        st_boundary.boundary_type = scc::BoundaryType::UNKNOWN;
    }

    double s_step = 0.0;
    double st_obs_v = st.v_lead();
    double st_obs_a = st.a_lead();
    double st_obs_j = 3.0;  //_J_MAX
    // 2.将st信息转换为离散bounds
    for (unsigned int i = 0; i <= config_.lon_num_step; i++) {
      sample_time = i * t;
      // 考虑前车减速的情况
      if (st.a_lead() < 0) {
        // s_step += CalcDeceleratedObstacleST();
        s_step += std::max(st_obs_v * t + 0.5 * st_obs_a * t_square +
                               1.0 / 6 * st_obs_j * t_cube,
                           0.0);
        st_obs_v =
            std::max(st_obs_v + st_obs_a * t + 0.5 * st_obs_j * t_square, 0.0);
        st_obs_a = std::min(st_obs_a + st_obs_j * t, 0.0);
        if (st_obs_a == 0.0) {
          st_obs_j = 0.0;
        }
      } else {
        s_step += st.v_lead() * t;
      }
      // 只更新关注的t区间内
      if (sample_time >= st.start_time() && sample_time <= st.end_time()) {
        // 考虑decision type是overtake的情况
        if (st.decision() == common::RealTimeLonObstacleSTInfo::YIELD) {
          // 没必要区分
          /*
          if (st.st_type() == common::RealTimeLonObstacleSTInfo::GAP) {
            s_ref = st.start_s() - st.desired_distance() +
                    st.desired_velocity() * sample_time;
          } else {
            s_ref = st.start_s() - st.desired_distance() + s_step;
          }
          */
          s_ref = st.start_s() - st.desired_distance() + s_step;
          // hard bound使用安全距离
          hard_bound.upper =
              std::max(st.start_s() - st.safe_distance() + s_step, 0.1);
          hard_bound.lower = 0.0;  // 应该至少使用自车s-10
          hard_bound.vel = st.v_lead();
          hard_bound.acc = st.a_lead();
          hard_bound.id = st.id();
          st_boundary.hard_bound.emplace_back(hard_bound);
          s_ref_update = std::min(
              hard_bound.upper, std::min(sref_update[i], std::max(s_ref, 0.0)));
          soft_bound.upper =
              std::min(0.5 * (hard_bound.upper + s_ref_update),
                       std::max(st.start_s() - st.desired_distance() + s_step +
                                    static_soft_bound_buffer,
                                0.0));
          soft_bound.lower = 0.0;  // 应该至少使用自车s-10
          soft_bound.vel = st.v_lead();
          soft_bound.acc = st.a_lead();
          st_boundary.soft_bound.emplace_back(soft_bound);
          // 根据障碍物跟车距离刷新s_refs
          sref_update[i] = s_ref_update;
        } else {
          s_ref = st.start_s() + st.desired_distance() + s_step;
          // hard bound使用安全距离
          hard_bound.upper = s_upper_bound;
          hard_bound.lower =
              std::max(st.start_s() + st.safe_distance() + s_step, 0.0);
          hard_bound.vel = st.v_lead();
          hard_bound.acc = st.a_lead();
          hard_bound.id = st.id();
          st_boundary.hard_bound.emplace_back(hard_bound);
          s_ref_update = std::max(
              hard_bound.lower, s_ref_update = std::max(sref_update[i], s_ref));
          // soft bound先使用期望跟车距离+buffer
          soft_bound.upper = s_upper_bound;
          soft_bound.lower =
              std::max(0.5 * (hard_bound.lower + s_ref_update), s_ref);
          soft_bound.vel = st.v_lead();
          soft_bound.acc = st.a_lead();
          st_boundary.soft_bound.emplace_back(soft_bound);
          // 根据障碍物跟车距离刷新s_refs
          sref_update[i] = s_ref_update;
        }
      } else {
        // 采用默认值,这个目前太过粗暴
        hard_bound.upper = s_upper_bound;
        hard_bound.lower = -10.0;
        hard_bound.vel = 0.0;
        hard_bound.acc = 0.0;
        hard_bound.id = 0.0;
        st_boundary.hard_bound.emplace_back(hard_bound);
        soft_bound.upper = s_upper_bound;
        soft_bound.lower = -10.0;  // 应该至少使用自车s-10
        soft_bound.vel = 0.0;
        soft_bound.acc = 0.0;
        st_boundary.soft_bound.emplace_back(soft_bound);
      }
    }
    st_boundaries_.emplace_back(st_boundary);
  }
  st_refs_ = sref_update;
}

void StGraphGenerator::UpdateNearObstacles(
    const planning::common::LatObsInfo &lateral_obstacles,
    const string &lc_request, double v_ego) {
  const double safety_distance = 2.0 + v_ego * 0.2;
  int cutin_status = 0;

  std::vector<const planning::common::TrackedObjectInfo *> near_cars,
      near_cars_sorted;
  std::array<int, 3> nearest_car_track_id{0, 0, 0};
  std::array<double, 3> v_limit_cutin{40.0, 40.0, 40.0};
  std::array<double, 3> a_limit_cutin{0.0, 0.0, 0.0};
  std::array<std::string, 3> cutin_condition{"", "", ""};

  LOG_DEBUG("----limit_accel_velocity_for_cutin--- \n");
  // filter near cars from front && side tracks
  near_cars.clear();
  for (auto &track : lateral_obstacles.front_tracks()) {
    // ignore obj without camera source
    if ((track.fusion_source() & OBSTACLE_SOURCE_CAMERA) == 0) {
      continue;
    };
    if (std::abs(track.y_rel()) < 10.0 && std::abs(track.d_rel()) < 20.0 &&
        track.type() != iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN) {
      near_cars.push_back(&track);
    }
  }

  for (auto &track : lateral_obstacles.side_tracks()) {
    // ignore obj without camera source
    if ((track.fusion_source() & OBSTACLE_SOURCE_CAMERA) == 0) {
      continue;
    };
    if (std::abs(track.y_rel()) < 10.0 && std::abs(track.d_rel()) < 20.0 &&
        track.type() != iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN) {
      near_cars.push_back(&track);
    }
  }

  // sort by abs(y_min) from min to max
  std::sort(near_cars.begin(), near_cars.end(),
            [](const planning::common::TrackedObjectInfo *a,
               const planning::common::TrackedObjectInfo *b) {
              return std::abs(a->y_min()) < std::abs(b->y_min());
            });

  // take at most 3 nearest car
  near_cars_sorted.clear();
  for (int i = 0; i < near_cars.size() && near_cars_sorted.size() < 3; i++) {
    if ((near_cars[i]->location_tail() < 2 * safety_distance) &&
        (near_cars[i]->location_tail() > -safety_distance - 5))
      near_cars_sorted.push_back(near_cars[i]);
  }

  for (int i = 0; i < near_cars_sorted.size(); i++) {
    if (lon_behav_input_->lat_obs_info().lead_one().track_id() ==
        near_cars_sorted[i]->track_id()) {
      continue;
    }
    nearest_car_track_id[i] = near_cars_sorted[i]->track_id();
    double car_length = near_cars_sorted[i]->location_head() -
                        near_cars_sorted[i]->location_tail();
    double d_x_offset = std::max(car_length - 5.0, 0.0);

    // calculate y_rel for ttc
    double y_rel = 0.0;
    if (near_cars_sorted[i]->y_x0() != 0.0) {
      y_rel = std::abs(std::abs(near_cars_sorted[i]->y_x0()) - 1.1);
    } else if (near_cars_sorted[i]->location_tail() < 0.0) {
      y_rel = std::abs(std::abs(near_cars_sorted[i]->y_min()) - 1.1);
    } else {
      double y_thres = interp(near_cars_sorted[i]->v_rel(), _Y_THRES_SPEED_BP,
                              _Y_THRES_SPEED_V);
      y_rel = std::abs(std::abs(near_cars_sorted[i]->y_min()) - y_thres);
    }

    double v_rel = near_cars_sorted[i]->v_rel() - 0.5;
    double vy_rel = near_cars_sorted[i]->vy_rel();
    double d_offset = 2.0 + 0.1 * v_ego;
    double v_coeff = interp(std::abs(near_cars_sorted[i]->y_min()),
                            _CUT_IN_COEFF_BP, _CUT_IN_COEFF_V);
    bool cutin_valid = true;

    // check cutin car
    bool cutin_car = false;
    bool potential_cutin_car_1 = false;
    bool potential_cutin_car_2 = false;
    bool NEAR_CAR_LAT_MOVING = fabs(vy_rel) > 0.1;
    if (near_cars_sorted[i]->y_min() < 0) {
      if (lc_request == "left_lane_change") {
        vy_rel = std::max(vy_rel, (vy_rel + near_cars_sorted[i]->v_lat()) / 2);
      }
      if (lc_request == "right_lane_change_wait") {
        if (NEAR_CAR_LAT_MOVING) {
          cutin_car =
              (near_cars_sorted[i]->y_min() + vy_rel * v_coeff >= -CUIIN_WIDTH);
          potential_cutin_car_1 =
              (near_cars_sorted[i]->y_min() + 1.25 * vy_rel * v_coeff >=
               -CUIIN_WIDTH);
          potential_cutin_car_2 =
              (near_cars_sorted[i]->y_min() + 1.5 * vy_rel * v_coeff >=
               -CUIIN_WIDTH);
        } else {
          cutin_car = (near_cars_sorted[i]->y_min() + vy_rel * v_coeff >=
                       -CUIIN_WIDTH_STATIC);
          potential_cutin_car_1 =
              (near_cars_sorted[i]->y_min() + 1.25 * vy_rel * v_coeff >=
               -CUIIN_WIDTH_STATIC);
          potential_cutin_car_2 =
              (near_cars_sorted[i]->y_min() + 1.5 * vy_rel * v_coeff >=
               -CUIIN_WIDTH_STATIC);
        }
      } else {
        if (NEAR_CAR_LAT_MOVING) {
          cutin_car =
              (near_cars_sorted[i]->y_min() + vy_rel * v_coeff >= -CUIIN_WIDTH);
          potential_cutin_car_1 =
              (near_cars_sorted[i]->y_min() + 1.5 * vy_rel * v_coeff >=
               -1 * (CUIIN_WIDTH + 0.01 * v_ego));
          potential_cutin_car_2 =
              (near_cars_sorted[i]->y_min() + 2.0 * vy_rel * v_coeff >=
               -1 * (CUIIN_WIDTH + 0.01 * v_ego));
        } else {
          cutin_car = (near_cars_sorted[i]->y_min() + vy_rel * v_coeff >=
                       -CUIIN_WIDTH_STATIC);
          potential_cutin_car_1 =
              (near_cars_sorted[i]->y_min() + 1.5 * vy_rel * v_coeff >=
               -1 * (CUIIN_WIDTH_STATIC + 0.01 * v_ego));
          potential_cutin_car_2 =
              (near_cars_sorted[i]->y_min() + 2.0 * vy_rel * v_coeff >=
               -1 * (CUIIN_WIDTH_STATIC + 0.01 * v_ego));
        }
      }
    } else {
      if (lc_request == "right_lane_change") {
        vy_rel = min(vy_rel, (vy_rel + near_cars_sorted[i]->v_lat()) / 2);
      }
      if (lc_request == "left_lane_change_wait") {
        if (NEAR_CAR_LAT_MOVING) {
          cutin_car =
              (near_cars_sorted[i]->y_min() + vy_rel * v_coeff <= CUIIN_WIDTH);
          potential_cutin_car_1 =
              (near_cars_sorted[i]->y_min() + 1.25 * vy_rel * v_coeff <=
               CUIIN_WIDTH);
          potential_cutin_car_2 =
              (near_cars_sorted[i]->y_min() + 1.5 * vy_rel * v_coeff <=
               CUIIN_WIDTH);
        } else {
          cutin_car = (near_cars_sorted[i]->y_min() + vy_rel * v_coeff <=
                       CUIIN_WIDTH_STATIC);
          potential_cutin_car_1 =
              (near_cars_sorted[i]->y_min() + 1.25 * vy_rel * v_coeff <=
               CUIIN_WIDTH_STATIC);
          potential_cutin_car_2 =
              (near_cars_sorted[i]->y_min() + 1.5 * vy_rel * v_coeff <=
               CUIIN_WIDTH_STATIC);
        }
      } else {
        if (NEAR_CAR_LAT_MOVING) {
          cutin_car =
              (near_cars_sorted[i]->y_min() + vy_rel * v_coeff <= CUIIN_WIDTH);
          potential_cutin_car_1 =
              (near_cars_sorted[i]->y_min() + 1.5 * vy_rel * v_coeff <=
               (CUIIN_WIDTH + 0.01 * v_ego));
          potential_cutin_car_2 =
              (near_cars_sorted[i]->y_min() + 2.0 * vy_rel * v_coeff <=
               (CUIIN_WIDTH + 0.01 * v_ego));
        } else {
          cutin_car = (near_cars_sorted[i]->y_min() + vy_rel * v_coeff <=
                       CUIIN_WIDTH_STATIC);
          potential_cutin_car_1 =
              (near_cars_sorted[i]->y_min() + 1.5 * vy_rel * v_coeff <=
               (CUIIN_WIDTH_STATIC + 0.01 * v_ego));
          potential_cutin_car_2 =
              (near_cars_sorted[i]->y_min() + 2.0 * vy_rel * v_coeff <=
               (CUIIN_WIDTH_STATIC + 0.01 * v_ego));
        }
      }
    }

    // calculate ttc
    vy_rel = near_cars_sorted[i]->y_min() > 0 ? vy_rel : -vy_rel;
    double ttc =
        std::max(y_rel / std::max(std::abs(std::min(vy_rel, 0.0)), 0.01), 0.2);
    // double ttc_org = ttc;
    bool lead_car = std::abs(near_cars_sorted[i]->y_min()) < 0.8;

    // fast cutin car
    double d_thres =
        interp(std::abs(vy_rel), _D_THRES_SPEED_BP, _D_THRES_SPEED_V);
    if (near_cars_sorted[i]->location_tail() < 0.0 &&
        near_cars_sorted[i]->location_tail() > -3.0 &&
        near_cars_sorted[i]->v_rel() > 2.0) {
      double predict_d_x = near_cars_sorted[i]->v_rel() * ttc +
                           near_cars_sorted[i]->location_tail();
      if (predict_d_x > d_thres) {
        ttc = std::max(std::abs(near_cars_sorted[i]->y_min()) /
                           std::max(std::abs(std::min(vy_rel, 0.0)), 0.01),
                       0.2);
      }
    }

    // calculate v(a)_limit_cutin
    if (cutin_valid &&
        ((0.8 <= std::abs(near_cars_sorted[i]->y_min())) &&
         (std::abs(near_cars_sorted[i]->y_min()) <= 2.5)) &&
        cutin_car &&
        ((-3.5 - d_x_offset - min(near_cars_sorted[i]->v_rel(), 1.0) <
          near_cars_sorted[i]->location_tail()) &&
         (near_cars_sorted[i]->location_tail() < safety_distance + 2))) {
      v_limit_cutin[i] =
          v_ego + v_rel -
          ((-(near_cars_sorted[i]->location_tail()) + safety_distance) / ttc);
      a_limit_cutin[i] =
          std::min(-0.5 + 2 *
                              (near_cars_sorted[i]->location_tail() +
                               near_cars_sorted[i]->v_rel() * ttc - d_offset) /
                              std::pow(ttc, 2),
                   a_limit_cutin[i] + 0.1);
      cutin_condition[i] = "cutin_car, stage 1";
    } else if (cutin_valid &&
               ((0.8 <= std::abs(near_cars_sorted[i]->y_min())) &&
                (std::abs(near_cars_sorted[i]->y_min()) <= 2.1)) &&
               cutin_car &&
               ((-6.0 - d_x_offset < near_cars_sorted[i]->location_tail()) &&
                (near_cars_sorted[i]->location_tail() < -3.5)) &&
               (near_cars_sorted[i]->v_rel() > 0.5) && (v_ego > 2.0)) {
      v_limit_cutin[i] =
          v_ego + v_rel -
          ((-(near_cars_sorted[i]->location_tail()) + safety_distance) / ttc);
      a_limit_cutin[i] =
          std::min(-0.5 + 2 *
                              (near_cars_sorted[i]->location_tail() +
                               near_cars_sorted[i]->v_rel() * ttc - d_offset) /
                              std::pow(ttc, 2),
                   a_limit_cutin[i] + 0.1);
      cutin_condition[i] = "cutin_car, stage 2";
    } else if (cutin_valid &&
               ((0.8 <= std::abs(near_cars_sorted[i]->y_min())) &&
                (std::abs(near_cars_sorted[i]->y_min()) <= 3.0)) &&
               potential_cutin_car_1 &&
               ((-3.5 - d_x_offset < near_cars_sorted[i]->location_tail()) &&
                (near_cars_sorted[i]->location_tail() < safety_distance))) {
      v_limit_cutin[i] =
          near_cars_sorted[i]->location_tail() < 0
              ? std::max(v_ego - 1, p1min_speed)
              : std::max({v_ego - 1, v_ego + v_rel - 1, p1min_speed});
      a_limit_cutin[i] =
          std::max(std::min(a_limit_cutin[i], -0.6) - 0.002, -0.8);
      cutin_condition[i] = "potential_cutin_car_1, stage 1";
      if (std::abs(near_cars_sorted[i]->y_min()) < 1.7 ||
          std::abs(vy_rel) > 0.7) {
        v_limit_cutin[i] = std::max(
            {std::min(v_ego + v_rel - 1, v_ego - 1), v_ego - 3, p1min_speed});
        a_limit_cutin[i] = -1.0;
        cutin_condition[i] = "potential_cutin_car_1, stage 1.5";
      }
    } else if (cutin_valid &&
               ((0.8 <= std::abs(near_cars_sorted[i]->y_min())) &&
                (std::abs(near_cars_sorted[i]->y_min()) <= 3.0)) &&
               potential_cutin_car_2 &&
               ((-3.5 - d_x_offset < near_cars_sorted[i]->location_tail()) &&
                (near_cars_sorted[i]->location_tail() < safety_distance))) {
      v_limit_cutin[i] =
          near_cars_sorted[i]->location_tail() < 0
              ? std::max(v_ego - 0.1, p2min_speed)
              : std::max({v_ego - 0.1, v_ego + v_rel - 1, p2min_speed});
      a_limit_cutin[i] = -0.6;
      cutin_condition[i] = "potential_cutin_car_2, stage 1";
      if (std::abs(near_cars_sorted[i]->y_min()) < 1.7) {
        v_limit_cutin[i] = std::max(
            {std::min(v_ego + v_rel - 1, v_ego - 1), v_ego - 3, p2min_speed});
        a_limit_cutin[i] = -1.0;
        cutin_condition[i] = "potential_cutin_car_2, stage 1.5";
      }
    } else if (cutin_valid &&
               ((0.8 <= std::abs(near_cars_sorted[i]->y_min())) &&
                (std::abs(near_cars_sorted[i]->y_min()) <= 3.0)) &&
               potential_cutin_car_2 &&
               ((-6.0 - d_x_offset <= near_cars_sorted[i]->location_tail()) &&
                (near_cars_sorted[i]->location_tail() <= -3.5)) &&
               (near_cars_sorted[i]->v_rel() > 1.0) &&
               (near_cars_sorted[i]->v_rel() + v_ego > 10.0)) {
      v_limit_cutin[i] = std::max(v_ego - 0.1, p2min_speed);
      a_limit_cutin[i] = -0.6;
      cutin_condition[i] = "potential_cutin_car_2, stage 2";
      if (std::abs(near_cars_sorted[i]->y_min()) < 1.7 ||
          std::abs(vy_rel) > 0.7) {
        v_limit_cutin[i] = std::max(v_ego - 1, p2min_speed);
        a_limit_cutin[i] = -1.0;
        cutin_condition[i] = "potential_cutin_car_2, stage 2.5";
      }
    } else if (lead_car && ((-3.5 < near_cars_sorted[i]->location_tail()) &&
                            (near_cars_sorted[i]->location_tail() <= 3.5))) {
      v_limit_cutin[i] = v_ego + v_rel - 1;
      // a_limit_cutin[i] =
      //     -0.5 + calc_critical_decel(near_cars_sorted[i]->location_tail(),
      //                                -near_cars_sorted[i]->v_rel(), d_offset,
      //                                0);
      cutin_condition[i] = "lead_car";
    } else {
      v_limit_cutin[i] =
          clip(v_limit_cutin[i] + 0.2, std::max(40.0, v_ego + 0.2), v_ego);
      a_limit_cutin[i] = a_limit_cutin[i] + 0.1;
      cutin_condition[i] = "stage other";
    }

    // set cutin msg
    // bool not_avoid =
    //     ((-3.5 - d_x_offset - std::min(near_cars_sorted[i]->v_rel(), 0.0) >
    //     near_cars_sorted[i]->location_tail() ||
    //     near_cars_sorted[i]->location_tail() > 2.0)
    //     && near_cars_sorted[i]->v_rel() > 0) ||
    //     std::abs(near_cars_sorted[i]->y_min()) < 0.8;

    v_limit_cutin[i] = std::max(v_limit_cutin[i], 0.0);
    a_limit_cutin[i] = clip(a_limit_cutin[i], 0.0, -4.0);

    // 临时出一下cutin的级别，便于可视化
    if (cutin_condition[i] == "cutin_car, stage 1" ||
        cutin_condition[i] == "cutin_car, stage 2") {
      cutin_status = 10;
    } else if (cutin_condition[i] == "potential_cutin_car_1, stage 1" ||
               cutin_condition[i] == "potential_cutin_car_1, stage 1.5") {
      cutin_status = 15;
    } else if (cutin_condition[i] == "potential_cutin_car_2, stage 1" ||
               cutin_condition[i] == "potential_cutin_car_2, stage 1.5" ||
               cutin_condition[i] == "potential_cutin_car_2, stage 2" ||
               cutin_condition[i] == "potential_cutin_car_2, stage 2.5") {
      cutin_status = 20;
    } else if (cutin_condition[i] == "lead_car") {
      cutin_status = 25;
    } else {
      cutin_status = 30;
    }
  }

  for (int i = 0; i < (3 - near_cars_sorted.size()); i++) {
    nearest_car_track_id[i + near_cars_sorted.size()] = 0;
    v_limit_cutin[i + near_cars_sorted.size()] =
        clip(v_limit_cutin[i + near_cars_sorted.size()] + 0.2,
             std::max(40.0, v_ego + 0.2), v_ego);
    a_limit_cutin[i + near_cars_sorted.size()] =
        clip(a_limit_cutin[i + near_cars_sorted.size()] + 0.1, 0.0, -4.0);
    cutin_condition[i + near_cars_sorted.size()] = "near_cars_sorted None";
    // cutin_info_[i + near_cars_sorted.size()] = "none";
  }

  int v_min_index = 0;
  for (int i = 0; i < v_limit_cutin.size(); i++) {
    if (v_limit_cutin[i] < v_limit_cutin[v_min_index]) {
      v_min_index = i;
    }
  }

  // acc_target_.first = std::min(acc_target_.first,
  // a_limit_cutin[v_min_index]);
  v_target_ = std::min(v_target_, v_limit_cutin[v_min_index]);

  LOG_DEBUG("nearest_car_track_id : [%d],[%d],[%d] \n", nearest_car_track_id[0],
            nearest_car_track_id[1], nearest_car_track_id[2]);
  LOG_DEBUG("v_limit_cutin : [%f], v_target : [%f] \n",
            v_limit_cutin[v_min_index], v_target_);

  JSON_DEBUG_VALUE("nearest_car_track_id_one", nearest_car_track_id[0]);
  JSON_DEBUG_VALUE("nearest_car_track_id_two", nearest_car_track_id[1]);
  JSON_DEBUG_VALUE("nearest_car_track_id_three", nearest_car_track_id[2]);
  JSON_DEBUG_VALUE("v_target_cutin", v_limit_cutin[v_min_index]);
  JSON_DEBUG_VALUE("cutin_status", cutin_status);
}

double StGraphGenerator::ProcessObstacleAcc(const double a_lead) {
  // soft threshold of 0.5m/s^2 applied to a_lead to reject noise, also not
  // considered positive a_lead
  double a_lead_threshold = 0.5;
  return std::min(a_lead + a_lead_threshold, 0.0);
}

double StGraphGenerator::CalcDesiredDistance(
    const planning::common::TrackedObjectInfo &lead_obstacle,
    const double v_ego, const std::string &lc_request) {
  LOG_DEBUG("-----CalcDesiredDistance \n");
  double desired_distance = 50.0;  // default value

  // 跟车距离两种方式：RSS和标定
  double desired_distance_rss = GetRSSDistance(lead_obstacle.v_lead(), v_ego);
  double desired_distance_calibrate = GetCalibratedDistance(
      lead_obstacle.v_lead(), v_ego, lc_request,
      lead_obstacle.is_accident_car(), lead_obstacle.is_temp_lead(),
      lead_obstacle.is_lead());
  JSON_DEBUG_VALUE("RealTime_desired_distance_rss", desired_distance_rss);
  JSON_DEBUG_VALUE("RealTime_desired_distance_calibrate",
                   desired_distance_calibrate);
  if (config_.enable_rss_model && lc_request == "none") {
    desired_distance = desired_distance_rss;
  } else {
    desired_distance = desired_distance_calibrate;
  }
  return desired_distance;
}

double StGraphGenerator::GetRSSDistance(const double obstacle_velocity,
                                        double ego_velocity) {
  const double stop_distance = config_.dis_zero_speed;
  double follow_distance{0.0};
  double cipv_velocity = obstacle_velocity;
  const double t_actuator_delay = config_.t_actuator_delay;  // actuator delay
  // maximum comfortable acceleration of the ego
  const double a_max_comfort_accel = config_.a_max_comfort_accel;
  // maximum comfortable braking deceleration of the ego
  const double a_max_comfort_brake = config_.a_max_comfort_brake;
  // maximum braking deceleration of the CIPV
  const double CIPV_max_brake = config_.CIPV_max_brake;

  follow_distance =
      ego_velocity * t_actuator_delay +
      0.5 * a_max_comfort_accel * std::pow(t_actuator_delay, 2) +
      (std::pow((ego_velocity + t_actuator_delay * a_max_comfort_accel), 2) /
           2.0 / std::fabs(a_max_comfort_brake) -
       std::pow(cipv_velocity, 2) / 2.0 / std::fabs(CIPV_max_brake));
  follow_distance = std::max(follow_distance, stop_distance);
  return follow_distance;
}

double StGraphGenerator::GetCalibratedDistance(
    const double v_lead, const double v_ego, const std::string &lc_request,
    const bool is_accident_car, const bool is_temp_lead, const bool is_lead) {
  LOG_DEBUG("-----calc_desired_distance \n");
  // 受限感知性能，取非负
  double v_lead_clip = std::max(v_lead, 0.0);
  // 这里查表、魔数都要优化
  double t_gap = interp(v_ego, _T_GAP_VEGO_BP, _T_GAP_VEGO_V);
  if (lc_request != "none") {
    t_gap = t_gap * (0.6 + v_ego * 0.01);
  }
  // 同一个障碍物从temp_lead_one变成lead_one后，temp_lead的标志未清除，导致t_gap计算有问题，这里先加一个二者互斥的判断
  if (is_temp_lead && !is_lead) {
    t_gap = t_gap * 0.3;
  }
  // Brake hysteresis
  double v_relative = std::min(std::max(v_ego - v_lead_clip, 0.0), 5.0);
  double distance_hysteresis = v_relative * config_.ttc_brake_hysteresis;
  // distance when at zero speed
  double d_offset = config_.dis_zero_speed;
  std::cout << "d_offset from config === : " << d_offset << std::endl;
  if (is_accident_car) {
    d_offset = config_.dis_zero_speed_accident;
  }
  LOG_DEBUG("distance_hysteresis : [%f] \n", distance_hysteresis);
  LOG_DEBUG("ttc gap : [%f] \n", t_gap);
  LOG_DEBUG("desired_distance : [%f] \n",
            d_offset + v_lead_clip * t_gap + distance_hysteresis);
  return d_offset + v_lead_clip * t_gap + distance_hysteresis;
}

double StGraphGenerator::CalcSafeDistance(const double obstacle_velocity,
                                          const double v_ego) {
  double safe_distance_base = config_.safe_distance_base;
  double safe_distance_ttc = config_.safe_distance_ttc;
  return std::max(
      0.0, safe_distance_base +
               std::max(v_ego - obstacle_velocity, 0.0) * safe_distance_ttc);
}

void StGraphGenerator::UpdateSpeedWithPotentialCutinCar(
    const planning::common::LatObsInfo &lateral_obstacles,
    const std::string &lc_request, double v_cruise, double v_ego,
    std::vector<planning::common::RealTimeLonObstacleSTInfo> &cut_in_st_info) {
  double a_processed = 0.0;
  double desired_distance = 0.0;
  double safe_distance = 0.0;
  double desired_velocity = 40.0;
  double time_to_entry = 0.0;
  double predict_distance = 0.0;
  double desired_distance_filtered = 0.0;

  double v_target_potential_cutin = 40.0;
  double v_limit = std::min(v_cruise, v_ego);
  std::pair<int, double> cutin_id_vt = {-1, 0.0};
  std::pair<double, double> acc_target = {-0.5, 0.5};

  auto cut_in_info =
      lon_behav_input_->mutable_lon_decision_info()->mutable_cutin_info();

  // threshold of cut in probability
  double cutinp_threshold = (lc_request == "none")
                                ? config_.lane_keep_cutinp_threshold
                                : config_.lane_change_cutinp_threshold;
  double corridor_width = config_.corridor_width;

  std::vector<int> front_cut_in_track_id;
  front_cut_in_track_id.clear();

  for (auto &track : lateral_obstacles.front_tracks()) {
    // ignore obj without camera source
    if ((track.fusion_source() & OBSTACLE_SOURCE_CAMERA) == 0) {
      continue;
    };
    if (!track.is_lead() &&
        (track.cutinp() > cutinp_threshold || track.is_new_cutin()) &&
        track.v_lat() < -0.01 &&
        track.type() != iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN) {
      cut_in_info->set_has_cutin(true);

      front_cut_in_track_id.push_back(track.track_id());
      time_to_entry = std::max(track.d_path() - corridor_width / 2, 0.0) /
                      std::max(-track.v_lat(), 0.01);
      if (time_to_entry >= 5.0) {
        // 目前考虑未来5s，注意time_to_entry是否会＞5s
        LOG_DEBUG("The Obj [%d] 's cut in speed is too slow. \n",
                  track.track_id());
        continue;
      }
      // Predict the distance when entering corridor
      predict_distance =
          std::max(0.0, track.d_rel() + time_to_entry * track.v_rel());
      // no prediction if obstalce is slower than ego
      if (track.v_rel() < 0.1) {
        predict_distance = track.d_rel();
      }

      a_processed = ProcessObstacleAcc(track.a_lead_k());
      // 计算期望跟车距离
      safe_distance = CalcSafeDistance(track.v_lead(), v_ego);
      desired_distance = CalcDesiredDistance(track, v_ego, lc_request);
      // 计算期望车速
      desired_velocity =
          CalcDesiredVelocity(track.d_rel(), desired_distance, track.v_lead());

      CalcAccLimits(track, desired_distance, desired_velocity, v_ego,
                    a_processed, acc_target);
      acc_target_.first = std::min(acc_target_.first, acc_target.first);
      acc_target_.second = std::min(acc_target_.second, acc_target.second);

      // 通过切入概率更新目标车速
      // v_target_potential_cutin =
      //     std::min(v_target_potential_cutin,
      //              (desired_velocity - v_limit) * track.cutinp() + v_limit);
      double v_potential_cutin = desired_velocity;
      if (v_target_potential_cutin > v_potential_cutin) {
        v_target_potential_cutin = v_potential_cutin;
        cutin_id_vt.first = track.track_id();
        cutin_id_vt.second = v_target_potential_cutin;
      }

      desired_distance_filtered = CutInDesiredDistanceFilter(
          track, v_ego, safe_distance, predict_distance, desired_distance);

      // update potential_cutin st
      common::RealTimeLonObstacleSTInfo st_info;
      st_info.set_st_type(common::RealTimeLonObstacleSTInfo::CUT_IN);
      st_info.set_id(track.track_id());
      st_info.set_a_lead(a_processed);
      st_info.set_v_lead(track.v_lead());
      st_info.set_s_lead(track.d_rel());
      st_info.set_desired_distance(desired_distance_filtered);
      st_info.set_desired_velocity(v_target_potential_cutin);
      st_info.set_safe_distance(safe_distance);
      st_info.set_start_time(time_to_entry);  // TBD:使用可配置参数
      st_info.set_end_time(5.0);              // TBD:使用可配置参数
      // st_info.set_start_s(time_to_entry * v_ego + predict_distance);
      st_info.set_start_s(track.d_rel());
      cut_in_st_info.emplace_back(st_info);
      v_target_ = std::min(v_target_, v_target_potential_cutin);

      LOG_DEBUG("potential_cutin_car's id: [%d], track.v_lat is: [%f]\n",
                track.track_id(), track.v_lat());
      LOG_DEBUG(
          "desired_distance: [%f], desired_velocity: [%f], "
          "v_target_potential_cutin: [%f]\n",
          desired_distance, desired_velocity, v_target_potential_cutin);
    } else {
      cut_in_info->Clear();
    }
  }
  JSON_DEBUG_VALUE("v_target_potential_cutin", cutin_id_vt.second);
  JSON_DEBUG_VALUE("potential_cutin_track_id", cutin_id_vt.first);
};

double StGraphGenerator::CalcDesiredVelocity(const double d_rel,
                                             const double d_des,
                                             const double v_lead) {
  // *** compute desired speed ***
  // the desired speed curve is divided in 4 portions:
  // 1-constant
  // 2-linear to regain distance
  // 3-linear to shorten distance
  // 4-parabolic (constant decel)
  const double max_runaway_speed = -2.;  // no slower than 2m/s over the lead
  //  interpolate the lookups to find the slopes for a give lead speed
  double l_slope = interp(v_lead, _L_SLOPE_BP, _L_SLOPE_V);
  double p_slope = interp(v_lead, _P_SLOPE_BP, _P_SLOPE_V);
  // this is where parabola && linear curves are tangents
  double x_linear_to_parabola = p_slope / std::pow(l_slope, 2);
  // parabola offset to have the parabola being tangent to the linear curve
  double x_parabola_offset = p_slope / (2 * std::pow(l_slope, 2));
  LOG_DEBUG("-----calc_desired_speed \n");
  LOG_DEBUG("l_slope : [%f] , p_slope : [%f]\n", l_slope, p_slope);
  LOG_DEBUG("x_linear_to_parabola : [%f] , x_parabola_offset : [%f]\n",
            x_linear_to_parabola, x_parabola_offset);

  double v_ego = lon_behav_input_->ego_info().ego_v();
  double v_rel = v_ego - v_lead;
  double v_rel_des = 0.0;
  double soft_brake_distance = 0.0;
  if (d_rel < d_des) {
    // calculate v_rel_des on the line that connects 0m at max_runaway_speed
    // to d_des
    double v_rel_des_1 = (-max_runaway_speed) / d_des * (d_rel - d_des);
    // calculate v_rel_des on one third of the linear slope
    double v_rel_des_2 = (d_rel - d_des) * l_slope / 3.0;
    // take the min of the 2 above
    v_rel_des = std::min(v_rel_des_1, v_rel_des_2);
    v_rel_des = std::max(v_rel_des, max_runaway_speed);
    soft_brake_distance = d_rel;
  } else if (d_rel < d_des + x_linear_to_parabola) {
    v_rel_des = (d_rel - d_des) * l_slope;
    v_rel_des = std::max(v_rel_des, max_runaway_speed);
    soft_brake_distance = v_rel / l_slope + d_des;
  } else {
    v_rel_des = std::sqrt(2 * (d_rel - d_des - x_parabola_offset) * p_slope);
    soft_brake_distance =
        std::pow(v_rel, 2) / (2 * p_slope) + x_parabola_offset + d_des;
  }
  // compute desired speed
  double v_target = v_rel_des + v_lead;
  LOG_DEBUG("v_rel_des : [%f], v_target : [%f] \n", v_rel_des, v_target);
  JSON_DEBUG_VALUE("soft_brake_distance_lead", soft_brake_distance);
  return v_target;
}

void StGraphGenerator::CalcSpeedInfoWithGap(
    const planning::common::TrackedObjectInfo &lead_one, const double v_cruise,
    const double v_ego, const string &lc_request, const string &lc_status,
    std::vector<planning::common::RealTimeLonObstacleSTInfo>
        &lane_change_st_info) {
  LOG_DEBUG("----entering CalcSpeedInfoWithGap--- \n");
  double lc_t_gap = 0.2;
  double lc_buffer = 2;
  double safe_distance =
      lane_changing_decider_->get_lc_safe_dist(lc_buffer, lc_t_gap, v_ego);
  double time_to_lc = 0.0;
  double predict_distance = 0.0;
  v_limit_lc_ = 40.0;
  lane_change_st_info.clear();

  std::vector<const planning::common::TrackedObjectInfo *> lane_changing_cars;
  std::vector<GapInfo> available_gap;
  int lane_changing_nearest_rear_car_track_id = -10;

  double lc_end_dis = lon_behav_input_->lc_info().lc_end_dis();
  int lc_map_decision = lon_behav_input_->lc_info().lc_map_decision();
  lane_changing_cars.clear();
  if ((lc_request != "none") &&
      ((lc_status == "none") || (lc_status == "left_lane_change_wait") ||
       (lc_status == "right_lane_change_wait"))) {
    LOG_DEBUG("!! lang change !! \n");
    // get target line tarcks
    if (lon_behav_input_->lc_info().has_target_lane()) {
      for (auto &track : lon_behav_input_->lc_info().lc_cars()) {
        // ignore obj without camera source
        if ((track.fusion_source() & OBSTACLE_SOURCE_CAMERA) == 0) {
          continue;
        };
        lane_changing_cars.push_back(&track);
      }
    }

    RTLaneChangeParams lane_changing_params;
    lane_changing_params.most_front_car_dist = 110.0;
    lane_changing_params.most_rear_car_dist = -100.0;
    lane_changing_params.cost_minus = 10.0;
    lane_changing_params.v_rel_bufer = 1.0;
    lane_changing_decider_->feed_config_and_target_cars(
        false, lane_changing_params, lc_end_dis, lane_changing_cars, lead_one,
        v_ego);

    lane_changing_decider_->process();

    available_gap = lane_changing_decider_->get_gap_list();
    if (available_gap.size() > 0) {
      auto gap = available_gap[0];
      if (gap.base_car_id == gap.front_id) {
        // safe_distance = CalcSafeDistance(gap.v_front, v_ego);
        v_limit_lc_ = gap.base_car_vrel -
                      clip((safe_distance - gap.base_car_drel) / safe_distance,
                           2.0, 0.0) -
                      1.0;
        if (v_limit_lc_ < 0) {
          // no need to decel when front car is far away
          const std::vector<double> _V_LIMIT_DISTANCE_BP{
              safe_distance + std::max(-gap.base_car_vrel, 0.0) * 2,
              safe_distance * 2 + std::max(-gap.base_car_vrel, 0.0) * 2};
          const std::vector<double> _V_LIMIT_DISTANCE_V{1.0, 0.0};
          v_limit_lc_ =
              v_limit_lc_ * interp(gap.base_car_drel, _V_LIMIT_DISTANCE_BP,
                                   _V_LIMIT_DISTANCE_V);
        }
        v_limit_lc_ = std::max(v_ego - 3.0, v_ego + v_limit_lc_);
        // a_target_lc = 0.0;
      } else {
        // safe_distance = CalcSafeDistance(gap.v_rear, v_ego);
        v_limit_lc_ =
            gap.base_car_vrel +
            clip((safe_distance + 5.0 + gap.base_car_drel) / safe_distance, 2.0,
                 0.0) +
            1.0;
        if (v_limit_lc_ < 0) {
          // no need to decel when front car is far away
          const std::vector<double> _V_LIMIT_DISTANCE_BP{
              safe_distance + 5.0 + std::max(gap.base_car_vrel, 0.0) * 2,
              safe_distance * 2 + 5.0 + std::max(gap.base_car_vrel, 0.0) * 2};
          const std::vector<double> _V_LIMIT_DISTANCE_V{1.0, 0.0};
          v_limit_lc_ =
              v_limit_lc_ * interp(-gap.base_car_drel, _V_LIMIT_DISTANCE_BP,
                                   _V_LIMIT_DISTANCE_V);
        }
        v_limit_lc_ = std::max(v_ego - 2.8, v_ego + v_limit_lc_);
        // a_target_lc = 0.6;
      }
      if (v_limit_lc_ < 6.0) {
        v_limit_lc_ = 6.0;
        // a_target_lc = 1.0;
      }
      JSON_DEBUG_VALUE("gap_v_limit_lc", v_limit_lc_);

      double safe_distance_lc_front =
          CalcSafeDistance(gap.v_front, v_limit_lc_);
      double lc_front_desired_distance = GetCalibratedDistance(
          gap.v_front, v_limit_lc_, lc_request, false, false, false);

      planning::common::TrackedObjectInfo front_obs;
      front_obs.set_track_id(gap.front_id);
      front_obs.set_v_rel(gap.v_front - v_limit_lc_);
      front_obs.set_d_rel(gap.s_front + gap.v_front * gap.acc_time -
                          v_limit_lc_ * gap.acc_time);

      double lc_front_desired_distance_filtered = LCGapDesiredDistanceFilter(
          front_obs, v_limit_lc_, safe_distance_lc_front,
          lc_front_desired_distance, true);

      // common::RealTimeLonObstacleSTInfo lc_gap_front_st_info;
      // lc_gap_front_st_info.set_st_type(common::RealTimeLonObstacleSTInfo::GAP);
      // lc_gap_front_st_info.set_id(gap.front_id);
      // lc_gap_front_st_info.set_a_lead(0.0);
      // lc_gap_front_st_info.set_v_lead(gap.v_front);
      // lc_gap_front_st_info.set_s_lead(gap.s_front);
      // lc_gap_front_st_info.set_desired_distance(
      //     lc_front_desired_distance_filtered);
      // lc_gap_front_st_info.set_desired_velocity(v_limit_lc_);
      // lc_gap_front_st_info.set_safe_distance(safe_distance_lc_front);
      // lc_gap_front_st_info.set_start_time(gap.acc_time);  //
      // TBD:使用可配置参数 lc_gap_front_st_info.set_end_time(5.0);  //
      // TBD:使用可配置参数 lc_gap_front_st_info.set_start_s(gap.s_front);
      // lane_change_st_info.emplace_back(lc_gap_front_st_info);

      double safe_distance_lc_rear = CalcSafeDistance(v_limit_lc_, gap.v_rear);
      double lc_rear_desired_distance = GetCalibratedDistance(
          v_limit_lc_, gap.v_rear, lc_request, false, false, false);

      planning::common::TrackedObjectInfo rear_obs;
      rear_obs.set_track_id(gap.rear_id);
      rear_obs.set_v_rel(gap.v_rear - v_limit_lc_);
      rear_obs.set_d_rel(gap.s_rear + gap.v_rear * gap.acc_time -
                         v_limit_lc_ * gap.acc_time);

      double lc_rear_desired_distance_filtered = LCGapDesiredDistanceFilter(
          rear_obs, v_limit_lc_, safe_distance_lc_rear,
          lc_rear_desired_distance, false);

      // common::RealTimeLonObstacleSTInfo lc_gap_rear_st_info;
      // lc_gap_rear_st_info.set_st_type(common::RealTimeLonObstacleSTInfo::GAP);
      // lc_gap_rear_st_info.set_decision(
      //     common::RealTimeLonObstacleSTInfo::OVERTAKE);
      // lc_gap_rear_st_info.set_id(gap.rear_id);
      // lc_gap_rear_st_info.set_a_lead(0.0);
      // lc_gap_rear_st_info.set_v_lead(gap.v_rear);
      // lc_gap_rear_st_info.set_s_lead(gap.s_rear);
      // lc_gap_rear_st_info.set_desired_distance(
      //     lc_rear_desired_distance_filtered);
      // lc_gap_rear_st_info.set_desired_velocity(v_limit_lc_);
      // lc_gap_rear_st_info.set_safe_distance(safe_distance_lc_rear);
      // lc_gap_rear_st_info.set_start_time(gap.acc_time);  //
      // TBD:使用可配置参数 lc_gap_rear_st_info.set_end_time(5.0);  //
      // TBD:使用可配置参数 lc_gap_rear_st_info.set_start_s(gap.s_rear);
      // lane_change_st_info.emplace_back(lc_gap_rear_st_info);
    } else {
      // decelerate to check next interval
      auto nearest_rear_car = lane_changing_decider_->nearest_rear_car_track();
      lane_changing_nearest_rear_car_track_id = nearest_rear_car.id;
      v_limit_lc_ =
          nearest_rear_car.v_rel -
          clip((safe_distance - nearest_rear_car.d_rel) / safe_distance, 2.0,
               0.0) -
          v_ego / 10.0;
      v_limit_lc_ = std::max({v_ego - 3.2, v_ego + v_limit_lc_,
                              6.0 + 4.0 * std::max(lc_map_decision - 2, 0)});
      // a_target_lc = 0.0;
      JSON_DEBUG_VALUE("gap_v_limit_lc", v_limit_lc_);
    }
  } else {
    JSON_DEBUG_VALUE("gap_v_limit_lc", 0);
  }
  // acc_target_.second = std::max(acc_target_.second, a_target_lc);
  v_target_ = std::min(v_target_, v_limit_lc_);
}

bool StGraphGenerator::CalcSpeedInfoWithVirtualObstacle(
  const std::shared_ptr<planning::planning_data::DynamicWorld>& dynamic_world,
  std::vector<planning::common::RealTimeLonObstacleSTInfo>& virtual_obs_st_info) {
  LOG_DEBUG("----calc_speed_for_virtual_obstacle--- \n");
  double virtual_obs_a_processed = 0.0;
  double virtual_obs_desired_distance = 0.0;
  double safe_distance = 0.0;
  double virtual_obs_desired_velocity = 40.0;
  double desired_distance_filtered = 0.0;
  double v_ego = lon_behav_input_->ego_info().ego_v();

  bool is_virtual_obs_exist = false;
  const planning::agent::Agent* virtual_obs = NULL;
  const auto agent_manager = dynamic_world->agent_manager();
  const auto &all_current_agents = agent_manager->GetAllCurrentAgents();
  for (int i = 0; i < all_current_agents.size(); i++) {
    const auto agt = all_current_agents[i];
    if(agt->is_tfl_virtual_obs()) {
      is_virtual_obs_exist = true;
      virtual_obs = agt;
      break;
    }
  }
  if(is_virtual_obs_exist) {
    virtual_obs_a_processed = ProcessObstacleAcc(virtual_obs->accel());
    safe_distance = CalcSafeDistance(virtual_obs->speed(), v_ego);
    virtual_obs_desired_distance = GetCalibratedDistance(
      virtual_obs->speed(), v_ego, lon_behav_input_->lat_output().lc_request(),
      false, false, false);
    double dis_to_virtual_obs = std::min(lon_behav_input_->dis_to_stopline() - 1, lon_behav_input_->dis_to_crosswalk() - 3);
    if(dis_to_virtual_obs < 1.0) {
      dis_to_virtual_obs = 1.0;
    }
    virtual_obs_desired_velocity = CalcDesiredVelocity(
        dis_to_virtual_obs, virtual_obs_desired_distance, virtual_obs->speed());

    //desired_distance_filtered = DesiredDistanceFilter(
    //    lead_one, v_ego, safe_distance, virtual_obs_desired_distance);

    // update virtual obs st
    common::RealTimeLonObstacleSTInfo virtual_obs_st;
    virtual_obs_st.set_st_type(common::RealTimeLonObstacleSTInfo::LEADS);
    virtual_obs_st.set_id(virtual_obs->agent_id());
    virtual_obs_st.set_a_lead(virtual_obs_a_processed);
    virtual_obs_st.set_v_lead(virtual_obs->speed());
    virtual_obs_st.set_s_lead(dis_to_virtual_obs);
    virtual_obs_st.set_desired_distance(virtual_obs_desired_distance);
    virtual_obs_st.set_desired_velocity(virtual_obs_desired_velocity);
    virtual_obs_st.set_safe_distance(safe_distance);
    virtual_obs_st.set_start_time(0.0);  // TBD:使用可配置参数
    virtual_obs_st.set_end_time(5.0);    // TBD:使用可配置参数
    virtual_obs_st.set_start_s(dis_to_virtual_obs);
    virtual_obs_st_info.emplace_back(virtual_obs_st);
    v_target_ = std::min(v_target_, virtual_obs_desired_velocity);

    JSON_DEBUG_VALUE("virtual_obs_id", virtual_obs->agent_id());
    JSON_DEBUG_VALUE("virtual_obs_dis", dis_to_virtual_obs);
    JSON_DEBUG_VALUE("virtual_obs_vel", virtual_obs->speed());
    JSON_DEBUG_VALUE("v_target_virtual_obs", virtual_obs_desired_velocity);
    JSON_DEBUG_VALUE("desired_distance_virtual_obs", virtual_obs_desired_distance);
  } else {
    JSON_DEBUG_VALUE("virtual_obs_id", -1);
    JSON_DEBUG_VALUE("v_target_virtual_obs", virtual_obs_desired_velocity);
  }

  return true;
}

bool StGraphGenerator::CalcSpeedInfoWithIntersection() {
  LOG_DEBUG("----calc_speed_for_intersection--- \n");
  double v_ego = lon_behav_input_->ego_info().ego_v();
  double v_target_intersection = 40.0;
  current_intersection_state_ = lon_behav_input_ ->intersection_state();
  if (current_intersection_state_ == planning::common::APPROACH_INTERSECTION ||
      current_intersection_state_ == planning::common::IN_INTERSECTION) {
    if (v_limit_with_intersection_ < 8.33) {
      ///v_target_intersection = std::max(v_ego - 3.0, 8.33);
      v_limit_with_intersection_ = std::max(v_ego - 3.0, 8.33);
    }
    v_target_intersection = v_limit_with_intersection_;
  } else {
    v_limit_with_intersection_ = 0.0;
  }
  v_target_ = std::min(v_target_intersection, v_target_);
  JSON_DEBUG_VALUE("v_target_intersection", v_target_intersection);
  JSON_DEBUG_VALUE("current_intersection_state", int(current_intersection_state_));
  JSON_DEBUG_VALUE("last_intersection_state", int(last_intersection_state_));
  last_intersection_state_ = current_intersection_state_;
  LOG_DEBUG("v_target : [%f] \n", v_target_);
  return true;
}

std::pair<double, double> StGraphGenerator::CalculateMaxAcc(double ego_v) {
  // refer to the International Standard: ISO 15622-2018
  // 为了和实时版本一致 这里将pair的first改为acc_low second改为acc_high
  std::pair<double, double> a_max;
  if (ego_v < 5.0) {
    a_max.first = -5.0;
    a_max.second = 4.0;
  } else if (ego_v > 20.0) {
    a_max.first = -2.5;
    a_max.second = 2.0;
  } else {
    a_max.second = -2.0 / 15.0 * ego_v + 14.0 / 3.0;
    a_max.first = 1.0 / 6.0 * ego_v - 35.0 / 6.0;
  }
  return a_max;
}

void StGraphGenerator::CalculateCruiseSrefs(const double v_ego,
                                            const double v_cruise,
                                            const double acc_ego,
                                            std::vector<double> &s_refs) {
  LOG_DEBUG("----entering CalculateCruiseSrefs--- \n");
  double one_a = acc_ego;
  double one_v = v_ego;
  // double one_s = 0.0;
  // 临时hack，需要对齐s
  double one_s = 0.0;  // lon_behav_input_->ego_info().init_s();
  double one_s_step = 0.0;
  s_refs.emplace_back(one_s);

  std::pair<double, double> max_acc_info = CalculateMaxAcc(v_ego);
  double a_max_accel = std::min(max_acc_info.second, 2.0);
  double a_max_brake = max_acc_info.first;

  if (v_ego <= v_cruise) {
    double one_j = _J_MAX;
    for (int i = 1; i <= config_.lon_num_step; i++) {
      one_s_step =
          std::max(one_v * config_.delta_time +
                       0.5 * one_a * std::pow(config_.delta_time, 2) +
                       1.0 / 6 * one_j * std::pow(config_.delta_time, 3),
                   0.0);
      one_s = one_s + one_s_step;
      one_v =
          std::max(std::min(one_v + one_a * config_.delta_time +
                                0.5 * one_j * std::pow(config_.delta_time, 2),
                            v_cruise),
                   0.0);
      if (one_v == v_cruise) {
        one_a = 0.0;
      } else {
        one_a = std::min(one_a + one_j * config_.delta_time, a_max_accel);
      }
      if (one_v == v_cruise || one_a == a_max_accel) {
        one_j = 0.0;
      }
      s_refs.emplace_back(one_s);
    }
  } else {
    double one_j = _J_MIN;
    for (int i = 1; i <= config_.lon_num_step; i++) {
      one_s = one_s + one_v * config_.delta_time +
              0.5 * one_a * std::pow(config_.delta_time, 2) +
              1.0 / 6 * one_j * std::pow(config_.delta_time, 3);
      one_v = std::max(one_v + one_a * config_.delta_time +
                           0.5 * one_j * std::pow(config_.delta_time, 2),
                       v_cruise);
      if (one_v == v_cruise) {
        one_a = 0.0;
      } else {
        one_a = std::max(one_a + one_j * config_.delta_time, a_max_brake);
      }
      if (one_v == v_cruise || one_a == a_max_brake) {
        one_j = 0.0;
      }
      s_refs.emplace_back(one_s);
    }
  }
}

double StGraphGenerator::DesiredDistanceFilter(
    const planning::common::TrackedObjectInfo &lead_obstacle,
    const double v_ego, double safe_distance, double desired_distance) {
  double desired_distance_new = desired_distance;
  auto leadone_info =
      lon_behav_input_->mutable_lon_decision_info()->mutable_leadone_info();

  // 更新lead初始信息
  if (leadone_info->leadone_information().obstacle_id() !=
          lead_obstacle.track_id() ||
      leadone_info->has_leadone() != true) {
    leadone_info->set_has_leadone(true);
    leadone_info->mutable_leadone_information()->set_obstacle_id(
        lead_obstacle.track_id());
    leadone_info->mutable_leadone_information()->set_desired_distance(
        std::min(lead_obstacle.d_rel(), safe_distance));
  }

  // TBD: 目前cut in和lead区分不明显，cut in会被判断为lead
  bool slow_car_cut_in = false;
  if (lead_obstacle.d_rel() < desired_distance &&
      lead_obstacle.v_rel() <= 0.5) {
    slow_car_cut_in = true;
  }
  lead_desired_distance_filter_.SetState(
      leadone_info->leadone_information().desired_distance());
  if (!(lon_behav_input_->dbw_status())) {
    lead_desired_distance_filter_.SetState(
        std::min(lead_obstacle.d_rel(), safe_distance));
  }
  if (slow_car_cut_in) {
    // 慢车切入
    lead_desired_distance_filter_.SetRate(-4.0,
                                          config_.slow_lead_distance_step);
    lead_desired_distance_filter_.Update(desired_distance);
    desired_distance_new = lead_desired_distance_filter_.GetOutput();
    JSON_DEBUG_VALUE("slow_lead_id", lead_obstacle.track_id());
  } else {
    // 快车切入
    lead_desired_distance_filter_.SetRate(-4.0,
                                          config_.fast_lead_distance_step);
    lead_desired_distance_filter_.Update(desired_distance);
    desired_distance_new = lead_desired_distance_filter_.GetOutput();
    JSON_DEBUG_VALUE("fast_lead_id", lead_obstacle.track_id());
  }
  leadone_info->mutable_leadone_information()->set_desired_distance(
      desired_distance_new);
  return desired_distance_new;
}

bool StGraphGenerator::CalcAccLimits(
    const planning::common::TrackedObjectInfo &lead_obstacle,
    const double desired_distance, const double v_target, const double v_ego,
    const double lead_one_a_processed, std::pair<double, double> &acc_target) {
  //
  double agent_v_rel = -lead_obstacle.v_rel();
  double a_lead_contr =
      lead_one_a_processed *
      interp(lead_obstacle.v_lead(), _A_LEAD_LOW_SPEED_BP,
             _A_LEAD_LOW_SPEED_V) *
      interp(desired_distance, _A_LEAD_DISTANCE_BP, _A_LEAD_DISTANCE_V) * 0.8;
  acc_target.second =
      CalcPositiveAccLimit(v_ego, agent_v_rel, acc_target.second);
  // compute max decel
  // assume the car is 1m/s slower
  double v_offset =
      1.2 * std::min(std::max(2.5 - lead_obstacle.d_path(), 0.0) / 1.5, 1.0);
  // assume the distance is 1m lower
  double d_offset = 0.5 + 0.2 * v_ego;
  if (v_target < v_ego) {
    // add small value to avoid by zero divisions
    // compute needed accel to get to 1m distance with -1m/s rel speed
    double decel_offset =
        interp(lead_obstacle.v_lead(), _DECEL_OFFSET_BP, _DECEL_OFFSET_V);

    double critical_decel = CalcCriticalDecel(lead_obstacle.d_rel(),
                                              agent_v_rel, d_offset, v_offset);
    acc_target.first = std::min(decel_offset + critical_decel + a_lead_contr,
                                acc_target.first);
  }
  // a_min can't be higher than a_max
  acc_target.first = min(acc_target.first, acc_target.second);
  // final check on limits
  acc_target.first = clip(acc_target.first, _A_MAX, _A_MIN);
  acc_target.second = clip(acc_target.second, _A_MAX, _A_MIN);
  return true;
}

double StGraphGenerator::CalcPositiveAccLimit(const double v_ego,
                                              const double v_rel,
                                              const double a_max_const) {
  double a_max = a_max_const;
  // same as cruise accel, plus add a small correction based on relative
  // lead speed if the lead car is faster, we can accelerate more, if the
  // car is slower, then we can reduce acceleration
  a_max = a_max + interp(v_ego, _A_CORR_BY_SPEED_BP, _A_CORR_BY_SPEED_V) *
                      clip(-v_rel / 4.0, 1.0, -0.5);
  return a_max;
}

double StGraphGenerator::CalcCriticalDecel(const double d_lead,
                                           const double v_rel,
                                           const double d_offset,
                                           const double v_offset) {
  // this function computes the required decel to avoid crashing, given safety
  // offsets
  double a_critical = -std::pow(std::max(0.0, v_rel + v_offset), 2) /
                      std::max(2 * (d_lead - d_offset), 0.5);
  return a_critical;
}

double StGraphGenerator::LeadtwoDesiredDistanceFilter(
    const planning::common::TrackedObjectInfo &lead_obstacle,
    const double v_ego, double safe_distance, double desired_distance) {
  double desired_distance_new = desired_distance;
  auto leadtwo_info =
      lon_behav_input_->mutable_lon_decision_info()->mutable_leadtwo_info();

  // 更新lead初始信息
  if (leadtwo_info->leadtwo_information().obstacle_id() !=
          lead_obstacle.track_id() ||
      leadtwo_info->has_leadtwo() != true) {
    leadtwo_info->set_has_leadtwo(true);
    leadtwo_info->mutable_leadtwo_information()->set_obstacle_id(
        lead_obstacle.track_id());
    leadtwo_info->mutable_leadtwo_information()->set_desired_distance(
        std::min(lead_obstacle.d_rel(), safe_distance));
  }

  // TBD: 目前cut in和lead区分不明显，cut in会被判断为lead
  bool slow_car_cut_in = false;
  if (lead_obstacle.d_rel() < desired_distance &&
      lead_obstacle.v_rel() <= 0.5) {
    slow_car_cut_in = true;
  }
  lead_two_desired_distance_filter_.SetState(
      leadtwo_info->leadtwo_information().desired_distance());

  if (!(lon_behav_input_->dbw_status())) {
    lead_two_desired_distance_filter_.SetState(
        std::min(lead_obstacle.d_rel(), safe_distance));
  }

  if (slow_car_cut_in) {
    // 慢车切入
    lead_two_desired_distance_filter_.SetRate(-4.0,
                                              config_.slow_lead_distance_step);
    lead_two_desired_distance_filter_.Update(desired_distance);
    desired_distance_new = lead_two_desired_distance_filter_.GetOutput();
    JSON_DEBUG_VALUE("slow_lead_id", lead_obstacle.track_id());
  } else {
    // 快车切入
    lead_two_desired_distance_filter_.SetRate(-4.0,
                                              config_.fast_lead_distance_step);
    lead_two_desired_distance_filter_.Update(desired_distance);
    desired_distance_new = lead_two_desired_distance_filter_.GetOutput();
    JSON_DEBUG_VALUE("fast_lead_id", lead_obstacle.track_id());
  }
  leadtwo_info->mutable_leadtwo_information()->set_desired_distance(
      desired_distance_new);
  return desired_distance_new;
}

double StGraphGenerator::TmpLeadDesiredDistanceFilter(
    const planning::common::TrackedObjectInfo &lead_obstacle,
    const double v_ego, double safe_distance, double desired_distance) {
  double desired_distance_new = desired_distance;
  auto leadone_info = lon_behav_input_->mutable_lon_decision_info()
                          ->mutable_temp_leadone_info();

  // 更新lead初始信息
  if (leadone_info->leadone_information().obstacle_id() !=
          lead_obstacle.track_id() ||
      leadone_info->has_leadone() != true) {
    leadone_info->set_has_leadone(true);
    leadone_info->mutable_leadone_information()->set_obstacle_id(
        lead_obstacle.track_id());
    leadone_info->mutable_leadone_information()->set_desired_distance(
        std::min(lead_obstacle.d_rel(), safe_distance));
  }

  // TBD: 目前cut in和lead区分不明显，cut in会被判断为cut in
  bool slow_car_cut_in = false;
  if (lead_obstacle.d_rel() < desired_distance &&
      lead_obstacle.v_rel() <= 0.5) {
    slow_car_cut_in = true;
  }

  lead_desired_distance_filter_.SetState(
      leadone_info->leadone_information().desired_distance());
  if (slow_car_cut_in) {
    // 慢车切入，从切入距离开始膨胀
    lead_desired_distance_filter_.SetRate(
        -4.0, config_.slow_lead_distance_step - lead_obstacle.v_rel());
    lead_desired_distance_filter_.Update(desired_distance);
    desired_distance_new = lead_desired_distance_filter_.GetOutput();
    JSON_DEBUG_VALUE("slow_lead_id", lead_obstacle.track_id());
  } else {
    // 快车切入，从切入距离开始膨胀
    lead_desired_distance_filter_.SetRate(-4.0,
                                          config_.fast_lead_distance_step);
    lead_desired_distance_filter_.Update(desired_distance);
    desired_distance_new = lead_desired_distance_filter_.GetOutput();
    JSON_DEBUG_VALUE("fast_lead_id", lead_obstacle.track_id());
  }
  leadone_info->mutable_leadone_information()->set_desired_distance(
      desired_distance_new);
  return desired_distance_new;
}

double StGraphGenerator::CutInDesiredDistanceFilter(
    const planning::common::TrackedObjectInfo &cut_in_obstacle,
    const double v_ego, double safe_distance, double predict_distance,
    double desired_distance) {
  double desired_distance_new = desired_distance;
  auto cut_in_info =
      lon_behav_input_->mutable_lon_decision_info()->mutable_cutin_info();

  if (cut_in_info->cutin_information().empty()) {
    cut_in_info->add_cutin_information();
  }
  // TBD: 暂时只用第一个cut in
  auto cutin_information = cut_in_info->mutable_cutin_information(0);

  // 更新cut in初始信息
  if (cutin_information->obstacle_id() != cut_in_obstacle.track_id()) {
    cutin_information->set_obstacle_id(cut_in_obstacle.track_id());
    cutin_information->set_desired_distance(
        std::min(predict_distance, safe_distance));
  }

  bool slow_car_cut_in = false;
  if (predict_distance < desired_distance && cut_in_obstacle.v_rel() <= 0.5) {
    slow_car_cut_in = true;
  }

  cut_in_desired_distance_filter_.SetState(
      cutin_information->desired_distance());
  if (!(lon_behav_input_->dbw_status())) {
    cut_in_desired_distance_filter_.SetState(
        std::min(predict_distance, safe_distance));
  }
  JSON_DEBUG_VALUE("fast_car_cut_in_id", -1.0);
  JSON_DEBUG_VALUE("slow_car_cut_in_id", -1.0);
  if (slow_car_cut_in) {
    // 慢车切入，膨胀速度较快
    cut_in_desired_distance_filter_.SetRate(
        -4.0, config_.cut_in_desired_distance_step - cut_in_obstacle.v_rel());
    cut_in_desired_distance_filter_.Update(desired_distance);
    desired_distance_new = cut_in_desired_distance_filter_.GetOutput();
    JSON_DEBUG_VALUE("slow_car_cut_in_id", cut_in_obstacle.track_id());
  } else {
    // 快车切入，缓慢膨胀
    cut_in_desired_distance_filter_.SetRate(
        -4.0, config_.cut_in_desired_distance_step);
    cut_in_desired_distance_filter_.Update(desired_distance);
    desired_distance_new = cut_in_desired_distance_filter_.GetOutput();
    JSON_DEBUG_VALUE("fast_car_cut_in_id", cut_in_obstacle.track_id());
  }

  cutin_information->set_desired_distance(desired_distance_new);
  return desired_distance_new;
}

double StGraphGenerator::LCGapDesiredDistanceFilter(
    const planning::common::TrackedObjectInfo &lead_obstacle,
    const double v_ego, double safe_distance, double desired_distance,
    bool is_front) {
  // 更新lead初始信息
  if (is_front) {
    if (lc_front_id_ != lead_obstacle.track_id()) {
      lc_front_id_ = lead_obstacle.track_id();
      lc_front_desired_distance_ =
          std::min(lead_obstacle.d_rel(), safe_distance);
    }
    double desired_front_distance_new = desired_distance;

    // TBD: 目前cut in和lead区分不明显，cut in会被判断为lead
    bool slow_car_cut_in = false;
    if (lead_obstacle.d_rel() < desired_distance &&
        lead_obstacle.v_rel() <= 0.5) {
      slow_car_cut_in = true;
    }
    lead_desired_distance_filter_.SetState(lc_front_desired_distance_);
    if (slow_car_cut_in) {
      // 慢车切入
      lead_desired_distance_filter_.SetRate(-4.0,
                                            config_.slow_lead_distance_step);
      lead_desired_distance_filter_.Update(desired_distance);
      desired_front_distance_new = lead_desired_distance_filter_.GetOutput();
      JSON_DEBUG_VALUE("slow_lead_id", lead_obstacle.track_id());
    } else {
      // 快车切入
      lead_desired_distance_filter_.SetRate(-4.0,
                                            config_.fast_lead_distance_step);
      lead_desired_distance_filter_.Update(desired_distance);
      desired_front_distance_new = lead_desired_distance_filter_.GetOutput();
      JSON_DEBUG_VALUE("fast_lead_id", lead_obstacle.track_id());
    }
    lc_front_desired_distance_ = desired_front_distance_new;
    return desired_front_distance_new;

  } else {
    if (lc_rear_id_ != lead_obstacle.track_id()) {
      lc_rear_id_ = lead_obstacle.track_id();
      lc_rear_desired_distance_ =
          std::min(0 - lead_obstacle.d_rel(), safe_distance);
    }
    double desired_rear_distance_new = desired_distance;

    // TBD: 目前cut in和lead区分不明显，cut in会被判断为lead
    bool slow_car_cut_in = false;
    if ((0 - lead_obstacle.d_rel()) < desired_distance &&
        (0 - lead_obstacle.v_rel()) <= 0.5) {
      slow_car_cut_in = true;
    }
    lead_desired_distance_filter_.SetState(lc_rear_desired_distance_);
    if (slow_car_cut_in) {
      // 慢车切入
      lead_desired_distance_filter_.SetRate(-4.0,
                                            config_.slow_lead_distance_step);
      lead_desired_distance_filter_.Update(desired_distance);
      desired_rear_distance_new = lead_desired_distance_filter_.GetOutput();
      JSON_DEBUG_VALUE("slow_lead_id", lead_obstacle.track_id());
    } else {
      // 快车切入
      lead_desired_distance_filter_.SetRate(-4.0,
                                            config_.fast_lead_distance_step);
      lead_desired_distance_filter_.Update(desired_distance);
      desired_rear_distance_new = lead_desired_distance_filter_.GetOutput();
      JSON_DEBUG_VALUE("fast_lead_id", lead_obstacle.track_id());
    }
    lc_rear_desired_distance_ = desired_rear_distance_new;
    return desired_rear_distance_new;
  }
}

common::StartStopInfo::StateType StGraphGenerator::UpdateStartStopState(
    const planning::common::TrackedObjectInfo &lead_one, const double v_ego,
    const TrajectoryPoints &last_traj) {
  // The AION's resolution of vehicle speed is 0.3m/s
  double v_start = config_.v_start;
  double v_startmode = config_.v_startmode;
  double obstacle_v_start = config_.obstacle_v_start;
  double distance_stop = config_.distance_stop;
  double distance_start = config_.distance_start;
  constexpr double lead_change_buffer = 1.0;

  start_stop_info_.CopyFrom(lon_behav_input_->start_stop_info());
  bool dbw_status = lon_behav_input_->dbw_status();
  // 这里有问题
  if (lead_one.track_id() == 0 || dbw_status == false) {
    // reset state as default
    start_stop_info_.set_state(common::StartStopInfo::CRUISE);
  } else {
    // 1. Calculate the condition
    std::string lc_request = "none";
    double desire_distance = CalcDesiredDistance(lead_one, v_ego, lc_request);
    bool is_lead_static = std::fabs(lead_one.v_lead()) < obstacle_v_start;
    bool stop_condition =
        (v_ego < v_start && is_lead_static &&
         std::fabs(lead_one.d_rel() - desire_distance) < distance_stop);
    bool cruise_condition = v_ego > v_startmode || (v_last_target_ > v_target_);
    bool lead_one_start =
        (lead_one.v_lead() > obstacle_v_start &&
         (lead_one.d_rel() - start_stop_info_.stop_distance_of_leadone()) >
             distance_start);
    // lead_one change: obj stopped adc by cut_in, then leaved
    bool lead_one_change =
        (lead_one.d_rel() - start_stop_info_.stop_distance_of_leadone()) >
        (distance_stop + lead_change_buffer);
    bool start_condition = lead_one_start || lead_one_change;

    // 2. Update the state
    if (start_stop_info_.state() == common::StartStopInfo::CRUISE &&
        stop_condition) {
      // CRUISE --> STOP
      start_stop_info_.set_state(common::StartStopInfo::STOP);
      // store the distance of leadone
      start_stop_info_.set_stop_distance_of_leadone(lead_one.d_rel());
      LOG_DEBUG("The distance error of STOP is [%f]m \n",
                lead_one.d_rel() - desire_distance);
    } else if (start_stop_info_.state() == common::StartStopInfo::STOP &&
               start_condition) {
      // STOP --> START
      start_stop_info_.set_state(common::StartStopInfo::START);
    } else if (start_stop_info_.state() == common::StartStopInfo::START &&
               cruise_condition) {
      // START --> CRUISE
      start_stop_info_.set_state(common::StartStopInfo::CRUISE);
    } else if (start_stop_info_.state() == common::StartStopInfo::START &&
               stop_condition) {
      // START --> STOP
      start_stop_info_.set_state(common::StartStopInfo::STOP);
    }
  }
  LOG_DEBUG("The start_stop_state_info is [%d] \n", start_stop_info_.state());
  return start_stop_info_.state();
}

void StGraphGenerator::UpdateVelRefs() {
  double v_ego = lon_behav_input_->ego_info().ego_v();
  auto &function_info = lon_behav_input_->function_info();
  // ACC : STANDSTILL to ACTIVE need confirmed by driver
  JSON_DEBUG_VALUE("STANDSTILL", 0.0);
  if (v_ego < 1.0 &&
      function_info.function_mode() == common::DrivingFunctionInfo::ACC &&
      function_info.function_state() ==
          common::DrivingFunctionInfo::STANDSTILL) {
    v_target_ = 0.0;
    JSON_DEBUG_VALUE("STANDSTILL", 1.0);
  }

  for (int i = 0; i < config_.lon_num_step + 1; ++i) {
    vt_refs_[i] = std::max(std::min(vt_refs_[i], v_target_), 0.0);
  }
  JSON_DEBUG_VALUE("v_target", v_target_);
}

void StGraphGenerator::CalculateSrefsByVref(const double v_ego,
                                            std::vector<double> &v_refs,
                                            const double acc_ego,
                                            std::vector<double> &s_refs) {
  LOG_DEBUG("----entering CalculateSrefsByVref--- \n");
  double one_a = lon_init_state_[2];
  double one_v = lon_init_state_[1];
  double one_s = 0.0;
  double t = config_.delta_time;
  double t_square = config_.delta_time * config_.delta_time;
  double t_cube = config_.delta_time * config_.delta_time * config_.delta_time;
  double v_ref = v_refs[0];
  // s_refs.emplace_back(one_s);
  s_refs[0] = one_s;

  std::pair<double, double> max_acc_info = CalculateMaxAcc(v_ego);
  double a_max_accel = std::min(max_acc_info.second, acc_bound_.second);
  double a_max_brake = std::max(max_acc_info.first, acc_bound_.first);

  if (v_ego <= v_ref) {
    double one_j = _J_MAX;
    for (int i = 1; i <= config_.lon_num_step; i++) {
      one_s += std::max(
          one_v * t + 0.5 * one_a * t_square + 1.0 / 6 * one_j * t_cube, 0.0);
      one_v = std::max(
          std::min(one_v + one_a * t + 0.5 * one_j * t_square, v_refs[i]), 0.0);
      if (one_v == v_ref) {
        one_a = 0.0;
        one_j = 0.0;
      } else {
        one_a = std::min(one_a + one_j * config_.delta_time, a_max_accel);
      }
      if (one_a == a_max_accel) {
        one_j = 0.0;
      }
      // s_refs.emplace_back(one_s);
      s_refs[i] = one_s;
    }
  } else {
    double one_j = _J_MIN;
    for (int i = 1; i <= config_.lon_num_step; i++) {
      one_s += one_v * t + 0.5 * one_a * t_square + 1.0 / 6 * one_j * t_cube;
      one_v = std::max(one_v + one_a * t + 0.5 * one_j * t_square, v_ref);
      if (one_v == v_ref) {
        one_a = 0.0;
        one_j = 0.0;
      } else {
        one_a = std::max(one_a + one_j * t, a_max_brake);
      }
      if (one_a == a_max_brake) {
        one_j = 0.0;
      }
      // s_refs.emplace_back(one_s);
      s_refs[i] = one_s;
    }
  }
}

void StGraphGenerator::MakeAccBound() {
  double acc_upper_bound_with_lower_speed = config_.lower_speed_acc_upper_bound;
  double acc_upper_bound_with_high_speed = config_.high_speed_acc_upper_bound;
  // larger acc bound for lane change
  const auto &lc_request = lon_behav_input_->lat_output().lc_request();
  const auto &lc_status = lon_behav_input_->lat_output().lc_status();
  const bool is_in_lane_change =
      ((lc_request != "none") && (lc_status != "none"));
  if (is_in_lane_change) {
    acc_upper_bound_with_lower_speed =
        config_.lane_change_low_speed_acc_upper_bound;
    acc_upper_bound_with_high_speed =
        config_.lane_change_high_speed_acc_upper_bound;
  }
  double acc_upper_bound_with_speed = planning_math::LerpWithLimit(
      acc_upper_bound_with_lower_speed,
      config_.low_speed_threshold_with_acc_upper_bound,
      acc_upper_bound_with_high_speed,
      config_.high_speed_threshold_with_acc_upper_bound, lon_init_state_[1]);
  // acc_bound_.first = (std::fmin(lon_init_state_[2],
  // config_.acc_lower_bound)); acc_bound_.second =
  //     (std::fmax(lon_init_state_[2], acc_upper_bound_with_speed));
  acc_bound_.first = (std::fmin(lon_init_state_[2], acc_target_.first));
  acc_bound_.second =
      std::fmin((std::fmax(lon_init_state_[2], acc_target_.second)), 1.0);
  // TODO: config_.v_target_stop_thrd(0.3) doesn't work in eoy, but need to work
  // in gasoline car
  if (start_stop_info_.state() == common::StartStopInfo::START) {
    acc_bound_.first = -config_.acc_start_max_bound;
    acc_bound_.second = config_.acc_start_max_bound;
  } else if (v_target_ < config_.v_target_stop_thrd &&
             start_stop_info_.state() != common::StartStopInfo::START) {
    acc_bound_.first = -config_.acc_stop_max_bound;
    acc_bound_.second = config_.acc_stop_max_bound;
  }
}

void StGraphGenerator::MakeJerkBound() {
  double jerk_upper_bound = config_.jerk_upper_bound;
  if (lon_init_state_[2] > 0.0) {
    jerk_upper_bound = config_.kSlowJerkUpperBound;
  }
  double jerk_lower_bound = config_.jerk_lower_bound;
  jerk_bound_.second = jerk_upper_bound;
  jerk_bound_.first = jerk_lower_bound;
}

void StGraphGenerator::CalculateNarrowLimitSpeed(
    const planning::common::LatObsInfo &lateral_obstacles,
    std::shared_ptr<planning::planning_data::DynamicWorld> dynamic_world,
    std::shared_ptr<VirtualLane> current_lane,
    std::vector<planning::common::RealTimeLonObstacleSTInfo> &leads_st_info) {
  if (!config_.enable_narrow_agent_limit) {
    return;
  }
  // 1) 构造横向KDPath
  std::vector<planning_math::PathPoint> lat_path_points;
  lat_path_points.reserve(lon_behav_input_->lat_output().spline_x_vec_size());
  const auto &spline_x_vec = lon_behav_input_->lat_output().spline_x_vec();
  const auto &spline_y_vec = lon_behav_input_->lat_output().spline_y_vec();
  for (int i = 0; i <= config_.lon_num_step; ++i) {
    if (std::isnan(spline_x_vec[i]) || std::isnan(spline_y_vec[i])) {
      LOG_ERROR("skip NaN point");
      continue;
    }
    planning_math::PathPoint path_point{spline_x_vec[i], spline_y_vec[i]};
    lat_path_points.emplace_back(path_point);
    if (not lat_path_points.empty()) {
      auto &last_pt = lat_path_points.back();
      if (planning_math::Vec2d(last_pt.x() - path_point.x(),
                               last_pt.y() - path_point.y())
              .Length() < 1e-3) {
        continue;
      }
    }
  }
  if (lat_path_points.size() <= 2) {
    return;
  }
  lat_path_coord_ = std::make_shared<KDPath>(std::move(lat_path_points));

  // 2) 获取障碍物
  narrow_agent_.clear();
  const auto &agent_manager = dynamic_world->agent_manager();
  const auto &all_current_agents = agent_manager->GetAllCurrentAgents();
  if (all_current_agents.empty()) {
    return;
  }

  for (auto agent : all_current_agents) {
    if (agent == nullptr) {
      continue;
    }
    // check static agent
    if (agent->speed() > kStaticAgentSpeedThr && (!agent->is_static())) {
      continue;
    }
    // check fusion source
    if ((agent->fusion_source() & OBSTACLE_SOURCE_CAMERA) == 0) {
      continue;
    }

    int agent_id = agent->agent_id();
    bool is_lead_one = false;
    bool is_lead_two = false;
    bool is_temp_one = false;
    bool is_temp_two = false;
    if (lateral_obstacles.lead_one().track_id() != -1) {
      is_lead_one =
          lateral_obstacles.lead_one().track_id() == agent_id ? true : false;
    }
    if (lateral_obstacles.lead_two().track_id() != -1) {
      is_lead_two =
          lateral_obstacles.lead_two().track_id() == agent_id ? true : false;
    }
    if (lateral_obstacles.temp_lead_one().track_id() != -1) {
      is_temp_one = lateral_obstacles.temp_lead_one().track_id() == agent_id
                        ? true
                        : false;
    }
    if (lateral_obstacles.temp_lead_two().track_id() != -1) {
      is_temp_two = lateral_obstacles.temp_lead_two().track_id() == agent_id
                        ? true
                        : false;
    }
    // ignore leadone or leadtwo
    if (is_lead_one || is_lead_two || is_temp_one || is_temp_two) {
      continue;
    }

    // 3) 侵入车道检查
    // 3.1 check intrude into ego lane (using current lane reference path)
    double agent_s = 0.0;
    double agent_l = 0.0;
    const auto current_lane_frenet_coord =
        current_lane->get_reference_path()->get_frenet_coord();
    if (!(current_lane_frenet_coord->XYToSL(agent->x(), agent->y(), &agent_s,
                                            &agent_l))) {
      continue;
    }
    // ignore narrow agent if lead closer
    if (lateral_obstacles.lead_one().track_id() != -1 &&
        lateral_obstacles.lead_one().d_rel() < agent_s) {
      continue;
    }
    if (lateral_obstacles.temp_lead_one().track_id() != -1 &&
        lateral_obstacles.temp_lead_one().d_rel() < agent_s) {
      continue;
    }
    // 3.2 calclate ego sl info (using current lane reference path)
    double ego_s = 0.0;
    double ego_l = 0.0;
    double ego_pose_x = lon_behav_input_->ego_info().ego_pose_x();
    double ego_pose_y = lon_behav_input_->ego_info().ego_pose_y();
    if (!(current_lane_frenet_coord->XYToSL(ego_pose_x, ego_pose_y, &ego_s,
                                            &ego_l))) {
      continue;
    }
    // 3.3 min/max sl
    double min_s = std::numeric_limits<double>::max();
    double max_s = std::numeric_limits<double>::lowest();
    double min_l = std::numeric_limits<double>::max();
    double max_l = std::numeric_limits<double>::lowest();
    CalculateAgentSLBoundary(current_lane_frenet_coord, *agent, &min_s, &max_s,
                             &min_l, &max_l);
    double min_lat_l = 0.0;
    if (agent_l >= 0) {
      min_lat_l = (agent_l * min_l) > 0 ? min_l : 0;
    } else {
      min_lat_l = (agent_l * max_l) > 0 ? max_l : 0;
    }
    double half_lane_width_by_s = 0.5 * current_lane->width_by_s(min_s);
    double invade_thr = half_lane_width_by_s - kStaticAgentBuffer;
    // invade thr: (1.1, half_lane_width - 0.12)
    if (fabs(min_lat_l) > invade_thr || fabs(min_lat_l) < kStaticLeadThr) {
      continue;
    }
    // 3.4 check agent is front of ego ?
    if (min_s - ego_s < 0.0) {
      continue;
    }

    // 4) 安全性检查
    // 4.1 calculate min_s and min_l (using lateral path)
    double min_s_by_lat_path = std::numeric_limits<double>::max();
    double max_s_by_lat_path = std::numeric_limits<double>::lowest();
    double min_l_by_lat_path = std::numeric_limits<double>::max();
    double max_l_by_lat_path = std::numeric_limits<double>::lowest();
    CalculateAgentSLBoundary(lat_path_coord_, *agent, &min_s_by_lat_path,
                             &max_s_by_lat_path, &min_l_by_lat_path,
                             &max_l_by_lat_path);
    double min_lat_l_by_lat_path = 0.0;
    if (agent_l >= 0) {
      min_lat_l_by_lat_path =
          (agent_l * min_l_by_lat_path) > 0 ? min_l_by_lat_path : 0;
    } else {
      min_lat_l_by_lat_path =
          (agent_l * max_l_by_lat_path) > 0 ? max_l_by_lat_path : 0;
    }
    // 4.2 collision check
    double end_s = min_s + kHalfEgoLength * 2 + agent->length();
    bool is_collison =
        LateralCollisionCheck(min_s, end_s, min_lat_l_by_lat_path);

    // calc v_limit
    // std::array<double, 5> xp1{0, 5, 40, 80, 120};
    // std::array<double, 5> fp1{1.28, 1.28, 1.21, 1.19, 1.1};
    // double lead_one_thr = interp(min_s, xp1, fp1);

    double v_ego = lon_behav_input_->ego_info().ego_v();
    double s_target = GetCalibratedDistance(
        agent->speed(), v_ego, lon_behav_input_->lat_output().lc_request(),
        false, false, false);
    double s_safe = CalcSafeDistance(agent->speed(), v_ego);
    double v_target = CalcDesiredVelocity(min_s, s_target, agent->speed());
    double avoid_offset =
        std::max(fabs(min_lat_l_by_lat_path) - fabs(min_lat_l), 0.0);
    std::array<double, 2> xp2{kStaticAgentPosThr, invade_thr + avoid_offset};
    std::array<double, 2> fp2{v_target, v_ego};
    double v_limit_narrow = interp(fabs(min_lat_l_by_lat_path), xp2, fp2);
    if (is_collison) {
      v_limit_narrow = v_target;
    }

    NarrowLead narrow_lead;
    narrow_lead.id = agent_id;
    narrow_lead.desire_distance = s_target;
    narrow_lead.min_s = min_s;
    narrow_lead.safe_distance = s_safe;
    narrow_lead.v_limit = v_limit_narrow;
    narrow_lead.is_collison = is_collison;

    narrow_agent_.emplace_back(narrow_lead);

    // sort
    auto comp = [](const NarrowLead &a, const NarrowLead &b) {
      return a.min_s < b.min_s;
    };
    std::sort(narrow_agent_.begin(), narrow_agent_.end(), comp);
  }

  if (narrow_agent_.empty()) {
    return;
  }

  auto it = narrow_agent_.begin();
  if (it != narrow_agent_.end()) {
    if (it->is_collison) {
      // if have collison, update lead one st
      common::RealTimeLonObstacleSTInfo lead_one_st_info;
      lead_one_st_info.set_st_type(common::RealTimeLonObstacleSTInfo::LEADS);
      lead_one_st_info.set_id(it->id);
      lead_one_st_info.set_a_lead(0.0);
      lead_one_st_info.set_v_lead(0.0);
      lead_one_st_info.set_s_lead(it->min_s);
      lead_one_st_info.set_desired_distance(it->desire_distance);
      lead_one_st_info.set_desired_velocity(it->v_limit);
      lead_one_st_info.set_safe_distance(it->safe_distance);
      lead_one_st_info.set_start_time(0.0);  // TBD:使用可配置参数
      lead_one_st_info.set_end_time(5.0);    // TBD:使用可配置参数
      lead_one_st_info.set_start_s(it->min_s);
      leads_st_info.emplace_back(lead_one_st_info);
      v_target_ = std::min(v_target_, it->v_limit);
      JSON_DEBUG_VALUE("narrow_agent_id", it->id)
      JSON_DEBUG_VALUE("narrow_agent_v_limit", v_target_)
    } else {
      v_target_ = std::min(v_target_, it->v_limit);
      JSON_DEBUG_VALUE("narrow_agent_id", it->id)
      JSON_DEBUG_VALUE("narrow_agent_v_limit", v_target_)
    }
  }
}

bool StGraphGenerator::LateralCollisionCheck(const double &start_s,
                                             const double &end_s,
                                             const double &agent_min_l) {
  const double sampling_interval_s = 1.0;
  for (double s = start_s; s <= end_s; s += sampling_interval_s) {
    double box_x = 0.0;
    double box_y = 0.0;
    lat_path_coord_->SLToXY(s, 0.0, &box_x, &box_y);
    double theta = lat_path_coord_->GetPathCurveHeading(s);
    planning_math::Box2d ego_box({box_x, box_y}, theta, kEgoLength, kEgoWidth);
    const auto &all_corners = ego_box.GetAllCorners();
    double min_l = std::numeric_limits<double>::max();
    double max_l = -min_l;

    for (const auto &corner : all_corners) {
      double ego_s = 0.0;
      double ego_l = 0.0;
      lat_path_coord_->XYToSL(corner.x(), corner.y(), &ego_s, &ego_l);
      min_l = std::fmin(min_l, ego_l);
      max_l = std::fmax(max_l, ego_l);
    }

    // check collision
    if ((agent_min_l < 0 && min_l < agent_min_l) ||
        (agent_min_l >= 0 && max_l > agent_min_l)) {
      return true;
    }
  }
  return false;
}

void StGraphGenerator::SetConfig(
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
}

}  // namespace scc
}  // namespace planning
