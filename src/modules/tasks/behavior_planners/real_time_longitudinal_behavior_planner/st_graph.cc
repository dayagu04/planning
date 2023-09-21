#include "st_graph.h"

#include "debug_info_log.h"
#include "ego_planning_config.h"
#include "lane_change_requests/lane_change_lane_manager.h"
#include "planning_context.h"
#include "scenario_state_machine.h"
#include "task_basic_types.h"
#include "task_basic_types.pb.h"
#include "virtual_lane_manager.h"

namespace planning {

StGraphGenerator::StGraphGenerator(
    const RealTimeLonBehaviorPlannerConfig &config)
    : config_(config) {
  lead_desired_distance_filter_.Init(-0.2, config_.fast_lead_distance_step, 0.0,
                                     150.0, 0.1);
  cut_in_desired_distance_filter_.Init(
      -0.2, config_.cut_in_desired_distance_step, 0.0, 150.0, 0.1);
}

void StGraphGenerator::Update(
    std::shared_ptr<common::RealTimeLonBehaviorInput> lon_behav_input) {
  lon_behav_input_ = std::move(lon_behav_input);
  LOG_DEBUG("=======Entering StGraphGenerator::Update======= \n");
  double v_ego = lon_behav_input_->ego_info().ego_v();
  double v_cruise = lon_behav_input_->ego_info().ego_cruise();
  double acc_ego = lon_behav_input_->ego_info().ego_acc();

  if (lane_changing_decider_ == nullptr) {
    lane_changing_decider_ = std::make_unique<RealTimeLaneChangeDecider>(
        lon_behav_input_->lc_info());
  }

  st_refs_.clear();
  st_boundaries_.clear();
  JSON_DEBUG_VALUE("RealTime_v_ego", v_ego);

  // 0. get start & stop state
  common::StartStopInfo::StateType stop_start_state =
      UpdateStartStopState(lon_behav_input_->lat_obs_info().lead_one(), v_ego);
  v_cruise = stop_start_state == common::StartStopInfo::STOP ? 0.0 : v_cruise;

  // 初始化v_refs
  v_target_ = v_cruise;
  vt_refs_.resize(config_.lon_num_step + 1);
  for (int i = 0; i < config_.lon_num_step + 1; ++i) {
    vt_refs_[i] = v_cruise;
  }
  // 1. 计算巡航s_ref
  std::vector<double> sref_vec;
  sref_vec.reserve(config_.lon_num_step + 1);

  CalculateCruiseSrefs(v_ego, v_cruise, acc_ego, sref_vec);

  // 2. 计算障碍物s-t
  // 2.1 计算leads: lead one, 选择性使用lead two
  std::vector<planning::common::RealTimeLonObstacleSTInfo> leads_st_info;
  CalcSpeedInfoWithLead(lon_behav_input_->lat_obs_info().lead_one(),
                        lon_behav_input_->lat_obs_info().lead_two(),
                        lon_behav_input_->lat_output().lc_request(), v_ego,
                        leads_st_info);

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

  std::vector<common::RealTimeLonObstacleSTInfo> st_infos;
  int st_infos_num = leads_st_info.size() + temp_leads_st_info.size() +
                     cut_in_st_info.size() + lane_change_st_info.size();
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

  // 3. update STboundaries & sref
  UpdateSTGraphs(st_infos, sref_vec);
  // 4. update v ref
  UpdateVelRefs();
}

bool StGraphGenerator::CalcSpeedInfoWithLead(
    const planning::common::TrackedObjectInfo &lead_one,
    const planning::common::TrackedObjectInfo &lead_two,
    const string &lc_request, const double v_ego,
    std::vector<planning::common::RealTimeLonObstacleSTInfo> &leads_st_info) {
  double lead_one_a_processed = 0.0;
  double lead_one_desired_distance = 0.0;
  double safe_distance = 0.0;
  double lead_one_desired_velocity = 0.0;
  double lead_two_a_processed = 0.0;
  double lead_two_desired_distance = 0.0;
  double lead_two_desired_velocity = 0.0;
  double desired_distance_filtered = 0.0;

  LOG_DEBUG("----compute_speed_with_leads--- \n");
  if (lead_one.track_id() != 0 &&
      lead_one.type() != Common::ObjectType::OBJECT_TYPE_UNKNOWN) {
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

    JSON_DEBUG_VALUE("RealTime_lead_one_id", lead_one.track_id());
    JSON_DEBUG_VALUE("RealTime_lead_one_distance", lead_one.d_rel());
    JSON_DEBUG_VALUE("RealTime_lead_one_velocity", lead_one.v_lead());
    JSON_DEBUG_VALUE("RealTime_lead_one_desire_vel", lead_one_desired_velocity);
    JSON_DEBUG_VALUE("REALTIME_desired_distance", desired_distance_filtered);

    // 对lead two进行类似的计算
    if (config_.enable_lead_two && lead_two.track_id() != 0 &&
        lead_two.type() != Common::ObjectType::OBJECT_TYPE_UNKNOWN) {
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

      // update lead two st
      planning::common::RealTimeLonObstacleSTInfo lead_two_st_info;
      lead_two_st_info.set_st_type(common::RealTimeLonObstacleSTInfo::LEADS);
      lead_two_st_info.set_id(lead_two.track_id());
      lead_two_st_info.set_a_lead(lead_two_a_processed);
      lead_two_st_info.set_v_lead(lead_two.v_lead());
      lead_two_st_info.set_s_lead(lead_two.d_rel());
      lead_two_st_info.set_desired_distance(lead_two_desired_distance);
      lead_two_st_info.set_desired_velocity(lead_two_desired_velocity);
      lead_two_st_info.set_safe_distance(safe_distance);
      lead_two_st_info.set_start_time(0.0);  // TBD:使用可配置参数
      lead_two_st_info.set_end_time(5.0);    // TBD:使用可配置参数
      lead_two_st_info.set_start_s(lead_two.d_rel());
      leads_st_info.emplace_back(lead_two_st_info);

      v_target_ = std::min(v_target_, lead_two_desired_velocity);

      JSON_DEBUG_VALUE("RealTime_lead_two_id", lead_two.track_id());
      JSON_DEBUG_VALUE("RealTime_lead_two_distance", lead_two.d_rel());
      JSON_DEBUG_VALUE("RealTime_lead_two_velocity", lead_two.v_lead());
      JSON_DEBUG_VALUE("RealTime_lead_two_desire_vel",
                       lead_two_desired_velocity);
    }
  } else {
    LOG_DEBUG("There is no lead \n");
    lon_behav_input_->mutable_lon_decision_info()
        ->mutable_leadone_info()
        ->set_has_leadone(false);

    JSON_DEBUG_VALUE("RealTime_lead_one_id", 0);
    JSON_DEBUG_VALUE("RealTime_lead_one_distance", 0);
    JSON_DEBUG_VALUE("RealTime_lead_one_velocity", 0);
    JSON_DEBUG_VALUE("RealTime_lead_one_desire_vel", 0);
    JSON_DEBUG_VALUE("RealTime_lead_two_id", 0);
    JSON_DEBUG_VALUE("RealTime_lead_two_distance", 0);
    JSON_DEBUG_VALUE("RealTime_lead_two_velocity", 0);
    JSON_DEBUG_VALUE("RealTime_lead_two_desire_vel", 0);
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
  double temp_lead_one_desired_velocity = 0.0;
  double temp_lead_two_a_processed = 0.0;
  double temp_lead_two_desired_distance = 0.0;
  double temp_lead_two_desired_velocity = 0.0;

  LOG_DEBUG("----CalcSpeedInfoWithTempLead--- \n");
  // temp leadone
  if (temp_lead_one.track_id() != 0 && !lateral_outputs.close_to_accident() &&
      (temp_lead_one.d_path_self() + std::min(temp_lead_one.v_lat(), 0.3)) <
          1.0 &&
      temp_lead_one.type() != Common::ObjectType::OBJECT_TYPE_UNKNOWN) {
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

    JSON_DEBUG_VALUE("RealTime_temp_lead_one_id", temp_lead_one.track_id());
    JSON_DEBUG_VALUE("RealTime_temp_lead_one_distance", temp_lead_one.d_rel());
    JSON_DEBUG_VALUE("RealTime_temp_lead_one_velocity", temp_lead_one.v_lead());
    JSON_DEBUG_VALUE("RealTime_temp_lead_one_desire_vel",
                     temp_lead_one_desired_velocity);

    // 对lead two进行类似的计算
    if (config_.enable_lead_two && temp_lead_two.track_id() != 0 &&
        temp_lead_two.type() != Common::ObjectType::OBJECT_TYPE_UNKNOWN) {
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

      JSON_DEBUG_VALUE("RealTime_temp_lead_two_id", temp_lead_two.track_id());
      JSON_DEBUG_VALUE("RealTime_temp_lead_two_distance",
                       temp_lead_two.d_rel());
      JSON_DEBUG_VALUE("RealTime_temp_lead_two_velocity",
                       temp_lead_two.v_lead());
      JSON_DEBUG_VALUE("RealTime_temp_lead_two_desire_vel",
                       temp_lead_two_desired_velocity);
    }
  } else {
    LOG_DEBUG("There is no temp lead \n");

    JSON_DEBUG_VALUE("RealTime_temp_lead_one_id", 0);
    JSON_DEBUG_VALUE("RealTime_temp_lead_one_distance", 0);
    JSON_DEBUG_VALUE("RealTime_temp_lead_one_velocity", 0);
    JSON_DEBUG_VALUE("RealTime_temp_lead_one_desire_vel", 0);
    JSON_DEBUG_VALUE("RealTime_temp_lead_two_id", 0);
    JSON_DEBUG_VALUE("RealTime_temp_lead_two_distance", 0);
    JSON_DEBUG_VALUE("RealTime_temp_lead_two_velocity", 0);
    JSON_DEBUG_VALUE("RealTime_temp_lead_two_desire_vel", 0);
  }
  return true;
}

void StGraphGenerator::CalcSpeedInfoWithCutin(
    const planning::common::LatObsInfo &lateral_obstacles,
    const std::string &lc_request, double v_cruise, double v_ego,
    std::vector<planning::common::RealTimeLonObstacleSTInfo> &cut_in_st_info) {
  constexpr double velocity_increase_cutin = 0.005;
  constexpr double p1min_speed = 2.0;
  constexpr double p2min_speed = 3.0;

  std::vector<const planning::common::TrackedObjectInfo *> near_cars,
      near_cars_sorted;
  // 更新周围障碍物的cut in信息，纵向评估的cut in,不用做计算
  UpdateNearObstacles(lateral_obstacles, lc_request, v_ego);

  UpdateSpeedWithPotentialCutinCar(lateral_obstacles, lc_request, v_cruise,
                                   v_ego, cut_in_st_info);
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
  Bound soft_bound;
  Bound hard_bound;
  double s_ref;
  double s_ref_update;
  std::vector<double> sref_update;

  // sref_update.reserve(config_.lon_num_step+1);
  sref_update = sref_vec;
  st_boundaries_.reserve(st_infos.size());
  for (auto &st : st_infos) {
    // 1.设置boundary类型，是否必要？
    real_time::STBoundary st_boundary;
    st_boundary.id = st.id();
    switch (st.st_type()) {
      case common::RealTimeLonObstacleSTInfo::LEADS:
        st_boundary.boundary_type = real_time::BoundaryType::YIELD;
        break;
      case common::RealTimeLonObstacleSTInfo::TEMP_LEADS:
        st_boundary.boundary_type = real_time::BoundaryType::YIELD;
        break;
      case common::RealTimeLonObstacleSTInfo::CUT_IN:
        st_boundary.boundary_type = real_time::BoundaryType::YIELD;
        break;
      case common::RealTimeLonObstacleSTInfo::GAP:
        st_boundary.boundary_type =
            st.decision() == common::RealTimeLonObstacleSTInfo::YIELD
                ? real_time::BoundaryType::YIELD
                : real_time::BoundaryType::OVERTAKE;
        break;
      default:
        st_boundary.boundary_type = real_time::BoundaryType::UNKNOWN;
    }
    // 2.将st信息转换为离散bounds
    for (unsigned int i = 0; i <= config_.lon_num_step; i++) {
      sample_time = i * config_.delta_time;
      // 只更新关注的t区间内
      if (sample_time >= st.start_time() && sample_time <= st.end_time()) {
        double s_step = st.v_lead() * sample_time;
        // 考虑decision type是overtake的情况
        if (st.decision() == common::RealTimeLonObstacleSTInfo::YIELD) {
          if (st.st_type() == common::RealTimeLonObstacleSTInfo::GAP) {
            s_ref = st.start_s() - st.desired_distance() +
                    st.desired_velocity() * sample_time;
          } else {
            s_ref = st.start_s() - st.desired_distance() + s_step;
          }
          // hard bound使用安全距离
          hard_bound.upper = st.start_s() - st.safe_distance() + s_step;
          hard_bound.lower = 0.0;  // 应该至少使用自车s-10
          st_boundary.hard_bound.emplace_back(hard_bound);
          s_ref_update = std::min(
              hard_bound.upper, std::min(sref_update[i], std::max(s_ref, 0.0)));
          soft_bound.upper = std::min(
              0.5 * (hard_bound.upper + s_ref_update),
              std::max(st.start_s() - st.desired_distance() + s_step, 0.0));
          soft_bound.lower = 0.0;  // 应该至少使用自车s-10
          st_boundary.soft_bound.emplace_back(soft_bound);
          // 根据障碍物跟车距离刷新s_refs
          sref_update[i] = s_ref_update;
        } else {
          s_ref = st.start_s() + st.desired_distance() + s_step;
          // hard bound使用安全距离
          hard_bound.upper = 150;
          hard_bound.lower = st.start_s() + st.safe_distance() + s_step;
          st_boundary.hard_bound.emplace_back(hard_bound);
          s_ref_update = std::max(
              hard_bound.lower, s_ref_update = std::max(sref_update[i], s_ref));
          // soft bound先使用期望跟车距离+buffer
          soft_bound.upper = 150;
          soft_bound.lower =
              std::max(hard_bound.lower, s_ref + st.v_lead() * 0.5);
          st_boundary.soft_bound.emplace_back(soft_bound);
          // 根据障碍物跟车距离刷新s_refs
          sref_update[i] = s_ref_update;
        }
      } else {
        // 采用默认值,这个目前太过粗暴
        hard_bound.upper = 150.0;
        hard_bound.lower = -10.0;
        st_boundary.hard_bound.emplace_back(hard_bound);
        soft_bound.upper = 150.0;
        soft_bound.lower = -10.0;  // 应该至少使用自车s-10
        st_boundary.soft_bound.emplace_back(soft_bound);
      }
    }
    st_boundaries_.emplace_back(st_boundary);
  }
  st_refs_ = sref_update;
}

void StGraphGenerator::UpdateNearObstacles(
    const planning::common::LatObsInfo &lateral_obstacles,
    const string &lc_request, double v_ego) {}

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
  double v_relative = std::min(std::max(v_ego - v_lead, 0.0), 5.0);
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
            d_offset + v_lead * t_gap + distance_hysteresis);
  return d_offset + v_lead * t_gap + distance_hysteresis;
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
  double desired_velocity = 0.0;
  double time_to_entry = 0.0;
  double predict_distance = 0.0;
  double desired_distance_filtered = 0.0;

  double v_target_potential_cutin = 40.0;
  double v_limit = std::min(v_cruise, v_ego);

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
    if ((track.fusion_source() != OBSTACLE_SOURCE_CAMERA) &&
        (track.fusion_source() != OBSTACLE_SOURCE_F_RADAR_CAMERA)) {
      continue;
    };
    if (!track.is_lead() && track.cutinp() > cutinp_threshold &&
        track.v_lat() < -0.01 &&
        track.type() != Common::ObjectType::OBJECT_TYPE_UNKNOWN) {
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
      // 通过切入概率更新目标车速
      v_target_potential_cutin =
          std::min(v_target_potential_cutin,
                   (desired_velocity - v_limit) * track.cutinp() + v_limit);

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
      st_info.set_start_s(time_to_entry * v_ego + predict_distance);
      cut_in_st_info.emplace_back(st_info);
      v_target_ = std::min(v_target_, v_target_potential_cutin);

      LOG_DEBUG("potential_cutin_car's id: [%d], track.v_lat is: [%f]\n",
                track.track_id(), track.v_lat());
      LOG_DEBUG(
          "desired_distance: [%f], desired_velocity: [%f], "
          "v_target_potential_cutin: [%f]\n",
          desired_distance, desired_velocity, v_target_potential_cutin);

      JSON_DEBUG_VALUE("RealTime_potential_cutin_track_id", track.track_id());
      JSON_DEBUG_VALUE("RealTime_potential_cutin_v_target",
                       v_target_potential_cutin);
      JSON_DEBUG_VALUE("REALTIME_cutin_desired_distance",
                       desired_distance_filtered);

    } else {
      cut_in_info->Clear();
      JSON_DEBUG_VALUE("RealTime_potential_cutin_track_id", 0);
      JSON_DEBUG_VALUE("RealTime_potential_cutin_v_target", 0);
    }
  }
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

  double v_rel_des = 0.0;
  if (d_rel < d_des) {
    // calculate v_rel_des on the line that connects 0m at max_runaway_speed
    // to d_des
    double v_rel_des_1 = (-max_runaway_speed) / d_des * (d_rel - d_des);
    // calculate v_rel_des on one third of the linear slope
    double v_rel_des_2 = (d_rel - d_des) * l_slope / 3.0;
    // take the min of the 2 above
    v_rel_des = std::min(v_rel_des_1, v_rel_des_2);
    v_rel_des = std::max(v_rel_des, max_runaway_speed);
  } else if (d_rel < d_des + x_linear_to_parabola) {
    v_rel_des = (d_rel - d_des) * l_slope;
    v_rel_des = std::max(v_rel_des, max_runaway_speed);
  } else {
    v_rel_des = std::sqrt(2 * (d_rel - d_des - x_parabola_offset) * p_slope);
  }
  // compute desired speed
  double v_target = v_rel_des + v_lead;
  LOG_DEBUG("v_rel_des : [%f], v_target : [%f] \n", v_rel_des, v_target);
  return v_target;
}

void StGraphGenerator::CalcSpeedInfoWithGap(
    const planning::common::TrackedObjectInfo &lead_one, const double v_cruise,
    const double v_ego, const string &lc_request, const string &lc_status,
    std::vector<planning::common::RealTimeLonObstacleSTInfo>
        &lane_change_st_info) {
  LOG_DEBUG("----entering CalcSpeedInfoWithGap--- \n");
  double a_processed = 0.0;
  double desired_distance = 0.0;
  double lc_t_gap = 0.2;
  double lc_buffer = 2;
  double safe_distance =
      lane_changing_decider_->get_lc_safe_dist(lc_buffer, lc_t_gap, v_ego);
  double desired_velocity = 0.0;
  double time_to_lc = 0.0;
  double predict_distance = 0.0;
  double v_limit_lc = 40.0;
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
        if ((track.fusion_source() != OBSTACLE_SOURCE_CAMERA) &&
            (track.fusion_source() != OBSTACLE_SOURCE_F_RADAR_CAMERA)) {
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
        v_limit_lc = gap.base_car_vrel -
                     clip((safe_distance - gap.base_car_drel) / safe_distance,
                          0.0, 2.0) -
                     1.0;
        if (v_limit_lc < 0) {
          // no need to decel when front car is far away
          const std::vector<double> _V_LIMIT_DISTANCE_BP{
              safe_distance + std::max(-gap.base_car_vrel, 0.0) * 2,
              safe_distance * 2 + std::max(-gap.base_car_vrel, 0.0) * 2};
          const std::vector<double> _V_LIMIT_DISTANCE_V{1.0, 0.0};
          v_limit_lc =
              v_limit_lc * interp(gap.base_car_drel, _V_LIMIT_DISTANCE_BP,
                                  _V_LIMIT_DISTANCE_V);
        }

        v_limit_lc = std::max(v_ego - 3.0, v_ego + v_limit_lc);
      } else {
        // safe_distance = CalcSafeDistance(gap.v_rear, v_ego);
        v_limit_lc =
            gap.base_car_vrel +
            clip((safe_distance + 5.0 + gap.base_car_drel) / safe_distance, 0.0,
                 2.0) +
            1.0;
        if (v_limit_lc < 0) {
          // no need to decel when front car is far away
          const std::vector<double> _V_LIMIT_DISTANCE_BP{
              safe_distance + 5.0 + std::max(gap.base_car_vrel, 0.0) * 2,
              safe_distance * 2 + 5.0 + std::max(gap.base_car_vrel, 0.0) * 2};
          const std::vector<double> _V_LIMIT_DISTANCE_V{1.0, 0.0};
          v_limit_lc =
              v_limit_lc * interp(-gap.base_car_drel, _V_LIMIT_DISTANCE_BP,
                                  _V_LIMIT_DISTANCE_V);
        }
        v_limit_lc = std::max(v_ego - 2.8, v_ego + v_limit_lc);
      }
      if (v_limit_lc < 6.0) {
        v_limit_lc = 6.0;
      }
      JSON_DEBUG_VALUE("RealTime_gap_v_limit_lc", v_limit_lc);
    } else {
      // decelerate to check next interval
      auto nearest_rear_car = lane_changing_decider_->nearest_rear_car_track();
      lane_changing_nearest_rear_car_track_id = nearest_rear_car.id;
      v_limit_lc =
          nearest_rear_car.v_rel -
          clip((safe_distance - nearest_rear_car.d_rel) / safe_distance, 0.0,
               2.0) -
          v_ego / 10.0;
      v_limit_lc = std::max({v_ego - 3.2, v_ego + v_limit_lc,
                             6.0 + 4.0 * std::max(lc_map_decision - 2, 0)});

      JSON_DEBUG_VALUE("RealTime_gap_v_limit_lc", v_limit_lc);
    }
  } else {
    JSON_DEBUG_VALUE("RealTime_gap_v_limit_lc", 0);
  }
  v_target_ = std::min(v_target_, v_limit_lc);
}

std::pair<double, double> StGraphGenerator::CalculateMaxAcc(double ego_v) {
  // refer to the International Standard: ISO 15622-2018
  std::pair<double, double> a_max;
  if (ego_v < 5.0) {
    a_max.first = 4.0;
    a_max.second = -5.0;
  } else if (ego_v > 20.0) {
    a_max.first = 2.0;
    a_max.second = -2.5;
  } else {
    a_max.first = -2.0 / 15.0 * ego_v + 14.0 / 3.0;
    a_max.second = 1.0 / 6.0 * ego_v - 35.0 / 6.0;
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
  double one_s = 0.0;
  double one_s_step = 0.0;
  s_refs.emplace_back(one_s);

  std::pair<double, double> max_acc_info = CalculateMaxAcc(v_ego);
  double a_max_accel = std::min(max_acc_info.first, 2.0);
  double a_max_brake = max_acc_info.second;

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
  if (slow_car_cut_in) {
    // 慢车切入
    lead_desired_distance_filter_.SetRate(-4.0,
                                          config_.slow_lead_distance_step);
    lead_desired_distance_filter_.Update(desired_distance);
    desired_distance_new = lead_desired_distance_filter_.GetOutput();
    JSON_DEBUG_VALUE("REALTIME_slow_lead_id", lead_obstacle.track_id());
  } else {
    // 快车切入
    lead_desired_distance_filter_.SetRate(-4.0,
                                          config_.fast_lead_distance_step);
    lead_desired_distance_filter_.Update(desired_distance);
    desired_distance_new = lead_desired_distance_filter_.GetOutput();
    JSON_DEBUG_VALUE("REALTIME_fast_lead_id", lead_obstacle.track_id());
  }
  leadone_info->mutable_leadone_information()->set_desired_distance(
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
    JSON_DEBUG_VALUE("REALTIME_slow_lead_id", lead_obstacle.track_id());
  } else {
    // 快车切入，从切入距离开始膨胀
    lead_desired_distance_filter_.SetRate(-4.0,
                                          config_.fast_lead_distance_step);
    lead_desired_distance_filter_.Update(desired_distance);
    desired_distance_new = lead_desired_distance_filter_.GetOutput();
    JSON_DEBUG_VALUE("REALTIME_fast_lead_id", lead_obstacle.track_id());
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
  JSON_DEBUG_VALUE("REALTIME_fast_car_cut_in_id", -1.0);
  JSON_DEBUG_VALUE("REALTIME_slow_car_cut_in_id", -1.0);
  if (slow_car_cut_in) {
    // 慢车切入，膨胀速度较快
    cut_in_desired_distance_filter_.SetRate(
        -4.0, config_.cut_in_desired_distance_step - cut_in_obstacle.v_rel());
    cut_in_desired_distance_filter_.Update(desired_distance);
    desired_distance_new = cut_in_desired_distance_filter_.GetOutput();
    JSON_DEBUG_VALUE("REALTIME_slow_car_cut_in_id", cut_in_obstacle.track_id());
  } else {
    // 快车切入，缓慢膨胀
    cut_in_desired_distance_filter_.SetRate(
        -4.0, config_.cut_in_desired_distance_step);
    cut_in_desired_distance_filter_.Update(desired_distance);
    desired_distance_new = cut_in_desired_distance_filter_.GetOutput();
    JSON_DEBUG_VALUE("REALTIME_fast_car_cut_in_id", cut_in_obstacle.track_id());
  }

  cutin_information->set_desired_distance(desired_distance_new);
  return desired_distance_new;
}

common::StartStopInfo::StateType StGraphGenerator::UpdateStartStopState(
    const planning::common::TrackedObjectInfo &lead_one, const double v_ego) {
  // The AION's resolution of vehicle speed is 0.3m/s
  double v_start = config_.v_start;
  double obstacle_v_start = config_.obstacle_v_start;
  double distance_stop = config_.distance_stop;
  double distance_start = config_.distance_start;

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
    bool cruise_condition = v_ego > v_start;
    bool lead_one_start =
        (lead_one.v_lead() > obstacle_v_start &&
         (lead_one.d_rel() - start_stop_info_.stop_distance_of_leadone()) >
             distance_start);
    // lead_one change: obj stopped adc by cut_in, then leaved
    bool lead_one_change =
        (lead_one.d_rel() - desire_distance) > (distance_stop + 1.0);
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
    }
  }
  LOG_DEBUG("The start_stop_state_info is [%d] \n", start_stop_info_.state());
  return start_stop_info_.state();
}

void StGraphGenerator::UpdateVelRefs() {
  double v_ego = lon_behav_input_->ego_info().ego_v();
  double v_target_clip = v_target_;
  if (v_target_ > v_ego) {
    v_target_clip =
        clip(v_target_, v_ego, v_ego + config_.cruise_set_acc * 0.1);
  }

  for (int i = 0; i < config_.lon_num_step + 1; ++i) {
    vt_refs_[i] = std::min(vt_refs_[i], v_target_clip);
  }
  JSON_DEBUG_VALUE("RealTime_v_ref", v_target_clip);
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

}  // namespace planning
