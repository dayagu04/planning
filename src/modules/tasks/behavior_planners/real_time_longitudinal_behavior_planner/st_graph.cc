#include "st_graph.h"

#include "ego_planning_config.h"
#include "lane_change_requests/lane_change_lane_manager.h"
#include "planning_context.h"
#include "scenario_state_machine.h"
#include "task_basic_types.h"
#include "virtual_lane_manager.h"

namespace planning {

StGraphGenerator::StGraphGenerator(
    const RealTimeLonBehaviorPlannerConfig &config)
    : config_(config) {
  lead_desired_distance_filter_.Init(0.0, config_.lead_desired_distance_step,
                                     0.0, 150.0, 0.1);
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

  // 0. get start & stop state
  common::StartStopInfo::StateType stop_start_state =
      UpdateStartStopState(lon_behav_input_->lat_obs_info().lead_one(), v_ego);
  v_cruise = stop_start_state == common::StartStopInfo::STOP ? 0.0 : v_cruise;

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
  // UpdateCutInInfo(lateral_obstacles, lateral_outputs.lc_status, v_ego);
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
  int st_infos_num = leads_st_info.size() + temp_leads_st_info.size() + cut_in_st_info.size() + lane_change_st_info.size();
  st_infos.resize(st_infos_num);
  for (int i = 0; i < leads_st_info.size(); ++i) {
    st_infos[i].CopyFrom(leads_st_info[i]);
  }
  for (int i = 0; i < temp_leads_st_info.size(); ++i) {
    st_infos[leads_st_info.size() + i].CopyFrom(temp_leads_st_info[i]);
  }
  for (int i = 0; i < cut_in_st_info.size(); ++i) {
    st_infos[leads_st_info.size() + temp_leads_st_info.size() + i].CopyFrom(cut_in_st_info[i]);
  }
  for (int i = 0; i < lane_change_st_info.size(); ++i) {
    st_infos[leads_st_info.size() + temp_leads_st_info.size() + cut_in_st_info.size() + i].CopyFrom(
      lane_change_st_info[i]);
  }

  // 3. update STboundaries & sref
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
  double lead_one_desired_velocity = 0.0;
  double lead_two_a_processed = 0.0;
  double lead_two_desired_distance = 0.0;
  double lead_two_desired_velocity = 0.0;

  auto leadone_info =
      lon_behav_input_->mutable_lon_decision_info()->mutable_leadone_info();
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

    // 更新lead信息
    if (leadone_info->leadone_information().obstacle_id() !=
        lead_one.track_id()) {
      leadone_info->set_has_leadone(true);
      leadone_info->mutable_leadone_information()->set_obstacle_id(
          lead_one.track_id());
      leadone_info->mutable_leadone_information()->set_desired_distance(
          lead_one.d_rel() + safe_distance);
    }
    // 快车切入，从切入距离开始膨胀
    bool fast_car = (lead_one.v_rel() > 0.5 &&
                     lead_one.d_rel() < lead_one_desired_distance);
    lead_desired_distance_filter_.SetState(
        leadone_info->leadone_information().desired_distance());
    if (fast_car) {
      lead_desired_distance_filter_.Update(lead_one_desired_distance);
      lead_one_desired_distance = lead_desired_distance_filter_.GetOutput();
      leadone_info->mutable_leadone_information()->set_desired_distance(
          lead_one_desired_distance);
    }
    // update lead one st
    common::RealTimeLonObstacleSTInfo lead_one_st_info;
    lead_one_st_info.set_st_type(common::RealTimeLonObstacleSTInfo::LEADS);
    lead_one_st_info.set_id(lead_one.track_id());
    lead_one_st_info.set_a_lead(lead_one_a_processed);
    lead_one_st_info.set_v_lead(lead_one.v_lead());
    lead_one_st_info.set_s_lead(lead_one.d_rel());
    lead_one_st_info.set_desired_distance(lead_one_desired_distance);
    lead_one_st_info.set_desired_velocity(lead_one_desired_velocity);
    lead_one_st_info.set_safe_distance(safe_distance);
    lead_one_st_info.set_start_time(0.0);  // TBD:使用可配置参数
    lead_one_st_info.set_end_time(5.0);    // TBD:使用可配置参数
    lead_one_st_info.set_start_s(lead_one.d_rel());
    leads_st_info.emplace_back(lead_one_st_info);

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
    }
  } else {
    LOG_DEBUG("There is no lead \n");
    leadone_info->set_has_leadone(false);
  }
  return true;
}

// 需要和CalcSpeedInfoWithLead进行整合
bool StGraphGenerator::CalcSpeedInfoWithTempLead(
    const planning::common::TrackedObjectInfo &temp_lead_one,
    const planning::common::TrackedObjectInfo &temp_lead_two, double v_ego,
    const planning::common::LatOutputInfo &lateral_outputs,
    std::vector<planning::common::RealTimeLonObstacleSTInfo> &temp_leads_st_info) {
  double lead_one_a_processed = 0.0;
  double lead_one_desired_distance = 0.0;
  double safe_distance = 0.0;
  double lead_one_desired_velocity = 0.0;
  double lead_two_a_processed = 0.0;
  double lead_two_desired_distance = 0.0;
  double lead_two_desired_velocity = 0.0;

  auto leadone_info =
      lon_behav_input_->mutable_lon_decision_info()->mutable_leadone_info();
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
    lead_one_a_processed = ProcessObstacleAcc(temp_lead_one.a_lead_k());
    safe_distance = CalcSafeDistance(temp_lead_one.v_lead(), v_ego);
    // compute desired distance
    lead_one_desired_distance =
        CalcDesiredDistance(temp_lead_one, v_ego, lateral_outputs.lc_request());
    // compute desired speed
    lead_one_desired_velocity =
        CalcDesiredVelocity(temp_lead_one.d_rel(), lead_one_desired_distance,
                            temp_lead_one.v_lead());

    // 更新lead信息
    if (leadone_info->leadone_information().obstacle_id() !=
        temp_lead_one.track_id()) {
      leadone_info->set_has_leadone(true);
      leadone_info->mutable_leadone_information()->set_obstacle_id(
          temp_lead_one.track_id());
      leadone_info->mutable_leadone_information()->set_desired_distance(
          temp_lead_one.d_rel() + safe_distance);
    }
    // 快车切入，从切入距离开始膨胀
    bool fast_car = (temp_lead_one.v_rel() > 0.5 &&
                     temp_lead_one.d_rel() < lead_one_desired_distance);
    lead_desired_distance_filter_.SetState(
        leadone_info->leadone_information().desired_distance());
    if (fast_car) {
      lead_desired_distance_filter_.Update(lead_one_desired_distance);
      lead_one_desired_distance = lead_desired_distance_filter_.GetOutput();
      leadone_info->mutable_leadone_information()->set_desired_distance(
          lead_one_desired_distance);
    }

    // update lead one st
    common::RealTimeLonObstacleSTInfo lead_one_st_info;
    lead_one_st_info.set_st_type(common::RealTimeLonObstacleSTInfo::TEMP_LEADS);
    lead_one_st_info.set_decision(common::RealTimeLonObstacleSTInfo::YIELD);
    lead_one_st_info.set_id(temp_lead_one.track_id());
    lead_one_st_info.set_a_lead(lead_one_a_processed);
    lead_one_st_info.set_v_lead(temp_lead_one.v_lead());
    lead_one_st_info.set_s_lead(temp_lead_one.d_rel());
    lead_one_st_info.set_desired_distance(lead_one_desired_distance);
    lead_one_st_info.set_desired_velocity(lead_one_desired_velocity);
    lead_one_st_info.set_safe_distance(safe_distance);
    lead_one_st_info.set_start_time(0.0);  // TBD:使用可配置参数
    lead_one_st_info.set_end_time(5.0);    // TBD:使用可配置参数
    lead_one_st_info.set_start_s(temp_lead_one.d_rel());
    temp_leads_st_info.emplace_back(lead_one_st_info);

    // 对lead two进行类似的计算
    if (config_.enable_lead_two && temp_lead_two.track_id() != 0 &&
        temp_lead_two.type() != Common::ObjectType::OBJECT_TYPE_UNKNOWN) {
      LOG_DEBUG(
          "target_temp_lead_two's id : [%i], d_rel is : [%f], v_lead is: "
          "[%f]\n",
          temp_lead_two.track_id(), temp_lead_two.d_rel(),
          temp_lead_two.v_lead());
      lead_two_a_processed = ProcessObstacleAcc(temp_lead_two.a_lead_k());
      safe_distance = CalcSafeDistance(temp_lead_two.v_lead(), v_ego);
      lead_two_desired_distance = CalcDesiredDistance(
          temp_lead_two, v_ego, lateral_outputs.lc_request());
      // leave enough space for leadOne
      lead_two_desired_distance += 7.0;
      lead_two_desired_velocity =
          CalcDesiredVelocity(temp_lead_two.d_rel(), lead_two_desired_distance,
                              temp_lead_two.v_lead());

      // update lead two st
      planning::common::RealTimeLonObstacleSTInfo lead_two_st_info;
      lead_two_st_info.set_st_type(common::RealTimeLonObstacleSTInfo::TEMP_LEADS);
      lead_two_st_info.set_id(temp_lead_two.track_id());
      lead_two_st_info.set_a_lead(lead_two_a_processed);
      lead_two_st_info.set_v_lead(temp_lead_two.v_lead());
      lead_two_st_info.set_s_lead(temp_lead_two.d_rel());
      lead_two_st_info.set_desired_distance(lead_two_desired_distance);
      lead_two_st_info.set_desired_velocity(lead_two_desired_velocity);
      lead_two_st_info.set_safe_distance(safe_distance);
      lead_two_st_info.set_start_time(0.0);  // TBD:使用可配置参数
      lead_two_st_info.set_end_time(5.0);    // TBD:使用可配置参数
      lead_two_st_info.set_start_s(temp_lead_two.d_rel());
      temp_leads_st_info.emplace_back(lead_two_st_info);
    }
  } else {
    LOG_DEBUG("There is no temp lead \n");
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

  UpdateSpeedWithPotentialCutinCar(lateral_obstacles, lc_request, v_cruise, v_ego,
                                   cut_in_st_info);
}

void StGraphGenerator::UpdateSTRefs(const std::vector<double> &sref_vec) {
  st_refs_ = sref_vec;
}

void StGraphGenerator::UpdateSTGraphs(
    const std::vector<common::RealTimeLonObstacleSTInfo> &st_infos,
    const std::vector<double> &sref_vec) {
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
        // 考虑decision type是overtake的情况
        if (st.decision() == common::RealTimeLonObstacleSTInfo::YIELD) {
          if (st.st_type() == common::RealTimeLonObstacleSTInfo::GAP) {
            s_ref = st.s_lead() - st.desired_distance() +
                    st.desired_velocity() * sample_time;
          } else {
            s_ref =
                st.s_lead() - st.desired_distance() + st.v_lead() * sample_time;
          }
          // hard bound使用安全距离
          hard_bound.upper =
              st.s_lead() - st.safe_distance() + st.v_lead() * sample_time;
          hard_bound.lower = 0.0;  // 应该至少使用自车s-10
          st_boundary.hard_bound.emplace_back(hard_bound);
          s_ref_update =
              std::min(hard_bound.upper, std::min(sref_update[i], s_ref));
          // soft bound先使用期望跟车距离+buffer
          soft_bound.upper =
              std::min(hard_bound.upper, s_ref_update + st.v_lead() * 0.5);
          soft_bound.lower = 0.0;  // 应该至少使用自车s-10
          st_boundary.soft_bound.emplace_back(soft_bound);
          // 根据障碍物跟车距离刷新s_refs
          sref_update[i] = s_ref_update;
        } else {
          s_ref =
              st.s_lead() + st.desired_distance() + st.v_lead() * sample_time;
          // hard bound使用安全距离
          hard_bound.upper = 150;
          hard_bound.lower =
              st.s_lead() + st.safe_distance() + st.v_lead() * sample_time;
          st_boundary.hard_bound.emplace_back(hard_bound);
          s_ref_update = std::max(
              hard_bound.lower, s_ref_update = std::max(sref_update[i], s_ref));
          // soft bound先使用期望跟车距离+buffer
          soft_bound.upper = 150;
          soft_bound.lower =
              std::max(hard_bound.lower, s_ref_update + st.v_lead() * 0.5);
          // std::max(hard_bound.lower, s_ref - st.v_lead() * 0.5);
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
  if (config_.enable_rss_model) {
    desired_distance = GetRSSDistance(lead_obstacle.v_lead(), v_ego);
  } else {
    desired_distance = GetCalibratedDistance(
        lead_obstacle.v_lead(), v_ego, lc_request,
        lead_obstacle.is_accident_car(), lead_obstacle.is_temp_lead());
  }
  return desired_distance;
}

double StGraphGenerator::GetRSSDistance(const double obstacle_velocity,
                                        double ego_velocity) {
  double follow_distance{0.0};
  double cipv_velocity = obstacle_velocity;
  double t_actuator_delay = config_.t_actuator_delay;  // actuator delay
  const double a_max_accel =
      3.0;  // maximum comfortable acceleration of the ego
  const double a_max_brake =
      -3.0;  // maximum comfortable braking deceleration of the ego
  const double CIPV_max_brake =
      -4.0;  // maximum braking deceleration of the CIPV

  follow_distance =
      ego_velocity * t_actuator_delay +
      0.5 * a_max_accel * std::pow(t_actuator_delay, 2) +
      (std::pow((ego_velocity + t_actuator_delay * a_max_accel), 2) / 2.0 /
           std::fabs(a_max_brake) -
       std::pow(cipv_velocity, 2) / 2.0 / std::fabs(CIPV_max_brake));
  follow_distance = std::max(follow_distance, 0.0);
  return follow_distance;
}

double StGraphGenerator::GetCalibratedDistance(const double v_lead,
                                               const double v_ego,
                                               const std::string &lc_request,
                                               const bool is_accident_car,
                                               const bool is_temp_lead) {
  LOG_DEBUG("-----calc_desired_distance \n");
  // 这里查表、魔数都要优化
  double t_gap = interp(v_ego, _T_GAP_VEGO_BP, _T_GAP_VEGO_V);
  if (lc_request != "none") {
    t_gap = t_gap * (0.6 + v_ego * 0.01);
  }
  if (is_temp_lead) {
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
  return std::max(0.0,
                  safe_distance_base + std::fabs(v_ego) * safe_distance_ttc);
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

  double v_target_potential_cutin = 40.0;
  double v_limit = std::min(v_cruise, v_ego);

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

      // update potential_cutin st
      common::RealTimeLonObstacleSTInfo st_info;
      st_info.set_st_type(common::RealTimeLonObstacleSTInfo::CUT_IN);
      st_info.set_id(track.track_id());
      st_info.set_a_lead(a_processed);
      st_info.set_v_lead(track.v_lead());
      st_info.set_s_lead(track.d_rel());
      st_info.set_desired_distance(desired_distance);
      st_info.set_desired_velocity(v_target_potential_cutin);
      st_info.set_safe_distance(safe_distance);
      st_info.set_start_time(time_to_entry);  // TBD:使用可配置参数
      st_info.set_end_time(5.0);              // TBD:使用可配置参数
      st_info.set_start_s(predict_distance);
      cut_in_st_info.emplace_back(st_info);

      LOG_DEBUG("potential_cutin_car's id: [%d], track.v_lat is: [%f]\n",
                track.track_id(), track.v_lat());
      LOG_DEBUG(
          "desired_distance: [%f], desired_velocity: [%f], "
          "v_target_potential_cutin: [%f]\n",
          desired_distance, desired_velocity, v_target_potential_cutin);
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
    std::vector<planning::common::RealTimeLonObstacleSTInfo> &lane_change_st_info) {
  LOG_DEBUG("----entering CalcSpeedInfoWithGap--- \n");
  double a_processed = 0.0;
  double desired_distance = 0.0;
  double safe_distance = 0.0;
  double desired_velocity = 0.0;
  double time_to_lc = 0.0;
  double predict_distance = 0.0;
  double v_limit_lc = 40.0;
  lane_change_st_info.clear();

  std::vector<planning::common::TrackedObjectInfo *> lane_changing_cars;
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
      for (auto track : lon_behav_input_->lc_info().lc_cars()) {
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
        safe_distance = CalcSafeDistance(gap.v_front, v_ego);
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
        safe_distance = CalcSafeDistance(gap.v_rear, v_ego);
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
      // gap front st
      planning::common::RealTimeLonObstacleSTInfo front_st_info;
      front_st_info.set_id(gap.front_id);
      front_st_info.set_st_type(common::RealTimeLonObstacleSTInfo::GAP);
      front_st_info.set_decision(common::RealTimeLonObstacleSTInfo::YIELD);
      front_st_info.set_a_lead(a_processed);
      front_st_info.set_v_lead(gap.v_front);
      front_st_info.set_s_lead(gap.s_front);
      front_st_info.set_desired_distance(std::min(safe_distance, gap.s_front));
      front_st_info.set_desired_velocity(v_limit_lc);
      front_st_info.set_safe_distance(std::min(safe_distance, gap.s_front));
      front_st_info.set_start_time(gap.acc_time);
      front_st_info.set_end_time(5.0);  // TBD:使用可配置参数
      front_st_info.set_start_s(gap.s_front + gap.v_front * gap.acc_time);
      lane_change_st_info.emplace_back(front_st_info);

      // gap rear st
      planning::common::RealTimeLonObstacleSTInfo rear_st_info;
      rear_st_info.set_id(gap.rear_id);
      rear_st_info.set_st_type(common::RealTimeLonObstacleSTInfo::GAP);
      rear_st_info.set_decision(common::RealTimeLonObstacleSTInfo::OVERTAKE);
      rear_st_info.set_a_lead(a_processed);
      rear_st_info.set_v_lead(gap.v_rear);
      rear_st_info.set_s_lead(gap.s_rear);
      rear_st_info.set_desired_distance(safe_distance);  // 后车这里需要再议
      rear_st_info.set_desired_velocity(v_limit_lc);
      rear_st_info.set_safe_distance(safe_distance);
      rear_st_info.set_start_time(gap.acc_time);
      rear_st_info.set_end_time(5.0);  // TBD:使用可配置参数
      rear_st_info.set_start_s(gap.s_rear + gap.v_rear * gap.acc_time);
      lane_change_st_info.emplace_back(rear_st_info);

    } else {
      // decelerate to check next interval
      auto nearest_rear_car = lane_changing_decider_->nearest_rear_car_track();
      safe_distance = CalcSafeDistance(nearest_rear_car.v_rel + v_ego, v_ego);
      lane_changing_nearest_rear_car_track_id = nearest_rear_car.id;
      v_limit_lc =
          nearest_rear_car.v_rel -
          clip((safe_distance - nearest_rear_car.d_rel) / safe_distance, 0.0,
               2.0) -
          v_ego / 10.0;
      v_limit_lc = std::max({v_ego - 3.2, v_ego + v_limit_lc,
                             6.0 + 4.0 * std::max(lc_map_decision - 2, 0)});

      planning::common::RealTimeLonObstacleSTInfo st_info;
      st_info.set_id(lane_changing_nearest_rear_car_track_id);
      st_info.set_st_type(common::RealTimeLonObstacleSTInfo::GAP);
      st_info.set_a_lead(a_processed);
      st_info.set_v_lead(nearest_rear_car.v_rel + v_ego);
      st_info.set_s_lead(nearest_rear_car.d_rel);
      st_info.set_desired_distance(safe_distance);
      st_info.set_desired_velocity(v_limit_lc);
      st_info.set_safe_distance(safe_distance);
      st_info.set_start_time(0);
      st_info.set_end_time(5.0);  // TBD:使用可配置参数
      st_info.set_start_s(nearest_rear_car.start_s);
      lane_change_st_info.emplace_back(st_info);
    }
  }
}

void StGraphGenerator::CalculateCruiseSrefs(const double v_ego,
                                            const double v_cruise,
                                            const double acc_ego,
                                            std::vector<double> &s_refs) {
  LOG_DEBUG("----entering CalculateCruiseSrefs--- \n");
  double one_a = acc_ego;
  double one_v = v_ego;
  double one_s = 0;
  s_refs.emplace_back(one_s);
  if (v_ego <= v_cruise) {
    double one_j = _J_MAX;
    for (int i = 1; i <= config_.lon_num_step; i++) {
      one_s = one_s + one_v * config_.delta_time +
              0.5 * one_a * std::pow(config_.delta_time, 2) +
              1.0 / 6 * one_j * std::pow(config_.delta_time, 3);
      one_v = std::min(one_v + one_a * config_.delta_time +
                           0.5 * one_j * std::pow(config_.delta_time, 2),
                       v_cruise);
      if (one_v == v_cruise) {
        one_a = 0.0;
      } else {
        one_a = std::min(one_a + one_j * config_.delta_time, _A_MAX);
      }
      if (one_v == v_cruise || one_a == _A_MAX) {
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
        one_a = std::max(one_a + one_j * config_.delta_time, _A_MIN);
      }
      if (one_v == v_cruise || one_a == _A_MIN) {
        one_j = 0.0;
      }
      s_refs.emplace_back(one_s);
    }
  }
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

void StGraphGenerator::SetConfig(planning::common::RealTimeLonBehaviorTunedParams &tuned_params) {
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
