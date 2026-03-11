#include "scc_lateral_obstacle_decider.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include "../../common/planning_gflags.h"
#include "debug_info_log.h"
#include "edt_manager.h"
#include "environment_model_debug_info.pb.h"
#include "environmental_model.h"
#include "math/math_utils.h"
#include "obstacle_manager.h"
#include "planning_context.h"
#include "src/modules/tasks/behavior_planners/general_lateral_decider/general_lateral_decider_utils.h"
#include "task_interface/lateral_obstacle_decider_output.h"

namespace planning {
const double kPlanningCycleTime = 1.0 / FLAGS_planning_loop_rate;
const double kSpeedThr = 100;
const double kRearVehicleTTCThreshold = 2.5;  // 交互时后方车TTC阈值 (s)
const double kNudgeDistanceToCenterLineThreshold = 0.1;  // 避让静态障碍物超越中心线的距离

SccLateralObstacleDecider::SccLateralObstacleDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : BaseLateralObstacleDecider(config_builder, session) {
  const VehicleParam &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  ego_rear_axis_to_front_edge_ = vehicle_param.front_edge_to_rear_axle;
  ego_rear_axle_to_center_ = vehicle_param.rear_axle_to_center;
  ego_length_ = vehicle_param.length;
  ego_width_ = vehicle_param.width;
  name_ = "SccLateralObstacleDecider";
  ego_rear_edge_to_rear_axle_ = vehicle_param.rear_edge_to_rear_axle;
  side_nudge_release_hysteresis_.SetThreValue(kSpeedThr + 5, kSpeedThr - 5);
  hybrid_ara_star_ = std::make_unique<SccHybridARAStar>(session);
}

bool SccLateralObstacleDecider::Execute() {
  if (!PreCheck()) {
    spatio_temporal_follow_obstacle_info_.clear();
    output_.clear();
    follow_obstacle_info_.clear();
    lateral_obstacle_history_info_.clear();
    obstacle_interaction_map_.clear();
    ILOG_DEBUG << "PreCheck failed";
    return false;
  }

  Init();

  // UpdateLaneBorrowDirection();

  UpdateIntersection();

  ConstructPlanHistoryTraj(reference_path_ptr_);
  ConstructUniformPlanHistoryTraj();

  CheckObstaclesIsReverse();

  CalLaneChangeGapInfo(lc_gap_info_);

  UpdateAvdObstacles();

  UpdateLateralObstacleDecisions();

  auto start_time = IflyTime::Now_ms();
  UpdateObstacleInteractionInfo();
  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("UpdateObstacleInteractionInfoCostTime", end_time - start_time);

  ARAStar();

  LateralObstacleDeciderOutput();

  Log();
  return true;
}

bool SccLateralObstacleDecider::Init() {
  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;

  auto &plan_history_traj = session_->mutable_planning_context()
                                ->mutable_lateral_obstacle_decider_output()
                                .plan_history_traj;
  auto &is_plan_history_traj_valid =
      session_->mutable_planning_context()
          ->mutable_lateral_obstacle_decider_output()
          .is_plan_history_traj_valid;
  auto &is_emergency_avoid_release =
      session_->mutable_planning_context()
          ->mutable_lateral_obstacle_decider_output()
          .is_emergency_avoid_release;
  auto &uniform_plan_history_traj =
      session_->mutable_planning_context()
          ->mutable_lateral_obstacle_decider_output()
          .uniform_plan_history_traj;
  auto &is_uniform_plan_history_traj_valid =
      session_->mutable_planning_context()
          ->mutable_lateral_obstacle_decider_output()
          .is_uniform_plan_history_traj_valid;
  auto &spatio_temporal_follow_obstacle_info =
      session_->mutable_planning_context()
          ->mutable_lateral_obstacle_decider_output()
          .spatio_temporal_follow_obstacle_info;
  obstacle_intrusion_distance_thr_.clear();
  obstacles_id_behind_ego_.clear();
  plan_history_traj.clear();
  is_plan_history_traj_valid = false;
  is_emergency_avoid_release.clear();
  uniform_plan_history_traj.clear();
  is_uniform_plan_history_traj_valid = false;
  spatio_temporal_follow_obstacle_info.clear();

  reference_path_ptr_ = coarse_planning_info.reference_path;

  ego_head_s_ = reference_path_ptr_->get_frenet_ego_state().head_s();
  ego_head_l_ = reference_path_ptr_->get_frenet_ego_state().head_l();
  ego_v_ = reference_path_ptr_->get_frenet_ego_state().velocity();
  ego_v_s_ = reference_path_ptr_->get_frenet_ego_state().velocity_s();
  ego_v_l_ = reference_path_ptr_->get_frenet_ego_state().velocity_l();
  side_nudge_release_hysteresis_.SetIsValidByValue(ego_v_ * 3.6);
  lane_width_ =
      session_->environmental_model()
          .get_virtual_lane_manager()
          ->get_lane_with_virtual_id(coarse_planning_info.target_lane_id)
          ->width();
  lc_gap_info_.Reset();
  ClearHistoryInfo();
  return true;
}

bool SccLateralObstacleDecider::CheckIsRightestLane() {
  const std::shared_ptr<VirtualLaneManager> virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  bool rightest_lane;
  if (((virtual_lane_manager->current_lane_virtual_id() ==
        virtual_lane_manager->get_lane_num() - 1) ||
       (virtual_lane_manager->current_lane_virtual_id() ==
            virtual_lane_manager->get_lane_num() - 2 &&
        virtual_lane_manager->get_right_lane() != nullptr &&
        virtual_lane_manager->get_right_lane()->get_lane_type() ==
            iflyauto::LANETYPE_NON_MOTOR)) &&
      virtual_lane_manager->current_lane_virtual_id() - 1 >= 0) {
    rightest_lane = true;
  } else {
    rightest_lane = false;
  }
  return rightest_lane;
}

void SccLateralObstacleDecider::UpdateAvdObstacles() {
  const auto target_state = session_->planning_context()
                                .lane_change_decider_output()
                                .coarse_planning_info.target_state;
  bool is_in_lane_change_execution_scene = target_state == kLaneChangeExecution;

  bool rightest_lane = CheckIsRightestLane();

  bool is_change_lanes = IsChangeLanes();
  if (is_change_lanes) {
    // fix_lane切换， 清空历史信息
    ResetObstaclesHistory(is_change_lanes);
    return;
  } else {
    ResetObstaclesHistory(is_change_lanes);
  }

  // farthest_distance
  // auto &last_traj_points =
  //     session_->planning_context().last_planning_result().traj_points;
  // double farthest_distance = DBL_MAX;
  // if (!last_traj_points.empty() && last_traj_points.back().frenet_valid) {
  //   farthest_distance = last_traj_points.back().s - last_traj_points.front().s;
  // }
  double farthest_distance = DBL_MAX;
  const auto& motion_planner_output =
      session_->planning_context().motion_planner_output();
  auto& frenet_coord = reference_path_ptr_->get_frenet_coord();
  Point2D frenet_pt{0.0, 0.0};
  if (motion_planner_output.is_valid_planning_end_xy_point &&
      frenet_coord->XYToSL(motion_planner_output.planning_end_xy_point,
                           frenet_pt)) {
    farthest_distance = std::max(
        0.0, frenet_pt.x - reference_path_ptr_->get_frenet_ego_state().s());
  }

  double expand_vel =
      interp(ego_v_, config_.expand_ego_vel, config_.expand_obs_rel_vel);

  for (auto frenet_obs : reference_path_ptr_->get_obstacles()) {
    UpdateAvdObstacle(*frenet_obs, expand_vel, farthest_distance, rightest_lane,
                      is_in_lane_change_execution_scene);
  }
}

bool SccLateralObstacleDecider::IsChangeLanes() {
  auto last_fix_lane_id = session_->environmental_model()
                              .get_virtual_lane_manager()
                              ->get_last_fix_lane_id();
  auto current_fix_lane_id = session_->planning_context()
                                 .lane_change_decider_output()
                                 .fix_lane_virtual_id;
  return last_fix_lane_id != current_fix_lane_id;
}

void SccLateralObstacleDecider::UpdateAvdObstacle(
    const FrenetObstacle &frenet_obs, double expand_vel,
    double farthest_distance, bool rightest_lane,
    bool is_in_lane_change_execution_scene) {
  LateralObstacleHistoryInfo &history =
      lateral_obstacle_history_info_[frenet_obs.id()];
  FollowObstacleInfo &follow_info = follow_obstacle_info_[frenet_obs.id()];

  CalculateCutInAndCross(frenet_obs);
  // 判断车辆相对位置，前车，后车，旁车（从前方来的，从后方来的，不知道从哪来的）
  if (history.side_car &&
      frenet_obs.frenet_relative_velocity_s() < expand_vel) {
    history.front_expand_len = 1.5;
  }
  if (in_intersection_ && history.side_car &&
      frenet_obs.frenet_relative_velocity_s() > -2) {
    history.rear_expand_len = 1.5;
  }

  GetPositionRelation(frenet_obs, history);
  int side_2_front_count_thr = config_.side_2_front_count_thr;
  GetSideCarNudge(frenet_obs, history, side_2_front_count_thr);

  if (frenet_obs.d_s_rel() <= 0 &&
      (history.side_2_front_count <= side_2_front_count_thr ||
       !config_.open_side_lat_offset_nudge)) {
    history.is_avd_car = false;
    if (frenet_obs.d_s_rel() <= -1 * (frenet_obs.length() + ego_length_)) {
      history.ncar_count = 0;
      history.ncar_count_in = false;
    }
    follow_info.is_need_folow = false;
    follow_info.follow_confidence = 0;
    return;
  }

  history.is_avd_car = IsPotentialAvoidingCar(
      frenet_obs, rightest_lane, farthest_distance, left_borrow_, right_borrow_,
      is_in_lane_change_execution_scene);

  IsPotentialFollowingObstacle(frenet_obs, is_in_lane_change_execution_scene);

  history.last_recv_time = frenet_obs.obstacle()->timestamp();
}

void SccLateralObstacleDecider::GetPositionRelation(
    const FrenetObstacle &frenet_obs, LateralObstacleHistoryInfo &history) {
  if (frenet_obs.d_s_rel() > history.front_expand_len) {
    history.front_car = true;
    history.side_car = false;
    history.rear_car = false;
  } else if (frenet_obs.d_s_rel() >= -(ego_length_ + history.rear_expand_len) &&
             frenet_obs.d_s_rel() <= history.front_expand_len) {
    if (CheckSideObstacle(frenet_obs)) {
      history.side_car = true;
    } else {
      history.side_car = false;
    }
  } else {
    history.front_car = false;
    history.side_car = false;
    history.rear_car = true;
  }
}

void SccLateralObstacleDecider::GetSideCarNudge(
    const FrenetObstacle& frenet_obs, LateralObstacleHistoryInfo& history,
    int& side_2_front_count_thr) {
  const int kCrossLaneMaxCount = 10;
  const int kCrossLaneCountThr = 3;
  const int kSide2FrontMaxCount = config_.side_2_front_max_count;
  const double half_width = lane_width_ * 0.5;
  double extra_cross_lane_buffer = 0;
  bool is_side_nudge_release = side_nudge_release_hysteresis_.IsValid();
  if (history.is_cross_lane) {
    extra_cross_lane_buffer = 0.2;
  }
  history.is_cross_lane =
      (frenet_obs.d_min_cpath() <= half_width + extra_cross_lane_buffer &&
       frenet_obs.d_max_cpath() >= half_width + extra_cross_lane_buffer) ||
      (frenet_obs.d_min_cpath() <= -half_width - extra_cross_lane_buffer &&
       frenet_obs.d_max_cpath() >= -half_width - extra_cross_lane_buffer);
  if (history.is_cross_lane) {
    history.cross_lane_count =
        std::min(history.cross_lane_count + 1, kCrossLaneMaxCount);
  } else {
    history.cross_lane_count = std::max(history.cross_lane_count - 1, 0);
  }
  if (history.cross_lane_count > kCrossLaneCountThr || !is_side_nudge_release) {
    // 连续三帧跨线或者速度较低
    side_2_front_count_thr = config_.cross_lane_side_2_front_count_thr;
  }
  if (history.side_car) {
    // 针对侧方->前方位置的转化，为了避免障碍物长时间在自车侧方导致不合理限制避让幅度从而引入记时操作
    if (std::fabs(frenet_obs.d_s_rel()) <= history.overlap_ego_head_thr) {
      history.side_2_front_count =
          std::min(history.side_2_front_count + 1, kSide2FrontMaxCount);
    } else {
      history.side_2_front_count = std::max(history.side_2_front_count - 1, 0);
    }
    if (history.side_2_front_count > side_2_front_count_thr) {
      // history.front_car = true;
      // history.rear_car = false;
      history.is_potential_avoiding_side_car = true;
      history.overlap_ego_head_thr = 2.5;
    } else {
      history.overlap_ego_head_thr = 2;
    }
  } else {
    history.side_2_front_count = 0;
    history.overlap_ego_head_thr = 2;
  }
}

void SccLateralObstacleDecider::ResetObstaclesHistory(bool is_change_lanes) {
  for (auto frenet_obs : reference_path_ptr_->get_obstacles()) {
    const Obstacle *obs = frenet_obs->obstacle();
    LateralObstacleHistoryInfo &history =
        lateral_obstacle_history_info_[obs->id()];
    FollowObstacleInfo &follow_info = follow_obstacle_info_[obs->id()];

    history.front_expand_len = 0.0;
    history.rear_expand_len = 0.0;
    history.is_potential_avoiding_side_car = false;
    history.is_behind_ego = false;
    if (history.can_not_avoid) {
      history.is_not_set = false;
    } else {
      history.is_not_set = true;
    }
    // ignore obj without camera source
    if (!(obs->fusion_source() & OBSTACLE_SOURCE_CAMERA) ||
        !frenet_obs->b_frenet_valid() || is_change_lanes) {
      history.is_avd_car = false;
      history.ncar_count = 0;
      history.ncar_count_in = false;
      history.is_cross_lane = false;
      history.cross_lane_count = 0;
      follow_info.is_need_folow = false;
      follow_info.follow_confidence = 0;
    }
  }
}

void SccLateralObstacleDecider::UpdateLateralObstacleDecisions() {
  last_output_ = output_;
  output_.clear();
  spatio_temporal_follow_obstacle_info_.clear();

  const auto target_state = session_->planning_context()
                                .lane_change_decider_output()
                                .coarse_planning_info.target_state;
  bool is_in_lane_change_execution_scene = target_state == kLaneChangeExecution;

  for (auto frenet_obs : reference_path_ptr_->get_obstacles()) {
    const Obstacle *obs = frenet_obs->obstacle();
    LateralObstacleHistoryInfo &history =
        lateral_obstacle_history_info_[obs->id()];
    FollowObstacleInfo &follow_info = follow_obstacle_info_[obs->id()];
    // ignore obj without camera source
    if (!(obs->fusion_source() & OBSTACLE_SOURCE_CAMERA) ||
        !frenet_obs->b_frenet_valid()) {
      follow_info.is_need_folow = false;
      follow_info.follow_confidence = 0;
      output_[obs->id()] = LatObstacleDecisionType::NOT_SET;
      continue;
    }
    LateralObstacleDecision(*frenet_obs, is_in_lane_change_execution_scene);
    if (history.last_is_avd_car && !history.is_avd_car) {
      HoldLatOffset(*frenet_obs);
    }
    history.last_is_avd_car = history.is_avd_car;
    // CheckLateralEmergencyAvoidObstacle(*frenet_obs);
  }
}

void SccLateralObstacleDecider::CheckLateralEmergencyAvoidObstacle(
    const FrenetObstacle &frenet_obstacle) {
  auto &is_emergency_avoid_release =
      session_->mutable_planning_context()
          ->mutable_lateral_obstacle_decider_output()
          .is_emergency_avoid_release;
  const auto &is_crossing_map = session_->planning_context()
                                    .crossing_agent_decider_output()
                                    .is_crossing_map;
  const auto lon_ref_path_decider_output =
      session_->planning_context().lon_ref_path_decider_output();
  auto &frenet_coord = reference_path_ptr_->get_frenet_coord();
  const Obstacle &obstacle = *frenet_obstacle.obstacle();
  LateralObstacleHistoryInfo &history =
      lateral_obstacle_history_info_[obstacle.id()];
  double s = frenet_obstacle.frenet_s();
  double l = frenet_obstacle.frenet_l();
  double v_s = frenet_obstacle.frenet_velocity_s();
  double v_l = frenet_obstacle.frenet_velocity_l();
  double v_lat = frenet_obstacle.frenet_velocity_lateral();
  double v_s_rel = frenet_obstacle.frenet_relative_velocity_s();
  double d_s_rel = frenet_obstacle.d_s_rel();
  double d_min_cpath = frenet_obstacle.d_min_cpath();
  double d_max_cpath = frenet_obstacle.d_max_cpath();
  double lon_ttc = 0;
  int max_emergency_avoid_count = 6;
  int emergency_avoid_count_thr = config_.emergency_avoid_count_thr;
  double half_lane_width = lane_width_ * 0.5;

  if (config_.is_use_last_lon_information) {
    // 上一帧纵向释放的紧急障碍物id
    const auto last_lon_emergency_avoid_obstacle_id =
        lon_ref_path_decider_output.danger_agent_info.agents_id_set;
    if (last_lon_emergency_avoid_obstacle_id.count(obstacle.id())) {
      if (!history.emergency_avoid) {
        // 前方障碍物
        if (history.front_car || history.side_car) {
          // ttc条件
          // 因为这是纵向制动减速的障碍物，按照匀速递推的ttc会更加考虑该障碍物
          lon_ttc = v_s_rel < 0 ? (d_s_rel / (-v_s_rel))
                                : std::numeric_limits<double>::max();
          bool is_lon_ttc_critical =
              (lon_ttc < config_.emegency_avoid_ttc_lower) ||
              (lon_ttc < config_.emegency_avoid_ttc_upper &&
               d_s_rel < config_.emegency_avoid_front_area);
          // 横向距离条件
          bool is_in_lateral_range =
              ((d_min_cpath > 0) &&
               (d_min_cpath <
                config_.emegency_avoid_lareral_area + half_lane_width)) ||
              ((d_max_cpath < 0) &&
               (d_max_cpath >
                -config_.emegency_avoid_lareral_area - half_lane_width));
          // 是否为正前方障碍物（如果纵向跟停的时候不会释放，可以先注释）
          bool is_straight_ahead = d_max_cpath > 0 && d_min_cpath < 0;
          // 过滤横穿的障碍物（VRU预测轨迹不准需要进一步优化）
          const auto cossing_map_iter = is_crossing_map.find(obstacle.id());
          bool is_cross_lane = (cossing_map_iter != is_crossing_map.end() &&
                                cossing_map_iter->second);
          bool is_emergency_avoid = is_lon_ttc_critical &&
                                    is_in_lateral_range && !is_straight_ahead &&
                                    !is_cross_lane;
          if (is_emergency_avoid) {
            history.emergency_avoid_count = std::min(
                history.emergency_avoid_count + 1, max_emergency_avoid_count);
          } else {
            history.emergency_avoid_count =
                std::max(history.emergency_avoid_count - 1, 0);
          }
          history.emergency_avoid =
              history.emergency_avoid_count > emergency_avoid_count_thr;
          is_emergency_avoid_release[obstacle.id()] =
              history.emergency_avoid ? true : false;
        } else {
          // 后方障碍物，纵向因为后方障碍物导致的急刹需要与纵向侧后方障碍物过滤保持一致先不添加
        }
      } else {
        // 如果已经触发了紧急避让，判定何时取消紧急避让
        // 超越紧急避让障碍物
        bool is_overtake_emergency_obstacle = history.rear_car;
        // 自车是否刹停
        bool is_near_static = std::fabs(ego_v_) < 0.1;
        // 自车速度是否低于障碍物速度：
        bool is_slow_obstacle = ego_v_ < v_s;
        history.emergency_avoid = !(is_overtake_emergency_obstacle ||
                                    is_near_static || is_slow_obstacle);
        is_emergency_avoid_release[obstacle.id()] =
            history.emergency_avoid ? true : false;
      }
    }

    // 纵向overtake、忽略的障碍物id
    // const auto last_lon_overtake_obstacle_id =
    // mutable_lon_ref_path_decider_output.danger_agent_info.agents_id_set;
    std::unordered_set<int32_t> last_lon_overtake_obstacle_id;
    if (last_lon_overtake_obstacle_id.count(obstacle.id())) {
      if (!history.lon_overtake_avoid) {
        // 前方障碍物
        if (history.front_car || history.side_car) {
          // ttc条件
          lon_ttc = v_s_rel < 0 ? (d_s_rel / (-v_s_rel))
                                : std::numeric_limits<double>::max();
          bool is_lon_ttc_critical =
              (lon_ttc < config_.emegency_avoid_ttc_lower) ||
              (lon_ttc < config_.emegency_avoid_ttc_upper &&
               d_s_rel < config_.emegency_avoid_front_area);
          // 横向距离条件
          bool is_in_lateral_range =
              ((d_min_cpath > 0) &&
               (d_min_cpath <
                config_.emegency_avoid_lareral_area + half_lane_width)) ||
              ((d_max_cpath < 0) &&
               (d_max_cpath >
                -config_.emegency_avoid_lareral_area - half_lane_width));
          // 横向是否ignore
          bool is_lateral_ignore =
              (output_[obstacle.id()] == LatObstacleDecisionType::IGNORE ||
               output_[obstacle.id()] == LatObstacleDecisionType::FOLLOW);
          bool lon_overtake_avoid =
              is_lon_ttc_critical && is_in_lateral_range && is_lateral_ignore;
          history.lon_overtake_avoid = lon_overtake_avoid;
        } else {
          // 后方障碍物，纵向正常overtake,横向先不管
        }
      } else {
        // 如果已经触发了overtake
        bool is_overtake_emergency_obstacle = history.rear_car;
        history.lon_overtake_avoid = !is_overtake_emergency_obstacle;
      }
    }
  } else {
    if (!history.emergency_avoid) {
      if (history.front_car && !history.side_car && !history.rear_car) {
        // 根据纵向ttc和制动力筛选避让障碍物
        lon_ttc = v_s_rel < 0 ? (d_s_rel / (-v_s_rel))
                              : std::numeric_limits<double>::max();
        bool is_lon_ttc_critical =
            (lon_ttc < config_.emegency_avoid_ttc_lower) ||
            (lon_ttc < config_.emegency_avoid_ttc_upper &&
             d_s_rel < config_.emegency_avoid_front_area);
        // 横向距离条件
        bool is_in_lateral_range =
            ((d_min_cpath > 0) &&
             (d_min_cpath <
              config_.emegency_avoid_lareral_area + half_lane_width)) ||
            ((d_max_cpath < 0) &&
             (d_max_cpath >
              -config_.emegency_avoid_lareral_area - half_lane_width));
        const auto cossing_map_iter = is_crossing_map.find(obstacle.id());
        bool is_cross_lane = (cossing_map_iter != is_crossing_map.end() &&
                              cossing_map_iter->second);
        // bool is_cross_lane = false;
        // 横向是否ignore
        bool is_lateral_ignore =
            (output_[obstacle.id()] == LatObstacleDecisionType::IGNORE ||
             output_[obstacle.id()] == LatObstacleDecisionType::FOLLOW);
        // 纵向以一定的制动力减速还来不及的障碍物
        bool is_lon_over_obstacle = CheckEgoOvertakeObstacle(frenet_obstacle);
        bool is_emergency_avoid = is_lon_ttc_critical && is_in_lateral_range &&
                                  is_lateral_ignore && is_lon_over_obstacle &&
                                  !is_cross_lane;
        if (is_emergency_avoid) {
          history.emergency_avoid_count = std::min(
              history.emergency_avoid_count + 1, max_emergency_avoid_count);
        } else {
          history.emergency_avoid_count =
              std::max(history.emergency_avoid_count - 1, 0);
        }
        history.emergency_avoid =
            history.emergency_avoid_count > emergency_avoid_count_thr;
      } else {
        // 过滤侧、后方障碍物
      }
    } else {
      // 如果已经触发了紧急避让，判定何时取消紧急避让
      // 超越紧急避让障碍物
      bool is_overtake_emergency_obstacle = history.rear_car;
      // 自车是否刹停
      bool is_near_static = std::fabs(ego_v_) < 0.1;
      // 自车速度是否低于障碍物速度
      bool is_slow_obstacle = ego_v_ < v_s;
      history.emergency_avoid = !(is_overtake_emergency_obstacle ||
                                  is_near_static || is_slow_obstacle);
    }
  }
}

void SccLateralObstacleDecider::HoldLatOffset(
    const FrenetObstacle &frenet_obstacle) {
  const Obstacle &obstacle = *frenet_obstacle.obstacle();
  LateralObstacleHistoryInfo &history =
      lateral_obstacle_history_info_[obstacle.id()];
  const auto lat_obs_decision_iter = output_.find(obstacle.id());
  if (lat_obs_decision_iter == output_.end()) {
    return;
  }
  double v_s_rel = frenet_obstacle.frenet_relative_velocity_s();
  double d_s_rel = frenet_obstacle.d_s_rel();
  bool maintain_avoid =
      ((output_[obstacle.id()] == LatObstacleDecisionType::IGNORE ||
        output_[obstacle.id()] == LatObstacleDecisionType::FOLLOW) &&
       !history.rear_car && v_s_rel < 0 &&
       ((d_s_rel / (-v_s_rel) < config_.emegency_cutin_ttc_lower) ||
        ((d_s_rel / (-v_s_rel) < config_.emegency_cutin_ttc_upper &&
          d_s_rel < config_.emegency_cutin_front_area))));
  if (maintain_avoid) {
    history.is_avd_car = true;
    history.maintain_avoid = true;
  } else {
    history.maintain_avoid = false;
  }
}

bool SccLateralObstacleDecider::CheckEgoOvertakeObstacle(
    const FrenetObstacle &frenet_obstacle) {
  const int obstacle_id = frenet_obstacle.id();
  const Obstacle &obstacle = *frenet_obstacle.obstacle();
  // 纵向以一定的制动力减速还来不及的障碍物
  const double desired_stopped_distance_to_obstacle = 3.0;
  const double break_a = 2.5;
  double ego_end_s = 0;
  double obstacke_start_s = 0.0;
  bool is_lon_over_obstacle = false;
  std::array<double, 6> timestamps{0, 1, 2, 3, 4, 5};
  bool ok = false;
  for (auto &i : timestamps) {
    Polygon2d obstacle_sl_polygon;
    ok = reference_path_ptr_->get_polygon_at_time(
        obstacle_id, false, int(i * 10), obstacle_sl_polygon);
    if (ok) {
      obstacke_start_s = obstacle_sl_polygon.min_x();
    }
    if ((std::fabs(ego_v_) / break_a) < i) {
      ego_end_s = 0.5 * ego_v_ * ego_v_ / break_a;
    } else {
      ego_end_s = std::fabs(ego_v_) * i - 0.5 * break_a * i * i;
    }
    is_lon_over_obstacle =
        (ego_head_s_ + ego_end_s) >
        obstacke_start_s - desired_stopped_distance_to_obstacle;
    if (is_lon_over_obstacle) {
      break;
    }
  }
  return is_lon_over_obstacle;
}

bool SccLateralObstacleDecider::IsPotentialAvoidingCar(
    const FrenetObstacle &frenet_obstacle, bool rightest_lane,
    double farthest_distance, bool can_left_borrow, bool can_right_borrow,
    bool is_in_lane_change_execution_scene) {
  ILOG_DEBUG << "----is_potential_avoiding_car-----";
  LateralObstacleHistoryInfo &history =
      lateral_obstacle_history_info_[frenet_obstacle.id()];

  const auto &lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto &target_state = session_->planning_context()
                                 .lane_change_decider_output()
                                 .coarse_planning_info.target_state;
  bool is_lane_change =
      (target_state == kLaneChangeExecution ||
       target_state == kLaneChangeHold || target_state == kLaneChangeCancel);

  // // static obs lat_safety_buffer
  // if ((l > 0 && can_right_borrow) || (l < 0 && can_left_borrow)) {
  //   static_obs_buffer = config_.large_static_obs_buffer;
  // }
  double lat_safety_buffer;
  if (!frenet_obstacle.obstacle()->is_static()) {
    lat_safety_buffer = CalDynamicLatBuffer(frenet_obstacle);
  } else {
    lat_safety_buffer = config_.large_static_obs_buffer;
  }

  obstacle_intrusion_distance_thr_[frenet_obstacle.id()] =
      std::max(lane_width_ - ego_width_ - lat_safety_buffer, 0.0);

  bool is_in_range = IsInRange(frenet_obstacle);
  bool is_about_to_enter_range = IsAboutToEnterRange(frenet_obstacle);

  bool is_avoidable = false;
  if (is_in_range || is_about_to_enter_range) {
    history.is_not_set = false;
    is_avoidable =
        IsAvoidable(frenet_obstacle, lat_safety_buffer, is_in_lane_change_execution_scene);
    history.can_avoid = HasEnoughNudgeSpace(frenet_obstacle, lat_safety_buffer,
                                            is_in_lane_change_execution_scene, true);
  } else {
    history.can_avoid = false;
    history.can_avoid_count = 0;
  }

  // for Intersection
  if (frenet_obstacle.d_s_rel() > farthest_distance + ego_length_ ||
      (frenet_obstacle.d_s_rel() > farthest_distance - ego_length_ &&
       ((frenet_obstacle.d_max_cpath() < 0 &&
         std::fabs(frenet_obstacle.d_max_cpath()) > lane_width_ * 0.5) ||
        (frenet_obstacle.d_min_cpath() > 0 &&
         frenet_obstacle.d_min_cpath() > lane_width_ * 0.5)))) {
    return false;
  }

  double ncar_count = GetAvoidCountThre(frenet_obstacle);

  if (UpdateObstacleAvoidCount(frenet_obstacle, is_avoidable, is_in_range,
                               is_about_to_enter_range, ncar_count,
                               rightest_lane, is_lane_change)) {
    return true;
  }

  return false;
}

bool SccLateralObstacleDecider::IsInRange(
    const FrenetObstacle &frenet_obstacle) {
  const LateralObstacleHistoryInfo &history =
      lateral_obstacle_history_info_[frenet_obstacle.id()];
  // hysteresis
  double in_range_v = config_.in_range_v;
  if (history.is_avd_car) {
    in_range_v = in_range_v * config_.in_range_v_hysteresis;
  }

  return frenet_obstacle.d_s_rel() < 20.0 &&
         frenet_obstacle.frenet_relative_velocity_s() < in_range_v;
}

bool SccLateralObstacleDecider::IsAboutToEnterRange(
    const FrenetObstacle &frenet_obstacle) {
  const auto state =
      session_->planning_context().lane_change_decider_output().curr_state;
  const auto lc_request_direction =
      session_->planning_context().lane_change_decider_output().lc_request;
  bool is_left_lc =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == LEFT_CHANGE);
  bool is_right_lc =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == RIGHT_CHANGE);

  double ttc_for_obs = config_.start_nudge_ttc;
  // hysteresis
  const auto lat_offset =
      session_->planning_context().lateral_behavior_planner_output().lat_offset;
  if (((lat_offset > 0 || is_right_lc) && frenet_obstacle.d_max_cpath() < 0) ||
      ((lat_offset < 0 || is_left_lc) && frenet_obstacle.d_min_cpath() > 0)) {
    if (frenet_obstacle.obstacle()->is_oversize_vehicle()) {
      ttc_for_obs = 8.0;
    } else {
      ttc_for_obs = 6.0;
    }
  }
  double max_enter_range =
      std::fabs(frenet_obstacle.frenet_relative_velocity_s()) * ttc_for_obs;
  max_enter_range = max_enter_range > 100 ? 100 : max_enter_range;
  max_enter_range = max_enter_range < 50 ? 50 : max_enter_range;

  return (frenet_obstacle.d_s_rel() <
              std::min(std::fabs(10.0 *
                                 frenet_obstacle.frenet_relative_velocity_s()),
                       max_enter_range) &&
          frenet_obstacle.frenet_relative_velocity_s() < -2.5);
}

bool SccLateralObstacleDecider::IsAvoidable(
    const FrenetObstacle &frenet_obstacle, double lat_safety_buffer,
    bool is_in_lane_change_execution_scene) {
  LateralObstacleHistoryInfo &history =
      lateral_obstacle_history_info_[frenet_obstacle.id()];

  double potential_near_car_v_ub = config_.potential_near_car_v_ub;
  double potential_near_car_v_lb = config_.potential_near_car_v_lb;
  bool is_same_side =
      (frenet_obstacle.d_min_cpath() > 0 && frenet_obstacle.d_max_cpath() > 0 ||
       frenet_obstacle.d_min_cpath() <= 0 &&
           frenet_obstacle.d_max_cpath() <= 0);

  const double v_lat = frenet_obstacle.frenet_velocity_lateral();

  double distance_to_lane_line_thre = CalDistanceThre2LaneLine(frenet_obstacle);
  double distance_to_center_line_thre =
      lane_width_ * 0.5 + distance_to_lane_line_thre;

  double potential_dist_limit =
      lane_width_ * 0.5 + config_.potential_near_car_thr;
  // hack 感知大车减去后视镜宽度
  if (IsTruck(frenet_obstacle)) {
    distance_to_center_line_thre += config_.extra_truck_lat_buffer;
    potential_dist_limit += config_.extra_truck_lat_buffer;
  } else {
    potential_near_car_v_lb = -0.1;
  }
  // need avoid flag
  bool is_need_avoid =
      (frenet_obstacle.d_max_cpath() < 0 && std::fabs(frenet_obstacle.d_max_cpath()) < distance_to_center_line_thre) ||
      (frenet_obstacle.d_min_cpath() > 0 && frenet_obstacle.d_min_cpath() < distance_to_center_line_thre) ||
      (frenet_obstacle.d_max_cpath() < 0 && std::fabs(frenet_obstacle.d_max_cpath()) < potential_dist_limit &&
        v_lat < potential_near_car_v_lb &&
        v_lat > potential_near_car_v_ub) ||
      (frenet_obstacle.d_min_cpath() > 0 && frenet_obstacle.d_min_cpath() < potential_dist_limit &&
        v_lat < potential_near_car_v_lb &&
        v_lat > potential_near_car_v_ub);

  // can avoid flag
  bool can_avoid =
      HasEnoughNudgeSpace(frenet_obstacle, lat_safety_buffer, is_in_lane_change_execution_scene);

  // if (is_lane_change &&
  //         frenet_obstacle.frenet_obstacle_boundary().s_start <
  //             lc_gap_info_.gap_front_s) {
  //   // 在变道状态，依据gap计算
  //   history.cut_in_or_cross = false;
  //   history.cut_in_or_cross_count = 0;
  // }

  if (is_need_avoid && !can_avoid) {
    history.can_not_avoid = true;
  }

  return is_same_side && is_need_avoid && can_avoid;
}

bool SccLateralObstacleDecider::HasEnoughNudgeSpace(
    const FrenetObstacle &frenet_obstacle, double lat_safety_buffer,
    bool is_in_lane_change_execution_scene, bool is_filter) {
  // 命名可进一步区分
  constexpr int kMaxCanAvoidCount = 6;
  constexpr int kEnoughNudgeSpaceCountThr = 2;

  LateralObstacleHistoryInfo &history =
      lateral_obstacle_history_info_[frenet_obstacle.id()];

  const double v_lat = frenet_obstacle.frenet_velocity_lateral();

  // 可以考虑使用预测
  double d_max_cpath_prediction = frenet_obstacle.d_max_cpath();
  double d_min_cpath_prediction = frenet_obstacle.d_min_cpath();
  std::array<double, 3> x_v_lat{-0.6, -0.4, -0.2};
  std::array<double, 3> f_times{10, 5, 1};
  double times = interp(v_lat, x_v_lat, f_times);
  if (frenet_obstacle.d_max_cpath() < 0) {
    d_max_cpath_prediction =
        frenet_obstacle.d_max_cpath() - v_lat * 0.1 * times;
  } else if (frenet_obstacle.d_min_cpath() > 0) {
    d_min_cpath_prediction =
        frenet_obstacle.d_min_cpath() + v_lat * 0.1 * times;
  }

  if (is_filter) {
    // 为了保证稳定性
    if (history.can_avoid) {
      lat_safety_buffer = lat_safety_buffer - 0.1;
      ;
    }
  }

  // can avoid flag
  bool has_enough_nudge_space =
      (d_min_cpath_prediction >
       std::min((ego_width_ + lat_safety_buffer) - lane_width_ / 2, 1.8)) ||
      (d_max_cpath_prediction <
       std::max(lane_width_ / 2 - (ego_width_ + lat_safety_buffer), -1.8));

  if (is_in_lane_change_execution_scene && frenet_obstacle.frenet_obstacle_boundary().s_start <
                            lc_gap_info_.gap_front_s) {
    has_enough_nudge_space = true;
  }

  if (is_filter) {
    if (has_enough_nudge_space) {
      double intrusion_distance = DBL_MAX;
      if (frenet_obstacle.d_max_cpath() < 0) {
        intrusion_distance =
            lane_width_ * 0.5 - std::fabs(frenet_obstacle.d_max_cpath());
      } else if (frenet_obstacle.d_min_cpath() > 0) {
        intrusion_distance = lane_width_ * 0.5 - frenet_obstacle.d_min_cpath();
      }
      std::array<double, 3> intrusion_distance_xp{0.1, 0.2, 0.3};
      std::array<double, 3> can_avoid_count_fp{3.1, 2.1, 1.1};
      int can_avoid_count = static_cast<int>(interp(
          intrusion_distance, intrusion_distance_xp, can_avoid_count_fp));
      history.can_avoid_count = std::min(
          history.can_avoid_count + can_avoid_count, kMaxCanAvoidCount);
    } else {
      // 如果当前帧不能避让，则不避让
      history.can_avoid_count = 0;
    }

    has_enough_nudge_space =
        history.can_avoid_count > kEnoughNudgeSpaceCountThr;
  }

  return has_enough_nudge_space;
}

double SccLateralObstacleDecider::CalDynamicLatBuffer(
    const FrenetObstacle &frenet_obstacle) {
  double lat_safety_buffer = config_.lat_safety_buffer;

  const auto obstacle = frenet_obstacle.obstacle();
  if (obstacle == nullptr) {
    return lat_safety_buffer;
  }

  if (obstacle->is_oversize_vehicle()) {
    lat_safety_buffer += config_.oversize_veh_addition_buffer;
  }

  // 减去大车后视镜
  if (obstacle->is_oversize_vehicle()) {
    lat_safety_buffer -= 0.2;
  }

  // hack 感知大车减去后视镜宽度
  if (IsTruck(frenet_obstacle)) {
    lat_safety_buffer += config_.extra_truck_lat_buffer;
  }

  // 对向车减小0.2m
  if (obstacle->is_reverse()) {
    lat_safety_buffer -= 0.2;
  }

  // addition buffer for VRU
  if (obstacle->is_VRU()) {
    lat_safety_buffer += 0.2;
  }

  // decre lat_safety_buffer
  std::array<double, 3> x_lat_buffer{3.2, 3.5, 3.8};
  std::array<double, 3> f_lat_buffer{0.3, 0.15, 0};
  double decre_buffer_for_lane_width =
      interp(lane_width_, x_lat_buffer, f_lat_buffer);
  lat_safety_buffer -= decre_buffer_for_lane_width;

  return lat_safety_buffer;
}

double SccLateralObstacleDecider::CalDistanceThre2LaneLine(
    const FrenetObstacle &frenet_obstacle) {
  const Obstacle &obstacle = *frenet_obstacle.obstacle();
  iflyauto::ObjectType type = obstacle.type();
  LateralObstacleHistoryInfo &history =
      lateral_obstacle_history_info_[obstacle.id()];

  if (obstacle.is_traffic_facilities()) {
    return -config_.traffic_cone_thr;  // 负值，代表车道线内
  }

  double min_near_car_thr = 0.09;
  std::array<double, 3> xp{20, 40, 60};
  std::array<double, 3> fp{config_.near_car_thr, 0.12, min_near_car_thr};
  double distance_to_lane_line_thre = interp(frenet_obstacle.d_s_rel(), xp, fp);

  // lower buffer for car
  if (!obstacle.is_static() &&
      (type == iflyauto::ObjectType::OBJECT_TYPE_COUPE ||
       type == iflyauto::ObjectType::OBJECT_TYPE_MINIBUS ||
       type == iflyauto::ObjectType::OBJECT_TYPE_VAN) &&
      !IsTruck(frenet_obstacle)) {
    // distance_to_lane_line_thre = distance_to_lane_line_thre *
    // car_addition_decre_factor;
    distance_to_lane_line_thre =
        distance_to_lane_line_thre - config_.car_addition_decre_buffer;
    distance_to_lane_line_thre =
        std::fmax(distance_to_lane_line_thre, min_near_car_thr);
  }

  // addition buffer for oversize vehicle
  if (obstacle.is_oversize_vehicle()) {
    std::array<double, 2> vel_xp_oversize_veh{2.7, 5.6};
    std::array<double, 2> vel_fp_oversize_veh{1, 2.5};
    double vel_factor_for_oversize_veh =
        interp(ego_v_, vel_xp_oversize_veh, vel_fp_oversize_veh);
    std::array<double, 3> dis_fp_oversize_veh{1, 1.5, 2};
    double dis_factor_for_oversize_veh =
        interp(frenet_obstacle.d_s_rel(), xp, dis_fp_oversize_veh);
    distance_to_lane_line_thre = distance_to_lane_line_thre *
                                 vel_factor_for_oversize_veh *
                                 dis_factor_for_oversize_veh;
  }

  // hysteresis
  if (history.is_avd_car) {
    distance_to_lane_line_thre =
        distance_to_lane_line_thre * config_.near_car_hysteresis;
  }

  const auto lat_offset =
      session_->planning_context().lateral_behavior_planner_output().lat_offset;
  const auto state =
      session_->planning_context().lane_change_decider_output().curr_state;
  const auto lc_request_direction =
      session_->planning_context().lane_change_decider_output().lc_request;
  bool is_left_lc =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == LEFT_CHANGE);
  bool is_right_lc =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == RIGHT_CHANGE);
  if (((lat_offset > 0 || is_right_lc) && frenet_obstacle.d_max_cpath() < 0) ||
      ((lat_offset < 0 || is_left_lc) && frenet_obstacle.d_min_cpath() > 0)) {
    std::array<double, 2> x1{50, 60};
    std::array<double, 2> f1{0, 0.1};
    double near_car_d_lane_buffer = interp(frenet_obstacle.d_s_rel(), x1, f1);
    distance_to_lane_line_thre += near_car_d_lane_buffer;
  }
  return distance_to_lane_line_thre;
}

double SccLateralObstacleDecider::GetAvoidCountThre(
    const FrenetObstacle &frenet_obstacle) {
  double ncar_count;
  const Obstacle &obstacle = *frenet_obstacle.obstacle();
  // for car
  if (obstacle.is_car()) {
    if (frenet_obstacle.d_s_rel() >= 20) {
      std::array<double, 2> xp2{-7.5, -2.5};
      std::array<double, 2> fp2{5, 20};
      ncar_count =
          interp(frenet_obstacle.frenet_relative_velocity_s(), xp2, fp2);
    } else {
      std::array<double, 4> xp2{-5, -2.499, 0, 1};
      std::array<double, 4> fp2{2, 3, 4, 20};
      std::array<double, 4> xp3{0, 2, 5, 10};
      std::array<double, 4> fp3{4, 3, 2, 0};

      ncar_count =
          interp(frenet_obstacle.frenet_relative_velocity_s(), xp2, fp2) +
          interp(ego_v_, xp3, fp3);
    }
    // for VRU
  } else if (obstacle.is_VRU()) {
    std::array<double, 2> xp2{20, 40};
    std::array<double, 2> fp2{5, 10};
    ncar_count = interp(frenet_obstacle.d_s_rel(), xp2, fp2);
    // TRAFFIC_BARRIER, TEMPORY_SIGN, FENCE, WATER_SAFETY_BARRIER, CTASH_BARREL
  } else {
    ncar_count = 1.0;
  }

  return ncar_count;
}

bool SccLateralObstacleDecider::UpdateObstacleAvoidCount(
    const FrenetObstacle &frenet_obstacle, bool is_avoidable, bool is_in_range,
    bool is_about_to_enter_range, double ncar_count, bool rightest_lane,
    bool is_lane_change) {
  const auto lead_one =
      session_->environmental_model().get_lateral_obstacle()->leadone();
  const Obstacle &obstacle = *frenet_obstacle.obstacle();
  const double v_lat = frenet_obstacle.frenet_velocity_lateral();

  LateralObstacleHistoryInfo &history =
      lateral_obstacle_history_info_[frenet_obstacle.id()];

  // if (obstacle.is_static()) {
  //   bool has_safe_space = CheckStaticObstacleAvoidSafety(frenet_obstacle);

  //   // 避免避让过程中，自车加速
  //   history.has_safe_space = (history.has_safe_space && frenet_obstacle.d_s_rel() < 20) || has_safe_space;
  // }

  const double gap = (history.last_recv_time == 0.0)
                         ? kPlanningCycleTime
                         : (obstacle.timestamp() - history.last_recv_time);
  const int count = (int)((gap + 0.01) / kPlanningCycleTime);

  if (lead_one != nullptr && lead_one->id() == frenet_obstacle.id()) {
    if ((is_in_range || is_about_to_enter_range) &&
        (!is_lane_change ||
         (is_lane_change &&
          frenet_obstacle.id() == lc_gap_info_.gap_front_id))) {
      double dist_intersect = 1000;
      double near_end_pos =
          0.5 * lane_width_ - 0.7 * (lane_width_ - ego_width_);
      double far_end_pos = 0.5 * lane_width_ + 0.2;

      bool is_in_avoid_range_by_nearest_point =
          lead_one->d_path() >= near_end_pos &&
          lead_one->d_path() < far_end_pos;

      bool is_in_avoid_range_by_nearest_line_in_left =
          lead_one->d_min_cpath() >= near_end_pos &&
          lead_one->d_min_cpath() < far_end_pos &&
          lead_one->d_max_cpath() >= near_end_pos;

      bool is_in_avoid_range_by_nearest_line_in_right =
          lead_one->d_max_cpath() > -far_end_pos &&
          lead_one->d_max_cpath() <= -near_end_pos &&
          lead_one->d_min_cpath() <= -near_end_pos;

      // 默认关闭
      bool borrow_bicycle_lane = false;
      if (!obstacle.is_static()) {
        if (!((history.is_avd_car) &&
              (is_in_avoid_range_by_nearest_point ||
              is_in_avoid_range_by_nearest_line_in_left ||
              is_in_avoid_range_by_nearest_line_in_right ||
              borrow_bicycle_lane || rightest_lane ||
              (dist_intersect - lead_one->d_s_rel() < 50 &&
                dist_intersect - lead_one->d_s_rel() >= -5 &&
                lead_one->type() ==
                    iflyauto::ObjectType::OBJECT_TYPE_TRAFFIC_CONE)))) {
          history.ncar_count =
              std::max(history.ncar_count - 10 * count * kPlanningCycleTime, 0.0);
        }
      }
    }
  }

  std::array<double, 4> x_cut_factor{0.2, 0.4, 0.6, 0.8};
  std::array<double, 4> f_cut_factor{10, 20, 30, 40};
  double cut_factor = interp(std::fabs(v_lat), x_cut_factor, f_cut_factor);
  bool is_normal_static_obstacle =
      obstacle.is_static() && (obstacle.accel_fusion() < 0.3 ||
                               (obstacle.accel_fusion() >= 0.3 && v_lat > -0.3));
  // 静态障碍物加速度小于0.3时，或大于0.3但是cut_out，不考虑横向速度，避免不避让
  if (is_avoidable) {
    // hack：missing prediction, considering v_lat
    // if (item.trajectory.intersection == 0 &&
    if ((v_lat > -0.3 && v_lat < 0.3) ||
        // 静止的车
        (obstacle.is_car() &&
         std::fabs(frenet_obstacle.frenet_velocity_s()) < 0.5 && v_lat > -0.3 &&
         v_lat < 0.3) ||
        // 横向无运动的人或锥桶 || 自车在最右车道
        (!obstacle.is_car() && (std::fabs(v_lat) < 0.3)) || rightest_lane ||
        is_normal_static_obstacle) {
      // hack: always true: 横向无运动的车 || 横向无运动的人或锥桶
      if (((v_lat > -0.3 && v_lat < 0.3 && obstacle.is_car()) ||
           (std::fabs(v_lat) < 0.3 && !obstacle.is_car())) ||
          is_normal_static_obstacle) {
        history.ncar_count =
            std::min(history.ncar_count + gap, 100 * kPlanningCycleTime);
      }
    } else {
      // cut in/out factor

      history.ncar_count = std::max(
          history.ncar_count - cut_factor * count * kPlanningCycleTime, 0.0);
    }

    if (history.ncar_count < ncar_count * kPlanningCycleTime &&
        obstacle.timestamp() > history.close_time + 2.5) {
      // is_ncar = false;
      return false;
    } else if (history.ncar_count >= ncar_count * kPlanningCycleTime) {
      if (history.ncar_count_in == false) {
        history.ncar_count = 100 * kPlanningCycleTime;
      }

      history.close_time = obstacle.timestamp();
      return true;
    }
  } else {
    // 这里衰减计数不合理，需要优化
    // 存在紧急避让，先取消这部分
    history.ncar_count =
        std::max(history.ncar_count - 2 * count * kPlanningCycleTime, 0.0);
    // if (!in_lon_near_area || !in_lat_near_area) {
    //   history.ncar_count =
    //       std::max(history.ncar_count - 2 * count * kPlanningCycleTime,
    //       0.0);
    // }

    if (frenet_obstacle.frenet_relative_velocity_s() > 1.5) {
      history.ncar_count =
          std::max(history.ncar_count - 5 * count * kPlanningCycleTime, 0.0);
    }

    // if (item.trajectory.intersection > 0) {
    //   history.ncar_count =
    //     std::max(history.ncar_count - 10 * count * kPlanningCycleTime, 0.0);
    // }

    // for cut in and cut out
    if (std::fabs(v_lat) > 0.3) {
      history.ncar_count = std::max(
          history.ncar_count - cut_factor * count * kPlanningCycleTime, 0.0);
    }

    if (frenet_obstacle.d_max_cpath() > 0 &&
        frenet_obstacle.d_min_cpath() < 0) {
      history.ncar_count = 0;
    }

    if (history.ncar_count > 60 * kPlanningCycleTime) {
      history.ncar_count_in = true;
      return true;
    } else {
      history.ncar_count = 0;
      history.ncar_count_in = false;
      return false;
    }
  }

  return false;
}

void SccLateralObstacleDecider::UpdateObstacleInteractionInfo() {
  if (CheckSpatioTemporalPlanner()) {
    return;
  }
  // 1 计算静态元素的free space
  UpdateObstacleInteractionInfoBaseStaticEnvironment();
  // 2 更新静态元素的避让标签，并取离自车最近的静态FOLLOW障碍物
  std::shared_ptr<FrenetObstacle> front_nearest_follow_obstacle = nullptr;
  UpdateStaticObstacleLateralDecisionBaseStaticFreeSpace(
      front_nearest_follow_obstacle);
  // 3 静态FOLLOW后面的静态障碍物，置为FOLLOW
  UpdateLateralObstacleDecisionBaseNearestFollowObstacle(
      front_nearest_follow_obstacle);
  // 4 计算动态元素的free space
  UpdateObstacleInteractionInfoBaseDynamicEnvironment(front_nearest_follow_obstacle);
  // 5 更新静态元素的避让标签
  UpdateStaticObstacleLateralDecisionBaseDynamicFreeSpace();

  // 6 计算紧急避让和时空FOLLOW
  for (auto frenet_obs : reference_path_ptr_->get_obstacles()) {
    if (frenet_obs == nullptr) {
      continue;
    }
    const Obstacle *obs = frenet_obs->obstacle();
    if (!(obs->fusion_source() & OBSTACLE_SOURCE_CAMERA) ||
        !frenet_obs->b_frenet_valid()) {
      continue;
    }
    CheckLateralEmergencyAvoidObstacle(*frenet_obs);
    GenerateSpatioTemporalFollowDecision(*frenet_obs);
  }
}

bool SccLateralObstacleDecider::CheckSpatioTemporalPlanner() {
  constexpr double kDistanceThresholdApproachToStopline = 10.0;
  constexpr int kEgoInIntersectionCount = 3;
  const auto lc_state = session_->planning_context()
                                .lane_change_decider_output()
                                .coarse_planning_info.target_state;
  const auto& construction_scene_output = session_->environmental_model()
                                              .get_construction_scene_manager()
                                              ->get_construction_scene_output();
  const auto& tfl_decider = session_->mutable_planning_context()
                                ->mutable_traffic_light_decider_output();
  const auto intersection_state = session_->environmental_model()
                                      .get_virtual_lane_manager()
                                      ->GetIntersectionState();
  const double distance_to_stopline = session_->environmental_model()
                                          .get_virtual_lane_manager()
                                          ->GetEgoDistanceToStopline();
  const double distance_to_crosswalk = session_->environmental_model()
                                           .get_virtual_lane_manager()
                                           ->GetEgoDistanceToCrosswalk();
  bool current_intersection_state =
      intersection_state == common::IntersectionState::IN_INTERSECTION ||
      distance_to_stopline <= kDistanceThresholdApproachToStopline;
  bool is_small_intersection = false;
  // bool is_small_intersection = tfl_decider.is_small_front_intersection;
  // distance_to_crosswalk <= kDistanceThresholdApproachToCrosswalk;
  if (current_intersection_state) {
    spatio_temporal_planner_intersection_count_ = kEgoInIntersectionCount;
  } else {
    spatio_temporal_planner_intersection_count_ =
        std::max(spatio_temporal_planner_intersection_count_ - 1, 0);
  }

  if (!config_.enable_use_spatio_temporal_planning) {
    return false;
  }

  if (lc_state != kLaneKeeping) {
    return false;
  }

  if (!(spatio_temporal_planner_intersection_count_ > 0 &&
        !is_small_intersection) &&
      !construction_scene_output.enable_construction_passage) {
    return false;
  }

  return true;
}

void SccLateralObstacleDecider::
    UpdateObstacleInteractionInfoBaseStaticEnvironment() {
  // 静态环境的交互
  for (auto frenet_obs : reference_path_ptr_->get_obstacles()) {
    if (frenet_obs == nullptr) {
      continue;
    }
    const auto obstacle_id = frenet_obs->id();
    ObstacleInfo& obstacle_interaction_info =
        obstacle_interaction_map_[obstacle_id];
    if (!frenet_obs->is_static() || frenet_obs->d_s_rel() <= 0) {
      // 目前仅对静态障碍物处理
      obstacle_interaction_info.Reset();
      obstacle_interaction_info.id = obstacle_id;
      continue;
    }
    UpdateObstaclelateraFreeSpaceAndTypeBaseStaticEnvironment(
        *frenet_obs, obstacle_interaction_info);
  }
}

void SccLateralObstacleDecider::
    UpdateObstacleInteractionInfoBaseDynamicEnvironment(
        std::shared_ptr<FrenetObstacle>& front_nearest_follow_obstacle) {
  // 动态环境的交互
  for (auto frenet_obs : reference_path_ptr_->get_obstacles()) {
    if (frenet_obs == nullptr) {
      continue;
    }
    if (obstacle_interaction_map_.find(frenet_obs->id()) ==
        obstacle_interaction_map_.end()) {
      // 不存在静态障碍物的交互
      continue;
    }
    CalLateralFreeSpaceBaseDynamicObstacle(
        *frenet_obs, obstacle_interaction_map_[frenet_obs->id()],
        front_nearest_follow_obstacle);
  }
}

void SccLateralObstacleDecider::
    UpdateStaticObstacleLateralDecisionBaseStaticFreeSpace(
        std::shared_ptr<FrenetObstacle>& front_nearest_follow_obstacle) {
  // 计算静态障碍物的决策标签,并输出最近的静态FOLLOW障碍物，
  // 动态FOLLOW，选择性比较多，可以跟着动态，避让静态
  const double kSafeLaneBoundarySpace = 0.6;
  const double kSafeRoadBoundarySpace = 0.6;
  const double kSafeStaticObstacleSpace = 0.7;
  const double kDesireVEnterMargin = 0;
  const double kDesireVExitMargin  = 3;

  for (auto frenet_obstacle : reference_path_ptr_->get_obstacles()) {
    if (frenet_obstacle == nullptr) {
      continue;
    }

    const Obstacle* obs = frenet_obstacle->obstacle();
    if (!(obs->fusion_source() & OBSTACLE_SOURCE_CAMERA) ||
        !frenet_obstacle->b_frenet_valid()) {
      continue;
    }

    if (!frenet_obstacle->is_static()) {
      continue;
    }

    if (output_.find(frenet_obstacle->id()) == output_.end()) {
      continue;
    }

    if (output_[frenet_obstacle->id()] == LatObstacleDecisionType::FOLLOW) {
      if (front_nearest_follow_obstacle == nullptr) {
        front_nearest_follow_obstacle = frenet_obstacle;
      } else {
        if (frenet_obstacle->frenet_obstacle_boundary().s_start <
            front_nearest_follow_obstacle->frenet_obstacle_boundary().s_start) {
          front_nearest_follow_obstacle = frenet_obstacle;
        }
      }
    }

    if (!(output_[frenet_obstacle->id()] == LatObstacleDecisionType::RIGHT ||
          output_[frenet_obstacle->id()] == LatObstacleDecisionType::LEFT)) {
      continue;
    }

    if (lateral_obstacle_history_info_.find(frenet_obstacle->id()) ==
        lateral_obstacle_history_info_.end()) {
      continue;
    }

    if (obstacle_interaction_map_.find(frenet_obstacle->id()) ==
        obstacle_interaction_map_.end()) {
      continue;
    }

    LateralObstacleHistoryInfo& history =
        lateral_obstacle_history_info_[frenet_obstacle->id()];
    ObstacleInfo& obstacle_interaction_info =
        obstacle_interaction_map_[frenet_obstacle->id()];
    double lane_boundary_free_space_desire_v = interp(
        obstacle_interaction_info.GetLateralConstraintDistance(
            LateralSpaceConstraintType::LANE_BOUNDARY),
        config_.free_space_lane_bp, config_.lane_static_limit_v_free_space);
    double road_boundary_free_space_desire_v = interp(
        obstacle_interaction_info.GetLateralConstraintDistance(
            LateralSpaceConstraintType::ROAD_BOUNDARY),
        config_.free_space_road_bp, config_.road_static_limit_v_free_space);
    double static_obstacle_free_space_desire_v =
        interp(obstacle_interaction_info.GetLateralConstraintDistance(
                   LateralSpaceConstraintType::STATIC_OBSTACLE),
               config_.free_space_static_obstacle_bp,
               config_.static_obstacle_static_limit_v_free_space);
    double desire_v = std::min(lane_boundary_free_space_desire_v,
                               road_boundary_free_space_desire_v);
    desire_v = std::min(desire_v, static_obstacle_free_space_desire_v);
    double space_hysteresis = 0;
    if (last_output_[frenet_obstacle->id()] == LatObstacleDecisionType::LEFT ||
        last_output_[frenet_obstacle->id()] == LatObstacleDecisionType::RIGHT) {
      space_hysteresis = -0.1;
    } else {
      space_hysteresis = 0.1;
    }

    bool has_safe_v_space_current = desire_v >= ego_v_;
    bool has_safe_v_space =
        has_safe_v_space_current ||
        (obstacle_interaction_info.has_static_safe_v_space &&
         ego_v_ <= obstacle_interaction_info.desire_nudge_static_safe_v);

    // 如果上一帧能避让，则允许加速
    bool has_enough_space =
        obstacle_interaction_info.GetLateralConstraintDistance(
            LateralSpaceConstraintType::ROAD_BOUNDARY) >
        kSafeRoadBoundarySpace + space_hysteresis &&
        obstacle_interaction_info.GetLateralConstraintDistance(
            LateralSpaceConstraintType::STATIC_OBSTACLE) >
        kSafeStaticObstacleSpace + space_hysteresis;
    // 如果上一帧不避让，则当前帧的space_hysteresis加滞回，防止抖动
    // 后续需要优化这个条件
    if (!(has_safe_v_space && has_enough_space)) {
      output_[frenet_obstacle->id()] = LatObstacleDecisionType::FOLLOW;
      lateral_obstacle_history_info_[frenet_obstacle->id()].is_avd_car = false;
      if (front_nearest_follow_obstacle == nullptr) {
        front_nearest_follow_obstacle = frenet_obstacle;
      } else {
        if (frenet_obstacle->frenet_obstacle_boundary().s_start <
            front_nearest_follow_obstacle->frenet_obstacle_boundary().s_start) {
          front_nearest_follow_obstacle = frenet_obstacle;
        }
      }
    }
    if (has_safe_v_space_current) {
      // 低速状态下允许自车速度变化较快，但是中高速时，这个条件需要适当缩小
      std::array<double, 5> x_v_factor{0.5, 4.17, 8.33, 16.67, 33.33};
      std::array<double, 5> f_v_factor{5, 4.5, 4, 4, 2.7};
      double desire_v_hysteresis = interp(std::fabs(ego_v_), x_v_factor, f_v_factor);
      obstacle_interaction_info.desire_nudge_static_safe_v =
          ego_v_ + desire_v_hysteresis;
    }
    obstacle_interaction_info.has_static_safe_v_space = has_safe_v_space;
  }
}

void SccLateralObstacleDecider::
    UpdateLateralObstacleDecisionBaseNearestFollowObstacle(
        std::shared_ptr<FrenetObstacle>& front_nearest_follow_obstacle) {
  if (front_nearest_follow_obstacle == nullptr) {
    return;
  }
  for (auto frenet_obstacle : reference_path_ptr_->get_obstacles()) {
    if (frenet_obstacle == nullptr) {
      continue;
    }

    const Obstacle* obs = frenet_obstacle->obstacle();
    if (!(obs->fusion_source() & OBSTACLE_SOURCE_CAMERA) ||
        !frenet_obstacle->b_frenet_valid()) {
      continue;
    }

    if (!frenet_obstacle->is_static()) {
      continue;
    }

    if (output_.find(frenet_obstacle->id()) == output_.end()) {
      continue;
    }

    if (!(output_[frenet_obstacle->id()] == LatObstacleDecisionType::RIGHT ||
          output_[frenet_obstacle->id()] == LatObstacleDecisionType::LEFT)) {
      continue;
    }

    if (frenet_obstacle->frenet_obstacle_boundary().s_start >
        front_nearest_follow_obstacle->frenet_obstacle_boundary().s_start) {
      output_[frenet_obstacle->id()] = LatObstacleDecisionType::IGNORE;
    }
  }
}

void SccLateralObstacleDecider::
    UpdateStaticObstacleLateralDecisionBaseDynamicFreeSpace() {
  // 计算静态障碍物的决策标签
  const double kSafeDynamicObstacleSpace = 0.8;
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double half_ego_width = 0.5 * vehicle_param.width;
  for (auto frenet_obstacle : reference_path_ptr_->get_obstacles()) {
    if (frenet_obstacle == nullptr) {
      continue;
    }
    const Obstacle* obs = frenet_obstacle->obstacle();
    if (!(obs->fusion_source() & OBSTACLE_SOURCE_CAMERA) ||
        !frenet_obstacle->b_frenet_valid()) {
      continue;
    }

    if (!frenet_obstacle->is_static()) {
      continue;
    }

    if (!(output_[frenet_obstacle->id()] == LatObstacleDecisionType::RIGHT ||
          output_[frenet_obstacle->id()] == LatObstacleDecisionType::LEFT)) {
      continue;
    }

    if (output_.find(frenet_obstacle->id()) == output_.end()) {
      continue;
    }

    if (lateral_obstacle_history_info_.find(frenet_obstacle->id()) ==
        lateral_obstacle_history_info_.end()) {
      continue;
    }

    if (obstacle_interaction_map_.find(frenet_obstacle->id()) ==
        obstacle_interaction_map_.end()) {
      continue;
    }

    double space_hysteresis = 0;
    if (last_output_[frenet_obstacle->id()] == LatObstacleDecisionType::LEFT ||
        last_output_[frenet_obstacle->id()] == LatObstacleDecisionType::RIGHT) {
      space_hysteresis = -0.1;
    } else {
      space_hysteresis = 0.25;// 动态障碍物不确定性比静态障碍物大
    }

    LateralObstacleHistoryInfo& history =
        lateral_obstacle_history_info_[frenet_obstacle->id()];
    ObstacleInfo& obstacle_interaction_info =
        obstacle_interaction_map_[frenet_obstacle->id()];
    double dynamic_obstacle_free_space_desire_v =
        interp(obstacle_interaction_info.GetLateralConstraintDistance(
                   LateralSpaceConstraintType::DYNAMIC_OBSTACLE),
               config_.free_space_dynamic_obstacle_bp,
               config_.dynamic_obstacle_static_limit_v_free_space);
    bool has_safe_v_space_current = dynamic_obstacle_free_space_desire_v >= ego_v_;
    bool has_safe_v_space =
        has_safe_v_space_current ||
        (obstacle_interaction_info.has_dynamic_safe_v_space &&
         ego_v_ <= obstacle_interaction_info.desire_nudge_dynamic_safe_v);

    // 如果上一帧能避让，则当前帧速度允许加速到期望速度的两倍
    bool has_enough_space =
        obstacle_interaction_info.GetLateralConstraintDistance(
            LateralSpaceConstraintType::DYNAMIC_OBSTACLE) >
        kSafeDynamicObstacleSpace + space_hysteresis;
    // 如果上一帧不避让，则当前帧的space_hysteresis加滞回，防止抖动
    // 后续需要优化这个条件
    if (!(has_safe_v_space && has_enough_space)) {
      double nudge_buffer_hysteresis = 0;
      if (last_output_[frenet_obstacle->id()] ==
          LatObstacleDecisionType::FOLLOW) {
        // 0.1的距离滞回
        nudge_buffer_hysteresis = 0.1;
      }
      bool is_pre_nudge_over_center_lane = false;
      double nudge_buffer = CalDesireStaticLateralDistance(*frenet_obstacle);
      if (frenet_obstacle->d_min_cpath() > 0) {
        is_pre_nudge_over_center_lane =
            frenet_obstacle->d_min_cpath() - nudge_buffer - half_ego_width <
            -kNudgeDistanceToCenterLineThreshold + nudge_buffer_hysteresis;
      } else if (frenet_obstacle->d_max_cpath() < 0) {
        is_pre_nudge_over_center_lane =
            frenet_obstacle->d_max_cpath() + nudge_buffer + half_ego_width >
            kNudgeDistanceToCenterLineThreshold - nudge_buffer_hysteresis;
      }
      if (is_pre_nudge_over_center_lane) {
        output_[frenet_obstacle->id()] = LatObstacleDecisionType::FOLLOW;
        lateral_obstacle_history_info_[frenet_obstacle->id()].is_avd_car =
            false;
      }
    }
    if (has_safe_v_space_current) {
      std::array<double, 5> x_v_factor{1, 4.17, 8.33, 16.67,33.33};
      std::array<double, 5> f_v_factor{2, 2, 1.5, 1.2, 1.08};
      double desire_v_hysteresis = interp(std::fabs(ego_v_), x_v_factor, f_v_factor);
      obstacle_interaction_info.desire_nudge_dynamic_safe_v =
          ego_v_ * desire_v_hysteresis;
    }
    obstacle_interaction_info.has_dynamic_safe_v_space = has_safe_v_space;
  }
}

double SccLateralObstacleDecider::CalDesireStaticLateralDistance(
    const FrenetObstacle& frenet_obstacle) {
  const double kStaticVRUMaxExtraLateralBuffer = 0.7;
  const double kConeMaxExtraLateralBuffer = 0.15;
  const double kBarrelMaxExtraLateralBuffer = 0.25;
  const double kStaticOtherMaxExtraLateralBuffer = 0.35;
  const double kExtraTruckNudgeBuffer = 0.2;
  const double kNudgeBaseDistance = 0.4;

  double max_extra_lateral_buffer = 0;
  if (frenet_obstacle.obstacle()->is_VRU()) {
    max_extra_lateral_buffer = kStaticVRUMaxExtraLateralBuffer;
  } else if (frenet_obstacle.obstacle()->type() ==
             iflyauto::ObjectType::OBJECT_TYPE_TRAFFIC_CONE) {
    max_extra_lateral_buffer = kConeMaxExtraLateralBuffer;
  } else if (frenet_obstacle.obstacle()->type() ==
             iflyauto::ObjectType::OBJECT_TYPE_CTASH_BARREL) {
    max_extra_lateral_buffer = kBarrelMaxExtraLateralBuffer;
  } else {
    max_extra_lateral_buffer = kStaticOtherMaxExtraLateralBuffer;
  }
  if (IsTruck(frenet_obstacle)) {
    max_extra_lateral_buffer += kExtraTruckNudgeBuffer;
  }
  return kNudgeBaseDistance + max_extra_lateral_buffer;
}

void SccLateralObstacleDecider::
    UpdateObstaclelateraFreeSpaceAndTypeBaseStaticEnvironment(
        const FrenetObstacle& frenet_obstacle,
        ObstacleInfo& obstacle_interaction_info) {
  double d_min_cpath = frenet_obstacle.d_min_cpath();
  double d_max_cpath = frenet_obstacle.d_max_cpath();
  const auto obstacle_id = frenet_obstacle.id();
  obstacle_interaction_info.id = obstacle_id;
  if (d_max_cpath > 0 && d_min_cpath < 0) {
    obstacle_interaction_info.lateral_avoid_direction =
        LatObstacleDecisionType::IGNORE;
  } else if (d_max_cpath < 0) {
    obstacle_interaction_info.lateral_avoid_direction =
        LatObstacleDecisionType::LEFT;
  } else if (d_min_cpath > 0) {
    obstacle_interaction_info.lateral_avoid_direction =
        LatObstacleDecisionType::RIGHT;
  }
  // 计算障碍物不同类型的空间
  CalLateralFreeSpaceBaseLaneBoundary(frenet_obstacle,
                                      obstacle_interaction_info);
  CalLateralFreeSpaceBaseRoadBoundary(frenet_obstacle,
                                      obstacle_interaction_info);
  CalLateralFreeSpaceBaseStaticObstacle(frenet_obstacle,
                                        obstacle_interaction_info);
}

void SccLateralObstacleDecider::CalLateralFreeSpaceBaseLaneBoundary(
    const FrenetObstacle& frenet_obstacle,
    ObstacleInfo& obstacle_interaction_info) {
  double free_space = 100;
  double s_with_min_lat_space = 0;
  ReferencePathPoint reference_path_point;
  if (obstacle_interaction_info.lateral_avoid_direction ==
      LatObstacleDecisionType::LEFT) {
    s_with_min_lat_space = frenet_obstacle.s_max_l().y;
    if (reference_path_ptr_->get_reference_point_by_lon(s_with_min_lat_space,
                                                        reference_path_point)) {
      free_space = reference_path_point.distance_to_left_lane_border -
                   frenet_obstacle.s_max_l().x;
    }
  } else if (obstacle_interaction_info.lateral_avoid_direction ==
             LatObstacleDecisionType::RIGHT) {
    s_with_min_lat_space = frenet_obstacle.s_min_l().y;
    if (reference_path_ptr_->get_reference_point_by_lon(s_with_min_lat_space,
                                                        reference_path_point)) {
      free_space = reference_path_point.distance_to_right_lane_border +
                   frenet_obstacle.s_min_l().x;
    }
  } else {
    free_space = 0;  // 表示不对该障碍物进行避让
  }
  free_space = std::max(0.0, free_space - ego_width_);
  obstacle_interaction_info.SetLateralConstraintFreeSpace(
      LateralSpaceConstraintType::LANE_BOUNDARY, free_space);
}

void SccLateralObstacleDecider::CalLateralFreeSpaceBaseRoadBoundary(
    const FrenetObstacle& frenet_obstacle,
    ObstacleInfo& obstacle_interaction_info) {
  double free_space = 100;
  double s_with_min_lat_space = 0;
  ReferencePathPoint reference_path_point;
  if (obstacle_interaction_info.lateral_avoid_direction ==
      LatObstacleDecisionType::LEFT) {
    s_with_min_lat_space = frenet_obstacle.s_max_l().y;
    if (reference_path_ptr_->get_reference_point_by_lon(s_with_min_lat_space,
                                                        reference_path_point)) {
      free_space = reference_path_point.distance_to_left_road_border -
                   frenet_obstacle.s_max_l().x;
    }
  } else if (obstacle_interaction_info.lateral_avoid_direction ==
             LatObstacleDecisionType::RIGHT) {
    s_with_min_lat_space = frenet_obstacle.s_min_l().y;
    if (reference_path_ptr_->get_reference_point_by_lon(s_with_min_lat_space,
                                                        reference_path_point)) {
      free_space = reference_path_point.distance_to_right_road_border +
                   frenet_obstacle.s_min_l().x;
    }
  } else {
    free_space = 0;  // 表示不对该障碍物进行避让
  }
  obstacle_interaction_info.SetLateralConstraintFreeSpace(
      LateralSpaceConstraintType::ROAD_BOUNDARY, free_space);
}

void SccLateralObstacleDecider::CalLateralFreeSpaceBaseStaticObstacle(
    const FrenetObstacle& frenet_obstacle,
    ObstacleInfo& obstacle_interaction_info) {
  double free_space = 100;
  // 膨胀buffer，与静态障碍物hard bound膨胀buffer一致
  double front_lon_buf_dis = 1.0;
  double rear_lon_buf_dis = 1.0;
  double kNearEgoLongDistanceBuffer =
      front_lon_buf_dis + rear_lon_buf_dis + ego_length_;
  if (obstacle_interaction_info.lateral_avoid_direction ==
      LatObstacleDecisionType::IGNORE) {
    obstacle_interaction_info.SetLateralConstraintFreeSpace(
        LateralSpaceConstraintType::STATIC_OBSTACLE, 0);
    return;
  }
  for (auto frenet_obs : reference_path_ptr_->get_obstacles()) {
    if (frenet_obs == nullptr) {
      continue;
    }
    if (!frenet_obs->is_static()) {
      continue;
    }
    if (frenet_obs->id() == frenet_obstacle.id()) {
      continue;
    }
    // 判断是否有overlap
    bool is_overlap_side =
        std::max(frenet_obstacle.frenet_obstacle_boundary().s_start,
                 frenet_obs->frenet_obstacle_boundary().s_start -
                     kNearEgoLongDistanceBuffer) <
        std::min(frenet_obstacle.frenet_obstacle_boundary().s_end,
                 frenet_obs->frenet_obstacle_boundary().s_end +
                     kNearEgoLongDistanceBuffer);
    if (!is_overlap_side) {
      continue;
    }
    if (obstacle_interaction_info.lateral_avoid_direction ==
        LatObstacleDecisionType::LEFT) {
      if (frenet_obs->d_min_cpath() > 0) {
        // 两个障碍物避让方向是否一致，后续通过初始标志位方向进行筛选
        double free_space_tmp =
            frenet_obs->d_min_cpath() - frenet_obstacle.d_max_cpath();
        // 不同类型的障碍物交互，空间权重不一样
        UpdateStaticFreeSpaceBaseInteractionType(*frenet_obs, free_space_tmp);
        free_space = std::fmin(free_space_tmp, free_space);
      } else {
        continue;
      }
    } else if (obstacle_interaction_info.lateral_avoid_direction ==
               LatObstacleDecisionType::RIGHT) {
      if (frenet_obs->d_max_cpath() < 0) {
        // 两个障碍物避让方向是否一致，后续通过初始标志位方向进行筛选
        double free_space_tmp =
            -frenet_obs->d_max_cpath() + frenet_obstacle.d_min_cpath();
        // 不同类型的障碍物交互，空间权重不一样
        UpdateStaticFreeSpaceBaseInteractionType(*frenet_obs, free_space_tmp);
        free_space = std::fmin(free_space_tmp, free_space);
      } else {
        continue;
      }
    }
  }
  free_space = std::max(0.0, free_space - ego_width_);
  obstacle_interaction_info.SetLateralConstraintFreeSpace(
      LateralSpaceConstraintType::STATIC_OBSTACLE, free_space);
}

void SccLateralObstacleDecider::CalLateralFreeSpaceBaseDynamicObstacle(
    const FrenetObstacle& frenet_obstacle,
    ObstacleInfo& obstacle_interaction_info,
    std::shared_ptr<FrenetObstacle>& front_nearest_follow_obstacle) {
  double free_space = 100;
  const double kLatOverlapBuffer = 0.3;

  if (obstacle_interaction_info.lateral_avoid_direction ==
      LatObstacleDecisionType::IGNORE) {
    obstacle_interaction_info.SetLateralConstraintFreeSpace(
        LateralSpaceConstraintType::DYNAMIC_OBSTACLE, 0);
    return;
  }
  for (auto frenet_obs : reference_path_ptr_->get_obstacles()) {
    if (frenet_obs == nullptr) {
      continue;
    }
    if (frenet_obs->is_static()) {
      continue;
    }
    if (frenet_obs->id() == frenet_obstacle.id()) {
      continue;
    }
    // 过滤正后方障碍物
    const double ego_l_start =
        reference_path_ptr_->get_ego_frenet_boundary().l_start;
    const double ego_l_end =
        reference_path_ptr_->get_ego_frenet_boundary().l_end;
    const double obstacle_l_start =
        frenet_obs->frenet_obstacle_boundary().l_start;
    const double obstacle_l_end =
        frenet_obs->frenet_obstacle_boundary().l_end;
    double start_l = std::max(ego_l_start, obstacle_l_start);
    double end_l = std::min(ego_l_end, obstacle_l_end);
    bool lat_overlap_for_rear_obs = (start_l < end_l - kLatOverlapBuffer);
    if (reference_path_ptr_->get_ego_frenet_boundary().s_start >
        frenet_obs->frenet_obstacle_boundary().s_end) {
      // 计算后方车ttc
      double ttc =
          frenet_obs->frenet_relative_velocity_s() > 0
              ? ((reference_path_ptr_->get_ego_frenet_boundary().s_start -
                  frenet_obs->frenet_obstacle_boundary().s_end) /
                 frenet_obs->frenet_relative_velocity_s())
              : std::numeric_limits<double>::max();
      bool is_care_rear_obstacle =
          ttc < kRearVehicleTTCThreshold && !lat_overlap_for_rear_obs;
      if (!is_care_rear_obstacle) {
        continue;
      }
    }

    if (obstacle_interaction_info.lateral_avoid_direction ==
        LatObstacleDecisionType::LEFT) {
      // 两个障碍物避让方向是否一致，后续通过初始标志位方向进行筛选
      if (frenet_obs->d_min_cpath() > 0) {
        // 按照自车匀速递推的轨迹，判断自车是否与动态障碍物存在overlap
        double min_right_boundary_l = 100;
        CheckEgoOverlapDynamicObstacle(*frenet_obs, frenet_obstacle,
                                       min_right_boundary_l, true,
                                       front_nearest_follow_obstacle);
        double free_space_tmp = min_right_boundary_l - frenet_obstacle.d_max_cpath();
        // 不同类型的障碍物交互，空间权重不一样
        UpdateDynamicFreeSpaceBaseInteractionType(*frenet_obs, free_space_tmp);
        free_space = std::fmin(free_space_tmp, free_space);
      } else {
        continue;
      }
    } else if (obstacle_interaction_info.lateral_avoid_direction ==
               LatObstacleDecisionType::RIGHT) {
      if (frenet_obs->d_max_cpath() < 0) {
        // 两个障碍物避让方向是否一致，后续通过初始标志位方向进行筛选
        double max_left_boundary_l = -100;
        CheckEgoOverlapDynamicObstacle(*frenet_obs, frenet_obstacle,
                                       max_left_boundary_l, false,
                                       front_nearest_follow_obstacle);
        double free_space_tmp = frenet_obstacle.d_min_cpath() - max_left_boundary_l;
        // 不同类型的障碍物交互，空间权重不一样
        UpdateDynamicFreeSpaceBaseInteractionType(*frenet_obs, free_space_tmp);
        free_space = std::fmin(free_space_tmp, free_space);
      } else {
        continue;
      }
    }
  }
  free_space = std::max(0.0, free_space - ego_width_);
  obstacle_interaction_info.SetLateralConstraintFreeSpace(
      LateralSpaceConstraintType::DYNAMIC_OBSTACLE, free_space);
}

void SccLateralObstacleDecider::CheckEgoOverlapDynamicObstacle(
    const FrenetObstacle& frenet_obstacle,
    const FrenetObstacle& target_static_obstacle,
    double& boundary_l, bool is_right_boundary,
    std::shared_ptr<FrenetObstacle>& front_nearest_follow_obstacle) {
  const auto& cipv_info = session_->planning_context().cipv_decider_output();
  const auto& obstacle_map = reference_path_ptr_->get_obstacles_map();
  // 后续使用时，要注意cipv跟frenet_obstacle不能是同一个障碍物
  if (obstacle_map.find(cipv_info.cipv_id()) != obstacle_map.end()) {
    if (!frenet_obstacle.obstacle()->is_reverse() &&
        frenet_obstacle.frenet_obstacle_boundary().s_start >
            obstacle_map.at(cipv_info.cipv_id())
                ->frenet_obstacle_boundary()
                .s_start) {
      return;
    }
  }

  if (front_nearest_follow_obstacle != nullptr) {
    if (!frenet_obstacle.obstacle()->is_reverse() &&
        frenet_obstacle.frenet_obstacle_boundary().s_start >
            front_nearest_follow_obstacle->frenet_obstacle_boundary().s_start) {
      return;
    }
  }

  if (!frenet_obstacle.obstacle()->is_reverse()) {
    if (frenet_obstacle.frenet_obstacle_boundary().s_start >
        target_static_obstacle.frenet_obstacle_boundary().s_end + 1) {
      return;
    }
  }

  // 先按照自车匀速递推的方式，计算与动态障碍物overlap的时刻
  // 然后计算动态障碍物与静态障碍物之间的横纵向距离
  // 后续考虑和TTC挂钩
  double front_lon_buf_dis = 1.0;
  double rear_lon_buf_dis = 1.0;
  double extra_lon_buf_dis_for_reverse_obstacle = 2.0;
  const auto ego_v = ego_v_s_;
  // 这里是和动态障碍物做博弈的关键点，自车的速度与静止障碍物无关
  const double a = 0.0;
  // 如果自车以a的加速度发现自车与动态障碍物没有恐慌感（overlap，说明可以避让这个障碍物）
  bool ok = false;
  bool is_overlap_with_dynamic_agent = false;
  bool is_overtake_target_static_obstacle = false;
  int start_idx = 0;     // 0.0s
  int end_idx   = 50;    // 5.0s  (50 * 0.1)
  int step_idx  = 2;     // 0.2s  (2 * 0.1)
  const double kIndexToTimeSec = 0.1;
  for (int t_idx = start_idx; t_idx <= end_idx; t_idx += step_idx) {
    Polygon2d obstacle_sl_polygon;
    ok = reference_path_ptr_->get_polygon_at_time(
        frenet_obstacle.id(), false, t_idx, obstacle_sl_polygon);
    double delt_s = kIndexToTimeSec * t_idx * ego_v_s_ +
                    0.5 * a * kIndexToTimeSec * kIndexToTimeSec * t_idx * t_idx;
    double ego_start_s =
        reference_path_ptr_->get_ego_frenet_boundary().s_start + delt_s;
    double ego_end_s =
        reference_path_ptr_->get_ego_frenet_boundary().s_end + delt_s;
    if (!ok) {
      continue;
    }
    double min_s = std::numeric_limits<double>::max();
    double max_s = std::numeric_limits<double>::lowest();
    double boundary_l_tmp = 100;
    if (!is_right_boundary) {
      boundary_l_tmp = -100;
    }
    for (auto& pt : obstacle_sl_polygon.points()) {
      if (is_right_boundary) {
        boundary_l_tmp = std::min(pt.y(), boundary_l_tmp);
      } else {
        boundary_l_tmp = std::max(pt.y(), boundary_l_tmp);
      }
      min_s = std::min(min_s, pt.x());
      max_s = std::max(max_s, pt.x());
    }
    if (!frenet_obstacle.obstacle()->is_reverse()) {
      is_overtake_target_static_obstacle =
          min_s > target_static_obstacle.frenet_obstacle_boundary().s_end +
                      rear_lon_buf_dis;
      is_overlap_with_dynamic_agent =
          std::max(min_s, ego_start_s - rear_lon_buf_dis) <
          std::min(max_s, ego_end_s + front_lon_buf_dis);
    } else {
      is_overtake_target_static_obstacle = false;
      is_overlap_with_dynamic_agent =
          std::max(min_s, ego_start_s - rear_lon_buf_dis -
                              extra_lon_buf_dis_for_reverse_obstacle) <
          std::min(max_s, ego_end_s + front_lon_buf_dis +
                              extra_lon_buf_dis_for_reverse_obstacle);
    }
    if (is_overtake_target_static_obstacle) {
      break;
    }
    if (!is_overlap_with_dynamic_agent) {
      continue;
    }
    if (is_right_boundary) {
      boundary_l = std::min(boundary_l, boundary_l_tmp);
    } else {
      boundary_l = std::max(boundary_l, boundary_l_tmp);
    }
  }
}

void SccLateralObstacleDecider::UpdateStaticFreeSpaceBaseInteractionType(
    const FrenetObstacle& frenet_obstacle, double& free_space) {
  // 静态障碍物暂默认不同类型空间恐慌感是一致的
  free_space = std::max(0.0, free_space);
}

void SccLateralObstacleDecider::UpdateDynamicFreeSpaceBaseInteractionType(
    const FrenetObstacle& frenet_obstacle, double& free_space) {
  // 动态的VRU和大车，恐慌感较强，横向距离额外减去0.3
  if (frenet_obstacle.obstacle()->is_VRU() || IsTruck(frenet_obstacle)) {
    free_space -= 0.3;
  }
  free_space = std::max(0.0, free_space);
}

bool SccLateralObstacleDecider::CheckStaticObstacleAvoidSafety(
    const FrenetObstacle& frenet_obstacle) {
  if (!frenet_obstacle.is_static()) {
    return true;
  }

  // || !lateral_obstacle_history_info_[frenet_obstacle.id()].is_avd_car
  if (lateral_obstacle_history_info_.find(frenet_obstacle.id()) ==
      lateral_obstacle_history_info_.end()) {
    return false;
  }
  if (obstacle_interaction_map_.find(frenet_obstacle.id()) ==
      obstacle_interaction_map_.end()) {
    return false;
  }
  // double free_space = 100;
  // double s_with_min_lat_space = 0;

  // ReferencePathPoint reference_path_point;
  // if (frenet_obstacle.d_max_cpath() < 0){
  //   s_with_min_lat_space = frenet_obstacle.s_max_l().y;

  //   // 后续可以考虑路沿
  //   if (reference_path_ptr_->get_reference_point_by_lon(s_with_min_lat_space,
  //   reference_path_point)) {
  //     free_space = reference_path_point.distance_to_left_lane_border -
  //     frenet_obstacle.s_max_l().x;
  //   }
  // } else if(frenet_obstacle.d_min_cpath() > 0) {
  //   s_with_min_lat_space = frenet_obstacle.s_min_l().y;
  //   if (reference_path_ptr_->get_reference_point_by_lon(s_with_min_lat_space,
  //   reference_path_point)) {
  //     free_space = reference_path_point.distance_to_right_lane_border +
  //     frenet_obstacle.s_min_l().x;
  //   }
  // } else {
  //   return false;
  // }

  // free_space = std::max(0.0, free_space - ego_width_);
  ObstacleInfo& obstacle_interaction_info =
      obstacle_interaction_map_[frenet_obstacle.id()];
  double lane_boundary_free_space_desire_v = interp(
      obstacle_interaction_info.GetLateralConstraintDistance(
          LateralSpaceConstraintType::LANE_BOUNDARY),
      config_.free_space_lane_bp, config_.lane_static_limit_v_free_space);
  if (lane_boundary_free_space_desire_v >= ego_v_) {
    return true;
  } else {
    return false;
  }
}

void SccLateralObstacleDecider::LateralObstacleDecision(
    const FrenetObstacle &frenet_obstacle, bool is_in_lane_change_execution_scene) {
  const Obstacle &obstacle = *frenet_obstacle.obstacle();
  LateralObstacleHistoryInfo &history =
      lateral_obstacle_history_info_[obstacle.id()];
  FollowObstacleInfo &follow_info = follow_obstacle_info_[obstacle.id()];
  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  const auto &gap_selector_decider_output =
      session_->planning_context().gap_selector_decider_output();
  bool is_LC_CHANGE =
      ((coarse_planning_info.target_state == kLaneChangeExecution) ||
       (coarse_planning_info.target_state == kLaneChangeComplete));
  bool is_LC_HOLD = coarse_planning_info.target_state == kLaneChangeHold;
  bool is_LC_BACK = coarse_planning_info.target_state == kLaneChangeCancel;
  bool lane_change_scene = false;
  if ((is_LC_CHANGE || is_LC_BACK || is_LC_HOLD) &&
      (gap_selector_decider_output.gap_selector_trustworthy)) {
    lane_change_scene = true;
  }
  // calculate info of obstacle
  int id = obstacle.id();
  iflyauto::ObjectType type = obstacle.type();
  double l = frenet_obstacle.frenet_l();
  double d_s_rel = frenet_obstacle.d_s_rel();
  double v_s_rel = frenet_obstacle.frenet_relative_velocity_s();
  double d_min_cpath = frenet_obstacle.d_min_cpath();
  double d_max_cpath = frenet_obstacle.d_max_cpath();

  double ref_dis = 1;
  double avoid_front_buffer = 0.0;

  bool lat_overlap =
      fabs(ego_head_l_ - l) < (ego_width_ + obstacle.width()) / 2;
  // 前方车辆
  if (history.is_avd_car) {
    // 已经被判定为is_avd_car的车，由于其他原因导致的ignore,
    // 赋予PRE_FOLLOW_WITHIN_LANE,需要特别注意，借道可以借FOLLOW_WITHIN_LANE
    if (d_max_cpath > 0 && d_min_cpath < 0) {
      output_[id] = LatObstacleDecisionType::IGNORE;
    } else if (d_max_cpath < 0) {
      output_[id] = LatObstacleDecisionType::LEFT;
    } else if (d_min_cpath > 0) {
      output_[id] = LatObstacleDecisionType::RIGHT;
    }
    // cut_in 或 横穿
    if (IsCutInIgnore(frenet_obstacle,is_in_lane_change_execution_scene)) {
      // output_[id] = LatObstacleDecisionType::FOLLOW;
      output_[id] = LatObstacleDecisionType::IGNORE;
    }
    if (follow_info.is_need_folow) {
      history.is_avd_car = false;
      output_[id] = LatObstacleDecisionType::FOLLOW;
    }
  } else if (d_s_rel > history.front_expand_len) {
    if (history.can_avoid) {
      const bool last_was_avoid =
          last_output_.find(id) != last_output_.end() &&
          (last_output_[id] == LatObstacleDecisionType::LEFT ||
           last_output_[id] == LatObstacleDecisionType::RIGHT);
      if (last_was_avoid) {
        avoid_front_buffer = config_.avoid_persistence_front_buffer;
      }
      if (obstacle.is_traffic_facilities()) {
        avoid_front_buffer += config_.traffic_cone_thr;
      }
      if (d_max_cpath < 0 &&
          std::fabs(d_max_cpath) > lane_width_ * 0.5 - avoid_front_buffer) {
        output_[id] = LatObstacleDecisionType::LEFT;
      } else if (d_min_cpath > lane_width_ * 0.5 - avoid_front_buffer) {
        output_[id] = LatObstacleDecisionType::RIGHT;
      } else {
        output_[id] = LatObstacleDecisionType::IGNORE;
      }
    } else {
      output_[id] = LatObstacleDecisionType::IGNORE;
    }
    // cut_in 或 横穿
    if (IsCutInIgnore(frenet_obstacle,is_in_lane_change_execution_scene)) {
      // output_[id] = LatObstacleDecisionType::FOLLOW;
      output_[id] = LatObstacleDecisionType::IGNORE;
    }

    // if (history.is_not_set) {
    //   output_[id] = LatObstacleDecisionType::NOT_SET;
    // }

    if (follow_info.is_need_folow) {
      history.is_avd_car = false;
      output_[id] = LatObstacleDecisionType::FOLLOW;
    }

    // 如果是静态IGNORE障碍物，直接赋予FOLLOW
    // 需要注意横向观测范围
    if (obstacle.is_static() && !history.is_not_set &&
        type != iflyauto::ObjectType::OBJECT_TYPE_TRAFFIC_CONE &&
        type != iflyauto::ObjectType::OBJECT_TYPE_CTASH_BARREL &&
        type != iflyauto::ObjectType::OBJECT_TYPE_WATER_SAFETY_BARRIER &&
        output_[id] == LatObstacleDecisionType::IGNORE) {
      output_[id] = LatObstacleDecisionType::FOLLOW;
    }

    // 平行车辆
  } else if (d_s_rel <= history.front_expand_len &&
             d_s_rel > -(ego_length_ + history.rear_expand_len)) {
    if (ego_head_l_ < l) {
      output_[id] = LatObstacleDecisionType::RIGHT;
    } else {
      output_[id] = LatObstacleDecisionType::LEFT;
    }
    // 防止感知误检，同时有横向和纵向overlap
    if (lat_overlap) {
      if (!in_intersection_) {
        obstacles_id_behind_ego_.emplace_back(id);
        output_[id] = LatObstacleDecisionType::IGNORE;
      } else {
        const auto &ego_state_manager =
            session_->environmental_model().get_ego_state_manager();
        planning_math::Box2d ego_box(
            {ego_state_manager->ego_pose().x +
                 ego_rear_axle_to_center_ *
                     std::cos(ego_state_manager->ego_pose().theta),
             ego_state_manager->ego_pose().y +
                 ego_rear_axle_to_center_ *
                     std::sin(ego_state_manager->ego_pose().theta)},
            ego_state_manager->ego_pose().theta, ego_length_, ego_width_);
        if (frenet_obstacle.obstacle()->perception_bounding_box().HasOverlap(
                ego_box)) {
          obstacles_id_behind_ego_.emplace_back(id);
          output_[id] = LatObstacleDecisionType::IGNORE;
        }
      }
    }
    // cut_in 或 横穿
    if (IsCutInIgnore(frenet_obstacle,is_in_lane_change_execution_scene)) {
      output_[id] = LatObstacleDecisionType::IGNORE;
    }
    // 后方车辆
  } else {
    bool lon_too_far = d_s_rel < -25;
    bool lat_too_far = std::abs(frenet_obstacle.l_relative_to_ego()) > 4;
    if (lon_too_far || lat_too_far || lane_change_scene) {
      if (d_max_cpath < 0 && !lat_overlap && d_max_cpath < -ref_dis) {
        output_[id] = LatObstacleDecisionType::LEFT;
      } else if (d_min_cpath > 0 && !lat_overlap && d_min_cpath > ref_dis) {
        output_[id] = LatObstacleDecisionType::RIGHT;
      } else {
        output_[id] = LatObstacleDecisionType::IGNORE;
        history.is_behind_ego = true;
        obstacles_id_behind_ego_.emplace_back(id);
      }
    } else {
      const double l_offset_thr = 0.2;
      const double heading_thr = 0.017;
      const double v_l_thr = 0.3;
      const double ego_l = reference_path_ptr_->get_frenet_ego_state().l();
      const double ego_v_l =
          reference_path_ptr_->get_frenet_ego_state().velocity_l();
      const double ego_heading =
          reference_path_ptr_->get_frenet_ego_state().heading_angle();
      if (std::fabs(ego_l) >= l_offset_thr ||
          std::fabs(ego_heading) >= heading_thr ||
          std::fabs(ego_v_l) >= v_l_thr) {
        const double ego_l_start =
            reference_path_ptr_->get_frenet_ego_state().polygon().min_y();
        const double ego_l_end =
            reference_path_ptr_->get_frenet_ego_state().polygon().max_y();

        const double obstacle_l_start =
            frenet_obstacle.frenet_obstacle_boundary().l_start;
        const double obstacle_l_end =
            frenet_obstacle.frenet_obstacle_boundary().l_end;
        double start_l = std::max(ego_l_start, obstacle_l_start);
        double end_l = std::min(ego_l_end, obstacle_l_end);
        constexpr double kLatOverlapBuffer = 0.5;
        bool lat_overlap_for_rear_obs = (start_l < end_l - kLatOverlapBuffer);
        if (lat_overlap_for_rear_obs) {
          output_[id] = LatObstacleDecisionType::IGNORE;
          history.is_behind_ego = true;
          obstacles_id_behind_ego_.emplace_back(id);
        } else if (ego_l < l) {
          output_[id] = LatObstacleDecisionType::RIGHT;
        } else {
          output_[id] = LatObstacleDecisionType::LEFT;
        }
      } else {
        output_[id] = LatObstacleDecisionType::IGNORE;
        history.is_behind_ego = true;
        obstacles_id_behind_ego_.emplace_back(id);
      }
    }
  }

  if (is_in_lane_change_execution_scene && d_s_rel > history.front_expand_len &&
      frenet_obstacle.frenet_obstacle_boundary().s_start <
          lc_gap_info_.gap_front_s) {
    // 在变道状态，依据gap计算
    if (d_max_cpath > 0 && d_min_cpath < 0) {
      output_[id] = LatObstacleDecisionType::IGNORE;
    } else if (d_max_cpath < 0) {
      output_[id] = LatObstacleDecisionType::LEFT;
    } else if (d_min_cpath > 0) {
      output_[id] = LatObstacleDecisionType::RIGHT;
    }
  }
  // cut_in 或 横穿
  if (IsCutInIgnore(frenet_obstacle,is_in_lane_change_execution_scene)) {
    history.is_avd_car = false;
    history.ncar_count = 0;
    history.ncar_count_in = false;
  }
}

void SccLateralObstacleDecider::GenerateSpatioTemporalFollowDecision(
    const FrenetObstacle &frenet_obstacle) {
  const Obstacle &obstacle = *frenet_obstacle.obstacle();
  int id = obstacle.id();
  LateralObstacleHistoryInfo &history =
      lateral_obstacle_history_info_[id];
  FollowObstacleInfo &follow_info = follow_obstacle_info_[id];
  if (output_[id] != LatObstacleDecisionType::FOLLOW) {
    return;
  }
  double base_safe_intrusoin_for_dynamic =
      config_.base_safe_intrusoin_for_dynamic;
  double extra_buffer_for_truck = config_.extra_buffer_for_truck;
  double extra_buffer_for_vru = config_.extra_buffer_for_vru;
  double lead_d_path_thr = base_safe_intrusoin_for_dynamic;
  if (IsTruck(frenet_obstacle)) {
    lead_d_path_thr += extra_buffer_for_truck;
  } else if (obstacle.is_VRU()) {
    lead_d_path_thr += extra_buffer_for_vru;
  }
  double lane_width_actor =
      interp(lane_width_, config_.lane_width_bp, config_.lane_width_factor);
  lead_d_path_thr = lead_d_path_thr * lane_width_actor;
  double lat_safety_buffer = CalDynamicLatBuffer(frenet_obstacle);
  lead_d_path_thr = std::max(
      0.0,
      std::min(lane_width_ - lat_safety_buffer - ego_width_, lead_d_path_thr));
  lead_d_path_thr =
      lead_d_path_thr * config_.extra_ratio_for_cut_out;  // cut_out 相对保守
  bool is_vru =
      obstacle.type() == iflyauto::ObjectType::OBJECT_TYPE_PEDESTRIAN ||
      obstacle.type() == iflyauto::ObjectType::OBJECT_TYPE_BICYCLE ||
      obstacle.type() == iflyauto::ObjectType::OBJECT_TYPE_MOTORCYCLE ||
      obstacle.type() == iflyauto::ObjectType::OBJECT_TYPE_TRICYCLE ||
      obstacle.type() == iflyauto::ObjectType::OBJECT_TYPE_CYCLE_RIDING ||
      obstacle.type() == iflyauto::ObjectType::OBJECT_TYPE_MOTORCYCLE_RIDING ||
      obstacle.type() == iflyauto::ObjectType::OBJECT_TYPE_TRICYCLE_RIDING ||
      obstacle.type() == iflyauto::ObjectType::OBJECT_TYPE_ANIMAL;
  if (obstacle.is_static() || is_vru) {
    // 如果静态障碍物、VRU被赋予了FOLLOW，则整个5s都为FOLLOW
    spatio_temporal_follow_obstacle_info_[id].follow_time_windows.push_back(
        {LatObstacleDecisionType::FOLLOW, TimeInterval{0.0, 5.0}});
  } else {
    // 动态障碍物暂且分为两个时间段,即follow_time_windows的最大size为2
    // 寻找分界点，即follow和非follow的转折点
    double intrusion_distance = DBL_MAX;
    double delta_t = 0.5; // 时间分辨率
    double total_prediction_t = 5;
    double follow_end_time = 5;
    bool ok = false;
    double start_t = 0;
    double end_t = 0;
    for (double i = 0; i < total_prediction_t; i += delta_t) {
      // 连续3帧判断不为follow，则释放follow
      Polygon2d obstacle_sl_polygon;
      double min_l = std::numeric_limits<double>::max();
      double max_l = std::numeric_limits<double>::lowest();
      ok = reference_path_ptr_->get_polygon_at_time(
          id, false, int(i * 10), obstacle_sl_polygon);
      if (!ok) {
        continue;
      }
      for (auto &pt : obstacle_sl_polygon.points()) {
        min_l = std::min(min_l, pt.y());
        max_l = std::max(max_l, pt.y());
      }
      if (max_l < 0) {
        intrusion_distance = lane_width_ * 0.5 - std::fabs(max_l);
      } else if (min_l > 0) {
        intrusion_distance = lane_width_ * 0.5 - min_l;
      }
      if (intrusion_distance < lead_d_path_thr) {
        // 不满足follow条件
        end_t = i + 1;
      } else {
        start_t = i;
        end_t = i;
      }
      if (end_t - start_t >= 3 * delta_t) {
        follow_end_time = start_t;
        break;
      }
    }
    if (follow_end_time >= total_prediction_t) {
      spatio_temporal_follow_obstacle_info_[id].follow_time_windows.push_back(
          {LatObstacleDecisionType::FOLLOW, TimeInterval{0.0, 5.0}});
    } else {
      spatio_temporal_follow_obstacle_info_[id].follow_time_windows.push_back(
          {LatObstacleDecisionType::FOLLOW, TimeInterval{0.0, follow_end_time}});
      spatio_temporal_follow_obstacle_info_[id].follow_time_windows.push_back(
          {LatObstacleDecisionType::NOT_SET, TimeInterval{follow_end_time, 5}});
    }
  }
}

bool SccLateralObstacleDecider::CalculateCutInAndCross(
    const FrenetObstacle &frenet_obstacle) {
  const int obstacle_id = frenet_obstacle.id();
  LateralObstacleHistoryInfo &history =
      lateral_obstacle_history_info_[obstacle_id];
  double oversize_veh_addition_buffer = config_.oversize_veh_addition_buffer;
  const auto &obstacle = *frenet_obstacle.obstacle();
  if (!frenet_obstacle.obstacle()->is_static() &&
      !(frenet_obstacle.d_s_rel() <= 0 && in_intersection_) &&
      frenet_obstacle.obstacle()->trajectory_valid()) {
    double lat_safety_buffer = config_.lat_safety_buffer;
    // addition buffer for VRU
    if (frenet_obstacle.obstacle()->is_VRU()) {
      lat_safety_buffer += 0.2;
    }
    // decre lat_safety_buffer in narrow lane
    std::array<double, 3> x_lat_buffer{3.2, 3.5, 3.8};
    std::array<double, 3> f_lat_buffer{0.3, 0.15, 0};
    double decre_buffer_for_lane_width =
        interp(lane_width_, x_lat_buffer, f_lat_buffer);
    lat_safety_buffer -= decre_buffer_for_lane_width;

    // addition buffer for oversize vehicle
    if (obstacle.is_oversize_vehicle()) {
      lat_safety_buffer += oversize_veh_addition_buffer;
    }

    // 减去大车后视镜
    if (frenet_obstacle.obstacle()->is_oversize_vehicle()) {
      lat_safety_buffer -= 0.2;
    }

    // hack 感知大车减去后视镜宽度
    if (IsTruck(frenet_obstacle)) {
      lat_safety_buffer += config_.extra_truck_lat_buffer;
    }

    // 对向车减小0.2m
    if (obstacle.is_reverse()) {
      lat_safety_buffer -= 0.2;
    }

    // 滞回
    if (history.cut_in_or_cross) {
      lat_safety_buffer += 0.1;
    }
    auto &frenet_coord = reference_path_ptr_->get_frenet_coord();
    double right_distance_to_center_line = 0.5 * lane_width_;
    double left_distance_to_center_line = 0.5 * lane_width_;
    ReferencePathPoint reference_path_point;
    if (reference_path_ptr_->get_reference_point_by_lon(
            reference_path_ptr_->get_frenet_ego_state().s(),
            reference_path_point)) {
      right_distance_to_center_line =
          reference_path_point.distance_to_right_lane_border;
      left_distance_to_center_line =
          reference_path_point.distance_to_left_lane_border;
    }
    double right_l_Threshold =
        ego_width_ + lat_safety_buffer - left_distance_to_center_line;
    double left_l_Threshold =
        ego_width_ + lat_safety_buffer - right_distance_to_center_line;
    constexpr double kAdditionL = 0.15;
    constexpr double kVelLThreshold = 0.15;
    std::array<uint8_t, 6> timestamps{0, 1, 2, 3, 4, 5};
    bool ok = false;
    bool is_cut_in = false;
    bool is_potential_cut_in = false;
    for (auto &i : timestamps) {
      Polygon2d obstacle_sl_polygon;
      ok = reference_path_ptr_->get_polygon_at_time(
          obstacle_id, false, int(i * 10), obstacle_sl_polygon);
      if (ok) {
        for (auto &pt : obstacle_sl_polygon.points()) {
          if (obstacle.is_reverse()) {
            // 匀速递推
            double ego_front_s = ego_head_s_ + i * ego_v_s_;
            // 使用历史轨迹
            // size_t index = static_cast<size_t>(i / config_.delta_t);
            // if (index < plan_history_traj_.size()) {
            //   ego_front_s = plan_history_traj_[index].s +
            //   ego_rear_axis_to_front_edge_;
            // }
            if (pt.x() <= ego_front_s) {
              break;
            }
          }
          ReferencePathPoint reference_path_point;
          if (reference_path_ptr_->get_reference_point_by_lon(
                  pt.x(), reference_path_point)) {
            right_distance_to_center_line =
                reference_path_point.distance_to_right_lane_border;
            left_distance_to_center_line =
                reference_path_point.distance_to_left_lane_border;
          }
          right_l_Threshold =
              ego_width_ + lat_safety_buffer - left_distance_to_center_line;
          left_l_Threshold =
              ego_width_ + lat_safety_buffer - right_distance_to_center_line;
          if (((frenet_obstacle.frenet_l() > 0 &&
                pt.y() < left_l_Threshold - kAdditionL) ||
               (frenet_obstacle.frenet_l() < 0 &&
                pt.y() > -(right_l_Threshold - kAdditionL))) &&
              !is_cut_in) {
            is_cut_in = true;
            break;
          } else if ((std::abs(frenet_obstacle.frenet_velocity_l()) <
                          kVelLThreshold &&
                      ((frenet_obstacle.frenet_l() > 0 &&
                        pt.y() < left_l_Threshold) ||
                       (frenet_obstacle.frenet_l() < 0 &&
                        pt.y() > -right_l_Threshold))) &&
                     !is_potential_cut_in) {
            is_potential_cut_in = true;
          } else {
            continue;
          }
        }
      }
      if (is_cut_in) {
        break;
      }
    }

    if (is_cut_in) {
      history.cut_in_or_cross = true;
      history.cut_in_or_cross_count = 5;
      return true;
    } else if (is_potential_cut_in) {
      history.cut_in_or_cross_count += 1;
      history.cut_in_or_cross_count =
          std::min(history.cut_in_or_cross_count, 5);
    } else {
      history.cut_in_or_cross_count -= 1;
      history.cut_in_or_cross_count =
          std::max(history.cut_in_or_cross_count, 0);
    }

    if (history.cut_in_or_cross_count > 2) {
      history.cut_in_or_cross = true;
      return true;
    } else {
      history.cut_in_or_cross = false;
      return false;
    }
  } else {
    history.cut_in_or_cross_count -= 1;
    history.cut_in_or_cross_count = std::max(history.cut_in_or_cross_count, 0);
    if (history.cut_in_or_cross_count > 2) {
      history.cut_in_or_cross = true;
      return true;
    } else {
      history.cut_in_or_cross = false;
      return false;
    }
  }
}

void SccLateralObstacleDecider::ConstructUniformPlanHistoryTraj() {
  const auto plan_history_traj = session_->mutable_planning_context()
                                     ->lateral_obstacle_decider_output()
                                     .plan_history_traj;
  const auto is_plan_history_traj_valid =
      session_->mutable_planning_context()
          ->lateral_obstacle_decider_output()
          .is_plan_history_traj_valid;
  auto &uniform_plan_history_traj =
      session_->mutable_planning_context()
          ->mutable_lateral_obstacle_decider_output()
          .uniform_plan_history_traj;
  auto &is_uniform_plan_history_traj_valid =
      session_->mutable_planning_context()
          ->mutable_lateral_obstacle_decider_output()
          .is_uniform_plan_history_traj_valid;
  if (is_plan_history_traj_valid) {
    // 把s信息变成匀速
    if (!plan_history_traj.empty()) {
      is_uniform_plan_history_traj_valid = true;
      const double delta_s =
          (plan_history_traj.back().s - plan_history_traj.front().s) /
          (plan_history_traj.back().t - plan_history_traj.front().t);
      for (size_t i = 0; i < plan_history_traj.size(); i++) {
        TrajectoryPoint pt = plan_history_traj[i];
        pt.s =
            plan_history_traj[0].s + delta_s * (pt.t - plan_history_traj[0].t);
        uniform_plan_history_traj.emplace_back(std::move(pt));
      }
    } else {
      is_uniform_plan_history_traj_valid = false;
    }
  } else {
    is_uniform_plan_history_traj_valid = false;
  }
}

void SccLateralObstacleDecider::UpdateIntersection() {
  const auto intersection_state = session_->environmental_model()
                                      .get_virtual_lane_manager()
                                      ->GetIntersectionState();
  const double distance_to_stopline = session_->environmental_model()
                                          .get_virtual_lane_manager()
                                          ->GetEgoDistanceToStopline();
  bool current_intersection_state =
      intersection_state == common::IntersectionState::IN_INTERSECTION ||
      intersection_state == common::IntersectionState::OFF_INTERSECTION ||
      (intersection_state == common::IntersectionState::APPROACH_INTERSECTION &&
       (distance_to_stopline > 500 || distance_to_stopline < 0));
  if (current_intersection_state) {
    intersection_count_ = 2;
  } else {
    intersection_count_ = std::max(intersection_count_ - 1, 0);
  }
  in_intersection_ = intersection_count_ > 0;
}

void SccLateralObstacleDecider::UpdateLaneBorrowDirection() {
  left_borrow_ = true;
  right_borrow_ = true;
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto current_lane_ptr = virtual_lane_manager->get_current_lane();
  const auto left_lane_ptr = virtual_lane_manager->get_left_lane();
  const auto right_lane_ptr = virtual_lane_manager->get_right_lane();
  double lane_line_length = 0.0;
  const auto &left_lane_boundarys = current_lane_ptr->get_left_lane_boundary();
  const auto &right_lane_boundarys =
      current_lane_ptr->get_right_lane_boundary();
  const auto ego_frenet_boundary = session_->environmental_model()
                                       .get_reference_path_manager()
                                       ->get_reference_path_by_current_lane()
                                       ->get_ego_frenet_boundary();
  iflyauto::LaneBoundaryType left_lane_boundary_type;
  iflyauto::LaneBoundaryType right_lane_boundary_type;
  // # Accumulate lane segment lengths.
  // Record current segment type and break loop when exceeding vehicle
  // wheelbase.
  const auto &lane_points = current_lane_ptr->lane_points();
  for (int i = 0; i < lane_points.size(); i++) {
    lane_line_length = lane_points[i].s;
    if (lane_line_length > ego_frenet_boundary.s_end) {
      left_lane_boundary_type = lane_points[i].left_lane_border_type;
      right_lane_boundary_type = lane_points[i].right_lane_border_type;
      break;
    }
  }
  // If the lane marking is not left dashed/right solid or double dashed, return
  // False.
  if (left_lane_boundary_type != iflyauto::LaneBoundaryType_MARKING_DASHED &&
      left_lane_boundary_type !=
          iflyauto::LaneBoundaryType_MARKING_DECELERATION_DASHED &&
      left_lane_boundary_type !=
          iflyauto::LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED &&
      left_lane_boundary_type !=
          iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED) {
    left_borrow_ = false;
  }
  if (left_lane_ptr == nullptr) {
    left_borrow_ = false;
  }

  // todo: if left lane is reverse, then left_boorow is false
  if (right_lane_boundary_type != iflyauto::LaneBoundaryType_MARKING_DASHED &&
      right_lane_boundary_type !=
          iflyauto::LaneBoundaryType_MARKING_DECELERATION_DASHED &&
      right_lane_boundary_type !=
          iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED) {
    right_borrow_ = false;
  }
  if (right_lane_ptr == nullptr) {
    right_borrow_ = false;
  }
}

void SccLateralObstacleDecider::CheckObstaclesIsReverse() {
  auto obstacle_manager =
      session_->mutable_environmental_model()->mutable_obstacle_manager();
  const double kMaxHeadingDiff = 2.3;
  auto frenet_obstacles = reference_path_ptr_->get_obstacles();
  for (auto &frenet_obstacle : frenet_obstacles) {
    bool is_reverse = std::fabs(NormalizeAngle(
                          frenet_obstacle->obstacle()->velocity_angle() -
                          reference_path_ptr_->get_frenet_coord()
                              ->GetPathPointByS(frenet_obstacle->frenet_s())
                              .theta())) > kMaxHeadingDiff;
    auto obstacle = obstacle_manager->find_obstacle(frenet_obstacle->id());
    if (obstacle != nullptr) {
      obstacle->set_is_reverse(is_reverse);
    }
  }
}

bool SccLateralObstacleDecider::ARAStar() {
  // 默认关闭
  return true;
  auto &hybrid_ara_result = session_->mutable_planning_context()
                                ->mutable_lateral_obstacle_decider_output()
                                .hybrid_ara_result;
  hybrid_ara_result.Clear();
  bool find_path = hybrid_ara_star_->Plan(output_, hybrid_ara_result, search_result_);

  // log
  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto hybrid_ara_path =
      planning_debug_data->mutable_hybrid_ara_info()->mutable_hybrid_ara_path();
  auto hybrid_ara_path_cost = planning_debug_data->mutable_hybrid_ara_info()
                                  ->mutable_hybrid_ara_path_cost();
  hybrid_ara_path->Clear();
  hybrid_ara_path_cost->Clear();
  for (const auto x : hybrid_ara_result.x) {
    hybrid_ara_path->add_x(x);
  }
  for (const auto y : hybrid_ara_result.y) {
    hybrid_ara_path->add_y(y);
  }
  for (const auto phi : hybrid_ara_result.phi) {
    hybrid_ara_path->add_phi(phi);
  }
  for (const auto s : hybrid_ara_result.s) {
    hybrid_ara_path->add_s(s);
  }
  for (const auto l : hybrid_ara_result.l) {
    hybrid_ara_path->add_l(l);
  }

  return (find_path && hybrid_ara_result.Valid());
}

void SccLateralObstacleDecider::LateralObstacleDeciderOutput() {
  auto &lateral_obstacle_decider_output =
      session_->mutable_planning_context()
          ->mutable_lateral_obstacle_decider_output();
  lateral_obstacle_decider_output.left_borrow = left_borrow_;
  lateral_obstacle_decider_output.right_borrow = right_borrow_;
  lateral_obstacle_decider_output.in_intersection = in_intersection_;
  lateral_obstacle_decider_output.lateral_obstacle_history_info =
      lateral_obstacle_history_info_;
  lateral_obstacle_decider_output.obstacles_id_behind_ego =
      obstacles_id_behind_ego_;
  lateral_obstacle_decider_output.obstacle_intrusion_distance_thr =
      obstacle_intrusion_distance_thr_;
  lateral_obstacle_decider_output.follow_obstacle_info = follow_obstacle_info_;
  lateral_obstacle_decider_output.lat_obstacle_decision = output_;
  lateral_obstacle_decider_output.spatio_temporal_follow_obstacle_info = spatio_temporal_follow_obstacle_info_;
}

void SccLateralObstacleDecider::Log() {
#ifdef ENABLE_PROTO_LOG
  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto environment_model_debug_info =
      planning_debug_data->mutable_environment_model_info();
  environment_model_debug_info->clear_obstacle();

  const auto &obstacle_map = reference_path_ptr_->get_obstacles_map();
  for (const auto & [ obstacle_id, decision ] : output_) {
    auto it = obstacle_map.find(obstacle_id);
    if (it == obstacle_map.end()) {
      continue;
    }

    const auto &frenet_obstacle = it->second;
    planning::common::Obstacle *obstacle_log =
        environment_model_debug_info->add_obstacle();
    obstacle_log->set_id(frenet_obstacle->id());
    obstacle_log->set_type(frenet_obstacle->type());
    obstacle_log->set_s(frenet_obstacle->frenet_s());
    obstacle_log->set_l(frenet_obstacle->frenet_l());
    obstacle_log->set_s_to_ego(frenet_obstacle->d_s_rel());
    obstacle_log->set_max_l_to_ref(frenet_obstacle->d_max_cpath());
    obstacle_log->set_min_l_to_ref(frenet_obstacle->d_min_cpath());
    // obstacle_log->set_s_with_min_l(frenet_obstacle->s_min);
    // obstacle_log->set_s_with_max_l(frenet_obstacle->s_max);
    // obstacle_log->set_nearest_l_to_desire_path(frenet_obstacle->d_path);
    // obstacle_log->set_nearest_l_to_ego(frenet_obstacle->d_path_self);
    obstacle_log->set_vs_lat_relative(
        frenet_obstacle->frenet_relative_velocity_l());
    obstacle_log->set_vs_lon_relative(
        frenet_obstacle->frenet_relative_velocity_s());
    obstacle_log->set_vs_lon(frenet_obstacle->frenet_velocity_s());
    // obstacle_log->set_nearest_y_to_desired_path(frenet_obstacle->y_rel());
    // obstacle_log->set_is_accident_car(frenet_obstacle->is_accident_car());
    // obstacle_log->set_is_accident_cnt(frenet_obstacle->is_accident_cnt());
    // obstacle_log->set_is_avoid_car(frenet_obstacle.);
    // obstacle_log->set_is_lane_lead_obstacle(frenet_obstacle->is_lead());
    // obstacle_log->set_current_lead_obstacle_to_ego(frenet_obstacle->is_temp_lead());
    obstacle_log->set_is_static(frenet_obstacle->is_static());
    obstacle_log->set_lat_decision(static_cast<uint32_t>(decision));
    // obstacle_log->set_cutin_p(frenet_obstacle.cutinp);
  }
#endif

  std::vector<double> avd_car_id;
  std::vector<double> maintain_avoid;
  std::vector<double> emergency_avoid;
  std::vector<double> lon_overtake_avoid;
  avd_car_id.reserve(10);
  maintain_avoid.reserve(10);
  emergency_avoid.reserve(10);
  lon_overtake_avoid.reserve(10);
  for (const auto frenet_obs : reference_path_ptr_->get_obstacles()) {
    LateralObstacleHistoryInfo &history =
        lateral_obstacle_history_info_[frenet_obs->id()];
    if (history.is_avd_car) {
      avd_car_id.emplace_back(frenet_obs->id());
    }
    if (history.maintain_avoid) {
      maintain_avoid.emplace_back(frenet_obs->id());
    }

    if (history.emergency_avoid) {
      emergency_avoid.emplace_back(frenet_obs->id());
    }
    if (history.lon_overtake_avoid) {
      lon_overtake_avoid.emplace_back(frenet_obs->id());
    }
  }

  JSON_DEBUG_VECTOR("emergency_avoid_obstacle_ids", emergency_avoid, 0);
  JSON_DEBUG_VECTOR("lon_overtake_avoid", lon_overtake_avoid, 0);
  JSON_DEBUG_VECTOR("maintain_avoid", maintain_avoid, 0);
  JSON_DEBUG_VECTOR("avoid_car_id", avd_car_id, 0);
  JSON_DEBUG_VALUE("can_left_borrow", left_borrow_);
  JSON_DEBUG_VALUE("can_right_borrow", right_borrow_);
}

bool SccLateralObstacleDecider::CheckSideObstacle(
    const FrenetObstacle &frenet_obstacle) {
  auto obstacle_manager =
      session_->mutable_environmental_model()->mutable_obstacle_manager();
  const double KOverlapSThrt = 0.3;
  const double KOverlapLThrt = 0.5;
  const auto ego_frenet_state = reference_path_ptr_->get_frenet_ego_state();
  const auto ego_s_start = ego_frenet_state.boundary().s_start;
  const auto ego_s_end = ego_frenet_state.boundary().s_start;
  const auto obstacle_boundary_s_start =
      frenet_obstacle.frenet_obstacle_boundary().s_start;
  const auto obstacle_boundary_s_end =
      frenet_obstacle.frenet_obstacle_boundary().s_end;
  const int obstacle_id = frenet_obstacle.id();
  if ((obstacle_boundary_s_end - ego_s_start > 0 &&
       obstacle_boundary_s_start - ego_s_start < 0)) {
    // 障碍物在自车前方先不考虑
    // (ego_s_end - obstacle_boundary_s_start > 0 &&
    // ego_s_end - obstacle_boundary_s_start <= KOverlapSThrt)
    Polygon2d obstacle_sl_polygon;
    const auto ok = reference_path_ptr_->get_polygon_at_time(
        obstacle_id, false, 0, obstacle_sl_polygon);
    const auto ego_sl_polygon = ego_frenet_state.polygon();
    if (ok && ego_sl_polygon.is_convex()) {
      Polygon2d care_overlap_polygon;
      bool b_overlap_with_care = false;
      b_overlap_with_care = obstacle_sl_polygon.ComputeOverlap(
          ego_sl_polygon, &care_overlap_polygon);
      if (b_overlap_with_care) {
        if (care_overlap_polygon.max_y() - care_overlap_polygon.min_y() >
            KOverlapLThrt) {
          auto obstacle = obstacle_manager->find_obstacle(frenet_obstacle.id());
          if (obstacle != nullptr) {
            obstacle->set_is_normal(false);
          }
          return false;
        }
      }
    }
  }
  return true;
}

bool SccLateralObstacleDecider::IsTruck(const FrenetObstacle &frenet_obstacle) {
  return (frenet_obstacle.type() == iflyauto::ObjectType::OBJECT_TYPE_BUS ||
          (frenet_obstacle.type() == iflyauto::ObjectType::OBJECT_TYPE_TRUCK &&
           frenet_obstacle.length() > 6));
}

void SccLateralObstacleDecider::IsPotentialFollowingObstacle(
    const FrenetObstacle &frenet_obstacle, bool is_in_lane_change_execution_scene) {
  // 根据侵入距离，计算潜在的跟车目标
  const Obstacle &obstacle = *frenet_obstacle.obstacle();
  LateralObstacleHistoryInfo &history =
      lateral_obstacle_history_info_[obstacle.id()];
  FollowObstacleInfo &follow_info = follow_obstacle_info_[obstacle.id()];
  // calculate info of obstacle
  int id = obstacle.id();
  iflyauto::ObjectType type = obstacle.type();
  double s = frenet_obstacle.frenet_s();
  double l = frenet_obstacle.frenet_l();
  double v_s = frenet_obstacle.frenet_velocity_s();
  double v_l = frenet_obstacle.frenet_velocity_l();
  double v_lat = frenet_obstacle.frenet_velocity_lateral();  // 根据历史差分
  double v_s_rel = frenet_obstacle.frenet_relative_velocity_s();
  double d_s_rel = frenet_obstacle.d_s_rel();
  double d_min_cpath = frenet_obstacle.d_min_cpath();
  double d_max_cpath = frenet_obstacle.d_max_cpath();

  double gap = (history.last_recv_time == 0.0)
                   ? kPlanningCycleTime
                   : (obstacle.timestamp() - history.last_recv_time);
  int count = (int)((gap + 0.01) / kPlanningCycleTime);
  double lateral_distacle_to_lane = 0.25;
  if (obstacle.is_static()) {
    // 先过滤静态障碍物，因为静态障碍物的不确定性较低
    follow_info.follow_confidence = 0;
    follow_info.is_need_folow = false;
    return;
  }
  if (is_in_lane_change_execution_scene &&
      frenet_obstacle.frenet_obstacle_boundary().s_start <
          lc_gap_info_.gap_front_s) {
    // 在变道状态，依据gap计算follow
    follow_info.follow_confidence = 0;
    follow_info.is_need_folow = false;
  } else {
    // 计算侵入距离
    double intrusion_distance = DBL_MAX;
    if (d_max_cpath < 0) {
      intrusion_distance = lane_width_ * 0.5 - std::fabs(d_max_cpath);
    } else if (d_min_cpath > 0) {
      intrusion_distance = lane_width_ * 0.5 - d_min_cpath;
    }
    double base_safe_intrusoin_for_dynamic =
        config_.base_safe_intrusoin_for_dynamic;
    double base_safe_intrusoin_for_static =
        config_.base_safe_intrusoin_for_static;
    double extra_buffer_for_truck = config_.extra_buffer_for_truck;
    double extra_buffer_for_vru = config_.extra_buffer_for_vru;
    double follow_hysteresis = config_.follow_hysteresis;
    double follow_confidence_cnt = 10 * kPlanningCycleTime;
    double follow_confidence_thr = 0.1;
    double lead_d_path_thr = base_safe_intrusoin_for_dynamic;
    // if (obstacle.is_static()) {
    //   lead_d_path_thr = base_safe_intrusoin_for_static;
    // }
    // 障碍物类型
    if (IsTruck(frenet_obstacle)) {
      lead_d_path_thr += extra_buffer_for_truck;
    } else if (obstacle.is_VRU()) {
      lead_d_path_thr += extra_buffer_for_vru;
    }
    // hysteresis
    if (follow_info.is_need_folow) {
      lead_d_path_thr = lead_d_path_thr * follow_hysteresis;
    }
    // 车道宽的影响,同样倾入距离的情况下，窄车道更容易跟车
    double lane_width_actor =
        interp(lane_width_, config_.lane_width_bp, config_.lane_width_factor);
    lead_d_path_thr = lead_d_path_thr * lane_width_actor;

    // 距离自车越远，置信度越高
    follow_confidence_thr = interp(d_s_rel, config_.distacle_to_ego_bp,
                                   config_.distacle_to_ego_bp_factor);

    // 自车速度的相关因素，自车速度越高，先减速然后跟车

    if (intrusion_distance >= lead_d_path_thr) {
      follow_info.follow_confidence =
          std::fmin(follow_info.follow_confidence + gap, follow_confidence_cnt);
    } else {
      // 针对cut_out或者横穿的障碍物，即远离自车道的障碍物，v_lat > 0,
      // 衰减需要加快
      std::array<double, 5> x_cut_factor{0.2, 0.3, 0.4, 0.6, 0.8};
      std::array<double, 5> f_cut_factor{0, 0, 2, 6, 11};
      double cut_factor = interp(v_lat, x_cut_factor, f_cut_factor);
      follow_info.follow_confidence = std::fmax(
          follow_info.follow_confidence - cut_factor * kPlanningCycleTime, 0.0);

      // 正常衰减
      follow_info.follow_confidence = std::fmax(
          follow_info.follow_confidence - 2 * count * kPlanningCycleTime, 0.0);

      // 针对离自车道较远的障碍物，不计数
      // 防止障碍物横向跳动较大，异常计数，导致纵向不提速或者减速现象
      if (intrusion_distance < -lateral_distacle_to_lane) {
        // 障碍物在车道线0.25m之外
        follow_info.follow_confidence = 0;
      }
    }
    follow_info.is_need_folow =
        follow_info.follow_confidence >= follow_confidence_thr;
  }
}

void SccLateralObstacleDecider::CalLaneChangeGapInfo(LcGapInfo &lc_gap_info) {
  const auto &target_state = session_->planning_context()
                                 .lane_change_decider_output()
                                 .coarse_planning_info.target_state;
  const auto &dynamic_world =
      session_->environmental_model().get_dynamic_world();

  bool is_in_lane_change_execution_scene = target_state == kLaneChangeExecution;
  if (!is_in_lane_change_execution_scene) {
    return;
  }

  const auto &obstacle_map = reference_path_ptr_->get_obstacles_map();
  const auto &lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto gap_front_node_id =
      lane_change_decider_output.lc_gap_info.front_node_id;
  const auto gap_rear_node_id =
      lane_change_decider_output.lc_gap_info.rear_node_id;
  int32_t gap_front_agent_id = -1;
  if (dynamic_world->GetNode(gap_front_node_id)) {
    gap_front_agent_id =
        dynamic_world->GetNode(gap_front_node_id)->node_agent_id();
  }
  int32_t gap_rear_agent_id = -1;
  if (dynamic_world->GetNode(gap_rear_node_id)) {
    gap_rear_agent_id =
        dynamic_world->GetNode(gap_rear_node_id)->node_agent_id();
  }

  if (obstacle_map.find(gap_front_agent_id) != obstacle_map.end()) {
    lc_gap_info.gap_front_id = gap_front_agent_id;
    lc_gap_info.gap_front_s =
        obstacle_map.at(gap_front_agent_id)->frenet_obstacle_boundary().s_start;
  }
  if (obstacle_map.find(gap_rear_agent_id) != obstacle_map.end()) {
    lc_gap_info.gap_rear_id = gap_rear_agent_id;
    lc_gap_info.gap_rear_s =
        obstacle_map.at(gap_rear_agent_id)->frenet_obstacle_boundary().s_start;
  }
}

void SccLateralObstacleDecider::ClearHistoryInfo() {
  const auto &obstacle_map = reference_path_ptr_->get_obstacles_map();
  for (auto it = follow_obstacle_info_.begin();
       it != follow_obstacle_info_.end();) {
    if (obstacle_map.find(it->first) == obstacle_map.end()) {
      // 当前帧不存在该障碍物
      it = follow_obstacle_info_.erase(it);
    } else {
      ++it;
    }
  }
  for (auto it = lateral_obstacle_history_info_.begin();
       it != lateral_obstacle_history_info_.end();) {
    if (obstacle_map.find(it->first) == obstacle_map.end()) {
      // 当前帧不存在该障碍物
      it = lateral_obstacle_history_info_.erase(it);
    } else {
      ++it;
    }
  }
  for (auto it = obstacle_interaction_map_.begin();
       it != obstacle_interaction_map_.end();) {
    if (obstacle_map.find(it->first) == obstacle_map.end()) {
      // 当前帧不存在该障碍物
      it = obstacle_interaction_map_.erase(it);
    } else {
      ++it;
    }
  }
}

bool SccLateralObstacleDecider::IsCutInIgnore(
    const FrenetObstacle& frenet_obstacle, bool is_in_lane_change_execution_scene) {
  LateralObstacleHistoryInfo& history =
      lateral_obstacle_history_info_[frenet_obstacle.id()];
  bool is_in_lane_change_gap =
      is_in_lane_change_execution_scene && frenet_obstacle.frenet_obstacle_boundary().s_start <
                            lc_gap_info_.gap_front_s;
  if (!is_in_lane_change_gap && history.cut_in_or_cross &&
      !frenet_obstacle.obstacle()->is_static()) {
    return true;
  }
  return false;
}

}  // namespace planning