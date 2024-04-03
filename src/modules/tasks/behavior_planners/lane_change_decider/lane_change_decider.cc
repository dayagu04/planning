#include "lane_change_decider.h"

#include <iterator>
#include <limits>
#include <numeric>

#include "../../common/planning_gflags.h"
#include "debug_info_log.h"
#include "ego_state_manager.h"
#include "lateral_behavior_object_selector.h"
#include "obstacle_manager.h"
#include "planning_context.h"
#include "reference_path.h"
#include "reference_path_manager.h"
#include "spline_projection.h"
#include "tasks/behavior_planners/vision_only_lateral_behavior_planner/vision_lateral_behavior_planner.h"
#include "utils/pose2d_utils.h"
#include "vehicle_config_context.h"

namespace planning {

namespace {
constexpr double kEps = 1e-6;
}

LaneChangeDecider::LaneChangeDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "LaneChangeDecider";
  config_ = config_builder->cast<ScenarioStateMachineConfig>();
  std::shared_ptr<VirtualLaneManager> virtual_lane_mgr =
      // session_->mutable_planning_context()->virtual_lane_manager();
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  lc_lane_mgr_ =
      std::make_shared<LaneChangeLaneManager>(virtual_lane_mgr, session_);
  lc_req_mgr_ = std::make_shared<LaneChangeRequestManager>(
      session_, config_builder, virtual_lane_mgr, lc_lane_mgr_);

  object_selector_ = std::make_shared<ObjectSelector>(config_builder, session_);

  Init();
}

void LaneChangeDecider::Init() {
  reset_state_machine();
  lc_lane_mgr_->reset_lc_lanes();
}

bool LaneChangeDecider::Execute() {
  const bool active = session_->environmental_model().GetVehicleDbwStatus();
  const auto &function_info = session_->environmental_model().function_info();

  get_lane_change_lane_manager()->upload_fix_lane_virtual_id();
  UpdateFixLaneVirtualId();

  if (!session_->planning_context().last_planning_success()) {
    reset_state_machine();
  }

  update_scenario();  // cruise or low speed or ...

  if (!active ||
      function_info.function_mode() == common::DrivingFunctionInfo::ACC) {
    lc_req_mgr_->FinishRequest();
    map_turn_signal_ = NO_CHANGE;
    reset_state_machine();
  }

  if (!UpdateObjectSelector(active)) {
    return false;
  }

  if (active) {
    LOG_DEBUG("[scenario_state_machine] active\n");
    if (scenario_ == SCENARIO_CRUISE) {
      // update lc_req_mgr_
      if (!session_->is_hpp_scene()) {
        if (!lc_req_mgr_->Update(object_selector_,
                                 transition_context_.target_state,
                                 session_->environmental_model().IsOnRoute())) {
          return false;
        }
      }

      gen_map_turn_signal();
      update_state_machine();
      post_process();
    } else {
      return false;
    }
  } else {
    LOG_DEBUG("[scenario_state_machine] not active\n");
    if (scenario_ == SCENARIO_CRUISE) {
      gen_map_turn_signal();
      update_state_machine();
      post_process();
    } else {
      return false;
    }
    // post_process();
  }

  if (transition_context_.target_state != transition_context_.source_state) {
    if (transition_context_.target_state == ROAD_NONE) {
      clear_lc_variables();
    } else if ((transition_context_.target_state == ROAD_LC_LCHANGE &&
                (transition_context_.source_state == ROAD_LC_LWAIT ||
                 transition_context_.source_state == ROAD_LC_LBACK)) ||
               (transition_context_.target_state == ROAD_LC_RCHANGE &&
                (transition_context_.source_state == ROAD_LC_RWAIT ||
                 transition_context_.source_state == ROAD_LC_RBACK))) {
      update_start_move_dist_lane();
    }
  } else if (transition_context_.direction == NO_CHANGE) {
    if (transition_context_.target_state == ROAD_LC_LCHANGE) {
      transition_context_.direction = LEFT_CHANGE;
    } else if (transition_context_.target_state == ROAD_LC_RCHANGE) {
      transition_context_.direction = RIGHT_CHANGE;
    }
  }

  UpdateCoarsePlanningInfo();

  if (!CheckEgoPosition()) {
    return false;
  }

  UpdateStateMachineDebugInfo();

  UpdateAdInfo();

  return true;
}

void LaneChangeDecider::update_state_machine() {
  if (transition_context_.target_state >= ROAD_NONE &&
      transition_context_.target_state <= INTER_UT_NONE) {
    if (scenario_ == SCENARIO_CRUISE) {
      UpdateLaneChangeState();
    } else {
      LOG_ERROR("Scenario Not Implemented!");
      transition_context_.Reset();
      lc_lane_mgr_->reset_lc_lanes();
      map_turn_signal_ = NO_CHANGE;
    }
  } else if (transition_context_.target_state > INTER_UT_NONE) {
    LOG_ERROR("State Not Implemented!");
    transition_context_.Reset();
    lc_lane_mgr_->reset_lc_lanes();
    map_turn_signal_ = NO_CHANGE;
  }
}

void LaneChangeDecider::reset_state_machine() {
  transition_context_.Reset();
  lc_lane_mgr_->reset_lc_lanes();
  lc_req_mgr_->FinishRequest();
  clear_lc_variables();
}

void LaneChangeDecider::clear_lc_variables() {
  clear_lc_stage_info();
  lc_valid_cnt_ = 0;
  lb_back_cnt_ = 0;
  lc_invalid_track_.reset();
  lc_back_track_.reset();

  near_cars_target_.clear();
  near_cars_origin_.clear();
  must_change_lane_ = false;
  start_move_dist_lane_ = 0;

  not_accident_ = true;
  behavior_suspend_ = false;  // lateral suspend
  suspend_obs_.clear();
}

void LaneChangeDecider::clear_lc_stage_info() {
  lane_change_stage_info_.gap_valid = false;
  lane_change_stage_info_.gap_approached = false;
  lane_change_stage_info_.gap_insertable = false;
  lane_change_stage_info_.side_approach = false;
  lane_change_stage_info_.should_suspend = false;
  lane_change_stage_info_.lc_pause = false;
  lane_change_stage_info_.lc_pause_id = -1000;
  lane_change_stage_info_.should_premove = false;
  lane_change_stage_info_.lc_invalid_reason = "none";
  lane_change_stage_info_.tr_pause_dv = 0.0;
  lane_change_stage_info_.tr_pause_l = 0.0;
  lane_change_stage_info_.tr_pause_s = -100.0;
  lane_change_stage_info_.accident_back = false;
  lane_change_stage_info_.need_clear_lb_car = false;
  lane_change_stage_info_.accident_ahead = false;
  lane_change_stage_info_.close_to_accident = false;
  lane_change_stage_info_.lc_should_back = false;
  lane_change_stage_info_.lc_valid = false;
  lane_change_stage_info_.lc_back_reason = "none";
}

void LaneChangeDecider::update_scenario() { scenario_ = SCENARIO_CRUISE; }

void LaneChangeDecider::update_start_move_dist_lane() {
  int fix_lane_virtual_id = lc_lane_mgr_->fix_lane_virtual_id();
  auto fix_reference_path =
      session_->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(fix_lane_virtual_id);
  auto frenet_ego_state = fix_reference_path->get_frenet_ego_state();
  start_move_dist_lane_ = frenet_ego_state.l();
}

void LaneChangeDecider::post_process() {
  RequestType turn_signal_this_frame = (map_turn_signal_ == NO_CHANGE)
                                           ? lc_req_mgr_->turn_signal()
                                           : map_turn_signal_;
  turn_signal_this_frame = (turn_signal_this_frame == NO_CHANGE)
                               ? merge_split_turn_signal_
                               : turn_signal_this_frame;
  if (turn_signal_ == NO_CHANGE && turn_signal_this_frame != NO_CHANGE) {
    // turn_signal_on_time_ = IflyTime::Now_s();
    turn_signal_on_time_ = IflyTime::Now_s();
  }
  turn_signal_ = turn_signal_this_frame;
  session_->mutable_planning_context()->mutable_planning_result().turn_signal =
      turn_signal_;

  LOG_DEBUG(
      "[LaneChangeDecider] turn_signal_on_time_ %f map_turn_signal: %d, "
      "lc_turn_signal: %d, merge_split_turn_signal: %d, turn_signal_: %d \n",
      turn_signal_on_time_, (int)map_turn_signal_,
      (int)lc_req_mgr_->turn_signal(), (int)merge_split_turn_signal_,
      (int)turn_signal_);
}

void LaneChangeDecider::gen_map_turn_signal() {
  std::shared_ptr<VirtualLaneManager> map_info_mgr =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
}

void LaneChangeDecider::gen_merge_split_turn_signal() {}

bool LaneChangeDecider::GapAvailable(RequestType direction,
                                     std::vector<int> &overtake_obstacles,
                                     std::vector<int> &yield_obstacles) {
  // 1.获取所需车道: current lane, target lane
  std::shared_ptr<VirtualLane> target_lane = lc_lane_mgr_->tlane();
  auto origin_lane = lc_lane_mgr_->olane();
  auto &virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  if (origin_lane == nullptr) {
    origin_lane = virtual_lane_manager->get_current_lane();
  }
  if (target_lane == nullptr || target_lane->get_reference_path() == nullptr) {
    LOG_WARNING("[GapAvailable] target_lane is null \n");
    return false;
  }
  // 2.获取当前车道参考线 & 参考线关联障碍物
  auto reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  auto current_reference_path =
      reference_path_manager->get_reference_path_by_current_lane();
  auto &frenet_obstacles = current_reference_path->get_obstacles();
  auto &frenet_ego_state = current_reference_path->get_frenet_ego_state();
  bool b_gap_available(true);
  // 3.交互式换道需要判断最近的gap是否满足需求
  // 获取目标车道中的障碍物id序列
  auto tlane_lane_obstacles =
      target_lane->get_reference_path()->get_lane_obstacles_ids();
  for (auto &frenet_obstacle : frenet_obstacles) {
    if (std::count(tlane_lane_obstacles.begin(), tlane_lane_obstacles.end(),
                   frenet_obstacle->id())) {
      double ego_s = frenet_ego_state.s();
      double obstacle_s = frenet_obstacle->frenet_s();
      if ((obstacle_s > ego_s) &&
          !IsFollowBufferEnough(
              frenet_obstacle->frenet_obstacle_boundary().s_start,
              frenet_ego_state.boundary().s_end,
              frenet_obstacle->frenet_velocity_s(),
              frenet_ego_state.velocity_s())) {
        // 目标车道近距离前侧存在障碍物使自车无法换道
        b_gap_available = false;
        LOG_DEBUG(
            "gap_available is: FALSE, obstacle id: [%d], obstacle s: [%f], ego "
            "s: [%f] \n",
            frenet_obstacle->id(), obstacle_s, ego_s);
      }
      if ((obstacle_s < ego_s) &&
          !IsFollowBufferEnough(
              frenet_ego_state.boundary().s_start,
              frenet_obstacle->frenet_obstacle_boundary().s_end,
              frenet_ego_state.velocity_s(),
              frenet_obstacle->frenet_velocity_s())) {
        // 目标车道近距离后侧存在障碍物使自车无法换道
        b_gap_available = false;
      }
    }
  }

  // 4.排序障碍物: 由远至近
  auto tlane_obstacles =
      target_lane->get_reference_path()->get_lane_obstacles_ids();
  std::vector<std::shared_ptr<FrenetObstacle>> sorted_obstacles;
  for (auto &frenet_obstacle : frenet_obstacles) {
    if (std::count(tlane_obstacles.begin(), tlane_obstacles.end(),
                   frenet_obstacle->id()) > 0) {
      sorted_obstacles.emplace_back(frenet_obstacle);
    }
  }
  std::sort(sorted_obstacles.begin(), sorted_obstacles.end(),
            [&](const std::shared_ptr<FrenetObstacle> &o1,
                const std::shared_ptr<FrenetObstacle> &o2) {
              return o1->frenet_s() > o2->frenet_s();
            });

  // 5.构建gap
  std::vector<GapInfo> gap_infos;
  if (sorted_obstacles.size() > 0) {
    GapInfo first_gap{};
    first_gap.front_id = -1;
    first_gap.s_front = std::numeric_limits<double>::max();
    first_gap.v_front = std::numeric_limits<double>::max();
    first_gap.rear_id = sorted_obstacles[0]->id();
    first_gap.s_rear = sorted_obstacles[0]->frenet_obstacle_boundary().s_end;
    first_gap.v_rear = sorted_obstacles[0]->frenet_velocity_s();

    gap_infos.emplace_back(first_gap);
  }
  for (size_t i = 0; i < sorted_obstacles.size(); i++) {
    GapInfo gap{};
    gap.front_id = sorted_obstacles[i]->id();
    gap.s_front = sorted_obstacles[i]->frenet_obstacle_boundary().s_start;
    gap.v_front = sorted_obstacles[i]->frenet_velocity_s();

    double s_rear = -5.0;
    double v_rear = 0;
    int id_rear = -1;
    if ((i + 1) < sorted_obstacles.size()) {
      s_rear = sorted_obstacles[i + 1]->frenet_obstacle_boundary().s_end;
      v_rear = sorted_obstacles[i + 1]->frenet_velocity_s();
      id_rear = sorted_obstacles[i + 1]->id();
    }
    gap.rear_id = id_rear;
    gap.s_rear = s_rear;
    gap.v_rear = v_rear;

    gap_infos.emplace_back(gap);
  }

  // 6.Select gap
  double s_ego_rear = frenet_ego_state.boundary().s_start;
  double s_ego_front = frenet_ego_state.boundary().s_end;
  double v_ego = frenet_ego_state.velocity_s();
  static const double time_headway = 0.5;
  static const double care_time = 2.0;
  static const double acc = 1.0;

  for (auto &gap_info : gap_infos) {
    double sc_f = gap_info.s_front + gap_info.v_front * care_time -
                  std::min(gap_info.v_front * time_headway, 5.0);
    double sc_r = gap_info.s_rear + gap_info.v_rear * care_time +
                  std::min(gap_info.v_rear * time_headway, 5.0);
    double s_upper =
        s_ego_front + v_ego * care_time - 0.5 * acc * care_time * care_time;
    double s_lower =
        s_ego_rear + v_ego * care_time + 0.5 * acc * care_time * care_time;
    LOG_DEBUG(
        "[gap info] front id: [%d], rear id: [%d], sc_f: [%f], sc_r: [%f], "
        "s_upper: [%f],  s_lower: [%f] \n",
        gap_info.front_id, gap_info.rear_id, sc_f, sc_r, s_upper, s_lower);
    if ((s_upper < sc_f && s_lower > sc_r) ||
        gap_info.s_rear < s_ego_rear - 20.0 || gap_info.rear_id == -1) {
      if (gap_info.front_id != -1) {
        yield_obstacles.push_back(gap_info.front_id);
      }
      if (gap_info.rear_id != -1) {
        overtake_obstacles.push_back(gap_info.rear_id);
      }
      break;
    }
  }
  LOG_DEBUG("gap_available is:[%d] \n", b_gap_available);
  return b_gap_available;
}

bool LaneChangeDecider::IsFollowBufferEnough(const double front_s,
                                             const double behind_s,
                                             const double front_v,
                                             const double behind_v) {
  // TBD: use config
  double kMinSafeDistance = 2.0;
  double kMaxEndurableAcc = 2.0;
  if ((front_s - behind_s) > 30) {
    return true;
  }
  if (transition_context_.target_state == ROAD_LC_LCHANGE ||
      transition_context_.target_state == ROAD_LC_RCHANGE) {
    kMinSafeDistance = 2.0;
    kMaxEndurableAcc = 2.0;
  } else if (transition_context_.target_state == ROAD_NONE ||
             transition_context_.target_state == ROAD_LC_LWAIT ||
             transition_context_.target_state == ROAD_LC_RWAIT ||
             transition_context_.target_state == ROAD_LC_LBACK ||
             transition_context_.target_state == ROAD_LC_RBACK) {
    kMinSafeDistance = 3.0;
    kMaxEndurableAcc = 0.75;
  }
  double delta_v = std::max(behind_v - front_v, 0.0);
  double s_safe = std::min(0.5 * (behind_v + delta_v), kMinSafeDistance);
  if ((front_s - behind_s) < s_safe) {
    return false;
  }
  double slow_down_distance = std::pow(delta_v, 2.0) / 2.0 / kMaxEndurableAcc;
  return slow_down_distance < (front_s - behind_s);
}

void LaneChangeDecider::compute_lc_valid_info(RequestType direction) {
  assert(direction == LEFT_CHANGE || direction == RIGHT_CHANGE);
  auto &lateral_obstacle =
      session_->environmental_model().get_lateral_obstacle();
  auto &ego_state = session_->environmental_model().get_ego_state_manager();

  const auto &lateral_output =
      session_->planning_context().lateral_behavior_planner_output();
  const auto &vel_sequence = lateral_output.vel_sequence;

  std::shared_ptr<VirtualLane> target_lane = lc_lane_mgr_->tlane();
  auto origin_lane = lc_lane_mgr_->olane();
  auto fix_lane = lc_lane_mgr_->flane();
  auto &virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  if (origin_lane == nullptr) {
    origin_lane = virtual_lane_manager->get_current_lane();
  }
  // if (direction == LEFT_CHANGE) {
  //   target_lane = virtual_lane_manager->get_left_neighbor(origin_lane);
  // } else if (direction == RIGHT_CHANGE) {
  //   target_lane = virtual_lane_manager->get_right_neighbor(origin_lane);
  // }
  if (target_lane == nullptr) {
    return;
  }
  if (fix_lane == nullptr) {
    fix_lane = virtual_lane_manager->get_current_lane();
  }

  // int current_lane_virtual_id = session_->mutable_environmental_model()
  //                                   ->virtual_lane_manager()
  //                                   ->current_lane_virtual_id();
  std::cout << "compute_lc_valid_info: fix_lane_virtual_id: "
            << lc_lane_mgr_->fix_lane_virtual_id()
            << " fix_lane->get_virtual_id(): " << fix_lane->get_virtual_id()
            << std::endl;
  std::cout << "compute_lc_valid_info: target_lane_virtual_id: "
            << lc_lane_mgr_->target_lane_virtual_id()
            << " target_lane->get_virtual_id(): "
            << target_lane->get_virtual_id() << std::endl;
  std::cout << "compute_lc_valid_info: clane_virtual_id: "
            << virtual_lane_manager->current_lane_virtual_id() << std::endl;
  auto reference_path_manager =
      session_->mutable_environmental_model()->get_reference_path_manager();
  auto fix_reference_path = reference_path_manager->get_reference_path_by_lane(
      lc_lane_mgr_->fix_lane_virtual_id());
  auto target_reference_path =
      reference_path_manager->get_reference_path_by_lane(
          lc_lane_mgr_->target_lane_virtual_id());
  // auto &frenet_obstacles = current_reference_path->get_obstacles();
  auto &frenet_ego_state = fix_reference_path->get_frenet_ego_state();
  auto &target_lane_frenet_ego_state =
      target_reference_path->get_frenet_ego_state();

  lane_change_stage_info_.gap_insertable = true;
  lane_change_stage_info_.gap_approached = true;
  lane_change_stage_info_.gap_valid = true;
  lane_change_stage_info_.side_approach = false;
  lane_change_stage_info_.should_premove = last_should_premove_;
  lane_change_stage_info_.gap = {-10, -10};

  // lc_valid_ = true;
  // lc_invalid_reason_ = "none";
  lc_invalid_track_.reset();
  // lc_pause_id_ = -1000;
  // lc_pause_ = false;

  // side_approaching_ = true;

  double coefficient = FLAGS_planning_loop_rate / 25.;

  if (!lateral_obstacle->sensors_okay()) {
    if (lateral_obstacle->fvf_dead()) {
      lane_change_stage_info_.lc_invalid_reason = "no front view";
    } else if (lateral_obstacle->svf_dead()) {
      lane_change_stage_info_.lc_invalid_reason = "no side view";
    }

    lane_change_stage_info_.gap_valid = false;
    lane_change_stage_info_.gap_approached = false;
    lane_change_stage_info_.gap_insertable = false;
    return;
  }

  double v_ego = ego_state->ego_v();
  double l_ego = frenet_ego_state.l();
  double safety_dist = v_ego * v_ego * 0.02 + 2.0;
  double dist_mline = std::fabs(target_lane_frenet_ego_state.l()) -
                      1.8;  // TODO(Rui): use target_lane()->get_lane_width()
  double t_reaction = (dist_mline == DBL_MAX) ? 0.5 : 0.5 * dist_mline / 1.8;

  near_cars_target_.clear();
  std::vector<TrackInfo> near_cars_target;

  std::vector<TrackedObject> side_target_tracks;
  std::vector<TrackedObject> front_target_tracks;
  auto &obstacle_manager =
      session_->mutable_environmental_model()->get_obstacle_manager();
  auto tlane_obstacles =
      target_lane->get_reference_path()->get_lane_obstacles_ids();

  for (auto &obstacle : lateral_obstacle->side_tracks()) {
    if (std::count(tlane_obstacles.begin(), tlane_obstacles.end(),
                   obstacle.track_id) > 0) {
      side_target_tracks.push_back(obstacle);
    }
  }

  for (auto &obstacle : lateral_obstacle->front_tracks()) {
    if (std::count(tlane_obstacles.begin(), tlane_obstacles.end(),
                   obstacle.track_id) > 0) {
      front_target_tracks.push_back(obstacle);
    }
  }

  for (auto &tr : side_target_tracks) {
    TrackInfo side_track(tr.track_id, tr.d_rel, tr.v_rel);
    near_cars_target.push_back(side_track);
  }

  double mss = 0.0;
  double mss_t = 0.0;

  bool is_side_target_valid = true;
  for (auto &tr : side_target_tracks) {
    if (is_side_target_valid == true) {
      if (tr.type == 2 || tr.type == 3 || tr.type == 4) {
        safety_dist = v_ego * v_ego * 0.015 + 2.0;
      }

      if (tr.d_rel < safety_dist && tr.d_rel > -5.0 - safety_dist &&
          tr.v_rel > 100.0) {
        is_side_target_valid = false;
        // lc_invalid_reason_ = "side view invalid";
        lc_invalid_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
      } else if (tr.v_rel < 100.0) {
        if (tr.d_rel > -5.0) {
          if (tr.v_rel < 0) {
            mss = tr.v_rel * tr.v_rel / 2 + safety_dist;
            mss_t = mss;

            if (v_ego + tr.v_rel < 1) {
              mss_t = -3.5;
            }
            if (tr.d_rel < 0. && v_ego < 3) {
              mss = -3.5 + tr.v_rel * tr.v_rel / 2;
            }
          } else {
            mss = -tr.v_rel * tr.v_rel / 2 + safety_dist;
            if (tr.d_rel < 0. && v_ego < 3) {
              mss = -3.5 - tr.v_rel * tr.v_rel / 2;
            }
          }

          std::array<double, 2> xp{0, 3.5};
          std::array<double, 2> fp{1, 0.7};
          if (((tr.d_rel < interp(tr.v_rel, xp, fp) * safety_dist ||
                tr.d_rel < mss || (mss_t != mss && mss_t > -3.5)) &&
               (tr.d_rel >= 0 || v_ego >= 3)) ||
              (tr.d_rel < 0 && tr.d_rel >= mss && v_ego < 3)) {
            is_side_target_valid = false;
            // lc_invalid_reason_ = "side view invalid";
            lc_invalid_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
          }
        } else {
          if (v_ego < 17 && vel_sequence.size() > 1) {
            double temp1 = tr.v_rel;
            double temp2 = tr.v_rel - (vel_sequence[5] - v_ego);
            std::array<double, 5> xp1{-10., -2.5, 0., 3., 5.};
            std::array<double, 5> fp1{5., 2.5, 1.5, 0.5, 0.3};
            double a = interp(temp2, xp1, fp1);
            int sign = temp2 < 0 ? -1 : 1;
            mss = std::min(
                std::max(temp1 * t_reaction + sign * temp2 * temp2 / (2. * a) +
                             safety_dist,
                         3.0),
                120.0);

            if (tr.type == 2 || tr.type == 3 || tr.type == 4) {
              mss = std::min(
                  std::max(temp1 * (t_reaction + 1) +
                               sign * temp2 * temp2 / (2 * a) + safety_dist,
                           3.0),
                  120.0);
            }

            for (size_t i = 0; i + 5 < vel_sequence.size(); i++) {
              mss_t =
                  tr.d_rel +
                  std::max(tr.v_rel - (vel_sequence[i] - v_ego) / 2.0, 0.0) *
                      0.1 * i;

              if (((mss_t > -7.0 || tr.d_rel > -5.0 - mss) && tr.v_lead > 1) ||
                  (tr.v_lead <= 1 && tr.d_rel > -5.0)) {
                is_side_target_valid = false;
                // lc_invalid_reason_ = "side view invalid";
                lc_invalid_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
                break;
              }
            }
          } else {
            double temp = tr.v_rel;
            std::array<double, 5> xp1{-10., -2.5, 0., 3., 5.};
            std::array<double, 5> fp1{5., 2.5, 1.5, 0.5, 0.3};
            double a = interp(temp, xp1, fp1);
            int sign = temp < 0 ? -1 : 1;
            mss = std::min(
                std::max(temp * t_reaction + sign * temp * temp / (2 * a) +
                             safety_dist,
                         3.0),
                120.0);

            if (tr.type == 2 || tr.type == 3 || tr.type == 4) {
              mss = std::min(
                  std::max(temp * (t_reaction + 1) +
                               sign * temp * temp / (2 * a) + safety_dist,
                           3.0),
                  120.0);
            }
          }

          if ((tr.d_rel > -5.0 - mss && tr.v_lead > 1) ||
              (tr.v_lead <= 1 && tr.d_rel > -5.0)) {
            is_side_target_valid = false;
            // lc_invalid_reason_ = "side view invalid";
            lc_invalid_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
          }

          if (tr.v_rel > 0.0 && tr.d_rel > -2 * (5.0 + mss)) {
            // side_approaching_ = true;
            lane_change_stage_info_.side_approach = true;
          }

          if (tr.v_rel > 0.0 && tr.d_rel > -4 * (5.0 + mss)) {
            // should_premove_ = true;
            lane_change_stage_info_.should_premove = true;
          }
        }
      }
    } else {
      break;
    }
  }
  if (!is_side_target_valid) {
    lane_change_stage_info_.gap_insertable = false;
    lane_change_stage_info_.lc_invalid_reason = "side view invalid";
  }

  for (auto &tr : front_target_tracks) {
    TrackInfo front_track(tr.track_id, tr.d_rel, tr.v_rel);
    near_cars_target_.push_back(front_track);
  }

  bool is_front_target_valid = true;
  for (auto &tr : front_target_tracks) {
    if (is_front_target_valid == true) {
      if (tr.d_rel > -5.0) {
        std::array<double, 5> a_dec_v{2.0, 1.5, 1.3, 1.2, 1.0};
        std::array<double, 5> a_ace_v{0.3, 0.6, 0.8, 1.0, 1.5};
        std::array<double, 5> v_ego_bp{0, 10, 15, 20, 30};

        double a_dflc = interp(v_ego, v_ego_bp, a_dec_v);
        double a_aflc = interp(v_ego, v_ego_bp, a_ace_v);

        if (tr.v_rel < 0.0) {
          if (v_ego < 17 && vel_sequence.size() > 6) {
            double temp = std::min(tr.v_rel - vel_sequence[5] + v_ego, 0.0);
            mss = temp * temp / (2 * a_dflc) + safety_dist;

            for (size_t i = 0; i + 5 < vel_sequence.size(); i++) {
              mss_t = tr.d_rel +
                      std::min(tr.v_rel - (vel_sequence[i] - v_ego) / 2, 0.0) *
                          0.1 * i;

              if (mss_t < 2.0 || tr.d_rel < mss) {
                is_front_target_valid = false;
                // lc_invalid_reason_ = "front view invalid";
                lc_invalid_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
                break;
              }
            }
          } else {
            mss = tr.v_rel * tr.v_rel / (2 * a_dflc) + safety_dist;
          }
        } else {
          if (v_ego < 17 && vel_sequence.size() > 1) {
            double temp = tr.v_rel - vel_sequence[5] + v_ego;
            mss = -temp * temp / (2 * a_aflc) + safety_dist;

            for (size_t i = 0; i + 5 < vel_sequence.size(); i++) {
              mss_t = tr.d_rel +
                      (tr.v_rel - (vel_sequence[i] - v_ego) / 2) * 0.1 * i;

              if (mss_t < 2.0 || tr.d_rel < mss) {
                is_front_target_valid = false;
                // lc_invalid_reason_ = "front view invalid";
                lc_invalid_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
                break;
              }
            }
          } else {
            mss = std::pow(std::max(2 - tr.v_rel, 0.0), 2) / (2 * a_aflc) +
                  safety_dist;
          }
        }

        std::array<double, 2> xp{0, 3.5};
        std::array<double, 2> fp{1, 0.7};

        if (tr.d_rel < interp(tr.v_rel, xp, fp) * safety_dist ||
            tr.d_rel < mss) {
          is_front_target_valid = false;
          // lc_invalid_reason_ = "front view invalid";
          lc_invalid_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
        }
      }
    } else {
      break;
    }
  }
  if (!is_front_target_valid) {
    lane_change_stage_info_.gap_insertable = false;
    lane_change_stage_info_.lc_invalid_reason = "front view invalid";
  }

  return;
}

LaneChangeStageInfo LaneChangeDecider::decide_lc_valid_info(
    const bool localation_valid, RequestType direction) {
  last_should_premove_ = lane_change_stage_info_.should_premove;
  clear_lc_stage_info();
  lane_change_stage_info_.gap_insertable = true;
  if (!localation_valid) {
    compute_lc_valid_info(direction);
  }
  double coefficient = FLAGS_planning_loop_rate / 25.;
  int lc_valid_thre = static_cast<int>(10.0 * coefficient);
  auto &virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  double dis_to_ramp = virtual_lane_manager->dis_to_ramp();
  if (dis_to_ramp < 1000.) lc_valid_thre = 1;
  // int lc_valid_thre = 1;
  if (lane_change_stage_info_.gap_insertable) {
    lc_valid_cnt_ += 1;
    LOG_DEBUG("decide_lc_valid_info lc_valid_cnt : %d \n", lc_valid_cnt_);
    if (lc_valid_cnt_ > lc_valid_thre) {
      // lc_valid_cnt_ = 0; // todo: set value when choose change stae finally
      lane_change_stage_info_.lc_valid = true;
    } else {
      lane_change_stage_info_.gap_insertable = false;
      lane_change_stage_info_.lc_invalid_reason = "valid cnt below threshold";
    }
  } else {
    LOG_DEBUG("arbitrator lc invalid reason %s ",
              lane_change_stage_info_.lc_invalid_reason.c_str());
    lc_valid_cnt_ = 0;
  }
  return lane_change_stage_info_;
}

void LaneChangeDecider::compute_lc_back_info(RequestType direction) {
  auto &lateral_obstacle =
      session_->environmental_model().get_lateral_obstacle();
  auto &ego_state = session_->environmental_model().get_ego_state_manager();

  std::shared_ptr<VirtualLane> target_lane = lc_lane_mgr_->tlane();
  auto origin_lane = lc_lane_mgr_->olane();
  auto fix_lane = lc_lane_mgr_->flane();
  auto &virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  if (origin_lane == nullptr) {
    origin_lane = virtual_lane_manager->get_current_lane();
  }
  // if (direction == LEFT_CHANGE) {
  //   target_lane = virtual_lane_manager->get_left_neighbor(origin_lane);
  // } else if (direction == RIGHT_CHANGE) {
  //   target_lane = virtual_lane_manager->get_right_neighbor(origin_lane);
  // }
  if (target_lane == nullptr) {
    return;
  }
  if (fix_lane == nullptr) {
    fix_lane = virtual_lane_manager->get_current_lane();
  }

  int current_lane_virtual_id = session_->mutable_environmental_model()
                                    ->get_virtual_lane_manager()
                                    ->current_lane_virtual_id();
  auto reference_path_manager =
      session_->mutable_environmental_model()->get_reference_path_manager();
  std::cout << "compute_lc_back_info: fix_lane_virtual_id: "
            << lc_lane_mgr_->fix_lane_virtual_id()
            << " fix_lane->get_virtual_id(): " << fix_lane->get_virtual_id()
            << std::endl;
  std::cout << "compute_lc_back_info: target_lane_virtual_id: "
            << lc_lane_mgr_->target_lane_virtual_id()
            << " target_lane->get_virtual_id(): "
            << target_lane->get_virtual_id() << std::endl;
  auto fix_reference_path = reference_path_manager->get_reference_path_by_lane(
      lc_lane_mgr_->fix_lane_virtual_id());
  auto target_reference_path =
      reference_path_manager->get_reference_path_by_lane(
          lc_lane_mgr_->target_lane_virtual_id());
  // auto &frenet_obstacles = current_reference_path->get_obstacles();
  auto &frenet_ego_state = fix_reference_path->get_frenet_ego_state();
  auto &target_lane_frenet_ego_state =
      target_reference_path->get_frenet_ego_state();

  // 可以考虑自车heading(姿态,最近点、最远点)， 以及横向速度. (heading越大,
  // move_thre越大)
  std::array<double, 3> xp_heading{0, 0.5, 1};
  std::array<double, 3> fp_l{0, 0.4, 0.6};
  double c1 = 0;

  if (direction == LEFT_CHANGE) {
    c1 = target_lane->get_right_lane_boundary().poly_coefficient[1];
  } else if (direction == RIGHT_CHANGE) {
    c1 = target_lane->get_left_lane_boundary().poly_coefficient[1];
  }

  double move_thre =
      std::max(interp(std::fabs(c1), xp_heading, fp_l) +
                   ego_state->ego_v() * config_.lc_t_actuator_delay,
               config_.lc_back_available_thr);
  double lane_width = 3.8;  // TODO(Rui):use fix_lne->get_lane_width()
  double left_lane_width = 3.8;
  double right_lane_width = 3.8;
  double car_width = 2.2;  // TODO(Rui):load config
  double l_ego = frenet_ego_state.l();

  lc_back_track_.reset();

  // if (!map_info.is_in_intersection() &&
  //     map_info.left_refline_points().size() > 0) {
  auto left_lane = virtual_lane_manager->get_left_lane();
  if (left_lane != nullptr) {
    for (auto &p : left_lane->lane_points()) {
      if (p.car_point.x >= 0 && p.car_point.x <= 20 && !p.is_in_intersection) {
        left_lane_width = p.lane_width;
        if (left_lane_width < 100) {
          break;
        }
      }
    }
  }

  // if (!map_info.is_in_intersection() &&
  //     map_info.right_refline_points().size() > 0) {
  auto right_lane = virtual_lane_manager->get_right_lane();
  if (right_lane != nullptr) {
    for (auto &p : right_lane->lane_points()) {
      if (p.car_point.x >= 0 && p.car_point.x <= 20 && !p.is_in_intersection) {
        right_lane_width = p.lane_width;
        if (right_lane_width < 100) {
          break;
        }
      }
    }
  }

  if (!lateral_obstacle->sensors_okay()) {
    if (lateral_obstacle->fvf_dead()) {
      lane_change_stage_info_.lc_back_reason = "no front view";
    } else if (lateral_obstacle->svf_dead()) {
      lane_change_stage_info_.lc_back_reason = "no side view";
    }
    lane_change_stage_info_.lc_should_back = true;
    // lc_back_cnt_ = 0;
    return;
  }

  double mss = 0.0;
  double mss_t = 0.0;

  double v_ego = ego_state->ego_v();
  double safety_dist = v_ego * 0.2 + 2;
  // double dist_mline = virtual_lane_mgr.dist_mline(direction);
  double dist_mline = std::fabs(target_lane_frenet_ego_state.l()) -
                      1.8;  // TODO(Rui): use target_lane()->get_lane_width()
  double pause_ttc = 2.0;
  double pause_v_rel = 2.0;

  double t_reaction = (dist_mline == DBL_MAX) ? 1.0 : 1.0 * dist_mline / 1.8;

  near_cars_target_.clear();

  std::vector<TrackInfo> near_cars_target;
  std::vector<TrackedObject> side_target_tracks;
  std::vector<TrackedObject> front_target_tracks;
  auto &obstacle_manager =
      session_->mutable_environmental_model()->get_obstacle_manager();
  auto tlane_obstacles =
      target_lane->get_reference_path()->get_lane_obstacles_ids();

  for (auto &obstacle : lateral_obstacle->side_tracks()) {
    if (std::count(tlane_obstacles.begin(), tlane_obstacles.end(),
                   obstacle.track_id) > 0) {
      side_target_tracks.push_back(obstacle);
    }
  }

  for (auto &obstacle : lateral_obstacle->front_tracks()) {
    if (std::count(tlane_obstacles.begin(), tlane_obstacles.end(),
                   obstacle.track_id) > 0) {
      front_target_tracks.push_back(obstacle);
    }
  }

  if (side_target_tracks.size() > 0) {
    for (auto &tr : side_target_tracks) {
      TrackInfo side_track(tr.track_id, tr.d_rel, tr.v_rel);
      near_cars_target.push_back(side_track);
    }

    for (auto &tr : side_target_tracks) {
      auto &x = tr.trajectory.x;
      auto &y = tr.trajectory.y;
      std::vector<double> ego_x;
      std::vector<double> ego_y;
      std::vector<double> ego_speed;
      std::vector<double> ego_yaw;
      double ego_fx = std::cos(ego_state->heading_angle());
      double ego_fy = std::sin(ego_state->heading_angle());
      double ego_lx = -ego_fy;
      double ego_ly = ego_fx;
      double theta = 0.0;
      double send = 0.0;
      double lend = 0.0;
      int end_idx = 0;
      if (tr.trajectory.intersection == 0) {
        for (int i = 0; i < x.size(); i++) {
          double dx = x[i] - ego_state->ego_pose().x;  // TODO:有问题
          double dy = y[i] - ego_state->ego_pose().y;

          double rel_x = dx * ego_fx + dy * ego_fy;
          double rel_y = dx * ego_lx + dy * ego_ly;
          ego_x.push_back(rel_x);
          ego_y.push_back(rel_y);
        }
        int end_range = min((int)ego_x.size() - 1, 25);
        for (int i = end_range; i >= 0; i--) {
          if (ego_x[i] < 0 || ego_x[i] > 80) {
            continue;
          }

          end_idx = i;
          break;
        }
        // f_refline.cartesian_frenet(ego_x[end_idx], ego_y[end_idx], send,
        // lend,
        //                            theta, false); //TODO(Rui)
      }
      if (lane_change_stage_info_.lc_should_back == false &&
          ((direction == LEFT_CHANGE &&
            lend < car_width + 0.3 - lane_width / 2 + tr.width / 2) ||
           (direction == RIGHT_CHANGE &&
            lend > -(car_width + 0.3) + lane_width / 2 - tr.width / 2) ||
           tr.trajectory.intersection != 0)) {
        if (tr.type == 2 || tr.type == 3 || tr.type == 4) {
          safety_dist = v_ego * 0.1 + 1;  // changed
        }

        if (tr.d_rel < safety_dist && tr.d_rel > -5.0 - safety_dist &&
            tr.v_rel > 100.0) {
          lane_change_stage_info_.lc_should_back = true;
          lane_change_stage_info_.lc_back_reason = "side view back";
          lc_back_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
          lane_change_stage_info_.lc_pause_id = tr.track_id;
        } else if (tr.v_rel < 100.0) {
          if (tr.d_rel > -5.0) {
            if (tr.v_rel < 0) {
              mss = tr.v_rel * tr.v_rel / 4 + safety_dist;
              mss_t = mss;

              if (v_ego + tr.v_rel > 1) {
                mss_t = -2;
              }
              if (tr.d_rel < 0. && tr.v_lead < 1) {
                mss = -2 + tr.v_rel * tr.v_rel / 2;
              }
            } else {
              mss = -tr.v_rel * tr.v_rel / 4 + safety_dist;
            }

            std::array<double, 2> xp{0, 3.5};
            std::array<double, 2> fp{1, 0.7};

            if (((tr.d_rel < 0.5 * interp(tr.v_rel, xp, fp) * safety_dist ||
                  tr.d_rel < mss || (mss_t != mss && mss_t > -2)) &&
                 tr.v_lead >= 1) ||
                (tr.d_rel > mss && tr.v_lead < 1)) {
              lane_change_stage_info_.lc_should_back = true;
              lane_change_stage_info_.lc_back_reason = "side view back";
              lc_back_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
              if (tr.d_rel < 0 &&
                  tr.d_rel > lane_change_stage_info_.tr_pause_s) {
                lane_change_stage_info_.lc_pause_id = tr.track_id;
                lane_change_stage_info_.tr_pause_s = tr.d_rel;
                lane_change_stage_info_.tr_pause_l = tr.d_center_cpath;
                lane_change_stage_info_.tr_pause_dv = tr.v_rel;
              }
            }
          } else {
            if (tr.v_rel >= 0) {
              double temp = std::max(tr.v_rel, 0.0);
              double lat_thre = car_width + 0.6 - lane_width / 2;
              std::array<double, 3> xp1{0., 3., 5.};
              std::array<double, 3> fp1{2, 1.5, 1.};
              double a = interp(temp, xp1, fp1);

              if (std::fabs(virtual_lane_manager->lc_map_decision(fix_lane)) ==
                  1) {
                mss = tr.v_rel * t_reaction / 2 * 0.7 +
                      temp * temp / (2 * a) * 0.7 + safety_dist -
                      2 / (tr.v_rel + 1);

                if (tr.type == 2 || tr.type == 3 || tr.type == 4) {
                  mss = tr.v_rel * (t_reaction + 2) / 2 * 0.7 +
                        temp * temp / 2 * 0.7 + safety_dist * 1.1 -
                        2 / (tr.v_rel + 1);
                }
              } else {
                mss = tr.v_rel * t_reaction / 2 + temp * temp / (2 * a) +
                      safety_dist - 2 / (tr.v_rel + 1);
                if (tr.type == 2 || tr.type == 3 || tr.type == 4) {
                  mss = tr.v_rel * (t_reaction + 2) / 2 + temp * temp / 2 +
                        safety_dist - 2 / (tr.v_rel + 1);
                }
              }

              if (tr.d_rel >
                      -5.0 -
                          (mss -
                           std::max(
                               std::max(
                                   (std::max(l_ego - tr.d_max_cpath - 1.6,
                                             tr.d_min_cpath - l_ego - 1.6)),
                                   0.0) /
                                   std::max((-tr.v_lat + 0.7), 0.1) * tr.v_rel,
                               0.0)) &&
                  ((tr.d_max_cpath >= -lat_thre &&
                    tr.d_min_cpath <= lat_thre) ||
                   (tr.d_max_cpath -
                        tr.v_lat * std::min(tr.d_rel + 5 / tr.v_rel, 4.) >=
                    -lat_thre) ||
                   (tr.d_min_cpath +
                        tr.v_lat * std::min(tr.d_rel + 5 / tr.v_rel, 4.) <=
                    lat_thre))) {
                lane_change_stage_info_.lc_should_back = true;
                lane_change_stage_info_.lc_back_reason = "side view back";
                lc_back_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
                if (tr.d_rel < 0 &&
                    tr.d_rel > lane_change_stage_info_.tr_pause_s) {
                  lane_change_stage_info_.lc_pause_id = tr.track_id;
                  lane_change_stage_info_.tr_pause_s = tr.d_rel;
                  lane_change_stage_info_.tr_pause_l = tr.d_center_cpath;
                  lane_change_stage_info_.tr_pause_dv = tr.v_rel;
                }
              }
            }
          }
        }
      } else {
        break;
      }
    }
  }

  double distance = frenet_ego_state.l();
  if (((direction == LEFT_CHANGE && start_move_dist_lane_ != 0 &&
        distance > -left_lane_width / 2 - move_thre) ||
       (direction == RIGHT_CHANGE && start_move_dist_lane_ != 0 &&
        distance < right_lane_width / 2 + move_thre) ||
       fix_lane->get_virtual_id() == current_lane_virtual_id)) {
    if (lane_change_stage_info_.lc_should_back &&
        lane_change_stage_info_.tr_pause_dv > pause_v_rel &&
        -lane_change_stage_info_.tr_pause_s /
                lane_change_stage_info_.tr_pause_dv <
            pause_ttc &&
        ((direction == LEFT_CHANGE &&
          lane_change_stage_info_.tr_pause_l - distance > 0.5) ||
         (direction == RIGHT_CHANGE &&
          distance - lane_change_stage_info_.tr_pause_l > 0.5))) {
      lane_change_stage_info_.lc_pause = true;
    }
    if (lane_change_stage_info_.lc_should_back) {
      behavior_suspend_ = true;  // lateral suspend
      suspend_obs_.push_back(lc_back_track_.track_id);
    }
    lane_change_stage_info_.lc_should_back = false;
    lane_change_stage_info_.lc_back_reason = "exceed move_thre, do not back";
    return;
  }

  if (front_target_tracks.size() > 0) {
    for (auto &tr : front_target_tracks) {
      TrackInfo front_track(tr.track_id, tr.d_rel, tr.v_rel);
      near_cars_target_.push_back(front_track);
    }

    for (auto &tr : front_target_tracks) {
      if (lane_change_stage_info_.lc_should_back == false) {
        std::array<double, 2> a_dec_v{3.0, 2.0};
        std::array<double, 2> v_ego_bp{6, 20};
        double a_dflc = interp(v_ego, v_ego_bp, a_dec_v);
        if (tr.v_rel < 0) {
          mss = tr.v_rel * tr.v_rel / (2 * a_dflc) + safety_dist;
        } else {
          mss = std::max(-tr.v_rel * tr.v_rel / 4 + safety_dist, 2.0);
        }

        double lat_condi = std::max((std::max(l_ego - tr.d_max_cpath - 1.6,
                                              tr.d_min_cpath - l_ego - 1.6)),
                                    0.0) /
                           (std::max(-tr.v_lat + 0.7, 0.3));
        if ((lat_condi < 1.5 && tr.d_rel < 1.0) ||
            (lat_condi <= 1.5 &&
             tr.d_rel < 0.8 * (mss - std::max(lat_condi * tr.v_rel, 0.0)))) {
          lane_change_stage_info_.lc_should_back = true;
          lane_change_stage_info_.lc_back_reason = "front view back";
          lc_back_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
        }
      } else {
        break;
      }
    }
  }

  if (lane_change_stage_info_.close_to_accident &&
      lane_change_stage_info_.lc_should_back) {
    lane_change_stage_info_.lc_should_back = false;
    lane_change_stage_info_.accident_back = true;
  }

  return;
}

LaneChangeStageInfo LaneChangeDecider::decide_lc_back_info(
    const bool localation_valid, RequestType direction) {
  clear_lc_stage_info();
  if (!localation_valid) {
    compute_lc_back_info(direction);
  }
  double coefficient = FLAGS_planning_loop_rate / 25.;
  int lc_back_thre = static_cast<int>(5 * coefficient);
  if (lane_change_stage_info_.lc_should_back) {
    lc_back_cnt_ += 1;
    if (lc_back_cnt_ > lc_back_thre) {
      // reset lc_back_cnt_ when choose back finally
    } else {
      lane_change_stage_info_.lc_should_back = false;
      lane_change_stage_info_.lc_back_reason = "but back cnt below threshold";
    }
  } else {
    lc_back_cnt_ = 0;
  }
  return lane_change_stage_info_;
}

bool LaneChangeDecider::check_lc_change_finish(RequestType direction) {
  auto &virtual_lane_mgr =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  auto clane = virtual_lane_mgr->get_current_lane();
  auto clane_virtual_id = virtual_lane_mgr->current_lane_virtual_id();

  if (clane == nullptr || !lc_lane_mgr_->has_target_lane() ||
      !lc_lane_mgr_->has_fix_lane()) {
    LOG_ERROR(
        "[check_lc_change_finish] invalid clane [%d] or tlane [%d] or flane",
        clane == nullptr, !lc_lane_mgr_->has_target_lane(),
        !lc_lane_mgr_->has_fix_lane());
    return false;
  }
  auto tlane = lc_lane_mgr_->tlane();
  auto target_lane_virtual_id = lc_lane_mgr_->target_lane_virtual_id();
  int fix_lane_virtual_id = lc_lane_mgr_->fix_lane_virtual_id();

  if (target_lane_virtual_id != clane_virtual_id) {
    // tlane与clane的virtual id和lc_map_decision都不相同时，换道一定没有完成.
    return false;
  }

  double dist_threshold = 0.5;
  double v_ego =
      session_->mutable_environmental_model()->get_ego_state_manager()->ego_v();
  std::vector<double> angle_thre_v{0.72, 0.48, 0.12};
  std::vector<double> angle_thre_bp{1.0, 3.0, 5.0};
  double angle_threshold = interp(v_ego, angle_thre_bp, angle_thre_v);

  std::shared_ptr<ReferencePathManager> reference_path_mgr =
      session_->mutable_environmental_model()->get_reference_path_manager();
  std::cout << "check_lc_change_finish: target_lane_virtual_id: "
            << target_lane_virtual_id
            << " clane_virtual_id: " << clane_virtual_id
            << " fix_lane_virtual_id: " << fix_lane_virtual_id << std::endl;
  auto target_reference_path =
      reference_path_mgr->get_reference_path_by_lane(target_lane_virtual_id);
  auto frenet_ego_state = target_reference_path->get_frenet_ego_state();

  bool lc_change_finish{false};
  if (direction == RIGHT_CHANGE) {
    lc_change_finish =
        ((frenet_ego_state.l() < dist_threshold) &&
         (std::fabs(frenet_ego_state.heading_angle()) < angle_threshold));
  } else if (direction == LEFT_CHANGE) {
    lc_change_finish =
        ((frenet_ego_state.l() > -dist_threshold) &&
         (std::fabs(frenet_ego_state.heading_angle()) < angle_threshold));
  } else {
    LOG_ERROR("[check_lc_change_finish] invalid direction[%d]", direction);
    lc_change_finish = true;
  }

  if (lc_change_finish == true) {
    auto ad_info = &(session_->mutable_planning_context()
                         ->mutable_planning_hmi_info()
                         ->ad_info);
    ad_info->lane_change_status = iflyauto::LC_COMPLETED;
  }
  return lc_change_finish;
}

bool LaneChangeDecider::check_lc_back_finish(RequestType direction) {
  if (!lc_lane_mgr_->has_origin_lane() || !lc_lane_mgr_->has_fix_lane()) {
    return false;
  }
  assert(lc_lane_mgr_->origin_lane_virtual_id() ==
         lc_lane_mgr_->fix_lane_virtual_id());

  // Find the half lane width
  std::shared_ptr<VirtualLaneManager> virtual_lane_mgr =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  auto flane = virtual_lane_mgr->get_lane_with_virtual_id(
      lc_lane_mgr_->fix_lane_virtual_id());
  double half_lane_width =
      1.6;  // HACK  TODO(Rui):get real fix_lane->width() / 2.
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double ego_width = vehicle_param.width;
  double dist_threshold = half_lane_width - 0.5 * ego_width - 0.1;
  double angle_threshold = 0.01;

  std::shared_ptr<ReferencePathManager> reference_path_mgr =
      session_->mutable_environmental_model()->get_reference_path_manager();
  auto fix_reference_path = reference_path_mgr->get_reference_path_by_lane(
      lc_lane_mgr_->fix_lane_virtual_id());
  auto frenet_ego_state = fix_reference_path->get_frenet_ego_state();

  bool lc_back_finish{false};
  if (direction == LEFT_CHANGE) {
    lc_back_finish =
        ((frenet_ego_state.l() < dist_threshold) &&
         (std::fabs(frenet_ego_state.heading_angle()) < angle_threshold));
  } else if (direction == RIGHT_CHANGE) {
    lc_back_finish =
        ((frenet_ego_state.l() > -dist_threshold) &&
         (std::fabs(frenet_ego_state.heading_angle()) < angle_threshold));
  } else {
    LOG_ERROR("[check_lc_back_finish] invalid direction[%d]", direction);
    lc_back_finish = true;
  }
  return lc_back_finish;
}

void LaneChangeDecider::generate_state_machine_output(
    const LaneChangeStageInfo &lc_info) {
  auto &lane_change_decider_output = session_->mutable_planning_context()
                                         ->mutable_lane_change_decider_output();

  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;

  lane_change_decider_output.curr_state = transition_context_.target_state;

  lane_change_decider_output.fix_lane_virtual_id =
      coarse_planning_info.target_lane_id;
  lane_change_decider_output.origin_lane_virtual_id =
      coarse_planning_info.source_lane_id;

  if (lane_change_decider_output.curr_state == ROAD_LC_LCHANGE ||
      lane_change_decider_output.curr_state == ROAD_LC_RCHANGE) {
    lane_change_decider_output.lc_timer += 0.1;  // cumulative lc time
  } else {
    lane_change_decider_output.lc_timer = 0.;  // reset
  }

  lane_change_decider_output.fix_lane_virtual_id =
      lc_lane_mgr_->fix_lane_virtual_id();
  lane_change_decider_output.origin_lane_virtual_id =
      lc_lane_mgr_->origin_lane_virtual_id();
  lane_change_decider_output.target_lane_virtual_id =
      lc_lane_mgr_->target_lane_virtual_id();
  lane_change_decider_output.has_target_lane = lc_lane_mgr_->has_target_lane();

  lane_change_decider_output.scenario = scenario_;
  // lane_change_decider_output.curr_state = target_state;
  // lane_change_decider_output.state_name = target_state_name;
  lane_change_decider_output.lc_back_reason = lc_info.lc_back_reason;
  lane_change_decider_output.lc_invalid_reason = lc_info.lc_invalid_reason;
  lane_change_decider_output.behavior_suspend =
      behavior_suspend_;  // lateral suspend
  lane_change_decider_output.suspend_obs.assign(
      suspend_obs_.begin(), suspend_obs_.end());  // lateral obstacles
  lane_change_decider_output.must_change_lane = must_change_lane_;

  // if (target_state == type2int<RoadState::None>::value) {
  //   lane_change_decider_output.turn_light = 0;
  // } else {
  if (map_turn_signal_ != NO_CHANGE) {
    lane_change_decider_output.turn_light = map_turn_signal_;
  } else if (lc_req_mgr_->turn_signal() > 0) {
    lane_change_decider_output.turn_light = lc_req_mgr_->turn_signal();
  } else {
    lane_change_decider_output.turn_light = 0;
  }
  // }
  lane_change_decider_output.map_turn_light = map_turn_signal_;
  lane_change_decider_output.accident_back = lc_info.accident_back;
  lane_change_decider_output.accident_ahead = lc_info.accident_ahead;
  lane_change_decider_output.close_to_accident = lc_info.close_to_accident;
  lane_change_decider_output.should_premove = lc_info.should_premove;
  lane_change_decider_output.should_suspend = lc_info.should_suspend;
  lane_change_decider_output.lc_pause = lc_info.lc_pause;
  lane_change_decider_output.lc_pause_id = lc_info.lc_pause_id;
  lane_change_decider_output.tr_pause_l = lc_info.tr_pause_l;
  lane_change_decider_output.tr_pause_s = lc_info.tr_pause_s;
  lane_change_decider_output.disable_l = false;
  lane_change_decider_output.disable_r = false;
  lane_change_decider_output.enable_l = false;
  lane_change_decider_output.enable_r = false;
  lane_change_decider_output.need_clear_lb_car = true;
  lane_change_decider_output.enable_lb = false;
  lane_change_decider_output.lc_request = lc_req_mgr_->request();
  lane_change_decider_output.lc_request_source = lc_req_mgr_->request_source();
  lane_change_decider_output.act_request_source =
      lc_req_mgr_->act_request_source();
  lane_change_decider_output.lc_turn_light = lc_req_mgr_->turn_signal();

  lane_change_decider_output.premovel = false;
  lane_change_decider_output.premover = false;
  lane_change_decider_output.premove_dist = 0.;

  // TODO(Rui):待加入object_selector
  lane_change_decider_output.left_is_faster =
      object_selector_->left_is_faster();
  lane_change_decider_output.right_is_faster =
      object_selector_->right_is_faster();
  lane_change_decider_output.neg_left_lb_car =
      object_selector_->neg_left_lb_car();
  lane_change_decider_output.neg_right_lb_car =
      object_selector_->neg_right_lb_car();
  lane_change_decider_output.neg_left_alc_car =
      object_selector_->neg_left_alc_car();
  lane_change_decider_output.neg_right_alc_car =
      object_selector_->neg_right_alc_car();

  lane_change_decider_output.left_lb_car = object_selector_->left_lb_car();
  lane_change_decider_output.left_alc_car = object_selector_->left_alc_car();
  lane_change_decider_output.right_lb_car = object_selector_->right_lb_car();
  lane_change_decider_output.right_alc_car = object_selector_->right_alc_car();

  lane_change_decider_output.enable_alc_car_protection = false;
  // lane_change_decider_output.alc_cars_ = alc_cars_; //TODO(Rui):待添加
  lane_change_decider_output.near_cars_target = near_cars_target_;
  lane_change_decider_output.near_cars_origin = near_cars_origin_;
  lane_change_decider_output.lc_invalid_track = lc_invalid_track_;
  lane_change_decider_output.lc_back_track = lc_back_track_;

  lane_change_decider_output.lc_valid_cnt = lc_valid_cnt_;
  lane_change_decider_output.lc_back_cnt = lc_back_cnt_;
  lane_change_decider_output.is_lc_valid = lc_info.lc_valid;
  // lane_change_decider_output.lc_back_invalid_reason =
  // lc_info.invalid_back_reason_;

  lane_change_decider_output.start_move_dist_lane = start_move_dist_lane_;
}

void LaneChangeDecider::UpdateCoarsePlanningInfo() {
  // Step 1) 计算state
  auto &coarse_planning_info = session_->mutable_planning_context()
                                   ->mutable_lane_change_decider_output()
                                   .coarse_planning_info;
  coarse_planning_info.source_state = transition_context_.source_state;
  coarse_planning_info.target_state = transition_context_.target_state;
  coarse_planning_info.lane_change_request_source =
      lc_req_mgr_->request_source();
  coarse_planning_info.source_lane_id = lc_lane_mgr_->origin_lane_virtual_id();
  coarse_planning_info.target_lane_id = lc_lane_mgr_->fix_lane_virtual_id();
  coarse_planning_info.bind_end_state = transition_context_.bind_end_state;
  if (transition_context_.target_state == ROAD_LC_LWAIT ||
      transition_context_.target_state == ROAD_LC_RWAIT ||
      transition_context_.target_state == ROAD_LC_LBACK ||
      transition_context_.target_state == ROAD_LC_RBACK) {
    coarse_planning_info.overtake_obstacles =
        transition_context_.overtake_obstacles;
    coarse_planning_info.yield_obstacles = transition_context_.yield_obstacles;
  } else {
    coarse_planning_info.overtake_obstacles.clear();
    coarse_planning_info.yield_obstacles.clear();
  }

  // Step 2) build reference path
  // session_
  //     ->mutable_planning_context()
  //     ->mutable_planning_result()
  //     .use_refined_reference_path = false;
  coarse_planning_info.reference_path =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->make_map_lane_reference_path(coarse_planning_info.target_lane_id);

  const auto &planning_init_point =
      coarse_planning_info.reference_path->get_frenet_ego_state()
          .planning_init_point();
  // Step 3) calculate trajectory points
  // generate reference path
  static const double min_ego_v_cruise = 2.0;
  const auto &v_ref_cruise = std::fmax(
      session_->environmental_model().get_ego_state_manager()->ego_v_cruise(),
      min_ego_v_cruise);

  static const size_t &N = config_.num_point;
  const auto &delta_time = config_.delta_t;

  auto &cart_ref_info = coarse_planning_info.cart_ref_info;
  const auto &frenet_coord =
      coarse_planning_info.reference_path->get_frenet_coord();

  double s = 0.0;
  Point2D frenet_pt{s, 0.0};
  Point2D cart_pt(0.0, 0.0);
  const auto &ref_point = coarse_planning_info.reference_path->get_points();
  auto point_size = ref_point.size();
  cart_ref_info.x_vec.resize(point_size);
  cart_ref_info.y_vec.resize(point_size);
  cart_ref_info.s_vec.resize(point_size);
  float normal_care_spline_length = 50.;
  const float preview_time = 20.;
  const double min_preview_spline_length = 20.;

  if (session_->is_hpp_scene()) {
    const double kHppMaxRearDistance = 30.0;
    double distance_to_first_point = kHppMaxRearDistance;
    if (point_size > 0) {
      double diff_x =
          ref_point.at(0).path_point.x - planning_init_point.lat_init_state.x();
      double diff_y =
          ref_point.at(0).path_point.y - planning_init_point.lat_init_state.y();
      distance_to_first_point = std::sqrt(diff_x * diff_x + diff_y * diff_y);
    }

    normal_care_spline_length =
        std::min(kHppMaxRearDistance, distance_to_first_point);
  }

  for (size_t i = 0; i < point_size; ++i) {
    cart_ref_info.x_vec[i] = ref_point.at(i).path_point.x;
    cart_ref_info.y_vec[i] = ref_point.at(i).path_point.y;
    cart_ref_info.s_vec[i] =
        i > 0 ? cart_ref_info.s_vec[i - 1] +
                    std::hypot(ref_point.at(i).path_point.x -
                                   ref_point.at(i - 1).path_point.x,
                               ref_point.at(i).path_point.y -
                                   ref_point.at(i - 1).path_point.y)
              : 0.;
    if (cart_ref_info.s_vec[i] >
        normal_care_spline_length +
            std::max(v_ref_cruise * preview_time, min_preview_spline_length)) {
      cart_ref_info.x_vec.resize(i);
      cart_ref_info.y_vec.resize(i);
      cart_ref_info.s_vec.resize(i);
      break;
    }
  }

  cart_ref_info.x_s_spline.set_points(cart_ref_info.s_vec, cart_ref_info.x_vec);
  cart_ref_info.y_s_spline.set_points(cart_ref_info.s_vec, cart_ref_info.y_vec);

  JSON_DEBUG_VECTOR("raw_refline_x_vec", cart_ref_info.x_vec, 2)
  JSON_DEBUG_VECTOR("raw_refline_y_vec", cart_ref_info.y_vec, 2)

  Eigen::Vector2d init_pos(planning_init_point.lat_init_state.x(),
                           planning_init_point.lat_init_state.y());
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();

  JSON_DEBUG_VALUE("ego_pos_x", ego_state->ego_pose_raw().x)
  JSON_DEBUG_VALUE("ego_pos_y", ego_state->ego_pose_raw().y)
  JSON_DEBUG_VALUE("ego_pos_yaw", ego_state->ego_pose_raw().theta)
  JSON_DEBUG_VALUE("init_pos_x", init_pos.x())
  JSON_DEBUG_VALUE("init_pos_y", init_pos.y())

  // start from project s
  pnc::spline::Projection projection_spline;
  projection_spline.CalProjectionPoint(
      cart_ref_info.x_s_spline, cart_ref_info.y_s_spline,
      cart_ref_info.s_vec.front(), cart_ref_info.s_vec.back(), init_pos);

  double s_ref = projection_spline.GetOutput().s_proj;

  const auto &frenet_length = frenet_coord->Length();

  s_ref = std::min(s_ref, frenet_length * 0.95);
  const double delta_s = frenet_length - s_ref;
  const double v_cruise_scale = std::min(delta_s / (v_ref_cruise * 5.0), 1.0);
  session_->mutable_planning_context()->set_v_ref_cruise(v_cruise_scale *
                                                         v_ref_cruise);

  coarse_planning_info.trajectory_points.clear();
  TrajectoryPoint point;
  for (size_t i = 0; i < N; ++i) {
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
    point.t = static_cast<double>(i) * delta_time;

    s_ref += v_cruise_scale * v_ref_cruise * delta_time;
    coarse_planning_info.trajectory_points.emplace_back(point);
  }
}

bool LaneChangeDecider::CheckEgoPosition() const {
  // Step 1) check reference_path
  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;

  if (coarse_planning_info.reference_path == nullptr) {
    LOG_DEBUG("reference_path is null");
    return false;
  }

  // Step 2) check Init state when target_state is CRUISE_CHANGE
  if (coarse_planning_info.target_state == ROAD_LC_LCHANGE ||
      coarse_planning_info.target_state == ROAD_LC_RCHANGE) {
    auto &reference_path = coarse_planning_info.reference_path;
    auto &frenet_ego_state = reference_path->get_frenet_ego_state();

    auto &ego_boundary = frenet_ego_state.boundary();
    std::vector<double> ego_s_list = {ego_boundary.s_start, ego_boundary.s_end,
                                      frenet_ego_state.s()};
    for (auto s : ego_s_list) {
      ReferencePathPoint refpath_pt{};
      (void)reference_path->get_reference_point_by_lon(s, refpath_pt);
      auto ego_l = frenet_ego_state.l();
      if (ego_l < -refpath_pt.distance_to_right_road_border or
          ego_l > refpath_pt.distance_to_left_road_border) {
        LOG_DEBUG("init_state error");
        return false;
      }
    }
  }

  return true;
}

void LaneChangeDecider::UpdateStateMachineDebugInfo() {
  const auto &lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  auto &debug_info_manager = DebugInfoManager::GetInstance();
  auto &planning_debug_data = debug_info_manager.GetDebugInfoPb();
  auto lat_behavior_common = planning_debug_data->mutable_lat_behavior_common();
  lat_behavior_common->set_current_state(lane_change_decider_output.curr_state);
  lat_behavior_common->set_lc_invalid_obj_id(
      lane_change_decider_output.lc_invalid_track.track_id);
  lat_behavior_common->set_lc_back_obj_id(
      lane_change_decider_output.lc_back_track.track_id);
  lat_behavior_common->set_lc_invalid_reason(
      lane_change_decider_output.lc_invalid_reason);
  lat_behavior_common->set_lc_back_reason(
      lane_change_decider_output.lc_back_reason);
  lat_behavior_common->set_lc_back_invalid_reason(
      lane_change_decider_output.lc_back_invalid_reason);
  lat_behavior_common->near_car_ids_origin().Clear();
  for (auto &near_car_origin : lane_change_decider_output.near_cars_origin) {
    lat_behavior_common->add_near_car_ids_origin(near_car_origin.track_id);
  }
  lat_behavior_common->near_car_ids_target().Clear();
  for (auto &near_car_target : lane_change_decider_output.near_cars_target) {
    lat_behavior_common->add_near_car_ids_target(near_car_target.track_id);
  }
  lat_behavior_common->set_is_faster_left_lane(
      lane_change_decider_output.left_is_faster);
  lat_behavior_common->set_is_faster_right_lane(
      lane_change_decider_output.right_is_faster);
  lat_behavior_common->left_alc_car_ids().Clear();
  for (auto id : lane_change_decider_output.left_alc_car) {
    lat_behavior_common->add_left_alc_car_ids(id);
  }
  lat_behavior_common->right_alc_car_ids().Clear();
  for (auto id : lane_change_decider_output.right_alc_car) {
    lat_behavior_common->add_right_alc_car_ids(id);
  }
  lat_behavior_common->set_is_forbid_left_alc_car(
      lane_change_decider_output.neg_left_alc_car);
  lat_behavior_common->set_is_forbid_right_alc_car(
      lane_change_decider_output.neg_right_alc_car);
  lat_behavior_common->set_fix_lane_virtual_id(
      lane_change_decider_output.fix_lane_virtual_id);
  lat_behavior_common->set_origin_lane_virtual_id(
      lane_change_decider_output.origin_lane_virtual_id);
  lat_behavior_common->set_target_lane_virtual_id(
      lane_change_decider_output.target_lane_virtual_id);

  lat_behavior_common->set_turn_light(lane_change_decider_output.turn_light);
  lat_behavior_common->set_map_turn_light(
      lane_change_decider_output.map_turn_light);
  lat_behavior_common->set_lc_request(lane_change_decider_output.lc_request);
  lat_behavior_common->set_lc_request_source(
      lane_change_decider_output.lc_request_source);
  lat_behavior_common->set_lc_turn_light(
      lane_change_decider_output.lc_turn_light);
  lat_behavior_common->set_act_request_source(
      lane_change_decider_output.act_request_source);

  lat_behavior_common->set_is_lc_valid(lane_change_decider_output.is_lc_valid);
  lat_behavior_common->set_lc_valid_cnt(
      lane_change_decider_output.lc_valid_cnt);
  lat_behavior_common->set_lc_back_cnt(lane_change_decider_output.lc_back_cnt);
}

void LaneChangeDecider::UpdateAdInfo() {
  auto ad_info = &(session_->mutable_planning_context()
                       ->mutable_planning_hmi_info()
                       ->ad_info);

  // feed hmi
  ad_info->lane_change_direction = iflyauto::LC_OTHER;
  if (transition_context_.target_state == ROAD_LC_LWAIT ||
      transition_context_.target_state == ROAD_LC_RWAIT) {
    ad_info->lane_change_status = iflyauto::LC_WAITING;
    if (transition_context_.target_state == ROAD_LC_LWAIT) {
      ad_info->lane_change_direction = iflyauto::LC_LEFT;
    } else {
      ad_info->lane_change_direction = iflyauto::LC_RIGHT;
    }
  } else if (transition_context_.target_state == ROAD_LC_LCHANGE ||
             transition_context_.target_state == ROAD_LC_RCHANGE) {
    if (transition_context_.target_state == ROAD_LC_LCHANGE) {
      ad_info->lane_change_direction = iflyauto::LC_LEFT;
    } else {
      ad_info->lane_change_direction = iflyauto::LC_RIGHT;
    }
    ad_info->lane_change_status = iflyauto::LC_STARTING;
  } else if (transition_context_.target_state == ROAD_NONE) {
    if (transition_context_.source_state == ROAD_LC_LWAIT ||
        transition_context_.source_state == ROAD_LC_RWAIT ||
        transition_context_.source_state == ROAD_LC_LCHANGE ||
        transition_context_.source_state == ROAD_LC_RCHANGE ||
        transition_context_.source_state == ROAD_LC_LBACK ||
        transition_context_.source_state == ROAD_LC_RBACK) {
      if (ad_info->lane_change_status == iflyauto::LC_COMPLETED) {
        // check_lc_change_finish
      } else {
        ad_info->lane_change_status = iflyauto::LC_CANCELLED;
      }

      if (transition_context_.source_state == ROAD_LC_LWAIT ||
          transition_context_.source_state == ROAD_LC_LCHANGE ||
          transition_context_.source_state == ROAD_LC_LBACK) {
        ad_info->lane_change_direction = iflyauto::LC_LEFT;
      } else {
        ad_info->lane_change_direction = iflyauto::LC_RIGHT;
      }
    }
  }

  if (transition_context_.source_state == ROAD_LC_LWAIT ||
      transition_context_.source_state == ROAD_LC_RWAIT ||
      transition_context_.source_state == ROAD_LC_LCHANGE ||
      transition_context_.source_state == ROAD_LC_RCHANGE) {
    auto target_reference =
        session_->environmental_model()
            .get_reference_path_manager()
            ->get_reference_path_by_lane(lc_lane_mgr_->target_lane_virtual_id(),
                                         false);
    if (target_reference != nullptr) {
      Point2D cart_point;
      if (target_reference->get_frenet_coord()->SLToXY(
              Point2D(target_reference->get_frenet_ego_state().s() + 5, 0),
              cart_point)) {
        ad_info->landing_point.relative_pos.x = cart_point.x;
        ad_info->landing_point.relative_pos.y = cart_point.y;
        ad_info->landing_point.relative_pos.z = 0;
        ad_info->landing_point.heading = 0;
      }
    }
  }
}

bool LaneChangeDecider::UpdateObjectSelector(bool active) {
#ifndef CHERY_T26
  if (!session_->is_hpp_scene()) {
    if (active && !object_selector_->update(session_->planning_context()
                                                .lane_change_decider_output()
                                                .curr_state,
                                            session_->planning_context()
                                                .lane_change_decider_output()
                                                .start_move_dist_lane,
                                            false, 80., false, false, false,
                                            false, false, -1)) {
      LOG_DEBUG("object_selector_update fail\n");
      return false;
    } else {
      if (!active) {
        object_selector_->left_alc_car().clear();
        object_selector_->right_alc_car().clear();
        object_selector_->left_alc_car_cnt().clear();
        object_selector_->right_alc_car_cnt().clear();
        LOG_DEBUG("object_selector_ cleared\n");
      } else {
        LOG_DEBUG("object_selector_updated \n");
      }
    }
    LOG_DEBUG("object_selector_update end\n");
  }
#endif

  return true;
}

void LaneChangeDecider::UpdateFixLaneVirtualId() {
  std::shared_ptr<ReferencePathManager> reference_path_mgr =
      session_->mutable_environmental_model()->get_reference_path_manager();

  auto &lane_change_decider_output = session_->mutable_planning_context()
                                         ->mutable_lane_change_decider_output();

  int fix_lane_virtual_id = lane_change_decider_output.fix_lane_virtual_id;
  int current_lane_virtual_id = session_->environmental_model()
                                    .get_virtual_lane_manager()
                                    ->current_lane_virtual_id();
  auto fix_reference_path = reference_path_mgr->get_reference_path_by_lane(
      fix_lane_virtual_id, false);
  if (fix_reference_path == nullptr) {
    lc_lane_mgr_->reset_lc_lanes();
    lane_change_decider_output.fix_lane_virtual_id = current_lane_virtual_id;
    lane_change_decider_output.origin_lane_virtual_id = current_lane_virtual_id;
    lane_change_decider_output.target_lane_virtual_id = current_lane_virtual_id;
  }
}

void LaneChangeDecider::UpdateLaneChangeState() {
  transition_context_.source_state = transition_context_.target_state;
  transition_context_.direction = lc_req_mgr_->request();
  switch (transition_context_.target_state) {
    case ROAD_NONE:
      ProcessNoneState();
      break;
    case ROAD_LC_LCHANGE:
    case ROAD_LC_RCHANGE:
      ProcessChangeState();
      break;
    case ROAD_LC_LWAIT:
    case ROAD_LC_RWAIT:
      ProcessWaitState();
      break;
    case ROAD_LC_LBACK:
    case ROAD_LC_RBACK:
      ProcessBackState();
      break;
    default:
      ProcessNoneState();
      break;
  }
}

void LaneChangeDecider::ProcessNoneState() {
  const RequestType lc_request = lc_req_mgr_->request();
  const int target_lane_virtual_id = lc_req_mgr_->target_lane_virtual_id();
  LaneChangeStageInfo lc_info;
  if (lc_request == LEFT_CHANGE || lc_request == RIGHT_CHANGE) {
    lc_lane_mgr_->assign_lc_lanes(target_lane_virtual_id);
    PrepareForWaitState();
  } else {
    PrepareForNoneState();
  }
  // TODO(xjli32): need to merge
  generate_state_machine_output(lc_info);

  transition_context_.bind_end_state = true;
  transition_context_.overtake_obstacles.clear();
  transition_context_.yield_obstacles.clear();
}

void LaneChangeDecider::ProcessWaitState() {
  const bool location_valid = session_->environmental_model().location_valid();

  const RequestType lc_request = lc_req_mgr_->request();
  const RequestSource lc_source = lc_req_mgr_->request_source();
  const bool aggressive_change = lc_req_mgr_->AggressiveChange();
  bool gap_available = true;

  bool enable_arbitrator = false;
  double lc_tstart = lc_req_mgr_->GetReqStartTime(lc_source);
  double delay_time = 0.;  // TODO(Rui):后面根据请求来源做成配置项
  const double dis_to_ramp =
      session_->environmental_model().get_virtual_lane_manager()->dis_to_ramp();
  if (dis_to_ramp < 1000.) {
    delay_time = 0.;
  }
  std::cout << " normal road state dis_to_ramp: " << dis_to_ramp
            << " delay_time: " << delay_time << std::endl;
  const double curr_time = IflyTime::Now_ms();  // 注意
  std::vector<int> overtake_obstacles;
  std::vector<int> yield_obstacles;
  LaneChangeStageInfo lane_change_info;
  if (lc_request != NO_CHANGE && lc_request == transition_context_.direction) {
    int target_lane_virtual_id = lc_req_mgr_->target_lane_virtual_id();
    if (!lc_lane_mgr_->has_target_lane() ||
        lc_lane_mgr_->target_lane_virtual_id() != target_lane_virtual_id) {
      lc_lane_mgr_->assign_lc_lanes(target_lane_virtual_id);
    }
    gap_available =
        location_valid
            ? true
            : GapAvailable(lc_request, overtake_obstacles, yield_obstacles);
    lane_change_info = decide_lc_valid_info(location_valid, lc_request);
    LOG_DEBUG(
        "[CruiseState::Wait] gap_available: %d, aggressive_change: %d, "
        "target_lane_virtual_id: %d, "
        "lane_change_info.gap_insertable: %d, curr_time: %f, lc_tstart: %f, "
        "delay_time: %f \n",
        gap_available, aggressive_change, target_lane_virtual_id,
        lane_change_info.gap_insertable, curr_time, lc_tstart, delay_time);
    // if (gap_available || aggressive_change) {
    // //TODO(Rui):后面把安全检查坐在gap_available里，统一通过gap_available判断
    if (curr_time > lc_tstart + delay_time && lane_change_info.gap_insertable) {
      PrepareForChangeState();
      if ((transition_context_.target_state == ROAD_LC_LCHANGE ||
           transition_context_.target_state == ROAD_LC_RCHANGE) &&
          enable_arbitrator) {
        PrepareForWaitState();
      }
    } else {
      PrepareForWaitState();
    }
  } else {
    PrepareForNoneState();
  }

  // 需要合并
  generate_state_machine_output(lane_change_info);

  UpdateTransitionContext(std::move(overtake_obstacles),
                          std::move(yield_obstacles), gap_available);
}

void LaneChangeDecider::ProcessChangeState() {
  const bool location_valid = session_->environmental_model().location_valid();

  RequestType lc_request = lc_req_mgr_->request();
  bool gap_available = true;
  bool enable_arbitrator = false;
  int fix_lane_virtual_id = lc_lane_mgr_->fix_lane_virtual_id();
  auto virtual_lane_mgr =
      session_->environmental_model().get_virtual_lane_manager();
  auto flane = virtual_lane_mgr->get_lane_with_virtual_id(fix_lane_virtual_id);
  //
  const double move_thr = 1.5;
  const double flane_width = flane->width();
  std::vector<int> overtake_obstacles;
  std::vector<int> yield_obstacles;
  LaneChangeStageInfo lc_back_info;
  if (check_lc_change_finish(transition_context_.direction)) {
    LOG_DEBUG("[RoadState::Change] Lane Change Finished\n");
    PrepareForNoneState();
    lc_req_mgr_->FinishRequest();
  } else if ((lc_request != NO_CHANGE &&
              lc_request == transition_context_.direction) ||
             (lc_request == NO_CHANGE &&
              (lc_lane_mgr_->is_ego_on(lc_lane_mgr_->tlane()) ||
               std::fabs(flane->get_ego_lateral_offset()) <
                   (flane_width / 2 + move_thr)))) {
    int target_lane_virtual_id = lc_req_mgr_->target_lane_virtual_id();
    if (!lc_lane_mgr_->has_target_lane() ||
        target_lane_virtual_id != lc_lane_mgr_->target_lane_virtual_id()) {
      lc_lane_mgr_->assign_lc_lanes(target_lane_virtual_id);
    }

    gap_available =
        location_valid
            ? true  // @cailiu: general planning consider lc valid
                    // in GapSelectorDecider
            : GapAvailable(lc_request, overtake_obstacles, yield_obstacles);

    if (lc_lane_mgr_->has_origin_lane() &&
        lc_lane_mgr_->origin_lane_virtual_id() !=
            lc_lane_mgr_->target_lane_virtual_id()) {
      lc_back_info = decide_lc_back_info(location_valid, lc_request);
      if (lc_back_info.lc_should_back) {
        PrepareForBackState();
        if ((transition_context_.target_state == ROAD_LC_LBACK ||
             transition_context_.target_state == ROAD_LC_RBACK) &&
            lc_lane_mgr_->has_target_lane() && enable_arbitrator) {
          PrepareForChangeState();
        }
      } else {
        transition_context_.target_state = ROAD_NONE;
        PrepareForChangeState();
        if (lc_request == NO_CHANGE &&
            transition_context_.direction != NO_CHANGE) {
          if (transition_context_.direction == LEFT_CHANGE) {
            transition_context_.target_state = ROAD_LC_LCHANGE;
          } else if (transition_context_.direction == RIGHT_CHANGE) {
            transition_context_.target_state = ROAD_LC_RCHANGE;
          }
        }
        std::cout << "Coming to prepare for change state !!!!! lc_request: "
                  << lc_request << " transition_context_.direction: "
                  << transition_context_.direction
                  << " target_lane_virtual_id: " << target_lane_virtual_id
                  << std::endl;
        if ((transition_context_.target_state == ROAD_LC_LCHANGE ||
             transition_context_.target_state == ROAD_LC_RCHANGE) &&
            !lc_lane_mgr_->is_ego_on(lc_lane_mgr_->tlane()) &&
            enable_arbitrator) {
          PrepareForBackState();
        }
      }
    } else if (lc_lane_mgr_->has_target_lane()) {
      transition_context_.target_state = ROAD_NONE;
      PrepareForChangeState();
    } else {
      PrepareForNoneState();
    }
  } else {
    if (lc_request != transition_context_.direction ||
        !lc_lane_mgr_->is_ego_on(lc_lane_mgr_->tlane())) {
      LOG_DEBUG("[RoadState::Change] Change to None\n");
      PrepareForNoneState();
    } else {
      LOG_DEBUG("[RoadState::Change] Change to Back\n");
      PrepareForBackState();
    }
  }

  generate_state_machine_output(lc_back_info);

  UpdateTransitionContext(std::move(overtake_obstacles),
                          std::move(yield_obstacles), gap_available);
}

void LaneChangeDecider::ProcessBackState() {
  const bool location_valid = session_->environmental_model().location_valid();
  RequestType lc_request = lc_req_mgr_->request();
  bool aggressive_change{lc_req_mgr_->AggressiveChange()};
  bool gap_available{true};
  std::vector<int> overtake_obstacles;
  std::vector<int> yield_obstacles;
  LaneChangeStageInfo lane_change_info;
  if (check_lc_back_finish(transition_context_.direction)) {
    // prepare for WAIT state
    LOG_DEBUG("[RoadeState::Back] Lane Back Finished\n");
    PrepareForWaitState();
  } else if (lc_request != NO_CHANGE &&
             lc_request == transition_context_.direction) {
    if (!lc_lane_mgr_->has_target_lane()) {
      int target_lane_virtual_id = lc_req_mgr_->target_lane_virtual_id();
      lc_lane_mgr_->assign_lc_lanes(target_lane_virtual_id);
    }
    gap_available =
        location_valid
            ? true
            : GapAvailable(lc_request, overtake_obstacles, yield_obstacles);
    lane_change_info = decide_lc_valid_info(location_valid, lc_request);
    // if (gap_available || aggressive_change) {
    // //TODO(Rui):后面把安全检查坐在gap_available里，统一通过gap_available判断
    if (lane_change_info.gap_insertable) {
      PrepareForChangeState();
    } else {
      LOG_DEBUG("[CruiseState haowen] Wait to Back\n");
      PrepareForBackState();
    }
  } else {
    PrepareForNoneState();
  }

  generate_state_machine_output(lane_change_info);

  UpdateTransitionContext(std::move(overtake_obstacles),
                          std::move(yield_obstacles), gap_available);
}

void LaneChangeDecider::PrepareForBackState() {
  if (lc_lane_mgr_->has_origin_lane()) {
    lc_lane_mgr_->set_fix_lane_to_origin();
    if (lc_req_mgr_->request() == LEFT_CHANGE) {
      transition_context_.target_state = ROAD_LC_LBACK;
    } else if (lc_req_mgr_->request() == RIGHT_CHANGE) {
      transition_context_.target_state = ROAD_LC_RBACK;
    } else {
      transition_context_.target_state = ROAD_NONE;
      lc_lane_mgr_->reset_lc_lanes();
    }
  } else {
    transition_context_.target_state = ROAD_NONE;
    lc_lane_mgr_->reset_lc_lanes();
  }
}

void LaneChangeDecider::PrepareForChangeState() {
  if (lc_lane_mgr_->has_target_lane()) {
    lc_lane_mgr_->set_fix_lane_to_target();
  } else {
    int target_lane_virtual_id = lc_req_mgr_->target_lane_virtual_id();
    lc_lane_mgr_->assign_lc_lanes(target_lane_virtual_id);
    lc_lane_mgr_->set_fix_lane_to_target();
  }

  // make sure origin lane exists, fitting reference line may need it.
  if (!lc_lane_mgr_->has_origin_lane()) {
    lc_lane_mgr_->reset_origin_lane();
  }

  if (lc_req_mgr_->request() == LEFT_CHANGE) {
    transition_context_.target_state = ROAD_LC_LCHANGE;
  } else if (lc_req_mgr_->request() == RIGHT_CHANGE) {
    transition_context_.target_state = ROAD_LC_RCHANGE;
  } else {
    // transition_context_.target_state = ROAD_NONE;
  }
}

void LaneChangeDecider::PrepareForWaitState() {
  if (lc_lane_mgr_->has_origin_lane()) {
    lc_lane_mgr_->set_fix_lane_to_origin();
    if (lc_req_mgr_->request() == LEFT_CHANGE) {
      transition_context_.target_state = ROAD_LC_LWAIT;
    } else if (lc_req_mgr_->request() == RIGHT_CHANGE) {
      transition_context_.target_state = ROAD_LC_RWAIT;
    } else {
      transition_context_.target_state = ROAD_NONE;
      lc_lane_mgr_->reset_lc_lanes();
    }
  } else {
    lc_lane_mgr_->reset_lc_lanes();
    transition_context_.target_state = ROAD_NONE;
  }
}

void LaneChangeDecider::PrepareForNoneState() {
  lc_lane_mgr_->reset_lc_lanes();
  transition_context_.target_state = ROAD_NONE;
}

void LaneChangeDecider::UpdateTransitionContext(
    std::vector<int> &&overtake_obstacles, std::vector<int> &&yield_obstacles,
    const bool gap_available) {
  transition_context_.bind_end_state =
      (transition_context_.target_state == ROAD_LC_LCHANGE ||
       transition_context_.target_state == ROAD_LC_RCHANGE)
          ? gap_available
          : true;
  ;
  if (transition_context_.target_state == ROAD_LC_LWAIT ||
      transition_context_.target_state == ROAD_LC_RWAIT ||
      transition_context_.target_state == ROAD_LC_LBACK ||
      transition_context_.target_state == ROAD_LC_RBACK) {
    transition_context_.overtake_obstacles = std::move(overtake_obstacles);
    transition_context_.yield_obstacles = std::move(yield_obstacles);
  }
}

}  // namespace planning
