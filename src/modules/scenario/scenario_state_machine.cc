#include "modules/scenario/scenario_state_machine.h"

#include <limits>
#include <numeric>

#include "modules/common/utils/pose2d_utils.h"
#include "modules/context/ego_state_manager.h"
#include "modules/context/obstacle_manager.h"
#include "modules/context/reference_path.h"
#include "modules/context/reference_path_manager.h"
#include "modules/context/vehicle_config_context.h"
#include "src/modules/tasks/behavior_planners/vision_only_lateral_behavior_planner/vision_lateral_behavior_planner.h"

namespace planning {

ScenarioStateMachine::ScenarioStateMachine(
    const EgoPlanningConfigBuilder *config_builder,
    planning::framework::Session *session)
    : scenario_fsm_(fsm_context_), session_(session) {
  std::shared_ptr<VirtualLaneManager> virtual_lane_mgr =
      // session_->mutable_planning_context()->virtual_lane_manager();
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  lc_lane_mgr_ =
      std::make_shared<LaneChangeLaneManager>(virtual_lane_mgr, session);
  lc_req_mgr_ = std::make_shared<LaneChangeRequestManager>(
      session, config_builder, virtual_lane_mgr, lc_lane_mgr_);
}

ScenarioStateMachine::~ScenarioStateMachine() {}

void ScenarioStateMachine::init() {
  fsm_context_.state_machine = shared_from_this();
  fsm_context_.state = ROAD_NONE;
  fsm_context_.external = false;
  fsm_context_.name = "ROAD_NONE";
  fsm_context_.session = session_;
  scenario_fsm_.changeTo<RoadState::None>();
  // scenario_fsm_.update(); // TODO fix null frame
  lc_lane_mgr_->reset_lc_lanes();
}

bool ScenarioStateMachine::update(planning::framework::Frame *frame) {
  fsm_context_.frame = frame;
  bool active = session_->environmental_model().GetVehicleDbwStatus();

  if (!session_->mutable_planning_context()->last_planning_success()) {
    reset_state_machine();
  }
  session_->mutable_planning_context()->mutable_planning_success() = false;

  update_scenario();  // cruise or low speed or ...

  if (active == false) {
    lc_req_mgr_->FinishRequest();
    map_turn_signal_ = NO_CHANGE;
    reset_state_machine();
  }

  if (!fsm_context_.initialized) {
    reset_state_machine();
    fsm_context_.initialized = true;
  }

  if (active) {
    LOG_DEBUG("[scenario_state_machine] active\n");
    if (scenario_ == SCENARIO_CRUISE) {
      // update lc_req_mgr_
      lc_req_mgr_->Update(fsm_context_.state,
                          session_->environmental_model().IsOnRoute());
      gen_map_turn_signal();
      update_state_machine();
      post_process();
    } else {
      return false;
    }
  } else {
    LOG_DEBUG("[scenario_state_machine] not active\n");
    if (scenario_ == SCENARIO_CRUISE) {
      // update lc_req_mgr_
      lc_req_mgr_->Update(fsm_context_.state,
                          session_->environmental_model().IsOnRoute());
      gen_map_turn_signal();
      update_state_machine();
      post_process();
    } else {
      return false;
    }
    // post_process();
  }
  return true;
}

void ScenarioStateMachine::update_state_machine() {
  if (fsm_context_.state >= ROAD_NONE && fsm_context_.state <= INTER_UT_NONE) {
    if (scenario_ == SCENARIO_CRUISE) {
      scenario_fsm_.update();
    } else {
      LOG_ERROR("Scenario Not Implemented!");
      change_state_external<RoadState::None>();
      lc_lane_mgr_->reset_lc_lanes();
      map_turn_signal_ = NO_CHANGE;
    }
  } else if (fsm_context_.state > INTER_UT_NONE) {
    LOG_ERROR("State Not Implemented!");
    change_state_external<RoadState::None>();
    lc_lane_mgr_->reset_lc_lanes();
    map_turn_signal_ = NO_CHANGE;
  }
}

void ScenarioStateMachine::reset_state_machine() {
  change_state_external<RoadState::None>();
  lc_lane_mgr_->reset_lc_lanes();
  lc_req_mgr_->FinishRequest();
  clear_lc_variables();
}

void ScenarioStateMachine::clear_lc_variables() {
  lc_valid_cnt_ = 0;
  lb_back_cnt_ = 0;
  lc_pause_id_ = -1000;
  lc_valid_ = true;
  lc_should_back_ = false;
  lc_valid_back_ = true;
  must_change_lane_ = false;

  lc_back_reason_ = "none";
  lc_invalid_reason_ = "none";
  invalid_back_reason_ = "none";

  lc_invalid_track_.reset();
  lc_back_track_.reset();

  near_cars_target_.clear();
  near_cars_origin_.clear();

  start_move_dist_lane_ = 0;

  should_premove_ = false;
  should_suspend_ = false;
  accident_ahead_ = false;
  close_to_accident_ = false;
  accident_back_ = false;
  lc_pause_ = false;
  not_accident_ = true;
  behavior_suspend_ = false;  // lateral suspend
  suspend_obs_.clear();
}

void ScenarioStateMachine::update_scenario() { scenario_ = SCENARIO_CRUISE; }

void ScenarioStateMachine::update_start_move_dist_lane() {
  int flane_virtual_id = lc_lane_mgr_->flane_virtual_id();
  auto fix_reference_path = session_->environmental_model()
                                .get_reference_path_manager()
                                ->get_reference_path_by_lane(flane_virtual_id);
  auto frenet_ego_state = fix_reference_path->get_frenet_ego_state();
  start_move_dist_lane_ = frenet_ego_state.l();
}

void ScenarioStateMachine::post_process() {
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
      "[ScenarioStateMachine] turn_signal_on_time_ %f map_turn_signal: %d, "
      "lc_turn_signal: %d, merge_split_turn_signal: %d, turn_signal_: %d",
      turn_signal_on_time_, (int)map_turn_signal_,
      (int)lc_req_mgr_->turn_signal(), (int)merge_split_turn_signal_,
      (int)turn_signal_);
}

void ScenarioStateMachine::gen_map_turn_signal() {
  std::shared_ptr<VirtualLaneManager> map_info_mgr =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
}

void ScenarioStateMachine::gen_merge_split_turn_signal() {}

bool ScenarioStateMachine::gap_available(RequestType direction,
                                         std::vector<int> &overtake_obstacles,
                                         std::vector<int> &yield_obstacles) {
  bool b_gap_available{true};
  return b_gap_available;
}

LaneChangeStageInfo ScenarioStateMachine::compute_lc_valid_info(
    RequestType direction) {
  assert(direction == LEFT_CHANGE || direction == RIGHT_CHANGE);
  LaneChangeStageInfo result_info;
  auto &lateral_obstacle =
      session_->environmental_model().get_lateral_obstacle();
  auto &ego_state = session_->environmental_model().get_ego_state_manager();

  auto &lateral_output =
      session_->planning_context().lateral_behavior_planner_output();
  auto &vel_sequence = lateral_output.vel_sequence;

  std::shared_ptr<VirtualLane> target_lane = nullptr;
  auto origin_lane = lc_lane_mgr_->olane();
  auto fix_lane = lc_lane_mgr_->flane();
  auto &virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  if (origin_lane == nullptr) {
    origin_lane = virtual_lane_manager->get_current_lane();
  }
  if (direction == LEFT_CHANGE) {
    target_lane = virtual_lane_manager->get_left_neighbor(origin_lane);
  } else if (direction == RIGHT_CHANGE) {
    target_lane = virtual_lane_manager->get_right_neighbor(origin_lane);
  }
  if (target_lane == nullptr) {
    return result_info;
  }
  if (fix_lane == nullptr) {
    fix_lane = virtual_lane_manager->get_current_lane();
  }

  // int current_lane_virtual_id = session_->mutable_environmental_model()
  //                                   ->virtual_lane_manager()
  //                                   ->current_lane_virtual_id();
  auto reference_path_manager =
      session_->mutable_environmental_model()->get_reference_path_manager();
  auto fix_reference_path = reference_path_manager->get_reference_path_by_lane(
      fix_lane->get_virtual_id());
  auto target_reference_path =
      reference_path_manager->get_reference_path_by_lane(
          target_lane->get_virtual_id());
  // auto &frenet_obstacles = current_reference_path->get_obstacles();
  auto &frenet_ego_state = fix_reference_path->get_frenet_ego_state();
  auto &target_lane_frenet_ego_state =
      target_reference_path->get_frenet_ego_state();

  result_info.gap_insertable = true;
  result_info.gap_approached = true;
  result_info.gap_valid = true;
  result_info.side_approach = false;
  result_info.should_premove = should_premove_;
  result_info.gap = {-10, -10};

  // lc_valid_ = true;
  // lc_invalid_reason_ = "none";
  lc_invalid_track_.reset();
  // lc_pause_id_ = -1000;
  // lc_pause_ = false;

  // side_approaching_ = true;

  double coefficient = 20. / 25;  // hack FLAGS_planning_loop_rate = 20.

  if (!lateral_obstacle->sensors_okay()) {
    if (lateral_obstacle->fvf_dead()) {
      result_info.lc_invalid_reason = "no front view";
    } else if (lateral_obstacle->svf_dead()) {
      result_info.lc_invalid_reason = "no side view";
    }

    result_info.gap_valid = false;
    result_info.gap_approached = false;
    result_info.gap_insertable = false;
    return result_info;
  }

  double v_ego = ego_state->ego_v();
  double l_ego = frenet_ego_state.l();
  double safety_dist = v_ego * v_ego * 0.01 + 2.0;
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
      obstacle_manager->get_lane_obstacles(target_lane->get_virtual_id());

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
            result_info.side_approach = true;
          }

          if (tr.v_rel > 0.0 && tr.d_rel > -4 * (5.0 + mss)) {
            // should_premove_ = true;
            result_info.should_premove = true;
          }
        }
      }
    } else {
      break;
    }
  }
  if (!is_side_target_valid) {
    result_info.gap_insertable = false;
    result_info.lc_invalid_reason = "side view invalid";
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
    result_info.gap_insertable = false;
    result_info.lc_invalid_reason = "front view invalid";
  }

  return result_info;
}

LaneChangeStageInfo ScenarioStateMachine::decide_lc_valid_info(
    RequestType direction) {
  auto raw_info = compute_lc_valid_info(direction);
  double coefficient = 20. / 25;  // TODO(Rui): FLAGS_planning_loop_rate = 20.
  int lc_valid_thre = static_cast<int>(10.0 * coefficient);
  // int lc_valid_thre = 0.0;
  if (raw_info.gap_insertable) {
    lc_valid_cnt_ += 1;
    LOG_DEBUG("decide_lc_valid_info lc_valid_cnt : %d", lc_valid_cnt_);
    if (lc_valid_cnt_ > lc_valid_thre) {
      // lc_valid_cnt_ = 0; // todo: set value when choose change stae finally
    } else {
      raw_info.gap_insertable = false;
      raw_info.lc_invalid_reason = "valid cnt below threshold";
    }
  } else {
    LOG_DEBUG("arbitrator lc invalid reason %s ",
              raw_info.lc_invalid_reason.c_str());
    lc_valid_cnt_ = 0;
  }

  // update
  lc_invalid_reason_ = raw_info.lc_invalid_reason;
  return raw_info;
}

LaneChangeStageInfo ScenarioStateMachine::compute_lc_back_info(
    RequestType direction) {
  LaneChangeStageInfo result;

  auto &lateral_obstacle =
      session_->environmental_model().get_lateral_obstacle();
  auto &ego_state = session_->environmental_model().get_ego_state_manager();

  auto &lateral_output =
      session_->planning_context().lateral_behavior_planner_output();
  auto &vel_sequence = lateral_output.vel_sequence;

  std::shared_ptr<VirtualLane> target_lane = nullptr;
  auto origin_lane = lc_lane_mgr_->olane();
  auto fix_lane = lc_lane_mgr_->flane();
  auto &virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  if (origin_lane == nullptr) {
    origin_lane = virtual_lane_manager->get_current_lane();
  }
  if (direction == LEFT_CHANGE) {
    target_lane = virtual_lane_manager->get_left_neighbor(origin_lane);
  } else if (direction == RIGHT_CHANGE) {
    target_lane = virtual_lane_manager->get_right_neighbor(origin_lane);
  }
  if (target_lane == nullptr) {
    return result;
  }
  if (fix_lane == nullptr) {
    fix_lane = virtual_lane_manager->get_current_lane();
  }

  int current_lane_virtual_id = session_->mutable_environmental_model()
                                    ->get_virtual_lane_manager()
                                    ->current_lane_virtual_id();
  auto reference_path_manager =
      session_->mutable_environmental_model()->get_reference_path_manager();
  auto fix_reference_path = reference_path_manager->get_reference_path_by_lane(
      fix_lane->get_virtual_id());
  auto target_reference_path =
      reference_path_manager->get_reference_path_by_lane(
          target_lane->get_virtual_id());
  // auto &frenet_obstacles = current_reference_path->get_obstacles();
  auto &frenet_ego_state = fix_reference_path->get_frenet_ego_state();
  auto &target_lane_frenet_ego_state =
      target_reference_path->get_frenet_ego_state();

  double move_thre = 0.4;
  double lane_width = 3.8;  // TODO(Rui):use fix_lne->get_lane_width()
  double left_lane_width = 3.8;
  double right_lane_width = 3.8;
  double car_width = 2.2;  // TODO(Rui):load config
  double l_ego = frenet_ego_state.l();

  lc_back_track_.reset();

  // if (!map_info.is_in_intersection() &&
  //     map_info.left_refline_points().size() > 0) {
  if (virtual_lane_manager->get_left_lane()->lane_points().size() > 0) {
    for (auto &p : virtual_lane_manager->get_left_lane()->lane_points()) {
      if (p.car_point().x() >= 0 && p.car_point().x() <= 20 &&
          !p.is_in_intersection()) {
        left_lane_width = p.lane_width();
        if (left_lane_width < 100) {
          break;
        }
      }
    }
  }

  // if (!map_info.is_in_intersection() &&
  //     map_info.right_refline_points().size() > 0) {
  if (virtual_lane_manager->get_right_lane()->lane_points().size() > 0) {
    for (auto &p : virtual_lane_manager->get_right_lane()->lane_points()) {
      if (p.car_point().x() >= 0 && p.car_point().x() <= 20 &&
          !p.is_in_intersection()) {
        right_lane_width = p.lane_width();
        if (right_lane_width < 100) {
          break;
        }
      }
    }
  }

  if (!lateral_obstacle->sensors_okay()) {
    if (lateral_obstacle->fvf_dead()) {
      result.lc_back_reason = "no front view";
    } else if (lateral_obstacle->svf_dead()) {
      result.lc_back_reason = "no side view";
    }
    result.lc_should_back = true;
    // lc_back_cnt_ = 0;
    return result;
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
      obstacle_manager->get_lane_obstacles(target_lane->get_virtual_id());

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
          double dx = x[i] - ego_state->ego_pose().x;
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
      if (lc_should_back_ == false &&
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
          result.lc_should_back = true;
          result.lc_back_reason = "side view back";
          lc_back_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
          result.lc_pause_id = tr.track_id;
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
              result.lc_should_back = true;
              result.lc_back_reason = "side view back";
              lc_back_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
              if (tr.d_rel < 0 && tr.d_rel > tr_pause_s_) {
                result.lc_pause_id = tr.track_id;
                result.tr_pause_s = tr.d_rel;
                result.tr_pause_l = tr.d_center_cpath;
                result.tr_pause_dv = tr.v_rel;
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
                result.lc_should_back = true;
                result.lc_back_reason = "side view back";
                lc_back_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
                if (tr.d_rel < 0 && tr.d_rel > tr_pause_s_) {
                  result.lc_pause_id = tr.track_id;
                  result.tr_pause_s = tr.d_rel;
                  result.tr_pause_l = tr.d_center_cpath;
                  result.tr_pause_dv = tr.v_rel;
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
       fix_lane->get_virtual_id() ==
           current_lane_virtual_id)) {  // TODO(Rui): &&
                                        // !object_selector_->jam_cancel()
    if (result.lc_should_back && result.tr_pause_dv > pause_v_rel &&
        -result.tr_pause_s / result.tr_pause_dv < pause_ttc &&
        ((direction == LEFT_CHANGE && tr_pause_l_ - distance > 0.5) ||
         (direction == RIGHT_CHANGE && distance - tr_pause_l_ > 0.5))) {
      result.lc_pause = true;
    }
    if (result.lc_should_back) {
      behavior_suspend_ = true;  // lateral suspend
      suspend_obs_.push_back(lc_back_track_.track_id);
    }
    result.lc_should_back = false;
    result.lc_back_reason = "exceed move_thre, do not back";
    return result;
  }

  if (front_target_tracks.size() > 0) {
    for (auto &tr : front_target_tracks) {
      TrackInfo front_track(tr.track_id, tr.d_rel, tr.v_rel);
      near_cars_target_.push_back(front_track);
    }

    for (auto &tr : front_target_tracks) {
      if (lc_should_back_ == false) {
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
          result.lc_should_back = true;
          result.lc_back_reason = "front view back";
          lc_back_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
        }
      } else {
        break;
      }
    }
  }

  if (close_to_accident_ && result.lc_should_back) {
    result.lc_should_back = false;
    result.accident_back = true;
  }
  // if (virtual_lane_mgr.has_target_lane() &&
  //     virtual_lane_mgr.is_on_lane(TARGET_LANE) &&
  //     object_selector_->jam_cancel()) {
  //   result.lc_should_back = true;
  // } //TODO(Rui)
  return result;
}

LaneChangeStageInfo ScenarioStateMachine::decide_lc_back_info(
    RequestType direction) {
  auto lc_info = compute_lc_back_info(direction);
  double coefficient = 20. / 25;  // TODO(Rui): FLAGS_planning_loop_rate = 20.
  int lc_back_thre = static_cast<int>(5 * coefficient);
  if (lc_info.lc_should_back) {
    lc_back_cnt_ += 1;
    if (lc_back_cnt_ > lc_back_thre) {
      // reset lc_back_cnt_ when choose back finally
    } else {
      lc_info.lc_should_back = false;
      lc_info.lc_back_reason = "but back cnt below threshold";
    }
  } else {
    lc_back_cnt_ = 0;
  }
  lc_back_reason_ = lc_info.lc_back_reason;
  return lc_info;
}

bool ScenarioStateMachine::check_lc_change_finish(RequestType direction) {
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
  auto tlane_virtual_id = lc_lane_mgr_->tlane_virtual_id();

  if (tlane_virtual_id != clane_virtual_id &&
      virtual_lane_mgr->lc_map_decision(clane) !=
          virtual_lane_mgr->lc_map_decision(tlane)) {
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
  auto target_reference_path =
      reference_path_mgr->get_reference_path_by_lane(tlane_virtual_id);
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
  return lc_change_finish;
}

bool ScenarioStateMachine::check_lc_back_finish(RequestType direction) {
  if (!lc_lane_mgr_->has_origin_lane() || !lc_lane_mgr_->has_fix_lane()) {
    return false;
  }
  assert(lc_lane_mgr_->olane_virtual_id() == lc_lane_mgr_->flane_virtual_id());

  // Find the half lane width
  std::shared_ptr<VirtualLaneManager> virtual_lane_mgr =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  auto flane = virtual_lane_mgr->get_lane_with_virtual_id(
      lc_lane_mgr_->flane_virtual_id());
  double half_lane_width =
      1.6;  // HACK  TODO(Rui):get real fix_lane->width() / 2.
  double ego_width =
      session_->mutable_vehicle_config_context()->get_vehicle_param().width;
  double dist_threshold = half_lane_width - 0.5 * ego_width - 0.1;
  double angle_threshold = 0.01;

  std::shared_ptr<ReferencePathManager> reference_path_mgr =
      session_->mutable_environmental_model()->get_reference_path_manager();
  auto fix_reference_path = reference_path_mgr->get_reference_path_by_lane(
      lc_lane_mgr_->flane_virtual_id());
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

void ScenarioStateMachine::generate_state_machine_output(
    const LaneChangeStageInfo &lc_info) {
  auto &state_machine_output =
      session_->mutable_planning_context()
          ->mutable_lat_behavior_state_machine_output();

  // context->mutable_planning_status()->lane_status.change_lane.target_gap_obs
  // = lc_info.gap;

  state_machine_output.scenario = scenario_;
  // state_machine_output.curr_state = target_state;
  // state_machine_output.state_name = target_state_name;
  state_machine_output.lc_back_reason = lc_info.lc_back_reason;
  state_machine_output.lc_invalid_reason = lc_info.lc_invalid_reason;
  state_machine_output.behavior_suspend = behavior_suspend_;  // lateral suspend
  state_machine_output.suspend_obs.assign(
      suspend_obs_.begin(), suspend_obs_.end());  // lateral obstacles
  state_machine_output.must_change_lane = must_change_lane_;

  // if (target_state == type2int<RoadState::None>::value) {
  //   state_machine_output.turn_light = 0;
  // } else {
  if (map_turn_signal_ != NO_CHANGE) {
    state_machine_output.turn_light = map_turn_signal_;
  } else if (lc_req_mgr_->turn_signal() > 0) {
    state_machine_output.turn_light = lc_req_mgr_->turn_signal();
  } else {
    state_machine_output.turn_light = 0;
  }
  // }
  state_machine_output.map_turn_light = map_turn_signal_;
  state_machine_output.accident_back = lc_info.accident_back;
  state_machine_output.accident_ahead = accident_ahead_;
  state_machine_output.close_to_accident = close_to_accident_;
  state_machine_output.should_premove = lc_info.should_premove;
  state_machine_output.should_suspend = should_suspend_;
  state_machine_output.lc_pause = lc_info.lc_pause;
  state_machine_output.lc_pause_id = lc_info.lc_pause_id;
  state_machine_output.tr_pause_l = lc_info.tr_pause_l;
  state_machine_output.tr_pause_s = lc_info.tr_pause_s;
  state_machine_output.disable_l = false;
  state_machine_output.disable_r = false;
  state_machine_output.enable_l = false;
  state_machine_output.enable_r = false;
  state_machine_output.need_clear_lb_car = true;
  state_machine_output.enable_lb = false;
  state_machine_output.lc_request = lc_req_mgr_->request();
  state_machine_output.lc_request_source = lc_req_mgr_->request_source();
  state_machine_output.act_request_source = lc_req_mgr_->act_request_source();
  state_machine_output.lc_turn_light = lc_req_mgr_->turn_signal();

  state_machine_output.premovel = false;
  state_machine_output.premover = false;
  state_machine_output.premove_dist = 0.;

  // TODO(Rui):待加入object_selector
  //  state_machine_output.left_is_faster = object_selector_->left_is_faster();
  //  state_machine_output.right_is_faster =
  //  object_selector_->right_is_faster();

  // state_machine_output.neg_left_lb_car = object_selector_->neg_left_lb_car();
  // state_machine_output.neg_right_lb_car =
  // object_selector_->neg_right_lb_car(); state_machine_output.neg_left_alc_car
  // = object_selector_->neg_left_alc_car();
  // state_machine_output.neg_right_alc_car =
  // object_selector_->neg_right_alc_car();

  // state_machine_output.left_lb_car = object_selector_->left_lb_car();
  // state_machine_output.left_alc_car = object_selector_->left_alc_car();
  // state_machine_output.right_lb_car = object_selector_->right_lb_car();
  // state_machine_output.right_alc_car = object_selector_->right_alc_car();

  state_machine_output.enable_alc_car_protection = false;
  // state_machine_output.alc_cars_ = alc_cars_; //TODO(Rui):待添加
  state_machine_output.near_cars_target = near_cars_target_;
  state_machine_output.near_cars_origin = near_cars_origin_;
  state_machine_output.lc_invalid_track = lc_invalid_track_;
  state_machine_output.lc_back_track = lc_back_track_;
}

}  // namespace planning
