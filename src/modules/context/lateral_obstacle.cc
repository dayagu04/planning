#include "lateral_obstacle.h"

#include "common.h"
#include "debug_info_log.h"
#include "environment_model_debug_info.pb.h"
#include "environmental_model.h"
#include "ifly_time.h"
#include "planning_context.h"
#include "vehicle_config_context.h"
#include "virtual_lane_manager.h"
#include "obstacle_manager.h"

namespace planning {

LateralObstacle::LateralObstacle(const EgoPlanningConfigBuilder *config_builder,
                                 planning::framework::Session *session)
    : session_(session) {
  // TODO: add config
  config_ = config_builder->cast<LateralObstacleConfig>();
  // maintainer_ =
  //     std::make_shared<planning::TrackletMaintainer>(session_, config_);
  double curr_time = IflyTime::Now_ms();
  warning_timer_[0] = curr_time;
  warning_timer_[1] = curr_time - 1;
  warning_timer_[2] = curr_time - 2;
  warning_timer_[3] = curr_time - 3;
  warning_timer_[4] = curr_time;
}

LateralObstacle::~LateralObstacle() { }

void LateralObstacle::SetConfig(
    const EgoPlanningConfigBuilder *config_builder) {
  config_ = config_builder->cast<LateralObstacleConfig>();
  // maintainer_->SetConfig(config_);
}

bool LateralObstacle::update() {
  update_sensors(
      session_->environmental_model().get_ego_state_manager(),
      session_->environmental_model().get_prediction_info(), false,
      false);  // TODO session_->environmental_model().get_hdmap_valid()
  return true;
}

bool LateralObstacle::update_sensors(
    const std::shared_ptr<EgoStateManager> &ego_state,
    const std::vector<PredictionObject> &predictions, bool isRedLightStop,
    bool hdmap_valid) {
  double fusion_timeout = 0.5;
  double curr_time = IflyTime::Now_ms();
  // std::vector<std::shared_ptr<FrenetObstacle>> tracked_objects;

  if (prediction_update_ || !hdmap_valid) {
    // maintainer_->apply_update(ego_state, predictions, tracked_objects,
    //                           lead_cars_, isRedLightStop, hdmap_valid);
    // LateralObstacleDecision(tracked_objects);

    update_tracks();
    update_lead_info();
    // is_static_avoid_scene_ = maintainer_->is_static_scene();
    prediction_update_ = false;
    fvf_time_ = curr_time;
    svf_time_ = curr_time;

    fvf_dead_ = false;
    svf_dead_ = false;
  } else if (curr_time - fvf_time_ > fusion_timeout) {
    front_tracks_.clear();
    front_tracks_copy_.clear();
    side_tracks_.clear();
    side_tracks_l_.clear();
    side_tracks_r_.clear();

    fvf_dead_ = true;
    svf_dead_ = true;

    ILOG_ERROR << "[LateralObstacle::update_sensors] Fusion lagging too large";

    double abs_time = IflyTime::Now_ms();
    if (abs_time - warning_timer_[2] > 5.0) {
      ILOG_ERROR << "[LateralObstacle::update_sensors] frontview fusion unavailable";
      warning_timer_[2] = abs_time;
    }

    if (abs_time - warning_timer_[1] > 5.0) {
      ILOG_ERROR << "[LateralObstacle::update_sensors] sideview fusion unavailable";
      warning_timer_[1] = abs_time;
    }
  }
  return true;
}

void LateralObstacle::update_lead_info() {
  double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto &obstacle_manager = session_->environmental_model().get_obstacle_manager();
  const auto &current_lane = virtual_lane_manager->get_current_lane();
  const auto &frenet_obstacles_map = current_lane->get_reference_path()->get_obstacles_map();
  const double v_ego = session_->environmental_model().get_ego_state_manager()->ego_v();

  auto extra_obstacle_info_map = extra_obstacle_info_map_;
  for (const auto& extra_obstacle_info : extra_obstacle_info_map) {
    if (frenet_obstacles_map.find(extra_obstacle_info.first) == frenet_obstacles_map.end() ||
        !frenet_obstacles_map.at(extra_obstacle_info.first)->b_frenet_valid()) {
      extra_obstacle_info_map_.erase(extra_obstacle_info.first);
    }
  }

  for (auto &item : front_tracks_) {
    ExtraObstacleInfo extra_obstacle_info;
    if (extra_obstacle_info_map_.find(item->id()) == extra_obstacle_info_map_.end()) {
      extra_obstacle_info.oncoming = item->frenet_velocity_s() < -3.0;
      extra_obstacle_info_map_[item->id()] = extra_obstacle_info;
    } else {
      extra_obstacle_info = extra_obstacle_info_map_.at(item->id());

      extra_obstacle_info.last_recv_time = extra_obstacle_info.timestamp;
      extra_obstacle_info.timestamp = item->obstacle()->timestamp();
      extra_obstacle_info.oncoming = item->frenet_velocity_s() < -3.0;
    }

    LOG_DEBUG("----is_potential_lead_one-----\n");
    double gap = (extra_obstacle_info.last_recv_time == 0.0)
                    ? planning_cycle_time
                    : std::max((extra_obstacle_info.timestamp - extra_obstacle_info.last_recv_time),
                                planning_cycle_time);
    LOG_DEBUG("the gap is : [%f]ms \n", gap);
    std::array<double, 5> xp1{1.5, 5.0, 10.0, 40.0, 60.0 + v_ego / 1.2};
    std::array<double, 5> fp1{0.3, 0.4, 0.3, 0.25, 0.0};
    double t_lookahead = interp(item->d_s_rel(), xp1, fp1);

    std::array<double, 3> xp2{1.5, 5.0, 10.0};
    std::array<double, 3> fp2{-0.85, -0.9, -0.85};
    double max_d_offset = interp(item->d_s_rel(), xp2, fp2);

    if (std::fabs(item->d_path_pos()) < 3.0 && item->frenet_velocity_l() < -0.2) {
      extra_obstacle_info.lat_coeff = std::min(extra_obstacle_info.lat_coeff * 1.035, 1.5);
    } else {
      extra_obstacle_info.lat_coeff = std::max(extra_obstacle_info.lat_coeff * 0.9, 1.0);
    }

    double lat_corr =
        std::max(max_d_offset, std::min(0.0, t_lookahead * item->frenet_velocity_l())) *
        extra_obstacle_info.lat_coeff;
    LOG_DEBUG("max_d_offset is: [%f], t_lookahead: [%f], lat_corr is: [%f]\n",
              max_d_offset, t_lookahead, lat_corr);
    if (extra_obstacle_info.oncoming && item->d_s_rel() >= 0) {
      lat_corr = 0.0;
    }

    double lead_d_path_thr;
    std::array<double, 5> xp3{0, 5, 40, 80, 120};
    std::array<double, 5> fp3{1.28, 1.28, 1.21, 1.19, 1.1};
    double result = interp(item->d_s_rel(), xp3, fp3);

    // hysteresis
    const double lead_hysteresis = 1.1;
    if (extra_obstacle_info.is_lead) {
      result = result * lead_hysteresis;
    }

    // increase threshold for vru
    const double lead_vru = 1.2;
    if (item->obstacle()->is_VRU()) {
      result = result * lead_vru;
    }

    double d_path = std::max(item->d_path_pos() + lat_corr, 0.0);

    if (item->frenet_s() > 2.0) {
      lead_d_path_thr = result;
    } else if (item->frenet_s() <= 2.0 && item->is_static()) {
      lead_d_path_thr = 1.1;
    } else {
      lead_d_path_thr = 1.2;
    }

    if (extra_obstacle_info.oncoming && item->d_s_rel() > 50) {
      lead_d_path_thr = 0.3;
    }
    LOG_DEBUG("lead_d_path_thr is: [%f]\n", lead_d_path_thr);
    LOG_DEBUG("the gap is : [%f]ms \n", gap);

    if (d_path < lead_d_path_thr && item->d_s_rel() > 0.0) {
      extra_obstacle_info.leadone_confidence_cnt =
          std::min(extra_obstacle_info.leadone_confidence_cnt + gap, 10 * planning_cycle_time);
    } else {
      LOG_DEBUG("gap is : [%f] \n", gap);
      int count = (int)((gap + 0.01) / planning_cycle_time);
      extra_obstacle_info.leadone_confidence_cnt = std::max(
          extra_obstacle_info.leadone_confidence_cnt - 2 * count * planning_cycle_time, 0.0);
      LOG_DEBUG("leadone_confidence_cnt is : [%f] \n",
                extra_obstacle_info.leadone_confidence_cnt);
    }

    std::array<double, 5> xp4{0, 30, 60, 90, 120};
    std::array<double, 5> fp4{1, 1, 2, 5, 5};
    double lead_confidence_thrshld = 1.0;
    lead_confidence_thrshld = interp(item->d_s_rel(), xp4, fp4);
    // Hack: if the cone bucket emergency lane change is triggered
    // set lead_confidence_thrshld = 1;
    const auto lc_request = session_->planning_context()
                                .lane_change_decider_output()
                                .lc_request_source;
    if (lc_request == CONE_REQUEST &&
        item->type() == iflyauto::OBJECT_TYPE_TRAFFIC_CONE) {
      lead_confidence_thrshld = 1.0;
    }
    LOG_DEBUG("lead_confidence_thrshld is : [%f]\n", lead_confidence_thrshld);
    extra_obstacle_info.is_lead = extra_obstacle_info.leadone_confidence_cnt >=
                  lead_confidence_thrshld * planning_cycle_time;
    LOG_DEBUG("item.is_lead: [%d]\n", extra_obstacle_info.is_lead);


    {
      LOG_DEBUG("----fill_possibility_of_cutin-----\n");
      double ttc = std::max(item->d_path() - 1.5, 0.0) / std::max(-item->frenet_velocity_l(), 0.01);
      ttc = std::min(15.0, ttc);

      double new_ttc;
      if (extra_obstacle_info.last_ttc == std::numeric_limits<double>::min()) {
        new_ttc = ttc;
        extra_obstacle_info.last_ttc = ttc;
      } else {
        double ttc_decay_rate = 0.5;
        new_ttc = ttc * (1.0 - ttc_decay_rate) + extra_obstacle_info.last_ttc * ttc_decay_rate;
      }
      double lower_dist_threshold = (item->frenet_velocity_s() > -1.0) ? -10.0 : -5.0;
      double upper_dist_threshold =
          20 + 2 * std::pow(std::min(item->rel_v(), 0.0), 2);

      double is_need_consider = (item->frenet_velocity_l() < -0.2) && (new_ttc < 10.0);
      double temp = item->d_s_rel() + new_ttc * item->rel_v();
      double is_in_range =
          (temp > lower_dist_threshold && temp < upper_dist_threshold);

      std::array<double, 5> xp5{1.5, 5.0, 10.0, 40.0, 60.0 + v_ego / 1.2};
      std::array<double, 5> fp5{0.8, 1.0, 0.8, 0.7, 0.0};
      t_lookahead = interp(item->d_s_rel(), xp5, fp5);
      lat_corr = std::max(max_d_offset, std::min(0.0, t_lookahead * item->frenet_velocity_l())) *
                extra_obstacle_info.lat_coeff;
      if (extra_obstacle_info.oncoming && item->d_s_rel() >= 0) {
        lat_corr = 0.0;
      }

      d_path = std::max(item->d_path_pos() + lat_corr, 0.0);
      if (d_path < lead_d_path_thr && item->d_s_rel() > 0.0) {
        extra_obstacle_info.cutin_confidence_cnt =
            std::min(extra_obstacle_info.cutin_confidence_cnt + gap, 50 * planning_cycle_time);
      } else {
        int count = (int)((gap + 0.01) / planning_cycle_time);
        extra_obstacle_info.cutin_confidence_cnt = std::max(
            extra_obstacle_info.cutin_confidence_cnt - 10 * count * planning_cycle_time, 0.0);
        LOG_DEBUG("!!!!!!!cutin_confidence_cnt is : [%f] \n",
                  extra_obstacle_info.cutin_confidence_cnt);
      }

      std::array<double, 5> xp4{0, 30, 60, 90, 120};
      std::array<double, 5> fp4{1, 1, 1, 5, 10};
      std::array<double, 5> fp4_2{1, 1, 1, 1, 1};
      double cutin_confidence_cnt = 1.0;
      if (item->type() == 1) {
        cutin_confidence_cnt = interp(item->d_s_rel(), xp4, fp4_2);
      } else {
        cutin_confidence_cnt = interp(item->d_s_rel(), xp4, fp4);
      }
      LOG_DEBUG("cutin_confidence_cnt is : [%f]\n", cutin_confidence_cnt);
      if (is_need_consider && is_in_range) {
        if (extra_obstacle_info.cutin_confidence_cnt >=
            cutin_confidence_cnt * planning_cycle_time) {
          extra_obstacle_info.cutinp =
              std::max(0.6, extra_obstacle_info.cutinp - gap * std::max(item->frenet_velocity_l(), -1.0) /
                                              std::max(item->d_path(), 0.01));
        } else {
          extra_obstacle_info.cutinp = extra_obstacle_info.cutinp - std::max(0.1, item->frenet_velocity_l() / 3);
        }
      } else {
        extra_obstacle_info.cutinp = extra_obstacle_info.cutinp - std::max(0.1, item->frenet_velocity_l() / 3);
      }
      extra_obstacle_info.last_ttc = new_ttc;
      extra_obstacle_info.cutinp = std::max(0.0, std::min(1.0, extra_obstacle_info.cutinp));
      if (extra_obstacle_info.oncoming) {
        extra_obstacle_info.cutinp = 0.0;
      }
      Obstacle *obstacle= obstacle_manager->find_obstacle(item->id());
      if (obstacle != nullptr) {
        obstacle->set_cutin_prob(extra_obstacle_info.cutinp);
      }
      extra_obstacle_info_map_[item->id()] = std::move(extra_obstacle_info);
    }
  }
  select_lead_cars();
}

void LateralObstacle::select_lead_cars() {
  std::shared_ptr<FrenetObstacle> lead_one = nullptr;

  for (auto item : tracks_map_) {

    if (extra_obstacle_info_map_.find(item.first) == extra_obstacle_info_map_.end()) {
      continue;
    }
    if (extra_obstacle_info_map_.at(item.first).is_lead == false) {
      continue;
    }

    if (lead_one == nullptr || item.second->d_s_rel() < lead_one->d_s_rel()) {
      lead_one = item.second;
    }
  }

  lead_one_ = lead_one;
}

void LateralObstacle::LateralObstacleDecision(
    const std::vector<std::shared_ptr<FrenetObstacle>> &tracked_objects) {
  auto config_builder =
      session_->environmental_model().highway_config_builder();
  LateralObstacleDeciderConfig config =
      config_builder->cast<LateralObstacleDeciderConfig>();
  const auto ego_l = DebugInfoManager::GetInstance()
                         .GetDebugInfoPb()
                         ->environment_model_info()
                         .ego_l();
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto ego_car_width = vehicle_param.width;
  const auto ego_car_length = vehicle_param.length;
  const auto &lateral_output =
      session_->planning_context().lateral_behavior_planner_output();
  auto lane_width = lateral_output.flane_width;
  double ref_dis = 1;
  double avoid_front_buffer = 0.0;
  double expand_vel =
      interp(session_->environmental_model().get_ego_state_manager()->ego_v(),
             config.expand_ego_vel, config.expand_obs_rel_vel);
  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto environment_model_debug_info =
      planning_debug_data->mutable_environment_model_info();

}

void LateralObstacle::update_tracks() {
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto &current_lane = virtual_lane_manager->get_current_lane();

  front_tracks_.clear();
  front_tracks_copy_.clear();
  front_tracks_l_.clear();
  front_tracks_r_.clear();
  side_tracks_.clear();
  side_tracks_l_.clear();
  side_tracks_r_.clear();
  // all_tracks_ = tracked_objects;
  tracks_map_.clear();

  if (current_lane == nullptr) {
    return;
  }

  const double ego_l = current_lane->get_reference_path()->get_frenet_ego_state().l();
  const auto &frenet_obstacles_map = current_lane->get_reference_path()->get_obstacles_map();

  for (const auto item : frenet_obstacles_map) {
    if (not item.second->b_frenet_valid()) {
      continue;
    }
    const auto frenet_obstacle = item.second;
    tracks_map_[item.first] = frenet_obstacle;
    if (frenet_obstacle->d_s_rel() > 0.0) {
      auto it = front_tracks_.begin();
      while (it != front_tracks_.end() && (*it)->d_s_rel() < frenet_obstacle->d_s_rel()) {
        ++it;
      }

      if (it != front_tracks_.end()) {
        front_tracks_.insert(it, frenet_obstacle);
      } else {
        front_tracks_.emplace_back(frenet_obstacle);
      }
    } else {
      auto it = side_tracks_.begin();
      while (it != side_tracks_.end() && (*it)->d_s_rel() > frenet_obstacle->d_s_rel()) {
        ++it;
      }

      if (it != side_tracks_.end()) {
        side_tracks_.insert(it, frenet_obstacle);
      } else {
        side_tracks_.emplace_back(frenet_obstacle);
      }
    }
  }

  front_tracks_copy_ = front_tracks_;

  for (const auto &tr : front_tracks_) {
    if (tr->obstacle()->y_relative_center() >= 0) {
      front_tracks_l_.emplace_back(tr);
    } else {
      front_tracks_r_.push_back(tr);
    }
  }

  for (const auto &tr : side_tracks_) {
    if (tr->obstacle()->y_relative_center() >= 0) {
      side_tracks_l_.emplace_back(tr);
    } else {
      side_tracks_r_.push_back(tr);
    }
  }
}

bool LateralObstacle::find_track(int track_id, std::shared_ptr<FrenetObstacle> &dest) {
  for (auto &tr : front_tracks_) {
    if (tr->id() == track_id) {
      dest = tr;
      return true;
    }
  }

  for (auto &tr : side_tracks_) {
    if (tr->id() == track_id) {
      dest = tr;
      return true;
    }
  }

  return false;
}

LaneTracksManager::LaneTracksManager(LateralObstacle &lateral_obstacle,
                                     VirtualLaneManager &virtual_lane_mgr,
                                     planning::framework::Session *session)
    : lateral_obstacle_(lateral_obstacle),
      virtual_lane_mgr_(virtual_lane_mgr),
      session_(session) {}

std::vector<std::shared_ptr<FrenetObstacle>> *LaneTracksManager::get_lane_tracks(
    int virtual_id, TrackType track_type) {
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto &current_lane = virtual_lane_manager->get_current_lane();
  const auto &left_lane = virtual_lane_manager->get_left_lane();
  const auto &right_lane = virtual_lane_manager->get_right_lane();

  if (current_lane != nullptr && current_lane->get_virtual_id() == virtual_id) {
    if (track_type == TrackType::FRONT_TRACK) {
      return &front_tracks_clane_;
    } else if (track_type == TrackType::SIDE_TRACK) {
      return &side_tracks_clane_;
    }
  }

  if (left_lane != nullptr && left_lane->get_virtual_id() == virtual_id) {
    if (track_type == TrackType::FRONT_TRACK) {
      return &front_tracks_llane_;
    } else if (track_type == TrackType::SIDE_TRACK) {
      return &side_tracks_llane_;
    }
  }

  if (right_lane != nullptr && right_lane->get_virtual_id() == virtual_id) {
    if (track_type == TrackType::FRONT_TRACK) {
      return &front_tracks_rlane_;
    } else if (track_type == TrackType::SIDE_TRACK) {
      return &side_tracks_rlane_;
    }
  }

  return nullptr;
}

void LaneTracksManager::update_lane_tracks() {
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto &current_lane = virtual_lane_manager->get_current_lane();
  const auto &left_lane = virtual_lane_manager->get_left_lane();
  const auto &right_lane = virtual_lane_manager->get_right_lane();

  reset();

  // const auto tracks = lateral_obstacle_.all_tracks();
  // std::unordered_map<int, std::shared_ptr<FrenetObstacle>> tracks_map;
  // for (auto track : tracks) {
  //   tracks_map[track.track_id] = track;
  // }

  if (current_lane == nullptr) {
    return;
  }

  const auto frenet_obstacles_map = current_lane->get_reference_path()->get_obstacles_map();

  auto obstacles_id =
      current_lane->get_reference_path()->get_lane_obstacles_ids();

  for (int obstacle_id : obstacles_id) {
    if (frenet_obstacles_map.find(obstacle_id) != frenet_obstacles_map.end()) {
      const auto frenet_obstacle = frenet_obstacles_map.at(obstacle_id);
      if (frenet_obstacle->b_frenet_valid()) {
        if (frenet_obstacle->d_s_rel() > 0) {
          front_tracks_clane_.emplace_back(frenet_obstacle);
        } else {
          side_tracks_clane_.emplace_back(frenet_obstacle);
        }
      }
    }
  }
    // for (const auto & frenet_obstacle : frenet_obstacles) {
    //   if (frenet_obstacle->b_frenet_valid()) {
    //     if (frenet_obstacle->d_s_rel() > 0) {
    //       front_tracks_clane_.emplace_back(frenet_obstacle);
    //     } else {
    //       side_tracks_clane_.emplace_back(frenet_obstacle);
    //     }
    //   }
    // }


  if (left_lane != nullptr) {
    auto obstacles_id =
        left_lane->get_reference_path()->get_lane_obstacles_ids();
    for (auto obstacle_id : obstacles_id) {
      if (frenet_obstacles_map.find(obstacle_id) != frenet_obstacles_map.end()) {
        const auto frenet_obstacle = frenet_obstacles_map.at(obstacle_id);
        if (frenet_obstacle->b_frenet_valid()) {
          if (frenet_obstacle->d_s_rel() > 0) {
            front_tracks_llane_.emplace_back(frenet_obstacle);
          } else {
            side_tracks_llane_.emplace_back(frenet_obstacle);
          }
        }
      }
    }
  }

  if (right_lane != nullptr) {
    auto obstacles_id =
        right_lane->get_reference_path()->get_lane_obstacles_ids();
    for (auto obstacle_id : obstacles_id) {
      if (frenet_obstacles_map.find(obstacle_id) != frenet_obstacles_map.end()) {
        const auto frenet_obstacle = frenet_obstacles_map.at(obstacle_id);
        if (frenet_obstacle->b_frenet_valid()) {
          if (frenet_obstacle->d_s_rel() > 0) {
            front_tracks_rlane_.emplace_back(frenet_obstacle);
          } else {
            side_tracks_rlane_.emplace_back(frenet_obstacle);
          }
        }
      }
    }
  }
}

void LaneTracksManager::reset() {
  front_tracks_clane_.clear();
  front_tracks_llane_.clear();
  front_tracks_rlane_.clear();
  side_tracks_clane_.clear();
  side_tracks_llane_.clear();
  side_tracks_rlane_.clear();
}

}  // namespace planning
