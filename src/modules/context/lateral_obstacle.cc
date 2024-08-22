#include "lateral_obstacle.h"

#include "common.h"
#include "debug_info_log.h"
#include "environment_model_debug_info.pb.h"
#include "environmental_model.h"
#include "ifly_time.h"
#include "planning_context.h"
#include "vehicle_config_context.h"
#include "virtual_lane_manager.h"

namespace planning {

LateralObstacle::LateralObstacle(const EgoPlanningConfigBuilder *config_builder,
                                 planning::framework::Session *session)
    : session_(session) {
  // TODO: add config
  config_ = config_builder->cast<LateralObstacleConfig>();
  maintainer_ =
      std::make_shared<planning::TrackletMaintainer>(session_, config_);
  double curr_time = IflyTime::Now_ms();
  warning_timer_[0] = curr_time;
  warning_timer_[1] = curr_time - 1;
  warning_timer_[2] = curr_time - 2;
  warning_timer_[3] = curr_time - 3;
  warning_timer_[4] = curr_time;

  lead_cars_.lead_one = nullptr;
  lead_cars_.lead_two = nullptr;
  lead_cars_.temp_lead_one = nullptr;
  lead_cars_.temp_lead_two = nullptr;
}

LateralObstacle::~LateralObstacle() { lead_cars_.clear(); }

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
  std::vector<TrackedObject> tracked_objects;

  if (prediction_update_ || !hdmap_valid) {
    maintainer_->apply_update(ego_state, predictions, tracked_objects,
                              lead_cars_, isRedLightStop, hdmap_valid);
    LateralObstacleDecision(tracked_objects);
    update_tracks(tracked_objects);
    is_static_avoid_scene_ = maintainer_->is_static_scene();
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

    lead_cars_.clear();
    fvf_dead_ = true;
    svf_dead_ = true;

    LOG_ERROR("[LateralObstacle::update_sensors] Fusion lagging too large");

    double abs_time = IflyTime::Now_ms();
    if (abs_time - warning_timer_[2] > 5.0) {
      LOG_ERROR(
          "[LateralObstacle::update_sensors] frontview fusion unavailable");
      warning_timer_[2] = abs_time;
    }

    if (abs_time - warning_timer_[1] > 5.0) {
      LOG_ERROR(
          "[LateralObstacle::update_sensors] sideview fusion unavailable");
      warning_timer_[1] = abs_time;
    }
  }

  return true;
}

void LateralObstacle::LateralObstacleDecision(
    const std::vector<TrackedObject> &tracked_objects) {
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

  lat_obstacle_decision_.clear();
  for (auto &item : tracked_objects) {
    if ((!(item.fusion_source & OBSTACLE_SOURCE_CAMERA)) ||
        !item.frenet_transform_valid) {
      continue;
    }

    bool lat_overlap = fabs(ego_l - item.l) < (ego_car_width + item.width) / 2;
    // 前方车辆
    if (item.is_avd_car) {
      if (item.d_max_cpath > 0 && item.d_min_cpath < 0) {
        lat_obstacle_decision_[item.track_id] = LatObstacleDecisionType::IGNORE;
      } else if (item.d_max_cpath < 0) {
        lat_obstacle_decision_[item.track_id] = LatObstacleDecisionType::LEFT;
      } else if (item.d_min_cpath > 0) {
        lat_obstacle_decision_[item.track_id] = LatObstacleDecisionType::RIGHT;
      }
    } else if (item.d_rel > 0) {
      if (item.d_max_cpath < 0 &&
          std::fabs(item.d_max_cpath) > lane_width * 0.5) {
        lat_obstacle_decision_[item.track_id] = LatObstacleDecisionType::LEFT;
      } else if (item.d_min_cpath > lane_width * 0.5) {
        lat_obstacle_decision_[item.track_id] = LatObstacleDecisionType::RIGHT;
      } else {
        lat_obstacle_decision_[item.track_id] = LatObstacleDecisionType::IGNORE;
      }
      // 平行车辆
    } else if (item.d_rel <= 0 &&
               item.d_rel > -(ego_car_length + item.length)) {
      if (ego_l < item.l) {
        lat_obstacle_decision_[item.track_id] = LatObstacleDecisionType::RIGHT;
      } else {
        lat_obstacle_decision_[item.track_id] = LatObstacleDecisionType::LEFT;
      }
      // 后方车辆
    } else {
      if (item.d_max_cpath < 0 && !lat_overlap && item.d_max_cpath < -ref_dis) {
        lat_obstacle_decision_[item.track_id] = LatObstacleDecisionType::LEFT;
      } else if (item.d_min_cpath > 0 && !lat_overlap &&
                 item.d_min_cpath > ref_dis) {
        lat_obstacle_decision_[item.track_id] = LatObstacleDecisionType::RIGHT;
      } else {
        lat_obstacle_decision_[item.track_id] = LatObstacleDecisionType::IGNORE;
      }
    }
  }
}

void LateralObstacle::update_tracks(
    const std::vector<TrackedObject> &tracked_objects) {
  front_tracks_.clear();
  front_tracks_copy_.clear();
  front_tracks_l_.clear();
  front_tracks_r_.clear();
  side_tracks_.clear();
  side_tracks_l_.clear();
  side_tracks_r_.clear();
  all_tracks_ = tracked_objects;
  for (auto &tr : tracked_objects) {
    if (tr.d_rel >= 0.0) {
      auto it = front_tracks_.begin();
      while (it != front_tracks_.end() && it->d_rel < tr.d_rel) {
        ++it;
      }

      if (it != front_tracks_.end()) {
        front_tracks_.insert(it, tr);
      } else {
        front_tracks_.push_back(tr);
      }
    } else {
      auto it = side_tracks_.begin();
      while (it != side_tracks_.end() && it->d_rel > tr.d_rel) {
        ++it;
      }

      if (it != side_tracks_.end()) {
        side_tracks_.insert(it, tr);
      } else {
        side_tracks_.push_back(tr);
      }
    }
  }

  front_tracks_copy_ = front_tracks_;

  for (auto &tr : front_tracks_) {
    if (tr.y_center_rel >= 0) {
      front_tracks_l_.push_back(tr);
    } else {
      front_tracks_r_.push_back(tr);
    }
  }

  for (auto &tr : side_tracks_) {
    if (tr.y_center_rel >= 0) {
      side_tracks_l_.push_back(tr);
    } else {
      side_tracks_r_.push_back(tr);
    }
  }
}

bool LateralObstacle::find_track(int track_id, TrackedObject &dest) {
  for (auto &tr : front_tracks_) {
    if (tr.track_id == track_id) {
      dest = tr;
      return true;
    }
  }

  for (auto &tr : side_tracks_) {
    if (tr.track_id == track_id) {
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

std::vector<TrackedObject> *LaneTracksManager::get_lane_tracks(
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

  const auto tracks = lateral_obstacle_.all_tracks();
  std::unordered_map<int, TrackedObject> tracks_map;
  for (auto track : tracks) {
    tracks_map[track.track_id] = track;
  }

  if (current_lane != nullptr) {
    auto obstacles_id =
        current_lane->get_reference_path()->get_lane_obstacles_ids();
    for (auto obstacle_id : obstacles_id) {
      if (tracks_map.find(obstacle_id) != tracks_map.end()) {
        auto d_rel = tracks_map[obstacle_id].d_rel;
        if (d_rel > 0) {
          front_tracks_clane_.emplace_back(tracks_map[obstacle_id]);
        } else {
          side_tracks_clane_.emplace_back(tracks_map[obstacle_id]);
        }
      }
    }
  }

  if (left_lane != nullptr) {
    auto obstacles_id =
        left_lane->get_reference_path()->get_lane_obstacles_ids();
    for (auto obstacle_id : obstacles_id) {
      if (tracks_map.find(obstacle_id) != tracks_map.end()) {
        auto d_rel = tracks_map[obstacle_id].d_rel;
        if (d_rel > 0) {
          front_tracks_llane_.emplace_back(tracks_map[obstacle_id]);
        } else {
          side_tracks_llane_.emplace_back(tracks_map[obstacle_id]);
        }
      }
    }
  }

  if (right_lane != nullptr) {
    auto obstacles_id =
        right_lane->get_reference_path()->get_lane_obstacles_ids();
    for (auto obstacle_id : obstacles_id) {
      if (tracks_map.find(obstacle_id) != tracks_map.end()) {
        auto d_rel = tracks_map[obstacle_id].d_rel;
        if (d_rel > 0) {
          front_tracks_rlane_.emplace_back(tracks_map[obstacle_id]);
        } else {
          side_tracks_rlane_.emplace_back(tracks_map[obstacle_id]);
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
