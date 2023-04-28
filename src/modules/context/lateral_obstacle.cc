#include "src/modules/context/lateral_obstacle.h"
#include "src/modules/context/virtual_lane_manager.h"
#include "src/common/ifly_time.h"
#include "src/modules/common/common.h"

namespace planning {

LateralObstacle::LateralObstacle(const EgoPlanningConfigBuilder *config_builder,
                                 planning::framework::Session *session)
    : session_(session) {
  // TODO: add config
  config_ = config_builder->cast<LateralObstacleConfig>();
  maintainer_ = std::make_shared<planning::TrackletMaintainer>(session_);
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
    maintainer_->apply_update(*ego_state, predictions, tracked_objects,
                              lead_cars_, isRedLightStop, hdmap_valid);
    update_tracks(tracked_objects);

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

// LaneTracksManager::LaneTracksManager(LateralObstacle &lateral_obstacle,
//                                      VirtualLaneManager &virtual_lane_mgr)
//     : lateral_obstacle_(lateral_obstacle),
//       virtual_lane_mgr_(virtual_lane_mgr) {}

// double LaneTracksManager::get_drel_close(int side) {
//   double d_rel_close = 150;
//   double fvf_drel_confident = 120;
//   if (side == 0) {
//     std::vector<TrackedObject> *front_tracks =
//         get_lane_tracks(CURRENT_LANE, FRONT_TRACK);
//     if (front_tracks == nullptr) {
//       MSD_LOG(ERROR,
//               "[LaneTracksManager::get_drel_close] front tracks is null");
//       return d_rel_close;
//     }

//     for (auto &tr : *front_tracks) {
//       if (tr.d_rel < fvf_drel_confident) {
//         d_rel_close = tr.d_rel;
//         break;
//       }
//     }
//   } else {
//     MSD_LOG(ERROR, "[LaneTracksManager::get_drel_close] Illegal side
//     argument");
//   }

//   return d_rel_close;
// }

// double LaneTracksManager::get_vrel_close(int side, int status) {
//   double v_rel_close = 15.;
//   double fvf_drel_confident = 120.;
//   std::vector<TrackedObject> *front_tracks = nullptr;

//   auto &map_info = map_info_mgr_.get_map_info();
//   auto &flane = virtual_lane_mgr_.get_fix_lane();
//   auto &tlane = virtual_lane_mgr_.get_target_lane();
//   auto &lead_cars = lateral_obstacle_.get_lead_cars();

//   int flane_left_id = flane.rids()[0];
//   int tlane_left_id = tlane.rids()[0];
//   if (flane_left_id == kInvalidRelativeId) {
//     flane_left_id = -99;
//   }
//   if (tlane_left_id == kInvalidRelativeId) {
//     tlane_left_id = -99;
//   }
//   auto traffic_light_direction = map_info.traffic_light_direction();
//   bool curr_direct_exist =
//       (map_info.current_lane_marks() & traffic_light_direction) ||
//       traffic_light_direction == MSD_DIRECTION_UNKNOWN ||
//       (map_info.current_lane_marks() == MSD_DIRECTION_UNKNOWN &&
//       map_info.current_lane_type() == MSD_LANE_TYPE_NORMAL);

//   bool left_direct_exist =
//       (map_info.left_lane_marks() & traffic_light_direction) ||
//       traffic_light_direction == MSD_DIRECTION_UNKNOWN ||
//       (map_info.left_lane_marks() == MSD_DIRECTION_UNKNOWN &&
//       map_info.lane_type(map_info.current_lane_index() - 1) ==
//       MSD_LANE_TYPE_NORMAL);

//   bool right_direct_exist =
//       (map_info.right_lane_marks() & traffic_light_direction) ||
//       traffic_light_direction == MSD_DIRECTION_UNKNOWN ||
//       (map_info.right_lane_marks() == MSD_DIRECTION_UNKNOWN &&
//       map_info.lane_type(map_info.current_lane_index() + 1) ==
//       MSD_LANE_TYPE_NORMAL);

//   bool curr_direct_has_straight =
//       (map_info.current_lane_marks() & MSD_DIRECTION_GO_STRAIGHT);

//   bool left_direct_has_straight =
//       (map_info.left_lane_marks() & MSD_DIRECTION_GO_STRAIGHT);
//   bool right_direct_has_straight =
//       (map_info.right_lane_marks() & MSD_DIRECTION_GO_STRAIGHT);

//   if (flane_left_id == -1 &&
//       ((lead_cars.lead_one != nullptr &&
//         map_info.dist_to_intsect() - lead_cars.lead_one->d_rel > 150) ||
//        (lead_cars.temp_lead_one != nullptr &&
//         map_info.dist_to_intsect() - lead_cars.temp_lead_one->d_rel > 150)))
//         {
//     if (lead_cars.lead_one != nullptr && status != 7 && status != 8) {
//       fvf_drel_confident = lead_cars.lead_one->d_rel + std::max(30.,
//       lead_cars.lead_one->v_lead * 5.) ;
//     } else if (lead_cars.temp_lead_one != nullptr &&
//                (status == 7 || status == 8)) {
//       fvf_drel_confident = lead_cars.temp_lead_one->d_rel + std::max(30.,
//       lead_cars.temp_lead_one->v_lead * 5.);
//     }
//   } else if (flane_left_id == -1) {
//     if (side == -1 && !left_direct_exist && curr_direct_exist &&
//         map_info.left_boundary_info().size() > 0 &&
//         map_info.left_boundary_info()[0].type == MSD_LANE_BOUNDARY_TYPE_DASH)
//         {
//       if (lead_cars.lead_one != nullptr && lead_cars.lead_one->v_lead < 1.0
//       &&
//           status != 7 && status != 8 &&
//           map_info.left_boundary_info()[0].length - lead_cars.lead_one->d_rel
//           >
//               0) {
//         fvf_drel_confident =
//             lead_cars.lead_one->d_rel +
//             std::min(15.0, map_info.left_boundary_info()[0].length -
//                                lead_cars.lead_one->d_rel);
//       } else if (lead_cars.temp_lead_one != nullptr &&
//                  lead_cars.temp_lead_one->v_lead < 1.0 &&
//                  (status == 7 || status == 8) &&
//                  map_info.left_boundary_info()[0].length -
//                          lead_cars.temp_lead_one->d_rel >
//                      0) {
//         fvf_drel_confident =
//             lead_cars.temp_lead_one->d_rel +
//             std::min(15.0, map_info.left_boundary_info()[0].length -
//                                lead_cars.temp_lead_one->d_rel);
//       } else if (lead_cars.temp_lead_one == nullptr &&
//                  (status == 7 || status == 8)) {
//         fvf_drel_confident =
//             std::min(20.0, map_info.left_boundary_info()[0].length);
//       }

//     } else if (side == 1 && !right_direct_exist && curr_direct_exist &&
//                map_info.right_boundary_info().size() > 0 &&
//                map_info.right_boundary_info()[0].type ==
//                    MSD_LANE_BOUNDARY_TYPE_DASH) {
//       if (lead_cars.lead_one != nullptr && lead_cars.lead_one->v_lead < 1.0
//       &&
//           status != 7 && status != 8 &&
//           map_info.right_boundary_info()[0].length -
//           lead_cars.lead_one->d_rel >
//               0) {
//         fvf_drel_confident =
//             lead_cars.lead_one->d_rel +
//             std::min(15.0, map_info.right_boundary_info()[0].length -
//                                lead_cars.lead_one->d_rel);
//       } else if (lead_cars.temp_lead_one != nullptr &&
//                  lead_cars.temp_lead_one->v_lead < 1.0 &&
//                  (status == 7 || status == 8) &&
//                  map_info.right_boundary_info()[0].length -
//                          lead_cars.temp_lead_one->d_rel >
//                      0) {
//         fvf_drel_confident =
//             lead_cars.temp_lead_one->d_rel +
//             std::min(15.0, map_info.right_boundary_info()[0].length -
//                                lead_cars.temp_lead_one->d_rel);
//       } else if (lead_cars.temp_lead_one == nullptr &&
//                  (status == 7 || status == 8)) {
//         fvf_drel_confident =
//             std::min(20.0, map_info.right_boundary_info()[0].length);
//       }
//     } else if (side == 0 && !curr_direct_exist) {
//       if (status == 7 && map_info.right_boundary_info().size() > 0 &&
//           map_info.right_boundary_info()[0].type ==
//               MSD_LANE_BOUNDARY_TYPE_DASH) {
//         if (lead_cars.temp_lead_one != nullptr &&
//             lead_cars.temp_lead_one->v_lead < 1.0 &&
//             map_info.right_boundary_info()[0].length -
//                     lead_cars.temp_lead_one->d_rel >
//                 0) {
//           fvf_drel_confident =
//               lead_cars.temp_lead_one->d_rel +
//               std::min(15.0, map_info.right_boundary_info()[0].length -
//                                  lead_cars.temp_lead_one->d_rel);
//         } else {
//           fvf_drel_confident =
//               std::min(20.0, map_info.right_boundary_info()[0].length);
//         }
//       } else if (status == 8 && map_info.left_boundary_info().size() > 0 &&
//                  map_info.left_boundary_info()[0].type ==
//                      MSD_LANE_BOUNDARY_TYPE_DASH) {
//         if (lead_cars.temp_lead_one != nullptr &&
//             lead_cars.temp_lead_one->v_lead < 1.0 &&
//             map_info.left_boundary_info()[0].length -
//                     lead_cars.temp_lead_one->d_rel >
//                 0) {
//           fvf_drel_confident =
//               lead_cars.temp_lead_one->d_rel +
//               std::min(15.0, map_info.left_boundary_info()[0].length -
//                                  lead_cars.temp_lead_one->d_rel);
//         } else {
//           fvf_drel_confident =
//               std::min(20.0, map_info.left_boundary_info()[0].length);
//         }
//       }
//     }
//   } else {
//     fvf_drel_confident = 30;
//     if (lead_cars.temp_lead_one != nullptr &&
//         (map_info.dist_to_intsect() - lead_cars.temp_lead_one->d_rel > 150 ||
//         map_info.is_in_intersection())) {
//       fvf_drel_confident = lead_cars.temp_lead_one->d_rel + 30;
//     }
//   }

//   if (side == 0) {
//     front_tracks =
//         get_lane_tracks(LaneProperty::CURRENT_LANE, TrackType::FRONT_TRACK);
//   } else if (side == -1) {
//     front_tracks =
//         get_lane_tracks(LaneProperty::LEFT_LANE, TrackType::FRONT_TRACK);
//   } else if (side == 1) {
//     front_tracks =
//         get_lane_tracks(LaneProperty::RIGHT_LANE, TrackType::FRONT_TRACK);
//   } else {
//     MSD_LOG(ERROR,
//             "[LaneTracksManager::get_vrel_close] Illegal side[%d] argument",
//             side);
//   }

//   if (front_tracks == nullptr) {
//     MSD_LOG(ERROR, "[LaneTracksManager::get_vrel_close] front tracks is
//     null"); return v_rel_close;
//   }

//   for (auto &tr : *front_tracks) {
//     if (tr.d_rel < fvf_drel_confident) {
//       v_rel_close = tr.v_rel;
//       break;
//     }
//   }

//   return v_rel_close;
// }

std::vector<TrackedObject> *LaneTracksManager::get_lane_tracks(int lane,
                                                               int
                                                               track_type) {
  assert(lane == 0 || lane == -1 || lane == 1);
  front_tracks_clane_.clear();
  front_tracks_llane_.clear();
  front_tracks_rlane_.clear();

  const auto &obstacle_manager = session_->environmental_model().get_obstacle_manager();
  const auto &virtual_lane_manager = session_->environmental_model().get_virtual_lane_manager();
  const auto &clane = virtual_lane_manager->get_current_lane();
  const auto &llane = virtual_lane_manager->get_left_lane();
  const auto &rlane = virtual_lane_manager->get_right_lane();
  const auto tracks = lateral_obstacle_.all_tracks();
  std::unordered_map<int, TrackedObject> tracks_map;
  for (auto track : tracks) {
    tracks_map[track.track_id] = track;
  }
  if (clane != nullptr) {
    auto abstacles_id = clane->get_reference_path()->get_lane_obstacles();
    for (auto abstacle_id : abstacles_id) {
      if (tracks_map.find(abstacle_id) != tracks_map.end()) {
        front_tracks_clane_.emplace_back(tracks_map[abstacle_id]);
      }
    }
  }

  if (llane != nullptr) {
    auto &abstacles_id = llane->get_reference_path()->get_lane_obstacles();
    for (auto abstacle_id : abstacles_id) {
      if (tracks_map.find(abstacle_id) != tracks_map.end()) {
        front_tracks_llane_.emplace_back(tracks_map[abstacle_id]);
      }
    }
  }

  if (rlane != nullptr) {
    auto &abstacles_id = rlane->get_reference_path()->get_lane_obstacles();
    for (auto abstacle_id : abstacles_id) {
      if (tracks_map.find(abstacle_id) != tracks_map.end()) {
        front_tracks_rlane_.emplace_back(tracks_map[abstacle_id]);
      }
    }
  }

  if (lane == -1) {
    return &front_tracks_llane_;
  } else if(lane == 0) {
    return &front_tracks_clane_;
  } else if(lane == 1) {
    return &front_tracks_rlane_;
  } 
  // if (lane != LaneProperty::TARGET_LANE && lane != LaneProperty::ORIGIN_LANE
  // &&
  //     lane != LaneProperty::CURRENT_LANE && lane != LaneProperty::LEFT_LANE
  //     && lane != LaneProperty::RIGHT_LANE) {
  //   MSD_LOG(ERROR,
  //           "[LaneTracksManager::get_lane_tracks] Illegal lane property[%d]",
  //           lane);
  //   return nullptr;
  // }

  // if (track_type != TrackType::SIDE_TRACK &&
  //     track_type != TrackType::FISHEYE_TRACK &&
  //     track_type != TrackType::FRONT_TRACK) {
  //   MSD_LOG(ERROR,
  //           "[LaneTracksManager::get_lane_tracks] Illegal track_type[%d]",
  //           track_type);
  //   return nullptr;
  // }

  // auto &clane = map_info_mgr_.clane_;
  // auto &llane = map_info_mgr_.llane_;
  // auto &rlane = map_info_mgr_.rlane_;
  // auto &olane = virtual_lane_mgr_.mutable_origin_lane();
  // auto &tlane = virtual_lane_mgr_.mutable_target_lane();

  // if (lane == LaneProperty::TARGET_LANE) {
  //   assert(tlane.has_master());

  //   if (tlane.has_master() == false) {
  //     MSD_LOG(
  //         ERROR,
  //         "[LaneTracksManager::get_lane_tracks] target lane has no master");
  //     return &empty_tracks_;
  //   }

  //   Lane *master = tlane.master();
  //   if (master != &clane && master != &llane && master != &rlane) {
  //     MSD_LOG(
  //         ERROR,
  //         "[LaneTracksManager::get_lane_tracks] Wrong target lane
  //         position[%d]", master->position());
  //     return &empty_tracks_;
  //   }
  // }

  // if (lane == LaneProperty::ORIGIN_LANE) {
  //   // assert(map_info_manager_.olane_.has_master());
  //   if (!(olane.has_master() == true)) {
  //     olane.attach(&clane);
  //   }

  //   Lane *master = olane.master();
  //   if (master != &clane && master != &llane && master != &rlane) {
  //     MSD_LOG(
  //         ERROR,
  //         "[LateralObstacle::get_lane_tracks] Wrong origin lane
  //         position[%d]", master->position());
  //     return &empty_tracks_;
  //   }
  // }

  // if (lane == LaneProperty::LEFT_LANE ||
  //     (lane == LaneProperty::TARGET_LANE && tlane.master() == &llane) ||
  //     (lane == LaneProperty::ORIGIN_LANE && olane.master() == &llane)) {
  //   update_lane_tracks(track_type, LanePosition::LEFT_POS);

  //   if (track_type == TrackType::SIDE_TRACK) {
  //     return &side_tracks_llane_;
  //   } else if (track_type == TrackType::FRONT_TRACK) {
  //     return &front_tracks_llane_;
  //   }
  // }

  // if (lane == LaneProperty::RIGHT_LANE ||
  //     (lane == LaneProperty::TARGET_LANE && tlane.master() == &rlane) ||
  //     (lane == LaneProperty::ORIGIN_LANE && olane.master() == &rlane)) {
  //   update_lane_tracks(track_type, LanePosition::RIGHT_POS);

  //   if (track_type == TrackType::SIDE_TRACK) {
  //     return &side_tracks_rlane_;
  //   } else if (track_type == TrackType::FRONT_TRACK) {
  //     return &front_tracks_rlane_;
  //   }
  // }

  // if (lane == LaneProperty::CURRENT_LANE ||
  //     (lane == LaneProperty::TARGET_LANE && tlane.master() == &clane) ||
  //     (lane == LaneProperty::ORIGIN_LANE && olane.master() == &clane)) {
  //   update_lane_tracks(track_type, LanePosition::CURR_POS);

  //   if (track_type == TrackType::SIDE_TRACK) {
  //     return &side_tracks_clane_;
  //   } else if (track_type == TrackType::FRONT_TRACK) {
  //     return &front_tracks_clane_;
  //   }
  // }

  // return nullptr;
}

// void LaneTracksManager::update_lane_tracks(int track_type, int position) {
//   std::tuple<int, int> track_info(track_type, position);

//   auto &clane = map_info_mgr_.clane_;
//   auto &llane = map_info_mgr_.llane_;
//   auto &rlane = map_info_mgr_.rlane_;
//   auto &f_refline = virtual_lane_mgr_.mutable_fix_refline();
//   auto &front_tracks = lateral_obstacle_.front_tracks();
//   auto &side_tracks = lateral_obstacle_.side_tracks();

//   if (track_type == TrackType::FRONT_TRACK) {
//     if (position == LanePosition::CURR_POS) {
//       if (lane_tracks_update_.count(track_info) == 0) {
//         front_tracks_clane_.clear();
//         for (auto &tr : front_tracks) {
//           if (clane.is_track_on(tr, track_type, f_refline)) {
//             front_tracks_clane_.push_back(tr);
//           }
//         }

//         lane_tracks_update_.insert(track_info);
//       }
//     } else if (position == LanePosition::LEFT_POS) {
//       if (lane_tracks_update_.count(track_info) == 0) {
//         front_tracks_llane_.clear();
//         for (auto &tr : front_tracks) {
//           if (llane.is_track_on(tr, track_type, f_refline)) {
//             front_tracks_llane_.push_back(tr);
//           }
//         }

//         lane_tracks_update_.insert(track_info);
//       }
//     } else if (position == LanePosition::RIGHT_POS) {
//       if (lane_tracks_update_.count(track_info) == 0) {
//         front_tracks_rlane_.clear();
//         for (auto &tr : front_tracks) {
//           if (rlane.is_track_on(tr, track_type, f_refline)) {
//             front_tracks_rlane_.push_back(tr);
//           }
//         }

//         lane_tracks_update_.insert(track_info);
//       }
//     } else {
//       MSD_LOG(
//           ERROR,
//           "[LateralObstacle::update_lane_tracks] Illegal position[%d]
//           argument", position);
//       return;
//     }
//   } else if (track_type == TrackType::SIDE_TRACK) {
//     if (position == LanePosition::CURR_POS) {
//       if (lane_tracks_update_.count(track_info) == 0) {
//         side_tracks_clane_.clear();
//         for (auto &tr : side_tracks) {
//           if (clane.is_track_on(tr, track_type, f_refline)) {
//             side_tracks_clane_.push_back(tr);
//           }
//         }

//         lane_tracks_update_.insert(track_info);
//       }
//     } else if (position == LanePosition::LEFT_POS) {
//       if (lane_tracks_update_.count(track_info) == 0) {
//         side_tracks_llane_.clear();
//         for (auto &tr : side_tracks) {
//           if (llane.is_track_on(tr, track_type, f_refline)) {
//             side_tracks_llane_.push_back(tr);
//           }
//         }

//         lane_tracks_update_.insert(track_info);
//       }
//     } else if (position == LanePosition::RIGHT_POS) {
//       if (lane_tracks_update_.count(track_info) == 0) {
//         side_tracks_rlane_.clear();
//         for (auto &tr : side_tracks) {
//           if (rlane.is_track_on(tr, track_type, f_refline)) {
//             side_tracks_rlane_.push_back(tr);
//           }
//         }

//         lane_tracks_update_.insert(track_info);
//       }
//     } else {
//       MSD_LOG(
//           ERROR,
//           "[LateralObstacle::update_lane_tracks] Illegal position[%d]
//           argument", position);
//       return;
//     }
//   }
// }

// std::pair<int, pair<double, double>> LaneTracksManager::get_vavg_poi(int
// side, double poi_back_limit, double poi_front_limit) {
//   std::vector<TrackedObject> *front_tracks = nullptr;
//   std::pair<int, pair<double, double>> ret(0, {0.0, 0.0});
//   if (side == 0) {
//     front_tracks =
//         get_lane_tracks(LaneProperty::CURRENT_LANE, TrackType::FRONT_TRACK);
//   } else if (side == -1) {
//     front_tracks =
//         get_lane_tracks(LaneProperty::LEFT_LANE, TrackType::FRONT_TRACK);
//   } else if (side == 1) {
//     front_tracks =
//         get_lane_tracks(LaneProperty::RIGHT_LANE, TrackType::FRONT_TRACK);
//   } else {
//     MSD_LOG(ERROR, "[LateralObstacle::get_vavg_poi] Illegal side[%d]
//     argument",
//               side);
//   }

//   if (front_tracks == nullptr) {
//     MSD_LOG(ERROR, "[LateralObstacle::get_vavg_poi] front tracks is null");
//     return ret;
//   }

//   const auto& map_info = map_info_mgr_.get_map_info();

//   int poi_cnt_total = 0;

//   double poi_v_sum_total = 0.0;
//   double poi_v_lat_sum_total = 0.0;

//   const auto &MergeType = map_info.next_merge_type();
//   const auto &MergeInfo = map_info.next_merge_info();
//   const auto &MergePointDis = map_info.distance_to_next_merge();

//   std::vector<TrackedObject> *cur_tracks =
//         get_lane_tracks(LaneProperty::CURRENT_LANE, TrackType::FRONT_TRACK);
//   std::vector<int> poi_ids;

//   if ((side == 1) && (MergePointDis > 0) && (MergePointDis < poi_front_limit)
//   &&
//      (!MergeInfo.is_split && (MergeInfo.orientation ==
//      MSD_LANE_MERGING_SPLITTING_POINT_TYPE_RIGHT || MergeInfo.orientation ==
//      MSD_LANE_MERGING_SPLITTING_POINT_TYPE_INTERSECT))) {
//     //  ((MergeType ==
//     MSD_LANE_MERGING_SPLITTING_POINT_TYPE_MERGE_FROM_RIGHT) || (MergeType ==
//     MSD_LANE_MERGING_SPLITTING_POINT_TYPE_MERGE_FROM_BOTH))) {
//     poi_ids.clear();
//     for (auto &tr : *cur_tracks) {
//       double tr_yaw = tr.theta;
//       int origin_id = inverse_hash_prediction_id(tr.track_id);
//       if (tr_yaw < 0.0) {
//         tr_yaw += 3.1415926;
//       }
//       if (tr_yaw < 0.0) {
//         tr_yaw += 3.1415926;
//       }
//       tr_yaw = std::min(std::abs(tr_yaw), std::abs(3.1415926 - tr_yaw));
//       if (std::abs(tr.d_max_cpath - tr.d_min_cpath) >= 3.0 && tr_yaw >= 0.52)
//       { // 60 deg
//         continue;
//       }
//       if ((tr.type < 1) || (tr.type > 5)) continue;
//       if (tr.d_max_cpath < + 1.0 - 3.8 / 2) continue;
//       if (tr.d_min_cpath > - 1.0 + 3.8 / 2) continue;
//       if (std::abs(tr.d_max_cpath - tr.d_min_cpath) <= 0.8) continue;
//       double tmp_d = tr.d_rel;
//       double tmp_v = tr.v_rel + ego_state_.ego_vel;
//       double tmp_v_lat = tr.v_lat;
//       if ((tmp_d < poi_front_limit) && (tmp_d >= MergePointDis) &&
//       std::find(poi_ids.begin(), poi_ids.end(), origin_id) == poi_ids.end())
//       {
//         poi_v_sum_total += tmp_v;
//         poi_v_lat_sum_total += tmp_v_lat;
//         poi_ids.emplace_back(origin_id);
//         ++poi_cnt_total;
//       }
//     }
//     poi_front_limit = std::min(poi_front_limit, MergePointDis);
//   }

//   if ((side == -1) && (MergePointDis > 0) && (MergePointDis <
//   poi_front_limit) &&
//      (!MergeInfo.is_split && (MergeInfo.orientation ==
//      MSD_LANE_MERGING_SPLITTING_POINT_TYPE_LEFT || MergeInfo.orientation ==
//      MSD_LANE_MERGING_SPLITTING_POINT_TYPE_INTERSECT))) {
//     poi_ids.clear();
//     for (auto &tr : *cur_tracks) {
//       double tr_yaw = tr.theta;
//       int origin_id = inverse_hash_prediction_id(tr.track_id);
//       if (tr_yaw < 0.0) {
//         tr_yaw += 3.1415926;
//       }
//       if (tr_yaw < 0.0) {
//         tr_yaw += 3.1415926;
//       }
//       tr_yaw = std::min(std::abs(tr_yaw), std::abs(3.1415926 - tr_yaw));
//       if (std::abs(tr.d_max_cpath - tr.d_min_cpath) >= 3.0 && tr_yaw >= 0.52)
//       { // 60 deg
//         continue;
//       }
//       if ((tr.type < 1) || (tr.type > 5)) continue;
//       if (tr.d_max_cpath < + 1.0 - 3.8 / 2) continue;
//       if (tr.d_min_cpath > - 1.0 + 3.8 / 2) continue;
//       if (std::abs(tr.d_max_cpath - tr.d_min_cpath) <= 0.8) continue;
//       double tmp_d = tr.d_rel;
//       double tmp_v = tr.v_rel + ego_state_.ego_vel;
//       double tmp_v_lat = tr.v_lat;
//       if ((tmp_d < poi_front_limit) && (tmp_d >= MergePointDis) &&
//       std::find(poi_ids.begin(), poi_ids.end(), origin_id) == poi_ids.end())
//       {
//         poi_v_sum_total += tmp_v;
//         poi_v_lat_sum_total += tmp_v_lat;
//         poi_ids.emplace_back(origin_id);
//         ++poi_cnt_total;
//       }
//     }
//     poi_front_limit = std::min(poi_front_limit, MergePointDis);
//   }

//   poi_ids.clear();
//   for (auto &tr : *front_tracks) {
//     double tr_yaw = tr.theta;
//     int origin_id = inverse_hash_prediction_id(tr.track_id);
//     if (tr_yaw < 0.0) {
//       tr_yaw += 3.1415926;
//     }
//     if (tr_yaw < 0.0) {
//       tr_yaw += 3.1415926;
//     }
//     tr_yaw = std::min(std::abs(tr_yaw), std::abs(3.1415926 - tr_yaw));
//     if (std::abs(tr.d_max_cpath - tr.d_min_cpath) >= 3.0 && tr_yaw >= 0.52) {
//     // 60 deg
//       continue;
//     }
//     if ((tr.type < 1) || (tr.type > 5)) continue;
//     if (tr.d_max_cpath < -3.8 * side + 1.0 - 3.8 / 2) continue;
//     if (tr.d_min_cpath > -3.8 * side - 1.0 + 3.8 / 2) continue;
//     if (std::abs(tr.d_max_cpath - tr.d_min_cpath) <= 0.8) continue;
//     double tmp_d = tr.d_rel;
//     double tmp_v = tr.v_rel + ego_state_.ego_vel;
//     double tmp_v_lat = tr.v_lat;
//     if (tmp_d < poi_front_limit && tmp_d >= poi_back_limit &&
//     std::find(poi_ids.begin(), poi_ids.end(), origin_id) == poi_ids.end()) {
//       poi_v_sum_total += tmp_v;
//       poi_v_lat_sum_total += tmp_v_lat;
//       poi_ids.emplace_back(origin_id);
//       ++poi_cnt_total;
//     }
//   }

//   if (side == 0 && poi_back_limit < 0.5 && poi_front_limit > 1.0) {
//     poi_v_sum_total += ego_state_.ego_vel;
//     ++poi_cnt_total;
//   }

//   double poi_v_avg_total = (poi_cnt_total > 0) ? (poi_v_sum_total /
//   poi_cnt_total) : 0.0; double poi_v_lat_avg_total = (poi_cnt_total > 0) ?
//   (poi_v_lat_sum_total / poi_cnt_total) : 0.0;

//   ret.first = poi_cnt_total;
//   ret.second.first = poi_v_avg_total;
//   ret.second.second = poi_v_lat_avg_total;
//   return ret;
// }

// std::pair<int, double> LaneTracksManager::get_vavg_poi_int(int side, double
// bound, double poi_back_limit, double poi_front_limit) {
//   const auto& map_info = map_info_mgr_.get_map_info();
//   std::pair<int, double> res(0, 0.0);
//   double poi_left_limit = 4;
//   double poi_right_limit = -4;

//   if (side == -1) {
//     poi_right_limit = bound;
//     poi_left_limit = bound + 8.0;  // dist to center line ?
//   } else if ( side == 1) {
//     poi_left_limit = bound;
//     poi_right_limit = bound - 8.0;  // dist to zebra ?
//   }

//   int poi_cnt = 0;
//   double poi_v_sum = 0.0;

//   for (auto &tr: lateral_obstacle_.front_tracks()) {
//     if (tr.type < 1 || tr.type > 4) continue;
//     if (tr.v_rel < -2.0) continue;  // ignore cars from the opposite
//     direction if (tr.d_rel < poi_back_limit || tr.d_rel >= poi_front_limit)
//     continue; if (tr.d_max_cpath < poi_right_limit || tr.d_min_cpath >=
//     poi_left_limit) continue;  // ? double tr_yaw = tr.theta; if (tr_yaw <
//     0.0) {
//       tr_yaw += 3.1415926;
//     }
//     if (tr_yaw < 0.0) {
//       tr_yaw += 3.1415926;
//     }
//     tr_yaw = std::min(std::abs(tr_yaw), std::abs(3.1415926 - tr_yaw));
//     if (std::abs(tr.d_max_cpath - tr.d_min_cpath) >= 3.0 && tr_yaw >= 0.52) {
//     // 60 deg
//       continue;
//     }
//     double tmp_v = tr.v_rel + ego_state_.ego_vel;
//     ++poi_cnt;

//     if ((tr.d_max_cpath < poi_left_limit && tr.d_min_cpath >=
//     poi_right_limit)) {
//       poi_v_sum += tmp_v;
//     }
//     else {
//       double fraction = (tr.d_max_cpath - tr.d_min_cpath) / 3.0;
//       fraction = std::max(fraction, 1.0);
//       poi_v_sum += tmp_v * fraction;
//     }
//   }

//   for (auto &tr: lateral_obstacle_.side_tracks()) {
//     if (tr.type < 1 || tr.type > 4) continue;
//     if (tr.v_rel < -2.0) continue;  // ignore cars from the opposite
//     direction if (tr.d_rel < poi_back_limit || tr.d_rel >= poi_front_limit)
//     continue; if (tr.d_max_cpath < poi_right_limit || tr.d_min_cpath >=
//     poi_left_limit) continue;  // ? double tr_yaw = tr.theta; if (tr_yaw <
//     0.0) {
//       tr_yaw += 3.1415926;
//     }
//     if (tr_yaw < 0.0) {
//       tr_yaw += 3.1415926;
//     }
//     tr_yaw = std::min(std::abs(tr_yaw), std::abs(3.1415926 - tr_yaw));
//     if (std::abs(tr.d_max_cpath - tr.d_min_cpath) >= 3.0 && tr_yaw >= 0.52) {
//     // 60 deg
//       continue;
//     }
//     double tmp_v = tr.v_rel + ego_state_.ego_vel;
//     ++poi_cnt;

//     if ((tr.d_max_cpath < poi_left_limit && tr.d_min_cpath >=
//     poi_right_limit)) {
//       poi_v_sum += tmp_v;
//     }
//     else {
//       double fraction = (tr.d_max_cpath - tr.d_min_cpath) / 3.0;
//       fraction = std::max(fraction, 1.0);
//       poi_v_sum += tmp_v * fraction;
//     }
//   }

//   res.first = poi_cnt;
//   res.second = poi_cnt > 0 ? poi_v_sum / poi_cnt : 0.0;
//   return res;
// }

// bool LaneTracksManager::front_cone_exist() {
//   std::vector<TrackedObject> *front_tracks =
//   get_lane_tracks(LaneProperty::CURRENT_LANE, TrackType::FRONT_TRACK);

//   if (front_tracks == nullptr) {
//     MSD_LOG(ERROR, "[LateralObstacle::front_cone_exist] front tracks is
//     null"); return false;
//   }

//   for (auto &tr : *front_tracks) {
//     if (tr.d_max_cpath < + 1.0 - 3.8 / 2) continue;
//     if (tr.d_min_cpath > - 1.0 + 3.8 / 2) continue;
//     if (tr.d_rel < 0.0) continue;
//     if (tr.type == 20001) return true;
//   }

//   return false;
// }

}  // namespace planning
