#include "avoid_obstacle_maintainer.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "ifly_time.h"
#include "lateral_obstacle.h"
#include "planning_context.h"

namespace planning {

double AvoidObstacleMaintainer::UpdateAntsidesStrict() {
  double strict_car = 10;
  double strict_side_car = 10;
  double front_ly_rel = 10;
  double front_ry_rel = 10;
  double side_ly_rel = 10;
  double side_ry_rel = 10;
  double ego_car_width = 2.2;
  double lat_safety_buffer = 0.2;

  int debug_final_id = -1000;
  int debug_front_id = -1000;
  int debug_side_id = -1000;

  auto &virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  auto &lateral_obstacle =
      session_->mutable_environmental_model()->get_lateral_obstacle();
  auto &ego_state =
      session_->mutable_environmental_model()->get_ego_state_manager();
  // auto &frenet_ego_state = reference_path_ptr_->get_frenet_ego_state();

  double v_ego = ego_state->ego_v();
  auto &front_tracks_copy = lateral_obstacle->front_tracks_copy();
  auto &side_tracks_l = lateral_obstacle->side_tracks_l();
  auto &side_tracks_r = lateral_obstacle->side_tracks_r();

  if (avd_car_past_[0].size() > 0) {
    if (avd_car_past_[0][5] >
            std::min(((ego_car_width + lat_safety_buffer) - lane_width_ / 2),
                     0.9) &&
        (avd_car_past_[1].size() == 0 ||
         avd_car_past_[1][5] >
             std::min(((ego_car_width + lat_safety_buffer) - lane_width_ / 2),
                      0.9))) {
      if (front_tracks_copy.size() > 0) {
        for (auto &tr : front_tracks_copy) {
          if (tr.d_max_cpath < -1 && tr.d_max_cpath > -strict_car &&
              tr.d_rel < std::max(std::min(-tr.v_rel * 15, 60.0), 20.0)) {
            strict_car = -tr.d_max_cpath;
            if (tr.type == 20001) {
              strict_car -= 0.7;
            }
            front_ry_rel = std::fabs(strict_car);
            debug_front_id = tr.track_id;
          }
        }
      }

      front_ry_rel = std::max(front_ry_rel - 2.0, 0.0);
      if (avd_car_past_[0][5] < 1.8) {
        if (std::fabs(avd_car_past_[0][5] - 1.8) > front_ry_rel) {
          front_ry_rel = std::fabs(avd_car_past_[0][5] - 1.8);
        }
      }

      if (side_tracks_r.size() > 0) {
        for (auto &tr : side_tracks_r) {
          if (tr.d_max_cpath < -1 && tr.d_max_cpath > -strict_side_car &&
              tr.d_rel > std::min(-10.0 - tr.v_rel * 2, -5.0) && tr.d_rel < 2) {
            strict_side_car = -tr.d_max_cpath;
            if (tr.type == 20001) {
              strict_side_car -= 0.7;
            }
            side_ry_rel = std::fabs(strict_side_car);
            debug_side_id = tr.track_id;
          }
        }
      }

      side_ry_rel = std::max(side_ry_rel - 2.0, 0.0);

      if (avd_car_past_[0][5] < 1.8) {
        if (std::fabs(avd_car_past_[0][5] - 1.8) > side_ry_rel) {
          side_ry_rel = std::fabs(avd_car_past_[0][5] - 1.8);
        }
      }

      if (side_ry_rel <= front_ry_rel &&
          std::fabs(side_ry_rel - final_y_rel_) > 0.1) {
        final_y_rel_ = side_ry_rel;
        debug_final_id = debug_side_id;
      } else if (side_ry_rel > front_ry_rel &&
                 std::fabs(front_ry_rel - final_y_rel_) > 0.1) {
        final_y_rel_ = front_ry_rel;
        debug_final_id = debug_front_id;
      }
    } else if (avd_car_past_[0][5] < 0 &&
               (avd_car_past_[1].size() == 0 ||
                (avd_car_past_[1][0] != -1 && avd_car_past_[1][5] < 0))) {
      if (front_tracks_copy.size() > 0) {
        for (auto &tr : front_tracks_copy) {
          if (avd_car_past_[0][6] < 0 && tr.d_min_cpath > 1 &&
              tr.d_min_cpath < strict_car && tr.d_min_cpath != 100 &&
              tr.d_rel < std::max(std::min(-tr.v_rel * 15, 60.0), 20.0)) {
            strict_car = tr.d_min_cpath;
            if (tr.type == 20001) {
              strict_car += 0.7;
            }
            front_ly_rel = std::fabs(strict_car);
            debug_front_id = tr.track_id;
          } else if (avd_car_past_[0][6] >= 0 && tr.d_max_cpath < -1 &&
                     tr.d_max_cpath > -strict_car &&
                     tr.d_rel <
                         std::max(std::min(std::fabs(tr.v_rel * 15), 60.0),
                                  20.0)) {
            strict_car = -tr.d_max_cpath;
            if (tr.type == 20001) {
              strict_car -= 0.7;
            }
            front_ly_rel = std::fabs(strict_car);
            debug_front_id = tr.track_id;
          }
        }
      }

      front_ly_rel = std::max(front_ly_rel - 2.0, 0.0);
      if (avd_car_past_[0][6] < 0 && avd_car_past_[0][6] > -1.8) {
        if (std::fabs(avd_car_past_[0][6] + 1.8) > front_ly_rel) {
          front_ly_rel = std::fabs(avd_car_past_[0][6] + 1.8);
        }
      }

      if (avd_car_past_[0][6] < 0 && side_tracks_l.size() > 0) {
        for (auto &tr : side_tracks_l) {
          if (tr.d_min_cpath > 1 && tr.d_min_cpath < strict_side_car &&
              tr.d_rel > std::min(-10.0 - tr.v_rel * 2, -5.0) && tr.d_rel < 2) {
            strict_side_car = tr.d_min_cpath;
            if (tr.type == 20001) {
              strict_side_car += 0.7;
            }
            side_ly_rel = std::fabs(strict_side_car);
            debug_side_id = tr.track_id;
          }
        }
      }

      if (avd_car_past_[0][6] >= 0 && side_tracks_r.size() > 0) {
        for (auto &tr : side_tracks_r) {
          if (tr.d_max_cpath < -1 && tr.d_max_cpath > -strict_side_car &&
              (((tr.v_rel > 0.5 || v_ego >= 3) &&
                tr.d_rel > std::min(-10.0 - tr.v_rel * 2, -5.0)) ||
               (v_ego < 3 && tr.v_rel <= 0.5 && tr.d_rel > -6.0)) &&
              tr.d_rel < 2) {
            strict_side_car = -tr.d_max_cpath;
            if (tr.type == 20001) {
              strict_side_car -= 0.7;
            }
            side_ly_rel = std::fabs(strict_side_car);
            debug_side_id = tr.track_id;
          }
        }
      }

      side_ly_rel = std::max(side_ly_rel - 2.0, 0.0);

      if (avd_car_past_[0][6] > -1.8 && avd_car_past_[0][6] < 0) {
        if (std::fabs(avd_car_past_[0][6] + 1.8) > side_ly_rel) {
          side_ly_rel = std::fabs(avd_car_past_[0][6] + 1.8);
        }
      }

      if (side_ly_rel <= front_ly_rel &&
          std::fabs(side_ly_rel - final_y_rel_) > 0.1) {
        final_y_rel_ = side_ly_rel;
        debug_final_id = debug_side_id;
      } else if (side_ly_rel > front_ly_rel &&
                 std::fabs(front_ly_rel - final_y_rel_) > 0.1) {
        final_y_rel_ = front_ly_rel;
        debug_final_id = debug_front_id;
      }
    }
  }
  JSON_DEBUG_VALUE("final_y_rel_id", debug_final_id);
  JSON_DEBUG_VALUE("final_y_rel", final_y_rel_);
  return final_y_rel_;
}

bool AvoidObstacleMaintainer::UpdateLFrontAvdsInfo(bool no_near_car) {
  auto &lateral_obstacle =
      session_->mutable_environmental_model()->get_lateral_obstacle();

  if (no_near_car == false ||
      lateral_obstacle->front_tracks_copy().size() == 0) {
    return no_near_car;
  }

  for (auto &tr : lateral_obstacle->front_tracks_copy()) {
    if (tr.d_rel < 8 &&
        ((tr.d_min_cpath > 1.0 && tr.d_min_cpath < lane_width_ - 1.1) ||
         (tr.d_path > 1.0 && tr.d_path < 1.4))) {
      avd_car_past_[0][0] = tr.track_id;
      avd_car_past_[0][3] = tr.d_rel;
      avd_car_past_[0][2] = tr.v_rel;

      if (std::fabs(avd_car_past_[0][5] - tr.d_min_cpath) > 0.1) {
        avd_car_past_[0][1] = tr.trajectory.intersection;
        avd_car_past_[0][4] = tr.v_lat;
        avd_car_past_[0][5] = tr.d_min_cpath;
        avd_car_past_[0][6] = tr.d_max_cpath;
      }

      return false;
    }
  }

  return no_near_car;
}

bool AvoidObstacleMaintainer::UpdateRFrontAvdsInfo(bool no_near_car) {
  auto &lateral_obstacle =
      session_->mutable_environmental_model()->get_lateral_obstacle();

  if (no_near_car == false ||
      lateral_obstacle->front_tracks_copy().size() == 0) {
    return no_near_car;
  }

  for (auto &tr : lateral_obstacle->front_tracks_copy()) {
    if (tr.d_rel < 8 &&
        ((tr.d_max_cpath > -(lane_width_ - 1.1) && tr.d_max_cpath < -1.0) ||
         (tr.d_path > 1.0 && tr.d_path < 1.4))) {
      avd_car_past_[0][0] = tr.track_id;
      avd_car_past_[0][3] = tr.d_rel;
      avd_car_past_[0][2] = tr.v_rel;

      if (std::fabs(avd_car_past_[0][5] - tr.d_min_cpath) > 0.1) {
        avd_car_past_[0][1] = tr.trajectory.intersection;
        avd_car_past_[0][4] = tr.v_lat;
        avd_car_past_[0][5] = tr.d_min_cpath;
        avd_car_past_[0][6] = tr.d_max_cpath;
      }

      return false;
    }
  }

  return no_near_car;
}

bool AvoidObstacleMaintainer::UpdateLSideAvdsInfo(bool no_near_car) {
  if (no_near_car == false) {
    return no_near_car;
  }

  auto &lateral_obstacle =
      session_->mutable_environmental_model()->get_lateral_obstacle();

  for (auto &tr : lateral_obstacle->side_tracks_l()) {
    if (tr.d_rel > -7 &&
        ((tr.d_min_cpath > 1.0 && tr.d_min_cpath < lane_width_ - 1.1) ||
         (tr.d_path > 1.0 && tr.d_path < 1.7))) {
      avd_car_past_[0][0] = tr.track_id;
      avd_car_past_[0][3] = tr.d_rel;
      avd_car_past_[0][2] = tr.v_rel;

      if (std::fabs(avd_car_past_[0][5] - tr.d_min_cpath) > 0.1) {
        avd_car_past_[0][1] = tr.trajectory.intersection;
        avd_car_past_[0][4] = tr.v_lat;
        avd_car_past_[0][5] = tr.d_min_cpath;
        avd_car_past_[0][6] = tr.d_max_cpath;
      }

      return false;
    }
  }

  return no_near_car;
}

bool AvoidObstacleMaintainer::UpdateRSideAvdsInfo(bool no_near_car) {
  if (no_near_car == false) {
    return no_near_car;
  }

  auto &lateral_obstacle =
      session_->mutable_environmental_model()->get_lateral_obstacle();

  for (auto &tr : lateral_obstacle->side_tracks_r()) {
    if (tr.d_rel > -7 &&
        ((tr.d_max_cpath > -(lane_width_ - 1.1) && tr.d_max_cpath < -1.0) ||
         (tr.d_path > 1.0 && tr.d_path < 1.7))) {
      avd_car_past_[0][0] = tr.track_id;
      avd_car_past_[0][3] = tr.d_rel;
      avd_car_past_[0][2] = tr.v_rel;

      if (std::fabs(avd_car_past_[0][5] - tr.d_min_cpath) > 0.1) {
        avd_car_past_[0][1] = tr.trajectory.intersection;
        avd_car_past_[0][4] = tr.v_lat;
        avd_car_past_[0][5] = tr.d_min_cpath;
        avd_car_past_[0][6] = tr.d_max_cpath;
      }

      return false;
    }
  }

  return no_near_car;
}

bool AvoidObstacleMaintainer::Process(planning::framework::Session *session) {
  int enter0 = 0;
  int enter1 = 0;
  int enter2 = 0;
  int ncar_cnt = 0;

  double fs_y_rel = 10.0;
  double safety_dist = 2.0;

  std::vector<std::vector<double>> avd_cars;
  std::vector<std::vector<double>> avd_temp_cars;

  t_avd_car_ = 3.0;
  is_ncar_ = false;

  session_ = session;
  auto &virtual_lane_manager =
      session->mutable_environmental_model()->get_virtual_lane_manager();
  auto &lateral_obstacle =
      session->mutable_environmental_model()->get_lateral_obstacle();
  auto &ego_state =
      session->mutable_environmental_model()->get_ego_state_manager();
  const CoarsePlanningInfo &coarse_planning_info =
      session_->planning_context()
          .lane_change_decider_output()
          .coarse_planning_info;

  int state = coarse_planning_info.target_state;

  TrackedObject *lead_one = lateral_obstacle->leadone();

  if (state == ROAD_NONE && intersection_cnt_ != 0) {
    intersection_cnt_ = 0;
  }

  double v_ego = ego_state->ego_v();
  // double l_ego = frenet_ego_state.l();
  double curr_time = IflyTime::Now_s();

  std::vector<TrackedObject> front_side_tracks;
  for (auto &tr : lateral_obstacle->front_tracks_copy()) {
    front_side_tracks.emplace_back(tr);
  }

  for (auto &tr : lateral_obstacle->side_tracks()) {
    front_side_tracks.emplace_back(tr);
  }

  if (is_ncar_ == true && avd_car_past_[0].size() > 0) {
    avd_car_past_[0][9] = UpdateAntsidesStrict();
  } else {
    if (lateral_obstacle->front_tracks_copy().size() > 0) {
      for (auto &tr : lateral_obstacle->front_tracks_copy()) {
        if (tr.is_avd_car == true && ncar_cnt < 3) {
          ncar_cnt += 1;

          if (tr.type > 10000) {
            if (tr.d_min_cpath > 0) {
              if (tr.type != 20001) {
                avd_temp_cars.push_back(
                    {(double)tr.track_id, 0, tr.v_rel, tr.d_rel - 3, tr.v_lat,
                     tr.d_min_cpath - 0.3, tr.d_max_cpath - 0.3,
                     double(tr.type), curr_time, final_y_rel_});
              } else {
                avd_temp_cars.push_back(
                    {(double)tr.track_id, 0, tr.v_rel, tr.d_rel - 3, tr.v_lat,
                     tr.d_min_cpath + 0.2, tr.d_max_cpath + 0.2,
                     double(tr.type), curr_time, final_y_rel_});
              }
            } else {
              if (tr.type != 20001) {
                avd_temp_cars.push_back(
                    {(double)tr.track_id, 0, tr.v_rel, tr.d_rel - 3, tr.v_lat,
                     tr.d_min_cpath + 0.3, tr.d_max_cpath + 0.3,
                     double(tr.type), curr_time, final_y_rel_});
              } else {
                avd_temp_cars.push_back(
                    {(double)tr.track_id, 0, tr.v_rel, tr.d_rel - 3, tr.v_lat,
                     tr.d_min_cpath - 0.2, tr.d_max_cpath - 0.2,
                     double(tr.type), curr_time, final_y_rel_});
              }
            }

            if (avd_temp_cars.size() > 1) {
              if (std::fabs(avd_temp_cars[1][3] - avd_temp_cars[0][3]) <
                  3 * v_ego) {
                if (((avd_temp_cars[0][5] >
                          std::min(2.4 - lane_width_ / 2, 0.) &&
                      avd_temp_cars[1][5] > std::min(lane_width_ - 2.4, 0.)) ||
                     (avd_temp_cars[0][6] <
                          std::max(lane_width_ / 2 - 2.4, 0.) &&
                      avd_temp_cars[1][6] <
                          std::max(lane_width_ / 2 - 2.4, 0.))) &&
                    true) {  // TODO(Rui):map_info.is_in_intersection() == false
                  if (std::fabs(avd_temp_cars[0][5]) >
                      std::fabs(avd_temp_cars[1][5])) {
                    avd_temp_cars[0][5] = avd_temp_cars[1][5];
                    avd_temp_cars[0][6] = avd_temp_cars[1][6];
                  }

                  avd_temp_cars.pop_back();
                  ncar_cnt -= 1;
                } else if (avd_temp_cars[0][5] > 0 && avd_temp_cars[1][5] < 0) {
                  if (avd_temp_cars.size() > 2 &&
                      std::fabs(avd_temp_cars[2][3] - avd_temp_cars[1][3]) <
                          3 * v_ego) {
                    if (avd_temp_cars[2][5] > 0 &&
                        avd_temp_cars[2][5] < avd_temp_cars[0][5]) {
                      avd_temp_cars[0][5] = avd_temp_cars[2][5];
                      avd_temp_cars[0][6] = avd_temp_cars[2][6];
                    } else if (avd_temp_cars[2][6] < 0 &&
                               avd_temp_cars[2][6] > avd_temp_cars[1][6]) {
                      avd_temp_cars[1][5] = avd_temp_cars[2][5];
                      avd_temp_cars[1][6] = avd_temp_cars[2][6];
                    }

                    avd_temp_cars.pop_back();
                  }

                  ncar_cnt -= 1;
                } else if (avd_temp_cars[0][5] < 0 && avd_temp_cars[1][5] > 0) {
                  if (avd_temp_cars.size() > 2 &&
                      std::fabs(avd_temp_cars[2][3] - avd_temp_cars[1][3]) <
                          3 * v_ego) {
                    if (avd_temp_cars[2][5] > 0 &&
                        avd_temp_cars[2][5] < avd_temp_cars[1][5]) {
                      avd_temp_cars[1][5] = avd_temp_cars[2][5];
                      avd_temp_cars[1][6] = avd_temp_cars[2][6];
                    } else if (avd_temp_cars[2][6] < 0 &&
                               avd_temp_cars[2][6] > avd_temp_cars[0][6]) {
                      avd_temp_cars[0][5] = avd_temp_cars[2][5];
                      avd_temp_cars[0][6] = avd_temp_cars[2][6];
                    }

                    avd_temp_cars.pop_back();
                  }

                  ncar_cnt -= 1;
                }
              }
            }
          } else {
            if (avd_cars.size() < 2) {
              avd_cars.push_back(
                  {(double)tr.track_id, (double)tr.trajectory.intersection,
                   tr.v_rel, tr.d_rel, tr.v_lat, tr.d_min_cpath, tr.d_max_cpath,
                   double(tr.d_rel), curr_time, final_y_rel_});
            }
          }

          if (avd_temp_cars.size() == 1 && avd_cars.size() == 1) {
            if (avd_temp_cars[0][3] <= avd_cars[0][3]) {
              avd_cars.insert(avd_cars.begin(), avd_temp_cars[0]);
              enter0 = 1;
            } else {
              avd_cars.push_back(avd_temp_cars[0]);
              enter1 = 1;
            }
          } else if (avd_temp_cars.size() == 1 && avd_cars.size() == 2) {
            if (!enter0 && !enter1) {
              if ((avd_cars[0][5] > 0 && avd_cars[1][5] > 0 &&
                   avd_temp_cars[0][5] < 0) ||
                  (avd_cars[0][6] < 0 && avd_cars[1][6] < 0 &&
                   avd_temp_cars[0][6] > 0)) {
                if (std::fabs(avd_temp_cars[0][3] - avd_cars[1][3]) <
                    5 * v_ego) {
                  if (std::fabs(avd_cars[0][5]) > std::fabs(avd_cars[1][5])) {
                    avd_cars[0][5] = avd_cars[1][5];
                    avd_cars[0][6] = avd_cars[1][6];
                    avd_cars[1] = avd_temp_cars[0];
                  }
                }
              } else if (avd_cars[0][5] > 0 && avd_cars[1][6] < 0 &&
                         std::fabs(avd_temp_cars[0][3] - avd_cars[1][3]) <
                             5 * v_ego &&
                         std::fabs(avd_cars[1][3] - avd_cars[0][3]) <
                             3 * v_ego) {
                if (avd_temp_cars[0][6] < 0 &&
                    avd_temp_cars[0][6] > avd_cars[1][6]) {
                  avd_cars[1][5] = avd_temp_cars[0][5];
                  avd_cars[1][6] = avd_temp_cars[0][6];
                } else if (avd_temp_cars[0][5] > 0 &&
                           avd_temp_cars[0][5] < avd_cars[0][5]) {
                  avd_cars[0][5] = avd_temp_cars[0][5];
                  avd_cars[0][6] = avd_temp_cars[0][6];
                }
              } else if (avd_cars[0][6] < 0 && avd_cars[1][5] > 0 &&
                         std::fabs(avd_temp_cars[0][3] - avd_cars[1][3]) <
                             5 * v_ego &&
                         std::fabs(avd_cars[1][3] - avd_cars[0][3]) <
                             3 * v_ego) {
                if (avd_temp_cars[0][6] < 0 &&
                    avd_temp_cars[0][6] > avd_cars[0][6]) {
                  avd_cars[0][5] = avd_temp_cars[0][5];
                  avd_cars[0][6] = avd_temp_cars[0][6];
                } else if (avd_temp_cars[0][5] > 0 &&
                           avd_temp_cars[0][5] < avd_cars[1][5]) {
                  avd_cars[1][5] = avd_temp_cars[0][5];
                  avd_cars[1][6] = avd_temp_cars[0][6];
                }
              }
            } else if (enter0 && std::fabs(avd_cars[0][5]) >
                                     std::fabs(avd_temp_cars[0][5])) {
              avd_cars[0][5] = avd_temp_cars[0][5];
              avd_cars[0][6] = avd_temp_cars[0][6];
            } else if (enter1 && std::fabs(avd_cars[1][5]) >
                                     std::fabs(avd_temp_cars[0][5])) {
              avd_cars[1][5] = avd_temp_cars[0][5];
              avd_cars[1][6] = avd_temp_cars[0][6];
            }
          } else if (avd_temp_cars.size() == 2 && avd_cars.size() == 1) {
            if (avd_temp_cars[0][3] <= avd_cars[0][3]) {
              avd_cars.push_back(avd_temp_cars[1]);
              avd_cars[0] = avd_temp_cars[0];
              enter2 = 1;
            } else if (avd_cars[0][6] < 0 && avd_temp_cars[0][5] > 0 &&
                       (avd_temp_cars[0][3] - avd_cars[0][3]) < 5 * v_ego) {
              avd_cars.push_back(avd_temp_cars[0]);
            } else if (avd_cars[0][6] < 0 && avd_temp_cars[0][6] < 0 &&
                       avd_temp_cars[1][5] > 0 &&
                       avd_temp_cars[1][3] < 5 * v_ego) {
              avd_cars.push_back(avd_temp_cars[1]);
            } else if (avd_cars[0][5] > 0 && avd_temp_cars[0][6] < 0 &&
                       avd_temp_cars[0][3] < 5 * v_ego) {
              avd_cars.push_back(avd_temp_cars[0]);
            } else if (avd_cars[0][5] > 0 && avd_temp_cars[0][5] > 0 &&
                       avd_temp_cars[1][6] < 0 &&
                       avd_temp_cars[1][3] < 5 * v_ego) {
              avd_cars.push_back(avd_temp_cars[1]);
            }
          } else if (avd_temp_cars.size() == 2 && avd_cars.size() == 2) {
            if (!enter0 && !enter1 && avd_temp_cars[0][3] >= avd_cars[1][3]) {
              if ((avd_cars[0][5] > 0 && avd_cars[1][5] > 0 &&
                   avd_temp_cars[0][5] < 0) ||
                  (avd_cars[0][6] < 0 && avd_cars[1][6] < 0 &&
                   avd_temp_cars[0][6] > 0)) {
                if (std::fabs(avd_temp_cars[0][3] - avd_cars[1][3]) <
                    5 * v_ego) {
                  if (std::fabs(avd_cars[0][5]) > std::fabs(avd_cars[1][5]) &&
                      std::fabs(avd_cars[1][3] - avd_cars[0][3]) < 3 * v_ego) {
                    avd_cars[0][5] = avd_cars[1][5];
                    avd_cars[0][6] = avd_cars[1][6];
                    avd_cars[1] = avd_temp_cars[0];
                  }
                }
              } else if (((avd_cars[0][5] > 0 && avd_cars[1][5] > 0 &&
                           avd_temp_cars[0][5] > 0 &&
                           avd_temp_cars[1][6] < 0) ||
                          (avd_cars[0][6] < 0 && avd_cars[1][6] < 0 &&
                           avd_temp_cars[0][6] < 0 &&
                           avd_temp_cars[1][5] > 0)) &&
                         std::fabs(avd_cars[1][3] - avd_cars[0][3]) <
                             3 * v_ego &&
                         std::fabs(avd_temp_cars[0][3] - avd_cars[1][3]) <
                             5 * v_ego &&
                         std::fabs(avd_temp_cars[1][3] - avd_temp_cars[1][3]) <
                             5 * v_ego) {
                if (avd_cars[0][5] > 0) {
                  avd_cars[0][5] =
                      std::min(avd_cars[0][5],
                               std::min(avd_cars[1][5], avd_temp_cars[0][5]));
                  avd_cars[0][6] =
                      std::min(avd_cars[0][6],
                               std::min(avd_cars[1][6], avd_temp_cars[0][6]));
                  avd_cars[1] = avd_temp_cars[1];
                } else if (avd_cars[0][6] < 0) {
                  avd_cars[0][5] =
                      std::max(avd_cars[0][5],
                               std::max(avd_cars[1][5], avd_temp_cars[0][5]));
                  avd_cars[0][6] =
                      std::max(avd_cars[0][6],
                               std::max(avd_cars[1][6], avd_temp_cars[0][6]));
                  avd_cars[1] = avd_temp_cars[1];
                }
              } else if (avd_cars[0][5] > 0 && avd_cars[1][6] < 0 &&
                         std::fabs(avd_temp_cars[0][3] - avd_cars[1][3]) <
                             5 * v_ego &&
                         std::fabs(avd_cars[1][3] - avd_cars[0][3]) <
                             3 * v_ego) {
                if (avd_temp_cars[0][6] < 0 &&
                    avd_temp_cars[0][6] > avd_cars[1][6]) {
                  avd_cars[1][5] = avd_temp_cars[0][5];
                  avd_cars[1][6] = avd_temp_cars[0][6];
                } else if (avd_temp_cars[0][5] > 0 &&
                           avd_temp_cars[0][5] < avd_cars[0][5]) {
                  avd_cars[0][5] = avd_temp_cars[0][5];
                  avd_cars[0][6] = avd_temp_cars[0][6];
                }
              } else if (avd_cars[0][6] < 0 && avd_cars[1][5] > 0 &&
                         std::fabs(avd_temp_cars[0][3] - avd_cars[1][3]) <
                             5 * v_ego &&
                         std::fabs(avd_cars[1][3] - avd_cars[0][3]) <
                             3 * v_ego) {
                if (avd_temp_cars[0][6] < 0 &&
                    avd_temp_cars[0][6] > avd_cars[0][6]) {
                  avd_cars[0][5] = avd_temp_cars[0][5];
                  avd_cars[0][6] = avd_temp_cars[0][6];
                } else if (avd_temp_cars[0][5] > 0 &&
                           avd_temp_cars[0][5] < avd_cars[1][5]) {
                  avd_cars[1][5] = avd_temp_cars[0][5];
                  avd_cars[1][6] = avd_temp_cars[0][6];
                }
              }
            } else if (enter2) {
              avd_cars = avd_temp_cars;
            } else if (enter0) {
              if (avd_cars[0][6] < 0 && avd_cars[1][5] > 0 &&
                  avd_temp_cars[1][5] > 0 &&
                  avd_temp_cars[1][5] < avd_cars[1][5] &&
                  std::fabs(avd_temp_cars[1][3] - avd_cars[1][3]) < 5 * v_ego) {
                avd_cars[1][5] = avd_temp_cars[1][5];
                avd_cars[1][6] = avd_temp_cars[1][6];
              } else if (avd_cars[0][6] < 0 && avd_cars[1][6] < 0 &&
                         avd_temp_cars[1][5] > 0 &&
                         std::fabs(avd_cars[1][3] - avd_cars[0][3]) <
                             3 * v_ego &&
                         std::fabs(avd_temp_cars[1][3] - avd_cars[1][3]) <
                             5 * v_ego) {
                avd_cars[0][5] = std::max(avd_cars[0][5], avd_cars[1][5]);
                avd_cars[0][6] = std::max(avd_cars[0][6], avd_cars[1][6]);
                avd_cars[1] = avd_temp_cars[1];
              } else if (avd_cars[0][5] > 0 && avd_cars[1][5] > 0 &&
                         avd_temp_cars[1][6] < 0 &&
                         std::fabs(avd_cars[1][3] - avd_cars[0][3]) <
                             3 * v_ego &&
                         std::fabs(avd_temp_cars[1][3] - avd_cars[1][3]) <
                             5 * v_ego) {
                avd_cars[0][5] = std::min(avd_cars[0][5], avd_cars[1][5]);
                avd_cars[0][6] = std::min(avd_cars[0][6], avd_cars[1][6]);
                avd_cars[1] = avd_temp_cars[1];
              }
            } else if (enter1) {
              if (avd_cars[0][6] < 0 && avd_cars[1][5] > 0 &&
                  avd_temp_cars[1][5] > 0 &&
                  avd_temp_cars[1][5] < avd_cars[1][5] &&
                  std::fabs(avd_temp_cars[1][3] - avd_cars[1][3]) < 5 * v_ego) {
                avd_cars[1][5] = avd_temp_cars[1][5];
                avd_cars[1][6] = avd_temp_cars[1][6];
              } else if (avd_cars[0][6] < 0 && avd_cars[1][6] < 0 &&
                         avd_temp_cars[1][5] > 0 &&
                         std::fabs(avd_cars[1][3] - avd_cars[0][3]) <
                             5 * v_ego &&
                         std::fabs(avd_temp_cars[1][3] - avd_cars[1][3]) <
                             5 * v_ego) {
                avd_cars[0][5] = std::max(avd_cars[0][5], avd_cars[1][5]);
                avd_cars[0][6] = std::max(avd_cars[0][6], avd_cars[1][6]);
                avd_cars[1] = avd_temp_cars[1];
              } else if (avd_cars[0][5] > 0 && avd_cars[1][5] > 0 &&
                         avd_temp_cars[1][6] < 0 &&
                         std::fabs(avd_cars[1][3] - avd_cars[0][3]) <
                             5 * v_ego &&
                         std::fabs(avd_temp_cars[1][3] - avd_cars[1][3]) <
                             5 * v_ego) {
                avd_cars[0][5] = std::min(avd_cars[0][5], avd_cars[1][5]);
                avd_cars[0][6] = std::min(avd_cars[0][6], avd_cars[1][6]);
                avd_cars[1] = avd_temp_cars[1];
              }
            }
          }
        }
      }

      if (avd_cars.size() == 0 && avd_temp_cars.size() > 0) {
        if (avd_temp_cars.size() == 1 || avd_temp_cars.size() == 2) {
          avd_cars = avd_temp_cars;
        } else if (avd_temp_cars.size() > 2) {
          avd_cars.clear();
          avd_cars.push_back(avd_temp_cars[0]);
          avd_cars.push_back(avd_temp_cars[1]);
        }
      }
    }

    if ((state == INTER_TR_NONE || state == INTER_TL_NONE) &&
        intersection_cnt_ == 0) {
      avd_cars.clear();
      avd_car_past_[0].clear();
      avd_car_past_[1].clear();
      intersection_cnt_ = 1;
    }

    if (avd_cars.size() == 2) {
      if ((avd_car_past_[0].size() > 0 &&
           ((int)avd_cars[0][0] == (int)avd_car_past_[0][0] ||
            (int)avd_cars[1][0] == (int)avd_car_past_[0][0])) ||
          (avd_car_past_[0].size() > 0 && avd_car_past_[1].size() > 0 &&
           (((int)avd_cars[0][0] == (int)avd_car_past_[1][0] &&
             (int)avd_cars[1][0] == (int)avd_car_past_[0][0]) ||
            ((int)avd_cars[1][0] == (int)avd_car_past_[1][0] &&
             (int)avd_cars[0][0] != (int)avd_car_past_[0][0])))) {
        if (avd_car_past_[1].size() == 0) {
          if ((int)avd_cars[0][0] == (int)avd_car_past_[0][0]) {
            avd_car_past_[1] = avd_cars[1];
          } else if ((int)avd_cars[1][0] == (int)avd_car_past_[0][0]) {
            avd_car_past_[0] = avd_cars[0];
            avd_car_past_[1] = avd_cars[1];
          }
        }

        ncar_change_ += 1;
        if (ncar_change_ != 1) {
          avd_cars[0][7] = avd_car_past_[0][7];
          avd_cars[1][7] = avd_car_past_[1][7];
        }

        avd_car_past_[0] = avd_cars[0];
        avd_car_past_[1] = avd_cars[1];

        avd_car_past_[0][8] = curr_time;
        avd_car_past_[1][8] = curr_time;

        // if(avd_car_past_[0][2] != 0){
        if (equal_zero(avd_car_past_[0][2]) == false) {
          double d_avd_ncar1 = avd_car_past_[0][3] + 5.0 + safety_dist;
          t_avd_car_ = d_avd_ncar1 / (-avd_car_past_[0][2]);
        }
      }
    } else if (avd_cars.size() == 1 && avd_car_past_[0].size() > 0) {
      if ((int)avd_cars[0][0] == (int)avd_car_past_[0][0]) {
        ncar_change_ += 1;
        if (ncar_change_ != 1) {
          avd_cars[0][7] = avd_car_past_[0][7];
        }

        avd_car_past_[0] = avd_cars[0];
        avd_car_past_[0][8] = curr_time;

        if (avd_car_past_[1].size() > 0 && avd_car_past_[1][3] > 0 &&
            avd_car_past_[1][3] > avd_car_past_[0][3]) {
          avd_car_past_[1].clear();
        }
      }

      if (lead_one != nullptr && lead_one->is_avd_car == true &&
          lead_one->d_min_cpath != 100 && lead_one->d_max_cpath != 100 &&
          lead_one->type < 10000) {
        std::vector<double> avd_leadone;
        if (lateral_obstacle->front_tracks_copy().size() > 0) {
          for (auto &item : lateral_obstacle->front_tracks_copy()) {
            if (item.track_id == lead_one->track_id) {
              avd_leadone.assign({-1, (double)item.trajectory.intersection,
                                  lead_one->v_rel, lead_one->d_rel,
                                  lead_one->v_lat, lead_one->d_min_cpath,
                                  lead_one->d_max_cpath, lead_one->d_rel,
                                  curr_time, final_y_rel_});
              break;
            }
          }
        }

        if (std::fabs(lead_one->d_rel) >= avd_car_past_[0][3]) {
          avd_car_past_[1] = avd_leadone;
        } else {
          avd_car_past_[1] = avd_car_past_[0];
          avd_car_past_[0] = avd_leadone;
        }
      }

      if ((int)avd_cars[0][0] != (int)avd_car_past_[0][0]) {
        if (avd_car_past_[1].size() > 0) {
          if ((int)avd_cars[0][0] != (int)avd_car_past_[1][0]) {
            if (std::fabs(avd_cars[0][3]) < std::fabs(avd_car_past_[0][3])) {
              avd_car_past_[1] = avd_car_past_[0];
              avd_car_past_[0] = avd_cars[0];
            } else if (avd_car_past_[1][3] - avd_car_past_[0][3] > 15) {
              avd_car_past_[1] = avd_cars[0];
            }
          } else {
            ncar_change_ += 1;
            if (ncar_change_ != 1) {
              avd_cars[0][7] = avd_car_past_[1][7];
            }

            avd_car_past_[1] = avd_cars[0];
            avd_car_past_[1][8] = curr_time;

            if (std::fabs(avd_car_past_[1][3]) <
                std::fabs(avd_car_past_[0][3])) {
              std::swap(avd_car_past_[0], avd_car_past_[1]);
            }
          }
        } else {
          if (std::fabs(avd_cars[0][3]) < std::fabs(avd_car_past_[0][3])) {
            if (avd_car_past_[0][3] >= 0) {
              avd_car_past_[1] = avd_car_past_[0];
              avd_car_past_[0] = avd_cars[0];
            } else {
              avd_car_past_[0] = avd_cars[0];
              avd_car_past_[1].clear();
            }
          } else {
            avd_car_past_[1] = avd_cars[0];
          }
        }
      }

      // if(avd_car_past_[0][2] != 0){
      if (equal_zero(avd_car_past_[0][2]) == false) {
        double d_avd_ncar1 = avd_car_past_[0][3] + 5.0 + safety_dist;
        t_avd_car_ = d_avd_ncar1 / (-avd_car_past_[0][2]);
      }

      if (lead_one == nullptr || lead_one->is_avd_car == false) {
        if (avd_car_past_[1].size() > 0 && (int)avd_car_past_[1][0] == -1 &&
            curr_time - avd_car_past_[1][8] > 2.5) {
          avd_car_past_[1].clear();
        } else if (avd_car_past_[0].size() > 0 && avd_car_past_[1].size() > 0 &&
                   (int)avd_car_past_[0][0] == -1 &&
                   curr_time - avd_car_past_[0][8] > 2.5) {
          avd_car_past_[0] = avd_car_past_[1];
          avd_car_past_[1].clear();
        }
      }
    } else if (avd_cars.size() == 0 && avd_car_past_[0].size() > 0) {
      if (lead_one != nullptr && lead_one->is_avd_car &&
          (int)avd_car_past_[0][0] == -1 && lead_one->type < 10000) {
        std::vector<double> avd_leadone;
        if (lateral_obstacle->front_tracks_copy().size() > 0) {
          for (auto &item : lateral_obstacle->front_tracks_copy()) {
            if (item.track_id == lead_one->track_id) {
              avd_leadone.assign({-1, (double)item.trajectory.intersection,
                                  lead_one->v_rel, lead_one->d_rel,
                                  lead_one->v_lat, lead_one->d_min_cpath,
                                  lead_one->d_max_cpath, lead_one->d_rel,
                                  curr_time, final_y_rel_});
              break;
            }
          }
        }

        if (lead_one->d_max_cpath < 0) {
          avd_car_past_[0] = avd_leadone;
        } else if (lead_one->d_min_cpath != 100) {
          avd_car_past_[0] = avd_leadone;
        }

        avd_car_past_[1].clear();
      }

      // if(avd_car_past_[0][2] != 0){
      if (equal_zero(avd_car_past_[0][2]) == false) {
        double d_avd_ncar1 = avd_car_past_[0][3] + 5.0 + safety_dist;
        t_avd_car_ = d_avd_ncar1 / (-avd_car_past_[0][2]);

        if (t_avd_car_ < 3) {
          t_avd_car_ = 1;
        }
      }
    }

    if ((avd_cars.size() == 2 && avd_car_past_[0].size() > 0 &&
         avd_car_past_[1].size() > 0 &&
         (int)avd_cars[0][0] == (int)avd_car_past_[1][0] &&
         (int)avd_cars[1][0] != (int)avd_car_past_[0][0]) ||
        (avd_cars.size() == 1 && avd_car_past_[1].size() > 0 &&
         (int)avd_cars[0][0] == (int)avd_car_past_[1][0]) ||
        (avd_cars.size() == 1 && avd_car_past_[0].size() > 0 &&
         avd_car_past_[1].size() > 0 && lead_one != nullptr &&
         lead_one->is_avd_car == true) ||
        (avd_cars.size() == 0 && avd_car_past_[0].size() > 0)) {
      if (avd_cars.size() == 0 && lead_one != nullptr &&
          lead_one->d_rel <= avd_car_past_[0][3] + 1) {
        if ((lead_one->d_min_cpath != 100 && lead_one->d_max_cpath != 100 &&
             true &&
             (std::fabs(lead_one->d_min_cpath) < 1.3 ||
              std::fabs(lead_one->d_max_cpath) < 1.3)) ||
            lead_one->d_path <
                1.3) {  // TODO(Rui):map_info.is_in_intersection() == false
          if (avd_car_past_[0][5] > 0) {
            bool no_near_car = true;
            no_near_car = UpdateLSideAvdsInfo(no_near_car);
            no_near_car = UpdateLFrontAvdsInfo(no_near_car);
            no_near_car = UpdateRSideAvdsInfo(no_near_car);
            no_near_car = UpdateRFrontAvdsInfo(no_near_car);

            if (no_near_car == true) {
              avd_car_past_[0].clear();
              avd_car_past_[1].clear();
              avd_back_cnt_ = 0;
              flag_avd_ = 1;
              ncar_change_ = 0;
            }
          } else if (avd_car_past_[0][5] < 0) {
            bool no_near_car = true;
            no_near_car = UpdateRSideAvdsInfo(no_near_car);
            no_near_car = UpdateRFrontAvdsInfo(no_near_car);
            no_near_car = UpdateLSideAvdsInfo(no_near_car);
            no_near_car = UpdateLFrontAvdsInfo(no_near_car);

            if (no_near_car == true) {
              avd_car_past_[0].clear();
              avd_car_past_[1].clear();
              avd_back_cnt_ = 0;
              flag_avd_ = 1;
              ncar_change_ = 0;
            }
          }
        }
      }

      if (avd_cars.size() == 0 && lead_one != nullptr &&
          lead_one->is_avd_car == true) {
        if (avd_car_past_[0].size() > 0 &&
            lead_one->track_id == (int)avd_car_past_[0][0] &&
            lead_one->type < 10000) {
          if (lateral_obstacle->front_tracks_copy().size() > 0) {
            for (auto &item : lateral_obstacle->front_tracks_copy()) {
              if (item.track_id == lead_one->track_id) {
                avd_car_past_[0].assign(
                    {-1, (double)item.trajectory.intersection, lead_one->v_rel,
                     lead_one->d_rel, lead_one->v_lat, lead_one->d_min_cpath,
                     lead_one->d_max_cpath, lead_one->d_rel, curr_time,
                     final_y_rel_});
                break;
              }
            }
          }
        }

        // if(avd_car_past_[0].size() > 0 && avd_car_past_[0][2] != 0){
        if (avd_car_past_[0].size() > 0 &&
            equal_zero(avd_car_past_[0][2]) == false) {
          double d_avd_ncar1 = avd_car_past_[0][3] + 5.0 + safety_dist;
          t_avd_car_ = d_avd_ncar1 / (-avd_car_past_[0][2]);

          if (t_avd_car_ < 3) {
            t_avd_car_ = 1;
          }
        }
      }

      if (t_avd_car_ > 0 && t_avd_car_ <= 6) {
        t_avd_car_ += 2;
      } else if (t_avd_car_ <= 0) {
        t_avd_car_ = 2.5;
      } else {
        t_avd_car_ = 5.0;
      }

      if (avd_car_past_[0].size() > 0 &&
          (curr_time - avd_car_past_[0][8] > t_avd_car_ ||
           avd_car_past_[0][3] < -7)) {
        if (avd_car_past_[0][3] < 5.0 && avd_car_past_[0][5] > 0) {
          bool no_near_car = true;
          no_near_car = UpdateLSideAvdsInfo(no_near_car);
          no_near_car = UpdateLFrontAvdsInfo(no_near_car);

          if (avd_cars.size() == 0 && avd_car_past_[1].size() > 0 &&
              avd_car_past_[1][3] < 5.0 && avd_car_past_[1][5] < 0) {
            if (no_near_car) {
              avd_car_past_[0] = avd_car_past_[1];
              avd_car_past_[1].clear();
            }

            no_near_car = UpdateRSideAvdsInfo(no_near_car);
            no_near_car = UpdateRFrontAvdsInfo(no_near_car);
          }

          if (no_near_car == true) {
            if (avd_cars.size() == 1) {
              avd_car_past_[0] = avd_cars[0];
              if (avd_car_past_[1].size() > 0 &&
                  (int)avd_car_past_[1][0] == (int)avd_car_past_[0][0]) {
                avd_car_past_[1].clear();
              }
            } else if (avd_cars.size() == 2) {
              avd_car_past_[0] = avd_cars[0];
              avd_car_past_[1] = avd_cars[1];
            } else if (avd_cars.size() == 0) {
              avd_car_past_[0].clear();
              avd_car_past_[1].clear();
              avd_back_cnt_ = 0;
              flag_avd_ = 1;
            }

            ncar_change_ = 0;
          } else {
            if (avd_cars.size() > 0 &&
                (int)avd_cars[0][0] != (int)avd_car_past_[0][0]) {
              avd_car_past_[1] = avd_cars[0];
            } else if (lead_one != nullptr && avd_cars.size() == 0) {
              if (lead_one->is_avd_car == true &&
                  lead_one->d_min_cpath != 100 && lead_one->type < 10000) {
                if (lateral_obstacle->front_tracks_copy().size() > 0) {
                  for (auto &item : lateral_obstacle->front_tracks_copy()) {
                    if (item.track_id == lead_one->track_id) {
                      avd_car_past_[1].assign(
                          {-1, (double)item.trajectory.intersection,
                           lead_one->v_rel, lead_one->d_rel, lead_one->v_lat,
                           lead_one->d_min_cpath, lead_one->d_max_cpath,
                           lead_one->d_rel, curr_time, final_y_rel_});
                      break;
                    }
                  }
                }
              }
            }
          }
        } else if (avd_car_past_[0].size() > 0 && avd_car_past_[0][3] < 5.0 &&
                   avd_car_past_[0][5] < 0) {
          bool no_near_car = true;
          no_near_car = UpdateRSideAvdsInfo(no_near_car);
          no_near_car = UpdateRFrontAvdsInfo(no_near_car);

          if (avd_cars.size() == 0 && avd_car_past_[1].size() > 0 &&
              avd_car_past_[1][3] < 5.0 && avd_car_past_[1][5] > 0) {
            if (no_near_car) {
              avd_car_past_[0] = avd_car_past_[1];
              avd_car_past_[1].clear();
            }

            no_near_car = UpdateLSideAvdsInfo(no_near_car);
            no_near_car = UpdateLFrontAvdsInfo(no_near_car);
          }

          if (no_near_car == true) {
            if (avd_cars.size() == 1) {
              avd_car_past_[0] = avd_cars[0];
              if (avd_car_past_[1].size() > 0 &&
                  (int)avd_car_past_[1][0] == (int)avd_car_past_[0][0]) {
                avd_car_past_[1].clear();
              }
            } else if (avd_cars.size() == 2) {
              avd_car_past_[0] = avd_cars[0];
              avd_car_past_[1] = avd_cars[1];
            } else if (avd_cars.size() == 0) {
              avd_car_past_[0].clear();
              avd_car_past_[1].clear();
              avd_back_cnt_ = 0;
              flag_avd_ = 1;
            }

            ncar_change_ = 0;
          } else {
            if (avd_cars.size() > 0 &&
                (int)avd_cars[0][0] != (int)avd_car_past_[0][0]) {
              avd_car_past_[1] = avd_cars[0];
            } else if (lead_one != nullptr && avd_cars.size() == 0) {
              if (lead_one->is_avd_car == true &&
                  lead_one->d_min_cpath != 100 && lead_one->type < 10000) {
                if (lateral_obstacle->front_tracks_copy().size() > 0) {
                  for (auto &item : lateral_obstacle->front_tracks_copy()) {
                    if (item.track_id == lead_one->track_id) {
                      avd_car_past_[1].assign(
                          {-1, (double)item.trajectory.intersection,
                           lead_one->v_rel, lead_one->d_rel, lead_one->v_lat,
                           lead_one->d_min_cpath, lead_one->d_max_cpath,
                           lead_one->d_rel, curr_time, final_y_rel_});
                      break;
                    }
                  }
                }
              }
            }
          }
        } else if (avd_car_past_[0].size() > 0 && avd_car_past_[0][3] >= 5.0 &&
                   curr_time - avd_car_past_[0][8] > 2.5) {
          avd_car_past_[0].clear();
          avd_car_past_[1].clear();
          avd_back_cnt_ = 0;
          flag_avd_ = 1;
        }
      } else if (avd_car_past_[0].size() > 0) {
        if ((int)avd_car_past_[0][0] == 0 && avd_car_past_[0].size() == 11) {
          bool no_near_car = true;
          if (avd_car_past_[0][5] > 0) {
            no_near_car = UpdateLSideAvdsInfo(no_near_car);
            no_near_car = UpdateLFrontAvdsInfo(no_near_car);
          } else {
            no_near_car = UpdateRSideAvdsInfo(no_near_car);
            no_near_car = UpdateRFrontAvdsInfo(no_near_car);
          }

          if (no_near_car == true) {
            avd_car_past_[0][3] += 0.15 * avd_car_past_[0][2];
          }
        } else {
          avd_car_past_[0][3] += 0.05 * avd_car_past_[0][2];
          if (v_ego < 8) {
            avd_car_past_[0][2] = -v_ego;
          }
        }

        if (avd_car_past_[0][3] >= 5.0 &&
            (curr_time - avd_car_past_[0][8] > 2.5 ||
             (curr_time - avd_car_past_[0][8] > 1 &&
              avd_car_past_[0][2] > 1.5))) {
          if (avd_car_past_[1].size() > 0) {
            avd_car_past_[0] = avd_car_past_[1];
            avd_car_past_[1].clear();
          } else {
            avd_car_past_[0].clear();
            avd_back_cnt_ = 0;
            flag_avd_ = 1;
          }
        }
      }
    } else {
      if (avd_cars.size() == 2) {
        if ((avd_car_past_[0].size() > 0 && avd_car_past_[1].size() > 0 &&
             (int)avd_cars[0][0] != (int)avd_car_past_[0][0] &&
             (int)avd_cars[1][0] != (int)avd_car_past_[1][0] &&
             (int)avd_cars[0][0] != (int)avd_car_past_[1][0] &&
             (int)avd_cars[1][0] != (int)avd_car_past_[0][0]) ||
            (avd_car_past_[0].size() > 0 && avd_car_past_[1].size() == 0 &&
             (int)avd_cars[0][0] != (int)avd_car_past_[0][0] &&
             (int)avd_cars[1][0] != (int)avd_car_past_[0][0])) {
          avd_car_past_[0] = avd_cars[0];
          avd_car_past_[1] = avd_cars[1];
          ncar_change_ = 0;
        } else if (avd_car_past_[0].size() == 0) {
          avd_car_past_[0] = avd_cars[0];
          avd_car_past_[1] = avd_cars[1];
          ncar_change_ = 0;
        }
      } else if (avd_cars.size() == 1) {
        if (avd_car_past_[0].size() > 0 && avd_car_past_[1].size() > 0) {
          if ((int)avd_cars[0][0] != (int)avd_car_past_[0][0] &&
              (int)avd_cars[0][0] != (int)avd_car_past_[1][0]) {
            avd_car_past_[1] = avd_cars[0];
            ncar_change_ = 0;
          }
        } else if (avd_car_past_[0].size() == 0) {
          avd_car_past_[0] = avd_cars[0];
          avd_car_past_[1].clear();
          ncar_change_ = 0;
        } else if (avd_car_past_[0].size() > 0 &&
                   (int)avd_cars[0][0] != (int)avd_car_past_[0][0]) {
          if (avd_car_past_[0][3] > 5) {
            avd_car_past_[0] = avd_cars[0];
            avd_car_past_[1].clear();
            ncar_change_ = 0;
          } else {
            avd_car_past_[1] = avd_cars[0];
          }
        }
      } else if (avd_cars.size() == 0) {
        if (lead_one != nullptr) {
          if (lead_one->is_avd_car == true && lead_one->type < 10000) {
            if (avd_leadone_ == 0 ||
                (avd_leadone_ == 50 && pre_leadone_id_ != lead_one->track_id)) {
              pre_leadone_id_ = lead_one->track_id;
              avd_leadone_ = 0;
            }

            if (avd_leadone_ < 50) {
              if (pre_leadone_id_ == lead_one->track_id) {
                avd_leadone_ += 1;
              } else {
                avd_leadone_ = 0;
              }
            }

            if (avd_leadone_ == 50 && lead_one->d_min_cpath != 100) {
              if (lateral_obstacle->front_tracks_copy().size() > 0) {
                for (auto &item : lateral_obstacle->front_tracks_copy()) {
                  if (item.track_id == lead_one->track_id) {
                    avd_car_past_[0].assign(
                        {-1, (double)item.trajectory.intersection,
                         lead_one->v_rel, lead_one->d_rel, lead_one->v_lat,
                         lead_one->d_min_cpath, lead_one->d_max_cpath,
                         lead_one->d_rel, curr_time, final_y_rel_});
                    break;
                  }
                }
              }

              avd_car_past_[1].clear();
            }
          } else {
            avd_leadone_ = 0;
          }
        } else {
          avd_car_past_[0].clear();
          avd_car_past_[1].clear();
        }
      }
    }

    if (avd_car_past_[0].size() > 0) {
      avd_car_past_[0][9] = UpdateAntsidesStrict();
    }

    if (flag_avd_ == 1) {
      avd_back_cnt_ += 1;
      if (avd_back_cnt_ == 30) {
        flag_avd_ = 0;
        avd_back_cnt_ = 0;
      }
    }
  }
  for (auto avd_car : avd_car_past_) {
    if (avd_car.size() != 0) LOG_DEBUG("avd_car id :%d ", (int)avd_car[0]);
  }

  return true;
}

}  // namespace planning
