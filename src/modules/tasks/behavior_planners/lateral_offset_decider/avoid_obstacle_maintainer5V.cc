#include "avoid_obstacle_maintainer5V.h"
#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>
#include "../../common/planning_gflags.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "ifly_time.h"
#include "planning_context.h"
#include "tasks/behavior_planners/lateral_offset_decider/lateral_offset_decider_utils.h"
#include "virtual_lane_manager.h"

static double curr_time = 0;
static const double safety_dist_level_1 = 1.0;
static const double safety_dist_level_2 = 2.0;
static const double keep_time_level_1 = 0.8;
static const double keep_time_level_2 = 1.3;
static const double keep_time_level_3 = 0.3;
namespace planning {

bool AvoidObstacleMaintainer5V::UpdateLFrontAvdsInfo(
    bool no_near_car, AvoidObstacleInfo &avd_obstacle1,
    AvoidObstacleInfo &avd_obstacle2) {
  auto &lateral_obstacle =
      session_->mutable_environmental_model()->get_lateral_obstacle();
  auto &ego_state =
      session_->mutable_environmental_model()->get_ego_state_manager();
  double v_ego = ego_state->ego_v();
  if (no_near_car == false ||
      lateral_obstacle->front_tracks_copy().size() == 0) {
    return no_near_car;
  }

  // TODO: 考虑加视觉障碍物的过滤
  for (auto &tr : lateral_obstacle->front_tracks_copy()) {
    if (!(tr.fusion_source & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    if (avd_obstacle2.flag != AvoidObstacleFlag::INVALID &&
        avd_obstacle2.track_id == tr.track_id) {
      continue;
    }
    if (tr.d_rel < 8 &&
        ((tr.d_min_cpath > 1.0 && tr.d_min_cpath < lane_width_ - 1.1) ||
         (tr.d_path > 1.0 && tr.d_path < 1.4)) &&
        !(tr.d_rel + ego_length_ < safety_dist_level_1 && tr.v_rel < -1)) {
      avd_obstacle1.flag = AvoidObstacleFlag::SIDE;
      avd_obstacle1.update_flag = AvoidObstacleUpdateFlag::Update;
      avd_obstacle1.curr_time = curr_time;
      avd_obstacle1.track_id = tr.track_id;
      avd_obstacle1.s_to_ego = tr.d_rel;
      avd_obstacle1.tail_s_to_ego = tr.tail_rel_s;
      avd_obstacle1.vs_lon_relative = tr.v_rel;
      avd_obstacle1.predict_vs_lon_relative = tr.v_rel;
      avd_obstacle1.vs = tr.v_lead;
      avd_obstacle1.length = tr.length;

      if (v_ego > 60 / 3.6 && fabs(tr.v_rel) > 1.5 && tr.d_rel <= 0) {
      } else {
        if (std::fabs(avd_obstacle1.min_l_to_ref - tr.d_min_cpath) > 0.1) {
          avd_obstacle1.intersection_index = tr.trajectory.intersection;
          avd_obstacle1.vs_lat_relative = tr.v_lat;
          avd_obstacle1.min_l_to_ref = tr.d_min_cpath;
          avd_obstacle1.max_l_to_ref = tr.d_max_cpath;
        }
      }

      return false;
    }
  }

  return no_near_car;
}

bool AvoidObstacleMaintainer5V::UpdateRFrontAvdsInfo(
    bool no_near_car, AvoidObstacleInfo &avd_obstacle1,
    AvoidObstacleInfo &avd_obstacle2) {
  auto &lateral_obstacle =
      session_->mutable_environmental_model()->get_lateral_obstacle();
  auto &ego_state =
      session_->mutable_environmental_model()->get_ego_state_manager();
  double v_ego = ego_state->ego_v();
  if (no_near_car == false ||
      lateral_obstacle->front_tracks_copy().size() == 0) {
    return no_near_car;
  }

  for (auto &tr : lateral_obstacle->front_tracks_copy()) {
    if (!(tr.fusion_source & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    if (avd_obstacle2.flag != AvoidObstacleFlag::INVALID &&
        avd_obstacle2.track_id == tr.track_id) {
      continue;
    }
    if (tr.d_rel < 8 &&
        ((tr.d_max_cpath > -(lane_width_ - 1.1) && tr.d_max_cpath < -1.0) ||
         (tr.d_path > 1.0 && tr.d_path < 1.4)) &&
        !(tr.d_rel + ego_length_ < safety_dist_level_1 && tr.v_rel < -1)) {
      avd_obstacle1.flag = AvoidObstacleFlag::SIDE;
      avd_obstacle1.update_flag = AvoidObstacleUpdateFlag::Update;
      avd_obstacle1.curr_time = curr_time;
      avd_obstacle1.track_id = tr.track_id;
      avd_obstacle1.s_to_ego = tr.d_rel;
      avd_obstacle1.tail_s_to_ego = tr.tail_rel_s;
      avd_obstacle1.vs_lon_relative = tr.v_rel;
      avd_obstacle1.predict_vs_lon_relative = tr.v_rel;
      avd_obstacle1.vs = tr.v_lead;
      avd_obstacle1.length = tr.length;

      if (v_ego > 60 / 3.6 && fabs(tr.v_rel) > 1.5 && tr.d_rel <= 0) {
      } else {
        if (std::fabs(avd_obstacle1.min_l_to_ref - tr.d_min_cpath) > 0.1) {
          avd_obstacle1.intersection_index = tr.trajectory.intersection;
          avd_obstacle1.vs_lat_relative = tr.v_lat;
          avd_obstacle1.min_l_to_ref = tr.d_min_cpath;
          avd_obstacle1.max_l_to_ref = tr.d_max_cpath;
        }
      }

      return false;
    }
  }

  return no_near_car;
}

bool AvoidObstacleMaintainer5V::UpdateLSideAvdsInfo(
    bool no_near_car, AvoidObstacleInfo &avd_obstacle1,
    AvoidObstacleInfo &avd_obstacle2) {
  if (no_near_car == false) {
    return no_near_car;
  }

  auto &lateral_obstacle =
      session_->mutable_environmental_model()->get_lateral_obstacle();
  auto &ego_state =
      session_->mutable_environmental_model()->get_ego_state_manager();
  double v_ego = ego_state->ego_v();
  for (auto &tr : lateral_obstacle->side_tracks_l()) {
    if (!(tr.fusion_source & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    if (avd_obstacle2.flag != AvoidObstacleFlag::INVALID &&
        avd_obstacle2.track_id == tr.track_id) {
      continue;
    }
    if (tr.d_rel + ego_length_ > -safety_dist_level_2 &&
        ((tr.d_min_cpath > 1.0 && tr.d_min_cpath < lane_width_ - 1.1) ||
         (tr.d_path > 1.0 && tr.d_path < 1.7)) &&
        !(tr.d_rel + ego_length_ < safety_dist_level_1 && tr.v_rel < -1)) {
      avd_obstacle1.flag = AvoidObstacleFlag::SIDE;
      avd_obstacle1.update_flag = AvoidObstacleUpdateFlag::Update;
      avd_obstacle1.curr_time = curr_time;
      avd_obstacle1.track_id = tr.track_id;
      avd_obstacle1.s_to_ego = tr.d_rel;  // 需要 -2/length
      avd_obstacle1.tail_s_to_ego = tr.tail_rel_s;
      avd_obstacle1.vs_lon_relative = tr.v_rel;
      avd_obstacle1.predict_vs_lon_relative = tr.v_rel;
      avd_obstacle1.vs = tr.v_lead;
      avd_obstacle1.length = tr.length;
      if (v_ego > 60 / 3.6 && fabs(tr.v_rel) > 1.5 && tr.d_rel <= 0) {
      } else {
        if (std::fabs(avd_obstacle1.min_l_to_ref - tr.d_min_cpath) > 0.1 &&
            avd_obstacle1.s_to_ego + ego_length_ >
                -safety_dist_level_1) {  // 可考虑区分融合类型
          avd_obstacle1.intersection_index = tr.trajectory.intersection;
          avd_obstacle1.vs_lat_relative = tr.v_lat;
          avd_obstacle1.min_l_to_ref = tr.d_min_cpath;
          avd_obstacle1.max_l_to_ref = tr.d_max_cpath;
        }
      }
      return false;
    }
  }

  return no_near_car;
}

bool AvoidObstacleMaintainer5V::UpdateRSideAvdsInfo(
    bool no_near_car, AvoidObstacleInfo &avd_obstacle1,
    AvoidObstacleInfo &avd_obstacle2) {
  if (no_near_car == false) {
    return no_near_car;
  }

  auto &lateral_obstacle =
      session_->mutable_environmental_model()->get_lateral_obstacle();
  auto &ego_state =
      session_->mutable_environmental_model()->get_ego_state_manager();
  double v_ego = ego_state->ego_v();
  for (auto &tr : lateral_obstacle->side_tracks_r()) {
    if (!(tr.fusion_source & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    if (avd_obstacle2.flag != AvoidObstacleFlag::INVALID &&
        avd_obstacle2.track_id == tr.track_id) {
      continue;
    }
    if (tr.d_rel + ego_length_ > -safety_dist_level_2 &&
        ((tr.d_max_cpath > -(lane_width_ - 1.1) && tr.d_max_cpath < -1.0) ||
         (tr.d_path > 1.0 && tr.d_path < 1.7)) &&
        !(tr.d_rel + ego_length_ < safety_dist_level_1 &&
          tr.v_rel < -1)) {  // 如果是角雷达的话，考虑速度
      avd_obstacle1.flag = AvoidObstacleFlag::SIDE;
      avd_obstacle1.update_flag = AvoidObstacleUpdateFlag::
          Update;  // 根据实际效果选择是否持续跟踪角雷达!!!
      avd_obstacle1.curr_time = curr_time;
      avd_obstacle1.track_id = tr.track_id;
      avd_obstacle1.s_to_ego = tr.d_rel;
      avd_obstacle1.tail_s_to_ego = tr.tail_rel_s;
      avd_obstacle1.vs_lon_relative = tr.v_rel;
      avd_obstacle1.predict_vs_lon_relative = tr.v_rel;
      avd_obstacle1.vs = tr.v_lead;
      avd_obstacle1.length = tr.length;

      if (v_ego > 60 / 3.6 && fabs(tr.v_rel) > 1.5 && tr.d_rel <= 0) {
      } else {
        if (std::fabs(avd_obstacle1.min_l_to_ref - tr.d_min_cpath) > 0.1 &&
            avd_obstacle1.s_to_ego + ego_length_ > -safety_dist_level_1) {
          avd_obstacle1.intersection_index = tr.trajectory.intersection;
          avd_obstacle1.vs_lat_relative = tr.v_lat;
          avd_obstacle1.min_l_to_ref = tr.d_min_cpath;
          avd_obstacle1.max_l_to_ref = tr.d_max_cpath;
        }
      }

      return false;
    }
  }

  return no_near_car;
}

void AvoidObstacleMaintainer5V::Reset() {
  for (int i = 0; i < avd_obstacles_.size(); i++) {
    avd_obstacles_[i].Reset();
  }
  for (int i = 0; i < avd_obstacles_history_.size(); i++) {
    avd_obstacles_history_[i].Reset();
  }
  for (int i = 0; i < avd_obstacles_.size(); i++) {
    avd_sp_obstacles_[i].Reset();
  }
  avd_obstacles_cur_.clear();
  is_ncar_ = false;
  final_y_rel_ = 10;
  flag_avd_ = 0;
  avd_back_start_time_ = 0;
  dist_rblane_ = 0;

  SaveDebugInfo();
}

bool AvoidObstacleMaintainer5V::IsOutAvoidArea(
    const std::shared_ptr<LateralObstacle> lateral_obstacle,
    AvoidObstacleInfo &avd_obstacle1) {
  int history_num_out_avd_area = avd_obstacle1.num_out_avd_area;
  avd_obstacle1.num_out_avd_area = 0;
  for (auto &tr : lateral_obstacle->side_tracks()) {
    if (tr.track_id == avd_obstacle1.track_id) {
      if (tr.d_rel + ego_length_ <= -safety_dist_level_1) {
        avd_obstacle1.num_out_avd_area = history_num_out_avd_area + 1;
        return true;
      } else {
        return false;
      }
    }
  }
  return false;
}

void AvoidObstacleMaintainer5V::UpdateAvoidObstacle(
    const std::shared_ptr<LateralObstacle> lateral_obstacle) {
  // check avd_obstacles_ is deleted according to front_tracks' info
  // found it ,but is_avd_obstacle is false
  // keep_time_level: keep stability
  const auto &lateral_obstacle_history_info =
      session_->mutable_planning_context()
          ->lateral_obstacle_decider_output()
          .lateral_obstacle_history_info;
  TrackedObject *lead_one = lateral_obstacle->leadone();

  if (avd_obstacles_[0].flag != AvoidObstacleFlag::INVALID) {
    TrackedObject tr;
    bool is_found =
        lateral_obstacle->find_track(avd_obstacles_[0].track_id, tr);
    if (is_found && ((tr.fusion_source & OBSTACLE_SOURCE_CAMERA) &&
                     tr.d_rel > 0)) {  // TODO filter
      double diff_time = curr_time - avd_obstacles_[0].curr_time;
      bool is_avd_car = false;
      auto lateral_obstacle_iter =
          lateral_obstacle_history_info.find(tr.track_id);
      if (lateral_obstacle_iter != lateral_obstacle_history_info.end()) {
        is_avd_car = lateral_obstacle_iter->second.is_avd_car;
      }
      if (!is_avd_car &&
          (diff_time > keep_time_level_2 ||
           (diff_time > keep_time_level_1 &&
            avd_obstacles_[0].vs_lon_relative > 1.5) ||  //
           (lead_one != nullptr && lead_one->track_id == tr.track_id &&
            diff_time >
                keep_time_level_3))) {  // 可根据更多的障碍物特性更改维持的避让时间
        avd_obstacles_[0].Reset();
      }  // 考虑利用tr更新avd_obstacles_数据
    }

    if (avd_obstacles_[1].flag != AvoidObstacleFlag::INVALID) {
      is_found = lateral_obstacle->find_track(avd_obstacles_[0].track_id, tr);
      if (is_found &&
          ((tr.fusion_source & OBSTACLE_SOURCE_CAMERA) && tr.d_rel > 0)) {
        double diff_time = curr_time - avd_obstacles_[1].curr_time;
        bool is_avd_car = false;
        auto lateral_obstacle_iter =
            lateral_obstacle_history_info.find(tr.track_id);
        if (lateral_obstacle_iter != lateral_obstacle_history_info.end()) {
          is_avd_car = lateral_obstacle_iter->second.is_avd_car;
        }
        if (!is_avd_car &&
            (curr_time - avd_obstacles_[1].curr_time > keep_time_level_2 ||
             (diff_time > keep_time_level_1 &&
              avd_obstacles_[1].vs_lon_relative > 1.5) ||  //
             (lead_one != nullptr && lead_one->track_id == tr.track_id &&
              diff_time > keep_time_level_3))) {
          avd_obstacles_[1].Reset();
        }
      }
    }
  }

  if (avd_obstacles_[0].flag == AvoidObstacleFlag::INVALID &&
      avd_obstacles_[1].flag != AvoidObstacleFlag::INVALID) {
    avd_obstacles_[0] = avd_obstacles_[1];
    avd_obstacles_[1].Reset();
  }
}

void AvoidObstacleMaintainer5V::SelectCurAvoidObstacles(
    const std::shared_ptr<LateralObstacle> lateral_obstacle, double v_ego,
    std::vector<AvoidObstacleInfo> &avd_obstacles) {
  int ncar_cnt = 0;
  int enter0 = 0;
  int enter1 = 0;
  int enter2 = 0;
  double half_width = lane_width_ * 0.5;
  std::vector<AvoidObstacleInfo> avd_temp_cars;
  const auto &lateral_obstacle_history_info =
      session_->mutable_planning_context()
          ->lateral_obstacle_decider_output()
          .lateral_obstacle_history_info;
  if (lateral_obstacle->front_tracks_copy().size() > 0) {
    for (auto &tr : lateral_obstacle->front_tracks_copy()) {
      bool is_avd_car = false;
      auto lateral_obstacle_iter =
          lateral_obstacle_history_info.find(tr.track_id);
      if (lateral_obstacle_iter != lateral_obstacle_history_info.end()) {
        is_avd_car = lateral_obstacle_iter->second.is_avd_car;
      }
      if (is_avd_car == true && ncar_cnt < 3) {
        ncar_cnt += 1;
        if (!(tr.type == iflyauto::OBJECT_TYPE_COUPE ||
              tr.type == iflyauto::OBJECT_TYPE_MINIBUS ||
              tr.type == iflyauto::OBJECT_TYPE_VAN ||
              tr.type == iflyauto::OBJECT_TYPE_BUS ||
              tr.type == iflyauto::OBJECT_TYPE_TRUCK ||
              tr.type == iflyauto::OBJECT_TYPE_TRAILER ||
              tr.type == iflyauto::OBJECT_TYPE_TRICYCLE)) {
          if (tr.d_min_cpath > 0) {
            if (tr.type != iflyauto::OBJECT_TYPE_TRAFFIC_CONE) {
              // length 后续调整
              avd_temp_cars.emplace_back(AvoidObstacleInfo(
                  AvoidObstacleFlag::NORMAL, 0, tr.v_lead, tr.v_rel, tr.v_rel,
                  tr.d_rel - 3, tr.tail_rel_s - 3, tr.v_lat,
                  tr.d_min_cpath - 0.3, tr.d_max_cpath - 0.3, tr.d_rel,
                  curr_time, MAX_T_EXCEED_AVD_CAR, final_y_rel_, tr.track_id,
                  tr.type, AvoidObstacleUpdateFlag::Update, 5.0, 0, false));
            } else {
              avd_temp_cars.emplace_back(AvoidObstacleInfo(
                  AvoidObstacleFlag::NORMAL, 0, tr.v_lead, tr.v_rel, tr.v_rel,
                  tr.d_rel - 3, tr.tail_rel_s - 3, tr.v_lat,
                  tr.d_min_cpath + 0.2, tr.d_max_cpath + 0.2, tr.d_rel,
                  curr_time, MAX_T_EXCEED_AVD_CAR, final_y_rel_, tr.track_id,
                  tr.type, AvoidObstacleUpdateFlag::Update, 5.0, 0, false));
            }
          } else {
            if (tr.type != iflyauto::OBJECT_TYPE_TRAFFIC_CONE) {
              avd_temp_cars.emplace_back(AvoidObstacleInfo(
                  AvoidObstacleFlag::NORMAL, 0, tr.v_lead, tr.v_rel, tr.v_rel,
                  tr.d_rel - 3, tr.tail_rel_s - 3, tr.v_lat,
                  tr.d_min_cpath + 0.3, tr.d_max_cpath + 0.3, tr.d_rel,
                  curr_time, MAX_T_EXCEED_AVD_CAR, final_y_rel_, tr.track_id,
                  tr.type, AvoidObstacleUpdateFlag::Update, 5.0, 0, false));
            } else {
              avd_temp_cars.emplace_back(AvoidObstacleInfo(
                  AvoidObstacleFlag::NORMAL, 0, tr.v_lead, tr.v_rel, tr.v_rel,
                  tr.d_rel - 3, tr.tail_rel_s - 3, tr.v_lat,
                  tr.d_min_cpath - 0.2, tr.d_max_cpath - 0.2, tr.d_rel,
                  curr_time, MAX_T_EXCEED_AVD_CAR, final_y_rel_, tr.track_id,
                  tr.type, AvoidObstacleUpdateFlag::Update, 0.3, 0, false));
            }
          }

          if (avd_temp_cars.size() > 1) {
            if (std::fabs(avd_temp_cars[1].s_to_ego -
                          avd_temp_cars[0].s_to_ego) < 3 * v_ego) {
              if (((avd_temp_cars[0].min_l_to_ref >
                        std::min(2.4 - half_width, 0.) &&
                    avd_temp_cars[1].min_l_to_ref >
                        std::min(lane_width_ - 2.4, 0.)) ||
                   (avd_temp_cars[0].max_l_to_ref <
                        std::max(half_width - 2.4, 0.) &&
                    avd_temp_cars[1].max_l_to_ref <
                        std::max(half_width - 2.4, 0.))) &&
                  true) {  // TODO(Rui):map_info.is_in_intersection() == false
                if (std::fabs(avd_temp_cars[0].min_l_to_ref) >
                    std::fabs(avd_temp_cars[1].min_l_to_ref)) {
                  avd_temp_cars[0].min_l_to_ref = avd_temp_cars[1].min_l_to_ref;
                  avd_temp_cars[0].max_l_to_ref = avd_temp_cars[1].max_l_to_ref;
                }

                avd_temp_cars.pop_back();
                ncar_cnt -= 1;
              } else if (avd_temp_cars[0].min_l_to_ref > 0 &&
                         avd_temp_cars[1].min_l_to_ref < 0) {
                if (avd_temp_cars.size() > 2 &&
                    std::fabs(avd_temp_cars[2].s_to_ego -
                              avd_temp_cars[1].s_to_ego) < 3 * v_ego) {
                  if (avd_temp_cars[2].min_l_to_ref > 0 &&
                      avd_temp_cars[2].min_l_to_ref <
                          avd_temp_cars[0].min_l_to_ref) {
                    avd_temp_cars[0].min_l_to_ref =
                        avd_temp_cars[2].min_l_to_ref;
                    avd_temp_cars[0].max_l_to_ref =
                        avd_temp_cars[2].max_l_to_ref;
                  } else if (avd_temp_cars[2].max_l_to_ref < 0 &&
                             avd_temp_cars[2].max_l_to_ref >
                                 avd_temp_cars[1].max_l_to_ref) {
                    avd_temp_cars[1].min_l_to_ref =
                        avd_temp_cars[2].min_l_to_ref;
                    avd_temp_cars[1].max_l_to_ref =
                        avd_temp_cars[2].max_l_to_ref;
                  }

                  avd_temp_cars.pop_back();
                }

                ncar_cnt -= 1;
              } else if (avd_temp_cars[0].min_l_to_ref < 0 &&
                         avd_temp_cars[1].min_l_to_ref > 0) {
                if (avd_temp_cars.size() > 2 &&
                    std::fabs(avd_temp_cars[2].s_to_ego -
                              avd_temp_cars[1].s_to_ego) < 3 * v_ego) {
                  if (avd_temp_cars[2].min_l_to_ref > 0 &&
                      avd_temp_cars[2].min_l_to_ref <
                          avd_temp_cars[1].min_l_to_ref) {
                    avd_temp_cars[1].min_l_to_ref =
                        avd_temp_cars[2].min_l_to_ref;
                    avd_temp_cars[1].max_l_to_ref =
                        avd_temp_cars[2].max_l_to_ref;
                  } else if (avd_temp_cars[2].max_l_to_ref < 0 &&
                             avd_temp_cars[2].max_l_to_ref >
                                 avd_temp_cars[0].max_l_to_ref) {
                    avd_temp_cars[0].min_l_to_ref =
                        avd_temp_cars[2].min_l_to_ref;
                    avd_temp_cars[0].max_l_to_ref =
                        avd_temp_cars[2].max_l_to_ref;
                  }

                  avd_temp_cars.pop_back();
                }

                ncar_cnt -= 1;
              }
            }
          }
        } else {
          if (avd_obstacles.size() < 2) {
            avd_obstacles.emplace_back(AvoidObstacleInfo(
                AvoidObstacleFlag::NORMAL, tr.trajectory.intersection,
                tr.v_lead, tr.v_rel, tr.v_rel, tr.d_rel, tr.tail_rel_s,
                tr.v_lat, tr.d_min_cpath, tr.d_max_cpath, tr.d_rel, curr_time,
                MAX_T_EXCEED_AVD_CAR, final_y_rel_, tr.track_id, tr.type,
                AvoidObstacleUpdateFlag::Update, tr.length, 0, false));
          }
        }

        if (avd_temp_cars.size() == 1 && avd_obstacles.size() == 1) {
          if (avd_temp_cars[0].s_to_ego <= avd_obstacles[0].s_to_ego) {
            avd_obstacles.insert(avd_obstacles.begin(), avd_temp_cars[0]);
            enter0 = 1;
          } else {
            avd_obstacles.push_back(avd_temp_cars[0]);
            enter1 = 1;
          }
        } else if (avd_temp_cars.size() == 1 && avd_obstacles.size() == 2) {
          if (!enter0 && !enter1) {
            if ((avd_obstacles[0].min_l_to_ref > 0 &&
                 avd_obstacles[1].min_l_to_ref > 0 &&
                 avd_temp_cars[0].min_l_to_ref < 0) ||
                (avd_obstacles[0].max_l_to_ref < 0 &&
                 avd_obstacles[1].max_l_to_ref < 0 &&
                 avd_temp_cars[0].max_l_to_ref > 0)) {
              if (std::fabs(avd_temp_cars[0].s_to_ego -
                            avd_obstacles[1].s_to_ego) < 5 * v_ego) {
                if (std::fabs(avd_obstacles[0].min_l_to_ref) >
                    std::fabs(avd_obstacles[1].min_l_to_ref)) {
                  avd_obstacles[0].min_l_to_ref = avd_obstacles[1].min_l_to_ref;
                  avd_obstacles[0].max_l_to_ref = avd_obstacles[1].max_l_to_ref;
                  avd_obstacles[1] = avd_temp_cars[0];
                }
              }
            } else if (avd_obstacles[0].min_l_to_ref > 0 &&
                       avd_obstacles[1].max_l_to_ref < 0 &&
                       std::fabs(avd_temp_cars[0].s_to_ego -
                                 avd_obstacles[1].s_to_ego) < 5 * v_ego &&
                       std::fabs(avd_obstacles[1].s_to_ego -
                                 avd_obstacles[0].s_to_ego) < 3 * v_ego) {
              if (avd_temp_cars[0].max_l_to_ref < 0 &&
                  avd_temp_cars[0].max_l_to_ref >
                      avd_obstacles[1].max_l_to_ref) {
                avd_obstacles[1].min_l_to_ref = avd_temp_cars[0].min_l_to_ref;
                avd_obstacles[1].max_l_to_ref = avd_temp_cars[0].max_l_to_ref;
              } else if (avd_temp_cars[0].min_l_to_ref > 0 &&
                         avd_temp_cars[0].min_l_to_ref <
                             avd_obstacles[0].min_l_to_ref) {
                avd_obstacles[0].min_l_to_ref = avd_temp_cars[0].min_l_to_ref;
                avd_obstacles[0].max_l_to_ref = avd_temp_cars[0].max_l_to_ref;
              }
            } else if (avd_obstacles[0].max_l_to_ref < 0 &&
                       avd_obstacles[1].min_l_to_ref > 0 &&
                       std::fabs(avd_temp_cars[0].s_to_ego -
                                 avd_obstacles[1].s_to_ego) < 5 * v_ego &&
                       std::fabs(avd_obstacles[1].s_to_ego -
                                 avd_obstacles[0].s_to_ego) < 3 * v_ego) {
              if (avd_temp_cars[0].max_l_to_ref < 0 &&
                  avd_temp_cars[0].max_l_to_ref >
                      avd_obstacles[0].max_l_to_ref) {
                avd_obstacles[0].min_l_to_ref = avd_temp_cars[0].min_l_to_ref;
                avd_obstacles[0].max_l_to_ref = avd_temp_cars[0].max_l_to_ref;
              } else if (avd_temp_cars[0].min_l_to_ref > 0 &&
                         avd_temp_cars[0].min_l_to_ref <
                             avd_obstacles[1].min_l_to_ref) {
                avd_obstacles[1].min_l_to_ref = avd_temp_cars[0].min_l_to_ref;
                avd_obstacles[1].max_l_to_ref = avd_temp_cars[0].max_l_to_ref;
              }
            }
          } else if (enter0 && std::fabs(avd_obstacles[0].min_l_to_ref) >
                                   std::fabs(avd_temp_cars[0].min_l_to_ref)) {
            avd_obstacles[0].min_l_to_ref = avd_temp_cars[0].min_l_to_ref;
            avd_obstacles[0].max_l_to_ref = avd_temp_cars[0].max_l_to_ref;
          } else if (enter1 && std::fabs(avd_obstacles[1].min_l_to_ref) >
                                   std::fabs(avd_temp_cars[0].min_l_to_ref)) {
            avd_obstacles[1].min_l_to_ref = avd_temp_cars[0].min_l_to_ref;
            avd_obstacles[1].max_l_to_ref = avd_temp_cars[0].max_l_to_ref;
          }
        } else if (avd_temp_cars.size() == 2 && avd_obstacles.size() == 1) {
          if (avd_temp_cars[0].s_to_ego <=
              avd_obstacles[0].s_to_ego) {  //似乎必然满足？？？？？
            avd_obstacles.push_back(avd_temp_cars[1]);
            avd_obstacles[0] = avd_temp_cars[0];
            enter2 = 1;
          } else if (avd_obstacles[0].max_l_to_ref < 0 &&
                     avd_temp_cars[0].min_l_to_ref > 0 &&
                     (avd_temp_cars[0].s_to_ego - avd_obstacles[0].s_to_ego) <
                         5 * v_ego) {
            avd_obstacles.push_back(avd_temp_cars[0]);
          } else if (avd_obstacles[0].max_l_to_ref < 0 &&
                     avd_temp_cars[0].max_l_to_ref < 0 &&
                     avd_temp_cars[1].min_l_to_ref > 0 &&
                     avd_temp_cars[1].s_to_ego < 5 * v_ego) {
            avd_obstacles.push_back(avd_temp_cars[1]);
          } else if (avd_obstacles[0].min_l_to_ref > 0 &&
                     avd_temp_cars[0].max_l_to_ref < 0 &&
                     avd_temp_cars[0].s_to_ego < 5 * v_ego) {
            avd_obstacles.push_back(avd_temp_cars[0]);
          } else if (avd_obstacles[0].min_l_to_ref > 0 &&
                     avd_temp_cars[0].min_l_to_ref > 0 &&
                     avd_temp_cars[1].max_l_to_ref < 0 &&
                     avd_temp_cars[1].s_to_ego < 5 * v_ego) {
            avd_obstacles.push_back(avd_temp_cars[1]);
          }
        } else if (avd_temp_cars.size() == 2 && avd_obstacles.size() == 2) {
          if (!enter0 && !enter1 &&
              avd_temp_cars[0].s_to_ego >= avd_obstacles[1].s_to_ego) {
            if ((avd_obstacles[0].min_l_to_ref > 0 &&
                 avd_obstacles[1].min_l_to_ref > 0 &&
                 avd_temp_cars[0].min_l_to_ref < 0) ||
                (avd_obstacles[0].max_l_to_ref < 0 &&
                 avd_obstacles[1].max_l_to_ref < 0 &&
                 avd_temp_cars[0].max_l_to_ref > 0)) {
              if (std::fabs(avd_temp_cars[0].s_to_ego -
                            avd_obstacles[1].s_to_ego) < 5 * v_ego) {
                if (std::fabs(avd_obstacles[0].min_l_to_ref) >
                        std::fabs(avd_obstacles[1].min_l_to_ref) &&
                    std::fabs(avd_obstacles[1].s_to_ego -
                              avd_obstacles[0].s_to_ego) < 3 * v_ego) {
                  avd_obstacles[0].min_l_to_ref = avd_obstacles[1].min_l_to_ref;
                  avd_obstacles[0].max_l_to_ref = avd_obstacles[1].max_l_to_ref;
                  avd_obstacles[1] = avd_temp_cars[0];
                }
              }
            } else if (((avd_obstacles[0].min_l_to_ref > 0 &&
                         avd_obstacles[1].min_l_to_ref > 0 &&
                         avd_temp_cars[0].min_l_to_ref > 0 &&
                         avd_temp_cars[1].max_l_to_ref < 0) ||
                        (avd_obstacles[0].max_l_to_ref < 0 &&
                         avd_obstacles[1].max_l_to_ref < 0 &&
                         avd_temp_cars[0].max_l_to_ref < 0 &&
                         avd_temp_cars[1].min_l_to_ref > 0)) &&
                       std::fabs(avd_obstacles[1].s_to_ego -
                                 avd_obstacles[0].s_to_ego) < 3 * v_ego &&
                       std::fabs(avd_temp_cars[0].s_to_ego -
                                 avd_obstacles[1].s_to_ego) < 5 * v_ego &&
                       std::fabs(avd_temp_cars[1].s_to_ego -
                                 avd_temp_cars[1].s_to_ego) < 5 * v_ego) {
              if (avd_obstacles[0].min_l_to_ref > 0) {
                avd_obstacles[0].min_l_to_ref =
                    std::min(avd_obstacles[0].min_l_to_ref,
                             std::min(avd_obstacles[1].min_l_to_ref,
                                      avd_temp_cars[0].min_l_to_ref));
                avd_obstacles[0].max_l_to_ref =
                    std::min(avd_obstacles[0].max_l_to_ref,
                             std::min(avd_obstacles[1].max_l_to_ref,
                                      avd_temp_cars[0].max_l_to_ref));
                avd_obstacles[1] = avd_temp_cars[1];
              } else if (avd_obstacles[0].max_l_to_ref < 0) {
                avd_obstacles[0].min_l_to_ref =
                    std::max(avd_obstacles[0].min_l_to_ref,
                             std::max(avd_obstacles[1].min_l_to_ref,
                                      avd_temp_cars[0].min_l_to_ref));
                avd_obstacles[0].max_l_to_ref =
                    std::max(avd_obstacles[0].max_l_to_ref,
                             std::max(avd_obstacles[1].max_l_to_ref,
                                      avd_temp_cars[0].max_l_to_ref));
                avd_obstacles[1] = avd_temp_cars[1];
              }
            } else if (avd_obstacles[0].min_l_to_ref > 0 &&
                       avd_obstacles[1].max_l_to_ref < 0 &&
                       std::fabs(avd_temp_cars[0].s_to_ego -
                                 avd_obstacles[1].s_to_ego) < 5 * v_ego &&
                       std::fabs(avd_obstacles[1].s_to_ego -
                                 avd_obstacles[0].s_to_ego) < 3 * v_ego) {
              if (avd_temp_cars[0].max_l_to_ref < 0 &&
                  avd_temp_cars[0].max_l_to_ref >
                      avd_obstacles[1].max_l_to_ref) {
                avd_obstacles[1].min_l_to_ref = avd_temp_cars[0].min_l_to_ref;
                avd_obstacles[1].max_l_to_ref = avd_temp_cars[0].max_l_to_ref;
              } else if (avd_temp_cars[0].min_l_to_ref > 0 &&
                         avd_temp_cars[0].min_l_to_ref <
                             avd_obstacles[0].min_l_to_ref) {
                avd_obstacles[0].min_l_to_ref = avd_temp_cars[0].min_l_to_ref;
                avd_obstacles[0].max_l_to_ref = avd_temp_cars[0].max_l_to_ref;
              }
            } else if (avd_obstacles[0].max_l_to_ref < 0 &&
                       avd_obstacles[1].min_l_to_ref > 0 &&
                       std::fabs(avd_temp_cars[0].s_to_ego -
                                 avd_obstacles[1].s_to_ego) < 5 * v_ego &&
                       std::fabs(avd_obstacles[1].s_to_ego -
                                 avd_obstacles[0].s_to_ego) < 3 * v_ego) {
              if (avd_temp_cars[0].max_l_to_ref < 0 &&
                  avd_temp_cars[0].max_l_to_ref >
                      avd_obstacles[0].max_l_to_ref) {
                avd_obstacles[0].min_l_to_ref = avd_temp_cars[0].min_l_to_ref;
                avd_obstacles[0].max_l_to_ref = avd_temp_cars[0].max_l_to_ref;
              } else if (avd_temp_cars[0].min_l_to_ref > 0 &&
                         avd_temp_cars[0].min_l_to_ref <
                             avd_obstacles[1].min_l_to_ref) {
                avd_obstacles[1].min_l_to_ref = avd_temp_cars[0].min_l_to_ref;
                avd_obstacles[1].max_l_to_ref = avd_temp_cars[0].max_l_to_ref;
              }
            }
          } else if (enter2) {
            avd_obstacles = avd_temp_cars;
          } else if (enter0) {
            if (avd_obstacles[0].max_l_to_ref < 0 &&
                avd_obstacles[1].min_l_to_ref > 0 &&
                avd_temp_cars[1].min_l_to_ref > 0 &&
                avd_temp_cars[1].min_l_to_ref < avd_obstacles[1].min_l_to_ref &&
                std::fabs(avd_temp_cars[1].s_to_ego -
                          avd_obstacles[1].s_to_ego) < 5 * v_ego) {
              avd_obstacles[1].min_l_to_ref = avd_temp_cars[1].min_l_to_ref;
              avd_obstacles[1].max_l_to_ref = avd_temp_cars[1].max_l_to_ref;
            } else if (avd_obstacles[0].max_l_to_ref < 0 &&
                       avd_obstacles[1].max_l_to_ref < 0 &&
                       avd_temp_cars[1].min_l_to_ref > 0 &&
                       std::fabs(avd_obstacles[1].s_to_ego -
                                 avd_obstacles[0].s_to_ego) < 3 * v_ego &&
                       std::fabs(avd_temp_cars[1].s_to_ego -
                                 avd_obstacles[1].s_to_ego) < 5 * v_ego) {
              avd_obstacles[0].min_l_to_ref = std::max(
                  avd_obstacles[0].min_l_to_ref, avd_obstacles[1].min_l_to_ref);
              avd_obstacles[0].max_l_to_ref = std::max(
                  avd_obstacles[0].max_l_to_ref, avd_obstacles[1].max_l_to_ref);
              avd_obstacles[1] = avd_temp_cars[1];
            } else if (avd_obstacles[0].min_l_to_ref > 0 &&
                       avd_obstacles[1].min_l_to_ref > 0 &&
                       avd_temp_cars[1].max_l_to_ref < 0 &&
                       std::fabs(avd_obstacles[1].s_to_ego -
                                 avd_obstacles[0].s_to_ego) < 3 * v_ego &&
                       std::fabs(avd_temp_cars[1].s_to_ego -
                                 avd_obstacles[1].s_to_ego) < 5 * v_ego) {
              avd_obstacles[0].min_l_to_ref = std::min(
                  avd_obstacles[0].min_l_to_ref, avd_obstacles[1].min_l_to_ref);
              avd_obstacles[0].max_l_to_ref = std::min(
                  avd_obstacles[0].max_l_to_ref, avd_obstacles[1].max_l_to_ref);
              avd_obstacles[1] = avd_temp_cars[1];
            }
          } else if (enter1) {
            if (avd_obstacles[0].max_l_to_ref < 0 &&
                avd_obstacles[1].min_l_to_ref > 0 &&
                avd_temp_cars[1].min_l_to_ref > 0 &&
                avd_temp_cars[1].min_l_to_ref < avd_obstacles[1].min_l_to_ref &&
                std::fabs(avd_temp_cars[1].s_to_ego -
                          avd_obstacles[1].s_to_ego) < 5 * v_ego) {
              avd_obstacles[1].min_l_to_ref = avd_temp_cars[1].min_l_to_ref;
              avd_obstacles[1].max_l_to_ref = avd_temp_cars[1].max_l_to_ref;
            } else if (avd_obstacles[0].max_l_to_ref < 0 &&
                       avd_obstacles[1].max_l_to_ref < 0 &&
                       avd_temp_cars[1].min_l_to_ref > 0 &&
                       std::fabs(avd_obstacles[1].s_to_ego -
                                 avd_obstacles[0].s_to_ego) < 5 * v_ego &&
                       std::fabs(avd_temp_cars[1].s_to_ego -
                                 avd_obstacles[1].s_to_ego) < 5 * v_ego) {
              avd_obstacles[0].min_l_to_ref = std::max(
                  avd_obstacles[0].min_l_to_ref, avd_obstacles[1].min_l_to_ref);
              avd_obstacles[0].max_l_to_ref = std::max(
                  avd_obstacles[0].max_l_to_ref, avd_obstacles[1].max_l_to_ref);
              avd_obstacles[1] = avd_temp_cars[1];
            } else if (avd_obstacles[0].min_l_to_ref > 0 &&
                       avd_obstacles[1].min_l_to_ref > 0 &&
                       avd_temp_cars[1].max_l_to_ref < 0 &&
                       std::fabs(avd_obstacles[1].s_to_ego -
                                 avd_obstacles[0].s_to_ego) < 5 * v_ego &&
                       std::fabs(avd_temp_cars[1].s_to_ego -
                                 avd_obstacles[1].s_to_ego) < 5 * v_ego) {
              avd_obstacles[0].min_l_to_ref = std::min(
                  avd_obstacles[0].min_l_to_ref, avd_obstacles[1].min_l_to_ref);
              avd_obstacles[0].max_l_to_ref = std::min(
                  avd_obstacles[0].max_l_to_ref, avd_obstacles[1].max_l_to_ref);
              avd_obstacles[1] = avd_temp_cars[1];
            }
          }
        }
      }
    }

    if (avd_obstacles.size() == 0 && avd_temp_cars.size() > 0) {
      if (avd_temp_cars.size() == 1 || avd_temp_cars.size() == 2) {
        avd_obstacles = avd_temp_cars;
      } else if (avd_temp_cars.size() > 2) {
        avd_obstacles.clear();
        avd_obstacles.push_back(avd_temp_cars[0]);
        avd_obstacles.push_back(avd_temp_cars[1]);
      }
    }
  }
}

void AvoidObstacleMaintainer5V::SelectAvoidObstacle(
    const std::vector<AvoidObstacleInfo> &avd_obstacles) {
  std::vector<AvoidObstacleInfo> avd_obstacle_left;
  std::vector<AvoidObstacleInfo> avd_obstacle_right;
  std::unordered_map<uint, AvoidObstacleInfo> avd_obstacle_map;
  for (auto &avd_obstacle : avd_obstacles) {
    avd_obstacle_map.insert(
        std::make_pair(avd_obstacle.track_id, avd_obstacle));
  }

  if (avd_obstacles_[0].flag != AvoidObstacleFlag::INVALID) {
    avd_obstacle_map.insert(
        std::make_pair(avd_obstacles_[0].track_id, avd_obstacles_[0]));
    if (avd_obstacles_[1].flag != AvoidObstacleFlag::INVALID) {
      avd_obstacle_map.insert(
          std::make_pair(avd_obstacles_[1].track_id, avd_obstacles_[1]));
    }
  }

  if (avd_obstacle_map.size() == 4 || avd_obstacle_map.size() == 3) {
    for (auto &avd_obstacle : avd_obstacle_map) {
      if (fabs(avd_obstacle.second.max_l_to_ref) >
          fabs(avd_obstacle.second.min_l_to_ref)) {
        avd_obstacle_left.emplace_back(avd_obstacle.second);
      } else {
        avd_obstacle_right.emplace_back(avd_obstacle.second);
      }
    }

    std::sort(avd_obstacle_left.begin(), avd_obstacle_left.end(),
              [&](AvoidObstacleInfo &o1, AvoidObstacleInfo &o2) {
                return CampareDistance(o1, o2);
              });
    std::sort(avd_obstacle_right.begin(), avd_obstacle_right.end(),
              [&](const AvoidObstacleInfo &o1, const AvoidObstacleInfo &o2) {
                return CampareDistance(o1, o2);
              });

    if (avd_obstacle_map.size() == 4) {  // 4选2
      if (avd_obstacle_left.size() == 4) {
        avd_obstacles_[0] = avd_obstacle_left[0];
        avd_obstacles_[1] = avd_obstacle_left[1];
      } else if (avd_obstacle_left.size() == 3) {  // 丢弃最后一个，3选2
        if (avd_obstacle_left[0].s_to_ego < avd_obstacle_right[0].s_to_ego) {
          avd_obstacles_[0] = avd_obstacle_left[0];
          if (avd_obstacle_right[0].s_to_ego - avd_obstacle_left[1].s_to_ego >
              15) {  //修改
            avd_obstacles_[1] = avd_obstacle_left[1];
          } else {
            avd_obstacles_[1] = avd_obstacle_right[0];
          }
        } else {
          avd_obstacles_[0] = avd_obstacle_right[0];
          avd_obstacles_[1] = avd_obstacle_left[0];
        }
      } else if (avd_obstacle_left.size() ==
                 2) {  // 选择最近的一个之后，再丢弃最远的一个，
                       // 最后2选1
        if (avd_obstacle_left[0].s_to_ego < avd_obstacle_right[0].s_to_ego) {
          avd_obstacles_[0] = avd_obstacle_left[0];
          if (avd_obstacle_right[0].s_to_ego < 5) {
            avd_obstacles_[1] = avd_obstacle_right[0];
          } else {
            if (avd_obstacle_left[1].s_to_ego <
                avd_obstacle_right[1]
                    .s_to_ego) {  // ignore avd_obstacle_right[1]
              if (avd_obstacle_right[0].s_to_ego -
                      avd_obstacle_left[1].s_to_ego >
                  15) {  //修改
                avd_obstacles_[1] = avd_obstacle_left[1];
              } else {
                avd_obstacles_[1] = avd_obstacle_right[0];
              }
            } else {  // ignore avd_obstacle_left[1]
              avd_obstacles_[1] = avd_obstacle_right[0];
            }
          }
        } else {
          avd_obstacles_[0] = avd_obstacle_right[0];
          if (avd_obstacle_left[0].s_to_ego < 5) {
            avd_obstacles_[1] = avd_obstacle_left[0];
          } else {
            if (avd_obstacle_right[1].s_to_ego <
                avd_obstacle_left[1].s_to_ego) {  // ignore avd_obstacle_left[1]
              if (avd_obstacle_left[0].s_to_ego -
                      avd_obstacle_right[1].s_to_ego >
                  15) {  //修改
                avd_obstacles_[1] = avd_obstacle_right[1];
              } else {
                avd_obstacles_[1] = avd_obstacle_left[0];
              }
            } else {  // ignore avd_obstacle_right[1]
              avd_obstacles_[1] = avd_obstacle_left[0];
            }
          }
        }
      } else if (avd_obstacle_left.size() == 1) {  // 丢弃最后一个，3选2
        if (avd_obstacle_right[0].s_to_ego < avd_obstacle_left[0].s_to_ego) {
          avd_obstacles_[0] = avd_obstacle_right[0];
          if (avd_obstacle_left[0].s_to_ego - avd_obstacle_right[0].s_to_ego >
              15) {
            avd_obstacles_[1] = avd_obstacle_right[1];
          } else {
            avd_obstacles_[1] = avd_obstacle_left[0];
          }
        } else {
          avd_obstacles_[0] = avd_obstacle_left[0];
          avd_obstacles_[1] = avd_obstacle_right[0];
        }
      } else if (avd_obstacle_left.size() == 0) {
        avd_obstacles_[0] = avd_obstacle_right[0];
        avd_obstacles_[1] = avd_obstacle_right[1];
      }
    } else if (avd_obstacle_map.size() == 3) {  // 3选2
      if (avd_obstacle_left.size() == 3) {
        avd_obstacles_[0] = avd_obstacle_left[0];
        avd_obstacles_[1] = avd_obstacle_left[1];
      } else if (avd_obstacle_left.size() == 2) {
        if (avd_obstacle_left[0].s_to_ego < avd_obstacle_right[0].s_to_ego) {
          avd_obstacles_[0] = avd_obstacle_left[0];
          if (avd_obstacle_right[0].s_to_ego - avd_obstacle_left[0].s_to_ego >
              15) {
            avd_obstacles_[1] = avd_obstacle_left[1];
          } else {
            avd_obstacles_[1] = avd_obstacle_right[0];
          }
        } else {
          avd_obstacles_[0] = avd_obstacle_right[0];
          avd_obstacles_[1] = avd_obstacle_left[0];
        }
      } else if (avd_obstacle_left.size() == 1) {
        if (avd_obstacle_right[0].s_to_ego < avd_obstacle_left[0].s_to_ego) {
          avd_obstacles_[0] = avd_obstacle_right[0];
          if (avd_obstacle_left[0].s_to_ego - avd_obstacle_right[0].s_to_ego >
              15) {
            avd_obstacles_[1] = avd_obstacle_right[1];
          } else {
            avd_obstacles_[1] = avd_obstacle_left[0];
          }
        } else {
          avd_obstacles_[0] = avd_obstacle_left[0];
          avd_obstacles_[1] = avd_obstacle_right[0];
        }
      } else {
        avd_obstacles_[0] = avd_obstacle_right[0];
        avd_obstacles_[1] = avd_obstacle_right[1];
      }
    }
  } else if (avd_obstacle_map.size() == 2) {
    auto it = avd_obstacle_map.begin();
    avd_obstacles_[0] = it->second;
    ++it;
    avd_obstacles_[1] = it->second;
  } else if (avd_obstacle_map.size() == 1) {
    avd_obstacles_[0] = avd_obstacle_map.begin()->second;
  }

  if (avd_obstacles_[0].flag != AvoidObstacleFlag::INVALID &&
      avd_obstacles_[1].flag != AvoidObstacleFlag::INVALID) {
    if (not CampareDistance(avd_obstacles_[0], avd_obstacles_[1])) {
      std::swap(avd_obstacles_[0], avd_obstacles_[1]);
    }
  }
}

void AvoidObstacleMaintainer5V::UpdateAvoidObstacleInfo1(
    std::vector<AvoidObstacleInfo> &avd_obstacles) {
  // update avd_obstacles_ info according to avd_obstacles
  // TODO(clren): 为了避让更平滑， 对avd_obstacles_的横向信息更新做调整
  auto TempHack = [](const planning::framework::Session *session,
                     const AvoidObstacleInfo &avd_obstacle_past,
                     AvoidObstacleInfo &avd_obstacles) {
    const CoarsePlanningInfo &coarse_planning_info =
        session->planning_context()
            .lane_change_decider_output()
            .coarse_planning_info;
    const auto fix_ref =
        session->environmental_model()
            .get_reference_path_manager()
            ->get_reference_path_by_lane(coarse_planning_info.target_lane_id);
    const auto &obstacles_map = fix_ref->get_obstacles_map();
    const auto &lateral_obstacle_history_info =
        session->planning_context()
            .lateral_obstacle_decider_output()
            .lateral_obstacle_history_info;
    if (obstacles_map.find(avd_obstacles.track_id) != obstacles_map.end()) {
      const auto frenet_obstacle = obstacles_map.at(avd_obstacles.track_id);
      double obs_relative_heading =
          frenet_obstacle->obstacle()->heading_angle() -
          fix_ref->get_frenet_coord()->GetPathCurveHeading(
              frenet_obstacle->frenet_s());
      // heading is out
      if (obs_relative_heading * frenet_obstacle->frenet_l() > 0 &&
          fabs(obs_relative_heading) > 0.05) {
        avd_obstacles.min_l_to_ref = avd_obstacle_past.min_l_to_ref;
        avd_obstacles.max_l_to_ref = avd_obstacle_past.max_l_to_ref;
      }

      // 后续联合考虑纵向距离与v_l
      if (fabs(frenet_obstacle->frenet_velocity_l()) >= 0.3) {
        avd_obstacles.min_l_to_ref = avd_obstacle_past.min_l_to_ref;
        avd_obstacles.max_l_to_ref = avd_obstacle_past.max_l_to_ref;
      }

      // hold lat_offset，不更新横向位置
      bool maintain_avoid = false;
      auto lateral_obstacle_iter =
          lateral_obstacle_history_info.find(avd_obstacles.track_id);
      if (lateral_obstacle_iter != lateral_obstacle_history_info.end()) {
        maintain_avoid = lateral_obstacle_iter->second.maintain_avoid;
      }
      if (maintain_avoid) {
        avd_obstacles.min_l_to_ref = avd_obstacle_past.min_l_to_ref;
        avd_obstacles.max_l_to_ref = avd_obstacle_past.max_l_to_ref;
      }
    }
  };

  if (avd_obstacles.size() == 2) {
    if (avd_obstacles_[0].flag != AvoidObstacleFlag::INVALID) {
      if (avd_obstacles_[0].track_id == avd_obstacles[0].track_id) {
        avd_obstacles[0].first_s_to_ego = std::max(
            avd_obstacles[0].first_s_to_ego, avd_obstacles_[0].first_s_to_ego);
        avd_obstacles[0].is_passive = avd_obstacles_[0].is_passive;
        TempHack(session_, avd_obstacles_[0], avd_obstacles[0]);
        avd_obstacles_[0] = avd_obstacles[0];
      } else if (avd_obstacles_[0].track_id == avd_obstacles[1].track_id) {
        avd_obstacles[1].first_s_to_ego = std::max(
            avd_obstacles[1].first_s_to_ego, avd_obstacles_[0].first_s_to_ego);
        avd_obstacles[1].is_passive = avd_obstacles_[0].is_passive;
        TempHack(session_, avd_obstacles_[0], avd_obstacles[1]);
        avd_obstacles_[0] = avd_obstacles[1];
      }
      if (avd_obstacles_[1].flag != AvoidObstacleFlag::INVALID) {
        if (avd_obstacles_[1].track_id == avd_obstacles[0].track_id) {
          avd_obstacles[0].first_s_to_ego =
              std::max(avd_obstacles[0].first_s_to_ego,
                       avd_obstacles_[1].first_s_to_ego);
          avd_obstacles[0].is_passive = avd_obstacles_[1].is_passive;
          TempHack(session_, avd_obstacles_[1], avd_obstacles[0]);
          avd_obstacles_[1] = avd_obstacles[0];
        } else if (avd_obstacles_[1].track_id == avd_obstacles[1].track_id) {
          avd_obstacles[1].first_s_to_ego =
              std::max(avd_obstacles[1].first_s_to_ego,
                       avd_obstacles_[1].first_s_to_ego);
          avd_obstacles[1].is_passive = avd_obstacles_[1].is_passive;
          TempHack(session_, avd_obstacles_[1], avd_obstacles[1]);
          avd_obstacles_[1] = avd_obstacles[1];
        }
      }
    }
  } else if (avd_obstacles.size() == 1) {
    if (avd_obstacles_[0].flag != AvoidObstacleFlag::INVALID) {
      if (avd_obstacles[0].track_id == avd_obstacles_[0].track_id) {
        avd_obstacles[0].first_s_to_ego = std::max(
            avd_obstacles[0].first_s_to_ego, avd_obstacles_[0].first_s_to_ego);
        avd_obstacles[0].is_passive = avd_obstacles_[0].is_passive;
        TempHack(session_, avd_obstacles_[0], avd_obstacles[0]);
        avd_obstacles_[0] = avd_obstacles[0];
      } else {
        if (avd_obstacles_[1].flag != AvoidObstacleFlag::INVALID) {
          if (avd_obstacles[0].track_id == avd_obstacles_[1].track_id) {
            avd_obstacles[0].first_s_to_ego =
                std::max(avd_obstacles[0].first_s_to_ego,
                         avd_obstacles_[1].first_s_to_ego);
            avd_obstacles[0].is_passive = avd_obstacles_[1].is_passive;
            TempHack(session_, avd_obstacles_[1], avd_obstacles[0]);
            avd_obstacles_[1] = avd_obstacles[0];
          }
        }
      }
    }
  }
}

void AvoidObstacleMaintainer5V::UpdateAvoidObstacleInfo2(
    const std::shared_ptr<LateralObstacle> lateral_obstacle,
    double t_interval) {
  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  auto ego_vs = reference_path_ptr->get_frenet_ego_state().velocity_s();
  double update_predict_vs_lon_relative_1, update_predict_vs_lon_relative_2;

  // update avd_obstacles_[0]
  if (avd_obstacles_[0].flag != AvoidObstacleFlag::INVALID &&
      avd_obstacles_[0].update_flag == AvoidObstacleUpdateFlag::Past) {
    TrackedObject tr;
    bool is_found =
        lateral_obstacle->find_track(avd_obstacles_[0].track_id, tr);
    if (is_found &&
        (tr.fusion_source & OBSTACLE_SOURCE_CAMERA)) {  // TODO filter
      // update avd_obstacle accord to real data
      avd_obstacles_[0].tail_s_to_ego = tr.tail_rel_s;
      avd_obstacles_[0].s_to_ego = tr.d_rel;
      avd_obstacles_[0].vs_lon_relative = tr.v_rel;
      avd_obstacles_[0].predict_vs_lon_relative = tr.v_rel;
      avd_obstacles_[0].vs = tr.v_lead;
    } else {
      // update avd_obstacle accord to prediction
      update_predict_vs_lon_relative_1 =
          avd_obstacles_[0].predict_vs_lon_relative;
      // consider change in the ego's speed
      update_predict_vs_lon_relative_1 +=
          (avd_obstacles_[0].vs - avd_obstacles_[0].predict_vs_lon_relative -
           ego_vs);

      double pred_distance = t_interval *
                             (update_predict_vs_lon_relative_1 +
                              avd_obstacles_[0].predict_vs_lon_relative) *
                             0.5;
      avd_obstacles_[0].predict_vs_lon_relative =
          update_predict_vs_lon_relative_1;
      avd_obstacles_[0].tail_s_to_ego += pred_distance;

      if (avd_obstacles_[0].tail_s_to_ego > 0) {
        avd_obstacles_[0].s_to_ego = avd_obstacles_[0].tail_s_to_ego;
      } else {
        double head_s_to_ego =
            avd_obstacles_[0].tail_s_to_ego + avd_obstacles_[0].length;
        if (head_s_to_ego > 0) {
          avd_obstacles_[0].s_to_ego = 0;
        } else {
          avd_obstacles_[0].s_to_ego = head_s_to_ego;
        }
      }

      if (ego_vs < 8) {
        avd_obstacles_[0].vs_lon_relative = -ego_vs;
      }
    }

    double diff_time = curr_time - avd_obstacles_[0].curr_time;
    if (avd_obstacles_[0].tail_s_to_ego >= 5.0 &&
        (diff_time > keep_time_level_2 ||
         (diff_time > keep_time_level_1 &&
          avd_obstacles_[0].vs_lon_relative > 1.5))) {  // hack
      avd_obstacles_[0].Reset();
    }
  }

  // update avd_obstacles_[1]
  if (avd_obstacles_[1].flag != AvoidObstacleFlag::INVALID &&
      avd_obstacles_[1].update_flag == AvoidObstacleUpdateFlag::Past) {
    TrackedObject tr;
    bool is_found =
        lateral_obstacle->find_track(avd_obstacles_[1].track_id, tr);
    if (is_found &&
        (tr.fusion_source & OBSTACLE_SOURCE_CAMERA)) {  // TODO filter
      avd_obstacles_[1].tail_s_to_ego = tr.tail_rel_s;
      avd_obstacles_[1].s_to_ego = tr.d_rel;
      avd_obstacles_[1].vs_lon_relative = tr.v_rel;
      avd_obstacles_[1].predict_vs_lon_relative = tr.v_rel;
      avd_obstacles_[1].vs = tr.v_lead;
    } else {
      update_predict_vs_lon_relative_2 =
          avd_obstacles_[1].predict_vs_lon_relative;
      // consider change in the ego's speed
      update_predict_vs_lon_relative_2 +=
          (avd_obstacles_[1].vs - avd_obstacles_[1].predict_vs_lon_relative -
           ego_vs);

      double pred_distance = t_interval *
                             (update_predict_vs_lon_relative_2 +
                              avd_obstacles_[1].predict_vs_lon_relative) *
                             0.5;
      avd_obstacles_[1].predict_vs_lon_relative =
          update_predict_vs_lon_relative_2;
      avd_obstacles_[1].tail_s_to_ego += pred_distance;

      if (avd_obstacles_[1].tail_s_to_ego > 0) {
        avd_obstacles_[1].s_to_ego = avd_obstacles_[1].tail_s_to_ego;
      } else {
        double head_s_to_ego =
            avd_obstacles_[1].tail_s_to_ego + avd_obstacles_[1].length;
        if (head_s_to_ego > 0) {
          avd_obstacles_[1].s_to_ego = 0;
        } else {
          avd_obstacles_[1].s_to_ego = head_s_to_ego;
        }
      }

      if (ego_vs < 8) {
        avd_obstacles_[1].vs_lon_relative = -ego_vs;
      }
    }

    double diff_time = curr_time - avd_obstacles_[1].curr_time;
    if (avd_obstacles_[1].tail_s_to_ego >= 5.0 &&
        (diff_time > keep_time_level_2 ||
         (diff_time > keep_time_level_1 &&
          avd_obstacles_[1].vs_lon_relative > 1.5))) {  // hack
      avd_obstacles_[1].Reset();
    }
  }

  if (avd_obstacles_[0].flag == AvoidObstacleFlag::INVALID) {
    if (avd_obstacles_[1].flag != AvoidObstacleFlag::INVALID) {
      avd_obstacles_[0] = avd_obstacles_[1];
      avd_obstacles_[1].Reset();
    }
  }

  if (avd_obstacles_[0].flag != AvoidObstacleFlag::INVALID &&
      avd_obstacles_[1].flag != AvoidObstacleFlag::INVALID) {
    if (not CampareDistance(avd_obstacles_[0], avd_obstacles_[1])) {
      std::swap(avd_obstacles_[0], avd_obstacles_[1]);
    }
  }
}

void AvoidObstacleMaintainer5V::CheckAvoidObstacle(
    const std::shared_ptr<LateralObstacle> lateral_obstacle) {
  if (avd_obstacles_[0].flag != AvoidObstacleFlag::INVALID &&
      avd_obstacles_[0].update_flag == AvoidObstacleUpdateFlag::Past) {
    if (avd_obstacles_[0].s_to_ego + ego_length_ < -safety_dist_level_1 ||
        curr_time - avd_obstacles_[0].curr_time >
            avd_obstacles_[0].t_exceed_avd_obstacle ||
        (IsOutAvoidArea(lateral_obstacle, avd_obstacles_[0]) &&
         avd_obstacles_[0].num_out_avd_area >= 2)) {
      bool no_near_car = true;
      if (avd_obstacles_[0].min_l_to_ref > 0) {
        no_near_car = UpdateLSideAvdsInfo(no_near_car, avd_obstacles_[0],
                                          avd_obstacles_[1]);
        no_near_car = UpdateLFrontAvdsInfo(no_near_car, avd_obstacles_[0],
                                           avd_obstacles_[1]);
      } else {
        no_near_car = UpdateRSideAvdsInfo(no_near_car, avd_obstacles_[0],
                                          avd_obstacles_[1]);
        no_near_car = UpdateRFrontAvdsInfo(no_near_car, avd_obstacles_[0],
                                           avd_obstacles_[1]);
      }
      if (no_near_car) {
        avd_obstacles_[0].Reset();
      }
    }
  }

  if (avd_obstacles_[1].flag != AvoidObstacleFlag::INVALID &&
      avd_obstacles_[1].update_flag == AvoidObstacleUpdateFlag::Past) {
    if (avd_obstacles_[1].s_to_ego + ego_length_ < -safety_dist_level_1 ||
        curr_time - avd_obstacles_[1].curr_time >
            avd_obstacles_[1].t_exceed_avd_obstacle ||
        (IsOutAvoidArea(lateral_obstacle, avd_obstacles_[1]) &&
         avd_obstacles_[1].num_out_avd_area >= 2)) {  // hack: -3
      bool no_near_car = true;
      if (avd_obstacles_[1].min_l_to_ref > 0) {
        // TODO:clren 可优化这部分逻辑
        no_near_car = UpdateLSideAvdsInfo(no_near_car, avd_obstacles_[1],
                                          avd_obstacles_[0]);
        no_near_car = UpdateLFrontAvdsInfo(no_near_car, avd_obstacles_[1],
                                           avd_obstacles_[0]);
      } else {
        no_near_car = UpdateRSideAvdsInfo(no_near_car, avd_obstacles_[1],
                                          avd_obstacles_[0]);
        no_near_car = UpdateRFrontAvdsInfo(no_near_car, avd_obstacles_[1],
                                           avd_obstacles_[0]);
      }
      if (no_near_car) {
        avd_obstacles_[1].Reset();
      }
    }
  }

  if (avd_obstacles_[0].flag != AvoidObstacleFlag::INVALID) {
    if (avd_obstacles_[1].flag != AvoidObstacleFlag::INVALID) {
      if (not CampareDistance(avd_obstacles_[0], avd_obstacles_[1])) {
        std::swap(avd_obstacles_[0], avd_obstacles_[1]);
      }
    }
  } else if (avd_obstacles_[1].flag != AvoidObstacleFlag::INVALID) {
    avd_obstacles_[0] = avd_obstacles_[1];
    avd_obstacles_[1].Reset();
  }
}

void AvoidObstacleMaintainer5V::UpdateAvoidObstacleInfo3() {
  // calculate time required to exceed avd_obstacle
  if (avd_obstacles_[0].flag != AvoidObstacleFlag::INVALID) {
    double d_exceed_avd_obstacle;
    if (equal_zero(avd_obstacles_[0].vs_lon_relative) == false &&
        avd_obstacles_[0].update_flag == AvoidObstacleUpdateFlag::Update) {
      d_exceed_avd_obstacle = avd_obstacles_[0].tail_s_to_ego + ego_length_ +
                              avd_obstacles_[0].length + safety_dist_level_2;
      avd_obstacles_[0].t_exceed_avd_obstacle =
          d_exceed_avd_obstacle / (-avd_obstacles_[0].vs_lon_relative);
      if (avd_obstacles_[0].t_exceed_avd_obstacle < 0) {
        avd_obstacles_[0].t_exceed_avd_obstacle = MAX_T_EXCEED_AVD_CAR;
      } else {
        avd_obstacles_[0].t_exceed_avd_obstacle =
            std::fmax(MIN_T_EXCEED_AVD_CAR,
                      std::fmin(MAX_T_EXCEED_AVD_CAR,
                                avd_obstacles_[0].t_exceed_avd_obstacle));
      }
    }

    if (avd_obstacles_[1].flag != AvoidObstacleFlag::INVALID) {
      if (equal_zero(avd_obstacles_[1].vs_lon_relative) == false &&
          avd_obstacles_[1].update_flag == AvoidObstacleUpdateFlag::Update) {
        d_exceed_avd_obstacle = avd_obstacles_[1].tail_s_to_ego + ego_length_ +
                                avd_obstacles_[1].length + safety_dist_level_2;
        avd_obstacles_[1].t_exceed_avd_obstacle =
            d_exceed_avd_obstacle / (-avd_obstacles_[1].vs_lon_relative);
        if (avd_obstacles_[1].t_exceed_avd_obstacle < 0) {
          avd_obstacles_[1].t_exceed_avd_obstacle = MAX_T_EXCEED_AVD_CAR;
        } else {
          avd_obstacles_[1].t_exceed_avd_obstacle =
              std::fmax(MIN_T_EXCEED_AVD_CAR,
                        std::fmin(MAX_T_EXCEED_AVD_CAR,
                                  avd_obstacles_[1].t_exceed_avd_obstacle));
        }
      }
    }
  }

  // calculate if avd_obstacle is passive
  // TODO(clren): cancel passive flag
  if (avd_obstacles_[0].flag != AvoidObstacleFlag::INVALID) {
    if ((avd_obstacles_history_[0].flag == AvoidObstacleFlag::INVALID ||
         avd_obstacles_history_[0].track_id != avd_obstacles_[0].track_id) &&
        (avd_obstacles_history_[1].flag == AvoidObstacleFlag::INVALID ||
         avd_obstacles_history_[1].track_id != avd_obstacles_[0].track_id)) {
      avd_obstacles_[0].is_passive =
          lateral_offset_decider::IsPassive(avd_obstacles_[0]);
    }

    if (avd_obstacles_[1].flag != AvoidObstacleFlag::INVALID) {
      if ((avd_obstacles_history_[0].flag == AvoidObstacleFlag::INVALID &&
           avd_obstacles_history_[0].track_id != avd_obstacles_[1].track_id) &&
          (avd_obstacles_history_[1].flag == AvoidObstacleFlag::INVALID &&
           avd_obstacles_history_[1].track_id != avd_obstacles_[1].track_id)) {
        avd_obstacles_[1].is_passive =
            lateral_offset_decider::IsPassive(avd_obstacles_[1]);
      }
    }
  }
}

// avd_obstacle1 < avd_obstacle2 : return true
bool AvoidObstacleMaintainer5V::CampareDistance(
    const AvoidObstacleInfo &avd_obstacle1,
    const AvoidObstacleInfo &avd_obstacle2) {
  if (avd_obstacle1.tail_s_to_ego > 0) {
    if (avd_obstacle2.tail_s_to_ego > 0) {
      return avd_obstacle1.tail_s_to_ego < avd_obstacle2.tail_s_to_ego;
    } else {
      if (avd_obstacle2.tail_s_to_ego + avd_obstacle2.length <
          -ego_length_) {  // has no overlap
        return fabs(avd_obstacle1.tail_s_to_ego) <
               fabs(avd_obstacle2.tail_s_to_ego + avd_obstacle2.length +
                    ego_length_);
      } else {  // has overlap
        return false;
      }
    }
  } else {
    if (avd_obstacle2.tail_s_to_ego > 0) {
      if (avd_obstacle1.tail_s_to_ego + avd_obstacle1.length <
          -ego_length_) {  // has no overlap
        return fabs(avd_obstacle1.tail_s_to_ego + avd_obstacle1.length +
                    ego_length_) < fabs(avd_obstacle2.tail_s_to_ego);
      } else {  // has overlap
        return true;
      }
    } else {  // compare head distance
      if (avd_obstacle1.tail_s_to_ego + avd_obstacle1.length > 0 &&
          avd_obstacle2.tail_s_to_ego + avd_obstacle2.length > 0) {
        return avd_obstacle1.tail_s_to_ego + avd_obstacle1.length <
               avd_obstacle2.tail_s_to_ego + avd_obstacle2.length;
      } else {
        return avd_obstacle1.tail_s_to_ego + avd_obstacle1.length >
               avd_obstacle2.tail_s_to_ego +
                   avd_obstacle2.length;  // it is different
      }
    }
  }
}

bool AvoidObstacleMaintainer5V::Process(planning::framework::Session *session) {
  session_ = session;
  avd_obstacles_cur_.clear();
  is_ncar_ = false;

  auto &lateral_obstacle =
      session_->mutable_environmental_model()->get_lateral_obstacle();
  auto &ego_state =
      session_->mutable_environmental_model()->get_ego_state_manager();
  const CoarsePlanningInfo &coarse_planning_info =
      session_->planning_context()
          .lane_change_decider_output()
          .coarse_planning_info;
  const auto flane =
      session_->environmental_model()
          .get_virtual_lane_manager()
          ->get_lane_with_virtual_id(coarse_planning_info.target_lane_id);
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  ego_length_ = vehicle_param.length;
  double v_ego = ego_state->ego_v();
  lane_width_ = flane->width();

#ifdef X86
  double t_interval = 1.0 / FLAGS_planning_loop_rate;
#else
  double t_interval = IflyTime::Now_s() - curr_time;
#endif
  curr_time += t_interval;

  avd_obstacles_history_ = avd_obstacles_;
  if (is_ncar_ == true &&
      avd_obstacles_[0].flag != AvoidObstacleFlag::INVALID) {
  } else {
    // step1: reset avd_obstacles_ update_flag
    avd_obstacles_[0].update_flag = AvoidObstacleUpdateFlag::Past;
    avd_obstacles_[1].update_flag = AvoidObstacleUpdateFlag::Past;

    // step2: select cur_avoid_obstacles at the current time
    SelectCurAvoidObstacles(lateral_obstacle, v_ego, avd_obstacles_cur_);

    // step3:
    // check avd_obstacles_ is deleted according to front_tracks' info
    // update avd_obstacles_ info according to front_tracks
    UpdateAvoidObstacle(lateral_obstacle);

    // step4:update avd_obstacles_ info according to avd_obstacles
    UpdateAvoidObstacleInfo1(avd_obstacles_cur_);

    // step5:
    // delete it if avd_obstacle_past disappear due to not entering the blind
    // area (Id changed or other)
    UpdateAvoidObstacleInfo2(lateral_obstacle, t_interval);

    // step6: if avd_obstacles_ is not updated , check avd_obstacles_
    CheckAvoidObstacle(lateral_obstacle);

    // step7, select 2 avd_obstacles_ from avd_obstacles and
    // avd_obstacles_;
    SelectAvoidObstacle(avd_obstacles_cur_);

    // step8, calc t_exceed_avd_obstacle
    UpdateAvoidObstacleInfo3();

    if (avd_obstacles_[0].flag == AvoidObstacleFlag::INVALID &&
        avd_obstacles_history_[0].flag != AvoidObstacleFlag::INVALID) {
      flag_avd_ = true;
      avd_back_start_time_ = curr_time;
    }

    if (flag_avd_) {
      if (curr_time - avd_back_start_time_ > 3) {
        flag_avd_ = false;
      }
    }
  }

  SaveDebugInfo();
  return true;
}

void AvoidObstacleMaintainer5V::SaveDebugInfo() {
  auto &debug_info_manager = DebugInfoManager::GetInstance();
  auto &planning_debug_data = debug_info_manager.GetDebugInfoPb();
  auto lateral_offset_decider_info =
      planning_debug_data->mutable_lateral_offset_decider_info();

  lateral_offset_decider_info->mutable_avoid_car_ids()->Clear();
  for (auto &item : avd_obstacles_cur_) {
    if (item.flag != AvoidObstacleFlag::INVALID) {
      lateral_offset_decider_info->add_avoid_car_ids(item.track_id);
    }
  }

  if (avd_obstacles_cur_.size() == 0) {
    JSON_DEBUG_VALUE("avoid_car_ids_1", -1000);
    JSON_DEBUG_VALUE("avoid_car_ids_2", -1000);
  } else if (avd_obstacles_cur_.size() == 1) {
    JSON_DEBUG_VALUE("avoid_car_ids_1", avd_obstacles_cur_[0].track_id);
    JSON_DEBUG_VALUE("avoid_car_ids_2", -1000);
  } else {
    JSON_DEBUG_VALUE("avoid_car_ids_1", avd_obstacles_cur_[0].track_id);
    JSON_DEBUG_VALUE("avoid_car_ids_2", avd_obstacles_cur_[1].track_id);
  }

  lateral_offset_decider_info->mutable_select_avoid_car_ids()->Clear();
  for (auto &item : avd_obstacles_) {
    lateral_offset_decider_info->add_select_avoid_car_ids(item.track_id);
  }

  JSON_DEBUG_VALUE("select_avoid_car_ids_1", avd_obstacles_[0].track_id);
  JSON_DEBUG_VALUE("select_avoid_car_ids_2", avd_obstacles_[1].track_id);

  for (int i = 0; i < 2; i++) {
    ILOG_DEBUG << "avd_obstacle id:" << avd_obstacles_[i].track_id;
    std::string prefix = "VOLat_avd_obstacles_" + std::to_string(i);
    JSON_DEBUG_VALUE(prefix + "_id", avd_obstacles_[i].track_id);
    JSON_DEBUG_VALUE(prefix + "_allow_max_opposite_offset",
                     avd_obstacles_[i].allow_max_opposite_offset);
    JSON_DEBUG_VALUE(prefix + "_vs_lon_relative",
                     avd_obstacles_[i].vs_lon_relative);
    JSON_DEBUG_VALUE(prefix + "_vs", avd_obstacles_[i].vs);
    JSON_DEBUG_VALUE(prefix + "_s_to_ego", avd_obstacles_[i].s_to_ego);
    JSON_DEBUG_VALUE(prefix + "_vs_lat_relative",
                     avd_obstacles_[i].vs_lat_relative);
    JSON_DEBUG_VALUE(prefix + "_min_l_to_ref", avd_obstacles_[i].min_l_to_ref);
    JSON_DEBUG_VALUE(prefix + "_max_l_to_ref", avd_obstacles_[i].max_l_to_ref);
    JSON_DEBUG_VALUE(prefix + "_first_s_to_ego",
                     avd_obstacles_[i].first_s_to_ego);
    JSON_DEBUG_VALUE(prefix + "_type", avd_obstacles_[i].type);
    JSON_DEBUG_VALUE(prefix + "_curr_time", avd_obstacles_[i].curr_time);
    JSON_DEBUG_VALUE(prefix + "_update_flag", avd_obstacles_[i].update_flag);
    JSON_DEBUG_VALUE(prefix + "_length", avd_obstacles_[i].length);
  }
}

}  // namespace planning
