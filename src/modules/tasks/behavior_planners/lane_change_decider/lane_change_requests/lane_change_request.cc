#include "lane_change_request.h"
#include <cmath>
#include "common_c.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "lateral_obstacle.h"
#include "obstacle_manager.h"
#include "planning_context.h"
#include "utils/lateral_utils.h"

namespace planning {
LaneChangeRequest::LaneChangeRequest(
    framework::Session *session,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : session_(session),
      virtual_lane_mgr_(virtual_lane_mgr),
      lane_change_lane_mgr_(lane_change_lane_mgr) {}

void LaneChangeRequest::GenerateRequest(RequestType direction) {
  if (direction != LEFT_CHANGE && direction != RIGHT_CHANGE) {
    LOG_DEBUG("[LaneChangeRequest::GenerateRequest] Illgeal direction[%d] \n",
              direction);
  }
  if (request_type_ == direction) {
    LOG_DEBUG(
        "[LaneChangeRequest::GenerateRequest] duplicated request, "
        "direction[%d] \n",
        direction);
    return;
  }
  request_type_ = direction;
  turn_signal_ = direction;
  tstart_ = IflyTime::Now_s();
}

void LaneChangeRequest::Finish() {
  if (request_type_ == NO_CHANGE) {
    LOG_DEBUG("[LaneChangeRequest::Finish] No request to finish \n");
    turn_signal_ = NO_CHANGE;
    return;
  }

  request_type_ = NO_CHANGE;
  turn_signal_ = NO_CHANGE;
  tfinish_ = IflyTime::Now_s();
}

bool LaneChangeRequest::AggressiveChange() const {
  auto origin_lane = lane_change_lane_mgr_->has_origin_lane()
                         ? lane_change_lane_mgr_->olane()
                         : virtual_lane_mgr_->get_current_lane();
  auto lc_map_decision =
      0;  // hack
          // origin_lane != nullptr ? origin_lane->lc_map_decision() : 0;
  auto aggressive_change_distance = 200.0;  // WB: hack
  // virtual_lane_mgr_->is_on_highway()
  //     ? aggressive_lane_change_distance_highway_
  //     : aggressive_lane_change_distance_urban_;

  // auto aggressive_change =
  //     origin_lane != nullptr
  //         ? origin_lane->must_change_lane(aggressive_change_distance *
  //                                         std::fabs(lc_map_decision))
  //         : false;
  auto aggressive_change =
      origin_lane != nullptr ? virtual_lane_mgr_->must_change_lane(
                                   origin_lane, aggressive_change_distance *
                                                    std::fabs(lc_map_decision))
                             : false;
  return aggressive_change && (request_type_ != NO_CHANGE);
}

bool LaneChangeRequest::IsDashedLineEnough(
    RequestType direction, const double ego_vel,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr) {
  LOG_DEBUG("dashed_enough: direction: %d \n", static_cast<int>(direction));
  LOG_DEBUG("dashed_enough: vel: %.2f \n", ego_vel);
  const double kInputBoundaryLenLimit = 145.;
  const double kDefaultBoundaryLen = 5000.;
  double dash_length = 80;
  const double kNeedLaneChangeTime = 4.0;
  double right_dash_line_len = virtual_lane_mgr->get_distance_to_dash_line(
      RIGHT_CHANGE, origin_lane_virtual_id_);
  double left_dash_line_len = virtual_lane_mgr->get_distance_to_dash_line(
      LEFT_CHANGE, origin_lane_virtual_id_);
  LOG_DEBUG("dashed_enough: right_dash_line_len: %.2f \n", right_dash_line_len);
  LOG_DEBUG("dashed_enough: left_dash_line_len: %.2f \n", left_dash_line_len);
  std::cout << "origin_lane_virtual_id_: " << origin_lane_virtual_id_
            << "origin_lane_order_id_: " << origin_lane_virtual_id_
            << std::endl;
  // HACK RUI
  if (virtual_lane_mgr->dis_to_ramp() < 500.) return true;
  if (direction == LEFT_CHANGE && left_dash_line_len > 0.) {
    if (left_dash_line_len > ego_vel * kNeedLaneChangeTime) {
      return true;
    } else {
      dash_length = left_dash_line_len;
    }
  } else if (direction == RIGHT_CHANGE && right_dash_line_len > 0.) {
    if (right_dash_line_len > ego_vel * kNeedLaneChangeTime) {
      LOG_DEBUG("dashed_enough: right_dash_line_len > ego_vel * 4 \n");
      return true;
    } else {
      LOG_DEBUG("dashed_enough: right_dash_line_len <= ego_vel * 4 \n");
      dash_length = right_dash_line_len;
    }
  } else {
    LOG_ERROR("!dashed_enough \n");
    return false;
  }
  dash_length = (dash_length > kInputBoundaryLenLimit) ? kDefaultBoundaryLen
                                                       : dash_length;
  double error_buffer = std::fmin(ego_vel * 0.5, 5);
  dash_length -= error_buffer;

  double v_target =
      session_->environmental_model().get_ego_state_manager()->ego_v_cruise();

  double distance_thld = std::max(v_target, ego_vel) * 4.0;
  // bool must_change_lane =
  //     virtual_lane_mgr->get_current_lane()->must_change_lane(distance_thld);
  auto current_lane = virtual_lane_mgr_->get_current_lane();
  auto must_change_lane =
      current_lane != nullptr
          ? virtual_lane_mgr_->must_change_lane(current_lane, distance_thld)
          : false;
  if (!must_change_lane && cal_lat_offset(ego_vel, dash_length) < 3.6) {
    LOG_ERROR("!dashed_enough \n");
    return false;
  }

  return true;
}

bool LaneChangeRequest::compute_lc_valid_info(RequestType direction) {
  assert(direction == LEFT_CHANGE || direction == RIGHT_CHANGE);
  auto &lateral_obstacle =
      session_->environmental_model().get_lateral_obstacle();
  auto &ego_state = session_->environmental_model().get_ego_state_manager();

  const auto &lateral_output =
      session_->planning_context().lateral_behavior_planner_output();
  const auto &vel_sequence = lateral_output.vel_sequence;
  std::shared_ptr<VirtualLane> target_lane = lane_change_lane_mgr_->tlane();

  if (direction == LEFT_CHANGE) {
    target_lane = virtual_lane_mgr_->get_left_lane();
  } else if (direction == RIGHT_CHANGE) {
    target_lane = virtual_lane_mgr_->get_right_lane();
  }

  auto origin_lane = lane_change_lane_mgr_->olane();
  auto fix_lane = lane_change_lane_mgr_->flane();
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
    return false;
  }
  if (fix_lane == nullptr) {
    fix_lane = virtual_lane_manager->get_current_lane();
  }

  // int current_lane_virtual_id = session_->mutable_environmental_model()
  //                                   ->virtual_lane_manager()
  //                                   ->current_lane_virtual_id();
  std::cout << "compute_lc_valid_info: fix_lane_virtual_id: "
            << lane_change_lane_mgr_->fix_lane_virtual_id()
            << " fix_lane->get_virtual_id(): " << fix_lane->get_virtual_id()
            << std::endl;
  std::cout << "compute_lc_valid_info: target_lane_virtual_id: "
            << lane_change_lane_mgr_->target_lane_virtual_id()
            << " target_lane->get_virtual_id(): "
            << target_lane->get_virtual_id() << std::endl;
  std::cout << "compute_lc_valid_info: clane_virtual_id: "
            << virtual_lane_manager->current_lane_virtual_id() << std::endl;
  auto reference_path_manager =
      session_->mutable_environmental_model()->get_reference_path_manager();
  auto fix_reference_path = reference_path_manager->get_reference_path_by_lane(
      lane_change_lane_mgr_->fix_lane_virtual_id());
  auto target_reference_path =
      reference_path_manager->get_reference_path_by_lane(
          target_lane->get_virtual_id());
  // auto &frenet_obstacles = current_reference_path->get_obstacles();
  auto &frenet_ego_state = fix_reference_path->get_frenet_ego_state();
  auto &target_lane_frenet_ego_state =
      target_reference_path->get_frenet_ego_state();

  lc_invalid_track_.reset();

  if (!lateral_obstacle->sensors_okay()) {
    return false;
  }

  double v_ego = ego_state->ego_v();
  double l_ego = frenet_ego_state.l();
  double safety_dist = v_ego * v_ego * 0.02 + 2.0;
  double dist_mline = std::fabs(target_lane_frenet_ego_state.l()) -
                      1.8;  // TODO(Rui): use target_lane()->get_lane_width()
  double t_reaction = (dist_mline == DBL_MAX) ? 0.5 : 0.5 * dist_mline / 1.8;

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

          //如果目标车道上后车的相对车速较自车快2m/s以上,
          //最小安全距离的阈值再增加2m，以免产生变道恐慌感
          if (tr.v_rel > 2.0) {
            mss = mss + 2;
          }
          if ((tr.d_rel > -5.0 - mss && tr.v_lead > 1) ||
              (tr.v_lead <= 1 && tr.d_rel > -5.0)) {
            is_side_target_valid = false;
            // lc_invalid_reason_ = "side view invalid";
            lc_invalid_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
          }
        }
      }
    } else {
      break;
    }
  }
  if (!is_side_target_valid) {
    return false;
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
    return false;
  }

  return true;
}

bool LaneChangeRequest::IsDashEnoughForRepeatSegments(
    const RequestType lc_request,
    const int origin_lane_id) const {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const double ego_v = ego_state->ego_v();
  double dash_length = 0.0;
  bool all_lane_boundary_types_are_dashed = true;
  double default_lc_boundary_length = 100.0;
  bool first_solid_second_dashed = false;
  double need_lane_change_time = 4.0;
  std::shared_ptr<planning_math::KDPath> target_boundary_path;
  const std::shared_ptr<VirtualLane> current_lane = virtual_lane_mgr_->get_lane_with_virtual_id(origin_lane_id);

  const auto& plannig_init_point = ego_state->planning_init_point();
  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  double ego_s = 0.0, ego_l = 0.0;
  if (lc_request == LEFT_CHANGE) {
    const auto& left_lane_boundarys = current_lane->get_left_lane_boundary();
    target_boundary_path = virtual_lane_mgr_->MakeBoundaryPath(left_lane_boundarys);
    if (target_boundary_path != nullptr) {
      if (!target_boundary_path->XYToSL(ego_x, ego_y, &ego_s, &ego_l)) {
        return false;
      }
    } else {
      return false;
    }
    if (left_lane_boundarys.type_segments[0].type ==
        iflyauto::LaneBoundaryType_MARKING_SOLID && 
        ego_s > left_lane_boundarys.type_segments[0].length &&
        ((left_lane_boundarys.type_segments[1].type ==
        iflyauto::LaneBoundaryType_MARKING_DASHED) ||
        (left_lane_boundarys.type_segments[1].type ==
        iflyauto::LaneBoundaryType_MARKING_VIRTUAL))) {
      first_solid_second_dashed = true;
    }
    for (int i = 0; i < left_lane_boundarys.type_segments_size; i++) {
      if (left_lane_boundarys.type_segments[i].type ==
          iflyauto::LaneBoundaryType_MARKING_DASHED ||
          left_lane_boundarys.type_segments[i].type ==
          iflyauto::LaneBoundaryType_MARKING_VIRTUAL ||
          (first_solid_second_dashed && i == 0)) {
        dash_length += left_lane_boundarys.type_segments[i].length;
      } else {
        all_lane_boundary_types_are_dashed = false;
        break;
      }
    }
  } else if (lc_request == RIGHT_CHANGE) {
    const auto& right_lane_boundarys = current_lane->get_right_lane_boundary();
    target_boundary_path = virtual_lane_mgr_->MakeBoundaryPath(right_lane_boundarys);
    if (target_boundary_path != nullptr) {
      if (!target_boundary_path->XYToSL(ego_x, ego_y, &ego_s, &ego_l)) {
        return false;
      }
    } else {
      return false;
    }
    if (right_lane_boundarys.type_segments[0].type ==
        iflyauto::LaneBoundaryType_MARKING_SOLID && 
        ego_s > right_lane_boundarys.type_segments[0].length &&
        ((right_lane_boundarys.type_segments[1].type ==
        iflyauto::LaneBoundaryType_MARKING_DASHED) ||
        (right_lane_boundarys.type_segments[1].type ==
        iflyauto::LaneBoundaryType_MARKING_VIRTUAL)))  {
      first_solid_second_dashed = true;
    }
    for (int i = 0; i < right_lane_boundarys.type_segments_size; i++) {
      if (right_lane_boundarys.type_segments[i].type ==
          iflyauto::LaneBoundaryType_MARKING_DASHED ||
          right_lane_boundarys.type_segments[i].type ==
          iflyauto::LaneBoundaryType_MARKING_VIRTUAL ||
          (first_solid_second_dashed && i == 0)) {
        dash_length += right_lane_boundarys.type_segments[i].length;
      } else {
        all_lane_boundary_types_are_dashed = false;
        break;
      }
    }
  }

  double lc_response_dist = ego_v * need_lane_change_time;  // hack
  dash_length -= ego_s;
  JSON_DEBUG_VALUE("dash_line_len", dash_length);
  std::cout << "dash_length:" << dash_length
            << ",lc_response_dist:" << lc_response_dist << std::endl;
  if (dash_length > default_lc_boundary_length || all_lane_boundary_types_are_dashed || dash_length > lc_response_dist) {
    return true;
  }

  std::cout << "dash lengh less than lc response dist!!!!" << std::endl;
  return false;
}

iflyauto::LaneBoundaryType LaneChangeRequest::MakesureCurrentBoundaryType(
    const RequestType lc_request,
    const int origin_lane_id) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  double lane_line_length = 0.0;
  std::shared_ptr<planning_math::KDPath> target_boundary_path;
  const auto& plannig_init_point = ego_state->planning_init_point();
  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  double ego_s = 0.0, ego_l = 0.0;
  const std::shared_ptr<VirtualLane> current_lane = virtual_lane_mgr_->get_lane_with_virtual_id(origin_lane_id);

  if (lc_request == LEFT_CHANGE) {
    const auto& left_lane_boundarys = current_lane->get_left_lane_boundary();
    target_boundary_path = virtual_lane_mgr_->MakeBoundaryPath(left_lane_boundarys);
    if (target_boundary_path != nullptr) {
      if (!target_boundary_path->XYToSL(ego_x, ego_y, &ego_s, &ego_l)) {
        return iflyauto::LaneBoundaryType_MARKING_SOLID;
      }
    } else {
      return iflyauto::LaneBoundaryType_MARKING_SOLID;
    }
    for (int i = 0; i < left_lane_boundarys.type_segments_size; i++) {
      lane_line_length += left_lane_boundarys.type_segments[i].length;
      if (lane_line_length > ego_s) {
        return left_lane_boundarys.type_segments[i].type;
      }
    }
  } else if (lc_request == RIGHT_CHANGE) {
    const auto& right_lane_boundarys = current_lane->get_right_lane_boundary();
    target_boundary_path = virtual_lane_mgr_->MakeBoundaryPath(right_lane_boundarys);
    if (target_boundary_path != nullptr) {
      if (!target_boundary_path->XYToSL(ego_x, ego_y, &ego_s, &ego_l)) {
        return iflyauto::LaneBoundaryType_MARKING_SOLID;
      }
    } else {
      return iflyauto::LaneBoundaryType_MARKING_SOLID;
    }
    for (int i = 0; i < right_lane_boundarys.type_segments_size; i++) {
      lane_line_length += right_lane_boundarys.type_segments[i].length;
      if (lane_line_length > ego_s) {
        return right_lane_boundarys.type_segments[i].type;
      }
    }
  } else {
    return iflyauto::LaneBoundaryType_MARKING_SOLID;;
  }
}
}  // namespace planning