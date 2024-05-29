#include "lateral_offset_decider_utils.h"
#include <algorithm>
#include <vector>
#include "environmental_model.h"
#include "planning_context.h"
namespace planning {
namespace lateral_offset_decider {

bool IsStaticObstacle(const TrackedObject &tr) {
  return tr.v < 0.1;
}

bool IsAboutToEnterLonRange(const TrackedObject &tr, bool is_front) {
  if (is_front) {
    return tr.d_rel < std::max(std::min(-tr.v_rel * 15, 60.0), 20.0);
  } else {
    const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
    const double ego_length = vehicle_param.length;
    if (IsStaticObstacle(tr)) {
      return tr.d_rel > -ego_length - 1;
    } else {
      return tr.d_rel > std::min(-10.0 - tr.v_rel * 2, -5.0) && tr.d_rel < 2;
    }
  }
}

bool IsFasterObstacle(const TrackedObject &tr) {
  // TODO(clren): consider lon
  std::array<double, 5> drel_x{1, 2, 5, 10, 20};
  std::array<double, 5> vrel_y{4, 2, 1.8, 1, 0.3};

  double min_desire_v = interp(tr.d_rel, drel_x, vrel_y);
  return tr.v_rel > min_desire_v;
}

// is_left: True: left tracked_object
//          False: right tracked_object
bool IsInConsiderLateralRange(const framework::Session *session,
                              const TrackedObject &tr, bool is_left) {
  const CoarsePlanningInfo &coarse_planning_info =
      session->planning_context()
          .lane_change_decider_output()
          .coarse_planning_info;
  const auto fix_ref =
      session->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(coarse_planning_info.target_lane_id);
  const double ego_position =
      is_left ? fix_ref->get_frenet_ego_state().boundary().l_end
              : fix_ref->get_frenet_ego_state().boundary().l_start;
  double lat_dis =
      is_left
          ? tr.d_min_cpath - fix_ref->get_frenet_ego_state().boundary().l_end
          : fix_ref->get_frenet_ego_state().boundary().l_start - tr.d_max_cpath;

  if (lat_dis <= 0) {
    return false;
  }

  bool is_in_lateral_range = is_left ? tr.d_min_cpath > 1: tr.d_max_cpath < -1;
  if (!is_in_lateral_range) {
    return false;
  }
  double t_lookforward = 1.0;
  double predict_lon_distance = tr.d_rel + tr.v_rel * t_lookforward;
  // tr.v_lat * t_lookforward
  // TODO(clren):consider prediction;  v_lat > 0.3
  // cutin
  // std::array<double, 5> drel_x{1, 2, 5, 10, 20};
  // std::array<double, 5> vrel_y{4, 2, 1.8, 1, 0.3};

  // double min_desire_v = interp(tr.d_rel, drel_x, vrel_y);
  // return tr.v_rel > min_desire_v;

  // if (tr.d_path_self &&tr.v_lat < -0.3) {
  //   return false;
  // }

  return true;
}

bool IsCameraObstacle(const TrackedObject &tr) {
  return tr.fusion_source & OBSTACLE_SOURCE_CAMERA;
}

// return true: avoid obstacle_1 first, then avoid obstacle_2
// return false: decide other way later
bool AvoidWaySelectForTwoObstaclev2(const framework::Session *session,
                                    const AvoidObstacleInfo &avoid_obstacle,
                                    const TrackedObject &tr) {
  const auto ego_cart_state_manager =
      session->environmental_model().get_ego_state_manager();
  const double v_ego = ego_cart_state_manager->ego_v();
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double ego_length = vehicle_param.length;
  const double v_obstacle_2 = v_ego + tr.v_rel;
  const double kDistanceOffset = 3.5;
  std::array<double, 3> t_gap_vego_v{1.35, 1.55, 2.0};
  std::array<double, 3> t_gap_vego_bp{5, 15, 30};
  const double safety_dist = 2.0 + v_ego * 0.2;

  // desired t_gap to obstacle_2 when exceed obstacle_1
  const double t_gap = interp(v_ego, t_gap_vego_bp, t_gap_vego_v);

  // desired distance to obstacle_2 when exceed obstacle_1
  const double desired_distance_to_obstacle_2 =
      kDistanceOffset + v_obstacle_2 * t_gap;
  double relative_distance_nudge_obstacle =
      tr.tail_rel_s - avoid_obstacle.tail_s_to_ego;
  const double relative_vel_nudge_obstacle =
      tr.v_rel - avoid_obstacle.vs_lon_relative;
  const double distance_exceed_obstacle_1 = avoid_obstacle.tail_s_to_ego +
                                            avoid_obstacle.length + ego_length +
                                            safety_dist;

  double t_exceed_obstacle_1 = 0.0;
  if (equal_zero(avoid_obstacle.vs_lon_relative) == false) {
    if (avoid_obstacle.vs_lon_relative < -1.0e-3) {
      t_exceed_obstacle_1 =
          -distance_exceed_obstacle_1 / avoid_obstacle.vs_lon_relative;
    } else {
      t_exceed_obstacle_1 = 5.;
    }
  } else {
    t_exceed_obstacle_1 = (v_ego < 1) ? 5. : 0.;
  }

  // relative distance between obstacle_1 and obstacle_2 when exceed obstacle_1
  relative_distance_nudge_obstacle +=
      relative_vel_nudge_obstacle * t_exceed_obstacle_1;

  bool is_side_way =
      relative_distance_nudge_obstacle >=
      desired_distance_to_obstacle_2 + 5.0 + safety_dist + 0.5 * v_ego;
  return is_side_way;
}

bool IsFrontObstacle(const TrackedObject &tr) { return tr.d_rel > 0; }

bool IsCutIn(const TrackedObject &tr) { return tr.cutinp > 0.2; }

bool IsFrontObstacleConsider(const framework::Session *session,
                             const TrackedObject &tr,
                             const AvoidObstacleInfo &avoid_obstacle,
                             bool is_left) {
  if (!IsCameraObstacle(tr)) {
    return false;
  }

  if (IsFrontObstacle(tr) &&
      (IsCutIn(tr) || tr.is_lead)) {
    return false;
  }

  if (!IsInConsiderLateralRange(session, tr, is_left)) {
    return false;
  }

  if (!IsAboutToEnterLonRange(tr, true)) {
    return false;
  }

  if (IsFasterObstacle(tr)) {
    return false;
  }

  if (AvoidWaySelectForTwoObstaclev2(session, avoid_obstacle, tr)) {
    return false;
  }
  return true;
}

bool IsSideObstacleConsider(const framework::Session *session,
                            const TrackedObject &tr, bool is_left) {
  if (!IsCameraObstacle(tr)) {
    return false;
  }

  bool is_in_lateral_range = is_left ? tr.d_min_cpath > 1: tr.d_max_cpath < -1;
  if (!is_in_lateral_range) {
    return false;
  }

  if (!IsInConsiderLateralRange(session, tr, is_left)) {
    return false;
  }

  if (!IsAboutToEnterLonRange(tr, false)) {
    return false;
  }

  return true;
}

bool HasOverlap(const framework::Session *session,
                const AvoidObstacleInfo &avoid_obstacle, double front_lon_buf,
                double rear_lon_buf) {
  const CoarsePlanningInfo &coarse_planning_info =
      session->planning_context()
          .lane_change_decider_output()
          .coarse_planning_info;
  const auto fix_ref =
      session->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(coarse_planning_info.target_lane_id);

  double ego_s_start = fix_ref->get_frenet_ego_state().boundary().s_start;
  double ego_s_end = fix_ref->get_frenet_ego_state().boundary().s_end;
  const auto frenet_obstacle_map = fix_ref->get_obstacles_map();
  double obstacle_s_start, obstacle_s_end;
  if (frenet_obstacle_map.find(avoid_obstacle.track_id) !=
      frenet_obstacle_map.end()) {
    const auto frenet_obstacle_boundary =
        frenet_obstacle_map.at(avoid_obstacle.track_id)
            ->frenet_obstacle_boundary();
    obstacle_s_start = frenet_obstacle_boundary.s_start;
    obstacle_s_end = frenet_obstacle_boundary.s_end;
  } else {
    obstacle_s_start = ego_s_start + avoid_obstacle.tail_s_to_ego;
    obstacle_s_end = obstacle_s_start + avoid_obstacle.length;
  }
  ego_s_end += front_lon_buf;
  ego_s_start -= rear_lon_buf;
  return obstacle_s_end >= ego_s_start && ego_s_end >= obstacle_s_start;
}

bool IsTruck(const AvoidObstacleInfo &avoid_obstacle) {
  return (avoid_obstacle.type == iflyauto::OBJECT_TYPE_BUS ||
          avoid_obstacle.type == iflyauto::OBJECT_TYPE_TRUCK) &&
         avoid_obstacle.length > lateral_offset_decider::kTruckMinLength;
}

bool IsVRU(const AvoidObstacleInfo &avoid_obstacle) {
  return avoid_obstacle.type == iflyauto::OBJECT_TYPE_BICYCLE ||
         avoid_obstacle.type == iflyauto::OBJECT_TYPE_PEDESTRIAN ||
         avoid_obstacle.type == iflyauto::OBJECT_TYPE_TRICYCLE ||
         avoid_obstacle.type == iflyauto::OBJECT_TYPE_ANIMAL;
}

bool IsCone(const AvoidObstacleInfo &avoid_obstacle) {
  return avoid_obstacle.type == iflyauto::OBJECT_TYPE_TRAFFIC_CONE;
}

bool IsPassive(const AvoidObstacleInfo &avoid_obstacle) {
  double ttc = avoid_obstacle.first_s_to_ego / (std::max(-avoid_obstacle.vs_lon_relative, 1e-6));
  bool is_passive = false;
  if (ttc <= 1.0 || avoid_obstacle.first_s_to_ego < 5 ) {
    is_passive = true;
  }
  return is_passive;
}

double GetLimitLateralDistance(const framework::Session *session, int track_id) {
  const CoarsePlanningInfo &coarse_planning_info =
      session->planning_context()
          .lane_change_decider_output()
          .coarse_planning_info;
  const auto fix_ref =
      session->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(coarse_planning_info.target_lane_id);
  const auto frenet_ego_state = fix_ref->get_frenet_ego_state();
  double ego_s_start = frenet_ego_state.boundary().s_start;
  double ego_s_end = frenet_ego_state.boundary().s_end;
  const auto frenet_obstacle_map = fix_ref->get_obstacles_map();
  double obstacle_s_start, obstacle_s_end;
  if (frenet_obstacle_map.find(track_id) !=
      frenet_obstacle_map.end()) {
    const auto& frenet_obstacle = frenet_obstacle_map.at(track_id);
    if (frenet_obstacle->b_frenet_valid()) {
      const auto frenet_obstacle_boundary =
        frenet_obstacle->frenet_obstacle_boundary();
      obstacle_s_start = frenet_obstacle_boundary.s_start;
      obstacle_s_end = frenet_obstacle_boundary.s_end;

      double lon_distance = 0.0;
      bool is_overlap = obstacle_s_end >= ego_s_start && ego_s_end >= obstacle_s_start;
      if (is_overlap) {
        lon_distance = 0.0;
      } else {
        if (obstacle_s_start > ego_s_end) {
          lon_distance = obstacle_s_start - ego_s_end;
        } else {
          lon_distance = obstacle_s_end - ego_s_start;
        }
      }

      // const double relative_v = frenet_obstacle->frenet_velocity_s() - frenet_ego_state->frenet_velocity_s();
      // const double t = lon_distance / relative_v;
      // std::array<double, 3> l_buffer_x{0, 0.05,  0.13, 0.15, 0.20, 0.25};
      // std::array<double, 3> t_gap_bp{1, 2, 4, 6, 8, 10};

      // // desired t_gap to obstacle_2 when exceed obstacle_1
      // const double l_buffer = interp(t, t_gap_bp, l_buffer_x);
    }
  } else {

  }
}

bool HasEnoughSpace(const AvoidObstacleInfo &avoid_obstacle_1,
                    const AvoidObstacleInfo &avoid_obstacle_2) {
  static bool has_enough_space = false;
  double lat_buf;
  if (has_enough_space) {
    lat_buf = 2.8;
  } else {
    lat_buf = 3.0;
  }
  has_enough_space = avoid_obstacle_1.min_l_to_ref - avoid_obstacle_2.max_l_to_ref > lat_buf ||
         avoid_obstacle_2.min_l_to_ref - avoid_obstacle_1.max_l_to_ref > lat_buf;

  return has_enough_space;
}

}  // namespace lateral_offset_decider
}  // namespace planning