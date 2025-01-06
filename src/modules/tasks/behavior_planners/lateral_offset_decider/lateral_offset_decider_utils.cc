#include "lateral_offset_decider_utils.h"
#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>
#include "environmental_model.h"
#include "planning_context.h"

namespace planning {
namespace lateral_offset_decider {
bool IsStaticObstacle(const TrackedObject &tr) { return tr.v < 0.1; }

bool IsStaticObstacle(const AvoidObstacleInfo &avoid_obstacle) {
  return avoid_obstacle.vs < 3;
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

bool IsFasterThanEgo(const TrackedObject &tr) {
  // TODO(clren): consider lon
  std::array<double, 5> drel_x{1, 2, 5, 10, 20};
  std::array<double, 5> vrel_y{4, 2, 1.8, 1, 0.3};

  double min_desire_v = interp(tr.d_rel, drel_x, vrel_y);
  return tr.v_rel > min_desire_v;
}

bool IsFasterThanAvoidObstacle(const TrackedObject &tr,
                               const AvoidObstacleInfo &avoid_obstacle) {
  return tr.v_lead > avoid_obstacle.vs;
}

// is_left: True: left tracked_object
//          False: right tracked_object
bool IsInConsiderFrontLateralRange(
    const framework::Session *session, const TrackedObject &tr, bool is_left,
    double normal_avoid_threshold,
    std::map<HysteresisType,
             std::variant<std::map<int, HysteresisDecision>,
                          std::map<std::pair<int, int>, HysteresisDecision>>>
        &hysteresis_maps) {
  const CoarsePlanningInfo &coarse_planning_info =
      session->planning_context()
          .lane_change_decider_output()
          .coarse_planning_info;
  const auto fix_ref =
      session->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(coarse_planning_info.target_lane_id);
  auto &is_in_consider_lateral_range_hysteresis_map =
      std::get<std::map<int, HysteresisDecision>>(
          hysteresis_maps[HysteresisType::IsInConsiderLateralRangeHysteresis]);

  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double half_ego_width = vehicle_param.max_width * 0.5;
  const double safe_lat_distance = 0.4;
  const double max_lat_position =
      -normal_avoid_threshold + half_ego_width + safe_lat_distance;

  if (is_left) {
    is_in_consider_lateral_range_hysteresis_map[tr.track_id].SetIsValidByValue(
        tr.d_min_cpath - max_lat_position);
  } else {
    is_in_consider_lateral_range_hysteresis_map[tr.track_id].SetIsValidByValue(
        -(tr.d_max_cpath + max_lat_position));
  }

  if (!is_in_consider_lateral_range_hysteresis_map[tr.track_id].IsValid()) {
    return false;
  }

  return true;
}

bool IsInConsiderSideLateralRange(
    const framework::Session *session, const TrackedObject &tr, bool is_left,
    std::map<HysteresisType,
             std::variant<std::map<int, HysteresisDecision>,
                          std::map<std::pair<int, int>, HysteresisDecision>>>
        &hysteresis_maps) {
  const CoarsePlanningInfo &coarse_planning_info =
      session->planning_context()
          .lane_change_decider_output()
          .coarse_planning_info;
  const auto fix_ref =
      session->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(coarse_planning_info.target_lane_id);
  auto &is_in_consider_lateral_range_hysteresis_map =
      std::get<std::map<int, HysteresisDecision>>(
          hysteresis_maps[HysteresisType::IsInConsiderLateralRangeHysteresis]);

  double lat_dis =
      is_left
          ? tr.d_min_cpath - fix_ref->get_frenet_ego_state().boundary().l_end
          : fix_ref->get_frenet_ego_state().boundary().l_start - tr.d_max_cpath;

  if (lat_dis <= 0) {
    return false;
  }

  const double max_lat_position = 1.0;
  if (is_left) {
    is_in_consider_lateral_range_hysteresis_map[tr.track_id].SetIsValidByValue(
        tr.d_min_cpath - max_lat_position);
  } else {
    is_in_consider_lateral_range_hysteresis_map[tr.track_id].SetIsValidByValue(
        -(tr.d_max_cpath + max_lat_position));
  }

  if (!is_in_consider_lateral_range_hysteresis_map[tr.track_id].IsValid()) {
    return false;
  }

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
  if (1) {
    const auto ego_cart_state_manager =
        session->environmental_model().get_ego_state_manager();
    const double v_ego = ego_cart_state_manager->ego_v();
    const auto &vehicle_param =
        VehicleConfigurationContext::Instance()->get_vehicle_param();
    const double ego_length = vehicle_param.length;
    const double v_obstacle_2 = v_ego + tr.v_rel;
    std::array<double, 3> t_gap_vego_v{1.35, 1.55, 2.0};
    std::array<double, 3> t_gap_vego_bp{5, 15, 30};
    const double safety_dist = 2.0;

    // desired t_gap to obstacle_2 when exceed obstacle_1
    const double t_gap = interp(v_ego, t_gap_vego_bp, t_gap_vego_v);

    // desired distance to obstacle_2 when exceed obstacle_1
    const double desired_distance_to_obstacle_2 =
        safety_dist + v_obstacle_2 * t_gap;
    double relative_distance_nudge_obstacle =
        tr.tail_rel_s - avoid_obstacle.tail_s_to_ego;
    const double relative_vel_nudge_obstacle =
        tr.v_rel - avoid_obstacle.vs_lon_relative;
    const double distance_exceed_obstacle_1 = avoid_obstacle.tail_s_to_ego +
                                              avoid_obstacle.length +
                                              ego_length + safety_dist;

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

    // relative distance between obstacle_1 and obstacle_2 when exceed
    // obstacle_1
    relative_distance_nudge_obstacle +=
        relative_vel_nudge_obstacle * t_exceed_obstacle_1;

    bool is_side_way = relative_distance_nudge_obstacle >=
                       desired_distance_to_obstacle_2 + 0.5 * v_ego;
    return is_side_way;
  } else {
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
                                              avoid_obstacle.length +
                                              ego_length + safety_dist;
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

    // relative distance between obstacle_1 and obstacle_2 when exceed
    // obstacle_1
    relative_distance_nudge_obstacle +=
        relative_vel_nudge_obstacle * (t_exceed_obstacle_1);

    bool is_side_way =
        relative_distance_nudge_obstacle >=
        desired_distance_to_obstacle_2 + 5.0 + safety_dist + 0.5 * v_ego;
    return is_side_way;
  }
}

bool IsFrontObstacle(const TrackedObject &tr) { return tr.d_rel > 0; }

bool IsCutIn(const TrackedObject &tr) { return tr.cutinp >= 0.4; }

bool IsFrontObstacleConsider(
    const framework::Session *session, const TrackedObject &tr, bool is_left,
    const AvoidInfo &avoid_info,
    std::map<HysteresisType,
             std::variant<std::map<int, HysteresisDecision>,
                          std::map<std::pair<int, int>, HysteresisDecision>>>
        &hysteresis_maps) {
  if (!IsCameraObstacle(tr)) {
    return false;
  }

  if (IsFrontObstacle(tr) && (tr.is_lead)) {
    return false;
  }

  // 考虑横向速度
  // 考虑是否静态障碍物
  if (!IsInConsiderFrontLateralRange(
          session, tr, is_left,
          is_left ? avoid_info.normal_left_avoid_threshold
                  : avoid_info.normal_right_avoid_threshold,
          hysteresis_maps)) {
    return false;
  }

  if (!IsAboutToEnterLonRange(tr, true)) {
    return false;
  }

  if (IsFasterThanEgo(tr)) {
    return false;
  }

  return true;
}

bool IsSideObstacleConsider(
    const framework::Session *session, const TrackedObject &tr, bool is_left,
    std::map<HysteresisType,
             std::variant<std::map<int, HysteresisDecision>,
                          std::map<std::pair<int, int>, HysteresisDecision>>>
        &hysteresis_maps) {
  if (!IsCameraObstacle(tr)) {
    return false;
  }

  if (!IsInConsiderSideLateralRange(session, tr, is_left, hysteresis_maps)) {
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
  return (avoid_obstacle.type == Common::ObjectType::OBJECT_TYPE_BUS ||
          (avoid_obstacle.type == Common::ObjectType::OBJECT_TYPE_TRUCK &&
          avoid_obstacle.length > 6));
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
  double ttc = avoid_obstacle.first_s_to_ego /
               (std::max(-avoid_obstacle.vs_lon_relative, 1e-6));
  bool is_passive = false;
  if (ttc <= 1.0 || avoid_obstacle.first_s_to_ego < 5) {
    is_passive = true;
  }
  return is_passive;
}

bool HasEnoughSpace(const AvoidObstacleInfo &avoid_obstacle_1,
                    const AvoidObstacleInfo &avoid_obstacle_2) {
  static HysteresisDecision has_enough_space_hysteresis(3.0, 2.8);
  static int ids[2];
  if (!((ids[0] == avoid_obstacle_1.track_id &&
         ids[1] == avoid_obstacle_2.track_id) ||
        (ids[0] == avoid_obstacle_2.track_id &&
         ids[1] == avoid_obstacle_1.track_id))) {
    ids[0] = avoid_obstacle_1.track_id;
    ids[1] = avoid_obstacle_2.track_id;
    has_enough_space_hysteresis.Reset();
  }
  has_enough_space_hysteresis.SetIsValidByValue(
      std::max(avoid_obstacle_1.min_l_to_ref - avoid_obstacle_2.max_l_to_ref,
               avoid_obstacle_2.min_l_to_ref - avoid_obstacle_1.max_l_to_ref));

  return has_enough_space_hysteresis.IsValid();
}

}  // namespace lateral_offset_decider
}  // namespace planning