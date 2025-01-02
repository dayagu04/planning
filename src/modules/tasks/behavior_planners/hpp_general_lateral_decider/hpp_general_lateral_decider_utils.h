#pragma once
#include "common/math/polygon2d.h"
#include "common_c.h"
#include "config/basic_type.h"
#include "ego_planning_config.h"
#include "math.h"
#include "obstacle.h"
#include "task_basic_types.h"
#include "utils/kd_path.h"

namespace planning {
namespace hpp_general_lateral_decider_utils {
double CalDesireLateralDistance(const double ego_vel, const double pred_ts,
                                const double agent_lateral_relative_speed,
                                iflyauto::ObjectType type,
                                const bool is_nudge_left, bool in_intersection,
                                HppGeneralLateralDeciderConfig &config);

double CalDesireLonDistance(double ego_vel, double agent_vel);
double CalDesireStaticLateralDistance(const double base_distance,
                                      const double ego_vel, const double ego_l,
                                      iflyauto::ObjectType type,
                                      bool is_update_hard_bound);
double GetBoundWeight(
    BoundType type,
    const std::unordered_map<BoundType, double> &map_bound_weight);
int GetBoundTypePriority(BoundType type);

std::vector<int> MatchRefTrajPoints(int s,
                                    const TrajectoryPoints &ref_traj_points);

TrajectoryPoint GetTrajectoryPointAtTime(
    const TrajectoryPoints trajectory_points, const double relative_time);

bool IsVRU(iflyauto::ObjectType type);
bool IsCone(iflyauto::ObjectType type);
bool IsTruck(iflyauto::ObjectType type);

ObstacleBorderInfo GetNearestObstacleBorder(
    const planning_math::Polygon2d &care_polygon, double care_area_s_start,
    double care_area_s_end,
    const std::vector<std::pair<int, planning_math::Polygon2d>>
        &obstacle_frenet_polygons,
    bool is_left, bool is_sorted, bool is_curve, int index,
    const TrajectoryPoints &traj_points);
void MakeLinePolygons(
    const Obstacle *const &obstacle,
    const std::shared_ptr<ReferencePath> &reference_path_ptr,
    std::vector<std::pair<int, planning_math::Polygon2d>> &left_polygons,
    std::vector<std::pair<int, planning_math::Polygon2d>> &right_polygons);
void MakePolygon(
    const int obstacle_id, const std::shared_ptr<KDPath> &frenet_coord,
    const planning_math::Polygon2d &polygon,
    std::vector<std::pair<int, planning_math::Polygon2d>> &left_polygons,
    std::vector<std::pair<int, planning_math::Polygon2d>> &right_polygons);
}  // namespace hpp_general_lateral_decider_utils
}  // namespace planning