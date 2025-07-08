#pragma once
#include "common_c.h"
#include "config/basic_type.h"
#include "ego_planning_config.h"
#include "math.h"
#include "task_basic_types.h"
#include "obstacle_manager.h"

namespace planning {
namespace general_lateral_decider_utils {
double CalDesireLateralDistance(const double ego_vel, const double pred_ts,
                                const double agent_lateral_relative_speed,
                                const std::shared_ptr<FrenetObstacle> obstacle,
                                const bool is_nudge_left, bool in_intersection,
                                bool is_same_side_obstacle_during_lane_change,
                                GeneralLateralDeciderConfig &config);

double CalDesireLonOverlapDistance(double ego_vel, double agent_vel,
                                   bool is_rear_obstacle);
double CalDesireLonDistance(double ego_vel, double agent_vel,
                            bool is_same_side_obstacle_during_lane_change,
                            GeneralLateralDeciderConfig &config);

double CalDesireStaticLateralDistance(const double base_distance,
                                      const double ego_vel, const double ego_l,
                                      iflyauto::ObjectType type,
                                      bool is_update_hard_bound,
                                      GeneralLateralDeciderConfig &config);
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
bool IsTruck(const std::shared_ptr<FrenetObstacle> obstacle);
}  // namespace general_lateral_decider_utils
}  // namespace planning