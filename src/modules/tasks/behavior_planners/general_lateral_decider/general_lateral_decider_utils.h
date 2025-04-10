#pragma once
#include "common_c.h"
#include "config/basic_type.h"
#include "ego_planning_config.h"
#include "math.h"
#include "task_basic_types.h"
namespace planning {
namespace general_lateral_decider_utils {
double CalDesireLateralDistance(const double ego_vel, const double pred_ts,
                                const double agent_lateral_relative_speed,
                                iflyauto::ObjectType type,
                                const bool is_nudge_left, bool in_intersection,
                                GeneralLateralDeciderConfig &config);

double CalDesireLonDistance(double ego_vel, double agent_vel);
double CalDesireLonOverlapDistance(double ego_vel, double agent_vel,
                                   bool use_obstacle_prediction_model_in_planning);
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
bool IsTruck(iflyauto::ObjectType type);
}  // namespace general_lateral_decider_utils
}  // namespace planning