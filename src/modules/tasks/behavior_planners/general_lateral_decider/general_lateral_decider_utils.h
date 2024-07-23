#pragma once
#include "math.h"
#include "task_basic_types.h"
#include "config/basic_type.h"
#include "common_c.h"
namespace planning {
namespace general_lateral_decider_utils {
double CalDesireLateralDistance(const double ego_vel, const double pred_ts,
                              const double agent_lateral_relative_speed,
                              iflyauto::ObjectType type,
                              const bool is_nudge_left);

double CalDesireLonDistance(double ego_vel, double agent_vel);
double CalDesireStaticLateralDistance(const double base_distance, const double ego_vel, const double ego_l, iflyauto::ObjectType type, bool is_update_hard_bound);
int GetBoundTypePriority(BoundType type);

std::vector<int> MatchRefTrajPoints(int s, const TrajectoryPoints &ref_traj_points);

TrajectoryPoint GetTrajectoryPointAtTime(const TrajectoryPoints trajectory_points,
    const double relative_time);

bool IsVRU(iflyauto::ObjectType type);
bool IsCone(iflyauto::ObjectType type);
bool IsTruck(iflyauto::ObjectType type);
}
}