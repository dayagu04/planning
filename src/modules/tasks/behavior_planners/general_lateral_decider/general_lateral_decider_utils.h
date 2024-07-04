#pragma once
#include "math.h"
#include "task_basic_types.h"
#include "config/basic_type.h"

namespace planning {
namespace general_lateral_decider_utils {
double CalDesireLateralDistance(const double ego_vel, const double pred_ts,
                              const double agent_lateral_relative_speed,
                              const bool is_vru_agent,
                              const bool is_nudge_left);

double CalDesireLonDistance(double ego_vel, double agent_vel);

int GetBoundTypePriority(BoundType type);

std::vector<int> MatchRefTrajPoints(int s, const TrajectoryPoints &ref_traj_points);

TrajectoryPoint GetTrajectoryPointAtTime(const TrajectoryPoints trajectory_points,
    const double relative_time);
}
}