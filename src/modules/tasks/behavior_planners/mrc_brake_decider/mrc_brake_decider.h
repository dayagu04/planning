#pragma once

#include <cstdint>
#include "behavior_planners/mrc_brake_decider/mrc_brake_decider_output.h"
#include "ego_planning_config.h"
#include "tasks/task.h"
#include "virtual_lane_manager.h"
#include "trajectory1d/second_order_time_optimal_trajectory.h"

namespace planning {

class MRCBrakeDecider : public Task {
  public:
    MRCBrakeDecider(const EgoPlanningConfigBuilder *config_builder,
                          framework::Session *session);
    virtual ~MRCBrakeDecider() = default;

    bool Execute() override;

    bool MRCBrakeProcess();

    void SaveToSession();

  private:
    SecondOrderTimeOptimalTrajectory GenerateMRCBrakeCurve() const;
    MRCBrakeDeciderConfig config_;
    SecondOrderTimeOptimalTrajectory mrc_brake_curv_;
    bool has_set_virtual_obs_ = false;
    planning::agent::Agent mrc_agent_;
};

}