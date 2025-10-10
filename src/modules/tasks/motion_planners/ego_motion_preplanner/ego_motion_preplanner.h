#pragma once
#include <memory.h>
#include <memory>
#include "basic_types.pb.h"
#include "config/basic_type.h"
#include "ego_planning_config.h"
#include "lat_lon_vehicle_motion_simulator.h"
#include "planning_context.h"
#include "tasks/task.h"
#include "utils/kd_path.h"

namespace planning {

class EgoMotionPreplanner : public Task {
 public:
  EgoMotionPreplanner(const EgoPlanningConfigBuilder* config_builder,
                      framework::Session* session);
  ~EgoMotionPreplanner() = default;

  bool Execute() override;

  ErrorType EgoMotionPreProcess();

  void SaveToSession();

 private:
  EgoMotionPreplannerConfig config_;
  simulator::LatLonVehicleMotionSimulator lat_lon_vehicle_motion_simulator_;
};

}  // namespace planning