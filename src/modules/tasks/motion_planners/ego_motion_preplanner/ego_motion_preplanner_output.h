#pragma once

#include <memory>

#include "lat_lon_vehicle_motion_simulator.h"
namespace planning {
class EgoMotionPreplannerOutput {
 public:
  EgoMotionPreplannerOutput() = default;
  ~EgoMotionPreplannerOutput() = default;

  std::shared_ptr<simulator::LatLonVehicleMotionSimulator::SimulationResult>&
  mutable_ego_motion_simulation_result() {
    return ego_motion_simulation_result_;
  }

  const std::shared_ptr<
      simulator::LatLonVehicleMotionSimulator::SimulationResult>&
  ego_motion_simulation_result() const {
    return ego_motion_simulation_result_;
  }

 private:
  std::shared_ptr<simulator::LatLonVehicleMotionSimulator::SimulationResult>
      ego_motion_simulation_result_;
};
}  // namespace planning