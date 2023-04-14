#include "src/modules/context/virtual_lane_manager.h"
#include "virtual_lane_manager.h"

namespace planning {

void VirtualLane::init(framework::Session* session) {}

VirtualLaneManager::VirtualLaneManager(
    framework::Session* session) {}

double VirtualLaneManager::get_distance_to_dash_line(
    const RequestType direction, int order_id) const {
  return 0.0;
}

double VirtualLaneManager::velocity_limit() const { return 0.0; }

double VirtualLaneManager::map_velocity_limit() const { return 0.0; }

bool VirtualLaneManager::Update() { return false; }

}  // namespace planning