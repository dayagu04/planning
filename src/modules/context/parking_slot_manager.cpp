
#include "parking_slot_manager.h"

namespace planning {
bool ParkingSlotManager::update(
    const IFLYParkingMap::ParkingInfo& parking_info) {
  points_.clear();
  const auto park_spaces = parking_info.road_tile_info().parking_space();
  for (const auto& park_space : park_spaces) {
    ParkingSlotPoints slot_point;
    for (auto lot_point : park_space.shape()) {
      slot_point.emplace_back(
          planning_math::Vec2d(lot_point.enu().x(), lot_point.enu().y()));
    }
    points_.emplace_back(slot_point);
  }
}
}  // namespace planning
