
#include "parking_slot_manager.h"
#include <limits>
#include "ego_state_manager.h"
#include "environmental_model.h"
#include "log.h"
namespace planning {
const double kMaxDistanceX = 40;
const double kMaxDistanceFrontY = 60;
const double kMaxDistanceBackY = 30;
ParkingSlotManager::ParkingSlotManager(planning::framework::Session *session)
    : session_(session) {}
bool ParkingSlotManager::update(
    const IFLYParkingMap::ParkingInfo& parking_info) {
  points_.clear();
  const auto ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto park_spaces = parking_info.road_tile_info().parking_space();
  const auto &enu2car_matrix = ego_state_manager->get_enu2car();
  Eigen::Vector3d v;
  double min_x, min_y;
  for (const auto &park_space : park_spaces) {
    ParkingSlotPoints slot_point;
    if (park_space.shape().size() != 4) {
      LOG_NOTICE("park_space point size is not 4");
      continue;
    }

    min_x = std::numeric_limits<double>::max();
    min_y = std::numeric_limits<double>::max();
    for (auto lot_point : park_space.shape()) {
      v.x() = lot_point.enu().x();
      v.y() = lot_point.enu().y();
      v.z() = lot_point.enu().z();
      auto park_space_point_car = enu2car_matrix(v);
      if (park_space_point_car.x() < min_x) {
        min_x = park_space_point_car.x();
      }
      if (park_space_point_car.y() < min_y) {
        min_y = park_space_point_car.y();
      }
      slot_point.emplace_back(
          planning_math::Vec2d(lot_point.enu().x(), lot_point.enu().y()));
    }
    if (fabs(min_x) < kMaxDistanceX && min_y < kMaxDistanceFrontY &&
        min_y > -kMaxDistanceBackY) {
      points_.emplace_back(slot_point);
    }
  }
  return true;
}  // namespace planning
}  // namespace planning
