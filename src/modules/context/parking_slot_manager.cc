
#include "parking_slot_manager.h"
#include <limits>
#include "ego_state_manager.h"
#include "environmental_model.h"
#include "log.h"
namespace planning {
const double kMaxDistanceY = 5;
const double kMaxDistanceFrontX = 40; // 后续根据实际需求更改
const double kMaxDistanceBackX = 30;
ParkingSlotManager::ParkingSlotManager(planning::framework::Session *session)
    : session_(session) {}
bool ParkingSlotManager::update(
    const IFLYParkingMap::ParkingInfo &parking_info) {
  points_.clear();
  const auto ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto park_spaces = parking_info.road_tile_info().parking_space();
  const auto &enu2car_matrix = ego_state_manager->get_enu2car();
  Eigen::Vector3d v;
  double min_x, min_y, max_x, max_y;
  for (const auto &park_space : park_spaces) {
    ParkingSlotPoints slot_point;
    if (park_space.shape().size() != 4) {
      LOG_NOTICE("park_space point size is not 4");
      continue;
    }

    min_x = std::numeric_limits<double>::max();
    min_y = std::numeric_limits<double>::max();
    max_x = std::numeric_limits<double>::lowest();
    max_y = std::numeric_limits<double>::lowest();
    for (const auto &lot_point : park_space.shape()) {
      v.x() = lot_point.boot().x();
      v.y() = lot_point.boot().y();
      v.z() = lot_point.boot().z();
      auto park_space_point_car = enu2car_matrix(v);
      min_x = std::fmin(min_x, park_space_point_car.x());
      min_y = std::fmin(min_y, park_space_point_car.y());
      max_x = std::fmax(max_x, park_space_point_car.x());
      max_y = std::fmax(max_y, park_space_point_car.y());
      slot_point.emplace_back(
          planning_math::Vec2d(lot_point.boot().x(), lot_point.boot().y()));
    }
    if (((min_y > 0 && min_y < kMaxDistanceY) || (max_y < 0 && max_y > -kMaxDistanceY) || (min_y <= 0 && max_y >=0)) && min_x < kMaxDistanceFrontX &&
        max_x > -kMaxDistanceBackX) {
      points_.emplace_back(std::move(slot_point));
    }
  }
  return true;
}
}  // namespace planning
