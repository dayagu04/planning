#pragma once

#include <vector>

#include "ifly_parking_map_c.h"
#include "session.h"
#include "vec2d.h"

namespace planning {
using ParkingSlotPoints = std::vector<planning_math::Vec2d>;

// todo: delete it
class ParkingSlotManager {
 public:
  ParkingSlotManager(planning::framework::Session *session);
  ~ParkingSlotManager() = default;

 public:
  bool update(const iflyauto::ParkingInfo &parking_info);
  std::vector<ParkingSlotPoints> get_points() { return points_; };

 private:
  planning::framework::Session *session_;
  std::vector<ParkingSlotPoints> points_;
};
}  // namespace planning