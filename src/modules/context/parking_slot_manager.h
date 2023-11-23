#ifndef ZNQC_MODULES_CONTEXT_PARKING_SLOT_MANAGER_H_
#define ZNQC_MODULES_CONTEXT_PARKING_SLOT_MANAGER_H_

#include <vector>
#include "ifly_parking_map.pb.h"
#include "session.h"
#include "vec2d.h"

namespace planning {
using ParkingSlotPoints = std::vector<planning_math::Vec2d>;

class ParkingSlotManager {
 public:
  ParkingSlotManager(planning::framework::Session *session);
  ~ParkingSlotManager() = default;

 public:
  bool update(const IFLYParkingMap::ParkingInfo& parking_info);
  std::vector<ParkingSlotPoints> get_points() { return points_; };

 private:
  planning::framework::Session *session_;
  std::vector<ParkingSlotPoints> points_;
};
}  // namespace planning
#endif