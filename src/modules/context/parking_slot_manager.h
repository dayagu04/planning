#pragma once

#include <cstddef>
#include <vector>

#include "ehr.pb.h"
#include "fusion_parking_slot_c.h"
#include "math/line_segment2d.h"
#include "reference_path.h"
#include "session.h"
#include "vec2d.h"

namespace planning {
using ParkingSlotPoints = std::vector<planning_math::Vec2d>;
using ParkingLimiters = std::vector<planning_math::LineSegment2d>;

// todo: delete it
class ParkingSlotManager {
 public:
  ParkingSlotManager(planning::framework::Session *session);
  ~ParkingSlotManager() = default;

  bool Update(const Map::StaticMap &static_map);

  bool Update(const iflyauto::ParkingFusionInfo &parking_fusion_info);

  bool CalculateDistanceToTargetSlot(
      const std::shared_ptr<ReferencePath> &reference_path);

  const std::vector<ParkingSlotPoints> &GetPoints() const { return points_; };

  const bool IsExistTargetSlot() const { return is_exist_target_slot_; }

  const size_t GetTargetSlotId() const { return target_slot_id_; }

  const double GetDistanceToTargetSlot() const {
    return distance_to_target_slot_;
  }

  const ParkingSlotPoints &GetTargetSlotPoints() const {
    return target_slot_;
  }

  const planning_math::Polygon2d &GetTargetSlotPolygon() const {
    return target_slot_polygon_;
  }

  const bool IsReachedTargetSlot() const { return is_reached_target_slot_; }

 private:
  void Init();

  planning::framework::Session *session_ = nullptr;
  bool is_reached_target_slot_;
  bool is_exist_target_slot_;
  size_t target_slot_id_;
  double distance_to_target_slot_;
  ParkingSlotPoints target_slot_;
  planning_math::Polygon2d target_slot_polygon_;
  std::vector<ParkingSlotPoints> points_;
  ParkingLimiters limiters_;
};
}  // namespace planning