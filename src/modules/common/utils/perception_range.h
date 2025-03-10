#ifndef MSQUARE_PNC_PERCEPTION_RANGE_HPP
#define MSQUARE_PNC_PERCEPTION_RANGE_HPP
#include <memory>

// #include "context/ego_state_manager.h"
// #include "map_info_manager.h"
#include "tracked_object.h"
#include "transform.h"
#include "utils/frenet_coordinate_system.h"

namespace planning {

class PerceptionRangeEstimator {
 public:
  virtual ~PerceptionRangeEstimator() = default;
  // Update frenet coordinate transformer
  virtual void updateFrenet(
      const std::shared_ptr<FrenetCoordinateSystem> &frenet) = 0;

  // Feed ego state to estimator
  // virtual void feedEgoState(const EgoStateManager &ego_state, const Transform
  // &car2enu) = 0;

  // Feed map info to estimator

  // virtual void Feed_IflytekEhrStaticMap(const MSDMapInfo &map_info) = 0;

  // Calculate occlusion distance by an obstacle
  // virtual double calculate(const TrackedObject &obstacle) const = 0;

  // Factory method
  static std::shared_ptr<PerceptionRangeEstimator> make();
};

}  // namespace planning

#endif  // MSQUARE_PNC_PERCEPTION_RANGE_HPP
