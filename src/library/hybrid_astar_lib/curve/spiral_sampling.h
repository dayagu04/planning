#pragma once

#include "curve_sampling_base.h"
#include "src/library/spiral/cubic_spiral_interface.h"
#include "src/library/spiral/spiral_path.h"

namespace planning {

// cubic spiral
class SpiralSampling : public CurveSampling {
 public:
  SpiralSampling() = default;

  explicit SpiralSampling(
      const MapBound* XYbounds, const ParkObstacleList* obstacles,
      const AstarRequest* request, HierarchyEulerDistanceTransform* hierarchy_edt,
      ParkReferenceLine* ref_line, const PlannerOpenSpaceConfig* config,
      const float min_radius,
      std::shared_ptr<NodeCollisionDetect> collision_detect);

  ~SpiralSampling() = default;

  // use cubic spiral path sampling to link start point and end point.
  bool SamplingByCubicSpiralForVerticalSlot(
      HybridAStarResult* result, const Pose2f& start, const Pose2f& target,
      const float lon_min_sampling_length);

  bool CallGetCubicSpiralPath(std::vector<AStarPathPoint>& path,
                              const Pose2f& start, const Pose2f& end,
                              AstarPathGear ref_gear) {
    return GetCubicSpiralPath(path, start, end, ref_gear);
  }

 private:
  const bool GetCubicSpiralPath(std::vector<AStarPathPoint>& path,
                                const Pose2f& start, const Pose2f& end,
                                const AstarPathGear ref_gear);
};

}  // namespace planning