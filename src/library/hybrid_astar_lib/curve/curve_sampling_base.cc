#include "curve_sampling_base.h"

namespace planning {

CurveSampling::CurveSampling(
    const MapBound* XYbounds, const ParkObstacleList* obstacles,
    const AstarRequest* request, EulerDistanceTransform* edt,
    ParkReferenceLine* ref_line, const PlannerOpenSpaceConfig* config,
    const float min_radius,
    std::shared_ptr<NodeCollisionDetect> collision_detect)
    : obstacles_(obstacles),
      edt_(edt),
      ref_line_(ref_line),
      grid_map_bound_(XYbounds),
      request_(request),
      config_(config),
      min_radius_(min_radius),
      collision_detect_(collision_detect) {}

}  // namespace planning