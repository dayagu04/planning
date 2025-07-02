#pragma once

#include "cubic_polynomial_path.h"
#include "curve_sampling_base.h"
#include "src/modules/common/math/curve1d/quintic_polynomial_curve1d.h"

namespace planning {

class PolynomialCurveSampling : public CurveSampling {
 public:
  PolynomialCurveSampling() = default;

  explicit PolynomialCurveSampling(
      const MapBound* XYbounds, const ParkObstacleList* obstacles,
      const AstarRequest* request, EulerDistanceTransform* edt,
      const ObstacleClearZone* clear_zone, ParkReferenceLine* ref_line,
      const PlannerOpenSpaceConfig* config, const float min_radius,
      std::shared_ptr<NodeCollisionDetect> collision_detect);

  ~PolynomialCurveSampling() = default;

  // use cubic path sampling to link start point and end point.
  bool SamplingByCubicPolyForVerticalSlot(HybridAStarResult* result,
                                          const Pose2f& start,
                                          const Pose2f& target,
                                          const float lon_min_sampling_length);

  // use cubic path sampling to link start point and end point.
  bool SamplingByCubicPolyForParallelSlot(HybridAStarResult* result,
                                          const Pose2f& start,
                                          const Pose2f& end,
                                          const float lon_min_sampling_length);

  bool SamplingByQunticPolynomial(Node3d* current_node,
                                  std::vector<AStarPathPoint>& path,
                                  Node3d* polynomial_node,
                                  PolynomialPathErrorCode* fail_type);

 private:
  void GetQunticPolynomialPath(std::vector<AStarPathPoint>& path,
                               const Pose2f& start, const float start_kappa,
                               const Pose2f& end);

 private:
  CubicPathInterface cubic_polynomial_path_;
};
}  // namespace planning