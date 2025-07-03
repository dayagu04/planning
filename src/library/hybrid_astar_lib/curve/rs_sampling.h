#pragma once
#include "curve_sampling_base.h"
#include "reeds_shepp_interface.h"
#include "rs_path_interpolate.h"

namespace planning {

class RSSampling : public CurveSampling {
 public:
  RSSampling() = default;

  explicit RSSampling(const MapBound* XYbounds,
                      const ParkObstacleList* obstacles,
                      const AstarRequest* request, EulerDistanceTransform* edt,
                      const ObstacleClearZone* clear_zone,
                      ParkReferenceLine* ref_line,
                      const PlannerOpenSpaceConfig* config,
                      const float min_radius,
                      std::shared_ptr<NodeCollisionDetect> collision_detect);

  ~RSSampling() = default;

  // use rs path sampling to link start point and end point.
  bool PlanByRSPathSampling(
      HybridAStarResult* result, const Pose2f& start, const Pose2f& end,
      const float lon_min_sampling_length);

  bool SamplingByRSPath(Node3d* current_node, Node3d* polynomial_node);

  const RSPath& GetConstRsPath() const { return rs_path_; }

 private:
  void RSPathCandidateByRadius(HybridAStarResult* result, const Pose2f& start,
                               const Pose2f& end,
                               const float lon_min_sampling_length,
                               const float radius);

  const bool IsExpectedGearForRsPath(const RSPath& path);

  // copy path from rs path
  void PathTransformByRSPath(const RSPath& rs_path, HybridAStarResult* result);

 private:
  RSPathInterface rs_path_interface_;
  RSPath rs_path_;
};
}  // namespace planning