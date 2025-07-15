#pragma once
#include "base_collision_detector.h"

namespace planning {
namespace apa_planner {

class GeometryCollisionDetector final : public BaseCollisionDetector {
 public:
  GeometryCollisionDetector() {}
  GeometryCollisionDetector(
      const std::shared_ptr<ApaObstacleManager> &obs_manager_ptr) {
    SetObsManager(obs_manager_ptr);
  }
  ~GeometryCollisionDetector(){};

  const ColResult Update(const geometry_lib::PathSegment &path_seg,
                         const double body_lat_buffer, const double lon_buffer,
                         const bool special_process_mirror = false,
                         const double mirror_lat_buffer = 0.08);

  void Reset();

 private:
  void Update(const geometry_lib::LineSegment &line_seg);
  void Update(const geometry_lib::Arc &arc_seg);
  void CalPathSegBound(const geometry_lib::PathSegment &path_seg);
};

}  // namespace apa_planner
}  // namespace planning