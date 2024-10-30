#ifndef OCCUPANCY_OBJECT_MANAGER_H_
#define OCCUPANCY_OBJECT_MANAGER_H_

#include <vector>

#include "../../src/library/occupancy_grid_map/euler_distance_transform.h"
#include "../../src/library/occupancy_grid_map/occupancy_grid_map.h"
#include "fusion_occupancy_objects_c.h"
#include "session.h"
#include "vec2d.h"

namespace planning {
using OccupancyObjectPoints = std::vector<planning_math::Vec2d>;

class OccupancyObjectManager {
 public:
  OccupancyObjectManager(planning::framework::Session *session);
  ~OccupancyObjectManager() = default;

  bool Update(const iflyauto::FusionOccupancyObjectsInfo &occupancy_objects_info);

  const std::vector<OccupancyObjectPoints>& GetPoints() const  {
    return points_;
  }

  const EulerDistanceTransform* GetEulerDistanceTransform() const {
    return &edt_;
  }

  const bool GetIsEulerDistanceTransformValid() const {
    return is_edt_valid_;
  }

 private:
  bool Init();

  bool UpdateEDT(const std::vector<PointCloudObstacle>& point_clouds);

  planning::framework::Session *session_;
  std::vector<OccupancyObjectPoints> points_;

  OccupancyGridMap ogm_;
  EulerDistanceTransform edt_;

  bool is_edt_valid_;
};
}  // namespace planning
#endif