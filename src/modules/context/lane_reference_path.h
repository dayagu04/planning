#pragma once
#include "modules/context/reference_path.h"

namespace planning {

class LaneReferencePath : public ReferencePath {
 public:
  LaneReferencePath(int target_lane_virtual_id);
  virtual ~LaneReferencePath() = default;

  int get_lane_id() { return target_lane_virtual_id_; }

  virtual void update(planning::framework::Session *session);

  virtual bool is_obstacle_ignorable(const FrenetObstacle &obstacle) override;

 private:
  virtual void update_obstacles();
  bool get_points_by_lane_id(int target_lane_virtual_id, ReferencePathPoints &points);

  int target_lane_virtual_id_ = 0;
};

}  // namespace planning