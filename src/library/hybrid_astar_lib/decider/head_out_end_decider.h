#pragma once
#include <string>

#include "hybrid_astar_request.h"
#include "node3d.h"
#include "node_collision_detect.h"
#include "pose2d.h"

namespace planning {

class HeadOutEndDecider {
 public:
  HeadOutEndDecider() = default;

  virtual ~HeadOutEndDecider() = default;

  const bool Process(
      const MapBound &XYbounds, const PlannerOpenSpaceConfig &config,
      const std::shared_ptr<NodeCollisionDetect> &collision_detect,
      const AstarRequest request, Pose2D &end, Node3d *astar_end_node,
      HybridAStarResult *result);
};

}  // namespace planning