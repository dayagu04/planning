#pragma once

#include "../tail_in/hybrid_astar_perpendicular_tail_in_path_generator.h"

namespace planning {
namespace apa_planner {

class HybridAStarPerpendicularHeadingInPathGenerator
    : public HybridAStarPerpendicularTailInPathGenerator {
 public:
  HybridAStarPerpendicularHeadingInPathGenerator() = default;
  HybridAStarPerpendicularHeadingInPathGenerator(
      const std::shared_ptr<CollisionDetectorInterface>&
          col_det_interface_ptr) {
    col_det_interface_ptr_ = col_det_interface_ptr;
  }
  ~HybridAStarPerpendicularHeadingInPathGenerator() = default;
};
}  // namespace apa_planner
}  // namespace planning