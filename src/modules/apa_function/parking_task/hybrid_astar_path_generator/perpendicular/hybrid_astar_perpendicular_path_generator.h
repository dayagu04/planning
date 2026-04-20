#pragma once

#include "hybrid_astar_path_generator.h"
namespace planning {
namespace apa_planner {
class HybridAStarPerpendicularPathGenerator : public HybridAStarPathGenerator {
 public:
 protected:
  virtual void UpdatePoseBoundary() override;
};
}  // namespace apa_planner
}  // namespace planning