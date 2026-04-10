#include "hybrid_astar_perpendicular_path_generator.h"
#include "log_glog.h"
namespace planning {
namespace apa_planner {
void HybridAStarPerpendicularPathGenerator::UpdatePoseBoundary() {
  const float bound = request_.ego_info_under_slot.slot_type == SlotType::SLANT
                          ? SLANT_SLOT_EXTEND_BOUND
                          : 0.0f;
  search_map_boundary_ = MapBound(-2.0f - bound, 20.0f + bound, -20.0f - bound,
                                  20.0f + bound, -M_PIf32, M_PIf32);

  search_map_grid_boundary_.x =
      std::ceil((search_map_boundary_.x_max - search_map_boundary_.x_min) *
                config_.xy_grid_resolution_inv);
  search_map_grid_boundary_.y =
      std::ceil((search_map_boundary_.y_max - search_map_boundary_.y_min) *
                config_.xy_grid_resolution_inv);
  search_map_grid_boundary_.phi =
      std::ceil((search_map_boundary_.phi_max - search_map_boundary_.phi_min) *
                config_.phi_grid_resolution_inv) +
      1;

  ILOG_INFO << "search map boundary, search_map_grid_boundary";
  search_map_boundary_.PrintInfo();
  search_map_grid_boundary_.PrintInfo();
}
}  // namespace apa_planner
}  // namespace planning