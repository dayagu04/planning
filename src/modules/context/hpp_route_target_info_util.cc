#include "hpp_route_target_info_util.h"

#include "define/geometry.h"
#include "environmental_model.h"
#include "parking_slot_manager.h"
#include "reference_path.h"
#include "route_info.h"
#include "session.h"

namespace planning {

bool UpdateHppRouteTargetInfoFromReferencePath(
    framework::Session* session,
    const std::shared_ptr<ReferencePath>& reference_path) {
  if (session == nullptr) {
    return false;
  }

  const auto& parking_slot_manager =
      session->environmental_model().get_parking_slot_manager();
  if (reference_path == nullptr || !reference_path->valid() ||
      !parking_slot_manager->IsExistTargetSlot()) {
    return true;
  }

  const auto& route_info_output =
      session->environmental_model().get_route_info()->get_route_info_output();
  double dist_to_target_dest =
      route_info_output.hpp_route_info_output.distance_to_target_dest;
  double dist_to_target_slot =
      route_info_output.hpp_route_info_output.distance_to_target_slot;

  const double ego_s = reference_path->get_frenet_ego_state().s();
  const auto& frenet_coord = reference_path->get_frenet_coord();

  if (frenet_coord) {
    const auto& target_slot_center =
        parking_slot_manager->GetTargetSlotCenter();
    Point2D target_slot_frenet_pt{0.0, 0.0};
    if (frenet_coord->XYToSL(
            Point2D(target_slot_center.x(), target_slot_center.y()),
            target_slot_frenet_pt)) {
      dist_to_target_slot = target_slot_frenet_pt.x - ego_s;
    }

    const auto& target_dest_point =
        route_info_output.hpp_route_info_output.target_dest_point;
    Point2D target_dest_frenet_pt{0.0, 0.0};
    if (frenet_coord->XYToSL(
            Point2D(target_dest_point.x(), target_dest_point.y()),
            target_dest_frenet_pt)) {
      dist_to_target_dest = target_dest_frenet_pt.x - ego_s;
    }
  }

  session->mutable_environmental_model()->get_route_info()->UpdateTargetInfo(
      dist_to_target_slot, dist_to_target_dest);
  return true;
}

}  // namespace planning
