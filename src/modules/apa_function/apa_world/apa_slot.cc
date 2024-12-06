#include "apa_slot.h"

namespace planning {

ApaSlot::ApaSlot() {}

void ApaSlot::Init() {
  slot_type = SlotType::INVALID;
  slot_side = SlotSide::INVALID;
  perception_id_ = 0;
  height_top_ = 0;
  height_bottom_ = 0;
  perception_source_type_ = iflyauto::SlotSourceType::SLOT_SOURCE_TYPE_INVALID;

  is_perception_release_ = false;
  slot_material_type_ = SlotMaterialType::UNKOWN;

  heading_ = 0.0;
  id_ = 0;
  is_planning_release_ = false;
  is_suggested_slot_ = false;

  ego_occupied_ratio_ = 0;

  release_info_.Clear();
  obs_info_.Clear();

  return;
}
}  // namespace planning