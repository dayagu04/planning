#include "apa_planner/apa_planner_dispatcher.h"

#include "apa_planner/common/planning_log_helper.h"
#include "apa_planner/diagonal/diagonal_in_planner.h"
#include "apa_planner/parallel/parallel_in_planner.h"
#include "context/environmental_model.h"

namespace planning {
namespace apa_planner {

using framework::Frame;

ApaPlannerDispatcher::ApaPlannerDispatcher() {
  RegisterPlanners();
}

void ApaPlannerDispatcher::RegisterPlanners() {
  planner_list_.clear();
  planner_list_.emplace_back(std::make_unique<DiagonalInPlanner>());
  planner_list_.emplace_back(std::make_unique<ParallelInPlanner>());
}

bool ApaPlannerDispatcher::Update(Frame* const frame) {
  const auto& parking_fusion = frame->session()->environmental_model().\
      get_local_view().parking_fusion_info;
  if (!parking_fusion.has_select_slot_id()) {
    PLANNING_LOG << "no select_slot_id" << std::endl;
    return false;
  }

  if (parking_fusion.parking_fusion_slot_lists_size() == 0) {
    PLANNING_LOG << "parking_fusion_slot_lists size is 0" << std::endl;
    return false;
  }

  bool is_select_slot_id_valid = false;
  const size_t selected_slot_id = parking_fusion.select_slot_id();
  PLANNING_LOG << "selected_slot_id:" << selected_slot_id << std::endl;
  const auto& slots = parking_fusion.parking_fusion_slot_lists();
  for (int i = 0; i < parking_fusion.parking_fusion_slot_lists_size(); ++i) {
    if (selected_slot_id ==
        parking_fusion.parking_fusion_slot_lists()[i].id()) {
      is_select_slot_id_valid = true;
      PLANNING_LOG << "selected slot type:" << slots[i].type() << std::endl;
      break;
    }
  }
  if (!is_select_slot_id_valid) {
    PLANNING_LOG << "selected_slot_id is invalid" << std::endl;
    return false;
  }

  bool is_planning_ok = false;
  for (auto& planner : planner_list_) {
    if (planner->Update(frame)) {
      is_planning_ok = true;
      break;
    }
  }
  return is_planning_ok;
}

} // namespace apa_planner
} // namespace planning