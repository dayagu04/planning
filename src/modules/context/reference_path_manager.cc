#include "modules/context/reference_path_manager.h"

#include "framework/session.h"
#include "modules/common/config/basic_type.h"
#include "modules/context/lane_reference_path.h"
#include "modules/context/virtual_lane_manager.h"

namespace planning {

ReferencePathManager::ReferencePathManager(planning::framework::Session* session) {
  session_ = session;
}

ReferencePathManager::~ReferencePathManager() {}

std::shared_ptr<ReferencePath> ReferencePathManager::get_reference_path_by_lane(
    int lane_virtual_id, bool create_if_not_exist) {
  auto key = ReferencePathKeyType(ReferencePathType::MAP_LANE, lane_virtual_id);
  auto it = reference_paths_.find(key);
  if (it == reference_paths_.end() and create_if_not_exist) {
    auto reference_path = std::make_shared<LaneReferencePath>(lane_virtual_id);
    reference_path->update(session_);
    if (reference_path->valid()) {
      reference_paths_[key] = reference_path;
    }
  }

  it = reference_paths_.find(key);
  return it == reference_paths_.end() ? nullptr : it->second;
}

std::shared_ptr<ReferencePath>
ReferencePathManager::get_reference_path_by_current_lane() {
  auto virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  return get_reference_path_by_lane(
      virtual_lane_manager->current_lane_virtual_id(), false);
}

void ReferencePathManager::update() {

  auto virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  //step1 construct current/left/right reference_path
  auto &current_lane = virtual_lane_manager->get_current_lane();
  auto &left_lane = virtual_lane_manager->get_left_lane();
  auto &right_lane = virtual_lane_manager->get_right_lane();
  // auto key = ReferencePathKeyType(ReferencePathType::MAP_LANE, lane_virtual_id);
  auto current_reference_path = get_reference_path_by_lane(current_lane->get_virtual_id(), true);
  auto current_lane_reference_path = dynamic_pointer_cast<LaneReferencePath>(current_reference_path);
  if (current_lane_reference_path != nullptr) {
    current_lane_reference_path->assign_obstacles_to_lanes();
  }

  if (left_lane != nullptr) {
    auto left_reference_path = get_reference_path_by_lane(left_lane->get_virtual_id(), true);
    auto left_lane_reference_path = dynamic_pointer_cast<LaneReferencePath>(left_reference_path);
    if (left_lane_reference_path != nullptr) {
      left_lane_reference_path->assign_obstacles_to_lanes();
    }
  }
  if (right_lane != nullptr) {
    auto right_reference_path = get_reference_path_by_lane(right_lane->get_virtual_id(), true);
    auto right_lane_reference_path = dynamic_pointer_cast<LaneReferencePath>(right_reference_path);
    if (right_lane_reference_path != nullptr) {
      right_lane_reference_path->assign_obstacles_to_lanes();
    }
  }

  // step2 check reference_paths_'s history, and update data
  for (auto it = reference_paths_.begin(); it != reference_paths_.end();) {
    auto lane_virtual_id = it->first.second;
    if (virtual_lane_manager->has_lane(lane_virtual_id)) {
      LOG_DEBUG("--------- for lane_virtual_id: update %d\n", lane_virtual_id);
      it->second->update(session_);
      ++it;
    } else {
      LOG_DEBUG("--------- for lane_virtual_id: delete %d\n", lane_virtual_id);
      it = reference_paths_.erase(it);
    }
  }
}
std::shared_ptr<ReferencePath> ReferencePathManager::make_map_lane_reference_path(int lane_virtual_id) {
  return get_reference_path_by_lane(lane_virtual_id, true);
}

}  // namespace planning
