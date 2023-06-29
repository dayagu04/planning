#include "reference_path_manager.h"

#include "config/basic_type.h"
#include "environmental_model.h"
#include "lane_reference_path.h"
#include "session.h"
#include "virtual_lane_manager.h"

namespace planning {

ReferencePathManager::ReferencePathManager(
    planning::framework::Session *session) {
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

bool ReferencePathManager::update() {
  reference_paths_.clear();
  auto &virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  // step1 construct current/left/right reference_path
  auto &current_lane = virtual_lane_manager->get_current_lane();
  auto &left_lane = virtual_lane_manager->get_left_lane();
  auto &right_lane = virtual_lane_manager->get_right_lane();
  assert(current_lane != nullptr);
  auto lane_virtual_id = current_lane->get_virtual_id();
  if (!get_reference_path_by_lane(lane_virtual_id, true)) {
    LOG_DEBUG("--------- for current_lane: update %d\n", lane_virtual_id);
    return false;
  }
  LOG_DEBUG("--------- for lane_virtual_id: update %d\n", lane_virtual_id);
  if (left_lane != nullptr) {
    lane_virtual_id = left_lane->get_virtual_id();
    get_reference_path_by_lane(lane_virtual_id, true);
    LOG_DEBUG("--------- for left_lane: update %d\n", lane_virtual_id);
  }
  if (right_lane != nullptr) {
    lane_virtual_id = right_lane->get_virtual_id();
    get_reference_path_by_lane(lane_virtual_id, true);
    LOG_DEBUG("--------- for right_lane: update %d\n", lane_virtual_id);
  }

  // step2 check reference_paths_'s history, and update data
  for (auto it = reference_paths_.begin(); it != reference_paths_.end();) {
    lane_virtual_id = it->first.second;
    if (virtual_lane_manager->has_lane(lane_virtual_id)) {
      // current_lane, left_lane, right_lane has called update()
      if (current_lane->get_virtual_id() == lane_virtual_id ||
          (left_lane != nullptr &&
           left_lane->get_virtual_id() == lane_virtual_id) ||
          (right_lane != nullptr &&
           right_lane->get_virtual_id() == lane_virtual_id)) {
        ++it;
        continue;
      }
      it->second->update(session_);
      LOG_DEBUG("--------- for lane_virtual_id: update %d\n", lane_virtual_id);
      ++it;
    } else {
      LOG_DEBUG("--------- for lane_virtual_id: delete %d\n", lane_virtual_id);
      it = reference_paths_.erase(it);
    }
  }
  return true;
}

std::shared_ptr<ReferencePath>
ReferencePathManager::make_map_lane_reference_path(int lane_virtual_id) {
  return get_reference_path_by_lane(lane_virtual_id, true);
}

}  // namespace planning
