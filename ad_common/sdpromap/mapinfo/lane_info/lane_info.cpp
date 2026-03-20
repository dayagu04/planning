#include "lane_info.h"

namespace ad_common {
namespace sdpromap {
void LaneInfo::Init(const iflymapdata::sdpro::LaneInfo& lane_info) {
  lanes_table_.clear();
  if (lane_info.lanes_size() > 0) {
    lanes_table_.reserve(lane_info.lanes().size());
    for (const auto& lane : lane_info.lanes()) {
      lanes_table_[lane.id()] = std::make_shared<Lane>(lane);
    }
  }
}

void LaneInfo::BuildSegmentKdTree(
    const ad_common::math::AABoxKDTreeParams& params) {
  /*segment_box_list_.clear();
  for (const auto& link_info : lanes_table_) {
      const auto* info = link_info.second.get();
      for (size_t id = 0; id < info->segments().size(); ++id) {
          const auto& segment = info->segments()[id];
          segment_box_list_.emplace_back(
          ad_common::math::AABox2d{segment.start(),
                                  segment.end()},
          info, segment, id);
      }
  }
  lane_segment_kdtree_.reset(new Lane::LaneSegmentKDTree(segment_box_list_,
  params));*/
  return;
}

std::vector<std::pair<uint64_t, uint64_t>> LaneInfo::GetObjects(
    const Vec2d& center, double& radius) {
  std::vector<std::pair<uint64_t, uint64_t>> result_ids;
  /*
  auto objects = lane_segment_kdtree_->GetObjects(center, radius);
  result_ids.reserve(objects.size());
  for (const auto *obj : objects) {
      result_ids.push_back(
          std::make_pair(obj->object()->getLane().id(), obj->id()));
  }*/
  return result_ids;
}

Lane::Ptr LaneInfo::getLaneById(const uint64_t& id) const {
  if (!lanes_table_.count(id)) {
    return nullptr;
  }
  return lanes_table_.at(id);
}

}  // namespace sdpromap
}  // namespace ad_common
