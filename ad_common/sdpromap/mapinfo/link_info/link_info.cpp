#include "link_info.h"

namespace ad_common {
namespace sdpromap {
void LinkInfo::Init(const iflymapdata::sdpro::LinkInfo& link_info,
                    const std::unordered_set<uint64_t>& route_link_ids_set) {
  links_table_.clear();
  total_segment_size_ = 0;
  route_link_ids_set_ = route_link_ids_set;
  if (link_info.links_size() > 0) {
    links_table_.reserve(link_info.links().size());
    for (const auto& link : link_info.links()) {
      if (link.has_points()) {
        const auto points = link.points();
        if (points.has_boot()) {
          links_table_[link.id()] = std::make_shared<Link>(link);
          total_segment_size_ += links_table_[link.id()]->segments().size();
        }
      }
    }
  }
}

void LinkInfo::BuildSegmentKdTree(
    const ad_common::math::AABoxKDTreeParams& params) {
  segment_box_list_.clear();
  segment_box_list_.reserve(total_segment_size_);
  for (const auto& link_info : links_table_) {
    const auto* info = link_info.second.get();
    if (!route_link_ids_set_.empty() &&
        route_link_ids_set_.find(info->getLink().id()) ==
            route_link_ids_set_.end()) {
      continue;
    }
    for (size_t id = 0; id < info->segments().size(); ++id) {
      const auto& segment = info->segments()[id];
      segment_box_list_.emplace_back(
          ad_common::math::AABox2d{segment.start(), segment.end()}, info,
          segment, id);
    }
  }
  link_segment_kdtree_ =
      std::make_unique<Link::LinkSegmentKDTree>(segment_box_list_, params);
  route_link_ids_set_.clear();
  return;
}

std::vector<std::pair<uint64_t, uint64_t>> LinkInfo::GetObjects(
    const Vec2d& center, double& radius) {
  std::vector<std::pair<uint64_t, uint64_t>> result_ids;
  auto objects = link_segment_kdtree_->GetObjects(center, radius);
  result_ids.reserve(objects.size());
  for (const auto* obj : objects) {
    result_ids.push_back(
        std::make_pair(obj->object()->getLink().id(), obj->id()));
  }
  return result_ids;
}

const Link::LinkSegmentBox* LinkInfo::getNearestObject(
    const Vec2d& center) const {
  const auto* obj = link_segment_kdtree_->GetNearestObject(center);
  if (!obj) {
    return nullptr;
  }
  return obj;
}

Link::Ptr LinkInfo::getLinkById(const uint64_t& id) const {
  if (!links_table_.count(id)) {
    return nullptr;
  }
  return links_table_.at(id);
}

}  // namespace sdpromap
}  // namespace ad_common
