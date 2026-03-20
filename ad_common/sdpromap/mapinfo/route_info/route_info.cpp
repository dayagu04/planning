#include "route_info.h"

namespace ad_common {
namespace sdpromap {
void RouteInfo::Init(const iflymapdata::sdpro::RouteInfo& route_info) {
  route_link_ids_.clear();
  link_id_index_pair_.clear();
  if (route_info.route_links_size() > 0) {
    route_link_ids_.reserve(route_info.route_links_size());
    link_id_index_pair_.reserve(route_info.route_links_size());
    for (const auto& route_link : route_info.route_links()) {
      link_id_index_pair_[route_link.link_id()].push_back(
          static_cast<int>(route_link_ids_.size()));
      route_link_ids_.push_back(route_link.link_id());
    }
  }
}

void RouteInfo::BuildSegmentKdTree(
    const ad_common::math::AABoxKDTreeParams& params) {
  return;
}

std::vector<std::pair<uint64_t, uint64_t>> RouteInfo::GetObjects(
    const Vec2d& center, double& radius) {
  std::vector<std::pair<uint64_t, uint64_t>> result_ids;
  return result_ids;
}

std::vector<uint64_t> RouteInfo::getRemainRouteLinkIds(
    const uint64_t& target_link_id) const {
  std::vector<uint64_t> result;
  auto it =
      std::find(route_link_ids_.begin(), route_link_ids_.end(), target_link_id);
  if (it != route_link_ids_.end()) {
    result.assign(it + 1, route_link_ids_.end());
  }
  return result;
}

bool RouteInfo::getPreviousLinkId(const uint64_t& target_link_id,
                                  uint64_t& pre_id) const {
  auto it =
      std::find(route_link_ids_.begin(), route_link_ids_.end(), target_link_id);
  if (it != route_link_ids_.end()) {
    if (it != route_link_ids_.begin()) {
      pre_id = *(it - 1);
      return true;
    }
  }
  return false;
}

bool RouteInfo::getNextLinkId(const uint64_t& target_link_id,
                              std::vector<uint64_t>& next_ids) const {
  next_ids.clear();
  auto it = link_id_index_pair_.find(target_link_id);
  if (it == link_id_index_pair_.end()) {
    return false;
  }

  for (const auto& idx : it->second) {
    if (idx + 1 < static_cast<int>(route_link_ids_.size())) {
      next_ids.push_back(route_link_ids_[idx + 1]);
    }
  }
  return !next_ids.empty();
}

bool RouteInfo::getNextLinkIdByCurIndex(const uint64_t& target_link_id,
                                        const int& cur_index, uint64_t& next_id,
                                        int* next_index) const {
  auto it = link_id_index_pair_.find(target_link_id);
  if (it == link_id_index_pair_.end()) {
    return false;
  }

  int min_index_dis = std::numeric_limits<int>::max();
  bool is_find = false;
  for (const auto& idx : it->second) {
    if (idx + 1 < static_cast<int>(route_link_ids_.size())) {
      if (idx - cur_index >= 0 && idx - cur_index < min_index_dis) {
        min_index_dis = idx - cur_index;
        next_id = route_link_ids_[idx + 1];
        if (next_index) {
          *next_index = idx + 1;
        }
        is_find = true;
      }
    }
  }
  return is_find;
}

bool RouteInfo::isOnRouteLinks(const uint64_t& target_link_id) const {
  auto it =
      std::find(route_link_ids_.begin(), route_link_ids_.end(), target_link_id);

  return it != route_link_ids_.end();
}

}  // namespace sdpromap
}  // namespace ad_common
