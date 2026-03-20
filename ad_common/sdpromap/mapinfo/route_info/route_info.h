#include "../sdpromap_utils.hpp"

namespace ad_common {
namespace sdpromap {

class RouteInfo final : public MapInfoBase {
 public:
  using Ptr = std::shared_ptr<const RouteInfo>;
  RouteInfo() {}
  RouteInfo(const iflymapdata::sdpro::RouteInfo& route_info)
      : route_info_(route_info) {}
  virtual void BuildSegmentKdTree(
      const ad_common::math::AABoxKDTreeParams& params) override;
  virtual std::vector<std::pair<uint64_t, uint64_t>> GetObjects(
      const Vec2d& center, double& radius) override;
  void Init(const iflymapdata::sdpro::RouteInfo& route_info);
  std::vector<uint64_t> getRouteLinkIds() const { return route_link_ids_; }
  bool getRouteLinkIndex(const uint64_t& link_id,
                         std::vector<int>& link_indexs) const {
    if (link_id_index_pair_.empty()) {
      return false;
    }
    if (link_id_index_pair_.find(link_id) == link_id_index_pair_.end()) {
      return false;
    }
    link_indexs = link_id_index_pair_.at(link_id);
    return true;
  }
  std::vector<uint64_t> getRemainRouteLinkIds(
      const uint64_t& target_link_id) const;
  bool getPreviousLinkId(const uint64_t& target_link_id,
                         uint64_t& pre_id) const;
  bool getNextLinkId(const uint64_t& target_link_id,
                     std::vector<uint64_t>& next_ids) const;
  bool getNextLinkIdByCurIndex(const uint64_t& target_link_id,
                               const int& cur_index, uint64_t& next_id,
                               int* next_index = nullptr) const;
  bool isOnRouteLinks(const uint64_t& target_link_id) const;

 public:
  const iflymapdata::sdpro::RouteInfo& getRouteInfo() const {
    return route_info_;
  }

 private:
  iflymapdata::sdpro::RouteInfo route_info_;
  std::vector<uint64_t> route_link_ids_;
  std::unordered_map<uint64_t, std::vector<int>> link_id_index_pair_;
};
}  // namespace sdpromap
}  // namespace ad_common
