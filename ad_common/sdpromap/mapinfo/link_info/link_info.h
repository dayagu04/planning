#include "../sdpromap_utils.hpp"

namespace ad_common {
namespace sdpromap {

class Link {
 public:
  using Ptr = std::shared_ptr<const Link>;
  using LinkSegmentBox = ObjectWithAABox<Link, ad_common::math::LineSegment2d>;
  using LinkSegmentKDTree = ad_common::math::AABoxKDTree2d<LinkSegmentBox>;
  Link(const iflymapdata::sdpro::LinkInfo_Link& link) : link_(link) {
    const auto points = link.points();
    segments_.clear();
    accumulated_s_.clear();
    headings_.clear();
    successor_link_ids_.clear();
    segments_.reserve(points.boot().points_size());
    accumulated_s_.reserve(points.boot().points_size());
    headings_.reserve(points.boot().points_size());
    successor_link_ids_.reserve(link.successor_link_ids_size());
    double s = 0.0;
    for (int i = 1; i < points.boot().points_size(); ++i) {
      accumulated_s_.push_back(s);
      segments_.emplace_back(
          Vec2d{points.boot().points(i - 1).x(),
                points.boot().points(i - 1).y()},
          Vec2d{points.boot().points(i).x(), points.boot().points(i).y()});
      s += segments_.back().length();
    }
    if (!accumulated_s_.empty()) {
      accumulated_s_.push_back(s);
    }
    for (const auto& line_seg : segments_) {
      headings_.push_back(line_seg.unit_direction().Angle());
    }
    for (const auto& successor_link_id : link.successor_link_ids()) {
      successor_link_ids_.insert(successor_link_id);
    }
  }
  const iflymapdata::sdpro::LinkInfo_Link& getLink() const { return link_; }
  const std::vector<ad_common::math::LineSegment2d>& segments() const {
    return segments_;
  }
  const std::vector<double>& AccumulatedS() const { return accumulated_s_; }
  const std::vector<double>& Headings() const { return headings_; }
  const std::unordered_set<uint64_t>& SuccessorLinkIds() const {
    return successor_link_ids_;
  }

 private:
  iflymapdata::sdpro::LinkInfo_Link link_;
  std::vector<ad_common::math::LineSegment2d> segments_;
  std::vector<double> headings_;       // 形点线段的角度值
  std::vector<double> accumulated_s_;  // 道路起点到各个形点的累积长度
  std::unordered_set<uint64_t> successor_link_ids_;  // 后继link_id
};

class LinkInfo final : public MapInfoBase {
 public:
  using Ptr = std::shared_ptr<const LinkInfo>;
  LinkInfo() {}
  LinkInfo(const iflymapdata::sdpro::LinkInfo& link_info,
           const std::unordered_set<uint64_t>& route_link_ids_set)
      : link_info_(link_info), route_link_ids_set_(route_link_ids_set) {}
  virtual void BuildSegmentKdTree(
      const ad_common::math::AABoxKDTreeParams& params) override;
  virtual std::vector<std::pair<uint64_t, uint64_t>> GetObjects(
      const Vec2d& center, double& radius) override;
  void Init(const iflymapdata::sdpro::LinkInfo& link_info,
            const std::unordered_set<uint64_t>& route_link_ids_set);
  Link::Ptr getLinkById(const uint64_t& id) const;
  const Link::LinkSegmentBox* getNearestObject(const Vec2d& center) const;
  const uint64_t getTotalSegmentSize() const { return total_segment_size_; }

 public:
  const iflymapdata::sdpro::LinkInfo& getLinkInfo() const { return link_info_; }
  const std::unique_ptr<Link::LinkSegmentKDTree>& getKdtreePtr() const {
    return link_segment_kdtree_;
  }

 private:
  iflymapdata::sdpro::LinkInfo link_info_;
  std::vector<Link::LinkSegmentBox> segment_box_list_;
  std::unique_ptr<Link::LinkSegmentKDTree> link_segment_kdtree_;
  std::unordered_map<uint64_t, Link::Ptr> links_table_;
  std::unordered_set<uint64_t> route_link_ids_set_;
  uint64_t total_segment_size_ = 0;
};
}  // namespace sdpromap
}  // namespace ad_common
