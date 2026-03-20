#include "../sdpromap_utils.hpp"

namespace ad_common {
namespace sdpromap {

class Lane {
 public:
  using Ptr = std::shared_ptr<const Lane>;
  using LaneSegmentBox = ObjectWithAABox<Lane, ad_common::math::LineSegment2d>;
  using LaneSegmentKDTree = ad_common::math::AABoxKDTree2d<LaneSegmentBox>;
  Lane(const iflymapdata::sdpro::Lane& lane) : lane_(lane) {
    const auto points = lane.points();
    for (int i = 1; i < points.boot().points_size(); ++i) {
      segments_.emplace_back(
          Vec2d{points.boot().points(i - 1).x(),
                points.boot().points(i - 1).x()},
          Vec2d{points.boot().points(i).x(), points.boot().points(i).y()});
    }
  }
  const iflymapdata::sdpro::Lane& getLane() const { return lane_; }
  const std::vector<ad_common::math::LineSegment2d>& segments() const {
    return segments_;
  }

 private:
  iflymapdata::sdpro::Lane lane_;
  std::vector<ad_common::math::LineSegment2d> segments_;
};

class LaneInfo final : public MapInfoBase {
 public:
  using Ptr = std::shared_ptr<const LaneInfo>;
  LaneInfo() {}
  LaneInfo(const iflymapdata::sdpro::LaneInfo& lane_info)
      : lane_info_(lane_info) {}
  virtual void BuildSegmentKdTree(
      const ad_common::math::AABoxKDTreeParams& params) override;
  virtual std::vector<std::pair<uint64_t, uint64_t>> GetObjects(
      const Vec2d& center, double& radius) override;
  void Init(const iflymapdata::sdpro::LaneInfo& lane_info);
  Lane::Ptr getLaneById(const uint64_t& id) const;

 public:
  const iflymapdata::sdpro::LaneInfo& getLaneInfo() const { return lane_info_; }

 private:
  iflymapdata::sdpro::LaneInfo lane_info_;
  std::vector<Lane::LaneSegmentBox> segment_box_list_;
  std::unique_ptr<Lane::LaneSegmentKDTree> lane_segment_kdtree_;
  std::unordered_map<uint64_t, Lane::Ptr> lanes_table_;
};
}  // namespace sdpromap
}  // namespace ad_common
