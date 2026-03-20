#include "ad_common/hdmap/hdmap.h"

#include "ad_common/hdmap/hdmap_utils.h"

namespace ad_common {
namespace hdmap {

using ad_common::math::Vec2d;
using ::Map::RoadMap;

int HDMap::LoadMapFromProto(const RoadMap& map_proto) {
  return impl_.LoadMapFromProto(map_proto);
}

LaneGroupConstPtr HDMap::GetLaneGroupById(const uint64_t id) const {
  return impl_.GetLaneGroupById(id);
}

LaneInfoConstPtr HDMap::GetLaneById(const uint64_t id) const {
  return impl_.GetLaneById(id);
}

LaneBoundaryInfoConstPtr HDMap::GetLaneBoundaryById(const uint64_t id) const {
  return impl_.GetLaneBoundaryById(id);
}

LaneBoundaryInfoConstPtr HDMap::GetRoadBoundaryById(const uint64_t id) const {
  return impl_.GetRoadBoundaryById(id);
}

int HDMap::GetNearestLane(const Vec2d& point, LaneInfoConstPtr* nearest_lane,
                          double* nearest_s, double* nearest_l) const {
  return impl_.GetNearestLane(point, nearest_lane, nearest_s, nearest_l);
}

int HDMap::GetNearestLaneWithHeading(const Vec2d& point, const double distance,
                                     const double central_heading,
                                     const double max_heading_difference,
                                     LaneInfoConstPtr* nearest_lane,
                                     double* nearest_s,
                                     double* nearest_l) const {
  return impl_.GetNearestLaneWithHeading(point, distance, central_heading,
                                         max_heading_difference, nearest_lane,
                                         nearest_s, nearest_l);
}

int HDMap::GetLanesWithHeading(const Vec2d& point, const double distance,
                               const double central_heading,
                               const double max_heading_difference,
                               std::vector<LaneInfoConstPtr>* lanes) const {
  return impl_.GetLanesWithHeading(point, distance, central_heading,
                                   max_heading_difference, lanes);
}

}  // namespace hdmap
}  // namespace ad_common
