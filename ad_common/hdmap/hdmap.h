#pragma once

#include <string>
#include <utility>
#include <vector>

#include "ad_common/hdmap/hdmap_impl.h"
#include "ad_common/hdmap/hdmap_lane_info.h"
#include "ehr.pb.h"

namespace ad_common {
namespace hdmap {

/**
 * @class HDMap
 *
 * @brief High-precision map loader interface.
 */
class HDMap {
 public:
  /**
   * @brief load map from a given protobuf message.
   * @param map_proto map data in protobuf format
   * @return 0:success, otherwise failed
   */
  int LoadMapFromProto(const ::Map::RoadMap& map_proto);

  LaneGroupConstPtr GetLaneGroupById(const uint64_t id) const;

  LaneInfoConstPtr GetLaneById(const uint64_t id) const;

  LaneBoundaryInfoConstPtr GetLaneBoundaryById(const uint64_t id) const;

  LaneBoundaryInfoConstPtr GetRoadBoundaryById(const uint64_t id) const;

  /**
   * @brief get all lanes in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param lanes store all lanes in target range
   * @return 0:success, otherwise failed
   */
  int GetLanes(const ad_common::math::Vec2d& point, double distance,
               std::vector<LaneInfoConstPtr>* lanes) const;

  /**
   * @brief get nearest lane from target point,
   * @param point the target point
   * @param nearest_lane the nearest lane that match search conditions
   * @param nearest_s the offset from lane start point along lane center line
   * @param nearest_l the lateral offset from lane center line
   * @return 0:success, otherwise, failed.
   */
  int GetNearestLane(const ad_common::math::Vec2d& point,
                     LaneInfoConstPtr* nearest_lane, double* nearest_s,
                     double* nearest_l) const;
  /**
   * @brief get the nearest lane within a certain range by pose
   * @param point the target position
   * @param distance the search radius
   * @param central_heading the base heading
   * @param max_heading_difference the heading range
   * @param nearest_lane the nearest lane that match search conditions
   * @param nearest_s the offset from lane start point along lane center line
   * @param nearest_l the lateral offset from lane center line
   * @return 0:success, otherwise, failed.
   */
  int GetNearestLaneWithHeading(const ad_common::math::Vec2d& point,
                                const double distance,
                                const double central_heading,
                                const double max_heading_difference,
                                LaneInfoConstPtr* nearest_lane,
                                double* nearest_s, double* nearest_l) const;
  /**
   * @brief get all lanes within a certain range by pose
   * @param point the target position
   * @param distance the search radius
   * @param central_heading the base heading
   * @param max_heading_difference the heading range
   * @param nearest_lane all lanes that match search conditions
   * @return 0:success, otherwise, failed.
   */
  int GetLanesWithHeading(const ad_common::math::Vec2d& point,
                          const double distance, const double central_heading,
                          const double max_heading_difference,
                          std::vector<LaneInfoConstPtr>* lanes) const;

 private:
  HDMapImpl impl_;
};

}  // namespace hdmap
}  // namespace ad_common