#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ad_common/hdmap/hdmap_lane_boundary_info.h"
#include "ad_common/hdmap/hdmap_lane_info.h"
#include "ad_common/math/aabox2d.h"
#include "ad_common/math/aaboxkdtree2d.h"
#include "ad_common/math/line_segment2d.h"
#include "ad_common/math/polygon2d.h"
#include "ad_common/math/vec2d.h"
#include "ehr.pb.h"

namespace ad_common {
namespace hdmap {
using LaneGroupConstPtr = std::shared_ptr<const ::Map::LaneGroup>;
using LaneBoundaryInfoConstPtr = std::shared_ptr<const LaneBoundaryInfo>;

/**
 * @class HDMapImpl
 *
 * @brief High-precision map loader implement.
 */
class HDMapImpl {
 public:
  using LaneInfoTable = std::unordered_map<uint64_t, std::shared_ptr<LaneInfo>>;
  using LaneGroupTable =
      std::unordered_map<uint64_t, std::shared_ptr<::Map::LaneGroup>>;

 public:
  /**
   * @brief load map from a protobuf message
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
  template <class Table, class BoxTable, class KDTree>
  static void BuildSegmentKDTree(
      const Table& table, const ad_common::math::AABoxKDTreeParams& params,
      BoxTable* const box_table, std::unique_ptr<KDTree>* const kdtree);

  template <class Table, class BoxTable, class KDTree>
  static void BuildPolygonKDTree(
      const Table& table, const ad_common::math::AABoxKDTreeParams& params,
      BoxTable* const box_table, std::unique_ptr<KDTree>* const kdtree);

  void BuildLaneSegmentKDTree();

  template <class KDTree>
  static int SearchObjects(const ad_common::math::Vec2d& center,
                           const double radius, const KDTree& kdtree,
                           std::vector<uint64_t>* const results);

  void Clear();

 private:
  LaneInfoTable lane_table_;
  LaneGroupTable lane_group_table_;
  LaneBoundaryInfoTable lane_boundary_table_;
  LaneBoundaryInfoTable road_boundary_table_;

  std::vector<LaneSegmentBox> lane_segment_boxes_;
  std::unique_ptr<LaneSegmentKDTree> lane_segment_kdtree_;
};

}  // namespace hdmap
}  // namespace ad_common
