#include "ad_common/hdmap/hdmap_impl.h"

#include <assert.h>

#include <algorithm>
#include <limits>
#include <mutex>
#include <set>
#include <unordered_set>

namespace ad_common {
namespace hdmap {
namespace {
using ad_common::math::AABoxKDTreeParams;
using ad_common::math::Vec2d;
using ::Map::LaneGroup;
using ::Map::RoadMap;
}  // namespace

int HDMapImpl::LoadMapFromProto(const RoadMap& map_proto) {
  Clear();

  for (const auto& lane_boundary : map_proto.lane_boundaries()) {
    lane_boundary_table_[lane_boundary.boundary_id()] = 
      std::make_unique<LaneBoundaryInfo>(lane_boundary);
    if (!lane_boundary_table_[lane_boundary.boundary_id()]->is_valid()) {
      return -1;
    }
  }
  for (const auto& road_boundary : map_proto.road_boundaries()) {
    road_boundary_table_[road_boundary.boundary_id()] =
      std::make_unique<LaneBoundaryInfo>(road_boundary);
    if (!road_boundary_table_[road_boundary.boundary_id()]->is_valid()) {
      return -1;
    }
  }
  for (const auto& lane : map_proto.lanes()) {
    lane_table_[lane.lane_id()] = 
      std::make_unique<LaneInfo>(lane, lane_boundary_table_, road_boundary_table_);
    if (!lane_table_[lane.lane_id()]->is_valid()) {
      return -1;
    }
  }
  for (const auto& lane_group : map_proto.lane_groups()) {
    lane_group_table_[lane_group.lane_group_id()] = 
      std::make_unique<LaneGroup>(lane_group);
  }
  BuildLaneSegmentKDTree();
  return 0;
}

LaneGroupConstPtr HDMapImpl::GetLaneGroupById(const uint64_t id) const {
  LaneGroupTable::const_iterator it = lane_group_table_.find(id);
  return it != lane_group_table_.end() ? it->second : nullptr;
}

LaneInfoConstPtr HDMapImpl::GetLaneById(const uint64_t id) const {
  LaneInfoTable::const_iterator it = lane_table_.find(id);
  return it != lane_table_.end() ? it->second : nullptr;
}

LaneBoundaryInfoConstPtr HDMapImpl::GetLaneBoundaryById(
    const uint64_t id) const {
  LaneBoundaryInfoTable::const_iterator it = lane_boundary_table_.find(id);
  return it != lane_boundary_table_.end() ? it->second : nullptr;
}

LaneBoundaryInfoConstPtr HDMapImpl::GetRoadBoundaryById(
    const uint64_t id) const {
  LaneBoundaryInfoTable::const_iterator it = road_boundary_table_.find(id);
  return it != road_boundary_table_.end() ? it->second : nullptr;
}

int HDMapImpl::GetLanes(const Vec2d& point, double distance,
                        std::vector<LaneInfoConstPtr>* lanes) const {
  if (lanes == nullptr || lane_segment_kdtree_ == nullptr) {
    return -1;
  }
  lanes->clear();
  std::vector<uint64_t> ids;
  const int status =
      SearchObjects(point, distance, *lane_segment_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto& id : ids) {
    auto lane_temp = GetLaneById(id);
    if (!lane_temp) { continue; }
    lanes->emplace_back(lane_temp);
  }
  return 0;
}

int HDMapImpl::GetNearestLane(const Vec2d& point,
                              LaneInfoConstPtr* nearest_lane, double* nearest_s,
                              double* nearest_l) const {
  if (nearest_lane == nullptr || nearest_s == nullptr || nearest_l == nullptr) {
    return -1;
  }
  // assert(nearest_lane != nullptr);
  // assert(nearest_s != nullptr);
  // assert(nearest_l != nullptr);

  const auto* segment_object = lane_segment_kdtree_->GetNearestObject(point);
  if (segment_object == nullptr) {
    return -1;
  }
  const uint64_t lane_id = segment_object->object()->id();
  *nearest_lane = GetLaneById(lane_id);
  if (*nearest_lane == nullptr) {
    return -1;
  }
  // assert(*nearest_lane != nullptr);

  const int id = segment_object->id();
  const auto& segment = (*nearest_lane)->segments()[id];
  Vec2d nearest_pt;
  segment.DistanceTo(point, &nearest_pt);
  *nearest_s = (*nearest_lane)->accumulate_s()[id] +
               nearest_pt.DistanceTo(segment.start());
  *nearest_l = segment.unit_direction().CrossProd(point - segment.start());

  return 0;
}

int HDMapImpl::GetNearestLaneWithHeading(
    const Vec2d& point, const double distance, const double central_heading,
    const double max_heading_difference, LaneInfoConstPtr* nearest_lane,
    double* nearest_s, double* nearest_l) const {
  if (nearest_lane == nullptr || nearest_s == nullptr || nearest_l == nullptr) {
    return -1;
  }
  // assert(nearest_lane != nullptr);
  // assert(nearest_s != nullptr);
  // assert(nearest_l != nullptr);

  std::vector<LaneInfoConstPtr> lanes;
  if (GetLanesWithHeading(point, distance, central_heading,
                          max_heading_difference, &lanes) != 0) {
    return -1;
  }

  double s = 0;
  size_t s_index = 0;
  Vec2d map_point;
  double min_distance = distance;
  for (const auto& lane : lanes) {
    double s_offset = 0.0;
    int s_offset_index = 0;
    double distance =
        lane->DistanceTo(point, &map_point, &s_offset, &s_offset_index);
    if (distance < min_distance) {
      min_distance = distance;
      *nearest_lane = lane;
      s = s_offset;
      s_index = s_offset_index;
    }
  }

  if (*nearest_lane == nullptr) {
    return -1;
  }

  *nearest_s = s;
  int segment_index = static_cast<int>(
      std::min(s_index, (*nearest_lane)->segments().size() - 1));
  const auto& segment_2d = (*nearest_lane)->segments()[segment_index];
  *nearest_l =
      segment_2d.unit_direction().CrossProd(point - segment_2d.start());

  return 0;
}

int HDMapImpl::GetLanesWithHeading(const Vec2d& point, const double distance,
                                   const double central_heading,
                                   const double max_heading_difference,
                                   std::vector<LaneInfoConstPtr>* lanes) const {
  if (lanes == nullptr) {
    return -1;
  }
  // assert(lanes != nullptr);
  std::vector<LaneInfoConstPtr> all_lanes;
  const int status = GetLanes(point, distance, &all_lanes);
  if (status < 0 || all_lanes.empty()) {
    return -1;
  }

  lanes->clear();
  for (auto& lane : all_lanes) {
    Vec2d proj_pt(0.0, 0.0);
    double s_offset = 0.0;
    int s_offset_index = 0;
    double dis = lane->DistanceTo(point, &proj_pt, &s_offset, &s_offset_index);
    if (dis <= distance) {
      double heading_diff =
          fabs(lane->headings()[s_offset_index] - central_heading);
      if (fabs(ad_common::math::NormalizeAngle(heading_diff)) <=
          max_heading_difference) {
        lanes->push_back(lane);
      }
    }
  }

  return 0;
}

template <class Table, class BoxTable, class KDTree>
void HDMapImpl::BuildSegmentKDTree(const Table& table,
                                   const AABoxKDTreeParams& params,
                                   BoxTable* const box_table,
                                   std::unique_ptr<KDTree>* const kdtree) {
  box_table->clear();
  for (const auto& info_with_id : table) {
    const auto* info = info_with_id.second.get();
    for (size_t id = 0; id < info->segments().size(); ++id) {
      const auto& segment = info->segments()[id];
      box_table->emplace_back(
          ad_common::math::AABox2d(segment.start(), segment.end()), info,
          &segment, id);
    }
  }
  *kdtree = std::make_unique<KDTree>(*box_table, params);
}

template <class Table, class BoxTable, class KDTree>
void HDMapImpl::BuildPolygonKDTree(const Table& table,
                                   const AABoxKDTreeParams& params,
                                   BoxTable* const box_table,
                                   std::unique_ptr<KDTree>* const kdtree) {
  box_table->clear();
  for (const auto& info_with_id : table) {
    const auto* info = info_with_id.second.get();
    const auto& polygon = info->polygon();
    box_table->emplace_back(polygon.AABoundingBox(), info, &polygon, 0);
  }
  *kdtree = std::make_unique<KDTree>(*box_table, params);
}

void HDMapImpl::BuildLaneSegmentKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 16;
  BuildSegmentKDTree(lane_table_, params, &lane_segment_boxes_,
                     &lane_segment_kdtree_);
}

template <class KDTree>
int HDMapImpl::SearchObjects(const Vec2d& center, const double radius,
                             const KDTree& kdtree,
                             std::vector<uint64_t>* const results) {
  if (results == nullptr) {
    return -1;
  }
  auto objects = kdtree.GetObjects(center, radius);
  std::unordered_set<uint64_t> result_ids;
  result_ids.reserve(objects.size());
  for (const auto* object_ptr : objects) {
    result_ids.insert(object_ptr->object()->id());
  }

  results->reserve(result_ids.size());
  results->assign(result_ids.begin(), result_ids.end());
  return 0;
}

void HDMapImpl::Clear() {
  lane_table_.clear();
  lane_group_table_.clear();
  lane_boundary_table_.clear();
  road_boundary_table_.clear();
  lane_segment_boxes_.clear();
  lane_segment_kdtree_.reset(nullptr);
}

}  // namespace hdmap
}  // namespace ad_common
