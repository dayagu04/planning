#include "ad_common/hdmap/hdmap_impl.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <fstream>
#include <iostream>
#include <string>

#include "gflags/gflags.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"

namespace ad_common {
namespace hdmap {

using ad_common::math::Vec2d;

namespace {

bool GetProtoFromASCIIFile(const std::string &file_name,
                           google::protobuf::Message *message) {
  using google::protobuf::TextFormat;
  using google::protobuf::io::FileInputStream;
  using google::protobuf::io::ZeroCopyInputStream;
  int file_descriptor = open(file_name.c_str(), O_RDONLY);
  if (file_descriptor < 0) {
    std::cout << "Failed to open file " << file_name << " in text mode."
              << std::endl;
    // Failed to open;
    return false;
  }

  ZeroCopyInputStream *input = new FileInputStream(file_descriptor);
  bool success = TextFormat::Parse(input, message);
  if (!success) {
    std::cout << "Failed to parse file " << file_name << " as text proto."
              << std::endl;
  }
  delete input;
  close(file_descriptor);
  return success;
}

bool GetProtoFromBinaryFile(const std::string &file_name,
                            google::protobuf::Message *message) {
  std::fstream input(file_name, std::ios::in | std::ios::binary);
  if (!input.good()) {
    std::cout << "Failed to open file " << file_name << " in binary mode."
              << std::endl;
    return false;
  }
  if (!message->ParseFromIstream(&input)) {
    std::cout << "Failed to parse file " << file_name << " as binary proto."
              << std::endl;
    return false;
  }
  return true;
}

bool GetProtoFromFile(const std::string &file_name,
                      google::protobuf::Message *message) {
  // Try the binary parser first if it's much likely a binary proto.
  static const std::string kBinExt = ".bin";
  if (std::equal(kBinExt.rbegin(), kBinExt.rend(), file_name.rbegin())) {
    return GetProtoFromBinaryFile(file_name, message) ||
           GetProtoFromASCIIFile(file_name, message);
  }

  return GetProtoFromASCIIFile(file_name, message) ||
         GetProtoFromBinaryFile(file_name, message);
}

}  // namespace

class HDMapImplTestSuite : public ::testing::Test {
 public:
  HDMapImplTestSuite() {
    const std::string config_file = "/asw/ad_common/test_data/road_map.pb.txt";
    ::Map::RoadMap map_proto;
    GetProtoFromFile(config_file, &map_proto);
    hdmap_impl_.LoadMapFromProto(map_proto);
  }

 public:
  HDMapImpl hdmap_impl_;
};

TEST_F(HDMapImplTestSuite, GetLaneGroupById) {
  LaneGroupConstPtr lane_group_ptr = hdmap_impl_.GetLaneGroupById(1);
  EXPECT_NE(nullptr, lane_group_ptr);
  EXPECT_NEAR(lane_group_ptr->length(), 100.0, 1e-3);
}

TEST_F(HDMapImplTestSuite, GetLaneById) {
  LaneInfoConstPtr lane_into_ptr = hdmap_impl_.GetLaneById(1);
  EXPECT_NE(nullptr, lane_into_ptr);
  EXPECT_EQ(lane_into_ptr->lane().lane_group_id(), 1);
}

TEST_F(HDMapImplTestSuite, GetLaneBoundaryById) {
  LaneBoundaryInfoConstPtr lane_boundary_ptr =
      hdmap_impl_.GetLaneBoundaryById(1);
  EXPECT_NE(nullptr, lane_boundary_ptr);
  EXPECT_NEAR(lane_boundary_ptr->total_length(), 100.0, 1e-3);
}

TEST_F(HDMapImplTestSuite, GetRoadBoundaryById) {
  LaneBoundaryInfoConstPtr road_boundary_ptr =
      hdmap_impl_.GetRoadBoundaryById(1);
  EXPECT_NE(nullptr, road_boundary_ptr);
  EXPECT_NEAR(road_boundary_ptr->total_length(), 100.0, 1e-3);
}

TEST_F(HDMapImplTestSuite, GetLanes) {
  Vec2d point(50.0, 0.0);
  const double distance1 = 1.0;
  std::vector<LaneInfoConstPtr> lanes1;
  const int res1 = hdmap_impl_.GetLanes(point, distance1, &lanes1);
  EXPECT_EQ(res1, 0);
  EXPECT_EQ(lanes1.size(), 1);
  EXPECT_EQ(lanes1[0]->id(), 2);

  const double distance2 = 5.0;
  std::vector<LaneInfoConstPtr> lanes2;
  const int res2 = hdmap_impl_.GetLanes(point, distance2, &lanes2);
  EXPECT_EQ(res2, 0);
  EXPECT_EQ(lanes2.size(), 3);
}

TEST_F(HDMapImplTestSuite, GetNearestLane) {
  Vec2d point(50.0, 1.0);
  LaneInfoConstPtr nearest_lane;
  double nearest_s = 0.0;
  double nearest_l = 0.0;
  const int res =
      hdmap_impl_.GetNearestLane(point, &nearest_lane, &nearest_s, &nearest_l);
  EXPECT_EQ(res, 0);
  EXPECT_NE(nullptr, nearest_lane);
  EXPECT_NEAR(nearest_s, 50.0, 1e-3);
  EXPECT_NEAR(nearest_l, 1.0, 1e-3);
  EXPECT_EQ(nearest_lane->id(), 2);
}

TEST_F(HDMapImplTestSuite, GetNearestLaneWithHeading) {
  Vec2d point(50.0, 0.5);
  const double distance = 1.0;
  const double central_heading = 0.0;
  const double max_heading_difference = 0.1;
  LaneInfoConstPtr nearest_lane;
  double nearest_s = 0.0;
  double nearest_l = 0.0;
  const int res = hdmap_impl_.GetNearestLaneWithHeading(
      point, distance, central_heading, max_heading_difference, &nearest_lane,
      &nearest_s, &nearest_l);
  EXPECT_EQ(res, 0);
  EXPECT_NE(nullptr, nearest_lane);
  EXPECT_NEAR(nearest_s, 50.0, 1e-3);
  EXPECT_NEAR(nearest_l, 0.5, 1e-3);
  EXPECT_EQ(nearest_lane->id(), 2);
}

TEST_F(HDMapImplTestSuite, GetLanesWithHeading) {
  Vec2d point(50.0, 0.5);
  const double distance = 1.0;
  const double central_heading = 0.0;
  const double max_heading_difference = 0.1;
  std::vector<LaneInfoConstPtr> lanes;
  double nearest_s = 0.0;
  double nearest_l = 0.0;
  const int res = hdmap_impl_.GetLanesWithHeading(
      point, distance, central_heading, max_heading_difference, &lanes);
  EXPECT_EQ(res, 0);
  EXPECT_EQ(lanes.size(), 1);
  EXPECT_EQ(lanes[0]->id(), 2);
}

TEST_F(HDMapImplTestSuite, LaneInfo) {
  LaneInfoConstPtr lane_into_ptr = hdmap_impl_.GetLaneById(1);
  EXPECT_NE(nullptr, lane_into_ptr);
  EXPECT_EQ(lane_into_ptr->lane_group_id(), 1);
  EXPECT_EQ(lane_into_ptr->points().size(), 2);
  EXPECT_EQ(lane_into_ptr->unit_directions().size(), 2);

  const double s = 10.0;
  std::vector<::Map::BoundaryAttributes::Type> left_lane_boundary_types =
      lane_into_ptr->GetLeftLaneBoundaryTypes(s);
  EXPECT_EQ(left_lane_boundary_types[0], ::Map::BoundaryAttributes::DASH);
  std::vector<::Map::BoundaryAttributes::Type> right_lane_boundary_types =
      lane_into_ptr->GetRightLaneBoundaryTypes(s);
  EXPECT_EQ(right_lane_boundary_types[0], ::Map::BoundaryAttributes::SOLID);
  std::vector<::Map::BoundaryAttributes::Type> left_road_boundary_types =
      lane_into_ptr->GetLeftRoadBoundaryTypes(s);
  EXPECT_EQ(left_road_boundary_types[0], ::Map::BoundaryAttributes::SOLID);
  std::vector<::Map::BoundaryAttributes::Type> right_road_boundary_types =
      lane_into_ptr->GetRightRoadBoundaryTypes(s);
  EXPECT_EQ(right_road_boundary_types[0], ::Map::BoundaryAttributes::SOLID);

  Vec2d lane_point1 = lane_into_ptr->GetPoint(s);
  EXPECT_NEAR(lane_point1.x(), 10.0, 1e-3);
  EXPECT_NEAR(lane_point1.y(), -4.0, 1e-3);

  const double heading = lane_into_ptr->GetHeading(s);
  EXPECT_NEAR(heading, 0.0, 1e-3);

  const double curvature = lane_into_ptr->GetCurvature(s);
  EXPECT_NEAR(curvature, 0.0, 1e-3);

  EXPECT_EQ(lane_into_ptr->headings().size(), 2);
  EXPECT_EQ(lane_into_ptr->segments().size(), 1);
  EXPECT_EQ(lane_into_ptr->accumulate_s().size(), 2);
  EXPECT_NEAR(lane_into_ptr->total_length(), 100.0, 1e-3);

  EXPECT_EQ(lane_into_ptr->left_widths().size(), 2);
  EXPECT_EQ(lane_into_ptr->right_widths().size(), 2);
  double left_width = 0.0;
  double right_width = 0.0;
  lane_into_ptr->GetWidth(s, &left_width, &right_width);
  EXPECT_NEAR(left_width, 2.0, 1e-3);
  EXPECT_NEAR(right_width, 2.0, 1e-3);
  double lane_width = lane_into_ptr->GetWidth(s);
  EXPECT_NEAR(lane_width, 4.0, 1e-3);
  double effective_lane_width = lane_into_ptr->GetEffectiveWidth(s);
  EXPECT_NEAR(effective_lane_width, 4.0, 1e-3);

  EXPECT_EQ(lane_into_ptr->left_road_widths().size(), 2);
  EXPECT_EQ(lane_into_ptr->right_road_widths().size(), 2);
  double left_road_width = 0.0;
  double right_road_width = 0.0;
  lane_into_ptr->GetRoadWidth(s, &left_road_width, &right_road_width);
  EXPECT_NEAR(left_road_width, 10.0, 1e-3);
  EXPECT_NEAR(right_road_width, 2.0, 1e-3);
  double road_width = lane_into_ptr->GetRoadWidth(s);
  EXPECT_NEAR(road_width, 12.0, 1e-3);

  EXPECT_NEAR(lane_into_ptr->max_speed_limit(), 20.0, 1e-3);
  EXPECT_NEAR(lane_into_ptr->min_speed_limit(), 0.0, 1e-3);

  Vec2d point1(10.0, -4.0);
  EXPECT_TRUE(lane_into_ptr->IsOnLane(point1));
  Vec2d point2(10.0, 0.0);
  EXPECT_FALSE(lane_into_ptr->IsOnLane(point2));

  Vec2d lane_point2 = lane_into_ptr->GetSmoothPoint(s);
  EXPECT_NEAR(lane_point2.x(), 10.0, 1e-3);
  EXPECT_NEAR(lane_point2.y(), -4.0, 1e-3);

  Vec2d point3(10.0, -3.0);
  const double distance1 = lane_into_ptr->DistanceTo(point3);
  EXPECT_NEAR(distance1, 1.0, 1e-3);
  Vec2d map_point;
  double s_offset = 0.0;
  int s_offset_index = 0;
  const double distance2 =
      lane_into_ptr->DistanceTo(point3, &map_point, &s_offset, &s_offset_index);
  EXPECT_NEAR(distance2, 1.0, 1e-3);
  EXPECT_NEAR(map_point.x(), 10.0, 1e-3);
  EXPECT_NEAR(map_point.y(), -4.0, 1e-3);
  EXPECT_NEAR(s_offset, 10.0, 1e-3);
  EXPECT_EQ(s_offset_index, 0);

  double distance3 = 0.0;
  Vec2d point4 = lane_into_ptr->GetNearestPoint(point3, &distance3);
  EXPECT_NEAR(point4.x(), 10.0, 1e-3);
  EXPECT_NEAR(point4.y(), -4.0, 1e-3);
  EXPECT_NEAR(distance3, 1.0, 1e-3);

  double accumulate_s = 0.0;
  double lateral = 0.0;
  bool res = lane_into_ptr->GetProjection(point3, &accumulate_s, &lateral);
  EXPECT_TRUE(res);
  EXPECT_NEAR(accumulate_s, 10.0, 1e-3);
  EXPECT_NEAR(lateral, 1.0, 1e-3);
}

}  // namespace hdmap
}  // namespace ad_common

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}