#include "sdmap/sdmap.h"

#include <fcntl.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cfloat>
#include <fstream>
#include <iostream>
#include <string>

#include "gtest/gtest.h"

using namespace ad_common;
using ad_common::math::Vec2d;

using google::protobuf::io::FileInputStream;

static bool ReadProtoTxt(const char *file_path,
                         google::protobuf::Message *proto) {
  int file = open(file_path, O_RDONLY);
  if (file == -1) {
    return false;
  }
  FileInputStream *input = new FileInputStream(file);
  bool success = google::protobuf::TextFormat::Parse(input, proto);
  delete input;
  close(file);
  return success;
}

class SdMapTestSuite : public ::testing::Test {
 public:
  SdMapTestSuite() {
    const std::string config_file =
        "/home/ros/Downloads/mdc/system_integration/components/ad_common/"
        "test_data/sdmap_data.txt";
    SdMapSwtx::SdMap map_proto;
    if (ReadProtoTxt(config_file.c_str(), &map_proto)) {
      sdmap_.LoadMapFromProto(map_proto);
    } else {
      // ASSERT_TRUE(false);
    }
  }

 public:
  ad_common::sdmap::SDMap sdmap_;
};

// TEST_F(SdMapTestSuite, TestRead) {
//   double seg0_s, seg0_l;
//   auto seg0 = sdmap_.GetNearestRoad(
//       Vec2d{70732.2228503782, -890163.33817525511}, seg0_s, seg0_l);
//   ASSERT_TRUE(seg0 != nullptr);
//   EXPECT_EQ(seg0->id(), 61127107UL);
//   EXPECT_EQ(seg0->speed_limit(), 40U);
//   EXPECT_EQ(seg0_s, 0);
//   // seg0->PrintDebugString();

//   auto seg1 = sdmap_.GetNearestRoad(
//       Vec2d{70731.9833822719, -890129.29713228589}, seg0_s, seg0_l);
//   ASSERT_TRUE(seg1 != nullptr);
//   EXPECT_EQ(seg1->id(), 61127107UL);
//   EXPECT_EQ(seg1->speed_limit(), 40U);
//   EXPECT_LE(abs(seg0_s - seg1->dis()), 1.0);

//   const SdMapSwtx::Segment *cross1 = nullptr;
//   const SdMapSwtx::Segment *cross2 = nullptr;
//   double cross_dis1 = 0;
//   double cross_dis2 = 0;
//   sdmap_.GetCrossInfo(Vec2d{70732.2228503782, -890163.33817525511}, cross1,
//                       cross_dis1, cross2, cross_dis2);
//   ASSERT_TRUE(cross1 != nullptr);
//   ASSERT_TRUE(cross2 != nullptr);
//   EXPECT_EQ(cross1->id(), 102398271UL);
//   EXPECT_EQ(cross2->id(), 606969560UL);
//   EXPECT_EQ(cross_dis1, 66);
//   EXPECT_EQ(cross_dis2, 14606);

//   auto [ramp_seg, ramp_dis] =
//       sdmap_.GetRampInfo(Vec2d{70732.2228503782, -890163.33817525511});
//   ASSERT_TRUE(ramp_seg != nullptr);
//   EXPECT_EQ(ramp_seg->id(), 501980456UL);
//   EXPECT_EQ(ramp_dis, 14606);

//   auto [ramp1, ramp_dis1] =
//       sdmap_.GetRampInfo(Vec2d{67392.094675909611, -876779.01120213047});
//   ASSERT_TRUE(ramp1 != nullptr);
//   EXPECT_EQ(ramp1->id(), 501980456UL);
//   EXPECT_EQ(ramp_dis1, 0);
// }

TEST_F(SdMapTestSuite, TestReadsd) {
  auto splits =
      sdmap_.GetSplitInfoList(Vec2d{3096.2225863946969, -14127.716079117745});
  EXPECT_EQ(splits.size(), 4UL);
  for (const auto &[seg, _] : splits) {
    EXPECT_EQ(seg->out_link_size(), 2);
  }

  auto merges =
      sdmap_.GetMergeInfoList(Vec2d{3096.2225863946969, -14127.716079117745});
  EXPECT_EQ(merges.size(), 4UL);
  for (const auto &[seg, _] : merges) {
    EXPECT_EQ(seg->in_link_size(), 2);
  }

  // double seg0_s, seg0_l;
  // auto current = sdmap_.GetNearestRoad(
  //     Vec2d{3210.615847911658, -13439.856617448837}, seg0_s, seg0_l);
  // ASSERT_TRUE(current != nullptr);
  // EXPECT_EQ(current->id(), 95179323UL);
  // EXPECT_EQ(seg0_s, 0);
}