#include "sdpromap/sdpromap.h"

#include <fcntl.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_interface/DebugInfo.h>
#include <struct_msgs/IFLYLocalization.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cfloat>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "ifly_localization_c.h"

using namespace ad_common;
using ad_common::math::Vec2d;

using google::protobuf::io::FileInputStream;

struct EgoMotion {
  EgoMotion() : x(0.), y(0.), z(0.), yaw(0.), pitch(0.), roll(0.) {}
  EgoMotion(double x, double y, double z, double yaw, double pitch, double roll)
      : x(x), y(y), z(z), yaw(yaw), pitch(pitch), roll(roll) {}
  double x;
  double y;
  double z;
  double yaw;
  double pitch;
  double roll;
};

std::map<uint64_t, iflymapdata::sdpro::MapData> sdpromap_msgs;
std::map<uint64_t, EgoMotion> localization_msgs;

// 前向声明
bool MergeRouteLinks(std::vector<uint64_t> &route_link_ids,
                     const std::vector<uint64_t> &frame_route_link_ids);
void ValidateLocalizationResults(const std::vector<uint64_t> &route_link_ids,
                                 const std::vector<uint64_t> &cur_link_ids,
                                 const std::vector<uint64_t> &next_link_ids);

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
        "/home/jdmo/ifly_workspace/tmp/system_integration/components/ad_common/"
        "test_data/sdpromap_data.txt";
    iflymapdata::sdpro::MapData map_proto;
    if (ReadProtoTxt(config_file.c_str(), &map_proto)) {
      sdpromap_.LoadMapFromProto(map_proto);
    } else {
      // ASSERT_TRUE(false);
    }
  }

 public:
  ad_common::sdpromap::SDProMap sdpromap_;
};

void loadRosBag(const std::string &bag_path) {
  std::cout << "Start loading ROS bag..." << std::endl;
  std::cout.flush();

  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
    std::cout << "Bag opened successfully" << std::endl;
    std::cout.flush();
  } catch (std::exception &e) {
    std::cout << "Failed to open bag: " << e.what() << std::endl;
    std::cout.flush();
    return;
  }

  rosbag::View view1(bag);
  ad_common::sdpromap::SDProMap sdpromap_;
  std::cout << "Processing messages..." << std::endl;
  std::cout.flush();

  for (rosbag::MessageInstance const m : view1) {
    if (m.getTopic() == "/iflytek/ehr/sdpromap_info") {
      auto ros_sdpromap_msg = m.instantiate<sensor_interface::DebugInfo>();
      if (!ros_sdpromap_msg) {
        std::cout << "Warning: Failed to instantiate DebugInfo message"
                  << std::endl;
        continue;
      }
      std::string str(ros_sdpromap_msg->debug_info.begin(),
                      ros_sdpromap_msg->debug_info.end());

      iflymapdata::sdpro::MapData sdpro_mapdata{};
      sdpro_mapdata.ParseFromString(str);
      sdpromap_msgs[sdpro_mapdata.header().timestamp()] = sdpro_mapdata;

      // sdpromap_.LoadMapFromProto(sdpro_mapdata);
      // double nearest_s = 0, nearest_l = 0;
      // const auto link = sdpromap_.GetNearestLinkWithHeading(
      //     ad_common::math::Vec2d(-196.91695631479107, -74.27713742092732),
      //     50, -2.742947607115556, 0.7853, nearest_s, nearest_l);
      // if (link) {
      //   std::cout << "--------link_id--------:" << link->id() << std::endl;
      // }
    } else if (m.getTopic() == "/iflytek/localization/egomotion") {
      auto msg_ptr = m.instantiate<struct_msgs::IFLYLocalization>();
      if (!msg_ptr) {
        std::cout << "Warning: Failed to instantiate IFLYLocalization message, "
                     "skipping..."
                  << std::endl;
        std::cout.flush();
        continue;
      }
      EgoMotion pose(msg_ptr->position.position_boot.x,
                     msg_ptr->position.position_boot.y,
                     msg_ptr->position.position_boot.z,
                     msg_ptr->orientation.euler_boot.yaw,
                     msg_ptr->orientation.euler_boot.pitch,
                     msg_ptr->orientation.euler_boot.roll);
      localization_msgs[msg_ptr->msg_header.stamp] = pose;
    }
  }

  std::cout << "sdpromap_msgs size: " << sdpromap_msgs.size() << std::endl;
  std::cout << "localization_msgs size: " << localization_msgs.size()
            << std::endl;
}

/*
  当前eval未涉及到非导航情况
*/
void run() {
  std::vector<uint64_t> route_link_ids;  // 收集所有route link ids（去重组合）
  std::vector<uint64_t> cur_link_ids;
  std::vector<uint64_t> next_link_ids;
  ad_common::sdpromap::SDProMap sdpromap;
  std::string path_id = "";
  bool has_load_sdpromap = false;
  if (localization_msgs.empty()) {
    std::cout << "localization_msgs is empty" << std::endl;
    return;
  }
  for (auto ego_motion : localization_msgs) {
    // std::cout << "ego_motion timestamp: " << ego_motion.first << std::endl;
    if (sdpromap_msgs.empty()) {
      std::cout << "sdmap_proto is empty" << std::endl;
      continue;
    }
    for (auto sdmap = sdpromap_msgs.begin(); sdmap != sdpromap_msgs.end();) {
      bool find_timealign = false;
      if (sdmap->first < ego_motion.first) {
        if (!sdmap->second.has_route()) {
          std::cout << "sdpromap has no route" << std::endl;
          sdmap = sdpromap_msgs.erase(sdmap);
          continue;
        }
        // std::cout << "sdmap timestamp is less than ego_motion timestamp:"
        //           << sdmap->first << std::endl;
        std::string new_path_id = sdmap->second.route().path_id();
        if (!path_id.empty() && path_id != new_path_id) {
          // 发生了导航路线的变化
          std::cout << "path id changed from " << path_id << " to "
                    << new_path_id << std::endl;
          ValidateLocalizationResults(route_link_ids, cur_link_ids,
                                      next_link_ids);
          // 清空route_link_ids, cur_link_ids, next_link_ids，sdpromap
          route_link_ids.clear();
          cur_link_ids.clear();
          next_link_ids.clear();
          sdpromap = ad_common::sdpromap::SDProMap();
          has_load_sdpromap = false;
        }
        path_id = new_path_id;
        sdpromap.LoadMapFromProto(sdmap->second);
        has_load_sdpromap = true;
        find_timealign = true;
        auto new_route_link_ids = sdpromap.getRouteLinksIds();
        // 智能合并route_link_ids，处理重规划情况
        if (route_link_ids.empty()) {
          route_link_ids = new_route_link_ids;
        } else {
          (void)MergeRouteLinks(route_link_ids, new_route_link_ids);
        }
        sdmap = sdpromap_msgs.erase(sdmap);
      }
      if (has_load_sdpromap) {
        std::cout << "===================================run "
                     "sdpromap start==================================="
                  << std::endl;
        double nearest_s = 0, nearest_l = 0;
        const auto link = sdpromap.GetNearestLinkWithHeading(
            ad_common::math::Vec2d(ego_motion.second.x, ego_motion.second.y),
            50.0, ego_motion.second.yaw, 0.78539815, nearest_s, nearest_l);
        if (!sdpromap.isGoingOffCourse()) {
          if (link) {
            std::cout << "nearest link id: " << link->id() << std::endl;
            if (cur_link_ids.empty() || cur_link_ids.back() != link->id()) {
              cur_link_ids.push_back(link->id());
            }
            /*
              避免cur_link已经是route的最后一段，而进行误报错
            */
            if (sdpromap.getCurRouteIndex() ==
                static_cast<int>(sdpromap.getRouteLinksIds().size()) - 1) {
              std::cout << "cur link is the last link in route" << std::endl;
            } else {
              const auto next_link = sdpromap.GetNextLinkOnRoute(link->id());
              if (next_link) {
                std::cout << "next link id: " << next_link->id() << std::endl;
                if (next_link_ids.empty() ||
                    next_link_ids.back() != next_link->id()) {
                  next_link_ids.push_back(next_link->id());
                }
              } else {
                std::cout << "cannot get next link" << std::endl;
              }
            }
          } else {
            std::cout << "cannot get nearest link" << std::endl;
          }
        }

        std::cout << "===================================run "
                     "sdrpromap end==================================="
                  << std::endl;
      }
      if (!find_timealign) {
        break;
      }
    }
  }
  std::cout << "Recorded Link IDs:" << std::endl;
  for (const auto &id : route_link_ids) {
    std::cout << "Route Link ID: " << id << std::endl;
  }
  for (const auto &id : cur_link_ids) {
    std::cout << "Cur Link ID: " << id << std::endl;
  }
  for (const auto &next_id : next_link_ids) {
    std::cout << "Next Link ID: " << next_id << std::endl;
  }

  // 校验定位结果
  ValidateLocalizationResults(route_link_ids, cur_link_ids, next_link_ids);
}

bool MergeRouteLinks(std::vector<uint64_t> &route_link_ids,
                     const std::vector<uint64_t> &new_route_link_ids) {
  // 动态规划求解最大子串
  std::vector<ad_common::math::SubStringInfo> sub_string_infos;
  ad_common::math::SolveLongestSubString(route_link_ids, new_route_link_ids,
                                         sub_string_infos);
  if (sub_string_infos.size() == 1) {
    // 检查sub_string_infos中的数据是否可以使用
    const auto &sub_string = sub_string_infos.front();
    if (sub_string.a_end_idx <= 0 || sub_string.b_end_idx <= 0 ||
        sub_string.max_len <= 0 ||
        sub_string.max_len > static_cast<int>(route_link_ids.size()) ||
        sub_string.max_len > static_cast<int>(new_route_link_ids.size()) ||
        sub_string.a_end_idx >= static_cast<int>(route_link_ids.size()) ||
        sub_string.b_end_idx >= static_cast<int>(new_route_link_ids.size())) {
      route_link_ids = new_route_link_ids;
      std::cout << "cannot merge route links for 1" << std::endl;
      return false;
    }
    /*
     检查最大子串是否满足：
     起始点 == new_route_link_ids起始点
     末尾点 == route_link_ids结尾点
    */
    if (sub_string.a_end_idx != static_cast<int>(route_link_ids.size()) - 1 ||
        sub_string.b_end_idx + 1 - sub_string.max_len != 0) {
      route_link_ids = new_route_link_ids;
      std::cout << "cannot merge route links for 2" << std::endl;
      return false;
    }
    // 检查没问题，开始拼接
    route_link_ids.insert(route_link_ids.end(),
                          new_route_link_ids.begin() + sub_string.b_end_idx + 1,
                          new_route_link_ids.end());
    return true;
  } else {
    route_link_ids = new_route_link_ids;
    std::cout << "cannot merge route links for 3" << std::endl;
    return false;
  }
}

// 校验定位结果是否符合route link的顺序
void ValidateLocalizationResults(const std::vector<uint64_t> &route_link_ids,
                                 const std::vector<uint64_t> &cur_link_ids,
                                 const std::vector<uint64_t> &next_link_ids) {
  std::cout << "\n==========================================" << std::endl;
  std::cout << "check map loc result..." << std::endl;
  std::cout << "==========================================" << std::endl;

  if (route_link_ids.empty() || cur_link_ids.empty() || next_link_ids.empty() ||
      cur_link_ids.size() != next_link_ids.size()) {
    return;
  }

  // 构建route link的索引映射，用于快速查找
  // 使用vector保存每个link_id的所有出现位置（因为link_id可能在route中重复出现）
  std::map<uint64_t, std::vector<size_t>> route_link_index;
  for (size_t i = 0; i < route_link_ids.size(); ++i) {
    route_link_index[route_link_ids[i]].push_back(i);
  }

  // 校验Cur Link ID是否符合route link的顺序
  std::cout << "\n--- check Cur Link ID is or not in order ---" << std::endl;
  size_t cur_route_index = 0;  // 当前在route中的位置

  for (size_t i = 0; i < cur_link_ids.size(); ++i) {
    const auto &cur_link_id = cur_link_ids[i];
    const auto &next_link_id = next_link_ids[i];
    if (route_link_index.find(cur_link_id) != route_link_index.end()) {
      bool has_found = false;
      for (const auto &index : route_link_index[cur_link_id]) {
        if (index >= cur_route_index) {
          // 增加=，防止cur_link_ids的起始点和route_link_ids起始点相同
          cur_route_index = index;
          has_found = true;
          break;
        } else {
          continue;
        }
      }
      if (!has_found) {
        std::cout << "check cur link id error 1" << std::endl;
      }
    } else {
      std::cout << "check cur link id error 2" << std::endl;
    }
    if (route_link_index.find(next_link_id) == route_link_index.end()) {
      for (const auto &index : route_link_index[next_link_id]) {
        if (index - 1 < 0 || route_link_ids[index - 1] != cur_link_id) {
          std::cout << "check next link id error" << std::endl;
        }
      }
    }
  }

  std::cout << "\n==========================================" << std::endl;
  std::cout << "check end" << std::endl;
  std::cout << "==========================================" << std::endl;
}

int main(int argc, char **argv) {
  std::cout << "========================================" << std::endl;
  std::cout << "Program started" << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout.flush();

  if (argc < 2) {
    std::cout << "./mega_rosbag_play <bag path>" << std::endl;
    return 0;
  }
  const std::string input_bag = argv[1];
  std::cout << "Loading bag file: " << input_bag << std::endl;
  std::cout.flush();

  loadRosBag(input_bag);
  run();

  return 0;
}
