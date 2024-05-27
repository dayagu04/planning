#pragma once

#include <google/protobuf/message.h>
#include <memory>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/any.hpp>
#include <cstdint>

#include "planning_adapter.h"
#include "struct.h"

namespace planning {
namespace planning_player {

using TopicMsgTimeCache =
    std::map<std::string, std::map<ros::Time, boost::any>>;
using TopicHeaderTimeCache =
    std::map<std::string, std::map<uint64_t, boost::any>>;
using PlanningMsgCache =
    std::map<std::string, std::map<ros::Time, struct_msgs::PlanningOutput>>;

class PlanningPlayer {
 public:
  struct DynamicState {
    DynamicState() {}
    DynamicState(const Eigen::Vector2d &pos_r, const double heading_r) {
      pos = pos_r;
      heading = heading_r;
    }

    double vel = 0.0;
    Eigen::Vector2d pos = Eigen::Vector2d::Zero();
    double heading = 0.0;
    double static_time = 0.0;
    bool static_flag = false;

    void Reset() {
      vel = 0.0;
      pos = Eigen::Vector2d::Zero();
      heading = 0.0;
    }
  };
  PlanningPlayer() = default;
  ~PlanningPlayer() = default;

  void Init(bool is_close_loop, double auto_time_sec,
            const std::string &scene_type);
  void Clear();
  bool LoadRosBag(const std::string &bag_path);
  void StoreRosBag(const std::string &bag_path);
  void PlayOneFrame(int frame_num,
                    const planning::common::TopicTimeList &input_time_list);
  void PlayAllFrames();

  void RunCloseLoop(const struct_msgs::PlanningOutput &planning_output);
  void PerfectControlHPP(const struct_msgs::PlanningOutput &plan_msg,
                         uint64_t delta_t,
                         struct_msgs::IFLYLocalization::Ptr loc_msg);
  void PerfectControlSCC(const struct_msgs::PlanningOutput &plan_msg,
                         uint64_t delta_t,
                         struct_msgs::LocalizationEstimate::Ptr loc_msg);
  void PerfectControlAPA(const struct_msgs::PlanningOutput &plan_msg,
                         uint64_t delta_t,
                         struct_msgs::LocalizationEstimate::Ptr loc_msg);

 private:
  DynamicState state_;
  std::unique_ptr<PlanningAdapter> planning_adapter_ = nullptr;
  TopicHeaderTimeCache header_cache_{};  // msg cache indexed by header time(us)
  TopicMsgTimeCache msg_cache_{};        // msg cache indexed by msg time(ns)
  PlanningMsgCache
      output_planning_msg_cache_{};  // output cache indexed by msg time(ns)
  std::map<uint64_t, boost::any> msg_cache_ordered_by_time_;
  std::map<std::string, std::string> proto_desc_map_{};
  ros::Time planning_msg_time_s_;
  uint64_t planning_header_time_us_ = 0;
  int frame_num_ = 0;
  ros::Time planning_dubug_info_msg_time_s_;
  uint64_t planning_dubug_info_header_time_us_ = 0;
  uint64_t input_time_list_map_ = 0;
  uint64_t input_time_list_road_fusion_ = 0;
  uint64_t planning_start_timestamp_ = 0;
  uint64_t next_loc_header_time_us_ = 0;
  uint64_t loc_header_time_us_ = 0;
  uint64_t next_loc_esti_header_time_us_ = 0;
  uint64_t loc_esti_header_time_us_ = 0;
  uint64_t planning_dubug_info_frame_num_ = 0;
  int frame_num_before_enter_auto_ = 0;
  std::string scene_type_ = "acc";

  template <class T>
  void cache_with_ros_msg_time(const rosbag::MessageInstance &msg);

  template <class T>
  void cache_with_ros_msg_and_header_time(const rosbag::MessageInstance &msg);

  template <class T>
  void write_ros_msg(const std::map<ros::Time, boost::any> &write_msg,
                     const std::string &topic_name, rosbag::Bag &bag);

  template <class T>
  typename T::Ptr find_ros_msg_with_header_time(const std::string &topic,
                                                uint64_t time);

  inline bool check_msg_exist(TopicMsgTimeCache &msg_cache,
                              const std::string &topic_name) {
    if (msg_cache.find(topic_name) == msg_cache.end()) {
      std::cerr << "topic not found:" << topic_name << std::endl;
      return false;
    }
    if (msg_cache[topic_name].empty()) {
      std::cerr << "no msg in topic:" << topic_name << std::endl;
      return false;
    }
    return true;
  }

  // planning::common::PlanningDebugInfo planning_debug_info;
  std::shared_ptr<planning::common::PlanningDebugInfo> convert_debug_info(
      boost::shared_ptr<sensor_interface::DebugInfo_<std::allocator<void>>>
          debug_info) {
    auto planning_debug_info =
        std::make_shared<planning::common::PlanningDebugInfo>();
    std::string planning_debug_info_str(debug_info->debug_info.begin(),
                                        debug_info->debug_info.end());
    planning_debug_info->ParseFromString(planning_debug_info_str);
    return planning_debug_info;
  }
};

template <class T>
void PlanningPlayer::cache_with_ros_msg_time(
    const rosbag::MessageInstance &msg) {
  typename T::Ptr obj_msg = msg.instantiate<T>();
  if (obj_msg == nullptr) {
    std::cout << "msg instantiate error, msg name: " << msg.getTopic()
              << std::endl;
  } else {
    // auto time = msg.getTime();
    // uint64_t time_in_ns = time.sec * 1000000000ULL + time.nsec;
    msg_cache_[msg.getTopic()][msg.getTime()] = obj_msg;  // ns
  }
}

template <class T>
void PlanningPlayer::cache_with_ros_msg_and_header_time(
    const rosbag::MessageInstance &msg) {
  typename T::Ptr obj_msg = msg.instantiate<T>();
  if (obj_msg == nullptr) {
    std::cout << "msg instantiate error, msg name: " << msg.getTopic()
              << std::endl;
  } else {
    // auto time = msg.getTime();
    // uint64_t time_in_ns = time.sec * 1000000000ULL + time.nsec;
    msg_cache_[msg.getTopic()][msg.getTime()] = obj_msg;  // ns
    header_cache_[msg.getTopic()][obj_msg->msg_header.timestamp] =
        obj_msg;  // us
  }
}

template <class T>
void PlanningPlayer::write_ros_msg(
    const std::map<ros::Time, boost::any> &write_msg,
    const std::string &topic_name, rosbag::Bag &bag) {
  for (const auto &i : write_msg) {
    auto msg = boost::any_cast<T>(i.second);
    bag.write(topic_name, i.first, msg);
  }
}

template <class T>
typename T::Ptr PlanningPlayer::find_ros_msg_with_header_time(
    const std::string &topic, uint64_t time) {
  auto it_topic = header_cache_.find(topic);
  if (it_topic != header_cache_.end()) {
    auto it_time = it_topic->second.find(time);
    if (it_time != it_topic->second.end()) {
      // typename T::Ptr msg = it_time->second->instantiate<T>();
      typename T::Ptr msg = boost::any_cast<typename T::Ptr>(it_time->second);

      return msg;
    }
  }
  return nullptr;
}

}  // namespace planning_player
}  // namespace planning