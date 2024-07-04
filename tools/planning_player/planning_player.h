#pragma once

#include <google/protobuf/message.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/any.hpp>
#include <cstdint>
#include <memory>

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
using PlanningHmiMsgCache =
    std::map<std::string,
             std::map<ros::Time, struct_msgs::PlanningHMIOutputInfoStr>>;
using PlanningDebugMsgCache =
    std::map<std::string, std::map<ros::Time, sensor_interface::DebugInfo>>;

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
            const std::string &scene_type, bool no_debug);
  void Clear();
  bool LoadRosBag(const std::string &bag_path, const std::string &out_bag,
                  bool is_close_loop, bool no_debug);
  void StoreRosBag(const std::string &bag_path);
  void GenMileage(const std::string &mileage_path);
  void NoDebugInfoMode();
  void PlayOneFrame(int frame_num,
                    const planning::common::TopicTimeList &input_time_list);
  void PlayAllFrames();

  void RunCloseLoop(const struct_msgs::PlanningOutput &planning_output);
  void PerpareTrajectory(const struct_msgs::PlanningOutput &plan_msg);
  void PerfectControlHPP(uint64_t delta_t,
                         struct_msgs::IFLYLocalization::Ptr loc_msg);
  void PerfectControlSCC(uint64_t delta_t,
                         struct_msgs::LocalizationEstimate::Ptr loc_msg);
  void PerfectControlAPA(const struct_msgs::PlanningOutput &plan_msg,
                         uint64_t delta_t,
                         struct_msgs::LocalizationEstimate::Ptr loc_msg);
  void UpdateVehicleService(
      uint64_t delta_t,
      struct_msgs::VehicleServiceOutputInfo::Ptr vehi_svc_msg);
  void UpdateVehicleServiceData();

 private:
  DynamicState state_;
  std::unique_ptr<PlanningAdapter> planning_adapter_ = nullptr;
  TopicHeaderTimeCache header_cache_{};  // msg cache indexed by header time(us)
  TopicMsgTimeCache msg_cache_{};        // msg cache indexed by msg time(ns)
  PlanningMsgCache
      output_planning_msg_cache_{};  // output cache indexed by msg time(ns)
  PlanningHmiMsgCache
      output_planning_hmi_msg_cache_{};  // output cache indexed by msg time(ns)
  PlanningDebugMsgCache
      output_planning_debug_msg_cache_{};  // output cache indexed by msg
                                           // time(ns)
  std::map<uint64_t, boost::any> msg_cache_ordered_by_time_;
  std::map<std::string, std::string> proto_desc_map_{};
  ros::Time planning_msg_time_s_;
  uint64_t planning_header_time_us_ = 0;
  ros::Time planning_hmi_msg_time_ns_;
  uint64_t planning_hmi_header_time_us_ = 0;
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
  uint64_t next_vehi_svc_header_time_us_ = 0;
  uint64_t vehi_svc_header_time_us_ = 0;
  uint64_t planning_dubug_info_frame_num_ = 0;
  uint64_t local_time_ = 0;
  int frame_num_before_enter_auto_ = 0;
  std::string scene_type_ = "acc";
  iflyauto::FunctionalState last_functional_state =
      iflyauto::FunctionalState_INIT;
  pnc::mathlib::spline x_t_spline_;
  pnc::mathlib::spline y_t_spline_;
  pnc::mathlib::spline theta_t_spline_;
  pnc::mathlib::spline v_t_spline_;
  pnc::mathlib::spline a_t_spline_;
  pnc::mathlib::spline yaw_rate_t_spline_;
  pnc::mathlib::spline curvature_t_spline_;

  template <class T>
  void cache_with_ros_msg_time(const rosbag::MessageInstance &msg);

  template <class T>
  void cache_with_ros_msg_and_header_time(const rosbag::MessageInstance &msg);

  template <class T>
  void cache_with_ros_msg_and_header_time_local(
      const rosbag::MessageInstance &msg, rosbag::Bag &new_bag,
      bool is_close_loop);

  template <class T>
  void write_ros_msg(const std::map<ros::Time, boost::any> &write_msg,
                     const std::string &topic_name, rosbag::Bag &bag);

  template <class T>
  typename T::Ptr find_ros_msg_with_header_time(const std::string &topic,
                                                uint64_t time);

  template <class T>
  typename T::Ptr find_ros_msg_with_header_time_upper_bound(
      const std::string &topic, uint64_t time);

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
    std::cout << "Error !!!!!!!!!! Incorrect interface version" << std::endl
              << "msg instantiate error, msg name: " << msg.getTopic()
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
    std::cout << "Error !!!!!!!!!! Incorrect interface version" << std::endl
              << "msg instantiate error, msg name: " << msg.getTopic()
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
void PlanningPlayer::cache_with_ros_msg_and_header_time_local(
    const rosbag::MessageInstance &msg, rosbag::Bag &new_bag,
    bool is_close_loop) {
  typename T::Ptr obj_msg = msg.instantiate<T>();
  if (obj_msg == nullptr) {
    std::cout << "Error !!!!!!!!!! Incorrect interface version" << std::endl
              << "msg instantiate error, msg name: " << msg.getTopic()
              << std::endl;
  } else {
    // auto time = msg.getTime();
    // uint64_t time_in_ns = time.sec * 1000000000ULL + time.nsec;
    msg_cache_[msg.getTopic()][msg.getTime()] = obj_msg;  // ns
    header_cache_[msg.getTopic()][obj_msg->msg_header.timestamp] =
        obj_msg;  // us
    if (is_close_loop) {
      auto origin_topic = msg.getTopic() + "_origin";
      new_bag.write(origin_topic, msg.getTime(), obj_msg);
    }
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

template <class T>
typename T::Ptr PlanningPlayer::find_ros_msg_with_header_time_upper_bound(
    const std::string &topic, uint64_t time) {
  auto it_topic = header_cache_.find(topic);
  if (it_topic != header_cache_.end()) {
    auto it_time = it_topic->second.lower_bound(time);
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