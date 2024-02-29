#pragma once

#include <cyber/record/record_message.h>
#include <cyber/record/record_writer.h>
#include <google/protobuf/message.h>

#include <cstdint>

#include "planning_adapter.h"

namespace planning {
namespace planning_player {

using TopicMsgCache =
    std::map<std::string,
             std::map<uint64_t, std::shared_ptr<google::protobuf::Message>>>;

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
  bool LoadCyberBag(const std::string &bag_path);
  void StoreCyberBag(const std::string &bag_path);
  void PlayOneFrame(int frame_num,
                    const planning::common::TopicTimeList &input_time_list);
  void PlayAllFrames();
  void RunCloseLoop(const PlanningOutput::PlanningOutput &planning_output);
  void PerfectControlHPP(
      const PlanningOutput::PlanningOutput &plan_msg, uint64_t delta_t,
      std::shared_ptr<IFLYLocalization::IFLYLocalization> &loc_msg);
  void PerfectControlSCC(
      const PlanningOutput::PlanningOutput &plan_msg, uint64_t delta_t,
      std::shared_ptr<LocalizationOutput::LocalizationEstimate> &loc_msg);
  void PerfectControlAPA(
      const PlanningOutput::PlanningOutput &plan_msg, uint64_t delta_t,
      std::shared_ptr<LocalizationOutput::LocalizationEstimate> &loc_msg);

 private:
  DynamicState state_;
  std::unique_ptr<PlanningAdapter> planning_adapter_ = nullptr;
  TopicMsgCache header_cache_{};      // msg cache indexed by header time(us)
  TopicMsgCache msg_cache_{};         // msg cache indexed by msg time(ns)
  TopicMsgCache output_msg_cache_{};  // output cache indexed by msg time(ns)
  std::map<uint64_t, std::shared_ptr<google::protobuf::Message>>
      msg_cache_ordered_by_time_;
  std::map<std::string, std::string> proto_desc_map_{};
  uint64_t planning_msg_time_ns_ = 0;
  uint64_t planning_header_time_us_ = 0;
  int frame_num_ = 0;
  uint64_t planning_dubug_info_msg_time_ns_ = 0;
  uint64_t planning_dubug_info_header_time_us_ = 0;
  uint64_t planning_hmi_msg_time_ns_ = 0;
  uint64_t planning_hmi_header_time_us_ = 0;
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
  void cache_with_msg_time(const apollo::cyber::record::RecordMessage &msg);

  template <class T>
  void cache_with_msg_and_header_time(
      const apollo::cyber::record::RecordMessage &msg);

  template <class T>
  void write_topic_msg(TopicMsgCache &msg_cache,
                       apollo::cyber::record::RecordWriter &record_writer,
                       const std::string &topic_name);

  template <class T>
  void write_msg(
      const std::pair<const unsigned long,
                      std::shared_ptr<google::protobuf::Message>> &msg,
      apollo::cyber::record::RecordWriter &record_writer,
      const std::string &topic_name);

  template <class T>
  std::shared_ptr<T> find_msg_with_header_time(const std::string &topic,
                                               uint64_t time);

  inline const std::string &get_proto_desc(const google::protobuf::Message &msg,
                                           const std::string &topic) {
    if (proto_desc_map_[topic].empty()) {
      apollo::cyber::message::ProtobufFactory::GetDescriptorString(
          msg, &proto_desc_map_[topic]);
    }
    return proto_desc_map_[topic];
  }

  inline bool check_msg_exist(TopicMsgCache &msg_cache,
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
};

template <class T>
void PlanningPlayer::cache_with_msg_time(
    const apollo::cyber::record::RecordMessage &msg) {
  auto obj_msg = std::make_shared<T>();
  obj_msg->ParseFromString(msg.content);
  msg_cache_[msg.channel_name][msg.time] = obj_msg;  // ns
}

template <class T>
void PlanningPlayer::cache_with_msg_and_header_time(
    const apollo::cyber::record::RecordMessage &msg) {
  auto obj_msg = std::make_shared<T>();
  obj_msg->ParseFromString(msg.content);
  msg_cache_[msg.channel_name][msg.time] = obj_msg;  // ns
  header_cache_[msg.channel_name][obj_msg->header().timestamp()] =
      obj_msg;  // us
}

template <class T>
void PlanningPlayer::write_topic_msg(
    TopicMsgCache &msg_cache,
    apollo::cyber::record::RecordWriter &record_writer,
    const std::string &topic_name) {
  if (!check_msg_exist(msg_cache, topic_name)) {
    return;
  }
  for (const auto &it_msg : msg_cache[topic_name]) {
    if (!it_msg.second) {
      continue;
    }
    if (!record_writer.WriteMessage(
            topic_name, *std::dynamic_pointer_cast<T>(it_msg.second),
            it_msg.first, get_proto_desc(*it_msg.second, topic_name))) {
      std::cerr << "write msg failed: " << topic_name << std::endl;
      return;
    }
  }
  std::cout << "writed " << topic_name
            << " num:" << record_writer.GetMessageNumber(topic_name)
            << " timespan:"
            << (msg_cache[topic_name].rbegin())->first -
                   msg_cache[topic_name].begin()->first
            << std::endl;
}

template <class T>
void PlanningPlayer::write_msg(
    const std::pair<const unsigned long,
                    std::shared_ptr<google::protobuf::Message>> &msg,
    apollo::cyber::record::RecordWriter &record_writer,
    const std::string &topic_name) {
  if (!msg.second) {
    return;
  }
  if (!record_writer.WriteMessage(
          topic_name, *std::dynamic_pointer_cast<T>(msg.second), msg.first,
          get_proto_desc(*msg.second, topic_name))) {
    std::cerr << "write msg failed: " << topic_name << std::endl;
    return;
  }
}

template <class T>
std::shared_ptr<T> PlanningPlayer::find_msg_with_header_time(
    const std::string &topic, uint64_t time) {
  auto it_topic = header_cache_.find(topic);
  if (it_topic != header_cache_.end()) {
    auto it_time = it_topic->second.find(time);
    if (it_time != it_topic->second.end()) {
      auto msg = std::dynamic_pointer_cast<T>(it_time->second);
      return msg;
    }
  }
  return nullptr;
}

}  // namespace planning_player
}  // namespace planning