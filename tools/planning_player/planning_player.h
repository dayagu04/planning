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
  PlanningPlayer() = default;
  ~PlanningPlayer() = default;

  void Init();
  void Clear();
  bool LoadCyberBag(const std::string &bag_path);
  void StoreCyberBag(const std::string &bag_path);
  void PlayOneFrame(int frame_num,
                    const planning::common::TopicTimeList &input_time_list);
  void PlayAllFrames();

 private:
  std::unique_ptr<PlanningAdapter> planning_adapter_ = nullptr;
  TopicMsgCache header_cache_{};      // msg cache indexed by header time(us)
  TopicMsgCache msg_cache_{};         // msg cache indexed by msg time(us)
  TopicMsgCache output_msg_cache_{};  // output cache indexed by msg time(us)
  std::map<std::string, std::string> proto_desc_map_{};
  uint64_t planning_msg_time_us_ = 0;
  uint64_t planning_header_time_us_ = 0;

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
  std::shared_ptr<T> find_msg_with_header_time(const std::string &topic,
                                               uint64_t time);
};

template <class T>
void PlanningPlayer::cache_with_msg_time(
    const apollo::cyber::record::RecordMessage &msg) {
  auto obj_msg = std::make_shared<T>();
  obj_msg->ParseFromString(msg.content);
  msg_cache_[msg.channel_name][msg.time / 1000] = obj_msg;  // us
}

template <class T>
void PlanningPlayer::cache_with_msg_and_header_time(
    const apollo::cyber::record::RecordMessage &msg) {
  auto obj_msg = std::make_shared<T>();
  obj_msg->ParseFromString(msg.content);
  msg_cache_[msg.channel_name][msg.time / 1000] = obj_msg;  // us
  header_cache_[msg.channel_name][obj_msg->header().timestamp()] =
      obj_msg;  // us
}

template <class T>
void PlanningPlayer::write_topic_msg(
    TopicMsgCache &msg_cache,
    apollo::cyber::record::RecordWriter &record_writer,
    const std::string &topic_name) {
  if (msg_cache.find(topic_name) == msg_cache.end()) {
    std::cerr << "topic not found:" << topic_name << std::endl;
    return;
  }
  if (msg_cache[topic_name].empty()) {
    std::cerr << "no msg in topic:" << topic_name << std::endl;
    return;
  }
  for (const auto &it_msg : msg_cache[topic_name]) {
    if (!record_writer.WriteMessage(
            topic_name, *std::dynamic_pointer_cast<T>(it_msg.second),
            it_msg.first * 1000, proto_desc_map_[topic_name])) {
      std::cerr << "write msg failed: " << topic_name << std::endl;
      return;
    }
  }
  std::cout << "writed " << topic_name
            << " num:" << record_writer.GetMessageNumber(topic_name)
            << " timespan:"
            << (msg_cache[topic_name].end()--)->first -
                   msg_cache[topic_name].begin()->first
            << std::endl;
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