#pragma once

#include <cyber/record/record_message.h>
#include <google/protobuf/message.h>
#include <cstdint>
#include "planning_adapter.h"

namespace planning {
namespace planning_player {

using TopicMsgCache = std::map<std::string, std::map<uint64_t, std::shared_ptr<google::protobuf::Message>>>;

class PlanningPlayer {
 public:
  PlanningPlayer() = default;
  ~PlanningPlayer() = default;

  void Init();
  void Clear();
  bool LoadCyberBag(const std::string &bag_path);
  void StoreCyberBag(const std::string &bag_path);
  void PlayOneFrame(int frame_num, const planning::common::TopicTimeList &input_time_list);
  void PlayAllFrames();

 private:
  // std::shared_ptr<ADSNode> planning_node_ = nullptr;
  // std::shared_ptr<Writer<PlanningOutput::PlanningOutput>> planning_writer_ = nullptr;
  // std::shared_ptr<Writer<planning::common::PlanningDebugInfo>> planning_debug_writer_ = nullptr;
  // std::shared_ptr<Writer<PlanningHMI::PlanningHMIOutputInfoStr>> planning_hmi_Info_writer_ = nullptr;

  std::unique_ptr<PlanningAdapter> planning_adapter_ = nullptr;
  TopicMsgCache msg_cache_{};
  TopicMsgCache output_msg_cache_{};
  std::map<std::string, std::string> proto_desc_map_{};
  uint64_t planning_timestamp_us_ = 0;
};

}  // namespace planning_player
}  // namespace planning