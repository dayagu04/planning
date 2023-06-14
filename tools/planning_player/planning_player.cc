#include "planning_player.h"

#include <cyber/record/record_reader.h>
#include <cyber/record/record_viewer.h>
#include <cyber/record/record_writer.h>
#include <google/protobuf/message.h>

#include <cstdint>
#include <fstream>
#include <ios>
#include <memory>

#include "general_planning.h"

#define CACHE_WITH_HEADER_TIME(T)        \
  auto obj_msg = std::make_shared<T>();  \
  obj_msg->ParseFromString(msg.content); \
  msg_cache_[msg.channel_name][obj_msg->header().timestamp()] = obj_msg;  // us

#define CACHE_WITH_MSG_TIME(T)           \
  auto obj_msg = std::make_shared<T>();  \
  obj_msg->ParseFromString(msg.content); \
  msg_cache_[msg.channel_name][msg.time / 1000] = obj_msg;  // us

namespace planning {
namespace planning_player {

void PlanningPlayer::Init() {
  planning_adapter_ = std::make_unique<PlanningAdapter>();
  planning_adapter_->Init();

  // -------------- writter topics --------------
  planning_adapter_->RegisterOutputWriter([this](const PlanningOutput::PlanningOutput& planning_output) {
    msg_cache_["/iflytek/planning/plan"][planning_timestamp_us_] =
        std::make_shared<PlanningOutput::PlanningOutput>(planning_output);
  });

  planning_adapter_->RegisterDebugInfoWriter([this](const planning::common::PlanningDebugInfo& planning_debug_info) {
    msg_cache_["/iflytek/planning/debug_info"][planning_timestamp_us_] =
        std::make_shared<planning::common::PlanningDebugInfo>(planning_debug_info);
  });

  planning_adapter_->RegisterHMIOutputInfoWriter(
      [this](const PlanningHMI::PlanningHMIOutputInfoStr& planning_hmi_ouput_info) {
        msg_cache_["/iflytek/planning/hmi"][planning_timestamp_us_] =
            std::make_shared<PlanningHMI::PlanningHMIOutputInfoStr>(planning_hmi_ouput_info);
      });
}

void PlanningPlayer::Clear() {
  msg_cache_.clear();
  proto_desc_map_.clear();
  planning_timestamp_us_ = 0;
}

bool PlanningPlayer::LoadCyberBag(const std::string& bag_path) {
  std::fstream file;
  file.open(bag_path, std::ios_base::in);
  if (!file) {
    std::cerr << "open bag failed" << bag_path << std::endl;
    return false;
  }

  std::shared_ptr<apollo::cyber::record::RecordReader> record_reader =
      std::make_shared<apollo::cyber::record::RecordReader>(bag_path);
  std::shared_ptr<apollo::cyber::record::RecordViewer> record_viewer =
      std::make_shared<apollo::cyber::record::RecordViewer>(record_reader);

  std::cout << record_reader->GetFile() << " channel_list:" << std::endl;
  for (const auto& channel_name : record_reader->GetChannelList()) {
    std::cout << channel_name << " number:" << record_reader->GetMessageNumber(channel_name)
              << " type:" << record_reader->GetMessageType(channel_name) << std::endl;
    proto_desc_map_[channel_name] = record_reader->GetProtoDesc(channel_name);
  }

  for (const auto& msg : *record_viewer) {
    if (msg.channel_name == "/iflytek/fusion/objects") {
      CACHE_WITH_HEADER_TIME(FusionObjects::FusionObjectsInfo);
    } else if (msg.channel_name == "/iflytek/fusion/road_fusion") {
      CACHE_WITH_HEADER_TIME(FusionRoad::RoadInfo);
    } else if (msg.channel_name == "/iflytek/localization/ego_pose") {
      CACHE_WITH_HEADER_TIME(LocalizationOutput::LocalizationEstimate);
    } else if (msg.channel_name == "/iflytek/prediction/prediction_result") {
      CACHE_WITH_HEADER_TIME(Prediction::PredictionResult);
    } else if (msg.channel_name == "/iflytek/vehicle_service") {
      CACHE_WITH_HEADER_TIME(VehicleService::VehicleServiceOutputInfo);
    } else if (msg.channel_name == "/iflytek/radar_perception_info") {
      CACHE_WITH_HEADER_TIME(RadarPerceptionObjects::RadarPerceptionObjectsInfo);
    } else if (msg.channel_name == "/iflytek/control/control_command") {
      CACHE_WITH_HEADER_TIME(ControlCommand::ControlOutput);
    } else if (msg.channel_name == "/iflytek/hmi/mcu_inner") {
      CACHE_WITH_HEADER_TIME(HmiMcuInner::HmiMcuInner);
    } else if (msg.channel_name == "/iflytek/fusion/parking_slot") {
      CACHE_WITH_HEADER_TIME(ParkingFusion::ParkingFusionInfo);
    } else if (msg.channel_name == "/iflytek/system_state/soc_state") {
      CACHE_WITH_HEADER_TIME(FuncStateMachine::FuncStateMachine);
    } else if (msg.channel_name == "/iflytek/planning/plan") {
      CACHE_WITH_MSG_TIME(PlanningOutput::PlanningOutput);
    } else if (msg.channel_name == "/iflytek/planning/debug_info") {
      CACHE_WITH_MSG_TIME(planning::common::PlanningDebugInfo);
    } else if (msg.channel_name == "/iflytek/planning/hmi") {
      CACHE_WITH_HEADER_TIME(PlanningHMI::PlanningHMIOutputInfoStr);
    } else {
      // std::cerr << "unsupported channel:" << msg.channel_name << std::endl;
    }
  }
  return true;
}

template <class T>
void write_topic_msg(TopicMsgCache& msg_cache, std::map<std::string, std::string>& proto_desc_map,
                     apollo::cyber::record::RecordWriter& record_writer, const std::string& topic_name) {
  if (msg_cache.find(topic_name) == msg_cache.end()) {
    std::cerr << "topic not found:" << topic_name << std::endl;
    return;
  }
  for (const auto& it_msg : msg_cache[topic_name]) {
    if (!record_writer.WriteMessage(topic_name, *std::dynamic_pointer_cast<T>(it_msg.second), it_msg.first * 1000,
                                    proto_desc_map[topic_name])) {
      std::cerr << "write msg failed: " << topic_name << std::endl;
      return;
    }
  }
  std::cout << "writed " << topic_name << " num:" << record_writer.GetMessageNumber(topic_name) << std::endl;
}

void PlanningPlayer::StoreCyberBag(const std::string& bag_path) {
  apollo::cyber::record::RecordWriter record_writer;
  record_writer.SetSizeOfFileSegmentation(0);
  record_writer.SetIntervalOfFileSegmentation(0);
  if (!record_writer.Open(bag_path)) {
    std::cerr << "open writer file failed: " << bag_path << std::endl;
    return;
  }
  write_topic_msg<FusionObjects::FusionObjectsInfo>(msg_cache_, proto_desc_map_, record_writer,
                                                    "/iflytek/fusion/objects");
  write_topic_msg<FusionRoad::RoadInfo>(msg_cache_, proto_desc_map_, record_writer, "/iflytek/fusion/road_fusion");
  write_topic_msg<LocalizationOutput::LocalizationEstimate>(msg_cache_, proto_desc_map_, record_writer,
                                                            "/iflytek/localization/ego_pose");
  write_topic_msg<Prediction::PredictionResult>(msg_cache_, proto_desc_map_, record_writer,
                                                "/iflytek/prediction/prediction_result");
  write_topic_msg<VehicleService::VehicleServiceOutputInfo>(msg_cache_, proto_desc_map_, record_writer,
                                                            "/iflytek/vehicle_service");
  write_topic_msg<RadarPerceptionObjects::RadarPerceptionObjectsInfo>(msg_cache_, proto_desc_map_, record_writer,
                                                                      "/iflytek/radar_perception_info");
  write_topic_msg<ControlCommand::ControlOutput>(msg_cache_, proto_desc_map_, record_writer,
                                                 "/iflytek/control/control_command");
  write_topic_msg<HmiMcuInner::HmiMcuInner>(msg_cache_, proto_desc_map_, record_writer, "/iflytek/hmi/mcu_inner");
  write_topic_msg<ParkingFusion::ParkingFusionInfo>(msg_cache_, proto_desc_map_, record_writer,
                                                    "/iflytek/fusion/parking_slot");
  write_topic_msg<FuncStateMachine::FuncStateMachine>(msg_cache_, proto_desc_map_, record_writer,
                                                      "/iflytek/system_state/soc_state");
  write_topic_msg<PlanningOutput::PlanningOutput>(msg_cache_, proto_desc_map_, record_writer, "/iflytek/planning/plan");
  write_topic_msg<planning::common::PlanningDebugInfo>(msg_cache_, proto_desc_map_, record_writer,
                                                       "/iflytek/planning/debug_info");
  write_topic_msg<PlanningHMI::PlanningHMIOutputInfoStr>(msg_cache_, proto_desc_map_, record_writer,
                                                         "/iflytek/planning/hmi");

  std::cout << "write bag:" << record_writer.GetFile() << std::endl;
  record_writer.Close();
}

template <class T>
std::shared_ptr<T> find_topic_msg(const TopicMsgCache& msg_cache, const std::string& topic, uint64_t time) {
  auto it_topic = msg_cache.find(topic);
  if (it_topic != msg_cache.end()) {
    auto it_time = it_topic->second.find(time);
    if (it_time != it_topic->second.end()) {
      return std::dynamic_pointer_cast<T>(it_time->second);
    }
  }
  return nullptr;
}

void PlanningPlayer::PlayOneFrame(int frame_num, const planning::common::TopicTimeList& input_time_list) {
  std::cout << "==================== frame " << frame_num << "====================\n"
            << input_time_list.DebugString() << std::endl;

  auto fusion_object_msg = find_topic_msg<FusionObjects::FusionObjectsInfo>(msg_cache_, "/iflytek/fusion/objects",
                                                                            input_time_list.fusion_object());
  if (fusion_object_msg) {
    planning_adapter_->FeedFusionObjects(fusion_object_msg);
  }

  auto fusion_road_msg =
      find_topic_msg<FusionRoad::RoadInfo>(msg_cache_, "/iflytek/fusion/road_fusion", input_time_list.fusion_road());
  if (fusion_road_msg) {
    planning_adapter_->FeedFusionRoad(fusion_road_msg);
  }

  auto localization_msg = find_topic_msg<LocalizationOutput::LocalizationEstimate>(
      msg_cache_, "/iflytek/localization/ego_pose", input_time_list.localization());
  if (localization_msg) {
    planning_adapter_->FeedLocalizationOutput(localization_msg);
  }

  auto prediction_msg = find_topic_msg<Prediction::PredictionResult>(
      msg_cache_, "/iflytek/prediction/prediction_result", input_time_list.prediction());
  if (prediction_msg) {
    planning_adapter_->FeedPredictionResult(prediction_msg);
  }

  auto vehicle_service_msg = find_topic_msg<VehicleService::VehicleServiceOutputInfo>(
      msg_cache_, "/iflytek/vehicle_service", input_time_list.vehicle_service());
  if (vehicle_service_msg) {
    planning_adapter_->FeedVehicleService(vehicle_service_msg);
  }

  auto radar_perception_msg = find_topic_msg<RadarPerceptionObjects::RadarPerceptionObjectsInfo>(
      msg_cache_, "/iflytek/radar_perception_info", input_time_list.radar_perception());
  if (radar_perception_msg) {
    planning_adapter_->FeedRadarPerceptionObjects(radar_perception_msg);
  }

  auto control_output_msg = find_topic_msg<ControlCommand::ControlOutput>(
      msg_cache_, "/iflytek/control/control_command", input_time_list.control_output());
  if (control_output_msg) {
    planning_adapter_->FeedControlCommand(control_output_msg);
  }

  auto hmi_mcu_msg =
      find_topic_msg<HmiMcuInner::HmiMcuInner>(msg_cache_, "/iflytek/hmi/mcu_inner", input_time_list.hmi());
  if (hmi_mcu_msg) {
    planning_adapter_->FeedHmiMcuInner(hmi_mcu_msg);
  }

  if (input_time_list.has_function_state_machine()) {
    auto func_state_machine_msg = find_topic_msg<FuncStateMachine::FuncStateMachine>(
        msg_cache_, "/iflytek/system_state/soc_state", input_time_list.function_state_machine());
    if (func_state_machine_msg) {
      planning_adapter_->FeedFuncStateMachine(func_state_machine_msg);
    }
  }

  planning_adapter_->Proc();
}

void PlanningPlayer::PlayAllFrames() {
  auto& debug_info_cache = msg_cache_["/iflytek/planning/debug_info"];
  int frame_num = 0;
  for (auto it : debug_info_cache) {
    auto debug_info = std::dynamic_pointer_cast<planning::common::PlanningDebugInfo>(it.second);
    if (!debug_info->has_input_topic_timestamp()) {
      std::cerr << "no input topic timestamp" << std::endl;
      continue;
    }
    planning_timestamp_us_ = it.first;
    PlayOneFrame(frame_num++, debug_info->input_topic_timestamp());
  }
}

}  // namespace planning_player
}  // namespace planning