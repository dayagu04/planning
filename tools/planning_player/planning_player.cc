#include "planning_player.h"

#include <cyber/record/record_message.h>
#include <cyber/record/record_reader.h>
#include <cyber/record/record_viewer.h>
#include <cyber/record/record_writer.h>
#include <google/protobuf/message.h>

#include <cstdint>
#include <fstream>
#include <ios>
#include <memory>

#include "general_planning.h"

namespace planning {
namespace planning_player {

static constexpr auto TOPIC_PLANNING_PLAN = "/iflytek/planning/plan";
static constexpr auto TOPIC_PLANNING_DEBUG_INFO = "/iflytek/planning/debug_info";
static constexpr auto TOPIC_PLANNING_HMI = "/iflytek/planning/hmi";
static constexpr auto TOPIC_FUSION_OBJECTS = "/iflytek/fusion/objects";
static constexpr auto TOPIC_ROAD_FUSION = "/iflytek/fusion/road_fusion";
static constexpr auto TOPIC_LOCALIZATION = "/iflytek/localization/ego_pose";
static constexpr auto TOPIC_PREDICTION_RESULT = "/iflytek/prediction/prediction_result";
static constexpr auto TOPIC_VEHICLE_SERVICE = "/iflytek/vehicle_service";
static constexpr auto TOPIC_CONTROL_COMMAN = "/iflytek/control/control_command";
static constexpr auto TOPIC_HMI_MCU_INNER = "/iflytek/hmi/mcu_inner";
static constexpr auto TOPIC_PARKING_FUSION = "/iflytek/fusion/parking_slot";
static constexpr auto TOPIC_FUNC_STATE_MACHINE = "/iflytek/system_state/soc_state";

void PlanningPlayer::Init() {
  planning_adapter_ = std::make_unique<PlanningAdapter>();
  planning_adapter_->Init();

  planning_adapter_->RegisterOutputWriter([this](const PlanningOutput::PlanningOutput& planning_output) {
    auto planning_output_ptr = std::make_shared<PlanningOutput::PlanningOutput>(planning_output);
    planning_output_ptr->mutable_meta()->mutable_header()->set_timestamp(planning_header_time_us_);
    output_msg_cache_[TOPIC_PLANNING_PLAN][planning_msg_time_us_] = planning_output_ptr;
  });

  planning_adapter_->RegisterDebugInfoWriter([this](const planning::common::PlanningDebugInfo& planning_debug_info) {
    auto planning_debug_info_ptr = std::make_shared<planning::common::PlanningDebugInfo>(planning_debug_info);
    planning_debug_info_ptr->set_timestamp(planning_header_time_us_);
    output_msg_cache_[TOPIC_PLANNING_DEBUG_INFO][planning_msg_time_us_] = planning_debug_info_ptr;
  });

  planning_adapter_->RegisterHMIOutputInfoWriter(
      [this](const PlanningHMI::PlanningHMIOutputInfoStr& planning_hmi_ouput_info) {
        output_msg_cache_[TOPIC_PLANNING_HMI][planning_msg_time_us_] =
            std::make_shared<PlanningHMI::PlanningHMIOutputInfoStr>(planning_hmi_ouput_info);
      });
}

void PlanningPlayer::Clear() {
  msg_cache_.clear();
  header_cache_.clear();
  output_msg_cache_.clear();
  proto_desc_map_.clear();
  planning_msg_time_us_ = 0;
  planning_header_time_us_ = 0;
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
    if (msg.channel_name == TOPIC_FUSION_OBJECTS) {
      cache_with_msg_and_header_time<FusionObjects::FusionObjectsInfo>(msg);
    } else if (msg.channel_name == TOPIC_ROAD_FUSION) {
      cache_with_msg_and_header_time<FusionRoad::RoadInfo>(msg);
    } else if (msg.channel_name == TOPIC_LOCALIZATION) {
      cache_with_msg_and_header_time<LocalizationOutput::LocalizationEstimate>(msg);
    } else if (msg.channel_name == TOPIC_PREDICTION_RESULT) {
      cache_with_msg_and_header_time<Prediction::PredictionResult>(msg);
    } else if (msg.channel_name == TOPIC_VEHICLE_SERVICE) {
      cache_with_msg_and_header_time<VehicleService::VehicleServiceOutputInfo>(msg);
    } else if (msg.channel_name == TOPIC_CONTROL_COMMAN) {
      cache_with_msg_and_header_time<ControlCommand::ControlOutput>(msg);
    } else if (msg.channel_name == TOPIC_HMI_MCU_INNER) {
      cache_with_msg_and_header_time<HmiMcuInner::HmiMcuInner>(msg);
    } else if (msg.channel_name == TOPIC_PARKING_FUSION) {
      cache_with_msg_and_header_time<ParkingFusion::ParkingFusionInfo>(msg);
    } else if (msg.channel_name == TOPIC_FUNC_STATE_MACHINE) {
      cache_with_msg_and_header_time<FuncStateMachine::FuncStateMachine>(msg);
    } else if (msg.channel_name == TOPIC_PLANNING_PLAN) {
      cache_with_msg_time<PlanningOutput::PlanningOutput>(msg);
    } else if (msg.channel_name == TOPIC_PLANNING_DEBUG_INFO) {
      cache_with_msg_time<planning::common::PlanningDebugInfo>(msg);
    } else if (msg.channel_name == TOPIC_PLANNING_HMI) {
      cache_with_msg_time<PlanningHMI::PlanningHMIOutputInfoStr>(msg);
    } else {
      // std::cerr << "unsupported channel:" << msg.channel_name << std::endl;
    }
  }
  return true;
}

void PlanningPlayer::StoreCyberBag(const std::string& bag_path) {
  apollo::cyber::record::RecordWriter record_writer;
  record_writer.SetSizeOfFileSegmentation(0);
  record_writer.SetIntervalOfFileSegmentation(0);
  if (!record_writer.Open(bag_path)) {
    std::cerr << "open writer file failed: " << bag_path << std::endl;
    return;
  }
  write_topic_msg<FusionObjects::FusionObjectsInfo>(msg_cache_, record_writer, TOPIC_FUSION_OBJECTS);
  write_topic_msg<FusionRoad::RoadInfo>(msg_cache_, record_writer, TOPIC_ROAD_FUSION);
  write_topic_msg<LocalizationOutput::LocalizationEstimate>(msg_cache_, record_writer, TOPIC_LOCALIZATION);
  write_topic_msg<Prediction::PredictionResult>(msg_cache_, record_writer, TOPIC_PREDICTION_RESULT);
  write_topic_msg<VehicleService::VehicleServiceOutputInfo>(msg_cache_, record_writer, TOPIC_VEHICLE_SERVICE);
  write_topic_msg<ControlCommand::ControlOutput>(msg_cache_, record_writer, TOPIC_CONTROL_COMMAN);
  write_topic_msg<HmiMcuInner::HmiMcuInner>(msg_cache_, record_writer, TOPIC_HMI_MCU_INNER);
  write_topic_msg<ParkingFusion::ParkingFusionInfo>(msg_cache_, record_writer, TOPIC_PARKING_FUSION);
  write_topic_msg<FuncStateMachine::FuncStateMachine>(msg_cache_, record_writer, TOPIC_FUNC_STATE_MACHINE);
  write_topic_msg<PlanningOutput::PlanningOutput>(output_msg_cache_, record_writer, TOPIC_PLANNING_PLAN);
  write_topic_msg<planning::common::PlanningDebugInfo>(output_msg_cache_, record_writer, TOPIC_PLANNING_DEBUG_INFO);
  write_topic_msg<PlanningHMI::PlanningHMIOutputInfoStr>(output_msg_cache_, record_writer, TOPIC_PLANNING_HMI);

  std::cout << "write bag:" << record_writer.GetFile() << std::endl;
  record_writer.Close();
}

void PlanningPlayer::PlayOneFrame(int frame_num, const planning::common::TopicTimeList& input_time_list) {
  std::cout << "==================== frame " << frame_num << "====================\n"
            << input_time_list.DebugString() << std::endl;

  auto fusion_object_msg = find_msg_with_header_time<FusionObjects::FusionObjectsInfo>(TOPIC_FUSION_OBJECTS,
                                                                                       input_time_list.fusion_object());
  if (fusion_object_msg) {
    planning_adapter_->FeedFusionObjects(fusion_object_msg);
  }

  auto fusion_road_msg =
      find_msg_with_header_time<FusionRoad::RoadInfo>(TOPIC_ROAD_FUSION, input_time_list.fusion_road());
  if (fusion_road_msg) {
    planning_adapter_->FeedFusionRoad(fusion_road_msg);
  }

  auto localization_msg = find_msg_with_header_time<LocalizationOutput::LocalizationEstimate>(
      TOPIC_LOCALIZATION, input_time_list.localization());
  if (localization_msg) {
    planning_adapter_->FeedLocalizationOutput(localization_msg);
  }

  auto prediction_msg =
      find_msg_with_header_time<Prediction::PredictionResult>(TOPIC_PREDICTION_RESULT, input_time_list.prediction());
  if (prediction_msg) {
    planning_adapter_->FeedPredictionResult(prediction_msg);
  }

  auto vehicle_service_msg = find_msg_with_header_time<VehicleService::VehicleServiceOutputInfo>(
      TOPIC_VEHICLE_SERVICE, input_time_list.vehicle_service());
  if (vehicle_service_msg) {
    planning_adapter_->FeedVehicleService(vehicle_service_msg);
  }

  auto control_output_msg =
      find_msg_with_header_time<ControlCommand::ControlOutput>(TOPIC_CONTROL_COMMAN, input_time_list.control_output());
  if (control_output_msg) {
    planning_adapter_->FeedControlCommand(control_output_msg);
  }

  auto hmi_mcu_msg = find_msg_with_header_time<HmiMcuInner::HmiMcuInner>(TOPIC_HMI_MCU_INNER, input_time_list.hmi());
  if (hmi_mcu_msg) {
    planning_adapter_->FeedHmiMcuInner(hmi_mcu_msg);
  }

  auto parking_fusion_msg =
      find_msg_with_header_time<ParkingFusion::ParkingFusionInfo>(TOPIC_PARKING_FUSION, input_time_list.hmi());
  if (parking_fusion_msg) {
    planning_adapter_->FeedParkingFusion(parking_fusion_msg);
  }

  if (input_time_list.has_function_state_machine()) {
    auto func_state_machine_msg = find_msg_with_header_time<FuncStateMachine::FuncStateMachine>(
        TOPIC_FUNC_STATE_MACHINE, input_time_list.function_state_machine());
    if (func_state_machine_msg) {
      planning_adapter_->FeedFuncStateMachine(func_state_machine_msg);
    }
  }

  planning_adapter_->Proc();
}

void PlanningPlayer::PlayAllFrames() {
  auto& debug_info_cache = msg_cache_[TOPIC_PLANNING_DEBUG_INFO];
  int frame_num = 0;
  for (auto it : debug_info_cache) {
    auto debug_info = std::dynamic_pointer_cast<planning::common::PlanningDebugInfo>(it.second);
    if (!debug_info->has_input_topic_timestamp()) {
      std::cerr << "no input topic timestamp" << std::endl;
      continue;
    }
    planning_header_time_us_ = debug_info->timestamp();
    planning_msg_time_us_ = it.first;
    PlayOneFrame(frame_num++, debug_info->input_topic_timestamp());
  }
}

}  // namespace planning_player
}  // namespace planning