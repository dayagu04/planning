#include "planning_player.h"

#include <cyber/record/record_message.h>
#include <cyber/record/record_reader.h>
#include <cyber/record/record_viewer.h>
#include <cyber/record/record_writer.h>
#include <google/protobuf/message.h>
#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <ios>
#include <memory>
#include <vector>

#include "func_state_machine.pb.h"
#include "general_planning.h"
#include "geometry_math.h"
#include "math_lib.h"
#include "planning_plan.pb.h"
#include "spline.h"
#include "transform_lib.h"

namespace planning {
namespace planning_player {

static constexpr auto TOPIC_PLANNING_PLAN = "/iflytek/planning/plan";
static constexpr auto TOPIC_PLANNING_DEBUG_INFO =
    "/iflytek/planning/debug_info";
static constexpr auto TOPIC_PLANNING_HMI = "/iflytek/planning/hmi";
static constexpr auto TOPIC_FUSION_OBJECTS = "/iflytek/fusion/objects";
static constexpr auto TOPIC_ROAD_FUSION = "/iflytek/fusion/road_fusion";
static constexpr auto TOPIC_LOCALIZATION_ESTIMATE =
    "/iflytek/localization/ego_pose";
static constexpr auto TOPIC_LOCALIZATION = "/iflytek/localization/ego_pose";
static constexpr auto TOPIC_PREDICTION_RESULT =
    "/iflytek/prediction/prediction_result";
static constexpr auto TOPIC_VEHICLE_SERVICE = "/iflytek/vehicle_service";
static constexpr auto TOPIC_CONTROL_COMMAN = "/iflytek/control/control_command";
static constexpr auto TOPIC_HMI_MCU_INNER = "/iflytek/hmi/mcu_inner";
static constexpr auto TOPIC_PARKING_FUSION = "/iflytek/fusion/parking_slot";
static constexpr auto TOPIC_FUNC_STATE_MACHINE =
    "/iflytek/system_state/soc_state";
static constexpr auto TOPIC_HD_MAP = "/iflytek/ehr/static_map";
static constexpr auto TOPIC_GROUND_LINE = "/iflytek/fusion/ground_line";
static constexpr auto TOPIC_EHR_PARKING_MAP = "/iflytek/ehr/parking_map";
void PlanningPlayer::Init(bool is_close_loop, double auto_time_sec,
                          const std::string& scene_type) {
  std::cout << "===========planning player init, is_close_loop="
            << is_close_loop << ", auto_time_sec=" << auto_time_sec
            << ", scene_type=" << scene_type << "==========" << std::endl;
  // 找到第几帧进自动
  if (scene_type == "scc") {
    for (const auto& it : msg_cache_[TOPIC_FUNC_STATE_MACHINE]) {
      auto fsm_msg =
          std::dynamic_pointer_cast<FuncStateMachine::FuncStateMachine>(
              it.second);
      auto current_state = fsm_msg->current_state();
      bool acc_mode =
          (current_state == FuncStateMachine::FunctionalState::ACC_ACTIVATE) ||
          (current_state ==
           FuncStateMachine::FunctionalState::ACC_STAND_ACTIVATE) ||
          (current_state ==
           FuncStateMachine::FunctionalState::ACC_STAND_WAIT) ||
          (current_state == FuncStateMachine::FunctionalState::ACC_OVERRIDE) ||
          (current_state == FuncStateMachine::FunctionalState::ACC_SECURE);
      bool scc_mode =
          (current_state == FuncStateMachine::FunctionalState::SCC_ACTIVATE) ||
          (current_state ==
           FuncStateMachine::FunctionalState::SCC_STAND_ACTIVATE) ||
          (current_state ==
           FuncStateMachine::FunctionalState::SCC_STAND_WAIT) ||
          (current_state == FuncStateMachine::FunctionalState::SCC_OVERRIDE) ||
          (current_state == FuncStateMachine::FunctionalState::SCC_SECURE);
      bool noa_mode =
          (current_state == FuncStateMachine::FunctionalState::NOA_ACTIVATE) ||
          (current_state == FuncStateMachine::FunctionalState::NOA_OVERRIDE) ||
          (current_state == FuncStateMachine::FunctionalState::NOA_SECUR);
      bool dbw_status = acc_mode || scc_mode || noa_mode;
      if (dbw_status == true) {
        break;
      } else {
        frame_num_before_enter_auto_++;
      }
    }
  } else if (scene_type == "apa") {
    for (const auto& it : msg_cache_[TOPIC_PLANNING_PLAN]) {
      auto plan_msg =
          std::dynamic_pointer_cast<PlanningOutput::PlanningOutput>(it.second);

      const auto apa_planning_status =
          plan_msg->planning_status().apa_planning_status();

      if (apa_planning_status ==
          PlanningOutput::ApaPlanningStatus::IN_PROGRESS) {
        break;
      } else {
        frame_num_before_enter_auto_++;
      }
    }
  }

  if (auto_time_sec > 1.0) {
    frame_num_before_enter_auto_ = static_cast<int>(auto_time_sec * 10);
  }

  scene_type_ = scene_type;
  planning_adapter_ = std::make_unique<PlanningAdapter>();
  planning_adapter_->Init();

  planning_adapter_->RegisterOutputWriter(
      [this,
       is_close_loop](const PlanningOutput::PlanningOutput& planning_output) {
        auto planning_output_ptr =
            std::make_shared<PlanningOutput::PlanningOutput>(planning_output);
        planning_output_ptr->mutable_meta()->mutable_header()->set_timestamp(
            planning_header_time_us_);
        output_msg_cache_[TOPIC_PLANNING_PLAN][planning_msg_time_ns_] =
            planning_output_ptr;
        // 进自动一段时间后（默认1.5s）再改变车辆位置，为的是准备好相关的输入topic
        // 假设录包时刻是T0，那T0时刻的planning用到的相关输入信息一定是T0时刻之前的
        // 而这些信息不在此包中
        if (is_close_loop && frame_num_ > (frame_num_before_enter_auto_ + 15)) {
          RunCloseLoop(planning_output);
        }
      });

  planning_adapter_->RegisterDebugInfoWriter(
      [this](const planning::common::PlanningDebugInfo& planning_debug_info) {
        auto planning_debug_info_ptr =
            std::make_shared<planning::common::PlanningDebugInfo>(
                planning_debug_info);
        planning_debug_info_ptr->set_timestamp(
            planning_dubug_info_header_time_us_);
        output_msg_cache_[TOPIC_PLANNING_DEBUG_INFO]
                         [planning_dubug_info_msg_time_ns_] =
                             planning_debug_info_ptr;
      });

  planning_adapter_->RegisterHMIOutputInfoWriter(
      [this](const PlanningHMI::PlanningHMIOutputInfoStr&
                 planning_hmi_ouput_info) {
        auto planning_hmi_ouput_info_ptr =
            std::make_shared<PlanningHMI::PlanningHMIOutputInfoStr>(
                planning_hmi_ouput_info);
        planning_hmi_ouput_info_ptr->mutable_header()->set_timestamp(
            planning_hmi_header_time_us_);
        output_msg_cache_[TOPIC_PLANNING_HMI][planning_hmi_msg_time_ns_] =
            planning_hmi_ouput_info_ptr;
      });
}

void PlanningPlayer::Clear() {
  msg_cache_.clear();
  header_cache_.clear();
  output_msg_cache_.clear();
  proto_desc_map_.clear();
  planning_msg_time_ns_ = 0;
  planning_header_time_us_ = 0;
  loc_header_time_us_ = 0;
  frame_num_before_enter_auto_ = 0;
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
    std::cout << channel_name
              << " number:" << record_reader->GetMessageNumber(channel_name)
              << " type:" << record_reader->GetMessageType(channel_name)
              << std::endl;
  }

  for (const auto& msg : *record_viewer) {
    if (msg.channel_name == TOPIC_FUSION_OBJECTS) {
      cache_with_msg_and_header_time<FusionObjects::FusionObjectsInfo>(msg);
    } else if (msg.channel_name == TOPIC_ROAD_FUSION) {
      cache_with_msg_and_header_time<FusionRoad::RoadInfo>(msg);
    } else if (msg.channel_name == TOPIC_LOCALIZATION_ESTIMATE) {
      cache_with_msg_and_header_time<LocalizationOutput::LocalizationEstimate>(
          msg);
    } else if (msg.channel_name == TOPIC_PREDICTION_RESULT) {
      cache_with_msg_and_header_time<Prediction::PredictionResult>(msg);
    } else if (msg.channel_name == TOPIC_VEHICLE_SERVICE) {
      cache_with_msg_and_header_time<VehicleService::VehicleServiceOutputInfo>(
          msg);
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
    } else if (msg.channel_name == TOPIC_HD_MAP) {
      cache_with_msg_and_header_time<Map::StaticMap>(msg);
    } else {
      // std::cerr << "unsupported channel:" << msg.channel_name << std::endl;
    }
  }
  return true;
}

void PlanningPlayer::StoreCyberBag_old(const std::string& bag_path) {
  apollo::cyber::record::RecordWriter record_writer;
  record_writer.SetSizeOfFileSegmentation(0);
  record_writer.SetIntervalOfFileSegmentation(0);
  if (!record_writer.Open(bag_path)) {
    std::cerr << "open writer file failed: " << bag_path << std::endl;
    return;
  }
  write_topic_msg<FusionObjects::FusionObjectsInfo>(msg_cache_, record_writer,
                                                    TOPIC_FUSION_OBJECTS);
  write_topic_msg<FusionRoad::RoadInfo>(msg_cache_, record_writer,
                                        TOPIC_ROAD_FUSION);
  write_topic_msg<LocalizationOutput::LocalizationEstimate>(
      msg_cache_, record_writer, TOPIC_LOCALIZATION_ESTIMATE);
  write_topic_msg<Prediction::PredictionResult>(msg_cache_, record_writer,
                                                TOPIC_PREDICTION_RESULT);
  write_topic_msg<VehicleService::VehicleServiceOutputInfo>(
      msg_cache_, record_writer, TOPIC_VEHICLE_SERVICE);
  write_topic_msg<ControlCommand::ControlOutput>(msg_cache_, record_writer,
                                                 TOPIC_CONTROL_COMMAN);
  write_topic_msg<HmiMcuInner::HmiMcuInner>(msg_cache_, record_writer,
                                            TOPIC_HMI_MCU_INNER);
  write_topic_msg<ParkingFusion::ParkingFusionInfo>(msg_cache_, record_writer,
                                                    TOPIC_PARKING_FUSION);
  write_topic_msg<FuncStateMachine::FuncStateMachine>(msg_cache_, record_writer,
                                                      TOPIC_FUNC_STATE_MACHINE);
  write_topic_msg<PlanningOutput::PlanningOutput>(
      output_msg_cache_, record_writer, TOPIC_PLANNING_PLAN);
  write_topic_msg<planning::common::PlanningDebugInfo>(
      output_msg_cache_, record_writer, TOPIC_PLANNING_DEBUG_INFO);
  write_topic_msg<PlanningHMI::PlanningHMIOutputInfoStr>(
      output_msg_cache_, record_writer, TOPIC_PLANNING_HMI);
  write_topic_msg<Map::StaticMap>(msg_cache_, record_writer, TOPIC_HD_MAP);

  std::cout << "write bag:" << record_writer.GetFile() << std::endl;
  record_writer.Close();
}

void PlanningPlayer::StoreCyberBag(const std::string& bag_path) {
  apollo::cyber::record::RecordWriter record_writer;
  record_writer.SetSizeOfFileSegmentation(0);
  record_writer.SetIntervalOfFileSegmentation(0);
  if (!record_writer.Open(bag_path)) {
    std::cerr << "open writer file failed: " << bag_path << std::endl;
    return;
  }

  for (auto& i : msg_cache_) {
    if (i.first != TOPIC_PLANNING_PLAN &&
        i.first != TOPIC_PLANNING_DEBUG_INFO && i.first != TOPIC_PLANNING_HMI) {
      for (auto& j : i.second) {
        msg_cache_ordered_by_time_[j.first] = j.second;
      }
    }
  }

  // PlanningOutput  PlanningDebugInfo  PlanningHMIOutputInfoStr
  for (auto& i : output_msg_cache_) {
    for (auto& j : i.second) {
      msg_cache_ordered_by_time_[j.first] = j.second;
    }
  }

  for (const auto& it_msg : msg_cache_ordered_by_time_) {
    if (!it_msg.second) {
      continue;
    }
    if (it_msg.second->GetTypeName() == "FusionRoad.RoadInfo") {
      write_msg<FusionRoad::RoadInfo>(it_msg, record_writer, TOPIC_ROAD_FUSION);
    } else if (it_msg.second->GetTypeName() ==
               "FusionObjects.FusionObjectsInfo") {
      write_msg<FusionObjects::FusionObjectsInfo>(it_msg, record_writer,
                                                  TOPIC_FUSION_OBJECTS);
    } else if (it_msg.second->GetTypeName() ==
               "LocalizationOutput.LocalizationEstimate") {
      write_msg<LocalizationOutput::LocalizationEstimate>(
          it_msg, record_writer, TOPIC_LOCALIZATION_ESTIMATE);
    } else if (it_msg.second->GetTypeName() == "Prediction.PredictionResult") {
      write_msg<Prediction::PredictionResult>(it_msg, record_writer,
                                              TOPIC_PREDICTION_RESULT);
    } else if (it_msg.second->GetTypeName() ==
               "VehicleService.VehicleServiceOutputInfo") {
      write_msg<VehicleService::VehicleServiceOutputInfo>(
          it_msg, record_writer, TOPIC_VEHICLE_SERVICE);
    } else if (it_msg.second->GetTypeName() == "ControlCommand.ControlOutput") {
      write_msg<ControlCommand::ControlOutput>(it_msg, record_writer,
                                               TOPIC_CONTROL_COMMAN);
    } else if (it_msg.second->GetTypeName() == "HmiMcuInner.HmiMcuInner") {
      write_msg<HmiMcuInner::HmiMcuInner>(it_msg, record_writer,
                                          TOPIC_HMI_MCU_INNER);
    } else if (it_msg.second->GetTypeName() ==
               "ParkingFusion.ParkingFusionInfo") {
      write_msg<ParkingFusion::ParkingFusionInfo>(it_msg, record_writer,
                                                  TOPIC_PARKING_FUSION);
    } else if (it_msg.second->GetTypeName() ==
               "FuncStateMachine.FuncStateMachine") {
      write_msg<FuncStateMachine::FuncStateMachine>(it_msg, record_writer,
                                                    TOPIC_FUNC_STATE_MACHINE);
    } else if (it_msg.second->GetTypeName() ==
               "PlanningOutput.PlanningOutput") {
      write_msg<PlanningOutput::PlanningOutput>(it_msg, record_writer,
                                                TOPIC_PLANNING_PLAN);
    } else if (it_msg.second->GetTypeName() ==
               "planning.common.PlanningDebugInfo") {
      write_msg<planning::common::PlanningDebugInfo>(it_msg, record_writer,
                                                     TOPIC_PLANNING_DEBUG_INFO);
    } else if (it_msg.second->GetTypeName() ==
               "PlanningHMI.PlanningHMIOutputInfoStr") {
      write_msg<PlanningHMI::PlanningHMIOutputInfoStr>(it_msg, record_writer,
                                                       TOPIC_PLANNING_HMI);
    } else if (it_msg.second->GetTypeName() == "Map.StaticMap") {
      write_msg<Map::StaticMap>(it_msg, record_writer, TOPIC_HD_MAP);
    } else {
      // std::cerr << "unsupported channel:" << msg.channel_name << std::endl;
    }
  }
  std::cout << "write bag:" << record_writer.GetFile() << std::endl;
  record_writer.Close();
}

void PlanningPlayer::PlayOneFrame(
    int frame_num, const planning::common::TopicTimeList& input_time_list) {
  std::cout << "==================== frame " << frame_num
            << "====================\n"
            << input_time_list.DebugString() << std::endl;

  auto fusion_object_msg =
      find_msg_with_header_time<FusionObjects::FusionObjectsInfo>(
          TOPIC_FUSION_OBJECTS, input_time_list.fusion_object());
  if (fusion_object_msg) {
    planning_adapter_->FeedFusionObjects(fusion_object_msg);
  } else {
    std::cerr << "frame_num " << frame_num_
              << " missing /iflytek/fusion/objects" << std::endl;
  }

  // 由于fusion_road的频率与planning相同，为了避免重复feed同一帧fusion_road而做对应判断
  if (input_time_list_road_fusion_ != input_time_list.fusion_road()) {
    input_time_list_road_fusion_ = input_time_list.fusion_road();
    auto fusion_road_msg = find_msg_with_header_time<FusionRoad::RoadInfo>(
        TOPIC_ROAD_FUSION, input_time_list.fusion_road());
    if (fusion_road_msg) {
      planning_adapter_->FeedFusionRoad(fusion_road_msg);
    } else {
      std::cerr << "frame_num " << frame_num_
                << " missing /iflytek/fusion/road_fusion" << std::endl;
    }
  }

  auto localization_estimate_msg =
      find_msg_with_header_time<LocalizationOutput::LocalizationEstimate>(
          TOPIC_LOCALIZATION_ESTIMATE, input_time_list.localization());

  if (localization_estimate_msg) {
    planning_adapter_->FeedLocalizationOutput(localization_estimate_msg);
  }

  auto prediction_msg = find_msg_with_header_time<Prediction::PredictionResult>(
      TOPIC_PREDICTION_RESULT, input_time_list.prediction());
  if (prediction_msg) {
    planning_adapter_->FeedPredictionResult(prediction_msg);
  } else {
    std::cerr << "frame_num " << frame_num_
              << " missing /iflytek/prediction/prediction_result" << std::endl;
  }

  auto vehicle_service_msg =
      find_msg_with_header_time<VehicleService::VehicleServiceOutputInfo>(
          TOPIC_VEHICLE_SERVICE, input_time_list.vehicle_service());
  if (vehicle_service_msg) {
    planning_adapter_->FeedVehicleService(vehicle_service_msg);
  } else {
    std::cerr << "frame_num " << frame_num_
              << " missing /iflytek/vehicle_service" << std::endl;
  }

  auto control_output_msg =
      find_msg_with_header_time<ControlCommand::ControlOutput>(
          TOPIC_CONTROL_COMMAN, input_time_list.control_output());
  if (control_output_msg) {
    planning_adapter_->FeedControlCommand(control_output_msg);
  } else {
    // std::cerr << "missing /iflytek/control/control_command" << std::endl;
  }

  auto hmi_mcu_msg = find_msg_with_header_time<HmiMcuInner::HmiMcuInner>(
      TOPIC_HMI_MCU_INNER, input_time_list.hmi());
  if (hmi_mcu_msg) {
    planning_adapter_->FeedHmiMcuInner(hmi_mcu_msg);
  } else {
    // std::cerr << "missing /iflytek/hmi/mcu_inner" << std::endl;
  }

  auto parking_fusion_msg =
      find_msg_with_header_time<ParkingFusion::ParkingFusionInfo>(
          TOPIC_PARKING_FUSION, input_time_list.parking_fusion());
  if (parking_fusion_msg) {
    planning_adapter_->FeedParkingFusion(parking_fusion_msg);
  } else {
    std::cerr << "frame_num " << frame_num_
              << " missing /iflytek/fusion/parking_slot" << std::endl;
  }

  // 由于static map的频率比planning低，为了避免重复feed同一帧static
  // map导致对map更新频率的误判而做对应判断
  if (input_time_list_map_ != input_time_list.map()) {
    input_time_list_map_ = input_time_list.map();
    auto hd_map_msg = find_msg_with_header_time<Map::StaticMap>(
        TOPIC_HD_MAP, input_time_list.map());
    if (hd_map_msg) {
      planning_adapter_->FeedMap(hd_map_msg);
    } else {
      std::cerr << "frame_num " << frame_num_
                << " missing /iflytek/ehr/static_map" << std::endl;
    }
  }

  auto func_state_machine_msg =
      std::make_shared<FuncStateMachine::FuncStateMachine>();
  if (input_time_list.has_function_state_machine()) {
    auto cached_func_state_machine_msg =
        find_msg_with_header_time<FuncStateMachine::FuncStateMachine>(
            TOPIC_FUNC_STATE_MACHINE, input_time_list.function_state_machine());
    if (cached_func_state_machine_msg) {
      func_state_machine_msg = cached_func_state_machine_msg;
    } else {
      std::cerr << "frame_num " << frame_num_
                << " missing /iflytek/system_state/soc_state" << std::endl;
    }
  }
  auto functional_state = ::FuncStateMachine::FunctionalState::INIT;
  if (frame_num >= frame_num_before_enter_auto_) {  // enter auto after 1.5s
    if (scene_type_ == "acc") {
      functional_state = ::FuncStateMachine::FunctionalState::ACC_ACTIVATE;
    } else if (scene_type_ == "apa") {
      functional_state =
          ::FuncStateMachine::FunctionalState::PARK_IN_ACTIVATE_CONTROL;
    } else if (scene_type_ == "scc") {
      functional_state = ::FuncStateMachine::FunctionalState::SCC_ACTIVATE;
    }
  }
  func_state_machine_msg->set_current_state(functional_state);
  planning_adapter_->FeedFuncStateMachine(func_state_machine_msg);

  uint64_t history_planning_duration =
      next_history_planning_start_time_ - history_planning_start_time_;
  uint64_t planning_duration = IflyTime::Now_us() - planning_start_timestamp_;
  if (planning_start_timestamp_ != 0 &&
      history_planning_duration > planning_duration) {
    usleep(history_planning_duration - planning_duration);
    planning_start_timestamp_ = IflyTime::Now_us();
    planning_adapter_->Proc();
  } else {
    planning_start_timestamp_ = IflyTime::Now_us();
    planning_adapter_->Proc();
  }
}

void PlanningPlayer::PlayAllFrames() {
  auto it_debug_info_msg = msg_cache_[TOPIC_PLANNING_DEBUG_INFO].begin();
  auto it_planning_msg = msg_cache_[TOPIC_PLANNING_PLAN].begin();
  auto it_planning_hmi_msg = msg_cache_[TOPIC_PLANNING_HMI].begin();

  for (size_t i = 0; i < msg_cache_[TOPIC_PLANNING_DEBUG_INFO].size() - 1;
       ++i) {
    auto debug_info =
        std::dynamic_pointer_cast<planning::common::PlanningDebugInfo>(
            it_debug_info_msg->second);
    auto planning_msg =
        std::dynamic_pointer_cast<PlanningOutput::PlanningOutput>(
            it_planning_msg->second);
    auto planning_hmi_msg =
        std::dynamic_pointer_cast<PlanningHMI::PlanningHMIOutputInfoStr>(
            it_planning_hmi_msg->second);

    if (!debug_info->has_input_topic_timestamp()) {
      std::cerr << "no input topic timestamp" << std::endl;
      continue;
    }
    planning_dubug_info_header_time_us_ = debug_info->timestamp();
    planning_dubug_info_msg_time_ns_ = it_debug_info_msg->first;
    it_debug_info_msg++;
    if (!std::dynamic_pointer_cast<planning::common::PlanningDebugInfo>(
             it_debug_info_msg->second)
             ->has_input_topic_timestamp()) {
      std::cerr << "no input topic timestamp" << std::endl;
      continue;
    }
    next_loc_header_time_us_ =
        std::dynamic_pointer_cast<planning::common::PlanningDebugInfo>(
            it_debug_info_msg->second)
            ->input_topic_timestamp()
            .localization();
    // 特殊处理最后一帧，否则最后几帧定位将和原包一致
    if (i == msg_cache_[TOPIC_PLANNING_DEBUG_INFO].size() - 2) {
      next_loc_header_time_us_ = UINT64_MAX;
    }

    planning_header_time_us_ = planning_msg->meta().header().timestamp();
    planning_msg_time_ns_ = it_planning_msg->first;
    for (auto& i : planning_msg->meta().header().input_list()) {
      if (i.input_type() == Common::InputHistoryTimestamp::
                                INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_PLANNING) {
        history_planning_start_time_ = i.in_ts_us();
        break;
      }
    }
    it_planning_msg++;
    for (auto& i : std::dynamic_pointer_cast<PlanningOutput::PlanningOutput>(
                       it_planning_msg->second)
                       ->meta()
                       .header()
                       .input_list()) {
      if (i.input_type() == Common::InputHistoryTimestamp::
                                INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_PLANNING) {
        next_history_planning_start_time_ = i.in_ts_us();
        break;
      }
    }

    planning_hmi_header_time_us_ = planning_hmi_msg->header().timestamp();
    planning_hmi_msg_time_ns_ = it_planning_hmi_msg->first;
    it_planning_hmi_msg++;

    loc_header_time_us_ = debug_info->input_topic_timestamp().localization();
    PlayOneFrame(frame_num_++, debug_info->input_topic_timestamp());
  }
}

void PlanningPlayer::RunCloseLoop(
    const PlanningOutput::PlanningOutput& planning_output) {
  if (scene_type_ == "scc") {  // scc
    if (!check_msg_exist(msg_cache_, TOPIC_PLANNING_DEBUG_INFO) ||
        !check_msg_exist(msg_cache_, TOPIC_LOCALIZATION)) {
      return;
    }
    auto traj_size = planning_output.trajectory().trajectory_points_size();
    if (traj_size < 10) {
      std::cerr << "RunCloseLoop fail, traj_points size=" << traj_size
                << std::endl;
      return;
    }

    for (auto it = msg_cache_[TOPIC_LOCALIZATION].begin();
         it != msg_cache_[TOPIC_LOCALIZATION].end(); it++) {
      auto loc_msg_i =
          std::const_pointer_cast<LocalizationOutput::LocalizationEstimate>(
              std::dynamic_pointer_cast<
                  LocalizationOutput::LocalizationEstimate>(it->second));
      auto loc_header_time_i = loc_msg_i->header().timestamp();
      if (loc_header_time_i > loc_header_time_us_) {
        if (loc_header_time_i <= next_loc_header_time_us_) {
          auto delta_t = loc_header_time_i - loc_header_time_us_;
          PerfectControlSCC(planning_output, delta_t, loc_msg_i);
        } else {
          break;
        }
      }
    }
  } else if (scene_type_ == "apa") {  // apa
    if (!check_msg_exist(msg_cache_, TOPIC_PLANNING_DEBUG_INFO) ||
        !check_msg_exist(msg_cache_, TOPIC_LOCALIZATION_ESTIMATE)) {
      return;
    }
    auto traj_size = planning_output.trajectory().trajectory_points_size();
    if (traj_size < 10) {
      std::cerr << "RunCloseLoop fail, traj_points size=" << traj_size
                << std::endl;
      return;
    }

    for (auto it = msg_cache_[TOPIC_LOCALIZATION_ESTIMATE].begin();
         it != msg_cache_[TOPIC_LOCALIZATION_ESTIMATE].end(); it++) {
      auto loc_msg_i =
          std::const_pointer_cast<LocalizationOutput::LocalizationEstimate>(
              std::dynamic_pointer_cast<
                  LocalizationOutput::LocalizationEstimate>(it->second));
      auto loc_header_time_i = loc_msg_i->header().timestamp();
      if (loc_header_time_i > loc_header_time_us_ &&
          loc_header_time_i < loc_header_time_us_ + 1000 * 1000) {
        auto delta_t = loc_header_time_i - loc_header_time_us_;
        PerfectControlAPA(planning_output, delta_t, loc_msg_i);
      }
    }
  }
}

void PlanningPlayer::PerfectControlAPA(
    const PlanningOutput::PlanningOutput& plan_msg, uint64_t delta_t,
    std::shared_ptr<LocalizationOutput::LocalizationEstimate>& loc_msg) {
  const double dt = static_cast<double>(delta_t) / 1e6;
  const auto path_size = plan_msg.trajectory().trajectory_points_size();

  if (path_size < 2) {
    std::cout << "planning error: path_size = " << path_size << std::endl;
    return;
  }

  std::vector<double> path_s_vec;
  std::vector<double> path_x_vec;
  std::vector<double> path_y_vec;
  std::vector<double> path_heading_vec;
  path_x_vec.reserve(path_size);
  path_y_vec.reserve(path_size);
  path_s_vec.reserve(path_size);
  path_heading_vec.reserve(path_size);

  double s = 0.0;
  double ds = 0.0;
  double angle_offset = 0.0;

  static const double pi_const = 3.141592654;
  static const double apa_vel_simulation = 0.5;
  for (int i = 0; i < path_size; ++i) {
    const auto& traj_point = plan_msg.trajectory().trajectory_points(i);

    path_x_vec.emplace_back(traj_point.x());
    path_y_vec.emplace_back(traj_point.y());
    path_s_vec.emplace_back(s);

    if (i <= path_size - 2) {
      const auto& traj_point_next =
          plan_msg.trajectory().trajectory_points(i + 1);

      ds = std::hypot(traj_point_next.x() - traj_point.x(),
                      traj_point_next.y() - traj_point.y());
      s += std::max(ds, 0.01);
    }

    auto heading = traj_point.heading_yaw();

    if (i > 0) {
      const auto& traj_point_last =
          plan_msg.trajectory().trajectory_points(i - 1);

      const auto d_heading =
          traj_point.heading_yaw() - traj_point_last.heading_yaw();

      if (d_heading > 1.5 * pi_const) {
        angle_offset -= 2.0 * pi_const;
      } else if (d_heading < -1.5 * pi_const) {
        angle_offset += 2.0 * pi_const;
      }

      heading += angle_offset;
    }

    path_heading_vec.emplace_back(heading);
  }

  pnc::mathlib::spline x_s_spline;
  pnc::mathlib::spline y_s_spline;
  pnc::mathlib::spline heading_s_spline;

  x_s_spline.set_points(path_s_vec, path_x_vec);
  y_s_spline.set_points(path_s_vec, path_y_vec);
  heading_s_spline.set_points(path_s_vec, path_heading_vec);

  const auto& current_pos = state_.pos;
  double path_length = path_s_vec.back();
  double s_proj = 0.0;
  bool success = pnc::geometry_lib::CalProjFromSplineByBisection(
      0.0, path_length, s_proj, current_pos, x_s_spline, y_s_spline);

  double remain_dist = 5.01;
  if (success == true) {
    remain_dist = path_length - s_proj;
  } else {
    std::cout << "remain_dist calculation error:input is error" << std::endl;
  }

  if (state_.static_flag) {
    if (state_.static_time > 0.6) {
      state_.static_flag = false;
    } else {
      state_.static_time += dt;
      state_.vel = 0.0;
      return;
    }
  }

  if (fabs(remain_dist) <= 0.06) {
    state_.vel = 0.0;
    state_.static_flag = true;
  } else {
    auto s_next = s_proj + state_.vel * dt;
    state_.vel = apa_vel_simulation;
    state_.pos << x_s_spline(s_next), y_s_spline(s_next);

    state_.heading =
        pnc::geometry_lib::NormalizeAngle(heading_s_spline(s_next));
  }

  Eigen::Vector3d euler_zxy;
  Eigen::Quaterniond q;

  q = pnc::transform::EulerZYX2Quat(euler_zxy);

  auto pose = loc_msg->mutable_pose();

  // vel
  pose->set_linear_velocity_from_wheel(state_.vel);

  // acc
  pose->mutable_linear_acceleration()->set_x(0.0);

  // atti
  pose->mutable_euler_angles()->set_yaw(euler_zxy[0]);
  pose->set_heading(pose->euler_angles().yaw());

  pose->mutable_orientation()->set_qw(q.w());
  pose->mutable_orientation()->set_qx(q.x());
  pose->mutable_orientation()->set_qy(q.y());
  pose->mutable_orientation()->set_qz(q.z());

  // pos
  pose->mutable_local_position()->set_x(state_.pos.x());
  pose->mutable_local_position()->set_y(state_.pos.y());
}

void PlanningPlayer::PerfectControlSCC(
    const PlanningOutput::PlanningOutput& plan_msg, uint64_t delta_t,
    std::shared_ptr<LocalizationOutput::LocalizationEstimate>& loc_msg) {
  const double dt = static_cast<double>(delta_t) / 1e6;
  const auto& trajectory = plan_msg.trajectory();
  auto traj_size = trajectory.trajectory_points_size();
  std::vector<double> x_vec(traj_size);
  std::vector<double> y_vec(traj_size);
  std::vector<double> theta_vec(traj_size);
  std::vector<double> v_vec(traj_size);
  std::vector<double> a_vec(traj_size);
  std::vector<double> t_vec(traj_size);

  double angle_offset = 0.0;
  static const double pi_const = 3.141592654;
  for (size_t i = 0; i < traj_size; ++i) {
    t_vec[i] = trajectory.trajectory_points(i).t();
    x_vec[i] = trajectory.trajectory_points(i).x();
    y_vec[i] = trajectory.trajectory_points(i).y();
    v_vec[i] = trajectory.trajectory_points(i).v();
    a_vec[i] = trajectory.trajectory_points(i).a();

    if (i == 0) {
      theta_vec[i] = trajectory.trajectory_points(i).heading_yaw();
    } else {
      const auto delta_theta =
          trajectory.trajectory_points(i).heading_yaw() -
          trajectory.trajectory_points(i - 1).heading_yaw();
      if (delta_theta > 1.5 * pi_const) {
        angle_offset -= 2.0 * pi_const;
      } else if (delta_theta < -1.5 * pi_const) {
        angle_offset += 2.0 * pi_const;
      }
      theta_vec[i] =
          trajectory.trajectory_points(i).heading_yaw() + angle_offset;
    }
  }

  pnc::mathlib::spline x_t_spline;
  pnc::mathlib::spline y_t_spline;
  pnc::mathlib::spline theta_t_spline;
  pnc::mathlib::spline v_t_spline;
  pnc::mathlib::spline a_t_spline;

  x_t_spline.set_points(t_vec, x_vec);
  y_t_spline.set_points(t_vec, y_vec);
  theta_t_spline.set_points(t_vec, theta_vec);
  v_t_spline.set_points(t_vec, v_vec);
  a_t_spline.set_points(t_vec, a_vec);

  const double x = x_t_spline(dt);
  const double y = y_t_spline(dt);
  const double v = v_t_spline(dt);
  const double a = a_t_spline(dt);
  const double theta = pnc::mathlib::DeltaAngleFix(theta_t_spline(dt));

  Eigen::Vector3d euler_zxy;
  Eigen::Quaterniond q;

  euler_zxy << theta, 0.0, 0.0;

  q = pnc::transform::EulerZYX2Quat(euler_zxy);

  auto pose = loc_msg->mutable_pose();

  // vel
  pose->set_linear_velocity_from_wheel(v);

  // acc
  pose->mutable_linear_acceleration()->set_x(a);

  // atti
  pose->mutable_euler_angles()->set_yaw(euler_zxy[0]);
  pose->set_heading(pose->euler_angles().yaw());

  pose->mutable_orientation()->set_qw(q.w());
  pose->mutable_orientation()->set_qx(q.x());
  pose->mutable_orientation()->set_qy(q.y());
  pose->mutable_orientation()->set_qz(q.z());

  // pos
  pose->mutable_local_position()->set_x(x);
  pose->mutable_local_position()->set_y(y);
}

}  // namespace planning_player
}  // namespace planning