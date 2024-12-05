#include "planning_player.h"

#include <google/protobuf/message.h>
#include <sys/types.h>

#include <boost/smart_ptr/make_shared_array.hpp>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <ios>
#include <vector>

#include "geometry_math.h"
#include "math_lib.h"
#include "nlohmann_json.hpp"
#include "proto_convert.h"
#include "spline.h"
#include "transform_lib.h"

namespace planning {
namespace planning_player {

static constexpr auto TOPIC_PLANNING_PLAN = "/iflytek/planning/plan";
static constexpr auto TOPIC_PLANNING_DEBUG_INFO =
    "/iflytek/planning/debug_info";
static constexpr auto TOPIC_PLANNING_DEBUG_INFO_ORIGIN =
    "/iflytek/planning/debug_info_origin";
static constexpr auto TOPIC_PLANNING_HMI = "/iflytek/planning/hmi";
static constexpr auto TOPIC_FUSION_OBJECTS = "/iflytek/fusion/objects";
static constexpr auto TOPIC_FUSION_OCCUPANCY_OBJECTS =
    "/iflytek/fusion/occupancy/objects";
static constexpr auto TOPIC_ROAD_FUSION = "/iflytek/fusion/road_fusion";
static constexpr auto TOPIC_LOCALIZATION_ESTIMATE =
    "/iflytek/localization/ego_pose";
static constexpr auto TOPIC_LOCALIZATION = "/iflytek/localization/egomotion";
static constexpr auto TOPIC_PREDICTION_RESULT =
    "/iflytek/prediction/prediction_result";
static constexpr auto TOPIC_VEHICLE_SERVICE = "/iflytek/vehicle_service";
static constexpr auto TOPIC_CONTROL_COMMAN = "/iflytek/control/control_command";
static constexpr auto TOPIC_HMI_MCU_INNER = "/iflytek/hmi/mcu_inner";
static constexpr auto TOPIC_PARKING_FUSION = "/iflytek/fusion/parking_slot";
static constexpr auto TOPIC_FUNC_STATE_MACHINE = "/iflytek/fsm/soc_state";
static constexpr auto TOPIC_HD_MAP = "/iflytek/ehr/static_map";
static constexpr auto TOPIC_SD_MAP = "/iflytek/ehr/sdmap_info";
static constexpr auto TOPIC_GROUND_LINE = "/iflytek/fusion/ground_line";
static constexpr auto TOPIC_EHR_PARKING_MAP = "/iflytek/ehr/parking_map";
static constexpr auto TOPIC_LANE_TOPO = "/iflytek/camera_perception/lane_topo";
static constexpr auto TOPIC_SYSTEM_VERSION = "/iflytek/system/version";
static constexpr auto TOPIC_TRAFFIC_SIGN =
    "/iflytek/camera_perception/traffic_sign_recognition";

// apa topics
static constexpr auto TOPIC_USS_WAVE_INFO = "/iflytek/uss/usswave_info";
static constexpr auto TOPIC_USS_PERCEPT_INFO =
    "/iflytek/uss/uss_perception_info";
static constexpr auto TOPIC_VISION_PARKING_SLOT =
    "/iflytek/camera_perception/parking_slot_list";
static constexpr auto TOPIC_CONTROL_DEBUG_INFO = "/iflytek/control/debug_info";

static const double KMaxCurvature = 1.0 / 5.6;
static const double KConstPi = 3.141592654;
static const double KApaVelSimulation = 0.3;

namespace fs = boost::filesystem;

void copy_confif_files(const fs::path& source, const fs::path& destination) {
  if (!fs::exists(source) || !fs::is_directory(source)) {
    std::cerr << "Source directory does not exist or is not a directory."
              << std::endl;
    return;
  }
  if (!fs::exists(destination)) {
    fs::create_directories(destination);
  }
  for (const auto& entry : fs::recursive_directory_iterator(source)) {
    fs::path target = destination / entry.path().filename();
    try {
      if (fs::is_directory(entry.status())) {
        fs::create_directory(target);
      } else {
        fs::copy_file(entry.path(), target,
                      fs::copy_option::overwrite_if_exists);
      }
    } catch (const fs::filesystem_error& e) {
      std::cerr << "Error in copy file : " << e.what() << std::endl;
    }
  }
}

bool PlanningPlayer::FindSceneType(const std::string& scene_type,
                                   const std::string& bag_path,
                                   std::string& out_bag) {
  // 找到第几帧进自动 && 确认场景
  scene_type_ = scene_type;
  bool find_scene_type = false;
  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException& e) {
    ROS_ERROR("Could not open bag file: %s", e.what());
    return false;
  }
  rosbag::View view(bag, rosbag::TopicQuery(TOPIC_FUNC_STATE_MACHINE));

  for (const auto& msg : view) {
    struct_msgs::FuncStateMachine::ConstPtr fsm_msg =
        msg.instantiate<struct_msgs::FuncStateMachine>();
    if (fsm_msg != nullptr) {
      auto current_state = fsm_msg->current_state;
      if ((current_state == iflyauto::FunctionalState_SCC_ACTIVATE) ||
          (current_state == iflyauto::FunctionalState_SCC_OVERRIDE)) {
        find_scene_type = true;
        scene_type_ = "scc";
        auto_timestamp_ = fsm_msg->msg_header.stamp;
        break;
      } else if ((current_state == iflyauto::FunctionalState_NOA_ACTIVATE) ||
                 (current_state == iflyauto::FunctionalState_NOA_OVERRIDE)) {
        find_scene_type = true;
        scene_type_ = "noa";
        auto_timestamp_ = fsm_msg->msg_header.stamp;
        break;
      } else if ((current_state == iflyauto::FunctionalState_HPP_IN_MEMORY) ||
                 (current_state ==
                  iflyauto::FunctionalState_HPP_IN_READY_EXISTROUTE) ||
                 (current_state ==
                  iflyauto::FunctionalState_HPP_IN_READY_REENTRYROUTE) ||
                 (current_state ==
                  iflyauto::FunctionalState_HPP_IN_MEMORY_READY) ||
                 (current_state ==
                  iflyauto::FunctionalState_HPP_IN_MEMORY_CRUISE) ||
                 (current_state == iflyauto::FunctionalState_HPP_IN_SECURE)) {
        find_scene_type = true;
        scene_type_ = "hpp";
        auto_timestamp_ = fsm_msg->msg_header.stamp;
        break;
      } else if ((current_state ==
                  iflyauto::FunctionalState_PARK_IN_SEARCHING) ||
                 (current_state == iflyauto::FunctionalState_PARK_GUIDANCE) ||
                 (current_state == iflyauto::FunctionalState_PARK_SUSPEND) ||
                 (current_state == iflyauto::FunctionalState_PARK_COMPLETED) ||
                 (current_state ==
                  iflyauto::FunctionalState_PARK_OUT_SEARCHING)) {
        find_scene_type = true;
        scene_type_ = "apa";
        break;
      }
    }
  }
  bag.close();
  if (!find_scene_type) {
    std::cout << "Attention!!!!!!! Unable to recognize the scene_type frome "
                 "bag, use default scene_type: "
              << scene_type_ << std::endl;
  }

  if (out_bag.find("bag.PP") == std::string::npos) {
    size_t lastDotPos = out_bag.rfind('.');
    if (lastDotPos != std::string::npos && lastDotPos != out_bag.length() - 1) {
      out_bag.insert(lastDotPos, "." + scene_type_);
    }
  }
  out_bag_ = out_bag;

  return true;
}

bool PlanningPlayer::Init(bool is_close_loop, double auto_time_sec,
                          bool no_debug, const std::string& car,
                          std::string& out_bag) {
  std::cout << "===========planning player init, is_close_loop="
            << is_close_loop << ", auto_time_sec=" << auto_time_sec
            << ", scene_type=" << scene_type_ << "==========" << std::endl;
  // 车辆配置文件
  car_ = car;
  fs::path source = "res/conf/product_configs/" + car;
  fs::path destination = "/asw/planning/res/conf/";
  copy_confif_files(source, destination);

  if (scene_type_ == "scc" || scene_type_ == "hpp" || scene_type_ == "noa" ||
      scene_type_ == "acc") {
    for (const auto& it : msg_cache_[TOPIC_PLANNING_DEBUG_INFO]) {
      auto debug_info_msg =
          boost::any_cast<sensor_interface::DebugInfo::Ptr>(it.second);
      planning::common::PlanningDebugInfo planning_debug_info;
      std::string planning_debug_info_str(debug_info_msg->debug_info.begin(),
                                          debug_info_msg->debug_info.end());
      planning_debug_info.ParseFromString(planning_debug_info_str);
      auto input_time_list = planning_debug_info.input_topic_timestamp();
      // 增加has_function_state_machine()的判断，应对该字段缺失情况
      if (input_time_list.has_function_state_machine() &&
          input_time_list.function_state_machine() < auto_timestamp_) {
        frame_num_before_enter_auto_++;
      } else {
        break;
      }
    }
  } else if (scene_type_ == "apa") {
    for (const auto& it : msg_cache_[TOPIC_PLANNING_PLAN]) {
      auto plan_msg =
          boost::any_cast<struct_msgs::PlanningOutput::Ptr>(it.second);
      const auto apa_planning_status =
          plan_msg->planning_status.apa_planning_status;
      if (apa_planning_status == iflyauto::ApaPlanningStatus::APA_IN_PROGRESS) {
        break;
      } else {
        frame_num_before_enter_auto_++;
      }
    }
  } else {
    std::cerr << "Error !!!!! Undefined scene type " << std::endl;
    return false;
  }

  // auto_time_sec默认1.5s，如果原包在1.5s之前进自动，则pp进自动时间为1.5s
  // 如果原包在1.5s之后进自动，则以原包为准
  if (auto_time_sec > 1.0 &&
      frame_num_before_enter_auto_ < (auto_time_sec * 10)) {
    frame_num_before_enter_auto_ = static_cast<int>(auto_time_sec * 10);
  }

  if (is_close_loop) {
    SimulationContext::Instance()->set_is_close_loop(true);
  }
  planning_adapter_ = std::make_unique<PlanningAdapter>();
  planning_adapter_->Init();

  ros::Time::init();

  ros::Time ros_start_time;
  if (check_msg_exist(msg_cache_, TOPIC_LOCALIZATION)) {
    ros_start_time =
        msg_cache_[TOPIC_LOCALIZATION].begin()->first + ros::Duration(2.0);
  } else if (check_msg_exist(msg_cache_, TOPIC_LOCALIZATION_ESTIMATE)) {
    ros_start_time = msg_cache_[TOPIC_LOCALIZATION_ESTIMATE].begin()->first +
                     ros::Duration(2.0);
  } else {
    std::cerr << "Error !!!!! missing localization msg" << std::endl;
    return false;
  }

  planning_adapter_->RegWriter_IflytekPlanningPlan(
      [this, is_close_loop, no_debug, ros_start_time](
          const iflyauto::PlanningOutput& planning_output) {
        auto planning_output_struct = planning_output;
        if (planning_output_struct.planning_status.apa_planning_status >
            iflyauto::ApaPlanningStatus::APA_IN_PROGRESS) {
          early_stop_ = true;
        }
        ros::Time ros_time;
        if (no_debug) {
          planning_output_struct.msg_header.stamp = local_time_;
          ros::Duration duration(0.1 * frame_num_);
          ros_time = ros_start_time + duration;
        } else {
          planning_output_struct.msg_header.stamp =
              planning_dubug_info_header_time_us_;
          ros_time = planning_dubug_info_msg_time_s_;
        }
        struct_msgs::PlanningOutput planning_output_ros_msg{};
        convert(planning_output_struct, planning_output_ros_msg,
                ConvertTypeInfo::TO_ROS);
        output_planning_msg_cache_[TOPIC_PLANNING_PLAN][ros_time] =
            planning_output_ros_msg;
        // 进自动之前不跟随新的轨迹，进自动的时间默认最少是1.5s
        if (is_close_loop && frame_num_ > frame_num_before_enter_auto_) {
          RunCloseLoop(planning_output_ros_msg);
        }
      });

  planning_adapter_->RegWriter_IflytekPlanningDebugInfo(
      [this, no_debug,
       ros_start_time](const iflyauto::StructContainer&
                           planning_debug_info) {
        ros::Time ros_time;
        planning::common::PlanningDebugInfo planning_debug_info_proto;
        planning_debug_info_proto.ParseFromString(
            planning_debug_info.payload());
        if (no_debug) {
          planning_debug_info_proto.set_timestamp(local_time_);
          planning_debug_info_proto.mutable_frame_info()->set_frame_num(
              frame_num_);
          ros::Duration duration(0.1 * frame_num_);
          ros_time = ros_start_time + duration;
        } else {
          planning_debug_info_proto.set_timestamp(
              planning_dubug_info_header_time_us_);
          planning_debug_info_proto.mutable_frame_info()->set_frame_num(
              planning_dubug_info_frame_num_);
          ros_time = planning_dubug_info_msg_time_s_;
        }
        std::string planning_dubug_info_str;
        planning_debug_info_proto.SerializeToString(&planning_dubug_info_str);
        sensor_interface::DebugInfo debug_info_msg;
        for (char c : planning_dubug_info_str) {
          debug_info_msg.debug_info.push_back(static_cast<unsigned char>(c));
        }
        output_planning_debug_msg_cache_[TOPIC_PLANNING_DEBUG_INFO][ros_time] =
            debug_info_msg;
      });

  planning_adapter_->RegWriter_IflytekPlanningHmi(
      [this, no_debug,
       ros_start_time](const iflyauto::PlanningHMIOutputInfoStr&
                           planning_hmi_ouput_info) {
        auto planning_hmi_ouput_info_struct = planning_hmi_ouput_info;
        ros::Time ros_time;
        if (no_debug) {
          planning_hmi_ouput_info_struct.msg_header.stamp = local_time_;
          ros::Duration duration(0.1 * frame_num_);
          ros_time = ros_start_time + duration;
        } else {
          planning_hmi_ouput_info_struct.msg_header.stamp =
              planning_dubug_info_header_time_us_;
          ros_time = planning_dubug_info_msg_time_s_;
        }
        struct_msgs::PlanningHMIOutputInfoStr planning_hmi_output_ros_msg{};
        convert(planning_hmi_ouput_info_struct, planning_hmi_output_ros_msg,
                ConvertTypeInfo::TO_ROS);
        output_planning_hmi_msg_cache_[TOPIC_PLANNING_HMI][ros_time] =
            planning_hmi_output_ros_msg;
      });

  return true;
}

void PlanningPlayer::Clear() {
  msg_cache_.clear();
  header_cache_.clear();
  output_planning_msg_cache_.clear();
  proto_desc_map_.clear();
  planning_header_time_us_ = 0;
  loc_header_time_us_ = 0;
  frame_num_before_enter_auto_ = 0;
}

void PlanningPlayer::getCommitHash(const std::string& directory, const int num,
                                   std::string& outVersion) {
  const std::string command = "cd " + directory + " && git rev-parse --short=" +
                              std::to_string(num) + " HEAD";
  FILE* pipe = popen(command.c_str(), "r");
  if (!pipe) {
    std::cerr << "Failed to run command: " << command << std::endl;
    return;
  }
  std::array<char, 41> buffer;
  if (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
    std::string commitHash(buffer.data());
    size_t newlinePos = commitHash.find('\n');
    if (newlinePos != std::string::npos) {
      outVersion = commitHash.substr(0, newlinePos);
    } else {
      outVersion = commitHash;  // No newline, use the whole string
    }
  }
  pclose(pipe);
}

void PlanningPlayer::VersinCheck(const std::string& bag_path) {
  std::cout << "=========== 版本信息 ===========" << std::endl;
  // bag version
  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException& e) {
    std::cerr << "Error opening bag file: " << e.what() << std::endl;
    return;
  }
  std::vector<std::string> topics;
  topics.push_back(TOPIC_PLANNING_PLAN);
  topics.push_back(TOPIC_SYSTEM_VERSION);
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  for (const auto& m : view) {
    if (m.getTopic() == TOPIC_PLANNING_PLAN) {
      struct_msgs::PlanningOutput::ConstPtr planning_output =
          m.instantiate<struct_msgs::PlanningOutput>();
      if (planning_output != nullptr) {
        std::string str(planning_output->msg_meta.version.begin(),
                        planning_output->msg_meta.version.end());
        bag_planning_version_ = str;
      }
    }
    if (m.getTopic() == TOPIC_SYSTEM_VERSION) {
      struct_msgs::SystemVersion::ConstPtr system_version =
          m.instantiate<struct_msgs::SystemVersion>();
      if (system_version != nullptr) {
        std::string str(system_version->interface_version.begin(),
                        system_version->interface_version.end());
        bag_interface_version_ = str;
      }
    }
    if (!bag_interface_version_.empty() && !bag_planning_version_.empty()) {
      break;
    }
  }
  bag.close();
  if (bag_planning_version_.size() > 24) {
    bag_planning_version_ = bag_planning_version_.substr(14, 8);
  }

  // local version
  getCommitHash(".", 8, local_planning_version_);
  getCommitHash("interface", 8, local_interface_version_);

  std::cout << std::endl;
  std::cout << "bag中planning版本: " << bag_planning_version_ << std::endl;
  std::cout << "本 地planning版本: " << local_planning_version_ << std::endl;
  std::cout << std::endl;
  std::cout << "bag中interface版本: " << bag_interface_version_ << std::endl;
  std::cout << "本 地interface版本: " << local_interface_version_ << std::endl;
  std::cout << std::endl;
}

bool PlanningPlayer::LoadRosBag(const std::string& bag_path, bool is_close_loop,
                                bool no_debug, bool interface_check) {
  std::cout << "=========== Start Load RosBag ===========" << std::endl;
  rosbag::Bag bag;
  rosbag::Bag new_bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
    new_bag.open(out_bag_, rosbag::bagmode::Write);
  } catch (rosbag::BagException& e) {
    std::cerr << "Error opening bag file: " << e.what() << std::endl;
    return false;
  }

  rosbag::View view(bag);

  for (const auto& msg : view) {
    if (msg.getTopic() == TOPIC_FUSION_OBJECTS) {
      cache_with_ros_msg_and_header_time<struct_msgs::FusionObjectsInfo>(msg);
    } else if (msg.getTopic() == TOPIC_FUSION_OCCUPANCY_OBJECTS) {
      cache_with_ros_msg_and_header_time<
          struct_msgs::FusionOccupancyObjectsInfo>(msg);
    } else if (msg.getTopic() == TOPIC_ROAD_FUSION) {
      cache_with_ros_msg_and_header_time<struct_msgs::RoadInfo>(msg);
    } else if (msg.getTopic() == TOPIC_LOCALIZATION_ESTIMATE) {
      cache_with_ros_msg_and_header_time_local_old<
          struct_msgs_legacy_v2_4_6::LocalizationEstimate>(msg, new_bag,
                                                           is_close_loop);
    } else if (msg.getTopic() == TOPIC_LOCALIZATION) {
      cache_with_ros_msg_and_header_time_local<struct_msgs::IFLYLocalization>(
          msg, new_bag, is_close_loop);
    } else if (msg.getTopic() == TOPIC_PREDICTION_RESULT) {
      cache_with_ros_msg_and_header_time<struct_msgs::PredictionResult>(msg);
    } else if (msg.getTopic() == TOPIC_VEHICLE_SERVICE) {
      cache_with_ros_msg_and_header_time<struct_msgs::VehicleServiceOutputInfo>(
          msg);
    } else if (msg.getTopic() == TOPIC_CONTROL_COMMAN) {
      cache_with_ros_msg_and_header_time<struct_msgs::ControlOutput>(msg);
    } else if (msg.getTopic() == TOPIC_HMI_MCU_INNER) {
      cache_with_ros_msg_and_header_time_old<
          struct_msgs_legacy_v2_4_5::HmiMcuInner>(msg);
    } else if (msg.getTopic() == TOPIC_PARKING_FUSION) {
      cache_with_ros_msg_and_header_time<struct_msgs::ParkingFusionInfo>(msg);
    } else if (msg.getTopic() == TOPIC_FUNC_STATE_MACHINE) {
      cache_with_ros_msg_and_header_time<struct_msgs::FuncStateMachine>(msg);
    } else if (msg.getTopic() == TOPIC_PLANNING_PLAN) {
      cache_with_ros_msg_time<struct_msgs::PlanningOutput>(msg);
    } else if (msg.getTopic() == TOPIC_PLANNING_DEBUG_INFO) {
      cache_with_ros_msg_time<sensor_interface::DebugInfo>(msg);
    } else if (msg.getTopic() == TOPIC_PLANNING_HMI) {
      cache_with_ros_msg_time<struct_msgs::PlanningHMIOutputInfoStr>(msg);
      // } else if (msg.getTopic() == TOPIC_HD_MAP) {
      //   cache_with_ros_msg_and_header_time<proto_msgs::StaticMap>(msg);
    } else if (msg.getTopic() == TOPIC_SD_MAP) {
      cache_with_ros_msg_time<sensor_interface::DebugInfo>(msg);
    } else if (msg.getTopic() == TOPIC_EHR_PARKING_MAP) {
      cache_with_ros_msg_and_header_time<struct_msgs::ParkingInfo>(msg);
    } else if (msg.getTopic() == TOPIC_GROUND_LINE) {
      cache_with_ros_msg_and_header_time<struct_msgs::GroundLinePerceptionInfo>(
          msg);
    } else if (msg.getTopic() == TOPIC_LANE_TOPO) {
      new_bag.write(msg.getTopic(), msg.getTime(), msg);
    } else if (msg.getTopic() == TOPIC_TRAFFIC_SIGN) {
      cache_with_ros_msg_and_header_time<struct_msgs::CameraPerceptionTsrInfo>(
          msg);
    } else {
      // std::cerr << "unsupported channel:" << msg.getTopic() << std::endl;
    }
  }

  if (scene_type_ == "apa") {
    for (const auto& msg : view) {
      if (msg.getTopic() == TOPIC_USS_WAVE_INFO) {
        cache_with_ros_msg_and_header_time<struct_msgs::UssWaveInfo>(msg);
      } else if (msg.getTopic() == TOPIC_USS_PERCEPT_INFO) {
        cache_with_ros_msg_and_header_time<struct_msgs::UssPerceptInfo>(msg);
      } else if (msg.getTopic() == TOPIC_VISION_PARKING_SLOT) {
        cache_with_ros_msg_and_header_time<struct_msgs::ParkingSlotSelectInfo>(
            msg);
      } else if (msg.getTopic() == TOPIC_CONTROL_DEBUG_INFO) {
        cache_with_ros_msg_time<sensor_interface::DebugInfo>(msg);
      } else {
        // std::cerr << "unsupported channel:" << msg.getTopic() << std::endl;
      }
    }
  }

  bag.close();
  new_bag.close();

  if (instant_error_) {
    if (interface_check) {
      return false;
    }
    std::cout << std::endl;
    std::cout << "********************************************************"
              << std::endl;
    std::cout << "bag与算法的interface版本不匹配, 导致以上topic无法正常读取"
              << std::endl;
    std::cout << "按回车键继续执行(缺少以上topic), Ctrl + c 退出程序"
              << std::endl;
    std::cout << "********************************************************"
              << std::endl;
    char ch = std::cin.get();
    // 检查输入是否为回车键
    if (ch == '\n') {
      return true;
    } else {
      return false;
    }
  }
  return true;
}

void PlanningPlayer::StoreRosBag() {
  rosbag::Bag bag;
  try {
    if (boost::filesystem::exists(out_bag_)) {
      // 如果文件存在，使用Append模式
      bag.open(out_bag_, rosbag::bagmode::Append);
    } else {
      // 如果文件不存在，使用Write模式
      bag.open(out_bag_, rosbag::bagmode::Write);
    }

    for (const auto& it_msg : msg_cache_) {
      if (it_msg.first == TOPIC_FUSION_OBJECTS) {
        write_ros_msg<struct_msgs::FusionObjectsInfo::Ptr>(
            it_msg.second, TOPIC_FUSION_OBJECTS, bag);
      } else if (it_msg.first == TOPIC_FUSION_OCCUPANCY_OBJECTS) {
        write_ros_msg<struct_msgs::FusionOccupancyObjectsInfo::Ptr>(
            it_msg.second, TOPIC_FUSION_OCCUPANCY_OBJECTS, bag);
      } else if (it_msg.first == TOPIC_ROAD_FUSION) {
        write_ros_msg<struct_msgs::RoadInfo::Ptr>(it_msg.second,
                                                  TOPIC_ROAD_FUSION, bag);
      } else if (it_msg.first == TOPIC_LOCALIZATION_ESTIMATE) {
        write_ros_msg<struct_msgs_legacy_v2_4_6::LocalizationEstimate::Ptr>(
            it_msg.second, TOPIC_LOCALIZATION_ESTIMATE, bag);
      } else if (it_msg.first == TOPIC_LOCALIZATION) {
        write_ros_msg<struct_msgs::IFLYLocalization::Ptr>(
            it_msg.second, TOPIC_LOCALIZATION, bag);
      } else if (it_msg.first == TOPIC_PREDICTION_RESULT) {
        write_ros_msg<struct_msgs::PredictionResult::Ptr>(
            it_msg.second, TOPIC_PREDICTION_RESULT, bag);
      } else if (it_msg.first == TOPIC_VEHICLE_SERVICE) {
        write_ros_msg<struct_msgs::VehicleServiceOutputInfo::Ptr>(
            it_msg.second, TOPIC_VEHICLE_SERVICE, bag);
      } else if (it_msg.first == TOPIC_CONTROL_COMMAN) {
        write_ros_msg<struct_msgs::ControlOutput::Ptr>(
            it_msg.second, TOPIC_CONTROL_COMMAN, bag);
      } else if (it_msg.first == TOPIC_HMI_MCU_INNER) {
        write_ros_msg<struct_msgs_legacy_v2_4_5::HmiMcuInner::Ptr>(
            it_msg.second, TOPIC_HMI_MCU_INNER, bag);
      } else if (it_msg.first == TOPIC_PARKING_FUSION) {
        write_ros_msg<struct_msgs::ParkingFusionInfo::Ptr>(
            it_msg.second, TOPIC_PARKING_FUSION, bag);
      } else if (it_msg.first == TOPIC_FUNC_STATE_MACHINE) {
        write_ros_msg<struct_msgs::FuncStateMachine::Ptr>(
            it_msg.second, TOPIC_FUNC_STATE_MACHINE, bag);
      } else if (it_msg.first == TOPIC_HD_MAP) {
        write_ros_msg<proto_msgs::StaticMap::Ptr>(it_msg.second, TOPIC_HD_MAP,
                                                  bag);
      } else if (it_msg.first == TOPIC_SD_MAP) {
        write_ros_msg<sensor_interface::DebugInfo::Ptr>(it_msg.second,
                                                        TOPIC_SD_MAP, bag);
      } else if (it_msg.first == TOPIC_EHR_PARKING_MAP) {
        write_ros_msg<struct_msgs::ParkingInfo::Ptr>(
            it_msg.second, TOPIC_EHR_PARKING_MAP, bag);
      } else if (it_msg.first == TOPIC_GROUND_LINE) {
        write_ros_msg<struct_msgs::GroundLinePerceptionInfo::Ptr>(
            it_msg.second, TOPIC_GROUND_LINE, bag);
      } else if (it_msg.first == TOPIC_TRAFFIC_SIGN) {
        write_ros_msg<struct_msgs::CameraPerceptionTsrInfo::Ptr>(
            it_msg.second, TOPIC_TRAFFIC_SIGN, bag);
      } else if (it_msg.first == TOPIC_USS_WAVE_INFO) {
        write_ros_msg<struct_msgs::UssWaveInfo::Ptr>(it_msg.second,
                                                     TOPIC_USS_WAVE_INFO, bag);
      } else if (it_msg.first == TOPIC_USS_PERCEPT_INFO) {
        write_ros_msg<struct_msgs::UssPerceptInfo::Ptr>(
            it_msg.second, TOPIC_USS_PERCEPT_INFO, bag);
      } else if (it_msg.first == TOPIC_VISION_PARKING_SLOT) {
        write_ros_msg<struct_msgs::ParkingSlotSelectInfo::Ptr>(
            it_msg.second, TOPIC_VISION_PARKING_SLOT, bag);
      } else if (it_msg.first == TOPIC_CONTROL_DEBUG_INFO) {
        write_ros_msg<sensor_interface::DebugInfo::Ptr>(
            it_msg.second, TOPIC_CONTROL_DEBUG_INFO, bag);
      } else if (it_msg.first == TOPIC_PLANNING_DEBUG_INFO) {
        write_ros_msg<sensor_interface::DebugInfo::Ptr>(
            it_msg.second, TOPIC_PLANNING_DEBUG_INFO_ORIGIN, bag);
      } else {
        // std::cerr << "unsupported channel:" << msg.channel_name <<
        // std::endl;
      }
    }

    for (const auto& it_msg : output_planning_msg_cache_) {
      if (it_msg.first == TOPIC_PLANNING_PLAN) {
        for (const auto& i : it_msg.second) {
          if (early_stop_time_ == ros::TIME_MIN ||
              i.first <= early_stop_time_) {
            bag.write(TOPIC_PLANNING_PLAN, i.first, i.second);
          }
        }
      }
    }

    for (const auto& it_msg : output_planning_hmi_msg_cache_) {
      if (it_msg.first == TOPIC_PLANNING_HMI) {
        for (const auto& i : it_msg.second) {
          if (early_stop_time_ == ros::TIME_MIN ||
              i.first <= early_stop_time_) {
            bag.write(TOPIC_PLANNING_HMI, i.first, i.second);
          }
        }
      }
    }

    for (const auto& it_msg : output_planning_debug_msg_cache_) {
      if (it_msg.first == TOPIC_PLANNING_DEBUG_INFO) {
        for (const auto& i : it_msg.second) {
          if (early_stop_time_ == ros::TIME_MIN ||
              i.first <= early_stop_time_) {
            bag.write(TOPIC_PLANNING_DEBUG_INFO, i.first, i.second);
          }
        }
      }
    }

    std::cout << "write bag: " << out_bag_ << std::endl;
    bag.close();
  } catch (rosbag::BagException& e) {
    std::cerr << "Error writing bag file: " << e.what() << std::endl;
    return;
  }
}

void PlanningPlayer::PlayOneFrame(
    int frame_num, const planning::common::TopicTimeList& input_time_list,
    bool is_close_loop) {
  std::cout << "************************************** frame " << frame_num
            << " **************************************" << std::endl;
  auto fusion_object_ros_msg =
      find_ros_msg_with_header_time<struct_msgs::FusionObjectsInfo>(
          TOPIC_FUSION_OBJECTS, input_time_list.fusion_object());
  if (fusion_object_ros_msg) {
    iflyauto::FusionObjectsInfo fusion_object_msg{};
    convert(fusion_object_msg, *fusion_object_ros_msg,
            ConvertTypeInfo::TO_STRUCT);
    planning_adapter_->Feed_IflytekFusionObjects(fusion_object_msg);
  } else {
    std::cerr << "frame_num " << frame_num_
              << " missing /iflytek/fusion/objects" << std::endl;
  }

  auto fusion_occ_object_ros_msg = find_ros_msg_with_header_time_upper_bound<
      struct_msgs::FusionOccupancyObjectsInfo>(TOPIC_FUSION_OCCUPANCY_OBJECTS,
                                               input_time_list.fusion_object());
  if (fusion_occ_object_ros_msg) {
    iflyauto::FusionOccupancyObjectsInfo fusion_occ_object_msg{};
    convert(fusion_occ_object_msg, *fusion_occ_object_ros_msg,
            ConvertTypeInfo::TO_STRUCT);
    planning_adapter_->Feed_IflytekFusionOccupancyObjects(fusion_occ_object_msg);
  } else {
    std::cerr << "frame_num " << frame_num_
              << " missing /iflytek/fusion/occupancy/objects" << std::endl;
  }

  // 由于fusion_road的频率与planning相同，为了避免重复feed同一帧fusion_road而做对应判断
  if (input_time_list_road_fusion_ != input_time_list.fusion_road()) {
    input_time_list_road_fusion_ = input_time_list.fusion_road();
    auto fusion_road_ros_msg =
        find_ros_msg_with_header_time<struct_msgs::RoadInfo>(
            TOPIC_ROAD_FUSION, input_time_list.fusion_road());
    if (fusion_road_ros_msg) {
      iflyauto::RoadInfo fusion_road_msg{};
      convert(fusion_road_msg, *fusion_road_ros_msg,
              ConvertTypeInfo::TO_STRUCT);
      planning_adapter_->Feed_IflytekFusionRoadFusion(fusion_road_msg);
    } else {
      std::cerr << "frame_num " << frame_num_
                << " missing /iflytek/fusion/road_fusion" << std::endl;
    }
  }

  // 兼容老版本的包，在老版本中，ego_pose的时间戳被加在input_topic_timestamp.localization字段
  auto input_time_localization_estimate =
      input_time_list.localization_estimate();
  if (0 == input_time_localization_estimate) {
    input_time_localization_estimate = input_time_list.localization();
  }
  auto localization_estimate_ros_msg = find_ros_msg_with_header_time<
      struct_msgs_legacy_v2_4_6::LocalizationEstimate>(
      TOPIC_LOCALIZATION_ESTIMATE, input_time_localization_estimate);
  if (localization_estimate_ros_msg) {
    iflyauto::interface_2_4_6::LocalizationEstimate localization_estimate_msg{};
    convert(localization_estimate_msg, *localization_estimate_ros_msg,
            ConvertTypeInfo::TO_STRUCT);
    planning_adapter_->FeedLocalizationEstimateOutput(
        localization_estimate_msg);
  } else {
    // std::cerr << "frame_num " << frame_num_
    //           << " missing /iflytek/localization/ego_pose" << std::endl;
  }

  auto localization_ros_msg =
      find_ros_msg_with_header_time<struct_msgs::IFLYLocalization>(
          TOPIC_LOCALIZATION, input_time_list.localization());
  if (localization_ros_msg) {
    iflyauto::IFLYLocalization localization_msg{};
    convert(localization_msg, *localization_ros_msg,
            ConvertTypeInfo::TO_STRUCT);
    planning_adapter_->Feed_IflytekLocalizationEgomotion(localization_msg);
  } else {
    // std::cerr << "frame_num " << frame_num_
    //           << " missing /iflytek/localization/egomotion" << std::endl;
  }

  auto prediction_ros_msg =
      find_ros_msg_with_header_time<struct_msgs::PredictionResult>(
          TOPIC_PREDICTION_RESULT, input_time_list.prediction());
  if (prediction_ros_msg) {
    iflyauto::PredictionResult prediction_msg{};
    convert(prediction_msg, *prediction_ros_msg, ConvertTypeInfo::TO_STRUCT);
    planning_adapter_->Feed_IflytekPredictionPredictionResult(prediction_msg);
  } else {
    // std::cerr << "frame_num " << frame_num_
    //           << " missing /iflytek/prediction/prediction_result" <<
    //           std::endl;
  }

  auto vehicle_service_ros_msg =
      find_ros_msg_with_header_time<struct_msgs::VehicleServiceOutputInfo>(
          TOPIC_VEHICLE_SERVICE, input_time_list.vehicle_service());
  if (vehicle_service_ros_msg) {
    iflyauto::VehicleServiceOutputInfo vehicle_service_msg{};
    convert(vehicle_service_msg, *vehicle_service_ros_msg,
            ConvertTypeInfo::TO_STRUCT);
    planning_adapter_->Feed_IflytekVehicleService(vehicle_service_msg);
  } else {
    std::cerr << "frame_num " << frame_num_
              << " missing /iflytek/vehicle_service" << std::endl;
  }

  auto control_output_ros_msg =
      find_ros_msg_with_header_time<struct_msgs::ControlOutput>(
          TOPIC_CONTROL_COMMAN, input_time_list.control_output());
  if (control_output_ros_msg) {
    iflyauto::ControlOutput control_output_msg{};
    convert(control_output_msg, *control_output_ros_msg,
            ConvertTypeInfo::TO_STRUCT);
    planning_adapter_->Feed_IflytekControlControlCommand(control_output_msg);
  } else {
    // std::cerr << "missing /iflytek/control/control_command" << std::endl;
  }

  auto hmi_mcu_ros_msg =
      find_ros_msg_with_header_time<struct_msgs_legacy_v2_4_5::HmiMcuInner>(
          TOPIC_HMI_MCU_INNER, input_time_list.hmi());
  if (hmi_mcu_ros_msg) {
    iflyauto::interface_2_4_5::HmiMcuInner hmi_inner_msg{};
    convert(hmi_inner_msg, *hmi_mcu_ros_msg, ConvertTypeInfo::TO_STRUCT);
    planning_adapter_->FeedHmiMcuInner(hmi_inner_msg);
  } else {
    // std::cerr << "missing /iflytek/hmi/mcu_inner" << std::endl;
  }

  auto parking_fusion_ros_msg =
      find_ros_msg_with_header_time<struct_msgs::ParkingFusionInfo>(
          TOPIC_PARKING_FUSION, input_time_list.parking_fusion());
  if (parking_fusion_ros_msg) {
    iflyauto::ParkingFusionInfo parking_fusion_msg{};
    convert(parking_fusion_msg, *parking_fusion_ros_msg,
            ConvertTypeInfo::TO_STRUCT);
    planning_adapter_->Feed_IflytekFusionParkingSlot(parking_fusion_msg);
  } else {
    // std::cerr << "frame_num " << frame_num_
    //           << " missing /iflytek/fusion/parking_slot" << std::endl;
  }

  auto perception_tsr_ros_msg =
      find_ros_msg_with_header_time<struct_msgs::CameraPerceptionTsrInfo>(
          TOPIC_TRAFFIC_SIGN, input_time_list.perception_tsr());
  if (perception_tsr_ros_msg) {
    iflyauto::CameraPerceptionTsrInfo perception_tsr_msg{};
    convert(perception_tsr_msg, *perception_tsr_ros_msg,
            ConvertTypeInfo::TO_STRUCT);
    planning_adapter_->Feed_IflytekCameraPerceptionTrafficSignRecognition(perception_tsr_msg);
  } else {
    std::cerr << "frame_num " << frame_num_
              << " missing /iflytek/camera_perception/traffic_sign_recognition"
              << std::endl;
  }

  auto uss_wave_ros_msg =
      find_ros_msg_with_header_time<struct_msgs::UssWaveInfo>(
          TOPIC_USS_WAVE_INFO, input_time_list.uss_wave());
  if (uss_wave_ros_msg) {
    iflyauto::UssWaveInfo uss_wave_msg{};
    convert(uss_wave_msg, *uss_wave_ros_msg, ConvertTypeInfo::TO_STRUCT);
    planning_adapter_->Feed_IflytekUssUsswaveInfo(uss_wave_msg);
  } else {
    // std::cerr << "frame_num " << frame_num_ << " missing
    // /iflytek/uss/wave_info"
    //           << std::endl;
  }

  auto uss_percept_ros_msg =
      find_ros_msg_with_header_time<struct_msgs::UssPerceptInfo>(
          TOPIC_USS_PERCEPT_INFO, input_time_list.uss_perception());
  if (uss_percept_ros_msg) {
    iflyauto::UssPerceptInfo uss_percept_msg{};
    convert(uss_percept_msg, *uss_percept_ros_msg, ConvertTypeInfo::TO_STRUCT);
    planning_adapter_->Feed_IflytekUssUssPerceptionInfo(uss_percept_msg);
  } else {
    // std::cerr << "frame_num " << frame_num_
    //           << " missing /iflytek/uss/uss_perception_info" << std::endl;
  }

  // 不再使用，注释掉
  // 由于static map的频率比planning低，为了避免重复feed同一帧static
  // map导致对map更新频率的误判而做对应判断
  // if (input_time_list_map_ != input_time_list.map()) {
  //   input_time_list_map_ = input_time_list.map();
  //   auto hd_map_ros_msg =
  //   find_ros_msg_with_header_time<proto_msgs::StaticMap>(
  //       TOPIC_HD_MAP, input_time_list.map());
  //   if (hd_map_ros_msg) {
  //     std::shared_ptr<Map::StaticMap> hd_map_msg;
  //     StaticMapToProto(*hd_map_msg, *hd_map_ros_msg);
  //     planning_adapter_->FeedMap(hd_map_msg);
  //   } else {
  //     std::cerr << "frame_num " << frame_num_
  //               << " missing /iflytek/ehr/static_map" << std::endl;
  //   }
  // }

  if (input_time_list_map_ != input_time_list.map()) {
    input_time_list_map_ = input_time_list.map();
    for (auto it = msg_cache_[TOPIC_SD_MAP].begin();
         it != msg_cache_[TOPIC_SD_MAP].end(); it++) {
      auto sd_map_msg_i =
          boost::any_cast<sensor_interface::DebugInfo::Ptr>(it->second);
      std::string sd_map_str(sd_map_msg_i->debug_info.begin(),
                             sd_map_msg_i->debug_info.end());
      auto sd_map = std::make_shared<SdMapSwtx::SdMap>();
      sd_map->ParseFromString(sd_map_str);
      if (sd_map->header().timestamp() == input_time_list_map_) {
        planning_adapter_->Feed_IflytekEhrSdmapInfo(*sd_map);
        break;
      }
    }
  }
  // TODO: for hpp, need FeedParkingMap() ready
  // TODO(zkxie2): 等interface就位
  // auto ehr_parking_map_ros_msg =
  //     find_ros_msg_with_header_time<struct_msgs::ParkingInfo>(
  //         TOPIC_EHR_PARKING_MAP, input_time_list.ehr_parking_map());
  // if (ehr_parking_map_ros_msg) {
  //   iflyauto::ParkingInfo ehr_parking_map_msg{};
  //   convert(ehr_parking_map_msg, *ehr_parking_map_ros_msg,
  //   ConvertTypeInfo::TO_STRUCT);
  //   planning_adapter_->FeedParkingMap(ehr_parking_map_msg);
  // } else {
  //   // std::cerr << "frame_num " << frame_num_
  //   //           << " missing /iflytek/ehr/parking_map" << std::endl;
  // }

  // auto ground_line_ros_msg =
  //     find_ros_msg_with_header_time<struct_msgs::GroundLinePerceptionInfo>(
  //         TOPIC_GROUND_LINE, input_time_list.ground_line());
  // if (ground_line_ros_msg) {
  //   iflyauto::GroundLinePerceptionInfo ground_line_msg{};
  //   convert(ground_line_msg, *ground_line_ros_msg,
  //   ConvertTypeInfo::TO_STRUCT);
  //   planning_adapter_->FeedGroundLinePerception(ground_line_msg);
  // } else {
  //   // std::cerr << "frame_num " << frame_num_
  //   //           << " missing /iflytek/fusion/ground_line" << std::endl;
  // }

  if (check_msg_exist(msg_cache_, TOPIC_FUNC_STATE_MACHINE)) {
    bool find_function_state_machine = false;
    struct_msgs::FuncStateMachine func_state_machine_ros_msg{};
    uint8_t functional_state = iflyauto::FunctionalState_MANUAL;
    if (input_time_list.function_state_machine()) {
      auto cached_func_state_machine_ros_msg =
          find_ros_msg_with_header_time<struct_msgs::FuncStateMachine>(
              TOPIC_FUNC_STATE_MACHINE,
              input_time_list.function_state_machine());
      if (cached_func_state_machine_ros_msg) {
        func_state_machine_ros_msg = *cached_func_state_machine_ros_msg;
        find_function_state_machine = true;
      } else {
        std::cerr << "frame_num " << frame_num_
                  << " missing /iflytek/fsm/soc_state" << std::endl;
      }
    }
    if (frame_num >= frame_num_before_enter_auto_) {  // enter auto after 1.5s
      if (scene_type_ == "acc") {
        functional_state = iflyauto::FunctionalState_ACC_ACTIVATE;
      } else if (scene_type_ == "apa") {
        if (find_function_state_machine) {
          if (is_close_loop) {
            if (last_functional_state !=
                    iflyauto::FunctionalState_PARK_IN_SEARCHING &&
                last_functional_state !=
                    iflyauto::FunctionalState_PARK_GUIDANCE) {
              last_functional_state =
                  iflyauto::FunctionalState_PARK_IN_SEARCHING;
            } else {
              last_functional_state = iflyauto::FunctionalState_PARK_GUIDANCE;
            }
            functional_state = last_functional_state;
          } else {
            functional_state = func_state_machine_ros_msg.current_state;
          }
        } else {
          functional_state = iflyauto::FunctionalState_MANUAL;
        }
      } else if (scene_type_ == "scc" || scene_type_ == "noa") {
        if (find_function_state_machine) {
          if (is_close_loop) {
            if (iflyauto::FunctionalState_SCC_STANDBY <=
                    func_state_machine_ros_msg.current_state &&
                func_state_machine_ros_msg.current_state <=
                    iflyauto::FunctionalState_SCC_OVERRIDE) {
              functional_state = iflyauto::FunctionalState_SCC_ACTIVATE;
            } else if (iflyauto::FunctionalState_NOA_STANDBY <=
                           func_state_machine_ros_msg.current_state &&
                       func_state_machine_ros_msg.current_state <=
                           iflyauto::FunctionalState_NOA_OVERRIDE) {
              functional_state = iflyauto::FunctionalState_NOA_ACTIVATE;
            } else if (iflyauto::FunctionalState_ACC_STANDBY <=
                           func_state_machine_ros_msg.current_state &&
                       func_state_machine_ros_msg.current_state <=
                           iflyauto::FunctionalState_ACC_OVERRIDE) {
              functional_state = iflyauto::FunctionalState_NOA_ACTIVATE;
            } else {
              functional_state = last_functional_state;
            }
          } else {
            functional_state = func_state_machine_ros_msg.current_state;
          }
        } else if (scene_type_ == "scc") {
          functional_state = iflyauto::FunctionalState_SCC_ACTIVATE;
        } else if (scene_type_ == "noa") {
          functional_state = iflyauto::FunctionalState_NOA_ACTIVATE;
        }
      } else if (scene_type_ == "hpp") {
        functional_state = iflyauto::FunctionalState_HPP_IN_MEMORY_CRUISE;
      }
    }
    func_state_machine_ros_msg.current_state = functional_state;
    last_functional_state = functional_state;
    iflyauto::FuncStateMachine func_state_machine_msg{};
    convert(func_state_machine_msg, func_state_machine_ros_msg,
            ConvertTypeInfo::TO_STRUCT);
    planning_adapter_->Feed_IflytekFsmSocState(func_state_machine_msg);
  } else {
    std::cerr << "Error !!!!! missing FUNC_STATE_MACHINE" << std::endl;
    return;
  }

  planning_adapter_->Proc();
}

void PlanningPlayer::PlayAllFrames(bool is_close_loop, bool play_in_loop) {
  auto it_debug_info_msg = msg_cache_[TOPIC_PLANNING_DEBUG_INFO].begin();

  for (size_t i = 0; i < msg_cache_[TOPIC_PLANNING_DEBUG_INFO].size() - 2;
       ++i) {
    auto debug_info = boost::any_cast<sensor_interface::DebugInfo::Ptr>(
        it_debug_info_msg->second);
    auto planning_debug_info = convert_debug_info(debug_info);

    // apa rosbag record ending time
    auto debug_info_topic_timestamp =
        planning_debug_info->input_topic_timestamp();
    auto early_stop_time_tmp = debug_info_topic_timestamp.localization();

    auto& debug_data_json = planning_debug_info->data_json();
    auto planning_loop_dt_start = debug_data_json.find("planning_loop_dt");
    if (planning_loop_dt_start != std::string::npos) {
      auto planning_loop_dt_end =
          debug_data_json.find(',', planning_loop_dt_start);
      auto planning_loop_dt = debug_data_json.substr(
          planning_loop_dt_start + 20,
          planning_loop_dt_end - planning_loop_dt_start - 20);
      SimulationContext::Instance()->set_planning_loop_dt(
          stod(planning_loop_dt));
    }
    auto prediction_relative_time_start =
        debug_data_json.find("prediction_relative_time");
    if (prediction_relative_time_start != std::string::npos) {
      auto prediction_relative_time_end =
          debug_data_json.find(',', prediction_relative_time_start);
      auto prediction_relative_time = debug_data_json.substr(
          prediction_relative_time_start + 28,
          prediction_relative_time_end - prediction_relative_time_start - 28);
      SimulationContext::Instance()->set_prediction_relative_time(
          stod(prediction_relative_time));
    }
    auto localizatoin_latency_start =
        debug_data_json.find("localizatoin_latency_inEgoStateManager");
    if (localizatoin_latency_start != std::string::npos) {
      auto localizatoin_latency_end =
          debug_data_json.find(',', localizatoin_latency_start);
      auto localizatoin_latency = debug_data_json.substr(
          localizatoin_latency_start + 42,
          localizatoin_latency_end - localizatoin_latency_start - 42);
      SimulationContext::Instance()->set_localizatoin_latency(
          stod(localizatoin_latency));
    }

    planning_dubug_info_header_time_us_ = planning_debug_info->timestamp();
    planning_dubug_info_frame_num_ =
        planning_debug_info->frame_info().frame_num();
    planning_dubug_info_msg_time_s_ = it_debug_info_msg->first;
    it_debug_info_msg++;

    auto next_debug_info = boost::any_cast<sensor_interface::DebugInfo::Ptr>(
        it_debug_info_msg->second);
    auto next_planning_debug_info = convert_debug_info(next_debug_info);
    auto next_input_topic_timestamp =
        next_planning_debug_info->input_topic_timestamp();

    next_loc_header_time_us_ = next_input_topic_timestamp.localization();
    next_loc_esti_header_time_us_ =
        next_input_topic_timestamp.localization_estimate();
    next_vehi_svc_header_time_us_ =
        next_input_topic_timestamp.vehicle_service();

    // 兼容老版本的包，在老版本中，ego_pose的时间戳被加在input_topic_timestamp.localization字段
    if (0 == next_loc_esti_header_time_us_) {
      next_loc_esti_header_time_us_ = next_loc_header_time_us_;
    }
    // 特殊处理最后一帧，否则最后几帧定位将和原包一致
    if (i == msg_cache_[TOPIC_PLANNING_DEBUG_INFO].size() - 3) {
      next_loc_header_time_us_ = UINT64_MAX;
      next_loc_esti_header_time_us_ = UINT64_MAX;
      next_vehi_svc_header_time_us_ = UINT64_MAX;
    }

    loc_header_time_us_ =
        planning_debug_info->input_topic_timestamp().localization();
    loc_esti_header_time_us_ =
        planning_debug_info->input_topic_timestamp().localization_estimate();
    // 兼容老版本的包，定位时间戳的存放位置较混乱
    if (0 == loc_esti_header_time_us_ && 0 != loc_header_time_us_) {
      loc_esti_header_time_us_ = loc_header_time_us_;
    } else if (0 == loc_header_time_us_ && 0 != loc_esti_header_time_us_) {
      loc_header_time_us_ = loc_esti_header_time_us_;
    }
    vehi_svc_header_time_us_ =
        planning_debug_info->input_topic_timestamp().vehicle_service();
    if (!early_stop_) {
      PlayOneFrame(frame_num_++, planning_debug_info->input_topic_timestamp(),
                   is_close_loop);
    } else {
      early_stop_time_ = ros::Time(early_stop_time_tmp * 1.0e-6);
      break;
    }

    if (play_in_loop) {
      if (i == msg_cache_[TOPIC_PLANNING_DEBUG_INFO].size() - 3) {
        i = 0;
        it_debug_info_msg = msg_cache_[TOPIC_PLANNING_DEBUG_INFO].begin();
      }
    }
  }
}

void PlanningPlayer::RunCloseLoop(
    const struct_msgs::PlanningOutput& planning_output) {
  if (scene_type_ == "scc" || scene_type_ == "noa" ||
      scene_type_ == "hpp") {  // scc
    if (!check_msg_exist(msg_cache_, TOPIC_PLANNING_DEBUG_INFO)) {
      std::cerr << "Error!!! missing planning debug info" << std::endl;
      return;
    }
    auto traj_size = planning_output.trajectory.trajectory_points_size;
    if (traj_size < 10) {
      std::cerr << "RunCloseLoop fail, traj_points size=" << traj_size
                << std::endl;
      return;
    }

    PerpareTrajectory(planning_output);
    if (check_msg_exist(msg_cache_, TOPIC_LOCALIZATION)) {
      for (auto it = msg_cache_[TOPIC_LOCALIZATION].begin();
           it != msg_cache_[TOPIC_LOCALIZATION].end(); it++) {
        auto loc_msg_i =
            boost::any_cast<struct_msgs::IFLYLocalization::Ptr>(it->second);
        auto loc_header_time_i = loc_msg_i->msg_header.stamp;
        if (loc_header_time_i > loc_header_time_us_) {
          if (loc_header_time_i <= next_loc_header_time_us_) {
            auto delta_t = loc_header_time_i - loc_header_time_us_;
            PerfectControlEgoMotion(delta_t, loc_msg_i);
          } else {
            break;
          }
        }
      }
    } else if (check_msg_exist(msg_cache_, TOPIC_LOCALIZATION_ESTIMATE)) {
      for (auto it = msg_cache_[TOPIC_LOCALIZATION_ESTIMATE].begin();
           it != msg_cache_[TOPIC_LOCALIZATION_ESTIMATE].end(); it++) {
        auto loc_msg_i = boost::any_cast<
            struct_msgs_legacy_v2_4_6::LocalizationEstimate::Ptr>(it->second);
        auto loc_header_time_i = loc_msg_i->msg_header.timestamp;
        if (loc_header_time_i > loc_esti_header_time_us_) {
          if (loc_header_time_i <= next_loc_esti_header_time_us_) {
            auto delta_t = loc_header_time_i - loc_esti_header_time_us_;
            PerfectControlEgoPose(delta_t, loc_msg_i);
          } else {
            break;
          }
        }
      }
    } else {
      std::cerr << "Error, missing localization topic !!!!" << std::endl;
      return;
    }
    UpdateVehicleServiceData();
  } else if (scene_type_ == "apa") {
    // apa
    if (!check_msg_exist(msg_cache_, TOPIC_PLANNING_DEBUG_INFO) ||
        !check_msg_exist(msg_cache_, TOPIC_LOCALIZATION)) {
      return;
    }
    int traj_size = planning_output.trajectory.trajectory_points_size;
    if (traj_size < 3) {
      std::cerr << "RunCloseLoop fail, traj_points size = " << traj_size
                << std::endl;
      return;
    }

    // localization msg
    if (check_msg_exist(msg_cache_, TOPIC_LOCALIZATION)) {
      update_spline_ = true;
      for (auto it = msg_cache_[TOPIC_LOCALIZATION].begin();
           it != msg_cache_[TOPIC_LOCALIZATION].end(); it++) {
        auto loc_msg_i =
            boost::any_cast<struct_msgs::IFLYLocalization::Ptr>(it->second);
        auto loc_header_time_i = loc_msg_i->msg_header.stamp;
        if (loc_header_time_i > loc_header_time_us_) {
          if (loc_header_time_i <= next_loc_header_time_us_) {
            auto delta_t = loc_header_time_i - loc_header_time_us_;
            PerfectControlAPANewLocalization(planning_output, delta_t,
                                             loc_msg_i);
            update_spline_ = false;
          } else {
            break;
          }
        }
      }
    } else if (check_msg_exist(msg_cache_, TOPIC_LOCALIZATION_ESTIMATE)) {
      for (auto it = msg_cache_[TOPIC_LOCALIZATION_ESTIMATE].begin();
           it != msg_cache_[TOPIC_LOCALIZATION_ESTIMATE].end(); it++) {
        auto loc_msg_i = boost::any_cast<
            struct_msgs_legacy_v2_4_6::LocalizationEstimate::Ptr>(it->second);
        auto loc_header_time_i = loc_msg_i->msg_header.timestamp;
        if (loc_header_time_i > loc_esti_header_time_us_) {
          if (loc_header_time_i <= next_loc_esti_header_time_us_) {
            auto delta_t = loc_header_time_i - loc_esti_header_time_us_;
            PerfectControlAPA(planning_output, delta_t, loc_msg_i);
          } else {
            break;
          }
        }
      }
    } else {
      std::cerr << "Error !!!!! missing localization msg" << std::endl;
      return;
    }
    UpdateVehicleServiceDataAPA();

  } else {
    std::cerr << "Error, unknown scene_type !" << std::endl;
  }
}

void PlanningPlayer::PerpareTrajectory(
    const struct_msgs::PlanningOutput& plan_msg) {
  const auto& trajectory = plan_msg.trajectory;
  auto traj_size = trajectory.trajectory_points_size;
  std::vector<double> x_vec(traj_size);
  std::vector<double> y_vec(traj_size);
  std::vector<double> theta_vec(traj_size);
  std::vector<double> v_vec(traj_size);
  std::vector<double> a_vec(traj_size);
  std::vector<double> t_vec(traj_size);
  std::vector<double> yaw_rate_vec(traj_size);
  std::vector<double> curvature_vec(traj_size);

  double angle_offset = 0.0;
  for (size_t i = 0; i < traj_size; ++i) {
    t_vec[i] = trajectory.trajectory_points[i].t;
    x_vec[i] = trajectory.trajectory_points[i].x;
    y_vec[i] = trajectory.trajectory_points[i].y;
    v_vec[i] = trajectory.trajectory_points[i].v;
    a_vec[i] = trajectory.trajectory_points[i].a;
    curvature_vec[i] = trajectory.trajectory_points[i].curvature;

    if (i == 0) {
      theta_vec[i] = trajectory.trajectory_points[i].heading_yaw;
    } else {
      const auto delta_theta = trajectory.trajectory_points[i].heading_yaw -
                               trajectory.trajectory_points[i - 1].heading_yaw;
      if (delta_theta > 1.5 * KConstPi) {
        angle_offset -= 2.0 * KConstPi;
      } else if (delta_theta < -1.5 * KConstPi) {
        angle_offset += 2.0 * KConstPi;
      }
      theta_vec[i] = trajectory.trajectory_points[i].heading_yaw + angle_offset;
      const auto delta_t = t_vec[i] - t_vec[i - 1];
      yaw_rate_vec[i] = delta_theta / delta_t;
    }
  }

  x_t_spline_.set_points(t_vec, x_vec);
  y_t_spline_.set_points(t_vec, y_vec);
  theta_t_spline_.set_points(t_vec, theta_vec);
  v_t_spline_.set_points(t_vec, v_vec);
  a_t_spline_.set_points(t_vec, a_vec);
  yaw_rate_t_spline_.set_points(t_vec, yaw_rate_vec);
  curvature_t_spline_.set_points(t_vec, curvature_vec);
}

void PlanningPlayer::PerfectControlEgoMotion(
    uint64_t delta_t, struct_msgs::IFLYLocalization::Ptr loc_msg) {
  const double dt = static_cast<double>(delta_t) / 1e6;
  const double x = x_t_spline_(dt);
  const double y = y_t_spline_(dt);
  const double v = v_t_spline_(dt);
  const double a = a_t_spline_(dt);
  const double theta = pnc::mathlib::DeltaAngleFix(theta_t_spline_(dt));

  Eigen::Vector3d euler_zxy;
  Eigen::Quaterniond q;

  euler_zxy << theta, 0.0, 0.0;

  q = pnc::transform::EulerZYX2Quat(euler_zxy);

  // vel
  loc_msg->velocity.velocity_boot.vx = v * cos(theta);
  loc_msg->velocity.velocity_boot.vy = v * sin(theta);
  loc_msg->velocity.velocity_boot.vz = 0;

  loc_msg->velocity.velocity_body.vx = v;
  loc_msg->velocity.velocity_body.vy = 0;
  loc_msg->velocity.velocity_body.vz = 0;

  // acc
  loc_msg->acceleration.acceleration_boot.ax = a * cos(theta);
  loc_msg->acceleration.acceleration_boot.ay = a * sin(theta);
  loc_msg->acceleration.acceleration_boot.az = 0;

  loc_msg->acceleration.acceleration_body.ax = a;
  loc_msg->acceleration.acceleration_body.ay = 0;
  loc_msg->acceleration.acceleration_body.az = 0;

  // atti
  loc_msg->orientation.euler_boot.yaw = euler_zxy[0];

  loc_msg->orientation.quaternion_boot.w = q.w();
  loc_msg->orientation.quaternion_boot.x = q.x();
  loc_msg->orientation.quaternion_boot.y = q.y();
  loc_msg->orientation.quaternion_boot.z = q.z();

  // pos
  loc_msg->position.position_boot.x = x;
  loc_msg->position.position_boot.y = y;
}

void PlanningPlayer::PerfectControlAPA(
    const struct_msgs::PlanningOutput& plan_msg, uint64_t delta_t,
    struct_msgs_legacy_v2_4_6::LocalizationEstimate::Ptr loc_msg) {
  const double dt = static_cast<double>(delta_t) / 1e6;
  const int path_size = plan_msg.trajectory.trajectory_points_size;

  if (path_size < 3) {
    std::cerr << "planning error: path_size = " << path_size << std::endl;
    return;
  }

  if (update_spline_) {
    path_x_vec_.clear();
    path_y_vec_.clear();
    path_s_vec_.clear();
    path_heading_vec_.clear();

    path_x_vec_.reserve(path_size);
    path_y_vec_.reserve(path_size);
    path_s_vec_.reserve(path_size);
    path_heading_vec_.reserve(path_size);

    double s = 0.0;
    double ds = 0.0;
    double angle_offset = 0.0;
    for (int i = 0; i < path_size; ++i) {
      const auto& traj_point = plan_msg.trajectory.trajectory_points[i];

      path_x_vec_.emplace_back(traj_point.x);
      path_y_vec_.emplace_back(traj_point.y);
      path_s_vec_.emplace_back(s);

      if (i <= path_size - 2) {
        const auto& traj_point_next =
            plan_msg.trajectory.trajectory_points[i + 1];

        ds = std::hypot(traj_point_next.x - traj_point.x,
                        traj_point_next.y - traj_point.y);
        s += std::max(ds, 0.01);
      }

      auto heading = traj_point.heading_yaw;

      if (i > 0) {
        const auto& traj_point_last =
            plan_msg.trajectory.trajectory_points[i - 1];

        const auto d_heading =
            traj_point.heading_yaw - traj_point_last.heading_yaw;

        if (d_heading > 1.5 * KConstPi) {
          angle_offset -= 2.0 * KConstPi;
        } else if (d_heading < -1.5 * KConstPi) {
          angle_offset += 2.0 * KConstPi;
        }

        heading += angle_offset;
      }

      path_heading_vec_.emplace_back(heading);
    }

    x_s_spline_.set_points(path_s_vec_, path_x_vec_);
    y_s_spline_.set_points(path_s_vec_, path_y_vec_);
    heading_s_spline_.set_points(path_s_vec_, path_heading_vec_);
  }

  const auto& current_pos = state_.pos;
  double path_length = path_s_vec_.back();
  double s_proj = 0.0;
  bool success = pnc::geometry_lib::CalProjFromSplineByBisection(
      0.0, path_length, s_proj, current_pos, x_s_spline_, y_s_spline_);

  double remain_dist = 5.01;
  if (success == true) {
    remain_dist = path_length - s_proj;
  } else {
    std::cerr << "remain_dist calculation error:input is error" << std::endl;
  }

  if (state_.static_flag) {
    if (state_.static_time > 0.6) {
      state_.static_flag = false;
    } else {
      state_.static_time += dt;
      state_.vel = 0.0;
    }
  }

  if (fabs(remain_dist) <= 0.06) {
    state_.vel = 0.0;
    state_.static_flag = true;
  } else {
    auto s_next = s_proj + state_.vel * dt;
    state_.vel = KApaVelSimulation;
    state_.pos << x_s_spline_(s_next), y_s_spline_(s_next);

    state_.heading =
        pnc::geometry_lib::NormalizeAngle(heading_s_spline_(s_next));
  }

  Eigen::Vector3d euler_zxy;
  Eigen::Quaterniond q;

  euler_zxy << state_.heading, 0.0, 0.0;
  q = pnc::transform::EulerZYX2Quat(euler_zxy);

  auto pose = loc_msg->pose;

  // vel
  pose.linear_velocity_from_wheel = state_.vel;

  // acc
  pose.linear_acceleration.x = 0.0;

  // atti
  pose.euler_angles.yaw = euler_zxy[0];
  pose.heading = pose.euler_angles.yaw;

  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();

  // pos
  pose.local_position.x = state_.pos.x();
  pose.local_position.y = state_.pos.y();
}

void PlanningPlayer::PerfectControlAPANewLocalization(
    const struct_msgs::PlanningOutput& plan_msg, uint64_t delta_t,
    struct_msgs::IFLYLocalization::Ptr loc_msg) {
  const double dt = static_cast<double>(delta_t) / 1e6;
  const int path_size = plan_msg.trajectory.trajectory_points_size;

  if (path_size < 3) {
    std::cerr << "planning error: path_size = " << path_size << std::endl;
    return;
  }

  if (update_spline_) {
    path_x_vec_.clear();
    path_y_vec_.clear();
    path_s_vec_.clear();
    path_heading_vec_.clear();

    path_x_vec_.reserve(path_size);
    path_y_vec_.reserve(path_size);
    path_s_vec_.reserve(path_size);
    path_heading_vec_.reserve(path_size);

    double s = 0.0;
    double ds = 0.0;
    double angle_offset = 0.0;
    for (int i = 0; i < path_size; ++i) {
      const auto& traj_point = plan_msg.trajectory.trajectory_points[i];

      path_x_vec_.emplace_back(traj_point.x);
      path_y_vec_.emplace_back(traj_point.y);
      path_s_vec_.emplace_back(s);

      if (i <= path_size - 2) {
        const auto& traj_point_next =
            plan_msg.trajectory.trajectory_points[i + 1];

        ds = std::hypot(traj_point_next.x - traj_point.x,
                        traj_point_next.y - traj_point.y);
        s += std::max(ds, 0.01);
      }

      auto heading = traj_point.heading_yaw;

      if (i > 0) {
        const auto& traj_point_last =
            plan_msg.trajectory.trajectory_points[i - 1];

        const auto d_heading =
            traj_point.heading_yaw - traj_point_last.heading_yaw;

        if (d_heading > 1.5 * KConstPi) {
          angle_offset -= 2.0 * KConstPi;
        } else if (d_heading < -1.5 * KConstPi) {
          angle_offset += 2.0 * KConstPi;
        }

        heading += angle_offset;
      }

      path_heading_vec_.emplace_back(heading);
    }

    x_s_spline_.set_points(path_s_vec_, path_x_vec_);
    y_s_spline_.set_points(path_s_vec_, path_y_vec_);
    heading_s_spline_.set_points(path_s_vec_, path_heading_vec_);
  }

  const auto& current_pos = state_.pos;
  double path_length = path_s_vec_.back();
  double s_proj = 0.0;
  bool success = pnc::geometry_lib::CalProjFromSplineByBisection(
      0.0, path_length, s_proj, current_pos, x_s_spline_, y_s_spline_);

  double remain_dist = 5.01;
  if (success == true) {
    state_.s_proj = s_proj;
    remain_dist = path_length - s_proj;
  } else {
    std::cerr << "remain_dist calculation error:input is error" << std::endl;
  }

  if (state_.static_flag) {
    if (state_.static_time > 0.6) {
      state_.static_flag = false;
    } else {
      state_.static_time += dt;
      state_.vel = 0.0;
    }
  }

  if (fabs(remain_dist) <= 0.06) {
    state_.vel = 0.0;
    state_.static_flag = true;
  } else {
    auto s_next = s_proj + state_.vel * dt;
    state_.s_proj = s_next;
    state_.vel = KApaVelSimulation;
    state_.pos << x_s_spline_(s_next), y_s_spline_(s_next);

    state_.heading =
        pnc::geometry_lib::NormalizeAngle(heading_s_spline_(s_next));
  }

  Eigen::Vector3d euler_zxy;
  Eigen::Quaterniond q;

  euler_zxy << state_.heading, 0.0, 0.0;
  q = pnc::transform::EulerZYX2Quat(euler_zxy);

  // vel
  loc_msg->velocity.velocity_boot.vx = state_.vel * cos(state_.heading);
  loc_msg->velocity.velocity_boot.vy = state_.vel * sin(state_.heading);
  loc_msg->velocity.velocity_boot.vz = 0;

  // acc
  loc_msg->acceleration.acceleration_boot.ax = 0;
  loc_msg->acceleration.acceleration_boot.ay = 0;
  loc_msg->acceleration.acceleration_boot.az = 0;

  loc_msg->acceleration.acceleration_body.ax = 0;
  loc_msg->acceleration.acceleration_body.ay = 0;
  loc_msg->acceleration.acceleration_body.az = 0;

  // atti
  loc_msg->orientation.euler_boot.yaw = euler_zxy[0];

  loc_msg->orientation.quaternion_boot.w = q.w();
  loc_msg->orientation.quaternion_boot.x = q.x();
  loc_msg->orientation.quaternion_boot.y = q.y();
  loc_msg->orientation.quaternion_boot.z = q.z();

  // pos
  loc_msg->position.position_boot.x = state_.pos.x();
  loc_msg->position.position_boot.y = state_.pos.y();
}

void PlanningPlayer::PerfectControlEgoPose(
    uint64_t delta_t,
    struct_msgs_legacy_v2_4_6::LocalizationEstimate::Ptr loc_msg) {
  const double dt = static_cast<double>(delta_t) / 1e6;
  const double x = x_t_spline_(dt);
  const double y = y_t_spline_(dt);
  const double v = v_t_spline_(dt);
  const double a = a_t_spline_(dt);
  const double theta = pnc::mathlib::DeltaAngleFix(theta_t_spline_(dt));

  Eigen::Vector3d euler_zxy;
  Eigen::Quaterniond q;

  euler_zxy << theta, 0.0, 0.0;

  q = pnc::transform::EulerZYX2Quat(euler_zxy);

  // vel
  loc_msg->pose.linear_velocity_from_wheel = v;

  // acc
  loc_msg->pose.linear_acceleration.x = a;

  // atti
  loc_msg->pose.euler_angles.yaw = euler_zxy[0];
  loc_msg->pose.heading = loc_msg->pose.euler_angles.yaw;

  loc_msg->pose.orientation.w = q.w();
  loc_msg->pose.orientation.x = q.x();
  loc_msg->pose.orientation.y = q.y();
  loc_msg->pose.orientation.z = q.z();

  // pos
  loc_msg->pose.local_position.x = x;
  loc_msg->pose.local_position.y = y;
}

void PlanningPlayer::UpdateVehicleServiceData() {
  for (auto it = msg_cache_[TOPIC_VEHICLE_SERVICE].begin();
       it != msg_cache_[TOPIC_VEHICLE_SERVICE].end(); it++) {
    auto vehi_svc_msg_i =
        boost::any_cast<struct_msgs::VehicleServiceOutputInfo::Ptr>(it->second);
    auto vehi_svc_header_time_i = vehi_svc_msg_i->msg_header.stamp;
    if (vehi_svc_header_time_i > vehi_svc_header_time_us_) {
      if (vehi_svc_header_time_i <= next_vehi_svc_header_time_us_) {
        auto delta_t = vehi_svc_header_time_i - vehi_svc_header_time_us_;
        UpdateVehicleService(delta_t, vehi_svc_msg_i);
      } else {
        break;
      }
    }
  }
}

void PlanningPlayer::UpdateVehicleService(
    uint64_t delta_t, struct_msgs::VehicleServiceOutputInfo::Ptr vehi_svc_msg) {
  const double dt = static_cast<double>(delta_t) / 1e6;
  const double v = v_t_spline_(dt);
  const double a = a_t_spline_(dt);
  const double yaw_rate = yaw_rate_t_spline_(dt);
  const double curvature = curvature_t_spline_(dt);

  // vel
  vehi_svc_msg->vehicle_speed = v;

  // acc
  vehi_svc_msg->long_acceleration = a;

  // yaw_rate
  vehi_svc_msg->yaw_rate = yaw_rate;

  // steering_angle = 1/R * L * steering_ratio * 补偿系数
  double compensation_factor = 1.0;
  double steer_ratio = 15.7;
  double wheel_base = 3.0;
  if (car_ == "CHERY_E0X") {
    compensation_factor = 1;
    steer_ratio = 13;
    wheel_base = 3.0;
  } else if (car_ == "JAC_S811") {
    compensation_factor = curvature > 0 ? 1.5 : 1;
    steer_ratio = 15.7;
    wheel_base = 2.7;
  }
  vehi_svc_msg->steering_wheel_angle =
      curvature * wheel_base * steer_ratio * compensation_factor;
}
// for apa
void PlanningPlayer::UpdateVehicleServiceDataAPA() {
  for (auto it = msg_cache_[TOPIC_VEHICLE_SERVICE].begin();
       it != msg_cache_[TOPIC_VEHICLE_SERVICE].end(); it++) {
    auto vehi_svc_msg_i =
        boost::any_cast<struct_msgs::VehicleServiceOutputInfo::Ptr>(it->second);
    auto vehi_svc_header_time_i = vehi_svc_msg_i->msg_header.stamp;
    if (vehi_svc_header_time_i > vehi_svc_header_time_us_) {
      if (vehi_svc_header_time_i <= next_vehi_svc_header_time_us_) {
        auto delta_t = vehi_svc_header_time_i - vehi_svc_header_time_us_;
        UpdateVehicleServiceAPA(delta_t, vehi_svc_msg_i);
      } else {
        break;
      }
    }
  }
}

void PlanningPlayer::UpdateVehicleServiceAPA(
    uint64_t delta_t, struct_msgs::VehicleServiceOutputInfo::Ptr vehi_svc_msg) {
  // TODO:
  //  1. how to compute curvature to change steering_wheel_angle
  //     warning: curvatur may change abruptly from -a to a,
  //     cause spline deriv failed
  //  2. how to change brake_pedal_pressed signal
  //     according to the state_.static_flag
  double curvature = pnc::mathlib::Limit(
      heading_s_spline_.deriv(1, state_.s_proj), KMaxCurvature);

  // steering_angle = 1/R * L * steering_ratio * 补偿系数
  double compensation_factor = 1.0;
  double steer_ratio = 15.7;
  double wheel_base = 3.0;
  if (car_ == "CHERY_E0X") {
    compensation_factor = 1;
    steer_ratio = 13;
    wheel_base = 3.0;
  } else if (car_ == "JAC_S811") {
    compensation_factor = curvature > 0 ? 1.5 : 1;
    steer_ratio = 15.7;
    wheel_base = 2.7;
  }
  vehi_svc_msg->steering_wheel_angle =
      curvature * wheel_base * steer_ratio * compensation_factor;

  vehi_svc_msg->brake_pedal_pressed = state_.static_flag;
}

void PlanningPlayer::GenMileage(const std::string& mileage_path) {
  if (mileage_path != "") {
    double pathLength = 0.0;
    if (scene_type_ == "scc" or scene_type_ == "noa" or scene_type_ == "hpp") {
      if (check_msg_exist(msg_cache_, TOPIC_LOCALIZATION)) {
        auto it_loc_msg = msg_cache_[TOPIC_LOCALIZATION].begin();
        for (size_t i = 0; i < msg_cache_[TOPIC_LOCALIZATION].size() - 1; ++i) {
          auto loc_msg_i = boost::any_cast<struct_msgs::IFLYLocalization::Ptr>(
              it_loc_msg->second);
          auto x1 = loc_msg_i->position.position_boot.x;
          auto y1 = loc_msg_i->position.position_boot.y;
          it_loc_msg++;
          auto loc_msg_i_next =
              boost::any_cast<struct_msgs::IFLYLocalization::Ptr>(
                  it_loc_msg->second);
          auto x2 = loc_msg_i_next->position.position_boot.x;
          auto y2 = loc_msg_i_next->position.position_boot.y;
          pathLength += sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
        }
      } else if (check_msg_exist(msg_cache_, TOPIC_LOCALIZATION_ESTIMATE)) {
        auto it_loc_esti_msg = msg_cache_[TOPIC_LOCALIZATION_ESTIMATE].begin();
        for (size_t i = 0;
             i < msg_cache_[TOPIC_LOCALIZATION_ESTIMATE].size() - 1; ++i) {
          auto loc_msg_i = boost::any_cast<
              struct_msgs_legacy_v2_4_6::LocalizationEstimate::Ptr>(
              it_loc_esti_msg->second);
          auto x1 = loc_msg_i->pose.local_position.x;
          auto y1 = loc_msg_i->pose.local_position.y;
          it_loc_esti_msg++;
          auto loc_msg_i_next = boost::any_cast<
              struct_msgs_legacy_v2_4_6::LocalizationEstimate::Ptr>(
              it_loc_esti_msg->second);
          auto x2 = loc_msg_i_next->pose.local_position.x;
          auto y2 = loc_msg_i_next->pose.local_position.y;
          pathLength += sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
        }
      }
    }
    nlohmann::json j;
    j["pathLength"] = pathLength;
    j["scene_type"] = scene_type_;
    std::ofstream outfile(mileage_path);
    outfile << j << std::endl;
    outfile.close();
  }
}

void PlanningPlayer::NoDebugInfoMode(bool is_close_loop, bool play_in_loop) {
  uint64_t start_time = 0;
  uint64_t end_time = 0;
  if (check_msg_exist(msg_cache_, TOPIC_LOCALIZATION)) {
    start_time = header_cache_[TOPIC_LOCALIZATION].begin()->first + 2E6;
    end_time = header_cache_[TOPIC_LOCALIZATION].rbegin()->first - 2E6;
  } else if (check_msg_exist(msg_cache_, TOPIC_LOCALIZATION_ESTIMATE)) {
    start_time =
        header_cache_[TOPIC_LOCALIZATION_ESTIMATE].begin()->first + 2E6;
    end_time = header_cache_[TOPIC_LOCALIZATION_ESTIMATE].rbegin()->first - 2E6;
  } else {
    std::cerr << "Error !!!!! missing localization msg" << std::endl;
    return;
  }

  const double init_start_time = start_time;
  while (start_time < end_time) {
    std::cout << "************************************** frame " << frame_num_
              << " **************************************" << std::endl;
    frame_num_++;
    start_time += 1E5;  // 0.1s
    auto fusion_object_ros_msg = find_ros_msg_with_header_time_upper_bound<
        struct_msgs::FusionObjectsInfo>(TOPIC_FUSION_OBJECTS, start_time);
    if (fusion_object_ros_msg) {
      iflyauto::FusionObjectsInfo fusion_object_msg{};
      convert(fusion_object_msg, *fusion_object_ros_msg,
              ConvertTypeInfo::TO_STRUCT);
      planning_adapter_->Feed_IflytekFusionObjects(fusion_object_msg);
    } else {
      std::cerr << "frame_num " << frame_num_
                << " missing /iflytek/fusion/objects" << std::endl;
    }

    auto fusion_occ_object_ros_msg = find_ros_msg_with_header_time_upper_bound<
        struct_msgs::FusionOccupancyObjectsInfo>(TOPIC_FUSION_OCCUPANCY_OBJECTS,
                                                 start_time);
    if (fusion_occ_object_ros_msg) {
      iflyauto::FusionOccupancyObjectsInfo fusion_occ_object_msg{};
      convert(fusion_occ_object_msg, *fusion_occ_object_ros_msg,
              ConvertTypeInfo::TO_STRUCT);
      planning_adapter_->Feed_IflytekFusionOccupancyObjects(fusion_occ_object_msg);
    } else {
      std::cerr << "frame_num " << frame_num_
                << " missing /iflytek/fusion/occupancy/objects" << std::endl;
    }

    auto fusion_road_ros_msg =
        find_ros_msg_with_header_time_upper_bound<struct_msgs::RoadInfo>(
            TOPIC_ROAD_FUSION, start_time);
    if (fusion_road_ros_msg) {
      iflyauto::RoadInfo fusion_road_msg{};
      convert(fusion_road_msg, *fusion_road_ros_msg,
              ConvertTypeInfo::TO_STRUCT);
      planning_adapter_->Feed_IflytekFusionRoadFusion(fusion_road_msg);
    } else {
      std::cerr << "frame_num " << frame_num_
                << " missing /iflytek/fusion/road_fusion" << std::endl;
    }

    // auto localization_estimate_ros_msg =
    //     find_ros_msg_with_header_time_upper_bound<
    //         struct_msgs_legacy_v2_4_6::LocalizationEstimate>(TOPIC_LOCALIZATION_ESTIMATE,
    //                                            start_time);
    // if (localization_estimate_ros_msg) {
    //   local_time_ = localization_estimate_ros_msg->msg_header.stamp;
    //   iflyauto::LocalizationEstimate localization_estimate_msg{};
    //   convert(localization_estimate_msg, *localization_estimate_ros_msg,
    //           ConvertTypeInfo::TO_STRUCT);
    //   planning_adapter_->FeedLocalizationEstimateOutput(
    //       localization_estimate_msg);
    // } else {
    //   // std::cerr << "frame_num " << frame_num_
    //   //           << " missing /iflytek/localization/ego_pose" <<
    //   std::endl;
    // }

    auto localization_ros_msg = find_ros_msg_with_header_time_upper_bound<
        struct_msgs::IFLYLocalization>(TOPIC_LOCALIZATION, start_time);
    if (localization_ros_msg) {
      local_time_ = localization_ros_msg->msg_header.stamp;
      iflyauto::IFLYLocalization localization_msg{};
      convert(localization_msg, *localization_ros_msg,
              ConvertTypeInfo::TO_STRUCT);
      planning_adapter_->Feed_IflytekLocalizationEgomotion(localization_msg);
    } else {
      // std::cerr << "frame_num " << frame_num_
      //           << " missing /iflytek/localization/egomotion" << std::endl;
    }

    auto prediction_ros_msg = find_ros_msg_with_header_time_upper_bound<
        struct_msgs::PredictionResult>(TOPIC_PREDICTION_RESULT, start_time);
    if (prediction_ros_msg) {
      iflyauto::PredictionResult prediction_msg{};
      convert(prediction_msg, *prediction_ros_msg, ConvertTypeInfo::TO_STRUCT);
      planning_adapter_->Feed_IflytekPredictionPredictionResult(prediction_msg);
    } else {
      // std::cerr << "frame_num " << frame_num_
      //           << " missing /iflytek/prediction/prediction_result" <<
      //           std::endl;
    }

    auto vehicle_service_ros_msg = find_ros_msg_with_header_time_upper_bound<
        struct_msgs::VehicleServiceOutputInfo>(TOPIC_VEHICLE_SERVICE,
                                               start_time);
    if (vehicle_service_ros_msg) {
      iflyauto::VehicleServiceOutputInfo vehicle_service_msg{};
      convert(vehicle_service_msg, *vehicle_service_ros_msg,
              ConvertTypeInfo::TO_STRUCT);
      planning_adapter_->Feed_IflytekVehicleService(vehicle_service_msg);
    } else {
      std::cerr << "frame_num " << frame_num_
                << " missing /iflytek/vehicle_service" << std::endl;
    }

    auto control_output_ros_msg =
        find_ros_msg_with_header_time_upper_bound<struct_msgs::ControlOutput>(
            TOPIC_CONTROL_COMMAN, start_time);
    if (control_output_ros_msg) {
      iflyauto::ControlOutput control_output_msg{};
      convert(control_output_msg, *control_output_ros_msg,
              ConvertTypeInfo::TO_STRUCT);
      planning_adapter_->Feed_IflytekControlControlCommand(control_output_msg);
    } else {
      // std::cerr << "missing /iflytek/control/control_command" << std::endl;
    }

    auto hmi_mcu_ros_msg = find_ros_msg_with_header_time_upper_bound<
        struct_msgs_legacy_v2_4_5::HmiMcuInner>(TOPIC_HMI_MCU_INNER,
                                                start_time);
    if (hmi_mcu_ros_msg) {
      iflyauto::interface_2_4_5::HmiMcuInner hmi_mcu_msg{};
      convert(hmi_mcu_msg, *hmi_mcu_ros_msg, ConvertTypeInfo::TO_STRUCT);
      planning_adapter_->FeedHmiMcuInner(hmi_mcu_msg);
    } else {
      // std::cerr << "missing /iflytek/hmi/mcu_inner" << std::endl;
    }

    auto parking_fusion_ros_msg = find_ros_msg_with_header_time_upper_bound<
        struct_msgs::ParkingFusionInfo>(TOPIC_PARKING_FUSION, start_time);
    if (parking_fusion_ros_msg) {
      iflyauto::ParkingFusionInfo parking_fusion_msg{};
      convert(parking_fusion_msg, *parking_fusion_ros_msg,
              ConvertTypeInfo::TO_STRUCT);
      planning_adapter_->Feed_IflytekFusionParkingSlot(parking_fusion_msg);
    } else {
      // std::cerr << "frame_num " << frame_num_
      //           << " missing /iflytek/fusion/parking_slot" << std::endl;
    }

    // apa module
    auto uss_wave_ros_msg =
        find_ros_msg_with_header_time_upper_bound<struct_msgs::UssWaveInfo>(
            TOPIC_USS_WAVE_INFO, start_time);
    if (uss_wave_ros_msg) {
      iflyauto::UssWaveInfo uss_wave_msg{};
      convert(uss_wave_msg, *uss_wave_ros_msg, ConvertTypeInfo::TO_STRUCT);
      planning_adapter_->Feed_IflytekUssUsswaveInfo(uss_wave_msg);
    } else {
      std::cerr << "frame_num " << frame_num_
                << " missing /iflytek/uss/wave_info" << std::endl;
    }

    auto uss_percept_ros_msg =
        find_ros_msg_with_header_time_upper_bound<struct_msgs::UssPerceptInfo>(
            TOPIC_USS_PERCEPT_INFO, start_time);
    if (uss_percept_ros_msg) {
      iflyauto::UssPerceptInfo uss_percept_msg{};
      convert(uss_percept_msg, *uss_percept_ros_msg,
              ConvertTypeInfo::TO_STRUCT);
      planning_adapter_->Feed_IflytekUssUssPerceptionInfo(uss_percept_msg);
    } else {
      std::cerr << "frame_num " << frame_num_
                << " missing /iflytek/uss/uss_perception_info" << std::endl;
    }

    // auto hd_map_ros_msg =
    // find_ros_msg_with_header_time_upper_bound<proto_msgs::StaticMap>(
    //     TOPIC_HD_MAP, start_time);
    // if (hd_map_ros_msg) {
    //   std::shared_ptr<Map::StaticMap> hd_map_msg;
    //   StaticMapToProto(*hd_map_msg, *hd_map_ros_msg);
    //   planning_adapter_->FeedMap(hd_map_msg);
    // } else {
    //   std::cerr << "frame_num " << frame_num_
    //             << " missing /iflytek/ehr/static_map" << std::endl;
    // }

    // TODO: for hpp, need FeedParkingMap() ready
    // auto ehr_parking_map_ros_msg =
    //     find_ros_msg_with_header_time_upper_bound<struct_msgs::ParkingInfo>(
    //         TOPIC_EHR_PARKING_MAP, input_time_list.ehr_parking_map());
    // if (ehr_parking_map_ros_msg) {
    //   iflyauto::ParkingInfo ehr_parking_map_msg{};
    //   convert(ehr_parking_map_msg, *ehr_parking_map_ros_msg,
    //   ConvertTypeInfo::TO_STRUCT);
    //   planning_adapter_->FeedParkingMap(ehr_parking_map_msg);
    // } else {
    //   // std::cerr << "frame_num " << frame_num_
    //   //           << " missing /iflytek/ehr/parking_map" << std::endl;
    // }

    // auto ground_line_ros_msg = find_ros_msg_with_header_time_upper_bound<
    //     struct_msgs::GroundLinePerceptionInfo>(TOPIC_GROUND_LINE,
    //     start_time);
    // if (ground_line_ros_msg) {
    //   iflyauto::GroundLinePerceptionInfo ground_line_msg{};
    //   convert(ground_line_msg, *ground_line_ros_msg,
    //           ConvertTypeInfo::TO_STRUCT);
    //   planning_adapter_->FeedGroundLinePerception(ground_line_msg);
    // } else {
    //   // std::cerr << "frame_num " << frame_num_
    //   //           << " missing /iflytek/fusion/ground_line" << std::endl;
    // }

    if (check_msg_exist(msg_cache_, TOPIC_FUNC_STATE_MACHINE)) {
      bool find_function_state_machine = false;
      struct_msgs::FuncStateMachine func_state_machine_ros_msg{};
      uint8_t functional_state = iflyauto::FunctionalState_MANUAL;

      auto cached_func_state_machine_ros_msg =
          find_ros_msg_with_header_time_upper_bound<
              struct_msgs::FuncStateMachine>(TOPIC_FUNC_STATE_MACHINE,
                                             start_time);
      if (cached_func_state_machine_ros_msg) {
        func_state_machine_ros_msg = *cached_func_state_machine_ros_msg;
        find_function_state_machine = true;
      } else {
        std::cerr << "frame_num " << frame_num_
                  << " missing /iflytek/fsm/soc_state" << std::endl;
      }

      if (frame_num_ >=
          frame_num_before_enter_auto_) {  // enter auto after 1.5s
        if (scene_type_ == "acc") {
          functional_state = iflyauto::FunctionalState_ACC_ACTIVATE;
        } else if (scene_type_ == "apa") {
          if (find_function_state_machine) {
            if (is_close_loop) {
              if (last_functional_state !=
                      iflyauto::FunctionalState_PARK_IN_SEARCHING &&
                  last_functional_state !=
                      iflyauto::FunctionalState_PARK_GUIDANCE) {
                last_functional_state =
                    iflyauto::FunctionalState_PARK_IN_SEARCHING;
              } else {
                last_functional_state = iflyauto::FunctionalState_PARK_GUIDANCE;
              }
              functional_state = last_functional_state;
            } else {
              functional_state = func_state_machine_ros_msg.current_state;
            }
          } else {
            functional_state = iflyauto::FunctionalState_PARK_GUIDANCE;
          }
        } else if (scene_type_ == "scc" || scene_type_ == "noa") {
          if (find_function_state_machine) {
            if (is_close_loop) {
              if (iflyauto::FunctionalState_SCC_STANDBY <=
                      func_state_machine_ros_msg.current_state &&
                  func_state_machine_ros_msg.current_state <=
                      iflyauto::FunctionalState_SCC_OVERRIDE) {
                functional_state = iflyauto::FunctionalState_SCC_ACTIVATE;
              } else if (iflyauto::FunctionalState_NOA_STANDBY <=
                             func_state_machine_ros_msg.current_state &&
                         func_state_machine_ros_msg.current_state <=
                             iflyauto::FunctionalState_NOA_OVERRIDE) {
                functional_state = iflyauto::FunctionalState_NOA_ACTIVATE;
              } else if (iflyauto::FunctionalState_ACC_STANDBY <=
                             func_state_machine_ros_msg.current_state &&
                         func_state_machine_ros_msg.current_state <=
                             iflyauto::FunctionalState_ACC_OVERRIDE) {
                functional_state = iflyauto::FunctionalState_NOA_ACTIVATE;
              } else {
                functional_state = last_functional_state;
              }
            } else {
              functional_state = func_state_machine_ros_msg.current_state;
            }
          } else if (scene_type_ == "scc") {
            functional_state = iflyauto::FunctionalState_SCC_ACTIVATE;
          } else if (scene_type_ == "noa") {
            functional_state = iflyauto::FunctionalState_NOA_ACTIVATE;
          }
        } else if (scene_type_ == "hpp") {
          functional_state = iflyauto::FunctionalState_HPP_IN_MEMORY_CRUISE;
        }
      }

      func_state_machine_ros_msg.current_state = functional_state;
      last_functional_state = functional_state;
      iflyauto::FuncStateMachine func_state_machine_msg{};
      convert(func_state_machine_msg, func_state_machine_ros_msg,
              ConvertTypeInfo::TO_STRUCT);
      planning_adapter_->Feed_IflytekFsmSocState(func_state_machine_msg);
    } else {
      std::cerr << "Error !!!!! missing FUNC_STATE_MACHINE" << std::endl;
      return;
    }

    planning_adapter_->Proc();

    if (play_in_loop) {
      if (start_time >= end_time) {
        start_time = init_start_time;
      }
    }
  }
}

}  // namespace planning_player
}  // namespace planning