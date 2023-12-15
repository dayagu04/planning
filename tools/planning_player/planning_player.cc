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
#include "math_lib.h"
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
static constexpr auto TOPIC_LOCALIZATION = "/iflytek/localization/egomotion";
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
void PlanningPlayer::Init(bool is_close_loop, int auto_frame,
                          const std::string& scene_type) {
  std::cout << "===========planning player init, is_close_loop="
            << is_close_loop << ", auto_frame=" << auto_frame
            << ", scene_type=" << scene_type << "==========" << std::endl;
  enter_auto_frame_num_ = auto_frame;
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
        if (is_close_loop) {
          RunCloseLoop(planning_output);
        }
      });

  planning_adapter_->RegisterDebugInfoWriter(
      [this](const planning::common::PlanningDebugInfo& planning_debug_info) {
        auto planning_debug_info_ptr =
            std::make_shared<planning::common::PlanningDebugInfo>(
                planning_debug_info);
        planning_debug_info_ptr->set_timestamp(planning_header_time_us_);
        output_msg_cache_[TOPIC_PLANNING_DEBUG_INFO][planning_msg_time_ns_] =
            planning_debug_info_ptr;
      });

  planning_adapter_->RegisterHMIOutputInfoWriter(
      [this](const PlanningHMI::PlanningHMIOutputInfoStr&
                 planning_hmi_ouput_info) {
        output_msg_cache_[TOPIC_PLANNING_HMI][planning_msg_time_ns_] =
            std::make_shared<PlanningHMI::PlanningHMIOutputInfoStr>(
                planning_hmi_ouput_info);
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
  enter_auto_frame_num_ = 0;
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
    } else if (msg.channel_name == TOPIC_LOCALIZATION) {
      cache_with_msg_and_header_time<IFLYLocalization::IFLYLocalization>(msg);
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
    } else if (msg.channel_name == TOPIC_EHR_PARKING_MAP) {
      cache_with_msg_and_header_time<IFLYParkingMap::ParkingInfo>(msg);
    } else if (msg.channel_name == TOPIC_GROUND_LINE) {
      cache_with_msg_and_header_time<
          GroundLinePerception::GroundLinePerceptionInfo>(msg);
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
  write_topic_msg<FusionObjects::FusionObjectsInfo>(msg_cache_, record_writer,
                                                    TOPIC_FUSION_OBJECTS);
  write_topic_msg<FusionRoad::RoadInfo>(msg_cache_, record_writer,
                                        TOPIC_ROAD_FUSION);
  write_topic_msg<LocalizationOutput::LocalizationEstimate>(
      msg_cache_, record_writer, TOPIC_LOCALIZATION_ESTIMATE);
  write_topic_msg<IFLYLocalization::IFLYLocalization>(msg_cache_, record_writer,
                                                      TOPIC_LOCALIZATION);
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
  write_topic_msg<IFLYParkingMap::ParkingInfo>(msg_cache_, record_writer,
                                               TOPIC_EHR_PARKING_MAP);
  write_topic_msg<GroundLinePerception::GroundLinePerceptionInfo>(
      msg_cache_, record_writer, TOPIC_GROUND_LINE);
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
  }

  auto fusion_road_msg = find_msg_with_header_time<FusionRoad::RoadInfo>(
      TOPIC_ROAD_FUSION, input_time_list.fusion_road());
  if (fusion_road_msg) {
    planning_adapter_->FeedFusionRoad(fusion_road_msg);
  }

  auto localization_estimate_msg =
      find_msg_with_header_time<LocalizationOutput::LocalizationEstimate>(
          TOPIC_LOCALIZATION_ESTIMATE, input_time_list.localization_estimate());
  if (localization_estimate_msg) {
    planning_adapter_->FeedLocalizationEstimateOutput(
        localization_estimate_msg);
  }

  auto localization_msg =
      find_msg_with_header_time<IFLYLocalization::IFLYLocalization>(
          TOPIC_LOCALIZATION, input_time_list.localization());
  if (localization_msg) {
    planning_adapter_->FeedLocalizationOutput(localization_msg);
  }

  auto prediction_msg = find_msg_with_header_time<Prediction::PredictionResult>(
      TOPIC_PREDICTION_RESULT, input_time_list.prediction());
  if (prediction_msg) {
    planning_adapter_->FeedPredictionResult(prediction_msg);
  }

  auto vehicle_service_msg =
      find_msg_with_header_time<VehicleService::VehicleServiceOutputInfo>(
          TOPIC_VEHICLE_SERVICE, input_time_list.vehicle_service());
  if (vehicle_service_msg) {
    planning_adapter_->FeedVehicleService(vehicle_service_msg);
  }

  auto control_output_msg =
      find_msg_with_header_time<ControlCommand::ControlOutput>(
          TOPIC_CONTROL_COMMAN, input_time_list.control_output());
  if (control_output_msg) {
    planning_adapter_->FeedControlCommand(control_output_msg);
  }

  auto hmi_mcu_msg = find_msg_with_header_time<HmiMcuInner::HmiMcuInner>(
      TOPIC_HMI_MCU_INNER, input_time_list.hmi());
  if (hmi_mcu_msg) {
    planning_adapter_->FeedHmiMcuInner(hmi_mcu_msg);
  }

  auto parking_fusion_msg =
      find_msg_with_header_time<ParkingFusion::ParkingFusionInfo>(
          TOPIC_PARKING_FUSION, input_time_list.hmi());
  if (parking_fusion_msg) {
    planning_adapter_->FeedParkingFusion(parking_fusion_msg);
  }

  auto hd_map_msg = find_msg_with_header_time<Map::StaticMap>(
      TOPIC_HD_MAP, input_time_list.map());
  if (hd_map_msg) {
    planning_adapter_->FeedMap(hd_map_msg);
  }

  auto ehr_parking_map_msg =
      find_msg_with_header_time<IFLYParkingMap::ParkingInfo>(
          TOPIC_EHR_PARKING_MAP, input_time_list.ehr_parking_map());
  if (ehr_parking_map_msg) {
    planning_adapter_->FeedParkingMap(ehr_parking_map_msg);
  }

  auto ground_line_msg =
      find_msg_with_header_time<GroundLinePerception::GroundLinePerceptionInfo>(
          TOPIC_EHR_PARKING_MAP, input_time_list.ground_line());
  if (ground_line_msg) {
    planning_adapter_->FeedGroundLinePerception(ground_line_msg);
  }

  auto func_state_machine_msg =
      std::make_shared<FuncStateMachine::FuncStateMachine>();
  if (input_time_list.has_function_state_machine()) {
    auto cached_func_state_machine_msg =
        find_msg_with_header_time<FuncStateMachine::FuncStateMachine>(
            TOPIC_FUNC_STATE_MACHINE, input_time_list.function_state_machine());
    if (cached_func_state_machine_msg) {
      func_state_machine_msg = cached_func_state_machine_msg;
    }
  }
  auto functional_state = ::FuncStateMachine::FunctionalState::INIT;
  if (frame_num >= enter_auto_frame_num_) {  // enter auto after 1.5s
    if (scene_type_ == "acc") {
      functional_state = ::FuncStateMachine::FunctionalState::ACC_ACTIVATE;
    } else if (scene_type_ == "apa") {
      functional_state =
          ::FuncStateMachine::FunctionalState::PARK_IN_ACTIVATE_CONTROL;
    } else if (scene_type_ == "hpp") {
      functional_state =
          ::FuncStateMachine::FunctionalState::HPP_IN_MEMORY_CRUISE;
    }
  }
  func_state_machine_msg->set_current_state(functional_state);
  planning_adapter_->FeedFuncStateMachine(func_state_machine_msg);

  planning_adapter_->Proc();
}

void PlanningPlayer::PlayAllFrames() {
  auto& debug_info_cache = msg_cache_[TOPIC_PLANNING_DEBUG_INFO];
  int frame_num = 0;
  for (auto it : debug_info_cache) {
    auto debug_info =
        std::dynamic_pointer_cast<planning::common::PlanningDebugInfo>(
            it.second);
    if (!debug_info->has_input_topic_timestamp()) {
      std::cerr << "no input topic timestamp" << std::endl;
      continue;
    }
    planning_header_time_us_ = debug_info->timestamp();
    planning_msg_time_ns_ = it.first;
    loc_header_time_us_ = debug_info->input_topic_timestamp().localization();
    PlayOneFrame(frame_num++, debug_info->input_topic_timestamp());
  }
}

void PlanningPlayer::RunCloseLoop(
    const PlanningOutput::PlanningOutput& planning_output) {
  if (1) {  // HPP
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
          std::const_pointer_cast<IFLYLocalization::IFLYLocalization>(
              std::dynamic_pointer_cast<IFLYLocalization::IFLYLocalization>(
                  it->second));
      auto loc_header_time_i = loc_msg_i->header().timestamp();
      if (loc_header_time_i > loc_header_time_us_ &&
          loc_header_time_i < loc_header_time_us_ + 1000 * 1000) {
        auto delta_t = loc_header_time_i - loc_header_time_us_;
        PerfectControlHPP(planning_output, delta_t, loc_msg_i);
      }
    }
  } else {  // APA
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

void PlanningPlayer::PerfectControlHPP(
    const PlanningOutput::PlanningOutput& plan_msg, uint64_t delta_t,
    std::shared_ptr<IFLYLocalization::IFLYLocalization>& loc_msg) {
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

  // auto pose = loc_msg->mutable_pose();

  // vel
  loc_msg->mutable_velocity()->mutable_velocity_boot()->set_vx(v);
  // acc
  loc_msg->mutable_acceleration()->mutable_acceleration_boot()->set_ax(a);
  // atti
  loc_msg->mutable_orientation()->mutable_euler_boot()->set_yaw(euler_zxy[0]);
  // pose->set_heading(pose->euler_angles().yaw());

  loc_msg->mutable_orientation()->mutable_quaternion_boot()->set_w(q.w());
  loc_msg->mutable_orientation()->mutable_quaternion_boot()->set_x(q.x());
  loc_msg->mutable_orientation()->mutable_quaternion_boot()->set_y(q.y());
  loc_msg->mutable_orientation()->mutable_quaternion_boot()->set_z(q.z());

  // pos
  loc_msg->mutable_position()->mutable_position_boot()->set_x(x);
  loc_msg->mutable_position()->mutable_position_boot()->set_y(y);
}

void PlanningPlayer::PerfectControlAPA(
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