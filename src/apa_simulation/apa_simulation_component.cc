#include "apa_simulation_component.h"

#include <cmath>
#include <string.h>

#include "apa_simulation/common/apa_sim_log_helper.h"
#include "common/utils/file.h"

namespace planning {

bool ApaSimulationComponent::Init() {
  APA_SIM_LOG << "The apa simulation component init!!!" << std::endl;

  const std::string apa_simulation_config_path =
      "/asw/Planning/Res/config/apa_simulation_config.pb.txt";

  common::util::GetProtoFromFile(apa_simulation_config_path, &apa_sim_config_);

  // 1.定义cyber node
  ADSNode::Init("simulation_node");
  simulation_node_ = std::make_shared<ADSNode>("simulation_node");

  // 2.定义收发topics
  auto planning_output_reader =
      simulation_node_->CreateReader<PlanningOutput>(
          "/planning",
          [this](const std::shared_ptr<PlanningOutput> &planning_msg) {
            std::lock_guard<std::mutex> lock(msg_mutex_);
            planning_output_msg_.CopyFrom(*planning_msg);
          });

  // -------------- writter topics --------------
  localization_estimate_writer_ =
      simulation_node_->CreateWriter<LocalizationEstimate>("/localization");
  parking_fusion_info_writer_ =
      simulation_node_->CreateWriter<ParkingFusionInfo>(
        "/iflytek/fusion/parking_slot");
  vehicle_service_output_info_writer_ =
      simulation_node_->CreateWriter<VehicleServiceOutputInfo>(
        "/vehicel_service_output_info");

  return true;
}

bool ApaSimulationComponent::Proc() {
  APA_SIM_LOG << "=======Apa simulation component running=======" << std::endl;
  uint64_t start_time = IflyTime::Now_ms();
  APA_SIM_LOG << "start time(ms):" << start_time << std::endl;

  MockParkingFusionInfo();
  MockLocalizationAndVehicleService();

  const uint64_t simulation_cost_time = IflyTime::Now_ms() - start_time;
  APA_SIM_LOG << "time cost(ms):" << simulation_cost_time << std::endl;

  return true;
}

void ApaSimulationComponent::MockLocalizationAndVehicleService() {
  ego_motion_msg_.Clear();
  {
    cur_planning_output_ = planning_output_msg_;
  }

  if (last_planning_gear_
      != cur_planning_output_.gear_command().gear_command_value()) {
    traj_pt_index_ = 0;
  }

  double ego_x = apa_sim_config_.init_x();
  double ego_y = apa_sim_config_.init_y();
  double ego_theta = apa_sim_config_.init_theta();
  double ego_spd = apa_sim_config_.init_speed();
  if (cur_planning_output_.trajectory().trajectory_points_size() > 1) {
    traj_pt_index_ = std::min(traj_pt_index_,
        cur_planning_output_.trajectory().trajectory_points_size() - 1);
    const auto& traj_points =
        cur_planning_output_.trajectory().trajectory_points();
    ego_x = traj_points[traj_pt_index_].x();
    ego_y = traj_points[traj_pt_index_].y();
    ego_theta = traj_points[traj_pt_index_].heading_yaw();
    ego_spd = fabs(traj_points[traj_pt_index_].v());
    ++traj_pt_index_;
  } else if (cur_planning_output_.trajectory().trajectory_points_size() == 1) {
    ego_x = last_ego_x_;
    ego_y = last_ego_y_;
    ego_theta = last_ego_theta_;
    ego_spd = last_ego_spd_;
  }


  LocalizationEstimate localization_estimate_msg;
  localization_estimate_msg.mutable_pose()->set_type(
      LocalizationOutput::Pose::LOCATION_LOCAL);
  auto local_position =
      localization_estimate_msg.mutable_pose()->mutable_local_position();
  local_position->set_x(ego_x);
  local_position->set_y(ego_y);
  local_position->set_z(0.0);
  auto euler_angles =
      localization_estimate_msg.mutable_pose()->mutable_euler_angles();
  euler_angles->set_yaw(ego_theta);
  euler_angles->set_pitch(0.0);
  euler_angles->set_roll(0.0);
  ego_motion_writer_->Write(localization_estimate_msg);

  VehicleServiceOutputInfo vehicle_service_output_info;
  vehicle_service_output_info.set_veh_spd(vehicle_speed);
  vehicle_service_output_info.set_vehicle_speed_available(true);
  vehicle_service_output_info_writer_->Write(localization_estimate_msg);

  last_planning_gear_ =
      cur_planning_output_.gear_command().gear_command_value();
  last_ego_x_ = ego_x;
  last_ego_y_ = ego_y;
  last_ego_theta_ = ego_theta;
  last_ego_spd_ = ego_spd;
}

void ApaSimulationComponent::MockParkingFusionInfo() {
  // mock parking fusion
  parking_fusion_info_writer_->Write(apa_sim_config_.parking_fusion_info());
}

}  // namespace planning
