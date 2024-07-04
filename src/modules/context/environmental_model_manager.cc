#include "environmental_model_manager.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <typeinfo>
#include <unordered_map>

#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/util/Constants.h"
#include "agent/agent_manager.h"
#include "basic_types.pb.h"
#include "common_platform_type_soc.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "ego_planning_config.h"
#include "ego_state_manager.h"
#include "frenet_ego_state.h"
#include "general_planning_context.h"
#include "history_obstacle_manager.h"
#include "ifly_localization_c.h"
#include "lateral_obstacle.h"
#include "log.h"
#include "math/linear_interpolation.h"
#include "obstacle_manager.h"
#include "planning_context.h"
#include "reference_path_manager.h"
#include "scene_type_config.pb.h"
#include "traffic_light_decision_manager.h"
#include "vehicle_model/vehicle_model.h"
#include "vehicle_service_c.h"
#include "vehicle_status.pb.h"
#include "virtual_lane_manager.h"

namespace planning {
namespace planner {

EnvironmentalModelManager::EnvironmentalModelManager() {
  LOG_DEBUG("EnvironmentalModelManager created\n");
}

void EnvironmentalModelManager::Init(planning::framework::Session *session) {
  session_ = session;
  InitContext();
}

EgoPlanningConfigBuilder *EnvironmentalModelManager::load_config_builder(
    const char *file_name) {
  auto config_file_dir =
      session_->environmental_model().get_module_config_file_dir();
  auto ego_planning_config_json_file = config_file_dir + "/" + file_name;
  LOG_DEBUG("%s\n", ego_planning_config_json_file.c_str());

  Json ego_planning_config_json;
  std::ifstream fin(ego_planning_config_json_file);
  fin >> ego_planning_config_json;
  fin.close();

  return session_->alloc<EgoPlanningConfigBuilder>(ego_planning_config_json,
                                                   file_name);
}

void EnvironmentalModelManager::InitContext() {
  auto parking_config_builder =
      load_config_builder("general_planner_module_parking.json");
  session_->mutable_environmental_model()->set_parking_config_builder(
      parking_config_builder);

  auto highway_config_builder =
      load_config_builder("general_planner_module_highway.json");
  session_->mutable_environmental_model()->set_highway_config_builder(
      highway_config_builder);

  auto hpp_config_builder =
      load_config_builder("general_planner_module_hpp.json");
  session_->mutable_environmental_model()->set_hpp_config_builder(
      hpp_config_builder);

  planning::common::SceneType scene_type = session_->get_scene_type();
  auto config_builder =
      session_->environmental_model().config_builder(scene_type);
  ego_config_ = config_builder->cast<planning::EgoPlanningConfig>();

  ego_state_manager_ptr_ =
      std::make_shared<planning::EgoStateManager>(config_builder, session_);
  session_->mutable_environmental_model()->set_ego_state(
      ego_state_manager_ptr_);

  virtual_lane_manager_ptr_ =
      std::make_shared<planning::VirtualLaneManager>(config_builder, session_);
  session_->mutable_environmental_model()->set_virtual_lane_manager(
      virtual_lane_manager_ptr_);

  obstacle_manager_ptr_ =
      std::make_shared<planning::ObstacleManager>(config_builder, session_);
  session_->mutable_environmental_model()->set_obstacle_manager(
      obstacle_manager_ptr_);

  reference_path_manager_ptr_ =
      std::make_shared<planning::ReferencePathManager>(session_);
  session_->mutable_environmental_model()->set_reference_path_manager(
      reference_path_manager_ptr_);

  lateral_obstacle_ptr_ =
      std::make_shared<planning::LateralObstacle>(config_builder, session_);
  session_->mutable_environmental_model()->set_lateral_obstacle(
      lateral_obstacle_ptr_);

  lane_tracks_mgr_ptr_ = std::make_shared<LaneTracksManager>(
      *lateral_obstacle_ptr_, *virtual_lane_manager_ptr_, session_);
  session_->mutable_environmental_model()->set_lane_tracks_manager(
      lane_tracks_mgr_ptr_);
  // agent node manager
  agent_node_mgr_ptr_ = std::make_shared<AgentNodeManager>();
  session_->mutable_environmental_model()->set_agent_node_manager(
      agent_node_mgr_ptr_);

  // parking_slot_manager_ptr_ = std::make_shared<ParkingSlotManager>(session_);
  // session_->mutable_environmental_model()->set_parking_slot_manager(
  //     parking_slot_manager_ptr_);
  history_obstacle_ptr_ = std::make_shared<planning::HistoryObstacleManager>(
      config_builder, session_);
  session_->mutable_environmental_model()->set_history_obstacle_manager(
      history_obstacle_ptr_);

  const std::string &config_file_dir =
      session_->mutable_environmental_model()->get_module_config_file_dir();
  common::VehicleModel::LoadVehicleModelConfig(config_file_dir);
  // new agent manager
  agent_manager_ptr_ =
      std::make_shared<agent::AgentManager>(config_builder, session_);
  session_->mutable_environmental_model()->set_agent_manager(
      agent_manager_ptr_);

  dynamic_world_ = std::make_shared<planning_data::DynamicWorld>(
      *agent_manager_ptr_, session_);
  session_->mutable_environmental_model()->set_dynamic_world(dynamic_world_);
}

bool EnvironmentalModelManager::Run() {
  LOG_DEBUG("EnvironmentalModelManager run\n");

  auto current_time = IflyTime::Now_ms();

  if (!session_->environmental_model().GetVehicleDbwStatus()) {
    LOG_WARNING("DBW_Disable, but EnvironmentalModelManager continue\n");
  }

  auto &local_view = session_->environmental_model().get_local_view();

  // 通过配置项进行实时长时的切换 true: 长时规划
  // bool msf_valid = local_view.localization_estimate.msf_status.available &&
  //                  (local_view.localization_estimate.msf_status.msf_status !=
  //                   iflyauto::MsfStatusType_ERROR);
  bool localization_valid =
      local_view.localization.status.status_info.mode !=
      iflyauto::IFLYStatusInfoMode::IFLY_STATUS_INFO_MODE_ERROR;
  bool fusion_localization_valid =
      local_view.road_info.local_point_valid &&
      local_view.fusion_objects_info.local_point_valid;
  bool planner_valid = g_context.GetParam().planner_type ==
                           planning::context::PlannerType::SCC_PLANNER ||
                       g_context.GetParam().planner_type ==
                           planning::context::PlannerType::LONGTIME_PLANNER ||
                       g_context.GetParam().planner_type ==
                           planning::context::PlannerType::HPP_PLANNER;
  printf("planner_type:%d\n", g_context.GetParam().planner_type);
  auto location_valid = /*msf_valid*/ localization_valid &&
                        fusion_localization_valid && planner_valid;

  if (session_->is_hpp_scene() && !location_valid) {
    LOG_ERROR("hpp location invalid\n");
    return false;
  }
  // location_valid = true; //hack

  auto environmental_model = session_->mutable_environmental_model();
  environmental_model->set_location_valid(location_valid);
  // Step 1) update vehicleDbwStatus
  auto fsm_state = local_view.function_state_machine_info.current_state;
  bool acc_mode = (fsm_state == iflyauto::FunctionalState_ACC_ACTIVATE) ||
                  (fsm_state == iflyauto::FunctionalState_ACC_OVERRIDE);
  bool scc_mode = (fsm_state == iflyauto::FunctionalState_SCC_ACTIVATE) ||
                  (fsm_state == iflyauto::FunctionalState_SCC_OVERRIDE);
  bool noa_mode = (fsm_state == iflyauto::FunctionalState_NOA_ACTIVATE) ||
                  (fsm_state == iflyauto::FunctionalState_NOA_OVERRIDE);
  bool dbw_status = acc_mode || scc_mode || noa_mode;
  environmental_model->UpdateVehicleDbwStatus(dbw_status);
  JSON_DEBUG_VALUE("dbw_status", dbw_status)

  common::DrivingFunctionInfo::DrivingFunctionstate function_state =
      common::DrivingFunctionInfo::ACTIVATE;
  if (fsm_state == iflyauto::FunctionalState_ACC_ACTIVATE ||
      fsm_state == iflyauto::FunctionalState_SCC_ACTIVATE ||
      fsm_state == iflyauto::FunctionalState_NOA_ACTIVATE) {
    function_state = common::DrivingFunctionInfo::ACTIVATE;
  } else if (fsm_state == iflyauto::FunctionalState_ACC_OVERRIDE ||
             fsm_state == iflyauto::FunctionalState_SCC_OVERRIDE ||
             fsm_state == iflyauto::FunctionalState_NOA_OVERRIDE) {
    function_state = common::DrivingFunctionInfo::OVERRIDE;
  }

  if (scc_mode) {
    environmental_model->set_function_info(common::DrivingFunctionInfo::SCC,
                                           function_state);
  } else if (acc_mode) {
    environmental_model->set_function_info(common::DrivingFunctionInfo::ACC,
                                           function_state);
  } else if (noa_mode) {
    environmental_model->set_function_info(common::DrivingFunctionInfo::NOA,
                                           function_state);
  } else {
    LOG_NOTICE("function mode error\n");
    environmental_model->set_function_info(common::DrivingFunctionInfo::ACC,
                                           function_state);
  }

  // 自动有效，临时hack
  // session_->mutable_environmental_model()->UpdateVehicleDbwStatus(true);
  last_feed_time_[FEED_VEHICLE_DBW_STATUS] =
      local_view.function_state_machine_info_recv_time;

  // Step 2) update ego_state
  auto time_start = IflyTime::Now_ms();
  if (!ego_state_update(current_time, local_view)) {
    LOG_ERROR("ego_state_update false\n");
    return false;
  }
  auto time_end = IflyTime::Now_ms();
  LOG_DEBUG("ego_state_update cost:%f\n", time_end - time_start);
  JSON_DEBUG_VALUE("ego_state_update_cost", time_end - time_start)

  // // Step 3) update static map info
  // environmental_model->UpdateStaticMap(local_view);

  //  Step 3) update static map info
  environmental_model->UpdateSdMap(local_view);

  // Step 4) update virtual_lane
  time_start = IflyTime::Now_ms();
  last_feed_time_[FEED_MAP_INFO] = local_view.static_map_info_recv_time;
  if (!virtual_lane_manager_ptr_->update(local_view.road_info)) {
    LOG_ERROR("virtual_lane_manager update failed\n");
    return false;
  } else {
    // 后面需要判断是否为地图
    last_feed_time_[FEED_FUSION_LANES_INFO] = local_view.road_info_recv_time;
  }
  time_end = IflyTime::Now_ms();
  LOG_DEBUG("virtual_lane_manager update cost:%f\n", time_end - time_start);
  JSON_DEBUG_VALUE("virtual_lane_manager_update_cost", time_end - time_start);

  // Step 5) update obstacle
  time_start = IflyTime::Now_ms();
  if (!obstacle_prediction_update(current_time, local_view)) {
    return false;
  }
  time_end = IflyTime::Now_ms();
  LOG_DEBUG("obstacle_prediction update cost:%f\n", time_end - time_start);
  JSON_DEBUG_VALUE("obstacle_prediction_update_cost", time_end - time_start);

  if (session_->is_hpp_scene()) {
    time_start = IflyTime::Now_ms();
    ground_line_obstacles_update(local_view);
    time_end = IflyTime::Now_ms();
    LOG_DEBUG("ground_line_obstacles update cost:%f\n", time_end - time_start);
    JSON_DEBUG_VALUE("ground_line_obstacles_cost", time_end - time_start);

    time_start = IflyTime::Now_ms();
    // parking_slot_manager_ptr_->update(local_view.parking_map_info);
    time_end = IflyTime::Now_ms();
    LOG_DEBUG("parking_slot_manager update cost:%f\n", time_end - time_start);
    JSON_DEBUG_VALUE("parking_slot_manager_cost", time_end - time_start);
  }

  time_start = IflyTime::Now_ms();
  obstacle_manager_ptr_->update();
  time_end = IflyTime::Now_ms();
  LOG_DEBUG("obstacle_manager cost:%f\n", time_end - time_start);
  JSON_DEBUG_VALUE("obstacle_manager_cost", time_end - time_start);

  time_start = IflyTime::Now_ms();
  agent_manager_ptr_->Update();
  time_end = IflyTime::Now_ms();
  LOG_DEBUG("agent manager cost:%f\n", time_end - time_start);
  JSON_DEBUG_VALUE("agent_manager_cost", time_end - time_start);

  // Step 5) update reference path
  time_start = IflyTime::Now_ms();
  if (!reference_path_manager_ptr_->update()) {
    LOG_ERROR("reference_path_manager update fail\n");
    return false;
  }
  time_end = IflyTime::Now_ms();
  LOG_DEBUG("reference_path_manager update cost:%f\n", time_end - time_start);
  JSON_DEBUG_VALUE("reference_path_manager_update_cost", time_end - time_start);

  if (not session_->is_hpp_scene()) {
    time_start = IflyTime::Now_ms();
    lateral_obstacle_ptr_->update();
    time_end = IflyTime::Now_ms();
    LOG_DEBUG("lateral_obstacle update cost:%f\n", time_end - time_start);
    JSON_DEBUG_VALUE("lateral_obstacle_update_cost", time_end - time_start);

    lane_tracks_mgr_ptr_->update_lane_tracks();
    // Step 7) update agent node manager
    time_start = IflyTime::Now_ms();
    agent_node_mgr_ptr_->init();
    time_end = IflyTime::Now_ms();
    LOG_DEBUG("agent_node_manager init cost:%f\n", time_end - time_start);
    JSON_DEBUG_VALUE("agent_node_manager_init_cost", time_end - time_start);
    // std::cout<< "agent_node_mgr time is : " << time_end - time_start
    // <<std::endl;
  } else {
    time_start = IflyTime::Now_ms();
    history_obstacle_ptr_->Update();
    time_end = IflyTime::Now_ms();
    LOG_DEBUG("history_obstacle update cost:%f\n", time_end - time_start);
    JSON_DEBUG_VALUE("history_obstacle_cost", time_end - time_start)
  }

  // DynamicWorld验证
  time_start = IflyTime::Now_ms();
  dynamic_world_->ConstructDynamicWorld();
  time_end = IflyTime::Now_ms();
  LOG_DEBUG("dynamic world update cost:%f\n", time_end - time_start);
  JSON_DEBUG_VALUE("dynamic_world_cost", time_end - time_start)

  auto end_time = IflyTime::Now_ms();
  LOG_DEBUG("EnvironmentalModelManager::Run cost time:%f\n",
            end_time - current_time);
  JSON_DEBUG_VALUE("EnvironmentalModelManagerCost", end_time - current_time);
  std::string status_msg;
  if (!InputReady(current_time, status_msg)) {
    LOG_ERROR("InputReady is failed !!!! \n");
    // return false;
  }

  return true;
}

bool EnvironmentalModelManager::ego_state_update(double current_time,
                                                 const LocalView &local_view) {
  common::VehicleStatus vehicle_status;
  vehicle_status_adaptor(current_time, local_view, vehicle_status);
  return ego_state_manager_ptr_->update(vehicle_status);
}

bool EnvironmentalModelManager::obstacle_prediction_update(
    double current_time, const LocalView &local_view) {
  // fusion and prediction update
  auto &prediction_info =
      session_->mutable_environmental_model()->get_mutable_prediction_info();
  auto &fusion_objs_info =
      session_->mutable_environmental_model()->get_mutable_fusion_info();
  prediction_info.clear();
  fusion_objs_info.clear();
  if (session_->environmental_model().location_valid()) {
    std::unordered_set<uint> prediction_obj_id_set;
    truncate_prediction_info(local_view.prediction_result,
                             local_view.localization_estimate.header.timestamp,
                             prediction_obj_id_set);
    for (int i = 0; i < local_view.fusion_objects_info.fusion_object_num; i++) {
      const auto &obj = local_view.fusion_objects_info.fusion_object[i];
      if (prediction_obj_id_set.find(obj.additional_info.track_id) ==
          prediction_obj_id_set.end()) {
        transform_fusion_to_prediction_longtime(
            obj, (double)local_view.fusion_objects_info.header.timestamp,
            prediction_info);
      }
      transform_fusion_to_prediction_longtime(
          obj, (double)local_view.fusion_objects_info.header.timestamp,
          fusion_objs_info);
    }
  } else {
    for (int i = 0; i < local_view.fusion_objects_info.fusion_object_num; i++) {
      const auto &obj = local_view.fusion_objects_info.fusion_object[i];
      transform_fusion_to_prediction(
          obj, (double)local_view.fusion_objects_info.header.timestamp,
          prediction_info);
    }
  }

  last_feed_time_[FEED_FUSION_INFO] = local_view.fusion_objects_info_recv_time;
  last_feed_time_[FEED_PREDICTION_INFO] =
      local_view.prediction_result_recv_time;
  return true;
}

bool EnvironmentalModelManager::ground_line_obstacles_update(
    const LocalView &local_view) {
  std::vector<GroundLinePoint> &ground_line_point_info =
      session_->mutable_environmental_model()
          ->get_mutable_ground_line_point_info();
  ground_line_point_info.clear();
  iflyauto::GroundLinePerceptionInfo groundline_data =
      local_view.ground_line_perception;

  for (size_t i = 0; i < groundline_data.ground_lines_size; ++i) {
    for (size_t j = 0; j < groundline_data.ground_lines[i].points_3d_size;
         j++) {
      GroundLinePoint point;
      point.point =
          planning_math::Vec2d(groundline_data.ground_lines[i].points_3d[j].x,
                               groundline_data.ground_lines[i].points_3d[j].y);
      point.status = GroundLinePoint::Status::UNCLASSIFIED;
      ground_line_point_info.emplace_back(point);
    }
  }

  return true;
}

void EnvironmentalModelManager::vehicle_status_adaptor(
    double current_time, const LocalView &local_view,
    common::VehicleStatus &vehicle_status) {
  const auto &vehicle_service_output_info =
      local_view.vehicle_service_output_info;
  const auto &localization_estimate = local_view.localization_estimate;
  const auto &localization = local_view.localization;
  // const auto &hmi_mcu_inner_info = local_view.hmi_mcu_inner_info;
  // const auto &hmi_inner_info = local_view.hmi_inner_info;
  const auto &function_state_machine_info = local_view.function_state_machine_info;
  vehicle_status.mutable_header()->set_timestamp_us(
      vehicle_service_output_info.header.timestamp);

  if (session_->environmental_model().location_valid()) {
    vehicle_status.mutable_heading_yaw()
        ->mutable_heading_yaw_data()
        ->set_value_rad(localization.orientation.euler_boot.yaw);
    // ->set_value_rad(localization_estimate.pose.euler_angles.yaw);
    vehicle_status.mutable_location()->set_available(true);
    // auto llh_position = localization_estimate.pose.llh_position;
    auto llh_position = localization.position.position_llh;
    vehicle_status.mutable_location()
        ->mutable_location_geographic()
        ->set_latitude_degree(llh_position.latitude);
    vehicle_status.mutable_location()
        ->mutable_location_geographic()
        ->set_longitude_degree(llh_position.longitude);
    vehicle_status.mutable_location()
        ->mutable_location_geographic()
        ->set_altitude_meter(llh_position.height);
    // auto local_position = localization_estimate.pose.local_position;
    auto local_position = localization.position.position_boot;
    vehicle_status.mutable_location()->mutable_location_enu()->set_x(
        local_position.x);
    vehicle_status.mutable_location()->mutable_location_enu()->set_y(
        local_position.y);
    vehicle_status.mutable_location()->mutable_location_enu()->set_z(
        local_position.z);
    vehicle_status.mutable_location()->mutable_location_enu()->set_timestamp_us(
        localization.header.timestamp);
    // auto enu_orientation = localization_estimate.pose.orientation;
    auto enu_orientation = localization.orientation.quaternion_boot;
    vehicle_status.mutable_location()
        ->mutable_location_enu()
        ->mutable_orientation()
        ->set_x(enu_orientation.x);
    vehicle_status.mutable_location()
        ->mutable_location_enu()
        ->mutable_orientation()
        ->set_y(enu_orientation.y);
    vehicle_status.mutable_location()
        ->mutable_location_enu()
        ->mutable_orientation()
        ->set_z(enu_orientation.z);
    vehicle_status.mutable_location()
        ->mutable_location_enu()
        ->mutable_orientation()
        ->set_w(enu_orientation.w);
    // last_feed_time_[FEED_EGO_ENU] =
    // local_view.localization_estimate_recv_time;
    last_feed_time_[FEED_EGO_ENU] = local_view.localization_recv_time;
  } else {
    vehicle_status.mutable_heading_yaw()
        ->mutable_heading_yaw_data()
        ->set_value_rad(0.);
    vehicle_status.mutable_location()->set_available(true);
    auto location_enu =
        vehicle_status.mutable_location()->mutable_location_enu();
    // Todo
    // location_enu->set_timestamp_us(static_cast<uint64_t>(current_time *
    // 1e6));
    location_enu->set_x(0.0);
    location_enu->set_y(0.0);
    location_enu->set_z(0.0);
    location_enu->mutable_orientation()->set_x(0.0);
    location_enu->mutable_orientation()->set_y(0.0);
    location_enu->mutable_orientation()->set_z(0.0);
    location_enu->mutable_orientation()->set_w(1.0);
    // last_feed_time_[FEED_EGO_ENU] =
    // local_view.localization_estimate_recv_time;
    last_feed_time_[FEED_EGO_ENU] = local_view.localization_recv_time;
  }

  vehicle_status.mutable_velocity()->mutable_cruise_velocity()->set_value_mps(
      function_state_machine_info.pilot_req.acc_curise_real_spd);

  if (vehicle_service_output_info.yaw_rate_available) {
    vehicle_status.mutable_angular_velocity()->set_available(true);
    vehicle_status.mutable_angular_velocity()
        ->mutable_heading_yaw_rate()
        ->set_value_rps(vehicle_service_output_info.yaw_rate);
  } else {
    vehicle_status.mutable_angular_velocity()->set_available(false);
    vehicle_status.mutable_angular_velocity()
        ->mutable_heading_yaw_rate()
        ->set_value_rps(0.);
  }

  // if (vehicle_service_output_info.vehicle_speed_available) {
  //   vehicle_status.mutable_velocity()->set_available(true);
  //   vehicle_status.mutable_velocity()
  //       ->mutable_heading_velocity()
  //       ->set_value_mps(vehicle_service_output_info.vehicle_speed);
  //   last_feed_time_[FEED_EGO_VEL] =
  //       local_view.vehicle_service_output_info_recv_time;
  // }

  // use new localization linear speed
  if (session_->environmental_model().location_valid()) {
    vehicle_status.mutable_velocity()->set_available(true);
    vehicle_status.mutable_velocity()
        ->mutable_heading_velocity()
        ->set_value_mps(localization.velocity.velocity_body.vx);
    last_feed_time_[FEED_EGO_VEL] = local_view.localization_recv_time;
  }

  // if (vehicle_service_output_info.long_acceleration_available) {
  //   vehicle_status.mutable_brake_info()
  //       ->mutable_brake_info_data()
  //       ->set_acceleration_on_vehicle_wheel(
  //           vehicle_service_output_info.long_acceleration);
  //   last_feed_time_[FEED_EGO_ACC] =
  //       local_view.vehicle_service_output_info_recv_time;
  // }

  // use new localization linear acc
  if (session_->environmental_model().location_valid()) {
    vehicle_status.mutable_brake_info()
        ->mutable_brake_info_data()
        ->set_acceleration_on_vehicle_wheel(
            localization.acceleration.acceleration_body.ax);
    last_feed_time_[FEED_EGO_ACC] = local_view.localization_recv_time;
  }

  if (vehicle_service_output_info.vehicle_speed_display_available) {
    vehicle_status.mutable_velocity()->set_available(true);
    vehicle_status.mutable_velocity()->set_hmi_speed(
        vehicle_service_output_info.vehicle_speed_display);
  }

  if (session_->environmental_model().location_valid()) {
    auto linear_velocity_from_wheel =
        localization_estimate.pose.linear_velocity_from_wheel;
    // WB：定位没有给linear_velocity_from_wheel赋值
    // vehicle_status.mutable_velocity()
    //     ->mutable_heading_velocity()
    //     ->set_value_mps(linear_velocity_from_wheel);
  }

  if (vehicle_service_output_info.steering_wheel_angle_available) {
    auto steering_data = vehicle_status.mutable_steering_wheel();
    steering_data->set_available(true);
    steering_data->mutable_steering_wheel_data()->set_steering_wheel_rad(
        vehicle_service_output_info.steering_wheel_angle);
    steering_data->mutable_steering_wheel_data()->set_steering_wheel_torque(
        vehicle_service_output_info.power_train_current_torque);

    last_feed_time_[FEED_EGO_STEER_ANGLE] =
        local_view.vehicle_service_output_info_recv_time;
  }

  if (vehicle_service_output_info.fl_wheel_speed_available &&
      vehicle_service_output_info.fr_wheel_speed_available &&
      vehicle_service_output_info.rl_wheel_speed_available &&
      vehicle_service_output_info.rr_wheel_speed_available) {
    vehicle_status.mutable_wheel_velocity()->set_available(true);  // hack
    auto wheel_velocity4d =
        vehicle_status.mutable_wheel_velocity()->mutable_wheel_velocity4d();
    wheel_velocity4d->set_front_left(
        vehicle_service_output_info.fl_wheel_speed);
    wheel_velocity4d->set_front_right(
        vehicle_service_output_info.fr_wheel_speed);
    wheel_velocity4d->set_rear_left(vehicle_service_output_info.rl_wheel_speed);
    wheel_velocity4d->set_rear_right(
        vehicle_service_output_info.rr_wheel_speed);
    last_feed_time_[FEED_WHEEL_SPEED_REPORT] =
        local_view.vehicle_service_output_info_recv_time;
  }

  if (vehicle_service_output_info.power_train_override_flag_available) {
    vehicle_status.mutable_throttle()->mutable_throttle_data()->set_override(
        vehicle_service_output_info.power_train_override_flag);
  }

  auto vehicle_light = vehicle_status.mutable_vehicle_light();
  // 转向拨杆状态
  if (vehicle_service_output_info.turn_switch_state_available) {
    // 紧急灯
    if (vehicle_service_output_info.turn_switch_state == NONE &&
        vehicle_service_output_info.hazard_light_state) {
      if (vehicle_service_output_info.hazard_light_state_available) {
        vehicle_light->mutable_vehicle_light_data()
            ->mutable_turn_signal()
            ->set_value(common::TurnSignalType::EMERGENCY_FLASHER);
      }
    } else {
      history_lc_source_[0] = history_lc_source_[1];  // 上上帧
      history_lc_source_[1] = session_->planning_context()
                                  .lane_change_decider_output()
                                  .lc_request_source;  // 上一帧
      // 根据传入的拨杆信号计算转向
      RunBlinkState(vehicle_service_output_info);
      vehicle_light->mutable_vehicle_light_data()
          ->mutable_turn_signal()
          ->set_value(current_turn_signal_);
    }
    last_feed_time_[FEED_MISC_REPORT] =
        local_view.vehicle_service_output_info_recv_time;
  }
  last_frame_turn_sinagl_ = current_turn_signal_;
  JSON_DEBUG_VALUE("turn_switch_state",
                   vehicle_service_output_info.turn_switch_state)

  if (vehicle_service_output_info.auto_light_state_available) {
    vehicle_light->mutable_vehicle_light_data()->set_auto_light_state(
        vehicle_service_output_info.auto_light_state);
    last_feed_time_[FEED_MISC_REPORT] =
        local_view.vehicle_service_output_info_recv_time;
  }

  if (vehicle_service_output_info.driver_hand_torque_available) {
    vehicle_status.mutable_driver_hand_state()->set_driver_hand_torque(
        vehicle_service_output_info.driver_hand_torque);
  }

  if (vehicle_service_output_info.driver_hands_off_state_available) {
    vehicle_status.mutable_driver_hand_state()->set_driver_hands_off_state(
        vehicle_service_output_info.driver_hands_off_state);
  }

  if (vehicle_service_output_info.gear_lever_state_available) {
    vehicle_status.mutable_gear()
        ->mutable_gear_data()
        ->mutable_gear_status()
        ->set_value(vehicle_service_output_info.gear_lever_state);
  }
}

void EnvironmentalModelManager::truncate_prediction_info(
    const iflyauto::PredictionResult &prediction_result,
    double cur_timestamp_us, std::unordered_set<uint> &prediction_obj_id_set) {
  assert(session_ != nullptr);
  double current_time =
      session_->planning_context().planning_result().timestamp;
  auto init_relative_time = session_->environmental_model()
                                .get_ego_state_manager()
                                ->planning_init_point()
                                .relative_time;
  auto &prediction_info =
      session_->mutable_environmental_model()->get_mutable_prediction_info();
  prediction_info.clear();
  auto &ego_state = session_->environmental_model().get_ego_state_manager();

  for (int i = 0; i < prediction_result.prediction_obstacle_list_size; i++) {
    const auto &prediction_object =
        prediction_result.prediction_obstacle_list[i];
    auto fusion_source =
        prediction_object.fusion_obstacle.additional_info.fusion_source;
    if (!(fusion_source & OBSTACLE_SOURCE_CAMERA)) {
      // 非相机融合成功的不用
      continue;
    }
    PredictionObject cur_predicion_obj;
    cur_predicion_obj.id =
        prediction_object.fusion_obstacle.additional_info.track_id;
    prediction_obj_id_set.emplace(cur_predicion_obj.id);
    cur_predicion_obj.type =
        (iflyauto::ObjectType)
            prediction_object.fusion_obstacle.common_info.type;
    cur_predicion_obj.fusion_source =
        prediction_object.fusion_obstacle.additional_info.fusion_source;
    // todo:后面接口变化时，取trajectory的时间戳
    cur_predicion_obj.timestamp_us = prediction_result.header.timestamp;
    double prediction_relative_time =
        clip(prediction_result.header.timestamp / 1.e+6 - current_time / 1.e+3 -
                 init_relative_time,
             0.0, -1.0);
    if (std::abs(prediction_relative_time) > 0.3) {
      LOG_DEBUG(
          "[prediction delay time] obstacle[%d] absolute start time %lu "
          "relative time %f start time %f "
          "init_relative_time %f \n",
          cur_predicion_obj.id, prediction_result.header.timestamp,
          prediction_result.header.timestamp / 1.e+6 - current_time / 1.e+3,
          prediction_relative_time, init_relative_time);
    }
    cur_predicion_obj.delay_time = prediction_relative_time;
    JSON_DEBUG_VALUE("prediction_relative_time", prediction_relative_time);
#ifdef X86
    cur_predicion_obj.delay_time =
        SimulationContext::Instance()->prediction_relative_time();
#endif
    if (prediction_object.obstacle_intent.type ==
        iflyauto::OBSTACLE_INTENT_COMMON) {
      cur_predicion_obj.intention = ObstacleIntentType::COMMON;
    } else if (prediction_object.obstacle_intent.type ==
               iflyauto::OBSTACLE_INTENT_CUT_IN) {
      cur_predicion_obj.intention = ObstacleIntentType::CUT_IN;
    }
    // cur_predicion_obj.b_backup_freemove =
    // prediction_object.b_backup_freemove(); todo: clren
    // cur_predicion_obj.cutin_score = prediction_object.cutin_score();  todo:
    // clren
    cur_predicion_obj.position_x =
        prediction_object.fusion_obstacle.common_info.center_position.x;
    cur_predicion_obj.position_y =
        prediction_object.fusion_obstacle.common_info.center_position.y;
    cur_predicion_obj.relative_position_x =
        prediction_object.fusion_obstacle.common_info.relative_center_position
            .x;
    cur_predicion_obj.relative_position_y =
        prediction_object.fusion_obstacle.common_info.relative_center_position
            .y;
    cur_predicion_obj.relative_speed_x =
        prediction_object.fusion_obstacle.common_info.relative_velocity.x;
    cur_predicion_obj.relative_speed_y =
        prediction_object.fusion_obstacle.common_info.relative_velocity.y;
    cur_predicion_obj.acceleration_relative_to_ground_x =
        prediction_object.fusion_obstacle.common_info.acceleration.x;
    cur_predicion_obj.acceleration_relative_to_ground_y =
        prediction_object.fusion_obstacle.common_info.acceleration.y;
    cur_predicion_obj.relative_acceleration_x =
        prediction_object.fusion_obstacle.common_info.relative_acceleration.x;
    cur_predicion_obj.relative_acceleration_y =
        prediction_object.fusion_obstacle.common_info.relative_acceleration.y;
    cur_predicion_obj.length =
        prediction_object.fusion_obstacle.common_info.shape.length;
    cur_predicion_obj.width =
        prediction_object.fusion_obstacle.common_info.shape.width;
    cur_predicion_obj.speed =
        std::hypot(prediction_object.fusion_obstacle.common_info.velocity.x,
                   prediction_object.fusion_obstacle.common_info.velocity.y);
    cur_predicion_obj.yaw =
        prediction_object.fusion_obstacle.common_info.heading_angle;
    if ((int)cur_predicion_obj.yaw == 255) {
      // set object's yaw with ego heading
      cur_predicion_obj.yaw = ego_state->heading_angle();
    }

    cur_predicion_obj.theta =
        std::atan2(prediction_object.fusion_obstacle.common_info.velocity.y,
                   prediction_object.fusion_obstacle.common_info.velocity.x);
    cur_predicion_obj.acc = std::hypot(
        prediction_object.fusion_obstacle.common_info.acceleration.x,
        prediction_object.fusion_obstacle.common_info.acceleration.y);
    // cur_predicion_obj.bottom_polygon_points =
    // prediction_object.bottom_polygon_points();
    // cur_predicion_obj.top_polygon_points =
    // prediction_object.top_polygon_points();

    const auto &prediction_traj = prediction_object.trajectory;
    PredictionTrajectory cur_prediction_trajectory;
    size_t traj_index = 0;
    double step_time = prediction_traj.relative_time;
    std::vector<PredictionTrajectoryPoint> trajectory_points;
    // Attention:PREDICTION_TRAJ_POINT_NUM whether valid in C struct
    // size of prediction_traj.trajectory_point maybe not equal to
    // PREDICTION_TRAJ_POINT_NUM
    for (int i = 0; i < PREDICTION_TRAJ_POINT_NUM; i++) {
      const auto &point = prediction_traj.trajectory_point[i];
      PredictionTrajectoryPoint trajectory_point;
      double point_relative_time =
          cur_predicion_obj.delay_time + step_time * traj_index;
      trajectory_point.relative_time = point_relative_time;
      trajectory_point.x = point.position.x;
      trajectory_point.y = point.position.y;
      trajectory_point.yaw = point.yaw;
      trajectory_point.speed = point.velocity;
      trajectory_point.theta = point.theta_vel;
      trajectory_point.prob = prediction_traj.confidence;
      trajectory_point.std_dev_x = point.gaussian_info.sigma_x;
      trajectory_point.std_dev_y = point.gaussian_info.sigma_y;
      // trajectory_point.std_dev_yaw = point.std_dev_yaw();
      // trajectory_point.std_dev_speed = point.std_dev_speed();
      trajectory_point.relative_ego_x = point.relative_position.x;
      trajectory_point.relative_ego_y = point.relative_position.y;
      trajectory_point.relative_ego_yaw = point.relative_yaw;
      trajectory_point.relative_ego_speed =
          std::hypot(point.relative_velocity.x, point.relative_velocity.y);

      // trajectory_point.relative_ego_std_dev_x =
      // point.relative_ego_std_dev_x();
      // trajectory_point.relative_ego_std_dev_y =
      // point.relative_ego_std_dev_y();
      // trajectory_point.relative_ego_std_dev_yaw =
      // point.relative_ego_std_dev_yaw();
      // trajectory_point.relative_ego_std_dev_speed =
      // point.relative_ego_std_dev_speed();
      traj_index++;
      trajectory_points.emplace_back(trajectory_point);
    }

    for (traj_index = 0; traj_index < 41; traj_index++) {
      auto trajectory_point =
          GetPointAtTime(trajectory_points, 0.2 * traj_index);
      cur_prediction_trajectory.trajectory.emplace_back(trajectory_point);
    }

    // cur_prediction_trajectory.prob = prediction_traj.prob;
    // cur_prediction_trajectory.intention =
    // prediction_traj.obstacle_intent().type();
    // cur_prediction_trajectory.source = prediction_traj.source;
    // cur_prediction_trajectory.b_valid_sigma =
    // prediction_traj.b_valid_sigma;
    // cur_prediction_trajectory.prediction_interval =
    // prediction_traj.prediction_interval;
    // cur_prediction_trajectory.num_of_points =
    // cur_prediction_trajectory.trajectory.size();
    // cur_prediction_trajectory.const_vel_prob =
    // prediction_traj.const_vel_prob;
    // cur_prediction_trajectory.const_acc_prob =
    // prediction_traj.const_acc_prob; cur_prediction_trajectory.still_prob =
    // prediction_traj.still_prob; cur_prediction_trajectory.coord_turn_prob =
    // prediction_traj.coord_turn_prob;
    // cur_prediction_trajectory.b_minor_modal =
    // prediction_traj.b_minor_modal;
    cur_predicion_obj.trajectory_array.emplace_back(cur_prediction_trajectory);
    prediction_info.emplace_back(cur_predicion_obj);
  }
}

PredictionTrajectoryPoint EnvironmentalModelManager::GetPointAtTime(
    const std::vector<PredictionTrajectoryPoint> &trajectory_points,
    const double relative_time) const {
  assert(trajectory_points.size() != 0);

  if (trajectory_points.size() < 2) {
    return trajectory_points.at(0);
  } else {
    auto comp = [](const PredictionTrajectoryPoint p, const double time) {
      return p.relative_time < time;
    };

    auto it_lower =
        std::lower_bound(trajectory_points.begin(), trajectory_points.end(),
                         relative_time, comp);

    if (it_lower == trajectory_points.begin()) {
      return *trajectory_points.begin();
    } else if (it_lower == trajectory_points.end()) {
      return *trajectory_points.rbegin();
    }
    return planning_math::InterpolateUsingLinearApproximation(
        *(it_lower - 1), *it_lower, relative_time);
  }
}

bool EnvironmentalModelManager::transform_fusion_to_prediction(
    const iflyauto::FusionObject &fusion_object, double timestamp,
    std::vector<PredictionObject> &objects_infos) {
  assert(session_ != nullptr);
  if (session_ == nullptr) {
    return false;
  }
  bool object_is_slow = false;
  auto &ego_state = session_->environmental_model().get_ego_state_manager();

  PredictionObject prediction_object;
  prediction_object.id = fusion_object.additional_info.track_id;
  prediction_object.type = fusion_object.common_info.type;
  prediction_object.fusion_source = fusion_object.additional_info.fusion_source;
  prediction_object.timestamp_us = timestamp;

  prediction_object.delay_time = 0.0;
  prediction_object.cutin_score = 0;
  prediction_object.position_x = fusion_object.common_info.center_position.x;
  prediction_object.position_y = fusion_object.common_info.center_position.y;
  prediction_object.length = fusion_object.common_info.shape.length;
  prediction_object.width = fusion_object.common_info.shape.width;
  prediction_object.speed = std::hypot(fusion_object.common_info.velocity.x,
                                       fusion_object.common_info.velocity.y);
  prediction_object.motion_pattern_current =
      fusion_object.additional_info.motion_pattern_current;
  prediction_object.is_oversize_vehicle =
      CheckIfOversizeVehicle(prediction_object.type);
  prediction_object.is_VRU = CheckIfVru(prediction_object.type);
  prediction_object.is_traffic_facilities =
      CheckIfTrafficFacilities(prediction_object.type);
  prediction_object.is_car = CheckIfCar(prediction_object.type);

  prediction_object.yaw = fusion_object.common_info.heading_angle;
  if ((int)prediction_object.yaw == 255) {
    prediction_object.yaw = ego_state->heading_angle();
  } else {
    prediction_object.yaw = fusion_object.common_info.heading_angle;
  }
  prediction_object.acc = std::hypot(fusion_object.common_info.acceleration.x,
                                     fusion_object.common_info.acceleration.y);
  // judge direction of obj acc
  auto obj_acc_heading_angle = atan2(fusion_object.common_info.acceleration.y,
                                     fusion_object.common_info.acceleration.x);
  Eigen::Vector2f obj_acc_heading_vec(cos(obj_acc_heading_angle),
                                      sin(obj_acc_heading_angle));
  Eigen::Vector2f obj_heading_vec(cos(prediction_object.yaw),
                                  sin(prediction_object.yaw));
  if (obj_acc_heading_vec.dot(obj_heading_vec) < 0.0) {
    prediction_object.acc *= -1.0;
  }
  // add relative info for highway
  prediction_object.relative_position_x =
      fusion_object.common_info.relative_center_position.x;
  prediction_object.relative_position_y =
      fusion_object.common_info.relative_center_position.y;
  prediction_object.relative_speed_x =
      fusion_object.common_info.relative_velocity.x;
  prediction_object.relative_speed_y =
      fusion_object.common_info.relative_velocity.y;
  prediction_object.relative_acceleration_x =
      fusion_object.common_info.relative_acceleration.x;
  prediction_object.relative_acceleration_y =
      fusion_object.common_info.relative_acceleration.y;
  prediction_object.acceleration_relative_to_ground_x =
      fusion_object.common_info.acceleration.x;
  prediction_object.acceleration_relative_to_ground_y =
      fusion_object.common_info.acceleration.y;

  double relative_v_x = fusion_object.common_info.relative_velocity.x;
  double relative_v_y = fusion_object.common_info.relative_velocity.y;

  // The agent is slow when it's speed < 10km/h (person's speed)
  object_is_slow = prediction_object.speed < 2.78 ? true : false;
  // For no prediction schemes, use heading angle when obstacles are slow
  if (object_is_slow) {
    prediction_object.relative_theta =
        fusion_object.common_info.relative_heading_angle;
    prediction_object.theta = fusion_object.common_info.heading_angle;
  } else {
    prediction_object.theta = std::atan2(fusion_object.common_info.velocity.y,
                                         fusion_object.common_info.velocity.x);
    prediction_object.relative_theta = std::atan2(
        relative_v_y, relative_v_x + ego_state_manager_ptr_->ego_v());
  }

  if ((int)prediction_object.relative_theta == 255) {
    prediction_object.relative_theta = 0;
  }

  PredictionTrajectoryPoint trajectory_point;
  trajectory_point.relative_time = 0;
  trajectory_point.x = prediction_object.position_x;
  trajectory_point.y = prediction_object.position_y;
  trajectory_point.yaw = prediction_object.yaw;
  trajectory_point.speed = prediction_object.speed;

  trajectory_point.theta = prediction_object.theta;
  trajectory_point.prob = 1;
  trajectory_point.relative_ego_x = prediction_object.relative_position_x;
  trajectory_point.relative_ego_y = prediction_object.relative_position_y;
  trajectory_point.relative_ego_yaw = prediction_object.relative_theta;
  trajectory_point.relative_ego_speed = std::hypot(
      prediction_object.relative_speed_x, prediction_object.relative_speed_y);
  PredictionTrajectory tra;
  tra.trajectory.emplace_back(std::move(trajectory_point));
  prediction_object.trajectory_array.emplace_back(std::move(tra));
  prediction_object.is_static = IsStatic(prediction_object);
  objects_infos.emplace_back(std::move(prediction_object));
  return true;
}

static inline float32 x_turn(float32 inputx, float32 inputy, float32 theta) {
  return inputx * cos(theta) - inputy * sin(theta);
}

static inline float32 y_turn(float32 inputx, float32 inputy, float32 theta) {
  return inputx * sin(theta) + inputy * cos(theta);
}

bool EnvironmentalModelManager::transform_fusion_to_prediction_longtime(
    const iflyauto::FusionObject &fusion_object, double timestamp,
    std::vector<PredictionObject> &objects_infos) {
  assert(session_ != nullptr);
  if (session_ == nullptr) {
    return false;
  }
  bool object_is_slow = false;
  auto &ego_state = session_->environmental_model().get_ego_state_manager();

  PredictionObject prediction_object;
  prediction_object.id = fusion_object.additional_info.track_id;
  prediction_object.type = fusion_object.common_info.type;
  prediction_object.fusion_source = fusion_object.additional_info.fusion_source;
  prediction_object.timestamp_us = timestamp;

  prediction_object.delay_time = 0.0;
  prediction_object.cutin_score = 0;
  prediction_object.position_x = fusion_object.common_info.center_position.x;
  prediction_object.position_y = fusion_object.common_info.center_position.y;
  prediction_object.length = fusion_object.common_info.shape.length;
  prediction_object.width = fusion_object.common_info.shape.width;
  prediction_object.speed = std::hypot(fusion_object.common_info.velocity.x,
                                       fusion_object.common_info.velocity.y);
  prediction_object.motion_pattern_current =
      fusion_object.additional_info.motion_pattern_current;
  prediction_object.is_oversize_vehicle =
      CheckIfOversizeVehicle(prediction_object.type);
  prediction_object.is_VRU = CheckIfVru(prediction_object.type);
  prediction_object.is_traffic_facilities =
      CheckIfTrafficFacilities(prediction_object.type);
  prediction_object.is_car = CheckIfCar(prediction_object.type);

  prediction_object.yaw = fusion_object.common_info.heading_angle;
  if ((int)prediction_object.yaw == 255) {
    prediction_object.yaw = ego_state->heading_angle();
  } else {
    prediction_object.yaw = fusion_object.common_info.heading_angle;
  }
  prediction_object.acc = std::hypot(fusion_object.common_info.acceleration.x,
                                     fusion_object.common_info.acceleration.y);
  // judge direction of obj acc
  auto obj_acc_heading_angle = atan2(fusion_object.common_info.acceleration.y,
                                     fusion_object.common_info.acceleration.x);
  Eigen::Vector2f obj_acc_heading_vec(cos(obj_acc_heading_angle),
                                      sin(obj_acc_heading_angle));
  Eigen::Vector2f obj_heading_vec(cos(prediction_object.yaw),
                                  sin(prediction_object.yaw));
  if (obj_acc_heading_vec.dot(obj_heading_vec) < 0.0) {
    prediction_object.acc *= -1.0;
  }
  // add relative info for highway
  prediction_object.relative_position_x =
      x_turn(prediction_object.position_x - ego_state->ego_pose().x,
             prediction_object.position_y - ego_state->ego_pose().y,
             -1 * ego_state->heading_angle());
  prediction_object.relative_position_y =
      y_turn(prediction_object.position_x - ego_state->ego_pose().x,
             prediction_object.position_y - ego_state->ego_pose().y,
             -1 * ego_state->heading_angle());

  auto relative_velocity_world_x =
      fusion_object.common_info.velocity.x -
      ego_state->ego_v() * cos(ego_state->heading_angle());
  auto relative_velocity_world_y =
      fusion_object.common_info.velocity.y -
      ego_state->ego_v() * sin(ego_state->heading_angle());
  prediction_object.relative_speed_x =
      x_turn(relative_velocity_world_x, relative_velocity_world_y,
             -1 * ego_state->heading_angle());
  prediction_object.relative_speed_y =
      y_turn(relative_velocity_world_x, relative_velocity_world_y,
             -1 * ego_state->heading_angle());

  auto relative_acceleration_world_x =
      fusion_object.common_info.acceleration.x -
      ego_state->ego_acc() * cos(ego_state->heading_angle());
  auto relative_acceleration_world_y =
      fusion_object.common_info.acceleration.y -
      ego_state->ego_acc() * sin(ego_state->heading_angle());
  prediction_object.relative_acceleration_x =
      x_turn(relative_acceleration_world_x, relative_acceleration_world_y,
             -1 * ego_state->heading_angle());
  prediction_object.relative_acceleration_y =
      y_turn(relative_acceleration_world_x, relative_acceleration_world_y,
             -1 * ego_state->heading_angle());

  prediction_object.acceleration_relative_to_ground_x =
      fusion_object.common_info.acceleration.x;
  prediction_object.acceleration_relative_to_ground_y =
      fusion_object.common_info.acceleration.y;

  // The agent is slow when it's speed < 10km/h (person's speed)
  object_is_slow = prediction_object.speed < 2.78 ? true : false;
  // For no prediction schemes, use heading angle when obstacles are slow
  if (object_is_slow) {
    prediction_object.relative_theta =
        fusion_object.common_info.heading_angle - ego_state->heading_angle();
    prediction_object.theta = fusion_object.common_info.heading_angle;
  } else {
    prediction_object.theta = std::atan2(fusion_object.common_info.velocity.y,
                                         fusion_object.common_info.velocity.x);
    prediction_object.relative_theta =
        prediction_object.theta - ego_state->heading_angle();
  }
  prediction_object.relative_theta =
      std::fmod(prediction_object.relative_theta, 2 * M_PI);
  if (prediction_object.relative_theta > M_PI) {
    prediction_object.relative_theta -= 2 * M_PI;
  }

  if ((int)prediction_object.relative_theta == 255) {
    prediction_object.relative_theta = 0;
  }

  PredictionTrajectoryPoint trajectory_point;
  trajectory_point.relative_time = 0;
  trajectory_point.x = prediction_object.position_x;
  trajectory_point.y = prediction_object.position_y;
  trajectory_point.yaw = prediction_object.yaw;
  trajectory_point.speed = prediction_object.speed;

  trajectory_point.theta = prediction_object.theta;
  trajectory_point.prob = 1;
  trajectory_point.relative_ego_x = prediction_object.relative_position_x;
  trajectory_point.relative_ego_y = prediction_object.relative_position_y;
  trajectory_point.relative_ego_yaw = prediction_object.relative_theta;
  trajectory_point.relative_ego_speed = std::hypot(
      prediction_object.relative_speed_x, prediction_object.relative_speed_y);
  PredictionTrajectory tra;
  tra.trajectory.emplace_back(std::move(trajectory_point));
  prediction_object.trajectory_array.emplace_back(std::move(tra));
  prediction_object.is_static = IsStatic(prediction_object);
  objects_infos.emplace_back(std::move(prediction_object));
  return true;
}

bool EnvironmentalModelManager::IsStatic(const PredictionObject &prediction_object) {
  auto &ego_state = session_->environmental_model().get_ego_state_manager();
  double prediction_trajectory_length = 10.0;
  double prediction_duration = 0.0;
  if (prediction_object.trajectory_array.size() > 0) {
    const auto &trajectory_array = prediction_object.trajectory_array.at(0);
    if (trajectory_array.trajectory.size() > 0) {
      const auto &start_point = trajectory_array.trajectory.at(0);
      const auto &end_point = trajectory_array.trajectory.at(
          trajectory_array.trajectory.size() - 1);
      prediction_trajectory_length = std::sqrt(
          (start_point.x - end_point.x) * (start_point.x - end_point.x) +
          (start_point.y - end_point.y) * (start_point.y - end_point.y));
      prediction_duration = end_point.relative_time;
    }
  }

  double max_speed_static_obstacle = 0.5;
  const double kMaxStaticPredictionLength =
      max_speed_static_obstacle * prediction_duration;
  std::array<double, 3> xp{10, 20, 30};
  std::array<double, 3> fp{1, 2, 3};
  double static_speed = interp(ego_state->ego_v(), xp, fp);
  bool is_static = prediction_object.speed < static_speed ||
                   prediction_object.trajectory_array.size() == 0 ||
                   prediction_trajectory_length < kMaxStaticPredictionLength ||
                   prediction_object.motion_pattern_current ==
                       iflyauto::OBJECT_MOTION_TYPE_STATIC ||
                   prediction_object.is_traffic_facilities;
  return is_static;
}

bool EnvironmentalModelManager::InputReady(double current_time,
                                           std::string &error_msg) {
  auto to_string = [](FeedType feed_type) -> const char * {
    switch (feed_type) {
      case FEED_VEHICLE_DBW_STATUS:
        return "vehicle_dbw_status";
      case FEED_EGO_VEL:
        return "ego_pose_and_vel";
      case FEED_EGO_STEER_ANGLE:
        return "ego_steer_angle";
      case FEED_EGO_ENU:
        return "ego_enu";
      case FEED_WHEEL_SPEED_REPORT:
        return "wheel_speed_report";
      case FEED_EGO_ACC:
        return "ego_acc";
      case FEED_MISC_REPORT:
        return "misc_report";
      case FEED_MAP_INFO:
        return "map_info";
      case FEED_FUSION_INFO:
        return "fusion_info";
      case FEED_PREDICTION_INFO:
        return "prediction_info";
      case FEED_FUSION_LANES_INFO:
        return "fusion_lanes_info";
      default:
        return "unknown type";
    };
  };

  static const std::vector<double> kCheckTimeDiff = {20, 20, 20, 20, 20, 20,
                                                     20, 50, -1, -1, -1};
  static const double kMapCheckTimeDiff = 1000;
  static const double kpredictionCheckTimeDiff = 50;
  static const double kfusionlaneCheckTimeDiff = 50;
  // set default value
  setFaultcode(666);
  static const std::vector<FeedType> input_longtime_with_hdmap{
      FEED_VEHICLE_DBW_STATUS, FEED_EGO_VEL,
      FEED_EGO_STEER_ANGLE,    FEED_EGO_ENU,
      FEED_WHEEL_SPEED_REPORT, FEED_EGO_ACC,
      FEED_MISC_REPORT,        FEED_FUSION_INFO,
      FEED_PREDICTION_INFO,    FEED_MAP_INFO,
  };

  static const std::vector<FeedType> input_longtime_without_hdmap{
      FEED_VEHICLE_DBW_STATUS, FEED_EGO_VEL,
      FEED_EGO_STEER_ANGLE,    FEED_EGO_ENU,
      FEED_WHEEL_SPEED_REPORT, FEED_EGO_ACC,
      FEED_MISC_REPORT,        FEED_FUSION_INFO,
      FEED_PREDICTION_INFO,    FEED_FUSION_LANES_INFO,
  };

  static const std::vector<FeedType> input_realtime_with_hdmap{
      FEED_VEHICLE_DBW_STATUS, FEED_EGO_VEL,
      FEED_EGO_STEER_ANGLE,    FEED_EGO_ENU,
      FEED_WHEEL_SPEED_REPORT, FEED_EGO_ACC,
      FEED_MISC_REPORT,        FEED_FUSION_INFO,
      FEED_MAP_INFO,
  };

  static const std::vector<FeedType> input_realtime_without_hdmap{
      FEED_VEHICLE_DBW_STATUS, FEED_EGO_VEL,
      FEED_EGO_STEER_ANGLE,    FEED_EGO_ENU,
      FEED_WHEEL_SPEED_REPORT, FEED_EGO_ACC,
      FEED_MISC_REPORT,        FEED_FUSION_INFO,
      FEED_FUSION_LANES_INFO,
  };

  error_msg.clear();

  bool res = true;
  const auto &input_list =
      session_->environmental_model().get_hdmap_valid()
          ? (session_->environmental_model().location_valid()
                 ? input_longtime_with_hdmap
                 : input_realtime_with_hdmap)
          : (session_->environmental_model().location_valid()
                 ? input_longtime_without_hdmap
                 : input_realtime_without_hdmap);
  for (int i : input_list) {
    auto feed_type = static_cast<FeedType>(i);
    const char *feed_type_str = to_string(feed_type);
    if (last_feed_time_[i] > 0.0) {
      LOG_DEBUG("(%s)topic latency: %s, %fms%s", __FUNCTION__, feed_type_str,
                current_time - last_feed_time_[i], "\n");
      if (current_time - last_feed_time_[i] > 200) {
        LOG_ERROR("(%s)input_delay: %d, %s", __FUNCTION__, i, feed_type_str,
                  "\n");
        error_msg += std::string(feed_type_str) + "; ";
        setFaultcode(39001);
        res = false;
      } else {
        if ((current_time - last_feed_time_[i] > kCheckTimeDiff[i] * 1.2) ||
            (current_time - last_feed_time_[i] < kCheckTimeDiff[i] * 0.8)) {
          if (feed_type == FEED_MAP_INFO &&
              current_time - last_feed_time_[i] <= kMapCheckTimeDiff * 1.2 &&
              current_time - last_feed_time_[i] >= kMapCheckTimeDiff * 0.8)
            continue;
          if (feed_type == FEED_FUSION_LANES_INFO &&
              current_time - last_feed_time_[i] <=
                  kfusionlaneCheckTimeDiff * 1.2 &&
              current_time - last_feed_time_[i] >=
                  kfusionlaneCheckTimeDiff * 0.8)
            continue;
          if (feed_type == FEED_PREDICTION_INFO &&
              current_time - last_feed_time_[i] <=
                  kpredictionCheckTimeDiff * 1.2 &&
              current_time - last_feed_time_[i] >=
                  kpredictionCheckTimeDiff * 0.8)
            continue;
          LOG_ERROR("(%s)input_rate_error: %d, %s", __FUNCTION__, i,
                    feed_type_str, "\n");
          error_msg += std::string(feed_type_str) + "; ";
          setFaultcode(39000);
          res = false;
        }
      }
    } else {
      // LOG_ERROR("(%s)no feed: %d, %s", __FUNCTION__, i, feed_type_str, "\n");
      LOG_ERROR("lost frame: '%s'\n", feed_type_str);
      error_msg += std::string(feed_type_str) + "; ";
      setFaultcode(39002);
      res = false;
    }
  }
  return res;
}
void EnvironmentalModelManager::RunBlinkState(
    const iflyauto::VehicleServiceOutputInfo &vehicle_service_output_info) {
  const bool active = session_->environmental_model().GetVehicleDbwStatus();
  const auto &state =
      session_->planning_context().lane_change_decider_output().curr_state;
  const auto &lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto lc_request_direction = lane_change_decider_output.lc_request;
  bool is_LC_LCHANGE =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == LEFT_CHANGE);
  bool is_LC_RCHANGE =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == RIGHT_CHANGE);
  switch (vehicle_service_output_info.turn_switch_state) {
    case NONE:
      if (active) {
        // 如果上一帧还是ilc，这一帧不是了，说明ilc状态变了，那么该置0.
        if (history_lc_source_[0] == INT_REQUEST &&
            history_lc_source_[1] != INT_REQUEST) {
          current_turn_signal_ = common::TurnSignalType::NONE;
        }
      } else {
        current_turn_signal_ = common::TurnSignalType::NONE;
      }
      break;
    case LEFT_FIRMLY_TOUCH:
      if (history_lc_source_[0] == INT_REQUEST &&
          history_lc_source_[1] == INT_REQUEST &&
          last_frame_turn_sinagl_ == common::TurnSignalType::RIGHT &&
          is_LC_RCHANGE) {
        // 表示在右变道过程中，向左重拨杆，那么首先归零，ilc_req=0，状态机会跳转至back
        current_turn_signal_ = common::TurnSignalType::NONE;
      } else if (is_LC_RCHANGE) {
        // 由于该信号会连续发50帧，所以来的这一帧有可能还是重拨信号，这时是在change过程中,说明已经过了能取消变道的阈值了，那么依然置0
        current_turn_signal_ = common::TurnSignalType::NONE;
      } else {
        current_turn_signal_ = common::TurnSignalType::LEFT;
      }
      break;
    case RIGHT_FIRMLY_TOUCH:
      // 与上同理
      if (history_lc_source_[0] == INT_REQUEST &&
          history_lc_source_[1] == INT_REQUEST &&
          last_frame_turn_sinagl_ == common::TurnSignalType::LEFT &&
          is_LC_LCHANGE) {
        current_turn_signal_ = common::TurnSignalType::NONE;
      } else if (is_LC_LCHANGE) {
        current_turn_signal_ = common::TurnSignalType::NONE;
      } else {
        current_turn_signal_ = common::TurnSignalType::RIGHT;
      }
      break;
    case LEFT_LIGHTLY_TOUCH:
      // 只有在向右变道的过程中才会起作用
      if (last_frame_turn_sinagl_ == common::TurnSignalType::RIGHT) {
        current_turn_signal_ = common::TurnSignalType::NONE;
      }
      break;
    case RIGHT_LIGHTLY_TOUCH:
      // 只有在向左变道的过程中才会起作用
      if (last_frame_turn_sinagl_ == common::TurnSignalType::LEFT) {
        current_turn_signal_ = common::TurnSignalType::NONE;
      }
      break;
    case ERROR:
      current_turn_signal_ = common::TurnSignalType::NONE;
      break;
  }
}

void EnvironmentalModelManager::setFaultcode(uint64_t faultcode) {
  if (faultcode >= 39000 && faultcode <= 39999) {
    faultcode_ = std::max(faultcode_, faultcode);
  } else {
    faultcode_ = 666;
  }
}

uint64_t EnvironmentalModelManager::getFaultcode() { return faultcode_; }

bool EnvironmentalModelManager::CheckIfOversizeVehicle(const int type) {
  if (type == iflyauto::ObjectType::OBJECT_TYPE_BUS ||
      type == iflyauto::ObjectType::OBJECT_TYPE_TRUCK ||
      type == iflyauto::ObjectType::OBJECT_TYPE_TRAILER) {
    return true;
  } else {
    return false;
  }
}

bool EnvironmentalModelManager::CheckIfVru(const int type) {
  if (type == iflyauto::ObjectType::OBJECT_TYPE_BICYCLE ||
      type == iflyauto::ObjectType::OBJECT_TYPE_MOTORCYCLE ||
      type == iflyauto::ObjectType::OBJECT_TYPE_TRICYCLE ||
      type == iflyauto::ObjectType::OBJECT_TYPE_PEDESTRIAN ||
      type == iflyauto::ObjectType::OBJECT_TYPE_ANIMAL ||
      type == iflyauto::ObjectType::OBJECT_TYPE_CYCLE_RIDING ||
      type == iflyauto::ObjectType::OBJECT_TYPE_MOTORCYCLE_RIDING ||
      type == iflyauto::ObjectType::OBJECT_TYPE_TRICYCLE_RIDING) {
    return true;
  } else {
    return false;
  }
}

bool EnvironmentalModelManager::CheckIfTrafficFacilities(const int type) {
  if (type == iflyauto::ObjectType::OBJECT_TYPE_TRAFFIC_CONE ||
      type == iflyauto::ObjectType::OBJECT_TYPE_TRAFFIC_BARREL ||
      type == iflyauto::ObjectType::OBJECT_TYPE_FENCE ||
      type == iflyauto::ObjectType::OBJECT_TYPE_TRAFFIC_TEM_SIGN ||
      type == iflyauto::ObjectType::OBJECT_TYPE_WATER_SAFETY_BARRIER ||
      type == iflyauto::ObjectType::OBJECT_TYPE_CTASH_BARREL) {
    return true;
  } else {
    return false;
  }
}

// TODO(zkxie): 包含UNKNOWN障碍物,确认感知什么时候会给UNKNOWN障碍物
bool EnvironmentalModelManager::CheckIfCar(const int type) {
  if (type <= iflyauto::ObjectType::OBJECT_TYPE_TRAILER) {
    return true;
  } else {
    return false;
  }
}

}  // namespace planner
}  // namespace planning
