#include "environmental_model_manager.h"

#include <memory>
#include <set>
#include <string>
#include <typeinfo>
#include <unordered_map>

#include "common/math/linear_interpolation.h"
#include "context/ego_planning_config.h"
#include "context/ego_state_manager.h"
#include "context/frenet_ego_state.h"
#include "context/lateral_obstacle.h"
#include "context/obstacle_manager.h"
#include "context/parking_slot_manager.h"
#include "context/planning_output_context.h"
#include "context/reference_path_manager.h"
#include "context/traffic_light_decision_manager.h"
#include "context/virtual_lane_manager.h"
#include "log.h"
#include "prediction.pb.h"
#include "scene_type_config.pb.h"
#include "vehicle_service.pb.h"
#include "vehicle_status.pb.h"

namespace planning {
namespace planner {

EnvironmentalModelManager::EnvironmentalModelManager() {
  LOG_DEBUG("EnvironmentalModelManager created\n");
}

void EnvironmentalModelManager::Init(planning::framework::Session *session) {
  session_ = session;
  InitContext();
}

void EnvironmentalModelManager::InitContext() {
  planning::common::SceneType scene_type = session_->get_scene_type();
  auto config_builder =
      session_->environmental_model().config_builder(scene_type);
  ego_config_ = config_builder->cast<planning::EgoPlanningConfig>();

  ego_state_manager_ptr_ =
      std::make_shared<planning::EgoStateManager>(session_);
  session_->mutable_environmental_model()->set_ego_state(
      ego_state_manager_ptr_);

  virtual_lane_manager_ptr_ =
      std::make_shared<planning::VirtualLaneManager>(session_);
  session_->mutable_environmental_model()->set_virtual_lane_manager(
      virtual_lane_manager_ptr_);

  obstacle_manager_ptr_ =
      std::make_shared<planning::ObstacleManager>(config_builder, session_);
  session_->mutable_environmental_model()->set_obstacle_manager(
      obstacle_manager_ptr_);

  //   traffic_light_decision_manager_ptr_ =
  //   std::make_shared<planning::TrafficLightDecisionManager>(
  //       config_builder, session_, virtual_lane_manager_ptr_);
  //   session_->mutable_environmental_model()->set_traffic_light_decision_manager(
  //       traffic_light_decision_manager_ptr_);

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
}

bool EnvironmentalModelManager::Run(planning::framework::Frame *frame) {
  LOG_DEBUG("EnvironmentalModelManager run\n");
  frame_ = frame;

  auto current_time = IflyTime::Now_ms();

  auto &local_view = session_->environmental_model().get_local_view();

  // 通过配置项进行实时长时的切换 true: 长时规划
  bool enable_NOA = ego_config_.enable_NOA;
  auto location_valid =
      local_view.localization_estimate.msf_status().available() &&
      (local_view.localization_estimate.msf_status().msf_status() !=
       LocalizationOutput::MsfStatus::ERROR) &&
      local_view.road_info.local_point_valid() &&
      local_view.fusion_objects_info.local_point_valid() && enable_NOA;
  // location_valid = true; //hack
  session_->mutable_environmental_model()->set_location_valid(location_valid);
  // Step 1) update vehicleDbwStatus
  session_->mutable_environmental_model()->UpdateVehicleDbwStatus(
      local_view.function_state_machine_info.current_state() ==
          FuncStateMachine::FunctionalState::SCC_ACTIVATE ||
      local_view.function_state_machine_info.current_state() ==
          FuncStateMachine::FunctionalState::ACC_ACTIVATE);

  // 自动有效，临时hack
  // session_->mutable_environmental_model()->UpdateVehicleDbwStatus(true);
  last_feed_time_[FEED_VEHICLE_DBW_STATUS] = current_time;

  // Step 2) update ego_state
  if (!ego_state_update(current_time, local_view)) {
    LOG_ERROR("ego_state_update false\n");
    return false;
  }

  // Step 3) update virtual_lane
  if (!virtual_lane_manager_ptr_->update(local_view.road_info)) {
    LOG_ERROR("virtual_lane_manager update failed");
    return false;
  } else {
    // 后面需要判断是否为地图
    last_feed_time_[FEED_FUSION_LANES_INFO] = local_view.road_info_recv_time;
  }

  // Step 4) update obstacle
  if (!obstacle_prediction_update(current_time, local_view)) {
    return false;
  }
  obstacle_manager_ptr_->update();

  // Step 5) update reference path
  if (!reference_path_manager_ptr_->update()) {
    LOG_ERROR("reference_path_manager update fail\n");
    return false;
  }

  lateral_obstacle_ptr_->update();
  lane_tracks_mgr_ptr_->update_lane_tracks();

  auto end_time = IflyTime::Now_ms();
  LOG_DEBUG("EnvironmentalModelManager::Run cost time:%f\n",
            end_time - current_time);

  std::string status_msg;
  if (!InputReady(current_time, status_msg)) {
    LOG_ERROR("InputReady is failed !!!! \n");
    return false;
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
  prediction_info.clear();
  if (local_view.prediction_result.prediction_obstacle_list_size() > 0) {
    std::unordered_set<uint> prediction_obj_id_set;
    truncate_prediction_info(
        local_view.prediction_result,
        local_view.localization_estimate.header().timestamp(),
        prediction_obj_id_set);
    int num = 0;
    for (auto &obj : local_view.fusion_objects_info.fusion_object()) {
      num++;
      if (num > local_view.fusion_objects_info.fusion_object_num()) {
        break;
      }
      // WB HACK： Only use front obstacles in long time planning
      if (obj.additional_info().fusion_source() != FusionSource::FUSION &&
          obj.additional_info().fusion_source() != FusionSource::CAMERA_ONLY) {
        LOG_DEBUG("[obstacle_prediction_update] ignore obstacle! : [%d] \n",
                  obj.additional_info().track_id());
        continue;
      }

      if (prediction_obj_id_set.find(obj.additional_info().track_id()) ==
          prediction_obj_id_set.end()) {
        transform_fusion_to_prediction(
            obj, (double)local_view.fusion_objects_info.header().timestamp());
      }
    }
  } else {
    int num = 0;
    for (auto &obj : local_view.fusion_objects_info.fusion_object()) {
      num++;
      if (num > local_view.fusion_objects_info.fusion_object_num()) {
        break;
      }

      // 过滤未与相机融合， 且在相机FOV之内的
      if ((!(obj.additional_info().fusion_source() & OBSTACLE_SOURCE_CAMERA)) && 
          (obj.common_info().relative_center_position().x() > 0 && 
          tan(25) > fabs(obj.common_info().relative_center_position().y() / obj.common_info().relative_center_position().x())) ||
          fabs(obj.common_info().relative_center_position().y()) > 10 ||
          obj.common_info().shape().length() == 0 ||
          obj.common_info().shape().width() == 0) {
        LOG_DEBUG("[obstacle_prediction_update] ignore obstacle! : [%d] \n",
                  obj.additional_info().track_id());
        continue;
      }
      transform_fusion_to_prediction(
          obj, (double)local_view.fusion_objects_info.header().timestamp());
    }
  }
  last_feed_time_[FEED_FUSION_INFO] = local_view.fusion_objects_info_recv_time;
  last_feed_time_[FEED_PREDICTION_INFO] =
      local_view.prediction_result_recv_time;
  return true;
}

void EnvironmentalModelManager::vehicle_status_adaptor(
    double current_time, const LocalView &local_view,
    common::VehicleStatus &vehicle_status) {
  const auto &vehicel_service_output_info =
      local_view.vehicel_service_output_info;
  const auto &localization_estimate = local_view.localization_estimate;
  const auto &hmi_mcu_inner_info = local_view.hmi_mcu_inner_info;
  vehicle_status.mutable_header()->set_timestamp_us(
      vehicel_service_output_info.header().timestamp());

  if (session_->environmental_model().location_valid()) {
    vehicle_status.mutable_heading_yaw()
        ->mutable_heading_yaw_data()
        ->set_value_rad(localization_estimate.pose().euler_angles().yaw());
    vehicle_status.mutable_location()->set_available(true);
    auto llh_position = localization_estimate.pose().llh_position();
    vehicle_status.mutable_location()
        ->mutable_location_geographic()
        ->set_latitude_degree(llh_position.lat());
    vehicle_status.mutable_location()
        ->mutable_location_geographic()
        ->set_longitude_degree(llh_position.lon());
    vehicle_status.mutable_location()
        ->mutable_location_geographic()
        ->set_altitude_meter(llh_position.height());
    auto local_position = localization_estimate.pose().local_position();
    vehicle_status.mutable_location()->mutable_location_enu()->set_x(
        local_position.x());
    vehicle_status.mutable_location()->mutable_location_enu()->set_y(
        local_position.y());
    vehicle_status.mutable_location()->mutable_location_enu()->set_z(
        local_position.z());

    auto enu_orientation = localization_estimate.pose().orientation();
    vehicle_status.mutable_location()
        ->mutable_location_enu()
        ->mutable_orientation()
        ->set_x(enu_orientation.qx());
    vehicle_status.mutable_location()
        ->mutable_location_enu()
        ->mutable_orientation()
        ->set_y(enu_orientation.qy());
    vehicle_status.mutable_location()
        ->mutable_location_enu()
        ->mutable_orientation()
        ->set_z(enu_orientation.qz());
    vehicle_status.mutable_location()
        ->mutable_location_enu()
        ->mutable_orientation()
        ->set_w(enu_orientation.qw());
    last_feed_time_[FEED_EGO_ENU] = local_view.localization_estimate_recv_time;
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
    last_feed_time_[FEED_EGO_ENU] = local_view.localization_estimate_recv_time;
  }

  vehicle_status.mutable_velocity()->mutable_cruise_velocity()->set_value_mps(
      hmi_mcu_inner_info.acc_set_disp_speed());

  // vehicle_status.mutable_velocity()->mutable_cruise_velocity()->set_value_mps(
  //     18.0);

  if (vehicel_service_output_info.yaw_rate_available()) {
    vehicle_status.mutable_angular_velocity()->set_available(true);
    vehicle_status.mutable_angular_velocity()
        ->mutable_heading_yaw_rate()
        ->set_value_rps(vehicel_service_output_info.yaw_rate());
  } else {
    vehicle_status.mutable_angular_velocity()->set_available(false);
    vehicle_status.mutable_angular_velocity()
        ->mutable_heading_yaw_rate()
        ->set_value_rps(0.);
  }

  if (vehicel_service_output_info.vehicle_speed_available()) {
    vehicle_status.mutable_velocity()->set_available(true);
    vehicle_status.mutable_velocity()
        ->mutable_heading_velocity()
        ->set_value_mps(vehicel_service_output_info.vehicle_speed());
    last_feed_time_[FEED_EGO_VEL] =
        local_view.vehicel_service_output_info_recv_time;
  }

  if (vehicel_service_output_info.long_acceleration_available()) {
    vehicle_status.mutable_brake_info()
        ->mutable_brake_info_data()
        ->set_acceleration_on_vehicle_wheel(
            vehicel_service_output_info.long_acceleration());
    last_feed_time_[FEED_EGO_ACC] =
        local_view.vehicel_service_output_info_recv_time;
  }

  if (vehicel_service_output_info.vehicle_speed_display_available()) {
    vehicle_status.mutable_velocity()->set_available(true);
    vehicle_status.mutable_velocity()->set_hmi_speed(
        vehicel_service_output_info.vehicle_speed_display());
  }

  if (session_->environmental_model().get_hdmap_valid()) {
    auto linear_velocity_from_wheel =
        localization_estimate.pose().linear_velocity_from_wheel();
    vehicle_status.mutable_velocity()
        ->mutable_heading_velocity()
        ->set_value_mps(linear_velocity_from_wheel);
  }

  if (vehicel_service_output_info.steering_wheel_angle_available()) {
    auto steering_data = vehicle_status.mutable_steering_wheel();
    steering_data->set_available(true);
    steering_data->mutable_steering_wheel_data()->set_steering_wheel_rad(
        vehicel_service_output_info.steering_wheel_angle());
    steering_data->mutable_steering_wheel_data()->set_steering_wheel_torque(
        vehicel_service_output_info.power_train_current_torque());

    last_feed_time_[FEED_EGO_STEER_ANGLE] =
        local_view.vehicel_service_output_info_recv_time;
  }

  if (vehicel_service_output_info.fl_wheel_speed_available() &&
      vehicel_service_output_info.fr_wheel_speed_available() &&
      vehicel_service_output_info.rl_wheel_speed_available() &&
      vehicel_service_output_info.rr_wheel_speed_available()) {
    vehicle_status.mutable_wheel_velocity()->set_available(true);  // hack
    auto wheel_velocity4d =
        vehicle_status.mutable_wheel_velocity()->mutable_wheel_velocity4d();
    wheel_velocity4d->set_front_left(
        vehicel_service_output_info.fl_wheel_speed());
    wheel_velocity4d->set_front_right(
        vehicel_service_output_info.fr_wheel_speed());
    wheel_velocity4d->set_rear_left(
        vehicel_service_output_info.rl_wheel_speed());
    wheel_velocity4d->set_rear_right(
        vehicel_service_output_info.rr_wheel_speed());
    last_feed_time_[FEED_WHEEL_SPEED_REPORT] =
        local_view.vehicel_service_output_info_recv_time;
  }

  if (vehicel_service_output_info.power_train_override_flag_available()) {
    vehicle_status.mutable_throttle()->mutable_throttle_data()->set_override(
        vehicel_service_output_info.power_train_override_flag());
  }

  auto vehicle_light = vehicle_status.mutable_vehicle_light();
  if (vehicel_service_output_info.left_turn_light_state() &&
      vehicel_service_output_info.left_turn_light_state_available()) {
    vehicle_light->mutable_vehicle_light_data()
        ->mutable_turn_signal()
        ->set_value(common::TurnSignalType::LEFT);
    last_feed_time_[FEED_MISC_REPORT] =
        local_view.vehicel_service_output_info_recv_time;
  } else if (vehicel_service_output_info.right_turn_light_state() &&
             vehicel_service_output_info.right_turn_light_state_available()) {
    vehicle_light->mutable_vehicle_light_data()
        ->mutable_turn_signal()
        ->set_value(common::TurnSignalType::RIGHT);
    last_feed_time_[FEED_MISC_REPORT] =
        local_view.vehicel_service_output_info_recv_time;
  } else if (vehicel_service_output_info.hazard_light_state() &&
             vehicel_service_output_info.hazard_light_state_available()) {
    vehicle_light->mutable_vehicle_light_data()
        ->mutable_turn_signal()
        ->set_value(common::TurnSignalType::EMERGENCY_FLASHER);
    last_feed_time_[FEED_MISC_REPORT] =
        local_view.vehicel_service_output_info_recv_time;
  } else {
    vehicle_light->mutable_vehicle_light_data()
        ->mutable_turn_signal()
        ->set_value(common::TurnSignalType::NONE);
    last_feed_time_[FEED_MISC_REPORT] =
        local_view.vehicel_service_output_info_recv_time;
  }

  if (vehicel_service_output_info.auto_light_state_available()) {
    vehicle_light->mutable_vehicle_light_data()->set_auto_light_state(
        vehicel_service_output_info.auto_light_state());
    last_feed_time_[FEED_MISC_REPORT] =
        local_view.vehicel_service_output_info_recv_time;
  }

  if (vehicel_service_output_info.driver_hand_torque_available()) {
    vehicle_status.mutable_driver_hand_state()->set_driver_hand_torque(
        vehicel_service_output_info.driver_hand_torque());
  }

  if (vehicel_service_output_info.driver_hands_off_state_available()) {
    vehicle_status.mutable_driver_hand_state()->set_driver_hands_off_state(
        vehicel_service_output_info.driver_hands_off_state());
  }
}

void EnvironmentalModelManager::truncate_prediction_info(
    const Prediction::PredictionResult &prediction_result,
    double cur_timestamp_us, std::unordered_set<uint> &prediction_obj_id_set) {
  assert(session_ != nullptr);
  double current_time = session_->planning_output_context()
                            .planning_status()
                            .planning_result.next_timestamp;
  auto init_relative_time = session_->environmental_model()
                                .get_ego_state_manager()
                                ->planning_init_point()
                                .relative_time;
  auto &prediction_info =
      session_->mutable_environmental_model()->get_mutable_prediction_info();
  prediction_info.clear();

  for (const auto &prediction_object :
       prediction_result.prediction_obstacle_list()) {
    // HACK: 不使用左前、右前radar
    if (prediction_object.fusion_obstacle().additional_info().fusion_source() ==
            4 ||
        prediction_object.fusion_obstacle().additional_info().fusion_source() ==
            8 ||
        prediction_object.fusion_obstacle().common_info().shape().length() ==
            0 ||
        prediction_object.fusion_obstacle().common_info().shape().width() ==
            0) {
      continue;
    }
    PredictionObject cur_predicion_obj;
    cur_predicion_obj.id =
        prediction_object.fusion_obstacle().additional_info().track_id();
    prediction_obj_id_set.emplace(cur_predicion_obj.id);
    cur_predicion_obj.type =
        prediction_object.fusion_obstacle().common_info().type();
    cur_predicion_obj.fusion_source =
        prediction_object.fusion_obstacle().additional_info().fusion_source();
    // todo:后面接口变化时，取trajectory的时间戳
    cur_predicion_obj.timestamp_us = prediction_result.header().timestamp();
    double prediction_relative_time =
        clip(prediction_result.header().timestamp() / 1.e+6 -
                 current_time / 1.e+3 - init_relative_time,
             0.0, -1.0);
    if (std::abs(prediction_relative_time) > 0.3) {
      LOG_DEBUG(
          "[prediction delay time] obstacle[%d] absolute start time %llu "
          "relative time %f start time %f "
          "init_relative_time %f",
          cur_predicion_obj.id, prediction_result.header().timestamp(),
          prediction_result.header().timestamp() / 1.e+6 - current_time / 1.e+3,
          prediction_relative_time, init_relative_time);
    }
    cur_predicion_obj.delay_time = prediction_relative_time;
#ifdef X86
    cur_predicion_obj.delay_time = 0;
#endif
    if (prediction_object.obstacle_intent().type() ==
        Prediction::ObstacleIntent::COMMON) {
      cur_predicion_obj.intention = ObstacleIntentType::COMMON;
    } else if (prediction_object.obstacle_intent().type() ==
               Prediction::ObstacleIntent::CUT_IN) {
      cur_predicion_obj.intention = ObstacleIntentType::CUT_IN;
    }
    // cur_predicion_obj.b_backup_freemove =
    // prediction_object.b_backup_freemove(); todo: clren
    // cur_predicion_obj.cutin_score = prediction_object.cutin_score();  todo:
    // clren
    cur_predicion_obj.position_x =
        prediction_object.fusion_obstacle().common_info().center_position().x();
    cur_predicion_obj.position_y =
        prediction_object.fusion_obstacle().common_info().center_position().y();
    cur_predicion_obj.relative_position_x = prediction_object.fusion_obstacle()
                                                .common_info()
                                                .relative_center_position()
                                                .x();
    cur_predicion_obj.relative_position_y = prediction_object.fusion_obstacle()
                                                .common_info()
                                                .relative_center_position()
                                                .y();
    cur_predicion_obj.relative_speed_x = prediction_object.fusion_obstacle()
                                             .common_info()
                                             .relative_velocity()
                                             .x();
    cur_predicion_obj.relative_speed_y = prediction_object.fusion_obstacle()
                                             .common_info()
                                             .relative_velocity()
                                             .y();
    cur_predicion_obj.acceleration_relative_to_ground_x =
        prediction_object.fusion_obstacle().common_info().acceleration().x();
    cur_predicion_obj.acceleration_relative_to_ground_y =
        prediction_object.fusion_obstacle().common_info().acceleration().y();
    cur_predicion_obj.relative_acceleration_x =
        prediction_object.fusion_obstacle()
            .common_info()
            .relative_acceleration()
            .x();
    cur_predicion_obj.relative_acceleration_y =
        prediction_object.fusion_obstacle()
            .common_info()
            .relative_acceleration()
            .y();
    cur_predicion_obj.length =
        prediction_object.fusion_obstacle().common_info().shape().length();
    cur_predicion_obj.width =
        prediction_object.fusion_obstacle().common_info().shape().width();
    cur_predicion_obj.speed = std::hypot(
        prediction_object.fusion_obstacle().common_info().velocity().x(),
        prediction_object.fusion_obstacle().common_info().velocity().y());
    cur_predicion_obj.yaw =
        prediction_object.fusion_obstacle().common_info().heading_angle();
    if ((int)cur_predicion_obj.yaw == 255) {
      cur_predicion_obj.yaw = 0;
    }
    cur_predicion_obj.theta = std::atan2(
        prediction_object.fusion_obstacle().common_info().velocity().y(),
        prediction_object.fusion_obstacle().common_info().velocity().x());
    cur_predicion_obj.acc = std::hypot(
        prediction_object.fusion_obstacle().common_info().acceleration().x(),
        prediction_object.fusion_obstacle().common_info().acceleration().y());
    // cur_predicion_obj.bottom_polygon_points =
    // prediction_object.bottom_polygon_points();
    // cur_predicion_obj.top_polygon_points =
    // prediction_object.top_polygon_points();

    for (const auto &prediction_traj : prediction_object.trajectory()) {
      PredictionTrajectory cur_prediction_trajectory;
      size_t traj_index = 0;
      double step_time = prediction_traj.relative_time();
      std::vector<PredictionTrajectoryPoint> trajectory_points;
      for (auto point : prediction_traj.trajectory_point()) {
        PredictionTrajectoryPoint trajectory_point;
        double point_relative_time =
            cur_predicion_obj.delay_time + step_time * traj_index;
        trajectory_point.relative_time = point_relative_time;
        trajectory_point.x = point.position().x();
        trajectory_point.y = point.position().y();
        trajectory_point.yaw = point.yaw();
        trajectory_point.speed = point.velocity();
        trajectory_point.theta = point.theta_vel();
        trajectory_point.prob = prediction_traj.confidence();
        trajectory_point.std_dev_x = point.gaussian_info().sigma_x();
        trajectory_point.std_dev_y = point.gaussian_info().sigma_y();
        // trajectory_point.std_dev_yaw = point.std_dev_yaw();
        // trajectory_point.std_dev_speed = point.std_dev_speed();
        trajectory_point.relative_ego_x = point.relative_position().x();
        trajectory_point.relative_ego_y = point.relative_position().y();
        trajectory_point.relative_ego_yaw = point.relative_yaw();
        trajectory_point.relative_ego_speed = std::hypot(
            point.relative_velocity().x(), point.relative_velocity().y());

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
      cur_predicion_obj.trajectory_array.emplace_back(
          cur_prediction_trajectory);
    }
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
    const FusionObjects::FusionObject &fusion_object, double timestamp) {
  assert(session_ != nullptr);
  if (session_ == nullptr) {
    return false;
  }

  auto &prediction_info =
      session_->mutable_environmental_model()->get_mutable_prediction_info();

  PredictionObject prediction_object;
  prediction_object.id = fusion_object.additional_info().track_id();
  prediction_object.type = fusion_object.common_info().type();
  prediction_object.fusion_source =
      fusion_object.additional_info().fusion_source();
  prediction_object.timestamp_us = timestamp;

  prediction_object.delay_time = 0.0;
  prediction_object.cutin_score = 0;
  prediction_object.position_x =
      fusion_object.common_info().center_position().x();
  prediction_object.position_y =
      fusion_object.common_info().center_position().y();
  prediction_object.length = fusion_object.common_info().shape().length();
  prediction_object.width = fusion_object.common_info().shape().width();
  prediction_object.speed =
      std::hypot(fusion_object.common_info().velocity().x(),
                 fusion_object.common_info().velocity().y());
  prediction_object.yaw = fusion_object.common_info().heading_angle();
  if ((int)prediction_object.yaw == 255) {
    prediction_object.yaw = 0;
  }
  prediction_object.acc =
      std::hypot(fusion_object.common_info().acceleration().x(),
                 fusion_object.common_info().acceleration().y());
  // add relative info for highway
  prediction_object.relative_position_x =
      fusion_object.common_info().relative_center_position().x();
  prediction_object.relative_position_y =
      fusion_object.common_info().relative_center_position().y();
  prediction_object.relative_speed_x =
      fusion_object.common_info().relative_velocity().x();
  prediction_object.relative_speed_y =
      fusion_object.common_info().relative_velocity().y();
  prediction_object.relative_acceleration_x =
      fusion_object.common_info().relative_acceleration().x();
  prediction_object.relative_acceleration_y =
      fusion_object.common_info().relative_acceleration().y();
  prediction_object.acceleration_relative_to_ground_x =
      fusion_object.common_info().acceleration().x();
  prediction_object.acceleration_relative_to_ground_y =
      fusion_object.common_info().acceleration().y();

  // TODO:clren  后面感知会直接给出yaw和theta;   这部分赋值需要重新更改
  prediction_object.relative_theta =
      fusion_object.common_info().relative_heading_angle();
  if ((int)prediction_object.relative_theta == 255) {
    prediction_object.relative_theta = 0;
  }

  PredictionTrajectoryPoint trajectory_point;
  trajectory_point.relative_time = 0;
  trajectory_point.x = prediction_object.position_x;
  trajectory_point.y = prediction_object.position_y;
  trajectory_point.yaw = prediction_object.yaw;
  trajectory_point.speed = prediction_object.speed;

  trajectory_point.theta =
      std::atan2(fusion_object.common_info().velocity().y(),
                 fusion_object.common_info().velocity().x());
  trajectory_point.prob = 1;
  trajectory_point.relative_ego_x = prediction_object.relative_position_x;
  trajectory_point.relative_ego_y = prediction_object.relative_position_y;
  trajectory_point.relative_ego_yaw = prediction_object.relative_theta;
  trajectory_point.relative_ego_speed = std::hypot(
      prediction_object.relative_speed_x, prediction_object.relative_speed_y);
  PredictionTrajectory tra;
  tra.trajectory.emplace_back(std::move(trajectory_point));
  prediction_object.trajectory_array.emplace_back(std::move(tra));
  prediction_info.emplace_back(std::move(prediction_object));
  return true;
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

  static const double kCheckTimeDiff = 2000.0;

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
    const char *feed_type_str = to_string(static_cast<FeedType>(i));
    if (last_feed_time_[i] > 0.0) {
      if (current_time - last_feed_time_[i] > kCheckTimeDiff) {
        LOG_ERROR("(%s)feed delay: %d, %s", __FUNCTION__, i, feed_type_str,
                  "\n");
        error_msg += std::string(feed_type_str) + "; ";
        res = false;
      }
    } else {
      // LOG_ERROR("(%s)no feed: %d, %s", __FUNCTION__, i, feed_type_str, "\n");
      LOG_ERROR("no feed: '%s'\n", feed_type_str);
      error_msg += std::string(feed_type_str) + "; ";
      res = false;
    }
  }
  return res;
}

}  // namespace planner
}  // namespace planning
