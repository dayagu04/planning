#include "src/modules/scenario/scenario_state_machine.h"
#include "src/modules/tasks/task_pipeline_context.h"
#include "src/modules/general_planning.h"
#include "src/modules/context/virtual_lane_manager.h"
#include "src/modules/context/ego_state_manager.h"
#include "src/modules/context/frenet_ego_state.h"
#include "src/modules/context/ego_state_manager.h"
#include "src/modules/context/frenet_ego_state.h"
#include "src/modules/context/lateral_obstacle.h"
#include "src/modules/context/obstacle_manager.h"
#include "src/modules/context/parking_slot_manager.h"
#include "src/modules/context/reference_path_manager.h"
#include "src/modules/context/traffic_light_decision_manager.h"
#include "src/modules/context/parking_slot_manager.h"
#include "src/modules/context/lateral_obstacle.h"
#include "src/modules/context/ego_planning_config.h"
#include "src/modules/common/math/linear_interpolation.h"
#include "res/include/proto/vehicle_service.pb.h"
#include "src/proto/generated_files/planning_config.pb.h"
#include "src/proto/generated_files/vehicle_status.pb.h"
#include "src/common/log.h"
#include "res/include/proto/fusion_road.pb.h"
#include "src/modules/environmental_model_manager.h"
#include "res/include/proto/fusion_objects.pb.h"
#include "src/common/ifly_time.h"
#include "src/modules/scenario/lateral_behavior_object_selector.h"


#include <array>
#include <cmath>

#include "gtest/gtest.h"

namespace planning {

// bool transform_fusion_to_prediction(const FusionObjects::FusionObject &fusion_object, double timestamp, planning::framework::Session *session);
bool transform_fusion_to_prediction(const FusionObjects::FusionObject &fusion_object, double timestamp, planning::framework::Session *session) {
    double ego_rear_axis_to_front_edge = 0;
    ego_rear_axis_to_front_edge = session->vehicle_config_context().get_vehicle_param().rear_axis_to_front_edge;

    double current_time = session->planning_output_context().planning_status().planning_result.next_timestamp;
    auto &prediction_info = session->mutable_environmental_model()->get_mutable_prediction_info();

    PredictionObject prediction_object;
    prediction_object.id = fusion_object.additional_info().track_id();
    prediction_object.type = fusion_object.common_info().type();
    prediction_object.timestamp_us = timestamp;

    prediction_object.delay_time = 0.0;
    prediction_object.cutin_score = 0;
    prediction_object.position_x = fusion_object.common_info().relative_center_position().x() - ego_rear_axis_to_front_edge;
    prediction_object.position_y = fusion_object.common_info().relative_center_position().y();
    prediction_object.length = fusion_object.common_info().shape().length();
    prediction_object.width = fusion_object.common_info().shape().width();
    prediction_object.speed = std::hypot(fusion_object.common_info().velocity().x(),
                                        fusion_object.common_info().velocity().y());
    prediction_object.yaw = fusion_object.common_info().heading_angle();
    prediction_object.acc = std::hypot(fusion_object.common_info().acceleration().x(),
                                        fusion_object.common_info().acceleration().y());
    // add relative info for highway
    prediction_object.relative_position_x = fusion_object.common_info().relative_center_position().x() - ego_rear_axis_to_front_edge;
    prediction_object.relative_position_y = fusion_object.common_info().relative_center_position().y();
    prediction_object.relative_speed_x = fusion_object.common_info().relative_velocity().x();
    prediction_object.relative_speed_y = fusion_object.common_info().relative_velocity().y();
    prediction_object.relative_acceleration_x = fusion_object.common_info().relative_acceleration().x();
    prediction_object.relative_acceleration_y = fusion_object.common_info().relative_acceleration().y();
    prediction_object.acceleration_relative_to_ground_x = fusion_object.common_info().acceleration().x();
    prediction_object.acceleration_relative_to_ground_y = fusion_object.common_info().acceleration().y();;
    prediction_object.relative_theta = fusion_object.common_info().relative_heading_angle();
    //std::vector<Point3d> bottom_polygon_points;
    //std::vector<Point3d> top_polygon_points;
    PredictionTrajectory tra;
    prediction_object.trajectory_array.emplace_back(tra);
    prediction_info.emplace_back(prediction_object);
}  

TEST(TestScenarioStateMachine, scenario_state_machine) {
  std::string log_file = "/asw/Planning/log/planning_log";
  bst::Log::getInstance().setConfig("Planning_Log", log_file.c_str(), bst::DEBUG);
  
  printf("TestScenarioStateMachine: scenario_state_machine");
  // std::unique_ptr<GeneralPlanning> planning_base_ = nullptr;
  // planning_base_ = std::make_unique<GeneralPlanning>();
  framework::Session session;
  session.Init();
  
  EgoPlanningConfigBuilder *config_builder;
  framework::Frame frame{&session};
  LocalView local_view;
  double time_stamp = IflyTime::Now_ms();
  for (int i = 0; i < 3; i++){
    auto road = local_view.road_info.add_lanes();
    road->set_order_id(i);
    road->set_virtual_id(i);
    road->set_relative_id(i - 1);
    road->set_ego_lateral_offset(std::fabs(1-i) * 3.8);
    road->set_lane_type(FusionRoad::LaneType::LANE_TYPE_NORMAL);
    road->set_lane_marks(FusionRoad::LaneDrivableDirection::DIRECTION_STRAIGHT);
    road->set_lane_source(FusionRoad::LaneSource::SOURCE_FUSION);
    road->mutable_lane_merge_split_point()->set_existence(true);
    auto merge_split_point_data = road->mutable_lane_merge_split_point()->add_merge_split_point_data();
    merge_split_point_data->set_distance(1000.);
    merge_split_point_data->set_length(100.);
    merge_split_point_data->set_is_split(false);
    merge_split_point_data->set_is_continue(true);
    merge_split_point_data->set_orientation(FusionRoad::LaneOrientation::ORIENTATION_UNKNOWN);
    road->mutable_left_lane_boundary()->set_existence(true);
    road->mutable_left_lane_boundary()->set_life_time(1000.);
    road->mutable_left_lane_boundary()->set_track_id(i-2 == 0 ? 1 : i-2);
    road->mutable_left_lane_boundary()->set_type(FusionRoad::LineType::LINE_TYPE_LANELINE);
    //   auto poly_coefficients = road->mutable_left_lane_boundary()->add_poly_coefficient(-1.9);
    road->mutable_left_lane_boundary()->add_poly_coefficient(1.9 + (1-i) * 3.8);
    road->mutable_left_lane_boundary()->add_poly_coefficient(0.);
    road->mutable_left_lane_boundary()->add_poly_coefficient(0.);
    road->mutable_left_lane_boundary()->add_poly_coefficient(0.);
    road->mutable_left_lane_boundary()->set_begin(0.);
    road->mutable_left_lane_boundary()->set_end(100.);
    auto left_segment = road->mutable_left_lane_boundary()->add_segment();
    left_segment->set_length(100.);
    left_segment->set_type(Common::LaneBoundaryType::MARKING_DASHED);
    road->mutable_right_lane_boundary()->set_existence(true);
    road->mutable_right_lane_boundary()->set_life_time(1000.);
    road->mutable_right_lane_boundary()->set_track_id(i-1 == 0 ? 1 : i-1);
    road->mutable_right_lane_boundary()->set_type(FusionRoad::LineType::LINE_TYPE_LANELINE);
    road->mutable_right_lane_boundary()->add_poly_coefficient(-1.9 + (1-i) * 3.8);
    road->mutable_right_lane_boundary()->add_poly_coefficient(0.);
    road->mutable_right_lane_boundary()->add_poly_coefficient(0.);
    road->mutable_right_lane_boundary()->add_poly_coefficient(0.);
    road->mutable_right_lane_boundary()->set_begin(0.);
    road->mutable_right_lane_boundary()->set_end(100.);
    auto right_segment = road->mutable_right_lane_boundary()->add_segment();
    right_segment->set_length(100.);
    right_segment->set_type(Common::LaneBoundaryType::MARKING_DASHED);
    for (int j = 0; j < 100; j++) {
        auto point = road->mutable_lane_reference_line()->add_virtual_lane_refline_points();
        point->mutable_car_point()->set_x(0. + j);
        point->mutable_car_point()->set_y(0. + (1-i)*3.8);
        point->mutable_enu_point()->set_x(0. + j);
        point->mutable_enu_point()->set_y(0. + (1-i)*3.8);
        point->mutable_enu_point()->set_z(0.);
        point->set_curvature(0.);
        point->set_heading(0.);
        point->set_distance_to_left_road_border(10.);
        point->set_distance_to_right_road_border(10.);
        point->set_distance_to_left_lane_border(1.9);
        point->set_distance_to_right_lane_border(1.9);
        point->set_lane_width(3.8);
        point->set_speed_limit(22.22); //m/s
        point->set_left_road_border_type(Common::LaneBoundaryType::MARKING_DASHED);
        point->set_right_road_border_type(Common::LaneBoundaryType::MARKING_DASHED);
        point->set_left_lane_border_type(Common::LaneBoundaryType::MARKING_DASHED);
        point->set_right_lane_border_type(Common::LaneBoundaryType::MARKING_DASHED);
        point->set_is_in_intersection(false);
        point->set_lane_type(FusionRoad::LaneType::LANE_TYPE_NORMAL);
    }
  }

  common::VehicleStatus vehicle_status;
  auto location_enu = vehicle_status.mutable_location()->mutable_location_enu();
  location_enu->set_x(10.0);
  vehicle_status.mutable_velocity()->mutable_heading_velocity()->set_value_mps(5.0);

  local_view.fusion_objects_info.mutable_header()->set_timestamp(time_stamp);
  local_view.fusion_objects_info.set_num(2);
  for (int i = 0; i < 2; i++) {
    auto fusion_obj = local_view.fusion_objects_info.add_fusion_object();
    fusion_obj->mutable_additional_info()->set_track_id(i+1);
    fusion_obj->mutable_common_info()->set_type(Common::ObjectType::OBJECT_TYPE_COUPE);
    // fusion_obj->mutable_common_info()->mutable_position()->set_x(fusion_object.long_position());
    // fusion_obj->mutable_common_info()->mutable_position()->set_y(fusion_object.lat_position());
    fusion_obj->mutable_common_info()->mutable_velocity()->set_x(0.1);
    fusion_obj->mutable_common_info()->mutable_velocity()->set_y(0.);
    fusion_obj->mutable_common_info()->mutable_relative_velocity()->set_x(-5.);
    fusion_obj->mutable_common_info()->mutable_relative_velocity()->set_y(0.);
    fusion_obj->mutable_common_info()->mutable_center_position()->set_x(20. + i * 10);
    fusion_obj->mutable_common_info()->mutable_center_position()->set_y(0. + i * 2.8);
    fusion_obj->mutable_common_info()->mutable_relative_center_position()->set_x(20. + i * 10);
    fusion_obj->mutable_common_info()->mutable_relative_center_position()->set_y(0. + i * 2.8);

    fusion_obj->mutable_common_info()->mutable_shape()->set_length(5.0);
    fusion_obj->mutable_common_info()->mutable_shape()->set_width(2.0);
    fusion_obj->mutable_common_info()->mutable_shape()->set_height(1.5);
  }

  
  // framework::Frame *frame;
  // frame(session);
  std::shared_ptr<planning::EgoStateManager> ego_state_manager_ptr_ = nullptr;
  std::shared_ptr<planning::ObstacleManager> obstacle_manager_ptr_ = nullptr;
  std::shared_ptr<planning::VirtualLaneManager> virtual_lane_manager_ptr_ =
      nullptr;
  std::shared_ptr<planning::ReferencePathManager> reference_path_manager_ptr_ =
      nullptr;
  std::shared_ptr<planning::LateralObstacle> lateral_obstacle_ptr_ = nullptr;
  std::shared_ptr<ObjectSelector> object_selector_ = nullptr;
  ego_state_manager_ptr_ =
      std::make_shared<planning::EgoStateManager>(&session);
  (&frame)->mutable_session()->mutable_environmental_model()->set_ego_state(
      ego_state_manager_ptr_);

  virtual_lane_manager_ptr_ =
      std::make_shared<planning::VirtualLaneManager>(&session);
  (&frame)->mutable_session()->mutable_environmental_model()->set_virtual_lane_manager(
      virtual_lane_manager_ptr_);

  obstacle_manager_ptr_ =
      std::make_shared<planning::ObstacleManager>(config_builder, &session);
  (&frame)->mutable_session()->mutable_environmental_model()->set_obstacle_manager(
      obstacle_manager_ptr_);

  reference_path_manager_ptr_ =
      std::make_shared<planning::ReferencePathManager>(&session);
  (&frame)->mutable_session()->mutable_environmental_model()->set_reference_path_manager(
      reference_path_manager_ptr_);

  lateral_obstacle_ptr_ =
      std::make_shared<planning::LateralObstacle>(config_builder, &session);
  (&frame)->mutable_session()->mutable_environmental_model()->set_lateral_obstacle(
      lateral_obstacle_ptr_);

  std::shared_ptr<ScenarioStateMachine> scenario_state_machine_ = nullptr;
  scenario_state_machine_ =
      std::make_shared<ScenarioStateMachine>(config_builder, &session);
  object_selector_ = std::make_shared<ObjectSelector>(config_builder, &session);
  (&frame)->mutable_session()->mutable_planning_context()->set_object_selector(
      object_selector_);

  ego_state_manager_ptr_->update(vehicle_status);
  virtual_lane_manager_ptr_->update(local_view.road_info);
  for (auto &obj : local_view.fusion_objects_info.fusion_object()) {
    transform_fusion_to_prediction(obj, (double)local_view.fusion_objects_info.header().timestamp(), (&frame)->mutable_session());
  }
//   obstacle_prediction_update(time_stamp, local_view);
  obstacle_manager_ptr_->update();
  // end_time = IflyTime::Now_ms();
  // LOG_DEBUG("obstacle_manager update time:%f\n", end_time - current_time);
  // current_time = end_time;

  reference_path_manager_ptr_->update();
  // end_time = IflyTime::Now_ms();
  // LOG_DEBUG("reference_path_manager update time:%f\n", end_time - current_time);
  // current_time = end_time;

  obstacle_manager_ptr_->assign_obstacles_to_lanes();
  // end_time = IflyTime::Now_ms();
  // LOG_DEBUG("assign_obstacles_to_lanes update time:%f\n", end_time - current_time);
  // current_time = end_time;
  //   traffic_light_decision_manager_ptr_->update();

  // TODO(Rui):lateral_obstacle_ptr_->update() only for real time planner
  lateral_obstacle_ptr_->update();

  scenario_state_machine_->init();
  (&frame)->mutable_session()->mutable_planning_context()->set_scenario_state_machine(
      scenario_state_machine_);

  object_selector_->update((&frame)->session()->planning_context().lat_behavior_state_machine_output().curr_state,
                           (&frame)->session()->planning_context().scenario_state_machine()->get_start_move_dist_lane(),
                           false, 80., false, false, false, false, false, -1);
  
  (void)scenario_state_machine_->update(&frame);

}
}  // namespace planning