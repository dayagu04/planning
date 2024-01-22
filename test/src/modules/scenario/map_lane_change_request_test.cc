#include <stdio.h>

#include <cstddef>

#include "gtest/gtest.h"
// #include "ilqr_define.h"
// #include "lane_change_requests/map_lane_change_request.h"
#include "adaptive_cruise_control.h"
#include "gtest/gtest.h"
#include "lateral_behavior_object_selector.h"
#include "mrc_condition.h"
#include "planning_output_context.h"
#include "scenario_state_machine.h"
#include "start_stop_enable.h"

using namespace std;

namespace planning {
namespace map_request_test {

bool transform_fusion_to_prediction(
    const FusionObjects::FusionObject &fusion_object, double timestamp,
    planning::framework::Session *session) {
  double ego_rear_axis_to_front_edge = 0;
  ego_rear_axis_to_front_edge = session->vehicle_config_context()
                                    .get_vehicle_param()
                                    .rear_axis_to_front_edge;

  double current_time = session->planning_output_context()
                            .planning_status()
                            .planning_result.next_timestamp;
  auto &prediction_info =
      session->mutable_environmental_model()->get_mutable_prediction_info();

  PredictionObject prediction_object;
  prediction_object.id = fusion_object.additional_info().track_id();
  prediction_object.type = fusion_object.common_info().type();
  prediction_object.timestamp_us = timestamp;

  prediction_object.delay_time = 0.0;
  prediction_object.cutin_score = 0;
  prediction_object.position_x =
      fusion_object.common_info().relative_center_position().x() -
      ego_rear_axis_to_front_edge;
  prediction_object.position_y =
      fusion_object.common_info().relative_center_position().y();
  prediction_object.length = fusion_object.common_info().shape().length();
  prediction_object.width = fusion_object.common_info().shape().width();
  prediction_object.speed =
      std::hypot(fusion_object.common_info().velocity().x(),
                 fusion_object.common_info().velocity().y());
  prediction_object.yaw = fusion_object.common_info().heading_angle();
  prediction_object.acc =
      std::hypot(fusion_object.common_info().acceleration().x(),
                 fusion_object.common_info().acceleration().y());
  // add relative info for highway
  prediction_object.relative_position_x =
      fusion_object.common_info().relative_center_position().x() -
      ego_rear_axis_to_front_edge;
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
  ;
  prediction_object.relative_theta =
      fusion_object.common_info().relative_heading_angle();
  // std::vector<Point3d> bottom_polygon_points;
  // std::vector<Point3d> top_polygon_points;
  PredictionTrajectory tra;
  prediction_object.trajectory_array.emplace_back(tra);
  prediction_info.emplace_back(prediction_object);
}
}  // namespace map_request_test

class MapLaneChangeRequestTest : public ::testing::Test {
 protected:
  void update() {
    // load engine configuration
    const std::string CONFIG_PATH = "/asw/planning/res/conf";
    std::string engine_config_path =
        CONFIG_PATH + "/planning_engine_config.json";
    common::ConfigurationContext::Instance()->load_engine_config_from_json(
        engine_config_path);
    auto engine_config =
        common::ConfigurationContext::Instance()->engine_config();

    std::string log_file = "/asw/planning/log/planning_log";
    bst::Log::getInstance().setConfig("Planning_Log", log_file.c_str(),
                                      bst::DEBUG);

    printf("TestMapLaneChangeRequest: map_lane_change_request");
    // std::unique_ptr<GeneralPlanning> planning_base_ = nullptr;
    // planning_base_ = std::make_unique<GeneralPlanning>();
    session.Init();

    auto config_builder =
        std::make_unique<EgoPlanningConfigBuilder>("", "empty");
    framework::Frame frame{&session};
    LocalView local_view;
    double time_stamp = IflyTime::Now_ms();
    for (int i = 0; i < 4; i++) {
      auto road = local_view.road_info.add_reference_line_msg();
      road->set_order_id(i);
      // road->set_virtual_id(i);
      road->set_relative_id(i - 3);
      // road->set_ego_lateral_offset(std::fabs(1 - i) * 3.8);
      auto lane_types = road->add_lane_types();
      lane_types->set_type(FusionRoad::LaneType::LANETYPE_NORMAL);
      lane_types->set_begin(0.);
      lane_types->set_end(200.);

      auto lane_marks = road->add_lane_marks();
      lane_marks->set_lane_mark(
          FusionRoad::LaneDrivableDirection::DIRECTION_STRAIGHT);
      lane_marks->set_begin(0.);
      lane_marks->set_begin(200.);

      auto lane_sources = road->add_lane_sources();
      lane_sources->set_source(
          FusionRoad::LaneSource::SOURCE_CAMERA_HDMAP_FUSION);
      lane_sources->set_begin(0.);
      lane_sources->set_begin(200.);

      if (road->order_id() == 2) {
        road->mutable_lane_merge_split_point()->set_existence(true);
        auto merge_split_point_data = road->mutable_lane_merge_split_point()
                                          ->add_merge_split_point_data();
        merge_split_point_data->set_distance(1000.);
        merge_split_point_data->set_length(100.);
        merge_split_point_data->set_is_split(true);
        merge_split_point_data->set_is_continue(true);
        merge_split_point_data->set_orientation(
            FusionRoad::LaneOrientation::ORIENTATION_UNKNOWN);
      }
      road->mutable_left_lane_boundary()->set_existence(true);
      road->mutable_left_lane_boundary()->set_life_time(1000.);
      road->mutable_left_lane_boundary()->set_track_id(i - 2 == 0 ? 1 : i - 2);
      road->mutable_left_lane_boundary()->set_type(
          FusionRoad::LineType::LINE_TYPE_LANELINE);
      //   auto poly_coefficients =
      //   road->mutable_left_lane_boundary()->add_poly_coefficient(-1.9);
      road->mutable_left_lane_boundary()->add_poly_coefficient(1.9 +
                                                               (3 - i) * 3.8);
      road->mutable_left_lane_boundary()->add_poly_coefficient(0.);
      road->mutable_left_lane_boundary()->add_poly_coefficient(0.);
      road->mutable_left_lane_boundary()->add_poly_coefficient(0.);
      road->mutable_left_lane_boundary()->set_begin(0.);
      road->mutable_left_lane_boundary()->set_end(200.);
      auto left_type_segments =
          road->mutable_left_lane_boundary()->add_type_segments();
      left_type_segments->set_length(200.);
      left_type_segments->set_type(Common::LaneBoundaryType::MARKING_DASHED);
      road->mutable_right_lane_boundary()->set_existence(true);
      road->mutable_right_lane_boundary()->set_life_time(1000.);
      road->mutable_right_lane_boundary()->set_track_id(i - 1 == 0 ? 1 : i - 1);
      road->mutable_right_lane_boundary()->set_type(
          FusionRoad::LineType::LINE_TYPE_LANELINE);
      road->mutable_right_lane_boundary()->add_poly_coefficient(-1.9 +
                                                                (3 - i) * 3.8);
      road->mutable_right_lane_boundary()->add_poly_coefficient(0.);
      road->mutable_right_lane_boundary()->add_poly_coefficient(0.);
      road->mutable_right_lane_boundary()->add_poly_coefficient(0.);
      road->mutable_right_lane_boundary()->set_begin(0.);
      road->mutable_right_lane_boundary()->set_end(200.);
      auto right_type_segments =
          road->mutable_right_lane_boundary()->add_type_segments();
      right_type_segments->set_length(200.);
      right_type_segments->set_type(Common::LaneBoundaryType::MARKING_DASHED);
      road->mutable_lane_reference_line()->add_poly_coefficient_car(
          0. + (3 - i) * 3.8);
      road->mutable_lane_reference_line()->add_poly_coefficient_car(0.);
      road->mutable_lane_reference_line()->add_poly_coefficient_car(0.);
      road->mutable_lane_reference_line()->add_poly_coefficient_car(0.);
      for (int j = 0; j < 200; j++) {
        auto point = road->mutable_lane_reference_line()
                         ->add_virtual_lane_refline_points();
        point->set_track_id(0);
        point->mutable_car_point()->set_x(0. + j);
        point->mutable_car_point()->set_y(0. + (3 - i) * 3.8);
        point->mutable_enu_point()->set_x(0. + j);
        point->mutable_enu_point()->set_y(0. + (3 - i) * 3.8);
        point->mutable_enu_point()->set_z(0.);
        point->set_curvature(0.);
        point->set_car_heading(0.);
        point->set_distance_to_left_road_border(10.);
        point->set_distance_to_right_road_border(10.);
        point->set_distance_to_left_lane_border(1.9);
        point->set_distance_to_right_lane_border(1.9);
        point->set_lane_width(3.8);
        point->set_speed_limit_max(22.22);  // m/s
        point->set_speed_limit_min(0.);
        point->set_left_road_border_type(
            Common::LaneBoundaryType::MARKING_DASHED);
        point->set_right_road_border_type(
            Common::LaneBoundaryType::MARKING_DASHED);
        point->set_left_lane_border_type(
            Common::LaneBoundaryType::MARKING_DASHED);
        point->set_right_lane_border_type(
            Common::LaneBoundaryType::MARKING_DASHED);
        point->set_is_in_intersection(false);
        point->set_lane_type(FusionRoad::LaneType::LANETYPE_NORMAL);
        point->set_s(0. + j);
      }
    }

    common::VehicleStatus vehicle_status;
    auto location_enu =
        vehicle_status.mutable_location()->mutable_location_enu();
    location_enu->set_x(10.0);
    vehicle_status.mutable_velocity()
        ->mutable_heading_velocity()
        ->set_value_mps(5.0);

    local_view.fusion_objects_info.mutable_header()->set_timestamp(time_stamp);
    local_view.fusion_objects_info.set_fusion_object_num(2);
    for (int i = 0; i < 2; i++) {
      auto fusion_obj = local_view.fusion_objects_info.add_fusion_object();
      fusion_obj->mutable_additional_info()->set_track_id(i + 1);
      fusion_obj->mutable_common_info()->set_type(
          Common::ObjectType::OBJECT_TYPE_COUPE);
      // fusion_obj->mutable_common_info()->mutable_position()->set_x(fusion_object.long_position());
      // fusion_obj->mutable_common_info()->mutable_position()->set_y(fusion_object.lat_position());
      fusion_obj->mutable_common_info()->mutable_velocity()->set_x(0.1);
      fusion_obj->mutable_common_info()->mutable_velocity()->set_y(0.);
      fusion_obj->mutable_common_info()->mutable_relative_velocity()->set_x(
          -5.);
      fusion_obj->mutable_common_info()->mutable_relative_velocity()->set_y(0.);
      fusion_obj->mutable_common_info()->mutable_center_position()->set_x(
          20. + i * 10);
      fusion_obj->mutable_common_info()->mutable_center_position()->set_y(
          0. + i * 2.8);
      fusion_obj->mutable_common_info()
          ->mutable_relative_center_position()
          ->set_x(20. + i * 10);
      fusion_obj->mutable_common_info()
          ->mutable_relative_center_position()
          ->set_y(0. + i * 2.8);

      fusion_obj->mutable_common_info()->mutable_shape()->set_length(5.0);
      fusion_obj->mutable_common_info()->mutable_shape()->set_width(2.0);
      fusion_obj->mutable_common_info()->mutable_shape()->set_height(1.5);
    }

    // framework::Frame *frame;
    // frame(session);

    ego_state_manager_ptr_ =
        std::make_shared<planning::EgoStateManager>(&session);
    (&frame)->mutable_session()->mutable_environmental_model()->set_ego_state(
        ego_state_manager_ptr_);

    virtual_lane_manager_ptr_ =
        std::make_shared<planning::VirtualLaneManager>(&session);
    (&frame)
        ->mutable_session()
        ->mutable_environmental_model()
        ->set_virtual_lane_manager(virtual_lane_manager_ptr_);

    obstacle_manager_ptr_ = std::make_shared<planning::ObstacleManager>(
        config_builder.get(), &session);
    (&frame)
        ->mutable_session()
        ->mutable_environmental_model()
        ->set_obstacle_manager(obstacle_manager_ptr_);

    reference_path_manager_ptr_ =
        std::make_shared<planning::ReferencePathManager>(&session);
    (&frame)
        ->mutable_session()
        ->mutable_environmental_model()
        ->set_reference_path_manager(reference_path_manager_ptr_);

    lateral_obstacle_ptr_ = std::make_shared<planning::LateralObstacle>(
        config_builder.get(), &session);
    (&frame)
        ->mutable_session()
        ->mutable_environmental_model()
        ->set_lateral_obstacle(lateral_obstacle_ptr_);

    lane_tracks_mgr_ptr_ = std::make_shared<LaneTracksManager>(
        *lateral_obstacle_ptr_, *virtual_lane_manager_ptr_, &session);
    session.mutable_environmental_model()->set_lane_tracks_manager(
        lane_tracks_mgr_ptr_);

    session.mutable_environmental_model()->set_highway_config_builder(
        config_builder.get());
    session.mutable_environmental_model()->set_location_valid(false);

    mrc_condition_ =
        std::make_shared<MrcCondition>(config_builder.get(), &session);
    (&frame)->mutable_session()->mutable_planning_context()->set_mrc_condition(
        mrc_condition_);

    adaptive_cruise_control_ =
        std::make_shared<AdaptiveCruiseControl>(config_builder.get(), &session);
    (&frame)
        ->mutable_session()
        ->mutable_planning_context()
        ->set_adaptive_cruise_control_function(adaptive_cruise_control_);

    start_stop_ptr_ =
        std::make_shared<StartStopEnable>(config_builder.get(), &session);
    (&frame)
        ->mutable_session()
        ->mutable_planning_context()
        ->set_start_stop_enable(start_stop_ptr_);

    object_selector_ =
        std::make_shared<ObjectSelector>(config_builder.get(), &session);
    (&frame)
        ->mutable_session()
        ->mutable_planning_context()
        ->set_object_selector(object_selector_);

    lc_lane_mgr_ = std::make_shared<LaneChangeLaneManager>(
        virtual_lane_manager_ptr_, &session);
    lc_req_mgr_ = std::make_shared<LaneChangeRequestManager>(
        &session, config_builder.get(), virtual_lane_manager_ptr_,
        lc_lane_mgr_);

    ego_state_manager_ptr_->update(vehicle_status);
    virtual_lane_manager_ptr_->update(local_view.road_info);
    for (auto &obj : local_view.fusion_objects_info.fusion_object()) {
      map_request_test::transform_fusion_to_prediction(
          obj, (double)local_view.fusion_objects_info.header().timestamp(),
          (&frame)->mutable_session());
    }
    //   obstacle_prediction_update(time_stamp, local_view);
    obstacle_manager_ptr_->update();
    // end_time = IflyTime::Now_ms();
    // LOG_DEBUG("obstacle_manager update time:%f\n", end_time - current_time);
    // current_time = end_time;

    reference_path_manager_ptr_->update();
    // end_time = IflyTime::Now_ms();
    // LOG_DEBUG("reference_path_manager update time:%f\n", end_time -
    // current_time); current_time = end_time;

    // obstacle_manager_ptr_->assign_obstacles_to_lanes();
    // end_time = IflyTime::Now_ms();
    // LOG_DEBUG("assign_obstacles_to_lanes update time:%f\n", end_time -
    // current_time); current_time = end_time;
    //   traffic_light_decision_manager_ptr_->update();

    // TODO(Rui):lateral_obstacle_ptr_->update() only for real time planner
    lateral_obstacle_ptr_->update();

    std::shared_ptr<ScenarioStateMachine> scenario_state_machine_ = nullptr;
    scenario_state_machine_ =
        std::make_shared<ScenarioStateMachine>(config_builder.get(), &session);

    // scenario_state_machine_->init();
    // (&frame)
    //     ->mutable_session()
    //     ->mutable_planning_context()
    //     ->set_scenario_state_machine(scenario_state_machine_);

    object_selector_->update((&frame)
                                 ->session()
                                 ->planning_context()
                                 .lat_behavior_state_machine_output()
                                 .curr_state,
                             0., false, 80., false, false, false, false, false,
                             -1);

    // (void)scenario_state_machine_->update(&frame);
    lc_req_mgr_->Update(ROAD_NONE, true);
  }

 public:
  framework::Session session;
  std::shared_ptr<planning::EgoStateManager> ego_state_manager_ptr_ = nullptr;
  std::shared_ptr<planning::ObstacleManager> obstacle_manager_ptr_ = nullptr;
  std::shared_ptr<planning::VirtualLaneManager> virtual_lane_manager_ptr_ =
      nullptr;
  std::shared_ptr<planning::ReferencePathManager> reference_path_manager_ptr_ =
      nullptr;
  std::shared_ptr<planning::LateralObstacle> lateral_obstacle_ptr_ = nullptr;
  std::shared_ptr<planning::LaneTracksManager> lane_tracks_mgr_ptr_ = nullptr;
  std::shared_ptr<MrcCondition> mrc_condition_ = nullptr;
  std::shared_ptr<AdaptiveCruiseControl> adaptive_cruise_control_ = nullptr;
  std::shared_ptr<StartStopEnable> start_stop_ptr_ = nullptr;
  std::shared_ptr<ObjectSelector> object_selector_ = nullptr;
  std::shared_ptr<planning::ScenarioStateMachine> scenario_state_machine_ =
      nullptr;
  std::shared_ptr<LaneChangeRequestManager> lc_req_mgr_ = nullptr;
  std::shared_ptr<LaneChangeLaneManager> lc_lane_mgr_;
};

TEST_F(MapLaneChangeRequestTest, OneFrameTest) { this->update(); }

}  // namespace planning
