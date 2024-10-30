#pragma once

#include "config/vehicle_param.h"
#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "opencv_viz.h"
#include "pose2d.h"
#include "viz2d_geometry.h"
#include "viz_subscribe.h"

namespace planning {
class Viz2dComponent {
 private:
  // reader
  std::shared_ptr<
      apollo::cyber::Reader<LocalizationOutput::LocalizationEstimate>>
      localization_reader_;

  LocalizationOutput::LocalizationEstimate localization_;

  std::shared_ptr<apollo::cyber::Reader<FusionObjects::FusionObjectsInfo>>
      fusion_obj_reader_;

  FusionObjects::FusionObjectsInfo fusion_objs_;

  std::shared_ptr<
      apollo::cyber::Reader<GroundLinePerception::FusionGroundLineInfo>>
      ground_line_reader_;

  GroundLinePerception::FusionGroundLineInfo ground_line_;

  std::shared_ptr<apollo::cyber::Reader<PlanningOutput::PlanningOutput>>
      planning_reader;
  PlanningOutput::PlanningOutput last_planning_;

  std::shared_ptr<apollo::cyber::Reader<planning::common::PlanningDebugInfo>>
      planning_debug_reader_;
  planning::common::PlanningDebugInfo planning_debug_;

  std::shared_ptr<
      apollo::cyber::Reader<VehicleService::VehicleServiceOutputInfo>>
      chassis_reader_;

  VehicleService::VehicleServiceOutputInfo chassis_msg_;

  std::shared_ptr<apollo::cyber::Reader<ControlCommand::ControlOutput>>
      control_reader_;
  ControlCommand::ControlOutput control_command_;

  // publish
  std::shared_ptr<apollo::cyber::Writer<ParkingFusion::ParkingFusionInfo>>
      park_slot_info_writer_;

  std::shared_ptr<apollo::cyber::Writer<FuncStateMachine::FuncStateMachine>>
      fsm_info_writer_;

  FuncStateMachine::FuncStateMachine fsm_info_;

  bool please_click_park_slot_;
  bool park_info_updated_;
  ParkingFusion::ParkingFusionInfo park_slot_info_;

  std::mutex mutex_;

  // node
  std::shared_ptr<apollo::cyber::Node> node_;

  // copy data
  VizSubscribe viz_subscribe_;

  // ***********************
  // internal  data
  viz2d_image* main_window_ = NULL;

  Polygon2D veh_local_polygon;
  Polygon2D veh_global_polygon;

  Pose2D car_global_pose_;
  double veh_pose_sin_theta_;
  double veh_pose_cos_theta_;
  // use a localization pose as base pose
  Pose2D global_base_pose_;

  Pose2D target_slot_pose_;

  double wheel_base;
  double front_wheel_angle;
  int key_value_ = -1;

  bool replay_mode_;

 public:
  Viz2dComponent(/* args */);
  ~Viz2dComponent(){};

  int Init();

  int Process(double max_steering_wheel_angle_);

  int Close();

  int UpdateLocalView(double max_steering_wheel_angle_);

  int PublishMsg();

 private:
  int GetMouseOrKeyBoardMsg();
};
}  // namespace planning