#include "viz2d_component.h"

#include <cmath>
#include <cstdint>

#include "common.pb.h"
#include "config/vehicle_param.h"
#include "cyber/common/global_data.h"
#include "log_glog.h"
#include "opencv_viz.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "src/common/ifly_time.h"
#include "src/common/utils_math.h"
#include "tools/visualization2d/opencv_viz.h"
#include "transform2d.h"
#include "viz2d_path.h"
#include "viz2d_perception.h"
#include "viz_text.h"
#include "viz_window.h"

namespace planning {
Viz2dComponent::Viz2dComponent(/* args */) {
  node_ = apollo::cyber::CreateNode(apollo::cyber::Binary::GetName());
}

int Viz2dComponent::Init() {
  localization_reader_ =
      node_->CreateReader<LocalizationOutput::LocalizationEstimate>(
          "/iflytek/localization/ego_pose",
          [this](
              const std::shared_ptr<LocalizationOutput::LocalizationEstimate>&
                  localization) {
            ILOG_DEBUG << "Received localization";
            std::lock_guard<std::mutex> lock(mutex_);
            localization_.CopyFrom(*localization);
          });

  fusion_obj_reader_ = node_->CreateReader<FusionObjects::FusionObjectsInfo>(
      "/iflytek/fusion/objects",
      [this](const std::shared_ptr<FusionObjects::FusionObjectsInfo>& obj) {
        ILOG_DEBUG << "Received obs";
        std::lock_guard<std::mutex> lock(mutex_);
        fusion_objs_.CopyFrom(*obj);
      });

  ground_line_reader_ =
      node_->CreateReader<GroundLinePerception::GroundLinePerceptionInfo>(
          "/iflytek/fusion/ground_line",
          [this](const std::shared_ptr<
                 GroundLinePerception::GroundLinePerceptionInfo>& gl) {
            ILOG_DEBUG << "Received groud line";
            std::lock_guard<std::mutex> lock(mutex_);
            ground_line_.CopyFrom(*gl);
          });

  planning_reader = node_->CreateReader<PlanningOutput::PlanningOutput>(
      "/iflytek/planning/plan",
      [this](const std::shared_ptr<PlanningOutput::PlanningOutput>& obj) {
        ILOG_DEBUG << "Received planning data: run planning callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        last_planning_.CopyFrom(*obj);
      });

  planning_debug_reader_ =
      node_->CreateReader<planning::common::PlanningDebugInfo>(
          "/iflytek/planning/debug_info",
          [this](
              const std::shared_ptr<planning::common::PlanningDebugInfo>& obj) {
            ILOG_DEBUG << "Received planning data: run planning callback.";
            std::lock_guard<std::mutex> lock(mutex_);
            planning_debug_.CopyFrom(*obj);
          });

  chassis_reader_ =
      node_->CreateReader<VehicleService::VehicleServiceOutputInfo>(
          "/iflytek/vehicle_service",
          [this](
              const std::shared_ptr<VehicleService::VehicleServiceOutputInfo>&
                  obj) {
            std::lock_guard<std::mutex> lock(mutex_);
            chassis_msg_.CopyFrom(*obj);
          });

  control_reader_ = node_->CreateReader<ControlCommand::ControlOutput>(
      "/iflytek/control/control_command",
      [this](const std::shared_ptr<ControlCommand::ControlOutput>&
                 control_command) {
        ILOG_DEBUG << "Received planning data: run planning callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        control_command_.CopyFrom(*control_command);
      });

  park_slot_info_writer_ =
      node_->CreateWriter<ParkingFusion::ParkingFusionInfo>(
          "/iflytek/fusion/parking_slot");

  fsm_info_writer_ = node_->CreateWriter<FuncStateMachine::FuncStateMachine>(
      "/iflytek/system_state/soc_state");

  ILOG_INFO << "open cv viz begin";

  main_window2d_init(viz2d_colors_dark_blue);

  main_window_ = get_main_window2d();

  viz2d_init_image(main_window_);

  VehicleParam vehicle_param;

  veh_local_polygon = init_adv_box(vehicle_param);

  park_slot_info_.Clear();
  park_info_updated_ = false;
  please_click_park_slot_ = false;

  global_base_pose_.x = 0.0;
  global_base_pose_.y = 0.0;
  global_base_pose_.theta = 0.0;

  fsm_info_.Clear();
  DisplayConfig config;
  replay_mode_ = config.replay_mode;

  return 0;
}

int Viz2dComponent::Close() {
  viz2d_release(main_window_);

  return 0;
}

int Viz2dComponent::UpdateLocalView(double max_steering_wheel_angle_) {
  {
    std::lock_guard<std::mutex> lock(mutex_);

    viz_subscribe_.localization_estimate.Clear();
    viz_subscribe_.localization_estimate.CopyFrom(localization_);

    car_global_pose_.x =
        viz_subscribe_.localization_estimate.pose().local_position().x();
    car_global_pose_.y =
        viz_subscribe_.localization_estimate.pose().local_position().y();
    car_global_pose_.theta =
        viz_subscribe_.localization_estimate.pose().heading();

    veh_pose_sin_theta_ = std::sin(car_global_pose_.theta);
    veh_pose_cos_theta_ = std::cos(car_global_pose_.theta);

    double time = IflyTime::Now_s();

    double delta_time = 1000;

    viz_subscribe_.fusion_objects_info.Clear();
    viz_subscribe_.fusion_objects_info.CopyFrom(fusion_objs_);

    viz_subscribe_.last_planning_output_.Clear();
    viz_subscribe_.last_planning_output_.CopyFrom(last_planning_);

    viz_subscribe_.ground_line_perception.Clear();
    ILOG_INFO << ground_line_.ground_lines_size();
    viz_subscribe_.ground_line_perception.CopyFrom(ground_line_);

    viz_subscribe_.control_output.Clear();
    viz_subscribe_.control_output.CopyFrom(control_command_);

    viz_subscribe_.vehicle_service.Clear();
    viz_subscribe_.vehicle_service.CopyFrom(chassis_msg_);

    viz_subscribe_.planning_debug_.Clear();
    viz_subscribe_.planning_debug_.CopyFrom(planning_debug_);
  }
  return 0;
}

int Viz2dComponent::Process(double max_steering_wheel_angle_) {
  viz2d_init_in_per_frame(main_window_);

  UpdateLocalView(max_steering_wheel_angle_);

  RULocalPolygonToGlobal(&veh_global_polygon, &veh_local_polygon,
                         &car_global_pose_);

  viz2d_draw_xy_axis(main_window_);

  ILOG_INFO << car_global_pose_.x << " y " << car_global_pose_.y << " theta "
            << car_global_pose_.theta;

  // use map ref frame as base pose in plot.
  bool ref_pose_is_map_ref_frame = false;
  bool ref_pose_is_veh_ref_frame = false;

  if (replay_mode_) {
    ref_pose_is_veh_ref_frame = true;
    global_base_pose_ = car_global_pose_;
  } else {
    ref_pose_is_map_ref_frame = true;
  }

  // draw localization polygon
  cv_draw_polygon(main_window_, &veh_global_polygon, &global_base_pose_,
                  viz2d_colors_orange, 2, ref_pose_is_map_ref_frame);

  double timestamp_s = -1.0;

  if (viz_subscribe_.localization_estimate.has_header()) {
    if (viz_subscribe_.localization_estimate.header().has_timestamp()) {
      timestamp_s = viz_subscribe_.localization_estimate.header().timestamp();

      timestamp_s = static_cast<double>(timestamp_s) / 1000000UL;

      apollo::cyber::Time time(timestamp_s);
      ILOG_INFO << time.ToString();
    }
  }

  if (timestamp_s < 0) {
    timestamp_s = IflyTime::Now_s();
  }

  viz2d_draw_localization_time(main_window_, timestamp_s);

  // obs
  std::vector<Position2D> obs_list;

  Position2D global;
  for (int i = 0; i < viz_subscribe_.fusion_objects_info.fusion_object_size();
       i++) {
    const Common::PointsSet3f& points =
        viz_subscribe_.fusion_objects_info.fusion_object(i)
            .additional_info()
            .polygon();

    for (size_t j = 0; j < points.points_size(); j++) {
      global.x = points.points(j).x();
      global.y = points.points(j).y();

      obs_list.emplace_back(global);
    }
  }

  draw_obstacle_points(obs_list, global_base_pose_, main_window_,
                       ref_pose_is_map_ref_frame);
  ILOG_INFO << "obs size " << obs_list.size();

  // ground line
  std::vector<Position2D> line_points;

  Position2D local_pos;

  planning::Transform2d tf;
  tf.SetBasePose(car_global_pose_);

  ILOG_INFO << viz_subscribe_.ground_line_perception.DebugString();

  for (int i = 0; i < viz_subscribe_.ground_line_perception.ground_lines_size();
       i++) {
    const GroundLinePerception::GroundLine& gl =
        viz_subscribe_.ground_line_perception.ground_lines(i);

    ILOG_INFO << "ground line point " << gl.points_2d_size();

    for (size_t j = 0; j < gl.points_2d_size(); j++) {
      local_pos.x = gl.points_2d(j).x();
      local_pos.y = gl.points_2d(j).y();

      tf.ULFLocalPointToGlobal(&global, local_pos);

      line_points.emplace_back(global);
    }
  }

  draw_ground_line_points(line_points, global_base_pose_, main_window_,
                          viz2d_colors_yellow, ref_pose_is_map_ref_frame);

  ILOG_INFO << "ground line size "
            << viz_subscribe_.ground_line_perception.ground_lines_size()
            << "point size" << line_points.size();

  // draw ratio for replay

  //   viz2d_draw_replay_info(main_window_, play_info_.ratio());

  // process mouse cmd

  cvSetMouseCallback(main_window_->win_name, on_Mouse);

  GetMouseOrKeyBoardMsg();

  if (park_slot_info_.has_header()) {
    draw_park_slot(park_slot_info_, global_base_pose_, main_window_,
                   viz2d_colors_silver, ref_pose_is_map_ref_frame);
  }

  viz2d_draw_global_trajectory(
      main_window_, viz_subscribe_.last_planning_output_, &global_base_pose_,
      viz2d_colors_purple, &veh_local_polygon, ref_pose_is_map_ref_frame);

  DrawHybridAstarPath(main_window_, viz_subscribe_.planning_debug_,
                      &global_base_pose_, ref_pose_is_map_ref_frame);

  viz_func_state_machine(fsm_info_, main_window_, &global_base_pose_);

  // plot a star virtual wall

  if (park_slot_info_.parking_fusion_slot_lists_size() > 0) {
    const ParkingFusion::ParkingFusionSlot& slot =
        park_slot_info_.parking_fusion_slot_lists(0);

    if (slot.corner_points_size() == 4) {
      Position2D right_up;
      Position2D left_up;
      Position2D right_lower;
      right_up.x = slot.corner_points(0).x();
      right_up.y = slot.corner_points(0).y();

      left_up.x = slot.corner_points(1).x();
      left_up.y = slot.corner_points(1).y();

      right_lower.x = slot.corner_points(2).x();
      right_lower.y = slot.corner_points(2).y();

      double slot_width = CalcPointDist(&right_up, &left_up);
      double slot_len = CalcPointDist(&right_up, &right_lower);

      // right-up-frame
      std::vector<Polygon2D> obs_list;
      GenerateVirtualWall(obs_list, 40.0, 15.0, slot_width, slot_len);

      Polygon2D global_polygon;
      for (size_t i = 0; i < obs_list.size(); i++) {
        RULocalPolygonToGlobal(&global_polygon, &obs_list[i],
                               &target_slot_pose_);

        cv_draw_polygon(main_window_, &global_polygon, &global_base_pose_,
                        viz2d_colors_yellow, 1.0, ref_pose_is_map_ref_frame);
      }
    }
  }

  // draw chassis
  VehicleParam vehicle_param;
  double v = viz_subscribe_.vehicle_service.vehicle_speed();
  double steering_wheel_angle =
      viz_subscribe_.vehicle_service.steering_wheel_angle();

  double front_wheel = steering_wheel_angle / vehicle_param.steer_ratio;
  front_wheel = front_wheel / M_PI * 180.0;

  uint32_t gear = viz_subscribe_.vehicle_service.gear_lever_state();

  viz2d_draw_chassis_feedback(main_window_, v, front_wheel, gear);

  // draw control cmd
  double acc = viz_subscribe_.control_output.acceleration();

  steering_wheel_angle = viz_subscribe_.control_output.steering();
  front_wheel = steering_wheel_angle / vehicle_param.steer_ratio;
  // front_wheel = front_wheel / M_PI * 180.0;

  Common::GearCommandValue control_gear =
      viz_subscribe_.control_output.gear_command_value();

  viz2d_draw_control_commond_info(main_window_, acc, front_wheel, control_gear);

  Position2D local_position;

  PublishMsg();

  viz2d_show_result_in_per_frame(main_window_);

  return 0;
}

int Viz2dComponent::GetMouseOrKeyBoardMsg() {
  cvSetMouseCallback(main_window_->win_name, on_Mouse);

  // get key
  key_value_ = cvWaitKey(50);  // 50 mill second delay

  if (key_value_ >= 0) {
    AINFO << "key is: " << key_value_;
  }

  // generate slot
  Position2D local_position;

  int ret = transform_cv_point_to_vrf_point(main_window_, &local_position);
  if (ret >= 0 && please_click_park_slot_) {
    Pose2D pose;
    // CvtPosLocalToGlobal(&pose.pos, &local_position, &global_base_pose_);
    pose.x = local_position.x;
    pose.y = local_position.y;

    // forward
    pose.theta = M_PI;

    target_slot_pose_ = pose;

    park_info_updated_ = true;

    please_click_park_slot_ = false;

    AINFO << "cv pose " << target_slot_pose_.x << target_slot_pose_.y;
  }

  bool is_parking_planning = false;
  bool is_parking_control = false;
  switch (key_value_) {
    case 'h':
      target_slot_pose_.theta += 15.0 * M_PI / 180.0;
      target_slot_pose_.theta = IflyUnifyTheta(target_slot_pose_.theta, M_PI);

      park_info_updated_ = true;
      break;
    case 'p':
      is_parking_planning = true;
      break;
    case 'c':
      is_parking_control = true;
      break;
    case 's':
      please_click_park_slot_ = true;
      break;

    default:
      break;
  }

  if (park_info_updated_) {
    park_slot_info_.Clear();

    uint64_t time_us = (uint64_t)IflyTime::Now_us();
    park_slot_info_.mutable_header()->set_timestamp(time_us);
    park_slot_info_.set_slot_counter(1);

    ParkingFusion::ParkingFusionSlot* slot =
        park_slot_info_.add_parking_fusion_slot_lists();
    slot->set_id(1);
    slot->set_uss_id(1);
    slot->set_type(Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL);
    slot->set_fusion_source(
        ParkingFusion::ParkingFusionSlot::FUSION_SLOT_SOURCE_TYPE_CAMERA_USS);

    slot->set_allow_parking(1);

    Polygon2D veh_box;
    Polygon2D slot_box;
    planning::VehicleParam veh_param;

    veh_box.vertexes[0].x = veh_param.right_edge_to_center;
    veh_box.vertexes[0].y = veh_param.length;

    veh_box.vertexes[1].x = -veh_param.left_edge_to_center;
    veh_box.vertexes[1].y = veh_param.length;

    veh_box.vertexes[2].x = veh_param.right_edge_to_center;
    veh_box.vertexes[2].y = 0;

    veh_box.vertexes[3].x = -veh_param.left_edge_to_center;
    veh_box.vertexes[3].y = 0;

    veh_box.vertex_num = 4;

    veh_box.shape = PolygonShape::box;

    slot_box = veh_box;

    double width = 0.3;
    double len = 1.0;

    slot_box.vertexes[0].x += width;
    slot_box.vertexes[0].y += len;

    slot_box.vertexes[1].x -= width;
    slot_box.vertexes[1].y += len;

    slot_box.vertexes[3].x -= width;

    slot_box.vertexes[2].x += width;

    Position2D global_position;
    for (size_t i = 0; i < 4; i++) {
      local_position = slot_box.vertexes[i];

      CvtPosLocalToGlobal(&global_position, &local_position,
                          &target_slot_pose_);

      Common::Point2f* point = slot->add_corner_points();
      point->set_x(global_position.x);
      point->set_y(global_position.y);
    }

    park_slot_info_.set_select_slot_id(1);
  }

  if (is_parking_planning) {
    uint64_t time_us = (uint64_t)IflyTime::Now_us();
    fsm_info_.mutable_header()->set_timestamp(time_us);

    if (fsm_info_.has_current_state() &&
        fsm_info_.current_state() == FuncStateMachine::PARK_IN_READY) {
      fsm_info_.set_current_state(FuncStateMachine::PARK_IN_ACTIVATE_WAIT);
    } else if (fsm_info_.current_state() ==
               FuncStateMachine::PARK_IN_ACTIVATE_WAIT) {
    } else {
      fsm_info_.set_current_state(FuncStateMachine::PARK_IN_READY);
    }

    fsm_info_.set_state_duration(100);
  }

  if (is_parking_control) {
    if (fsm_info_.has_current_state() &&
        fsm_info_.current_state() == FuncStateMachine::PARK_IN_ACTIVATE_WAIT) {
      fsm_info_.set_current_state(FuncStateMachine::PARK_IN_ACTIVATE_CONTROL);
    }
  }

  return 0;
}

int Viz2dComponent::PublishMsg() {
  park_slot_info_writer_->Write(park_slot_info_);

  fsm_info_writer_->Write(fsm_info_);

  park_info_updated_ = false;

  key_value_ = -1;

  return 0;
}

}  // namespace planning