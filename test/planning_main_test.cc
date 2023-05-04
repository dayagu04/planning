#include "planning_main_test.h"

#include "src/planning_component.h"
#include "modules/common/config_context.h"
#include "src/common/log.h"
#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "cyber/parameter/parameter.h"
#include "thirdparty/mjson/include/mjson/mjson.hpp"
#include <unistd.h>
#include "src/modules/common/define/debug_output.h"
#include "../res/include/proto/planning_plan.pb.h"
#include "../src/test/session_build.h"
using apollo::cyber::record::RecordReader;
using apollo::cyber::record::RecordMessage;
using apollo::cyber::Parameter;
using apollo::cyber::proto::Param;
ros::Publisher pubEgoObj;
ros::Publisher pubFusionOutObj;
ros::Publisher pubPlanningOutObj;
ros::Publisher pubRadarSurroundOutObj;
ros::Publisher pubControlObj;
// void sendMsg(const planning::LocalView& local_view,const FeiMa::Planning::PlanningOutput& planning_output,const FeiMa::control::ControlCommand& control_output,
//              const planning::DebugOutPut& debug_out_put);
void convertMsg(const OldLocalView &old_local_view, planning::LocalView &local_view);
bool init() {
  std::cout << "The planning component init!!!" << std::endl;
  std::string engine_config_path =
      std::string(CONFIG_PATH) + "/engine_config.json";
  if (!planning::common::ConfigurationContext::Instance()->load_engine_config_from_json(
      engine_config_path)) {
    return false;
  }

  auto engine_config =
      planning::common::ConfigurationContext::Instance()->engine_config();
  double time_stamp = IflyTime::Now_s();

  std::string log_file = engine_config.log_file_dir + "/planning_log_" +
                         std::to_string(time_stamp);
  std::cout << "log_file!!!" << log_file << std::endl;
  // Nanolog
  bst::LogLevel log_level;
  if (engine_config.log_level == "FETAL") {
    log_level = bst::FETAL;
  } else if (engine_config.log_level == "ERROR") {
    log_level = bst::ERROR;
  } else if (engine_config.log_level == "WARNING") {
    log_level = bst::WARNING;
  } else if (engine_config.log_level == "NOTICE") {
    log_level = bst::NOTICE;
  } else if (engine_config.log_level == "DEBUG") {
    log_level = bst::DEBUG;
  } else {
    log_level = bst::ERROR;
  }

  std::cout << "log_level!!!" << engine_config.log_level << std::endl;
  bst::Log::getInstance().setConfig("Planning_Log", log_file.c_str(),
                                    log_level);
  LOG_DEBUG("The planning component init!!! \n");
}
int main(int argc, char *argv[]) {
  while(1) {
    auto session= planning::update();
  }
}
int main1(int argc, char *argv[]) {
  int return_value=system("source /usr/bin/cyber/setup.bash");
  printf("result:%d",return_value);
  planning::PlanningComponent planningComponent;
  planningComponent.Init();
  OldLocalView old_local_view;
  planning::LocalView local_view;
  // init();
  std::unique_ptr<planning::GeneralPlanning> planning_base = std::make_unique<planning::GeneralPlanning>();

  const std::string channel_ego = "/ego_motion";
  const std::string channel_fusionout = "/fusion/fusion_object";
  const std::string channel_srnd_radar = "/surround_radar";
  const std::string channel_control = "/control";
  std::string bagPath="";
  if (argc>2) {
    LOG_DEBUG("argv input error\n");
    LOG_DEBUG("input num <=1\n");
    return 0;
  }
  if(argc==2) {
    bagPath=argv[1];
  } else {
    bagPath="/home/ros/Downloads/bag/jichang_urban_2.00000"; ///home/root/shuxihu_0118/fangxingdadao_4.00000   /home/ros/Downloads/360_jichang_26.00000
  }///home/ros/Downloads/bag/shuxihu_curve_brake_2.00000
  LOG_DEBUG("bag path is %s\n",bagPath.c_str());
  if (access(bagPath.c_str(), F_OK) != 0) {
    LOG_ERROR("%s is not exist\n",bagPath.c_str());
    std::cout<<bagPath<<" is not exist"<<std::endl;
    return 0;
  }
  std::shared_ptr<RecordReader> reader = std::make_shared<RecordReader>(bagPath);

  RecordMessage message;
  uint64_t reader_time;
  if (reader->ReadMessage(&message)) {
      reader_time = message.time + 50000000; // 50ms
  }

  apollo::cyber::Rate rate(15.0);
  FeiMa::FusionOut::FusionOut fusion_msg;
  FeiMa::EgoMotion::VehicleEgoMotion ego_motion_msg;
  J2::Radar::Radar_Surround_Info radar_surround_msg;
  FeiMa::control::ControlCommand control_msg;
  //ros相关
  ros::init(argc, argv, "talker");
  bool isrosMasterOk=ros::master::check();
  if (isrosMasterOk) {
    ros::NodeHandle nodeHandle;
    pubEgoObj = nodeHandle.advertise<proto_msgs::VehicleEgoMotion>("/sensor/proto/ego_motion", 1000);
    pubFusionOutObj = nodeHandle.advertise<proto_msgs::FusionOut>("/sensor/proto/fusion/fusion_object", 1000);
    pubPlanningOutObj = nodeHandle.advertise<proto_msgs::PlanningOutput>("/sensor/proto/planning", 1000);
    pubRadarSurroundOutObj = nodeHandle.advertise<proto_msgs::RadarSurroundInfo>("/sensor/proto/surround_radar", 2000);
    pubControlObj = nodeHandle.advertise<proto_msgs::ControlCommand>("/sensor/proto/control", 1000);
  }


  uint32_t count;
  while (apollo::cyber::OK()) //apollo::cyber::OK()
  {
    count++;
    LOG_DEBUG("***********read new frame %d******************\n",count);
    static uint32_t failTimes=0;
    while (reader->ReadMessage(&message, 0, reader_time)) {
      if(1) {
        RecordMessage message;
        while (reader->ReadMessage(&message, 0, reader_time)) {
            if (message.channel_name == channel_fusionout) {
                Parameter cyer_para("apollo::cyber::proto::ParamType::PROTOBUF", message.content, "FeiMa::FusionOut::FusionOut", reader->GetProtoDesc(message.channel_name));
                fusion_msg = cyer_para.value<FeiMa::FusionOut::FusionOut>();
            } else if (message.channel_name == channel_ego) {
                Parameter cyer_para("apollo::cyber::proto::ParamType::PROTOBUF", message.content, "FeiMa::EgoMotion::VehicleEgoMotion", reader->GetProtoDesc(message.channel_name));
                ego_motion_msg = cyer_para.value<FeiMa::EgoMotion::VehicleEgoMotion>();
            } else if (message.channel_name == channel_srnd_radar) {
                Parameter cyer_para("apollo::cyber::proto::ParamType::PROTOBUF", message.content, "J2::Radar::Radar_Surround_Info", reader->GetProtoDesc(message.channel_name));
                radar_surround_msg = cyer_para.value<J2::Radar::Radar_Surround_Info>();
            } else if (message.channel_name == channel_control) {
                Parameter cyer_para("apollo::cyber::proto::ParamType::PROTOBUF", message.content, "FeiMa::control::ControlCommand", reader->GetProtoDesc(message.channel_name));
                control_msg = cyer_para.value<FeiMa::control::ControlCommand>();
            }
        }
      }
      failTimes=0;
    }
    failTimes++;
    if(failTimes>2) {
      std::cout<<"reading bag endding"<<std::endl;
      return 1;
    }
    uint64_t time_int = reader_time / 1000000;
    reader_time += 50000000;
    PlanningOutput::PlanningOutput planning_output;
    planning::DebugOutput debug_output;
    PlanningHMI::PlanningHMIOutputInfoStr planning_hmi_Info;

  // std::unique_ptr<GeneralPlanning> planning_base =
  //     std::make_unique<GeneralPlanning>();
  // std::cout << "==============The planning enters RunOnce============="
  //           << std::endl;
  //
    old_local_view.fusion_out = fusion_msg;
    old_local_view.ego_motion = ego_motion_msg;
    old_local_view.srnd_radar_info = radar_surround_msg;
    convertMsg(old_local_view, local_view);
    planning_base->RunOnce(local_view, &planning_output, debug_output, planning_hmi_Info);
    if (isrosMasterOk) {
      // sendMsg(local_view,planning_output,control_msg,debug_out_put);
    } else {
      isrosMasterOk=ros::master::check();
      if (isrosMasterOk) {
        ros::NodeHandle nodeHandle;
        pubEgoObj = nodeHandle.advertise<proto_msgs::VehicleEgoMotion>("/sensor/proto/ego_motion", 1000);
        pubFusionOutObj = nodeHandle.advertise<proto_msgs::FusionOut>("/sensor/proto/fusion/fusion_object", 1000);
        pubPlanningOutObj = nodeHandle.advertise<proto_msgs::PlanningOutput>("/sensor/proto/planning", 1000);
        pubRadarSurroundOutObj = nodeHandle.advertise<proto_msgs::RadarSurroundInfo>("/sensor/proto/surround_radar", 2000);
        pubControlObj = nodeHandle.advertise<proto_msgs::ControlCommand>("/sensor/proto/control", 1000);
      }
    }
    rate.Sleep();
  }
  return 0;
}

void convertMsg(const OldLocalView &old_local_view, planning::LocalView &local_view) {
  local_view.vehicel_service_output_info.mutable_header()->set_timestamp(old_local_view.ego_motion.timestamp());
  local_view.vehicel_service_output_info.set_yaw_rate_available(1);
  local_view.vehicel_service_output_info.set_yaw_rate(old_local_view.ego_motion.yaw_rate());
  local_view.vehicel_service_output_info.set_long_acceleration(1);
  local_view.vehicel_service_output_info.set_long_acceleration(old_local_view.ego_motion.accel());

  local_view.vehicel_service_output_info.set_vehicle_speed_available(1);
  local_view.vehicel_service_output_info.set_vehicle_speed(old_local_view.ego_motion.veh_spd());

  local_view.vehicel_service_output_info.set_long_acceleration_available(1);

  local_view.vehicel_service_output_info.set_vehicle_speed_display_available(1);
  local_view.vehicel_service_output_info.set_vehicle_speed_display(old_local_view.ego_motion.veh_spd()+3);

  local_view.vehicel_service_output_info.set_steering_wheel_angle_available(1);
  local_view.vehicel_service_output_info.set_steering_wheel_angle(0.3);

  local_view.vehicel_service_output_info.set_left_turn_light_state_available(1);
  local_view.vehicel_service_output_info.set_right_turn_light_state_available(1);
  local_view.vehicel_service_output_info.set_left_turn_light_state(old_local_view.ego_motion.left_turn_light_state());
  local_view.vehicel_service_output_info.set_right_turn_light_state(old_local_view.ego_motion.right_turn_light_state());

  local_view.vehicel_service_output_info.set_fl_wheel_speed_available(1);
  local_view.vehicel_service_output_info.set_fr_wheel_speed_available(1);
  local_view.vehicel_service_output_info.set_rl_wheel_speed_available(1);
  local_view.vehicel_service_output_info.set_rr_wheel_speed_available(1);

  local_view.hmi_mcu_inner_info.set_acc_set_disp_speed(old_local_view.ego_motion.function().function_data().acc_cruise_velocity());
  local_view.hmi_mcu_inner_info.set_noa_active_switch(old_local_view.ego_motion.function().function_data().acc_active_switch());
  // local_view.hmi_mcu_inner_info
  // vehicleEgoMotionmsg.accel=old_local_view.ego_motion.accel();
  // vehicleEgoMotionmsg.yaw_rate=old_local_view.ego_motion.yaw_rate();
  // vehicleEgoMotionmsg.veh_spd=old_local_view.ego_motion.veh_spd();
  // vehicleEgoMotionmsg.ego_curvature=old_local_view.ego_motion.ego_curvature();
  // vehicleEgoMotionmsg.veh_motion_state=old_local_view.ego_motion.veh_motion_state();
  // vehicleEgoMotionmsg.left_turn_light_state=old_local_view.ego_motion.left_turn_light_state();
  // vehicleEgoMotionmsg.right_turn_light_state=old_local_view.ego_motion.right_turn_light_state();
  // vehicleEgoMotionmsg.function.function_data.acc_active_switch=old_local_view.ego_motion.function().function_data().acc_active_switch();
  // vehicleEgoMotionmsg.function.function_data.acc_cruise_velocity=old_local_view.ego_motion.function().function_data().acc_cruise_velocity();
  // vehicleEgoMotionmsg.function.function_data.lcc_active_switch=old_local_view.ego_motion.function().function_data().lcc_active_switch();

  //车道线
  FusionRoad::RoadInfo road_info;
  auto lane = road_info.add_lanes();
  lane->set_virtual_id(0);
  lane->set_relative_id(0);
  lane->mutable_left_lane_boundary()->set_existence(1);
  lane->mutable_left_lane_boundary()->add_poly_coefficient(1.5);
  lane->mutable_left_lane_boundary()->add_poly_coefficient(0);
  lane->mutable_left_lane_boundary()->add_poly_coefficient(0);
  lane->mutable_left_lane_boundary()->add_poly_coefficient(0);
  lane->mutable_left_lane_boundary()->set_begin(0);
  lane->mutable_left_lane_boundary()->set_end(80);
  lane->mutable_right_lane_boundary()->set_existence(1);
  lane->mutable_right_lane_boundary()->add_poly_coefficient(-1.5);
  lane->mutable_right_lane_boundary()->add_poly_coefficient(0);
  lane->mutable_right_lane_boundary()->add_poly_coefficient(0);
  lane->mutable_right_lane_boundary()->add_poly_coefficient(0);
  lane->mutable_right_lane_boundary()->set_begin(0);
  lane->mutable_right_lane_boundary()->set_end(80);
  double distance = 80;
  // lane->lane_reference_line().clear_virtual_lane_refline_points();
  for(float dis = -30 ; dis < distance; dis += 2) {

    auto point = lane->mutable_lane_reference_line()->add_virtual_lane_refline_points();
    point->mutable_car_point()->set_x(dis);
    point->mutable_car_point()->set_y(0);
    point->mutable_enu_point()->set_x(dis);
    point->mutable_enu_point()->set_y(0);
  }
  local_view.road_info = road_info;
  //感知障碍物

  local_view.fusion_objects_info.mutable_header()->set_timestamp(old_local_view.fusion_out.camera_timestamp());
  local_view.fusion_objects_info.set_num(old_local_view.fusion_out.fusion().num_obstacle());
  local_view.fusion_objects_info.clear_fusion_object();
  for (auto &fusion_object : old_local_view.fusion_out.fusion().objects()) {
    auto fusion_obj = local_view.fusion_objects_info.add_fusion_object();
    fusion_obj->mutable_additional_info()->set_track_id(fusion_object.track_id());
    fusion_obj->mutable_common_info()->set_type(fusion_object.type());
    // fusion_obj->mutable_common_info()->mutable_position()->set_x(fusion_object.long_position());
    // fusion_obj->mutable_common_info()->mutable_position()->set_y(fusion_object.lat_position());
    fusion_obj->mutable_common_info()->mutable_velocity()->set_x(fusion_object.long_velocity());
    fusion_obj->mutable_common_info()->mutable_velocity()->set_y(fusion_object.lat_velocity());
    fusion_obj->mutable_common_info()->mutable_relative_velocity()->set_x(local_view.vehicel_service_output_info.vehicle_speed());
    fusion_obj->mutable_common_info()->mutable_relative_velocity()->set_y(fusion_object.lat_velocity());
    fusion_obj->mutable_common_info()->mutable_center_position()->set_x(fusion_object.long_position());
    fusion_obj->mutable_common_info()->mutable_center_position()->set_y(fusion_object.lat_position());
    fusion_obj->mutable_common_info()->mutable_relative_center_position()->set_x(fusion_object.long_position());
    fusion_obj->mutable_common_info()->mutable_relative_center_position()->set_y(fusion_object.lat_position());

    fusion_obj->mutable_common_info()->mutable_shape()->set_length(fusion_object.length());
    fusion_obj->mutable_common_info()->mutable_shape()->set_width(fusion_object.width());
    fusion_obj->mutable_common_info()->mutable_shape()->set_height(fusion_object.height());
  }







}
// void sendMsg(const planning::LocalView& local_view,const FeiMa::Planning::PlanningOutput& planning_output,const FeiMa::control::ControlCommand& control_output,
//              const planning::DebugOutPut& debug_out_put)
// {
  // proto_msgs::VehicleEgoMotion vehicleEgoMotionmsg;
  // proto_msgs::FusionOut fusionOutMsg;
  // proto_msgs::PlanningOutput planningOutputMsg;
  // proto_msgs::RadarSurroundInfo radarSurroundInfoMsg;
  // proto_msgs::ControlCommand controlCommandMsg;

  // /* 自车信息*/
  // vehicleEgoMotionmsg.timestamp=local_view.ego_motion.accel();
  // vehicleEgoMotionmsg.accel=local_view.ego_motion.accel();
  // vehicleEgoMotionmsg.yaw_rate=local_view.ego_motion.yaw_rate();
  // vehicleEgoMotionmsg.veh_spd=local_view.ego_motion.veh_spd();
  // vehicleEgoMotionmsg.ego_curvature=local_view.ego_motion.ego_curvature();
  // vehicleEgoMotionmsg.veh_motion_state=local_view.ego_motion.veh_motion_state();
  // vehicleEgoMotionmsg.left_turn_light_state=local_view.ego_motion.left_turn_light_state();
  // vehicleEgoMotionmsg.right_turn_light_state=local_view.ego_motion.right_turn_light_state();
  // vehicleEgoMotionmsg.function.function_data.acc_active_switch=local_view.ego_motion.function().function_data().acc_active_switch();
  // vehicleEgoMotionmsg.function.function_data.acc_cruise_velocity=local_view.ego_motion.function().function_data().acc_cruise_velocity();
  // vehicleEgoMotionmsg.function.function_data.lcc_active_switch=local_view.ego_motion.function().function_data().lcc_active_switch();


  // /* 融合车道线输出*/
  // for (auto &fusion_lane_raw : local_view.fusion_out.lane().lane_marker()) {
  //   proto_msgs::LaneMarkerInfo lane_marker_msg;
  //   lane_marker_msg.track_id=fusion_lane_raw.track_id();
  //   lane_marker_msg.confidence=fusion_lane_raw.confidence();
  //   lane_marker_msg.measuring_status=fusion_lane_raw.measuring_status();
  //   lane_marker_msg.type_class=fusion_lane_raw.type_class();
  //   lane_marker_msg.color=fusion_lane_raw.color();
  //   lane_marker_msg.quality=fusion_lane_raw.quality();
  //   lane_marker_msg.crossing=fusion_lane_raw.crossing();
  //   lane_marker_msg.width_marking=fusion_lane_raw.width_marking();
  //   lane_marker_msg.view_range_start=fusion_lane_raw.view_range_start();
  //   lane_marker_msg.view_range_end=fusion_lane_raw.view_range_end();
  //   lane_marker_msg.coeff_a0=fusion_lane_raw.coeff_a0();
  //   lane_marker_msg.coeff_a1=fusion_lane_raw.coeff_a1();
  //   lane_marker_msg.coeff_a2=fusion_lane_raw.coeff_a2();
  //   lane_marker_msg.coeff_a3=fusion_lane_raw.coeff_a3();
  //   lane_marker_msg.tlc=fusion_lane_raw.tlc();
  //   lane_marker_msg.pos_type=fusion_lane_raw.pos_type();
  //   fusionOutMsg.lane.lane_marker.push_back(lane_marker_msg);
  // }

  // /* 融合障碍物输出*/
  // int i=0;
  // for (auto &fusion_object : local_view.fusion_out.fusion().objects()) {
  //   fusionOutMsg.fusion.objects[i].long_position=fusion_object.long_position();
  //   fusionOutMsg.fusion.objects[i].lat_position=fusion_object.lat_position();
  //   fusionOutMsg.fusion.objects[i].heading_angle=fusion_object.heading_angle();
  //   fusionOutMsg.fusion.objects[i].speed=fusion_object.speed();
  //   fusionOutMsg.fusion.objects[i].lat_velocity=fusion_object.lat_velocity();
  //   fusionOutMsg.fusion.objects[i].acceleration=fusion_object.acceleration();
  //   fusionOutMsg.fusion.objects[i].curvature=fusion_object.curvature();
  //   fusionOutMsg.fusion.objects[i].type=fusion_object.type();
  //   fusionOutMsg.fusion.objects[i].width=fusion_object.width();
  //   fusionOutMsg.fusion.objects[i].height=fusion_object.height();
  //   fusionOutMsg.fusion.objects[i].length=fusion_object.length();
  //   fusionOutMsg.fusion.objects[i].turn_indicator=fusion_object.turn_indicator();
  //   fusionOutMsg.fusion.objects[i].motion_pattern_current=fusion_object.motion_pattern_current();
  //   fusionOutMsg.fusion.objects[i].fusion_source=fusion_object.fusion_source();
  //   fusionOutMsg.fusion.objects[i].id=fusion_object.id();
  //   fusionOutMsg.fusion.objects[i].track_id=fusion_object.track_id();
  //   fusionOutMsg.fusion.objects[i].vision_id=fusion_object.vision_id();
  //   fusionOutMsg.fusion.objects[i].radar_id=fusion_object.radar_id();
  //   fusionOutMsg.fusion.objects[i].target_selection_type=fusion_object.target_selection_type();
  //   fusionOutMsg.fusion.objects[i].cut_type=fusion_object.cut_type();
  //   fusionOutMsg.fusion.objects[i].feature_target_cutflag=fusion_object.feature_target_cutflag();
  //   fusionOutMsg.fusion.objects[i].dst_lat_from_mid_of_laneself=fusion_object.dst_lat_from_mid_of_laneself();
  //   fusionOutMsg.fusion.objects[i].target_selection_ccr=fusion_object.target_selection_ccr();
  //   fusionOutMsg.fusion.objects[i].target_selection_vru=fusion_object.target_selection_vru();
  //   i++;
  // }

  // fusionOutMsg.ego_motion.timestamp=local_view.fusion_out.ego_motion().timestamp();
  // fusionOutMsg.ego_motion.yaw_rate=local_view.fusion_out.ego_motion().yaw_rate();
  // fusionOutMsg.ego_motion.veh_spd=local_view.fusion_out.ego_motion().veh_spd();
  // fusionOutMsg.ego_motion.accel=local_view.fusion_out.ego_motion().accel();
  // fusionOutMsg.ego_motion.ego_curvature=local_view.fusion_out.ego_motion().ego_curvature();
  // fusionOutMsg.ego_motion.veh_motion_state=local_view.fusion_out.ego_motion().veh_motion_state();

  // /* 雷达输出*/
  // radarSurroundInfoMsg.timestamp=local_view.srnd_radar_info.timestamp();
  // for (auto &one_srnd_radar_objs : local_view.srnd_radar_info.radar_surround_data()){
  //   proto_msgs::RadarSurroundData radar_Surround_Data_T_msg;
  //   radar_Surround_Data_T_msg.timestamp = one_srnd_radar_objs.timestamp();
  //   radar_Surround_Data_T_msg.radar_type = one_srnd_radar_objs.radar_type();
  //   radar_Surround_Data_T_msg.obj_report_cnt = one_srnd_radar_objs.obj_report_cnt();
  //   radar_Surround_Data_T_msg.num_obstacles = one_srnd_radar_objs.num_obstacles();
  //   for (auto &obj : one_srnd_radar_objs.obj_type()) {
  //     proto_msgs::Radar_OBJ_TYPE_T radar_OBJ_TYPE_T_msg;
  //     radar_OBJ_TYPE_T_msg.obj_id = obj.obj_id();
  //     radar_OBJ_TYPE_T_msg.obj_update_flag= obj.obj_update_flag();
  //     radar_OBJ_TYPE_T_msg.obj_x_pos = obj.obj_x_pos ();
  //     radar_OBJ_TYPE_T_msg.obj_y_pos = obj.obj_y_pos ();
  //     radar_OBJ_TYPE_T_msg.obj_x_vel_rel = obj.obj_x_vel_rel();
  //     radar_OBJ_TYPE_T_msg.obj_y_vel_rel = obj.obj_y_vel_rel();
  //     radar_OBJ_TYPE_T_msg.obj_x_acc_rel = obj.obj_x_acc_rel();
  //     radar_OBJ_TYPE_T_msg.obj_y_acc_rel = obj.obj_y_acc_rel();
  //     radar_OBJ_TYPE_T_msg.obj_confidence  = obj.obj_confidence();
  //     radar_OBJ_TYPE_T_msg.obj_type  = obj.obj_type();
  //     radar_OBJ_TYPE_T_msg.obj_heading_angle  = obj.obj_heading_angle();
  //     radar_OBJ_TYPE_T_msg.obj_width  = obj.obj_width();
  //     radar_OBJ_TYPE_T_msg.obj_length   = obj.obj_length();
  //     radar_Surround_Data_T_msg.obj_type.push_back(radar_OBJ_TYPE_T_msg);
  //   }
  //   radarSurroundInfoMsg.radar_surround_data.push_back(radar_Surround_Data_T_msg);
  // }

  // /*planning输出*/
  // planningOutputMsg.meta.timestamp_us=planning_output.meta().timestamp_us();
  // planningOutputMsg.meta.plan_timestamp_us=planning_output.meta().plan_timestamp_us();

  // planningOutputMsg.velocity.type=planning_output.velocity().type();
  // planningOutputMsg.velocity.target_value=planning_output.velocity().target_value();
  // planningOutputMsg.acceleration.type=planning_output.acceleration().type();
  // planningOutputMsg.acceleration.range_limit.min=planning_output.acceleration().range_limit().min();
  // planningOutputMsg.acceleration.range_limit.max=planning_output.acceleration().range_limit().max();

  // planningOutputMsg.turn_state.available=planning_output.turn_state().available();
  // planningOutputMsg.turn_state.TurnSignal=planning_output.turn_state().turnsignal();

  // // 横向输出为多项式
  // planningOutputMsg.trajectory.type=planning_output.trajectory().type();
  // for (int i=0;i<planning_output.trajectory().polynomial_curve().polynomial_size();i++) {
  //   planningOutputMsg.trajectory.polynomial_curve.polynomial.push_back(planning_output.trajectory().polynomial_curve().polynomial(i));
  // }
  // for (int i=0;i<planning_output.trajectory().trajectory_points_size();i++) {
  //   proto_msgs::TrajectoryPoint TrajectoryPoint_Msg;
  //   TrajectoryPoint_Msg.x=planning_output.trajectory().trajectory_points(i).x();
  //   TrajectoryPoint_Msg.y=planning_output.trajectory().trajectory_points(i).y();
  //   TrajectoryPoint_Msg.heading_angle=planning_output.trajectory().trajectory_points(i).heading_angle();
  //   TrajectoryPoint_Msg.curvature=planning_output.trajectory().trajectory_points(i).curvature();
  //   TrajectoryPoint_Msg.t=planning_output.trajectory().trajectory_points(i).t();
  //   TrajectoryPoint_Msg.v=planning_output.trajectory().trajectory_points(i).v();
  //   TrajectoryPoint_Msg.a=planning_output.trajectory().trajectory_points(i).a();
  //   TrajectoryPoint_Msg.s=planning_output.trajectory().trajectory_points(i).s();
  //   TrajectoryPoint_Msg.l=planning_output.trajectory().trajectory_points(i).l();
  //   TrajectoryPoint_Msg.frenet_valid=planning_output.trajectory().trajectory_points(i).frenet_valid();
  //   planningOutputMsg.trajectory.trajectory_points.push_back(TrajectoryPoint_Msg);
  // }

  // auto debug_info = mjson::Json(mjson::Json::object());

  // debug_info["fix_lane_a"]=debug_out_put.fix_lane.a;
  // debug_info["fix_lane_b"]=debug_out_put.fix_lane.b;
  // debug_info["fix_lane_c"]=debug_out_put.fix_lane.c;
  // debug_info["fix_lane_d"]=debug_out_put.fix_lane.d;
  // if (debug_out_put.target_lane.a!=0&&debug_out_put.target_lane.b!=0&&
  //     debug_out_put.target_lane.c!=0&&debug_out_put.target_lane.d!=0) {
  //   debug_info["target_lane_a"]=debug_out_put.target_lane.a;
  //   debug_info["target_lane_b"]=debug_out_put.target_lane.b;
  //   debug_info["target_lane_c"]=debug_out_put.target_lane.c;
  //   debug_info["target_lane_d"]=debug_out_put.target_lane.d;
  // }
  // planningOutputMsg.extra.available = 1;
  // planningOutputMsg.extra.json = debug_info.dump();

  // /*control输出*/
  // controlCommandMsg.timestamp_us=control_output.timestamp_us();
  // controlCommandMsg.lat_control_mode=control_output.lat_control_mode();
  // for(auto tr:control_output.mpc_trajectory()) {
  //   proto_msgs::MPCTrajectoryPoint trajectoryPointMsg;
  //   trajectoryPointMsg.mpc_out_x=tr.ref_x();
  //   trajectoryPointMsg.ref_y=tr.ref_y();
  //   trajectoryPointMsg.mpc_out_x=tr.mpc_out_x();
  //   trajectoryPointMsg.mpc_out_y=tr.mpc_out_y();
  //   controlCommandMsg.mpc_trajectory.push_back(trajectoryPointMsg);
  // }

  // pubEgoObj.publish(vehicleEgoMotionmsg);
  // pubFusionOutObj.publish(fusionOutMsg);
  // pubPlanningOutObj.publish(planningOutputMsg);
  // pubRadarSurroundOutObj.publish(radarSurroundInfoMsg);
  // pubControlObj.publish(controlCommandMsg);
// }