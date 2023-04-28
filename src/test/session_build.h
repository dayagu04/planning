
#include <array>
#include <cmath>

#include "gtest/gtest.h"
#include "src/modules/tasks/task_pipeline_context.h"
#include "src/common/log.h"
#include "src/modules/common/local_view.h"
#include "../../test/old_include/proto/EgoMotion.pb.h"
#include "../../test/old_include/proto/PerceptionOut.pb.h"
#include "../../test/old_include/proto/FusionOut.pb.h"
#include "../../test/old_include/proto/PlanningOutput.pb.h"
#include "../../test/old_include/proto/Radar.pb.h"
#include "../../test/old_include/proto/ControlCommand.pb.h"
using apollo::cyber::record::RecordReader;
using apollo::cyber::record::RecordMessage;
using apollo::cyber::Parameter;
using apollo::cyber::proto::Param;

namespace planning {
struct OldLocalView {
  FeiMa::FusionOut::FusionOut fusion_out;
  FeiMa::EgoMotion::VehicleEgoMotion ego_motion;
  J2::Radar::Radar_Surround_Info srnd_radar_info;

  // 待补充输入
  // FeiMa::SystemFunc::ModuleStatus module_status;
  // FeiMa::VehicleStatus vehicle_status;
};

void convertMsg(const OldLocalView &old_local_view, LocalView &local_view) {
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
  
  local_view.fusion_objects_info.clear_fusion_object();
  int num = 0;
  for (auto &fusion_object : old_local_view.fusion_out.fusion().objects()) {
    if (fusion_object.track_id() == 0) {
      continue;
    }
    num++;
    auto fusion_obj = local_view.fusion_objects_info.add_fusion_object();
    fusion_obj->mutable_additional_info()->set_track_id(fusion_object.track_id());
    printf("id:%d\n", fusion_object.track_id());
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
  local_view.fusion_objects_info.set_num(num);
}

planning::framework::Session* update() {
  static bool flag =true;
  static std::unique_ptr<planning::GeneralPlanning> planning_base;
  static planning::PlanningComponent planningComponent;
  const std::string channel_ego = "/ego_motion";
  const std::string channel_fusionout = "/fusion/fusion_object";
  const std::string channel_srnd_radar = "/surround_radar";
  const std::string channel_control = "/control";
  std::string bagPath;
  static std::shared_ptr<RecordReader> reader;
  if (flag) {
    int return_value=system("source /usr/bin/cyber/setup.bash");
    printf("result:%d",return_value);
    planningComponent.Init();
    planning_base = std::make_unique<planning::GeneralPlanning>();

    bagPath = "/home/ros/Downloads/bag/jichang_urban_2.00000"; ///home/root/shuxihu_0118/fangxingdadao_4.00000   /home/ros/Downloads/360_jichang_26.00000
    LOG_DEBUG("bag path is %s\n",bagPath.c_str());
    if (access(bagPath.c_str(), F_OK) != 0) {
      LOG_ERROR("%s is not exist\n",bagPath.c_str());
      std::cout<<bagPath<<" is not exist"<<std::endl;
      exit(0);
      // return false;
    }
    flag = false;
    reader = std::make_shared<RecordReader>(bagPath);
  }

  OldLocalView old_local_view;
  LocalView local_view;
   

  static RecordMessage message;
  static uint64_t reader_time;
  if (reader->ReadMessage(&message)) {
      reader_time = message.time + 50000000; // 50ms
  }

  apollo::cyber::Rate rate(15.0);
  FeiMa::FusionOut::FusionOut fusion_msg;
  FeiMa::EgoMotion::VehicleEgoMotion ego_motion_msg;
  J2::Radar::Radar_Surround_Info radar_surround_msg;
  FeiMa::control::ControlCommand control_msg;

  static uint32_t count;
  // while (apollo::cyber::OK()) //apollo::cyber::OK()
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
    // session = planning_base->MutableSession();
    rate.Sleep();
    return planning_base->MutableSession();
  }
  return 1;
}


}  // namespace planning