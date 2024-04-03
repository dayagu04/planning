#include "lane_keep_assist_manager.h"

#include "planning_context.h"
#include "vehicle_config_context.h"

static std::string ReadJsonFile(const std::string &path) {
  FILE *file = fopen(path.c_str(), "r");
  assert(file != nullptr);
  std::shared_ptr<FILE> fp(file, [](FILE *file) { fclose(file); });
  fseek(fp.get(), 0, SEEK_END);
  std::vector<char> content(ftell(fp.get()));
  fseek(fp.get(), 0, SEEK_SET);
  auto read_bytes = fread(content.data(), 1, content.size(), fp.get());
  assert(read_bytes == content.size());
  (void)read_bytes;
  return std::string(content.begin(), content.end());
}

namespace planning {
void LaneKeepAssistManager::SyncParameters() {
  // read json file
  std::string config_file =
      ReadJsonFile("/asw/planning/res/conf/adas_params.json");
  auto adas_config = mjson::Reader(config_file);

  // get params
  std::string json_car_type;
  ADAS_JSON_READ_VALUE(json_car_type, std::string, "car_type");
  double ldw_enable_speed = 0.0;
  ADAS_JSON_READ_VALUE(ldw_enable_speed, double, "ldw_enable_speed");
  std::cout << json_car_type << std::endl;

  ADAS_JSON_READ_VALUE(lkas_input_.param.ldp_tlc_thrd, double, "ldp_tlc_thrd");
  ADAS_JSON_READ_VALUE(lkas_input_.param.ldp_c0_right_offset, double,
                       "ldp_c0_right_offset");
  ADAS_JSON_READ_VALUE(lkas_input_.param.ldp_ttlc_right_hack, double,
                       "ldp_ttlc_right_hack");
  std::cout << lkas_input_.param.ldp_tlc_thrd << std::endl;
}

void LaneKeepAssistManager::Init(framework::Session *session) {
  // session init
  session_ = session;
  // ldw init
  lane_depart_warning_.Init(&lkas_input_);
  // ldp init
  lane_depart_prevention_.Init(&lkas_input_);
  // elk init
  emergency_lane_keep_.Init(&lkas_input_, session_);

  LOG_DEBUG("LaneKeepAssistManager::ldw_ldp_elk has been inited \n");
}
void LaneKeepAssistManager::Update() {
  // paramters init
  if (init_flag_ == false) {
    init_flag_ = true;
    SyncParameters();
  }
  uint8 left_turn_light_state_delay =
      lkas_input_.vehicle_info.left_turn_light_state;
  uint8 left_turn_light_state;
  if (session_->environmental_model().get_ego_state_manager()->ego_blinker() ==
      1) {
    left_turn_light_state = 1;
  } else {
    { left_turn_light_state = 0; }
  }
  if ((left_turn_light_state_delay == 0) && (left_turn_light_state == 1)) {
    SyncParameters();
  }

  // input update
  lkas_input_.road_info.left_line_existence = false;  // 本车道左侧道线存在性
  lkas_input_.road_info.left_line_valid =
      false;  // 本车道左侧道线有效性 0:Invalid 1:Valid
  lkas_input_.road_info.left_line_type =
      0;  // 本车道左侧车道线类型 0：虚线  1：实线
  lkas_input_.road_info.left_line_c0 = 0.0;  // 本车道左侧道线方程系数c0
  lkas_input_.road_info.left_line_c1 = 0.0;  // 本车道左侧道线方程系数c1
  lkas_input_.road_info.left_line_c2 = 0.0;  // 本车道左侧道线方程系数c3
  lkas_input_.road_info.left_line_c3 = 0.0;  // 本车道左侧道线方程系数c3
  lkas_input_.road_info.right_line_existence = false;  // 本车道右侧道线存在性
  lkas_input_.road_info.right_line_valid =
      false;  // 本车道右侧道线有效性 0:Invalid 1:Valid
  lkas_input_.road_info.right_line_type =
      0;  // 本车道右侧车道线类型 0：虚线  1：实线
  lkas_input_.road_info.right_line_c0 = 0.0;  // 本车道右侧道线方程系数c0
  lkas_input_.road_info.right_line_c1 = 0.0;  // 本车道右侧道线方程系数c1
  lkas_input_.road_info.right_line_c2 = 0.0;  // 本车道右侧道线方程系数c2
  lkas_input_.road_info.right_line_c3 = 0.0;  // 本车道右侧道线方程系数c3
  lkas_input_.road_info.left_roadedge_existence =
      false;  // 本车道左侧路缘存在性
  lkas_input_.road_info.left_roadedge_valid =
      false;  // 本车道左侧路缘有效性 0:Invalid 1:Valid
  lkas_input_.road_info.left_roadedge_c0 = 0.0;  // 本车道左侧路缘方程系数c0
  lkas_input_.road_info.left_roadedge_c1 = 0.0;  // 本车道左侧路缘方程系数c1
  lkas_input_.road_info.left_roadedge_c2 = 0.0;  // 本车道左侧路缘方程系数c3
  lkas_input_.road_info.left_roadedge_c3 = 0.0;  // 本车道左侧路缘方程系数c3
  lkas_input_.road_info.right_roadedge_existence =
      false;  // 本车道右侧路缘存在性
  lkas_input_.road_info.right_roadedge_valid =
      false;  // 本车道右侧路缘有效性 0:Invalid 1:Valid
  lkas_input_.road_info.right_roadedge_c0 = 0.0;  // 本车道右侧路缘方程系数c0
  lkas_input_.road_info.right_roadedge_c1 = 0.0;  // 本车道右侧路缘方程系数c1
  lkas_input_.road_info.right_roadedge_c2 = 0.0;  // 本车道右侧路缘方程系数c2
  lkas_input_.road_info.right_roadedge_c3 = 0.0;  // 本车道右侧路缘方程系数c3
  lkas_input_.road_info.lane_width_valid =
      false;  // 当前车道宽度信息有效性 0:Invalid 1:Valid
  lkas_input_.road_info.lane_width = 0.0;  // 当前车道宽度,单位:m

  // 车道线信息初始化
  // 当前车道地址
  // auto ptr_environmental_model = session_->mutable_environmental_model();
  //   std::shared_ptr<VirtualLane> ptr_current_lane = nullptr;
  auto ptr_virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  if (ptr_virtual_lane_manager != nullptr) {
    auto ptr_current_lane = ptr_virtual_lane_manager->get_current_lane();
    if (ptr_current_lane != nullptr) {
      // 左侧边界结构体
      auto ptr_current_lane_left_boundary =
          ptr_current_lane->get_left_lane_boundary();
      // 判断边界是路沿还是道线
      bool left_boundary_existence = ptr_current_lane_left_boundary.existence;
      /*
      LineType_LINE_TYPE_UNKNOWN = 0,
      LineType_LINE_TYPE_LANELINE = 1,//车道线
      LineType_LINE_TYPE_BORDERLINE = 2, 路沿，边沿线，物理隔离障碍物统称
      LineType_LINE_TYPE_CENTER = 3
       */
      iflyauto::LineType left_boundary_type =
          ptr_current_lane_left_boundary.type;
      left_boundary_type = iflyauto::LINE_TYPE_LANELINE;

      if ((left_boundary_type == iflyauto::LineType::LINE_TYPE_LANELINE) &&
          left_boundary_existence == true) {
        lkas_input_.road_info.left_line_existence = true;
      } else {
        lkas_input_.road_info.left_line_existence = false;
      }

      if (left_boundary_type == iflyauto::LINE_TYPE_BORDERLINE &&
          left_boundary_existence == true) {
        lkas_input_.road_info.left_roadedge_existence = true;
      } else {
        lkas_input_.road_info.left_roadedge_existence = false;
      }

      if (lkas_input_.road_info.left_line_existence == true)  // 判断为道线
      {
        lkas_input_.road_info.left_line_valid = true;
        // LaneBoundaryType_MARKING_UNKNOWN = 0, /* 未知线型 */
        // LaneBoundaryType_MARKING_DASHED = 1, /* 虚线 */
        // LaneBoundaryType_MARKING_SOLID = 2, /* 实线 */
        // LaneBoundaryType_MARKING_SHORT_DASHED = 3, /* 短虚线 */
        // LaneBoundaryType_MARKING_DOUBLE_DASHED = 4, /* 双虚线 */
        // LaneBoundaryType_MARKING_DOUBLE_SOLID = 5, /* 双实线 */
        // LaneBoundaryType_MARKING_LEFT_DASHED_RIGHT_SOLID = 6, /* 左虚右实线*/
        // LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED = 7 /* 左实右虚线*/

        auto left_line_type =
            ptr_current_lane_left_boundary.type_segments[0].type;
        if (left_line_type == Common::LaneBoundaryType::MARKING_DASHED ||
            left_line_type == Common::LaneBoundaryType::MARKING_SHORT_DASHED ||
            left_line_type == Common::LaneBoundaryType::MARKING_DOUBLE_DASHED ||
            left_line_type ==
                Common::LaneBoundaryType::MARKING_LEFT_SOLID_RIGHT_DASHED) {
          lkas_input_.road_info.left_line_type = 0;  // 可跨越道线
        } else {
          lkas_input_.road_info.left_line_type = 1;  // 不可跨越道线
        }

        lkas_input_.road_info.left_line_c0 =
            ptr_current_lane_left_boundary.poly_coefficient[0];
        lkas_input_.road_info.left_line_c1 =
            ptr_current_lane_left_boundary.poly_coefficient[1];
        lkas_input_.road_info.left_line_c2 =
            ptr_current_lane_left_boundary.poly_coefficient[2];
        lkas_input_.road_info.left_line_c3 =
            ptr_current_lane_left_boundary.poly_coefficient[3];

      } else if (lkas_input_.road_info.left_roadedge_existence ==
                 true)  // 判断为路沿
      {
        lkas_input_.road_info.left_roadedge_valid = true;

        lkas_input_.road_info.left_roadedge_c0 =
            ptr_current_lane_left_boundary.poly_coefficient[0];
        lkas_input_.road_info.left_roadedge_c1 =
            ptr_current_lane_left_boundary.poly_coefficient[1];
        lkas_input_.road_info.left_roadedge_c2 =
            ptr_current_lane_left_boundary.poly_coefficient[2];
        lkas_input_.road_info.left_roadedge_c3 =
            ptr_current_lane_left_boundary.poly_coefficient[3];

      } else {
        lkas_input_.road_info.left_line_valid = false;
        lkas_input_.road_info.left_roadedge_valid = false;
      }
      // 右侧边界
      auto ptr_current_lane_right_boundary =
          ptr_current_lane->get_right_lane_boundary();
      bool right_boundary_existence = ptr_current_lane_right_boundary.existence;
      /*
      LineType_LINE_TYPE_UNKNOWN = 0,
      LineType_LINE_TYPE_LANELINE = 1,//车道线
      LineType_LINE_TYPE_BORDERLINE = 2, 路沿，边沿线，物理隔离障碍物统称
      LineType_LINE_TYPE_CENTER = 3
       */
      iflyauto::LineType right_boundary_type =
          ptr_current_lane_right_boundary.type;
      right_boundary_type = iflyauto::LINE_TYPE_LANELINE;

      if ((right_boundary_type == iflyauto::LINE_TYPE_LANELINE) &&
          right_boundary_existence == true) {
        lkas_input_.road_info.right_line_existence = true;
      } else {
        lkas_input_.road_info.right_line_existence = false;
      }

      if (right_boundary_type == iflyauto::LINE_TYPE_BORDERLINE &&
          right_boundary_existence == true) {
        lkas_input_.road_info.right_roadedge_existence = true;
      } else {
        lkas_input_.road_info.right_roadedge_existence = false;
      }

      if (lkas_input_.road_info.right_line_existence == true) {
        lkas_input_.road_info.right_line_valid = true;
        // LaneBoundaryType_MARKING_UNKNOWN = 0, /* 未知线型 */
        // LaneBoundaryType_MARKING_DASHED = 1, /* 虚线 */
        // LaneBoundaryType_MARKING_SOLID = 2, /* 实线 */
        // LaneBoundaryType_MARKING_SHORT_DASHED = 3, /* 短虚线 */
        // LaneBoundaryType_MARKING_DOUBLE_DASHED = 4, /* 双虚线 */
        // LaneBoundaryType_MARKING_DOUBLE_SOLID = 5, /* 双实线 */
        // LaneBoundaryType_MARKING_LEFT_DASHED_RIGHT_SOLID = 6, /* 左虚右实线*/
        // LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED = 7 /* 左实右虚线*/

        auto right_line_type =
            ptr_current_lane_right_boundary.type_segments[0].type;
        if ((right_line_type == Common::LaneBoundaryType::MARKING_DASHED) ||
            (right_line_type ==
             Common::LaneBoundaryType::MARKING_SHORT_DASHED) ||
            (right_line_type ==
             Common::LaneBoundaryType::MARKING_DOUBLE_DASHED) ||
            (right_line_type ==
             Common::LaneBoundaryType::MARKING_LEFT_DASHED_RIGHT_SOLID)) {
          lkas_input_.road_info.right_line_type = 0;
        } else {
          lkas_input_.road_info.right_line_type = 1;
        }

        lkas_input_.road_info.right_line_c0 =
            ptr_current_lane_right_boundary.poly_coefficient[0] +
            lkas_input_.param.ldp_c0_right_offset;
        lkas_input_.road_info.right_line_c1 =
            ptr_current_lane_right_boundary.poly_coefficient[1];
        lkas_input_.road_info.right_line_c2 =
            ptr_current_lane_right_boundary.poly_coefficient[2];
        lkas_input_.road_info.right_line_c3 =
            ptr_current_lane_right_boundary.poly_coefficient[3];

      } else if (lkas_input_.road_info.right_roadedge_existence == true) {
        lkas_input_.road_info.right_roadedge_valid = true;

        lkas_input_.road_info.right_roadedge_c0 =
            ptr_current_lane_right_boundary.poly_coefficient[0] +
            lkas_input_.param.ldp_c0_right_offset;
        lkas_input_.road_info.right_roadedge_c1 =
            ptr_current_lane_right_boundary.poly_coefficient[1];
        lkas_input_.road_info.right_roadedge_c2 =
            ptr_current_lane_right_boundary.poly_coefficient[2];
        lkas_input_.road_info.right_roadedge_c3 =
            ptr_current_lane_right_boundary.poly_coefficient[3];

      } else {
        lkas_input_.road_info.right_line_valid = false;
        lkas_input_.road_info.right_roadedge_valid = false;
      }
      // 道路宽度
      lkas_input_.road_info.lane_width_valid = true;
      lkas_input_.road_info.lane_width = ptr_current_lane->width();

      // update
      CalculateWheelToLine();
    } else {  // ptr_current_lane == nullptr
      /*do nothing*/
    }
  } else {  // ptr_virtual_lane_manager == nullptr
    /*do nothing*/
  }

  // 车辆信息初始化、参数初始化
  lkas_input_.function_state = session_->environmental_model()
                                   .get_local_view()
                                   .function_state_machine_info.current_state;
  auto ptr_ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  lkas_input_.vehicle_info.veh_actual_speed = ptr_ego_state_manager->ego_v();
  lkas_input_.vehicle_info.veh_display_speed =
      ptr_ego_state_manager->ego_hmi_v();
  lkas_input_.vehicle_info.veh_yaw_rate = ptr_ego_state_manager->ego_yaw_rate();
  lkas_input_.vehicle_info.driver_hand_torque =
      ptr_ego_state_manager->driver_hand_torque();
  uint8 turn_light_flag = ptr_ego_state_manager->ego_blinker();

  // TURN_SIGNAL_TYPE_NONE = 0,              // 无请求
  // TURN_SIGNAL_TYPE_LEFT = 1,              // 请求左转向
  // TURN_SIGNAL_TYPE_RIGHT = 2,             // 请求右转向
  // TURN_SIGNAL_TYPE_EMERGENCY_FLASH = 3,   // 双闪
  // 左转向灯开启状态
  if (turn_light_flag == 1) {
    lkas_input_.vehicle_info.left_turn_light_state = 1;
  } else {
    lkas_input_.vehicle_info.left_turn_light_state = 0;
  }
  // 右转向灯开启状态
  if (turn_light_flag == 2) {
    lkas_input_.vehicle_info.right_turn_light_state = 1;
  } else {
    lkas_input_.vehicle_info.right_turn_light_state = 0;
  }
  // ldw switch
  lkas_input_.vehicle_info.ldw_main_switch =
      session_->mutable_environmental_model()->get_hmi_info().ldw_main_switch;
  // ldp switch
  lkas_input_.vehicle_info.ldp_main_switch =
      session_->mutable_environmental_model()->get_hmi_info().ldp_main_switch;
  // elk switch
  lkas_input_.vehicle_info.elk_main_switch =
      session_->mutable_environmental_model()->get_hmi_info().elk_main_switch;
  // ldw sensitvity set
  lkas_input_.vehicle_info.ldw_tlc_level =
      session_->mutable_environmental_model()
          ->get_hmi_info()
          .ldw_set_sensitivity_level;

  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  lkas_input_.vehicle_info.common_front_over =
      vehicle_param.front_edge_to_center - vehicle_param.wheel_base;
  lkas_input_.vehicle_info.common_rear_over = vehicle_param.back_edge_to_center;
  lkas_input_.vehicle_info.common_wheel_base = vehicle_param.wheel_base;
  lkas_input_.vehicle_info.common_veh_width = vehicle_param.width;

  // 计算偏离速度
  if (lkas_input_.road_info.left_line_valid) {
    lkas_input_.vehicle_info.veh_left_departure_speed =
        lkas_input_.road_info.left_line_c1 *
        lkas_input_.vehicle_info.veh_actual_speed;
  } else if (lkas_input_.road_info.left_roadedge_valid) {
    lkas_input_.vehicle_info.veh_left_departure_speed =
        lkas_input_.road_info.left_roadedge_c1 *
        lkas_input_.vehicle_info.veh_actual_speed;
  } else {
    lkas_input_.vehicle_info.veh_left_departure_speed = 0.0;
  }

  if (lkas_input_.road_info.right_line_valid) {
    lkas_input_.vehicle_info.veh_right_departure_speed =
        lkas_input_.road_info.right_line_c1 *
        lkas_input_.vehicle_info.veh_actual_speed;
  } else if (lkas_input_.road_info.right_roadedge_valid) {
    lkas_input_.vehicle_info.veh_right_departure_speed =
        lkas_input_.road_info.right_roadedge_c1 *
        lkas_input_.vehicle_info.veh_actual_speed;
  } else {
    lkas_input_.vehicle_info.veh_right_departure_speed = 0.0;
  }

  JSON_DEBUG_VALUE("lkas_function::left_line_c0",
                   lkas_input_.road_info.left_line_c0);
  JSON_DEBUG_VALUE("lkas_function::left_line_c1",
                   lkas_input_.road_info.left_line_c1);
  JSON_DEBUG_VALUE("lkas_function::left_line_c2",
                   lkas_input_.road_info.left_line_c2);
  JSON_DEBUG_VALUE("lkas_function::left_line_c3",
                   lkas_input_.road_info.left_line_c3);
  JSON_DEBUG_VALUE("lkas_function::right_line_c0",
                   lkas_input_.road_info.right_line_c0);
  JSON_DEBUG_VALUE("lkas_function::right_line_c1",
                   lkas_input_.road_info.right_line_c1);
  JSON_DEBUG_VALUE("lkas_function::right_line_c2",
                   lkas_input_.road_info.right_line_c2);
  JSON_DEBUG_VALUE("lkas_function::right_line_c3",
                   lkas_input_.road_info.right_line_c3);
  JSON_DEBUG_VALUE("lkas_veh_yaw_rate", lkas_input_.vehicle_info.veh_yaw_rate);
  JSON_DEBUG_VALUE("lkas_veh_actual_speed",
                   lkas_input_.vehicle_info.veh_actual_speed);
  JSON_DEBUG_VALUE("lkas_left_departure_speed",
                   lkas_input_.vehicle_info.veh_left_departure_speed);
  JSON_DEBUG_VALUE("lkas_right_departure_speed",
                   lkas_input_.vehicle_info.veh_right_departure_speed);
}
void LaneKeepAssistManager::RunOnce() {
  Update();
  lane_depart_warning_.RunOnce();
  lane_depart_prevention_.RunOnce();
  emergency_lane_keep_.RunOnce();
  set_lka_output_info();

  LOG_DEBUG("LaneKeepAssistManager::RunOnce \n");
  LOG_DEBUG("LaneKeepAssistManager::ldw_state = %d \n", ldw_state_);
  LOG_DEBUG("LaneKeepAssistManager::ldp_state = %d \n", ldp_state_);
  LOG_DEBUG("LaneKeepAssistManager::elk_state = %d \n", elk_state_);
}
void LaneKeepAssistManager::CalculateWheelToLine() {
  /*fl_wheel_distance_to_line*/
  lkas_input_.wheel_to_line.fl_wheel_distance_to_line =
      lkas_input_.road_info.left_line_c0 +
      lkas_input_.road_info.left_line_c1 *
          lkas_input_.vehicle_info.common_wheel_base +
      lkas_input_.road_info.left_line_c2 *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base +
      lkas_input_.road_info.left_line_c3 *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base -
      Common_FrontCamera_PosYL;
  /*fl_wheel_distance_to_roadedge*/
  lkas_input_.wheel_to_line.fl_wheel_distance_to_roadedge =
      lkas_input_.road_info.left_roadedge_c0 +
      lkas_input_.road_info.left_roadedge_c1 *
          lkas_input_.vehicle_info.common_wheel_base +
      lkas_input_.road_info.left_roadedge_c2 *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base +
      lkas_input_.road_info.left_roadedge_c3 *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base -
      Common_FrontCamera_PosYL;
  /*fr_wheel_distance_to_line*/
  lkas_input_.wheel_to_line.fr_wheel_distance_to_line =
      lkas_input_.road_info.right_line_c0 +
      lkas_input_.road_info.right_line_c1 *
          lkas_input_.vehicle_info.common_wheel_base +
      lkas_input_.road_info.right_line_c2 *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base +
      lkas_input_.road_info.right_line_c3 *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base +
      Common_FrontCamera_PosYR;
  /*fr_wheel_distance_to_roadedge*/
  lkas_input_.wheel_to_line.fr_wheel_distance_to_roadedge =
      lkas_input_.road_info.right_roadedge_c0 +
      lkas_input_.road_info.right_roadedge_c1 *
          lkas_input_.vehicle_info.common_wheel_base +
      lkas_input_.road_info.right_roadedge_c2 *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base +
      lkas_input_.road_info.right_roadedge_c3 *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base +
      Common_FrontCamera_PosYR;
}
void LaneKeepAssistManager::Output() {
  auto lka_output_str =
      session_->mutable_planning_context()->mutable_planning_hmi_info();
  /*
  0:Unavailable
  1:Off
  2:Standby
  3:Active(No Intervention)
  4:Active(Left Intervention)
  5:Active(Right Intervention)
  */
}

void LaneKeepAssistManager::set_lka_output_info() {
  /*for ldw output info*/
  ldw_state_ = lane_depart_warning_.get_ldw_state();
  ldw_left_warning_ = lane_depart_warning_.get_ldw_left_warning_info();
  ldw_right_warning_ = lane_depart_warning_.get_ldw_right_warning_info();
  /*for ldp output info*/
  ldp_state_ = lane_depart_prevention_.get_ldp_state();
  ldp_left_intervention_flag_ =
      lane_depart_prevention_.get_ldp_left_intervention_flag_info();
  ldp_right_intervention_flag_ =
      lane_depart_prevention_.get_ldp_right_intervention_flag_info();
  /*for elk output info*/
  elk_state_ = emergency_lane_keep_.get_elk_state();
  elk_left_intervention_flag_ =
      emergency_lane_keep_.get_elk_left_intervention_flag_info();
  elk_right_intervention_flag_ =
      emergency_lane_keep_.get_elk_right_intervention_flag_info();
}

}  // namespace planning