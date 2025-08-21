#include "adas_function_preprocess.h"

#include "common_c.h"
#include "common_platform_type_soc.h"

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

namespace adas_function {
namespace preprocess {

void Preprocess::Init(void) { SyncParameters(); }

void Preprocess::SyncParameters(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto engine_config =
      planning::common::ConfigurationContext::Instance()->engine_config();
  std::string path = engine_config.vehicle_cfg_dir + "/adas_params.json";
  // read json file
  std::string config_file = ReadJsonFile(path);
  auto adas_config = mjson::Reader(config_file);

  // get adas_function运行参数
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->dt, double, "dt");

  // get 车辆参数
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->car_type, std::string,
                       "car_type");
  ILOG_DEBUG << "car_type = " << GetContext.mutable_param()->car_type;
  mjson::Json json_vehicle_common_params =
      adas_config.get<mjson::Json>("jac_s811");
  std::string json_car_type = adas_config.get<std::string>("car_type");
  if ((int)json_car_type.find("jac_s811") != -1) {
    json_vehicle_common_params = adas_config.get<mjson::Json>("jac_s811");
  } else if ((int)json_car_type.find("chery_e0x") != -1) {
    json_vehicle_common_params = adas_config.get<mjson::Json>("chery_e0x");
  } else if ((int)json_car_type.find("chery_m32t") != -1) {
    json_vehicle_common_params = adas_config.get<mjson::Json>("chery_m32t");
  }else{
    json_vehicle_common_params = adas_config.get<mjson::Json>("chery_e0x");
  }
  auto vehicle_common_json_reader = mjson::Reader(json_vehicle_common_params);

  VEHICLE_COMMON_JSON_READ_VALUE(GetContext.mutable_param()->wheel_base, double,
                                 "wheel_base");
  VEHICLE_COMMON_JSON_READ_VALUE(GetContext.mutable_param()->steer_ratio,
                                 double, "steer_ratio");
  VEHICLE_COMMON_JSON_READ_VALUE(GetContext.mutable_param()->ego_length, double,
                                 "ego_length");
  VEHICLE_COMMON_JSON_READ_VALUE(GetContext.mutable_param()->ego_width, double,
                                 "ego_width");
  VEHICLE_COMMON_JSON_READ_VALUE(
      GetContext.mutable_param()->origin_2_front_bumper, double,
      "origin_2_front_bumper");
  VEHICLE_COMMON_JSON_READ_VALUE(
      GetContext.mutable_param()->origin_2_rear_bumper, double,
      "origin_2_rear_bumper");
  ILOG_DEBUG << "GetContext.mutable_param()->wheel_base = "
             << GetContext.mutable_param()->wheel_base;
  // get LKAS_Function 参数
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->lon_distance_buffer0, double,
                       "lon_distance_buffer0");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->lon_distance_buffer1, double,
                       "lon_distance_buffer1");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->lat_buffer_to_line, double,
                       "lat_buffer_to_line");
  ADAS_JSON_READ_VALUE(
      GetContext.mutable_param()->lane_boundary_vaild_length_set, double,
      "lane_boundary_vaild_length_set");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->ldw_enable_speed, double,
                       "ldw_enable_speed");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->ldw_tlc_thrd, double,
                       "ldw_tlc_thrd");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->ldp_tlc_thrd, double,
                       "ldp_tlc_thrd");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->ldp_roadedge_tlc_thrd,
                       double, "ldp_roadedge_tlc_thrd");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->ldp_roadedge_offset, double,
                       "ldp_roadedge_offset");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->ldp_roadedge_distance_limit,
                       double, "ldp_roadedge_distance_limit");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->sideway_exist_gap_thrd,
                       double, "sideway_exist_gap_thrd");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->safe_departure_ttc, double,
                       "safe_departure_ttc");

  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->ldp_c0_right_offset, double,
                       "ldp_c0_right_offset");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->ldp_ttlc_right_hack, double,
                       "ldp_ttlc_right_hack");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->ldp_center_line_offset,
                       double, "ldp_center_line_offset");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->ldp_center_roadedge_offset,
                       double, "ldp_center_roadedge_offset");

  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->ldw_main_switch, bool,
                       "ldw_main_switch");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->ldp_main_switch, bool,
                       "ldp_main_switch");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->elk_main_switch, bool,
                       "elk_main_switch");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->tsr_main_switch, bool,
                       "tsr_main_switch");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->elk_tlc_thrd, double,
                       "elk_tlc_thrd");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->elk_roadedge_tlc_thrd,
                       double, "elk_roadedge_tlc_thrd");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->elk_roadedge_offset, double,
                       "elk_roadedge_offset");

  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->force_no_sideway_switch,
                       bool, "force_no_sideway_switch");
  ADAS_JSON_READ_VALUE(
      GetContext.mutable_param()->force_no_safe_departure_switch, bool,
      "force_no_safe_departure_switch");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->adas_sim_switch, bool,
                       "adas_sim_switch");

  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->hmi_test_switch, bool,
                       "hmi_test_switch");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->hmi_ldw_state, int,
                       "hmi_ldw_state");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->hmi_ldp_state, int,
                       "hmi_ldp_state");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->hmi_elk_state, int,
                       "hmi_elk_state");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->hmi_tsr_state, int,
                       "hmi_tsr_state");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->hmi_tsr_speed_limit, int,
                       "hmi_tsr_speed_limit");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->tsr_reset_path_length,
                       double, "tsr_reset_path_length");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->lane_line_width, double,
                       "lane_line_width");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->ldp_vel_vector,
                       std::vector<double>, "ldp_vel_vector");
  ADAS_JSON_READ_VALUE(GetContext.mutable_param()->ldp_tlc_vector,
                       std::vector<double>, "ldp_tlc_vector");
  // SetEgoAroundAreaRange();
  ILOG_DEBUG << "SyncParameters() is run over!!";
}

void Preprocess::UpdateStateInfo(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;
  GetContext.mutable_state_info()->ctrl_output_steering_angle =
      GetContext.mutable_session()
          ->environmental_model()
          .get_local_view()
          .control_output.steering;
  // vehicle_speed
  GetContext.mutable_state_info()->vehicle_speed =
      vehicle_service_output_info_ptr->vehicle_speed;
  GetContext.mutable_state_info()->display_vehicle_speed =
      vehicle_service_output_info_ptr->vehicle_speed_display;
  // enu2car 变换矩阵
  Eigen::Vector2d current_pos_i(GetContext.mutable_session()
                                    ->environmental_model()
                                    .get_local_view()
                                    .localization.position.position_boot.x,
                                GetContext.mutable_session()
                                    ->environmental_model()
                                    .get_local_view()
                                    .localization.position.position_boot.y);
  GetContext.mutable_state_info()->current_pos_i = current_pos_i;
  GetContext.mutable_state_info()->rotm2d = pnc::transform::Angle2Rotm2d(
      GetContext.mutable_session()
          ->environmental_model()
          .get_local_view()
          .localization.orientation.euler_boot.yaw);
  // yaw_rate
  GetContext.mutable_state_info()->yaw_rate =
      vehicle_service_output_info_ptr->yaw_rate;
  // yaw_rate_observer
  double front_wheel_angle =
      vehicle_service_output_info_ptr->steering_wheel_angle /
      GetContext.get_param()->steer_ratio;
  GetContext.mutable_state_info()->yaw_rate_observer =
      GetContext.get_state_info()->vehicle_speed * std::tan(front_wheel_angle) /
      GetContext.get_param()->wheel_base;
  double yaw_rad_current = GetContext.mutable_session()
                               ->mutable_environmental_model()
                               ->get_local_view()
                               .localization_estimate.pose.euler_angles.yaw;
  double yaw_rad_last = GetContext.get_last_cycle_info()->yaw_rad;
  double yaw_rate_loc = (yaw_rad_current - yaw_rad_last) * 10.0;
  GetContext.mutable_state_info()->yaw_rate_loc = yaw_rate_loc;

  // ego_curvature
  if (abs(GetContext.mutable_state_info()->vehicle_speed) < 0.5) {
    GetContext.mutable_state_info()->ego_curvature = 0.0;
  } else {
    GetContext.mutable_state_info()->ego_curvature =
        GetContext.mutable_state_info()->yaw_rate /
        GetContext.mutable_state_info()->vehicle_speed;
  }

  // left_turn_light_off_time
  if (vehicle_service_output_info_ptr->left_turn_light_state == true) {
    // 转向灯开启,转向灯关闭状态计时器清零
    GetContext.mutable_state_info()->left_turn_light_off_time = 0.0;
  } else {
    GetContext.mutable_state_info()->left_turn_light_off_time +=
        GetContext.mutable_param()->dt;
    if (GetContext.mutable_state_info()->left_turn_light_off_time >= 60.0) {
      GetContext.mutable_state_info()->left_turn_light_off_time = 60.0;
    } else {
      /*do nothing*/
    }
  }

  // right_turn_light_off_time
  if (vehicle_service_output_info_ptr->right_turn_light_state == true) {
    // 转向灯开启,转向灯关闭状态计时器清零
    GetContext.mutable_state_info()->right_turn_light_off_time = 0.0;
  } else {
    GetContext.mutable_state_info()->right_turn_light_off_time +=
        GetContext.mutable_param()->dt;
    if (GetContext.mutable_state_info()->right_turn_light_off_time >= 60.0) {
      GetContext.mutable_state_info()->right_turn_light_off_time = 60.0;
    } else {
      /*do nothing*/
    }
  }
  // acc
  if (vehicle_service_output_info_ptr->accelerator_pedal_pos_available) {
    GetContext.mutable_state_info()->accelerator_pedal_pos =
        vehicle_service_output_info_ptr->accelerator_pedal_pos;
  } else {
    GetContext.mutable_state_info()->accelerator_pedal_pos = 0.0;
  }
  // brake_pedal_pos
  if (vehicle_service_output_info_ptr->brake_pedal_pos_available) {
    GetContext.mutable_state_info()->brake_pedal_pos =
        vehicle_service_output_info_ptr->brake_pedal_pos;
  } else {
    GetContext.mutable_state_info()->brake_pedal_pos = 0.0;
  }
  if (vehicle_service_output_info_ptr->brake_pedal_pressed_available) {
    GetContext.mutable_state_info()->brake_pedal_pressed =
        vehicle_service_output_info_ptr->brake_pedal_pressed;
  } else {
    GetContext.mutable_state_info()->brake_pedal_pressed = false;
  }

  if (vehicle_service_output_info_ptr->lat_acceleration_available) {
    GetContext.mutable_state_info()->lat_departure_acc =
        vehicle_service_output_info_ptr->lat_acceleration;
  } else {
    GetContext.mutable_state_info()->lat_departure_acc = 0.0;
  }
  /*计算车辆左前、右前两个角点至车道线合路沿的距离*/
  // 公共部分：左前角点/右前角点坐标：
  // 计算tlc秒后,后轴中心的坐标值、计算角点位置
  double x_0 = 0.0;
  double y_0 = 0.0;
  double yaw_0 = 0.0;
  double dt = 0.0;
  double v = GetContext.mutable_state_info()->vehicle_speed;
  double yaw_rate = GetContext.mutable_state_info()->yaw_rate;
  double ego_x = x_0 + (dt * (v * cos(yaw_0 + yaw_rate * dt) +
                              4.0 * v * cos(yaw_0 + (yaw_rate * dt) / 2.0) +
                              v * cos(yaw_0))) /
                           6.0;
  double ego_y = y_0 + (dt * (v * sin(yaw_0 + yaw_rate * dt) +
                              4.0 * v * sin(yaw_0 + (yaw_rate * dt) / 2.0) +
                              v * sin(yaw_0))) /
                           6.0;

  adas_function::Box2DStr ego_box;  // 本车的二维矩形结构体定义
  ego_box.input.heading_angle = yaw_rate * dt;
  ego_box.input.sin_heading_angle = sin(ego_box.input.heading_angle);
  ego_box.input.cos_heading_angle = cos(ego_box.input.heading_angle);
  ego_box.input.length = GetContext.mutable_param()->ego_length;
  ego_box.input.width = GetContext.mutable_param()->ego_width;
  ego_box.input.x =
      ego_x + (GetContext.mutable_param()->origin_2_front_bumper -
               0.5F * ego_box.input.length) *
                  ego_box.input.cos_heading_angle;  // box几何中心的x坐标
  ego_box.input.y =
      ego_y + (GetContext.mutable_param()->origin_2_front_bumper -
               0.5F * ego_box.input.length) *
                  ego_box.input.sin_heading_angle;  // box几何中心的y坐标
  adas_function::BoxCornersCoordinateUpdate(&ego_box);
  // 左前
  double preview_distance_fl_x = ego_box.state.fl_x;
  double preview_distance_fl_y = ego_box.state.fl_y;
  // 右前
  double preview_distance_fr_x = ego_box.state.fr_x;
  double preview_distance_fr_y = ego_box.state.fr_y;
  // fl_wheel_distance_to_line
  if (GetContext.mutable_road_info()->current_lane.left_line.valid == true) {
    double line_width = 0.15;  // 道线的宽度,单位:m
    // 计算前轮处道线的横向坐标值
    Eigen::Vector2d pos_proj;
    Eigen::Vector2d current_pos = Eigen::Vector2d::Zero();
    current_pos.x() = preview_distance_fl_x;
    current_pos.y() = preview_distance_fl_y;
    double s_proj = 0.0;
    CalProjectionPointByNewtonIteration(
        GetContext.mutable_road_info()->current_lane.left_line.dx_s_spline_,
        GetContext.mutable_road_info()->current_lane.left_line.dy_s_spline_,
        0.0, GetContext.mutable_road_info()->current_lane.left_line.end_s,
        current_pos, s_proj);
    GetContext.mutable_state_info()->fl_wheel_distance_to_line =
        GetContext.mutable_road_info()->current_lane.left_line.dy_s_spline_(
            s_proj) -
        line_width * 0.5 - 0.5 * GetContext.mutable_param()->ego_width;
  } else {
    GetContext.mutable_state_info()->fl_wheel_distance_to_line = 66.66;
  }

  // fr_wheel_distance_to_line
  if (GetContext.mutable_road_info()->current_lane.right_line.valid) {
    double line_width = 0.15;  // 道线的宽度,单位:m
    // 计算前轮处道线的横向坐标值
    Eigen::Vector2d pos_proj;
    Eigen::Vector2d current_pos = Eigen::Vector2d::Zero();
    current_pos.x() = preview_distance_fr_x;
    current_pos.y() = preview_distance_fr_y;
    double s_proj = 0.0;
    CalProjectionPointByNewtonIteration(
        GetContext.mutable_road_info()->current_lane.right_line.dx_s_spline_,
        GetContext.mutable_road_info()->current_lane.right_line.dy_s_spline_,
        0.0, GetContext.mutable_road_info()->current_lane.right_line.end_s,
        current_pos, s_proj);
    GetContext.mutable_state_info()->fr_wheel_distance_to_line =
        GetContext.mutable_road_info()->current_lane.right_line.dy_s_spline_(
            s_proj) +
        line_width * 0.5 + 0.5 * GetContext.mutable_param()->ego_width;
  } else {
    GetContext.mutable_state_info()->fr_wheel_distance_to_line = -66.66;
  }

  // fl_wheel_distance_to_roadedge
  if (GetContext.get_road_info()->current_lane.left_roadedge.valid == true) {
    // 计算tlc秒后,后轴中心的坐标值
    double line_width = 0.0;  // 道线的宽度,单位:m
    // 计算前轮处道线的横向坐标值
    Eigen::Vector2d pos_proj;
    Eigen::Vector2d current_pos = Eigen::Vector2d::Zero();
    current_pos.x() = preview_distance_fl_x;
    current_pos.y() = preview_distance_fl_y;
    double s_proj = 0.0;
    CalProjectionPointByNewtonIteration(
        GetContext.mutable_road_info()->current_lane.left_roadedge.dx_s_spline_,
        GetContext.mutable_road_info()->current_lane.left_roadedge.dy_s_spline_,
        0.0, GetContext.mutable_road_info()->current_lane.left_roadedge.end_s,
        current_pos, s_proj);
    GetContext.mutable_state_info()->fl_wheel_distance_to_roadedge =
        GetContext.mutable_road_info()->current_lane.left_roadedge.dy_s_spline_(
            s_proj) -
        line_width * 0.5 - 0.5 * GetContext.mutable_param()->ego_width;
  } else {
    GetContext.mutable_state_info()->fl_wheel_distance_to_roadedge = 66.66;
  }
  // fr_wheel_distance_to_roadedge
  if (GetContext.mutable_road_info()->current_lane.right_roadedge.valid ==
      true) {                 // 计算tlc秒后,后轴中心的坐标值
    double line_width = 0.0;  // 道线的宽度,单位:m
    // 计算前轮处道线的横向坐标值
    Eigen::Vector2d pos_proj;
    Eigen::Vector2d current_pos = Eigen::Vector2d::Zero();
    current_pos.x() = preview_distance_fr_x;
    current_pos.y() = preview_distance_fr_y;
    double s_proj = 0.0;
    CalProjectionPointByNewtonIteration(
        GetContext.mutable_road_info()
            ->current_lane.right_roadedge.dx_s_spline_,
        GetContext.mutable_road_info()
            ->current_lane.right_roadedge.dy_s_spline_,
        0.0, GetContext.mutable_road_info()->current_lane.right_roadedge.end_s,
        current_pos, s_proj);
    GetContext.mutable_state_info()->fr_wheel_distance_to_roadedge =
        GetContext.mutable_road_info()
            ->current_lane.right_roadedge.dy_s_spline_(s_proj) +
        line_width * 0.5 + 0.5 * GetContext.mutable_param()->ego_width;
  } else {
    GetContext.mutable_state_info()->fr_wheel_distance_to_roadedge = -66.66;
  }
  double left_distance_to_car = 0.0;
  double right_distance_to_car = 0.0;
  GetContext.mutable_road_info()->current_lane.lane_width_valid = true;
  if (GetContext.get_road_info()->current_lane.left_line.valid) {
    left_distance_to_car =
        GetContext.get_state_info()->fl_wheel_distance_to_line;
  } else if (GetContext.mutable_road_info()->current_lane.left_roadedge.valid) {
    left_distance_to_car =
        GetContext.get_state_info()->fl_wheel_distance_to_roadedge;
  } else {
    GetContext.mutable_road_info()->current_lane.lane_width_valid = false;
    GetContext.mutable_road_info()->current_lane.lane_width = 10.0;
  }

  if (GetContext.get_road_info()->current_lane.right_line.valid) {
    right_distance_to_car =
        GetContext.get_state_info()->fr_wheel_distance_to_line;
  } else if (GetContext.get_road_info()->current_lane.right_roadedge.valid) {
    right_distance_to_car =
        GetContext.get_state_info()->fr_wheel_distance_to_roadedge;
  } else {
    GetContext.mutable_road_info()->current_lane.lane_width_valid = false;
    GetContext.mutable_road_info()->current_lane.lane_width = 10.0;
  }
  GetContext.mutable_road_info()->current_lane.lane_width =
      left_distance_to_car + -1.0 * right_distance_to_car +
      GetContext.get_param()->ego_width;
  // 驾驶员手力矩
  GetContext.mutable_state_info()->driver_hand_trq =
      vehicle_service_output_info_ptr->driver_hand_torque;

  // 车辆偏离车道线速度
  GetContext.mutable_state_info()->veh_left_departure_speed =
      -1.0 * GetContext.get_state_info()->vehicle_speed *
      GetContext.get_road_info()->current_lane.left_line.c1;
  GetContext.mutable_state_info()->veh_right_departure_speed =
      -1.0 * GetContext.get_state_info()->vehicle_speed *
      GetContext.get_road_info()->current_lane.right_line.c1;

  // 方向盘转角 左正右负 单位 degree
  GetContext.mutable_state_info()->steer_wheel_angle_degree =
      vehicle_service_output_info_ptr->steering_wheel_angle * 57.3;

  // 是否换道判断
  bool left_line_changed_flag = false;
  if (last_left_line_valid_flag_ &&
      GetContext.get_road_info()->current_lane.left_line.valid) {
    if (std::fabs(last_left_line_distance_ -
                  GetContext.get_state_info()->fl_wheel_distance_to_line) >
        2.0) {
      left_line_changed_flag = true;
    }
  } else {
    left_line_changed_flag = false;
  }
  bool right_line_changed_flag = false;
  if (last_right_line_valid_flag_ &&
      GetContext.get_road_info()->current_lane.right_line.valid) {
    if (std::fabs(last_right_line_distance_ -
                  GetContext.get_state_info()->fr_wheel_distance_to_line) >
        2.0) {
      right_line_changed_flag = true;
    }
  } else {
    right_line_changed_flag = false;
  }
  if (left_line_changed_flag && right_line_changed_flag) {
    GetContext.mutable_road_info()->current_lane.lane_changed_flag = true;
  } else {
    GetContext.mutable_road_info()->current_lane.lane_changed_flag = false;
  }
  last_left_line_valid_flag_ =
      GetContext.get_road_info()->current_lane.left_line.valid;
  last_left_line_distance_ =
      GetContext.get_state_info()->fl_wheel_distance_to_line;
  last_right_line_valid_flag_ =
      GetContext.get_road_info()->current_lane.right_line.valid;
  last_right_line_distance_ =
      GetContext.get_state_info()->fr_wheel_distance_to_line;
}

void Preprocess::SetLineInfoDefault(
    adas_function::context::LineInfo *line_info_ptr, double c0) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  uint32 car_points_num = 20;

  // set line_type
  line_info_ptr->line_type = context::Enum_LineType::Enum_LineType_SelfSet;

  // set boundary_type
  line_info_ptr->boundary_type = iflyauto::LaneBoundaryType_MARKING_UNKNOWN;

  // set 车道线方程系数
  line_info_ptr->c0 = c0;
  line_info_ptr->c1 = 0.0;
  line_info_ptr->c2 = GetContext.mutable_state_info()->ego_curvature / 2.0;
  line_info_ptr->c3 = 0.0;

  line_info_ptr->dx_vec_.resize(car_points_num, 0);
  line_info_ptr->dx_vec_.clear();
  line_info_ptr->dx_vec_.reserve(car_points_num);
  line_info_ptr->dy_vec_.resize(car_points_num, 0);
  line_info_ptr->dy_vec_.clear();
  line_info_ptr->dy_vec_.reserve(car_points_num);
  line_info_ptr->s_vec_.resize(car_points_num, 0);
  line_info_ptr->s_vec_.clear();
  line_info_ptr->s_vec_.reserve(car_points_num);
  double x = -50.0;
  double s = 0.0;
  for (uint32 i = 0; i < car_points_num; i++) {
    line_info_ptr->dx_vec_.emplace_back(x);
    const double y =
        line_info_ptr->c0 +
        (line_info_ptr->c1 + (line_info_ptr->c2 + line_info_ptr->c3 * x) * x) *
            x;
    line_info_ptr->dy_vec_.emplace_back(y);

    if (i == 0) {
      line_info_ptr->s_vec_.emplace_back(0.0);
    } else {
      const double ds =
          std::hypot(line_info_ptr->dx_vec_[i] - line_info_ptr->dx_vec_[i - 1],
                     line_info_ptr->dy_vec_[i] - line_info_ptr->dy_vec_[i - 1]);

      s += std::max(ds, 1e-3);
      line_info_ptr->s_vec_.emplace_back(s);
    }

    x += 10.0;
  }
  line_info_ptr->begin_s = 0.0;
  line_info_ptr->end_s = s;
  line_info_ptr->dx_s_spline_.set_points(line_info_ptr->s_vec_,
                                         line_info_ptr->dx_vec_);
  line_info_ptr->dy_s_spline_.set_points(line_info_ptr->s_vec_,
                                         line_info_ptr->dy_vec_);

  if (!line_info_ptr->dx_vec_.empty()) {
    line_info_ptr->begin = line_info_ptr->dx_vec_.front();
    line_info_ptr->end = line_info_ptr->dx_vec_.back();
  } else {
    line_info_ptr->begin = 0.0;
    line_info_ptr->end = 0.0;
  }
}

bool Preprocess::SetLineInfo(adas_function::context::LineInfo *line_info_ptr,
                             const iflyauto::LaneBoundary &lane_boundary_ptr) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  // lane boundary existence check
  bool lane_boundary_info_valid = false;
  if (lane_boundary_ptr.existence == true) {
    lane_boundary_info_valid = true;
  } else {
    lane_boundary_info_valid = false;
  }

  // enu_points_num check
  uint32 enu_points_num = lane_boundary_ptr.enu_points_size;
  if (enu_points_num <= 2) {
    lane_boundary_info_valid = false;
    return false;
  } else {
    // do nothing
  }
  if (enu_points_num > FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM) {
    lane_boundary_info_valid = false;
  } else {
    // do nothing
  }

  // set 车道线方程系数
  line_info_ptr->c0 = lane_boundary_ptr.poly_coefficient[0];
  line_info_ptr->c1 = lane_boundary_ptr.poly_coefficient[1];
  line_info_ptr->c2 = lane_boundary_ptr.poly_coefficient[2];
  line_info_ptr->c3 = lane_boundary_ptr.poly_coefficient[3];

  line_info_ptr->dx_vec_.resize(enu_points_num, 0);
  line_info_ptr->dx_vec_.clear();
  line_info_ptr->dx_vec_.reserve(enu_points_num);
  line_info_ptr->dy_vec_.resize(enu_points_num, 0);
  line_info_ptr->dy_vec_.clear();
  line_info_ptr->dy_vec_.reserve(enu_points_num);

  line_info_ptr->s_vec_.resize(enu_points_num, 0);
  line_info_ptr->s_vec_.clear();
  line_info_ptr->s_vec_.reserve(enu_points_num);
  double s = 0.0;
  // Eigen::Matrix2d rotm2d = Eigen::Matrix2d::Identity();
  const auto rotm2d = GetContext.get_state_info()->rotm2d;
  for (uint32 i = 0; i < enu_points_num; i++) {
    Eigen::Vector2d enu_pos_i(lane_boundary_ptr.enu_points[i].x,
                              lane_boundary_ptr.enu_points[i].y);
    auto transformed_point =
        rotm2d.transpose() *
        (enu_pos_i - GetContext.get_state_info()->current_pos_i);
    line_info_ptr->dx_vec_.emplace_back(transformed_point.x());
    line_info_ptr->dy_vec_.emplace_back(transformed_point.y());
    if (i == 0) {
      line_info_ptr->s_vec_.emplace_back(0.0);
    } else {
      const double ds =
          std::hypot(line_info_ptr->dx_vec_[i] - line_info_ptr->dx_vec_[i - 1],
                     line_info_ptr->dy_vec_[i] - line_info_ptr->dy_vec_[i - 1]);
      s += std::max(ds, 1e-3);
      line_info_ptr->s_vec_.emplace_back(s);
    }
  }
  line_info_ptr->begin_s = 0.0;
  line_info_ptr->end_s = s;
  line_info_ptr->dx_s_spline_.set_points(line_info_ptr->s_vec_,
                                         line_info_ptr->dx_vec_);
  line_info_ptr->dy_s_spline_.set_points(line_info_ptr->s_vec_,
                                         line_info_ptr->dy_vec_);

  if (!line_info_ptr->dx_vec_.empty()) {
    line_info_ptr->begin = line_info_ptr->dx_vec_.front();
    line_info_ptr->end = line_info_ptr->dx_vec_.back();
  } else {
    line_info_ptr->begin = 0.0;
    line_info_ptr->end = 0.0;
  }

  Eigen::Vector2d current_pos = Eigen::Vector2d::Zero();
  double s_proj = 0.0;
  double s_proj_front = 0.0;
  double s_proj_back = 0.0;
  int front_segment_loc_index = 0;
  int back_segment_loc_index = 0;
  CalProjectionPointByNewtonIteration(
      line_info_ptr->dx_s_spline_, line_info_ptr->dy_s_spline_, 0,
      line_info_ptr->end_s, current_pos, s_proj);
  // boundary_type check
  auto boundary_type_raw = lane_boundary_ptr.type_segments[0].type;

  // boundary_type check
  boundary_type_raw = lane_boundary_ptr.type_segments[0].type;
  auto boundary_type_raw_front = lane_boundary_ptr.type_segments[0].type;
  auto boundary_type_raw_middle = lane_boundary_ptr.type_segments[0].type;
  auto boundary_type_raw_back = lane_boundary_ptr.type_segments[0].type;
  s_proj_front = std::min(
      s_proj + 0.75 * GetContext.get_param()->lane_boundary_vaild_length_set +
          GetContext.get_state_info()->vehicle_speed * 1.0,
      line_info_ptr->end_s);  // 对车辆所处位置、偏离点位置的线型做判断
  s_proj_back = std::max(
      (s_proj - 0.33 * GetContext.get_param()->lane_boundary_vaild_length_set),
      0.0);
  // double distance_to_first_point = 0.0 - lane_boundary_ptr.car_points[0].x;
  if (s_proj < lane_boundary_ptr.type_segments[0].length) {
    boundary_type_raw = lane_boundary_ptr.type_segments[0].type;
  } else if (s_proj < (lane_boundary_ptr.type_segments[0].length +
                       lane_boundary_ptr.type_segments[1].length)) {
    boundary_type_raw = lane_boundary_ptr.type_segments[1].type;
  } else if (s_proj < (lane_boundary_ptr.type_segments[0].length +
                       lane_boundary_ptr.type_segments[1].length +
                       lane_boundary_ptr.type_segments[2].length)) {
    boundary_type_raw = lane_boundary_ptr.type_segments[2].type;
  } else {
    boundary_type_raw = lane_boundary_ptr.type_segments[3].type;
  }

  if (s_proj_front < lane_boundary_ptr.type_segments[0].length) {
    boundary_type_raw_front = lane_boundary_ptr.type_segments[0].type;
    front_segment_loc_index = 0;
  } else if (s_proj_front < (lane_boundary_ptr.type_segments[0].length +
                             lane_boundary_ptr.type_segments[1].length)) {
    boundary_type_raw_front = lane_boundary_ptr.type_segments[1].type;
    front_segment_loc_index = 1;
  } else if (s_proj_front < (lane_boundary_ptr.type_segments[0].length +
                             lane_boundary_ptr.type_segments[1].length +
                             lane_boundary_ptr.type_segments[2].length)) {
    boundary_type_raw_front = lane_boundary_ptr.type_segments[2].type;
    front_segment_loc_index = 2;
  } else {
    boundary_type_raw_front = lane_boundary_ptr.type_segments[3].type;
    front_segment_loc_index = 3;
  }

  if (s_proj_back < lane_boundary_ptr.type_segments[0].length) {
    boundary_type_raw_back = lane_boundary_ptr.type_segments[0].type;
    back_segment_loc_index = 0;
  } else if (s_proj_back < (lane_boundary_ptr.type_segments[0].length +
                            lane_boundary_ptr.type_segments[1].length)) {
    boundary_type_raw_back = lane_boundary_ptr.type_segments[1].type;
    back_segment_loc_index = 1;
  } else if (s_proj_back < (lane_boundary_ptr.type_segments[0].length +
                            lane_boundary_ptr.type_segments[1].length +
                            lane_boundary_ptr.type_segments[2].length)) {
    boundary_type_raw_back = lane_boundary_ptr.type_segments[2].type;
    back_segment_loc_index = 2;
  } else {
    boundary_type_raw_back = lane_boundary_ptr.type_segments[3].type;
    back_segment_loc_index = 3;
  }

  if (front_segment_loc_index == back_segment_loc_index) {
    boundary_type_raw_front =
        lane_boundary_ptr.type_segments[front_segment_loc_index].type;
    boundary_type_raw = boundary_type_raw_front;
  } else {
    if ((front_segment_loc_index - back_segment_loc_index) == 1) {
      boundary_type_raw_front =
          lane_boundary_ptr.type_segments[front_segment_loc_index].type;
      boundary_type_raw_back =
          lane_boundary_ptr.type_segments[back_segment_loc_index].type;
      if ((boundary_type_raw_front ==
           iflyauto::LaneBoundaryType_MARKING_UNKNOWN) ||
          (boundary_type_raw_back ==
           iflyauto::LaneBoundaryType_MARKING_UNKNOWN)) {
        boundary_type_raw = iflyauto::LaneBoundaryType_MARKING_UNKNOWN;
      } else if ((boundary_type_raw_front ==
                  iflyauto::LaneBoundaryType_MARKING_VIRTUAL) ||
                 (boundary_type_raw_back ==
                  iflyauto::LaneBoundaryType_MARKING_VIRTUAL)) {
        boundary_type_raw = iflyauto::LaneBoundaryType_MARKING_VIRTUAL;
      } else if ((boundary_type_raw_front ==
                  iflyauto::LaneBoundaryType_MARKING_SOLID) ||
                 (boundary_type_raw_back ==
                  iflyauto::LaneBoundaryType_MARKING_SOLID)) {
        boundary_type_raw = iflyauto::LaneBoundaryType_MARKING_SOLID;
      } else {
        boundary_type_raw = iflyauto::LaneBoundaryType_MARKING_DASHED;
      }
    } else if ((front_segment_loc_index - back_segment_loc_index) == 2) {
      boundary_type_raw_front =
          lane_boundary_ptr.type_segments[front_segment_loc_index].type;
      boundary_type_raw_back =
          lane_boundary_ptr.type_segments[back_segment_loc_index].type;
      boundary_type_raw_middle =
          lane_boundary_ptr.type_segments[back_segment_loc_index + 1].type;
      if ((boundary_type_raw_middle ==
           iflyauto::LaneBoundaryType_MARKING_UNKNOWN) ||
          (boundary_type_raw_front ==
           iflyauto::LaneBoundaryType_MARKING_UNKNOWN) ||
          (boundary_type_raw_back ==
           iflyauto::LaneBoundaryType_MARKING_UNKNOWN)) {
        boundary_type_raw = iflyauto::LaneBoundaryType_MARKING_UNKNOWN;
      } else if ((boundary_type_raw_middle ==
                  iflyauto::LaneBoundaryType_MARKING_VIRTUAL) ||
                 (boundary_type_raw_front ==
                  iflyauto::LaneBoundaryType_MARKING_VIRTUAL) ||
                 (boundary_type_raw_back ==
                  iflyauto::LaneBoundaryType_MARKING_VIRTUAL)) {
        boundary_type_raw = iflyauto::LaneBoundaryType_MARKING_VIRTUAL;
      } else if ((boundary_type_raw_middle ==
                  iflyauto::LaneBoundaryType_MARKING_SOLID) ||
                 (boundary_type_raw_front ==
                  iflyauto::LaneBoundaryType_MARKING_SOLID) ||
                 (boundary_type_raw_back ==
                  iflyauto::LaneBoundaryType_MARKING_SOLID)) {
        boundary_type_raw = iflyauto::LaneBoundaryType_MARKING_SOLID;
      } else {
        boundary_type_raw = iflyauto::LaneBoundaryType_MARKING_DASHED;
      }
    } else {
      if ((lane_boundary_ptr.type_segments[0].type ==
           iflyauto::LaneBoundaryType_MARKING_UNKNOWN) ||
          (lane_boundary_ptr.type_segments[1].type ==
           iflyauto::LaneBoundaryType_MARKING_UNKNOWN) ||
          (lane_boundary_ptr.type_segments[2].type ==
           iflyauto::LaneBoundaryType_MARKING_UNKNOWN) ||
          (lane_boundary_ptr.type_segments[3].type ==
           iflyauto::LaneBoundaryType_MARKING_UNKNOWN)) {
        boundary_type_raw = iflyauto::LaneBoundaryType_MARKING_UNKNOWN;
      } else if ((lane_boundary_ptr.type_segments[0].type ==
                  iflyauto::LaneBoundaryType_MARKING_VIRTUAL) ||
                 (lane_boundary_ptr.type_segments[1].type ==
                  iflyauto::LaneBoundaryType_MARKING_VIRTUAL) ||
                 (lane_boundary_ptr.type_segments[2].type ==
                  iflyauto::LaneBoundaryType_MARKING_VIRTUAL) ||
                 (lane_boundary_ptr.type_segments[3].type ==
                  iflyauto::LaneBoundaryType_MARKING_VIRTUAL)) {
        boundary_type_raw = iflyauto::LaneBoundaryType_MARKING_VIRTUAL;
      } else if ((lane_boundary_ptr.type_segments[0].type ==
                  iflyauto::LaneBoundaryType_MARKING_SOLID) ||
                 (lane_boundary_ptr.type_segments[1].type ==
                  iflyauto::LaneBoundaryType_MARKING_SOLID) ||
                 (lane_boundary_ptr.type_segments[2].type ==
                  iflyauto::LaneBoundaryType_MARKING_SOLID) ||
                 (lane_boundary_ptr.type_segments[3].type ==
                  iflyauto::LaneBoundaryType_MARKING_SOLID)) {
        boundary_type_raw = iflyauto::LaneBoundaryType_MARKING_SOLID;
      } else {
        boundary_type_raw = iflyauto::LaneBoundaryType_MARKING_DASHED;
      }
    }
  }

  // if ((boundary_type_raw == iflyauto::LaneBoundaryType_MARKING_UNKNOWN) ||
  //     (boundary_type_raw_front == iflyauto::LaneBoundaryType_MARKING_UNKNOWN)
  //     || (boundary_type_raw_back ==
  //     iflyauto::LaneBoundaryType_MARKING_UNKNOWN)) {
  //   boundary_type_raw = iflyauto::LaneBoundaryType_MARKING_UNKNOWN;
  // } else if ((boundary_type_raw ==
  //             iflyauto::LaneBoundaryType_MARKING_VIRTUAL) ||
  //            (boundary_type_raw_front ==
  //             iflyauto::LaneBoundaryType_MARKING_VIRTUAL) ||
  //            (boundary_type_raw_back ==
  //             iflyauto::LaneBoundaryType_MARKING_VIRTUAL)) {
  //   boundary_type_raw = iflyauto::LaneBoundaryType_MARKING_VIRTUAL;
  // } else if ((boundary_type_raw == iflyauto::LaneBoundaryType_MARKING_SOLID)
  // ||
  //            (boundary_type_raw_front ==
  //             iflyauto::LaneBoundaryType_MARKING_SOLID) ||
  //            (boundary_type_raw_back ==
  //             iflyauto::LaneBoundaryType_MARKING_SOLID)) {
  //   boundary_type_raw = iflyauto::LaneBoundaryType_MARKING_SOLID;
  // } else if ((boundary_type_raw == iflyauto::LaneBoundaryType_MARKING_DASHED)
  // &&
  //            (boundary_type_raw_front ==
  //             iflyauto::LaneBoundaryType_MARKING_DASHED) &&
  //            (boundary_type_raw_back ==
  //             iflyauto::LaneBoundaryType_MARKING_DASHED)) {
  //   boundary_type_raw = iflyauto::LaneBoundaryType_MARKING_DASHED;
  // } else {
  //   boundary_type_raw = iflyauto::LaneBoundaryType_MARKING_UNKNOWN;
  // }

  if (boundary_type_raw == iflyauto::LaneBoundaryType_MARKING_UNKNOWN) {
    lane_boundary_info_valid = false;
  } else {
    // do nothing
  }
  line_info_ptr->segment0_length = lane_boundary_ptr.type_segments[0].length;
  line_info_ptr->segment0_type = lane_boundary_ptr.type_segments[0].type;
  line_info_ptr->segment1_length = lane_boundary_ptr.type_segments[1].length;
  line_info_ptr->segment1_type = lane_boundary_ptr.type_segments[1].type;
  line_info_ptr->segment2_length = lane_boundary_ptr.type_segments[2].length;
  line_info_ptr->segment2_type = lane_boundary_ptr.type_segments[2].type;
  line_info_ptr->segment3_length = lane_boundary_ptr.type_segments[3].length;
  line_info_ptr->segment3_type = lane_boundary_ptr.type_segments[3].type;

  // set boundary_type
  line_info_ptr->boundary_type = boundary_type_raw;
  // set line_type
  if (boundary_type_raw == iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
    line_info_ptr->line_type = context::Enum_LineType::Enum_LineType_Virtual;
  } else if (boundary_type_raw == iflyauto::LaneBoundaryType_MARKING_DASHED) {
    line_info_ptr->line_type = context::Enum_LineType::Enum_LineType_Dashed;
  } else if (boundary_type_raw == iflyauto::LaneBoundaryType_MARKING_SOLID) {
    line_info_ptr->line_type = context::Enum_LineType::Enum_LineType_Solid;
  } else {
    line_info_ptr->line_type = context::Enum_LineType::Enum_LineType_Other;
  }

  if (lane_boundary_info_valid == false) {
    return false;
  } else {
    // do nothing
  }

  return true;
}

void Preprocess::UpdateRoadInfo(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  bool current_lane_ptr_valid;

  auto ptr_virtual_lane_manager = GetContext.mutable_session()
                                      ->mutable_environmental_model()
                                      ->get_virtual_lane_manager();

  // 判断指针是否为空
  if (ptr_virtual_lane_manager != nullptr) {
    if (ptr_virtual_lane_manager->get_current_lane() != nullptr) {
      current_lane_ptr_valid = true;
    } else {
      current_lane_ptr_valid = false;
    }
  } else {
    current_lane_ptr_valid = false;
  }

  // 设置当前车道信息
  bool current_lane_left_line_valid;
  bool current_lane_right_line_valid;
  if (current_lane_ptr_valid) {
    // 设置左侧道线信息
    auto ptr_current_lane_left_boundary =
        ptr_virtual_lane_manager->get_current_lane()->get_left_lane_boundary();
    current_lane_left_line_valid =
        SetLineInfo(&GetContext.mutable_road_info()->current_lane.left_line,
                    ptr_current_lane_left_boundary);
    // 设置右侧道线信息
    auto ptr_current_lane_right_boundary =
        ptr_virtual_lane_manager->get_current_lane()->get_right_lane_boundary();
    current_lane_right_line_valid =
        SetLineInfo(&GetContext.mutable_road_info()->current_lane.right_line,
                    ptr_current_lane_right_boundary);

    // 设置车宽信息
    GetContext.mutable_road_info()->current_lane.lane_width =
        ptr_virtual_lane_manager->get_current_lane()->width();
  } else {
    current_lane_left_line_valid = false;
    current_lane_right_line_valid = false;
  }

  // 根据自车轨迹虚拟左侧边界线
  if (current_lane_left_line_valid == false) {
    SetLineInfoDefault(&GetContext.mutable_road_info()->current_lane.left_line,
                       1.87);
    // 设置车宽信息
    GetContext.mutable_road_info()->current_lane.lane_width = 3.75;
  } else {
    // do nothing
  }

  // 根据自车轨迹虚拟右侧边界线
  if (current_lane_right_line_valid == false) {
    SetLineInfoDefault(&GetContext.mutable_road_info()->current_lane.right_line,
                       -1.87);
    // 设置车宽信息
    GetContext.mutable_road_info()->current_lane.lane_width = 3.75;
  } else {
    // do nothing
  }
  if ((GetContext.get_road_info()->current_lane.left_line.line_type ==
       context::Enum_LineType::Enum_LineType_SelfSet) ||
      (GetContext.get_road_info()->current_lane.left_line.line_type ==
       context::Enum_LineType::Enum_LineType_Virtual)) {
    GetContext.mutable_road_info()->current_lane.left_line.valid = false;
  } else {
    GetContext.mutable_road_info()->current_lane.left_line.valid = true;
  }
  if ((GetContext.get_road_info()->current_lane.right_line.line_type ==
       context::Enum_LineType::Enum_LineType_SelfSet) ||
      (GetContext.get_road_info()->current_lane.right_line.line_type ==
       context::Enum_LineType::Enum_LineType_Virtual)) {
    GetContext.mutable_road_info()->current_lane.right_line.valid = false;
  } else {
    GetContext.mutable_road_info()->current_lane.right_line.valid = true;
  }
  // 根据当前车道中心线及其到边界的距离计算路沿信息
  SetRoadedgeInfo();
  // 路两侧是否有分、汇流口
  SidewayExistJudge();
}

void Preprocess::SetRoadedgeInfo(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  bool current_lane_ptr_valid;

  auto ptr_virtual_lane_manager = GetContext.mutable_session()
                                      ->mutable_environmental_model()
                                      ->get_virtual_lane_manager();
  // 判断指针是否为空
  if (ptr_virtual_lane_manager != nullptr) {
    if (ptr_virtual_lane_manager->get_current_lane() != nullptr) {
      if (ptr_virtual_lane_manager->get_current_lane()->lane_points().size() >
          1) {
        current_lane_ptr_valid = true;
      } else {
        current_lane_ptr_valid = false;
      }
    } else {
      current_lane_ptr_valid = false;
    }
  } else {
    current_lane_ptr_valid = false;
  }

  auto &left_roadedge_info =
      GetContext.mutable_road_info()->current_lane.left_roadedge;
  auto &right_roadedge_info =
      GetContext.mutable_road_info()->current_lane.right_roadedge;

  if (current_lane_ptr_valid) {
    auto &virtual_lane_refline_points =
        ptr_virtual_lane_manager->get_current_lane()->lane_points();
    int lane_points_num =
        ptr_virtual_lane_manager->get_current_lane()->lane_points().size();
    // 设置左侧路沿信息
    // left_roadedge_info.dx_vec_.resize(car_points_num, 0.0);
    left_roadedge_info.dx_vec_.clear();
    left_roadedge_info.dx_vec_.reserve(lane_points_num);
    // left_roadedge_info.dy_vec_.resize(car_points_num, 0.0);
    left_roadedge_info.dy_vec_.clear();
    left_roadedge_info.dy_vec_.reserve(lane_points_num);
    // left_roadedge_info.s_vec_.resize(car_points_num, 0.0);
    left_roadedge_info.s_vec_.clear();
    left_roadedge_info.s_vec_.reserve(lane_points_num);
    // left_roadedge_info.all_dx_vec_.resize(car_points_num, 0.0);
    left_roadedge_info.all_dx_vec_.clear();
    left_roadedge_info.all_dx_vec_.reserve(lane_points_num);
    // left_roadedge_info.all_dy_vec_.resize(car_points_num, 0.0);
    left_roadedge_info.all_dy_vec_.clear();
    left_roadedge_info.all_dy_vec_.reserve(lane_points_num);
    double s_left = 0.0;
    bool left_roadedge_begin_eixt_flag = false;
    bool left_roadedge_end_eixt_flag = false;
    double current_point_dx_in_order = 0.0;
    double current_point_dy_in_order = 0.0;
    bool the_left_fisrt_point_flag = true;
    int valid_point_nums = 0;
    for (uint32 i = 0; i < lane_points_num; i++) {
      Eigen::Vector2d enu_pos_i(virtual_lane_refline_points[i].enu_point.x,
                                virtual_lane_refline_points[i].enu_point.y);
      auto transformed_point =
          GetContext.get_state_info()->rotm2d.transpose() *
          (enu_pos_i - GetContext.get_state_info()->current_pos_i);
      double car_point_x = transformed_point.x();
      double car_point_y = transformed_point.y();
      current_point_dx_in_order = car_point_x;
      current_point_dy_in_order =
          car_point_y +
          virtual_lane_refline_points[i].distance_to_left_road_border;
      left_roadedge_info.all_dx_vec_.emplace_back(current_point_dx_in_order);
      left_roadedge_info.all_dy_vec_.emplace_back(current_point_dy_in_order);
      if (current_point_dy_in_order >
              GetContext.get_param()->ldp_roadedge_distance_limit ||
          left_roadedge_end_eixt_flag == true) {
        if (left_roadedge_begin_eixt_flag) {
          left_roadedge_end_eixt_flag = true;
        }
        continue;
      }
      left_roadedge_info.dx_vec_.emplace_back(current_point_dx_in_order);
      left_roadedge_info.dy_vec_.emplace_back(current_point_dy_in_order);
      valid_point_nums++;
      if (the_left_fisrt_point_flag) {
        the_left_fisrt_point_flag = false;
        left_roadedge_info.s_vec_.emplace_back(0.0);
      } else {
        const double ds =
            std::hypot(left_roadedge_info.dx_vec_[valid_point_nums - 1] -
                           left_roadedge_info.dx_vec_[valid_point_nums - 2],
                       left_roadedge_info.dy_vec_[valid_point_nums - 1] -
                           left_roadedge_info.dy_vec_[valid_point_nums - 2]);
        s_left += std::max(ds, 1e-3);
        left_roadedge_info.s_vec_.emplace_back(s_left);
      }

      if (!left_roadedge_begin_eixt_flag) {
        /*寻找第一个有效点，当寻找后不再更新*/
        left_roadedge_info.begin_x = current_point_dx_in_order;
        left_roadedge_info.begin_s = s_left;
        left_roadedge_begin_eixt_flag = true;
      }
      if (left_roadedge_begin_eixt_flag) {
        /*当寻找到第一个有效点后，开始寻找最后一个有效点，当未出现异常点，对end一直更新，直至出现异常点结束更新*/
        left_roadedge_info.end_x = current_point_dx_in_order;
        left_roadedge_info.end_s = s_left;
      }
    }
    /*计算*/
    if ((left_roadedge_info.end_s - left_roadedge_info.begin_s) > 10.0 &&
        left_roadedge_begin_eixt_flag && valid_point_nums > 2 &&
        left_roadedge_info.begin_x < 5.0 && left_roadedge_info.end_x > 18.0) {
      left_roadedge_info.dx_s_spline_.set_points(left_roadedge_info.s_vec_,
                                                 left_roadedge_info.dx_vec_);
      left_roadedge_info.dy_s_spline_.set_points(left_roadedge_info.s_vec_,
                                                 left_roadedge_info.dy_vec_);
      left_roadedge_info.valid = true;
    } else {
      left_roadedge_info.valid = false;
    }
    // 设置右侧路沿信息
    right_roadedge_info.dx_vec_.clear();
    // right_roadedge_info.dx_vec_.resize(car_points_num, 0.0);
    right_roadedge_info.dx_vec_.reserve(lane_points_num);
    right_roadedge_info.dy_vec_.clear();
    // right_roadedge_info.dy_vec_.resize(car_points_num, 0.0);
    right_roadedge_info.dy_vec_.reserve(lane_points_num);
    right_roadedge_info.s_vec_.clear();
    // right_roadedge_info.s_vec_.resize(car_points_num, 0.0);
    right_roadedge_info.s_vec_.reserve(lane_points_num);
    right_roadedge_info.all_dx_vec_.clear();
    // right_roadedge_info.all_dx_vec_.resize(car_points_num, 0.0);
    right_roadedge_info.all_dx_vec_.reserve(lane_points_num);
    right_roadedge_info.all_dy_vec_.clear();
    // right_roadedge_info.all_dy_vec_.resize(car_points_num, 0.0);
    right_roadedge_info.all_dy_vec_.reserve(lane_points_num);

    double s_right = 0.0;
    bool right_roadedge_begin_eixt_flag = false;
    bool right_roadedge_end_eixt_flag = false;
    bool the_right_fisrt_point_flag = true;
    valid_point_nums = 0;
    for (uint32 i = 0; i < lane_points_num; i++) {
      Eigen::Vector2d enu_pos_i(virtual_lane_refline_points[i].enu_point.x,
                                virtual_lane_refline_points[i].enu_point.y);
      auto transformed_point =
          GetContext.get_state_info()->rotm2d.transpose() *
          (enu_pos_i - GetContext.get_state_info()->current_pos_i);
      double car_point_x = transformed_point.x();
      double car_point_y = transformed_point.y();
      current_point_dx_in_order = car_point_x;
      current_point_dy_in_order =
          car_point_y -
          virtual_lane_refline_points[i].distance_to_right_road_border;
      right_roadedge_info.all_dx_vec_.emplace_back(current_point_dx_in_order);
      right_roadedge_info.all_dy_vec_.emplace_back(current_point_dy_in_order);
      if (current_point_dy_in_order <
              (-1.0 * GetContext.get_param()->ldp_roadedge_distance_limit) ||
          right_roadedge_end_eixt_flag == true) {
        if (right_roadedge_begin_eixt_flag == true) {
          right_roadedge_end_eixt_flag = true;
        }
        continue;
      }
      right_roadedge_info.dx_vec_.emplace_back(current_point_dx_in_order);
      right_roadedge_info.dy_vec_.emplace_back(current_point_dy_in_order);
      valid_point_nums++;
      if (the_right_fisrt_point_flag) {
        the_right_fisrt_point_flag = false;
        right_roadedge_info.s_vec_.emplace_back(0.0);
      } else {
        const double ds =
            std::hypot(right_roadedge_info.dx_vec_[valid_point_nums - 1] -
                           right_roadedge_info.dx_vec_[valid_point_nums - 2],
                       right_roadedge_info.dy_vec_[valid_point_nums - 1] -
                           right_roadedge_info.dy_vec_[valid_point_nums - 2]);
        s_right += std::max(ds, 1e-3);
        right_roadedge_info.s_vec_.emplace_back(s_right);
      }
      if (!right_roadedge_begin_eixt_flag) {
        /*寻找第一个有效点，当寻找后不再更新*/
        right_roadedge_info.begin_x = current_point_dx_in_order;
        right_roadedge_info.begin_s = s_right;
        right_roadedge_begin_eixt_flag = true;
      }
      if (right_roadedge_begin_eixt_flag) {
        /*当寻找到第一个有效点后，开始寻找最后一个有效点，当未出现异常点，对end一直更新，直至出现异常点结束更新*/
        right_roadedge_info.end_x = current_point_dx_in_order;
        right_roadedge_info.end_s = s_right;
      }
    }

    /*计算*/
    if ((right_roadedge_info.end_s - right_roadedge_info.begin_s) > 10.0 &&
        right_roadedge_begin_eixt_flag && valid_point_nums > 5 &&
        right_roadedge_info.begin_x < 5.0 && right_roadedge_info.end_x > 18.0) {
      right_roadedge_info.dx_s_spline_.set_points(right_roadedge_info.s_vec_,
                                                  right_roadedge_info.dx_vec_);
      right_roadedge_info.dy_s_spline_.set_points(right_roadedge_info.s_vec_,
                                                  right_roadedge_info.dy_vec_);
      right_roadedge_info.valid = true;
    } else {
      right_roadedge_info.valid = false;
    }
  } else {
    left_roadedge_info.valid = false;
    right_roadedge_info.valid = false;
  }
}

void Preprocess::SidewayExistJudge(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto ptr_virtual_lane_manager = GetContext.mutable_session()
                                      ->mutable_environmental_model()
                                      ->get_virtual_lane_manager();
  bool left_sideway_exist_flag = false;
  if (GetContext.get_road_info()->current_lane.left_line.valid == false ||
      ptr_virtual_lane_manager != nullptr) {
    // 当前车道的左侧道线无效，默认左侧分岔口不存在
  } else {
    if (ptr_virtual_lane_manager->get_left_lane() == nullptr) {
      // 表明左侧车道不存在，左侧分岔口判定为不存在
    } else {
      auto &current_lane_left_boundary =
          ptr_virtual_lane_manager->get_current_lane()
              ->get_left_lane_boundary();
      auto &left_lane_right_boundary =
          ptr_virtual_lane_manager->get_left_lane()->get_right_lane_boundary();

      uint32 left_num_1 = current_lane_left_boundary.car_points_size;
      uint32 left_num_2 = left_lane_right_boundary.car_points_size;
      double near_gap_tmp = 0.0;
      double un_match_point_num = 0;
      double un_match_point_num_max = 1;
      if (left_num_1 != left_num_2) {
        // 当点数不一致，表明左侧车道是不合理，左侧分岔口判定为不存在
      } else {
        const auto rotm2d = GetContext.get_state_info()->rotm2d;
        for (int i = 0; i < left_num_1; i++) {
          Eigen::Vector2d current_left_boun_enu_pos_i(
              current_lane_left_boundary.enu_points[i].x,
              current_lane_left_boundary.enu_points[i].y);
          Eigen::Vector2d left_right_boun_enu_pos_i(
              left_lane_right_boundary.enu_points[i].x,
              left_lane_right_boundary.enu_points[i].y);
          auto current_left_car_point =
              rotm2d.transpose() * (current_left_boun_enu_pos_i -
                                    GetContext.get_state_info()->current_pos_i);
          auto left_right_car_point =
              rotm2d.transpose() * (left_right_boun_enu_pos_i -
                                    GetContext.get_state_info()->current_pos_i);

          if (current_left_car_point.x() < 0.0 ||
              current_left_car_point.x() > 50.0) {
            continue;
          }
          near_gap_tmp = current_left_car_point.y() - left_right_car_point.y();
          if (near_gap_tmp > GetContext.get_param()->sideway_exist_gap_thrd) {
            un_match_point_num++;
          }
          if (un_match_point_num > un_match_point_num_max) {
            break;
          }
        }
        if (un_match_point_num > un_match_point_num_max) {
          left_sideway_exist_flag = true;
        }
      }
    }
  }

  bool right_sideway_exist_flag = false;
  if (GetContext.get_road_info()->current_lane.right_line.valid == false ||
      ptr_virtual_lane_manager != nullptr) {
    // 当前车道的右侧道线无效，默认右侧分岔口不存在
  } else {
    if (ptr_virtual_lane_manager->get_right_lane() == nullptr) {
      // 表明右侧车道不存在，左侧分岔口判定为不存在
    } else {
      auto &current_lane_right_boundary =
          ptr_virtual_lane_manager->get_current_lane()
              ->get_right_lane_boundary();
      auto &right_lane_left_boundary =
          ptr_virtual_lane_manager->get_right_lane()->get_left_lane_boundary();
      uint32 right_num_1 = current_lane_right_boundary.car_points_size;
      uint32 right_num_2 = right_lane_left_boundary.car_points_size;
      double near_gap_tmp = 0.0;
      double un_match_point_num = 0;
      double un_match_point_num_max = 1;
      if (right_num_1 != right_num_2) {
        // 当点数不一致，表明右侧车道是不合理，右侧分岔口判定为不存在
      } else {
        const auto rotm2d = GetContext.get_state_info()->rotm2d;
        for (int i = 0; i < right_num_1; i++) {
          Eigen::Vector2d current_right_boun_enu_pos_i(
              current_lane_right_boundary.enu_points[i].x,
              current_lane_right_boundary.enu_points[i].y);
          Eigen::Vector2d right_left_boun_enu_pos_i(
              right_lane_left_boundary.enu_points[i].x,
              right_lane_left_boundary.enu_points[i].y);
          auto current_right_car_point =
              rotm2d.transpose() * (current_right_boun_enu_pos_i -
                                    GetContext.get_state_info()->current_pos_i);
          auto right_lrft_car_point =
              rotm2d.transpose() * (right_left_boun_enu_pos_i -
                                    GetContext.get_state_info()->current_pos_i);
          if (current_right_car_point.x() < 0.0 ||
              current_right_car_point.x() > 50.0) {
            continue;
          }
          near_gap_tmp = current_right_car_point.y() - right_lrft_car_point.y();
          if (near_gap_tmp <
              (-1.0 * GetContext.get_param()->sideway_exist_gap_thrd)) {
            un_match_point_num++;
          }
          if (un_match_point_num > un_match_point_num_max) {
            break;
          }
        }
        if (un_match_point_num > un_match_point_num_max) {
          right_sideway_exist_flag = true;
        }
      }
    }
  }

  // GetContext.mutable_road_info()->current_lane.left_sideway_exist_flag =
  //     left_sideway_exist_flag;
  // GetContext.mutable_road_info()->current_lane.right_sideway_exist_flag =
  //     right_sideway_exist_flag;

  if (ptr_virtual_lane_manager == nullptr) {
    GetContext.mutable_road_info()->current_lane.left_sideway_exist_flag =
        false;
    GetContext.mutable_road_info()->current_lane.right_sideway_exist_flag =
        false;
  } else {
    auto relative_id_zero_nums =
        ptr_virtual_lane_manager->origin_relative_id_zero_nums();
    if (relative_id_zero_nums == 1 || relative_id_zero_nums == 0) {
      no_sideway_exist_time_counts += 0.1;
    } else {
      no_sideway_exist_time_counts = 0.0;
    }

    if (no_sideway_exist_time_counts > 1.0) {
      GetContext.mutable_road_info()->current_lane.left_sideway_exist_flag =
          false;
      GetContext.mutable_road_info()->current_lane.right_sideway_exist_flag =
          false;
      no_sideway_exist_time_counts = 1.0;
    } else {
      if ((ptr_virtual_lane_manager->get_left_lane() == nullptr) && (relative_id_zero_nums > 1)) {
        GetContext.mutable_road_info()->current_lane.left_sideway_exist_flag =
            true;
      }
      if ((ptr_virtual_lane_manager->get_right_lane() == nullptr)&& (relative_id_zero_nums > 1)) {
        GetContext.mutable_road_info()->current_lane.right_sideway_exist_flag =
            true;
      }
    }
  }
  return;
}

void Preprocess::UpdateObjsInfo(void) {
  // 根据目标物坐标合车道线信息，赋予目标物车道属性
  ObjInLaneJudge();
  // 划分车辆周边8个区域
  SetEgoAroundAreaRange();
  // 筛选出不同区域内最有可能碰撞的车、人
  SingleAreaObjSelect();
  // 根据周边障碍物判断是否需要进行安全偏离
  SafeDeparturePermissionJudge();
  return;
}

void Preprocess::ObjInLaneJudge() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  const auto &fusion_objs = GetContext.get_session()
                                ->environmental_model()
                                .get_local_view()
                                .fusion_objects_info.fusion_object;
  int fusion_objs_num = GetContext.get_session()
                            ->environmental_model()
                            .get_local_view()
                            .fusion_objects_info.fusion_object_size;
  auto &objs_vector = GetContext.mutable_objs_info()->all_objs_vector;
  objs_vector.clear();
  objs_vector.resize(fusion_objs_num);
  ILOG_DEBUG << "fusion_objs_num == " << fusion_objs_num;
  ILOG_DEBUG << "objs_vector.size() == " << objs_vector.size();
  // context::FusionObjExtractInfo obj_info{};
  for (int i = 0; i < fusion_objs_num; i++) {
    auto &obj_info = objs_vector[i];
    obj_info.type = fusion_objs[i].common_info.type;
    obj_info.fusion_source = fusion_objs[i].additional_info.fusion_source;
    obj_info.track_age = fusion_objs[i].additional_info.track_age;
    obj_info.confidence = fusion_objs[i].additional_info.confidence;
    obj_info.track_status = fusion_objs[i].additional_info.track_status;
    obj_info.relative_position_x =
        fusion_objs[i].common_info.relative_center_position.x;
    obj_info.relative_position_y =
        fusion_objs[i].common_info.relative_center_position.y;
    obj_info.relative_speed_x = fusion_objs[i].common_info.relative_velocity.x;
    obj_info.relative_speed_y = fusion_objs[i].common_info.relative_velocity.y;
    obj_info.relative_acceleration_x =
        fusion_objs[i].common_info.relative_acceleration.x;
    obj_info.relative_acceleration_y =
        fusion_objs[i].common_info.relative_acceleration.y;
    obj_info.width = fusion_objs[i].common_info.shape.width;
    obj_info.length = fusion_objs[i].common_info.shape.length;
    obj_info.relative_theta = fusion_objs[i].common_info.relative_heading_angle;
    // calculate obj outline
    obj_info.relative_position_x_up =
        obj_info.relative_position_x + 0.5 * obj_info.length;
    obj_info.relative_position_x_down =
        obj_info.relative_position_x - 0.5 * obj_info.length;
    obj_info.relative_position_y_left =
        obj_info.relative_position_y + 0.5 * obj_info.width;
    obj_info.relative_position_y_right =
        obj_info.relative_position_y - 0.5 * obj_info.width;
    // judge
    double left_y_relative_line = 0.0;
    double right_y_relative_line = 0.0;
    if ((GetContext.get_road_info()->current_lane.left_line.valid ||
         GetContext.get_road_info()->current_lane.left_line.line_type ==
             context::Enum_LineType_Virtual) &&
        (GetContext.get_road_info()->current_lane.right_line.valid ||
         GetContext.get_road_info()->current_lane.right_line.line_type ==
             context::Enum_LineType_Virtual)) {
      /*left*/
      left_y_relative_line =
          GetContext.get_road_info()->current_lane.left_line.c0 +
          GetContext.get_road_info()->current_lane.left_line.c1 *
              obj_info.relative_position_x +
          GetContext.get_road_info()->current_lane.left_line.c2 *
              pow(obj_info.relative_position_x, 2) +
          GetContext.get_road_info()->current_lane.left_line.c3 *
              pow(obj_info.relative_position_x, 3);
      if ((obj_info.relative_position_y - left_y_relative_line) > 0.0 &&
          (obj_info.relative_position_y - left_y_relative_line) <
              GetContext.get_param()->lat_buffer_to_line) {
        obj_info.obj_loc_in_lane = context::Enum_LaneLocType::Enum_Left_Lane;
        continue;
      } else if (obj_info.relative_position_y - left_y_relative_line >
                 GetContext.get_param()->lat_buffer_to_line) {
        obj_info.obj_loc_in_lane = context::Enum_LaneLocType::Enum_Other;
        continue;
      } else {
        /*do nothing*/
      }
      /*right*/
      right_y_relative_line =
          GetContext.get_road_info()->current_lane.right_line.c0 +
          GetContext.get_road_info()->current_lane.right_line.c1 *
              obj_info.relative_position_x +
          GetContext.get_road_info()->current_lane.right_line.c2 *
              pow(obj_info.relative_position_x, 2) +
          GetContext.get_road_info()->current_lane.right_line.c3 *
              pow(obj_info.relative_position_x, 3);
      if (((obj_info.relative_position_y - right_y_relative_line) < 0.0) &&
          ((obj_info.relative_position_y - right_y_relative_line) >=
           (-1.0 * GetContext.get_param()->lat_buffer_to_line))) {
        obj_info.obj_loc_in_lane = context::Enum_LaneLocType::Enum_Right_Lane;
        continue;
      } else if (obj_info.relative_position_y - right_y_relative_line <
                 (-1.0 * GetContext.get_param()->lat_buffer_to_line)) {
        obj_info.obj_loc_in_lane = context::Enum_LaneLocType::Enum_Other;
        continue;
      } else {
        obj_info.obj_loc_in_lane = context::Enum_LaneLocType::Enum_Current_Lane;
        continue;
      }
    } else {
      obj_info.obj_loc_in_lane = context::Enum_LaneLocType::Enum_Other;
    }
    // else if (GetContext.get_road_info()->current_lane.left_line.valid ||
    //            GetContext.get_road_info()->current_lane.left_line.line_type
    //            ==
    //                context::Enum_LineType_Virtual) {
    //   /*left*/
    //   left_y_relative_line =
    //       GetContext.get_road_info()->current_lane.left_line.c0 +
    //       GetContext.get_road_info()->current_lane.left_line.c1 *
    //           obj_info.relative_position_x +
    //       GetContext.get_road_info()->current_lane.left_line.c2 *
    //           pow(obj_info.relative_position_x, 2) +
    //       GetContext.get_road_info()->current_lane.left_line.c3 *
    //           pow(obj_info.relative_position_x, 3);
    //   if ((obj_info.relative_position_y - left_y_relative_line) > 0.0 &&
    //       (obj_info.relative_position_y - left_y_relative_line) <
    //           GetContext.get_param()->lat_buffer_to_line) {
    //     obj_info.obj_loc_in_lane = context::Enum_LaneLocType::Enum_Left_Lane;
    //   } else if (obj_info.relative_position_y - left_y_relative_line >
    //              GetContext.get_param()->lat_buffer_to_line) {
    //     obj_info.obj_loc_in_lane = context::Enum_LaneLocType::Enum_Other;
    //   } else if (obj_info.relative_position_y - left_y_relative_line >
    //              (-1.0 * GetContext.get_param()->lat_buffer_to_line)) {
    //     obj_info.obj_loc_in_lane =
    //     context::Enum_LaneLocType::Enum_Current_Lane;
    //   } else {
    //     obj_info.obj_loc_in_lane = context::Enum_LaneLocType::Enum_Other;
    //   }
    // } else if (GetContext.get_road_info()->current_lane.right_line.valid ||
    //            GetContext.get_road_info()->current_lane.right_line.line_type
    //            ==
    //                context::Enum_LineType_Virtual) {
    //   /*right*/
    //   right_y_relative_line =
    //       GetContext.get_road_info()->current_lane.right_line.c0 +
    //       GetContext.get_road_info()->current_lane.right_line.c1 *
    //           obj_info.relative_position_x +
    //       GetContext.get_road_info()->current_lane.right_line.c2 *
    //           pow(obj_info.relative_position_x, 2) +
    //       GetContext.get_road_info()->current_lane.right_line.c3 *
    //           pow(obj_info.relative_position_x, 3);
    //   if (((obj_info.relative_position_y - right_y_relative_line) < 0.0) &&
    //       ((obj_info.relative_position_y - right_y_relative_line) >=
    //        (-1.0 * GetContext.get_param()->lat_buffer_to_line))) {
    //     obj_info.obj_loc_in_lane =
    //     context::Enum_LaneLocType::Enum_Right_Lane;
    //   } else if (obj_info.relative_position_y - right_y_relative_line <
    //              (-1.0 * GetContext.get_param()->lat_buffer_to_line)) {
    //     obj_info.obj_loc_in_lane = context::Enum_LaneLocType::Enum_Other;
    //   } else if (((obj_info.relative_position_y - right_y_relative_line) >
    //               0.0) &&
    //              ((obj_info.relative_position_y - right_y_relative_line) <=
    //               GetContext.get_param()->lat_buffer_to_line)) {
    //     obj_info.obj_loc_in_lane =
    //     context::Enum_LaneLocType::Enum_Current_Lane;
    //   } else {
    //     obj_info.obj_loc_in_lane = context::Enum_LaneLocType::Enum_Other;
    //   }
    // } else {
    //   obj_info.obj_loc_in_lane = context::Enum_LaneLocType::Enum_Other;
    // }
  }
  return;
}
void Preprocess::SetEgoAroundAreaRange() {
  // 区域划分
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  /*
          y_1   y_2      y_3   y_4
x_0-------|-----|--------|-----|----------
          |     |        |     |lon_distance_buffer1
          |     |        |     |
          |     |        |     |
          |     |        |     |
          |     |        |     |
x_1   ----|-----|        |-----|----------
          |*****|        |*****|lon_distance_buffer0
          |*****|        |*****|
x_2-------|#####|  ^^^^  |#####|
          |#####| |-qz-| |#####|
          |#####| |vehi| |#####|
          |#####| |-hz-| |#####|
x_3-------|#####|  ~~~~  |#####|----------
          |*****|        |*****|buffer
          |*****|        |*****|
x_4   ----|-----|        |-----|----------
          |     |        |     |
          |     |        |     |
          |     |        |     |
          |     |        |     |
          |     |        |     |
x_5-------|-----|--------|-----|----------
  */
  auto &ego_around_area_x_vector =
      GetContext.mutable_objs_info()->ego_around_area_x_vector;
  ego_around_area_x_vector.clear();
  ego_around_area_x_vector.reserve(6);
  ego_around_area_x_vector[0] = GetContext.get_param()->origin_2_front_bumper +
                                GetContext.get_param()->lon_distance_buffer0 +
                                GetContext.get_param()->lon_distance_buffer1;
  ego_around_area_x_vector[1] = GetContext.get_param()->origin_2_front_bumper +
                                GetContext.get_param()->lon_distance_buffer0;
  ego_around_area_x_vector[2] = GetContext.get_param()->origin_2_front_bumper;
  ego_around_area_x_vector[3] =
      -1.0 * GetContext.get_param()->origin_2_rear_bumper;
  ego_around_area_x_vector[4] =
      -1.0 * GetContext.get_param()->origin_2_rear_bumper -
      GetContext.get_param()->lon_distance_buffer0;
  ego_around_area_x_vector[5] =
      -1.0 * GetContext.get_param()->origin_2_rear_bumper -
      GetContext.get_param()->lon_distance_buffer0 -
      GetContext.get_param()->lon_distance_buffer1;
  return;
}
void Preprocess::SingleAreaObjSelect() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto &objs_vector = GetContext.get_objs_info()->all_objs_vector;
  auto &objs_selrcted_struct = GetContext.mutable_objs_info()->objs_selected;
  objs_selrcted_struct.fl_objs.person_info_valid = false;
  objs_selrcted_struct.fl_objs.vehicle_info_valid = false;
  objs_selrcted_struct.fm_objs.person_info_valid = false;
  objs_selrcted_struct.fm_objs.vehicle_info_valid = false;
  objs_selrcted_struct.fr_objs.person_info_valid = false;
  objs_selrcted_struct.fr_objs.vehicle_info_valid = false;
  objs_selrcted_struct.ml_objs.person_info_valid = false;
  objs_selrcted_struct.ml_objs.vehicle_info_valid = false;
  objs_selrcted_struct.mr_objs.person_info_valid = false;
  objs_selrcted_struct.mr_objs.vehicle_info_valid = false;
  objs_selrcted_struct.rl_objs.person_info_valid = false;
  objs_selrcted_struct.rl_objs.vehicle_info_valid = false;
  objs_selrcted_struct.rm_objs.person_info_valid = false;
  objs_selrcted_struct.rm_objs.vehicle_info_valid = false;
  objs_selrcted_struct.rr_objs.person_info_valid = false;
  objs_selrcted_struct.rr_objs.vehicle_info_valid = false;
  int objs_num = objs_vector.size();
  for (int i = 0; i < objs_num; i++) {
    /*enum ObjectType {
  OBJECT_TYPE_UNKNOWN = 0,
  OBJECT_TYPE_UNKNOWN_MOVABLE = 1,
  OBJECT_TYPE_UNKNOWN_IMMOVABLE = 2,
  OBJECT_TYPE_COUPE = 3,
  OBJECT_TYPE_MINIBUS = 4,
  OBJECT_TYPE_VAN = 5,
  OBJECT_TYPE_BUS = 6,
  OBJECT_TYPE_TRUCK = 7,
  OBJECT_TYPE_TRAILER = 8,
  OBJECT_TYPE_BICYCLE = 9,
  OBJECT_TYPE_MOTORCYCLE = 10,
  OBJECT_TYPE_TRICYCLE = 11,
  OBJECT_TYPE_PEDESTRIAN = 12,
  OBJECT_TYPE_ANIMAL = 13,
  OBJECT_TYPE_CONE = 14,
  OBJECT_TYPE_TRAFFIC_BARRIER = 15,
  OBJECT_TYPE_TEMPORY_SIGN = 16,
  OBJECT_TYPE_FENCE = 17*/
    if ((objs_vector[i].obj_loc_in_lane ==
         context::Enum_LaneLocType::Enum_Other) ||
        (objs_vector[i].type < 3 || objs_vector[i].type > 8)) {
      continue;
    }
    int lon_position_judge = 0;  // 0:f 1:m 2:r 3:false

    // if (objs_vector[i].relative_position_x >=
    //                    GetContext.get_objs_info()->ego_around_area_x_vector[4]
    //                    &&
    //                objs_vector[i].relative_position_x <=
    //                    GetContext.get_objs_info()->ego_around_area_x_vector[1]){
    //                     lon_position_judge = 1;
    //                    }else if(objs_vector[i].relative_position_x >
    //             GetContext.get_objs_info()->ego_around_area_x_vector[1] &&
    //         objs_vector[i].relative_position_x <
    //             GetContext.get_objs_info()->ego_around_area_x_vector[0]) {
    //               lon_position_judge = 0;
    //             } else if (objs_vector[i].relative_position_x >=
    //                    GetContext.get_objs_info()->ego_around_area_x_vector[5]
    //                    &&
    //                objs_vector[i].relative_position_x <
    //                    GetContext.get_objs_info()->ego_around_area_x_vector[4])
    //                    {
    //       lon_position_judge = 2;
    //     } else {
    //       lon_position_judge = 3;
    //     }
    double blind_up_tmp =
        GetContext.get_objs_info()->ego_around_area_x_vector[1];
    double blind_down_tmp =
        GetContext.get_objs_info()->ego_around_area_x_vector[4];
    if (objs_vector[i].obj_loc_in_lane ==
        context::Enum_LaneLocType::Enum_Current_Lane) {
      blind_up_tmp = GetContext.get_objs_info()->ego_around_area_x_vector[2];
      blind_down_tmp = GetContext.get_objs_info()->ego_around_area_x_vector[3];
    }
    if (objs_vector[i].relative_position_x_down > blind_up_tmp &&
        objs_vector[i].relative_position_x_down <=
            GetContext.get_objs_info()->ego_around_area_x_vector[0]) {
      lon_position_judge = 0;
    } else if (objs_vector[i].relative_position_x_up >=
                   GetContext.get_objs_info()->ego_around_area_x_vector[5] &&
               objs_vector[i].relative_position_x_up < blind_down_tmp) {
      lon_position_judge = 2;
    } else if (objs_vector[i].relative_position_x_up <
                   GetContext.get_objs_info()->ego_around_area_x_vector[5] ||
               objs_vector[i].relative_position_x_down >
                   GetContext.get_objs_info()->ego_around_area_x_vector[0]) {
      lon_position_judge = 3;
    } else {
      lon_position_judge = 1;
    }
    if (lon_position_judge == 3) {
      continue;
    }
    if (objs_vector[i].obj_loc_in_lane ==
        context::Enum_LaneLocType::Enum_Left_Lane) {
      if (lon_position_judge == 0) {
        if (objs_selrcted_struct.fl_objs.vehicle_info_valid == false) {
          objs_selrcted_struct.fl_objs.vehicle_info = objs_vector[i];
          objs_selrcted_struct.fl_objs.vehicle_info_valid = true;
        } else {
          if (objs_vector[i].relative_position_x <
              objs_selrcted_struct.fl_objs.vehicle_info.relative_position_x) {
            objs_selrcted_struct.fl_objs.vehicle_info = objs_vector[i];
          }
        }
      } else if (lon_position_judge == 1) {
        if (objs_selrcted_struct.ml_objs.vehicle_info_valid == false) {
          objs_selrcted_struct.ml_objs.vehicle_info = objs_vector[i];
          objs_selrcted_struct.ml_objs.vehicle_info_valid = true;
        } else {
          continue;
        }
      } else if (lon_position_judge == 2) {
        if (objs_selrcted_struct.rl_objs.vehicle_info_valid == false) {
          objs_selrcted_struct.rl_objs.vehicle_info = objs_vector[i];
          objs_selrcted_struct.rl_objs.vehicle_info_valid = true;
        } else {
          if (objs_vector[i].relative_position_x >
              objs_selrcted_struct.rl_objs.vehicle_info.relative_position_x) {
            objs_selrcted_struct.rl_objs.vehicle_info = objs_vector[i];
          }
        }
      } else {
        continue;
      }
    } else if (objs_vector[i].obj_loc_in_lane ==
               context::Enum_LaneLocType::Enum_Current_Lane) {
      if (lon_position_judge == 0) {
        if (objs_selrcted_struct.fm_objs.vehicle_info_valid == false) {
          objs_selrcted_struct.fm_objs.vehicle_info = objs_vector[i];
          objs_selrcted_struct.fm_objs.vehicle_info_valid = true;
        } else {
          if (objs_vector[i].relative_position_x <
              objs_selrcted_struct.fm_objs.vehicle_info.relative_position_x) {
            objs_selrcted_struct.fm_objs.vehicle_info = objs_vector[i];
          }
        }
      } else if (lon_position_judge == 1) {
        continue;
      } else if (lon_position_judge == 2) {
        if (objs_selrcted_struct.rm_objs.vehicle_info_valid == false) {
          objs_selrcted_struct.rm_objs.vehicle_info = objs_vector[i];
          objs_selrcted_struct.rm_objs.vehicle_info_valid = true;
        } else {
          if (objs_vector[i].relative_position_x >
              objs_selrcted_struct.rm_objs.vehicle_info.relative_position_x) {
            objs_selrcted_struct.rm_objs.vehicle_info = objs_vector[i];
          }
        }
      } else {
        continue;
      }
    } else {
      if (lon_position_judge == 0) {
        if (objs_selrcted_struct.fr_objs.vehicle_info_valid == false) {
          objs_selrcted_struct.fr_objs.vehicle_info = objs_vector[i];
          objs_selrcted_struct.fr_objs.vehicle_info_valid = true;
        } else {
          if (objs_vector[i].relative_position_x <
              objs_selrcted_struct.fr_objs.vehicle_info.relative_position_x) {
            objs_selrcted_struct.fr_objs.vehicle_info = objs_vector[i];
          }
        }
      } else if (lon_position_judge == 1) {
        if (objs_selrcted_struct.mr_objs.vehicle_info_valid == false) {
          objs_selrcted_struct.mr_objs.vehicle_info = objs_vector[i];
          objs_selrcted_struct.mr_objs.vehicle_info_valid = true;
        } else {
          continue;
        }
      } else if (lon_position_judge == 2) {
        if (objs_selrcted_struct.rr_objs.vehicle_info_valid == false) {
          objs_selrcted_struct.rr_objs.vehicle_info = objs_vector[i];
          objs_selrcted_struct.rr_objs.vehicle_info_valid = true;
        } else {
          if (objs_vector[i].relative_position_x >
              objs_selrcted_struct.rr_objs.vehicle_info.relative_position_x) {
            objs_selrcted_struct.rr_objs.vehicle_info = objs_vector[i];
          }
        }
      } else {
        continue;
      }
    }
  }
}

void Preprocess::SafeDeparturePermissionJudge(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  if (GetContext.get_param()->force_no_safe_departure_switch == true) {
    GetContext.mutable_road_info()
        ->current_lane.left_safe_departure_permission_flag = false;
    GetContext.mutable_road_info()
        ->current_lane.right_safe_departure_permission_flag = false;
    return;
  }

  double fl_obj_ttc = 10.0;
  double fm_obj_ttc = 10.0;
  double fr_obj_ttc = 10.0;
  double left_obj_ttc = 10.0;
  double right_obj_ttc = 10.0;
  if (GetContext.get_objs_info()->objs_selected.fm_objs.vehicle_info_valid) {
    fm_obj_ttc = ObjCalculateTTC(
        GetContext.get_objs_info()->objs_selected.fm_objs.vehicle_info);
  }
  if (GetContext.get_objs_info()->objs_selected.fl_objs.vehicle_info_valid) {
    fl_obj_ttc = ObjCalculateTTC(
        GetContext.get_objs_info()->objs_selected.fl_objs.vehicle_info);
  }
  if (GetContext.get_objs_info()->objs_selected.fr_objs.vehicle_info_valid) {
    fr_obj_ttc = ObjCalculateTTC(
        GetContext.get_objs_info()->objs_selected.fr_objs.vehicle_info);
  }
  left_obj_ttc = std::min(fm_obj_ttc, fl_obj_ttc);
  right_obj_ttc = std::min(fm_obj_ttc, fr_obj_ttc);
  if (GetContext.get_objs_info()->objs_selected.mr_objs.vehicle_info_valid) {
    // 右侧有车并行,且左侧没路沿，允许向左安全偏离
    if (GetContext.get_road_info()->current_lane.left_roadedge.valid == false) {
      GetContext.mutable_road_info()
          ->current_lane.left_safe_departure_permission_flag = true;
    }
  } else {
    if (GetContext.get_road_info()
            ->current_lane.left_safe_departure_permission_flag == false) {
      if ((right_obj_ttc < GetContext.get_param()->safe_departure_ttc) &&
          (GetContext.get_road_info()->current_lane.left_roadedge.valid ==
           false)) {
        // 右侧车道或者正前方有碰撞风险，有车允许向左安全偏离
        GetContext.mutable_road_info()
            ->current_lane.left_safe_departure_permission_flag = true;
      }
    } else {
      if (right_obj_ttc > (GetContext.get_param()->safe_departure_ttc + 0.25)) {
        // 滞回
        GetContext.mutable_road_info()
            ->current_lane.left_safe_departure_permission_flag = false;
      }
    }
  }
  if (GetContext.get_objs_info()->objs_selected.ml_objs.vehicle_info_valid) {
    // 左侧有车并行且右侧无路沿，允许向右安全偏离
    if (GetContext.get_road_info()->current_lane.right_roadedge.valid ==
        false) {
      GetContext.mutable_road_info()
          ->current_lane.right_safe_departure_permission_flag = true;
    }
  } else {
    if (GetContext.get_road_info()
            ->current_lane.right_safe_departure_permission_flag == false) {
      if ((left_obj_ttc < GetContext.get_param()->safe_departure_ttc) &&
          (GetContext.get_road_info()->current_lane.right_roadedge.valid ==
           false)) {
        // 左侧车道或者正前方有碰撞风险，有车允许向右安全偏离
        GetContext.mutable_road_info()
            ->current_lane.right_safe_departure_permission_flag = true;
      }
    } else {
      if (left_obj_ttc > (GetContext.get_param()->safe_departure_ttc + 0.25)) {
        // 滞回
        GetContext.mutable_road_info()
            ->current_lane.right_safe_departure_permission_flag = false;
      }
    }
  }
  return;
}

double Preprocess::ObjCalculateTTC(const context::FusionObjExtractInfo &obj) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  double collision_x = 0.0;
  double obj_collision_x = 0.0;
  bool requre_obj_speed_direction_positive = true;
  if (obj.relative_position_x > 0) {
    collision_x = GetContext.get_param()->origin_2_front_bumper;
    requre_obj_speed_direction_positive = false;
    obj_collision_x = obj.relative_position_x_down;
  } else {
    collision_x = -1.0 * GetContext.get_param()->origin_2_rear_bumper;
    obj_collision_x = obj.relative_position_x_up;
  }
  double ttc_collision_tmp = 10.0;
  if (requre_obj_speed_direction_positive) {
    if (obj.relative_speed_x < 0.01) {
      return ttc_collision_tmp;
    }
  } else {
    if (obj.relative_speed_x > -0.01) {
      return ttc_collision_tmp;
    }
  }
  ttc_collision_tmp =
      std::fabs((obj_collision_x - collision_x) / obj.relative_speed_x);
  return ttc_collision_tmp;
}

void Preprocess::RunOnce(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  // 读取json文件,更新param_
  bool left_turn_light_state =
      GetContext.mutable_session()
          ->mutable_environmental_model()
          ->get_local_view()
          .vehicle_service_output_info.left_turn_light_state;

  bool left_turn_light_state_delay =
      GetContext.mutable_last_cycle_info()->left_turn_light_state;
  if ((left_turn_light_state_delay == false) &&
      (left_turn_light_state == true)) {
    SyncParameters();
  } else {
    // do nothing
  }

  // 更新road_info_
  UpdateRoadInfo();

  // 更新state_info_
  UpdateStateInfo();

  // 更新objs_info_
  UpdateObjsInfo();

  count_++;
  ILOG_DEBUG << "Preprocess::RunOnce count=" << count_;
}

}  // namespace preprocess
}  // namespace adas_function