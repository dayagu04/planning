#include "lane_keep_assist_type.h"
namespace planning {
uint16 uint16_bit[16] = {1,   2,   4,    8,    16,   32,   64,    128,
                         256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
bool LKALineLeftIntervention(float32 tlc_to_line_threshold,
                                planning::LkasInput *lkas_input) {
  float32 preview_distance_x =
      0;  // 在tlc_to_line_threshold秒后,车辆左前轮所处的纵向坐标,单位：m
  preview_distance_x =
      lkas_input->vehicle_info.veh_display_speed * tlc_to_line_threshold +
      lkas_input->vehicle_info.common_wheel_base;

  float32 front_left_wheel_distance_to_line =
      0;  // 左前轮至左侧道线的距离,单位：m
  front_left_wheel_distance_to_line =
      lkas_input->road_info.left_line_c0 +
      lkas_input->road_info.left_line_c1 * preview_distance_x +
      lkas_input->road_info.left_line_c2 * preview_distance_x *
          preview_distance_x +
      lkas_input->road_info.left_line_c3 * preview_distance_x *
          preview_distance_x * preview_distance_x -
      Common_FrontCamera_PosYL;  // 左前轮至左侧道线的距离,单位:m

  float32 preview_distance_y =
      0;  // 依据当前运动趋势,tlc_to_line_threshold秒后,车辆左前轮所处的横向坐标,单位:m
  // 按照当前车速以及横摆角速度，本车的运动轨迹：
  // Path(t)=0.5*ay*t^2=0.5*(V^2/R)*t^2=0.5*V*YawRate*t^2
  preview_distance_y = 0.5 * lkas_input->vehicle_info.veh_display_speed *
                       (lkas_input->vehicle_info.veh_yaw_rate) *
                       tlc_to_line_threshold * tlc_to_line_threshold;

  if ((front_left_wheel_distance_to_line - preview_distance_y) < 0) {
    return true;
  } else {
    return false;
  }
}
bool LKARoadEdgeLeftIntervention(float32 tlc_to_roadedge_threshold,
                                    planning::LkasInput *lkas_input) {
  float32 preview_distance_x =
      0;  // 在tlc_to_roadedge_threshold秒后,车辆左前轮所处的纵向坐标,单位：m
  preview_distance_x =
      lkas_input->vehicle_info.veh_display_speed * tlc_to_roadedge_threshold +
      lkas_input->vehicle_info.common_wheel_base;

  float32 front_left_wheel_distance_to_roadedge =
      0;  // 左前轮至左侧路沿的距离,单位：m
  front_left_wheel_distance_to_roadedge =
      lkas_input->road_info.left_roadedge_c0 +
      lkas_input->road_info.left_roadedge_c1 * preview_distance_x +
      lkas_input->road_info.left_roadedge_c2 * preview_distance_x *
          preview_distance_x +
      lkas_input->road_info.left_roadedge_c3 * preview_distance_x *
          preview_distance_x * preview_distance_x -
      Common_FrontCamera_PosYL;  // 左前轮至左侧路沿的距离,单位:m

  float32 preview_distance_y =
      0;  // 依据当前运动趋势,tlc_to_roadedge_threshold秒后,车辆左前轮所处的横向坐标,单位:m
  // 按照当前车速以及横摆角速度，本车的运动轨迹：
  // Path(t)=0.5*ay*t^2=0.5*(V^2/R)*t^2=0.5*V*YawRate*t^2
  preview_distance_y = 0.5 * lkas_input->vehicle_info.veh_display_speed *
                       (lkas_input->vehicle_info.veh_yaw_rate) *
                       tlc_to_roadedge_threshold * tlc_to_roadedge_threshold;

  if ((front_left_wheel_distance_to_roadedge - preview_distance_y) < 0) {
    return true;
  } else {
    return false;
  }
}
bool LKALineRightIntervention(float32 tlc_to_line_threshold,
                                 planning::LkasInput *lkas_input) {
  float32 preview_distance_x =
      0.0F;  // 在tlc_to_line_threshold秒后,车辆右前轮所处的纵向坐标,单位：m
  preview_distance_x =
      lkas_input->vehicle_info.veh_display_speed * tlc_to_line_threshold +
      lkas_input->vehicle_info.common_wheel_base;

  float32 front_right_wheel_distance_to_line =
      0.0F;  // 右前轮至右侧道线的距离,单位：m
  front_right_wheel_distance_to_line =
      lkas_input->road_info.right_line_c0 +
      lkas_input->road_info.right_line_c1 * preview_distance_x +
      lkas_input->road_info.right_line_c2 * preview_distance_x *
          preview_distance_x +
      lkas_input->road_info.right_line_c3 * preview_distance_x *
          preview_distance_x * preview_distance_x +
      Common_FrontCamera_PosYR;  // 右前轮至右侧道线的距离,单位:m

  float32 preview_distance_y =
      0;  // 依据当前运动趋势,tlc_to_line_threshold秒后,车辆右前轮所处的横向坐标,单位:m
  // 按照当前车速以及横摆角速度，本车的运动轨迹：
  // Path(t)=0.5*ay*t^2=0.5*(V^2/R)*t^2=0.5*V*YawRate*t^2
  preview_distance_y = 0.5 * lkas_input->vehicle_info.veh_display_speed *
                       (lkas_input->vehicle_info.veh_yaw_rate) *
                       tlc_to_line_threshold * tlc_to_line_threshold;

  if ((front_right_wheel_distance_to_line - preview_distance_y) > 0) {
    return true;
  } else {
    return false;
  }
}
bool LKARoadEdgeRightIntervention(float32 tlc_to_roadedge_threshold,
                                     planning::LkasInput *lkas_input) {
  float32 preview_distance_x =
      0;  // 在tlc_to_roadedge_threshold秒后,车辆右前轮所处的纵向坐标,单位：m
  preview_distance_x =
      lkas_input->vehicle_info.veh_display_speed * tlc_to_roadedge_threshold +
      lkas_input->vehicle_info.common_wheel_base;

  float32 front_right_wheel_distance_to_roadedge =
      0;  // 右前轮至右侧路沿的距离,单位：m
  front_right_wheel_distance_to_roadedge =
      lkas_input->road_info.right_roadedge_c0 +
      lkas_input->road_info.right_roadedge_c1 * preview_distance_x +
      lkas_input->road_info.right_roadedge_c2 * preview_distance_x *
          preview_distance_x +
      lkas_input->road_info.right_roadedge_c3 * preview_distance_x *
          preview_distance_x * preview_distance_x +
      Common_FrontCamera_PosYR;  // 右前轮至右侧路沿的距离,单位:m

  float32 preview_distance_y =
      0;  // 依据当前运动趋势,tlc_to_roadedge_threshold秒后,车辆右前轮所处的横向坐标,单位:m
  // 按照当前车速以及横摆角速度，本车的运动轨迹：
  // Path(t)=0.5*ay*t^2=0.5*(V^2/R)*t^2=0.5*V*YawRate*t^2
  preview_distance_y = 0.5 * lkas_input->vehicle_info.veh_display_speed *
                       (lkas_input->vehicle_info.veh_yaw_rate) *
                       tlc_to_roadedge_threshold * tlc_to_roadedge_threshold;

  if ((front_right_wheel_distance_to_roadedge - preview_distance_y) > 0) {
    return true;
  } else {
    return false;
  }
}
}  // namespace planning