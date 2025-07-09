#include "adas_function_lib.h"

#include "adas_function_struct.h"

namespace adas_function {

uint16 uint16_bit[16] = {1,   2,   4,    8,    16,   32,   64,    128,
                         256, 512, 1024, 2048, 4096, 8192, 16384, 32768};

uint32 uint32_bit[32] = {1,   2,   4,    8,    16,   32,   64,    128,
                         256, 512, 1024, 2048, 4096, 8192, 16384, 32768,
                         65536, 131072, 262144, 524288, 1048576, 2097152, 4194304, 8388608,
                         16777216, 33554432, 67108864, 134217728, 268435456, 536870912, 1073741824, 2147483648};

// 函数功能:更新box四个角点的坐标
void BoxCornersCoordinateUpdate(Box2DStr *box) {
  float32 dx1 = 0.5F * box->input.length * box->input.cos_heading_angle;
  float32 dy1 = 0.5F * box->input.length * box->input.sin_heading_angle;
  float32 dx2 = 0.5F * box->input.width * box->input.sin_heading_angle;
  float32 dy2 = 0.5F * box->input.width * box->input.cos_heading_angle;

  // 获取前左角点坐标
  box->state.fl_x = box->input.x + dx1 - dx2;
  box->state.fl_y = box->input.y + dy1 + dy2;

  // 获取前右角点坐标
  box->state.fr_x = box->input.x + dx1 + dx2;
  box->state.fr_y = box->input.y + dy1 - dy2;

  // 获取后左角点坐标
  box->state.rl_x = box->input.x - dx1 - dx2;
  box->state.rl_y = box->input.y - dy1 + dy2;

  // 获取后右角点坐标
  box->state.rr_x = box->input.x - dx1 + dx2;
  box->state.rr_y = box->input.y - dy1 - dy2;

  // 获取max_x
  box->state.max_x = box->state.fl_x;
  box->state.max_x = fmaxf(box->state.fr_x, box->state.max_x);
  box->state.max_x = fmaxf(box->state.rl_x, box->state.max_x);
  box->state.max_x = fmaxf(box->state.rr_x, box->state.max_x);

  // 获取min_x
  box->state.min_x = box->state.fl_x;
  box->state.min_x = fminf(box->state.fr_x, box->state.min_x);
  box->state.min_x = fminf(box->state.rl_x, box->state.min_x);
  box->state.min_x = fminf(box->state.rr_x, box->state.min_x);

  // 获取max_y
  box->state.max_y = box->state.fl_y;
  box->state.max_y = fmaxf(box->state.fr_y, box->state.max_y);
  box->state.max_y = fmaxf(box->state.rl_y, box->state.max_y);
  box->state.max_y = fmaxf(box->state.rr_y, box->state.max_y);

  // 获取min_y
  box->state.min_y = box->state.fl_y;
  box->state.min_y = fminf(box->state.fr_y, box->state.min_y);
  box->state.min_y = fminf(box->state.rl_y, box->state.min_y);
  box->state.min_y = fminf(box->state.rr_y, box->state.min_y);
}

void CalProjectionPointByNewtonIteration(const pnc::mathlib::spline &x_s_spline,
                                         const pnc::mathlib::spline &y_s_spline,
                                         const double s_start,
                                         const double s_end,
                                         const Eigen::Vector2d &x,
                                         double &s_proj) {
  double max_iter_ = 10;
  double tol_ = 1e-3;
  if (s_start > s_end) {
    return;
  }

  // newton iteration to calculate projection point
  const double half_length = (s_end - s_start) * 0.5;
  s_proj = (s_end + s_start) * 0.5;

  auto const &x0 = x.x();
  auto const &y0 = x.y();

  double cost =
      std::hypot((x0 - x_s_spline(s_proj)), (y0 - y_s_spline(s_proj)));

  // get s_proj
  for (size_t i = 0; i < max_iter_; ++i) {
    const double xs = x_s_spline(s_proj);
    const double xs_derv_1st = x_s_spline.deriv(1, s_proj);
    const double xs_derv_2nd = x_s_spline.deriv(2, s_proj);

    const double ys = y_s_spline(s_proj);
    const double ys_derv_1st = y_s_spline.deriv(1, s_proj);
    const double ys_derv_2nd = y_s_spline.deriv(2, s_proj);

    const double deriv_1st = xs * xs_derv_1st + ys * ys_derv_1st -
                             x0 * xs_derv_1st - y0 * ys_derv_1st;

    double deriv_2nd = xs_derv_1st * xs_derv_1st + xs * xs_derv_2nd +
                       ys_derv_1st * ys_derv_1st + ys * ys_derv_2nd -
                       x0 * xs_derv_2nd - y0 * ys_derv_2nd;

    if (deriv_2nd < 1e-3) {
      deriv_2nd = 1.0e-3;
    }

    const double d = -deriv_1st / deriv_2nd;
    double ds = 0.0;
    double alpha = 1.0;

    // line search
    for (size_t i = 0; i < 10; ++i) {
      ds = pnc::mathlib::Limit(d * alpha, half_length);
      s_proj = pnc::mathlib::Clamp(s_proj + ds, s_start + 0.01, s_end - 0.01);

      const double new_cost =
          std::hypot((x0 - x_s_spline(s_proj)), (y0 - y_s_spline(s_proj)));

      if (new_cost < cost) {
        cost = new_cost;
        break;
      }

      alpha = alpha * 0.5;
    }

    // terminate when tol achived
    if (std::fabs(ds) < tol_) {
      break;
    }
  }
  // cal the euclidean distance between the projection point and the current pos
  Eigen::Vector2d point_proj(x_s_spline(s_proj), y_s_spline(s_proj));
  const double dist_proj = (point_proj - x).norm();

  // first point of trajectory
  const Eigen::Vector2d first_pos(x_s_spline(s_start), y_s_spline(s_start));
  const double dist_first = (first_pos - x).norm();

  // last point of trajectory
  const Eigen::Vector2d last_pos(x_s_spline(s_end), y_s_spline(s_end));
  const double dist_last = (last_pos - x).norm();

  // projection point can also be first (probably) or last point
  if (dist_first < dist_proj && dist_first < dist_last) {
    point_proj = first_pos;
    s_proj = s_start;
  } else if (dist_last < dist_proj && dist_last < dist_first) {
    point_proj = last_pos;
    s_proj = s_end;
  }
}
void PreviewEgoPosisation(double tlc_to_line_threshold,
                          std::vector<double> &ego_box_predict_vector) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  ego_box_predict_vector.clear();
  ego_box_predict_vector.reserve(8);
  // 计算tlc秒后,后轴中心的坐标值
  double x_0 = 0.0;
  double y_0 = 0.0;
  double yaw_0 = 0.0;
  double dt = tlc_to_line_threshold;
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
  ego_box.input.length = GetContext.get_param()->ego_length;
  ego_box.input.width = GetContext.get_param()->ego_width;
  ego_box.input.x =
      ego_x + (GetContext.get_param()->origin_2_front_bumper -
               0.5F * ego_box.input.length) *
                  ego_box.input.cos_heading_angle;  // box几何中心的x坐标
  ego_box.input.y =
      ego_y + (GetContext.get_param()->origin_2_front_bumper -
               0.5F * ego_box.input.length) *
                  ego_box.input.sin_heading_angle;  // box几何中心的y坐标
  adas_function::BoxCornersCoordinateUpdate(&ego_box);
  ego_box_predict_vector.emplace_back(ego_box.state.fl_x);
  ego_box_predict_vector.emplace_back(ego_box.state.fl_y);
  ego_box_predict_vector.emplace_back(ego_box.state.fr_x);
  ego_box_predict_vector.emplace_back(ego_box.state.fr_y);
  ego_box_predict_vector.emplace_back(ego_box.state.rl_x);
  ego_box_predict_vector.emplace_back(ego_box.state.rl_y);
  ego_box_predict_vector.emplace_back(ego_box.state.rr_x);
  ego_box_predict_vector.emplace_back(ego_box.state.rr_y);
  return;
}
double LkasLineLeftIntervention(double tlc_to_line_threshold) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  // left_line_valid
  bool left_line_valid;
  if (((GetContext.mutable_road_info()->current_lane.left_line.line_type ==
        context::Enum_LineType::Enum_LineType_Dashed) ||
       (GetContext.mutable_road_info()->current_lane.left_line.line_type ==
        context::Enum_LineType::Enum_LineType_Solid)) &&
      (GetContext.mutable_road_info()->current_lane.left_line.valid == true)) {
    left_line_valid = true;
  } else {
    left_line_valid = false;
  }

  if (left_line_valid == false) {
    return 1.0;
  }

  // 计算tlc秒后,后轴中心的坐标值
  double x_0 = 0.0;
  double y_0 = 0.0;
  double yaw_0 = 0.0;
  double dt = tlc_to_line_threshold;
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
  ego_box.input.length = GetContext.get_param()->ego_length;
  ego_box.input.width = GetContext.get_param()->ego_width;
  ego_box.input.x =
      ego_x + (GetContext.get_param()->origin_2_front_bumper -
               0.5F * ego_box.input.length) *
                  ego_box.input.cos_heading_angle;  // box几何中心的x坐标
  ego_box.input.y =
      ego_y + (GetContext.get_param()->origin_2_front_bumper -
               0.5F * ego_box.input.length) *
                  ego_box.input.sin_heading_angle;  // box几何中心的y坐标
  adas_function::BoxCornersCoordinateUpdate(&ego_box);
  double preview_distance_x = ego_box.state.fl_x;
  double preview_distance_y = ego_box.state.fl_y;

  double line_width = GetContext.get_param()->lane_line_width;  // 道线的宽度,单位:m

  // 计算前轮处道线的横向坐标值
  Eigen::Vector2d pos_proj;
  Eigen::Vector2d current_pos = Eigen::Vector2d::Zero();
  current_pos.x() = preview_distance_x;
  current_pos.y() = preview_distance_y;
  double s_proj = 0.0;
  CalProjectionPointByNewtonIteration(
      GetContext.mutable_road_info()->current_lane.left_line.dx_s_spline_,
      GetContext.mutable_road_info()->current_lane.left_line.dy_s_spline_, 0,
      GetContext.mutable_road_info()->current_lane.left_line.end_s, current_pos,
      s_proj);
  double left_line_y =
      GetContext.mutable_road_info()->current_lane.left_line.dy_s_spline_(
          s_proj) -
      line_width * 0.5;
  return (left_line_y - preview_distance_y);
}

double LkasLineRightIntervention(double tlc_to_line_threshold) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  // right_line_valid
  bool right_line_valid;
  if (((GetContext.mutable_road_info()->current_lane.right_line.line_type ==
        context::Enum_LineType::Enum_LineType_Dashed) ||
       (GetContext.mutable_road_info()->current_lane.right_line.line_type ==
        context::Enum_LineType::Enum_LineType_Solid)) &&
      (GetContext.mutable_road_info()->current_lane.right_line.valid == true)) {
    right_line_valid = true;
  } else {
    right_line_valid = false;
  }

  if (right_line_valid == false) {
    return -1.0;
  }

  // 计算tlc秒后,后轴中心的坐标值
  double x_0 = 0.0;
  double y_0 = 0.0;
  double yaw_0 = 0.0;
  double dt = tlc_to_line_threshold;
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
  ego_box.input.length = GetContext.get_param()->ego_length;
  ego_box.input.width = GetContext.get_param()->ego_width;
  ego_box.input.x =
      ego_x + (GetContext.get_param()->origin_2_front_bumper -
               0.5F * ego_box.input.length) *
                  ego_box.input.cos_heading_angle;  // box几何中心的x坐标
  ego_box.input.y =
      ego_y + (GetContext.get_param()->origin_2_front_bumper -
               0.5F * ego_box.input.length) *
                  ego_box.input.sin_heading_angle;  // box几何中心的y坐标
  adas_function::BoxCornersCoordinateUpdate(&ego_box);
  double preview_distance_x = ego_box.state.fr_x;
  double preview_distance_y = ego_box.state.fr_y;

  double line_width = GetContext.get_param()->lane_line_width;  // 道线的宽度,单位:m

  // 计算前轮处道线的横向坐标值
  Eigen::Vector2d pos_proj;
  Eigen::Vector2d current_pos = Eigen::Vector2d::Zero();
  current_pos.x() = preview_distance_x;
  current_pos.y() = preview_distance_y;
  double s_proj = 0.0;
  CalProjectionPointByNewtonIteration(
      GetContext.mutable_road_info()->current_lane.right_line.dx_s_spline_,
      GetContext.mutable_road_info()->current_lane.right_line.dy_s_spline_, 0,
      GetContext.mutable_road_info()->current_lane.right_line.end_s,
      current_pos, s_proj);
  double right_line_y =
      GetContext.mutable_road_info()->current_lane.right_line.dy_s_spline_(
          s_proj) +
      line_width * 0.5;

  return (right_line_y - preview_distance_y);
}

double LkasRoadedgeLeftIntervention(double tlc_to_line_threshold,
                                    double roadedge_offset) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  // left_line_valid
  bool left_line_valid;
  if (GetContext.get_road_info()->current_lane.left_roadedge.valid == true) {
    left_line_valid = true;
  } else {
    left_line_valid = false;
  }

  if (left_line_valid == false) {
    return 1.0;
  }

  // 计算tlc秒后,后轴中心的坐标值
  double x_0 = 0.0;
  double y_0 = 0.0;
  double yaw_0 = 0.0;
  double dt = tlc_to_line_threshold;
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
  ego_box.input.length = GetContext.get_param()->ego_length;
  ego_box.input.width = GetContext.get_param()->ego_width;
  ego_box.input.x =
      ego_x + (GetContext.get_param()->origin_2_front_bumper -
               0.5F * ego_box.input.length) *
                  ego_box.input.cos_heading_angle;  // box几何中心的x坐标
  ego_box.input.y =
      ego_y + (GetContext.get_param()->origin_2_front_bumper -
               0.5F * ego_box.input.length) *
                  ego_box.input.sin_heading_angle;  // box几何中心的y坐标
  adas_function::BoxCornersCoordinateUpdate(&ego_box);
  double preview_distance_x = ego_box.state.fl_x;
  double preview_distance_y = ego_box.state.fl_y;

  // 计算前轮处道线的横向坐标值
  Eigen::Vector2d pos_proj;
  Eigen::Vector2d current_pos = Eigen::Vector2d::Zero();
  current_pos.x() = preview_distance_x;
  current_pos.y() = preview_distance_y;
  double s_proj = 0.0;
  CalProjectionPointByNewtonIteration(
      GetContext.mutable_road_info()->current_lane.left_roadedge.dx_s_spline_,
      GetContext.mutable_road_info()->current_lane.left_roadedge.dy_s_spline_,
      0, GetContext.mutable_road_info()->current_lane.left_roadedge.end_s,
      current_pos, s_proj);
  double left_line_y =
      GetContext.mutable_road_info()->current_lane.left_roadedge.dy_s_spline_(
          s_proj) -
      roadedge_offset;

  return (left_line_y - preview_distance_y);
  // if ((left_line_y - preview_distance_y) < 0.0) {
  //   return true;
  // } else {
  //   return false;
  // }
}

double LkasRoadedgeRightIntervention(double tlc_to_line_threshold,
                                     double roadedge_offset) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  // right_line_valid
  bool right_line_valid;
  if (GetContext.mutable_road_info()->current_lane.right_roadedge.valid ==
      true) {
    right_line_valid = true;
  } else {
    right_line_valid = false;
  }

  if (right_line_valid == false) {
    return -1.0;
  }

  // 计算tlc秒后,后轴中心的坐标值
  double x_0 = 0.0;
  double y_0 = 0.0;
  double yaw_0 = 0.0;
  double dt = tlc_to_line_threshold;
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
  ego_box.input.length = GetContext.get_param()->ego_length;
  ego_box.input.width = GetContext.get_param()->ego_width;
  ego_box.input.x =
      ego_x + (GetContext.get_param()->origin_2_front_bumper -
               0.5F * ego_box.input.length) *
                  ego_box.input.cos_heading_angle;  // box几何中心的x坐标
  ego_box.input.y =
      ego_y + (GetContext.get_param()->origin_2_front_bumper -
               0.5F * ego_box.input.length) *
                  ego_box.input.sin_heading_angle;  // box几何中心的y坐标
  adas_function::BoxCornersCoordinateUpdate(&ego_box);
  double preview_distance_x = ego_box.state.fr_x;
  double preview_distance_y = ego_box.state.fr_y;

  // 计算前轮处道线的横向坐标值
  Eigen::Vector2d pos_proj;
  Eigen::Vector2d current_pos = Eigen::Vector2d::Zero();
  current_pos.x() = preview_distance_x;
  current_pos.y() = preview_distance_y;
  double s_proj = 0.0;
  CalProjectionPointByNewtonIteration(
      GetContext.mutable_road_info()->current_lane.right_roadedge.dx_s_spline_,
      GetContext.mutable_road_info()->current_lane.right_roadedge.dy_s_spline_,
      0, GetContext.mutable_road_info()->current_lane.right_roadedge.end_s,
      current_pos, s_proj);
  double right_line_y =
      GetContext.mutable_road_info()->current_lane.right_roadedge.dy_s_spline_(
          s_proj) +
      roadedge_offset;

  return (right_line_y - preview_distance_y);
  // if ((right_line_y - preview_distance_y) > 0.0) {
  //   return true;
  // } else {
  //   return false;
  // }
}

void CalProjectionPointForLong(const pnc::mathlib::spline &x_s_spline,
                               const pnc::mathlib::spline &y_s_spline,
                               const double s_start, const double s_end,
                               const Eigen::Vector2d &x, double &s_proj) {
  double max_iter_ = 10;
  double tol_ = 1e-3;
  if (s_start > s_end) {
    return;
  }

  // newton iteration to calculate projection point
  const double half_length = (s_end - s_start) * 0.5;
  s_proj = (s_end + s_start) * 0.5;

  auto const &x0 = x.x();
  auto const &y0 = x.y();

  double cost =
      std::hypot((x0 - x_s_spline(s_proj)), (y0 - y_s_spline(s_proj)));

  // get s_proj
  for (size_t i = 0; i < max_iter_; ++i) {
    const double xs = x_s_spline(s_proj);
    const double xs_derv_1st = x_s_spline.deriv(1, s_proj);
    const double xs_derv_2nd = x_s_spline.deriv(2, s_proj);

    const double ys = y_s_spline(s_proj);
    const double ys_derv_1st = y_s_spline.deriv(1, s_proj);
    const double ys_derv_2nd = y_s_spline.deriv(2, s_proj);

    const double deriv_1st = xs * xs_derv_1st + ys * ys_derv_1st -
                             x0 * xs_derv_1st - y0 * ys_derv_1st;

    double deriv_2nd = xs_derv_1st * xs_derv_1st + xs * xs_derv_2nd +
                       ys_derv_1st * ys_derv_1st + ys * ys_derv_2nd -
                       x0 * xs_derv_2nd - y0 * ys_derv_2nd;

    if (deriv_2nd < 1e-3) {
      deriv_2nd = 1.0e-3;
    }

    const double d = -deriv_1st / deriv_2nd;
    double ds = 0.0;
    double alpha = 1.0;

    // line search
    for (size_t i = 0; i < 10; ++i) {
      ds = pnc::mathlib::Limit(d * alpha, half_length);
      s_proj = pnc::mathlib::Clamp(s_proj + ds, s_start + 0.01, s_end - 0.01);

      const double new_cost =
          std::hypot((x0 - x_s_spline(s_proj)), (y0 - y_s_spline(s_proj)));

      if (new_cost < cost) {
        cost = new_cost;
        break;
      }

      alpha = alpha * 0.5;
    }

    // terminate when tol achived
    if (std::fabs(ds) < tol_) {
      break;
    }
  }
  // cal the euclidean distance between the projection point and the current pos
  Eigen::Vector2d point_proj(x_s_spline(s_proj), y_s_spline(s_proj));
  const double dist_proj = (point_proj - x).norm();

  // first point of trajectory
  const Eigen::Vector2d first_pos(x_s_spline(s_start), y_s_spline(s_start));
  const double dist_first = (first_pos - x).norm();

  // last point of trajectory
  const Eigen::Vector2d last_pos(x_s_spline(s_end), y_s_spline(s_end));
  const double dist_last = (last_pos - x).norm();

  // projection point can also be first (probably) or last point
  if (dist_first < dist_proj && dist_first < dist_last) {
    point_proj = first_pos;
    s_proj = s_start;
  } else if (dist_last < dist_proj && dist_last < dist_first) {
    point_proj = last_pos;
    s_proj = s_end;
  }
}

std::vector<double> ObjCornersCalculate(
    const context::FusionObjExtractInfo &obj) {
  std::vector<double> obj_vec;
  obj_vec.clear();
  obj_vec.resize(8);
  double cal_x_tmp = 0.0;
  double cal_y_tmp = 0.0;
  cal_x_tmp = obj.relative_position_x + 0.5 * obj.length;
  cal_y_tmp = obj.relative_position_y + 0.5 * obj.width;
  obj_vec[0] = cal_x_tmp;
  obj_vec[1] = cal_y_tmp;
  cal_x_tmp = obj.relative_position_x + 0.5 * obj.length;
  cal_y_tmp = obj.relative_position_y - 0.5 * obj.width;
  obj_vec[2] = cal_x_tmp;
  obj_vec[3] = cal_y_tmp;
  cal_x_tmp = obj.relative_position_x - 0.5 * obj.length;
  cal_y_tmp = obj.relative_position_y + 0.5 * obj.width;
  obj_vec[4] = cal_x_tmp;
  obj_vec[5] = cal_y_tmp;
  cal_x_tmp = obj.relative_position_x - 0.5 * obj.length;
  cal_y_tmp = obj.relative_position_y - 0.5 * obj.width;
  obj_vec[6] = cal_x_tmp;
  obj_vec[7] = cal_y_tmp;
  return obj_vec;
}

}  // namespace adas_function