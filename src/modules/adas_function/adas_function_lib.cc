#include "adas_function_lib.h"

#include "adas_function_struct.h"

namespace adas_function {

uint16 uint16_bit[16] = {1,   2,   4,    8,    16,   32,   64,    128,
                         256, 512, 1024, 2048, 4096, 8192, 16384, 32768};

uint32 uint32_bit[32] = {
    1,         2,         4,          8,         16,       32,       64,
    128,       256,       512,        1024,      2048,     4096,     8192,
    16384,     32768,     65536,      131072,    262144,   524288,   1048576,
    2097152,   4194304,   8388608,    16777216,  33554432, 67108864, 134217728,
    268435456, 536870912, 1073741824, 2147483648};

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

  double line_width =
      GetContext.get_param()->lane_line_width;  // 道线的宽度,单位:m

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

  double line_width =
      GetContext.get_param()->lane_line_width;  // 道线的宽度,单位:m

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

// 定义最小二乘法确定车道线参数
double integer_power(const double x, int order) {
  double ret = 1.0;
  while (order > 0) {
    ret *= x;
    order--;
  }
  return ret;
}

void leastSquareFitting(const std::vector<double>& points_x_vec,
                        const std::vector<double>& points_y_vec,
                        const int order,
                        adas_function::context::LineInfo* line_info_ptr) {
  std::vector<double> ret(order + 1, 0.0);
  if (points_x_vec.empty() || points_x_vec.size() < order + 1) {
    line_info_ptr->c0 = ret[0];
    line_info_ptr->c1 = ret[1];
    line_info_ptr->c2 = ret[2];
    line_info_ptr->c3 = ret[3];
    return;
  }

  int points_num = points_x_vec.size();
  Eigen::MatrixXd X(points_num, order + 1);
  Eigen::MatrixXd Y(points_num, 1);

  for (size_t i = 0; i < points_num; ++i) {
    for (size_t j = 0; j < order + 1; ++j) {
      X(i, j) = integer_power(points_x_vec[i], j);
    }
    Y(i, 0) = points_y_vec[i];
  }

  Eigen::MatrixXd XT_X = X.transpose() * X;
  Eigen::VectorXd XT_Y = X.transpose() * Y;
  Eigen::VectorXd beta = XT_X.ldlt().solve(XT_Y);

  for (int i = 0; i <= order; ++i) {
    ret[i] = std::isnan(beta[i]) ? 0.0 : beta[i];
  }
  line_info_ptr->c0 = ret[0];
  line_info_ptr->c1 = ret[1];
  line_info_ptr->c2 = ret[2];
  line_info_ptr->c3 = ret[3];
}

void leastSquareFittingForRoadedge(
    const std::vector<double>& points_x_vec,
    const std::vector<double>& points_y_vec, const int order,
    adas_function::context::RoadedgeInfo* line_info_ptr) {
  std::vector<double> ret(order + 1, 0.0);
  if (points_x_vec.empty() || points_x_vec.size() < order + 1) {
    line_info_ptr->c0 = ret[0];
    line_info_ptr->c1 = ret[1];
    line_info_ptr->c2 = ret[2];
    line_info_ptr->c3 = ret[3];
    return;
  }

  int points_num = points_x_vec.size();
  Eigen::MatrixXd X(points_num, order + 1);
  Eigen::MatrixXd Y(points_num, 1);

  for (size_t i = 0; i < points_num; ++i) {
    for (size_t j = 0; j < order + 1; ++j) {
      X(i, j) = integer_power(points_x_vec[i], j);
    }
    Y(i, 0) = points_y_vec[i];
  }

  Eigen::MatrixXd XT_X = X.transpose() * X;
  Eigen::VectorXd XT_Y = X.transpose() * Y;
  Eigen::VectorXd beta = XT_X.ldlt().solve(XT_Y);

  for (int i = 0; i <= order; ++i) {
    ret[i] = std::isnan(beta[i]) ? 0.0 : beta[i];
  }
  line_info_ptr->c0 = ret[0];
  line_info_ptr->c1 = ret[1];
  line_info_ptr->c2 = ret[2];
  line_info_ptr->c3 = ret[3];
}

OdObjGroup GetOdObjGroup(const iflyauto::ObjectType type) {
  switch (type) {
    case iflyauto::ObjectType::OBJECT_TYPE_ADULT:
    case iflyauto::ObjectType::OBJECT_TYPE_CHILD:
    case iflyauto::ObjectType::OBJECT_TYPE_PEDESTRIAN:
    case iflyauto::ObjectType::OBJECT_TYPE_TRAFFIC_POLICE:
      return OdObjGroup::kPeople;
    case iflyauto::ObjectType::OBJECT_TYPE_COUPE:
    case iflyauto::ObjectType::OBJECT_TYPE_MINIBUS:
    case iflyauto::ObjectType::OBJECT_TYPE_VAN:
    case iflyauto::ObjectType::OBJECT_TYPE_BUS:
    case iflyauto::ObjectType::OBJECT_TYPE_TRUCK:
    case iflyauto::ObjectType::OBJECT_TYPE_TRAILER:
      return OdObjGroup::kCar;
    case iflyauto::ObjectType::OBJECT_TYPE_CYCLE_RIDING:
    case iflyauto::ObjectType::OBJECT_TYPE_MOTORCYCLE_RIDING:
    case iflyauto::ObjectType::OBJECT_TYPE_TRICYCLE_RIDING:
    case iflyauto::ObjectType::OBJECT_TYPE_BICYCLE:
    case iflyauto::ObjectType::OBJECT_TYPE_MOTORCYCLE:
    case iflyauto::ObjectType::OBJECT_TYPE_TRICYCLE:
      return OdObjGroup::kMotor;
    default:
      return OdObjGroup::kDefault;
  }
}

float32 GetEgoCurvature(float32 v, float32 yaw_rate, float32 steer_angle,
                        float32 steer_ratio, float32 wheel_base) {
  double curv_low_spd = steer_angle / steer_ratio / wheel_base;
  double curv_high_spd = yaw_rate / fmax(v, 0.1);

  double curv_low_spd_factor;
  double curv_knee_pt1_mps = 3.0;
  double curv_knee_pt2_mps = 6.0;
  if (v < curv_knee_pt1_mps) {
    curv_low_spd_factor = 1.0;
  } else if (v < curv_knee_pt2_mps) {
    curv_low_spd_factor =
        (curv_knee_pt2_mps - v) / (curv_knee_pt2_mps - curv_knee_pt1_mps);
  } else {
    curv_low_spd_factor = 0.0;
  }
  double irregular_yawrate_thr = 0.75;
  if (fabs(yaw_rate) > irregular_yawrate_thr) {
    curv_low_spd_factor = 1.0;
  }

  double curv = curv_low_spd_factor * curv_low_spd +
                (1.0 - curv_low_spd_factor) * curv_high_spd;

  if (fabs(curv) < 0.00001) {
    curv = 0.00001;
  } else {
    // do nothing
  }

  return curv;
}

int32 min_int32(int32 x, int32 y) { return (x < y) ? x : y; }
float32 min_float(float32 x, float32 y) { return (x < y) ? x : y; }
int32 max_int32(int32 x, int32 y) { return (x > y) ? x : y; }
float32 max_float(float32 x, float32 y) { return (x > y) ? x : y; }

/*MySinRad函数功能:
使用泰勒展开法替代<math.h>中的sin函数，解决计算量太大造成程序卡死问题
输入接口：
(1)angle_rad:角度 单位:rad 范围:-4pi 到 4pi
输出接口：
sin(angle_rad)
*/
float32 MySinRad(float32 angle_rad) {
  float32 x = angle_rad;
  float32 pi = 3.14159265358979323846F;
  float32 two_pi = 2.0F * pi;

  // 将x值等效至[-4.0*pi,4.0*pi]之间
  float32 two_pi_times;  // angle_rad/(2*pi)取整的结果
  two_pi_times = (int)(angle_rad / two_pi + 0.5);
  x = angle_rad - two_pi_times * two_pi;

  if ((x > 4.0 * pi) && (x < -4.0 * pi)) {
    return 0.0F;
  }

  if (isnan(x)) {
    return 0.0F;
  }

  // 将x值的范围控制在[0,2*pi]之间
  while (x > two_pi)  // 大于360度
  {
    x = x - two_pi;
  }
  while (x < 0.0F)  // 小于0度
  {
    x = x + two_pi;
  }

  // 泰勒展开公式(x有效输入范围为[-0.5*pi,0.5*pi])
  // sin x = x - x^3/3! + x^5/5! - x^7/7! +……+(-1)^(k-1)*(x^(2k-1))/(2k-1)!
  // cos x = 1 - x^2/2! + x^4/4! - x^6/6! +……+(-1)^(k)*(x^(2k))/(2k)!
  float32 result_sign = 1.0F;
  if ((x >= 0.0F) && (x <= (pi / 2.0F))) {
    //[0,90]deg
    result_sign = 1.0F;
  } else if (x <= (pi * 3.0F / 2.0F)) {
    //(90,270]deg
    // sin(pi+x) = -sin(x)
    x = x - pi;  //(-90,90]
    result_sign = -1.0F;
  } else {
    //(270,360]deg
    // sin(two_pi+x) = sin(x)
    x = x - two_pi;  //(-90,0]
    result_sign = 1.0F;
  }

  float32 x_2 = x * x;
  float32 x_3 = x_2 * x;
  float32 x_5 = x_3 * x_2;
  float32 x_7 = x_5 * x_2;
  float32 result = 0.0F;
  float32 result_temp = 0.0F;
  result_temp = x - x_3 / 6.0F + x_5 / 120.0F - x_7 / 5040.0F;
  result = result_temp * result_sign;
  return result;
}

/*MyCosRad函数功能:
使用泰勒展开法替代<math.h>中的cos函数，解决计算量太大造成程序卡死问题
输入接口：
(1)angle_rad:角度 单位:rad 范围:-4pi 到 4pi
输出接口：
sin(angle_rad)
*/
float32 MyCosRad(float32 angle_rad) {
  float32 x = angle_rad;
  float32 pi = 3.14159265358979323846F;
  float32 two_pi = 2.0F * pi;

  // 将x值等效至[-4.0*pi,4.0*pi]之间
  float32 two_pi_times;  // angle_rad/(2*pi)取整的结果
  two_pi_times = (int)(angle_rad / two_pi + 0.5);
  x = angle_rad - two_pi_times * two_pi;

  if ((x > 4.0 * pi) && (x < -4.0 * pi)) {
    return 0.0F;
  }

  if (isnan(x)) {
    return 0.0F;
  }

  // 将x值的范围控制在[0,2*pi]之间
  while (x > two_pi)  // 大于360度
  {
    x = x - two_pi;
  }
  while (x < 0.0F)  // 小于0度
  {
    x = x + two_pi;
  }

  // 泰勒展开公式(x有效输入范围为[-0.5*pi,0.5*pi])
  // sin x = x - x^3/3! + x^5/5! - x^7/7! +……+(-1)^(k-1)*(x^(2k-1))/(2k-1)!
  // cos x = 1 - x^2/2! + x^4/4! - x^6/6! +……+(-1)^(k)*(x^(2k))/(2k)!
  float32 result_sign = 1.0F;
  if ((x >= 0.0F) && (x <= (pi / 2.0F))) {
    //[0,90]deg
    result_sign = 1.0F;
  } else if (x <= (pi * 3.0F / 2.0F)) {
    //(90,270]deg
    // cos(pi+x) = -cos(x)
    x = x - pi;  //(-90,90]
    result_sign = -1.0F;
  } else {
    //(270,360]deg
    // cos(two_pi+x) = cos(x)
    x = x - two_pi;  //(-90,0]
    result_sign = 1.0F;
  }

  float32 x_2 = x * x;
  float32 x_4 = x_2 * x_2;
  float32 x_6 = x_4 * x_2;
  float32 x_8 = x_6 * x_2;
  float32 result = 0.0F;
  float32 result_temp = 0.0F;
  result_temp = 1.0F - x_2 / 2.0F + x_4 / 24.0F - x_6 / 720.0F + x_8 / 40320.0F;
  result = result_temp * result_sign;
  return result;
}

}  // namespace adas_function