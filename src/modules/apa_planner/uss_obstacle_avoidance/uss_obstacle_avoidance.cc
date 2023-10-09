#include "uss_obstacle_avoidance.h"

#include <stdio.h>

#include <cmath>
#include <complex>
#include <cstddef>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "math_lib.h"
#include "transform_lib.h"

// #define __USS_OA_DEBUG__

// car data
static const std::vector<double> car_vertex_x_vec = {
    3.187342,  3.424531,  3.593071,  3.593071,  3.424531,  3.187342,
    2.177994,  1.916421,  1.96496,   -0.476357, -0.798324, -0.879389,
    -0.879389, -0.798324, -0.476357, 1.96496,   1.916421,  2.177994};

static const std::vector<double> car_vertex_y_vec = {
    0.887956,  0.681712,         0.334651,  -0.334651, -0.681712,     -0.887956,
    -0.887956, -(1.06715 + 0.1), -0.887956, -0.887956, -0.706505,     -0.334845,
    0.334845,  0.706505,         0.887956,  0.887956,  1.06715 + 0.1, 0.887956};

// uss data
static const std::vector<double> uss_vertex_x_vec = {
    3.187342,  3.424531,  3.593071,  3.593071,  3.424531,  3.187342,
    -0.476357, -0.798324, -0.879389, -0.879389, -0.798324, -0.476357};

static const std::vector<double> uss_vertex_y_vec = {
    0.887956,  0.681712,  0.334651,  -0.334651, -0.681712, -0.887956,
    -0.887956, -0.706505, -0.334845, 0.334845,  0.706505,  0.887956};

static const std::vector<double> uss_normal_angle_deg_vec = {
    170.0, 130.0, 92.0,  88.0,  50.0,  8.0,
    352.0, 298.0, 275.0, 264.0, 242.0, 187.0};

static const double uss_scan_angle_deg = 60.0;

static const std::vector<size_t> wdis_index_front = {0, 9, 6, 3, 1, 11};
static const std::vector<size_t> wdis_index_back = {0, 1, 3, 6, 9, 11};

namespace planning {
void UssObstacleAvoidance::InitVertexData() {
  // init car vertex
  car_vertex_vec_.clear();
  car_vertex_vec_.reserve(car_vertex_x_vec.size());
  for (size_t i = 0; i < car_vertex_x_vec.size(); ++i) {
    car_vertex_vec_.emplace_back(
        Eigen::Vector2d(car_vertex_x_vec[i], car_vertex_y_vec[i]));
  }

  // init uss vertex
  uss_vertex_vec_.clear();
  uss_vertex_vec_.reserve(uss_vertex_x_vec.size());
  uss_normal_angle_vec_.clear();
  uss_normal_angle_vec_.reserve(uss_vertex_x_vec.size());

  for (size_t i = 0; i < uss_vertex_x_vec.size(); ++i) {
    uss_vertex_vec_.emplace_back(
        Eigen::Vector2d(uss_vertex_x_vec[i], uss_vertex_y_vec[i]));
    uss_normal_angle_vec_.emplace_back(
        pnc::mathlib::Deg2Rad(uss_normal_angle_deg_vec[i] - 90.0));
  }
}

// only for considerably small steering angle
void UssObstacleAvoidance::GenCarLine() {
  car_arc_vec_.clear();
  car_arc_vec_.reserve(car_vertex_vec_.size());

  for (const auto &car_vertex : car_vertex_vec_) {
    Arc car_arc;
    car_arc.circle_info.is_circle = false;
    car_arc.pA = car_vertex;
    car_arc.pB = car_arc.pA;

    if (!reverse_flag_) {
      car_arc.pB.x() += param_.max_remain_dist;
    } else {
      car_arc.pB.x() -= param_.max_remain_dist;
    }

    car_arc.circle_info.is_circle = false;

    car_arc_vec_.emplace_back(std::move(car_arc));
  }
}

// only for non small steering angle
void UssObstacleAvoidance::GenCarArc() {
  car_arc_vec_.clear();
  car_arc_vec_.reserve(car_vertex_vec_.size());

  double sign = 1.0;
  if ((!reverse_flag_ && steer_angle_ > 0.0) ||
      (reverse_flag_ && steer_angle_ < 0.0)) {
    sign = 1.0;
  } else {
    sign = -1.0;
  }

  const double rot_angle =
      param_.max_remain_dist / rear_axle_center_radius_ * sign;

  const auto rot_m = pnc::transform::Angle2Rotm2d(rot_angle);

  // std::cout << "sign = " << sign << std::endl;
  // std::cout << "rot_angle = " << rot_angle * 57.3 << std::endl;

  // int i = 0;
  for (const auto &car_vertex : car_vertex_vec_) {
    Arc car_arc;
    car_arc.circle_info.is_circle = true;
    car_arc.circle_info.center = turning_center_;
    car_arc.circle_info.radius = (car_vertex - turning_center_).norm();
    car_arc.pA = car_vertex;
    const auto vec_OA = car_arc.pA - turning_center_;
    car_arc.pB = turning_center_ + rot_m * vec_OA;
    // std::cout << "-------------------- car_arc info --------------------"
    //           << std::endl;
    // std::cout << "car_arc.pA = \n"
    //           << car_arc.pA << std::endl
    //           << "car_arc.pB = \n"
    //           << car_arc.pB << std::endl
    //           << "car_arc.radius = " << car_arc.circle_info.radius
    //           << std::endl
    //           << "car_arc.center = \n"
    //           << turning_center_ << std::endl
    //           << "(car_arc.pA - turning_center_) = "
    //           << (car_arc.pA - turning_center_).norm() << std::endl
    //           << "(car_arc.pB - turning_center_) = "
    //           << (car_arc.pB - turning_center_).norm() << std::endl;

    car_arc_vec_.emplace_back(std::move(car_arc));
  }
}

void UssObstacleAvoidance::GenUssArc() {
  uss_arc_vec_.clear();
  uss_arc_vec_.reserve(uss_vertex_vec_.size());

  const auto rot_m_normal = pnc::transform::Angle2Rotm2d(
      pnc::mathlib::Deg2Rad(uss_scan_angle_deg * 0.5));

  const auto rot_m_large = pnc::transform::Angle2Rotm2d(
      pnc::mathlib::Deg2Rad(1.4 * uss_scan_angle_deg * 0.5));

  Eigen::Matrix2d rot_m;

  for (size_t i = 0; i < uss_vertex_vec_.size(); ++i) {
    if (i == 1 || i == 4 || i == 7 || i == 10) {
      rot_m = rot_m_large;
    } else {
      rot_m = rot_m_normal;
    }

    Arc uss_arc;
    const auto &pO = uss_vertex_vec_[i];
    if (uss_raw_dist_vec_[i] > param_.max_remain_dist) {
      uss_arc.is_considered = false;
      uss_arc_vec_.emplace_back(std::move(uss_arc));
      continue;
    }

    uss_arc.is_considered = true;
    uss_arc.circle_info.is_circle = true;
    uss_arc.circle_info.center = pO;
    uss_arc.circle_info.radius = uss_raw_dist_vec_[i];

    const auto &theta_uss = uss_normal_angle_vec_[i];
    const auto center_of_arc =
        pO + uss_raw_dist_vec_[i] *
                 Eigen::Vector2d(std::cos(theta_uss), std::sin(theta_uss));

    const auto vec_OC = center_of_arc - pO;

    uss_arc.pA = rot_m.transpose() * vec_OC + pO;
    uss_arc.pB = rot_m * vec_OC + pO;

    // if (i == 8) {
    //   std::cout << "-------------------- uss_arc info --------------------"
    //             << std::endl;
    //   std::cout << "uss_arc.pA = \n"
    //             << uss_arc.pA << std::endl
    //             << "uss_arc.pB = \n"
    //             << uss_arc.pB << std::endl
    //             << "uss_arc.radius = " << uss_arc.circle_info.radius
    //             << std::endl
    //             << "uss_arc.center = \n"
    //             << uss_arc.circle_info.center << std::endl
    //             << "center_of_arc = \n"
    //             << center_of_arc << std::endl
    //             << "(uss_arc.pA - uss_arc.circle_info.center) = "
    //             << (uss_arc.pA - uss_arc.circle_info.center).norm() <<
    //             std::endl
    //             << "(uss_arc.pB - uss_arc.circle_info.center) = "
    //             << (uss_arc.pB - uss_arc.circle_info.center).norm()
    //             << std::endl;
    // }

    uss_arc_vec_.emplace_back(std::move(uss_arc));
  }
}

void UssObstacleAvoidance::Preprocess() {
  // process some measuremnts
  steer_angle_ =
      local_view_ptr_->vehicle_service_output_info.steering_wheel_angle();

  uint8_t gear_cmd = Common::GearCommandValue::GEAR_COMMAND_VALUE_NONE;
  if (planning_output_->gear_command().available()) {
    gear_cmd = planning_output_->gear_command().gear_command_value();
  }
  reverse_flag_ =
      (gear_cmd == Common::GearCommandValue::GEAR_COMMAND_VALUE_REVERSE);

  // reverse_flag_ =
  //     (local_view_ptr_->vehicle_service_output_info.shift_lever_state() ==
  //     1);

  // std::cout << "reverse_flag = " << reverse_flag_ << std::endl;

  heading_ = local_view_ptr_->localization_estimate.pose().heading();

  // set uss raw dist data
  uss_raw_dist_vec_.clear();
  uss_raw_dist_vec_.reserve(uss_vertex_x_vec.size());
  const auto &upa_dis_info_buf =
      local_view_ptr_->uss_wave_info.upa_dis_info_buf();

  // front uss
  for (size_t i = 0; i < wdis_index_front.size(); ++i) {
    uss_raw_dist_vec_.emplace_back(
        upa_dis_info_buf[0].wdis(wdis_index_front[i]).wdis_value(0));
  }

  // back uss
  for (size_t i = 0; i < wdis_index_back.size(); ++i) {
    uss_raw_dist_vec_.emplace_back(
        upa_dis_info_buf[1].wdis(wdis_index_back[i]).wdis_value(0));
  }
}

const bool UssObstacleAvoidance::CheckTwoCircleIntersection(
    const Circle &c1, const Circle &c2) const {
  const double d = (c1.center - c2.center).norm();
  if (d < c1.radius + c2.radius &&
      d > std::abs(c1.radius - c2.radius)) {  // maybe need some buffer here
    return true;
  } else {
    return false;
  }
}

const std::pair<Eigen::Vector2d, Eigen::Vector2d>
UssObstacleAvoidance::GetTwoCircleIntersection(const Circle &c1,
                                               const Circle &c2) const {
  const double d = (c1.center - c2.center).norm();

  // TODO: should consider very small d

  const double r1 = c1.radius;
  const double r2 = c2.radius;
  const double a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
  const double h = sqrt(r1 * r1 - a * a);

  const double x3 = c1.center.x() + a * (c2.center.x() - c1.center.x()) / d;
  const double y3 = c1.center.y() + a * (c2.center.y() - c1.center.y()) / d;

  std::pair<Eigen::Vector2d, Eigen::Vector2d> intersects;
  intersects.first =
      Eigen::Vector2d(x3 + h * (c2.center.y() - c1.center.y()) / d,
                      y3 - h * (c2.center.x() - c1.center.x()) / d);

  intersects.second =
      Eigen::Vector2d(x3 - h * (c2.center.y() - c1.center.y()) / d,
                      y3 + h * (c2.center.x() - c1.center.x()) / d);

  return intersects;
}

const bool UssObstacleAvoidance::CheckPointLiesOnArc(
    const Arc &arc, const Eigen::Vector2d &pC) const {
  const auto &pO = arc.circle_info.center;
  const auto v_OA = arc.pA - pO;
  const auto v_OB = arc.pB - pO;
  const auto v_OC = pC - pO;

  const auto norm_v_OA = v_OA.norm();
  const auto norm_v_OB = v_OB.norm();
  const auto norm_v_OC = v_OC.norm();

  const auto cos_AOC = v_OA.dot(v_OC) / (norm_v_OA * norm_v_OC);
  const auto cos_BOC = v_OB.dot(v_OC) / (norm_v_OB * norm_v_OC);
  const auto cos_AOB = v_OA.dot(v_OB) / (norm_v_OA * norm_v_OB);

  return (cos_AOC > cos_AOB && cos_BOC > cos_AOB);
}

const bool UssObstacleAvoidance::GetTwoArcIntersection(
    Eigen::Vector2d &intersection, const Arc &arc1, const Arc &arc2) const {
  // step 1: consider intersection between two circles rather than arcs
  if (!CheckTwoCircleIntersection(arc1.circle_info, arc2.circle_info)) {
    return false;
  }

  const auto intersection_pair =
      GetTwoCircleIntersection(arc1.circle_info, arc2.circle_info);

  // step 2: consider two arcs, assume that just one intersection point
  if (CheckPointLiesOnArc(arc1, intersection_pair.first) &&
      CheckPointLiesOnArc(arc2, intersection_pair.first)) {
    intersection = intersection_pair.first;
    return true;
  } else if (CheckPointLiesOnArc(arc1, intersection_pair.second) &&
             CheckPointLiesOnArc(arc2, intersection_pair.second)) {
    intersection = intersection_pair.second;
    return true;
  } else {
    return false;
  }
}

const bool UssObstacleAvoidance::GetArcLineIntersection(
    Eigen::Vector2d &intersection, const Arc &arc1, const Arc &line2) const {
  // use pA and pB of arc to fake line
  const auto &circle_info = arc1.circle_info;

  // const auto AB = line2.pB - line2.pA;
  // const Eigen::Vector2d OA(line2.pA.x() - circle_info.center.x(),
  //                          line2.pA.y() - circle_info.center.y());
  // // |OA + lambda * AB|^2 = r^2 means that line segment AB intersects at
  // circle
  // // OA·OA + 2*OA·AB*lambda + AB·AB*lambda^2 = r^2
  // const auto a = AB.dot(AB);
  // const auto b = (2 * OA).dot(AB);
  // const auto c = OA.dot(OA) - pow(circle_info.radius, 2);

  const auto dx = line2.pB.x() - line2.pA.x();
  const auto dy = line2.pB.y() - line2.pA.y();
  const auto A = dx * dx + dy * dy;

  const auto B = 2.0 * (dx * (line2.pA.x() - circle_info.center.x()) +
                        dy * (line2.pA.y() - circle_info.center.y()));

  const auto C = pow(circle_info.center.x() - line2.pA.x(), 2) +
                 pow(circle_info.center.y() - line2.pA.y(), 2) -
                 pow(circle_info.radius, 2);

  const auto delta = B * B - 4.0 * A * C;

  if (delta < 0) {
    return false;
  } else {
    const auto lambda1 = (-B + sqrt(delta)) / (2.0 * A);
    const auto lambda2 = (-B - sqrt(delta)) / (2.0 * A);

    // note that 0.0 < lambda < 1.0 forces intersection to lied on line
    if (pnc::mathlib::IsInBound(lambda1, 0.0, 1.0) ||
        pnc::mathlib::IsInBound(lambda2, 0.0, 1.0)) {
      intersection = lambda1 * line2.pB + (1.0 - lambda1) * line2.pA;
      if (CheckPointLiesOnArc(arc1, intersection)) {
        return true;
      } else {
        intersection = lambda2 * line2.pB + (1.0 - lambda2) * line2.pA;
        if (CheckPointLiesOnArc(arc1, intersection)) {
          return true;
        }
      }
    } else {
      return false;
    }
  }

  return false;
}

void UssObstacleAvoidance::CalRemainDist() {
  remain_dist_ = param_.max_remain_dist;
  double dist = param_.max_remain_dist;
  for (size_t i = 0; i < car_arc_vec_.size(); ++i) {
    for (size_t j = 0; j < uss_arc_vec_.size(); ++j) {
      // skip uss arc for unconsidered
      if (!uss_arc_vec_[j].is_considered) {
        continue;
      }

      Eigen::Vector2d intersection;
      if (car_point_mode_ == LINE_MODE) {
        if (GetArcLineIntersection(intersection, uss_arc_vec_[j],
                                   car_arc_vec_[i])) {
          dist = (car_arc_vec_[i].pA - intersection).norm();
        }
      } else {
        if (GetTwoArcIntersection(intersection, uss_arc_vec_[j],
                                  car_arc_vec_[i])) {
          const auto angle = pnc::transform::GetAngleFromTwoVec(
              car_arc_vec_[i].pA - car_arc_vec_[i].circle_info.center,
              intersection - car_arc_vec_[i].circle_info.center);

          dist = rear_axle_center_radius_ * angle;
        }
      }

      // printf("dist[%ld][%ld] = %.2f\n", i, j, dist);
      if (dist < remain_dist_) {
        remain_dist_ = dist;
        min_dist_car_arc_index_ = i;
        min_dist_uss_arc_index_ = j;
      }
    }
  }
}

bool UssObstacleAvoidance::Update(
    PlanningOutput::PlanningOutput *const planning_output) {
  // update planning output
  planning_output_ = planning_output;

  // preprocess, get some measurement and construct arcs
  Preprocess();

  // if (!reverse_flag_) {
  //   std::cout << "reverse_flag OFF!" << std::endl;
  // } else {
  //   std::cout << "reverse_flag ON!" << std::endl;
  // }

  // line & arc or two arcs, depended on radiuas (i.e. steer angle)
  if (std::abs(steer_angle_) <
      pnc::mathlib::Deg2Rad(param_.arc_line_shift_steer_angle_deg)) {
    // std::cout << "LINE_MODE!" << std::endl;

    // small steer angle results in line & arc
    car_point_mode_ = LINE_MODE;
    rear_axle_center_radius_ = 1.0e4;
    turning_center_.setZero();
    GenCarLine();
  } else {
    // std::cout << "ARC_MODE!" << std::endl;

    // large steer angle results in two arcs
    car_point_mode_ = ARC_MODE;
    const auto wheel_angle_norm = std::abs(steer_angle_ / param_.steer_ratio);
    rear_axle_center_radius_ = 1.0 / (param_.c1 * std::tan(wheel_angle_norm));

    turning_center_.setZero();

    if (steer_angle_ > 0.0) {
      turning_center_.y() = rear_axle_center_radius_;
    } else {
      turning_center_.y() = -rear_axle_center_radius_;
    }

    // std::cout << "turning_center = \n" << turning_center_ << std::endl;

    GenCarArc();
  }

  // update Uss arc all the time
  GenUssArc();

  // calculate remaining distance
  CalRemainDist();

  return true;
}

}  // namespace planning