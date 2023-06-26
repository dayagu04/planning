#include "diagonal/diagonal_in_geometry_plan.h"

#include <math.h>
#include <limits>

#include "common/apa_cos_sin.h"
#include "common/apa_utils.h"
#include "common/planning_log_helper.h"
#include "log_glog.h"
#include "math/box2d.h"
#include "math/math_utils.h"

namespace planning {
namespace apa_planner {

using planning::planning_math::Box2d;
using planning::planning_math::LineSegment2d;
using planning::planning_math::Polygon2d;
using planning::planning_math::Vec2d;

namespace {
constexpr double kEps = 1e-6;
constexpr double kYawStepRatio = 0.2;
constexpr double kDefaultThetaStep = 0.032;
constexpr double kMinThetaStep = 0.01;
constexpr double kMinSegmentLen = 0.3;
constexpr double kLatBuffer = 0.1;
constexpr double kLonBuffer = 0.1;
constexpr double kLonReverseBuffer = 0.0;
constexpr double kTargetXStep = 0.01;
constexpr int kTargetXNum = 11;
constexpr int kHalfTargetXNum = kTargetXNum / 2;
constexpr double kMaxXOffset = kTargetXStep * kHalfTargetXNum;
constexpr double kMaxXOffsetInDE3 = 0.5;
constexpr double kMaxXOffsetInCD2 = 0.15;
constexpr double kMaxYOffset = 0.2;
constexpr double kMaxThetaDiff = 0.02;
constexpr double kMaxYBackwardPos = -2.5;
constexpr double kSegmentLengthWeight = 1.0;
constexpr double kPointxBiasWeight = 20.0;
constexpr double kRadiusWeight = 1.0;
constexpr double kEndThetaWeight = 100.0;
constexpr double kSegmentCost = 10000.0;
constexpr double kBCThetaDiffCost = 10.0;
constexpr double kPointDThetaCost = 10.0;
constexpr double kDELengthWeight = 100;
constexpr double kDEStandardLength = 1.0;
}  // namespace

bool DiagonalInGeometryPlan::CheckSlotOpenSideWrong(
    const PlanningPoint &start_point) {
  bool side_wrong = false;
  if (slot_sign_ * start_point.y < 0) {
    side_wrong = true;
  }
  if (std::fabs(start_point.theta) > M_PI_4) {
    side_wrong = true;
  }
  return side_wrong;
}

bool DiagonalInGeometryPlan::ABSegment(const PlanningPoint &point_a,
                                       bool is_start, bool is_rough_calc,
                                       DiagonalSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  if (CheckSlotOpenSideWrong(point_a)) {
    AERROR << "open side wrong";
    return false;
  }

  const double rear_axle_over_dis = 3.0;
  const double max_ab_len = 7.0;
  const double ab_len =
      planning_math::Clamp(max_ab_len, 0.0, rear_axle_over_dis - point_a.x);
  const double min_l_step = 0.2;
  double max_l_step = 4.0;
  int sparse_step_num = 0;
  const double sparse_len = std::fmax(-point_a.x, 0.0);
  if (sparse_len > 0.0) {
    sparse_step_num = sparse_len / max_l_step + 1;
    max_l_step = sparse_len / sparse_step_num;
  }
  const int dense_step_num = (ab_len - sparse_len) / min_l_step;
  const int point_num = std::max(sparse_step_num + dense_step_num, 1);
  AINFO << "sparse_step_num:" << sparse_step_num
        << ", dense_step_num:" << dense_step_num;

  const double cos_point_a_theta = apa_cos(point_a.theta);
  const double sin_point_a_theta = apa_sin(point_a.theta);

  PlanningPoint point_b;
  size_t collide_cnt = 0;
  size_t bc_fail_cnt = 0;
  size_t total_cost_nan_cnt = 0;
  const double front_buffer = kLonBuffer;
  const double rear_buffer = 0.0;
  const double lat_buffer = kLatBuffer;
  Polygon2d init_ego_polygon = std::move(ConstructVehiclePolygonWithBuffer(
      point_a, front_buffer, rear_buffer, lat_buffer));
  for (int i = 0; i < point_num; ++i) {
    double l_ab = 0.0;
    if (i <= sparse_step_num) {
      l_ab = max_l_step * i;
    } else {
      l_ab = (i - sparse_step_num) * min_l_step + sparse_len;
    }
    point_b.x = point_a.x + l_ab * cos_point_a_theta;
    point_b.y = point_a.y + l_ab * sin_point_a_theta;
    point_b.theta = point_a.theta;

    if (CollideWithObjectsByPolygon(point_a, point_b, init_ego_polygon)) {
      ++collide_cnt;
      break;
    }

    DiagonalSegmentsInfo tmp_segments_info;
    if (!BCSegment(point_b, l_ab, false, is_rough_calc, &tmp_segments_info)) {
      ++bc_fail_cnt;
      continue;
    }

    if (std::isinf(tmp_segments_info.total_cost)) {
      ++total_cost_nan_cnt;
      continue;
    }

    const double total_cost = tmp_segments_info.total_cost;
    if (std::isinf(total_cost)) {
      continue;
    }
    if (total_cost < segments_info->total_cost) {
      segments_info->total_cost = total_cost;

      // ab
      segments_info->opt_point_b = point_b;

      // bc
      segments_info->opt_radius_bc = tmp_segments_info.opt_radius_bc;
      segments_info->opt_point_c = tmp_segments_info.opt_point_c;

      // cd
      segments_info->opt_radius_cd = tmp_segments_info.opt_radius_cd;
      segments_info->opt_point_d = tmp_segments_info.opt_point_d;

      // de
      segments_info->opt_radius_de = tmp_segments_info.opt_radius_de;
      segments_info->opt_point_e = tmp_segments_info.opt_point_e;

      // ef
      segments_info->opt_radius_ef = tmp_segments_info.opt_radius_ef;
      segments_info->opt_point_f = tmp_segments_info.opt_point_f;

      if (is_rough_calc) {
        return true;
      }
    }
  }

  AINFO << "total_cnt:" << point_num << ", collide_cnt:" << collide_cnt
        << ", bc_fail_cnt:" << bc_fail_cnt
        << ", total_cost_nan_cnt:" << total_cost_nan_cnt;

  if (std::isinf(segments_info->total_cost)) {
    return false;
  }

  return true;
}

bool DiagonalInGeometryPlan::ReverseABSegment(
    const PlanningPoint &point_a, bool is_start, bool is_rough_calc,
    DiagonalSegmentsInfo *segments_info) {
  AINFO << "ReverseABSegment start!!!";
  segments_info->total_cost = std::numeric_limits<double>::infinity();
  if (CheckSlotOpenSideWrong(point_a)) {
    AERROR << "open side wrong";
    return false;
  }
  const double min_l_step = 0.2;
  double max_l_step = 2.0;
  int sparse_step_num = 0;
  const double sparse_len = std::fmax(point_a.x - 3.0, 0.0);
  if (sparse_len > 0.0) {
    sparse_step_num = sparse_len / max_l_step + 1;
    max_l_step = sparse_len / sparse_step_num;
  }
  const double slot_with = 1.2;
  const int dense_step_num = (point_a.x - sparse_len + slot_with) / min_l_step;
  const int point_num = std::max(sparse_step_num + dense_step_num, 1);
  AINFO << "sparse_step_num:" << sparse_step_num
        << ", dense_step_num:" << dense_step_num;

  const double cos_point_a_theta = apa_cos(point_a.theta);
  const double sin_point_a_theta = apa_sin(point_a.theta);

  PlanningPoint point_b;
  size_t collide_cnt = 0;
  size_t cd_fail_cnt = 0;
  size_t total_cost_nan_cnt = 0;
  const double front_buffer = 0.0;
  const double rear_buffer = kLonBuffer;
  const double lat_buffer = kLatBuffer;
  Polygon2d init_ego_polygon = std::move(ConstructVehiclePolygonWithBuffer(
      point_a, front_buffer, rear_buffer, lat_buffer));

  for (int i = 0; i < point_num; ++i) {
    double l_ab = 0.0;
    if (i <= sparse_step_num) {
      l_ab = i * max_l_step;
    } else {
      l_ab = (i - sparse_step_num) * min_l_step + sparse_len;
    }
    point_b.x = point_a.x - l_ab * cos_point_a_theta;
    point_b.y = point_a.y - l_ab * sin_point_a_theta;
    point_b.theta = point_a.theta;
    if (CollideWithObjectsByPolygon(point_a, point_b, init_ego_polygon)) {
      ++collide_cnt;
      break;
    }
    DiagonalSegmentsInfo tmp_segments_info;
    if (!CDSegment(point_b, false, is_rough_calc, &tmp_segments_info)) {
      ++cd_fail_cnt;
      continue;
    }
    const double r_len_ab = std::fabs(point_b.x - point_a.x);
    const double len_cost = CalSegmentLengthCost(r_len_ab);
    const double total_cost = len_cost + tmp_segments_info.total_cost;
    if (std::isinf(total_cost)) {
      ++total_cost_nan_cnt;
      continue;
    }

    if (total_cost < segments_info->total_cost) {
      segments_info->total_cost = total_cost;

      // r_ac
      segments_info->opt_point_c = point_b;

      // cd
      segments_info->opt_radius_cd = tmp_segments_info.opt_radius_cd;
      segments_info->opt_point_d = tmp_segments_info.opt_point_d;

      // de
      segments_info->opt_radius_de = tmp_segments_info.opt_radius_de;
      segments_info->opt_point_e = tmp_segments_info.opt_point_e;

      // ef
      segments_info->opt_radius_ef = tmp_segments_info.opt_radius_ef;
      segments_info->opt_point_f = tmp_segments_info.opt_point_f;

      if (is_rough_calc) {
        return true;
      }
    }
  }
  AINFO << "totoal_cnt:" << point_num << ", collide_cnt:" << collide_cnt
        << ", bc_fail_cnt:" << cd_fail_cnt
        << ", total_cost_nan_cnt:" << total_cost_nan_cnt;
  if (std::isinf(segments_info->total_cost)) {
    return false;
  }
  return true;
}
bool DiagonalInGeometryPlan::BCSegment(const PlanningPoint &point_b,
                                       double len_ab, bool is_start,
                                       bool is_rough_calc,
                                       DiagonalSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  if (point_b.theta * slot_sign_ >= M_PI_2) {
    return false;
  }

  const double yaw_step = kDefaultThetaStep * slot_sign_;
  const double max_theta_diff = M_PI_4 * slot_sign_;
  const int size_step =
      std::max(static_cast<int>(max_theta_diff / yaw_step), 1);

  const double cos_point_b_theta = apa_cos(point_b.theta);
  const double sin_point_b_theta = apa_sin(point_b.theta);

  PlanningPoint point_c;
  size_t collide_cnt = 0;
  size_t cd_fail_cnt = 0;
  size_t total_cost_nan_cnt = 0;
  const double front_buffer = kLonBuffer;
  const double rear_buffer = 0.0;
  const double lat_buffer = kLatBuffer;
  Polygon2d init_ego_polygon = std::move(ConstructVehiclePolygonWithBuffer(
      point_b, front_buffer, rear_buffer, lat_buffer));
  for (int i = 0; i < size_step; ++i) {
    point_c.theta = point_b.theta + yaw_step * i;
    point_c.x = point_b.x + slot_sign_ * min_turn_radius_ *
                                (apa_sin(point_c.theta) - sin_point_b_theta);
    point_c.y = point_b.y + slot_sign_ * min_turn_radius_ *
                                (cos_point_b_theta - apa_cos(point_c.theta));

    if (CollideWithObjectsByPolygon(point_b, point_c, init_ego_polygon)) {
      ++collide_cnt;
      break;
    }

    DiagonalSegmentsInfo tmp_segments_info;
    if (!CDSegment(point_c, false, is_rough_calc, &tmp_segments_info)) {
      ++cd_fail_cnt;
      continue;
    }

    const double len_bc =
        std::fabs(point_b.theta - point_c.theta) * min_turn_radius_;
    const double len_cost = CalSegmentLengthCost(len_bc + len_ab);
    const double radius_cost = CalRadiusCost(min_turn_radius_, len_bc);
    const double theta_diff = std::fabs(point_b.theta - point_c.theta);
    const double theta_diff_cost = CalBCThetaDiffCost(theta_diff);
    const double total_cost =
        len_cost + radius_cost + theta_diff_cost + tmp_segments_info.total_cost;
    if (std::isinf(total_cost)) {
      ++total_cost_nan_cnt;
      continue;
    }

    if (total_cost < segments_info->total_cost) {
      segments_info->total_cost = total_cost;

      // bc
      segments_info->opt_radius_bc = min_turn_radius_;
      segments_info->opt_point_c = point_c;

      // cd
      segments_info->opt_radius_cd = tmp_segments_info.opt_radius_cd;
      segments_info->opt_point_d = tmp_segments_info.opt_point_d;

      // de
      segments_info->opt_radius_de = tmp_segments_info.opt_radius_de;
      segments_info->opt_point_e = tmp_segments_info.opt_point_e;

      // ef
      segments_info->opt_radius_ef = tmp_segments_info.opt_radius_ef;
      segments_info->opt_point_f = tmp_segments_info.opt_point_f;

      if (is_rough_calc) {
        return true;
      }
    }
  }

  AINFO << "total_cnt:" << size_step << ", collide_cnt:" << collide_cnt
        << ", cd_fail_cnt:" << cd_fail_cnt
        << ", total_cost_nan_cnt:" << total_cost_nan_cnt;

  if (std::isinf(segments_info->total_cost)) {
    return false;
  }

  return true;
}

bool DiagonalInGeometryPlan::CDSegment(const PlanningPoint &point_c,
                                       bool is_start, bool is_rough_calc,
                                       DiagonalSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  DiagonalSegmentsInfo tmp_segments_info;
  if (CD1Segment(point_c, is_start, is_rough_calc, &tmp_segments_info)) {
    segments_info->total_cost = tmp_segments_info.total_cost;
    segments_info->opt_radius_cd = tmp_segments_info.opt_radius_cd;
    segments_info->opt_point_d = tmp_segments_info.opt_point_d;
    // AINFO << "cd1 success";
    return true;
  }

  if (CD2Segment(point_c, is_start, is_rough_calc, &tmp_segments_info)) {
    segments_info->total_cost = tmp_segments_info.total_cost + kSegmentCost;
    segments_info->opt_radius_cd = tmp_segments_info.opt_radius_cd;
    segments_info->opt_point_d = tmp_segments_info.opt_point_d;
    // AINFO << "cd2 success";
    return true;
  }

  if (CD3Segment(point_c, is_start, is_rough_calc, &tmp_segments_info)) {
    segments_info->total_cost =
        tmp_segments_info.total_cost + 2.0 * kSegmentCost;
    segments_info->opt_radius_cd = tmp_segments_info.opt_radius_cd;
    segments_info->opt_point_d = tmp_segments_info.opt_point_d;
    // AINFO << "cd3 success";
    return true;
  }

  return false;
}

bool DiagonalInGeometryPlan::CD1Segment(const PlanningPoint &point_c,
                                        bool is_start, bool is_rough_calc,
                                        DiagonalSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  if (is_start &&
      std::fabs(point_c.theta - target_point_.theta) <= kMaxThetaDiff &&
      std::fabs(point_c.x - target_point_.x) <= kMaxYOffset) {
    segments_info->total_cost = 0.0;
    segments_info->opt_point_d.x = point_c.x;
    segments_info->opt_point_d.y = target_point_.y;
    segments_info->opt_point_d.theta = target_point_.theta;
    segments_info->opt_radius_cd = std::numeric_limits<double>::infinity();
    return true;
  }

  const double sin_point_c_theta = apa_sin(point_c.theta);
  const double cos_point_c_theta = apa_cos(point_c.theta);

  PlanningPoint point_d;
  point_d.theta = target_point_.theta;
  size_t radius_cnt1 = 0;
  size_t dy_cnt1 = 0;
  size_t collide_cnt1 = 0;
  const double front_buffer = is_start ? kLonReverseBuffer : 0.0;
  const double rear_buffer = kLonBuffer;
  const double lat_buffer = kLatBuffer;
  Polygon2d init_ego_polygon = std::move(ConstructVehiclePolygonWithBuffer(
      point_c, front_buffer, rear_buffer, lat_buffer));
  // const std::vector<double> target_x_vec =
  //     is_start ? target_x_vec_ : std::vector<double>{target_point_.x};
  for (const double x : target_x_vec_) {
    point_d.x = x;
    double radius_cd_tmp = slot_sign_ * (point_c.x - point_d.x) /
                           (sin_target_point_theta_ - sin_point_c_theta);
    if (radius_cd_tmp < min_turn_radius_) {
      ++radius_cnt1;
      continue;
    }

    point_d.y = point_c.y + slot_sign_ * radius_cd_tmp *
                                (cos_target_point_theta_ - cos_point_c_theta);

    if (point_d.y * slot_sign_ < kMaxYBackwardPos) {
      ++dy_cnt1;
      continue;
    }

    if (CollideWithObjectsByPolygon(point_c, point_d, radius_cd_tmp,
                                    init_ego_polygon)) {
      ++collide_cnt1;
      continue;
    }

    const double x_diff = std::fabs(x - target_point_.x);
    const double bias_cost = CalPointxBiasCost(x_diff);

    const double len_bc =
        std::fabs(point_c.theta - point_d.theta) * radius_cd_tmp;
    const double len_c_end = std::fabs(point_d.y - target_point_.y);
    const double len_cost = CalSegmentLengthCost(len_bc + len_c_end);
    const double radius_cost = CalRadiusCost(radius_cd_tmp, len_bc);
    const double total_cost = bias_cost + len_cost + radius_cost;

    if (total_cost < segments_info->total_cost) {
      segments_info->total_cost = total_cost;
      segments_info->opt_point_d = point_d;
      segments_info->opt_radius_cd = radius_cd_tmp;

      if (is_rough_calc) {
        return true;
      }
    }
  }

  if (is_start) {
    AINFO << "total_cnt1:" << target_x_vec_.size()
          << ", radius_cnt1:" << radius_cnt1 << ", dy_cnt1:" << dy_cnt1
          << ", collide_cnt1:" << collide_cnt1;
  }

  const int radius_cd_num = is_start ? 10 : 1;
  const double radius_step = 1.0;
  size_t dx_cnt2 = 0;
  size_t dy_cnt2 = 0;
  size_t collide_cnt2 = 0;
  for (int i = 0; i < radius_cd_num; ++i) {
    const double radius_cd_tmp = min_turn_radius_ + radius_step * i;
    point_d.x = point_c.x + slot_sign_ * radius_cd_tmp *
                                (sin_point_c_theta - sin_target_point_theta_);
    if (std::fabs(point_d.x - target_point_.x) > kMaxXOffset) {
      ++dx_cnt2;
      continue;
    }

    point_d.y = point_c.y + slot_sign_ * radius_cd_tmp *
                                (-cos_point_c_theta + cos_target_point_theta_);
    if (point_d.y * slot_sign_ < kMaxYBackwardPos) {
      ++dy_cnt2;
      continue;
    }

    if (CollideWithObjectsByPolygon(point_c, point_d, radius_cd_tmp,
                                    init_ego_polygon)) {
      ++collide_cnt2;
      continue;
    }

    const double x_diff = std::fabs(point_d.x - target_point_.x);
    const double bias_cost = CalPointxBiasCost(x_diff);

    const double len_bc =
        std::fabs(point_c.theta - point_d.theta) * radius_cd_tmp;
    const double len_c_end = std::fabs(point_d.y - target_point_.y);
    const double len_cost = CalSegmentLengthCost(len_bc + len_c_end);
    const double radius_cost = CalRadiusCost(radius_cd_tmp, len_bc);
    const double total_cost = bias_cost + len_cost + radius_cost;

    if (total_cost < segments_info->total_cost) {
      segments_info->total_cost = total_cost;

      segments_info->opt_radius_cd = radius_cd_tmp;
      segments_info->opt_point_d = point_d;

      if (is_rough_calc) {
        return true;
      }
    }
  }

  if (is_start) {
    AINFO << "total_cnt2:" << radius_cd_num << ", dx_cnt2:" << dx_cnt2
          << ", dy_cnt2:" << dy_cnt2 << ", collide_cnt2:" << collide_cnt2;
  }

  if (std::isinf(segments_info->total_cost)) {
    return false;
  }

  return true;
}

bool DiagonalInGeometryPlan::CD2Segment(const PlanningPoint &point_c,
                                        bool is_start, bool is_rough_calc,
                                        DiagonalSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  if (point_c.theta * slot_sign_ >= M_PI_2) {
    return false;
  }

  const double sin_point_c_theta = apa_sin(point_c.theta);
  const double cos_point_c_theta = apa_cos(point_c.theta);
  PlanningPoint point_d;
  size_t collide_cnt = 0;
  size_t de_fail_cnt = 0;
  size_t total_cost_nan_cnt = 0;
  const double radius_cd_step = 1;
  const double theta_step = kDefaultThetaStep * slot_sign_;
  const int size_step = std::max(
      static_cast<int>((target_point_.theta - point_c.theta) / theta_step), 1);
  const int size_radius_cd = is_start ? 10 : 1;
  const double front_buffer = is_start ? kLonReverseBuffer : 0.0;
  const double rear_buffer = kLonBuffer;
  const double lat_buffer = kLatBuffer;
  Polygon2d init_ego_polygon = std::move(ConstructVehiclePolygonWithBuffer(
      point_c, front_buffer, rear_buffer, lat_buffer));
  for (int i = 0; i < size_radius_cd; ++i) {
    const double radius_cd_tmp = min_turn_radius_ + i * radius_cd_step;
    for (int j = 0; j < size_step; ++j) {
      point_d.theta = point_c.theta + theta_step * j;
      point_d.x = point_c.x + slot_sign_ * radius_cd_tmp *
                                  (sin_point_c_theta - apa_sin(point_d.theta));
      point_d.y = point_c.y + slot_sign_ * radius_cd_tmp *
                                  (apa_cos(point_d.theta) - cos_point_c_theta);

      if (CollideWithObjectsByPolygon(point_c, point_d, init_ego_polygon)) {
        ++collide_cnt;
        break;
      }

      if (point_d.x > target_point_.x + kMaxXOffsetInCD2) {
        continue;
      }

      DiagonalSegmentsInfo tmp_segments_info;
      if (!DE1Segment(point_d, false, is_rough_calc, &tmp_segments_info)) {
        ++de_fail_cnt;
        continue;
      }

      if (std::isinf(tmp_segments_info.total_cost)) {
        ++total_cost_nan_cnt;
        continue;
      }

      const double len_cd =
          std::fabs(point_c.theta - point_d.theta) * radius_cd_tmp;
      const double len_cost = CalSegmentLengthCost(len_cd);
      const double radius_cost = CalRadiusCost(radius_cd_tmp, len_cd);
      const double theta_cost = CalPointDThetaCost(point_d.theta);

      const double total_cost =
          len_cost + radius_cost + theta_cost + tmp_segments_info.total_cost;

      if (total_cost < segments_info->total_cost) {
        segments_info->total_cost = total_cost;

        // cd
        segments_info->opt_radius_cd = radius_cd_tmp;
        segments_info->opt_point_d = point_d;

        // de
        segments_info->opt_radius_de = tmp_segments_info.opt_radius_de;
        segments_info->opt_point_e = tmp_segments_info.opt_point_e;

        // ef
        segments_info->opt_radius_ef = tmp_segments_info.opt_radius_ef;
        segments_info->opt_point_f = tmp_segments_info.opt_point_f;

        if (is_rough_calc) {
          return true;
        }
      }
    }
  }

  if (is_start) {
    AINFO << "total_cnt:" << (size_radius_cd * size_step)
          << ", collide_cnt:" << collide_cnt << ", de_fail_cnt:" << de_fail_cnt
          << ", total_cost_nan_cnt:" << total_cost_nan_cnt;
  }

  if (std::isinf(segments_info->total_cost)) {
    return false;
  }

  return true;
}

bool DiagonalInGeometryPlan::DE1Segment(const PlanningPoint &point_d,
                                        bool is_start, bool is_rough_calc,
                                        DiagonalSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  const double sin_point_d_theta = apa_sin(point_d.theta);
  const double cos_point_d_theta = apa_cos(point_d.theta);
  if (point_d.theta * slot_sign_ > M_PI_2) {
    return false;
  }

  PlanningPoint point_e;
  const double radius_step = 1.0;
  const int radius_de_num = is_start ? 10 : 1;
  const int radius_ef_num = is_start ? 10 : 1;
  const double front_buffer = kLonBuffer;
  const double rear_buffer = is_start ? kLonReverseBuffer : 0.0;
  const double lat_buffer = kLatBuffer;
  Polygon2d init_ego_polygon = std::move(ConstructVehiclePolygonWithBuffer(
      point_d, front_buffer, rear_buffer, lat_buffer));
  for (int i = 0; i < radius_de_num; i++) {
    const double radius_de_tmp = min_turn_radius_ + radius_step * i;
    for (int j = 0; j < radius_ef_num; j++) {
      const double radius_ef_tmp = min_turn_radius_ + radius_step * j;
      const double sin_e_tmp = ((target_point_.x - point_d.x) * slot_sign_ +
                                radius_de_tmp * sin_point_d_theta +
                                radius_ef_tmp * sin_target_point_theta_) /
                               (radius_de_tmp + radius_ef_tmp);

      if (sin_e_tmp * slot_sign_ <= 0 || fabs(sin_e_tmp) >= 1.0) {
        continue;
      }

      point_e.theta = asin(sin_e_tmp);
      if (fabs(point_e.theta) > fabs(target_point_.theta)) {
        continue;
      }
      if (fabs(point_e.theta) < fabs(point_d.theta)) {
        continue;
      }

      const double sin_point_e_theta = sin_e_tmp;
      const double cos_point_e_theta = apa_cos(point_e.theta);
      point_e.x = point_d.x + slot_sign_ * radius_de_tmp *
                                  (sin_point_e_theta - sin_point_d_theta);
      point_e.y = point_d.y + slot_sign_ * radius_de_tmp *
                                  (cos_point_d_theta - cos_point_e_theta);

      if (CollideWithObjectsByPolygon(point_d, point_e, radius_de_tmp,
                                      init_ego_polygon)) {
        continue;
      }

      DiagonalSegmentsInfo tmp_segments_info;
      if (!EFSegment(point_e, false, is_rough_calc, &tmp_segments_info)) {
        continue;
      }

      if (std::isinf(tmp_segments_info.total_cost)) {
        continue;
      }

      const double len_de =
          std::fabs(point_d.theta - point_e.theta) * radius_de_tmp;
      // const double len_cost = CalSegmentLengthCost(len_de);
      // TODO(fengwang31):updata the CalDESegmentLengthCost
      const double len_cost = CalDESegmentLengthCost(len_de);
      const double radius_cost = CalRadiusCost(radius_de_tmp, len_de);

      const double total_cost =
          len_cost + radius_cost + tmp_segments_info.total_cost;
      if (std::isinf(total_cost)) {
        continue;
      }

      if (total_cost < segments_info->total_cost) {
        segments_info->total_cost = total_cost;

        // de
        segments_info->opt_radius_de = radius_de_tmp;
        segments_info->opt_point_e = point_e;

        // ef
        segments_info->opt_radius_ef = tmp_segments_info.opt_radius_ef;
        segments_info->opt_point_f = tmp_segments_info.opt_point_f;

        if (is_rough_calc) {
          return true;
        }
      }
    }
  }
  if (std::isinf(segments_info->total_cost)) {
    return false;
  }

  return true;
}

bool DiagonalInGeometryPlan::EFSegment(const PlanningPoint &point_e,
                                       bool is_start, bool is_rough_calc,
                                       DiagonalSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  DiagonalSegmentsInfo tmp_segments_info;
  if (!CD1Segment(point_e, false, is_rough_calc, &tmp_segments_info)) {
    return false;
  }

  *segments_info = tmp_segments_info;
  segments_info->opt_radius_ef = tmp_segments_info.opt_radius_cd;
  segments_info->opt_point_f = tmp_segments_info.opt_point_d;

  return true;
}

bool DiagonalInGeometryPlan::CD3Segment(const PlanningPoint &point_c,
                                        bool is_start, bool is_rough_calc,
                                        DiagonalSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  if (point_c.theta * slot_sign_ >= M_PI_2) {
    return false;
  }

  const double sin_point_c_theta = apa_sin(point_c.theta);
  const double cos_point_c_theta = apa_cos(point_c.theta);
  PlanningPoint point_d;
  size_t collide_cnt = 0;
  size_t ex1_cnt = 0;
  size_t ex2_cnt = 0;
  size_t total_cost_nan_cnt = 0;
  const double radius_cd_step = 1;
  const double theta_step = kDefaultThetaStep * slot_sign_;
  const int size_step = std::max(
      static_cast<int>((target_point_.theta - point_c.theta) / theta_step), 1);
  const int size_radius_cd = is_start ? 10 : 1;
  const double front_buffer = is_start ? kLonReverseBuffer : 0.0;
  const double rear_buffer = kLonBuffer;
  const double lat_buffer = kLatBuffer;
  Polygon2d init_ego_polygon = std::move(ConstructVehiclePolygonWithBuffer(
      point_c, front_buffer, rear_buffer, lat_buffer));
  for (int i = 0; i < size_radius_cd; ++i) {
    const double radius_cd_tmp = min_turn_radius_ + i * radius_cd_step;
    for (int j = 0; j < size_step; ++j) {
      point_d.theta = point_c.theta + theta_step * j;
      point_d.x = point_c.x + slot_sign_ * radius_cd_tmp *
                                  (-apa_sin(point_d.theta) + sin_point_c_theta);
      point_d.y = point_c.y + slot_sign_ * radius_cd_tmp *
                                  (apa_cos(point_d.theta) - cos_point_c_theta);

      if (CollideWithObjectsByPolygon(point_c, point_d, init_ego_polygon)) {
        ++collide_cnt;
        break;
      }

      if (point_d.x < target_point_.x - kMaxXOffset) {
        ++ex1_cnt;
        break;
      }

      if (point_d.x > target_point_.x + kMaxXOffset) {
        ++ex2_cnt;
        continue;
      }

      const double len_cd =
          std::fabs(point_c.theta - point_d.theta) * radius_cd_tmp;
      const double len_cost = CalSegmentLengthCost(len_cd);
      const double radius_cost = CalRadiusCost(radius_cd_tmp, len_cd);
      const double theta_cost = CalEndThetaCost(point_d.theta);

      const double total_cost = len_cost + radius_cost + theta_cost;

      if (total_cost < segments_info->total_cost) {
        segments_info->total_cost = total_cost;

        // cd
        segments_info->opt_radius_cd = radius_cd_tmp;
        segments_info->opt_point_d = point_d;

        if (is_rough_calc) {
          return true;
        }
      }
    }
  }

  if (is_start) {
    AINFO << "total_cnt:" << (size_radius_cd * size_step)
          << ", collide_cnt:" << collide_cnt << ", ex1_cnt:" << ex1_cnt
          << ", ex2_cnt:" << ex2_cnt
          << ", total_cost_nan_cnt:" << total_cost_nan_cnt;
  }

  if (std::isinf(segments_info->total_cost)) {
    return false;
  }
  return true;
}

bool DiagonalInGeometryPlan::DESegment(const PlanningPoint &point_d,
                                       bool is_start, bool is_rough_calc,
                                       DiagonalSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  DiagonalSegmentsInfo tmp_segments_info;
  if (DE1Segment(point_d, is_start, is_rough_calc, &tmp_segments_info)) {
    segments_info->total_cost = tmp_segments_info.total_cost;
    segments_info->opt_radius_de = tmp_segments_info.opt_radius_de;
    segments_info->opt_point_e = tmp_segments_info.opt_point_e;
    // AINFO << "de1 success";
    return true;
  }

  if (DE2Segment(point_d, is_start, is_rough_calc, &tmp_segments_info)) {
    segments_info->total_cost = tmp_segments_info.total_cost + kSegmentCost;
    segments_info->opt_radius_de = tmp_segments_info.opt_radius_de;
    segments_info->opt_point_e = tmp_segments_info.opt_point_e;
    // AINFO << "de2 success";
    return true;
  }

  if (DE3Segment(point_d, is_start, is_rough_calc, &tmp_segments_info)) {
    segments_info->total_cost =
        tmp_segments_info.total_cost + 2.0 * kSegmentCost;
    segments_info->opt_radius_de = tmp_segments_info.opt_radius_de;
    segments_info->opt_point_e = tmp_segments_info.opt_point_e;
    // AINFO << "de3 success";
    return true;
  }

  return false;
}

bool DiagonalInGeometryPlan::DE2Segment(const PlanningPoint &point_d,
                                        bool is_start, bool is_rough_calc,
                                        DiagonalSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  if (point_d.theta * slot_sign_ >= M_PI_2) {
    return false;
  }

  const double max_point_e_theta = 0.0;
  const double yaw_step = kDefaultThetaStep * slot_sign_;
  const int point_num = std::max(
      static_cast<int>((max_point_e_theta - point_d.theta) / yaw_step), 1);

  const double cos_point_d_theta = apa_cos(point_d.theta);
  const double sin_point_d_theta = apa_sin(point_d.theta);

  PlanningPoint point_e;
  size_t ex_cnt1 = 0;
  size_t collide_cnt = 0;
  size_t ef_fail_cnt = 0;
  size_t total_cost_nan_cnt = 0;
  const int radius_de_num = is_start ? 20 : 1;
  const double radius_step = 1.0;
  const double front_buffer = kLonBuffer;
  const double rear_buffer = is_start ? kLonReverseBuffer : 0.0;
  const double lat_buffer = kLatBuffer;
  Polygon2d init_ego_polygon = std::move(ConstructVehiclePolygonWithBuffer(
      point_d, front_buffer, rear_buffer, lat_buffer));
  for (int i = 0; i < radius_de_num; ++i) {
    const double radius_de = min_turn_radius_ + radius_step * i;
    for (int j = 0; j < point_num; ++j) {
      point_e.theta = point_d.theta + yaw_step * j;
      point_e.x = point_d.x + slot_sign_ * radius_de *
                                  (-sin_point_d_theta + apa_sin(point_e.theta));
      if (point_e.x > target_point_.x + kMaxXOffset) {
        ++ex_cnt1;
        break;
      }

      point_e.y = point_d.y + slot_sign_ * radius_de *
                                  (cos_point_d_theta - apa_cos(point_e.theta));

      if (CollideWithObjectsByPolygon(point_e, point_e, init_ego_polygon)) {
        ++collide_cnt;
        break;
      }

      DiagonalSegmentsInfo tmp_segments_info;
      if (!EFSegment(point_e, false, is_rough_calc, &tmp_segments_info)) {
        ++ef_fail_cnt;
        continue;
      }

      const double len_de =
          std::fabs(point_d.theta - point_e.theta) * radius_de;
      const double len_cost = CalDESegmentLengthCost(len_de);
      const double radius_cost = CalRadiusCost(radius_de, len_de);

      const double total_cost =
          len_cost + radius_cost + tmp_segments_info.total_cost;

      if (std::isinf(total_cost)) {
        ++total_cost_nan_cnt;
        continue;
      }
      if (total_cost < segments_info->total_cost) {
        segments_info->total_cost = total_cost;

        // ef
        segments_info->opt_radius_de = radius_de;
        segments_info->opt_point_e = point_e;

        // ef
        segments_info->opt_radius_ef = tmp_segments_info.opt_radius_ef;
        segments_info->opt_point_f = tmp_segments_info.opt_point_f;

        if (is_rough_calc) {
          return true;
        }
      }
    }
  }

  if (is_start) {
    AINFO << "total_cnt:" << (point_num * radius_de_num)
          << ", collide_cnt:" << collide_cnt << ", ex_cnt1:" << ex_cnt1
          << ", ef_fail_cnt:" << ef_fail_cnt
          << ", total_cost_nan_cnt:" << total_cost_nan_cnt;
  }

  if (std::isinf(segments_info->total_cost)) {
    return false;
  }

  return true;
}

bool DiagonalInGeometryPlan::DE3Segment(const PlanningPoint &point_d,
                                        bool is_start, bool is_rough_calc,
                                        DiagonalSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  if (point_d.theta * slot_sign_ >= M_PI_2) {
    return false;
  }

  const double yaw_step =
      is_start ? kMinThetaStep * slot_sign_ : kDefaultThetaStep * slot_sign_;
  const int point_num = std::max(
      static_cast<int>((target_point_.theta - point_d.theta) / yaw_step), 1);

  const double cos_point_d_theta = apa_cos(point_d.theta);
  const double sin_point_d_theta = apa_sin(point_d.theta);

  PlanningPoint point_e;
  size_t ex_cnt2 = 0;
  size_t collide_cnt = 0;
  size_t theta_cnt = 0;
  size_t total_cost_nan_cnt = 0;
  const int radius_de_num = is_start ? 20 : 1;
  const double radius_step = 1.0;
  const double front_buffer = is_start ? kLonReverseBuffer : 0.0;
  const double rear_buffer = kLonBuffer;
  const double lat_buffer = kLatBuffer;
  Polygon2d init_ego_polygon = std::move(ConstructVehiclePolygonWithBuffer(
      point_d, front_buffer, rear_buffer, lat_buffer));
  for (int i = 0; i < radius_de_num; ++i) {
    const double radius_de = min_turn_radius_ + radius_step * i;
    for (int j = 0; j < point_num; ++j) {
      point_e.theta = point_d.theta + yaw_step * j;
      point_e.x = point_d.x + slot_sign_ * radius_de *
                                  (-sin_point_d_theta + apa_sin(point_e.theta));

      if (point_e.x > target_point_.x + kMaxXOffsetInDE3) {
        ++ex_cnt2;
        break;
      }

      point_e.y = point_d.y + slot_sign_ * radius_de *
                                  (cos_point_d_theta - apa_cos(point_e.theta));

      if (CollideWithObjectsByPolygon(point_d, point_e, init_ego_polygon)) {
        ++collide_cnt;
        break;
      }

      if (std::fabs(point_e.theta - target_point_.theta) < 0.05) {
        ++theta_cnt;
        break;
      }

      const double len_de =
          std::fabs(point_d.theta - point_e.theta) * radius_de;
      const double len_cost = CalDESegmentLengthCost(len_de);
      const double radius_cost = CalRadiusCost(radius_de, len_de);
      const double theta_cost = CalEndThetaCost(point_e.theta);

      const double total_cost = len_cost + radius_cost + theta_cost;

      if (std::isinf(total_cost)) {
        ++total_cost_nan_cnt;
        continue;
      }
      if (total_cost < segments_info->total_cost) {
        segments_info->total_cost = total_cost;

        // de
        segments_info->opt_radius_de = radius_de;
        segments_info->opt_point_e = point_e;

        if (is_rough_calc) {
          return true;
        }
      }
    }
  }

  if (is_start) {
    AINFO << "total_cnt:" << (point_num * radius_de_num)
          << ", collide_cnt:" << collide_cnt << ", theta_cnt:" << theta_cnt
          << ", ex_cnt2:" << ex_cnt2
          << ", total_cost_nan_cnt:" << total_cost_nan_cnt;
  }

  if (std::isinf(segments_info->total_cost)) {
    return false;
  }

  return true;
}

bool DiagonalInGeometryPlan::CollideWithObjectsByBox(
    const PlanningPoint &veh_point, const double front_buffer,
    const double rear_buffer, const double lat_buffer) const {
  if (objects_map_line_segments_.empty()) {
    return false;
  }

  const double front_edge_to_center_with_safe_dst =
      front_edge_to_center_ + front_buffer;
  const double back_edge_to_center_with_safe_dst =
      back_edge_to_center_ + rear_buffer;
  const double shift_distance =
      (front_edge_to_center_with_safe_dst - back_edge_to_center_with_safe_dst) *
      0.5;
  const double veh_length_with_safe_dst =
      front_edge_to_center_with_safe_dst + back_edge_to_center_with_safe_dst;
  const double veh_width_with_safe_dis = width_veh_ + lat_buffer * 2.0;
  double veh_x = veh_point.x + shift_distance * apa_cos(veh_point.theta);
  double veh_y = veh_point.y + shift_distance * apa_sin(veh_point.theta);
  Box2d ego_box({veh_x, veh_y}, veh_point.theta, veh_length_with_safe_dst,
                veh_width_with_safe_dis);
  for (const auto &line_segment : objects_map_line_segments_) {
    if (ego_box.HasOverlap(line_segment)) {
      return true;
    }
  }

  return false;
}

bool DiagonalInGeometryPlan::CollideWithObjectsByBox(
    const PlanningPoint &veh_point1, const PlanningPoint &veh_point2,
    const double radius, const double front_buffer, const double rear_buffer,
    const double lat_buffer) const {
  if (objects_map_line_segments_.empty()) {
    return false;
  }

  const double sin_point_1_theta = apa_sin(veh_point1.theta);
  const double cos_point_1_theta = apa_cos(veh_point1.theta);
  double yaw_step = kYawStepRatio / radius;
  const double theta_diff = std::fabs(veh_point1.theta - veh_point2.theta);
  int size_theta = std::ceil(theta_diff / yaw_step);
  size_theta = std::max(size_theta, 1);
  yaw_step = theta_diff / size_theta;
  // TODO(xujian.li): temporary method
  double x_sign = 1.0;
  if (veh_point2.x < veh_point1.x) {
    x_sign = -1.0;
  }
  double y_sign = 1.0;
  if (veh_point2.y < veh_point1.y) {
    y_sign = -1.0;
  }
  double theta_sign = 1.0;
  if (veh_point2.theta < veh_point1.theta) {
    theta_sign = -1.0;
  }

  const double front_edge_to_center_with_safe_dst =
      front_edge_to_center_ + front_buffer;
  const double back_edge_to_center_with_safe_dst =
      back_edge_to_center_ + rear_buffer;
  const double shift_distance =
      (front_edge_to_center_with_safe_dst - back_edge_to_center_with_safe_dst) *
      0.5;
  const double veh_length_with_safe_dst =
      front_edge_to_center_with_safe_dst + back_edge_to_center_with_safe_dst;
  const double veh_width_with_safe_dis = width_veh_ + lat_buffer * 2.0;

  for (int i = 0; i <= size_theta; ++i) {
    const double veh_theta = veh_point1.theta + theta_sign * yaw_step * i;
    const double sin_veh_theta = apa_sin(veh_theta);
    const double cos_veh_theta = apa_cos(veh_theta);
    double veh_x =
        veh_point1.x +
        x_sign * radius * std::fabs(-sin_point_1_theta + sin_veh_theta) +
        shift_distance * cos_veh_theta;
    double veh_y =
        veh_point1.y +
        y_sign * radius * std::fabs(cos_point_1_theta - cos_veh_theta) +
        shift_distance * sin_veh_theta;
    Box2d ego_box({veh_x, veh_y}, veh_theta, veh_length_with_safe_dst,
                  veh_width_with_safe_dis);
    for (const auto &line_segment : objects_map_line_segments_) {
      if (ego_box.HasOverlap(line_segment)) {
        return true;
      }
    }
  }

  return false;
}

bool DiagonalInGeometryPlan::CollideWithObjectsByPolygon(
    const PlanningPoint &veh_point1, const PlanningPoint &veh_point2,
    const Polygon2d &init_ego_polygon) const {
  if (objects_map_line_segments_.empty()) {
    return false;
  }

  Polygon2d ego_polygon(init_ego_polygon);
  const double rotate_angle = veh_point2.theta - veh_point1.theta;
  const double sin_rotate_angle = apa_sin(rotate_angle);
  const double cos_rotate_angle = apa_cos(rotate_angle);
  ego_polygon.RotateAndTranslate(
      Vec2d(veh_point1.x, veh_point1.y), sin_rotate_angle, cos_rotate_angle,
      Vec2d(veh_point2.x - veh_point1.x, veh_point2.y - veh_point1.y));

  for (const auto &line_segment : objects_map_line_segments_) {
    if (ego_polygon.HasOverlap(line_segment)) {
      return true;
    }
  }

  return false;
}

bool DiagonalInGeometryPlan::CollideWithObjectsByPolygon(
    const PlanningPoint &veh_point1, const PlanningPoint &veh_point2,
    const double radius, const Polygon2d &init_ego_polygon) const {
  if (objects_map_line_segments_.empty()) {
    return false;
  }

  const double sin_point_1_theta = apa_sin(veh_point1.theta);
  const double cos_point_1_theta = apa_cos(veh_point1.theta);
  double yaw_step = kYawStepRatio / radius;
  const double theta_diff = std::fabs(veh_point1.theta - veh_point2.theta);
  int size_theta = std::ceil(theta_diff / yaw_step);
  size_theta = std::max(size_theta, 1);
  yaw_step = theta_diff / size_theta;
  // TODO(xujian.li): temporary method
  double x_sign = 1.0;
  if (veh_point2.x < veh_point1.x) {
    x_sign = -1.0;
  }
  double y_sign = 1.0;
  if (veh_point2.y < veh_point1.y) {
    y_sign = -1.0;
  }
  double theta_sign = 1.0;
  if (veh_point2.theta < veh_point1.theta) {
    theta_sign = -1.0;
  }

  PlanningPoint veh_point;
  for (int i = 0; i <= size_theta; ++i) {
    veh_point.theta = veh_point1.theta + theta_sign * yaw_step * i;
    const double sin_veh_theta = apa_sin(veh_point.theta);
    const double cos_veh_theta = apa_cos(veh_point.theta);
    veh_point.x =
        veh_point1.x +
        x_sign * radius * std::fabs(-sin_point_1_theta + sin_veh_theta);
    veh_point.y =
        veh_point1.y +
        y_sign * radius * std::fabs(cos_point_1_theta - cos_veh_theta);

    Polygon2d ego_polygon(init_ego_polygon);
    const double rotate_angle = veh_point.theta - veh_point1.theta;
    const double sin_rotate_angle = apa_sin(rotate_angle);
    const double cos_rotate_angle = apa_cos(rotate_angle);
    ego_polygon.RotateAndTranslate(
        Vec2d(veh_point1.x, veh_point1.y), sin_rotate_angle, cos_rotate_angle,
        Vec2d(veh_point.x - veh_point1.x, veh_point.y - veh_point1.y));

    for (const auto &line_segment : objects_map_line_segments_) {
      if (ego_polygon.HasOverlap(line_segment)) {
        return true;
      }
    }
  }

  return false;
}

bool DiagonalInGeometryPlan::CEndCollideCheck(const PlanningPoint &point_c,
                                              const double safe_dst) const {
  if (objects_map_line_segments_.empty()) {
    return false;
  }

  const double front_edge_to_center_with_safe_dst =
      front_edge_to_center_ + safe_dst;
  const double back_edge_to_center_with_safe_dst =
      back_edge_to_center_ + safe_dst;
  const double shift_distance =
      (front_edge_to_center_with_safe_dst - back_edge_to_center_with_safe_dst) *
      0.5 * sin_target_point_theta_;
  const double veh_length_with_safe_dst =
      front_edge_to_center_with_safe_dst + back_edge_to_center_with_safe_dst;
  const double veh_width_with_safe_dis = width_veh_ + safe_dst * 2.0;

  double y_step = veh_length_with_safe_dst;
  const double y_diff = std::fabs(point_c.y - target_point_.y);
  int size_y = std::ceil(y_diff / y_step);
  y_step = y_diff / size_y;

  for (int i = 0; i <= size_y; ++i) {
    const double veh_y = point_c.y - slot_sign_ * y_step * i + shift_distance;
    Box2d ego_box({point_c.x, veh_y}, target_point_.theta,
                  veh_length_with_safe_dst, veh_width_with_safe_dis);
    for (const auto &line_segment : objects_map_line_segments_) {
      if (ego_box.HasOverlap(line_segment)) {
        return true;
      }
    }
  }

  return false;
}

void DiagonalInGeometryPlan::SetTargetPoint(const PlanningPoint &target_point) {
  target_point_ = target_point;
  CalTargetXVec();
  sin_target_point_theta_ = apa_sin(target_point_.theta);
  cos_target_point_theta_ = apa_cos(target_point_.theta);
}

void DiagonalInGeometryPlan::CalTargetXVec() {
  target_x_vec_.clear();
  target_x_vec_.reserve(kTargetXNum);
  for (int i = 0; i < kTargetXNum; ++i) {
    const double target_x =
        target_point_.x + (i - kHalfTargetXNum) * kTargetXStep;
    target_x_vec_.push_back(target_x);
    // AINFO << "target_x:" << target_x;
  }
}

double DiagonalInGeometryPlan::CalPointxBiasCost(const double x_bias) const {
  return kPointxBiasWeight * x_bias;
}

double DiagonalInGeometryPlan::CalSegmentLengthCost(
    const double segment_len) const {
  if (segment_len < kMinSegmentLen) {
    return std::numeric_limits<double>::infinity();
  } else {
    return kSegmentLengthWeight * segment_len;
  }

  return std::numeric_limits<double>::infinity();
}

double DiagonalInGeometryPlan::CalRadiusCost(const double radius,
                                             const double len) const {
  if (radius < min_turn_radius_) {
    return std::numeric_limits<double>::infinity();
  } else {
    return kRadiusWeight * len / radius;
  }

  return std::numeric_limits<double>::infinity();
}

double DiagonalInGeometryPlan::CalEndThetaCost(const double theta) const {
  return std::fabs(target_point_.theta - theta) * kEndThetaWeight;
}

double DiagonalInGeometryPlan::CalBCThetaDiffCost(
    const double theta_diff) const {
  return theta_diff * kBCThetaDiffCost;
}

double DiagonalInGeometryPlan::CalPointDThetaCost(const double theta) const {
  return std::fabs(target_point_.theta - theta) * kPointDThetaCost;
}

double DiagonalInGeometryPlan::CalDESegmentLengthCost(
    const double segment_len) const {
  if (segment_len < kMinSegmentLen) {
    return std::numeric_limits<double>::infinity();
  } else {
    if (segment_len < kDEStandardLength) {
      return kSegmentLengthWeight * (1 / segment_len) * kDELengthWeight;
    } else {
      return kSegmentLengthWeight * segment_len * kDELengthWeight;
    }
  }
  return 0.0;
}

}  // namespace apa_planner
}  // namespace planning
