#include "apa_planner/parallel/parallel_in_geometry_plan.h"

#include <limits>
#include <math.h>

#include "apa_planner/common/apa_cos_sin.h"
#include "apa_planner/common/apa_utils.h"
#include "apa_planner/common/planning_log_helper.h"
#include "common/math/box2d.h"
#include "common/math/math_utils.h"

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
constexpr double kLatBuffer = 0.1;
constexpr double kLonBuffer = 0.1;
constexpr double kLonReverseBuffer = -1.5;
constexpr double kBackwardOverLen = 2.0;
constexpr double kForwardOverLen = 2.0;
constexpr double kMaxThetaDiff = 0.05;
constexpr double M_PI_10 = M_PI * 0.1;
constexpr double M_PI_20 = M_PI * 0.05;
constexpr double kTargetYStep = 0.02;
constexpr int kTargetYNum = 21;
constexpr int kHalfTargetYNum = kTargetYNum / 2;
constexpr double kMaxYOffset = kTargetYStep * kHalfTargetYNum;
constexpr double kMaxXOffset = 0.2;
constexpr double kSegmentLengthWeight = 1.0;
constexpr double kPointyBiasWeight = 20.0;
constexpr double kRadiusWeight = 1.0;
constexpr double kGearShiftWeight = 2.0;
constexpr double kEndXWeight = 100.0;
constexpr double kEndYWeight = 100.0;
constexpr double kEndThetaWeight = 100.0;
constexpr double kSegmentCost = 10000.0;
constexpr double kMaxXRearAxle = 12.0;
}  // namespace

bool ParallelInGeometryPlan::CheckSlotOpenSideWrong(
    const PlanningPoint &start_point) {
  bool side_wrong = false;
  if (slot_sign_ * start_point.y < 0) {
    side_wrong = true;
  }
  if (std::fabs(start_point.theta) > M_PI_10) {
    side_wrong = true;
  }
  return side_wrong;
}

bool ParallelInGeometryPlan::ABSegment(const PlanningPoint &point_a,
    bool is_start, bool is_rough_calc, ParallelSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  if (CheckSlotOpenSideWrong(point_a)) {
    PLANNING_LOG << "open side wrong" << std::endl;
    return false;
  }

  const double rear_axle_over_dis = 4.0;
  const double max_ab_len = 15.0;
  const double ab_len =
      planning_math::Clamp(max_ab_len, 0.0, rear_axle_over_dis - point_a.x);
  const double min_l_step = 0.2;
  double max_l_step = 4.0;
  int sparse_step_num = 0;
  const double sparse_len = std::fmax(back_edge_to_center_ - point_a.x, 0.0);
  if (sparse_len > 0.0) {
    sparse_step_num = sparse_len / max_l_step + 1;
    max_l_step = sparse_len / sparse_step_num;
  }
  int dense_step_num = (ab_len - sparse_len) / min_l_step;
  dense_step_num = std::max(dense_step_num, 0);
  const int point_num = std::max(sparse_step_num + dense_step_num, 1);
  PLANNING_LOG << "sparse_step_num:" << sparse_step_num
      << ", dense_step_num:" << dense_step_num << std::endl;

  const double cos_point_a_theta = apa_cos(point_a.theta);
  const double sin_point_a_theta = apa_sin(point_a.theta);
  PlanningPoint point_b;
  // size_t collide_cnt = 0;
  // size_t bc_fail_cnt = 0;
  // size_t total_cost_nan_cnt = 0;
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
      // ++collide_cnt;
      break;
    }

    ParallelSegmentsInfo tmp_segments_info;
    if (!BCSegment(point_b, l_ab, false, is_rough_calc, &tmp_segments_info)) {
      // ++bc_fail_cnt;
      continue;
    }

    const double len_cost = 0.0;
    const double total_cost = len_cost + tmp_segments_info.total_cost;
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
      segments_info->opt_point_e = tmp_segments_info.opt_point_e;

      // ef
      segments_info->opt_radius_ef = tmp_segments_info.opt_radius_ef;
      segments_info->opt_point_f = tmp_segments_info.opt_point_f;

      // fg
      segments_info->opt_radius_fg = tmp_segments_info.opt_radius_fg;
      segments_info->opt_point_g = tmp_segments_info.opt_point_g;

      // fh
      segments_info->opt_radius_fh = tmp_segments_info.opt_radius_fh;
      segments_info->opt_point_h = tmp_segments_info.opt_point_h;

      if (is_rough_calc) {
        return true;
      }
    }
  }

  // PLANNING_LOG << "total_cnt:" << point_num
  //     << ", collide_cnt:" << collide_cnt
  //     << ", bc_fail_cnt:" << zb_fail_cnt
  //     << ", total_cost_nan_cnt:" << total_cost_nan_cnt
  //     << std::endl;

  if (std::isinf(segments_info->total_cost)) {
    return false;
  }

  return true;
}

bool ParallelInGeometryPlan::ReverseABSegment(const PlanningPoint &point_a,
    bool is_start, bool is_rough_calc, ParallelSegmentsInfo *segments_info) {

  if (CheckSlotOpenSideWrong(point_a)) {
    PLANNING_LOG << "open side wrong" << std::endl;
    return false;
  }
  const double rear_axle_over_dis = 4.0;
  if (point_a.x > kMaxXRearAxle) {
    PLANNING_LOG << "ego car's X coordination is larger than upper limit of backward A-B plan " << std::endl;
    return false;
  } else if (point_a.x <= 0) {
    PLANNING_LOG << "ego car's X coordination is smaller than lower limit of backward A-B plan" << std::endl;
    return false;
  }

  const double max_ab_backwards_len = kMaxXRearAxle;
  const double ab_len =
      planning_math::Clamp(max_ab_backwards_len, rear_axle_over_dis, point_a.x);
  const double min_l_step = 0.2;
  double max_l_step = 4.0;
  int sparse_step_num = 0;
  const double sparse_len = std::fmax(point_a.x - rear_axle_over_dis, 0);
  if (sparse_len > 0.0) {
    sparse_step_num = sparse_len / max_l_step + 1;
    max_l_step = sparse_len / sparse_step_num;
  }
  int dense_step_num = (ab_len - sparse_len) / min_l_step;
  dense_step_num = std::max(dense_step_num, 0);
  const int point_num = std::max(sparse_step_num + dense_step_num, 1);
  PLANNING_LOG << "sparse_step_num:" << sparse_step_num
      << ", dense_step_num:" << dense_step_num << std::endl;
  PLANNING_LOG << "sparse_step: " << sparse_len << ", dense_step: " 
      << ab_len - sparse_len << std::endl;

  const double cos_point_a_theta = apa_cos(point_a.theta);
  const double sin_point_a_theta = apa_sin(point_a.theta);
  PlanningPoint point_b;
  // size_t collide_cnt = 0;
  // size_t bc_fail_cnt = 0;
  // size_t total_cost_nan_cnt = 0;
  const double front_buffer = 0.0;
  const double rear_buffer = kLonBuffer;
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
    point_b.x = point_a.x - l_ab * cos_point_a_theta;
    point_b.y = point_a.y - l_ab * sin_point_a_theta;
    point_b.theta = point_a.theta;

    if (CollideWithObjectsByPolygon(point_a, point_b, init_ego_polygon)) {
      // ++collide_cnt;
      break;
    }

    ParallelSegmentsInfo tmp_segments_info;
    if (!CDSegment(point_b, false, is_rough_calc, &tmp_segments_info)) {
      // ++CD_fail_cnt;
      continue;
    }

    const double len_cost = 0.0;
    const double total_cost = len_cost + tmp_segments_info.total_cost;
    if (std::isinf(total_cost)) {
      continue;
    }
    if (total_cost < segments_info->total_cost) {
      segments_info->total_cost = total_cost;

      // ab
      segments_info->opt_point_b = point_b;

      // in backward ab segment,point b is point c.
      segments_info->opt_point_c = point_b;

      // cd
      segments_info->opt_radius_cd = tmp_segments_info.opt_radius_cd;
      segments_info->opt_point_d = tmp_segments_info.opt_point_d;

      // de
      segments_info->opt_point_e = tmp_segments_info.opt_point_e;

      // ef
      segments_info->opt_radius_ef = tmp_segments_info.opt_radius_ef;
      segments_info->opt_point_f = tmp_segments_info.opt_point_f;

      // fg
      segments_info->opt_radius_fg = tmp_segments_info.opt_radius_fg;
      segments_info->opt_point_g = tmp_segments_info.opt_point_g;

      // fh
      segments_info->opt_radius_fh = tmp_segments_info.opt_radius_fh;
      segments_info->opt_point_h = tmp_segments_info.opt_point_h;

      if (is_rough_calc) {
        return true;
      }
    }
  }

  // PLANNING_LOG << "total_cnt:" << point_num
  //     << ", collide_cnt:" << collide_cnt
  //     << ", bc_fail_cnt:" << zb_fail_cnt
  //     << ", total_cost_nan_cnt:" << total_cost_nan_cnt
  //     << std::endl;

  if (std::isinf(segments_info->total_cost)) {
    return false;
  }

  return true;
}

bool ParallelInGeometryPlan::BCSegment(const PlanningPoint &point_b,
    double len_ab, bool is_start, bool is_rough_calc,
    ParallelSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  if (point_b.theta * slot_sign_ > M_PI_10) {
    return false;
  }

  const double yaw_step = kDefaultThetaStep * slot_sign_;
  const double max_theta_diff = M_PI_20 * slot_sign_;
  const int point_num = 1;
  // const int point_num = std::max(static_cast<int>(max_theta_diff / yaw_step), 1);

  const double cos_point_b_theta = apa_cos(point_b.theta);
  const double sin_point_b_theta = apa_sin(point_b.theta);

  PlanningPoint point_c;
  // size_t collide_cnt = 0;
  // size_t bc_fail_cnt = 0;
  // size_t total_cost_nan_cnt = 0;
  const double front_buffer = kLonBuffer;
  const double rear_buffer = 0.0;
  const double lat_buffer = kLatBuffer;
  Polygon2d init_ego_polygon = std::move(ConstructVehiclePolygonWithBuffer(
        point_b, front_buffer, rear_buffer, lat_buffer));
  for (int i = 0; i < point_num; ++i) {
    point_c.theta = point_b.theta + yaw_step * i;
    point_c.x = point_b.x + slot_sign_ * min_turn_radius_ *\
        (-sin_point_b_theta + apa_sin(point_c.theta));
    point_c.y = point_b.y + slot_sign_ * min_turn_radius_ *\
        (cos_point_b_theta - apa_cos(point_c.theta));

    if (CollideWithObjectsByPolygon(point_b, point_c, init_ego_polygon)) {
      // ++collide_cnt;
      break;
    }

    ParallelSegmentsInfo tmp_segments_info;
    if (!CDSegment(point_c, false, is_rough_calc, &tmp_segments_info)) {
      // ++bc_fail_cnt;
      continue;
    }

    const double len_bc =
        std::fabs(point_b.theta - point_c.theta) * min_turn_radius_;
    const double len_cost = CalSegmentLengthCost(len_ab + len_bc);
    const double radius_cost = CalRadiusCost(min_turn_radius_, len_bc);

    const double total_cost = len_cost + radius_cost +
        tmp_segments_info.total_cost;
    if (std::isinf(total_cost)) {
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
      segments_info->opt_point_e = tmp_segments_info.opt_point_e;

      // ef
      segments_info->opt_radius_ef = tmp_segments_info.opt_radius_ef;
      segments_info->opt_point_f = tmp_segments_info.opt_point_f;

      // fg
      segments_info->opt_radius_fg = tmp_segments_info.opt_radius_fg;
      segments_info->opt_point_g = tmp_segments_info.opt_point_g;

      // fh
      segments_info->opt_radius_fh = tmp_segments_info.opt_radius_fh;
      segments_info->opt_point_h = tmp_segments_info.opt_point_h;

      if (is_rough_calc) {
        return true;
      }
    }

  }

  // PLANNING_LOG << "total_cnt:" << point_num
  //     << ", collide_cnt:" << collide_cnt
  //     << ", bc_fail_cnt:" << bc_fail_cnt
  //     << ", total_cost_nan_cnt:" << total_cost_nan_cnt
  //     << std::endl;

  if (std::isinf(segments_info->total_cost)) {
    return false;
  }

  return true;
}

bool ParallelInGeometryPlan::CDSegment(const PlanningPoint &point_c,
    bool is_start, bool is_rough_calc, ParallelSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  if (point_c.theta * slot_sign_ >= M_PI_4) {
    return false;
  }

  const double max_theta_diff = M_PI_4 * slot_sign_;
  const double yaw_step = kDefaultThetaStep * slot_sign_;
  const int point_num = std::max(static_cast<int>(max_theta_diff / yaw_step), 1);

  const double cos_point_c_theta = apa_cos(point_c.theta);
  const double sin_point_c_theta = apa_sin(point_c.theta);

  PlanningPoint point_d;
  // size_t collide_cnt = 0;
  // size_t de_fail_cnt = 0;
  // size_t total_cost_nan_cnt = 0;
  const double front_buffer = 0.0;
  const double rear_buffer = kLonBuffer;
  const double lat_buffer = kLatBuffer;
  Polygon2d init_ego_polygon = std::move(ConstructVehiclePolygonWithBuffer(
      point_c, front_buffer, rear_buffer, lat_buffer));
  const int radius_num = is_start ? 4 : 1;
  const double radius_step = 1.0;
  for (int i = 0; i < radius_num; ++i) {
    const double radius_cd = min_turn_radius_ + radius_step * i;
    for (int j = 0; j < point_num; ++j) {
      point_d.theta = point_c.theta + yaw_step * j;
      point_d.x = point_c.x +
          slot_sign_ * radius_cd * (sin_point_c_theta - apa_sin(point_d.theta));
      point_d.y = point_c.y +
          slot_sign_ * radius_cd * (-cos_point_c_theta + apa_cos(point_d.theta));

      if (CollideWithObjectsByPolygon(
          point_c, point_d, init_ego_polygon)) {
        // ++collide_cnt;
        break;
      }

      ParallelSegmentsInfo tmp_segments_info;
      if (!DESegment(point_d, false, is_rough_calc, &tmp_segments_info)) {
        // ++de_fail_cnt;
        continue;
      }

      const double len_cd =
          std::fabs(point_c.theta - point_d.theta) * radius_cd;
      const double len_cost = 0.0;
      const double radius_cost = CalRadiusCost(radius_cd, len_cd);
      const double total_cost = len_cost + radius_cost +
          tmp_segments_info.total_cost;
      if (std::isinf(total_cost)) {
        // ++total_cost_nan_cnt;
        continue;
      }

      if (total_cost < segments_info->total_cost) {
        segments_info->total_cost = total_cost;

        // cd
        segments_info->opt_radius_cd = radius_cd;
        segments_info->opt_point_d = point_d;

        // de
        segments_info->opt_point_e = tmp_segments_info.opt_point_e;

        // ef
        segments_info->opt_radius_ef = tmp_segments_info.opt_radius_ef;
        segments_info->opt_point_f = tmp_segments_info.opt_point_f;

        // fg
        segments_info->opt_radius_fg = tmp_segments_info.opt_radius_fg;
        segments_info->opt_point_g = tmp_segments_info.opt_point_g;

        // fh
        segments_info->opt_radius_fh = tmp_segments_info.opt_radius_fh;
        segments_info->opt_point_h = tmp_segments_info.opt_point_h;

        if (is_rough_calc) {
          return true;
        }
      }

    }
  }

  // if (is_start) {
  //   PLANNING_LOG << "total_cnt:" << (radius_num * point_num)
  //       << ", collide_cnt:" << collide_cnt
  //       << ", de_fail_cnt:" << de_fail_cnt
  //       << ", total_cost_nan_cnt:" << total_cost_nan_cnt
  //       << std::endl;
  // }

  if (std::isinf(segments_info->total_cost)) {
    return false;
  }

  return true;
}

bool ParallelInGeometryPlan::DESegment(const PlanningPoint &point_d,
    bool is_start, bool is_rough_calc, ParallelSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  const double max_de_len = 5.0;

  const double l_step = 0.2;
  const int point_num = std::max(static_cast<int>(max_de_len / l_step), 1);

  const double cos_point_d_theta = apa_cos(point_d.theta);
  const double sin_point_d_theta = apa_sin(point_d.theta);

  PlanningPoint point_e;
  // size_t collide_cnt = 0;
  // size_t ef_fail_cnt = 0;
  // size_t total_cost_nan_cnt = 0;
  const double front_buffer = 0.0;
  const double rear_buffer = kLonBuffer;
  const double lat_buffer = kLatBuffer;
  Polygon2d init_ego_polygon = std::move(ConstructVehiclePolygonWithBuffer(
        point_d, front_buffer, rear_buffer, lat_buffer));
  for (int i = 0; i < point_num; ++i) {
    const double l_de = l_step * i;
    point_e.x = point_d.x - l_de * cos_point_d_theta;
    if (point_e.x < target_point_.x - kBackwardOverLen) {
      break;
    }

    point_e.y = point_d.y - l_de * sin_point_d_theta;
    point_e.theta = point_d.theta;

    if (CollideWithObjectsByPolygon(point_d, point_e, init_ego_polygon)) {
      // ++collide_cnt;
      break;
    }

    ParallelSegmentsInfo tmp_segments_info;
    if (!EFSegment(point_e, false, is_rough_calc, &tmp_segments_info)) {
      // ++ef_fail_cnt;
      continue;
    }

    const double len_cost = 0.0;
    const double total_cost = len_cost + tmp_segments_info.total_cost;
    if (std::isinf(total_cost)) {
      // ++total_cost_nan_cnt;
      continue;
    }
    if (total_cost < segments_info->total_cost) {
      segments_info->total_cost = total_cost;

      // de
      segments_info->opt_point_e = point_e;

      // ef
      segments_info->opt_radius_ef = tmp_segments_info.opt_radius_ef;
      segments_info->opt_point_f = tmp_segments_info.opt_point_f;

      // fg
      segments_info->opt_radius_fg = tmp_segments_info.opt_radius_fg;
      segments_info->opt_point_g = tmp_segments_info.opt_point_g;

      // fh
      segments_info->opt_radius_fh = tmp_segments_info.opt_radius_fh;
      segments_info->opt_point_h = tmp_segments_info.opt_point_h;

      if (is_rough_calc) {
        return true;
      }
    }

  }

  // PLANNING_LOG << "total_cnt:" << point_num
  //     << ", collide_cnt:" << collide_cnt
  //     << ", ef_fail_cnt:" << ef_fail_cnt
  //     << ", total_cost_nan_cnt:" << total_cost_nan_cnt
  //     << std::endl;

  if (std::isinf(segments_info->total_cost)) {
    return false;
  }

  return true;
}

bool ParallelInGeometryPlan::EFSegment(const PlanningPoint &point_e,
    bool is_start, bool is_rough_calc, ParallelSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  ParallelSegmentsInfo tmp_segments_info;
  if (EF1Segment(point_e, is_start, is_rough_calc, &tmp_segments_info)) {
    segments_info->total_cost = tmp_segments_info.total_cost;
    segments_info->opt_radius_ef = tmp_segments_info.opt_radius_ef;
    segments_info->opt_point_f = tmp_segments_info.opt_point_f;
    // PLANNING_LOG << "ef1 success" << std::endl;
    return true;
  }

  if (EF2Segment(point_e, is_start, is_rough_calc, &tmp_segments_info)) {
    segments_info->total_cost = tmp_segments_info.total_cost + kSegmentCost;
    segments_info->opt_radius_ef = tmp_segments_info.opt_radius_ef;
    segments_info->opt_point_f = tmp_segments_info.opt_point_f;
    // PLANNING_LOG << "ef2 success" << std::endl;
    return true;
  }

  if (EF3Segment(point_e, is_start, is_rough_calc, &tmp_segments_info)) {
    segments_info->total_cost =
        tmp_segments_info.total_cost + 2.0 * kSegmentCost;
    segments_info->opt_radius_ef = tmp_segments_info.opt_radius_ef;
    segments_info->opt_point_f = tmp_segments_info.opt_point_f;
    // PLANNING_LOG << "ef3 success" << std::endl;
    return true;
  }

  return false;
}

bool ParallelInGeometryPlan::EF1Segment(const PlanningPoint &point_e,
    bool is_start, bool is_rough_calc, ParallelSegmentsInfo *segments_info) {
  if (is_start
      && std::fabs(point_e.theta - target_point_.theta) <= kMaxThetaDiff
      && std::fabs(point_e.y - target_point_.y) <= kMaxYOffset) {
    segments_info->total_cost = 0.0;
    segments_info->opt_point_f.x = target_point_.x;
    segments_info->opt_point_f.y = point_e.y;
    segments_info->opt_point_f.theta = target_point_.theta;
    segments_info->opt_radius_ef = std::numeric_limits<double>::infinity();
    return true;
  }

  segments_info->total_cost = std::numeric_limits<double>::infinity();

  const double sin_point_e_theta = apa_sin(point_e.theta);
  const double cos_point_e_theta = apa_cos(point_e.theta);

  PlanningPoint point_f;
  point_f.theta = target_point_.theta;
  size_t radius_cnt1 = 0;
  size_t fx_cnt1 = 0;
  size_t collide_cnt1 = 0;
  const double front_buffer = is_start ? kLonReverseBuffer : 0.0;
  const double rear_buffer = kLonBuffer;
  const double lat_buffer = kLatBuffer;
  Polygon2d init_ego_polygon = std::move(ConstructVehiclePolygonWithBuffer(
        point_e, front_buffer, rear_buffer, lat_buffer));
  // const std::vector<double> target_y_vec =
  //     is_start ? target_y_vec_ : std::vector<double>{target_point_.y};
  for (const double y : target_y_vec_) {
    point_f.y = y;
    const double radius_ef_tmp = slot_sign_ * (-point_e.y + point_f.y) /
        (cos_point_e_theta - cos_target_point_theta_);
    if (radius_ef_tmp < min_turn_radius_) {
      ++radius_cnt1;
      continue;
    }

    point_f.x = point_e.x + slot_sign_ * radius_ef_tmp *
        (-sin_point_e_theta + sin_target_point_theta_);

    if (point_f.x < target_point_.x - kBackwardOverLen) {
      ++fx_cnt1;
      continue;
    }

    if (point_f.x > target_point_.x + kForwardOverLen) {
      ++fx_cnt1;
      continue;
    }

    if (CollideWithObjectsByPolygon(
        point_e, point_f, radius_ef_tmp, init_ego_polygon)) {
      ++collide_cnt1;
      continue;
    }

    PlanningPoint updated_target_point(target_point_);
    const double total_cost = CalEFSegmentCost(
        point_e, point_f, radius_ef_tmp, &updated_target_point);
    if (total_cost < segments_info->total_cost) {
      segments_info->total_cost = total_cost;

      segments_info->opt_radius_ef = radius_ef_tmp;
      segments_info->opt_point_f = point_f;

      updated_target_point_ = updated_target_point;

      if (is_rough_calc) {
        return true;
      }
    }
  }

  if (is_start) {
    PLANNING_LOG << "total_cnt1:" << target_y_vec_.size()
        << ", radius_cnt1:" << radius_cnt1
        << ", fx_cnt1:" << fx_cnt1
        << ", collide_cnt1:" << collide_cnt1
        << std::endl;
  }

  const int radius_ef_num = is_start ? 10 : 1;
  const double radius_step = 1.0;
  size_t fy_cnt2 = 0;
  size_t fx_cnt2 = 0;
  size_t collide_cnt2 = 0;
  for (int i = 0; i < radius_ef_num; ++i) {
    const double radius_ef_tmp = min_turn_radius_ + radius_step * i;
    point_f.y = point_e.y + slot_sign_ * radius_ef_tmp *
        (cos_point_e_theta - cos_target_point_theta_);
    if (std::fabs(point_f.y - target_point_.y) > kMaxYOffset) {
      ++fy_cnt2;
      continue;
    }
    point_f.x = point_e.x + slot_sign_ * radius_ef_tmp *
        (-sin_point_e_theta + sin_target_point_theta_);

    if (point_f.x < target_point_.x - kBackwardOverLen) {
      ++fx_cnt2;
      continue;
    }

    if (point_f.x > target_point_.x + kForwardOverLen) {
      ++fx_cnt2;
      continue;
    }

    if (CollideWithObjectsByPolygon(
        point_e, point_f, radius_ef_tmp, init_ego_polygon)) {
      ++collide_cnt2;
      continue;
    }

    PlanningPoint updated_target_point(target_point_);
    const double total_cost = CalEFSegmentCost(
        point_e, point_f, radius_ef_tmp, &updated_target_point);
    if (total_cost < segments_info->total_cost) {
      segments_info->total_cost = total_cost;

      segments_info->opt_radius_ef = radius_ef_tmp;
      segments_info->opt_point_f = point_f;

      updated_target_point_ = updated_target_point;

      if (is_rough_calc) {
        return true;
      }
    }
  }

  if (is_start) {
    PLANNING_LOG << "total_cnt2:" << radius_ef_num
        << ", fy_cnt2:" << fy_cnt2
        << ", fx_cnt2:" << fx_cnt2
        << ", collide_cnt2:" << collide_cnt2
        << std::endl;
  }

  if (std::isinf(segments_info->total_cost)) {
    return false;
  }

  return true;
}

bool ParallelInGeometryPlan::EF2Segment(const PlanningPoint &point_e,
    bool is_start, bool is_rough_calc, ParallelSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  const double sin_point_e_theta = apa_sin(point_e.theta);
  const double cos_point_e_theta = apa_cos(point_e.theta);
  if (point_e.theta * slot_sign_ > M_PI_2) {
    return false;
  }

  PlanningPoint point_f;
  const double radius_step = 1.0;
  const int size_i_ef = is_start ? 10 : 1;
  const int size_i_fg = is_start ? 10 : 1;
  const double front_buffer = is_start ? kLonReverseBuffer : 0.0;
  const double rear_buffer = kLonBuffer;
  const double lat_buffer = kLatBuffer;
  Polygon2d init_ego_polygon = std::move(ConstructVehiclePolygonWithBuffer(
        point_e, front_buffer, rear_buffer, lat_buffer));
  for (int i = 0; i < size_i_ef; ++i) {
    const double radius_ef_tmp = min_turn_radius_ + radius_step * i;
    for (int j = 0; j < size_i_fg; ++j) {
      const double radius_fg_tmp = min_turn_radius_ + radius_step * j;
      const double cos_f_tmp = ((point_e.y - target_point_.y) * slot_sign_ +
              radius_ef_tmp * cos_point_e_theta +
              radius_fg_tmp * cos_target_point_theta_) /
          (radius_ef_tmp + radius_fg_tmp);

      if (fabs(cos_f_tmp) > 1.0) {
        continue;
      }

      point_f.theta = acos(cos_f_tmp);
      if (point_f.theta * slot_sign_ < 0.0) {
        continue;
      }

      const double cos_point_f_theta = cos_f_tmp;
      const double sin_point_f_theta = apa_sin(point_f.theta);
      point_f.x = point_e.x +
          slot_sign_ * radius_ef_tmp * (-sin_point_e_theta + sin_point_f_theta);
      if (point_f.x < target_point_.x - kBackwardOverLen) {
        continue;
      }
      point_f.y = point_e.y +
          slot_sign_ * radius_ef_tmp * (cos_point_e_theta - cos_point_f_theta);

      if (CollideWithObjectsByPolygon(
          point_e, point_f, radius_ef_tmp, init_ego_polygon)) {
        continue;
      }

      if (point_f.y * slot_sign_ > target_point_.y * slot_sign_ + kMaxYOffset) {
        continue;
      }

      ParallelSegmentsInfo tmp_segments_info;
      if (!FGSegment(point_f, false, is_rough_calc, &tmp_segments_info)) {
        continue;
      }

      if (std::isinf(tmp_segments_info.total_cost)) {
        continue;
      }

      const double len_ef =
          std::fabs(point_e.theta - point_f.theta) * radius_ef_tmp;

      const double len_cost = 0.0;
      const double radius_cost = CalRadiusCost(radius_ef_tmp, len_ef);

      const double total_cost =
          len_cost + radius_cost + tmp_segments_info.total_cost;
      if (std::isinf(total_cost)) {
        continue;
      }

      if (total_cost < segments_info->total_cost) {
        segments_info->total_cost = total_cost;

        // ef
        segments_info->opt_radius_ef = radius_ef_tmp;
        segments_info->opt_point_f = point_f;

        // fg
        segments_info->opt_radius_fg = tmp_segments_info.opt_radius_fg;
        segments_info->opt_point_g = tmp_segments_info.opt_point_g;

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

bool ParallelInGeometryPlan::FGSegment(const PlanningPoint &point_f,
    bool is_start, bool is_rough_calc, ParallelSegmentsInfo *segments_info) {
  if (is_start
      && std::fabs(point_f.theta - target_point_.theta) <= kMaxThetaDiff
      && std::fabs(point_f.y - target_point_.y) <= kMaxYOffset) {
    segments_info->total_cost = 0.0;
    segments_info->opt_point_g.x = target_point_.x;
    segments_info->opt_point_g.y = point_f.y;
    segments_info->opt_point_g.theta = target_point_.theta;
    segments_info->opt_radius_fg = std::numeric_limits<double>::infinity();
    return true;
  }

  segments_info->total_cost = std::numeric_limits<double>::infinity();

  const double sin_point_f_theta = apa_sin(point_f.theta);
  const double cos_point_f_theta = apa_cos(point_f.theta);

  PlanningPoint point_g;
  point_g.theta = target_point_.theta;
  size_t radius_cnt1 = 0;
  size_t collide_cnt1 = 0;
  const double front_buffer = kLonBuffer;
  const double rear_buffer = is_start ? kLonReverseBuffer : 0.0;
  const double lat_buffer = kLatBuffer;
  Polygon2d init_ego_polygon = std::move(ConstructVehiclePolygonWithBuffer(
        point_f, front_buffer, rear_buffer, lat_buffer));
  // const std::vector<double> target_y_vec =
  //     is_start ? target_y_vec_ : std::vector<double>{target_point_.y};
  for (const double y : target_y_vec_) {
    point_g.y = y;
    const double radius_fg_tmp = slot_sign_ * (point_f.y - point_g.y) /
        (cos_point_f_theta - cos_target_point_theta_);
    if (radius_fg_tmp < min_turn_radius_) {
      ++radius_cnt1;
      continue;
    }

    point_g.x = point_f.x + slot_sign_ * radius_fg_tmp *
        (sin_point_f_theta - sin_target_point_theta_);

    if (CollideWithObjectsByPolygon(
        point_f, point_g, radius_fg_tmp, init_ego_polygon)) {
      ++collide_cnt1;
      continue;
    }

    PlanningPoint updated_target_point(target_point_);
    const double total_cost = CalFGSegmentCost(
        point_f, point_g, radius_fg_tmp, &updated_target_point);
    if (total_cost < segments_info->total_cost) {
      segments_info->total_cost = total_cost;
      segments_info->opt_point_g = point_g;
      segments_info->opt_radius_fg = radius_fg_tmp;
      updated_target_point_ = updated_target_point;

      if (is_rough_calc) {
        return true;
      }
    }
  }
  if (is_start) {
    PLANNING_LOG << "total_cnt1:" << target_y_vec_.size()
        << ", fg_radius_cnt1:" << radius_cnt1
        << ", collide_cnt1:" << collide_cnt1
        << std::endl;
  }

  const int radius_fg_num = is_start ? 10 : 1;
  const double radius_step = 1.0;
  size_t gx_cnt2 = 0;
  size_t collide_cnt2 = 0;
  for (int i = 0; i < radius_fg_num; ++i) {
    const double radius_fg_tmp = min_turn_radius_ + radius_step * i;
    point_g.y = point_f.y + slot_sign_ * radius_fg_tmp *
        (-cos_point_f_theta + cos_target_point_theta_);
    if (std::fabs(point_g.y - target_point_.y) > kMaxYOffset) {
      ++gx_cnt2;
      continue;
    }
    point_g.x = point_f.x + slot_sign_ * radius_fg_tmp *
        (sin_point_f_theta - sin_target_point_theta_);

    if (CollideWithObjectsByPolygon(
        point_f, point_g, radius_fg_tmp, init_ego_polygon)) {
      ++collide_cnt2;
      continue;
    }

    PlanningPoint updated_target_point(target_point_);
    const double total_cost = CalFGSegmentCost(
        point_f, point_g, radius_fg_tmp, &updated_target_point);
    if (total_cost < segments_info->total_cost) {
      segments_info->total_cost = total_cost;
      segments_info->opt_point_g = point_g;
      segments_info->opt_radius_fg = radius_fg_tmp;
      updated_target_point_ = updated_target_point;

      if (is_rough_calc) {
        return true;
      }
    }
  }
  if (is_start) {
    PLANNING_LOG << "total_cnt2:" << radius_fg_num
        << ", gx_cnt2:" << gx_cnt2
        << ", collide_cnt2:" << collide_cnt2
        << std::endl;
  }

  if (std::isinf(segments_info->total_cost)) {
    return false;
  }

  return true;
}

bool ParallelInGeometryPlan::EF3Segment(const PlanningPoint &point_e,
    bool is_start, bool is_rough_calc, ParallelSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  if (point_e.theta * slot_sign_ >= M_PI_4) {
    return false;
  }

  const double max_point_f_theta = 0.0;
  const double yaw_step = is_start ? -kMinThetaStep * slot_sign_ :
      -kDefaultThetaStep * slot_sign_;
  const int point_num = std::max(
      static_cast<int>((max_point_f_theta - point_e.theta) / yaw_step), 1);

  const double cos_point_e_theta = apa_cos(point_e.theta);
  const double sin_point_e_theta = apa_sin(point_e.theta);

  PlanningPoint point_f;
  size_t fx_cnt1 = 0;
  size_t fx_cnt2 = 0;
  size_t collide_cnt = 0;
  size_t fy_cnt1 = 0;
  size_t fy_cnt2 = 0;
  size_t total_cost_nan_cnt = 0;
  const int radius_ef_num = is_start ? 20 : 1;
  const double radius_step = 1.0;
  const double front_buffer = is_start ? kLonReverseBuffer : 0.0;
  const double rear_buffer = kLonBuffer;
  const double lat_buffer = kLatBuffer;
  Polygon2d init_ego_polygon = std::move(ConstructVehiclePolygonWithBuffer(
        point_e, front_buffer, rear_buffer, lat_buffer));
  for (int i = 0; i < radius_ef_num; ++i) {
    const double radius_ef = min_turn_radius_ + radius_step * i;
    for (int j = 0; j < point_num; ++j) {
      point_f.theta = point_e.theta + yaw_step * j;
      point_f.x = point_e.x +
          slot_sign_ * radius_ef * (-sin_point_e_theta + apa_sin(point_f.theta));
      if (point_f.x < target_point_.x - kBackwardOverLen) {
        ++fx_cnt1;
        break;
      }

      point_f.y = point_e.y +
          slot_sign_ * radius_ef * (cos_point_e_theta - apa_cos(point_f.theta));

      if (CollideWithObjectsByPolygon(point_e, point_f, init_ego_polygon)) {
        ++collide_cnt;
        break;
      }

      if (point_f.y * slot_sign_ > target_point_.y * slot_sign_ + kMaxYOffset) {
        ++fy_cnt1;
        continue;
      }

      const double len_ef =
          std::fabs(point_e.theta - point_f.theta) * radius_ef;
      const double len_cost = is_start ? CalSegmentLengthCost(len_ef) : 0.0;
      const double radius_cost = CalRadiusCost(radius_ef, len_ef);
      // const double x_cost = CalEndXCost(point_f.x, false);
      // const double total_cost = len_cost + radius_cost + x_cost;
      const double theta_cost = CalEndThetaCost(point_f.theta);
      const double y_cost = CalEndYCost(point_f.y);
      const double total_cost = len_cost + radius_cost + theta_cost + y_cost;

      if (std::isinf(total_cost)) {
        ++total_cost_nan_cnt;
        continue;
      }
      if (total_cost < segments_info->total_cost) {
        segments_info->total_cost = total_cost;

        // ef
        segments_info->opt_radius_ef = radius_ef;
        segments_info->opt_point_f = point_f;

        if (is_rough_calc) {
          return true;
        }
      }

    }
  }

  if (is_start) {
    PLANNING_LOG << "total_cnt:" << (point_num * radius_ef_num)
        << ", collide_cnt:" << collide_cnt
        << ", fx_cnt1:" << fx_cnt1
        << ", fx_cnt2:" << fx_cnt2
        << ", fy_cnt1:" << fy_cnt1
        << ", fy_cnt2:" << fy_cnt2
        << ", total_cost_nan_cnt:" << total_cost_nan_cnt
        << std::endl;
  }

  if (std::isinf(segments_info->total_cost)) {
    return false;
  }

  return true;
}

bool ParallelInGeometryPlan::FHSegment(const PlanningPoint &point_f,
    bool is_start, bool is_rough_calc, ParallelSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  ParallelSegmentsInfo tmp_segments_info;
  if (FH1Segment(point_f, true, is_rough_calc, &tmp_segments_info)) {
    segments_info->total_cost = tmp_segments_info.total_cost;
    segments_info->opt_radius_fh = tmp_segments_info.opt_radius_fh;
    segments_info->opt_point_h = tmp_segments_info.opt_point_h;
    // PLANNING_LOG << "fh1 success" << std::endl;
    return true;
  }

  if (FH2Segment(point_f, true, is_rough_calc, &tmp_segments_info)) {
    segments_info->total_cost = tmp_segments_info.total_cost + kSegmentCost;
    segments_info->opt_radius_fh = tmp_segments_info.opt_radius_fh;
    segments_info->opt_point_h = tmp_segments_info.opt_point_h;
    // PLANNING_LOG << "fh2 success" << std::endl;
    return true;
  }

  if (FH3Segment(point_f, true, is_rough_calc, &tmp_segments_info)) {
    segments_info->total_cost =
        tmp_segments_info.total_cost + 2.0 * kSegmentCost;
    segments_info->opt_radius_fh = tmp_segments_info.opt_radius_fh;
    segments_info->opt_point_h = tmp_segments_info.opt_point_h;
    // PLANNING_LOG << "fh3 success" << std::endl;
    return true;
  }

  return false;
}

bool ParallelInGeometryPlan::FH1Segment(const PlanningPoint &point_f,
    bool is_start, bool is_rough_calc, ParallelSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  ParallelSegmentsInfo tmp_segments_info;
  if (!FGSegment(point_f, is_start, is_rough_calc, &tmp_segments_info)) {
    return false;
  }

  *segments_info = tmp_segments_info;
  segments_info->opt_radius_fh = tmp_segments_info.opt_radius_fg;
  segments_info->opt_point_h = tmp_segments_info.opt_point_g;

  return true;
}

bool ParallelInGeometryPlan::FH2Segment(const PlanningPoint &point_f,
    bool is_start, bool is_rough_calc, ParallelSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  const double sin_point_f_theta = apa_sin(point_f.theta);
  const double cos_point_f_theta = apa_cos(point_f.theta);
  if (point_f.theta * slot_sign_ > M_PI_4) {
    return false;
  }

  PlanningPoint point_h;
  const double radius_step = 1.0;
  const int size_i_fh = is_start ? 10 : 1;
  const int size_i_hi = is_start ? 10 : 1;
  const double front_buffer = kLonBuffer;
  const double rear_buffer = is_start ? kLonReverseBuffer : 0.0;
  const double lat_buffer = kLatBuffer;
  Polygon2d init_ego_polygon = std::move(ConstructVehiclePolygonWithBuffer(
        point_f, front_buffer, rear_buffer, lat_buffer));
  for (int i = 0; i < size_i_fh; ++i) {
    const double radius_fh_tmp = min_turn_radius_ + radius_step * i;
    for (int j = 0; j < size_i_hi; ++j) {
      const double radius_hi_tmp = min_turn_radius_ + radius_step * j;
      const double cos_h_tmp = ((-point_f.y + target_point_.y) * slot_sign_ +
              radius_fh_tmp * cos_point_f_theta +
              radius_hi_tmp * cos_target_point_theta_) /
          (radius_fh_tmp + radius_hi_tmp);

      if (fabs(cos_h_tmp) > 1.0) {
        continue;
      }

      point_h.theta = acos(cos_h_tmp);
      if (point_h.theta * slot_sign_ < 0.0) {
        continue;
      }

      const double cos_point_h_theta = cos_h_tmp;
      const double sin_point_h_theta = apa_sin(point_h.theta);
      point_h.x = point_f.x +
          slot_sign_ * radius_fh_tmp * (sin_point_f_theta - sin_point_h_theta);
      point_h.y = point_f.y +
          slot_sign_ * radius_fh_tmp * (-cos_point_f_theta + cos_point_h_theta);

      if (CollideWithObjectsByPolygon(
          point_f, point_h, radius_fh_tmp, init_ego_polygon)) {
        continue;
      }

      if (point_h.y * slot_sign_ < target_point_.y * slot_sign_ - kMaxYOffset) {
        continue;
      }

      ParallelSegmentsInfo tmp_segments_info;
      if (!HISegment(point_h, false, is_rough_calc, &tmp_segments_info)) {
        continue;
      }

      if (std::isinf(tmp_segments_info.total_cost)) {
        continue;
      }

      const double len_fh =
          std::fabs(point_f.theta - point_h.theta) * radius_fh_tmp;

      const double len_cost = CalSegmentLengthCost(len_fh);
      const double radius_cost = CalRadiusCost(radius_fh_tmp, len_fh);

      const double total_cost =
          len_cost + radius_cost + tmp_segments_info.total_cost;
      if (std::isinf(total_cost)) {
        continue;
      }

      if (total_cost < segments_info->total_cost) {
        segments_info->total_cost = total_cost;

        // fh
        segments_info->opt_radius_fh = radius_fh_tmp;
        segments_info->opt_point_h = point_h;

        // hi
        segments_info->opt_radius_hi = tmp_segments_info.opt_radius_hi;
        segments_info->opt_point_i = tmp_segments_info.opt_point_i;

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

bool ParallelInGeometryPlan::HISegment(const PlanningPoint &point_f,
    bool is_start, bool is_rough_calc, ParallelSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  ParallelSegmentsInfo tmp_segments_info;
  if (!EF1Segment(point_f, false, is_rough_calc, &tmp_segments_info)) {
    return false;
  }

  *segments_info = tmp_segments_info;
  segments_info->opt_radius_hi = tmp_segments_info.opt_radius_ef;
  segments_info->opt_point_i = tmp_segments_info.opt_point_f;

  return true;
}

bool ParallelInGeometryPlan::FH3Segment(const PlanningPoint &point_f,
    bool is_start, bool is_rough_calc, ParallelSegmentsInfo *segments_info) {
  segments_info->total_cost = std::numeric_limits<double>::infinity();

  if (point_f.theta * slot_sign_ >= M_PI_4) {
    return false;
  }

  const double max_point_h_theta = 0.0;
  const double yaw_step = is_start ? -kMinThetaStep * slot_sign_ :
      -kDefaultThetaStep * slot_sign_;
  const int point_num = std::max(
      static_cast<int>((max_point_h_theta - point_f.theta) / yaw_step), 1);

  const double cos_point_f_theta = apa_cos(point_f.theta);
  const double sin_point_f_theta = apa_sin(point_f.theta);

  PlanningPoint point_h;
  size_t h3x_cnt1 = 0;
  size_t h3x_cnt2 = 0;
  size_t collide_cnt = 0;
  size_t h3y_cnt1 = 0;
  size_t h3y_cnt2 = 0;
  size_t total_cost_nan_cnt = 0;
  const int radius_fh_num = is_start ? 20 : 1;
  const double radius_step = 1.0;
  const double front_buffer = kLonBuffer;
  const double rear_buffer = is_start ? kLonReverseBuffer : 0.0;
  const double lat_buffer = kLatBuffer;
  Polygon2d init_ego_polygon = std::move(ConstructVehiclePolygonWithBuffer(
        point_f, front_buffer, rear_buffer, lat_buffer));
  for (int i = 0; i < radius_fh_num; ++i) {
    const double radius_fh = min_turn_radius_ + radius_step * i;
    for (int j = 0; j < point_num; ++j) {
      point_h.theta = point_f.theta + yaw_step * j;
      point_h.x = point_f.x +
          slot_sign_ * radius_fh * (sin_point_f_theta - apa_sin(point_h.theta));

      point_h.y = point_f.y +
          slot_sign_ * radius_fh * (-cos_point_f_theta + apa_cos(point_h.theta));

      if (CollideWithObjectsByPolygon(point_f, point_h, init_ego_polygon)) {
        ++collide_cnt;
        break;
      }

      if (point_h.y * slot_sign_ < target_point_.y * slot_sign_ - kMaxYOffset) {
        ++h3y_cnt1;
        continue;
      }

      const double len_fh =
          std::fabs(point_f.theta - point_h.theta) * radius_fh;
      const double len_cost = is_start ? CalSegmentLengthCost(len_fh) : 0.0;
      const double radius_cost = CalRadiusCost(radius_fh, len_fh);
      // const double x_cost = CalEndXCost(point_h.x, true);
      // const double total_cost = len_cost + radius_cost + x_cost;
      const double theta_cost = CalEndThetaCost(point_h.theta);
      const double total_cost = len_cost + radius_cost + theta_cost;
      if (std::isinf(total_cost)) {
        ++total_cost_nan_cnt;
        continue;
      }
      if (total_cost < segments_info->total_cost) {
        segments_info->total_cost = total_cost;

        // fh
        segments_info->opt_radius_fh = radius_fh;
        segments_info->opt_point_h = point_h;

        if (is_rough_calc) {
          return true;
        }
      }

    }
  }

  if (is_start) {
    PLANNING_LOG << "total_cnt:" << (point_num * radius_fh_num)
        << ", collide_cnt:" << collide_cnt
        << ", h3x_cnt1:" << h3x_cnt1
        << ", h3x_cnt2:" << h3x_cnt2
        << ", h3y_cnt1:" << h3y_cnt1
        << ", h3y_cnt2:" << h3y_cnt2
        << ", total_cost_nan_cnt:" << total_cost_nan_cnt
        << std::endl;
  }

  if (std::isinf(segments_info->total_cost)) {
    return false;
  }

  return true;
}

bool ParallelInGeometryPlan::CollideWithObjectsByBox(
    const PlanningPoint &veh_point, const double front_buffer,
    const double rear_buffer, const double lat_buffer) const {
  if (objects_map_line_segments_.empty()) {
    return false;
  }

  const double front_edge_to_center_with_safe_dst =
      front_edge_to_center_ + front_buffer;
  const double back_edge_to_center_with_safe_dst =
      back_edge_to_center_ + rear_buffer;
  const double shift_distance = (front_edge_to_center_with_safe_dst -
      back_edge_to_center_with_safe_dst) * 0.5;
  const double veh_length_with_safe_dst =
      front_edge_to_center_with_safe_dst + back_edge_to_center_with_safe_dst;
  const double veh_width_with_safe_dis = width_veh_ + lat_buffer * 2.0;
  double veh_x = veh_point.x + shift_distance * apa_cos(veh_point.theta);
  double veh_y = veh_point.y + shift_distance * apa_sin(veh_point.theta);
  Box2d ego_box({veh_x, veh_y}, veh_point.theta,
      veh_length_with_safe_dst, veh_width_with_safe_dis);
  for (const auto &line_segment : objects_map_line_segments_) {
    if (ego_box.HasOverlap(line_segment)) {
      return true;
    }
  }

  return false;
}

bool ParallelInGeometryPlan::CollideWithObjectsByBox(
    const PlanningPoint &veh_point1, const PlanningPoint &veh_point2,
    const double radius, const double front_buffer,
    const double rear_buffer, const double lat_buffer) const {
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
  const double shift_distance = (front_edge_to_center_with_safe_dst
      - back_edge_to_center_with_safe_dst) * 0.5;
  const double veh_length_with_safe_dst =
      front_edge_to_center_with_safe_dst + back_edge_to_center_with_safe_dst;
  const double veh_width_with_safe_dis = width_veh_ + lat_buffer * 2.0;

  for (int i = 0; i <= size_theta; ++i) {
    const double veh_theta = veh_point1.theta + theta_sign * yaw_step * i;
    const double sin_veh_theta = apa_sin(veh_theta);
    const double cos_veh_theta = apa_cos(veh_theta);
    double veh_x = veh_point1.x +
        x_sign * radius * std::fabs(-sin_point_1_theta + sin_veh_theta) +
        shift_distance * cos_veh_theta;
    double veh_y = veh_point1.y +
        y_sign * radius * std::fabs(cos_point_1_theta - cos_veh_theta) +
        shift_distance * sin_veh_theta;
    Box2d ego_box({veh_x, veh_y}, veh_theta,
        veh_length_with_safe_dst, veh_width_with_safe_dis);
    for (const auto &line_segment : objects_map_line_segments_) {
      if (ego_box.HasOverlap(line_segment)) {
        return true;
      }
    }
  }

  return false;
}

bool ParallelInGeometryPlan::CollideWithObjectsByPolygon(
    const PlanningPoint &veh_point1, const PlanningPoint &veh_point2,
    const Polygon2d& init_ego_polygon) const {
  if (objects_map_line_segments_.empty()) {
    return false;
  }

  Polygon2d ego_polygon(init_ego_polygon);
  const double rotate_angle = veh_point2.theta - veh_point1.theta;
  const double sin_rotate_angle = apa_sin(rotate_angle);
  const double cos_rotate_angle = apa_cos(rotate_angle);
  ego_polygon.RotateAndTranslate(Vec2d(veh_point1.x, veh_point1.y),
      sin_rotate_angle, cos_rotate_angle,
      Vec2d(veh_point2.x - veh_point1.x, veh_point2.y - veh_point1.y));

  for (const auto &line_segment : objects_map_line_segments_) {
    if (ego_polygon.HasOverlap(line_segment)) {
      return true;
    }
  }

  return false;
}

bool ParallelInGeometryPlan::CollideWithObjectsByPolygon(
    const PlanningPoint &veh_point1, const PlanningPoint &veh_point2,
    const double radius, const Polygon2d& init_ego_polygon) const {
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
    veh_point.x = veh_point1.x +
        x_sign * radius * std::fabs(-sin_point_1_theta + sin_veh_theta);
    veh_point.y = veh_point1.y +
        y_sign * radius * std::fabs(cos_point_1_theta - cos_veh_theta);

    Polygon2d ego_polygon(init_ego_polygon);
    const double rotate_angle = veh_point.theta - veh_point1.theta;
    const double sin_rotate_angle = apa_sin(rotate_angle);
    const double cos_rotate_angle = apa_cos(rotate_angle);
    ego_polygon.RotateAndTranslate(Vec2d(veh_point1.x, veh_point1.y),
        sin_rotate_angle, cos_rotate_angle,
        Vec2d(veh_point.x - veh_point1.x, veh_point.y - veh_point1.y));

    for (const auto &line_segment : objects_map_line_segments_) {
      if (ego_polygon.HasOverlap(line_segment)) {
        return true;
      }
    }
  }

  return false;
}

bool ParallelInGeometryPlan::CEndCollideCheck(const PlanningPoint &point_c,
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

double ParallelInGeometryPlan::CalPointYBiasCost(const double y_bias) const {
  return kPointyBiasWeight * y_bias;
}

double ParallelInGeometryPlan::CalSegmentLengthCost(
    const double segment_len) const {
  if (segment_len < min_segment_len_) {
    return std::numeric_limits<double>::infinity();
  } else {
    return kSegmentLengthWeight * segment_len;
  }

  return std::numeric_limits<double>::infinity();
}

double ParallelInGeometryPlan::CalRadiusCost(const double radius,
    const double len) const {
  if (radius < min_turn_radius_) {
    return std::numeric_limits<double>::infinity();
  } else {
    return kRadiusWeight * len / radius;
  }

  return std::numeric_limits<double>::infinity();
}

double ParallelInGeometryPlan::CalEndXCost(const double x,
    const bool is_forward) const {
  if (is_forward) {
    return (target_point_.x - x) * kEndXWeight;
  } else {
    return -(target_point_.x - x) * kEndXWeight;
  }
  return 0.0;
}

double ParallelInGeometryPlan::CalEndYCost(const double y) const {
  return (y - target_point_.y) * kEndYWeight * slot_sign_;
}

double ParallelInGeometryPlan::CalEndThetaCost(const double theta) const {
  return std::fabs(target_point_.theta - theta) * kEndThetaWeight;
}

void ParallelInGeometryPlan::SetTargetPoint(const PlanningPoint& target_point) {
  target_point_= target_point;
  updated_target_point_ = target_point_;
  CalTargetYVec();
  sin_target_point_theta_ = apa_sin(target_point_.theta);
  cos_target_point_theta_ = apa_cos(target_point_.theta);
}

void ParallelInGeometryPlan::CalTargetYVec() {
  target_y_vec_.clear();
  target_y_vec_.reserve(kTargetYNum);
  const int half_target_y_num = kTargetYNum / 2;
  for (int i = 0; i < kTargetYNum; ++i) {
    const double target_y =
        target_point_.y + (i - half_target_y_num) * kTargetYStep;
    target_y_vec_.push_back(target_y);
    // PLANNING_LOG << "target_y:" << target_y << std::endl;
  }
}

double ParallelInGeometryPlan::CalFGSegmentCost(
    const PlanningPoint& point_f, const PlanningPoint& point_g,
    const double radius_fg, PlanningPoint* const updated_target_point) const {
  const double y_diff = std::fabs(point_g.y - target_point_.y);
  const double bias_cost = CalPointYBiasCost(y_diff);
  const double len_fg =
      std::fabs(point_f.theta - point_g.theta) * radius_fg;
  const double len_g_end = std::fabs(point_g.x - target_point_.x);
  double len_cost = 0.0;
  double gear_shift_cost = 0.0;
  if (point_g.x <= target_point_.x) {
    double total_len = len_fg + len_g_end;
    if (total_len < min_segment_len_) {
      const double extending_len =
          std::min(kMaxXOffset, min_segment_len_ - total_len);
      total_len += extending_len;
      updated_target_point->x = target_point_.x + extending_len;
    }
    len_cost = CalSegmentLengthCost(total_len);
  } else {
    len_cost = CalSegmentLengthCost(len_fg);
    gear_shift_cost = kGearShiftWeight;
  }
  const double radius_cost = CalRadiusCost(radius_fg, len_fg);
  const double total_cost = bias_cost + len_cost + gear_shift_cost +
      radius_cost;
  return total_cost;
}

double ParallelInGeometryPlan::CalEFSegmentCost(
    const PlanningPoint& point_e, const PlanningPoint& point_f,
    const double radius_ef, PlanningPoint* const updated_target_point) const {
  const double y_diff = std::fabs(point_f.y - target_point_.y);
  const double bias_cost = CalPointYBiasCost(y_diff);
  const double len_ef =
      std::fabs(point_e.theta - point_f.theta) * radius_ef;
  const double len_f_end = std::fabs(point_f.x - target_point_.x);
  double len_cost = 0.0;
  double gear_shift_cost = 0.0;
  if (point_f.x >= target_point_.x) {
    double total_len = len_ef + len_f_end;
    if (total_len < min_segment_len_) {
      const double extending_len =
          std::min(kMaxXOffset, min_segment_len_ - total_len);
      total_len += extending_len;
      updated_target_point->x = target_point_.x - extending_len;
    }
    len_cost = CalSegmentLengthCost(total_len);
  } else {
    len_cost = CalSegmentLengthCost(len_ef);
    gear_shift_cost = kGearShiftWeight;
  }
  const double radius_cost = CalRadiusCost(radius_ef, len_ef);
  const double total_cost = bias_cost + len_cost + gear_shift_cost +
      radius_cost;
  return total_cost;
}

} // namespace apa_planner
} // namespace planning