#include "joint_motion_planning_cost.h"

#include "joint_motion_planning_model.h"

using namespace pnc::mathlib;
namespace pnc {
namespace joint_motion_planning {
static const double kEps = 1e-6;

double EgoReferenceCostTerm::GetCost(const ilqr_solver::State &x,
                                     const ilqr_solver::Control &) {
  const double ego_cost_x = 0.5 * cost_config_ptr_->at(W_EGO_REF_X) *
                            (x[EGO_X] - cost_config_ptr_->at(EGO_REF_X)) *
                            (x[EGO_X] - cost_config_ptr_->at(EGO_REF_X));
  const double ego_cost_y = 0.5 * cost_config_ptr_->at(W_EGO_REF_Y) *
                            (x[EGO_Y] - cost_config_ptr_->at(EGO_REF_Y)) *
                            (x[EGO_Y] - cost_config_ptr_->at(EGO_REF_Y));
  const double ego_cost_theta =
      0.5 * cost_config_ptr_->at(W_EGO_REF_THETA) *
      (x[EGO_THETA] - cost_config_ptr_->at(EGO_REF_THETA)) *
      (x[EGO_THETA] - cost_config_ptr_->at(EGO_REF_THETA));
  const double ego_cost_delta =
      0.5 * cost_config_ptr_->at(W_EGO_REF_DELTA) *
      (x[EGO_DELTA] - cost_config_ptr_->at(EGO_REF_DELTA)) *
      (x[EGO_DELTA] - cost_config_ptr_->at(EGO_REF_DELTA));
  const double ego_cost_v = 0.5 * cost_config_ptr_->at(W_EGO_REF_VEL) *
                            (x[EGO_VEL] - cost_config_ptr_->at(EGO_REF_VEL)) *
                            (x[EGO_VEL] - cost_config_ptr_->at(EGO_REF_VEL));
  const double ego_cost_a = 0.5 * cost_config_ptr_->at(W_EGO_REF_ACC) *
                            (x[EGO_ACC] - cost_config_ptr_->at(EGO_REF_ACC)) *
                            (x[EGO_ACC] - cost_config_ptr_->at(EGO_REF_ACC));
  return ego_cost_x + ego_cost_y + ego_cost_theta + ego_cost_delta +
         ego_cost_v + ego_cost_a;
}

void EgoReferenceCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control &,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT &, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) {
  lx(EGO_X) += cost_config_ptr_->at(W_EGO_REF_X) *
               (x[EGO_X] - cost_config_ptr_->at(EGO_REF_X));
  lx(EGO_Y) += cost_config_ptr_->at(W_EGO_REF_Y) *
               (x[EGO_Y] - cost_config_ptr_->at(EGO_REF_Y));
  lx(EGO_THETA) += cost_config_ptr_->at(W_EGO_REF_THETA) *
                   (x[EGO_THETA] - cost_config_ptr_->at(EGO_REF_THETA));
  lx(EGO_DELTA) += cost_config_ptr_->at(W_EGO_REF_DELTA) *
                   (x[EGO_DELTA] - cost_config_ptr_->at(EGO_REF_DELTA));
  lx(EGO_VEL) += cost_config_ptr_->at(W_EGO_REF_VEL) *
                 (x[EGO_VEL] - cost_config_ptr_->at(EGO_REF_VEL));
  lx(EGO_ACC) += cost_config_ptr_->at(W_EGO_REF_ACC) *
                 (x[EGO_ACC] - cost_config_ptr_->at(EGO_REF_ACC));
  lxx(EGO_X, EGO_X) += cost_config_ptr_->at(W_EGO_REF_X);
  lxx(EGO_Y, EGO_Y) += cost_config_ptr_->at(W_EGO_REF_Y);
  lxx(EGO_THETA, EGO_THETA) += cost_config_ptr_->at(W_EGO_REF_THETA);
  lxx(EGO_DELTA, EGO_DELTA) += cost_config_ptr_->at(W_EGO_REF_DELTA);
  lxx(EGO_VEL, EGO_VEL) += cost_config_ptr_->at(W_EGO_REF_VEL);
  lxx(EGO_ACC, EGO_ACC) += cost_config_ptr_->at(W_EGO_REF_ACC);
}

EgoThreeDiscSafeCostTerm::ThreeDiscResult
EgoThreeDiscSafeCostTerm::CalculateThreeDiscDistances(
    const ilqr_solver::State &x) {
  auto calc_three_centers = [](double x, double y, double theta, double length,
                               bool is_ego = true) {
    double L = length / 6;
    double backcenter = is_ego ? L : 3 * L;
    std::array<double, 3> offsets = {L - backcenter, 3 * L - backcenter,
                                     5 * L - backcenter};
    std::array<std::pair<double, double>, 3> centers;
    for (int i = 0; i < 3; ++i) {
      centers[i] = {x + offsets[i] * std::cos(theta),
                    y + offsets[i] * std::sin(theta)};
    }
    return centers;
  };

  auto calc_disc_num = [](double length, double width) {
    double aspect_ratio = length / width;
    int disc_num = std::max(2, static_cast<int>(aspect_ratio * 2));
    return std::min(disc_num, 6);
  };

  auto calc_dynamic_centers = [](double x, double y, double theta,
                                 double length, int disc_num,
                                 bool is_ego = false) {
    double segment_length = length / disc_num;
    double backcenter = is_ego ? segment_length / 2 : length / 2;

    std::vector<std::pair<double, double>> centers;
    centers.reserve(disc_num);

    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);

    for (int i = 0; i < disc_num; ++i) {
      double offset = (i + 0.5) * segment_length - backcenter;
      double center_x = x + offset * cos_theta;
      double center_y = y + offset * sin_theta;
      centers.push_back({center_x, center_y});
    }
    return centers;
  };

  auto calc_radius = [](double length, double width) {
    return sqrt(pow(length / 6, 2) + pow(width / 2, 2));
  };

  auto calc_dynamic_radius = [](double length, double width, int disc_num) {
    return sqrt(pow(length / (2 * disc_num), 2) + pow(width / 2, 2));
  };
  auto calc_distance = [](double x1, double y1, double x2, double y2, double r1,
                          double r2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)) - r1 - r2;
  };
  auto calc_distance_squared = [](double x1, double y1, double x2, double y2,
                                  double r1, double r2) {
    double dx = x1 - x2;
    double dy = y1 - y2;
    double center_dist_sq = dx * dx + dy * dy;
    return center_dist_sq;
  };
  const int obs_num = cost_config_ptr_->at(OBS_NUM);
  double min_dist_squared = std::numeric_limits<double>::max();
  double min_dist = std::numeric_limits<double>::max();
  double min_obs_x = 0.0, min_obs_y = 0.0, min_obs_radius = 0.0;
  int closest_obs_index = -1;
  double closest_ego_disc_x = 0.0, closest_ego_disc_y = 0.0;
  double closest_obs_disc_x = 0.0, closest_obs_disc_y = 0.0;
  int closest_ego_disc_index = -1;
  if (obs_num == 0) {
    min_obs_x = x[EGO_X];
    min_obs_y = x[EGO_Y];
    min_obs_radius = 0.0;
    min_dist = 0.0;
    return {min_dist,           min_obs_x,
            min_obs_y,          min_obs_radius,
            closest_obs_index,  closest_ego_disc_x,
            closest_ego_disc_y, closest_obs_disc_x,
            closest_obs_disc_y, closest_ego_disc_index};
  }
  double ego_length = cost_config_ptr_->at(EGO_LENGTH);
  double ego_width = cost_config_ptr_->at(EGO_WIDTH);
  double ego_x = x[EGO_X];
  double ego_y = x[EGO_Y];
  double ego_theta = x[EGO_THETA];

  auto ego_centers =
      calc_three_centers(ego_x, ego_y, ego_theta, ego_length, true);
  double ego_radius = calc_radius(ego_length, ego_width);

  for (int i = 0; i < obs_num; ++i) {
    // Get obstacle position from reference trajectory.
    const int ref_x_idx = GetObsRefStateIdx(i, obs_num, OBS_X);
    const int ref_y_idx = GetObsRefStateIdx(i, obs_num, OBS_Y);
    const int ref_theta_idx = GetObsRefStateIdx(i, obs_num, OBS_THETA);
    double obs_x = cost_config_ptr_->at(ref_x_idx);
    double obs_y = cost_config_ptr_->at(ref_y_idx);
    double obs_theta = cost_config_ptr_->at(ref_theta_idx);
    double obs_length = cost_config_ptr_->at(GetObsLengthIdx(i, obs_num));
    double obs_width = cost_config_ptr_->at(GetObsWidthIdx(i, obs_num));

    int obs_disc_num = calc_disc_num(obs_length, obs_width);
    auto obs_centers = calc_dynamic_centers(obs_x, obs_y, obs_theta, obs_length,
                                            obs_disc_num, false);
    double obs_radius =
        calc_dynamic_radius(obs_length, obs_width, obs_disc_num);

    for (int ego_disc_idx = 0; ego_disc_idx < 3; ++ego_disc_idx) {
      const auto &ego_c = ego_centers[ego_disc_idx];
      for (int obs_disc_idx = 0; obs_disc_idx < obs_disc_num; ++obs_disc_idx) {
        const auto &obs_c = obs_centers[obs_disc_idx];
        double dist_squared =
            calc_distance_squared(ego_c.first, ego_c.second, obs_c.first,
                                  obs_c.second, ego_radius, obs_radius);
        if (dist_squared < min_dist_squared) {
          min_dist_squared = dist_squared;
          min_dist = calc_distance(ego_c.first, ego_c.second, obs_c.first,
                                   obs_c.second, ego_radius, obs_radius);
          min_obs_x = obs_x;
          min_obs_y = obs_y;
          min_obs_radius = obs_radius;
          closest_obs_index = i;
          closest_ego_disc_x = ego_c.first;
          closest_ego_disc_y = ego_c.second;
          closest_obs_disc_x = obs_c.first;
          closest_obs_disc_y = obs_c.second;
          closest_ego_disc_index = ego_disc_idx;
        }
      }
    }
  }
  return {min_dist,           min_obs_x,
          min_obs_y,          min_obs_radius,
          closest_obs_index,  closest_ego_disc_x,
          closest_ego_disc_y, closest_obs_disc_x,
          closest_obs_disc_y, closest_ego_disc_index};
}

double EgoThreeDiscSafeCostTerm::GetCost(const ilqr_solver::State &x,
                                         const ilqr_solver::Control &u) {
  auto result = CalculateThreeDiscDistances(x);
  if (cost_config_ptr_->at(OBS_NUM) == 0) {
    return 0.0;
  }
  double safe_distance = cost_config_ptr_->at(THREE_DISC_SAFE_DIST);
  double weight = cost_config_ptr_->at(W_THREE_DISC_SAFE_DIST_WEIGHT);

  double cost = 0.0;
  if (result.min_dist < safe_distance) {
    double violation = safe_distance - result.min_dist;
    cost = weight * (std::exp(violation) - 1.0);
  }

  return cost;
}

void EgoThreeDiscSafeCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control &u,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT &lu, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT &, ilqr_solver::LuuMT &luu) {
  auto result = CalculateThreeDiscDistances(x);
  if (cost_config_ptr_->at(OBS_NUM) == 0) {
    return;
  }
  double safe_distance = cost_config_ptr_->at(THREE_DISC_SAFE_DIST);
  double weight = cost_config_ptr_->at(W_THREE_DISC_SAFE_DIST_WEIGHT);

  if (result.min_dist >= safe_distance) {
    return;
  }

  double violation = safe_distance - result.min_dist;
  double gradient_coeff = -weight * std::exp(violation);
  double ego_length = cost_config_ptr_->at(EGO_LENGTH);
  double ego_theta = x[EGO_THETA];
  double rel_x = result.closest_ego_disc_x - result.closest_obs_disc_x;
  double rel_y = result.closest_ego_disc_y - result.closest_obs_disc_y;
  double rel_distance = std::sqrt(rel_x * rel_x + rel_y * rel_y);
  if (rel_distance < kEps) {
    return;
  }
  double unit_x = rel_x / rel_distance;
  double unit_y = rel_y / rel_distance;
  double L = ego_length / 6;
  double backcenter = L;
  std::array<double, 3> offsets = {L - backcenter, 3 * L - backcenter,
                                   5 * L - backcenter};
  double offset = offsets[result.closest_ego_disc_index];
  double dx_dego_x = 1.0;
  double dx_dego_y = 0.0;
  double dx_dego_theta = -offset * std::sin(ego_theta);
  double dy_dego_x = 0.0;
  double dy_dego_y = 1.0;
  double dy_dego_theta = offset * std::cos(ego_theta);
  double ddist_dego_x = unit_x * dx_dego_x + unit_y * dy_dego_x;
  double ddist_dego_y = unit_x * dx_dego_y + unit_y * dy_dego_y;
  double ddist_dego_theta = unit_x * dx_dego_theta + unit_y * dy_dego_theta;
  lx(EGO_X) += gradient_coeff * ddist_dego_x;
  lx(EGO_Y) += gradient_coeff * ddist_dego_y;
  lx(EGO_THETA) += gradient_coeff * ddist_dego_theta;
  double dunitx_dx =
      (rel_y * rel_y) / (rel_distance * rel_distance * rel_distance);
  double dunitx_dy =
      -(rel_x * rel_y) / (rel_distance * rel_distance * rel_distance);
  double dunitx_dtheta = (-offset * std::sin(ego_theta) * rel_y * rel_y +
                          offset * std::cos(ego_theta) * rel_x * rel_y) /
                         (rel_distance * rel_distance * rel_distance);
  double dunity_dx =
      -(rel_x * rel_y) / (rel_distance * rel_distance * rel_distance);
  double dunity_dy =
      (rel_x * rel_x) / (rel_distance * rel_distance * rel_distance);
  double dunity_dtheta = (-offset * std::sin(ego_theta) * rel_x * rel_y +
                          offset * std::cos(ego_theta) * rel_x * rel_x) /
                         (rel_distance * rel_distance * rel_distance);
  double d2dist_dx2 = dunitx_dx * dx_dego_x + unit_x * 0.0 +
                      dunity_dx * dy_dego_x + unit_y * 0.0;
  double d2dist_dy2 = dunitx_dy * dx_dego_y + unit_x * 0.0 +
                      dunity_dy * dy_dego_y + unit_y * 0.0;
  double d2dist_dtheta2 =
      dunitx_dtheta * dx_dego_theta + unit_x * (-offset * std::cos(ego_theta)) +
      dunity_dtheta * dy_dego_theta + unit_y * (-offset * std::sin(ego_theta));
  double hess_coeff = weight * std::exp(violation);
  lxx(EGO_X, EGO_X) +=
      hess_coeff * (ddist_dego_x * ddist_dego_x + violation * d2dist_dx2);
  lxx(EGO_Y, EGO_Y) +=
      hess_coeff * (ddist_dego_y * ddist_dego_y + violation * d2dist_dy2);
  lxx(EGO_THETA, EGO_THETA) +=
      hess_coeff *
      (ddist_dego_theta * ddist_dego_theta + violation * d2dist_dtheta2);
  lxx(EGO_X, EGO_Y) += hess_coeff * ddist_dego_x * ddist_dego_y;
  lxx(EGO_Y, EGO_X) += hess_coeff * ddist_dego_y * ddist_dego_x;
  lxx(EGO_X, EGO_THETA) += hess_coeff * ddist_dego_x * ddist_dego_theta;
  lxx(EGO_THETA, EGO_X) += hess_coeff * ddist_dego_theta * ddist_dego_x;
  lxx(EGO_Y, EGO_THETA) += hess_coeff * ddist_dego_y * ddist_dego_theta;
  lxx(EGO_THETA, EGO_Y) += hess_coeff * ddist_dego_theta * ddist_dego_y;
}

void EgoRoadBoundaryCostTerm::CalculateBoundaryDistancesInfo(
    const ilqr_solver::State &x) {
  double ego_x = x[EGO_X];
  double ego_y = x[EGO_Y];
  double ego_theta = x[EGO_THETA];
  double ego_wheel_base = cost_config_ptr_->at(EGO_WHEEL_BASE);
  double safe_distance = cost_config_ptr_->at(ROAD_BOUNDARY_SAFE_DIST);
  double cos_theta = std::cos(ego_theta);
  double sin_theta = std::sin(ego_theta);
  double front_center_x = ego_x + ego_wheel_base * cos_theta;
  double front_center_y = ego_y + ego_wheel_base * sin_theta;
  planning::planning_math::Vec2d front_center(front_center_x, front_center_y);
  planning::planning_math::Vec2d rear_center(ego_x, ego_y);
  planning::planning_math::Vec2d front_nearest_left, front_nearest_right;
  planning::planning_math::Vec2d rear_nearest_left, rear_nearest_right;
  double front_dist_to_left =
      road_left_boundary_path_->DistanceTo(front_center, &front_nearest_left);
  double rear_dist_to_left =
      road_left_boundary_path_->DistanceTo(rear_center, &rear_nearest_left);
  double front_dist_to_right =
      road_right_boundary_path_->DistanceTo(front_center, &front_nearest_right);
  double rear_dist_to_right =
      road_right_boundary_path_->DistanceTo(rear_center, &rear_nearest_right);
  double front_dist_to_left_sq = front_dist_to_left * front_dist_to_left;
  double rear_dist_to_left_sq = rear_dist_to_left * rear_dist_to_left;
  double front_dist_to_right_sq = front_dist_to_right * front_dist_to_right;
  double rear_dist_to_right_sq = rear_dist_to_right * rear_dist_to_right;
  double dist_to_left = front_dist_to_left_sq < rear_dist_to_left_sq
                            ? front_dist_to_left
                            : rear_dist_to_left;
  double dist_to_right = front_dist_to_right_sq < rear_dist_to_right_sq
                             ? front_dist_to_right
                             : rear_dist_to_right;
  bool left_front_closer = front_dist_to_left_sq < rear_dist_to_left_sq;
  bool right_front_closer = front_dist_to_right_sq < rear_dist_to_right_sq;
  bool ego_left_valid = false;
  planning::planning_math::Vec2d ego_left_unit_vector(0, 0);
  if (dist_to_left < safe_distance + cost_config_ptr_->at(EGO_WIDTH) / 2.0) {
    const auto &query_point = left_front_closer ? front_center : rear_center;
    const auto &nearest_point =
        left_front_closer ? front_nearest_left : rear_nearest_left;
    double dx = query_point.x() - nearest_point.x();
    double dy = query_point.y() - nearest_point.y();
    double dist_to_nearest_point = std::sqrt(dx * dx + dy * dy);
    if (dist_to_nearest_point > kEps) {
      ego_left_unit_vector.set_x(dx / dist_to_nearest_point);
      ego_left_unit_vector.set_y(dy / dist_to_nearest_point);
    }
    ego_left_valid = dist_to_nearest_point > kEps;
  }
  bool ego_right_valid = false;
  planning::planning_math::Vec2d ego_right_unit_vector(0, 0);
  if (dist_to_right < safe_distance + cost_config_ptr_->at(EGO_WIDTH) / 2.0) {
    const auto &query_point = right_front_closer ? front_center : rear_center;
    const auto &nearest_point =
        right_front_closer ? front_nearest_right : rear_nearest_right;
    double dx = query_point.x() - nearest_point.x();
    double dy = query_point.y() - nearest_point.y();
    double dist_to_nearest_point = std::sqrt(dx * dx + dy * dy);
    if (dist_to_nearest_point > kEps) {
      ego_right_unit_vector.set_x(dx / dist_to_nearest_point);
      ego_right_unit_vector.set_y(dy / dist_to_nearest_point);
    }
    ego_right_valid = dist_to_nearest_point > kEps;
  }
  dist_result_.min_dist_to_left = dist_to_left;
  dist_result_.min_dist_to_right = dist_to_right;
  dist_result_.left_front_closer = left_front_closer;
  dist_result_.right_front_closer = right_front_closer;
  dist_result_.front_center = front_center;
  dist_result_.rear_center = rear_center;
  dist_result_.ego_left_valid = ego_left_valid;
  dist_result_.ego_left_unit_vector = ego_left_unit_vector;
  dist_result_.ego_right_valid = ego_right_valid;
  dist_result_.ego_right_unit_vector = ego_right_unit_vector;
}

double EgoRoadBoundaryCostTerm::GetCost(const ilqr_solver::State &x,
                                        const ilqr_solver::Control &u) {
  if (!road_left_boundary_path_ || !road_right_boundary_path_ ||
      !road_left_boundary_path_->KdtreeValid() ||
      !road_right_boundary_path_->KdtreeValid()) {
    return 0.0;
  }
  CalculateBoundaryDistancesInfo(x);
  double safe_distance = cost_config_ptr_->at(ROAD_BOUNDARY_SAFE_DIST);
  double weight = cost_config_ptr_->at(W_ROAD_BOUNDARY);
  double ego_half_width = cost_config_ptr_->at(EGO_WIDTH) / 2.0;
  double ego_third_wheel_base = cost_config_ptr_->at(EGO_WHEEL_BASE) / 3.0;
  double ego_radius = std::sqrt(ego_half_width * ego_half_width +
                                ego_third_wheel_base * ego_third_wheel_base);
  double cost = 0.0;
  if (dist_result_.min_dist_to_left < safe_distance + ego_radius) {
    double violation =
        safe_distance - dist_result_.min_dist_to_left + ego_radius;
    cost += weight * (std::exp(violation) - 1.0);
  }
  if (dist_result_.min_dist_to_right < safe_distance + ego_radius) {
    double violation =
        safe_distance - dist_result_.min_dist_to_right + ego_radius;
    cost += weight * (std::exp(violation) - 1.0);
  }
  return cost;
}

void EgoRoadBoundaryCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control &u,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT &lu, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT &lxu, ilqr_solver::LuuMT &luu) {
  if (!road_left_boundary_path_ || !road_right_boundary_path_ ||
      !road_left_boundary_path_->KdtreeValid() ||
      !road_right_boundary_path_->KdtreeValid()) {
    return;
  }
  CalculateBoundaryDistancesInfo(x);
  double safe_distance = cost_config_ptr_->at(ROAD_BOUNDARY_SAFE_DIST);
  double weight = cost_config_ptr_->at(W_ROAD_BOUNDARY);
  double ego_theta = x[EGO_THETA];
  double ego_length = cost_config_ptr_->at(EGO_LENGTH);
  double ego_half_width = cost_config_ptr_->at(EGO_WIDTH) / 2.0;
  double ego_one_third_wheel_base = cost_config_ptr_->at(EGO_WHEEL_BASE) / 3.0;
  double ego_radius =
      std::sqrt(ego_half_width * ego_half_width +
                ego_one_third_wheel_base * ego_one_third_wheel_base);
  if (dist_result_.ego_left_valid &&
      dist_result_.min_dist_to_left < safe_distance + ego_radius) {
    double unit_x = dist_result_.ego_left_unit_vector.x();
    double unit_y = dist_result_.ego_left_unit_vector.y();
    double violation =
        safe_distance - dist_result_.min_dist_to_left + ego_radius;
    double dx_dego_x, dx_dego_y, dx_dego_theta;
    double dy_dego_x, dy_dego_y, dy_dego_theta;
    if (dist_result_.left_front_closer) {
      dx_dego_x = 1.0;
      dx_dego_y = 0.0;
      dx_dego_theta = -ego_length / 2 * std::sin(ego_theta);
      dy_dego_x = 0.0;
      dy_dego_y = 1.0;
      dy_dego_theta = ego_length / 2 * std::cos(ego_theta);
    } else {
      dx_dego_x = 1.0;
      dx_dego_y = 0.0;
      dx_dego_theta = 0.0;
      dy_dego_x = 0.0;
      dy_dego_y = 1.0;
      dy_dego_theta = 0.0;
    }
    double ddist_dego_x = unit_x * dx_dego_x + unit_y * dy_dego_x;
    double ddist_dego_y = unit_x * dx_dego_y + unit_y * dy_dego_y;
    double ddist_dego_theta = unit_x * dx_dego_theta + unit_y * dy_dego_theta;
    double gradient_coeff = -weight * std::exp(violation);
    lx(EGO_X) += gradient_coeff * ddist_dego_x;
    lx(EGO_Y) += gradient_coeff * ddist_dego_y;
    lx(EGO_THETA) += gradient_coeff * ddist_dego_theta;
    double rel_x = dist_result_.front_center.x() - dist_result_.rear_center.x();
    double rel_y = dist_result_.front_center.y() - dist_result_.rear_center.y();
    double rel_distance = std::sqrt(rel_x * rel_x + rel_y * rel_y);
    if (rel_distance > kEps) {
      double dunitx_dx =
          (rel_y * rel_y) / (rel_distance * rel_distance * rel_distance);
      double dunitx_dy =
          -(rel_x * rel_y) / (rel_distance * rel_distance * rel_distance);
      double dunitx_dtheta = 0.0;
      double dunity_dx =
          -(rel_x * rel_y) / (rel_distance * rel_distance * rel_distance);
      double dunity_dy =
          (rel_x * rel_x) / (rel_distance * rel_distance * rel_distance);
      double dunity_dtheta = 0.0;
      if (dist_result_.left_front_closer) {
        dunitx_dtheta = (-ego_length / 2 * std::sin(ego_theta) * rel_y * rel_y +
                         ego_length / 2 * std::cos(ego_theta) * rel_x * rel_y) /
                        (rel_distance * rel_distance * rel_distance);
        dunity_dtheta = (-ego_length / 2 * std::sin(ego_theta) * rel_x * rel_y +
                         ego_length / 2 * std::cos(ego_theta) * rel_x * rel_x) /
                        (rel_distance * rel_distance * rel_distance);
      }
      double d2dist_dx2 = dunitx_dx * dx_dego_x + unit_x * 0.0 +
                          dunity_dx * dy_dego_x + unit_y * 0.0;
      double d2dist_dy2 = dunitx_dy * dx_dego_y + unit_x * 0.0 +
                          dunity_dy * dy_dego_y + unit_y * 0.0;
      double d2dist_dtheta2 = dunitx_dtheta * dx_dego_theta +
                              unit_x * (-ego_length / 2 * std::cos(ego_theta)) +
                              dunity_dtheta * dy_dego_theta +
                              unit_y * (-ego_length / 2 * std::sin(ego_theta));
      double hess_coeff = weight * std::exp(violation);
      lxx(EGO_X, EGO_X) +=
          hess_coeff * (ddist_dego_x * ddist_dego_x + violation * d2dist_dx2);
      lxx(EGO_Y, EGO_Y) +=
          hess_coeff * (ddist_dego_y * ddist_dego_y + violation * d2dist_dy2);
      lxx(EGO_THETA, EGO_THETA) +=
          hess_coeff *
          (ddist_dego_theta * ddist_dego_theta + violation * d2dist_dtheta2);
      lxx(EGO_X, EGO_Y) += hess_coeff * ddist_dego_x * ddist_dego_y;
      lxx(EGO_Y, EGO_X) += hess_coeff * ddist_dego_y * ddist_dego_x;
      lxx(EGO_X, EGO_THETA) += hess_coeff * ddist_dego_x * ddist_dego_theta;
      lxx(EGO_THETA, EGO_X) += hess_coeff * ddist_dego_theta * ddist_dego_x;
      lxx(EGO_Y, EGO_THETA) += hess_coeff * ddist_dego_y * ddist_dego_theta;
      lxx(EGO_THETA, EGO_Y) += hess_coeff * ddist_dego_theta * ddist_dego_y;
    }
  }
  if (dist_result_.ego_right_valid &&
      dist_result_.min_dist_to_right < safe_distance + ego_radius) {
    double unit_x = dist_result_.ego_right_unit_vector.x();
    double unit_y = dist_result_.ego_right_unit_vector.y();
    double violation =
        safe_distance - dist_result_.min_dist_to_right + ego_radius;
    double dx_dego_x, dx_dego_y, dx_dego_theta;
    double dy_dego_x, dy_dego_y, dy_dego_theta;
    if (dist_result_.right_front_closer) {
      dx_dego_x = 1.0;
      dx_dego_y = 0.0;
      dx_dego_theta = -ego_length / 2 * std::sin(ego_theta);
      dy_dego_x = 0.0;
      dy_dego_y = 1.0;
      dy_dego_theta = ego_length / 2 * std::cos(ego_theta);
    } else {
      dx_dego_x = 1.0;
      dx_dego_y = 0.0;
      dx_dego_theta = 0.0;
      dy_dego_x = 0.0;
      dy_dego_y = 1.0;
      dy_dego_theta = 0.0;
    }
    double ddist_dego_x = unit_x * dx_dego_x + unit_y * dy_dego_x;
    double ddist_dego_y = unit_x * dx_dego_y + unit_y * dy_dego_y;
    double ddist_dego_theta = unit_x * dx_dego_theta + unit_y * dy_dego_theta;
    double gradient_coeff = -weight * std::exp(violation);
    lx(EGO_X) += gradient_coeff * ddist_dego_x;
    lx(EGO_Y) += gradient_coeff * ddist_dego_y;
    lx(EGO_THETA) += gradient_coeff * ddist_dego_theta;
    double rel_x = dist_result_.front_center.x() - dist_result_.rear_center.x();
    double rel_y = dist_result_.front_center.y() - dist_result_.rear_center.y();
    double rel_distance = std::sqrt(rel_x * rel_x + rel_y * rel_y);
    if (rel_distance > kEps) {
      double dunitx_dx =
          (rel_y * rel_y) / (rel_distance * rel_distance * rel_distance);
      double dunitx_dy =
          -(rel_x * rel_y) / (rel_distance * rel_distance * rel_distance);
      double dunitx_dtheta = 0.0;
      double dunity_dx =
          -(rel_x * rel_y) / (rel_distance * rel_distance * rel_distance);
      double dunity_dy =
          (rel_x * rel_x) / (rel_distance * rel_distance * rel_distance);
      double dunity_dtheta = 0.0;
      if (dist_result_.right_front_closer) {
        dunitx_dtheta = (-ego_length / 2 * std::sin(ego_theta) * rel_y * rel_y +
                         ego_length / 2 * std::cos(ego_theta) * rel_x * rel_y) /
                        (rel_distance * rel_distance * rel_distance);
        dunity_dtheta = (-ego_length / 2 * std::sin(ego_theta) * rel_x * rel_y +
                         ego_length / 2 * std::cos(ego_theta) * rel_x * rel_x) /
                        (rel_distance * rel_distance * rel_distance);
      }
      double d2dist_dx2 = dunitx_dx * dx_dego_x + unit_x * 0.0 +
                          dunity_dx * dy_dego_x + unit_y * 0.0;
      double d2dist_dy2 = dunitx_dy * dx_dego_y + unit_x * 0.0 +
                          dunity_dy * dy_dego_y + unit_y * 0.0;
      double d2dist_dtheta2 = dunitx_dtheta * dx_dego_theta +
                              unit_x * (-ego_length / 2 * std::cos(ego_theta)) +
                              dunity_dtheta * dy_dego_theta +
                              unit_y * (-ego_length / 2 * std::sin(ego_theta));
      double hess_coeff = weight * std::exp(violation);
      lxx(EGO_X, EGO_X) +=
          hess_coeff * (ddist_dego_x * ddist_dego_x + violation * d2dist_dx2);
      lxx(EGO_Y, EGO_Y) +=
          hess_coeff * (ddist_dego_y * ddist_dego_y + violation * d2dist_dy2);
      lxx(EGO_THETA, EGO_THETA) +=
          hess_coeff *
          (ddist_dego_theta * ddist_dego_theta + violation * d2dist_dtheta2);
      lxx(EGO_X, EGO_Y) += hess_coeff * ddist_dego_x * ddist_dego_y;
      lxx(EGO_Y, EGO_X) += hess_coeff * ddist_dego_y * ddist_dego_x;
      lxx(EGO_X, EGO_THETA) += hess_coeff * ddist_dego_x * ddist_dego_theta;
      lxx(EGO_THETA, EGO_X) += hess_coeff * ddist_dego_theta * ddist_dego_x;
      lxx(EGO_Y, EGO_THETA) += hess_coeff * ddist_dego_y * ddist_dego_theta;
      lxx(EGO_THETA, EGO_Y) += hess_coeff * ddist_dego_theta * ddist_dego_y;
    }
  }
}

double EgoAccCostTerm::GetCost(const ilqr_solver::State &x,
                               const ilqr_solver::Control &) {
  double ego_acc = x[EGO_ACC];
  double weight = cost_config_ptr_->at(W_EGO_ACC);
  return 0.5 * weight * ego_acc * ego_acc;
}

void EgoAccCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control &,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT &, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) {
  double ego_acc = x[EGO_ACC];
  double weight = cost_config_ptr_->at(W_EGO_ACC);

  lx(EGO_ACC) += weight * ego_acc;

  lxx(EGO_ACC, EGO_ACC) += weight;
}

double EgoJerkCostTerm::GetCost(const ilqr_solver::State &x,
                                const ilqr_solver::Control &u) {
  double ego_jerk = u[EGO_JERK];
  double weight = cost_config_ptr_->at(W_EGO_JERK);
  return 0.5 * weight * ego_jerk * ego_jerk;
}

void EgoJerkCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control &u,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT &lu, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT &, ilqr_solver::LuuMT &luu) {
  double ego_jerk = u[EGO_JERK];
  double weight = cost_config_ptr_->at(W_EGO_JERK);

  lu(EGO_JERK) += weight * ego_jerk;

  luu(EGO_JERK, EGO_JERK) += weight;
}

double EgoOmegaCostTerm::GetCost(const ilqr_solver::State &x,
                                 const ilqr_solver::Control &u) {
  double ego_omega = u[EGO_OMEGA];
  double weight = cost_config_ptr_->at(W_EGO_OMEGA);
  return 0.5 * weight * ego_omega * ego_omega;
}

void EgoOmegaCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control &u,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT &lu, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT &, ilqr_solver::LuuMT &luu) {
  double ego_omega = u[EGO_OMEGA];
  double weight = cost_config_ptr_->at(W_EGO_OMEGA);

  lu(EGO_OMEGA) += weight * ego_omega;

  luu(EGO_OMEGA, EGO_OMEGA) += weight;
}

double EgoDeltaCostTerm::GetCost(const ilqr_solver::State &x,
                                 const ilqr_solver::Control &) {
  double ego_delta = x[EGO_DELTA];
  double weight = cost_config_ptr_->at(W_EGO_DELTA);
  return 0.5 * weight * ego_delta * ego_delta;
}

void EgoDeltaCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control &,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT &, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) {
  double ego_delta = x[EGO_DELTA];
  double weight = cost_config_ptr_->at(W_EGO_DELTA);

  lx(EGO_DELTA) += weight * ego_delta;
  lxx(EGO_DELTA, EGO_DELTA) += weight;
}

double EgoAccBoundCostTerm::GetCost(const ilqr_solver::State &x,
                                    const ilqr_solver::Control &) {
  double ego_acc = x[EGO_ACC];
  double weight = cost_config_ptr_->at(W_EGO_ACC_BOUND);
  double acc_max = cost_config_ptr_->at(EGO_ACC_MAX);
  double acc_min = cost_config_ptr_->at(EGO_ACC_MIN);

  double cost = 0.0;

  if (ego_acc > acc_max) {
    double violation = ego_acc - acc_max;
    cost += weight * violation * violation;
  }

  if (ego_acc < acc_min) {
    double violation = acc_min - ego_acc;
    cost += weight * violation * violation;
  }

  return cost;
}

void EgoAccBoundCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control &,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT &, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT &, ilqr_solver::LuuMT &) {
  double ego_acc = x[EGO_ACC];
  double weight = cost_config_ptr_->at(W_EGO_ACC_BOUND);
  double acc_max = cost_config_ptr_->at(EGO_ACC_MAX);
  double acc_min = cost_config_ptr_->at(EGO_ACC_MIN);

  if (ego_acc > acc_max) {
    double violation = ego_acc - acc_max;
    lx(EGO_ACC) += 2.0 * weight * violation;
    lxx(EGO_ACC, EGO_ACC) += 2.0 * weight;
  }

  if (ego_acc < acc_min) {
    double violation = acc_min - ego_acc;
    lx(EGO_ACC) += 2.0 * weight * violation;
    lxx(EGO_ACC, EGO_ACC) += 2.0 * weight;
  }
}

double EgoJerkBoundCostTerm::GetCost(const ilqr_solver::State &,
                                     const ilqr_solver::Control &u) {
  double ego_jerk = u[EGO_JERK];
  double weight = cost_config_ptr_->at(W_EGO_JERK_BOUND);
  double jerk_max = cost_config_ptr_->at(EGO_JERK_MAX);
  double jerk_min = cost_config_ptr_->at(EGO_JERK_MIN);

  double cost = 0.0;

  if (ego_jerk > jerk_max) {
    double violation = ego_jerk - jerk_max;
    cost += weight * violation * violation;
  }

  if (ego_jerk < jerk_min) {
    double violation = jerk_min - ego_jerk;
    cost += weight * violation * violation;
  }

  return cost;
}

void EgoJerkBoundCostTerm::GetGradientHessian(
    const ilqr_solver::State &, const ilqr_solver::Control &u,
    ilqr_solver::LxMT &, ilqr_solver::LuMT &lu, ilqr_solver::LxxMT &,
    ilqr_solver::LxuMT &, ilqr_solver::LuuMT &luu) {
  double ego_jerk = u[EGO_JERK];
  double weight = cost_config_ptr_->at(W_EGO_JERK_BOUND);
  double jerk_max = cost_config_ptr_->at(EGO_JERK_MAX);
  double jerk_min = cost_config_ptr_->at(EGO_JERK_MIN);

  if (ego_jerk > jerk_max) {
    double violation = ego_jerk - jerk_max;
    lu(EGO_JERK) += 2.0 * weight * violation;
    luu(EGO_JERK, EGO_JERK) += 2.0 * weight;
  }

  if (ego_jerk < jerk_min) {
    double violation = jerk_min - ego_jerk;
    lu(EGO_JERK) += 2.0 * weight * violation;
    luu(EGO_JERK, EGO_JERK) += 2.0 * weight;
  }
}

std::vector<SoftHalfplaneCostTerm::SoftHalfplaneResult>
SoftHalfplaneCostTerm::CalculateSoftHalfplane(const ilqr_solver::State &x) {
  std::vector<SoftHalfplaneResult> results;

  const int obs_num = cost_config_ptr_->at(OBS_NUM);
  if (obs_num == 0) {
    return results;
  }

  const double ego_x = x[EGO_X];
  const double ego_y = x[EGO_Y];
  const double ego_theta = x[EGO_THETA];
  const double ego_vel = x[EGO_VEL];
  const double ego_length = cost_config_ptr_->at(EGO_LENGTH);
  const double ego_front_edge_to_rear_axle =
      cost_config_ptr_->at(EGO_FRONT_EDGE_TO_REAR_AXLE);
  const double ego_rear_edge_to_rear_axle =
      cost_config_ptr_->at(EGO_LENGTH) - ego_front_edge_to_rear_axle;

  const double s0 = cost_config_ptr_->at(SOFT_HALFPLANE_S0);
  const double tau = cost_config_ptr_->at(SOFT_HALFPLANE_TAU);

  for (int i = 0; i < obs_num; ++i) {
    const int label_idx = GetObsLongitudinalLabelIdx(i, obs_num);
    const int label_value = static_cast<int>(cost_config_ptr_->at(label_idx));

    if (label_value != 1 && label_value != 2) {
      continue;
    }

    if (label_value == 2) {
      continue;
    }

    // Get obstacle state from reference trajectory.
    const int ref_x_idx = GetObsRefStateIdx(i, obs_num, OBS_X);
    const int ref_y_idx = GetObsRefStateIdx(i, obs_num, OBS_Y);
    const int ref_theta_idx = GetObsRefStateIdx(i, obs_num, OBS_THETA);
    const int ref_vel_idx = GetObsRefStateIdx(i, obs_num, OBS_VEL);
    const double obs_x = cost_config_ptr_->at(ref_x_idx);
    const double obs_y = cost_config_ptr_->at(ref_y_idx);
    const double obs_theta = cost_config_ptr_->at(ref_theta_idx);
    const double obs_vel = cost_config_ptr_->at(ref_vel_idx);
    const double obs_length = cost_config_ptr_->at(GetObsLengthIdx(i, obs_num));

    const double obs_normal_x = std::cos(obs_theta);
    const double obs_normal_y = std::sin(obs_theta);
    const double ego_normal_x = std::cos(ego_theta);
    const double ego_normal_y = std::sin(ego_theta);

    double s_current = 0.0;
    double s_target = 0.0;

    if (label_value == 1) {
      // OVERTAKE: 自车超越障碍物
      // s_current = 障碍物前端 → 自车后端 的距离
      const double ego_rear_x =
          ego_x - ego_rear_edge_to_rear_axle * std::cos(ego_theta);
      const double ego_rear_y =
          ego_y - ego_rear_edge_to_rear_axle * std::sin(ego_theta);

      const double obs_front_x = obs_x + (obs_length / 2.0) * obs_normal_x;
      const double obs_front_y = obs_y + (obs_length / 2.0) * obs_normal_y;

      const double dx = ego_rear_x - obs_front_x;
      const double dy = ego_rear_y - obs_front_y;

      s_current = dx * ego_normal_x + dy * ego_normal_y;

      if (s_current < 0) {
        continue;
      }

      // 目标距离基于自车速度
      s_target = s0 + tau * ego_vel;

    } else if (label_value == 2) {
      // YIELD: 自车让行障碍物
      // s_current = 障碍物后端 → 自车前端 的距离
      const double ego_front_x =
          ego_x + ego_front_edge_to_rear_axle * std::cos(ego_theta);
      const double ego_front_y =
          ego_y + ego_front_edge_to_rear_axle * std::sin(ego_theta);

      const double obs_rear_x = obs_x - (obs_length / 2.0) * obs_normal_x;
      const double obs_rear_y = obs_y - (obs_length / 2.0) * obs_normal_y;

      const double dx = obs_rear_x - ego_front_x;
      const double dy = obs_rear_y - ego_front_y;

      s_current = dx * ego_normal_x + dy * ego_normal_y;

      if (s_current <= 0) {
        continue;
      }

      s_target = s0 + tau * obs_vel;
    }

    SoftHalfplaneResult result;
    result.obs_index = i;
    result.label_type = label_value;
    result.s_current = s_current;
    result.s_target = s_target;
    result.normal_x = std::cos(ego_theta);
    result.normal_y = std::sin(ego_theta);

    results.push_back(result);
  }

  return results;
}

double SoftHalfplaneCostTerm::GetCost(const ilqr_solver::State &x,
                                      const ilqr_solver::Control &u) {
  auto results = CalculateSoftHalfplane(x);

  if (results.empty()) {
    return 0.0;
  }

  const double weight = cost_config_ptr_->at(W_SOFT_HALFPLANE);
  constexpr double epsilon = 1e-3;
  double total_cost = 0.0;

  for (const auto &result : results) {
    const double violation = result.s_current - result.s_target;
    if (violation < -epsilon) {
      total_cost += weight * violation * violation;
    }
  }

  return total_cost;
}

void SoftHalfplaneCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control &u,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT &lu, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT &lxu, ilqr_solver::LuuMT &luu) {
  auto results = CalculateSoftHalfplane(x);

  if (results.empty()) {
    return;
  }

  const double weight = cost_config_ptr_->at(W_SOFT_HALFPLANE);
  const double alpha =
      cost_config_ptr_->at(SOFT_HALFPLANE_COST_ALLOCATION_RATIO);
  const double tau = cost_config_ptr_->at(SOFT_HALFPLANE_TAU);
  constexpr double epsilon = 1e-3;

  for (const auto &result : results) {
    const double violation = result.s_current - result.s_target;
    if (violation >= -epsilon) {
      continue;
    }

    const int obs_idx = result.obs_index;
    const int label_type = result.label_type;
    const int state_base_idx = EGO_STATE_SIZE + obs_idx * OBS_STATE_SIZE;

    const double normal_x = result.normal_x;
    const double normal_y = result.normal_y;

    // 代价分配系数
    const double ego_weight = alpha;
    const double obs_weight = 1.0 - alpha;

    const double gradient_coeff = 2.0 * weight * violation;
    const double hess_coeff = 2.0 * weight;

    if (label_type == 1) {

      const double ddist_dego_x = normal_x;
      const double ddist_dego_y = normal_y;
      const double ddist_dego_vel = -tau;

      lx(EGO_X) += ego_weight * gradient_coeff * ddist_dego_x;
      lx(EGO_Y) += ego_weight * gradient_coeff * ddist_dego_y;
      lx(EGO_VEL) += ego_weight * gradient_coeff * ddist_dego_vel;

      lxx(EGO_X, EGO_X) +=
          ego_weight * hess_coeff * ddist_dego_x * ddist_dego_x;
      lxx(EGO_Y, EGO_Y) +=
          ego_weight * hess_coeff * ddist_dego_y * ddist_dego_y;
      lxx(EGO_VEL, EGO_VEL) +=
          ego_weight * hess_coeff * ddist_dego_vel * ddist_dego_vel;
      lxx(EGO_X, EGO_Y) +=
          ego_weight * hess_coeff * ddist_dego_x * ddist_dego_y;
      lxx(EGO_Y, EGO_X) +=
          ego_weight * hess_coeff * ddist_dego_y * ddist_dego_x;
      lxx(EGO_X, EGO_VEL) +=
          ego_weight * hess_coeff * ddist_dego_x * ddist_dego_vel;
      lxx(EGO_VEL, EGO_X) +=
          ego_weight * hess_coeff * ddist_dego_vel * ddist_dego_x;
      lxx(EGO_Y, EGO_VEL) +=
          ego_weight * hess_coeff * ddist_dego_y * ddist_dego_vel;
      lxx(EGO_VEL, EGO_Y) +=
          ego_weight * hess_coeff * ddist_dego_vel * ddist_dego_y;

    } else if (label_type == 2) {


      const double ddist_dego_x = -normal_x;
      const double ddist_dego_y = -normal_y;

      lx(EGO_X) += ego_weight * gradient_coeff * ddist_dego_x;
      lx(EGO_Y) += ego_weight * gradient_coeff * ddist_dego_y;

      lxx(EGO_X, EGO_X) +=
          ego_weight * hess_coeff * ddist_dego_x * ddist_dego_x;
      lxx(EGO_Y, EGO_Y) +=
          ego_weight * hess_coeff * ddist_dego_y * ddist_dego_y;
      lxx(EGO_X, EGO_Y) +=
          ego_weight * hess_coeff * ddist_dego_x * ddist_dego_y;
      lxx(EGO_Y, EGO_X) +=
          ego_weight * hess_coeff * ddist_dego_y * ddist_dego_x;

    }
  }
}

std::vector<HardHalfplaneCostTerm::HardHalfplaneResult>
HardHalfplaneCostTerm::CalculateObsHardHalfplane(const ilqr_solver::State &x) {
  std::vector<HardHalfplaneResult> results;

  const int obs_num = cost_config_ptr_->at(OBS_NUM);
  if (obs_num == 0) {
    return results;
  }

  const double ego_x = x[EGO_X];
  const double ego_y = x[EGO_Y];
  const double ego_theta = x[EGO_THETA];
  const double ego_length = cost_config_ptr_->at(EGO_LENGTH);
  const double ego_front_edge_to_rear_axle =
      cost_config_ptr_->at(EGO_FRONT_EDGE_TO_REAR_AXLE);
  const double ego_rear_edge_to_rear_axle =
      cost_config_ptr_->at(EGO_LENGTH) - ego_front_edge_to_rear_axle;

  for (int i = 0; i < obs_num; ++i) {
    const int label_idx = GetObsLongitudinalLabelIdx(i, obs_num);
    const int label_value = static_cast<int>(cost_config_ptr_->at(label_idx));

    if (label_value != 1 && label_value != 2) {
      continue;
    }

    // Get obstacle state from reference trajectory.
    const int ref_x_idx = GetObsRefStateIdx(i, obs_num, OBS_X);
    const int ref_y_idx = GetObsRefStateIdx(i, obs_num, OBS_Y);
    const int ref_theta_idx = GetObsRefStateIdx(i, obs_num, OBS_THETA);
    const double obs_x = cost_config_ptr_->at(ref_x_idx);
    const double obs_y = cost_config_ptr_->at(ref_y_idx);
    const double obs_theta = cost_config_ptr_->at(ref_theta_idx);
    const double obs_length = cost_config_ptr_->at(GetObsLengthIdx(i, obs_num));

    const double obs_normal_x = std::cos(obs_theta);
    const double obs_normal_y = std::sin(obs_theta);
    const double ego_normal_x = std::cos(ego_theta);
    const double ego_normal_y = std::sin(ego_theta);
    const double hard_dist = cost_config_ptr_->at(HARD_HALFPLANE_DIST);

    double plane_dist = 0.0;

    if (label_value == 1) {
      const double ego_rear_x =
          ego_x - ego_rear_edge_to_rear_axle * std::cos(ego_theta);
      const double ego_rear_y =
          ego_y - ego_rear_edge_to_rear_axle * std::sin(ego_theta);

      const double obs_front_x = obs_x + (obs_length / 2.0) * obs_normal_x;
      const double obs_front_y = obs_y + (obs_length / 2.0) * obs_normal_y;

      const double dx = ego_rear_x - obs_front_x;
      const double dy = ego_rear_y - obs_front_y;

      const double ego_normal_x = std::cos(ego_theta);
      const double ego_normal_y = std::sin(ego_theta);
      const double dist_along_ego_heading =
          dx * ego_normal_x + dy * ego_normal_y;

      if (dist_along_ego_heading < 0) {
        continue;
      }

      plane_dist = dist_along_ego_heading - hard_dist;

    } else if (label_value == 2) {
      const double ego_front_x =
          ego_x + ego_front_edge_to_rear_axle * std::cos(ego_theta);
      const double ego_front_y =
          ego_y + ego_front_edge_to_rear_axle * std::sin(ego_theta);

      const double obs_rear_x = obs_x - (obs_length / 2.0) * obs_normal_x;
      const double obs_rear_y = obs_y - (obs_length / 2.0) * obs_normal_y;

      const double dx = obs_rear_x - ego_front_x;
      const double dy = obs_rear_y - ego_front_y;

      const double dist_along_ego_heading =
          dx * ego_normal_x + dy * ego_normal_y;

      if (dist_along_ego_heading <= 0) {
        continue;
      }

      plane_dist = dist_along_ego_heading - hard_dist;
    }

    HardHalfplaneResult result;
    result.obs_index = i;
    result.label_type = label_value;
    result.plane_dist = plane_dist;
    result.normal_x = ego_normal_x;
    result.normal_y = ego_normal_y;

    results.push_back(result);
  }

  return results;
}

double HardHalfplaneCostTerm::GetCost(const ilqr_solver::State &x,
                                      const ilqr_solver::Control &u) {
  auto results = CalculateObsHardHalfplane(x);

  if (results.empty()) {
    return 0.0;
  }

  const double weight = cost_config_ptr_->at(W_HARD_HALFPLANE);
  constexpr double epsilon = 1e-3;
  double total_cost = 0.0;

  for (const auto &result : results) {
    if (result.plane_dist < -epsilon) {
      const double violation = result.plane_dist;
      total_cost += weight * violation * violation;
    }
  }

  return total_cost;
}

void HardHalfplaneCostTerm::GetGradientHessian(
    const ilqr_solver::State &x, const ilqr_solver::Control &u,
    ilqr_solver::LxMT &lx, ilqr_solver::LuMT &lu, ilqr_solver::LxxMT &lxx,
    ilqr_solver::LxuMT &lxu, ilqr_solver::LuuMT &luu) {
  auto results = CalculateObsHardHalfplane(x);

  if (results.empty()) {
    return;
  }

  const double weight = cost_config_ptr_->at(W_HARD_HALFPLANE);
  const double alpha = cost_config_ptr_->at(HALFPLANE_COST_ALLOCATION_RATIO);
  constexpr double epsilon = 1e-3;

  for (const auto &result : results) {
    if (result.plane_dist >= -epsilon) {
      continue;
    }

    const int obs_idx = result.obs_index;
    const int label_type = result.label_type;
    const int state_base_idx = EGO_STATE_SIZE + obs_idx * OBS_STATE_SIZE;

    const double violation = result.plane_dist;
    const double normal_x = result.normal_x;
    const double normal_y = result.normal_y;

    // 代价分配系数：alpha=0 全部施加到障碍物, alpha=1 全部施加到自车
    const double ego_weight = alpha;
    const double obs_weight = 1.0 - alpha;

    const double gradient_coeff = 2.0 * weight * violation;
    const double hess_coeff = 2.0 * weight;

    if (label_type == 1) {
      // OVERTAKE: 自车超越障碍物
      // plane_dist = (ego_rear - obs_front) · ego_normal - hard_dist

      // 对自车的梯度：∂dist/∂ego = ego_normal
      const double ddist_dego_x = normal_x;
      const double ddist_dego_y = normal_y;

      lx(EGO_X) += ego_weight * gradient_coeff * ddist_dego_x;
      lx(EGO_Y) += ego_weight * gradient_coeff * ddist_dego_y;

      lxx(EGO_X, EGO_X) +=
          ego_weight * hess_coeff * ddist_dego_x * ddist_dego_x;
      lxx(EGO_Y, EGO_Y) +=
          ego_weight * hess_coeff * ddist_dego_y * ddist_dego_y;
      lxx(EGO_X, EGO_Y) +=
          ego_weight * hess_coeff * ddist_dego_x * ddist_dego_y;
      lxx(EGO_Y, EGO_X) +=
          ego_weight * hess_coeff * ddist_dego_y * ddist_dego_x;

    } else if (label_type == 2) {
      // YIELD: 自车让行障碍物
      // plane_dist = (obs_rear - ego_front) · ego_normal - hard_dist

      // 对自车的梯度：∂dist/∂ego = -ego_normal
      const double ddist_dego_x = -normal_x;
      const double ddist_dego_y = -normal_y;

      lx(EGO_X) += ego_weight * gradient_coeff * ddist_dego_x;
      lx(EGO_Y) += ego_weight * gradient_coeff * ddist_dego_y;

      lxx(EGO_X, EGO_X) +=
          ego_weight * hess_coeff * ddist_dego_x * ddist_dego_x;
      lxx(EGO_Y, EGO_Y) +=
          ego_weight * hess_coeff * ddist_dego_y * ddist_dego_y;
      lxx(EGO_X, EGO_Y) +=
          ego_weight * hess_coeff * ddist_dego_x * ddist_dego_y;
      lxx(EGO_Y, EGO_X) +=
          ego_weight * hess_coeff * ddist_dego_y * ddist_dego_x;
    }
  }
}

}  // namespace joint_motion_planning
}  // namespace pnc
