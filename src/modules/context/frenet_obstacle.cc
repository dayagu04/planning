#include "frenet_obstacle.h"

#include <cassert>

#include "ego_state_manager.h"
#include "math/linear_interpolation.h"
#include "reference_path.h"
#include "session.h"
#include "utils/kd_path.h"
#include "utils/pose2d_utils.h"

namespace planning {

FrenetObstacle::FrenetObstacle(
    const Obstacle *obstacle_ptr, const ReferencePath &reference_path,
    const std::shared_ptr<EgoStateManager> ego_state_info,
    bool is_location_valid)
    : id_(obstacle_ptr->id()),
      obstacle_ptr_(obstacle_ptr),
      is_location_valid_(is_location_valid) {
  compute_frenet_obstacle(reference_path);
  if (is_location_valid_) {
    compute_frenet_obstacle_boundary(reference_path);
    compute_frenet_polygon_sequence(reference_path);
  }
}

void FrenetObstacle::compute_frenet_obstacle(
    const ReferencePath &reference_path) {
  FrenetEgoState frenet_ego_state = reference_path.get_frenet_ego_state();
  double ego_s = frenet_ego_state.s();
  double ego_l = frenet_ego_state.l();
  double ego_head_s = frenet_ego_state.head_s();
  double ego_head_l = frenet_ego_state.head_l();

  const auto &frenet_coord = reference_path.get_frenet_coord();
  // center frenet

  Point2D frenet_point, carte_point;
  if (is_location_valid_) {
    carte_point.x = obstacle_ptr_->x_center();
    carte_point.y = obstacle_ptr_->y_center();
  } else {
    carte_point.x = obstacle_ptr_->x_relative_center();
    carte_point.y = obstacle_ptr_->y_relative_center();
  }

  if (!frenet_coord->XYToSL(carte_point, frenet_point) ||
      std::isnan(frenet_point.x) || std::isnan(frenet_point.y)) {
    // frenet_s_ = obs_end_s;
    // frenet_l_ = obs_end_l;
    frenet_relative_velocity_angle_ = 0.0;
    b_frenet_valid_ = false;  // TODO: 注意这个标志位的使用
    return;
  } else {
    frenet_s_ = frenet_point.x;
    frenet_l_ = frenet_point.y;
  }

  // corner frenet
  const double obs_length = obstacle_ptr_->length();
  const double obs_width = obstacle_ptr_->width();
  double obs_relative_heading = obstacle_ptr_->heading_angle() -
                                frenet_coord->GetPathCurveHeading(frenet_s_);
  if (not is_location_valid_) {
    obs_relative_heading = obstacle_ptr_->relative_heading_angle() -
                           frenet_coord->GetPathCurveHeading(frenet_s_);
  }
  frenet_obstacle_corners_.s_front_left =
      frenet_s_ + obs_length / 2.0 * std::cos(obs_relative_heading) -
      obs_width / 2.0 * std::sin(obs_relative_heading);
  frenet_obstacle_corners_.l_front_left =
      frenet_l_ + obs_length / 2.0 * std::sin(obs_relative_heading) +
      obs_width / 2.0 * std::cos(obs_relative_heading);
  frenet_obstacle_corners_.s_front_right =
      frenet_s_ + obs_length / 2.0 * std::cos(obs_relative_heading) +
      obs_width / 2.0 * std::sin(obs_relative_heading);
  frenet_obstacle_corners_.l_front_right =
      frenet_l_ + obs_length / 2.0 * std::sin(obs_relative_heading) -
      obs_width / 2.0 * std::cos(obs_relative_heading);
  frenet_obstacle_corners_.s_rear_left =
      frenet_s_ - obs_length / 2.0 * std::cos(obs_relative_heading) -
      obs_width / 2.0 * std::sin(obs_relative_heading);
  frenet_obstacle_corners_.l_rear_left =
      frenet_l_ - obs_length / 2.0 * std::sin(obs_relative_heading) +
      obs_width / 2.0 * std::cos(obs_relative_heading);
  frenet_obstacle_corners_.s_rear_right =
      frenet_s_ - obs_length / 2.0 * std::cos(obs_relative_heading) +
      obs_width / 2.0 * std::sin(obs_relative_heading);
  frenet_obstacle_corners_.l_rear_right =
      frenet_l_ - obs_length / 2.0 * std::sin(obs_relative_heading) -
      obs_width / 2.0 * std::cos(obs_relative_heading);

  double curve_heading = frenet_coord->GetPathCurveHeading(frenet_s_);
  frenet_relative_velocity_angle_ = planning_math::NormalizeAngle(
      obstacle_ptr_->velocity_angle() - curve_heading);
  frenet_velocity_s_ =
      obstacle_ptr_->velocity() * std::cos(frenet_relative_velocity_angle_);
  frenet_velocity_l_ =
      obstacle_ptr_->velocity() * std::sin(frenet_relative_velocity_angle_);
  b_frenet_valid_ = true;

  std::vector<double> corners_l = {frenet_obstacle_corners_.l_front_left,
                                   frenet_obstacle_corners_.l_front_right,
                                   frenet_obstacle_corners_.l_rear_left,
                                   frenet_obstacle_corners_.l_rear_right};
  std::vector<double> corners_s = {frenet_obstacle_corners_.s_front_left,
                                   frenet_obstacle_corners_.s_front_right,
                                   frenet_obstacle_corners_.s_rear_left,
                                   frenet_obstacle_corners_.s_rear_right};

  s_with_max_l_.x = *std::max_element(corners_l.begin(), corners_l.end());
  auto it = std::find(corners_l.begin(), corners_l.end(), s_with_max_l_.x);
  size_t index = (size_t)(it - corners_l.begin());
  if (index < corners_l.size()) {
    s_with_max_l_.y = std::max(corners_s[index], 0.0);
  }

  s_with_min_l_.x = *std::min_element(corners_l.begin(), corners_l.end());
  it = std::find(corners_l.begin(), corners_l.end(), s_with_min_l_.x);
  index = (size_t)(it - corners_l.begin());

  if (index < corners_l.size()) {
    s_with_min_l_.y = std::max(corners_s[index], 0.0);
  }

  double min_s, max_s, min_l, max_l;
  min_s = *std::min_element(corners_s.begin(), corners_s.end());
  max_s = *std::max_element(corners_s.begin(), corners_s.end());
  min_l = s_with_min_l_.x;
  max_l = s_with_max_l_.x;
  if (frenet_s_ > frenet_ego_state.s()) {
    rel_s_ =
        (min_s > frenet_ego_state.s()) ? (min_s - frenet_ego_state.s()) : 0;
  } else {
    rel_s_ =
        (max_s < frenet_ego_state.s()) ? (max_s - frenet_ego_state.s()) : 0;
  }

  std::vector<double> corners_l_relative_ego = {
      frenet_obstacle_corners_.l_front_left - ego_l,
      frenet_obstacle_corners_.l_front_right - ego_l,
      frenet_obstacle_corners_.l_rear_left - ego_l,
      frenet_obstacle_corners_.l_rear_right - ego_l};
  double min_corners_l_relative_ego = *std::min_element(
      corners_l_relative_ego.begin(), corners_l_relative_ego.end());
  if (frenet_l_ >= ego_l) {
    l_relative_to_ego_ =
        min_corners_l_relative_ego > 0 ? min_corners_l_relative_ego : 0;
  } else {
    l_relative_to_ego_ =
        min_corners_l_relative_ego < 0 ? min_corners_l_relative_ego : 0;
  }

  frenet_relative_velocity_s_ =
      frenet_velocity_s_ - frenet_ego_state.velocity_s();
  frenet_relative_velocity_l_ =
      frenet_velocity_l_ - frenet_ego_state.velocity_l();
  frenet_velocity_lateral_ =
      (frenet_l_ > 0) ? frenet_velocity_l_ : -frenet_velocity_l_;

  // calculate d_rel, d_min_cpath, d_max_cpath
  double half_length = obs_length * 0.5;
  double half_width = obs_width * 0.5;
  // 对车辆障碍物half_width限定在1-2m以内
  // 其余障碍物使用其默认的half_width
  iflyauto::ObjectType type = obstacle_ptr_->type();
  bool is_vehicle_type = type == iflyauto::OBJECT_TYPE_COUPE ||
                         type == iflyauto::OBJECT_TYPE_MINIBUS ||
                         type == iflyauto::OBJECT_TYPE_VAN ||
                         type == iflyauto::OBJECT_TYPE_BUS ||
                         type == iflyauto::OBJECT_TYPE_TRUCK ||
                         type == iflyauto::OBJECT_TYPE_TRAILER;
  if (is_vehicle_type) {
    half_width = std::max(1.0, std::min(2.0, half_width));
  }

  // recalculate min_s, max_s, min_l, max_l
  if (std::fabs(half_width - obs_width * 0.5) > 1e-6) {
    std::array<int, 2> sgn_list{1, -1};
    std::array<std::vector<double>, 2> obstacle_box;
    enum box_corner { box_s, box_l };

    for (int sgn_length : sgn_list) {
      for (int sgn_width : sgn_list) {
        double _s = frenet_s_ +
                    sgn_length * std::cos(obs_relative_heading) * half_length -
                    sgn_width * std::sin(obs_relative_heading) * half_width;
        if ((frenet_s_ - ego_head_s) * (_s - ego_head_s) <= 0) {
          half_width = obs_width * 0.5;
          break;
        }
      }
    }
    if (std::fabs(half_width - obs_width * 0.5) > 1e-6) {
      for (int sgn_length : sgn_list) {
        for (int sgn_width : sgn_list) {
          double _s =
              frenet_s_ +
              sgn_length * std::cos(obs_relative_heading) * half_length -
              sgn_width * std::sin(obs_relative_heading) * half_width;
          double _l =
              frenet_l_ +
              sgn_length * std::sin(obs_relative_heading) * half_length +
              sgn_width * std::cos(obs_relative_heading) * half_width;
          obstacle_box[box_s].push_back(_s);
          obstacle_box[box_l].push_back(_l);
          // update min_s, max_s, min_l, max_l
          if (obstacle_box[box_s].size() == 1 || _s < min_s) min_s = _s;
          if (obstacle_box[box_s].size() == 1 || _s > max_s) max_s = _s;
          if (obstacle_box[box_l].size() == 1 || _l < min_l) min_l = _l;
          if (obstacle_box[box_l].size() == 1 || _l > max_l) max_l = _l;
        }
      }
    }
  }

  d_min_cpath_ = min_l;
  d_max_cpath_ = max_l;
  d_s_rel_ = 0;
  if (frenet_s_ > ego_head_s && min_s > ego_head_s) {
    d_s_rel_ = min_s - ego_head_s;  // obstacle in front of ego
  } else if (frenet_s_ < ego_head_s && max_s < ego_head_s) {
    d_s_rel_ = max_s - ego_head_s;  // obstacle behind ego
  }
}
void FrenetObstacle::compute_frenet_obstacle_boundary(
    const ReferencePath &reference_path) {
  const auto &frenet_coord = reference_path.get_frenet_coord();

  double obs_start_s(std::numeric_limits<double>::max());
  double obs_end_s(std::numeric_limits<double>::lowest());
  double obs_start_l(std::numeric_limits<double>::max());
  double obs_end_l(std::numeric_limits<double>::lowest());

  std::vector<planning_math::Vec2d> obstacle_points;
  auto perception_bounding_box = obstacle_ptr_->perception_bounding_box();
  perception_bounding_box.GetAllCorners(&obstacle_points);

  for (const planning_math::Vec2d &obs_point : obstacle_points) {
    Point2D frenet_point, carte_point;
    carte_point.x = obs_point.x();
    carte_point.y = obs_point.y();
    if (!frenet_coord->XYToSL(carte_point, frenet_point) ||
        std::isnan(frenet_point.x) || std::isnan(frenet_point.y)) {
      b_frenet_valid_ = false;
      return;
    }
    obs_start_s = std::min(obs_start_s, frenet_point.x);
    obs_end_s = std::max(obs_end_s, frenet_point.x);
    obs_start_l = std::min(obs_start_l, frenet_point.y);
    obs_end_l = std::max(obs_end_l, frenet_point.y);
  }
  // boundary frenet
  frenet_obstacle_boundary_.s_start = obs_start_s;
  frenet_obstacle_boundary_.s_end = obs_end_s;
  frenet_obstacle_boundary_.l_start = obs_start_l;
  frenet_obstacle_boundary_.l_end = obs_end_l;
}

void FrenetObstacle::compute_frenet_polygon_sequence(
    const ReferencePath &reference_path) {
  if (b_frenet_valid_ == false) {
    return;
  }
  const auto &frenet_coord = reference_path.get_frenet_coord();
  // const auto &frenet_ego_state = reference_path.get_frenet_ego_state();
  std::pair<double, double> time_range(0.0, 4.0);
  const double time_gap = 0.2;
  const bool enable_heuristic_search = true;
  constexpr double kDefaultCurvatureRadius = 5.0;
  constexpr double kMaxHeuristicDis = 5.0;

  frenet_polygon_sequence_.clear();
  // static obstacle
  if (obstacle_ptr_->is_static() ||
      (!obstacle_ptr_->trajectory().empty() &&
       std::fabs(obstacle_ptr_->trajectory().back().path_point.s() -
                 obstacle_ptr_->trajectory().front().path_point.s()) < 1.e-2)) {
    PolygonWithT polygon0, polygon1;
    polygon0.first = time_range.first;
    polygon1.first = time_range.second;
    std::vector<planning_math::Vec2d> cart_vertexes, frenet_vertexes;

    auto perception_polygon = obstacle_ptr_->perception_polygon();
    for (auto cart_vertex : perception_polygon.points()) {
      Point2D cart_point, frenet_point;
      cart_point.x = cart_vertex.x();
      cart_point.y = cart_vertex.y();
      if (!frenet_coord->XYToSL(cart_point, frenet_point) ||
          std::isnan(frenet_point.x) || std::isnan(frenet_point.y)) {
        b_frenet_polygon_sequence_invalid_ = true;
        return;
      }
      planning_math::Vec2d frenet_vertex(frenet_point.x, frenet_point.y);
      frenet_vertexes.push_back(frenet_vertex);
    }
    planning_math::Polygon2d convex_polygon;
    if (!planning_math::Polygon2d::ComputeConvexHull(frenet_vertexes,
                                                     &convex_polygon)) {
      b_frenet_polygon_sequence_invalid_ = true;
      return;
    }
    generate_precise_frenet_polygon(convex_polygon, frenet_coord);
    polygon0.second = convex_polygon;
    polygon1.second = convex_polygon;
    frenet_polygon_sequence_.push_back(polygon0);
    frenet_polygon_sequence_.push_back(polygon1);

    b_frenet_polygon_sequence_invalid_ = false;
    return;
  } else {  // dynamic obstacle
    int time_step =
        static_cast<int>((time_range.second - time_range.first) / time_gap);
    double last_s_distance = 0.0;
    std::vector<std::pair<double, double>> invalid_time_sections;
    std::pair<double, double> invalid_time_section{8.0, 0.0};
    bool has_invalid_time_section{false};
    double min_obstacle_check_length = std::min(
        2.0, std::min(obstacle_ptr_->perception_bounding_box().width(),
                      obstacle_ptr_->perception_bounding_box().length()));
    double obstacle_size =
        std::max(obstacle_ptr_->perception_bounding_box().width(),
                 obstacle_ptr_->perception_bounding_box().length());
    auto init_traj_point = obstacle_ptr_->get_point_at_time(0.0);

    for (int i = 0; i < time_step; ++i) {
      double t = i * time_gap + time_range.first;
      auto traj_point = obstacle_ptr_->get_point_at_time(t);
      if (i != 0 && i != time_step - 1 &&
          traj_point.path_point.s() - last_s_distance <
              min_obstacle_check_length) {
        continue;
      }

      // heuristic search for frenet
      bool has_heuristics = false;
      double heuristic_s_begin{0.0};
      double heuristic_s_end{0.0};
      if (enable_heuristic_search &&
          false == frenet_polygon_sequence_.empty()) {
        auto &last_polygon = frenet_polygon_sequence_.back().second;
        double min_kappa;
        double max_kappa;
        frenet_coord->GetKappaByS(
            clip(last_polygon.min_x(), frenet_coord->Length(), 0.0),
            &min_kappa);
        frenet_coord->GetKappaByS(
            clip(last_polygon.max_x(), frenet_coord->Length(), 0.0),
            &max_kappa);

        double curvature = std::max(std::abs(min_kappa), std::abs(max_kappa));
        double cur_radius = curvature > 0.0
                                ? 1.0 / curvature
                                : std::numeric_limits<double>::infinity();
        double euler_distance =
            std::abs(traj_point.path_point.s() - last_s_distance);
        double last_frenet_l = std::max(std::abs(last_polygon.min_y()),
                                        std::abs(last_polygon.max_y()));
        if (cur_radius > std::max(kDefaultCurvatureRadius,
                                  2 * (last_frenet_l + euler_distance +
                                       obstacle_size)) &&
            std::abs(traj_point.path_point.s() - last_s_distance) <
                kMaxHeuristicDis) {
          double theta = std::asin((euler_distance + obstacle_size) /
                                   (cur_radius - last_frenet_l));
          double search_buffer1 = theta * cur_radius;
          double search_buffer =
              std::max(std::abs(traj_point.path_point.s() - last_s_distance),
                       search_buffer1);
          has_heuristics = true;
          heuristic_s_begin =
              std::max(0.0, last_polygon.min_x() - search_buffer);
          heuristic_s_end = std::max(0.0, last_polygon.max_x() + search_buffer);
        }
      }

      planning_math::Polygon2d moving_polygon;
      moving_polygon = obstacle_ptr_->get_polygon_at_point(traj_point);

      PolygonWithT p_point;
      p_point.first = t;
      std::vector<planning_math::Vec2d> cart_vertexes, frenet_vertexes;
      bool is_vertexes_valid = true;
      for (auto cart_vertex : moving_polygon.points()) {
        Point2D cart_point, frenet_point;
        cart_point.x = cart_vertex.x();
        cart_point.y = cart_vertex.y();
        if (!frenet_coord->XYToSL(cart_point, frenet_point) ||
            std::isnan(frenet_point.x) || std::isnan(frenet_point.y)) {
          is_vertexes_valid = false;
          break;
        }
        planning_math::Vec2d frenet_vertex(frenet_point.x, frenet_point.y);
        frenet_vertexes.push_back(frenet_vertex);
      }
      if (!is_vertexes_valid) {
        if (!has_invalid_time_section) {
          has_invalid_time_section = true;
          if (frenet_polygon_sequence_.empty()) {
            invalid_time_section.first = t;
          } else {
            invalid_time_section.first =
                (frenet_polygon_sequence_.back().first + t) / 2.0;
          }
        }
        invalid_time_section.second = std::max(invalid_time_section.second, t);
        if (i == time_step - 1) {
          invalid_time_sections.emplace_back(invalid_time_section);
        }
        continue;
      }
      planning_math::Polygon2d convex_polygon;
      if (!planning_math::Polygon2d::ComputeConvexHull(frenet_vertexes,
                                                       &convex_polygon)) {
        if (!has_invalid_time_section) {
          has_invalid_time_section = true;
          if (frenet_polygon_sequence_.empty()) {
            invalid_time_section.first = t;
          } else {
            invalid_time_section.first =
                (frenet_polygon_sequence_.back().first + t) / 2.0;
          }
        }
        invalid_time_section.second = std::max(invalid_time_section.second, t);
        if (i == time_step - 1) {
          invalid_time_sections.emplace_back(invalid_time_section);
        }
        continue;
      }
      generate_precise_frenet_polygon(convex_polygon, frenet_coord);
      p_point.second = convex_polygon;
      if (has_invalid_time_section) {
        has_invalid_time_section = false;
        invalid_time_section.second =
            std::max(invalid_time_section.second,
                     (invalid_time_section.second + t) / 2.0);
        invalid_time_sections.emplace_back(invalid_time_section);
        invalid_time_section = {8.0, 0.0};
      }
      frenet_polygon_sequence_.push_back(p_point);
      last_s_distance = traj_point.path_point.s();
    }
    frenet_polygon_sequence_.set_invalid_time_sections(invalid_time_sections);
    if (frenet_polygon_sequence_.size() >= 2) {
      b_frenet_polygon_sequence_invalid_ = false;
    } else {
      b_frenet_polygon_sequence_invalid_ = true;
      frenet_polygon_sequence_.clear();
    }
  }
}

void FrenetObstacle::generate_precise_frenet_polygon(
    planning_math::Polygon2d &polygon,
    std::shared_ptr<planning_math::KDPath> frenet_coord) {
  planning_math::Polygon2d result_polygon;
  double max_curvature = 0.0;
  double min_l = std::numeric_limits<double>::max();
  constexpr int kMaxInterpolateNums = 5;
  constexpr double kMinCurvatureRadius = 5.0;
  constexpr double kDeltaRadian = 0.4;
  constexpr double kLRange = 5.0;
  auto origin_points = polygon.GetAllVertices();
  assert(origin_points.size() > 2);
  std::vector<double> curvatures;
  for (auto point : origin_points) {
    if (point.x() < 0.0 || point.x() > frenet_coord->Length()) {
      return;
    }
    double curvature;
    frenet_coord->GetKappaByS(point.x(), &curvature);
    curvatures.push_back(curvature);
    max_curvature = std::max(max_curvature, curvature);
    min_l = std::min(min_l, std::abs(point.y()));
  }
  double cur_radius = max_curvature > 0.0
                          ? 1.0 / max_curvature
                          : std::numeric_limits<double>::infinity();
  if (cur_radius > kMinCurvatureRadius) {
    return;
  }
  if (min_l > kLRange) {
    return;
  }
  std::vector<planning_math::Vec2d> new_points;
  std::vector<planning_math::Vec2d> origin_cart_points;
  for (auto point : origin_points) {
    Point2D fren_point, cart_point;
    fren_point.x = point.x();
    fren_point.y = point.y();
    if (!frenet_coord->SLToXY(fren_point, cart_point)) {
      return;
    }
    origin_cart_points.push_back({cart_point.x, cart_point.y});
  }
  for (size_t i = 1; i <= origin_points.size(); ++i) {
    size_t index_begin = (i - 1) % origin_points.size();
    size_t index_end = i % origin_points.size();
    auto begin_point = origin_points[index_begin];
    auto end_point = origin_points[index_end];
    double s_diff = std::abs(end_point.x() - begin_point.x());
    double ref_curvature =
        std::max(curvatures[index_begin], curvatures[index_end]);
    double delta_s = std::max(0.5, kDeltaRadian / ref_curvature);
    int inter_num =
        std::min(kMaxInterpolateNums, std::max(2, int(ceil(s_diff / delta_s))));
    for (int j = 1; j <= inter_num; ++j) {
      Point2D cur_cart_p, cur_fren_p;
      if (j == inter_num) {
        new_points.emplace_back(end_point);
        continue;
      }
      cur_cart_p.x =
          planning_math::lerp(origin_cart_points[index_begin].x(), 0,
                              origin_cart_points[index_end].x(), inter_num, j);
      cur_cart_p.y =
          planning_math::lerp(origin_cart_points[index_begin].y(), 0,
                              origin_cart_points[index_end].y(), inter_num, j);
      if (!frenet_coord->XYToSL(cur_cart_p, cur_fren_p)) {
        return;
      }
      assert(!std::isnan(cur_fren_p.x) && !std::isnan(cur_fren_p.y));
      planning_math::Vec2d cur_point(cur_fren_p.x, cur_fren_p.y);
      new_points.emplace_back(cur_point);
    }
  }

  if (new_points.size() <= 2) {
    return;
  }

  if (!planning_math::Polygon2d::ComputeConvexHull(new_points,
                                                   &result_polygon)) {
    return;
  } else {
    polygon = result_polygon;
  }
}

bool FrenetObstacle::get_polygon_at_time(
    const double relative_time,
    const std::shared_ptr<ReferencePath> &reference_path,
    planning_math::Polygon2d &obstacle_polygon) const {
  auto enu_polygon = obstacle_ptr_->get_polygon_at_point(
      obstacle_ptr_->get_point_at_time(relative_time));

  auto &frenet_coord = reference_path->get_frenet_coord();
  std::vector<planning_math::Vec2d> frenet_points;
  for (auto &pt : enu_polygon.points()) {
    Point2D frenet_point, carte_point;
    carte_point.x = pt.x();
    carte_point.y = pt.y();
    if (!frenet_coord->XYToSL(carte_point, frenet_point)) {
      LOG_DEBUG(
          "Frenet_coord failed, the obstacle [%i]'s enu ploygon min_x: [%f], "
          "max_x: [%f], min_y: [%f], max_y: [%f] \n",
          obstacle_ptr_->id(), enu_polygon.min_x(), enu_polygon.max_x(),
          enu_polygon.min_y(), enu_polygon.max_y());
      continue;
    }
    frenet_points.push_back(
        planning_math::Vec2d(frenet_point.x, frenet_point.y));
  }

  bool ok = planning_math::Polygon2d::ComputeConvexHull(frenet_points,
                                                        &obstacle_polygon);
  return ok;
}

bool FrenetObstacle::get_polygon_at_time_tmp(
    const double relative_time,
    const std::shared_ptr<ReferencePath> &reference_path,
    planning_math::Polygon2d &obstacle_polygon) const {
  // auto enu_polygon = obstacle_ptr_->get_polygon_at_point(
  //     obstacle_ptr_->get_point_at_time(relative_time));

  // auto &frenet_coord = reference_path->get_frenet_coord();
  // std::vector<planning_math::Vec2d> frenet_points;
  // for (auto &pt : enu_polygon.points()) {
  //   Point2D frenet_point, carte_point;
  //   carte_point.x = pt.x();
  //   carte_point.y = pt.y();
  //   if (!frenet_coord->XYToSL(carte_point, frenet_point)) {
  //     LOG_DEBUG(
  //         "Frenet_coord failed, the obstacle [%i]'s enu ploygon min_x: [%f],
  //         " "max_x: [%f], min_y: [%f], max_y: [%f] \n", obstacle_ptr_->id(),
  //         enu_polygon.min_x(), enu_polygon.max_x(), enu_polygon.min_y(),
  //         enu_polygon.max_y());
  //     continue;
  //   }
  //   frenet_points.push_back(
  //       planning_math::Vec2d(frenet_point.x, frenet_point.y));
  // }

  // bool ok = planning_math::Polygon2d::ComputeConvexHull(frenet_points,
  //                                                       &obstacle_polygon);
  // return ok;
  // Hack(clren): 当前无预测，横向l不变，纵向速度递推
  double prediction_frenet_s = frenet_velocity_s_ * relative_time;
  auto enu_polygon =
      obstacle_ptr_->get_polygon_at_point(obstacle_ptr_->get_point_at_time(0));

  auto &frenet_coord = reference_path->get_frenet_coord();
  std::vector<planning_math::Vec2d> frenet_points;
  for (auto &pt : enu_polygon.points()) {
    Point2D frenet_point, carte_point;
    carte_point.x = pt.x();
    carte_point.y = pt.y();
    if (!frenet_coord->XYToSL(carte_point, frenet_point)) {
      LOG_DEBUG(
          "Frenet_coord failed, the obstacle [%i]'s enu ploygon min_x: [%f], "
          "max_x: [%f], min_y: [%f], max_y: [%f] \n",
          obstacle_ptr_->id(), enu_polygon.min_x(), enu_polygon.max_x(),
          enu_polygon.min_y(), enu_polygon.max_y());
      continue;
    }
    frenet_points.push_back(planning_math::Vec2d(
        frenet_point.x + prediction_frenet_s, frenet_point.y));
  }

  bool ok = planning_math::Polygon2d::ComputeConvexHull(frenet_points,
                                                        &obstacle_polygon);
  return ok;
}

}  // namespace planning
