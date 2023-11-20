#include "groundline_decider.h"


namespace planning {

int GroundLineDecider::min_pts_ = 1;
double GroundLineDecider::eps_ = 0.5;

void GroundLineDecider::update_params(const int min_pts, const double eps) {
  min_pts_ = min_pts;
  eps_ = eps;
}

std::vector<std::vector<planning_math::Vec2d>> GroundLineDecider::execute(
    const std::vector<GroundLinePoint> &ground_line_points) {

  std::vector<std::vector<planning_math::Vec2d>> result;
  std::vector<GroundLinePoint> points = ground_line_points;
  // update_points(ground_line_points, points);
  for (auto &point : points) {
    if (point.status == GroundLinePoint::Status::UNCLASSIFIED) {
      std::vector<planning_math::Vec2d> cluster = expand_cluster(point, points);
      if (cluster.size() > 0) {
        result.emplace_back(cluster);
      }
    }
  }
  return result;
}

void GroundLineDecider::update_points(
    const std::vector<GroundLinePoint> &ground_line_points,
    std::vector<GroundLinePoint> &points) {
  // points.clear();
  // for (auto &pt : ground_line_points) {
  //   GroundLinePoint point;
  //   point.point = planning_math::Vec2d(pt.position().x(), pt.position().y());
  //   point.status = GroundLinePoint::Status::UNCLASSIFIED;
  //   points.emplace_back(point);
  // }
}

std::vector<int> GroundLineDecider::calc_cluster(
    GroundLinePoint &point, std::vector<GroundLinePoint> &points) {
  std::vector<int> cluster_index;

  for (size_t i = 0; i < points.size(); i++) {
    if (point != points[i] && point.point.DistanceTo(points[i].point) <= eps_) {
      cluster_index.emplace_back(i);
    }
  }
  return cluster_index;
}

std::vector<planning_math::Vec2d> GroundLineDecider::expand_cluster(
    GroundLinePoint &point, std::vector<GroundLinePoint> &points) {
  std::vector<planning_math::Vec2d> result;
  result.clear();

  std::vector<int> cluster = calc_cluster(point, points);
  if (static_cast<int>(cluster.size()) + 1 < min_pts_) {
    point.status = GroundLinePoint::Status::NOISE;
    return result;
  }

  point.status = GroundLinePoint::Status::CLASSIFIED;
  result.emplace_back(point.point);
  for (size_t i = 0; i < cluster.size(); i++) {
    std::vector<int> cluster_exp = calc_cluster(points.at(cluster[i]), points);
    points.at(cluster[i]).status = GroundLinePoint::Status::CLASSIFIED;
    result.emplace_back(points.at(cluster[i]).point);

    if (static_cast<int>(cluster_exp.size()) >= min_pts_) {
      for (size_t j = 0; j < cluster_exp.size(); j++) {
        if (points.at(cluster_exp[j]).status ==
            GroundLinePoint::Status::UNCLASSIFIED) {
          if (find(cluster.begin(), cluster.end(), cluster_exp[j]) ==
              cluster.end()) {
            cluster.push_back(cluster_exp[j]);
          }
        } else if (points.at(cluster_exp[j]).status ==
                   GroundLinePoint::Status::NOISE) {
          points.at(cluster_exp[j]).status =
              GroundLinePoint::Status::CLASSIFIED;
          result.emplace_back(points.at(cluster_exp[j]).point);
        }
      }
    }
  }

  return result;
}

}  // namespace planning
