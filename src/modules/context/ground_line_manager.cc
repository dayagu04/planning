#include "ground_line_manager.h"

#include <utility>

namespace planning {
GroundLineManager::GroundLineManager() { Init(); }

void GroundLineManager::Init() {
  points_.clear();
  min_pts_ = 3;
  eps_ = 0.5;
  is_cluster_ = false;
}

bool GroundLineManager::Update(const Map::StaticMap &static_map_info) {
  points_.clear();
  const Map::ParkingAssistInfo &groundline_data =
      static_map_info.parking_assist_info();
  if (groundline_data.road_obstacles_size() > 0) {
    if (is_cluster_) {
      std::vector<GroundLinePoint> ground_line_point;
      for (auto &groundline : groundline_data.road_obstacles()) {
        for (size_t i = 0; i < groundline.shape_size(); ++i) {
          GroundLinePoint point;
          point.point = planning_math::Vec2d(groundline.shape(i).x(),
                                             groundline.shape(i).y());
          point.status = GroundLinePoint::Status::UNCLASSIFIED;
          ground_line_point.emplace_back(point);
        }
      }
      points_ = Execute(ground_line_point);
    } else {
      std::vector<GroundLinePoints> ground_line_points;
      for (auto &groundline : groundline_data.road_obstacles()) {
        if (groundline.shape_size() >= 3) {
          GroundLinePoints point;
          for (size_t i = 0; i < groundline.shape_size(); ++i) {
            point.emplace_back(planning_math::Vec2d(groundline.shape(i).x(),
                                                    groundline.shape(i).y()));
          }
          points_.emplace_back(point);
        }
      }
    }
  }
  return true;
}

bool GroundLineManager::Update(
    const iflyauto::FusionGroundLineInfo &fusion_ground_line_info) {
  points_.clear();
  const size_t groundline_size = fusion_ground_line_info.groundline_size;
  if (groundline_size > 0) {
    std::vector<GroundLinePoint> ground_line_point;
    for (size_t i = 0; i < groundline_size; ++i) {
      const auto &groundline = fusion_ground_line_info.groundline[i];
      for (size_t j = 0; j < groundline.groundline_point_size; ++j) {
        GroundLinePoint point;
        point.point =
            planning_math::Vec2d(groundline.shape[j].x, groundline.shape[j].y);
        point.status = GroundLinePoint::Status::UNCLASSIFIED;
        ground_line_point.emplace_back(point);
      }
    }
    points_ = Execute(ground_line_point);
  }
  return true;
}

void GroundLineManager::UpdateParams(const int min_pts, const double eps) {
  min_pts_ = min_pts;
  eps_ = eps;
}

std::vector<std::vector<planning_math::Vec2d>> GroundLineManager::Execute(
    const std::vector<GroundLinePoint> &ground_line_points) {
  std::vector<std::vector<planning_math::Vec2d>> result;
  std::vector<GroundLinePoint> points = ground_line_points;
  // update_points(ground_line_points, points);
  for (auto &point : points) {
    if (point.status == GroundLinePoint::Status::UNCLASSIFIED) {
      std::vector<planning_math::Vec2d> cluster = ExpandCluster(point, points);
      if (cluster.size() > 0) {
        result.emplace_back(cluster);
      }
    }
  }
  return result;
}

void GroundLineManager::UpdatePoints(
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

std::vector<int> GroundLineManager::CalcCluster(
    GroundLinePoint &point, std::vector<GroundLinePoint> &points) {
  std::vector<int> cluster_index;

  for (size_t i = 0; i < points.size(); i++) {
    if (point != points[i] && point.point.DistanceTo(points[i].point) <= eps_) {
      cluster_index.emplace_back(i);
    }
  }
  return cluster_index;
}

std::vector<planning_math::Vec2d> GroundLineManager::ExpandCluster(
    GroundLinePoint &point, std::vector<GroundLinePoint> &points) {
  std::vector<planning_math::Vec2d> result;
  result.clear();

  std::vector<int> cluster = CalcCluster(point, points);
  if (static_cast<int>(cluster.size()) < min_pts_) {
    point.status = GroundLinePoint::Status::NOISE;
    return result;
  }

  point.status = GroundLinePoint::Status::CLASSIFIED;
  result.emplace_back(point.point);
  for (size_t i = 0; i < cluster.size(); i++) {
    std::vector<int> cluster_exp = CalcCluster(points.at(cluster[i]), points);
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
