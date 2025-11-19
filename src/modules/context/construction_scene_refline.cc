#include "construction_scene_refline.h"

namespace planning {
static constexpr double kSamplingBackwardDistance = 5.0;
static constexpr double kSamplingBoundaryLonGap = 5.0;
static constexpr double kSamplingBoundaryLatGap = 0.3;
static constexpr double kSamplingLineLonGap = 2.0;
static constexpr double kLateralDistanceDiff = 0.05;
static constexpr double kAgentForwardDistance = 2.0;
static constexpr double kAgentBackwardDistance = 5.0;
static constexpr double kMinLaneWidthBuffer = 0.4;

ConstructionSceneRefline::ConstructionSceneRefline() {
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  min_lane_width_ =
      vehicle_param.max_width + 2.0 * kMinLaneWidthBuffer;
  InitInfo();
}

bool ConstructionSceneRefline::InitInfo() {
  lane_num_ = 0;
  center_line_.clear();
  construction_ref_path_.clear();
  return true;
}

bool ConstructionSceneRefline::Update(
    const std::shared_ptr<LaneReferencePath>& reference_path,
    const std::map<int, RoadBoundaryCluster>& road_boundaries_clusters_map,
    const std::map<int, ConstructionAgentClusterArea>& construction_agent_cluster_attribute_set) {
  // check reference path
  if (reference_path == nullptr) {
    return false;
  }
  // check frenet coord
  const auto& frenet_coord = reference_path->get_frenet_coord();
  if (frenet_coord == nullptr) {
    return false;
  }
  // check road border and static objects
  if (road_boundaries_clusters_map.empty() &&
      construction_agent_cluster_attribute_set.empty()) {
    return false;
  }
  std::map<planning::ConstructionDirection, std::map<double, double>> boundaries;  // direction, s, l
  // handle road boundary
  ExtractRoadBoundaries(frenet_coord, road_boundaries_clusters_map, boundaries);
  // handle agent cluster
  ExtractAgentBoundaries(reference_path, construction_agent_cluster_attribute_set, boundaries);
  // generate final boundary
  bool is_boundary_valid = GeneratePassableBoundary(boundaries);
  if (is_boundary_valid) {
    std::vector<Point2d> frenet_refline;
    // generate center lines
    GenerateCenterLines(reference_path, frenet_refline);
    // generate center lines
    GenerateConstructionRefLine(reference_path, frenet_refline);
    return true;
  } else {
    // update history center lines
  }
  return false;
}

bool ConstructionSceneRefline::ExtractRoadBoundaries(
    const std::shared_ptr<planning::planning_math::KDPath>& frenet_coord,
    const std::map<int, RoadBoundaryCluster>& road_boundary_map,
    std::map<planning::ConstructionDirection, std::map<double, double>>& boundaries) {
  // 基于施工区域路沿的位置和方向计算边界
  for (const auto& [cluster_id, cluster_area] : road_boundary_map) {
    if (cluster_area.points.empty()) {
      continue;
    }
    if (cluster_area.direction != ConstructionDirection::LEFT &&
        cluster_area.direction != ConstructionDirection::RIGHT) {
      continue;
    }
    for (const auto& point : cluster_area.points) {
      double point_s, point_l;
      if (frenet_coord->XYToSL(point.x, point.y, &point_s, &point_l)) {
        auto last_point = boundaries[cluster_area.direction].find(point_s);
        if (last_point != boundaries[cluster_area.direction].end()) {
          if (cluster_area.direction == ConstructionDirection::LEFT) {
            boundaries[cluster_area.direction][point_s] = std::max(last_point->second, point_l);
          } else if (cluster_area.direction == ConstructionDirection::RIGHT) {
            boundaries[cluster_area.direction][point_s] = std::min(last_point->second, point_l);
          }
        } else {
          boundaries[cluster_area.direction][point_s] = point_l;
        }
      }
    }
  }
  return true;
}

bool ConstructionSceneRefline::ExtractAgentBoundaries(
    const std::shared_ptr<planning::LaneReferencePath>& reference_path,
    const std::map<int, ConstructionAgentClusterArea>& agent_map,
    std::map<planning::ConstructionDirection, std::map<double, double>>& boundaries) {
  const auto& obs_map = reference_path->get_obstacles_map();
  // 基于施工区域障碍物的位置和方向计算边界
  for (const auto& [cluster_id, cluster_area] : agent_map) {
    if (cluster_area.points.empty()) {
      continue;
    }
    if (cluster_area.direction != ConstructionDirection::LEFT &&
        cluster_area.direction != ConstructionDirection::RIGHT) {
      continue;
    }
    for (const auto& point : cluster_area.points) {
      auto it = obs_map.find(point.id);
      if (it == obs_map.end()) {
        continue;
      }
      if (!it->second->b_frenet_valid()) {
        continue;
      }
      const auto& obs_boundary = it->second->frenet_obstacle_boundary();
      double obs_nearest_l = obs_boundary.l_start;
      if (cluster_area.direction == ConstructionDirection::LEFT) {
        obs_nearest_l =  obs_boundary.l_end;
      }
      for (double obs_s = obs_boundary.s_start - kAgentBackwardDistance;
           obs_s < obs_boundary.s_end + kAgentForwardDistance + kSamplingLineLonGap;
           obs_s += kSamplingLineLonGap) {
        auto last_point_iter = boundaries[cluster_area.direction].find(obs_s);
        if (last_point_iter != boundaries[cluster_area.direction].end()) {
          if (cluster_area.direction == ConstructionDirection::LEFT) {
            boundaries[cluster_area.direction][obs_s] = std::max(last_point_iter->second, obs_nearest_l);
          } else if (cluster_area.direction == ConstructionDirection::RIGHT) {
            boundaries[cluster_area.direction][obs_s] = std::min(last_point_iter->second, obs_nearest_l);
          }
        } else {
          boundaries[cluster_area.direction][obs_s] = obs_nearest_l;
        }
      }
    }
  }
  return true;
}

bool ConstructionSceneRefline::GeneratePassableBoundary(
    const std::map<planning::ConstructionDirection, std::map<double, double>>& boundaries) {
  // left boundary
  std::vector<double> left_s_vec;
  std::vector<double> left_l_vec;
  auto left_boundaries_iter = boundaries.find(ConstructionDirection::RIGHT);
  if (left_boundaries_iter != boundaries.end()) {
    const auto& left_boundaries = left_boundaries_iter->second;
    double last_s = left_boundaries.begin()->first;
    double last_l = left_boundaries.begin()->second;
    for (auto iter = left_boundaries.begin(); iter != left_boundaries.end(); ++iter) {
      double ds = iter->first - last_s;
      if (ds <= 1e-6) {
        continue;
      }
      if (ds > kSamplingBoundaryLonGap) {
        left_s_vec.emplace_back(last_s);
        left_l_vec.emplace_back(last_l);
        last_s = iter->first;
        if (iter->second - last_l > kSamplingBoundaryLatGap) {
          last_l += kSamplingBoundaryLatGap;
        } else {
          last_l = iter->second;
        }
      } else {
        if (iter->second < last_l) {
          // last_s = iter->first;
          last_l = iter->second;
        }
      }
    }
    if (!left_s_vec.empty()) {
      if (last_s - left_s_vec.back() > 1e-6) {
        left_s_vec.emplace_back(last_s);
        left_l_vec.emplace_back(last_l);
      }
    }
  }
  //  right boundary
  std::vector<double> right_s_vec;
  std::vector<double> right_l_vec;
  auto right_boundaries_iter = boundaries.find(ConstructionDirection::LEFT);
  if (right_boundaries_iter != boundaries.end()) {
    const auto& right_boundaries = right_boundaries_iter->second;
    double last_s = right_boundaries.begin()->first;
    double last_l = right_boundaries.begin()->second;
    for (auto iter = right_boundaries.begin(); iter != right_boundaries.end(); ++iter) {
      double ds = iter->first - last_s;
      if (ds <= 1e-6) {
        continue;
      }
      if (ds > kSamplingBoundaryLonGap) {
        right_s_vec.emplace_back(last_s);
        right_l_vec.emplace_back(last_l);
        last_s = iter->first;
        if (last_l - iter->second > kSamplingBoundaryLatGap) {
          last_l -= kSamplingBoundaryLatGap;
        } else {
          last_l = iter->second;
        }
      } else {
        if (iter->second > last_l) {
          // last_s = iter->first;
          last_l = iter->second;
        }
      }
    }
    if (!right_s_vec.empty()) {
      if (last_s - right_s_vec.back() > 1e-6) {
        right_s_vec.emplace_back(last_s);
        right_l_vec.emplace_back(last_l);
      }
    }
  }
  // spline
  if (left_s_vec.size() < 4 &&
      right_s_vec.size() < 4) {
    return false;
  } else if (left_s_vec.size() < 4) {
    left_s_vec.clear();
    left_l_vec.clear();
    left_s_vec = right_s_vec;
    for (size_t i = 0; i < right_l_vec.size(); ++i) {
      left_l_vec.emplace_back(right_l_vec[i] + min_lane_width_);
    }
  } else if (right_s_vec.size() < 4) {
    right_s_vec.clear();
    right_l_vec.clear();
    right_s_vec = left_s_vec;
    for (size_t i = 0; i < left_l_vec.size(); ++i) {
      right_l_vec.emplace_back(left_l_vec[i] - min_lane_width_);
    }
  }
  left_boundary_spline_.set_points(left_s_vec, left_l_vec);
  right_boundary_spline_.set_points(right_s_vec, right_l_vec);
  return true;
}

bool ConstructionSceneRefline::GenerateCenterLines(
    const std::shared_ptr<planning::LaneReferencePath>& reference_path,
    std::vector<Point2d>& frenet_refline) {
  double left_boundary_start = left_boundary_spline_.get_x().front();
  double left_boundary_end = left_boundary_spline_.get_x().back();
  double right_boundary_start = right_boundary_spline_.get_x().front();
  double right_boundary_end = right_boundary_spline_.get_x().back();
  double min_valid_line_start_s =
      std::min(left_boundary_start, right_boundary_start);
  double max_valid_line_end_s =
      std::max(left_boundary_end, right_boundary_end);
  // min_boundary_width
  double min_boundary_width = 20.0;
  std::pair<double, double> narrowest_boundary{20.0, -20.0};
  for (double line_s = min_valid_line_start_s; line_s < max_valid_line_end_s; line_s += kSamplingLineLonGap) {
    double left_boundary_l = left_boundary_spline_(line_s);
    if (line_s < left_boundary_start) {
      left_boundary_l = std::max(left_boundary_spline_.get_y().front(), 0.5 * min_lane_width_);
    } else if (line_s > left_boundary_end) {
      left_boundary_l = std::max(left_boundary_spline_.get_y().back(), 0.5 * min_lane_width_);
    }
    double right_boundary_l = right_boundary_spline_(line_s);
    if (line_s < right_boundary_start) {
      right_boundary_l = std::min(right_boundary_spline_.get_y().front(), -0.5 * min_lane_width_);
    } else if (line_s > right_boundary_end) {
      right_boundary_l = std::min(right_boundary_spline_.get_y().back(), -0.5 * min_lane_width_);
    }
    double lateral_distance = std::max(left_boundary_l - right_boundary_l, 0.0);
    if (lateral_distance < min_boundary_width) {
      min_boundary_width = lateral_distance;
      narrowest_boundary.first = left_boundary_l;
      narrowest_boundary.second = right_boundary_l;
    }
  }
  // lane num
  lane_num_ = std::max(static_cast<int>(std::floor(min_boundary_width / min_lane_width_)), 1);
  // init line
  size_t max_valid_line_size =
      std::max(left_boundary_spline_.get_x().size(), right_boundary_spline_.get_x().size());
  for (int i = 0; i < lane_num_; ++i) {
    center_line_[i].reserve(max_valid_line_size);
  }
  frenet_refline.reserve(max_valid_line_size);
  // generate lines
  double last_left_boundary_l = 20.0;
  double last_right_boundary_l = -20.0;
  double raw_line_end_s = reference_path->get_points().back().path_point.s();
  for (double line_s = 0.0; line_s <= raw_line_end_s; line_s += kSamplingLineLonGap) {
    double lane_width = min_lane_width_;
    ReferencePathPoint refpath_pt;
    if (reference_path->get_reference_point_by_lon(line_s, refpath_pt)) {
      lane_width = std::max(refpath_pt.lane_width + kMinLaneWidthBuffer, lane_width);
    }
    double half_lane_width = 0.5 * lane_width;
    double left_boundary_l = left_boundary_spline_(line_s);
    if (line_s < left_boundary_start) {
      left_boundary_l = std::max(left_boundary_spline_.get_y().front(), half_lane_width);
    } else if (line_s > left_boundary_end) {
      left_boundary_l = std::max(left_boundary_spline_.get_y().back(), half_lane_width);
    }
    double right_boundary_l = right_boundary_spline_(line_s);
    if (line_s < right_boundary_start) {
      right_boundary_l = std::min(right_boundary_spline_.get_y().front(), -half_lane_width);
    } else if (line_s > right_boundary_end) {
      right_boundary_l = std::min(right_boundary_spline_.get_y().back(), -half_lane_width);
    }
    if (left_boundary_l > last_left_boundary_l &&
        right_boundary_l < last_right_boundary_l) {
      left_boundary_l = last_left_boundary_l;
      right_boundary_l = last_right_boundary_l;
    } else {
      double diff_left_l = left_boundary_l - last_left_boundary_l;
      double diff_right_l = right_boundary_l - last_right_boundary_l;
      if (diff_left_l > 1e-6 &&
          diff_right_l > 1e-6 &&
          diff_left_l > diff_right_l) {
        left_boundary_l = last_left_boundary_l + diff_right_l;
      } else if (diff_left_l < -1e-6 &&
                 diff_right_l < -1e-6 &&
                 diff_left_l > diff_right_l) {
        right_boundary_l = last_right_boundary_l + diff_left_l;
      }
    }
    double lateral_distance = left_boundary_l - right_boundary_l;
    if (lateral_distance <= 1e-6) {
      double mid_boundary_l = 0.5 * (left_boundary_l + right_boundary_l);
      left_boundary_l = mid_boundary_l;
      right_boundary_l = mid_boundary_l;
      lateral_distance = 0.;
    }
    Point2d pt(line_s, 0.);
    double nearest_dist = 20.0;
    double diff_l = lateral_distance / std::max(lane_num_, 1);
    for (int lane_index = 0; lane_index < lane_num_; ++lane_index) {
      double line_l = diff_l * (lane_index + 0.5) + right_boundary_l;
      center_line_[lane_index].emplace_back(Point2d(line_s, line_l));
      double init_point_dist_to_line = std::fabs(line_l);
      if (init_point_dist_to_line < nearest_dist) {
        nearest_dist = init_point_dist_to_line;
        pt.y = line_l;
      }
    }
    if (lateral_distance >= lane_width) {
      double dist_to_left_boundary = std::min(left_boundary_l - half_lane_width, 0.0);
      double dist_to_right_boundary = std::max(right_boundary_l + half_lane_width, 0.0);
      pt.y = (dist_to_left_boundary + dist_to_right_boundary);
    } else {
      pt.y = 0.5 * (left_boundary_l + right_boundary_l);
    }
    if (!frenet_refline.empty()) {
      double last_line_l = frenet_refline.back().y;
      if (last_line_l < left_boundary_l - 0.5 * min_lane_width_ &&
          last_line_l > right_boundary_l + 0.5 * min_lane_width_) {
        if (pt.y - last_line_l > kLateralDistanceDiff) {
          pt.y = last_line_l + kLateralDistanceDiff;
        } else if (last_line_l - pt.y > kLateralDistanceDiff) {
          pt.y = last_line_l - kLateralDistanceDiff;
        }
      }
    }
    frenet_refline.emplace_back(std::move(pt));
  }
  return true;
}

bool ConstructionSceneRefline::GenerateConstructionRefLine(
    const std::shared_ptr<planning::LaneReferencePath>& reference_path,
    const std::vector<Point2d>& frenet_refline) {
  const auto& frenet_coord = reference_path->get_frenet_coord();
  std::vector<double> refline_x_vec, refline_y_vec;
  refline_x_vec.reserve(frenet_refline.size());
  refline_y_vec.reserve(frenet_refline.size());
  construction_ref_path_.reserve(frenet_refline.size());
  constexpr double kDefaultLaneBorderDis = 20.0;
  for (const Point2d& line_point : frenet_refline) {
    ReferencePathPoint raw_ref_path_pt;
    if (reference_path->get_reference_point_by_lon(line_point.x, raw_ref_path_pt)) {
      ReferencePathPoint ref_path_pt;
      double point_x, point_y;
      if (frenet_coord->SLToXY(line_point.x, line_point.y, &point_x, &point_y)) {
        refline_x_vec.emplace_back(point_x);
        refline_y_vec.emplace_back(point_y);
        ref_path_pt.path_point.set_x(point_x);
        ref_path_pt.path_point.set_y(point_y);
        ref_path_pt.distance_to_left_road_border = std::fmin(
            raw_ref_path_pt.distance_to_left_road_border - line_point.y, kDefaultLaneBorderDis);
        ref_path_pt.distance_to_right_road_border = std::fmin(
            raw_ref_path_pt.distance_to_right_road_border + line_point.y, kDefaultLaneBorderDis);
        ref_path_pt.distance_to_left_lane_border = raw_ref_path_pt.lane_width * 0.5;
        ref_path_pt.distance_to_right_lane_border = raw_ref_path_pt.lane_width * 0.5;
        ref_path_pt.left_road_border_type = raw_ref_path_pt.left_road_border_type;
        ref_path_pt.right_road_border_type = raw_ref_path_pt.right_road_border_type;
        ref_path_pt.left_lane_border_type = raw_ref_path_pt.left_lane_border_type;
        ref_path_pt.right_lane_border_type = raw_ref_path_pt.right_lane_border_type;
        ref_path_pt.lane_width = raw_ref_path_pt.lane_width;
        ref_path_pt.max_velocity = raw_ref_path_pt.max_velocity;
        ref_path_pt.min_velocity = raw_ref_path_pt.min_velocity;
        ref_path_pt.type = ReferencePathPointType::MAP;
        ref_path_pt.is_in_intersection = raw_ref_path_pt.is_in_intersection;
        construction_ref_path_.emplace_back(std::move(ref_path_pt));
      }
    }
  }
  JSON_DEBUG_VECTOR("construction_refline_x", refline_x_vec, 2);
  JSON_DEBUG_VECTOR("construction_refline_y", refline_y_vec, 2);
  return true;
}

}  // namespace planning