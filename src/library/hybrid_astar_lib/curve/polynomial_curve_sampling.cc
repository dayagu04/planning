#include "polynomial_curve_sampling.h"
#include <cmath>
#include "geometry_math.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "math_utils.h"
#include "src/library/hybrid_astar_lib/decider/path_comparator.h"

namespace planning {

PolynomialCurveSampling::PolynomialCurveSampling(
    const MapBound* XYbounds, const ParkObstacleList* obstacles,
    const AstarRequest* request, EulerDistanceTransform* edt,
    const ObstacleClearZone* clear_zone, ParkReferenceLine* ref_line,
    const PlannerOpenSpaceConfig* config, const float min_radius,
    std::shared_ptr<NodeCollisionDetect> collision_detect)
    : CurveSampling(XYbounds, obstacles, request, edt, clear_zone, ref_line,
                    config, min_radius, collision_detect) {}

bool PolynomialCurveSampling::SamplingByQunticPolynomial(
    Node3d* current_node, std::vector<AStarPathPoint>& path,
    Node3d* polynomial_node, PolynomialPathErrorCode* fail_type) {
  *fail_type = PolynomialPathErrorCode::NONE;
  if (current_node->GetY() > 0.8 || current_node->GetY() < -0.8) {
    *fail_type = PolynomialPathErrorCode::OUT_OF_Y_BOUNDARY;
    return false;
  }

  float x_diff = current_node->GetX() - request_->real_goal.GetX();
  if (x_diff < 0.2) {
    *fail_type = PolynomialPathErrorCode::OUT_OF_X_BOUNDARY;
    return false;
  }

  if (std::fabs(current_node->GetRadius()) < min_radius_ - 1e-5) {
    *fail_type = PolynomialPathErrorCode::LINK_POINT_INVALID_KAPPA;
    return false;
  }

  float heading_diff = IflyUnifyTheta(current_node->GetPhi(), M_PIf32) -
                       IflyUnifyTheta(search_goal_.GetPhi(), M_PIf32);
  heading_diff = IflyUnifyTheta(heading_diff, M_PIf32);
  if (heading_diff > ifly_deg2rad(60.0) || heading_diff < ifly_deg2rad(-60.0)) {
    *fail_type = PolynomialPathErrorCode::OUT_OF_HEADING_BOUNDARY;
    return false;
  }

  // init
  polynomial_node->Clear();
  float min_straight_dist = 0.7;
  float sample_range =
      search_goal_.GetX() - (request_->real_goal.GetX() + min_straight_dist);
  int sampline_numer = std::ceil(sample_range / 0.1);
  sampline_numer = std::max(1, sampline_numer);
  bool valid_path = false;

  Pose2f end_pose = search_goal_;
  for (int i = 0; i < sampline_numer; i++) {
    path.clear();

    // check two point position, to delete singular solution.
    if (current_node->GetPose().x < end_pose.x + 0.5) {
      continue;
    }

    GetQunticPolynomialPath(path, current_node->GetPose(),
                            current_node->GetRadius(), end_pose);

    if (path.size() < 2) {
      continue;
    }

    // set node by rs path
    if (collision_detect_->IsPolynomialPathSafeByEDT(path, polynomial_node)) {
      valid_path = true;
      break;
    }

    end_pose.x -= 0.1;
  }

  if (!valid_path) {
    *fail_type = PolynomialPathErrorCode::COLLISION;
    return false;
  }

  NodePath node_path;
  node_path.path_dist = std::fabs(path.back().accumulated_s);
  node_path.point_size = 1;

  node_path.points[0].x = path.back().x;
  node_path.points[0].y = path.back().y;
  node_path.points[0].theta = IflyUnifyTheta(path.back().phi, M_PIf32);

  polynomial_node->Set(node_path, *grid_map_bound_, *config_, node_path.path_dist);
  polynomial_node->SetPathType(AstarPathType::QUNTIC_POLYNOMIAL);
  polynomial_node->SetGearType(path.back().gear);
  polynomial_node->SetPre(current_node);
  polynomial_node->SetNext(nullptr);

  if (!polynomial_node->IsNodeValid()) {
    return false;
  }

  return true;
}

bool PolynomialCurveSampling::SamplingByCubicPolyForVerticalSlot(
    HybridAStarResult* result, const Pose2f& start, const Pose2f& end,
    const float lon_min_sampling_length) {
  double astar_start_time = IflyTime::Now_ms();
  ILOG_INFO << "hybrid astar begin, by cubic polynomial";

  // sampling for path end
  // sampling start point: move start point forward dist
  // (lon_min_sampling_length)
  Pose2f sampling_end = start;
  sampling_end.y = 0.0;
  sampling_end.theta = 0.0;
  sampling_end.x = start.x + lon_min_sampling_length;

  float sampling_step = 0.1;
  size_t max_sampling_num = std::ceil((end.x - sampling_end.x) / sampling_step);
  ILOG_INFO << "max_sampling_num = " << max_sampling_num << " "
            << ", lon_min_sampling_length = " << lon_min_sampling_length
            << ", start.x " << start.x << ", end.x " << end.x
            << ", sampling_end.x " << sampling_end.x << ", sampling_step "
            << sampling_step;
  size_t plan_num = 0;
  HybridAStarResult path;
  path.Clear();
  size_t path_points_size = 1000;
  size_t expected_dist_id = 0;
  max_sampling_num = 100;

  for (size_t k = 0; k < max_sampling_num; k++) {
    plan_num = k;
    sampling_end.x += sampling_step;
    std::vector<float> coefficients_vec =
        cubic_polynomial_path_.GeneratePolynomialCoefficients(start,
                                                              sampling_end);
    ILOG_INFO << " coefficients_a : " << coefficients_vec[0]
              << " coefficients_b : " << coefficients_vec[1]
              << " coefficients_c : " << coefficients_vec[2]
              << " coefficients_d : " << coefficients_vec[3];
    std::vector<AStarPathPoint> cubic_path;
    cubic_path.clear();
    cubic_polynomial_path_.GeneratePolynomialPath(
        cubic_path, coefficients_vec, sampling_step * 0.5, start, sampling_end);

    if (cubic_path.empty()) {
      ILOG_INFO << "cubic_path empty";
      continue;
    }

    // check gear
    bool has_reverse = false;
    for (int j = 0; j < cubic_path.size(); j++) {
      if (cubic_path[j].gear == AstarPathGear::REVERSE) {
        // ILOG_INFO << " rs path seg need single shot by drive gear ";
        has_reverse = true;
        break;
      }
    }
    if (has_reverse) {
      ILOG_INFO << "gear is invalid";
      continue;
    }

    // check curvature
    if (cubic_polynomial_path_.GetMinCurvatureRadius() < min_radius_) {
      ILOG_INFO << "curvature is invalid, CurvatureRadius = "
                << cubic_polynomial_path_.GetMinCurvatureRadius();
      continue;
    }

    path.Clear();

    result->Clear();

    for (int i = 0; i < cubic_path.size(); i++) {
      path.x.emplace_back(cubic_path[i].x);
      path.y.emplace_back(cubic_path[i].y);
      path.phi.emplace_back(cubic_path[i].phi);
      path.gear.emplace_back(cubic_path[i].gear);
      path.type.emplace_back(AstarPathType::CUBIC_POLYNOMIAL);
      path.kappa.emplace_back(cubic_path[i].kappa);
    }

    // get path lengh
    path_points_size = path.x.size();

    float accumulated_s = 0.0;
    path.accumulated_s.clear();
    auto last_x = path.x.front();
    auto last_y = path.y.front();
    float x_diff;
    float y_diff;

    for (size_t i = 0; i < path_points_size; ++i) {
      x_diff = path.x[i] - last_x;
      y_diff = path.y[i] - last_y;
      accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
      path.accumulated_s.emplace_back(accumulated_s);

      if (accumulated_s <= lon_min_sampling_length) {
        expected_dist_id = i;
      }

      last_x = path.x[i];
      last_y = path.y[i];
    }

    path_points_size = std::min(path_points_size, expected_dist_id);

    // collision check
    size_t collision_id = collision_detect_->GetPathCollisionIDByEDT(&path);
    if (collision_id > 2) {
      collision_id -= 2;
    }

    path_points_size = std::min(path_points_size, collision_id);
    if (path_points_size <= 1) {
      ILOG_INFO << "collision_id = " << collision_id << ", sampling id = " << k
                << ", max_sampling_num=" << max_sampling_num;
      continue;
    }

    ILOG_INFO << "point size= " << path.x.size()
              << ",expected_dist_id= " << expected_dist_id
              << ", path len= " << path.accumulated_s.back();

    if (path_points_size >= expected_dist_id) {
      break;
    }
  }

  result->base_pose = request_->base_pose;

  if (plan_num > max_sampling_num - 1 || plan_num == max_sampling_num - 1) {
    ILOG_INFO << "cubic plan fail";
    return false;
  }

  path_points_size = std::min(path_points_size, path.x.size());
  float valid_dist = 0.0;
  if (path_points_size > 0) {
    valid_dist = path.accumulated_s[path_points_size];
  }

  if (valid_dist >= 1.0) {
    for (size_t i = 0; i < path_points_size; i++) {
      result->x.emplace_back(path.x[i]);
      result->y.emplace_back(path.y[i]);
      result->phi.emplace_back(path.phi[i]);
      result->gear.emplace_back(path.gear[i]);
      result->type.emplace_back(path.type[i]);
      result->kappa.emplace_back(path.kappa[i]);
      result->accumulated_s.emplace_back(path.accumulated_s[i]);
    }
    result->base_pose = request_->base_pose;

    ILOG_INFO << "path valid, point size= " << result->x.size();
  }

  // DebugRSPath(&rs_path_);

  double astar_end_time = IflyTime::Now_ms();
  ILOG_INFO << "cubic polynomial curve sampling time (ms): "
            << astar_end_time - astar_start_time;

  return true;
}

bool PolynomialCurveSampling::SamplingByCubicPolyForParallelSlot(
    HybridAStarResult* result, const Pose2f& start, const Pose2f& target,
    const float lon_min_sampling_length) {
  double astar_start_time = IflyTime::Now_ms();
  ILOG_INFO << "hybrid astar begin, by cubic polynomial";

  result->Clear();

  // generate sampling bound
  Boundary2D x_sample_bound;
  x_sample_bound.min = start.x + lon_min_sampling_length;
  x_sample_bound.max = start.x + 2.0;

  Boundary2D y_sample_bound;
  y_sample_bound.min = start.y - 0.3;
  y_sample_bound.max = start.y + 0.3;

  float x_sampling_step = 0.1;
  float y_sampling_step = 0.05;
  int x_max_sampling_num =
      std::ceil((x_sample_bound.max - x_sample_bound.min) / x_sampling_step);
  int y_max_sampling_num =
      std::ceil((y_sample_bound.max - y_sample_bound.min) / y_sampling_step);

  ILOG_INFO << "max_sampling_num = " << x_max_sampling_num << " "
            << ", lon_min_sampling_length = " << lon_min_sampling_length;

  size_t path_points_size = 1000;
  std::vector<AStarPathPoint> cubic_path;
  std::vector<AStarPathPoint> best_cubic_path;
  PolynomialPathCost path_cost;
  PolynomialPathCost best_path_cost;
  path_cost.Clear();
  best_path_cost.Clear();

  // sampling for path end
  Pose2f sampling_end;
  PathComparator path_comparator;
  path_comparator.SetHeuristicPose(*request_);

  sampling_end.x = x_sample_bound.min - x_sampling_step;
  for (size_t j = 0; j < x_max_sampling_num; j++) {
    sampling_end.x += x_sampling_step;
    sampling_end.y = y_sample_bound.min - y_sampling_step;

    // 横向遍历
    for (size_t k = 0; k < y_max_sampling_num; k++) {
      sampling_end.y += y_sampling_step;

      // filter some end points
      if (std::fabs(sampling_end.y) > std::fabs(start.y) + 1e-4) {
        continue;
      }

      start.DebugString();
      sampling_end.DebugString();

      std::vector<float> coefficients_vec =
          cubic_polynomial_path_.GeneratePolynomialCoefficients(start,
                                                                sampling_end);

      cubic_polynomial_path_.GeneratePolynomialPath(
          cubic_path, coefficients_vec, 0.05, start, sampling_end);

      if (cubic_path.empty()) {
        ILOG_INFO << "cubic_path empty";
        continue;
      }

      // check curvature
      if (cubic_polynomial_path_.GetMinCurvatureRadius() < min_radius_) {
        ILOG_INFO << "curvature is invalid, Radius = "
                  << cubic_polynomial_path_.GetMinCurvatureRadius();
        continue;
      }

      // get path lengh
      path_points_size = cubic_path.size();

      float accumulated_s = 0.0;
      auto last_x = cubic_path.front().x;
      auto last_y = cubic_path.front().y;
      float x_diff;
      float y_diff;

      for (size_t i = 0; i < path_points_size; ++i) {
        x_diff = cubic_path[i].x - last_x;
        y_diff = cubic_path[i].y - last_y;
        accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
        cubic_path[i].accumulated_s = accumulated_s;

        last_x = cubic_path[i].x;
        last_y = cubic_path[i].y;
      }

      // collision check
      size_t collision_id =
          collision_detect_->GetPathCollisionIDByEDT(cubic_path);
      if (collision_id < 1) {
        continue;
      }

      path_points_size = std::min(path_points_size, collision_id);
      if (path_points_size < 7) {
        ILOG_INFO << "collision_id = " << collision_id;
        continue;
      }

      path_cost.accumulated_s = cubic_path[path_points_size - 1].accumulated_s;
      path_cost.offset_to_center = cubic_path[path_points_size - 1].y;
      path_cost.tail_heading = cubic_path[path_points_size - 1].phi;
      path_cost.point_size = path_points_size;

      if (std::fabs(path_cost.offset_to_center) > std::fabs(start.y) + 1e-4) {
        continue;
      }

      if (path_comparator.PolynomialPathBetter(path_cost, best_path_cost)) {
        best_path_cost = path_cost;
        best_cubic_path = cubic_path;
      }

      ILOG_INFO << "cubic poly point size= " << cubic_path.size()
                << ", path len= " << cubic_path.back().accumulated_s;
    }

    if (std::fabs(best_path_cost.tail_heading) < ifly_deg2rad(1.5) &&
        std::fabs(best_path_cost.offset_to_center) < 0.1) {
      ILOG_INFO << "find best cubic poly";
      break;
    }
  }

  // end
  result->base_pose = request_->base_pose;

  path_points_size = best_path_cost.point_size;
  if (path_points_size > 7) {
    for (size_t i = 0; i < path_points_size; i++) {
      result->x.emplace_back(best_cubic_path[i].x);
      result->y.emplace_back(best_cubic_path[i].y);
      result->phi.emplace_back(best_cubic_path[i].phi);
      result->gear.emplace_back(best_cubic_path[i].gear);
      result->type.emplace_back(best_cubic_path[i].type);
      result->kappa.emplace_back(best_cubic_path[i].kappa);
      result->accumulated_s.emplace_back(best_cubic_path[i].accumulated_s);
    }
    result->base_pose = request_->base_pose;

    ILOG_INFO << "path valid, point size= " << result->x.size();
  }

  // DebugRSPath(&rs_path_);

  double astar_end_time = IflyTime::Now_ms();
  ILOG_INFO << "hybrid astar total time (ms): "
            << astar_end_time - astar_start_time;

  return true;
}

void PolynomialCurveSampling::GetQunticPolynomialPath(
    std::vector<AStarPathPoint>& path, const Pose2f& start,
    const float start_radius, const Pose2f& end) {
  planning_math::QuinticPolynomialCurve1d curve;

  // attention: ref line is slot center line
  // polynomial start
  float y0 = end.y;
  float dy0 = 0.0;
  float ddy0 = 0.0;

  // polynomial end
  float y1 = start.y;
  float dy1;
  if (request_->direction_request == ParkingVehDirection::HEAD_IN) {
    dy1 = std::tan(start.theta + M_PI);
  } else {
    dy1 = std::tan(start.theta);
  }
  float ddy1;
  if (std::fabs(start_radius > 1000.0)) {
    ddy1 = 0.0;
  } else {
    ddy1 = std::pow((1.0 + dy1 * dy1), 1.5) / start_radius;
  }

  // ref s
  float total_ref_s = start.x - end.x;
  curve.SetParam(y0, dy0, ddy0, y1, dy1, ddy1, total_ref_s);

  int point_num = std::ceil(total_ref_s / 0.05);
  float ref_s = 0;
  float kappa;
  float max_kappa = 1.0 / min_radius_ + 1e-5;

  // interpolate
  AStarPathPoint point;
  ref_s = total_ref_s;
  float accumulated_s = 0;
  float dist = 0;
  float theta;

  for (int i = 0; i <= point_num; i++) {
    const float y = curve.Evaluate(0, ref_s);
    const float dy = curve.Evaluate(1, ref_s);
    theta = std::atan(dy);
    kappa = curve.EvaluateKappa(ref_s);

    if (kappa > max_kappa || kappa < -max_kappa) {
      path.clear();

      // ILOG_INFO << "i=" << i << ",radius bound =" << min_radius
      //           << ",radius=" << 1 / kappa << ",accumulated_s=" <<
      //           accumulated_s
      //           << ",x =" << end.x + ref_s << ",y=" << x << ",phi=" << theta;
      // start.DebugString();
      return;
    }

    point.x = end.x + ref_s;
    point.y = y;
    if (request_->direction_request == ParkingVehDirection::TAIL_IN) {
      point.phi = theta;
      point.gear = AstarPathGear::REVERSE;
    } else {
      point.phi = IflyUnifyTheta(theta + M_PIf32, M_PIf32);
      point.gear = AstarPathGear::DRIVE;
    }

    point.type = AstarPathType::QUNTIC_POLYNOMIAL;
    point.kappa = kappa;

    if (path.size() < 1) {
      accumulated_s = 0.0;
    } else {
      const AStarPathPoint& last_point = path[path.size() - 1];
      dist = std::sqrt((point.x - last_point.x) * (point.x - last_point.x) +
                       (point.y - last_point.y) * (point.y - last_point.y));

      // same point
      if (dist < 0.0001) {
        break;
      }

      accumulated_s += dist;
    }

    point.accumulated_s = accumulated_s;
    path.push_back(point);

    ref_s -= 0.05;
    ref_s = std::max(0.0f, ref_s);
  }

  return;
}

}  // namespace planning