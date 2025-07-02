#include "rs_sampling.h"
#include "hybrid_astar_common.h"
#include "ifly_time.h"
#include "log_glog.h"

namespace planning {

RSSampling::RSSampling(const MapBound* XYbounds,
                       const ParkObstacleList* obstacles,
                       const AstarRequest* request, EulerDistanceTransform* edt,
                       const ObstacleClearZone* clear_zone,
                       ParkReferenceLine* ref_line,
                       const PlannerOpenSpaceConfig* config,
                       const float min_radius,
                       std::shared_ptr<NodeCollisionDetect> collision_detect)
    : CurveSampling(XYbounds, obstacles, request, edt, clear_zone, ref_line,
                    config, min_radius, collision_detect) {}

const bool RSSampling::IsExpectedGearForRsPath(const RSPath& path) {
  for (int j = 0; j < path.size; j++) {
    if (path.paths[j].gear != request_->first_action_request.gear_request) {
      // ILOG_INFO << " rs path seg need single shot by drive gear ";
      return false;
    }
  }

  return true;
}

void RSSampling::PathTransformByRSPath(const RSPath& rs_path,
                                       HybridAStarResult* result) {
  result->Clear();

  const RSPathSegment* last_segment = nullptr;
  for (int i = 0; i < rs_path.size; i++) {
    const RSPathSegment* segment = &rs_path.paths[i];

    // get rs segment first point
    int point_id = 0;
    // delete first same point
    if (last_segment != nullptr) {
      if (segment->gear == last_segment->gear) {
        point_id = 1;
      }
    }

    for (; point_id < segment->size; point_id++) {
      result->x.emplace_back(segment->points[point_id].x);
      result->y.emplace_back(segment->points[point_id].y);
      result->phi.emplace_back(segment->points[point_id].theta);
      result->gear.emplace_back(segment->gear);
      result->type.emplace_back(AstarPathType::REEDS_SHEPP);
      result->kappa.emplace_back(segment->points[point_id].kappa);
    }

    last_segment = segment;
  }

  if (result->x.size() < 1) {
    return;
  }

  // get path lengh
  float accumulated_s = 0.0;
  auto last_x = result->x.front();
  auto last_y = result->y.front();
  float x_diff;
  float y_diff;

  for (size_t i = 0; i < result->x.size(); ++i) {
    x_diff = result->x[i] - last_x;
    y_diff = result->y[i] - last_y;
    accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
    result->accumulated_s.emplace_back(accumulated_s);

    last_x = result->x[i];
    last_y = result->y[i];
  }

  return;
}

void RSSampling::RSPathCandidateByRadius(HybridAStarResult* result,
                                         const Pose2f& start, const Pose2f& end,
                                         const float lon_min_sampling_length,
                                         const float radius) {
  bool is_connected_to_goal;
  RSPathRequestType rs_request = RSPathRequestType::NONE;

  // sampling for path end
  // sampling start point: move start point forward dist
  // (lon_min_sampling_length)
  Pose2f sampling_end = start;
  sampling_end.y = 0.0;
  sampling_end.theta = end.theta;
  sampling_end.x = start.x + lon_min_sampling_length;

  float sampling_step = 0.2;
  float sampling_range = end.x - sampling_end.x;
  size_t max_sampling_num = 0;
  if (sampling_range > 0.0) {
    max_sampling_num = std::ceil(sampling_range / sampling_step);
  }
  float sampling_radius = radius;

  result->Clear();
  HybridAStarResult path;
  path.Clear();
  HybridAStarResult best_path;
  best_path.Clear();

  size_t best_path_valid_point_size = 0;
  size_t path_valid_point_size = 1000;
  size_t expected_dist_id = 0;

  for (size_t k = 0; k < max_sampling_num; k++) {
    sampling_end.x += sampling_step;
    rs_path_interface_.GeneShortestRSPath(
        &rs_path_, &is_connected_to_goal, &start, &sampling_end,
        sampling_radius, true, false, rs_request);

    if (rs_path_.total_length < 0.01 || !is_connected_to_goal) {
      ILOG_INFO << "rs path fail";
      continue;
    }

    // check gear
    if (!IsExpectedGearForRsPath(rs_path_)) {
      // ILOG_INFO << "gear is invalid";
      continue;
    }

    PathTransformByRSPath(rs_path_, &path);

    // collision check
    size_t collision_id = collision_detect_->GetPathCollisionIDByEDT(&path);
    // add extra lon buffer
    if (collision_id > 2) {
      collision_id -= 2;
    }
    path_valid_point_size = std::min(path.x.size(), collision_id);

#if 0
    ILOG_INFO << "point size= " << path.x.size() << ", path len= "
              << ((path.accumulated_s.size() > 0) ? path.accumulated_s.back()
                                                  : 0.0)
              << ", path_valid_point_size=" << path_valid_point_size
              << ",collision_id = " << collision_id;
#endif

    // find longer path
    if (best_path_valid_point_size < path_valid_point_size) {
      best_path_valid_point_size = path_valid_point_size;
      best_path = path;
    }

    // check size
    if (best_path_valid_point_size <= 1) {
      // ILOG_INFO << "collision_id = " << collision_id << ",sampling id = " <<
      // k;
      continue;
    }

    // find long enough path
    if (best_path_valid_point_size >= path.accumulated_s.size() &&
        path.accumulated_s[best_path_valid_point_size - 1] >
            lon_min_sampling_length) {
      break;
    }
  }

  best_path_valid_point_size =
      std::min(best_path_valid_point_size, best_path.x.size());
  float valid_dist = 0.0;
  if (best_path_valid_point_size > 0) {
    valid_dist = best_path.accumulated_s.back();
  }

  if (valid_dist >= 1.0) {
    for (size_t i = 0; i < best_path_valid_point_size; i++) {
      result->x.emplace_back(best_path.x[i]);
      result->y.emplace_back(best_path.y[i]);
      result->phi.emplace_back(best_path.phi[i]);
      result->gear.emplace_back(best_path.gear[i]);
      result->type.emplace_back(best_path.type[i]);
      result->kappa.emplace_back(best_path.kappa[i]);
      result->accumulated_s.emplace_back(best_path.accumulated_s[i]);

      // no need too long path
      if (best_path.accumulated_s[i] > lon_min_sampling_length) {
        break;
      }
    }
    result->base_pose = request_->base_pose_;

    ILOG_INFO << "path valid, point size= " << result->x.size();
  }

  return;
}

bool RSSampling::PlanByRSPathSampling(HybridAStarResult* result,
                                      const Pose2f& start, const Pose2f& end,
                                      const float lon_min_sampling_length) {
  double astar_start_time = IflyTime::Now_ms();
  ILOG_INFO << "hybrid astar begin, generate path by rs.";

  rs_path_.Clear();

  // todo, use spiral/rs/polynomial to generate candidate path.
  result->Clear();
  std::vector<HybridAStarResult> candidates;

  float max_radius = 85.0;
  float min_radius = 8.0;
  float radius_step = 5.0;

  int sampline_numer = std::ceil((max_radius - min_radius) / radius_step);
  HybridAStarResult path;

  float radius = max_radius;
  for (int i = 0; i < sampline_numer; i++) {
    radius -= radius_step;
    radius = std::max(radius, min_radius);

    RSPathCandidateByRadius(&path, start, end, lon_min_sampling_length, radius);

    // find best path
    if (path.accumulated_s.size() > 1) {
      if (path.accumulated_s.back() > lon_min_sampling_length - 0.2) {
        candidates.clear();
        candidates.push_back(path);
        break;
      }
    }

    if (path.x.size() > 1) {
      candidates.push_back(path);
    }

    if (radius < min_radius + 1e-2) {
      break;
    }
  }

  // compare path candidates
  if (candidates.size() > 0) {
    size_t best_id = 0;
    float best_length = 0.0;
    for (size_t i = 0; i < candidates.size(); i++) {
      HybridAStarResult& tmp_path = candidates[i];
      if (tmp_path.accumulated_s.size() > 1 &&
          tmp_path.accumulated_s.back() > best_length) {
        best_id = i;
        best_length = tmp_path.accumulated_s.back();
      }
    }

    if (best_length > 0.0) {
      *result = candidates[best_id];
    }
  }

  double astar_end_time = IflyTime::Now_ms();
  result->time_ms = astar_end_time - astar_start_time;
  ILOG_INFO << "rs sampling total time (ms) = " << result->time_ms;

  return true;
}

bool RSSampling::SamplingByRSPath(Node3d* current_node,
                                  Node3d* rs_node_to_goal) {
  if (current_node->GetY() > 0.8 || current_node->GetY() < -0.8) {
    return false;
  }

  float x_diff = current_node->GetX() - request_->real_goal.GetX();
  if (x_diff < 0.2) {
    return false;
  }

  if (std::fabs(current_node->GetRadius()) < min_radius_ - 1e-5) {
    return false;
  }

  float heading_diff = IflyUnifyTheta(current_node->GetPhi(), M_PIf32) -
                       IflyUnifyTheta(search_goal_.GetPhi(), M_PIf32);
  heading_diff = IflyUnifyTheta(heading_diff, M_PIf32);
  if (heading_diff > ifly_deg2rad(60.0) || heading_diff < ifly_deg2rad(-60.0)) {
    return false;
  }

  // init
  rs_node_to_goal->Clear();
  float min_straight_dist = 0.7;
  if (request_->direction_request == ParkingVehDirection::TAIL_IN) {
    min_straight_dist = 0.7;
  } else if (request_->direction_request == ParkingVehDirection::HEAD_IN) {
    min_straight_dist = 0.3;
  }

  float sample_range =
      search_goal_.GetX() - (request_->real_goal.GetX() + min_straight_dist);
  int sampline_numer = std::ceil(sample_range / 0.1);
  sampline_numer = std::max(1, sampline_numer);
  bool valid_path = false;
  bool is_connected_to_goal = false;

  Pose2f end_pose = search_goal_;
  end_pose.x += 0.1;

  for (int q = 0; q < sampline_numer; q++) {
    end_pose.x -= 0.1;
    rs_path_interface_.GeneShortestRSPath(
        &rs_path_, &is_connected_to_goal, &current_node->GetPose(), &end_pose,
        min_radius_, false, false, RSPathRequestType::NONE);

    // check length
    if (rs_path_.total_length < 0.01 || !is_connected_to_goal) {
      continue;
    }

    // check gear
    if (!IsExpectedGearForRsPath(rs_path_)) {
      continue;
    }

    // interpolate
    rs_path_interface_.RSPathInterpolate(&rs_path_, &current_node->GetPose(),
                                         min_radius_);

    // check safe
    if (collision_detect_->IsRSPathSafeByEDT(&rs_path_, rs_node_to_goal)) {
      valid_path = true;
      break;
    }
  }

  if (!valid_path) {
    return false;
  }

  NodePath node_path;
  node_path.path_dist = std::fabs(rs_path_.total_length);
  node_path.point_size = 1;
  // use first path end point to fill astar node
  RSPoint rs_end_point;
  rs_path_.FirstPathEndPoint(&rs_end_point);

  node_path.points[0].x = rs_end_point.x;
  node_path.points[0].y = rs_end_point.y;
  node_path.points[0].theta = IflyUnifyTheta(rs_end_point.theta, M_PIf32);

  rs_node_to_goal->Set(node_path, *XYbounds_, *config_, node_path.path_dist);

  rs_node_to_goal->SetPathType(AstarPathType::REEDS_SHEPP);
  rs_node_to_goal->SetGearType(request_->direction_request ==
                                       ParkingVehDirection::TAIL_IN
                                   ? AstarPathGear::REVERSE
                                   : AstarPathGear::DRIVE);
  rs_node_to_goal->SetPre(current_node);
  rs_node_to_goal->SetNext(nullptr);

  if (!rs_node_to_goal->IsNodeValid()) {
    return false;
  }

  return true;
}
}  // namespace planning