#include "spiral_sampling.h"

namespace planning {

SpiralSampling::SpiralSampling(
    const MapBound* XYbounds, const ParkObstacleList* obstacles,
    const AstarRequest* request, EulerDistanceTransform* edt,
    const ObstacleClearZone* clear_zone, ParkReferenceLine* ref_line,
    const PlannerOpenSpaceConfig* config, const float min_radius,
    std::shared_ptr<NodeCollisionDetect> collision_detect)
    : CurveSampling(XYbounds, obstacles, request, edt, clear_zone, ref_line,
                    config, min_radius, collision_detect) {}

/**
 * This function is a general function for generating a cubic spiral.Originally,
 *it was intended to generate a cubic spiral in Analytic Expansion to connect
 *the endpoints. It is not needed for the time being.
 *
 **/
const bool SpiralSampling::GetCubicSpiralPath(std::vector<AStarPathPoint>& path,
                                              const Pose2D& start,
                                              const Pose2D& end,
                                              const AstarPathGear ref_gear) {
  // ILOG_INFO << "---- generate cubic spiral path ----";

  CubicSpiralInterface cubic_spiral_interface;
  spiral_path_point_t start_spiral;
  start_spiral.x = start.x;
  start_spiral.y = start.y;
  start_spiral.theta = start.theta;
  start_spiral.kappa = 0.0;

  spiral_path_point_t goal_spiral;
  goal_spiral.x = end.x;
  goal_spiral.y = end.y;
  goal_spiral.theta = end.theta;
  goal_spiral.kappa = 0.0;

  // *
  // Reverse gear requires changing the heading of the path point
  if (ref_gear == AstarPathGear::REVERSE) {
    start_spiral.theta = IflyUnifyTheta(start.theta + M_PI, M_PI);
    goal_spiral.theta = IflyUnifyTheta(goal_spiral.theta + M_PI, M_PI);
  }

  // ILOG_INFO << "start_spiral ( " << start_spiral.x << ", " << start_spiral.y
  //           << ", " << start_spiral.theta << " ) ";
  // ILOG_INFO << "goal_spiral ( " << goal_spiral.x << ", " << goal_spiral.y
  //           << ", " << goal_spiral.theta << " ) ";
  bool constrain_start_k = true;
  bool constrain_goal_k = true;
  const float spiral_step_length = 0.1;
  std::vector<AStarPathPoint> cubic_spiral_path;
  cubic_spiral_path.reserve(MAX_SPIRAL_PATH_POINT_NUM);

  std::vector<spiral_path_point_t> states;
  states.reserve(MAX_SPIRAL_PATH_POINT_NUM);

  solution_cubic_t solution;

  bool success = cubic_spiral_interface.GenerateCubicSpiralPathByStrictSolve(
      &solution, states, &start_spiral, &goal_spiral, spiral_step_length);

  if (!success) {
    // ILOG_INFO << "generate cubic spiral path failed ";
    return false;
  }

  AStarPathPoint point;
  float accumulated_s = 0.0;
  path.clear();

  for (auto& state : states) {
    point.x = state.x;
    point.y = state.y;
    point.phi =
        (ref_gear == AstarPathGear::DRIVE) ? state.theta : state.theta - M_PI;
    point.gear = ref_gear;
    point.type = AstarPathType::SPIRAL;
    point.kappa = state.kappa;
    point.accumulated_s = accumulated_s;
    path.emplace_back(point);
    accumulated_s += spiral_step_length;

    // ILOG_INFO << "spiral s: " << point.accumulated_s << ", ( " << point.x
    //           << ", " << point.y << ", " << point.phi << ", "
    //           << static_cast<int>(point.gear) << " ) ";
  }
  // ILOG_INFO << "expansion cubic spiral path size " << path.size();

  return true;
}

bool SpiralSampling::SamplingByCubicSpiralForVerticalSlot(
    HybridAStarResult* result, const Pose2D& start, const Pose2D& end,
    const float lon_min_sampling_length) {
  // double astar_start_time = IflyTime::Now_ms();
  ILOG_INFO << "hybrid astar begin, by cubic spiral";

  // init
  result->Clear();

  const AstarPathGear spiral_gear =
      request_->direction_request == ParkingVehDirection::TAIL_IN
          ? AstarPathGear::DRIVE
          : AstarPathGear::REVERSE;

  // sampling for path end
  // sampling start point: move start point forward dist
  // (lon_min_sampling_length)
  Pose2D sampling_end = start;
  sampling_end.y = 0.0;
  sampling_end.theta = end.theta;
  sampling_end.x = start.x + lon_min_sampling_length;

  const float sampling_step = 0.1;
  size_t max_sampling_num = 50;

  // ILOG_INFO << "max_sampling_num = " << max_sampling_num << " "
  //           << ", lon_min_sampling_length = " << lon_min_sampling_length
  //           << ", start.x " << start.x << ", start.y " << start.y
  //           << ", start.theta " << start.theta << ", end.x " << end.x
  //           << ", end.y " << end.y << ", end.theta " << end.theta
  //           << ", sampling_step " << sampling_step;

  HybridAStarResult path;
  path.Clear();
  size_t path_points_size = 1000;
  size_t expected_dist_id = 0;

  std::vector<AStarPathPoint> cubic_spiral_path;
  cubic_spiral_path.reserve(MAX_SPIRAL_PATH_POINT_NUM);

  for (size_t k = 0; k < max_sampling_num; k++) {
    cubic_spiral_path.clear();

    if (!GetCubicSpiralPath(cubic_spiral_path, start, sampling_end,
                            spiral_gear)) {
      sampling_end.x += 0.1;
      continue;
    }

    std::vector<spiral_path_point_t> states;
    states.reserve(MAX_SPIRAL_PATH_POINT_NUM);

    auto last_state = cubic_spiral_path.back();
    // ILOG_INFO << "last_state.x " << last_state.x << ", sampling_end.x "
    //           << sampling_end.x << ", last_state.y " << last_state.y
    //           << ",sampling_end.y " << sampling_end.y << ",last_state.theta"
    //           << last_state.phi << ", sampling_end.theta "
    //           << sampling_end.theta;
    float end_point_error_x = last_state.x - sampling_end.x;
    float end_point_error_y = last_state.y - sampling_end.y;
    float end_point_error_theta = last_state.phi - sampling_end.theta;
    sampling_end.x += 0.1;

    if (std::fabs(end_point_error_x) >= 1e-2 ||
        std::fabs(end_point_error_y) >= 1e-2) {
      ILOG_INFO << "spiral path end point insufficient accuracy";
      continue;
    }

    path.Clear();

    for (int i = 0; i < cubic_spiral_path.size(); i++) {
      path.x.emplace_back(cubic_spiral_path[i].x);
      path.y.emplace_back(cubic_spiral_path[i].y);
      path.phi.emplace_back(cubic_spiral_path[i].phi);
      path.gear.emplace_back(cubic_spiral_path[i].gear);
      path.type.emplace_back(AstarPathType::SPIRAL);
      path.kappa.emplace_back(cubic_spiral_path[i].kappa);
      path.accumulated_s.emplace_back(cubic_spiral_path[i].accumulated_s);
    }

    // get path lengh
    path_points_size = path.x.size();

    for (size_t i = 0; i < path_points_size; ++i) {
      if (path.accumulated_s[i] <= lon_min_sampling_length) {
        expected_dist_id = i;
      }
    }

    // collision check
    size_t collision_id = collision_detect_->GetPathCollisionIDByEDT(&path);
    if (collision_id > 2) {
      collision_id -= 2;
    }

    path_points_size = std::min(path_points_size, collision_id);
    if (path_points_size <= 1) {
      continue;
    }

    // ILOG_INFO << " point size = " << path.x.size()
    //           << ", expected_dist_id = " << expected_dist_id
    //           << ", path_points_size = " << path_points_size
    //           << ", path len= " << path.accumulated_s.back()
    //           << ", collision_id is " << collision_id;

    if (path_points_size >= expected_dist_id) {
      break;
    }
  }

  result->base_pose = request_->base_pose_;
  path_points_size = std::min(path_points_size, path.x.size());
  if (path_points_size < 2) {
    return false;
  }

  float valid_dist = 0.0;
  if (path_points_size > 0) {
    valid_dist = path.accumulated_s[path_points_size - 1];
  }
  if (valid_dist >= 1.0) {
    ILOG_INFO << "path_points_size = " << path_points_size;
    for (size_t i = 0; i < path_points_size; i++) {
      result->x.emplace_back(path.x[i]);
      result->y.emplace_back(path.y[i]);
      result->phi.emplace_back(path.phi[i]);
      result->gear.emplace_back(path.gear[i]);
      result->type.emplace_back(path.type[i]);
      result->kappa.emplace_back(path.kappa[i]);
      result->accumulated_s.emplace_back(path.accumulated_s[i]);

      // no need too long path
      if (path.accumulated_s[i] > lon_min_sampling_length) {
        break;
      }
    }
    result->base_pose = request_->base_pose_;

    if (result->accumulated_s.size() > 0 &&
        result->accumulated_s.back() < lon_min_sampling_length - 0.1) {
      return false;
    }

    ILOG_INFO << "path valid, point size= " << result->x.size();
  } else {
    ILOG_INFO << "cubic spiral path invalid";
    return false;
  }

  // for (int i = 0; i < result->x.size(); i++) {
  //   ILOG_INFO << "[ " << i << " ] " << static_cast<int>(result->gear[i])
  //             << ", ( " << result->x[i] << ", " << result->y[i] << ", "
  //             << result->phi[i] << " )";
  // }

  // double astar_end_time = IflyTime::Now_ms();
  // ILOG_INFO << "spiral path time (ms): " << astar_end_time -
  // astar_start_time;

  return true;
}

}  // namespace planning