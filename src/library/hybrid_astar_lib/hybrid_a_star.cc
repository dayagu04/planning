#include "hybrid_a_star.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>

#include "aabb2d.h"
#include "ad_common/math/math_utils.h"
#include "collision_detect_types.h"
#include "footprint_circle_model.h"
#include "h_cost.h"
#include "hybrid_astar_common.h"
#include "log_glog.h"
#include "math_utils.h"
#include "node3d.h"
#include "path_comparator.h"
#include "pose2d.h"
#include "reeds_shepp.h"
#include "rs_path_interpolate.h"
#include "src/common/ifly_time.h"
#include "src/library/spiral/cubic_spiral_interface.h"
#include "src/library/spiral/spiral_path.h"
#include "src/modules/common/math/curve1d/quintic_polynomial_curve1d.h"
#include "transform2d.h"
#include "utils_math.h"

namespace planning {

#define PLOT_RS_COST_PATH (0)
#define PLOT_RS_EXNTEND_TO_END (0)
#define PLOT_CHILD_NODE (0)
#define PLOT_SEARCH_SEQUENCE (0)
#define PLOT_DELETE_NODE (0)
#define RS_H_COST_MAX_NUM (32)

#define DEBUG_SEARCH_RESULT (0)
#define DEBUG_CHILD_NODE (0)
#define DEBUG_REF_LINE_COST (0)
#define DEBUG_EDT (0)

#define DEBUG_NODE_MAX_NUM (10000)
#define DEBUG_NODE_GEAR_SWITCH_NUMBER (0)

#define LOG_TIME_PROFILE (0)
#define DEBUG_GJK (0)

#define DEBUG_ONE_SHOT_PATH (0)
#define DEBUG_ONE_SHOT_PATH_MAX_NODE (10000)
#define ENABLE_OBS_DIST_G_COST (0)

HybridAStar::HybridAStar(const PlannerOpenSpaceConfig& open_space_conf,
                         const VehicleParam& veh_param) {
  config_ = open_space_conf;

  vehicle_param_ = veh_param;
}

int HybridAStar::UpdateConfig(const PlannerOpenSpaceConfig& open_space_conf) {
  config_ = open_space_conf;
  return 0;
}

bool HybridAStar::CalcRSPathToGoal(Node3d* current_node,
                                   const bool need_rs_dense_point,
                                   const bool need_anchor_point,
                                   const RSPathRequestType rs_request,
                                   const float rs_radius) {
#if LOG_TIME_PROFILE
  double rs_start_time = IflyTime::Now_ms();
#endif

  const Pose2D& start_pose = current_node->GetPose();
  const Pose2D& end_pose = rs_expansion_decider_.GetRSEndPose();

  bool is_connected_to_goal;
  rs_path_interface_.GeneShortestRSPath(
      &rs_path_, &is_connected_to_goal, &start_pose, &end_pose, rs_radius,
      need_rs_dense_point, need_anchor_point, rs_request);

#if LOG_TIME_PROFILE
  double rs_end_time = IflyTime::Now_ms();
  rs_time_ms_ += rs_end_time - rs_start_time;
#endif

  if (rs_path_.total_length < 0.01 || !is_connected_to_goal) {
    ILOG_INFO << " path is short";
    return false;
  }

  return true;
}

bool HybridAStar::PlanByRSPathSampling(
    HybridAStarResult* result, const Pose2D& start, const Pose2D& end,
    const float lon_min_sampling_length, const MapBound& XYbounds,
    const ParkObstacleList& obstacles, const AstarRequest& request,
    EulerDistanceTransform* edt, const ObstacleClearZone* clear_zone,
    ParkReferenceLine* ref_line) {
  double astar_start_time = IflyTime::Now_ms();
  ILOG_INFO << "hybrid astar begin, generate path by rs.";

  // init
  obstacles_ = &obstacles;
  edt_ = edt;

  // load XYbounds
  XYbounds_ = XYbounds;

  request_ = request;
  DebugAstarRequestString(request_);
  rs_path_.Clear();

  clear_zone_ = clear_zone;
  ref_line_ = ref_line;

  // todo, use spiral/rs/polynomial to generate candidate path.
  result->Clear();
  rs_path_h_cost_debug_.clear();
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

bool HybridAStar::SamplingByCubicPolyForVerticalSlot(
    HybridAStarResult* result, const Pose2D& start, const Pose2D& end,
    const float lon_min_sampling_length, const MapBound& XYbounds,
    const ParkObstacleList& obstacles, const AstarRequest& request,
    EulerDistanceTransform* edt, const ObstacleClearZone* clear_zone,
    ParkReferenceLine* ref_line) {
  double astar_start_time = IflyTime::Now_ms();
  ILOG_INFO << "hybrid astar begin, by cubic polynomial";

  // init
  obstacles_ = &obstacles;
  edt_ = edt;
  ref_line_ = ref_line;

  // load XYbounds
  XYbounds_ = XYbounds;

  request_ = request;
  DebugAstarRequestString(request_);

  clear_zone_ = clear_zone;

  // sampling for path end
  // sampling start point: move start point forward dist
  // (lon_min_sampling_length)
  Pose2D sampling_end = start;
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
        cubic_path_interface_.GeneratePolynomialCoefficients(start,
                                                             sampling_end);
    ILOG_INFO << " coefficients_a : " << coefficients_vec[0]
              << " coefficients_b : " << coefficients_vec[1]
              << " coefficients_c : " << coefficients_vec[2]
              << " coefficients_d : " << coefficients_vec[3];
    std::vector<AStarPathPoint> cubic_path;
    cubic_path.clear();
    cubic_path_interface_.GeneratePolynomialPath(
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
    if (cubic_path_interface_.GetMinCurvatureRadius() <
        vehicle_param_.min_turn_radius) {
      ILOG_INFO << "curvature is invalid, CurvatureRadius = "
                << cubic_path_interface_.GetMinCurvatureRadius();
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
    size_t collision_id = GetPathCollisionIDByEDT(&path);
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

  result->base_pose = request.base_pose_;

  if (plan_num > max_sampling_num - 1 || plan_num == max_sampling_num - 1) {
    ILOG_INFO << "cubic plan fail";
    return false;
  }

  path_points_size = std::min(path_points_size, path.x.size());
  float valid_dist = 0.0;
  if (path_points_size > 0) {
    valid_dist = path.accumulated_s[path_points_size];
  }
  if (valid_dist >= 1.2) {
    for (size_t i = 0; i < path_points_size; i++) {
      result->x.emplace_back(path.x[i]);
      result->y.emplace_back(path.y[i]);
      result->phi.emplace_back(path.phi[i]);
      result->gear.emplace_back(path.gear[i]);
      result->type.emplace_back(path.type[i]);
      result->kappa.emplace_back(path.kappa[i]);
      result->accumulated_s.emplace_back(path.accumulated_s[i]);
    }
    result->base_pose = request.base_pose_;

    ILOG_INFO << "path valid, point size= " << result->x.size();
  } else if (valid_dist > 0.5) {
    // if path is too short by collision check or gear check, use a fallback
    // path with no collision check.
    fallback_path_.Clear();
    for (size_t i = 0; i < path_points_size; i++) {
      fallback_path_.x.emplace_back(path.x[i]);
      fallback_path_.y.emplace_back(path.y[i]);
      fallback_path_.phi.emplace_back(path.phi[i]);
      fallback_path_.gear.emplace_back(path.gear[i]);
      fallback_path_.type.emplace_back(path.type[i]);
      fallback_path_.kappa.emplace_back(path.kappa[i]);
      fallback_path_.accumulated_s.emplace_back(path.accumulated_s[i]);
    }
    fallback_path_.base_pose = request.base_pose_;
    ILOG_INFO << "path invalid, point size= " << path.x.size();
  }

  // DebugRSPath(&rs_path_);

  double astar_end_time = IflyTime::Now_ms();
  ILOG_INFO << "cubic polynomial curve sampling time (ms): "
            << astar_end_time - astar_start_time;

  return true;
}

bool HybridAStar::SamplingByCubicSpiralForVerticalSlot(
    HybridAStarResult* result, const Pose2D& start, const Pose2D& end,
    const float lon_min_sampling_length, const MapBound& XYbounds,
    const ParkObstacleList& obstacles, const AstarRequest& request,
    EulerDistanceTransform* edt, const ObstacleClearZone* clear_zone,
    ParkReferenceLine* ref_line) {
  // double astar_start_time = IflyTime::Now_ms();
  ILOG_INFO << "hybrid astar begin, by cubic spiral";

  // init
  result->Clear();
  obstacles_ = &obstacles;
  edt_ = edt;
  ref_line_ = ref_line;

  // load XYbounds
  XYbounds_ = XYbounds;

  request_ = request;
  DebugAstarRequestString(request_);
  child_node_debug_.clear();
  clear_zone_ = clear_zone;

  const AstarPathGear spiral_gear =
      request_.direction_request == ParkingVehDirection::TAIL_IN
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

#if PLOT_CHILD_NODE
    child_node_debug_.emplace_back(
        DebugAstarSearchPoint(sampling_end.x, sampling_end.y, true));
#endif

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
    size_t collision_id = GetPathCollisionIDByEDT(&path);
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

  result->base_pose = request.base_pose_;
  path_points_size = std::min(path_points_size, path.x.size());
  if (path_points_size < 2) {
    return false;
  }

  float valid_dist = 0.0;
  if (path_points_size > 0) {
    valid_dist = path.accumulated_s[path_points_size - 1];
  }
  if (valid_dist >= 1.2) {
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
    result->base_pose = request.base_pose_;

    if (result->accumulated_s.size() > 0 &&
        result->accumulated_s.back() < lon_min_sampling_length - 0.1) {
      return false;
    }

    ILOG_INFO << "path valid, point size= " << result->x.size();
  } else if (valid_dist > 0.5) {
    // if path is too short by collision check or gear check, use a fallback
    // path with no collision check.
    fallback_path_.Clear();
    for (size_t i = 0; i < path_points_size; i++) {
      fallback_path_.x.emplace_back(path.x[i]);
      fallback_path_.y.emplace_back(path.y[i]);
      fallback_path_.phi.emplace_back(path.phi[i]);
      fallback_path_.gear.emplace_back(path.gear[i]);
      fallback_path_.type.emplace_back(path.type[i]);
      fallback_path_.kappa.emplace_back(path.kappa[i]);
      fallback_path_.accumulated_s.emplace_back(path.accumulated_s[i]);
    }
    fallback_path_.base_pose = request.base_pose_;
    ILOG_INFO << "cubic spiral path invalid, point size = " << path.x.size();
    return false;
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

bool HybridAStar::AnalyticExpansionByRS(Node3d* current_node,
                                        const PathGearRequest gear_request_info,
                                        Node3d* rs_node_to_goal) {
  // check gear and steering wheel
  if (!rs_expansion_decider_.IsNeedRsExpansion(current_node, &request_)) {
    // ILOG_INFO << "no need rs path link";
    return false;
  }

  const float rs_radius = vehicle_param_.min_turn_radius + 0.2;

  RSPathRequestType rs_request = RSPathRequestType::GEAR_SWITCH_LESS_THAN_TWICE;
  bool need_anchor_point = false;
  if (request_.direction_request == ParkingVehDirection::HEAD_IN) {
    need_anchor_point = true;
  }
  if (!CalcRSPathToGoal(current_node, false, need_anchor_point, rs_request,
                        rs_radius)) {
    ILOG_INFO << " generate rs fail";

    return false;
  }

  // set node by rs path
  if (rs_path_.size < 1) {
    return false;
  }

  // todo: move to rs link decision
  // request check
  if (request_.first_action_request.has_request) {
    // start node rs gear is not expectation
    if (current_node->IsStartNode()) {
      if (IsGearDifferent(request_.first_action_request.gear_request,
                          rs_path_.paths[0].gear)) {
        rs_node_to_goal->ClearPath();

        // ILOG_INFO << "gear is not expectation";
        return false;
      }
    }

    // search node rs gear is not expectation
    if (current_node->GetGearType() ==
            request_.first_action_request.gear_request &&
        current_node->GetDistToStart() <
            request_.first_action_request.dist_request) {
      if (IsGearDifferent(request_.first_action_request.gear_request,
                          rs_path_.paths[0].gear)) {
        rs_node_to_goal->ClearPath();

        // ILOG_INFO << "gear is not expectation";
        return false;
      } else {
        if ((current_node->GetDistToStart() + rs_path_.GetFirstGearLength()) <
            request_.first_action_request.dist_request) {
          // ILOG_INFO << "dist is not expectation";
          return false;
        }
      }
    }

    // search node gear is not expectation
    if (current_node->GetGearSwitchNum() == 0 &&
        current_node->IsPathGearChange(
            request_.first_action_request.gear_request)) {
      rs_node_to_goal->ClearPath();

      // ILOG_INFO << "gear is not expectation";
      return false;
    }
  }

  // todo, need to get backward pass dist in all nodes.
  float parent_node_path_dist = current_node->GetNodePathDistance();
  if (current_node->GetGearType() != rs_path_.paths[0].gear) {
    parent_node_path_dist = 0;
  }

  // length check
  if (!IsRsPathFirstSegmentLongEnough(&rs_path_, parent_node_path_dist)) {
    // ILOG_INFO << "length is not expectation";
    return false;
  }

  // last segment gear check
  if (!RsLastSegmentSatisfyRequest(&rs_path_)) {
    // ILOG_INFO << "gear is not expectation";
    return false;
  }

  // gear check
  if (!CheckRSPathGear(&rs_path_, gear_request_info)) {
    // ILOG_INFO << "gear is not expectation";
    return false;
  }

  // interpolation
#if LOG_TIME_PROFILE
  double rs_start_time = IflyTime::Now_ms();
#endif

  const Pose2D& start_pose = current_node->GetPose();
  rs_path_interface_.RSPathInterpolate(&rs_path_, &start_pose, rs_radius);

#if LOG_TIME_PROFILE
  double rs_end_time = IflyTime::Now_ms();
  rs_interpolate_time_ms_ += rs_end_time - rs_start_time;
#endif

  NodePath path;
  path.path_dist = std::fabs(rs_path_.total_length);
  path.point_size = 1;
  // use first path end point to fill astar node
  RSPoint rs_end_point;
  rs_path_.FirstPathEndPoint(&rs_end_point);

  path.points[0].x = rs_end_point.x;
  path.points[0].y = rs_end_point.y;
  path.points[0].theta = IflyUnifyTheta(rs_end_point.theta, M_PI);

  rs_node_to_goal->Set(path, XYbounds_, config_, path.path_dist);
  if (!NodeInSearchBound(rs_node_to_goal->GetIndex())) {
    // ILOG_INFO << "node positiong is not expectation";
    rs_node_to_goal->ClearPath();
    return false;
  }

  rs_node_to_goal->SetPathType(AstarPathType::REEDS_SHEPP);
  rs_node_to_goal->SetGearType(rs_path_.paths[0].gear);

  // collision check
  if (!RSPathCollisionCheck(&rs_path_, rs_node_to_goal)) {
    rs_node_to_goal->ClearPath();

    // ILOG_INFO << "rs collision";
    return false;
  }

  // ILOG_INFO << "Reach the end configuration with Reeds Shepp";

  rs_node_to_goal->SetPre(current_node);

  if (!rs_node_to_goal->IsNodeValid()) {
    // ILOG_INFO << "node invalid";
    return false;
  }

  float gcost =
      CalcRSGCostToParentNode(current_node, rs_node_to_goal, &rs_path_);
  rs_node_to_goal->SetGCost(current_node->GetGCost() + gcost);
  rs_node_to_goal->SetHeuCost(0.0);
  rs_node_to_goal->SetFCost();

#if PLOT_RS_EXNTEND_TO_END
  if (rs_path_h_cost_debug_.size() < RS_H_COST_MAX_NUM &&
      current_node->GetGearSwitchNum() <= 1 &&
      current_node->GetGearType() == AstarPathGear::DRIVE) {
    rs_path_h_cost_debug_.emplace_back(rs_path_);
  }
#endif

  // DebugRSPath(&rs_path_);

  return true;
}

bool HybridAStar::SamplingByQunticPolynomial(
    Node3d* current_node, std::vector<AStarPathPoint>& path,
    Node3d* polynomial_node, PolynomialPathErrorCode* fail_type) {
  *fail_type = PolynomialPathErrorCode::NONE;
  if (current_node->GetY() > 0.8 || current_node->GetY() < -0.8) {
    *fail_type = PolynomialPathErrorCode::OUT_OF_Y_BOUNDARY;
    return false;
  }

  float x_diff = current_node->GetX() - astar_end_node_->GetX();
  if (x_diff < 0.2) {
    *fail_type = PolynomialPathErrorCode::OUT_OF_X_BOUNDARY;
    return false;
  }

  if (std::fabs(current_node->GetRadius()) < min_radius_ - 1e-5) {
    *fail_type = PolynomialPathErrorCode::LINK_POINT_INVALID_KAPPA;
    return false;
  }

  float heading_diff = IflyUnifyTheta(current_node->GetPhi(), M_PI) -
                        IflyUnifyTheta(astar_end_node_->GetPhi(), M_PI);
  heading_diff = IflyUnifyTheta(heading_diff, M_PI);
  if (heading_diff > ifly_deg2rad(60.0) || heading_diff < ifly_deg2rad(-60.0)) {
    *fail_type = PolynomialPathErrorCode::OUT_OF_HEADING_BOUNDARY;
    return false;
  }

  // init
  polynomial_node->Clear();
  float min_straight_dist = 0.7;
  float sample_range =
      astar_end_node_->GetX() - (request_.real_goal.GetX() + min_straight_dist);
  int sampline_numer = std::ceil(sample_range / 0.1);
  sampline_numer = std::max(1, sampline_numer);
  bool valid_path = false;

  Pose2D end_pose = astar_end_node_->GetPose();
  for (int i = 0; i < sampline_numer; i++) {
    path.clear();

    GetQunticPolynomialPath(path, current_node->GetPose(),
                            current_node->GetRadius(), end_pose);

    // set node by rs path
    if (path.size() > 1 && IsPolynomialPathSafeByEDT(path, polynomial_node)) {
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
  node_path.points[0].theta = IflyUnifyTheta(path.back().phi, M_PI);

  polynomial_node->Set(node_path, XYbounds_, config_, node_path.path_dist);
  if (!NodeInSearchBound(polynomial_node->GetIndex())) {
    *fail_type = PolynomialPathErrorCode::OUT_OF_SEARCH_BOUNDARY;
    polynomial_node->Clear();
    return false;
  }

  polynomial_node->SetPathType(AstarPathType::QUNTIC_POLYNOMIAL);
  polynomial_node->SetGearType(path.back().gear);
  polynomial_node->SetPre(current_node);
  polynomial_node->SetNext(nullptr);

  if (!polynomial_node->IsNodeValid()) {
    return false;
  }

  Node3d* end_in_pool = node_pool_.AllocateNode();
  if (end_in_pool == nullptr) {
    *fail_type = PolynomialPathErrorCode::FAIL_TO_ALLOCATE_NODE;
    polynomial_node->Clear();
    return false;
  }

  *end_in_pool = *polynomial_node;

  if (end_in_pool != nullptr) {
    ILOG_INFO << " path size " << path.size();
  }

  return true;
}

bool HybridAStar::IsAllPathSegmentLongEnough(const RSPath* reeds_shepp_to_end,
                                             const float father_node_dist) {
  float same_gear_path_min_dist;

  // 双指针搜索
  int rs_path_seg_size = reeds_shepp_to_end->size;
  int left_pointer_id = 0;
  int right_pointer_id = 1;

  while (left_pointer_id < rs_path_seg_size) {
    AstarPathGear gear = reeds_shepp_to_end->paths[left_pointer_id].gear;

    same_gear_path_min_dist =
        std::fabs(reeds_shepp_to_end->paths[left_pointer_id].length);
    if (left_pointer_id == 0) {
      same_gear_path_min_dist += father_node_dist;
    }

    // ILOG_INFO << "left_pointer_id " << left_pointer_id;

    // search same gear path
    for (right_pointer_id = left_pointer_id + 1;
         right_pointer_id < rs_path_seg_size; right_pointer_id++) {
      // same gear
      if (gear == reeds_shepp_to_end->paths[right_pointer_id].gear) {
        same_gear_path_min_dist +=
            std::fabs(reeds_shepp_to_end->paths[right_pointer_id].length);

      } else {
        break;
      }
    }

    left_pointer_id = right_pointer_id;

    if (same_gear_path_min_dist < config_.rs_path_seg_advised_dist) {
      ILOG_INFO << " rs path seg len " << same_gear_path_min_dist;
      return false;
    }
  }

  return true;
}

bool HybridAStar::IsRsPathFirstSegmentLongEnough(
    const RSPath* reeds_shepp_to_end, const float father_node_dist) {
  float len = reeds_shepp_to_end->GetFirstGearLength() + father_node_dist;

  if (len < config_.rs_path_seg_advised_dist) {
    // ILOG_INFO << " rs path first seg len " << len;
    return false;
  }

  return true;
}

bool HybridAStar::RsLastSegmentSatisfyRequest(
    const RSPath* reeds_shepp_to_end) {
  int rs_path_seg_size = reeds_shepp_to_end->size;
  if (rs_path_seg_size < 1) {
    return true;
  }

  AstarPathGear first_gear = reeds_shepp_to_end->paths[0].gear;
  const AstarPathGear last_gear =
      reeds_shepp_to_end->paths[rs_path_seg_size - 1].gear;

  if (request_.space_type == ParkSpaceType::VERTICAL) {
    if (request_.direction_request == ParkingVehDirection::TAIL_IN &&
        request_.rs_request == RSPathRequestType::LAST_PATH_FORBID_FORWARD) {
      if (last_gear == AstarPathGear::DRIVE) {
        // ILOG_INFO << " rs path last seg len is drive gear ";

        return false;
      }
    } else if (request_.direction_request == ParkingVehDirection::HEAD_IN) {
      if (request_.rs_request == RSPathRequestType::LAST_PATH_FORBID_REVERSE &&
          last_gear == AstarPathGear::REVERSE) {
        // ILOG_INFO << " rs path last seg len is reverse gear ";
        return false;
      } else if (first_gear == AstarPathGear::DRIVE) {
        int i = 1;
        while (i < rs_path_seg_size && first_gear == AstarPathGear::DRIVE) {
          first_gear = reeds_shepp_to_end->paths[i].gear;
          i++;
        }
        const RSPoint first_drive_path_end_pos =
            rs_path_interface_.GetAnchorPoint().points[i];
        // ILOG_INFO << "first drive end pos = " << first_drive_path_end_pos.x
        //           << ", " << first_drive_path_end_pos.y;

        if (first_drive_path_end_pos.x < astar_end_node_->GetX() - 0.15 &&
            std::fabs(first_drive_path_end_pos.y) <
                config_.headin_limit_y_shrink) {
          return false;
        }
      }
    }
  }

  return true;
}

bool HybridAStar::CheckRSPathGear(const RSPath* reeds_shepp_to_end,
                                  const PathGearRequest gear_request_info) {
  int rs_path_seg_size = reeds_shepp_to_end->size;
  if (rs_path_seg_size < 1) {
    return true;
  }

  // delete gear switch bigger than 1.
  if (reeds_shepp_to_end->gear_change_number > 1) {
    return false;
  }

  AstarPathGear gear;

  for (int i = 0; i < rs_path_seg_size; i++) {
    gear = reeds_shepp_to_end->paths[i].gear;

    if (request_.space_type == ParkSpaceType::VERTICAL) {
      if (request_.rs_request == RSPathRequestType::ALL_PATH_FORBID_FORWARD &&
          gear == AstarPathGear::DRIVE) {
        // ILOG_INFO << " rs path seg need single shot by reverse gear ";
        return false;

      } else if (request_.rs_request ==
                     RSPathRequestType::ALL_PATH_FORBID_REVERSE &&
                 gear == AstarPathGear::REVERSE) {
        // ILOG_INFO << " rs path seg need single shot by drive gear ";
        return false;
      }

      if (gear_request_info == PathGearRequest::GEAR_REVERSE_ONLY &&
          gear == AstarPathGear::DRIVE) {
        // ILOG_INFO << "gear is not expectation";
        return false;
      } else if (gear_request_info == PathGearRequest::GEAR_DRIVE_ONLY &&
                 gear == AstarPathGear::REVERSE) {
        // ILOG_INFO << "gear is not expectation";
        return false;
      }
    }
  }

  return true;
}

bool HybridAStar::RSPathCollisionCheck(const RSPath* reeds_shepp_to_end,
                                       Node3d* rs_node_to_goal) {
  // length check
  if (reeds_shepp_to_end->size < 1) {
    return false;
  }

  // collision check
#if LOG_TIME_PROFILE
  double check_start_time = IflyTime::Now_ms();
#endif

  bool is_valid = IsRSPathSafeByEDT(reeds_shepp_to_end, rs_node_to_goal);
  // if (!is_valid) {
  // ILOG_INFO << "rs collision "
  //           << static_cast<int>(rs_end_node_.GetConstCollisionType());
  // }

#if LOG_TIME_PROFILE
  double check_end_time = IflyTime::Now_ms();
  collision_check_time_ms_ += check_end_time - check_start_time;
#endif

  return is_valid;
}

bool HybridAStar::ValidityCheckByConvex(Node3d* node) {
  if (node == nullptr) {
    return false;
  }

  if (node->GetStepSize() <= 0) {
    return false;
  }

  if (obstacles_->IsEmpty()) {
    return true;
  }

  size_t node_step_size = node->GetStepSize();
  const NodePath& path = node->GetNodePath();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t check_start_index = 0;
  if (node_step_size == 1) {
    check_start_index = 0;
  } else {
    check_start_index = 1;
  }

#if LOG_TIME_PROFILE
  double check_start_time = IflyTime::Now_ms();
#endif

  Polygon2D global_polygon;
  Pose2D global_pose;
  bool is_collision = false;
  cdl::AABB path_point_aabb;

  Polygon2D* veh_local_polygon = GetVehPolygon(node->GetGearType());

  for (size_t i = check_start_index; i < node_step_size; ++i) {
    // check bound
    if (IsPointBeyondBound(path.points[i].x, path.points[i].y)) {
      node->SetCollisionType(NodeCollisionType::MAP_BOUND);
      return false;
    }

    global_pose = path.points[i];

    RULocalPolygonToGlobal(&global_polygon, veh_local_polygon, &global_pose);

    GetBoundingBoxByPolygon(&path_point_aabb, &global_polygon);
    if (clear_zone_->IsContain(path_point_aabb)) {
      // ILOG_INFO << "clear";
      continue;
    }

    for (const auto& obstacle : obstacles_->virtual_obs) {
      gjk_interface_.PolygonPointCollisionDetect(&is_collision, &global_polygon,
                                                 obstacle);

      if (is_collision) {
        node->SetCollisionType(NodeCollisionType::VIRTUAL_WALL);
        return false;
      }
    }

    for (const auto& obstacle : obstacles_->point_cloud_list) {
      // envelop box check
      gjk_interface_.PolygonCollisionByCircleCheck(
          &is_collision, &obstacle.envelop_polygon, &global_polygon, 0.01);

      if (!is_collision) {
        continue;
      }

      // internal points
      for (size_t j = 0; j < obstacle.points.size(); j++) {
        gjk_interface_.PolygonPointCollisionDetect(
            &is_collision, &global_polygon, obstacle.points[j]);

        if (is_collision) {
          node->SetCollisionType(NodeCollisionType::FUSION_OCC_OBS);
          return false;
        }
      }
    }
  }

#if LOG_TIME_PROFILE
  double check_end_time = IflyTime::Now_ms();
  collision_check_time_ms_ += check_end_time - check_start_time;

#endif

  return true;
}

const bool HybridAStar::ValidityCheckByEDT(Node3d* node) {
  if (node == nullptr) {
    return false;
  }

  if (node->GetStepSize() <= 0) {
    return false;
  }

  if (obstacles_->IsEmpty()) {
    return true;
  }

  node->SetDistToObs(0.0f);
  const NodePath& path = node->GetNodePath();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t node_step_size = node->GetStepSize();
  size_t check_start_index = 0;
  if (node_step_size == 1) {
    check_start_index = 0;
  } else {
    check_start_index = 1;
  }

  Polygon2D global_polygon;
  Pose2D global_pose;
  cdl::AABB path_point_aabb;
  Transform2d tf;
  AstarPathGear node_gear = node->GetGearType();
  AstarPathGear point_gear;
  Polygon2D* veh_local_polygon = GetVehPolygon(node_gear);
  bool is_circle_path = IsCirclePathBySteeringWheel(node->GetSteer());
  FootPrintCircleModel* footprint_model =
      GetCircleFootPrintModel(path.points[0], is_circle_path);

  float dist = 100.0;
  float min_dist = 100.0;

  for (size_t i = check_start_index; i < node_step_size; ++i) {
    // check bound
    if (IsPointBeyondBound(path.points[i].x, path.points[i].y)) {
      node->SetCollisionType(NodeCollisionType::MAP_BOUND);
      // node->SetCollisionID(i);

      // ILOG_INFO << "x " << path.points[i].x << " y " << path.points[i].y
      //           << " i " << i;

      return false;
    }

    global_pose = path.points[i];
    tf.SetBasePose(global_pose);

    RULocalPolygonToGlobalFast(&global_polygon, veh_local_polygon, &global_pose,
                               tf.GetCosTheta(), tf.GetSinTheta());

    GetBoundingBoxByPolygon(&path_point_aabb, &global_polygon);
    if (clear_zone_->IsContain(path_point_aabb)) {
      // ILOG_INFO << "clear";
      continue;
    }

    if (i == node_step_size - 1) {
      point_gear = node_gear;
    } else {
      point_gear = AstarPathGear::NONE;
    }

// for accelerate calculation, use macro
#if ENABLE_OBS_DIST_G_COST
    if (edt_->DistanceCheckForPoint(&dist, &tf, point_gear)) {
      node->SetCollisionType(NodeCollisionType::FUSION_OCC_OBS);
      node->SetDistToObs(dist);
      // node->SetCollisionID(i);

      return false;
    }

    if (dist < min_dist) {
      min_dist = dist;
    }
#else

    if (edt_->IsCollisionForPoint(&tf, point_gear, footprint_model)) {
      node->SetCollisionType(NodeCollisionType::FUSION_OCC_OBS);
      // node->SetCollisionID(i);

      return false;
    }

#endif

    // ILOG_INFO << "path size " << node_step_size << " ,pt id " << i
    //           << " , no collision ";
  }

  node->SetDistToObs(min_dist);

  return true;
}

void HybridAStar::DebugEDTCheck(HybridAStarResult* path) {
  if (path == nullptr || path->x.size() < 1) {
    return;
  }

  Pose2D global_pose;
  // bool is_collision = false;
  Transform2d tf;
  AstarPathGear gear = AstarPathGear::NONE;

  float min_dist = 100;
  float dist;
  size_t point_size = path->x.size();
  for (size_t i = 0; i < point_size; ++i) {
    global_pose.x = path->x[i];
    global_pose.y = path->y[i];
    global_pose.theta = path->phi[i];
    tf.SetBasePose(global_pose);

    if (edt_->DistanceCheckForPoint(&dist, &tf, gear)) {
      ILOG_INFO << "collision";
    }

    min_dist = std::min(dist, min_dist);
    ILOG_INFO << "path size " << point_size << ", pt id " << i
              << ", dist= " << dist;
  }

  ILOG_INFO << "min_dist = " << min_dist;

  return;
}

bool HybridAStar::IsRSPathSafeByConvexHull(const RSPath* reeds_shepp_path,
                                           Node3d* node) {
  if (reeds_shepp_path == nullptr) {
    return false;
  }

  if (reeds_shepp_path->size <= 0) {
    return true;
  }

  if (obstacles_->IsEmpty()) {
    return true;
  }

  size_t point_size;
  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t check_start_index = 0;

  Polygon2D global_polygon;
  Pose2D global_pose;
  bool is_collision;
  cdl::AABB path_point_aabb;
  Polygon2D* veh_local_polygon = nullptr;

  for (int seg_id = 0; seg_id < reeds_shepp_path->size; seg_id++) {
    const RSPathSegment* segment = &reeds_shepp_path->paths[seg_id];

    point_size = segment->size;
    if (point_size == 1) {
      check_start_index = 0;
    } else {
      check_start_index = 1;
    }

    for (size_t i = check_start_index; i < point_size; ++i) {
      // check bound
      if (IsPointBeyondBound(segment->points[i].x, segment->points[i].y)) {
        node->SetCollisionType(NodeCollisionType::MAP_BOUND);
        return false;
      }

      global_pose.x = segment->points[i].x;
      global_pose.y = segment->points[i].y;
      global_pose.theta = segment->points[i].theta;

      veh_local_polygon = GetVehPolygon(segment->gear);

      RULocalPolygonToGlobal(&global_polygon, veh_local_polygon, &global_pose);

      GetBoundingBoxByPolygon(&path_point_aabb, &global_polygon);
      if (clear_zone_->IsContain(path_point_aabb)) {
        // ILOG_INFO << "clear";
        continue;
      }

      for (const auto& obstacle : obstacles_->virtual_obs) {
        gjk_interface_.PolygonPointCollisionDetect(&is_collision,
                                                   &global_polygon, obstacle);

        if (is_collision) {
          node->SetCollisionType(NodeCollisionType::VIRTUAL_WALL);
          return false;
        }
      }

      for (const auto& obstacle : obstacles_->point_cloud_list) {
        // envelop box check
        gjk_interface_.PolygonCollisionByCircleCheck(
            &is_collision, &obstacle.envelop_polygon, &global_polygon, 0.01);

        if (!is_collision) {
          continue;
        }

        // internal points
        for (size_t j = 0; j < obstacle.points.size(); j++) {
          gjk_interface_.PolygonPointCollisionDetect(
              &is_collision, &global_polygon, obstacle.points[j]);

          if (is_collision) {
            node->SetCollisionType(NodeCollisionType::FUSION_OCC_OBS);
            return false;
          }
        }
      }
      //
    }
  }

  return true;
}

const bool HybridAStar::IsRSPathSafeByEDT(const RSPath* reeds_shepp_path,
                                          Node3d* node) {
  if (reeds_shepp_path == nullptr) {
    return false;
  }

  if (reeds_shepp_path->size <= 0) {
    return true;
  }

  if (obstacles_->IsEmpty()) {
    return true;
  }

  size_t point_size;

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t check_start_index = 0;

  Polygon2D global_polygon;
  Pose2D global_pose;
  // bool is_collision;
  cdl::AABB path_point_aabb;
  Transform2d tf;

  Polygon2D* veh_local_polygon = nullptr;
  AstarPathGear point_gear;
  bool is_circle_path;

  for (int seg_id = 0; seg_id < reeds_shepp_path->size; seg_id++) {
    const RSPathSegment* segment = &reeds_shepp_path->paths[seg_id];

    point_size = segment->size;
    if (point_size == 1) {
      check_start_index = 0;
    } else {
      check_start_index = 1;
    }
    is_circle_path = IsCirclePathByKappa(segment->kappa);

    for (size_t i = check_start_index; i < point_size; ++i) {
      // check bound
      if (IsPointBeyondBound(segment->points[i].x, segment->points[i].y)) {
        node->SetCollisionType(NodeCollisionType::MAP_BOUND);
        return false;
      }

      global_pose.x = segment->points[i].x;
      global_pose.y = segment->points[i].y;
      global_pose.theta = segment->points[i].theta;

      tf.SetBasePose(global_pose);

      veh_local_polygon = GetVehPolygon(segment->gear);

      RULocalPolygonToGlobalFast(&global_polygon, veh_local_polygon,
                                 &global_pose, tf.GetCosTheta(),
                                 tf.GetSinTheta());

      GetBoundingBoxByPolygon(&path_point_aabb, &global_polygon);
      if (clear_zone_->IsContain(path_point_aabb)) {
        continue;
      }

      if (i == point_size - 1) {
        point_gear = segment->gear;
      } else {
        point_gear = AstarPathGear::NONE;
      }

      if (edt_->IsCollisionForPoint(
              &tf, point_gear,
              GetCircleFootPrintModel(global_pose, is_circle_path))) {
        node->SetCollisionType(NodeCollisionType::FUSION_OCC_OBS);
        return false;
      }
    }
  }

  return true;
}

const bool HybridAStar::IsPolynomialPathSafeByEDT(
    const std::vector<AStarPathPoint>& path, Node3d* node) {
  if (path.size() <= 0) {
    return true;
  }

  if (obstacles_->IsEmpty()) {
    return true;
  }

  size_t point_size = path.size();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t check_start_index = 0;

  Polygon2D global_polygon;
  Pose2D global_pose;
  // bool is_collision;
  cdl::AABB path_point_aabb;
  Transform2d tf;

  Polygon2D* veh_local_polygon = nullptr;
  AstarPathGear point_gear;

  for (size_t i = check_start_index; i < point_size; ++i) {
    // check bound
    if (IsPointBeyondBound(path[i].x, path[i].y)) {
      node->SetCollisionType(NodeCollisionType::MAP_BOUND);
      return false;
    }

    global_pose.x = path[i].x;
    global_pose.y = path[i].y;
    global_pose.theta = path[i].phi;

    tf.SetBasePose(global_pose);

    veh_local_polygon = GetVehPolygon(path[i].gear);

    // ILOG_INFO << "gear " << PathGearDebugString(segment->gear);

    RULocalPolygonToGlobalFast(&global_polygon, veh_local_polygon, &global_pose,
                               tf.GetCosTheta(), tf.GetSinTheta());

    GetBoundingBoxByPolygon(&path_point_aabb, &global_polygon);
    if (clear_zone_->IsContain(path_point_aabb)) {
      // ILOG_INFO << "clear";
      continue;
    }

    if (i == point_size - 1) {
      point_gear = path[i].gear;
    } else {
      point_gear = AstarPathGear::NONE;
    }

    if (edt_->IsCollisionForPoint(
            &tf, point_gear,
            GetCircleFootPrintModel(global_pose,
                                    false))) {
      node->SetCollisionType(NodeCollisionType::FUSION_OCC_OBS);
      return false;
    }
  }

  return true;
}

size_t HybridAStar::GetPathCollisionIndex(HybridAStarResult* result) {
  if (result == nullptr) {
    return 0;
  }

  if (result->x.size() <= 0) {
    return 0;
  }

  size_t path_end_id = result->x.size() - 1;

  if (obstacles_->point_cloud_list.empty() && obstacles_->virtual_obs.empty()) {
    return path_end_id;
  }

  Polygon2D polygon;
  Pose2D global_pose;
  bool is_collision;

  size_t collision_index = 100000;
  cdl::AABB path_point_aabb;

  Polygon2D* veh_local_polygon = nullptr;

  for (size_t i = 0; i <= path_end_id; ++i) {
    // check bound
    if (IsPointBeyondBound(result->x[i], result->y[i])) {
      collision_index = i;

#if DEBUG_GJK
      ILOG_INFO << "i=" << i << "beyond bound";
#endif
      break;
    }

    global_pose.x = result->x[i];
    global_pose.y = result->y[i];
    global_pose.theta = result->phi[i];

    veh_local_polygon = GetVehPolygon(result->gear[i]);
    RULocalPolygonToGlobal(&polygon, veh_local_polygon, &global_pose);

    GetBoundingBoxByPolygon(&path_point_aabb, &polygon);
    if (clear_zone_->IsContain(path_point_aabb)) {
#if DEBUG_GJK
      ILOG_INFO << "i=" << i << "clear";
#endif
      continue;
    }

    for (const auto& obstacle : obstacles_->point_cloud_list) {
      // envelop box check
      gjk_interface_.PolygonCollisionByCircleCheck(
          &is_collision, &obstacle.envelop_polygon, &polygon, 0.01);

      if (!is_collision) {
        continue;
      }

      // internal points
      for (size_t j = 0; j < obstacle.points.size(); j++) {
        gjk_interface_.PolygonPointCollisionDetect(&is_collision, &polygon,
                                                   obstacle.points[j]);

        if (is_collision) {
          collision_index = i;

#if DEBUG_GJK
          ILOG_INFO << "path id=" << i
                    << ",obs type=" << static_cast<int>(obstacle.obs_type)
                    << ",obs size=" << obstacle.points.size();
#endif
          return collision_index;
        }
      }
    }
  }

  return collision_index;
}

size_t HybridAStar::GetPathCollisionIDByEDT(HybridAStarResult* result) {
  if (result == nullptr) {
    return 0;
  }

  if (result->x.size() <= 0) {
    return 0;
  }

  size_t path_end_id = result->x.size() - 1;

  if (obstacles_->point_cloud_list.empty() && obstacles_->virtual_obs.empty()) {
    return path_end_id;
  }

  Polygon2D polygon;
  Pose2D global_pose;

  size_t collision_index = 100000;
  cdl::AABB path_point_aabb;
  Polygon2D* veh_local_polygon = nullptr;
  Transform2d tf;

  for (size_t i = 0; i <= path_end_id; ++i) {
    // check bound
    if (IsPointBeyondBound(result->x[i], result->y[i])) {
      collision_index = i;

      break;
    }

    global_pose.x = result->x[i];
    global_pose.y = result->y[i];
    global_pose.theta = result->phi[i];
    tf.SetBasePose(global_pose);

    veh_local_polygon = GetVehPolygon(result->gear[i]);
    RULocalPolygonToGlobalFast(&polygon, veh_local_polygon, &global_pose,
                               tf.GetCosTheta(), tf.GetSinTheta());

    GetBoundingBoxByPolygon(&path_point_aabb, &polygon);
    if (clear_zone_->IsContain(path_point_aabb)) {
      continue;
    }

    if (edt_->IsCollisionForPoint(&tf, result->gear[i],
                                  GetCircleFootPrintModel(global_pose,false))) {
      return i;
    }
  }

  return collision_index;
}

void HybridAStar::GetPathByBicycleModel(NodePath* path, const float arc,
                                        const float radius,
                                        const bool is_forward) {
  int path_point_num = std::ceil(arc / node_path_dist_resolution_);
  // ILOG_INFO << "path point num " << path_point_num;
  // ILOG_INFO << "node path s resolution " << traveled_distance;

  int kinetics_model_number =
      std::round(node_path_dist_resolution_ / kinetics_model_step_);

  path->path_dist = 0;

  Pose2D* old_point;
  old_point = &path->points[0];

  for (int i = 0; i < path_point_num; ++i) {
    UpdatePoseBySamplingNumber(old_point, radius, kinetics_model_number,
                               &path->points[path->point_size], is_forward);

    old_point = &path->points[path->point_size];

    path->point_size++;
    path->path_dist += node_path_dist_resolution_;
    if (path->point_size >= NODE_PATH_MAX_POINT) {
      ILOG_INFO << "size too much";
      break;
    }
  }

  return;
}

void HybridAStar::GetPathByCircle(NodePath* path, const float arc,
                                  const float radius, const bool is_forward) {
  int path_point_num = std::ceil(arc / node_path_dist_resolution_);
  // ILOG_INFO << "path point num " << path_point_num;
  // ILOG_INFO << "node path s resolution " << traveled_distance;

  // get vehicle circle
  VehicleCircle veh_circle;
  AstarPathGear gear;
  if (is_forward) {
    gear = AstarPathGear::DRIVE;
  } else {
    gear = AstarPathGear::REVERSE;
  }

  GetVehCircleByPose(&veh_circle, &path->points[0], radius, gear);

  // interpolate
  path->path_dist = 0;

  Pose2D* start_pose;
  Pose2D* next_pose;
  start_pose = &path->points[0];

  float acc_s = 0.0;

  for (int i = 0; i < path_point_num; ++i) {
    next_pose = &path->points[path->point_size];
    acc_s += node_path_dist_resolution_;

    InterpolateByArcOffset(next_pose, &veh_circle, start_pose, acc_s,
                           inv_radius_);

    path->point_size++;
    path->path_dist += node_path_dist_resolution_;

    if (path->point_size >= NODE_PATH_MAX_POINT) {
      ILOG_INFO << "size too much";
      break;
    }
  }

  return;
}

void HybridAStar::GetPathByLine(NodePath* path, const float arc,
                                const bool is_forward) {
  int path_point_num = std::ceil(arc / node_path_dist_resolution_);
  // ILOG_INFO << "path point num " << path_point_num;
  // ILOG_INFO << "node path s resolution " << traveled_distance;

  float inc_dist;
  if (is_forward) {
    inc_dist = node_path_dist_resolution_;
  } else {
    inc_dist = -node_path_dist_resolution_;
  }

  path->path_dist = 0;
  float acc_s = 0.0;

  // get unit vector
  Pose2D unit_vector;
  unit_vector.x = std::cos(path->points[0].GetPhi());
  unit_vector.y = std::sin(path->points[0].GetPhi());

  Pose2D* start_pose = &path->points[0];
  Pose2D* next_pose;

  for (int j = 0; j < path_point_num; j++) {
    next_pose = &path->points[path->point_size];
    acc_s += inc_dist;

    GetStraightLinePoint(next_pose, start_pose, acc_s, &unit_vector);

    // ILOG_INFO << "start " << state_next->x << " " << state_next->y << " acc_s
    // "
    //           << acc_s;

    path->point_size++;
    path->path_dist += node_path_dist_resolution_;

    if (path->point_size >= NODE_PATH_MAX_POINT) {
      ILOG_INFO << "size too much";
      break;
    }
  }

  return;
}

const NodeShrinkType HybridAStar::NextNodeGenerator(
    Node3d* new_node, Node3d* parent_node, size_t next_node_index,
    const PathGearRequest gear_request_info) {
  float front_wheel_angle = 0.0;
  float radius = 0.0;
  float traveled_distance = 0.0;

  // clear
  new_node->ClearPath();

  // update angle
  if (next_node_index < next_node_angles_.size) {
    front_wheel_angle = next_node_angles_.angles[next_node_index];
    radius = next_node_angles_.radius[next_node_index];
    traveled_distance = node_path_dist_resolution_;
  } else if (next_node_index < next_node_angles_.size * 2) {
    size_t index = next_node_index - next_node_angles_.size;
    front_wheel_angle = next_node_angles_.angles[index];
    radius = next_node_angles_.radius[index];
    traveled_distance = -node_path_dist_resolution_;
  } else {
    return NodeShrinkType::NONE;
  }

  // gear check
  if (gear_request_info == PathGearRequest::GEAR_REVERSE_ONLY &&
      traveled_distance > 0.0) {
    return NodeShrinkType::UNEXPECTED_GEAR;
  } else if (gear_request_info == PathGearRequest::GEAR_DRIVE_ONLY &&
             traveled_distance < 0.0) {
    return NodeShrinkType::UNEXPECTED_GEAR;
  }

  if (parent_node->IsStartNode()) {
    if (request_.first_action_request.gear_request == AstarPathGear::DRIVE &&
        traveled_distance < 0.0) {
      return NodeShrinkType::UNEXPECTED_GEAR;
    } else if (request_.first_action_request.gear_request ==
                   AstarPathGear::REVERSE &&
               traveled_distance > 0.0) {
      return NodeShrinkType::UNEXPECTED_GEAR;
    }
  }

  // take above motion primitive to generate a curve driving the car to a
  // different grid
  // float node_step = std::sqrt(2) * xy_grid_resolution_;
  float node_step = config_.node_step;

  NodePath path;
  path.point_size = 1;
  path.points[0].x = parent_node->GetX();
  path.points[0].y = parent_node->GetY();
  path.points[0].theta = parent_node->GetPhi();

  bool is_forward = traveled_distance > 0.0 ? true : false;

  // generate path by circle
  // if (std::fabs(front_wheel_angle) > 0.0001) {
  //   GetPathByCircle(&path, node_step, radius, is_forward);
  // } else {
  //   GetPathByLine(&path, node_step, is_forward);
  // }

  // generate path by bycicle model
  if (std::fabs(front_wheel_angle) > 0.0001) {
    GetPathByBicycleModel(&path, node_step, radius, is_forward);
  } else {
    GetPathByLine(&path, node_step, is_forward);
  }

  // check if the vehicle runs outside of XY boundary
  const Pose2D& end_point = path.GetEndPoint();
  if (IsPointBeyondBound(end_point.x, end_point.y)) {
    return NodeShrinkType::OUT_OF_BOUNDARY;
  }

  new_node->Set(path, XYbounds_, config_, path.path_dist);

  // check search bound
  if (!NodeInSearchBound(new_node->GetIndex())) {
    new_node->ClearPath();
    return NodeShrinkType::OUT_OF_BOUNDARY;
  }

  bool heading_legal = false;
  heading_legal = node_shrink_decider_.IsLegalForHeading(new_node->GetPhi());
  if (!heading_legal) {
#if PLOT_DELETE_NODE
    delete_queue_path_debug_.emplace_back(
        Vec2df32(new_node->GetX(), new_node->GetY()));
#endif
    // ILOG_INFO << "heading is illegal";
    new_node->ClearPath();
    return NodeShrinkType::UNEXPECTED_HEADING;
  }

  // headin shrink limit pose
  if (!node_shrink_decider_.IsLegalByXBound(new_node->GetX())) {
#if PLOT_DELETE_NODE
    delete_queue_path_debug_.emplace_back(
        Vec2df32(new_node->GetX(), new_node->GetY()));
#endif
    // ILOG_INFO << "pos is illegal";
    new_node->ClearPath();
    return NodeShrinkType::UNEXPECTED_POS;
  }

  new_node->SetPre(parent_node);

  AstarPathGear gear;
  if (traveled_distance > 0.0) {
    gear = AstarPathGear::DRIVE;
  } else {
    gear = AstarPathGear::REVERSE;
  }
  new_node->SetGearType(gear);
  if (parent_node->IsPathGearChange(gear)) {
    new_node->SetGearSwitchNum(parent_node->GetGearSwitchNum() + 1);
  } else {
    new_node->SetGearSwitchNum(parent_node->GetGearSwitchNum());
  }

  new_node->SetSteer(front_wheel_angle);
  new_node->SetPathType(AstarPathType::NODE_SEARCHING);
  new_node->SetDistToStart(path.path_dist + parent_node->GetDistToStart());
  new_node->SetRadius(radius);

  if (new_node->GetGearSwitchNum() == 0) {
    new_node->SetGearSwitchNode(nullptr);
  } else if (new_node->GetGearSwitchNum() == 1 &&
             parent_node->IsPathGearChange(gear)) {
    new_node->SetGearSwitchNode(parent_node);
  } else {
    new_node->SetGearSwitchNode(parent_node->GearSwitchNode());
  }

  // ILOG_INFO << "next node end";
  // new_node->GetPose().DebugString();

  return NodeShrinkType::NONE;
}

void HybridAStar::CalculateNodeFCost(Node3d* current_node, Node3d* next_node) {
  CalculateNodeGCost(current_node, next_node);

  CalculateNodeHeuristicCost(current_node, next_node);

  return;
}

void HybridAStar::CalculateNodeHeuristicCost(Node3d* father_node,
                                             Node3d* next_node) {
#if LOG_TIME_PROFILE
  const double start_time = IflyTime::Now_ms();
#endif

  NodeHeuristicCost cost;
  // evaluate heuristic cost
  float optimal_path_cost = 0.0;
  float dp_path_dist = 0.0;

  float dp_path_cost = 0.0;
  dp_path_dist = ObstacleHeuristicWithHolonomic(next_node);
  dp_path_cost = dp_path_dist * config_.traj_forward_penalty;
  cost.astar_dist = dp_path_cost;

  float rs_path_cost = 0.0;
  rs_path_cost = GenerateHeuristicCostByRsPath(next_node, &cost);
  optimal_path_cost = std::max(dp_path_cost, rs_path_cost);

  // heading cost
  float ref_line_heading_cost = 0.0;
  ref_line_heading_cost = GenerateRefLineHeuristicCost(next_node, dp_path_dist);
  cost.ref_line_heading_cost = ref_line_heading_cost;

  optimal_path_cost = std::max(optimal_path_cost, ref_line_heading_cost);
  // optimal_path_cost += ref_line_heading_cost;

  // euler cost
  float euler_dist_cost = 0.0;
  euler_dist_cost = next_node->GetEulerDist(astar_end_node_);
  cost.euler_dist = euler_dist_cost;

  optimal_path_cost = std::max(euler_dist_cost, optimal_path_cost);

  next_node->SetHeuCost(optimal_path_cost);
  // next_node->SetHeuCostDebug(cost);

#if LOG_TIME_PROFILE
  const double end_time = IflyTime::Now_ms();
  heuristic_time_ += end_time - start_time;
#endif

  return;
}

void HybridAStar::GetSingleShotNodeHeuCost(const Node3d* father_node,
                                           Node3d* next_node) {
  NodeHeuristicCost cost;
  float optimal_path_cost = 0.0;

  // euler cost
  float euler_dist_cost = 0.0;
  euler_dist_cost = next_node->GetEulerDist(astar_end_node_);
  cost.euler_dist = euler_dist_cost;

  optimal_path_cost = std::max(euler_dist_cost, optimal_path_cost);

  next_node->SetHeuCost(optimal_path_cost);
  // next_node->SetHeuCostDebug(cost);

  return;
}

void HybridAStar::CalculateNodeGCost(Node3d* current_node, Node3d* next_node) {
  next_node->SetGCost(current_node->GetGCost() +
                      CalcGCostToParentNode(current_node, next_node));

  return;
}

float HybridAStar::GenerateHeuristicCostByRsPath(Node3d* next_node,
                                                  NodeHeuristicCost* cost) {
  RSPathRequestType rs_request = RSPathRequestType::NONE;
  if (!CalcRSPathToGoal(next_node, false, false, rs_request,
                        vehicle_param_.min_turn_radius)) {
    ILOG_INFO << "ShortestRSP failed";
    return 100.0;
  }

  float path_dist = std::fabs(rs_path_.total_length);

  float dist_cost = path_dist * config_.traj_forward_penalty;
  cost->rs_path_dist = dist_cost;

  // float gear_cost = 0.0;
  // float steer_change_cost = 0.0;

  // for (int i = 0; i < rs_path_.size - 1; i++) {
  //   // gear cost
  //   if (rs_path_.paths[i].gear != rs_path_.paths[i + 1].gear) {
  //     gear_cost += config_.gear_switch_penalty_heu;
  //   }

  //   // steer change cost
  //   if (rs_path_.paths[i].steer != rs_path_.paths[i + 1].steer) {
  //     if (rs_path_.paths[i].steer == RS_STRAIGHT ||
  //         rs_path_.paths[i + 1].steer == RS_STRAIGHT) {
  //       steer_change_cost +=
  //           config_.traj_steer_change_penalty * max_steer_angle_;
  //     } else {
  //       steer_change_cost +=
  //           config_.traj_steer_change_penalty * max_steer_angle_ * 2;
  //     }
  //   }
  // }

  // // gear cost
  // if (next_node->GetGearType() != rs_path_.paths[0].gear) {
  //   gear_cost += config_.gear_switch_penalty_heu;
  // }

  // // steer cost
  // if (next_node->GetSteer() > 0.0 && rs_path_.paths[0].steer != RS_LEFT) {
  //   steer_change_cost += config_.traj_steer_change_penalty * max_steer_angle_;
  // } else if (next_node->GetSteer() < 0.0 &&
  //            rs_path_.paths[0].steer != RS_RIGHT) {
  //   steer_change_cost += config_.traj_steer_change_penalty * max_steer_angle_;
  // }

  // cost->rs_path_gear = gear_cost;
  // cost->rs_path_steer = steer_change_cost;

#if PLOT_RS_COST_PATH
  if (rs_path_h_cost_debug_.size() < RS_H_COST_MAX_NUM) {
    const Pose2D& rs_start_pose = next_node->GetPose();
    rs_path_interface_.RSPathInterpolate(&rs_path_, &rs_start_pose,
                                         vehicle_param_.min_turn_radius);
    rs_path_h_cost_debug_.emplace_back(rs_path_);
  }
#endif

  return dist_cost;
}

float HybridAStar::CalcGCostToParentNode(Node3d* current_node,
                                         Node3d* next_node) {
  // evaluate cost on the trajectory and add current cost
  float piecewise_cost = 0.0;
  float path_dist = 0.0;
  path_dist = next_node->GetNodePathDistance();

  if (next_node->IsForward()) {
    piecewise_cost += path_dist * config_.traj_forward_penalty;
  } else {
    piecewise_cost += path_dist * config_.traj_reverse_penalty;
  }

  // gear punish
  if (current_node->IsPathGearChange(next_node->GetGearType())) {
    piecewise_cost += config_.gear_switch_penalty;
  }

  // steering wheel angle cost
  // for start node, steering angle can be any value
  if (!current_node->IsStartNode()) {
    piecewise_cost +=
        config_.traj_steer_penalty * std::fabs(next_node->GetSteer());

    // steering wheel change
    piecewise_cost +=
        config_.traj_steer_change_penalty *
        std::fabs(next_node->GetSteer() - current_node->GetSteer());
  }

  // request dist and gear cost
  if (request_.plan_reason == PlanningReason::FIRST_PLAN) {
    if (current_node->GetDistToStart() <
        request_.first_action_request.dist_request) {
      if (current_node->IsPathGearChange(next_node->GetGearType())) {
        piecewise_cost += config_.expect_dist_penalty;
      }
    }
  } else {
    if (next_node->GetDistToStart() <
            request_.first_action_request.dist_request ||
        current_node->GetDistToStart() <
            request_.first_action_request.dist_request) {
      // gear is different
      if (next_node->IsPathGearChange(
              request_.first_action_request.gear_request)) {
        piecewise_cost += config_.expect_dist_penalty;
      }
    }
  }

  // safe dist cost
#if ENABLE_OBS_DIST_G_COST
  float safe_punish = 0.0;
  safe_punish = CalcSafeDistCost(next_node);
  piecewise_cost += safe_punish;
#endif

  // ref line heading cost
  // float heading_cost =
  //     std::fabs(GetThetaDiff(next_node->GetPhi(), ref_line_.GetHeading())) *
  //     config_.ref_line_heading_penalty;
  // piecewise_cost += heading_cost;

  return piecewise_cost;
}

float HybridAStar::CalcRSGCostToParentNode(Node3d* current_node,
                                           Node3d* rs_node,
                                           const RSPath* rs_path) {
  rs_node->SetGearSwitchNum(current_node->GetGearSwitchNum());
  // evaluate cost on the trajectory and add current cost
  float piecewise_cost = 0.0;
  float path_dist = rs_path->total_length;
  piecewise_cost += path_dist * config_.traj_forward_penalty;

  float gear_cost = 0.0;
  float steer_change_cost = 0.0;
  for (int i = 0; i < rs_path_.size - 1; i++) {
    // gear cost
    if (rs_path_.paths[i].gear != rs_path_.paths[i + 1].gear) {
      gear_cost += config_.gear_switch_penalty;

      rs_node->AddGearSwitchNumber();
    }

    // steer change cost
    if (rs_path_.paths[i].steer != rs_path_.paths[i + 1].steer) {
      if (rs_path_.paths[i].steer == RS_STRAIGHT ||
          rs_path_.paths[i + 1].steer == RS_STRAIGHT) {
        steer_change_cost +=
            config_.traj_steer_change_penalty * max_steer_angle_;
      } else {
        steer_change_cost +=
            config_.traj_steer_change_penalty * max_steer_angle_ * 2;
      }
    }
  }

  // gear cost
  bool gear_switch = current_node->IsPathGearChange(rs_path_.paths[0].gear);
  if (gear_switch) {
    gear_cost += config_.gear_switch_penalty;
    rs_node->AddGearSwitchNumber();
  }
  piecewise_cost += gear_cost;

  // steer cost
  if (current_node->GetSteer() > 0.0 && rs_path_.paths[0].steer != RS_LEFT) {
    steer_change_cost += config_.traj_steer_change_penalty * max_steer_angle_;
  } else if (current_node->GetSteer() < 0.0 &&
             rs_path_.paths[0].steer != RS_RIGHT) {
    steer_change_cost += config_.traj_steer_change_penalty * max_steer_angle_;
  }
  piecewise_cost += steer_change_cost;

  // request dist and gear cost
  if (current_node->GetDistToStart() <
      request_.first_action_request.dist_request) {
    if (current_node->IsPathGearChange(rs_path_.paths[0].gear)) {
      piecewise_cost += config_.expect_dist_penalty;
    } else if (current_node->GetDistToStart() +
                   std::fabs(rs_path_.paths[0].length) <
               request_.first_action_request.dist_request) {
      piecewise_cost += config_.expect_dist_penalty;
    }
  }

  rs_node->SetDistToStart(path_dist + current_node->GetDistToStart());

  if (current_node->GetGearSwitchNum() > 0) {
    rs_node->SetGearSwitchNode(current_node->GearSwitchNode());
  } else if (rs_node->GetGearSwitchNum() == 0) {
    rs_node->SetGearSwitchNode(nullptr);
  } else if (gear_switch) {
    // 正常节点没有换档，rs起点换档
    rs_node->SetGearSwitchNode(current_node);
  } else {
    // rs 起点没有换档
    rs_node->SetGearSwitchNode(nullptr);
  }

  return piecewise_cost;
}

void HybridAStar::GetSingleShotNodeGCost(Node3d* current_node,
                                         Node3d* next_node) {
  // evaluate cost on the trajectory and add current cost
  float piecewise_cost = 0.0;
  float path_dist = 0.0;
  path_dist = next_node->GetNodePathDistance();

  if (next_node->IsForward()) {
    piecewise_cost += path_dist * config_.traj_forward_penalty;
  } else {
    piecewise_cost += path_dist * config_.traj_reverse_penalty;
  }

  // steering wheel angle cost
  // for start node, steering angle can be any value
  if (!current_node->IsStartNode()) {
    piecewise_cost += 0.0 * std::fabs(next_node->GetSteer());

    // steering wheel change
    piecewise_cost +=
        1.0 * std::fabs(next_node->GetSteer() - current_node->GetSteer());
  }

  // ref line heading cost
  // float heading_cost =
  //     std::fabs(GetThetaDiff(next_node->GetPhi(), ref_line_.GetHeading())) *
  //     config_.ref_line_heading_penalty;
  // piecewise_cost += heading_cost;

  next_node->SetGCost(current_node->GetGCost() + piecewise_cost);

  return;
}

float HybridAStar::ObstacleHeuristicWithHolonomic(Node3d* next_node) {
  return dp_heuristic_generator_->CheckDpMap(next_node->GetX(),
                                             next_node->GetY());
}

float HybridAStar::GenerateHeuristicCost(Node3d* next_node) {
  float h_cost = 0.0;

  h_cost += ObstacleHeuristicWithHolonomic(next_node);

  return h_cost;
}

float HybridAStar::GenerateRefLineHeuristicCost(Node3d* next_node,
                                                 const float dist_to_go) {
  float dist_cost = 0.0;
  // if (0) {
  //   // position cost
  //   Vec2df32 point;
  //   point.set_x(next_node->GetX());
  //   point.set_y(next_node->GetY());

  //   Vec2df32 line = point - ref_line_.GetStartPoint();
  //   float lateral_dist = ref_line_.UnitDirection().CrossProd(line);

  //   dist_cost = std::fabs(lateral_dist) * config_.traj_forward_penalty;
  // }

  // heading cost
  float theta1 = next_node->GetPhi();
  float theta2 = ref_line_->GetHeading();

  float heading_cost_weight = 2.0;
  float heading_cost = dist_to_go + std::fabs(GetThetaDiff(theta1, theta2)) *
                                         heading_cost_weight;

  // float heading_cost = std::fabs(GetThetaDiff(theta1, theta2)) *
  //                       config_.ref_line_heading_penalty;

#if DEBUG_REF_LINE_COST
  ILOG_INFO << "node heading = " << next_node->GetPhi() * 57.3 << " dist cost "
            << dist_cost << " heading cost " << heading_cost
            << ", ref line heading =" << theta2 * 57.3;
#endif

  return std::max(dist_cost, heading_cost);
}

const bool HybridAStar::BackwardPassByRSPath(HybridAStarResult* result,
                                             Node3d* best_rs_node,
                                             const RSPath* rs_path) {
  Node3d* parent_node = nullptr;
  Node3d* child_node = best_rs_node;
  best_rs_node->SetNext(nullptr);

  result->base_pose = request_.base_pose_;
  result->gear_change_num = 0;

  // all nodes
  std::vector<Node3d*> node_list;

  ILOG_INFO << "get result start backward pass by rs";

  // backward pass
  while (child_node->GetPreNode() != nullptr) {
    parent_node = child_node->GetPreNode();

    parent_node->SetNext(child_node);

    child_node = parent_node;
  }

  size_t point_size;
  float kappa;

  AstarPathGear last_gear_type = AstarPathGear::NONE;
  AstarPathGear cur_gear_type;
  while (child_node != nullptr) {
    // break
    if (child_node->IsRsPath()) {
      break;
    }

    const NodePath& path = child_node->GetNodePath();

    AstarPathType path_type = child_node->GetPathType();
    cur_gear_type = child_node->GetGearType();

    // todo
    if (child_node->GetConstNextNode() == nullptr) {
      point_size = path.point_size;
    }
    // gear change
    else if (child_node->IsPathGearChange(
                 child_node->GetConstNextNode()->GetGearType())) {
      point_size = path.point_size;
    }
    // same gear
    else {
      // delete same point
      point_size = path.point_size - 1;
    }

    kappa = std::tan(child_node->GetSteer()) / vehicle_param_.wheel_base;

    for (size_t k = 0; k < point_size; k++) {
      result->x.emplace_back(path.points[k].x);
      result->y.emplace_back(path.points[k].y);
      result->phi.emplace_back(path.points[k].theta);
      result->type.emplace_back(path_type);
      result->gear.emplace_back(cur_gear_type);
      result->kappa.emplace_back(kappa);
    }

    // check gear switch number
    if (last_gear_type != AstarPathGear::NONE) {
      if (last_gear_type != cur_gear_type) {
        result->gear_change_num++;
      }
    }

    node_list.push_back(child_node);

    last_gear_type = cur_gear_type;
    parent_node = child_node;
    child_node = child_node->GetMutableNextNode();
  }

  // get rs path
  AstarPathType path_type = child_node->GetPathType();
  if (child_node != nullptr && child_node->IsRsPath()) {
    for (int seg_id = 0; seg_id < rs_path->size; seg_id++) {
      const RSPathSegment* segment = &rs_path->paths[seg_id];

      if (segment->size < 1) {
        ILOG_ERROR << "result size check failed";
        continue;
      }

      cur_gear_type = segment->gear;
      kappa = segment->kappa;
      for (int k = 0; k < segment->size; k++) {
        result->x.emplace_back(segment->points[k].x);
        result->y.emplace_back(segment->points[k].y);
        result->phi.emplace_back(segment->points[k].theta);
        result->type.emplace_back(path_type);
        result->gear.emplace_back(cur_gear_type);
        result->kappa.emplace_back(kappa);
      }

      // check gear switch number
      if (last_gear_type != AstarPathGear::NONE) {
        if (last_gear_type != cur_gear_type) {
          result->gear_change_num++;
        }
      }

      last_gear_type = cur_gear_type;
    }
  }

  size_t pt_size = result->x.size();
  if (pt_size != result->y.size() || pt_size != result->phi.size() ||
      pt_size != result->gear.size()) {
    ILOG_ERROR << "state sizes not equal, "
               << "result->x.size(): " << result->x.size() << "result->y.size()"
               << result->y.size() << "result->phi.size()"
               << result->phi.size();

    return false;
  }

  ReversePathBySwapStartGoal(result);

  // get path lengh
  size_t path_points_size = result->x.size();

  float accumulated_s = 0.0;
  result->accumulated_s.clear();
  auto last_x = result->x.front();
  auto last_y = result->y.front();
  float x_diff;
  float y_diff;
  for (size_t i = 0; i < path_points_size; ++i) {
    x_diff = result->x[i] - last_x;
    y_diff = result->y[i] - last_y;
    accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
    result->accumulated_s.push_back(accumulated_s);
    last_x = result->x[i];
    last_y = result->y[i];
  }

  ILOG_INFO << "get result finish, path point size " << result->x.size();

  result->fail_type = AstarFailType::SUCCESS;

  // DebugPathString(result);

#if DEBUG_SEARCH_RESULT
  ILOG_INFO << "path node num " << node_list.size();
  for (size_t i = 0; i < node_list.size(); i++) {
    ILOG_INFO << "node id " << i << " node steer "
              << node_list[i]->GetSteer() * 57.3 << " forward "
              << static_cast<int>(node_list[i]->GetGearType()) << " is rs path "
              << (node_list[i]->GetPathType() == AstarPathType::REEDS_SHEPP)
              << ", length: "
              << node_path_dist_resolution_ * node_list[i]->GetStepSize();
  }
#endif

  return true;
}

const bool HybridAStar::BackwardPassByPolynomialPath(
    HybridAStarResult* result, Node3d* poly_node,
    const std::vector<AStarPathPoint>& poly_path) {
  Node3d* parent_node = nullptr;
  Node3d* child_node = poly_node;

  result->base_pose = request_.base_pose_;
  result->gear_change_num = 0;

  ILOG_INFO << "get result backward pass by polynomial";
  poly_node->DebugPoseString();

  // DebugPolynomialPath(poly_path);

  // backward pass
  int i = 0;
  while (child_node->GetPreNode() != nullptr) {
    parent_node = child_node->GetPreNode();

    parent_node->SetNext(child_node);

    child_node = parent_node;
  }

  i = 0;
  size_t point_size;
  float kappa;

  AstarPathGear last_gear_type = AstarPathGear::NONE;
  AstarPathGear cur_gear_type;
  while (child_node != nullptr) {
    // break
    if (child_node->IsQunticPolynomialPath()) {
      break;
    }

    const NodePath& path = child_node->GetNodePath();

    AstarPathType path_type = child_node->GetPathType();
    cur_gear_type = child_node->GetGearType();

    // todo
    if (child_node->GetConstNextNode() == nullptr) {
      point_size = path.point_size;
    }
    // gear change
    else if (child_node->IsPathGearChange(
                 child_node->GetConstNextNode()->GetGearType())) {
      point_size = path.point_size;
    }
    // same gear
    else {
      // delete same point
      point_size = path.point_size - 1;
    }

    kappa = std::tan(child_node->GetSteer()) / vehicle_param_.wheel_base;

    for (size_t k = 0; k < point_size; k++) {
      result->x.emplace_back(path.points[k].x);
      result->y.emplace_back(path.points[k].y);
      result->phi.emplace_back(path.points[k].theta);
      result->type.emplace_back(path_type);
      result->gear.emplace_back(cur_gear_type);
      result->kappa.emplace_back(kappa);
    }

    // check gear switch number
    if (last_gear_type != AstarPathGear::NONE) {
      if (last_gear_type != cur_gear_type) {
        result->gear_change_num++;
      }
    }

    last_gear_type = cur_gear_type;
    parent_node = child_node;
    child_node = child_node->GetMutableNextNode();
  }

  // get path
  if (child_node != nullptr && child_node->IsQunticPolynomialPath()) {
    if (poly_path.size() < 1) {
      return false;
    }

    AstarPathType path_type = child_node->GetPathType();

    for (size_t k = 0; k < poly_path.size(); k++) {
      result->x.emplace_back(poly_path[k].x);
      result->y.emplace_back(poly_path[k].y);
      result->phi.emplace_back(poly_path[k].phi);
      result->type.emplace_back(path_type);
      result->gear.emplace_back(poly_path[k].gear);
      result->kappa.emplace_back(poly_path[k].kappa);
    }
  }

  size_t pt_size = result->x.size();
  if (pt_size != result->y.size() || pt_size != result->phi.size() ||
      pt_size != result->gear.size()) {
    ILOG_ERROR << "state sizes not equal, "
               << "result->x.size(): " << result->x.size() << "result->y.size()"
               << result->y.size() << "result->phi.size()"
               << result->phi.size();

    return false;
  }

  // get path lengh
  size_t path_points_size = result->x.size();

  float accumulated_s = 0.0;
  result->accumulated_s.clear();
  auto last_x = result->x.front();
  auto last_y = result->y.front();
  float x_diff;
  float y_diff;
  for (size_t i = 0; i < path_points_size; ++i) {
    x_diff = result->x[i] - last_x;
    y_diff = result->y[i] - last_y;
    accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
    result->accumulated_s.push_back(accumulated_s);
    last_x = result->x[i];
    last_y = result->y[i];
  }

  ILOG_INFO << "get result finish, path point size " << result->x.size();

  result->fail_type = AstarFailType::SUCCESS;

#if DEBUG_SEARCH_RESULT
  ILOG_INFO << "path node num " << node_list.size();
  for (size_t i = 0; i < node_list.size(); i++) {
    ILOG_INFO << "node id " << i << " node steer "
              << node_list[i]->GetSteer() * 57.3 << " forward "
              << static_cast<int>(node_list[i]->GetGearType()) << " is rs path "
              << (node_list[i]->GetPathType() == AstarPathType::REEDS_SHEPP)
              << ", length: "
              << node_path_dist_resolution_ * node_list[i]->GetStepSize();
  }
#endif

  return true;
}

void HybridAStar::LinkRsToAstarEndPoint(HybridAStarResult* result,
                                        const Pose2D& astar_end) {
  if (result->x.size() < 1) {
    ILOG_INFO << "no path";
    return;
  }

  Eigen::Vector2f rs_end_point;

  rs_end_point[0] = result->x.back();
  rs_end_point[1] = result->y.back();

  float x_diff = std::fabs(rs_end_point[0] - astar_end.x);
  float y_diff = std::fabs(rs_end_point[1] - astar_end.y);
  float length = std::sqrt(x_diff * x_diff + y_diff * y_diff);

  float phi = result->phi.back();
  AstarPathGear gear = result->gear.back();
  AstarPathType path_type = AstarPathType::LINE_SEGMENT;

  const Eigen::Vector2f unit_line_vec = Eigen::Vector2f(-1.0, 0.0);
  float s = 0.1;
  float ds = 0.1;

  Eigen::Vector2f point;
  while (s < length) {
    point = rs_end_point + s * unit_line_vec;
    result->x.emplace_back(point[0]);
    result->y.emplace_back(point[1]);
    result->phi.emplace_back(phi);
    result->gear.emplace_back(gear);
    result->type.emplace_back(path_type);
    result->kappa.emplace_back(0.0);

    s += ds;
  }

  // add astar end
  result->x.emplace_back(astar_end.x);
  result->y.emplace_back(astar_end.y);
  result->phi.emplace_back(astar_end.theta);
  result->gear.emplace_back(gear);
  result->type.emplace_back(path_type);
  result->kappa.emplace_back(0.0);

  return;
}

void HybridAStar::GearRerversePathAttempt(
    const MapBound& XYbounds, const ParkObstacleList& obstacles,
    const AstarRequest& request, const ObstacleClearZone* clear_zone,
    const Pose2D& start, const Pose2D& target, HybridAStarResult* result,
    EulerDistanceTransform* edt, ParkReferenceLine* ref_line) {
  result->Clear();

  if (request.first_action_request.gear_request == AstarPathGear::DRIVE) {
    ILOG_INFO << "gear is drive";
    return;
  }

  if (request.direction_request != ParkingVehDirection::TAIL_IN) {
    return;
  }

  if (start.GetX() < 1.0) {
    ILOG_INFO << "start.GetX() =" << start.GetX();
    return;
  }

  if (start.GetY() < -5.0 || start.GetY() > 5.0) {
    ILOG_INFO << "start.GetY() =" << start.GetY();
    return;
  }

  float heading = IflyUnifyTheta(start.GetPhi(), M_PI);
  if (std::fabs(heading) > (M_PI_2 + 0.001)) {
    ILOG_INFO << "start.GetPhi() =" << heading * 57.4;
    return;
  }

  // clear containers
  ResetNodePool();
  open_pq_.clear();
  node_set_.clear();

  // debug
  child_node_debug_.clear();
  queue_path_debug_.clear();
  rs_path_h_cost_debug_.clear();
  rs_path_.Clear();

  collision_check_time_ms_ = 0.0;
  rs_time_ms_ = 0.0;
  rs_interpolate_time_ms_ = 0.0;
  ILOG_INFO << "gear reverse search begin";

  obstacles_ = &obstacles;
  edt_ = edt;
  request_ = request;
  ref_line_ = ref_line;

  // load XYbounds
  XYbounds_ = XYbounds;
  clear_zone_ = clear_zone;

  // check bound
  NodeGridIndex grid_index;
  Node3d::CoordinateToGridIndex(XYbounds_.x_max, XYbounds_.y_max, M_PI,
                                &grid_index, XYbounds_, config_);

  if (!NodeInSearchBound(grid_index)) {
    ILOG_ERROR << "search bound size too big, please change range "
               << grid_index.x << " " << grid_index.y << " " << grid_index.phi;

    result->fail_type = AstarFailType::OUT_OF_BOUND;
    return;
  }

  // start
  start_node_ = node_pool_.AllocateNode();

  // load nodes and obstacles
  if (start_node_ == nullptr) {
    ILOG_ERROR << "start node nullptr";

    result->fail_type = AstarFailType::ALLOCATE_NODE_FAIL;
    return;
  }

  start_node_->Set(NodePath(start), XYbounds_, config_, 0.0);
  if (!start_node_->IsNodeValid()) {
    ILOG_ERROR << "start_node invalid";

    result->fail_type = AstarFailType::OUT_OF_BOUND;
    return;
  }

  // in searching, start node gear is forward or backward which will be ok.
  start_node_->SetGearType(AstarPathGear::NONE);
  start_node_->SetSteer(0.0);
  start_node_->SetIsStartNode(true);
  start_node_->SetPathType(AstarPathType::START_NODE);
  start_node_->SetDistToStart(0.0);
  start_node_->SetGearSwitchNum(0);
  start_node_->DebugString();
  // start node gcost is 0

  // check start
  double check_start_time = IflyTime::Now_ms();
  if (!ValidityCheckByEDT(start_node_)) {
    // double check
    if (IsFootPrintCollision(Transform2d(start_node_->GetPose()))) {
      ILOG_ERROR << "start_node in collision with obstacles "
                 << static_cast<int>(start_node_->GetConstCollisionType());

      // start_node_->DebugString();
      result->fail_type = AstarFailType::START_COLLISION;
      return;
    }
  }
  double check_end_time = IflyTime::Now_ms();
  collision_check_time_ms_ += check_end_time - check_start_time;

  // allocate end
  astar_end_node_ = node_pool_.AllocateNode();
  if (astar_end_node_ == nullptr) {
    ILOG_ERROR << "end node nullptr";

    result->fail_type = AstarFailType::OUT_OF_BOUND;
    return;
  }
  astar_end_node_->Set(NodePath(target), XYbounds_, config_, 0.0);
  astar_end_node_->SetGearType(AstarPathGear::NONE);
  astar_end_node_->SetPathType(AstarPathType::END_NODE);
  astar_end_node_->DebugString();

  if (!astar_end_node_->IsNodeValid()) {
    ILOG_ERROR << "end_node invalid";

    result->fail_type = AstarFailType::OUT_OF_BOUND;
    return;
  }

  // check end
  check_start_time = IflyTime::Now_ms();
  if (!ValidityCheckByEDT(astar_end_node_)) {
    ILOG_INFO << "end_node in collision with obstacles "
              << static_cast<int>(astar_end_node_->GetConstCollisionType());

    astar_end_node_->DebugString();
    result->fail_type = AstarFailType::GOAL_COLLISION;
    return;
  }

  check_end_time = IflyTime::Now_ms();
  collision_check_time_ms_ += check_end_time - check_start_time;

  // node shrink related
  node_shrink_decider_.Process(start, target, request_.direction_request,
                               request_.real_goal, XYbounds_);
  rs_expansion_decider_.Process(
      vehicle_param_.min_turn_radius, request_.slot_width, request_.slot_length,
      start, target, vehicle_param_.width, request_.space_type,
      request_.direction_request);

  // load open set, pq
  start_node_->SetMultiMapIter(
      open_pq_.insert(std::make_pair(0.0, start_node_)));

  node_set_.emplace(start_node_->GetGlobalID(), start_node_);

  // a star searching
  size_t explored_node_num = 0;
  size_t explored_rs_path_num = 0;
  size_t h_cost_rs_path_num = 0;
  double astar_search_start_time = IflyTime::Now_ms();
  double astar_search_time;
  heuristic_time_ = 0.0;

  Node3d* current_node = nullptr;
  Node3d* next_node_in_pool = nullptr;
  Node3d new_node;
  Node3d rs_node_to_goal;
  rs_node_to_goal.Clear();

  AstarNodeVisitedType vis_type;
  PathGearRequest gear_request = PathGearRequest::GEAR_REVERSE_ONLY;
  bool is_safe = false;
  float child_node_dist;
  float father_node_dist;
  NodeShrinkType node_shrink_type;
  std::vector<AStarPathPoint> poly_path;
  Node3d polynomial_node;
  PolynomialPathErrorCode poly_path_fail_type;
  polynomial_node.Clear();

  while (!open_pq_.empty()) {
    // take out the lowest cost neighboring node

    current_node = open_pq_.begin()->second;
    open_pq_.erase(open_pq_.begin());
    if (current_node == nullptr) {
      ILOG_INFO << "pq is null node";
      continue;
    }

    current_node->SetVisitedType(AstarNodeVisitedType::IN_CLOSE);

#if DEBUG_ONE_SHOT_PATH
    if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
      ILOG_INFO << "****************************";
      ILOG_INFO << "cycle, explored_node_num " << explored_node_num
                << " open set size for now " << open_pq_.size();
      current_node->DebugString();
    }
#endif

#if PLOT_SEARCH_SEQUENCE
    queue_path_debug_.emplace_back(
        Vec2df32(current_node->GetX(), current_node->GetY()));
#endif

    // check if an analystic curve could be connected from current
    // configuration to the end configuration without collision. if so,
    // search ends.
    double current_time = IflyTime::Now_ms();

    // if bigger than 100 ms，break
    astar_search_time = current_time - astar_search_start_time;
    if (astar_search_time > config_.max_search_time_ms_for_no_gear_switch) {
      ILOG_INFO << "time out " << astar_search_time;
      break;
    }

    if (SamplingByQunticPolynomial(current_node, poly_path, &polynomial_node,
                                   &poly_path_fail_type)) {
      ILOG_INFO << "polynomial success";

      current_node->DebugPoseString();

      ILOG_INFO << "path " << poly_path[0].x << ",y " << poly_path[0].y
                << ",heading = " << poly_path[0].phi * 57.4
                << ",radius = " << current_node->GetRadius();

      break;
    } else if (SamplingByRSPath(current_node, &rs_node_to_goal)) {
      ILOG_INFO << "RS success";
      break;
    }

    explored_rs_path_num++;

    for (size_t i = 0; i < next_node_num_; ++i) {
      node_shrink_type =
          NextNodeGenerator(&new_node, current_node, i, gear_request);
      explored_node_num++;

      if (!new_node.IsNodeValid()) {
        continue;
      }

      child_node_dist = new_node.DistToPose(astar_end_node_->GetPose());
      father_node_dist = current_node->DistToPose(astar_end_node_->GetPose());

      // dist is bigger
      if (child_node_dist > father_node_dist - 0.001) {
        continue;
      }

      if (node_shrink_decider_.IsShrinkByStartNode(start_node_->GetGlobalID(),
                                                   &new_node)) {
        continue;
      }

      if (node_shrink_decider_.IsShrinkByParent(current_node, &new_node)) {
        continue;
      }

      // collision check
#if LOG_TIME_PROFILE
      check_start_time = IflyTime::Now_ms();
#endif

      is_safe = ValidityCheckByEDT(&new_node);

#if LOG_TIME_PROFILE
      check_end_time = IflyTime::Now_ms();
      collision_check_time_ms_ += check_end_time - check_start_time;
#endif

      if (!is_safe) {
#if PLOT_CHILD_NODE
        // if node is unsafe, plot it also.
        if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
          child_node_debug_.emplace_back(
              DebugAstarSearchPoint(new_node.GetX(), new_node.GetY(), false));
        }
#endif

        continue;
      }

#if DEBUG_ONE_SHOT_PATH
      if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
        ILOG_INFO << "  ======================== ";
        ILOG_INFO << "  search child node cycle, open set size "
                  << open_pq_.size()
                  << " ,gear change num:" << current_node->GetGearSwitchNum();
        current_node->DebugString();
        new_node.DebugString();
      }
#endif

      // allocate new node
      if (node_set_.find(new_node.GetGlobalID()) == node_set_.end()) {
        next_node_in_pool = node_pool_.AllocateNode();

        vis_type = AstarNodeVisitedType::NOT_VISITED;
      } else {
        next_node_in_pool = node_set_[new_node.GetGlobalID()];

        vis_type = next_node_in_pool->GetVisitedType();
      }

      // check null
      if (next_node_in_pool == nullptr) {
        continue;
      }

      GetSingleShotNodeGCost(current_node, &new_node);

#if DEBUG_ONE_SHOT_PATH
      if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
        ILOG_INFO << "  vis type " << static_cast<int>(vis_type)
                  << " new node g " << new_node.GetGCost() << " pool node g "
                  << next_node_in_pool->GetGCost()
                  << " ,safe dist: " << new_node.GetDistToObs();
      }
#endif

      // find a new node
      if (vis_type == AstarNodeVisitedType::NOT_VISITED) {
        GetSingleShotNodeHeuCost(current_node, &new_node);

        h_cost_rs_path_num++;

        *next_node_in_pool = new_node;
        next_node_in_pool->SetFCost();
        next_node_in_pool->SetVisitedType(AstarNodeVisitedType::IN_OPEN);

        next_node_in_pool->SetMultiMapIter(open_pq_.insert(
            std::make_pair(next_node_in_pool->GetFCost(), next_node_in_pool)));

        node_set_.emplace(next_node_in_pool->GetGlobalID(), next_node_in_pool);

#if DEBUG_ONE_SHOT_PATH
        if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
          ILOG_INFO << "new point";
          next_node_in_pool->DebugCost();
        }
#endif

      } else if (vis_type == AstarNodeVisitedType::IN_OPEN) {
        // in open set and need update
        if (new_node.GetGCost() < next_node_in_pool->GetGCost()) {
          GetSingleShotNodeHeuCost(current_node, &new_node);

          h_cost_rs_path_num++;

          open_pq_.erase(next_node_in_pool->GetMultiMapIter());

          *next_node_in_pool = new_node;
          next_node_in_pool->SetFCost();
          next_node_in_pool->SetVisitedType(AstarNodeVisitedType::IN_OPEN);

          // put node in open set again and record it.
          next_node_in_pool->SetMultiMapIter(open_pq_.insert(std::make_pair(
              next_node_in_pool->GetFCost(), next_node_in_pool)));

#if DEBUG_ONE_SHOT_PATH
          if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
            ILOG_INFO << "node has in open, open set size= " << open_pq_.size();
            next_node_in_pool->DebugCost();
          }
#endif
        }
      } else {
        // in close set and need update
        if (new_node.GetGCost() < next_node_in_pool->GetGCost() + 1e-1) {
          if (node_shrink_decider_.IsLoopBackNode(&new_node,
                                                  next_node_in_pool)) {
            continue;
          }

          if (!node_shrink_decider_.IsSameGridNodeContinuous(
                  &new_node, next_node_in_pool)) {
            continue;
          }

          GetSingleShotNodeHeuCost(current_node, &new_node);

          h_cost_rs_path_num++;

          *next_node_in_pool = new_node;
          next_node_in_pool->SetFCost();
          next_node_in_pool->SetVisitedType(AstarNodeVisitedType::IN_OPEN);

          // put node in open set again and record it.
          next_node_in_pool->SetMultiMapIter(open_pq_.insert(std::make_pair(
              next_node_in_pool->GetFCost(), next_node_in_pool)));

#if DEBUG_ONE_SHOT_PATH
          if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
            ILOG_INFO << "node has in close";
            next_node_in_pool->DebugCost();
          }
#endif
        }
      }

#if PLOT_CHILD_NODE
      if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
        child_node_debug_.emplace_back(
            DebugAstarSearchPoint(new_node.GetX(), new_node.GetY(), true));
      }
#endif
    }

    // search neighbor nodes  over
  }

  // todo, use all astar node, maybe no need use rs path.
  if (polynomial_node.IsNodeValid()) {
    BackwardPassByPolynomialPath(result, &polynomial_node, poly_path);
  } else if (rs_node_to_goal.IsNodeValid()) {
    BackwardPassByRSPath(result, &rs_node_to_goal, &rs_path_);
  } else {
    result->fail_type = AstarFailType::SEARCH_TOO_MUCH_NODE;
  }

  ILOG_INFO << "explored node num is " << explored_node_num
            << " ,rs path size is: " << explored_rs_path_num
            << " ,h cost rs num " << h_cost_rs_path_num
            << " ,node pool size:" << node_pool_.PoolSize()
            << " ,open_pq_.size=" << open_pq_.size();

  ILOG_INFO << "heuristic time " << heuristic_time_ << " ,rs params time "
            << rs_time_ms_ << ",rs interpolate time:" << rs_interpolate_time_ms_
            << " ,collision time " << collision_check_time_ms_
            << ",search fail=" << static_cast<int>(result->fail_type)
            << "hybrid astar search time (ms)= " << astar_search_time;

  double astar_end_time = IflyTime::Now_ms();
  result->time_ms = astar_end_time - check_start_time;
  ILOG_INFO << "reverse search total time (ms) = " << result->time_ms;

  return;
}

void HybridAStar::GearDrivePathAttempt(
    const MapBound& XYbounds, const ParkObstacleList& obstacles,
    const AstarRequest& request, const ObstacleClearZone* clear_zone,
    const Pose2D& start, const Pose2D& target, HybridAStarResult* result,
    EulerDistanceTransform* edt, ParkReferenceLine* ref_line) {
  result->Clear();
  if (request.direction_request != ParkingVehDirection::HEAD_IN) {
    return;
  }

  if (request.first_action_request.gear_request == AstarPathGear::REVERSE) {
    ILOG_INFO << "gear is reverse";
    return;
  }

  float heading = IflyUnifyTheta(start.GetPhi(), M_PI);
  if (std::fabs(heading) < (M_PI_2 - 0.001)) {
    ILOG_INFO << "start.GetPhi() =" << heading * 57.4;
    return;
  }

  if (start.GetX() < 6.0) {
    ILOG_INFO << "start.GetX() =" << start.GetX();
    return;
  }

  if (start.GetY() < -3.8 || start.GetY() > 3.8) {
    ILOG_INFO << "start.GetY() =" << start.GetY();
    return;
  }

  // clear containers
  ResetNodePool();
  open_pq_.clear();
  node_set_.clear();

  // debug
  child_node_debug_.clear();
  queue_path_debug_.clear();
  delete_queue_path_debug_.clear();
  rs_path_h_cost_debug_.clear();
  rs_path_.Clear();

  collision_check_time_ms_ = 0.0;
  rs_time_ms_ = 0.0;
  rs_interpolate_time_ms_ = 0.0;
  ILOG_INFO << "gear drive searching begin";

  obstacles_ = &obstacles;
  edt_ = edt;
  request_ = request;
  ref_line_ = ref_line;

  // load XYbounds
  XYbounds_ = XYbounds;
  clear_zone_ = clear_zone;

  // check bound
  NodeGridIndex grid_index;
  Node3d::CoordinateToGridIndex(XYbounds_.x_max, XYbounds_.y_max, M_PI,
                                &grid_index, XYbounds_, config_);

  if (!NodeInSearchBound(grid_index)) {
    ILOG_ERROR << "search bound size too big, please change range "
               << grid_index.x << " " << grid_index.y << " " << grid_index.phi;

    result->fail_type = AstarFailType::OUT_OF_BOUND;
    return;
  }

  // start
  start_node_ = node_pool_.AllocateNode();

  // load nodes and obstacles
  if (start_node_ == nullptr) {
    ILOG_ERROR << "start node nullptr";

    result->fail_type = AstarFailType::ALLOCATE_NODE_FAIL;
    return;
  }

  start_node_->Set(NodePath(start), XYbounds_, config_, 0.0);
  if (!start_node_->IsNodeValid()) {
    ILOG_ERROR << "start_node invalid";

    result->fail_type = AstarFailType::OUT_OF_BOUND;
    return;
  }

  // in searching, start node gear is forward or backward which will be ok.
  start_node_->SetGearType(AstarPathGear::NONE);
  start_node_->SetSteer(0.0);
  start_node_->SetIsStartNode(true);
  start_node_->SetPathType(AstarPathType::START_NODE);
  start_node_->SetDistToStart(0.0);
  start_node_->SetGearSwitchNum(0);
  start_node_->DebugString();
  // start node gcost is 0

  // check start
  double check_start_time = IflyTime::Now_ms();
  if (!ValidityCheckByEDT(start_node_)) {
    // double check
    if (IsFootPrintCollision(Transform2d(start_node_->GetPose()))) {
      ILOG_ERROR << "start_node in collision with obstacles "
                 << static_cast<int>(start_node_->GetConstCollisionType());

      // start_node_->DebugString();
      result->fail_type = AstarFailType::START_COLLISION;
      return;
    }
  }

  double check_end_time = IflyTime::Now_ms();
  collision_check_time_ms_ += check_end_time - check_start_time;

  // allocate end
  astar_end_node_ = node_pool_.AllocateNode();
  if (astar_end_node_ == nullptr) {
    ILOG_ERROR << "end node nullptr";

    result->fail_type = AstarFailType::OUT_OF_BOUND;
    return;
  }
  astar_end_node_->Set(NodePath(target), XYbounds_, config_, 0.0);
  astar_end_node_->SetGearType(AstarPathGear::NONE);
  astar_end_node_->SetPathType(AstarPathType::END_NODE);
  astar_end_node_->DebugString();

  if (!astar_end_node_->IsNodeValid()) {
    ILOG_ERROR << "end_node invalid";

    result->fail_type = AstarFailType::OUT_OF_BOUND;
    return;
  }

  // check end
  check_start_time = IflyTime::Now_ms();
  if (!ValidityCheckByEDT(astar_end_node_)) {
    ILOG_INFO << "end_node in collision with obstacles "
              << static_cast<int>(astar_end_node_->GetConstCollisionType());

    astar_end_node_->DebugString();
    result->fail_type = AstarFailType::GOAL_COLLISION;
    return;
  }

  check_end_time = IflyTime::Now_ms();
  collision_check_time_ms_ += check_end_time - check_start_time;

  // node shrink related
  node_shrink_decider_.Process(start, target, request_.direction_request,
                               request_.real_goal, XYbounds_);
  rs_expansion_decider_.Process(
      vehicle_param_.min_turn_radius, request_.slot_width, request_.slot_length,
      start, target, vehicle_param_.width, request_.space_type,
      request_.direction_request);

  // load open set, pq
  start_node_->SetMultiMapIter(
      open_pq_.insert(std::make_pair(0.0, start_node_)));

  node_set_.emplace(start_node_->GetGlobalID(), start_node_);

  // a star searching
  size_t explored_node_num = 0;
  size_t explored_rs_path_num = 0;
  size_t h_cost_rs_path_num = 0;
  double astar_search_start_time = IflyTime::Now_ms();
  double astar_search_time;
  heuristic_time_ = 0.0;

  Node3d* current_node = nullptr;
  Node3d* next_node_in_pool = nullptr;
  Node3d* best_node = nullptr;
  Node3d new_node;
  Node3d rs_node_to_goal;
  rs_node_to_goal.Clear();

  AstarNodeVisitedType vis_type;
  PathGearRequest gear_request = PathGearRequest::GEAR_DRIVE_ONLY;
  bool is_safe = false;
  float child_node_dist;
  float father_node_dist;
  NodeShrinkType node_shrink_type;
  std::vector<AStarPathPoint> poly_path;
  Node3d polynomial_node;
  PolynomialPathErrorCode poly_path_fail_type;
  polynomial_node.Clear();

  PathComparator path_comparator;

  while (!open_pq_.empty()) {
    // take out the lowest cost neighboring node

    current_node = open_pq_.begin()->second;
    open_pq_.erase(open_pq_.begin());
    if (current_node == nullptr) {
      ILOG_INFO << "pq is null node";
      continue;
    }

    current_node->SetVisitedType(AstarNodeVisitedType::IN_CLOSE);

    // generate best node
    if (best_node == nullptr) {
      best_node = current_node;
    } else if (path_comparator.NodeCompare(request_.real_goal, best_node,
                                           current_node)) {
      best_node = current_node;
    }

#if DEBUG_ONE_SHOT_PATH
    if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
      ILOG_INFO << "****************************";
      ILOG_INFO << "cycle, explored_node_num " << explored_node_num
                << " open set size for now " << open_pq_.size();
      current_node->DebugString();
    }
#endif

#if PLOT_SEARCH_SEQUENCE
    queue_path_debug_.emplace_back(
        Vec2df32(current_node->GetX(), current_node->GetY()));
#endif

    // check if an analystic curve could be connected from current
    // configuration to the end configuration without collision. if so,
    // search ends.
    double current_time = IflyTime::Now_ms();

    // if bigger than 100 ms，break
    astar_search_time = current_time - astar_search_start_time;
    if (astar_search_time > config_.max_search_time_ms_for_no_gear_switch) {
      ILOG_INFO << "time out " << astar_search_time;
      break;
    }

    if (SamplingByQunticPolynomial(current_node, poly_path, &polynomial_node,
                                   &poly_path_fail_type)) {
      ILOG_INFO << "polynomial success";

      current_node->DebugPoseString();

      ILOG_INFO << "path " << poly_path[0].x << ",y " << poly_path[0].y
                << ",heading = " << poly_path[0].phi * 57.4
                << ",radius = " << current_node->GetRadius();

      break;
    } else if (SamplingByRSPath(current_node, &rs_node_to_goal)) {
      ILOG_INFO << "RS success";
      break;
    }

    explored_rs_path_num++;

    for (size_t i = 0; i < next_node_num_; ++i) {
      node_shrink_type =
          NextNodeGenerator(&new_node, current_node, i, gear_request);
      explored_node_num++;

      if (!new_node.IsNodeValid()) {
        continue;
      }

      child_node_dist = new_node.DistToPose(request_.real_goal);
      father_node_dist = current_node->DistToPose(request_.real_goal);

      // dist is bigger
      if (child_node_dist > father_node_dist - 0.001) {
        continue;
      }

      if (node_shrink_decider_.IsShrinkByStartNode(start_node_->GetGlobalID(),
                                                   &new_node)) {
        continue;
      }

      if (node_shrink_decider_.IsShrinkByParent(current_node, &new_node)) {
        continue;
      }

      // collision check
#if LOG_TIME_PROFILE
      check_start_time = IflyTime::Now_ms();
#endif

      is_safe = ValidityCheckByEDT(&new_node);

#if LOG_TIME_PROFILE
      check_end_time = IflyTime::Now_ms();
      collision_check_time_ms_ += check_end_time - check_start_time;
#endif

      if (!is_safe) {
#if PLOT_CHILD_NODE
        // if node is unsafe, plot it also.
        if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
          child_node_debug_.emplace_back(
              DebugAstarSearchPoint(new_node.GetX(), new_node.GetY(), false));
        }
#endif

        continue;
      }

#if DEBUG_ONE_SHOT_PATH
      if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
        ILOG_INFO << "  ======================== ";
        ILOG_INFO << "  search child node cycle, open set size "
                  << open_pq_.size()
                  << " ,gear change num:" << current_node->GetGearSwitchNum();
        current_node->DebugString();
        new_node.DebugString();
      }
#endif

      // allocate new node
      if (node_set_.find(new_node.GetGlobalID()) == node_set_.end()) {
        next_node_in_pool = node_pool_.AllocateNode();

        vis_type = AstarNodeVisitedType::NOT_VISITED;
      } else {
        next_node_in_pool = node_set_[new_node.GetGlobalID()];

        vis_type = next_node_in_pool->GetVisitedType();
      }

      // check null
      if (next_node_in_pool == nullptr) {
        continue;
      }

      GetSingleShotNodeGCost(current_node, &new_node);

#if DEBUG_ONE_SHOT_PATH
      if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
        ILOG_INFO << "  vis type " << static_cast<int>(vis_type)
                  << " new node g " << new_node.GetGCost() << " pool node g "
                  << next_node_in_pool->GetGCost()
                  << " ,safe dist: " << new_node.GetDistToObs();
      }
#endif

      // find a new node
      if (vis_type == AstarNodeVisitedType::NOT_VISITED) {
        GetSingleShotNodeHeuCost(current_node, &new_node);

        h_cost_rs_path_num++;

        // next_node_in_pool->CopyNode(&new_node);
        *next_node_in_pool = new_node;
        next_node_in_pool->SetFCost();
        next_node_in_pool->SetVisitedType(AstarNodeVisitedType::IN_OPEN);

        next_node_in_pool->SetMultiMapIter(open_pq_.insert(
            std::make_pair(next_node_in_pool->GetFCost(), next_node_in_pool)));

        node_set_.emplace(next_node_in_pool->GetGlobalID(), next_node_in_pool);

#if DEBUG_ONE_SHOT_PATH
        if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
          ILOG_INFO << "new point";
          next_node_in_pool->DebugCost();
        }
#endif

      } else if (vis_type == AstarNodeVisitedType::IN_OPEN) {
        // in open set and need update
        if (new_node.GetGCost() < next_node_in_pool->GetGCost()) {
          GetSingleShotNodeHeuCost(current_node, &new_node);

          h_cost_rs_path_num++;

          open_pq_.erase(next_node_in_pool->GetMultiMapIter());

          *next_node_in_pool = new_node;
          next_node_in_pool->SetFCost();
          next_node_in_pool->SetVisitedType(AstarNodeVisitedType::IN_OPEN);

          // put node in open set again and record it.
          next_node_in_pool->SetMultiMapIter(open_pq_.insert(std::make_pair(
              next_node_in_pool->GetFCost(), next_node_in_pool)));

#if DEBUG_ONE_SHOT_PATH
          if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
            ILOG_INFO << "node has in open, open set size= " << open_pq_.size();
            next_node_in_pool->DebugCost();
          }
#endif
        }
      } else {
        // in close set and need update
        if (new_node.GetGCost() < next_node_in_pool->GetGCost() + 1e-1) {
          if (node_shrink_decider_.IsLoopBackNode(&new_node,
                                                  next_node_in_pool)) {
            continue;
          }

          if (!node_shrink_decider_.IsSameGridNodeContinuous(
                  &new_node, next_node_in_pool)) {
            continue;
          }

          GetSingleShotNodeHeuCost(current_node, &new_node);

          h_cost_rs_path_num++;

          *next_node_in_pool = new_node;
          next_node_in_pool->SetFCost();
          next_node_in_pool->SetVisitedType(AstarNodeVisitedType::IN_OPEN);

          // put node in open set again and record it.
          next_node_in_pool->SetMultiMapIter(open_pq_.insert(std::make_pair(
              next_node_in_pool->GetFCost(), next_node_in_pool)));

#if DEBUG_ONE_SHOT_PATH
          if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
            ILOG_INFO << "node has in close";
            next_node_in_pool->DebugCost();
          }
#endif
        }
      }

#if PLOT_CHILD_NODE
      if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
        child_node_debug_.emplace_back(
            DebugAstarSearchPoint(new_node.GetX(), new_node.GetY(), true));
      }
#endif
    }

    // search neighbor nodes  over
  }

  // todo, use all astar node, maybe no need use rs path.
  if (polynomial_node.IsNodeValid()) {
    BackwardPassByPolynomialPath(result, &polynomial_node, poly_path);
  } else if (rs_node_to_goal.IsNodeValid()) {
    BackwardPassByRSPath(result, &rs_node_to_goal, &rs_path_);
  } else if (BestNodeIsNice(best_node)) {
    BackwardPassByNode(result, best_node);
  } else {
    result->fail_type = AstarFailType::SEARCH_TOO_MUCH_NODE;
  }

  ILOG_INFO << "explored node num is " << explored_node_num
            << " ,rs path size is: " << explored_rs_path_num
            << " ,h cost rs num " << h_cost_rs_path_num
            << " ,node pool size:" << node_pool_.PoolSize()
            << " ,open_pq_.size=" << open_pq_.size();

  ILOG_INFO << "heuristic time " << heuristic_time_ << " ,rs params time "
            << rs_time_ms_ << ",rs interpolate time:" << rs_interpolate_time_ms_
            << " ,collision time " << collision_check_time_ms_
            << ", search fail= " << static_cast<int>(result->fail_type)
            << ", hybrid astar search time (ms)= " << astar_search_time;

  double astar_end_time = IflyTime::Now_ms();
  result->time_ms = astar_end_time - check_start_time;
  ILOG_INFO << "hybrid astar total time (ms) = " << result->time_ms;

  return;
}

bool HybridAStar::AstarSearch(
    const Pose2D& start, const Pose2D& end, const MapBound& XYbounds,
    const ParkObstacleList& obstacles, const AstarRequest& request,
    const ObstacleClearZone* clear_zone, HybridAStarResult* result,
    EulerDistanceTransform* edt, ParkReferenceLine* ref_line) {
  double astar_start_time = IflyTime::Now_ms();

  // clear containers
  ResetNodePool();
  open_pq_.clear();
  node_set_.clear();

  // debug
  child_node_debug_.clear();
  queue_path_debug_.clear();
  delete_queue_path_debug_.clear();
  rs_path_h_cost_debug_.clear();
  rs_path_.Clear();
  result->Clear();

  collision_check_time_ms_ = 0.0;
  rs_time_ms_ = 0.0;
  rs_interpolate_time_ms_ = 0.0;

  ILOG_INFO << "hybrid astar begin";

  obstacles_ = &obstacles;
  edt_ = edt;
  ref_line_ = ref_line;

  DebugObstacleString();

  request_ = request;

  // load XYbounds
  XYbounds_ = XYbounds;
  // ILOG_INFO << "map bound, xmin " << XYbounds_.x_min << " , ymin "
  //           << XYbounds_.y_min << " ,xmax " << XYbounds_.x_max << " , ymax "
  //           << XYbounds_.y_max;

  clear_zone_ = clear_zone;

  // check bound
  NodeGridIndex grid_index;

  Node3d::CoordinateToGridIndex(XYbounds_.x_max, XYbounds_.y_max, M_PI,
                                &grid_index, XYbounds_, config_);

  if (!NodeInSearchBound(grid_index)) {
    ILOG_ERROR << "search bound size too big, please change range "
               << grid_index.x << " " << grid_index.y << " " << grid_index.phi;

    result->fail_type = AstarFailType::OUT_OF_BOUND;
    return false;
  }

  // start
  start_node_ = node_pool_.AllocateNode();

  // load nodes and obstacles
  if (start_node_ == nullptr) {
    ILOG_ERROR << "start node nullptr";

    result->fail_type = AstarFailType::ALLOCATE_NODE_FAIL;
    return false;
  }

  start_node_->Set(NodePath(start), XYbounds_, config_, 0.0);
  if (!start_node_->IsNodeValid()) {
    ILOG_ERROR << "start_node invalid";

    result->fail_type = AstarFailType::OUT_OF_BOUND;
    return false;
  }

  // in searching, start node gear is forward or backward which will be ok.
  start_node_->SetGearType(AstarPathGear::NONE);
  start_node_->SetSteer(0.0);
  start_node_->SetIsStartNode(true);
  start_node_->SetPathType(AstarPathType::START_NODE);
  start_node_->SetDistToStart(0.0);
  start_node_->SetGearSwitchNum(0);
  start_node_->DebugString();
  // start node gcost is 0

  // check start
  double check_start_time = IflyTime::Now_ms();
  if (!ValidityCheckByEDT(start_node_)) {
    // double check
    if (IsFootPrintCollision(Transform2d(start_node_->GetPose()))) {
      ILOG_ERROR << "start_node in collision with obstacles "
                 << static_cast<int>(start_node_->GetConstCollisionType());

      // start_node_->DebugString();

      result->fail_type = AstarFailType::START_COLLISION;
      return false;
    }
  }
  double check_end_time = IflyTime::Now_ms();
  collision_check_time_ms_ += check_end_time - check_start_time;

  // allocate end
  astar_end_node_ = node_pool_.AllocateNode();

  if (astar_end_node_ == nullptr) {
    ILOG_ERROR << "end node nullptr";

    result->fail_type = AstarFailType::OUT_OF_BOUND;
    return false;
  }
  astar_end_node_->Set(NodePath(end), XYbounds_, config_, 0.0);
  astar_end_node_->SetGearType(AstarPathGear::NONE);
  astar_end_node_->SetPathType(AstarPathType::END_NODE);
  astar_end_node_->DebugString();

  if (!astar_end_node_->IsNodeValid()) {
    ILOG_ERROR << "end_node invalid";

    result->fail_type = AstarFailType::OUT_OF_BOUND;
    return false;
  }

  // check end
  check_start_time = IflyTime::Now_ms();
  if (!ValidityCheckByEDT(astar_end_node_)) {
    ILOG_INFO << "end_node in collision with obstacles "
              << static_cast<int>(astar_end_node_->GetConstCollisionType());

    astar_end_node_->DebugString();
    result->fail_type = AstarFailType::GOAL_COLLISION;
    return false;
  }

  check_end_time = IflyTime::Now_ms();
  collision_check_time_ms_ += check_end_time - check_start_time;

  // node shrink related
  node_shrink_decider_.Process(start, end, request_.direction_request,
                               request_.real_goal, XYbounds_);

  check_start_time = IflyTime::Now_ms();
  // ILOG_INFO << "time " << IflyTime::DateString();

  // generate a star dist
  if (dp_heuristic_generator_ == nullptr) {
    ILOG_ERROR << "h cost is null";

    result->fail_type = AstarFailType::DP_COST_FAIL;
    return false;
  }
  dp_heuristic_generator_->GenerateDpMap(end.x, end.y, XYbounds, obstacles_,
                                         car_half_width_);

  check_end_time = IflyTime::Now_ms();
  ILOG_INFO << "dp map time(ms) " << check_end_time - check_start_time;

  rs_expansion_decider_.Process(
      vehicle_param_.min_turn_radius, request_.slot_width, request_.slot_length,
      start, end, vehicle_param_.width, request_.space_type,
      request_.direction_request);

  PathComparator path_comparator;

  // load open set, pq
  start_node_->SetMultiMapIter(
      open_pq_.insert(std::make_pair(0.0, start_node_)));

  node_set_.emplace(start_node_->GetGlobalID(), start_node_);

  // Hybrid A*
  size_t explored_node_num = 0;
  size_t explored_rs_path_num = 0;
  size_t h_cost_rs_path_num = 0;
  double astar_search_start_time = IflyTime::Now_ms();
  double astar_search_time;
  heuristic_time_ = 0.0;

  Node3d* current_node = nullptr;
  Node3d* next_node_in_pool = nullptr;
  Node3d new_node;
  Node3d rs_node_to_goal;
  rs_node_to_goal.Clear();

  AstarNodeVisitedType vis_type;
  bool is_safe = false;
  PathGearRequest gear_request = PathGearRequest::NONE;
  double current_time;

  int rs_path_success_num = 0;
  RSPath best_rs_path;
  Node3d best_rs_node;
  best_rs_node.ClearPath();
  best_rs_node.SetFCost(1000000.0);

  while (!open_pq_.empty()) {
    // take out the lowest cost neighboring node

    current_node = open_pq_.begin()->second;
    open_pq_.erase(open_pq_.begin());
    if (current_node == nullptr) {
      ILOG_INFO << "pq is null node";
      continue;
    }

    current_node->SetVisitedType(AstarNodeVisitedType::IN_CLOSE);

#if DEBUG_CHILD_NODE
    if (explored_node_num < DEBUG_NODE_MAX_NUM) {
      ILOG_INFO << "============== main cycle =========== ";
      ILOG_INFO << "explored_node_num " << explored_node_num
                << " open set size for now " << open_pq_.size();
      current_node->DebugString();
    }
#endif

#if PLOT_SEARCH_SEQUENCE
    queue_path_debug_.emplace_back(
        Vec2df32(current_node->GetX(), current_node->GetY()));
#endif

    current_time = IflyTime::Now_ms();
    // if bigger than 5.0 s，break
    astar_search_time = current_time - astar_search_start_time;
    if (astar_search_time > config_.max_search_time_ms) {
      // ILOG_INFO << "time out " << astar_search_time;
      break;
    }

    // check if an analystic curve could be connected from current
    // configuration to the end configuration without collision. if so,
    // search ends.
    if (AnalyticExpansionByRS(current_node, gear_request, &rs_node_to_goal)) {
      rs_path_success_num++;

      if (path_comparator.Compare(&request_, &best_rs_node, &rs_node_to_goal)) {
        best_rs_node = rs_node_to_goal;
        best_rs_path = rs_path_;
      }

#if PLOT_CHILD_NODE
      // if node is gear switch point, plot it also.
      if (rs_node_to_goal.GearSwitchNode() != nullptr) {
        child_node_debug_.emplace_back(DebugAstarSearchPoint(
            rs_node_to_goal.GearSwitchNode()->GetX(),
            rs_node_to_goal.GearSwitchNode()->GetY(), true, true));
      }
#endif

      // in try searching, no need optimal path.
      if (request_.path_generate_method ==
          AstarPathGenerateType::TRY_SEARCHING) {
        break;
      }

      // total gear switch number is 0, break;
      if (rs_node_to_goal.GetGearSwitchNum() < 1) {
        break;
      }

      /** If got some solutions, and search time bigger than 300.0, searching
       *  can break. If not satisfy break condition, continue to do searching.
       */
      if (rs_path_success_num > 5 && astar_search_time > 300.0) {
        break;
      }

      // If search time bigger than 1000 ms, break
      if (rs_path_success_num > 0 && astar_search_time > 1000.0) {
        break;
      }
    }

    explored_rs_path_num++;

    for (size_t i = 0; i < next_node_num_; ++i) {
      NextNodeGenerator(&new_node, current_node, i, gear_request);
      explored_node_num++;

#if DEBUG_CHILD_NODE
      if (explored_node_num < DEBUG_NODE_MAX_NUM) {
        ILOG_INFO << "~~~~~~~~~~ child node cycle ~~~~~~~~~";
        ILOG_INFO << "open set size " << open_pq_.size()
                  << ", gear change num:" << current_node->GetGearSwitchNum();
        new_node.DebugString();
      }
#endif

      // boundary check failure handle
      if (!new_node.IsNodeValid()) {
        continue;
      }

      if (node_shrink_decider_.IsShrinkByStartNode(start_node_->GetGlobalID(),
                                                   &new_node)) {
        continue;
      }

      if (node_shrink_decider_.IsShrinkByParent(current_node, &new_node)) {
        continue;
      }

      if (node_shrink_decider_.IsShrinkByGearSwitchNumber(&new_node)) {
        continue;
      }

      // collision check
#if LOG_TIME_PROFILE
      check_start_time = IflyTime::Now_ms();
#endif

      is_safe = ValidityCheckByEDT(&new_node);

#if LOG_TIME_PROFILE
      check_end_time = IflyTime::Now_ms();
      collision_check_time_ms_ += check_end_time - check_start_time;
#endif

      if (!is_safe) {
#if PLOT_CHILD_NODE
        // if node is unsafe, plot it also.
        int gear_switch_num = new_node.GetGearSwitchNum();
        if (explored_node_num < DEBUG_NODE_MAX_NUM &&
            gear_switch_num <= DEBUG_NODE_GEAR_SWITCH_NUMBER) {
          child_node_debug_.emplace_back(
              DebugAstarSearchPoint(new_node.GetX(), new_node.GetY(), false));
        }
#endif
        continue;
      }

      // allocate new node
      if (node_set_.find(new_node.GetGlobalID()) == node_set_.end()) {
        next_node_in_pool = node_pool_.AllocateNode();

        vis_type = AstarNodeVisitedType::NOT_VISITED;
      } else {
        next_node_in_pool = node_set_[new_node.GetGlobalID()];

        vis_type = next_node_in_pool->GetVisitedType();
      }

      // check null
      if (next_node_in_pool == nullptr) {
        // ILOG_INFO << " next node is null";
        continue;
      }

      CalculateNodeGCost(current_node, &new_node);

#if DEBUG_CHILD_NODE
      if (explored_node_num < DEBUG_NODE_MAX_NUM) {
        ILOG_INFO << "  vis type " << static_cast<int>(vis_type)
                  << " new node g " << new_node.GetGCost() << " pool node g "
                  << next_node_in_pool->GetGCost()
                  << " ,safe dist: " << new_node.GetDistToObs();
      }
#endif

      // find a new node
      if (vis_type == AstarNodeVisitedType::NOT_VISITED) {
        CalculateNodeHeuristicCost(current_node, &new_node);

        h_cost_rs_path_num++;

        *next_node_in_pool = new_node;
        next_node_in_pool->SetFCost();
        next_node_in_pool->SetVisitedType(AstarNodeVisitedType::IN_OPEN);

        next_node_in_pool->SetMultiMapIter(open_pq_.insert(
            std::make_pair(next_node_in_pool->GetFCost(), next_node_in_pool)));

        node_set_.emplace(next_node_in_pool->GetGlobalID(), next_node_in_pool);

#if DEBUG_CHILD_NODE
        if (explored_node_num < DEBUG_NODE_MAX_NUM) {
          ILOG_INFO << "new point";
          next_node_in_pool->DebugCost();
        }
#endif

      } else if (vis_type == AstarNodeVisitedType::IN_OPEN) {
        // in open set and need update
        if (new_node.GetGCost() < next_node_in_pool->GetGCost()) {
          CalculateNodeHeuristicCost(current_node, &new_node);

          h_cost_rs_path_num++;

          open_pq_.erase(next_node_in_pool->GetMultiMapIter());

          *next_node_in_pool = new_node;
          next_node_in_pool->SetFCost();
          next_node_in_pool->SetVisitedType(AstarNodeVisitedType::IN_OPEN);

          // put node in open set again and record it.
          next_node_in_pool->SetMultiMapIter(open_pq_.insert(std::make_pair(
              next_node_in_pool->GetFCost(), next_node_in_pool)));

#if DEBUG_CHILD_NODE
          if (explored_node_num < DEBUG_NODE_MAX_NUM) {
            ILOG_INFO << "node has in open";
            next_node_in_pool->DebugCost();
          }
#endif
        }
      } else {
        // in close set and need update
        if (new_node.GetGCost() < next_node_in_pool->GetGCost() + 1e-1) {
          if (node_shrink_decider_.IsLoopBackNode(&new_node,
                                                  next_node_in_pool)) {
            continue;
          }

          if (!node_shrink_decider_.IsSameGridNodeContinuous(
                  &new_node, next_node_in_pool)) {
            continue;
          }

          CalculateNodeHeuristicCost(current_node, &new_node);

          h_cost_rs_path_num++;

          *next_node_in_pool = new_node;
          next_node_in_pool->SetFCost();
          next_node_in_pool->SetVisitedType(AstarNodeVisitedType::IN_OPEN);

          // put node in open set again and record it.
          next_node_in_pool->SetMultiMapIter(open_pq_.insert(std::make_pair(
              next_node_in_pool->GetFCost(), next_node_in_pool)));

#if DEBUG_CHILD_NODE
          if (explored_node_num < DEBUG_NODE_MAX_NUM) {
            ILOG_INFO << "node has in close";
            next_node_in_pool->DebugCost();
          }
#endif
        }
      }

#if PLOT_CHILD_NODE
      int gear_switch_num = new_node.GetGearSwitchNum();
      if (explored_node_num < DEBUG_NODE_MAX_NUM &&
          gear_switch_num <= DEBUG_NODE_GEAR_SWITCH_NUMBER) {
        child_node_debug_.emplace_back(
            DebugAstarSearchPoint(new_node.GetX(), new_node.GetY(), true));
      }
#endif
    }

    // search neighbor nodes  over
  }

  double search_end_time = IflyTime::Now_ms();
  ILOG_INFO << "hybrid astar search time (ms) is "
            << search_end_time - astar_search_start_time;

  // todo, use all astar node, maybe no need use rs path.
  if (best_rs_node.IsNodeValid()) {
    BackwardPassByRSPath(result, &best_rs_node, &best_rs_path);
  } else {
    result->fail_type = AstarFailType::SEARCH_TOO_MUCH_NODE;
  }

  ILOG_INFO << "explored node num is " << explored_node_num
            << " ,rs path size is: " << explored_rs_path_num
            << " ,h cost rs num " << h_cost_rs_path_num
            << " ,node pool size:" << node_pool_.PoolSize()
            << ",open_pq.size: " << open_pq_.size()
            << ",RS success number=" << rs_path_success_num;

  ILOG_INFO << "heuristic time " << heuristic_time_ << " ,rs params time "
            << rs_time_ms_ << ",rs interpolate time:" << rs_interpolate_time_ms_
            << " ,collision time " << collision_check_time_ms_;

  double astar_end_time = IflyTime::Now_ms();
  result->time_ms = astar_end_time - astar_start_time;
  ILOG_INFO << "hybrid astar total time (ms) = " << result->time_ms;

  // ILOG_INFO << "hybrid astar finished";
  // ILOG_INFO << "child node size" << child_node_debug_.size();

#if DEBUG_EDT
  DebugEDTCheck(result);
#endif

  return true;
}

void HybridAStar::Init() {
  // rs_path_generator_ = nullptr;
  dp_heuristic_generator_ = nullptr;

  next_node_num_ = config_.next_node_num;
  max_steer_angle_ =
      vehicle_param_.max_steer_angle / vehicle_param_.steer_ratio;

  // min_radius_ = vehicle_param_.wheel_base / std::tan(max_steer_angle_);
  min_radius_ = vehicle_param_.min_turn_radius;
  inv_radius_ = 1.0 / min_radius_;

  // front sampling number is 5, back sampling number is 5.
  next_node_angles_.angles[0] = max_steer_angle_;
  next_node_angles_.radius[0] = min_radius_;

  next_node_angles_.angles[1] = -max_steer_angle_;
  next_node_angles_.radius[1] = -next_node_angles_.radius[0];

  next_node_angles_.angles[2] = 0.0;
  next_node_angles_.radius[2] = 100000.0;

  next_node_angles_.angles[3] = max_steer_angle_ * 0.5;
  next_node_angles_.radius[3] =
      vehicle_param_.wheel_base / std::tan(max_steer_angle_ * 0.5);

  next_node_angles_.angles[4] = -max_steer_angle_ * 0.5;
  next_node_angles_.radius[4] = -next_node_angles_.radius[3];
  next_node_angles_.size = 5;

  node_path_dist_resolution_ = config_.node_path_dist_resolution;
  xy_grid_resolution_ = config_.xy_grid_resolution;
  phi_grid_resolution_ = config_.phi_grid_resolution;

  car_half_width_ = vehicle_param_.width / 2 + config_.heuristic_safe_dist;

  if (config_.safe_buffer.lat_safe_buffer_outside.size() > 0) {
    UpdateCarBoxBySafeBuffer(config_.safe_buffer.lat_safe_buffer_outside[0],
                             config_.safe_buffer.lat_safe_buffer_inside[0],
                             config_.safe_buffer.lon_safe_buffer[0]);
  }

  if (dp_heuristic_generator_ == nullptr) {
    dp_heuristic_generator_ = std::make_unique<GridSearch>(config_);

    dp_heuristic_generator_->Init();
  }

  max_x_search_size_ = 128;
  max_y_search_size_ = 220;
  max_theta_search_size_ = 128;
  kinetics_model_step_ = 0.05;

  NodePoolInit();

  rs_path_h_cost_debug_.clear();
  rs_path_.Clear();

  ILOG_INFO << "astar init finish";

  return;
}

void HybridAStar::GetRSPathForDebug(std::vector<float>& x,
                                    std::vector<float>& y,
                                    std::vector<float>& phi) {
  x.clear();
  y.clear();
  phi.clear();

  for (int i = 0; i < std::min(MAX_RS_PATH_NUM, rs_path_.size); i++) {
    RSPathSegment* segment = &rs_path_.paths[i];

    for (int j = 0; j < std::min(RS_PATH_MAX_POINT, segment->size); j++) {
      x.emplace_back(segment->points[j].x);
      y.emplace_back(segment->points[j].y);
      phi.emplace_back(segment->points[j].theta);
    }
  }

  ILOG_INFO << "rs size " << x.size();

  return;
}

void HybridAStar::DebugLineSegment(
    const ad_common::math::LineSegment2d& line) const {
  ILOG_INFO << "start " << line.start().x() << " " << line.start().y();
  ILOG_INFO << "end " << line.end().x() << " " << line.end().y();

  return;
}

void HybridAStar::DebugObstacleString() const {
  // ILOG_INFO << "obs size " << obstacles_->obs_list.size();

  // for (size_t i = 0; i < obstacles_->obs_list.size(); i++) {
  //   ILOG_INFO << "type " <<
  //   static_cast<int>(obstacles_->obs_list[i].obs_type)
  //             << "pt size " << obstacles_->obs_list[i].polygons.size();

  //   for (size_t j = 0; j < obstacles_->obs_list[i].polygons.size(); j++) {
  //     ILOG_INFO << " r " << obstacles_->obs_list[i].polygons[j].radius
  //               << " center " <<
  //               obstacles_->obs_list[i].polygons[j].center_pt.x
  //               << " " << obstacles_->obs_list[i].polygons[j].center_pt.y
  //               << "pt size" <<
  //               obstacles_->obs_list[i].polygons[j].vertex_num;
  //   }
  // }

  return;
}

void HybridAStar::DebugPathString(const HybridAStarResult* result) const {
  ILOG_INFO << "path x point size " << result->x.size() << " gear size "
            << result->gear.size() << "y size " << result->y.size()
            << "phi size " << result->phi.size() << "type size "
            << result->type.size() << "s size " << result->accumulated_s.size();

  for (size_t i = 0; i < result->x.size(); i++) {
    ILOG_INFO << "i = " << i << " x, y, theta, gear:  " << result->x[i] << ", "
              << result->y[i] << ", " << result->phi[i] * 57.4 << ", "
              << PathGearDebugString(result->gear[i])
              << ",path type = " << static_cast<int>(result->type[i])
              << ", s = " << result->accumulated_s[i];
  }

  return;
}

void HybridAStar::DebugRSPath(const RSPath* reeds_shepp_path) {
  ILOG_INFO << "rs seg size " << reeds_shepp_path->size << " total len "
            << reeds_shepp_path->total_length;

  for (int i = 0; i < reeds_shepp_path->size; i++) {
    ILOG_INFO << "rs seg len " << reeds_shepp_path->paths[i].length << " steer "
              << static_cast<int>(reeds_shepp_path->paths[i].steer)
              << ", steering wheel: "
              << std::atan(reeds_shepp_path->paths[i].kappa *
                           vehicle_param_.wheel_base) *
                     vehicle_param_.steer_ratio * 57.4
              << ",rs seg size= " << reeds_shepp_path->paths[i].size;
  }

  for (size_t i = 0; i < reeds_shepp_path->size; i++) {
    const RSPathSegment* seg = &reeds_shepp_path->paths[i];
    for (size_t j = 0; j < seg->size; j++) {
      ILOG_INFO << "rs point, id " << j << "x " << seg->points[j].x << " y "
                << seg->points[j].y << " len" << seg->length << " ,kappa "
                << seg->kappa;
    }
  }

  return;
}

const bool HybridAStar::IsPointBeyondBound(const float x,
                                           const float y) const {
  if (x > XYbounds_.x_max || x < XYbounds_.x_min || y > XYbounds_.y_max ||
      y < XYbounds_.y_min) {
    return true;
  }
  return false;
}

void HybridAStar::GetNodeListMessage(planning::common::AstarNodeList* list) {
  planning::common::TrajectoryPoint point;

  for (auto i = node_set_.begin(); i != node_set_.end(); i++) {
    if (i->second->GetStepSize() < 2 || i->second->IsRsPath()) {
      continue;
    }

    const NodePath& path = i->second->GetNodePath();

    planning::common::AstarNode* tmp_node = list->add_nodes();

    for (size_t m = 0; m < path.point_size; m++) {
      point.set_x(path.points[m].x);
      point.set_y(path.points[m].y);
      point.set_heading_angle(path.points[m].theta);

      planning::common::TrajectoryPoint* tmp_point = tmp_node->add_path_point();

      tmp_point->CopyFrom(point);
    }
  }

  return;
}

void HybridAStar::GetNodeListMessage(
    std::vector<std::vector<Eigen::Vector2f>>& list) {
  for (auto i = node_set_.begin(); i != node_set_.end(); i++) {
    if (i->second->GetStepSize() < 2 || i->second->IsRsPath()) {
      continue;
    }

    const NodePath& path = i->second->GetNodePath();

    std::vector<Eigen::Vector2f> node;
    for (size_t m = 0; m < path.point_size; m++) {
      node.emplace_back(Eigen::Vector2f(path.points[m].x, path.points[m].y));
    }

    list.emplace_back(node);
  }

  return;
}

void HybridAStar::ResetNodePool() {
  node_pool_.Clear();

  return;
}

void HybridAStar::NodePoolInit() {
  node_pool_.Init();

  return;
}

bool HybridAStar::NodeInSearchBound(Node3d* node) {
  if (!NodeInSearchBound(node->GetIndex())) {
    return false;
  }

  return true;
}

bool HybridAStar::NodeInSearchBound(const NodeGridIndex& id) {
  if (id.x < 0 || id.x >= max_x_search_size_ || id.y < 0 ||
      id.y >= max_y_search_size_ || id.phi < 0 ||
      id.phi >= max_theta_search_size_) {
    return false;
  }

  return true;
}

Polygon2D* HybridAStar::GetVehPolygon(const AstarPathGear& gear) {
  if (gear == AstarPathGear::DRIVE) {
    return &veh_box_gear_drive_;
  } else if (gear == AstarPathGear::REVERSE) {
    return &veh_box_gear_reverse_;
  }

  return &veh_box_gear_none_;
}

const std::vector<DebugAstarSearchPoint>& HybridAStar::GetChildNodeForDebug() {
  // ILOG_INFO << "child node size" << child_node_debug_.size();
  return child_node_debug_;
}

const std::vector<Vec2df32>& HybridAStar::GetQueuePathForDebug() {
  return queue_path_debug_;
}

const std::vector<Vec2df32>&
HybridAStar::GetDelQueuePathForDebug() {
  return delete_queue_path_debug_;
}

const std::vector<RSPath>& HybridAStar::GetRSPathHeuristic() {
  return rs_path_h_cost_debug_;
}

void HybridAStar::KineticsModel(const Pose2D* old_pose, const float radius,
                                Pose2D* pose, const bool is_forward) {
  float dist = is_forward ? kinetics_model_step_ : -kinetics_model_step_;
  pose->x = old_pose->x + dist * std::cos(old_pose->theta);
  pose->y = old_pose->y + dist * std::sin(old_pose->theta);

  float new_phi;
  new_phi = old_pose->theta + dist / radius;
  pose->theta = ad_common::math::NormalizeAngle(new_phi);

  return;
}

void HybridAStar::UpdatePoseByPathPointInterval(const Pose2D* old_pose,
                                                const float radius,
                                                const float interval,
                                                Pose2D* pose,
                                                const bool is_forward) {
  int number = std::ceil(interval / kinetics_model_step_);

  Pose2D start = *old_pose;
  Pose2D end;

  for (int i = 0; i < number; i++) {
    KineticsModel(&start, radius, &end, is_forward);

    start = end;
  }

  *pose = end;

  return;
}

void HybridAStar::UpdatePoseBySamplingNumber(const Pose2D* old_pose,
                                             const float radius,
                                             const int number, Pose2D* pose,
                                             const bool is_forward) {
  Pose2D start = *old_pose;

  for (int i = 0; i < number; i++) {
    KineticsModel(&start, radius, pose, is_forward);

    start = *pose;
  }

  return;
}

float HybridAStar::CalcSafeDistCost(Node3d* node) {
  // weight: 15
  // [0-0.15], cost: 1000;
  // [0.15-0.5],cost: (1/dist -2) * weight;
  // [0.5-1000], cost:0;

  float dist = node->GetDistToObs();
  float weight = 10.0f;

  if (dist > 0.4f) {
    return 0.0;
  } else if (dist > 0.15f) {
    return (1.0f / dist - 2.5f) * weight;
  }

  return 100.0;
}

const ParkReferenceLine* HybridAStar::GetConstRefLine() const {
  return ref_line_;
}

int HybridAStar::InterpolateByArcOffset(Pose2D* pose,
                                        const VehicleCircle* veh_circle,
                                        const Pose2D* start_pose,
                                        const float arc,
                                        const float inverse_radius) {
  float delta_theta, theta;

  delta_theta = arc * inverse_radius;

  // left turn
  if (veh_circle->radius > 0.0) {
    if (veh_circle->gear == AstarPathGear::REVERSE) {
      delta_theta = -delta_theta;
    }
  } else {
    // right turn, gear is d
    if (veh_circle->gear == AstarPathGear::DRIVE) {
      delta_theta = -delta_theta;
    }
  }

  // update next point theta
  theta = start_pose->theta + delta_theta;

  float radius = veh_circle->radius;

  pose->x = veh_circle->center.x + radius * std::sin(theta);
  pose->y = veh_circle->center.y - radius * std::cos(theta);
  pose->theta = IflyUnifyTheta(theta, M_PI);

  return 1;
}

// radius: if left turn, radius is positive
int HybridAStar::GetVehCircleByPose(VehicleCircle* veh_circle,
                                    const Pose2D* pose, const float radius,
                                    const AstarPathGear gear) {
  veh_circle->radius = radius;
  veh_circle->gear = gear;

  veh_circle->center.x = pose->x - radius * std::sin(pose->theta);
  veh_circle->center.y = pose->y + radius * std::cos(pose->theta);

  return 1;
}

int HybridAStar::GetStraightLinePoint(Pose2D* goal_state,
                                      const Pose2D* start_state,
                                      const float dist_to_start,
                                      const Pose2D* unit_vector) {
  goal_state->x = start_state->x + dist_to_start * unit_vector->x;
  goal_state->y = start_state->y + dist_to_start * unit_vector->y;

  goal_state->theta = start_state->theta;

  return 1;
}

void HybridAStar::UpdateCarBoxBySafeBuffer(const float lat_buffer_outside,
                                           const float lat_buffer_inside,
                                           const float lon_buffer) {
  // gear d
  float safe_half_width =
      (vehicle_param_.max_width + lat_buffer_outside * 2 + 0.1) * 0.5;

  GetRightUpCoordinatePolygonByParam(
      &veh_box_gear_drive_,
      vehicle_param_.rear_edge_to_rear_axle +
          config_.safe_buffer.lon_min_safe_buffer,
      vehicle_param_.wheel_base + vehicle_param_.front_overhanging + lon_buffer,
      safe_half_width);

  // gear r
  GetRightUpCoordinatePolygonByParam(
      &veh_box_gear_reverse_,
      vehicle_param_.rear_edge_to_rear_axle + lon_buffer,
      vehicle_param_.wheel_base + vehicle_param_.front_overhanging +
          config_.safe_buffer.lon_min_safe_buffer,
      safe_half_width);

  // gear none
  GetRightUpCoordinatePolygonByParam(
      &veh_box_gear_none_,
      vehicle_param_.rear_edge_to_rear_axle +
          config_.safe_buffer.lon_min_safe_buffer,
      vehicle_param_.wheel_base + vehicle_param_.front_overhanging +
          config_.safe_buffer.lon_min_safe_buffer,
      safe_half_width);

  GenerateVehCompactPolygon(lat_buffer_inside,
                            config_.safe_buffer.lon_min_safe_buffer,
                            &cvx_hull_foot_print_);

  // PolygonDebugString(&veh_box_gear_drive_, "drive");
  // PolygonDebugString(&veh_box_gear_reverse_, "reverse");
  // PolygonDebugString(&veh_box_gear_none_, "none gear");
  // PolygonDebugString(&cvx_hull_foot_print_.body, "body");
  // PolygonDebugString(&cvx_hull_foot_print_.mirror_left, "left mirror");
  // PolygonDebugString(&cvx_hull_foot_print_.mirror_right, "right mirror");

  hierachy_circle_model_
      .footprint_model[HierarchySafeBuffer::OUTSIDE_SLOT_BUFFER]
      .UpdateSafeBuffer(lat_buffer_outside, lon_buffer, lat_buffer_outside);
  hierachy_circle_model_
      .footprint_model[HierarchySafeBuffer::INSIDE_SLOT_BUFFER]
      .UpdateSafeBuffer(lat_buffer_inside, lon_buffer, lat_buffer_inside);

  float lat_buffer = lat_buffer_outside +
              config_.safe_buffer.circle_path_extra_buffer_outside;
  hierachy_circle_model_
      .footprint_model[HierarchySafeBuffer::CIRCLE_PATH_OUTSIDE_SLOT_BUFFER]
      .UpdateSafeBuffer(lat_buffer, lon_buffer, lat_buffer);

  lat_buffer = lat_buffer_inside +
              config_.safe_buffer.circle_path_extra_buffer_inside;
  hierachy_circle_model_
      .footprint_model[HierarchySafeBuffer::CIRCLE_PATH_INSIDE_SLOT_BUFFER]
      .UpdateSafeBuffer(lat_buffer, lon_buffer, lat_buffer);

  ILOG_INFO << "outside buffer = " << lat_buffer_outside
            << ", inside buffer = " << lat_buffer_inside;

  return;
}

void HybridAStar::Clear() {
  fallback_path_.Clear();
  return;
}

void HybridAStar::CopyFallbackPath(HybridAStarResult* path) {
  *path = fallback_path_;
  return;
}

void HybridAStar::RSPathCandidateByRadius(HybridAStarResult* result,
                                          const Pose2D& start,
                                          const Pose2D& end,
                                          const float lon_min_sampling_length,
                                          const float radius) {
  bool is_connected_to_goal;
  RSPathRequestType rs_request = RSPathRequestType::NONE;

  // sampling for path end
  // sampling start point: move start point forward dist
  // (lon_min_sampling_length)
  Pose2D sampling_end = start;
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
        sampling_radius, true, true, rs_request);

#if PLOT_RS_EXNTEND_TO_END
    if (rs_path_h_cost_debug_.size() < RS_H_COST_MAX_NUM) {
      rs_path_h_cost_debug_.emplace_back(rs_path_);
    }
#endif

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
    size_t collision_id = GetPathCollisionIDByEDT(&path);
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

  if (valid_dist >= 1.2) {
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
    result->base_pose = request_.base_pose_;

    ILOG_INFO << "path valid, point size= " << result->x.size();
  } else if (valid_dist > 0.5) {
    // if path is too short by collision check or gear check, use a fallback
    // path with no collision check.
    fallback_path_.Clear();
    for (size_t i = 0; i < best_path_valid_point_size; i++) {
      fallback_path_.x.emplace_back(best_path.x[i]);
      fallback_path_.y.emplace_back(best_path.y[i]);
      fallback_path_.phi.emplace_back(best_path.phi[i]);
      fallback_path_.gear.emplace_back(best_path.gear[i]);
      fallback_path_.type.emplace_back(best_path.type[i]);
      fallback_path_.kappa.emplace_back(best_path.kappa[i]);
      fallback_path_.accumulated_s.emplace_back(best_path.accumulated_s[i]);
    }
    fallback_path_.base_pose = request_.base_pose_;
    ILOG_INFO << "path invalid, point size= " << best_path.x.size();
  }

  return;
}

void HybridAStar::GetQunticPolynomialPath(std::vector<AStarPathPoint>& path,
                                          const Pose2D& start,
                                          const float start_radius,
                                          const Pose2D& end) {
  planning_math::QuinticPolynomialCurve1d curve;

  // attention: ref line is slot center line
  // polynomial start
  float y0 = end.y;
  float dy0 = 0.0;
  float ddy0 = 0.0;

  // polynomial end
  float y1 = start.y;
  float dy1 = std::tan(start.theta);
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
  float min_radius = vehicle_param_.min_turn_radius;
  float max_kappa = 1.0 / min_radius + 1e-5;

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
    if (request_.direction_request == ParkingVehDirection::TAIL_IN) {
      point.phi = theta;
      point.gear = AstarPathGear::REVERSE;
    } else {
      point.phi = IflyUnifyTheta(theta + M_PI, M_PI);
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

/**
 * This function is a general function for generating a cubic spiral.Originally,
 *it was intended to generate a cubic spiral in Analytic Expansion to connect
 *the endpoints. It is not needed for the time being.
 *
 **/
const bool HybridAStar::GetCubicSpiralPath(std::vector<AStarPathPoint>& path,
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

void HybridAStar::DebugPolynomialPath(
    const std::vector<AStarPathPoint>& poly_path) {
  for (size_t i = 0; i < poly_path.size(); i++) {
    ILOG_INFO << "x = " << poly_path[i].x << ",y=" << poly_path[i].y
              << ",theta=" << poly_path[i].phi
              << ",kappa=" << poly_path[i].kappa
              << ",s = " << poly_path[i].accumulated_s
              << ", gear = " << static_cast<int>(poly_path[i].gear);
  }

  return;
}

bool HybridAStar::SamplingByCubicPolyForParallelSlot(
    HybridAStarResult* result, const Pose2D& start, const Pose2D& target,
    const float lon_min_sampling_length, const MapBound& XYbounds,
    const ParkObstacleList& obstacles, const AstarRequest& request,
    EulerDistanceTransform* edt, const ObstacleClearZone* clear_zone,
    ParkReferenceLine* ref_line) {
  double astar_start_time = IflyTime::Now_ms();
  ILOG_INFO << "hybrid astar begin, by cubic polynomial";

  // init
  obstacles_ = &obstacles;
  edt_ = edt;
  ref_line_ = ref_line;
  XYbounds_ = XYbounds;
  request_ = request;
  clear_zone_ = clear_zone;
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
  Pose2D sampling_end;
  PathComparator path_comparator;

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
          cubic_path_interface_.GeneratePolynomialCoefficients(start,
                                                               sampling_end);

      cubic_path_interface_.GeneratePolynomialPath(cubic_path, coefficients_vec,
                                                   0.05, start, sampling_end);

      if (cubic_path.empty()) {
        ILOG_INFO << "cubic_path empty";
        continue;
      }

      // check curvature
      if (cubic_path_interface_.GetMinCurvatureRadius() <
          vehicle_param_.min_turn_radius) {
        ILOG_INFO << "curvature is invalid, Radius = "
                  << cubic_path_interface_.GetMinCurvatureRadius();
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
      size_t collision_id = GetPathCollisionIDByEDT(cubic_path);
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

    if (std::fabs(best_path_cost.tail_heading) < DEG2RAD(1.5) &&
        std::fabs(best_path_cost.offset_to_center) < 0.1) {
      ILOG_INFO << "find best cubic poly";
      break;
    }
  }

  // end
  result->base_pose = request.base_pose_;

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
    result->base_pose = request.base_pose_;

    ILOG_INFO << "path valid, point size= " << result->x.size();
  } else {
    // if path is too short by collision check or gear check, use a fallback
    // path with no collision check.
    fallback_path_.Clear();
    for (size_t i = 0; i < path_points_size; i++) {
      result->x.emplace_back(best_cubic_path[i].x);
      result->y.emplace_back(best_cubic_path[i].y);
      result->phi.emplace_back(best_cubic_path[i].phi);
      result->gear.emplace_back(best_cubic_path[i].gear);
      result->type.emplace_back(best_cubic_path[i].type);
      result->kappa.emplace_back(best_cubic_path[i].kappa);
      result->accumulated_s.emplace_back(best_cubic_path[i].accumulated_s);
    }
    fallback_path_.base_pose = request.base_pose_;
    ILOG_INFO << "path invalid, point size= " << fallback_path_.x.size();

    return false;
  }

  // DebugRSPath(&rs_path_);

  double astar_end_time = IflyTime::Now_ms();
  ILOG_INFO << "hybrid astar total time (ms): "
            << astar_end_time - astar_start_time;

  return true;
}

size_t HybridAStar::GetPathCollisionIDByEDT(
    const std::vector<AStarPathPoint>& poly_path) {
  if (poly_path.empty()) {
    return 0;
  }

  size_t path_end_id = poly_path.size() - 1;

  if (obstacles_->point_cloud_list.empty() && obstacles_->virtual_obs.empty()) {
    return path_end_id;
  }

  Polygon2D polygon;
  Pose2D global_pose;

  size_t collision_index = 100000;
  cdl::AABB path_point_aabb;
  Polygon2D* veh_local_polygon = nullptr;
  Transform2d tf;

  for (size_t i = 0; i <= path_end_id; ++i) {
    // check bound
    if (IsPointBeyondBound(poly_path[i].x, poly_path[i].y)) {
      collision_index = i;

      break;
    }

    global_pose.x = poly_path[i].x;
    global_pose.y = poly_path[i].y;
    global_pose.theta = poly_path[i].phi;
    tf.SetBasePose(global_pose);

    veh_local_polygon = GetVehPolygon(poly_path[i].gear);
    RULocalPolygonToGlobalFast(&polygon, veh_local_polygon, &global_pose,
                               tf.GetCosTheta(), tf.GetSinTheta());

    GetBoundingBoxByPolygon(&path_point_aabb, &polygon);
    if (clear_zone_->IsContain(path_point_aabb)) {
      continue;
    }

    if (edt_->IsCollisionForPoint(
            &tf, poly_path[i].gear,
            GetCircleFootPrintModel(global_pose,
                                    IsCirclePathByKappa(poly_path[i].kappa)))) {
      return i;
    }
  }

  return collision_index;
}

void HybridAStar::UpdateConfig(const AstarRequest& request) {
  if (request.space_type == ParkSpaceType::PARALLEL) {
    config_.node_step = config_.parallel_slot_node_step;
  } else {
    config_.node_step = config_.perpendicular_slot_node_step;
  }

  if (request.direction_request == ParkingVehDirection::HEAD_IN) {
    slot_box_ = cdl::AABB(
        cdl::Vector2r(0.0f, -request.slot_width / 2),
        cdl::Vector2r(request.slot_length + (float)vehicle_param_.length,
                      request.slot_width / 2));
  } else {
    slot_box_ = cdl::AABB(
        cdl::Vector2r(0.0f, -request.slot_width / 2),
        cdl::Vector2r(request.slot_length + 1.5, request.slot_width / 2));
  }

  slot_box_.DebugString();

  return;
}

void HybridAStar::ReversePathBySwapStartGoal(HybridAStarResult* result) {
  if (!request_.swap_start_goal) {
    return;
  }

  if (result->x.size() < 2) {
    return;
  }

  std::reverse(result->x.begin(), result->x.end());
  std::reverse(result->y.begin(), result->y.end());
  std::reverse(result->phi.begin(), result->phi.end());
  std::reverse(result->gear.begin(), result->gear.end());
  std::reverse(result->type.begin(), result->type.end());
  std::reverse(result->kappa.begin(), result->kappa.end());

  for (size_t i = 0; i < result->gear.size(); i++) {
    if (result->gear[i] == AstarPathGear::DRIVE) {
      result->gear[i] = AstarPathGear::REVERSE;
    } else {
      result->gear[i] = AstarPathGear::DRIVE;
    }
  }

  return;
}

const bool HybridAStar::IsPolygonCollision(const Polygon2D* polygon) {
  bool is_collision = false;
  for (const auto& obstacle : obstacles_->point_cloud_list) {
    // envelop box check
    gjk_interface_.PolygonCollisionByCircleCheck(
        &is_collision, &obstacle.envelop_polygon, polygon, 0.001);

    if (!is_collision) {
      continue;
    }

    // internal points
    for (size_t j = 0; j < obstacle.points.size(); j++) {
      gjk_interface_.PolygonPointCollisionDetect(&is_collision, polygon,
                                                 obstacle.points[j]);

      if (is_collision) {
        // ILOG_INFO << "size = " << obstacle.points.size() << " j =" << j;
        return true;
      }

      // ILOG_INFO << "point no collision";
    }
  }

  for (const auto& obstacle : obstacles_->virtual_obs) {
    gjk_interface_.PolygonPointCollisionDetect(&is_collision, polygon,
                                               obstacle);

    if (is_collision) {
      return true;
    }
  }

  return false;
}

const bool HybridAStar::IsFootPrintCollision(const Transform2d& tf) {
  Polygon2D veh_global_polygon;
  bool is_collision = false;
  ULFLocalPolygonToGlobal(&veh_global_polygon,
                          &cvx_hull_foot_print_.max_polygon, tf);

  is_collision = IsPolygonCollision(&veh_global_polygon);
  if (!is_collision) {
    return false;
  }

  ULFLocalPolygonToGlobal(&veh_global_polygon, &cvx_hull_foot_print_.body, tf);
  is_collision = IsPolygonCollision(&veh_global_polygon);
  if (is_collision) {
    return true;
  }

  ULFLocalPolygonToGlobal(&veh_global_polygon,
                          &cvx_hull_foot_print_.mirror_left, tf);
  // ILOG_INFO << "left mirror check";
  is_collision = IsPolygonCollision(&veh_global_polygon);
  if (is_collision) {
    return true;
  }

  ULFLocalPolygonToGlobal(&veh_global_polygon,
                          &cvx_hull_foot_print_.mirror_right, tf);

  is_collision = IsPolygonCollision(&veh_global_polygon);
  if (is_collision) {
    return true;
  }

  return false;
}

bool HybridAStar::SamplingByRSPath(Node3d* current_node,
                                   Node3d* rs_node_to_goal) {
  if (current_node->GetY() > 0.8 || current_node->GetY() < -0.8) {
    return false;
  }

  float x_diff = current_node->GetX() - astar_end_node_->GetX();
  if (x_diff < 0.2) {
    return false;
  }

  if (std::fabs(current_node->GetRadius()) < min_radius_ - 1e-5) {
    return false;
  }

  float heading_diff = IflyUnifyTheta(current_node->GetPhi(), M_PI) -
                        IflyUnifyTheta(astar_end_node_->GetPhi(), M_PI);
  heading_diff = IflyUnifyTheta(heading_diff, M_PI);
  if (heading_diff > ifly_deg2rad(60.0) || heading_diff < ifly_deg2rad(-60.0)) {
    return false;
  }

  // init
  rs_node_to_goal->Clear();
  float min_straight_dist = 0.7;
  if (request_.direction_request == ParkingVehDirection::TAIL_IN) {
    min_straight_dist = 0.7;
  } else if (request_.direction_request == ParkingVehDirection::HEAD_IN) {
    min_straight_dist = 0.3;
  }

  float sample_range =
      astar_end_node_->GetX() - (request_.real_goal.GetX() + min_straight_dist);
  int sampline_numer = std::ceil(sample_range / 0.1);
  sampline_numer = std::max(1, sampline_numer);
  bool valid_path = false;
  bool is_connected_to_goal = false;

  Pose2D end_pose = astar_end_node_->GetPose();
  end_pose.x += 0.1;

  for (int q = 0; q < sampline_numer; q++) {
    end_pose.x -= 0.1;
    rs_path_interface_.GeneShortestRSPath(
        &rs_path_, &is_connected_to_goal, &current_node->GetPose(), &end_pose,
        vehicle_param_.min_turn_radius, false, false, RSPathRequestType::NONE);

    // check length
    if (rs_path_.total_length < 0.01 || !is_connected_to_goal) {
      continue;
    }

    // check gear
    bool has_different_gear = false;
    for (int j = 0; j < rs_path_.size; j++) {
      if (rs_path_.paths[j].gear !=
          request_.first_action_request.gear_request) {
        // ILOG_INFO << " rs path seg need single shot by drive gear ";
        has_different_gear = true;
        break;
      }
    }
    if (has_different_gear) {
      continue;
    }

    // interpolate
    rs_path_interface_.RSPathInterpolate(&rs_path_, &current_node->GetPose(),
                                         vehicle_param_.min_turn_radius);

    // check safe
    if (!IsRSPathSafeByEDT(&rs_path_, rs_node_to_goal)) {
      continue;
    }

    valid_path = true;
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
  node_path.points[0].theta = IflyUnifyTheta(rs_end_point.theta, M_PI);

  rs_node_to_goal->Set(node_path, XYbounds_, config_, node_path.path_dist);
  if (!NodeInSearchBound(rs_node_to_goal->GetIndex())) {
    rs_node_to_goal->Clear();
    return false;
  }

  rs_node_to_goal->SetPathType(AstarPathType::REEDS_SHEPP);
  rs_node_to_goal->SetGearType(AstarPathGear::REVERSE);
  rs_node_to_goal->SetPre(current_node);
  rs_node_to_goal->SetNext(nullptr);

  if (!rs_node_to_goal->IsNodeValid()) {
    return false;
  }

  Node3d* end_in_pool = node_pool_.AllocateNode();
  if (end_in_pool == nullptr) {
    rs_node_to_goal->Clear();
    return false;
  }

  *end_in_pool = *rs_node_to_goal;

  return true;
}

FootPrintCircleModel* HybridAStar::GetCircleFootPrintModel(
    const Pose2D& pose, const bool is_circle_path) {
  // 60 degree
  if (slot_box_.contain(pose) &&
      std::fabs(ad_common::math::NormalizeAngle(pose.theta -
                                                request_.goal_.theta)) < 1.05) {
    if (is_circle_path) {
      return &hierachy_circle_model_.footprint_model
                  [HierarchySafeBuffer::CIRCLE_PATH_INSIDE_SLOT_BUFFER];
    }

    return &hierachy_circle_model_
                .footprint_model[HierarchySafeBuffer::INSIDE_SLOT_BUFFER];
  }

  if (is_circle_path) {
    return &hierachy_circle_model_.footprint_model
                [HierarchySafeBuffer::CIRCLE_PATH_OUTSIDE_SLOT_BUFFER];
  }
  return &hierachy_circle_model_
              .footprint_model[HierarchySafeBuffer::OUTSIDE_SLOT_BUFFER];
}

FootPrintCircleModel* HybridAStar::GetSlotOutsideCircleFootPrint() {
  return &hierachy_circle_model_
              .footprint_model[HierarchySafeBuffer::OUTSIDE_SLOT_BUFFER];
}

const bool HybridAStar::IsExpectedGearForRsPath(const RSPath& path) {
  for (int j = 0; j < path.size; j++) {
    if (path.paths[j].gear != request_.first_action_request.gear_request) {
      // ILOG_INFO << " rs path seg need single shot by drive gear ";
      return false;
    }
  }

  return true;
}

void HybridAStar::PathTransformByRSPath(const RSPath& rs_path,
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

const bool HybridAStar::BackwardPassByNode(HybridAStarResult* result,
                                           Node3d* end_node) {
  if (end_node == nullptr) {
    result->Clear();
    return false;
  }

  Node3d* parent_node = nullptr;
  Node3d* child_node = end_node;
  end_node->SetNext(nullptr);

  result->base_pose = request_.base_pose_;
  result->gear_change_num = 0;

  ILOG_INFO << "get result start backward pass by node";

  // backward pass
  while (child_node->GetPreNode() != nullptr) {
    parent_node = child_node->GetPreNode();
    parent_node->SetNext(child_node);
    child_node = parent_node;
  }

  size_t point_size;
  float kappa;

  AstarPathGear last_gear_type = AstarPathGear::NONE;
  AstarPathGear cur_gear_type;
  // all nodes
  std::vector<Node3d*> node_list;
  while (child_node != nullptr) {
    const NodePath& path = child_node->GetNodePath();
    AstarPathType path_type = child_node->GetPathType();
    cur_gear_type = child_node->GetGearType();

    // todo
    if (child_node->GetConstNextNode() == nullptr) {
      point_size = path.point_size;
    }
    // gear change
    else if (child_node->IsPathGearChange(
                 child_node->GetConstNextNode()->GetGearType())) {
      point_size = path.point_size;
    }
    // same gear
    else {
      // delete same point
      point_size = path.point_size - 1;
    }

    kappa = std::tan(child_node->GetSteer()) / vehicle_param_.wheel_base;

    for (size_t k = 0; k < point_size; k++) {
      result->x.emplace_back(path.points[k].x);
      result->y.emplace_back(path.points[k].y);
      result->phi.emplace_back(path.points[k].theta);
      result->type.emplace_back(path_type);
      result->gear.emplace_back(cur_gear_type);
      result->kappa.emplace_back(kappa);
    }

    // check gear switch number
    if (last_gear_type != AstarPathGear::NONE) {
      if (last_gear_type != cur_gear_type) {
        result->gear_change_num++;
      }
    }

    node_list.push_back(child_node);

    last_gear_type = cur_gear_type;
    child_node = child_node->GetMutableNextNode();
  }

  size_t pt_size = result->x.size();
  if (pt_size != result->y.size() || pt_size != result->phi.size() ||
      pt_size != result->gear.size()) {
    ILOG_ERROR << "state sizes not equal, "
               << "result->x.size(): " << result->x.size() << "result->y.size()"
               << result->y.size() << "result->phi.size()"
               << result->phi.size();

    return false;
  }

  ReversePathBySwapStartGoal(result);

  // get path lengh
  size_t path_points_size = result->x.size();

  float accumulated_s = 0.0;
  result->accumulated_s.clear();
  auto last_x = result->x.front();
  auto last_y = result->y.front();
  float x_diff;
  float y_diff;
  for (size_t i = 0; i < path_points_size; ++i) {
    x_diff = result->x[i] - last_x;
    y_diff = result->y[i] - last_y;
    accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
    result->accumulated_s.push_back(accumulated_s);
    last_x = result->x[i];
    last_y = result->y[i];
  }

  ILOG_INFO << "get result finish, path point size " << result->x.size();

  result->fail_type = AstarFailType::SUCCESS;

  // DebugPathString(result);

#if DEBUG_SEARCH_RESULT
  ILOG_INFO << "path node num " << node_list.size();
  for (size_t i = 0; i < node_list.size(); i++) {
    ILOG_INFO << "node id " << i << " node steer "
              << node_list[i]->GetSteer() * 57.3 << " forward "
              << static_cast<int>(node_list[i]->GetGearType()) << " is rs path "
              << (node_list[i]->GetPathType() == AstarPathType::REEDS_SHEPP)
              << ", length: "
              << node_path_dist_resolution_ * node_list[i]->GetStepSize();
  }
#endif

  return true;
}

const bool HybridAStar::BestNodeIsNice(const Node3d* node) {
  bool node_is_good = false;
  if (node == nullptr) {
    return false;
  }

  // ILOG_INFO <<"check best node";
  // node->DebugPoseString();

  if (std::fabs((node->GetPose().y - request_.real_goal.y)) > 0.05) {
    return false;
  }

  if (std::fabs(ad_common::math::NormalizeAngle(
          node->GetPose().theta - request_.real_goal.theta)) > 0.02) {
    return false;
  }

  return true;
}

}  // namespace planning
