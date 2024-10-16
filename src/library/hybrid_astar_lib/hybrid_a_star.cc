#include "hybrid_a_star.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>

#include "ad_common/math/math_utils.h"
#include "h_cost.h"
#include "hybrid_astar_common.h"
#include "log_glog.h"
#include "math_utils.h"
#include "node3d.h"
#include "pose2d.h"
#include "reeds_shepp.h"
#include "src/common/ifly_time.h"
#include "transform2d.h"
#include "utils_math.h"
#include "src/modules/common/math/curve1d/quintic_polynomial_curve1d.h"

namespace planning {

#define PLOT_RS_COST_PATH (1)
#define PLOT_RS_EXNTEND_TO_END (0)
#define PLOT_CHILD_NODE (1)
#define PLOT_SEARCH_SEQUENCE (0)
#define RS_H_COST_MAX_NUM (32)

#define DEBUG_SEARCH_RESULT (0)
#define DEBUG_CHILD_NODE (1)
#define DEBUG_TARGET_HEADING_COST (0)
#define DEBUG_EDT (0)

#define DEBUG_NODE_MAX_NUM (10000)

#define LOG_TIME_PROFILE (0)
#define DEBUG_GJK (0)

#define DEBUG_ONE_SHOT_PATH (0)
#define DEBUG_ONE_SHOT_PATH_MAX_NODE (10000)

constexpr int backward_pass_max_point = 100000;
// 控制可以执行的距离
constexpr double rs_path_seg_advised_dist = 0.35;
constexpr double max_search_time = 60.0;

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
                                   const RSPathRequestType rs_request,
                                   const double rs_radius) {
#if LOG_TIME_PROFILE
  double rs_start_time = IflyTime::Now_ms();
#endif

  const Pose2D& start_pose = current_node->GetPose();
  const Pose2D& end_pose = rs_expansion_decider_.GetRSEndPose();

  bool is_connected_to_goal;
  rs_path_interface_.GeneShortestRSPath(&rs_path_, &is_connected_to_goal,
                                        &start_pose, &end_pose, rs_radius,
                                        need_rs_dense_point, rs_request);

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

bool HybridAStar::PlanByRSPathLink(HybridAStarResult* result,
                                   const Pose2D& start, const Pose2D& end,
                                   const double expected_path_dist,
                                   const MapBound& XYbounds,
                                   const ParkObstacleList& obstacles,
                                   const AstarRequest& request,
                                   const ObstacleClearZone *clear_zone,
                                   EulerDistanceTransform* edt,
                                   ParkReferenceLine* ref_line) {
  double astar_start_time = IflyTime::Now_ms();
  ILOG_INFO << "hybrid astar begin, generate path by rs.";

  rs_path_.Clear();
  result->Clear();

  bool is_connected_to_goal;
  RSPathRequestType rs_request = RSPathRequestType::none;
  rs_path_interface_.GeneShortestRSPath(
      &rs_path_, &is_connected_to_goal, &start, &end,
      vehicle_param_.min_turn_radius + 3.0, true, rs_request);

  if (rs_path_.total_length < 0.01 || !is_connected_to_goal) {
    ILOG_INFO << "rs path fail";

    return false;
  }

  for (int i = 0; i < rs_path_.size; i++) {
    RSPathSegment* segment = &rs_path_.paths[i];

    for (int j = 0; j < segment->size; j++) {
      result->x.emplace_back(segment->points[j].x);
      result->y.emplace_back(segment->points[j].y);
      result->phi.emplace_back(segment->points[j].theta);
      result->gear.emplace_back(segment->gear);
      result->type.emplace_back(AstarPathType::REEDS_SHEPP);
      result->kappa.emplace_back(segment->points[j].kappa);
    }
  }

  // init
  obstacles_ = &obstacles;

  // load XYbounds
  XYbounds_ = XYbounds;

  request_ = request;
  DebugAstarRequestString(request_);

  clear_zone_ = clear_zone;
  edt_ = edt;
  ref_line_ = ref_line;

  // get path lengh
  size_t path_points_size = result->x.size();

  double accumulated_s = 0.0;
  result->accumulated_s.clear();
  auto last_x = result->x.front();
  auto last_y = result->y.front();
  double x_diff;
  double y_diff;

  size_t expected_dist_id = 0;

  for (size_t i = 0; i < path_points_size; ++i) {
    x_diff = result->x[i] - last_x;
    y_diff = result->y[i] - last_y;
    accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
    result->accumulated_s.emplace_back(accumulated_s);

    if (accumulated_s <= expected_path_dist) {
      expected_dist_id = i;
    }

    last_x = result->x[i];
    last_y = result->y[i];
  }

  // collision check
  size_t collision_id = GetPathCollisionIndex(result);

  // shrink length
  for (size_t i = 0; i < path_points_size; i++) {
    if (result->x.size() <= 1) {
      break;
    }

    size_t end_point_id = result->x.size() - 1;

    if (end_point_id >= collision_id || end_point_id > expected_dist_id) {
      result->x.pop_back();
      result->y.pop_back();
      result->phi.pop_back();
      result->gear.pop_back();
      result->type.pop_back();
      result->accumulated_s.pop_back();
      result->kappa.pop_back();
    }
  }

  result->base_pose = request.base_pose_;

  // DebugRSPath(&rs_path_);

  double astar_end_time = IflyTime::Now_ms();
  ILOG_INFO << "hybrid astar total time (ms): "
            << astar_end_time - astar_start_time;

  return true;
}

bool HybridAStar::PlanByRSPathSampling(HybridAStarResult* result,
                                       const Pose2D& start, const Pose2D& end,
                                       const double lon_min_sampling_length,
                                       const MapBound& XYbounds,
                                       const ParkObstacleList& obstacles,
                                       const AstarRequest& request,
                                       EulerDistanceTransform* edt,
                                       const ObstacleClearZone* clear_zone,
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

  double max_radius = 85.0;
  double min_radius = 8.0;
  double radius_step = 5.0;

  int sampline_numer = std::ceil(max_radius - min_radius) / radius_step;

  HybridAStarResult path;

  double radius = max_radius;
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
    double best_length = 0.0;
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
  ILOG_INFO << "hybrid astar total time (ms) = " << result->time_ms;

  return true;
}

bool HybridAStar::PlanByCubicPath(HybridAStarResult* result,
                                  const Pose2D& start, const Pose2D& end,
                                  const double lon_min_sampling_length,
                                  const MapBound& XYbounds,
                                  const ParkObstacleList& obstacles,
                                  const AstarRequest& request,
                                  EulerDistanceTransform* edt,
                                  const ObstacleClearZone *clear_zone,
                                  ParkReferenceLine* ref_line) {
  double astar_start_time = IflyTime::Now_ms();
  ILOG_INFO << "hybrid astar begin, generate path by cubic.";

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
  // sampling start point: move start point forward dist (expected_path_dist)
  Pose2D sampling_end = start;
  sampling_end.y = 0.0;
  sampling_end.theta = 0.0;
  sampling_end.x = start.x + lon_min_sampling_length;

  double sampling_step = 0.1;
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
    std::vector<double> coefficients_vec =
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
      if (cubic_path[j].gear == AstarPathGear::reverse) {
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

    double accumulated_s = 0.0;
    path.accumulated_s.clear();
    auto last_x = path.x.front();
    auto last_y = path.y.front();
    double x_diff;
    double y_diff;

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
  double valid_dist = 0.0;
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
  } else {
    // if path is too short by collision check or gear check, use a fallback
    // path with no collision check.
    fallback_path_.Clear();
    for (size_t i = 0; i < expected_dist_id; i++) {
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
  ILOG_INFO << "hybrid astar total time (ms): "
            << astar_end_time - astar_start_time;

  return true;
}

bool HybridAStar::AnalyticExpansionByRS(Node3d* current_node,
                                        const PathGearRequest gear_request_info,
                                        Node3d* rs_node_to_goal) {
  // check gear and steering wheel
  if (!rs_expansion_decider_.IsNeedRsExpansion(current_node)) {
    // ILOG_INFO << "no need rs path link";
    return false;
  }

  const double rs_radius = vehicle_param_.min_turn_radius + 0.2;

  RSPathRequestType rs_request = RSPathRequestType::gear_switch_less_than_twice;
  if (!CalcRSPathToGoal(current_node, false, rs_request, rs_radius)) {
    ILOG_INFO << " generate rs fail";

    return false;
  }

  // set node by rs path
  if (rs_path_.size < 1) {
    return false;
  }

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
        if ((current_node->GetDistToStart() +
             std::fabs(rs_path_.paths[0].length)) <
            request_.first_action_request.dist_request) {
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
  double first_path_dist = current_node->GetNodePathDistance();
  if (current_node->GetGearType() != rs_path_.paths[0].gear) {
    first_path_dist = 0;
  }

  // length check
  if (!IsRsPathFirstSegmentLongEnough(&rs_path_, first_path_dist)) {
    // ILOG_INFO << "length is not expectation";
    return false;
  }

  // last segment gear check
  if (!RsLastSegmentSatisfyRequest(&rs_path_)) {
    return false;
  }

  // single shot check
  if (!IsRSPathSingleShot(&rs_path_)) {
    return false;
  }

  AstarPathGear gear;
  for (int i = 0; i < rs_path_.size; i++) {
    gear = rs_path_.paths[i].gear;
    if (gear_request_info == PathGearRequest::GEAR_REVERSE_ONLY &&
        gear == AstarPathGear::drive) {
      return false;
    } else if (gear_request_info == PathGearRequest::GEAR_DRIVE_ONLY &&
               gear == AstarPathGear::reverse) {
      return false;
    }
  }

  // interpolation
#if LOG_TIME_PROFILE
  double rs_start_time = IflyTime::Now_ms();
#endif

  const Pose2D &start_pose = current_node->GetPose();
  rs_path_interface_.RSPathInterpolate(&rs_path_, &start_pose, rs_radius);

#if LOG_TIME_PROFILE
  double rs_end_time = IflyTime::Now_ms();
  rs_interpolate_time_ms_ += rs_end_time - rs_start_time;
#endif

#if PLOT_RS_EXNTEND_TO_END
  if (rs_path_h_cost_debug_.size() < RS_H_COST_MAX_NUM) {
    rs_path_h_cost_debug_.emplace_back(rs_path_);
  }
#endif

  NodePath path;
  path.path_dist = std::fabs(rs_path_.total_length);
  path.point_size = 1;
  // use end point to fill astar node
  RSPoint rs_end_point;
  rs_path_.BackPoint(&rs_end_point);

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

  Node3d* rs_end_in_pool = node_pool_.AllocateNode();
  if (rs_end_in_pool == nullptr) {
    ILOG_ERROR << " rs end fail";
    rs_node_to_goal->ClearPath();
    return false;
  }

  *rs_end_in_pool = *rs_node_to_goal;

  if (rs_end_in_pool != nullptr) {
    ILOG_INFO << "rs path size " << rs_path_.size;
  }

  // DebugRSPath(&rs_path_);

  return true;
}

bool HybridAStar::ExpansionByQunticPolynomial(
    Node3d* current_node, std::vector<AStarPathPoint>& path,
    Node3d* polynomial_node, PolynomialPathErrorCode* fail_type) {
  *fail_type = PolynomialPathErrorCode::NONE;
  if (current_node->GetY() > 0.8 || current_node->GetY() < -0.8) {
    *fail_type = PolynomialPathErrorCode::OUT_OF_Y_BOUNDARY;
    return false;
  }

  double x_diff = current_node->GetX() - astar_end_node_->GetX();
  if (x_diff < 0.2) {
    *fail_type = PolynomialPathErrorCode::OUT_OF_X_BOUNDARY;
    return false;
  }

  if (std::fabs(current_node->GetRadius()) < min_radius_ - 1e-5) {
    *fail_type = PolynomialPathErrorCode::LINK_POINT_INVALID_KAPPA;
    return false;
  }

  double heading_diff = IflyUnifyTheta(current_node->GetPhi(), M_PI) -
                        IflyUnifyTheta(astar_end_node_->GetPhi(), M_PI);
  heading_diff = IflyUnifyTheta(heading_diff, M_PI);
  if (heading_diff > ifly_deg2rad(60.0) || heading_diff < ifly_deg2rad(-60.0)) {
    *fail_type = PolynomialPathErrorCode::OUT_OF_HEADING_BOUNDARY;
    return false;
  }

  // init
  polynomial_node->Clear();
  path.clear();

  GetQunticPolynomialPath(path, current_node->GetPose(),
                          current_node->GetRadius(),
                          astar_end_node_->GetPose());

  // set node by rs path
  if (path.size() < 1) {
    *fail_type = PolynomialPathErrorCode::OUT_OF_KAPPA_BOUNDARY;
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

  // collision check
  if (!IsPolynomialPathSafeByEDT(path, polynomial_node)) {
    *fail_type = PolynomialPathErrorCode::COLLISION;
    polynomial_node->Clear();
    return false;
  }

  // ILOG_INFO << "Reach the end configuration with Reeds Shepp";

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
                                             const double father_node_dist) {
  double same_gear_path_min_dist;

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

    if (same_gear_path_min_dist < rs_path_seg_advised_dist) {
      ILOG_INFO << " rs path seg len " << same_gear_path_min_dist;
      return false;
    }
  }

  return true;
}

bool HybridAStar::IsRsPathFirstSegmentLongEnough(
    const RSPath* reeds_shepp_to_end, const double father_node_dist) {
  double same_gear_path_min_dist;

  int rs_path_seg_size = reeds_shepp_to_end->size;
  int left_pointer_id = 0;
  int right_pointer_id = 1;

  AstarPathGear gear = reeds_shepp_to_end->paths[left_pointer_id].gear;
  double len = reeds_shepp_to_end->paths[left_pointer_id].length;

  same_gear_path_min_dist = std::fabs(len);
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

  if (same_gear_path_min_dist < rs_path_seg_advised_dist) {
    // ILOG_INFO << " rs path first seg len is small " <<
    // same_gear_path_min_dist;
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

  AstarPathGear gear = reeds_shepp_to_end->paths[rs_path_seg_size - 1].gear;

  if (request_.parking_task == ParkingTask::parking_in &&
      request_.space_type == ParkSpaceType::vertical &&
      request_.head_request == ParkingVehDirectionRequest::tail_in_first &&
      request_.rs_request == RSPathRequestType::last_path_forbid_forward) {
    if (gear == AstarPathGear::drive) {
      // ILOG_INFO << " rs path last seg len is drive gear ";

      return false;
    }
  }

  return true;
}

bool HybridAStar::IsRSPathSingleShot(const RSPath* reeds_shepp_to_end) {
  int rs_path_seg_size = reeds_shepp_to_end->size;
  if (rs_path_seg_size < 1) {
    return true;
  }

  AstarPathGear gear;

  for (int i = 0; i < rs_path_seg_size; i++) {
    gear = reeds_shepp_to_end->paths[i].gear;

    if (request_.parking_task == ParkingTask::parking_in &&
        request_.space_type == ParkSpaceType::vertical) {
      if (request_.rs_request == RSPathRequestType::all_path_forbid_forward &&
          gear == AstarPathGear::drive) {
        // ILOG_INFO << " rs path seg need single shot by reverse gear ";
        return false;

      } else if (request_.rs_request ==
                     RSPathRequestType::all_path_forbid_reverse &&
                 gear == AstarPathGear::reverse) {
        // ILOG_INFO << " rs path seg need single shot by drive gear ";
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
      node->SetCollisionType(NodeCollisionType::map_bound);
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
        node->SetCollisionType(NodeCollisionType::virtual_wall);
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
          node->SetCollisionType(NodeCollisionType::obs);
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

  Polygon2D global_polygon;
  Pose2D global_pose;
  // bool is_collision = false;
  cdl::AABB path_point_aabb;
  Transform2d tf;
  AstarPathGear gear = node->GetGearType();

  Polygon2D* veh_local_polygon = GetVehPolygon(gear);

  float dist = 100.0;
  float min_dist = 100.0;

  for (size_t i = check_start_index; i < node_step_size; ++i) {
    // check bound
    if (IsPointBeyondBound(path.points[i].x, path.points[i].y)) {
      node->SetCollisionType(NodeCollisionType::map_bound);
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

    if (config_.enable_obs_dist_g_cost) {
      if (edt_->DistanceCheckForPoint(&dist, &tf, gear)) {
        node->SetCollisionType(NodeCollisionType::obs);
        node->SetDistToObs(dist);
        // node->SetCollisionID(i);

        return false;
      }

      if (dist < min_dist) {
        min_dist = dist;
      }
    } else {
      if (edt_->IsCollisionForPoint(&tf, gear)) {
        node->SetCollisionType(NodeCollisionType::obs);
        // node->SetCollisionID(i);

        return false;
      }
    }

    // ILOG_INFO << "path size " << node_step_size << " ,pt id " << i
    //           << " , no collision ";

    // if (i == 0) {
    //   break;
    // }
  }

  node->SetDistToObs(min_dist);

  return true;
}

void HybridAStar::DebugEDTCheck(HybridAStarResult* path) {
  if (path == nullptr || path->x.size() < 1) {
    return;
  }

  size_t node_step_size = path->x.size();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t check_start_index = 0;

  Polygon2D global_polygon;
  Pose2D global_pose;
  // bool is_collision = false;
  cdl::AABB path_point_aabb;
  Transform2d tf;
  AstarPathGear first_gear = path->gear[0];
  AstarPathGear gear = path->gear[0];

  Polygon2D* veh_local_polygon = GetVehPolygon(gear);

  for (size_t i = check_start_index; i < node_step_size; ++i) {
    // check bound
    if (IsPointBeyondBound(path->x[i], path->y[i])) {
      ILOG_INFO << "no coll";
      continue;
    }

    global_pose.x = path->x[i];
    global_pose.y = path->y[i];
    global_pose.theta = path->phi[i];
    tf.SetBasePose(global_pose);

    RULocalPolygonToGlobalFast(&global_polygon, veh_local_polygon, &global_pose,
                               tf.GetCosTheta(), tf.GetSinTheta());

    GetBoundingBoxByPolygon(&path_point_aabb, &global_polygon);
    if (clear_zone_->IsContain(path_point_aabb)) {
      ILOG_INFO << "clear";
      continue;
    }

    gear = path->gear[i];

    if (gear != first_gear) {
      break;
    }

    if (edt_->IsCollisionForPoint(&tf, gear)) {
      ILOG_INFO << "collision";

      break;
    }

    ILOG_INFO << "path size " << node_step_size << " ,pt id " << i
              << " , no collision ";
  }

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
        node->SetCollisionType(NodeCollisionType::map_bound);
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
          node->SetCollisionType(NodeCollisionType::virtual_wall);
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
            node->SetCollisionType(NodeCollisionType::obs);
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
        node->SetCollisionType(NodeCollisionType::map_bound);
        return false;
      }

      global_pose.x = segment->points[i].x;
      global_pose.y = segment->points[i].y;
      global_pose.theta = segment->points[i].theta;

      tf.SetBasePose(global_pose);

      veh_local_polygon = GetVehPolygon(segment->gear);

      // ILOG_INFO << "gear " << PathGearDebugString(segment->gear);

      RULocalPolygonToGlobalFast(&global_polygon, veh_local_polygon,
                                 &global_pose, tf.GetCosTheta(),
                                 tf.GetSinTheta());

      GetBoundingBoxByPolygon(&path_point_aabb, &global_polygon);
      if (clear_zone_->IsContain(path_point_aabb)) {
        // ILOG_INFO << "clear";
        continue;
      }

      // if (edt_->IsCollisionForPoint(&tf, AstarPathGear::none)) {
      if (edt_->IsCollisionForPoint(&tf, segment->gear)) {
        node->SetCollisionType(NodeCollisionType::obs);
        return false;
      }

      // if (i == 0) {
      //   break;
      // }
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

  for (size_t i = check_start_index; i < point_size; ++i) {
    // check bound
    if (IsPointBeyondBound(path[i].x, path[i].y)) {
      node->SetCollisionType(NodeCollisionType::map_bound);
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

    // if (edt_->IsCollisionForPoint(&tf, AstarPathGear::none)) {
    if (edt_->IsCollisionForPoint(&tf, path[i].gear)) {
      node->SetCollisionType(NodeCollisionType::obs);
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

    if (edt_->IsCollisionForPoint(&tf, result->gear[i])) {
      return i;
    }
  }

  return collision_index;
}

void HybridAStar::GetPathByBicycleModel(NodePath* path, const double arc,
                                        const double radius,
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

void HybridAStar::GetPathByCircle(NodePath* path, const double arc,
                                  const double radius, const bool is_forward) {
  int path_point_num = std::ceil(arc / node_path_dist_resolution_);
  // ILOG_INFO << "path point num " << path_point_num;
  // ILOG_INFO << "node path s resolution " << traveled_distance;

  // get vehicle circle
  VehicleCircle veh_circle;
  AstarPathGear gear;
  if (is_forward) {
    gear = AstarPathGear::drive;
  } else {
    gear = AstarPathGear::reverse;
  }

  GetVehCircleByPose(&veh_circle, &path->points[0], radius, gear);

  // interpolate
  path->path_dist = 0;

  Pose2D* start_pose;
  Pose2D* next_pose;
  start_pose = &path->points[0];

  double acc_s = 0.0;

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

void HybridAStar::GetPathByLine(NodePath* path, const double arc,
                                const bool is_forward) {
  int path_point_num = std::ceil(arc / node_path_dist_resolution_);
  // ILOG_INFO << "path point num " << path_point_num;
  // ILOG_INFO << "node path s resolution " << traveled_distance;

  double inc_dist;
  if (is_forward) {
    inc_dist = node_path_dist_resolution_;
  } else {
    inc_dist = -node_path_dist_resolution_;
  }

  path->path_dist = 0;
  double acc_s = 0.0;

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
  double front_wheel_angle = 0.0;
  double radius = 0.0;
  double traveled_distance = 0.0;

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

  // take above motion primitive to generate a curve driving the car to a
  // different grid
  // double arc = std::sqrt(2) * xy_grid_resolution_;
  double arc = config_.node_step;

  NodePath path;
  path.point_size = 1;
  path.points[0].x = parent_node->GetX();
  path.points[0].y = parent_node->GetY();
  path.points[0].theta = parent_node->GetPhi();

  bool is_forward = traveled_distance > 0.0 ? true : false;

  // generate path by circle
  // if (std::fabs(front_wheel_angle) > 0.0001) {
  //   GetPathByCircle(&path, arc, radius, is_forward);
  // } else {
  //   GetPathByLine(&path, arc, is_forward);
  // }

  // generate path by bycicle model
  GetPathByBicycleModel(&path, arc, radius, is_forward);

  // check if the vehicle runs outside of XY boundary
  const Pose2D& end_point = path.GetEndPoint();
  if (IsPointBeyondBound(end_point.x, end_point.y)) {
    return NodeShrinkType::OUT_OF_BOUNDARY;
  }

  // todo, check collision

  // maybe search the same point, but new a node?
  new_node->Set(path, XYbounds_, config_, path.path_dist);

  // check search bound
  if (!NodeInSearchBound(new_node->GetIndex())) {
    new_node->ClearPath();
    return NodeShrinkType::OUT_OF_BOUNDARY;
  }

  bool heading_legal = false;
  heading_legal = node_shrink_decider_.IsLegalForHeading(new_node->GetPhi());
  if (!heading_legal) {
    new_node->ClearPath();

    // ILOG_INFO << "heading is illegal";
    return NodeShrinkType::UNEXPECTED_HEADING;
  }

  new_node->SetPre(parent_node);

  AstarPathGear gear;
  if (traveled_distance > 0.0) {
    gear = AstarPathGear::drive;
  } else {
    gear = AstarPathGear::reverse;
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

  // ILOG_INFO << "next node end";
  // next_node->DebugString();

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
  double optimal_path_cost = 0.0;
  double dp_path_dist = 0.0;

  double dp_path_cost = 0.0;
  if (config_.enable_dp_cost_for_vertical_park) {
    dp_path_dist = ObstacleHeuristicWithHolonomic(next_node);
    dp_path_cost = dp_path_dist * config_.traj_forward_penalty;
    cost.astar_dist = dp_path_cost;
  }

  double rs_path_cost = 0.0;
  if (config_.enable_rs_path_h_cost_for_vertical_park) {
    rs_path_cost = GenerateHeuristicCostByRsPath(next_node, &cost);
  }

  optimal_path_cost = std::max(dp_path_cost, rs_path_cost);

  // heading cost
  double ref_line_heading_cost = 0.0;
  if (config_.enable_ref_line_h_cost_for_vertical_park) {
    ref_line_heading_cost =
        GenerateRefLineHeuristicCost(next_node, dp_path_dist);
    cost.ref_line_heading_cost = ref_line_heading_cost;
  }

  optimal_path_cost = std::max(optimal_path_cost, ref_line_heading_cost);
  // optimal_path_cost += ref_line_heading_cost;

  // euler cost
  double euler_dist_cost = 0.0;
  if (config_.enable_euler_cost_for_vertical_park) {
    euler_dist_cost = next_node->GetEulerDist(astar_end_node_);

    cost.euler_dist = euler_dist_cost;
  }

  optimal_path_cost = std::max(euler_dist_cost, optimal_path_cost);

  next_node->SetHeuCost(optimal_path_cost);
  next_node->SetHeuCostDebug(cost);

#if LOG_TIME_PROFILE
  const double end_time = IflyTime::Now_ms();
  heuristic_time_ += end_time - start_time;
#endif

  return;
}

void HybridAStar::GetSingleShotNodeHeuCost(const Node3d* father_node,
                                           Node3d* next_node) {
  NodeHeuristicCost cost;
  double optimal_path_cost = 0.0;

  // euler cost
  double euler_dist_cost = 0.0;
  euler_dist_cost = next_node->GetEulerDist(astar_end_node_);
  cost.euler_dist = euler_dist_cost;

  optimal_path_cost = std::max(euler_dist_cost, optimal_path_cost);

  next_node->SetHeuCost(optimal_path_cost);
  next_node->SetHeuCostDebug(cost);

  return;
}

void HybridAStar::CalculateNodeGCost(Node3d* current_node, Node3d* next_node) {
  next_node->SetGCost(current_node->GetGCost() +
                      CalcGCostToParentNode(current_node, next_node));

  return;
}

double HybridAStar::GenerateHeuristicCostByRsPath(Node3d* next_node,
                                                  NodeHeuristicCost* cost) {
  RSPathRequestType rs_request = RSPathRequestType::none;
  if (!CalcRSPathToGoal(next_node, false, rs_request,
                        vehicle_param_.min_turn_radius)) {
    ILOG_INFO << "ShortestRSP failed";
    return 100.0;
  }

  double path_dist = std::fabs(rs_path_.total_length);

  double dist_cost = path_dist * config_.traj_forward_penalty;
  cost->rs_path_dist = dist_cost;

  double gear_cost = 0.0;
  double steer_change_cost = 0.0;

  for (int i = 0; i < rs_path_.size - 1; i++) {
    // gear cost
    if (rs_path_.paths[i].gear != rs_path_.paths[i + 1].gear) {
      gear_cost += config_.gear_switch_penalty_heu;
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
  if (next_node->GetGearType() != rs_path_.paths[0].gear) {
    gear_cost += config_.gear_switch_penalty_heu;
  }

  // steer cost
  if (next_node->GetSteer() > 0.0 && rs_path_.paths[0].steer != RS_LEFT) {
    steer_change_cost += config_.traj_steer_change_penalty * max_steer_angle_;
  } else if (next_node->GetSteer() < 0.0 &&
             rs_path_.paths[0].steer != RS_RIGHT) {
    steer_change_cost += config_.traj_steer_change_penalty * max_steer_angle_;
  }

  cost->rs_path_gear = gear_cost;
  cost->rs_path_steer = steer_change_cost;

  double collision_cost = 0.0;

#if PLOT_RS_COST_PATH
  if (rs_path_h_cost_debug_.size() < RS_H_COST_MAX_NUM) {
    const Pose2D& rs_start_pose = next_node->GetPose();
    rs_path_interface_.RSPathInterpolate(&rs_path_, &rs_start_pose,
                                         vehicle_param_.min_turn_radius);
    rs_path_h_cost_debug_.emplace_back(rs_path_);
  }
#endif

  return dist_cost + gear_cost + steer_change_cost + collision_cost;
}

double HybridAStar::CalcGCostToParentNode(Node3d* current_node,
                                          Node3d* next_node) {
  // evaluate cost on the trajectory and add current cost
  double piecewise_cost = 0.0;
  double path_dist = 0.0;
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
  double safe_punish = 0.0;
  if (config_.enable_obs_dist_g_cost) {
    safe_punish = CalcSafeDistCost(next_node);
  }
  piecewise_cost += safe_punish;

  // ref line heading cost
  // double heading_cost =
  //     std::fabs(GetThetaDiff(next_node->GetPhi(), ref_line_.GetHeading())) *
  //     config_.ref_line_heading_penalty;
  // piecewise_cost += heading_cost;

  return piecewise_cost;
}

void HybridAStar::GetSingleShotNodeGCost(Node3d* current_node,
                                         Node3d* next_node) {
  // evaluate cost on the trajectory and add current cost
  double piecewise_cost = 0.0;
  double path_dist = 0.0;
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
  // double heading_cost =
  //     std::fabs(GetThetaDiff(next_node->GetPhi(), ref_line_.GetHeading())) *
  //     config_.ref_line_heading_penalty;
  // piecewise_cost += heading_cost;

  next_node->SetGCost(current_node->GetGCost() + piecewise_cost);

  return;
}

double HybridAStar::ObstacleHeuristicWithHolonomic(Node3d* next_node) {
  return dp_heuristic_generator_->CheckDpMap(next_node->GetX(),
                                             next_node->GetY());
}

double HybridAStar::GenerateHeuristicCost(Node3d* next_node) {
  double h_cost = 0.0;

  h_cost += ObstacleHeuristicWithHolonomic(next_node);

  return h_cost;
}

double HybridAStar::GenerateRefLineHeuristicCost(Node3d* next_node,
                                                 const double dist_to_go) {
  double dist_cost = 0.0;
  // if (0) {
  //   // position cost
  //   ad_common::math::Vec2d point;
  //   point.set_x(next_node->GetX());
  //   point.set_y(next_node->GetY());

  //   ad_common::math::Vec2d line = point - ref_line_.GetStartPoint();
  //   double lateral_dist = ref_line_.UnitDirection().CrossProd(line);

  //   dist_cost = std::fabs(lateral_dist) * config_.traj_forward_penalty;
  // }

  // heading cost
  double theta1 = next_node->GetPhi();
  double theta2 = ref_line_->GetHeading();

  double heading_cost_weight = 2.0;
  double heading_cost = dist_to_go + std::fabs(GetThetaDiff(theta1, theta2)) *
                                         heading_cost_weight;

  // double heading_cost = std::fabs(GetThetaDiff(theta1, theta2)) *
  //                       config_.ref_line_heading_penalty;

#if DEBUG_TARGET_HEADING_COST

  ILOG_INFO << "heading " << next_node->GetPhi() * 57.3 << " dist cost "
            << dist_cost << " heading cost " << heading_cost;

#endif

  return std::max(dist_cost, heading_cost);
}

const bool HybridAStar::BackwardPassByRSPath(HybridAStarResult* result,
                                             Node3d* rs_node_to_goal) {
  Node3d* parent_node = nullptr;
  Node3d* child_node = rs_node_to_goal;
  rs_node_to_goal->SetNext(nullptr);

  result->base_pose = request_.base_pose_;
  result->gear_change_num = 0;

  // all nodes
  std::vector<Node3d*> node_list;

  ILOG_INFO << "get result start backward pass by rs";

  // backward pass
  int i = 0;
  while (child_node->GetPreNode() != nullptr) {
    parent_node = child_node->GetPreNode();

    parent_node->SetNext(child_node);

    child_node = parent_node;

    // add break for dead loop
    i++;
    if (i > backward_pass_max_point) {
      ILOG_INFO << " i " << i;
      return false;
    }
  }

  i = 0;
  size_t point_size;
  double kappa;

  AstarPathGear last_gear_type = AstarPathGear::none;
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
    if (last_gear_type != AstarPathGear::none) {
      if (last_gear_type != cur_gear_type) {
        result->gear_change_num++;
      }
    }

    node_list.push_back(child_node);

    last_gear_type = cur_gear_type;
    parent_node = child_node;
    child_node = child_node->GetMutableNextNode();

    // if (parent_node != nullptr) {
    //   Pose2D end = parent_node->GetNodePath()
    //                    .points[parent_node->GetNodePath().point_size - 1];

    //   Pose2D start = child_node->GetNodePath().points[0];

    //   if (CalcPointDist(&end, &start) < 0.1) {
    //     ILOG_INFO << "par";
    //     parent_node->DebugString();
    //     if (parent_node->GetPreNode() != nullptr) {
    //       ILOG_INFO << "par par";
    //       parent_node->GetPreNode()->DebugString();
    //     }

    //     ILOG_INFO << "child ";
    //     child_node->DebugString();
    //     if (child_node->GetPreNode() != nullptr) {
    //       ILOG_INFO << "child p ====";
    //       child_node->GetPreNode()->DebugString();
    //     }
    //   }
    // }

    // add break for dead loop, will be retired.
    i++;
    if (i > backward_pass_max_point) {
      ILOG_ERROR << " i " << i;
      return false;
    }
  }

  // get rs path
  AstarPathType path_type = child_node->GetPathType();
  if (child_node != nullptr && child_node->IsRsPath()) {
    for (int seg_id = 0; seg_id < rs_path_.size; seg_id++) {
      const RSPathSegment* segment = &rs_path_.paths[seg_id];

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
      if (last_gear_type != AstarPathGear::none) {
        if (last_gear_type != cur_gear_type) {
          result->gear_change_num++;
        }
      }

      last_gear_type = cur_gear_type;
    }
  }

  // link rs end to astar end
  if (!rs_expansion_decider_.IsSameEndPointForRsWithAstar()) {
    LinkRsToAstarEndPoint(result, astar_end_node_->GetPose());
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

  double accumulated_s = 0.0;
  result->accumulated_s.clear();
  auto last_x = result->x.front();
  auto last_y = result->y.front();
  double x_diff;
  double y_diff;
  for (size_t i = 0; i < path_points_size; ++i) {
    x_diff = result->x[i] - last_x;
    y_diff = result->y[i] - last_y;
    accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
    result->accumulated_s.push_back(accumulated_s);
    last_x = result->x[i];
    last_y = result->y[i];
  }

  ILOG_INFO << "get result finish, path point size " << result->x.size();

  result->fail_type = AstarFailType::success;

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

  DebugPolynomialPath(poly_path);

  // backward pass
  int i = 0;
  while (child_node->GetPreNode() != nullptr) {
    parent_node = child_node->GetPreNode();

    parent_node->SetNext(child_node);

    child_node = parent_node;
  }

  i = 0;
  size_t point_size;
  double kappa;

  AstarPathGear last_gear_type = AstarPathGear::none;
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
    if (last_gear_type != AstarPathGear::none) {
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

  double accumulated_s = 0.0;
  result->accumulated_s.clear();
  auto last_x = result->x.front();
  auto last_y = result->y.front();
  double x_diff;
  double y_diff;
  for (size_t i = 0; i < path_points_size; ++i) {
    x_diff = result->x[i] - last_x;
    y_diff = result->y[i] - last_y;
    accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
    result->accumulated_s.push_back(accumulated_s);
    last_x = result->x[i];
    last_y = result->y[i];
  }

  ILOG_INFO << "get result finish, path point size " << result->x.size();

  result->fail_type = AstarFailType::success;

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

  Eigen::Vector2d rs_end_point;

  rs_end_point[0] = result->x.back();
  rs_end_point[1] = result->y.back();

  double x_diff = std::fabs(rs_end_point[0] - astar_end.x);
  double y_diff = std::fabs(rs_end_point[1] - astar_end.y);
  double length = std::sqrt(x_diff * x_diff + y_diff * y_diff);

  double phi = result->phi.back();
  AstarPathGear gear = result->gear.back();
  AstarPathType path_type = AstarPathType::LINE_SEGMENT;

  const Eigen::Vector2d unit_line_vec = Eigen::Vector2d(-1.0, 0.0);
  double s = 0.1;
  double ds = 0.1;

  Eigen::Vector2d point;
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

void HybridAStar::SingleShotPathAttempt(const MapBound& XYbounds,
                                        const ParkObstacleList& obstacles,
                                        const AstarRequest& request,
                                        const ObstacleClearZone *clear_zone,
                                        HybridAStarResult* result,
                                        EulerDistanceTransform* edt,
                                        ParkReferenceLine* ref_line) {
  result->Clear();

  if (request.history_gear == AstarPathGear::reverse) {
    ILOG_INFO << "history gear is reverse";
    return;
  }

  if (request.space_type != ParkSpaceType::vertical) {
    ILOG_INFO << "vertical";
    return;
  }

  if (request.parking_task != ParkingTask::parking_in) {
    ILOG_INFO << "parking_in";
    return;
  }

  const Pose2D start = request.start_;
  Pose2D end = request.real_goal;
  end.x += config_.dp_search_goal_adjust_dist;

  if (start.GetX() < 1.0) {
    ILOG_INFO << "start.GetX() =" << start.GetX();
    return;
  }

  if (start.GetY() < -5.0 || start.GetY() > 5.0) {
    ILOG_INFO << "start.GetY() =" << start.GetY();
    return;
  }

  double heading = IflyUnifyTheta(start.GetPhi(), M_PI);
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
  ILOG_INFO << "gear reverse searching";

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

    result->fail_type = AstarFailType::out_of_bound;
    return;
  }

  // start
  start_node_ = node_pool_.AllocateNode();

  // load nodes and obstacles
  if (start_node_ == nullptr) {
    ILOG_ERROR << "start node nullptr";

    result->fail_type = AstarFailType::allocate_node_fail;
    return;
  }

  start_node_->Set(NodePath(start), XYbounds_, config_, 0.0);
  if (!start_node_->IsNodeValid()) {
    ILOG_ERROR << "start_node invalid";

    result->fail_type = AstarFailType::out_of_bound;
    return;
  }

  // in searching, start node gear is forward or backward which will be ok.
  start_node_->SetGearType(AstarPathGear::none);
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
    ILOG_ERROR << "start_node in collision with obstacles "
               << static_cast<int>(start_node_->GetConstCollisionType());

    // start_node_->DebugString();

    result->fail_type = AstarFailType::start_collision;
    return;
  }
  double check_end_time = IflyTime::Now_ms();
  collision_check_time_ms_ += check_end_time - check_start_time;

  // allocate end
  astar_end_node_ = node_pool_.AllocateNode();
  if (astar_end_node_ == nullptr) {
    ILOG_ERROR << "end node nullptr";

    result->fail_type = AstarFailType::out_of_bound;
    return;
  }
  astar_end_node_->Set(NodePath(end), XYbounds_, config_, 0.0);
  astar_end_node_->SetGearType(AstarPathGear::none);
  astar_end_node_->SetPathType(AstarPathType::END_NODE);
  astar_end_node_->DebugString();

  if (!astar_end_node_->IsNodeValid()) {
    ILOG_ERROR << "end_node invalid";

    result->fail_type = AstarFailType::out_of_bound;
    return;
  }

  // check end
  check_start_time = IflyTime::Now_ms();
  if (!ValidityCheckByEDT(astar_end_node_)) {
    ILOG_INFO << "end_node in collision with obstacles "
               << static_cast<int>(astar_end_node_->GetConstCollisionType());

    astar_end_node_->DebugString();
    result->fail_type = AstarFailType::goal_collision;
    return;
  }

  check_end_time = IflyTime::Now_ms();
  collision_check_time_ms_ += check_end_time - check_start_time;

  // node shrink related
  node_shrink_decider_.Process(start, end);
  rs_expansion_decider_.Process(vehicle_param_.min_turn_radius,
                                request_.slot_width, request_.slot_length,
                                start, end, vehicle_param_.width);

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
  // 100 ms
  constexpr double dp_max_search_time = 100.0;

  Node3d* current_node = nullptr;
  Node3d* next_node_in_pool = nullptr;
  Node3d new_node;
  Node3d rs_node_to_goal;
  rs_node_to_goal.Clear();

  AstarNodeVisitedType vis_type;
  PathGearRequest gear_request = PathGearRequest::GEAR_REVERSE_ONLY;
  bool is_safe = false;
  double child_node_dist;
  double father_node_dist;
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
      ILOG_INFO <<"pq is null node";
      continue;
    }

    current_node->SetVisitedType(AstarNodeVisitedType::in_close);

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
        ad_common::math::Vec2d(current_node->GetX(), current_node->GetY()));
#endif

    // check if an analystic curve could be connected from current
    // configuration to the end configuration without collision. if so,
    // search ends.
    double current_time = IflyTime::Now_ms();

    // if bigger than 100 ms，break
    astar_search_time = current_time - astar_search_start_time;
    if (astar_search_time > dp_max_search_time) {
      ILOG_INFO << "time out " << astar_search_time;
      break;
    }

    if (ExpansionByQunticPolynomial(current_node, poly_path, &polynomial_node,
                                    &poly_path_fail_type)) {
      ILOG_INFO << "polynomial success";

      current_node->DebugPoseString();

      ILOG_INFO << "path " << poly_path[0].x << ",y " << poly_path[0].y
                << ",heading = " << poly_path[0].phi * 57.4
                << ",radius = " << current_node->GetRadius();

      break;
    } else if (AnalyticExpansionByRS(current_node, gear_request,
                                     &rs_node_to_goal)) {
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

        vis_type = AstarNodeVisitedType::not_visited;
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
      if (vis_type == AstarNodeVisitedType::not_visited) {
        GetSingleShotNodeHeuCost(current_node, &new_node);

        h_cost_rs_path_num++;

        // next_node_in_pool->CopyNode(&new_node);
        *next_node_in_pool = new_node;
        next_node_in_pool->SetFCost();
        next_node_in_pool->SetVisitedType(AstarNodeVisitedType::in_open);

        next_node_in_pool->SetMultiMapIter(open_pq_.insert(
            std::make_pair(next_node_in_pool->GetFCost(), next_node_in_pool)));

        node_set_.emplace(next_node_in_pool->GetGlobalID(), next_node_in_pool);

#if DEBUG_ONE_SHOT_PATH
        if (explored_node_num < DEBUG_ONE_SHOT_PATH_MAX_NODE) {
          ILOG_INFO << "new point";
          next_node_in_pool->DebugCost();
        }
#endif

      } else if (vis_type == AstarNodeVisitedType::in_open) {
        // in open set and need update
        if (new_node.GetGCost() < next_node_in_pool->GetGCost()) {
          GetSingleShotNodeHeuCost(current_node, &new_node);

          h_cost_rs_path_num++;

          open_pq_.erase(next_node_in_pool->GetMultiMapIter());

          *next_node_in_pool = new_node;
          next_node_in_pool->SetFCost();
          next_node_in_pool->SetVisitedType(AstarNodeVisitedType::in_open);

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
        if (new_node.GetGCost() < next_node_in_pool->GetGCost()) {
          GetSingleShotNodeHeuCost(current_node, &new_node);

          h_cost_rs_path_num++;

          *next_node_in_pool = new_node;
          next_node_in_pool->SetFCost();
          next_node_in_pool->SetVisitedType(AstarNodeVisitedType::in_open);

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
    BackwardPassByRSPath(result, &rs_node_to_goal);
  } else {
    result->fail_type = AstarFailType::search_too_much_node;
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
  DebugAstarRequestString(request_);

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

    result->fail_type = AstarFailType::out_of_bound;
    return false;
  }

  // start
  start_node_ = node_pool_.AllocateNode();

  // load nodes and obstacles
  if (start_node_ == nullptr) {
    ILOG_ERROR << "start node nullptr";

    result->fail_type = AstarFailType::allocate_node_fail;
    return false;
  }

  start_node_->Set(NodePath(start), XYbounds_, config_, 0.0);
  if (!start_node_->IsNodeValid()) {
    ILOG_ERROR << "start_node invalid";

    result->fail_type = AstarFailType::out_of_bound;
    return false;
  }

  // in searching, start node gear is forward or backward which will be ok.
  start_node_->SetGearType(AstarPathGear::none);
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
    ILOG_ERROR << "start_node in collision with obstacles "
               << static_cast<int>(start_node_->GetConstCollisionType());

    // start_node_->DebugString();

    result->fail_type = AstarFailType::start_collision;
    return false;
  }
  double check_end_time = IflyTime::Now_ms();
  collision_check_time_ms_ += check_end_time - check_start_time;

  // allocate end
  astar_end_node_ = node_pool_.AllocateNode();

  if (astar_end_node_ == nullptr) {
    ILOG_ERROR << "end node nullptr";

    result->fail_type = AstarFailType::out_of_bound;
    return false;
  }
  astar_end_node_->Set(NodePath(end), XYbounds_, config_, 0.0);
  astar_end_node_->SetGearType(AstarPathGear::none);
  astar_end_node_->SetPathType(AstarPathType::END_NODE);
  astar_end_node_->DebugString();

  if (!astar_end_node_->IsNodeValid()) {
    ILOG_ERROR << "end_node invalid";

    result->fail_type = AstarFailType::out_of_bound;
    return false;
  }

  // check end
  check_start_time = IflyTime::Now_ms();
  if (!ValidityCheckByEDT(astar_end_node_)) {
    ILOG_INFO << "end_node in collision with obstacles "
               << static_cast<int>(astar_end_node_->GetConstCollisionType());

    astar_end_node_->DebugString();
    result->fail_type = AstarFailType::goal_collision;
    return false;
  }

  check_end_time = IflyTime::Now_ms();
  collision_check_time_ms_ += check_end_time - check_start_time;

  const double map_time = IflyTime::Now_ms();

  ILOG_INFO << "init time(ms) " << map_time - astar_start_time;

  // ILOG_INFO << "time " << IflyTime::DateString();

  // node shrink related
  node_shrink_decider_.Process(start, end);

  // generate a star dist
  if (dp_heuristic_generator_ == nullptr) {
    ILOG_ERROR << "h cost is null";

    result->fail_type = AstarFailType::dp_cost_fail;
    return false;
  }
  dp_heuristic_generator_->GenerateDpMap(end.x, end.y, XYbounds, obstacles_,
                                         car_half_width_);

  const double end_timestamp = IflyTime::Now_ms();
  const double time_consumption = end_timestamp - map_time;

  ILOG_INFO << "map time(ms) " << time_consumption;

  rs_expansion_decider_.Process(vehicle_param_.min_turn_radius,
                                request_.slot_width, request_.slot_length,
                                start, end, vehicle_param_.width);

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

  while (!open_pq_.empty()) {
    // take out the lowest cost neighboring node

    current_node = open_pq_.begin()->second;
    open_pq_.erase(open_pq_.begin());
    if (current_node == nullptr) {
      ILOG_INFO <<"pq is null node";
      continue;
    }

    current_node->SetVisitedType(AstarNodeVisitedType::in_close);

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
        ad_common::math::Vec2d(current_node->GetX(), current_node->GetY()));
#endif

    // check if an analystic curve could be connected from current
    // configuration to the end configuration without collision. if so,
    // search ends.
    double current_time = IflyTime::Now_ms();

    // if bigger than 60 s，break
    astar_search_time = current_time - astar_search_start_time;
    if (astar_search_time > max_search_time * 1000.0) {
      ILOG_INFO << "time out " << astar_search_time;
      break;
    }

    if (AnalyticExpansionByRS(current_node, gear_request, &rs_node_to_goal)) {
      ILOG_INFO << "RS success";

      break;
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
        if (explored_node_num < DEBUG_NODE_MAX_NUM) {
          child_node_debug_.emplace_back(
              DebugAstarSearchPoint(new_node.GetX(), new_node.GetY(), false));
        }
#endif
        continue;
      }

      // allocate new node
      if (node_set_.find(new_node.GetGlobalID()) == node_set_.end()) {
        next_node_in_pool = node_pool_.AllocateNode();

        vis_type = AstarNodeVisitedType::not_visited;
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
      if (vis_type == AstarNodeVisitedType::not_visited) {
        CalculateNodeHeuristicCost(current_node, &new_node);

        h_cost_rs_path_num++;

        // next_node_in_pool->CopyNode(&new_node);
        *next_node_in_pool = new_node;
        next_node_in_pool->SetFCost();
        next_node_in_pool->SetVisitedType(AstarNodeVisitedType::in_open);

        next_node_in_pool->SetMultiMapIter(open_pq_.insert(
            std::make_pair(next_node_in_pool->GetFCost(), next_node_in_pool)));

        node_set_.emplace(next_node_in_pool->GetGlobalID(), next_node_in_pool);

#if DEBUG_CHILD_NODE
        if (explored_node_num < DEBUG_NODE_MAX_NUM) {
          ILOG_INFO << "new point";
          next_node_in_pool->DebugCost();
        }
#endif

      } else if (vis_type == AstarNodeVisitedType::in_open) {
        // in open set and need update
        if (new_node.GetGCost() < next_node_in_pool->GetGCost()) {
          CalculateNodeHeuristicCost(current_node, &new_node);

          h_cost_rs_path_num++;

          open_pq_.erase(next_node_in_pool->GetMultiMapIter());

          *next_node_in_pool = new_node;
          next_node_in_pool->SetFCost();
          next_node_in_pool->SetVisitedType(AstarNodeVisitedType::in_open);

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
        if (new_node.GetGCost() < next_node_in_pool->GetGCost()) {
          CalculateNodeHeuristicCost(current_node, &new_node);

          h_cost_rs_path_num++;

          *next_node_in_pool = new_node;
          next_node_in_pool->SetFCost();
          next_node_in_pool->SetVisitedType(AstarNodeVisitedType::in_open);

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
      if (explored_node_num < DEBUG_NODE_MAX_NUM) {
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
  if (rs_node_to_goal.IsNodeValid()) {
    BackwardPassByRSPath(result, &rs_node_to_goal);
  } else {
    result->fail_type = AstarFailType::search_too_much_node;
  }

  ILOG_INFO << "explored node num is " << explored_node_num
            << " ,rs path size is: " << explored_rs_path_num
            << " ,h cost rs num " << h_cost_rs_path_num
            << " ,node pool size:" << node_pool_.PoolSize();

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

  if (config_.lat_hierarchy_safe_buffer.size() > 0) {
    UpdateCarBoxBySafeBuffer(config_.lat_hierarchy_safe_buffer[0],
                             config_.lon_hierarchy_safe_buffer[0]);
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

void HybridAStar::GetRSPathForDebug(std::vector<double>& x,
                                    std::vector<double>& y,
                                    std::vector<double>& phi) {
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
            << result->type.size() << "s size " <<
            result->accumulated_s.size();

  for (size_t i = 0; i < result->x.size(); i++) {
    ILOG_INFO << "i = " << i << " x, y, theta, gear:  " << result->x[i] << ", "
              << result->y[i] << ", " << result->phi[i] * 57.4 << ", "
              << PathGearDebugString(result->gear[i]) << ",paht type = "
              << static_cast<int>(result->type[i]) << ", s = "
              << result->accumulated_s[i];
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

const bool HybridAStar::IsPointBeyondBound(const double x,
                                           const double y) const {
  if (x > XYbounds_.x_max || x < XYbounds_.x_min ||
      y > XYbounds_.y_max || y < XYbounds_.y_min) {
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
    std::vector<std::vector<Eigen::Vector2d>>& list) {
  for (auto i = node_set_.begin(); i != node_set_.end(); i++) {
    if (i->second->GetStepSize() < 2 || i->second->IsRsPath()) {
      continue;
    }

    const NodePath& path = i->second->GetNodePath();

    std::vector<Eigen::Vector2d> node;
    for (size_t m = 0; m < path.point_size; m++) {
      node.emplace_back(Eigen::Vector2d(path.points[m].x, path.points[m].y));
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
  if (gear == AstarPathGear::drive) {
    return &veh_polygon_gear_drive_;
  } else if (gear == AstarPathGear::reverse) {
    return &veh_polygon_gear_reverse_;
  }

  return &veh_polygon_gear_none_;
}

const std::vector<DebugAstarSearchPoint>& HybridAStar::GetChildNodeForDebug() {
  ILOG_INFO << "child node size" << child_node_debug_.size();
  return child_node_debug_;
}

const std::vector<ad_common::math::Vec2d>& HybridAStar::GetQueuePathForDebug() {
  return queue_path_debug_;
}

const std::vector<RSPath>& HybridAStar::GetRSPathHeuristic() {
  return rs_path_h_cost_debug_;
}

void HybridAStar::KineticsModel(const Pose2D* old_pose, const double radius,
                                Pose2D* pose, const bool is_forward) {
  double dist = is_forward ? kinetics_model_step_ : -kinetics_model_step_;
  pose->x = old_pose->x + dist * std::cos(old_pose->theta);
  pose->y = old_pose->y + dist * std::sin(old_pose->theta);

  double new_phi;
  new_phi = old_pose->theta + dist / radius;
  pose->theta = ad_common::math::NormalizeAngle(new_phi);

  return;
}

void HybridAStar::UpdatePoseByPathPointInterval(const Pose2D* old_pose,
                                                const double radius,
                                                const double interval,
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
                                             const double radius,
                                             const int number, Pose2D* pose,
                                             const bool is_forward) {
  Pose2D start = *old_pose;

  for (int i = 0; i < number; i++) {
    KineticsModel(&start, radius, pose, is_forward);

    start = *pose;
  }

  return;
}

double HybridAStar::CalcSafeDistCost(Node3d* node) {
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
                                        const double arc,
                                        const double inverse_radius) {
  double delta_theta, theta;

  delta_theta = arc * inverse_radius;

  // left turn
  if (veh_circle->radius > 0.0) {
    if (veh_circle->gear == AstarPathGear::reverse) {
      delta_theta = -delta_theta;
    }
  } else {
    // right turn, gear is d
    if (veh_circle->gear == AstarPathGear::drive) {
      delta_theta = -delta_theta;
    }
  }

  // update next point theta
  theta = start_pose->theta + delta_theta;

  double radius = veh_circle->radius;

  pose->x = veh_circle->center.x + radius * std::sin(theta);
  pose->y = veh_circle->center.y - radius * std::cos(theta);
  pose->theta = IflyUnifyTheta(theta, M_PI);

  return 1;
}

// radius: if left turn, radius is positive
int HybridAStar::GetVehCircleByPose(VehicleCircle* veh_circle,
                                    const Pose2D* pose, const double radius,
                                    const AstarPathGear gear) {
  veh_circle->radius = radius;
  veh_circle->gear = gear;

  veh_circle->center.x = pose->x - radius * std::sin(pose->theta);
  veh_circle->center.y = pose->y + radius * std::cos(pose->theta);

  return 1;
}

int HybridAStar::GetStraightLinePoint(Pose2D* goal_state,
                                      const Pose2D* start_state,
                                      const double dist_to_start,
                                      const Pose2D* unit_vector) {
  goal_state->x = start_state->x + dist_to_start * unit_vector->x;
  goal_state->y = start_state->y + dist_to_start * unit_vector->y;

  goal_state->theta = start_state->theta;

  return 1;
}

void HybridAStar::UpdateCarBoxBySafeBuffer(const double lat_buffer,
                                           const double lon_buffer) {
  // gear d
  double safe_half_width =
      (vehicle_param_.width + config_.width_mirror * 2 + lat_buffer * 2) * 0.5;

  GetRightUpCoordinatePolygonByParam(
      &veh_polygon_gear_drive_,
      config_.rear_overhanging + config_.lon_min_safe_buffer,
      vehicle_param_.wheel_base + config_.front_overhanging + lon_buffer,
      safe_half_width);

  // gear r
  GetRightUpCoordinatePolygonByParam(
      &veh_polygon_gear_reverse_, config_.rear_overhanging + lon_buffer,
      vehicle_param_.wheel_base + config_.front_overhanging +
          config_.lon_min_safe_buffer,
      safe_half_width);

  // gear none
  GetRightUpCoordinatePolygonByParam(
      &veh_polygon_gear_none_,
      config_.rear_overhanging + config_.lon_min_safe_buffer,
      vehicle_param_.wheel_base + config_.front_overhanging +
          config_.lon_min_safe_buffer,
      safe_half_width);

  ILOG_INFO << "lat buffer = " << lat_buffer;

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

void HybridAStar::RSPathCandidateByRadius(
    HybridAStarResult* result, const Pose2D& start, const Pose2D& end,
    const double lon_min_sampling_length, const double radius) {
  bool is_connected_to_goal;
  RSPathRequestType rs_request = RSPathRequestType::none;

  // sampling for path end
  // sampling start point: move start point forward dist (expected_path_dist)
  Pose2D sampling_end = start;
  sampling_end.y = 0.0;
  sampling_end.theta = 0.0;
  sampling_end.x = start.x + lon_min_sampling_length;

  double sampling_step = 0.2;
  size_t max_sampling_num = std::ceil((end.x - sampling_end.x) / sampling_step);
  double sampling_radius = radius;

  result->Clear();
  HybridAStarResult path;
  path.Clear();
  HybridAStarResult best_path;
  best_path.Clear();

  size_t best_path_valid_point_size = 0;
  size_t path_valid_point_size = 1000;
  size_t expected_dist_point_size = 0;

  for (size_t k = 0; k < max_sampling_num; k++) {
    sampling_end.x += sampling_step;
    rs_path_interface_.GeneShortestRSPath(&rs_path_, &is_connected_to_goal,
                                          &start, &sampling_end,
                                          sampling_radius, true, rs_request);


#if 0
  if (rs_path_h_cost_debug_.size() < RS_H_COST_MAX_NUM) {
    rs_path_h_cost_debug_.emplace_back(rs_path_);
  }
#endif

    if (rs_path_.total_length < 0.01 || !is_connected_to_goal) {
      ILOG_INFO << "rs path fail";

      continue;
    }

    // check gear
    bool has_reverse = false;
    for (int j = 0; j < rs_path_.size; j++) {
      if (rs_path_.paths[j].gear == AstarPathGear::reverse) {
        // ILOG_INFO << " rs path seg need single shot by drive gear ";
        has_reverse = true;
        break;
      }
    }
    if (has_reverse) {
      ILOG_INFO << "gear is invalid";
      continue;
    }

    path.Clear();
    RSPathSegment* last_segment = nullptr;
    for (int i = 0; i < rs_path_.size; i++) {
      RSPathSegment* segment = &rs_path_.paths[i];

      int point_id = 0;
      // delete first same point
      if (last_segment != nullptr) {
        if (segment->gear == last_segment->gear) {
          point_id = 1;
        }
      }

      for (; point_id < segment->size; point_id++) {
        path.x.emplace_back(segment->points[point_id].x);
        path.y.emplace_back(segment->points[point_id].y);
        path.phi.emplace_back(segment->points[point_id].theta);
        path.gear.emplace_back(segment->gear);
        path.type.emplace_back(AstarPathType::REEDS_SHEPP);
        path.kappa.emplace_back(segment->points[point_id].kappa);
      }

      last_segment = segment;
    }

    // get path lengh
    path_valid_point_size = path.x.size();

    double accumulated_s = 0.0;
    path.accumulated_s.clear();
    auto last_x = path.x.front();
    auto last_y = path.y.front();
    double x_diff;
    double y_diff;

    expected_dist_point_size = 0;
    for (size_t i = 0; i < path_valid_point_size; ++i) {
      x_diff = path.x[i] - last_x;
      y_diff = path.y[i] - last_y;
      accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
      path.accumulated_s.emplace_back(accumulated_s);

      if (accumulated_s <= lon_min_sampling_length) {
        expected_dist_point_size = i;
      }

      last_x = path.x[i];
      last_y = path.y[i];
    }

    path_valid_point_size =
        std::min(path_valid_point_size, expected_dist_point_size);

    // collision check
    size_t collision_id = GetPathCollisionIDByEDT(&path);
    // add extra lon buffer
    if (collision_id > 2) {
      collision_id -= 2;
    }

    path_valid_point_size = std::min(path_valid_point_size, collision_id);
    if (path_valid_point_size <= 1) {
      ILOG_INFO << "collision_id = " << collision_id << ", sampling id = " << k
                << ", max_sampling_num=" << max_sampling_num;
      continue;
    }

#if 0
    ILOG_INFO << "point size= " << path.x.size()
              << ",expected_dist_id= " << expected_dist_point_size
              << ", path len= "
              << ((path.accumulated_s.size() > 0) ? path.accumulated_s.back()
                                                  : 0.0)
              << ", path_valid_point_size=" << path_valid_point_size;
#endif

    if (best_path_valid_point_size < path_valid_point_size) {
      best_path_valid_point_size = path_valid_point_size;
      best_path = path;
    }

    if (path_valid_point_size >= expected_dist_point_size) {
      break;
    }
  }

  best_path_valid_point_size =
      std::min(best_path_valid_point_size, best_path.x.size());
  double valid_dist = 0.0;
  if (best_path_valid_point_size > 0) {
    if (best_path_valid_point_size < best_path.accumulated_s.size()) {
      valid_dist = best_path.accumulated_s[best_path_valid_point_size];
    } else {
      valid_dist = best_path.accumulated_s.back();
    }
  }

  if (valid_dist >= 1.2) {
    for (size_t i = 0; i < path_valid_point_size; i++) {
      result->x.emplace_back(best_path.x[i]);
      result->y.emplace_back(best_path.y[i]);
      result->phi.emplace_back(best_path.phi[i]);
      result->gear.emplace_back(best_path.gear[i]);
      result->type.emplace_back(best_path.type[i]);
      result->kappa.emplace_back(best_path.kappa[i]);
      result->accumulated_s.emplace_back(best_path.accumulated_s[i]);
    }
    result->base_pose = request_.base_pose_;

    ILOG_INFO << "path valid, point size= " << result->x.size();
  } else {
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
                                          const double start_radius,
                                          const Pose2D& end) {
  planning_math::QuinticPolynomialCurve1d curve;

  // attention: ref line is slot center line
  // polynomial start
  double x0 = end.y;
  double dx0 = 0.0;
  double ddx0 = 0.0;

  // polynomial end
  double x1 = start.y;
  double dx1 = std::tan(start.theta);
  double ddx1;
  if (std::fabs(start_radius > 1000.0)) {
    ddx1 = 0.0;
  } else {
    ddx1 = std::pow((1.0 + dx1 * dx1), 1.5) / start_radius;
  }

  // ref s
  double total_ref_s = start.x - end.x;
  curve.SetParam(x0, dx0, ddx0, x1, dx1, ddx1, total_ref_s);

  int point_num = std::ceil(total_ref_s / 0.05);
  double ref_s = 0;
  double kappa;
  double min_radius = vehicle_param_.min_turn_radius;
  double max_kappa = 1.0 / min_radius + 1e-5;

  // interpolate
  AStarPathPoint point;
  ref_s = total_ref_s;
  double accumulated_s = 0;
  double dist = 0;
  double theta;

  for (int i = 0; i <= point_num; i++) {
    const double x = curve.Evaluate(0, ref_s);
    const double dx = curve.Evaluate(1, ref_s);
    theta = std::atan(dx);
    kappa = curve.EvaluateKappa(ref_s);

    if (kappa > max_kappa || kappa < -max_kappa) {
      path.clear();

      // ILOG_INFO << "i=" << i << ",radius bound =" << min_radius
      //           << ",radius=" << 1 / kappa << ",accumulated_s=" << accumulated_s
      //           << ",x =" << end.x + ref_s << ",y=" << x << ",phi=" << theta;
      // start.DebugString();
      return;
    }

    point.x = end.x + ref_s;
    point.y = x;
    point.phi = theta;
    point.gear = AstarPathGear::reverse;
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
    ref_s = std::max(0.0, ref_s);
  }

  return;
}

void HybridAStar::DebugPolynomialPath(
    const std::vector<AStarPathPoint>& poly_path) {
  for (size_t i = 0; i < poly_path.size(); i++) {
    ILOG_INFO << "x = " << poly_path[i].x << ",y=" << poly_path[i].y
              << ",theta=" << poly_path[i].phi
              << ",kappa=" << poly_path[i].kappa
              << ",s = " << poly_path[i].accumulated_s;
  }

  return;
}

}  // namespace planning
