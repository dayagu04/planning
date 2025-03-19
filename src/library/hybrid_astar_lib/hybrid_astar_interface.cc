#include "hybrid_astar_interface.h"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <utility>

#include "aabb2d.h"
#include "debug_info_log.h"
#include "future_path_decider.h"
#include "hybrid_a_star.h"
#include "hybrid_astar_common.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "rs_path_interpolate.h"
#include "transform2d.h"
#include "utils_math.h"

namespace planning {
#define DEBUG_HYBRID_ASTAR_INTERFACE (0)
#define PUBLISH_ASTAR_NODE_MESSAGE (1)

HybridAStarInterface::HybridAStarInterface() {}

HybridAStarInterface::~HybridAStarInterface() {}

int HybridAStarInterface::Init(const double back_edge_to_rear_axis,
                               const double car_length, const double car_width,
                               const double steer_ratio,
                               const double wheel_base,
                               const double min_turn_radius,
                               const double mirror_width) {
  config_.InitConfig();

  // read vehicle params
  vehicle_param_.length = car_length;
  vehicle_param_.width = car_width;
  vehicle_param_.max_width = car_width + mirror_width * 2;
  vehicle_param_.front_overhanging =
      car_length - back_edge_to_rear_axis - wheel_base;
  vehicle_param_.steer_ratio = steer_ratio;
  vehicle_param_.wheel_base = wheel_base;
  vehicle_param_.min_turn_radius = min_turn_radius;
  vehicle_param_.mirror_width = mirror_width;
  vehicle_param_.rear_edge_to_rear_axle = back_edge_to_rear_axis;
  vehicle_param_.front_edge_to_rear_axle = car_length - back_edge_to_rear_axis;
  double front_wheel =
      std::atan(vehicle_param_.wheel_base / std::max(0.001, min_turn_radius));

  vehicle_param_.max_steer_angle =
      std::fabs(front_wheel * vehicle_param_.steer_ratio);

  hybrid_astar_ = std::make_shared<HybridAStar>(config_, vehicle_param_);
  if (hybrid_astar_ != nullptr) {
    hybrid_astar_->Init();
  }

  search_state_ = AstarSearchState::NONE;

  ogm_.Init();
  if (config_.safe_buffer.lat_safe_buffer_outside.size() > 0) {
    edt_.Init(
        static_cast<float>(config_.safe_buffer.lat_safe_buffer_outside[0]),
        static_cast<float>(config_.safe_buffer.lon_safe_buffer[0]),
        static_cast<float>(config_.safe_buffer.lat_safe_buffer_outside[0]));
  }

  ILOG_INFO << "astar interface success";

  return 0;
}

void HybridAStarInterface::UpdateInput(const ParkObstacleList& obs_list,
                                      const AstarRequest& request) {
  request_ = request;

  // start state
  ego_state_ = request.start_;

  // end state
  goal_state_ = request.goal_;

  obs_ = obs_list;

  if (hybrid_astar_ == nullptr) {
    ILOG_ERROR << "hybrid_astar_ is nullptr";
    return;
  }

  return;
}

int HybridAStarInterface::UpdateEDT() {
  Pose2D ogm_base_pose;
  UpdateEDTBasePose(ogm_base_pose);

  ogm_.Clear();
  ogm_.Process(ogm_base_pose);
  ogm_.AddParkingObs(obs_);

  edt_.Excute(ogm_, ogm_base_pose);

  return 0;
}

void HybridAStarInterface::UpdateEDTByObs(const ParkObstacleList& obs_list) {
  Pose2D ogm_base_pose;
  UpdateEDTBasePose(ogm_base_pose);

  ogm_.Clear();
  ogm_.Process(ogm_base_pose);
  ogm_.AddParkingObs(obs_list);

  edt_.Excute(ogm_, ogm_base_pose);

  return;
}

void HybridAStarInterface::UpdateOutput() {
  double response_start_time = IflyTime::Now_ms();

  if (search_state_ == AstarSearchState::SEARCHING) {
    ILOG_INFO << "path searching, please wait";
    return;
  }

  PathClear(&coarse_traj_);
  search_state_ = AstarSearchState::SEARCHING;

  UpdateSearchBoundary();

  UpdateEDT();
  // update clear zone. This zone not contain any obstacle.
  clear_zone_.GenerateBoundingBox(request_.start_, &obs_);

  hybrid_astar_->Clear();
  hybrid_astar_->UpdateConfig(request_);

  // vertical parking center ref line
  if (request_.direction_request == ParkingVehDirection::HEAD_IN) {
    ref_line_.Process(request_.real_goal,
                      Pose2D(request_.real_goal.x - 10.0, request_.real_goal.y,
                             request_.real_goal.theta));
  } else {
    ref_line_.Process(request_.real_goal,
                      Pose2D(request_.real_goal.x + 10.0, request_.real_goal.y,
                             request_.real_goal.theta));
  }

  // update future path decider
  edt_.UpdateSafeBuffer(0.2, 0.4, 0.2);
  FuturePathDecider future_path_decider;
  future_path_decider.Process(
      &coarse_traj_, request_.plan_reason, request_.start_, &edt_, &ref_line_,
      vehicle_param_.min_turn_radius, request_.swap_start_goal,
      request_.path_generate_method, &request_.first_action_request);

  RSExpansionDecider::UpdateRSPathRequest(&request_);

  bool is_ego_overlap_with_slot = IsEgoOverlapWithSlot();

  TargetPoseRegulator target_pose_regulator;
  Pose2D center_line_pose;
  // 揉库时，使用车位内pose判断车位内目标点障碍物距离.
  if (IsEgoPoseAdjustPlanning(request_.path_generate_method)) {
    center_line_pose = request_.real_goal;
  } else {
    // 正常规划时，使用搜索目标点判断车位内目标点障碍物距离
    center_line_pose = goal_state_;
  }
  target_pose_regulator.Process(&edt_, &request_, request_.start_,
                                center_line_pose, vehicle_param_);
  double ego_obs_dist = target_pose_regulator.GetEgoObsDist();

  if (request_.path_generate_method == AstarPathGenerateType::ASTAR_SEARCHING) {
    PathSearchForScenarioRunning(target_pose_regulator, ego_obs_dist,
                                 is_ego_overlap_with_slot);
  } else if (request_.path_generate_method ==
             AstarPathGenerateType::TRY_SEARCHING) {
    PathSearchForScenarioTry(target_pose_regulator);
  } else if (request_.path_generate_method ==
             AstarPathGenerateType::CUBIC_POLYNOMIAL_SAMPLING) {
    PathSamplingForScenarioRunning();
  }

  search_state_ = AstarSearchState::SUCCESS;
  double response_end_time = IflyTime::Now_ms();
  ILOG_INFO << "hybrid astar finish, plan once time = "
            << response_end_time - response_start_time;

  // hybrid_astar_->DebugPathString(&coarse_traj_);

  return;
}

void HybridAStarInterface::GeneratePath(const Eigen::Vector3d& start,
                                        const Eigen::Vector3d& end,
                                        const ParkObstacleList& obs_list,
                                        const AstarRequest& request) {
  if (search_state_ == AstarSearchState::SEARCHING) {
    ILOG_INFO << "path searching, please wait";
    return;
  }
  request_ = request;
  hybrid_astar_->UpdateConfig(request);

  // range
  UpdateSearchBoundary();

  ILOG_INFO << "map bound, xmin " << map_bounds_.x_min << " , ymin "
            << map_bounds_.y_min << " ,xmax " << map_bounds_.x_max << " , ymax "
            << map_bounds_.y_max;

  // start state
  ego_state_.x = start[0];
  ego_state_.y = start[1];
  ego_state_.theta = start[2];

  // end state
  goal_state_.x = end[0];
  goal_state_.y = end[1];
  goal_state_.theta = end[2];

  if (hybrid_astar_ == nullptr) {
    ILOG_ERROR << "hybrid_astar_ is nullptr";

    return;
  }

  PathClear(&coarse_traj_);
  search_state_ = AstarSearchState::SEARCHING;

  UpdateEDTByObs(obs_list);
  clear_zone_.GenerateBoundingBox(request_.start_, &obs_list);
  // vertical parking center ref line
  if (request_.direction_request == ParkingVehDirection::HEAD_IN) {
    ref_line_.Process(request_.real_goal,
                      Pose2D(request_.real_goal.x - 10.0, request_.real_goal.y,
                             request_.real_goal.theta));
  } else {
    ref_line_.Process(request_.real_goal,
                      Pose2D(request_.real_goal.x + 10.0, request_.real_goal.y,
                             request_.real_goal.theta));
  }

  TargetPoseRegulator target_pose_regulator;
  Pose2D center_line_pose;
  if (IsEgoPoseAdjustPlanning(request_.path_generate_method)) {
    center_line_pose = request_.real_goal;
  } else {
    center_line_pose = goal_state_;
  }
  target_pose_regulator.Process(&edt_, &request_, request_.start_,
                                center_line_pose, vehicle_param_);

  double lat_buffer = 0.1;
  double lon_buffer = 0.2;
  if (request_.space_type == ParkSpaceType::PARALLEL) {
    lat_buffer = 0.1;
    lon_buffer = 0.2;
    hybrid_astar_->UpdateCarBoxBySafeBuffer(lat_buffer, lat_buffer, lon_buffer);
  } else {
    lat_buffer = 0.2;
    lon_buffer = 0.4;
    hybrid_astar_->UpdateCarBoxBySafeBuffer(lat_buffer, lat_buffer, lon_buffer);
  }

  std::pair<Pose2D, double> target_regulator_result;
  target_regulator_result = target_pose_regulator.GetCandidatePose(lat_buffer);

  if (target_regulator_result.second < lat_buffer) {
    ILOG_INFO << "dist_goal_collide = " << target_regulator_result.second;
    ILOG_INFO << "target_regulator_goal_ will collide";
    search_state_ = AstarSearchState::FAILURE;
    return;
  }
  target_regulator_goal_ = target_regulator_result.first;

  if (request.path_generate_method == AstarPathGenerateType::ASTAR_SEARCHING) {
    hybrid_astar_->AstarSearch(GetStartPoint(), GetGoalPoint(), map_bounds_,
                               obs_list, request, &clear_zone_, &coarse_traj_,
                               &edt_, &ref_line_);
  } else if (request.path_generate_method ==
             AstarPathGenerateType::GEAR_REVERSE_SEARCHING) {
    hybrid_astar_->GearRerversePathAttempt(
        map_bounds_, obs_list, request, &clear_zone_, GetStartPoint(),
        GetGoalPoint(), &coarse_traj_, &edt_, &ref_line_);

  } else if (request.path_generate_method ==
             AstarPathGenerateType::GEAR_DRIVE_SEARCHING) {
    hybrid_astar_->GearDrivePathAttempt(
        map_bounds_, obs_list, request, &clear_zone_, GetStartPoint(),
        GetGoalPoint(), &coarse_traj_, &edt_, &ref_line_);

  } else {
    Pose2D start_pose;
    Pose2D end_pose;
    start_pose.x = start[0];
    start_pose.y = start[1];
    start_pose.theta = start[2];

    end_pose.x = end[0];
    end_pose.y = end[1];
    end_pose.theta = end[2];

    search_state_ = AstarSearchState::SEARCHING;

    double dist_to_slot_up_edge =
        request.slot_length - ego_state_.DistanceToOrigin();
    double lon_min_sampling_dist;
    if (request_.space_type == ParkSpaceType::VERTICAL ||
        request_.space_type == ParkSpaceType::SLANTING) {
      lon_min_sampling_dist = std::max(2.0, dist_to_slot_up_edge);
    } else {
      lon_min_sampling_dist = 0.4;
    }

    hybrid_astar_->PlanByRSPathSampling(
        &coarse_traj_, start_pose, end_pose, lon_min_sampling_dist, map_bounds_,
        obs_list, request, &edt_, &clear_zone_, &ref_line_);
  }

  search_state_ = AstarSearchState::SUCCESS;
  ILOG_INFO << "hybrid astar finish, point size " << coarse_traj_.x.size();

  return;
}

const AstarSearchState HybridAStarInterface::GetFullLengthPath(
    HybridAStarResult* result) {
  if (coarse_traj_.x.size() < 1) {
    GetFallBackPath(result);

    return AstarSearchState::FAILURE;
  }

  *result = coarse_traj_;

  AstarSearchState search_state = AstarSearchState::SUCCESS;

  return search_state;
}

int HybridAStarInterface::ExtendPathToRealTargetPose(const Pose2D& real_end) {
  ExtendPathToRealParkSpacePoint(&coarse_traj_, real_end);

  return 0;
}

void HybridAStarInterface::ExtendPathToRealParkSpacePoint(
    HybridAStarResult* result, const Pose2D& real_end) {
  if (result->x.size() < 1) {
    ILOG_INFO << "no path";
    return;
  }

  Eigen::Vector2d astar_end_point;
  astar_end_point[0] = result->x.back();
  astar_end_point[1] = result->y.back();

  // check path end
  if (astar_end_point[0] <= real_end.x) {
    return;
  }

  double extend_dist = real_end.DistanceTo(
      Pose2D(astar_end_point[0], astar_end_point[1], result->phi.back()));
  if (extend_dist < 0.1) {
    return;
  }

  double phi = result->phi.back();
  AstarPathGear gear = result->gear.back();
  double astar_end_s = result->accumulated_s.back();
  AstarPathType path_type = AstarPathType::LINE_SEGMENT;

  Eigen::Vector2d unit_line_vec = Eigen::Vector2d(-1.0, 0.0);

  double s = 0.1;
  double ds = 0.1;

  Eigen::Vector2d point;
  while (s < extend_dist) {
    point = astar_end_point + s * unit_line_vec;
    result->x.emplace_back(point[0]);
    result->y.emplace_back(point[1]);
    result->phi.emplace_back(phi);
    result->gear.emplace_back(gear);
    result->type.emplace_back(path_type);
    result->accumulated_s.emplace_back(astar_end_s + s);
    result->kappa.emplace_back(0.0);

    s += ds;
  }

  double x_diff = real_end.x - result->x.back();
  double dist_diff = std::sqrt(x_diff * x_diff);
  if (dist_diff > 1e-2) {
    double last_s = result->accumulated_s.back();

    // add end
    result->x.emplace_back(real_end.x);
    result->y.emplace_back(astar_end_point[1]);
    result->phi.emplace_back(phi);
    result->gear.emplace_back(gear);
    result->type.emplace_back(path_type);
    result->accumulated_s.emplace_back(last_s + dist_diff);
    result->kappa.emplace_back(0);
  }

  return;
}

const std::vector<DebugAstarSearchPoint>&
HybridAStarInterface::GetChildNodeForDebug() {
  return hybrid_astar_->GetChildNodeForDebug();
}

const std::vector<ad_common::math::Vec2d>&
HybridAStarInterface::GetPriorQueueNode() {
  return hybrid_astar_->GetQueuePathForDebug();
}

const std::vector<ad_common::math::Vec2d>&
HybridAStarInterface::GetDelNodeQueueNode() {
  return hybrid_astar_->GetDelQueuePathForDebug();
}

void HybridAStarInterface::GetRSPathHeuristic(
    std::vector<std::vector<ad_common::math::Vec2d>>& path_list) {
  if (hybrid_astar_ == nullptr) {
    return;
  }

  const std::vector<RSPath>& paths = hybrid_astar_->GetRSPathHeuristic();

  for (size_t i = 0; i < paths.size(); i++) {
    std::vector<ad_common::math::Vec2d> tmp_path;

    const RSPath& path = paths[i];

    // hybrid_astar_->DebugRSPath(&path);

    for (int j = 0; j < path.size; j++) {
      const RSPathSegment* path_segment = &path.paths[j];

      for (int k = 0; k < path_segment->size; k++) {
        tmp_path.emplace_back(ad_common::math::Vec2d(
            path_segment->points[k].x, path_segment->points[k].y));
      }
    }

    path_list.emplace_back(tmp_path);
  }
  return;
}

const int HybridAStarInterface::GetFallBackPath(
    std::vector<AStarPathPoint>& result) {
  // init
  result.clear();
  ILOG_INFO << "path fail";

  AStarPathPoint point;
  point = AStarPathPoint(ego_state_.x, ego_state_.y, ego_state_.theta,
                         AstarPathGear::PARKING, 0.0, AstarPathType::START_NODE,
                         0.0);
  result.emplace_back(point);

  return 0;
}

const int HybridAStarInterface::GetFallBackPath(HybridAStarResult* result) {
  ILOG_INFO << "path fail";

  result->x.emplace_back(ego_state_.x);
  result->y.emplace_back(ego_state_.y);
  result->phi.emplace_back(ego_state_.theta);
  result->gear.emplace_back(AstarPathGear::PARKING);
  result->type.emplace_back(AstarPathType::START_NODE);
  result->accumulated_s.emplace_back(0.0);
  result->kappa.emplace_back(0.0);

  return 0;
}

const bool HybridAStarInterface::GetFirstSegmentPath(
    std::vector<AStarPathPoint>& result) {
  // init
  result.clear();

  AStarPathPoint point;
  double kappa_bound = 0.15;
  bool kappa_change_too_much = false;

  if (coarse_traj_.x.size() > 0) {
    size_t x_size = coarse_traj_.x.size();
    size_t y_size = coarse_traj_.y.size();
    size_t phi_size = coarse_traj_.phi.size();
    size_t gear_size = coarse_traj_.gear.size();
    size_t accumulated_s_size = coarse_traj_.accumulated_s.size();
    double kappa_diff;

    AstarPathGear first_point_gear = coarse_traj_.gear[0];
    for (size_t i = 0; i < x_size; i++) {
      if (i >= gear_size || i >= x_size || i >= y_size || i >= phi_size ||
          i >= accumulated_s_size) {
        ILOG_ERROR << "point size " << i << ",x_size=" << x_size
                   << ",y_size=" << y_size << "phi_size=" << phi_size
                   << ",gear_size=" << gear_size << ",accumulated_s_size"
                   << accumulated_s_size;
        break;
      }

      if (coarse_traj_.gear[i] == first_point_gear) {
        point = AStarPathPoint(coarse_traj_.x[i], coarse_traj_.y[i],
                               coarse_traj_.phi[i], coarse_traj_.gear[i],
                               coarse_traj_.accumulated_s[i],
                               coarse_traj_.type[i], coarse_traj_.kappa[i]);

        result.emplace_back(point);

        // ILOG_INFO << "xy " << point.x << " " << point.y;
      } else {
        break;
      }

      // check kappa
      if (i > 0) {
        kappa_diff = coarse_traj_.kappa[i] - coarse_traj_.kappa[i - 1];

        if (std::fabs(kappa_diff) > kappa_bound) {
          kappa_change_too_much = true;
        }
      }
    }
  }

  if (result.size() < 1) {
    GetFallBackPath(result);
  }

  ILOG_INFO << "cur path s " << result.back().accumulated_s << " size "
            << result.size();

  return kappa_change_too_much;
}

const AstarSearchState HybridAStarInterface::TransformFirstSegmentPath(
    std::vector<AStarPathPoint>& result, const HybridAStarResult& full_path,
    const Pose2D& start) {
  // init
  result.clear();
  AstarSearchState state;

  AStarPathPoint point;

  if (full_path.x.size() > 0) {
    size_t x_size = full_path.x.size();
    size_t y_size = full_path.y.size();
    size_t phi_size = full_path.phi.size();
    size_t gear_size = full_path.gear.size();
    size_t accumulated_s_size = full_path.accumulated_s.size();

    AstarPathGear first_point_gear = full_path.gear[0];
    for (size_t i = 0; i < x_size; i++) {
      if (i >= gear_size || i >= x_size || i >= y_size || i >= phi_size ||
          i >= accumulated_s_size) {
        ILOG_ERROR << "point size " << i;
        break;
      }

      if (full_path.gear[i] == first_point_gear) {
        point = AStarPathPoint(full_path.x[i], full_path.y[i], full_path.phi[i],
                               full_path.gear[i], full_path.accumulated_s[i],
                               full_path.type[i], full_path.kappa[i]);

        result.emplace_back(point);
      } else {
        break;
      }
    }
  }

  if (result.size() < 1) {
    AStarPathPoint point;

    point =
        AStarPathPoint(start.x, start.y, start.theta, AstarPathGear::PARKING,
                       0.0, AstarPathType::START_NODE, 0.0);

    result.emplace_back(point);
  }

  ILOG_INFO << "cur path s " << result.back().accumulated_s << " size "
            << result.size();

  state = AstarSearchState::SUCCESS;

  return state;
}

const bool HybridAStarInterface::IsSelectedRealTargetPose() const {
  double y_diff = std::fabs(request_.start_.y);

  // car heading and car position is near ref_line_
  if (y_diff < request_.slot_width * 0.5) {
    return true;
  }

  return false;
}

void HybridAStarInterface::GetRSPathInFullPath(
    std::vector<double>& x, std::vector<double>& y, std::vector<double>& phi,
    const HybridAStarResult& result) {
  x.clear();
  y.clear();
  phi.clear();

  for (size_t i = 0; i < result.x.size(); i++) {
    if (result.type[i] == AstarPathType::REEDS_SHEPP) {
      x.emplace_back(result.x[i]);
      y.emplace_back(result.x[i]);
      phi.emplace_back(result.x[i]);
    }
  }

  return;
}

void HybridAStarInterface::PathClear(HybridAStarResult* path) {
  path->Clear();

  // ILOG_INFO << "reset path";

  return;
}

void HybridAStarInterface::GetNodeListMessage(
    planning::common::AstarNodeList* list) {
  if (list == nullptr) {
    ILOG_INFO << "nullptr";
    return;
  }

  list->Clear();

  if (PUBLISH_ASTAR_NODE_MESSAGE) {
    hybrid_astar_->GetNodeListMessage(list);
  }

  return;
}

void HybridAStarInterface::GetNodeListMessage(
    std::vector<std::vector<Eigen::Vector2d>>& list) {
  list.clear();

  if (PUBLISH_ASTAR_NODE_MESSAGE) {
    hybrid_astar_->GetNodeListMessage(list);
  }

  return;
}

ParkObstacleList& HybridAStarInterface::GetMutableObstacleList() {
  return obs_;
}

const ParkObstacleList& HybridAStarInterface::GetConstObstacles() const {
  return obs_;
}

void HybridAStarInterface::GetRSPathForDebug(std::vector<double>& x,
                                             std::vector<double>& y,
                                             std::vector<double>& phi) {
  if (hybrid_astar_ != nullptr) {
    hybrid_astar_->GetRSPathForDebug(x, y, phi);
  }

  return;
}

const HybridAStarResult& HybridAStarInterface::GetConstFullLengthPath() const {
  return coarse_traj_;
}

const ParkReferenceLine& HybridAStarInterface::GetConstRefLine() const {
  return ref_line_;
}

void HybridAStarInterface::GetPolynomialPathForDebug(std::vector<double>& x,
                                                     std::vector<double>& y,
                                                     std::vector<double>& phi) {
  for (size_t i = 0; i < coarse_traj_.x.size(); i++) {
    if (coarse_traj_.type[i] == AstarPathType::QUNTIC_POLYNOMIAL) {
      x.push_back(coarse_traj_.x[i]);
      y.push_back(coarse_traj_.y[i]);
      phi.push_back(coarse_traj_.phi[i]);
    }
  }

  return;
}

void HybridAStarInterface::UpdateSearchBoundary() {
  // range
  if (request_.space_type == ParkSpaceType::VERTICAL ||
      request_.space_type == ParkSpaceType::SLANTING) {
    map_bounds_.x_min = -2;
    map_bounds_.x_max = 20;
    map_bounds_.y_min = -20;
    map_bounds_.y_max = 20;
  } else {
    map_bounds_.x_min = -10;
    map_bounds_.x_max = 14;
    map_bounds_.y_min = -12;
    map_bounds_.y_max = 15;
  }

  return;
}

void HybridAStarInterface::UpdateEDTBasePose(Pose2D& ogm_base_pose) {
  // range
  if (request_.space_type == ParkSpaceType::VERTICAL ||
      request_.space_type == ParkSpaceType::SLANTING) {
    ogm_base_pose.x = -3.0;
    ogm_base_pose.y = -20.0;
    ogm_base_pose.theta = 0.0;
  } else {
    ogm_base_pose.x = -10.0;
    ogm_base_pose.y = -20.0;
    ogm_base_pose.theta = 0.0;
  }

  return;
}

const Pose2D& HybridAStarInterface::GetStartPoint() {
  if (request_.swap_start_goal) {
    return target_regulator_goal_;
  }

  return ego_state_;
}

const Pose2D& HybridAStarInterface::GetGoalPoint() {
  if (request_.swap_start_goal) {
    return ego_state_;
  }

  return target_regulator_goal_;
}

FootPrintCircleModel* HybridAStarInterface::GetSlotOutsideCircleFootPrint() {
  return hybrid_astar_->GetSlotOutsideCircleFootPrint();
}

const bool HybridAStarInterface::IsEgoOverlapWithSlot() {
  Polygon2D ego_local_polygon;
  Polygon2D ego_global_polygon;

  GenerateUpLeftFrameBox(
      &ego_local_polygon, -vehicle_param_.rear_edge_to_rear_axle,
      -vehicle_param_.max_width / 2.0,
      vehicle_param_.wheel_base + vehicle_param_.front_overhanging,
      vehicle_param_.max_width / 2.0);
  ULFLocalPolygonToGlobal(&ego_global_polygon, &ego_local_polygon, ego_state_);

  Polygon2D slot_polygon;
  GenerateUpLeftFrameBox(&slot_polygon, 0.0, -request_.slot_width / 2,
                         request_.slot_length, request_.slot_width / 2);

  GJK2DInterface gjk;
  bool is_collision;
  gjk.PolygonCollisionByCircleCheck(&is_collision, &slot_polygon,
                                    &ego_global_polygon, 0.1);

  return is_collision;
}

void HybridAStarInterface::PathSearchForScenarioRunning(
    const TargetPoseRegulator& regulator, const double ego_obs_dist,
    const bool is_ego_overlap_with_slot) {
  double lat_buffer_outside;
  double advised_lat_buffer_inside;
  double lon_buffer;

  // judge target regulator goal if collide
  std::pair<Pose2D, double> target_regulator_result;
  target_regulator_result =
      regulator.GetCandidatePose(config_.safe_buffer.lat_safe_buffer_inside[0]);
  advised_lat_buffer_inside = GetLatBufferForInsideSlot(
      target_regulator_result.second, ego_obs_dist, is_ego_overlap_with_slot);

  target_regulator_goal_ = target_regulator_result.first;

  // If target slot is not wide enough, return.
  if (target_regulator_result.second < advised_lat_buffer_inside) {
    ILOG_INFO << "goal dist to obs = " << target_regulator_result.second
              << ", lat buffer inside = " << advised_lat_buffer_inside;
    search_state_ = AstarSearchState::FAILURE;
    return;
  }

  for (size_t i = 0; i < config_.safe_buffer.lat_safe_buffer_outside.size();
       i++) {
    lat_buffer_outside = config_.safe_buffer.lat_safe_buffer_outside[i];
    lon_buffer = config_.safe_buffer.lon_safe_buffer[i];
    hybrid_astar_->UpdateCarBoxBySafeBuffer(
        lat_buffer_outside, advised_lat_buffer_inside, lon_buffer);

    // search single shot path.
    if (advised_lat_buffer_inside > config_.single_shot_path_width_thresh) {
      if (request_.direction_request == ParkingVehDirection::HEAD_IN) {
        hybrid_astar_->GearDrivePathAttempt(
            map_bounds_, obs_, request_, &clear_zone_, GetStartPoint(),
            GetGoalPoint(), &coarse_traj_, &edt_, &ref_line_);
      } else {
        hybrid_astar_->GearRerversePathAttempt(
            map_bounds_, obs_, request_, &clear_zone_, GetStartPoint(),
            GetGoalPoint(), &coarse_traj_, &edt_, &ref_line_);
      }

      // check path
      if (coarse_traj_.x.size() > 2) {
        ILOG_INFO << "path is single shot";

        ExtendPathToRealParkSpacePoint(&coarse_traj_, request_.real_goal);
        break;
      }
    }

    // todo: init pointer in init function, do not transport every pointer
    // address into internal.
    hybrid_astar_->AstarSearch(GetStartPoint(), GetGoalPoint(), map_bounds_,
                               obs_, request_, &clear_zone_, &coarse_traj_,
                               &edt_, &ref_line_);

    ExtendPathToRealParkSpacePoint(&coarse_traj_, request_.real_goal);

    // check time
    if (coarse_traj_.time_ms > config_.max_search_time_ms) {
      break;
    }

    // check path
    if (coarse_traj_.x.size() > 1) {
      break;
    }
  }

  return;
}

void HybridAStarInterface::PathSearchForScenarioTry(
    const TargetPoseRegulator& regulator) {
  double lat_buffer_outside;
  double advised_lat_buffer_inside;
  double lon_buffer;

  lat_buffer_outside = config_.safe_buffer.scenario_try_lat_buffer_outside;
  advised_lat_buffer_inside =
      config_.safe_buffer.scenario_try_lat_buffer_inside;
  lon_buffer = config_.safe_buffer.scenario_try_lon_buffer;
  hybrid_astar_->UpdateCarBoxBySafeBuffer(
      lat_buffer_outside, advised_lat_buffer_inside, lon_buffer);

  // todo: 需要限制搜索时间
  ILOG_INFO << "scenario try planning";
  std::pair<Pose2D, double> target_regulator_result;
  target_regulator_result =
      regulator.GetCandidatePose(advised_lat_buffer_inside);
  if (target_regulator_result.second < advised_lat_buffer_inside) {
    ILOG_INFO << "dist_goal_collide = " << target_regulator_result.second;
    ILOG_INFO << "target_regulator_goal_ will collide";
  }

  target_regulator_goal_ = target_regulator_result.first;
  hybrid_astar_->AstarSearch(GetStartPoint(), GetGoalPoint(), map_bounds_, obs_,
                             request_, &clear_zone_, &coarse_traj_, &edt_,
                             &ref_line_);

  return;
}

void HybridAStarInterface::PathSamplingForScenarioRunning() {
  double dist_to_slot_up_edge =
      request_.slot_length - ego_state_.DistanceToOrigin();
  double lon_min_sampling_length;
  if (request_.space_type == ParkSpaceType::VERTICAL ||
      request_.space_type == ParkSpaceType::SLANTING) {
    lon_min_sampling_length =
        std::max(config_.adjust_dist_inside_slot, dist_to_slot_up_edge);
  } else {
    lon_min_sampling_length = 0.4;
  }

  target_regulator_goal_ = request_.goal_;

  double lat_buffer_outside;
  double advised_lat_buffer_inside;
  double lon_buffer;

  coarse_traj_.Clear();
  HybridAStarResult path;
  path.Clear();

  for (size_t i = 0; i < config_.safe_buffer.lat_safe_buffer_outside.size();
       i++) {
    lat_buffer_outside = config_.safe_buffer.lat_safe_buffer_outside[i];
    lon_buffer = config_.safe_buffer.lon_safe_buffer[i];
    advised_lat_buffer_inside = config_.safe_buffer.lat_safe_buffer_inside[i];
    hybrid_astar_->UpdateCarBoxBySafeBuffer(
        lat_buffer_outside, advised_lat_buffer_inside, lon_buffer);

    if (request_.path_generate_method ==
        AstarPathGenerateType::CUBIC_POLYNOMIAL_SAMPLING) {
      // parallel
      if (request_.space_type == ParkSpaceType::PARALLEL) {
        hybrid_astar_->SamplingByCubicPolyForParallelSlot(
            &path, GetStartPoint(), GetGoalPoint(),
            lon_min_sampling_length, map_bounds_, obs_, request_, &edt_,
            &clear_zone_, &ref_line_);
      } else {
        if (hybrid_astar_->SamplingByCubicSpiralForVerticalSlot(
                &path, GetStartPoint(), GetGoalPoint(),
                lon_min_sampling_length, map_bounds_, obs_, request_, &edt_,
                &clear_zone_, &ref_line_)) {
        } else {
          hybrid_astar_->PlanByRSPathSampling(
              &path, GetStartPoint(), GetGoalPoint(),
              lon_min_sampling_length, map_bounds_, obs_, request_, &edt_,
              &clear_zone_, &ref_line_);
        }
      }

    } else {
      hybrid_astar_->PlanByRSPathSampling(
          &path, GetStartPoint(), GetGoalPoint(),
          lon_min_sampling_length, map_bounds_, obs_, request_, &edt_,
          &clear_zone_, &ref_line_);
    }

    // check time
    if (path.time_ms > config_.max_search_time_ms) {
      break;
    }

    // compare path
    if (coarse_traj_.accumulated_s.size() < path.accumulated_s.size()) {
      coarse_traj_ = path;
    }

    // check path is long enough
    if (coarse_traj_.accumulated_s.size() > 0 &&
        coarse_traj_.accumulated_s.back() > lon_min_sampling_length - 0.1) {
      break;
    }
  }

  if (coarse_traj_.x.size() <= 1) {
    hybrid_astar_->CopyFallbackPath(&coarse_traj_);
    ILOG_INFO << "copy fallback path";
  }

  ILOG_INFO << "hybrid astar finish, path point size = "
            << coarse_traj_.x.size();

  return;
}

const double HybridAStarInterface::GetLatBufferForInsideSlot(
    const double target_obs_dist, const double ego_obs_dist,
    const bool is_ego_overlap_with_slot) {
  double safe_buffer = 0.0;

  for (size_t i = 0; i < config_.safe_buffer.lat_safe_buffer_inside.size();
       i++) {
    safe_buffer = config_.safe_buffer.lat_safe_buffer_inside[i];

    // ego is inside slot
    if (is_ego_overlap_with_slot) {
      if (safe_buffer < ego_obs_dist && safe_buffer < target_obs_dist) {
        break;
      }
    } else {
      // ego is outside slot
      if (safe_buffer < target_obs_dist) {
        break;
      }
    }
  }

  return safe_buffer;
}

}  // namespace planning