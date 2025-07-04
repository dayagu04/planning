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
#include "hybrid_astar_request.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "rs_path_interpolate.h"
#include "transform2d.h"
#include "utils_math.h"

namespace planning {
#define DEBUG_HYBRID_ASTAR_INTERFACE (0)
#define PUBLISH_ASTAR_NODE_MESSAGE (0)

HybridAStarInterface::HybridAStarInterface() {}

HybridAStarInterface::~HybridAStarInterface() {}

int HybridAStarInterface::Init(const float back_edge_to_rear_axis,
                               const float car_length, const float car_width,
                               const float steer_ratio,
                               const float wheel_base,
                               const float min_turn_radius,
                               const float mirror_width) {
  config_.InitConfig();

  // read vehicle params
  vehicle_param_.length = car_length;
  vehicle_param_.width = car_width;
  vehicle_param_.max_width = car_width + mirror_width * 2;
  vehicle_param_.front_overhanging =
      car_length - back_edge_to_rear_axis - wheel_base;
  vehicle_param_.steer_ratio = steer_ratio;
  vehicle_param_.wheel_base = wheel_base;
  vehicle_param_.min_turn_radius = min_turn_radius + config_.turn_radius_buffer;
  vehicle_param_.mirror_width = mirror_width;
  vehicle_param_.rear_edge_to_rear_axle = back_edge_to_rear_axis;
  vehicle_param_.front_edge_to_rear_axle = car_length - back_edge_to_rear_axis;
  float front_wheel =
      std::atan(vehicle_param_.wheel_base /
                std::max(0.001, vehicle_param_.min_turn_radius));

  vehicle_param_.max_steer_angle =
      std::fabs(front_wheel * vehicle_param_.steer_ratio);

  search_state_ = AstarSearchState::NONE;

  ogm_.Init();
  if (config_.safe_buffer.lat_safe_buffer_outside.size() > 0) {
    edt_.Init(
        static_cast<float>(config_.safe_buffer.lat_safe_buffer_outside[0]),
        static_cast<float>(config_.safe_buffer.lon_safe_buffer[0]),
        static_cast<float>(config_.safe_buffer.lat_safe_buffer_outside[0]));
  }

  dp_heuristic_generator_ = std::make_shared<GridSearch>(config_);
  hybrid_astar_ = std::make_shared<HybridAStar>(config_, vehicle_param_, &obs_,
                                                &edt_, &clear_zone_, &ref_line_,
                                                dp_heuristic_generator_);
  hybrid_astar_->Init();

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
  Pose2f ogm_base_pose;
  UpdateEDTBasePose(ogm_base_pose);

  ogm_.Clear();
  ogm_.Process(ogm_base_pose);
  ogm_.AddParkingObs(obs_);

  edt_.Excute(ogm_, ogm_base_pose);

  return 0;
}

void HybridAStarInterface::UpdateEDTByObs(const ParkObstacleList& obs_list) {
  Pose2f ogm_base_pose;
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

  PathClear();
  search_state_ = AstarSearchState::SEARCHING;

  UpdateSearchBoundary();

  UpdateEDT();
  // update clear zone. This zone not contain any obstacle.
  clear_zone_.GenerateBoundingBox(ego_state_, &obs_);

  dp_heuristic_generator_->GenerateDpMap(
      request_.real_goal.x, request_.real_goal.y, map_bounds_, &obs_);

  hybrid_astar_->Clear();

  // vertical parking center ref line
  switch (request_.direction_request) {
    case ParkingVehDirection::HEAD_IN:
      ref_line_.Process(request_.real_goal,
                        Pose2f(request_.real_goal.x - 10.0,
                               request_.real_goal.y, request_.real_goal.theta));
      break;

    case ParkingVehDirection::TAIL_IN:
      ref_line_.Process(request_.real_goal,
                        Pose2f(request_.real_goal.x + 10.0,
                               request_.real_goal.y, request_.real_goal.theta));
      break;

    case ParkingVehDirection::HEAD_OUT_TO_LEFT:
    case ParkingVehDirection::TAIL_OUT_TO_LEFT:
      ILOG_INFO << "OUT_TO_LEFT";
      ref_line_.Process(request_.real_goal, Pose2f(request_.real_goal.x,
                                                   request_.real_goal.y + 10.0,
                                                   request_.real_goal.theta));
      break;

    case ParkingVehDirection::HEAD_OUT_TO_RIGHT:
    case ParkingVehDirection::TAIL_OUT_TO_RIGHT:
      ILOG_INFO << "OUT_TO_RIGHT";
      ref_line_.Process(request_.real_goal, Pose2f(request_.real_goal.x,
                                                   request_.real_goal.y - 10.0,
                                                   request_.real_goal.theta));
      break;

    case ParkingVehDirection::HEAD_OUT_TO_MIDDLE:
    case ParkingVehDirection::TAIL_OUT_TO_MIDDLE:
    default:
      ILOG_INFO << "Unknown Direction : OUT_TO_MIDDLE";
      ref_line_.Process(request_.real_goal,
                        Pose2f(request_.real_goal.x + 5.0, request_.real_goal.y,
                               request_.real_goal.theta));
      break;
  }

  // update future path decider
  FuturePathDecider future_path_decider;
  future_path_decider.Process(&ref_line_, vehicle_param_.min_turn_radius,
                              config_.node_step, &edt_, request_);

  RSExpansionDecider::UpdateRSPathRequest(&request_);

  bool is_ego_overlap_with_slot = IsEgoOverlapWithSlot();

  TargetPoseRegulator target_pose_regulator;
  target_pose_regulator.Process(&edt_, &request_, ego_state_, goal_state_,
                                vehicle_param_);
  float ego_obs_dist = target_pose_regulator.GetEgoObsDist();

  DebugAstarRequestString(request_);
  hybrid_astar_->SetRequest(request_);

  if (IsSearchBasedPlanning(request_.path_generate_method)) {
    PathSearchForScenarioRunning(target_pose_regulator, ego_obs_dist,
                                 is_ego_overlap_with_slot);
  } else if (request_.path_generate_method ==
             AstarPathGenerateType::TRY_SEARCHING) {
    PathSearchForScenarioTry(target_pose_regulator);
  } else if (IsSamplingBasedPlanning(request_.path_generate_method)) {
    PathSamplingForScenarioRunning();
  }

  search_state_ = AstarSearchState::SUCCESS;
  double response_end_time = IflyTime::Now_ms();
  ILOG_INFO << "hybrid astar finish, plan once time = "
            << response_end_time - response_start_time;

  // DebugPathString(best_traj_);

  return;
}

void HybridAStarInterface::GeneratePath(const Eigen::Vector3d& start,
                                        const Eigen::Vector3d& end,
                                        const AstarRequest& request) {
  if (search_state_ == AstarSearchState::SEARCHING) {
    ILOG_INFO << "path searching, please wait";
    return;
  }
  request_ = request;
  DebugAstarRequestString(request_);

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

  PathClear();
  search_state_ = AstarSearchState::SEARCHING;

  UpdateEDT();
  clear_zone_.GenerateBoundingBox(ego_state_, &obs_);
  // vertical parking center ref line
  switch (request_.direction_request) {
    case ParkingVehDirection::HEAD_IN:
      ILOG_INFO << "HEAD_IN";
      ref_line_.Process(request_.real_goal,
                        Pose2f(request_.real_goal.x - 10.0,
                               request_.real_goal.y, request_.real_goal.theta));
      break;

    case ParkingVehDirection::TAIL_IN:
      ILOG_INFO << "TAIL_IN";
      ref_line_.Process(request_.real_goal,
                        Pose2f(request_.real_goal.x + 10.0,
                               request_.real_goal.y, request_.real_goal.theta));
      break;

    case ParkingVehDirection::HEAD_OUT_TO_LEFT:
    case ParkingVehDirection::TAIL_OUT_TO_LEFT:
      ILOG_INFO << "OUT_TO_LEFT";
      ref_line_.Process(request_.real_goal, Pose2f(request_.real_goal.x,
                                                   request_.real_goal.y + 10.0,
                                                   request_.real_goal.theta));
      break;

    case ParkingVehDirection::HEAD_OUT_TO_RIGHT:
    case ParkingVehDirection::TAIL_OUT_TO_RIGHT:
      ILOG_INFO << "OUT_TO_RIGHT";
      ref_line_.Process(request_.real_goal, Pose2f(request_.real_goal.x,
                                                   request_.real_goal.y - 10.0,
                                                   request_.real_goal.theta));
      break;

    case ParkingVehDirection::HEAD_OUT_TO_MIDDLE:
    case ParkingVehDirection::TAIL_OUT_TO_MIDDLE:
    default:
      ILOG_INFO << "Unknown Direction : OUT_TO_MIDDLE";
      ref_line_.Process(request_.real_goal,
                        Pose2f(request_.real_goal.x + 5.0, request_.real_goal.y,
                               request_.real_goal.theta));
      break;
  }

  // update future path decider
  FuturePathDecider future_path_decider;
  future_path_decider.Process(&ref_line_, vehicle_param_.min_turn_radius,
                              config_.node_step, &edt_, request_);

  TargetPoseRegulator target_pose_regulator;
  target_pose_regulator.Process(&edt_, &request_, ego_state_, goal_state_,
                                vehicle_param_);
  hybrid_astar_->SetRequest(request_);

  float lat_buffer = 0.1;
  float lon_buffer = 0.2;
  if (request_.space_type == ParkSpaceType::PARALLEL) {
    lat_buffer = 0.1;
    lon_buffer = 0.2;
    hybrid_astar_->UpdateCarBoxBySafeBuffer(lat_buffer, lat_buffer, lon_buffer);
  } else {
    lat_buffer = 0.2;
    lon_buffer = 0.4;
    hybrid_astar_->UpdateCarBoxBySafeBuffer(lat_buffer, lat_buffer, lon_buffer);
  }

  DebugAstarRequestString(request_);

  std::pair<Pose2f, float> target_regulator_result;
  target_regulator_result = target_pose_regulator.GetCandidatePose(lat_buffer);

  // todo : head out no need target_pose_regulator;
  if (request_.direction_request == ParkingVehDirection::TAIL_IN ||
      request_.direction_request == ParkingVehDirection::HEAD_IN) {
    if (target_regulator_result.second < lat_buffer) {
      ILOG_INFO << "dist_goal_collide = " << target_regulator_result.second
                << ", lat_buffer " << lat_buffer << ", best_candidate->pose "
                << target_regulator_result.first.GetX() << ", "
                << target_regulator_result.first.GetY();
      ILOG_INFO << "target_regulator_goal_ will collide";
      search_state_ = AstarSearchState::FAILURE;
      return;
    }
  }

  target_regulator_goal_ = target_regulator_result.first;

  dp_heuristic_generator_->GenerateDpMap(GetGoalPoint().x, GetGoalPoint().y,
                                         map_bounds_, &obs_);

  if (request.path_generate_method == AstarPathGenerateType::ASTAR_SEARCHING) {
    hybrid_astar_->AstarSearch(GetStartPoint(), GetGoalPoint(), map_bounds_,
                               &traj_candidates_[0]);
  } else if (request.path_generate_method ==
                 AstarPathGenerateType::GEAR_REVERSE_SEARCHING ||
             request.path_generate_method ==
                 AstarPathGenerateType::GEAR_DRIVE_SEARCHING) {
    hybrid_astar_->OneShotPathAttempt(map_bounds_, GetStartPoint(),
                                      GetGoalPoint(), &traj_candidates_[0]);
  } else {
    Pose2f start_pose;
    Pose2f end_pose;
    start_pose.x = start[0];
    start_pose.y = start[1];
    start_pose.theta = start[2];

    end_pose.x = end[0];
    end_pose.y = end[1];
    end_pose.theta = end[2];

    search_state_ = AstarSearchState::SEARCHING;

    float dist_to_slot_up_edge =
        request.slot_length - ego_state_.DistanceToOrigin();
    float lon_min_sampling_dist;
    if (request_.space_type == ParkSpaceType::VERTICAL ||
        request_.space_type == ParkSpaceType::SLANTING) {
      lon_min_sampling_dist = std::max(2.0f, dist_to_slot_up_edge);
    } else {
      lon_min_sampling_dist = 0.4;
    }

    hybrid_astar_->PlanByRSPathSampling(&traj_candidates_[0], start_pose,
                                        end_pose, lon_min_sampling_dist);
  }

  search_state_ = AstarSearchState::SUCCESS;

  best_traj_ = &traj_candidates_[0];
  ExtendPathToRealParkSpacePoint(best_traj_, request_.real_goal);

  ILOG_INFO << "hybrid astar finish, point size " << best_traj_->x.size();

  return;
}

const AstarSearchState HybridAStarInterface::GetFullLengthPath(
    HybridAStarResult* result) {
  if (best_traj_ == nullptr || best_traj_->x.size() < 1) {
    GetFallBackPath(result);

    return AstarSearchState::FAILURE;
  }

  *result = *best_traj_;

  AstarSearchState search_state = AstarSearchState::SUCCESS;

  return search_state;
}

const std::vector<DebugAstarSearchPoint>&
HybridAStarInterface::GetChildNodeForDebug() {
  return hybrid_astar_->GetChildNodeForDebug();
}

const std::vector<Vec2f>&
HybridAStarInterface::GetPriorQueueNode() {
  return hybrid_astar_->GetQueuePathForDebug();
}

const std::vector<Vec2f>&
HybridAStarInterface::GetDelNodeQueueNode() {
  return hybrid_astar_->GetDelQueuePathForDebug();
}

void HybridAStarInterface::GetRSPathHeuristic(
    std::vector<std::vector<Vec2f>>& path_list) {
  if (hybrid_astar_ == nullptr) {
    return;
  }

  const std::vector<RSPath>& paths = hybrid_astar_->GetRSPathHeuristic();

  for (size_t i = 0; i < paths.size(); i++) {
    std::vector<Vec2f> tmp_path;

    const RSPath& path = paths[i];

    // hybrid_astar_->DebugRSPath(&path);

    for (int j = 0; j < path.size; j++) {
      const RSPathSegment* path_segment = &path.paths[j];

      for (int k = 0; k < path_segment->size; k++) {
        tmp_path.emplace_back(Vec2f(
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

  if (best_traj_ == nullptr) {
    return false;
  }

  AStarPathPoint point;
  float kappa_bound = 0.15;
  bool kappa_change_too_much = false;

  if (best_traj_->x.size() > 0) {
    size_t x_size = best_traj_->x.size();
    size_t y_size = best_traj_->y.size();
    size_t phi_size = best_traj_->phi.size();
    size_t gear_size = best_traj_->gear.size();
    size_t accumulated_s_size = best_traj_->accumulated_s.size();
    float kappa_diff;

    AstarPathGear first_point_gear = best_traj_->gear[0];
    for (size_t i = 0; i < x_size; i++) {
      if (i >= gear_size || i >= x_size || i >= y_size || i >= phi_size ||
          i >= accumulated_s_size) {
        ILOG_ERROR << "point size " << i << ",x_size=" << x_size
                   << ",y_size=" << y_size << "phi_size=" << phi_size
                   << ",gear_size=" << gear_size << ",accumulated_s_size"
                   << accumulated_s_size;
        break;
      }

      if (best_traj_->gear[i] == first_point_gear) {
        point = AStarPathPoint(best_traj_->x[i], best_traj_->y[i],
                               best_traj_->phi[i], best_traj_->gear[i],
                               best_traj_->accumulated_s[i],
                               best_traj_->type[i], best_traj_->kappa[i]);

        result.emplace_back(point);

        // ILOG_INFO << "xy " << point.x << " " << point.y;
      } else {
        break;
      }

      // check kappa
      if (i > 0) {
        kappa_diff = best_traj_->kappa[i] - best_traj_->kappa[i - 1];

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
    const Pose2f& start) {
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

void HybridAStarInterface::GetRSPathInFullPath(
    std::vector<float>& x, std::vector<float>& y, std::vector<float>& phi,
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

void HybridAStarInterface::PathClear() {
  best_traj_ = nullptr;
  for (size_t i = 0; i < traj_candidates_.size(); i++) {
    traj_candidates_.at(i).Clear();
  }
  search_time_ = 0.0;

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

void HybridAStarInterface::GetRSPathForDebug(std::vector<float>& x,
                                             std::vector<float>& y,
                                             std::vector<float>& phi) {
  if (hybrid_astar_ != nullptr) {
    hybrid_astar_->GetRSPathForDebug(x, y, phi);
  }

  return;
}

const HybridAStarResult* HybridAStarInterface::GetConstFullLengthPath() const {
  return best_traj_;
}

const ParkReferenceLine& HybridAStarInterface::GetConstRefLine() const {
  return ref_line_;
}

void HybridAStarInterface::GetPolynomialPathForDebug(std::vector<float>& x,
                                                     std::vector<float>& y,
                                                     std::vector<float>& phi) {
  if (best_traj_ == nullptr) {
    return;
  }

  for (size_t i = 0; i < best_traj_->x.size(); i++) {
    if (best_traj_->type[i] == AstarPathType::QUNTIC_POLYNOMIAL) {
      x.push_back(best_traj_->x[i]);
      y.push_back(best_traj_->y[i]);
      phi.push_back(best_traj_->phi[i]);
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

void HybridAStarInterface::UpdateEDTBasePose(Pose2f& ogm_base_pose) {
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

const Pose2f& HybridAStarInterface::GetStartPoint() {
  if (request_.swap_start_goal) {
    return target_regulator_goal_;
  }

  return ego_state_;
}

const Pose2f& HybridAStarInterface::GetGoalPoint() {
  if (request_.swap_start_goal) {
    return ego_state_;
  }

  return target_regulator_goal_;
}

FootPrintCircleModel* HybridAStarInterface::GetSlotOutsideCircleFootPrint() {
  if (hybrid_astar_ == nullptr) {
    return nullptr;
  }

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
    const TargetPoseRegulator& regulator, const float ego_obs_dist,
    const bool is_ego_overlap_with_slot) {
  float lat_buffer_outside;
  float lat_buffer_inside;
  float advised_lat_buffer_inside;
  float lon_buffer;

  // judge target regulator goal if collide
  std::pair<Pose2f, float> target_regulator_result;
  target_regulator_result =
      regulator.GetCandidatePose(config_.safe_buffer.lat_safe_buffer_inside[0]);
  advised_lat_buffer_inside = GetLatBufferForInsideSlot(
      target_regulator_result.second, ego_obs_dist, is_ego_overlap_with_slot);

  target_regulator_goal_ = target_regulator_result.first;

  // If target slot is not wide enough, return.
  if (target_regulator_result.second < advised_lat_buffer_inside &&
      !IsParkingOutRequest(request_.direction_request)) {
    ILOG_INFO << "goal dist = " << target_regulator_result.second
              << ", lat buffer inside = " << advised_lat_buffer_inside;
    search_state_ = AstarSearchState::FAILURE;
    return;
  }

  for (size_t i = 0; i < config_.safe_buffer.lat_safe_buffer_outside.size();
       i++) {
    if (IsParkingOutRequest(request_.direction_request)) {
      lat_buffer_outside =
          config_.safe_buffer.head_out_lat_safe_buffer_outside[i];
      lat_buffer_inside = config_.safe_buffer.lat_safe_buffer_inside[i];
    } else {
      lat_buffer_outside = config_.safe_buffer.lat_safe_buffer_outside[i];
      lat_buffer_inside = advised_lat_buffer_inside;
    }
    lon_buffer = config_.safe_buffer.lon_safe_buffer[i];
    hybrid_astar_->UpdateCarBoxBySafeBuffer(lat_buffer_outside,
                                            lat_buffer_inside, lon_buffer);

    // search single shot path.
    if (advised_lat_buffer_inside > config_.single_shot_path_width_thresh ||
        request_.path_generate_method ==
            AstarPathGenerateType::GEAR_DRIVE_SEARCHING ||
        request_.path_generate_method ==
            AstarPathGenerateType::GEAR_REVERSE_SEARCHING) {
      hybrid_astar_->OneShotPathAttempt(map_bounds_, GetStartPoint(),
                                        GetGoalPoint(), &traj_candidates_[i]);

      // check path
      if (traj_candidates_[i].x.size() > 2) {
        ILOG_INFO << "path is single shot";

        ExtendPathToRealParkSpacePoint(&traj_candidates_[i],
                                       request_.real_goal);
        break;
      }
    }

    if (request_.path_generate_method ==
        AstarPathGenerateType::ASTAR_SEARCHING) {
      // todo: init pointer in init function, do not transport every pointer
      // address into internal.

      hybrid_astar_->AstarSearch(GetStartPoint(), GetGoalPoint(), map_bounds_,
                                 &traj_candidates_[i]);

      if (!IsParkingOutRequest(request_.direction_request)) {
        ExtendPathToRealParkSpacePoint(&traj_candidates_[i],
                                       request_.real_goal);
      }
    }

    // check time
    search_time_ += traj_candidates_[i].time_ms;
    if (search_time_ > config_.max_search_time_ms) {
      ILOG_INFO << "time out";
      break;
    }

    // check path
    if (request_.plan_reason == PlanningReason::FIRST_PLAN &&
        traj_candidates_[i].gear_change_num <=
            gear_switch_number_scenario_try_ &&
        traj_candidates_[i].x.size() > 2) {
      ILOG_INFO << "path gear is nice";
      break;
    }
  }

  PathCandidateCompare();

  return;
}

void HybridAStarInterface::PathSearchForScenarioTry(
    const TargetPoseRegulator& regulator) {
  float lat_buffer_outside;
  float advised_lat_buffer_inside;
  float lon_buffer;

  lat_buffer_outside = config_.safe_buffer.scenario_try_lat_buffer_outside;
  advised_lat_buffer_inside =
      config_.safe_buffer.scenario_try_lat_buffer_inside;
  lon_buffer = config_.safe_buffer.scenario_try_lon_buffer;
  hybrid_astar_->UpdateCarBoxBySafeBuffer(
      lat_buffer_outside, advised_lat_buffer_inside, lon_buffer);

  // todo: 需要限制搜索时间
  ILOG_INFO << "scenario try planning";
  std::pair<Pose2f, float> target_regulator_result;
  target_regulator_result =
      regulator.GetCandidatePose(advised_lat_buffer_inside);
  if (target_regulator_result.second < advised_lat_buffer_inside) {
    ILOG_INFO << "dist_goal_collide = " << target_regulator_result.second;
    ILOG_INFO << "target_regulator_goal_ will collide";
  }

  target_regulator_goal_ = target_regulator_result.first;
  hybrid_astar_->AstarSearch(GetStartPoint(), GetGoalPoint(), map_bounds_,
                             &traj_candidates_[0]);

  best_traj_ = &traj_candidates_[0];
  gear_switch_number_scenario_try_ = best_traj_->gear_change_num;

  return;
}

void HybridAStarInterface::PathSamplingForScenarioRunning() {
  float dist_to_slot_up_edge =
      request_.slot_length - ego_state_.DistanceToOrigin();
  float lon_min_sampling_length;
  if (request_.space_type == ParkSpaceType::VERTICAL ||
      request_.space_type == ParkSpaceType::SLANTING) {
    lon_min_sampling_length =
        std::max(config_.adjust_dist_inside_slot, dist_to_slot_up_edge);
  } else {
    lon_min_sampling_length = 0.4;
  }

  target_regulator_goal_ = request_.goal_;

  float lat_buffer_outside;
  float advised_lat_buffer_inside;
  float lon_buffer;

  best_traj_ = &traj_candidates_[0];
  HybridAStarResult path;
  path.Clear();

  for (size_t i = 0; i < config_.safe_buffer.lat_safe_buffer_outside.size();
       i++) {
    lat_buffer_outside = config_.safe_buffer.lat_safe_buffer_outside[i];
    lon_buffer = config_.safe_buffer.lon_safe_buffer[i];
    advised_lat_buffer_inside = config_.safe_buffer.lat_safe_buffer_inside[i];
    hybrid_astar_->UpdateCarBoxBySafeBuffer(
        lat_buffer_outside, advised_lat_buffer_inside, lon_buffer);

    if (IsSamplingBasedPlanning(request_.path_generate_method)) {
      // parallel
      if (request_.space_type == ParkSpaceType::PARALLEL) {
        hybrid_astar_->SamplingByCubicPolyForParallelSlot(
            &path, GetStartPoint(), GetGoalPoint(), lon_min_sampling_length);
      } else {
        if (hybrid_astar_->SamplingByCubicSpiralForVerticalSlot(
                &path, GetStartPoint(), GetGoalPoint(),
                lon_min_sampling_length)) {
        } else {
          hybrid_astar_->PlanByRSPathSampling(
              &path, GetStartPoint(), GetGoalPoint(), lon_min_sampling_length);
        }
      }

    } else {
      hybrid_astar_->PlanByRSPathSampling(
          &path, GetStartPoint(), GetGoalPoint(), lon_min_sampling_length);
    }

    // check time
    if (path.time_ms > config_.max_search_time_ms) {
      break;
    }

    // compare path
    if (best_traj_->accumulated_s.size() < path.accumulated_s.size()) {
      *best_traj_ = path;
    }

    // check path is long enough
    if (best_traj_->accumulated_s.size() > 0 &&
        best_traj_->accumulated_s.back() > lon_min_sampling_length - 0.1) {
      break;
    }
  }

  if (best_traj_->x.size() <= 1) {
    hybrid_astar_->CopyFallbackPath(best_traj_);
    ILOG_INFO << "copy fallback path";
  }

  ILOG_INFO << "hybrid astar finish, path point size = "
            << best_traj_->x.size();

  return;
}

const float HybridAStarInterface::GetLatBufferForInsideSlot(
    const float target_obs_dist, const float ego_obs_dist,
    const bool is_ego_overlap_with_slot) {
  float safe_buffer = 0.1;
  // For searching a solution easily, safe buffer should smaller than obstacle
  // distance.
  float soft_buffer = 0.15;

  for (size_t i = 0; i < config_.safe_buffer.lat_safe_buffer_inside.size();
       i++) {
    safe_buffer = config_.safe_buffer.lat_safe_buffer_inside[i];

    // ego is inside slot
    if (is_ego_overlap_with_slot) {
      if (safe_buffer < ego_obs_dist - soft_buffer &&
          safe_buffer < target_obs_dist - soft_buffer) {
        break;
      }
    } else {
      // ego is outside slot
      if (safe_buffer < target_obs_dist - soft_buffer) {
        break;
      }
    }
  }

  return safe_buffer;
}

void HybridAStarInterface::PathCandidateCompare() {
  best_traj_ = &traj_candidates_[0];
  for (size_t i = 1; i < traj_candidates_.size(); i++) {
    // best traj is invalid, update
    if (best_traj_->x.size() < 2) {
      best_traj_ = &traj_candidates_[i];
      continue;
    }

    // current traj is better
    if ((traj_candidates_[i].gear_change_num <=
         best_traj_->gear_change_num - 2) &&
        (traj_candidates_[i].x.size() > 1)) {
      best_traj_ = &traj_candidates_[i];
    }
  }

  return;
}

}  // namespace planning