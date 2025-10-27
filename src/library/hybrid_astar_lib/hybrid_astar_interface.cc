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
#include "hybrid_astar_response.h"
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
                               const float steer_ratio, const float wheel_base,
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
  vehicle_param_.min_turn_radius =
      min_turn_radius + config_.node_turn_radius_buffer;
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
  gear_switch_number_scenario_try_ = -1;
  time_benchmark_.Clear();
  search_traj_info_.Clear();

  ILOG_INFO << "astar interface success";

  return 0;
}

void HybridAStarInterface::UpdateInput(const ParkObstacleList& obs_list,
                                       const AstarRequest& request) {
  request_ = request;

  // start state
  ego_state_ = request.start_pose;

  // end state
  goal_state_ = request.goal;

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

void HybridAStarInterface::UpdateOutput() {
  double response_start_time = IflyTime::Now_ms();

  if (search_state_ == AstarSearchState::SEARCHING) {
    ILOG_INFO << "path searching, please wait";
    return;
  }

  PathClear();
  search_state_ = AstarSearchState::SEARCHING;
  DebugMapBoundString(request_.recommend_route_bound);

  UpdateGridMapBound();

  UpdateEDT();
  // update clear zone. This zone not contain any obstacle.
  clear_zone_.GenerateBoundingBox(ego_state_, &obs_, config_.enable_clear_zone);

  dp_heuristic_generator_->GenerateDpMap(
      request_.real_goal.x, request_.real_goal.y, map_bounds_, &obs_);

  hybrid_astar_->Clear();

  // vertical parking center ref line
  GenerateRefLine();

  // update future path decider
  FuturePathDecider future_path_decider;
  future_path_decider.Process(&ref_line_, vehicle_param_.min_turn_radius,
                              config_.node_step, &edt_, request_);

  RSExpansionDecider::UpdateRSPathRequest(&request_);

  bool is_ego_overlap_with_slot = IsEgoOverlapWithSlot();

  TargetPoseRegulator target_pose_regulator;
  target_pose_regulator.Process(&edt_, &request_, ego_state_,
                                request_.real_goal, vehicle_param_,
                                request_.direction_request);
  float ego_obs_dist = target_pose_regulator.GetEgoObsDist();

  if (request_.path_generate_method == AstarPathGenerateType::TRY_SEARCHING ||
      gear_switch_number_scenario_try_ < 0) {
    request_.gear_switch_num = config_.max_gear_change_num;
  } else {
    request_.gear_switch_num = std::min(gear_switch_number_scenario_try_ + 8,
                                        config_.max_gear_change_num);
  }

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
  time_benchmark_.total_time_ms = response_end_time - response_start_time;
  ILOG_INFO << "hybrid astar finish, plan once time = "
            << time_benchmark_.total_time_ms;

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
  UpdateGridMapBound();
  DebugMapBoundString(map_bounds_);

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
  clear_zone_.GenerateBoundingBox(ego_state_, &obs_, config_.enable_clear_zone);
  // vertical parking center ref line
  GenerateRefLine();

  // update future path decider
  FuturePathDecider future_path_decider;
  future_path_decider.Process(&ref_line_, vehicle_param_.min_turn_radius,
                              config_.node_step, &edt_, request_);

  TargetPoseRegulator target_pose_regulator;
  target_pose_regulator.Process(&edt_, &request_, ego_state_,
                                request_.real_goal, vehicle_param_,
                                request_.direction_request);
  hybrid_astar_->SetRequest(request_);

  float lat_buffer = 0.1;
  float lon_buffer = 0.2;
  float lat_buffer_inside_slot = 0.2;
  if (request_.space_type == ParkSpaceType::PARALLEL) {
    lat_buffer = 0.1;
    lon_buffer = 0.2;
  } else {
    lat_buffer = 0.2;
    lon_buffer = 0.4;
  }

  DebugAstarRequestString(request_);

  bool is_ego_overlap_with_slot = IsEgoOverlapWithSlot();
  float ego_obs_dist = target_pose_regulator.GetEgoObsDist();
  const TerminalCandidatePoint target_pose =
      target_pose_regulator.GetCandidatePose();
  // judge target regulator goal if collide
  lat_buffer_inside_slot = GetLatBufferForInsideSlot(
      target_pose.dist_to_obs, ego_obs_dist, is_ego_overlap_with_slot);

  target_regulator_goal_ = target_pose.pose;
  ILOG_INFO << "dist to obs = " << target_pose.dist_to_obs
            << ", lat buffer inside = " << lat_buffer_inside_slot;

  // If target slot is not wide enough, return.
  if (target_pose.dist_to_obs < lat_buffer_inside_slot) {
    search_state_ = AstarSearchState::FAILURE;
    return;
  }

  hybrid_astar_->UpdateCarBoxBySafeBuffer(lat_buffer, lat_buffer_inside_slot,
                                          lon_buffer);
  hybrid_astar_->SetSearchTime(config_.max_search_time_ms);

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

  // DebugPathString(best_traj_);
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
  return AstarSearchState::SUCCESS;
}

const std::vector<DebugAstarSearchPoint>&
HybridAStarInterface::GetChildNodeForDebug() {
  return hybrid_astar_->GetChildNodeForDebug();
}

const std::vector<Vec2f>& HybridAStarInterface::GetPriorQueueNode() {
  return hybrid_astar_->GetQueuePathForDebug();
}

const std::vector<Vec2f>& HybridAStarInterface::GetDelNodeQueueNode() {
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
        tmp_path.emplace_back(
            Vec2f(path_segment->points[k].x, path_segment->points[k].y));
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

  if (best_traj_ == nullptr || best_traj_->x.empty()) {
    return false;
  }

  AStarPathPoint point;
  float kappa_bound = 0.15;
  bool kappa_change_too_much = false;
  float kappa_diff;

  size_t point_size = best_traj_->GetFirstGearPathPointSize();
  result.reserve(point_size);

  AstarPathGear first_point_gear = best_traj_->gear[0];
  for (size_t i = 0; i < point_size; i++) {
    point =
        AStarPathPoint(best_traj_->x[i], best_traj_->y[i], best_traj_->phi[i],
                       best_traj_->gear[i], best_traj_->accumulated_s[i],
                       best_traj_->type[i], best_traj_->kappa[i]);
    result.emplace_back(point);

    // check kappa
    if (i > 0) {
      kappa_diff = best_traj_->kappa[i] - best_traj_->kappa[i - 1];

      if (std::fabs(kappa_diff) > kappa_bound) {
        kappa_change_too_much = true;
      }
    }
  }

  if (result.size() < 1) {
    GetFallBackPath(result);
  }

  search_traj_info_.first_seg_path_length = result.back().accumulated_s;

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
  // ILOG_INFO << "reset path";
  time_benchmark_.Clear();
  search_traj_info_.Clear();

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

void HybridAStarInterface::UpdateGridMapBound() {
  // update grid map range, ego pose need to be in range map_bounds.
  if (request_.space_type == ParkSpaceType::VERTICAL ||
      request_.space_type == ParkSpaceType::SLANTING) {
    map_bounds_.x_min = config_.vertical_map_bound_x_lower;
    map_bounds_.x_max = 20.0f;
    map_bounds_.y_min = -20.0f;
    map_bounds_.y_max = 20.0f;

    // check ego position
    std::array<Position2f, 4> veh_box;
    GetVehPolygonBy4Edge(veh_box, vehicle_param_.rear_edge_to_rear_axle,
                         vehicle_param_.front_edge_to_rear_axle,
                         vehicle_param_.max_width / 2.0 + 0.2f);

    Transform2f tf;
    tf.SetBasePose(request_.start_pose);
    Position2f global;
    for (int i = 0; i < 4; i++) {
      tf.ULFLocalPointToGlobal(&global, veh_box[i]);

      map_bounds_.x_min = std::min(map_bounds_.x_min, global.x);
      map_bounds_.x_min =
          std::max(map_bounds_.x_min, config_.vertical_map_bound_min_x_lower);
    }
  } else {
    map_bounds_.x_min = -10.0f;
    map_bounds_.x_max = 14.0f;
    map_bounds_.y_min = -20.0f;
    map_bounds_.y_max = 15.0f;
  }

  return;
}

void HybridAStarInterface::UpdateEDTBasePose(Pose2f& ogm_base_pose) {
  ogm_base_pose.x = map_bounds_.x_min;
  ogm_base_pose.y = map_bounds_.y_min;
  ogm_base_pose.theta = 0.0;

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

FootPrintCircleModel* HybridAStarInterface::GetCircleFootPrint(
    const HierarchySafeBuffer buffer) {
  if (hybrid_astar_ == nullptr) {
    return nullptr;
  }

  return hybrid_astar_->GetCircleFootPrint(buffer);
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
    TargetPoseRegulator& regulator, const float ego_obs_dist,
    const bool is_ego_overlap_with_slot) {
  float lat_buffer_outside;
  float lat_buffer_inside;
  float advised_lat_buffer_inside;
  float lon_buffer;

  // judge target regulator goal if collide
  const TerminalCandidatePoint target_pose =
      regulator.GetCandidatePose(GenLatBufferForCandidatePose(), 0.08f);
  advised_lat_buffer_inside = GetLatBufferForInsideSlot(
      target_pose.dist_to_obs, ego_obs_dist, is_ego_overlap_with_slot);

  target_regulator_goal_ = target_pose.pose;
  ILOG_INFO << "dist to obs = " << target_pose.dist_to_obs
            << ", lat buffer inside = " << advised_lat_buffer_inside;

  // If target slot is not wide enough, return.
  if (target_pose.dist_to_obs < advised_lat_buffer_inside) {
    search_state_ = AstarSearchState::FAILURE;
    return;
  }

  double search_time = 0.0;
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
    hybrid_astar_->SetSearchTime(config_.search_time_by_buffer[i]);

    // search single shot path.
    if (target_pose.dist_to_obs > config_.single_shot_path_width_thresh ||
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
        time_benchmark_.time_ms[i] = traj_candidates_[i].time_ms;
        time_benchmark_.node_pool_size[i] = hybrid_astar_->NodePoolSize();
        time_benchmark_.size++;
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

    // just for debug
    AstarPathGear gear = traj_candidates_[i].gear.empty()
                             ? AstarPathGear::NONE
                             : traj_candidates_[i].gear[0];
    search_traj_info_.first_action_gear[i] = static_cast<int32_t>(gear);
    search_traj_info_.first_action_gear_request[i] =
        static_cast<int32_t>(request_.first_action_request.gear_request);
    search_traj_info_.size++;

    // check time
    search_time += traj_candidates_[i].time_ms;
    time_benchmark_.time_ms[i] = traj_candidates_[i].time_ms;
    time_benchmark_.node_pool_size[i] = hybrid_astar_->NodePoolSize();
    time_benchmark_.size++;
    if (search_time > config_.max_search_time_ms) {
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
    TargetPoseRegulator& regulator) {
  float lat_buffer_outside;
  float advised_lat_buffer_inside;
  float lon_buffer;

  lat_buffer_outside = config_.safe_buffer.scenario_try_lat_buffer_outside;
  advised_lat_buffer_inside =
      config_.safe_buffer.scenario_try_lat_buffer_inside;
  lon_buffer = config_.safe_buffer.scenario_try_lon_buffer;
  hybrid_astar_->UpdateCarBoxBySafeBuffer(
      lat_buffer_outside, advised_lat_buffer_inside, lon_buffer);
  hybrid_astar_->SetSearchTime(config_.max_search_time_ms);

  // todo: 需要限制搜索时间
  ILOG_INFO << "scenario try planning";

  std::fill(feasible_directions_.begin(), feasible_directions_.end(), false);

  if (request_.direction_request_size > 1) {
    ParkingDirectionAttempt(advised_lat_buffer_inside);
  } else {
    const TerminalCandidatePoint target_pose =
        regulator.GetCandidatePose(0.15f);
    if (target_pose.dist_to_obs < advised_lat_buffer_inside) {
      ILOG_INFO << "goal_point dist to obs = " << target_pose.dist_to_obs;
    }

    target_regulator_goal_ = target_pose.pose;

    hybrid_astar_->AstarSearch(GetStartPoint(), GetGoalPoint(), map_bounds_,
                               &traj_candidates_[0]);
    if (request_.direction_request == ParkingVehDirection::HEAD_IN ||
        request_.direction_request == ParkingVehDirection::TAIL_IN) {
      ExtendPathToRealParkSpacePoint(&traj_candidates_[0], request_.real_goal);
    }
    best_traj_ = &traj_candidates_[0];
    gear_switch_number_scenario_try_ = best_traj_->gear_change_num;
    time_benchmark_.time_ms[0] = best_traj_->time_ms;
    time_benchmark_.node_pool_size[0] = hybrid_astar_->NodePoolSize();
    time_benchmark_.size = 1;
  }

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

  target_regulator_goal_ = request_.goal;

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
    hybrid_astar_->SetSearchTime(config_.search_time_by_buffer[i]);

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
  // 1. generate obs dist
  float inside_slot_min_obs_dist = 0.0f;
  // ego is inside slot
  if (is_ego_overlap_with_slot) {
    inside_slot_min_obs_dist = std::min(ego_obs_dist, target_obs_dist);
  } else {
    // ego is outside slot
    inside_slot_min_obs_dist = target_obs_dist;
  }

  // 2. For searching a solution easily, safe buffer should smaller than
  // obstacle distance.
  float search_efficiency_buffer = 0.15f;
  inside_slot_min_obs_dist -= search_efficiency_buffer;
  inside_slot_min_obs_dist = std::max(0.0f, inside_slot_min_obs_dist);

  float safe_buffer = 0.0f;
  for (size_t i = 0; i < config_.safe_buffer.lat_safe_buffer_inside.size();
       i++) {
    safe_buffer = config_.safe_buffer.lat_safe_buffer_inside[i];

    if (safe_buffer < inside_slot_min_obs_dist) {
      break;
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

    if (traj_candidates_[i].x.size() <= 1) {
      continue;
    }

    // current traj is better
    if ((traj_candidates_[i].gear_change_num <=
         best_traj_->gear_change_num - 2) &&
        IsPathGearSameWithRequest(traj_candidates_[i].gear[0],
                                  request_.first_action_request.gear_request)) {
      best_traj_ = &traj_candidates_[i];
    }
  }

  return;
}

void HybridAStarInterface::GenerateRefLine() {
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
}

void HybridAStarInterface::ParkingDirectionAttempt(
    const float& advised_lat_buffer_inside) {
  for (int8_t i = 0; i < request_.direction_request_size; i++) {
    ILOG_INFO << "***************** [ " << static_cast<int>(i)
              << " ] try, dir :  "
              << static_cast<int>(request_.direction_request_stack[i])
              << " ****************";
    TargetPoseRegulator target_pose_regulator;
    target_pose_regulator.Process(&edt_, &request_, ego_state_,
                                  request_.real_goal_stack[i], vehicle_param_,
                                  request_.direction_request_stack[i]);
    float ego_obs_dist = target_pose_regulator.GetEgoObsDist();

    const TerminalCandidatePoint target_regulator_result =
        target_pose_regulator.GetCandidatePose(GenLatBufferForCandidatePose(),
                                               0.08f);
    if (target_regulator_result.dist_to_obs < advised_lat_buffer_inside) {
      ILOG_INFO << "dist_goal_collide = "
                << target_regulator_result.dist_to_obs;
      ILOG_INFO << "target_regulator_goal_ will collide";
      continue;
    }
    if (request_.direction_request_stack[i] ==
            ParkingVehDirection::TAIL_OUT_TO_LEFT ||
        request_.direction_request_stack[i] ==
            ParkingVehDirection::TAIL_OUT_TO_RIGHT ||
        request_.direction_request_stack[i] ==
            ParkingVehDirection::HEAD_OUT_TO_LEFT ||
        request_.direction_request_stack[i] ==
            ParkingVehDirection::HEAD_OUT_TO_RIGHT) {
      if (fabs(target_regulator_result.pose.y) < 1.0 &&
          request_.space_type == ParkSpaceType::VERTICAL) {
        ILOG_INFO << "target_regulator_goal_ = "
                  << target_regulator_result.pose.x << ", "
                  << target_regulator_result.pose.y;
        ILOG_INFO << "insufficient space on both sides";
        continue;
      }
    }

    target_regulator_goal_ = target_regulator_result.pose;

    hybrid_astar_->AstarSearch(request_.start_pose, GetGoalPoint(), map_bounds_,
                               &traj_candidates_[0]);
    if (traj_candidates_[0].x.size() > 5 && i < feasible_directions_.size()) {
      feasible_directions_[i] = true;
    }

    // if (request_.direction_request == request_.direction_request_stack[i])
    // {
    //   best_traj_ = &traj_candidates_[0];
    //   gear_switch_number_scenario_try_ = best_traj_->gear_change_num;
    // }
  }
}

const float HybridAStarInterface::GenLatBufferForCandidatePose() {
  float lat_buffer = 0.0f;
  switch (request_.direction_request) {
    case ParkingVehDirection::TAIL_OUT_TO_LEFT:
    case ParkingVehDirection::TAIL_OUT_TO_RIGHT:
    case ParkingVehDirection::HEAD_OUT_TO_LEFT:
    case ParkingVehDirection::HEAD_OUT_TO_RIGHT:
      lat_buffer = 0.50f;
      break;
    default:
      lat_buffer = 0.15f;
      break;
  }

  return lat_buffer;
}

}  // namespace planning