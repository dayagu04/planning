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
#include "src/modules/apa_function/apa_param_config.h"
#include "node3d.h"
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
  config_.rear_overhanging = back_edge_to_rear_axis;
  vehicle_param_.length = car_length;
  vehicle_param_.width = car_width;
  config_.front_overhanging = car_length - back_edge_to_rear_axis - wheel_base;
  vehicle_param_.steer_ratio = steer_ratio;
  vehicle_param_.wheel_base = wheel_base;
  vehicle_param_.min_turn_radius = min_turn_radius;
  config_.width_mirror = mirror_width;
  double front_wheel =
      std::atan(vehicle_param_.wheel_base / std::max(0.001, min_turn_radius));

  vehicle_param_.max_steer_angle =
      std::fabs(front_wheel * vehicle_param_.steer_ratio);

  ILOG_INFO << "veh_param.max_steer_angle " << vehicle_param_.max_steer_angle
            << " l " << vehicle_param_.length << "w " << vehicle_param_.width
            << "base " << vehicle_param_.wheel_base;
  ILOG_INFO << "gear_switch_penalty " << config_.gear_switch_penalty;
  ILOG_INFO << "enable_dp_cost_for_vertical_park "
            << config_.enable_dp_cost_for_vertical_park;
  ILOG_INFO << "enable_euler_cost_for_vertical_park "
            << config_.enable_euler_cost_for_vertical_park;
  ILOG_INFO << "enable_rs_path_h_cost_for_vertical_park "
            << config_.enable_rs_path_h_cost_for_vertical_park;
  ILOG_INFO << "enable_ref_line_h_cost_for_vertical_park "
            << config_.enable_ref_line_h_cost_for_vertical_park;

  hybrid_astar_ = std::make_shared<HybridAStar>(config_, vehicle_param_);
  if (hybrid_astar_ != nullptr) {
    hybrid_astar_->Init();
  }

  search_state_ = AstarSearchState::NONE;

  ogm_.Init();
  if (config_.lat_hierarchy_safe_buffer.size() > 0) {
    edt_.Init(static_cast<float>(config_.lat_hierarchy_safe_buffer[0]),
              static_cast<float>(config_.lon_front_safe_buffer),
              static_cast<float>(config_.lat_hierarchy_safe_buffer[0]));
  }

  ILOG_INFO << "astar interface success";

  return 0;
}

int HybridAStarInterface::UpdateInput(const ParkObstacleList& obs_list,
                                      const AstarRequest& request) {
  request_ = request;

  // start state
  initial_state_ = request.start_;
  ego_pose_ = initial_state_;

  // end state
  goal_state_ = request.goal_;

  obs_ = obs_list;

  if (hybrid_astar_ == nullptr) {
    ILOG_ERROR << "hybrid_astar_ is nullptr";

    return 0;
  }

  return 0;
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

int HybridAStarInterface::UpdateEDTByObs(const ParkObstacleList& obs_list) {
  Pose2D ogm_base_pose;
  UpdateEDTBasePose(ogm_base_pose);

  ogm_.Clear();
  ogm_.Process(ogm_base_pose);
  ogm_.AddParkingObs(obs_list);

  edt_.Excute(ogm_, ogm_base_pose);

  return 0;
}

void HybridAStarInterface::AdjustGoalBySafeCheck(Pose2D* adjust_goal,
                                                 const Pose2D& request_goal) {
  Transform2d tf;
  double sin_theta = std::sin(request_goal.theta);
  double cos_theta = std::cos(request_goal.theta);

  *adjust_goal = request_goal;
  tf.SetBasePose(*adjust_goal, sin_theta, cos_theta);

  AstarPathGear gear = AstarPathGear::reverse;

  for (size_t i = 0; i < 20; i++) {
    if (edt_.IsCollisionForPoint(&tf, gear)) {
      adjust_goal->x += 0.1;
      tf.SetBasePose(*adjust_goal, sin_theta, cos_theta);

      ILOG_INFO << "adjust goal by safe";
    } else {
      break;
    }

    // check relation in start and goal
    if (adjust_goal->x > initial_state_.x - 0.2) {
      break;
    }
  }

  return;
}

int HybridAStarInterface::UpdateOutput() {
  double response_start_time = IflyTime::Now_ms();

  if (search_state_ == AstarSearchState::SEARCHING) {
    ILOG_INFO << "path searching, please wait";
    return 0;
  }

  search_state_ = AstarSearchState::SEARCHING;

  UpdateSearchBoundary();

  UpdateEDT();
  // update clear zone. This zone not contain any obstacle.
  clear_zone_.GenerateBoundingBox(request_.start_, &obs_);

  hybrid_astar_->Clear();

  DebugAstarRequestString(request_);

  // vertical parking center ref line
  ref_line_.Init(request_.real_goal,
                 Pose2D(request_.real_goal.x + 10.0, request_.real_goal.y,
                        request_.real_goal.theta));

  // update future path decider
  FuturePathDecider future_path_decider;
  bool no_gear_switch;
  edt_.UpdateSafeBuffer(0.2, 0.4, 0.2);
  future_path_decider.Process(&coarse_traj_, request_.plan_reason, ego_pose_,
                              &edt_, &ref_line_, vehicle_param_.min_turn_radius,
                              &request_.first_action_request);
  no_gear_switch = future_path_decider.IsNextPathNoGearSwitchByHistory();

  RSExpansionDecider::UpdateRSPathRequest(
      &request_.rs_request, no_gear_switch,
      future_path_decider.GetNextPathGearByHistory(), ego_pose_,
      request_.slot_width);

  double lat_buffer;
  double lon_buffer;
  if (request_.path_generate_method == AstarPathGenerateType::ASTAR_SEARCHING) {
    // 要给控制留一点直线余量来跟踪，所以将最后的目标点移动一定距离，再使用直线连接
    for (size_t i = 0; i < config_.lat_hierarchy_safe_buffer.size(); i++) {
      lat_buffer = config_.lat_hierarchy_safe_buffer[i];
      lon_buffer = config_.lon_hierarchy_safe_buffer[i];
      edt_.UpdateSafeBuffer(static_cast<float>(lat_buffer),
                            static_cast<float>(config_.lon_front_safe_buffer),
                            static_cast<float>(lat_buffer));

      hybrid_astar_->UpdateCarBoxBySafeBuffer(lat_buffer, lon_buffer);

      // search single shot path.
      if (lat_buffer > 0.2 - 1e-4) {
        hybrid_astar_->SingleShotPathAttempt(map_bounds_, obs_, request_,
                                             &clear_zone_, &coarse_traj_, &edt_,
                                             &ref_line_);

        // check path
        if (coarse_traj_.x.size() > 2) {
          ILOG_INFO << "path is single shot";

          ExtendPathToRealParkSpacePoint(&coarse_traj_, request_.real_goal);
          break;
        }
      }

      // todo: init pointer in init function, do not transport every pointer
      // address into internal.
      hybrid_astar_->AstarSearch(initial_state_, goal_state_, map_bounds_, obs_,
                                 request_, &clear_zone_, &coarse_traj_, &edt_,
                                 &ref_line_);

      ExtendPathToRealParkSpacePoint(&coarse_traj_, request_.real_goal);

      // check time
      if (coarse_traj_.time_ms > 1000.0) {
        break;
      }

      // check path
      if (coarse_traj_.x.size() > 1) {
        break;
      }
    }
  } else if (request_.path_generate_method ==
             AstarPathGenerateType::TRY_SEARCHING) {
    lat_buffer = 0.11;
    lon_buffer = 0.4;
    edt_.UpdateSafeBuffer(static_cast<float>(lat_buffer),
                          static_cast<float>(config_.lon_front_safe_buffer),
                          static_cast<float>(lat_buffer));

    hybrid_astar_->UpdateCarBoxBySafeBuffer(lat_buffer, lon_buffer);

    // todo: 需要限制搜索时间
    ILOG_INFO << "scenario try planning";
    hybrid_astar_->AstarSearch(initial_state_, goal_state_, map_bounds_, obs_,
                               request_, &clear_zone_, &coarse_traj_, &edt_,
                               &ref_line_);

  } else {
    double dist_to_slot_up_edge =
        request_.slot_length - initial_state_.DistanceToOrigin();
    double lon_min_sampling_length;
    if (request_.space_type == ParkSpaceType::VERTICAL ||
        request_.space_type == ParkSpaceType::SLANTING) {
      lon_min_sampling_length = std::max(2.0, dist_to_slot_up_edge);
    } else {
      lon_min_sampling_length = 0.4;
    }

    for (size_t i = 0; i < config_.lat_hierarchy_safe_buffer.size(); i++) {
      lat_buffer = config_.lat_hierarchy_safe_buffer[i];
      lon_buffer = config_.lon_hierarchy_safe_buffer[i];
      edt_.UpdateSafeBuffer(static_cast<float>(lat_buffer),
                            static_cast<float>(lon_buffer),
                            static_cast<float>(lat_buffer));

      hybrid_astar_->UpdateCarBoxBySafeBuffer(lat_buffer, lon_buffer);

      if (request_.path_generate_method ==
          AstarPathGenerateType::CUBIC_POLYNOMIAL_SAMPLING) {
        // parallel
        if (request_.space_type == ParkSpaceType::PARALLEL) {
          hybrid_astar_->SamplingByCubicPolyForParallelSlot(
              &coarse_traj_, initial_state_, goal_state_,
              lon_min_sampling_length, map_bounds_, obs_, request_, &edt_,
              &clear_zone_, &ref_line_);
        } else {
          if (hybrid_astar_->SamplingByCubicPolyForVerticalSlot(
                  &coarse_traj_, initial_state_, goal_state_,
                  lon_min_sampling_length, map_bounds_, obs_, request_, &edt_,
                  &clear_zone_, &ref_line_)) {
          } else {
            hybrid_astar_->PlanByRSPathSampling(
                &coarse_traj_, initial_state_, goal_state_,
                lon_min_sampling_length, map_bounds_, obs_, request_, &edt_,
                &clear_zone_, &ref_line_);
          }
        }

      } else {
        hybrid_astar_->PlanByRSPathSampling(
            &coarse_traj_, initial_state_, goal_state_, lon_min_sampling_length,
            map_bounds_, obs_, request_, &edt_, &clear_zone_, &ref_line_);

      }

      ILOG_INFO << "hybrid astar finish, rs path point size = "
                << coarse_traj_.x.size();

      // check time
      if (coarse_traj_.time_ms > 1000.0) {
        break;
      }

      // check path
      if (coarse_traj_.x.size() > 1) {
        break;
      }
    }

    if (coarse_traj_.x.size() <= 1) {
      hybrid_astar_->CopyFallbackPath(&coarse_traj_);
      ILOG_INFO << "copy fallback path";
    }
  }

  search_state_ = AstarSearchState::SUCCESS;
  double response_end_time = IflyTime::Now_ms();
  ILOG_INFO << "hybrid astar finish, plan once time = "
            << response_end_time - response_start_time;

  // hybrid_astar_->DebugPathString(&coarse_traj_);

  return 0;
}

int HybridAStarInterface::GeneratePath(const Eigen::Vector3d& start,
                                       const Eigen::Vector3d& end,
                                       const ParkObstacleList& obs_list,
                                       const AstarRequest& request) {
  if (search_state_ == AstarSearchState::SEARCHING) {
    ILOG_INFO << "path searching, please wait";
    return 0;
  }
  request_ = request;

  DebugAstarRequestString(request);

  // range
  UpdateSearchBoundary();

  ILOG_INFO << "map bound, xmin " << map_bounds_.x_min << " , ymin "
            << map_bounds_.y_min << " ,xmax " << map_bounds_.x_max << " , ymax "
            << map_bounds_.y_max;

  // start state
  initial_state_.x = start[0];
  initial_state_.y = start[1];
  initial_state_.theta = start[2];

  ego_pose_ = initial_state_;

  // end state
  goal_state_.x = end[0];
  goal_state_.y = end[1];
  goal_state_.theta = end[2];

  if (hybrid_astar_ == nullptr) {
    ILOG_ERROR << "hybrid_astar_ is nullptr";

    return 0;
  }

  PathClear(&coarse_traj_);
  search_state_ = AstarSearchState::SEARCHING;

  UpdateEDTByObs(obs_list);
  clear_zone_.GenerateBoundingBox(request_.start_, &obs_list);
  // vertical parking center ref line
  ref_line_.Init(request_.real_goal,
                 Pose2D(request_.real_goal.x + 10.0, request_.real_goal.y,
                        request_.real_goal.theta));

  if (request.path_generate_method == AstarPathGenerateType::ASTAR_SEARCHING) {
    hybrid_astar_->AstarSearch(initial_state_, goal_state_, map_bounds_,
                               obs_list, request, &clear_zone_, &coarse_traj_,
                               &edt_, &ref_line_);
  } else if (request.path_generate_method ==
             AstarPathGenerateType::GEAR_REVERSE_DYNAMIC_PROGRAMMING) {
    hybrid_astar_->SingleShotPathAttempt(map_bounds_, obs_list, request,
                                         &clear_zone_, &coarse_traj_, &edt_,
                                         &ref_line_);

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

    double dist_to_slot_up_edge = request.slot_length - initial_state_.DistanceToOrigin();
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

  return 0;
}

const AstarSearchState HybridAStarInterface::GetFullLengthPath(
    HybridAStarResult* result) {
  if (coarse_traj_.x.size() < 1) {
    GetFallBackPath(result);

    return AstarSearchState::FAILURE;
  }

  // *result = std::move(coarse_traj_);
  *result = coarse_traj_;

  AstarSearchState search_state = AstarSearchState::SUCCESS;

  return search_state;
}

int HybridAStarInterface::ExtendPathToRealTargetPose(const Pose2D& real_end) {
  ExtendPathToRealParkSpacePoint(&coarse_traj_, real_end);

  return 0;
}

int HybridAStarInterface::ExtendPathToRealParkSpacePoint(
    HybridAStarResult* result, const Pose2D& real_end) {
  if (result->x.size() < 1) {
    ILOG_INFO << "no path";
    return 0;
  }

  Eigen::Vector2d astar_end_point;

  astar_end_point[0] = result->x.back();
  astar_end_point[1] = result->y.back();

  double extend_dist = real_end.DistanceTo(
      Pose2D(astar_end_point[0], astar_end_point[1], result->phi.back()));

  double phi = result->phi.back();
  AstarPathGear gear = result->gear.back();
  double astar_end_s = result->accumulated_s.back();
  AstarPathType path_type = AstarPathType::LINE_SEGMENT;

  const Eigen::Vector2d unit_line_vec = Eigen::Vector2d(1.0, 0.0);
  double s = 0.1;
  double ds = 0.1;

  Eigen::Vector2d point;
  while (s < extend_dist) {
    point = astar_end_point - s * unit_line_vec;
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
  double y_diff = real_end.y - result->y.back();
  double dist_diff = std::sqrt(x_diff * x_diff + y_diff * y_diff);
  if (dist_diff > 1e-2) {
    double last_s = result->accumulated_s.back();

    // add end
    result->x.emplace_back(real_end.x);
    result->y.emplace_back(real_end.y);
    result->phi.emplace_back(real_end.theta);
    result->gear.emplace_back(gear);
    result->type.emplace_back(path_type);
    result->accumulated_s.emplace_back(last_s + dist_diff);
    result->kappa.emplace_back(0);
  }

  return 0;
}

const std::vector<DebugAstarSearchPoint>&
HybridAStarInterface::GetChildNodeForDebug() {
  return hybrid_astar_->GetChildNodeForDebug();
}

const std::vector<ad_common::math::Vec2d>&
HybridAStarInterface::GetPriorQueueNode() {
  return hybrid_astar_->GetQueuePathForDebug();
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

  point = AStarPathPoint(initial_state_.x, initial_state_.y,
                         initial_state_.theta, AstarPathGear::parking, 0.0,
                         AstarPathType::START_NODE, 0.0);

  result.emplace_back(point);

  return 0;
}

const int HybridAStarInterface::GetFallBackPath(HybridAStarResult* result) {
  ILOG_INFO << "path fail";

  result->x.emplace_back(initial_state_.x);
  result->y.emplace_back(initial_state_.y);
  result->phi.emplace_back(initial_state_.theta);
  result->gear.emplace_back(AstarPathGear::parking);
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
        AStarPathPoint(start.x, start.y, start.theta, AstarPathGear::parking,
                       0.0, AstarPathType::START_NODE, 0.0);

    result.emplace_back(point);
  }

  ILOG_INFO << "cur path s " << result.back().accumulated_s << " size "
            << result.size();

  state = AstarSearchState::SUCCESS;

  return state;
}

const bool HybridAStarInterface::IsSelectedRealTargetPose() const {
  double y_diff = std::fabs(ego_pose_.y);

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

}  // namespace planning