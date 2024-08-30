#include "st_graph_input.h"

#include "environmental_model.h"
#include "planning_context.h"
#include "st_graph_utils.h"
#include "virtual_lane.h"
#include "virtual_lane_manager.h"

namespace {

constexpr double kPathSampleInterval = 1.0;
constexpr double kExtendTime = 0.3;
constexpr double kTimeBuffer = 0.5;
constexpr double kTimeResolution = 0.2;
constexpr int32_t kMinPathPointSize = 2;
constexpr double kPlanningHorizon = 5.0;

void GenerateParallelMap(
    std::shared_ptr<VirtualLaneManager> virtual_Lane_Manager,
    std::shared_ptr<VirtualLane> ptr_ego_lane,
    std::unordered_map<int32_t, bool>* const ptr_is_parallel_lane_map) {
  if (nullptr == ptr_ego_lane || ptr_ego_lane->lane_points().empty() ||
      nullptr == ptr_is_parallel_lane_map) {
    return;
  }
  std::shared_ptr<VirtualLane> ptr_base_lane = nullptr;
  std::shared_ptr<VirtualLane> ptr_neighbor_lane = nullptr;
  const auto& all_lanes = virtual_Lane_Manager->get_virtual_lanes();
  for (const auto& lane : all_lanes) {
    const auto& lane_id = lane->get_virtual_id();
    const auto& ptr_lane =
        virtual_Lane_Manager->get_lane_with_virtual_id(lane_id);
    if (lane_id == ptr_ego_lane->get_virtual_id() || nullptr == ptr_lane ||
        ptr_lane->lane_points().empty()) {
      continue;
    }

    // TODO: judge ego lane whether is too long compare to other lanes
    const bool ego_lane_is_long = false;
    ptr_base_lane = ego_lane_is_long ? ptr_ego_lane : ptr_lane;
    ptr_neighbor_lane = ego_lane_is_long ? ptr_lane : ptr_ego_lane;

    if (nullptr == ptr_base_lane || nullptr == ptr_neighbor_lane ||
        ptr_neighbor_lane->lane_points().empty()) {
      continue;
    }

    const auto& front_point =
        ptr_neighbor_lane->lane_points().front().local_point;
    const auto& back_point =
        ptr_neighbor_lane->lane_points().back().local_point;

    const auto& frenet_coord = ptr_base_lane->get_lane_frenet_coord();
    if (frenet_coord == nullptr) {
      continue;
    }
    double front_s = 0.0;
    double front_l = 0.0;
    if (!frenet_coord->XYToSL(front_point.x, front_point.y, &front_s,
                              &front_l)) {
      continue;
    }
    double back_s = 0.0;
    double back_l = 0.0;
    if (!frenet_coord->XYToSL(back_point.x, back_point.y, &back_s, &back_l)) {
      continue;
    }
    constexpr double kParallelLateralThresholdM = 1.5;
    constexpr double kMinLaneWidthM = 2.5;
    const bool is_parallel =
        (front_l * back_l > 0.0) &&
        (std::fabs(front_l - back_l) < kParallelLateralThresholdM) &&
        std::fabs(front_l) > kMinLaneWidthM &&
        std::fabs(back_l) > kMinLaneWidthM;
    // std::cout << "is parallel:" << is_parallel << ",lane_id:" << lane_id <<
    // ",front_l:" << front_l
    //           << ",back_l:" << back_l << '\n';
    (*ptr_is_parallel_lane_map)[lane_id] = is_parallel;
  }
}
}  // namespace

namespace planning {
namespace speed {

StGraphInput::StGraphInput(const EgoPlanningConfigBuilder* config_builder,
                           planning::framework::Session* session)
    : session_(session) {
  config_ = config_builder->cast<STGraphConfig>();
}

void StGraphInput::Update() {
  Reset();
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto& lateral_behavior_planner_output =
      session_->planning_context().lateral_behavior_planner_output();
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto lane_change_status = lateral_behavior_planner_output.lc_status;
  const auto lane_change_request = lateral_behavior_planner_output.lc_request;
  const auto& planned_kd_path =
      session_->planning_context().motion_planner_output().lateral_path_coord;
  GetAgentOfTargetLane(dynamic_world, lane_change_status, lane_change_request);
  const auto& init_point = ego_state_manager->planning_init_point();
  PlanningInitPointToTrajectoryPoint(init_point);
  MakeBuffer(lane_change_status, lane_change_request, config_);

  virtual_lane_manager_ =
      session_->environmental_model().get_virtual_lane_manager();
  ego_lane_ = virtual_lane_manager_->get_current_lane();
  vehicle_param_ = VehicleConfigurationContext::Instance()->get_vehicle_param();
  mutable_agent_manager_ = session_->environmental_model().get_agent_manager();
  const auto& agents = dynamic_world->agent_manager()->GetAllCurrentAgents();
  const auto& ego_center_line_coord = virtual_lane_manager_->get_current_lane()
                                          ->get_reference_path()
                                          ->get_frenet_coord();
  // decision_output_ = decision_output;

  const double path_extend_distance = planning_init_point_.vel() * kExtendTime;
  path_range_.first = 0.0;
  path_range_.second = planned_kd_path->Length() + path_extend_distance;
  time_range_.first = 0.0;
  time_range_.second = kPlanningHorizon + kTimeBuffer;

  GenerateParallelMap(virtual_lane_manager_, ego_lane_, &is_parallel_lane_map_);

  FilterAgentsByDecisionType(agents);

  ExtendProcessedPath(lane_change_status, lane_change_request,
                      ego_center_line_coord, planned_kd_path);

  // update for the forward extend path
  if (nullptr != processed_path_ && !processed_path_->path_points().empty()) {
    path_range_.second =
        processed_path_->path_points().back().s() + path_extend_distance;
  }

  MakePathBorderQuerier(planned_kd_path);

  const auto& max_acceleration_curve =
      GenerateMaxAccelerationCurve(planning_init_point_, ego_state_manager);
  max_acceleration_curve_ = &max_acceleration_curve;
  if (config_.enable_backward_extend_st_boundary) {
    enable_backward_extend_st_boundary_ = config_.enable_backward_extend_st_boundary;
    backward_extend_time_s_ = config_.backward_extend_time_s;
  }
  MakePlanningInitPointBox();
}

void StGraphInput::GetAgentOfTargetLane(
    const std::shared_ptr<planning_data::DynamicWorld>& dynamic_world,
    const std::string lane_change_status,
    const std::string lane_change_request) {
  const bool is_lane_keeping = lane_change_request == "none" ||
                               lane_change_status == "none" ||
                               lane_change_status == "left_lane_change_wait" ||
                               lane_change_status == "right_lane_change_wait";
  if (is_lane_keeping) {
    return;
  }

  front_agent_of_target_ = StGraphUtils::GetFrontAgentOfTargetLane(
      dynamic_world, lane_change_status, lane_change_request);
  rear_agent_of_target_ = StGraphUtils::GetRearAgentOfTargetLane(
      dynamic_world, lane_change_status, lane_change_request);
}

void StGraphInput::FilterAgentsByDecisionType(
    const std::vector<const agent::Agent*>& origin_agents) {
  if (origin_agents.empty()) {
    return;
  }
  filtered_agents_.clear();
  filtered_agents_.reserve(origin_agents.size());
  for (const auto* agent : origin_agents) {
    if (agent == nullptr) {
      continue;
    }
    if (agent->agent_decision().agent_decision_type() ==
        agent::AgentDecisionType::IGNORE) {
      continue;
    }
    if (!(agent->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    filtered_agents_.emplace_back(agent);
  }
}

void StGraphInput::ExtendProcessedPath(
    const std::string lane_change_status,
    const std::string lane_change_request,
    const std::shared_ptr<planning_math::KDPath>& lane_fusion_ego_center_lane,
    const std::shared_ptr<planning_math::KDPath>& planned_path) {
  std::vector<planning_math::PathPoint> path_points;
  path_points.reserve(planned_path->path_points().size());
  ForwardExtendPlannedPath(lane_change_status, lane_change_request,
                           lane_fusion_ego_center_lane, planned_path,
                           &path_points);
  const bool is_lane_keeping = lane_change_request == "none" ||
                               lane_change_status == "none" ||
                               lane_change_status == "left_lane_change_wait" ||
                               lane_change_status == "right_lane_change_wait";
  if (!is_lane_keeping) {
    BackwardExtendPoints(planned_path, &path_points);
  }
  if (path_points.size() < kMinPathPointSize) {
    return;
  }
  const bool need_reset_s = false;
  processed_path_ = std::make_shared<planning_math::KDPath>(
      std::move(path_points), need_reset_s);
}

void StGraphInput::ForwardExtendPlannedPath(
    const std::string lane_change_status,
    const std::string lane_change_request,
    const std::shared_ptr<planning_math::KDPath>& lane_fusion_ego_center_lane,
    const std::shared_ptr<planning_math::KDPath>& planned_path,
    std::vector<planning_math::PathPoint>* const ptr_path_points) {
  const bool is_in_lane_keeping =
      lane_change_request == "none" || lane_change_status == "none" ||
      lane_change_status == "left_lane_change_wait" ||
      lane_change_status == "right_lane_change_wait";

  // cal desire path length
  constexpr double kMinLength = 30.0;
  const double init_v = std::fmax(0.0, planning_init_point_.vel());
  const double init_acc = std::fmax(0.0, planning_init_point_.acc());
  const double ego_max_acc = 5.0;
  const double plan_time_length = 5.0;
  double desired_path_length =
      init_v * plan_time_length +
      0.5 * ego_max_acc * plan_time_length * plan_time_length;
  desired_path_length = std::fmax(kMinLength, desired_path_length);

  if (is_in_lane_keeping) {
    ForwardExtendPlannedPathWithEgoLane(lane_fusion_ego_center_lane,
                                        planned_path, desired_path_length,
                                        ptr_path_points);
  } else {
    ForwardLinearlyExtendPlannedPath(planned_path, desired_path_length,
                                     ptr_path_points);
  }
  return;
}

void StGraphInput::ForwardExtendPlannedPathWithEgoLane(
    const std::shared_ptr<planning_math::KDPath>& lane_fusion_ego_center_lane,
    const std::shared_ptr<planning_math::KDPath>& planned_path,
    const double desired_path_length,
    std::vector<planning_math::PathPoint>* const ptr_path_points) {
  if (nullptr == ptr_path_points || planned_path->path_points().empty()) {
    return;
  }

  if (lane_fusion_ego_center_lane->path_points().empty()) {
    *ptr_path_points = planned_path->path_points();
    return;
  }

  const auto& planned_path_last_point = planned_path->path_points().back();
  double project_s = 0.0;
  double project_l = 0.0;
  if (!lane_fusion_ego_center_lane->XYToSL(planned_path_last_point.x(),
                                           planned_path_last_point.y(),
                                           &project_s, &project_l)) {
    *ptr_path_points = planned_path->path_points();
    return;
  }
  const double remain_length = desired_path_length - planned_path->Length();
  const double lane_fusion_ego_center_lane_length =
      lane_fusion_ego_center_lane->Length();
  constexpr double kDistanceThr = 3.0;
  if (project_s > lane_fusion_ego_center_lane_length - kDistanceThr) {
    ForwardLinearlyExtendPlannedPath(planned_path, desired_path_length,
                                     ptr_path_points);
  } else {
    // fusion ego lane
    *ptr_path_points = planned_path->path_points();
    const std::vector<planning_math::PathPoint>& ego_lane_path_points =
        lane_fusion_ego_center_lane->path_points();

    auto it = lane_fusion_ego_center_lane->QueryLowerBound(ego_lane_path_points,
                                                           project_s + 1.0);
    if (it == ego_lane_path_points.end()) {
      --it;
    }
    double sum_s = 0.0;
    for (; it != ego_lane_path_points.end(); ++it) {
      const double dx = it->x() - ptr_path_points->back().x();
      const double dy = it->y() - ptr_path_points->back().y();
      const double ds = std::sqrt(dx * dx + dy * dy);
      sum_s += ds;
      planning_math::PathPoint tmp = *it;
      tmp.set_s(ptr_path_points->back().s() + ds);
      ptr_path_points->emplace_back(tmp);
      if (sum_s > remain_length) {
        break;
      }
    }
    if (sum_s < remain_length && ptr_path_points->size() > 1) {
      // linearly extend ego lanes
      const auto& end_point = ptr_path_points->back();
      const auto& start_point =
          ptr_path_points->at(ptr_path_points->size() - 2);
      planning_math::LineSegment2d back_seg(start_point, end_point);
      double extend_heading = back_seg.heading();

      const double linear_extend_length =
          remain_length - lane_fusion_ego_center_lane_length + project_s;

      auto extend_point =
          end_point + linear_extend_length *
                          planning_math::Vec2d::CreateUnitVec2d(extend_heading);

      planning_math::PathPoint last_point(extend_point.x(), extend_point.y(),
                                          desired_path_length, 0.0,
                                          extend_heading, 0.0, 0.0, 0.0);
      ptr_path_points->emplace_back(last_point);
    }
  }
  return;
}

void StGraphInput::ForwardLinearlyExtendPlannedPath(
    const std::shared_ptr<planning_math::KDPath>& planned_path,
    const double desired_path_length,
    std::vector<planning_math::PathPoint>* const ptr_path_points) {
  if (nullptr == ptr_path_points) {
    return;
  }
  constexpr double kDesireLengthBuffer = 1.0;
  // no need to extend: planning path is long enough
  if ((planned_path->Length() + kDesireLengthBuffer > desired_path_length)) {
    *ptr_path_points = planned_path->path_points();
    return;
  }

  const double remain_length =
      desired_path_length - planned_path->Length();  // must satisfy
  // 2. extend path with ego lane
  constexpr double kMinRemainLaneLength = 10.0;
  *ptr_path_points = planned_path->path_points();
  constexpr double kConsiderPathHeadingBuffer = 1.0;
  auto back_point = planned_path->GetPathPointByS(planned_path->Length() -
                                                  kConsiderPathHeadingBuffer);
  planning_math::LineSegment2d back_seg(back_point,
                                        planned_path->path_points().back());
  double extend_heading = back_seg.heading();
  auto extend_point =
      planned_path->path_points().back() +
      remain_length * planning_math::Vec2d::CreateUnitVec2d(extend_heading);
  planning_math::PathPoint end_point(extend_point.x(), extend_point.y(),
                                     desired_path_length, 0.0, extend_heading,
                                     0.0, 0.0, 0.0);
  ptr_path_points->emplace_back(end_point);
  return;
}

void StGraphInput::BackwardExtendPoints(
    const std::shared_ptr<planning_math::KDPath>& planned_path,
    std::vector<planning_math::PathPoint>* const ptr_path_points) {
  if (nullptr == ptr_path_points || nullptr == ego_lane_) {
    return;
  }

  std::vector<planning_math::PathPoint> linear_extend_path_points;
  std::vector<planning_math::PathPoint> ego_lane_path_points;
  // get project_s of ego_pose on ego_lane
  double ego_project_s = 0.0;
  double ego_project_l = 0.0;
  const auto& ego_frenet_coord = ego_lane_->get_lane_frenet_coord();
  if (!ego_frenet_coord->XYToSL(planning_init_point_.x(),
                                planning_init_point_.y(), &ego_project_s,
                                &ego_project_l)) {
    return;
  }
  const auto& planned_path_points = planned_path->path_points();
  // if project_s < thr, then extend planned_path
  const double delta_s = config_.backward_extend_sample_resolution;
  const double backward_extend_length =
      config_.backward_extend_length_for_lane_change;
  if (ego_project_s < delta_s) {
    if (!planned_path_points.empty()) {
      const auto& first_point = planned_path_points.front();
      const double theta = NormalizeAngle(first_point.theta() + M_PI);
      const planning_math::Vec2d first_point_vec(first_point.x(),
                                                 first_point.y());
      linear_extend_path_points.reserve(
          int32_t(backward_extend_length / delta_s));
      for (double s = delta_s; s < backward_extend_length; s += delta_s) {
        auto point =
            first_point_vec + planning_math::Vec2d::CreateUnitVec2d(theta) * s;
        linear_extend_path_points.emplace_back(point.x(), point.y(), 0.0, 0.0,
                                               0.0, 0.0, 0.0, 0.0, 0.0);
      }
    }
  } else {
    // save path_points of ego_lane from 0 to project_s
    ego_lane_path_points.reserve(int32_t(ego_project_s / delta_s));
    for (double s = ego_project_s - delta_s; s > 0; s -= delta_s) {
      const auto& point = ego_frenet_coord->GetPathPointByS(s);
      ego_lane_path_points.emplace_back(point);
    }

    // if ego_lane_path_points size > 2, then extend ego_lane
    if (!ego_lane_path_points.empty()) {
      const auto& last_point = ego_lane_path_points.back();
      const double theta = NormalizeAngle(last_point.theta() + M_PI);
      const double target_len = backward_extend_length - ego_project_s;
      for (double s = delta_s; s < target_len; s += delta_s) {
        auto point =
            last_point + planning_math::Vec2d::CreateUnitVec2d(theta) * s;
        linear_extend_path_points.emplace_back(point.x(), point.y(), 0.0, 0.0,
                                               0.0, 0.0, 0.0, 0.0, 0.0);
      }
    }
  }

  ego_lane_path_points.insert(ego_lane_path_points.end(),
                              linear_extend_path_points.begin(),
                              linear_extend_path_points.end());

  double sum_s = 0.0;
  for (size_t i = 0; i < ego_lane_path_points.size(); ++i) {
    if (i == 0) {
      double dx = ego_lane_path_points[i].x() - planning_init_point_.x();
      double dy = ego_lane_path_points[i].y() - planning_init_point_.y();
      sum_s = -std::sqrt(dx * dx + dy * dy);
    } else {
      double dx = ego_lane_path_points[i].x() - ego_lane_path_points[i - 1].x();
      double dy = ego_lane_path_points[i].y() - ego_lane_path_points[i - 1].y();
      sum_s -= std::sqrt(dx * dx + dy * dy);
    }
    ego_lane_path_points[i].set_s(sum_s);
  }

  std::reverse(ego_lane_path_points.begin(), ego_lane_path_points.end());
  std::vector<planning_math::PathPoint> final_path_points;
  final_path_points.reserve(ego_lane_path_points.size() +
                            ptr_path_points->size());
  final_path_points.insert(final_path_points.end(),
                           ego_lane_path_points.begin(),
                           ego_lane_path_points.end());
  final_path_points.insert(final_path_points.end(), ptr_path_points->begin(),
                           ptr_path_points->end());
  ptr_path_points->swap(final_path_points);
}

const double StGraphInput::start_absolute_time() const {
  return planning_init_point_.absolute_time();
}

const int32_t StGraphInput::reserve_num() const {
  return int32_t((time_range_.second - time_range_.first) / kTimeResolution) +
         1;
}

const std::vector<const agent::Agent*>& StGraphInput::filtered_agents() const {
  return filtered_agents_;
}

std::shared_ptr<agent::AgentManager> StGraphInput::mutable_agent_manager() {
  return mutable_agent_manager_;
}

const std::pair<double, double>& StGraphInput::path_range() const {
  return path_range_;
}

const std::pair<double, double>& StGraphInput::time_range() const {
  return time_range_;
}

const std::shared_ptr<planning_math::KDPath> StGraphInput::processed_path()
    const {
  return nullptr == processed_path_ ? nullptr : processed_path_;
}

const PathBorderQuerier* StGraphInput::path_border_querier() const {
  return nullptr == path_border_querier_ ? nullptr : path_border_querier_.get();
}

const trajectory::TrajectoryPoint& StGraphInput::time_aligned_ego_state()
    const {
  return time_aligned_ego_state_;
}

const trajectory::TrajectoryPoint& StGraphInput::planning_init_point() const {
  return planning_init_point_;
}

double StGraphInput::large_agent_expand_param_for_consistency() const {
  return large_agent_expand_param_for_consistency_;
}

double StGraphInput::large_agent_small_expand_param_for_consistency() const {
  return large_agent_small_expand_param_for_consistency_;
}

bool StGraphInput::IsParallelToEgoLane(const int32_t lane_id) const {
  const auto iter = is_parallel_lane_map_.find(lane_id);
  return iter == is_parallel_lane_map_.end() ? false : iter->second;
}

const std::shared_ptr<VirtualLaneManager>
StGraphInput::ptr_virtual_lane_manager() const {
  return virtual_lane_manager_;
}

const std::shared_ptr<VirtualLane> StGraphInput::ego_lane() const {
  return ego_lane_;
}

bool StGraphInput::is_lane_keeping() const { return is_lane_keeping_; }

const string StGraphInput::lane_change_request() const {
  return session_->planning_context()
      .lateral_behavior_planner_output()
      .lc_request;
}

const string StGraphInput::lane_change_status() const {
  return session_->planning_context()
      .lateral_behavior_planner_output()
      .lc_status;
};

bool StGraphInput::enable_backward_extend_st_boundary() const {
  return enable_backward_extend_st_boundary_;
}

const SecondOrderTimeOptimalTrajectory* StGraphInput::max_acceleration_curve()
    const {
  return max_acceleration_curve_;
}

double StGraphInput::backward_extend_time_s() const {
  return backward_extend_time_s_;
}

const agent::Agent* StGraphInput::front_agent_of_target() const {
  return front_agent_of_target_;
}

const agent::Agent* StGraphInput::rear_agent_of_target() const {
  return rear_agent_of_target_;
}

double StGraphInput::front_agent_lower_s_safety_buffer_for_lane_change() const {
  return config_.front_agent_lower_s_safety_buffer_for_lane_change;
}

const planning_math::Box2d& StGraphInput::planning_init_point_box() const {
  return planning_init_point_box_;
}

void StGraphInput::MakeBuffer(const std::string lane_change_status,
                              const std::string lane_change_request,
                              const STGraphConfig& config) {
  is_lane_keeping_ = lane_change_request == "none" ||
                     lane_change_status == "none" ||
                     lane_change_status == "left_lane_change_wait" ||
                     lane_change_status == "right_lane_change_wait";
  const double lane_keeping_lower_lateral_buffer_m =
      config.lane_keeping_lower_lateral_buffer_m;
  const double lane_keeping_upper_lateral_buffer_m =
      config.lane_keeping_upper_lateral_buffer_m;
  const double lane_keeping_lower_speed_kph =
      config.lane_keeping_lower_speed_kph;
  const double lane_keeping_upper_speed_kph =
      config.lane_keeping_upper_speed_kph;
  const double lane_keeping_large_agent_lateral_buffer_m =
      config.lane_keeping_large_agent_lateral_buffer_m;
  const double lane_change_lateral_buffer_m =
      config.lane_change_lateral_buffer_m;
  const double lane_keeping_large_agent_lower_lateral_buffer_m =
      config.lane_keeping_large_agent_lower_lateral_buffer_m;
  const double lane_keeping_large_agent_upper_lateral_buffer_m =
      config.lane_keeping_large_agent_upper_lateral_buffer_m;
  const double lane_keeping_large_agent_lower_speed_kph =
      config.lane_keeping_large_agent_lower_speed_kph;
  const double lane_keeping_large_agent_upper_speed_kph =
      config.lane_keeping_large_agent_upper_speed_kph;
  const double large_agent_expand_param_for_consistency =
      config.large_agent_expand_param_for_consistency;
  const double large_agent_small_expand_param_for_consistency =
      config.large_agent_small_expand_param_for_consistency;
  const double cone_lateral_buffer_m = config.cone_lateral_buffer_m;
  const double lane_keeping_large_heading_diff_lon_buffer_m =
      config.lane_keeping_large_heading_diff_lon_buffer_m;
  const double person_lat_buffer_m = config.person_lat_buffer_m;
  const double person_lon_buffer_m = config.person_lon_buffer_m;
  const double bycicle_lat_buffer_m = config.bycicle_lat_buffer_m;
  const double bycicle_lon_buffer_m = config.bycicle_lon_buffer_m;
  const double tricycle_lat_buffer_m = config.tricycle_lat_buffer_m;
  const double tricycle_lon_buffer_m = config.tricycle_lon_buffer_m;
  const double reverse_vehicle_lat_buffer_m =
      config.reverse_vehicle_lat_buffer_m;

  // 1. normal lane keeping and lane change
  lat_buffer_ =
      is_lane_keeping_
          ? StGraphUtils::CalculateLateralBufferForNormalLaneKeeping(
                planning_init_point_, lane_keeping_lower_lateral_buffer_m,
                lane_keeping_upper_lateral_buffer_m,
                lane_keeping_lower_speed_kph, lane_keeping_upper_speed_kph)
          : lane_change_lateral_buffer_m;

  // 2. large agent
  lat_buffer_for_large_agent_ =
      StGraphUtils::CalculateLateralBufferForNormalLaneKeeping(
          planning_init_point_, lane_keeping_large_agent_lower_lateral_buffer_m,
          lane_keeping_large_agent_upper_lateral_buffer_m,
          lane_keeping_large_agent_lower_speed_kph,
          lane_keeping_large_agent_upper_speed_kph);

  // 3. cone
  lat_buffer_for_cone_ = cone_lateral_buffer_m;

  // 4. large agent expand buffer for consistency
  large_agent_expand_param_for_consistency_ =
      large_agent_expand_param_for_consistency;
  large_agent_small_expand_param_for_consistency_ =
      large_agent_small_expand_param_for_consistency;

  // 5. lon buffer
  lon_buffer_for_large_heading_diff_ =
      lane_keeping_large_heading_diff_lon_buffer_m;

  // 5. vru buffer
  person_lat_buffer_ = person_lat_buffer_m;
  person_lon_buffer_ = person_lon_buffer_m;
  bycicle_lat_buffer_ = bycicle_lat_buffer_m;
  bycicle_lon_buffer_ = bycicle_lon_buffer_m;
  tricycle_lat_buffer_ = tricycle_lat_buffer_m;
  tricycle_lon_buffer_ = tricycle_lon_buffer_m;
  reverse_vehicle_lat_buffer_m_ = reverse_vehicle_lat_buffer_m;
}

const double StGraphInput::lat_buffer() const { return lat_buffer_; }

const double StGraphInput::GetSuitableLateralBuffer(
    const agent::Agent& agent) const {
  if (agent.is_vru()) {
    const auto agent_type = agent.type();
    if (agent_type == agent::AgentType::PEDESTRIAN) {
      return std::fmax(lat_buffer_, person_lat_buffer_);
    } else if (agent_type == agent::AgentType::BICYCLE) {
      return std::fmax(lat_buffer_, bycicle_lat_buffer_);
    } else if (agent_type == agent::AgentType::TRICYCLE) {
      return std::fmax(lat_buffer_, tricycle_lat_buffer_);
    }
  }

  if (agent.is_reverse() && agent.is_vehicle_type()) {
    return reverse_vehicle_lat_buffer_m_;
  }

  if (StGraphUtils::IsLargeAgent(agent)) {
    return lat_buffer_for_large_agent_;
  }

  if (agent::AgentType::TRAFFIC_CONE == agent.type()) {
    return lat_buffer_for_cone_;
  }

  // tips:return the value directly because the lane change case and lane
  // keeping case is calculate in 'MakeLateralBuffer'.
  return lat_buffer_;
}

const double StGraphInput::GetSuitableLonBuffer(
    const agent::Agent& agent) const {
  double buffer = 0.0;
  if (agent.is_vru()) {
    const auto agent_type = agent.type();
    if (agent_type == agent::AgentType::PEDESTRIAN) {
      buffer =
          std::fmax(lon_buffer_for_large_heading_diff_, person_lon_buffer_);
    } else if (agent_type == agent::AgentType::BICYCLE) {
      buffer =
          std::fmax(lon_buffer_for_large_heading_diff_, bycicle_lon_buffer_);
    } else if (agent_type == agent::AgentType::TRICYCLE) {
      buffer =
          std::fmax(lon_buffer_for_large_heading_diff_, tricycle_lon_buffer_);
    }
    return buffer;
  }
  if (!is_lane_keeping_) {
    return buffer;
  }
  constexpr double kLargeHeadingDiffThreshold = 45.0 / 57.3;
  const double heading_diff = std::fabs(planning_math::NormalizeAngle(
      agent.theta() - planning_init_point_.theta()));
  buffer = heading_diff > kLargeHeadingDiffThreshold
               ? lon_buffer_for_large_heading_diff_
               : 0.0;
  // std::cout << "[diff]agent id:" << agent.agent_id() << ",diff:" <<
  // heading_diff << ","
  //           << heading_diff / 3.14 * 180.0 << ",buf:" << buffer << '\n';
  return buffer;
}

void StGraphInput::MakePlanningInitPointBox() {
  const double vehicle_width = vehicle_param_.width;
  const double vehicle_length = vehicle_param_.length;
  const double front_edge_to_center = vehicle_param_.front_edge_to_rear_axle;
  const double back_edge_to_center = vehicle_param_.rear_edge_to_rear_axle;
  const double back_axle_to_center =
      0.5 * (front_edge_to_center - back_edge_to_center);
  Vec2d target_center;
  target_center.set_x(planning_init_point_.x() +
                      std::cos(planning_init_point_.theta()) *
                          back_axle_to_center);
  target_center.set_y(planning_init_point_.y() +
                      std::sin(planning_init_point_.theta()) *
                          back_axle_to_center);
  planning_init_point_box_ = Box2d(target_center, planning_init_point_.theta(),
                                   vehicle_length, vehicle_width);
}

void StGraphInput::MakePathBorderQuerier(
    const std::shared_ptr<planning_math::KDPath>& planned_path) {
  const double front_edge_to_center = vehicle_param_.front_edge_to_rear_axle;
  const double back_edge_to_center = vehicle_param_.rear_edge_to_rear_axle;
  const double vehicle_length = vehicle_param_.length;
  const double vehicle_width = vehicle_param_.width;
  const double back_axle_to_center_dist =
      0.5 * (front_edge_to_center - back_edge_to_center);

  const int32_t start_idx =
      std::max(0, int32_t(path_range_.first / kPathSampleInterval));
  const int32_t end_idx = int32_t(path_range_.second / kPathSampleInterval);
  const double total_len = path_range_.second - path_range_.first;
  int32_t search_num = int32_t(total_len / kPathSampleInterval) + 1;
  std::vector<PathBorderSegment> path_border_segments;
  path_border_segments.reserve(search_num);
  for (int32_t i = start_idx; i < end_idx; ++i) {
    double desired_s = i * kPathSampleInterval;
    auto point = planned_path->GetPathPointByS(desired_s);
    Vec2d center = point + Vec2d::CreateUnitVec2d(point.theta()) *
                               back_axle_to_center_dist;
    Box2d ego_box(center, point.theta(), vehicle_length, vehicle_width);
    auto corners = ego_box.GetAllCorners();

    LineSegment2d left_segment(corners[2], corners[1]);
    LineSegment2d right_segment(corners[3], corners[0]);

    PathBorderSegment path_border_segment(
        i, desired_s, desired_s - back_edge_to_center,
        desired_s + front_edge_to_center, left_segment, right_segment);
    path_border_segments.emplace_back(path_border_segment);
  }
  path_border_querier_ =
      std::make_shared<PathBorderQuerier>(path_border_segments);
}

void StGraphInput::PlanningInitPointToTrajectoryPoint(
    const PlanningInitPoint& init_point) {
  planning_init_point_.set_x(init_point.x);
  planning_init_point_.set_y(init_point.y);
  planning_init_point_.set_theta(init_point.heading_angle);
  planning_init_point_.set_vel(init_point.v);
  planning_init_point_.set_acc(init_point.a);
  planning_init_point_.set_s(init_point.frenet_state.s);
  planning_init_point_.set_l(init_point.frenet_state.r);
  planning_init_point_.set_dkappa(init_point.dkappa);
  planning_init_point_.set_kappa(init_point.curvature);
  planning_init_point_.set_absolute_time(0.0);
  // planning_init_point_.set_relative_time(init_point.relative_time);
}

SecondOrderTimeOptimalTrajectory StGraphInput::GenerateMaxAccelerationCurve(
    const trajectory::TrajectoryPoint& planning_init_point,
    const std::shared_ptr<EgoStateManager>& ego_state_manager) {
  LonState init_state = {0.0, planning_init_point.vel(),
                         planning_init_point.acc()};

  constexpr double kRefSpeedBuffer = 0.1;
  constexpr double kSpeedBoundFactor = 1.1;
  const double cruise_speed = ego_state_manager->ego_v_cruise();
  const double cruise_speed_uppper_bound = cruise_speed * kSpeedBoundFactor;
  const double ego_speed_upper_bound = init_state.v * kSpeedBoundFactor;

  const double max_speed = init_state.v < cruise_speed_uppper_bound
                               ? cruise_speed_uppper_bound
                               : ego_speed_upper_bound;

  StateLimit state_limit;
  state_limit.v_end = max_speed;

  const double low_speed_acc_upper_bound = 1.8;
  const double high_speed_acc_upper_bound = 1.2;
  const double low_speed_threshold_with_acc_upper_bound = 5.5;
  const double high_speed_threshold_with_acc_upper_bound = 16.67;
  const double acc_lower_bound = -5.0;

  constexpr double kAccBuffer = 0.2;
  constexpr double kSlowJerkUpperBound = 6.0;
  constexpr double kSlowJerkLowerBound = -3.0;
  double acc_upper_bound_with_speed = planning_math::LerpWithLimit(
      low_speed_acc_upper_bound, low_speed_threshold_with_acc_upper_bound,
      high_speed_acc_upper_bound, high_speed_threshold_with_acc_upper_bound,
      init_state.v);
  state_limit.a_max = acc_upper_bound_with_speed - kAccBuffer;
  state_limit.a_min = acc_lower_bound;
  state_limit.j_max = kSlowJerkUpperBound;
  state_limit.j_min = kSlowJerkLowerBound;
  return SecondOrderTimeOptimalTrajectory(init_state, state_limit);
}

void StGraphInput::Reset() {
  is_parallel_lane_map_.clear();
}

}  // namespace speed
}  // namespace planning