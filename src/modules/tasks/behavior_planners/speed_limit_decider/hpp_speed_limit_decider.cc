#include "hpp_speed_limit_decider.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "debug_info_log.h"
#include "environmental_model.h"
#include "planning_context.h"

namespace planning {

using planning::common::HppSpeedLimitDeciderDebug;
using planning::common::HppSpeedLimitSimpleSnapshot;
using planning::common::HppSpeedLimitType;
using planning::common::HppSpeedLimitZoneSnapshot;

inline HppSpeedLimitType ToProtoSpeedLimitType(SpeedLimitType t) {
  return static_cast<HppSpeedLimitType>(static_cast<int>(t));
}

inline HppSpeedLimitDeciderDebug* MutableHppSpeedLimitDeciderDebug() {
  return DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_hpp_speed_limit_decider_debug();
}

void FillSimpleSnapshot(HppSpeedLimitSimpleSnapshot* snap, SpeedLimitType type,
                        double v_limit) {
  if (snap == nullptr) {
    return;
  }
  snap->set_module_type(ToProtoSpeedLimitType(type));
  snap->set_v_limit(v_limit);
}

void FillZoneSnapshot(HppSpeedLimitZoneSnapshot* snap, SpeedLimitType type,
                      double v_limit, const HPPSpeedLimitZoneInfo& zone) {
  if (snap == nullptr) {
    return;
  }
  snap->set_module_type(ToProtoSpeedLimitType(type));
  snap->set_v_limit(v_limit);
  snap->set_in_speed_limit_zone(zone.in_speed_limit_zone);
  snap->set_approaching_speed_limit_zone(zone.approaching_speed_limit_zone);
  snap->set_distance_to_zone(zone.distance_to_zone);
  snap->clear_s_segments();
  for (const auto& seg : zone.s_segments) {
    auto* p = snap->add_s_segments();
    p->set_first(seg.first);
    p->set_second(seg.second);
  }
}

HPPSpeedLimitDecider::HPPSpeedLimitDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session) {
  hpp_speed_limit_config_ = config_builder->cast<LongitudinalDeciderV3Config>();
  name_ = "HPPSpeedLimitDecider";
}

bool HPPSpeedLimitDecider::Execute() {
  ILOG_INFO << "=======HPPSpeedLimitDecider=======";
  const auto& environmental_model = session_->environmental_model();
  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  const auto plan_init_point = ego_state_mgr->planning_init_point();
  double v_cruise = ego_state_mgr->ego_v_cruise();
  v_target_ = v_cruise;
  v_target_type_ = SpeedLimitType::CRUISE;

  speed_limit_segments_.clear();

  CalculateUserSpeedLimit();

  CalculateMapSpeedLimit();

  CalculateCurveSpeedLimit();

  CalculateNarrowAreaSpeedLimit();

  CalculateAvoidLimit();

  CalculateBumpLimit();

  CalculateRampLimit();

  CalculateIntersectionRoadLimit();

  // construct a smooth speed limit curve
  const auto& reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  if (reference_path_ptr && !speed_limit_segments_.empty()) {
    const auto& frenet_ego_state = reference_path_ptr->get_frenet_ego_state();
    const double ego_s = frenet_ego_state.s();
    const double ego_v = frenet_ego_state.planning_init_point().v;
    BuildSmoothedSpeedProfile(ego_s, ego_v, v_cruise);
  }

  auto hpp_speed_limit_output = session_->mutable_planning_context()
                                    ->mutable_speed_limit_decider_output();
  hpp_speed_limit_output->SetSpeedLimit(v_target_, v_target_type_);

#ifdef ENABLE_PROTO_LOG
  {
    auto* dbg = MutableHppSpeedLimitDeciderDebug();
    dbg->set_final_v_target(v_target_);
    dbg->set_final_speed_limit_type(static_cast<int32_t>(
        std::underlying_type<SpeedLimitType>::type(v_target_type_)));
  }
#endif

  auto ad_info = &(session_->mutable_planning_context()
                       ->mutable_planning_hmi_info()
                       ->ad_info);

  if (SpeedLimitType::CURVATURE == v_target_type_) {
    ad_info->is_curva = true;
  } else {
    ad_info->is_curva = false;
  }

  return true;
}

void HPPSpeedLimitDecider::CalculateUserSpeedLimit() {
  const double user_velocity_limit =
      session_->environmental_model().get_ego_state_manager()->ego_v_cruise();
#ifdef ENABLE_PROTO_LOG
  FillSimpleSnapshot(MutableHppSpeedLimitDeciderDebug()->mutable_user(),
                     SpeedLimitType::USER, user_velocity_limit);
#endif

  if (user_velocity_limit < v_target_) {
    v_target_ = user_velocity_limit;
    v_target_type_ = SpeedLimitType::USER;
  }
}

void HPPSpeedLimitDecider::CalculateMapSpeedLimit() { return; }

void HPPSpeedLimitDecider::CalculateCurveSpeedLimit() {
  double v_limit_curv = hpp_speed_limit_config_.velocity_upper_bound;
  if (!session_->is_hpp_scene()) {
    return;
  }

  const auto& traj_points = session_->mutable_planning_context()
                                ->mutable_planning_result()
                                .traj_points;

  if (traj_points.empty()) {
    return;
  }

  const double max_lat_acceleration = ComputeMaxLatAcceleration();

  double scan_distance = 0.0;
  v_limit_curv = ComputeCurvatureSpeedLimit(traj_points, scan_distance);

#ifdef ENABLE_PROTO_LOG
  FillSimpleSnapshot(MutableHppSpeedLimitDeciderDebug()->mutable_curvature(),
                     SpeedLimitType::CURVATURE, v_limit_curv);
#endif

  if (v_limit_curv < hpp_speed_limit_config_.velocity_upper_bound &&
      scan_distance > 0.0) {
    const auto& reference_path_ptr = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info.reference_path;
    if (reference_path_ptr) {
      const double ego_s = reference_path_ptr->get_frenet_ego_state().s();
      speed_limit_segments_.push_back({ego_s, ego_s + scan_distance,
                                       v_limit_curv,
                                       SpeedLimitType::CURVATURE});
    }
  }
}

void HPPSpeedLimitDecider::CalculateNarrowAreaSpeedLimit() {
  double v_limit_speed_narrow_passage =
      hpp_speed_limit_config_.velocity_upper_bound;
  if (!session_->is_hpp_scene()) {
    return;
  }

  const double approach_distance_threshold =
      hpp_speed_limit_config_.speed_narrow_passage_approach_distance;
  HPPSpeedLimitZoneInfo zone_info;

  if (!BuildSpeedObjectiveZoneInfo(
          zone_info, CRoadType::Ignore, CPassageType::NarrowPassage,
          CElemType::Ignore, approach_distance_threshold)) {
#ifdef ENABLE_PROTO_LOG
    FillZoneSnapshot(
        MutableHppSpeedLimitDeciderDebug()->mutable_narrow_passage(),
        SpeedLimitType::NARROW_PASSAGE, v_limit_speed_narrow_passage,
        zone_info);
#endif
    return;
  }

  for (const auto& seg : zone_info.s_segments) {
    speed_limit_segments_.push_back(
        {seg.first, seg.second,
         hpp_speed_limit_config_.target_speed_narrow_passage_area,
         SpeedLimitType::NARROW_PASSAGE});
  }

  LOG_DEBUG(
      "Speed bump: v_limit=%f m/s, in_zone=%d, approaching=%d, distance=%f m",
      hpp_speed_limit_config_.target_speed_narrow_passage_area,
      zone_info.in_speed_limit_zone, zone_info.approaching_speed_limit_zone,
      zone_info.distance_to_zone);

#ifdef ENABLE_PROTO_LOG
  FillZoneSnapshot(MutableHppSpeedLimitDeciderDebug()->mutable_narrow_passage(),
                   SpeedLimitType::NARROW_PASSAGE,
                   hpp_speed_limit_config_.target_speed_narrow_passage_area,
                   zone_info);
#endif
  return;
}

void HPPSpeedLimitDecider::CalculateAvoidLimit() {
  double v_limit_avoid = hpp_speed_limit_config_.velocity_upper_bound;

  if (!session_->is_hpp_scene()) {
    return;
  }

  const auto& hpp_lateral_output =
      session_->planning_context().hpp_general_lateral_decider_output();
  if (hpp_lateral_output.avoid_ids.empty()) {
#ifdef ENABLE_PROTO_LOG
    FillSimpleSnapshot(MutableHppSpeedLimitDeciderDebug()->mutable_avoid(),
                       SpeedLimitType::AVOID, v_limit_avoid);
#endif
    return;
  }

  const auto& reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  if (!reference_path_ptr) {
    return;
  }

  constexpr double kAvoidAgentMinSpeed = 2.0;

  const auto& frenet_ego_state = reference_path_ptr->get_frenet_ego_state();
  const double ego_s = frenet_ego_state.s();
  const double ego_head_s = frenet_ego_state.head_s();

  const auto agent_manager =
      session_->environmental_model().get_agent_manager();
  if (!agent_manager) {
    return;
  }

  for (const auto avoid_id : hpp_lateral_output.avoid_ids) {
    const auto* agent = agent_manager->GetAgent(avoid_id);
    if (!agent) {
      continue;
    }

    // 1. longitudinal filtering: Obstacles must be ahead of the self-driving
    // vehicle
    double agent_s = 0.0, agent_l = 0.0;
    const auto& frenet_coord = reference_path_ptr->get_frenet_coord();
    if (!frenet_coord) {
      continue;
    }
    if (!frenet_coord->XYToSL(agent->x(), agent->y(), &agent_s, &agent_l)) {
      continue;
    }

    // the distance from the rear edge of the obstacle to the front of the
    // self-driving vehicle
    const double longitudinal_dist =
        agent_s - 0.5 * agent->length() - ego_head_s;
    if (longitudinal_dist < 0.0) {
      continue;
    }

    // 2. horizontal distance (approximation using center point)
    const double lateral_dist = std::abs(agent_l);

    // 3. horizontal threshold: HPP lanes are narrower, so a fixed threshold is
    // used instead of the lane width ratio
    const double lateral_threshold =
        hpp_speed_limit_config_.hpp_avoid_lateral_threshold;
    if (lateral_dist > lateral_threshold) {
      continue;
    }

    // 4. calculate the target speed limit and push the segment
    const double target_v =
        agent->is_static()
            ? hpp_speed_limit_config_.hpp_avoid_velocity_limit
            : std::max(agent->speed() + kAvoidAgentMinSpeed,
                       hpp_speed_limit_config_.velocity_lower_bound);

    const double approach_dist =
        hpp_speed_limit_config_.hpp_avoid_approach_distance;

    // the obstacle is too far away, so it won't be triggered
    if (longitudinal_dist > approach_dist) {
      continue;
    }

    const double seg_start = agent_s - 0.5 * agent->length();
    const double seg_end = agent_s + 0.5 * agent->length();
    speed_limit_segments_.push_back(
        {seg_start, seg_end, target_v, SpeedLimitType::AVOID});
  }

#ifdef ENABLE_PROTO_LOG
  FillSimpleSnapshot(MutableHppSpeedLimitDeciderDebug()->mutable_avoid(),
                     SpeedLimitType::AVOID,
                     hpp_speed_limit_config_.velocity_upper_bound);
#endif
}

const double HPPSpeedLimitDecider::ComputeMaxLatAcceleration() {
  constexpr double kUrbanVelocityThld = 3.0;
  constexpr double kHighwayVelocityThld = 20.0;
  constexpr double kUrbanLatAccBuffMax = 0.7;
  constexpr double kHighwayLatAccBuffMax = 0.5;
  constexpr double kUrbanLatAccRate = 0.2;
  constexpr double kHighwayLatAccRate = 0.0;
  constexpr double kLowSpeedLaneChangeAccBuff = 0.3;
  const auto& reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  const FrenetEgoState& frenet_ego_state =
      reference_path_ptr->get_frenet_ego_state();
  double ego_velocity = frenet_ego_state.planning_init_point().v;
  const CoarsePlanningInfo& coarse_planning_info =
      session_->planning_context()
          .lane_change_decider_output()
          .coarse_planning_info;
  const LaneChangeDeciderOutput& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const StateMachineLaneChangeStatus state = coarse_planning_info.target_state;
  const int lc_request_direction = lane_change_decider_output.lc_request;
  bool is_LC_LCHANGE =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == LEFT_CHANGE);
  bool is_LC_RCHANGE =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == RIGHT_CHANGE);
  auto lc_acc_decay =
      ((is_LC_LCHANGE || is_LC_RCHANGE) && ego_velocity < kUrbanVelocityThld)
          ? kLowSpeedLaneChangeAccBuff
          : 0.0;

  const double max_lat_acceleration =
      hpp_speed_limit_config_.urban_max_lat_acceleration - lc_acc_decay +
      std::min(
          kUrbanLatAccBuffMax,
          kUrbanLatAccRate * std::max(ego_velocity - kUrbanVelocityThld, 0.0));
  return max_lat_acceleration;
}

const double HPPSpeedLimitDecider::ComputeCurvatureSpeedLimit(
    const TrajectoryPoints& traj_points, double& scan_dist_out) {
  const double ego_velocity = session_->environmental_model()
                                  .get_ego_state_manager()
                                  ->planning_init_point()
                                  .v;

  // 1. based on the current vehicle speed, the "trigger distance for forward
  // scanning" is obtained through interpolation
  const double scan_distance = interp(
      ego_velocity, hpp_speed_limit_config_.curv_trigger_velocity_breakpoints,
      hpp_speed_limit_config_.curv_trigger_distances);

  scan_dist_out = scan_distance;

  // 2. consider the instantaneous curvature corresponding to the steering wheel
  // angle (the vehicle is currently turning)
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double steer_curvature =
      std::tan(ego_state->ego_steer_angle() / vehicle_param.steer_ratio) /
      vehicle_param.wheel_base;

  // 3. within the range of [0, scan_distance], find the maximum curvature ahead
  // and its position. Do not scan the entire area, but only scan the points
  // within the triggering range
  constexpr double kCurvatureThreshold = 1e-4;
  double max_curvature = std::fabs(steer_curvature);
  double max_curv_s = 0.0;
  double s_accumulate = 0.0;

  for (size_t i = 1; i < traj_points.size(); ++i) {
    s_accumulate +=
        planning_math::fast_hypot(traj_points[i].x - traj_points[i - 1].x,
                                  traj_points[i].y - traj_points[i - 1].y);

    if (s_accumulate > scan_distance) {
      break;  // out of scanning range, stop
    }

    double curv = std::fabs(traj_points[i].curvature);
    if (curv > max_curvature) {
      max_curvature = curv;
      max_curv_s = s_accumulate;
    }
  }

  // 4. the curvature is not sufficient to trigger speed restriction, exiting
  // directly
  if (max_curvature <= kCurvatureThreshold) {
    max_curvature_ = max_curvature;
#ifdef ENABLE_PROTO_LOG
    FillSimpleSnapshot(MutableHppSpeedLimitDeciderDebug()->mutable_curvature(),
                       SpeedLimitType::CURVATURE,
                       hpp_speed_limit_config_.velocity_upper_bound);
#endif
    return hpp_speed_limit_config_.velocity_upper_bound;
  }

  max_curvature_ = max_curvature;

  // 5. calculate the target speed limit based on curvature
  double v_limit_curv;

  double radius = 1.0 / max_curvature;
  v_limit_curv = interp(radius, hpp_speed_limit_config_.curv_radius_breakpoints,
                        hpp_speed_limit_config_.curv_speed_limits_ms);

  v_limit_curv =
      std::clamp(v_limit_curv, hpp_speed_limit_config_.velocity_lower_bound,
                 hpp_speed_limit_config_.velocity_upper_bound);

  return v_limit_curv;
}

void HPPSpeedLimitDecider::CalculateBumpLimit() {
  double v_limit_speed_bump = hpp_speed_limit_config_.velocity_upper_bound;
  if (!session_->is_hpp_scene()) {
    return;
  }

  const double approach_distance_threshold =
      hpp_speed_limit_config_.speed_bump_approach_distance;
  HPPSpeedLimitZoneInfo zone_info;

  if (!BuildSpeedObjectiveZoneInfo(
          zone_info, CRoadType::Ignore, CPassageType::Ignore,
          CElemType::SpeedBumpRoad, approach_distance_threshold)) {
    auto& planning_result =
        session_->mutable_planning_context()->mutable_planning_result();
    planning_result.speed_bump_path_segments.clear();
#ifdef ENABLE_PROTO_LOG
    FillZoneSnapshot(MutableHppSpeedLimitDeciderDebug()->mutable_bump(),
                     SpeedLimitType::BUMP_ROAD, v_limit_speed_bump, zone_info);
#endif
    return;
  }

  auto& planning_result =
      session_->mutable_planning_context()->mutable_planning_result();
  planning_result.speed_bump_path_segments.clear();

  for (const auto& seg : zone_info.s_segments) {
    planning_result.speed_bump_path_segments.push_back({seg.first, seg.second});
    speed_limit_segments_.push_back(
        {seg.first, seg.second,
         hpp_speed_limit_config_.target_speed_speed_bump_area,
         SpeedLimitType::BUMP_ROAD});
  }

  LOG_DEBUG(
      "Speed bump: v_limit=%f m/s, in_zone=%d, approaching=%d, distance=%f m",
      hpp_speed_limit_config_.target_speed_speed_bump_area,
      zone_info.in_speed_limit_zone, zone_info.approaching_speed_limit_zone,
      zone_info.distance_to_zone);

#ifdef ENABLE_PROTO_LOG
  FillZoneSnapshot(MutableHppSpeedLimitDeciderDebug()->mutable_bump(),
                   SpeedLimitType::BUMP_ROAD,
                   hpp_speed_limit_config_.target_speed_speed_bump_area,
                   zone_info);
#endif
}

bool HPPSpeedLimitDecider::BuildSpeedObjectiveZoneInfo(
    HPPSpeedLimitZoneInfo& zone_info, const CRoadType& road_type,
    const CPassageType& passage_type, const CElemType& elem_type,
    const double approach_distance_threshold) {
  const auto& reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;

  if (!reference_path_ptr) {
    return false;
  }

  const auto& frenet_ego_state = reference_path_ptr->get_frenet_ego_state();
  const double ego_head_s = frenet_ego_state.head_s();
  const double ego_s = frenet_ego_state.s();

  const ConstStaticAnalysisStoragePtr static_analysis_storage =
      reference_path_ptr->get_static_analysis_storage();
  if (!static_analysis_storage) {
    return false;
  }

  QueryTypeInfo query_info = {road_type, passage_type, elem_type};

  const SRangeList object_near_range_list =
      static_analysis_storage->GetSRangeList(query_info);
  if (object_near_range_list.empty()) {
    return false;
  }

  const std::pair<double, double> object_near_range =
      static_analysis_storage->GetFrontSRange(query_info, ego_s);

  if (object_near_range.first > object_near_range.second) {
    return false;
  }

  zone_info.s_segments = object_near_range_list;
  zone_info.in_speed_limit_zone = object_near_range.first <= ego_head_s &&
                                  object_near_range.second >= ego_s;

  zone_info.distance_to_zone =
      std::max(0.0, object_near_range.first - ego_head_s);
  zone_info.approaching_speed_limit_zone =
      zone_info.distance_to_zone > 0 &&
      zone_info.distance_to_zone < approach_distance_threshold;

  return true;
}

double HPPSpeedLimitDecider::GetSpeedLimitInObjectiveZone(
    const HPPSpeedLimitZoneInfo& zone_info, const double target_v) {
  // 区域内限速
  const double kSpeedObjectiveZoneLimit = target_v;
  // 减速度
  const double kDecelerationRate =
      hpp_speed_limit_config_.approaching_zone_deceleration;

  if (zone_info.in_speed_limit_zone || zone_info.approaching_speed_limit_zone) {
    // 接近区域，根据距离和减速度计算限速
    // 使用运动学公式: v² = v0² + 2*a*s
    // 其中 v = kSpeedObjectiveZoneLimit, a = kDecelerationRate, s =
    // distance_to_zone 求解 v0 = sqrt(v² - 2*a*s)
    double target_velocity_squared =
        kSpeedObjectiveZoneLimit * kSpeedObjectiveZoneLimit;
    double velocity_squared_at_distance =
        target_velocity_squared -
        2.0 * kDecelerationRate * zone_info.distance_to_zone;

    return std::sqrt(velocity_squared_at_distance);
  }

  return std::numeric_limits<double>::max();
}

void HPPSpeedLimitDecider::BuildSmoothedSpeedProfile(double ego_s, double ego_v,
                                                     double v_cruise) {
  const double lookahead = hpp_speed_limit_config_.hpp_profile_lookahead;
  const double ds = hpp_speed_limit_config_.hpp_profile_ds;
  const int N = static_cast<int>(lookahead / ds) + 1;

  // 1. initialize to v_cruise
  std::vector<double> v(N, v_cruise);
  std::vector<SpeedLimitType> binding_type(N, SpeedLimitType::CRUISE);

  // 2. stack all segments and take the minimum value at each sampling point
  for (const auto& seg : speed_limit_segments_) {
    int i_start = std::max(0, static_cast<int>((seg.s_start - ego_s) / ds));
    int i_end = std::min(N - 1, static_cast<int>((seg.s_end - ego_s) / ds));
    for (int i = i_start; i <= i_end; ++i) {
      if (seg.v_limit < v[i]) {
        v[i] = seg.v_limit;
        binding_type[i] = seg.type;
      }
    }
  }

  // 3. backward smoothing (deceleration constraint)
  const double a_decel =
      std::abs(hpp_speed_limit_config_.approaching_zone_deceleration);
  for (int i = N - 2; i >= 0; --i) {
    double v_from_next = std::sqrt(v[i + 1] * v[i + 1] + 2.0 * a_decel * ds);
    if (v_from_next < v[i]) {
      v[i] = v_from_next;
    }
  }

  // 4. forward smoothing (acceleration constraint) - key: eliminate jumps
  // between scenes, the starting point is set to max(v[0], ego_v): no speed
  // limit reduction within the zone, and prevention of sudden changes outside
  // the zone
  const double a_accel = hpp_speed_limit_config_.hpp_profile_comfort_accel;
  double v_prev = std::max(v[0], ego_v);
  for (int i = 1; i < N; ++i) {
    double v_from_prev = std::sqrt(v_prev * v_prev + 2.0 * a_accel * ds);
    if (v_from_prev < v[i]) {
      v[i] = v_from_prev;
    }
    v_prev = v[i];
  }

  // 4. output
  v_target_ =
      std::clamp(v[0], hpp_speed_limit_config_.velocity_lower_bound, v_cruise);
  v_target_type_ = binding_type[0];
}

void HPPSpeedLimitDecider::CalculateRampLimit() {
  double v_limit_speed_ramp = hpp_speed_limit_config_.velocity_upper_bound;
  if (!session_->is_hpp_scene()) {
    return;
  }
  const double approach_distance_threshold =
      hpp_speed_limit_config_.speed_ramp_approach_distance;
  HPPSpeedLimitZoneInfo zone_info;

  if (!BuildSpeedObjectiveZoneInfo(zone_info, CRoadType::Ignore,
                                   CPassageType::Ignore, CElemType::RampRoad,
                                   approach_distance_threshold)) {
#ifdef ENABLE_PROTO_LOG
    FillZoneSnapshot(MutableHppSpeedLimitDeciderDebug()->mutable_ramp(),
                     SpeedLimitType::RAMP_ROAD, v_limit_speed_ramp, zone_info);
#endif
    return;
  }

  for (const auto& seg : zone_info.s_segments) {
    speed_limit_segments_.push_back(
        {seg.first, seg.second, hpp_speed_limit_config_.target_speed_ramp_area,
         SpeedLimitType::RAMP_ROAD});
  }

  LOG_DEBUG(
      "Speed bump: v_limit=%f m/s, in_zone=%d, approaching=%d, distance=%f m",
      hpp_speed_limit_config_.target_speed_ramp_area,
      zone_info.in_speed_limit_zone, zone_info.approaching_speed_limit_zone,
      zone_info.distance_to_zone);

#ifdef ENABLE_PROTO_LOG
  FillZoneSnapshot(MutableHppSpeedLimitDeciderDebug()->mutable_ramp(),
                   SpeedLimitType::RAMP_ROAD,
                   hpp_speed_limit_config_.target_speed_ramp_area, zone_info);
#endif

  return;
}

void HPPSpeedLimitDecider::CalculateIntersectionRoadLimit() {
  double v_limit_speed_intersection =
      hpp_speed_limit_config_.velocity_upper_bound;
  if (!session_->is_hpp_scene()) {
    return;
  }

  const double approach_distance_threshold =
      hpp_speed_limit_config_.speed_intersection_approach_distance;
  HPPSpeedLimitZoneInfo zone_info;

  if (!BuildSpeedObjectiveZoneInfo(
          zone_info, CRoadType::Ignore, CPassageType::Ignore,
          CElemType::IntersectionRoad, approach_distance_threshold)) {
#ifdef ENABLE_PROTO_LOG
    FillZoneSnapshot(MutableHppSpeedLimitDeciderDebug()->mutable_intersection(),
                     SpeedLimitType::INTERSECTION_ROAD,
                     v_limit_speed_intersection, zone_info);
#endif
    return;
  }

  for (const auto& seg : zone_info.s_segments) {
    speed_limit_segments_.push_back(
        {seg.first, seg.second,
         hpp_speed_limit_config_.target_speed_intersection_road_area,
         SpeedLimitType::INTERSECTION_ROAD});
  }

  LOG_DEBUG(
      "Speed bump: v_limit=%f m/s, in_zone=%d, approaching=%d, distance=%f m",
      hpp_speed_limit_config_.target_speed_intersection_road_area,
      zone_info.in_speed_limit_zone, zone_info.approaching_speed_limit_zone,
      zone_info.distance_to_zone);

#ifdef ENABLE_PROTO_LOG
  FillZoneSnapshot(MutableHppSpeedLimitDeciderDebug()->mutable_intersection(),
                   SpeedLimitType::INTERSECTION_ROAD,
                   hpp_speed_limit_config_.target_speed_intersection_road_area,
                   zone_info);
#endif

  return;
}
}  // namespace planning