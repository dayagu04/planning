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
#include "math/math_utils.h"
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
  if (!session_->is_hpp_scene()) {
    return;
  }

  const auto& traj_points = session_->mutable_planning_context()
                                ->mutable_planning_result()
                                .traj_points;

  if (traj_points.empty()) {
    return;
  }

  // 1. Scan trajectory once and build a curvature profile shared by the
  //    curvature-related sub-functions.
  CurvatureProfile profile = ScanTrajectory(traj_points);

  // 2. Base curvature-based speed limit.
  double v_limit_curv = ComputeCurvatureSpeedLimit(profile);

  // 3. S-curve speed limit.
  CalculateSCurveLimit(v_limit_curv);

  // 4. U-turn speed limit.
  CalculateUTurnLimit(v_limit_curv);
}

void HPPSpeedLimitDecider::CalculateNarrowAreaSpeedLimit() {
  if (!session_->is_hpp_scene()) {
    return;
  }
  // Physical narrow-passage detection from hard_bounds.
  CalculateNarrowAreaSpeedLimitFromBounds();
  // Map-annotated narrow-passage detection.
  CalculateNarrowAreaSpeedLimitFromMap();
}

void HPPSpeedLimitDecider::CalculateNarrowAreaSpeedLimitFromMap() {
  const double v_limit_speed_narrow_passage =
      hpp_speed_limit_config_.velocity_upper_bound;
  const double approach_distance_threshold =
      hpp_speed_limit_config_.speed_narrow_passage_approach_distance;
  HPPSpeedLimitZoneInfo zone_info;

  // Narrow passage: half-vehicle exit is NOT enabled. Lateral clearance is
  // tight, so the ego must be fully out of the zone before resuming normal
  // speed, otherwise the rear overhang may scrape or hug the edge.
  if (!BuildSpeedObjectiveZoneInfo(
          zone_info, CRoadType::Ignore, CPassageType::NarrowPassage,
          CElemType::Ignore, approach_distance_threshold, false)) {
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

    // 2. Determine parameters based on obstacle type.
    const agent::AgentType agent_type = agent->type();
    bool is_vru = (agent_type == agent::AgentType::PEDESTRIAN ||
                   agent_type == agent::AgentType::ADULT ||
                   agent_type == agent::AgentType::CHILD ||
                   agent_type == agent::AgentType::BICYCLE ||
                   agent_type == agent::AgentType::CYCLE_RIDING ||
                   agent_type == agent::AgentType::MOTORCYCLE_RIDING ||
                   agent_type == agent::AgentType::TRICYCLE_RIDING);

    // Lateral threshold: relax by 20% for VRUs.
    double lateral_threshold =
        hpp_speed_limit_config_.hpp_avoid_lateral_threshold;
    if (is_vru) {
      lateral_threshold *= 1.2;  // VRU lateral threshold +20%
    }

    const double lateral_dist = std::abs(agent_l);
    if (lateral_dist > lateral_threshold) {
      continue;
    }

    // 3. Longitudinal trigger distance: trigger earlier for VRUs.
    double approach_dist = hpp_speed_limit_config_.hpp_avoid_approach_distance;
    if (is_vru) {
      approach_dist *= 1.5;  // VRU trigger +50% earlier
    }

    if (longitudinal_dist > approach_dist) {
      continue;
    }

    // 4. Compute the target speed limit.
    const double target_v =
        agent->is_static()
            ? hpp_speed_limit_config_.hpp_avoid_velocity_limit
            : std::max(agent->speed() + kAvoidAgentMinSpeed,
                       hpp_speed_limit_config_.velocity_lower_bound);

    // 5. Segment extension: VRUs need a longer deceleration window.
    double extend_forward = is_vru ? 5.0 : 0.0;
    double extend_backward = is_vru ? 1.0 : 0.0;

    const double seg_start = agent_s - 0.5 * agent->length() - extend_forward;
    const double seg_end = agent_s + 0.5 * agent->length() + extend_backward;
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

CurvatureProfile HPPSpeedLimitDecider::ScanTrajectory(
    const TrajectoryPoints& traj_points) {
  CurvatureProfile profile;

  const double ego_velocity = session_->environmental_model()
                                  .get_ego_state_manager()
                                  ->planning_init_point()
                                  .v;

  // Interpolate scan distance from current ego velocity.
  profile.scan_distance = interp(
      ego_velocity, hpp_speed_limit_config_.curv_trigger_velocity_breakpoints,
      hpp_speed_limit_config_.curv_trigger_distances);

  // Account for the instantaneous curvature induced by the steering angle.
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double steer_curvature =
      std::tan(ego_state->ego_steer_angle() / vehicle_param.steer_ratio) /
      vehicle_param.wheel_base;

  // Seed the profile with the steering curvature at s=0.
  profile.s_values.push_back(0.0);
  profile.curvature_values.push_back(steer_curvature);
  profile.max_curvature = std::fabs(steer_curvature);
  profile.max_curvature_s = 0.0;

  // Walk the trajectory points, recording s and curvature.
  double s_accumulate = 0.0;
  for (size_t i = 1; i < traj_points.size(); ++i) {
    s_accumulate +=
        planning_math::fast_hypot(traj_points[i].x - traj_points[i - 1].x,
                                  traj_points[i].y - traj_points[i - 1].y);

    if (s_accumulate > profile.scan_distance) {
      break;
    }

    profile.s_values.push_back(s_accumulate);
    profile.curvature_values.push_back(traj_points[i].curvature);

    double curv_abs = std::fabs(traj_points[i].curvature);
    if (curv_abs > profile.max_curvature) {
      profile.max_curvature = curv_abs;
      profile.max_curvature_s = s_accumulate;
    }
  }

  return profile;
}

double HPPSpeedLimitDecider::ComputeCurvatureSpeedLimit(
    const CurvatureProfile& profile) {
  constexpr double kCurvatureThreshold = 1e-4;

  // Curvature below threshold: no speed limit triggered.
  if (profile.max_curvature <= kCurvatureThreshold) {
    max_curvature_ = profile.max_curvature;
#ifdef ENABLE_PROTO_LOG
    FillSimpleSnapshot(MutableHppSpeedLimitDeciderDebug()->mutable_curvature(),
                       SpeedLimitType::CURVATURE,
                       hpp_speed_limit_config_.velocity_upper_bound);
#endif
    return hpp_speed_limit_config_.velocity_upper_bound;
  }

  max_curvature_ = profile.max_curvature;

  // Compute speed limit from the maximum curvature.
  double radius = 1.0 / profile.max_curvature;
  double v_limit_curv =
      interp(radius, hpp_speed_limit_config_.curv_radius_breakpoints,
             hpp_speed_limit_config_.curv_speed_limits_ms);

  v_limit_curv =
      std::clamp(v_limit_curv, hpp_speed_limit_config_.velocity_lower_bound,
                 hpp_speed_limit_config_.velocity_upper_bound);

#ifdef ENABLE_PROTO_LOG
  FillSimpleSnapshot(MutableHppSpeedLimitDeciderDebug()->mutable_curvature(),
                     SpeedLimitType::CURVATURE, v_limit_curv);
#endif

  // Emit speed limit segment.
  if (v_limit_curv < hpp_speed_limit_config_.velocity_upper_bound &&
      profile.scan_distance > 0.0) {
    const auto& reference_path_ptr = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info.reference_path;
    if (reference_path_ptr) {
      const double ego_s = reference_path_ptr->get_frenet_ego_state().s();
      speed_limit_segments_.push_back({ego_s, ego_s + profile.scan_distance,
                                       v_limit_curv,
                                       SpeedLimitType::CURVATURE});
    }
  }

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
          CElemType::SpeedBumpRoad, approach_distance_threshold, true)) {
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
    const double approach_distance_threshold, bool use_half_vehicle_exit) {
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

  if (use_half_vehicle_exit) {
    // Vehicle params do not change at runtime; cache on first call to avoid
    // reading VehicleConfigurationContext every frame.
    static const double half_vehicle_length =
        VehicleConfigurationContext::Instance()->get_vehicle_param().length *
        0.5;
    for (auto& seg : zone_info.s_segments) {
      seg.second = std::max(seg.first, seg.second - half_vehicle_length);
    }
  }

  zone_info.in_speed_limit_zone = object_near_range.first <= ego_head_s &&
                                  object_near_range.second >= ego_s;

  zone_info.distance_to_zone =
      std::max(0.0, object_near_range.first - ego_head_s);
  zone_info.approaching_speed_limit_zone =
      zone_info.distance_to_zone > 0 &&
      zone_info.distance_to_zone < approach_distance_threshold;

  return true;
}

void HPPSpeedLimitDecider::BuildSmoothedSpeedProfile(double ego_s, double ego_v,
                                                     double v_cruise) {
  const double lookahead = hpp_speed_limit_config_.hpp_profile_lookahead;
  const double ds = hpp_speed_limit_config_.hpp_profile_ds;
  const int N = static_cast<int>(lookahead / ds) + 1;

  std::vector<double> v(N, v_cruise);
  std::vector<SpeedLimitType> binding_type(N, SpeedLimitType::CRUISE);

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

  const double a_decel =
      std::abs(hpp_speed_limit_config_.approaching_zone_deceleration);
  for (int i = N - 2; i >= 0; --i) {
    double v_from_next = std::sqrt(v[i + 1] * v[i + 1] + 2.0 * a_decel * ds);
    if (v_from_next < v[i]) {
      v[i] = v_from_next;
    }
  }

  const double a_accel = hpp_speed_limit_config_.hpp_profile_comfort_accel;
  if (binding_type[0] == SpeedLimitType::CRUISE && v[0] > ego_v) {
    double dist_to_next_zone = lookahead;
    double next_zone_v = v_cruise;
    for (int i = 1; i < N; ++i) {
      if (binding_type[i] != SpeedLimitType::CRUISE) {
        dist_to_next_zone = i * ds;
        next_zone_v = v[i];
        break;
      }
    }

    double v_peak_sq =
        (a_decel * ego_v * ego_v + a_accel * next_zone_v * next_zone_v +
         2.0 * a_accel * a_decel * dist_to_next_zone) /
        (a_accel + a_decel);
    double v_peak = std::sqrt(std::max(v_peak_sq, ego_v * ego_v));

    constexpr double kMinSpeedGain = 1.0;
    if (v_peak - ego_v < kMinSpeedGain) {
      v[0] = ego_v;
    } else {
      double v_accel_limit =
          std::sqrt(ego_v * ego_v + 2.0 * a_accel * dist_to_next_zone);
      v[0] = std::min(v[0], v_accel_limit);
    }
  }

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
                                   approach_distance_threshold, true)) {
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
          CElemType::IntersectionRoad, approach_distance_threshold, true)) {
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

void HPPSpeedLimitDecider::CalculateSCurveLimit(double v_limit_curv) {
  const auto& reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  if (!reference_path_ptr) {
    return;
  }

  const ConstStaticAnalysisStoragePtr static_analysis_storage =
      reference_path_ptr->get_static_analysis_storage();
  if (!static_analysis_storage) {
    return;
  }

  QueryTypeInfo query_info = {CRoadType::SCurveStaight, CPassageType::Ignore,
                              CElemType::Ignore};

  const SRangeList s_curve_range_list =
      static_analysis_storage->GetSRangeList(query_info);
  if (s_curve_range_list.empty()) {
    return;
  }

  const auto& frenet_ego_state = reference_path_ptr->get_frenet_ego_state();
  const double ego_s = frenet_ego_state.s();

  constexpr double kSCurveSpeedFactor = 0.8;

  // S-curve speed limit = base curvature limit * 0.8.
  double s_curve_v_limit = v_limit_curv * kSCurveSpeedFactor;
  s_curve_v_limit =
      std::clamp(s_curve_v_limit, hpp_speed_limit_config_.velocity_lower_bound,
                 hpp_speed_limit_config_.velocity_upper_bound);

  for (const auto& s_range : s_curve_range_list) {
    const double s_start = s_range.first;
    const double s_end = s_range.second;

    if (s_end < ego_s) {
      continue;
    }

    // Use the map-annotated zone directly; early deceleration is handled by
    // the backward pass in BuildSmoothedSpeedProfile.
    speed_limit_segments_.push_back(
        {s_start, s_end, s_curve_v_limit, SpeedLimitType::CURVATURE});

    ILOG_INFO << "S-curve zone: s=[" << s_start << ", " << s_end
              << "], v_limit=" << s_curve_v_limit << " m/s";
  }
}

void HPPSpeedLimitDecider::CalculateUTurnLimit(double v_limit_curv) {
  const auto& reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  if (!reference_path_ptr) {
    return;
  }

  const ConstStaticAnalysisStoragePtr static_analysis_storage =
      reference_path_ptr->get_static_analysis_storage();
  if (!static_analysis_storage) {
    return;
  }

  QueryTypeInfo query_info = {CRoadType::UTurn, CPassageType::Ignore,
                              CElemType::Ignore};

  const SRangeList uturn_range_list =
      static_analysis_storage->GetSRangeList(query_info);
  if (uturn_range_list.empty()) {
    return;
  }

  const auto& frenet_ego_state = reference_path_ptr->get_frenet_ego_state();
  const double ego_s = frenet_ego_state.s();

  // U-turn speed factor: U-turns have very high curvature and need a tighter
  // speed limit.
  constexpr double kUTurnSpeedFactor = 0.8;

  // U-turn speed limit = base curvature limit * 0.8.
  double uturn_v_limit = v_limit_curv * kUTurnSpeedFactor;
  uturn_v_limit =
      std::clamp(uturn_v_limit, hpp_speed_limit_config_.velocity_lower_bound,
                 hpp_speed_limit_config_.velocity_upper_bound);

  for (const auto& s_range : uturn_range_list) {
    const double s_start = s_range.first;
    const double s_end = s_range.second;

    if (s_end < ego_s) {
      continue;
    }

    speed_limit_segments_.push_back(
        {s_start, s_end, uturn_v_limit, SpeedLimitType::CURVATURE});

    ILOG_INFO << "U-turn zone: s=[" << s_start << ", " << s_end
              << "], v_limit=" << uturn_v_limit << " m/s";
  }
}
void HPPSpeedLimitDecider::CalculateNarrowAreaSpeedLimitFromBounds() {
  const auto& lateral_output =
      session_->planning_context().general_lateral_decider_output();
  const auto& hard_bounds_frenet_point =
      lateral_output.hard_bounds_frenet_point;
  const auto& hard_bounds_info = lateral_output.hard_bounds_info;

  const auto& traj_points =
      session_->planning_context().planning_result().traj_points;

  if (hard_bounds_frenet_point.empty() ||
      hard_bounds_frenet_point.size() != hard_bounds_info.size() ||
      hard_bounds_frenet_point.size() != traj_points.size()) {
    return;
  }

  const auto& reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  if (!reference_path_ptr) {
    return;
  }

  const auto& frenet_ego_state = reference_path_ptr->get_frenet_ego_state();
  const double ego_s = frenet_ego_state.s();
  const double ego_v = frenet_ego_state.planning_init_point().v;

  const double lookahead =
      interp(ego_v, hpp_speed_limit_config_.narrow_passage_velocity_breakpoints,
             hpp_speed_limit_config_.narrow_passage_lookahead_distances);
  const double s_end = ego_s + lookahead;

  constexpr double kControlDelay = 0.25;
  const double delay_distance = ego_v * kControlDelay;
  const double target_s = ego_s + delay_distance;

  const size_t n = traj_points.size();
  double target_v_limit = hpp_speed_limit_config_.velocity_upper_bound;
  bool has_limit = false;

  for (size_t i = 0; i < n; ++i) {
    const double s = traj_points[i].s;

    if (s < ego_s) {
      continue;
    }

    if (s > s_end) {
      break;
    }

    const double width =
        hard_bounds_frenet_point[i].second - hard_bounds_frenet_point[i].first;
    const double v_limit = ComputeNarrowSpeedLimit(
        width, hard_bounds_info[i].first, hard_bounds_info[i].second);

    if (s >= target_s && !has_limit) {
      if (v_limit < hpp_speed_limit_config_.velocity_upper_bound) {
        target_v_limit = v_limit;
        has_limit = true;
        break;
      }
    }
  }

  if (has_limit) {
    speed_limit_segments_.push_back(
        {ego_s, s_end, target_v_limit, SpeedLimitType::NARROW_PASSAGE});
  }
}

bool HPPSpeedLimitDecider::IsDynamicBound(BoundType type) const {
  return type == BoundType::AGENT || type == BoundType::DYNAMIC_AGENT ||
         type == BoundType::ADJACENT_AGENT ||
         type == BoundType::LOW_PRIORITY_AGENT ||
         type == BoundType::REVERSE_AGENT;
}

bool HPPSpeedLimitDecider::IsImmovableObstacle(agent::AgentType type) const {
  return type == agent::AgentType::UNKNOWN_IMMOVABLE ||
         type == agent::AgentType::TRAFFIC_CONE ||
         type == agent::AgentType::TRAFFIC_BARREL ||
         type == agent::AgentType::FENCE ||
         type == agent::AgentType::WATER_SAFETY_BARRIER ||
         type == agent::AgentType::CTASH_BARREL ||
         type == agent::AgentType::COLUMN ||
         type == agent::AgentType::CYLINDER_BARRIER ||
         type == agent::AgentType::CONSTRUCTION_SIGNS;
}

double HPPSpeedLimitDecider::ComputeObstacleSpeedCompensation(
    const BoundInfo& bound_info) const {
  // Non-obstacle bound (road boundary, etc.), id = -100.
  if (bound_info.id == -100) {
    return 0.0;  // No compensation for static immovable objects.
  }

  // Fetch agent_manager.
  const auto agent_manager =
      session_->environmental_model().get_agent_manager();
  if (!agent_manager) {
    return 0.0;
  }

  const auto* agent = agent_manager->GetAgent(bound_info.id);
  if (!agent) {
    return 0.0;
  }

  const agent::AgentType agent_type = agent->type();

  // 1. Static immovable obstacles: no extra deceleration.
  if (IsImmovableObstacle(agent_type)) {
    return 0.0;
  }

  // 2. Interpolate compensation based on obstacle speed.
  const double speed = agent->speed();
  return planning_math::ClampInterpolate(
      hpp_speed_limit_config_.narrow_obstacle_speed_lower,
      hpp_speed_limit_config_.narrow_obstacle_compensation_lower,
      hpp_speed_limit_config_.narrow_obstacle_speed_upper,
      hpp_speed_limit_config_.narrow_obstacle_compensation_upper, speed);
}

double HPPSpeedLimitDecider::ComputeNarrowSpeedLimit(
    double width, const BoundInfo& left_bound_info,
    const BoundInfo& right_bound_info) const {
  const double vehicle_width =
      VehicleConfigurationContext::Instance()->get_vehicle_param().max_width;

  // Base speed limit: interpolate from passage width.
  const double v_base = planning_math::ClampInterpolate(
      hpp_speed_limit_config_.narrow_speed_width_lower,
      hpp_speed_limit_config_.narrow_speed_v_lower, vehicle_width,
      hpp_speed_limit_config_.narrow_speed_v_upper, width);

  // Take the stricter compensation between the two sides.
  const double compensation =
      std::max(ComputeObstacleSpeedCompensation(left_bound_info),
               ComputeObstacleSpeedCompensation(right_bound_info));

  return std::clamp(v_base - compensation,
                    hpp_speed_limit_config_.velocity_lower_bound,
                    hpp_speed_limit_config_.velocity_upper_bound);
}

}  // namespace planning