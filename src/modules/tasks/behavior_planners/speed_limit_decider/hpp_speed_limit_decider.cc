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

  // 构建平滑限速曲线
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
  v_limit_curv = ComputeCurvatureSpeedLimit(traj_points, &scan_distance);

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

  // --- 遍历绕避 Agent ---
  for (const auto avoid_id : hpp_lateral_output.avoid_ids) {
    const auto* agent = agent_manager->GetAgent(avoid_id);
    if (!agent) {
      continue;
    }

    // 1. 纵向过滤：障碍物需在自车前方
    double agent_s = 0.0, agent_l = 0.0;
    const auto& frenet_coord = reference_path_ptr->get_frenet_coord();
    if (!frenet_coord) {
      continue;
    }
    if (!frenet_coord->XYToSL(agent->x(), agent->y(), &agent_s, &agent_l)) {
      continue;
    }

    // 障碍物后沿到自车头部的距离
    const double longitudinal_dist =
        agent_s - 0.5 * agent->length() - ego_head_s;
    if (longitudinal_dist < 0.0) {
      continue;
    }

    // 2. 横向距离（用中心点近似）
    const double lateral_dist = std::abs(agent_l);

    // 3. 横向阈值：HPP 车道较窄，用固定阈值而非车道宽比例
    const double lateral_threshold =
        hpp_speed_limit_config_.hpp_avoid_lateral_threshold;
    if (lateral_dist > lateral_threshold) {
      continue;
    }

    // 4. 计算目标限速并推送 segment
    const double target_v =
        agent->is_static()
            ? hpp_speed_limit_config_.hpp_avoid_velocity_limit
            : std::max(agent->speed() + kAvoidAgentMinSpeed,
                       hpp_speed_limit_config_.velocity_lower_bound);

    const double approach_dist =
        hpp_speed_limit_config_.hpp_avoid_approach_distance;

    // 障碍物还太远，不触发
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
    const TrajectoryPoints& traj_points, double* scan_dist_out) {
  const double ego_velocity = session_->environmental_model()
                                  .get_ego_state_manager()
                                  ->planning_init_point()
                                  .v;

  // 1. 根据当前车速，插值得到"向前扫描的触发距离"
  //    速度越高，需要向更远处预判曲率
  const double scan_distance = interp(
      ego_velocity, hpp_speed_limit_config_.curv_trigger_velocity_breakpoints,
      hpp_speed_limit_config_.curv_trigger_distances);

  if (scan_dist_out) {
    *scan_dist_out = scan_distance;
  }

  // 2. 考虑方向盘转角对应的即时曲率（车辆当前已经在转弯）
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double steer_curvature =
      std::tan(ego_state->ego_steer_angle() / vehicle_param.steer_ratio) /
      vehicle_param.wheel_base;

  // 3. 在 [0, scan_distance] 范围内，找前方最大曲率及其位置
  //    不再扫描全局，只扫描触发范围内的点
  constexpr double kCurvatureThreshold = 1e-4;        // 忽略近似直道的曲率
  double max_curvature = std::fabs(steer_curvature);  // 先用方向盘曲率初始化
  double max_curv_s = 0.0;  // 方向盘对应的曲率就在当前位置（s=0）
  double s_accumulate = 0.0;

  for (size_t i = 1; i < traj_points.size(); ++i) {
    s_accumulate +=
        planning_math::fast_hypot(traj_points[i].x - traj_points[i - 1].x,
                                  traj_points[i].y - traj_points[i - 1].y);

    if (s_accumulate > scan_distance) {
      break;  // 超出扫描范围，停止
    }

    double curv = std::fabs(traj_points[i].curvature);
    if (curv > max_curvature) {
      max_curvature = curv;
      max_curv_s = s_accumulate;
    }
  }

  // 4. 曲率不足以触发限速，直接退出
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

  // 5. 根据曲率计算目标限速
  double v_limit_curv;

  // HPP 场景：查表（半径 → 限速）
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

  // (a) 初始化为 v_cruise
  std::vector<double> v(N, v_cruise);
  std::vector<SpeedLimitType> binding_type(N, SpeedLimitType::CRUISE);

  // (b) 叠加所有 segments，每个采样点取 min
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

  // (c) 后向平滑（减速约束）
  const double a_decel =
      std::abs(hpp_speed_limit_config_.approaching_zone_deceleration);
  for (int i = N - 2; i >= 0; --i) {
    double v_from_next = std::sqrt(v[i + 1] * v[i + 1] + 2.0 * a_decel * ds);
    if (v_from_next < v[i]) {
      v[i] = v_from_next;
    }
  }

  // (d) 前向平滑（加速约束）— 消除场景间跳变
  //     在无限速场景时，计算到下一个限速场景的距离，
  //     限速为从 ego_v 以 a_accel 加速该距离后能到的速度
  const double a_accel = hpp_speed_limit_config_.hpp_profile_comfort_accel;
  if (binding_type[0] == SpeedLimitType::CRUISE && v[0] > ego_v) {
    double dist_to_next_zone = lookahead;
    for (int i = 1; i < N; ++i) {
      if (binding_type[i] != SpeedLimitType::CRUISE) {
        dist_to_next_zone = i * ds;
        break;
      }
    }
    double v_accel_limit =
        std::sqrt(ego_v * ego_v + 2.0 * a_accel * dist_to_next_zone);
    v[0] = std::min(v[0], v_accel_limit);
  }

  // (e) 输出
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