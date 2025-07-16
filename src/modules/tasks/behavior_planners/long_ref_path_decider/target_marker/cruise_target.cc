#include "cruise_target.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>

#include "behavior_planners/long_ref_path_decider/target_marker/target.h"
#include "behavior_planners/speed_limit_decider/speed_limit_decider_output.h"
#include "common/st_graph/st_graph_utils.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "math/linear_interpolation.h"
#include "planning_context.h"
#include "trajectory1d/piecewise_jerk_acceleration_trajectory1d.h"
#include "utils/pose2d_utils.h"

namespace planning {

namespace {
constexpr double kEpsilon = 1e-6;
constexpr double kNoSpeedLimit = std::numeric_limits<double>::max();
constexpr double kConeSpeedLimitMaxDistanceThreshold = 80.0;
constexpr double kConeSpeedLimitMinDistanceThreshold = 50.0;
constexpr double kLateralDistanceThreshold = 2.4;
constexpr double kSpeedLimitFiftyMetersAhead = 9.72;
constexpr double kMaxSpeedLimitAcc = -2.5;
constexpr double kSameLaneLateralDist = 1.0;
constexpr double kSideLaneLargeRatio = 0.9;
constexpr double kSideLaneSmallRatio = 0.85;
constexpr double kLargeKappaThres = 0.002;
constexpr double kSideMostKappaThreshold = 0.0025;
constexpr double kMinCheckLength = 30.0;
constexpr double kMaxCheckTime = 3.0;
constexpr double kKphToMps = 1.0 / 3.6;
constexpr double kLowSpeedFollowCIPVTrajLength = 35.0;
constexpr double kLowSpeedFollowCIPVDis = 10.0;
constexpr double kLowSpeedFollowTflCIPVDis = 5.0;
//constexpr double kLowSpeedFollowTrajLengthThres = 10.0;
//constexpr double kLowSpeedFollowJerkPosBoundHigh = 1.0;
//constexpr double kLowSpeedFollowJerkPosBoundLow = 0.5;
constexpr double kReleaseBrakeMaxJerk = 6.0;
constexpr double kReleaseAccelMinJerk = -5.0;
}  // namespace

CruiseTarget::CruiseTarget(const SpeedPlannerConfig& config,
                           framework::Session* session)
    : Target(config, session) {
  cruise_target_pb_.Clear();
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& speed_limit_decider_output =
      session_->planning_context().speed_limit_decider_output();
  MakeSpeedLimitKinematicTable(init_lon_state_[1], speed_limit_decider_output);
  // if (speed_limit_kinematics_bound_table_.count(SpeedLimitType::CRUISE) > 0) {
  //   auto& kinematic_bound =
  //       speed_limit_kinematics_bound_table_[SpeedLimitType::CRUISE];
  //   const double determined_cruise_acc_bound =
  //       session_->planning_context()
  //           .longitudinal_decision_decider_output()
  //           .determined_cruise_bound()
  //           .acc_positive_mps2;
  //   kinematic_bound.acc_positive_mps2 =
  //       kinematic_bound.acc_positive_mps2 < determined_cruise_acc_bound
  //           ? determined_cruise_acc_bound
  //           : kinematic_bound.acc_positive_mps2;
  // }
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto lane_change_state = lane_change_decider_output.curr_state;
  const auto is_in_lane_change_execution =
      lane_change_state == kLaneChangeExecution ||
      lane_change_state == kLaneChangeComplete ||
      lane_change_state == kLaneChangeCancel;

  //   MakeObstacleDistanceSpeedLimitTables();
  const double cruise_speed = ego_state_manager->ego_v_cruise();
  double speed_limit_normal = cruise_speed;
  const double speed_limit_from_lane_change =
      is_in_lane_change_execution
          ? config_.lane_change_upper_speed_limit_kph / 3.6
          : std::numeric_limits<double>::max();
  speed_limit_normal =
      std::fmin(speed_limit_normal, speed_limit_from_lane_change);
  double speed_limit_ref = kNoSpeedLimit;
  auto speed_limit_type_ref = SpeedLimitType::NONE;
  speed_limit_decider_output.GetSpeedLimit(&speed_limit_ref,
                                           &speed_limit_type_ref);
  // auto ref_speed_limit_type = SpeedLimitType::CRUISE;
  // if (speed_limit_decider_type != SpeedLimitType::CRUISE &&
  //     speed_limit_decider_result < speed_limit) {
  //   speed_limit = speed_limit_decider_result;
  //   ref_speed_limit_type = speed_limit_decider_type;
  //   // DeterminRoundaboutAccLowerBound();
  // }
  speed_limit_ref = std::fmin(speed_limit_normal, speed_limit_ref);
  //if low speed follow and creep
  double low_speed_follow_a = 0.0;
  double low_speed_follow_j = 0.0;
  if (CalcLowSpeedFollowAccAndJerk(&low_speed_follow_a, &low_speed_follow_j)) {
    if (speed_limit_kinematics_bound_table_.count(speed_limit_type_ref) > 0) {
      auto& kinematic_bound =
        speed_limit_kinematics_bound_table_[speed_limit_type_ref];
      kinematic_bound.acc_positive_mps2 = low_speed_follow_a;
      kinematic_bound.jerk_positive_mps3 = low_speed_follow_j;
      kinematic_bound.jerk_negative_mps3 = kReleaseAccelMinJerk;
    }
  }
  auto acceleration_trajectory1d =
      MakeTarget(speed_limit_ref, speed_limit_type_ref);
  size_t count = 0;
  for (int32_t i = 0; i < plan_points_num_; ++i) {
    const double relative_t = i * dt_;
    bool has_target = true;
    double s_target_value = acceleration_trajectory1d->Evaluate(0, relative_t);
    double v_target_value = acceleration_trajectory1d->Evaluate(1, relative_t);
    TargetType type = TargetType::kCruiseSpeed;
    target_values_.emplace_back(relative_t, has_target, s_target_value,
                                v_target_value, type);
    ++count;
  }
  std::cout << ++count << std::endl;
  AddCruiseTargetDataToProto();
}

bool CruiseTarget::MakeKinematicsBound(
    const double ego_speed, const SpeedLimitType& speed_limit_type,
    KinematicsBound* const kinematic_bound) const {
  auto kinematic_param = config_.comfort_kinematic_param;
  switch (speed_limit_type) {
    case SpeedLimitType::NORMAL_KAPPA:
    case SpeedLimitType::CURVATURE:
      kinematic_param = config_.kappa_kinematic_param;
      break;
    case SpeedLimitType::CRUISE:
      break;
    case SpeedLimitType::NONE:
    case SpeedLimitType::MERGE:
    case SpeedLimitType::CONE_BUCKET:
    case SpeedLimitType::CIPV_LOST:
    case SpeedLimitType::ROUNDABOUT:
    case SpeedLimitType::NOT_OVERTAKE_FROM_RIGHT:
    case SpeedLimitType::VRU_ROUND:
    case SpeedLimitType::MERGE_ALC:
    case SpeedLimitType::MAP_NEAR_RAMP:
    case SpeedLimitType::MAP_ON_RAMP:
    case SpeedLimitType::INTERSECTION:
    case SpeedLimitType::NEAR_TFL:
    case SpeedLimitType::LANE_BORROW:
    case SpeedLimitType::AVOID_AGENT:
      kinematic_param = config_.avoid_agent_kinematic_param;
    case SpeedLimitType::DANGEROUS_OBSTACLE:
      kinematic_param = config_.kappa_kinematic_param;
      break;
    default:
      break;
  }
  // make acc_positive_mps2
  kinematic_bound->acc_positive_mps2 = planning_math::LerpWithLimit(
      kinematic_param.acc_positive_upper,
      kinematic_param.acc_positive_speed_lower,
      kinematic_param.acc_positive_lower,
      kinematic_param.acc_positive_speed_upper, ego_speed);
  // make acc_negative_mps2
  kinematic_bound->acc_negative_mps2 = planning_math::LerpWithLimit(
      kinematic_param.acc_negative_lower,
      kinematic_param.acc_negative_speed_lower,
      kinematic_param.acc_negative_upper,
      kinematic_param.acc_negative_speed_upper, ego_speed);

  // make jerk_positive_mps3
  kinematic_bound->jerk_positive_mps3 = planning_math::LerpWithLimit(
      kinematic_param.jerk_positive_upper,
      kinematic_param.jerk_positive_speed_lower,
      kinematic_param.jerk_positive_lower,
      kinematic_param.jerk_positive_speed_upper, ego_speed);

  // make jerk_negative_mps3
  kinematic_bound->jerk_negative_mps3 = planning_math::LerpWithLimit(
      kinematic_param.jerk_negative_lower,
      kinematic_param.jerk_negative_speed_lower,
      kinematic_param.jerk_negative_upper,
      kinematic_param.jerk_negative_speed_upper, ego_speed);

  return true;
}

bool CruiseTarget::MakeSpeedLimitKinematicTable(
    const double ego_speed,
    const SpeedLimitDeciderOutput& speed_limit_decider_output) {
  const auto& speed_limit_types =
      speed_limit_decider_output.GetAllSpeedLimitTypes();
  for (const auto& type : speed_limit_types) {
    KinematicsBound kinematic_bound;
    MakeKinematicsBound(ego_speed, type, &kinematic_bound);
    speed_limit_kinematics_bound_table_[type] = kinematic_bound;
  }
  return true;
}

bool CruiseTarget::CalcLowSpeedFollowAccAndJerk(double* acc, double* jerk) {
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  double ego_v = ego_state_manager->ego_v();

  const auto& cipv_decider_output =
      session_->planning_context().cipv_decider_output();

  auto cipv_agent_id = cipv_decider_output.cipv_id();
  double cipv_relative_s = cipv_decider_output.relative_s();
  auto* agent = session_->environmental_model().get_agent_manager()->GetAgent(
      cipv_agent_id);
  if (agent == nullptr) {
    return false;
  }
  if (agent->trajectories_used_by_st_graph().empty()) {
    return false;
  }
  const auto& trajectory = agent->trajectories_used_by_st_graph().front();
  if (trajectory.empty()) {
    return false;
  }

  const auto& ego_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();

  if (ego_lane == nullptr) {
    return false;
  }
  // get reference path from ego lane
  const auto& ego_reference_path = ego_lane->get_reference_path();
  if (ego_reference_path == nullptr) {
    return false;
  }
  const auto& ego_lane_coord = ego_reference_path->get_frenet_coord();
  if (ego_lane_coord == nullptr) {
    return false;
  }

  double first_traj_pt_s, first_traj_pt_l;
  double last_traj_pt_s, last_traj_pt_l;
  const auto& first_point = trajectory.front();
  const auto& end_point = trajectory.back();
  if (!ego_lane_coord->XYToSL(first_point.x(), first_point.y(),
                              &first_traj_pt_s, &first_traj_pt_l)) {
    return false;
  }
  if (!ego_lane_coord->XYToSL(end_point.x(), end_point.y(), &last_traj_pt_s,
                              &last_traj_pt_l)) {
    return false;
  }
  double cipv_traj_length = last_traj_pt_s - first_traj_pt_s;
  const bool is_tfl_virtual_agent = agent->is_tfl_virtual_obs();
  const double low_speed_follow_Cipv_dis =
      is_tfl_virtual_agent ? kLowSpeedFollowTflCIPVDis : kLowSpeedFollowCIPVDis;
  if (cipv_traj_length < kLowSpeedFollowCIPVTrajLength &&
      cipv_relative_s < low_speed_follow_Cipv_dis) {
    if (cipv_traj_length < config_.low_speed_follow_accel_release_traj_len &&
        ego_v > config_.low_speed_follow_speed_thred_mps) {
      *acc = 0.0;
    } else {
      *acc = interp(cipv_traj_length,
                    config_.low_speed_follow_acc_traj_table.traj_table,
                    config_.low_speed_follow_acc_traj_table.acc_table);
    }
    //*jerk = (cipv_traj_length < kLowSpeedFollowTrajLengthThres) ?
    //        kLowSpeedFollowJerkPosBoundLow:  kLowSpeedFollowJerkPosBoundHigh;
    *jerk = interp(cipv_traj_length,
                   config_.low_speed_follow_jerk_traj_table.traj_table,
                   config_.low_speed_follow_jerk_traj_table.jerk_table);
    return true;
  } else {
    return false;
  }
}

std::unique_ptr<PiecewiseJerkAccelerationTrajectory1d> CruiseTarget::MakeTarget(
    const double ref_speed, const SpeedLimitType& ref_speed_limit_type) {
  auto ptr_lon_traj = std::make_unique<PiecewiseJerkAccelerationTrajectory1d>(
      init_lon_state_[0], init_lon_state_[1], init_lon_state_[2]);
  const double t_step = dt_;
  ptr_lon_traj->AppendSegment(init_lon_state_[2], kEpsilon);
  SpeedLimitBoundInfo speed_limit_bound_info;
  for (double t = kEpsilon; t <= planning_time_; t += t_step) {
    const double t_protection =
        std::fmin(ptr_lon_traj->ParamLength() - kEpsilon, t);
    const double s = ptr_lon_traj->Evaluate(0, t_protection);
    const double v = ptr_lon_traj->Evaluate(1, t_protection);
    const double a = ptr_lon_traj->Evaluate(2, t_protection);
    // calculate speed upper bound with path curvature
    // const double path_curvature_speed_limit =
    //     MakeSpeedUpperBound(s, t_protection, &speed_limit_bound_info);
    // KinematicsBound kinematic_bound =
    //     speed_limit_kinematics_bound_table_[SpeedLimitType::CRUISE];
    // if (ref_speed < path_curvature_speed_limit) {
    //   kinematic_bound =
    //       speed_limit_kinematics_bound_table_[ref_speed_limit_type];
    // } else {
    //   kinematic_bound =
    //       speed_limit_kinematics_bound_table_[SpeedLimitType::NORMAL_KAPPA];
    // }
    // const double objective_speed =
    //     std::fmin(ref_speed, path_curvature_speed_limit);
    // double a_next = (objective_speed - v) / t_step;
    double a_next = (ref_speed - v) / t_step;
    KinematicsBound kinematic_bound =
        speed_limit_kinematics_bound_table_[ref_speed_limit_type];
    a_next =
        CalculateAccelerationWithinBound(a_next, a, t_step, kinematic_bound);
    ptr_lon_traj->AppendSegment(a_next, t_step);
  }
  return ptr_lon_traj;
}

double CruiseTarget::CalculateAccelerationWithinBound(
    const double a_next, const double a_t, const double t_step_length,
    const KinematicsBound& kinematics_bound) const {
  double acc_next = a_next;
  acc_next = std::min(acc_next, kinematics_bound.acc_positive_mps2);
  acc_next = std::max(acc_next, kinematics_bound.acc_negative_mps2);
  if (acc_next > 0.5 && a_t < -0.5) {
    acc_next = std::min(
      acc_next, a_t + kReleaseBrakeMaxJerk * t_step_length);
  } else {
    acc_next = std::min(
        acc_next, a_t + kinematics_bound.jerk_positive_mps3 * t_step_length);
  }
  acc_next = std::max(
      acc_next, a_t + kinematics_bound.jerk_negative_mps3 * t_step_length);
  return acc_next;
}

double CruiseTarget::MakeSpeedUpperBound(
    const double s, const double t,
    SpeedLimitBoundInfo* const speed_limit_bound_info) {
  std::vector<double> speed_upper_bound_list(
      static_cast<size_t>(SpeedUpperBoundType::kMaxType),
      std::numeric_limits<double>::max());
  // 1. consider path kappa
  speed_upper_bound_list[static_cast<size_t>(SpeedUpperBoundType::kCurvature)] =
      CalculateSpeedUpperBoundWithPathCurvature(s, speed_limit_bound_info);
  // 2. consider close pass agent
  //   speed_upper_bound_list[static_cast<size_t>(SpeedUpperBoundType::kClosePass)]
  //   =
  //       CalculateSpeedUpperBoundWithClosePassAgent(s, t,
  //       speed_limit_bound_info);
  return *std::min_element(speed_upper_bound_list.begin(),
                           speed_upper_bound_list.end());
}

double CruiseTarget::CalculateSpeedUpperBoundWithPathCurvature(
    const double current_s,
    SpeedLimitBoundInfo* const speed_limit_bound_info) const {
  const auto& planned_kd_path =
      session_->planning_context().motion_planner_output().lateral_path_coord;
  double s = std::min(current_s, planned_kd_path->Length());
  auto path_point = planned_kd_path->GetPathPointByS(s);
  const double curr_kappa = path_point.kappa();
  speed_limit_bound_info->max_kappa =
      std::fmax(speed_limit_bound_info->max_kappa, std::fabs(curr_kappa));
  // make speed with kappa
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto lane_change_state = lane_change_decider_output.curr_state;
  const auto is_in_lane_change_execution =
      lane_change_state == kLaneChangeExecution ||
      lane_change_state == kLaneChangeComplete ||
      lane_change_state == kLaneChangeCancel;
  //   const cp_common::DrivingStyle driving_style =
  //       planning_data_.system_manager_info().driving_style();
  double kappa_limit_speed = 0.0;
  if (is_in_lane_change_execution) {
    double lane_change_speed_limit_speed =
        MatchSpeedWithKappaSpeedLimitTable(true, curr_kappa);
    kappa_limit_speed = lane_change_speed_limit_speed;
  } else {
    const double normal_kappa_limit_speed =
        MatchSpeedWithKappaSpeedLimitTable(false, curr_kappa);
    kappa_limit_speed = normal_kappa_limit_speed;
  }

  return kappa_limit_speed;
}

double CruiseTarget::MatchSpeedWithKappaSpeedLimitTable(
    const bool is_lane_change_execution, const double kappa,
    const DrivingStyle driving_style) const {
  const auto& table = is_lane_change_execution
                          ? config_.lane_change_kappa_speed_limit_table
                          : config_.normal_kappa_speed_limit_table;
  double speed = interp(kappa, table.kappa_table, table.speed_table);
  return speed * kKphToMps;
}

void CruiseTarget::AddCruiseTargetDataToProto() {
  auto& debug_info_pb = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto mutable_cruise_target_data =
      debug_info_pb->mutable_lon_target_s_ref()->mutable_cruise_target();
  if (!target_values_.empty()) {
    for (const auto& value : target_values_) {
      auto* ptr = cruise_target_pb_.add_cruise_target_s_ref();
      ptr->set_s(value.s_target_val());
      ptr->set_t(value.relative_t());
      ptr->set_target_type(static_cast<int32_t>(value.target_type()));
    }
  }
  mutable_cruise_target_data->CopyFrom(cruise_target_pb_);
}

// TODO: This function is confusing!
// double CruiseTarget::CalculateSpeedUpperBoundWithClosePassAgent(
//     const double s, const double relative_t,
//     SpeedLimitBoundInfo* const speed_limit_bound_info) const {
//   using STPoint = planning::STPoint;
//   using StGraphUtils = planning::speed::StGraphUtils;
//   const auto dynamic_world =
//       session_->environmental_model().get_dynamic_world();
//   constexpr double kCartesianDistanceThreshold = 5.0;
//   double min_speed_limited_by_distance = 2.78;  // 10kph
//   double speed_limit = std::numeric_limits<double>::max();
//   const bool enable = true /*config_.distance_speed_limit_table().enable()*/;
//   const auto& close_pass_boundary_id_st_boundaries_map =
//       session_->planning_context()
//           .st_graph_helper()
//           ->close_pass_boundary_id_st_boundaries_map();
//   //   .st_graph_helper()->close_pass_boundary_id_st_boundaries_map();
//   const auto& planned_kd_path =
//       session_->planning_context().motion_planner_output().lateral_path_coord;
//   if (!enable || nullptr == speed_limit_bound_info ||
//       close_pass_boundary_id_st_boundaries_map.empty() ||
//       nullptr == dynamic_world || nullptr == dynamic_world->agent_manager())
//       {
//     return speed_limit;
//   }

//   const auto ego_box = StGraphUtils::MakeEgoBox(planned_kd_path, s);
//   const double absolute_time =
//   session_->environmental_model().get_ego_state_manager()->planning_init_point().
//     //   planning_data_.planning_init_point().absolute_time();
//   const auto* agent_manager =
//   planning_data_.dynamic_world()->agent_manager();

//   STPoint ego_lower_point;
//   STPoint ego_upper_point;
//   StGraphUtils::CalculateEgoStPoint(s, relative_t, &ego_lower_point,
//                                     &ego_upper_point);
//   for (const auto& item : close_pass_boundary_id_st_boundaries_map) {
//     const int64_t boundary_id = item.first;
//     const auto& st_boundary = item.second;
//     if (st_boundary == nullptr) {
//       continue;
//     }
//     STPoint agent_lower_point;
//     STPoint agent_upper_point;
//     if (!st_boundary->GetBoundaryBounds(relative_t, &agent_lower_point,
//                                         &agent_upper_point)) {
//       continue;
//     }
//     double distance = StGraphUtils::CalculateDistanceBetween(
//         ego_lower_point, ego_upper_point, agent_lower_point,
//         agent_upper_point);
//     const int32_t agent_id = agent_upper_point.agent_id();
//     const auto* ptr_agent = agent_manager->GetAgent(agent_id);
//     proto::ObstacleDistanceSpeedLimitTable::Mode mode =
//         ptr_agent ? GetModeByAgentType(*ptr_agent)
//                   : proto::ObstacleDistanceSpeedLimitTable::NORMAL;
//     if (distance < kCartesianDistanceThreshold && ptr_agent) {
//       distance = StGraphUtils::DistanceToEgoBox(ego_box, *ptr_agent,
//                                                 absolute_time + relative_t);
//     }
//     double non_negative_obstacle_speed =
//         std::max(0.0, agent_upper_point.velocity());
//     double tmp_limit = std::max(
//         min_speed_limited_by_distance,
//         MapDistance2Speed(mode, distance) + non_negative_obstacle_speed);
//         //here is confusing
//     speed_limit = std::min(speed_limit, tmp_limit);
//   }
//   return speed_limit;
// }

}  // namespace planning