#include "longitudinal_decision_decider.h"

#include <type_traits>

#include "src/modules/context/environmental_model.h"
#include "src/modules/context/planning_context.h"
#include "utils_math.h"

namespace planning {

LongitudinalDecisionDecider::LongitudinalDecisionDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "LongitudinalDecisionDecider";
  // 读取配置文件
  config_ = config_builder->cast<EgoPlanningConfig>();
  plan_time_ = config_.trajectory_time_length;
  dt_ = config_.planning_dt;
  plan_points_num_ = static_cast<size_t>(plan_time_ / dt_) + 1;
}

void LongitudinalDecisionDecider::Reset() {
  cruise_accelerate_count_.first = 0.0;
  cruise_accelerate_count_.second = 0.0;
}

bool LongitudinalDecisionDecider::Execute() {
  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }

  bool need_reset = false;  // binwang33 TBD: Add reset conditions
  if (need_reset) {
    Reset();
  }

  const auto &environmental_model = session_->environmental_model();
  DetermineKinematicBoundForCruiseScenario();

  // 对于横向侵入无避让的障碍物, 放入ST图中
  // UpdateInvadeNeighborResults();

  // 存储debug信息，这种形式挺好的
  MakeDebugMessage();

  return true;
}

void LongitudinalDecisionDecider::DetermineKinematicBoundForCruiseScenario() {
  const auto &environmental_model = session_->environmental_model();
  const auto &ego_state_mgr = environmental_model.get_ego_state_manager();
  const auto &dynamic_world = environmental_model.get_dynamic_world();
  const auto &planning_context = session_->planning_context();

  // 获取init point
  const auto &planning_init_point = ego_state_mgr->planning_init_point();
  const double ego_vel = planning_init_point.v;
  bool can_increase_acc_bound = true;

  // 1.lane change
  const auto lane_change_status =
      planning_context.lane_change_decider_output().curr_state;
  const bool is_in_lane_keeping =
      lane_change_status == StateMachineLaneChangeStatus::kLaneKeeping;
  if (!is_in_lane_keeping) {
    can_increase_acc_bound = false;
  }

  // 2.cruise speed
  const double cruise_speed = ego_state_mgr->ego_v_cruise();
  if (cruise_speed < kCruiseSpeedMinThd) {
    can_increase_acc_bound = false;
  }

  // 3.speed diff
  const double ego_vel_diff_cruise = ego_vel - cruise_speed;
  if (ego_vel_diff_cruise > -kEgoSpeedWithCruiseSpeedDiffThd) {
    can_increase_acc_bound = false;
  }

  // 4.cutin
  if (dynamic_world == nullptr) {
    return;
  }
  const auto *agent_manager = dynamic_world->agent_manager();
  if (agent_manager == nullptr) {
    return;
  }
  const auto &agents = agent_manager->GetAllCurrentAgents();
  for (const auto *ptr_agent : agents) {
    if (ptr_agent == nullptr) {
      continue;
    }
    if (ptr_agent->is_cutin()) {
      can_increase_acc_bound = false;
      break;
    }
  }

  // 5.path curv
  const auto &planned_path = planning_context.planner_output().planned_path();
  // binwang33: 翼闻添加接口合入后，此处统一掉
  const double k_preview_distance_thd = ego_vel * kEgoPreviewTimeThd;
  double sample_distance = 0.0;
  while (sample_distance < k_preview_distance_thd) {
    sample_distance += kPreviewDistanceStep;
    auto path_point = planned_path.GetPathPointByS(sample_distance);
    const double curr_kappa = path_point.kappa();
    if (curr_kappa > kMaxCurvThd) {
      can_increase_acc_bound = false;
      break;
    }
  }

  // 6.agents average speed
  const double agent_around_average_speed =
      CalculateAgentsAverageSpeedAroundEgo();
  if (agent_around_average_speed <
      cruise_speed * kAgentsAverageSpeedRatioByCruiseThd) {
    can_increase_acc_bound = false;
  }

  // 7.max_acc_curv VS st_corridor
  // 需要获取st信息，判断最大减速度时，和前车是否安全，前车从 agents_headway_map
  // 中获取
  const bool is_max_acc_curv_safe = IsMaxAccCurvSafeInStGraph();
  if (!is_max_acc_curv_safe) {
    can_increase_acc_bound = false;
  }

  // 8.计算增加后的acc bound
  if (can_increase_acc_bound) {
    cruise_accelerate_count_.first =
        std::min(++(cruise_accelerate_count_.first), kIncreaseAccBoundCountThd);
  } else {
    cruise_accelerate_count_.first =
        std::max(--(cruise_accelerate_count_.first), 0);
  }
}

double LongitudinalDecisionDecider::CalculateAgentsAverageSpeedAroundEgo()
    const {
  const auto &environmental_model = session_->environmental_model();
  const auto &ego_state_mgr = environmental_model.get_ego_state_manager();
  const auto &dynamic_world = environmental_model.get_dynamic_world();
  const auto &virtual_lane_manager =
      environmental_model.get_virtual_lane_manager();
  const auto &agent_manager = dynamic_world->agent_manager();

  if (dynamic_world == nullptr || virtual_lane_manager == nullptr) {
    LOG_ERROR("dynamic_world || virtual_lane_manager is nullptr");
    return 0.0;
  }
  if (agent_manager == nullptr) {
    LOG_ERROR("agent_manager is nullptr");
    return 0.0;
  }

  const auto &planning_init_point = ego_state_mgr->planning_init_point();
  const double ego_vel = planning_init_point.v;
  const double cruise_speed = ego_state_mgr->ego_v_cruise();
  const double k_longitudinal_preview_distance =
      ego_vel * kAroundEgoLongitudinalPreviewTimeThd;
  const double k_longitudinal_backward_distance =
      ego_vel * kAroundEgoLongitudinalBackwardTimeThd;

  const auto &ego_lane = virtual_lane_manager->get_current_lane();
  if (ego_lane == nullptr || ego_lane->get_reference_path() == nullptr ||
      ego_lane->get_reference_path()->get_frenet_coord()) {
    return 0.0;
  }

  const auto &ego_lane_coord =
      ego_lane->get_reference_path()->get_frenet_coord();
  double ego_s_base_ego_lane = 0.0, ego_l_base_ego_lane = 0.0;
  if (!ego_lane_coord->XYToSL(planning_init_point.x, planning_init_point.y,
                              &ego_s_base_ego_lane, &ego_l_base_ego_lane)) {
    return 0.0;
  }

  const auto &agents = agent_manager->GetAllCurrentAgents();
  if (agents.empty()) {
    return cruise_speed;
  }

  std::vector<int64_t> agents_around_ego;
  agents_around_ego.reserve(agents.size());
  for (const auto *ptr_agent : agents) {
    double agent_s_base_ego_lane = 0.0, agent_l_base_ego_lane = 0.0;
    if (!ego_lane_coord->XYToSL(ptr_agent->x(), ptr_agent->y(),
                                &agent_s_base_ego_lane,
                                &agent_l_base_ego_lane)) {
      continue;
    }
    const double agent_distance_to_ego =
        agent_s_base_ego_lane - ego_s_base_ego_lane;
    if (std::fabs(agent_l_base_ego_lane) > kAroundEgoLateralDistanceThd) {
      continue;
    }
    if (agent_distance_to_ego > k_longitudinal_preview_distance ||
        agent_distance_to_ego < -k_longitudinal_backward_distance) {
      continue;
    }
    agents_around_ego.emplace_back(ptr_agent->agent_id());
  }
  if (agents_around_ego.empty()) {
    return cruise_speed;
  }

  double agent_around_speed_total = 0.0;

  for (const auto &agent_id : agents_around_ego) {
    const auto *around_agent = agent_manager->GetAgent(agent_id);
    if (around_agent->is_vru()) {
      // Interrupt whenever there is a VRU
      return 0.0;
    }
    agent_around_speed_total += around_agent->speed();
  }
  double agent_around_average_speed =
      agent_around_speed_total / agents_around_ego.size();

  return agent_around_average_speed;
}

bool LongitudinalDecisionDecider::IsMaxAccCurvSafeInStGraph() const {
  constexpr double kMinFollowDistance = 3.0;
  constexpr double kFollowTimeGap = 1.5;
  constexpr double kCruiseAccelerateThd = 1.0;

  const auto &environmental_model = session_->environmental_model();
  const auto &planning_context = session_->planning_context();
  const auto &ego_state_mgr = environmental_model.get_ego_state_manager();

  // 依赖ST的合入
  // const auto *st_graph = planning_context.st_graph_helper();
  // if (st_graph == nullptr) {
  //   return false;
  // }
  // const auto &planning_init_point = ego_state_mgr->planning_init_point();
  // const double ego_vel = planning_init_point.v;

  // const auto& agents_headway_map =
  //     planning_data->decision_output().agent_headway_decider_output().agents_headway_Info();
  // auto max_deceleration_curve = GenerateMaxDecelerationCurve(planning_init_point);


  return true;
}

void LongitudinalDecisionDecider::MakeDebugMessage() {
  // auto planning_debug_msg = planning_data->mutable_planning_debug_message();
  // if (planning_debug_msg == nullptr) {
  // }
  // auto* ptr_debug_string = planning_debug_msg->mutable_debug_string();
  // if (ptr_debug_string == nullptr) {
  //   return;
  // }

  // std::string cruise_acc_count_info = " DetermineKinematicBound: count ";
  // cruise_acc_count_info += std::to_string(cruise_accelerate_count_.first);
  // cruise_acc_count_info += " increase_acc ";
  // cruise_acc_count_info += std::to_string(cruise_accelerate_count_.second);
  // std::stringstream acc_bound;
  // acc_bound << std::fixed << std::setprecision(2) << " acc_bound "
  //           << planning_data->decision_output()
  //                  .longitudinal_decision_decider_output()
  //                  .determined_cruise_bound()
  //                  .acc_positive_mps2;
  // cruise_acc_count_info += acc_bound.str();
  // ptr_debug_string->append(cruise_acc_count_info);
}

}  // namespace planning
