#include "lat_lon_joint_decision_generator.h"

#include <algorithm>

#include "agent/agent.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "ifly_time.h"
#include "lat_lon_joint_decision_output.h"
#include "src/joint_decision_input_builder.h"
#include "src/joint_decision_obstacles_selector.h"
#include "src/joint_decision_planning_problem.h"
#include "trajectory/trajectory.h"
#include "trajectory/trajectory_point.h"

namespace planning {
using namespace lane_change_joint_decision;
LatLonJointDecision::LatLonJointDecision(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session) {
  name_ = "LatLonJointDecision";
  input_builder_ = std::make_unique<
      planning::lane_change_joint_decision::JointDecisionInputBuilder>(
      config_builder, session);
  obstacles_selector_ = std::make_shared<
      planning::lane_change_joint_decision::JointDecisionObstaclesSelector>(
      session);

  input_builder_->SetObstaclesSelector(obstacles_selector_);

  Init();
}

void LatLonJointDecision::Init() {
  planning_problem_ptr_ = std::make_shared<
      pnc::lane_change_joint_decision::JointDecisionPlanningProblem>();
  planning_problem_ptr_->Init();

  const size_t N = 26;
  planning_input_.mutable_ref_x_vec()->Reserve(N);
  planning_input_.mutable_ref_y_vec()->Reserve(N);
  planning_input_.mutable_ref_theta_vec()->Reserve(N);
  planning_input_.mutable_ref_delta_vec()->Reserve(N);
  planning_input_.mutable_ref_vel_vec()->Reserve(N);
  planning_input_.mutable_ref_acc_vec()->Reserve(N);
  planning_input_.mutable_ref_s_vec()->Reserve(N);

  planning_input_.mutable_ref_x_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_y_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_theta_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_delta_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_vel_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_acc_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_s_vec()->Resize(N, 0.0);
}
void LatLonJointDecision::SetLaneChangeDecisionInfo(
    const LaneChangeDecisionInfo& lc_info) {
  lc_prior_info_.gap_front_agent_id = lc_info.gap_front_agent_id;
  lc_prior_info_.gap_rear_agent_id = lc_info.gap_rear_agent_id;
  lc_prior_info_.origin_agent_id = lc_info.origin_agent_id;
  lc_prior_info_.ego_ref_traj = lc_info.ego_ref_traj;
  lc_prior_info_.target_lane_virtual_id = lc_info.target_lane_virtual_id;
  lc_prior_info_.origin_lane_virtual_id = lc_info.origin_lane_virtual_id;
}
void LatLonJointDecision::ClearLaneChangeDecisionInfo() {
  lc_prior_info_.gap_front_agent_id = -1;
  lc_prior_info_.gap_rear_agent_id = -1;
  lc_prior_info_.origin_agent_id = -1;
  lc_prior_info_.ego_ref_traj.clear();
}

bool LatLonJointDecision::Execute() {
  LOG_DEBUG("=======LatLonJointDecision======= \n");

  // if (!PreCheck()) {
  //   LOG_DEBUG("PreCheck failed\n");
  //   return false;
  // }

  auto start_time = IflyTime::Now_ms();

  // input_builder_->BuildInput(planning_input_, planning_problem_ptr_);
  input_builder_->BuildLaneChangeInput(planning_input_, planning_problem_ptr_,
                                       lc_prior_info_);

  auto build_input_end_time = IflyTime::Now_ms();

  Update();

  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("LatLonJointDecisionTime", end_time - start_time);
  JSON_DEBUG_VALUE("JointDecisionOptimizationTime",
                   end_time - build_input_end_time);

  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_joint_decision_input()
      ->CopyFrom(planning_input_);

  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_joint_decision_output()
      ->CopyFrom(planning_problem_ptr_->GetOutput());

  auto& context_output = session_->mutable_planning_context()
                             ->mutable_lat_lon_joint_decision_output();
  context_output = lat_lon_decision_output_;
  SetAgentTrajecTory();
  return true;
}
void LatLonJointDecision::SetAgentTrajecTory() {
  // 判断求解是否成功
  const auto& planning_output = planning_problem_ptr_->GetOutput();
  uint8_t solver_condition = planning_output.solver_info().solver_condition();
  bool planning_success =
      solver_condition < ilqr_solver::iLqr::BACKWARD_PASS_FAIL;
  if (!planning_success) {
    return;
  }
  // 提取障碍物优化轨迹并赋值给 agent
  const auto& key_agent_ids = input_builder_->GetKeyAgentIds();
  std::shared_ptr<agent::AgentManager> agent_manager =
      session_->environmental_model().get_agent_manager();
  const size_t N =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->horizon +
      1;
  const auto& dt =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->model_dt;

  // 处理后方障碍物轨迹
  if (lc_prior_info_.gap_rear_agent_id >= 0 && agent_manager != nullptr) {
    // 在 key_agent_ids 中找到 gap_rear_agent_id 对应的索引
    auto it = std::find(key_agent_ids.begin(), key_agent_ids.end(),
                        static_cast<double>(lc_prior_info_.gap_rear_agent_id));
    if (it != key_agent_ids.end()) {
      size_t obs_index = std::distance(key_agent_ids.begin(), it);
      if (obs_index < planning_output.obs_opt_trajectory_size()) {
        const auto& obs_opt_traj =
            planning_output.obs_opt_trajectory(obs_index);
        trajectory::Trajectory rear_agent_trajectory;
        rear_agent_trajectory.reserve(N);

        for (size_t i = 0; i < N && i < obs_opt_traj.x_vec_size(); ++i) {
          trajectory::TrajectoryPoint point(
              obs_opt_traj.x_vec(i), obs_opt_traj.y_vec(i),
              obs_opt_traj.theta_vec(i), obs_opt_traj.vel_vec(i),
              obs_opt_traj.acc_vec(i),
              i * dt,                     // absolute_time
              obs_opt_traj.omega_vec(i),  // heading_rate
              obs_opt_traj.jerk_vec(i),   // jerk
              obs_opt_traj.s_vec(i));     // s
          rear_agent_trajectory.push_back(point);
        }

        auto* rear_agent = const_cast<agent::Agent*>(
            agent_manager->GetAgent(lc_prior_info_.gap_rear_agent_id));
        if (rear_agent != nullptr) {
          rear_agent->set_trajectory_optimized(rear_agent_trajectory);
        }
      }
    }
  }

  // 处理前方障碍物轨迹（如果需要）
  if (lc_prior_info_.gap_front_agent_id >= 0 && agent_manager != nullptr) {
    auto it = std::find(key_agent_ids.begin(), key_agent_ids.end(),
                        static_cast<double>(lc_prior_info_.gap_front_agent_id));
    if (it != key_agent_ids.end()) {
      size_t obs_index = std::distance(key_agent_ids.begin(), it);
      if (obs_index < planning_output.obs_opt_trajectory_size()) {
        const auto& obs_opt_traj =
            planning_output.obs_opt_trajectory(obs_index);
        trajectory::Trajectory front_agent_trajectory;
        front_agent_trajectory.reserve(N);

        for (size_t i = 0; i < N && i < obs_opt_traj.x_vec_size(); ++i) {
          trajectory::TrajectoryPoint point(
              obs_opt_traj.x_vec(i), obs_opt_traj.y_vec(i),
              obs_opt_traj.theta_vec(i), obs_opt_traj.vel_vec(i),
              obs_opt_traj.acc_vec(i), i * dt, obs_opt_traj.omega_vec(i),
              obs_opt_traj.jerk_vec(i), obs_opt_traj.s_vec(i));
          front_agent_trajectory.push_back(point);
        }

        auto* front_agent = const_cast<agent::Agent*>(
            agent_manager->GetAgent(lc_prior_info_.gap_front_agent_id));
        if (front_agent != nullptr) {
          front_agent->set_trajectory_optimized(front_agent_trajectory);
        }
      }
    }
  }
  return;
}

void LatLonJointDecision::Update() {
  auto solver_condition = planning_problem_ptr_->Update(planning_input_);

  lat_lon_decision_output_.Clear();

  const size_t N =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->horizon +
      1;
  const auto& dt =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->model_dt;
  const auto& planning_output = planning_problem_ptr_->GetOutput();

  const bool motion_failed =
      solver_condition >= ilqr_solver::iLqr::BACKWARD_PASS_FAIL;

  lat_lon_decision_output_.SetPlanningSuccess(!motion_failed);

  const auto& key_agent_ids = input_builder_->GetKeyAgentIds();
  std::vector<int32_t> obstacle_ids;
  obstacle_ids.reserve(key_agent_ids.size());
  for (double id : key_agent_ids) {
    obstacle_ids.push_back(static_cast<int32_t>(id));
  }
  lat_lon_decision_output_.SetSelectedObstacleIds(obstacle_ids);
  if (!motion_failed) {
    auto& ego_trajectory = lat_lon_decision_output_.GetLaneChangeEgoTrajectory();
    ego_trajectory.resize(N);
    const  auto& ref_path = session_->environmental_model().get_reference_path_manager()->get_reference_path_by_current_lane();
    double planning_init_s = 50.0;
    if(ref_path != nullptr) {
      const auto& ego_state = ref_path->get_frenet_ego_state();
      const auto& planning_init_point = ego_state.planning_init_point();
      planning_init_s = planning_init_point.frenet_state.s;
    }
      
    for (size_t i = 0; i < N; ++i) {
      auto& point = ego_trajectory[i];
      point.t = i * dt;
      point.x = planning_output.x_vec(i);
      point.y = planning_output.y_vec(i);
      point.theta = planning_output.theta_vec(i);
      point.vel = planning_output.vel_vec(i);
      point.acc = planning_output.acc_vec(i);
      point.delta = planning_output.delta_vec(i);
      point.omega = planning_output.omega_vec(i);
      point.jerk = planning_output.jerk_vec(i);
      point.s = planning_output.s_vec(i) + planning_init_s;
    }
  }

}

}  // namespace planning
