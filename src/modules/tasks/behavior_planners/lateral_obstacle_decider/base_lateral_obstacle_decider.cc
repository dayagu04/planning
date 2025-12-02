#include "base_lateral_obstacle_decider.h"
#include "src/modules/tasks/behavior_planners/general_lateral_decider/general_lateral_decider_utils.h"

namespace planning {
BaseLateralObstacleDecider::BaseLateralObstacleDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  session_ = session;
  config_ = config_builder->cast<LateralObstacleDeciderConfig>();
}

bool BaseLateralObstacleDecider::Execute() { return true; }

void BaseLateralObstacleDecider::ConstructPlanHistoryTraj(
    const std::shared_ptr<ReferencePath> &reference_path_ptr) {
  if (reference_path_ptr == nullptr) {
    return;
  }
  auto &plan_history_traj = session_->mutable_planning_context()
                                ->mutable_lateral_obstacle_decider_output()
                                .plan_history_traj;
  auto &is_plan_history_traj_valid =
      session_->mutable_planning_context()
          ->mutable_lateral_obstacle_decider_output()
          .is_plan_history_traj_valid;
  const auto &frenet_coord = reference_path_ptr->get_frenet_coord();
  auto &last_traj_points = session_->mutable_planning_context()
                               ->mutable_last_planning_result()
                               .raw_traj_points;
  TrajectoryPoints plan_history_traj_tmp;
  for (size_t i = 0; i < last_traj_points.size(); ++i) {
    // frenet info
    Point2D frenet_pt{0.0, 0.0};
    Point2D cart_pt(last_traj_points[i].x, last_traj_points[i].y);
    if (frenet_coord->XYToSL(cart_pt, frenet_pt)) {
      last_traj_points[i].s = frenet_pt.x;
      last_traj_points[i].l = frenet_pt.y;
      plan_history_traj_tmp.emplace_back(last_traj_points[i]);
    } else {
      ILOG_DEBUG << "plan_history_traj frenet error";
    }
  }
  if (plan_history_traj_tmp.empty()) {
    is_plan_history_traj_valid = false;
    return;
  }
  auto ego_s = reference_path_ptr->get_frenet_ego_state()
                   .planning_init_point()
                   .frenet_state.s;
  // auto ego_s = ego_frenet_state_.s();
  if (ego_s <= plan_history_traj_tmp.front().s) {
    for (double t = 0; t <= plan_history_traj_tmp.back().t;
         t += config_.delta_t) {
      TrajectoryPoint pt =
          general_lateral_decider_utils::GetTrajectoryPointAtTime(
              plan_history_traj_tmp, t);
      pt.s = pt.s - (ego_s - plan_history_traj_tmp.front().s);
      plan_history_traj.emplace_back(std::move(pt));
    }
  } else if (ego_s >= plan_history_traj_tmp.back().s) {
    // assert(false);
  } else {
    int index = 1;
    while (index < plan_history_traj_tmp.size()) {
      if (plan_history_traj_tmp[index].s >= ego_s) {
        break;
      }
      index++;
    }
    const auto &traj_1 = plan_history_traj_tmp[index - 1];
    const auto &traj_2 = plan_history_traj_tmp[index];
    const double weight0 = (ego_s - traj_1.s) / (traj_2.s - traj_1.s);
    const double weight1 = 1.0 - weight0;
    const double base_t = weight1 * traj_1.t + weight0 * traj_2.t;
    for (double t = base_t; t <= plan_history_traj_tmp.back().t;
         t += config_.delta_t) {
      TrajectoryPoint pt =
          general_lateral_decider_utils::GetTrajectoryPointAtTime(
              plan_history_traj_tmp, t);
      plan_history_traj.emplace_back(std::move(pt));
    }
    for (auto &traj : plan_history_traj) {
      traj.t -= base_t;
    }
    if (plan_history_traj.size() == 0) {
    } else {
      for (int point_num = plan_history_traj.size();
           point_num < config_.num_step + 1; point_num++) {
        TrajectoryPoint pt = plan_history_traj.back();
        // For now, only s and t are modified
        pt.s += pt.v * config_.delta_t;
        pt.t += config_.delta_t;
        plan_history_traj.emplace_back(std::move(pt));
      }
    }
  }
  is_plan_history_traj_valid = true;
}
}  // namespace planning