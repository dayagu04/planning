#include "rads_lateral_obstacle_decider.h"

#include "edt_manager.h"
#include "environmental_model.h"
#include "task_interface/lateral_obstacle_decider_output.h"

namespace planning {

RADSLateralObstacleDecider::RADSLateralObstacleDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : BaseLateralObstacleDecider(config_builder, session) {
  name_ = "RADSLateralObstacleDecider";
}

bool RADSLateralObstacleDecider::Execute() {
  if (!PreCheck()) {
    output_.clear();
    last_output_.clear();
    ILOG_DEBUG << "PreCheck failed";
    return false;
  }

  InitInfo();

  ConstructPlanHistoryTraj(reference_path_ptr_);

  UpdateLatDecision();

  GenerateOutput();

  Log();

  return true;
}

void RADSLateralObstacleDecider::InitInfo() {
  reference_path_ptr_ = session_->planning_context()
                                .lane_change_decider_output()
                                .coarse_planning_info.reference_path;
  output_.clear();
  auto &lateral_obstacle_decider_output =
      session_->mutable_planning_context()->mutable_lateral_obstacle_decider_output();
  auto &plan_history_traj = lateral_obstacle_decider_output.plan_history_traj;
  auto &is_plan_history_traj_valid =
      lateral_obstacle_decider_output.is_plan_history_traj_valid;
  plan_history_traj.clear();
  is_plan_history_traj_valid = false;
}

void RADSLateralObstacleDecider::UpdateLatDecision() {
  auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double init_s =
      reference_path_ptr_->get_frenet_ego_state().planning_init_point().frenet_state.s;
  const double kMinCareLatThr = 0.5 * vehicle_param.max_width - 0.1;
  // const double kMaxCareLatThr = 0.5 * vehicle_param.max_width + 0.25;
  const double kPosChangeBuffer = 0.05;
  for (auto &obstacle : reference_path_ptr_->get_obstacles()) {
    if (obstacle->b_frenet_valid()) {
      const double obstacle_l_start =
          obstacle->frenet_obstacle_boundary().l_start;
      const double obstacle_l_end =
          obstacle->frenet_obstacle_boundary().l_end;
      const double obstacle_s_end =
          obstacle->frenet_obstacle_boundary().s_end;
      if (EdtManager::FilterObstacleForAra(*obstacle)) {
        LatObstacleDecisionType last_decision = LatObstacleDecisionType::NOT_SET;
        auto last_itr = last_output_.find(obstacle->id());
        if (last_itr != last_output_.end()) {
          last_decision = last_itr->second;
        }
        double pos_change_buffer = 0.0;
        if (last_decision == LatObstacleDecisionType::LEFT ||
            last_decision == LatObstacleDecisionType::RIGHT) {
          pos_change_buffer = kPosChangeBuffer;
        } else if (last_decision == LatObstacleDecisionType::IGNORE) {
          pos_change_buffer = -kPosChangeBuffer;
        }
        if (obstacle_s_end > init_s) {
          if (obstacle->frenet_l() > 0) {
            // 左侧
            if (obstacle_l_start >= (kMinCareLatThr - pos_change_buffer)) {
                // && obstacle_l_start <= (kMaxCareLatThr + pos_change_buffer)) {
              output_[obstacle->id()] = LatObstacleDecisionType::RIGHT;
            } else {
              output_[obstacle->id()] = LatObstacleDecisionType::IGNORE;
            }
          } else {
            // 右侧
            if (obstacle_l_end <= (-kMinCareLatThr + kPosChangeBuffer)) {
                // && obstacle_l_end >= (-kMaxCareLatThr - kPosChangeBuffer)) {
              output_[obstacle->id()] =
                  LatObstacleDecisionType::LEFT;
            } else {
              output_[obstacle->id()] =
                  LatObstacleDecisionType::IGNORE;
            }
          }
        } else {
          output_[obstacle->id()] = LatObstacleDecisionType::IGNORE;
        }
      } else {
        output_[obstacle->id()] = LatObstacleDecisionType::IGNORE;
      }
    } else {
      output_[obstacle->id()] = LatObstacleDecisionType::IGNORE;
    }
  }
}

void RADSLateralObstacleDecider::GenerateOutput() {
  auto &lateral_obstacle_decider_output =
      session_->mutable_planning_context()->mutable_lateral_obstacle_decider_output();
  lateral_obstacle_decider_output.lat_obstacle_decision = output_;
  last_output_ = output_;
};

void RADSLateralObstacleDecider::Log() {
#ifdef ENABLE_PROTO_LOG
  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto environment_model_debug_info =
      planning_debug_data->mutable_environment_model_info();
  environment_model_debug_info->clear_obstacle();
  auto &lat_obstacle_decision = session_->mutable_planning_context()
                                        ->mutable_lateral_obstacle_decider_output()
                                        .lat_obstacle_decision;
  for (auto &obstacle : reference_path_ptr_->get_obstacles()) {
    // log
    planning::common::Obstacle *obstacle_log =
        environment_model_debug_info->add_obstacle();
    obstacle_log->set_id(obstacle->id());
    obstacle_log->set_type(obstacle->type());
    obstacle_log->set_is_static(obstacle->is_static());
    obstacle_log->set_lat_decision(
        static_cast<uint32_t>(lat_obstacle_decision[obstacle->id()]));
    obstacle_log->set_vs_lat_relative(obstacle->frenet_velocity_l());
    obstacle_log->set_vs_lon_relative(obstacle->frenet_velocity_s());
    if (obstacle->source_type() == SourceType::GroundLine ||
        obstacle->source_type() == SourceType::OCC ||
        obstacle->source_type() == SourceType::OD ||
        obstacle->source_type() == SourceType::MAP) {
      for (const auto &polygon :
           obstacle->obstacle()->perception_polygon().points()) {
        planning::common::Point2d *obstacle_polygon =
            obstacle_log->add_polygon_points();
        obstacle_polygon->set_x(polygon.x());
        obstacle_polygon->set_y(polygon.y());
      }
    }
  }
#endif
}
}