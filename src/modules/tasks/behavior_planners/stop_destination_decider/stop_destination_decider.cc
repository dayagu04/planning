#include "stop_destination_decider.h"

#include <math.h>

#include "debug_info_log.h"
#include "log.h"
#include "planning_context.h"
#include "task.h"

namespace planning {
namespace {
constexpr double kRADSCollisionCheckSrefRecoverCounter = 5;
constexpr double kRADSCollisionCheckSrefSetLowThred = 0.1;
constexpr double kRADSCollisionCheckSrefSetHighThred = 0.25;

}  // namespace

StopDestinationDecider::StopDestinationDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session),
      config_(config_builder->cast<StopDestinationDeciderConfig>()),
      stop_destination_virtual_agent_time_headway_(
          config_.stop_destination_virtual_agent_time_headway) {
  lon_collision_checker_ = std::make_shared<CollisionChecker>();
  name_ = "StopDestinationDecider";
}

bool StopDestinationDecider::Execute() {
  if (!PreCheck()) {
    ILOG_ERROR << "PreCheck failed";
    return false;
  }
  rads_bound_s_by_collision_check_.assign(26, 200.0);
  const auto function_mode =
      session_->environmental_model().function_info().function_mode();
  if (function_mode == common::DrivingFunctionInfo::RADS) {
    StopDestinationProcess();
  }

  return true;
}

void StopDestinationDecider::StopDestinationProcess() {
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  current_lane_ = virtual_lane_manager->get_current_lane();

  if (session_->is_rads_scene()) {
    std::vector<planning_math::CollisionCheckStatus> collision_results;
    auto &ego_planning_result =
      session_->mutable_planning_context()->mutable_planning_result();
    auto &ego_model = lon_collision_checker_->get_ego_model();
    ego_model->set_model_type(EgoModelType::ORIGIN);
    const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
    double deviation_length = vehicle_param.rear_axle_to_center;
    double lon_expansion = 0.0;
    ego_model->set_expansion(0.0, lon_expansion);
    lon_collision_checker_->set_params(deviation_length);

    double collision_threshold = 0.05;
    auto reference_path_ptr = session_->environmental_model()
                            .get_reference_path_manager()
                            ->get_reference_path_by_current_lane();
    auto &obstacles = reference_path_ptr->get_obstacles();
    int16_t traj_point_index = 0;
    collision_results.clear();
    for (auto &traj_pt : ego_planning_result.traj_points) {
      auto obstacle_pos_time = traj_pt.t;
      Pose2D traj_check_point;
      traj_check_point.x = traj_pt.x;
      traj_check_point.y = traj_pt.y;
      traj_check_point.theta = traj_pt.heading_angle;
      lon_collision_checker_->set_point(traj_check_point);
      if (!obstacles.empty()) {
        for (auto &obstacle : obstacles) {
          if (nullptr == obstacle) {
            continue;
          }
          CollisionCheckStatus result_tmp;
          result_tmp.is_collision = false;
          //double obstacle_pos_time_on_ego_side = 100.0;

          auto l_center = obstacle->frenet_l();
          double kLatIgnoreDistance = 10.0;
          if (std::fabs(l_center) > kLatIgnoreDistance) {
            continue;
          }
          /* if (check_obstacle_both_sides(obstacle)) {
            obstacle_pos_time_on_ego_side = 0.0;
          } */
          // only check 0s when obj is at the side of ego
          //obstacle_pos_time =
          //    std::min(obstacle_pos_time, obstacle_pos_time_on_ego_side);
          auto obstacle_point =
              obstacle->obstacle()->get_point_at_time(obstacle_pos_time);
          const planning_math::Polygon2d &polygon =
              obstacle->obstacle()->get_polygon_at_point(obstacle_point);
          result_tmp = lon_collision_checker_->collision_check(
              polygon, collision_threshold,
              CollisionCheckStatus::CollisionType::AGENT_COLLISION);
          result_tmp.point_index = traj_point_index;
          if (result_tmp.is_collision) {
            result_tmp.obstacle_id = obstacle->id();
            collision_results.emplace_back(result_tmp);
            // NLOGE(
            //     "The trajectory has collision with obstacle %i, the distance "
            //     "is: %f",
            //     obstacle.obstacle()->id(), result_tmp.min_distance);
            // NLOGE("The collision point x: %f, y: %f, theta: %f, index: %i",
            //       result_tmp.ego_poit.x, result_tmp.ego_poit.y,
            //       result_tmp.ego_poit.theta, result_tmp.point_index);
          }
        }
      }
      traj_point_index++;
    }
    int size_collision_res = collision_results.size();
    double collision_point_s = std::numeric_limits<double>::max();
    double collision_bound = std::numeric_limits<double>::max();
    int collision_obstacle_id = 0;
    int collision_position_index = std::numeric_limits<int>::max();
    auto collison_type = CollisionCheckStatus::CollisionType::NONE_COLLISION;
    double ego_init_frenet_s = reference_path_ptr->get_frenet_ego_state().s();
    if (!ego_planning_result.traj_points.empty()){
      ego_init_frenet_s = ego_planning_result.traj_points[0].s;
    }
    double safe_distance_collision = 0.3;
    double collision_TTC = 1.0;
    double distance_safe_buffer = 0.0;
    double ego_velocity =
        reference_path_ptr->get_frenet_ego_state().velocity_s();
    distance_safe_buffer =
        safe_distance_collision + ego_velocity * collision_TTC;

    for (auto collision_result : collision_results) {
      Point2D pt_catresian, pt_frenet;
      pt_catresian.x = collision_result.ego_point.x;
      pt_catresian.y = collision_result.ego_point.y;
      if (!reference_path_ptr->get_frenet_coord()->XYToSL(pt_catresian, pt_frenet)) {
        continue;
      }
      collision_point_s = pt_frenet.x;
      collision_position_index = collision_result.point_index;

      if (static_cast<size_t>(collision_position_index) >= 26) {
        continue;
      }

      double one_collision_bound = std::fmax(collision_point_s - distance_safe_buffer - ego_init_frenet_s, 0.0);
      /* if (one_collision_bound < collision_bound) {
        collision_bound = one_collision_bound;
        collision_obstacle_id = collision_result.obstacle_id;
      } */
      if (one_collision_bound < rads_bound_s_by_collision_check_[collision_position_index]) {
        rads_bound_s_by_collision_check_[collision_position_index] = one_collision_bound;
      }

    }
    double min_s_bound_val = 1000.0;
    auto min_it =
      std::min_element(rads_bound_s_by_collision_check_.begin(), rads_bound_s_by_collision_check_.end());
    min_s_bound_val = *min_it;
    std::fill(rads_bound_s_by_collision_check_.begin(), rads_bound_s_by_collision_check_.end(),
        min_s_bound_val);
    bool sref_set = min_s_bound_val < kRADSCollisionCheckSrefSetLowThred;
    bool sref_recover = min_s_bound_val > kRADSCollisionCheckSrefSetHighThred;
    if (sref_set) {
      rads_collision_check_sref_set_flag_ = sref_set;
      rads_collision_check_sref_recover_counter_ = 0;
    } else {
      if (rads_collision_check_sref_set_flag_ &&
          rads_collision_check_sref_recover_counter_ < kRADSCollisionCheckSrefRecoverCounter) {
            if (sref_recover) {
              rads_collision_check_sref_recover_counter_++;
            } else {
              rads_collision_check_sref_recover_counter_ = 0;
            }
      } else {
        rads_collision_check_sref_set_flag_ = false;
      }
    }
  }
  AddVirtualObstacle();
  SaveToSession();
}

// set virtual obstacle at the end of the lane
bool StopDestinationDecider::AddVirtualObstacle() {
  const auto &current_reference_path =
      current_lane_->get_reference_path();
  agent::Agent virtual_agent;
  if (current_reference_path == nullptr) {
    virtual_agent.set_agent_id(agent::AgentDefaultInfo::kNoAgentId);
    return false;
  }
  const auto& current_raw_end_point =
      current_reference_path->GetRawEndRefPathPoint();
  static double stop_destination_extended_s_buffer =
      config_.stop_destination_extended_s_buffer;
  stop_destination_virtual_agent_id_ =
      agent::AgentDefaultInfo::kRadsStopDestinationVirtualAgentId;
  virtual_agent.set_agent_id(stop_destination_virtual_agent_id_);
  virtual_agent.set_type(agent::AgentType::VIRTUAL);
  virtual_agent.set_is_tfl_virtual_obs(false);
  virtual_agent.set_is_stop_destination_virtual_obs(true);
  virtual_agent.set_x(current_raw_end_point.path_point.x() +
                      stop_destination_extended_s_buffer *
                          cos(current_raw_end_point.path_point.theta()));
  virtual_agent.set_y(current_raw_end_point.path_point.y() +
                      stop_destination_extended_s_buffer *
                          sin(current_raw_end_point.path_point.theta()));
  virtual_agent.set_length(0.5);
  virtual_agent.set_width(2.0);
  virtual_agent.set_fusion_source(1);
  virtual_agent.set_is_static(true);
  virtual_agent.set_speed(0.0);
  virtual_agent.set_theta(current_raw_end_point.path_point.theta());
  virtual_agent.set_accel(0.0);
  virtual_agent.set_time_range({0.0, 5.0});

  planning::planning_math::Box2d box(
      planning::planning_math::Vec2d(virtual_agent.x(), virtual_agent.y()),
      virtual_agent.theta(), virtual_agent.length(), virtual_agent.width());
  virtual_agent.set_box(box);
  virtual_agent.set_timestamp_s(0.0);
  virtual_agent.set_timestamp_us(0.0);
  auto *agent_manager = session_->environmental_model()
                            .get_dynamic_world()
                            ->mutable_agent_manager();
  std::unordered_map<int32_t, planning::agent::Agent> agent_table;
  agent_table.insert({virtual_agent.agent_id(), virtual_agent});
  agent_manager->Append(agent_table);
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_pos_x", virtual_agent.x())
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_pos_y", virtual_agent.y())
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_theta",
                   virtual_agent.theta())
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_id",
                   virtual_agent.agent_id())
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_width",
                   virtual_agent.width())
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_length",
                   virtual_agent.length())
  return true;
}

void StopDestinationDecider::SaveToSession() {
  auto &mutable_stop_destination_decider_output =
      session_->mutable_planning_context()
          ->mutable_stop_destination_decider_output();
  mutable_stop_destination_decider_output
      .mutable_stop_destination_virtual_agent_id() =
      stop_destination_virtual_agent_id_;
  mutable_stop_destination_decider_output
      .mutable_stop_destination_virtual_agent_time_headway() =
      stop_destination_virtual_agent_time_headway_;
  mutable_stop_destination_decider_output.update_rads_bound_s_by_collision_check(
                                        rads_bound_s_by_collision_check_);
  mutable_stop_destination_decider_output.mutable_rads_collision_check_sref_set_flag() =
      rads_collision_check_sref_set_flag_;
}

}  // namespace planning
