#include "stop_destination_decider.h"

#include <algorithm>
#include <math.h>

#include "debug_info_log.h"
#include "define/geometry.h"
#include "environmental_model.h"
#include "hpp_route_target_info_util.h"
#include "planning_context.h"
#include "reference_path.h"
#include "task.h"

namespace planning {
namespace {

bool CalculateHppDestinationPoint(framework::Session *session,
                                  const std::shared_ptr<ReferencePath> &reference_path,
                                  double hpp_stop_distance_to_destination,
                                  planning_math::PathPoint *destination_point) {
  if (session == nullptr || reference_path == nullptr || destination_point == nullptr) {
    return false;
  }
  const auto &route_info_output =
      session->environmental_model().get_route_info()->get_route_info_output();
  const auto &planning_init_point = reference_path->get_frenet_ego_state().planning_init_point();
  double distance_to_destination =
      route_info_output.hpp_route_info_output.distance_to_target_dest;
  const auto &parking_slot_manager =
      session->environmental_model().get_parking_slot_manager();
  const auto &frenet_coord = reference_path->get_frenet_coord();
  const auto &current_state = session->environmental_model()
                                  .get_local_view()
                                  .function_state_machine_info.current_state;
  // Keep old HPP destination logic for stop point.
  double stop_distance_to_destination = hpp_stop_distance_to_destination;

  if (current_state == iflyauto::FunctionalState_HPP_CRUISE_ROUTING) {
    if (parking_slot_manager->IsExistTargetSlot()) {
      const auto &target_slot_center = parking_slot_manager->GetTargetSlotCenter();
      Point2D frenet_point;
      if (frenet_coord != nullptr &&
          frenet_coord->XYToSL(
              Point2D(target_slot_center.x(), target_slot_center.y()),
              frenet_point)) {
        distance_to_destination =
            frenet_point.x - planning_init_point.frenet_state.s;
      }
    }
  } else if (current_state == iflyauto::FunctionalState_HPP_CRUISE_SEARCHING) {
    bool find_slot = parking_slot_manager->CalculateDistanceToNearestSlot(
        reference_path);
    if (find_slot) {
      stop_distance_to_destination = 0.0;
      distance_to_destination = parking_slot_manager->GetDistanceToNearestSlot();
    }
  }

  const double destination_s =
      planning_init_point.frenet_state.s +
      std::max((distance_to_destination - stop_distance_to_destination), 0.0);
  ReferencePathPoint destination_ref_point;
  if (!reference_path->get_reference_point_by_lon(destination_s, destination_ref_point)) {
    return false;
  }
  *destination_point = destination_ref_point.path_point;
  return true;
}
constexpr double kRADSCollisionCheckSrefRecoverCounter = 5;
constexpr double kRADSCollisionCheckSrefSetLowThred = 0.1;
constexpr double kRADSCollisionCheckSrefSetHighThred = 0.25;

}  // namespace

StopDestinationDecider::StopDestinationDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session),
      config_(config_builder->cast<StopDestinationDeciderConfig>()),
      hpp_stop_distance_to_destination_(
          config_builder->cast<LongitudinalDeciderV3Config>()
              .stop_distance_to_destination),
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
  if (session_->is_rads_scene() || session_->is_hpp_scene()) {
    StopDestinationProcess();
  }

  return true;
}

void StopDestinationDecider::StopDestinationProcess() {
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  current_lane_ = virtual_lane_manager->get_current_lane();

  // HPP：先统一按当前 reference path 更新 distance_to_target_dest/slot，
  // 再计算虚拟终点位置，供本 decider 与后续 StartStopDecider 共用
  if (session_->is_hpp_scene() && current_lane_ != nullptr) {
    const auto &current_reference_path = current_lane_->get_reference_path();
    if (current_reference_path != nullptr) {
      UpdateHppRouteTargetInfoFromReferencePath(session_, current_reference_path);
    }
  }

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
  if (current_lane_ == nullptr) {
    stop_destination_virtual_agent_id_ = agent::AgentDefaultInfo::kNoAgentId;
    return false;
  }
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
  // RADS（倒车）和HPP（前进）分别使用独立的虚拟障碍物ID，便于区分业务场景
  if (session_->is_hpp_scene()) {
    stop_destination_virtual_agent_id_ =
        agent::AgentDefaultInfo::kHppStopDestinationVirtualAgentId;
  } else {
    stop_destination_virtual_agent_id_ =
        agent::AgentDefaultInfo::kRadsStopDestinationVirtualAgentId;
  }
  virtual_agent.set_agent_id(stop_destination_virtual_agent_id_);
  virtual_agent.set_type(agent::AgentType::VIRTUAL);
  virtual_agent.set_is_tfl_virtual_obs(false);
  virtual_agent.set_is_stop_destination_virtual_obs(true);
  planning_math::PathPoint stop_point = current_raw_end_point.path_point;
  if (session_->is_hpp_scene()) {
    // HPP终点位置使用原HPP停车逻辑（参考general_longitudinal_decider的termination逻辑）
    if (!CalculateHppDestinationPoint(session_, current_reference_path,
                                      hpp_stop_distance_to_destination_,
                                      &stop_point)) {
      ILOG_WARN << "StopDestinationDecider: fallback to raw end point for HPP";
      stop_destination_virtual_agent_id_ = agent::AgentDefaultInfo::kNoAgentId;
      return false;
    }
  }
  // 终点虚拟障碍物沿路径航向延长（HPP/非HPP统一处理）
  stop_point.set_x(stop_point.x() + stop_destination_extended_s_buffer *
                                      cos(stop_point.theta()));
  stop_point.set_y(stop_point.y() + stop_destination_extended_s_buffer *
                                      sin(stop_point.theta()));
  virtual_agent.set_x(stop_point.x());
  virtual_agent.set_y(stop_point.y());
  virtual_agent.set_length(0.5);
  virtual_agent.set_width(2.0);
  virtual_agent.set_fusion_source(1);
  virtual_agent.set_is_static(true);
  virtual_agent.set_speed(0.0);
  virtual_agent.set_theta(stop_point.theta());
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
