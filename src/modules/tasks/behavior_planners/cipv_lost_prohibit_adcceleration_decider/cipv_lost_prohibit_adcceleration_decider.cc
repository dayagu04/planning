#include "cipv_lost_prohibit_adcceleration_decider.h"

#include <assert.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "agent/agent_manager.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "ifly_time.h"
#include "lateral_obstacle.h"
#include "planning_context.h"
#include "planning_hmi_c.h"
#include "session.h"

namespace planning {

namespace {
constexpr size_t kHistoryCipvSaveNum = 5;
constexpr double kCipvLostTtcThr = 5.0;
constexpr double kStopEgoHoldSpdMps = 0.5 / 3.6;
constexpr double kMinSpeedThr = 0.5;
}  // namespace

CipvLostProhibitAccelerationDecider::CipvLostProhibitAccelerationDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "CipvLostProhibitAccelerationDecider";
  session_ = session;
}

void CipvLostProhibitAccelerationDecider::Reset() {
  cipv_has_ = false;
  last_cipv_fn_ = false;
  prohibit_acc_ = false;
  counter_ = 0;
  speed_limit_ = std::numeric_limits<double>::max();
  start_time_ = -1.0;
  end_time_ = -1.0;
  duration_ = 0.0;
  pre_cipv_id_ = -1;
  pre_cipv_lost_id_ = -1;
  cipv_id_ = -1;
  pre_cipv_rel_s_ = std::numeric_limits<double>::max();
  pre_cipv_ttc_ = std::numeric_limits<double>::max();
  history_cipv_ids_.clear();
}

bool CipvLostProhibitAccelerationDecider::Execute() {
  LOG_DEBUG("=======CipvLostProhibitAccelerationDecider======= \n");

  const auto &environmental_model = session_->environmental_model();

  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  const auto &planning_init_point = ego_state_mgr->planning_init_point();
  const auto &dynamic_world = environmental_model.get_dynamic_world();
  const bool dbw_status = environmental_model.GetVehicleDbwStatus();
  const auto lateral_obstacle = environmental_model.get_lateral_obstacle();
  const auto &lateral_behavior_planner_output =
      session_->planning_context().lateral_behavior_planner_output();
  auto lc_status = lateral_behavior_planner_output.lc_status;

  // 1、读取当前cipv信息，包括是否存在cipv和cipv的id
  cipv_id_ = -1;
  if ((lc_status != "left_lane_change") && (lc_status != "right_lane_change")) {
    if (lateral_obstacle->leadone() != nullptr &&
        lateral_obstacle->leadone()->type != 0) {
      cipv_has_ = true;
      cipv_id_ = lateral_obstacle->leadone()->track_id;
    }
  } else {
    if (lateral_obstacle->tleadone() != nullptr &&
        lateral_obstacle->tleadone()->type != 0) {
      cipv_has_ = true;
      cipv_id_ = lateral_obstacle->tleadone()->track_id;
    }
  }

  if (cipv_id_ == -1) {
    cipv_has_ = false;
  }

  // 2、continous_flag来判断当前cipv的稳定性
  bool continous_flag = true;
  for (const auto &id : history_cipv_ids_) {
    if (id != cipv_id_) {
      continous_flag = false;
      break;
    }
  }

  // 3、判断：存在cipv，当前车速度大于kStopEgoHoldSpdMps，处在ACC或PILOT模式
  if (cipv_has_ && planning_init_point.v > kStopEgoHoldSpdMps && dbw_status) {
    const auto &agent_manager = dynamic_world->agent_manager();
    const planning::agent::Agent *cipv_ptr = agent_manager->GetAgent(cipv_id_);

    // 新cipv开始稳定稳定，并且ttc小于阈值，自车速度比它大
    if (pre_cipv_id_ > 0 && cipv_id_ == pre_cipv_id_ && continous_flag &&
        (pre_cipv_ttc_ < kCipvLostTtcThr ||
         planning_init_point.v > cipv_ptr->speed())) {
      pre_cipv_lost_id_ = cipv_id_;  //当cipv稳定时存储
      ++counter_;
      //当前cipv没变动
    } else if (pre_cipv_lost_id_ > 0 && cipv_id_ == pre_cipv_lost_id_) {
      ++counter_;
    } else {
      // pre_cipv_lost_id_ = -1;
      counter_ = 0;
    }
  } else {
    pre_cipv_lost_id_ = -1;
    counter_ = 0;
  }

  //设置禁止加速标志位，禁止10帧
  if (counter_ < 10 && counter_) {
    prohibit_acc_ = true;
  } else {
    prohibit_acc_ = false;
  }

  // 4、设置禁止加速的速度限制开始时间
  if (prohibit_acc_ && planning_init_point.v > kStopEgoHoldSpdMps) {
    //设置速度限制
    speed_limit_ = planning_init_point.v;
    start_time_ = IflyTime::Now_s();
  }

  if (prohibit_acc_) {
    //记录禁止了多长时间
    duration_ = IflyTime::Now_s() - start_time_;
  } else {
    duration_ = 0.0;
    start_time_ = -1;
    end_time_ = -1;
    speed_limit_ = std::numeric_limits<double>::max();
  }
  JSON_DEBUG_VALUE("prohibit_acc_", prohibit_acc_);
  // 5、更新历史cipv信息
  Update();
  auto &mutable_output =
      session_->mutable_planning_context()
          ->mutable_cipv_lost_prohibit_acceleration_decider_output();
  mutable_output.prohibit_acceleration_ = prohibit_acc_;
  mutable_output.speed_limit_ = speed_limit_;
  return true;
}

double CipvLostProhibitAccelerationDecider::CalculateRelativeDistance(
    const std::shared_ptr<KDPath> &planned_path, planning::agent::Agent cipv) {
  double project_dist = std::numeric_limits<double>::max();
  if (cipv.agent_id() <= 0) {
    return project_dist;
  }
  const auto &corners = cipv.box().GetAllCorners();
  for (const auto &corner : corners) {
    double project_s = 0.0;
    double project_l = 0.0;
    if (!planned_path->XYToSL(corner.x(), corner.y(), &project_s, &project_l)) {
      continue;
    }
    project_dist = std::fmin(project_dist, project_s);
  }
  const double front_edge_to_center = VehicleConfigurationContext::Instance()
                                          ->get_vehicle_param()
                                          .front_edge_to_rear_axle;
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();

  Point2D frenet_point, cart_point;
  cart_point.x = ego_state->ego_carte().x;
  cart_point.y = ego_state->ego_carte().y;
  if (planned_path->XYToSL(cart_point, frenet_point)) {
    auto ego_s = frenet_point.x;
    return std::fmax(0.0, project_dist - front_edge_to_center - ego_s);
  }
}
const planning::agent::Agent *CipvLostProhibitAccelerationDecider::QuerryCipv(
    std::shared_ptr<planning::planning_data::DynamicWorld> dynamic_world,
    const planning::agent::Agent *cipv) {
  const auto &agent_manager = dynamic_world->agent_manager();
  if (nullptr == agent_manager) {
    return nullptr;
  }
  const planning::agent::Agent *cipv_ptr = agent_manager->GetAgent(cipv_id_);
  if (nullptr == cipv_ptr) {
    return nullptr;
  } else {
    return cipv_ptr;
  }
}

void CipvLostProhibitAccelerationDecider::Update() {
  const planning::agent::Agent *cipv;
  const auto &dynamic_world =
      session_->environmental_model().get_dynamic_world();
  cipv = QuerryCipv(dynamic_world, cipv);

  if (cipv_has_) {
    const auto &environmental_model = session_->environmental_model();
    const auto ego_state_mgr = environmental_model.get_ego_state_manager();
    const auto &planning_init_point = ego_state_mgr->planning_init_point();

    const auto &current_lane =
        environmental_model.get_virtual_lane_manager()->get_current_lane();
    const auto planned_path =
        current_lane->get_reference_path()->get_frenet_coord();

    pre_cipv_rel_s_ = CalculateRelativeDistance(planned_path, *cipv);
    pre_cipv_ttc_ =
        pre_cipv_rel_s_ /
        std::fmax(kMinSpeedThr, planning_init_point.v - cipv->speed());
    pre_cipv_id_ = cipv->agent_id();
  } else {
    pre_cipv_rel_s_ = std::numeric_limits<double>::max();
    pre_cipv_ttc_ = std::numeric_limits<double>::max();
  }

  if (history_cipv_ids_.size() < kHistoryCipvSaveNum) {
    history_cipv_ids_.emplace_back(pre_cipv_id_);
  } else {
    for (size_t i = 0; i <= 3; ++i) {
      history_cipv_ids_[i] = history_cipv_ids_[i + 1];
    }
    history_cipv_ids_[4] = pre_cipv_id_;
  }
}
}  // namespace planning
