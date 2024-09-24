#include "cipv_lost_prohibit_acceleration_decider.h"

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
constexpr size_t kHistoryCipvSaveNum = 3;
constexpr double kCipvLostTtcThr = 5.0;
constexpr double kStopEgoHoldSpdMps = 0.5 / 3.6;
constexpr double kMinSpeedThr = 0.1;
constexpr size_t kProhibitCount = 5;
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
  const auto &ego_state_mgr = environmental_model.get_ego_state_manager();
  const auto &v_ego = ego_state_mgr->ego_v();
  const bool &dbw_status = environmental_model.GetVehicleDbwStatus();
  const auto &lateral_obstacle = environmental_model.get_lateral_obstacle();
  const auto &lateral_behavior_planner_output =
      session_->planning_context().lateral_behavior_planner_output();
  auto lc_status = lateral_behavior_planner_output.lc_status;

  // 1、读取当前cipv信息，包括是否存在cipv和cipv的id
  UpdateCipvInfo(lateral_obstacle, lc_status);

  // 2、continous_flag来判断当前cipv的稳定性
  bool continous_flag = true;
  for (const auto &id : history_cipv_ids_) {
    if (id != cipv_id_) {
      continous_flag = false;
      break;
    }
  }

  // 3、判断：存在cipv，当前车速度大于kStopEgoHoldSpdMps，进自动
  if (cipv_has_ && v_ego > kStopEgoHoldSpdMps && dbw_status) {
    // 判断cipv lost id是否为当前cipv id
    if (pre_cipv_lost_id_ > 0 && cipv_id_ == pre_cipv_lost_id_) {
      ++counter_;
    } else if (pre_cipv_id_ > 0 && cipv_id_ == pre_cipv_id_ && continous_flag &&
              (pre_cipv_ttc_ < kCipvLostTtcThr)) {
      // 更新cipv lost id，满足：
      // 1. 上一帧cipv id是否为当前cipv id
      // 2. cipv是否连续3帧稳定
      // 3. 自车与cipv的纵向ttc是否小于5s
      pre_cipv_lost_id_ = cipv_id_;
      ++counter_;
    } else {
      pre_cipv_lost_id_ = -1;
      counter_ = 0;
    }
  } else {
    pre_cipv_lost_id_ = -1;
    counter_ = 0;
  }

  // 设置禁止加速标志位，禁止10帧
  if (counter_ < kProhibitCount && counter_) {
    prohibit_acc_ = true;
  } else {
    prohibit_acc_ = false;
  }

  // 4、设置禁止加速的速度限制开始时间
  if (prohibit_acc_ && v_ego > kStopEgoHoldSpdMps) {
    // 设置速度限制：保持当前车速
    speed_limit_ = v_ego;
    start_time_ = IflyTime::Now_s();
  }

  if (prohibit_acc_) {
    // 记录禁止了多长时间
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
    const std::shared_ptr<KDPath> &planned_path, const agent::Agent *cipv) {
  double project_dist = std::numeric_limits<double>::max();
  if (cipv->agent_id() <= 0) {
    return project_dist;
  }

  if (planned_path == nullptr) {
    return project_dist;
  }

  const auto &corners = cipv->box().GetAllCorners();

  // 遍历每个角点，并计算投影距离
  for (const auto &corner : corners) {
    double project_s = 0.0;
    double project_l = 0.0;
    // 将每个角点从 XY 坐标转为 SL 坐标系
    if (planned_path->XYToSL(corner.x(), corner.y(), &project_s, &project_l)) {
      project_dist = std::fmin(project_dist, project_s);
    }
  }
  const double front_edge_to_center = VehicleConfigurationContext::Instance()
                                          ->get_vehicle_param()
                                          .front_edge_to_rear_axle;
  // 获取自车的状态
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();

  Point2D frenet_point, cart_point;
  cart_point.x = ego_state->ego_carte().x;
  cart_point.y = ego_state->ego_carte().y;
  // 检查是否成功转换为 SL 坐标，并计算相对距离
  if (planned_path->XYToSL(cart_point, frenet_point)) {
    const double ego_s = frenet_point.x;
    // 返回自车与 CIPV 的相对距离，确保距离为正值
    return std::fmax(0.0, project_dist - front_edge_to_center - ego_s);
  }

  // 如果转换失败，返回最大距离
  return std::numeric_limits<double>::max();
}

const agent::Agent *CipvLostProhibitAccelerationDecider::QuerryCipv(
    std::shared_ptr<agent::AgentManager> agent_manager) {
  if (nullptr == agent_manager) {
    return nullptr;
  }
  const auto *cipv_ptr = agent_manager->GetAgent(cipv_id_);
  if (nullptr == cipv_ptr) {
    return nullptr;
  } else {
    return cipv_ptr;
  }
}

void CipvLostProhibitAccelerationDecider::Update() {
  const auto &agent_manager =
      session_->environmental_model().get_agent_manager();
  const auto cipv = QuerryCipv(agent_manager);

  if (cipv != nullptr) {
    pre_cipv_id_ = cipv->agent_id();
    if (cipv_has_) {
      const auto &environmental_model = session_->environmental_model();
      const auto &ego_state_mgr = environmental_model.get_ego_state_manager();
      const auto &v_ego = ego_state_mgr->ego_v();

      const auto &current_lane =
          environmental_model.get_virtual_lane_manager()->get_current_lane();
      if (current_lane == nullptr) {
        pre_cipv_rel_s_ = std::numeric_limits<double>::max();
        pre_cipv_ttc_ = std::numeric_limits<double>::max();
        pre_cipv_id_ = -1;
        return;
      }
      const auto planned_path =
          current_lane->get_reference_path()->get_frenet_coord();

      pre_cipv_rel_s_ = CalculateRelativeDistance(planned_path, cipv);
      pre_cipv_ttc_ =
          pre_cipv_rel_s_ / std::fmax(kMinSpeedThr, v_ego);
    } else {
      pre_cipv_rel_s_ = std::numeric_limits<double>::max();
      pre_cipv_ttc_ = std::numeric_limits<double>::max();
    }
  } else {
    pre_cipv_rel_s_ = std::numeric_limits<double>::max();
    pre_cipv_ttc_ = std::numeric_limits<double>::max();
    pre_cipv_id_ = -1;
  }

  history_cipv_ids_.emplace_back(pre_cipv_id_);

  if (history_cipv_ids_.size() > kHistoryCipvSaveNum) {
    // 如果超出保存数量，移除第一个元素
    history_cipv_ids_.erase(history_cipv_ids_.begin());
  }
}

void CipvLostProhibitAccelerationDecider::UpdateCipvInfo(
    const std::shared_ptr<LateralObstacle> &lateral_obstacle,
    const string &lc_status) {
  // check lead one
  bool is_lead_one_vehicle = IsLeadVehicle(lateral_obstacle->leadone());
  // check lead two
  bool is_lead_two_vehicle = IsLeadVehicle(lateral_obstacle->leadtwo());
  // check temp lead one
  bool is_temp_lead_one_vehicle = IsLeadVehicle(lateral_obstacle->tleadone());
  // check temp lead two
  bool is_temp_lead_two_vehicle = IsLeadVehicle(lateral_obstacle->tleadtwo());

  cipv_id_ = -1;
  if ((lc_status != "left_lane_change") && (lc_status != "right_lane_change")) {
    if (is_lead_one_vehicle) {
      cipv_id_ = lateral_obstacle->leadone()->track_id;
      cipv_has_ = true;
    } else if (!is_lead_one_vehicle && is_lead_two_vehicle) {
      cipv_id_ = lateral_obstacle->leadtwo()->track_id;
      cipv_has_ = true;
    } else {
      cipv_has_ = false;
      cipv_id_ = -1;
    }
  } else {
    if (is_temp_lead_one_vehicle) {
      cipv_id_ = lateral_obstacle->tleadone()->track_id;
      cipv_has_ = true;
    } else if (!is_temp_lead_one_vehicle && is_temp_lead_two_vehicle) {
      cipv_id_ = lateral_obstacle->tleadtwo()->track_id;
      cipv_has_ = true;
    } else {
      cipv_has_ = false;
      cipv_id_ = -1;
    }
  }
}

bool CipvLostProhibitAccelerationDecider::IsLeadVehicle(
    const TrackedObject *lead) {
  if (lead != nullptr) {
    return lead->track_id != -1 && lead->is_lead && lead->is_car;
  } else {
    return false;
  }
}

}  // namespace planning