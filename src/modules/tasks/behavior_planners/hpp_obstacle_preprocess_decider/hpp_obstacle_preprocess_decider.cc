#include "hpp_obstacle_preprocess_decider.h"

#include <cmath>
#include <limits>
#include <unordered_map>

#include "common/agent/agent.h"
#include "common/agent/agent_manager.h"
#include "common/tracked_object.h"
#include "context/environmental_model.h"
#include "context/obstacle.h"
#include "context/reference_path.h"
#include "context/reference_path_manager.h"
#include "trajectory/trajectory.h"
#include "trajectory/trajectory_point.h"

namespace planning {

HppObstaclePreprocessDecider::HppObstaclePreprocessDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "HppObstaclePreprocessDecider";
}

bool HppObstaclePreprocessDecider::Execute() {
  ProcessGroundLines();
  ProcessDynamicObstacles();
  return true;
}

void HppObstaclePreprocessDecider::ProcessGroundLines() {
  const auto &reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  const auto &reference_path =
      reference_path_manager->get_reference_path_by_current_lane();

  if (reference_path == nullptr) {
    return;
  }

  const auto &ground_lines = reference_path->get_free_space_ground_lines();
  if (ground_lines.empty()) {
    return;
  }

  const auto &frenet_coord = reference_path->get_frenet_coord();
  const double ego_s = reference_path->get_frenet_ego_state().s();

  // 使用较大的ID段来避免与其他障碍物ID冲突
  int virtual_agent_id = 5500000;

  for (const auto *obstacle : ground_lines) {
    if (obstacle == nullptr) {
      continue;
    }

    // 遍历地面线上的点，找到在自车前方且最近的路径内碰撞点
    double min_s = std::numeric_limits<double>::max();
    double collision_l = 0.0;
    bool collision_found = false;

    // 横向检测范围（约半车道宽度）
    const double kLateralCheckRange = 1.5;

    for (const auto &point : obstacle->perception_points()) {
      Point2D cart_point(point.x(), point.y());
      Point2D frenet_point;
      if (frenet_coord->XYToSL(cart_point, frenet_point)) {
        if (std::abs(frenet_point.y) < kLateralCheckRange &&
            frenet_point.x > ego_s) {
          if (frenet_point.x < min_s) {
            min_s = frenet_point.x;
            collision_l = frenet_point.y;
            collision_found = true;
          }
        }
      }
    }

    if (collision_found) {
      CreateVirtualAgentFromGroundLine(obstacle, min_s, collision_l,
                                       virtual_agent_id++);
    }
  }
}

void HppObstaclePreprocessDecider::CreateVirtualAgentFromGroundLine(
    const Obstacle *obstacle, double s, double l, int id) {
  const auto &reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  const auto &reference_path =
      reference_path_manager->get_reference_path_by_current_lane();
  if (reference_path == nullptr) return;

  const auto &frenet_coord = reference_path->get_frenet_coord();

  // Frenet (s,l) -> 笛卡尔坐标
  Point2D frenet_point(s, l);
  Point2D cart_point;
  if (!frenet_coord->SLToXY(frenet_point, cart_point)) {
    return;
  }

  // 获取该s处的参考路径朝向
  double heading = frenet_coord->GetPathPointByS(s).theta();

  agent::Agent virtual_agent;
  virtual_agent.set_agent_id(id);
  virtual_agent.set_type(agent::AgentType::VIRTUAL);
  virtual_agent.set_is_static(true);
  virtual_agent.set_x(cart_point.x);
  virtual_agent.set_y(cart_point.y);
  virtual_agent.set_theta(heading);
  virtual_agent.set_length(kGroundLineVirtualAgentLength);
  virtual_agent.set_width(kGroundLineVirtualAgentWidth);
  virtual_agent.set_speed(0.0);
  virtual_agent.set_accel(0.0);

  // 必须设置fusion_source为OBSTACLE_SOURCE_CAMERA，否则StGraphInput会过滤掉该agent
  virtual_agent.set_fusion_source(OBSTACLE_SOURCE_CAMERA);

  // 设置包围盒
  planning_math::Box2d box({cart_point.x, cart_point.y}, heading,
                            kGroundLineVirtualAgentLength,
                            kGroundLineVirtualAgentWidth);
  virtual_agent.set_box(box);

  planning_math::Polygon2d polygon(box);
  virtual_agent.set_polygon(polygon);

  // 时间范围从0开始（与StGraphInput的相对时间域对齐）
  virtual_agent.set_time_range({0.0, kPredictionHorizon});
  virtual_agent.set_timestamp_s(0.0);

  auto &agent_manager =
      session_->mutable_environmental_model()->mutable_agent_manager();
  std::unordered_map<int32_t, agent::Agent> agent_table;
  agent_table.insert({virtual_agent.agent_id(), virtual_agent});
  agent_manager->Append(agent_table);

  ILOG_INFO << "Created ground line virtual agent id=" << id
            << " at s=" << s << " l=" << l;
}

void HppObstaclePreprocessDecider::ProcessDynamicObstacles() {
  auto &agent_manager =
      session_->mutable_environmental_model()->mutable_agent_manager();

  const auto &agent_ids = agent_manager->GetAgentSet();

  for (int32_t id : agent_ids) {
    auto *agent = agent_manager->mutable_agent(id);
    if (agent == nullptr) continue;

    // 对于被AgentManager错误标记为静态、但实际速度较大、且无预测轨迹的障碍物，
    // 修正is_static标志并补充匀速预测轨迹，使ST图能正确处理
    if (agent->is_static() && agent->speed() > kDynamicObstacleSpeedThreshold &&
        agent->trajectories_used_by_st_graph().empty()) {
      agent->set_is_static(false);
      GenerateConstantVelocityTrajectory(agent);
      ILOG_INFO << "Fixed dynamic agent id=" << id
                << " speed=" << agent->speed()
                << " as dynamic with CV trajectory";
    }
  }
}

void HppObstaclePreprocessDecider::GenerateConstantVelocityTrajectory(
    agent::Agent *agent) {
  double x = agent->x();
  double y = agent->y();
  double theta = agent->theta();
  double v = agent->speed();

  trajectory::Trajectory traj;
  for (double t = 0.0; t <= kPredictionHorizon + 1e-6; t += kTimeResolution) {
    double dist = v * t;
    double nx = x + dist * std::cos(theta);
    double ny = y + dist * std::sin(theta);

    trajectory::TrajectoryPoint tp;
    tp.set_x(nx);
    tp.set_y(ny);
    tp.set_theta(theta);
    tp.set_vel(v);
    tp.set_acc(0.0);
    tp.set_s(dist);
    // absolute_time在Agent轨迹中存储的是相对时间（与Agent构造逻辑保持一致），
    // StGraphInput将planning_init_point_.absolute_time设为0，因此从0开始
    tp.set_absolute_time(t);

    traj.emplace_back(tp);
  }

  std::vector<trajectory::Trajectory> trajectories;
  trajectories.push_back(std::move(traj));
  agent->set_trajectories_used_by_st_graph(trajectories);
}

}  // namespace planning
