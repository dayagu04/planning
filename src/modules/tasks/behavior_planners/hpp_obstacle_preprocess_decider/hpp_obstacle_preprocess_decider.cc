#include "hpp_obstacle_preprocess_decider.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

#include "common/agent/agent.h"
#include "common/agent/agent_manager.h"
#include "common/config/basic_type.h"
#include "common/constraint_check/collision_checker.h"
#include "common/tracked_object.h"
#include "context/environmental_model.h"
#include "context/obstacle.h"
#include "context/planning_context.h"
#include "context/reference_path.h"
#include "context/reference_path_manager.h"
#include "math/line_segment2d.h"
#include "trajectory/trajectory.h"
#include "trajectory/trajectory_point.h"
#include "vehicle_config_context.h"

namespace planning {
namespace {

constexpr double kCoarseCollisionCheckRange = 20.0;
constexpr int32_t kCollisionPointThreshold = 3;
constexpr double kHitPointDedupDistance = 0.05;
constexpr double kHitSegmentSplitDistance = 0.6;
constexpr double kAgentSProximityThreshold = 0.8;

struct GroundLineFirstCollision {
  int traj_point_index = -1;
  double collision_s_on_traj = std::numeric_limits<double>::max();
  std::vector<planning_math::Vec2d> collision_points;

  bool IsValid() const {
    return traj_point_index >= 0 && !collision_points.empty();
  }
};

bool IsPointWithinRange(const Pose2D &traj_check_point,
                        const planning_math::Vec2d &point,
                        double check_range) {
  const double dx = traj_check_point.x - point.x();
  const double dy = traj_check_point.y - point.y();
  return dx * dx + dy * dy <= check_range * check_range;
}

bool IsGroundLineFrenetObstacleValid(
    const std::shared_ptr<FrenetObstacle> &frenet_obstacle) {
  return frenet_obstacle != nullptr && frenet_obstacle->b_frenet_valid() &&
         frenet_obstacle->source_type() == SourceType::GroundLine;
}

std::unordered_map<int, const Obstacle *> BuildGroundLineObstacleMap(
    const std::vector<const Obstacle *> &ground_lines) {
  std::unordered_map<int, const Obstacle *> groundline_obstacle_map;
  groundline_obstacle_map.reserve(ground_lines.size());
  for (const auto *groundline : ground_lines) {
    if (groundline == nullptr) {
      continue;
    }
    groundline_obstacle_map[groundline->id()] = groundline;
  }
  return groundline_obstacle_map;
}

GroundLineFirstCollision FindEarliestGroundLineCollisionOnTrajectory(
    const Obstacle &groundline_obstacle, const TrajectoryPoints &traj_points,
    planning_math::CollisionChecker *collision_checker,
    double collision_threshold) {
  GroundLineFirstCollision earliest_collision;

  for (size_t i = 0; i < traj_points.size(); ++i) {
    const auto &traj_pt = traj_points[i];
    if (!traj_pt.frenet_valid) {
      continue;
    }

    // 1. 将当前轨迹点设置为碰撞检测时的自车位姿。
    Pose2D ego_pose;
    ego_pose.x = traj_pt.x;
    ego_pose.y = traj_pt.y;
    ego_pose.theta = traj_pt.heading_angle;
    collision_checker->set_point(ego_pose);

    // 2. 遍历 groundline 点云，统计当前轨迹点对应的有效碰撞点。
    int32_t collision_point_count = 0;
    std::vector<planning_math::Vec2d> collision_points;
    const auto &groundline_points = groundline_obstacle.perception_points();
    for (const auto &groundline_point : groundline_points) {
      if (!IsPointWithinRange(ego_pose, groundline_point,
                              kCoarseCollisionCheckRange)) {
        continue;
      }

      const auto collision_result = collision_checker->collision_check(
          groundline_point, collision_threshold,
          planning_math::CollisionCheckStatus::CollisionType::GROUNDLINE_COLLISION);
      if (!collision_result.is_collision) {
        continue;
      }

      ++collision_point_count;
      collision_points.push_back(groundline_point);
    }

    // 3. 只保留沿轨迹最早出现的有效碰撞结果。
    if (collision_point_count <= kCollisionPointThreshold ||
        traj_pt.s >= earliest_collision.collision_s_on_traj) {
      continue;
    }

    earliest_collision.collision_s_on_traj = traj_pt.s;
    earliest_collision.traj_point_index = static_cast<int>(i);
    earliest_collision.collision_points = std::move(collision_points);
  }

  return earliest_collision;
}

bool HasNearbyGeneratedAgent(const std::vector<double> &generated_agent_s,
                             double target_s) {
  for (const double existing_s : generated_agent_s) {
    if (std::fabs(existing_s - target_s) < kAgentSProximityThreshold) {
      return true;
    }
  }
  return false;
}

planning_math::CollisionChecker CreateGroundLineCollisionChecker() {
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();

  planning_math::CollisionChecker collision_checker;
  auto &ego_model = collision_checker.get_ego_model();
  ego_model->set_model_type(planning_math::EgoModelType::ORIGIN);
  ego_model->set_expansion(0.0, 0.0);
  collision_checker.set_params(vehicle_param.rear_axle_to_center);
  return collision_checker;
}

std::vector<planning_math::Vec2d> DeduplicateHitPoints(
    const std::vector<planning_math::Vec2d> &input) {
  std::vector<planning_math::Vec2d> dedup_points;
  dedup_points.reserve(input.size());
  for (const auto &point : input) {
    bool exists = false;
    for (const auto &saved_point : dedup_points) {
      const double dx = point.x() - saved_point.x();
      const double dy = point.y() - saved_point.y();
      if (dx * dx + dy * dy <=
          kHitPointDedupDistance * kHitPointDedupDistance) {
        exists = true;
        break;
      }
    }
    if (!exists) {
      dedup_points.push_back(point);
    }
  }
  return dedup_points;
}

std::vector<std::vector<planning_math::Vec2d>> SplitHitPointsToSegments(
    const std::vector<planning_math::Vec2d> &ordered_points) {
  std::vector<std::vector<planning_math::Vec2d>> segments;
  if (ordered_points.empty()) {
    return segments;
  }
  std::vector<planning_math::Vec2d> current_segment;
  current_segment.push_back(ordered_points.front());
  for (size_t i = 1; i < ordered_points.size(); ++i) {
    const double dx = ordered_points[i].x() - ordered_points[i - 1].x();
    const double dy = ordered_points[i].y() - ordered_points[i - 1].y();
    const double dist = std::hypot(dx, dy);
    if (dist > kHitSegmentSplitDistance && !current_segment.empty()) {
      segments.push_back(current_segment);
      current_segment.clear();
    }
    current_segment.push_back(ordered_points[i]);
  }
  if (!current_segment.empty()) {
    segments.push_back(current_segment);
  }
  return segments;
}

std::vector<planning_math::Vec2d> SelectMainHitSegment(
    const std::vector<std::vector<planning_math::Vec2d>> &segments,
    const TrajectoryPoint &anchor_traj_point) {
  if (segments.empty()) {
    return {};
  }
  double best_dist_sq = std::numeric_limits<double>::max();
  size_t best_index = 0;
  for (size_t i = 0; i < segments.size(); ++i) {
    if (segments[i].empty()) {
      continue;
    }
    for (const auto &point : segments[i]) {
      const double dx = point.x() - anchor_traj_point.x;
      const double dy = point.y() - anchor_traj_point.y;
      const double dist_sq = dx * dx + dy * dy;
      if (dist_sq < best_dist_sq) {
        best_dist_sq = dist_sq;
        best_index = i;
      }
    }
  }
  return segments[best_index];
}

std::pair<planning_math::Vec2d, planning_math::Vec2d> FindFarthestPoints(
    const std::vector<planning_math::Vec2d> &points) {
  if (points.size() < 2) {
    return {planning_math::Vec2d(), planning_math::Vec2d()};
  }
  double max_dist_sq = -1.0;
  std::pair<planning_math::Vec2d, planning_math::Vec2d> farthest_pair{points[0],
                                                                      points[1]};
  for (size_t i = 0; i < points.size(); ++i) {
    for (size_t j = i + 1; j < points.size(); ++j) {
      const double dx = points[i].x() - points[j].x();
      const double dy = points[i].y() - points[j].y();
      const double dist_sq = dx * dx + dy * dy;
      if (dist_sq > max_dist_sq) {
        max_dist_sq = dist_sq;
        farthest_pair = {points[i], points[j]};
      }
    }
  }
  return farthest_pair;
}

}  // namespace

HppObstaclePreprocessDecider::HppObstaclePreprocessDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "HppObstaclePreprocessDecider";
  lon_config_ = config_builder->cast<LongitudinalDeciderV3Config>();
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

  const auto &traj_points =
      session_->planning_context().planning_result().traj_points;
  if (traj_points.empty()) {
    return;
  }

  const auto &frenet_obstacles = reference_path->get_obstacles();
  if (frenet_obstacles.empty()) {
    return;
  }

  const auto groundline_obstacle_map = BuildGroundLineObstacleMap(ground_lines);
  if (groundline_obstacle_map.empty()) {
    return;
  }

  const double collision_threshold = lon_config_.hpp_collision_threshold;
  auto collision_checker = CreateGroundLineCollisionChecker();

  std::vector<double> generated_virtual_agent_s;
  generated_virtual_agent_s.reserve(ground_lines.size());

  for (const auto &frenet_obstacle : frenet_obstacles) {
    // 1. 仅处理有效的 groundline frenet 障碍物。
    if (!IsGroundLineFrenetObstacleValid(frenet_obstacle)) {
      continue;
    }

    // 2. 根据障碍物 id 找到对应的 groundline 几何信息。
    auto obstacle_it = groundline_obstacle_map.find(frenet_obstacle->id());
    if (obstacle_it == groundline_obstacle_map.end() ||
        obstacle_it->second == nullptr) {
      continue;
    }

    // 3. 沿规划轨迹搜索该 groundline 的最早有效碰撞位置。
    const auto earliest_collision = FindEarliestGroundLineCollisionOnTrajectory(
        *obstacle_it->second, traj_points, &collision_checker,
        collision_threshold);
    if (!earliest_collision.IsValid()) {
      continue;
    }

    // 4. 避免在相近 s 位置重复生成 virtual agent。
    if (HasNearbyGeneratedAgent(generated_virtual_agent_s,
                                earliest_collision.collision_s_on_traj)) {
      continue;
    }

    // 5. 基于碰撞点生成 virtual agent，直接复用对应 groundline 的 obstacle id。
    CreateVirtualAgentFromGroundLine(
        earliest_collision.collision_points,
        traj_points[earliest_collision.traj_point_index],
        frenet_obstacle->id());
    generated_virtual_agent_s.push_back(earliest_collision.collision_s_on_traj);
  }
}


void HppObstaclePreprocessDecider::CreateVirtualAgentFromGroundLine(
    const std::vector<planning_math::Vec2d> &hit_points,
    const TrajectoryPoint &anchor_traj_point, int id) {
  if (hit_points.empty()) {
    return;
  }

  agent::Agent virtual_agent;
  virtual_agent.set_agent_id(id);
  virtual_agent.set_type(agent::AgentType::VIRTUAL);
  virtual_agent.set_is_static(true);
  virtual_agent.set_x(anchor_traj_point.x);
  virtual_agent.set_y(anchor_traj_point.y);
  virtual_agent.set_theta(anchor_traj_point.heading_angle);
  virtual_agent.set_length(kGroundLineVirtualAgentLength);
  virtual_agent.set_width(kGroundLineVirtualAgentWidth);
  virtual_agent.set_speed(0.0);
  virtual_agent.set_accel(0.0);
  virtual_agent.set_fusion_source(OBSTACLE_SOURCE_CAMERA);

  // 1. 对碰撞点做去重和分段，优先保留离锚点轨迹最近的主碰撞段。
  const auto unique_collision_points = DeduplicateHitPoints(hit_points);
  const auto collision_segments =
      SplitHitPointsToSegments(unique_collision_points);
  const auto main_collision_segment =
      SelectMainHitSegment(collision_segments, anchor_traj_point);
  const auto &shape_points =
      main_collision_segment.empty() ? unique_collision_points
                                     : main_collision_segment;

  // 2. 根据碰撞点拟合 virtual agent 的包围盒和多边形。
  planning_math::Polygon2d virtual_agent_polygon;
  planning_math::Box2d virtual_agent_box(
      {anchor_traj_point.x, anchor_traj_point.y},
      anchor_traj_point.heading_angle, kGroundLineVirtualAgentLength,
      kGroundLineVirtualAgentWidth);

  const bool can_build_convex_hull =
      shape_points.size() >= 3 &&
      planning_math::Polygon2d::ComputeConvexHull(shape_points,
                                                  &virtual_agent_polygon) &&
      virtual_agent_polygon.is_convex() &&
      virtual_agent_polygon.points().size() >= 3;
  if (can_build_convex_hull) {
    virtual_agent_box = virtual_agent_polygon.MinAreaBoundingBox();
  } else if (shape_points.size() >= 2) {
    const auto farthest_shape_points = FindFarthestPoints(shape_points);
    planning_math::LineSegment2d long_axis(farthest_shape_points.first,
                                           farthest_shape_points.second);
    virtual_agent_box = planning_math::Box2d(long_axis, 0.1);
    virtual_agent_polygon = planning_math::Polygon2d(virtual_agent_box);
  } else {
    virtual_agent_polygon = planning_math::Polygon2d(virtual_agent_box);
  }

  // 3. 用拟合结果回填 virtual agent 的几何属性。
  virtual_agent.set_box(virtual_agent_box);
  virtual_agent.set_polygon(virtual_agent_polygon);
  virtual_agent.set_x(virtual_agent_box.center_x());
  virtual_agent.set_y(virtual_agent_box.center_y());
  virtual_agent.set_theta(virtual_agent_box.heading());
  virtual_agent.set_length(virtual_agent_box.length());
  virtual_agent.set_width(virtual_agent_box.width());

  // 4. 设置时域信息，并写回 agent manager。
  virtual_agent.set_time_range({0.0, kPredictionHorizon});
  virtual_agent.set_timestamp_s(0.0);

  auto &agent_manager =
      session_->mutable_environmental_model()->mutable_agent_manager();
  std::unordered_map<int32_t, agent::Agent> agent_table;
  agent_table.insert({virtual_agent.agent_id(), virtual_agent});
  agent_manager->Append(agent_table);

  ILOG_INFO << "Created ground line virtual agent id=" << id
            << " at traj_s=" << anchor_traj_point.s
            << " traj_l=" << anchor_traj_point.l;
}


void HppObstaclePreprocessDecider::ProcessDynamicObstacles() {
  auto &agent_manager =
      session_->mutable_environmental_model()->mutable_agent_manager();

  const auto &agent_ids = agent_manager->GetAgentSet();

  for (int32_t id : agent_ids) {
    auto *agent = agent_manager->mutable_agent(id);
    if (agent == nullptr) continue;

    // 对于无预测轨迹的障碍物，统一强制按静态障碍物处理，并补齐时域轨迹。
    // 轨迹点直接使用当前位姿，等效于每个时刻都赋予障碍物当前位置。
    if (agent->trajectories_used_by_st_graph().empty()) {
      agent->set_is_static(true);
      GenerateCurrentPoseTrajectory(agent);
      ILOG_INFO << "Force static agent id=" << id
                << " with current-pose trajectory";
    }
  }
}

void HppObstaclePreprocessDecider::GenerateCurrentPoseTrajectory(
    agent::Agent *agent) {
  double x = agent->x();
  double y = agent->y();
  double theta = agent->theta();

  trajectory::Trajectory traj;
  for (double t = 0.0; t <= kPredictionHorizon + 1e-6; t += kTimeResolution) {
    trajectory::TrajectoryPoint tp;
    tp.set_x(x);
    tp.set_y(y);
    tp.set_theta(theta);
    tp.set_vel(0.0);
    tp.set_acc(0.0);
    tp.set_s(0.0);
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
