#include "hpp_lon_obstacle_preprocess_decider.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

#include "common/agent/agent.h"
#include "common/agent/agent_manager.h"
#include "common/config/basic_type.h"
#include "common/math/polygon2d.h"
#include "common/tracked_object.h"
#include "context/environmental_model.h"
#include "context/obstacle.h"
#include "context/obstacle_manager.h"
#include "context/planning_context.h"
#include "context/reference_path_manager.h"
#include "math/line_segment2d.h"
#include "utils/point_segment_utils.h"
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

struct GroundLineAgentCandidate {
  int obstacle_id = -1;
  GroundLineFirstCollision collision;
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

bool HasNearbyGeneratedAgent(const std::vector<double> &generated_agent_s,
                             double target_s) {
  for (const double existing_s : generated_agent_s) {
    if (std::fabs(existing_s - target_s) < kAgentSProximityThreshold) {
      return true;
    }
  }
  return false;
}

bool CompareGroundLineAgentCandidateByCollisionS(
    const GroundLineAgentCandidate &lhs,
    const GroundLineAgentCandidate &rhs) {
  return lhs.collision.collision_s_on_traj < rhs.collision.collision_s_on_traj;
}

// 计算轨迹点处自车4个角点（世界坐标），half_w 含横向膨胀量
std::vector<planning_math::Vec2d> GetEgoCorners(const TrajectoryPoint &pt,
                                                double half_w, double front,
                                                double rear) {
  const double cos_h = std::cos(pt.heading_angle);
  const double sin_h = std::sin(pt.heading_angle);
  auto to_world = [&](double lx, double ly) {
    return planning_math::Vec2d(pt.x + lx * cos_h - ly * sin_h,
                                pt.y + lx * sin_h + ly * cos_h);
  };
  return {to_world(front, half_w), to_world(front, -half_w),
          to_world(-rear, -half_w), to_world(-rear, half_w)};
}

GroundLineFirstCollision FindEarliestGroundLineCollisionOnTrajectory(
    const Obstacle &groundline_obstacle, const TrajectoryPoints &traj_points,
    double collision_threshold) {
  GroundLineFirstCollision earliest_collision;

  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double half_w = vehicle_param.width * 0.5;
  const double front = vehicle_param.front_edge_to_rear_axle;
  const double rear = vehicle_param.rear_edge_to_rear_axle;

  const auto &groundline_points = groundline_obstacle.perception_points();

  for (size_t i = 0; i + 1 < traj_points.size(); ++i) {
    const auto &traj_pt = traj_points[i];
    const auto &traj_pt_next = traj_points[i + 1];

    if (!traj_pt.frenet_valid || !traj_pt_next.frenet_valid) {
      continue;
    }
    if (traj_pt.s >= earliest_collision.collision_s_on_traj) {
      break;
    }

    // 构造相邻两帧的扫略凸包（8角点）
    auto corners = GetEgoCorners(traj_pt, half_w, front, rear);
    const auto corners_next = GetEgoCorners(traj_pt_next, half_w, front, rear);
    corners.insert(corners.end(), corners_next.begin(), corners_next.end());

    planning_math::Polygon2d swept;
    if (!planning_math::Polygon2d::ComputeConvexHull(corners, &swept)) {
      continue;
    }

    // 粗筛 + 精检：groundline 点到扫略凸包的距离
    Pose2D ego_pose;
    ego_pose.x = traj_pt.x;
    ego_pose.y = traj_pt.y;
    ego_pose.theta = traj_pt.heading_angle;

    int32_t collision_point_count = 0;
    std::vector<planning_math::Vec2d> collision_points;
    for (const auto &groundline_point : groundline_points) {
      if (!IsPointWithinRange(ego_pose, groundline_point,
                              kCoarseCollisionCheckRange)) {
        continue;
      }
      if (swept.DistanceTo(groundline_point) > collision_threshold) {
        continue;
      }
      ++collision_point_count;
      collision_points.push_back(groundline_point);
    }

    if (collision_point_count <= kCollisionPointThreshold) {
      continue;
    }

    earliest_collision.collision_s_on_traj = traj_pt.s;
    earliest_collision.traj_point_index = static_cast<int>(i);
    earliest_collision.collision_points = std::move(collision_points);
  }

  return earliest_collision;
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

HppLonObstaclePreprocessDecider::HppLonObstaclePreprocessDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "HppLonObstaclePreprocessDecider";
  lon_config_ = config_builder->cast<LongitudinalDeciderV3Config>();
}

bool HppLonObstaclePreprocessDecider::Execute() {
  ProcessGroundLines();
  //ProcessDynamicObstacles();
  return true;
}

void HppLonObstaclePreprocessDecider::ProcessGroundLines() {
  const auto &reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  const auto &reference_path =
      reference_path_manager->get_reference_path_by_current_lane();
  if (reference_path == nullptr) {
    return;
  }

  const auto &obstacle_manager =
      session_->environmental_model().get_obstacle_manager();
  if (obstacle_manager == nullptr) {
    return;
  }
  const auto &ground_lines = obstacle_manager->get_groundline_obstacles().Items();
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

  std::vector<GroundLineAgentCandidate> ground_line_agent_candidates;
  ground_line_agent_candidates.reserve(ground_lines.size());

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
        *obstacle_it->second, traj_points, collision_threshold);
    if (!earliest_collision.IsValid()) {
      continue;
    }

    ground_line_agent_candidates.push_back(
        {frenet_obstacle->id(), std::move(earliest_collision)});
  }

  std::sort(ground_line_agent_candidates.begin(),
            ground_line_agent_candidates.end(),
            CompareGroundLineAgentCandidateByCollisionS);

  std::vector<double> generated_virtual_agent_s;
  generated_virtual_agent_s.reserve(ground_line_agent_candidates.size());
  for (const auto &ground_line_agent_candidate : ground_line_agent_candidates) {
    // 4. 避免在相近 s 位置重复生成 virtual agent。
    if (HasNearbyGeneratedAgent(
            generated_virtual_agent_s,
            ground_line_agent_candidate.collision.collision_s_on_traj)) {
      continue;
    }

    // 5. 基于碰撞点生成 virtual agent，直接复用对应 groundline 的 obstacle id。
    const auto *groundline_obstacle =
        groundline_obstacle_map.at(ground_line_agent_candidate.obstacle_id);
    const auto agent_type =
        static_cast<agent::AgentType>(groundline_obstacle->type());
    CreateVirtualAgentFromGroundLine(
        ground_line_agent_candidate.collision.collision_points,
        traj_points[ground_line_agent_candidate.collision.traj_point_index],
        ground_line_agent_candidate.obstacle_id, agent_type);
    generated_virtual_agent_s.push_back(
        ground_line_agent_candidate.collision.collision_s_on_traj);
  }
}


void HppLonObstaclePreprocessDecider::CreateVirtualAgentFromGroundLine(
    const std::vector<planning_math::Vec2d> &hit_points,
    const TrajectoryPoint &anchor_traj_point, int id,
    agent::AgentType agent_type) {
  if (hit_points.empty()) {
    return;
  }

  agent::Agent virtual_agent;
  virtual_agent.set_agent_id(id);
  virtual_agent.set_type(agent_type);
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
  const auto unique_collision_points =
      DeduplicatePointsByDistance(hit_points, kHitPointDedupDistance);
  const auto collision_segments = SplitPointsIntoSegmentsByGap(
      unique_collision_points, kHitSegmentSplitDistance);
  const auto main_collision_segment =
      SelectClosestSegmentToAnchorPoint(collision_segments, anchor_traj_point);
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


void HppLonObstaclePreprocessDecider::ProcessDynamicObstacles() {
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

void HppLonObstaclePreprocessDecider::GenerateCurrentPoseTrajectory(
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
