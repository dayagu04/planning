
#include "hybrid_astar_common.h"
#include <cmath>
#include "point_cloud_obstacle.h"

namespace planning {

std::string PathGearDebugString(const AstarPathGear gear) {
  switch (gear) {
    case AstarPathGear::DRIVE:
      return "drive";
    case AstarPathGear::REVERSE:
      return "reverse";
    case AstarPathGear::NORMAL:
      return "normal";
    default:
      return "parking";
      break;
  }

  return "none";
}

std::string GetPathSteerDebugString(const AstarPathSteer type) {
  switch (type) {
    case AstarPathSteer::LEFT:
      return "left";
    case AstarPathSteer::RIGHT:
      return "right";
    case AstarPathSteer::STRAIGHT:
      return "straight";
    default:
      break;
  }

  return "none";
}

std::string GetNodeCurveDebugString(const AstarPathType type) {
  switch (type) {
    case AstarPathType::REEDS_SHEPP:
      return "REEDS_SHEPP";
    case AstarPathType::DUBINS:
      return "DUBINS";
    case AstarPathType::NODE_SEARCHING:
      return "NODE_SEARCHING";
    case AstarPathType::LINE_SEGMENT:
      return "LINE_SEGMENT";
    case AstarPathType::CUBIC_POLYNOMIAL:
      return "CUBIC_POLYNOMIAL";
    case AstarPathType::QUNTIC_POLYNOMIAL:
      return "QUNTIC_POLYNOMIAL";
    case AstarPathType::SPIRAL:
      return "SPIRAL";
    default:
      break;
  }

  return "NONE";
}

bool IsGearDifferent(const AstarPathGear left, const AstarPathGear right) {
  if (left == AstarPathGear::DRIVE && right == AstarPathGear::REVERSE) {
    return true;
  }

  if (left == AstarPathGear::REVERSE && right == AstarPathGear::DRIVE) {
    return true;
  }

  return false;
}

std::string PlanReasonDebugString(const PlanningReason reason) {
  switch (reason) {
    case PlanningReason::FIRST_PLAN:
      return "FIRST_PLAN";
    case PlanningReason::ADJUST_SELF_CAR_POSE:
      return "ADJUST_SELF_CAR_POSE";
    case PlanningReason::GEOMETRY_CURVE_FAIL:
      return "GEOMETRY_CURVE_FAIL";
    case PlanningReason::PATH_COMPLETED:
      return "PATH_COMPLETED";
    case PlanningReason::PATH_STUCKED:
      return "PATH_STUCKED";
    case PlanningReason::SLOT_REFRESHED:
      return "SLOT_REFRESHED";
    case PlanningReason::SIMULATION_TRIGGER:
      return "SIMULATION_TRIGGER";
    default:
      return "none";
      break;
  }

  return "none";
}

void DebugPolynomialPath(const std::vector<AStarPathPoint>& poly_path) {
  for (size_t i = 0; i < poly_path.size(); i++) {
    ILOG_INFO << "x = " << poly_path[i].x << ",y=" << poly_path[i].y
              << ",theta=" << poly_path[i].phi
              << ",kappa=" << poly_path[i].kappa
              << ",s = " << poly_path[i].accumulated_s
              << ", gear = " << static_cast<int>(poly_path[i].gear);
  }

  return;
}

void DebugPathString(const HybridAStarResult* result) {
  if (result == nullptr) {
    return;
  }

  ILOG_INFO << "path x point size " << result->x.size() << " gear size "
            << result->gear.size() << "y size " << result->y.size()
            << "phi size " << result->phi.size() << "type size "
            << result->type.size() << "s size " << result->accumulated_s.size();

  for (size_t i = 0; i < result->x.size(); i++) {
    ILOG_INFO << "i = " << i << " x, y, theta, gear:  " << result->x[i] << ", "
              << result->y[i] << ", " << result->phi[i] * 57.4 << ", "
              << PathGearDebugString(result->gear[i])
              << ",path type = " << GetNodeCurveDebugString(result->type[i])
              << ", s = " << result->accumulated_s[i]
              << ", kappa = " << result->kappa[i];
  }

  return;
}

void ExtendPathToRealParkSpacePoint(HybridAStarResult* result,
                                    const Pose2f& real_end) {
  if (result == nullptr || result->x.size() < 1) {
    ILOG_INFO << "no path";
    return;
  }

  Eigen::Vector2f astar_end_point;
  astar_end_point[0] = result->x.back();
  astar_end_point[1] = result->y.back();

  // check path end
  if (astar_end_point[0] <= real_end.x) {
    return;
  }

  float extend_dist = real_end.DistanceTo(
      Pose2f(astar_end_point[0], astar_end_point[1], result->phi.back()));
  if (extend_dist < 0.1) {
    return;
  }

  float phi = result->phi.back();
  AstarPathGear gear = result->gear.back();
  float astar_end_s = result->accumulated_s.back();
  AstarPathType path_type = AstarPathType::LINE_SEGMENT;

  Eigen::Vector2f unit_line_vec = Eigen::Vector2f(-1.0, 0.0);

  float s = 0.1;
  float ds = 0.1;

  Eigen::Vector2f point;
  while (s < extend_dist) {
    point = astar_end_point + s * unit_line_vec;
    result->x.emplace_back(point[0]);
    result->y.emplace_back(point[1]);
    result->phi.emplace_back(phi);
    result->gear.emplace_back(gear);
    result->type.emplace_back(path_type);
    result->accumulated_s.emplace_back(astar_end_s + s);
    result->kappa.emplace_back(0.0);

    s += ds;
  }

  float x_diff = real_end.x - result->x.back();
  float dist_diff = std::sqrt(x_diff * x_diff);
  if (dist_diff > 1e-2) {
    float last_s = result->accumulated_s.back();

    // add end
    result->x.emplace_back(real_end.x);
    result->y.emplace_back(astar_end_point[1]);
    result->phi.emplace_back(phi);
    result->gear.emplace_back(gear);
    result->type.emplace_back(path_type);
    result->accumulated_s.emplace_back(last_s + dist_diff);
    result->kappa.emplace_back(0);
  }

  return;
}

bool IsSearchNode(const AstarPathType type) {
  if (type == AstarPathType::NODE_SEARCHING ||
      type == AstarPathType::START_NODE || type == AstarPathType::END_NODE) {
    return true;
  }

  return false;
}

const cdl::AABB GetAABoxByPath(const HybridAStarResult& result,
                               const double back_overhanging,
                               const double front_edge_to_rear_axis,
                               const double half_width) {
  Polygon2D global_polygon;
  Pose2D global_pose;
  cdl::AABB path_point_aabb;
  cdl::AABB box = cdl::AABB();
  Polygon2D veh_local_polygon;
  GetUpLeftCoordinatePolygonByParam(&veh_local_polygon, back_overhanging,
                                    front_edge_to_rear_axis, half_width);

  for (size_t i = 0; i < result.x.size(); ++i) {
    global_pose.x = result.x[i];
    global_pose.y = result.y[i];
    global_pose.theta = result.phi[i];
    ULFLocalPolygonToGlobal(&global_polygon, &veh_local_polygon, global_pose);

    GetBoundingBoxByPolygon(&path_point_aabb, &global_polygon);

    box.combine(path_point_aabb);
  }

  return box;
}

const MapBound TransformMapBound(const cdl::AABB& box) {
  MapBound bound;
  bound.x_max = box.max_[0];
  bound.y_max = box.max_[1];
  bound.x_min = box.min_[0];
  bound.y_min = box.min_[1];

  return bound;
}

void DebugMapBoundString(const MapBound& box) {
  ILOG_INFO << "map bound, xmin " << box.x_min << " , ymin " << box.y_min
            << " ,xmax " << box.x_max << " , ymax " << box.y_max;
  return;
}

const cdl::AABB2f TransformMapBound(const MapBound& box) {
  cdl::AABB2f  bound;
  bound.min_[0] = box.x_min;
  bound.min_[1] = box.y_min;
  bound.max_[0] = box.x_max;
  bound.max_[1] = box.y_max;

  return bound;
}

}  // namespace planning