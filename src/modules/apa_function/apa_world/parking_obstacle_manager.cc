#include "parking_obstacle_manager.h"

#include "local_view.h"
#include "environmental_model.h"

namespace planning {

ParkObstacleManager::ParkObstacleManager(framework::Session *session)
    : session_(session) {}

void ParkObstacleManager::Update() {
  const LocalView &local_view =
      session_->mutable_environmental_model()->get_local_view();

  ParkObstacle obs;
  Position2D global_point;
  std::vector<Position3D> perception_points;
  obs_id_ = 0;

  // fusion obj
  uint8 fusion_obj_number =
      local_view.fusion_occupancy_objects_info.fusion_object_size;
  for (uint8 i = 0; i < fusion_obj_number; i++) {
    const iflyauto::FusionOccupancyAdditional& points =
        local_view.fusion_occupancy_objects_info.fusion_object[i]
            .additional_occupancy_info;

    obs.SetPerceptionSourceType(ParkObstacleType::FUSION_OBJECT_POINT_CLOUD);
    cdl::AABB box;
    perception_points.clear();

    for (uint32 j = 0; j < points.polygon_points_size; j++) {
      global_point.x = points.polygon_points[j].x;
      global_point.y = points.polygon_points[j].y;

      box.MergePoint(cdl::Vector2r(global_point.x, global_point.y));
      perception_points.push_back(
          Position3D(global_point.x, global_point.y, 0));
    }

    // use box to generate polygon for future safe check
    if (perception_points.size() > 0) {
      obs.SetPerceptionBox(box);
    }

    obs.SetPoints(perception_points);

    obstacles_.Add(obs_id_, obs);
    obs_id_++;
  }

  // ground line
  uint8 ground_line_number =
      local_view.ground_line_perception.ground_lines_size;
  for (uint8 i = 0; i < ground_line_number; i++) {
    const iflyauto::GroundLine& gl =
        local_view.ground_line_perception.ground_lines[i];

    obs.SetPerceptionSourceType(ParkObstacleType::GROUND_LINE);
    cdl::AABB box;
    perception_points.clear();

    for (uint8 j = 0; j < gl.points_3d_size; j++) {
      global_point.x = gl.points_3d[j].x;
      global_point.y = gl.points_3d[j].y;

      box.MergePoint(cdl::Vector2r(global_point.x, global_point.y));
      perception_points.push_back(
          Position3D(global_point.x, global_point.y, 0));
    }

    if (perception_points.size() > 0) {
      obs.SetPerceptionBox(box);
    }

    obs.SetPoints(perception_points);

    obstacles_.Add(obs_id_, obs);
    obs_id_++;
  }

  // limiters
  std::vector<Position2D> limiter_points;
  const iflyauto::ParkingFusionInfo* slot_list =
      &local_view.parking_fusion_info;

  for (uint8 i = 0; i < slot_list->parking_fusion_slot_lists_size; i++) {
    const iflyauto::ParkingFusionSlot* slot =
        &slot_list->parking_fusion_slot_lists[i];

    if (slot_list->select_slot_id == slot->id) {
      continue;
    }

    obs.SetPerceptionSourceType(ParkObstacleType::SLOT_LIMITER);
    cdl::AABB box;
    perception_points.clear();

    for (uint8 j = 0; j < slot->limiters_size; j++) {
      const iflyauto::ParkingFusionLimiter* limiter = &slot->limiters[j];

      SampleInLineSegment(
          Eigen::Vector2d(limiter->end_points[0].x, limiter->end_points[0].y),
          Eigen::Vector2d(limiter->end_points[1].x, limiter->end_points[1].y),
          &limiter_points);

      for (size_t point_id = 0; point_id < limiter_points.size(); point_id++) {
        global_point.x = limiter_points[point_id].x;
        global_point.y = limiter_points[point_id].y;

        box.MergePoint(cdl::Vector2r(global_point.x, global_point.y));
        perception_points.push_back(
            Position3D(global_point.x, global_point.y, 0));
      }
    }

    if (perception_points.size() > 0) {
      obs.SetPerceptionBox(box);
    }

    obs.SetPoints(perception_points);

    obstacles_.Add(obs_id_, obs);
    obs_id_++;
  }

  ILOG_INFO << "obs, size= " << obstacles_.Items().size();

  return;
}

void ParkObstacleManager::Clear() {
  obstacles_ = IndexedList<int, ParkObstacle>();

  return;
}

ParkObstacle *ParkObstacleManager::AddObstacle(const ParkObstacle &obstacle) {
  return obstacles_.Add(obstacle.Id(), obstacle);
}

ParkObstacle *ParkObstacleManager::FindObstacle(int object_id) {
  return obstacles_.Find(object_id);
}

const ParkObstacle *ParkObstacleManager::FindObstacle(int object_id) const {
  return obstacles_.Find(object_id);
}

const IndexedList<int, ParkObstacle> &ParkObstacleManager::GetObstacles()
    const {
  return obstacles_;
}

void ParkObstacleManager::SampleInLineSegment(
    const Eigen::Vector2d& start, const Eigen::Vector2d& end,
    std::vector<Position2D>* points) {
  points->clear();

  const Eigen::Vector2d line = end - start;

  if (std::sqrt(line.x() * line.x() + line.y() * line.y()) < 0.1) {
    points->emplace_back(Position2D(start.x(), start.y()));
    points->emplace_back(Position2D(end.x(), end.y()));
    return;
  }

  const Eigen::Vector2d unit_line_vec = line.normalized();
  double len = line.norm();

  double s = 0.0;
  double ds = 0.2;

  Eigen::Vector2d point;
  while (s < len) {
    point = start + s * unit_line_vec;

    points->emplace_back(Position2D(point.x(), point.y()));

    s += ds;
  }

  points->emplace_back(Position2D(end.x(), end.y()));

  return;
}

}  // namespace planning
