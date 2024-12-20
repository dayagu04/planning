
#include "occupancy_object_manager.h"
#include <cstddef>

#include "config/basic_type.h"
#include "environmental_model.h"
#include "log.h"
namespace planning {

const double kMaxDistanceY = 128.0;       // 128 6.4m  320 16m
const double kMaxDistanceFrontX = 200.0;  // 192 9.6m  600 30m
const double kMaxDistanceBackX = 40.0;    // 128 6.4m  320 16m
const double kResolution = 0.05;

OccupancyObjectManager::OccupancyObjectManager(planning::framework::Session *session)
    : session_(session) {
  Init();
}

bool OccupancyObjectManager::Init() {
  occupancy_object_.clear();
  ogm_.Clear();
  is_edt_valid_ = false;
  ogm_.Init();
  std::vector<float> lat_hierarchy_safe_buffer = {0.2, 0.1};
  float lon_front_safe_buffer = 0.2;
  edt_.Init(lat_hierarchy_safe_buffer[0],
            lon_front_safe_buffer,
            lat_hierarchy_safe_buffer[0]);
  // edt_.UpdateSafeBuffer(,,);
  return true;
}

bool OccupancyObjectManager::Update(const iflyauto::FusionOccupancyObjectsInfo &occupancy_objects_info) {
  occupancy_object_.clear();
  const bool local_point_valid = occupancy_objects_info.local_point_valid;
  if (!local_point_valid) {
    LOG_DEBUG("occupancy object points is invalid\n");
    return false;
  }
  // method 1:
  const auto &ego_state_manager = session_->environmental_model().get_ego_state_manager();
  const auto &enu2car_matrix = ego_state_manager->get_enu2car();
  // method 2:
  // Transform2d ego_base;
  // ego_base.SetBasePose(
  //   Pose2D(ego_state_manager->ego_pose().x,
  //                   ego_state_manager->ego_pose().y,
  //               ego_state_manager->ego_pose().theta));
  const size_t occupancy_objects_size = occupancy_objects_info.fusion_object_size;
  const auto occupancy_objects = occupancy_objects_info.fusion_object;
  Eigen::Vector3d v;
  std::vector<PointCloudObstacle> point_clouds;
  for (uint8 i = 0; i < occupancy_objects_size; i++) {
    // std::vector<planning_math::Vec2d> object_points;
    PointCloudObstacle point_cloud;
    size_t polygon_points_size = occupancy_objects[i].additional_occupancy_info.polygon_points_size;
    auto polygon_points = occupancy_objects[i].additional_occupancy_info.polygon_points;
    for (uint j = 0; j < polygon_points_size; j++) {
      // object_points.emplace_back(
      //     planning_math::Vec2d(polygon_points[j].x, polygon_points[j].y));
      v.x() = polygon_points[j].x;
      v.y() = polygon_points[j].y;
      v.z() = 0;
      const auto& polygon_point_car = enu2car_matrix(v);
      point_cloud.points.emplace_back(
          Position2D(polygon_point_car.x(), polygon_point_car.y()));
      // Pose2D local;
      // ego_base.GlobalPointToULFLocal(
      //   &local,
      //   Pose2D(polygon_points[j].x, polygon_points[j].y, 0));
      // point_cloud.points.emplace_back(
      //     Position2D(local.x, local.y));
    }
    // occupancy_object_.emplace(occupancy_objects[i].common_occupancy_info.type, std::move(object_points));
    point_clouds.emplace_back(std::move(point_cloud));
  }
  if (UpdateEDT(point_clouds)) {
    is_edt_valid_ = true;
  } else {
    is_edt_valid_ = false;
  }
  return true;
}

bool OccupancyObjectManager::UpdateEDT(const std::vector<PointCloudObstacle>& point_clouds) {
  ogm_.Clear();
  // base grid bound
  OccupancyGridBound grid_bound(
    -kMaxDistanceBackX * kResolution,
    -kMaxDistanceY * kResolution,
    kMaxDistanceFrontX * kResolution,
    kMaxDistanceY * kResolution);

  // reload grid bound
  ogm_.Process(grid_bound, kResolution);

  // generate obstacles
  ogm_.AddSlotCoordinatePointCloud(point_clouds);

  // edt handle
  edt_.Excute(ogm_, grid_bound, kResolution);

  return true;
}
}  // namespace planning