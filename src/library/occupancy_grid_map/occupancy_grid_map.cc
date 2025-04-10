
#include "occupancy_grid_map.h"

#include <cstdint>

#include "ogm_common.h"
#include "pose2d.h"

namespace planning {

void OccupancyGridMap::Process(const Pose2D &ogm_pose,
                               const float _ogm_resolution) {
  OccupancyGridCoordinate::Process(ogm_pose, _ogm_resolution);

  return;
}

void OccupancyGridMap::Process(const OccupancyGridBound &bound,
                               const float _ogm_resolution) {
  OccupancyGridCoordinate::Process(bound, _ogm_resolution);
  return;
}

void OccupancyGridMap::Clear() {
  std::memset(ogm, false, sizeof(ogm));
  // for (int32_t i = 0; i < ogm_grid_x_max; i++) {
  //   for (int32_t j = 0; j < ogm_grid_y_max; j++) {
  //     ogm[i][j] = false;
  //   }
  // }
  return;
}

template <typename T>
void OccupancyGridMap::AddSlotPoint(const T &point) {
  Pose2D local;

  local.x = point.x - bound_.min_x;
  local.y = point.y - bound_.min_y;

  OgmIndex index;
  OgmPoseToIndex(&index, local);

  if (IsIndexValid(index)) {
    ogm[index.x][index.y] = true;
  }

  return;
}

void OccupancyGridMap::AddSlotCoordinatePoint(
    const Position2D &point) {
  Pose2D local;

  local.x = point.x - bound_.min_x;
  local.y = point.y - bound_.min_y;

  OgmIndex index;
  OgmPoseToIndex(&index, local);

  if (IsIndexValid(index)) {
    ogm[index.x][index.y] = true;
  }

  return;
}

void OccupancyGridMap::AddSlotCoordinatePoints(
    const std::vector<Position2D> &points) {
  Pose2D local;
  OgmIndex index;

  for (size_t i = 0; i < points.size(); i++) {
    local.x = points[i].x - bound_.min_x;
    local.y = points[i].y - bound_.min_y;

    OgmPoseToIndex(&index, local);

    if (IsIndexValid(index)) {
      ogm[index.x][index.y] = true;
    }
  }

  return;
}

template <typename T>
void OccupancyGridMap::AddLineSegment(const T &start, const T &end) {
  Pose2D local;
  OgmIndex index;

  Eigen::Vector2f dir(end.x - start.x, end.y - start.y);
  float len = dir.norm();
  dir.normalize();

  float s = 0.0;
  float ds = 0.2;

  Position2D point;
  while (s < len) {
    point.x = start.x + s * dir.x();
    point.y = start.y + s * dir.y();

    local.x = point.x - bound_.min_x;
    local.y = point.y - bound_.min_y;

    OgmPoseToIndex(&index, local);

    if (IsIndexValid(index)) {
      ogm[index.x][index.y] = true;
    }

    s += ds;
  }

  local.x = end.x - bound_.min_x;
  local.y = end.y - bound_.min_y;
  OgmPoseToIndex(&index, local);
  if (IsIndexValid(index)) {
    ogm[index.x][index.y] = true;
  }

  return;
}

void OccupancyGridMap::AddSlotCoordinatePointCloud(
    const std::vector<PointCloudObstacle> &point_cloud_list) {
  Pose2D local;
  OgmIndex index;

  for (size_t i = 0; i < point_cloud_list.size(); i++) {
    const std::vector<Position2D> &points = point_cloud_list[i].points;

    for (size_t j = 0; j < points.size(); j++) {
      local.x = points[j].x - bound_.min_x;
      local.y = points[j].y - bound_.min_y;

      OgmPoseToIndex(&index, local);

      if (IsIndexValid(index)) {
        ogm[index.x][index.y] = true;
      }
    }
  }
  return;
}

void OccupancyGridMap::TransformToMatrix(cv::Mat *mat) const {
  int row_num = mat->rows;
  int column_num = mat->cols;
  OgmIndex index;

  for (int32_t i = 0; i < row_num; i++) {
    uchar *data = mat->ptr<uchar>(i);

    for (int32_t j = 0; j < column_num; j++) {
      index.x = i;
      index.y = j;
      if (IsIndexValid(index)) {
        if (ogm[i][j]) {
          data[j] = 0;
        }
      }
    }
  }

  return;
}

void OccupancyGridMap::Init() { return; }

void OccupancyGridMap::AddParkingObs(const ParkObstacleList &obs) {
  AddSlotCoordinatePoints(obs.virtual_obs);
  AddSlotCoordinatePointCloud(obs.point_cloud_list);

  return;
}

}  // namespace planning