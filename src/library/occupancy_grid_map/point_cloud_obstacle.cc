
#include "point_cloud_obstacle.h"

#include "aabb2d.h"
#include "log_glog.h"
#include "modules/apa_function/apa_common/apa_obstacle.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "src/modules/apa_function/apa_param_config.h"

namespace planning {

#define DEBUG_POINT_CLOUD_OBS (0)

void PointCloudObstacleTransform::GenerateLocalObstacle(
    std::shared_ptr<apa_planner::ApaObstacleManager> obs_manager,
    ParkObstacleList& obs_list, const Pose2D& ego_pose,
    const cdl::AABB& slot_box, const bool delete_slot_obs) {
  if (obs_manager == nullptr) {
    return;
  }

  Polygon2D ego_local_polygon;
  GetVehPolygonBy8Edge(1e-3, 1e-3, &ego_local_polygon);

  Polygon2D ego_global_polygon;
  ULFLocalPolygonToGlobal(&ego_global_polygon, &ego_local_polygon, ego_pose);

  const apa_planner::ApaParameters& config = apa_param.GetParam();
  bool is_collision;
  Position2D position;

  obs_list.point_cloud_list.reserve(obs_manager->GetObstacleSize());
  planning::PointCloudObstacle obs;
  for (auto& pair : obs_manager->GetObstacles()) {
    if (pair.second.GetObsMovementType() ==
        apa_planner::ApaObsMovementType::MOTION) {
      continue;
    }
    if (pair.second.GetObsAttributeType() ==
        apa_planner::ApaObsAttributeType::VIRTUAL_POINT_CLOUD) {
      continue;
    }
    if (pair.second.GetObsAttributeType() ==
        apa_planner::ApaObsAttributeType::MAP_BOUND) {
      continue;
    }

    obs.points.clear();
    obs.points.reserve(pair.second.GetPtClout2dLocal().size());

    for (const auto& pt : pair.second.GetPtClout2dLocal()) {
      if (delete_slot_obs && slot_box.contain(cdl::Vector2r(pt.x(), pt.y()))) {
        continue;
      }

      position.x = pt.x();
      position.y = pt.y();
      if (config.astar_config.enable_delete_occ_in_ego) {
        gjk_.PolygonPointCollisionDetect(&is_collision, &ego_global_polygon,
                                         position);

        if (is_collision) {
          continue;
        }
      }

      obs.points.emplace_back(position);
    }

    pair.second.GenerateLocalBoundingbox(&obs.box);

    if (pair.second.GetPtClout2dLocal().size() > 0) {
      GeneratePolygonByAABB(&obs.envelop_polygon, obs.box);
    } else {
      obs.envelop_polygon.Clear();
    }

    obs.obs_type = pair.second.GetObsAttributeType();
    obs.height_type = pair.second.GetObsHeightType();
    obs_list.point_cloud_list.emplace_back(obs);
  }

  return;
}

}  // namespace planning