#include "rule_based_slot_release.h"

#include <cmath>
#include <cstddef>

#include "apa_obstacle.h"
#include "apa_param_config.h"
#include "apa_state_machine_manager.h"
#include "ego_planning_config.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "point_cloud_obstacle.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "slot_manager.h"

namespace planning {
namespace apa_planner {

static const uint8_t kSlotReleaseVoteCount = 5;
static const uint8_t kMaxSlotReleaseCount = 8;

void RuleBasedSlotRelease::Reset() { obs_list_.Clear(); }

void RuleBasedSlotRelease::Process(
    const LocalView *local_view,
    const std::shared_ptr<ApaMeasureDataManager> measure_data_ptr,
    const std::shared_ptr<ApaStateMachineManager> state_machine_ptr,
    const std::shared_ptr<ApaObstacleManager> obstacle_manager_ptr,
    std::unordered_map<size_t, iflyauto::ParkingFusionSlot> &fusion_slot_map,
    apa_planner::SlotManager::Frame &frame) {
  if (obstacle_manager_ptr == nullptr || state_machine_ptr == nullptr ||
      measure_data_ptr == nullptr) {
    return;
  }
  frame_ = &frame;
  config_ = &apa_param.GetParam();
  local_view_ = local_view;
  state_machine_ptr_ = state_machine_ptr;
  measure_data_ptr_ = measure_data_ptr;
  obstacle_manager_ptr_ = obstacle_manager_ptr;

  GenerateParkObstacleList();

  // assemble slot_management_info
  frame_->slot_management_info.mutable_slot_info_vec()->Clear();
  if (state_machine_ptr_->IsSeachingStatus()) {
    ParkingLotCruiseProcess(fusion_slot_map);
  } else {
    ParkingActivateProcess(fusion_slot_map);
  }

  return;
}

void RuleBasedSlotRelease::ParkingActivateProcess(
    std::unordered_map<size_t, iflyauto::ParkingFusionSlot> &fusion_slot_map) {
  return;
}

void RuleBasedSlotRelease::ParkingLotCruiseProcess(
    std::unordered_map<size_t, iflyauto::ParkingFusionSlot> &fusion_slot_map) {
  // Sort by the distance between the slot and the car
  const double time_start = IflyTime::Now_ms();
  std::map<double, apa_planner::SlotInfoWindow *> slot_map;
  for (auto &pair : frame_->slot_info_window_map) {
    const auto &slot = pair.second.GetFusedInfo();
    Eigen::Vector2d car_mirror =
        measure_data_ptr_->GetPos() + measure_data_ptr_->GetHeadingVec() *
                                          config_->lon_dist_mirror_to_rear_axle;

    const double dist =
        (car_mirror - Eigen::Vector2d(slot.center().x(), slot.center().y()))
            .norm();
    slot_map[dist] = &pair.second;
  }

  const bool is_ego_collision = IsEgoCloseToObs();
  uint8_t release_slot_count = 0;

  for (auto &pair : slot_map) {
    auto slot = frame_->slot_management_info.add_slot_info_vec();
    *slot = pair.second->GetFusedInfo();

    ILOG_INFO << "slot id = " << slot->id();

    if (is_ego_collision) {
      slot->set_is_release(false);
      slot->set_is_occupied(true);
      continue;
    }

    if (!slot->is_release()) {
      continue;
    }

    // only extra protect
    if (fusion_slot_map.count(slot->id()) == 0 ||
        fusion_slot_map[static_cast<size_t>(slot->id())].allow_parking == 0) {
      slot->set_is_release(false);
      slot->set_is_occupied(true);
      continue;
    }

    if (release_slot_count > kMaxSlotReleaseCount) {
      slot->set_is_release(false);
      slot->set_is_occupied(true);
      continue;
    }

    // distance is big
    if (pair.first > 10.68) {
      slot->set_is_release(false);
      slot->set_is_occupied(true);
      ILOG_INFO << "distance is big";

      continue;
    }

    if (slot->corner_points().corner_point_size() != 4) {
      slot->set_is_release(false);
      slot->set_is_occupied(true);
      continue;
    }

    if (!IsSlotCoarseRelease(slot)) {
      ILOG_INFO << "slot coarse release is false, slot id = " << slot->id();
      slot->set_is_release(false);
      slot->set_is_occupied(true);
      continue;
    }

    release_slot_count++;
  }

  ILOG_INFO << "apa lot cruise consume time = "
            << IflyTime::Now_ms() - time_start << " ms";

  return;
}

const bool RuleBasedSlotRelease::IsSlotCoarseRelease(common::SlotInfo *slot) {
  if (slot->slot_type() ==
          Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING &&
      frame_->slot_info_direction.count(slot->id()) != 0) {
    if (!frame_->slot_info_direction[slot->id()]) {
      ILOG_INFO << "car and slot is no same direction, slot id = "
                << slot->id();
      return false;
    }
  }

  // 检查自车和通道内是否有障碍物
  bool is_obs_in_slot_passage_area = false;
  if ((slot->slot_type() ==
           Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL ||
       slot->slot_type() ==
           Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING)) {
    is_obs_in_slot_passage_area =
        IsPerpendicularSlotAndPassageAreaOccupied(slot);
    ILOG_INFO << "check perpendicular or slant slot is has obs";
  } else if (slot->slot_type() ==
             Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL) {
    is_obs_in_slot_passage_area = IsParallelSlotAndPassageAreaOccupied(slot);
    ILOG_INFO << "check parallel slot is has obs";
  }

  if (is_obs_in_slot_passage_area) {
    if (frame_->slot_release_voter.count(slot->id()) != 0) {
      frame_->slot_release_voter[slot->id()] = 0;
    }
    ILOG_INFO << "obs is in slot_passage_area, slot id = " << slot->id();
    return false;
  } else {
    if (frame_->slot_release_voter.count(slot->id()) == 0) {
      frame_->slot_release_voter[slot->id()] = 1;
    } else {
      if (frame_->slot_release_voter[slot->id()] < kSlotReleaseVoteCount + 5) {
        frame_->slot_release_voter[slot->id()]++;
      }
      if (frame_->slot_release_voter[slot->id()] < kSlotReleaseVoteCount) {
        ILOG_INFO << "voter count is not enough, slot id = " << slot->id();
        return false;
      }
    }
  }

  return true;
}

void RuleBasedSlotRelease::GenerateParkObstacleList() {
  obs_list_.Clear();
  if (obstacle_manager_ptr_ == nullptr) {
    return;
  }

  for (const auto &pair : obstacle_manager_ptr_->GetObstacles()) {
    if (!apa_param.GetParam().use_uss_pt_clound &&
        pair.second.GetObsAttributeType() ==
            ApaObsAttributeType::USS_POINT_CLOUD) {
      continue;
    }
    planning::PointCloudObstacle obs;
    obs.points.reserve(pair.second.GetPtClout2dGlobal().size());
    for (const auto &pt : pair.second.GetPtClout2dGlobal()) {
      obs.points.emplace_back(Position2D(pt.x(), pt.y()));
    }
    obs.box = pair.second.GetBoxGlobal();
    obs.envelop_polygon = pair.second.GetPolygon2DGlobal();
    obs.obs_type = pair.second.GetObsAttributeType();
    obs_list_.point_cloud_list.emplace_back(obs);
  }
}

const bool RuleBasedSlotRelease::IsEgoCloseToObs() {
  PathSafeChecker safe_check;
  Pose2D ego =
      Pose2D(measure_data_ptr_->GetPos()[0], measure_data_ptr_->GetPos()[1],
             measure_data_ptr_->GetHeading());
  return safe_check.CalcEgoCollision(&obs_list_, ego, 0.1, 0.1);
}

const bool RuleBasedSlotRelease::IsEgoCloseToObs(const LocalView *local_view) {
  // move all obstacle related code to one place.
  PointCloudObstacleTransform transform;

  obs_list_.Clear();
  transform.GenerateGlobalObstacle(obs_list_, local_view, false);

  PathSafeChecker safe_check;
  Pose2D ego =
      Pose2D(measure_data_ptr_->GetPos()[0], measure_data_ptr_->GetPos()[1],
             measure_data_ptr_->GetHeading());
  bool collision = safe_check.CalcEgoCollision(&obs_list_, ego, 0.1, 0.1);

  return collision;
}

const bool RuleBasedSlotRelease::IsPerpendicularSlotAndPassageAreaOccupied(
    const common::SlotInfo *slot) {
  const auto &slot_points = slot->corner_points().corner_point();
  if (slot_points.size() != 4) {
    ILOG_INFO << "slot_points.size = " << slot_points.size();
    return true;
  }
  std::vector<Eigen::Vector2d> pt;
  pt.resize(4);
  for (size_t i = 0; i < 4; ++i) {
    pt[i] << slot_points[i].x(), slot_points[i].y();
  }
  const Eigen::Vector2d pM01 = 0.5 * (pt[0] + pt[1]);
  const Eigen::Vector2d pM23 = 0.5 * (pt[2] + pt[3]);
  const Eigen::Vector2d t = (pt[1] - pt[0]).normalized();
  const Eigen::Vector2d n = (pM01 - pM23).normalized();
  const double heading = std::atan2(n.y(), n.x());

  const Eigen::Vector2d origin_target_pos =
      pM01 - n * (config_->wheel_base + config_->front_overhanging - 0.68);

  const std::vector<double> move_slot_dist_vec{0.0,   0.05,  0.10,  0.15, 0.2,
                                               -0.05, -0.10, -0.15, -0.2};
  PathSafeChecker safe_check;
  const double lat_buffer = 0.16;
  const double lon_buffer = 0.1;
  double move_slot_dist = 0.0;
  bool is_slot_occupied = true;
  for (const double dist : move_slot_dist_vec) {
    const Eigen::Vector2d target_pos = origin_target_pos + dist * t;
    const Pose2D target_pose(target_pos.x(), target_pos.y(), heading);
    if (!safe_check.CalcEgoCollision(&obs_list_, target_pose, lat_buffer,
                                     lon_buffer)) {
      move_slot_dist = dist;
      ILOG_INFO << "move_slot_dist = " << move_slot_dist;
      is_slot_occupied = false;
      break;
    }
  }

  if (is_slot_occupied) {
    ILOG_INFO << "slot is occupied";
    return true;
  }

  Eigen::Vector2d pt_0 = pM01 - t * (config_->max_car_width * 0.5 + lat_buffer);
  Eigen::Vector2d pt_1 = pM01 + t * (config_->max_car_width * 0.5 + lat_buffer);

  pt_0 = pt_0 + move_slot_dist * t;
  pt_1 = pt_1 + move_slot_dist * t;

  const double channel_width = 3.0;
  Polygon2D polygon;
  polygon.vertexes[0].x = pt_0.x();
  polygon.vertexes[0].y = pt_0.y();

  polygon.vertexes[1].x = (pt_0 + channel_width * n).x();
  polygon.vertexes[1].y = (pt_0 + channel_width * n).y();

  polygon.vertexes[1].x = (pt_1 + channel_width * n).x();
  polygon.vertexes[1].y = (pt_1 + channel_width * n).y();

  polygon.vertexes[2].x = pt_1.x();
  polygon.vertexes[2].y = pt_1.y();

  polygon.vertex_num = 4;
  polygon.shape = PolygonShape::box;
  UpdatePolygonValue(&polygon, NULL, 0, false, POLYGON_MAX_RADIUS);

  polygon.min_tangent_radius = 0.6;

  safe_check.SetObstacle(&obs_list_);
  if (safe_check.IsPolygonCollision(&polygon)) {
    ILOG_INFO << "passage is occupied";
    return true;
  }

  return false;
}

const bool RuleBasedSlotRelease::IsParallelSlotAndPassageAreaOccupied(
    const common::SlotInfo *slot) {
  const auto &slot_points = slot->corner_points().corner_point();
  if (slot_points.size() != 4) {
    return true;
  }
  std::vector<Eigen::Vector2d> pt;
  pt.resize(4);
  for (size_t i = 0; i < 4; ++i) {
    pt[i] << slot_points[i].x(), slot_points[i].y();
  }
  const Eigen::Vector2d pM02 = 0.5 * (pt[0] + pt[2]);
  const Eigen::Vector2d pM13 = 0.5 * (pt[1] + pt[3]);
  const Eigen::Vector2d t = (pt[0] - pt[2]).normalized();
  const Eigen::Vector2d n(-t.y(), t.x());
  const double heading = std::atan2(n.y(), n.x());

  const Eigen::Vector2d origin_target_pos =
      pM02 + n * (config_->rear_overhanging + 0.168);

  const std::vector<double> move_slot_dist_vec{0.0, 0.1, 0.2, 0.3};

  PathSafeChecker safe_check;
  const double lat_buffer = 0.1;
  const double lon_buffer = 0.05;
  double move_slot_dist = 0.0;
  bool is_slot_occupied = true;
  for (const double dist : move_slot_dist_vec) {
    const Eigen::Vector2d target_pos = origin_target_pos + dist * t;
    const Pose2D target_pose(target_pos.x(), target_pos.y(), heading);
    if (!safe_check.CalcEgoCollision(&obs_list_, target_pose, lat_buffer,
                                     lon_buffer)) {
      move_slot_dist = dist;
      ILOG_INFO << "move_slot_dist = " << move_slot_dist;
      is_slot_occupied = false;
      break;
    }
  }
  return is_slot_occupied;
}

const bool RuleBasedSlotRelease::IsSlotOccupied(const common::SlotInfo *slot) {
  const auto slot_pts = slot->corner_points().corner_point();

  Eigen::Vector2d pt_0 = Eigen::Vector2d(slot_pts[0].x(), slot_pts[0].y());
  Eigen::Vector2d pt_1 = Eigen::Vector2d(slot_pts[1].x(), slot_pts[1].y());

  const Eigen::Vector2d pt_01_vec = pt_1 - pt_0;
  Eigen::Vector2d pt_01_vec_n(pt_01_vec.y(), -pt_01_vec.x());
  pt_01_vec_n.normalize();

  const Eigen::Vector2d pt_01_mid = (pt_0 + pt_1) * 0.5;

  Eigen::Vector2d target_pos =
      pt_01_mid -
      pt_01_vec_n * (config_->wheel_base + config_->front_overhanging);

  Pose2D target_pose;
  target_pose.x = target_pos[0];
  target_pose.y = target_pos[1];
  target_pose.theta = std::atan2(pt_01_vec_n[1], pt_01_vec_n[0]);

  PathSafeChecker safe_check;
  // 车位释放很激进，buffer是负值
  double lateral_buffer = -0.2;
  double lon_buffer = -0.1;
  bool collision = safe_check.CalcEgoCollision(&obs_list_, target_pose,
                                               lateral_buffer, lon_buffer);

  return collision ? true : false;
}

bool RuleBasedSlotRelease::IsPassageAreaEnough(const common::SlotInfo *slot) {
  // 暂时使用库口2米内没有障碍物判断.
  // 未来可以使用最大子矩阵和算法，来判定一个通道的大小.
  const auto slot_pts = slot->corner_points().corner_point();

  Eigen::Vector2d pt_0 = Eigen::Vector2d(slot_pts[0].x(), slot_pts[0].y());
  Eigen::Vector2d pt_1 = Eigen::Vector2d(slot_pts[1].x(), slot_pts[1].y());

  Eigen::Vector2d pt_01_vec = pt_1 - pt_0;
  pt_01_vec.normalize();

  Eigen::Vector2d pt_01_vec_n(pt_01_vec.y(), -pt_01_vec.x());
  pt_01_vec_n.normalize();

  const Eigen::Vector2d pt_01_mid = (pt_0 + pt_1) * 0.5;

  // move pt0, pt1
  pt_0 = pt_01_mid - pt_01_vec * (config_->max_car_width / 2 - 0.3);
  pt_1 = pt_01_mid + pt_01_vec * (config_->max_car_width / 2 - 0.3);

  // check length
  Polygon2D polygon;
  polygon.vertexes[0].x = (pt_0)[0];
  polygon.vertexes[0].y = (pt_0)[1];
  polygon.vertexes[1].x = (pt_0 + pt_01_vec_n * 2.0)[0];
  polygon.vertexes[1].y = (pt_0 + pt_01_vec_n * 2.0)[1];

  polygon.vertexes[2].x = (pt_1 + pt_01_vec_n * 2.0)[0];
  polygon.vertexes[2].y = (pt_1 + pt_01_vec_n * 2.0)[1];
  polygon.vertexes[3].x = (pt_1)[0];
  polygon.vertexes[3].y = (pt_1)[1];

  polygon.vertex_num = 4;
  polygon.shape = PolygonShape::box;
  UpdatePolygonValue(&polygon, NULL, 0, false, POLYGON_MAX_RADIUS);

  polygon.min_tangent_radius = 0.6;

  PathSafeChecker safe_check;

  safe_check.SetObstacle(&obs_list_);
  bool collision = safe_check.IsPolygonCollision(&polygon);

  return collision ? false : true;
}

}  // namespace apa_planner
}  // namespace planning