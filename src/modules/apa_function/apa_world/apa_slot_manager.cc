#include "apa_slot_manager.h"

#include <cstddef>
#include <map>
#include <unordered_map>

#include "apa_param_config.h"
#include "apa_slot.h"
#include "apa_state_machine_manager.h"
#include "collision_detection/path_safe_checker.h"
#include "ifly_time.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {

static const uint8_t kSlotReleaseVoteCount = 5;
static const uint8_t kMaxSlotReleaseCount = 8;

void ApaSlotManager::Update(
    const LocalView* local_view,
    const std::shared_ptr<ApaStateMachineManager> state_machine_ptr,
    const std::shared_ptr<ApaMeasureDataManager> measure_data_ptr,
    const std::shared_ptr<ApaObstacleManager> obstacle_manager_ptr) {
  if (local_view == nullptr || state_machine_ptr == nullptr ||
      measure_data_ptr == nullptr || obstacle_manager_ptr == nullptr) {
    ILOG_ERROR << "Update ApaSlotManager, local_view_ptr is nullptr";
    return;
  }

  ILOG_INFO << "Update ApaSlotManager";

  state_machine_ptr_ = state_machine_ptr;
  measure_data_ptr_ = measure_data_ptr;
  obstacle_manager_ptr_ = obstacle_manager_ptr;

  dist_id_map_.clear();
  slots_map_.clear();

  for (uint8_t i = 0;
       i < local_view->parking_fusion_info.parking_fusion_slot_lists_size;
       ++i) {
    const auto& fusion_slot =
        local_view->parking_fusion_info.parking_fusion_slot_lists[i];

    ApaSlot slot;
    slot.Update(fusion_slot);

    const Eigen::Vector2d car_mirror_pos =
        (measure_data_ptr->GetLeftMirrorPos() +
         measure_data_ptr->GetRightMirrorPos()) *
        0.5;

    const double dist =
        (car_mirror_pos - slot.GetOriginCornerCoordGlobal().pt_center).norm();

    dist_id_map_[dist] = slot.GetId();
    slots_map_[slot.GetId()] = slot;
  }

  // 泊出
  if (state_machine_ptr_->IsParkOutStatus()) {
    if (state_machine_ptr_->IsSeachingStatus()) {
      ILOG_INFO << "dist_id_map_.begin()->second = "
                << dist_id_map_.begin()->second;
      ILOG_INFO << "slots_map_[ego_info_under_slot_.id].slot_type_ = "
                << static_cast<int>(
                       slots_map_[ego_info_under_slot_.id].slot_type_);
      ego_info_under_slot_.id = dist_id_map_.begin()->second;
      ego_info_under_slot_.slot_type =
          slots_map_[ego_info_under_slot_.id].slot_type_;
    } else if (state_machine_ptr_->IsParkingStatus()) {
    }
  }

  // 泊入
  if (state_machine_ptr->IsParkInStatus()) {
    if (state_machine_ptr_->IsSeachingStatus()) {
      ParkingLotCruiseProcess();
      if (slots_map_.count(local_view->parking_fusion_info.select_slot_id) !=
          0) {
        ego_info_under_slot_.id =
            local_view->parking_fusion_info.select_slot_id;
        ego_info_under_slot_.slot_type =
            slots_map_[ego_info_under_slot_.id].slot_type_;
      } else {
        ego_info_under_slot_.Reset();
      }
    } else if (state_machine_ptr_->IsParkingStatus()) {
    }
  }

  const SlotReleaseState last_geometry_release =
      ego_info_under_slot_.slot.release_info_
          .release_state[GEOMETRY_PLANNING_RELEASE];

  const SlotReleaseState last_astar_release =
      ego_info_under_slot_.slot.release_info_
          .release_state[ASTAR_PLANNING_RELEASE];

  if (slots_map_.count(ego_info_under_slot_.id) != 0 &&
      !ego_info_under_slot_.fix_slot) {
    ego_info_under_slot_.slot = slots_map_[ego_info_under_slot_.id];
  }

  // keep last release state here, and would change later when searching
  ego_info_under_slot_.slot.release_info_
      .release_state[GEOMETRY_PLANNING_RELEASE] = last_geometry_release;
  ego_info_under_slot_.slot.release_info_
      .release_state[ASTAR_PLANNING_RELEASE] = last_astar_release;

  ILOG_INFO << "select slot id = " << ego_info_under_slot_.id
            << "  type = " << GetSlotTypeString(ego_info_under_slot_.slot_type);
}

void ApaSlotManager::GenerateReleaseSlotIdVec() {
  if (!state_machine_ptr_->IsSeachingStatus()) {
    return;
  }
  release_slot_id_vec_.clear();
  for (const auto& pair : slots_map_) {
    if (pair.first != ego_info_under_slot_.id) {
      if (pair.second.release_info_.release_state[RULE_BASED_RELEASE] ==
          SlotReleaseState::RELEASE) {
        release_slot_id_vec_.emplace_back(pair.first);
      }
    } else {
      if (ego_info_under_slot_.slot.release_info_
                  .release_state[GEOMETRY_PLANNING_RELEASE] ==
              SlotReleaseState::RELEASE ||
          ego_info_under_slot_.slot.release_info_
                  .release_state[ASTAR_PLANNING_RELEASE] ==
              SlotReleaseState::RELEASE) {
        release_slot_id_vec_.emplace_back(pair.first);
      }
    }
  }
}

void ApaSlotManager::ParkingLotCruiseProcess() {
  // 在泊入寻库阶段通过简单的规则判断车位是否应该释放
  const double time_start = IflyTime::Now_ms();

  const bool is_ego_collision = IsEgoCloseToObs();
  uint8_t release_slot_count = 0;

  // 按距离自车近远顺序进行遍历
  for (const auto& dist_id : dist_id_map_) {
    ApaSlot& slot = slots_map_[dist_id.second];
    ILOG_INFO << "slot id = " << slot.GetId()
              << "  type = " << GetSlotTypeString(slot.GetType())
              << "  fusion_release = "
              << GetSlotReleaseStateString(
                     slot.release_info_.release_state[FUSION_RELEASE]);

    if (slot.release_info_.release_state[FUSION_RELEASE] ==
        SlotReleaseState::NOT_RELEASE) {
      continue;
    }

    if (is_ego_collision) {
      slot.release_info_.release_state[RULE_BASED_RELEASE] =
          SlotReleaseState::NOT_RELEASE;
      continue;
    }

    if (release_slot_count > kMaxSlotReleaseCount) {
      slot.release_info_.release_state[RULE_BASED_RELEASE] =
          SlotReleaseState::NOT_RELEASE;
      continue;
    }

    if (dist_id.first > 10.68) {
      slot.release_info_.release_state[RULE_BASED_RELEASE] =
          SlotReleaseState::NOT_RELEASE;
      continue;
    }

    if (!IsSlotCoarseRelease(slot)) {
      ILOG_INFO << "slot coarse release is false, slot id = " << slot.id_;
      slot.release_info_.release_state[RULE_BASED_RELEASE] =
          SlotReleaseState::NOT_RELEASE;
      continue;
    }

    slot.release_info_.release_state[RULE_BASED_RELEASE] =
        SlotReleaseState::RELEASE;

    release_slot_count++;
  }

  ILOG_INFO << "apa lot cruise consume time = "
            << IflyTime::Now_ms() - time_start << " ms";

  return;
}

const bool ApaSlotManager::IsEgoCloseToObs() {
  PathSafeChecker safe_check;
  Pose2D ego =
      Pose2D(measure_data_ptr_->GetPos()[0], measure_data_ptr_->GetPos()[1],
             measure_data_ptr_->GetHeading());
  return safe_check.CalcEgoCollision(obstacle_manager_ptr_, ego, 0.268, 0.1);
}

const bool ApaSlotManager::IsSlotCoarseRelease(const ApaSlot& slot) {
  if (slot.slot_type_ == SlotType::SLANT) {
    // 车尾泊入和车头泊入释放要求车位和自车的相对方向不一致
    // 因次不在此作基于规则释放 在规划器内部预规划时做是否释放
  }

  // 检查自车和通道内是否有障碍物
  bool is_obs_in_slot_passage_area = false;
  if ((slot.slot_type_ == SlotType::PERPENDICULAR ||
       slot.slot_type_ == SlotType::SLANT)) {
    is_obs_in_slot_passage_area =
        IsPerpendicularSlotAndPassageAreaOccupied(slot);
  } else if (slot.slot_type_ == SlotType::PARALLEL) {
    is_obs_in_slot_passage_area = IsParallelSlotAndPassageAreaOccupied(slot);
  }

  if (is_obs_in_slot_passage_area) {
    if (slot_release_voter_.count(slot.id_) != 0) {
      slot_release_voter_[slot.id_] = 0;
    }
    ILOG_INFO << "obs is in slot_passage_area, slot id = " << slot.id_;
    return false;
  } else {
    if (slot_release_voter_.count(slot.id_) == 0) {
      slot_release_voter_[slot.id_] = 1;
    } else {
      if (slot_release_voter_[slot.id_] < kSlotReleaseVoteCount + 5) {
        slot_release_voter_[slot.id_]++;
      }
    }

    if (slot_release_voter_[slot.id_] < kSlotReleaseVoteCount) {
      ILOG_INFO << "voter count is not enough, slot id = " << slot.id_;
      return false;
    }
  }

  return true;
}

const bool ApaSlotManager::IsPerpendicularSlotAndPassageAreaOccupied(
    const ApaSlot& slot) {
  const auto& param = apa_param.GetParam();
  const Eigen::Vector2d pM01 = slot.processed_corner_coord_global_.pt_01_mid;
  const Eigen::Vector2d pM23 = slot.processed_corner_coord_global_.pt_23_mid;
  const Eigen::Vector2d t =
      slot.processed_corner_coord_global_.pt_01_vec.normalized();
  const Eigen::Vector2d n =
      slot.processed_corner_coord_global_.pt_23mid_01_mid.normalized();

  const double heading = std::atan2(n.y(), n.x());

  const double max_move_dist = slot.slot_width_ * 0.5 - 1.1 + 0.2;
  std::vector<double> move_slot_dist_vec{0.0};
  double move_dist = 0.05;
  while (move_dist < max_move_dist + 0.04) {
    move_slot_dist_vec.emplace_back(move_dist);
    move_slot_dist_vec.emplace_back(-move_dist);
    move_dist += 0.05;
  }

  const std::vector<double> move_up_dist{0.68, 1.08, 1.48};

  PathSafeChecker safe_check;
  double lat_buffer = param.slot_release_car_lat_buffer;
  const double lon_buffer = 0.1;
  double move_slot_dist = 0.0;
  bool is_slot_occupied = true;
  for (const double dist : move_slot_dist_vec) {
    bool col = false;
    for (const double up_dist : move_up_dist) {
      const Eigen::Vector2d origin_target_pos =
          pM01 - n * (param.wheel_base + param.front_overhanging - up_dist);
      const Eigen::Vector2d target_pos = origin_target_pos + dist * t;
      const Pose2D target_pose(target_pos.x(), target_pos.y(), heading);
      if (safe_check.CalcEgoCollision(obstacle_manager_ptr_, target_pose,
                                      lat_buffer, lon_buffer)) {
        col = true;
        break;
      }
    }
    if (col) {
      continue;
    }
    move_slot_dist = dist;
    ILOG_INFO << "move_slot_dist = " << move_slot_dist;
    is_slot_occupied = false;
    break;
  }

  if (is_slot_occupied) {
    ILOG_INFO << "slot is occupied";
    return true;
  }

  lat_buffer = 0.0368;

  Eigen::Vector2d pt_0 = pM01 - t * (param.max_car_width * 0.5 + lat_buffer);
  Eigen::Vector2d pt_1 = pM01 + t * (param.max_car_width * 0.5 + lat_buffer);

  pt_0 = pt_0 + move_slot_dist * t;
  pt_1 = pt_1 + move_slot_dist * t;

  const double channel_width = 3.0;
  Polygon2D polygon;
  polygon.vertexes[0].x = pt_0.x();
  polygon.vertexes[0].y = pt_0.y();

  polygon.vertexes[1].x = (pt_0 + channel_width * n).x();
  polygon.vertexes[1].y = (pt_0 + channel_width * n).y();

  polygon.vertexes[2].x = (pt_1 + channel_width * n).x();
  polygon.vertexes[2].y = (pt_1 + channel_width * n).y();

  polygon.vertexes[3].x = pt_1.x();
  polygon.vertexes[3].y = pt_1.y();

  polygon.vertex_num = 4;
  polygon.shape = PolygonShape::box;
  UpdatePolygonValue(&polygon, NULL, 0, false, POLYGON_MAX_RADIUS);

  polygon.min_tangent_radius = 0.6;

  safe_check.SetObstacle(obstacle_manager_ptr_);
  if (safe_check.IsPolygonCollision(&polygon)) {
    ILOG_INFO << "passage is occupied";
    return true;
  }

  return false;
}

const bool ApaSlotManager::IsParallelSlotAndPassageAreaOccupied(
    const ApaSlot& slot) {
  const Eigen::Vector2d t = (slot.processed_corner_coord_global_.pt_0 -
                             slot.processed_corner_coord_global_.pt_2)
                                .normalized();
  const Eigen::Vector2d n(-t.y(), t.x());
  const double heading = std::atan2(n.y(), n.x());
  const Eigen::Vector2d pM02 = 0.5 * (slot.processed_corner_coord_global_.pt_0 +
                                      slot.processed_corner_coord_global_.pt_2);
  const Eigen::Vector2d pM13 = 0.5 * (slot.processed_corner_coord_global_.pt_1 +
                                      slot.processed_corner_coord_global_.pt_3);

  const Eigen::Vector2d origin_target_pos =
      pM02 + n * (apa_param.GetParam().rear_overhanging + 0.168);

  const std::vector<double> move_slot_dist_vec{0.0, 0.1, 0.2, 0.3};

  PathSafeChecker safe_check;
  const double lat_buffer = 0.1;
  const double lon_buffer = 0.05;
  double move_slot_dist = 0.0;
  bool is_slot_occupied = true;
  for (const double dist : move_slot_dist_vec) {
    const Eigen::Vector2d target_pos = origin_target_pos + dist * t;
    const Pose2D target_pose(target_pos.x(), target_pos.y(), heading);
    if (!safe_check.CalcEgoCollision(obstacle_manager_ptr_, target_pose,
                                     lat_buffer, lon_buffer)) {
      move_slot_dist = dist;
      ILOG_INFO << "move_slot_dist = " << move_slot_dist;
      is_slot_occupied = false;
      break;
    }
  }
  return is_slot_occupied;
}

}  // namespace apa_planner
}  // namespace planning