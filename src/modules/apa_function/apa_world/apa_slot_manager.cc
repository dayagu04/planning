#include "apa_slot_manager.h"

#include <cstddef>
#include <map>
#include <unordered_map>

#include "apa_param_config.h"
#include "apa_slot.h"
#include "apa_state_machine_manager.h"
#include "collision_detection/path_safe_checker.h"
#include "geometry_math.h"
#include "ifly_time.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {

static const uint8_t kSlotReleaseVoteCount = 6;
static const uint8_t kMaxSlotReleaseCount = 8;

void ApaSlotManager::Update(
    const LocalView* local_view,
    const std::shared_ptr<ApaStateMachineManager>& state_machine_ptr,
    const std::shared_ptr<ApaMeasureDataManager>& measure_data_ptr,
    const std::shared_ptr<ApaObstacleManager>& obstacle_manager_ptr,
    const std::shared_ptr<CollisionDetectorInterface>& col_det_interface_ptr) {
  if (local_view == nullptr || state_machine_ptr == nullptr ||
      measure_data_ptr == nullptr || obstacle_manager_ptr == nullptr ||
      col_det_interface_ptr == nullptr) {
    ILOG_ERROR << "Update ApaSlotManager, local_view_ptr is nullptr";
    return;
  }

  ILOG_INFO << "Update ApaSlotManager";
  measure_data_ptr_ = measure_data_ptr;
  state_machine_ptr_ = state_machine_ptr;
  obstacle_manager_ptr_ = obstacle_manager_ptr;
  col_det_interface_ptr_ = col_det_interface_ptr;

  slots_map_.clear();
  dist_id_map_.clear();
  ILOG_INFO << "parking_fusion_slot_lists_size = "
            << static_cast<int>(local_view->parking_fusion_info
                                    .parking_fusion_slot_lists_size);

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
      // If the selected slot id does not exist during the parking process,
      // simply reset and exit the parking
      if (slots_map_.count(local_view->parking_fusion_info.select_slot_id) ==
          0) {
        ILOG_INFO << "the selected slot disappear when parking";
        ego_info_under_slot_.Reset();
      }
    }
  }

  ILOG_INFO << "select slot id = " << ego_info_under_slot_.id
            << "  type = " << GetSlotTypeString(ego_info_under_slot_.slot_type);

  const SlotReleaseState last_geometry_release =
      ego_info_under_slot_.slot.release_info_
          .release_state[GEOMETRY_PLANNING_RELEASE];

  const SlotReleaseState last_astar_release =
      ego_info_under_slot_.slot.release_info_
          .release_state[ASTAR_PLANNING_RELEASE];

  if (slots_map_.count(ego_info_under_slot_.id) != 0 &&
      !ego_info_under_slot_.fix_slot) {
    ILOG_INFO << "Update selected slot";
    ego_info_under_slot_.slot = slots_map_[ego_info_under_slot_.id];
    ego_info_under_slot_.confidence =
        slots_map_[ego_info_under_slot_.id].confidence_;
  }

  // keep last release state here, and would change later when searching
  ego_info_under_slot_.slot.release_info_
      .release_state[GEOMETRY_PLANNING_RELEASE] = last_geometry_release;
  ego_info_under_slot_.slot.release_info_
      .release_state[ASTAR_PLANNING_RELEASE] = last_astar_release;
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
  const Pose2D ego =
      Pose2D(measure_data_ptr_->GetPos()[0], measure_data_ptr_->GetPos()[1],
             measure_data_ptr_->GetHeading());
  return col_det_interface_ptr_->GetPathSafeCheckPtr()->CalcEgoCollision(
      ego, 0.268, 0.1);
}

const bool ApaSlotManager::IsSlotCoarseRelease(const ApaSlot& slot) {
  if (slot.slot_type_ == SlotType::SLANT) {
    // 车尾泊入和车头泊入释放要求车位和自车的相对方向不一致
    // 因次不在此作基于规则释放 在规划器内部预规划时做是否释放
  }

  // 检查自车和通道内是否有障碍物
  bool is_obs_in_slot_passage_area = false;
  SlotReleaseVoterType release_voter_type = SlotReleaseVoterType::CLEAR;
  if ((slot.slot_type_ == SlotType::PERPENDICULAR ||
       slot.slot_type_ == SlotType::SLANT)) {
    release_voter_type = IsPerpendicularSlotAndPassageAreaOccupied(slot);
  } else if (slot.slot_type_ == SlotType::PARALLEL) {
    release_voter_type = IsParallelSlotAndPassageAreaOccupied(slot);
  }

  if (release_voter_type == SlotReleaseVoterType::CLEAR) {
    slot_release_voter_[slot.id_] = 0;
    ILOG_INFO << "obs is in slot_passage_area, slot id = " << slot.id_;
  } else if (release_voter_type == SlotReleaseVoterType::MAXIMUM) {
    slot_release_voter_[slot.id_] = kSlotReleaseVoteCount * 4;
  } else if (release_voter_type == SlotReleaseVoterType::ACCUMULATE) {
    if (slot_release_voter_.count(slot.id_) == 0) {
      slot_release_voter_[slot.id_] = 1;
    } else {
      if (slot_release_voter_[slot.id_] < kSlotReleaseVoteCount + 168) {
        slot_release_voter_[slot.id_]++;
      }
    }
  } else if (release_voter_type == SlotReleaseVoterType::SUBTRACT) {
    if (slot_release_voter_.count(slot.id_) != 0 &&
        slot_release_voter_[slot.id_] > 1) {
      slot_release_voter_[slot.id_]--;
    } else {
      slot_release_voter_[slot.id_] = 0;
    }
  } else if (release_voter_type == SlotReleaseVoterType::HOLD) {
    // do nothing
  } else {
    slot_release_voter_[slot.id_] = 0;
  }

  ILOG_INFO << "release_voter_type = "
            << GetSlotReleaseVoterTypeString(release_voter_type)
            << "  release_vote_number = "
            << static_cast<int>(slot_release_voter_[slot.id_]);

  if (slot_release_voter_.count(slot.id_) != 0 &&
      slot_release_voter_[slot.id_] >= kSlotReleaseVoteCount) {
    return true;
  }

  return false;
}

const SlotReleaseVoterType
ApaSlotManager::IsPerpendicularSlotAndPassageAreaOccupied(const ApaSlot& slot) {
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

  const std::vector<double> move_up_dist{0.68, 0.88, 1.08, 1.28, 1.48};
  // 产品定义是最大车辆宽度加0.4米释放  即单侧buffer 0.2米
  // 0.26米如果没有碰撞 直接释放
  // 0.20米如果没有碰撞 累加
  // 0.17米如果没有碰撞 维持不变
  // 0.16米如果没有碰撞 累减
  // 0.16米如果碰撞 直接不释放
  std::vector<std::pair<double, SlotReleaseVoterType>> lat_buffer_pair_vec = {
      {0.26, SlotReleaseVoterType::MAXIMUM},
      {0.20, SlotReleaseVoterType::ACCUMULATE},
      {0.17, SlotReleaseVoterType::HOLD},
      {0.16, SlotReleaseVoterType::SUBTRACT}};

  const double lon_buffer = 0.1;
  double move_slot_dist = 0.0;
  bool is_slot_occupied = true;
  std::pair<double, SlotReleaseVoterType> lat_buffer_pair;

  for (size_t i = 0; i < lat_buffer_pair_vec.size(); ++i) {
    lat_buffer_pair = lat_buffer_pair_vec[i];
    for (const double dist : move_slot_dist_vec) {
      bool col = false;
      for (const double up_dist : move_up_dist) {
        const Eigen::Vector2d origin_target_pos =
            pM01 - n * (param.wheel_base + param.front_overhanging - up_dist);

        const geometry_lib::PathPoint target_pose(origin_target_pos + dist * t,
                                                  heading);

        if (col_det_interface_ptr_->GetGJKCollisionDetectorPtr()
                ->Update(std::vector<geometry_lib::PathPoint>{target_pose},
                         lat_buffer_pair.first, 0.0, false, false)
                .col_flag) {
          col = true;
          break;
        }
      }
      if (col) {
        continue;
      }
      move_slot_dist = dist;
      is_slot_occupied = false;
      break;
    }
    if (!is_slot_occupied) {
      break;
    }
  }

  if (is_slot_occupied) {
    ILOG_INFO << "slot is occupied";
    return SlotReleaseVoterType::CLEAR;
  }

  ILOG_INFO << "move_slot_dist = " << move_slot_dist
            << "  lat_buffer = " << lat_buffer_pair.first << "  voter type = "
            << GetSlotReleaseVoterTypeString(lat_buffer_pair.second);

  const double lat_buffer = 0.0368;

  Eigen::Vector2d pt_0 = pM01 - t * (param.max_car_width * 0.5 + lat_buffer);
  Eigen::Vector2d pt_1 = pM01 + t * (param.max_car_width * 0.5 + lat_buffer);

  pt_0 = pt_0 + move_slot_dist * t;
  pt_1 = pt_1 + move_slot_dist * t;

  const double channel_width = param.slot_release_channel_width;
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

  if (col_det_interface_ptr_->GetGJKCollisionDetectorPtr()->IsPolygonCollision(
          polygon, false)) {
    ILOG_INFO << "passage is occupied";
    return SlotReleaseVoterType::CLEAR;
  }

  return lat_buffer_pair.second;
}

const SlotReleaseVoterType ApaSlotManager::IsParallelSlotAndPassageAreaOccupied(
    const ApaSlot& slot) {
  const auto& v_ego_heading = measure_data_ptr_->GetHeadingVec();
  Eigen::Vector2d n = (slot.processed_corner_coord_global_.pt_0 -
                       slot.processed_corner_coord_global_.pt_1)
                          .normalized();
  if (n.dot(v_ego_heading) < 1e-9) {
    n *= -1.0;
  }
  const double slot_heading = std::atan2(n.y(), n.x());

  const Eigen::Vector2d t(-n.y(), n.x());

  const Eigen::Vector2d center = slot.origin_corner_coord_global_.pt_center;

  const double slot_length = (slot.origin_corner_coord_global_.pt_0 -
                              slot.origin_corner_coord_global_.pt_1)
                                 .norm();

  const double center_to_rac_dist = 0.5 * apa_param.GetParam().car_length -
                                    apa_param.GetParam().rear_overhanging;

  const Eigen::Vector2d target_ego_pos =
      center - center_to_rac_dist * v_ego_heading;

  const std::vector<double> move_slot_dist_vec{-0.5, 0.0, 0.5};

  const double lat_buffer = 0.05;
  const double lon_buffer = 0.1;
  double move_slot_dist = 0.0;
  bool is_slot_occupied = true;
  for (const double dist : move_slot_dist_vec) {
    const Eigen::Vector2d target_pos = target_ego_pos + dist * t;
    const Pose2D target_pose(target_pos.x(), target_pos.y(), slot_heading);

    const bool is_collided =
        col_det_interface_ptr_->GetPathSafeCheckPtr()->CalcEgoCollision(
            target_pose, lat_buffer, lon_buffer);

    ILOG_INFO << "lateral moving dist = " << dist
              << ". target_pos= " << target_pose.GetX() << ", "
              << target_pose.GetY() << ", is_collided = " << is_collided;

    if (!is_collided) {
      move_slot_dist = dist;
      ILOG_INFO << "release slot with move_slot_dist = " << move_slot_dist;
      is_slot_occupied = false;
      break;
    }
  }
  ILOG_INFO << "final parallel slot is occupied = " << is_slot_occupied;

  if (is_slot_occupied) {
    return SlotReleaseVoterType::CLEAR;
  }
  return SlotReleaseVoterType::ACCUMULATE;
}

const std::string GetSlotReleaseVoterTypeString(
    const SlotReleaseVoterType release_voter_type) {
  std::string type;
  switch (release_voter_type) {
    case SlotReleaseVoterType::ACCUMULATE:
      type = "ACCUMULATE";
      break;
    case SlotReleaseVoterType::SUBTRACT:
      type = "SUBTRACT";
      break;
    case SlotReleaseVoterType::MAXIMUM:
      type = "MAXIMUM";
      break;
    case SlotReleaseVoterType::CLEAR:
      type = "CLEAR";
      break;
    default:
      type = "CLEAR";
      break;
  }
  return type;
}

const bool ApaSlotManager::IsTargetSlotReleaseByRule() const {
  if (ego_info_under_slot_.slot.release_info_
          .release_state[RULE_BASED_RELEASE] == SlotReleaseState::RELEASE) {
    return true;
  }

  return false;
}

}  // namespace apa_planner
}  // namespace planning