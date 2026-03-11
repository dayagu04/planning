#include "apa_slot_manager.h"

#include <cmath>
#include <cstddef>
#include <map>
#include <unordered_map>
#include <vector>

#include "apa_param_config.h"
#include "apa_slot.h"
#include "apa_state_machine_manager.h"
#include "collision_detection/path_safe_checker.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "ifly_time.h"
#include "library/convex_collision_detection/aabb2d.h"
#include "log_glog.h"
#include "target_pose_decider/target_pose_decider.h"
#include "time_benchmark.h"

namespace planning {
namespace apa_planner {

static const int kSlotFreeId = 1;
static const int kSlotInvalidId = -1000;  // 融合输出车位 id 不会为负
static const int kEgoSlotInvalidId = 0;   // 自车车位id无效时，融合会发 0.
static const uint8_t kSlotReleaseVoteCount = 6;
static const uint8_t kMaxSlotReleaseCount = 8;
static const double kMaxEgoSlotAbsoluteDist = 6.86;
static const double kMaxEgoSlotRelativeDist = 3.86;
static const int kMaxSlotObserveFrameCount = 30;

void ApaSlotManager::Update(
    const LocalView* local_view,
    const iflyauto::PlanningOutput* planning_output,
    const std::shared_ptr<ApaStateMachineManager>& state_machine_ptr,
    const std::shared_ptr<ApaMeasureDataManager>& measure_data_ptr,
    const std::shared_ptr<ApaObstacleManager>& obstacle_manager_ptr,
    const std::shared_ptr<CollisionDetectorInterface>& col_det_interface_ptr) {
  if (local_view == nullptr || planning_output == nullptr ||
      state_machine_ptr == nullptr || measure_data_ptr == nullptr ||
      obstacle_manager_ptr == nullptr || col_det_interface_ptr == nullptr) {
    ILOG_ERROR << "Update ApaSlotManager, local_view_ptr is nullptr";
    return;
  }

  double start_time = IflyTime::Now_ms();

  ILOG_INFO << "Update ApaSlotManager";
  measure_data_ptr_ = measure_data_ptr;
  state_machine_ptr_ = state_machine_ptr;
  obstacle_manager_ptr_ = obstacle_manager_ptr;
  col_det_interface_ptr_ = col_det_interface_ptr;

  slots_map_.clear();
  dist_id_map_.clear();
  if (!perpendicular_redefine_info_map_.empty() && state_machine_ptr_->IsSearchingInStatus()) {
    perpendicular_redefine_info_map_.clear();
  }

  ApaRunningMode running_mode = state_machine_ptr->GetParkRunningMode();

  const size_t slot_size =
      local_view->parking_fusion_info.parking_fusion_slot_lists_size;
  size_t select_slot_id = local_view->parking_fusion_info.select_slot_id;
  size_t ego_slot_id = local_view->parking_fusion_info.ego_slot_id;

  const auto is_sapa_mode = state_machine_ptr->IsSAPAMode();
  const auto sapa_status = state_machine_ptr->GetSAPAStatus();
  ILOG_INFO << "is_sapa_mode_ : " << is_sapa_mode << " sapa_status : "
            << ApaStateMachineManager::GetParkingSAPAStatusString(sapa_status);
  ILOG_INFO << "parking_fusion slot size = " << slot_size
            << "  select slot id = " << select_slot_id;

  const Eigen::Vector2d car_mirror_pos =
      0.5 * (measure_data_ptr->GetLeftMirrorPos() +
             measure_data_ptr->GetRightMirrorPos());

  for (size_t i = 0; i < slot_size; ++i) {
    const iflyauto::ParkingFusionSlot& fusion_slot =
        local_view->parking_fusion_info.parking_fusion_slot_lists[i];
    if (fusion_slot.id != ApaStateMachineManager::kSlotFreeIdx_ &&
        is_sapa_mode) {
      continue;
    }
    ApaSlot slot(fusion_slot);
    if (state_machine_ptr_->IsSearchingInStatus()) {
      if (IsSideParkingPerpendicularSlot(slot)) {
        slot.ResetAsParallel(
            fusion_slot,
            perpendicular_redefine_info_map_.at(slot.GetId()).first,
            perpendicular_redefine_info_map_.at(slot.GetId()).second);
      }
    } else if (state_machine_ptr_->IsParkingInStatus() &&
               select_slot_id == slot.GetId()) {
      if (perpendicular_redefine_info_map_.find(slot.GetId()) !=
          perpendicular_redefine_info_map_.end())
        slot.ResetAsParallel(
            fusion_slot,
            perpendicular_redefine_info_map_.at(slot.GetId()).first,
            perpendicular_redefine_info_map_.at(slot.GetId()).second);
    }
    if (is_sapa_mode && slot.GetId() == ApaStateMachineManager::kSlotFreeIdx_) {
      slot.SetSourceType(SlotSourceType::SELF_DEFINE);
    }

    if (slot.GetId() == select_slot_id) {
      slot.is_selected_ = true;
    }

    const SlotCoord& slot_global_coord = slot.GetOriginCornerCoordGlobal();
    const double dist = (car_mirror_pos - slot_global_coord.pt_01_mid).norm();

    dist_id_map_[dist] = slot.GetId();
    slots_map_[slot.GetId()] = slot;

    const double ego_dist =
        geometry_lib::CalPoint2LineDist(car_mirror_pos, slot.GetMidLine());
    if (ego_slot_min_dist_map_.count(slot.GetId()) != 0) {
      ego_slot_min_dist_map_[slot.GetId()] =
          std::min(ego_slot_min_dist_map_[slot.GetId()], ego_dist);
    } else {
      ego_slot_min_dist_map_[slot.GetId()] = ego_dist;
    }
  }

  // 确定待泊入/泊出目标车位的 id
  if (is_sapa_mode) {
    if (sapa_status != ApaSAPAStatus::SAPA_STATUS_FINISHED) {
      select_slot_id = kSlotInvalidId;
    } else if (slots_map_.find(ApaStateMachineManager::kSlotFreeIdx_) ==
               slots_map_.end()) {
      ILOG_ERROR << "SAPA mode, but free slot id is not in slot map";
      select_slot_id = kSlotInvalidId;
    } else {
      select_slot_id = ApaStateMachineManager::kSlotFreeIdx_;
    }
  } else if (state_machine_ptr_->IsPAMode()) {
    // TODO(taolu10):
    // 一件贴边：需要区分状态，对于确定贴边方向后，需要基于方向确认 id
    if (slots_map_.empty()) {
      select_slot_id = kSlotInvalidId;
    } else {
      select_slot_id = slots_map_.begin()->second.GetId();
    }
  } else {
    if (state_machine_ptr_->IsParkOutStatus()) {
      select_slot_id = ego_slot_id;
    }
  }

  // 更新 ego_info_under_slot
  if (state_machine_ptr_->IsSeachingStatus()) {
    if (select_slot_id == kSlotInvalidId ||
        slots_map_.find(select_slot_id) == slots_map_.end()) {
      ego_info_under_slot_.Reset();
    } else {
      auto& slot = slots_map_.at(select_slot_id);
      ego_info_under_slot_.history_id = ego_info_under_slot_.id;
      ego_info_under_slot_.history_slot_type = ego_info_under_slot_.slot_type;
      ego_info_under_slot_.id = select_slot_id;
      ego_info_under_slot_.slot_type = slot.slot_type_;
    }
  } else if (state_machine_ptr->IsParkingInStatus()) {
    if (slots_map_.count(ego_info_under_slot_.id) == 0) {
      ILOG_INFO << "the selected slot disappear when parking";
      ego_info_under_slot_.slot_disappear_flag = true;
      if (measure_data_ptr_->GetStaticFlag()) {
        ILOG_INFO << "car is static, reset ego_info_under_slot";
        ego_info_under_slot_.Reset();
      }
    } else {
      ego_info_under_slot_.slot_disappear_flag = false;
    }
  } else {  // ParkingOutStatus
  }

  // 更新基于规则的车位释放
  if (state_machine_ptr_->IsSeachingStatus() ||
      state_machine_ptr_->IsManualStatus()) {
    if (is_sapa_mode && sapa_status != ApaSAPAStatus::SAPA_STATUS_FINISHED) {
      // TODO(taolu10): 确认这部分逻辑的合理性
      for (int i = 0; i < SLOT_RELEASE_METHOD_MAX_NUM; ++i) {
        ego_info_under_slot_.slot.release_info_.release_state[i] =
            SlotReleaseState::NOT_RELEASE;
      }
    }
    if (state_machine_ptr_->IsSearchingInStatus()) {
      if (measure_data_ptr->GetFoldMirrorFlag()) {
        col_det_interface_ptr_->Init(true);
      } else {
        if (apa_param.GetParam()
                .smart_fold_mirror_params.has_smart_fold_mirror) {
          col_det_interface_ptr_->Init(true);
        } else {
          col_det_interface_ptr_->Init(false);
        }
      }
      ParkingLotCruiseProcess();
    }
    if (state_machine_ptr_->IsSeachingOutStatus() &&
        select_slot_id != kEgoSlotInvalidId) {
      ApaSlot& slot = slots_map_[ego_info_under_slot_.id];
      ego_info_under_slot_.relative_direction_between_ego_and_slot =
          measure_data_ptr_->GetHeadingVec().dot(
              slot.GetOriginCornerCoordGlobal().pt_23mid_01mid_unit_vec);
      slot.release_info_.release_state[RULE_BASED_RELEASE] =
          SlotReleaseState::RELEASE;
    }
  }

  // 更新泊出推荐
  recommend_park_out_ =
      GetRecommendApaParkingOperationType(local_view->parking_fusion_info);

  if (ego_info_under_slot_.slot.GetType() == SlotType::PARALLEL) {
    const iflyauto::ParkingFusionSlot* fusion_slot;
    ego_info_under_slot_.neigbor_rear_heading = -100.0;
    ego_info_under_slot_.neigbor_front_heading = -100.0;
    double neigbor_front_heading_obs = -100.0;
    double neigbor_front_heading_slot = -100.0;
    for (size_t i = 0; i < slot_size; ++i) {
      fusion_slot =
          &local_view->parking_fusion_info.parking_fusion_slot_lists[i];
      if (fusion_slot->id == ego_info_under_slot_.id) {
        break;
      }
    }

    Eigen::Vector3d ego_pose;
    const iflyauto::IFLYLocalization& localization_info =
        local_view->localization;
    ego_pose[0] = localization_info.position.position_boot.x;
    ego_pose[1] = localization_info.position.position_boot.y;
    ego_pose[2] = localization_info.orientation.euler_boot.yaw;

    const std::array<double, 4> d_per_edge = {1.0, 2.5, 1.0, 2.5};
    for (size_t i = 0; i < slot_size; ++i) {
      const iflyauto::ParkingFusionSlot* fusion_slot1 =
          &local_view->parking_fusion_info.parking_fusion_slot_lists[i];
      std::array<Eigen::Vector2d, 4> slot_vertexs = {
          Eigen::Vector2d(fusion_slot1->corner_points[0].x,
                          fusion_slot1->corner_points[0].y),
          Eigen::Vector2d(fusion_slot1->corner_points[2].x,
                          fusion_slot1->corner_points[2].y),
          Eigen::Vector2d(fusion_slot1->corner_points[1].x,
                          fusion_slot1->corner_points[1].y),
          Eigen::Vector2d(fusion_slot1->corner_points[3].x,
                          fusion_slot1->corner_points[3].y),

      };
      Eigen::Vector2d slot_center{0.0, 0.0};
      for (auto pts : slot_vertexs) {
        slot_center += pts;
      }
      slot_center /= 4;
      auto obs_ego_line_heading = std::atan2(slot_center.y() - ego_pose[1],
                                             slot_center.x() - ego_pose[0]);
      bool is_front_slot =
          std::abs(obs_ego_line_heading - ego_pose[2]) < M_PI / 2;
      if (fusion_slot1->id == ego_info_under_slot_.id) {
        continue;
      }
      auto res = ApaObstacleManager::CheckParaSlotObsPtsAreNeighbour(
          slot_vertexs, fusion_slot, d_per_edge, ego_pose);
      if (is_front_slot && res.first == 0) {
        neigbor_front_heading_slot = res.second;
        ILOG_INFO << "neigbor_front_heading_slot first"
                  << neigbor_front_heading_slot;
        if (neigbor_front_heading_slot > 2 * M_PI ||
            neigbor_front_heading_slot < -2 * M_PI) {
          neigbor_front_heading_slot = 0.0;
        } else if (neigbor_front_heading_slot > M_PI_2) {
          neigbor_front_heading_slot = -M_PI + neigbor_front_heading_slot;
        } else if (neigbor_front_heading_slot < -M_PI_2) {
          neigbor_front_heading_slot = M_PI + neigbor_front_heading_slot;
        }
        if (std::abs(neigbor_front_heading_slot) < pnc::mathlib::Deg2Rad(5.0) ||
            std::abs(neigbor_front_heading_slot) >
                pnc::mathlib::Deg2Rad(45.0)) {
          neigbor_front_heading_slot = 0.0;
        }
        if (slots_map_.count(fusion_slot1->id)) {
          ego_info_under_slot_.slot.front_slot_limiter_ =
              slots_map_[fusion_slot1->id].GetLimiter();
        }
      }
    }
    auto its = obstacle_manager_ptr_->GetParallelSlotNeighbourObjsHeading();
    if (its[0] != -100.0) {
      neigbor_front_heading_obs = its[0];
      ILOG_INFO << "neigbor_front_heading_obs first"
                << neigbor_front_heading_obs;

      if (neigbor_front_heading_obs > 2 * M_PI ||
          neigbor_front_heading_obs < -2 * M_PI) {
        neigbor_front_heading_obs = 0.0;
      } else if (neigbor_front_heading_obs > M_PI_2) {
        neigbor_front_heading_obs = -M_PI + neigbor_front_heading_obs;
      } else if (neigbor_front_heading_obs < -M_PI_2) {
        neigbor_front_heading_obs = M_PI + neigbor_front_heading_obs;
      }
      if (std::abs(neigbor_front_heading_obs) < pnc::mathlib::Deg2Rad(5.0) ||
          std::abs(neigbor_front_heading_obs) > pnc::mathlib::Deg2Rad(45.0)) {
        neigbor_front_heading_obs = 0.0;
      }
    }
    if (its[1] != -100.0) {
      ego_info_under_slot_.neigbor_rear_heading = its[1];
    }
    if (neigbor_front_heading_obs > 2 * M_PI ||
        neigbor_front_heading_obs < -2 * M_PI) {
      ILOG_INFO << "neigbor_front_heading_obs before"
                << neigbor_front_heading_obs;
      neigbor_front_heading_obs = 0.0;
    }
    if (neigbor_front_heading_slot > 2 * M_PI ||
        neigbor_front_heading_slot < -2 * M_PI) {
      neigbor_front_heading_slot = 0.0;
      ILOG_INFO << "neigbor_front_heading_slot before"
                << neigbor_front_heading_slot;
    }
    if (std::abs(neigbor_front_heading_obs) >
        std::abs(neigbor_front_heading_slot)) {
      ego_info_under_slot_.neigbor_front_heading = neigbor_front_heading_obs;
      ILOG_INFO << "ego_info_under_slot_.neigbor_front_heading use front "
                   "obs heading"
                << ego_info_under_slot_.neigbor_front_heading;
    } else {
      ego_info_under_slot_.neigbor_front_heading = neigbor_front_heading_slot;
      ILOG_INFO << "ego_info_under_slot_.neigbor_front_heading use front "
                   "slot heading"
                << ego_info_under_slot_.neigbor_front_heading;
    }

    ILOG_INFO << "ego_info_under_slot_.neigbor_front_heading = "
              << ego_info_under_slot_.neigbor_front_heading;
  }

  ILOG_INFO << "select slot id = " << ego_info_under_slot_.id
            << ", history_slot_id = " << ego_info_under_slot_.history_id
            << "  type = " << GetSlotTypeString(ego_info_under_slot_.slot_type)
            << "  history_slot_type = "
            << GetSlotTypeString(ego_info_under_slot_.history_slot_type);

  if (state_machine_ptr->IsSeachingStatus()) {
    ego_info_under_slot_.fix_slot = false;
    ego_info_under_slot_.fix_limiter = false;
  }

  const SlotReleaseState last_geometry_release =
      ego_info_under_slot_.slot.release_info_
          .release_state[GEOMETRY_PLANNING_RELEASE];

  const SlotReleaseState last_astar_release =
      ego_info_under_slot_.slot.release_info_
          .release_state[ASTAR_PLANNING_RELEASE];

  if (!measure_data_ptr_->GetStaticFlag() ||
      (is_sapa_mode && sapa_status != ApaSAPAStatus::SAPA_STATUS_FINISHED) ||
      running_mode == ApaRunningMode::RUNNING_PA) {
    pre_plan_fail_slot_id_uset_.clear();
  } else if (state_machine_ptr->IsSearchingInStatus() &&
             ego_info_under_slot_.slot.GetId() != 0) {
    if ((last_geometry_release == SlotReleaseState::UNKNOWN ||
         last_geometry_release == SlotReleaseState::NOT_RELEASE) &&
        (last_astar_release == SlotReleaseState::UNKNOWN ||
         last_astar_release == SlotReleaseState::NOT_RELEASE)) {
      pre_plan_fail_slot_id_uset_.emplace(ego_info_under_slot_.slot.GetId());
    }
  }

  if (!(apa_param.GetParam()
            .smart_fold_mirror_params.locked_obs_slot_with_fold_mirror &&
        planning_output->rear_view_mirror_signal_command.available &&
        planning_output->rear_view_mirror_signal_command
                .rear_view_mirror_value == iflyauto::REAR_VIEW_MIRROR_FOLD)) {
    if (slots_map_.count(ego_info_under_slot_.id) != 0 &&
        !ego_info_under_slot_.fix_slot) {
      ILOG_INFO << "Update selected slot";
      ego_info_under_slot_.slot = slots_map_[ego_info_under_slot_.id];
      ego_info_under_slot_.confidence =
          slots_map_[ego_info_under_slot_.id].confidence_;
    }
  }

  // keep last release state here, and would change later when searching
  if (ego_info_under_slot_.id == ego_info_under_slot_.history_id &&
      ego_info_under_slot_.slot_type ==
          ego_info_under_slot_.history_slot_type) {
    ego_info_under_slot_.slot.release_info_
        .release_state[GEOMETRY_PLANNING_RELEASE] = last_geometry_release;
    ego_info_under_slot_.slot.release_info_
        .release_state[ASTAR_PLANNING_RELEASE] = last_astar_release;
  }

  JSON_DEBUG_VALUE("total_slot_size", slots_map_.size())

  TimeBenchmark::Instance().SetTime(TimeBenchmarkType::TB_APA_SLOT_MANAGER_TIME,
                                    IflyTime::Now_ms() - start_time);

  return;
}

void ApaSlotManager::GenerateReleaseSlotIdVec() {
  if (!state_machine_ptr_->IsSeachingStatus() &&
      !state_machine_ptr_->IsManualStatus()) {
    return;
  }
  release_slot_id_vec_.clear();
  release_slot_narrow_flag_vec_.clear();
  release_slot_id_vec_.reserve(slots_map_.size());
  release_slot_narrow_flag_vec_.reserve(slots_map_.size());

  if (state_machine_ptr_->IsSAPAMode() &&
      state_machine_ptr_->GetSAPAStatus() !=
          ApaSAPAStatus::SAPA_STATUS_FINISHED) {
    return;
  }

  for (const auto& pair : dist_id_map_) {
    if (slots_map_.count(pair.second) == 0) {
      continue;
    }
    const ApaSlot& slot = slots_map_[pair.second];

    if (slot.release_info_.release_state[RULE_BASED_RELEASE] !=
        SlotReleaseState::RELEASE) {
      continue;
    }

    if (!state_machine_ptr_->IsSAPAMode() &&
        pre_plan_fail_slot_id_uset_.count(slot.id_) != 0) {
      continue;
    }

    bool is_slot_release = false;

    if (slot.id_ != ego_info_under_slot_.id) {
      is_slot_release = true;
    } else {
      const SlotReleaseInfo& ego_release_info =
          ego_info_under_slot_.slot.release_info_;
      const SlotReleaseState& geometry_release_state =
          ego_release_info.release_state[GEOMETRY_PLANNING_RELEASE];
      const SlotReleaseState& astar_release_state =
          ego_release_info.release_state[ASTAR_PLANNING_RELEASE];
      if (geometry_release_state == SlotReleaseState::COMPUTING ||
          geometry_release_state == SlotReleaseState::RELEASE ||
          astar_release_state == SlotReleaseState::COMPUTING ||
          astar_release_state == SlotReleaseState::RELEASE) {
        is_slot_release = true;
      }
    }

    if (is_slot_release) {
      release_slot_id_vec_.emplace_back(slot.id_);
      release_slot_narrow_flag_vec_.emplace_back(slot.is_narrow_slot_);
    }
  }
}

void ApaSlotManager::ParkingLotCruiseProcess() {
  const double time_start = IflyTime::Now_ms();

  // const bool is_ego_collision = IsEgoCloseToObs();

  is_ego_col_vertical_ = true;
  ego_col_safe_lat_buffer_ = 0.268;
  ego_col_safe_lon_buffer_ = 0.15;
  const std::vector<double> lat_buffer_vec{0.22, 0.1};
  const std::vector<double> lon_buffer_vec{0.14, 0.06};
  for (const double lon_buffer : lon_buffer_vec) {
    for (const double lat_buffer : lat_buffer_vec) {
      is_ego_col_vertical_ =
          IsEgoCloseToObs(lat_buffer, lat_buffer, lon_buffer);
      if (!is_ego_col_vertical_) {
        ego_col_safe_lon_buffer_ = lon_buffer;
        ego_col_safe_lat_buffer_ = lat_buffer;
        break;
      }
    }
    if (!is_ego_col_vertical_) {
      break;
    }
  }

  ILOG_INFO << "is_ego_col_vertical_ = " << is_ego_col_vertical_
            << "  ego_col_safe_lat_buffer_ = " << ego_col_safe_lat_buffer_
            << "  ego_col_safe_lon_buffer_ = " << ego_col_safe_lon_buffer_;

  is_ego_col_parallel_ = IsEgoCloseToObs(0.1, 0.01, 0.1);

  uint8_t release_slot_count = 0;

  for (const auto& dist_id : dist_id_map_) {
    ApaSlot& slot = slots_map_[dist_id.second];
    ILOG_INFO << "slot id = " << slot.GetId()
              << "  type = " << GetSlotTypeString(slot.GetType())
              << "  fusion_release = "
              << GetSlotReleaseStateString(
                     slot.release_info_.release_state[FUSION_RELEASE]);

    if (slot.release_info_.release_state[FUSION_RELEASE] ==
        SlotReleaseState::NOT_RELEASE) {
      ILOG_INFO << "NOT_RELEASE reason: fusion not release";
      continue;
    }

    // if (is_ego_collision) {
    //   slot.release_info_.release_state[RULE_BASED_RELEASE] =
    //       SlotReleaseState::NOT_RELEASE;
    //   continue;
    // }

    if (release_slot_count >= kMaxSlotReleaseCount) {
      slot.release_info_.release_state[RULE_BASED_RELEASE] =
          SlotReleaseState::NOT_RELEASE;
      ILOG_INFO << "NOT_RELEASE reason: over max released size "
                << static_cast<int>(kMaxSlotReleaseCount);
      continue;
    }

    if (ego_slot_min_dist_map_.count(slot.GetId()) != 0 &&
        slot.GetType() != SlotType::PARALLEL &&
        ego_slot_min_dist_map_[slot.GetId()] > kMaxEgoSlotRelativeDist) {
      slot.release_info_.release_state[RULE_BASED_RELEASE] =
          SlotReleaseState::NOT_RELEASE;
      ILOG_INFO << "NOT_RELEASE reason: ego slot dist over "
                << kMaxEgoSlotRelativeDist << " m!";
      continue;
    }

    if (dist_id.first > kMaxEgoSlotAbsoluteDist &&
        slot.GetType() != SlotType::PARALLEL) {
      slot.release_info_.release_state[RULE_BASED_RELEASE] =
          SlotReleaseState::NOT_RELEASE;
      ILOG_INFO << "NOT_RELEASE reason: nearest slot dist over "
                << kMaxEgoSlotAbsoluteDist << " m!";
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
    ILOG_INFO << "RULE_BASED_RELEASE !";
  }

  ILOG_INFO << "apa lot cruise consume time = "
            << IflyTime::Now_ms() - time_start << " ms";

  return;
}

const bool ApaSlotManager::IsEgoCloseToObs(const double body_lat_buffer,
                                           const double mirror_lat_buffer,
                                           const double lon_buffer) {
  const Pose2D ego =
      Pose2D(measure_data_ptr_->GetPos()[0], measure_data_ptr_->GetPos()[1],
             measure_data_ptr_->GetHeading());

  return col_det_interface_ptr_->GetPathSafeCheckPtr()->CalcEgoCollision(
      ego, body_lat_buffer, lon_buffer, false);
}

const bool ApaSlotManager::IsSlotCoarseRelease(ApaSlot& slot) {
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
ApaSlotManager::IsPerpendicularSlotAndPassageAreaOccupied(ApaSlot& slot) {
  if (is_ego_col_vertical_) {
    return SlotReleaseVoterType::CLEAR;
  }

  const ApaParameters& param = apa_param.GetParam();
  const Eigen::Vector2d pM01 = slot.origin_corner_coord_global_.pt_01_mid;
  const Eigen::Vector2d pM23 = slot.origin_corner_coord_global_.pt_23_mid;
  const Eigen::Vector2d t = slot.origin_corner_coord_global_.pt_01_unit_vec;
  const Eigen::Vector2d n =
      slot.origin_corner_coord_global_.pt_23mid_01mid_unit_vec;

  const Eigen::Vector2d pt_23_unit_vec =
      slot.origin_corner_coord_global_.pt_23_unit_vec;

  const Eigen::Vector2d pt_01_unit_vec =
      slot.origin_corner_coord_global_.pt_01_unit_vec;

  const Eigen::Vector2d pM23_ego = measure_data_ptr_->GetPos() - pM23;
  const Eigen::Vector2d pM01_ego = measure_data_ptr_->GetPos() - pM01;
  if (geometry_lib::GetCrossFromTwoVec2d(pt_23_unit_vec, pM23_ego) > 0.0) {
    // area 3
    ILOG_INFO << "Ego pos in area 3 relative to slot (id: " << slot.id_ << ")";
    return SlotReleaseVoterType::CLEAR;
  } else if (geometry_lib::GetCrossFromTwoVec2d(pt_01_unit_vec, pM01_ego) <
             0.0) {
    // area 1
  } else {
    // area 2
    const double heading_err_deg =
        std::fabs(measure_data_ptr_->GetHeading() - slot.slot_heading_) *
        kRad2Deg;
    if (mathlib::IsInBound(heading_err_deg, 30.0, 150.0)) {
      ILOG_INFO << "Ego pos in area 2 relative to slot (id: " << slot.id_
                << "), heading error between 30 and 150 degrees";
      return SlotReleaseVoterType::CLEAR;
    }
  }

  const auto& slot_release_buffer = param.lat_lon_slot_release_buffer;

  const std::vector<double> lat_body_buffer_vec{
      slot_release_buffer.maximum_lat_body_buffer,
      slot_release_buffer.accumulate_lat_body_buffer,
      slot_release_buffer.hold_lat_body_buffer,
      slot_release_buffer.subtract_lat_body_buffer};

  const std::vector<double> lat_mirror_buffer_vec{
      slot_release_buffer.maximum_lat_mirror_buffer,
      slot_release_buffer.accumulate_lat_mirror_buffer,
      slot_release_buffer.hold_lat_mirror_buffer,
      slot_release_buffer.subtract_lat_mirror_buffer};

  TargetPoseDecider tar_pose_decider(col_det_interface_ptr_);
  TargetPoseDeciderRequest tar_pose_decider_request(
      lat_body_buffer_vec, lat_mirror_buffer_vec,
      slot_release_buffer.lon_buffer,
      ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN, true, false,
      ApaSlotLatPosPreference::MID, true);

  TargetPoseDeciderResult res =
      tar_pose_decider.CalcTargetPose(slot, tar_pose_decider_request);

  if (res.target_pose_type == TargetPoseType::FAIL) {
    ILOG_INFO << "slot is occupied";
    return SlotReleaseVoterType::CLEAR;
  }

  ILOG_INFO << "lat_move_slot_dist = " << res.safe_lat_move_dist
            << "  lon_move_slot_dist = " << res.safe_lon_move_dist
            << "  lat_body_buffer = " << res.safe_lat_body_buffer
            << "  lat_mirror_buffer = " << res.safe_lat_mirror_buffer;

  if (res.safe_lat_body_buffer <
      slot_release_buffer.maximum_lat_body_buffer - 1e-3) {
    slot.is_narrow_slot_ = true;
  }

  SlotReleaseVoterType release_voter_type;
  if (ego_col_safe_lon_buffer_ < 0.12) {
    release_voter_type = SlotReleaseVoterType::SUBTRACT;
  } else if (ego_col_safe_lat_buffer_ < 0.20) {
    release_voter_type = SlotReleaseVoterType::SUBTRACT;
  } else if (res.exceed_allow_max_dx > 0.02) {
    release_voter_type = SlotReleaseVoterType::SUBTRACT;
  } else if (geometry_lib::IsTwoNumerEqual(
                 slot_release_buffer.maximum_lat_body_buffer,
                 res.safe_lat_body_buffer)) {
    release_voter_type = SlotReleaseVoterType::MAXIMUM;
  } else if (geometry_lib::IsTwoNumerEqual(
                 slot_release_buffer.accumulate_lat_body_buffer,
                 res.safe_lat_body_buffer)) {
    release_voter_type = SlotReleaseVoterType::ACCUMULATE;
  } else if (geometry_lib::IsTwoNumerEqual(
                 slot_release_buffer.hold_lat_body_buffer,
                 res.safe_lat_body_buffer)) {
    release_voter_type = SlotReleaseVoterType::HOLD;
  } else if (geometry_lib::IsTwoNumerEqual(
                 slot_release_buffer.subtract_lat_body_buffer,
                 res.safe_lat_body_buffer)) {
    release_voter_type = SlotReleaseVoterType::SUBTRACT;
  }

  ILOG_INFO << "release_voter_type = "
            << GetSlotReleaseVoterTypeString(release_voter_type);

  Polygon2D polygon;
  polygon.FillTangentCircleParams(std::vector<Eigen::Vector2d>{
      pM01 + 2.0 * n, pM01 + slot.slot_width_ * t + 2.0 * n,
      pM01 + slot.slot_width_ * t - 2.0 * n, pM01 - 2.0 * n});

  const bool left_empty =
      !col_det_interface_ptr_->GetGJKColDetPtr()->IsPolygonCollision(
          polygon, GJKColDetRequest(false));

  polygon.FillTangentCircleParams(std::vector<Eigen::Vector2d>{
      pM01 + 2.0 * n, pM01 - 2.0 * n, pM01 - slot.slot_width_ * t - 2.0 * n,
      pM01 - slot.slot_width_ * t + 2.0 * n});

  const bool right_empty =
      !col_det_interface_ptr_->GetGJKColDetPtr()->IsPolygonCollision(
          polygon, GJKColDetRequest(false));

  double channel_width = slot_release_buffer.channel_width;

  if (release_voter_type == SlotReleaseVoterType::MAXIMUM) {
    channel_width = slot_release_buffer.channel_width - 0.2;
  } else if (release_voter_type == SlotReleaseVoterType::HOLD) {
    channel_width = slot_release_buffer.channel_width + 0.2;
  }

  if (left_empty && right_empty) {
    channel_width = slot_release_buffer.two_side_empty_channel_width;
  } else if (left_empty || right_empty) {
    channel_width = slot_release_buffer.one_side_empty_channel_width;
  }

  const double lat_buffer = slot_release_buffer.channel_lat_offset;

  Eigen::Vector2d pt_0 = pM01 - t * (param.car_width * 0.5 + lat_buffer);
  Eigen::Vector2d pt_1 = pM01 + t * (param.car_width * 0.5 + lat_buffer);

  pt_0 = pt_0 + res.safe_lat_move_dist * t;
  pt_1 = pt_1 + res.safe_lat_move_dist * t;

  double safe_channel_width = 0.0;
  const std::vector<double> channel_width_vec{channel_width,
                                              channel_width - 0.2};

  for (const double channel : channel_width_vec) {
    polygon.FillTangentCircleParams(std::vector<Eigen::Vector2d>{
        pt_0, pt_0 + channel * n, pt_1 + channel * n, pt_1});
    if (!col_det_interface_ptr_->GetGJKColDetPtr()->IsPolygonCollision(
            polygon, GJKColDetRequest(false))) {
      safe_channel_width = channel;
      break;
    }
  }

  if (safe_channel_width > channel_width_vec[0] - 0.01) {
    return release_voter_type;
  } else if (safe_channel_width > channel_width_vec[1] - 0.01) {
    ILOG_INFO << "passage is not enough";
    return SlotReleaseVoterType::SUBTRACT;
  } else {
    ILOG_INFO << "passage is occupied";
    return SlotReleaseVoterType::CLEAR;
  }
}

const SlotReleaseVoterType ApaSlotManager::IsParallelSlotAndPassageAreaOccupied(
    ApaSlot& slot) {
  if (is_ego_col_parallel_) {
    return SlotReleaseVoterType::CLEAR;
  }
  const auto& v_ego_heading = measure_data_ptr_->GetHeadingVec();

  SlotCoord corrected_global_slot = slot.processed_corner_coord_global_;

  Eigen::Vector2d v_10 =
      (corrected_global_slot.pt_0 - corrected_global_slot.pt_1).normalized();

  if (v_10.dot(v_ego_heading) < 1e-9) {
    v_10 *= -1.0;
    corrected_global_slot.pt_0.swap(corrected_global_slot.pt_1);
    corrected_global_slot.pt_2.swap(corrected_global_slot.pt_3);
  }
  corrected_global_slot.CalExtraCoord();
  const double slot_heading = std::atan2(v_10.y(), v_10.x());
  const Eigen::Vector2d n(-v_10.y(), v_10.x());

  const pnc::geometry_lib::LineSegment line_01(
      corrected_global_slot.pt_1, corrected_global_slot.pt_0, slot_heading);

  const double dist_01_2 =
      pnc::geometry_lib::CalPoint2LineDist(corrected_global_slot.pt_2, line_01);

  const double dist_01_3 =
      pnc::geometry_lib::CalPoint2LineDist(corrected_global_slot.pt_3, line_01);

  const double slot_width = std::min(dist_01_2, dist_01_3);
  const double slot_length =
      (corrected_global_slot.pt_0 - corrected_global_slot.pt_1).norm();

  const Eigen::Vector2d slot_13_mid =
      0.5 * (corrected_global_slot.pt_1 + corrected_global_slot.pt_3);
  const Eigen::Vector2d slot_02_mid =
      0.5 * (corrected_global_slot.pt_0 + corrected_global_slot.pt_2);

  geometry_lib::GlobalToLocalTf g2l_tf(slot_13_mid, slot_heading);
  geometry_lib::LocalToGlobalTf l2g_tf(slot_13_mid, slot_heading);

  // cacl loc
  double target_x = 0.5 * (slot_length - apa_param.GetParam().car_length) +
                    apa_param.GetParam().rear_overhanging;

  bool is_limiter_rear = true;
  const auto& limiter = slot.GetLimiter();
  ILOG_INFO << "limiter.valid = " << limiter.valid;
  if (limiter.valid) {
    // transfer limiter in slot coordination
    const auto limiter_start_pt_local = g2l_tf.GetPos(limiter.start_pt);
    const auto limiter_end_pt_local = g2l_tf.GetPos(limiter.end_pt);
    ILOG_INFO << "limiter_start_pt_local start pos = "
              << limiter_start_pt_local.x() << " "
              << limiter_start_pt_local.y();
    ILOG_INFO << "limiter_end_pt_local end pos = " << limiter_end_pt_local.x()
              << " " << limiter_end_pt_local.y();

    const double max_limiter_x =
        std::max(limiter_start_pt_local.x(), limiter_end_pt_local.x());

    const double min_limiter_x =
        std::min(limiter_start_pt_local.x(), limiter_end_pt_local.x());

    // limiter behind
    if (max_limiter_x < 0.5 * slot_length) {
      is_limiter_rear = true;
      target_x = std::max(
          target_x,
          max_limiter_x +
              apa_param.GetParam().parallel_ego_ac_x_offset_with_limiter);
      ILOG_INFO << "limiter in rear!";
    } else {
      // limiter front
      is_limiter_rear = false;
      target_x = std::min(
          target_x,
          min_limiter_x - apa_param.GetParam().wheel_base -
              apa_param.GetParam().parallel_ego_ac_x_offset_with_limiter);
      ILOG_INFO << "limiter in front, not release!";
      return SlotReleaseVoterType::CLEAR;
    }
  }

  const Eigen::Vector2d target_ego_pos_global =
      l2g_tf.GetPos(Eigen::Vector2d(target_x, 0.0));

  // avoid foreach obs or establish tlane, so use rough try.
  const std::vector<double> lat_mov_dist_vec{0.0, -0.4, 0.4};

  std::vector<double> lon_mov_dist_vec{0.0};

  // if one side is occupied with obs
  const double invade_x_limit =
      0.5 * slot_length +
      apa_param.GetParam().parallel_max_ego_x_offset_with_invasion -
      0.5 * apa_param.GetParam().car_length;

  const double step = 0.05;
  double x_diff = step;

  while (x_diff < invade_x_limit) {
    if (!limiter.valid || (limiter.valid && is_limiter_rear)) {
      lon_mov_dist_vec.emplace_back(x_diff);
    }
    if (!limiter.valid || (limiter.valid && (!is_limiter_rear))) {
      lon_mov_dist_vec.emplace_back(-x_diff);
    }
    x_diff += step;
  }

  bool is_slot_occupied = true;
  const double lat_buffer = 0.1;
  double lon_buffer = 0.3;
  if (limiter.valid) {
    lon_buffer = 0.4;
  }

  for (const double lat_move_dist : lat_mov_dist_vec) {
    for (const double lon_move_dist : lon_mov_dist_vec) {
      const Eigen::Vector2d target_pos =
          target_ego_pos_global + lon_move_dist * v_10 + lat_move_dist * n;

      const Pose2D target_pose(target_pos.x(), target_pos.y(), slot_heading);

      const bool is_collided =
          col_det_interface_ptr_->GetPathSafeCheckPtr()->CalcEgoCollision(
              target_pose, lat_buffer, lon_buffer, false);

      if (!is_collided) {
        ILOG_INFO << "release slot with lat offset = " << lat_move_dist;
        ILOG_INFO << "release slot with lon offset = " << lon_move_dist;
        is_slot_occupied = false;
        break;
      }
    }
    if (!is_slot_occupied) {
      break;
    }
  }
  bool car_is_left = false;
  const auto& ego_pos_global = measure_data_ptr_->GetPos();
  // ego_slot : slot origin -->ego pos
  const auto& ego_slot = ego_pos_global - slot_13_mid;

  // v10 1-->0
  const auto cross_v10_ego_slot =
      v_10.x() * ego_slot.y() - v_10.y() * ego_slot.x();
  car_is_left = cross_v10_ego_slot > 0;

  if (!state_machine_ptr_->IsParkingOutStatus() &&
      !state_machine_ptr_->IsPAMode()) {
    Polygon2D polygon;
    auto& param = apa_param.GetParam();
    Eigen::Vector2d pt0 = l2g_tf.GetPos(
        Eigen::Vector2d(0.8, car_is_left * (slot_width / 2 - 0.4)));
    Eigen::Vector2d pt1 = l2g_tf.GetPos(Eigen::Vector2d(
        0.8, car_is_left * (slot_width / 2 + param.car_width / 2 + 0.6)));
    Eigen::Vector2d pt2 = l2g_tf.GetPos(Eigen::Vector2d(
        slot_length - 0.4,
        car_is_left * (slot_width / 2 + param.car_width / 2 + 0.6)));
    Eigen::Vector2d pt3 = l2g_tf.GetPos(Eigen::Vector2d(
        slot_length - 0.4, car_is_left * (slot_width) / 2 - 0.4));

    polygon.FillTangentCircleParams(
        std::vector<Eigen::Vector2d>{pt0, pt1, pt2, pt3});
    auto col_req = GJKColDetRequest(false);
    col_req.movement_type = ApaObsMovementType::STATIC;
    col_req.use_limiter = false;
    is_slot_occupied =
        is_slot_occupied ||
        col_det_interface_ptr_->GetGJKColDetPtr()->IsPolygonCollision(polygon,
                                                                      col_req);
  }

  ILOG_INFO << "final parallel slot is occupied = " << is_slot_occupied;
  // Use sliding Windows to prevent parking slot flashing
  int parallel_slot_not_release_count =
      parallel_slot_not_release_count_map_[slot.GetId()];
  int parallel_slot_release_count =
      parallel_slot_release_count_map_[slot.GetId()];
  if (is_slot_occupied) {
    parallel_slot_not_release_count++;
  } else {
    parallel_slot_release_count++;
  }
  if ((parallel_slot_not_release_count + parallel_slot_release_count) >=
      kMaxSlotObserveFrameCount) {
    if (is_slot_occupied) {
      parallel_slot_release_count--;
    } else {
      parallel_slot_not_release_count--;
    }
    parallel_slot_not_release_count = std::clamp(
        parallel_slot_not_release_count, 0, kMaxSlotObserveFrameCount);
    parallel_slot_release_count =
        std::clamp(parallel_slot_release_count, 0, kMaxSlotObserveFrameCount);
  }
  ILOG_INFO << "slot id: " << slot.GetId()
            << " parallel_slot_not_release_count = "
            << parallel_slot_not_release_count
            << " parallel_slot_release_count = " << parallel_slot_release_count;
  if (is_slot_occupied &&
      parallel_slot_not_release_count > parallel_slot_release_count) {
    parallel_slot_not_release_count = 0;
    parallel_slot_not_release_count_map_[slot.GetId()] =
        parallel_slot_not_release_count;
    parallel_slot_release_count_map_[slot.GetId()] =
        parallel_slot_release_count;
    return SlotReleaseVoterType::CLEAR;
  }
  parallel_slot_not_release_count_map_[slot.GetId()] =
      parallel_slot_not_release_count;
  parallel_slot_release_count_map_[slot.GetId()] = parallel_slot_release_count;
  return SlotReleaseVoterType::MAXIMUM;
}

const std::string GetSlotReleaseVoterTypeString(
    const SlotReleaseVoterType release_voter_type) {
  std::string type;
  switch (release_voter_type) {
    case SlotReleaseVoterType::ACCUMULATE:
      type = "ACCUMULATE";
      break;
    case SlotReleaseVoterType::HOLD:
      type = "HOLD";
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

const SlotReleaseState ApaSlotManager::GetSlotReleaseState() const {
  if (ego_info_under_slot_.slot.release_info_
          .release_state[SlotReleaseMethod::RULE_BASED_RELEASE] ==
      SlotReleaseState::NOT_RELEASE) {
    return SlotReleaseState::NOT_RELEASE;
  }

  if (ego_info_under_slot_.slot.release_info_
          .release_state[SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE] ==
      SlotReleaseState::RELEASE) {
    return SlotReleaseState::RELEASE;
  }

  return ego_info_under_slot_.slot.release_info_
      .release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE];
}

const SlotReleaseState ApaSlotManager::GetSlotReleaseStateFreeSlot() const {
  if (ego_info_under_slot_.slot.release_info_
              .release_state[SlotReleaseMethod::RULE_BASED_RELEASE] ==
          SlotReleaseState::NOT_RELEASE ||
      ego_info_under_slot_.slot.release_info_
              .release_state[SlotReleaseMethod::FUSION_RELEASE] ==
          SlotReleaseState::NOT_RELEASE) {
    return SlotReleaseState::NOT_RELEASE;
  }

  if (ego_info_under_slot_.slot.release_info_
          .release_state[SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE] ==
      SlotReleaseState::RELEASE) {
    return SlotReleaseState::RELEASE;
  }

  return ego_info_under_slot_.slot.release_info_
      .release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE];
}

const size_t ApaSlotManager::GetEgoSlotInfoID() const {
  return ego_info_under_slot_.id;
}

const bool ApaSlotManager::GetRecommendApaParkingOperationType(
    const iflyauto::ParkingFusionInfo& parking_fusion_info) {
  const bool is_e541_car = apa_param.GetParam().car_type == 4;
  const bool is_curr_manual_status = state_machine_ptr_->IsManualStatus();
  const bool is_curr_static = measure_data_ptr_->GetStaticFlag();
  if (!is_e541_car || !is_curr_manual_status || !is_curr_static) {
    return false;
  }

  if (parking_fusion_info.is_in_parking_slot) {
    ILOG_INFO << "at this moment, the self-driving car is in the parking space";
    auto it = slots_map_.find(ego_info_under_slot_.id);
    if (it != slots_map_.end()) {
      ego_info_under_slot_.slot_type = it->second.slot_type_;
      // forced release of self slot
      const ApaSlot& slot = it->second;
      double dot_product = 0.0;

      const bool condition_0 = LateralConditions(dot_product, slot);
      const bool condition_1 = LongitudinalConditions(dot_product, slot);

      if (condition_0 && condition_1 &&
          slot.slot_type_ == SlotType::PERPENDICULAR) {
        return true;
      } else {
        return false;
      }
    }
  } else {
    ILOG_INFO
        << "at this moment, the self-driving car is not in the parking space";
  }

  return false;
}

const bool ApaSlotManager::LongitudinalConditions(const double dot_produc,
                                                  const ApaSlot& slot) const {
  const ApaParameters& param = apa_param.GetParam();
  const auto global_pt_23mid_01mid_heading_vec =
      slot.processed_corner_coord_global_.pt_23mid_01mid_vec.normalized();
  const auto origin_pose_global_heading =
      std::atan2(ego_info_under_slot_.origin_pose_global.heading_vec.y(),
                 ego_info_under_slot_.origin_pose_global.heading_vec.x());
  const auto origin_pose_global_pos =
      slot.processed_corner_coord_global_.pt_01_mid -
      slot.slot_length_ * global_pt_23mid_01mid_heading_vec;

  const auto g2l_tf = geometry_lib::GlobalToLocalTf(origin_pose_global_pos,
                                                    origin_pose_global_heading);
  const auto local_pt_0 = g2l_tf.GetPos(slot.origin_corner_coord_global_.pt_0);
  const auto local_pt_2 = g2l_tf.GetPos(slot.origin_corner_coord_global_.pt_2);
  const double solt_length = (local_pt_0 - local_pt_2).norm();

  const auto cur_pose_pos_local = g2l_tf.GetPos(measure_data_ptr_->GetPos());

  double dist_exceeds_slot = cur_pose_pos_local.x();
  if (dot_produc > 0.0) {
    dist_exceeds_slot = cur_pose_pos_local.x() + param.car_length -
                        param.rear_overhanging - solt_length;
  } else {
    dist_exceeds_slot =
        cur_pose_pos_local.x() + param.rear_overhanging - solt_length;
  }

  ILOG_INFO << "dist_exceeds_slot " << dist_exceeds_slot << ", solt_length "
            << solt_length
            << ", GetPos x = " << measure_data_ptr_->GetPos().x();
  constexpr double kDistanceThreshold = 1.2;
  return dist_exceeds_slot <= kDistanceThreshold;
}

const bool ApaSlotManager::LateralConditions(double& dot_product,
                                             const ApaSlot& slot) const {
  const Eigen::Vector2d& ego_heading_vec = measure_data_ptr_->GetHeadingVec();
  const Eigen::Vector2d& pt_23mid_01mid_unit_vec =
      slot.GetOriginCornerCoordGlobal().pt_23mid_01mid_unit_vec;
  dot_product = ego_heading_vec.dot(pt_23mid_01mid_unit_vec);
  const double norm_a = ego_heading_vec.norm();
  const double norm_b = pt_23mid_01mid_unit_vec.norm();

  if (norm_a < 1e-9 || norm_b < 1e-9) {
    return false;
  }

  const double cos_theta = dot_product / (norm_a * norm_b);
  constexpr double kCosineValueMaximumAngle = 0.9962;  // cos(5°)

  return cos_theta > kCosineValueMaximumAngle;
}

bool ApaSlotManager::IsSideParkingPerpendicularSlot(const ApaSlot& slot) {
  if (slot.slot_source_type_ != SlotSourceType::CAMERA) {
    return false;
  }
  if (slot.slot_type_ != SlotType::PERPENDICULAR) {
    return false;
  }
  if (slot.release_info_.release_state[FUSION_RELEASE] !=
      SlotReleaseState::RELEASE) {
    return false;
  }

  const std::unordered_map<size_t, ApaObstacle>& obs =
      obstacle_manager_ptr_->GetObstacles();
  geometry_lib::PathPoint ego_local_point;
  bool left_area_is_empty = true, right_area_is_empty = true,
       front_area_is_empty = true;
  bool is_redefine_slot_type = false;
  int ego_side_to_slot = 0;  // 1:车在车位右侧 -1：车在车位左侧

  const Eigen::Vector2d ego_global_point = measure_data_ptr_->GetPos();
  const Eigen::Vector2d left_mirror_global_point =
      measure_data_ptr_->GetLeftMirrorPos();
  const Eigen::Vector2d right_mirror_global_point =
      measure_data_ptr_->GetRightMirrorPos();
  ego_local_point.pos = slot.g2l_tf_.GetPos(ego_global_point);
  ego_local_point.heading =
      slot.g2l_tf_.GetHeading(measure_data_ptr_->GetHeading()) * 57.3;

  if (ego_local_point.heading > 80.0 || ego_local_point.heading < -80.0) {
    return false;
  }

  if (fabs(ego_local_point.pos.y()) > 5.0) {
    return false;
  }
  ILOG_INFO << "ego_local_point.pos.x = " << ego_local_point.pos.x()
            << "ego_local_point.pos.y = " << ego_local_point.pos.y()
            << "ego_local_point.heading = " << ego_local_point.heading;

  planning_math::Vec2d center;
  Eigen::Vector2d P0, P1, P2, P3;
  double length, width, heading;
  const Eigen::Vector2d pt02_vec = slot.origin_corner_coord_global_.pt_0 -
                                   slot.origin_corner_coord_global_.pt_2;
  const Eigen::Vector2d pt13_vec = slot.origin_corner_coord_global_.pt_1 -
                                   slot.origin_corner_coord_global_.pt_3;
  const Eigen::Vector2d pt13_unit_vec = pt13_vec.normalized();
  const Eigen::Vector2d pt13_unit_normal_vec = Eigen::Vector2d(
      -pt13_unit_vec.y(), pt13_unit_vec.x());  // counterclockwise

  length = pt13_vec.norm();
  width = 1.5;
  heading = std::atan2(pt13_unit_vec.y(), pt13_unit_vec.x());
  P0 = slot.origin_corner_coord_global_.pt_3 + pt13_unit_vec * 1.5;
  P1 = slot.origin_corner_coord_global_.pt_1;
  P2 = slot.origin_corner_coord_global_.pt_1 + pt13_unit_normal_vec * 1.5;
  P3 = P0 + pt13_unit_normal_vec * 1.5;
  center.set_x((P0.x() + P1.x() + P2.x() + P3.x()) * 0.25);
  center.set_y((P0.y() + P1.y() + P2.y() + P3.y()) * 0.25);
  const planning_math::Box2d& slot_left_area =
      planning_math::Box2d(center, heading, length, width);

  const Eigen::Vector2d pt02_unit_vec = pt02_vec.normalized();
  const Eigen::Vector2d pt02_unit_normal_vec =
      Eigen::Vector2d(pt02_unit_vec.y(), -pt02_unit_vec.x());  // clockwise

  length = pt02_vec.norm();
  width = 1.5;
  heading = std::atan2(pt02_unit_vec.y(), pt02_unit_vec.x());
  P0 = slot.origin_corner_coord_global_.pt_2 + pt02_unit_vec * 1.5;
  P1 = P0 + pt02_unit_normal_vec * 1.5;
  P2 = slot.origin_corner_coord_global_.pt_0 + pt02_unit_normal_vec * 1.5;
  P3 = slot.origin_corner_coord_global_.pt_0;
  center.set_x((P0.x() + P1.x() + P2.x() + P3.x()) * 0.25);
  center.set_y((P0.y() + P1.y() + P2.y() + P3.y()) * 0.25);
  const planning_math::Box2d& slot_right_area =
      planning_math::Box2d(center, heading, length, width);
  const Eigen::Vector2d pt01_unit_normal_vec =
      Eigen::Vector2d(slot.origin_corner_coord_global_.pt_01_unit_vec.y(),
                      -slot.origin_corner_coord_global_.pt_01_unit_vec.x());
  length = 3.5;
  width = slot.origin_corner_coord_global_.pt_01_vec.norm();
  heading = std::atan2(pt01_unit_normal_vec.y(), pt01_unit_normal_vec.x());
  P0 = slot.origin_corner_coord_global_.pt_0 +
       slot.origin_corner_coord_global_.pt_01_unit_vec * 0.3;
  P1 = P0 + pt01_unit_normal_vec * 3.5;
  P3 = slot.origin_corner_coord_global_.pt_1 +
       slot.origin_corner_coord_global_.pt_01_unit_vec * (-0.3);
  P2 = P3 + pt01_unit_normal_vec * 3.5;
  center.set_x((P0.x() + P1.x() + P2.x() + P3.x()) * 0.25);
  center.set_y((P0.y() + P1.y() + P2.y() + P3.y()) * 0.25);
  const planning_math::Box2d& slot_front_area =
      planning_math::Box2d(center, heading, length, width);

  for (const auto& pair : obs) {
    for (const auto& pt : pair.second.GetPtClout2dGlobal()) {
      if (left_area_is_empty) {
        left_area_is_empty =
            !(slot_left_area.IsPointIn(planning_math::Vec2d(pt.x(), pt.y())));
      }
      if (right_area_is_empty) {
        right_area_is_empty =
            !(slot_right_area.IsPointIn(planning_math::Vec2d(pt.x(), pt.y())));
      }
      if (front_area_is_empty) {
        front_area_is_empty =
            !(slot_front_area.IsPointIn(planning_math::Vec2d(pt.x(), pt.y())));
      }
    }
  }

  ILOG_INFO << "slot origin id = " << slot.GetId()
            << "  left_area_is_empty = " << static_cast<int>(left_area_is_empty)
            << "  right_area_is_empty = "
            << static_cast<int>(right_area_is_empty)
            << "  front_area_is_empty = "
            << static_cast<int>(front_area_is_empty);

  const Eigen::Vector2d ego2pt2_vec =
      ego_global_point - slot.origin_corner_coord_global_.pt_2;
  const Eigen::Vector2d ego2pt3_vec =
      ego_global_point - slot.origin_corner_coord_global_.pt_3;

  const double ego2pt2_vec_dot =
      ego2pt2_vec.dot(slot.origin_corner_coord_global_.pt_23_unit_vec);
  const double ego2pt3_vec_dot =
      ego2pt3_vec.dot(slot.origin_corner_coord_global_.pt_23_unit_vec);

  const Eigen::Vector2d left_mirror2pt2_vec =
      left_mirror_global_point - slot.origin_corner_coord_global_.pt_2;
  const Eigen::Vector2d right_mirror2pt3_vec =
      right_mirror_global_point - slot.origin_corner_coord_global_.pt_3;
  const double left_mirror2pt2_vec_dot = left_mirror2pt2_vec.dot(pt02_unit_vec);
  const double right_mirror2pt3_vec_dot =
      right_mirror2pt3_vec.dot(pt13_unit_vec);
  if (ego2pt2_vec_dot < 0.0) {
    ego_side_to_slot = 1;
    if (!right_area_is_empty) {
      is_redefine_slot_type = false;
    } else {
      if (!front_area_is_empty) {
        is_redefine_slot_type = true;
      } else {
        if (left_mirror2pt2_vec_dot <= pt02_vec.norm()) {
          if (ego_local_point.heading <= 4.0 &&
              ego_local_point.heading > -80.0) {
            is_redefine_slot_type = true;
          } else {
            is_redefine_slot_type = false;
          }
        } else {
          if (ego_local_point.heading > 2.0) {
            is_redefine_slot_type = false;
          } else {
            is_redefine_slot_type = true;
          }
        }
      }
    }
  } else if (ego2pt3_vec_dot > 0.0) {
    ego_side_to_slot = -1;
    if (!left_area_is_empty) {
      is_redefine_slot_type = false;
    } else {
      if (!front_area_is_empty) {
        is_redefine_slot_type = true;
      } else {
        if (right_mirror2pt3_vec_dot <= pt13_vec.norm()) {
          if (ego_local_point.heading >= -4.0 &&
              ego_local_point.heading < 80.0) {
            is_redefine_slot_type = true;
          } else {
            is_redefine_slot_type = false;
          }
        } else {
          if (ego_local_point.heading < -2.0) {
            is_redefine_slot_type = false;
          } else {
            is_redefine_slot_type = true;
          }
        }
      }
    }
  }
  ILOG_INFO << "is_redefine_slot_type = "
            << static_cast<int>(is_redefine_slot_type);
  if (is_redefine_slot_type)
    perpendicular_redefine_info_map_.insert(
        perpendicular_redefine_info_map_.end(),
        {slot.GetId(), {is_redefine_slot_type, ego_side_to_slot}});
  return is_redefine_slot_type;
}

}  // namespace apa_planner
}  // namespace planning
