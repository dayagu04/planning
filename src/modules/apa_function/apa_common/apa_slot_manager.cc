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
#include "log_glog.h"
#include "target_pose_decider/target_pose_decider.h"

namespace planning {
namespace apa_planner {

static const uint8_t kSlotReleaseVoteCount = 6;
static const uint8_t kMaxSlotReleaseCount = 8;

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

  ILOG_INFO << "Update ApaSlotManager";
  measure_data_ptr_ = measure_data_ptr;
  state_machine_ptr_ = state_machine_ptr;
  obstacle_manager_ptr_ = obstacle_manager_ptr;
  col_det_interface_ptr_ = col_det_interface_ptr;

  slots_map_.clear();
  dist_id_map_.clear();

  const size_t slot_size =
      local_view->parking_fusion_info.parking_fusion_slot_lists_size;
  const size_t select_slot_id = local_view->parking_fusion_info.select_slot_id;

  ILOG_INFO << "parking_fusion slot size = " << slot_size
            << "  select slot id = " << select_slot_id;

  const Eigen::Vector2d car_mirror_pos =
      0.5 * (measure_data_ptr->GetLeftMirrorPos() +
             measure_data_ptr->GetRightMirrorPos());

  for (size_t i = 0; i < slot_size; ++i) {
    const iflyauto::ParkingFusionSlot& fusion_slot =
        local_view->parking_fusion_info.parking_fusion_slot_lists[i];

    ApaSlot slot;
    slot.Update(fusion_slot);

    const double dist =
        (car_mirror_pos - slot.GetOriginCornerCoordGlobal().pt_center).norm();

    dist_id_map_[dist] = slot.GetId();
    slots_map_[slot.GetId()] = slot;
  }

  // 泊出
  if (state_machine_ptr_->IsParkOutStatus()) {
    if (state_machine_ptr_->IsSeachingStatus()) {
      if (!dist_id_map_.empty()) {
        ego_info_under_slot_.history_id = ego_info_under_slot_.id;
        ego_info_under_slot_.history_slot_type = ego_info_under_slot_.slot_type;
        ego_info_under_slot_.id = dist_id_map_.begin()->second;

        auto it = slots_map_.find(ego_info_under_slot_.id);
        if (it != slots_map_.end()) {
          ego_info_under_slot_.slot_type = it->second.slot_type_;
          // forced release of self slot
          ApaSlot& slot = slots_map_[ego_info_under_slot_.id];
          const double dot_ego_slot = measure_data_ptr_->GetHeadingVec().dot(
              slot.GetOriginCornerCoordGlobal().pt_23mid_01mid_unit_vec);
          if (slot.slot_type_ == SlotType::PERPENDICULAR &&
              state_machine_ptr_->IsHeadOutStatus()) {
            // 对于垂直车头泊出，开口方向与自车方向不一致不释放车位
            if (dot_ego_slot > 0.0) {
              slot.release_info_.release_state[RULE_BASED_RELEASE] =
                  SlotReleaseState::RELEASE;
            } else {
              slot.release_info_.release_state[RULE_BASED_RELEASE] =
                  SlotReleaseState::NOT_RELEASE;
            }
          } else {
            slot.release_info_.release_state[RULE_BASED_RELEASE] =
                SlotReleaseState::RELEASE;
          }
        } else {
          ILOG_WARN << "slot id = " << ego_info_under_slot_.id
                    << " not found in slots_map_";
          ego_info_under_slot_.Reset();
        }

      } else {
        ILOG_WARN
            << "dist_id_map_ is empty, cannot update ego_info_under_slot_";
        ego_info_under_slot_.Reset();
      }

    } else if (state_machine_ptr_->IsParkingStatus()) {
    }
  }

  // 泊入
  if (state_machine_ptr->IsParkInStatus()) {
    if (state_machine_ptr_->IsSeachingStatus()) {
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

      if (slots_map_.count(select_slot_id) != 0) {
        ego_info_under_slot_.history_id = ego_info_under_slot_.id;
        ego_info_under_slot_.history_slot_type = ego_info_under_slot_.slot_type;
        ego_info_under_slot_.id = select_slot_id;
        ego_info_under_slot_.slot_type = slots_map_[select_slot_id].slot_type_;
      } else {
        ego_info_under_slot_.Reset();
      }
    } else if (state_machine_ptr_->IsParkingStatus()) {
      // 泊车过程中锁定车位id和类型, 不进行更新, 选中车位如果消失做特殊处理
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
    }
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

  return;
}

void ApaSlotManager::GenerateReleaseSlotIdVec() {
  if (!state_machine_ptr_->IsSeachingStatus()) {
    return;
  }
  release_slot_id_vec_.clear();
  for (const auto& pair : dist_id_map_) {
    if (slots_map_.count(pair.second) == 0) {
      continue;
    }
    const ApaSlot& slot = slots_map_[pair.second];
    if (slot.id_ != ego_info_under_slot_.id) {
      if (slot.release_info_.release_state[RULE_BASED_RELEASE] ==
          SlotReleaseState::RELEASE) {
        release_slot_id_vec_.emplace_back(slot.id_);
      }
    } else {
      if (ego_info_under_slot_.slot.release_info_
                  .release_state[RULE_BASED_RELEASE] ==
              SlotReleaseState::RELEASE &&
          (ego_info_under_slot_.slot.release_info_
                   .release_state[GEOMETRY_PLANNING_RELEASE] ==
               SlotReleaseState::RELEASE ||
           ego_info_under_slot_.slot.release_info_
                   .release_state[ASTAR_PLANNING_RELEASE] ==
               SlotReleaseState::RELEASE)) {
        release_slot_id_vec_.emplace_back(slot.id_);
      } else if (ego_info_under_slot_.slot.release_info_
                         .release_state[RULE_BASED_RELEASE] ==
                     SlotReleaseState::RELEASE &&
                 ego_info_under_slot_.slot.release_info_
                         .release_state[ASTAR_PLANNING_RELEASE] ==
                     SlotReleaseState::COMPUTING) {
        release_slot_id_vec_.emplace_back(slot.id_);
      }
    }
  }
}

void ApaSlotManager::ParkingLotCruiseProcess() {
  // 在泊入寻库阶段通过简单的规则判断车位是否应该释放
  const double time_start = IflyTime::Now_ms();

  // const bool is_ego_collision = IsEgoCloseToObs();

  is_ego_col_vertical_ = IsEgoCloseToObs();
  is_ego_col_parallel_ = IsEgoCloseToObs(0.1, 0.01, 0.1);

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
      ILOG_INFO << "NOT_RELEASE reason: fusion not release";
      continue;
    }

    // if (is_ego_collision) {
    //   slot.release_info_.release_state[RULE_BASED_RELEASE] =
    //       SlotReleaseState::NOT_RELEASE;
    //   continue;
    // }

    if (release_slot_count > kMaxSlotReleaseCount) {
      slot.release_info_.release_state[RULE_BASED_RELEASE] =
          SlotReleaseState::NOT_RELEASE;
      ILOG_INFO << "NOT_RELEASE reason: over max released size "
                << kMaxSlotReleaseCount;
      continue;
    }

    if (dist_id.first > 10.68) {
      slot.release_info_.release_state[RULE_BASED_RELEASE] =
          SlotReleaseState::NOT_RELEASE;
      ILOG_INFO << "NOT_RELEASE reason: nearest slot dist over " << 10.68
                << " m!";
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
      ego, body_lat_buffer, lon_buffer);
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
  if (is_ego_col_vertical_) {
    return SlotReleaseVoterType::CLEAR;
  }

  const ApaParameters& param = apa_param.GetParam();
  const Eigen::Vector2d pM01 = slot.processed_corner_coord_global_.pt_01_mid;
  const Eigen::Vector2d pM23 = slot.processed_corner_coord_global_.pt_23_mid;
  const Eigen::Vector2d t = slot.processed_corner_coord_global_.pt_01_unit_vec;
  const Eigen::Vector2d n =
      slot.processed_corner_coord_global_.pt_23mid_01mid_unit_vec;

  // 构建车位可泊最低要求区域内box, 检查box内是否有障碍物 若有 不释放
  // 主要是为了缓解融合误释放车位
  Polygon2D polygon;
  polygon.FillTangentCircleParams(
      slot.GetCustomSlotPolygon(2.68, -2.0, -0.4, -0.4, false));
  if (col_det_interface_ptr_->GetGJKColDetPtr()->IsPolygonCollision(
          polygon, GJKColDetRequest(false))) {
    ILOG_INFO << "slot min parking area is occupied";
    return SlotReleaseVoterType::CLEAR;
  }

  // 产品定义是最大车辆宽度加0.4米释放  即单侧buffer 0.2米
  // 0.26米如果没有碰撞 直接释放
  // 0.20米如果没有碰撞 累加
  // 0.17米如果没有碰撞 维持不变
  // 0.16米如果没有碰撞 累减
  // 0.16米如果碰撞 直接不释放
  // const std::vector<std::pair<double, SlotReleaseVoterType>>
  //     lat_buffer_pair_vec = {{0.26, SlotReleaseVoterType::MAXIMUM},
  //                            {0.20, SlotReleaseVoterType::ACCUMULATE},
  //                            {0.17, SlotReleaseVoterType::HOLD},
  //                            {0.16, SlotReleaseVoterType::SUBTRACT}};

  std::vector<double> lat_buffer_vec{0.23, 0.17, 0.14, 0.13};

  TargetPoseDecider tar_pose_decider(col_det_interface_ptr_);
  TargetPoseDeciderRequest tar_pose_decider_request(
      lat_buffer_vec, 0.3, ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN,
      true, false);

  TargetPoseDeciderResult res =
      tar_pose_decider.CalcTargetPose(slot, tar_pose_decider_request);

  if (res.target_pose_type == TargetPoseType::FAIL) {
    ILOG_INFO << "slot is occupied";
    return SlotReleaseVoterType::CLEAR;
  }

  ILOG_INFO << "lat_move_slot_dist = " << res.safe_lat_move_dist
            << "  lon_move_slot_dist = " << res.safe_lon_move_dist
            << "  lat_buffer = " << res.safe_lat_buffer;

  SlotReleaseVoterType release_voter_type;
  if (geometry_lib::IsTwoNumerEqual(0.23, res.safe_lat_buffer)) {
    release_voter_type = SlotReleaseVoterType::MAXIMUM;
  } else if (geometry_lib::IsTwoNumerEqual(0.17, res.safe_lat_buffer)) {
    release_voter_type = SlotReleaseVoterType::ACCUMULATE;
  } else if (geometry_lib::IsTwoNumerEqual(0.14, res.safe_lat_buffer)) {
    release_voter_type = SlotReleaseVoterType::HOLD;
  } else if (geometry_lib::IsTwoNumerEqual(0.13, res.safe_lat_buffer)) {
    release_voter_type = SlotReleaseVoterType::SUBTRACT;
  }

  ILOG_INFO << "release_voter_type = "
            << GetSlotReleaseVoterTypeString(release_voter_type);

  const double lat_buffer = 0.0368;

  Eigen::Vector2d pt_0 = pM01 - t * (param.max_car_width * 0.5 + lat_buffer);
  Eigen::Vector2d pt_1 = pM01 + t * (param.max_car_width * 0.5 + lat_buffer);

  pt_0 = pt_0 + res.safe_lat_move_dist * t;
  pt_1 = pt_1 + res.safe_lat_move_dist * t;

  double channel_width = param.slot_release_channel_width;

  if (release_voter_type == SlotReleaseVoterType::MAXIMUM) {
    channel_width = param.slot_release_channel_width - 0.2;
  } else if (release_voter_type == SlotReleaseVoterType::HOLD) {
    channel_width = param.slot_release_channel_width + 0.2;
  }

  // 判断车位左侧或者右侧是否有障碍物 来判断是否可以放宽通道宽要求
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

  if (left_empty && right_empty) {
    channel_width = param.two_side_empty_slot_release_channel_width;
  } else if (left_empty || right_empty) {
    channel_width = param.one_side_empty_slot_release_channel_width;
  }

  polygon.FillTangentCircleParams(std::vector<Eigen::Vector2d>{
      pt_0, pt_0 + channel_width * n, pt_1 + channel_width * n, pt_1});

  if (col_det_interface_ptr_->GetGJKColDetPtr()->IsPolygonCollision(
          polygon, GJKColDetRequest(false))) {
    ILOG_INFO << "passage is occupied";
    return SlotReleaseVoterType::CLEAR;
  }

  return release_voter_type;
}

const SlotReleaseVoterType ApaSlotManager::IsParallelSlotAndPassageAreaOccupied(
    const ApaSlot& slot) {
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
              << limiter_start_pt_local.transpose();
    ILOG_INFO << "limiter_end_pt_local end pos = "
              << limiter_end_pt_local.transpose();

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
  const double lon_buffer = 0.3;

  for (const double lat_move_dist : lat_mov_dist_vec) {
    for (const double lon_move_dist : lon_mov_dist_vec) {
      const Eigen::Vector2d target_pos =
          target_ego_pos_global + lon_move_dist * v_10 + lat_move_dist * n;

      const Pose2D target_pose(target_pos.x(), target_pos.y(), slot_heading);

      const bool is_collided =
          col_det_interface_ptr_->GetPathSafeCheckPtr()->CalcEgoCollision(
              target_pose, lat_buffer, lon_buffer);

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
  ILOG_INFO << "final parallel slot is occupied = " << is_slot_occupied;

  if (is_slot_occupied) {
    return SlotReleaseVoterType::CLEAR;
  }
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

  if (ego_info_under_slot_.slot.release_info_
          .release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE] ==
      SlotReleaseState::RELEASE) {
    return SlotReleaseState::RELEASE;
  } else if (ego_info_under_slot_.slot.release_info_
                 .release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE] ==
             SlotReleaseState::NOT_RELEASE) {
    return SlotReleaseState::NOT_RELEASE;
  } else if (ego_info_under_slot_.slot.release_info_
                 .release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE] ==
             SlotReleaseState::COMPUTING) {
    return SlotReleaseState::COMPUTING;
  }

  return SlotReleaseState::UNKOWN;
}

const size_t ApaSlotManager::GetEgoSlotInfoID() const {
  return ego_info_under_slot_.id;
}

}  // namespace apa_planner
}  // namespace planning
