#include "slot_management.h"

#include <math.h>
#include <sys/types.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "apa_param_setting.h"
#include "basic_types.pb.h"
#include "camera_preception_groundline_c.h"
#include "common.h"
#include "common.pb.h"
#include "common_c.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "dubins_lib.h"
#include "func_state_machine_c.h"
#include "fusion_objects_c.h"
#include "fusion_parking_slot_c.h"
#include "geometry_math.h"
#include "math_lib.h"
#include "perpendicular_path_planner.h"
#include "slot_management_info.pb.h"
#include "transform_lib.h"

namespace planning {

namespace {
constexpr double kPie = 3.141592653589793;
constexpr double kEps = 1e-6;
}  // namespace

bool SlotManagement::Update(const LocalView *local_view_ptr) {
  return Update(&local_view_ptr->function_state_machine_info,
                &local_view_ptr->parking_fusion_info,
                &local_view_ptr->localization_estimate,
                &local_view_ptr->uss_wave_info,
                &local_view_ptr->uss_percept_info,
                &local_view_ptr->ground_line_perception,
                &local_view_ptr->fusion_objects_info);
}

bool SlotManagement::Update(
    const iflyauto::FuncStateMachine *func_statemachine,
    const iflyauto::ParkingFusionInfo *parking_slot_info,
    const iflyauto::LocalizationEstimate *localization_info,
    const iflyauto::UssWaveInfo *uss_wave_info,
    const iflyauto::UssPerceptInfo *uss_percept_info,
    const iflyauto::GroundLinePerceptionInfo *ground_line_perception_info,
    const iflyauto::FusionObjectsInfo *fusion_objects_info) {
  DEBUG_PRINT("---------- slot management --------------------");
  // set ptrs
  frame_.func_state_ptr = func_statemachine;
  frame_.parking_slot_ptr = parking_slot_info;
  frame_.localization_ptr = localization_info;
  frame_.uss_wave_info_ptr = uss_wave_info;
  frame_.uss_percept_info_ptr = uss_percept_info;
  frame_.ground_line_perception_info_ptr = ground_line_perception_info;
  frame_.fusion_objects_info_ptr = fusion_objects_info;

  if (!IsInAPAState() || frame_.param.force_clear) {
    DEBUG_PRINT("reset");
    Reset();
    return false;
  }

  // preprocess
  Preprocess();

  if (apa_param.GetParam().believe_in_fus_obs) {
    // update obs
    AddObstacles();
  }

  bool update_slot_in_searching_flag = false;
  bool update_slot_in_parking_flag = false;
  // update_slot_in_searching_flag is always false, only update slot
  if (IsInSearchingState()) {
    if (!apa_param.GetParam().believe_in_fus_obs) {
      // update obs
      AddObstacles();
    }
    update_slot_in_searching_flag = UpdateSlotsInSearching();
  } else if (IsInParkingState()) {
    update_slot_in_parking_flag = UpdateSlotsInParking();
  } else {
    // apa finish or fail
    Reset();
  }

  // for hmi
  UpdateReleasedSlotInfo();

  // restore slot management info
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->clear_slot_management_info();
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_slot_management_info()
      ->CopyFrom(frame_.slot_management_info);

  Log();

  return update_slot_in_searching_flag || update_slot_in_parking_flag;
}

void SlotManagement::AddObstacles() {
  frame_.obs_pt_vec.clear();
  AddFusionObjects();
  if (!frame_.obs_pt_vec.empty()) {
    // fus obj is valid
    frame_.fus_obj_valid_flag = true;
    AddGroundLineObstacles();
  } else {
    frame_.fus_obj_valid_flag = false;
    // AddUssPerceptObstacles();
  }
  DEBUG_PRINT("fus_obj_valid_flag  = " << frame_.fus_obj_valid_flag);
}

void SlotManagement::AddFusionObjects() {
  if (frame_.fusion_objects_info_ptr == nullptr) {
    DEBUG_PRINT("fusion_objects_info_ptr is nullptr");
    return;
  }

  const uint8 fusion_object_num =
      frame_.fusion_objects_info_ptr->fusion_object_num;

  if (fusion_object_num == 0) {
    DEBUG_PRINT("fusion objects is empty");
    return;
  }

  // Assuming an object has a maximum of 20 obstacle points
  frame_.obs_pt_vec.reserve(frame_.obs_pt_vec.size() + fusion_object_num * 20);
  const size_t N_begin = frame_.obs_pt_vec.size();

  Eigen::Vector2d fs_pt;
  iflyauto::FusionObject fusion_object;
  for (uint8 i = 0; i < fusion_object_num; ++i) {
    fusion_object = frame_.fusion_objects_info_ptr->fusion_object[i];
    for (uint32 j = 0; j < fusion_object.additional_info.polygon_points_num;
         ++j) {
      fs_pt << fusion_object.additional_info.polygon_points[j].x,
          fusion_object.additional_info.polygon_points[j].y;
      frame_.obs_pt_vec.emplace_back(fs_pt);
    }
  }

  const size_t N_end = frame_.obs_pt_vec.size();
  if (N_end == N_begin) {
    DEBUG_PRINT("fusion objects is empty");
    return;
  } else {
    DEBUG_PRINT("fusion objects size = " << N_end - N_begin);
  }
}

void SlotManagement::AddGroundLineObstacles() {
  if (frame_.ground_line_perception_info_ptr == nullptr) {
    DEBUG_PRINT("ground_line_perception_info_ptr is nullptr");
    return;
  }

  const uint8_t ground_lines_size =
      frame_.ground_line_perception_info_ptr->ground_lines_size;

  if (ground_lines_size == 0) {
    DEBUG_PRINT("ground line is empty");
    return;
  }

  // Assuming a ground line has a maximum of 20 obstacle points
  frame_.obs_pt_vec.reserve(frame_.obs_pt_vec.size() + ground_lines_size * 20);
  const size_t N_begin = frame_.obs_pt_vec.size();

  Eigen::Vector2d gl_pt;
  iflyauto::GroundLine gl;
  for (uint8_t i = 0; i < ground_lines_size; ++i) {
    gl = frame_.ground_line_perception_info_ptr->ground_lines[i];
    for (uint8 j = 0; j < gl.points_3d_size; ++j) {
      gl_pt << gl.points_3d[j].x, gl.points_3d[j].y;
      frame_.obs_pt_vec.emplace_back(gl_pt);
    }
  }

  const size_t N_end = frame_.obs_pt_vec.size();
  if (N_end == N_begin) {
    DEBUG_PRINT("ground line is empty");
    return;
  } else {
    DEBUG_PRINT("ground line size = " << N_end - N_begin);
  }
}

void SlotManagement::AddUssPerceptObstacles() {
  if (frame_.uss_percept_info_ptr == nullptr) {
    DEBUG_PRINT("uss_percept_info_ptr is empty");
    return;
  }

  const auto &obj_info_desample =
      frame_.uss_percept_info_ptr
          ->out_line_dataori[0];  // 0 means desample while 1 means raw model
                                  // output

  const uint32 uss_pt_num = obj_info_desample.obj_pt_cnt;

  if (uss_pt_num == 0) {
    DEBUG_PRINT("uss obs is empty");
    return;
  }

  frame_.obs_pt_vec.reserve(frame_.obs_pt_vec.size() + uss_pt_num);
  Eigen::Vector2d uss_pt;
  for (uint32 i = 0; i < uss_pt_num; ++i) {
    uss_pt << obj_info_desample.obj_pt_global[i].x,
        obj_info_desample.obj_pt_global[i].y;
    frame_.obs_pt_vec.emplace_back(uss_pt);
  }
}

bool SlotManagement::IsInAPAState() const {
  if ((frame_.func_state_ptr->current_state >=
           iflyauto::FunctionalState_PARK_IN_APA_IN &&
       frame_.func_state_ptr->current_state <=
           iflyauto::FunctionalState_PARK_IN_COMPLETED) ||
      frame_.param.force_apa_on) {
    DEBUG_PRINT("apa in at present");
    return true;
  }
  return false;
}

void SlotManagement::Reset() { frame_.Reset(); }

void SlotManagement::Preprocess() {
  auto &measurement = frame_.measurement;
  const auto &local_pos = frame_.localization_ptr->pose;

  measurement.ego_pos << local_pos.local_position.x, local_pos.local_position.y;

  measurement.heading = local_pos.heading;
  measurement.ego_heading_vec =
      pnc::geometry_lib::GetUnitTangVecByHeading(measurement.heading);

  const Eigen::Vector2d ego_heading_vec_turn_right(
      measurement.ego_heading_vec.y(), -measurement.ego_heading_vec.x());

  const Eigen::Vector2d ego_heading_vec_turn_left(
      -measurement.ego_heading_vec.y(), measurement.ego_heading_vec.x());

  measurement.right_mirror_pos =
      measurement.ego_pos +
      apa_param.GetParam().lon_dist_mirror_to_rear_axle *
          measurement.ego_heading_vec +
      apa_param.GetParam().lat_dist_mirror_to_center *
          ego_heading_vec_turn_right;

  measurement.left_mirror_pos =
      measurement.ego_pos +
      apa_param.GetParam().lon_dist_mirror_to_rear_axle *
          measurement.ego_heading_vec +
      apa_param.GetParam().lat_dist_mirror_to_center *
          ego_heading_vec_turn_left;

  measurement.v_ego = frame_.localization_ptr->pose.linear_velocity_from_wheel;
}

bool SlotManagement::IsInSearchingState() const {
  if ((frame_.func_state_ptr->current_state >=
           iflyauto::FunctionalState_PARK_IN_APA_IN &&
       frame_.func_state_ptr->current_state <=
           iflyauto::FunctionalState_PARK_IN_NO_READY) ||
      (frame_.param.force_apa_on && (!frame_.param.is_switch_parking))) {
    return true;
  }
  return false;
}

// used in searching state
bool SlotManagement::UpdateEgoSlotInfo(EgoSlotInfo &ego_slot_info,
                                       const common::SlotInfo *slot_info) {
  const auto &slot_points = slot_info->corner_points().corner_point();
  if (slot_points.size() < 4) {
    DEBUG_PRINT("slot_points size is not normal, quit");
    return false;
  }

  ego_slot_info.slot_type = slot_info->slot_type();
  ego_slot_info.select_slot_id = slot_info->id();

  std::vector<Eigen::Vector2d> pt;
  pt.resize(4);
  for (size_t i = 0; i < 4; ++i) {
    pt[i] << slot_points[i].x(), slot_points[i].y();
  }
  const auto pM01 = 0.5 * (pt[0] + pt[1]);
  const auto pM23 = 0.5 * (pt[2] + pt[3]);
  const double real_slot_length = (pM01 - pM23).norm();
  // const auto t = (pt[1] - pt[0]).normalized();
  // const auto n = Eigen::Vector2d(t.y(), -t.x());
  const auto n = (pM01 - pM23).normalized();
  pt[2] = pt[0] - real_slot_length * n;
  pt[3] = pt[1] - real_slot_length * n;

  ego_slot_info.slot_corner = pt;

  const double virtual_slot_length =
      apa_param.GetParam().car_length +
      apa_param.GetParam().slot_compare_to_car_length;

  ego_slot_info.slot_origin_pos = pM01 - virtual_slot_length * n;
  ego_slot_info.slot_origin_heading = std::atan2(n.y(), n.x());
  ego_slot_info.slot_origin_heading_vec = n;
  ego_slot_info.slot_length = virtual_slot_length;
  ego_slot_info.slot_width = (pt[0] - pt[1]).norm();

  ego_slot_info.g2l_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  ego_slot_info.l2g_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  ego_slot_info.ego_pos_slot =
      ego_slot_info.g2l_tf.GetPos(frame_.measurement.ego_pos);

  ego_slot_info.ego_heading_slot =
      ego_slot_info.g2l_tf.GetHeading(frame_.measurement.heading);

  ego_slot_info.ego_heading_slot_vec
      << std::cos(ego_slot_info.ego_heading_slot),
      std::sin(ego_slot_info.ego_heading_slot);

  ego_slot_info.target_ego_pos_slot << apa_param.GetParam().terminal_target_x,
      apa_param.GetParam().terminal_target_y;

  ego_slot_info.target_ego_heading_slot =
      apa_param.GetParam().terminal_target_heading;

  ego_slot_info.sin_angle = 1.0;
  ego_slot_info.origin_pt_0_heading = 0.0;
  ego_slot_info.pt_0 = ego_slot_info.g2l_tf.GetPos(pt[0]);
  ego_slot_info.pt_1 = ego_slot_info.g2l_tf.GetPos(pt[1]);
  if (slot_info->slot_type() == Common::PARKING_SLOT_TYPE_SLANTING) {
    if (frame_.slot_info_angle.count(slot_info->id()) != 0) {
      ego_slot_info.sin_angle = frame_.slot_info_angle[slot_info->id()].second;
      ego_slot_info.origin_pt_0_heading =
          90.0 - frame_.slot_info_angle[slot_info->id()].first;
      ego_slot_info.pt_0 = ego_slot_info.g2l_tf.GetPos(
          frame_.slot_info_corner_01[slot_info->id()].first);
      ego_slot_info.pt_1 = ego_slot_info.g2l_tf.GetPos(
          frame_.slot_info_corner_01[slot_info->id()].second);
    }
  }
  if (ego_slot_info.pt_0.y() > ego_slot_info.pt_1.y()) {
    std::swap(ego_slot_info.pt_0, ego_slot_info.pt_1);
  }

  ego_slot_info.obs_pt_vec_slot.clear();
  ego_slot_info.fus_obj_valid_flag = frame_.fus_obj_valid_flag;
  if (!ego_slot_info.fus_obj_valid_flag) {
    // use uss obs
    if (frame_.obs_pt_map.count(slot_info->id()) == 0) {
      return true;
    }
    const auto &obs_pt_vec = frame_.obs_pt_map[slot_info->id()];
    ego_slot_info.obs_pt_vec_slot.reserve(obs_pt_vec.size());
    for (const auto &obs_pt : obs_pt_vec) {
      const auto obs_pt_slot = ego_slot_info.g2l_tf.GetPos(obs_pt);
      ego_slot_info.obs_pt_vec_slot.emplace_back(std::move(obs_pt_slot));
    }
  }

  else {
    // use fus obj and ground line
    ego_slot_info.obs_pt_vec_slot.reserve(frame_.obs_pt_vec.size());
    // obs global coord transform to local coord
    for (const auto &obs_pt : frame_.obs_pt_vec) {
      const auto obs_pt_slot = ego_slot_info.g2l_tf.GetPos(obs_pt);
      ego_slot_info.obs_pt_vec_slot.emplace_back(std::move(obs_pt_slot));
    }
  }

  return true;
}

bool SlotManagement::GenTLane(
    EgoSlotInfo &ego_slot_info,
    apa_planner::PerpendicularPathPlanner::Tlane &slot_tlane,
    apa_planner::PerpendicularPathPlanner::Tlane &obs_tlane) {
  using namespace pnc::geometry_lib;

  const Eigen::Vector2d pt_01_norm_vec =
      (ego_slot_info.pt_1 - ego_slot_info.pt_0).normalized();
  const Eigen::Vector2d pt_10_norm_vec =
      (ego_slot_info.pt_0 - ego_slot_info.pt_1).normalized();
  const Eigen::Vector2d pt_01_norm_up_vec(pt_01_norm_vec.y(),
                                          -pt_01_norm_vec.x());
  const Eigen::Vector2d pt_01_norm_down_vec(-pt_01_norm_vec.y(),
                                            pt_01_norm_vec.x());

  const double x_max =
      ((ego_slot_info.pt_1 + ego_slot_info.pt_0) * 0.5 +
       apa_param.GetParam().obs_consider_long_threshold * pt_01_norm_up_vec)
          .x();

  const double y_max = std::fabs(
      (ego_slot_info.pt_1 +
       apa_param.GetParam().obs_consider_lat_threshold * pt_01_norm_vec)
          .y());

  // construct channel width and length, only for uss obs
  if (!ego_slot_info.fus_obj_valid_flag) {
    std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>, Compare>
        channel_width_pq_for_x(Compare(1));
    for (const auto &obstacle_point_slot : ego_slot_info.obs_pt_vec_slot) {
      if (obstacle_point_slot.x() < x_max ||
          std::fabs(obstacle_point_slot.y()) > y_max) {
        continue;
      }
      channel_width_pq_for_x.emplace(obstacle_point_slot);
    }
    if (channel_width_pq_for_x.empty()) {
      channel_width_pq_for_x.emplace(Eigen::Vector2d(
          ((ego_slot_info.pt_1 + ego_slot_info.pt_0) * 0.5).x() +
              apa_param.GetParam().channel_width,
          0.0));
    }

    const double channel_width =
        channel_width_pq_for_x.top().x() -
        ((ego_slot_info.pt_1 + ego_slot_info.pt_0) * 0.5).x();

    ego_slot_info.channel_width = pnc::mathlib::DoubleConstrain(
        channel_width, apa_param.GetParam().min_channel_width,
        apa_param.GetParam().channel_width);
  } else {
    ego_slot_info.channel_width = 10.68;
  }

  // DEBUG_PRINT("channel_width = " << ego_slot_info.channel_width);

  // construct tlane pq
  // left y is positive, right y is negative
  // left y should be smallest, right y should be largest
  // all x should be largest
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>, Compare>
      left_pq_for_y(Compare(3));
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>, Compare>
      left_pq_for_x(Compare(0));
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>, Compare>
      right_pq_for_y(Compare(2));
  std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>, Compare>
      right_pq_for_x(Compare(0));

  // only hack for obs is not accurate
  const double y_min = ego_slot_info.slot_width * 0.5 - 0.068;
  const double x_min = ((ego_slot_info.pt_1 + ego_slot_info.pt_0) * 0.5 +
                        4.28 * pt_01_norm_down_vec)
                           .x();

  // sift obstacles that meet requirement
  for (const auto &obstacle_point_slot : ego_slot_info.obs_pt_vec_slot) {
    if (std::fabs(obstacle_point_slot.x()) > x_max ||
        obstacle_point_slot.x() < x_min ||
        std::fabs(obstacle_point_slot.y()) > y_max ||
        std::fabs(obstacle_point_slot.y()) < y_min) {
      continue;
    }
    if (obstacle_point_slot.y() > 1e-6) {
      left_pq_for_y.emplace(obstacle_point_slot);
      left_pq_for_x.emplace(obstacle_point_slot);
    } else {
      right_pq_for_y.emplace(obstacle_point_slot);
      right_pq_for_x.emplace(obstacle_point_slot);
    }
  }

  if (apa_param.GetParam().conservative_mono_enable) {
    if (!left_pq_for_x.empty() || !right_pq_for_x.empty()) {
      apa_param.SetPram().mono_plan_enable = false;
    }
  }

  // If there are no obstacles on either side, set up a virtual obstacle
  // that is farther away
  const double virtual_x =
      ((ego_slot_info.pt_1 + ego_slot_info.pt_0) * 0.5 +
       apa_param.GetParam().virtual_obs_x_pos * pt_01_norm_down_vec)
          .x();

  const double virtual_left_y =
      (ego_slot_info.pt_1 +
       apa_param.GetParam().virtual_obs_y_pos * pt_01_norm_vec)
          .y();
  const double virtual_right_y =
      (ego_slot_info.pt_0 +
       apa_param.GetParam().virtual_obs_y_pos * pt_10_norm_vec)
          .y();

  bool left_empty = false;
  bool right_empty = false;

  if (left_pq_for_x.empty()) {
    // DEBUG_PRINT("left space is empty");
    left_empty = true;
    left_pq_for_x.emplace(Eigen::Vector2d(virtual_x, 0.0));
    left_pq_for_y.emplace(Eigen::Vector2d(0.0, virtual_left_y));
  }
  if (right_pq_for_x.empty()) {
    // DEBUG_PRINT("right space is empty");
    right_empty = true;
    right_pq_for_x.emplace(Eigen::Vector2d(virtual_x, 0.0));
    right_pq_for_y.emplace(Eigen::Vector2d(0.0, virtual_right_y));
  }

  // If the car is parked according to the actual slot, its leftmost and
  // rightmost coordinates which includes rearview mirror are as follows
  const double car_half_width_with_mirror =
      apa_param.GetParam().max_car_width * 0.5;

  const double virtual_slot_width =
      apa_param.GetParam().max_car_width +
      apa_param.GetParam().slot_compare_to_car_width;

  const double real_slot_width = ego_slot_info.slot_width;

  DEBUG_PRINT("max_car_width = " << apa_param.GetParam().max_car_width
                                 << "  virtual slot width = "
                                 << virtual_slot_width
                                 << "  real slot width = " << real_slot_width);

  const double safe_threshold = apa_param.GetParam().safe_threshold;

  double left_y = left_pq_for_y.top().y();

  double real_left_y = left_y;

  double left_x = left_pq_for_x.top().x();

  double real_left_x = left_x;

  double right_y = right_pq_for_y.top().y();

  double real_right_y = right_y;

  double right_x = right_pq_for_x.top().x();

  double real_right_x = right_x;

  if (apa_param.GetParam().tmp_no_consider_obs_dy &&
      !ego_slot_info.fus_obj_valid_flag) {
    if (!left_empty) {
      left_y = real_slot_width * 0.5 + apa_param.GetParam().tmp_virtual_obs_dy;
    }
    if (!right_empty) {
      right_y =
          -real_slot_width * 0.5 - apa_param.GetParam().tmp_virtual_obs_dy;
    }
  }

  const double threshold = 0.4268;
  if (ego_slot_info.fus_obj_valid_flag) {
    left_y = std::max(left_y, car_half_width_with_mirror + threshold);
    left_x = std::min(left_x, ego_slot_info.pt_1.x() - 1.68 * threshold);
    right_y = std::min(right_y, -car_half_width_with_mirror - threshold);
    right_x = std::min(right_x, ego_slot_info.pt_0.x() - 1.68 * threshold);
  }

  // DEBUG_PRINT("real_left_y = " << real_left_y
  //                              << "  real_right_y = " << real_right_y);

  // DEBUG_PRINT("left_y = " << left_y << "  right_y = " << right_y
  //                         << "  left_x = " << left_x
  //                         << "  right_x = " << right_x);

  // todo: consider actual obs pos to let slot release or not release or
  // move target pose
  double left_dis_obs_car = 0.0;
  double right_dis_obs_car = 0.0;
  if (ego_slot_info.fus_obj_valid_flag) {
    // use fus obj
    if (!apa_param.GetParam().believe_in_fus_obs) {
      real_left_y = std::min(real_left_y, ego_slot_info.pt_1.y());
      real_right_y = std::max(real_right_y, ego_slot_info.pt_0.y());
    }
    left_dis_obs_car = real_left_y - car_half_width_with_mirror;
    right_dis_obs_car = -car_half_width_with_mirror - real_right_y;
  } else {
    // use uss
    left_dis_obs_car = left_y - car_half_width_with_mirror;
    right_dis_obs_car = -car_half_width_with_mirror - right_y;
  }

  DEBUG_PRINT("left_dis_obs_car = " << left_dis_obs_car
                                    << "  right_dis_obs_car = "
                                    << right_dis_obs_car);

  // ensure it can move slot to make both side safe
  if (left_dis_obs_car + right_dis_obs_car < 2.0 * safe_threshold) {
    DEBUG_PRINT("obs to slot safe dist doesnot meet safe_threshold");
    return false;
  }
  bool left_obs_meet_safe_require = false;
  bool right_obs_meet_safe_require = false;
  left_obs_meet_safe_require = left_dis_obs_car > safe_threshold ? true : false;
  right_obs_meet_safe_require =
      right_dis_obs_car > safe_threshold ? true : false;

  bool need_move_slot = false;
  double move_slot_dist = 0.0;
  if (!left_obs_meet_safe_require) {
    // left side is dangerous, should move toward right
    move_slot_dist = safe_threshold - left_dis_obs_car;
    move_slot_dist *= -1.0;
    need_move_slot = true;
  } else if (!right_obs_meet_safe_require) {
    // right side is dangerous, should move toward left
    move_slot_dist = safe_threshold - right_dis_obs_car;
    need_move_slot = true;
  }

  if (need_move_slot) {
    // cal max_move_slot_dist to avoid car press line
    // no consider mirror
    const double half_car_width = apa_param.GetParam().car_width * 0.5;
    const double half_slot_width = ego_slot_info.slot_width * 0.5;
    const double car2line_dist_threshold =
        apa_param.GetParam().car2line_dist_threshold;

    // first sure if car is parked in the center, does it meet the slot line
    // distance requirement
    const double max_move_slot_dist =
        half_slot_width - half_car_width - car2line_dist_threshold;
    if (max_move_slot_dist > 0.0 &&
        (std::fabs(move_slot_dist) > max_move_slot_dist)) {
      if (move_slot_dist > 0.0) {
        move_slot_dist = max_move_slot_dist;
      }
      if (move_slot_dist < 0.0) {
        move_slot_dist = -max_move_slot_dist;
      }
    }
  }

  const auto pM01 =
      0.5 * (ego_slot_info.slot_corner[0] + ego_slot_info.slot_corner[1]);

  const auto pM23 =
      0.5 * (ego_slot_info.slot_corner[2] + ego_slot_info.slot_corner[3]);

  Eigen::Vector2d ego_to_slot_center_vec =
      0.5 * (pM01 + pM23) - frame_.measurement.ego_pos;

  const double cross_ego_to_slot_center =
      pnc::geometry_lib::GetCrossFromTwoVec2d(
          frame_.measurement.ego_heading_vec, ego_to_slot_center_vec);

  const double cross_ego_to_slot_heading =
      pnc::geometry_lib::GetCrossFromTwoVec2d(
          frame_.measurement.ego_heading_vec,
          ego_slot_info.slot_origin_heading_vec);

  if (cross_ego_to_slot_heading > 0.0 && cross_ego_to_slot_center < 0.0) {
    slot_tlane.slot_side = pnc::geometry_lib::SLOT_SIDE_RIGHT;
  } else if (cross_ego_to_slot_heading < 0.0 &&
             cross_ego_to_slot_center > 0.0) {
    slot_tlane.slot_side = pnc::geometry_lib::SLOT_SIDE_LEFT;
  } else {
    slot_tlane.slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;
    return false;
  }

  // construct slot_t_lane_, left is positive, right is negative
  const double slot_width = std::min(virtual_slot_width, real_slot_width);

  Eigen::Vector2d corner_left_slot(ego_slot_info.slot_length, 0.5 * slot_width);

  Eigen::Vector2d corner_right_slot(ego_slot_info.slot_length,
                                    -0.5 * slot_width);

  const auto &slot_side = slot_tlane.slot_side;
  if (slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    // inside is right, outside is left
    slot_tlane.corner_outside_slot = corner_left_slot;
    slot_tlane.corner_inside_slot = corner_right_slot;
    slot_tlane.pt_outside = corner_left_slot;
    slot_tlane.pt_inside = corner_right_slot;
    slot_tlane.pt_inside.x() = std::min(real_right_x, ego_slot_info.pt_0.x()) +
                               apa_param.GetParam().tlane_safe_dx;
    slot_tlane.pt_inside.y() =
        std::max(real_right_y, ego_slot_info.pt_0.y() + 0.05);
  } else if (slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    // outside is right, inside is left
    slot_tlane.corner_outside_slot = corner_right_slot;
    slot_tlane.corner_inside_slot = corner_left_slot;
    slot_tlane.pt_outside = corner_right_slot;
    slot_tlane.pt_inside = corner_left_slot;
    slot_tlane.pt_inside.x() = std::min(real_left_x, ego_slot_info.pt_1.x()) +
                               apa_param.GetParam().tlane_safe_dx;
    slot_tlane.pt_inside.y() =
        std::max(real_left_y, ego_slot_info.pt_1.y() - 0.05);
  }

  slot_tlane.pt_terminal_pos << ego_slot_info.target_ego_pos_slot.x(),
      ego_slot_info.target_ego_pos_slot.y();

  slot_tlane.pt_terminal_heading = ego_slot_info.target_ego_heading_slot;

  if (need_move_slot) {
    slot_tlane.pt_terminal_pos.y() += move_slot_dist;
    slot_tlane.pt_inside.y() += move_slot_dist;
    slot_tlane.pt_outside.y() += move_slot_dist;
    // DEBUG_PRINT(
    //     "should move slot according to obs pt, move dist = " <<
    //     move_slot_dist);
  }

  slot_tlane.pt_lower_boundry_pos = slot_tlane.pt_terminal_pos;
  // subtrace 0.05 to avoid plan failure due to col det
  slot_tlane.pt_lower_boundry_pos.x() =
      slot_tlane.pt_lower_boundry_pos.x() -
      apa_param.GetParam().rear_overhanging -
      apa_param.GetParam().col_obs_safe_dist_normal - 0.05;

  // construct obstacle_t_lane_
  // for onstacle_t_lane    right is inside, left is outside
  obs_tlane.slot_side = slot_side;
  if (slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    obs_tlane.pt_inside.x() = right_x + apa_param.GetParam().obs_safe_dx;
    obs_tlane.pt_inside.y() = right_y;
    obs_tlane.pt_outside.x() = left_x + apa_param.GetParam().obs_safe_dx;
    obs_tlane.pt_outside.y() = left_y;
  } else if (slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    obs_tlane.pt_inside.x() = left_x + apa_param.GetParam().obs_safe_dx;
    obs_tlane.pt_inside.y() = left_y;
    obs_tlane.pt_outside.x() = right_x + apa_param.GetParam().obs_safe_dx;
    obs_tlane.pt_outside.y() = right_y;
  }

  obs_tlane.pt_terminal_pos = slot_tlane.pt_terminal_pos;
  obs_tlane.pt_terminal_heading = slot_tlane.pt_terminal_heading;
  obs_tlane.pt_lower_boundry_pos = slot_tlane.pt_lower_boundry_pos;

  // tmp method, obstacle is temporarily unavailable, force lift slot_t_lane
  // pt_inside and pt_outside
  if (apa_param.GetParam().force_both_side_occupied) {
    slot_tlane.pt_inside.x() = corner_right_slot.x();
    slot_tlane.pt_outside.x() = corner_left_slot.x();

    slot_tlane.pt_inside +=
        Eigen::Vector2d(apa_param.GetParam().occupied_pt_inside_dx,
                        apa_param.GetParam().occupied_pt_inside_dy);

    slot_tlane.pt_outside +=
        Eigen::Vector2d(apa_param.GetParam().occupied_pt_outside_dx,
                        apa_param.GetParam().occupied_pt_outside_dy);
  }

  DEBUG_PRINT("t_lane.pt_inside.x() = " << slot_tlane.pt_inside.x());

  return true;
}

bool SlotManagement::GenObstacles(
    std::shared_ptr<CollisionDetector> &collision_detector_ptr,
    const apa_planner::PerpendicularPathPlanner::Tlane &slot_tlane,
    apa_planner::PerpendicularPathPlanner::Tlane &obs_tlane,
    const EgoSlotInfo &ego_slot_info) {
  if (!collision_detector_ptr) {
    return false;
  }
  collision_detector_ptr->ClearObstacles();
  // set obstacles
  double channel_width = ego_slot_info.channel_width;
  double channel_length = apa_param.GetParam().channel_length;

  if (apa_param.GetParam().force_both_side_occupied) {
    obs_tlane = slot_tlane;
  }

  // add tlane obstacle
  //  B is always outside
  int slot_side = 1;
  bool is_left_side = false;
  if (obs_tlane.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    slot_side = -1;
    is_left_side = false;
  } else if (obs_tlane.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    slot_side = 1;
    is_left_side = true;
  }
  Eigen::Vector2d B(obs_tlane.pt_outside);
  const Eigen::Vector2d pt_01_vec = ego_slot_info.pt_1 - ego_slot_info.pt_0;
  const Eigen::Vector2d pt_01_norm_vec = pt_01_vec.normalized();
  const double obs_length = (channel_length - pt_01_vec.norm()) * 0.5;
  Eigen::Vector2d A = B - slot_side * pt_01_norm_vec * obs_length;

  Eigen::Vector2d C(obs_tlane.pt_lower_boundry_pos);
  C.y() = B.y();

  Eigen::Vector2d E(obs_tlane.pt_inside);
  Eigen::Vector2d D(obs_tlane.pt_lower_boundry_pos);
  D.y() = E.y();

  Eigen::Vector2d F = E + slot_side * pt_01_norm_vec * obs_length;

  // add channel obstacle
  const double pt_01_x = ((ego_slot_info.pt_0 + ego_slot_info.pt_1) * 0.5).x();
  const double top_x = pt_01_x + channel_width / ego_slot_info.sin_angle;
  Eigen::Vector2d channel_point_1 =
      Eigen::Vector2d(top_x, 0.0) -
      slot_side * pt_01_norm_vec * channel_length * 0.5;
  Eigen::Vector2d channel_point_2 =
      Eigen::Vector2d(top_x, 0.0) +
      slot_side * pt_01_norm_vec * channel_length * 0.5;

  Eigen::Vector2d channel_point_3;
  pnc::geometry_lib::LineSegment channel_line;
  std::vector<pnc::geometry_lib::LineSegment> channel_line_vec;
  channel_line.SetPoints(channel_point_1, channel_point_2);
  channel_line_vec.emplace_back(channel_line);
  channel_point_3 = F;
  channel_line.SetPoints(channel_point_2, channel_point_3);
  channel_line_vec.emplace_back(channel_line);

  const double ds = apa_param.GetParam().obstacle_ds;
  std::vector<Eigen::Vector2d> point_set;
  std::vector<Eigen::Vector2d> channel_obstacle_vec;
  channel_obstacle_vec.clear();
  channel_obstacle_vec.reserve(68);
  for (const auto &line : channel_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line, ds);
    channel_obstacle_vec.insert(channel_obstacle_vec.end(), point_set.begin(),
                                point_set.end());
  }

  collision_detector_ptr->SetObstacles(channel_obstacle_vec,
                                       CollisionDetector::CHANNEL_OBS);

  pnc::geometry_lib::LineSegment tlane_line;
  std::vector<pnc::geometry_lib::LineSegment> tlane_line_vec;
  tlane_line.SetPoints(A, B);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(B, C);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(C, D);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(D, E);
  tlane_line_vec.emplace_back(tlane_line);
  tlane_line.SetPoints(E, F);
  tlane_line_vec.emplace_back(tlane_line);

  // tmp method, should modify
  std::vector<Eigen::Vector2d> tlane_obstacle_vec;
  tlane_obstacle_vec.clear();
  tlane_obstacle_vec.reserve(88);
  for (const auto &line : tlane_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line, ds);
    tlane_obstacle_vec.insert(tlane_obstacle_vec.end(), point_set.begin(),
                              point_set.end());
  }
  pnc::geometry_lib::PathPoint ego_pose;
  ego_pose.Set(frame_.ego_slot_info.ego_pos_slot,
               frame_.ego_slot_info.ego_heading_slot);

  if (!frame_.fus_obj_valid_flag) {
    // when no fus obj, temp hack, only increase plan success ratio
    double safe_dist = apa_param.GetParam().max_obs2car_dist_out_slot;
    if (frame_.ego_slot_info.slot_occupied_ratio >
            apa_param.GetParam().max_obs2car_dist_slot_occupied_ratio &&
        std::fabs(frame_.ego_slot_info.terminal_err.heading) * 57.3 < 36.6) {
      safe_dist = apa_param.GetParam().max_obs2car_dist_in_slot;
    }
    for (const auto &obs_pos : tlane_obstacle_vec) {
      if (!collision_detector_ptr->IsObstacleInCar(obs_pos, ego_pose,
                                                   safe_dist)) {
        collision_detector_ptr->AddObstacles(obs_pos,
                                             CollisionDetector::TLANE_OBS);
      }
    }
  }

  else {
    collision_detector_ptr->AddObstacles(tlane_obstacle_vec,
                                         CollisionDetector::TLANE_OBS);

    // add actual fus obs
    Eigen::Vector2d pt_left = obs_tlane.pt_outside;
    Eigen::Vector2d pt_right = obs_tlane.pt_inside;
    if (obs_tlane.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
      std::swap(pt_left, pt_right);
    }
    std::vector<Eigen::Vector2d> fus_obs_vec;
    std::pair<Eigen::Vector2d, Eigen::Vector2d> slot_pt =
        std::make_pair(ego_slot_info.pt_1, ego_slot_info.pt_0);
    for (const auto &obs_pos : ego_slot_info.obs_pt_vec_slot) {
      CollisionDetector::ObsSlotType obs_slot_type =
          collision_detector_ptr->GetObsSlotType(obs_pos, slot_pt,
                                                 is_left_side);

      if (obs_slot_type == CollisionDetector::ObsSlotType::SLOT_IN_OBS &&
          !apa_param.GetParam().believe_in_fus_obs) {
        // obs is in slot, temp hack, lose it, todo, should not del obs
        continue;
      }

      if (obs_slot_type == CollisionDetector::ObsSlotType::SLOT_ENTRANCE_OBS) {
        // obs is slot entrance, when replan, no conside it, but when dynamic
        // move and col det, conside it
        continue;
      }

      if (obs_pos.y() > pt_left.y() && obs_pos.x() < pt_left.x()) {
        // obs is in the lower left T-lane area, lose it
        continue;
      }

      if (obs_pos.y() < pt_right.y() && obs_pos.x() < pt_right.x()) {
        // obs is in the lower right T-lane area, lose it
        continue;
      }

      fus_obs_vec.emplace_back(obs_pos);
    }
    collision_detector_ptr->AddObstacles(fus_obs_vec,
                                         CollisionDetector::FUSION_OBS);
  }

  return true;
}

// select nearby obs pt from ori USS pt for given slot
const bool SlotManagement::AddUssPerceptObstacles(
    const common::SlotInfo &slot_info) {
  // tmp: no consider obs point
  if (apa_param.GetParam().force_both_side_occupied ||
      frame_.uss_percept_info_ptr == NULL) {
    frame_.obs_pt_map.clear();
    return false;
  }
  if (frame_.uss_percept_info_ptr->out_line_dataori[0].obj_pt_cnt == 0) {
    DEBUG_PRINT("obs is empty");
    frame_.obs_pt_map.clear();
    return false;
  }
  const auto &obj_info_desample =
      frame_.uss_percept_info_ptr->out_line_dataori[0];  // 0 means desample
                                                         // while 1 means
                                                         // raw model output

  const size_t selected_id = slot_info.id();
  frame_.obs_pt_map.erase(selected_id);

  const Eigen::Vector2d slot_center(slot_info.center().x(),
                                    slot_info.center().y());

  double filtered_obs_dis = apa_param.GetParam().obs2slot_max_dist;
  if (slot_info.slot_type() == Common::PARKING_SLOT_TYPE_HORIZONTAL) {
    filtered_obs_dis = apa_param.GetParam().parallel_obs2slot_max_dist;
  } else if (slot_info.slot_type() == Common::PARKING_SLOT_TYPE_SLANTING &&
             frame_.slot_info_angle.count(slot_info.id()) != 0) {
    filtered_obs_dis = apa_param.GetParam().obs2slot_max_dist /
                       frame_.slot_info_angle[slot_info.id()].second;
  }

  Eigen::Vector2d obs_pt;
  std::vector<Eigen::Vector2d> slot_obs_vec;
  for (int i = 0; i < obj_info_desample.obj_pt_cnt; ++i) {
    obs_pt << obj_info_desample.obj_pt_global[i].x,
        obj_info_desample.obj_pt_global[i].y;
    const double dist = (slot_center - obs_pt).norm();
    // todo: consider dist from ego to obs
    if (dist < filtered_obs_dis) {
      slot_obs_vec.emplace_back(obs_pt);
    }
  }
  if (!slot_obs_vec.empty()) {
    DEBUG_PRINT("there are obs around slot " << selected_id);
  } else {
    DEBUG_PRINT("there are no obs around slot " << selected_id);
  }
  frame_.obs_pt_map[selected_id] = slot_obs_vec;
  return true;
}

bool SlotManagement::UpdateSlotsInSearching() {
  // DEBUG_PRINT("apa state is in searching!");
  // Update slots
  std::unordered_map<size_t, iflyauto::ParkingFusionSlot> fusion_slot_map;
  for (int i = 0; i < frame_.parking_slot_ptr->parking_fusion_slot_lists_size;
       ++i) {
    const auto &fusion_slot =
        frame_.parking_slot_ptr->parking_fusion_slot_lists[i];
    fusion_slot_map[fusion_slot.id] = fusion_slot;
    common::SlotInfo slot_info;
    if (!ProcessRawSlot(fusion_slot, slot_info)) {
      continue;
    }
    const auto fusion_slot_source_type = fusion_slot.fusion_source;
    if (frame_.slot_info_window_map.count(slot_info.id()) == 0) {  // get new id
      if (LonDifUpdateCondition(slot_info, fusion_slot_source_type)) {
        SlotInfoWindow slot_info_window;
        slot_info_window.Add(slot_info);
        frame_.slot_info_window_map.insert(
            std::make_pair((slot_info.id()), slot_info_window));
      }
    } else {  // get old id
      // slot update strategy
      if (IfUpdateSlot(slot_info, fusion_slot_source_type)) {
        frame_.slot_info_window_map[slot_info.id()].Add(slot_info);
      }
    }
  }

  // delete slot in window map when the slot is not exist in fusion slot
  std::vector<size_t> del_id_vec;
  for (const auto &pair : frame_.slot_info_window_map) {
    if (fusion_slot_map.count(pair.first) == 0) {
      del_id_vec.emplace_back(pair.first);
    }
  }
  for (const size_t &id : del_id_vec) {
    frame_.slot_info_angle.erase(id);
    frame_.slot_info_direction.erase(id);
    frame_.slot_info_corner_01.erase(id);
    frame_.slot_info_window_map.erase(id);
  }

  // assemble slot_management_info
  frame_.slot_management_info.mutable_slot_info_vec()->Clear();
  for (auto &pair : frame_.slot_info_window_map) {
    auto slot = frame_.slot_management_info.add_slot_info_vec();
    // auto slot_info = frame_.slot_info_window_vec[j].GetFusedInfo();
    // ModifySlot2Rectangle(slot_info);
    // *slot = slot_info;
    *slot = pair.second.GetFusedInfo();

    // only extra protect, it can delete to be fast
    if (fusion_slot_map.count(slot->id()) == 0) {
      slot->set_is_release(false);
      slot->set_is_occupied(true);
      continue;
    } else {
      slot->set_is_release(fusion_slot_map[slot->id()].allow_parking == 1);
      slot->set_is_occupied(!slot->is_release());
    }

    if (!apa_param.GetParam().release_slot_by_prepare) {
      continue;
    }
    if ((slot->slot_type() ==
             Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL ||
         slot->slot_type() ==
             Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) &&
        slot->is_release()) {
      if (slot->slot_type() ==
              Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING &&
          frame_.slot_info_direction.count(slot->id()) != 0) {
        if (!frame_.slot_info_direction[slot->id()]) {
          slot->set_is_release(false);
          slot->set_is_occupied(true);
          DEBUG_PRINT("car and slot is no same direction slot id = "
                      << slot->id() << "  slot type = " << slot->slot_type()
                      << "  is_release = " << slot->is_release());
        }
      }

      if (!frame_.fus_obj_valid_flag) {
        // no fus obs, should consider uss obs
        AddUssPerceptObstacles(*slot);
      }

      const double lon_dist = CalLonDistSlot2Car(*slot);
      // DEBUG_PRINT("lon_dist = " << lon_dist);
      // DEBUG_PRINT("angle = " << CalAngleSlot2Car(*slot) * 57.3);

      if (!frame_.fus_obj_valid_flag &&
          lon_dist <
              apa_param.GetParam().min_slot_release_long_dist_slot2mirror &&
          pair.second.GetOccupied()) {
        slot->set_is_release(false);
        slot->set_is_occupied(true);
        DEBUG_PRINT("CalLonDistSlot2Car slot id = "
                    << slot->id() << "  slot type = " << slot->slot_type()
                    << "  is_release = " << slot->is_release());
        continue;
      } else {
        pair.second.SetOccupied(false);
      }

      EgoSlotInfo ego_slot_info;
      // get ego slot info
      if (!UpdateEgoSlotInfo(ego_slot_info, slot)) {
        slot->set_is_release(false);
        slot->set_is_occupied(true);
        DEBUG_PRINT("UpdateEgoSlotInfo slot id = "
                    << slot->id() << "  slot type = " << slot->slot_type()
                    << "  is_release = " << slot->is_release());
        continue;
      }

      // gen T_Lane
      apa_planner::PerpendicularPathPlanner::Tlane slot_tlane;
      planning::apa_planner::PerpendicularPathPlanner::Tlane obs_tlane;
      if (!GenTLane(ego_slot_info, slot_tlane, obs_tlane)) {
        slot->set_is_release(false);
        slot->set_is_occupied(true);
        DEBUG_PRINT("GenTLane slot id = "
                    << slot->id() << "  slot type = " << slot->slot_type()
                    << "  is_release = " << slot->is_release());
        continue;
      }

      apa_planner::PerpendicularPathPlanner::Input path_planner_input;
      path_planner_input.pt_0 = ego_slot_info.pt_0;
      path_planner_input.pt_1 = ego_slot_info.pt_1;
      path_planner_input.sin_angle = ego_slot_info.sin_angle;
      path_planner_input.origin_pt_0_heading =
          ego_slot_info.origin_pt_0_heading;
      path_planner_input.tlane = slot_tlane;
      path_planner_input.ego_pose.Set(ego_slot_info.ego_pos_slot,
                                      ego_slot_info.ego_heading_slot);

      std::shared_ptr<planning::CollisionDetector> collision_detector_ptr =
          std::make_shared<planning::CollisionDetector>();
      if (!GenObstacles(collision_detector_ptr, slot_tlane, obs_tlane,
                        ego_slot_info)) {
        slot->set_is_release(false);
        slot->set_is_occupied(true);
        DEBUG_PRINT("GenObstacles slot id = "
                    << slot->id() << "  slot type = " << slot->slot_type()
                    << "  is_release = " << slot->is_release());
        continue;
      }

      apa_planner::PerpendicularPathPlanner path_planner;
      path_planner.SetInput(path_planner_input);
      path_planner.SetColPtr(collision_detector_ptr);
      if (!path_planner.UpdateByPrePlan()) {
        slot->set_is_release(false);
        slot->set_is_occupied(true);
        DEBUG_PRINT("prepare slot id = "
                    << slot->id() << "  slot type = " << slot->slot_type()
                    << "  is_release = " << slot->is_release());
        continue;
      }

      DEBUG_PRINT("slot id = " << slot->id()
                               << "  slot type = " << slot->slot_type()
                               << "  is_release = " << slot->is_release());
    } else if (slot->slot_type() ==
                   Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL &&
               slot->is_release()) {
      const double lon_dist = CalLonDistSlot2Car(*slot);
      const double parallel_pre_release_lon_dist = 1.3;

      if (frame_.fus_obj_valid_flag) {
        DEBUG_PRINT("use fusion obs, lon_dist = " << lon_dist);
        const size_t slot_id = static_cast<size_t>(slot->id());
        // if slot released by fusion is too late, just add nearby obs once.
        if (frame_.obs_pt_map[slot_id].size() == 0) {
          frame_.obs_pt_map[slot_id] = frame_.obs_pt_vec;
        }

        // then use lon dist to update relatively accurate obs
        if (pnc::mathlib::IsInBound(lon_dist, 0.0,
                                    parallel_pre_release_lon_dist)) {
          // DEBUG_PRINT("frame_.obs_pt_vec size = " <<
          // frame_.obs_pt_vec.size());
          frame_.obs_pt_map[slot_id] = frame_.obs_pt_vec;
        }

        // DEBUG_PRINT("frame_.obs_pt_map[slot_id] size = "
        //             << frame_.obs_pt_map[slot_id].size());
        if (lon_dist < parallel_pre_release_lon_dist &&
            pair.second.GetOccupied()) {
          slot->set_is_release(false);
          slot->set_is_occupied(true);
        } else {
          pair.second.SetOccupied(false);
          slot->set_is_release(true);
          slot->set_is_occupied(false);
        }

      } else {
        DEBUG_PRINT("use uss obs");
        // select nearby obs pt from ori USS pt for given slot
        AddUssPerceptObstacles(*slot);
        if (lon_dist < apa_param.GetParam()
                           .min_parallel_slot_release_long_dist_slot2mirror &&
            pair.second.GetOccupied()) {
          slot->set_is_release(false);
          slot->set_is_occupied(true);
        } else {
          pair.second.SetOccupied(false);
          slot->set_is_release(true);
          slot->set_is_occupied(false);
        }
      }

      DEBUG_PRINT("Parallel slot id = "
                  << slot->id() << "  is_release = " << slot->is_release()
                  << "  is_occupied = " << slot->is_occupied());
    }
  }

  return false;
}

const bool SlotManagement::ProcessRawSlot(
    const iflyauto::ParkingFusionSlot &parking_fusion_slot,
    common::SlotInfo &slot_info) {
  slot_info.Clear();
  if (!SlotInfoTransfer(parking_fusion_slot, slot_info)) {
    DEBUG_PRINT("fusion slot is err");
    return false;
  }

  if (!slot_info.has_corner_points()) {
    DEBUG_PRINT("slot doesnot have corner points");
    return false;
  }

  // check slot is valid
  if (!IsValidParkingSlot(slot_info)) {
    DEBUG_PRINT("slot line is not parallel or vertical");
    return false;
  }

  // correct slot corner point order
  if (CorrectSlotPointsOrder(slot_info)) {
    frame_.fusion_order_error_cnt++;
  }

  // if slot type is slant, should postprocess to vertical slot
  ProcessSlantSlot(slot_info, parking_fusion_slot);

  // make slot more rectangular
  ModifySlot2Rectangle(slot_info);

  return true;
}

const bool SlotManagement::ProcessSlantSlot(
    common::SlotInfo &slot_info,
    const iflyauto::ParkingFusionSlot &parking_fusion_slot) {
  if (slot_info.slot_type() != Common::PARKING_SLOT_TYPE_SLANTING) {
    return false;
  }
  DEBUG_PRINT("slant slot, should postprocess corner to vertical");
  Eigen::Vector2d slot_pt_0(slot_info.corner_points().corner_point(0).x(),
                            slot_info.corner_points().corner_point(0).y());

  Eigen::Vector2d slot_pt_1(slot_info.corner_points().corner_point(1).x(),
                            slot_info.corner_points().corner_point(1).y());

  Eigen::Vector2d slot_pt_2(slot_info.corner_points().corner_point(2).x(),
                            slot_info.corner_points().corner_point(2).y());

  Eigen::Vector2d slot_pt_3(slot_info.corner_points().corner_point(3).x(),
                            slot_info.corner_points().corner_point(3).y());

  frame_.slot_info_corner_01[slot_info.id()] =
      std::make_pair(slot_pt_0, slot_pt_1);

  const Eigen::Vector2d pt_01_vec = slot_pt_1 - slot_pt_0;
  const Eigen::Vector2d pt_01_unit_vec = pt_01_vec.normalized();
  const Eigen::Vector2d pt_02_vec = slot_pt_2 - slot_pt_0;
  const Eigen::Vector2d pt_02_unit_vec = pt_02_vec.normalized();
  const Eigen::Vector2d pt_13_vec = slot_pt_3 - slot_pt_1;
  const Eigen::Vector2d pt_13_unit_vec = pt_13_vec.normalized();

  const double cos_theta = pt_01_unit_vec.dot(pt_02_unit_vec);

  if (cos_theta > 0.0) {
    // toward right
    const double dis_0_0dot = pt_01_vec.dot(pt_02_unit_vec);
    const Eigen::Vector2d pt_0dot = slot_pt_0 + dis_0_0dot * pt_02_unit_vec;
    const double dist_0dot_2 = pt_02_vec.norm() - dis_0_0dot;
    const Eigen::Vector2d pt_3dot = slot_pt_1 + dist_0dot_2 * pt_02_unit_vec;
    slot_info.mutable_corner_points()->mutable_corner_point(0)->set_x(
        pt_0dot.x());
    slot_info.mutable_corner_points()->mutable_corner_point(0)->set_y(
        pt_0dot.y());
    slot_info.mutable_corner_points()->mutable_corner_point(3)->set_x(
        pt_3dot.x());
    slot_info.mutable_corner_points()->mutable_corner_point(3)->set_y(
        pt_3dot.y());
  } else {
    // toward left
    const Eigen::Vector2d pt_10_vec = -pt_01_vec;
    const double dist_1_1dot = pt_10_vec.dot(pt_13_unit_vec);
    const Eigen::Vector2d pt_1dot = slot_pt_1 + dist_1_1dot * pt_13_unit_vec;
    const double dist_1dot_3 = pt_13_vec.norm() - dist_1_1dot;
    const Eigen::Vector2d pt_2dot = slot_pt_0 + dist_1dot_3 * pt_13_unit_vec;
    slot_info.mutable_corner_points()->mutable_corner_point(1)->set_x(
        pt_1dot.x());
    slot_info.mutable_corner_points()->mutable_corner_point(1)->set_y(
        pt_1dot.y());
    slot_info.mutable_corner_points()->mutable_corner_point(2)->set_x(
        pt_2dot.x());
    slot_info.mutable_corner_points()->mutable_corner_point(2)->set_y(
        pt_2dot.y());
  }

  double accumulated_x = 0.0;
  double accumulated_y = 0.0;
  for (size_t i = 0; i < 4; ++i) {
    accumulated_x += slot_info.corner_points().corner_point(i).x();
    accumulated_y += slot_info.corner_points().corner_point(i).y();
  }

  slot_info.mutable_center()->set_x(accumulated_x / 4.0);
  slot_info.mutable_center()->set_y(accumulated_y / 4.0);

  // cal slot angle
  // get origin slant slot info
  const Eigen::Vector2d origin_pt_01_vec = pt_01_vec;

  slot_pt_0 << slot_info.corner_points().corner_point(0).x(),
      slot_info.corner_points().corner_point(0).y();
  slot_pt_1 << slot_info.corner_points().corner_point(1).x(),
      slot_info.corner_points().corner_point(1).y();
  slot_pt_2 << slot_info.corner_points().corner_point(2).x(),
      slot_info.corner_points().corner_point(2).y();
  slot_pt_3 << slot_info.corner_points().corner_point(3).x(),
      slot_info.corner_points().corner_point(3).y();

  const Eigen::Vector2d pt_23mid_01_mid =
      (slot_pt_0 + slot_pt_1 - slot_pt_2 - slot_pt_3) * 0.5;

  double angle = std::fabs(pnc::geometry_lib::GetAngleFromTwoVec(
                     pt_23mid_01_mid, origin_pt_01_vec)) *
                 57.3;

  if (angle > 90.0) {
    angle = 180.0 - angle;
  }
  angle = pnc::mathlib::DoubleConstrain(angle, 10.0, 80.0);
  double sin_angle = std::sin(angle / 57.3);
  frame_.slot_info_angle[slot_info.id()] = std::make_pair(angle, sin_angle);

  const Eigen::Vector2d slot_heading_vec = pt_23mid_01_mid;
  const Eigen::Vector2d ego_heading_vec = frame_.measurement.ego_heading_vec;
  const Eigen::Vector2d slot_center =
      Eigen::Vector2d(slot_info.center().x(), slot_info.center().y());

  const Eigen::Vector2d ego_slot_vec =
      slot_center - Eigen::Vector2d(frame_.measurement.ego_pos.x(),
                                    frame_.measurement.ego_pos.y());

  const double cross_ego_slot_heading = pnc::geometry_lib::GetCrossFromTwoVec2d(
      ego_heading_vec, slot_heading_vec);
  const double cross_ego_slot_center =
      pnc::geometry_lib::GetCrossFromTwoVec2d(ego_heading_vec, ego_slot_vec);

  size_t slot_side = pnc::geometry_lib::SLOT_SIDE_RIGHT;
  if (cross_ego_slot_heading > 0.0 && cross_ego_slot_center < 0.0) {
    slot_side = pnc::geometry_lib::SLOT_SIDE_RIGHT;
  } else if (cross_ego_slot_heading < 0.0 && cross_ego_slot_center > 0.0) {
    slot_side = pnc::geometry_lib::SLOT_SIDE_LEFT;
  } else {
    return false;
  }

  bool is_same_direction = false;
  if (slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    if (cos_theta > 0.0) {
      is_same_direction = true;
    }
  } else {
    if (cos_theta < 0.0) {
      is_same_direction = true;
    }
  }
  frame_.slot_info_direction[slot_info.id()] = is_same_direction;

  return true;
}

common::SlotInfo SlotManagement::SlotInfoTransfer(
    const iflyauto::ParkingFusionSlot &fusion_slot) {
  double accumulated_x = 0.0;
  double accumulated_y = 0.0;
  common::SlotInfo slot_info;
  static const auto fusion_slots_size = 4;
  for (auto j = 0; j < fusion_slots_size; j++) {
    auto add_point = slot_info.mutable_corner_points()->add_corner_point();
    add_point->set_x(fusion_slot.corner_points[j].x);
    add_point->set_y(fusion_slot.corner_points[j].y);

    accumulated_x += fusion_slot.corner_points[j].x;
    accumulated_y += fusion_slot.corner_points[j].y;
  }

  slot_info.mutable_center()->set_x(accumulated_x /
                                    static_cast<double>(fusion_slots_size));

  slot_info.mutable_center()->set_y(accumulated_y /
                                    static_cast<double>(fusion_slots_size));

  slot_info.set_id(fusion_slot.id);

  if (IsInSearchingState()) {
    slot_info.set_is_release((fusion_slot.allow_parking == 1));
    slot_info.set_is_occupied((fusion_slot.allow_parking == 0));
  }

  if (IsInParkingState()) {
    // the selected slot in parking state is forced to release
    slot_info.set_is_release(true);
    slot_info.set_is_occupied(false);
  }

  slot_info.set_slot_type(fusion_slot.type);

  return slot_info;
}

const bool SlotManagement::SlotInfoTransfer(
    const iflyauto::ParkingFusionSlot &fusion_slot,
    common::SlotInfo &slot_info) {
  double accumulated_x = 0.0;
  double accumulated_y = 0.0;
  static const int fusion_slots_size = 4;
  if (NUM_OF_CORNER_POINT_NUM != fusion_slots_size) {
    return false;
  }
  for (int j = 0; j < fusion_slots_size; j++) {
    auto add_point = slot_info.mutable_corner_points()->add_corner_point();
    add_point->set_x(fusion_slot.corner_points[j].x);
    add_point->set_y(fusion_slot.corner_points[j].y);

    accumulated_x += fusion_slot.corner_points[j].x;
    accumulated_y += fusion_slot.corner_points[j].y;
  }

  slot_info.mutable_center()->set_x(accumulated_x /
                                    static_cast<double>(fusion_slots_size));

  slot_info.mutable_center()->set_y(accumulated_y /
                                    static_cast<double>(fusion_slots_size));

  slot_info.set_id(fusion_slot.id);

  if (IsInSearchingState()) {
    slot_info.set_is_release((fusion_slot.allow_parking == 1));
    slot_info.set_is_occupied((fusion_slot.allow_parking == 0));
  }

  if (IsInParkingState()) {
    // the selected slot in parking state is forced to release
    slot_info.set_is_release(true);
    slot_info.set_is_occupied(false);
  }

  slot_info.set_slot_type(fusion_slot.type);

  return true;
}

void SlotManagement::ModifySlot2Rectangle(common::SlotInfo &slot_info) {
  std::vector<Eigen::Vector2d> original_vertices;
  std::vector<Eigen::Vector2d> target_boundingbox;
  original_vertices.reserve(4);
  original_vertices.clear();
  target_boundingbox.reserve(4);
  target_boundingbox.clear();
  for (google::protobuf::int32 i = 0;
       i < slot_info.corner_points().corner_point_size(); i++) {
    original_vertices.emplace_back(
        Eigen::Vector2d(slot_info.corner_points().corner_point(i).x(),
                        slot_info.corner_points().corner_point(i).y()));
  }

  const auto is_need_correct = pnc::geometry_lib::MinimumBoundingBox(
      original_vertices, target_boundingbox);

  if (is_need_correct) {
    DEBUG_PRINT("slot should modify to rectangle");
    for (size_t i = 0; i < target_boundingbox.size(); i++) {
      DEBUG_PRINT(i << " : " << target_boundingbox[i].x() << " "
                    << target_boundingbox[i].y() << " ");
    }
    slot_info.mutable_corner_points()->mutable_corner_point(0)->set_x(
        target_boundingbox[0].x());
    slot_info.mutable_corner_points()->mutable_corner_point(0)->set_y(
        target_boundingbox[0].y());

    slot_info.mutable_corner_points()->mutable_corner_point(1)->set_x(
        target_boundingbox[1].x());
    slot_info.mutable_corner_points()->mutable_corner_point(1)->set_y(
        target_boundingbox[1].y());

    slot_info.mutable_corner_points()->mutable_corner_point(2)->set_x(
        target_boundingbox[2].x());
    slot_info.mutable_corner_points()->mutable_corner_point(2)->set_y(
        target_boundingbox[2].y());

    slot_info.mutable_corner_points()->mutable_corner_point(3)->set_x(
        target_boundingbox[3].x());
    slot_info.mutable_corner_points()->mutable_corner_point(3)->set_y(
        target_boundingbox[3].y());
  }
}

bool SlotManagement::IsValidParkingSlot(
    const common::SlotInfo &slot_info) const {
  if (slot_info.slot_type() ==
      Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL) {
    return true;
  }
  const auto &pts = slot_info.corner_points();

  Eigen::Vector2d slot_line02_vec(
      pts.corner_point(2).x() - pts.corner_point(0).x(),
      pts.corner_point(2).y() - pts.corner_point(0).y());

  Eigen::Vector2d slot_line13_vec(
      pts.corner_point(3).x() - pts.corner_point(1).x(),
      pts.corner_point(3).y() - pts.corner_point(1).y());

  Eigen::Vector2d slot_line23_vec(
      pts.corner_point(3).x() - pts.corner_point(2).x(),
      pts.corner_point(3).y() - pts.corner_point(2).y());

  Eigen::Vector2d slot_line01_vec(
      pts.corner_point(1).x() - pts.corner_point(0).x(),
      pts.corner_point(1).y() - pts.corner_point(0).y());

  // 1. Check if the boundary lines 02 and 13 of the parking
  // slot are approximately parallel
  const double slot_line_angle_dif = std::fabs(
      pnc::transform::GetAngleFromTwoVec(slot_line02_vec, slot_line13_vec));
  const double slot_line_angle_dif_deg = slot_line_angle_dif * 57.3;

  const bool slot_line_parallel_condition =
      slot_line_angle_dif_deg <=
      apa_param.GetParam().max_slot_boundary_line_angle_dif_deg;

  if (!slot_line_parallel_condition) {
    return false;
  }

  if (slot_info.slot_type() ==
      Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) {
    return true;
  }

  // 2.  Check if the nearby boundary lines of the parking
  // slot are approximately vertical
  Eigen::Vector2d slot_line20_vec = -slot_line02_vec;
  const double corner2_angle_dif =
      std::fabs(kPie * 0.5 - pnc::transform::GetAngleFromTwoVec(
                                 slot_line20_vec, slot_line23_vec));
  const double corner3_angle_dif =
      std::fabs(kPie * 0.5 - pnc::transform::GetAngleFromTwoVec(
                                 slot_line13_vec, slot_line23_vec));
  const double max_corner_angle_dif =
      std::max(corner2_angle_dif, corner3_angle_dif);
  const bool corner_vertical_condition =
      max_corner_angle_dif <=
      apa_param.GetParam().max_slot_boundary_line_angle_dif_deg;
  if (corner_vertical_condition) {
    return true;
  } else {
    return false;
  }
}

bool SlotManagement::CorrectSlotPointsOrder(common::SlotInfo &slot_info) const {
  Eigen::Vector2d slot_pt_0(slot_info.corner_points().corner_point(0).x(),
                            slot_info.corner_points().corner_point(0).y());

  Eigen::Vector2d slot_pt_1(slot_info.corner_points().corner_point(1).x(),
                            slot_info.corner_points().corner_point(1).y());

  Eigen::Vector2d slot_pt_2(slot_info.corner_points().corner_point(2).x(),
                            slot_info.corner_points().corner_point(2).y());

  Eigen::Vector2d slot_pt_3(slot_info.corner_points().corner_point(3).x(),
                            slot_info.corner_points().corner_point(3).y());

  Eigen::Vector2d middle_pt_01 = (slot_pt_0 + slot_pt_1) * 0.5;
  Eigen::Vector2d middle_pt_23 = (slot_pt_2 + slot_pt_3) * 0.5;

  Eigen::Vector2d slot_middle_vec = middle_pt_01 - middle_pt_23;

  Eigen::Vector2d middle_pt23_to_pt0_vec = slot_pt_0 - middle_pt_23;

  const double cross = slot_middle_vec(0) * middle_pt23_to_pt0_vec(1) -
                       slot_middle_vec(1) * middle_pt23_to_pt0_vec(0);
  // slot pt 0 need to change with pt 1   , 2 <->3
  if (cross > 0) {
    slot_info.mutable_corner_points()->mutable_corner_point(0)->set_x(
        slot_pt_1(0));
    slot_info.mutable_corner_points()->mutable_corner_point(0)->set_y(
        slot_pt_1(1));

    slot_info.mutable_corner_points()->mutable_corner_point(1)->set_x(
        slot_pt_0(0));
    slot_info.mutable_corner_points()->mutable_corner_point(1)->set_y(
        slot_pt_0(1));

    slot_info.mutable_corner_points()->mutable_corner_point(2)->set_x(
        slot_pt_3(0));
    slot_info.mutable_corner_points()->mutable_corner_point(2)->set_y(
        slot_pt_3(1));

    slot_info.mutable_corner_points()->mutable_corner_point(3)->set_x(
        slot_pt_2(0));
    slot_info.mutable_corner_points()->mutable_corner_point(3)->set_y(
        slot_pt_2(1));
    return true;
  }
  return false;
}

bool SlotManagement::IfUpdateSlot(const common::SlotInfo &new_slot_info,
                                  const size_t fusion_slot_source_type) {
  if ((fusion_slot_source_type ==
       iflyauto::SlotSourceType::SLOT_SOURCE_TYPE_ONLY_USS) ||
      (fusion_slot_source_type ==
       iflyauto::SlotSourceType::SLOT_SOURCE_TYPE_CAMERA_USS)) {
    // DEBUG_PRINT("it is uss slot");
    return true;
  }
  // DEBUG_PRINT("it is vision slot");
  // update by angle between ego_heading_axis and slot_heading_axis (new
  // slot)
  const bool angle_update_condition = AngleUpdateCondition(new_slot_info);

  // update by lon dif between slot center and mirror middle point
  const bool lon_update_condition =
      LonDifUpdateCondition(new_slot_info, fusion_slot_source_type);

  return true || (angle_update_condition && lon_update_condition);
}

bool SlotManagement::LonDifUpdateCondition(
    const common::SlotInfo &new_slot_info,
    const size_t parking_fusion_slot_source_type) {
  if ((parking_fusion_slot_source_type ==
       iflyauto::SLOT_SOURCE_TYPE_ONLY_USS) ||
      (parking_fusion_slot_source_type ==
       iflyauto::SLOT_SOURCE_TYPE_CAMERA_USS)) {
    return true;
  }

  const auto &measurement = frame_.measurement;
  const auto slot_pts = new_slot_info.corner_points().corner_point();
  const Eigen::Vector2d ego_pos_to_pt0_vec(
      slot_pts[0].x() - measurement.ego_pos.x(),
      slot_pts[0].y() - measurement.ego_pos.y());

  const Eigen::Vector2d ego_pos_to_pt2_vec(
      slot_pts[2].x() - measurement.ego_pos.x(),
      slot_pts[2].y() - measurement.ego_pos.y());

  const auto ego_unit_heading =
      pnc::geometry_lib::GetUnitTangVecByHeading(measurement.heading);

  const Eigen::Vector2d slot_pt01_mid(
      (slot_pts[0].x() + slot_pts[1].x()) * 0.5,
      (slot_pts[0].y() + slot_pts[1].y()) * 0.5);

  const Eigen::Vector2d slot_pt23_mid(
      (slot_pts[2].x() + slot_pts[3].x()) * 0.5,
      (slot_pts[2].y() + slot_pts[3].y()) * 0.5);

  const auto slot_heading_vec_unit =
      (slot_pt01_mid - slot_pt23_mid).normalized();

  bool lon_dif_update_condition = false;

  if (new_slot_info.slot_type() ==
      Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL) {
    const auto cross_product = pnc::geometry_lib::GetCrossFromTwoVec2d(
        ego_unit_heading, ego_pos_to_pt0_vec);

    Eigen::Vector2d mirror_pos;
    if (cross_product < -1e-5) {
      // right side slot
      mirror_pos = measurement.right_mirror_pos;
    } else if (cross_product > 1e-5) {
      // left side slot
      mirror_pos = measurement.left_mirror_pos;
    } else {
      return false;
    }

    const Eigen::Vector2d slot_center_to_mirror_vec(
        mirror_pos.x() - new_slot_info.center().x(),
        mirror_pos.y() - new_slot_info.center().y());

    // longitudinal distance from slot center to car mirror
    double lon_dist = pnc::geometry_lib::GetCrossFromTwoVec2d(
        slot_heading_vec_unit, slot_center_to_mirror_vec);

    bool outside_case = false;
    bool inside_case = false;
    if (cross_product < -1e-5) {
      // right side
      outside_case =
          lon_dist >= apa_param.GetParam().outside_lon_dist_min_slot2mirror &&
          lon_dist <= apa_param.GetParam().outside_lon_dist_max_slot2mirror;
      inside_case =
          lon_dist >= -apa_param.GetParam().inside_lon_dist_max_slot2mirror &&
          lon_dist <= -apa_param.GetParam().inside_lon_dist_min_slot2mirror;
    } else {
      // left side
      outside_case =
          lon_dist >= -apa_param.GetParam().outside_lon_dist_max_slot2mirror &&
          lon_dist <= -apa_param.GetParam().outside_lon_dist_min_slot2mirror;
      inside_case =
          lon_dist >= apa_param.GetParam().inside_lon_dist_min_slot2mirror &&
          lon_dist <= apa_param.GetParam().inside_lon_dist_max_slot2mirror;
    }
    lon_dif_update_condition = outside_case || inside_case;
  }

  else if (new_slot_info.slot_type() ==
           Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL) {
    // calc slot side first
    const bool is_left_side =
        (pnc::geometry_lib::GetCrossFromTwoVec2d(
             frame_.measurement.ego_heading_vec, ego_pos_to_pt2_vec) > 0.0);

    Eigen::Vector2d mirror_pos;
    if (!is_left_side) {
      // right side slot
      mirror_pos = measurement.right_mirror_pos;
    } else if (is_left_side) {
      // left side slot
      mirror_pos = measurement.left_mirror_pos;
    } else {
      return false;
    }

    const Eigen::Vector2d slot_center_to_mirror_vec(
        mirror_pos.x() - new_slot_info.center().x(),
        mirror_pos.y() - new_slot_info.center().y());

    // make lon_dif has the meaning of positive value crossing the slot
    // center line, negative value before the slot center line
    double lon_dif = pnc::geometry_lib::GetCrossFromTwoVec2d(
        slot_heading_vec_unit, slot_center_to_mirror_vec);
    if (!is_left_side) {
      lon_dif = -lon_dif;
    }

    // DEBUG_PRINT("---parallel slot id =" << new_slot_info.id()
    //           << " type =" << new_slot_info.slot_type());
    // DEBUG_PRINT("parallel is left side =" << is_left_side);
    // DEBUG_PRINT("lon dif =" << lon_dif);

    lon_dif_update_condition = pnc::mathlib::IsInBound(lon_dif, -5.0, -1.7) ||
                               pnc::mathlib::IsInBound(lon_dif, 0.3, 1.0);
  }

  else if (new_slot_info.slot_type() ==
           Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) {
    const auto cross_product = pnc::geometry_lib::GetCrossFromTwoVec2d(
        ego_unit_heading, ego_pos_to_pt0_vec);

    Eigen::Vector2d mirror_pos;
    if (cross_product < -1e-5) {
      // right side slot
      mirror_pos = measurement.right_mirror_pos;
    } else if (cross_product > 1e-5) {
      // left side slot
      mirror_pos = measurement.left_mirror_pos;
    } else {
      return false;
    }

    Eigen::Vector2d origin_pt_0 =
        Eigen::Vector2d(slot_pts[0].x(), slot_pts[0].y());
    Eigen::Vector2d origin_pt_1 =
        Eigen::Vector2d(slot_pts[1].x(), slot_pts[1].y());

    if (frame_.slot_info_corner_01.count(new_slot_info.id()) != 0) {
      origin_pt_0 = frame_.slot_info_corner_01[new_slot_info.id()].first;
      origin_pt_1 = frame_.slot_info_corner_01[new_slot_info.id()].second;
    }

    const Eigen::Vector2d origin_pt_01_vec = origin_pt_1 - origin_pt_0;
    const Eigen::Vector2d origin_pt_01_vec_n(origin_pt_01_vec.y(),
                                             -origin_pt_01_vec.x());
    const Eigen::Vector2d origin_pt_01_mid = (origin_pt_0 + origin_pt_1) * 0.5;
    pnc::geometry_lib::LineSegment line(
        origin_pt_01_mid, origin_pt_01_mid + origin_pt_01_vec_n.normalized());

    double lon_dist = pnc::geometry_lib::CalPoint2LineDist(mirror_pos, line);

    const Eigen::Vector2d pt_01_mid_mirr_vec = mirror_pos - origin_pt_01_mid;
    const double cros = pnc::geometry_lib::GetCrossFromTwoVec2d(
        origin_pt_01_vec_n, pt_01_mid_mirr_vec);

    if (cros < 0.0) {
      // car is at inside
      lon_dist *= -1.0;
    }

    bool outside_case = false;
    bool inside_case = false;
    if (cross_product < -1e-5) {
      // right side
      outside_case =
          lon_dist >= apa_param.GetParam().outside_lon_dist_min_slot2mirror &&
          lon_dist <= apa_param.GetParam().outside_lon_dist_max_slot2mirror;
      inside_case =
          lon_dist >= -apa_param.GetParam().inside_lon_dist_max_slot2mirror &&
          lon_dist <= -apa_param.GetParam().inside_lon_dist_min_slot2mirror;
    } else {
      // left side
      outside_case =
          lon_dist >= -apa_param.GetParam().outside_lon_dist_max_slot2mirror &&
          lon_dist <= -apa_param.GetParam().outside_lon_dist_min_slot2mirror;
      inside_case =
          lon_dist >= apa_param.GetParam().inside_lon_dist_min_slot2mirror &&
          lon_dist <= apa_param.GetParam().inside_lon_dist_max_slot2mirror;
    }
    lon_dif_update_condition = outside_case || inside_case;
  }

  return true || lon_dif_update_condition;
}

const double SlotManagement::CalLonDistSlot2Car(
    const common::SlotInfo &new_slot_info) const {
  const auto &measurement = frame_.measurement;
  const auto slot_pts = new_slot_info.corner_points().corner_point();

  Eigen::Vector2d origin_pt_0 =
      Eigen::Vector2d(slot_pts[0].x(), slot_pts[0].y());
  Eigen::Vector2d origin_pt_1 =
      Eigen::Vector2d(slot_pts[1].x(), slot_pts[1].y());

  if (new_slot_info.slot_type() == Common::PARKING_SLOT_TYPE_SLANTING &&
      frame_.slot_info_corner_01.count(new_slot_info.id()) != 0) {
    origin_pt_0 = frame_.slot_info_corner_01.at(new_slot_info.id()).first;
    origin_pt_1 = frame_.slot_info_corner_01.at(new_slot_info.id()).second;
  }

  const Eigen::Vector2d ego_pos_to_pt0_vec = origin_pt_0 - measurement.ego_pos;

  const Eigen::Vector2d ego_unit_heading =
      pnc::geometry_lib::GetUnitTangVecByHeading(measurement.heading);

  const double cross_product = pnc::geometry_lib::GetCrossFromTwoVec2d(
      ego_unit_heading, ego_pos_to_pt0_vec);

  Eigen::Vector2d mirror_pos;
  if (cross_product < -1e-5) {
    // right side slot
    mirror_pos = measurement.right_mirror_pos;
  } else if (cross_product > 1e-5) {
    // left side slot
    mirror_pos = measurement.left_mirror_pos;
  }

  const Eigen::Vector2d origin_pt_01_vec = origin_pt_1 - origin_pt_0;
  const Eigen::Vector2d origin_pt_01_vec_n(origin_pt_01_vec.y(),
                                           -origin_pt_01_vec.x());

  const Eigen::Vector2d origin_pt_01_mid = (origin_pt_0 + origin_pt_1) * 0.5;
  pnc::geometry_lib::LineSegment line(
      origin_pt_01_mid, origin_pt_01_mid + origin_pt_01_vec_n.normalized());

  double lon_dist = pnc::geometry_lib::CalPoint2LineDist(mirror_pos, line);

  const Eigen::Vector2d pt_01_mid_mirr_vec = mirror_pos - origin_pt_01_mid;
  const double cros = pnc::geometry_lib::GetCrossFromTwoVec2d(
      origin_pt_01_vec_n, pt_01_mid_mirr_vec);

  // when car is at outside, lon_dist should be negative
  if ((cross_product < 1e-5 && cros > 0.0) ||
      (cross_product > 1e-5 && cros < 0.0)) {
    // right side slot and car is at outside
    // left side slot and car is at outside
    lon_dist *= -1.0;
  }

  return lon_dist;
}

bool SlotManagement::AngleUpdateCondition(
    const common::SlotInfo &new_slot_info) {
  const Measurement measurement = frame_.measurement;

  const Eigen::Vector2d mirror_center_pos =
      (measurement.left_mirror_pos + measurement.right_mirror_pos) * 0.5;

  const Eigen::Vector2d mirror_center_to_slot_center_vec(
      new_slot_info.center().x() - mirror_center_pos.x(),
      new_slot_info.center().y() - mirror_center_pos.y());

  double angle = std::fabs(pnc::transform::GetAngleFromTwoVec(
      measurement.ego_heading_vec, mirror_center_to_slot_center_vec));

  const double angle_offset = pnc::mathlib::Deg2Rad(
      apa_param.GetParam().max_slots_update_angle_dis_limit_deg);

  if (new_slot_info.slot_type() == Common::PARKING_SLOT_TYPE_SLANTING &&
      frame_.slot_info_corner_01.count(new_slot_info.id()) != 0) {
    const Eigen::Vector2d origin_pt_0 =
        frame_.slot_info_corner_01[new_slot_info.id()].first;
    const Eigen::Vector2d origin_pt_1 =
        frame_.slot_info_corner_01[new_slot_info.id()].second;

    const Eigen::Vector2d origin_pt_01_vec = origin_pt_1 - origin_pt_0;
    const Eigen::Vector2d origin_pt_01_vec_n_down(-origin_pt_01_vec.y(),
                                                  origin_pt_01_vec.x());
    const Eigen::Vector2d origin_pt_01_mid = (origin_pt_0 + origin_pt_1) * 0.5;
    const Eigen::Vector2d slot_center =
        origin_pt_01_mid + 2.5 * origin_pt_01_vec_n_down;
    const Eigen::Vector2d mirror_center_vec = slot_center - mirror_center_pos;
    angle = std::fabs(pnc::transform::GetAngleFromTwoVec(
        measurement.ego_heading_vec, mirror_center_vec));
  }

  const double mid_angle = 0.5 * kPie;

  return pnc::mathlib::IsInBound(angle, mid_angle - angle_offset,
                                 mid_angle + angle_offset);
}

const double SlotManagement::CalAngleSlot2Car(
    const common::SlotInfo &new_slot_info) const {
  const Measurement measurement = frame_.measurement;

  const Eigen::Vector2d mirror_center_pos =
      (measurement.left_mirror_pos + measurement.right_mirror_pos) * 0.5;

  const Eigen::Vector2d mirror_center_to_slot_center_vec(
      new_slot_info.center().x() - mirror_center_pos.x(),
      new_slot_info.center().y() - mirror_center_pos.y());

  double angle = std::fabs(pnc::transform::GetAngleFromTwoVec(
      measurement.ego_heading_vec, mirror_center_to_slot_center_vec));

  if (new_slot_info.slot_type() == Common::PARKING_SLOT_TYPE_SLANTING &&
      frame_.slot_info_corner_01.count(new_slot_info.id()) != 0) {
    const Eigen::Vector2d origin_pt_0 =
        frame_.slot_info_corner_01.at(new_slot_info.id()).first;
    const Eigen::Vector2d origin_pt_1 =
        frame_.slot_info_corner_01.at(new_slot_info.id()).second;

    const Eigen::Vector2d origin_pt_01_vec = origin_pt_1 - origin_pt_0;
    const Eigen::Vector2d origin_pt_01_vec_n_down(-origin_pt_01_vec.y(),
                                                  origin_pt_01_vec.x());
    const Eigen::Vector2d origin_pt_01_mid = (origin_pt_0 + origin_pt_1) * 0.5;
    const Eigen::Vector2d slot_center =
        origin_pt_01_mid + 2.5 * origin_pt_01_vec_n_down;
    const Eigen::Vector2d mirror_center_vec = slot_center - mirror_center_pos;
    angle = std::fabs(pnc::transform::GetAngleFromTwoVec(
        measurement.ego_heading_vec, mirror_center_vec));
  }

  return angle;
}

bool SlotManagement::IsInParkingState() const {
  if ((frame_.func_state_ptr->current_state ==
           iflyauto::FunctionalState_PARK_IN_ACTIVATE_WAIT ||
       frame_.func_state_ptr->current_state ==
           iflyauto::FunctionalState_PARK_IN_ACTIVATE_CONTROL ||
       frame_.func_state_ptr->current_state ==
           iflyauto::FunctionalState_PARK_IN_SUSPEND_ACTIVATE ||
       frame_.func_state_ptr->current_state ==
           iflyauto::FunctionalState_PARK_IN_SUSPEND_CLOSE) ||
      (frame_.param.force_apa_on && frame_.param.is_switch_parking)) {
    return true;
  }
  return false;
}

bool SlotManagement::UpdateSlotsInParking() {
  DEBUG_PRINT("apa state is in parking");
  if (frame_.parking_slot_ptr->select_slot_id == 0) {
    DEBUG_PRINT("Error: no selected id");
    return false;
  }

  const size_t select_slot_id = frame_.parking_slot_ptr->select_slot_id;
  if (select_slot_id == 0) {
    DEBUG_PRINT("select_slot_id = 0, is not valid");
    return false;
  }
  DEBUG_PRINT("select_slot_id:" << select_slot_id);

  if (frame_.slot_info_window_map.count(select_slot_id) == 0) {
    DEBUG_PRINT("select slot is not in slot_info_window_vec");
    return false;
  }

  DEBUG_PRINT("select slot is in slot_info_window_map");
  if (frame_.slot_info_window_map.empty() ||
      frame_.slot_info_window_map[select_slot_id].IsEmpty()) {
    DEBUG_PRINT("slot_info_window_map is empty!");
    return false;
  }

  iflyauto::ParkingFusionSlot select_fusion_slot;
  bool valid_select_slot = false;
  for (int i = 0; i < frame_.parking_slot_ptr->parking_fusion_slot_lists_size;
       ++i) {
    const auto &fusion_slot =
        frame_.parking_slot_ptr->parking_fusion_slot_lists[i];
    if (select_slot_id == fusion_slot.id) {
      select_fusion_slot = fusion_slot;
      if (fusion_slot.type == iflyauto::PARKING_SLOT_TYPE_VERTICAL) {
        DEBUG_PRINT("perpendicular slot selected in fusion");
      } else if (fusion_slot.type == iflyauto::PARKING_SLOT_TYPE_HORIZONTAL) {
        DEBUG_PRINT("parallel slot selected in fusion");
      } else if (fusion_slot.type == iflyauto::PARKING_SLOT_TYPE_SLANTING) {
        DEBUG_PRINT("slant slot selected in fusion");
      } else {
        DEBUG_PRINT("current slot selected is no supported");
        break;
      }
      valid_select_slot = true;
      break;
    }
  }

  if (!valid_select_slot) {
    DEBUG_PRINT("selected slot is invalid!");
    return false;
  }

  common::SlotInfo select_slot;
  if (!ProcessRawSlot(select_fusion_slot, select_slot)) {
    select_slot = frame_.slot_info_window_map[select_slot_id].GetFusedInfo();
    select_slot.set_is_release(true);
    select_slot.set_is_occupied(false);
  }

  // make sure apa is always running when begin, todo, should change
  // according to fusion current is only put a patch
  if (select_slot.is_release() == false) {
    DEBUG_PRINT("selected slot is not released!");
    return false;
  }

  if (select_slot.slot_type() ==
          Common::ParkingSlotType::PARKING_SLOT_TYPE_VERTICAL ||
      select_slot.slot_type() ==
          Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) {
    // only use the obs when searching
    // AddUssPerceptObstacles(select_slot);
    if (!UpdateEgoSlotInfo(select_slot_id, select_slot, select_fusion_slot)) {
      return false;
    }
    UpdateSlotInfoInParking();

    UpdateLimiterInfoInParking();
  }

  else if (select_slot.slot_type() ==
           Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL) {
    // AddUssPerceptObstacles(select_slot);
    if (!UpdateEgoParallelSlotInfo(select_slot_id, select_slot,
                                   select_fusion_slot)) {
      return false;
    }
    UpdateParallelSlotInfoInParking();
  }

  else if (select_slot.slot_type() ==
           Common::ParkingSlotType::PARKING_SLOT_TYPE_SLANTING) {
  }

  return true;
}

bool SlotManagement::UpdateEgoSlotInfo(
    const google::protobuf::uint32 &select_slot_id,
    const common::SlotInfo &select_slot,
    const iflyauto::ParkingFusionSlot &select_fusion_slot) {
  auto &ego_slot_info = frame_.ego_slot_info;
  auto &ego_pose_info = frame_.measurement;

  if (select_fusion_slot.type ==
      iflyauto::ParkingSlotType::PARKING_SLOT_TYPE_INVALID) {
    return false;
  }
  if (ego_slot_info.slot_type ==
      iflyauto::ParkingSlotType::PARKING_SLOT_TYPE_INVALID) {
    ego_slot_info.slot_type = select_fusion_slot.type;
  } else {
    if (ego_slot_info.slot_type != select_fusion_slot.type) {
      DEBUG_PRINT("selecte_fusion_slot type is changed, error");
      return false;
    }
  }

  ego_slot_info.slot_type = select_fusion_slot.type;
  ego_slot_info.select_slot_id = select_slot_id;
  ego_slot_info.select_fusion_slot = select_fusion_slot;
  ego_slot_info.select_slot = select_slot;
  ego_slot_info.select_slot_filter =
      frame_.slot_info_window_map[select_slot_id].GetFusedInfo();

  const auto &slot_points =
      ego_slot_info.select_slot_filter.corner_points().corner_point();

  std::vector<Eigen::Vector2d> pt;
  pt.resize(4);
  for (size_t i = 0; i < 4; ++i) {
    pt[i] << slot_points[i].x(), slot_points[i].y();
  }

  const auto pM01 = 0.5 * (pt[0] + pt[1]);
  const auto pM23 = 0.5 * (pt[2] + pt[3]);
  const double real_slot_length = (pM01 - pM23).norm();

  const double virtual_slot_length =
      apa_param.GetParam().car_length +
      apa_param.GetParam().slot_compare_to_car_length;

  const double use_slot_length =
      std::min(real_slot_length, virtual_slot_length);

  // const auto t = (pt[1] - pt[0]).normalized();
  // const auto n = Eigen::Vector2d(t.y(), -t.x());
  const auto n = (pM01 - pM23).normalized();
  ego_slot_info.slot_origin_pos = pM01 - use_slot_length * n;
  ego_slot_info.slot_origin_heading = std::atan2(n.y(), n.x());
  ego_slot_info.slot_origin_heading_vec = n;
  ego_slot_info.slot_length = use_slot_length;
  ego_slot_info.slot_width = (pt[0] - pt[1]).norm();

  ego_slot_info.g2l_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  ego_slot_info.l2g_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  ego_slot_info.ego_pos_slot =
      ego_slot_info.g2l_tf.GetPos(ego_pose_info.ego_pos);

  ego_slot_info.ego_heading_slot =
      ego_slot_info.g2l_tf.GetHeading(ego_pose_info.heading);

  ego_slot_info.ego_heading_slot_vec
      << std::cos(ego_slot_info.ego_heading_slot),
      std::sin(ego_slot_info.ego_heading_slot);

  // update limiter
  if (!frame_.limiter_point_window.IsEmpty()) {
    const auto limiter = frame_.limiter_point_window.GetFusedLimiterPoints();
    ego_slot_info.limiter.first << limiter.first.x(), limiter.first.y();
    ego_slot_info.limiter.second << limiter.second.x(), limiter.second.y();
  }

  // cal target pos
  ego_slot_info.target_ego_pos_slot
      << (ego_slot_info.limiter.first.x() + ego_slot_info.limiter.second.x()) *
             0.5,
      apa_param.GetParam().terminal_target_y;

  ego_slot_info.target_ego_heading_slot =
      apa_param.GetParam().terminal_target_heading;

  // cal terminal err
  ego_slot_info.terminal_err.Set(
      ego_slot_info.ego_pos_slot - ego_slot_info.target_ego_pos_slot,
      ego_slot_info.ego_heading_slot - ego_slot_info.target_ego_heading_slot);

  // cal occupied ratio
  if (std::fabs(ego_slot_info.terminal_err.pos.y()) <
          apa_param.GetParam().slot_occupied_ratio_max_lat_err &&
      std::fabs(ego_slot_info.ego_heading_slot) <
          apa_param.GetParam().slot_occupied_ratio_max_heading_err / 57.3) {
    ego_slot_info.slot_occupied_ratio = pnc::mathlib::Clamp(
        1.0 - (ego_slot_info.terminal_err.pos.x() / ego_slot_info.slot_length),
        0.0, 1.0);
  } else {
    ego_slot_info.slot_occupied_ratio = 0.0;
  }

  ego_slot_info.sin_angle = 1.0;
  ego_slot_info.origin_pt_0_heading = 0.0;
  ego_slot_info.pt_0 = ego_slot_info.g2l_tf.GetPos(pt[0]);
  ego_slot_info.pt_1 = ego_slot_info.g2l_tf.GetPos(pt[1]);
  if (select_slot.slot_type() == Common::PARKING_SLOT_TYPE_SLANTING) {
    if (frame_.slot_info_angle.count(select_slot.id()) != 0) {
      ego_slot_info.sin_angle = frame_.slot_info_angle[select_slot.id()].second;
      ego_slot_info.origin_pt_0_heading =
          90.0 - frame_.slot_info_angle[select_slot.id()].first;
      ego_slot_info.pt_0 = ego_slot_info.g2l_tf.GetPos(
          frame_.slot_info_corner_01[select_slot.id()].first);
      ego_slot_info.pt_1 = ego_slot_info.g2l_tf.GetPos(
          frame_.slot_info_corner_01[select_slot.id()].second);
    }
  }
  if (ego_slot_info.pt_0.y() > ego_slot_info.pt_1.y()) {
    std::swap(ego_slot_info.pt_0, ego_slot_info.pt_1);
  }

  ego_slot_info.fus_obj_valid_flag = frame_.fus_obj_valid_flag;
  ego_slot_info.obs_pt_vec_slot.clear();
  if (!frame_.fus_obj_valid_flag) {
    // use uss obs
    if (frame_.obs_pt_map.count(select_slot.id()) == 0) {
      return true;
    }
    const auto &obs_pt_vec = frame_.obs_pt_map[select_slot.id()];
    ego_slot_info.obs_pt_vec_slot.reserve(obs_pt_vec.size());
    for (const auto &obs_pt : obs_pt_vec) {
      const auto obs_pt_slot = ego_slot_info.g2l_tf.GetPos(obs_pt);
      ego_slot_info.obs_pt_vec_slot.emplace_back(std::move(obs_pt_slot));
    }
  }

  else {
    // use fus obj and ground line
    ego_slot_info.obs_pt_vec_slot.reserve(frame_.obs_pt_vec.size());
    // obs global coord transform to local coord
    for (const auto &obs_pt : frame_.obs_pt_vec) {
      const auto obs_pt_slot = ego_slot_info.g2l_tf.GetPos(obs_pt);
      ego_slot_info.obs_pt_vec_slot.emplace_back(std::move(obs_pt_slot));
    }
  }

  return true;
}

const bool SlotManagement::UpdateEgoParallelSlotInfo(
    const google::protobuf::uint32 &select_slot_id,
    const common::SlotInfo &select_slot,
    const iflyauto::ParkingFusionSlot &select_fusion_slot) {
  auto &ego_slot_info = frame_.ego_slot_info;
  auto &ego_pose_info = frame_.measurement;

  if (select_fusion_slot.type ==
      iflyauto::ParkingSlotType::PARKING_SLOT_TYPE_INVALID) {
    return false;
  }

  if (ego_slot_info.slot_type ==
      iflyauto::ParkingSlotType::PARKING_SLOT_TYPE_INVALID) {
    ego_slot_info.slot_type = select_fusion_slot.type;
  } else {
    if (ego_slot_info.slot_type != select_fusion_slot.type) {
      DEBUG_PRINT("selecte_fusion_slot type is changed, error");
      return false;
    }
  }

  ego_slot_info.select_slot_id = select_slot_id;
  ego_slot_info.select_fusion_slot = select_fusion_slot;
  ego_slot_info.select_slot = select_slot;
  ego_slot_info.select_slot_filter =
      frame_.slot_info_window_map[select_slot_id].GetFusedInfo();

  const auto &slot_points =
      ego_slot_info.select_slot_filter.corner_points().corner_point();

  std::vector<Eigen::Vector2d> pt;
  pt.resize(4);
  // DEBUG_PRINT("pt in select_slot_filter");
  for (size_t i = 0; i < 4; ++i) {
    pt[i] << slot_points[i].x(), slot_points[i].y();
    // DEBUG_PRINT("no. " << i << " pt : " << pt[i].transpose());
  }

  if (!frame_.is_side_calc_in_parking) {
    const Eigen::Vector2d v_10_unit = (pt[0] - pt[1]).normalized();
    // DEBUG_PRINT("v10_unit = " << v_10_unit.transpose());
    // DEBUG_PRINT(
    //     "ego heading vec = " <<
    //     ego_pose_info.ego_heading_vec.transpose());

    const double dot_ego_to_v10 = ego_pose_info.ego_heading_vec.dot(v_10_unit);
    // DEBUG_PRINT("dot ego to v10 = " << dot_ego_to_v10);

    // judge slot side via slot pt3
    if (dot_ego_to_v10 < -1e-8) {
      frame_.ego_slot_info.slot_side = pnc::geometry_lib::SLOT_SIDE_LEFT;
      DEBUG_PRINT("left!");
    } else if (dot_ego_to_v10 > 1e-8) {
      frame_.ego_slot_info.slot_side = pnc::geometry_lib::SLOT_SIDE_RIGHT;
      DEBUG_PRINT("right!");
    } else {
      frame_.ego_slot_info.slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;
      DEBUG_PRINT("calculate parallel slot side error ");
      return false;
    }
    frame_.is_side_calc_in_parking = true;
  }

  Eigen::Vector2d n = Eigen::Vector2d::Zero();
  Eigen::Vector2d t = Eigen::Vector2d::Zero();

  ego_slot_info.slot_length = (pt[0] - pt[1]).norm();
  pnc::geometry_lib::LineSegment line_01(pt[0], pt[1]);

  DEBUG_PRINT("slot side in slm = "
              << static_cast<int>(frame_.ego_slot_info.slot_side));

  // note: slot points' order is corrected in slot management
  if (frame_.ego_slot_info.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    ego_slot_info.slot_width =
        pnc::geometry_lib::CalPoint2LineDist(pt[2], line_01);

    n = (pt[0] - pt[1]).normalized();
    t << -n.y(), n.x();
    ego_slot_info.slot_origin_pos = pt[0] - ego_slot_info.slot_length * n -
                                    0.5 * ego_slot_info.slot_width * t;
  } else {
    ego_slot_info.slot_width =
        pnc::geometry_lib::CalPoint2LineDist(pt[3], line_01);

    n = -(pt[0] - pt[1]).normalized();
    t << -n.y(), n.x();
    ego_slot_info.slot_origin_pos = pt[1] - ego_slot_info.slot_length * n +
                                    0.5 * ego_slot_info.slot_width * t;
  }

  DEBUG_PRINT("slot width =" << ego_slot_info.slot_width);

  ego_slot_info.slot_origin_heading = std::atan2(n.y(), n.x());
  ego_slot_info.slot_origin_heading_vec = n;

  DEBUG_PRINT("origin heading =" << ego_slot_info.slot_origin_heading * 57.3);

  ego_slot_info.g2l_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  ego_slot_info.l2g_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  ego_slot_info.ego_pos_slot =
      ego_slot_info.g2l_tf.GetPos(ego_pose_info.ego_pos);

  ego_slot_info.ego_heading_slot =
      ego_slot_info.g2l_tf.GetHeading(ego_pose_info.heading);

  ego_slot_info.ego_heading_slot_vec
      << std::cos(ego_slot_info.ego_heading_slot),
      std::sin(ego_slot_info.ego_heading_slot);

  static const double kRearStopBuffer = 0.55;
  ego_slot_info.target_ego_pos_slot
      << apa_param.GetParam().rear_overhanging + kRearStopBuffer,
      0.0;

  ego_slot_info.target_ego_heading_slot = 0.0;

  DEBUG_PRINT("target ego pos in slot ="
              << ego_slot_info.target_ego_pos_slot.transpose()
              << " heading =" << ego_slot_info.target_ego_heading_slot * 57.3);

  // calc terminal error once
  ego_slot_info.terminal_err.Set(
      ego_slot_info.ego_pos_slot - ego_slot_info.target_ego_pos_slot,
      pnc::geometry_lib::NormalizeAngle(ego_slot_info.ego_heading_slot -
                                        ego_slot_info.target_ego_heading_slot));

  // calc slot occupied ratio

  double slot_occupied_ratio = 0.0;
  if (pnc::mathlib::IsInBound(ego_slot_info.terminal_err.pos.x(), -3.0, 4.0)) {
    const double y_err_ratio =
        ego_slot_info.terminal_err.pos.y() / (0.5 * ego_slot_info.slot_width);

    if (ego_slot_info.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
      slot_occupied_ratio = pnc::mathlib::Clamp(1 - y_err_ratio, 0.0, 1.0);
    } else if (ego_slot_info.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
      slot_occupied_ratio = pnc::mathlib::Clamp(1.0 + y_err_ratio, 0.0, 1.0);
    }
  }
  ego_slot_info.slot_occupied_ratio = slot_occupied_ratio;

  DEBUG_PRINT("ego_slot_info.slot_occupied_ratio = "
              << ego_slot_info.slot_occupied_ratio);

  // set obs
  ego_slot_info.obs_pt_vec_slot.clear();
  if (frame_.obs_pt_map.count(select_slot.id()) == 0) {
    return true;
  }
  const auto &obs_pt_vec = frame_.obs_pt_map[select_slot.id()];
  ego_slot_info.obs_pt_vec_slot.reserve(obs_pt_vec.size());
  for (const auto &obs_pt : obs_pt_vec) {
    const auto obs_pt_slot = ego_slot_info.g2l_tf.GetPos(obs_pt);
    ego_slot_info.obs_pt_vec_slot.emplace_back(std::move(obs_pt_slot));
  }

  return true;
}

void SlotManagement::UpdateSlotInfoInParking() {
  auto &ego_slot_info = frame_.ego_slot_info;

  bool reset_slot_flag = false;
  bool update_slot_flag = false;

  bool update_slot_condition_1 =
      IfUpdateSlot(ego_slot_info.select_slot,
                   ego_slot_info.select_fusion_slot.fusion_source);

  double slot_update_out_heading_max =
      apa_param.GetParam().slot_update_out_heading_max;
  double slot_update_out_heading_min =
      apa_param.GetParam().slot_update_out_heading_min;
  double slot_update_out_lat_max = apa_param.GetParam().slot_update_out_lat_max;
  double slot_update_out_lat_min = apa_param.GetParam().slot_update_out_lat_min;

  if (ego_slot_info.slot_type == Common::PARKING_SLOT_TYPE_SLANTING) {
    slot_update_out_heading_max -= ego_slot_info.origin_pt_0_heading;
    slot_update_out_heading_min -= ego_slot_info.origin_pt_0_heading;
    slot_update_out_lat_min = 0.01;
  }

  bool update_slot_condition_2 =
      (ego_slot_info.slot_occupied_ratio <
           apa_param.GetParam().slot_update_in_or_out_occupied_ratio ||
       std::fabs(ego_slot_info.ego_heading_slot * 57.3 > 22.8)) &&
      (std::fabs(ego_slot_info.ego_heading_slot) <
           slot_update_out_heading_max / 57.3 &&
       std::fabs(ego_slot_info.ego_heading_slot) >
           slot_update_out_heading_min / 57.3) &&
      (std::fabs(ego_slot_info.ego_pos_slot.y()) < slot_update_out_lat_max &&
       std::fabs(ego_slot_info.ego_pos_slot.y()) > slot_update_out_lat_min) &&
      (std::fabs(ego_slot_info.ego_pos_slot.x() < 7.86));

  bool update_slot_condition_3 =
      (ego_slot_info.slot_occupied_ratio >=
           apa_param.GetParam().slot_update_in_or_out_occupied_ratio &&
       std::fabs(ego_slot_info.ego_heading_slot) <
           apa_param.GetParam().slot_update_in_heading / 57.3 &&
       std::fabs(ego_slot_info.ego_pos_slot.y()) <
           apa_param.GetParam().slot_update_in_lat);

  // DEBUG_PRINT("update_slot_condition_1 = "
  //             << update_slot_condition_1
  //             << " update_slot_condition_2 = " << update_slot_condition_2
  //             << " update_slot_condition_3 = " <<
  //             update_slot_condition_3);

  update_slot_flag = true || update_slot_condition_1 ||
                     update_slot_condition_2 || update_slot_condition_3;

  if (!update_slot_flag) {
    frame_.no_update_slot_count++;
  }

  if (update_slot_flag) {
    if (frame_.no_update_slot_count >
        static_cast<size_t>(apa_param.GetParam().slot_reset_threshold)) {
      reset_slot_flag = true;
      frame_.no_update_slot_count = 0;
    }

    if (reset_slot_flag) {
      frame_.slot_info_window_map[ego_slot_info.select_slot_id].Reset();
    }
    frame_.slot_info_window_map[ego_slot_info.select_slot_id].Add(
        ego_slot_info.select_slot);

    // auto slot =
    // frame_.slot_management_info.mutable_slot_info_vec(slot_idx); *slot =
    // frame_.slot_info_window_vec[slot_idx].GetFusedInfo();

    ego_slot_info.select_slot_filter =
        frame_.slot_info_window_map[ego_slot_info.select_slot_id]
            .GetFusedInfo();
  }
}

void SlotManagement::UpdateParallelSlotInfoInParking() {
  // DEBUG_PRINT("occupied ratio =" <<
  // frame_.ego_slot_info.slot_occupied_ratio
  //           << ", vel mag =" << std::fabs(frame_.measurement.v_ego)
  //           << ", !parallel_slot_reseted_once ="
  //           << !frame_.parallel_slot_reseted_once);

  if ((frame_.ego_slot_info.slot_occupied_ratio > 0.55) &&
      (std::fabs(frame_.measurement.v_ego) <
       apa_param.GetParam().car_static_velocity) &&
      (!frame_.parallel_slot_reseted_once)) {
    DEBUG_PRINT("reset parallel slot once!");

    frame_.slot_info_window_map[frame_.ego_slot_info.select_slot_id].Reset();

    frame_.slot_info_window_map[frame_.ego_slot_info.select_slot_id].Add(
        frame_.ego_slot_info.select_slot);

    // auto slot =
    // frame_.slot_management_info.mutable_slot_info_vec(slot_idx); *slot =
    // frame_.slot_info_window_vec[slot_idx].GetFusedInfo();

    frame_.ego_slot_info.select_slot_filter =
        frame_.slot_info_window_map[frame_.ego_slot_info.select_slot_id]
            .GetFusedInfo();

    frame_.parallel_slot_reseted_once = true;
  }
}

void SlotManagement::UpdateLimiterInfoInParking() {
  const auto &ego_slot_info = frame_.ego_slot_info;
  if (frame_.limiter_point_window.IsEmpty()) {
    std::pair<Eigen::Vector2d, Eigen::Vector2d> limiter_slot =
        std::make_pair(Eigen::Vector2d(apa_param.GetParam().terminal_target_x,
                                       ego_slot_info.slot_width * 0.5),
                       Eigen::Vector2d(apa_param.GetParam().terminal_target_x,
                                       -ego_slot_info.slot_width * 0.5));
    frame_.limiter_point_window.Add(limiter_slot);
  }

  const bool update_limiter_flag_1 =
      (ego_slot_info.slot_occupied_ratio >=
           apa_param.GetParam().limiter_update_min_occupied_ratio &&
       ego_slot_info.slot_occupied_ratio <=
           apa_param.GetParam().limiter_update_max_occupied_ratio);

  auto current_limiter_slot =
      frame_.limiter_point_window.GetFusedLimiterPoints();

  Eigen::Vector2d p0(current_limiter_slot.first.x(),
                     current_limiter_slot.first.y());

  Eigen::Vector2d p1(current_limiter_slot.second.x(),
                     current_limiter_slot.second.y());

  const pnc::geometry_lib::LineSegment limiter_line(p0, p1);
  const auto limiter_update_distance_to_car =
      pnc::geometry_lib::CalPoint2LineDist(ego_slot_info.ego_pos_slot,
                                           limiter_line);

  const bool update_limiter_flag_2 =
      (limiter_update_distance_to_car >=
           apa_param.GetParam().limiter_update_distance_to_car &&
       ego_slot_info.slot_occupied_ratio >=
           apa_param.GetParam().limiter_update_occupied_ratio);

  const bool update_limiter_flag =
      (update_limiter_flag_1 || update_limiter_flag_2);

  if (update_limiter_flag) {
    const auto &select_fusion_slot = ego_slot_info.select_fusion_slot;
    const auto &select_slot_filter = ego_slot_info.select_slot_filter;
    std::pair<Eigen::Vector2d, Eigen::Vector2d> limiter_global;
    std::pair<Eigen::Vector2d, Eigen::Vector2d> limiter_slot;
    double move_dist;
    const double limiter_len =
        std::hypot(select_fusion_slot.limiter_position[0].x -
                       select_fusion_slot.limiter_position[1].x,
                   select_fusion_slot.limiter_position[0].y -
                       select_fusion_slot.limiter_position[1].y);
    if (limiter_len > kEps) {
      // there is limiter in slot
      limiter_global.first << select_fusion_slot.limiter_position[0].x,
          select_fusion_slot.limiter_position[0].y;
      //std::cout << "fus has limiter\n";

      limiter_global.second << select_fusion_slot.limiter_position[1].x,
          select_fusion_slot.limiter_position[1].y;

      move_dist = apa_param.GetParam().limiter_move_dist;

    } else {
      // there is no limiter in slot
      // std::cout << "fus has not limiter\n";
      limiter_global.first
          << select_slot_filter.corner_points().corner_point(2).x(),
          select_slot_filter.corner_points().corner_point(2).y();

      limiter_global.second
          << select_slot_filter.corner_points().corner_point(3).x(),
          select_slot_filter.corner_points().corner_point(3).y();

      move_dist = apa_param.GetParam().terminal_target_x;
    }

    limiter_slot.first = ego_slot_info.g2l_tf.GetPos(limiter_global.first);
    limiter_slot.second = ego_slot_info.g2l_tf.GetPos(limiter_global.second);
    limiter_slot.first.y() = ego_slot_info.slot_width * 0.5;
    limiter_slot.second.y() = -ego_slot_info.slot_width * 0.5;
    limiter_slot.first.x() += move_dist;
    limiter_slot.second.x() += move_dist;
    frame_.limiter_point_window.Add(limiter_slot);
  }

  current_limiter_slot = frame_.limiter_point_window.GetFusedLimiterPoints();
  Eigen::Vector2d current_limiter_slot_left(current_limiter_slot.first.x(),
                                            current_limiter_slot.first.y());

  Eigen::Vector2d current_limiter_slot_right(current_limiter_slot.second.x(),
                                             current_limiter_slot.second.y());

  Eigen::Vector2d current_limiter_global_left =
      ego_slot_info.l2g_tf.GetPos(current_limiter_slot_left);

  Eigen::Vector2d current_limiter_global_right =
      ego_slot_info.l2g_tf.GetPos(current_limiter_slot_right);

  common::Point2d current_limiter_global_left_p;
  current_limiter_global_left_p.set_x(current_limiter_global_left.x());
  current_limiter_global_left_p.set_y(current_limiter_global_left.y());
  common::Point2d current_limiter_global_right_p;
  current_limiter_global_right_p.set_x(current_limiter_global_right.x());
  current_limiter_global_right_p.set_y(current_limiter_global_right.y());

  if (frame_.slot_management_info.limiter_points_size() == 0) {
    auto limiter = frame_.slot_management_info.add_limiter_points();
    *limiter = current_limiter_global_left_p;
    limiter = frame_.slot_management_info.add_limiter_points();
    *limiter = current_limiter_global_right_p;
  } else {
    auto limiter = frame_.slot_management_info.mutable_limiter_points(0);
    limiter->set_x(current_limiter_global_left_p.x());
    limiter->set_y(current_limiter_global_left_p.y());
    limiter = frame_.slot_management_info.mutable_limiter_points(1);
    limiter->set_x(current_limiter_global_right_p.x());
    limiter->set_y(current_limiter_global_right_p.y());
  }
}

void SlotManagement::UpdateReleasedSlotInfo() {
  frame_.released_slot_info_vec.clear();
  iflyauto::SuccessfulSlotsInfo released_slot_info;
  for (const auto &slot_info : frame_.slot_management_info.slot_info_vec()) {
    if (slot_info.is_release()) {
      memset(&released_slot_info, 0, sizeof(released_slot_info));
      released_slot_info.id = slot_info.id();
      frame_.released_slot_info_vec.emplace_back(released_slot_info);
    } else {
      // if in parking, force set the slot release and send to hmi
      if (IsInParkingState() &&
          (static_cast<int>(slot_info.id()) ==
           static_cast<int>(frame_.ego_slot_info.select_slot_id))) {
        released_slot_info.Clear();
        released_slot_info.set_id(slot_info.id());
        frame_.released_slot_info_vec.emplace_back(released_slot_info);
      }
    }
  }
}

const bool SlotManagement::GetSelectedSlot(common::SlotInfo &slot_info,
                                           const int selected_id) {
  if (frame_.slot_info_window_map.count(selected_id) == 0) {
    return false;
  } else {
    slot_info = frame_.slot_info_window_map[selected_id].GetFusedInfo();
    return true;
  }
}

const bool SlotManagement::GetSelectedSlot(common::SlotInfo &slot_info) {
  const size_t selected_id = frame_.parking_slot_ptr->select_slot_id;
  if (frame_.slot_info_window_map.count(selected_id) == 0) {
    return false;
  } else {
    slot_info = frame_.slot_info_window_map[selected_id].GetFusedInfo();
    return true;
  }
}

const bool SlotManagement::GetSelectedLimiter(
    std::pair<Eigen::Vector2d, Eigen::Vector2d> &fused_limiter) const {
  if (frame_.slot_management_info.limiter_points_size() > 0) {
    fused_limiter.first << frame_.slot_management_info.limiter_points(0).x(),
        frame_.slot_management_info.limiter_points(0).y();

    fused_limiter.second << frame_.slot_management_info.limiter_points(1).x(),
        frame_.slot_management_info.limiter_points(1).y();
    return true;
  }
  return false;
}

const bool SlotManagement::SetRealtime() {
  DEBUG_PRINT("use real time slot");
  google::protobuf::uint32 select_slot_id = 0;
  select_slot_id = frame_.parking_slot_ptr->select_slot_id;
  common::SlotInfo select_slot;
  iflyauto::ParkingFusionSlot select_fusion_slot;
  for (const auto &fusion_slot :
       frame_.parking_slot_ptr->parking_fusion_slot_lists) {
    if (select_slot_id == fusion_slot.id) {
      select_fusion_slot = fusion_slot;
      break;
    }
  }
  const bool is_valid_slot = ProcessRawSlot(select_fusion_slot, select_slot);
  if (!is_valid_slot) {
    return false;
  }
  // update slot
  frame_.slot_info_window_map[select_slot_id].Reset();
  frame_.slot_info_window_map[select_slot_id].Add(select_slot);

  frame_.ego_slot_info.select_slot_filter =
      frame_.slot_info_window_map[select_slot_id].GetFusedInfo();

  return true;
}

const std::vector<Eigen::Vector2d> SlotManagement::GetSelectedSlotObsVec() {
  const size_t id = static_cast<size_t>(frame_.ego_slot_info.select_slot_id);
  if (id != 0) {
    return frame_.obs_pt_map[id];
  } else {
    return std::vector<Eigen::Vector2d>();
  }
}

void SlotManagement::Log() {
  const auto select_slot_id =
      static_cast<size_t>(frame_.ego_slot_info.select_slot_id);
  std::vector<double> nearby_obs_x_vec;
  std::vector<double> nearby_obs_y_vec;
  if (frame_.obs_pt_map.count(select_slot_id) != 0) {
    for (const auto &obs_pt : frame_.obs_pt_map[select_slot_id]) {
      nearby_obs_x_vec.emplace_back(obs_pt.x());
      nearby_obs_y_vec.emplace_back(obs_pt.y());
    }
  } else {
    nearby_obs_x_vec.emplace_back(0);
    nearby_obs_y_vec.emplace_back(0);
  }
  JSON_DEBUG_VECTOR("slm_selected_obs_x", nearby_obs_x_vec, 2)
  JSON_DEBUG_VECTOR("slm_selected_obs_y", nearby_obs_y_vec, 2)
}

void SlotManagement::FinishApa() {
  if (frame_.func_state_ptr->current_state ==
          iflyauto::FunctionalState_PARK_IN_COMPLETED ||
      frame_.func_state_ptr->current_state ==
          iflyauto::FunctionalState_PARK_IN_SECURE ||
      frame_.func_state_ptr->current_state ==
          iflyauto::FunctionalState_PARK_OUT_COMPLETED ||
      frame_.func_state_ptr->current_state ==
          iflyauto::FunctionalState_PARK_OUT_SECURE) {
    DebugInfoManager::GetInstance()
        .GetDebugInfoPb()
        ->clear_slot_management_info();
  }
}

}  // namespace planning