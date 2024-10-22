#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>
#include <unistd.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <iostream>
#include <memory>
#include <vector>

#include "ad_common/math/linear_interpolation.h"
#include "apa_param_setting.h"
#include "apa_plan_base.h"
#include "apa_plan_interface.h"
#include "func_state_machine_c.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_interface.h"
#include "ifly_parking_map_c.h"
#include "ifly_time.h"
#include "interface/src/c/camera_preception_groundline_c.h"
#include "interface/src/c/fusion_objects_c.h"
#include "interface/type_convert/struct_convert/camera_preception_groundline_c.h"
#include "interface/type_convert/struct_convert/fusion_objects_c.h"
#include "interface/src/c/fusion_occupancy_objects_c.h"
#include "interface/type_convert/struct_convert/fusion_occupancy_objects_c.h"

#include "perfect_control.h"
#include "planning_debug_info.pb.h"
#include "planning_plan_c.h"
#include "serialize_utils.h"
#include "slot_management.h"
#include "src/common/debug_info_log.h"
#include "log_glog.h"
#include "pose2d.h"
#include "transform2d.h"
#include "src/library/collision_detection/gjk2d_interface.h"
#include "polygon_base.h"
#include "src/library/hybrid_astar_lib/hybrid_astar_thread.h"
#include "src/library/occupancy_grid_map/virtual_wall_decider.h"
#include "src/library/occupancy_grid_map/point_cloud_obstacle.h"
#include "struct_convert/common_c.h"
#include "struct_convert/func_state_machine_c.h"
#include "struct_convert/fusion_parking_slot_c.h"
// #include "struct_convert/localization_c.h"
#include "interface/src/c/func_state_machine_c.h"
#include "struct_convert/ifly_localization_c.h"
#include "struct_convert/planning_plan_c.h"
#include "struct_convert/uss_perception_info_c.h"
#include "struct_convert/uss_wave_info_c.h"
#include "struct_convert/vehicle_service_c.h"
#include "struct_msgs/FuncStateMachine.h"
#include "struct_msgs/FusionObjectsInfo.h"
#include "struct_msgs/FusionOccupancyObjectsInfo.h"
#include "struct_msgs/GroundLinePerceptionInfo.h"
#include "struct_msgs/IFLYLocalization.h"
#include "struct_msgs/ParkingFusionInfo.h"
#include "struct_msgs/PlanningOutput.h"
#include "struct_msgs/UssPerceptInfo.h"
#include "struct_msgs/UssWaveInfo.h"
#include "struct_msgs/VehicleServiceOutputInfo.h"
#include "path_safe_checker.h"
#include "hybrid_astar_park_planner.h"

namespace py = pybind11;
using namespace planning;
using namespace planning::apa_planner;

typedef std::vector<Eigen::Vector2d> EigenPath2d;
typedef std::vector<Eigen::Vector2d> EigenPointSet2d;

static apa_planner::ApaPlanInterface *apa_interface_ptr = nullptr;
static PerfectControl *perfect_control_ptr;
static std::shared_ptr<planning::HybridAStarInterface> hybrid_astar_interface_;
std::shared_ptr<apa_planner::HybridAStarParkPlanner> hybrid_astar_park_;
HybridAStarThreadSolver *thread_solver_;
planning::apa_planner::ApaPlannerBase::EgoSlotInfo ego_slot_info_;

static planning::LocalView local_view;
std::vector<Eigen::Vector3d> global_astar_path_;
std::vector<double> global_path_s_;
// record rs path in astar total path.
std::vector<Eigen::Vector3d> static_rs_path_;
Eigen::Vector3d astar_end_pose_;
Eigen::Vector2i path_collision_info_;
ParkObstacleList hybrid_astar_obs_;
EigenPointSet2d virtual_wall_points_;
std::vector<EigenPath2d> real_time_node_list_;
// 所有启发项的rs path，record in here
std::vector<EigenPath2d> static_rs_path_list_;
Pose2D base_pose_;
EigenPath2d static_ref_line_;

// bit 4 is flag
Eigen::Vector4d car_pose_by_s_;
EigenPointSet2d search_sequence_path_;
Eigen::Vector3d coordinate_system_;

// all search node, not only include: open + close, and include deleted node.
std::vector<Eigen::Vector3d> all_searched_node_;
AstarPathGear history_gear_request_;

int Init() {
  FilePath::SetName("open_space_replay");
  InitGlog(FilePath::GetName().c_str());

  apa_interface_ptr = new apa_planner::ApaPlanInterface();

  apa_interface_ptr->Init();

  perfect_control_ptr = new PerfectControl();
  perfect_control_ptr->Init();

  std::shared_ptr<apa_planner::ApaPlannerBase> planner =
      apa_interface_ptr->GetPlannerByType(ApaPlannerType::HYBRID_ASTAR_PLANNER);
  hybrid_astar_park_ =
      std::dynamic_pointer_cast<apa_planner::HybridAStarParkPlanner>(planner);

  thread_solver_ = hybrid_astar_park_->GetThread();
  hybrid_astar_interface_ = thread_solver_->GetHybridAStarInterface();
  ILOG_INFO << "replay init success";

  return 0;
}

int StopPybind() {
  StopGlog();
  return 0;
}

int GetPathFromHybridAstar() {
  //
  global_astar_path_.clear();
  global_path_s_.clear();
  bool success = false;
  static_rs_path_.clear();

  HybridAStarResult result;

  Pose2D base_pose;

  thread_solver_->GetFullLengthPathInThread(&result, &base_pose);

  Transform2d tf;
  tf.SetBasePose(base_pose);

  size_t i;
  Pose2D local_position;
  Pose2D global_position;

  if (result.x.size() > 0) {
    for (i = 0; i < result.x.size(); i++) {
      local_position.x = result.x[i];
      local_position.y = result.y[i];
      local_position.theta = result.phi[i];

      tf.ULFLocalPoseToGlobal(&global_position, local_position);

      global_astar_path_.emplace_back(Eigen::Vector3d(
          global_position.x, global_position.y, global_position.theta));

      global_path_s_.emplace_back(result.accumulated_s[i]);

      if (result.type[i] == planning::AstarPathType::REEDS_SHEPP) {
        static_rs_path_.push_back(Eigen::Vector3d(
            global_position.x, global_position.y, global_position.theta));
      }
    }
  }

  // global_astar_path_.reserve(result.x.size());

  ILOG_INFO << "astar path size " << global_astar_path_.size();

  // ILOG_INFO << "rs path size " << rs_path_.size();

  const Pose2D target_local = thread_solver_->GetAstarTargetPose();

  tf.ULFLocalPoseToGlobal(&global_position, target_local);

  astar_end_pose_[0] = global_position.x;
  astar_end_pose_[1] = global_position.y;
  astar_end_pose_[2] = global_position.theta;

  ILOG_INFO << "target_local ";

  // node list
  real_time_node_list_.clear();
  thread_solver_->GetNodeListMessageInThread(real_time_node_list_);

  ILOG_INFO << "get node list ";

  for (int i = 0; i < real_time_node_list_.size(); i++) {
    for (int j = 0; j < real_time_node_list_[i].size(); j++) {
      tf.ULFLocalPoseToGlobal(&global_position,
                              Pose2D(real_time_node_list_[i][j].x(),
                                     real_time_node_list_[i][j].y(), 0));

      real_time_node_list_[i][j] =
          Eigen::Vector2d(global_position.x, global_position.y);
    }
  }

  ILOG_INFO << "pybind node size " << real_time_node_list_.size();

  static_rs_path_list_.clear();
  std::vector<std::vector<ad_common::math::Vec2d>> path_list;
  thread_solver_->GetRSPathHeuristicInThread(path_list);

  for (i = 0; i < path_list.size(); i++) {
    std::vector<Eigen::Vector2d> path;
    for (size_t j = 0; j < path_list[i].size(); j++) {
      local_position.x = path_list[i][j].x();
      local_position.y = path_list[i][j].y();

      tf.ULFLocalPoseToGlobal(&global_position, local_position);

      path.emplace_back(Eigen::Vector2d(global_position.x, global_position.y));
    }

    static_rs_path_list_.emplace_back(path);
  }

  std::vector<ad_common::math::Vec2d> rs_path;
  thread_solver_->GetRSPathLinkInThread(rs_path);
  std::vector<Eigen::Vector2d> tmp_path;
  for (size_t j = 0; j < rs_path.size(); j++) {
    local_position.x = rs_path[j].x();
    local_position.y = rs_path[j].y();

    tf.ULFLocalPoseToGlobal(&global_position, local_position);

    tmp_path.emplace_back(
        Eigen::Vector2d(global_position.x, global_position.y));
  }

  static_rs_path_list_.emplace_back(tmp_path);

  ILOG_INFO << "rs path size = " << static_rs_path_list_.size();

  // update ref line
  ParkReferenceLine ref_line;
  thread_solver_->GetRefLine(&ref_line);
  static_ref_line_.clear();

  // start
  ad_common::math::Vec2d point;
  ref_line.GetPointByDist(&point, -5.0);
  local_position.x = point.x();
  local_position.y = point.y();
  tf.ULFLocalPoseToGlobal(&global_position, local_position);

  static_ref_line_.emplace_back(
      Eigen::Vector2d(global_position.x, global_position.y));

  // end
  ref_line.GetPointByDist(&point, 15.0);
  local_position.x = point.x();
  local_position.y = point.y();
  tf.ULFLocalPoseToGlobal(&global_position, local_position);
  static_ref_line_.emplace_back(
      Eigen::Vector2d(global_position.x, global_position.y));

  // get collision pose in path
  const size_t collision_id = hybrid_astar_park_->GetPathCollisionID();
  path_collision_info_[0] = static_cast<int>(collision_id);
  if (hybrid_astar_park_->IsPathCollision()) {
    path_collision_info_[1] = 1;
  } else {
    path_collision_info_[1] = 0;
  }

  // 为了调试搜索过程，plot it
  search_sequence_path_.clear();
  const std::vector<ad_common::math::Vec2d> &search_path =
      hybrid_astar_interface_->GetPriorQueueNode();

  for (i = 0; i < search_path.size(); i++) {
    local_position.x = search_path[i].x();
    local_position.y = search_path[i].y();
    tf.ULFLocalPoseToGlobal(&global_position, local_position);

    search_sequence_path_.emplace_back(
        Eigen::Vector2d(global_position.x, global_position.y));
  }

  // 基坐标位置
  coordinate_system_[0] = ego_slot_info_.slot_origin_pos[0];
  coordinate_system_[1] = ego_slot_info_.slot_origin_pos[1];

  // plot all searched node
  const std::vector<DebugAstarSearchPoint> &all_search_node =
      hybrid_astar_interface_->GetChildNodeForDebug();

  all_searched_node_.clear();
  double is_safe = 0;
  for (i = 0; i < all_search_node.size(); i++) {
    local_position.x = all_search_node[i].pos.x;
    local_position.y = all_search_node[i].pos.y;
    tf.ULFLocalPoseToGlobal(&global_position, local_position);

    is_safe = all_search_node[i].safe ? 1.0 : 0.0;

    all_searched_node_.emplace_back(
        Eigen::Vector3d(global_position.x, global_position.y, is_safe));
  }

  AstarRequest request = thread_solver_->GetAstarRequest();
  history_gear_request_ = request.first_action_request.gear_request;

  return 0;
}

const void UpdateLocalView(
    py::bytes &func_statemachine_bytes, py::bytes &parking_slot_info_bytes,
    py::bytes &localization_info_bytes,
    py::bytes &vehicle_service_output_info_bytes,
    py::bytes &uss_wave_info_bytes, py::bytes &uss_perception_info_bytes,
    py::bytes &fus_objs, py::bytes &fus_occ_obj_msg_bytes, bool force_plan,
    std::vector<double> target_managed_slot_x_vec,
    std::vector<double> target_managed_slot_y_vec,
    std::vector<double> target_managed_limiter_x_vec,
    std::vector<double> target_managed_limiter_y_vec, int current_state) {
  iflyauto::FuncStateMachine func_statemachine;
  func_statemachine.current_state = static_cast<FunctionalState>(current_state);

  iflyauto::ParkingFusionInfo parking_slot_info =
      BytesToStruct<iflyauto::ParkingFusionInfo,
                    struct_msgs::ParkingFusionInfo>(parking_slot_info_bytes);

  iflyauto::IFLYLocalization localization_info =
      BytesToStruct<iflyauto::IFLYLocalization, struct_msgs::IFLYLocalization>(
          localization_info_bytes);

  iflyauto::VehicleServiceOutputInfo vehicle_service_output_info =
      BytesToStruct<iflyauto::VehicleServiceOutputInfo,
                    struct_msgs::VehicleServiceOutputInfo>(
          vehicle_service_output_info_bytes);

  iflyauto::UssWaveInfo uss_wave_info =
      BytesToStruct<iflyauto::UssWaveInfo, struct_msgs::UssWaveInfo>(
          uss_wave_info_bytes);

  iflyauto::FusionObjectsInfo fusion_objs =
      BytesToStruct<iflyauto::FusionObjectsInfo,
                    struct_msgs::FusionObjectsInfo>(fus_objs);

  // auto ground_line_ =
  //     BytesToStruct<GroundLinePerception::GroundLinePerceptionInfo>(
  //         ground_line);

  iflyauto::FusionOccupancyObjectsInfo fus_occ_obj_info =
      BytesToStruct<iflyauto::FusionOccupancyObjectsInfo,
                    struct_msgs::FusionOccupancyObjectsInfo>(
          fus_occ_obj_msg_bytes);

  iflyauto::UssPerceptInfo uss_perception_info =
      BytesToStruct<iflyauto::UssPerceptInfo, struct_msgs::UssPerceptInfo>(
          uss_perception_info_bytes);

  local_view.localization = localization_info;
  local_view.vehicle_service_output_info = vehicle_service_output_info;
  local_view.parking_fusion_info = parking_slot_info;
  local_view.uss_wave_info = uss_wave_info;
  local_view.function_state_machine_info = func_statemachine;
  local_view.uss_percept_info = uss_perception_info;
  // local_view.ground_line_perception = ground_line_;
  local_view.fusion_objects_info = fusion_objs;
  local_view.fusion_occupancy_objects_info = fus_occ_obj_info;

  return;
}

static const int CopyVirtualWallForPlot(
    const ParkObstacleList &obs_list,
    const planning::apa_planner::ApaPlannerBase::EgoSlotInfo &slot) {
  const ApaParameters &park_param = apa_param.GetParam();

  // publish to python
  virtual_wall_points_.clear();

  double global_start_theta;
  Eigen::Vector2d global;
  double global_theta;

  if (obs_list.virtual_obs.size() < 1) {
    return 0;
  }

  for (const auto &obs : obs_list.virtual_obs) {
    Eigen::Vector2d local = {obs.x, obs.y};

    global = slot.l2g_tf.GetPos(local);

    virtual_wall_points_.emplace_back(global);
  }

  return 0;
}

void GetTrajPoseBySDist(const double s) {
  size_t left_idx = 0;
  size_t right_idx = 0;

  car_pose_by_s_[3] = -1.0;

  if (global_astar_path_.size() < 1) {
    return;
  }

  // ILOG_INFO << "s " << s << " path size "
  //           << global_astar_path_.size();

  // for (size_t i = 0; i < global_path_s_.size(); i++) {
  //   ILOG_INFO << "i " << i << "s " << global_path_s_[i] << " "
  //             << global_astar_path_[i].x() << " " <<
  //             global_astar_path_[i].y();
  // }

  if (s <= global_path_s_[0]) {
    left_idx = 0;
    right_idx = 0;
  } else if (global_path_s_.size() > 0 &&
             s >= global_path_s_[global_path_s_.size() - 1]) {
    left_idx = global_path_s_.size() - 1;
    right_idx = left_idx;
  } else {
    for (size_t i = 0; i < global_path_s_.size(); i++) {
      if (i == 0) {
        if (s <= global_path_s_[1]) {
          left_idx = 0;
          right_idx = 1;
          break;
        }
      }

      if (s <= global_path_s_[i] && s >= global_path_s_[i - 1]) {
        left_idx = i - 1;
        right_idx = i;
        break;
      }
    }
  }

  double left_s = global_path_s_[left_idx];
  double right_s = global_path_s_[right_idx];

  if (left_idx == right_idx) {
    car_pose_by_s_[0] = global_astar_path_[left_idx][0];
    car_pose_by_s_[1] = global_astar_path_[left_idx][1];
    car_pose_by_s_[2] = global_astar_path_[left_idx][2];
  } else {
    car_pose_by_s_[0] =
        ad_common::math::lerp(global_astar_path_[left_idx][0], left_s,
                              global_astar_path_[right_idx][0], right_s, s);

    car_pose_by_s_[1] =
        ad_common::math::lerp(global_astar_path_[left_idx][1], left_s,
                              global_astar_path_[right_idx][1], right_s, s);

    car_pose_by_s_[2] =
        ad_common::math::slerp(global_astar_path_[left_idx][2], left_s,
                               global_astar_path_[right_idx][2], right_s, s);
  }

  car_pose_by_s_[3] = 1.0;

  ILOG_INFO << "left s " << left_s << "right s " << right_s << " left phi "
            << global_astar_path_[left_idx][2] << " right phi "
            << global_astar_path_[right_idx][2] << " left_idx " << left_idx
            << " right_idx" << right_idx;

  return;
}

const bool PlanOnce(
    py::bytes &func_statemachine_bytes, py::bytes &parking_slot_info_bytes,
    py::bytes &localization_info_bytes,
    py::bytes &vehicle_service_output_info_bytes,
    py::bytes &uss_wave_info_bytes, py::bytes &uss_perception_info_bytes,
    py::bytes &fus_objs, py::bytes &fus_occ_obj_msg_bytes, int select_id,
    bool force_plan, bool is_path_optimization, bool is_cilqr_optimization,
    bool is_reset, bool is_complete_path, double sample_ds,
    std::vector<double> target_managed_slot_x_vec,
    std::vector<double> target_managed_slot_y_vec,
    std::vector<double> target_managed_limiter_x_vec,
    std::vector<double> target_managed_limiter_y_vec, int current_state) {
  double start_time = IflyTime::Now_us();

  SimulationParam sim_param;
  sim_param.is_complete_path = is_complete_path;
  sim_param.force_plan = force_plan;
  sim_param.is_path_optimization = is_path_optimization;
  sim_param.is_cilqr_optimization = is_cilqr_optimization;
  sim_param.sample_ds = sample_ds;
  sim_param.is_reset = is_reset;
  sim_param.target_managed_slot_x_vec = target_managed_slot_x_vec;
  sim_param.target_managed_slot_y_vec = target_managed_slot_y_vec;
  sim_param.target_managed_limiter_x_vec = target_managed_limiter_x_vec;
  sim_param.target_managed_limiter_y_vec = target_managed_limiter_y_vec;
  sim_param.use_slot_in_bag = false;

  apa_interface_ptr->SetSimuParam(sim_param);

  // iflyauto::FuncStateMachine func_statemachine =
  //     BytesToStruct<iflyauto::FuncStateMachine, struct_msgs::FuncStateMachine>(
  //         func_statemachine_bytes);

  iflyauto::FuncStateMachine func_statemachine;
  func_statemachine.current_state = static_cast<FunctionalState>(current_state);

  iflyauto::ParkingFusionInfo parking_slot_info =
      BytesToStruct<iflyauto::ParkingFusionInfo,
                    struct_msgs::ParkingFusionInfo>(parking_slot_info_bytes);

  iflyauto::IFLYLocalization localization_info =
      BytesToStruct<iflyauto::IFLYLocalization, struct_msgs::IFLYLocalization>(
          localization_info_bytes);

  iflyauto::VehicleServiceOutputInfo vehicle_service_output_info =
      BytesToStruct<iflyauto::VehicleServiceOutputInfo,
                    struct_msgs::VehicleServiceOutputInfo>(
          vehicle_service_output_info_bytes);

  iflyauto::UssWaveInfo uss_wave_info =
      BytesToStruct<iflyauto::UssWaveInfo, struct_msgs::UssWaveInfo>(
          uss_wave_info_bytes);

  iflyauto::FusionObjectsInfo fusion_objs =
      BytesToStruct<iflyauto::FusionObjectsInfo,
                    struct_msgs::FusionObjectsInfo>(fus_objs);

  // auto ground_line_ =
  //     BytesToStruct<GroundLinePerception::GroundLinePerceptionInfo>(
  //         ground_line);

  iflyauto::FusionOccupancyObjectsInfo fus_occ_obj_info =
      BytesToStruct<iflyauto::FusionOccupancyObjectsInfo,
                    struct_msgs::FusionOccupancyObjectsInfo>(
          fus_occ_obj_msg_bytes);

  iflyauto::UssPerceptInfo uss_perception_info =
      BytesToStruct<iflyauto::UssPerceptInfo, struct_msgs::UssPerceptInfo>(
          uss_perception_info_bytes);

  local_view.localization = localization_info;
  local_view.vehicle_service_output_info = vehicle_service_output_info;
  local_view.parking_fusion_info = parking_slot_info;
  local_view.uss_wave_info = uss_wave_info;
  local_view.function_state_machine_info = func_statemachine;
  local_view.uss_percept_info = uss_perception_info;
  // local_view.ground_line_perception = ground_line_;
  local_view.fusion_objects_info = fusion_objs;
  local_view.fusion_occupancy_objects_info = fus_occ_obj_info;

  double copy_data_time = IflyTime::Now_us();
  ILOG_INFO << " copy data time ms " << (copy_data_time - start_time) / 1000.0;

  if (force_plan) {
    local_view.function_state_machine_info.current_state =
        iflyauto::FunctionalState_PARK_IN_SEARCHING;
  }
  if (select_id > 0) {
    local_view.parking_fusion_info.select_slot_id = select_id;
    ILOG_INFO << "pybind select slot id "
              << local_view.parking_fusion_info.select_slot_id;
  }

  ILOG_INFO << "pybind select slot id " << select_id;

  const bool result = apa_interface_ptr->Update(&local_view);
  apa_interface_ptr->UpdateDebugInfo();

  double plan_time = IflyTime::Now_us();
  ILOG_INFO << "plan time ms " << (plan_time - copy_data_time) / 1000.0;

  const std::shared_ptr<apa_planner::ApaPlannerBase> vertical_space_decider_ =
      apa_interface_ptr->GetPlannerByType(ApaPlannerType::HYBRID_ASTAR_PLANNER);

  if (vertical_space_decider_ != nullptr) {
    const apa_planner::ApaPlannerBase::Frame &frame =
        vertical_space_decider_->GetFrame();

    const apa_planner::ApaPlannerBase::EgoSlotInfo &ego_slot_info =
        frame.ego_slot_info;

    ego_slot_info_ = ego_slot_info;

    GetPathFromHybridAstar();

    ParkObstacleList virtual_wall_obs;
    VirtualWallDecider wall_decider;

    wall_decider.Process(
        virtual_wall_obs.virtual_obs, 40.0, 15.0, ego_slot_info.slot_width,
        ego_slot_info.slot_length,
        Pose2D(ego_slot_info.ego_pos_slot[0], ego_slot_info.ego_pos_slot[1],
               ego_slot_info.ego_heading_slot),
        Pose2D(ego_slot_info.target_ego_pos_slot[0],
               ego_slot_info.target_ego_pos_slot[1],
               ego_slot_info.target_ego_heading_slot));

    CopyVirtualWallForPlot(virtual_wall_obs, ego_slot_info);

  } else {
    ILOG_INFO << "hybrid_astar_interface_ is null";
  }

  double data_process_time = IflyTime::Now_us();
  ILOG_INFO << "plan process time ms "
            << (data_process_time - plan_time) / 1000.0;

  return true;
}

const bool RefreshThreadResult() {
  GetPathFromHybridAstar();

  ILOG_INFO << " RefreshThreadResult ";

  if (global_astar_path_.size() > 0) {
    return true;
  }

  return false;
}

const bool TriggerPlan(bool force_plan, bool is_path_optimization,
                       bool is_cilqr_optimization, bool is_reset,
                       std::vector<double> target_managed_slot_x_vec,
                       std::vector<double> target_managed_slot_y_vec,
                       std::vector<double> target_managed_limiter_x_vec,
                       std::vector<double> target_managed_limiter_y_vec,
                       std::vector<double> &end_pose, const double time) {
  SimulationParam sim_param;
  sim_param.force_plan = force_plan;
  sim_param.is_path_optimization = is_path_optimization;
  sim_param.is_cilqr_optimization = is_cilqr_optimization;
  sim_param.is_reset = is_reset;
  sim_param.target_managed_slot_x_vec = target_managed_slot_x_vec;
  sim_param.target_managed_slot_y_vec = target_managed_slot_y_vec;
  sim_param.target_managed_limiter_x_vec = target_managed_limiter_x_vec;
  sim_param.target_managed_limiter_y_vec = target_managed_limiter_y_vec;

  apa_interface_ptr->SetSimuParam(sim_param);

  if (force_plan) {
    local_view.function_state_machine_info.current_state =
        iflyauto::FunctionalState_PARK_IN_SEARCHING;
  }

  bool update_path = false;

  if (force_plan) {
    planning::apa_planner::ApaPlannerBase::Frame frame;
    planning::apa_planner::ApaPlannerBase::EgoSlotInfo ego_slot_info =
        frame.ego_slot_info;

    if (1) {
      ego_slot_info = ego_slot_info_;
    } else {
      ego_slot_info.slot_origin_pos[0] = end_pose[0];
      ego_slot_info.slot_origin_pos[1] = end_pose[1];
      ego_slot_info.slot_origin_heading = end_pose[2];

      ego_slot_info.slot_origin_heading_vec =
          Eigen::Vector2d(std::cos(end_pose[2]), std::sin(end_pose[2]));

      ego_slot_info.slot_length = 6;

      ego_slot_info.slot_width = 2.3;

      // base coordinate
      ego_slot_info.g2l_tf.Init(ego_slot_info.slot_origin_pos,
                                ego_slot_info.slot_origin_heading);

      ego_slot_info.l2g_tf.Init(ego_slot_info.slot_origin_pos,
                                ego_slot_info.slot_origin_heading);

      // update ego pose
      Eigen::Vector2d ego_global_position(
          local_view.localization.position.position_boot.x,
          local_view.localization.position.position_boot.y);
      double heading_ego = local_view.localization.orientation.euler_boot.yaw;

      Pose2D ego_global_pose = {ego_global_position.x(),
                                ego_global_position.y(), heading_ego};

      ego_slot_info.ego_pos_slot =
          ego_slot_info.g2l_tf.GetPos(ego_global_position);
      ego_slot_info.ego_heading_slot =
          ego_slot_info.g2l_tf.GetHeading(heading_ego);

      // ILOG_INFO << "  ego_pos_slot = " << ego_slot_info.ego_pos_slot.x()
      //           << ego_slot_info.ego_pos_slot.y() << "  ego_heading_slot = "
      //           << ego_slot_info.ego_heading_slot * 57.3;

      ego_slot_info.ego_heading_slot_vec =
          Eigen::Vector2d(std::cos(ego_slot_info.ego_heading_slot),
                          std::sin(ego_slot_info.ego_heading_slot));

      // cal target pos
      const planning::apa_planner::ApaParameters &parking_param =
          apa_param.GetParam();

      ego_slot_info.target_ego_pos_slot = Eigen::Vector2d(
          parking_param.terminal_target_x, parking_param.terminal_target_y);

      ego_slot_info.target_ego_heading_slot =
          parking_param.terminal_target_heading;

      // get global
      const auto &target_ego_pos_global =
          ego_slot_info.l2g_tf.GetPos(ego_slot_info.target_ego_pos_slot);
      const auto &target_ego_heading_global = ego_slot_info.l2g_tf.GetHeading(
          ego_slot_info.target_ego_heading_slot);

      // ILOG_INFO << "target_ego_pos_slot = " <<
      // ego_slot_info.target_ego_pos_slot[0]
      //           << ", " << ego_slot_info.target_ego_pos_slot[1]
      //           << "  target_ego_heading_slot = "
      //           << ego_slot_info.target_ego_heading_slot * 57.3;

      // cal terminal error
      ego_slot_info.terminal_err.Set(
          ego_slot_info.ego_pos_slot - ego_slot_info.target_ego_pos_slot,
          ego_slot_info.ego_heading_slot -
              ego_slot_info.target_ego_heading_slot);
    }

    hybrid_astar_obs_.Clear();

    // obs
    const ApaParameters &park_param = apa_param.GetParam();
    // translate perception
    Pose2D slot_base_pose = Pose2D(ego_slot_info.slot_origin_pos.x(),
                                   ego_slot_info.slot_origin_pos.y(),
                                   ego_slot_info.slot_origin_heading);

    // start
    Pose2D start =
        Pose2D(ego_slot_info.ego_pos_slot[0], ego_slot_info.ego_pos_slot[1],
               ego_slot_info.ego_heading_slot);

    Pose2D real_end = Pose2D(ego_slot_info.target_ego_pos_slot[0],
                             ego_slot_info.target_ego_pos_slot[1],
                             ego_slot_info.target_ego_heading_slot);
    PointCloudObstacleTransform obstacle_generator;
    obstacle_generator.GenerateLocalObstacle(
        hybrid_astar_obs_, &local_view, true, ego_slot_info.slot_length,
        ego_slot_info.slot_width, slot_base_pose, start, real_end);

    CopyVirtualWallForPlot(hybrid_astar_obs_, ego_slot_info);

    int virtual_wall_size = hybrid_astar_obs_.virtual_obs.size();

    ILOG_INFO << "virtual_wall_size " << virtual_wall_size;
    ILOG_INFO << "fusion obs size "
              << hybrid_astar_obs_.point_cloud_list.size();

    // end
    Eigen::Vector3d end;
    end[0] = ego_slot_info.target_ego_pos_slot[0] +
             park_param.vertical_slot_target_adjust_dist;
    end[1] = ego_slot_info.target_ego_pos_slot[1];
    end[2] = ego_slot_info.target_ego_heading_slot;

    AstarRequest request;
    request.first_action_request.has_request = true;
    if (history_gear_request_ == AstarPathGear::drive) {
      request.first_action_request.gear_request = AstarPathGear::reverse;
    } else {
      request.first_action_request.gear_request = AstarPathGear::drive;
    }
    request.path_generate_method =
        planning::AstarPathGenerateType::ASTAR_SEARCHING;

    base_pose_ = slot_base_pose;
    request.start_ = start;
    request.start_.DebugString();

    request.goal_ = Pose2D(end[0], end[1], end[2]);

    request.real_goal = real_end;
    request.vertical_slot_target_adjust_dist_ =
        apa_param.GetParam().vertical_slot_target_adjust_dist;

    request.base_pose_ = base_pose_;

    request.space_type = ParkSpaceType::vertical;
    request.parking_task = ParkingTask::parking_in;
    request.head_request = ParkingVehDirectionRequest::tail_in_first;
    request.rs_request = RSPathRequestType::none;
    request.slot_width = ego_slot_info.slot_width;
    request.slot_length = ego_slot_info.slot_length;
    request.history_gear = history_gear_request_;

    thread_solver_->SetRequest(hybrid_astar_obs_, request);

    ego_slot_info_ = ego_slot_info;

    GetPathFromHybridAstar();

    update_path = true;

  } else {
    ILOG_INFO << "hybrid_astar_interface_ is null";
  }

  double s = time * 0.4;
  GetTrajPoseBySDist(s);

  return update_path;
}

py::bytes GetPlanningOutput() {
  iflyauto::PlanningOutput planning_output =
      apa_interface_ptr->GetPlaningOutput();

  return StructToBytes<iflyauto::PlanningOutput, struct_msgs::PlanningOutput>(
      planning_output);
}

py::bytes GetPlanningDebugInfo() {
  return ProtoToBytes(apa_interface_ptr->GetPlanningDebugInfo());
}

void DynamicsUpdate(py::bytes &planning_output_bytes, double dt) {
  iflyauto::PlanningOutput planning_output =
      BytesToStruct<iflyauto::PlanningOutput, struct_msgs::PlanningOutput>(
          planning_output_bytes);

  perfect_control_ptr->Update(planning_output, dt);
}

std::vector<double> GetDynamicState() {
  const auto &state = perfect_control_ptr->GetState();

  std::vector<double> res = {state.pos.x(), state.pos.y(), state.heading,
                             state.vel};

  return res;
}

Eigen::Vector4d GetTrajPoseByDist() { return car_pose_by_s_; }

void DynamicsSwitchBuf(double x, double y, double heading) {
  perfect_control_ptr->SetState(
      PerfectControl::DynamicState(Eigen::Vector2d(x, y), heading));
}

const std::vector<Eigen::Vector3d> &GetReedsShapePath() {
  return static_rs_path_;
}

const Eigen::Vector3d GetAstarEndPose() { return astar_end_pose_; }

const Eigen::Vector2i GetAstarPathCollisionID() { return path_collision_info_; }

const std::vector<Eigen::Vector3d> &GetAstarPath() {
  // ILOG_INFO << " check  astar path ";

  // for (size_t i = 0; i < global_path_s_.size(); i++) {
  //   ILOG_INFO << "i " << i << ", s " << global_path_s_[i] << " "
  //             << global_astar_path_[i].x() << " " <<
  //             global_astar_path_[i].y();
  // }

  return global_astar_path_;
}

const bool SetFsm(py::bytes &func_statemachine_bytes) {
  iflyauto::FuncStateMachine func_statemachine;
  // auto func_statemachine =
  //     BytesToStruct<iflyauto::FuncStateMachine, struct_msgs::FuncStateMachine>(
  //         func_statemachine_bytes);

  local_view.function_state_machine_info = func_statemachine;

  return true;
}

const bool SetLocalization(py::bytes &localization_info_bytes) {
  iflyauto::IFLYLocalization localization_info =
      BytesToStruct<iflyauto::IFLYLocalization, struct_msgs::IFLYLocalization>(
          localization_info_bytes);

  local_view.localization = localization_info;

  return true;
}

const bool SetGroundLine(py::bytes &line) {
  auto ground_line = BytesToStruct<iflyauto::GroundLinePerceptionInfo,
                                   struct_msgs::GroundLinePerceptionInfo>(line);

  local_view.ground_line_perception = ground_line;

  return true;
}

const bool SetFusionObject(py::bytes &info) {
  auto obs = BytesToStruct<iflyauto::FusionObjectsInfo,
                           struct_msgs::FusionObjectsInfo>(info);

  local_view.fusion_objects_info = obs;

  return true;
}

const bool SetSlotInfo() {
  local_view.parking_fusion_info.select_slot_id = 0;

  return true;
}

const std::vector<std::vector<Eigen::Vector2d>> &GetAstarAllNodes() {
  return real_time_node_list_;
}

const std::vector<Eigen::Vector2d> &GetVirtualWall() {
  for (size_t i = 0; i < virtual_wall_points_.size(); i++) {
    // ILOG_INFO << "line " << virtual_wall_list_[i];
  }

  return virtual_wall_points_;
}

const std::vector<Eigen::Vector2d> &GetPlotRefLine() {

  return static_ref_line_;
}

const std::vector<std::vector<Eigen::Vector2d>> &GetRSHeuristicPath() {
  return static_rs_path_list_;
}

const std::vector<Eigen::Vector2d> &GetSearchSequencePath() {
  return search_sequence_path_;
}

const Eigen::Vector3d GetCoordinateSystem() {
  return coordinate_system_;
}

const std::vector<Eigen::Vector3d> &GetAllSearchNode() {
  return all_searched_node_;
}

PYBIND11_MODULE(replay_simulation_hybrid_astar, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("UpdateLocalView", &UpdateLocalView)
      .def("PlanOnce", &PlanOnce)
      .def("GetPlanningOutput", &GetPlanningOutput)
      .def("DynamicsUpdate", &DynamicsUpdate)
      .def("DynamicsSwitchBuf", &DynamicsSwitchBuf)
      .def("GetReedsShapePath", &GetReedsShapePath)
      .def("GetAstarPath", &GetAstarPath)
      .def("SetLocalization", &SetLocalization)
      .def("TriggerPlan", &TriggerPlan)
      .def("SetSlotInfo", &SetSlotInfo)
      .def("GetVirtualWall", &GetVirtualWall)
      .def("SetGroundLine", &SetGroundLine)
      .def("SetFusionObject", &SetFusionObject)
      .def("GetTrajPoseByDist", &GetTrajPoseByDist)
      .def("GetAstarEndPose", &GetAstarEndPose)
      .def("GetAstarPathCollisionID", &GetAstarPathCollisionID)
      .def("GetAstarAllNodes", &GetAstarAllNodes)
      .def("GetRSHeuristicPath", &GetRSHeuristicPath)
      .def("RefreshThreadResult", &RefreshThreadResult)
      .def("GetPlotRefLine", &GetPlotRefLine)
      .def("GetSearchSequencePath", &GetSearchSequencePath)
      .def("GetCoordinateSystem", &GetCoordinateSystem)
      .def("GetAllSearchNode", &GetAllSearchNode)
      .def("GetDynamicState", &GetDynamicState);
}
