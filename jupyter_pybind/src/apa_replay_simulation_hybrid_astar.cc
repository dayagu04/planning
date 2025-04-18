#include <bits/stdint-intn.h>
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
#include "apa_param_config.h"
#include "apa_plan_interface.h"
#include "apa_world.h"
#include "camera_perception_groundline_c.h"
#include "collision_detection/path_safe_checker.h"
#include "config_context.h"
#include "control_command_c.h"
#include "debug_info_log.h"
#include "func_state_machine_c.h"
#include "fusion_groundline_c.h"
#include "fusion_objects_c.h"
#include "fusion_occupancy_objects_c.h"
#include "fusion_parking_slot_c.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_interface.h"
#include "ifly_localization_c.h"
#include "ifly_parking_map_c.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "perfect_control.h"
#include "planning_debug_info.pb.h"
#include "planning_plan_c.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "serialize_utils.h"
#include "slot_manager.h"
#include "src/common/debug_info_log.h"
#include "src/library/convex_collision_detection/gjk2d_interface.h"
#include "src/library/hybrid_astar_lib/hybrid_astar_thread.h"
#include "src/library/occupancy_grid_map/point_cloud_obstacle.h"
#include "src/modules/apa_function/parking_scenario/narrow_space/narrow_space_scenario.h"
#include "struct_convert/camera_perception_groundline_c.h"
#include "struct_convert/common_c.h"
#include "struct_convert/control_command_c.h"
#include "struct_convert/func_state_machine_c.h"
#include "struct_convert/fusion_groundline_c.h"
#include "struct_convert/fusion_objects_c.h"
#include "struct_convert/fusion_occupancy_objects_c.h"
#include "struct_convert/fusion_parking_slot_c.h"
#include "struct_convert/hmi_inner_c.h"
#include "struct_convert/ifly_localization_c.h"
#include "struct_convert/planning_plan_c.h"
#include "struct_convert/uss_perception_info_c.h"
#include "struct_convert/uss_wave_info_c.h"
#include "struct_convert/vehicle_service_c.h"
#include "struct_msgs/ControlOutput.h"
#include "struct_msgs/FuncStateMachine.h"
#include "struct_msgs/FusionGroundLineInfo.h"
#include "struct_msgs/FusionObjectsInfo.h"
#include "struct_msgs/FusionOccupancyObjectsInfo.h"
#include "struct_msgs/IFLYLocalization.h"
#include "struct_msgs/ParkingFusionInfo.h"
#include "struct_msgs/PlanningOutput.h"
#include "struct_msgs/UssPerceptInfo.h"
#include "struct_msgs/UssWaveInfo.h"
#include "struct_msgs/VehicleServiceOutputInfo.h"
#include "transform2d.h"

namespace py = pybind11;
using namespace planning;
using namespace planning::apa_planner;

typedef std::vector<Eigen::Vector2d> EigenPath2d;
typedef std::vector<Eigen::Vector2d> EigenPointSet2d;

static apa_planner::ApaPlanInterface *apa_interface_ptr = nullptr;
static PerfectControl *perfect_control_ptr;
static std::shared_ptr<planning::HybridAStarInterface> hybrid_astar_interface_;
std::shared_ptr<apa_planner::NarrowSpaceScenario> hybrid_astar_park_;
HybridAStarThreadSolver *thread_solver_;
planning::apa_planner::EgoInfoUnderSlot ego_slot_info_;

static planning::LocalView local_view;
std::vector<Eigen::Vector3d> global_astar_path_;
std::vector<double> global_path_s_;
// record rs path in astar total path.
std::vector<Eigen::Vector3d> static_rs_path_;
Eigen::Vector3d astar_end_pose_;
Eigen::Vector2i path_collision_info_;
ParkObstacleList hybrid_astar_obs_;
EigenPointSet2d virtual_wall_points_;
std::vector<std::vector<Eigen::Vector2f>> real_time_node_list_;
// 所有启发项的rs path，record in here
std::vector<EigenPath2d> static_rs_path_list_;
Pose2D base_pose_;
EigenPath2d static_ref_line_;

// bit 4 is flag
EigenPointSet2d search_sequence_path_;
EigenPointSet2d deletenode_sequence_path_;
Eigen::Vector3d coordinate_system_;

// all search node, not only include: open + close, and include deleted node.
std::vector<Eigen::Vector4d> all_searched_node_;
AstarPathGear history_gear_request_;
// local coordinate system
std::vector<Eigen::Vector3d> footprint_circle_model_normal_gear_;
std::vector<Eigen::Vector3d> footprint_circle_model_drive_gear_;
std::vector<Eigen::Vector3d> footprint_circle_model_reverse_gear_;

int Init() {
  FilePath::SetName("open_space_replay");
  InitGlog(FilePath::GetName().c_str());

  apa_interface_ptr = new apa_planner::ApaPlanInterface();

  apa_interface_ptr->Init();

  perfect_control_ptr = new PerfectControl();
  perfect_control_ptr->Init();

  std::shared_ptr<apa_planner::ParkingScenario> planner =
      apa_interface_ptr->GetPlannerByType(
          ParkingScenarioType::SCENARIO_NARROW_SPACE);
  hybrid_astar_park_ =
      std::dynamic_pointer_cast<apa_planner::NarrowSpaceScenario>(planner);

  thread_solver_ = hybrid_astar_park_->GetThread();
  hybrid_astar_interface_ = thread_solver_->GetHybridAStarInterface();
  ILOG_INFO << "replay init success";

  return 0;
}

int StopPybind() {
  StopGlog();
  return 0;
}

void UpdateFootprintCircle(const AstarPathGear gear,
                           std::vector<Eigen::Vector3d> &footprint_circle) {
  footprint_circle.clear();

  FootPrintCircleModel *model =
      hybrid_astar_interface_->GetSlotOutsideCircleFootPrint();
  if (model == nullptr) {
    return;
  }

  const FootPrintCircleList circle_footprint =
      model->GetLocalFootPrintCircleByGear(gear);

  const FootPrintCircle *circle = &circle_footprint.max_circle;
  footprint_circle.push_back(
      Eigen::Vector3d(circle->pos.x, circle->pos.y, circle->radius));
  for (int i = 0; i < circle_footprint.size; i++) {
    circle = &circle_footprint.circles[i];

    footprint_circle.push_back(
        Eigen::Vector3d(circle->pos.x, circle->pos.y, circle->radius));
  }

  return;
}

void UpdateFootprintCircleList() {
  UpdateFootprintCircle(AstarPathGear::NORMAL,
                        footprint_circle_model_normal_gear_);

  UpdateFootprintCircle(AstarPathGear::DRIVE,
                        footprint_circle_model_drive_gear_);

  UpdateFootprintCircle(AstarPathGear::REVERSE,
                        footprint_circle_model_reverse_gear_);

  return;
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
          Eigen::Vector2f(global_position.x, global_position.y);
    }
  }

  ILOG_INFO << "pybind node size " << real_time_node_list_.size();

  static_rs_path_list_.clear();
  std::vector<std::vector<Vec2df32>> path_list;
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

  std::vector<Vec2df32> rs_path;
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
  Vec2df32 point;
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

  // todo: rm it
  // get collision pose in path
  path_collision_info_[0] = 0;
  path_collision_info_[1] = 0;

  // 为了调试搜索过程，plot it
  search_sequence_path_.clear();
  const std::vector<Vec2df32> &search_path =
      hybrid_astar_interface_->GetPriorQueueNode();

  for (i = 0; i < search_path.size(); i++) {
    local_position.x = search_path[i].x();
    local_position.y = search_path[i].y();
    tf.ULFLocalPoseToGlobal(&global_position, local_position);

    search_sequence_path_.emplace_back(
        Eigen::Vector2d(global_position.x, global_position.y));
  }

  deletenode_sequence_path_.clear();
  const std::vector<Vec2df32> &delnode_path =
      hybrid_astar_interface_->GetDelNodeQueueNode();

  for (i = 0; i < delnode_path.size(); i++) {
    local_position.x = delnode_path[i].x();
    local_position.y = delnode_path[i].y();
    tf.ULFLocalPoseToGlobal(&global_position, local_position);

    deletenode_sequence_path_.emplace_back(
        Eigen::Vector2d(global_position.x, global_position.y));
  }

  // 基坐标位置
  coordinate_system_[0] = ego_slot_info_.origin_pose_global.pos[0];
  coordinate_system_[1] = ego_slot_info_.origin_pose_global.pos[1];

  // plot all searched node
  const std::vector<DebugAstarSearchPoint> &all_search_node =
      hybrid_astar_interface_->GetChildNodeForDebug();

  all_searched_node_.clear();
  double is_safe = 0;
  double is_gear_switch_node = 0;
  for (i = 0; i < all_search_node.size(); i++) {
    local_position.x = all_search_node[i].pos.x;
    local_position.y = all_search_node[i].pos.y;
    tf.ULFLocalPoseToGlobal(&global_position, local_position);

    is_safe = all_search_node[i].safe ? 1.0 : 0.0;
    is_gear_switch_node = all_search_node[i].gear_switch_point ? 1.0 : 0.0;

    all_searched_node_.emplace_back(Eigen::Vector4d(
        global_position.x, global_position.y, is_safe, is_gear_switch_node));
  }

  AstarRequest request = thread_solver_->GetAstarRequest();
  history_gear_request_ = request.first_action_request.gear_request;

  UpdateFootprintCircleList();

  ILOG_INFO << "receive finish";

  return 0;
}

const void UpdateLocalView(
    py::bytes &func_statemachine_bytes, py::bytes &parking_slot_info_bytes,
    py::bytes &localization_info_bytes,
    py::bytes &vehicle_service_output_info_bytes,
    py::bytes &uss_wave_info_bytes, py::bytes &uss_perception_info_bytes,
    py::bytes &ground_line_info_bytes, py::bytes &fus_objs,
    py::bytes &fus_occ_obj_msg_bytes, bool force_plan,
    std::vector<double> target_managed_slot_x_vec,
    std::vector<double> target_managed_slot_y_vec,
    std::vector<double> target_managed_limiter_x_vec,
    std::vector<double> target_managed_limiter_y_vec, int current_state) {
  iflyauto::FuncStateMachine func_statemachine =
      BytesToStruct<iflyauto::FuncStateMachine, struct_msgs::FuncStateMachine>(
          func_statemachine_bytes);

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

  iflyauto::FusionGroundLineInfo ground_line_ =
      BytesToStruct<iflyauto::FusionGroundLineInfo,
                    struct_msgs::FusionGroundLineInfo>(ground_line_info_bytes);

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
  local_view.ground_line_perception = ground_line_;
  local_view.fusion_objects_info = fusion_objs;
  local_view.fusion_occupancy_objects_info = fus_occ_obj_info;

  return;
}

static const int CopyVirtualWallForPlot(
    const ParkObstacleList &obs_list,
    const planning::apa_planner::EgoInfoUnderSlot &slot) {
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

const bool PlanOnce(py::bytes &func_statemachine_bytes,
                    py::bytes &parking_slot_info_bytes,
                    py::bytes &localization_info_bytes,
                    py::bytes &vehicle_service_output_info_bytes,
                    py::bytes &uss_wave_info_bytes,
                    py::bytes &uss_perception_info_bytes,
                    py::bytes &ground_line_bytes, py::bytes &fus_objs,
                    py::bytes &fus_occ_obj_msg_bytes, int select_id,
                    bool force_plan, bool is_path_optimization,
                    bool is_cilqr_optimization, bool is_reset,
                    bool is_complete_path, double sample_ds,
                    std::vector<double> target_managed_slot_x_vec,
                    std::vector<double> target_managed_slot_y_vec,
                    std::vector<double> target_managed_limiter_x_vec,
                    std::vector<double> target_managed_limiter_y_vec,
                    const int path_plan_method) {
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

  switch (path_plan_method) {
    case 0:
      apa_param.SetPram().path_generator_type =
          ParkPathGenerationType::GEOMETRY_BASED;
      break;
    case 1:
      apa_param.SetPram().path_generator_type =
          ParkPathGenerationType::SEARCH_BASED;
      break;
    default:
      break;
  }

  iflyauto::FuncStateMachine func_statemachine =
      BytesToStruct<iflyauto::FuncStateMachine, struct_msgs::FuncStateMachine>(
          func_statemachine_bytes);

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

  iflyauto::FusionGroundLineInfo ground_line_info =
      BytesToStruct<iflyauto::FusionGroundLineInfo,
                    struct_msgs::FusionGroundLineInfo>(ground_line_bytes);

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
  local_view.ground_line_perception = ground_line_info;
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

  PlanningResult navigation_traj;
  const bool result = apa_interface_ptr->Update(&local_view, &navigation_traj);
  apa_interface_ptr->UpdateDebugInfo();

  double plan_time = IflyTime::Now_us();
  ILOG_INFO << "plan time ms " << (plan_time - copy_data_time) / 1000.0;

  const std::shared_ptr<apa_planner::ParkingScenario> scenario =
      apa_interface_ptr->GetPlannerByType(
          ParkingScenarioType::SCENARIO_NARROW_SPACE);

  if (scenario != nullptr) {
    const apa_planner::EgoInfoUnderSlot &ego_info =
        scenario->GetApaWorldPtr()->GetSlotManagerPtr()->ego_info_under_slot_;
    ego_slot_info_ = ego_info;

    GetPathFromHybridAstar();

    ParkObstacleList virtual_wall_obs;
    thread_solver_->GetVirtualWallPoints(&virtual_wall_obs.virtual_obs);
    CopyVirtualWallForPlot(virtual_wall_obs, ego_info);

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

  bool update_path = false;

  if (force_plan) {
    planning::apa_planner::EgoInfoUnderSlot ego_info;
    ego_info = ego_slot_info_;

    hybrid_astar_obs_.Clear();

    // obs
    const ApaParameters &park_param = apa_param.GetParam();
    // translate perception
    Pose2D slot_base_pose = Pose2D(ego_info.origin_pose_global.pos.x(),
                                   ego_info.origin_pose_global.pos.y(),
                                   ego_info.origin_pose_global.heading);

    // start
    Pose2D start = Pose2D(ego_info.cur_pose.pos[0], ego_info.cur_pose.pos[1],
                          ego_info.cur_pose.heading);

    Pose2D real_end =
        Pose2D(ego_info.target_pose.pos[0], ego_info.target_pose.pos[1],
               ego_info.target_pose.heading);
    PointCloudObstacleTransform obstacle_generator;

    ParkSpaceType slot_type;
    if (ego_info.slot_type == SlotType::PARALLEL) {
      slot_type = ParkSpaceType::PARALLEL;
    } else if (ego_info.slot_type == SlotType::SLANT) {
      slot_type = ParkSpaceType::SLANTING;
    } else {
      slot_type = ParkSpaceType::VERTICAL;
    }

    ParkingVehDirection parking_in_type;
    const std::shared_ptr<apa_planner::ApaWorld> world =
        hybrid_astar_park_->GetApaWorldPtr();

    if (world->GetStateMachineManagerPtr()->GetStateMachine() ==
            ApaStateMachine::ACTIVE_IN_CAR_REAR ||
        world->GetStateMachineManagerPtr()->GetStateMachine() ==
            ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR) {
      parking_in_type = ParkingVehDirection::TAIL_IN;
    } else {
      parking_in_type = ParkingVehDirection::HEAD_IN;
    }

    VirtualWallDecider *wall_decider =
        hybrid_astar_park_->MutableVirtualWallDecider();
    wall_decider->Process(
        hybrid_astar_obs_.virtual_obs, ego_info.slot.GetWidth(),
        ego_info.slot.GetLength(), start, real_end, slot_type,
        pnc::geometry_lib::SlotSide::SLOT_SIDE_INVALID, parking_in_type);

    obstacle_generator.GenerateLocalObstacleByLocalView(
        hybrid_astar_obs_, &local_view, ego_info.slot.GetLength(),
        ego_info.slot.GetWidth(), slot_base_pose, start, false);

    CopyVirtualWallForPlot(hybrid_astar_obs_, ego_info);

    // end
    Eigen::Vector3d end;
    double end_straight_dist = 0.0;
    if (world->GetStateMachineManagerPtr()->GetStateMachine() ==
            ApaStateMachine::ACTIVE_IN_CAR_REAR ||
        world->GetStateMachineManagerPtr()->GetStateMachine() ==
            ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR) {
      end_straight_dist =
          apa_param.GetParam().astar_config.vertical_tail_in_end_straight_dist;
    } else {
      end_straight_dist = 0.5;
    }

    end[0] = ego_info.target_pose.pos.x() + end_straight_dist;
    end[1] = ego_info.target_pose.pos.y();
    end[2] = ego_info.target_pose.heading;

    AstarRequest request;
    request.first_action_request.has_request = true;
    if (history_gear_request_ == AstarPathGear::DRIVE) {
      request.first_action_request.gear_request = AstarPathGear::REVERSE;
    } else {
      request.first_action_request.gear_request = AstarPathGear::DRIVE;
    }
    request.path_generate_method =
        planning::AstarPathGenerateType::ASTAR_SEARCHING;

    base_pose_ = slot_base_pose;
    request.start_ = start;
    ILOG_INFO << "start pose";
    request.start_.DebugString();
    request.goal_ = Pose2D(end[0], end[1], end[2]);
    request.real_goal = real_end;
    request.base_pose_ = base_pose_;
    request.space_type = ParkSpaceType::VERTICAL;
    request.swap_start_goal = false;

    if (world->GetStateMachineManagerPtr()->GetStateMachine() ==
            ApaStateMachine::ACTIVE_IN_CAR_FRONT ||
        world->GetStateMachineManagerPtr()->GetStateMachine() ==
            ApaStateMachine::SEARCH_IN_SELECTED_CAR_FRONT) {
      request.direction_request = ParkingVehDirection::HEAD_IN;
    } else {
      request.direction_request = ParkingVehDirection::TAIL_IN;
    }

    request.rs_request = RSPathRequestType::NONE;
    request.slot_width = ego_info.slot.GetWidth();
    request.slot_length = ego_info.slot.GetLength();
    request.history_gear = history_gear_request_;

    thread_solver_->SetRequest(hybrid_astar_obs_, request);

    ego_slot_info_ = ego_info;

    GetPathFromHybridAstar();

    update_path = true;

  } else {
    ILOG_INFO << "hybrid_astar_interface_ is null";
  }

  ILOG_INFO << "trigger plan ";

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
  auto func_statemachine =
      BytesToStruct<iflyauto::FuncStateMachine, struct_msgs::FuncStateMachine>(
          func_statemachine_bytes);

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
  auto ground_line = BytesToStruct<iflyauto::FusionGroundLineInfo,
                                   struct_msgs::FusionGroundLineInfo>(line);

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

const std::vector<std::vector<Eigen::Vector2f>> &GetAstarAllNodes() {
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

const std::vector<Eigen::Vector2d> &GetDelNodeSequencePath() {
  return deletenode_sequence_path_;
}

const Eigen::Vector3d GetCoordinateSystem() { return coordinate_system_; }

const std::vector<Eigen::Vector4d> &GetAllSearchNode() {
  return all_searched_node_;
}

std::vector<Eigen::VectorXd> GetDpSpeedConstraints() {
  std::vector<Eigen::VectorXd> speed_debug_data;
  Eigen::VectorXd v(7);
  v.setZero();

  auto &debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();
  planning::common::ApaSpeedDebug *speed_debug = nullptr;
  if (debug_->has_apa_speed_debug()) {
    speed_debug = debug_->mutable_apa_speed_debug();
  }

  if (speed_debug == nullptr) {
    speed_debug_data.emplace_back(v);
    return speed_debug_data;
  }

  int size = 0;
  if (speed_debug->has_dp_speed_constraint()) {
    size = speed_debug->dp_speed_constraint().s_size();
  }

  for (int i = 0; i < size; i++) {
    v[0] = speed_debug->dp_speed_constraint().s(i);

    if (i < speed_debug->dp_speed_constraint().obs_dist_size()) {
      v[1] = speed_debug->dp_speed_constraint().obs_dist(i);
    }

    if (i < speed_debug->dp_speed_constraint().v_upper_bound_size()) {
      v[2] = speed_debug->dp_speed_constraint().v_upper_bound(i);
    }

    if (i < speed_debug->dp_speed_constraint().a_upper_bound_size()) {
      v[3] = speed_debug->dp_speed_constraint().a_upper_bound(i);
    }

    if (i < speed_debug->dp_speed_constraint().a_lower_bound_size()) {
      v[4] = speed_debug->dp_speed_constraint().a_lower_bound(i);
    }

    if (i < speed_debug->dp_speed_constraint().jerk_upper_bound_size()) {
      v[5] = speed_debug->dp_speed_constraint().jerk_upper_bound(i);
    }

    if (i < speed_debug->dp_speed_constraint().jerk_lower_bound_size()) {
      v[6] = speed_debug->dp_speed_constraint().jerk_lower_bound(i);
    }

    speed_debug_data.emplace_back(v);
  }

  if (speed_debug_data.size() == 0) {
    speed_debug_data.emplace_back(v);
  }

  return speed_debug_data;
}

std::vector<Eigen::Vector2d> GetQPSpeedConstraints() {
  std::vector<Eigen::Vector2d> speed_debug_data;
  Eigen::Vector2d v;
  v.setZero();

  auto &debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();
  planning::common::ApaSpeedDebug *speed_debug = nullptr;
  if (debug_->has_apa_speed_debug()) {
    speed_debug = debug_->mutable_apa_speed_debug();
  }

  if (speed_debug == nullptr) {
    speed_debug_data.emplace_back(v);
    return speed_debug_data;
  }

  int size = 0;
  if (speed_debug->has_qp_speed_constraint()) {
    size = speed_debug->qp_speed_constraint().s_size();
  }

  for (int i = 0; i < size; i++) {
    v[0] = speed_debug->qp_speed_constraint().s(i);
    if (i < speed_debug->qp_speed_constraint().v_upper_bound_size()) {
      v[1] = speed_debug->qp_speed_constraint().v_upper_bound(i);
    }

    speed_debug_data.emplace_back(v);
  }

  if (speed_debug_data.size() == 0) {
    speed_debug_data.emplace_back(v);
  }

  return speed_debug_data;
}

const std::vector<Eigen::Vector3d> &GetFootPrintModel(const int32_t gear) {
  switch (gear) {
    case 0:
      return footprint_circle_model_normal_gear_;
    case 4:
      return footprint_circle_model_drive_gear_;
    case 2:
      return footprint_circle_model_reverse_gear_;
    default:
      break;
  }

  return footprint_circle_model_normal_gear_;
}

const double GetRefCruiseSpeed() {
  auto &debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();
  planning::common::ApaSpeedDebug *speed_debug = nullptr;
  if (debug_->has_apa_speed_debug()) {
    speed_debug = debug_->mutable_apa_speed_debug();
  }
  if (speed_debug == nullptr) {
    return 0.0;
  }

  double speed = 0.0;
  if (speed_debug->has_ref_cruise_speed()) {
    speed = speed_debug->ref_cruise_speed();
  }

  return speed;
}

std::vector<Eigen::VectorXd> GetDPSpeedOptimizationData() {
  std::vector<Eigen::VectorXd> speed_profile;
  Eigen::VectorXd v(5);

  auto &debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();
  planning::common::ApaSpeedDebug *speed_debug = nullptr;
  if (debug_->has_apa_speed_debug()) {
    speed_debug = debug_->mutable_apa_speed_debug();
  }

  if (speed_debug == nullptr) {
    speed_profile.emplace_back(v);
    return speed_profile;
  }

  int size = speed_debug->dp_profile_size();
  for (int i = 0; i < size; i++) {
    v[0] = speed_debug->dp_profile(i).s();
    v[1] = speed_debug->dp_profile(i).t();
    v[2] = speed_debug->dp_profile(i).vel();
    v[3] = speed_debug->dp_profile(i).acc();
    v[4] = speed_debug->dp_profile(i).jerk();

    speed_profile.push_back(v);
  }

  return speed_profile;
}

std::vector<Eigen::VectorXd> GetQPSpeedOptimizationData() {
  std::vector<Eigen::VectorXd> speed_profile;
  Eigen::VectorXd v(5);

  auto &debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();
  planning::common::ApaSpeedDebug *speed_debug = nullptr;
  if (debug_->has_apa_speed_debug()) {
    speed_debug = debug_->mutable_apa_speed_debug();
  }

  if (speed_debug == nullptr) {
    speed_profile.emplace_back(v);
    return speed_profile;
  }

  int size = speed_debug->qp_profile_size();
  for (int i = 0; i < size; i++) {
    v[0] = speed_debug->qp_profile(i).s();
    v[1] = speed_debug->qp_profile(i).t();
    v[2] = speed_debug->qp_profile(i).vel();
    v[3] = speed_debug->qp_profile(i).acc();
    v[4] = speed_debug->qp_profile(i).jerk();

    speed_profile.push_back(v);
  }

  return speed_profile;
}

std::vector<Eigen::VectorXd> GetJLTSpeedData() {
  std::vector<Eigen::VectorXd> speed_profile;
  Eigen::VectorXd point(5);

  auto &debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();
  planning::common::ApaSpeedDebug *speed_debug = nullptr;
  if (debug_->has_apa_speed_debug()) {
    speed_debug = debug_->mutable_apa_speed_debug();
  }

  if (speed_debug == nullptr) {
    speed_profile.emplace_back(point);
    return speed_profile;
  }

  int size = speed_debug->jlt_profile_size();
  for (int i = 0; i < size; i++) {
    point[0] = speed_debug->jlt_profile(i).s();
    point[1] = speed_debug->jlt_profile(i).t();
    point[2] = speed_debug->jlt_profile(i).vel();
    point[3] = speed_debug->jlt_profile(i).acc();
    point[4] = speed_debug->jlt_profile(i).jerk();

    speed_profile.push_back(point);
  }

  return speed_profile;
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
      .def("GetAstarEndPose", &GetAstarEndPose)
      .def("GetAstarPathCollisionID", &GetAstarPathCollisionID)
      .def("GetAstarAllNodes", &GetAstarAllNodes)
      .def("GetRSHeuristicPath", &GetRSHeuristicPath)
      .def("RefreshThreadResult", &RefreshThreadResult)
      .def("GetPlotRefLine", &GetPlotRefLine)
      .def("GetSearchSequencePath", &GetSearchSequencePath)
      .def("GetDelNodeSequencePath", GetDelNodeSequencePath)
      .def("GetCoordinateSystem", &GetCoordinateSystem)
      .def("GetAllSearchNode", &GetAllSearchNode)
      .def("GetDpSpeedConstraints", &GetDpSpeedConstraints)
      .def("GetQPSpeedConstraints", &GetQPSpeedConstraints)
      .def("GetRefCruiseSpeed", &GetRefCruiseSpeed)
      .def("GetQPSpeedOptimizationData", &GetQPSpeedOptimizationData)
      .def("GetDPSpeedOptimizationData", &GetDPSpeedOptimizationData)
      .def("GetJLTSpeedData", &GetJLTSpeedData)
      .def("GetFootPrintModel", &GetFootPrintModel)
      .def("GetDynamicState", &GetDynamicState);
}
