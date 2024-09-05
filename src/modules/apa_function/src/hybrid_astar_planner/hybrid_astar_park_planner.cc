#include "hybrid_astar_park_planner.h"

#include "hybrid_astar_request.h"
#include "hybrid_astar_response.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "path_safe_checker.h"
#include "point_cloud_obstacle.h"
#include "polygon_base.h"
#include "transform2d.h"
#include "utils_math.h"

namespace planning {
namespace apa_planner {

HybridAStarParkPlanner::HybridAStarParkPlanner(
    const std::shared_ptr<ApaWorld>& apa_world_ptr) {
  SetApaWorldPtr(apa_world_ptr);

  Init();
}

void HybridAStarParkPlanner::Reset() {
  frame_.Reset();
  current_path_point_global_vec_.clear();

  HybridAStarThreadSolver* thread_solver =
      HybridAStarThreadSolver::GetInstance();

  thread_solver->Clear();

  return;
}

void HybridAStarParkPlanner::Init() {
  const ApaParameters& params = apa_param.GetParam();

  // todo, system should use same vehicle parameter configuration file and
  // data structure.
  ILOG_INFO << "init astar thread";

  HybridAStarThreadSolver* solver = HybridAStarThreadSolver::GetInstance();

  solver->Init(params.rear_overhanging, params.car_length, params.car_width,
               params.steer_ratio, params.wheel_base,
               params.min_turn_radius + 0.1,
               (params.max_car_width - params.car_width) * 0.5);

  solver->Start();

  return;
}

const bool HybridAStarParkPlanner::CheckSegCompleted() {
  bool is_seg_complete = false;
  if (frame_.spline_success) {
    if (frame_.remain_dist < apa_param.GetParam().max_replan_remain_dist &&
        apa_world_ptr_->GetApaDataPtr()->measurement_data.static_flag) {
      if (frame_.stuck_uss_time > 0.068) {
        is_seg_complete = true;
      }
    }
  }

  return is_seg_complete;
}

const bool HybridAStarParkPlanner::CheckUssStucked() {
  if (frame_.remain_dist_uss < apa_param.GetParam().max_replan_remain_dist &&
      apa_world_ptr_->GetApaDataPtr()->measurement_data.static_flag) {
    if (frame_.stuck_uss_time >
        apa_param.GetParam().uss_stuck_replan_wait_time) {
      frame_.is_replan_by_uss = true;
      return true;
    }
  }

  return false;
}

const bool HybridAStarParkPlanner::CheckReplan() {
  if (frame_.is_replan_first == true) {
    frame_.replan_reason = FIRST_PLAN;
    return true;
  }

  frame_.is_replan_by_uss = false;
  frame_.is_replan_dynamic = false;

  if (CheckSegCompleted()) {
    frame_.replan_reason = SEG_COMPLETED_PATH;
    return true;
  }

  if (CheckUssStucked()) {
    frame_.replan_reason = SEG_COMPLETED_USS;
    return true;
  }

  if (frame_.stuck_uss_time > apa_param.GetParam().stuck_replan_time) {
    // if plan once, the stuck_uss_time is clear and accumlate again
    frame_.replan_reason = STUCKED;
    return true;
  }

  frame_.replan_reason = NOT_REPLAN;

  return false;
}

const bool HybridAStarParkPlanner::CheckFinished() {
  const auto& ego_slot_info = frame_.ego_slot_info;

  const bool lon_condition =
      ego_slot_info.terminal_err.pos.x() < apa_param.GetParam().finish_lon_err;

  const double y1 = ego_slot_info.ego_pos_slot.y();
  const double y2 =
      (ego_slot_info.ego_pos_slot + (apa_param.GetParam().wheel_base +
                                     apa_param.GetParam().front_overhanging) *
                                        ego_slot_info.ego_heading_slot_vec)
          .y();

  const bool lat_condition_1 =
      std::fabs(y1) <= apa_param.GetParam().finish_lat_err;

  const bool lat_condition_2 =
      std::fabs(y1) <= apa_param.GetParam().finish_lat_err_strict &&
      std::fabs(y2) <= apa_param.GetParam().finish_lat_err_strict;

  const bool heading_condition_1 =
      std::fabs(ego_slot_info.terminal_err.heading) <=
      apa_param.GetParam().finish_heading_err * kDeg2Rad;

  const bool heading_condition_2 =
      std::fabs(ego_slot_info.terminal_err.heading) <=
      (apa_param.GetParam().finish_heading_err + 1.988) * kDeg2Rad;

  const bool lat_condition = (lat_condition_1 && heading_condition_1) &&
                             (lat_condition_2 && heading_condition_2);

  const bool static_condition =
      apa_world_ptr_->GetApaDataPtr()->measurement_data.static_flag;

  const bool remain_s_condition =
      frame_.remain_dist < apa_param.GetParam().max_replan_remain_dist;

  bool parking_finish =
      lon_condition && lat_condition && static_condition && remain_s_condition;

  if (parking_finish) {
    return true;
  }

  // stucked by directly behind uss
  const auto& uss_obstacle_avoider_ptr =
      apa_world_ptr_->GetUssObstacleAvoidancePtr();
  const bool enter_slot_condition =
      frame_.ego_slot_info.slot_occupied_ratio >
      apa_param.GetParam().finish_uss_slot_occupied_ratio;
  const bool remain_uss_condition =
      frame_.remain_dist_uss < apa_param.GetParam().max_replan_remain_dist;
  if (uss_obstacle_avoider_ptr->CheckIsDirectlyBehindUss()) {
    parking_finish = lat_condition && static_condition &&
                     enter_slot_condition && remain_uss_condition;
  }

  if (parking_finish) {
    return true;
  }

  parking_finish = lat_condition && static_condition && enter_slot_condition &&
                   (ego_slot_info.terminal_err.pos.x() < 0.568);

  return parking_finish;
}

void HybridAStarParkPlanner::PlanCore() {
  // prepare simulation
  InitSimulation();

  // check planning status
  if (!apa_world_ptr_->GetApaDataPtr()->simu_param.force_plan &&
      CheckPlanSkip()) {
    return;
  }

  if (CheckPaused()) {
    SetParkingStatus(PARKING_PAUSED);
    if (frame_.pause_time > apa_param.GetParam().pause_failed_time) {
      SetParkingStatus(PARKING_FAILED);
      frame_.plan_fail_reason = PAUSE_FAILED_TIME;
    }
    return;
  }

  // update remain dist
  UpdateRemainDist();

  // update ego slot info
  if (!UpdateEgoSlotInfo()) {
    SetParkingStatus(PARKING_FAILED);
    frame_.plan_fail_reason = UPDATE_EGO_SLOT_INFO;
    return;
  }

  // check finish
  if (CheckFinished()) {
    SetParkingStatus(PARKING_FINISHED);
    ILOG_INFO << "park finish";
    return;
  }

  // check failed
  if (CheckStuckFailed()) {
    SetParkingStatus(PARKING_FAILED);
    frame_.plan_fail_reason = STUCK_FAILED_TIME;
    return;
  }

  bool is_replan = CheckReplan();
  bool update_thread_path = UpdateThreadPath();
  uint8_t path_plan_result = PathPlannerResult::PLAN_FAILED;

  // check replan
  if (apa_world_ptr_->GetApaDataPtr()->simu_param.force_plan || is_replan ||
      update_thread_path) {
    ILOG_INFO << "plan reason = " << GetPlanReason(frame_.replan_reason)
              << ",force replan = "
              << apa_world_ptr_->GetApaDataPtr()->simu_param.force_plan
              << ",thread update = " << update_thread_path
              << ",is_replan = " << is_replan;

    frame_.replan_flag = true;
    path_plan_result = PlanBySearchBasedMethod();
    frame_.pathplan_result = path_plan_result;

    switch (path_plan_result) {
      case PathPlannerResult::PLAN_UPDATE:
        if (PostProcessPath()) {
          SetParkingStatus(PARKING_PLANNING);
        } else {
          SetParkingStatus(PARKING_FAILED);
        }
        break;
      case PathPlannerResult::PLAN_FAILED:
        SetParkingStatus(PARKING_FAILED);
        break;
      default:
        SetParkingStatus(PARKING_PLANNING);
        break;
    }

  } else {
    SetParkingStatus(PARKING_RUNNING);
    HybridAstarDebugInfoClear();
    ILOG_INFO << "use history path";
  }

  return;
}

void HybridAStarParkPlanner::Log() const { return; }

void HybridAStarParkPlanner::GenTlane() { return; }

void HybridAStarParkPlanner::GenObstacles() { return; }

const uint8_t HybridAStarParkPlanner::PathPlanOnce() { return false; }

const std::string HybridAStarParkPlanner::GetPlanReason(const uint8_t type) {
  switch (type) {
    case 1:
      return "first_replan";
    case 2:
      return "SEG_COMPLETED_PATH";
    case 3:
      return "SEG_COMPLETED_USS";
    case 4:
      return "STUCKED";
    case 5:
      return "DYNAMIC";
    case 6:
      return "SEG_COMPLETED_COL_DET";
    default:
      return "NOT_REPLAN";
  }

  return "none";
}

void HybridAStarParkPlanner::ShrinkPathByFusionObj() {
  double path_checker_start_time = IflyTime::Now_ms();

  is_ego_collision_ = false;
  // obs generate
  ParkObstacleList obs;
  PointCloudObstacleTransform obstacle_generator;
  const LocalView* local_view = apa_world_ptr_->GetLocalViewPtr();
  const MeasurementData* measures_ptr =
      &(apa_world_ptr_->GetApaDataPtr()->measurement_data);
  Pose2D ego_pose(measures_ptr->pos[0], measures_ptr->pos[1],
                  measures_ptr->heading);

  obstacle_generator.GenerateGlobalObstacle(obs, local_view);

  PathSafeChecker path_safe_checker;
  path_safe_checker.Excute(
      current_path_point_global_vec_,
      static_cast<pnc::geometry_lib::PathSegGear>(frame_.gear_command), &obs,
      ego_pose);

  Pose2D* collision_pose = path_safe_checker.GetCollisionPose();
  *collision_pose = Pose2D(0, 0, 0);

  if (path_safe_checker.IsPathCollision()) {
    path_safe_checker.UpdatePathValidDist(current_path_point_global_vec_,
                                          ego_pose);

    double valid_dist = path_safe_checker.GetPathValidDist();
    // for stop safely, add a lon buffer
    valid_dist -= 0.1;
    valid_dist = std::max(0.0, valid_dist);

    if (frame_.remain_dist > valid_dist) {
      frame_.remain_dist = valid_dist;

      ILOG_INFO << "valid dist=" << valid_dist;
    }

    // for debug
    const size_t collision_id = path_safe_checker.GetPathCollisionID();
    const pnc::geometry_lib::PathPoint& point =
        current_path_point_global_vec_[collision_id];
    *collision_pose = Pose2D(point.pos[0], point.pos[1], point.heading);
  }

  double path_checker_end_time = IflyTime::Now_ms();
  ILOG_INFO << "path checker time ms "
            << path_checker_end_time - path_checker_start_time;

  return;
}

void HybridAStarParkPlanner::UpdateRemainDist() {
  ApaPlannerBase::UpdateRemainDist();

  ShrinkPathByFusionObj();

  return;
}

ApaPlannerBase::PathPlannerResult
HybridAStarParkPlanner::PlanBySearchBasedMethod() {
  const auto& ego_slot_info = frame_.ego_slot_info;

  PathPlannerResult res = ApaPlannerBase::PathPlannerResult::WAIT_PATH;

  // start
  Pose2D start;
  start.x = ego_slot_info.ego_pos_slot[0];
  start.y = ego_slot_info.ego_pos_slot[1];
  start.theta = ego_slot_info.ego_heading_slot;

  // real target pose in slot
  Pose2D real_end;
  real_end.x = ego_slot_info.target_ego_pos_slot[0];
  real_end.y = ego_slot_info.target_ego_pos_slot[1];
  real_end.theta = ego_slot_info.target_ego_heading_slot;

  // astar end, maybe different with real end.
  Pose2D end = real_end;
  end.x = real_end.x + apa_param.GetParam().vertical_slot_target_adjust_dist;

  HybridAStarThreadSolver* thread_solver =
      HybridAStarThreadSolver::GetInstance();

  if (thread_solver == nullptr) {
    ILOG_ERROR << "plan fail";
    return ApaPlannerBase::PathPlannerResult::PLAN_FAILED;
  }

  double astar_start_time = IflyTime::Now_ms();
  // obs generate
  ParkObstacleList obs;
  PointCloudObstacleTransform obstacle_generator;

  Pose2D slot_base_pose = Pose2D(ego_slot_info.slot_origin_pos.x(),
                                 ego_slot_info.slot_origin_pos.y(),
                                 ego_slot_info.slot_origin_heading);
  const LocalView* local_view = apa_world_ptr_->GetLocalViewPtr();

  // hack: delete obstacle around ego and slot. In the future, it will be
  // retired.
  obstacle_generator.GenerateLocalObstacle(
      obs, local_view, true, ego_slot_info.slot_length,
      ego_slot_info.slot_width, slot_base_pose, start, real_end);

  double search_start_time = IflyTime::Now_ms();
  ILOG_INFO << "fusion obj time ms " << search_start_time - astar_start_time;

  // set input
  AstarRequest history_request = thread_solver->GetAstarRequest();
  AstarRequest cur_request;
  cur_request.path_generate_method =
      planning::AstarPathGenerateType::astar_searching;
  cur_request.first_action_request.has_request = false;
  cur_request.space_type = ParkSpaceType::vertical;
  cur_request.parking_task = ParkingTask::parking_in;
  cur_request.head_request = ParkingVehDirectionRequest::tail_in_first;
  cur_request.rs_request = RSPathRequestType::none;
  cur_request.timestamp_ms = astar_start_time;

  cur_request.start_ = start;
  cur_request.base_pose_ = slot_base_pose;
  cur_request.real_goal = real_end;

  cur_request.vertical_slot_target_adjust_dist_ =
      apa_param.GetParam().vertical_slot_target_adjust_dist;
  cur_request.slot_width = ego_slot_info.slot_width;
  cur_request.slot_length = ego_slot_info.slot_length;

  switch (frame_.replan_reason) {
    case FIRST_PLAN:
      cur_request.plan_reason = PlanningReason::first_plan;
      break;
    case SEG_COMPLETED_PATH:
      cur_request.plan_reason = PlanningReason::path_completed;
      break;
    case SEG_COMPLETED_USS:
      cur_request.plan_reason = PlanningReason::path_stucked;
      break;
    case STUCKED:
      cur_request.plan_reason = PlanningReason::path_stucked;
      break;
    case DYNAMIC:
      cur_request.plan_reason = PlanningReason::adjust_self_car_pose;
      break;
    case SEG_COMPLETED_COL_DET:
      cur_request.plan_reason = PlanningReason::path_stucked;
      break;
    default:
      cur_request.plan_reason = PlanningReason::none;
      break;
  }

  // generate request
  // If ego is in slot, use rs path to generate path, not astar searching.
  double dist = start.DistanceTo(real_end);
  bool need_drive_forward = false;
  if (dist < 3.0 && std::fabs(start.theta) < 5.0 * M_PI / 180.0 &&
      history_request.history_gear == AstarPathGear::reverse) {
    need_drive_forward = true;

    ILOG_INFO << "use rs path";
  }

  if (need_drive_forward) {
    cur_request.path_generate_method =
        planning::AstarPathGenerateType::reeds_shepp;

    end = real_end;
    end.x = real_end.x + 20.0;
    cur_request.goal_ = end;

  } else {
    // gear need be different with history in next replanning
    if (frame_.replan_reason != 1) {
      cur_request.first_action_request.has_request = true;

      switch (history_request.history_gear) {
        case AstarPathGear::reverse:
          cur_request.first_action_request.gear_request = AstarPathGear::drive;
          break;
        case AstarPathGear::drive:
          cur_request.first_action_request.gear_request =
              AstarPathGear::reverse;
          break;
        default:
          cur_request.first_action_request.has_request = false;
          break;
      }
    }

    cur_request.goal_ = end;
  }

  // search state
  AstarResponse response;

  // check result
  if (thread_state_ == RequestResponseState::has_response) {
    // get output
    // HybridAStarResult full_length_path;
    // std::vector<AStarPathPoint> first_seg_path;
    // Pose2D base_pose;

    thread_solver->PublishResponse(&response);
    ILOG_INFO << "publish path";

    bool is_nice = false;

    is_nice = IsResponseNice(cur_request, response);
    if (!is_nice) {
      thread_solver->ResetResponse();
      thread_state_ = RequestResponseState::none;
    }
  }

  // publish result
  if (thread_state_ == RequestResponseState::has_response) {
    Transform2d response_tf;
    response_tf.SetBasePose(response.request.base_pose_);

    // success
    if (response.first_seg_path.size() >= 5) {
      std::vector<pnc::geometry_lib::PathPoint> local_path;
      size_t i;
      for (i = 0; i < response.first_seg_path.size(); i++) {
        local_path.emplace_back(pnc::geometry_lib::PathPoint(
            Eigen::Vector2d(response.first_seg_path[i].x,
                            response.first_seg_path[i].y),
            response.first_seg_path[i].phi, response.first_seg_path[i].kappa));
      }

      // todo:
      if (frame_.is_replan_first) {
        frame_.is_replan_first = false;
      }

      double search_end_time = IflyTime::Now_ms();

      double path_dist = 0.0;
      if (response.first_seg_path.size() > 0) {
        path_dist = response.first_seg_path.back().accumulated_s;
      }

      ILOG_INFO << "gear = "
                << PathGearDebugString(response.first_seg_path[0].gear)
                << " ,dist = " << path_dist;

      // if (response.kappa_change_too_much) {
      //   ILOG_INFO << "path kappa change too much";

      // } else {
      // }

      PathOptimizationByCILRQ(path_dist, local_path, &response_tf);

      double lqr_end_time = IflyTime::Now_ms();
      ILOG_INFO << "lqr time ms " << lqr_end_time - search_end_time;

      PublishHybridAstarDebugInfo(response.result, thread_solver, &response_tf);

      double publish_end_time = IflyTime::Now_ms();
      ILOG_INFO << "publish time ms " << publish_end_time - lqr_end_time;

      res = PathPlannerResult::PLAN_UPDATE;
    } else {
      res = PathPlannerResult::WAIT_PATH;

      // publish fallback path
      GenerateFallBackPath();

      ILOG_INFO << "path plan point less 5";
    }

    if (response.first_seg_path.size() > 0) {
      thread_solver->ResetResponse();
      ILOG_INFO << "clear thread";
    }

  } else if (thread_state_ == RequestResponseState::none ||
             thread_state_ == RequestResponseState::has_published_response) {
    // send request
    thread_solver->SetRequest(obs, cur_request);
    res = PathPlannerResult::WAIT_PATH;

    // publish fallback path
    GenerateFallBackPath();

    ILOG_INFO << "set input";
  } else if (thread_state_ == RequestResponseState::has_request) {
    res = PathPlannerResult::WAIT_PATH;

    // publish fallback path
    GenerateFallBackPath();
    HybridAstarDebugInfoClear();

    ILOG_INFO << "has input";
  }

  ILOG_INFO << "path size " << current_path_point_global_vec_.size();

  return res;
}

const int HybridAStarParkPlanner::PublishHybridAstarDebugInfo(
    const HybridAStarResult& result, HybridAStarThreadSolver* thread,
    Transform2d* tf) {
  if (result.x.size() < 1) {
    ILOG_INFO << "no path";
    return 0;
  }

  size_t i;
  Pose2D local_position;
  Pose2D global_position;

  auto& debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();

  debug_->mutable_refline_info()->Clear();

  for (i = 0; i < result.x.size(); i++) {
    local_position.x = result.x[i];
    local_position.y = result.y[i];
    local_position.theta = result.phi[i];

    tf->ULFLocalPoseToGlobal(&global_position, local_position);

    planning::common::TrajectoryPoint* point = debug_->add_refline_info();

    point->set_x(global_position.x);
    point->set_y(global_position.y);
    point->set_heading_angle(global_position.theta);
    point->set_s(result.accumulated_s[i]);

    // todo, add hybrid astar msg. but now reuse TrajectoryPoint.
    if (result.type[i] == AstarPathType::Reeds_Shepp) {
      point->set_l(-1.0);
    } else {
      point->set_l(1.0);
    }
  }

  planning::common::AstarNodeList* list = debug_->mutable_node_list();
  list->Clear();

  thread->GetNodeListMessagePublish(list);
  ILOG_INFO << "list size " << list->nodes_size();

  for (int i = 0; i < list->nodes_size(); i++) {
    for (int j = 0; j < list->nodes(i).path_point_size(); j++) {
      local_position.x = list->nodes(i).path_point(j).x();
      local_position.y = list->nodes(i).path_point(j).y();

      tf->ULFLocalPoseToGlobal(&global_position, local_position);

      list->mutable_nodes(i)->mutable_path_point(j)->set_x(global_position.x);
      list->mutable_nodes(i)->mutable_path_point(j)->set_y(global_position.y);
    }
  }

  return 0;
}

const int HybridAStarParkPlanner::HybridAstarDebugInfoClear() {
  auto& debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();

  // debug_->mutable_refline_info()->Clear();

  planning::common::AstarNodeList* list = debug_->mutable_node_list();
  list->Clear();

  return 0;
}

const int HybridAStarParkPlanner::GenerateFallBackPath() {
  const auto& ego_slot_info = frame_.ego_slot_info;

  if (frame_.replan_reason == 5) {
    return 0;
  }

  current_path_point_global_vec_.clear();

  pnc::geometry_lib::PathPoint global_point;
  global_point.Set(
      ego_slot_info.l2g_tf.GetPos(ego_slot_info.ego_pos_slot),
      ego_slot_info.l2g_tf.GetHeading(ego_slot_info.ego_heading_slot));
  global_point.kappa = 0.0;

  // todo, use one point
  current_path_point_global_vec_.emplace_back(global_point);

  return 0;
}

const int HybridAStarParkPlanner::PathOptimizationByCILRQ(
    const double paht_s,
    const std::vector<pnc::geometry_lib::PathPoint>& local_path,
    Transform2d* tf) {
  LocalPathToGlobal(local_path, tf);
  ILOG_INFO << "output path by coarse a star path";

  return 0;
}

const int HybridAStarParkPlanner::LocalPathToGlobal(
    const std::vector<pnc::geometry_lib::PathPoint>& local_path,
    Transform2d* tf) {
  // TODO: longitudinal path optimization
  current_path_point_global_vec_.clear();
  current_path_point_global_vec_.reserve(local_path.size());

  pnc::geometry_lib::PathPoint global_point;
  Pose2D global;
  for (const auto& path_point : local_path) {
    tf->ULFLocalPoseToGlobal(
        &global,
        Pose2D(path_point.pos.x(), path_point.pos.y(), path_point.heading));

    global_point.Set(Eigen::Vector2d(global.x, global.y), global.theta);
    global_point.kappa = path_point.kappa;

    current_path_point_global_vec_.emplace_back(global_point);
  }

  return 0;
}

const bool HybridAStarParkPlanner::UpdateThreadPath() {
  HybridAStarThreadSolver* thread_solver =
      HybridAStarThreadSolver::GetInstance();

  if (thread_solver == nullptr) {
    return false;
  }

  thread_solver->GetThreadState(&thread_state_);

  ILOG_INFO << "thread state " << static_cast<int>(thread_state_);

  if (thread_state_ == RequestResponseState::has_response) {
    ILOG_INFO << "fetch path";

    return true;
  }

  return false;
}

const bool HybridAStarParkPlanner::UpdateEgoSlotInfo() {
  const auto* measures_ptr = &apa_world_ptr_->GetApaDataPtr()->measurement_data;
  const auto slot_manager_ptr_ = apa_world_ptr_->GetSlotManagerPtr();

  frame_.correct_path_for_limiter = false;
  frame_.replan_flag = false;

  EgoSlotInfo& ego_slot_info = frame_.ego_slot_info;

    ego_slot_info.target_managed_slot =
        slot_manager_ptr_->GetEgoSlotInfo().select_slot_filter;

    const auto& slot_points =
        ego_slot_info.target_managed_slot.corner_points().corner_point();
    std::vector<Eigen::Vector2d> pt;
    pt.resize(4);
    for (size_t i = 0; i < 4; ++i) {
      if (apa_world_ptr_->GetApaDataPtr()->simu_param.is_simulation &&
          apa_world_ptr_->GetApaDataPtr()
                  ->simu_param.target_managed_slot_x_vec.size() == 4 &&
          apa_world_ptr_->GetApaDataPtr()->simu_param.use_slot_in_bag) {
        pt[i] << apa_world_ptr_->GetApaDataPtr()
                     ->simu_param.target_managed_slot_x_vec[i],
            apa_world_ptr_->GetApaDataPtr()
                ->simu_param.target_managed_slot_y_vec[i];
      } else {
        pt[i] << slot_points[i].x(), slot_points[i].y();
      }
    }
    const Eigen::Vector2d pM01 = 0.5 * (pt[0] + pt[1]);
    const Eigen::Vector2d pM23 = 0.5 * (pt[2] + pt[3]);
    const double real_slot_length = (pM01 - pM23).norm();
    const Eigen::Vector2d t = (pt[1] - pt[0]).normalized();
    // n is vec that slot opening orientation
    const Eigen::Vector2d n = Eigen::Vector2d(t.y(), -t.x());
    // const Eigen::Vector2d n = (pM01 - pM23).normalized();
    pt[2] = pt[0] - real_slot_length * n;
    pt[3] = pt[1] - real_slot_length * n;

    ego_slot_info.slot_corner = pt;

    ego_slot_info.slot_center = (pt[0] + pt[1] + pt[2] + pt[3]) * 0.25;

    // const double virtual_slot_length =
    //     apa_param.GetParam().car_length +
    //     apa_param.GetParam().slot_compare_to_car_length;

    // const double use_slot_length =
    //     std::min(real_slot_length, virtual_slot_length);

    const double use_slot_length = real_slot_length;

    ego_slot_info.slot_origin_pos = pM01 - use_slot_length * n;
    ego_slot_info.slot_origin_heading = std::atan2(n.y(), n.x());
    ego_slot_info.slot_origin_heading_vec = n;
    ego_slot_info.slot_length = use_slot_length;
    ego_slot_info.slot_width = (pt[0] - pt[1]).norm();

    ego_slot_info.g2l_tf.Init(ego_slot_info.slot_origin_pos,
                              ego_slot_info.slot_origin_heading);

    ego_slot_info.l2g_tf.Init(ego_slot_info.slot_origin_pos,
                              ego_slot_info.slot_origin_heading);

    if (ego_slot_info.target_managed_slot.slot_type() ==
        Common::PARKING_SLOT_TYPE_SLANTING) {
      const Eigen::Vector2d origin_pt_0 =
          Eigen::Vector2d(slot_manager_ptr_->GetEgoSlotInfo()
                              .select_fusion_slot.corner_points[0]
                              .x,
                          slot_manager_ptr_->GetEgoSlotInfo()
                              .select_fusion_slot.corner_points[0]
                              .y);

      const Eigen::Vector2d origin_pt_1 =
          Eigen::Vector2d(slot_manager_ptr_->GetEgoSlotInfo()
                              .select_fusion_slot.corner_points[1]
                              .x,
                          slot_manager_ptr_->GetEgoSlotInfo()
                              .select_fusion_slot.corner_points[1]
                              .y);

      ego_slot_info.pt_0 = ego_slot_info.g2l_tf.GetPos(origin_pt_0);
      ego_slot_info.pt_1 = ego_slot_info.g2l_tf.GetPos(origin_pt_1);

      if (ego_slot_info.pt_0.y() > ego_slot_info.pt_1.y()) {
        std::swap(ego_slot_info.pt_0, ego_slot_info.pt_1);
      }

      const Eigen::Vector2d pt_01_vec = ego_slot_info.pt_1 - ego_slot_info.pt_0;

      double angle = std::fabs(pnc::geometry_lib::GetAngleFromTwoVec(
                         Eigen::Vector2d(real_slot_length, 0.0), pt_01_vec)) *
                     kRad2Deg;

      if (angle > 90.0) {
        angle = 180.0 - angle;
      }

      angle = pnc::mathlib::DoubleConstrain(angle, 10.0, 80.0);
      ego_slot_info.sin_angle = std::sin(angle * kDeg2Rad);
      ego_slot_info.origin_pt_0_heading = 90.0 - angle;
    } else {
      ego_slot_info.pt_0 = ego_slot_info.g2l_tf.GetPos(pt[0]);
      ego_slot_info.pt_1 = ego_slot_info.g2l_tf.GetPos(pt[1]);
      if (ego_slot_info.pt_0.y() > ego_slot_info.pt_1.y()) {
        std::swap(ego_slot_info.pt_0, ego_slot_info.pt_1);
      }
      ego_slot_info.sin_angle = 1.0;
      ego_slot_info.origin_pt_0_heading = 0.0;
    }

  ego_slot_info.ego_pos_slot = ego_slot_info.g2l_tf.GetPos(measures_ptr->pos);

  ego_slot_info.ego_heading_slot =
      ego_slot_info.g2l_tf.GetHeading(measures_ptr->heading);

  ego_slot_info.ego_heading_slot_vec
      << std::cos(ego_slot_info.ego_heading_slot),
      std::sin(ego_slot_info.ego_heading_slot);

  ego_slot_info.fus_obj_valid_flag =
      slot_manager_ptr_->GetEgoSlotInfo().fus_obj_valid_flag;
  ego_slot_info.obs_pt_vec_slot.clear();
  ego_slot_info.obs_pt_vec_slot.reserve(
      slot_manager_ptr_->GetEgoSlotInfo().obs_pt_vec_slot.size());

  for (const Eigen::Vector2d& obs_pt :
       slot_manager_ptr_->GetEgoSlotInfo().obs_pt_vec_slot) {
    const Eigen::Vector2d obs_pt_slot = ego_slot_info.g2l_tf.GetPos(obs_pt);
    ego_slot_info.obs_pt_vec_slot.emplace_back(std::move(obs_pt_slot));
  }
  // ego_slot_info.obs_pt_vec_slot =
  //     slot_manager_ptr_->GetEgoSlotInfo().obs_pt_vec_slot;

  ego_slot_info.limiter = slot_manager_ptr_->GetEgoSlotInfo().limiter;

  if (apa_world_ptr_->GetApaDataPtr()->simu_param.is_simulation &&
      apa_world_ptr_->GetApaDataPtr()
              ->simu_param.target_managed_limiter_x_vec.size() == 2 &&
      apa_world_ptr_->GetApaDataPtr()->simu_param.use_slot_in_bag) {
    ego_slot_info.limiter.first
        << apa_world_ptr_->GetApaDataPtr()
               ->simu_param.target_managed_limiter_x_vec[0],
        apa_world_ptr_->GetApaDataPtr()
            ->simu_param.target_managed_limiter_y_vec[0];
    ego_slot_info.limiter.first =
        ego_slot_info.g2l_tf.GetPos(ego_slot_info.limiter.first);
    ego_slot_info.limiter.second
        << apa_world_ptr_->GetApaDataPtr()
               ->simu_param.target_managed_limiter_x_vec[1],
        apa_world_ptr_->GetApaDataPtr()
            ->simu_param.target_managed_limiter_y_vec[1];
    ego_slot_info.limiter.second =
        ego_slot_info.g2l_tf.GetPos(ego_slot_info.limiter.second);
  }

  ego_slot_info.limiter_corner.clear();
  ego_slot_info.limiter_corner.reserve(2);
  ego_slot_info.limiter_corner.emplace_back(ego_slot_info.limiter.first);
  ego_slot_info.limiter_corner.emplace_back(ego_slot_info.limiter.second);

  // cal target pos
  ego_slot_info.target_ego_pos_slot
      << (ego_slot_info.limiter.first.x() + ego_slot_info.limiter.second.x()) /
             2.0,
      apa_param.GetParam().terminal_target_y;
  ego_slot_info.target_ego_heading_slot =
      apa_param.GetParam().terminal_target_heading;

  // cal terminal error
  ego_slot_info.terminal_err.Set(
      ego_slot_info.ego_pos_slot - ego_slot_info.target_ego_pos_slot,
      ego_slot_info.ego_heading_slot - ego_slot_info.target_ego_heading_slot);

  // cal slot occupied ratio
  if (std::fabs(ego_slot_info.terminal_err.pos.y()) <
          apa_param.GetParam().slot_occupied_ratio_max_lat_err &&
      std::fabs(ego_slot_info.ego_heading_slot) <
          apa_param.GetParam().slot_occupied_ratio_max_heading_err * kDeg2Rad) {
    const std::vector<double> x_tab = {
        ego_slot_info.target_ego_pos_slot.x(),
        ego_slot_info.slot_length + apa_param.GetParam().rear_overhanging};

    const std::vector<double> occupied_ratio_tab = {1.0, 0.0};
    ego_slot_info.slot_occupied_ratio = pnc::mathlib::Interp1(
        x_tab, occupied_ratio_tab, ego_slot_info.ego_pos_slot.x());
  } else {
    ego_slot_info.slot_occupied_ratio = 0.0;
  }

  // calc slot side and init gear and init steer at first
  if (frame_.is_replan_first) {
    frame_.car_already_move_dist = 0.0;

    const auto pM01 =
        0.5 * (ego_slot_info.slot_corner[0] + ego_slot_info.slot_corner[1]);
    const auto pM23 =
        0.5 * (ego_slot_info.slot_corner[2] + ego_slot_info.slot_corner[3]);
    Eigen::Vector2d ego_to_slot_center_vec =
        0.5 * (pM01 + pM23) - measures_ptr->pos;

    const double cross_ego_to_slot_center =
        pnc::geometry_lib::GetCrossFromTwoVec2d(measures_ptr->heading_vec,
                                                ego_to_slot_center_vec);

    const double cross_ego_to_slot_heading =
        pnc::geometry_lib::GetCrossFromTwoVec2d(
            measures_ptr->heading_vec, ego_slot_info.slot_origin_heading_vec);

    // judge slot side via slot center and heading
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
    if (cross_ego_to_slot_heading > 0.0 && cross_ego_to_slot_center < 0.0) {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_RIGHT;
    } else if (cross_ego_to_slot_heading < 0.0 &&
               cross_ego_to_slot_center > 0.0) {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_LEFT;
    } else {
      frame_.current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;
      frame_.current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
    }
  }

  // update stuck by uss time
  if (frame_.plan_stm.planning_status == PARKING_RUNNING &&
      measures_ptr->static_flag && !measures_ptr->brake_flag &&
      apa_world_ptr_->GetApaDataPtr()->cur_state ==
          ApaStateMachine::ACTIVE_IN) {
    frame_.stuck_uss_time += apa_param.GetParam().plan_time;
  } else {
    frame_.stuck_uss_time = 0.0;
  }

  // update stuck time
  if ((frame_.plan_stm.planning_status == PARKING_RUNNING ||
       frame_.plan_stm.planning_status == PARKING_PLANNING) &&
      measures_ptr->static_flag && !measures_ptr->brake_flag &&
      apa_world_ptr_->GetApaDataPtr()->cur_state ==
          ApaStateMachine::ACTIVE_IN) {
    frame_.stuck_time += apa_param.GetParam().plan_time;
  } else {
    frame_.stuck_time = 0.0;
  }

  // update pause time
  if (frame_.plan_stm.planning_status == PARKING_PAUSED) {
    frame_.pause_time += apa_param.GetParam().plan_time;
  } else {
    frame_.pause_time = 0.0;
  }

  // fix slot
  if (ego_slot_info.slot_occupied_ratio >
          apa_param.GetParam().fix_slot_occupied_ratio &&
      !frame_.is_fix_slot && measures_ptr->static_flag) {
    frame_.is_fix_slot = true;
    ego_slot_info.fix_limiter = true;
  }

  return true;
}

HybridAStarParkPlanner::~HybridAStarParkPlanner() {}

}  // namespace apa_planner
}  // namespace planning