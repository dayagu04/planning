#ifndef __PARKING_SCENARIO_H__
#define __PARKING_SCENARIO_H__

#include <bits/stdint-uintn.h>
#include <sys/types.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "apa_param_config.h"
#include "apa_world.h"
#include "collision_detection/collision_detection.h"
#include "geometry_math.h"
#include "lateral_path_optimizer.h"
#include "local_view.h"
#include "planning_hmi_c.h"
#include "planning_plan_c.h"

#include "parking_task/parking_task.h"

namespace planning {
namespace apa_planner {

enum class ParkingScenarioStatus {
  STATUS_UNKNOWN = 0,
  // 表示点击泊车，这个场景正在运行
  STATUS_RUNNING = 1,
  STATUS_DONE = 2,
  // 表示点击车位，尝试计算这个场景
  STATUS_TRY = 3,
  STATUS_FAIL = 4,
};

enum PathPlannerResult {
  PLAN_FAILED,  // path plan failed
  PLAN_HOLD,    // follow last
  PLAN_UPDATE,
  WAIT_PATH,
};

// 1. 对于泊车而言，不同的行为对应不同的场景，包括垂直泊入场景、垂直泊出场景,等等.
// 2. 每一个场景, 都有不同的计算任务.
// 例如计算任务包括：路径计算,速度计算,超声波处理,
// 障碍物决策,目标点位置决策,路径优化,速度优化,等等.
// 3. 对于路径计算这个任务而言，一般可以分为curve_based_path_generator,
// hybrid_astar_path_generator.
// TODO: move all scenario related codes in here. and move all task related code
// to Task. For long term development, you can create new Task and update old
// Task.
class ParkingScenario {
 public:
  enum ReplanReason {
    NOT_REPLAN,
    FIRST_PLAN,
    SEG_COMPLETED_PATH,
    SEG_COMPLETED_USS,
    STUCKED,
    DYNAMIC,
    SEG_COMPLETED_COL_DET,
    FORCE_PLAN,
  };

  struct EgoSlotInfo {
    common::SlotInfo target_managed_slot;
    std::pair<Eigen::Vector2d, Eigen::Vector2d> limiter;
    Eigen::Vector2d target_ego_pos_slot = Eigen::Vector2d::Zero();
    double target_ego_heading_slot = 0.0;

    std::vector<Eigen::Vector2d> slot_corner;
    double last_move_slot_dist = 0.0;
    double move_slot_dist = 0.0;
    std::vector<Eigen::Vector2d> limiter_corner;

    Eigen::Vector2d slot_center;

    size_t selected_slot_id = 0;
    size_t slot_type = 0;

    Eigen::Vector2d slot_origin_pos = Eigen::Vector2d::Zero();
    double slot_origin_heading = 0.0;
    Eigen::Vector2d slot_origin_heading_vec = Eigen::Vector2d::Zero();

    Eigen::Vector2d ego_pos_slot = Eigen::Vector2d::Zero();
    Eigen::Vector2d ego_heading_slot_vec = Eigen::Vector2d::Zero();
    double ego_heading_slot = 0.0;

    double slot_width = 3.0;
    double slot_length = 5.2;

    pnc::geometry_lib::PathPoint terminal_err;
    pnc::geometry_lib::PathPoint initial_err;
    double slot_occupied_ratio = 0.0;

    pnc::geometry_lib::GlobalToLocalTf g2l_tf;
    pnc::geometry_lib::LocalToGlobalTf l2g_tf;

    bool fix_limiter = false;

    std::vector<Eigen::Vector2d> obs_pt_vec_slot;

    double origin_pt_0_heading = 0.0;
    double sin_angle = 1.0;
    Eigen::Vector2d pt_0;
    Eigen::Vector2d pt_1;

    double channel_width;

    bool fus_obj_valid_flag = false;

    void Reset() {
      target_managed_slot.Clear();
      selected_slot_id = 0;
      slot_type = 0;
      target_ego_pos_slot = Eigen::Vector2d::Zero();
      target_ego_heading_slot = 0.0;

      slot_corner.clear();
      limiter_corner.clear();
      move_slot_dist = 0.0;
      last_move_slot_dist = 0.0;

      slot_center.setZero();

      limiter.first.setZero();
      limiter.second.setZero();

      slot_origin_pos.setZero();

      slot_origin_heading = 0.0;
      slot_origin_heading_vec.setZero();

      ego_heading_slot_vec.setZero();
      ego_heading_slot = 0.0;

      ego_pos_slot.setZero();

      slot_width = 3.0;
      slot_length = 4.8;

      terminal_err.Set(Eigen::Vector2d(1.0, 1.0), 0.5);
      slot_occupied_ratio = 0.0;

      fix_limiter = false;

      obs_pt_vec_slot.clear();

      origin_pt_0_heading = 0.0;
      sin_angle = 1.0;
      pt_0.setZero();
      pt_1.setZero();

      channel_width = apa_param.GetParam().channel_width;

      fus_obj_valid_flag = false;
    }
  };

  struct PlannerStateMachine {  // planner states
    bool path_plan_success = false;
    uint8_t planning_status = ParkingStatus::PARKING_IDLE;

    void Reset() {
      path_plan_success = false;
      planning_status = ParkingStatus::PARKING_IDLE;
    }
  };

  struct Frame {
    void Reset() {
      gear_change_count = 0;
      is_replan = false;
      // parking stage: go to prepare point; go to slot point;
      is_replan_first = true;
      is_replan_second = false;
      is_replan_dynamic = false;
      is_dynamic_replan_first = true;
      dynamic_replan_count = 0;
      total_plan_count = 0;
      in_slot_plan_count = 0;
      is_finished = false;
      is_fix_slot = false;
      stuck_time = 0.0;
      stuck_uss_time = 0.0;
      pause_time = 0.0;
      dynamic_plan_time = 0.0;
      remain_dist = 5.01;
      remain_dist_uss = 5.01;
      remain_dist_col_det = 5.01;
      vel_target = 1.168;
      car_already_move_dist = 0.0;
      spline_success = false;
      current_path_length = 0.0;
      headin_current_path_length = 0.0;
      path_extended_dist = 1.0;
      is_replan_by_uss = false;
      ego_slot_info.Reset();
      plan_stm.Reset();
      pathplan_result = 0;
      current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
      current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;
      replan_reason = NOT_REPLAN;
      plan_fail_reason = NOT_FAILED;
      need_update_slot = true;
      correct_path_for_limiter = false;
      replan_flag = false;
      dynamic_plan_fail_flag = false;
      gear_command = pnc::geometry_lib::SEG_GEAR_INVALID;

      is_left_empty = false;
      is_right_empty = false;

      ego_car_info_slot.Reset();
    }
    bool is_left_empty = false;
    bool is_right_empty = false;

    bool is_replan = false;
    bool is_replan_first = true;
    bool is_replan_second = false;
    bool is_replan_dynamic = false;
    bool is_dynamic_replan_first = true;
    uint8_t dynamic_replan_count = 0;
    uint8_t replan_reason = NOT_REPLAN;
    uint8_t plan_fail_reason = NOT_FAILED;
    int total_plan_count = 0;
    uint8_t in_slot_plan_count = 0;
    bool is_finished = false;
    bool is_fix_slot = false;
    bool spline_success = false;
    bool is_replan_by_uss = false;
    uint8_t pathplan_result = 0;
    size_t gear_change_count = 0;
    double current_path_length = 0.0;
    double headin_current_path_length = 0.0;
    double path_extended_dist = 1.0;
    double vel_target = 1.168;
    double stuck_time = 0.0;
    double stuck_uss_time = 0.0;
    double pause_time = 0.0;
    double dynamic_plan_time = 0.0;
    double remain_dist = 5.01;
    double remain_dist_uss = 5.01;
    double remain_dist_col_det = 5.01;
    double car_already_move_dist = 0.0;
    pnc::mathlib::spline x_s_spline;
    pnc::mathlib::spline y_s_spline;
    pnc::mathlib::spline headin_x_s_spline;
    pnc::mathlib::spline headin_y_s_spline;

    PlannerStateMachine plan_stm;
    EgoSlotInfo ego_slot_info;

    uint8_t current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
    uint8_t current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;

    bool need_update_slot = true;

    bool correct_path_for_limiter = false;
    bool replan_flag = false;
    bool dynamic_plan_fail_flag = false;

    uint8_t gear_command = pnc::geometry_lib::SEG_GEAR_INVALID;

    EgoCarInfoUnderSlot ego_car_info_slot;
  };

  enum ParkingStatus {
    PARKING_IDLE,        // not in control
    PARKING_RUNNING,     // in control and no replan
    PARKING_GEARCHANGE,  // replan by non-updated path
    PARKING_PLANNING,    // replan by updated path
    PARKING_FINISHED,    // parking successful
    PARKING_FAILED,      // parking failed
    PARKING_PAUSED,      // parking paused
  };

  enum ParkingFailReason {
    NOT_FAILED,
    PAUSE_FAILED_TIME,
    STUCK_FAILED_TIME,
    UPDATE_EGO_SLOT_INFO,
    POST_PROCESS_PATH_POINT_SIZE,
    POST_PROCESS_PATH_POINT_SAME,
    SET_SEG_INDEX,
    CHECK_GEAR_LENGTH,
    PATH_PLAN_FAILED,
    PLAN_COUNT_EXCEED_LIMIT,
  };

 public:
  virtual ~ParkingScenario() = default;

  ParkingScenario();

  ParkingScenario(const std::shared_ptr<ApaWorld>& apa_world_ptr);

  virtual void Init();

  // 点击泊车之后，更新speed and path
  virtual void ScenarioRunning();

  virtual void Reset();

  virtual std::string GetName();

  const PlannerStateMachine &GetPlannerStates() const {
    return frame_.plan_stm;
  }

  const Frame &GetFrame() const { return frame_; }

  const iflyauto::PlanningOutput &GetOutput() const { return planning_output_; }

  const iflyauto::APAHMIData &GetAPAHmi() const { return apa_hmi_; }

  void SetApaWorldPtr(const std::shared_ptr<ApaWorld> &apa_world_ptr) {
    apa_world_ptr_ = apa_world_ptr;
  }

  const std::shared_ptr<ApaWorld> GetApaWorldPtr() { return apa_world_ptr_; }

  virtual void Process();

  virtual void Exit();

  virtual void Enter(const ParkingScenarioStatus status);

  const ParkingScenarioStatus& GetStatus() const {
    return scenario_status_;
  }

  const std::string& Name() const { return name_; }

  const ParkingScenarioType ScenarioType() const { return type_; }

  // 如果用户点击了车位，但是还没有点击开始泊车，那么调用这个函数，尝试一下看看路径生成是否成功.
  virtual const ParkingScenarioStatus ScenarioTry();

  // clear thread related
  virtual void ThreadClear();

  void SetScenerioType(const ParkingScenarioType type) {
    type_ = type;
    return;
  }

 protected:
  virtual const bool CheckFinished() = 0;
  virtual const bool CheckReplan() = 0;
  virtual void PlanCore() = 0;
  virtual void Log() const = 0;
  virtual void GenTlane() = 0;
  virtual void GenObstacles() = 0;
  virtual const bool UpdateEgoSlotInfo() = 0;
  virtual const uint8_t PathPlanOnce() = 0;

  virtual void InitSimulation();
  virtual const bool CheckPaused() const;
  virtual const bool CheckPlanSkip() const;
  virtual void SetParkingStatus(uint8_t status);
  virtual void GenPlanningOutput();
  virtual void GenPlanningHmiOutput();
  virtual void GenPlanningPath();
  virtual const bool CheckStuckFailed();
  virtual void UpdateRemainDist(const double uss_safe_dist);
  virtual const double CalRemainDistFromPath();
  virtual const double CalRemainDistFromUss(const double safe_dist);
  virtual const bool PostProcessPath();

  void CreateTasks();

 protected:
  // TODO:
  // 1.
  // 场景内部，尽量只包含指针，不要创建大的数据，因为一旦场景多了之后，这些数据都是重复继承;
  // 比如：planning_output_，apa_hmi_，frame_，current_path_point_global_vec_,
  // 这种大的数据（10多Mb）就可以放到apa_world中.
  // 2. 泊车框架有2个数据流：
  // A：apa_data，存储处理的上游数据，以及发布给下游的数据；
  // B: apa_world，存储scenario, task各种内部数据;
  std::shared_ptr<ApaWorld> apa_world_ptr_;

  iflyauto::PlanningOutput planning_output_;
  iflyauto::APAHMIData apa_hmi_;

  Frame frame_;

  std::vector<pnc::geometry_lib::PathPoint> current_path_point_global_vec_;

  ParkingScenarioStatus scenario_status_;
  ParkingScenarioType type_;

  std::string name_;
};

}  // namespace apa_planner
}  // namespace planning

#endif
