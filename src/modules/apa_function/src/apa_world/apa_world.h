#ifndef __APA_WORLD_H__
#define __APA_WORLD_H__

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <memory>

#include "collision_detection.h"
#include "common.pb.h"
#include "func_state_machine_c.h"
#include "local_view.h"
#include "slot_management.h"
#include "slot_management_info.pb.h"
#include "spline.h"
#include "spline_projection.h"
#include "uss_obstacle_avoidance.h"

#define APA_COMPARE_PLANNING_TRAJ_POINTS_NUM 26

namespace planning {
namespace apa_planner {

class ApaWorld {
 public:
  enum ApaPlannerType {
    PERPENDICULAR_PARK_IN_PLANNER,
    PARALLEL_PARK_IN_PLANNER,
    PLANNER_COUNT,
    NONE_PLANNER,
  };

  enum GeneralApaFunction {
    NONE_FUNCTION,
    PARK_IN_FUNCTION,
    PARK_OUT_FUNCTION,
  };

 public:
  struct Measurements {
    // systems states
    uint8_t planner_type = NONE_PLANNER;
    uint8_t current_state = iflyauto::FunctionalState_PARK_STANDBY;
    uint8_t general_apa_function = GeneralApaFunction::NONE_FUNCTION;
    uint8_t history_apa_function = GeneralApaFunction::NONE_FUNCTION;

    // measurements
    double vel_ego = 0.0;
    Eigen::Vector2d pos_ego = Eigen::Vector2d::Zero();

    double heading_ego = 0.0;
    Eigen::Vector2d heading_ego_vec = Eigen::Vector2d::Zero();

    // static params
    double car_static_timer_by_pos_strict = 0.0;
    double car_static_timer_by_pos_normal = 0.0;
    double car_static_timer_by_vel_strict = 0.0;
    double car_static_timer_by_vel_normal = 0.0;
    bool static_flag = false;

    uint8_t slot_type = Common::PARKING_SLOT_TYPE_VERTICAL;
    common::SlotInfo target_managed_slot;

    void Reset() {
      planner_type = NONE_PLANNER;
      general_apa_function = NONE_FUNCTION;
      history_apa_function = NONE_FUNCTION;
      vel_ego = 0.0;
      pos_ego.setZero();
      heading_ego_vec.setZero();
      heading_ego = 0.0;
      car_static_timer_by_pos_strict = 0.0;
      car_static_timer_by_pos_normal = 0.0;
      car_static_timer_by_vel_strict = 0.0;
      car_static_timer_by_vel_normal = 0.0;
      static_flag = false;
      current_state = iflyauto::FunctionalState_PARK_STANDBY;
    }
  };

  ApaWorld() { Init(); }
  ~ApaWorld() {}

  void Init();
  void Reset();
  const bool Update(const LocalView* const local_view);
  const bool Update();
  std::shared_ptr<Measurements> GetMeasurementsPtr() { return measures_ptr_; }

  std::shared_ptr<SlotManagement> GetSlotManagerPtr() {
    return slot_manager_ptr_;
  }

  std::shared_ptr<UssObstacleAvoidance> GetUssObstacleAvoidancePtr() {
    return uss_obstacle_avoider_ptr_;
  }

  std::shared_ptr<CollisionDetector> GetCollisionDetectorPtr() {
    return collision_detector_ptr_;
  }

  const LocalView* GetLocalViewPtr() { return local_view_ptr_; }

 private:
  void Preprocess();
  void UpdateEgoState();

  const bool CheckSelectedSlot() const;
  const bool CheckParkInState() const;
  const bool CheckParkInActivated() const;
  const bool CheckParkOutState() const;

  std::shared_ptr<SlotManagement> slot_manager_ptr_;
  std::shared_ptr<Measurements> measures_ptr_;
  std::shared_ptr<UssObstacleAvoidance> uss_obstacle_avoider_ptr_;
  std::shared_ptr<CollisionDetector> collision_detector_ptr_;

  const LocalView* local_view_ptr_ = nullptr;
};

}  // namespace apa_planner
}  // namespace planning

#endif
