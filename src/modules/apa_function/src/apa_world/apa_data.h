#ifndef __APA_DATA_H__
#define __APA_DATA_H__

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>
#include "Eigen/Core"
#include "local_view.h"

namespace planning {
namespace apa_planner {

struct SimulationParam {
  bool is_simulation = false;
  bool is_complete_path = false;
  bool force_plan = false;
  bool sim_to_target = false;
  bool use_slot_in_bag = true;
  bool use_obs_in_bag = true;
  bool is_path_optimization = false;
  bool is_cilqr_optimization = false;
  bool is_reset = false;
  double sample_ds = 0.02;
  std::vector<double> target_managed_slot_x_vec;
  std::vector<double> target_managed_slot_y_vec;
  std::vector<double> target_managed_limiter_x_vec;
  std::vector<double> target_managed_limiter_y_vec;
  std::vector<double> obs_x_vec;
  std::vector<double> obs_y_vec;

  double q_ref_xy = 100.0;
  double q_ref_theta = 100.0;
  double q_terminal_xy = 1000.0;
  double q_terminal_theta = 9000.0;
  double q_k = 10.0;
  double q_u = 10.0;
  double q_k_bound = 100.0;
  double q_u_bound = 50.0;
};

struct MeasurementData {
  double vel = 0.0;
  Eigen::Vector2d pos = Eigen::Vector2d::Zero();
  double heading = 0.0;
  Eigen::Vector2d heading_vec = Eigen::Vector2d::Zero();
  Eigen::Vector2d right_mirror_pos = Eigen::Vector2d::Zero();
  Eigen::Vector2d left_mirror_pos = Eigen::Vector2d::Zero();

  double car_static_timer_by_pos_strict = 0.0;
  double car_static_timer_by_pos_normal = 0.0;
  double car_static_timer_by_vel_strict = 0.0;
  double car_static_timer_by_vel_normal = 0.0;
  bool static_flag = true;

  double steer_wheel_angle = 0.0;

  bool brake_flag = false;

  void Reset() {
    vel = 0.0;
    pos.setZero();
    heading_vec.setZero();
    heading = 0.0;
    car_static_timer_by_pos_strict = 0.0;
    car_static_timer_by_pos_normal = 0.0;
    car_static_timer_by_vel_strict = 0.0;
    car_static_timer_by_vel_normal = 0.0;
    static_flag = true;
    steer_wheel_angle = 0.0;
    brake_flag = false;
  }
};

enum class ApaFunction : uint8_t {
  PARK_IN,
  PARK_OUT,
  COUNT,
  INVALID,
};

enum class ApaPlannerType : uint8_t {
  PERPENDICULAR_PARK_IN_PLANNER,
  PERPENDICULAR_PARK_HEADING_IN_PLANNER,
  PERPENDICULAR_PARK_OUT_PLANNER,
  SLANT_PARK_IN_PLANNER,
  SLANT_PARK_HEADING_IN_PLANNER,
  SLANT_PARK_OUT_PLANNER,
  PARALLEL_PARK_IN_PLANNER,
  PARALLEL_PARK_HEADING_IN_PLANNER,
  PARALLER_PARK_OUT_PLANNER,
  HYBRID_ASTAR_PLANNER,
  COUNT_PLANNER,
  INVALID_PLANNER,
};

enum class ApaStateMachine : uint8_t {
  SEARCH_IN,
  ACTIVE_WAIT_IN,
  ACTIVE_IN,
  SEARCH_OUT,
  ACTIVE_WAIT_OUT,
  ACTIVE_OUT,
  SUSPEND,
  SECURE,
  COMPLETE,
  COUNT,
  INVALID,
};

struct Limiter {
  Eigen::Vector2d start_pt = Eigen::Vector2d::Zero();
  Eigen::Vector2d end_pt = Eigen::Vector2d::Zero();
  bool valid = false;

  void Reset() {
    start_pt.setZero();
    end_pt.setZero();
    valid = false;
  }
};

struct SlotCoord {
  Eigen::Vector2d pt_1 = Eigen::Vector2d::Zero();       // left up
  Eigen::Vector2d pt_0 = Eigen::Vector2d::Zero();       // right up
  Eigen::Vector2d pt_2 = Eigen::Vector2d::Zero();       // right down
  Eigen::Vector2d pt_3 = Eigen::Vector2d::Zero();       // left_down
  Eigen::Vector2d pt_center = Eigen::Vector2d::Zero();  // center

  void Reset() {
    pt_1.setZero();
    pt_0.setZero();
    pt_2.setZero();
    pt_3.setZero();
    pt_center.setZero();
  }
};

enum class SlotType : uint8_t {
  PERPENDICULAR,
  PARALLEL,
  SLANT,
  COUNT,
  INVALID,
};

enum class SlotSide : uint8_t {
  LEFT,
  RIGHT,
  COUNT,
  INVALID,
};

struct ApaSlot {
  SlotCoord slot_coord;
  SlotType slot_type = SlotType::INVALID;
  SlotSide slot_side = SlotSide::INVALID;
  int slot_id = 0;
  Limiter limiter;
  double heading = 0.0;
  Eigen::Vector2d heading_vec = Eigen::Vector2d::Zero();

  void Reset() {
    slot_coord.Reset();
    slot_type = SlotType::INVALID;
    slot_side = SlotSide::INVALID;
    slot_id = 0;
    limiter.Reset();
    heading = 0.0;
    heading_vec.setZero();
  }
};

struct ApaSlots {
  std::vector<ApaSlot> slots_vec;
  int slot_size = 0;

  void Reset() {
    slots_vec.clear();
    slot_size = 0;
  }
};

struct UssDistance {
  std::vector<double> uss_dis_vec;  // m
  int uss_size;

  void Reset() {
    uss_dis_vec.clear();
    uss_size = 0;
  }
};

enum class ObstacleType {
  FUSION,
  GROUND_LINE,
  USS,
  VIRTUAL,
  COUNT,
  INVALID,
};

struct ApaObstacle {
  size_t id = 0;
  double vel = 0.0;
  Eigen::Vector2d center_2d = Eigen::Vector2d(0.0, 0.0);
  Eigen::Vector3d center_3d = Eigen::Vector3d(0.0, 0.0, 0.0);
  std::vector<Eigen::Vector2d> obs2d_pt_vec;
  std::vector<Eigen::Vector3d> obs3d_pt_vec;
  size_t obs2d_pt_size = 0;
  size_t obs3d_pt_size = 0;
  ObstacleType obs_type = ObstacleType::INVALID;
};

struct ApaData {
  const iflyauto::FuncStateMachine* func_state_ptr;
  const iflyauto::ParkingFusionInfo* parking_slot_ptr;
  const iflyauto::IFLYLocalization* localization_ptr;
  const iflyauto::VehicleServiceOutputInfo* vehicle_service_info_ptr;
  const iflyauto::UssWaveInfo* uss_wave_info_ptr;
  const iflyauto::UssPerceptInfo* uss_percept_info_ptr;
  const iflyauto::GroundLinePerceptionInfo* ground_line_perception_info_ptr;
  const iflyauto::FusionObjectsInfo* fusion_objects_info_ptr;
  const iflyauto::FusionOccupancyObjectsInfo* fusion_occupancy_objects_info_ptr;

  ApaStateMachine cur_state = ApaStateMachine::INVALID;
  ApaPlannerType planner_type = ApaPlannerType::INVALID_PLANNER;
  ApaFunction apa_function = ApaFunction::INVALID;

  MeasurementData measurement_data;
  ApaSlots apa_slots;
  UssDistance uss_dis;

  // 暂时用这个 无需改变太多之前代码
  std::unordered_map<ObstacleType, std::vector<Eigen::Vector2d>> apa_obs_map;

  // 后面需要用这个
  // std::unordered_map<ObstacleType, std::vector<ApaObstacle>> apa_obs_map;

  SimulationParam simu_param;

  uint8_t current_state = iflyauto::FunctionalState_PARK_STANDBY;
  uint8_t slot_type = Common::PARKING_SLOT_TYPE_INVALID;
  uint8_t slot_id = 0;
  bool is_slot_type_fixed = false;

  void Reset() {
    cur_state = ApaStateMachine::INVALID;
    planner_type = ApaPlannerType::INVALID_PLANNER;
    apa_function = ApaFunction::INVALID;

    measurement_data.Reset();
    apa_slots.Reset();
    uss_dis.Reset();
    apa_obs_map.clear();

    current_state = iflyauto::FunctionalState_PARK_STANDBY;
    slot_type = Common::PARKING_SLOT_TYPE_INVALID;
    slot_id = 0;
    is_slot_type_fixed = false;
  }
};

void PrintApaPlannerType(const ApaPlannerType planner_type);

const std::string GetApaPlannerTypeString(const ApaPlannerType planner_type);

void PrintApaStateMachine(const ApaStateMachine apa_state);

const std::string GetApaStateMachine(const ApaStateMachine apa_state);

}  // namespace apa_planner
}  // namespace planning

#endif
