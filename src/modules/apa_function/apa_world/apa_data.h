#ifndef __APA_DATA_H__
#define __APA_DATA_H__

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>
#include "Eigen/Core"
#include "apa_obstacle.h"
#include "apa_slot.h"
#include "geometry_math.h"
#include "local_view.h"

namespace planning {
namespace apa_planner {

enum class ParkingScenarioType {
  SCENARIO_UNKNOWN = 0,
  SCENARIO_PERPENDICULAR_TAIL_IN = 1,
  SCENARIO_PERPENDICULAR_HEAD_IN = 2,
  SCENARIO_PERPENDICULAR_TAIL_OUT = 3,
  SCENARIO_PERPENDICULAR_HEAD_OUT = 4,
  SCENARIO_PARALLEL_IN = 5,
  SCENARIO_PARALLEL_OUT = 6,
  SCENARIO_SLANT_TAIL_IN = 7,
  SCENARIO_SLANT_HEAD_IN = 8,
  SCENARIO_SLANT_TAIL_OUT = 9,
  SCENARIO_SLANT_HEAD_OUT = 10,
  // todo, remove narrow space scenario
  SCENARIO_NARROW_SPACE = 11,
};

struct TLane {
  Eigen::Vector2d A;
  Eigen::Vector2d B;
  Eigen::Vector2d C;
  Eigen::Vector2d D;
  Eigen::Vector2d E;
  Eigen::Vector2d F;
  Eigen::Vector2d G;
  Eigen::Vector2d H;

  void Reset() {
    A.setZero();
    B.setZero();
    C.setZero();
    D.setZero();
    E.setZero();
    F.setZero();
    G.setZero();
    H.setZero();
  }
};

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

  void CalCenter() { pt_center = 0.25 * (pt_1 + pt_0 + pt_2 + pt_3); }
};

enum class SlotType : uint8_t {
  PERPENDICULAR,
  PARALLEL,
  SLANT,
  COUNT,
  INVALID,
};

// enum class SlotSide : uint8_t {
//   LEFT,
//   RIGHT,
//   COUNT,
//   INVALID,
// };

enum class SlotSourceType : uint8_t {
  CAMERA,
  USS,
  CAMERA_USS,
  INVALID,
};

enum class SlotOccupiedReason : uint8_t {
  FUSION,
  TWO_SIDE_OBS,
  CHANNEL_OBS,
  SLOT_IN_OBS,
  PREPARE_PLAN,
  INVALID,
};

enum class PlanMethod : uint8_t {
  GEOMETRY,
  ASTAR,
  INVALID,
};

enum class ObstacleType {
  FUSION,
  GROUND_LINE,
  USS,
  VIRTUAL,
  COUNT,
  INVALID,
};

// 只存储与车位相关的信息 自车信息以及误差或者其他信息不要存储在该结构体里!!!
struct ApaSlot {
  SlotCoord origin_corner_coord_global;
  SlotCoord processed_corner_coord_global;
  SlotCoord origin_corner_coord_local;
  SlotCoord processed_corner_coord_local;
  SlotType slot_type = SlotType::INVALID;
  SlotSourceType slot_source_type = SlotSourceType::INVALID;
  SlotOccupiedReason slot_occupied_reason = SlotOccupiedReason::INVALID;
  PlanMethod plan_method = PlanMethod::INVALID;
  bool is_release = false;
  size_t slot_id = 0;
  Limiter limiter;

  // 包含了车位坐标原点的位姿和航向向量
  pnc::geometry_lib::PathPoint origin_pose_global;
  pnc::geometry_lib::PathPoint origin_pose_local;

  pnc::geometry_lib::GlobalToLocalTf g2l_tf;
  pnc::geometry_lib::LocalToGlobalTf l2g_tf;

  double angle = 90.0;  // 23角点中点与01角点中点连线和01角点连线的锐角或直角
                        // 一般为90 60 45 垂直车位即90度
  double sin_angle = 1.0;

  double slot_length = 0.0;
  double slot_width = 0.0;

  TLane obs_tlane;

  void Reset() {
    origin_corner_coord_global.Reset();
    processed_corner_coord_global.Reset();
    origin_corner_coord_local.Reset();
    processed_corner_coord_local.Reset();
    slot_type = SlotType::INVALID;
    slot_source_type = SlotSourceType::INVALID;
    slot_occupied_reason = SlotOccupiedReason::INVALID;
    plan_method = PlanMethod::INVALID;
    is_release = false;
    slot_id = 0;
    limiter.Reset();
    origin_pose_global.Reset();
    origin_pose_local.Reset();
    l2g_tf.Reset();
    g2l_tf.Reset();
    angle = 90.0;
    sin_angle = 1.0;
    slot_length = 0.0;
    slot_width = 0.0;

    obs_tlane.Reset();
  }
};

// 包含选中车位的信息 自车在车位坐标系下的信息 误差信息 终点信息
// 障碍物坐标信息等
struct EgoCarInfoUnderSlot {
  pnc::geometry_lib::PathPoint cur_pose;
  pnc::geometry_lib::PathPoint target_pose;

  pnc::geometry_lib::PathPoint terminal_err;

  double slot_occupied_ratio = 0.0;

  // 车位相对于自车车头的方向
  uint8_t slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;

  // 根据自车位置决定
  double channel_width = 0.0;

  std::unordered_map<ObstacleType, std::vector<Eigen::Vector2d>> obs_map;

  ApaSlot slot;

  // 根据障碍物移动车位 对于垂直车位 向左为正 对于平行车位 向上为正
  double move_slot_dist = 0.0;
  double last_move_slot_dist = 0.0;

  Eigen::Vector2d pt_inside;

  void Reset() {
    cur_pose.Reset();
    target_pose.Reset();
    terminal_err.Reset();
    slot_occupied_ratio = 0.0;
    slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;
    channel_width = 0.0;
    slot.Reset();
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

struct CarPredictTraj {
  std::vector<pnc::geometry_lib::PathPoint> car_predict_pt_vec;

  void Reset() { car_predict_pt_vec.clear(); }
};

struct ApaData {
  // 这些指针只准在apa_world里的preprocess使用 其他代码后续应当不能访问
  const iflyauto::FuncStateMachine* func_state_ptr;
  const iflyauto::ParkingFusionInfo* parking_slot_ptr;
  const iflyauto::IFLYLocalization* localization_ptr;
  const iflyauto::VehicleServiceOutputInfo* vehicle_service_info_ptr;
  const iflyauto::UssWaveInfo* uss_wave_info_ptr;
  const iflyauto::UssPerceptInfo* uss_percept_info_ptr;
  const iflyauto::GroundLinePerceptionInfo* ground_line_perception_info_ptr;
  const iflyauto::FusionObjectsInfo* fusion_objects_info_ptr;
  const iflyauto::FusionOccupancyObjectsInfo* fusion_occupancy_objects_info_ptr;
  const iflyauto::ControlOutput* control_output_ptr;
  const iflyauto::PlanningOutput* plan_output_ptr;

  CarPredictTraj car_predict_traj;

  MeasurementData measurement_data;
  ApaSlots apa_slots;
  UssDistance uss_dis;

  // 暂时用这个 无需改变太多之前代码
  std::unordered_map<ObstacleType, std::vector<Eigen::Vector2d>> apa_obs_map;

  // 后面需要用这个
  // std::unordered_map<ObstacleType, std::vector<ApaObstacle>> apa_obs_map;

  SimulationParam simu_param;

  // If not select any slot, this is null.
  uint8_t slot_type = Common::PARKING_SLOT_TYPE_INVALID;
  uint8_t slot_id = 0;
  bool is_slot_type_fixed = false;

  const LocalView* local_view_ptr_ = nullptr;

  void Reset() {
    car_predict_traj.Reset();
    measurement_data.Reset();
    apa_slots.Reset();
    uss_dis.Reset();
    apa_obs_map.clear();

    slot_type = Common::PARKING_SLOT_TYPE_INVALID;
    slot_id = 0;
    is_slot_type_fixed = false;
  }
};

void PrintApaPlannerType(const ApaPlannerType planner_type);

const std::string GetApaPlannerTypeString(const ApaPlannerType planner_type);

}  // namespace apa_planner
}  // namespace planning

#endif
