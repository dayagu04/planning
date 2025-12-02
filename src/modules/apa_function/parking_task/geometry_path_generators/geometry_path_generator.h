#ifndef __APA_PATH_PLANNER_H__
#define __APA_PATH_PLANNER_H__

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "apa_slot.h"
#include "apa_slot_manager.h"
#include "collision_detection/collision_detection.h"
#include "dubins_lib.h"
#include "geometry_math.h"
#include "parking_task.h"

namespace planning {
namespace apa_planner {

// only use for parallel parking, If parallel parking is unified behind, Tlane
// can be deleted

struct TlaneCorner {
  Eigen::Vector2d A = Eigen::Vector2d::Zero();
  Eigen::Vector2d B = Eigen::Vector2d::Zero();
  Eigen::Vector2d C = Eigen::Vector2d::Zero();
  Eigen::Vector2d D = Eigen::Vector2d::Zero();
  Eigen::Vector2d E = Eigen::Vector2d::Zero();
  Eigen::Vector2d F = Eigen::Vector2d::Zero();
  Eigen::Vector2d channel_point_1 = Eigen::Vector2d::Zero();
  Eigen::Vector2d channel_point_2 = Eigen::Vector2d::Zero();

  TlaneCorner() = default;

  TlaneCorner(const Eigen::Vector2d& A_, const Eigen::Vector2d& B_,
              const Eigen::Vector2d& C_, const Eigen::Vector2d& D_,
              const Eigen::Vector2d& E_, const Eigen::Vector2d& F_,
              const Eigen::Vector2d& channel_point_1_,
              const Eigen::Vector2d& channel_point_2_)
      : A(A_),
        B(B_),
        C(C_),
        D(D_),
        E(E_),
        F(F_),
        channel_point_1(channel_point_1_),
        channel_point_2(channel_point_2_) {}

  void Reset() {
    A.setZero();
    B.setZero();
    C.setZero();
    D.setZero();
    E.setZero();
    F.setZero();
    channel_point_1.setZero();
    channel_point_2.setZero();
  }
};

struct Tlane {
  Eigen::Vector2d corner_outside_slot = Eigen::Vector2d::Zero();
  Eigen::Vector2d corner_inside_slot = Eigen::Vector2d::Zero();
  Eigen::Vector2d pt_outside = Eigen::Vector2d::Zero();
  Eigen::Vector2d pt_inside = Eigen::Vector2d::Zero();
  Eigen::Vector2d pt_terminal_pos = Eigen::Vector2d::Zero();
  Eigen::Vector2d pt_lower_boundry_pos = Eigen::Vector2d::Zero();
  TlaneCorner tlane_corner;

  Limiter limiter;
  bool is_inside_rigid = false;
  double pt_terminal_heading = 0.0;
  uint8_t slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;

  // obs tlane
  Eigen::Vector2d obs_pt_inside = Eigen::Vector2d::Zero();
  Eigen::Vector2d obs_pt_outside = Eigen::Vector2d::Zero();
  double curb_y = 2.6;

  double slot_side_sgn = 1.0;

  double slot_width = 2.4;
  double slot_length = 6.0;

  double channel_y = 6.5;
  double channel_x_limit = 16.6;
  bool is_short_channel = false;

  double car_to_curb_dis = 0.0;
  double first_occupied_ratio = 0.0;
  bool use_sturn_plan = false;

  void Reset() {
    corner_outside_slot.setZero();
    corner_inside_slot.setZero();
    pt_outside.setZero();
    pt_inside.setZero();
    pt_terminal_pos.setZero();
    pt_lower_boundry_pos.setZero();
    tlane_corner.Reset();
    pt_terminal_heading = 0.0;
    slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;

    obs_pt_inside.setZero();
    obs_pt_outside.setZero();

    curb_y = 2.6;
    slot_width = 2.4;
    slot_length = 6.0;
    channel_y = 6.5;
    channel_x_limit = 16.6;
    slot_side_sgn = 1.0;
    limiter.Reset();
    is_inside_rigid = false;
    is_short_channel = false;
    car_to_curb_dis = 0.0;
    first_occupied_ratio = 0.0;
    use_sturn_plan = false;
  }
};

// 如果某个模块有定制变量  放在这个Input里 不要放在EgoInfoUnderSlot里
struct GeometryPathInput {
  EgoInfoUnderSlot ego_info_under_slot;
  bool is_complete_path = false;
  bool is_replan_first = true;
  bool is_replan_second = false;
  bool is_replan_dynamic = false;
  int force_mid_process_plan = 0;
  bool can_first_plan_again = true;
  double sample_ds = 0.02;
  uint8_t ref_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
  uint8_t ref_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;
  bool is_searching_stage = false;

  uint8_t path_planner_state = 0;

  bool is_left_empty = false;
  bool is_right_empty = false;

  bool is_simulation = false;

  bool enable_smart_fold_mirror = false;
  int parallel_replan_again_ = 0;

  Tlane tlane;
};

struct GeometryPathOutput {
  bool path_available = false;
  bool is_first_path = true;
  bool is_last_path = false;
  bool gear_shift = false;
  bool multi_reach_target_pose = false;
  bool linearc_reach_target_pose = false;
  double length = 0.0;
  uint8_t gear_change_count = 0;
  uint8_t current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
  uint8_t current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;
  std::pair<size_t, size_t> path_seg_index = std::make_pair(0, 0);
  std::vector<uint8_t> gear_cmd_vec;
  std::vector<uint8_t> steer_vec;
  std::vector<pnc::geometry_lib::PathSegment> path_segment_vec;
  std::vector<pnc::geometry_lib::PathPoint> path_point_vec;
  std::vector<pnc::geometry_lib::PathPoint> all_gear_path_point_vec;

  std::vector<geometry_lib::GeometryPath> perferred_geometry_path_vec;
  geometry_lib::GeometryPath perferred_geometry_path;

  double actual_ds = 0.0;
  double cur_gear_length = 0.0;
  bool is_need_optimizer = false;

  void Reset() {
    path_available = false;
    is_first_path = true;
    is_last_path = false;
    gear_shift = false;
    multi_reach_target_pose = false;
    linearc_reach_target_pose = false;
    length = 0.0;
    gear_change_count = 0;
    path_seg_index = std::make_pair(0, 0);
    gear_cmd_vec.clear();
    steer_vec.clear();
    path_segment_vec.clear();
    path_point_vec.clear();
    all_gear_path_point_vec.clear();
    current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
    current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;

    actual_ds = 0.0;
    cur_gear_length = 0.0;
  }
};

class GeometryPathGenerator : public ParkingTask {
 public:
  virtual void Reset() = 0;
  virtual const bool Update() = 0;

  virtual const bool Update(
      const std::shared_ptr<CollisionDetector> &collision_detector_ptr);
  virtual const bool SetCurrentPathSegIndex();
  virtual const bool CheckCurrentGearLength();
  virtual const bool SampleCurrentPathSeg();

  virtual void PrintOutputSegmentsInfo() const;

  void SetInput(const GeometryPathInput &input) { input_ = input; }
  void SetColPtr(
      const std::shared_ptr<CollisionDetector> &collision_detector_ptr) {
    collision_detector_ptr_ = collision_detector_ptr;
  }
  const GeometryPathOutput &GetOutput() const { return output_; }
  GeometryPathOutput *GetOutputPtr() { return &output_; }
  const std::vector<double> GetPathEle(size_t index) const;

 protected:
  virtual void Preprocess() = 0;

  GeometryPathInput input_;
  GeometryPathOutput output_;
  pnc::dubins_lib::DubinsLibrary dubins_planner_;
  std::shared_ptr<CollisionDetector> collision_detector_ptr_ = nullptr;
};
}  // namespace apa_planner
}  // namespace planning

#endif