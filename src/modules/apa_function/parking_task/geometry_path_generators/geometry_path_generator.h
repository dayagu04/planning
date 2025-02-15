#ifndef __APA_PATH_PLANNER_H__
#define __APA_PATH_PLANNER_H__

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "apa_slot_manager.h"
#include "collision_detection/collision_detection.h"
#include "dubins_lib.h"
#include "geometry_math.h"
#include "parking_task.h"

namespace planning {
namespace apa_planner {

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
  // 后续等所有模块适配结束之后， 把下面的Tlane和Input删除
  struct Tlane {
    Eigen::Vector2d corner_outside_slot = Eigen::Vector2d::Zero();
    Eigen::Vector2d corner_inside_slot = Eigen::Vector2d::Zero();
    Eigen::Vector2d pt_outside = Eigen::Vector2d::Zero();
    Eigen::Vector2d pt_inside = Eigen::Vector2d::Zero();
    Eigen::Vector2d pt_terminal_pos = Eigen::Vector2d::Zero();
    Eigen::Vector2d pt_lower_boundry_pos = Eigen::Vector2d::Zero();
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

    void Reset() {
      corner_outside_slot.setZero();
      corner_inside_slot.setZero();
      pt_outside.setZero();
      pt_inside.setZero();
      pt_terminal_pos.setZero();
      pt_lower_boundry_pos.setZero();
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
    }
  };

  struct Input {
    Tlane tlane;
    pnc::geometry_lib::PathPoint ego_pose;
    bool is_complete_path = false;
    bool is_replan_first = true;
    bool is_replan_second = false;
    bool is_replan_dynamic = false;
    double sample_ds = 0.02;
    uint8_t ref_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
    uint8_t ref_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;
    double slot_occupied_ratio = 0.0;
    double origin_pt_0_heading = 0.0;
    double sin_angle = 1.0;
    Eigen::Vector2d pt_0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d pt_1 = Eigen::Vector2d::Zero();
    double channel_width = 0.0;
    bool is_left_empty = false;
    bool is_right_empty = false;

    pnc::geometry_lib::GlobalToLocalTf global2slot_tf;
    pnc::geometry_lib::LocalToGlobalTf slot2global_tf;

    uint8_t path_planner_state = 0;

    bool is_searching_stage = false;

    void Set(const Tlane &tlane_in,
             const pnc::geometry_lib::PathPoint &ego_pose_in,
             bool is_complete_path_in) {
      tlane = tlane_in;
      ego_pose = ego_pose_in;
      is_complete_path = is_complete_path_in;
    }
  };

 public:
  virtual void Reset() = 0;
  virtual const bool Update() = 0;

  virtual const bool Update(
      const std::shared_ptr<CollisionDetector> &collision_detector_ptr);
  virtual const bool SetCurrentPathSegIndex();
  virtual const bool CheckCurrentGearLength();
  virtual const bool SampleCurrentPathSeg();

  virtual void PrintOutputSegmentsInfo() const;

  void SetInput(const Input &input) { input_ = input; }
  void SetGInput(const GeometryPathInput &ginput) { ginput_ = ginput; }
  void SetColPtr(
      const std::shared_ptr<CollisionDetector> &collision_detector_ptr) {
    collision_detector_ptr_ = collision_detector_ptr;
  }
  const GeometryPathOutput &GetOutput() const { return output_; }
  GeometryPathOutput *GetOutputPtr() { return &output_; }
  const std::vector<double> GetPathEle(size_t index) const;

 protected:
  virtual void Preprocess() = 0;

  Input input_;
  GeometryPathInput ginput_;
  GeometryPathOutput output_;
  pnc::dubins_lib::DubinsLibrary dubins_planner_;
  std::shared_ptr<CollisionDetector> collision_detector_ptr_ = nullptr;
};
}  // namespace apa_planner
}  // namespace planning

#endif