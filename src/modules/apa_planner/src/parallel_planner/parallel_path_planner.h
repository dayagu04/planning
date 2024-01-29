#ifndef __PARALLEL_PATH_PLANNER_H__
#define __PARALLEL_PATH_PLANNER_H__

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "apa_plan_base.h"
#include "collision_detection.h"
#include "dubins_lib.h"
#include "geometry_math.h"

namespace planning {
namespace apa_planner {

class ParallelPathPlanner {
 public:
  struct Tlane {
    Eigen::Vector2d pt_outside = Eigen::Vector2d::Zero();
    Eigen::Vector2d pt_inside = Eigen::Vector2d::Zero();
    Eigen::Vector2d pt_terminal_pos = Eigen::Vector2d::Zero();
    uint8_t slot_side = ApaPlannerBase::SLOT_SIDE_INVALID;

    double channel_width = 5.5;
    double channel_length = 10.0;

    void Reset() {
      pt_outside = Eigen::Vector2d::Zero();
      pt_inside = Eigen::Vector2d::Zero();
      pt_terminal_pos = Eigen::Vector2d::Zero();
      channel_width = 5.5;
      channel_length = 10.0;
    }
  };

  enum PathColDetRes {
    PATH_COL_INVALID,
    PATH_COL_SHORTEN,
    PATH_COL_NORMAL,
    PATH_COL_COUNT,
  };

  enum class PathPlanMethod {
    INVALID,
    ONE_ARC,
    TWO_ARC,
    LINE_ARC,
    COUNT,
  };

  struct Input {
    Tlane tlane;
    pnc::geometry_lib::PathPoint ego_pose;
    bool is_complete_path = false;
    bool is_replan_first = true;
    double sample_ds = 0.02;
    uint8_t ref_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
    uint8_t ref_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;
    double slot_occupied_ratio = 0.0;

    void Set(const Tlane &tlane_r,
             const std::vector<Eigen::Vector2d> &obstacle_vec_r,
             const pnc::geometry_lib::PathPoint &ego_pose_r,
             bool is_complete_path_r) {
      tlane = tlane_r;
      ego_pose = ego_pose_r;
      is_complete_path = is_complete_path_r;
    }
  };

  struct Output {
    bool path_available = false;
    bool is_first_path = true;
    bool is_last_path = false;
    double length = 0.0;
    uint8_t gear_change_count = 0;
    uint8_t current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
    uint8_t current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;

    std::vector<uint8_t> steer_vec;
    std::vector<uint8_t> gear_cmd_vec;
    std::vector<pnc::geometry_lib::PathPoint> path_point_vec;
    std::vector<pnc::geometry_lib::PathSegment> path_segment_vec;
    std::pair<size_t, size_t> path_seg_index = std::make_pair(0, 0);
    void Reset() {
      path_available = false;
      is_first_path = true;
      is_last_path = false;
      length = 0.0;
      gear_change_count = 0;

      gear_cmd_vec.clear();
      steer_vec.clear();
      path_segment_vec.clear();
      path_point_vec.clear();
      current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
      current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;
      path_seg_index = std::make_pair(0, 0);
    }
  };

  struct PlannerParams {
    bool is_left_side = false;
    double slot_side_sgn = 1.0;  // 1.0 in right slot
    pnc::geometry_lib::PathPoint safe_circle_target_pose;
    pnc::geometry_lib::Circle safe_circle;
    pnc::geometry_lib::LineSegment target_line;
    pnc::geometry_lib::LineSegment prepare_line;  // pA is tag point
    Eigen::Vector2d v_target_line = Eigen::Vector2d::Zero();
    Eigen::Vector2d v_prepare_line = Eigen::Vector2d::Zero();

    void Reset() {
      is_left_side = true;
      slot_side_sgn = 1.0;
      safe_circle_target_pose.Reset();
      safe_circle.Reset();
      target_line.Reset();
      prepare_line.Reset();
      v_target_line.setZero();
      v_prepare_line.setZero();
    }
  };

 public:
  void Reset();
  void Preprocess();
  const bool Update();
  const bool Update(
      const std::shared_ptr<CollisionDetector> &collision_detector_ptr);

  const bool SetCurrentPathSegIndex();
  void SetLineSegmentHeading();
  void ExtendCurrentFollowLastPath(double extend_distance);
  void InsertLineSegAfterCurrentFollowLastPath(double extend_distance);
  const bool SampleCurrentPathSeg();

  void PrintOutputSegmentsInfo() const;
  void PrintSegmentInfo(const pnc::geometry_lib::PathSegment &seg) const;

  void SetInput(const Input &input) { input_ = input; }
  const Output &GetOutput() const { return output_; }
  const Output *GetOutputPtr() const { return &output_; }

  // pybind simulation
  const std::vector<double> GetPathEle(size_t index) const;
  const std::vector<double> GetMinSafeCircle() const;

  // for debug
  struct DebugInfo {
    Eigen::Vector2d tag_point;
    double headingB = 0.0;
  };
  const bool CalInverseTwoArcGeometry(
      const pnc::geometry_lib::PathPoint &start_pose, const uint8_t direction,
      const uint8_t steer,
      std::vector<pnc::geometry_lib::PathSegment> &inverse_two_segmemts,
      DebugInfo &debuginfo) const;

 private:
  // prepare plan start
  const bool PreparePlan();
  const bool PreparePlanOnce(const double x_coord, const double y_coord);
  const bool GenPathOutputByDubins();
  // normal plan
  const bool ProcessPlan();
  const bool CalMinSafeCircle();
  const bool CheckParkOutWithCollisionFree(
      const pnc::geometry_lib::PathPoint &current_pose,
      const double radius) const;

  const bool TripleStepPlan(const pnc::geometry_lib::PathPoint &current_pose,
                            const pnc::geometry_lib::PathPoint &target_pose,
                            const double radius);

  // multi plan
  const bool MultiPlan();
  // adjust plan
  const bool AdjustPlan();
  const bool OneArcPlan(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &current_pose,
      const uint8_t current_gear);

  const bool AlignBodyPlan(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &current_pose,
      const uint8_t current_gear);

  const bool STurnParallelPlan(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &current_pose,
      const uint8_t current_gear, const double steer_change_ratio1,
      const double steer_change_radius);

  const bool CalSinglePathInAdjust(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &current_pose,
      const uint8_t current_gear, const double steer_change_ratio,
      const double steer_change_radius);

  const uint8_t TrimPathByCollisionDetection(
      pnc::geometry_lib::PathSegment &path_seg);

  const bool OneLinePlan(
      pnc::geometry_lib::LineSegment &line,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const uint8_t current_gear);

  const bool ArcLineArcDubinsPlan(
      const pnc::geometry_lib::PathPoint &start_pose,
      const pnc::geometry_lib::PathPoint &target_pose,
      const double radius);  // LSR for left slot , RSL for right slot

  const bool LineArcLinePlan(const pnc::geometry_lib::PathPoint &start_pose,
                             const uint8_t direction, const uint8_t steer,
                             const double radius,
                             std::vector<pnc::geometry_lib::PathSegment>
                                 &line_arc_line_segments) const;

  void SampleLineSegment(const pnc::geometry_lib::LineSegment &line_seg,
                         const double ds);

  void SampleArcSegment(const pnc::geometry_lib::Arc &cur_arc_seg,
                        const double ds);

  const Eigen::Vector2d CalEgoTurningCenter(
      const pnc::geometry_lib::PathPoint &ego_pose, const double radius,
      const uint8_t steer) const;

  const bool CheckTwoPoseInCircle(const Eigen::Vector2d &ego_pos0,
                                  const double ego_heading0,
                                  const Eigen::Vector2d &ego_pos1,
                                  const double ego_heading1,
                                  const Eigen::Vector2d &center) const;

  const bool IsRightCircle(const pnc::geometry_lib::PathPoint &ego_pose,
                           const Eigen::Vector2d &center) const;

  const bool IsRightCircle(const Eigen::Vector2d &ego_pos,
                           const double ego_heading,
                           const Eigen::Vector2d &center) const;

  const bool IsLocatedOnTarget(
      const pnc::geometry_lib::PathPoint &current_pose) const;

  Input input_;
  Output output_;
  PlannerParams calc_params_;
  pnc::dubins_lib::DubinsLibrary dubins_planner_;
  std::shared_ptr<CollisionDetector> collision_detector_ptr_ = nullptr;
};

}  // namespace apa_planner
}  // namespace planning

#endif