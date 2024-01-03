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
    Eigen::Vector2d p0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d p1 = Eigen::Vector2d::Zero();
    Eigen::Vector2d pt = Eigen::Vector2d::Zero();
    double channel_x = 0.0;
    uint8_t slot_side = ApaPlannerBase::SLOT_SIDE_INVALID;

    void Reset() {
      p0 = Eigen::Vector2d::Zero();
      p1 = Eigen::Vector2d::Zero();
      pt = Eigen::Vector2d::Zero();
      channel_x = 0.0;
    }
  };

  struct PathSegment {
    uint8_t seg_type = pnc::geometry_lib::SEG_TYPE_LINE;
    uint8_t seg_steer = pnc::geometry_lib::SEG_STEER_STRAIGHT;
    uint8_t seg_direction = pnc::geometry_lib::SEG_GEAR_DRIVE;

    pnc::geometry_lib::LineSegment line_seg;
    pnc::geometry_lib::Arc arc_seg;

    PathSegment() = default;

    const double Getlength() const {
      if (seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
        return line_seg.length;
      } else if (seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
        return arc_seg.length;
      } else {
        return 0.0;
      }
    }

    PathSegment(const uint8_t seg_type_r, const uint8_t seg_steer_r,
                const uint8_t seg_direction_r,
                const pnc::geometry_lib::LineSegment &line_seg_r,
                const pnc::geometry_lib::Arc &arc_seg_r) {
      seg_type = seg_type_r;
      seg_steer = seg_steer_r;
      seg_direction = seg_direction_r;
      line_seg = line_seg_r;
      arc_seg = arc_seg_r;
    }

    // construct line segment
    PathSegment(const uint8_t seg_direction_r,
                const pnc::geometry_lib::LineSegment &line_seg_r) {
      seg_type = pnc::geometry_lib::SEG_TYPE_LINE;

      seg_steer = pnc::geometry_lib::SEG_STEER_STRAIGHT;
      seg_direction = seg_direction_r;
      line_seg = line_seg_r;
    }

    // construct arc segment
    PathSegment(const uint8_t seg_steer_r, const uint8_t seg_direction_r,
                const pnc::geometry_lib::Arc &arc_seg_r) {
      seg_type = pnc::geometry_lib::SEG_TYPE_ARC;
      seg_steer = seg_steer_r;
      seg_direction = seg_direction_r;
      arc_seg = arc_seg_r;
    }

    const pnc::geometry_lib::LineSegment &GetLineSeg() const {
      return line_seg;
    }
    const pnc::geometry_lib::Arc &GetArcSeg() const { return arc_seg; }
  };

  struct Input {
    Tlane tlane;
    std::vector<Eigen::Vector2d> obstacle_vec;
    pnc::geometry_lib::PathPoint ego_pose;
    bool is_complete_path = false;
    bool is_replan_first = true;
    double sample_ds = 0.02;
    uint8_t ref_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
    uint8_t ref_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;

    void Set(const Tlane &tlane_r,
             const std::vector<Eigen::Vector2d> &obstacle_vec_r,
             const pnc::geometry_lib::PathPoint &ego_pose_r,
             bool is_complete_path_r) {
      tlane = tlane_r;
      obstacle_vec = obstacle_vec_r;
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
    uint8_t current_gear = 0;
    uint8_t current_arc_steer = 0;
    bool is_prepare = false;
    bool use_line_step = false;
    std::pair<size_t, size_t> path_seg_index = std::make_pair(0, 0);
    std::vector<uint8_t> gear_cmd_vec;
    std::vector<uint8_t> steer_vec;
    std::vector<PathSegment> path_segment_vec;
    std::vector<pnc::geometry_lib::PathPoint> path_point_vec;

    void Reset() {
      path_available = false;
      is_first_path = true;
      is_last_path = false;
      length = 0.0;
      gear_change_count = 0;
      path_seg_index = std::make_pair(0, 0);
      gear_cmd_vec.clear();
      steer_vec.clear();
      path_segment_vec.clear();
      path_point_vec.clear();
      current_gear = 0;
      is_prepare = false;
      use_line_step = false;
    }
  };

  struct PlannerParams {
    bool is_right_side = true;
    double slot_side_sgn = 1.0;
    double dist_f_corner_to_rac = 0.0;
    double dist_r_corner_to_rac = 0.0;
    double min_fo_radius = 0.0;
    double min_fi_radius = 0.0;
    double target_heading = 0.0;

    pnc::geometry_lib::LineSegment target_line;

    Eigen::Vector2d fr_corner_vec = Eigen::Vector2d::Zero();
    Eigen::Vector2d fl_corner_vec = Eigen::Vector2d::Zero();
    Eigen::Vector2d rl_corner_vec = Eigen::Vector2d::Zero();
    Eigen::Vector2d rr_corner_vec = Eigen::Vector2d::Zero();

    Eigen::Vector2d safe_circle_key_pt = Eigen::Vector2d::Zero();
    double safe_circle_key_heading = 0.0;

    pnc::geometry_lib::Circle mono_safe_circle;
    pnc::geometry_lib::Circle multi_safe_circle;

    pnc::geometry_lib::LineSegment prepare_line;  // pA is tag point
    Eigen::Vector2d pre_line_tangent_vec = Eigen::Vector2d::Zero();
    Eigen::Vector2d pre_line_normal_vec = Eigen::Vector2d::Zero();

    void Reset() {
      is_right_side = true;
      slot_side_sgn = 1.0;
      dist_f_corner_to_rac = 0.0;
      dist_r_corner_to_rac = 0.0;
      min_fo_radius = 0.0;
      min_fi_radius = 0.0;
      target_heading = 0.0;

      fr_corner_vec = Eigen::Vector2d::Zero();
      fl_corner_vec = Eigen::Vector2d::Zero();
      rl_corner_vec = Eigen::Vector2d::Zero();
      rr_corner_vec = Eigen::Vector2d::Zero();

      safe_circle_key_heading = 0.0;
      safe_circle_key_pt = Eigen::Vector2d::Zero();

      mono_safe_circle.center.setZero();
      mono_safe_circle.radius = 0.0;
      multi_safe_circle.center.setZero();
      multi_safe_circle.radius = 0.0;

      pre_line_tangent_vec.setZero();
      pre_line_normal_vec.setZero();
    }
  };

 public:
  void Reset();
  void Preprocess();
  bool Update();
  const bool SetCurrentPathSegIndex();
  void SetLineSegmentHeading();
  void ExtendCurrentFollowLastPath(double extend_distance);
  void InsertLineSegAfterCurrentFollowLastPath(double extend_distance);
  const bool SampleCurrentPathSeg();

  void PrintOutputSegmentsInfo() const;
  void PrintSegmentInfo(const PathSegment &seg) const;

  void SetCollisionDetectorPtr(
      std::shared_ptr<CollisionDetector> collision_detector_ptr) {
    collision_detector_ptr_ = collision_detector_ptr;
  }

  // input
  void SetInput(const Input &input) { input_ = input; }
  void SetTlane(const Tlane &tlane) { input_.tlane = tlane; }
  void SetObstacle(const std::vector<Eigen::Vector2d> &obstacle_vec) {
    input_.obstacle_vec = obstacle_vec;
  }
  void SetEgoPose(const pnc::geometry_lib::PathPoint &ego_pose) {
    input_.ego_pose = ego_pose;
  }
  void SetIsCompletePath(bool is_complete_path) {
    input_.is_complete_path = is_complete_path;
  }
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
      const uint8_t steer, std::vector<PathSegment> &inverse_two_segmemts,
      DebugInfo &debuginfo) const;

 private:
  // adjust step plan
  const bool AdjustStepPlanOnce(const double shift_scale,
                                const double min_shift_radius);

  const bool AdjustStepPlan();

  const uint8_t CalArcGear(const pnc::geometry_lib::Arc &arc) const;
  const uint8_t CalArcSteer(const pnc::geometry_lib::Arc &arc) const;
  // multi-step plan
  const bool MultiStepPlan();

  const bool CalOnePathInMultiStep(
      const pnc::geometry_lib::PathPoint &current_pose, const uint8_t direction,
      const uint8_t steer, std::vector<PathSegment> &segments);

  const bool LineArcLinePlan(
      const pnc::geometry_lib::PathPoint &start_pose, const uint8_t direction,
      const uint8_t steer, const double radius,
      std::vector<PathSegment> &line_arc_line_segments) const;

  const bool GetCorrectCircleIndexofLineArcLine(
      const double start_heading, const double target_heading,
      const uint8_t steer, const std::vector<Eigen::Vector2d> &possible_centers,
      const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>
          &possible_tangent_pts,
      size_t &circle_index) const;

  const bool SelectLineArcLineByDir(
      const pnc::geometry_lib::PathPoint &start_pose,
      const pnc::geometry_lib::PathPoint &target_pose,
      const std::pair<Eigen::Vector2d, Eigen::Vector2d> &tangent_pts,
      const pnc::geometry_lib::Circle &circle, const uint8_t ideal_dir,
      std::vector<PathSegment> &line_arc_line_segments) const;

  const bool InverseTwoArcPlan(const pnc::geometry_lib::PathPoint &current_pose,
                               const uint8_t direction, const uint8_t steer,
                               const double radius,
                               PathSegment &opt_arc_segment);

  const bool CalInverseTwoArcGeometry(
      const pnc::geometry_lib::PathPoint &start_pose, const uint8_t direction,
      const uint8_t steer,
      std::vector<PathSegment> &inverse_two_segmemts) const;

  const bool IsSeenAsLine(
      const pnc::geometry_lib::PathPoint &current_pose,
      const pnc::geometry_lib::PathPoint &target_pose) const;

  const bool OneLineGeometryPlan(
      const pnc::geometry_lib::PathPoint &current_pose,
      const pnc::geometry_lib::LineSegment &target_line,
      PathSegment &line_seg) const;

  const bool OneArcGeometryPlan(
      const pnc::geometry_lib::PathPoint &current_pose,
      const pnc::geometry_lib::LineSegment &target_line, const double radius,
      const uint8_t steer, PathSegment &arc_segment) const;

  const bool UpdateMultiStepMinSafeCircle();

  const bool CheckOneLineStepInSlot();
  const bool MonoStepPlan();
  const bool CheckMonoStepInSlot();
  void CalcMonoStepMinSafetyCircle();

  void GenPrepareTargetLine(pnc::geometry_lib::LineSegment &line_seg,
                            const double delta_heading,
                            const Eigen::Vector2d &pos);

  const bool PreparePlanOnce(const double y_offset,
                             const double heading_offset);

  const bool PreparePlan();
  const bool GenPathOutputByDubins();
  const bool MonoPreparePlan(Eigen::Vector2d &tag_point);
  const bool MultiPreparePlan(Eigen::Vector2d &tag_point);
  void CalMonoSafeCircle();
  const bool CheckMonoIsFeasible();
  bool CalMultiSafeCircle();

  const bool CheckArcCollided(const pnc::geometry_lib::Arc &arc_step) const;

  const bool CheckTwoPoseInCircle(const Eigen::Vector2d &ego_pos0,
                                  const double ego_heading0,
                                  const Eigen::Vector2d &ego_pos1,
                                  const double ego_heading1,
                                  const Eigen::Vector2d &center) const;

  void SampleLineSegment(const pnc::geometry_lib::LineSegment &line_seg,
                         const double ds);

  void SampleArcSegment(const pnc::geometry_lib::Arc &cur_arc_seg,
                        const double ds);

  const Eigen::Vector2d CalEgoTurningCenter(const Eigen::Vector2d &ego_pos,
                                            const double ego_heading,
                                            const double radius,
                                            const uint8_t steer) const;

  const bool IsRightCircle(const pnc::geometry_lib::PathPoint &ego_pose,
                           const Eigen::Vector2d &center) const;

  const bool IsRightCircle(const Eigen::Vector2d &ego_pos,
                           const double ego_heading,
                           const Eigen::Vector2d &center) const;

  pnc::geometry_lib::LineSegment ConstructEgoHeadingLine(
      const Eigen::Vector2d &ego_pos, const double ego_heading) const;

  Input input_;
  Output output_;
  PlannerParams calc_params_;

  std::shared_ptr<CollisionDetector> collision_detector_ptr_;
  pnc::dubins_lib::DubinsLibrary dubins_planner_;
};

}  // namespace apa_planner
}  // namespace planning

#endif