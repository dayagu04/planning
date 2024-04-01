#ifndef __PERPENDICULAR_PATH_PLANNER_H__
#define __PERPENDICULAR_PATH_PLANNER_H__

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "Eigen/Core"
//#include "apa_plan_base.h"
#include "collision_detection.h"
#include "dubins_lib.h"
#include "geometry_math.h"

namespace planning {
namespace apa_planner {

class PerpendicularPathPlanner {
 public:
  struct Tlane {
    Eigen::Vector2d pt_outside = Eigen::Vector2d::Zero();
    Eigen::Vector2d pt_inside = Eigen::Vector2d::Zero();
    Eigen::Vector2d pt_terminal_pos = Eigen::Vector2d::Zero();
    double pt_terminal_heading = 0.0;
    uint8_t slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;

    void Reset() {
      pt_outside = Eigen::Vector2d::Zero();
      pt_inside = Eigen::Vector2d::Zero();
      pt_terminal_pos = Eigen::Vector2d::Zero();
      pt_terminal_heading = 0.0;
      slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;
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
    bool is_replan_second = false;
    bool is_replan_dynamic = false;
    double sample_ds = 0.02;
    uint8_t ref_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
    uint8_t ref_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;
    double slot_occupied_ratio = 0.0;

    void Set(const Tlane &tlane_in,
             const pnc::geometry_lib::PathPoint &ego_pose_in,
             bool is_complete_path_in) {
      tlane = tlane_in;
      ego_pose = ego_pose_in;
      is_complete_path = is_complete_path_in;
    }
  };

  struct Output {
    bool path_available = false;
    bool is_first_path = true;
    bool is_last_path = false;
    bool gear_shift = false;
    double length = 0.0;
    uint8_t gear_change_count = 0;
    uint8_t current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
    uint8_t current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;
    std::pair<size_t, size_t> path_seg_index = std::make_pair(0, 0);
    std::vector<uint8_t> gear_cmd_vec;
    std::vector<uint8_t> steer_vec;
    std::vector<pnc::geometry_lib::PathSegment> path_segment_vec;
    std::vector<pnc::geometry_lib::PathPoint> path_point_vec;

    void Reset() {
      path_available = false;
      is_first_path = true;
      is_last_path = false;
      gear_shift = false;
      length = 0.0;
      gear_change_count = 0;
      path_seg_index = std::make_pair(0, 0);
      gear_cmd_vec.clear();
      steer_vec.clear();
      path_segment_vec.clear();
      path_point_vec.clear();
      current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
      current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;
    }
  };

  struct PlannerParams {
    bool is_left_side = true;
    double slot_side_sgn = 1.0;

    bool should_prepare_second = false;
    bool should_prepare_third = false;
    bool first_multi_plan = true;

    pnc::geometry_lib::LineSegment target_line;

    pnc::geometry_lib::Circle mono_safe_circle;
    pnc::geometry_lib::Circle multi_safe_circle;

    pnc::geometry_lib::PathPoint safe_circle_tang_pt;
    bool cal_tang_pt_success = false;
    bool directly_use_ego_pose = false;

    pnc::geometry_lib::LineSegment prepare_line;  // pA is tag point
    Eigen::Vector2d pre_line_tangent_vec = Eigen::Vector2d::Zero();
    Eigen::Vector2d pre_line_normal_vec = Eigen::Vector2d::Zero();

    void Reset() {
      is_left_side = true;
      slot_side_sgn = 1.0;

      should_prepare_second = false;
      should_prepare_third = false;
      first_multi_plan = true;

      target_line.Reset();

      mono_safe_circle.Reset();
      multi_safe_circle.Reset();

      safe_circle_tang_pt.Reset();
      cal_tang_pt_success = false;
      directly_use_ego_pose = false;

      pre_line_tangent_vec.setZero();
      pre_line_normal_vec.setZero();
    }
  };

 public:
  void Reset();
  void Preprocess();
  bool Update();
  bool Update(const std::shared_ptr<CollisionDetector> &collision_detector_ptr);
  bool UpdateByPrePlan();
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

    std::vector<Eigen::Vector2d> center_vec;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> tangent_pt_vec;
  };
  const bool CalInverseTwoArcGeometry(
      const pnc::geometry_lib::PathPoint &start_pose, const uint8_t direction,
      const uint8_t steer,
      std::vector<pnc::geometry_lib::PathSegment> &inverse_two_segmemts,
      DebugInfo &debuginfo) const;

 private:
  // member function
  // prepare plan start
  const bool PreparePlan();
  const bool PreparePlanOnce(const double x_offset,
                             const double heading_offset);

  const bool PreparePlanV2();
  const bool PreparePlanOnceV2(const double x_offset,
                               const double heading_offset);
  const bool PreparePlanAdjust(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const uint8_t current_gear);

  const bool PreparePlanSecond();

  const bool PreparePlanThird();

  const bool GenPathOutputByDubins();
  const bool MonoPreparePlan(Eigen::Vector2d &tag_point);
  void CalMonoSafeCircle();
  const bool CheckMonoIsFeasible();
  const bool MultiPreparePlan(Eigen::Vector2d &tag_point);
  bool CalMultiSafeCircle();
  // prepare plan end

  // multi plan start
  const bool CheckMultiPlanSuitable(
      const pnc::geometry_lib::PathPoint &current_pose,
      const double &slot_occupied_ratio);

  const bool MultiPlan();
  const bool CalSinglePathInMulti(
      const pnc::geometry_lib::PathPoint &current_pose,
      const uint8_t current_gear, const uint8_t current_arc_steer,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec);

  const bool OneArcPlan(
      pnc::geometry_lib::Arc &arc,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const uint8_t current_gear, const uint8_t current_arc_steer);

  const bool TwoArcPlan(
      pnc::geometry_lib::Arc &arc,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const uint8_t current_gear, const uint8_t current_arc_steer);

  const bool LineArcPlan(
      pnc::geometry_lib::Arc &arc,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const uint8_t current_gear, const uint8_t current_arc_steer);

  const bool OneLinePlan(
      pnc::geometry_lib::LineSegment &line,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const uint8_t current_gear);
  // multi plan end

  // adjust plan start
  const bool CheckAdjustPlanSuitable(
      const pnc::geometry_lib::PathPoint &current_pose,
      const double slot_occupied_ratio = 0.0);

  const bool AdjustPlan();
  const bool CalSinglePathInAdjust(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &current_pose,
      const uint8_t current_gear, const double steer_change_ratio,
      const double steer_change_radius);

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
  // adjust plan end

  // sample path start
  void SampleLineSegment(const pnc::geometry_lib::LineSegment &line_seg,
                         const double ds);

  void SampleArcSegment(const pnc::geometry_lib::Arc &cur_arc_seg,
                        const double ds);
  // sample path end

  // collision detect start
  const uint8_t TrimPathByCollisionDetection(
      pnc::geometry_lib::PathSegment &path_seg);
  // collision detect end

  const bool CheckArcOrLineAvailable(const pnc::geometry_lib::Arc &arc);
  const bool CheckArcOrLineAvailable(
      const pnc::geometry_lib::LineSegment &line);

  const bool CheckPathIsNormal(const pnc::geometry_lib::PathSegment &path_seg);

  const bool CheckReachTargetPose(
      const pnc::geometry_lib::PathPoint &current_pose);

  const bool CheckReachTargetPose();

  const double CalOccupiedRatio(
      const pnc::geometry_lib::PathPoint &current_pose);

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

  // member variable
  Input input_;
  Output output_;
  PlannerParams calc_params_;

  pnc::dubins_lib::DubinsLibrary dubins_planner_;

  std::shared_ptr<CollisionDetector> collision_detector_ptr_ = nullptr;
};

}  // namespace apa_planner
}  // namespace planning

#endif