#ifndef __PARALLEL_PATH_PLANNER_H__
#define __PARALLEL_PATH_PLANNER_H__

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/src/Core/Matrix.h"
#include "apa_plan_base.h"
#include "collision_detection.h"
#include "dubins_lib.h"
#include "geometry_math.h"

namespace planning {
namespace apa_planner {

class ParallelPathPlanner {
 public:
  struct Tlane {
    // tlane
    Eigen::Vector2d pt_outside = Eigen::Vector2d::Zero();
    Eigen::Vector2d pt_inside = Eigen::Vector2d::Zero();
    Eigen::Vector2d pt_terminal_pos = Eigen::Vector2d::Zero();

    // slot corner
    Eigen::Vector2d corner_inside_slot = Eigen::Vector2d::Zero();
    Eigen::Vector2d corner_outside_slot = Eigen::Vector2d::Zero();

    // obs tlane
    Eigen::Vector2d obs_pt_inside = Eigen::Vector2d::Zero();
    Eigen::Vector2d obs_pt_outside = Eigen::Vector2d::Zero();
    double curb_y = 2.6;

    uint8_t slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;
    double slot_side_sgn = 1.0;

    double slot_width = 2.4;
    double slot_length = 6.0;

    double channel_y = 6.5;
    double channel_x_limit = 16.6;

    void Reset() {
      pt_outside = Eigen::Vector2d::Zero();
      pt_inside = Eigen::Vector2d::Zero();
      pt_terminal_pos = Eigen::Vector2d::Zero();

      corner_inside_slot = Eigen::Vector2d::Zero();
      corner_outside_slot = Eigen::Vector2d::Zero();

      obs_pt_inside = Eigen::Vector2d::Zero();
      obs_pt_outside = Eigen::Vector2d::Zero();

      curb_y = 2.6;
      slot_width = 2.4;
      slot_length = 6.0;
      channel_y = 6.5;
      channel_x_limit = 16.6;
      slot_side_sgn = 1.0;
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

  enum PathPlanState {
    NonePlanState,
    PrepareStepPlan,
    TripleStepPlan,
    MultiPlanInSlot,
    AdjustPlanInSlot,
    PathPlanCount,
  };

  enum MultiPlanMethod {
    OneArcMultiPlan,
    TwoArcMultiPlan,
    LineArcMultiPlan,
    MultiPlanMethodCount,
  };

  struct Input {
    uint8_t path_planner_state;
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
    pnc::geometry_lib::PathPoint target_pose;
    pnc::geometry_lib::PathPoint park_out_pose;
    pnc::geometry_lib::PathPoint safe_circle_root_pose;
    pnc::geometry_lib::Circle safe_circle;
    pnc::geometry_lib::LineSegment target_line;
    pnc::geometry_lib::LineSegment prepare_line;  // pA is tag point
    Eigen::Vector2d v_target_line = Eigen::Vector2d::Zero();
    Eigen::Vector2d v_prepare_line = Eigen::Vector2d::Zero();

    double min_outer_front_corner_radius = 5.5;
    double min_inner_rear_corner_radius = 5.5;
    Eigen::Vector2d v_ego_farest_front_corner = Eigen::Vector2d::Zero();
    Eigen::Vector2d v_ego_farest_rear_corner = Eigen::Vector2d::Zero();

    std::vector<pnc::geometry_lib::PathPoint> valid_target_pt_vec;
    std::vector<pnc::geometry_lib::PathSegment> inversed_park_out_path;
    std::vector<pnc::geometry_lib::PathSegment> park_out_path_in_slot;
    std::vector<pnc::geometry_lib::PathSegment> triple_step_path;

    void Reset() {
      is_left_side = true;
      slot_side_sgn = 1.0;
      target_pose.Reset();
      park_out_pose.Reset();
      safe_circle_root_pose.Reset();
      safe_circle.Reset();
      target_line.Reset();
      prepare_line.Reset();
      v_target_line.setZero();
      v_prepare_line.setZero();
      inversed_park_out_path.clear();
      inversed_park_out_path.reserve(3);
      park_out_path_in_slot.clear();
      park_out_path_in_slot.reserve(10);
      triple_step_path.clear();
      triple_step_path.reserve(6);
      valid_target_pt_vec.clear();
      valid_target_pt_vec.reserve(20);
      min_outer_front_corner_radius = 5.5;
      min_inner_rear_corner_radius = 5.5;
      v_ego_farest_front_corner = Eigen::Vector2d::Zero();
      v_ego_farest_rear_corner = Eigen::Vector2d::Zero();
    }
  };

 public:
  void Reset();
  void Preprocess();
  const bool Update();
  const bool Update(
      const std::shared_ptr<CollisionDetector> &collision_detector_ptr);

  const bool CheckTlaneAvailable() const;

  const bool SetCurrentPathSegIndex();
  void SetLineSegmentHeading();
  void ExtendCurrentFollowLastPath(double extend_distance);
  const bool InsertTwoLinePathBetweenPathSeg(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &pose, const uint8_t gear,
      const double extend_distance);
  void InsertLineSegAfterCurrentFollowLastPath(double extend_distance);
  const bool SampleCurrentPathSeg();

  void SetInput(const Input &input) { input_ = input; }
  const Output &GetOutput() const { return output_; }
  const Output *GetOutputPtr() const { return &output_; }
  const PlannerParams &GetPlannerParams() const { return calc_params_; }

  // pybind simulation
  const std::vector<double> GetPathEle(size_t index) const;
  const std::vector<double> GetMinSafeCircle() const;

 private:
  // prepare plan
  const bool PreparePlan();
  const bool PlanFromEgoToParkOutRootPose(const bool is_prepare_step);

  const bool PrepareStepFromEgo(
      const std::vector<pnc::geometry_lib::PathSegment>
          &inversed_park_out_path);

  const bool ConstructParkOutArcLengthVec(
      std::vector<double> &first_arc_length_vec, pnc::geometry_lib::Arc &arc);

  const bool ConstructParkOutLineLengthVec(
      std::vector<double> &line_length_vec,
      pnc::geometry_lib::LineSegment &line);

  const bool CalcParkOutPath(
      std::vector<pnc::geometry_lib::PathSegment> &reversed_park_out_path,
      const pnc::geometry_lib::Arc &first_arc,
      const pnc::geometry_lib::LineSegment &line,
      const double park_out_target_heading);

  const bool PlanFromEgoToParkOutEndPose(
      const Eigen::Vector2d &park_out_end_pos,
      const double park_out_end_heading, const bool is_prepare_step);

  // use dubins
  const bool OneStepDubinsPlan(const pnc::geometry_lib::PathPoint &start_pose,
                               const pnc::geometry_lib::PathPoint &target_pose,
                               const double radius, const double buffer = 0.0);
  const bool IsDubinsCollided(const double buffer = 0.0);

  const bool RSCurvePlan(const pnc::geometry_lib::PathPoint &current_pose,
                         const pnc::geometry_lib::PathPoint &target_pose,
                         const double radius);

  void GenPathOutputByDubins();

  void GetPathSegVecByDubins(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec);

  // normal plan
  const bool PlanFromTargetToEgoLine();

  const bool PlanFromTargetToLine(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &start_pose);

  const bool MonoStepPlanWithShift();
  const bool MonoStepPlanOnceWithShift(
      bool &is_drive_out_safe, pnc::geometry_lib::PathPoint &target_pose);
  const bool BackwardNormalPlan();

  const bool BackwardNormalPlan(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &start_pose);
  const bool BackWardTripleStepPlan();
  const bool BackWardTripleStepPlanOnce(
      std::vector<pnc::geometry_lib::PathSegment> &inversed_park_out_path,
      const double arc_length, const double line_length,
      const double park_out_target_heading);

  const bool OneStepDubinsTryInTripplePlan(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &start_pose);
  // optimized prepare plan
  const bool OutsideSlotPlan();

  const bool CheckEgoInSlot() const;
  const bool CalMinSafeCircle();
  const bool CalcParkOutPose(pnc::geometry_lib::PathSegment &park_out_seg);

  // inverse search from target in slot
  const bool InverseSearchLoopInSlot(
      std::vector<pnc::geometry_lib::PathSegment> &search_out_res,
      const pnc::geometry_lib::PathPoint &terminal_pose);

  const bool ReduceRootPoseHeadingInSlot(
      std::vector<pnc::geometry_lib::PathSegment> &search_out_res);

  const bool CalcLineStepLimitPose(
      pnc::geometry_lib::LineSegment &line,
      const uint8_t gear);  // start pose should be given in line

  const bool CalcArcStepLimitPose(
      pnc::geometry_lib::Arc &arc, bool &is_drive_out_safe, const uint8_t gear,
      const uint8_t steer,
      const double buffer =
          0.15);  // start pose and radius should be given in arc

  const bool CheckParkOutCornerSafeWithObsPin(
      const pnc::geometry_lib::Arc &first_arc) const;

  const bool TwoSameGearArcPlanToLine(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &start_pose,
      const pnc::geometry_lib::LineSegment &target_line, const uint8_t gear,
      const double buffer = 0.0);

  const bool CalcParkOutPath(
      std::vector<pnc::geometry_lib::PathSegment> &reversed_park_out_path,
      const double arc_length, const double line_length,
      const double park_out_target_heading);

  // fill output
  void AddPathSegToOutPut(
      const std::vector<pnc::geometry_lib::PathSegment> &path_seg);
  void AddLastArc();
  void GenTriplePath();
  void AddPathSegToOutPut(const pnc::geometry_lib::PathSegment &path_seg);
  void AddPathSegVecToOutput(
      const std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec);

  // multi plan
  const bool MultiPlan();
  const bool CheckMultiPlanSuitable(
      const pnc::geometry_lib::PathPoint &current_pose) const;

  const bool CalSinglePathInMulti(
      const pnc::geometry_lib::PathPoint &current_pose,
      const uint8_t current_gear, const uint8_t current_arc_steer,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec);

  const bool MultiAlignBody();  // multi align body if multi plan failed

  const bool OneArcPlan(
      pnc::geometry_lib::Arc &arc,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const uint8_t current_gear, const uint8_t current_arc_steer);

  // adjust plan
  const bool AdjustPlan();
  const bool ParallelAdjustPlan();
  const bool CSCSAdjustPlan();

  const bool CheckAdjustPlanSuitable(
      const pnc::geometry_lib::PathPoint &current_pose) const;

  const bool CalcLineDirAllValidPose(
      std::vector<pnc::geometry_lib::PathPoint> &target_tan_pose_vec,
      const pnc::geometry_lib::PathPoint start_pose,
      const pnc::geometry_lib::LineSegment &terminal_line,
      const double shift_ratio = 1.0);

  const bool CalcLineDirAllValidPose(
      std::vector<pnc::geometry_lib::PathPoint> &target_tan_pose_vec,
      const pnc::geometry_lib::LineSegment &target_line);

  const bool SameSteerCSCToLine(
      std::vector<pnc::geometry_lib::PathPoint> &target_tan_pose_vec,
      const uint8_t start_steer, const uint8_t start_gear,
      const pnc::geometry_lib::PathPoint &start_pose,
      const pnc::geometry_lib::LineSegment &terminal_line,
      const double shift_ratio = 1.0);

  const bool SameSteerCSCPlan(const pnc::geometry_lib::PathPoint &start_pose,
                              const pnc::geometry_lib::PathPoint &target_pose,
                              const uint8_t start_steer,
                              const uint8_t start_gear);

  const bool CSCSInclinedStepPlan(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &current_pose,
      const pnc::geometry_lib::LineSegment &target_line,
      const uint8_t current_gear);

  const bool OneArcPlan(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &current_pose,
      const uint8_t current_gear);

  const bool OneArcPlan(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &current_pose,
      const pnc::geometry_lib::LineSegment &target_line,
      const uint8_t current_gear);

  const bool TwoArcPlan(
      pnc::geometry_lib::Arc &arc,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const uint8_t current_gear, const uint8_t current_arc_steer);

  const bool LineArcPlan(
      pnc::geometry_lib::Arc &arc,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const uint8_t current_gear, const uint8_t current_arc_steer);

  const bool AlignBodyPlan(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &current_pose,
      const uint8_t current_gear);

  const bool AlignBodyPlan(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &current_pose,
      const double target_heading, const uint8_t current_gear);

  const bool STurnParallelPlan(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &current_pose,
      const pnc::geometry_lib::LineSegment &target_line,
      const uint8_t current_gear, const double steer_change_ratio1,
      const double radius);

  const bool CalSinglePathInAdjust(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &current_pose,
      const uint8_t current_gear, const double steer_change_ratio,
      const double radius);

  const uint8_t TrimPathByCollisionDetection(
      pnc::geometry_lib::PathSegment &path_seg, const double buffer = 0.3);

  const uint8_t TrimPathByCollisionDetection(
      pnc::geometry_lib::PathSegment &path_seg, Eigen::Vector2d &collision_pt,
      const double buffer = 0.3);

  const bool CheckPathSegCollided(
      const pnc::geometry_lib::PathSegment &path_seg,
      const double buffer = 0.3) const;

  const bool CheckPathSegVecCollided(
      const std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const double buffer = 0.3) const;

  const bool OneLinePlan(
      pnc::geometry_lib::LineSegment &line,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const uint8_t current_gear);

  const bool OneLinePlan(pnc::geometry_lib::LineSegment &line,
                         const pnc::geometry_lib::PathPoint &target_pose)
      const;  // start pose is given in line

  const bool OneLinePlanInCSCS(
      pnc::geometry_lib::LineSegment &line,
      const pnc::geometry_lib::PathPoint &target_pose) const;

  const bool OneLinePlanAlongEgoHeading(
      pnc::geometry_lib::LineSegment &line,
      const pnc::geometry_lib::PathPoint
          &target_pose);  // start pose is given in line

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

  const bool IsOnTarget(const pnc::geometry_lib::PathPoint &current_pose) const;

  const bool CheckLonToTarget(
      const pnc::geometry_lib::PathPoint &current_pose) const;

  const bool IsOnTargetLine(const pnc::geometry_lib::PathPoint &current_pose);

  const bool CheckSamePose(const pnc::geometry_lib::PathPoint &pose1,
                           const pnc::geometry_lib::PathPoint &pose2) const;

  void CalcEgoParams();

  Input input_;
  Output output_;
  PlannerParams calc_params_;
  pnc::dubins_lib::DubinsLibrary dubins_planner_;
  std::shared_ptr<CollisionDetector> collision_detector_ptr_ = nullptr;
};

}  // namespace apa_planner
}  // namespace planning

#endif