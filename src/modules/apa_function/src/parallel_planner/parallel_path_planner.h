#ifndef __PARALLEL_PATH_PLANNER_H__
#define __PARALLEL_PATH_PLANNER_H__

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "apa_path_planner.h"
#include "collision_detection.h"
#include "dubins_lib.h"
#include "geometry_math.h"

namespace planning {
namespace apa_planner {

class ParallelPathPlanner : public ApaPathPlanner {
 public:
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
    double min_outer_front_corner_deta_y = 0.8;
    Eigen::Vector2d v_ego_farest_front_corner = Eigen::Vector2d::Zero();
    Eigen::Vector2d v_ego_farest_rear_corner = Eigen::Vector2d::Zero();

    std::vector<double> lat_outside_slot_buffer_vec = {0.4, 0.4};
    std::vector<Eigen::Vector2d> front_corner_obs_vec;
    std::vector<Eigen::Vector2d> channel_obs_vec;
    std::vector<Eigen::Vector2d> virtual_channel_obs_vec;
    std::vector<pnc::geometry_lib::PathPoint> valid_target_pt_vec;
    std::vector<pnc::geometry_lib::PathSegment> park_out_path_in_slot;
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
      park_out_path_in_slot.clear();
      park_out_path_in_slot.reserve(10);
      valid_target_pt_vec.clear();
      valid_target_pt_vec.reserve(20);
      front_corner_obs_vec.clear();
      front_corner_obs_vec.reserve(20);
      channel_obs_vec.clear();
      channel_obs_vec.reserve(50);
      virtual_channel_obs_vec.clear();
      virtual_channel_obs_vec.reserve(50);
      lat_outside_slot_buffer_vec = {0.4, 0.4};
      min_outer_front_corner_radius = 5.5;
      min_inner_rear_corner_radius = 5.5;
      min_outer_front_corner_deta_y = 0.8;
      v_ego_farest_front_corner = Eigen::Vector2d::Zero();
      v_ego_farest_rear_corner = Eigen::Vector2d::Zero();
    }
  };

  struct GeometryPath {
    void Reset() {
      length = 0.0;
      gear_change_count = 0;
      park_out_heading_deg = 0.0;
      gear_cmd_vec.clear();
      gear_cmd_vec.reserve(10);
      path_segment_vec.clear();
      path_segment_vec.reserve(10);
    }

    double length = 0.0;
    double first_path_length = 0.0;
    size_t gear_change_count = 0;
    double park_out_heading_deg = 0.0;
    std::vector<uint8_t> gear_cmd_vec;
    std::vector<pnc::geometry_lib::PathSegment> path_segment_vec;
  };

  struct DebugInfo {
    std::vector<pnc::geometry_lib::Arc> debug_arc_vec;
    std::vector<GeometryPath> debug_all_path_vec;
    std::vector<pnc::geometry_lib::PathSegment> tra_search_out_res;
  };

 public:
  virtual void Reset() override;
  virtual const bool Update() override;
  virtual const bool Update(const std::shared_ptr<CollisionDetector>
                                &collision_detector_ptr) override;

  void CalcEgoParams();
  void ExpandPInObstacles();
  void AddPInVirtualObstacles();
  void DeletePInVirtualObstacles();
  void MoveChannelObstacles();
  void RecorverChannelObstacles();

  const bool CheckTlaneAvailable() const;

  void ExtendCurrentFollowLastPath(double extend_distance);
  const bool InsertTwoLinePathBetweenPathSeg(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &pose, const uint8_t gear,
      const double extend_distance);
  void InsertLineSegAfterCurrentFollowLastPath(double extend_distance);

  const PlannerParams &GetPlannerParams() const { return calc_params_; }

  const DebugInfo &GetDebugInfo() const { return debug_info_; };

  const std::vector<Eigen::Vector2d> GetVirtualObs() {
    std::vector<Eigen::Vector2d> obs_vec;
    obs_vec = calc_params_.virtual_channel_obs_vec;
    for (const auto &obs_pt : calc_params_.front_corner_obs_vec) {
      obs_vec.emplace_back(obs_pt);
    }
    return obs_vec;
  }

 private:
  virtual void Preprocess() override;

  const bool AssempleGeometryPath(
      GeometryPath &geometry_path,
      const std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec);

  const bool CalcParkOutPath(
      std::vector<pnc::geometry_lib::PathSegment> &reversed_park_out_path,
      const pnc::geometry_lib::Arc &first_arc,
      const pnc::geometry_lib::LineSegment &line,
      const double park_out_target_heading);

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

  const bool PlanFromTargetToLine(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &start_pose);

  const bool PlanFromTargetToLineInNarrowChannel(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::Arc &arc1, const pnc::geometry_lib::Arc &arc_2);

  const bool CalSinglePathInNarrowChannel(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &start_pose,
      const pnc::geometry_lib::PathPoint &target_pose, const uint8_t gear,
      const uint8_t steer);

  const bool MonoStepPlanWithShift();
  const bool MonoStepPlanOnceWithShift(
      bool &is_drive_out_safe, const pnc::geometry_lib::PathPoint &target_pose);
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

  const bool SelectBestPathOutsideSlot(
      const std::vector<GeometryPath> &path_vec, size_t &best_path_idx);

  const bool PlanToPreparingLine(
      std::vector<pnc::geometry_lib::PathSegment> &ego_to_prepare_seg_vec,
      const pnc::geometry_lib::PathPoint &ego_pose,
      const pnc::geometry_lib::LineSegment &prepare_line);
  const std::vector<double> GetMinDistOfEgoToObs();

  const bool GenAlignedPreparingLine(
      std::vector<double> &preparing_y_vec,
      const pnc::geometry_lib::PathPoint &ego_pose);
  const bool GenPreparingLineVec(std::vector<double> &preparing_y_vec);

  const bool CheckEgoInSlot() const;
  const bool CalMinSafeCircle();
  const bool CalcParkOutPose(pnc::geometry_lib::PathSegment &park_out_seg);

  const bool ReversePathSegVec(
      std::vector<pnc::geometry_lib::PathSegment> &park_out_res);

  // inverse search from target in slot
  const bool InverseSearchLoopInSlot(
      std::vector<pnc::geometry_lib::PathSegment> &search_out_res,
      const pnc::geometry_lib::PathPoint &terminal_pose);

  const bool ReduceRootPoseHeadingInSlot(
      std::vector<pnc::geometry_lib::PathSegment> &search_out_res);

  const bool AdvancedInversedTrialsInSlot(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &target_pose);

  const size_t CalPathGearChangeCounts(
      const std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec);

  const bool GenLineStepValidEnd(
      std::vector<Eigen::Vector2d> &line_end_vec,
      const pnc::geometry_lib::PathPoint &target_pose);

  const bool InversedTrialsByGivenGear(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &start_pose,
      const uint8_t current_gear);

  const bool CalcLineStepLimitPose(
      pnc::geometry_lib::LineSegment &line, const uint8_t gear,
      const double buffer = 0.3);  // start pose should be given in line

  const bool CalcArcStepLimitPose(
      pnc::geometry_lib::Arc &arc, const uint8_t gear, const uint8_t steer,
      const double buffer =
          0.15);  // start pose and radius should be given in arc

  const bool CheckParkOutCornerSafeWithObsPin(
      const pnc::geometry_lib::Arc &first_arc) const;

  const bool TwoSameGearArcPlanToLine(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &start_pose,
      const pnc::geometry_lib::LineSegment &target_line, const uint8_t gear,
      const double buffer = 0.0);

  const bool TwoSameGearArcPlanToLine(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &start_pose,
      const pnc::geometry_lib::LineSegment &target_line, const uint8_t gear,
      const double radius, const double buffer = 0.0);

  const bool CalcParkOutPath(
      std::vector<pnc::geometry_lib::PathSegment> &reversed_park_out_path,
      const double arc_length, const double line_length,
      const double park_out_target_heading);

  // fill output
  void AddPathSegToOutPut(
      const std::vector<pnc::geometry_lib::PathSegment> &path_seg);
  void AddLastArc();
  void AddPathSegToOutPut(const pnc::geometry_lib::PathSegment &path_seg);
  void AddPathSegVecToOutput(
      const std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec);

  const bool MultiPlan(std::vector<GeometryPath> &path_vec,
                       const pnc::geometry_lib::PathPoint &start_pose,
                       const pnc::geometry_lib::PathPoint &target_pose,
                       const uint8_t ref_gear);
  // multi plan
  const bool MultiPlan();
  const bool CheckMultiPlanSuitable(
      const pnc::geometry_lib::PathPoint &current_pose) const;

  const bool CalSinglePathInMulti(
      const pnc::geometry_lib::PathPoint &current_pose,
      const pnc::geometry_lib::PathPoint &target_pose,
      const uint8_t current_gear, const uint8_t current_arc_steer,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec);

  const bool MultiAlignBody();  // multi align body if multi plan failed

  const bool OneArcPlan(
      pnc::geometry_lib::Arc &arc,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      pnc::geometry_lib::LineSegment &target_line, const uint8_t current_gear,
      const uint8_t current_arc_steer);

  // adjust plan
  const bool AdjustPlan();
  const bool ParallelAdjustPlan();

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

  const bool OneArcPlan(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &current_pose,
      const uint8_t current_gear);

  const bool OneArcPlan(
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      const pnc::geometry_lib::PathPoint &current_pose,
      pnc::geometry_lib::LineSegment &target_line, const uint8_t current_gear);

  const bool TwoArcPlan(
      pnc::geometry_lib::Arc &arc,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      pnc::geometry_lib::LineSegment &target_line, const uint8_t current_gear,
      const uint8_t current_arc_steer);

  const bool LineArcPlan(
      pnc::geometry_lib::Arc &arc,
      std::vector<pnc::geometry_lib::PathSegment> &path_seg_vec,
      pnc::geometry_lib::LineSegment &target_line, const uint8_t current_gear,
      const uint8_t current_arc_steer);

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

  const bool OneLinePlan(pnc::geometry_lib::PathSegment &line_seg,
                         const pnc::geometry_lib::PathPoint &start_pose,
                         const pnc::geometry_lib::LineSegment &target_line,
                         const double lon_buffer = 0.3) const;

  const bool OneLinePlanAlongEgoHeading(
      pnc::geometry_lib::LineSegment &line,
      const pnc::geometry_lib::PathPoint
          &target_pose);  // start pose is given in line

  const Eigen::Vector2d CalEgoTurningCenter(
      const pnc::geometry_lib::PathPoint &ego_pose, const double radius,
      const uint8_t steer) const;

  const bool IsOnTarget(const pnc::geometry_lib::PathPoint &current_pose) const;

  const bool CheckLonToTarget(
      const pnc::geometry_lib::PathPoint &current_pose) const;

  const bool IsOnTargetLine(const pnc::geometry_lib::PathPoint &current_pose);

  const bool CheckSamePose(const pnc::geometry_lib::PathPoint &pose1,
                           const pnc::geometry_lib::PathPoint &pose2) const;

  const bool CheckSamePos(const Eigen::Vector2d &pos0,
                          const Eigen::Vector2d &pos1) const;

  PlannerParams calc_params_;
  DebugInfo debug_info_;
};

}  // namespace apa_planner
}  // namespace planning

#endif