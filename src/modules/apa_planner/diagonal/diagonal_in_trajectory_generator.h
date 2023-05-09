#pragma once

#include "frame.h"
#include "apa_planner/common/geometry_planning_io.h"
#include "apa_planner/diagonal/diagonal_in_geometry_plan.h"
#include "common/local_view.h"
#include "common/math/box2d.h"

namespace planning {
namespace apa_planner {

class DiagonalInTrajectoryGenerator {
 public:
  DiagonalInTrajectoryGenerator() = default;
  ~DiagonalInTrajectoryGenerator() = default;

  bool Plan(framework::Frame* const frame);

 private:
  bool SingleSlotPlan(const int slot_index,
      PlanningOutput::PlanningOutput *const planning_output);

  bool GeometryPlan(const PlanningPoint &start_point,
      int idx, PlanningOutput::PlanningOutput *const planning_output);

  bool ABSegmentPlan(
      const PlanningPoint &point_a, bool is_start, int idx,
      DiagonalInGeometryPlan * const geometry_planning,
      PlanningOutput::PlanningOutput *const planning_output) const;

  bool BCSegmentPlan(
      const PlanningPoint &point_b, bool is_start, int idx,
      DiagonalInGeometryPlan * const geometry_planning,
      PlanningOutput::PlanningOutput *const planning_output) const;

  bool CDSegmentPlan(
      const PlanningPoint &point_c, bool is_start, int idx,
      DiagonalInGeometryPlan * const geometry_planning,
      PlanningOutput::PlanningOutput *const planning_output) const;

  bool DESegmentPlan(
      const PlanningPoint &point_d, bool is_start, int idx,
      DiagonalInGeometryPlan * const geometry_planning,
      PlanningOutput::PlanningOutput *const planning_output) const;

  bool GenerateABSegmentTrajectory(
      const DiagonalSegmentsInfo& segments_info,
      PlanningOutput::PlanningOutput *const planning_output) const;

  bool GenerateBCSegmentTrajectory(
      const DiagonalSegmentsInfo& segments_info,
      PlanningOutput::PlanningOutput *const planning_output) const;

  bool GenerateCDSegmentTrajectory(
      const DiagonalSegmentsInfo& segments_info,
      PlanningOutput::PlanningOutput *const planning_output) const;

  bool GenerateDESegmentTrajectory(
      const DiagonalSegmentsInfo& segments_info,
      PlanningOutput::PlanningOutput *const planning_output) const;

  void SetApaObjectInfo(int idx, DiagonalInGeometryPlan* geometry_planning) const;

  void SetGeometryPlanningParameter(int idx,
      DiagonalInGeometryPlan *geometry_planning) const;

  void GetCurPtSpeed(const double segment_len, const double cur_s,
      const double spd_sign,
      PlanningOutput::TrajectoryPoint* trajectory_point) const;

  void GetCurPtSpeed(const double spd_sign,
      PlanningOutput::TrajectoryPoint* trajectory_point) const;

  PlanningPoint FromLocal2GlobalCor(const PlanningPoint &ego,
      const PlanningPoint &local) const;

  PlanningPoint FromGlobal2LocalCor(const PlanningPoint &ego,
      const PlanningPoint &global) const;

  double CalApaTargetY() const;

  double CalApaTargetX(int idx) const;

  void CalApaTargetInSlot(int idx);

  void CalEgoPostionInSlotAndOdom(int idx);

  void CalSlotOriginInodom(const int idx);

  void CalSlotPointsInM(const int idx);

  void SquareSlot();

  bool IsReplan(PlanningOutput::PlanningOutput *const planning_output);

  void SetPlanningOutputInfo(
      PlanningOutput::PlanningOutput *const planning_output) const;

  bool IsSamePoint( const PlanningPoint& p1, const PlanningPoint& p2) const;

  void UpdateStandstillTime();

  void UpdatePosUnchangedCount();

  bool IsApaFinished() const;

  void SetFinishedPlanningOutput(
      PlanningOutput::PlanningOutput *const planning_output) const;

  void PrintTrajectoryPoints(
      const PlanningOutput::PlanningOutput& planning_output) const;

  bool IsSelectedSlotValid(framework::Frame* const frame) const;

 private:
  DiagonalInGeometryPlan geometry_planning_;

  int slot_sign_ = 0;  // 1:Right ,-1:Left, 0:Invalid

  const LocalView* local_view_ = nullptr;

  PlanningPoint slot_origin_in_odom_;

  PlanningPoint target_point_in_slot_;
  PlanningPoint target_point_in_odom_;

  PlanningPoint cur_pos_in_slot_;
  PlanningPoint cur_pos_in_odom_;
  PlanningPoint last_pos_in_odom_;

  std::vector<PlanningPoint> raw_slot_points_in_m_;
  std::vector<PlanningPoint> slot_points_in_m_;

  double slot_width_ = 0.0;
  double slot_length_ = 0.0;

  bool is_replan_ = false;
  std::string last_segment_name_ = "";
  uint64_t standstill_time_ = 0;
  uint64_t last_time_ = 0;
  uint64_t pos_unchanged_cnt_ = 0;

  bool is_rough_calc_ = false;

  ::FuncStateMachine::FunctionalState current_state_ =
        ::FuncStateMachine::FunctionalState::INIT;
};

} // namespace apa_planner
} // namespace planning