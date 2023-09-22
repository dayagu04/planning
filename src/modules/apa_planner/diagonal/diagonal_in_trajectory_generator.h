#pragma once

#include <memory>

#include "Platform_Types.h"
#include "common/geometry_planning_io.h"
#include "diagonal/diagonal_in_geometry_plan.h"
#include "frame.h"
#include "local_view.h"
#include "math/box2d.h"
#include "planning_plan.pb.h"
#include "slot_management/slot_management.h"
#include "speed_smoother/apa_speed_smoother.h"

namespace planning {
namespace apa_planner {

class DiagonalInTrajectoryGenerator {
 public:
  struct SimulationParam {
    uint8_t force_last_seg_name_ = 0;
    int selected_id_ = false;
    bool force_planning_ = false;
    uint8_t current_state_ = 0;
  };

  enum SegmentName {
    SEGMENT_NONE,
    SEGMENT_AB,
    SEGMENT_BC,
    SEGMENT_CD,
    SEGMENT_DE
  };

  DiagonalInTrajectoryGenerator() {
    managed_parking_fusion_info_ptr_ =
        std::make_shared<ParkingFusion::ParkingFusionInfo>();
  };
  ~DiagonalInTrajectoryGenerator() = default;

  bool Plan(framework::Frame *const frame);

  // for pybind & simulation
  void SetLocalView(LocalView *local_view_ptr) { local_view_ = local_view_ptr; }

  PlanningOutput::PlanningOutput GetOutput() { return planning_output_; }

  const common::SlotManagementInfo GetSlotManagementOutput() const {
    return slot_manager_.GetOutput();
  }

  void SetSimulationParam(SimulationParam &param) { simu_param_ = param; }

  bool SingleSlotPlanSimulation();

 private:
  bool SingleSlotPlan(const int slot_index,
                      PlanningOutput::PlanningOutput *const planning_output);

  bool GeometryPlan(const PlanningPoint &start_point, int idx,
                    PlanningOutput::PlanningOutput *const planning_output);

  bool ABSegmentPlan(
      const PlanningPoint &point_a, bool is_start, int idx,
      DiagonalInGeometryPlan *const geometry_planning,
      PlanningOutput::PlanningOutput *const planning_output) const;

  bool ReverseABSegmentPlan(
      const PlanningPoint &point_a, bool is_start, int idx,
      DiagonalInGeometryPlan *const geometry_planning,
      PlanningOutput::PlanningOutput *const planning_output) const;

  bool BCSegmentPlan(
      const PlanningPoint &point_b, bool is_start, int idx,
      DiagonalInGeometryPlan *const geometry_planning,
      PlanningOutput::PlanningOutput *const planning_output) const;

  bool CDSegmentPlan(
      const PlanningPoint &point_c, bool is_start, int idx,
      DiagonalInGeometryPlan *const geometry_planning,
      PlanningOutput::PlanningOutput *const planning_output) const;

  bool DESegmentPlan(
      const PlanningPoint &point_d, bool is_start, int idx,
      DiagonalInGeometryPlan *const geometry_planning,
      PlanningOutput::PlanningOutput *const planning_output) const;

  bool GenerateABSegmentTrajectory(
      const DiagonalSegmentsInfo &segments_info,
      PlanningOutput::PlanningOutput *const planning_output) const;

  bool GenerateRACSegmentTrajectory(
      const DiagonalSegmentsInfo &segments_info,
      PlanningOutput::PlanningOutput *const planning_output) const;

  bool GenerateBCSegmentTrajectory(
      const DiagonalSegmentsInfo &segments_info,
      PlanningOutput::PlanningOutput *const planning_output) const;

  bool GenerateCDSegmentTrajectory(
      const DiagonalSegmentsInfo &segments_info,
      PlanningOutput::PlanningOutput *const planning_output) const;

  bool GenerateDESegmentTrajectory(
      const DiagonalSegmentsInfo &segments_info,
      PlanningOutput::PlanningOutput *const planning_output) const;

  void SetApaObjectInfo(int idx, DiagonalInGeometryPlan *geometry_planning);

  void SetGeometryPlanningParameter(int idx,
                                    DiagonalInGeometryPlan *geometry_planning);

  void GetCurPtSpeed(const double segment_len, const double cur_s,
                     const double spd_sign,
                     PlanningOutput::TrajectoryPoint *trajectory_point) const;

  void GetCurPtSpeed(const double spd_sign,
                     PlanningOutput::TrajectoryPoint *trajectory_point) const;

  PlanningPoint FromLocal2GlobalCor(const PlanningPoint &ego,
                                    const PlanningPoint &local) const;

  PlanningPoint FromGlobal2LocalCor(const PlanningPoint &ego,
                                    const PlanningPoint &global) const;

  double CalApaTargetY() const;

  double CalApaTargetX() const;

  void CalApaTargetInSlot();

  void CalEgoPostionInSlotAndOdom();

  void CalSlotOriginInodom();

  void CalSlotPointsInM(const int idx);

  void SquareSlot();

  bool IsReplan(PlanningOutput::PlanningOutput *const planning_output);

  void SetPlanningOutputInfo(
      PlanningOutput::PlanningOutput *const planning_output) const;

  bool IsSamePoint(const PlanningPoint &p1, const PlanningPoint &p2) const;

  void UpdateStandstillTime();

  void UpdatePosUnchangedCount();

  bool IsApaFinished() const;

  void PrintTrajectoryPoints(
      const PlanningOutput::PlanningOutput &planning_output) const;

  bool IsSelectedSlotValid(framework::Frame *const frame) const;

  void PrintSlotInfo() const;

  void UpdateManagedParkingFusion(const int select_slot_index);

  bool IsSegmentExist(const PlanningPoint &end_point) const;

 private:
  DiagonalInGeometryPlan geometry_planning_;

  int slot_sign_ = 0;  // 1:Right ,-1:Left, 0:Invalid

  const LocalView *local_view_ = nullptr;
  framework::Frame *frame_ = nullptr;

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
  uint8_t last_segment_name_ = SEGMENT_NONE;
  uint64_t standstill_time_ = 0;
  uint64_t last_time_ = 0;
  uint64_t pos_unchanged_cnt_ = 0;

  bool is_rough_calc_ = false;

  uint8_t current_state_ = ::FuncStateMachine::FunctionalState::INIT;

  std::vector<planning_math::LineSegment2d> objects_map_in_global_cor_;

  ApaSpeedSmoother apa_speed_smoother_;

  // for simulation
  PlanningOutput::PlanningOutput planning_output_;
  bool simulation_enable_flag_ = false;
  SimulationParam simu_param_;

  SlotManagement slot_manager_;

  ParkingFusion::ParkingFusionInfo managed_parking_fusion_info_;

  std::shared_ptr<ParkingFusion::ParkingFusionInfo>
      managed_parking_fusion_info_ptr_;
};

}  // namespace apa_planner
}  // namespace planning