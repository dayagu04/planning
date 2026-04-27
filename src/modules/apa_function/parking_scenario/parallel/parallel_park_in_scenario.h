#ifndef __PARALLEL_PARK_IN_PLANNER_H__
#define __PARALLEL_PARK_IN_PLANNER_H__

#include <cstdint>
#include <deque>
#include <memory>
#include <vector>

#include "apa_world.h"
#include "dubins_lib.h"
#include "parallel_path_generator.h"
#include "src/modules/apa_function/parking_scenario/parking_scenario.h"
#include "src/modules/apa_function/apa_common/relative_loc_observer_manager.h"
#include "src/library/hybrid_astar_lib/hybrid_astar_interface.h"
#include "src/library/hybrid_astar_lib/hybrid_astar_thread.h"
#include "src/library/hybrid_astar_lib/hybrid_astar_common.h"
#include "src/modules/apa_function/apa_common/apa_slot.h"
#include "src/modules/apa_function/util/apa_utils.h"
#include "src/modules/apa_function/parking_task/deciders/virtual_wall_decider/virtual_wall_decider.h"

namespace planning {
namespace apa_planner {

struct AngleResult {
  double ang;
  bool res;
  double curb_y;

  AngleResult() = default;
  AngleResult(double ang_, bool res_, double curb_y_)
      : ang(ang_), res(res_), curb_y(curb_y_) {}

  bool operator<(const AngleResult& other) const { return ang < other.ang; }
};

struct AngleResultHeightObs {
  double ang;
  bool res;


  AngleResultHeightObs() = default;
  AngleResultHeightObs(double ang_, bool res_)
      : ang(ang_), res(res_) {}

  bool operator<(const AngleResult& other) const { return ang < other.ang; }
};

struct PaTryResult {
  std::vector<pnc::geometry_lib::PathPoint> all_path_point;
  double to_curb_dis;

  PaTryResult() = default;
  PaTryResult(const std::vector<pnc::geometry_lib::PathPoint>& path_points,
              double curb_dis)
      : all_path_point(path_points), to_curb_dis(curb_dis) {}
};

class ParallelParkInScenario : public ParkingScenario {
 public:
  ParallelParkInScenario() = default;
  ParallelParkInScenario(const std::shared_ptr<ApaWorld>& apa_world_ptr) {
    SetApaWorldPtr(apa_world_ptr);
    memset(&last_apa_hmi_, 0, sizeof(last_apa_hmi_));
    Init();
  }

  virtual void Init() override;
  virtual void ThreadClearState() override;
  virtual void Reset() override;
  virtual std::string GetName() override { return typeid(this).name(); };
  virtual const bool UpdateEgoSlotInfo() override;
  virtual const bool GenTlane() override;
  virtual const uint8_t PathPlanOnce() override;
  virtual const bool CheckFinished() override;
  virtual const bool GenObstacles() override;
  virtual void ExcutePathPlanningTask() override;
  virtual void ScenarioTry() override;
  virtual void Log() const override;

  virtual const PathPlannerResult PathPlanOnceGeometry();
  virtual void SetTargetGroup(AstarRequest& cur_request);
  virtual void UpdateStartPosByInSlot(
      Pose2f& start_pos, pnc::geometry_lib::PathSegGear& last_path_gear);
  virtual void SetRequestForScenarioTry(AstarRequest& cur_request,
                                const EgoInfoUnderSlot& ego_info);
  virtual void UpdatePathByGeometry(const AstarResponse& response);
  virtual const bool SetAstarRequest(AstarRequest& astar_request,
                                     ParkObstacleList& obs_hastar);
  virtual const PathPlannerResult PubResponseForScenarioTry(
      const EgoInfoUnderSlot& ego_info);
  virtual const PathPlannerResult PubResponseForScenarioRunning(
      const EgoInfoUnderSlot& ego_info);
  virtual const PathPlannerResult PathPlanOnceHybridAStar();
  virtual void UpdatePostProcessStatus(PathPlannerResult pathplan_result);

  const double CalcSlotOccupiedRatio(const Eigen::Vector2d& terminal_err,
                                     const double slot_width,
                                     const bool is_right_side) const;
  const double CalcSlotOccupiedRatio(
      const pnc::geometry_lib::PathPoint start_pose) const;

  void GenTBoundaryObstacles();

  const Tlane& GetTlane() { return t_lane_; }

  virtual const double CalRealTimeBrakeDist() override;
  const bool IsPointInOrientedRectangle(const Eigen::Vector2d& point,
                                        const Eigen::Vector2d& rect_center,
                                        const double rect_heading,
                                        const double rect_length,
                                        const double rect_width);
  SlotCoord AlignAndMoveSlotToLine(
      const SlotCoord& slot, const Eigen::Vector2d& ego_pos,
      const Eigen::Vector2d& ego_head, double k, double b, double slot_width,
      double distance = 0.2,
      const ApaPADirection direction = ApaPADirection::PA_INVALID);
  const bool LineFittingWithRansac(Eigen::Vector2d& line_coeffs,
                                   const std::vector<Eigen::Vector2d>& points,
                                   std::vector<bool>& inliers);
  const bool GetOffsetLineCoeffsWithEgoPose(
      Eigen::Vector2d& line_coeffs_out, const Eigen::Vector2d& line_coeffs_in,
      const std::vector<Eigen::Vector2d>& points,
      const std::vector<bool>& inliers, const Eigen::Vector2d& ego_pose);

  const bool GeneralPASlot(const ApaPADirection direction);
  const uint8_t ParkInTry(const ApaSlot& slot, const ApaPADirection direction =
                                                   ApaPADirection::PA_INVALID);
  const bool CheckPAFinished();

  const bool UpdatePASlotInfo();
  void ExtractLongestLineSegmentPointsPCA(
      const std::vector<Eigen::Vector2d>& obs_pos,
      std::vector<Eigen::Vector2d>& pa_curb_obs, Eigen::Vector2d& line_coeffs,
      double distance_threshold = 0.2);
  void EnablePAPark() {
    enable_pa_park_ = true;
    return;
  }
  void RecordDebugPaInfo(const Eigen::Vector2d& line_coeffs,
                         const SlotCoord& slot,
                         const std::vector<Eigen::Vector2d>& pa_curb_obs);
  void UpdatePARemainDistance();
  HybridAStarThreadSolver* GetThread() { return &thread_; }

  const int PublishHybridAstarCompletePathInfo(const HybridAStarResult& result,
                                        Transform2d* tf);
  const bool IsNeedClipping(const HybridAStarResult& result, const size_t i);
  const pnc::geometry_lib::PathSegGear GetGear(const AstarPathGear gear);

  const int LocalPathToGlobal(
      const std::vector<pnc::geometry_lib::PathPoint>& local_path,
      Transform2d* tf);
  const int PublishHybridAstarCurrentPathInfo(
      const std::vector<AStarPathPoint>& first_seg_path, Transform2d* tf);

  void FillPlanningReason(AstarRequest& astar_request);
  void FillPlanningMethod(AstarRequest& astar_request);
  void FillGearRequest(AstarRequest& astar_request,
                       pnc::geometry_lib::PathSegGear& last_path_gear);
  const bool UpdateThreadPath();
  void RecordSearchTime(const SearchTimeBenchmark& time);
  void RecordSearchTrajectoryInfo(const SearchTrajectoryInfo& search_traj_info);
  const int HybridAstarDebugInfoClear();

  SlotCoord slot_new_;
  Eigen::Vector2d line_coeffs_out_;

 protected:
  void LateralPathOptimization(
      std::vector<pnc::geometry_lib::PathPoint>& out_path_vec,
      const std::vector<pnc::geometry_lib::PathPoint>& in_path_vec,
      const uint8_t gear_cmd);
  bool UseLastPathForAstar();
  void TrimPathToFirstGearChange(
      std::vector<pnc::geometry_lib::PathPoint>& complete_path_point_vec,
      std::vector<pnc::geometry_lib::PathPoint>& current_path_point_vec);
  bool IsContinuousSTurn(
      const std::vector<pnc::geometry_lib::PathPoint>& path_points,
      int& start_idx, int& end_idx);
  bool SimplifyComplexSTurn(std::vector<pnc::geometry_lib::PathPoint>& points);
  bool IsPointDuplicate(const Eigen::Vector2d& point,
                        const std::vector<Eigen::Vector2d>& existing_points,
                        double tolerance = 1e-3);
  std::vector<Eigen::Vector2d> FilterByLightVisibilityMultiPoints(
      const std::vector<Eigen::Vector2d>& filtered_channel_obs_vec,
      const Eigen::Vector2d& veh_pos, double yaw, double angle_resolution = 0.5,
      double start_angle = -180.0, double end_angle = 180.0,
      double max_range = 12.0, double max_neighbor_dist = 0.5,
      int max_points_per_bin = 5);
  std::vector<Eigen::Vector2d> FilterByLightVisibilityDualAxle(
      const Eigen::Vector2d& rear_axle_pos, const double yaw,
      const std::vector<Eigen::Vector2d>& filtered_channel_obs_vec,
      double angle_resolution = 5.0, int max_points_per_bin = 5,
      double start_angle = -180.0, double end_angle = 180.0,
      double max_range = 12.0, double duplicate_tolerance = 1e-3);
  bool IsReplayRequest();
  std::vector<Eigen::Vector2d> ProcessCurbPointsAndGetNearestAbsY(
      double& nearest_abs_y,
      const std::unordered_map<size_t, std::vector<Eigen::Vector2d>>&
          same_parent_id_curb_obs,
      const double curb_c_x, const double curb_d_x,
      const std::vector<double>& y_of_c_d_curb_y);
  bool ProcessCurbPointsAndGetPoints(
      std::vector<Eigen::Vector2d>& point_set,
      const std::unordered_map<size_t, std::vector<Eigen::Vector2d>>&
          obs_id_pt_map,
      const Eigen::Vector2d& C_curb, const Eigen::Vector2d& D_curb,
      const std::vector<double>& y_of_c_d_curb_y);
  void ProcessTruncationPoints(std::vector<Eigen::Vector2d>& curb_points,
                               const double curb_y,
                               pnc::geometry_lib::LineSegment& tlane_line);
  void AddMultiFrameResult(const PathPlannerResult& result);
  bool CalcMultiFrameResult();

  std::unordered_map<size_t, std::vector<Eigen::Vector2d>> obs_id_pt_map_;

 private:
  // virtual func
  void CalStaticBufferInDiffSteps(double& lat_buffer,
                                  double& safe_uss_remain_dist) const;

  void CalDynamicBufferInDiffSteps(double& dynaminc_lat_buffer,
                                   double& dynamic_lon_buffer) const;

  const bool CheckOneReverseToSlot();
  bool CheckFirstReverseReplan(const pnc::geometry_lib::PathPoint& cur_pose,
                               const pnc::geometry_lib::PathSegGear gear,
                               const double slot_length);
  bool CheckFirstDriveReplanParallel(const Eigen::Vector2d& cur_pose,
                                     const pnc::geometry_lib::PathSegGear gear,
                                     Eigen::Vector2d slot_01_mid);
  bool CheckReplanParallel();
  const ParallelPathGenerator& SuitablePathReplan();
  const bool CheckLastPathCollided();
  const int CalGearChangeNumForOutPath(
      const std::vector<pnc::geometry_lib::PathSegment>& in_path_seg_vec,
      const std::vector<uint8_t>& in_gear_vec);
  const ParallelPathGenerator& UseOrNotUseLastPath();
  void CheckEgoPoseWhenPlanFaild(ParkingFailReason reason);

  const double UpdateRemainDistObs(const double remain_dist_path,
                                   const double remain_dist_obs);
  const bool PostProcessPathPara();

  const bool CheckEgoToSlotRelation();

  void UpdatePADirection();

  const bool PostProcessToZigZagPath();
  void SetReplanAgainBit(const DynamicReplanStatus status) {
    parallel_replan_again_bit_ |= 1U << uint16_t(status);
    return;
  }
  template <typename... Rest>
  void SetReplanAgainBit(DynamicReplanStatus first, Rest... rest) {
    SetReplanAgainBit(first);
    SetReplanAgainBit(rest...);
  }
  bool GetReplanAgainBit(DynamicReplanStatus status) const {
    return (parallel_replan_again_bit_ &
            (1U << static_cast<uint16_t>(status))) != 0;
  }

  template <typename... Rest>
  bool GetReplanAgainBit(DynamicReplanStatus first, Rest... rest) const {
    return GetReplanAgainBit(first) || GetReplanAgainBit(rest...);
  }

  void ClearReplanAgainBit(DynamicReplanStatus status) {
    parallel_replan_again_bit_ &= ~(1U << static_cast<uint16_t>(status));
  }

  Tlane t_lane_;
  std::unordered_map<size_t, std::vector<Eigen::Vector2d>> obs_pt_local_vec_;
  ParallelPathGenerator parallel_path_planner_;
  ParallelPathGenerator previous_parallel_path_planner_;
  uint16_t parallel_replan_again_bit_ = 0;
  std::vector<pnc::geometry_lib::PathPoint>
      previous_current_path_point_global_vec_;
  std::deque<double> previous_remain_dist_obs;
  bool enable_pa_park_ = false;
  ApaSlot first_plan_slot;
  Eigen::Vector2d first_line_coeffs_;
  double slot2curb_dist_;
  pnc::geometry_lib::PathPoint first_plan_cur_pos;
  std::unordered_map<size_t, PaTryResult> multi_parkin_path_vec_;
  std::unordered_map<ApaPADirection, std::vector<double>>
      pa_mean_move_dist_map_;
  iflyauto::APAHMIData last_apa_hmi_;
  // success to failed nums
  int pa_try_s2f_count_ = 0;
  int pa_try_success_count_ = 0;

  RelativeLocObserverManager relative_loc_observer_manager_;
  std::unordered_map<size_t, std::set<AngleResult>> try_bound_map_;
  bool delay_check_finish_ = false;

  std::unordered_map<size_t, int> parent_total_count;
  std::unordered_map<size_t, int> parent_height_count;
  std::unordered_map<int, std::vector<AngleResultHeightObs>> multi_frame_height_obs_map_;
  bool last_frame_limiter_valid_ = false;
  bool out_again_path_better_ = false;
  std::vector<std::vector<pnc::geometry_lib::PathSegment>>
      path_in_slot_for_request_;
  std::vector<std::vector<pnc::geometry_lib::PathSegment>>
      path_in_slot_for_with_path_;

 protected:
  RequestResponseState thread_state_;
  HybridAStarThreadSolver thread_;
  // do not clear it every frame in cruise state.
  AstarResponse response_;

  AstarPathGear current_gear_ = AstarPathGear::PARKING;
  ParkObstacleList obs_hastar_;
  VirtualWallDecider virtual_wall_decider_;
  AstarSearchState astar_state_ = AstarSearchState::NONE;
  ReplanReason last_replan_reason_ = ReplanReason::NOT_REPLAN;
  bool used_last_path_ = false;
  std::vector<PathPlannerResult> multi_frame_result_;
};

}  // namespace apa_planner
}  // namespace planning

#endif
