#pragma once

#include <memory>
#include <string>

#include "apa_world.h"
#include "parallel_out_path_generator.h"
#include "parking_scenario.h"
#include "parallel_park_in_scenario.h"

namespace planning {
namespace apa_planner {

class ParallelParkOutScenario : public ParallelParkInScenario {
 public:
  ParallelParkOutScenario() = default;
  ParallelParkOutScenario(const std::shared_ptr<ApaWorld>& apa_world_ptr) {
    SetApaWorldPtr(apa_world_ptr);
    if (apa_param.GetParam().parallel_enable_hybrid_astar) {
      VehicleParam vehicle_param;
      UpdateVehicleParam(vehicle_param);
      thread_.Init(vehicle_param);
      thread_.Start();
      response_.Clear();
    }
  }

  virtual void Reset() override;
  virtual void ExcutePathPlanningTask() override;
  virtual const bool GenTlane() override;
  virtual const bool CheckFinished() override;
  virtual const uint8_t PathPlanOnce() override;
  virtual const bool UpdateEgoSlotInfo() override;
  void ScenarioTry() override;
  bool ParkOutPlanTry();
  bool ParkOutDirectionTry();
  bool ParkOutDirectionTryHybridAStar();

  void GenTBoundaryObstacles();

  const PathPlannerResult PathPlanOnceGeometry();

  void SetReleaseDirection(iflyauto::APAHMIData& apa_hmi_data,
                           const AstarRequest& cur_request);
  void SetRequestForScenarioTry(AstarRequest& cur_request,
                                const EgoInfoUnderSlot& ego_info);

  void SetTargetGroup(AstarRequest& cur_request);
  void UpdateStartPosByInSlot(Pose2f& start_pos,
                              pnc::geometry_lib::PathSegGear& last_path_gear);
  void UpdatePathByGeometry();
  const bool SetAstarRequest(AstarRequest& astar_request);
  const PathPlannerResult PubResponseForScenarioTry(
      const EgoInfoUnderSlot& ego_info, const ParkObstacleList& obs);
  const PathPlannerResult PubResponseForScenarioRunning(
      const EgoInfoUnderSlot& ego_info, const ParkObstacleList& obs);
  const PathPlannerResult PathPlanOnceHybridAStar();
  void UpdatePostProcessStatus(PathPlannerResult pathplan_result);

  const double CalcSlotOccupiedRatio(
      const pnc::geometry_lib::PathPoint start_pose) const;

  const Tlane& GetTlane() const { return t_lane_; }
  const ParallelOutPathGenerator& GetPathPlanner() const {
    return parallel_out_path_planner_;
  }

  ParallelOutPathGenerator* GetMutablePathPlanner() {
    return &parallel_out_path_planner_;
  }

  virtual const double CalRealTimeBrakeDist() override;

  void CheckEgoPoseWhenParkOutFaild(ParkingFailReason reason);

 private:
  void Log() const override;
  virtual const bool GenObstacles() override;

  void CalStaticBufferInDiffSteps(double& lat_buffer,
                                  double& safe_uss_remain_dist) const;

  void CalDynamicBufferInDiffSteps(double& dynaminc_lat_buffer,
                                   double& dynamic_lon_buffer) const;
  const bool PostProcessPathPara();  void JudgeArcSlot();
  const bool IsNearPathEnd();
 private:
  Tlane t_lane_;
  std::unordered_map<size_t, std::vector<Eigen::Vector2d>> obs_pt_local_vec_;
  ParallelOutPathGenerator parallel_out_path_planner_;
  // std::vector<bool> multi_parkout_direction; // 0: left front, 1: right
  // front, 2: left back, 3: right back
  std::unordered_map<ApaParkOutDirection, bool> multi_parkout_direction;
  std::unordered_map<ApaParkOutDirection,
                     std::vector<pnc::geometry_lib::PathPoint>>
      multi_parkout_path_vec;
  ApaParkOutDirection parkout_direction_ = ApaParkOutDirection::INVALID;
  bool is_try_tlane_ = false;
  std::vector<pnc::geometry_lib::PathPoint>
      previous_current_path_point_global_vec_;
  bool delay_check_finish_ = false;
  bool is_last_pose_set_ = false;
  pnc::geometry_lib::PathPoint last_target_pose_;

  double arc_slot_init_out_heading_ = 0.0;
  bool is_outer_arc_slot_ = false;
  bool is_arc_slot_ = false;
  bool path_end_heading_is_met_= false;
  ParallelOutPathGenerator previous_parallel_out_path_planner_;
  double strict_channel_y = 6.5;
};
}  // namespace apa_planner
}  // namespace planning
