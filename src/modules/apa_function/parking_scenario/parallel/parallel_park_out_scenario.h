#pragma once

#include <memory>
#include <string>

#include "apa_world.h"
#include "parallel_out_path_generator.h"
#include "parking_scenario.h"

namespace planning {
namespace apa_planner {

class ParallelParkOutScenario : public ParkingScenario {
 public:
  ParallelParkOutScenario() = default;
  ParallelParkOutScenario(const std::shared_ptr<ApaWorld>& apa_world_ptr) {
    SetApaWorldPtr(apa_world_ptr);
  }

  virtual void Reset() override;
  virtual void ExcutePathPlanningTask() override;
  virtual const bool GenTlane() override;
  virtual const bool CheckFinished() override;
  virtual const uint8_t PathPlanOnce() override;
  virtual const bool UpdateEgoSlotInfo() override;
  void ScenarioTry() override;
  bool ParkOutDirectionTry();

  void GenTBoundaryObstacles();

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
  bool CheckFinishParallel();

 private:
  void Log() const override;
  virtual const bool GenObstacles() override;

  void CalStaticBufferInDiffSteps(double& lat_buffer,
                                  double& safe_uss_remain_dist) const;

  void CalDynamicBufferInDiffSteps(double& dynaminc_lat_buffer,
                                   double& dynamic_lon_buffer) const;

 private:
  Tlane t_lane_;
  std::vector<Eigen::Vector2d> obs_pt_local_vec_;
  ParallelOutPathGenerator parallel_out_path_planner_;
  // std::vector<bool> multi_parkout_direction; // 0: left front, 1: right front, 2: left back, 3: right back
  std::unordered_map<ApaParkOutDirection, bool> multi_parkout_direction;
  std::unordered_map<ApaParkOutDirection,
      std::vector<pnc::geometry_lib::PathPoint>> multi_parkout_path_vec;
  ApaParkOutDirection parkout_direction_ = ApaParkOutDirection::INVALID;
};
}  // namespace apa_planner
}  // namespace planning
