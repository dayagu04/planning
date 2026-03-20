#pragma once

#include <array>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "apa_measure_data_manager.h"
#include "apa_obstacle.h"
#include "apa_state_machine_manager.h"
#include "geometry_math.h"
#include "local_view.h"
#include "session.h"
#include "src/modules/apa_function/apa_param_config.h"

namespace planning {
namespace apa_planner {
// obstacle manager: 管理所有障碍物，包括超声波、限位器、视觉、虚拟
// apa_obstacle: 是泊车障碍物的基本数据结构.

#define HAVE_3D_OBS_INTERFACE (1)

class ApaObstacleManager final {
 public:
  ApaObstacleManager() {}
  ~ApaObstacleManager() {}

  void Update(const LocalView *local_view,
              const iflyauto::PlanningOutput *planning_output,
              const ApaStateMachineManager &state_machine_manager,
              const size_t ego_slot_id);

  void Reset() {
    ResetSingleFrameObs();
    ResetObstacleODTracking();
  }

  void ResetSingleFrameObs() {
    obs_id_generate_ = 0;
    obstacles_.clear();
    uss_dis_vec_.clear();
  }

  void ResetObstacleODTracking() {
    fusion_id_to_local_id_.clear();
    obs_tracking_id_generate_ = 1;
    obstacles_od_tracking_.clear();
  }

  const std::unordered_map<size_t, ApaObstacle> &GetObstacles() const {
    return obstacles_;
  }

  const std::unordered_map<size_t, ApaObstacle> &GetObstaclesODTracking() const {
    return obstacles_od_tracking_;
  }

  std::array<float, 2> GetParallelSlotNeighbourObjsHeading() const {
    return parallel_slot_neigbour_objs_heading_;
  }

  std::unordered_map<size_t, ApaObstacle> &GetMutableObstacles() {
    return obstacles_;
  }

  std::unordered_map<size_t, ApaObstacle> &GetMutableObstaclesODTracking() {
    return obstacles_od_tracking_;
  }

  const size_t GetObsIdGenerate() const { return obs_id_generate_; }

  void AddObstacle(const ApaObstacle &apa_obs) {
    obstacles_[obs_id_generate_] = apa_obs;
    obs_id_generate_++;
  }

  void SetObstacle(const size_t id, const ApaObstacle &apa_obs);

  const bool GetObstacle(const size_t id, ApaObstacle *obs);

  const bool GetObstacle(const ApaObsAttributeType type,
                         std::vector<ApaObstacle *> &obs_vec);

  const std::vector<double> &GetUssDisVec() { return uss_dis_vec_; }

  void TransformCoordFromGlobalToLocal(
      const pnc::geometry_lib::GlobalToLocalTf &g2l_tf);

  const bool IsOccType(const iflyauto::ObjectType type);
  const bool NeedTrackingObjectType(const iflyauto::ObjectType type);

  void GenerateObsByOD(const LocalView *local_view,
                       const ObjectDetectObsConfig &od_config);
  void GenerateObsByODTracking(const LocalView *local_view,
                       const ObjectDetectObsConfig &od_config);

  const size_t GetObstacleSize() const { return obstacles_.size(); }

  void GenerateUss(const LocalView *local_view);
  std::pair<int, float> CheckParaSlotObsAreNeighbour(
      iflyauto::Obstacle obs, const iflyauto::ParkingFusionSlot *slot,
      Eigen::Vector3d ego_pose);
  static std::pair<int, float> CheckParaSlotObsPtsAreNeighbour(
      std::array<Eigen::Vector2d, 4> obs_vex_pts_in_global,
      const iflyauto::ParkingFusionSlot *slot,
      const std::array<double, 4> d_per_edge, Eigen::Vector3d ego_pose);

  void UpdateObstacleODLostFrames(
      const std::unordered_set<size_t> &current_ids);

 private:
  // not allow any ptr variable
  size_t obs_id_generate_{0};
  std::unordered_map<size_t, ApaObstacle> obstacles_;
  std::vector<double> uss_dis_vec_;

  std::unordered_map<size_t, size_t> fusion_id_to_local_id_;
  size_t obs_tracking_id_generate_{0};
  std::unordered_map<size_t, ApaObstacle> obstacles_od_tracking_;

  ApaStateMachineManager state_machine_manager_;

  // record localization pose in activate state
  LocalizationPath localization_path_;
  bool ego_slot_is_parallel_ = false;
  const iflyauto::ParkingFusionSlot *last_ego_slot_;

  std::array<float, 2> parallel_slot_neigbour_objs_heading_ = {-100, -100};
};

typedef IndexedList<int, ApaObstacle> IndexedParkObstacles;
typedef ThreadSafeIndexedList<int, ApaObstacle> ThreadSafeParkObstacles;

}  // namespace apa_planner
}  // namespace planning
