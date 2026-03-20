#ifndef __PLAN_DATA_H__
#define __PLAN_DATA_H__

#include <cstdint>

#include "planning_debug_info.pb.h"
#include "planning_hmi_c.h"
#include "planning_plan_c.h"

namespace planning {
namespace plan_interface {
struct PlanningOutputData {
  iflyauto::PlanningOutput planning_output;
  common::PlanningDebugInfo planning_debug_info;
  iflyauto::PlanningHMIOutputInfoStr planning_hmi_info;

  void Reset() {
    // planning_output.Clear();
    planning_debug_info.Clear();
    // planning_hmi_info.Clear();
    memset(&planning_output, 0, sizeof(planning_output));
    memset(&planning_hmi_info, 0, sizeof(planning_hmi_info));
  }
};

enum PlannerInterfaceType {
  APA_PLANNER,
  // SCC_PLANNER,
  COUNT,
  NONE_PLANNER,
};

struct PlanningStateData {
  uint64_t frame_num = 0;
  uint8_t planner_interface_type = PlannerInterfaceType::NONE_PLANNER;
  uint8_t scene_type = planning::common::SceneType::NOT_DEFINED;
};

struct PlanningWorldData {};

struct LongitudinalBehaviorData {};

struct LateralBehaviorData {};

struct LongitudinalMotionData {};

struct LateralMotionData {};

class PlanData {
 public:
  struct Data {
    PlanningOutputData output_data{};
    PlanningStateData state_data{};
    PlanningWorldData world_data{};
    LongitudinalBehaviorData lon_bp_data{};
    LateralBehaviorData lat_bp_data{};
    LateralMotionData lat_mp_data{};
    LongitudinalMotionData lon_mp_data{};
  };

  const Data &GetData() const { return plan_data_; }
  Data &MutableData() { return plan_data_; }

  void Reset();

 private:
  Data plan_data_;
};

}  // namespace plan_interface
}  // namespace planning

#endif
