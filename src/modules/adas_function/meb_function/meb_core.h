#ifndef MEB_CORE_H_
#define MEB_CORE_H_
#include "adas_function_context.h"
#include "adas_function_lib.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "meb_box_collision_lib.h"
#include "meb_preprocess.h"
#include "scenarios/occ_straight_scenario.h"
#include "scenarios/od_crossing_scenario.h"
#include "scenarios/od_straight_scenario.h"
#include "scenarios/uss_straight_scenario.h"
using namespace planning;
namespace adas_function {
namespace meb_core {

enum FirstState {
  INIT = 0,     // Init
  OFF = 1,      // Off
  STANDBY = 2,  // Standby
  ACTIVE = 3,   // Active
  FAULT = 4,    // Fault
};

enum SecondtState {
  Intervertion = 0,
  NoIntervertion = 1,
  Suppression = 2,
};

struct StateMachineInfo {
  bool state_machine_init_flag = false;
  FirstState first_state = INIT;
  SecondtState second_state = Intervertion;
  uint32 enable_code = 0;
  uint32 disable_code = 0;
  uint32 fault_code = 0;
  uint32 kickdown_code = 0;
  uint32 supp_code = 0;
};

class MebCore {
 public:
  MebCore() { InitOnce(); }
  ~MebCore() = default;
  void RunOnce(void);
  bool meb_intervention_flag_;
  StateMachineInfo meb_state_info_;
  iflyauto::MEBFunctionFSMWorkState meb_state_;
  double meb_brake_duration_;
  double meb_no_response_duration_;
  double meb_hold_duration_;
  double meb_cooling_time_remain_;
  long long meb_index_;

 private:
  void InitOnce(void);
  // Preprocess
  std::shared_ptr<adas_function::OdStraightScenario> od_straight_scenario_ptr_;
  std::shared_ptr<adas_function::OccStraightScenario>
      occ_straight_scenario_ptr_;
  std::shared_ptr<adas_function::UssStraightScenario>
      uss_straight_scenario_ptr_;
  std::shared_ptr<adas_function::OdCrossingScenario> od_crossing_scenario_ptr_;
  void UpdateMebEnableCode(void);
  void UpdateMebDisableCode(void);
  void UpdateMebFaultCode(void);
  void UpdateMebKickdownCode(void);
  void UpdateMebSuppCode(void);
  void MebStateMachine(void);
  void SetMebOutputInfo(void);
  void Log(void);
};

}  // namespace meb_core
}  // namespace adas_function
#endif