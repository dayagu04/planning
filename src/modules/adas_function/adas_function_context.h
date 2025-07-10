#ifndef __ADAS_FUNCTION_CONTEXT_H__
#define __ADAS_FUNCTION_CONTEXT_H__

#include "adas_function_struct.h"
#include "common/config_context.h"
#include "environmental_model.h"
#include "planning_context.h"
#include "session.h"

namespace adas_function {
namespace context {

// 定义单例
class AdasFunctionContext {
 public:
  static AdasFunctionContext &GetInstance() {
    static AdasFunctionContext instance;
    return instance;
  }

  Parameters *mutable_param() { return &param_; }
  const Parameters *const get_param() { return &param_; }

  StateInfo *mutable_state_info() { return &state_info_; }
  const StateInfo *const get_state_info() { return &state_info_; }

  RoadInfo *mutable_road_info() { return &road_info_; }
  const RoadInfo *const get_road_info() { return &road_info_; }

  LastCycleInfo *mutable_last_cycle_info() { return &last_cycle_info_; }
  const LastCycleInfo *const get_last_cycle_info() { return &last_cycle_info_; }

  ObjectsInfo *mutable_objs_info() { return &objs_info_; }
  const ObjectsInfo *const get_objs_info() { return &objs_info_; }

  // 标识牌信息
  TsrInfo *mutable_tsr_info() { return &tsr_info_; }
  const TsrInfo *const get_tsr_info() { return &tsr_info_; }

  iflyauto::Trajectory *mutable_lka_trajectory_info() {
    return &lks_trajectory_;
  }
  const iflyauto::Trajectory *const get_lka_trajectory_info() {
    return &lks_trajectory_;
  }

  void set_session_mutable(planning::framework::Session *session) {
    session_ = session;
  }
  planning::framework::Session *mutable_session() { return session_; }
  const planning::framework::Session *const get_session() { return session_; }

  AdasOutputInfo *mutable_output_info() { return &output_info_; }
  const AdasOutputInfo *const get_output_info() { return &output_info_; }

 private:
  AdasFunctionContext(){};

  planning::framework::Session *session_ = nullptr;

  Parameters param_;

  StateInfo state_info_;

  RoadInfo road_info_;

  LastCycleInfo last_cycle_info_;

  TsrInfo tsr_info_;

  AdasOutputInfo output_info_;

  ObjectsInfo objs_info_;
  // Local坐标系下的目标轨迹
  iflyauto::Trajectory lks_trajectory_;
};

}  // namespace context
}  // namespace adas_function
#endif