// meb_scenario_base.h
#ifndef MEB_SCENARIO_BASE_H_
#define MEB_SCENARIO_BASE_H_
#include "../meb_preprocess.h"
#include "adas_function_context.h"
#include "adas_function_lib.h"
using namespace planning;
namespace adas_function {

class MebScenarioBase {
 public:
  MebScenarioBase(){};
  virtual ~MebScenarioBase() = default;
  // MebCore 作为调度者（Caller），在调用子类的 Process
  // 方法时，把需要的数据作为参数传进去。
  virtual void Process(void) = 0;

  virtual void CollisionCalculate(double stop_distance_buffer_reduction);

  virtual uint64_t FalseTriggerStratege(const MebTempObj obj) = 0;

  virtual int SceneCode(void) = 0;

  virtual int SelcetInterestObject(MebTempObj &temp_obj) = 0;

  const std::vector<double> GetOdBufferTable(
      const adas_function::context::Parameters &param, const OdObjGroup group,
      const bool is_reverse, const bool is_static, const bool park_mode);
  // 障碍物来源未知，interest_code_大小不确定
  InterestObjInfo interest_obj_info_;
  InterestObjInfo collision_obj_info_;
  InterestObjInfo final_collision_obj_info_;
  bool brake_alert_ = false;
  int scene_code_ = -1;

 protected:
  virtual void Init(void) = 0;
  // 0:满足场景限制,可以进行目标筛选

  BoxCollisonLib box_collision_;
};
}  // namespace adas_function
#endif  // MEB_SCENARIO_BASE_H_