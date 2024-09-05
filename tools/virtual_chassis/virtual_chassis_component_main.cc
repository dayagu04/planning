#include "config/vehicle_param.h"
#include "cyber/cyber.h"
#include "log_glog.h"
#include "pose2d.h"
#include "src/common/ifly_time.h"
#include "src/common/utils_math.h"
#include "tools/visualization2d/system_state.h"
#include "tools/visualization2d/viz_window.h"
#include "virtual_chassis.h"
#include "virtual_chassis_component.h"

// using apollo::cyber::Rate;
using apollo::cyber::Time;
using namespace planning;

/** virutal localization
 * virtual chassis
 *
 * planner get virtual localization,virtual chasis;
 */

int main(int argc, char** argv) {
  // google::ParseCommandLineFlags(&argc, &argv, true);

  // module state
  SystemState module_state;

  FilePath::SetName("virtual_chassis");
  InitGlog(FilePath::GetName().c_str());

  // init cyber framework
  apollo::cyber::Init(FilePath::GetName().c_str());

  SystemStateInit(module_state);

  // viz config
  viz2d_window_read_config("./../modules");

  // 底盘初始化，定位初始化。对于仿真而言，需要模拟定位数据、底盘数据、感知数据。
  // 而规划、控制则是根据这些数据来计算的
  Pose2D start_point;

  start_point.x = 0.0;
  start_point.y = 2.0;
  start_point.theta = M_PI_2;

  // 虚拟底盘
  VirtualChassisComponent chassis_component;

  double interval_time_ms;

  // 10 ms, 保持和控制的频率一致
  interval_time_ms = 20.0;

  VehicleParam veh_param;

  chassis_component.Init(veh_param, start_point, interval_time_ms / 1000.0);

  running_time_debug_info();

  double history_time;
  double delta_time;
  history_time = apollo::cyber::Time::Now().ToSecond();

  while (apollo::cyber::OK()) {
    const RunningTimeDebug* debug = get_debug_info();

    if (debug->pause_debug.enabled) {
      continue;
    }

    double cur_time = apollo::cyber::Time::Now().ToSecond();
    delta_time = (cur_time - history_time) * 1000.0;

    if (delta_time < interval_time_ms) {
      continue;
    }

    ILOG_INFO << "chassis frame interval (ms): " << delta_time;

    chassis_component.Process();

    // 底盘状态,发送出去
    chassis_component.PublishMessages();

    history_time = cur_time;
  }

  // terminate
  apollo::cyber::Clear();
  // google::ShutDownCommandLineFlags();

  StopGlog();

  return 0;
}
