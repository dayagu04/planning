#include "log_glog.h"
#include "src/common/ifly_time.h"
#include "viz2d_component.h"
#include "viz_window.h"

// using apollo::cyber::Rate;

/** virutal localization
 * virtual chasis
 *
 * planner get virtual localization,virtual chasis;
 */

int main(int argc, char** argv) {
  // apollo::cyber::Clock::SetMode(apollo::cyber::proto::MODE_MOCK);
  // apollo::cyber::Clock::SetNowInSeconds(curr_time_stamp);
  // todo, 将来所有数据的时钟都需要改称apollo cyber时钟

  // google::ParseCommandLineFlags(&argc, &argv, true);

  FilePath::SetName("display2d");
  InitGlog(FilePath::GetName().c_str());

  // init cyber framework
  apollo::cyber::Init(FilePath::GetName().c_str());

  // module state
  planning::SystemState module_state;
  SystemStateInit(module_state);

  // init apollo module
  std::string base_dir = "./../modules";
  // viz2d_window_read_config(base_dir);

  // FLAGS_vehicle_config_path =
  //         "./../modules/common/data/vehicle_param.pb.txt";

  // FLAGS_vehicle_model_config_filename = "./../modules/common/vehicle_model/"
  //                                       "conf/vehicle_model_config.pb.txt";

  // reader

  // writter

  int ret_value;

  // running_time_debug_info();

  // vis
  planning::Viz2dComponent viz2d;
  viz2d.Init();

  int key_value;

  // 10毫秒一个周期
  double interval_time = 20.0;
  double history_time;
  double current_time;
  double delta_time;
  history_time = apollo::cyber::Time::Now().ToSecond();

  // common::VehicleConfig vehicle_config_;

  //
  // vehicle_config_ = apollo::common::VehicleConfigHelper::GetConfig();

  double max_steering_wheel_angle_ = 26;
  // vehicle_config_.vehicle_param().max_steer_angle();

  max_steering_wheel_angle_ = max_steering_wheel_angle_ * 180 / M_PI;

  while (apollo::cyber::OK()) {
    double cur_time = apollo::cyber::Time::Now().ToSecond();
    delta_time = (cur_time - history_time) * 1000.0;

    if (delta_time < interval_time) {
      continue;
    }

    ILOG_INFO << "frame interval (ms): " << delta_time;

    viz2d.Process(max_steering_wheel_angle_);

    // const apollo::RunningTimeDebug *debug = apollo::get_debug_info();

    // if (debug->pause_debug.enabled)
    // {
    //     continue;
    // }
    history_time = cur_time;
  }

  // terminate
  viz2d.Close();
  apollo::cyber::Clear();

  StopGlog();

  // google::ShutDownCommandLineFlags();

  return 0;
}
