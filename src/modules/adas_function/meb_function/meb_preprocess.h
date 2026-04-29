#ifndef MEB_PREPROCESS_H_
#define MEB_PREPROCESS_H_
#include "adas_function_context.h"
#include "adas_function_lib.h"
#include "meb_box_collision_lib.h"

#define MEB_VEHICLE_SERVICE_FRAME_SIZE 50
#define MEB_CYCLE_TIME_SEC 0.1
#define OBJECT_LOGGER_MAX_RECORDS 50

using namespace planning;
namespace adas_function {

// 障碍物循环记录器结构体
struct ObjectLoggerStr {
  std::vector<iflyauto::FusionObjectsInfo> objects;  // 记录数组
  int current_index;                                 // 当前写入索引
  bool is_full;                                      // 是否已满
};

struct SelfVehicleServiceFrameInfo {
  uint64 stamp;          // 消息发送时刻 (微秒)
  double vehicle_speed;  // m/s
  uint8 vehicle_speed_display_kph;
  double ego_acceleration;
  double steering_angle;  // rad
  double steering_angle_speed;
  double yaw_rate;
  bool yaw_rate_available;
  double yaw_rate_jerk;
  bool yaw_rate_jerk_available;
  double accelerator_pedal_pos;
  bool brake_pedal_pressed;
  double brake_pedal_pos;
  int32 start_turning_index;
  int32 turning_count;
  int32 straight_count;
  uint64 start_turning_timestamp;
  uint64 start_straight_running_timestamp;
  uint64 latest_straight_running_timestamp;
  bool need_update_backright;
  bool driver_linear_state;
  bool driver_linear_state_available;
  bool drive_slow_down;
};
struct MebFunctionInput {
  bool meb_main_switch;  // meb主开关 false:关闭 true:打开
  double ego_radius;
  double ego_curvature;
  double signed_ego_vel_mps;  // 本车速度 单位：m/s D挡为正 ，R挡 为负
  double shift_direction_index;
  double esp_active_time = 0.0;
  double brake_pedal_pos_rate = 0.0;
  bool park_mode = false;
  std::array<SelfVehicleServiceFrameInfo, MEB_VEHICLE_SERVICE_FRAME_SIZE>
      single_frame_vehicle_service;
  // 头部索引：指向下一个写入位置 (初始化为0)
  size_t history_head_idx_ = 0;
  // 满载标志：标记 buffer 是否已经写满过至少一轮 (初始化为false)
  bool history_full_ = false;

  ObjectLoggerStr obj_logger_;
};

struct RearKeyObjStr {
  int key_obj_index;
  int key_obj_id;
  double key_obj_relative_x;
  double key_obj_relative_y;
  double key_obj_relative_heading_angle;
  double key_obj_relative_v_x;
  double key_obj_relative_v_y;
  double key_obj_relative_lon_acc;
  double key_obj_relative_lat_acc;
  double key_obj_length;
  double key_obj_width;
  double obj_absolute_v_x;
  double obj_absolute_v_y;
};

struct FrontRadarKeyObjStr {
  int key_obj_index;
  int key_obj_id;
  double key_obj_relative_x;
  double key_obj_relative_y;
  double key_obj_relative_heading_angle;
  double key_obj_relative_v_x;
  double key_obj_relative_v_y;
  double key_obj_relative_lon_acc;
  double key_obj_relative_lat_acc;
  double key_obj_length;
  double key_obj_width;
  double obj_absolute_v_x;
  double obj_absolute_v_y;
};

struct FrontCipvKeyObjStr {
  int key_obj_index;
  int key_obj_id;
  double key_obj_relative_x;
  double key_obj_relative_y;
  double key_obj_relative_heading_angle;
  double key_obj_relative_v_x;
  double key_obj_relative_v_y;
  double key_obj_relative_lon_acc;
  double key_obj_relative_lat_acc;
  double key_obj_length;
  double key_obj_width;
  double obj_absolute_v_x;
  double obj_absolute_v_y;
};

struct MebParameters {
  // meb 内置参数

  uint8 enable_vehspd_display_kph_min = 4.0;    // 激活的最小仪表车速
  uint8 enable_vehspd_display_kph_max = 10.0;   // 激活的最大仪表车速
  uint8 disable_vehspd_display_kph_min = 4.0;   // 退出的最小仪表车速
  uint8 disable_vehspd_display_kph_max = 10.0;  // 退出的最大仪表车速
  double predict_t = 5.0;
  // double earliest_warning_line = 1.5;  // 触发的最早报警线，单位：m
  // double latest_warning_line = -0.3;   // 触发的最晚报警线，单位：m
  // double reset_warning_line = 0.15;    // 触发的报警重置线，单位：m
  // double supp_turn_light_recovery_time = 2.0;  // 转向灯抑制恢复时长，单位：s
  // double warning_time_min = 1.0;  // 单次最大报警时长，单位：s
  // double warning_time_max = 2.0;  // 单次最大报警时长，单位：s
  double brake_pedal_pos = 20.0;

  /*
    /-1----2----3----4-\
   /                    \
  0                      5
  |                      |
  |                      |
  |                      |
  |                      |
  11                     6
   \                    /
    \-10---9----8----7-/

  */

  std::vector<double> uss_radar_x_vector = {3.245,  3.58,   3.725, 3.725,
                                            3.58,   3.245,  -0.5,  -0.96,
                                            -1.064, -1.064, -0.96, -0.5};
  std::vector<double> uss_radar_y_vector = {0.941,  0.694,  0.331,  -0.331,
                                            -0.694, -0.941, -0.912, -0.731,
                                            -0.328, 0.328,  0.731,  0.912};
  double uss_rabar_angle_range_rad = 0.5;
  std::vector<int> uss_pos_index_vector = {0, 1, 2, 3, 4,  5,
                                           6, 7, 8, 9, 10, 11};
  bool dynamic_uss_collision_switch = true;
  double dynamic_uss_collision_ttc = 1.5;
};

class MebPreprocess {
 public:
  // 1. 获取单例的静态方法
  static MebPreprocess& GetInstance() {
    static MebPreprocess instance;  // 局部静态变量，C++11 保证线程安全初始化
    return instance;
  }
  // 2. 禁用拷贝构造和赋值操作符
  MebPreprocess(const MebPreprocess&) = delete;
  MebPreprocess& operator=(const MebPreprocess&) = delete;

  ~MebPreprocess() = default;

  void UpdateMebInput(void);
  const MebFunctionInput& GetMebInput() const { return meb_input_; };
  const MebParameters& GetMebParam() const { return meb_param_; };

  RearKeyObjStr GetRearKeyObjInfo() { return rear_key_obj_info_; };

  void UpdateSingleFrameVehicleService(void);
  const SelfVehicleServiceFrameInfo* GetHistoryFrame(size_t delay_cnt) const;

  void UpdateObjectLogger(void);
  const iflyauto::FusionObjectsInfo* GetHistoryObjectLogger(
      size_t delay_cnt) const;

  bool CheckDriveLinear(double turn_yaw_vel_threshold,
                        double linear_steer_angle_diff,
                        int linear_ego_length_threshold);
  bool CheckSpeedReduction(double speed_reduction_threshold,
                           int32 check_length_nums);

  //  * @brief 检查指定障碍物历史帧中的横向和纵向速度是否稳定（基于趋势检测）
  //  * @param object_logger 记录器结构体指针
  //  * @param self_vehicle_service_frames 自车历史服务信息帧数组
  //  * @param track_id 目标障碍物的track_id
  //  * @param check_frames 检查的历史帧数
  //  * @param velocity_x_threshold
  //  纵向速度变化阈值（米/秒），相邻帧速度变化超过该值才计入趋势判断
  //  * @param velocity_y_threshold
  //  横向速度变化阈值（米/秒），相邻帧速度变化超过该值才计入趋势判断
  //  * @return TRUE:速度稳定（无明显持续增大或减小趋势）,
  //  FALSE:速度不稳定（有持续变化趋势）或数据不足
  bool ObjectLoggerCheckVelocityStability(int track_id, int check_frames,
                                          double velocity_x_threshold,
                                          double velocity_y_threshold,
                                          double speed_angle_thres);

  // 判断障碍物在历史帧中是否一直处于车宽边缘
  bool ObjectLoggerCheckHistoryWithinEgoWidth(int track_id, int check_frames,
                                              double half_ego_width);
  double FindClosestVehicleSpeed(uint64 target_timestamp,
                                 uint16 max_search_frames);

  void UpdateRearKeyObjInfo(void);
  uint8 GetRearKeyObjSelectIntersetCode(iflyauto::FusionObject& obj);
  const RearKeyObjStr* GetRearKeyObjInfo(void) const;

  void UpdateFrontRadarKeyObjInfo(void);
  uint8 GetFrontRadarKeyObjSelectIntersetCode(iflyauto::FusionObject& obj);
  FrontRadarKeyObjStr GetFrontRadarKeyObjInfo() {
    return front_radar_key_obj_info_;
  };

  void UpdateFrontCipvKeyObjInfo(void);
  uint8 GetFrontCipvKeyObjSelectIntersetCode(iflyauto::FusionObject& obj);
  FrontCipvKeyObjStr GetFrontCipvKeyObjInfo() {
    return front_cipv_key_obj_info_;
  };

  void Log(void);

 private:
  MebPreprocess() { InitOnce(); }  // 构造函数私有化，只有 GetInstance 能调用

  void InitOnce(void);

  MebFunctionInput meb_input_;

  MebParameters meb_param_;

  RearKeyObjStr rear_key_obj_info_;

  FrontRadarKeyObjStr front_radar_key_obj_info_;

  FrontCipvKeyObjStr front_cipv_key_obj_info_;
};
}  // namespace adas_function
#endif