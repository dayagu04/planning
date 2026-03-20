// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_LEGACY_INTERFACE2_4_5_HMI_HPP_OUTER_H_
#define _IFLYAUTO_LEGACY_INTERFACE2_4_5_HMI_HPP_OUTER_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "interface2.4.5/common_c.h"

#ifdef __cplusplus
namespace iflyauto {
  namespace interface_2_4_5 {
#endif

#pragma pack(4)

typedef enum {
  ALERT_NONE,                       // 无
  ALERT_EXPERIENCE_MEMORY_PARKING,  // 体验记忆泊车提醒
  ALERT_USE_MEMORY_PARKING,         // 使用记忆泊车提醒
  ALERT_LEARNING_COMPLETED,         // 完成学习
} _ENUM_PACKED_ AlertMsg;

typedef enum {
  DISPLAY_NONE,                        // 无
  DISPLAY_MOVE_FORWARD,                // 请往前开，正在找车位
  DISPLAY_START_PARKING,               // 请踩住刹车，点击开始
  DISPLAY_CONTINUE_DRIVING,            // 继续行驶x.xm
  DISPLAY_OBSTACLE_AVOIDANCE,          // 正在避让障碍物
  DISPLAY_PARKING_START,               // 泊车开始，注意周围环境
  DISPLAY_PARKING_PAUSED,              // 泊车暂停
  DISPLAY_PARKING_COMPLETED,           // 泊车已完成
  DISPLAY_ROUTE_MEMORY_IN_PROGRESS,    // 路线记忆中,完成学习无法点亮
  DISPLAY_ROUTE_MEMORY_PARKING,        // 路线记忆中,开始泊车
  DISPLAY_ROUTE_MEMORY_COMPLETED,      // 路线记忆中，完成学习点亮
  DISPLAY_ROUTE_SAVE_IN_PROGRESS,      // 路线保存中
  DISPLAY_RELEASE_BRAKE,               // 请松开刹车
  DISPLAY_MEMORY_PARKING_IN_PROGRESS,  // 记忆泊车工作中
  DISPLAY_MEMORY_PARKING_EXITED,       // 记忆泊车已退出
  DISPLAY_LEFT_RIGHT_TURN,             // 前方即将左转/右转
  DISPLAY_WATCH_FOR_PEDESTRIAN,        // 请注意行人
  DISPLAY_WAITING_VEHICLE_MOVE,        // 正在等待前方车辆行驶
  DISPLAY_AVOID_OBSTACLE,              // 正在避让附近障碍物
  DISPLAY_CHOOSE_PARKING_SPACE,        // 选择车位后，点击开始
  DISPLAY_NOTICE_INTERSECTION,         // 前方路口，注意安全
  DISPLAY_PARKING_SPACE_APPROACHING,   // 即将到达车位
  DISPLAY_PARKING_READY,               // 即将泊入车位
} _ENUM_PACKED_ DisplayMsg;

typedef enum {
  VOICE_NONE,
  VOICE_FIRST_SPACE_FOUND,            // 首次找到车位（找到车位、如需泊入请停车）
  VOICE_DRIVE_TO_ROUTE_START,         // 请开往学习路线起点
  VOICE_PRESS_BRAKE_AND_CLICK_START,  // 请踩住刹车，点击开始
  VOICE_PARKING_START,                // 泊车开始，注意周围环境
  VOICE_PARKING_PAUSE_FOR_BREAK,      // 泊车暂停，请确认安全后点击继续
  VOICE_PARKING_PAUSE_FOR_OBSTACLE,   // 有障碍物阻挡，泊车暂停，请确认安全后点击继续
  VOICE_PARKING_CONTINUE,             // 泊车继续
  VOICE_PARKING_COMPLETED,            // 泊车完成
  VOICE_PARKING_HELP_REQUEST,         // 泊车帮助请求
  VOICE_PARKING_EXIT,                 // 泊车已退出，请接管车辆
  VOICE_ROUTE_MEMORY_START,           // 请记住你的路线起点，开往……
  VOICE_SPEED_HIGH,                   // 请降低行驶车速，注意安全
  VOICE_ROUTE_MEMORY_PARKING_START,   // 请选择你的车位，点击开始后，我将为您泊入……
  VOICE_MANUAL_PARKING,               // 请手动倒入你的车位，泊入完成后，请挂P档
  VOICE_LEARNING_COMPLETED_VOICE,     // 如果已泊入你的车位，请点击完成
  VOICE_SAVE_FAILED,                  // 保存失败，请查看失败原因并重新学习
  VOICE_START_MEMORY_PARKING,         // 松开刹车，我们就出发啦
  VOICE_WATCH_SURROUNDING,            // 请注意周围环境
  VOICE_TAKE_OVER_VEHICLE_REQUEST,    // 请立即接管车辆，记忆泊车已退出
  VOICE_MEMORY_PARKING_EXITED,        // 记忆泊车已退出
  VOICE_LEFT_RIGHT_TURN,              // 前方即将左转/右转，已为您减速（汽车减速过弯）
  VOICE_WATCH_FOR_PEDESTRIAN,         // 请注意行人
  VOICE_WAITING_VEHICLE_MOVE,         // 等待前方车辆行驶
  VOICE_DRIVING_HELP_REQUEST,         // 避让障碍物（这段路有点难，帮我开过去吧）
  VOICE_PARKING_SPACE_OCCUPIED,       // 车位好像被占了，请寻找其他车位吧
  VOICE_WATCH_FOR_INTERSECTION,       // 前方路口，请注意周围环境安全，已为您减速通过
  VOICE_APPROACHING_PARKING_SPACE,    // 即将到达车位，请告诉我此次是否有额外泊车需求
  VOICE_PARKING_COMMAND_RESPONSE,     // 好的，即将为您泊入
  VOICE_PARKING_READY,                // 即将泊入你的车位，正在调整位置
  VOICE_EXPERIENCE_MEMORY_PARKING,    // 体验记忆泊车提醒
  VOICE_USE_MEMORY_PARKING,           // 使用记忆泊车提醒
  VOICE_SPEED_BUMP_REMINDER,          // 经过减速带提醒（经过第二个减速带时提醒）
  VOICE_NOT_IN_PARKING,               // 请先进入地库，再开启记忆泊车
  VOICE_NEED_NEXT_RETURN_PARKING,     // 请下次回到地库后再开启记忆泊车
  VOICE_SHIFTLEVER_INCORRECT,         // 档位不在P/D，无法开启HPP
  VOICE_HPP_UNAVAILABLE,              // 记忆泊车暂不可用
  VOICE_NOT_BACKWARDS,                // 开始学习前，请勿在地库倒行
} _ENUM_PACKED_ VoiceMsg;

typedef enum {
  NONE_REASON,    // 没有故障
  NETWORK_FAULT,  // 网络故障
  OTHER,          // 其他原因
} _ENUM_PACKED_ RouteSaveFailureReason;

typedef enum {
  NONE_LOCATION,  // 无法确定
  ON_THE_GROUND,  // 地上
  UNDERGROUND,    // 地下
} _ENUM_PACKED_ Location;

typedef enum {
  P,  // P挡
  R,  // R挡
  N,  // N挡
  D,  // D挡
} _ENUM_PACKED_ ShiftLever;

// HMI输出信息
typedef struct {
  Header header;               // 头信息
  FunctionalState func_state;  // 状态机状态

  AlertMsg alert_msg;   // 弹窗消息
  DisplayMsg disp_msg;  // 长显提示消息
  VoiceMsg voice_msg;   // 语音消息

  bool memory_parking_available;         // 记忆泊车是否可用
  bool is_first_time_using;              // 是否首次使用记忆泊车/已有泊车路线
  bool is_start_point_reached;           // 是否已经开到已有泊车路线的起点处
  bool memory_parking_resume_available;  // 记忆泊车是否可恢复
  uint32 speed_bumps_count;              // 减速带计数
  float learning_distance;               // 已学习距离           (米)
  float distance_to_parking_space;       // 距离泊车车位距离     (米)
  float distance_remaining_to_park;      // 距离泊入还剩距离     (米)
  ShiftLever shift_lever;                // 档位信息

  uint32 pedestrian_avoidance_count;  // 避让行人次数
  uint32 vehicle_avoidance_count;     // 避让车辆次数
  uint32 hpp_time_minute;             // 路线记忆时间、记忆泊车时间   (分钟)
  uint32 apa_time_minute;             // 泊车时间         (分钟)
  bool takeover_reminder;             // 请求接管提醒     (true:提醒/false:无需提醒)
  Location location;                  // 位置
  RouteSaveFailureReason
      route_save_failure_reason;  // 路线保存失败原因 // 失败原因： 定位失败，感知车位线识别，等待超时等梳理文案表
  bool is_position_need_refresh;  // 是否需要刷新定位
} _STRUCT_ALIGNED_ HmiHppOutput;

#pragma pack()
#ifdef __cplusplus
  }  // namespace interface_2_4_5
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_LEGACY_INTERFACE2_4_5_HMI_HPP_OUTER_H_