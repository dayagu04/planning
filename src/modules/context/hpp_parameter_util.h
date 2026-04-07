#ifndef HPP_PARAMETER_UTIL_H_
#define HPP_PARAMETER_UTIL_H_

#include <cstdint>
#include <unordered_set>

#include "config/vehicle_param.h"
#include "ego_planning_config.h"
#include "math/math_utils.h"

namespace planning {
  enum BufferType {
    BUFFER_TYPE_NONE = 0,       //默认
    BUFFER_TYPE_ROAD_LINE = 1,  //道路划线边界
    BUFFER_TYPE_SLOT_LINE = 2,  //车位线边界
    BUFFER_TYPE_CURB = 3,      //路沿边界
    BUFFER_TYPE_UNMOVABLE_OBJ = 4,  //不可移动物体
    BUFFER_TYPE_VRU = 5,    //可移动 VRU
    BUFFER_TYPE_VEHICLE = 6,    //可移动 车辆
  };
  struct InterpolateParam {
    double min_x;
    double max_x;
    double min_y;
    double max_y;
  };
  enum EnvType {
    RAMP = 0,   //坡道
    TURN= 1,    //转弯
  };
  struct BaseParam {
    double default_buffer = 0.3;      //默认横向 buffer
    std::unordered_map<BufferType, double> buffer;    // 不同类型对象的横向 buffer
    InterpolateParam ego_v_interpolate_param;   // 自车速度插值参数
    InterpolateParam obs_v_interpolate_param;   // 自车速度插值参数
    std::unordered_map<EnvType, double> env_buffer;    // 不同环境的横向 buffer
  };
class HPPParameterUtil {
  public:
static BaseParam GetDefaultBaseParam(const LateralObstacleDeciderConfig* config = nullptr) {
  BaseParam base_param;
  if(config == nullptr) {
    base_param.buffer[BufferType::BUFFER_TYPE_ROAD_LINE] = 0.2;
    base_param.buffer[BufferType::BUFFER_TYPE_SLOT_LINE] = 0.2;
    base_param.buffer[BufferType::BUFFER_TYPE_CURB] = 0.3;
    base_param.buffer[BufferType::BUFFER_TYPE_UNMOVABLE_OBJ] = 0.3;
    base_param.buffer[BufferType::BUFFER_TYPE_VRU] = 0.4;
    base_param.buffer[BufferType::BUFFER_TYPE_VEHICLE] = 0.4;

    base_param.env_buffer[EnvType::RAMP] = 0.1;
    base_param.env_buffer[EnvType::TURN] = 0.1;
  } else {
    base_param.buffer[BufferType::BUFFER_TYPE_ROAD_LINE] = config->lat_buffer_for_road_line;
    base_param.buffer[BufferType::BUFFER_TYPE_SLOT_LINE] = config->lat_buffer_for_slot_line;
    base_param.buffer[BufferType::BUFFER_TYPE_CURB] = config->lat_buffer_for_curb;
    base_param.buffer[BufferType::BUFFER_TYPE_UNMOVABLE_OBJ] = config->lat_buffer_for_unmovable_obj;
    base_param.buffer[BufferType::BUFFER_TYPE_VRU] = config->lat_buffer_for_vru;
    base_param.buffer[BufferType::BUFFER_TYPE_VEHICLE] = config->lat_buffer_for_vehicle;

    base_param.env_buffer[EnvType::RAMP] = config->extra_lat_buffer_for_ramp;
    base_param.env_buffer[EnvType::TURN] = config->extra_lat_buffer_for_turn;
  }
  base_param.ego_v_interpolate_param.min_x = 2.0;
  base_param.ego_v_interpolate_param.min_y = 0.0;
  base_param.ego_v_interpolate_param.max_x = 4.0;
  base_param.ego_v_interpolate_param.max_y = 0.2;

  base_param.obs_v_interpolate_param.min_x = 2.0;
  base_param.obs_v_interpolate_param.min_y = 0.0;
  base_param.obs_v_interpolate_param.max_x = 6.0;
  base_param.obs_v_interpolate_param.max_y = 0.2;
  return base_param;
}


/*
  ego_v：自车绝对速度，单位 m/s
  obs_v：他车相对自车速度，单位 m/s
*/
static double CalculateLatBuffer(const BufferType type, const double ego_v,
                                 const double obs_v,
                                 const std::vector<EnvType>& env_types) {
  BaseParam base_param = GetDefaultBaseParam();
  return CalculateLatBuffer(base_param, type, ego_v, obs_v, env_types);
}

static double CalculateLatBuffer(const BaseParam& base_param,
                                 const BufferType type, const double ego_v,
                                 const double obs_v,
                                 const std::vector<EnvType>& env_types) {
  //S1：基于类型获取默认 buffer
  double base_buffer = base_param.default_buffer;
  if (base_param.buffer.find(type) != base_param.buffer.end()) {
    base_buffer = base_param.buffer.at(type);
  }

  //S2：基于自车速度获取额外 buffer
  const auto& ego_v_param = base_param.ego_v_interpolate_param;
  double ego_v_buffer = planning_math::ClampInterpolate(
      ego_v_param.min_x, ego_v_param.min_y, ego_v_param.max_x,
      ego_v_param.max_y, std::fabs(ego_v));

  //S3: 基于他车速度获取额外 buffer
  const auto& obs_v_param = base_param.obs_v_interpolate_param;
  double obs_v_buffer = planning_math::ClampInterpolate(
      obs_v_param.min_x, obs_v_param.min_y, obs_v_param.max_x,
      obs_v_param.max_y, std::fabs(obs_v));

  //S4: 基于环境获取额外 buffer
  double env_buffer = 0.0;
  for (const auto& env_type : env_types) {
    if (base_param.env_buffer.find(env_type) != base_param.env_buffer.end()) {
      env_buffer += base_param.env_buffer.at(env_type);
    }
  }

  return base_buffer + ego_v_buffer + obs_v_buffer + env_buffer;
}
};

}  // namespace planning

#endif
