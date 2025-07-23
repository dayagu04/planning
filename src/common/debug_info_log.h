/**
 * @file
 * debug_info_log.h
 * 通过单例模式，实现json调试信息的添加
 **/

#pragma once

#include "arena.h"
#include "macro.h"
#include "mjson/mjson.hpp"
#include "planning_debug_info.pb.h"
#include "planning_gflags.h"
namespace planning {

/**
 * @brief 创建单例用来管理debuginfo，各主要模块输出输入采用proto形式录入debug
 */
class DebugInfoManager : public planning::common::Arena {
 public:
  static DebugInfoManager &GetInstance() {
    static DebugInfoManager instance;
    return instance;
  }
  // 获取proto内容
  std::unique_ptr<planning::common::PlanningDebugInfo> &GetDebugInfoPb() {
    return debug_info_pb_;
  }
  // 获取json内容
  std::unique_ptr<mjson::Json::object> &GetDebugJson() {
    return debug_info_json_;
  }

 private:
  DebugInfoManager()
      : debug_info_pb_(std::make_unique<planning::common::PlanningDebugInfo>()),
        debug_info_json_(std::make_unique<mjson::Json::object>()) {}

  std::unique_ptr<planning::common::PlanningDebugInfo> debug_info_pb_;
  std::unique_ptr<mjson::Json::object> debug_info_json_;
};

#ifndef LOG_ARRAY_CACHE
#define LOG_ARRAY_CACHE 5000
#endif

#define JSON_DEBUG_VALUE(var_name, var_value)                     \
  if (std::isnan(var_value))                                      \
    (*DebugInfoManager::GetInstance().GetDebugJson())[var_name] = \
        mjson::Json(static_cast<int>(1000));                      \
  else if (sizeof(var_value) <= 4)                                \
    (*DebugInfoManager::GetInstance().GetDebugJson())[var_name] = \
        mjson::Json(static_cast<int>(var_value));                 \
  else                                                            \
    (*DebugInfoManager::GetInstance().GetDebugJson())[var_name] = \
        mjson::Json(static_cast<double>(var_value));

#define JSON_DEBUG_VECTOR(var_name, var_value, keep_length)     \
  (*DebugInfoManager::GetInstance().GetDebugJson())[var_name] = \
      mjson::Json(Utils::vec_to_char_array(var_value, keep_length));

#define JSON_READ_VALUE(var_name, type, json_name) \
  var_name = config.get<type>(json_name, false, var_name)

#ifdef DEBUG_PRINT_ENABLE
#define DEBUG_PRINT(content)           \
  if (FLAGS_enable_apa_debug_print) {  \
    std::cout << content << std::endl; \
  }
#else
#define DEBUG_PRINT(content)
#endif

class Utils {
 public:
  Utils(){};
  ~Utils(){};
  static char *vec_to_char_array(const std::vector<double> &vec,
                                 const unsigned char &precision) {
    static char s[LOG_ARRAY_CACHE];
    static char format1[] = "%.6f,";  // support 0~9 precision
    static char format2[] = "%.6f";
    format1[2] = '0' + precision;
    format2[2] = '0' + precision;
    s[0] = '\0';
    if (vec.empty()) {
      return s;
    }
    for (int i = 0; i < (int)vec.size() - 1; i++) {
      snprintf(s + strlen(s), LOG_ARRAY_CACHE - strlen(s), format1, vec[i]);
    }
    snprintf(s + strlen(s), LOG_ARRAY_CACHE - strlen(s), format2, vec.back());
    return s;
  }
};

}  // namespace planning
