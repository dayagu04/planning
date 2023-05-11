/**
 * @file
 * debug_info_context.h
 * 通过单例模式，实现json调试信息的添加
 **/

#pragma once

#include "arena.h"
#include "macro.h"
#include "mjson/mjson.hpp"

namespace planning {

class DebugInfoJson : public planning::common::Arena {
 public:
  static DebugInfoJson &GetInstance() {
    static DebugInfoJson instance;
    return instance;
  }
  std::shared_ptr<mjson::Json::object> GetDebugJson() { return debug_json_ptr_; }

 private:
  DebugInfoJson() : debug_json_ptr_(std::make_shared<mjson::Json::object>()) {}
  std::shared_ptr<mjson::Json::object> debug_json_ptr_;
};

#ifndef LOG_ARRAY_CACHE
#define LOG_ARRAY_CACHE 5000
#endif

#define JSON_DEBUG_VALUE(var_name, var_value)                \
  if (std::isnan(var_value))                                 \
    (*DebugInfoJson::GetInstance().GetDebugJson())[var_name] = \
        mjson::Json(static_cast<int>(1000));                 \
  else if (sizeof(var_value) <= 4)                           \
    (*DebugInfoJson::GetInstance().GetDebugJson())[var_name] = \
        mjson::Json(static_cast<int>(var_value));            \
  else                                                       \
    (*DebugInfoJson::GetInstance().GetDebugJson())[var_name] = \
        mjson::Json(static_cast<double>(var_value));

#define JSON_DEBUG_VECTOR(var_name, var_value, keep_length) \
  (*DebugInfoJson::GetInstance().GetDebugJson())[var_name] =  \
      mjson::Json(Utils::vec_to_char_array(var_value, keep_length));

#define JSON_READ_VALUE(var_name, type, json_name) \
  var_name = config.get<type>(json_name, false, var_name)

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
      sprintf(s + strlen(s), format1, vec[i]);
    }
    sprintf(s + strlen(s), format2, vec.back());
    return s;
  }
};

}  // namespace planning
