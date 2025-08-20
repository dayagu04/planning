#pragma once

#include <fstream>
#include <iostream>
#include <iterator>
#include <unordered_map>

#include "macro.h"
#include "nlohmann_json.hpp"
#include "scene_type_config.pb.h"
#include "utils/file.h"
#include "log_glog.h"

namespace planning {
namespace common {

struct LogConfiguration {
  std::string log_file;
  std::string log_level;
};

struct EngineConfiguration {
  std::string module_cfg_dir;  // module的配置文件路径
  std::string vehicle_cfg_dir;
  LogConfiguration log_conf;
};

struct AlgorithmParam {
  double srnd_obj_roi_x_max;
  double srnd_obj_roi_x_min;
  double srnd_obj_roi_y_max;
  double srnd_obj_roi_y_min;
};

class ConfigurationContext {
 private:
  // this is a singleton class
  IFLY_DECLARE_SINGLETON(ConfigurationContext);

 public:
  common::SceneTypeConfig scene_type_config() { return scene_type_config_; }

  EngineConfiguration engine_config() { return engine_conf_; }

  AlgorithmParam algorithm_config() { return algorithm_conf_; }

  bool load_params_from_json(std::string config_file_dir) {
    ILOG_DEBUG << "config_file_dir ====" << config_file_dir;
    if (access((config_file_dir + "/scene.pb.txt").c_str(), F_OK) == -1) {
      ILOG_ERROR << config_file_dir << "/scene.pb.txt not exist!";
      return false;
    }
    bool ret = common::util::GetProtoFromFile(config_file_dir + "/scene.pb.txt",
                                              &scene_type_config_);
    ILOG_DEBUG << "scene_type_config=" << scene_type_config_.ShortDebugString()
             ;
    return ret;
  }

  bool load_engine_config_from_json(std::string config_path) {
    std::ifstream fjson(config_path);
    if (!fjson.is_open()) {
      ILOG_ERROR << "Failed to open engine config file.";
      return false;
    }
    nlohmann::json json_data;
    try {
      fjson >> json_data;
    } catch (nlohmann::json::parse_error& err) {
      ILOG_ERROR << "Failed to parse config file: " << err.what();
      return false;
    }

    // 加载module配置文件路径
    if (!json_data.count("module_cfg_dir")) {
      ILOG_ERROR << "There is no module config file!";
      return false;
    }
    auto module_cfg_dir = json_data["module_cfg_dir"];
    ILOG_INFO << "json module_cfg_dir: !!!====" << module_cfg_dir;
    engine_conf_.module_cfg_dir = module_cfg_dir;

    // 加载log配置文件路径
    if (!json_data.count("log_cfg")) {
      ILOG_ERROR << "There is no log config file!";
      return false;
    }
    std::string log_file = json_data["log_cfg"]["log_file"];
    ILOG_INFO << "json log_file: !!!====" << log_file;
    std::string log_level = json_data["log_cfg"]["log_level"];
    ILOG_INFO << "json log_level: !!!====" << log_level;
    engine_conf_.log_conf.log_file = log_file;
    engine_conf_.log_conf.log_level = log_level;

    // 加载车辆模型配置文件路径
    if (!json_data.count("vehicle_cfg_dir")) {
      ILOG_ERROR << "There is no vehicle config file!";
      return false;
    }
    auto vehicle_cfg_dir = json_data["vehicle_cfg_dir"];
    ILOG_INFO << "json vehicle_cfg_dir: !!!====" << vehicle_cfg_dir;
    engine_conf_.vehicle_cfg_dir = vehicle_cfg_dir;

    ILOG_INFO << "Load engine config DONE!";
    return true;
  }

  bool load_engine_config_from_json(std::string res_path,
                                    std::string config_path) {
    std::ifstream fjson(config_path);
    if (!fjson.is_open()) {
      ILOG_ERROR << "Failed to open engine config file.";
      return false;
    }
    nlohmann::json json_data;
    try {
      fjson >> json_data;
    } catch (nlohmann::json::parse_error& err) {
      ILOG_ERROR << "Failed to parse config file: " << err.what();
      return false;
    }

    // 加载module配置文件路径
    if (!json_data.count("module_cfg_dir")) {
      ILOG_ERROR << "There is no module config file!";
      return false;
    }
    std::string module_cfg_dir_temp = json_data["module_cfg_dir"];
    std::string module_cfg_dir = res_path + module_cfg_dir_temp;
    ILOG_INFO << "json module_cfg_dir: !!!====" << module_cfg_dir;
    engine_conf_.module_cfg_dir = module_cfg_dir;

    // 加载log配置文件路径
    if (!json_data.count("log_cfg")) {
      ILOG_ERROR << "There is no log config file!";
      return false;
    }
    std::string log_file_temp = json_data["log_cfg"]["log_file"];
    std::string log_file = res_path + log_file_temp;
    ILOG_INFO << "json log_file: !!!====" << log_file;
    std::string log_level_temp = json_data["log_cfg"]["log_level"];
    std::string log_level = res_path + log_level_temp;
    ILOG_INFO << "json log_level: !!!====" << log_level;
    engine_conf_.log_conf.log_file = log_file;
    engine_conf_.log_conf.log_level = log_level;

    // 加载车辆模型配置文件路径
    if (!json_data.count("vehicle_cfg_dir")) {
      ILOG_ERROR << "There is no vehicle config file!";
      return false;
    }
    std::string vehicle_cfg_dir_temp = json_data["vehicle_cfg_dir"];
    std::string vehicle_cfg_dir = res_path + vehicle_cfg_dir_temp;
    ILOG_INFO << "json vehicle_cfg_dir: !!!====" << vehicle_cfg_dir
             ;
    engine_conf_.vehicle_cfg_dir = vehicle_cfg_dir;

    ILOG_INFO << "Load engine config DONE!";
    return true;
  }

 private:
  common::SceneTypeConfig scene_type_config_;
  EngineConfiguration engine_conf_;
  AlgorithmParam algorithm_conf_;
};

}  // namespace common
}  // namespace planning