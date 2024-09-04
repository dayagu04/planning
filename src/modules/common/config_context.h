#pragma once

#include <fstream>
#include <iostream>
#include <iterator>
#include <unordered_map>

#include "macro.h"
#include "nlohmann_json.hpp"
#include "scene_type_config.pb.h"
#include "utils/file.h"

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
    std::cout << "config_file_dir ====" << config_file_dir << std::endl;
    if (access((config_file_dir + "/scene.pb.txt").c_str(), F_OK) == -1) {
      std::cout << config_file_dir << "/scene.pb.txt not exist!" << std::endl;
      return false;
    }
    bool ret = common::util::GetProtoFromFile(config_file_dir + "/scene.pb.txt",
                                              &scene_type_config_);
    std::cout << "scene_type_config=" << scene_type_config_.ShortDebugString()
              << std::endl;
    return ret;
  }

  bool load_engine_config_from_json(std::string config_path) {
    std::ifstream fjson(config_path);
    if (!fjson.is_open()) {
      LOG_ERROR("Failed to open engine config file. \n");
      return false;
    }
    nlohmann::json json_data;
    try {
      fjson >> json_data;
    } catch (nlohmann::json::parse_error& err) {
      std::cout << "Failed to parse config file: " << err.what() << std::endl;
      return false;
    }

    // 加载module配置文件路径
    if (!json_data.count("module_cfg_dir")) {
      LOG_ERROR("There is no module config file! \n");
      return false;
    }
    auto module_cfg_dir = json_data["module_cfg_dir"];
    std::cout << "json module_cfg_dir: !!!====" << module_cfg_dir << std::endl;
    engine_conf_.module_cfg_dir = module_cfg_dir;

    // 加载log配置文件路径
    if (!json_data.count("log_cfg")) {
      LOG_ERROR("There is no log config file! \n");
      return false;
    }
    std::string log_file = json_data["log_cfg"]["log_file"];
    std::cout << "json log_file: !!!====" << log_file << std::endl;
    std::string log_level = json_data["log_cfg"]["log_level"];
    std::cout << "json log_level: !!!====" << log_level << std::endl;
    engine_conf_.log_conf.log_file = log_file;
    engine_conf_.log_conf.log_level = log_level;

    // 加载车辆模型配置文件路径
    if (!json_data.count("vehicle_cfg_dir")) {
      LOG_ERROR("There is no vehicle config file! \n");
      return false;
    }
    auto vehicle_cfg_dir = json_data["vehicle_cfg_dir"];
    std::cout << "json vehicle_cfg_dir: !!!====" << vehicle_cfg_dir
              << std::endl;
    engine_conf_.vehicle_cfg_dir = vehicle_cfg_dir;

    std::cout << "Load engine config DONE!" << std::endl;
    return true;
  }

 private:
  common::SceneTypeConfig scene_type_config_;
  EngineConfiguration engine_conf_;
  AlgorithmParam algorithm_conf_;
};

}  // namespace common
}  // namespace planning