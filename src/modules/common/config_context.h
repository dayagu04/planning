#pragma once

#include <fstream>
#include <iostream>
#include <iterator>

#include "macro.h"
#include "nlohmann_json.hpp"
// #include "planning/config/scenario_facade_config.h"
// #include "planning/config/vision_only_longitudinal_motion_planner_param.h"
#include <unordered_map>

#include "planning_config.pb.h"
#include "tasks_configs.pb.h"
#include "utils/file.h"
// #include "thirdparty/mjson/include/mjson/mjson.hpp"

namespace planning {
namespace common {

struct LogConfiguration {
  std::string log_file_dir;
  std::string log_level;
};

struct EngineConfiguration {
  std::string module_cfg_dir;    // module的配置文件路径
  std::string scenario_cfg_dir;  // 场景配置文件路径
  std::string vehicle_cfg_dir;
  LogConfiguration log_conf;
};

struct SyntheticConfiguration {
  std::string scene_type;
  std::string sensor_configuration;
  std::string cpu_configuration;
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
  DECLARE_SINGLETON(ConfigurationContext);

 public:
  common::SceneType scene_type() { return scene_type_; }

  SyntheticConfiguration synthetic_config() { return synthetic_conf_; }

  EngineConfiguration engine_config() { return engine_conf_; }

  AlgorithmParam algorithm_config() { return algorithm_conf_; }

  bool reset_synthetic_config(SyntheticConfiguration synthetic_config, std::string config_file_dir) {
    if (synthetic_config.scene_type != synthetic_conf_.scene_type) {
      LOG_DEBUG("ConfigContext: reset scene_type %s \n", synthetic_config.scene_type.c_str());
      std::string target_scene_config_dir;
      if (synthetic_config.scene_type == "highway") {
        std::cout << "---------------+ " << std::endl;
        scene_type_ = common::SceneType::HIGHWAY;
        target_scene_config_dir = config_file_dir + "/highway";
      } else if (synthetic_config.scene_type == "urban") {
        std::cout << ",,,,,,,,, " << std::endl;
        scene_type_ = common::SceneType::URBAN;
        target_scene_config_dir = config_file_dir + "/urban";
      } else {
        scene_type_ = common::SceneType::NOT_DEFINED;
        LOG_ERROR("ConfigContext: invalid scene type!\n");
        return false;
      }
      //   load_scene_params_from_json(target_scene_config_dir);
      synthetic_conf_ = synthetic_config;
      return true;
    } else {
      LOG_DEBUG("scene_type is no changed: %s\n", synthetic_conf_.scene_type.c_str());
      synthetic_conf_ = synthetic_config;
      return false;
    }
  }

  // load all scenarios configuration for target scene
  bool load_params_from_json(std::string config_file_dir) {
    // judge urban or highway scene
    // 采用proto加载配置文件
    std::cout << "config_file_dir ====" << config_file_dir << std::endl;
    common::TasksConfig task_config;
    if (access((config_file_dir + "/scene.pb.txt").c_str(), F_OK) == -1) {
      std::cout << config_file_dir << "/scene.pb.txt not exist!" << std::endl;
      return false;
    }
    common::util::GetProtoFromFile(config_file_dir + "/scene.pb.txt", &task_config);
    auto scene_type = task_config.scene_type();
    synthetic_conf_.scene_type = scene_type;
    synthetic_conf_.sensor_configuration = task_config.sensor_configuration();
    synthetic_conf_.cpu_configuration = task_config.cpu_configuration();

    std::string target_scene_config_dir;
    LOG_DEBUG("ConfigContext: scene_type %s", task_config.scene_type().c_str(), "\n");
    std::cout << "ConfigContext: scene_type ====" << task_config.scene_type() << std::endl;
    if (scene_type == "highway") {
      scene_type_ = common::SceneType::HIGHWAY;
      target_scene_config_dir = config_file_dir + "/highway";
    } else if (scene_type == "urban") {
      scene_type_ = common::SceneType::URBAN;
      target_scene_config_dir = config_file_dir + "/urban";
    } else {
      scene_type_ = common::SceneType::NOT_DEFINED;
      LOG_ERROR("ConfigContext: invalid scene type!");
    }
    // load_scene_params_from_json(target_scene_config_dir);
    return true;
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
    std::string log_file_dir = json_data["log_cfg"]["log_file_dir"];
    std::cout << "json log_file_dir: !!!====" << log_file_dir << std::endl;
    std::string log_level = json_data["log_cfg"]["log_level"];
    std::cout << "json log_level: !!!====" << log_level << std::endl;
    engine_conf_.log_conf.log_file_dir = log_file_dir;
    engine_conf_.log_conf.log_level = log_level;

    // 加载场景配置文件路径
    if (!json_data.count("scenario_cfg_dir")) {
      LOG_ERROR("There is no scenario config file! \n");
      return false;
    }
    std::string scenario_cfg_dir = json_data["scenario_cfg_dir"];
    std::cout << "json scenario_cfg_dir: !!!====" << scenario_cfg_dir << std::endl;
    engine_conf_.scenario_cfg_dir = scenario_cfg_dir;

    // 加载车辆模型配置文件路径
    if (!json_data.count("vehicle_cfg_dir")) {
      LOG_ERROR("There is no vehicle config file! \n");
      return false;
    }
    auto vehicle_cfg_dir = json_data["vehicle_cfg_dir"];
    std::cout << "json vehicle_cfg_dir: !!!====" << vehicle_cfg_dir << std::endl;
    engine_conf_.vehicle_cfg_dir = vehicle_cfg_dir;

    std::cout << "Load engine config DONE!" << std::endl;
    return true;
  }

  // void load_algorithm_config_from_json(std::string config_path) {
  //   std::ifstream fjson(config_path);
  //   std::string json_str((std::istreambuf_iterator<char>(fjson)),
  //                        std::istreambuf_iterator<char>());
  //   mjson::Reader reader(json_str);
  //   auto srnd_obj_roi_x_max = reader.get<mjson::Json>("srnd_object_roi")
  //                                 .object_value()["x_thrs_max"]
  //                                 .number_value();
  //   algorithm_conf_.srnd_obj_roi_x_max = srnd_obj_roi_x_max;

  //   auto srnd_obj_roi_x_min = reader.get<mjson::Json>("srnd_object_roi")
  //                                 .object_value()["x_thrs_min"]
  //                                 .number_value();
  //   algorithm_conf_.srnd_obj_roi_x_min = srnd_obj_roi_x_min;

  //   auto srnd_obj_roi_y_max = reader.get<mjson::Json>("srnd_object_roi")
  //                                 .object_value()["y_thrs_max"]
  //                                 .number_value();
  //   algorithm_conf_.srnd_obj_roi_y_max = srnd_obj_roi_y_max;

  //   auto srnd_obj_roi_y_min = reader.get<mjson::Json>("srnd_object_roi")
  //                                 .object_value()["y_thrs_min"]
  //                                 .number_value();
  //   algorithm_conf_.srnd_obj_roi_y_min = srnd_obj_roi_y_min;

  //   std::cout << "algorithm_param json load well !!!" << std::endl;
  // }

 private:
  common::SceneType scene_type_;
  SyntheticConfiguration synthetic_conf_;
  EngineConfiguration engine_conf_;
  AlgorithmParam algorithm_conf_;
};

}  // namespace common
}  // namespace planning