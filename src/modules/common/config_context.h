#pragma once

#include <iterator>
#include <fstream>
#include "common/macro.h"
// #include "planning/config/scenario_facade_config.h"
// #include "planning/config/vision_only_longitudinal_motion_planner_param.h"
#include "modules/common/utils/file.h"
#include <unordered_map>
#include "thirdparty/mjson/include/mjson/mjson.hpp"

#include "proto/generated_files/tasks_configs.pb.h"
#include "proto/generated_files/planning_config.pb.h"


namespace planning {
namespace common {

struct SyntheticConfiguration {
  std::string scene_type;
  std::string sensor_configuration;
  std::string cpu_configuration;
};

struct EngineConfiguration {
  std::string module_cfg_dir;
  std::string scenario_cfg_dir;
  std::string vehicle_cfg_dir;
  std::string algorithm_param_cfg_dir;
  std::string log_file_dir;
  std::string log_level;
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

  EngineConfiguration engine_config() { return engine_conf_;}

  AlgorithmParam algorithm_config() {return algorithm_conf_;}

  bool reset_synthetic_config(SyntheticConfiguration synthetic_config, std::string config_file_dir) {
    if (synthetic_config.scene_type != synthetic_conf_.scene_type) {
      LOG_DEBUG("ConfigContext: reset scene_type %s \n", synthetic_config.scene_type.c_str());
      std::string target_scene_config_dir;
      if (synthetic_config.scene_type == "highway") {
        std::cout<<"---------------+ " <<std::endl;
        scene_type_ = common::SceneType::HIGHWAY;
        target_scene_config_dir = config_file_dir + "/highway";
      } else if (synthetic_config.scene_type == "urban") {
        std::cout<<",,,,,,,,, " <<std::endl;
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
  void load_params_from_json(std::string config_file_dir) {
    // judge urban or highway scene
    // 采用proto加载配置文件
    std::cout<<"config_file_dir ===="<<config_file_dir<<std::endl;
    common::TasksConfig task_config;
    common::util::GetProtoFromFile(config_file_dir + "/scene.pb.txt", &task_config);
    auto scene_type = task_config.scene_type();
    synthetic_conf_.scene_type = scene_type;
    synthetic_conf_.sensor_configuration = task_config.sensor_configuration();
    synthetic_conf_.cpu_configuration = task_config.cpu_configuration();

    std::string target_scene_config_dir;
    LOG_DEBUG("ConfigContext: scene_type %s", task_config.scene_type().c_str(), "\n");
    std::cout<<"ConfigContext: scene_type ===="<<task_config.scene_type()<<std::endl;
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
  }

  bool load_engine_config_from_json(std::string config_path) {
    std::ifstream fjson(config_path);
    if (!fjson.is_open()) {
      return false;
    }
    std::string json_str((std::istreambuf_iterator<char>(fjson)),
                         std::istreambuf_iterator<char>());
    mjson::Reader reader(json_str);
    auto json_keys = reader.get_json_keys();

    if (std::find(json_keys.begin(), json_keys.end(), "module_cfg_dir") == json_keys.end()) {
      return false;
    }
    auto module_cfg_dir = reader.get<mjson::Json>("module_cfg_dir").string_value();
    std::cout<<"json module_cfg_dir: !!!===="<< module_cfg_dir <<std::endl;
    engine_conf_.module_cfg_dir = module_cfg_dir;

    if (std::find(json_keys.begin(), json_keys.end(), "log_file_dir") == json_keys.end()) {
      return false;
    }
    auto log_file_dir = reader.get<mjson::Json>("log_file_dir").string_value();
    std::cout<<"json log_file_dir: !!!===="<< log_file_dir <<std::endl;
    engine_conf_.log_file_dir = log_file_dir;

    if (std::find(json_keys.begin(), json_keys.end(), "log_level") == json_keys.end()) {
      return false;
    }
    auto log_level = reader.get<mjson::Json>("log_level").string_value();
    std::cout<<"json log_level: !!!===="<< log_level <<std::endl;
    engine_conf_.log_level = log_level;

    if (std::find(json_keys.begin(), json_keys.end(), "scenario_cfg_dir") == json_keys.end()) {
      return false;
    }
    auto scenario_cfg_dir = reader.get<mjson::Json>("scenario_cfg_dir").string_value();
    std::cout<<"json scenario_cfg_dir: !!!===="<< scenario_cfg_dir <<std::endl;
    engine_conf_.scenario_cfg_dir = scenario_cfg_dir;

    if (std::find(json_keys.begin(), json_keys.end(), "vehicle_cfg_dir") == json_keys.end()) {
      return false;
    }
    auto vehicle_cfg_dir = reader.get<mjson::Json>("vehicle_cfg_dir").string_value();
    std::cout<<"json vehicle_cfg_dir: !!!===="<< vehicle_cfg_dir <<std::endl;
    engine_conf_.vehicle_cfg_dir = vehicle_cfg_dir;

    if (std::find(json_keys.begin(), json_keys.end(), "algorithm_param_cfg_dir") == json_keys.end()) {
      return false;
    }
    auto algorithm_param_cfg_dir = reader.get<mjson::Json>("algorithm_param_cfg_dir").string_value();
    std::cout<<"json algorithm_param_cfg_dir: !!!===="<< algorithm_param_cfg_dir <<std::endl;
    engine_conf_.algorithm_param_cfg_dir = algorithm_param_cfg_dir;
    return true;
  }

   void load_algorithm_config_from_json(std::string config_path) {
    std::ifstream fjson(config_path);
    std::string json_str((std::istreambuf_iterator<char>(fjson)),
                         std::istreambuf_iterator<char>());
    mjson::Reader reader(json_str);
    auto srnd_obj_roi_x_max = reader.get<mjson::Json>("srnd_object_roi").object_value()["x_thrs_max"].number_value();
    algorithm_conf_.srnd_obj_roi_x_max = srnd_obj_roi_x_max;

    auto srnd_obj_roi_x_min = reader.get<mjson::Json>("srnd_object_roi").object_value()["x_thrs_min"].number_value();
    algorithm_conf_.srnd_obj_roi_x_min = srnd_obj_roi_x_min;

    auto srnd_obj_roi_y_max = reader.get<mjson::Json>("srnd_object_roi").object_value()["y_thrs_max"].number_value();
    algorithm_conf_.srnd_obj_roi_y_max = srnd_obj_roi_y_max;

    auto srnd_obj_roi_y_min = reader.get<mjson::Json>("srnd_object_roi").object_value()["y_thrs_min"].number_value();
    algorithm_conf_.srnd_obj_roi_y_min = srnd_obj_roi_y_min;

    std::cout<<"algorithm_param json load well !!!"<<std::endl;

   }

 private:
  common::SceneType scene_type_;
  SyntheticConfiguration synthetic_conf_;
  EngineConfiguration engine_conf_;
  AlgorithmParam algorithm_conf_;
};

} // namespace common
} // namespace planning