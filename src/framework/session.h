#pragma once

#include "arena.h"
#include "macro.h"
#include "planning_def.h"
#include "scene_type_config.pb.h"

namespace planning {

class PlanningContext;
class PlanningOutputContext;
class EnvironmentalModel;
class VehicleConfigurationContext;

namespace framework {

class Session : public planning::common::Arena {
 public:
  Session();
  ~Session();

  bool Init();
  void Update();
  void Reset();

  const planning::EnvironmentalModel &environmental_model() const {
    return *environmental_model_;
  }

  planning::EnvironmentalModel *mutable_environmental_model() {
    return environmental_model_;
  }

  const PlanningContext &planning_context() const { return *planning_context_; }

  PlanningContext *mutable_planning_context() { return planning_context_; }

  const PlanningOutputContext &planning_output_context() const {
    return *planning_output_context_;
  }

  PlanningOutputContext *mutable_planning_output_context() {
    return planning_output_context_;
  }

  const VehicleConfigurationContext &vehicle_config_context() const {
    return *vehicle_config_context_;
  }

  VehicleConfigurationContext *mutable_vehicle_config_context() {
    return vehicle_config_context_;
  }

  bool is_parking_scene() const {
    return get_scene_type() == planning::common::SceneType::HPP;
  }

  bool is_hpp_scene() const {
    return get_scene_type() == planning::common::SceneType::HPP;
  }

  planning::common::SceneType get_scene_type() const { return scene_type_; }
  void set_scene_type(const planning::common::SceneType &default_scene_type) {
    scene_type_ = default_scene_type;
  }

  common::ModuleList module_name_list() const {
    auto it = module_name_map_.find(scene_type_);
    if (it == module_name_map_.end()) {
      return {};
    }
    return it->second;
  }

  const PlanningInitConfig &planning_init_config() const {
    return planning_init_config_;
  }

 private:
  planning::EnvironmentalModel *environmental_model_;
  PlanningContext *planning_context_;
  PlanningOutputContext *planning_output_context_;
  VehicleConfigurationContext *vehicle_config_context_;
  planning::common::SceneType scene_type_;
  std::map<common::SceneType, common::ModuleList> module_name_map_;
  PlanningInitConfig planning_init_config_;

  DISALLOW_COPY_AND_ASSIGN(Session);
};

}  // namespace framework
}  // namespace planning
