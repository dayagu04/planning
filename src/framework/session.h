#pragma once

#include "common/arena.h"
#include "src/common/macro.h"
#include "framework/planning_def.h"
#include "modules/context/planning_context.h"
#include "modules/context/planning_output_context.h"
#include "modules/context/environmental_model.h"
#include "modules/context/vehicle_config_context.h"

namespace planning {
namespace framework {

class Session : public planning::common::Arena {
 public:
  Session();
  ~Session();

  void Init();
  void Update();
  void Reset();

  const planning::EnvironmentalModel &environmental_model() const { return *environmental_model_; }

  planning::EnvironmentalModel *mutable_environmental_model() { return environmental_model_; }

  const PlanningContext &planning_context() const { return *planning_context_; }

  PlanningContext *mutable_planning_context() { return planning_context_; }

  const PlanningOutputContext &planning_output_context() const { return *planning_output_context_; }

  PlanningOutputContext *mutable_planning_output_context() { return planning_output_context_; }

  const VehicleConfigurationContext &vehicle_config_context() const { return *vehicle_config_context_; }

  VehicleConfigurationContext *mutable_vehicle_config_context() { return vehicle_config_context_; }

  bool is_highway_scene() const {
    return get_scene_type() == planning::common::SceneType::HIGHWAY;
  }
  bool is_urban_scene() const {
    return get_scene_type() == planning::common::SceneType::URBAN;
  }
  bool is_parking_scene() const {
    return get_scene_type() == planning::common::SceneType::PARKING;
  }

  void feed_scene_type(planning::common::SceneType value) {
    // environmental_model_->mutable_local_view()->set_scene_type(value);
  }

  planning::common::SceneType get_scene_type() const {
    // return environmental_model_->get_local_view().scene_type();
    // hack
    return default_scene_type_;
  }
  const std::string &get_scene_type_name() const {
    return planning::common::SceneType_Name(get_scene_type());
  }
  planning::common::SceneType default_scene_type() const {
    return default_scene_type_;
  }
  const std::string &default_scene_type_name() const {
    return planning::common::SceneType_Name(default_scene_type());
  }

  // void feed_function_mode(uint8_t value) { function_mode_ = value; }
  // std::string get_function_mode_name() const;

  const PlanningInitConfig &planning_init_config() const { return planning_init_config_; }

 private:
  planning::EnvironmentalModel *environmental_model_;
  PlanningContext *planning_context_;
  PlanningOutputContext *planning_output_context_;
  VehicleConfigurationContext *vehicle_config_context_;
  planning::common::SceneType default_scene_type_;
  // uint8_t function_mode_ = planning::common::FunctionModeEnum::NOA;
  PlanningInitConfig planning_init_config_;

  DISALLOW_COPY_AND_ASSIGN(Session);
};

}  // namespace framework
}  // namespace planning

