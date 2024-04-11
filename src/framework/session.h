#pragma once

#include "arena.h"
#include "macro.h"
#include "planning_def.h"
#include "scene_type_config.pb.h"
#include "simulation_context.h"

namespace planning {

class PlanningContext;
class PlanningOutputContext;
class EnvironmentalModel;
class VehicleConfigurationContext;
class SimulationContext;

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

  bool is_parking_scene() const {
    return get_scene_type() == planning::common::SceneType::PARKING_APA;
  }

  bool is_hpp_scene() const {
    return get_scene_type() == planning::common::SceneType::HPP;
  }

  planning::common::SceneType get_scene_type() const { return scene_type_; }
  void set_scene_type(const planning::common::SceneType &default_scene_type) {
    scene_type_ = default_scene_type;
  }

  const PlanningInitConfig &planning_init_config() const {
    return planning_init_config_;
  }

  const SimulationContext &simulation_context() const {
    return *simulation_context_;
  }

  SimulationContext *mutable_simulation_context() {
    return simulation_context_;
  }

 private:
  planning::EnvironmentalModel *environmental_model_;

  PlanningContext *planning_context_;

  planning::common::SceneType scene_type_;

  PlanningInitConfig planning_init_config_;

  SimulationContext *simulation_context_;

  DISALLOW_COPY_AND_ASSIGN(Session);
};

}  // namespace framework
}  // namespace planning
