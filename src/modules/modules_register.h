#pragma once

#include "environmental_model_module.h"
#include "general_planner_module.h"
#include "module.h"

template <>
ModuleFactoryRegistry::FactoryMap ModuleFactoryRegistry::factory_map_{};

namespace planning {
namespace modules {

REGISTER_MODULE_FACTORY(GeneralPlannerModule);
REGISTER_MODULE_FACTORY(EnvironmentalModelModule);

}  // namespace modules
}  // namespace planning