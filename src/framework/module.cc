#include "framework/module.h"

namespace planning {
namespace framework {

template <>
ModuleFactoryRegistry::FactoryMap ModuleFactoryRegistry::factory_map_{};

bool register_module(const char* name, ModuleFactory* const factory) {
  return ModuleFactoryRegistry::insert_factory(name, factory);
}
bool deregister_module(const char* name, ModuleFactory* const factory) {
  return ModuleFactoryRegistry::erase_factory(name, factory);
}

}  // namespace framework
}  // namespace planning