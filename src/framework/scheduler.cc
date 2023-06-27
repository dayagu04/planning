#include "scheduler.h"

#include <iostream>
#include <vector>
#include "log.h"

// #include "src/common/log.h"

namespace planning {
namespace framework {

Scheduler::Scheduler() : run_count_(0) {}
Scheduler::~Scheduler() {}

void Scheduler::Init(Session *session) {
  run_count_ = 0;
  session_ = session;
  (void)InitModuleList(session);
}

void Scheduler::Reset() {
  LOG_DEBUG("Scheduler::reset\n");
  run_count_ = 0;
  for (auto &module_ptr : module_map_) {
    PlanningModule *p = dynamic_cast<PlanningModule *>(module_ptr.second);
    if (p != nullptr) {
      p->reset(nullptr);
    }
  }
}

void Scheduler::RunOnce() {
  run_count_++;
  Frame frame{session_};

  for (int i = 0; i < frame.session()->module_name_list().module_list_size();
       i++) {
    const auto module_name = frame.session()->module_name_list().module_list(i);
    if (module_map_.find(module_name.c_str()) == module_map_.end()) {
      LOG_WARNING("%s not found in module_map\n", module_name.c_str());
      continue;
    }
    PlanningModule *p =
        dynamic_cast<PlanningModule *>(module_map_[module_name.c_str()]);
    if (p != nullptr) {
      if (p->compute(&frame) != true) {
        LOG_DEBUG("%s compute failed \n", p->name().c_str());
        break;
      };
    }
  }
  // frame is destroyed here
}

bool Scheduler::InitModuleList(Session *session) {
  LOG_DEBUG("registry=%s\n", ModuleFactoryRegistry::dump().c_str());

  bool ret = true;
  std::vector<const char *> module_names{
      "planning.modules.EnvironmentalModelModule",
      "planning.modules.GeneralPlannerModule",
      "planning.modules.ApaPlanningModule"};

  // init modules
  for (auto &&name : module_names) {
    ModuleFactory *factory = ModuleFactoryRegistry::get_factory(name);
    if (factory == nullptr) {
      ret = false;
      continue;
    }
    ::google::protobuf::Message *module_config = nullptr;
    BaseModule *module = factory->create_module(module_config, session);
    if (module == nullptr) {
      ret = false;
      continue;
    }
    LOG_DEBUG("create_module succ: name=%s\n", module->name().c_str());

    module_map_[name] = module;
  }

  return ret;
}

}  // namespace framework
}  // namespace planning
