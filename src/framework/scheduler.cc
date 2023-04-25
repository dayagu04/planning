#include "framework/scheduler.h"

#include <iostream>
#include <vector>

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
  for (auto &module_ptr : module_list_) {
    PlanningModule *p = dynamic_cast<PlanningModule *>(module_ptr);
    if (p != nullptr) {
      p->reset(nullptr);
    }
  }
}

void Scheduler::RunOnce() {
  run_count_++;
  Frame frame{session_};

  for (auto &module_ptr : module_list_) {
    PlanningModule *p = dynamic_cast<PlanningModule *>(module_ptr);
    if (p != nullptr) {
      p->compute(&frame);
    }
  }
  // frame is destroyed here
}

bool Scheduler::InitModuleList(Session *session) {
  LOG_DEBUG("registry=%s\n", ModuleFactoryRegistry::dump().c_str());

  bool ret = true;
  // TODO read module list from config file
  std::vector<const char *> module_names{
      "planning.modules.EnvironmentalModelModule",
      "planning.modules.GeneralPlanningModule",
      // "planning.modules.CandidatesRunner",
  };

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

    module_list_.push_back(module);
  }

  return ret;
}

}  // namespace framework
}  // namespace planning
