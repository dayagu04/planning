#pragma once

#include "framework/frame.h"
#include "framework/module.h"
#include "modules/apa_planner/apa_planner_dispatcher.h"

namespace planning {
namespace modules {

class ApaPlanningModule : public framework::PlanningModule {
 public:
  void compute(framework::Frame* frame) override;

  planning::framework::BaseModule* clone() const override;

  int init(const ::google::protobuf::Message* config,
      planning::framework::Session* session) override;

  int reset(const ::google::protobuf::Message* config) override;

 private:
  std::unique_ptr<apa_planner::ApaPlannerDispatcher> planner_dispatcher_
      = nullptr;
};

REGISTER_MODULE_FACTORY(ApaPlanningModule)

} // namespace modules
} // namespace planning