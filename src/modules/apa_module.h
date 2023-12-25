#pragma once

#include "apa_plan_interface.h"
#include "frame.h"
#include "module.h"

namespace planning {
namespace modules {

class ApaPlanningModule : public framework::PlanningModule {
 public:
  ApaPlanningModule();
  ~ApaPlanningModule() = default;

  bool compute(framework::Frame* frame) override;

  planning::framework::BaseModule* clone() const override;

  bool init(const ::google::protobuf::Message* config,
            planning::framework::Session* session) override;

  bool reset(const ::google::protobuf::Message* config) override;

 private:
  std::unique_ptr<apa_planner::ApaPlanInterface> apa_interface_ptr_ = nullptr;
};

// REGISTER_MODULE_FACTORY(ApaPlanningModule)

}  // namespace modules
}  // namespace planning