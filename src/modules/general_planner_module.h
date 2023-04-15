#ifndef ZNQC_MODULES_GENERAL_PLANNER_MODULE_H
#define ZNQC_MODULES_GENERAL_PLANNER_MODULE_H

#include "framework/module.h"
#include "general_planner.h"

namespace planning {
namespace modules {

class GeneralPlannerModule : public planning::framework::PlanningModule {
 public:
  GeneralPlannerModule();
  ~GeneralPlannerModule();

  planning::framework::BaseModule* clone() const override;

  int init(const ::google::protobuf::Message* config,
           planning::framework::Session* session) override;

  int reset(const ::google::protobuf::Message* config) override;

  void compute(planning::framework::Frame* frame) override;

 private:
  planning::planner::GeneralPlanner general_planner_;

  EgoPlanningConfigBuilder* load_config_builder(
      planning::framework::Session* session, const char* file_name);
};

REGISTER_MODULE_FACTORY(GeneralPlannerModule)

}  // namespace modules
}  // namespace planning

#endif  // ZNQC_MODULES_DGENERAL_PLANNER_MODULE_H
