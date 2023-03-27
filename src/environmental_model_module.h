#ifndef ZNQC_MODULES_ENVIRONMENTAL_MODEL_MODULE_H
#define ZNQC_MODULES_ENVIRONMENTAL_MODEL_MODULE_H

#include "framework/module.h"
#include "environmental_model_manager.h"

namespace planning {
namespace modules {

class EnvironmentalModelModule : public planning::framework::PlanningModule {
 public:
  EnvironmentalModelModule();
  ~EnvironmentalModelModule();

  planning::framework::BaseModule* clone() const override;

  int init(const ::google::protobuf::Message* config,
           planning::framework::Session* session) override;

  int reset(const ::google::protobuf::Message* config) override;

  void compute(planning::framework::Frame* frame) override;

 private:
  planning::planner::EnvironmentalModelManager environmental_model_manager_;

  EgoPlanningConfigBuilder* load_config_builder(
      planning::framework::Session* session, const char* file_name);
};

REGISTER_MODULE_FACTORY(EnvironmentalModelModule)

}  // namespace modules
}  // namespace planning

#endif  // ZNQC_MODULES_ENVIRONMENTAL_MODEL_MODULE_H
