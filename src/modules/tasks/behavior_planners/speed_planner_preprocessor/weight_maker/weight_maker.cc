#include "weight_maker.h"

namespace planning {

WeightMaker::WeightMaker(const EgoPlanningConfigBuilder *config_builder,
                         framework::Session *session,
                         const TargetMaker &target_maker) {
  ;
}

common::Status WeightMaker::Run() { return common::Status::OK(); }
}  // namespace planning