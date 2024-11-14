#include "bound_maker.h"

namespace planning {

BoundMaker::BoundMaker(const EgoPlanningConfigBuilder *config_builder,
                       framework::Session *session) {}

common::Status BoundMaker::Run() { return common::Status::OK(); }

}  // namespace planning