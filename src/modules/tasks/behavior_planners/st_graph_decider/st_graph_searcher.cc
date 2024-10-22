#include "st_graph_searcher.h"

namespace planning {

StGraphSearcher::StGraphSearcher(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  name_ = "StGraphSearcher";
}
bool StGraphSearcher::Execute(){return true;}

}  // namespace planning
