# include <iostream>
#include "local_view.h"
#include "SDPro_route_info_strategy.h"
#include "route_info_strategy.h"

namespace planning{

SDProRouteInfoStrategy::SDProRouteInfoStrategy(
    const MLCDeciderConfig* config_builder,
    const planning::framework::Session* session)
    : RouteInfoStrategy(config_builder, session) {}

void SDProRouteInfoStrategy::Update(RouteInfoOutput& route_info_output) {
  ILOG_INFO << "update";
  return;
}

void SDProRouteInfoStrategy::CalculateMLCDecider(
    const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    RouteInfoOutput& route_info_output) {
  ILOG_INFO << "update";
  return;
}

bool SDProRouteInfoStrategy::get_sdpromap_valid() { return sdmap_valid_; }

const ad_common::sdpromap::SDProMap& SDProRouteInfoStrategy::get_sdpro_map() {
  return sd_map_;
}
}