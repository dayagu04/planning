#pragma once
# include <iostream>
#include "local_view.h"
#include "route_info_strategy.h"

namespace planning{
class SDProRouteInfoStrategy : public RouteInfoStrategy{
public:

SDProRouteInfoStrategy(const MLCDeciderConfig* config_builder,
                   const planning::framework::Session* session);

void Update(RouteInfoOutput& route_info_output) override;

void CalculateMLCDecider(
    std::vector<std::shared_ptr<VirtualLane>> relative_id_lanes,
    RouteInfoOutput& route_info_output) override;

bool get_sdpromap_valid() override;

const ad_common::sdpromap::SDProMap& get_sdpro_map() override;

//todo(ldh): 其他virutal函数。
protected:
 ad_common::sdpromap::SDProMap sd_map_;
 LocalView local_view_;
 bool sdmap_valid_{false};


};
}