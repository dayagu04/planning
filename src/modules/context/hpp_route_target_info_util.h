#pragma once

#include <memory>

namespace planning {

class ReferencePath;
namespace framework {
class Session;
}

// 基于当前 reference path 统一更新 HPP 的 distance_to_target_dest 与
// distance_to_target_slot，并写回 route_info。应在 StopDestinationDecider 中
// 先调用，供虚拟终点计算与 StartStopDecider 共用。
bool UpdateHppRouteTargetInfoFromReferencePath(
    framework::Session* session,
    const std::shared_ptr<ReferencePath>& reference_path);

}  // namespace planning
