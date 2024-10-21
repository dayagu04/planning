#include "st_graph_helper.h"

#include "st_graph_utils.h"

namespace planning {
namespace speed {

namespace {

constexpr double kMaxPathLength = 400.0;
constexpr double kMathEpsilon = 1e-10;
constexpr double kTimeResolution = 0.1;
constexpr bool kEnablePlotLowSpeedAgentOriginStBoundary = false;
}  // namespace

StGraphHelper::StGraphHelper(const STGraph& st_graph) : st_graph_(st_graph) {}

}  // namespace speed
}  // namespace planning