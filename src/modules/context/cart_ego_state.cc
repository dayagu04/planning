#include "modules/context/cart_ego_state.h"
#include "modules/context/vehicle_config_context.h"
#include "modules/context/environmental_model.h"

namespace planning {

CartEgoStateManager::CartEgoStateManager(const VehicleParam &vehicle_param)
    : vehicle_param_(vehicle_param) {}

void CartEgoStateManager::update_transform() {
  Eigen::Vector4d q;
  q.x() = cart_ego_state_.ego_enu().orientation().x();
  q.y() = cart_ego_state_.ego_enu().orientation().y();
  q.z() = cart_ego_state_.ego_enu().orientation().z();
  q.w() = cart_ego_state_.ego_enu().orientation().w();
  Eigen::Vector3d v;
  v.x() = cart_ego_state_.ego_enu().position().x();
  v.y() = cart_ego_state_.ego_enu().position().y();
  v.z() = cart_ego_state_.ego_enu().position().z();

  car2enu_ = define::Transform(q, v);
  enu2car_ = define::Transform(q, v).inverse();
}

}  // namespace planning
