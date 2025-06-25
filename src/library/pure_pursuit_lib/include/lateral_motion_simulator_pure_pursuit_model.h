// #pragma once

// #include <memory.h>

// #include <memory>
// #include <vector>

// #include "../include/basic_pure_pursuit_model.h"
// #include "spline_projection.h"
// #include "src/library/advanced_ctrl_lib/include/spline.h"
// #include "src/modules/common/config/basic_type.h"
// #include "src/modules/context/reference_path.h"
// #include "vehicle_model_simulation.h"

// namespace planning {
// namespace simulator {
// class LatMotionSimulatorPurePursuitModel
//     : public control::BasicPurePursuitModel {
//  public:
//   LatMotionSimulatorPurePursuitModel(const ModelParam &model_param,
//                                      const State &init_state)
//       : model_param_(model_param), init_state_(init_state) {}

//   ErrorType CalculateAlpha(const std::shared_ptr<ReferencePath> &reference_path,
//                            bool need_calculate_init_projection = false);

//  private:
//   ModelParam model_param_;
//   State init_state_;
//   // calculate info
//   double alpha_;
//   double lat_position_error_;
//   double lat_theta_error_;
//   Eigen::Vector2d goal_poinit_;
//   double delta_;
// };
// }  // namespace simulator
// }  // namespace planning