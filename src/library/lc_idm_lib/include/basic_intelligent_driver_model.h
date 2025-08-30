#pragma once
#include "src/modules/common/config/basic_type.h"

namespace planning {

class BasicIntelligentDriverModel {
 public:
  struct ModelState {
    double s{0.0};        // longitudinal distance
    double vel{0.0};      // longitudinal speed
    double s_front{0.0};  // leading vehicle
    double vel_front{0.0};

    ModelState() {}
    ModelState(const double &s, const double &vel, const double &s_front,
               const double &v_front)
        : s(s), vel(vel), s_front(s_front), vel_front(v_front) {}
  };

  struct ModelParam {
    double kDesiredVelocity = 0.0;
    double kVehicleLength = 5.0;                   // l_alpha-1
    double kMinimumSpacing = 2.0;                  // s0
    double kDesiredHeadwayTime = 1.0;              // T
    double kAcceleration = 2.0;                    // a
    double kComfortableBrakingDeceleration = 0.8;  // b
    double kHardBrakingDeceleration = 1.0;
    int kExponent = 4;  // delta
  };

  BasicIntelligentDriverModel() = default;

  ErrorType GetIdmDesiredAcceleration(const ModelParam &param,
                                      const ModelState &cur_state, double *acc);
  ErrorType GetIIdmDesiredAcceleration(const ModelParam &param,
                                       const ModelState &cur_state, double *acc);
  ErrorType GetAccDesiredAcceleration(const ModelParam &param,
                                      const ModelState &cur_state, double *acc);
};

}  // namespace planning