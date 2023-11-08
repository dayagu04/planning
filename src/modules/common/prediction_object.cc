#include "prediction_object.h"

namespace planning {

int hash_prediction_id(int perception_id, int traj_index) {
  return perception_id + traj_index * kPredictionHashBit;
}

int inverse_hash_prediction_id(int prediction_id) {
  return prediction_id % kPredictionHashBit;
}
}  // namespace planning
