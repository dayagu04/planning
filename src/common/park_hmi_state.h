#pragma once

#include <cstdint>
#include "adas_function/lkas_function/lane_keep_assist_type.h"
namespace planning {

// planning generate parking direction for hmi
enum ApaRecommendationDirection {
  ParityBit = 0,
  VerticalFrontLeft = 1,
  ParallelFrontLeft = 2,
  VerticalFront = 3,
  VerticalFrontRight = 4,
  ParallelFrontRight = 5,
  VerticalBack = 6,
  VerticalBackLeft = 7,
  VerticalBackRight = 8,
  VerticalHeadIn = 11,
  VerticalTailIn = 12,
};

class ApaDirectionGenerator {
 public:
  ApaDirectionGenerator() = default;
  ~ApaDirectionGenerator() = default;

  uint16_t RecommendationDirectionMask(const uint16_t state_bit) {
    return 1 << state_bit;
  }

  void SetRecommendationDirectionFlag(iflyauto::APAHMIData &state,
                                      const uint16_t state_bit) {
    state.planning_park_dir |= RecommendationDirectionMask(state_bit);
  }

  void ClearRecommendationDirectionFlag(iflyauto::APAHMIData &state) {
    state.planning_park_dir = 0;
  }

  const bool IsRecommendationDirection(iflyauto::APAHMIData &state,
                                       const uint16_t state_bit) {
    uint16_t res =
        state.planning_park_dir & RecommendationDirectionMask(state_bit);

    if (res > 0) {
      return true;
    }

    return false;
  }
};


}