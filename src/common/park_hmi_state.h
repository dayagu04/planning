#pragma once

#include <cstdint>

#include "planning_hmi_c.h"

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

enum APAPaRecommendedDirection {
  PaParitybit = 0,
  PaLeft = 1,   // 向左贴边
  PaRight = 2,  // 向右贴边
};

class ApaDirectionGenerator {
 public:
  ApaDirectionGenerator() = default;
  ~ApaDirectionGenerator() = default;

  uint16_t RecommendationDirectionMask(const uint16_t state_bit) {
    return 1 << state_bit;
  }

  void SetReleaseDirectionFlag(iflyauto::APAHMIData &state,
                               const uint16_t state_bit) {
    state.planning_park_dir |= RecommendationDirectionMask(state_bit);
  }

  void ClearReleaseDirectionFlag(iflyauto::APAHMIData &state) {
    state.planning_park_dir = 0;
  }

  void SetRecommendationDirectionFlag(iflyauto::APAHMIData &state,
                                      const uint16_t state_bit) {
    state.planning_recommend_park_dir |= RecommendationDirectionMask(state_bit);
  }

  void ClearRecommendationDirectionFlag(iflyauto::APAHMIData &state) {
    state.planning_recommend_park_dir = 0;
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

class ApaPAStateGeneral {
 public:
  ApaPAStateGeneral() = default;
  ~ApaPAStateGeneral() = default;

  uint16_t PADirectionMask(const uint16_t state_bit) { return 1 << state_bit; }
  void SetPADirectionFlag(iflyauto::APAHMIData &state,
                          const uint16_t state_bit) {
    state.planning_park_pa_dir |= PADirectionMask(state_bit);
  }
  void ClearPADirectionFlag(iflyauto::APAHMIData &state) {
    state.planning_park_pa_dir = 0;
  }
  void SetPARemainDistance(iflyauto::APAHMIData &state, float32 distance) {
    state.pa_remain_distance = distance;
  }
  void SetPARemainDistancePercentage(iflyauto::APAHMIData &state,
                                     float32 percentage) {
    state.remain_distance_percentage = percentage;
  }
  void SetRecommendPADirectionFlag(iflyauto::APAHMIData &state,
                                   const uint16_t state_bit) {
    state.planning_recommend_pa_dir |= PADirectionMask(state_bit);
  }
  void ClearRecommendPADirectionFlag(iflyauto::APAHMIData &state) {
    state.planning_recommend_pa_dir = 0;
  }
};

inline bool HasBit(uint16_t mask, uint16_t bit) {
  return (mask & (1u << bit)) != 0;
}
}  // namespace planning
