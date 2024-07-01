#include "hysteresis_decision.h"

namespace planning {

void HysteresisDecision::SetThreValue(double up_thre, double down_thre) {
  up_thre_ = up_thre;
  down_thre_ = down_thre;
}

void HysteresisDecision::SetValidByCount() {
  if (is_valid_) {
    // is_valid
    count_ = 0;
  } else {
    ++count_;
    if (count_ >= up_thre_) {
      is_valid_ = true;
      count_ = 0;
    }
  }
}

void HysteresisDecision::SetInvalidCount() {
  if (!is_valid_) {
    count_ = 0;
  } else {
    ++count_;
    if (count_ >= down_thre_) {
      is_valid_ = false;
      count_ = 0;
    }
  }
}

void HysteresisDecision::SetIsValidByValue(double value) {
  if (is_valid_) {
    if (value <= down_thre_) {
      is_valid_ = false;
    }
  } else {
    if (value >= up_thre_) {
      is_valid_ = true;
    }
  }
}

bool HysteresisDecision::IsValid() { return is_valid_; }

int HysteresisDecision::Value() { return count_; }

void HysteresisDecision::Reset() {
  count_ = 0;
  is_valid_ = false;
}

}  // namespace planning