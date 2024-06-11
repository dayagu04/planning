#pragma once

namespace planning {

class HysteresisDecision {
  // a hysteresis counter.
  // set true from false with continuous up_thre_ Set API call.
  // set false from true with continuous down_thre_ Reset API call.
 public:
  HysteresisDecision() {}
  HysteresisDecision(double up_thre, double down_thre)
      : up_thre_(up_thre), down_thre_(down_thre) {}
  // HysteresisDecision& operator=(const HysteresisDecision& other) {
  //   if (this != &other) {
  //       data = other.data;
  //   }
  //   return *this;
  // }
  void SetThreValue(double up_thre, double down_thre);
  void SetValidByCount();
  void SetInvalidCount();
  void SetIsValidByValue(double value);
  void Reset();
  bool IsValid();
  int Value();

 private:
  double up_thre_{1};
  double down_thre_{1};
  int count_{0};
  bool is_valid_{false};
};
}  // namespace planning