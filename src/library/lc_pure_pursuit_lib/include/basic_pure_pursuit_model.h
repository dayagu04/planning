#pragma once

#include <cstddef>
#include <limits>
#include <vector>

#include "reference_path.h"
#include "spline.h"
#include "spline_projection.h"
#include "src/modules/common/config/basic_type.h"
namespace planning {
class BasicPurePursuitModel {
 public:
  struct ModelParam {
    double ld{6.0};  // look ahead distance
    double wheelbase_length{3.3};
    ModelParam() = default;
    ModelParam(const double ld, const double wheelbase_length)
        : ld(ld), wheelbase_length(wheelbase_length) {}
    ModelParam(const ModelParam& param)
        : ld(param.ld), wheelbase_length(param.wheelbase_length) {}
  };
  struct ModelState {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    double vel{0.0};
    ModelState() = default;
    ModelState(const double x, const double y, const double theta,
               const double vel)
        : x(x), y(y), theta(theta), vel(vel) {}
    ModelState(const ModelState& state)
        : x(state.x), y(state.y), theta(state.theta), vel(state.vel) {}
  };

  BasicPurePursuitModel(const ModelParam& model_param,
                        const ModelState& current_state)
      : model_param_(model_param), mdoel_state_(current_state) {
    is_model_param_set_ = true;
    is_current_state_set_ = true;
  }

  BasicPurePursuitModel() {
    is_model_param_set_ = false;
    is_current_state_set_ = false;
  };

  void set_model_state(const ModelState& current_state) {
    mdoel_state_ = current_state;
    is_current_state_set_ = true;
  }

  void set_model_param(const ModelParam& model_param) {
    model_param_ = model_param;
    is_model_param_set_ = true;
  }

  void Reset();

  const double get_alpha() const { return alpha_; }

  const double get_delta() const { return delta_; }

  const Eigen::Vector2d get_goal_point() const {return goal_point_; }

  const double get_ld_actual_lenghth() const { return ld_actual_length_; }

  static ErrorType CalculateDesiredDelta(const double wheelbase_len,
                                         const double angle_diff,
                                         const double look_ahead_dist,
                                         double* delta);

  ErrorType ProcessReferencePath(
      std::shared_ptr<ReferencePath> reference_path);

  ErrorType CalculateDesiredDelta(const double lat_offset);

 private:
  ErrorType CalculateAlpha(const double lat_offset);

 private:
  ModelParam model_param_;
  ModelState mdoel_state_;
  // set model param flag
  bool is_model_param_set_{false};
  // set model state flag
  bool is_current_state_set_{false};
  // set input flag
  bool is_input_set_{false};

  // calculate info
  double alpha_;
  double lat_position_error_;
  double ld_actual_length_;
  double lat_theta_error_;
  double lat_offset_;
  Eigen::Vector2d goal_point_;
  // ref info
  pnc::mathlib::spline ref_x_s_spline_;
  pnc::mathlib::spline ref_y_s_spline_;
  pnc::mathlib::spline ref_theta_s_spline_;
  std::vector<double> ref_s_vec_;
  std::vector<double> ref_x_vec_;
  std::vector<double> ref_y_vec_;
  std::vector<double> ref_theta_vec_;
  size_t ref_points_size_{0};

  double delta_;
  std::shared_ptr<ReferencePath> reference_path_;
};
}  // namespace planning