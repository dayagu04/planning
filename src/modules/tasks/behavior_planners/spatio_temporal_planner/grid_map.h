#pragma once

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <array>
#include <cstdint>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <assert.h>
#include "config/basic_type.h"
#include "define/geometry.h"
#include "math/aabox2d.h"
#include "session.h"
#include "slt_point.h"
#include "src/common/vec2d.h"
#include "src/modules/common/agent/agent.h"
#include "src/modules/common/math/box2d.h"
#include "task_basic_types.h"
namespace planning {
using namespace planning_math;

enum STErrorType {
  kSuccess = 0,
  kWrongStatus = 1,
  kIllegalInput = 2,
  kUnknown = 3
};

struct AgentFrenetSpatioTemporalInFo {
  int agent_id = -1;
  int agent_type;
  // std::vector<std::vector<SLTPoint>> frenet_vertices;
  double time_gap = 0.2;
  std::array<AABox2d, 26> agent_box_set;
  std::unordered_map<int, AABox2d> agent_boxs_set;
  AABox2d max_agent_box;
};

struct VirtualAgentSpatioTemporalInFo {
  int agent_id = -1;
  SLTPoint frenet_slt_info;
};

template <typename T, int N_DIM>
class GridMapND {
 public:
  enum ValType {
    OCCUPIED = 70,
    // FREE = 102,
    FREE = 0,
    SCANNED_OCCUPIED = 128,
    UNKNOWN = 0
  };

  /**
   * @brief Construct a new GridMapND object
   *
   */
  GridMapND();

  /**
   * @brief Construct a new GridMapND object
   *
   * @param dims_size size of each dimension
   * @param dims_resolution resolution of each dimension
   * @param dims_name name of each dimension
   */
  GridMapND(const std::array<int, N_DIM> &dims_size,
            const std::array<double, N_DIM> &dims_resolution,
            const std::array<std::string, N_DIM> &dims_name);

  virtual ~GridMapND() = default;

  std::array<int, N_DIM> dims_size() const { return dims_size_; }

  int dims_size(const int &dim) const { return dims_size_.at(dim); }

  std::array<int, N_DIM> dims_step() const { return dims_step_; }

  int dims_step(const int &dim) const { return dims_step_.at(dim); }

  std::array<double, N_DIM> dims_resolution() const { return dims_resolution_; }

  double dims_resolution(const int &dim) const {
    return dims_resolution_.at(dim);
  }

  std::array<std::string, N_DIM> dims_name() const { return dims_name_; }

  std::string dims_name(const int &dim) const { return dims_name_.at(dim); }
  std::array<double, N_DIM> origin() const { return origin_; }

  int data_size() const { return data_size_; }

  const std::vector<T> *data() const { return &data_; }

  T data(const int &i) const { return data_[i]; };

  T *get_data_ptr() { return data_.data(); }

  const T *data_ptr() const { return data_.data(); }

  void set_origin(const std::array<double, N_DIM> &origin) { origin_ = origin; }

  void set_dims_size(const std::array<int, N_DIM> &dims_size) {
    dims_size_ = dims_size;
    SetNDimSteps(dims_size);
    SetDataSize(dims_size);
  }

  void set_dims_resolution(const std::array<double, N_DIM> &dims_resolution) {
    dims_resolution_ = dims_resolution;
  }

  void set_dims_name(const std::array<std::string, N_DIM> &dims_name) {
    dims_name_ = dims_name;
  }

  void set_data(const std::vector<T> &in) { data_ = in; }

  void clear_data() { data_ = std::vector<T>(data_size_, 0); }

  void fill_data(const T &val) { data_ = std::vector<T>(data_size_, val); }

  STErrorType GetValueUsingCoordinate(const std::array<int, N_DIM> &coord,
                                      T *val) const;

  STErrorType GetValueUsingGlobalPosition(const std::array<double, N_DIM> &p_w,
                                          T *val) const;

  STErrorType CheckIfEqualUsingGlobalPosition(
      const std::array<double, N_DIM> &p_w, const T &val_in, bool *res) const;

  STErrorType CheckIfEqualUsingCoordinate(const std::array<int, N_DIM> &coord,
                                          const T &val_in, bool *res) const;

  STErrorType SetValueUsingCoordinate(const std::array<int, N_DIM> &coord,
                                      const T &val);

  STErrorType SetValueUsingGlobalPosition(const std::array<double, N_DIM> &p_w,
                                          const T &val);

  std::array<int, N_DIM> GetCoordUsingGlobalPosition(
      const std::array<double, N_DIM> &p_w) const;

  std::array<double, N_DIM> GetRoundedPosUsingGlobalPosition(
      const std::array<double, N_DIM> &p_w) const;

  void GetGlobalPositionUsingCoordinate(const std::array<int, N_DIM> &coord,
                                        std::array<double, N_DIM> *p_w) const;

  void GetCoordUsingGlobalMetricOnSingleDim(const double &metric, const int &i,
                                            int *idx) const;

  void GetGlobalMetricUsingCoordOnSingleDim(const int &idx, const int &i,
                                            double *metric) const;

  bool CheckCoordInRange(const std::array<int, N_DIM> &coord) const;

  bool CheckCoordInRangeOnSingleDim(const int &idx, const int &i) const;

  int GetMonoIdxUsingNDimIdx(const std::array<int, N_DIM> &idx) const;

  std::array<int, N_DIM> GetNDimIdxUsingMonoIdx(const int &idx) const;

 private:
  void SetNDimSteps(const std::array<int, N_DIM> &dims_size);

  void SetDataSize(const std::array<int, N_DIM> &dims_size);

  std::array<int, N_DIM> dims_size_;
  std::array<int, N_DIM> dims_step_;
  std::array<double, N_DIM> dims_resolution_;
  std::array<std::string, N_DIM> dims_name_;
  std::array<double, N_DIM> origin_;

  int data_size_{0};
  std::vector<T> data_;
};

}  // namespace planning