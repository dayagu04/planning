#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <assert.h>
#include "config/basic_type.h"
#include "define/geometry.h"
#include "session.h"
#include "task_basic_types.h"
#include "grid_map.h"

namespace planning {
using namespace planning_math;

template <typename T, int N_DIM>
GridMapND<T, N_DIM>::GridMapND() {}

template <typename T, int N_DIM>
GridMapND<T, N_DIM>::GridMapND(
    const std::array<int, N_DIM> &dims_size,
    const std::array<double, N_DIM> &dims_resolution,
    const std::array<std::string, N_DIM> &dims_name) {
  dims_size_ = dims_size;
  dims_resolution_ = dims_resolution;
  dims_name_ = dims_name;

  SetNDimSteps(dims_size_);
  SetDataSize(dims_size_);
  data_ = std::vector<T>(data_size_, 0);
  origin_.fill(0);
}

template <typename T, int N_DIM>
STErrorType GridMapND<T, N_DIM>::GetValueUsingCoordinate(
    const std::array<int, N_DIM> &coord, T *val) const {
  if (!CheckCoordInRange(coord)) {
    // printf("[GridMapND] Out of range\n");
    return kWrongStatus;
  }
  int idx = GetMonoIdxUsingNDimIdx(coord);
  *val = data_[idx];
  return kSuccess;
}

template <typename T, int N_DIM>
STErrorType GridMapND<T, N_DIM>::GetValueUsingGlobalPosition(
    const std::array<double, N_DIM> &p_w, T *val) const {
  std::array<int, N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
  GetValueUsingCoordinate(coord, val);
  return kSuccess;
}

template <typename T, int N_DIM>
STErrorType GridMapND<T, N_DIM>::CheckIfEqualUsingGlobalPosition(
    const std::array<double, N_DIM> &p_w, const T &val_in, bool *res) const {
  std::array<int, N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
  T val;
  if (GetValueUsingCoordinate(coord, &val) != kSuccess) {
    *res = false;
  } else {
    *res = (val == val_in);
  }
  return kSuccess;
}

template <typename T, int N_DIM>
STErrorType GridMapND<T, N_DIM>::CheckIfEqualUsingCoordinate(
    const std::array<int, N_DIM> &coord, const T &val_in, bool *res) const {
  T val;
  if (GetValueUsingCoordinate(coord, &val) != kSuccess) {
    *res = false;
  } else {
    *res = (val == val_in);
  }
  return kSuccess;
}

template <typename T, int N_DIM>
STErrorType GridMapND<T, N_DIM>::SetValueUsingCoordinate(
    const std::array<int, N_DIM> &coord, const T &val) {
  if (!CheckCoordInRange(coord)) {
    // printf("[GridMapND] Out of range\n");
    return kWrongStatus;
  }
  int idx = GetMonoIdxUsingNDimIdx(coord);
  data_[idx] = val;
  return kSuccess;
}

template <typename T, int N_DIM>
STErrorType GridMapND<T, N_DIM>::SetValueUsingGlobalPosition(
    const std::array<double, N_DIM> &p_w, const T &val) {
  std::array<int, N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
  SetValueUsingCoordinate(coord, val);
  return kSuccess;
}

template <typename T, int N_DIM>
std::array<int, N_DIM> GridMapND<T, N_DIM>::GetCoordUsingGlobalPosition(
    const std::array<double, N_DIM> &p_w) const {
  std::array<int, N_DIM> coord = {};
  for (int i = 0; i < N_DIM; ++i) {
    coord[i] = std::round((p_w[i] - origin_[i]) / dims_resolution_[i]);
  }
  return coord;
}

template <typename T, int N_DIM>
std::array<double, N_DIM>
GridMapND<T, N_DIM>::GetRoundedPosUsingGlobalPosition(
    const std::array<double, N_DIM> &p_w) const {
  std::array<int, N_DIM> coord = {};
  for (int i = 0; i < N_DIM; ++i) {
    coord[i] = std::round((p_w[i] - origin_[i]) / dims_resolution_[i]);
  }
  std::array<double, N_DIM> round_pos = {};
  for (int i = 0; i < N_DIM; ++i) {
    round_pos[i] = coord[i] * dims_resolution_[i] + origin_[i];
  }
  return round_pos;
}

template <typename T, int N_DIM>
void GridMapND<T, N_DIM>::GetGlobalPositionUsingCoordinate(
    const std::array<int, N_DIM> &coord,
    std::array<double, N_DIM> *p_w) const {
  auto ptr = p_w->data();
  for (int i = 0; i < N_DIM; ++i) {
    *(ptr + i) = coord[i] * dims_resolution_[i] + origin_[i];
  }
  return;
}

template <typename T, int N_DIM>
void GridMapND<T, N_DIM>::GetCoordUsingGlobalMetricOnSingleDim(
    const double &metric, const int &i, int *idx) const {
  *idx = std::round((metric - origin_[i]) / dims_resolution_[i]);
  return;
}

template <typename T, int N_DIM>
void GridMapND<T, N_DIM>::GetGlobalMetricUsingCoordOnSingleDim(
    const int &idx, const int &i, double *metric) const {
  *metric = idx * dims_resolution_[i] + origin_[i];
  return;
}

template <typename T, int N_DIM>
bool GridMapND<T, N_DIM>::CheckCoordInRange(
    const std::array<int, N_DIM> &coord) const {
  for (int i = 0; i < N_DIM; ++i) {
    if (coord[i] < 0 || coord[i] >= dims_size_[i]) {
      return false;
    }
  }
  return true;
}

template <typename T, int N_DIM>
bool GridMapND<T, N_DIM>::CheckCoordInRangeOnSingleDim(const int &idx,
                                                       const int &i) const {
  return (idx >= 0) && (idx < dims_size_[i]);
}

template <typename T, int N_DIM>
int GridMapND<T, N_DIM>::GetMonoIdxUsingNDimIdx(
    const std::array<int, N_DIM> &idx) const {
  int mono_idx = 0;
  for (int i = 0; i < N_DIM; ++i) {
    mono_idx += dims_step_[i] * idx[i];
  }
  return mono_idx;
}

template <typename T, int N_DIM>
std::array<int, N_DIM> GridMapND<T, N_DIM>::GetNDimIdxUsingMonoIdx(
    const int &idx) const {
  std::array<int, N_DIM> idx_nd = {};
  int tmp = idx;
  for (int i = N_DIM - 1; i >= 0; --i) {
    idx_nd[i] = tmp / dims_step_[i];
    tmp = tmp % dims_step_[i];
  }
  return idx_nd;
}

template <typename T, int N_DIM>
void GridMapND<T, N_DIM>::SetNDimSteps(
    const std::array<int, N_DIM> &dims_size) {
  int step = 1;
  for (int i = 0; i < N_DIM; ++i) {
    dims_step_[i] = step;
    step = step * dims_size[i];
  }
  return;
}

template <typename T, int N_DIM>
void GridMapND<T, N_DIM>::SetDataSize(
    const std::array<int, N_DIM> &dims_size) {
  int total_ele_num = 1;
  for (int i = 0; i < N_DIM; ++i) {
    total_ele_num = total_ele_num * dims_size_[i];
  }
  data_size_ = total_ele_num;
  return;
}

template class GridMapND<uint8_t, 2>;
template class GridMapND<uint8_t, 3>;
template class GridMapND<int, 2>;
template class GridMapND<int, 3>;

}  // namespace planning