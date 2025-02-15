/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "utils.h"
#include "rapidjson/internal/dtoa.h"
#include "rapidjson/internal/itoa.h"

#include <cmath>
#include <vector>

namespace planning {
namespace common {

static constexpr size_t int_to_str_buf_size = 22U;
static constexpr size_t double_to_str_buf_size =
    4U + std::numeric_limits<double>::digits10 -
    std::numeric_limits<double>::min_exponent10;

std::string to_string(const uint64_t value) {
  char buffer[int_to_str_buf_size];
  auto end = rapidjson::internal::u64toa(value, buffer);
  *end = '\0';
  return std::string{buffer, end};
}

std::string to_string(const int64_t value) {
  char buffer[int_to_str_buf_size];
  auto end = rapidjson::internal::i64toa(value, buffer);
  *end = '\0';
  return std::string{buffer, end};
}

std::string to_string(const uint32_t value) {
  char buffer[int_to_str_buf_size];
  auto end = rapidjson::internal::u32toa(value, buffer);
  *end = '\0';
  return std::string{buffer, end};
}

std::string to_string(const int32_t value) {
  char buffer[int_to_str_buf_size];
  auto end = rapidjson::internal::i32toa(value, buffer);
  *end = '\0';
  return std::string{buffer, end};
}

std::string to_string(const uint16_t value) {
  return to_string(static_cast<uint32_t>(value));
}
std::string to_string(const int16_t value) {
  return to_string(static_cast<int32_t>(value));
}
std::string to_string(const uint8_t value) {
  return to_string(static_cast<uint32_t>(value));
}
std::string to_string(const int8_t value) {
  return to_string(static_cast<int32_t>(value));
}

std::string to_string(const double value) {
  char buffer[double_to_str_buf_size];
  auto end = rapidjson::internal::dtoa(value, buffer);
  *end = '\0';
  return std::string{buffer, end};
}

}  // namespace common
}  // namespace planning
