#include "common/apa_cos_sin.h"

#include "common/apa_cos_table.h"
#include "common/apa_sin_table.h"
#include "utils_math.h"

namespace planning {
double apa_cos(const double theta_in_rad) {
  const int index = HALF_APA_COS_TABLE_SIZE + theta_in_rad * M_10000_PI;
  return APA_COS_TABLE[index];
}

double apa_sin(const double theta_in_rad) {
  const int index = HALF_APA_SIN_TABLE_SIZE + theta_in_rad * M_10000_PI;
  return APA_SIN_TABLE[index];
}
}  // namespace  planning