#pragma once

namespace planning {

// theta value from -pi to pi by step of 0.0001 * pi, total length is 20001
#ifndef APA_COS_TABLE_SIZE
#define APA_COS_TABLE_SIZE  20001
#endif

#ifndef HALF_APA_COS_TABLE_SIZE
#define HALF_APA_COS_TABLE_SIZE 10000
#endif

extern const double APA_COS_TABLE[APA_COS_TABLE_SIZE];
} // namespace  planning