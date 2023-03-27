#pragma once
// #include "osqp.h"
#include "thirdparty/osqp/include/osqp.h"

int OptimizeWithOsqp(c_float P_x[], c_int P_nnz, c_int P_i[], c_int P_p[],
                     c_float q[], c_float A_x[], c_int A_nnz, c_int A_i[],
                     c_int A_p[], c_float l[], c_float u[], c_int n, c_int m,
                     c_int max_iter, c_float result[]);
