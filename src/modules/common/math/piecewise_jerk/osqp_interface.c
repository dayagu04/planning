#include "osqp.h"

int OptimizeWithOsqp(c_float P_x[], c_int P_nnz, c_int P_i[], c_int P_p[],
                     c_float q[], c_float A_x[], c_int A_nnz, c_int A_i[],
                     c_int A_p[], c_float l[], c_float u[], c_int n, c_int m,
                     c_int max_iter, c_float result[]) {
  // Problem settings
  OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
  if (!settings) {
    return 0;
  }

  // Structures
  OSQPWorkspace *work;  // Workspace
  OSQPData *data;       // OSQPData

  // Populate data
  data = (OSQPData *)c_malloc(sizeof(OSQPData));
  if (!data) {
    return 0;
  }
  data->n = n;
  data->m = m;
  data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
  data->q = q;
  data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
  data->l = l;
  data->u = u;

  // Define Solver settings as default
  osqp_set_default_settings(settings);
  settings->max_iter = max_iter;

  // Setup workspace
  work = osqp_setup(data, settings);

  // Solve Problem
  osqp_solve(work);

  if (work == 0) {
    return 0;
  }

  for (c_int i = 0; i < n; i++) {
    result[i] = work->solution->x[i];
  }
  int status = work->info->status_val;

  // Clean workspace
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return status;
}
