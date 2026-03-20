#ifndef __SIG_BACKTRACE_H__
#define __SIG_BACKTRACE_H__

#include <stdio.h>

void signal_backtrace_set_file_saved_path(const char* path);
int signal_backtrace_init(void);

#endif