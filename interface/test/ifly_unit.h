#ifndef IFLY_UNIT
#define IFLY_UNIT

#include <stdio.h>

#define ifly_assert(message, test)                                             \
  do {                                                                         \
    if (!(test))                                                               \
      return message;                                                          \
  } while (0)

#define ifly_run_test(test)                                                    \
  do {                                                                         \
    auto message = test();                                                     \
    tests_summary++;                                                           \
    if (message != "") {                                                       \
      tests_failed++;                                                          \
      printf("\033[0m\033[1;31m%s\033[0m\n", message.c_str());                 \
    } else {                                                                   \
      tests_access++;                                                          \
    }                                                                          \
  } while (0)

extern int tests_access;
extern int tests_failed;
extern int tests_summary;

#endif // IFLY_UNIT