#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>


#define RETURN_OK 0
#define RETURN_FAIL -1
#define RETURN_INVALID_ARG -2
#define RETURN_NOT_FOUND -3

extern int g_current_debug_log_level;
extern int g_debug_print_view_on;
typedef unsigned long long UINT64_T;
typedef long long INT64_T;
typedef unsigned int UINT32_T;
typedef int INT32_T;
typedef short INT16_T;
// typedef char int8_t;
typedef enum {
    LOG_LEVEL_OFF = 0,
    LOG_LEVEL_ERR,
    LOG_LEVEL_WARN,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG
} LOG_DEBUG_LEVEL_E;

#define LOG_PRINT(fmt, ...)  \
    printf(fmt, ##__VA_ARGS__)

#define LOG_DEBUG(...)   \
    do { \
        if(g_current_debug_log_level >= LOG_LEVEL_DEBUG) \
            LOG_PRINT("[debug] " __VA_ARGS__); \
    } while(0)


#define LOG_INFO(...)   \
    do { \
        if(g_current_debug_log_level >= LOG_LEVEL_INFO) \
            LOG_PRINT("[info] " __VA_ARGS__); \
    } while(0)

#define LOG_WARN(...)   \
    do { \
        if(g_current_debug_log_level >= LOG_LEVEL_WARN) \
            LOG_PRINT("[warn] " __VA_ARGS__); \
    } while(0)

#define LOG_ERROR(...)   \
    do { \
        if(g_current_debug_log_level >= LOG_LEVEL_ERR) \
            LOG_PRINT("[error] " __VA_ARGS__); \
    } while(0)

#define PRINT_VIEW(...)  \
    do { \
        if(g_debug_print_view_on) \
            printf(__VA_ARGS__); \
    } while(0)
