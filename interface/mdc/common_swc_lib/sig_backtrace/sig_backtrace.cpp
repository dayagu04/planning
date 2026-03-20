#include "sig_backtrace.h"
#include "common_def.h"
#include <signal.h>
#include <execinfo.h>
#include <ucontext.h>
#include <sys/syscall.h>
#include <string.h>
#include <errno.h>
#include <functional>
#include <time.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdint.h>
#include <sys/stat.h>
#include <libgen.h>
#include <elf.h>
#include <link.h>
#include <dlfcn.h>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#define BACKTRACE_SIZE 128
#define MAX_PRE_DIR_PATH_LEN 128
#define MAX_FILE_PATH_LEN 256

int g_current_debug_log_level = LOG_LEVEL_ERR;
int g_debug_print_view_on = 0;
static char a_backtrace_dir_path[MAX_PRE_DIR_PATH_LEN] = ".";
static FILE* a_saved_file_handle = NULL;

// 会导致崩溃的信号
static const int crash_signals[] = {
    SIGQUIT,
    SIGXCPU,
    SIGXFSZ,
    SIGSYS,
    SIGSTKFLT,
    SIGTTIN,
    SIGTTOU,
    SIGPROF,
    SIGSEGV, // 段错误 (内存访问违规)
    SIGABRT, // 中止信号 (assert失败等)
    SIGFPE,  // 浮点异常
    SIGILL,  // 非法指令
    SIGBUS,  // 总线错误
    SIGTRAP, // 断点/跟踪陷阱
};

// 检查目录是否存在
static int _directory_exists(const char *path) {
    struct stat st;
    if (stat(path, &st) == 0) {
        return S_ISDIR(st.st_mode); // 如果是目录返回1
    }
    return 0; // 不存在或不是目录
}

// 创建目录（支持多级目录）
static int _create_directory(const char *path) {
    if (!path) {
        return -1;
    }
    char path_copy[MAX_FILE_PATH_LEN] = {};
    strcpy(path_copy, path);
    char *p = path_copy;
    // 跳过根目录
    if (*p == '/') {
        p++;
    }
    
    while (*p != '\0') {
        // 找到下一个斜杠
        while (*p != '\0' && *p != '/') {
            p++;
        }
        
        // 临时保存当前位置
        char save = *p;
        *p = '\0';
        
        // 创建当前层级的目录
        if (mkdir(path_copy, 0755) != 0) {
            if (errno != EEXIST) { // 如果目录已存在，不视为错误
                return -1;
            }
        }
        
        // 恢复斜杠并继续
        if (save != '\0') {
            *p = save;
            p++;
        }
    }
    return 0;
}

// 创建文件所在目录
static int _check_to_create_file_directory(const char *filename) {
    if (!filename) {
        return -1;
    }
    char dir_path[MAX_FILE_PATH_LEN] = {};
    strcpy(dir_path, filename);    
    // 获取目录路径
    char *dir_name = dirname(dir_path);
    
    int result = 0;
    // 如果目录不存在，则创建
    if (!_directory_exists(dir_name)) {
        LOG_INFO("目录不存在，创建目录: %s\n", dir_name);
        result = _create_directory(dir_name);
    }
    
    return result;
}

static void LOG2FILE(const char* format, ...)
{
    if (a_saved_file_handle == NULL) {
        char file_saved_name[MAX_FILE_PATH_LEN] = {};
        const char* proc_name = program_invocation_short_name;
        time_t now = time(NULL);
        struct tm* t = localtime(&now);
        snprintf(file_saved_name, MAX_FILE_PATH_LEN, "%s/backtrace/backtrace_%s_%04d%02d%02d_%02d%02d%02d.log",
             a_backtrace_dir_path, proc_name,
             t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
             t->tm_hour, t->tm_min, t->tm_sec);

        _check_to_create_file_directory(file_saved_name);
        a_saved_file_handle = fopen(file_saved_name, "w");
        if (a_saved_file_handle == NULL) {
            LOG_ERROR("can not open file: %s \n", file_saved_name);
            return;
        }
    }
    fwrite(format, strlen(format) + 1, 1, a_saved_file_handle);
}

// static void dump_process_maps(pid_t pid)
// {
//     if (a_saved_file_handle != NULL)
//     {
//         char maps_file_path[64] = {};
//         sprintf(maps_file_path, "/proc/%d/maps", pid);
//         FILE* maps_fd = fopen(maps_file_path, "rb");
//         if (maps_fd != NULL) {
//             char map_buf[1024];
//             size_t read_len;
//             while ((read_len = fread(map_buf, 1, sizeof(map_buf), maps_fd)) > 0) {
//                 fwrite(map_buf, read_len, 1, a_saved_file_handle);
//             }
//             fclose(maps_fd);
//         }
//         else
//         {
//             LOG_ERROR("can not open file: %s \n", maps_file_path);
//         }
//     }
// }

static void signal_callback_function(int sig, siginfo_t *info, void *ucontext) {
    void *array[BACKTRACE_SIZE];
    char **strings;
    
    // 获取当前线程信息
    pid_t tid = syscall(SYS_gettid);
    pthread_t ptid = pthread_self();
    pid_t pid =  getpid();

    char log_buffer[8*1024] = {};
    strcat(log_buffer, "\n>>> ======== Process Crashed ========\n");
    sprintf(log_buffer+strlen(log_buffer), ">>> Signal: %d (%s)\n", sig, strsignal(sig));
    sprintf(log_buffer+strlen(log_buffer), ">>> Process ID: %d\n", pid);
    sprintf(log_buffer+strlen(log_buffer), ">>> Thread ID (pthread): %lu\n", (unsigned long)ptid);
    sprintf(log_buffer+strlen(log_buffer), ">>> Thread ID (system): %d\n", tid);
    sprintf(log_buffer+strlen(log_buffer), ">>> Fault address: %p\n", info->si_addr);
    
    // 尝试获取线程名
    char thread_name[16] = "unknown";
    pthread_getname_np(pthread_self(), thread_name, sizeof(thread_name));
    sprintf(log_buffer+strlen(log_buffer), ">>> Thread name: %s\n", thread_name);
    
    ucontext_t *uc = (ucontext_t *)ucontext;
    // 获取程序计数器（崩溃点的指令地址）
#if defined(__x86_64__)
    strcat(log_buffer, ">>> platform x86_66\n");
    void *crash_pc = (void*)uc->uc_mcontext.gregs[REG_RIP];
    void *crash_bp = (void*)uc->uc_mcontext.gregs[REG_RBP];
    void *crash_sp = (void*)uc->uc_mcontext.gregs[REG_RSP];
#elif defined(__i386__)
    strcat(log_buffer, ">>> platform i386\n");
    void *crash_pc = (void*)uc->uc_mcontext.gregs[REG_EIP];
    void *crash_bp = (void*)uc->uc_mcontext.gregs[REG_EBP];
    void *crash_sp = (void*)uc->uc_mcontext.gregs[REG_ESP];
#elif defined(__arm__)
    strcat(log_buffer, ">>> platform arm\n");
    void *crash_pc = (void*)uc->uc_mcontext.arm_pc;
    void *crash_bp = (void*)uc->uc_mcontext.arm_fp;
    void *crash_sp = (void*)uc->uc_mcontext.arm_sp;
#elif defined(__aarch64__)
    strcat(log_buffer, ">>> platform aarch64\n");
    void *crash_pc = (void*)uc->uc_mcontext.pc;
    void *crash_bp = (void*)uc->uc_mcontext.regs[29]; // x29 is FP
    void *crash_sp = (void*)uc->uc_mcontext.sp;
#endif

    sprintf(log_buffer+strlen(log_buffer), ">>> Crash PC: %p\n", crash_pc);
    sprintf(log_buffer+strlen(log_buffer), ">>> Stack Pointer: %p\n", crash_sp);
    sprintf(log_buffer+strlen(log_buffer), ">>> Frame Pointer: %p\n", crash_bp);
    
    array[0] = crash_pc;
    int full_size = 1;
    uintptr_t* current_bp = (uintptr_t*)crash_bp;
    while (current_bp && full_size < BACKTRACE_SIZE) {
        // 返回地址在当前帧指针 + 1 的位置（64位系统是8字节）
        if ((void*)current_bp[1] == NULL) break;
        array[full_size] = (void*)current_bp[1];
        full_size++;
        // 下一个帧指针在当前帧指针指向的位置
        uintptr_t* next_bp = (uintptr_t*)current_bp[0];
        
        // 堆栈边界检查
        if (next_bp == NULL || next_bp <= current_bp) {
            break;
        }
        
        current_bp = next_bp;
    }
    // 获取当前堆栈（包含信号处理框架）
    //int full_size = backtrace(array + 1, BACKTRACE_SIZE - 1);

    strings = backtrace_symbols(array, full_size);
    
    if (strings) {     
        sprintf(log_buffer+strlen(log_buffer), ">>> Actual crash stack (%d frames):\n", full_size);
        for (int i = 0; i < full_size; i++) {
            sprintf(log_buffer+strlen(log_buffer), ">>> [%02d] %s\n", i, strings[i]);
        }
        free(strings);
    }

    LOG2FILE(log_buffer);
    //LOG2FILE("\n\n\n***********maps***********\n");
   // dump_process_maps(pid);

    if(a_saved_file_handle != NULL)
    {
       fclose(a_saved_file_handle);
    }
    // 恢复默认处理
    signal(sig, SIG_DFL);
    raise(sig);
}

// 设置信号处理
static void setup_signal_handlers() {
    struct sigaction sa;
    sa.sa_sigaction = signal_callback_function;
    sa.sa_flags = SA_SIGINFO | SA_RESTART;
    sigemptyset(&sa.sa_mask);
    
    // 注册所有崩溃信号
    for (size_t i = 0; i < sizeof(crash_signals)/sizeof(crash_signals[0]); i++) {
        sigaction(crash_signals[i], &sa, NULL);
    }
    
    // 忽略某些信号，避免干扰
    signal(SIGPIPE, SIG_IGN);  // 管道断开
}

void signal_backtrace_set_file_saved_path(const char* path)
{
    memset(a_backtrace_dir_path, 0, sizeof(a_backtrace_dir_path));
    strcat(a_backtrace_dir_path, path);
}

int signal_backtrace_init(void)
{
    setup_signal_handlers();
    return 0;
}