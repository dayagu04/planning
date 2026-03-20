#include "system_monitor.h"
#include <dirent.h>
#include <signal.h>
#include <string.h>
#include <sys/time.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#include "ini_parser.h"
#include "file_helper.h"
#include <sys/sysinfo.h>


const int g_soc_id = 0;
int g_current_debug_log_level = LOG_LEVEL_INFO;
int g_debug_print_view_on = 0;
// 全局变量用于控制程序退出
volatile sig_atomic_t stop_flag = 0;

static int a_sampling_interval = DEFAULT_SAMPLING_INTERVAL;
static std::string a_data_saved_path = "./loading";   // DEFAULT_DATA_SAVED_PATH;
static int a_data_saved_mode = DEFAULT_DATA_SAVED_MODE;  // 0: 不保存, 1: 保存到文件, 2: topic发布
static void signal_handler(int) { stop_flag = 1; }
static void printHeader() {
    PRINT_VIEW("\033[2J\033[1;1H");
    PRINT_VIEW("=== Linux系统负载监控 === (刷新间隔: %f秒)\n", (double)a_sampling_interval / 1000);
    PRINT_VIEW("==========================================================\n");
}

// 格式化字节大小为人类可读格式
static std::string formatBytes(UINT64_T bytes) {
    const char* sizes[] = {"B", "KB", "MB", "GB", "TB"};
    int order = 0;
    double size = bytes;

    while (size >= 1024 && order < 4) {
        order++;
        size /= 1024;
    }

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << size << " " << sizes[order];
    return ss.str();
}

static void initFromIniConfig()
{
   // 使用增强版本获取不同类型值
    INIParser adv_parser;
    if (adv_parser.load(INI_FILE_PATH)) {
        a_sampling_interval =
            adv_parser.getInt(SYSTEM_MONITOR_SECTION_NAME, SYSTEM_MONITOR_KEY_SAMPLING_INTERVAL, DEFAULT_SAMPLING_INTERVAL);
        a_data_saved_mode =
            adv_parser.getInt(SYSTEM_MONITOR_SECTION_NAME, SYSTEM_MONITOR_KEY_DATA_SAVED_MODE, DEFAULT_DATA_SAVED_MODE);
        a_data_saved_path = adv_parser.getString(SYSTEM_MONITOR_SECTION_NAME, SYSTEM_MONITOR_KEY_DATA_SAVED_PATH,
                                                 DEFAULT_DATA_SAVED_PATH);
        g_current_debug_log_level = adv_parser.getInt(SYSTEM_MONITOR_SECTION_NAME, SYSTEM_MONITOR_KEY_LOG_LEVEL, LOG_LEVEL_ERR);
    } else {
        LOG_ERROR("%s - %d, can not open ini file: %s \n", __FUNCTION__, __LINE__, INI_FILE_PATH);
    }
    LOG_INFO(" sampling interval: %ums\n", a_sampling_interval);
    LOG_INFO(" data saved mode: %u\n", a_data_saved_mode);
    LOG_INFO(" data saved path: %s\n", a_data_saved_path.c_str());
    LOG_INFO(" debug level: %u\n", g_current_debug_log_level);
}

static void monitoring()
{
    SystemMonitor monitor(g_soc_id);
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::unique_ptr<file_helper::FileHelper> file_helper = nullptr; 
    if(a_data_saved_mode == DATA_SAVED_MODE_FILE)
    {
        char header[512] = "timestamp,pid,process name,cpu,rss,shr,pss,uss,vsize\n";
        file_helper = std::make_unique<file_helper::FileHelper>(a_data_saved_path, "datacapture_soc"+ std::to_string(g_soc_id), "csv");
        file_helper->setRotationParam(50, 5, header);
    }
    else if(a_data_saved_mode == DATA_SAVED_MODE_LOG)
    {
        g_debug_print_view_on = true;
    }
    
    while (!stop_flag) {
        for (int i = 0; i < 10 && !stop_flag; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(a_sampling_interval) / 10);
        }
        printHeader();
        // 获取瞬时CPU使用率
        monitor.sampling();
        auto cpu_usage_map = monitor.getInstantCPUUsage();

        // 显示总体CPU使用率

        PRINT_VIEW("总体CPU使用率: /t %.2f%% (瞬时值) \n", cpu_usage_map[0]);

        // 显示各核心CPU使用率
        PRINT_VIEW("各核心CPU使用率: ->\n");
        for (int i = 1; i <= monitor.numberOfCores(); ++i) {
            PRINT_VIEW("CPU%d:\t %.1f%%\t", i, cpu_usage_map[i - 1]);
            if ((i) % 5 == 0) {
                PRINT_VIEW("\n");
            }
        }

        // 获取并显示内存使用情况
        MEMORY_STATS_T mem = monitor.currentMemStat();
        uint64_t mem_used = mem.total - mem.available;
        uint64_t swap_used = mem.swap_total - mem.swap_free;
        double mem_usage = (mem_used / (double)mem.total) * 100.0;

        double swap_usage =
            (mem.swap_total > 0) ? (swap_used / (double)mem.swap_total) * 100.0 : 0.0;

        PRINT_VIEW("\n内存使用: %.2f%% (%s/%s)\n", mem_usage, formatBytes(mem_used).c_str(),
                   formatBytes(mem.total).c_str());

        PRINT_VIEW("交换区使用: %.2f%% (%s/%s)\n", swap_usage, formatBytes(swap_used).c_str(),
                   formatBytes(mem.swap_total).c_str());
        
        PRINT_VIEW("page size: %llu, freq: %llu\n", monitor.pageSize(), monitor.clockFreq());

        // struct sysinfo info;
        // sysinfo(&info);
        // swap_used = info.totalswap - info.freeswap;
        // mem_used = info.totalram - info.freeram - info.bufferram;
        // mem_usage = (mem_used / (double)info.totalram) * 100.0;
        // swap_usage =
        //     (info.totalswap > 0) ? (swap_used / (double)info.totalswap) * 100.0 : 0.0;
        // PRINT_VIEW("\n内存使用: %.2f%% (%s/%s)\n", mem_usage, formatBytes(mem_used).c_str(),
        //            formatBytes(info.totalram).c_str());
        // PRINT_VIEW("交换区使用: %.2f%% (%s/%s)\n", swap_usage, formatBytes(swap_used).c_str(),
        //            formatBytes(info.totalswap).c_str());

        // 获取进程瞬时CPU使用率
        auto process_cpu_usage = monitor.getProcessStats();
        
        // 显示进程信息
        PRINT_VIEW(
            "PID    \t 进程名             \t CPU%%    \t RSS     \t SHR     \t PSS     \t USS     \t VIRT     \t \n");
        PRINT_VIEW("%s\n", std::string(90, '-').c_str());

        const PROCESS_STATS_T& total = process_cpu_usage[0];
        PRINT_VIEW("%-7d\t %-15s\t %8.2f%%\t %-8s\t %-8s\t %-8s\t %-8s\t %-8s\t \n", total.pid, total.name,
                           total.cpu_usage, formatBytes(total.rss).c_str(), formatBytes(total.shr).c_str(),
                           formatBytes(total.pss).c_str(), formatBytes(total.uss).c_str(),
                           formatBytes(total.vsize).c_str());
        
        // 按CPU使用率排序,倒序
        std::sort(process_cpu_usage.begin(), process_cpu_usage.end(),
                  [](const PROCESS_STATS_T& a, const PROCESS_STATS_T& b) { return a.cpu_usage < b.cpu_usage; });
        
        for (size_t i = 1; i <= process_cpu_usage.size(); ++i) {
            const PROCESS_STATS_T& stats = process_cpu_usage[process_cpu_usage.size() - i];
            if (i <= 10 && stats.pid != 0) {
                PRINT_VIEW("%-7d\t %-15s\t %8.2f%%\t %-8s\t %-8s\t %-8s\t %-8s\t %-8s\t \n", stats.pid, stats.name,
                           stats.cpu_usage, formatBytes(stats.rss).c_str(), formatBytes(stats.shr).c_str(),
                           formatBytes(stats.pss).c_str(), formatBytes(stats.uss).c_str(),
                           formatBytes(stats.vsize).c_str());
            }
            if (file_helper) {
                char process_data[512] = {};
                sprintf(process_data, "%llu,%u,%s,%f,%llu,%llu,%llu,%llu,%llu\n", stats.record_timestamp, stats.pid,
                        stats.name, stats.cpu_usage, stats.rss, stats.shr, stats.pss, stats.uss, stats.vsize);
                file_helper->write_bytes(process_data, strlen(process_data));
            }
        }

        PRINT_VIEW("监控中... 按Ctrl+C退出\n");
    }
    PRINT_VIEW("\n监控程序已退出 \n");
}

int main(int argc, char** argv) {
    const char* optstring = "s";
    int opt;
    while ((opt = getopt(argc, argv, optstring)) != -1) {
        switch (opt) {
        case 's':
            g_debug_print_view_on = 1;
            break;
        default:
            LOG_ERROR("invalid argment for -%d\n", opt);
            return -1;
        }
    }
    initFromIniConfig();
    try {
        monitoring();
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}