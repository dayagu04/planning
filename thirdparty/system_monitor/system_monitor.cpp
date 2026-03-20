

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
#include <fcntl.h>
#include <unistd.h>

#define MAX_PROCESS_NAME_LEN_IN_STAT 15

// 获取当前时间戳（秒级）
// static time_t getCurrentTimestamp() { return std::time(nullptr); }

// 获取当前时间戳（毫秒级）
static int64_t getCurrentTimestampMs() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

// 获取当前时间戳（微秒级）
// static int64_t getCurrentTimestampUs() {
//     struct timeval tv;
//     gettimeofday(&tv, nullptr);
//     return (int64_t)tv.tv_sec * 1000000 + tv.tv_usec;
// }

static void extractFilename(std::string& process_name) {
    size_t last_slash = process_name.find_last_of("/\\");
    if (last_slash != std::string::npos) {
        process_name.erase(0, last_slash + 1);
    }
}
SystemMonitor::SystemMonitor(int soc_id) {
    // 获取CPU核心数量
    mNumCores = sysconf(_SC_NPROCESSORS_ONLN);
    mPageSize = sysconf(_SC_PAGESIZE);
    mClockFreq = sysconf(_SC_CLK_TCK);
    mSocId = soc_id;

    mcurMemStat = getTotalMemoryStats();
    mMemTotal = mcurMemStat.total;
    // 第一次采样
    mCurPidList = getProcessList();
    mCurProcessStats = readProcessStates(mCurPidList);
    mCurCpuStats = readAllCPUStats();
    mSamplingTimestamp = getCurrentTimestampMs();
}
SystemMonitor::~SystemMonitor() {}

// 读取所有CPU的统计信息
std::vector<CPU_STATS_T> SystemMonitor::readAllCPUStats() {
    std::vector<CPU_STATS_T> stats_map(mNumCores + 1);
    std::ifstream file("/proc/stat");
    std::string line;

    while (std::getline(file, line)) {
        if (line.substr(0, 3) == "cpu") {
            std::istringstream iss(line);
            std::string cpu_label;
            iss >> cpu_label;

            int cpu_id = 0;  // 总体
            if (cpu_label.length() > 3) {
                cpu_id = std::stoi(cpu_label.substr(3)) + 1;
            }
            if (cpu_id > mNumCores) {
                LOG_ERROR("%s - %d, cpu id %d is out of range\n", __FUNCTION__, __LINE__, cpu_id);
                continue;
            }
            CPU_STATS_T stats;
            iss >> stats.user >> stats.nice >> stats.system >> stats.idle >> stats.iowait >> stats.irq >>
                stats.softirq >> stats.steal >> stats.guest >> stats.guest_nice;

            stats_map[cpu_id] = stats;
        }
    }

    return stats_map;
}
std::unordered_map<int, UINT64_T> SystemMonitor::readSharePool(){
    std::unordered_map<int, UINT64_T> shp;
    std::ifstream file("/proc/sharepool/proc_overview");
    std::string line;
    std::getline(file, line); //skip the first line for title
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        int pid;
        std::string comm;
        UINT64_T sp_alloc;
        iss >> pid >> comm >> sp_alloc;
        int multiplier = 1;
        shp[pid] = sp_alloc * multiplier;
    }

    return shp;
}

MEMORY_STATS_T SystemMonitor::getTotalMemoryStats() {
    MEMORY_STATS_T stats = {};
    std::ifstream file("/proc/meminfo");
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string key;
        UINT64_T value;
        std::string unit;

        iss >> key >> value >> unit;
        int multiplier = 1;
        if (unit == "kB") multiplier = 1024;

        if (key == "MemTotal:")
            stats.total = value * multiplier;
        else if (key == "MemFree:")
            stats.free = value * multiplier;
        else if (key == "MemAvailable:")
            stats.available = value * multiplier;
        else if (key == "Buffers:")
            stats.buffers = value * multiplier;
        else if (key == "Cached:")
            stats.cached = value * multiplier;
        else if (key == "SwapTotal:")
            stats.swap_total = value * multiplier;
        else if (key == "SwapFree:")
            stats.swap_free = value * multiplier;
    }

    return stats;
}

std::vector<int> SystemMonitor::getProcessList() {
    std::vector<int> pids;
    DIR* dir = opendir("/proc");
    dirent* entry;
    if (dir == nullptr) return pids;

    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_type == DT_DIR) {
            std::string name = entry->d_name;
            if (std::all_of(name.begin(), name.end(), ::isdigit)) {
                pids.push_back(std::stoi(name));
            }
        }
    }

    closedir(dir);
    return pids;
}

std::string SystemMonitor::getProcessName(int pid)
{
    std::unordered_map<int, std::string>::iterator it = mProcessNameMap.find(pid);
    if(it != mProcessNameMap.end()){
        return it->second;
    }
    std::string process_name;
    // std::ifstream cmdline_file("/proc/" + std::to_string(pid) + "/cmdline");
    // if (cmdline_file.is_open()) {
    //     std::getline(cmdline_file, process_name);
    //     extractFilename(process_name);
    //     cmdline_file.close();
    //     if(!process_name.empty())
    //     {
    //         mProcessNameMap[pid] = process_name;
    //     }
    // }
    std::string exe_path = "/proc/" + std::to_string(pid) + "/exe";
    char buffer[PATH_MAX];
    ssize_t len = readlink(exe_path.c_str(), buffer, sizeof(buffer) - 1);
    if (len != -1) {
        buffer[len] = '\0';
        process_name = buffer;
        extractFilename(process_name);
    }
    return process_name;
}
std::vector<PROCESS_STATS_T> SystemMonitor::getProcessStats() {
    std::vector<PROCESS_STATS_T> process_stats;

    //insert the total loading, pid is 0
    PROCESS_STATS_T total_state = {};
    total_state.record_timestamp = mSamplingTimestamp;
    total_state.soc_id = mSocId;
    total_state.pid = 0;
    strcpy(total_state.name, "total");
    total_state.cpu_usage = mcurCpuUsage[0];
    float mem_usage = ((mcurMemStat.total - mcurMemStat.available) / (float)mcurMemStat.total) * 100.0;
    total_state.memory_usage = mem_usage;
    total_state.rss = mcurMemStat.total - mcurMemStat.available;
    process_stats.push_back(total_state);

    //insert process loading
    for (int pid : mCurPidList) {
        auto ite_1 = mLastProcessStats.find(pid);
        auto ite_2 = mCurProcessStats.find(pid);
        if (ite_1 == mLastProcessStats.end() || ite_2 == mCurProcessStats.end()) {
            continue;
        }
        PID_STATS_T pid_stat_1 = ite_1->second;
        PID_STATS_T pid_stat_2 = ite_2->second;
        PROCESS_STATS_T stats = {};
        stats.pid = pid;
        stats.soc_id =mSocId;
        UINT64_T pid_time_diff = pid_stat_2.cpu_total() - pid_stat_1.cpu_total();
        UINT64_T system_time_diff = mCurCpuStats[0].total() - mLastCpuStats[0].total();
        stats.cpu_usage = (pid_time_diff * 100.0 * mNumCores) / system_time_diff;
        stats.record_timestamp = mSamplingTimestamp;
        stats.rss = pid_stat_2.rss;
        stats.vsize = pid_stat_2.vsize;
        stats.memory_usage = (stats.rss / (float)mMemTotal) * 100.0;
        if (stats.cpu_usage == 0 || stats.rss == 0 || stats.vsize == 0)  // 如果cpu占用为0的进程，直接跳过
        {
            LOG_DEBUG("Skip process[%d] cpu:%f, rss:%llu, vsize:%llu \n", pid, stats.cpu_usage, stats.rss, stats.vsize);
            continue;
        }
        // fillMemoryFromStatm(stats);
        fillMemoryFromSmaps(stats);

        std::string process_name;
        if(strlen(pid_stat_2.comm) >= MAX_PROCESS_NAME_LEN_IN_STAT)
        {
            process_name = getProcessName(pid);
        }
        if (process_name.empty()) {
            process_name = pid_stat_2.comm;
        }

        if (process_name.empty()) {
            strcpy(stats.name, UNKNOWN_NAME);
        } else {
            if (process_name.length() > MAX_PROCESS_NAME_LEN) {
                strcpy(stats.name,
                       process_name.substr(0, MAX_PROCESS_NAME_LEN - 1).c_str());
            } else {
                strcpy(stats.name, process_name.c_str());
            }
        }

        stats.shr_pool = mCurSharePool[stats.pid];

        // 获取进程IO统计（内存带宽）
        std::ifstream io_file("/proc/" + std::to_string(pid) + "/io");
        if (io_file.is_open()) {
            std::string line;
            while (std::getline(io_file, line)) {
                std::istringstream iss(line);
                std::string key;
                UINT64_T value;
                iss >> key >> value;

                if (key == "read_bytes:")
                    stats.read_bytes = value;
                else if (key == "write_bytes:")
                    stats.write_bytes = value;
            }
        }

        process_stats.push_back(stats);
    }

    return process_stats;
}

int SystemMonitor::fillMemoryFromStatm(PROCESS_STATS_T& process) {
    std::string filename = "/proc/" + std::to_string(process.pid) + "/statm";
    std::ifstream file(filename);
    if (!file.is_open()) {
        LOG_WARN("%s - %d, failed to open file: %s \n", __FUNCTION__, __LINE__, filename.c_str());
        return RETURN_FAIL;
    }

    long size, resident, share;
    file >> size >> resident >> share;

    process.rss = resident * mPageSize;
    process.shr = share * mPageSize;
    file.close();

    return RETURN_OK;
}

/*
int SystemMonitor::fillMemoryFromSmapsRollup(PROCESS_STATS_T& process) {    
    
    std::string filename = "/proc/" + std::to_string(process.pid) + "/smaps_rollup";
    std::ifstream file(filename);

    if (!file.is_open()) {
        LOG_WARN("%s - %d, failed to open file: %s \n", __FUNCTION__, __LINE__, filename.c_str());
        return RETURN_FAIL;
    }

    process.pss = 0;
    long private_clean = 0;
    long private_dirty = 0;
    long shared_clean = 0;
    long shared_dirty = 0;

    std::string line;
    while (std::getline(file, line)) {
        if (line.find("Pss:") == 0) {
            process.pss = std::stol(line.substr(4));
        } else if (line.find("Private_Clean:") == 0) {
            private_clean = std::stol(line.substr(14));
        } else if (line.find("Private_Dirty:") == 0) {
            private_dirty = std::stol(line.substr(14));
        } else if (line.find("Shared_Clean:") == 0) {
            shared_clean = std::stol(line.substr(13));
        } else if (line.find("Shared_Dirty:") == 0) {
            shared_dirty = std::stol(line.substr(13));
        }
    }

    process.uss = private_clean + private_dirty;  // USS = 私有内存
    process.shr = shared_clean + shared_dirty;    // SHR = 共享内存
    process.pss *= 1024;
    process.uss *= 1024;
    process.shr *= 1024;
    return RETURN_OK;
}
*/

int SystemMonitor::fillMemoryFromSmapsRollup(PROCESS_STATS_T& process) {    
    
    char path[64];
    snprintf(path, sizeof(path), "/proc/%d/smaps_rollup", process.pid);
    
    int fd = open(path, O_RDONLY);
    if (fd < 0)
    {
       // LOG_WARN("%s - %d, failed to open file: %s \n", __FUNCTION__, __LINE__, path);
        return RETURN_FAIL;
    } 
    
    char buffer[2048];
    ssize_t bytes_read = read(fd, buffer, sizeof(buffer) - 1);
    close(fd);
    
    if (bytes_read <= 0) return false;
    buffer[bytes_read] = '\0';
    
    process.pss = 0;
    long private_clean = 0;
    long private_dirty = 0;
    long shared_clean = 0;
    long shared_dirty = 0;
    // 解析聚合数据，行数少很多
    char* pos = buffer;    
    while (pos && *pos) {
        if (strncmp(pos, "Pss:", 4) == 0) {
            process.pss = strtoul(pos + 4, nullptr, 10);
        } else if (strncmp(pos, "Private_Clean:", 14) == 0) {
            private_clean = strtoul(pos + 14, nullptr, 10);
        } else if (strncmp(pos, "Private_Dirty:", 14) == 0) {
            private_dirty = strtoul(pos + 14, nullptr, 10);
        } else if (strncmp(pos, "Shared_Clean:", 13) == 0) {
            shared_clean = strtoul(pos + 13, nullptr, 10);
        } else if (strncmp(pos, "Shared_Dirty:", 13) == 0) {
            shared_dirty = strtoul(pos + 13, nullptr, 10);
        }
        pos = strchr(pos, '\n');
        if (pos) pos++;
    }

    process.uss = private_clean + private_dirty;  // USS = 私有内存
    process.shr = shared_clean + shared_dirty;    // SHR = 共享内存
    process.pss *= 1024;
    process.uss *= 1024;
    process.shr *= 1024;
    return RETURN_OK;
}
int SystemMonitor::fillMemoryFromSmaps(PROCESS_STATS_T& process) {
 
    int ret =  fillMemoryFromSmapsRollup(process);
    if(ret == RETURN_OK)
    {
        return ret;
    }
    
    std::string filename = "/proc/" + std::to_string(process.pid) + "/smaps";
    std::ifstream file(filename);

    if (!file.is_open()) {
       // LOG_WARN("%s - %d, failed to open file: %s \n", __FUNCTION__, __LINE__, filename.c_str());
        return RETURN_FAIL;
    }

    process.pss = 0;
    long private_clean = 0;
    long private_dirty = 0;
    long shared_clean = 0;
    long shared_dirty = 0;

    std::string line;
    while (std::getline(file, line)) {
        if (line.find("Pss:") == 0) {
            process.pss += std::stol(line.substr(4));
        } else if (line.find("Private_Clean:") == 0) {
            private_clean += std::stol(line.substr(14));
        } else if (line.find("Private_Dirty:") == 0) {
            private_dirty += std::stol(line.substr(14));
        } else if (line.find("Shared_Clean:") == 0) {
            shared_clean += std::stol(line.substr(13));
        } else if (line.find("Shared_Dirty:") == 0) {
            shared_dirty += std::stol(line.substr(13));
        }
    }

    process.uss = private_clean + private_dirty;  // USS = 私有内存
    process.shr = shared_clean + shared_dirty;    // SHR = 共享内存
    process.pss *= 1024;
    process.uss *= 1024;
    process.shr *= 1024;

    file.close();
    return RETURN_OK;
}

// 获取瞬时CPU使用率（基于非常短的时间间隔）
std::vector<float> SystemMonitor::calculateCpuUsage() {
    std::vector<float> usage_map(mNumCores + 1);
    // 计算每个CPU的使用率
    for (int cpu_id = 0; cpu_id < mNumCores + 1; cpu_id++) {
        const CPU_STATS_T& prev = mLastCpuStats[cpu_id];
        const CPU_STATS_T& curr = mCurCpuStats[cpu_id];

        UINT64_T total_diff = curr.total() - prev.total();
        UINT64_T work_diff = curr.work_time() - prev.work_time();

        if (total_diff > 0) {
            float usage = (float)work_diff / total_diff * 100.0;
            usage_map[cpu_id] = usage;
        } else {
            usage_map[cpu_id] = 0.0;
        }
    }

    return usage_map;
}

// 读取进程状态
std::unordered_map<int, PID_STATS_T> SystemMonitor::readProcessStates(std::vector<int>& pids) {
    std::unordered_map<int, PID_STATS_T> process_states;
    for (int pid : pids) {
        try {
            std::ifstream stat_file("/proc/" + std::to_string(pid) + "/stat");
            if (stat_file.is_open()) {
                std::string line;
                std::getline(stat_file, line);
                std::istringstream iss(line);
                PID_STATS_T stats = {};
                stats.pid = pid;
                size_t start = line.find('(');
                size_t end = line.rfind(')');
                if (start != std::string::npos && end != std::string::npos) {
                    strcpy(stats.comm, line.substr(start + 1, end - start - 1).c_str());
                }

                // 跳过前13个字段
                std::string dummy;
                for (int i = 0; i < 13; ++i) iss >> dummy;
                iss >> stats.utime >> stats.stime >> stats.cutime >> stats.cstime;
                for (int i = 0; i < 5; ++i) iss >> dummy;
                iss >> stats.vsize >> stats.rss;
                stats.rss *= mPageSize;
                process_states[pid] = stats;
            }
        } catch (...) {
            // 忽略无法访问的进程
        }
    }
    return process_states;
}

void SystemMonitor::sampling() {
    mSamplingTimestamp = getCurrentTimestampMs();
    mLastCpuStats = mCurCpuStats;
    mCurCpuStats = readAllCPUStats();
    mLastProcessStats = mCurProcessStats;
    mCurPidList = getProcessList();
    mCurProcessStats = readProcessStates(mCurPidList);
    mcurMemStat = getTotalMemoryStats();
    mcurCpuUsage = calculateCpuUsage();
    mCurSharePool = readSharePool();
}

