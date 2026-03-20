#ifndef __SYSTEM_MONITOR_H__
#define __SYSTEM_MONITOR_H__

#include <string>
#include <vector>
#include <unordered_map>
#include <unistd.h>
#include "common_def.h"

#define MAX_PROCESS_NAME_LEN 64
#define UNKNOWN_NAME "Unknown"


typedef enum{
    DATA_SAVED_MODE_LOG = 1 << 0,
    DATA_SAVED_MODE_FILE = 1 << 1,
    DATA_SAVED_MODE_TOPIC = 1 << 2,
} DATA_SAVED_MODE_E;

#define INI_FILE_PATH "/opt/usr/app/1/gea/conf/platform.ini"
#define SYSTEM_MONITOR_SECTION_NAME "system_monitor"
#define SYSTEM_MONITOR_KEY_SAMPLING_INTERVAL "interval"
#define SYSTEM_MONITOR_KEY_DATA_SAVED_PATH "data_saved_path"
#define SYSTEM_MONITOR_KEY_DATA_SAVED_MODE "data_saved_mode"
#define SYSTEM_MONITOR_KEY_LOG_LEVEL "log_level"
#define SYSTEM_MONITOR_KEY_DATA_FILE_SIZE "max_data_file_size"
#define SYSTEM_MONITOR_KEY_DATA_FILE_NUM "max_data_file_num"
#define DEFAULT_SAMPLING_INTERVAL 3000
#define DEFAULT_DATA_SAVED_PATH "./"
#define DEFAULT_DATA_SAVED_MODE DATA_SAVED_MODE_FILE
#define DEFAULT_DATA_FILE_SIZE (10*1024) //KB
#define DEFAULT_DATA_FILE_NUM (10)

typedef struct _CPU_STATS {
    UINT64_T user;
    UINT64_T nice;
    UINT64_T system;
    UINT64_T idle;
    UINT64_T iowait;
    UINT64_T irq;
    UINT64_T softirq;
    UINT64_T steal;
    UINT64_T guest;
    UINT64_T guest_nice;
    // 计算总时间
    UINT64_T total() const {
        return user + nice + system + idle + iowait + irq + softirq + steal;
    }
    
    // 计算空闲时间
    UINT64_T idle_time() const {
        return idle + iowait;
    }
    
    // 计算工作时间
    UINT64_T work_time() const {
        return total() - idle_time();
    }
} CPU_STATS_T;

typedef struct _MEMORY_STATS {
    UINT64_T record_timestamp;
    UINT64_T total;
    UINT64_T free;
    UINT64_T available;
    UINT64_T buffers;
    UINT64_T cached;
    UINT64_T swap_total;
    UINT64_T swap_free;
} MEMORY_STATS_T;

typedef struct _PID_STATS {
    int pid;
    char comm[MAX_PROCESS_NAME_LEN];
   // char state;  //进程状态，可能的值包括： R: 运行中 S: 可中断的睡眠状态 D: 不可中断的睡眠状态 Z: 僵尸进程 T: 停止状态 t: 跟踪停止 X: 死亡状态 
   // int ppid;    //父进程 ID
   // int pgrp;    //进程组 ID
   // int session;  //会话 ID
   // int tty_nr;   //终端设备号
  //  int tpgid;    //终端的前台进程组 ID
  //  UINT64_T flags;  //进程标志位
  //  UINT64_T minflt;  //次要缺页中断次数
  //  UINT64_T cminflt; //子进程的次要缺页中断次数
  //  UINT64_T majflt;  //主要缺页中断次数
  //  UINT64_T cmajflt;  //子进程的主要缺页中断次数
    UINT64_T utime;   //用户态运行时间，单位为 jiffies
    UINT64_T stime;   //内核态运行时间，单位为 jiffies
    UINT64_T cutime;  //子进程的用户态运行时间，单位为 jiffies
    UINT64_T cstime;  //子进程的内核态运行时间，单位为 jiffies
  //  UINT64_T priority;  //进程优先级
  //  UINT64_T nice;      //nice 值，范围为 19 到 -20
  //  UINT64_T num_threads;  //线程数
  //  UINT64_T itrealvalue;  //下一个 SIGALRM 发送的时间，单位为 jiffies
  //  UINT64_T starttime;    //进程启动时间，单位为 jiffies
    UINT64_T vsize;   //虚拟内存大小，单位为字节
    int rss;          //驻留内存大小，单位为页
  //  UINT64_T rsslim;  //驻留内存的软限制，单位为字节
    UINT64_T cpu_total() const {
        return utime + stime + cutime + cstime;
    }
  
} PID_STATS_T;

#pragma pack(4)
typedef struct _PROCESS_STATS {
    int soc_id;
    int pid;
    UINT64_T record_timestamp;
    char name[MAX_PROCESS_NAME_LEN];
    float cpu_usage;
    float memory_usage;
    UINT64_T rss;
    UINT64_T shr;
    UINT64_T pss;
    UINT64_T uss;
    UINT64_T vsize;
    UINT64_T shr_pool;
    UINT64_T read_bytes;
    UINT64_T write_bytes;
} PROCESS_STATS_T;
#pragma pack()
class SystemMonitor {

public:
    SystemMonitor(int soc_id);
    ~SystemMonitor();

    // 获取内存使用情况
    MEMORY_STATS_T currentMemStat()
    {
        return mcurMemStat;
    }
        
    std::vector<float> getInstantCPUUsage()
    {
        return mcurCpuUsage;
    }
    std::vector<CPU_STATS_T> readAllCPUStats();
    std::unordered_map<int, PID_STATS_T> readProcessStates(std::vector<int> &pids);
    std::unordered_map<int, UINT64_T> readSharePool();
    // 获取进程统计信息
    std::vector<PROCESS_STATS_T> getProcessStats();
    
    void sampling();
    UINT64_T lastSamplingTimestamp()
    {
        return mSamplingTimestamp;
    }
    int numberOfCores()
    {
        return mNumCores;
    }
    UINT64_T pageSize()
    {
        return mPageSize;
    }
    UINT64_T memTotal()
    {
        return mMemTotal;
    }
    UINT64_T clockFreq()
    {
        return mClockFreq;
    }
    
private:
    // 获取所有进程列表
    std::vector<int> getProcessList();
    int fillMemoryFromStatm(PROCESS_STATS_T& process);
    int fillMemoryFromSmaps(PROCESS_STATS_T& process);
    int fillMemoryFromSmapsRollup(PROCESS_STATS_T& process);
    MEMORY_STATS_T getTotalMemoryStats();
    std::string getProcessName(int pid);
    std::unordered_map<int, PID_STATS_T> mLastProcessStats;
    std::vector<CPU_STATS_T> mLastCpuStats;
    std::unordered_map<int, PID_STATS_T> mCurProcessStats;
    std::unordered_map<int, UINT64_T> mCurSharePool;
    std::unordered_map<int, std::string> mProcessNameMap;
    std::vector<CPU_STATS_T> mCurCpuStats;
    std::vector<int> mCurPidList;
    UINT64_T mSamplingTimestamp;
    int mNumCores;
    UINT64_T mPageSize;
    UINT64_T mMemTotal;
    UINT64_T mClockFreq;
    MEMORY_STATS_T mcurMemStat;
    std::vector<float> mcurCpuUsage;
    std::vector<float> calculateCpuUsage();
    int mSocId;
};

#endif // __SYSTEM_MONITOR_H__