#ifndef _FILE_HELPER_H_
#define _FILE_HELPER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <set>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include "ThreadSafeQueue.h"

#define MAX_PRE_DIR_PATH_LEN 128
#define MAX_FILE_PATH_LEN 256

int file_helper_check_to_create_directory(const char *filename);


namespace file_helper {
class FileHelper final {
public:
    FileHelper(std::string dir, std::string file_pre, std::string file_suf);
    ~FileHelper();
    void setRotationParam(uint32_t max_size, uint32_t max_num, std::string title = "");
    int write_bytes(const char* data, uint32_t size);
private:
    void initFileList();
    void startCompressThread();

    std::string mDir;
    std::string mFilePre;
    std::string mFileSuf;
    uint32_t mFileIndex;
    uint32_t mMaxSize;  //KB
    uint32_t mMaxNum;
    std::string mFileDataTitle;
    std::set<std::string> mFileList; //for time order

    std::thread mCompressThreadHandle_;
    iflyauto::ThreadSafeQueue<std::string> mCompressQueue_;
    std::atomic<bool> mCompressThreadRunning_;
    std::string mCurFileFullName;
    FILE* mCurFilePtr_;
    uint32_t mCurFileSize;  //Bytes
};
}

#endif