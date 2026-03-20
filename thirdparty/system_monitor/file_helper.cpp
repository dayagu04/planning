

#include "file_helper.h"

#include <assert.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <libgen.h>

#include <sys/file.h>
#include <sys/stat.h>
#include <time.h>
#include <zlib.h>
#include <string.h>
#include "common_def.h"

#define FILE_COMPRESS_LEVEL 6
#define FILE_ROTATION_MAX_FILE_SIZE 30
#define FILE_ROTATION_MAX_FILE_NUM 10
#define COMPRESS_THREAD_NAME "file/compress"

static int _directory_exists(const char* path) {
    struct stat st;
    if (stat(path, &st) == 0) {
        return S_ISDIR(st.st_mode);  // 如果是目录返回1
    }
    return 0;  // 不存在或不是目录
}

// 创建目录（支持多级目录）
static int _create_directory(const char* path) {
    if (!path) {
        return -1;
    }
    char path_copy[MAX_FILE_PATH_LEN] = {};
    strcpy(path_copy, path);
    char* p = path_copy;
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
            if (errno != EEXIST) {  // 如果目录已存在，不视为错误
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
int file_helper_check_to_create_directory(const char* filename) {
    if (!filename) {
        return -1;
    }
    char dir_path[MAX_FILE_PATH_LEN] = {};
    strcpy(dir_path, filename);
    // 获取目录路径
    char* dir_name = dirname(dir_path);

    int result = 0;
    // 如果目录不存在，则创建
    if (!_directory_exists(dir_name)) {
        LOG_INFO("目录不存在，创建目录: %s\n", dir_name);
        result = _create_directory(dir_name);
    }

    return result;
}

std::string generate_filename(const char* dir, const char* file_pre_name, const char* file_suf_name,
                            uint32_t& index) {
    char file_name[MAX_FILE_PATH_LEN] = {};
    time_t now = time(NULL);
    struct tm* t = localtime(&now);
    sprintf(file_name, "%s/%s_%04d%02d%02d%02d%02d%02d_%u.%s", dir, file_pre_name, t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
            t->tm_hour, t->tm_min, t->tm_sec, index, file_suf_name);
    index++;
    return file_name;
}

static int compress_file(const char* src_file, char* dest_file, size_t dest_size) {
    if (!src_file) return RETURN_INVALID_ARG;
    snprintf(dest_file, dest_size, "%s.gz", src_file);
    FILE* src = fopen(src_file, "rb");
    if (!src)
    {
        LOG_WARN("open compress src file:%s failed\n", src_file);
        return RETURN_NOT_FOUND;
    }
     
    gzFile dst = gzopen(dest_file, "wb");
    if (!dst) {
        LOG_WARN("gzopen compress dst file:%s failed\n", dest_file);
        fclose(src);
        return RETURN_FAIL;
    }

    gzsetparams(dst, FILE_COMPRESS_LEVEL, Z_DEFAULT_STRATEGY);

    char buffer[8192];

    size_t bytes_read;
    int ret = RETURN_OK;
    while ((bytes_read = fread(buffer, 1, sizeof(buffer), src)) > 0) {
        int gz_size = gzwrite(dst, buffer, bytes_read);
        if (gz_size <= 0) {
            LOG_WARN("gzwrite failed, ret:%d\n", gz_size);
            ret = RETURN_FAIL;
            break;
        }
    }

    fclose(src);
    gzclose(dst);

    if (ret == RETURN_OK) {
        // 删除原文件
        unlink(src_file);
    } else {
        // 删除压缩失败的文件
        unlink(dest_file);
    }
    return ret;
}

namespace file_helper {
FileHelper::FileHelper(std::string dir, std::string file_pre, std::string file_suf):  mDir(dir), mFilePre(file_pre), mFileSuf(file_suf),
             mFileIndex(0), mMaxSize(FILE_ROTATION_MAX_FILE_SIZE), mMaxNum(FILE_ROTATION_MAX_FILE_NUM), mCurFilePtr_(nullptr), mCurFileSize(0)
{
    mCompressThreadRunning_.store(false);
    initFileList();
    startCompressThread();
}

FileHelper::~FileHelper()
{
    mCompressThreadRunning_.store(false);
    mCompressQueue_.stop();
    if(mCurFilePtr_)
    {
        fclose(mCurFilePtr_);
    }
}

int FileHelper::write_bytes(const char* data, uint32_t size)
{
    if(mCurFileFullName.empty())
    {
        mCurFileFullName = generate_filename(mDir.c_str(), mFilePre.c_str(), mFileSuf.c_str(), mFileIndex);
    }
    if(access(mCurFileFullName.c_str(), F_OK) != 0 || mCurFilePtr_ == NULL)
    {
        if(mCurFilePtr_ != NULL)
        {
            fclose(mCurFilePtr_);
        }
        file_helper_check_to_create_directory(mCurFileFullName.c_str());
        mCurFilePtr_ = fopen(mCurFileFullName.c_str(), "wb");
        if(mCurFilePtr_ == NULL)
        {
            LOG_WARN("create file:%s failed\n", mCurFileFullName.c_str());
            return RETURN_FAIL;
        }
        LOG_WARN("create file:%s success\n", mCurFileFullName.c_str());
        if(!mFileDataTitle.empty())
        {
            size_t act_size = fwrite(mFileDataTitle.c_str(), 1, mFileDataTitle.size(), mCurFilePtr_);
            mCurFileSize += act_size;
        }
    }

    size_t act_size = fwrite(data, 1, size, mCurFilePtr_);
    mCurFileSize += act_size;
    if(mCurFileSize >= (mMaxSize*1024))
    {
        fclose(mCurFilePtr_);
        mCurFilePtr_ = nullptr;
        mCurFileSize = 0;
        mCompressQueue_.push(mCurFileFullName);
        mCurFileFullName.clear();
    }
    return act_size;
}

void FileHelper::setRotationParam(uint32_t max_size, uint32_t max_num, std::string title)
{
    mMaxSize = max_size;
    mMaxNum = max_num;
    mFileDataTitle = title;
}

void FileHelper::initFileList()
{
    DIR* dp = opendir(mDir.c_str());
    if (!dp)
    {
        LOG_WARN("fileHelper opendir failed, dir: %s \n", mDir.c_str());
        return;
    }
    struct dirent* entry;
    while ((entry = readdir(dp))) {
        if (std::string(entry->d_name) == "." || std::string(entry->d_name) == "..")
            continue;
        std::string fullPath = mDir + "/" + entry->d_name;
        // 检查是否是文件
        struct stat sb;
        if (lstat(fullPath.c_str(), &sb) == -1) 
            continue;
        if (!S_ISREG(sb.st_mode))
            continue;
        // 检查前缀后缀
        if (strncmp(mFilePre.c_str(), entry->d_name, mFilePre.size()) == 0) {
            if(strncmp(entry->d_name+(strlen(entry->d_name) - 3), ".gz", 3) == 0)
            {
                LOG_INFO("%s will add to rotation list\n", fullPath.c_str());
                mFileList.insert(fullPath);
            }
            else
            {
                if(sb.st_size <= 0)
                {
                    LOG_INFO("%s is empty file, will delete\n", fullPath.c_str());
                    unlink(fullPath.c_str());
                }
                else
                {
                    LOG_INFO("try to compress %s ... \n", fullPath.c_str());
                    mCompressQueue_.push(fullPath);
                }
            }
        }
    }
    closedir(dp);
}

void FileHelper::startCompressThread()
{
    if(mCompressThreadRunning_.load())
    {
        LOG_WARN("compress thread is already running\n");
        return;
    }
    mCompressThreadHandle_ = std::thread([this]() {
        LOG_INFO("compress thread start\n");
        mCompressThreadRunning_.store(true);
        pthread_setname_np(pthread_self(), COMPRESS_THREAD_NAME);
        if(nice(10) == -1) {
            LOG_WARN("nice failed");
        }
        
        while (mCompressThreadRunning_.load()) {
            std::string src_file;
            bool b_success = mCompressQueue_.wait_and_pop(src_file);
            if(!b_success)
            {
                continue;
            }
            char dst_file_name[MAX_FILE_PATH_LEN] = {};
            int ret = compress_file(src_file.c_str(), dst_file_name, sizeof(dst_file_name));
            if(ret == RETURN_OK)
            {
                src_file = dst_file_name;
            }
            mFileList.insert(src_file);
            while(mFileList.size() > mMaxNum)
            {
                std::string del_file = *mFileList.begin();
                LOG_INFO("delete file: %s\n", del_file.c_str());
                unlink(del_file.c_str());
                mFileList.erase(mFileList.begin());
            }
        }
        LOG_INFO("compress thread exit\n");
    });
    mCompressThreadHandle_.detach();
}

}