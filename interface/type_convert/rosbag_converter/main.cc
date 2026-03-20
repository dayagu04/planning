#include <getopt.h>

#include "bag_converter.h"
#include "interface_version/interface_version.h"

#define MAX_PATH_LENGTH 128
#define VERSION_START_CHAR 'v'
#define VERSION_NUMBER_SPLIT_CHAR '.'
#define VERSION_NUMBER_PART_LEN 2
#define ROSBAG_FILE_NAME_SUFFIX ".bag"

bool isValidVersion(const std::string& ver) {
    if (ver.empty() || ver[0] != VERSION_START_CHAR) return false;

    size_t dotCount = count(ver.begin(), ver.end(), VERSION_NUMBER_SPLIT_CHAR);
    if (dotCount != VERSION_NUMBER_PART_LEN - 1) return false;

    for (size_t i = 1; i < ver.size(); ++i) {
        if (ver[i] != VERSION_NUMBER_SPLIT_CHAR && !isdigit(ver[i])) {
            return false;
        }
    }
    return true;
}

size_t findVersionEnd(const std::string& str, size_t start) {
    if (start >= str.size()) return start;
    size_t end = start + 1;  // 跳过'v'
    int dotCount = 0;
    bool lastWasDot = false;
    while (end < str.size()) {
        if (str[end] == VERSION_NUMBER_SPLIT_CHAR) {
            if (lastWasDot || dotCount >= VERSION_NUMBER_PART_LEN - 1) break;
            // 确保点号后是数字
            dotCount++;
            lastWasDot = true;
            if (end + 1 >= str.size() || !isdigit(str[end + 1])) break;
        } else if (!isdigit(str[end])) {
            break;
        }
        else
        {
            lastWasDot =false;
        }
        ++end;
    }
    bool valid = (dotCount == VERSION_NUMBER_PART_LEN - 1);
    return valid ? end : start;
}
size_t findVersionPosition(const std::string& str) {
    size_t pos = str.find(VERSION_START_CHAR);
    while (pos != std::string::npos) {
        // 检查是否符合版本号格式
        if (pos + 1 < str.size() && isdigit(str[pos + 1])) {
            size_t end = findVersionEnd(str, pos);
            if (end != pos) {  // 找到有效版本号
                return pos;
            }
        }
        pos = str.find(VERSION_START_CHAR, pos + 1);
    }
    return std::string::npos;
}
bool processFilename(std::string& filename, const std::string& newVersion) {
    if (!isValidVersion(newVersion)) {
        std::cout << "invalid new version: " << newVersion << std::endl;
        return false;
    }

    size_t versionPos = findVersionPosition(filename);

    if (versionPos != std::string::npos) {
        // 替换现有版本号
        size_t versionEnd = findVersionEnd(filename, versionPos);
        filename.replace(versionPos, versionEnd - versionPos, newVersion);
    } else {
        // 在扩展名前插入新版本号
        size_t extPos = filename.rfind(ROSBAG_FILE_NAME_SUFFIX);
        if (extPos != std::string::npos && extPos > 0) {
            filename.insert(extPos, newVersion);
        } else {
            // 无扩展名，直接追加
            filename += newVersion;
        }
    }
    return true;
}

int main(int argc, char** argv) {
    std::string bag_path, out_bag, log_file;

    int opt, lopt, loidx;
    const char* optstring = "";
    const struct option long_options[] = {{"help", no_argument, &lopt, 1},
                                          {"input", optional_argument, &lopt, 2},
                                          {"output", optional_argument, &lopt, 3},
                                          {"test", optional_argument, &lopt, 4}};

    while ((opt = getopt_long(argc, argv, optstring, long_options, &loidx)) != -1) {
        if (opt == 0) opt = lopt;
        switch (opt) {
        case 1:
            std::cout << "--help             print this message" << std::endl;
            break;
        case 2:
            bag_path = std::string(optarg);
            if (bag_path.empty()) {
                std::cerr << "empty bag path" << std::endl;
                return -1;
            }
            break;
        case 3:
            out_bag = std::string(optarg);
            break;
        case 4:
            break;
        default:
            std::cerr << "unknown option " << opt << std::endl;
            return -1;
        }
    }

    if (out_bag.empty()) {
        out_bag = bag_path;
        processFilename(out_bag, std::string(AUTONOMOUS_DRIVING_INTERFACE_VERSION).replace(0, strlen("interface"),"v"));
    }
    bag_converter(bag_path, out_bag);
    return 0;
}