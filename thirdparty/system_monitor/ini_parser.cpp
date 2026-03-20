
#include "ini_parser.h"

#include <algorithm>
#include <fstream>
#include <sstream>

static std::string trim(const std::string& str) {
    size_t start = str.find_first_not_of(" \t\r\n");
    size_t end = str.find_last_not_of(" \t\r\n");
    if (start == std::string::npos || end == std::string::npos) return "";
    return str.substr(start, end - start + 1);
}

bool INIParser::load(const std::string& filename) {
    // 实现与基础版本相同
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }

    std::string line;
    std::string current_section = "global";

    while (std::getline(file, line)) {
        line = trim(line);

        if (line.empty() || line[0] == ';' || line[0] == '#') {
            continue;
        }

        if (line[0] == '[' && line.back() == ']') {
            current_section = trim(line.substr(1, line.length() - 2));
            continue;
        }

        size_t pos = line.find('=');
        if (pos != std::string::npos) {
            std::string key = trim(line.substr(0, pos));
            std::string value = trim(line.substr(pos + 1));
            data[current_section][key] = value;
        }
    }

    file.close();
    return true;
}

std::string INIParser::getString(const std::string& section, const std::string& key,
                                 const std::string& default_value) {
    return getValue(section, key, default_value);
}

// 获取整数值
int INIParser::getInt(const std::string& section, const std::string& key, int default_value) {
    std::string value = getValue(section, key, "");
    return value.empty() ? default_value : std::stoi(value);
}

// 获取浮点数值
double INIParser::getDouble(const std::string& section, const std::string& key, double default_value) {
    std::string value = getValue(section, key, "");
    return value.empty() ? default_value : std::stod(value);
}

// 获取布尔值（支持true/false, yes/no, 1/0）
bool INIParser::getBool(const std::string& section, const std::string& key, bool default_value) {
    std::string value = getValue(section, key, "");
    if (value.empty()) return default_value;

    // 转换为小写进行比较
    std::string lower_value = value;
    std::transform(lower_value.begin(), lower_value.end(), lower_value.begin(), ::tolower);

    if (lower_value == "true" || lower_value == "yes" || lower_value == "1") {
        return true;
    } else if (lower_value == "false" || lower_value == "no" || lower_value == "0") {
        return false;
    }

    return default_value;
}

// 基础获取方法
std::string INIParser::getValue(const std::string& section, const std::string& key,
                                const std::string& default_value) {
    auto section_it = data.find(section);
    if (section_it == data.end()) {
        return default_value;
    }

    auto key_it = section_it->second.find(key);
    if (key_it == section_it->second.end()) {
        return default_value;
    }

    return key_it->second;
}

// 其他辅助方法...
bool INIParser::hasSection(const std::string& section) { return data.find(section) != data.end(); }

bool INIParser::hasKey(const std::string& section, const std::string& key) {
    auto section_it = data.find(section);
    if (section_it == data.end()) {
        return false;
    }
    return section_it->second.find(key) != section_it->second.end();
}

