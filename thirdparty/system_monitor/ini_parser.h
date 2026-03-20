#ifndef __INI_PARSER_H__
#define __INI_PARSER_H__

#include <cctype>
#include <string>
#include <unordered_map>
#include <vector>

class INIParser {
public:
    bool load(const std::string& filename);
    // 获取字符串值
    std::string getString(const std::string& section, const std::string& key, const std::string& default_value = "");

    // 获取整数值
    int getInt(const std::string& section, const std::string& key, int default_value = 0);

    // 获取浮点数值
    double getDouble(const std::string& section, const std::string& key, double default_value = 0.0);

    // 获取布尔值（支持true/false, yes/no, 1/0）
    bool getBool(const std::string& section, const std::string& key, bool default_value = false);

    // 基础获取方法
    std::string getValue(const std::string& section, const std::string& key, const std::string& default_value = "");

    // 其他辅助方法...
    bool hasSection(const std::string& section);

    bool hasKey(const std::string& section, const std::string& key);

private:

    std::unordered_map<std::string, std::unordered_map<std::string, std::string>> data;
};

#endif