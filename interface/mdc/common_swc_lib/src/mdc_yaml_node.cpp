/* *
 * FUNCTION: Define MdcYamlNode Class
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#include <stack>
#include <cstdio>
#include <cerrno>
#include <string>
#include <sstream>
#include <unistd.h>
#include "mdc/utility/mdc_yaml_node.h"

namespace mdc {
MdcYamlNode::MdcYamlNode(const std::string fileName)
{
    configFileName = fileName;
    isReady = Init();
    if (isReady) {
        isReady = LoadYamlFile();
    }
}
 
template <class T> bool MdcYamlNode::GetValue(const std::string &key, T &value) const
{
    if (nodeMap.count(key) > 0) {
        value = static_cast<std::map<std::string, mdc::MdcYamlNode> >(nodeMap)[key].as<T>();
        return true;
    }
    // std::cout <<"Key: "<<key <<" is not exist!";
    return false;
}
 
template bool MdcYamlNode::GetValue<bool>(const std::string &key, bool &value) const;
template bool MdcYamlNode::GetValue<uint32_t>(const std::string &key, uint32_t &value) const;
template bool MdcYamlNode::GetValue<int32_t>(const std::string &key, int32_t &value) const;
template bool MdcYamlNode::GetValue<int64_t>(const std::string &key, int64_t &value) const;
template bool MdcYamlNode::GetValue<size_t>(const std::string &key, size_t &value) const;
template bool MdcYamlNode::GetValue<std::string>(const std::string &key, std::string &value) const;
template bool MdcYamlNode::GetValue<std::vector<uint32_t>>(const std::string &key, std::vector<uint32_t> &value) const;
template bool MdcYamlNode::GetValue<std::vector<int32_t>>(const std::string &key, std::vector<int32_t> &value) const;
template bool MdcYamlNode::GetValue<std::vector<size_t>>(const std::string &key, std::vector<size_t> &value) const;
template bool MdcYamlNode::GetValue<std::vector<std::string>>(const std::string &key, std::vector<std::string> &value) const;
 
const mdc::MdcYamlNode MdcYamlNode::operator[](const std::string &key) const
{
    if (nodeMap.count(key) > 0) {
        return static_cast<std::map<std::string, mdc::MdcYamlNode> >(nodeMap)[key];
    }
    // std::cout <<"Key: "<<key <<" is not exist!";
    return mdc::MdcYamlNode();
    
}
 
template <class T> T MdcYamlNode::as() const
{
    T t;
    as_if(t);
    return t;
}
 
template bool MdcYamlNode::as<bool>() const;
template uint32_t MdcYamlNode::as<uint32_t>() const;
template int32_t MdcYamlNode::as<int32_t>() const;
template int64_t MdcYamlNode::as<int64_t>() const;
template size_t MdcYamlNode::as<size_t>() const;
template std::string MdcYamlNode::as<std::string>() const;
template std::vector<uint32_t> MdcYamlNode::as<std::vector<uint32_t>>() const;
template std::vector<int32_t> MdcYamlNode::as<std::vector<int32_t>>() const;
template std::vector<size_t> MdcYamlNode::as<std::vector<size_t>>() const;
template std::vector<std::string> MdcYamlNode::as<std::vector<std::string>>() const;
 
bool MdcYamlNode::Init()
{
    if (configFileName.empty()) {
        std::cout << "The configuration file name is empty." << &std::endl;
        return false;
    }
    if (access(configFileName.c_str(), R_OK) < 0) {
        std::cout << "Filed is not readable! file name is "<< configFileName << &std::endl;
        return false;
    }
    return true;
}
 
bool MdcYamlNode::LoadYamlFile()
{
    std::unique_ptr<FILE, decltype(&fclose)> fh(fopen(configFileName.c_str(), "rb"), fclose);
    if (!fh) {
        std::cerr << "Failed to open file! file name is "<< configFileName << &std::endl;
        return false;
    }

    yaml_parser_t parser {};
    if (yaml_parser_initialize(&parser) == 0) {
        std::cout << "Failed to initialize parser!" << &std::endl;
        return false;
    }
    try {
        yaml_parser_set_input_file(&parser, fh.get());
    } catch (const std::exception &ex) {
        std::cerr << "Yaml parser failed to set input file " << ex.what() << &std::endl;
        return false;
    }

    parseDone = false;
    do {
        if (yaml_parser_parse(&parser, &event) == 0) {
            yaml_parser_delete(&parser);
            std::cerr << "Parser error is "<< parser.error << &std::endl;
            return false;
        }
        MdcYamlNode mapObj;

        if (table.find(event.type) == table.end()) {
            std::cerr << "Yaml event type is not supported!" << &std::endl;
            return false;
        }
        table[event.type](mapObj);
 
        yaml_event_delete(&event);
    } while (!parseDone); 
 
    yaml_parser_delete(&parser);
    return true;
}

void MdcYamlNode::as_if(bool &rhs) const
{
    if ((value.size() > 0) && (NotNull(value))) {
        rhs = (value == "true") ? true : false;
    }
    return;
}
 
void mdc::MdcYamlNode::as_if(int32_t &rhs) const
{
    if ((value.size() > 0) && (NotNull(value))) {
        rhs = stoi(value);
    }
    return;
}
 
void mdc::MdcYamlNode::as_if(int64_t &rhs) const
{
    if ((value.size() > 0) && (NotNull(value))) {
        rhs = stoi(value);
    }
    return;
}
 
void mdc::MdcYamlNode::as_if(uint32_t &rhs) const
{
    if ((value.size() > 0) && (NotNull(value))) {
        rhs = stoi(value);
    }
    return;
}
 
void mdc::MdcYamlNode::as_if(size_t &rhs) const
{
    if ((value.size() > 0) && (NotNull(value))) {
        rhs = stoi(value);
    }
    return;
}
 
void MdcYamlNode::as_if(std::string &rhs) const
{
    if (NotNull(value)) {
        rhs = value;
    }
    return;
}
 
void MdcYamlNode::as_if(std::vector<int32_t> &rhs) const
{
    for (auto &obj : mapObjects) {
        if (NotNull(obj.value)) {
            rhs.push_back(stoi(obj.value));
        }
    }
    return;
}
 
void MdcYamlNode::as_if(std::vector<uint32_t> &rhs) const
{
    for (auto &obj : mapObjects) {
        if (NotNull(obj.value)) {
            rhs.push_back(stoi(obj.value));
        }
    }
    return;
}
 
void MdcYamlNode::as_if(std::vector<size_t> &rhs) const
{
    for (auto &obj : mapObjects) {
        if (NotNull(obj.value)) {
            rhs.push_back(stoi(obj.value));
        }
    }
    return;
}
 
void MdcYamlNode::as_if(std::vector<std::string> &rhs) const
{
    for (auto &obj : mapObjects) {
        if (NotNull(obj.value)) {
            rhs.push_back(obj.value);
        }
    }
    return;
}
 
bool MdcYamlNode::NotNull(std::string val) const
{
    if ((val.size() > 0) && (val == "null")) {
        // std::cout << "The value is null, please check the configuration file.";
        return false;
    }
    return true;
}
void MdcYamlNode::SetDoneProcess()
{
    parseDone = true;
    return;
}

void MdcYamlNode::DocumentStartEventProcess()
{
    dataStack.push(this);
    return;
}

void MdcYamlNode::DocumentEndEventProcess()
{
    if (dataStack.top()->current_is_key) {
        dataStack.top()->value = dataStack.top()->keys.front();
        dataStack.top()->keys.pop_back();
    }
    dataStack.pop();
    return;
}
 
void MdcYamlNode::MapStartEventProcess(MdcYamlNode &mapObj)
{
    if (dataStack.top()->keys.size() && dataStack.top()->current_is_key) {
        dataStack.push(&dataStack.top()->nodeMap[dataStack.top()->keys.back()]);
    } else if (dataStack.top()->CompareType(MAP_OBJ_VECTOR)) {
        dataStack.top()->mapObjects.push_back(mapObj);
        dataStack.push(&dataStack.top()->mapObjects.back());
    } else {
        dataStack.push(dataStack.top());
    }
    dataStack.top()->SetType(MAP_OBJ_MAP);
    return;
}
void MdcYamlNode::MapEndEventProcess()
{
    if (dataStack.top()->keys.size()) {
        dataStack.top()->keys.erase(std::unique(dataStack.top()->keys.begin(), dataStack.top()->keys.end()),
            dataStack.top()->keys.end());
    }
    if (!dataStack.empty())
        dataStack.pop();
    if ((!dataStack.empty()) && (dataStack.top()->CompareType(MAP_OBJ_MAP))) {
        dataStack.top()->current_is_key = false;
    }
    return;
}
 
void MdcYamlNode::SequenceStartEventProcess(MdcYamlNode &mapObj)
{
    if (dataStack.top()->keys.size() && dataStack.top()->current_is_key) {
        lastKey = dataStack.top()->keys.back();
        dataStack.top()->nodeMap[lastKey] = mapObj;
        dataStack.top()->current_is_key = false;
        dataStack.push(&dataStack.top()->nodeMap[lastKey]);
    } else if (dataStack.top()->CompareType(MAP_OBJ_VECTOR)) {
        dataStack.top()->mapObjects.push_back(mapObj);
        dataStack.push(&dataStack.top()->mapObjects.back());
    } else {
        dataStack.push(dataStack.top());
    }
    dataStack.top()->SetType(MAP_OBJ_VECTOR);
    return;
}
 
void MdcYamlNode::SequenceEndEventProcess()
{
    if (!dataStack.empty())
        dataStack.pop();
    return;
}
 
void MdcYamlNode::ScaleEventProcess(MdcYamlNode &mapObj)
{
    std::stringstream istream;
    istream << event.data.scalar.value; 
    if (!dataStack.top()->current_is_key) {
        if (find(dataStack.top()->keys.begin(), dataStack.top()->keys.end(),
            istream.str()) != dataStack.top()->keys.end()) {
            // std::cout << "error error";
            return;
        }
    }
    if ((dataStack.top()->CompareType(MAP_OBJ_MAP)) && (!dataStack.top()->current_is_key)) {
        dataStack.top()->keys.push_back(istream.str());
        dataStack.top()->current_is_key = true;
    } else if ((dataStack.top()->CompareType(MAP_OBJ_MAP)) && (dataStack.top()->current_is_key)) {
        std::string key = dataStack.top()->keys.back();
        mapObj.value = istream.str();
        mapObj.SetType(MAP_OBJ_VALUE);
        dataStack.top()->nodeMap[key] = mapObj;
        dataStack.top()->current_is_key = false;
    } else if (dataStack.top()->CompareType(MAP_OBJ_VECTOR)) {
        mapObj.value = istream.str();
        mapObj.SetType(MAP_OBJ_VALUE);
        dataStack.top()->mapObjects.push_back(mapObj);
    } else {
        dataStack.top()->value = istream.str();
        dataStack.top()->SetType(MAP_OBJ_VALUE);
    }
    return;
}
}
