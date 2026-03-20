/* *
 * FUNCTION: Define MdcYamlNode Class
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#ifndef MDC_YAML_NODE_H
#define MDC_YAML_NODE_H

#include "yaml.h"
#include <string>
#include <memory>
#include <map>
#include <vector>
#include <algorithm>
#include <iostream>
#include <functional>
#include <unordered_map>
#include <stack>
namespace mdc {
 
enum YamlMapObjectType {
    MAP_OBJ_UNINIT,
    MAP_OBJ_VECTOR,
    MAP_OBJ_MAP,
    MAP_OBJ_VALUE,
    MAP_OBJ_FAILED
};
 
class MdcYamlNode {
public:
    using YamlMapping = std::map<std::string, class MdcYamlNode>;
    using YamlFuncType = std::function<void(MdcYamlNode &)>;
    MdcYamlNode() = default;
    MdcYamlNode(std::string fileName);
    MdcYamlNode(const MdcYamlNode&) = default;
    MdcYamlNode& operator=(const MdcYamlNode&) & = default;
    MdcYamlNode(MdcYamlNode&&) = default;
    MdcYamlNode& operator=(MdcYamlNode&&) & = default;
    ~MdcYamlNode() = default;
 
    template <class T> bool GetValue(const std::string &key, T &value) const;
 
    bool HasKeyValue(const std::string &key) const
    {
        if (nodeMap.count(key) > 0) {
            return true;
        }
        return false;
    }
 
    const mdc::MdcYamlNode operator[](const std::string &key) const;

    template <class T> T as() const;
    std::vector<class MdcYamlNode> GetNode() const
    {
        return mapObjects;
    }
private:
    bool Init();
    bool LoadYamlFile();
 
    void SetType(const YamlMapObjectType &typeValue)
    {
        type_ = typeValue;
    }
    bool CompareType(const YamlMapObjectType &typeValue)
    {
        return (type_ == typeValue);
    }
 
    std::string configFileName;
    std::string value{};
    std::vector<class MdcYamlNode> mapObjects;
    YamlMapping nodeMap;
    YamlMapObjectType type_ { MAP_OBJ_UNINIT };
    bool current_is_key { false };
    std::vector<std::string> keys;
    bool NotNull(const std::string val) const;
    void as_if(bool &rhs) const;
    void as_if(int32_t &rhs) const;
    void as_if(int64_t &rhs) const;
    void as_if(uint32_t &rhs) const;
    void as_if(size_t &rhs) const;
    void as_if(std::string &rhs) const;
    void as_if(std::vector<int32_t> &rhs) const;
    void as_if(std::vector<uint32_t> &rhs) const;
    void as_if(std::vector<size_t> &rhs) const;
    void as_if(std::vector<std::string> &rhs) const;
 
    void SetDoneProcess();
    void DocumentStartEventProcess();
    void DocumentEndEventProcess();
    void SequenceStartEventProcess(MdcYamlNode &mapObj);
    void SequenceEndEventProcess();
    void MapStartEventProcess(MdcYamlNode &mapObj);
    void MapEndEventProcess();
    void ScaleEventProcess(MdcYamlNode &mapObj);
    std::function<void()> nullProcess = []() {};
 
    std::unordered_map<yaml_event_type_t, YamlFuncType> table { 
        { YAML_NO_EVENT, std::bind(&MdcYamlNode::SetDoneProcess, this) },
        { YAML_STREAM_START_EVENT, std::bind(&MdcYamlNode::nullProcess, this) },
        { YAML_STREAM_END_EVENT, std::bind(&MdcYamlNode::SetDoneProcess, this) },
        { YAML_DOCUMENT_START_EVENT, std::bind(&MdcYamlNode::DocumentStartEventProcess, this) },
        { YAML_DOCUMENT_END_EVENT, std::bind(&MdcYamlNode::DocumentEndEventProcess, this) },
        { YAML_SEQUENCE_START_EVENT, std::bind(&MdcYamlNode::SequenceStartEventProcess, this, std::placeholders::_1) },
        { YAML_SEQUENCE_END_EVENT, std::bind(&MdcYamlNode::SequenceEndEventProcess, this) },
        { YAML_MAPPING_START_EVENT, std::bind(&MdcYamlNode::MapStartEventProcess, this, std::placeholders::_1) },
        { YAML_MAPPING_END_EVENT, std::bind(&MdcYamlNode::MapEndEventProcess, this) },
        { YAML_ALIAS_EVENT, std::bind(&MdcYamlNode::nullProcess, this) },
        { YAML_SCALAR_EVENT, std::bind(&MdcYamlNode::ScaleEventProcess, this, std::placeholders::_1) } };
 
    std::string lastKey;
    bool isReady { false };
    std::stack<MdcYamlNode *> dataStack {};
    yaml_event_t event {};
    bool parseDone { false };
};
}
#endif