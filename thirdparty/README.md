# ad_third_party_libraries

智能驾驶产品线-依赖的开源代码

## 用途
存储讯飞智能驾驶产品线依赖的开源第三方代码。

### 开源库列表
1. **acado** ：(LGPL)用于自动控制和动态优化的软件环境和算法集合。
2. **eigen** ：(MPL2)线性算术的 C++ 模板库。
3. **hfsm** ：(MIT)hierarchical state machine for games and interactive applications，分层优先状态机。
4. **nlohmann_json** ：(MIT)易于集成和使用的json解析库，近似python。
5. **osqp** ：(Apache2.0)用于路径规划的二次规划求解器。
6. **rapidjson** ：(MIT)一个C++的JSON解析器及生成器，代码小，速度快。

## 使用说明
1. 可使用submodule或subtree下载到模块3rd/ad_third_party_libraries下（建议）。系统版本不会使用模块内的ad_third_party_libraries。
2. **根目录CMakeLists.txt**里，add_subdirectory引用所需库的CMakeLists.txt。
3. 模块里链接所编译的第三方库。

