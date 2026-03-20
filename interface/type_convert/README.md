# c结构体到rosmsg的转换

## 转换工具cstruct_to_rosmsg.py的使用方法
基于正则表达式的结构体生成rosmsg，**可能有少量错误需要手动修正**
```
python cstruct_to_rosmsg.py [input_path] [output_path]
```
## 目录说明
1. 当前[input_path]为`../src/c`
2. 当前[output_path]为`struct_msgs`


# rosmsg到c结构体的转换

## 转换工具cstruct_gen_convert_function_cpp.py的使用方法
基于正则表达式的结构体生成ros和cstruct互转代码
```
python cstruct_gen_convert_function_cpp.py [input_path] [output_path]
```
## 目录说明
1. 当前[input_path]为`../src/c`
2. 当前[output_path]为`struct_convert`


# proto到rosmsg的转换（暂不维护）
proto定义涉及mutable、add、set等操作，没有自动生成，需要手动写msg和转换


# 转包工具（rosbag_converter）

## 工具功能说明
支持将上一个接口版本的数据包转换到当前接口版本，不支持跨版本转换。

## 目录说明
1. 工具代码目录`rosbag_converter`
2. 转包头文件目录`ros_convert`，通过工具生成

## 构建流程
1. cd path_to_system_integration    进入系统集成仓库
2. source env.sh                    选择X86
3. ./dev.py container               进入X86 docker
4. cd path_to_interface             进入接口仓库
5. make                             编译,产物为rosbag_converter
6. 工具生成在`install/bin`目录下

## 自动生成代码检查
在`构建流程`基础上执行以下步骤：
1. make checkall                    检查所有convert文件
2. export CHECK_MODULE=control_command_c.h
   make check                       检查单个convert文件

## 使用说明
./rosbag_converter --input=input.bag [--output=output.bag]
output.bag可以不指定，默认输出bag名为：inputvx.x.bag


# rosbag converter代码自动生成工具

## 使用说明
1. cd path_to_interface/type_convert		进入type_convert目录
2. python3 topic_description_parser.py		根据interface_description.md解析出需要转包的topic列表,存放在single_topic_convert.md
3. 确认singlel_topic_convert.md 和direct_topic_convert.md 都存在
3. python3 ros_convert_generator.py		生成convert转包代码和转包调用逻辑

## 文件说明
1. single_topic_convert.md 由interface_description.md解析而来,里面包含全量topic的命名和数据结构信息,用来生成topic转换注册入口
2. direct_topic_convert.md 由用户自己编写,里面只填写topic的命名信息,该topic不会用来转换,直接透传到新包里
3. topic_description_parser.py 解析interface_description的脚本
4. ros_convert_generator.py convert转包代码生成脚本
