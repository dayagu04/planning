# interface

公共接口模块-代码仓库

## 目录结明
```
├── CHANGELOG.MD  
├── CMakeLists.txt  
├── doc `文档`  
├── Makefile  
├── mdc `mdc拓扑`  
├── README.md  
├── rosmsg `ros文件`   
│   ├── IflyAutoMDCDCComm `工具链用基础ros文件`   
│   ├── proto_msgs `工具链用proto对应的ros文件`  
│   ├── sensor_interface `工具链用传感器的ros文件`  
│   ├── struct_msgs `c结构体生成的ros文件`  
│   ├── struct_msgs_legacy `历史c结构体生成的ros文件`  
│   └── struct_msgs_v2_10 `上版本c结构体生成的ros文件`  
├── script  
├── src `存放结构体文件`  
│   ├── c `存放接口c结构体定义`  
│   ├── interface_description.md `结构体定义说明`  
│   ├── interface_version  
│   │   └── interface_version.h `版本号定义`  
│   ├── legacy `历史结构体定义文件(请勿添加删除)`  
│   ├── manifest.yaml `模板引擎定义文件`  
│   ├── mcu_base_interface `mcu公用文件`  
│   │   └── define_common.h `基础宏定义文件`  
│   ├── project `项目专用的接口结构体定义文件`  
│   └── proto `接口proto定义`  
├── target  
│   └── build.sh `后处理 生成/检查 脚本`  
├── test  
│   ├── CMakeLists.txt  
│   ├── convert  
│   ├── ifly_unit.h  
│   └── interface `接口定义自测文件`  
└── type_convert `后处理工具文件`  
    ├── base_convert.h `基础转换函数头文件`  
    ├── cstruct_to_rosmsg.py `c结构体转ros脚本`  
    ├── cstruct_gen_convert_function_cpp.py `ros转c结构体脚本`  
    ├── struct_convert `ros转c结构体脚本生成产物`  
    ├── struct_convert_legacy `历史ros转c结构体脚本生成产物(请勿添加删除)`  
    ├── private_proto `工具链存放功能未知`  
    ├── proto_convert `工具链存放功能未知` 
    ├── README.md
    ├── rosbag_converter `rosbag升级工具代码`  
    ├── ros_convert_generator.py `ros_convert转包规则代码生成脚本`  
    ├── topic_description_parser.py `解析interface_description的脚本`  
    ├── ros_convert `rosbag升级规则代码(自动生成)`  
    ├── single_topic_convert.md `rosbag转包配置文件 由topic_description_parser.py生成`  
    └── direct_topic_convert.md `rosbag转包配置文件 直接拷贝的topic`  
```

## MDC平台接口修改方式
1. 通信拓扑绘图
   1. 切换到**你的接口分支**。
   2. 使用华为MMC工具打开mdc下arxml目录，该目录为MMC工程。
   3. 根据需求修改拓扑，修改后保存并提交。
2. 通信拓扑生成
   1. 到**系统集成仓库根目录**
   2. `source env.sh`，选`CHERY_E0X_MDC510`对应的环境
   3. 下载接口仓库`./dev.py fetch --range interface`
   4. 进入`CHERY_E0X_MDC510`的docker`./dev.py container`
   5. 切换到**你的接口分支**
   6. 执行mdc/generate_config.sh脚本，自动下载MDS工具，并根据arxml重新生成generate/ outputcfg/目录。
   7. 检查是否正确生成，并提交改动


## 接口变更步骤
1. 准备联调分支
   1. 从系统集成仓库拉出**你的系统联调分支**
   2. 从接口预发版分支(feature/interface_vX_X_pre)切出**你的接口联调分支**
   3. 下载**你的系统联调分支**
   4. 切换到**你的接口联调分支**`git checkout XXX`
2. 变更接口内容，包括：
   1. 修改src目录下相关代码
   2. 修改src/interface_description.md文档
   3. 在test/interface_test.cpp文件中添加/删除对应头文件
3. 生成type_convert
   1. 回到**系统集成仓库根目录**
   2. `source env.sh`，选`X86`对应的环境
   3. 下载接口仓库`./dev.py fetch --range interface`
   4. 进入`X86`的docker`./dev.py container`
   5. 进入接口仓库target目录`cd components/interface/target`
   6. 执行命令`./build.sh generate` ，观察执行结果
      1. 执行成功打印
         ```
         "[WARNING] please clean <// warning :> in type_convert/ros_convert !!!"
         ```
      2. 若执行失败，请根据失败原因修复问题
   7. 搜索转包工具目录type_convert/ros_convert目录下所有文件中的`// warning :`
   8. 根据转包需求，修改`// warning :`对应的内容
4. 提交接口仓库内的代码改动
5.  弄完以上步骤，可以自测一下
   1. 需要先进到接口仓库target目录
   2. 执行检查命令`./build.sh check`
   3. 测试完请勿再次提交（测试会用到`./build.sh generate`，造成的改动请丢弃）
6. 接口仓库ci流程
   1. 修改**你的系统联调分支**的`configs/common.json`文件，将interface模块的`branch`和`commitid`设置成**你的接口联调分支**的`branch`和`commitid`
   2. 提交改动
   3. 浏览器进入接口仓库ci工程`http://console.devops.iflytek.com/ibuild/serviceDetail?projectId=c2c82102c1c93bf71ecf2de4fd57a40c&appId=277010cc47e049264b98fc03990dd430&appType=SERVER&scmType=GIT`
   4. 点击右上角`执行`按钮
   5. 勾选`构建自测`，`git分支`处填写**你的系统联调分支**
   6. 点击右下角`执行`按钮
   7. 执行成功后，将结果附到接口PR上
   8. 如有问题请及时反馈系统集成组

## 接口预发版分支生成步骤
1. 环境准备
   1. 进入系统集成仓库（2.7.x以上版本）
   2. 从接口develop分支切除预发版分支，命名规则`feature/interface_vX_X_pre`（例如：`feature/interface_v2_7_pre`）
   3. `source env.sh`，选`X86`对应的环境
   4. 下载接口仓库`./dev.py fetch --range interface`
   5. 进入`X86`的docker`./dev.py container`
2. 生成
   1. 进入接口仓库target目录`cd components/interface/target`
   2. 执行命令`./build.sh init 新版本的版本号` ，新版本的版本号请按示例：`./build.sh init 2.7`
   3. 提交代码
3. 测试验证
   1. 执行检查命令`./build.sh check`，会跑到check_type_convert_regenerated报错`"[ERROR] ros_convert regeneration check no warning, check failed"`

## 接口变更工具的常见错误解答
1. "[ERROR] make interface_test_build failed"
   1. 结构体编不过，结构体语法错误
2. "[ERROR] ros_convert regeneration check no warning, check failed"
   1. 转包工具ros_convert没有生成
3. "[ERROR] make type_convert failed"
   1. 转包工具生成失败，可能是ros_convert中某个warning改漏了


## 接口适配ROS类型转换方法说明（执行以下步骤实现当前接口的c结构体转ros类型）
   备注：接口变更如果不执行以下步骤，工控机回流的ros包中可能不会包含接口变更的内容。
1. 生成对应的ROS消息类型
   1. cd type_convert
   2. python3 cstruct_to_rosmsg.py  ../src/c  ../rosmsg/struct_msgs
2. 生成对应的转换函数
   1. cd type_convert
   2. python3 cstruct_gen_convert_function_cpp.py ../src/c  ./struct_convert