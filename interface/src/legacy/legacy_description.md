# 历史遗留topic使用规范
## 用途
用于读取历史数据包，实现仿真功能。

## 废弃规范
1. 原废弃接口文件移到此处，将依赖的结构体移到本废弃结构体文件中。
2. 废弃接口的最后一个版本号作为废弃接口的命名空间。
3. 历史topic的名称作为唯一识别标识。
4. 为历史topic提供ros转c接口。

# interface2.4.5
+ **接口：** mcu发往soc的输入信号
  + **topic：** /iflytek/hmi/mcu_inner
  + **头文件：** interface2.4.5/hmi_mcu_inner_c.h
  + **结构体：** iflyauto::interface_2_4_5::HmiMcuInner
  + **频率：** 20Hz
+ **接口：** soc收到系统外部输入
  + **topic：** /iflytek/hmi/soc_inner
  + **头文件：** interface2.4.5/hmi_soc_inner_c.h
  + **结构体：** iflyauto::interface_2_4_5::HmiSocInner
  + **频率：** 20Hz
+ **接口：** soc发往mcu的输出信号
  + **topic：** /iflytek/hmi/soc_outer
  + **头文件：** interface2.4.5/hmi_soc_outer_c.h
  + **结构体：** iflyauto::interface_2_4_5::HmiSocOuter
  + **频率：** 50Hz
+ **接口：** soc收到hpp外部输入
  + **topic：** /iflytek/hmi/hpp_inner
  + **头文件：** interface2.4.5/hmi_hpp_inner_c.h
  + **结构体：** iflyauto::interface_2_4_5::HmiHppInput
  + **频率：** 20Hz
+ **接口：** soc发出的hpp输出
  + **topic：** /iflytek/hmi/hpp_outer
  + **头文件：** interface2.4.5/hmi_hpp_outer_c.h
  + **结构体：** iflyauto::interface_2_4_5::HmiHppOutput
  + **频率：** 50Hz
+ **接口：** 功能状态机输出
  + **topic：** /iflytek/system_state/soc_state
  + **头文件：** interface2.4.5/func_state_machine_c.h
  + **结构体：** iflyauto::interface_2_4_5::FuncStateMachine
  + **频率：** 50Hz

# interface2.4.6
+ **接口：** 定位信息
  + **topic：** /iflytek/localization/ego_pose
  + **头文件：** interface2.4.6/localization_c.h
  + **结构体：** iflyauto::interface_2_4_6::LocalizationEstimate
  + **频率：** 100Hz