# planning

算法-规划-代码仓库

# Usage
1.拉取所依赖的子模块，目前仅拉取interface的源码
```
make submodules
```
2.本地调试，编译、部署：采用gcc编译器，生成产物在build文件夹内
```
.ci/dev.py --platform=X86
cd planning
sudo make clean 可以删除原有build内容
make build
```
3.实车产物编译，生成产物在build文件夹内，打包至target/planning.tar
```
.ci/dev.py --platform=BZT --root
cd /home/用户名/planning
make clean
make build BUILD_TYPE=Release
make package
```
4.启动：
```
mainboard -d /asw/planning/planning.dag
```

# 版本集成
1.系统调用各个模块`target/build.sh`进行编译打包，产物为Release版本
2.产物生成在`install/planning/`下

查看产物版本号：
```
strings install/xxx/Lib/libxxx_component.so | grep auto_version
```

# 实车调试
1.部署产物包至/asw，原有/asw/planning会被备份为/asw/planning.时间戳（使用target/下的脚本）
```
deploy.sh
```
2.录包，没有时长限制，自动递增序号，需要传入两个参数：路径和bag名前缀
```
record.sh /user_data/ long_time
```

# Tools
##### 1.代码格式化（.ci/clang-format/clang-format-5.0）：
```
make clang-format
```
##### 2.VSCode插件:
- clangd 用于代码跳转和自动补全(需要先`cd build && cmake ..`一下)

##### 3.Planning Player 本地播包
- 编译
```
make pp_build
```
如果想编译X86的Release版本，执行以下命令
```
make pp_build BUILD_TYPE=Release
```
- 跑planning，新bag生成在/mnt/xxxxx/xxxx.0000.xxxx.plan, log重定向到指定文件
```
build/tools/planning_player/pp --play /mnt/xxxxx/xxxx.0000 --close-loop > planning.log
```
- 闭环仿真（车辆完美跟随，仅在长时有效），不加此参数默认开环
```
--close-loop
```
- 新bag生成在xxx.bag，不指定的话默认与输入bag同路径
```
--out-bag=xxx.bag[注意要有等号]
```
- 修改进自动帧数，默认15（1.5s）
```
--auto-time=15
```
- 支持指定场景，行车/泊车，默认scc
```
--scene-type=acc 或 apa
```
- no-debug模式，不依赖planning/debug_info，适用于没起planning模块或planning模块崩溃的情况，默认关闭
```
--no-debug
```
- 修改`.vscode/launch.json`中的bag路径，在VSCode调试界面选择 planning player，可以断点调试代码
- 修改`.vscode/launch.json`中的bag路径，在VSCode调试界面选择 pp no debug，可以用no-debug模式断点调试代码
- localview画图中会打印当前帧号frame_num，据此可以在planning log中找到对应帧日志
- 如果想断点调试特定时刻，在可视化中确定该时刻对应的frame_num，通常会在可视化的底部通过类似`planning debug info: 171`打印出来，然后在`planning_player.cc`对应位置添加条件断点，类似 frame_num_ == 171

##### 4.Planning Player 本地批量生成包
```
--dir            必填参数; 存放bag的路径, 可包含子文件夹
--out_suffix     必填参数; PP生成的新包的后缀
--close_loop     可选参数; 是否为闭环, 不填表示开环
--input_suffix   可选参数; 输入包的后缀, 填入只运行包含特定后缀的文件。不填则会运行所有以'.00000'和'.record'结尾的文件
--ignore_suffix  可选参数; 运行除了以'.json'结尾之外的所有文件, 默认关闭
--out_dir        可选参数; 生成的新包的存放路径, 不填则表示与--dir的路径相同

示例（注意不要漏掉双引号）：
python tools/planning_player/pp.py --dir "/xxxxx/" --out_suffix ".PP"
python tools/planning_player/pp.py --dir "/xxxxx/" --out_suffix ".PP" --out_dir "/xxxxx/" --close_loop 1 --ignore_suffix 1
```
##### 5.启动jupyter服务器：

启动容器：
```
.ci/dev.py --platform=X86
```
vscode连接到容器后，在终端中运行：
```
sudo su
make jupyter_start
```
浏览器打开屏幕上链接，打开tools/common/jupyter/scripts/下文件，修改bag路径后运行所有即可可视化
图标bag解析和图标显示逻辑在tools/common/jupyter/lib下

##### 6.html生成方法：

进入文件夹
jupyter_pybind/notebooks/scripts
运行 html_generator.py 脚本

针对单个bag文件生成html
```
python html_generator.py plot_ctrl_html /home/xlwang71/Downloads/0713/long_time_1.00000
```
其中plot_ctrl_html代表脚本类型，目前支持plot_local_view_html, plot_ctrl_html, plot_lat_plan_html, plot_lon_plan_html, plot_vo_lat_behavior_html

若想针对某一路径下的所有bag均生成脚本,则可采用如下方式
```
python html_generator.py plot_ctrl_html /home/xlwang71/Downloads/0713/
```

##### 7.pybind调试编译方法：

```
make pybind_build
```
