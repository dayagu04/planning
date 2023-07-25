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
1.代码格式化（.ci/clang-format/clang-format-5.0）：
```
make clang-format
```
2.VSCode插件:
- clangd 用于代码跳转和自动补全(需要先`cd build && cmake ..`一下)

3.Planning Player 本地播包
- 编译
```
make pp_build
```
- 跑planning，新bag生成在/mnt/xxxxx/xxxx.0000.xxxx.plan, log重定向到指定文件
```
build/tools/planning_player/pp --play /mnt/xxxxx/xxxx.0000 --close-loop > planning.log
```
- 闭环仿真（车辆完美跟随，仅在长时有效），不加此参数默认开环
```
--close-loop
```
- 新bag生成在xxx.bag
```
--out-bag=xxx.bag[注意要有等号]
```
- 修改进自动帧数，默认15（1.5s）
```
--auto-frame=15
```
支持指定场景，行车/泊车，默认行车
```
--scene-type=acc 或 apa
```
- 修改`.vscode/launch.json`中的bag路径，在VSCode调试界面选择planning player，可以断点调试代码
- localview画图中会打印当前帧号frame_num，据此可以在planning log中找到对应帧日志

4.启动jupyter服务器：

```
启动容器：
.ci/dev.py --platform=X86

vscode连接到容器后，在终端中运行：
sudo su
make jupyter_start

浏览器打开屏幕上链接，打开tools/common/jupyter/scripts/下文件，修改bag路径后运行所有即可可视化
图标bag解析和图标显示逻辑在tools/common/jupyter/lib下
```
5.html生成方法：

```
进入文件夹
jupyter_pybind/notebooks/scripts

运行 html_generator.py 脚本
```
针对单个bag文件生成html
```
python html_generator.py plot_ctrl_html /home/xlwang71/Downloads/0713/long_time_1.00000
```
其中plot_ctrl_html代表脚本类型，目前支持plot_ctrl_html, plot_lat_plan_html, plot_lon_plan_html

若想针对某一路径下的所有bag均生成脚本,则可采用如下方式
```
python html_generator.py plot_ctrl_html /home/xlwang71/Downloads/0713/
```
