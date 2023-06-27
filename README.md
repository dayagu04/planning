# planning

算法-规划-代码仓库

# Usage
1.拉取所依赖的子模块，目前仅拉取interface的源码
```
make submodules
```
2.本地调试，编译、部署：采用gcc编译器，生成产物在build文件夹内
```
make build
```
3.实车产物编译，生成产物在build文件夹内
```
make build BUILD_TYPE=Release PLATFORM=BZT
```
4.启动：
```
mainboard -d /asw/planning/planning.dag
```

# 版本集成
1.系统调用各个模块`target/build.sh`进行编译打包，产物为Release版本
2.产物生成在`install/planning/`下

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
- 跑planning，新bag生成在/mnt/xxxxx/xxxx.0000.xxxx.plan
```
build/tools/planning_player/pp --play /mnt/xxxxx/xxxx.0000
```
- 跑planning，新bag生成在xxx.bag
```
build/tools/planning_player/pp --play /mnt/xxxxx/xxxx.0000 --out-bag=xxx.bag[注意要有等号]
```
- 修改`.vscode/launch.json`中的bag路径，在VSCode调试界面选择planning player，可以断点调试代码

4.通过jupyter看bag
``
make jupyter_start
``

# Test
1.单元测试:
```
make unittest_build
make unittest_run
```