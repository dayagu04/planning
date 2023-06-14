# planning

算法-规划-代码仓库

# Usage
1.拉取所依赖的子模块，目前仅拉取interface的源码
```
git submodule update --init --recursive
```
2.编译：采用gcc编译器，生成产物在build文件夹内
```
mkdir build && cd build
cmake ..
make -j
```
3.部署：将需要的产物打包，用于部署实车,打包后产物路径：build/planning，将其部署在FDC域控路径: /asw 下即可
```
make install
```
4.启动：
```
mainboard -d /asw/planning/planning.dag
```

# Tools
1.代码格式化（.ci/clang-format/clang-format-5.0）：
```
.ci/format.py
```
2.VSCode插件:
- clangd 用于代码跳转和自动补全(需要先`cd build && cmake ..`一下)

3.Planning Player 本地播包
- 编译
```
cd build && cmake ..
make pp -j
make install
```
- 跑planning，新bag生成在/mnt/xxxxx/xxxx.0000.xxxx.plan
```
tools/planning_player/pp --play /mnt/xxxxx/xxxx.0000
```
- 跑planning，新bag生成在xxx.bag
```
tools/planning_player/pp --play /mnt/xxxxx/xxxx.0000 --out-bag xxx.bag
```
- 修改`.vscode/launch.json`中的bag路径，在VSCode调试界面选择planning player，可以断点调试代码

# Test
1.单元测试:
```
mkdir build && cd build
cmake ..
make unit_test -j
make install
./test/unit_test
```