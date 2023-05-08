# planning

算法-规划-代码仓库

# Usage
1.拉取所依赖的子模块，目前仅拉取interface的源码
```
git submodule update --init --recursive
```
2.编译：采用gcc编译器，生成产物在build文件夹内
```
mkdir build & cd build
cmake ..
make -j 32
```
3.部署：将需要的产物打包，用于部署实车,打包后产物路径：build/planning，将其部署在FDC域控路径: /asw 下即可
```
make install
```
4.启动：
```
mainboard -d /asw/planning/planning.dag
```