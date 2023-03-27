# planning

算法-规划-代码仓库

# Usage
1.编译：采用gcc编译器，生成产物在build文件夹内
```
make clean & make build
```
2.部署：将需要的产物打包，用于部署实车,打包后产物路径：build/Planning，将其部署在FDC域控路径: /asw 下即可
```
make deploy
```
3.启动：
```
mainboard -d /asw/Planning/planning.dag
```