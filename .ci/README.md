# ci

#### 介绍

ZNQC_AutonomousDriving CI support
提供代码格式化、CI支持、容器管理等通用功能，供算法模块集成

#### 使用说明

集成到算法仓库：

```
git submodule add ../../nywang/ci.git .ci
cd .ci
git checkout develop
```

代码格式化：
```
make clang-format
```

拉取依赖模块：
```
make submodules
```

编译清理：
```
make clean
```

编译：

```
黑芝麻：
.ci/dev.py --platform=BZT --root
cd 模块目录
make build BUILD_TYPE=Release

本地调试：
.ci/dev.py --platform=X86
cd 模块目录
make build
```

查看产物版本号：
```
strings install/xxx/Lib/libxxx_component.so | grep auto_version
```

打包tar压缩包至target：
```
make package
```

测试用例编译运行：
```
make unittest_build
make unittest_run 或 make unittest_gdb
```

启动jupyter服务器：
```
启动容器：
.ci/dev.py --platform=X86

vscode连接到容器后，在终端中运行：
sudo su
make jupyter_start

浏览器打开屏幕上链接，打开tools/common/jupyter/scripts/下文件，修改bag路径后运行所有即可可视化
图标bag解析和图标显示逻辑在tools/common/jupyter/lib下
```

启动/进入容器：

容器默认挂载用户目录至/home/用户名，建议将代码下在用户目录；
用户git配置和ssh公钥和服务器一致；

进入X86容器本地开发，进入容器后sudo可以提升为root权限
```
.ci/dev.py --platform=X86
```

使用root身份进入黑芝麻容器
```
.ci/dev.py --platform=BZT --root
```

容器启动后以用户名为后缀命名，保持后台运行，随时通过相同命令进入，ctrl-d断开
且可以通过stop参数停止并删除容器
```
--stop
```