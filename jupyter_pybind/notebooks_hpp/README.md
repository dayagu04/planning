# 启动jupyter服务器：

```
启动容器：
.ci/dev.py --platform=X86

vscode连接到容器后，在终端中运行：
sudo su
make jupyter_start

浏览器打开屏幕上链接，打开tools/common/jupyter/scripts/下文件，修改bag路径后运行所有即可可视化
图标bag解析和图标显示逻辑在tools/common/jupyter/lib下
```

# Jupyter-pybin11(原有README)

This project is used to support development with jupyter notebook and python bindings.

Jupyter notebook: https://jupyter.org/

pybind11: https://pybind11.readthedocs.io/en/stable/

bokeh plot: https://docs.bokeh.org/en/latest/

jupytext: https://jupytext.readthedocs.io/en/latest/install.html#jupytext-s-contents-manager

yep: https://www.paddlepaddle.org.cn/documentation/docs/zh/advanced_guide/performance_improving/analysis_tools/cpu_profiling_cn.html

# Install dependencies (TODO: integrate into docker images, use google-pprof):
```
sudo apt-get update
sudo apt-get install -y libgoogle-perftools-dev
pip3 install jupyter jupytext pybind11 bokeh yep ipywidgets scipy
```

# Usage
Need to enable jupytext (one time) for notebooks to get easy version control of jupyter notebook in .py format
```
jupyter nbextension enable --py widgetsnbextension
jupyter serverextension enable jupytext
```

Add port 8080 to ros-dev/script/config.json.example-msquare

Build python binding for cpp code. Need to update pybind11 location in CMakeLists.txt if build fails.
```
pwd: /home/ros/conan/maf_controller
cd bag_recurrent
mkdir build && cd build
cmake .. && make
```

Run jupyter notebook:
```
pwd: /home/ros/conan/maf_controller/bag_recurrent/notebooks
jupyter notebook --ip 0.0.0.0 --port 8080
```
Now you can open a jupyter notebook link in a browser outside the container.
<img title="Slider example" alt="Alt text" src="/img/slider_pybind.gif">
#

html生成方法：

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