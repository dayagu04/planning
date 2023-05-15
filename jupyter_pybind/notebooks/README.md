# Jupyter-pybin11

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