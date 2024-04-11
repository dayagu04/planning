import sys
import os
from abc import ABC, abstractmethod
import bokeh.plotting as bkp
from bokeh.models import HoverTool, Slider, CustomJS, Div, WheelZoomTool, DataTable, TableColumn, Panel, Tabs
from bokeh.io import output_notebook, push_notebook, output_file, export_png
from bokeh.layouts import layout, column, row
from bokeh.plotting import figure, output_file, show, ColumnDataSource

import numpy as np
from IPython.core.display import display, HTML

import logging
sys.path.append('..')
sys.path.append('../..')
sys.path.append('../../..')
from lib.basic_layers import *
from lib.bag_loader import *
from lib.local_view_lib import *

bag_path = "/share/mnt/0721/real_time_22.00000"
html_file = bag_path +".html"

# bokeh创建的html在jupyter中显示
display(HTML("<style>.container { width:95% !important;  }</style>"))
display(
    HTML('''<style>.widget-label {min-width: 25ex !important; }</style>'''))
output_notebook()

# 判断是否在jupyter中运行
def isINJupyter():
    try:
        __file__
    except NameError:
        return True
    else:
        return False

def plotOnce(bag_path, html_file):
    # 加载bag
    try:
        dataLoader = LoadCyberbag(bag_path)
    except:
        print('load cyber_bag error!')
        return

    if isINJupyter():
        max_time = dataLoader.load_all_data()
        print("is in jupyter now!")
    else:
        max_time = dataLoader.load_all_data(False)
    
    layer_manager = LayerManager()

    fig_local_view = draw_local_view(dataLoader, layer_manager)

    min_t = sys.maxsize
    max_t = 0
    for gdlabel in layer_manager.gds.keys():
        gd = layer_manager.gds[gdlabel]
        min_t = min(min_t, gd.getMinT())
        max_t = max(max_t, gd.getMaxT())

    data_tmp = {'mt': [min_t]}

    for gdlabel in layer_manager.data_key.keys():
        gd = layer_manager.gds[gdlabel]
        data_label = layer_manager.data_key[gdlabel]
        data_tmp[data_label+'s'] = [gd.xys]
        data_tmp[data_label+'ts'] = [gd.ts]
    bag_data = ColumnDataSource(data=data_tmp)

    callback_arg = slider_callback_arg(bag_data)
    for layerlabels in layer_manager.layers.keys():
        callback_arg.AddSource(layerlabels, layer_manager.layers[layerlabels])
    # find the front one of ( the first which is larger than k)
    binary_search = """
            function binarySearch(ts, k){
                if(ts.length == 0){
                    return 0;
                }
                var left = 0;
                var right = ts.length -1;
                while(left<right){
                    var mid = Math.floor((left + right) / 2);
                    if(ts[mid]<=k){ //if the middle value is less than or equal to k
                        left = mid + 1; //set the left value to the middle value + 1
                    }else{
                        right = mid; //set the right value to the middle value
                    }
                }
                if(left == 0){ //if the left value is 0
                    return 0; //return 0
                }
                return left-1; //return the left value - 1
            }
    """

    car_slider = Slider(start=0, end=max_time-0,
                        value=0, step=0.05, title="time")
    code0 = """
    %s
            console.log("cb_objS");
            console.log(cb_obj);
            console.log("cb_obj.value");
            console.log(cb_obj.value);
            console.log("bag_source");
            console.log(bag_source);
            console.log("bag_source.data");
            console.log(Object.keys(bag_source.data));
            const step = cb_obj.value;
            const data = bag_source.data;

    """

    codes = (layer_manager.code) % (code0) % (binary_search)
    callback = CustomJS(args=callback_arg.arg, code=codes)

    car_slider.js_on_change('value', callback)

    for gdlabel in layer_manager.gds.keys():
        gd = layer_manager.gds[gdlabel]
        if gdlabel is 'ep_source' or gdlabel is 'ep_source2' or gdlabel.startswith('global'):
            gd_frame = gd.atT(max_t)

        else:
            gd_frame = gd.atT(min_t)
        if gdlabel is 'online_obj_source' or gdlabel is 'onlinel_obj_source':
            layer_manager.layers[gdlabel].update(
                gd_frame[0], gd_frame[1], gd_frame[2])
            # print(gd_frame)
        elif gdlabel is 'cfb_source':
            pass
        else:
            if layer_manager.plotdim[gdlabel] == 3:
                layer_manager.layers[gdlabel].update(
                    gd_frame[0], gd_frame[1], gd_frame[2])
            else:
                layer_manager.layers[gdlabel].update(gd_frame[0], gd_frame[1])

    output_file(html_file)

    if isINJupyter():
        # display in jupyter notebook
        output_notebook()

    # pan_lt = Panel(child=row(column(fig_local_view, fig_sv), column(fig_tp, fig_tv, fig_ta, fig_tj)), title="Longtime")
    # pan_rt = Panel(child=row(tab_rt, column(fig_rtv)), title="Realtime")
    # pans = Tabs(tabs=[ pan_lt, pan_rt ])
    bkp.show(layout(car_slider, fig_local_view))

def plotMain():
    # print('sys.argv = ', sys.argv)

    if(len(sys.argv) == 2):
        bag_path = str(sys.argv[1])
        html_file = bag_path +".html"

    else:
        bag_path = str(sys.argv[1])
        html_file = str(sys.argv[2])

    bag_path = sys.argv[1]
    html_path = bag_path

    # print("bag_path: {}\nhtml_path: {}".format(bag_path, html_path))

    if os.path.isfile(bag_path) and (not os.path.isdir(html_path)):
        print("process one bag ...")
        html_file = bag_path +".html"
        plotOnce(bag_path, html_file)
        return

    if (not os.path.isdir(bag_path)) or (not os.path.isdir(html_path)):
        print("INVALID ARGV:\n bag_path: {}\nhtml_path: {}".format(
            bag_path, html_path))
        return

    if not os.path.exists(html_path):
        os.makedirs(html_path)

    all_bag_files = os.listdir(bag_path)
    print("find {} files".format(len(all_bag_files)))
    generated_count = 0
    for bag_name in all_bag_files:
        if (".0000" in bag_name) and (bag_name.find(".html") == -1):
            print("process {} ...".format(bag_name))
            bag_file = os.path.join(bag_path, bag_name)
            html_file = bag_file +".html"
            try:
                plotOnce(bag_file, html_file)
                print("html_file = ", html_file)
                generated_count += 1
            except Exception:
                print('failed')

    print("{} html files generated\n".format(generated_count))

if __name__ == '__main__':
    if isINJupyter():
        plotOnce(bag_path, html_file)
    else:
        plotMain()

