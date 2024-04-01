

import sys
import os
from abc import ABC, abstractmethod
import bokeh.plotting as bkp
from bokeh.models import HoverTool, Slider, CustomJS, Div, WheelZoomTool, NumericInput, DataTable, TableColumn, Panel, Tabs
from bokeh.io import output_notebook, push_notebook, output_file, export_png
from bokeh.layouts import layout, column, row
from bokeh.plotting import figure, output_file, show, ColumnDataSource

import numpy as np
from IPython.core.display import display, HTML
from plot_local_view_html import *
import logging
sys.path.append('..')
sys.path.append('../..')
sys.path.append('../../..')
from lib.basic_layers import *
from lib.load_bag import *
from lib.local_view_lib import *

# 先手动写死bag
bag_path = "/share/mnt/s811_1_0824_1/realtime_cutin_9.00000"
# bag_path = '/share/data_cold/abu_zone/hpp/1219bag/memory1219_12.00000'
bag_path = '/share/data_cold/abu_zone/hpp/hpp1225/hpp_first_loop_0.00000'
bag_path = "/share/data_cold/abu_zone/autoparse/jac_s811_35kw2/trigger/20240223/20240223-14-08-20/data_collection_JAC_S811_35KW2_EVENT_MANUAL_2024-02-23-14-08-20_no_camera.record.1709812513.plan"


html_file = bag_path +".local_view.html" 
# -
def isINJupyter():
    try:
        __file__
    except NameError:
        return True
    else:
        return False
# bokeh创建的html在jupyter中显示
if isINJupyter():
    display(HTML("<style>.container { width:95% !important;  }</style>"))
    display(
        HTML('''<style>.widget-label {min-width: 25ex !important; }</style>'''))
    output_notebook()

table_params={
    'width': 300,
    'height':800,
}


def plotOnce(bag_path, html_file):
    # 加载bag
    try:
        dataLoader = LoadCyberbag(bag_path)
    except:
        print('load cyber_bag error!')
        return
    max_time = dataLoader.load_all_data(True)
    layer_manager = LayerManager()

    fig_local_view, plan_debug_table_view = draw_local_view(dataLoader, layer_manager)
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
                    } else{
                        right = mid; //set the right value to the middle value
                    }
                }
                if(left == 0){ //if the left value is 0
                    return 0; //return 0
                }
                return left-1; //return the left value - 1
            }
    """

    car_slider = Slider(start=0, end=max(max_time-0,1),
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
            var obstacle_selector_value=0
            console.log("obstacle_selector", obstacle_selector.value);
            obstacle_selector_value = obstacle_selector.value

    """

    codes = (layer_manager.code) % (code0) % (binary_search)
    codes +="""
    let obstacle_generate_table_index = "obstacle_generate_table_"+obstacle_selector_value+"s"
    if (obstacle_generate_table_index in data){
      lat_rt_obstacle_table_source.data['pts_xs'] = data[obstacle_generate_table_index][0][lat_rt_obstacle_table_index][0];
      lat_rt_obstacle_table_source.data['pts_ys'] = data[obstacle_generate_table_index][0][lat_rt_obstacle_table_index][1];
      lat_rt_obstacle_table_source.change.emit();
    }else{
    lat_rt_obstacle_table_source.data["id"]="not exist"
    }
  """
    callback = CustomJS(args=callback_arg.arg, code=codes)
    selector_callback=CustomJS(args=dict(
       car_slider=car_slider
    ),code="""
    console.log("obstacle_selector")
    console.log(cb_obj)
    console.log("car_slider",car_slider.value);
    let val = car_slider.value;
    car_slider.value=val-1.0;
    car_slider.value=val;
    """)
    car_slider.js_on_change('value', callback)
    obstacle_selector.js_on_change('value',selector_callback)
    
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
    bkp.show(layout(car_slider,fig_local_view))


def printHelp():
    print('''\n
USAGE:
    1. <jupyter mode>      change “bag_path” and "html_file" and run
    2. <single file mode>  python3 plot_bag.py bag_file html_file
    3. <folder batch mode> python3 plot_bag.py bag_folder html_folder
\n''')


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
        html_file = bag_path + ".local_view" + ".html"
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
            html_file = bag_file + ".lat_plan" + ".html"
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
