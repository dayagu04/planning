import sys
import os
from abc import ABC, abstractmethod
import bokeh.plotting as bkp
from bokeh.models import HoverTool, Slider, CustomJS, Div, WheelZoomTool, DataTable, TableColumn, Panel, Tabs, Arrow, NormalHead, Label
from bokeh.io import output_notebook, push_notebook, output_file, export_png
from bokeh.layouts import layout, column, row
from bokeh.plotting import figure, output_file, show, ColumnDataSource
from bokeh.events import Tap

import numpy as np
from IPython.core.display import display, HTML

from bokeh.layouts import gridplot
from bokeh.plotting import figure, show
from bokeh.models import Slider

import logging
sys.path.append('..')
sys.path.append('../..')
sys.path.append('../../..')
from lib.basic_layers import *
from lib.bag_loader import *
from lib.local_view_lib import *
from lib.load_local_view_parking import LoadCyberbag
from lib.load_local_view_parking import apa_draw_local_view, apa_draw_local_view_parking_ctrl

bag_path = '/data_cold/abu_zone/autoparse/chery_e0y_18049/trigger/20241121/20241121-16-03-00/park_in_data_collection_CHERY_E0Y_18049_ALL_FILTER_2024-11-21-16-03-01_no_camera.bag'
html_file = bag_path +".apa.html"
plot_ctrl_flag = True
fig1_time_step = 0.1
fig_other_time_step = 0.02
slider_time_step = 0.1

# bokeh创建的html在jupyter中显示
if isINJupyter():
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
        dataLoader = LoadCyberbag(bag_path, True)
    except:
        print('load cyber_bag error!')
        return

    if isINJupyter():
        max_time = dataLoader.load_all_data()
        print("is in jupyter now!")
    else:
        max_time = dataLoader.load_all_data()

    layer_manager = LayerManager()

    # fig1: local view
    # try:
    vehicle_type = None
    if len(sys.argv) > 2:
        if sys.argv[2] == CHERY_T26:
            vehicle_type = CHERY_T26
        elif sys.argv[2] == CHERY_E0X:
            vehicle_type = CHERY_E0X
        else:
            vehicle_type = JAC_S811
    else:
        vehicle_type = JAC_S811
    fig_local_view, tab_debug_layer = apa_draw_local_view(dataLoader, layer_manager, max_time, fig1_time_step, vehicle_type, plot_ctrl_flag = True)
    # except:
    #     print("fig_local_view plot error!")
    #     fig_local_view = bkp.figure(x_axis_label='y', y_axis_label='x', width=600, height=1000, match_aspect = True, aspect_scale=1)
    #     fig_local_view.x_range.flipped = True
    #     fig_local_view.toolbar.active_scroll = fig_local_view.select_one(WheelZoomTool)
    # if plot_ctrl_flag:
    fig2, fig3, fig4, fig5, fig6, fig7 = apa_draw_local_view_parking_ctrl(dataLoader, layer_manager, max_time, time_step=fig_other_time_step)


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

    car_slider = Slider(start=0.0, end=max_time-0,
                        value=-0.1, step=slider_time_step, title="time")
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

    #####
    source = ColumnDataSource(data=dict(x=[], y=[]))
    fig_local_view.circle('x', 'y', size=10, source=source, color='red', legend_label='measure tool')
    line_source = ColumnDataSource(data=dict(x=[], y=[]))
    fig_local_view.line('x', 'y', source=source, line_width=3, line_color = 'pink', line_dash = 'solid', legend_label='measure tool')
    text_source = ColumnDataSource(data=dict(x=[], y=[], text=[]))
    fig_local_view.text('x', 'y', 'text', source=text_source, text_color='red', text_align='center', text_font_size='15pt', legend_label='measure tool')

    # Define the JavaScript callback code
    callback_code = """
        var x = cb_obj.x;
        var y = cb_obj.y;

        source.data['x'].push(x);
        source.data['y'].push(y);

        if (source.data['x'].length > 2) {
            source.data['x'].shift();
            source.data['y'].shift();
            source.data['x'].shift();
            source.data['y'].shift();
        }
        source.change.emit();

        if (source.data['x'].length >= 2) {
            var x1 = source.data['x'][source.data['x'].length - 2];
            var y1 = source.data['y'][source.data['y'].length - 2];
            var x2 = x;
            var y2 = y;
            var x3 = (x1 + x2) / 2;
            var y3 = (y1 + y2) / 2;

            var distance = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));

            console.log("Distance between the last two points: " + distance);

            distance = distance.toFixed(4);
            text_source.data = {'x': [x3], 'y': [y3], 'text': [distance]};
            text_source.change.emit();

            line_source.data = {'x': [x1, x2], 'y': [y1, y2]};
            line_source.change.emit();
        }

        if (source.data['x'].length == 1) {
            text_source.data['x'].shift();
            text_source.data['y'].shift();
            text_source.data['text'].shift();
        }
        text_source.change.emit();
    """

    # Create a CustomJS callback with the defined code
    callback = CustomJS(args=dict(source=source, line_source=line_source, text_source=text_source), code=callback_code)

    # Attach the callback to the Tap event on the plot
    fig_local_view.js_on_event(Tap, callback)

    for gdlabel in layer_manager.gds.keys():
        gd = layer_manager.gds[gdlabel]
        if gdlabel is 'ep_source' or gdlabel is 'ep_source2' or gdlabel.startswith('global'):
            gd_frame = gd.atT(max_t)

        else:
            gd_frame = gd.atT(min_t)
        if gdlabel is 'online_obj_source' or gdlabel is 'onlinel_obj_source':
            layer_manager.layers[gdlabel].update(
                gd_frame[0], gd_frame[1], gd_frame[2])
        elif gdlabel is 'cfb_source':
            pass
        else:
            try:
                if layer_manager.plotdim[gdlabel] == 3:
                    layer_manager.layers[gdlabel].update(
                        gd_frame[0], gd_frame[1], gd_frame[2])
                elif layer_manager.plotdim[gdlabel] == 5:
                    layer_manager.layers[gdlabel].update(
                        gd_frame[0], gd_frame[1], gd_frame[2], gd_frame[3], gd_frame[4])
                else:
                    layer_manager.layers[gdlabel].update(gd_frame[0], gd_frame[1])
            except:
                pass
    output_file(html_file)

    if isINJupyter():
        # display in jupyter notebook
        output_notebook()

    # layout = gridplot([[fig_local_view],
    #                [car_slider]])
    # show(layout)
    bkp.show(layout(car_slider, row(fig_local_view, row(column(fig2.fig, fig3.fig, fig4.fig, fig5.fig), column(fig6.fig, fig7.fig, tab_debug_layer)))))
    # bkp.show(fig_local_view, notebook_handle=True)

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
        html_file = bag_path + ".apa" + ".html"
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
            html_file = bag_file + ".apa" + ".html"
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
