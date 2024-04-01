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
from lib.load_bag import *
from lib.local_view_lib import *

bag_path = "/share//data_cold/abu_zone/autoparse/jac_s811_72kx6/trigger/20240306/20240306-15-40-10/data_collection_JAC_S811_72KX6_EVENT_MANUAL_2024-03-06-15-40-10.record"
html_file = bag_path +".ctrl.html"

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
        dataLoader = LoadCyberbag(bag_path)
    except:
        print('load cyber_bag error!')
        return

    if isINJupyter():
        max_time = dataLoader.load_all_data()
        print("is in jupyter now!")
    else:
        max_time = dataLoader.load_all_data(False)

    json_value_list = ["controller_status", "lat_mpc_status", "lat_enable", "lon_enable", "planning_type",
                       "steer_angle_cmd", "steer_angle", "driver_hand_torque", "steer_bias_deg",
                       "yaw_conti", "euler_angle_yaw", "euler_angle_pitch",
                       "vel_raw_cmd", "vel_cmd", "vel_ego",
                       "vel_out", "vel_raw_out", "vel_ffwd_out", "vel_KP_term", "vel_KI_term",
                       "acc_vel", "acc_ego", "slope_acc", "throttle_brake",
                       "lat_err", "phi_err", "vel_error"]

    json_value_xys_dict = GenerateJsonValueData(dataLoader.ctrl_debug_msg['json'], dataLoader.ctrl_debug_msg['t'], json_value_list)


    json_vector_list = ["dx_ref_mpc_vec", "dy_ref_mpc_vec", "dphi_ref_mpc_vec", "dx_mpc_vec", "dy_mpc_vec", "delta_mpc_vec", "dphi_mpc_vec"]
    json_vector_xys_dict = GenerateJsonVectorData(dataLoader.ctrl_debug_msg['json'], dataLoader.ctrl_debug_msg['t'], json_vector_list)
    topic_vector_list = ["dy_ref_mpc_vec", "dphi_ref_mpc_vec", "dy_mpc_vec", "dphi_mpc_vec", "delta_mpc_vec"]
    topic_vector_dict = GenerateTopicVectorData(dataLoader.ctrl_debug_msg['data'], dataLoader.ctrl_debug_msg['t'], topic_vector_list)
    layer_manager = LayerManager()

    # fig1: local view
    try:
        fig_local_view, _ = draw_local_view(dataLoader, layer_manager)
    except:
        print("fig_local_view plot error!")
        fig_local_view = bkp.figure(x_axis_label='y', y_axis_label='x', width=600, height=1000, match_aspect = True, aspect_scale=1)
        fig_local_view.x_range.flipped = True
        fig_local_view.toolbar.active_scroll = fig_local_view.select_one(WheelZoomTool)
    # fig_local_view = draw_local_view(dataLoader, layer_manager)

    # fig2: control status
    fig2 = FigureLayer(bkp.figure(x_axis_label='time',
                                  y_axis_label='status',
                                  x_range = [-0.1, max_time],
                                  width=800,
                                  height=200,
                                  match_aspect = True,
                                  aspect_scale=1))
    fig2.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "controller_status"), "controller_status")
    fig2.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "lat_mpc_status"), "lat_mpc_status")
    fig2.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "lat_enable"), "lat_enable")
    fig2.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "lon_enable"), "lon_enable")
    fig2.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "planning_type"), "planning_type")

    # fig3: steering states
    fig3 = FigureLayer(bkp.figure(x_axis_label='time',
                                y_axis_label='steering angle',
                                x_range = fig2.fig.x_range,
                                width=800,
                                height=200,
                                match_aspect = True,
                                aspect_scale=1))
    fig3.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "steer_angle_cmd", 57.3), "steer_angle_cmd_deg")
    fig3.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "steer_angle", 57.3), "steer_angle_deg")
    fig3.AddCurv(layer_manager, ScalarGeneratorFromJson(json_value_xys_dict, "steer_bias_deg", -1.0),
                 "steer_bias_deg")
    fig3.AddCurv(layer_manager, ScalarGeneratorFromJson(json_value_xys_dict, "driver_hand_torque", 1.0),
                 "driver_hand_torque")

    # fig4: attitude
    fig4 = FigureLayer(bkp.figure(x_axis_label='time',
                                  y_axis_label='attitude',
                                  x_range = fig2.fig.x_range,
                                  width=800,
                                  height=200,
                                  match_aspect = True,
                                  aspect_scale=1))
    fig4.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "yaw_conti", 57.3), "yaw_conti")
    fig4.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "euler_angle_yaw", 57.3), "euler_angle_yaw")
    fig4.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "euler_angle_pitch", 57.3), "euler_angle_pitch")

    # fig5: vel cmd
    fig5 = FigureLayer(bkp.figure(x_axis_label='time',
                                  y_axis_label='vel cmd',
                                  x_range = fig2.fig.x_range,
                                  width=800,
                                  height=200,
                                  match_aspect = True,
                                  aspect_scale=1))
    fig5.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "vel_raw_cmd", 1.0), "vel_raw_cmd")
    fig5.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "vel_cmd", 1.0), "vel_cmd")
    fig5.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "vel_ego", 1.0), "vel_ego")

    # fig6: vel ctr
    fig6 = FigureLayer(bkp.figure(x_axis_label='time',
                                  y_axis_label='vel ctrl',
                                  x_range = fig2.fig.x_range,
                                  width=800,
                                  height=200,
                                  match_aspect = True,
                                  aspect_scale=1))
    fig6.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "vel_out", 1.0), "vel_out")
    fig6.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "vel_raw_out", 1.0), "vel_raw_out")
    fig6.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "vel_ffwd_out", 1.0), "vel_ffwd_out")
    fig6.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "vel_KP_term", 1.0), "vel_KP_term")
    fig6.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "vel_KI_term", 1.0), "vel_KI_term")

    # fig7: mpc dy
    fig7 = DynamicFigureLayer(bkp.figure(x_axis_label='time',
                                         y_axis_label='dy',
                                         width=600,
                                         height=200))
    # fig7.AddCurv(layer_manager,
    #              VectorGeneratorFromJson(topic_vector_dict, "dy_ref_mpc_vec", 1.0), "dy_ref_mpc_vec")
    fig7.AddCurv(layer_manager,
                 VectorGeneratorFromJson(topic_vector_dict, "dy_mpc_vec", 1.0), "dy_mpc_vec")

    # fig8: mpc dphi
    fig8 = DynamicFigureLayer(bkp.figure(x_axis_label='time',
                                         y_axis_label='dphi',
                                         width=600,
                                         x_range = fig7.fig.x_range,
                                         height=200))
    fig8.AddCurv(layer_manager,
                 VectorGeneratorFromJson(topic_vector_dict, "dphi_ref_mpc_vec", 57.3), "dphi_ref_mpc_vec")
    fig8.AddCurv(layer_manager,
                 VectorGeneratorFromJson(topic_vector_dict, "dphi_mpc_vec", 57.3), "dphi_mpc_vec")

    # fig9: mpc delta (steer angle)
    fig9 = DynamicFigureLayer(bkp.figure(x_axis_label='time',
                                         y_axis_label='steer angle deg',
                                         width=600,
                                         x_range = fig7.fig.x_range,
                                         height=200))
    fig9.AddCurv(layer_manager,
                 VectorGeneratorFromJson(topic_vector_dict, "delta_mpc_vec", 57.3 * 15.7), "delta_mpc_vec")


    # fig10: acc state
    fig10 = FigureLayer(bkp.figure(x_axis_label='time',
                                  y_axis_label='acc state',
                                  x_range = fig2.fig.x_range,
                                  width=800,
                                  height=200,
                                  match_aspect = True,
                                  aspect_scale=1))
    fig10.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "vel_out", 1.0), "acc_cmd")
    fig10.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "acc_vel", 1.0), "acc_vel")
    fig10.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "acc_ego", 1.0), "acc_ego")
    fig10.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "slope_acc", 1.0), "slope_acc")
    fig10.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "throttle_brake", 1.0), "throttle_brake")

    # fig11: ctrl err
    fig11 = FigureLayer(bkp.figure(x_axis_label='time',
                                  y_axis_label='ctrl err',
                                  x_range = fig2.fig.x_range,
                                  width=800,
                                  height=200,
                                  match_aspect = True,
                                  aspect_scale=1))
    fig11.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "lat_err", 1.0), "lat_err")
    fig11.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "phi_err", 57.3), "phi_err")
    fig11.AddCurv(layer_manager,
                 ScalarGeneratorFromJson(json_value_xys_dict, "vel_error", 1.0), "vel_err")

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
                        value=0, step=0.02, title="time")
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

    bkp.show(layout(row(fig_local_view, row(column(fig2.fig, fig3.fig, fig4.fig, fig5.fig, fig6.fig), column(fig7.fig, fig8.fig, fig9.fig, fig10.fig, fig11.fig))), car_slider))


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
        html_file = bag_path + ".ctrl" + ".html"
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
            html_file = bag_file + ".ctrl" + ".html"
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
