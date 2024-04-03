import sys, os, copy
sys.path.append("..")
# from lib.load_cyberbag import *
from lib.load_local_view_parking import *
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('../../python_proto')

from jupyter_pybind import optimizer_apa_simulation_py
from python_proto import lateral_path_optimizer_pb2
from struct_msgs.msg import PlanningOutput
from lib.load_local_view_parking import *



# bag path and frame dt
bag_path = '/data_cold/abu_zone/autoparse/jac_s811_96tj0/parking/20240320/20240320-15-04-51/park_in_data_collection_JAC_S811_96TJ0_MANUAL_ALL_2024-03-20-15-04-51_no_camera.record'
frame_dt = 0.1  # sec
parking_flag = True

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path, parking_flag)
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure_parking()

# try before sliders
optimizer_apa_simulation_py.Init()

data_planning_tune = ColumnDataSource(data={'plan_path_x': [],
                                            'plan_path_y': [],
                                            'plan_path_heading': [], })

data_sim_pos = ColumnDataSource(data={'x': [], 'y': []})
data_sim_car = ColumnDataSource(data={'car_xn': [], 'car_yn': []})
data_car_box = ColumnDataSource(data={'x_vec': [], 'y_vec': []})
data_tlane = ColumnDataSource(
    data={'x': [-11.3691, -11.56], 'y': [-4.08125, -6.54795]})
# data_tlane = ColumnDataSource(data = {'x':[1.12249], 'y':[0.618948]})


# ilqr optimizer
# data
data_x = ColumnDataSource(data={'x_vec_origin': [],
                                'x_vec_tune': [],
                                's_vec_origin_x': []
                                })

data_y = ColumnDataSource(data={'y_vec_origin': [],
                                'y_vec_tune': [],
                                's_vec_origin_y': []
                                })

data_theta = ColumnDataSource(data={'theta_vec_origin': [],
                                    'theta_vec_tune': [],
                                    's_vec_origin_theta': []})


data_k = ColumnDataSource(data={
    'k_vec_tune': [],
    's_vec_origin_k': [],
    'k_vec_max': [],
    'k_vec_min': []
})

data_u = ColumnDataSource(data={'u_vec_tune': [],
                                'u_vec_max': [],
                                'u_vec_min': [],
                                's_vec_origin_u': []})


# plot figure
fig1.circle('plan_path_y', 'plan_path_x', source=data_planning_tune,  size=4, color='yellow', legend_label='tuned plan')
fig1.line('plan_path_y', 'plan_path_x', source=data_planning_tune, line_width=6,line_color='green', line_dash='solid', line_alpha=0.5, legend_label='tuned plan')
fig1.circle('y', 'x', source=data_sim_pos, size=8, color='red')
fig1.patch('car_yn', 'car_xn', source=data_sim_car, fill_color="red", fill_alpha=0.25,line_color="black", line_width=1, legend_label='sim_car', visible=False)
fig1.patches('y_vec', 'x_vec', source=data_car_box, fill_color="#98FB98", fill_alpha=0.0, line_color="black", line_width=1, legend_label='sampled carbox', visible=False)
fig1.circle('y', 'x', source=data_tlane, size=8, color='green', legend_label='tlane')


fig2 = bkp.figure(x_axis_label='s', y_axis_label='theta', width=600, height=160)
fig3 = bkp.figure(x_axis_label='s', y_axis_label='k',x_range=fig2.x_range, width=600, height=160)
fig4 = bkp.figure(x_axis_label='s', y_axis_label='x',x_range=fig2.x_range, width=600, height=160)
fig5 = bkp.figure(x_axis_label='s', y_axis_label='y', x_range=fig2.x_range, width=600, height=160)
fig6 = bkp.figure(x_axis_label='s', y_axis_label='u',x_range=fig2.x_range, width=600, height=160)

f2 = fig2.line('s_vec_origin_theta', 'theta_vec_origin', source=data_theta, line_width=2, line_color='red', line_dash='solid', legend_label='ref theta')
fig2.line('s_vec_origin_theta', 'theta_vec_tune', source=data_theta, line_width=2,line_color='green', line_dash='solid', legend_label='tune theta')

f3 = fig3.line('s_vec_origin_k', 'k_vec_tune', source=data_k, line_width=2,line_color='green', line_dash='solid', legend_label='tune k')
fig3.line('s_vec_origin_k', 'k_vec_max', source=data_k, line_width=2,line_color='blue', line_dash='dashed', legend_label='max k')
fig3.line('s_vec_origin_k', 'k_vec_min', source=data_k, line_width=2,line_color='blue', line_dash='solid', legend_label='min k')

f4 = fig4.line('s_vec_origin_x', 'x_vec_origin', source=data_x, line_width=2, line_color='red', line_dash='solid', legend_label='ref x')
fig4.line('s_vec_origin_x', 'x_vec_tune', source=data_x, line_width=2, line_color='green', line_dash='solid', legend_label='tune x')

f5 = fig5.line('s_vec_origin_y', 'y_vec_origin', source=data_y, line_width=2, line_color='red', line_dash='solid', legend_label='ref y')
fig5.line('s_vec_origin_y', 'y_vec_tune', source=data_y, line_width=2,line_color='green', line_dash='solid', legend_label='tune y')

f6 = fig6.line('s_vec_origin_u', 'u_vec_tune', source=data_u, line_width=2, line_color='red', line_dash='solid', legend_label='u')
fig6.line('s_vec_origin_u', 'u_vec_max', source=data_u, line_width=2,line_color='blue', line_dash='dashed', legend_label='u_max')
fig6.line('s_vec_origin_u', 'u_vec_min', source=data_u, line_width=2,line_color='blue', line_dash='solid', legend_label='u_min')

hover2 = HoverTool(renderers=[f2], tooltips=[('s', '@s_vec_origin_theta'), ('ref theta', '@theta_vec_origin'), ('tune theta', '@theta_vec_tune')], mode='vline')
hover3 = HoverTool(renderers=[f3], tooltips=[('s', '@s_vec_origin_k'), ('tune k','@k_vec_tune'), ('max k', '@k_vec_max'), ('min k', '@k_vec_min')], mode='vline')

fig2.add_tools(hover2)
fig3.add_tools(hover3)
fig2.toolbar.active_scroll = fig2.select_one(WheelZoomTool)
fig3.toolbar.active_scroll = fig3.select_one(WheelZoomTool)
fig2.legend.click_policy = 'hide'
fig3.legend.click_policy = 'hide'

hover4 = HoverTool(renderers=[f4], tooltips=[('s', '@s_vec_origin_x'), ('ref x', '@x_vec_origin'), ('tune x', '@x_vec_tune')], mode='vline')
hover5 = HoverTool(renderers=[f5], tooltips=[('s', '@s_vec_origin_y'), ('ref y', '@y_vec_origin'), ('tune y', '@y_vec_tune')], mode='vline')
hover6 = HoverTool(renderers=[f6], tooltips=[('s', '@s_vec_origin_u'), ('u','@u_vec_tune'), ('u_max', '@u_vec_max'), ('u_min', '@u_vec_min')], mode='vline')
fig4.add_tools(hover4)
fig5.add_tools(hover5)
fig6.add_tools(hover6)

fig4.toolbar.active_scroll = fig4.select_one(WheelZoomTool)
fig5.toolbar.active_scroll = fig5.select_one(WheelZoomTool)
fig6.toolbar.active_scroll = fig6.select_one(WheelZoomTool)

fig4.legend.click_policy = 'hide'
fig5.legend.click_policy = 'hide'
fig6.legend.click_policy = 'hide'


# sliders config
class LocalViewSlider:
    def __init__(self,  slider_callback):
        self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='75%'), description="bag_time", min=0.0, max=max_time, value=-0.1, step=frame_dt)
        self.select_id_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout(width='18%'), description="select_id", min=0, max=20, value=0, step=1)
        self.force_plan_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout( width='15%'), description="force_plan", min=0, max=1, value=0, step=1)
        self.is_path_optimization_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout( width='15%'), description="path_optimization", min=0, max=1, value=0, step=1)
        self.is_cilqr_optimization_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout( width='15%'), description="cilqr_optimization", min=0, max=1, value=0, step=1)
        self.is_reset_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout( width='15%'), description="is_reset", min=0, max=1, value=0, step=1)
        self.is_complete_path_slider = ipywidgets.IntSlider(layout=ipywidgets.Layout( width='15%'), description="is_complete_path", min=0, max=1, value=0, step=1)
        self.sample_ds_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout( width='25%'), description="sample_ds", min=0.02, max=2.0, value=0.02, step=0.02)

        self.q_ref_xy_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout( width='50%'), description="q_ref_xy", min=0.0, max=20000.0, value=100.0, step=0.1)
        self.q_ref_theta_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout( width='50%'), description="q_ref_theta", min=0.0, max=100000.0, value=100.0, step=0.1)
        self.q_terminal_xy = ipywidgets.FloatSlider(layout=ipywidgets.Layout( width='50%'), description="q_terminal_xy", min=0.0, max=100000.0, value=9000.0, step=0.01)
        self.q_terminal_theta = ipywidgets.FloatSlider(layout=ipywidgets.Layout( width='50%'), description="q_terminal_theta", min=0.0, max=100000.0, value=9000.0, step=0.01)
        self.q_k_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="q_k", min=0.0, max=200.0, value=10.0, step=0.1)
        self.q_u_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout( width='50%'), description="q_u", min=0.0, max=200.0, value=10.0, step=0.1)
        self.q_k_bound = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description="q_k_bound", min=0.0, max=2000.0, value=100.0, step=0.1)
        self.q_u_bound = ipywidgets.FloatSlider(layout=ipywidgets.Layout( width='50%'), description="q_u_bound", min=0.0, max=2000.0, value=50.0, step=0.1)

        self.lon_pos_dif_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout( width='40%'), description="lon_pos_dif", min=-20.0, max=20.0, value=0.0, step=0.01)
        self.lat_pos_dif_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout( width='40%'), description="lat_pos_dif", min=-20.0, max=20.0, value=0.0, step=0.01)
        self.heading_dif_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='40%'), description="heading_dif", min=-45.0, max=45.0, value=0.0, step=0.1)

        ipywidgets.interact(slider_callback,
                            bag_time=self.time_slider,
                            select_id=self.select_id_slider,
                            force_plan=self.force_plan_slider,
                            is_path_optimization=self.is_path_optimization_slider,
                            is_cilqr_optimization = self.is_cilqr_optimization_slider,
                            is_reset=self.is_reset_slider,
                            is_complete_path=self.is_complete_path_slider,
                            sample_ds=self.sample_ds_slider,
                            q_ref_xy=self.q_ref_xy_slider,
                            q_ref_theta=self.q_ref_theta_slider,
                            q_terminal_theta=self.q_terminal_theta,
                            q_terminal_xy=self.q_terminal_xy,
                            q_k=self.q_k_slider,
                            q_u=self.q_u_slider,
                            q_k_bound=self.q_k_bound,
                            q_u_bound=self.q_u_bound,
                            lon_pos_dif=self.lon_pos_dif_slider,
                            lat_pos_dif=self.lat_pos_dif_slider,
                            heading_dif=self.heading_dif_slider)

# sliders callback


def slider_callback(bag_time, select_id, force_plan, is_path_optimization, is_cilqr_optimization, is_reset, is_complete_path, sample_ds,
                    q_ref_xy, q_ref_theta, q_terminal_xy, q_terminal_theta, q_k, q_u, q_k_bound, q_u_bound,
                    lon_pos_dif, lat_pos_dif, heading_dif):
    kwargs = locals()
    update_local_view_data_parking(fig1, bag_loader, bag_time, local_view_data)
    index_map = bag_loader.get_msg_index(bag_time)

    plan_debug_msg = bag_loader.plan_debug_msg['json'][index_map['plan_debug_msg_idx']]
    fus_parking_msg = bag_loader.fus_parking_msg['data'][index_map['fus_parking_msg_idx']]
    wave_msg = bag_loader.wave_msg['data'][index_map['wave_msg_idx']]
    vs_msg = bag_loader.vs_msg['data'][index_map['vs_msg_idx']]
    soc_state_msg = bag_loader.soc_state_msg['data'][index_map['soc_state_msg_idx']]
    loc_msg = copy.deepcopy(
        bag_loader.loc_msg['data'][index_map['loc_msg_idx']])

    if soc_state_msg.current_state == 30:
        tlane_p0_x = plan_debug_msg['tlane_p0_x']
        tlane_p0_y = plan_debug_msg['tlane_p0_y']
        tlane_p1_x = plan_debug_msg['tlane_p1_x']
        tlane_p1_y = plan_debug_msg['tlane_p1_y']
        obstacle_x = plan_debug_msg['obstaclesX']
        obstacle_x.append(tlane_p0_x)
        obstacle_x.append(tlane_p1_x)
        obstacle_y = plan_debug_msg['obstaclesY']
        obstacle_y.append(tlane_p0_y)
        obstacle_y.append(tlane_p1_y)
        data_tlane.data.update({
            'x': obstacle_x,
            'y': obstacle_y,
        })
    else:
        data_tlane.data.update({
            'y': [],
            'x': [],
        })

    current_ego_x = loc_msg.pose.local_position.x
    current_ego_y = loc_msg.pose.local_position.y
    sim_ego_heading = loc_msg.pose.euler_angles.yaw + heading_dif / 57.2958

    sim_ego_x = current_ego_x + lon_pos_dif * \
        math.cos(sim_ego_heading) - lat_pos_dif * math.sin(sim_ego_heading)
    sim_ego_y = current_ego_y + lon_pos_dif * \
        math.sin(sim_ego_heading) + lat_pos_dif * math.cos(sim_ego_heading)

    loc_msg.pose.local_position.x = sim_ego_x
    loc_msg.pose.local_position.y = sim_ego_y
    loc_msg.pose.euler_angles.yaw = sim_ego_heading
    loc_msg.pose.heading = sim_ego_heading

    data_sim_pos.data.update({
        'x': [sim_ego_x],
        'y': [sim_ego_y],
    })

    car_xn = []
    car_yn = []
    for i in range(len(car_xb)):
        tmp_x, tmp_y = local2global(
            car_xb[i], car_yb[i], sim_ego_x, sim_ego_y, sim_ego_heading)
        car_xn.append(tmp_x)
        car_yn.append(tmp_y)

    data_sim_car.data.update({
        'car_xn': car_xn,
        'car_yn': car_yn,
    })

    res = optimizer_apa_simulation_py.InterfaceUpdateParam(soc_state_msg.SerializeToString(),
                                                           fus_parking_msg.SerializeToString(),
                                                           loc_msg.SerializeToString(),
                                                           vs_msg.SerializeToString(),
                                                           wave_msg.SerializeToString(),
                                                           select_id, force_plan, is_path_optimization, is_cilqr_optimization, is_reset, is_complete_path, sample_ds,
                                                           q_ref_xy, q_ref_theta, q_terminal_xy, q_terminal_theta, q_k, q_u, q_k_bound, q_u_bound)

    data_planning_tune.data = {'plan_path_x': [],
                               'plan_path_y': [],
                               'plan_path_heading': []}

    if res == True:
        # fig 1 path
        tuned_planning_output = PlanningOutput()
        tuned_planning_output.deserialize(
            optimizer_apa_simulation_py.GetPlanningOutput())

        plan_path_x = []
        plan_path_y = []
        plan_path_heading = []
        for i in range(len(tuned_planning_output.trajectory.trajectory_points)):
            plan_path_x.append(
                tuned_planning_output.trajectory.trajectory_points[i].x)
            plan_path_y.append(
                tuned_planning_output.trajectory.trajectory_points[i].y)
            plan_path_heading.append(
                tuned_planning_output.trajectory.trajectory_points[i].heading_yaw)

        data_planning_tune.data.update({
            'plan_path_x': plan_path_x,
            'plan_path_y': plan_path_y,
            'plan_path_heading': plan_path_heading,
        })

        # path ego car
        car_box_x_vec = []
        car_box_y_vec = []
        for k in range(len(tuned_planning_output.trajectory.trajectory_points)):
            car_xn = []
            car_yn = []
            for i in range(len(car_xb)):
                tmp_x, tmp_y = local2global(
                    car_xb[i], car_yb[i], plan_path_x[k], plan_path_y[k], plan_path_heading[k])
                car_xn.append(tmp_x)
                car_yn.append(tmp_y)
            car_box_x_vec.append(car_xn)
            car_box_y_vec.append(car_yn)

        data_car_box.data.update({
            'x_vec': car_box_x_vec,
            'y_vec': car_box_y_vec,
        })
        print("tuned_gear_command = ", tuned_planning_output.gear_command)

        # ilqr path
        output_planning_debug = lateral_path_optimizer_pb2.LateralPathOptimizerOutput()
        output_planning_debug_tmp = optimizer_apa_simulation_py.GetPlanningOutputDebugInfo()
        output_planning_debug.ParseFromString(output_planning_debug_tmp)

        input_planning_debug = lateral_path_optimizer_pb2.LateralPathOptimizerInput()
        input_planning_debug_tmp = optimizer_apa_simulation_py.GetPlanningInputDebugInfo()
        input_planning_debug.ParseFromString(input_planning_debug_tmp)

        # print(output_planning_debug)
        # print(input_planning_debug)
        # print("terminal pos error = ", planning_debug.terminal_pos_error)
        # print("terminal heading error = ", planning_debug.terminal_heading_error)
        try:
            x_vec_origin = []
            y_vec_origin = []
            theta_vec_origin = []
            x_vec_tune = []
            y_vec_tune = []
            theta_vec_tune = []
            k_vec_tune = []
            u_vec_tune = []
            s_vec_tune = []
            k_max_origin = []
            k_min_origin = []
            u_max_origin = []
            u_min_origin = []

            for i in range(len(output_planning_debug.s_vec)):
                x_vec_tune.append(
                    output_planning_debug.x_vec[i])
                y_vec_tune.append(
                    output_planning_debug.y_vec[i])
                theta_vec_tune.append(
                    output_planning_debug.theta_vec[i] * 57.3)
                k_vec_tune.append(
                    output_planning_debug.k_vec[i])
                u_vec_tune.append(
                    output_planning_debug.u_vec[i])
                s_vec_tune.append(
                    output_planning_debug.s_vec[i])

                k_max_origin.append(
                    input_planning_debug.k_max_vec[i])
                k_min_origin.append(
                    input_planning_debug.k_min_vec[i])
                u_max_origin.append(
                    input_planning_debug.u_max_vec[i])
                u_min_origin.append(
                    input_planning_debug.u_min_vec[i])

                x_vec_origin.append(
                    input_planning_debug.ref_x_vec[i])
                y_vec_origin.append(
                    input_planning_debug.ref_y_vec[i])
                theta_vec_origin.append(
                    input_planning_debug.ref_theta_vec[i] * 57.3)

            data_x.data.update({
                'x_vec_origin': x_vec_origin,
                'x_vec_tune': x_vec_tune,
                's_vec_origin_x': s_vec_tune
            })

            data_y.data.update({
                'y_vec_origin': y_vec_origin,
                'y_vec_tune': y_vec_tune,
                's_vec_origin_y': s_vec_tune
            })

            data_theta.data.update({
                'theta_vec_origin': theta_vec_origin,
                'theta_vec_tune': theta_vec_tune,
                's_vec_origin_theta': s_vec_tune
            })

            data_k.data.update({
                'k_vec_tune': k_vec_tune,
                's_vec_origin_k': s_vec_tune,
                'k_vec_max': k_max_origin,
                'k_vec_min': k_min_origin
            })

            data_u.data.update({
                'u_vec_tune': u_vec_tune,
                'u_vec_max': u_max_origin,
                'u_vec_min': u_min_origin,
                's_vec_origin_u': s_vec_tune
            })
        except:
            pass

    push_notebook()


bkp.show(row(fig1, column(fig4, fig5, fig2, fig3, fig6)), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
