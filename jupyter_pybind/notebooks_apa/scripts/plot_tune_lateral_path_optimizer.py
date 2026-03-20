import sys
import os
sys.path.append("..")
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')

sys.path.append('../../python_proto')

from jupyter_pybind import lateral_path_optimizer_py
from python_proto import lateral_path_optimizer_pb2
from lib.load_local_view import *

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()


# init pybind
lateral_path_optimizer_py.Init()

# data
data_path = ColumnDataSource(data={'x_vec_origin': [],
                                   'y_vec_origin': [],
                                   'x_vec_tune': [],
                                   'y_vec_tune': []
                                   })

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

# plot fig
fig1 = bkp.figure(x_axis_label='y', y_axis_label='x', width=960,
                  height=800, match_aspect=True, aspect_scale=1)
fig1.x_range.flipped = True
fig1.line('y_vec_origin', 'x_vec_origin', source=data_path, line_width=2,
          line_color='red', line_dash='solid', legend_label='data_path_ref')
fig1.line('y_vec_tune', 'x_vec_tune', source=data_path, line_width=2,
          line_color='green', line_dash='solid', legend_label='data_path_tune')
fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
fig1.legend.click_policy = 'hide'


fig2 = bkp.figure(x_axis_label='s', y_axis_label='theta',
                  x_range=[-0.1, 5.2], width=600, height=160)
fig3 = bkp.figure(x_axis_label='s', y_axis_label='k',
                  x_range=fig2.x_range, width=600, height=160)
fig4 = bkp.figure(x_axis_label='s', y_axis_label='x',
                  x_range=fig2.x_range, width=600, height=160)
fig5 = bkp.figure(x_axis_label='s', y_axis_label='y',
                  x_range=fig2.x_range, width=600, height=160)
fig6 = bkp.figure(x_axis_label='s', y_axis_label='u',
                  x_range=fig2.x_range, width=600, height=160)

f2 = fig2.line('s_vec_origin_theta', 'theta_vec_origin', source=data_theta,
               line_width=2, line_color='red', line_dash='solid', legend_label='ref theta')
fig2.line('s_vec_origin_theta', 'theta_vec_tune', source=data_theta, line_width=2,
          line_color='green', line_dash='solid', legend_label='tune theta')

f3 = fig3.line('s_vec_origin_k', 'k_vec_tune', source=data_k, line_width=2,
               line_color='green', line_dash='solid', legend_label='tune k')
fig3.line('s_vec_origin_k', 'k_vec_max', source=data_k, line_width=2,
          line_color='blue', line_dash='dashed', legend_label='max k')
fig3.line('s_vec_origin_k', 'k_vec_min', source=data_k, line_width=2,
          line_color='blue', line_dash='solid', legend_label='min k')

f4 = fig4.line('s_vec_origin_x', 'x_vec_origin', source=data_x,
               line_width=2, line_color='red', line_dash='solid', legend_label='ref x')
fig4.line('s_vec_origin_x', 'x_vec_tune', source=data_x, line_width=2,
          line_color='green', line_dash='solid', legend_label='tune x')

f5 = fig5.line('s_vec_origin_y', 'y_vec_origin', source=data_y,
               line_width=2, line_color='red', line_dash='solid', legend_label='ref y')
fig5.line('s_vec_origin_y', 'y_vec_tune', source=data_y, line_width=2,
          line_color='green', line_dash='solid', legend_label='tune y')

f6 = fig6.line('s_vec_origin_u', 'u_vec_tune', source=data_u,
               line_width=2, line_color='red', line_dash='solid', legend_label='u')
fig6.line('s_vec_origin_u', 'u_vec_max', source=data_u, line_width=2,
          line_color='blue', line_dash='dashed', legend_label='u_max')
fig6.line('s_vec_origin_u', 'u_vec_min', source=data_u, line_width=2,
          line_color='blue', line_dash='solid', legend_label='u_min')

hover2 = HoverTool(renderers=[f2], tooltips=[('s', '@s_vec_origin_theta'),
                   ('ref theta', '@theta_vec_origin'), ('tune theta', '@theta_vec_tune')], mode='vline')
hover3 = HoverTool(renderers=[f3], tooltips=[('s', '@s_vec_origin_k'), ('tune k',
                   '@k_vec_tune'), ('max k', '@k_vec_max'), ('min k', '@k_vec_min')], mode='vline')

fig2.add_tools(hover2)
fig3.add_tools(hover3)
fig2.toolbar.active_scroll = fig2.select_one(WheelZoomTool)
fig3.toolbar.active_scroll = fig3.select_one(WheelZoomTool)
fig2.legend.click_policy = 'hide'
fig3.legend.click_policy = 'hide'

hover4 = HoverTool(renderers=[f4], tooltips=[('s', '@s_vec_origin_x'),
                   ('ref x', '@x_vec_origin'), ('tune x', '@x_vec_tune')], mode='vline')
hover5 = HoverTool(renderers=[f5], tooltips=[('s', '@s_vec_origin_y'),
                   ('ref y', '@y_vec_origin'), ('tune y', '@y_vec_tune')], mode='vline')
hover6 = HoverTool(renderers=[f6], tooltips=[('s', '@s_vec_origin_u'), ('u',
                   '@s_vec_origin_u'), ('u_max', '@u_vec_max'), ('u_min', '@u_vec_min')], mode='vline')
fig4.add_tools(hover4)
fig5.add_tools(hover5)
fig6.add_tools(hover6)

fig4.toolbar.active_scroll = fig4.select_one(WheelZoomTool)
fig5.toolbar.active_scroll = fig5.select_one(WheelZoomTool)
fig6.toolbar.active_scroll = fig6.select_one(WheelZoomTool)

fig4.legend.click_policy = 'hide'
fig5.legend.click_policy = 'hide'
fig6.legend.click_policy = 'hide'


class LocalViewSlider:
    def __init__(self, slider_callback):
        self.q_ref_xy_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='50%'), description="q_ref_xy", min=0.0, max=20000.0, value=800.0, step=0.1)
        self.q_ref_theta_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='50%'), description="q_ref_theta", min=0.0, max=100000.0, value=400.0, step=0.1)
        self.q_terminal_theta = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='50%'), description="q_terminal_theta", min=0.0, max=100000.0, value=40000.0, step=0.01)
        self.q_terminal_xy = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='50%'), description="q_terminal_xy", min=0.0, max=100000.0, value=1000.0, step=0.01)
        self.q_k_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='50%'), description="q_k", min=0.0, max=200.0, value=200.0, step=0.1)
        self.q_u_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='50%'), description="q_u", min=0.0, max=200.0, value=50.0, step=0.1)

        self.q_k_bound = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='50%'), description="q_k_bound", min=0.0, max=2000.0, value=2000.0, step=0.1)
        self.q_u_bound = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='50%'), description="q_u_bound", min=0.0, max=2000.0, value=50.0, step=0.1)

        ipywidgets.interact(slider_callback,
                            q_ref_xy=self.q_ref_xy_slider,
                            q_ref_theta=self.q_ref_theta_slider,
                            q_terminal_theta=self.q_terminal_theta,
                            q_terminal_xy=self.q_terminal_xy,
                            q_k=self.q_k_slider,
                            q_u=self.q_u_slider,
                            q_k_bound=self.q_k_bound,
                            q_u_bound=self.q_u_bound
                            )


def slider_callback(q_ref_xy, q_ref_theta, q_terminal_theta, q_terminal_xy, q_k, q_u, q_k_bound, q_u_bound):
    kwargs = locals()

    # init path
    ds = 0.18
    nums = lateral_path_optimizer_py.GenPathPoints(ds)
    x_vec = lateral_path_optimizer_py.GetXVec()
    y_vec = lateral_path_optimizer_py.GetYVec()
    heading_vec = lateral_path_optimizer_py.GetHeadingVec()
    curvature_vec = lateral_path_optimizer_py.GetCurvatureVec()
    length = lateral_path_optimizer_py.GetLength()
    print("nums =", nums)
    print("length =", length)

    # init params
    lat_motion_plan_input0 = lateral_path_optimizer_pb2.LateralPathOptimizerInput()
    lat_motion_plan_input0.r_min = 5.4
    lat_motion_plan_input0.curv_factor = 0.3
    lat_motion_plan_input0.delta_max = 400 / 57.3 / 15
    lat_motion_plan_input0.ref_vel = 1.688

    lat_motion_plan_input0.init_state.x = x_vec[0]
    lat_motion_plan_input0.init_state.y = y_vec[0]
    lat_motion_plan_input0.init_state.theta = heading_vec[0]
    lat_motion_plan_input0.init_state.k = -curvature_vec[0]

    lat_motion_plan_input0.last_theta = heading_vec[-1]
    lat_motion_plan_input0.last_y = y_vec[-1]
    lat_motion_plan_input0.last_x = x_vec[-1]

    k_max = 1.0 / lat_motion_plan_input0.r_min
    k_min = - k_max
    u_max = lat_motion_plan_input0.curv_factor * \
        lat_motion_plan_input0.delta_max / lat_motion_plan_input0.ref_vel
    u_min = - u_max

    x_vec_origin = []
    y_vec_origin = []
    theta_vec_origin = []
    k_max_origin = []
    k_min_origin = []
    u_max_origin = []
    u_min_origin = []
    for i in range(nums):
        lat_motion_plan_input0.ref_x_vec.append(x_vec[i])
        lat_motion_plan_input0.ref_y_vec.append(y_vec[i])
        lat_motion_plan_input0.ref_theta_vec.append(heading_vec[i])
        lat_motion_plan_input0.control_vec.append(0.0)
        lat_motion_plan_input0.k_max_vec.append(k_max)
        lat_motion_plan_input0.u_max_vec.append(u_max)
        x_vec_origin.append(x_vec[i])
        y_vec_origin.append(y_vec[i])
        theta_vec_origin.append(heading_vec[i] * 57.3)
        k_max_origin.append(k_max)
        k_min_origin.append(k_min)
        u_max_origin.append(u_max)
        u_min_origin.append(u_min)

    input_string = lat_motion_plan_input0.SerializeToString()
    lateral_path_optimizer_py.UpdateByParams(
        input_string, q_ref_xy, q_ref_xy, q_ref_theta, q_terminal_theta, q_terminal_xy, q_k, q_u, q_k_bound, q_u_bound)
    planning_output = lateral_path_optimizer_pb2.LateralPathOptimizerOutput()
    planning_output_tmp = lateral_path_optimizer_py.GetOutputBytes()
    planning_output.ParseFromString(planning_output_tmp)

    x_vec_tune = []
    y_vec_tune = []
    theta_vec_tune = []
    k_vec_tune = []
    u_vec_tune = []
    s_vec_tune = []
    for i in range(len(planning_output.s_vec)):
        x_vec_tune.append(planning_output.x_vec[i])
        y_vec_tune.append(planning_output.y_vec[i])
        theta_vec_tune.append(planning_output.theta_vec[i] * 57.3)
        k_vec_tune.append(planning_output.k_vec[i])
        u_vec_tune.append(planning_output.u_vec[i])
        s_vec_tune.append(planning_output.s_vec[i])

    data_path.data.update({
        'x_vec_origin': x_vec_origin,
        'y_vec_origin': y_vec_origin,
        'x_vec_tune': x_vec_tune,
        'y_vec_tune': y_vec_tune
    })

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

    push_notebook()


bkp.show(row(fig1, column(fig4, fig5, fig2, fig3, fig6)), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
