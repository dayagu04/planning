import sys, os
sys.path.append("..")
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')
from lib.load_local_view import *
from jupyter_pybind import jerk_limited_trajectory_py
import numpy as np

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

def load_figure():
  data_jlt = ColumnDataSource(data = {'t_vec':[], 's_vec':[], 'v_vec':[], 'a_vec':[], 'j_vec':[]})
  # fig1 s-t
  fig1 = bkp.figure(x_axis_label='time', y_axis_label='pos', width=600, height=200)
  # fig2 v-t
  fig2 = bkp.figure(x_axis_label='time', y_axis_label='vel',x_range = fig1.x_range, width=600, height=200)
  # fig3 a-t
  fig3 = bkp.figure(x_axis_label='time', y_axis_label='acc',x_range = fig2.x_range, width=600, height=200)
  # fig4 j-t
  fig4 = bkp.figure(x_axis_label='time', y_axis_label='jerk',x_range = fig3.x_range, width=600, height=200)

  fig1.line('t_vec', 's_vec', source = data_jlt, line_width=1, legend_label='s_trajectory', color="blue")
  fig2.line('t_vec', 'v_vec', source = data_jlt, line_width=1, legend_label='v_trajectory', color="blue")
  fig3.line('t_vec', 'a_vec', source = data_jlt, line_width=1, legend_label='a_trajectory', color="blue")
  fig4.line('t_vec', 'j_vec', source = data_jlt, line_width=1, legend_label='j_trajectory', color="blue")

  return data_jlt, fig1, fig2, fig3, fig4

def updatedata(data_jlt, t_vec, s_output, v_output, a_output, j_output):
    data_jlt.data.update({
    't_vec': t_vec,
    's_vec': s_output,
    'v_vec': v_output,
    'a_vec': a_output,
    'j_vec': j_output
  })

data_jlt, fig1, fig2, fig3, fig4 = load_figure()

# init pybind
jerk_limited_trajectory_py.Init()

class LocalViewSlider:
  def __init__(self,  slider_callback):
    self.v_max_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "v_max", min=0.1, max=30, value=10, step=0.1)
    self.v_min_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "v_min", min=-20, max=-0.1, value=-1, step=0.1)
    self.a_max_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "a_max", min=0.1, max=5, value=2, step=0.1)
    self.a_min_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "a_min", min=-7, max=-0.1, value=-5, step=0.1)
    self.j_max_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "j_max", min=0.1, max=7, value=5, step=0.1)
    self.j_min_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "j_min", min=-7, max=-0.1, value=-5, step=0.1)
    self.s_0_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "s_0", min=-50, max=150, value=0, step=0.1)
    self.v_0_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "v_0", min=0, max=20, value=0, step=0.1)
    self.a_0_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "a_0", min=-7, max=7, value=0, step=0.1)
    self.s_des_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "s_des", min=-50, max=150, value=10, step=0.1)
    self.v_des_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(width='50%'), description= "v_des", min=0, max=20, value=5, step=0.1)

    ipywidgets.interact(slider_callback,v_max = self.v_max_slider,
                                        v_min = self.v_min_slider,
                                        a_max = self.a_max_slider,
                                        a_min = self.a_min_slider,
                                        j_max = self.j_max_slider,
                                        j_min = self.j_min_slider,
                                        s_0 = self.s_0_slider,
                                        v_0 = self.v_0_slider,
                                        a_0 = self.a_0_slider,
                                        s_des = self.s_des_slider,
                                        v_des = self.v_des_slider
                                        )

### sliders callback
def slider_callback(v_max, v_min, a_max, a_min, j_max, j_min, s_0, v_0, a_0, s_des, v_des):
  jerk_limited_trajectory_py.UpdateByParams(s_0, v_0, a_0, s_des, v_des, v_min, v_max, a_min, a_max, j_min, j_max)

  t_output = jerk_limited_trajectory_py.GetTotalTime()
  t_vec = np.arange(0, t_output, 0.1)
  s_output = jerk_limited_trajectory_py.GetSOutput()
  v_output = jerk_limited_trajectory_py.GetVelOutput()
  a_output = jerk_limited_trajectory_py.GetAccOutput()
  j_output = jerk_limited_trajectory_py.GetJerkOutput()

  updatedata(data_jlt, t_vec, s_output, v_output, a_output, j_output)

  push_notebook()

bkp.show(column(fig1, fig2, fig3, fig4), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)
