import sys
import os
sys.path.append("..")
sys.path.append("../lib/")
# from lib.load_ros_bag import is_match_planning, is_bag_main
sys.path.append('../..')
sys.path.append('../../../build')
sys.path.append('../../../')
from python_proto import potential_dangerous_agent_decider_info_pb2
from jupyter_pybind import potential_dangerous_agent_decider_py
from google.protobuf.descriptor import FieldDescriptor
from bokeh.models import WheelZoomTool, HoverTool
import bokeh.plotting as bkp
from bokeh.models import ColumnDataSource
from functools import partial
from collections import namedtuple
import threading
from IPython.display import clear_output
from ipywidgets import Button, HBox
from IPython.display import display
import ipywidgets as widgets
from bokeh.models import Label, DataTable, TableColumn
from IPython.core.display import display, HTML
from bokeh.layouts import layout, column, row
from bokeh.io import output_notebook, push_notebook
import ipywidgets
import time
import numpy as np
from lib.load_ros_bag import LoadRosbag
from lib.load_json import *
from lib.load_rotate import *
from lib.load_struct import *
from lib.load_local_view import *



# load bag info
car_xb, car_yb = load_car_params_patch()
coord_tf = coord_transformer()
bag_path = "/data_cold/abu_zone/autoparse/bestune_e541_88446/trigger/20251129/20251129-13-39-07/data_collection_BESTUNE_E541_88446_EVENT_KEY_2025-11-29-13-39-07_no_camera.bag"
frame_dt = 0.1

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadRosbag(bag_path)  # 1-2 bag_loader
max_time = bag_loader.load_all_data()
fig1, local_view_data = load_local_view_figure()
fig1.height = 1200
fig1.width = 800
rss = potential_dangerous_agent_decider_py.RssInterface()


potential_dangerous_agent_data = ColumnDataSource({
    'name': [],
    'data': []
})

potential_dangerous_agent_data_tuned = ColumnDataSource({
    'name': [],
    'data': []
})
columns = [
    TableColumn(field="name", title="name",),
    TableColumn(field="data", title="data"),
]
potential_dangerous_agent_table = DataTable(
    source=potential_dangerous_agent_data, columns=columns, width=500, height=1000)
potential_dangerous_agent_table_updated = DataTable(
    source=potential_dangerous_agent_data_tuned, columns=columns, width=500, height=1000)


def update_potential_dangerous_agent_data_tuned(info):
    # 创建表格数据
    agent_data = []

    # 添加决策器信息
    agent_data.append({
        'name': 'risk_free_lateral_distance',
        'value': info.risk_free_lateral_distance
    })
    agent_data.append({
        'name': 'risk_free_longitudinal_distance',
        'value': info.risk_free_longitudinal_distance
    })
    agent_data.append({
        'name': 'ego_lateral_vel',
        'value': info.ego_lateral_vel
    })
    agent_data.append({
        'name': 'ego_longitudinal_vel',
        'value': info.ego_longitudinal_vel
    })

    # 添加每个危险代理的信息
    for idx, agent in enumerate(info.potential_dangerous_agent):
        if agent.risk_level == 0:
            continue
        agent_data.append({
            'name': f'Agent {idx+1} - ID',
            'value': agent.id
        })
        agent_data.append({
            'name': f'Agent {idx+1} - Risk Level',
            'value': agent.risk_level
        })
        agent_data.append({
            'name': f'Agent {idx+1} - Longitudinal Distance',
            'value': agent.longitudinal_distance
        })
        agent_data.append({
            'name': f'Agent {idx+1} - longitudinal_reckless_safe_distance',
            'value': agent.longitudinal_reckless_safe_distance
        })
        agent_data.append({
            'name': f'Agent {idx+1} - longitudinal_moderate_safe_distance',
            'value': agent.longitudinal_moderate_safe_distance
        })

        agent_data.append({
            'name': f'Agent {idx+1} - Lateral Distance',
            'value': agent.lateral_distance
        })
        agent_data.append({
            'name': f'Agent {idx+1} - lateral_moderate_safe_distance',
            'value': agent.lateral_moderate_safe_distance
        })
        agent_data.append({
            'name': f'Agent {idx+1} - lateral_reckless_safe_distance',
            'value': agent.lateral_reckless_safe_distance
        })
        agent_data.append({
            'name': f'Agent {idx+1} - obstacle_rel_pos_type',
            'value': agent.obstacle_rel_pos_type
        })
        agent_data.append({
            'name': f'Agent {idx+1} - recommended_lateral_maneuver',
            'value': agent.recommended_lateral_maneuver
        })
        agent_data.append({
            'name': f'Agent {idx+1} - recommended_longitudinal_maneuver',
            'value': agent.recommended_longitudinal_maneuver
        })

        agent_data.append({
            'name': f'Agent {idx+1} - Lateral Velocity',
            'value': agent.lateral_vel
        })
        agent_data.append({
            'name': f'Agent {idx+1} - Longitudinal Velocity',
            'value': agent.longitudinal_vel
        })

    # 更新数据源
    names = [item['name'] for item in agent_data]
    values = [item['value'] for item in agent_data]

    potential_dangerous_agent_data_tuned.data.update({
        'name': names,
        'data': values
    })
    print('2   Potential dangerous agent data updated')


def update_potential_dangerous_agent_data(info):
    # 创建表格数据
    agent_data = []

    # 添加决策器信息
    agent_data.append({
        'name': 'risk_free_lateral_distance',
        'value': info.risk_free_lateral_distance
    })
    agent_data.append({
        'name': 'risk_free_longitudinal_distance',
        'value': info.risk_free_longitudinal_distance
    })
    agent_data.append({
        'name': 'ego_lateral_vel',
        'value': info.ego_lateral_vel
    })
    agent_data.append({
        'name': 'ego_longitudinal_vel',
        'value': info.ego_longitudinal_vel
    })

    # 添加每个危险代理的信息
    for idx, agent in enumerate(info.potential_dangerous_agent):
        if agent.risk_level == 0:
            continue
        agent_data.append({
            'name': f'Agent {idx+1} - ID',
            'value': agent.id
        })
        agent_data.append({
            'name': f'Agent {idx+1} - Risk Level',
            'value': agent.risk_level
        })
        agent_data.append({
            'name': f'Agent {idx+1} - Longitudinal Distance',
            'value': agent.longitudinal_distance
        })
        agent_data.append({
            'name': f'Agent {idx+1} - longitudinal_reckless_safe_distance',
            'value': agent.longitudinal_reckless_safe_distance
        })
        agent_data.append({
            'name': f'Agent {idx+1} - longitudinal_moderate_safe_distance',
            'value': agent.longitudinal_moderate_safe_distance
        })

        agent_data.append({
            'name': f'Agent {idx+1} - Lateral Distance',
            'value': agent.lateral_distance
        })
        agent_data.append({
            'name': f'Agent {idx+1} - lateral_moderate_safe_distance',
            'value': agent.lateral_moderate_safe_distance
        })
        agent_data.append({
            'name': f'Agent {idx+1} - lateral_reckless_safe_distance',
            'value': agent.lateral_reckless_safe_distance
        })
        agent_data.append({
            'name': f'Agent {idx+1} - obstacle_rel_pos_type',
            'value': agent.obstacle_rel_pos_type
        })
        agent_data.append({
            'name': f'Agent {idx+1} - recommended_lateral_maneuver',
            'value': agent.recommended_lateral_maneuver
        })
        agent_data.append({
            'name': f'Agent {idx+1} - recommended_longitudinal_maneuver',
            'value': agent.recommended_longitudinal_maneuver
        })

        agent_data.append({
            'name': f'Agent {idx+1} - Lateral Velocity',
            'value': agent.lateral_vel
        })
        agent_data.append({
            'name': f'Agent {idx+1} - Longitudinal Velocity',
            'value': agent.longitudinal_vel
        })

    # 更新数据源
    names = [item['name'] for item in agent_data]
    values = [item['value'] for item in agent_data]

    potential_dangerous_agent_data.data.update({
        'name': names,
        'data': values
    })
    print('Potential dangerous agent data updated')


class LocalViewSlider:
    def __init__(self,  slider_callback):
        self.config_type = widgets.IntSlider(layout=ipywidgets.Layout(
            width='80%'), description="config_type", min=1, max=3, value=3, step=1)
        self.response_time = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="rt", min=0.0, max=2.0, value=0.2, step=0.05)


        self.longitudinal_acc_max = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="lon_acc_max", min=0.0, max=5.0, value=2.3, step=0.05)

        self.longitudinal_brake_min = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
             width='80%', display='flex'), description="lon_bk_min", min=0.0, max=6.0, value=1.5, step=0.1)
 
        self.longitudinal_brake_max = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
             width='80%', display='flex'), description="lon_bk_max",  min=0.0, max=6.0, value=5.0, step=0.1)

        self.lateral_acc_max = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
             width='80%', display='flex'), description="lat_acc_max", min=0.0, max=4.0, value=2.0, step=0.1)

        self.lateral_brake_min = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
             width='80%', display='flex'), description="lat_bk_min", min=0.0, max=5.0, value=2.55, step=0.05)


        self.lateral_brake_max = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="lat_bk_max", min=0.0, max=3.0, value=1.1, step=0.05)

        self.lateral_miu = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="lat_miu", min=0.0, max=1.0, value=0.32, step=0.01)

        self.response_time_reckless = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
             width='80%', display='flex'), description="rt_reck", min=0.0, max=2.0, value=0.2, step=0.05)
        self.longitudinal_acc_max_reckless = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
             width='80%', display='flex'), description="lon_acc_max_reck", min=0.0, max=5.0, value=1.7, step=0.05)
        self.longitudinal_brake_min_reckless = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="lon_bk_min_reck", min=0.0, max=6.0, value=3.0, step=0.1)
        self.longitudinal_brake_max_reckless = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
             width='80%', display='flex'), description="lon_bk_max_reck", min=0.0, max=6.0, value=5.0, step=0.1)
        self.lateral_acc_max_reckless = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
             width='80%', display='flex'), description="lat_acc_max_reck", min=0.0, max=4.0, value=1.1, step=0.1)
        self.lateral_brake_min_reckless = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="lat_bk_min_reck", min=0.0, max=5.0, value=2.5, step=0.05)
        self.lateral_brake_max_reckless = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
             width='80%', display='flex'), description="lat_bk_max_reck", min=0.0, max=3.0, value=1.1, step=0.05)

        self.lateral_miu_reckless = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%', display='flex'), description="lat_miu_reck", min=0.0, max=1.0, value=0.32, step=0.01)

        
        self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='80%'), description="bag_time", min=0.1, max=max_time, value=0.1, step=frame_dt)
        ipywidgets.interact(
            slider_callback,
            bag_time=self.time_slider,
            config_type=self.config_type,
            response_time=self.response_time,
            longitudinal_acc_max=self.longitudinal_acc_max,
            longitudinal_brake_min=self.longitudinal_brake_min,
            longitudinal_brake_max=self.longitudinal_brake_max,
            lateral_acc_max=self.lateral_acc_max,
            lateral_brake_min=self.lateral_brake_min,
            lateral_brake_max=self.lateral_brake_max,
            lateral_miu=self.lateral_miu,
            response_time_reckless=self.response_time_reckless,
            longitudinal_acc_max_reckless=self.longitudinal_acc_max_reckless,
            longitudinal_brake_min_reckless=self.longitudinal_brake_min_reckless,
            longitudinal_brake_max_reckless=self.longitudinal_brake_max_reckless,
            lateral_acc_max_reckless=self.lateral_acc_max_reckless,
            lateral_brake_min_reckless=self.lateral_brake_min_reckless,
            lateral_brake_max_reckless=self.lateral_brake_max_reckless,
            lateral_miu_reckless=self.lateral_miu_reckless,

        
        )


def slider_callback(bag_time,
                    config_type,
                    response_time,
                    longitudinal_acc_max,
                    longitudinal_brake_min,
                    longitudinal_brake_max,
                    lateral_acc_max,
                    lateral_brake_min,
                    lateral_brake_max,
                    lateral_miu,
                    response_time_reckless,
                    longitudinal_acc_max_reckless,
                    longitudinal_brake_min_reckless,
                    longitudinal_brake_max_reckless,
                    lateral_acc_max_reckless,
                    lateral_brake_min_reckless,
                    lateral_brake_max_reckless,
                    lateral_miu_reckless
                   ):
    kwargs = locals()
    update_local_view_data(fig1, bag_loader, bag_time, local_view_data)
    plan_debug_msg = local_view_data['data_msg']['plan_debug_msg']
    pda_debug_msg = plan_debug_msg.potential_dangerous_agent_decider_info

    pda_debug_msg_string = pda_debug_msg.SerializeToString()
    success = rss.UpdateModerateParams(
        config_type,
        response_time,
        longitudinal_acc_max,
        longitudinal_brake_min,
        longitudinal_brake_max,
        lateral_acc_max,
        lateral_brake_min,
        lateral_brake_max,
        lateral_miu
    )
    success = rss.UpdateRecklessParams(
        config_type,
        response_time_reckless,
        longitudinal_acc_max_reckless,
        longitudinal_brake_min_reckless,
        longitudinal_brake_max_reckless,
        lateral_acc_max_reckless,
        lateral_brake_min_reckless,
        lateral_brake_max_reckless,
        lateral_miu_reckless
    )
    try:
        update_potential_dangerous_agent_data(pda_debug_msg)
        output = rss.CalculateAgentRSSDistance(pda_debug_msg_string)
        rss_model_output = potential_dangerous_agent_decider_info_pb2.PotentialDangerousAgentDeciderInfo()
        rss_model_output.ParseFromString(output)
        update_potential_dangerous_agent_data_tuned(rss_model_output)
    except:
        pass
    push_notebook()


bkp.show(row(fig1, row(potential_dangerous_agent_table,
         potential_dangerous_agent_table_updated)), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)


