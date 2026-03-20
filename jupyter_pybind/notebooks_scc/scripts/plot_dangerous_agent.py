import os
import sys

sys.path.append("..")
sys.path.append("../lib/")
sys.path.append('../..')
sys.path.append('../../../')

from google.protobuf.descriptor import FieldDescriptor
from bokeh.resources import INLINE
from bokeh.models import TextInput
from bokeh.models import ColumnDataSource, DataTable, DateFormatter, TableColumn
from lib.load_local_view import *
from lib.load_ros_bag import LoadRosbag
import inspect

# bag path and frame dt
bag_path = "/data_cold/abu_zone/autoparse/chery_e0y_20260/trigger/20250613/20250613-16-15-30/data_collection_CHERY_E0Y_20260_EVENT_FILTER_2025-06-13-16-15-30_no_camera.bag.1752065534.open-loop.scc.plan"
# frame dt
frame_dt = 0.1  # sec
display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook(resources=INLINE)

bag_loader = LoadRosbag(bag_path)
max_time = bag_loader.load_all_data()
# JAC_S811 CHERY_T26 CHERY_E0X CHERY_M32T
global_var.set_value('car_type', 'CHERY_E0X')
global_var.set_value('g_is_display_enu', False)
fig1, local_view_data = load_local_view_figure()
fig1.legend.label_text_font_size = "8pt"
fig1.height = 1200
# fig1.width = 700

plan_debug_msg_idx = 0
obj_id = 0
# sliders config


class DangerousAgentSlider:
    def __init__(self,  slider_callback):
        self.time_slider = ipywidgets.FloatSlider(layout=ipywidgets.Layout(
            width='100%'), description="bag_time", min=0.0, max=max_time, value=0.1, step=frame_dt)
        ipywidgets.interact(slider_callback, bag_time=self.time_slider)


potential_dangerous_agent_data = ColumnDataSource({
    'name': [],
    'data': []
})

columns = [
    TableColumn(field="name", title="name",),
    TableColumn(field="data", title="data"),
]
potential_dangerous_agent_table = DataTable(
    source=potential_dangerous_agent_data, columns=columns, width=750, height=500)

# 修改update_potential_dangerous_agent_data函数
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



def slider_callback(bag_time):
    global plan_debug_msg_idx
    local_view_data_ = update_local_view_data(
        fig1, bag_loader, bag_time, local_view_data)

    if bag_loader.plan_debug_msg['enable'] == True:
        potential_dangerous_agent_common = local_view_data['data_msg'][
            'plan_debug_msg'].potential_dangerous_agent_decider_info
        try:
            update_potential_dangerous_agent_data(
                potential_dangerous_agent_common)
        except:
            pass
    push_notebook()


# +
bkp.show(
    row(fig1, potential_dangerous_agent_table),
    notebook_handle=True
)  # table combine
slider_class = DangerousAgentSlider(slider_callback)

# slider_class = ObjText(obj_id_handler)
# -




