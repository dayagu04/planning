import sys, os
sys.path.append("..")
from lib.load_local_view import *
sys.path.append('../..')
sys.path.append('../../../')

# bag path and frame dt
bag_path = "/PanShi_data/autoparse/jac_s811_72kx6/trigger/20231120/20231120-20-26-45/data_collection_JAC_S811_72KX6_EVENT_MANUAL_2023-11-20-20-26-45.record"
frame_dt = 0.02 # sec

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path)
max_time = bag_loader.load_all_data()
fig1 = bag_loader.msg_timeline_figure()

bkp.show(row(fig1), notebook_handle=True)
