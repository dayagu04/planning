import sys, os
sys.path.append("..")
from lib.load_local_view import *
sys.path.append('../..')
sys.path.append('../../../')

# bag path and frame dt
bag_path = "/opt/planning/bags/long_time_hualong_1.00000"
frame_dt = 0.02 # sec

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

bag_loader = LoadCyberbag(bag_path)
max_time = bag_loader.load_all_data()
fig1 = bag_loader.msg_timeline_figure()

bkp.show(row(fig1), notebook_handle=True)
