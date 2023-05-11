import sys, os
sys.path.append("..")
from lib.load_cyberbag import *

from scipy import interpolate
import numpy as np
# 采用jupyter + bokeh作图
import ipywidgets
from bokeh.io import output_notebook, push_notebook
from bokeh.layouts import layout, column, row
from bokeh.plotting import figure, show, output_file, reset_output, save
from IPython.core.display import display, HTML

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

# +
bag_path = "/home/root/autodrive_pnc_tools/bag/217609/2023-05-08-16-15-28.record"

bag_loder = CyberBagLoader(bag_path)

topic_list = bag_loder.load_topics()

# -
control_data = bag_loder.load_control_data()

