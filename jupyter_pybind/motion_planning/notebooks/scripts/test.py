# +
import ipywidgets as widgets
from IPython.display import display
import time

slider = widgets.FloatSlider(value=0, min=-10, max=10, step=0.1, description='Slider:')
display(slider)

button = widgets.Button(description="Click me!")
display(button)

def on_button_clicked(b):
    start_value = 0
    end_value = 10
    step_size = 0.1
    value = start_value
    while value <= end_value:
        slider.value = value
        value += step_size
        time.sleep(0.1)

button.on_click(on_button_clicked)
