import os
import sys
import subprocess
import json
import time
from multiprocessing import Process
from pathlib import Path

def simulation_open(file_path):
    dir_path = os.path.dirname(file_path)

    time_fold = str(int(time.time()))
    out_bag_path = file_path + "." + time_fold + ".default_open_loop.plan"
    command = f"/root/planning/install/bin/pp --play {file_path} --out-bag {out_bag_path} --interface-check --no-version-check"
    try:
        subprocess.run(command, shell=True, text=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Runing PP error: {e}")
        return None
    
    for item in os.listdir(dir_path):
        item_path = os.path.join(dir_path, item)
        if os.path.isfile(item_path):
            if time_fold in item:
                 return item_path
    print(f"out pp path no found")    # return None
    return None
if __name__ == "__main__":
    if len(sys.argv) > 1:
        file_path = sys.argv[1]
    else:
        file_path = "/share/data_cold/abu_zone/autoparse/chery_m32t_72216/trigger/20250905/20250905-15-47-25/data_collection_CHERY_M32T_72216_EVENT_KEY_2025-09-05-15-47-25_no_camera.bag"
    simulation_open(file_path)