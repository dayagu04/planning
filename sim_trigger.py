# -*- coding: utf-8 -*-

#运行前置
#pip3 install requests polling

#使用步骤
#1. 在ibuild流水线填入pr目标分支的分支号  （TARGET_BRANCH） 例：[TARGET_BRANCH] | [main] 
#2. 在ibuild流水线启动前填入pr 目标分支的commitid  （TARGET_COMMIT） 例：[TARGET_COMMIT] | [12a8152] 
#3. 勾选仿真模块, 仿真报告上传模块
#注 1和2可以省略

import sys
import subprocess
import argparse
import os
import requests
from polling import poll
import time
import json
from datetime import datetime

def check_status():
    get_url = f"http://172.30.170.35:9003/manager/simulation-task-api/{current_time}/"
    response = requests.get(url=get_url)
    res = response.json()
    print('res:' + res['status'])
    if res['status'] != 'pending':
        return res

if __name__ == '__main__':
    #整合发送内容
    in_dir = sys.argv[1]
    target_branch = sys.argv[2] if len(sys.argv) >= 3 else 'main'
    commit_id2 = sys.argv[3] if len(sys.argv) >= 4 else ''
    current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    url = "http://172.30.170.35:9003/manager/simulation-task-api/"
    headers = {"Content-Type": "application/json"}
    data = {
        "task_id": current_time,
        "version_1": os.getenv('GIT_BRANCH'),
        "version_2": target_branch,
        "commit_id_1": os.getenv('COMMIT_ID'),
        "commit_id_2": commit_id2,
        "current_time": current_time,
        "in_dir": in_dir
    }
    response = requests.post(url, headers=headers, json=data)
    if response.json()['status'] == 'failed':
        print('ci Simulate failed, please check sim server status!')
        exit()
    #轮询发送数据
    print("The simulation program has started running.")
    result = poll(
        check_status,
        timeout=3 * 60 * 60,
        step=60,
        ignore_exceptions=(requests.exceptions.ConnectionError,)
    )
    # #处理收到的数据
    if result['in_dir'] != '':
        dir_path = "target/sim_report"
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)
        file_path = os.path.join(dir_path, "sim_report.txt")
        with open(file_path, "w", encoding="utf-8") as file:
            file.write('sim_start_time: ' + current_time + '\n')
            file.write('sim_result: ' + result['status']+ '\n')
            file.write('sim_server: 172.30.170.34\n')
            file.write('sim_report_path: /data_cold/abu_zone/simulation_test_result/CI/'+ current_time)
        exit()
    print("task timeout!")
    