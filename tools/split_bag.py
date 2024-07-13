'''
截取bag的脚本, 支持批量截取和单个截取
1, 批量截取: 创建一个txt文本, 每一行的第一个元素表示要截取的bag路径, 第二个元素表示截取的开始时间，第三个元素表示截取的结束时间，元素之间用空格分隔
             然后将txt文件的路径填入函数中的txt_path, 运行 python split_bag.py
2, 单个截取: python split_bag.py [bag_path] [start_second] [end_second]
             e.g.: python split_bag.py xxx.bag 15 30
'''

import sys
import os
import subprocess
from datetime import datetime, timedelta

def add_seconds(time_str, seconds):
    time_format = "%b %d %Y %H:%M:%S.%f"
    time_obj = datetime.strptime(time_str, time_format)
    new_time_obj = time_obj + timedelta(seconds=seconds)
    new_time_str = new_time_obj.strftime(time_format)
    return new_time_str
def split_bag(bag_path, start_second, end_second):
  process = subprocess.Popen(["rosbag", "info", bag_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
  for line in process.stdout:
    str_out = line.decode("utf-8").strip()
    if str_out.startswith("start"):
      str_out = str_out.split(" "*7)
      time_str = float(str_out[1].rsplit(" ", 1)[1].strip('(').strip(')'))
      start_time_str = str(time_str + float(start_second))
      end_time_str = str(time_str + float(end_second))
      # start_time_str = add_seconds(time_str, int(start_second))
      # end_time_str = add_seconds(time_str, int(end_second))
      cmd = f"rosbag filter {bag_path} {bag_path + '.' + start_second + '-' + end_second +'.split'}  't.to_sec() >=  {start_time_str}  and t.to_sec() <= {end_time_str}'"
      subprocess.call(cmd, shell=True)
if __name__ == "__main__":
  txt_path = "/docker_share/planning/test.txt"
  if len(sys.argv) == 1:
    if os.path.isfile(txt_path):
      with open(txt_path, 'r', encoding='utf-8') as file:
        for line in file:
            words = line.strip().split()
            bag_path = words[0]
            if bag_path.startswith('"') or bag_path.startswith("'"):
                bag_path = bag_path[1:]
            if bag_path.endswith('"') or bag_path.endswith("'"):
                bag_path = bag_path[:-1]
            start_second = words[1]
            end_second = words[2]
            print("bag_path: ", bag_path)
            print("start_second: ", start_second)
            print("end_second: ", end_second)
            split_bag(bag_path, start_second, end_second)
    else:
      print("txt_path不存在")
  elif len(sys.argv) == 4:
    if os.path.isfile(sys.argv[1]):
      split_bag(sys.argv[1], sys.argv[2], sys.argv[3])
    else:
      print("bag不存在")
  else:
     print("Error, wrong way to use")