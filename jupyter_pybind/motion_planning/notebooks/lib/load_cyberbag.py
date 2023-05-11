import json
import numpy
import math
import os
import sys
import numpy as np
import copy
from cyber_record.record import Record

class CyberBagLoader:
    def __init__(self, bagfile):
        try:
            self.record = Record(bagfile)
        except OSError:
            print(f'Error: File {bagfile} does not exist.')
            self.record = None
    # 加载bag topics
    def load_topics(self):
        topic_list = []
        print('The bag has topics: ')
        for topic, message, t in self.record.read_messages():
            if topic not in topic_list:
                topic_list.append(topic)
                print('    ', topic)
        return topic_list

    # 1.加载自车运动状态信息:
    def load_ego_motion(self):
        if self.record is None:
            print('Error: Record object is not initialized.')
            return None
        ego_motion = {'data':[], 't':[]}
        try:
            for topic, message, t in self.record.read_messages('/ego_motion'):
                ego_motion['data'].append(message)
                ego_motion['t'].append(t)
        except Exception as e:
            print(f'Error: Failed to read messages from /ego_motion: {e}')
            return None
        return ego_motion

    # 2.加载自车定位信息:
    def load_localization(self):
        if self.record is None:
            print('Error: Record object is not initialized.')
            return None
        ego_motion = {'data':[], 't':[]}
        try:
            for topic, message, t in self.record.read_messages('/ego_motion'):
                ego_motion['data'].append(message)
                ego_motion['t'].append(t)
        except Exception as e:
            print(f'Error: Failed to read messages from /ego_motion: {e}')
            return None
        return ego_motion

    # 3.加载融合车道线信息:
    def load_fusion_road(self):
        if self.record is None:
            print('Error: Record object is not initialized.')
            return None
        ego_motion = {'data':[], 't':[]}
        try:
            for topic, message, t in self.record.read_messages('/ego_motion'):
                ego_motion['data'].append(message)
                ego_motion['t'].append(t)
        except Exception as e:
            print(f'Error: Failed to read messages from /ego_motion: {e}')
            return None
        return ego_motion

    # 4.加载融合障碍物信息:

    # 5.加载mobileye信息:

    # 6.加载control信息:
    def load_control_data(self):
        if self.record is None:
            print('Error: Record object is not initialized.')
            return None
        control_data = {'data':[], 't':[]}
        try:
            for topic, message, t in self.record.read_messages('/control_command'):
                control_data['data'].append(message)
                control_data['t'].append(t)
                # print(control_data)
        except Exception as e:
            print(f'Error: Failed to read messages from /control_command: {e}')
            return None
        return control_data
    # 7.加载planning信息:

    # 7.加载planning debug信息:
