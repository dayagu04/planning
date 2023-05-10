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

    def load_ego_motion(self):
        if self.record is None:
            print('Error: Record object is not initialized.')
            return None
        ego_motion = {'data':[], 't':[]}
        try:
            for message, t in self.bagdata.read_messages('/ego_motion'):
                ego_motion['data'].append(message)
                ego_motion['t'].append(t)
        except Exception as e:
            print(f'Error: Failed to read messages from /ego_motion: {e}')
            return None
        return ego_motion
    