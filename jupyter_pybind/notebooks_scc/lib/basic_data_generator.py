import numpy as np
from abc import ABC, abstractmethod

# 数据处理的基本类
class DataGeneratorBase(ABC):
    def __init__(self, xys, ts, accu=False, name="Default Name"):
        self.xys = xys
        self.ts = np.array(ts)

        self.accu = accu
        self.name = name

    def getMinT(self):
        if len(self.ts) > 0:
            return self.ts[0]
        else:
            return float('inf')

    def getMaxT(self):
        if len(self.ts) > 0:
            return self.ts[-1]
        else:
            return -1.0

    def atTa(self, T):
        if len(self.xys[0]) == 0:
            return [[], []]
        if self.getMinT() > T:
            return (self.xys[0][0:1], self.xys[1][0:1])

        first_larger_index = np.where(self.ts > T)[0]
        if first_larger_index.size == 0:
            return (self.xys[0], self.xys[1])

        return (self.xys[0][0: first_larger_index[0]], self.xys[1][0:first_larger_index[0]])

    def atT(self, T):
        if self.accu:
            return self.atTa(T)
        if len(self.xys) == 0:
            return [[], []]
        if self.getMinT() > T:
            return self.xys[0]

        first_larger_index = np.where(self.ts > T)[0]
        if first_larger_index.size == 0:
            return self.xys[-1]

        return self.xys[first_larger_index[0] - 1]

class LineGenerator(DataGeneratorBase):
  def __init__(self, attr):
    self.ts = []
    self.xys = []
    super().__init__(self.xys, self.ts)
    self.line = attr
    return


class CommonGenerator(DataGeneratorBase):
  def __init__(self):
    self.ts = []
    self.xys = []
    super().__init__(self.xys, self.ts)
    return
class TextGenerator(DataGeneratorBase):
  def __init__(self):
    self.ts = []
    self.xys = []
    self.txt = "text"
    super().__init__(self.xys, self.ts)
    return

class CircleGenerator(DataGeneratorBase):
  def __init__(self):
    self.ts = []
    self.xys = []
    self.rs = []
    super().__init__(self.xys, self.ts)
    return

class WedgesGenerator(DataGeneratorBase):
  def __init__(self):
    self.ts = []
    self.xys = []
    self.rs = []
    self.min_angle = []
    self.max_angle = []
    super().__init__(self.xys, self.ts)
    return
class DotGenerator(DataGeneratorBase):
  def __init__(self):
    self.ts = []
    self.xys = []
    super().__init__(self.xys, self.ts)
    return

class ObjTextGenerator(DataGeneratorBase):
  def __init__(self):
    self.ts = []
    self.xys = []
    self.obj_txt = "text"
    super().__init__(self.xys, self.ts)
    return