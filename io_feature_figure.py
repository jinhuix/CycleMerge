import ctypes
from ctypes import c_uint32, c_int32, POINTER, Structure, byref
import time
from public import HeadInfo, IOUint, IOVector, InputParam, OutputParam
from public import AccessTime, TapeBeltSegWearInfo, KeyMetrics, lib
from public import read_case_file
import os
os.environ['MPLBACKEND'] = 'Agg'
import matplotlib.pyplot as plt
import numpy as np
import plotly.graph_objects as go
import math

x = []
y = []
z_values = {}
seekt_arr = []
pos_arr = []
wrap_start = 0
pos_start = 20000
for w in [2]:
# for w in range(0, 280, 7):
    # for l in [300000, 500000]:
    for l in range(1, 10000, 100):
        start = HeadInfo(wrap=wrap_start, lpos=pos_start, status=1)
        end = HeadInfo(wrap=w, lpos=l, status=1)
        seek_time = lib.SeekTimeCalculate(byref(start), byref(end))
        x.append(l)
        y.append(w)
        z_values[(l, w)] = seek_time
        # 记录pos差值带来的寻道时延差异
        pos_arr.append(abs(pos_start-l))
        seekt_arr.append(seek_time)
        print(f"Seek Time from (wrap: {start.wrap}, lpos: {start.lpos}) to (wrap: {end.wrap}, lpos: {end.lpos}): {seek_time} ms")

x_ticks = sorted(list(x))
y_ticks = sorted(list(y))
x, y = np.meshgrid(np.array(x_ticks), np.array(y_ticks))

print(x.shape, y.shape)

z = []
for i in range(len(x)):
    z.append([])
    for j in range(len(x[i])):
        z[i].append(z_values[(x[i][j], y[i][j])])

print(len(z), len(z[0]))

# 定义 x 和 y 的数据
pos_arr = np.array(pos_arr)
seekt_arr = np.array(seekt_arr)

# 使用 polyfit 进行一次函数拟合
coefficients = np.polyfit(pos_arr, seekt_arr, 1)

# 获取拟合函数的系数
a = coefficients[0]
b = coefficients[1]

# 打印拟合函数
print(f"拟合函数: y = {a:.2f} * x + {b:.2f}")