import ctypes
from ctypes import c_uint32, c_int32, POINTER, Structure, byref
import time
from public import HeadInfo, IOUint, IOVector, InputParam, OutputParam
from public import AccessTime, TapeBeltSegWearInfo, KeyMetrics, lib
from public import read_case_file
import matplotlib.pyplot as plt
import numpy as np

x=[]
y=[]


z_values = {}

# for w in range(0,280,10):
for w in range(0,280,7):
    for l in range(1,730994,10000):
        start = HeadInfo(wrap=0, lpos=0, status=1)
        end = HeadInfo(wrap=w, lpos=l, status=1)
        seek_time = lib.SeekTimeCalculate(byref(start), byref(end))
        x.append(l)
        y.append(w)
        z_values[(l,w)] = seek_time
        print(f"Seek Time from (wrap: {start.wrap}, lpos: {start.lpos}) to (wrap: {end.wrap}, lpos: {end.lpos}): {seek_time} ms")

x_ticks = sorted(list(x))
y_ticks = sorted(list(y))
x, y = np.meshgrid(np.array(x_ticks), np.array(y_ticks))

print(x.shape,y.shape)

z=[]
for i in range(len(x)):
    z.append([])
    for j in range(len(x[i])):
        z[i].append(z_values[(x[i][j],y[i][j])])

print(len(z),len(z[0]))

# 创建图形
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制点
ax.plot_surface(x, y, np.array(z), cmap='coolwarm')

# 设置标签
ax.set_xlabel('Lpos Difference')
ax.set_ylabel('Wrap Difference')
ax.set_zlabel('Seek Time')

# 显示图形
plt.show()