import ctypes
from ctypes import c_uint32,c_uint,c_int, c_int32, POINTER, Structure, byref
import time
from public import HeadInfo, IOUint, IOVector, InputParam, OutputParam
from public import AccessTime, TapeBeltSegWearInfo, KeyMetrics, lib
from public import read_case_file
import matplotlib.pyplot as plt
import numpy as np

# sym=dir(lib)
# for s in sym:
#     print(s)

# 加载共享库
lib2 = ctypes.CDLL('./build/libtest_algorithms.so')



# 定义函数原型
lib2.partition_scan_t.argtypes = [POINTER(InputParam), POINTER(OutputParam), c_int, POINTER(c_int), c_int]
lib2.partition_scan_t.restype = c_int32

# # 列出共享库中的所有符号
# symbols = dir(lib2)
# # 打印所有符号
# for symbol in symbols:
#     print(symbol)

# 读取文件内容
def read_input_param_from_file(file_path):
    head_info, io_list= read_case_file(file_path)
    # print("head_info:", head_info)
    # print("io_list:", io_list)

    # 解析 io_count
    io_count = int(len(io_list))
    io_array = (IOUint * io_count)()
    for i in range(io_count):
        io_array[i].id = io_list[i][0]
        io_array[i].wrap = io_list[i][1]
        io_array[i].startLpos = io_list[i][2]
        io_array[i].endLpos = io_list[i][3]

    input_param = InputParam()
    input_param.headInfo.wrap = head_info[0]
    input_param.headInfo.lpos = head_info[1]
    input_param.headInfo.status = head_info[2]
    input_param.ioVec = IOVector(len=io_count, ioArray=io_array)
    return input_param