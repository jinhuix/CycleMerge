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
# lib2 = ctypes.CDLL('./build/libtest_algorithms.so')
lib2 = ctypes.CDLL('./lib/libtest_algorithms.so')



# 定义函数原型
lib2.partition_scan_t.argtypes = [POINTER(InputParam), POINTER(OutputParam), c_int, POINTER(c_int), c_int]
lib2.partition_scan_t.restype = c_int32

lib2.p_scan_t.argtypes = [POINTER(InputParam), POINTER(OutputParam), c_int, POINTER(c_int), c_int, POINTER(c_int)]
lib2.p_scan_t.restype = c_int32


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

# # 初始化参数
# p_num = 4
# partition_len = 5000
# partitions = (c_int * p_num)()
# partitions[0] = 2
# partitions[1] = 4
# partitions[2] = 8
# partitions[3] = 133
# scan_method = (c_int * p_num)()
# scan_method[0] = 0
# scan_method[1] = 1
# scan_method[2] = 2
# scan_method[3] = 1

# # 假设 ioVec 和 ioArray 已经初始化
# io_array = (IOUint * 10)()
# for i in range(10):
#     io_array[i].startLpos = i * 100

# # 从文件中读取 input_param
# input_param = read_input_param_from_file('./dataset/case_5.txt')

# output_param = OutputParam()
# output_param.sequence = (c_uint * 10)()  # 分配 c_uint 数组
# output_param.len = 10

# # 调用函数
# # result = lib2.partition_scan_t(byref(input_param), byref(output_param), partition_len, partitions, p_num)
# result = lib2.p_scan_t(byref(input_param), byref(output_param), partition_len, partitions, p_num, scan_method)

# print("Result:", result)
# print("Output sequence:", [output_param.sequence[i] for i in range(output_param.len)])

# # 计算总时间和磨损
# access_time = AccessTime()
# # 创建 TapeBeltSegWearInfo 实例
# seg_wear_info = TapeBeltSegWearInfo()

# lib.TotalAccessTime(byref(input_param), byref(output_param), byref(access_time))
# total_belt_wear_lib = lib.TotalTapeBeltWearTimes(byref(input_param), byref(output_param), byref(seg_wear_info))
# total_motor_wear_lib = lib.TotalMotorWearTimes(byref(input_param), byref(output_param))
# print(f"Total Addressing Duration (lib): {access_time.addressDuration} ms")
# print(f"Total Read Duration (lib): {access_time.readDuration} ms")
# print(f"Total Tape Belt Wear (lib): {total_belt_wear_lib} times")
# print(f"Total Motor Wear (lib): {total_motor_wear_lib} times")