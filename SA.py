import ctypes
from ctypes import c_uint32, c_int32, POINTER, Structure, byref
import time
from public import HeadInfo, IOUint, IOVector, InputParam, OutputParam
from public import AccessTime, TapeBeltSegWearInfo, KeyMetrics, lib
from public import read_case_file
import random
import math

# 初始化磁头状态和IO序列
head_info, io_list = read_case_file('./dataset/case_5.txt')

# 创建输入参数
io_array = (IOUint * len(io_list))(*[IOUint(*io) for io in io_list])
io_vector = IOVector(len=len(io_list), ioArray=io_array)
input_param = InputParam(headInfo=HeadInfo(*head_info), ioVec=io_vector)

# 定义目标函数
def calculate_total_delay(sequence, input_param):
    output_param = OutputParam(len=len(sequence), sequence=(c_uint32 * len(sequence))(*sequence))
    access_time = AccessTime()
    lib.TotalAccessTime(byref(input_param), byref(output_param), byref(access_time))
    return access_time.addressDuration + access_time.readDuration

# 初始化参数
initial_temperature = 1000
cooling_rate = 0.995
min_temperature = 1
max_iterations = 1000

# 初始化解
current_sequence = list(range(1, len(io_list) + 1))
current_cost = calculate_total_delay(current_sequence, input_param)
best_sequence = current_sequence[:]
best_cost = current_cost

# 定义邻域结构
def get_neighbor(sequence):
    neighbor = sequence[:]
    i, j = random.sample(range(len(sequence)), 2)
    neighbor[i], neighbor[j] = neighbor[j], neighbor[i]
    return neighbor

# 模拟退火过程
temperature = initial_temperature
iteration = 0

while temperature > min_temperature and iteration < max_iterations:
    neighbor_sequence = get_neighbor(current_sequence)
    neighbor_cost = calculate_total_delay(neighbor_sequence, input_param)

    if neighbor_cost < current_cost or random.uniform(0, 1) < math.exp((current_cost - neighbor_cost) / temperature):
        current_sequence = neighbor_sequence
        current_cost = neighbor_cost

        if current_cost < best_cost:
            best_sequence = current_sequence
            best_cost = current_cost

    temperature *= cooling_rate
    iteration += 1

print(f"Best sequence: {best_sequence}")
print(f"Best cost: {best_cost}")