import ctypes
from ctypes import c_uint32, c_int32, POINTER, Structure, byref
import time
import numpy as np
import random
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque
from public import HeadInfo, IOUint, IOVector, InputParam, OutputParam
from public import AccessTime, TapeBeltSegWearInfo, KeyMetrics, lib
from public import read_case_file
from reinforcement_learning import QNetwork
# DQN 参数
gamma = 0.9  # 折扣因子
epsilon_start = 1.0
epsilon_end = 0.01
epsilon_decay = 0.995

epsilon = epsilon_start
epsilon_min = 0.01
learning_rate = 0.001
num_episodes = 3  # 训练轮数
batch_size = 64
memory_size = 10000
hidden_size1 = 128
hidden_size2 = 64
# 初始化 Q 网络
state_size = 2  # wrap 和 lpos
event_feature_size = 3 # wrap, startLpos, endLpos

q_network = QNetwork(state_size, event_feature_size, hidden_size1, hidden_size2)
target_network = QNetwork(state_size, event_feature_size, hidden_size1, hidden_size2)
optimizer = optim.Adam(q_network.parameters(), lr=learning_rate)
# 加载模型
checkpoint = torch.load('model.pth')
q_network.load_state_dict(checkpoint['q_network_state_dict'])
optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
epsilon = checkpoint['epsilon']
# 找出最优序列
optimal_sequence = []
# 初始化磁头状态和IO序列
head_info, io_list = read_case_file('./dataset/case_1.txt')
# 创建输入参数
io_array = (IOUint * len(io_list))(*[IOUint(*io) for io in io_list])
io_vector = IOVector(len=len(io_list), ioArray=io_array)
input_param = InputParam(headInfo=HeadInfo(*head_info), ioVec=io_vector)
state_size = 2  # wrap 和 lpos
event_feature_size = 3 # wrap, startLpos, endLpos
action_size = len(io_list)
start = HeadInfo(wrap=input_param.headInfo.wrap, lpos=input_param.headInfo.lpos, status=input_param.headInfo.status)
state = (start.wrap, start.lpos)
state = np.array(state)
visited = set()  # 记录已访问的事件索引
# 准备所有可能的事件特征
event_features = []
for i in range(input_param.ioVec.len):
    io = input_param.ioVec.ioArray[i]
    event_feature = (io.wrap, io.startLpos, io.endLpos)  # 假设事件特征由这些属性组成
    event_features.append(event_feature)
event_features = np.array(event_features)

for t in range(action_size):
    available_actions = [a for a in range(action_size) if a not in visited]
    if not available_actions:
        break
    state_tensor = torch.FloatTensor(state).unsqueeze(0)
    available_event_features = event_features[available_actions]
    event_feature_tensors = torch.FloatTensor(available_event_features)
    q_values = q_network(state_tensor, event_feature_tensors).squeeze(1)
    action_index = q_values.argmax().item()
    action = available_actions[action_index]
    optimal_sequence.append(action + 1)
    curIO = input_param.ioVec.ioArray[action]
    end = HeadInfo(wrap=curIO.wrap, lpos=curIO.startLpos, status=1)
    start = end
    state = (start.wrap, start.lpos)
    state = np.array(state)
    visited.add(action)  # 标记事件索引为已访问

# 创建输出参数
output_param = OutputParam(len=len(io_list), sequence=(c_uint32 * len(io_list))(*optimal_sequence))

print(f"Optimal Sequence: {optimal_sequence}")

# 初始化磁头状态
start = HeadInfo(wrap=input_param.headInfo.wrap, lpos=input_param.headInfo.lpos, status=input_param.headInfo.status)

# 初始化总时间和磨损
total_seek_time = 0
total_read_time = 0
total_belt_wear = 0
total_motor_wear = 0

# 计算各个seek和read操作的时间和磨损
for i in range(output_param.len):
    curIO = output_param.sequence[i] - 1
    end = HeadInfo(wrap=input_param.ioVec.ioArray[curIO].wrap, lpos=input_param.ioVec.ioArray[curIO].startLpos, status=1)
    seek_time = lib.SeekTimeCalculate(byref(start), byref(end))
    belt_wear = lib.BeltWearTimes(byref(start), byref(end), None)
    motor_wear = lib.MotorWearTimes(byref(start), byref(end))
    total_seek_time += seek_time
    total_belt_wear += belt_wear
    total_motor_wear += motor_wear
    print(f"Seek from (wrap: {start.wrap}, lpos: {start.lpos}) to (wrap: {end.wrap}, lpos: {end.lpos})")
    print(f"Seek Time: {seek_time} ms, Belt Wear: {belt_wear} lpos, Motor Wear: {motor_wear} lpos")

    start.wrap = input_param.ioVec.ioArray[curIO].wrap
    start.lpos = input_param.ioVec.ioArray[curIO].startLpos
    start.status = 1
    end.wrap = input_param.ioVec.ioArray[curIO].wrap
    end.lpos = input_param.ioVec.ioArray[curIO].endLpos
    end.status = 1
    read_time = lib.ReadTimeCalculate(abs(start.lpos - end.lpos))
    belt_wear = lib.BeltWearTimes(byref(start), byref(end), None)
    motor_wear = lib.MotorWearTimes(byref(start), byref(end))
    total_read_time += read_time
    total_belt_wear += belt_wear
    total_motor_wear += motor_wear
    print(f"Read from (wrap: {start.wrap}, lpos: {start.lpos}) to (wrap: {end.wrap}, lpos: {end.lpos})")
    print(f"Read Time: {read_time} ms, Belt Wear: {belt_wear} lpos, Motor Wear: {motor_wear} lpos\n")
    start = end

# 计算总时间和磨损
access_time = AccessTime()
# 创建 TapeBeltSegWearInfo 实例
seg_wear_info = TapeBeltSegWearInfo()

lib.TotalAccessTime(byref(input_param), byref(output_param), byref(access_time))
total_belt_wear_lib = lib.TotalTapeBeltWearTimes(byref(input_param), byref(output_param), byref(seg_wear_info))
total_motor_wear_lib = lib.TotalMotorWearTimes(byref(input_param), byref(output_param))

print(f"Total Seek Time: {total_seek_time} ms")
print(f"Total Read Time: {total_read_time} ms")
print(f"Total Belt Wear: {total_belt_wear} lpos")
print(f"Total Motor Wear: {total_motor_wear} lpos\n")

print(f"Total Addressing Duration (lib): {access_time.addressDuration} ms")
print(f"Total Read Duration (lib): {access_time.readDuration} ms")
print(f"Total Tape Belt Wear (lib): {total_belt_wear_lib} times")
print(f"Total Motor Wear (lib): {total_motor_wear_lib} times")