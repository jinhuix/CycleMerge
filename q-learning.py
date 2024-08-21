import ctypes
from ctypes import c_uint32, c_int32, POINTER, Structure, byref
import time
import numpy as np
import random
from public import HeadInfo, IOUint, IOVector, InputParam, OutputParam
from public import AccessTime, TapeBeltSegWearInfo, KeyMetrics, lib
from public import read_case_file

# 初始化磁头状态和IO序列
head_info, io_list = read_case_file('./dataset/case_2.txt')

print(f"Head Info: {head_info}")
# id, wrap, startLpos, endLpos
print(f"IO List: {io_list}")

# 创建输入参数
io_array = (IOUint * len(io_list))(*[IOUint(*io) for io in io_list])
io_vector = IOVector(len=len(io_list), ioArray=io_array)
input_param = InputParam(headInfo=HeadInfo(*head_info), ioVec=io_vector)

# Q-learning 参数
alpha = 0.1  # 学习率
gamma = 0.9  # 折扣因子
epsilon = 0.5  # 探索率
num_episodes = 100000  # 训练轮数

# 初始化 Q 表
# num_states = (max(IOUint(*io).wrap for io in io_list) + 1, max(max(IOUint(*io).endLpos,IOUint(*io).startLpos) for io in io_list) + 1)
# print(f"Number of States: {num_states}")
num_actions = len(io_list)
# Q = np.zeros((*num_states, num_actions))

num_io_requests = len(io_list)
Q = np.zeros((num_io_requests + 1, num_io_requests + 1))

# 初始化 Q 表为两两位置之间的寻址耗时
for i in range(num_io_requests + 1):
    for j in range(1, num_io_requests + 1):
        if i == j:
            Q[i][j] = -np.inf
            continue
        if i == 0:
            # 初始状态到各个 IO 请求的寻道时间
            start = HeadInfo(wrap=input_param.headInfo.wrap, lpos=input_param.headInfo.lpos, status=1)
            end = HeadInfo(wrap=IOUint(*io_list[j-1]).wrap, lpos=IOUint(*io_list[j-1]).startLpos, status=1)
        else:
            # 各个 IO 请求之间的寻道时间
            start = HeadInfo(wrap=IOUint(*io_list[i-1]).wrap, lpos=IOUint(*io_list[i-1]).endLpos, status=1)
            end = HeadInfo(wrap=IOUint(*io_list[j-1]).wrap, lpos=IOUint(*io_list[j-1]).startLpos, status=1)
        
        seek_time = lib.SeekTimeCalculate(byref(start), byref(end))
        Q[i][j] = -seek_time
        # print(f"Seek Time from (wrap: {start.wrap}, lpos: {start.lpos}) to (wrap: {end.wrap}, lpos: {end.lpos}): {seek_time} ms")


# 定义奖励函数
def calculate_reward(start, end):
    seek_time = lib.SeekTimeCalculate(byref(start), byref(end))
    read_time = lib.ReadTimeCalculate(abs(start.lpos - end.lpos))
    total_time = seek_time + read_time
    return -total_time  # 奖励为负的总时间

# Q-learning 算法
for episode in range(num_episodes):
    if episode % 10000 == 0:
        print(f"Episode {episode + 1}/{num_episodes}")
    start = HeadInfo(wrap=input_param.headInfo.wrap, lpos=input_param.headInfo.lpos, status=input_param.headInfo.status)
    # state = (start.wrap, start.lpos)  # 初始状态
    state = 0  # 初始状态
    visited = set()  # 记录已访问的动作
    for t in range(num_actions):
        if random.uniform(0, 1) < epsilon:
            action = random.choice([a for a in range(num_actions) if a not in visited])  # 探索
        else:
            # 生成噪声
            noise = np.random.randn(num_io_requests + 1) * (1. / (episode + 1))
            # 添加噪声到 Q 值
            q_values_with_noise = Q[state] + noise
            # 过滤已访问过的动作
            available_actions = [a for a in range(num_io_requests + 1) if a not in visited]
            available_q_values_with_noise = [q_values_with_noise[a] for a in available_actions]
            # 选择 Q 值最大的动作
            action = available_actions[np.argmax(available_q_values_with_noise)]

        curIO = action
        end = HeadInfo(wrap=input_param.ioVec.ioArray[curIO].wrap, lpos=input_param.ioVec.ioArray[curIO].startLpos, status=1)
        reward = calculate_reward(start, end)
        # next_state = (end.wrap, end.lpos)
        next_state = curIO

        # 更新 Q 表
        Q[state][action] = Q[state][action] + alpha * (reward + gamma * np.max(Q[next_state]) - Q[state][action])

        start = end
        state = next_state
        visited.add(action)  # 标记动作为已访问

# 找出最优序列
optimal_sequence = []
start = HeadInfo(wrap=input_param.headInfo.wrap, lpos=input_param.headInfo.lpos, status=input_param.headInfo.status)
# state = (start.wrap, start.lpos)
state = 0
visited = set()  # 记录已访问的动作
for t in range(num_actions):
    available_actions = [a for a in range(1,num_actions+1) if a not in visited]
    print(f"Available Actions: {available_actions}")
    if not available_actions:
        break
    # 获取可用动作的 Q 值
    available_q_values = [Q[state][action] for action in available_actions]
    # 找到最大 Q 值的索引
    max_q_index = np.argmax(available_q_values)
    # 选择对应的动作
    action = available_actions[max_q_index]
    optimal_sequence.append(action)
    print(f"Action {t + 1}: {action}")
    curIO = action
    end = HeadInfo(wrap=input_param.ioVec.ioArray[curIO].wrap, lpos=input_param.ioVec.ioArray[curIO].startLpos, status=1)
    start = end
    # state = (start.wrap, start.lpos)
    state = curIO
    visited.add(action)  # 标记动作为已访问

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