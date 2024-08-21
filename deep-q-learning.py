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

# 初始化磁头状态和IO序列
head_info, io_list = read_case_file('./dataset/case_2.txt')

# 创建输入参数
io_array = (IOUint * len(io_list))(*[IOUint(*io) for io in io_list])
io_vector = IOVector(len=len(io_list), ioArray=io_array)
input_param = InputParam(headInfo=HeadInfo(*head_info), ioVec=io_vector)

# DQN 参数
gamma = 0.9  # 折扣因子
epsilon = 0.5  # 探索率
epsilon_min = 0.01
epsilon_decay = 0.995
learning_rate = 0.001
num_episodes = 10000  # 训练轮数
batch_size = 64
memory_size = 10000

# 定义 Q 网络
class QNetwork(nn.Module):
    def __init__(self, state_size, action_size):
        super(QNetwork, self).__init__()
        self.fc1 = nn.Linear(state_size, 128)
        self.fc2 = nn.Linear(128, 128)
        self.fc3 = nn.Linear(128, action_size)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return self.fc3(x)

# 初始化 Q 网络
state_size = 2  # wrap 和 lpos
action_size = len(io_list)
q_network = QNetwork(state_size, action_size)
target_network = QNetwork(state_size, action_size)
target_network.load_state_dict(q_network.state_dict())
target_network.eval()

optimizer = optim.Adam(q_network.parameters(), lr=learning_rate)
memory = deque(maxlen=memory_size)

# 定义奖励函数
def calculate_reward(start, end):
    seek_time = lib.SeekTimeCalculate(byref(start), byref(end))
    read_time = lib.ReadTimeCalculate(abs(start.lpos - end.lpos))
    total_time = seek_time + read_time
    return -total_time  # 奖励为负的总时间

# 存储经验
def store_experience(state, action, reward, next_state, done):
    memory.append((state, action, reward, next_state, done))

# 选择动作
def select_action(state, epsilon):
    if random.uniform(0, 1) < epsilon:
        return random.choice(range(action_size))  # 探索
    else:
        state = torch.FloatTensor(state).unsqueeze(0)
        with torch.no_grad():
            q_values = q_network(state)
        return q_values.argmax().item()  # 利用

# 训练 Q 网络
def train_q_network():
    if len(memory) < batch_size:
        return
    batch = random.sample(memory, batch_size)
    states, actions, rewards, next_states, dones = zip(*batch)

    states = torch.FloatTensor(states)
    actions = torch.LongTensor(actions)
    rewards = torch.FloatTensor(rewards)
    next_states = torch.FloatTensor(next_states)
    dones = torch.FloatTensor(dones)

    q_values = q_network(states).gather(1, actions.unsqueeze(1)).squeeze(1)
    next_q_values = target_network(next_states).max(1)[0]
    target_q_values = rewards + (gamma * next_q_values * (1 - dones))

    loss = nn.MSELoss()(q_values, target_q_values.detach())
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()

# DQN 算法
for episode in range(num_episodes):
    if episode % 100 == 0:
        print(f"Episode {episode + 1}/{num_episodes}")
    start = HeadInfo(wrap=input_param.headInfo.wrap, lpos=input_param.headInfo.lpos, status=input_param.headInfo.status)
    state = (start.wrap, start.lpos)  # 初始状态
    state = np.array(state)
    visited = set()  # 记录已访问的动作
    total_reward = 0
    for t in range(action_size):
        action = select_action(state, epsilon)
        while action in visited:
            action = select_action(state, epsilon)
        curIO = action
        end = HeadInfo(wrap=input_param.ioVec.ioArray[curIO].wrap, lpos=input_param.ioVec.ioArray[curIO].startLpos, status=1)
        reward = calculate_reward(start, end)
        next_state = (end.wrap, end.lpos)
        next_state = np.array(next_state)
        done = t == action_size - 1

        store_experience(state, action, reward, next_state, done)
        train_q_network()

        start = end
        state = next_state
        visited.add(action)  # 标记动作为已访问
        total_reward += reward

    if epsilon > epsilon_min:
        epsilon *= epsilon_decay

    if episode % 1000 == 0:
        target_network.load_state_dict(q_network.state_dict())
        print(f"Total Reward: {total_reward}")

# 找出最优序列
optimal_sequence = []
start = HeadInfo(wrap=input_param.headInfo.wrap, lpos=input_param.headInfo.lpos, status=input_param.headInfo.status)
state = (start.wrap, start.lpos)
state = np.array(state)
visited = set()  # 记录已访问的动作
for t in range(action_size):
    available_actions = [a for a in range(action_size) if a not in visited]
    if not available_actions:
        break
    state_tensor = torch.FloatTensor(state).unsqueeze(0)
    q_values = q_network(state_tensor)
    action = q_values.argmax().item()
    while action in visited:
        q_values[0, action] = float('-inf')
        action = q_values.argmax().item()
    optimal_sequence.append(action + 1)
    curIO = action
    end = HeadInfo(wrap=input_param.ioVec.ioArray[curIO].wrap, lpos=input_param.ioVec.ioArray[curIO].startLpos, status=1)
    start = end
    state = (start.wrap, start.lpos)
    state = np.array(state)
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