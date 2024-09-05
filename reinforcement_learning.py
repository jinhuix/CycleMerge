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
head_info, io_list = read_case_file('./dataset/case_6.txt')

# 创建输入参数
io_array = (IOUint * len(io_list))(*[IOUint(*io) for io in io_list])
io_vector = IOVector(len=len(io_list), ioArray=io_array)
input_param = InputParam(headInfo=HeadInfo(*head_info), ioVec=io_vector)

# DQN 参数
gamma = 0.9  # 折扣因子
epsilon_start = 1.0
epsilon_end = 0.01
epsilon_decay = 0.995

epsilon = epsilon_start
epsilon_min = 0.01
learning_rate = 0.001
num_episodes = 1000  # 训练轮数
batch_size = 64
memory_size = 10000
hidden_size1 = 128
hidden_size2 = 64

# 定义 Q 网络
class QNetwork(nn.Module):
    def __init__(self, state_size, event_feature_size, hidden_size1, hidden_size2):
        super(QNetwork, self).__init__()
        self.fc1 = nn.Linear(state_size + event_feature_size, hidden_size1)
        self.fc2 = nn.Linear(hidden_size1, hidden_size2)
        self.fc3 = nn.Linear(hidden_size2, 1)

    def forward(self, state, event_features):
        x = torch.cat((state, event_features), dim=-1)
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        q_value = self.fc3(x)
        return q_value

# 初始化 Q 网络
state_size = 2  # wrap 和 lpos
event_feature_size = 3 # wrap, startLpos, endLpos
action_size = len(io_list)
q_network = QNetwork(state_size, event_feature_size, hidden_size1, hidden_size2)
target_network = QNetwork(state_size, event_feature_size, hidden_size1, hidden_size2)
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
def store_experience(state, event_feature, reward, next_state, done):
    memory.append((state, event_feature, reward, next_state, done))

# 训练Q网络
def train_q_network(event_features_arr):
    if len(memory) < batch_size:
        return
    batch = random.sample(memory, batch_size)
    states, event_features, rewards, next_states, dones = zip(*batch)
    states = np.array(states)
    states = torch.FloatTensor(states)
    event_features = np.array(event_features)
    event_features = torch.FloatTensor(event_features)
    rewards = np.array(rewards)
    rewards = torch.FloatTensor(rewards)
    next_states = np.array(next_states)
    next_states = torch.FloatTensor(next_states)
    dones = np.array(dones)
    dones = torch.FloatTensor(dones)
    event_features_arr = torch.FloatTensor(event_features_arr)

    q_values = q_network(states, event_features)

    next_q_values = []
    for next_state in next_states:
        next_q_value = float('-inf')
        for event_feature in event_features_arr:
            if target_network(next_state, event_feature) > next_q_value:
                next_q_value = target_network(next_state, event_feature)
        next_q_values.append(next_q_value)
    next_q_values = torch.FloatTensor(next_q_values)
    target_q_values = rewards + (gamma * next_q_values * (1 - dones))
    # 计算所有可能的下一个事件特征

    # next_event_features = torch.FloatTensor(event_features_arr).unsqueeze(0).repeat(batch_size, 1, 1)

    # # 计算下一个状态的最大Q值
    # next_q_values = target_network(next_states.unsqueeze(1).repeat(1, len(event_features_arr), 1), next_event_features)
    # next_q_values, _ = next_q_values.max(dim=1)

    # target_q_values = rewards + (gamma * next_q_values.unsqueeze(1) * (1 - dones))


    loss = nn.MSELoss()(q_values, target_q_values.detach())
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()

def select_action(state, event_features, epsilon):
    if random.uniform(0, 1) < epsilon:
        return random.choice(range(len(event_features)))  # 探索
    else:
        state = torch.FloatTensor(state).unsqueeze(0)
        event_features = torch.FloatTensor(event_features)
        with torch.no_grad():
            q_values = []
            for event_feature in event_features:
                event_feature = event_feature.unsqueeze(0)
                q_value = q_network(state, event_feature)
                q_values.append(q_value)
            q_values = torch.stack(q_values).squeeze(1)
        return q_values.argmax().item()  # 利用

# DQN 算法
for episode in range(num_episodes):
    if episode % 1 == 0:
        print(f"Episode {episode + 1}/{num_episodes}")
    start = HeadInfo(wrap=input_param.headInfo.wrap, lpos=input_param.headInfo.lpos, status=input_param.headInfo.status)
    state = (start.wrap, start.lpos)  # 初始状态
    state = np.array(state)
    visited = set()  # 记录已访问的动作
    total_reward = 0
    
    # 准备所有可能的事件特征
    event_features = []
    for i in range(input_param.ioVec.len):
        io = input_param.ioVec.ioArray[i]
        event_feature = (io.wrap, io.startLpos, io.endLpos)  # 假设事件特征由这些属性组成
        event_features.append(event_feature)
    print(len(event_features))
    event_features = np.array(event_features)
    
    for t in range(action_size):
        action_index = select_action(state, event_features, epsilon)
        while action_index in visited:
            action_index = select_action(state, event_features, epsilon)
        
        curIO = input_param.ioVec.ioArray[action_index]
        end = HeadInfo(wrap=curIO.wrap, lpos=curIO.startLpos, status=1)
        reward = calculate_reward(start, end)
        next_state = (end.wrap, end.lpos)
        next_state = np.array(next_state)
        done = t == action_size - 1
        
        # 存储经验时,同时存储状态和事件特征
        store_experience(state, event_features[action_index], reward, next_state, done)
        train_q_network(event_features)
        
        start = end
        state = next_state
        visited.add(action_index)  # 标记动作为已访问
        total_reward += reward
    
    epsilon = max(epsilon_end, epsilon_decay * epsilon)
    
    if episode % 64 == 0:
        target_network.load_state_dict(q_network.state_dict())
        print(f"Total Reward: {total_reward}")
# 保存模型
torch.save({
    'q_network_state_dict': q_network.state_dict(),
    'optimizer_state_dict': optimizer.state_dict(),
    'epsilon': epsilon
}, 'model.pth')
