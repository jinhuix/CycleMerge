import re
import matplotlib.pyplot as plt
import os
import numpy as np

def parse_file(file_path):
    algorithms = []
    addressing_durations = []

    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()

    # 使用正则表达式解析文件内容
    algorithm_pattern = re.compile(r'/\* 算法名称 \*/\nAlgorithm: (.+?)\n/\* 关键指标结构体 \*/\nioCount: \d+ \nalgorithmRunningDuration\(ms\): [\d.]+ \nmemoryUse\(ms\): \d+ \naddressingDuration\(ms\): (\d+) \nreadDuration\(ms\): \d+\ntapeBeltWear\(times\): \d+\ntapeMotorWear\(times\): \d+ \nerrorIOCount: \d+ \n\n')
    matches = algorithm_pattern.findall(content)

    for match in matches:
        if(match[0] == 'FCFS'):
            continue
        if(match[0] == 'MPScanStar'):
            algorithms.append('MPScan*')
        else:
            algorithms.append(match[0])
        addressing_durations.append(int(match[1]))

    return algorithms, addressing_durations

def plot_addressing_durations(algorithms, addressing_durations, output_path):
    plt.figure(figsize=(10, 6))
    
    # 使用颜色渐变来为每个柱赋予颜色
    # cmap = plt.get_cmap('viridis')
    # cmap = plt.get_cmap('coolwarm')
    cmap = plt.get_cmap('RdBu')
    colors = cmap(np.linspace(0.2, 0.8, len(algorithms)))
    
    # 找出所有最小值的索引，并将这些索引对应的柱和其值标红
    min_duration = min(addressing_durations)
    min_indices = [i for i, duration in enumerate(addressing_durations) if duration == min_duration]
    
    # for i in min_indices:
    #     colors[i] = "red"
    
    bars = plt.bar(algorithms, addressing_durations, color=colors)
    plt.xlabel('Algorithms')
    plt.ylabel('addressingDuration (ms)')
    plt.title(os.path.basename(output_path)[:-4])
    plt.xticks(rotation=45, ha='right')
    
    # 在每个柱的顶部标注其值
    for bar in bars:
        yval = bar.get_height()
        color = 'red' if yval == min_duration else 'black'
        plt.text(bar.get_x() + bar.get_width()/2, yval + 1, int(yval), ha='center', va='bottom', color=color)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=500)
    plt.close()

def process_directory(directory_path):
    if not os.path.exists('./pic/bar'):
        os.makedirs('./pic/bar')

    for filename in os.listdir(directory_path):
        if filename.endswith('.txt'):
            file_path = os.path.join(directory_path, filename)
            algorithms, addressing_durations = parse_file(file_path)
            output_path = os.path.join('./pic/bar', f'{os.path.splitext(filename)[0]}.png')
            plot_addressing_durations(algorithms, addressing_durations, output_path)
            print(f'Saved plot to {output_path}')

if __name__ == "__main__":
    directory_path = './result'  # 替换为你的文件夹路径
    process_directory(directory_path)