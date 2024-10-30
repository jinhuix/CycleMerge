import re
import matplotlib.pyplot as plt
import os
import numpy as np

def parse_file(file_path):
    algorithms = []
    addressing_durations = []
    totcosts = []

    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()

    # 使用正则表达式解析文件内容
    algorithm_pattern = re.compile(r'/\* 算法名称 \*/\nAlgorithm: (.+?)\n/\* 关键指标结构体 \*/\nioCount: \d+ \ntotcost: (\d+) \nalgorithmRunningDuration\(ms\): [\d.]+ \nmemoryUse\(ms\): \d+ \naddressingDuration\(ms\): (\d+) \nreadDuration\(ms\): \d+\ntapeBeltWear\(times\): \d+\ntapeMotorWear\(times\): \d+ \nerrorIOCount: \d+ \n\n')
    matches = algorithm_pattern.findall(content)

    for match in matches:
        if(match[0] == 'FCFS'):
            continue
        if(match[0] == 'MPScanStar'):
            algorithms.append('MPScan*')
        else:
            algorithms.append(match[0])
        totcost=int(match[1])
        addressing_duration=int(match[2])
        # print(f'totcost:{totcost}, addressing_duration:{addressing_duration}\n')
        totcosts.append(totcost)
        addressing_durations.append(addressing_duration)

    return algorithms, addressing_durations, totcosts

def plot_addressing_durations(algorithms, addressing_durations, totcosts, output_path):
    plt.figure(figsize=(10, 6))
    
    # 使用颜色渐变来为每个柱赋予颜色
    # cmap = plt.get_cmap('viridis')
    cmap1 = plt.get_cmap('RdBu')
    colors1 = cmap1(np.linspace(0.2, 0.8, len(algorithms)))
    cmap2 = plt.get_cmap('coolwarm')
    colors2 = cmap2(np.linspace(0.2, 0.8, len(algorithms)))
    
    # 找出所有最小值的索引，并将这些索引对应的柱和其值标红
    min_duration = min(addressing_durations)
    min_indices1 = [i for i, duration in enumerate(addressing_durations) if duration == min_duration]
    
    min_totcost = min(totcosts)
    min_indices2 = [i for i, totcost in enumerate(totcosts) if totcost == min_totcost]
    
    bar_width = 0.35  # 设置柱的宽度
    index = np.arange(len(algorithms))  # 设置柱的位置
    
    
    # for i in min_indices:
    #     colors[i] = "red"
    
    # 绘制 addressing_durations 的柱状图
    bars1 = plt.bar(index, addressing_durations, bar_width, color=colors1, label='Addressing Durations')
    
    # 绘制 totcosts 的柱状图
    bars2 = plt.bar(index + bar_width, totcosts, bar_width, color=colors2, label='Total Costs')
    
    plt.xlabel('Algorithms')
    plt.ylabel('addressingDuration (ms)/totcosts')
    plt.title(os.path.basename(output_path)[:-4])
    plt.xticks(index + bar_width / 2, algorithms, rotation=45, ha='right')
    
    # 在每个柱的顶部标注其值
    for bar in bars1:
        yval = bar.get_height()
        color = 'red' if yval == min_duration else 'black'
        plt.text(bar.get_x() + bar.get_width()/2, yval + 1, int(yval), ha='center', va='bottom', color=color)
    
    for bar in bars2:
        yval = bar.get_height()
        color = 'orange' if yval == min_totcost else 'black'
        plt.text(bar.get_x() + bar.get_width()/2, yval + 1, int(yval), ha='center', va='bottom', color=color)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=500)
    plt.close()

def process_directory(directory_path):
    if not os.path.exists('./pic/bar'):
        os.makedirs('./pic/bar')

    for filename in os.listdir(directory_path):
        if filename.endswith('.txt'):
            print(f'processing {filename}')
            file_path = os.path.join(directory_path, filename)
            algorithms, addressing_durations, totcosts = parse_file(file_path)
            output_path = os.path.join('./pic/bar', f'{os.path.splitext(filename)[0]}.png')
            plot_addressing_durations(algorithms, addressing_durations, totcosts, output_path)
            print(f'Saved plot to {output_path}\n')

if __name__ == "__main__":
    directory_path = './result'  # 替换为你的文件夹路径
    process_directory(directory_path)