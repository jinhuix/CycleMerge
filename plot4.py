import os
import re
import matplotlib.pyplot as plt
import numpy as np

def parse_file(file_path):
    algorithms = []
    io_counts = []
    addressing_durations = []

    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()

    # 使用正则表达式解析文件内容
    algorithm_pattern = re.compile(r'/\* 算法名称 \*/\nAlgorithm: (.+?)\n/\* 关键指标结构体 \*/\nioCount: (\d+) \nalgorithmRunningDuration\(ms\): [\d.]+ \nmemoryUse\(ms\): \d+ \naddressingDuration\(ms\): (\d+) \nreadDuration\(ms\): \d+\ntapeBeltWear\(times\): \d+\ntapeMotorWear\(times\): \d+ \nerrorIOCount: \d+ \n\n')
    matches = algorithm_pattern.findall(content)

    for match in matches:
        if match[0] == 'FCFS':
            continue
        algorithms.append(match[0])
        io_counts.append(int(match[1]))
        addressing_durations.append(int(match[2]))

    return algorithms, io_counts, addressing_durations

def plot_experiment_variation(data, category, output_path):
    plt.figure(figsize=(10, 6))
    
    for algorithm in data:
        io_counts = data[algorithm]['io_counts']
        addressing_durations = data[algorithm]['addressing_durations']
        plt.plot(io_counts, addressing_durations, marker='o', label=algorithm)
    
    plt.xlabel('IO Count')
    plt.ylabel('Addressing Duration (ms)')
    plt.title(f'Experiment Variation for {category}')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(output_path, dpi=500)
    plt.close()

def process_directory(directory_path):
    categories = ['random', 'gaussian', 'mixed']
    data = {category: {} for category in categories}

    for filename in os.listdir(directory_path):
        if filename.endswith('.txt'):
            file_path = os.path.join(directory_path, filename)
            algorithms, io_counts, addressing_durations = parse_file(file_path)
            
            for category in categories:
                if category in filename:
                    for algorithm, io_count, addressing_duration in zip(algorithms, io_counts, addressing_durations):
                        if algorithm not in data[category]:
                            data[category][algorithm] = {'io_counts': [], 'addressing_durations': []}
                        data[category][algorithm]['io_counts'].append(io_count)
                        data[category][algorithm]['addressing_durations'].append(addressing_duration)
                    break

    if not os.path.exists('./pic/line'):
        os.makedirs('./pic/line/')

    for category in categories:
        output_path = os.path.join('./pic/line', f'{category}_experiment_variation.png')
        plot_experiment_variation(data[category], category, output_path)
        print(f'Saved plot to {output_path}')

if __name__ == "__main__":
    directory_path = './result'  # 替换为你的文件夹路径
    process_directory(directory_path)