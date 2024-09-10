import os
import matplotlib.pyplot as plt

# 读取txt文件并解析数据
def read_data_from_file(file_path):
    io_data = []
    with open(file_path, 'r') as file:
        lines = file.readlines()
        io_section = False
        for line in lines:
            line = line.strip()
            if line == '["io":"id","wrap","startLpos","endLpos"]':
                io_section = True
                continue
            if io_section:
                if line.startswith('[') and line.endswith(']'):
                    # 去掉方括号并分割数据
                    values = line[1:-1].split(',')
                    io_data.append([int(values[0]), int(values[1]), int(values[2]), int(values[3])])
    return io_data

# 读取结果文件并解析数据
def read_result_from_file(result_path):
    algorithms = []
    print(result_path)
    with open(result_path, 'r') as file:
        lines = file.readlines()
        algorithm_name = None
        output_sequence = None
        for line in lines:
            line = line.strip()
            if line.startswith('Algorithm:'):
                algorithm_name = line.split(':')[1].strip()
            elif line.startswith('Output sequence:'):
                output_sequence = [int(x) for x in line.split(':')[1].strip().strip('[]').split(', ') if x]
                if algorithm_name and output_sequence:
                    algorithms.append((algorithm_name, output_sequence))
                    algorithm_name = None
                    output_sequence = None
    return algorithms


# 绘制图形
def plot_io_requests(io_map, seq, output_path):
    plt.figure(figsize=(10, 6))
    prev_end_lpos = None
    prev_wrap = None

    for id in seq:
        wrap, start_lpos, end_lpos = io_map[id]
        color = 'blue' if wrap % 2 == 0 else 'red'
        
        plt.plot([min((start_lpos, end_lpos)), max((start_lpos, end_lpos))], [wrap, wrap], color, linewidth=2)

        # 标注坐标
        # plt.text(wrap, start_lpos, f'({wrap}, {start_lpos})', fontsize=8, ha='right')
        # plt.text(wrap, end_lpos, f'({wrap}, {end_lpos})', fontsize=8, ha='right')

        # if prev_end_lpos is not None and prev_wrap is not None:
        #     plt.arrow(prev_end_lpos, prev_wrap, start_lpos - prev_end_lpos, wrap - prev_wrap,
        #               head_width=0.5, head_length=0.5, fc='green', ec='green')
        if prev_end_lpos is not None and prev_wrap is not None:
            plt.annotate('', xy=(start_lpos, wrap), xytext=(prev_end_lpos, prev_wrap),
                         arrowprops=dict(facecolor='green', edgecolor='green', arrowstyle='->', lw=0.5))

        prev_end_lpos = end_lpos
        prev_wrap = wrap

    print(f'Output saved to {output_path}')
    if(output_path=="dataimg/case_1_partition_scan.png"):
        partition_size = 30000
    elif(output_path=="dataimg/case_2_partition_scan.png"):
        partition_size = 95000
    elif(output_path=="dataimg/case_3_partition_scan.png"):
        partition_size = 100000
    elif(output_path=="dataimg/case_4_partition_scan.png"):
        partition_size = 25000
    elif(output_path=="dataimg/case_5_partition_scan.png"):
        partition_size = 0
    else:
        partition_size = 0
    if(partition_size!=0):
        print("partition_size:",partition_size)
        # 在图中每隔partition_size画一条竖直虚线
        for i in range(0, int(max([io[2] for io in io_map.values()])), partition_size):
            plt.axvline(x=i, color='r', linestyle='--', linewidth=0.5)


    plt.xlabel('Lpos')
    plt.ylabel('Wrap')
    plt.title('IO Requests: Lpos vs Wrap')
    plt.grid(True)

    # 保存图形到指定路径
    plt.savefig(output_path, dpi=1000)
    plt.close()


# 处理单个文件
def process_file(file_path, result_path, output_dir):
    io_data = read_data_from_file(file_path)
    # print(io_data)
    io_map={}
    for data in io_data:
        io_map[data[0]] = [data[1], data[2], data[3]]
        # io_map[data[0]] = {'wrap':data[1],'startLpos':data[2],'endLpos':data[3]}
    # print(io_map)
    
    file_name = os.path.basename(file_path)
    case_number = file_name.split('_')[1].split('.')[0]  # 提取case编号
    result_file = f'metrics_case{case_number}.txt'
    result_path = os.path.join(result_path, result_file)
    
    algorithms = read_result_from_file(result_path)
    
    # output_file_name = file_name.replace('.txt', '.png')
    for algorithm_name, output_sequence in algorithms:
        print(f'{algorithm_name} {output_sequence}')
        output_file_name = f'{file_name.replace(".txt", "")}_{algorithm_name}.png'
        output_path = os.path.join(output_dir, output_file_name)
        plot_io_requests(io_map, output_sequence, output_path)


def get_io_img():
    dataset_dir = 'dataset'  # 需要处理的txt文件夹
    output_dir = 'dataimg'   # 输出图片文件夹
    result_dir = 'result'    # 输出结果文件夹

    # 创建输出文件夹，如果不存在
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # 遍历dataset文件夹下的所有txt文件
    for file_name in os.listdir(dataset_dir):
        if file_name.endswith('.txt'):
            file_path = os.path.join(dataset_dir, file_name)
            process_file(file_path, result_dir, output_dir)
            print(f'Processed {file_name} and saved to {output_dir}')


# 主函数
def main():
    get_io_img()


# 运行主函数
if __name__ == "__main__":
    main()