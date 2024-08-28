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

# 绘制图形
def plot_io_requests(io_data, output_path):
    plt.figure(figsize=(10, 6))
    for io_request in io_data:
        id, wrap, start_lpos, end_lpos = io_request
        color = 'blue' if wrap % 2 == 0 else 'red'
        plt.plot([min((start_lpos, end_lpos)), max((start_lpos, end_lpos))], [wrap, wrap], color, linewidth=2)
    
    plt.xlabel('Lpos')
    plt.ylabel('Wrap')
    plt.title('IO Requests: Lpos vs Wrap')
    plt.grid(True)
    
    # 保存图形到指定路径
    plt.savefig(output_path)
    plt.close()

# 处理单个文件
def process_file(file_path, output_dir):
    io_data = read_data_from_file(file_path)
    file_name = os.path.basename(file_path)
    output_file_name = file_name.replace('.txt', '.png')
    output_path = os.path.join(output_dir, output_file_name)
    plot_io_requests(io_data, output_path)

def get_io_img():
    dataset_dir = 'dataset'  # 需要处理的txt文件夹
    output_dir = 'dataimg'   # 输出图片文件夹

    # 创建输出文件夹，如果不存在
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # 遍历dataset文件夹下的所有txt文件
    for file_name in os.listdir(dataset_dir):
        if file_name.endswith('.txt'):
            file_path = os.path.join(dataset_dir, file_name)
            process_file(file_path, output_dir)
            print(f'Processed {file_name} and saved to {output_dir}')


# 主函数
def main():
    get_io_img()


# 运行主函数
if __name__ == "__main__":
    main()
