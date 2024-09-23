# 使用 DBSCAN 算法对 IO 进行聚类，然后对每个聚类内的 IO 进行 scan
# 效果一般
# case1: 382058 ms
# case2: 296958 ms
# case3: 358004 ms
# case4: 704789 ms
# case5: 1036901 ms

from public import HeadInfo, IOUint, IOVector, InputParam, OutputParam
from public import AccessTime, TapeBeltSegWearInfo, KeyMetrics, lib
from public import read_case_file
from ctypes import c_uint32, c_int32, POINTER, Structure, byref
from sklearn.cluster import DBSCAN
from sklearn.neighbors import KDTree
import numpy as np
from ctypes import c_uint32,c_uint,c_int, c_int32, POINTER, Structure, byref

def calculate_distance_matrix(input_param):
    io_num = input_param.ioVec.len
    distance_matrix = np.zeros((io_num, io_num))
    for i in range(io_num):
        if(i%1000 == 0):
            print(f'Calculating distance matrix: {i}/{io_num}')
        for j in range(io_num):
            if i != j:
                head1 = HeadInfo(wrap=input_param.ioVec.ioArray[i].wrap, lpos=input_param.ioVec.ioArray[i].endLpos, status=1)
                head2 = HeadInfo(wrap=input_param.ioVec.ioArray[j].wrap, lpos=input_param.ioVec.ioArray[j].startLpos, status=1)
                distance_matrix[i][j] = lib.SeekTimeCalculate(byref(head1), byref(head2))
            else:
                distance_matrix[i][j] = 0
    return distance_matrix

def scan(points):
    forward = []
    backward = []
    for tape_pos in points:
        if tape_pos.wrap % 2 == 0:
            forward.append(tape_pos)
        else:
            backward.append(tape_pos)
    forward.sort(key=lambda x: x.startLpos)
    backward.sort(key=lambda x: x.startLpos, reverse=True)
    return forward + backward

def mpscan(points):
    forward = []
    backward = []
    for tape_pos in points:
        if tape_pos.wrap % 2 == 0:
            forward.append(tape_pos)
        else:
            backward.append(tape_pos)
    forward.sort(key=lambda x: x.startLpos)
    backward.sort(key=lambda x: x.startLpos, reverse=True)
    seq = []
    visited = {tape_pos.id: False for tape_pos in points}
    while len(seq) < len(points):
        last_io = 0
        for tape_pos in forward:
            if visited[tape_pos.id]:
                continue
            if last_io == 0 or not (tape_pos.wrap != last_io.wrap and tape_pos.startLpos//30 == last_io.startLpos//30):
                seq.append(tape_pos)
                visited[tape_pos.id] = True
                last_io = tape_pos
        last_io = 0
        for tape_pos in backward:
            if visited[tape_pos.id]:
                continue
            if last_io == 0 or not (tape_pos.wrap != last_io.wrap and tape_pos.startLpos//30 == last_io.startLpos//30):
                seq.append(tape_pos)
                visited[tape_pos.id] = True
                last_io = tape_pos

    return seq

def cluster(data):
    head_info, io_list = read_case_file(data)
    io_array = (IOUint * len(io_list))(*[IOUint(*io) for io in io_list])
    io_vector = IOVector(len=len(io_list), ioArray=io_array)
    input_param = InputParam(headInfo=HeadInfo(*head_info), ioVec=io_vector)

    distance_matrix = calculate_distance_matrix(input_param)
    # print("111\n")
    clustering = DBSCAN(eps=20000, min_samples=2, metric='precomputed').fit(distance_matrix)
    
    points = np.array([[i, j, distance_matrix[i][j]] for i in range(len(distance_matrix)) for j in range(len(distance_matrix[i])) if i != j])
    kdtree = KDTree(points[:, :2], metric='euclidean')
    clustering = DBSCAN(eps=20000, min_samples=2, metric='precomputed').fit(distance_matrix)
    
    # Output clustering results
    labels = clustering.labels_
    print("\nCluster labels: ", labels)
    
    # Group points by cluster
    clusters = {}
    cluster_cnt = 0
    for idx, label in enumerate(labels):
        if label not in clusters:
            clusters[label] = []
            cluster_cnt = max(cluster_cnt+1,label)
        clusters[label].append(idx+1)
    
    # for i in range(len(io_list)):
    #     print(f"IO {i}: {io_list[i]} {input_param.ioVec.ioArray[i].startLpos}")
    for cluster_id, points in clusters.items():
        print(f"Cluster {cluster_id}: {points}")
    
    # 将噪声点单独作为一个聚类
    noise_points = clusters.pop(-1, [])
    for idx in noise_points:
        clusters[cluster_cnt] = [idx]
        cluster_cnt+=1
    
    print("\nNoise points: ", noise_points)
    for cluster_id, points in clusters.items():
        print(f"Cluster {cluster_id}: {points}")
    
    # 计算每个聚类的平均 lpos
    cluster_lpos = {}
    for cluster_id, points in clusters.items():
        total_lpos = sum(input_param.ioVec.ioArray[point-1].startLpos for point in points)
        average_lpos = total_lpos / len(points)
        cluster_lpos[cluster_id] = average_lpos

    # 按平均 lpos 对聚类进行排序, item[0]为cluster_id, item[1]为average_lpos
    sorted_clusters = sorted(cluster_lpos.items(), key=lambda item: item[1])

    print("\nSorted clusters(by average startLpos): ", sorted_clusters)
    # 对每个聚类内部采取scan
    seq = []
    for cluster_id, average_lpos in sorted_clusters:
        # print(f"Cluster {cluster_id} average lpos: {average_lpos}")
        p_list = []
        for point in clusters[cluster_id]:
            p_list.append(input_param.ioVec.ioArray[point-1])
        seq.extend(scan(p_list))
    
    print("Sequence: ", [p.id for p in seq])
    output_param = OutputParam()
    output_param.sequence = (c_uint * len(seq))()  # 分配 c_uint 数组
    output_param.len = len(seq)
    for i in range(len(seq)):
        output_param.sequence[i] = seq[i].id
    
    # 计算总时间和磨损
    access_time = AccessTime()
    # 创建 TapeBeltSegWearInfo 实例
    seg_wear_info = TapeBeltSegWearInfo()

    lib.TotalAccessTime(byref(input_param), byref(output_param), byref(access_time))
    total_belt_wear_lib = lib.TotalTapeBeltWearTimes(byref(input_param), byref(output_param), byref(seg_wear_info))
    total_motor_wear_lib = lib.TotalMotorWearTimes(byref(input_param), byref(output_param))
    print(f"Total Addressing Duration (lib): {access_time.addressDuration} ms")
    print(f"Total Read Duration (lib): {access_time.readDuration} ms")
    print(f"Total Tape Belt Wear (lib): {total_belt_wear_lib} times")
    print(f"Total Motor Wear (lib): {total_motor_wear_lib} times")
    


# 示例数据
data = "./dataset/case_5.txt"

# data = "./dataset/gen/gen_case1_io10000.txt"
cluster(data)