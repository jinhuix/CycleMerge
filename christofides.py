# 之前找的一个开源的christofides实现，感觉实现的有点粗糙
# https://github.com/Retsediv/ChristofidesAlgorithm/blob/master/christofides.py
# 实际效果一般，case1~case5均劣于SCAN、Nearest
# 并且不稳定，每次跑结果都可能不一样
# 使用python无法处理10000规模的io请求，仅读取数据就会超时
# case1：355164ms
# case2：304643ms
# case3：402867ms
# case4：842728ms
# case5：1238103ms

from public import HeadInfo, IOUint, IOVector, InputParam, OutputParam
from public import AccessTime, TapeBeltSegWearInfo, KeyMetrics, lib
from public import read_case_file
from ctypes import c_uint32, c_int32, POINTER, Structure, byref

def tsp(data):
    head_info, io_list = read_case_file(data)
    io_array = (IOUint * len(io_list))(*[IOUint(*io) for io in io_list])
    io_vector = IOVector(len=len(io_list), ioArray=io_array)
    input_param = InputParam(headInfo=HeadInfo(*head_info), ioVec=io_vector)
    # build a graph
    G = build_graph(input_param)
    print("Graph: ", G)

    # build a minimum spanning tree
    MSTree = minimum_spanning_tree(G)
    print("MSTree: ", MSTree)

    # find odd vertexes
    odd_vertexes = find_odd_vertexes(MSTree)
    print("Odd vertexes in MSTree: ", odd_vertexes)

    # add minimum weight matching edges to MST
    minimum_weight_matching(MSTree, G, odd_vertexes)
    print("Minimum weight matching: ", MSTree)

    min_seek_time = 0x3f3f3f3f
    start_node=0
    for i in range(input_param.ioVec.len):
        head1=HeadInfo(wrap=input_param.headInfo.wrap,lpos=input_param.ioVec.ioArray[i].startLpos,status=1)
        seek_time=lib.SeekTimeCalculate(byref(input_param.headInfo),byref(head1))
        if seek_time<min_seek_time:
            min_seek_time=seek_time
            start_node=i
            # start_node=input_param.ioVec.ioArray[i].id
    print("Start node: ", start_node)
    
    # find an eulerian tour
    eulerian_tour = find_eulerian_tour(MSTree, G, start_node)

    print("Eulerian tour: ", eulerian_tour)
    sequence=[eulerian_tour[i]+1 for i in range(len(eulerian_tour)-1)]
    print("Sequence: ", sequence)
    # 创建输出参数
    output_param = OutputParam(len=len(io_list), sequence=(c_uint32 * len(io_list))(*range(1, len(io_list) + 1)))
    # output_param = OutputParam(len=len(io_list), sequence=(c_uint32 * len(io_list))(*sequence))
    for i in range(len(io_list)):
        output_param.sequence[i] = sequence[i]

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

    # current = eulerian_tour[0]
    # path = [current]
    # visited = [False] * len(eulerian_tour)
    # visited[eulerian_tour[0]] = True
    # length = 0

    # for v in eulerian_tour:
    #     if not visited[v]:
    #         path.append(v)
    #         visited[v] = True

    #         length += G[current][v]
    #         current = v

    # length +=G[current][eulerian_tour[0]]
    # path.append(eulerian_tour[0])

    # print("Result path: ", path)
    # print("Result length of the path: ", length)

    # return length, path


def get_length(x1, y1, x2, y2):
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** (1.0 / 2.0)


def build_graph(input_param):
    
    graph = {}
    io_num=input_param.ioVec.len
    for i in range(io_num):
        for j in range(io_num):
            if i!=j:
                if i not in graph:
                    graph[i]={}
                head1=HeadInfo(wrap=input_param.ioVec.ioArray[i].wrap,lpos=input_param.ioVec.ioArray[i].endLpos,status=1)
                head2=HeadInfo(wrap=input_param.ioVec.ioArray[j].wrap,lpos=input_param.ioVec.ioArray[j].startLpos,status=1)
                graph[i][j]=lib.SeekTimeCalculate(byref(head1), byref(head2))
    # for this in range(len(data)):
    #     for another_point in range(len(data)):
    #         if this != another_point:
    #             if this not in graph:
    #                 graph[this] = {}

    #             graph[this][another_point] = get_length(data[this][0], data[this][1], data[another_point][0],
    #                                                     data[another_point][1])

    return graph


class UnionFind:
    def __init__(self):
        self.weights = {}
        self.parents = {}

    def __getitem__(self, object):
        if object not in self.parents:
            self.parents[object] = object
            self.weights[object] = 1
            return object

        # find path of objects leading to the root
        path = [object]
        root = self.parents[object]
        while root != path[-1]:
            path.append(root)
            root = self.parents[root]

        # compress the path and return
        for ancestor in path:
            self.parents[ancestor] = root
        return root

    def __iter__(self):
        return iter(self.parents)

    def union(self, *objects):
        roots = [self[x] for x in objects]
        heaviest = max([(self.weights[r], r) for r in roots])[1]
        for r in roots:
            if r != heaviest:
                self.weights[heaviest] += self.weights[r]
                self.parents[r] = heaviest


def minimum_spanning_tree(G):
    tree = []
    subtrees = UnionFind()
    for W, u, v in sorted((G[u][v], u, v) for u in G for v in G[u]):
        if subtrees[u] != subtrees[v]:
            tree.append((u, v, W))
            subtrees.union(u, v)

    return tree


def find_odd_vertexes(MST):
    tmp_g = {}
    vertexes = []
    for edge in MST:
        if edge[0] not in tmp_g:
            tmp_g[edge[0]] = 0

        if edge[1] not in tmp_g:
            tmp_g[edge[1]] = 0

        tmp_g[edge[0]] += 1
        tmp_g[edge[1]] += 1

    for vertex in tmp_g:
        if tmp_g[vertex] % 2 == 1:
            vertexes.append(vertex)

    return vertexes


def minimum_weight_matching(MST, G, odd_vert):
    import random
    # random.shuffle(odd_vert)

    while odd_vert:
        v = odd_vert.pop()
        length = float("inf")
        u = 1
        closest = 0
        for u in odd_vert:
            if v != u and G[v][u] < length:
                length = G[v][u]
                closest = u

        MST.append((v, closest, length))
        odd_vert.remove(closest)


def find_eulerian_tour(MatchedMSTree, G, start=None):
    # find neigbours
    neighbours = {}
    for edge in MatchedMSTree:
        if edge[0] not in neighbours:
            neighbours[edge[0]] = []

        if edge[1] not in neighbours:
            neighbours[edge[1]] = []

        neighbours[edge[0]].append(edge[1])
        neighbours[edge[1]].append(edge[0])

    # print("Neighbours: ", neighbours)

    # finds the hamiltonian circuit
    start_vertex = MatchedMSTree[0][0]
    # start_vertex = start
    # EP = [neighbours[start_vertex][0]]
    EP = [start]
    print("EP: ", EP)

    while len(MatchedMSTree) > 0:
        for i, v in enumerate(EP):
            if len(neighbours[v]) > 0:
                break

        while len(neighbours[v]) > 0:
            w = neighbours[v][0]

            remove_edge_from_matchedMST(MatchedMSTree, v, w)

            del neighbours[v][(neighbours[v].index(w))]
            del neighbours[w][(neighbours[w].index(v))]

            i += 1
            EP.insert(i, w)

            v = w

    return EP


def remove_edge_from_matchedMST(MatchedMST, v1, v2):

    for i, item in enumerate(MatchedMST):
        if (item[0] == v2 and item[1] == v1) or (item[0] == v1 and item[1] == v2):
            del MatchedMST[i]

    return MatchedMST


# tsp([[1380, 939], [2848, 96], [3510, 1671], [457, 334], [3888, 666], [984, 965], [2721, 1482], [1286, 525],
#     [2716, 1432], [738, 1325], [1251, 1832], [2728, 1698], [3815, 169], [3683, 1533], [1247, 1945], [123, 862],
#     [1234, 1946], [252, 1240], [611, 673], [2576, 1676], [928, 1700], [53, 857], [1807, 1711], [274, 1420],
#     [2574, 946], [178, 24], [2678, 1825], [1795, 962], [3384, 1498], [3520, 1079], [1256, 61], [1424, 1728],
#     [3913, 192], [3085, 1528], [2573, 1969], [463, 1670], [3875, 598], [298, 1513], [3479, 821], [2542, 236],
#     [3955, 1743], [1323, 280], [3447, 1830], [2936, 337], [1621, 1830], [3373, 1646], [1393, 1368],
#     [3874, 1318], [938, 955], [3022, 474], [2482, 1183], [3854, 923], [376, 825], [2519, 135], [2945, 1622],
#     [953, 268], [2628, 1479], [2097, 981], [890, 1846], [2139, 1806], [2421, 1007], [2290, 1810], [1115, 1052],
#     [2588, 302], [327, 265], [241, 341], [1917, 687], [2991, 792], [2573, 599], [19, 674], [3911, 1673],
#     [872, 1559], [2863, 558], [929, 1766], [839, 620], [3893, 102], [2178, 1619], [3822, 899], [378, 1048],
#     [1178, 100], [2599, 901], [3416, 143], [2961, 1605], [611, 1384], [3113, 885], [2597, 1830], [2586, 1286],
#     [161, 906], [1429, 134], [742, 1025], [1625, 1651], [1187, 706], [1787, 1009], [22, 987], [3640, 43],
#     [3756, 882], [776, 392], [1724, 1642], [198, 1810], [3950, 1558]])

tsp("./dataset/case_1.txt")
# tsp("./dataset/gen/gen_case1_io10000.txt")

# tsp([[1, 1], [2, 5], [8, 0]])

#
# tsp([
#     [0, 0],
#     [3, 0],
#     [6, 0],
#
#     [0, 3],
#     [3, 3],
#     [6, 3],
#
#     [0, 6],
#     [3, 6],
#     [6, 6],
#
# ])