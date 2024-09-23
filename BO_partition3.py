import ctypes
from ctypes import c_uint32, c_int, c_uint, c_int32, POINTER, Structure, byref
import time
import numpy as np
from openbox import space as sp, Optimizer
from ConfigSpace import Configuration
import math
import os
import logging
import copy
logging.getLogger().setLevel(logging.INFO)
import sys
from collections import deque
from public import HeadInfo, IOUint, IOVector, InputParam, OutputParam
from public2 import read_input_param_from_file
from collections import Counter
lib2 = ctypes.CDLL('./lib/libtest_algorithms.so')



# 定义函数原型
lib2.partition_scan_t.argtypes = [POINTER(InputParam), POINTER(OutputParam), c_int, POINTER(c_int), c_int]
lib2.partition_scan_t.restype = c_int32

lib2.p_scan_t.argtypes = [POINTER(InputParam), POINTER(OutputParam), c_int, POINTER(c_int), c_int, POINTER(c_int)]
lib2.p_scan_t.restype = c_int32

max_lpos=730994

def find_most_common_element(arr):
    # 使用 Counter 统计每个元素出现的次数
    count = Counter(arr)
    
    # 找出出现次数最多的元素
    most_common = count.most_common(1)
    
    # 返回出现次数最多的元素
    return most_common[0][0]

class Black_box():
    def __init__(self, case_file='./dataset/case_4.txt'):
        self.partition_len = 5000
        self.partition_number = (max_lpos+self.partition_len-1)//self.partition_len
        self.input_param = read_input_param_from_file(case_file)
        self.output_param = OutputParam()
        self.output_param.sequence = (c_uint * self.input_param.ioVec.len)()

    def get_cost_from_partitions(self, partition_size, scan_method):
        partition_len=self.partition_len
        p_num=(max_lpos+self.partition_len-1)//(self.partition_len*partition_size)
        partitions = []
        intact_partition_number = self.partition_number // partition_size
        remain_partition = self.partition_number % partition_size
        partitions = [partition_size] * intact_partition_number
        if remain_partition > 0:
            partitions.append(remain_partition)
        partitions_cpp = (c_int * len(partitions))()
        scan_method_cpp = (c_int * len(partitions))()
        for i in range(len(partitions)):
            partitions_cpp[i]=partitions[i]
            scan_method_cpp[i] = scan_method[i]
        cost=lib2.p_scan_t(byref(self.input_param), byref(self.output_param), partition_len, partitions_cpp, len(partitions), scan_method_cpp)
        # print(partitions)
        # print("Result:", cost)
        # print("Output sequence:", [self.output_param.sequence[i] for i in range(self.output_param.len)])
        # exit()
        return cost

class Bo_process():
    def __init__(self, black_box: Black_box = None , prf = True, max_runs = 50, random = False):
        self.black_box = black_box
        if self.black_box is None:
            self.black_box = Black_box()
        self.partition_number = self.black_box.partition_number
        space = sp.Space()
        space_config = []
        space_config.append(sp.Int('x', lower = 1, upper = self.partition_number, default_value = 1))
        for i in range(self.partition_number):
            space_config.append(sp.Categorical(f's_{i}', choices = [0, 1]))
        space.add_variables(space_config)
        self.opt = Optimizer(
                objective_function = self.obj_func,
                config_space = space,
                num_constraints = 0,
                num_objectives = 1,
                surrogate_type = 'prf' if prf else 'gp',
                acq_optimizer_type = 'auto',
                max_runs = max_runs,
                task_id = 'basic_search',
                logging_dir = 'logs_gray_clapboard_search',
                visualization = 'basic',
                initial_runs = 20, 
                random_state = 168,
                advisor_type = 'random' if random else 'default',
            )
        self.history = None
        
    def obj_func(self, config: sp.Configuration):
        partitions, scan_method =  self.get_partitions_from_config(config)
        cost = self.black_box.get_cost_from_partitions(partitions, scan_method)
        logging.info(f'cost = {cost}')
        result = {
            'objectives': [cost, ],
        } 
        return result
    
    def get_partitions_from_config(self, config: sp.Configuration):
        scan_method = []
        partition_size = 1 * config['x']
        # partition_size = 1
        for i in range(0, self.partition_number, partition_size):
            most_common_element = find_most_common_element([config[f's_{j}'] for j in range(i, min(i+20, self.partition_number))])
            scan_method.append(most_common_element)
        # for i in range(math.ceil(self.partition_number / partition_size)):
        #     scan_method.append(config[f's_{i}'])
        print(partition_size, math.ceil(self.partition_number / partition_size))
        return partition_size, scan_method
    
    
    def run(self):
        self.history = self.opt.run()
        print(self.history)


black_box = Black_box()
BO = Bo_process(black_box)
BO.run()
