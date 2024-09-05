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
from public import AccessTime, TapeBeltSegWearInfo, KeyMetrics, lib
from public import read_case_file
from public2 import lib2, read_input_param_from_file

max_lpos=730994

class Black_box():
    def __init__(self, case_file='./dataset/case_2.txt'):
        self.partition_len = 5000
        self.partition_number = (max_lpos+self.partition_len-1)//self.partition_len
        self.input_param = read_input_param_from_file(case_file)
        self.output_param = OutputParam()
        self.output_param.sequence = (c_uint * self.input_param.ioVec.len)()

    def get_cost_from_partitions(self, partitions):
        partition_len=self.partition_len
        p_num=len(partitions)
        partitions_cpp = (c_int * p_num)()
        for i in range(p_num):
            partitions_cpp[i]=partitions[i]
        cost=lib2.partition_scan_t(byref(self.input_param), byref(self.output_param), partition_len, partitions_cpp, p_num)
        # print(partitions)
        # print("Result:", cost)
        # print("Output sequence:", [self.output_param.sequence[i] for i in range(self.output_param.len)])
        # exit()
        return cost

class Bo_process():
    def __init__(self, black_box: Black_box = None , prf = True, max_runs = 30, random = False):
        self.black_box = black_box
        if self.black_box is None:
            self.black_box = Black_box()
        self.partition_number = self.black_box.partition_number
        space = sp.Space()
        space_config = []
        for i in range(self.partition_number - 1):
            space_config.append(sp.Categorical(f'x{i}', choices = [0, 1]))
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
                initial_runs = 8, 
                random_state = 999,
                advisor_type = 'random' if random else 'default',
            )
        self.history = None
        
    def obj_func(self, config: sp.Configuration):
        partitions =  self.get_partitions_from_config(config)
        cost = self.black_box.get_cost_from_partitions(partitions)
        logging.info(f'cost = {cost}')
        result = {
            'objectives': [cost, ],
        } 
        return result
    
    def get_partitions_from_config(self, config: sp.Configuration):
        partitions = []
        partition_size = 1
        for i in range(self.partition_number - 1):
            if config[f'x{i}'] == 0:
                partition_size += 1
            elif config[f'x{i}'] == 1:
                partitions.append(partition_size)
                partition_size = 1
            else:
                assert 1, "wrong value"
        partitions.append(partition_size)
        return partitions
    
    
    def run(self):
        self.history = self.opt.run()
        print(self.history)


black_box = Black_box()
BO = Bo_process(black_box)
BO.run()
