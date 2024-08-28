import ctypes
from ctypes import c_uint32, c_int32, POINTER, Structure, byref
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

class Black_box():
    def __init__(self):
        # 初始化磁头状态和IO序列
        head_info, io_list = read_case_file('./dataset/case_4.txt')

        # 创建输入参数
        self.io_list = io_list
        io_array = (IOUint * len(io_list))(*[IOUint(*io) for io in io_list])
        io_vector = IOVector(len=len(io_list), ioArray=io_array)
        self.input_param = input_param = InputParam(headInfo=HeadInfo(*head_info), ioVec=io_vector)
        self.io_len = len(io_list)
        self.start = HeadInfo(wrap=input_param.headInfo.wrap, lpos=input_param.headInfo.lpos, status=input_param.headInfo.status)
        self.default_seq = [47, 39, 20, 1, 24, 27, 46, 38, 28, 50, 43, 3, 45, 9, 22, 14, 49, 12, 36, 16, 13, 29, 10, 15, 41, 42, 21, 37, 44, 7, 4, 33, 48, 34, 26, 23, 11, 40, 25, 30, 2, 6, 18, 31, 35, 32, 17, 19, 8, 5, ]


    def get_cost_from_seq(self, seq):
        # action_seq = [self.input_param.ioVec.ioArray[0]] * self.io_len
        # for io_id,seq_id in enumerate(seq):
        #     action_seq[seq_id] = self.input_param.ioVec.ioArray[io_id]
        action_seq = [s+1 for s in seq]
        output_param = OutputParam(len = self.io_len, sequence=(c_uint32 * self.io_len)(*action_seq))
        # 计算总时间和磨损
        access_time = AccessTime()  
        # 创建 TapeBeltSegWearInfo 实例
        seg_wear_info = TapeBeltSegWearInfo()
        lib.TotalAccessTime(byref(self.input_param), byref(output_param), byref(access_time))
        total_belt_wear_lib = lib.TotalTapeBeltWearTimes(byref(self.input_param), byref(output_param), byref(seg_wear_info))
        total_motor_wear_lib = lib.TotalMotorWearTimes(byref(self.input_param), byref(output_param))
        total_cost = access_time.addressDuration
        print([output_param.sequence[i] for i in range(output_param.len)] ,total_cost)
        return total_cost

class Bo_process():
    def __init__(self, io_len, black_box: Black_box = None , prf = True, max_runs = 50, random = False):
        params = {}
        self.io_len = io_len
        self.black_box = black_box
        # default_seq = [0] * self.io_len
        default_seq = [s for s in self.black_box.default_seq]
        for i in range(self.io_len):
            params[f'x{i}'] = (0, self.io_len-1, default_seq[i]-1)
        space = sp.Space()
        space_config = []
        for name, para in params.items():
            space_config.append(sp.Int(name, *para))
        if self.black_box is None:
            self.black_box = Black_box()
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
        seq =  self.get_seq_from_config(config)
        cost = self.black_box.get_cost_from_seq(seq)
        logging.info(f'cost = {cost}')
        result = {
            'objectives': [cost, ],
        } 
        return result
    
    def get_seq_from_config(self, config: sp.Configuration):
        vec = [config[f'x{i}'] for i in range(self.io_len)]
        # data_with_indices = [(value, index) for index, value in enumerate(vec)]
        # sorted_data_with_indices = sorted(data_with_indices, key=lambda x: x[0], reverse=True)
        # ranking = [0] * len(vec)
        # for rank, (_, index) in enumerate(sorted_data_with_indices, start=0):
        #     ranking[index] = rank
        ranking = vec
        return ranking
    
    
    def run(self):
        self.history = self.opt.run()
        print(self.history)


black_box = Black_box()
BO = Bo_process(io_len = black_box.io_len ,black_box = black_box)
BO.run()
