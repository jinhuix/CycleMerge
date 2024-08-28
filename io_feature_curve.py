import ctypes
from ctypes import c_uint32, c_int32, POINTER, Structure, byref
import time
import numpy as np
import os
import logging
import copy
logging.getLogger().setLevel(logging.INFO)
import sys
from collections import deque
from public import HeadInfo, IOUint, IOVector, InputParam, OutputParam
from public import AccessTime, TapeBeltSegWearInfo, KeyMetrics, SeekTimeCalculate
from public import read_case_file
from public import MAX_WRAP, MAX_LPOS, HEAD_STATUS,HEAD_RW

head_info, io_list = read_case_file('./dataset/case_new.txt')

# 创建输入参数
io_list = io_list
io_array = (IOUint * len(io_list))(*[IOUint(*io) for io in io_list])
io_vector = IOVector(len=len(io_list), ioArray=io_array)
input_param = input_param = InputParam(headInfo=HeadInfo(*head_info), ioVec=io_vector)
start = HeadInfo(wrap=input_param.headInfo.wrap, lpos=input_param.headInfo.lpos, status=input_param.headInfo.status)
seekT_dic = {}
for i in range(MAX_WRAP):
    for j in range(0):
        end = HeadInfo(wrap=i, lpos=j, status=HEAD_RW)
        seekT = SeekTimeCalculate(*start, *end)
        if i in seekT_dic:
            pass
        else:
            seekT_dic[i] = {}
        seekT_dic[i][j] = seekT
print(seekT_dic)
