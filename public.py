import ctypes
from ctypes import c_uint32, c_int32, POINTER, Structure, byref
import time
import numpy as np
import random

# 定义与C结构体对应的Python类
class HeadInfo(Structure):
    _fields_ = [("wrap", c_uint32),
                ("lpos", c_uint32),
                ("status", c_int32)]

class IOUint(Structure):
    _fields_ = [("id", c_uint32),
                ("wrap", c_uint32),
                ("startLpos", c_uint32),
                ("endLpos", c_uint32)]

class IOVector(Structure):
    _fields_ = [("len", c_uint32),
                ("ioArray", POINTER(IOUint))]

class InputParam(Structure):
    _fields_ = [("headInfo", HeadInfo),
                ("ioVec", IOVector)]

class OutputParam(Structure):
    _fields_ = [("len", c_uint32),
                ("sequence", POINTER(c_uint32))]

class AccessTime(Structure):
    _fields_ = [("addressDuration", c_uint32),
                ("readDuration", c_uint32)]

class TapeBeltSegWearInfo(Structure):
    _fields_ = [("segWear", c_uint32 * 730994)]

class KeyMetrics(Structure):
    _fields_ = [("ioCount", c_uint32),
                ("algorithmRunningDuration", ctypes.c_double),
                ("memoryUse", ctypes.c_long),
                ("addressingDuration", c_uint32),
                ("readDuration", c_uint32),
                ("tapeBeltWear", c_uint32),
                ("tapeMotorWear", c_uint32),
                ("errorIOCount", c_uint32)]

# 加载共享库
lib = ctypes.CDLL('./lib/libseek_model.so')

# 定义函数原型
lib.SeekTimeCalculate.argtypes = [POINTER(HeadInfo), POINTER(HeadInfo)]
lib.SeekTimeCalculate.restype = c_uint32

lib.BeltWearTimes.argtypes = [POINTER(HeadInfo), POINTER(HeadInfo), POINTER(TapeBeltSegWearInfo)]
lib.BeltWearTimes.restype = c_uint32

lib.MotorWearTimes.argtypes = [POINTER(HeadInfo), POINTER(HeadInfo)]
lib.MotorWearTimes.restype = c_uint32

lib.ReadTimeCalculate.argtypes = [c_uint32]
lib.ReadTimeCalculate.restype = c_uint32

lib.TotalAccessTime.argtypes = [POINTER(InputParam), POINTER(OutputParam), POINTER(AccessTime)]
lib.TotalAccessTime.restype = None

lib.TotalTapeBeltWearTimes.argtypes = [POINTER(InputParam), POINTER(OutputParam), POINTER(TapeBeltSegWearInfo)]
lib.TotalTapeBeltWearTimes.restype = c_uint32

lib.TotalMotorWearTimes.argtypes = [POINTER(InputParam), POINTER(OutputParam)]
lib.TotalMotorWearTimes.restype = c_uint32

# 读取文件并解析内容
def read_case_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    head_info = list(map(int, lines[1].strip()[1:-1].split(',')))
    io_count = int(lines[3].strip()[1:-1])
    io_list = [list(map(int, line.strip()[1:-1].split(','))) for line in lines[5:5+io_count]]
    
    return head_info, io_list


