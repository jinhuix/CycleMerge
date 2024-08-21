import ctypes
from ctypes import c_uint32, c_int32, POINTER, Structure, byref
import time
from public import HeadInfo, IOUint, IOVector, InputParam, OutputParam
from public import AccessTime, TapeBeltSegWearInfo, KeyMetrics, lib
from public import read_case_file


# 初始化磁头状态和IO序列
head_info, io_list = read_case_file('./dataset/case_4.txt')

# 创建输入参数
io_array = (IOUint * len(io_list))(*[IOUint(*io) for io in io_list])
io_vector = IOVector(len=len(io_list), ioArray=io_array)
input_param = InputParam(headInfo=HeadInfo(*head_info), ioVec=io_vector)

# 创建输出参数
output_param = OutputParam(len=len(io_list), sequence=(c_uint32 * len(io_list))(*range(1, len(io_list) + 1)))

# 初始化磁头状态
start = HeadInfo(wrap=input_param.headInfo.wrap, lpos=input_param.headInfo.lpos, status=input_param.headInfo.status)

# 初始化总时间和磨损
total_seek_time = 0
total_read_time = 0
total_belt_wear = 0
total_motor_wear = 0

# 计算各个seek和read操作的时间和磨损
for i in range(output_param.len):
    curIO = output_param.sequence[i] - 1
    end = HeadInfo(wrap=input_param.ioVec.ioArray[curIO].wrap, lpos=input_param.ioVec.ioArray[curIO].startLpos, status=1)
    seek_time = lib.SeekTimeCalculate(byref(start), byref(end))
    belt_wear = lib.BeltWearTimes(byref(start), byref(end), None)
    motor_wear = lib.MotorWearTimes(byref(start), byref(end))
    total_seek_time += seek_time
    total_belt_wear += belt_wear
    total_motor_wear += motor_wear
    print(f"Seek from (wrap: {start.wrap}, lpos: {start.lpos}) to (wrap: {end.wrap}, lpos: {end.lpos})")
    print(f"Seek Time: {seek_time} ms, Belt Wear: {belt_wear} lpos, Motor Wear: {motor_wear} lpos")

    start.wrap = input_param.ioVec.ioArray[curIO].wrap
    start.lpos = input_param.ioVec.ioArray[curIO].startLpos
    start.status = 1
    end.wrap = input_param.ioVec.ioArray[curIO].wrap
    end.lpos = input_param.ioVec.ioArray[curIO].endLpos
    end.status = 1
    read_time = lib.ReadTimeCalculate(abs(start.lpos - end.lpos))
    belt_wear = lib.BeltWearTimes(byref(start), byref(end), None)
    motor_wear = lib.MotorWearTimes(byref(start), byref(end))
    total_read_time += read_time
    total_belt_wear += belt_wear
    total_motor_wear += motor_wear
    print(f"Read from (wrap: {start.wrap}, lpos: {start.lpos}) to (wrap: {end.wrap}, lpos: {end.lpos})")
    print(f"Read Time: {read_time} ms, Belt Wear: {belt_wear} lpos, Motor Wear: {motor_wear} lpos\n")
    start = end

# 计算总时间和磨损
access_time = AccessTime()
# 创建 TapeBeltSegWearInfo 实例
seg_wear_info = TapeBeltSegWearInfo()

lib.TotalAccessTime(byref(input_param), byref(output_param), byref(access_time))
total_belt_wear_lib = lib.TotalTapeBeltWearTimes(byref(input_param), byref(output_param), byref(seg_wear_info))
total_motor_wear_lib = lib.TotalMotorWearTimes(byref(input_param), byref(output_param))

print(f"Total Seek Time: {total_seek_time} ms")
print(f"Total Read Time: {total_read_time} ms")
print(f"Total Belt Wear: {total_belt_wear} lpos")
print(f"Total Motor Wear: {total_motor_wear} lpos\n")

print(f"Total Addressing Duration (lib): {access_time.addressDuration} ms")
print(f"Total Read Duration (lib): {access_time.readDuration} ms")
print(f"Total Tape Belt Wear (lib): {total_belt_wear_lib} times")
print(f"Total Motor Wear (lib): {total_motor_wear_lib} times")