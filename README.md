## Introduction

The project was an entry for the 2nd University Student Information Storage Technology Competition organised by Huawei and won the Grand Prize. The official website of the competition: https://competition.huaweicloud.com/information/1000042106/introduction

Introduction to the theme of the competition: 

Tape media has a high capacity/price ratio and is widely used in archiving and data protection scenarios. Tape can only be a single concurrent read and write operations, any drop, random addressing or unreasonable tape access order will wear out the hardware life, resulting in tens of seconds/minutes of user delay, thus affecting the performance and life of the tape. In scenarios such as read/write correlation, tapes can utilize systematic techniques such as SSD caching and data scheduling to optimize lifetime and performance. However, in random read scenarios, tapes often use IO scheduling techniques to optimally sequence a batch of data accesses in order to significantly reduce seek latency and access wear. Magnetron disk IO scheduling techniques are typical OTSP (Open Loop Travelling Salesman Problem) optimal scheduling problems.

The main challenge of this problem is how to achieve the optimal scheduling of IO by considering the constraints of multiple variables such as the number of IO requests, IO request time, algorithm complexity, etc. and constructing and solving a combined optimization model with the optimization objective of minimizing the tape seek latency and wear and tear.



## Development Environment Requirements

Ubuntu  ≥   18.04
cmake   ≥   3.27.5
make    ≥   4.1
GCC     ≥   7.5



## Compile and Run

Compile:

```shell
cd project_hw
mkdir build
cd build
cmake ..
make
./project_hw -f /home/project_hw/dataset/case_1.txt
```

Running Example:

```shell
[root@kwepwebenv20531 project_hw]# mkdir build
[root@kwepwebenv20531 project_hw]# cd build/
[root@kwepwebenv20531 build]# cmake ..
[root@kwepwebenv20531 build]# make
Scanning dependencies of target project_hw
[ 33%] Building C object CMakeFiles/project_hw.dir/main.c.o
[ 66%] Building C object CMakeFiles/project_hw.dir/algorithm/algorithm.c.o
[100%] Linking C executable project_hw
[100%] Built target project_hw
[root@kwepwebenv20531 build]# 
[root@kwepwebenv20531 build]# ./project_hw -f /home/project_hw/dataset/case_4.txt
Welcome to HW project.
The file path is: /home/project_hw/dataset/case_4.txt
head info : [5,1000,0]

io count = 10
input io array:
io [1] : [1,4,100,150]
io [2] : [2,125,58,5]
io [3] : [3,9,90,30]
io [4] : [4,3,29,120]
io [5] : [5,6,500,1500]
io [6] : [6,4,100,150]
io [7] : [7,125,58,5]
io [8] : [8,9,90,30]
io [9] : [9,3,29,120]
io [10] : [10,6,500,1500]


Key Metrics:
    algorithmRunningDuration:    45.000000 ms
    addressingDuration:      123 ms
    readDuration:            456 ms
    tapeBeltWear:            200
    tapeMotorWear:           100
output sequence: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, ]
```