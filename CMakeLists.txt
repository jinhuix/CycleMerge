cmake_minimum_required(VERSION 2.8)

set(CMAKE_C_STANDARD 99)

set(LIB_DIR ${PROJECT_SOURCE_DIR}/lib)
set(ALG_DIR ${PROJECT_SOURCE_DIR}/algorithm)

# 设置头文件目录变量
set(HEADER_DIRS
    ${LIB_DIR}
    ${ALG_DIR}
)

# 添加源文件变量
set(SOURCES
    ${PROJECT_SOURCE_DIR}/main.c
    ${ALG_DIR}/algorithm.c
    # ${ALG_DIR}/watchdog.c
    ${ALG_DIR}/hungarian.cpp
)

# 包含头文件目录
include_directories(${HEADER_DIRS})

# 生成可执行文件
add_executable(project_hw  ${SOURCES})

find_library(M_LIB m)
target_link_libraries(project_hw ${M_LIB})

target_link_libraries(project_hw PRIVATE ${PROJECT_SOURCE_DIR}/lib/libseek_model.so)

# 添加编译选项和宏定义
target_compile_options(project_hw PRIVATE -O3 -std=c99)
target_compile_definitions(project_hw PRIVATE _GNU_SOURCE)
# 设置CPP的优化O3
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11")

# 多线程
find_package(Threads REQUIRED)
target_link_libraries(project_hw PRIVATE ${CMAKE_THREAD_LIBS_INIT})