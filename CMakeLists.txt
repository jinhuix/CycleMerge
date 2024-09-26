cmake_minimum_required(VERSION 2.8)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")

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
)

set(TEST
    ${PROJECT_SOURCE_DIR}/test.c
    ${PROJECT_SOURCE_DIR}/logging.c
)

set(TEST2
    ${PROJECT_SOURCE_DIR}/test2.c
    ${PROJECT_SOURCE_DIR}/logging.c
    ${PROJECT_SOURCE_DIR}/test_algorithms.c
    ${PROJECT_SOURCE_DIR}/interface.c
    ${PROJECT_SOURCE_DIR}/operator_optimization.c

)

set(test_algorithms
    ${PROJECT_SOURCE_DIR}/logging.c
    ${PROJECT_SOURCE_DIR}/test_algorithms.h
    ${PROJECT_SOURCE_DIR}/test_algorithms.c
)

set(DATA_GEN
    ${PROJECT_SOURCE_DIR}/data_gen.cpp
    ${PROJECT_SOURCE_DIR}/logging.c
)

# 包含头文件目录
include_directories(${HEADER_DIRS})

# 生成可执行文件
add_executable(project_hw  ${SOURCES})
target_link_libraries(project_hw PRIVATE ${PROJECT_SOURCE_DIR}/lib/libseek_model.so)

add_executable(test  ${PROJECT_SOURCE_DIR}/test.c ${PROJECT_SOURCE_DIR}/logging.c)
target_link_libraries(test PRIVATE ${PROJECT_SOURCE_DIR}/lib/libseek_model.so)

add_executable(test2  ${TEST2})
target_link_libraries(test2 PRIVATE ${PROJECT_SOURCE_DIR}/lib/libseek_model.so)

add_library(test_algorithms SHARED ${test_algorithms})
target_link_libraries(test_algorithms PRIVATE ${PROJECT_SOURCE_DIR}/lib/libseek_model.so)

add_executable(data_gen  ${DATA_GEN})

# 添加编译选项和宏定义
target_compile_options(project_hw PRIVATE -g -std=c99)
target_compile_definitions(project_hw PRIVATE _GNU_SOURCE)

# 查找数学库
find_library(M_LIB m)

# 将数学库链接到你的目标
target_link_libraries(project_hw ${M_LIB})
target_link_libraries(test ${M_LIB})
target_link_libraries(test2 ${M_LIB})
