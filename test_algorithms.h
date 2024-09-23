#include <stdio.h>
#include <stdlib.h>
#include "public.h"

#define MAX_SCANS 100   // 最大扫描来回次数

/**
 * @brief  算法接口
 * @param  input            输入参数
 * @param  output           输出参数
 * @return int32_t          返回成功或者失败，RETURN_OK 或 RETURN_ERROR
 */
int32_t IOScheduleAlgorithm(const InputParam *input, OutputParam *output);

int32_t NearestNeighborAlgorithm(const InputParam *input, OutputParam *output);

int32_t SimulatedAnnealing(const InputParam *input, OutputParam *output);

int32_t TabuSearch(const InputParam *input, OutputParam *output);

int32_t HillClimbing(const InputParam *input, OutputParam *output);

int32_t GeneticAlgorithm(const InputParam *input, OutputParam *output);

int32_t MPSCAN(const InputParam *input, OutputParam *output);

int32_t AlgorithmRun(const InputParam *input, OutputParam *output, char *algorithm);

#ifdef _WIN32
#ifdef BUILDING_DLL
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT __declspec(dllimport)
#endif
#else
#define DLL_EXPORT
#endif

DLL_EXPORT int32_t partition_scan_t(const InputParam *input, OutputParam *output, int partition_len, int *partitions, int p_num);