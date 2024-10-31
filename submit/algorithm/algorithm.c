#include "algorithm.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#define MAX_TRIALS 200       // 最大尝试次数
#define NUM_EMPLOYED_BEES 40 // 雇佣蜂的数量
#define NUM_ONLOOKER_BEES 20 // 跟随蜂的数量
#define MAX_ITERATIONS 1000  // 最大迭代次数

void startRecordTime()
{
    gettimeofday(&g_TimeRecord.start, NULL);
}

int32_t getDurationMicroseconds()
{
    struct timeval end;
    gettimeofday(&end, NULL);
    return (end.tv_sec - g_TimeRecord.start.tv_sec) * 1000000 + end.tv_usec - g_TimeRecord.start.tv_usec;
}

// 计算总代价：寻址时长 + 带体磨损 + 电机磨损
uint32_t getTotalCost(const InputParam *input, OutputParam *output)
{
    AccessTime accessTime = {0};
    TotalAccessTime(input, output, &accessTime);                                    // 寻址时长
    uint32_t tapeBeltWear = TotalTapeBeltWearTimes(input, output, NULL);            // 带体磨损
    uint32_t tapeMotorWear = TotalMotorWearTimes(input, output);                    // 电机磨损
    uint32_t totalCost = accessTime.addressDuration + tapeBeltWear + tapeMotorWear; // 总代价
    return totalCost;
}

// 计算单次代价：寻址时长 + 带体磨损 + 电机磨损
uint32_t getCost(const HeadInfo *start, const HeadInfo *target)
{
    uint32_t seekT = SeekTimeCalculate(start, target);
    uint32_t beltW = BeltWearTimes(start, target, NULL);
    uint32_t motorW = MotorWearTimes(start, target);
    uint32_t cost = seekT + beltW + motorW;
    return cost;
}

/**
 * @brief  算法接口
 * @param  input            输入参数
 * @param  output           输出参数
 * @return int32_t          返回成功或者失败，RETURN_OK 或 RETURN_ERROR
 */
int32_t IOScheduleAlgorithm(const InputParam *input, OutputParam *output)
{
    uint32_t min_cost = 0xFFFFFFFF;
    uint32_t total_cost = 0xFFFFFFFF;
    int *best_sequence = (int *)malloc(input->ioVec.len * sizeof(int));
    if (input->ioVec.len > 1000)
    {
        int flag = 1;
        AccessTime accessTime = {0};
        merge(input, output);
        total_cost = getTotalCost(input, output);
        if (total_cost < min_cost)
        {
            min_cost = total_cost;
            memcpy(best_sequence, output->sequence, input->ioVec.len * sizeof(int));
            flag = 1;
        }
        merge_random(input, output);
        total_cost = getTotalCost(input, output);
        if (total_cost < min_cost)
        {
            min_cost = total_cost;
            memcpy(best_sequence, output->sequence, input->ioVec.len * sizeof(int));
            flag = 2;
        }
        memcpy(output->sequence, best_sequence, input->ioVec.len * sizeof(int));
        printf("flag=%d\n", flag);
    }
    else
    {
        int flag = 3;
        partition_scan(input, output); // 在算法内部还是只考虑寻址时长
        AccessTime accessTime = {0};
        total_cost = getTotalCost(input, output);
        if (total_cost < min_cost)
        {
            min_cost = total_cost;
            memcpy(best_sequence, output->sequence, input->ioVec.len * sizeof(int));
            flag = 3;
        }
        MPScan(input, output); // 在算法内部还是只考虑寻址时长
        total_cost = getTotalCost(input, output);
        if (total_cost < min_cost)
        {
            min_cost = total_cost;
            memcpy(best_sequence, output->sequence, input->ioVec.len * sizeof(int));
            flag = 4;
        }
        merge(input, output);
        total_cost = getTotalCost(input, output);
        if (total_cost < min_cost)
        {
            min_cost = total_cost;
            memcpy(best_sequence, output->sequence, input->ioVec.len * sizeof(int));
            flag = 5;
        }
        merge_random(input, output);
        total_cost = getTotalCost(input, output);
        if (total_cost < min_cost)
        {
            min_cost = total_cost;
            memcpy(best_sequence, output->sequence, input->ioVec.len * sizeof(int));
            flag = 6;
        }
        memcpy(output->sequence, best_sequence, input->ioVec.len * sizeof(int));
        printf("flag=%d\n", flag);
    }

    free(best_sequence);
    return 0;
}

/**
 * @brief  算法运行的主入口
 * @param  input            输入参数
 * @param  output           输出参数
 * @return uint32_t          返回成功或者失败，RETURN_OK 或 RETURN_ERROR
 */
int32_t AlgorithmRun(const InputParam *input, OutputParam *output)
{
    int32_t ret, duration_us;

    startRecordTime();

    ret = IOScheduleAlgorithm(input, output);
    printf("cost before operator optimization: %ld\n", getTotalCost(input, output));
    ret = operator_optimization(input, output);
    printf("cost afther operator optimization: %ld\n", getTotalCost(input, output));
    duration_us = getDurationMicroseconds();

    // printf("duration_us=%d\n", duration_us);

    return RETURN_OK;
}

int32_t operator_optimization(const InputParam *input, OutputParam *output)
{
    // SimpleOperatorOptimization(input, output);

    ArtificialBeeColony(input, output);
    return RETURN_OK;
}

int32_t SimpleOperatorOptimization(const InputParam *input, OutputParam *output)
{
    int32_t ret;

    int maxIterations = 1000000;

    uint32_t currentCost = getTotalCost(input, output); // 总代价
    uint32_t bestCost = currentCost;
    uint32_t bestSequence[input->ioVec.len];

    // 当前求到的初始解
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
        bestSequence[i] = output->sequence[i];

    // 用于存放邻域解
    OutputParam *temp_output;
    temp_output = (OutputParam *)malloc(sizeof(OutputParam));
    temp_output->len = input->ioVec.len;
    temp_output->sequence = (uint32_t *)malloc(input->ioVec.len * sizeof(uint32_t));

    uint32_t iteration = 0;
    uint32_t flag = 50000;

    while (iteration < flag)
    {
        // 生成邻域解
        for (uint32_t i = 0; i < input->ioVec.len; ++i)
            temp_output->sequence[i] = output->sequence[i];
        {
            // 将一个点插入另一个位置
            uint32_t from = rand() % input->ioVec.len;
            uint32_t to;
            to = rand() % input->ioVec.len;
            uint32_t temp = temp_output->sequence[from];
            if (from < to)
                for (uint32_t i = from; i < to; ++i)
                    temp_output->sequence[i] = temp_output->sequence[i + 1];
            else
                for (uint32_t i = from; i > to; --i)
                    temp_output->sequence[i] = temp_output->sequence[i - 1];
            temp_output->sequence[to] = temp;
        }

        // 计算邻域解的总时延
        uint32_t neighborCost = getTotalCost(input, temp_output);
        // 接受邻域解的条件，比当前解好或温度条件满足接受一个更劣的解
        if (neighborCost < currentCost)
        {
            printf("%ld iter, one operator optimization cost: %ld\n", iteration, neighborCost);
            // 接受邻域解
            for (uint32_t i = 0; i < input->ioVec.len; ++i)
                output->sequence[i] = temp_output->sequence[i];

            currentCost = neighborCost;

            // 若当前解比最优解好，则更新最优解
            if (currentCost < bestCost)
            {
                for (uint32_t i = 0; i < input->ioVec.len; ++i)
                    bestSequence[i] = output->sequence[i];
                bestCost = currentCost;
            }
        }

        ++iteration;
    }

    // 将最优解复制到输出参数
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++)
        output->sequence[i] = bestSequence[i];

    printf("accessTime after search:%d\n", bestCost);

    free(temp_output->sequence);
    free(temp_output);

    return RETURN_OK;
    return RETURN_OK;
}

int32_t SimulatedAnnealingRandom(const InputParam *input, OutputParam *output)
{
    int32_t ret;

    // 初始化参数
    double initialTemperature = 5000;
    double coolingRate = 0.995;
    double minTemperature = 1;
    int maxIterations = 10000000;

    uint32_t currentCost = getTotalCost(input, output); // 总代价
    uint32_t bestCost = currentCost;
    uint32_t bestSequence[input->ioVec.len];

    // 当前求到的初始解
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
        bestSequence[i] = output->sequence[i];

    // 用于存放邻域解
    OutputParam *temp_output;
    temp_output = (OutputParam *)malloc(sizeof(OutputParam));
    temp_output->len = input->ioVec.len;
    temp_output->sequence = (uint32_t *)malloc(input->ioVec.len * sizeof(uint32_t));

    // 模拟退火过程
    double temperature = initialTemperature;
    uint32_t iteration = 0;
    uint32_t flag = 100000;

    while (temperature > minTemperature && iteration < maxIterations)
    {
        // 生成邻域解
        for (uint32_t i = 0; i < input->ioVec.len; ++i)
            temp_output->sequence[i] = output->sequence[i];

        // double randValue = (double)rand() / RAND_MAX;
        // if (randValue < 0.33)
        // {
        //     // 交换两个点
        //     uint32_t i = rand() % input->ioVec.len;
        //     uint32_t j = rand() % input->ioVec.len;
        //     uint32_t temp = temp_output->sequence[i];
        //     temp_output->sequence[i] = temp_output->sequence[j];
        //     temp_output->sequence[j] = temp;
        // }
        // else if (randValue < 0.66)
        // {
        //     // 反转一段区间
        //     uint32_t start = rand() % input->ioVec.len;
        //     uint32_t end = rand() % input->ioVec.len;
        //     if (start > end)
        //     {
        //         uint32_t temp = start;
        //         start = end;
        //         end = temp;
        //     }
        //     while (start < end)
        //     {
        //         uint32_t temp = temp_output->sequence[start];
        //         temp_output->sequence[start] = temp_output->sequence[end];
        //         temp_output->sequence[end] = temp;
        //         start++;
        //         end--;
        //     }
        // }
        // else
        {
            // 将一个点插入另一个位置
            uint32_t from = rand() % input->ioVec.len;
            uint32_t to;
            to = rand() % input->ioVec.len;
            uint32_t temp = temp_output->sequence[from];
            if (from < to)
                for (uint32_t i = from; i < to; ++i)
                    temp_output->sequence[i] = temp_output->sequence[i + 1];
            else
                for (uint32_t i = from; i > to; --i)
                    temp_output->sequence[i] = temp_output->sequence[i - 1];
            temp_output->sequence[to] = temp;
        }

        // 计算邻域解的总时延
        uint32_t neighborCost = getTotalCost(input, temp_output);
        // 接受邻域解的条件，比当前解好或温度条件满足接受一个更劣的解
        if (neighborCost < currentCost || (iteration >= flag && ((double)rand() / RAND_MAX) < exp((currentCost - neighborCost) / temperature)))
        {
            // printf("%ld iter, one operator optimization cost: %ld\n", iteration, neighborCost);
            // 接受邻域解
            for (uint32_t i = 0; i < input->ioVec.len; ++i)
                output->sequence[i] = temp_output->sequence[i];

            currentCost = neighborCost;

            // 若当前解比最优解好，则更新最优解
            if (currentCost < bestCost)
            {
                if (currentCost * 1.0001 < bestCost)
                    flag = 0;
                for (uint32_t i = 0; i < input->ioVec.len; ++i)
                    bestSequence[i] = output->sequence[i];
                bestCost = currentCost;
            }
        }
        if (iteration >= flag)
        {
            // 降低温度
            temperature *= coolingRate;
        }
        ++iteration;
    }

    // 将最优解复制到输出参数
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++)
        output->sequence[i] = bestSequence[i];

    printf("accessTime after search:%d\n", bestCost);

    free(temp_output->sequence);
    free(temp_output);

    return RETURN_OK;
}

int32_t SimulatedAnnealing(const InputParam *input, OutputParam *output)
{
    int32_t ret;

    // 初始化参数
    double initialTemperature = 5000;
    double coolingRate = 0.995;
    double minTemperature = 1;
    int maxIterations = 10000000;

    uint32_t currentCost = getTotalCost(input, output); // 总代价
    uint32_t bestCost = currentCost;
    uint32_t bestSequence[input->ioVec.len];

    // 当前求到的初始解
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
        bestSequence[i] = output->sequence[i];

    // 用于存放邻域解
    OutputParam *temp_output;
    temp_output = (OutputParam *)malloc(sizeof(OutputParam));
    temp_output->len = input->ioVec.len;
    temp_output->sequence = (uint32_t *)malloc(input->ioVec.len * sizeof(uint32_t));

    // 模拟退火过程
    double temperature = initialTemperature;
    uint32_t iteration = 0;

    while (temperature > minTemperature && iteration < maxIterations)
    {
        // 生成邻域解
        for (uint32_t i = 0; i < input->ioVec.len; ++i)
            temp_output->sequence[i] = output->sequence[i];

        double randValue = (double)rand() / RAND_MAX;
        if (randValue < 0.33)
        {
            // 交换两个点
            uint32_t i = rand() % input->ioVec.len;
            uint32_t j = rand() % input->ioVec.len;
            uint32_t temp = temp_output->sequence[i];
            temp_output->sequence[i] = temp_output->sequence[j];
            temp_output->sequence[j] = temp;
        }
        else if (randValue < 0.66)
        {
            // 反转一段区间
            uint32_t start = rand() % input->ioVec.len;
            uint32_t end = rand() % input->ioVec.len;
            if (start > end)
            {
                uint32_t temp = start;
                start = end;
                end = temp;
            }
            while (start < end)
            {
                uint32_t temp = temp_output->sequence[start];
                temp_output->sequence[start] = temp_output->sequence[end];
                temp_output->sequence[end] = temp;
                start++;
                end--;
            }
        }
        else
        {
            // 将一个点插入另一个位置
            uint32_t from = rand() % input->ioVec.len;
            uint32_t to;
            to = rand() % input->ioVec.len;
            uint32_t temp = temp_output->sequence[from];
            if (from < to)
                for (uint32_t i = from; i < to; ++i)
                    temp_output->sequence[i] = temp_output->sequence[i + 1];
            else
                for (uint32_t i = from; i > to; --i)
                    temp_output->sequence[i] = temp_output->sequence[i - 1];
            temp_output->sequence[to] = temp;
        }

        // 计算邻域解的总时延
        uint32_t neighborCost = getTotalCost(input, temp_output);
        // 接受邻域解的条件，比当前解好或温度条件满足接受一个更劣的解
        double deltaCost = (double)(currentCost - neighborCost);
        if (neighborCost < currentCost || ((double)rand() / RAND_MAX) < exp(deltaCost / temperature))
        {
            // printf("%ld iter, one operator optimization cost: %ld\n", iteration, neighborCost);
            // 接受邻域解
            for (uint32_t i = 0; i < input->ioVec.len; ++i)
                output->sequence[i] = temp_output->sequence[i];

            currentCost = neighborCost;

            // 若当前解比最优解好，则更新最优解
            if (currentCost < bestCost)
            {
                for (uint32_t i = 0; i < input->ioVec.len; ++i)
                    bestSequence[i] = output->sequence[i];
                bestCost = currentCost;
            }
        }

        // 降低温度
        temperature *= coolingRate;
        ++iteration;
    }

    // 将最优解复制到输出参数
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++)
        output->sequence[i] = bestSequence[i];

    printf("accessTime after search:%d\n", bestCost);

    free(temp_output->sequence);
    free(temp_output);

    return RETURN_OK;
}

// 模拟蜂群
//  复制解
void copySolution(uint32_t *dest, const uint32_t *src, uint32_t len)
{
    for (uint32_t i = 0; i < len; ++i)
    {
        dest[i] = src[i];
    }
}

// 生成随机解
void generateRandomSolution(OutputParam *solution, uint32_t len)
{
    uint32_t i = rand() % len;
    uint32_t j = rand() % len;
    double randValue = (double)rand() / RAND_MAX;
    if (randValue < 0.33)
    {
        // 随机交换两个随机位置的元素
        uint32_t temp = solution->sequence[i];
        solution->sequence[i] = solution->sequence[j];
        solution->sequence[j] = temp;
    }
    else if (randValue < 0.66)
    {
        if (i > j)
        {
            uint32_t temp = i;
            i = j;
            j = temp;
        }
        while (i < j)
        {
            uint32_t temp = solution->sequence[i];
            solution->sequence[i] = solution->sequence[j];
            solution->sequence[j] = temp;
            i++;
            j--;
        }
    }
    else
    {
        uint32_t temp = solution->sequence[i];
        if (i < j)
            for (uint32_t k = i; k < j; ++k)
                solution->sequence[k] = solution->sequence[k + 1];
        else
            for (uint32_t k = i; k > j; --k)
                solution->sequence[k] = solution->sequence[k - 1];
        solution->sequence[j] = temp;
    }
}

// 生成邻域解（随机插入元素）
void generateNeighborSolution(OutputParam *neighbor, const OutputParam *current, uint32_t len)
{
    copySolution(neighbor->sequence, current->sequence, len);
    uint32_t i = rand() % len;
    uint32_t j = rand() % len;
    uint32_t temp = neighbor->sequence[i];
    if (i < j)
        for (uint32_t k = i; k < j; ++k)
            neighbor->sequence[k] = neighbor->sequence[k + 1];
    else
        for (uint32_t k = i; k > j; --k)
            neighbor->sequence[k] = neighbor->sequence[k - 1];
    neighbor->sequence[j] = temp;
}

// 人工蜂群算法
int32_t ArtificialBeeColony(const InputParam *input, OutputParam *output)
{
    int32_t ret;

    uint32_t len = input->ioVec.len;

    // 初始化蜜蜂群
    OutputParam employedBees[NUM_EMPLOYED_BEES];
    OutputParam bestSolution;
    bestSolution.len = len;
    bestSolution.sequence = (uint32_t *)malloc(len * sizeof(uint32_t));

    uint32_t employedBeesCost[NUM_EMPLOYED_BEES]; // 雇佣蜂对应的解的代价
    uint32_t bestCost = UINT32_MAX;               // 最好的解的代价
    int trials[NUM_EMPLOYED_BEES] = {0};          // 每个雇佣蜂的尝试次数

    // 初始化雇佣蜂
    for (int i = 0; i < NUM_EMPLOYED_BEES; ++i)
    {
        employedBees[i].len = len;
        employedBees[i].sequence = (uint32_t *)malloc(len * sizeof(uint32_t));
        for (uint32_t j = 0; j < input->ioVec.len; ++j)
            employedBees[i].sequence[j] = output->sequence[j];
        if (i != 0) // 第一只不随机
            generateRandomSolution(&employedBees[i], len);
        employedBeesCost[i] = getTotalCost(input, &employedBees[i]);
        printf("%ld emplyed bee initial cost: %ld\n", i, employedBeesCost[i]);
        // 更新最优解
        if (employedBeesCost[i] < bestCost)
        {
            bestCost = employedBeesCost[i];
            copySolution(bestSolution.sequence, employedBees[i].sequence, len);
        }
    }

    // 临时变量用于存储邻域解
    OutputParam neighbor;
    neighbor.len = len;
    neighbor.sequence = (uint32_t *)malloc(len * sizeof(uint32_t));

    int iteration = 0;
    while (iteration < MAX_ITERATIONS)
    {
        // 雇佣蜂阶段
        for (int i = 0; i < NUM_EMPLOYED_BEES; ++i)
        {
            generateNeighborSolution(&neighbor, &employedBees[i], len);
            uint32_t neighborCost = getTotalCost(input, &neighbor);

            // 如果邻域解更好，接受邻域解
            if (neighborCost < employedBeesCost[i])
            {
                employedBeesCost[i] = neighborCost;
                copySolution(employedBees[i].sequence, neighbor.sequence, len);
                trials[i] = 0; // 重置尝试次数
            }
            else
            {
                trials[i]++; // 尝试次数增加
            }

            // 更新最优解
            if (employedBeesCost[i] < bestCost)
            {
                bestCost = employedBeesCost[i];
                copySolution(bestSolution.sequence, employedBees[i].sequence, len);
                printf("%ld iter, one operator optimization cost: %ld\n", iteration, bestCost);
            }
        }

        // 计算选择概率（根据解的质量）
        double totalFitness = 0.0;
        double fitness[NUM_EMPLOYED_BEES];
        for (int i = 0; i < NUM_EMPLOYED_BEES; ++i)
        {
            fitness[i] = 1.0 / (1.0 + employedBeesCost[i]); // 代价越小，适应度越高
            totalFitness += fitness[i];
        }

        // 跟随蜂阶段
        for (int i = 0; i < NUM_ONLOOKER_BEES; ++i)
        {
            // 根据适应度选择解
            double r = (double)rand() / RAND_MAX * totalFitness;
            double cumulativeFitness = 0.0;
            int selectedBeeIndex = 0;
            for (int j = 0; j < NUM_EMPLOYED_BEES; ++j)
            {
                cumulativeFitness += fitness[j];
                if (cumulativeFitness >= r)
                {
                    selectedBeeIndex = j;
                    break;
                }
            }

            // 跟随蜂搜索邻域解
            generateNeighborSolution(&neighbor, &employedBees[selectedBeeIndex], len);
            uint32_t neighborCost = getTotalCost(input, &neighbor);

            // 如果邻域解更好，接受邻域解
            if (neighborCost < employedBeesCost[selectedBeeIndex])
            {
                employedBeesCost[selectedBeeIndex] = neighborCost;
                copySolution(employedBees[selectedBeeIndex].sequence, neighbor.sequence, len);
                trials[selectedBeeIndex] = 0; // 重置尝试次数
            }
            else
            {
                trials[selectedBeeIndex]++; // 尝试次数增加
            }

            // 更新最优解
            if (employedBeesCost[selectedBeeIndex] < bestCost)
            {
                bestCost = employedBeesCost[selectedBeeIndex];
                copySolution(bestSolution.sequence, employedBees[selectedBeeIndex].sequence, len);
                printf("%ld iter, one operator optimization cost: %ld\n", iteration, bestCost);
            }
        }

        // 侦查蜂阶段：检查是否有过多尝试次数的解，如果有则重新生成随机解
        for (int i = 0; i < NUM_EMPLOYED_BEES; ++i)
        {
            if (trials[i] > MAX_TRIALS)
            {
                generateRandomSolution(&employedBees[i], len);
                employedBeesCost[i] = getTotalCost(input, &employedBees[i]);
                trials[i] = 0; // 重置尝试次数
            }
        }

        iteration++;
    }

    // 将最优解复制到输出参数
    output->len = bestSolution.len;
    copySolution(output->sequence, bestSolution.sequence, bestSolution.len);

    printf("Final best cost: %u\n", bestCost);

    // 释放内存
    for (int i = 0; i < NUM_EMPLOYED_BEES; ++i)
    {
        free(employedBees[i].sequence);
    }
    free(neighbor.sequence);
    free(bestSolution.sequence);

    return 0;
}

// 快速排序
void QuickSort(IOUint *a, int len)
{
    int low = 0;
    int high = len - 1;
    int stack[high - low + 1];
    int top = -1;
    stack[++top] = low;
    stack[++top] = high;

    while (top >= 0)
    {
        // 从栈中弹出 high 和 low 值，表示当前需要排序的子数组的边界
        high = stack[top--];
        low = stack[top--];

        // 选择子数组的最后一个元素作为枢轴（pivot），并初始化变量 i 为 low - 1
        uint32_t pivot = a[high].startLpos;
        int i = low - 1;

        // 遍历当前子数组
        for (int j = low; j < high; ++j)
        {
            // 将所有小于枢轴的元素移到枢轴的左边
            if (a[j].startLpos < pivot)
            {
                ++i; // i 指向当前小于枢轴的元素的位置
                IOUint temp = a[i];
                a[i] = a[j];
                a[j] = temp;
            }
        }

        // 将枢轴元素放到正确的位置
        IOUint temp = a[i + 1];
        a[i + 1] = a[high];
        a[high] = temp;
        int pi = i + 1;

        // 根据枢轴的位置 pi，将左子数组和右子数组的边界压入栈中
        if (pi - 1 > low)
        {
            stack[++top] = low;
            stack[++top] = pi - 1;
        }
        if (pi + 1 < high)
        {
            stack[++top] = pi + 1;
            stack[++top] = high;
        }
    }
}

int32_t partition_scan(const InputParam *input, OutputParam *output)
{
    // 复制 IO 请求数组并按 lpos 排序
    IOUint *sortedIOs = (IOUint *)malloc(input->ioVec.len * sizeof(IOUint));
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
        sortedIOs[i] = input->ioVec.ioArray[i];
    QuickSort(sortedIOs, input->ioVec.len);

    for (int i = 0; i < input->ioVec.len; i++)
    {
        if (i + 1 == input->ioVec.len)
            break;
        if (sortedIOs[i + 1].startLpos < sortedIOs[i].startLpos)
        {
            printf("sort error!\n");
            abort();
        }
    }

    //----排序结束----

    //----搜索最佳分割参数----
    int min_time = 0x3f3f3f3f, best_partition_size = 5000;
    int best_scan_method = 1;
    int *best_sequence = (int *)malloc(input->ioVec.len * sizeof(int));
    bool *vis = (bool *)malloc((input->ioVec.len + 1) * sizeof(bool));
    int partition_io_num[1000] = {0};
    int partition_io_start[1000] = {0};

    // 按固定长度进行分区，对长度参数进行搜索
    for (int i = 5000; i <= 740000; i += 5000)
    {
        int partition_start_now = 0;
        int now = 0;
        int partition_threshold = i;
        memset(partition_io_start, 0, sizeof(partition_io_start));
        memset(partition_io_num, 0, sizeof(partition_io_num));
        // 遍历所有请求，统计每个分区的起始请求和请求数量
        for (int j = 0; j < input->ioVec.len; j++)
        {
            if (sortedIOs[j].startLpos >= partition_start_now + partition_threshold)
            {
                while (sortedIOs[j].startLpos >= partition_start_now + partition_threshold)
                {
                    partition_start_now += partition_threshold;
                    now++;
                }
                partition_io_start[now] = j;
            }
            partition_io_num[now]++;
        }
        // 分区处理方法，SORT、SCAN、MPScan、MPScan*
        int scan_method[] = {0, 1, 2, 3};
        // int scan_method[] = {0, 1, 2};
        // int scan_method[] = {3};
        int method_num = sizeof(scan_method) / sizeof(scan_method[0]);
        // DEBUG("method_num=%d\n", method_num);
        for (int method_idx = 0; method_idx < method_num; method_idx++)
        {
            // DEBUG("partition_len=%d, method=%d\n", i, scan_method[method_idx]);
            // 重置 vis 数组
            for (int j = 0; j < input->ioVec.len + 1; j++)
                vis[j] = 0;
            HeadInfo head = input->headInfo;
            head.status = HEAD_RW;
            // 对每个分区进行处理
            for (int j = 0; j <= now; j++)
            {
                // 跳过空分区
                if (partition_io_num[j] == 0)
                    continue;
                if (scan_method[method_idx] < 3)
                    Sort_Scan_MPScan_Perpartition(output, sortedIOs, vis, &head, partition_io_start[j], partition_io_num[j], scan_method[method_idx]);
                else if (scan_method[method_idx] == 3)
                {
                    MPScan_star_PerPartition(input, output, sortedIOs, vis, &head, partition_io_start[j], partition_io_start[j] + partition_io_num[j]);
                }
                head.wrap = sortedIOs[partition_io_start[j] + partition_io_num[j] - 1].wrap;
                head.lpos = sortedIOs[partition_io_start[j] + partition_io_num[j] - 1].endLpos;
            }
            AccessTime accessTime = {0};
            // 计算当前分区方案下的总寻址时间
            TotalAccessTime(input, output, &accessTime);
            int time = accessTime.addressDuration;
            // 记录最小寻址时间的分区方案
            if (time <= min_time)
            {
                best_scan_method = scan_method[method_idx];
                best_partition_size = i;
                min_time = time;
                for (int j = 0; j < input->ioVec.len; j++)
                    best_sequence[j] = output->sequence[j];
            }
        }
    }
    // printf("best_scan_method=%d best_partition_size=%d\n", best_scan_method, best_partition_size);
    for (int i = 0; i < input->ioVec.len; i++)
    {
        output->sequence[i] = best_sequence[i];
    }
    free(best_sequence);
    free(sortedIOs);
}

int32_t Sort_Scan_MPScan_Perpartition(OutputParam *output, IOUint *sortedIOs, bool *vis, HeadInfo *head, int partition_start, int partition_len, const int scan_method)
{
    uint32_t index = partition_start;
    HeadInfo currentHead = *head;
    int direction = 1; // 扫描方向：1 表示从 BOT 向 EOT 扫描，-1 表示从 EOT 向 BOT 扫描
    while (index < partition_start + partition_len)
    {
        if (scan_method == 0)
        { // SORT
            output->sequence[index] = sortedIOs[index].id;
            index++;
            continue;
        }
        if (direction == 1)
        {
            // 从 BOT 向 EOT 扫描
            for (uint32_t i = partition_start; i < partition_start + partition_len; ++i)
            {
                if (sortedIOs[i].wrap & 1 || vis[sortedIOs[i].id])
                {
                    continue;
                }
                if (scan_method == 1)
                { // SCAN1，两趟扫描
                    if (sortedIOs[i].startLpos >= currentHead.lpos)
                    {
                        output->sequence[index++] = sortedIOs[i].id;
                        vis[sortedIOs[i].id] = 1;
                        currentHead.wrap = sortedIOs[i].wrap;
                        currentHead.lpos = sortedIOs[i].startLpos;
                    }
                }
                else if (scan_method == 2)
                { // SCAN2，多趟扫描
                    if (sortedIOs[i].startLpos > currentHead.lpos || currentHead.lpos == 0)
                    {
                        output->sequence[index++] = sortedIOs[i].id;
                        vis[sortedIOs[i].id] = 1;
                        currentHead.wrap = sortedIOs[i].wrap;
                        currentHead.lpos = sortedIOs[i].endLpos;
                    }
                }
            }
            direction = -1; // 改变扫描方向
            // 赋值为最大值以保证下一轮扫描中大于当前位置的请求不会被忽略
            currentHead.lpos = MAX_LPOS + 1;
        }
        else
        {
            // 从 EOT 向 BOT 扫描
            for (int32_t i = partition_start + partition_len - 1; i >= partition_start; --i)
            {
                if (!(sortedIOs[i].wrap & 1) || vis[sortedIOs[i].id])
                {
                    continue;
                }
                if (scan_method == 1)
                { // SCAN1，两趟扫描
                    if (sortedIOs[i].startLpos <= currentHead.lpos || currentHead.lpos == 0)
                    {
                        output->sequence[index++] = sortedIOs[i].id;
                        vis[sortedIOs[i].id] = 1;
                        currentHead.wrap = sortedIOs[i].wrap;
                        currentHead.lpos = sortedIOs[i].startLpos;
                    }
                }
                else if (scan_method == 2)
                { // SCAN2，多趟扫描
                    if (sortedIOs[i].startLpos < currentHead.lpos || currentHead.lpos == 0)
                    {
                        output->sequence[index++] = sortedIOs[i].id;
                        vis[sortedIOs[i].id] = 1;
                        currentHead.wrap = sortedIOs[i].wrap;
                        currentHead.lpos = sortedIOs[i].endLpos;
                    }
                }
            }
            direction = 1; // 改变扫描方向
            // 赋值为0以保证下一轮扫描中小于当前位置的请求不会被忽略
            currentHead.lpos = 0;
        }
    }
    return RETURN_OK;
}

// 用MPScan处理每个分区内部
int32_t MPScan_star_PerPartition(const InputParam *input, OutputParam *output, IOUint *sortedIOs, bool *vis, HeadInfo *head, int partition_start, int partition_end)
{
    uint32_t index = partition_start;
    HeadInfo currentHead = *head;
    int direction = 1; // 扫描方向：1 表示从 BOT 向 EOT 扫描，-1 表示从 EOT 向 BOT 扫描

    while (index < partition_end)
    {
        if (direction == 1)
        {
            // 从 BOT 向 EOT 扫描
            for (uint32_t i = partition_start; i < partition_end; ++i)
            {
                if (sortedIOs[i].wrap & 1 || vis[sortedIOs[i].id])
                    continue;
                if (sortedIOs[i].startLpos > currentHead.lpos || currentHead.lpos == 0)
                {
                    output->sequence[index++] = sortedIOs[i].id;
                    vis[sortedIOs[i].id] = 1;
                    currentHead.wrap = sortedIOs[i].wrap;
                    currentHead.lpos = sortedIOs[i].endLpos;
                }
            }
            direction = -1;                  // 改变扫描方向
            currentHead.lpos = MAX_LPOS + 1; // 赋值为最大值以保证下一轮扫描中大于当前位置的请求不会被忽略
        }
        else
        {
            // 从 EOT 向 BOT 扫描
            for (int32_t i = partition_end - 1; i >= partition_start; --i)
            {
                if (!(sortedIOs[i].wrap & 1) || vis[sortedIOs[i].id])
                    continue;
                if (sortedIOs[i].startLpos < currentHead.lpos || currentHead.lpos == 0)
                {
                    output->sequence[index++] = sortedIOs[i].id;
                    vis[sortedIOs[i].id] = 1;
                    currentHead.wrap = sortedIOs[i].wrap;
                    currentHead.lpos = sortedIOs[i].endLpos;
                }
            }
            direction = 1;        // 改变扫描方向
            currentHead.lpos = 0; // 赋值为0以保证下一轮扫描中小于当前位置的请求不会被忽略
        }
    }

    // 将最后一轮扫描插入前面
    OutputParam *tmp = (OutputParam *)malloc(sizeof(OutputParam));
    tmp->len = input->ioVec.len;
    tmp->sequence = (uint32_t *)malloc(input->ioVec.len * sizeof(uint32_t));
    memcpy(tmp->sequence, output->sequence, input->ioVec.len * sizeof(int));
    AccessTime accessTime;
    TotalAccessTime(input, output, &accessTime);
    while (true)
    {
        // 最后一轮扫描在 output 中的下标范围为 [idx+1, output->len - 1]
        int32_t idx = partition_end - 1;
        if (input->ioVec.ioArray[output->sequence[idx] - 1].wrap & 1)
            while (idx > partition_start && (input->ioVec.ioArray[output->sequence[idx - 1] - 1].wrap & 1) &&
                   input->ioVec.ioArray[output->sequence[idx] - 1].startLpos < input->ioVec.ioArray[output->sequence[idx - 1] - 1].startLpos)
            { // 奇数
                idx--;
            }
        if (!(input->ioVec.ioArray[output->sequence[idx] - 1].wrap & 1))
            while (idx > partition_start && !(input->ioVec.ioArray[output->sequence[idx - 1] - 1].wrap & 1) &&
                   input->ioVec.ioArray[output->sequence[idx] - 1].startLpos > input->ioVec.ioArray[output->sequence[idx - 1] - 1].startLpos)
            { // 偶数
                idx--;
            }
        if (idx == partition_start)
            break; // 当前已经是最后一轮扫描

        // 遍历最后一轮的每个 IO
        for (int i = idx; i < partition_end; ++i)
        {
            int32_t minTime = INT32_MAX;
            int32_t best_pos = -1;
            HeadInfo z = {input->ioVec.ioArray[output->sequence[i] - 1].wrap, input->ioVec.ioArray[output->sequence[i] - 1].startLpos, HEAD_RW};

            // 寻找插入的最佳位置
            for (int j = partition_start; j < i - 1; ++j)
            {
                HeadInfo x = {input->ioVec.ioArray[output->sequence[j] - 1].wrap, input->ioVec.ioArray[output->sequence[j] - 1].startLpos, HEAD_RW};
                HeadInfo y = {input->ioVec.ioArray[output->sequence[j + 1] - 1].wrap, input->ioVec.ioArray[output->sequence[j + 1] - 1].startLpos, HEAD_RW};
                int32_t seekTime = SeekTimeCalculate(&x, &z) + SeekTimeCalculate(&z, &y) - SeekTimeCalculate(&x, &y);
                if (seekTime < minTime)
                {
                    best_pos = j;
                    minTime = seekTime;
                }
            }

            if (best_pos == -1)
            {
                continue;
            }
            // 将当前 IO 插入到 best_pos 后面
            for (int j = i; j > best_pos + 1; --j)
                tmp->sequence[j] = tmp->sequence[j - 1];
            tmp->sequence[best_pos + 1] = output->sequence[i]; // 更新该处的 IO 序号
        }

        AccessTime tmpTime;
        TotalAccessTime(input, tmp, &tmpTime);

        if (tmpTime.addressDuration < accessTime.addressDuration)
        {
            accessTime.addressDuration = tmpTime.addressDuration;
            memcpy(output->sequence, tmp->sequence, input->ioVec.len * sizeof(int));
        }
        else
        {
            break;
        }
    }

    free(tmp->sequence);
    free(tmp);

    return RETURN_OK;
}

int32_t SCAN2(const InputParam *input, OutputParam *output)
{
    // 初始化输出参数
    output->len = input->ioVec.len;

    // 复制 IO 请求数组并按 lpos 排序
    IOUint *sortedIOs = (IOUint *)malloc(input->ioVec.len * sizeof(IOUint));
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
    {
        sortedIOs[i] = input->ioVec.ioArray[i];
    }
    QuickSort(sortedIOs, input->ioVec.len);

    // 初始化当前头位置为输入的头状态
    HeadInfo currentHead = {input->headInfo.wrap, input->headInfo.lpos, input->headInfo.status};
    // HeadInfo currentHead = {0, 0, 0};

    // 扫描方向：1 表示从 BOT 向 EOT 扫描，-1 表示从 EOT 向 BOT 扫描
    int direction = 1;
    uint32_t index = 0;

    bool vis[input->ioVec.len + 1];
    memset(vis, 0, sizeof(vis));
    int last_cnt = 1;
    while (index < input->ioVec.len)
    {
        if (direction == 1)
        {
            int cur_cnt = 0;
            // 从 BOT 向 EOT 扫描
            for (uint32_t i = 0; i < input->ioVec.len; ++i)
            {
                if (sortedIOs[i].wrap & 1 || vis[sortedIOs[i].id])
                {
                    continue;
                }
                if (sortedIOs[i].startLpos >= currentHead.lpos)
                {
                    output->sequence[index++] = sortedIOs[i].id;
                    vis[sortedIOs[i].id] = 1;
                    cur_cnt++;
                    currentHead.wrap = sortedIOs[i].wrap;
                    currentHead.lpos = sortedIOs[i].endLpos;
                }
            }
            direction = -1; // 改变扫描方向
            currentHead.lpos = MAX_LPOS;
            last_cnt = cur_cnt;
        }
        else
        {
            int cur_cnt = 0;
            // 从 EOT 向 BOT 扫描
            for (int32_t i = input->ioVec.len - 1; i >= 0; --i)
            {
                if (!(sortedIOs[i].wrap & 1) || vis[sortedIOs[i].id])
                {
                    continue;
                }
                if (sortedIOs[i].startLpos <= currentHead.lpos)
                {
                    output->sequence[index++] = sortedIOs[i].id;
                    vis[sortedIOs[i].id] = 1;
                    cur_cnt++;
                    currentHead.wrap = sortedIOs[i].wrap;
                    currentHead.lpos = sortedIOs[i].endLpos;
                }
            }
            direction = 1; // 改变扫描方向
            currentHead.lpos = 0;
            last_cnt = cur_cnt;
        }
    }

    free(sortedIOs);
    return RETURN_OK;
}

int32_t MPScan(const InputParam *input, OutputParam *output)
{
    SCAN2(input, output);

    // 将最后一轮扫描插入前面
    OutputParam *tmp = (OutputParam *)malloc(sizeof(OutputParam));
    tmp->len = input->ioVec.len;
    tmp->sequence = (uint32_t *)malloc(input->ioVec.len * sizeof(uint32_t));
    memcpy(tmp->sequence, output->sequence, input->ioVec.len * sizeof(int));
    AccessTime accessTime;
    TotalAccessTime(input, output, &accessTime);
    while (true)
    {
        // 最后一轮扫描在 output 中的下标范围为 [idx+1, output->len - 1]
        int32_t io_len = 0, idx = output->len - 1;
        while (idx >= 1 && (input->ioVec.ioArray[output->sequence[idx] - 1].wrap & 1) &&
               input->ioVec.ioArray[output->sequence[idx] - 1].startLpos < input->ioVec.ioArray[output->sequence[idx - 1] - 1].startLpos)
        { // 奇数
            idx--;
        }
        while (idx >= 1 && !(input->ioVec.ioArray[output->sequence[idx] - 1].wrap & 1) &&
               input->ioVec.ioArray[output->sequence[idx] - 1].startLpos > input->ioVec.ioArray[output->sequence[idx - 1] - 1].startLpos)
        { // 偶数
            idx--;
        }
        if (idx < 0)
            break; // 当前已经是最后一轮扫描

        // printf("\n%d:", output->sequence[idx]);
        // 遍历最后一轮的每个 IO
        for (int i = idx; i < output->len; ++i)
        {
            // printf("\ni = %d , ", output->sequence[i]);
            int32_t minTime = INT32_MAX;
            int32_t best_pos = -1;
            HeadInfo z = {input->ioVec.ioArray[output->sequence[i] - 1].wrap, input->ioVec.ioArray[output->sequence[i] - 1].startLpos, HEAD_RW};

            // 寻找插入的最佳位置
            for (int j = 0; j < i - 1; ++j)
            {
                // if(input->ioVec.ioArray[output->sequence[j]-1].wrap != input->ioVec.ioArray[output->sequence[i]-1].wrap)
                //     continue;
                HeadInfo x = {input->ioVec.ioArray[output->sequence[j] - 1].wrap, input->ioVec.ioArray[output->sequence[j] - 1].startLpos, HEAD_RW};
                HeadInfo y = {input->ioVec.ioArray[output->sequence[j + 1] - 1].wrap, input->ioVec.ioArray[output->sequence[j + 1] - 1].startLpos, HEAD_RW};
                int32_t seekTime = SeekTimeCalculate(&x, &z) + SeekTimeCalculate(&z, &y) - SeekTimeCalculate(&x, &y);
                // printf("seekTime = %d, minTime = %d ", seekTime, minTime);
                if (seekTime < minTime)
                {
                    best_pos = j;
                    minTime = seekTime;
                }
            }

            // printf("best_pos = %d , ", best_pos);
            // 将当前 IO 插入到 best_pos 后面
            for (int j = i; j > best_pos + 1; --j)
            {
                tmp->sequence[j] = tmp->sequence[j - 1];
            }
            tmp->sequence[best_pos + 1] = output->sequence[i]; // 更新该处的 IO 序号
        }

        AccessTime tmpTime;
        TotalAccessTime(input, tmp, &tmpTime);

        // printf("\ntmp[ ");
        // for (int i = 0; i < tmp->len; ++i) {
        //     printf("%d ", tmp->sequence[i]);
        // }
        // printf("]\nout[ ");
        // for (int i = 0; i < output->len; ++i) {
        //     printf("%d ", output->sequence[i]);
        // }
        // printf("]\ntmp: %d, output: %d\n", tmpTime.addressDuration, accessTime.addressDuration);

        if (tmpTime.addressDuration < accessTime.addressDuration)
        {
            accessTime.addressDuration = tmpTime.addressDuration;
            memcpy(output->sequence, tmp->sequence, input->ioVec.len * sizeof(int));
        }
        else
        {
            break;
        }
    }

    return RETURN_OK;
}

MinHeap *createMinHeap(int capacity)
{
    MinHeap *heap = (MinHeap *)malloc(sizeof(MinHeap));
    heap->size = 0;
    heap->capacity = capacity;
    heap->nodes = (Node *)malloc(capacity * sizeof(Node));
    return heap;
}

void swap(Node *a, Node *b)
{
    Node temp = *a;
    *a = *b;
    *b = temp;
}

void heapify(MinHeap *heap, int idx)
{
    int smallest = idx;
    int left = 2 * idx + 1;
    int right = 2 * idx + 2;

    if (left < heap->size && heap->nodes[left].dis < heap->nodes[smallest].dis)
        smallest = left;

    if (right < heap->size && heap->nodes[right].dis < heap->nodes[smallest].dis)
        smallest = right;

    if (smallest != idx)
    {
        swap(&heap->nodes[idx], &heap->nodes[smallest]);
        heapify(heap, smallest);
    }
}

Node *extractMin(MinHeap *heap)
{ // 弹出最小值
    if (heap->size == 0)
    {
        return NULL;
    }

    swap(&heap->nodes[0], &heap->nodes[heap->size - 1]);
    heap->size--;
    heapify(heap, 0);

    return &heap->nodes[heap->size];
}

Node *getMin(MinHeap *heap) // 弹出最小值
{
    if (heap->size == 0)
    {
        return NULL;
    }

    return &heap->nodes[0];
}

void insertHeap(MinHeap *heap, Node node)
{
    if (heap->size == heap->capacity)
    {
        printf("Heap overflow\n");
        return;
    }

    heap->size++;
    int i = heap->size - 1;
    heap->nodes[i] = node;

    // 向上调整
    while (i != 0 && heap->nodes[(i - 1) / 2].dis > heap->nodes[i].dis)
    {
        swap(&heap->nodes[i], &heap->nodes[(i - 1) / 2]);
        i = (i - 1) / 2;
    }
}

int getValueInHeapArray(MinHeapArray *arr, int idx)
{
    return getMin(arr->heap_array[idx])->dis;
}

int getMinValueInHeapArray(MinHeapArray *arr)
{
    int value = getMin(arr->heap_array[0])->dis;
    return value;
}

void swapInHeapArray(MinHeapArray *arr, int idx1, int idx2)
{
    MinHeap *tmp = arr->heap_array[idx1];
    arr->heap_array[idx1] = arr->heap_array[idx2];
    arr->heap_array[idx2] = tmp;
}

void insertHeapInHeapArray(MinHeapArray *arr, MinHeap *heap)
{
    arr->heap_array[arr->size++] = heap;
    int i = arr->size - 1;
    while (i != 0 && getValueInHeapArray(arr, (i - 1) / 2) >= getValueInHeapArray(arr, i))
    {
        swapInHeapArray(arr, i, (i - 1) / 2);
        i = (i - 1) / 2;
    }
}

Node *getNodeInHeapArray(MinHeapArray *arr)
{
    return getMin(arr->heap_array[0]);
}

Node *popNodeInHeapArray(MinHeapArray *arr)
{
    if (arr->size == 0)
    {
        return NULL;
    }
    Node *tmp = extractMin(arr->heap_array[0]);
    minHeapArrayHeapify(arr, 0);
    return tmp;
}

MinHeap *popHeapInHeapArray(MinHeapArray *arr)
{
    if (arr->size == 0)
    {
        return NULL;
    }
    MinHeap *tmp = arr->heap_array[0];
    arr->heap_array[0] = arr->heap_array[arr->size - 1];
    arr->heap_array[arr->size - 1] = tmp;
    arr->size--;

    minHeapArrayHeapify(arr, 0);

    return arr->heap_array[arr->size];
}

void minHeapArrayHeapify(MinHeapArray *arr, int idx)
{
    int smallest = idx;
    int left = 2 * idx + 1;
    int right = 2 * idx + 2;

    if (left < arr->size && getValueInHeapArray(arr, left) < getValueInHeapArray(arr, smallest))
        smallest = left;

    if (right < arr->size && getValueInHeapArray(arr, right) < getValueInHeapArray(arr, smallest))
        smallest = right;

    if (smallest != idx)
    {
        swapInHeapArray(arr, idx, smallest);
        minHeapArrayHeapify(arr, smallest);
    }
}

const int maxn = 10001;
int *fa, *h, *sz;

void initUnionSet()
{
    fa = (int *)malloc(maxn * sizeof(int));
    h = (int *)malloc(maxn * sizeof(int));
    sz = (int *)malloc(maxn * sizeof(int));
    for (int i = 0; i < maxn; ++i)
    {
        fa[i] = i;
        h[i] = 1;
        sz[i] = 1;
    }
}

void freeUnionSet()
{
    free(fa);
    free(h);
}

int find(int x)
{
    return x == fa[x] ? x : (fa[x] = find(fa[x]));
}

void unite(int x, int y)
{
    x = find(x), y = find(y);
    if (h[x] < h[y])
    {
        fa[x] = y;
        sz[x] = sz[y] = sz[x] + sz[y];
    }
    else
    {
        fa[y] = x;
        if (h[x] == h[y])
            h[y]++;
        sz[x] = sz[y] = sz[x] + sz[y];
    }
}

void destoryMinHeap(MinHeap *heap)
{
    free(heap->nodes);
    free(heap);
    heap = NULL;
}

int32_t TailReinsert(const InputParam *input, OutputParam *output, int *pos, int pointnums)
{
    AccessTime accessTime;
    TotalAccessTime(input, output, &accessTime);
    uint32_t tapeBeltWear = TotalTapeBeltWearTimes(input, output, NULL); // 带体磨损
    uint32_t tapeMotorWear = TotalMotorWearTimes(input, output);         // 电机磨损

    uint32_t currentCost = accessTime.addressDuration + tapeBeltWear + tapeMotorWear; // 总代价
    uint32_t bestCost = currentCost;
    printf("cost before TailReinsert:%d\n", currentCost);

    // 将最后的点插入前面
    OutputParam *tmp = (OutputParam *)malloc(sizeof(OutputParam));
    tmp->len = input->ioVec.len;
    tmp->sequence = (uint32_t *)malloc(input->ioVec.len * sizeof(uint32_t));
    memcpy(tmp->sequence, output->sequence, input->ioVec.len * sizeof(int));

    uint32_t tail = pointnums - 1;
    while (tail)
    {
        int p_now = 0;
        for (int i = 0; i < output->len; i++)
            if (output->sequence[i] == pos[tail])
                p_now = i;

        printf("tail: %d, pos_now: %d\n", tail, p_now);

        // 最后一个 IO
        int32_t minCost = INT32_MAX;
        int32_t best_pos = -1;
        HeadInfo z = {input->ioVec.ioArray[output->sequence[p_now] - 1].wrap, input->ioVec.ioArray[output->sequence[p_now] - 1].startLpos, HEAD_RW};

        // 寻找插入的最佳位置，从前一个点的前面位置开始
        for (int j = 0; j < input->ioVec.len - 1; ++j)
        {
            if (j == p_now - 1)
                continue;
            HeadInfo x = {input->ioVec.ioArray[output->sequence[j] - 1].wrap, input->ioVec.ioArray[output->sequence[j] - 1].startLpos, HEAD_RW};
            HeadInfo y = {input->ioVec.ioArray[output->sequence[j + 1] - 1].wrap, input->ioVec.ioArray[output->sequence[j + 1] - 1].startLpos, HEAD_RW};
            int32_t cost = getCost(&x, &z) + getCost(&z, &y) - getCost(&x, &y);
            if (cost < minCost)
            {
                best_pos = j;
                minCost = cost;
            }
        }

        if (best_pos == -1)
        {
            tail--;
            continue;
        }

        // 将当前 IO 插入到 best_pos 后面
        if (best_pos > p_now)
        {
            for (int j = p_now; j <= best_pos; ++j)
                tmp->sequence[j] = tmp->sequence[j + 1];
            tmp->sequence[best_pos + 1] = output->sequence[p_now];
        }
        else
        {
            for (int j = p_now; j > best_pos + 1; --j)
                tmp->sequence[j] = tmp->sequence[j - 1];
            tmp->sequence[best_pos + 1] = output->sequence[p_now]; // 更新该处的 IO 序号
        }
        AccessTime tmpTime;
        TotalAccessTime(input, tmp, &tmpTime);
        tapeBeltWear = TotalTapeBeltWearTimes(input, tmp, NULL); // 带体磨损
        tapeMotorWear = TotalMotorWearTimes(input, tmp);         // 电机磨损

        currentCost = tmpTime.addressDuration + tapeBeltWear + tapeMotorWear; // 总代价
        printf("currentCost:%u\n", currentCost);
        if (currentCost < bestCost)
        {
            bestCost = currentCost;
            memcpy(output->sequence, tmp->sequence, input->ioVec.len * sizeof(int));
        }
        tail--;
    }

    printf("accessTime after TailReinsert:%d\n", bestCost);

    free(tmp->sequence);
    free(tmp);
}

int32_t merge(const InputParam *input, OutputParam *output)
{
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
    {
        output->sequence[i] = input->ioVec.ioArray[i].id;
    }

    MinHeapArray heap_array = {maxn, 0, (MinHeap **)malloc(maxn * sizeof(MinHeap *))};
    MinHeap *heap = createMinHeap(maxn + 1);

    int selected_value_sum = 0;

    // 初始化当前头位置为输入的头状态
    int max_edge_num = 8 * 1024 * 1024 / 8; // 假设最多分配8MB的内存给节点
    int edge_per_node = max_edge_num / (input->ioVec.len + 1);
    if (edge_per_node > input->ioVec.len + 1)
    {
        edge_per_node = input->ioVec.len + 1;
    }
    HeadInfo currentHead = {input->headInfo.wrap, input->headInfo.lpos, input->headInfo.status};
    for (int i = 0; i < input->ioVec.len; i++)
    {
        HeadInfo status_tmp = {input->ioVec.ioArray[i].wrap, input->ioVec.ioArray[i].startLpos, HEAD_RW};
        Node min_node = {0, i + 1, getCost(&currentHead, &status_tmp)};

        for (int j = 0; j < input->ioVec.len; j++)
        {
            if (i == j)
                continue;
            HeadInfo status1 = {input->ioVec.ioArray[j].wrap, input->ioVec.ioArray[j].endLpos, HEAD_RW};

            Node tmp_node = {j + 1, i + 1, getCost(&status1, &status_tmp)};
            if (tmp_node.dis < min_node.dis)
            {
                min_node = tmp_node;
            }
        }
        insertHeap(heap, min_node);
    }

    int nex[maxn], vis[maxn];
    memset(nex, 0, sizeof(nex));
    memset(vis, 0, sizeof(vis));
    initUnionSet();
    int set_num = input->ioVec.len + 1;

    // 记录最后merge选取的最后10条边
    int lastpoints = 10;
    int *pos = (int *)malloc(lastpoints * sizeof(int));
    int tot_sz = 0;

    while (sz[0] != input->ioVec.len + 1)
    {
        if (sz[0] == input->ioVec.len + 1)
        {
            break;
        }

        int min_value = INT32_MAX;
        int min_heap_idx = -1;
        Node *node = extractMin(heap);

        if (nex[node->x] == 0 && (node->x == 0 || (nex[node->y] != node->x)) && vis[node->y] == 0 && find(node->x) != find(node->y))
        {
            tot_sz++;
            // 记录最后merge选取的最后10条边
            if (tot_sz > input->ioVec.len - lastpoints)
            {
                pos[tot_sz + lastpoints - input->ioVec.len - 1] = node->y;
                printf("pos[%d]=%d\n", tot_sz + lastpoints - input->ioVec.len - 1, node->y);
            }
            unite(node->x, node->y);
            nex[node->x] = node->y;
            vis[node->y] = 1;
            selected_value_sum += node->dis;
            set_num--;
        }
        else
        {
            int target_id = node->y;
            HeadInfo status_tmp = {input->ioVec.ioArray[target_id - 1].wrap, input->ioVec.ioArray[target_id - 1].startLpos, HEAD_RW};
            Node min_node;
            min_node.dis = INT32_MAX;
            if (nex[0] == 0)
            {
                Node tmp_node = {0, target_id, getCost(&currentHead, &status_tmp)};
                min_node = tmp_node;
            }
            for (int source_id = 1; source_id < input->ioVec.len + 1; source_id++)
            {
                if (target_id == source_id || nex[source_id] != 0 || find(source_id) == find(target_id))
                    continue;
                HeadInfo status1 = {input->ioVec.ioArray[source_id - 1].wrap, input->ioVec.ioArray[source_id - 1].endLpos, HEAD_RW};
                Node tmp_node = {source_id, target_id, getCost(&status1, &status_tmp)};
                if (tmp_node.dis < min_node.dis)
                {
                    min_node = tmp_node;
                }
            }
            insertHeap(heap, min_node);
        }
    }
    int now = nex[0], cnt = 0;
    while (cnt < input->ioVec.len)
    {
        output->sequence[cnt++] = now;
        now = nex[now];
    }
    destoryMinHeap(heap);

    TailReinsert(input, output, pos, lastpoints);

    return RETURN_OK;
}

Node *randomExtractMin(MinHeap *heap)
{ // 弹出最小值
    if (heap->size == 0)
    {
        return NULL;
    }

    int random_choice = rand() % 100; // 生成 0 到 99 的随机数

    if (random_choice < 90)
    {
        // 90% 的概率选择堆中的最小节点（堆顶）
        swap(&heap->nodes[0], &heap->nodes[heap->size - 1]);
    }
    else
    {
        // 10% 的概率随机选择堆中的其他节点
        if (heap->size > 1)
        {
            int random_index = (rand() % (heap->size - 1) / 2) + 1;         // 生成 1 到 heap->size - 1 的随机索引
            swap(&heap->nodes[random_index], &heap->nodes[heap->size - 1]); // 将随机节点与最后一个节点交换
        }
        else
        {
            swap(&heap->nodes[0], &heap->nodes[heap->size - 1]);
        }
    }

    heap->size--;     // 减少堆大小
    heapify(heap, 0); // 对堆进行堆化，维护堆的特性

    return &heap->nodes[heap->size]; // 返回被弹出的节点
}

int32_t merge_random(const InputParam *input, OutputParam *output)
{
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
    {
        output->sequence[i] = input->ioVec.ioArray[i].id;
    }

    // return RETURN_OK;

    MinHeapArray heap_array = {maxn, 0, (MinHeap **)malloc(maxn * sizeof(MinHeap *))};
    MinHeap *heap = createMinHeap(maxn + 1);

    int selected_value_sum = 0;

    // 初始化当前头位置为输入的头状态
    int max_edge_num = 8 * 1024 * 1024 / 8; // 假设最多分配8MB的内存给节点
    int edge_per_node = max_edge_num / (input->ioVec.len + 1);
    if (edge_per_node > input->ioVec.len + 1)
    {
        edge_per_node = input->ioVec.len + 1;
    }
    HeadInfo currentHead = {input->headInfo.wrap, input->headInfo.lpos, input->headInfo.status};
    for (int i = 0; i < input->ioVec.len; i++)
    {
        HeadInfo status_tmp = {input->ioVec.ioArray[i].wrap, input->ioVec.ioArray[i].startLpos, HEAD_RW};
        Node min_node = {0, i + 1, getCost(&currentHead, &status_tmp)};

        for (int j = 0; j < input->ioVec.len; j++)
        {
            if (i == j)
                continue;
            HeadInfo status1 = {input->ioVec.ioArray[j].wrap, input->ioVec.ioArray[j].endLpos, HEAD_RW};

            Node tmp_node = {j + 1, i + 1, getCost(&status1, &status_tmp)};
            if (tmp_node.dis < min_node.dis)
            {
                min_node = tmp_node;
            }
        }
        insertHeap(heap, min_node);
    }

    int nex[maxn], vis[maxn];
    memset(nex, 0, sizeof(nex));
    memset(vis, 0, sizeof(vis));
    initUnionSet();
    int set_num = input->ioVec.len + 1;
    while (sz[0] != input->ioVec.len + 1)
    {
        if (sz[0] == input->ioVec.len + 1)
        {
            break;
        }

        int min_value = INT32_MAX;
        int min_heap_idx = -1;
        Node *node = randomExtractMin(heap);

        if (nex[node->x] == 0 && (node->x == 0 || (nex[node->y] != node->x)) && vis[node->y] == 0 && find(node->x) != find(node->y))
        {
            unite(node->x, node->y);
            nex[node->x] = node->y;
            vis[node->y] = 1;
            selected_value_sum += node->dis;
            set_num--;
            // printf("%d\n", set_num);
        }
        else
        {
            int target_id = node->y;
            // printf("target_id: %d, source_id: %d, %d, %d, %d, %d, %d, %d\n", target_id, node->x, nex[node->x], vis[node->x], vis[node->y], nex[node->y] ,find(node->x) ,find(node->y));
            HeadInfo status_tmp = {input->ioVec.ioArray[target_id - 1].wrap, input->ioVec.ioArray[target_id - 1].startLpos, HEAD_RW};
            Node min_node;
            min_node.dis = INT32_MAX;
            if (nex[0] == 0)
            {
                Node tmp_node = {0, target_id, getCost(&currentHead, &status_tmp)};
                min_node = tmp_node;
            }
            for (int source_id = 1; source_id < input->ioVec.len + 1; source_id++)
            {
                if (target_id == source_id || nex[source_id] != 0 || find(source_id) == find(target_id))
                    continue;
                HeadInfo status1 = {input->ioVec.ioArray[source_id - 1].wrap, input->ioVec.ioArray[source_id - 1].endLpos, HEAD_RW};
                Node tmp_node = {source_id, target_id, getCost(&status1, &status_tmp)};
                if (tmp_node.dis < min_node.dis)
                {
                    min_node = tmp_node;
                }
            }
            insertHeap(heap, min_node);
        }
    }
    int now = nex[0], cnt = 0;
    while (cnt < input->ioVec.len)
    {
        output->sequence[cnt++] = now;
        now = nex[now];
    }
    destoryMinHeap(heap);

    // free(dis);
    return RETURN_OK;
}

// 包装 merge 函数的线程执行函数
// TODO：cost加上磨损
void *merge_thread(void *arg)
{
    ThreadArg *threadArg = (ThreadArg *)arg;

    // 创建线程返回值
    ThreadResult *result = (ThreadResult *)malloc(sizeof(ThreadResult));
    result->output = threadArg->output;

    merge(threadArg->input, result->output);
    TotalAccessTime(threadArg->input, result->output, &result->accessTime);

    pthread_exit(result); // 返回结果
}

// 包装 merge_random 函数的线程执行函数
// TODO：cost加上磨损
void *merge_random_thread(void *arg)
{
    ThreadArg *threadArg = (ThreadArg *)arg;

    // 创建线程返回值
    ThreadResult *result = (ThreadResult *)malloc(sizeof(ThreadResult));
    result->output = threadArg->output;

    merge_random(threadArg->input, result->output);
    TotalAccessTime(threadArg->input, result->output, &result->accessTime);

    pthread_exit(result); // 返回结果
}

// 线程1：执行 partition_scan 和 merge
// TODO：cost加上磨损
void *partition_scan_merge_thread(void *arg)
{
    int min_time = 0x3f3f3f3f;
    ThreadArg *threadArg = (ThreadArg *)arg;

    int *best_sequence = (int *)malloc(threadArg->input->ioVec.len * sizeof(int));
    if (!best_sequence)
        return NULL; // 内存分配失败，直接返回

    partition_scan(threadArg->input, threadArg->output);
    TotalAccessTime(threadArg->input, threadArg->output, threadArg->accessTime);
    if (threadArg->accessTime->addressDuration < min_time)
    {
        min_time = threadArg->accessTime->addressDuration;
        memcpy(best_sequence, threadArg->output->sequence, threadArg->input->ioVec.len * sizeof(int));
    }

    merge(threadArg->input, threadArg->output);
    TotalAccessTime(threadArg->input, threadArg->output, threadArg->accessTime);
    if (threadArg->accessTime->addressDuration < min_time)
    {
        min_time = threadArg->accessTime->addressDuration;
        memcpy(best_sequence, threadArg->output->sequence, threadArg->input->ioVec.len * sizeof(int));
    }

    memcpy(threadArg->output->sequence, best_sequence, threadArg->input->ioVec.len * sizeof(int));
    free(best_sequence);
    return NULL;
}

// 线程2：执行 MPScan 和 merge_random
// TODO：cost加上磨损
void *mp_scan_merge_random_thread(void *arg)
{
    int min_time = 0x3f3f3f3f;
    ThreadArg *threadArg = (ThreadArg *)arg;

    int *best_sequence = (int *)malloc(threadArg->input->ioVec.len * sizeof(int));
    if (!best_sequence)
        return NULL; // 内存分配失败，直接返回

    MPScan(threadArg->input, threadArg->output);
    TotalAccessTime(threadArg->input, threadArg->output, threadArg->accessTime);
    if (threadArg->accessTime->addressDuration < min_time)
    {
        min_time = threadArg->accessTime->addressDuration;
        memcpy(best_sequence, threadArg->output->sequence, threadArg->input->ioVec.len * sizeof(int));
    }

    merge_random(threadArg->input, threadArg->output);
    TotalAccessTime(threadArg->input, threadArg->output, threadArg->accessTime);
    if (threadArg->accessTime->addressDuration < min_time)
    {
        min_time = threadArg->accessTime->addressDuration;
        memcpy(best_sequence, threadArg->output->sequence, threadArg->input->ioVec.len * sizeof(int));
    }

    memcpy(threadArg->output->sequence, best_sequence, threadArg->input->ioVec.len * sizeof(int));
    free(best_sequence);
    return NULL;
}