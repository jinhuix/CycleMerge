#include "test_algorithms.h"

/**
 * @brief  算法接口
 * @param  input            输入参数
 * @param  output           输出参数
 * @return int32_t          返回成功或者失败，RETURN_OK 或 RETURN_ERROR
 */
int32_t IOScheduleAlgorithm(const InputParam *input, OutputParam *output)
{
    int32_t ret;

    /* 算法示例：先入先出算法 */
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++)
    {
        output->sequence[i] = input->ioVec.ioArray[i].id;
    }

    /* 调用公共函数示例：调用电机寻址、带体磨损、电机磨损函数 */
    HeadInfo start = {input->ioVec.ioArray[0].wrap, input->ioVec.ioArray[0].endLpos, HEAD_RW};
    HeadInfo end = {input->ioVec.ioArray[1].wrap, input->ioVec.ioArray[1].endLpos, HEAD_RW};
    int32_t seekT = 0;
    int32_t beltW = 0;
    int32_t motorW = 0;
    for (uint32_t i = 0; i < 10000; i++)
    {
        seekT = SeekTimeCalculate(&start, &end);
        beltW = BeltWearTimes(&start, &end, NULL);
        motorW = MotorWearTimes(&start, &end);
    }

    /* 调用公共函数示例：调用IO读写时间函数 */
    uint32_t rwT = ReadTimeCalculate(abs(input->ioVec.ioArray[0].endLpos - input->ioVec.ioArray[0].startLpos));

    return RETURN_OK;
}

int32_t SCAN(const InputParam *input, OutputParam *output)
{
    // 初始化输出参数
    output->len = input->ioVec.len;

    // 复制 IO 请求数组并按 lpos 排序
    IOUint *sortedIOs = (IOUint *)malloc(input->ioVec.len * sizeof(IOUint));
    if (sortedIOs == NULL)
    {
        free(output->sequence);
        return RETURN_ERROR;
    }
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
    {
        sortedIOs[i] = input->ioVec.ioArray[i];
    }

    // 快速排序
    int low = 0;
    int high = input->ioVec.len - 1;
    int stack[high - low + 1];
    int top = -1;

    stack[++top] = low;
    stack[++top] = high;

    while (top >= 0)
    {
        high = stack[top--];
        low = stack[top--];

        uint32_t pivot = sortedIOs[high].startLpos;
        int i = low - 1;

        for (int j = low; j < high; ++j)
        {
            if (sortedIOs[j].startLpos < pivot)
            {
                ++i;
                IOUint temp = sortedIOs[i];
                sortedIOs[i] = sortedIOs[j];
                sortedIOs[j] = temp;
            }
        }

        IOUint temp = sortedIOs[i + 1];
        sortedIOs[i + 1] = sortedIOs[high];
        sortedIOs[high] = temp;

        int pi = i + 1;

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

    // 初始化当前头位置为输入的头状态
    HeadInfo currentHead = {input->headInfo.wrap, input->headInfo.lpos, input->headInfo.status};

    // 扫描方向：1 表示从 BOT 向 EOT 扫描，-1 表示从 EOT 向 BOT 扫描
    int direction = 1;
    uint32_t index = 0;

    while (index < input->ioVec.len)
    {
        if (direction == 1)
        {
            // 从 BOT 向 EOT 扫描
            for (uint32_t i = 0; i < input->ioVec.len; ++i)
            {
                if (sortedIOs[i].startLpos >= currentHead.lpos)
                {
                    output->sequence[index++] = sortedIOs[i].id;
                    currentHead.wrap = sortedIOs[i].wrap;
                    currentHead.lpos = sortedIOs[i].endLpos;
                }
            }
            direction = -1; // 改变扫描方向
        }
        else
        {
            // 从 EOT 向 BOT 扫描
            for (int32_t i = input->ioVec.len - 1; i >= 0; --i)
            {
                if (sortedIOs[i].startLpos <= currentHead.lpos)
                {
                    output->sequence[index++] = sortedIOs[i].id;
                    currentHead.wrap = sortedIOs[i].wrap;
                    currentHead.lpos = sortedIOs[i].endLpos;
                }
            }
            direction = 1; // 改变扫描方向
        }
    }

    free(sortedIOs);
    return RETURN_OK;
}

int32_t NearestNeighborAlgorithm(const InputParam *input, OutputParam *output)
{
    // 初始化输出参数
    output->len = input->ioVec.len;

    // 记录哪些请求已经被处理
    bool processed[input->ioVec.len];
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
    {
        processed[i] = false;
    }

    // 初始化当前头位置为输入的头状态
    HeadInfo currentHead = {input->headInfo.wrap, input->headInfo.lpos, input->headInfo.status};

    // 对于每一个请求，找到距离当前头位置最近的未处理请求
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
    {
        int32_t minSeekTime = INT32_MAX;
        uint32_t nextRequestIndex = 0;

        for (uint32_t j = 0; j < input->ioVec.len; ++j)
        {
            if (processed[j])
                continue;

            HeadInfo nextHead = {input->ioVec.ioArray[j].wrap, input->ioVec.ioArray[j].startLpos, HEAD_RW};
            int32_t seekTime = SeekTimeCalculate(&currentHead, &nextHead);

            if (seekTime < minSeekTime)
            {
                minSeekTime = seekTime;
                nextRequestIndex = j;
            }
        }

        // 更新当前头位置为找到的最近请求的末尾位置
        currentHead.wrap = input->ioVec.ioArray[nextRequestIndex].wrap;
        currentHead.lpos = input->ioVec.ioArray[nextRequestIndex].endLpos;
        currentHead.status = HEAD_RW;

        // 将该请求标记为已处理
        processed[nextRequestIndex] = true;

        // 将该请求添加到输出序列中
        output->sequence[i] = input->ioVec.ioArray[nextRequestIndex].id;
    }

    return RETURN_OK;
}

int32_t SimulatedAnnealing(const InputParam *input, OutputParam *output)
{
    int32_t ret;

    // 初始化参数
    double initialTemperature = 100000;
    double coolingRate = 0.995;
    double minTemperature = 1;
    int maxIterations = 1000000;

    NearestNeighborAlgorithm(input, output);
    // 初始化解
    uint32_t currentSequence[input->ioVec.len];
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
    {
        // currentSequence[i] = input->ioVec.ioArray[i].id;
        currentSequence[i] = output->sequence[i];
    }

    // 计算初始解的总时延
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++)
    {
        output->sequence[i] = currentSequence[i];
    }
    AccessTime accessTime;
    TotalAccessTime(input, output, &accessTime);
    int32_t currentCost = accessTime.addressDuration + accessTime.readDuration;

    uint32_t bestSequence[input->ioVec.len];
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
    {
        bestSequence[i] = currentSequence[i];
    }
    int32_t bestCost = currentCost;

    // 模拟退火过程
    double temperature = initialTemperature;
    int iteration = 0;

    while (temperature > minTemperature && iteration < maxIterations)
    {
        // 生成邻域解
        uint32_t neighborSequence[input->ioVec.len];
        for (uint32_t i = 0; i < input->ioVec.len; ++i)
        {
            neighborSequence[i] = currentSequence[i];
        }
        uint32_t i = rand() % input->ioVec.len;
        uint32_t j = rand() % input->ioVec.len;
        uint32_t temp = neighborSequence[i];
        neighborSequence[i] = neighborSequence[j];
        neighborSequence[j] = temp;

        // 计算邻域解的总时延
        for (uint32_t i = 0; i < output->len; i++)
        {
            output->sequence[i] = neighborSequence[i];
        }
        TotalAccessTime(input, output, &accessTime);
        int32_t neighborCost = accessTime.addressDuration + accessTime.readDuration;

        // 接受邻域解的条件
        if (neighborCost < currentCost || ((double)rand() / RAND_MAX) < exp((currentCost - neighborCost) / temperature))
        {
            for (uint32_t i = 0; i < input->ioVec.len; ++i)
            {
                currentSequence[i] = neighborSequence[i];
            }
            currentCost = neighborCost;

            if (currentCost < bestCost)
            {
                for (uint32_t i = 0; i < input->ioVec.len; ++i)
                {
                    bestSequence[i] = currentSequence[i];
                }
                bestCost = currentCost;
            }
        }

        temperature *= coolingRate;
        ++iteration;
    }

    // 将最优解复制到输出参数
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++)
    {
        output->sequence[i] = bestSequence[i];
    }

    return RETURN_OK;
}

int32_t TabuSearch(const InputParam *input, OutputParam *output)
{
    int32_t ret;

    // 初始化参数
    int maxIterations = 1000;
    int tabuTenure = 10;

    NearestNeighborAlgorithm(input, output);

    // 初始化解
    uint32_t currentSequence[input->ioVec.len];
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
    {
        // currentSequence[i] = input->ioVec.ioArray[i].id;
        currentSequence[i] = output->sequence[i];
    }

    // 计算初始解的总时延
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++)
    {
        output->sequence[i] = currentSequence[i];
    }
    AccessTime accessTime;
    TotalAccessTime(input, output, &accessTime);
    int32_t currentCost = accessTime.addressDuration + accessTime.readDuration;

    uint32_t bestSequence[input->ioVec.len];
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
    {
        bestSequence[i] = currentSequence[i];
    }
    int32_t bestCost = currentCost;

    // 初始化禁忌表
    int tabuList[input->ioVec.len][input->ioVec.len];
    memset(tabuList, 0, sizeof(tabuList));

    // 禁忌搜索过程
    int iteration = 0;

    while (iteration < maxIterations)
    {
        // 生成邻域解并选择最优解
        uint32_t bestNeighborSequence[input->ioVec.len];
        int32_t bestNeighborCost = INT32_MAX;
        uint32_t bestI = 0, bestJ = 0;

        for (uint32_t i = 0; i < input->ioVec.len; ++i)
        {
            for (uint32_t j = i + 1; j < input->ioVec.len; ++j)
            {
                uint32_t neighborSequence[input->ioVec.len];
                for (uint32_t k = 0; k < input->ioVec.len; ++k)
                {
                    neighborSequence[k] = currentSequence[k];
                }
                uint32_t temp = neighborSequence[i];
                neighborSequence[i] = neighborSequence[j];
                neighborSequence[j] = temp;

                // 计算邻域解的总时延
                for (uint32_t k = 0; k < output->len; k++)
                {
                    output->sequence[k] = neighborSequence[k];
                }
                TotalAccessTime(input, output, &accessTime);
                int32_t neighborCost = accessTime.addressDuration + accessTime.readDuration;

                // 检查是否在禁忌表中或满足某些条件
                if ((tabuList[i][j] == 0 || neighborCost < bestCost) && neighborCost < bestNeighborCost)
                {
                    bestNeighborCost = neighborCost;
                    for (uint32_t k = 0; k < input->ioVec.len; ++k)
                    {
                        bestNeighborSequence[k] = neighborSequence[k];
                    }
                    bestI = i;
                    bestJ = j;
                }
            }
        }

        // 更新当前解
        for (uint32_t i = 0; i < input->ioVec.len; ++i)
        {
            currentSequence[i] = bestNeighborSequence[i];
        }
        currentCost = bestNeighborCost;

        // 更新禁忌表
        for (uint32_t i = 0; i < input->ioVec.len; ++i)
        {
            for (uint32_t j = 0; j < input->ioVec.len; ++j)
            {
                if (tabuList[i][j] > 0)
                {
                    tabuList[i][j]--;
                }
            }
        }
        tabuList[bestI][bestJ] = tabuTenure;

        // 更新最优解
        if (currentCost < bestCost)
        {
            for (uint32_t i = 0; i < input->ioVec.len; ++i)
            {
                bestSequence[i] = currentSequence[i];
            }
            bestCost = currentCost;
        }

        ++iteration;
    }

    // 将最优解复制到输出参数
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++)
    {
        output->sequence[i] = bestSequence[i];
    }

    return RETURN_OK;
}

int32_t HillClimbing(const InputParam *input, OutputParam *output)
{
    int32_t ret;

    NearestNeighborAlgorithm(input, output);

    // 初始化解
    uint32_t currentSequence[input->ioVec.len];
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
    {
        // currentSequence[i] = input->ioVec.ioArray[i].id;
        currentSequence[i] = output->sequence[i];
    }

    // 计算初始解的总时延
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++)
    {
        output->sequence[i] = currentSequence[i];
    }
    AccessTime accessTime;
    TotalAccessTime(input, output, &accessTime);
    int32_t currentCost = accessTime.addressDuration + accessTime.readDuration;

    uint32_t bestSequence[input->ioVec.len];
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
    {
        bestSequence[i] = currentSequence[i];
    }
    int32_t bestCost = currentCost;

    // 爬山算法过程
    int improved = 1;

    while (improved)
    {
        improved = 0;
        uint32_t neighborSequence[input->ioVec.len];
        int32_t bestNeighborCost = currentCost;

        for (uint32_t i = 0; i < input->ioVec.len; ++i)
        {
            for (uint32_t j = i + 1; j < input->ioVec.len; ++j)
            {
                for (uint32_t k = 0; k < input->ioVec.len; ++k)
                {
                    neighborSequence[k] = currentSequence[k];
                }
                uint32_t temp = neighborSequence[i];
                neighborSequence[i] = neighborSequence[j];
                neighborSequence[j] = temp;

                // 计算邻域解的总时延
                for (uint32_t k = 0; k < output->len; k++)
                {
                    output->sequence[k] = neighborSequence[k];
                }
                TotalAccessTime(input, output, &accessTime);
                int32_t neighborCost = accessTime.addressDuration + accessTime.readDuration;

                // 检查邻域解是否优于当前解
                if (neighborCost < bestNeighborCost)
                {
                    bestNeighborCost = neighborCost;
                    for (uint32_t k = 0; k < input->ioVec.len; ++k)
                    {
                        bestSequence[k] = neighborSequence[k];
                    }
                    improved = 1;
                }
            }
        }

        if (improved)
        {
            for (uint32_t i = 0; i < input->ioVec.len; ++i)
            {
                currentSequence[i] = bestSequence[i];
            }
            currentCost = bestNeighborCost;
        }
    }

    // 将最优解复制到输出参数
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++)
    {
        output->sequence[i] = bestSequence[i];
    }

    return RETURN_OK;
}

int32_t GeneticAlgorithm(const InputParam *input, OutputParam *output)
{
    // 初始化参数
    int populationSize = 50;
    double crossoverRate = 0.8;
    double mutationRate = 0.01;
    int maxIterations = 1000;

    // 初始化种群
    uint32_t population[populationSize][input->ioVec.len];
    for (int i = 0; i < populationSize; ++i)
    {
        for (uint32_t j = 0; j < input->ioVec.len; ++j)
        {
            population[i][j] = input->ioVec.ioArray[j].id;
        }
        // 随机打乱顺序
        for (uint32_t j = 0; j < input->ioVec.len; ++j)
        {
            uint32_t k = rand() % input->ioVec.len;
            uint32_t temp = population[i][j];
            population[i][j] = population[i][k];
            population[i][k] = temp;
        }
    }

    uint32_t bestSequence[input->ioVec.len];
    int32_t bestCost = INT32_MAX;

    // 遗传算法过程
    for (int iteration = 0; iteration < maxIterations; ++iteration)
    {
        // 计算适应度
        int32_t fitness[populationSize];
        for (int i = 0; i < populationSize; ++i)
        {
            OutputParam tempOutput;
            tempOutput.len = input->ioVec.len;
            tempOutput.sequence = (uint32_t *)malloc(input->ioVec.len * sizeof(uint32_t));
            if (tempOutput.sequence == NULL)
            {
                return RETURN_ERROR;
            }
            for (uint32_t j = 0; j < tempOutput.len; j++)
            {
                tempOutput.sequence[j] = population[i][j];
            }
            AccessTime accessTime;
            TotalAccessTime(input, &tempOutput, &accessTime);
            fitness[i] = accessTime.addressDuration + accessTime.readDuration;

            if (fitness[i] < bestCost)
            {
                bestCost = fitness[i];
                for (uint32_t j = 0; j < input->ioVec.len; ++j)
                {
                    bestSequence[j] = population[i][j];
                }
            }
            free(tempOutput.sequence);
        }

        // 选择
        uint32_t newPopulation[populationSize][input->ioVec.len];
        for (int i = 0; i < populationSize; ++i)
        {
            int parent1 = rand() % populationSize;
            int parent2 = rand() % populationSize;
            int betterParent = (fitness[parent1] < fitness[parent2]) ? parent1 : parent2;
            for (uint32_t j = 0; j < input->ioVec.len; ++j)
            {
                newPopulation[i][j] = population[betterParent][j];
            }
        }

        // 交叉
        for (int i = 0; i < populationSize; i += 2)
        {
            if (((double)rand() / RAND_MAX) < crossoverRate)
            {
                uint32_t crossoverPoint = rand() % input->ioVec.len;
                uint32_t child1[input->ioVec.len];
                uint32_t child2[input->ioVec.len];
                int used1[input->ioVec.len];
                int used2[input->ioVec.len];
                memset(used1, 0, sizeof(used1));
                memset(used2, 0, sizeof(used2));

                for (uint32_t j = 0; j < crossoverPoint; ++j)
                {
                    child1[j] = newPopulation[i][j];
                    child2[j] = newPopulation[i + 1][j];
                    used1[child1[j]] = 1;
                    used2[child2[j]] = 1;
                }

                uint32_t index1 = crossoverPoint;
                uint32_t index2 = crossoverPoint;
                for (uint32_t j = 0; j < input->ioVec.len; ++j)
                {
                    if (!used1[newPopulation[i + 1][j]])
                    {
                        child1[index1++] = newPopulation[i + 1][j];
                        used1[newPopulation[i + 1][j]] = 1;
                    }
                    if (!used2[newPopulation[i][j]])
                    {
                        child2[index2++] = newPopulation[i][j];
                        used2[newPopulation[i][j]] = 1;
                    }
                }

                for (uint32_t j = 0; j < input->ioVec.len; ++j)
                {
                    newPopulation[i][j] = child1[j];
                    newPopulation[i + 1][j] = child2[j];
                }
            }
        }

        // 变异
        for (int i = 0; i < populationSize; ++i)
        {
            if (((double)rand() / RAND_MAX) < mutationRate)
            {
                uint32_t point1 = rand() % input->ioVec.len;
                uint32_t point2 = rand() % input->ioVec.len;
                uint32_t temp = newPopulation[i][point1];
                newPopulation[i][point1] = newPopulation[i][point2];
                newPopulation[i][point2] = temp;
            }
        }

        // 更新种群
        for (int i = 0; i < populationSize; ++i)
        {
            for (uint32_t j = 0; j < input->ioVec.len; ++j)
            {
                population[i][j] = newPopulation[i][j];
            }
        }
    }

    // 将最优解复制到输出参数
    output->len = input->ioVec.len;
    output->sequence = (uint32_t *)malloc(output->len * sizeof(uint32_t));
    if (output->sequence == NULL)
    {
        return RETURN_ERROR;
    }
    for (uint32_t i = 0; i < output->len; i++)
    {
        output->sequence[i] = bestSequence[i];
    }

    return RETURN_OK;
}

int32_t AlgorithmRun(const InputParam *input, OutputParam *output, char *algorithm)
{
    int32_t ret;

    if (strcmp(algorithm, "FCFS") == 0)
    {
        ret = IOScheduleAlgorithm(input, output);
    }
    else if (strcmp(algorithm, "Nearest") == 0)
    {
        ret = NearestNeighborAlgorithm(input, output);
    }
    else if (strcmp(algorithm, "SA") == 0)
    {
        ret = SimulatedAnnealing(input, output);
    }
    else if (strcmp(algorithm, "TS") == 0)
    {
        ret = TabuSearch(input, output);
    }
    else if (strcmp(algorithm, "HC") == 0)
    {
        ret = HillClimbing(input, output);
    }
    else if (strcmp(algorithm, "GA") == 0)
    {
        // ret = GeneticAlgorithm(input, output);
        ret = IOScheduleAlgorithm(input, output);
    }
    else
    {
        printf("Invalid algorithm name.\n");
        return RETURN_ERROR;
    }

    // for (uint32_t i = 0; i < output->len; i++)
    // {
    //     printf("%d ", output->sequence[i]);
    // }
    // printf("\n");

    return RETURN_OK;
}
