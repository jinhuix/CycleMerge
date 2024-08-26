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
        // 从栈中弹出 high 和 low 值，表示当前需要排序的子数组的边界
        high = stack[top--];
        low = stack[top--];

        // 选择子数组的最后一个元素作为枢轴（pivot），并初始化变量 i 为 low - 1
        uint32_t pivot = sortedIOs[high].startLpos;
        int i = low - 1;

        // 遍历当前子数组
        for (int j = low; j < high; ++j)
        {
            // 将所有小于枢轴的元素移到枢轴的左边
            if (sortedIOs[j].startLpos < pivot)
            {
                ++i; // i 指向当前小于枢轴的元素的位置
                IOUint temp = sortedIOs[i];
                sortedIOs[i] = sortedIOs[j];
                sortedIOs[j] = temp;
            }
        }

        // 将枢轴元素放到正确的位置
        IOUint temp = sortedIOs[i + 1];
        sortedIOs[i + 1] = sortedIOs[high];
        sortedIOs[high] = temp;

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

    for (int i = 0; i < input->ioVec.len; i++)
    {
        // printf("%d ", sortedIOs[i].startLpos);
        if (i + 1 == input->ioVec.len)
            break;
        if (sortedIOs[i + 1].startLpos < sortedIOs[i].startLpos)
        {
            printf("sort error!\n");
            abort();
        }
    }

    // 初始化当前头位置为输入的头状态
    // HeadInfo currentHead = {input->headInfo.wrap, input->headInfo.lpos, input->headInfo.status};
    HeadInfo currentHead = {0, 0, 0};

    // 扫描方向：1 表示从 BOT 向 EOT 扫描，-1 表示从 EOT 向 BOT 扫描
    int direction = 1;
    uint32_t index = 0;

    bool vis[input->ioVec.len + 1];
    memset(vis, 0, sizeof(vis));
    while (index < input->ioVec.len)
    {
        if (direction == 1)
        {
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
                    currentHead.wrap = sortedIOs[i].wrap;
                    // currentHead.lpos = sortedIOs[i].endLpos;
                    currentHead.lpos = sortedIOs[i].startLpos;
                }
            }
            direction = -1; // 改变扫描方向
            currentHead.lpos = MAX_LPOS;
        }
        else
        {
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
                    currentHead.wrap = sortedIOs[i].wrap;
                    // currentHead.lpos = sortedIOs[i].endLpos;
                    currentHead.lpos = sortedIOs[i].startLpos;
                }
            }
            direction = 1; // 改变扫描方向
            currentHead.lpos = 0;
        }
    }

    free(sortedIOs);
    return RETURN_OK;
}

int32_t SCAN2(const InputParam *input, OutputParam *output)
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
        // 从栈中弹出 high 和 low 值，表示当前需要排序的子数组的边界
        high = stack[top--];
        low = stack[top--];

        // 选择子数组的最后一个元素作为枢轴（pivot），并初始化变量 i 为 low - 1
        uint32_t pivot = sortedIOs[high].startLpos;
        int i = low - 1;

        // 遍历当前子数组
        for (int j = low; j < high; ++j)
        {
            // 将所有小于枢轴的元素移到枢轴的左边
            if (sortedIOs[j].startLpos < pivot)
            {
                ++i; // i 指向当前小于枢轴的元素的位置
                IOUint temp = sortedIOs[i];
                sortedIOs[i] = sortedIOs[j];
                sortedIOs[j] = temp;
            }
        }

        // 将枢轴元素放到正确的位置
        IOUint temp = sortedIOs[i + 1];
        sortedIOs[i + 1] = sortedIOs[high];
        sortedIOs[high] = temp;

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

    for (int i = 0; i < input->ioVec.len; i++)
    {
        // printf("%d ", sortedIOs[i].startLpos);
        if (i + 1 == input->ioVec.len)
            break;
        if (sortedIOs[i + 1].startLpos < sortedIOs[i].startLpos)
        {
            printf("sort error!\n");
            abort();
        }
    }

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
                    // currentHead.lpos = sortedIOs[i].startLpos;
                }
            }
            direction = -1; // 改变扫描方向

            // if (last_cnt == 0)
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
                    // currentHead.lpos = sortedIOs[i].startLpos;
                }
            }
            direction = 1; // 改变扫描方向

            // if (last_cnt == 0)
            currentHead.lpos = 0;
            last_cnt = cur_cnt;
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
    double initialTemperature = 1000;
    double coolingRate = 0.995;
    double minTemperature = 1;
    int maxIterations = 10000000;

    NearestNeighborAlgorithm(input, output);

    AccessTime accessTime;
    TotalAccessTime(input, output, &accessTime);
    int32_t currentCost = accessTime.addressDuration + accessTime.readDuration;
    int32_t bestCost = currentCost;
    uint32_t bestSequence[input->ioVec.len];
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
    {
        bestSequence[i] = output->sequence[i];
    }

    SCAN(input, output);
    TotalAccessTime(input, output, &accessTime);
    if (accessTime.addressDuration + accessTime.readDuration < bestCost)
    {
        bestCost = accessTime.addressDuration + accessTime.readDuration;
        for (uint32_t i = 0; i < input->ioVec.len; ++i)
        {
            bestSequence[i] = output->sequence[i];
        }
    }

    OutputParam *temp_output;
    temp_output = (OutputParam *)malloc(sizeof(OutputParam));
    temp_output->len = input->ioVec.len;
    temp_output->sequence = (uint32_t *)malloc(input->ioVec.len * sizeof(uint32_t));

    // 模拟退火过程
    double temperature = initialTemperature;
    int iteration = 0;

    while (temperature > minTemperature && iteration < maxIterations)
    {
        // 生成邻域解
        for (uint32_t i = 0; i < input->ioVec.len; ++i)
        {
            temp_output->sequence[i] = output->sequence[i];
        }

        double randValue = (double)rand() / RAND_MAX;
        if (randValue < 0.33)
        {
            // Swap two nodes
            uint32_t i = rand() % input->ioVec.len;
            uint32_t j = rand() % input->ioVec.len;
            uint32_t temp = temp_output->sequence[i];
            temp_output->sequence[i] = temp_output->sequence[j];
            temp_output->sequence[j] = temp;
        }
        else if (randValue < 0.66)
        {
            // Reverse a sub-path
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
            // Insert a node
            uint32_t from = rand() % input->ioVec.len;
            uint32_t to = rand() % input->ioVec.len;
            uint32_t temp = temp_output->sequence[from];
            if (from < to)
            {
                for (uint32_t i = from; i < to; ++i)
                {
                    temp_output->sequence[i] = temp_output->sequence[i + 1];
                }
            }
            else
            {
                for (uint32_t i = from; i > to; --i)
                {
                    temp_output->sequence[i] = temp_output->sequence[i - 1];
                }
            }
            temp_output->sequence[to] = temp;
        }

        // 计算邻域解的总时延
        TotalAccessTime(input, temp_output, &accessTime);
        int32_t neighborCost = accessTime.addressDuration + accessTime.readDuration;

        // 接受邻域解的条件
        if (neighborCost < currentCost || ((double)rand() / RAND_MAX) < exp((currentCost - neighborCost) / temperature))
        {
            for (uint32_t i = 0; i < input->ioVec.len; ++i)
            {
                output->sequence[i] = temp_output->sequence[i];
            }
            currentCost = neighborCost;

            if (currentCost < bestCost)
            {
                for (uint32_t i = 0; i < input->ioVec.len; ++i)
                {
                    bestSequence[i] = output->sequence[i];
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

    free(temp_output->sequence);
    free(temp_output);

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
    int maxIterations = 1000;

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
    int improved = 1, iteration = 0;

    while (improved)
    {
        iteration++;
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

typedef struct
{
    int x, y, dis;
} Node;

typedef struct
{
    Node *nodes;
    int size;
    int capacity;
} MinHeap;

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

Node *extractMin(MinHeap *heap) // 弹出最小值
{
    if (heap->size == 0)
    {
        return NULL;
    }

    // Node root = heap->nodes[0];
    // heap->nodes[0] = heap->nodes[heap->size - 1];
    swap(&heap->nodes[0], &heap->nodes[heap->size - 1]);
    heap->size--;
    heapify(heap, 0);

    return &heap->nodes[heap->size];
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

typedef struct
{
    int id;
    struct req_node *next;
} req_node;

typedef struct
{
    req_node *head;
    req_node *tail;
    int len;
} req_list;

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

int32_t merge(const InputParam *input, OutputParam *output)
{
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
    {
        output->sequence[i] = input->ioVec.ioArray[i].id;
    }

    // int dis[maxn][maxn];

    // int **dis;
    // // 分配指针数组
    // dis = (int **)malloc(maxn * sizeof(int *));
    // if (dis == NULL)
    // {
    //     perror("Failed to allocate memory");
    //     exit(EXIT_FAILURE);
    // }

    // // 分配每一行的数组
    // for (int i = 0; i < maxn; i++)
    // {
    //     dis[i] = (int *)malloc(maxn * sizeof(int));
    //     if (dis[i] == NULL)
    //     {
    //         perror("Failed to allocate memory");
    //         exit(EXIT_FAILURE);
    //     }
    // }

    MinHeap *heap = createMinHeap(maxn * maxn);

    // 初始化当前头位置为输入的头状态
    HeadInfo currentHead = {input->headInfo.wrap, input->headInfo.lpos, input->headInfo.status};
    for (int i = 0; i < input->ioVec.len; i++)
    {
        HeadInfo status_tmp = {input->ioVec.ioArray[i].wrap, input->ioVec.ioArray[i].startLpos, HEAD_RW};
        insertHeap(heap, (Node){0, i + 1, SeekTimeCalculate(&currentHead, &status_tmp)});
        for (int j = 0; j < input->ioVec.len; j++)
        {
            if (i == j)
                continue;
            // printf("i=%d j=%d\n", i, j);
            HeadInfo status1 = {input->ioVec.ioArray[i].wrap, input->ioVec.ioArray[i].endLpos, HEAD_RW};
            HeadInfo status2 = {input->ioVec.ioArray[j].wrap, input->ioVec.ioArray[j].startLpos, HEAD_RW};

            insertHeap(heap, (Node){i + 1, j + 1, SeekTimeCalculate(&status1, &status2)});
            // dis[i][j] = SeekTimeCalculate(&status1, &status2);
            // printf("wrap=%d lpos=%d status=%d\n", status1.wrap, status1.lpos, status1.status);
            // printf("wrap=%d lpos=%d status=%d\n", status2.wrap, status2.lpos, status2.status);
            // printf("i=%d j=%d dis=%d\n", i, j, dis[i][j]);
        }
    }

    int nex[maxn], vis[maxn];
    memset(nex, 0, sizeof(nex));
    memset(vis, 0, sizeof(vis));
    initUnionSet();
    while (heap->size)
    {
        if (sz[0] == input->ioVec.len + 1)
            break;
        // printf("size=%d\n", heap->size);
        Node *node = extractMin(heap);
        // printf("x=%d y=%d dis=%d\n", node->x, node->y, node->dis);
        // printf("nex[%d]=%d\n", node->x, nex[node->x]);
        if (nex[node->x] == 0 && nex[node->y] != node->x && vis[node->y] == 0)
        {
            if (find(node->x) == find(node->y))
                continue;
            unite(node->x, node->y);
            nex[node->x] = node->y;
            vis[node->y] = 1;
        }
    }

    // for (int i = 0; i <= input->ioVec.len; i++)
    // {
    //     printf("nex[%d]=%d\n", i, nex[i]);
    // }

    // printf("sz[0]=%d\n", sz[0]);
    int now = nex[0], cnt = 0;
    while (cnt < input->ioVec.len)
    {
        output->sequence[cnt++] = now;
        // printf("cnt=%d now=%d\n", cnt, now);
        now = nex[now];
    }

    // free(dis);
    return RETURN_OK;
}

int32_t AlgorithmRun(const InputParam *input, OutputParam *output, char *algorithm)
{
    int32_t ret;

    if (strcmp(algorithm, "FCFS") == 0)
    {
        ret = IOScheduleAlgorithm(input, output);
    }
    else if (strcmp(algorithm, "SCAN") == 0)
    {
        ret = SCAN(input, output);
    }
    else if (strcmp(algorithm, "SCAN2") == 0)
    {
        ret = SCAN2(input, output);
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
        // ret = TabuSearch(input, output);
        ret = IOScheduleAlgorithm(input, output);
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
    else if (strcmp(algorithm, "merge") == 0)
    {
        ret = merge(input, output);
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
