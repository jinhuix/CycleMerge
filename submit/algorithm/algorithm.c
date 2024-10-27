#include "algorithm.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void startRecordTime(){
    gettimeofday(&g_TimeRecord.start, NULL);
}

int32_t getDurationMicroseconds(){
    struct timeval end;
    gettimeofday(&end, NULL);
    return (end.tv_sec - g_TimeRecord.start.tv_sec) * 1000000 + end.tv_usec - g_TimeRecord.start.tv_usec;
}

/**
 * @brief  算法接口
 * @param  input            输入参数
 * @param  output           输出参数
 * @return int32_t          返回成功或者失败，RETURN_OK 或 RETURN_ERROR
 */
int32_t IOScheduleAlgorithm(const InputParam *input, OutputParam *output)
{
    int min_time = 0x3f3f3f3f;
    int *best_sequence = (int *)malloc(input->ioVec.len * sizeof(int));
    if (input->ioVec.len > 1000)
    {
        // int flag = 1;
        AccessTime accessTime = {0};
        merge(input, output);
        TotalAccessTime(input, output, &accessTime);
        if (accessTime.addressDuration < min_time)
        {
            min_time = accessTime.addressDuration;
            memcpy(best_sequence, output->sequence, input->ioVec.len * sizeof(int));
            // flag = 1;
        }
        merge_random(input, output);
        TotalAccessTime(input, output, &accessTime);
        if (accessTime.addressDuration < min_time)
        {
            min_time = accessTime.addressDuration;
            memcpy(best_sequence, output->sequence, input->ioVec.len * sizeof(int));
            // flag = 2;
        }
        memcpy(output->sequence, best_sequence, input->ioVec.len * sizeof(int));
        // printf("flag=%d\n", flag);
    }
    else
    {
        // int flag = 3;
        partition_scan(input, output);
        AccessTime accessTime = {0};
        TotalAccessTime(input, output, &accessTime);
        if (accessTime.addressDuration < min_time)
        {
            min_time = accessTime.addressDuration;
            memcpy(best_sequence, output->sequence, input->ioVec.len * sizeof(int));
            // flag = 3;
        }
        MPScan(input, output);
        TotalAccessTime(input, output, &accessTime);
        if (accessTime.addressDuration < min_time)
        {
            min_time = accessTime.addressDuration;
            memcpy(best_sequence, output->sequence, input->ioVec.len * sizeof(int));
            // flag = 4;
        }
        merge(input, output);
        TotalAccessTime(input, output, &accessTime);
        if (accessTime.addressDuration < min_time)
        {
            min_time = accessTime.addressDuration;
            memcpy(best_sequence, output->sequence, input->ioVec.len * sizeof(int));
            // flag = 5;
        }
        merge_random(input, output);
        TotalAccessTime(input, output, &accessTime);
        if (accessTime.addressDuration < min_time)
        {
            min_time = accessTime.addressDuration;
            memcpy(best_sequence, output->sequence, input->ioVec.len * sizeof(int));
            // flag = 6;
        }
        memcpy(output->sequence, best_sequence, input->ioVec.len * sizeof(int));
        // printf("flag=%d\n", flag);
    }

    return RETURN_OK;
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

    ret = operator_optimization(input, output);

    duration_us = getDurationMicroseconds();

    // printf("duration_us=%d\n", duration_us);

    return RETURN_OK;
}

int32_t operator_optimization(const InputParam *input, OutputParam *output)
{
    SimpleOperatorOptimization(input, output);
    return RETURN_OK;
}

int32_t SimpleOperatorOptimization(const InputParam *input, OutputParam *output)
{
    /* Step2：通过算子优化，微调ouput的顺序，事例 */
    // output->len = input->ioVec.len;
    // for (uint32_t i = 0; i < output->len; i++)
    // {
    //     output->sequence[i] = input->ioVec.ioArray[i].id;
    // }
    return RETURN_OK;
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
        Node min_node = {0, i + 1, SeekTimeCalculate(&currentHead, &status_tmp)};

        for (int j = 0; j < input->ioVec.len; j++)
        {
            if (i == j)
                continue;
            HeadInfo status1 = {input->ioVec.ioArray[j].wrap, input->ioVec.ioArray[j].endLpos, HEAD_RW};

            Node tmp_node = {j + 1, i + 1, SeekTimeCalculate(&status1, &status_tmp)};
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
        Node *node = extractMin(heap);

        if (nex[node->x] == 0 && (node->x == 0 || (nex[node->y] != node->x)) && vis[node->y] == 0 && find(node->x) != find(node->y))
        {
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
                Node tmp_node = {0, target_id, SeekTimeCalculate(&currentHead, &status_tmp)};
                min_node = tmp_node;
            }
            for (int source_id = 1; source_id < input->ioVec.len + 1; source_id++)
            {
                if (target_id == source_id || nex[source_id] != 0 || find(source_id) == find(target_id))
                    continue;
                HeadInfo status1 = {input->ioVec.ioArray[source_id - 1].wrap, input->ioVec.ioArray[source_id - 1].endLpos, HEAD_RW};
                Node tmp_node = {source_id, target_id, SeekTimeCalculate(&status1, &status_tmp)};
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
        Node min_node = {0, i + 1, SeekTimeCalculate(&currentHead, &status_tmp)};

        for (int j = 0; j < input->ioVec.len; j++)
        {
            if (i == j)
                continue;
            HeadInfo status1 = {input->ioVec.ioArray[j].wrap, input->ioVec.ioArray[j].endLpos, HEAD_RW};

            Node tmp_node = {j + 1, i + 1, SeekTimeCalculate(&status1, &status_tmp)};
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
                Node tmp_node = {0, target_id, SeekTimeCalculate(&currentHead, &status_tmp)};
                min_node = tmp_node;
            }
            for (int source_id = 1; source_id < input->ioVec.len + 1; source_id++)
            {
                if (target_id == source_id || nex[source_id] != 0 || find(source_id) == find(target_id))
                    continue;
                HeadInfo status1 = {input->ioVec.ioArray[source_id - 1].wrap, input->ioVec.ioArray[source_id - 1].endLpos, HEAD_RW};
                Node tmp_node = {source_id, target_id, SeekTimeCalculate(&status1, &status_tmp)};
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