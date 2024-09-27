#include "test_algorithms.h"

#include "interface.h"
#include "logging.h"
#include "operator_optimization.h"

const int maxn = 10001;
int *fa, *h, *sz;

/**
 * @brief  算法接口
 * @param  input            输入参数
 * @param  output           输出参数
 * @return int32_t          返回成功或者失败，RETURN_OK 或 RETURN_ERROR
 */
int32_t IOScheduleAlgorithm(const InputParam *input, OutputParam *output) {
    int32_t ret;

    /* Step1：先入先出算法 */
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++) {
        output->sequence[i] = input->ioVec.ioArray[i].id;
    }

    return RETURN_OK;
}

// 快速排序
void QuickSort(IOUint *a, int len) {
    int low = 0;
    int high = len - 1;
    int stack[high - low + 1];
    int top = -1;
    stack[++top] = low;
    stack[++top] = high;

    while (top >= 0) {
        // 从栈中弹出 high 和 low 值，表示当前需要排序的子数组的边界
        high = stack[top--];
        low = stack[top--];

        // 选择子数组的最后一个元素作为枢轴（pivot），并初始化变量 i 为 low - 1
        uint32_t pivot = a[high].startLpos;
        int i = low - 1;

        // 遍历当前子数组
        for (int j = low; j < high; ++j) {
            // 将所有小于枢轴的元素移到枢轴的左边
            if (a[j].startLpos < pivot) {
                ++i;  // i 指向当前小于枢轴的元素的位置
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
        if (pi - 1 > low) {
            stack[++top] = low;
            stack[++top] = pi - 1;
        }
        if (pi + 1 < high) {
            stack[++top] = pi + 1;
            stack[++top] = high;
        }
    }
}

int32_t MPScan(const InputParam *input, OutputParam *output) {
    SCAN2(input, output);

    // 将最后一轮扫描插入前面
    OutputParam *tmp = (OutputParam *)malloc(sizeof(OutputParam));
    tmp->len = input->ioVec.len;
    tmp->sequence = (uint32_t *)malloc(input->ioVec.len * sizeof(uint32_t));
    memcpy(tmp->sequence, output->sequence, input->ioVec.len * sizeof(int));
    AccessTime accessTime;
    TotalAccessTime(input, output, &accessTime);
    while (true) {
        // 最后一轮扫描在 output 中的下标范围为 [idx+1, output->len - 1]
        int32_t io_len = 0, idx = output->len - 1;
        while (idx >= 1 && (input->ioVec.ioArray[output->sequence[idx] - 1].wrap & 1) &&
               input->ioVec.ioArray[output->sequence[idx] - 1].startLpos < input->ioVec.ioArray[output->sequence[idx - 1] - 1].startLpos) {  // 奇数
            idx--;
        }
        while (idx >= 1 && !(input->ioVec.ioArray[output->sequence[idx] - 1].wrap & 1) &&
               input->ioVec.ioArray[output->sequence[idx] - 1].startLpos > input->ioVec.ioArray[output->sequence[idx - 1] - 1].startLpos) {  // 偶数
            idx--;
        }
        if (idx < 0)
            break;  // 当前已经是最后一轮扫描

        // printf("\n%d:", output->sequence[idx]);
        // 遍历最后一轮的每个 IO
        for (int i = idx; i < output->len; ++i) {
            // printf("\ni = %d , ", output->sequence[i]);
            int32_t minTime = INT32_MAX;
            int32_t best_pos = -1;
            HeadInfo z = {input->ioVec.ioArray[output->sequence[i] - 1].wrap, input->ioVec.ioArray[output->sequence[i] - 1].startLpos, HEAD_RW};

            // 寻找插入的最佳位置
            for (int j = 0; j < i - 1; ++j) {
                // if(input->ioVec.ioArray[output->sequence[j]-1].wrap != input->ioVec.ioArray[output->sequence[i]-1].wrap)
                //     continue;
                HeadInfo x = {input->ioVec.ioArray[output->sequence[j] - 1].wrap, input->ioVec.ioArray[output->sequence[j] - 1].startLpos, HEAD_RW};
                HeadInfo y = {input->ioVec.ioArray[output->sequence[j + 1] - 1].wrap, input->ioVec.ioArray[output->sequence[j + 1] - 1].startLpos, HEAD_RW};
                int32_t seekTime = SeekTimeCalculate(&x, &z) + SeekTimeCalculate(&z, &y) - SeekTimeCalculate(&x, &y);
                // printf("seekTime = %d, minTime = %d ", seekTime, minTime);
                if (seekTime < minTime) {
                    best_pos = j;
                    minTime = seekTime;
                }
            }

            // printf("best_pos = %d , ", best_pos);
            // 将当前 IO 插入到 best_pos 后面
            for (int j = i; j > best_pos + 1; --j) {
                tmp->sequence[j] = tmp->sequence[j - 1];
            }
            tmp->sequence[best_pos + 1] = output->sequence[i];  // 更新该处的 IO 序号
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

        if (tmpTime.addressDuration < accessTime.addressDuration) {
            accessTime.addressDuration = tmpTime.addressDuration;
            memcpy(output->sequence, tmp->sequence, input->ioVec.len * sizeof(int));
        } else {
            break;
        }
    }

    return RETURN_OK;
}

int32_t MPScanPartition(const InputParam *input, OutputParam *output) {
    // DEBUG("进入 MPScanPartition\n");
    // 复制 IO 请求数组并按 lpos 排序
    IOUint *sortedIOs = (IOUint *)malloc(input->ioVec.len * sizeof(IOUint));
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
        sortedIOs[i] = input->ioVec.ioArray[i];
    QuickSort(sortedIOs, input->ioVec.len);
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
        output->sequence[i] = sortedIOs[i].id;

    //----搜索最佳分割参数----
    int min_time = 0x3f3f3f3f, best_partition_size = 5000;
    int *best_sequence = (int *)malloc(input->ioVec.len * sizeof(int));
    bool *vis = (bool *)malloc((input->ioVec.len + 1) * sizeof(bool));
    int partition_io_num[1000] = {0};    // 存储每个分区中的IO数量
    int partition_io_start[1000] = {0};  // 存储每个分区起始IO的索引

    // 按固定长度进行分区，对长度参数进行搜索
    for (int i = 5000; i <= 740000; i += 5000) {
        int partition_start_now = 0;  // 当前分区的起始位置
        int now = 0;                  // 当前分区索引
        int partition_threshold = i;  // 当前分区长度
        memset(partition_io_start, 0, sizeof(partition_io_start));
        memset(partition_io_num, 0, sizeof(partition_io_num));
        for (int j = 0; j < input->ioVec.len; j++) {
            // DEBUG("\nj=%d startLpos=%d\n", j, sortedIOs[j].startLpos);
            if (sortedIOs[j].startLpos >= partition_start_now + partition_threshold) {
                while (sortedIOs[j].startLpos >= partition_start_now + partition_threshold) {
                    partition_start_now += partition_threshold;
                    now++;
                }
                partition_io_start[now] = j;
            }
            // DEBUG("partition_start_now=%d now=%d\n", partition_start_now, now);
            partition_io_num[now]++;
        }

        // DEBUG("partition_len=%d, partition_num=%d\n", i, now);
        // for (int j = 0; j <= now; j++) {
        //     if (partition_io_num[j] != 0)
        //         DEBUG("partition %d start at %d, io_num=%d\n", j, partition_io_start[j], partition_io_num[j]);
        // }

        for (int j = 0; j < input->ioVec.len + 1; j++)
            vis[j] = 0;
        HeadInfo head = input->headInfo;
        for (int j = 0; j <= now; j++) {
            // DEBUG("partition %d start at %d, io_num=%d\n", j, partition_io_start[j], partition_io_num[j]);
            if (partition_io_num[j] == 0)
                continue;
            // for (int j = 0; j < input->ioVec.len; j++)
            //     printf("vis[%d]=%d\n", sortedIOs[j].id, vis[sortedIOs[j].id]);
            // printf("\n");
            MPScanPerPartition(input, output, sortedIOs, vis, &head, partition_io_start[j], partition_io_start[j] + partition_io_num[j]);
            head.wrap = input->ioVec.ioArray[output->sequence[partition_io_start[j] + partition_io_num[j] - 1] - 1].wrap;
            head.lpos = input->ioVec.ioArray[output->sequence[partition_io_start[j] + partition_io_num[j] - 1] - 1].endLpos;
        }
        AccessTime accessTime = {0};
        TotalAccessTime(input, output, &accessTime);
        if (accessTime.addressDuration <= min_time) {
            best_partition_size = i;
            min_time = accessTime.addressDuration;
            for (int j = 0; j < input->ioVec.len; j++)
                best_sequence[j] = output->sequence[j];
        }
    }
    printf("best_partition_size=%d\n", best_partition_size);
    for (int i = 0; i < input->ioVec.len; i++)
        output->sequence[i] = best_sequence[i];

    free(best_sequence);
    free(sortedIOs);
}

// 用MPScan处理每个分区内部
int32_t MPScanPerPartition(const InputParam *input, OutputParam *output, IOUint *sortedIOs, bool *vis, HeadInfo *head, int partition_start, int partition_end) {
    uint32_t index = partition_start;
    HeadInfo currentHead = *head;
    int direction = 1;  // 扫描方向：1 表示从 BOT 向 EOT 扫描，-1 表示从 EOT 向 BOT 扫描

    // DEBUG("partition_start=%d, partition_end=%d\n", partition_start, partition_end);
    for (int i = partition_start; i < partition_end; i++) {
        if (vis[sortedIOs[i].id])
            ERROR("vis[%d]=1\n", sortedIOs[i].id);
    }

    while (index < partition_end) {
        if (direction == 1) {
            // 从 BOT 向 EOT 扫描
            for (uint32_t i = partition_start; i < partition_end; ++i) {
                if (sortedIOs[i].wrap & 1 || vis[sortedIOs[i].id])
                    continue;
                if (sortedIOs[i].startLpos > currentHead.lpos || currentHead.lpos == 0) {
                    output->sequence[index++] = sortedIOs[i].id;
                    vis[sortedIOs[i].id] = 1;
                    currentHead.wrap = sortedIOs[i].wrap;
                    currentHead.lpos = sortedIOs[i].endLpos;
                }
            }
            direction = -1;                   // 改变扫描方向
            currentHead.lpos = MAX_LPOS + 1;  // 赋值为最大值以保证下一轮扫描中大于当前位置的请求不会被忽略
        } else {
            // 从 EOT 向 BOT 扫描
            for (int32_t i = partition_end - 1; i >= partition_start; --i) {
                if (!(sortedIOs[i].wrap & 1) || vis[sortedIOs[i].id])
                    continue;
                if (sortedIOs[i].startLpos < currentHead.lpos || currentHead.lpos == 0) {
                    output->sequence[index++] = sortedIOs[i].id;
                    vis[sortedIOs[i].id] = 1;
                    currentHead.wrap = sortedIOs[i].wrap;
                    currentHead.lpos = sortedIOs[i].endLpos;
                }
            }
            direction = 1;         // 改变扫描方向
            currentHead.lpos = 0;  // 赋值为0以保证下一轮扫描中小于当前位置的请求不会被忽略
        }
    }

    // 将最后一轮扫描插入前面
    OutputParam *tmp = (OutputParam *)malloc(sizeof(OutputParam));
    tmp->len = input->ioVec.len;
    tmp->sequence = (uint32_t *)malloc(input->ioVec.len * sizeof(uint32_t));
    memcpy(tmp->sequence, output->sequence, input->ioVec.len * sizeof(int));
    AccessTime accessTime;
    TotalAccessTime(input, output, &accessTime);
    while (true) {
        // 最后一轮扫描在 output 中的下标范围为 [idx+1, output->len - 1]
        int32_t idx = partition_end - 1;
        if (input->ioVec.ioArray[output->sequence[idx] - 1].wrap & 1)
            while (idx > partition_start && (input->ioVec.ioArray[output->sequence[idx - 1] - 1].wrap & 1) &&
                   input->ioVec.ioArray[output->sequence[idx] - 1].startLpos < input->ioVec.ioArray[output->sequence[idx - 1] - 1].startLpos) {  // 奇数
                idx--;
            }
        if (!(input->ioVec.ioArray[output->sequence[idx] - 1].wrap & 1))
            while (idx > partition_start && !(input->ioVec.ioArray[output->sequence[idx - 1] - 1].wrap & 1) &&
                   input->ioVec.ioArray[output->sequence[idx] - 1].startLpos > input->ioVec.ioArray[output->sequence[idx - 1] - 1].startLpos) {  // 偶数
                idx--;
            }
        if (idx == partition_start)
            break;  // 当前已经是最后一轮扫描

        // printf("\n%d:", output->sequence[idx]);
        // 遍历最后一轮的每个 IO
        for (int i = idx; i < partition_end; ++i) {
            // printf("\ni = %d , ", output->sequence[i]);
            int32_t minTime = INT32_MAX;
            int32_t best_pos = -1;
            HeadInfo z = {input->ioVec.ioArray[output->sequence[i] - 1].wrap, input->ioVec.ioArray[output->sequence[i] - 1].startLpos, HEAD_RW};

            // 寻找插入的最佳位置
            for (int j = partition_start; j < i - 1; ++j) {
                HeadInfo x = {input->ioVec.ioArray[output->sequence[j] - 1].wrap, input->ioVec.ioArray[output->sequence[j] - 1].startLpos, HEAD_RW};
                HeadInfo y = {input->ioVec.ioArray[output->sequence[j + 1] - 1].wrap, input->ioVec.ioArray[output->sequence[j + 1] - 1].startLpos, HEAD_RW};
                int32_t seekTime = SeekTimeCalculate(&x, &z) + SeekTimeCalculate(&z, &y) - SeekTimeCalculate(&x, &y);
                if (seekTime < minTime) {
                    best_pos = j;
                    minTime = seekTime;
                }
            }

            // printf("best_pos = %d , ", best_pos);
            if (best_pos == -1) {
                continue;
            }
            // 将当前 IO 插入到 best_pos 后面
            for (int j = i; j > best_pos + 1; --j)
                tmp->sequence[j] = tmp->sequence[j - 1];
            tmp->sequence[best_pos + 1] = output->sequence[i];  // 更新该处的 IO 序号
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

        if (tmpTime.addressDuration < accessTime.addressDuration) {
            accessTime.addressDuration = tmpTime.addressDuration;
            memcpy(output->sequence, tmp->sequence, input->ioVec.len * sizeof(int));
        } else {
            break;
        }
    }

    return RETURN_OK;
}

int32_t SORT(const InputParam *input, OutputParam *output) {
    // 初始化输出参数
    output->len = input->ioVec.len;

    // 复制 IO 请求数组并按 lpos 排序
    IOUint *sortedIOs = (IOUint *)malloc(input->ioVec.len * sizeof(IOUint));
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
        sortedIOs[i] = input->ioVec.ioArray[i];

    QuickSort(sortedIOs, input->ioVec.len);

    // --- 排序完毕 ---

    // 初始化当前头位置为输入的头状态
    // HeadInfo currentHead = {input->headInfo.wrap, input->headInfo.lpos, input->headInfo.status};
    HeadInfo currentHead = {0, 0, 0};

    for (uint32_t i = 0; i < input->ioVec.len; i++) {
        output->sequence[i] = sortedIOs[i].id;
    }

    free(sortedIOs);
    return RETURN_OK;
}

int32_t SCAN(const InputParam *input, OutputParam *output) {
    // 初始化输出参数
    output->len = input->ioVec.len;

    // 复制 IO 请求数组并按 lpos 排序
    IOUint *sortedIOs = (IOUint *)malloc(input->ioVec.len * sizeof(IOUint));
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
        sortedIOs[i] = input->ioVec.ioArray[i];
    QuickSort(sortedIOs, input->ioVec.len);

    // 初始化当前头位置为输入的头状态
    // HeadInfo currentHead = {input->headInfo.wrap, input->headInfo.lpos, input->headInfo.status};
    HeadInfo currentHead = {0, 0, 0};

    // 扫描方向：1 表示从 BOT 向 EOT 扫描，-1 表示从 EOT 向 BOT 扫描
    int direction = 1;
    uint32_t index = 0;

    bool vis[input->ioVec.len + 1];
    memset(vis, 0, sizeof(vis));
    int cnt = 0;
    while (index < input->ioVec.len) {
        if (direction == 1) {
            // 从 BOT 向 EOT 扫描
            for (uint32_t i = 0; i < input->ioVec.len; ++i) {
                if (sortedIOs[i].wrap & 1 || vis[sortedIOs[i].id]) {
                    continue;
                }
                if (sortedIOs[i].startLpos >= currentHead.lpos) {
                    output->sequence[index++] = sortedIOs[i].id;
                    vis[sortedIOs[i].id] = 1;
                    currentHead.wrap = sortedIOs[i].wrap;
                    // currentHead.lpos = sortedIOs[i].endLpos;
                    currentHead.lpos = sortedIOs[i].startLpos;
                }
            }
            direction = -1;  // 改变扫描方向
            currentHead.lpos = MAX_LPOS;
            cnt++;
        } else {
            // 从 EOT 向 BOT 扫描
            for (int32_t i = input->ioVec.len - 1; i >= 0; --i) {
                if (!(sortedIOs[i].wrap & 1) || vis[sortedIOs[i].id]) {
                    continue;
                }
                if (sortedIOs[i].startLpos <= currentHead.lpos) {
                    output->sequence[index++] = sortedIOs[i].id;
                    vis[sortedIOs[i].id] = 1;
                    currentHead.wrap = sortedIOs[i].wrap;
                    // currentHead.lpos = sortedIOs[i].endLpos;
                    currentHead.lpos = sortedIOs[i].startLpos;
                }
            }
            direction = 1;  // 改变扫描方向
            currentHead.lpos = 0;
        }
    }

    free(sortedIOs);
    // DEBUG("SCAN 扫描了来回%d趟\n", cnt);
    return RETURN_OK;
}

int32_t SCAN2(const InputParam *input, OutputParam *output) {
    // 初始化输出参数
    output->len = input->ioVec.len;

    // 复制 IO 请求数组并按 lpos 排序
    IOUint *sortedIOs = (IOUint *)malloc(input->ioVec.len * sizeof(IOUint));
    for (uint32_t i = 0; i < input->ioVec.len; ++i) {
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
    while (index < input->ioVec.len) {
        if (direction == 1) {
            int cur_cnt = 0;
            // 从 BOT 向 EOT 扫描
            for (uint32_t i = 0; i < input->ioVec.len; ++i) {
                if (sortedIOs[i].wrap & 1 || vis[sortedIOs[i].id]) {
                    continue;
                }
                if (sortedIOs[i].startLpos >= currentHead.lpos) {
                    output->sequence[index++] = sortedIOs[i].id;
                    vis[sortedIOs[i].id] = 1;
                    cur_cnt++;
                    currentHead.wrap = sortedIOs[i].wrap;
                    currentHead.lpos = sortedIOs[i].endLpos;
                }
            }
            direction = -1;  // 改变扫描方向
            currentHead.lpos = MAX_LPOS;
            last_cnt = cur_cnt;
        } else {
            int cur_cnt = 0;
            // 从 EOT 向 BOT 扫描
            for (int32_t i = input->ioVec.len - 1; i >= 0; --i) {
                if (!(sortedIOs[i].wrap & 1) || vis[sortedIOs[i].id]) {
                    continue;
                }
                if (sortedIOs[i].startLpos <= currentHead.lpos) {
                    output->sequence[index++] = sortedIOs[i].id;
                    vis[sortedIOs[i].id] = 1;
                    cur_cnt++;
                    currentHead.wrap = sortedIOs[i].wrap;
                    currentHead.lpos = sortedIOs[i].endLpos;
                }
            }
            direction = 1;  // 改变扫描方向
            currentHead.lpos = 0;
            last_cnt = cur_cnt;
        }
    }

    free(sortedIOs);
    return RETURN_OK;
}

int32_t NearestNeighborAlgorithm(const InputParam *input, OutputParam *output) {
    // 初始化输出参数
    output->len = input->ioVec.len;

    // 记录哪些请求已经被处理
    bool processed[input->ioVec.len];
    for (uint32_t i = 0; i < input->ioVec.len; ++i) {
        processed[i] = false;
    }

    // 初始化当前头位置为输入的头状态
    HeadInfo currentHead = {input->headInfo.wrap, input->headInfo.lpos, input->headInfo.status};

    // 对于每一个请求，找到距离当前头位置最近的未处理请求
    for (uint32_t i = 0; i < input->ioVec.len; ++i) {
        int32_t minSeekTime = INT32_MAX;
        uint32_t nextRequestIndex = 0;

        for (uint32_t j = 0; j < input->ioVec.len; ++j) {
            if (processed[j])
                continue;

            HeadInfo nextHead = {input->ioVec.ioArray[j].wrap, input->ioVec.ioArray[j].startLpos, HEAD_RW};
            int32_t seekTime = SeekTimeCalculate(&currentHead, &nextHead);

            if (seekTime < minSeekTime) {
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

int32_t SimulatedAnnealing(const InputParam *input, OutputParam *output) {
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
    for (uint32_t i = 0; i < input->ioVec.len; ++i) {
        bestSequence[i] = output->sequence[i];
    }

    SCAN(input, output);
    TotalAccessTime(input, output, &accessTime);
    if (accessTime.addressDuration + accessTime.readDuration < bestCost) {
        bestCost = accessTime.addressDuration + accessTime.readDuration;
        for (uint32_t i = 0; i < input->ioVec.len; ++i) {
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

    while (temperature > minTemperature && iteration < maxIterations) {
        // 生成邻域解
        for (uint32_t i = 0; i < input->ioVec.len; ++i) {
            temp_output->sequence[i] = output->sequence[i];
        }

        double randValue = (double)rand() / RAND_MAX;
        if (randValue < 0.33) {
            // Swap two nodes
            uint32_t i = rand() % input->ioVec.len;
            uint32_t j = rand() % input->ioVec.len;
            uint32_t temp = temp_output->sequence[i];
            temp_output->sequence[i] = temp_output->sequence[j];
            temp_output->sequence[j] = temp;
        } else if (randValue < 0.66) {
            // Reverse a sub-path
            uint32_t start = rand() % input->ioVec.len;
            uint32_t end = rand() % input->ioVec.len;
            if (start > end) {
                uint32_t temp = start;
                start = end;
                end = temp;
            }
            while (start < end) {
                uint32_t temp = temp_output->sequence[start];
                temp_output->sequence[start] = temp_output->sequence[end];
                temp_output->sequence[end] = temp;
                start++;
                end--;
            }
        } else {
            // Insert a node
            uint32_t from = rand() % input->ioVec.len;
            uint32_t to = rand() % input->ioVec.len;
            uint32_t temp = temp_output->sequence[from];
            if (from < to) {
                for (uint32_t i = from; i < to; ++i) {
                    temp_output->sequence[i] = temp_output->sequence[i + 1];
                }
            } else {
                for (uint32_t i = from; i > to; --i) {
                    temp_output->sequence[i] = temp_output->sequence[i - 1];
                }
            }
            temp_output->sequence[to] = temp;
        }

        // 计算邻域解的总时延
        TotalAccessTime(input, temp_output, &accessTime);
        int32_t neighborCost = accessTime.addressDuration + accessTime.readDuration;

        // 接受邻域解的条件
        if (neighborCost < currentCost || ((double)rand() / RAND_MAX) < exp((currentCost - neighborCost) / temperature)) {
            for (uint32_t i = 0; i < input->ioVec.len; ++i) {
                output->sequence[i] = temp_output->sequence[i];
            }
            currentCost = neighborCost;

            if (currentCost < bestCost) {
                for (uint32_t i = 0; i < input->ioVec.len; ++i) {
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
    for (uint32_t i = 0; i < output->len; i++) {
        output->sequence[i] = bestSequence[i];
    }

    free(temp_output->sequence);
    free(temp_output);

    return RETURN_OK;
}

int32_t TabuSearch(const InputParam *input, OutputParam *output) {
    int32_t ret;

    // 初始化参数
    int maxIterations = 1000;
    int tabuTenure = 10;

    NearestNeighborAlgorithm(input, output);

    // 初始化解
    uint32_t currentSequence[input->ioVec.len];
    for (uint32_t i = 0; i < input->ioVec.len; ++i) {
        // currentSequence[i] = input->ioVec.ioArray[i].id;
        currentSequence[i] = output->sequence[i];
    }

    // 计算初始解的总时延
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++) {
        output->sequence[i] = currentSequence[i];
    }
    AccessTime accessTime;
    TotalAccessTime(input, output, &accessTime);
    int32_t currentCost = accessTime.addressDuration + accessTime.readDuration;

    uint32_t bestSequence[input->ioVec.len];
    for (uint32_t i = 0; i < input->ioVec.len; ++i) {
        bestSequence[i] = currentSequence[i];
    }
    int32_t bestCost = currentCost;

    // 初始化禁忌表
    int tabuList[input->ioVec.len][input->ioVec.len];
    memset(tabuList, 0, sizeof(tabuList));

    // 禁忌搜索过程
    int iteration = 0;

    while (iteration < maxIterations) {
        // 生成邻域解并选择最优解
        uint32_t bestNeighborSequence[input->ioVec.len];
        int32_t bestNeighborCost = INT32_MAX;
        uint32_t bestI = 0, bestJ = 0;

        for (uint32_t i = 0; i < input->ioVec.len; ++i) {
            for (uint32_t j = i + 1; j < input->ioVec.len; ++j) {
                uint32_t neighborSequence[input->ioVec.len];
                for (uint32_t k = 0; k < input->ioVec.len; ++k) {
                    neighborSequence[k] = currentSequence[k];
                }
                uint32_t temp = neighborSequence[i];
                neighborSequence[i] = neighborSequence[j];
                neighborSequence[j] = temp;

                // 计算邻域解的总时延
                for (uint32_t k = 0; k < output->len; k++) {
                    output->sequence[k] = neighborSequence[k];
                }
                TotalAccessTime(input, output, &accessTime);
                int32_t neighborCost = accessTime.addressDuration + accessTime.readDuration;

                // 检查是否在禁忌表中或满足某些条件
                if ((tabuList[i][j] == 0 || neighborCost < bestCost) && neighborCost < bestNeighborCost) {
                    bestNeighborCost = neighborCost;
                    for (uint32_t k = 0; k < input->ioVec.len; ++k) {
                        bestNeighborSequence[k] = neighborSequence[k];
                    }
                    bestI = i;
                    bestJ = j;
                }
            }
        }

        // 更新当前解
        for (uint32_t i = 0; i < input->ioVec.len; ++i) {
            currentSequence[i] = bestNeighborSequence[i];
        }
        currentCost = bestNeighborCost;

        // 更新禁忌表
        for (uint32_t i = 0; i < input->ioVec.len; ++i) {
            for (uint32_t j = 0; j < input->ioVec.len; ++j) {
                if (tabuList[i][j] > 0) {
                    tabuList[i][j]--;
                }
            }
        }
        tabuList[bestI][bestJ] = tabuTenure;

        // 更新最优解
        if (currentCost < bestCost) {
            for (uint32_t i = 0; i < input->ioVec.len; ++i) {
                bestSequence[i] = currentSequence[i];
            }
            bestCost = currentCost;
        }

        ++iteration;
    }

    // 将最优解复制到输出参数
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++) {
        output->sequence[i] = bestSequence[i];
    }

    return RETURN_OK;
}

int32_t HillClimbing(const InputParam *input, OutputParam *output) {
    int32_t ret;
    int maxIterations = 1000;

    NearestNeighborAlgorithm(input, output);

    // 初始化解
    uint32_t currentSequence[input->ioVec.len];
    for (uint32_t i = 0; i < input->ioVec.len; ++i) {
        // currentSequence[i] = input->ioVec.ioArray[i].id;
        currentSequence[i] = output->sequence[i];
    }

    // 计算初始解的总时延
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++) {
        output->sequence[i] = currentSequence[i];
    }
    AccessTime accessTime;
    TotalAccessTime(input, output, &accessTime);
    int32_t currentCost = accessTime.addressDuration + accessTime.readDuration;

    uint32_t bestSequence[input->ioVec.len];
    for (uint32_t i = 0; i < input->ioVec.len; ++i) {
        bestSequence[i] = currentSequence[i];
    }
    int32_t bestCost = currentCost;

    // 爬山算法过程
    int improved = 1, iteration = 0;

    while (improved) {
        iteration++;
        improved = 0;
        uint32_t neighborSequence[input->ioVec.len];
        int32_t bestNeighborCost = currentCost;

        for (uint32_t i = 0; i < input->ioVec.len; ++i) {
            for (uint32_t j = i + 1; j < input->ioVec.len; ++j) {
                for (uint32_t k = 0; k < input->ioVec.len; ++k) {
                    neighborSequence[k] = currentSequence[k];
                }
                uint32_t temp = neighborSequence[i];
                neighborSequence[i] = neighborSequence[j];
                neighborSequence[j] = temp;

                // 计算邻域解的总时延
                for (uint32_t k = 0; k < output->len; k++) {
                    output->sequence[k] = neighborSequence[k];
                }
                TotalAccessTime(input, output, &accessTime);
                int32_t neighborCost = accessTime.addressDuration + accessTime.readDuration;

                // 检查邻域解是否优于当前解
                if (neighborCost < bestNeighborCost) {
                    bestNeighborCost = neighborCost;
                    for (uint32_t k = 0; k < input->ioVec.len; ++k) {
                        bestSequence[k] = neighborSequence[k];
                    }
                    improved = 1;
                }
            }
        }

        if (improved) {
            for (uint32_t i = 0; i < input->ioVec.len; ++i) {
                currentSequence[i] = bestSequence[i];
            }
            currentCost = bestNeighborCost;
        }
    }

    // 将最优解复制到输出参数
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++) {
        output->sequence[i] = bestSequence[i];
    }

    return RETURN_OK;
}

int32_t GeneticAlgorithm(const InputParam *input, OutputParam *output) {
    // 初始化参数
    int populationSize = 50;
    double crossoverRate = 0.8;
    double mutationRate = 0.01;
    int maxIterations = 1000;

    // 初始化种群
    uint32_t population[populationSize][input->ioVec.len];
    for (int i = 0; i < populationSize; ++i) {
        for (uint32_t j = 0; j < input->ioVec.len; ++j) {
            population[i][j] = input->ioVec.ioArray[j].id;
        }
        // 随机打乱顺序
        for (uint32_t j = 0; j < input->ioVec.len; ++j) {
            uint32_t k = rand() % input->ioVec.len;
            uint32_t temp = population[i][j];
            population[i][j] = population[i][k];
            population[i][k] = temp;
        }
    }

    uint32_t bestSequence[input->ioVec.len];
    int32_t bestCost = INT32_MAX;

    // 遗传算法过程
    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        // 计算适应度
        int32_t fitness[populationSize];
        for (int i = 0; i < populationSize; ++i) {
            OutputParam tempOutput;
            tempOutput.len = input->ioVec.len;
            tempOutput.sequence = (uint32_t *)malloc(input->ioVec.len * sizeof(uint32_t));
            if (tempOutput.sequence == NULL) {
                return RETURN_ERROR;
            }
            for (uint32_t j = 0; j < tempOutput.len; j++) {
                tempOutput.sequence[j] = population[i][j];
            }
            AccessTime accessTime;
            TotalAccessTime(input, &tempOutput, &accessTime);
            fitness[i] = accessTime.addressDuration + accessTime.readDuration;

            if (fitness[i] < bestCost) {
                bestCost = fitness[i];
                for (uint32_t j = 0; j < input->ioVec.len; ++j) {
                    bestSequence[j] = population[i][j];
                }
            }
            free(tempOutput.sequence);
        }

        // 选择
        uint32_t newPopulation[populationSize][input->ioVec.len];
        for (int i = 0; i < populationSize; ++i) {
            int parent1 = rand() % populationSize;
            int parent2 = rand() % populationSize;
            int betterParent = (fitness[parent1] < fitness[parent2]) ? parent1 : parent2;
            for (uint32_t j = 0; j < input->ioVec.len; ++j) {
                newPopulation[i][j] = population[betterParent][j];
            }
        }

        // 交叉
        for (int i = 0; i < populationSize; i += 2) {
            if (((double)rand() / RAND_MAX) < crossoverRate) {
                uint32_t crossoverPoint = rand() % input->ioVec.len;
                uint32_t child1[input->ioVec.len];
                uint32_t child2[input->ioVec.len];
                int used1[input->ioVec.len];
                int used2[input->ioVec.len];
                memset(used1, 0, sizeof(used1));
                memset(used2, 0, sizeof(used2));

                for (uint32_t j = 0; j < crossoverPoint; ++j) {
                    child1[j] = newPopulation[i][j];
                    child2[j] = newPopulation[i + 1][j];
                    used1[child1[j]] = 1;
                    used2[child2[j]] = 1;
                }

                uint32_t index1 = crossoverPoint;
                uint32_t index2 = crossoverPoint;
                for (uint32_t j = 0; j < input->ioVec.len; ++j) {
                    if (!used1[newPopulation[i + 1][j]]) {
                        child1[index1++] = newPopulation[i + 1][j];
                        used1[newPopulation[i + 1][j]] = 1;
                    }
                    if (!used2[newPopulation[i][j]]) {
                        child2[index2++] = newPopulation[i][j];
                        used2[newPopulation[i][j]] = 1;
                    }
                }

                for (uint32_t j = 0; j < input->ioVec.len; ++j) {
                    newPopulation[i][j] = child1[j];
                    newPopulation[i + 1][j] = child2[j];
                }
            }
        }

        // 变异
        for (int i = 0; i < populationSize; ++i) {
            if (((double)rand() / RAND_MAX) < mutationRate) {
                uint32_t point1 = rand() % input->ioVec.len;
                uint32_t point2 = rand() % input->ioVec.len;
                uint32_t temp = newPopulation[i][point1];
                newPopulation[i][point1] = newPopulation[i][point2];
                newPopulation[i][point2] = temp;
            }
        }

        // 更新种群
        for (int i = 0; i < populationSize; ++i) {
            for (uint32_t j = 0; j < input->ioVec.len; ++j) {
                population[i][j] = newPopulation[i][j];
            }
        }
    }

    // 将最优解复制到输出参数
    output->len = input->ioVec.len;
    output->sequence = (uint32_t *)malloc(output->len * sizeof(uint32_t));
    if (output->sequence == NULL) {
        return RETURN_ERROR;
    }
    for (uint32_t i = 0; i < output->len; i++) {
        output->sequence[i] = bestSequence[i];
    }

    return RETURN_OK;
}

MinHeap *createMinHeap(int capacity) {
    MinHeap *heap = (MinHeap *)malloc(sizeof(MinHeap));
    heap->size = 0;
    heap->capacity = capacity;
    heap->nodes = (Node *)malloc(capacity * sizeof(Node));
    return heap;
}

void destoryMinHeap(MinHeap * heap){
    free(heap->nodes);
    free(heap);
    heap = NULL;
}

void swap(Node *a, Node *b) {
    Node temp = *a;
    *a = *b;
    *b = temp;
}

void heapify(MinHeap *heap, int idx) {
    int smallest = idx;
    int left = 2 * idx + 1;
    int right = 2 * idx + 2;

    if (left < heap->size && heap->nodes[left].dis < heap->nodes[smallest].dis)
        smallest = left;

    if (right < heap->size && heap->nodes[right].dis < heap->nodes[smallest].dis)
        smallest = right;

    if (smallest != idx) {
        swap(&heap->nodes[idx], &heap->nodes[smallest]);
        heapify(heap, smallest);
    }
}

Node *extractMin(MinHeap *heap) {  // 弹出最小值
    if (heap->size == 0) {
        return NULL;
    }

    // Node root = heap->nodes[0];
    // heap->nodes[0] = heap->nodes[heap->size - 1];
    swap(&heap->nodes[0], &heap->nodes[heap->size - 1]);
    heap->size--;
    heapify(heap, 0);

    return &heap->nodes[heap->size];
}

Node *getMin(MinHeap *heap)  // 弹出最小值
{
    if (heap->size == 0) {
        return NULL;
    }
    

    return &heap->nodes[0];
}

void insertHeap(MinHeap *heap, Node node) {
    // if (heap->size == heap->capacity) {
    //     printf("Heap overflow\n");
    //     return;
    // }

    if(heap->size < heap->capacity){
        heap->size++;
    }
    else{
        if(heap->nodes[heap->size-1].dis < node.dis){
            return;
        }
    }
    int i = heap->size - 1;
    heap->nodes[i] = node;


    // 向上调整
    while (i != 0 && heap->nodes[(i - 1) / 2].dis > heap->nodes[i].dis) {
        swap(&heap->nodes[i], &heap->nodes[(i - 1) / 2]);
        i = (i - 1) / 2;
    }
}

int getValueInHeapArray(MinHeapArray * arr, int idx){
    return getMin(arr->heap_array[idx])->dis;
}

int getMinValueInHeapArray(MinHeapArray * arr){
    int value = getMin(arr->heap_array[0])->dis;
    return value;
}

void swapInHeapArray(MinHeapArray * arr, int idx1, int idx2){
    MinHeap * tmp = arr->heap_array[idx1];
    arr->heap_array[idx1] = arr->heap_array[idx2];
    arr->heap_array[idx2] = tmp;
}


void insertHeapInHeapArray(MinHeapArray * arr, MinHeap * heap){
    arr->heap_array[arr->size++] = heap;
    int i = arr->size - 1;
    while (i != 0 && getValueInHeapArray(arr, (i - 1) / 2) >= getValueInHeapArray(arr, i))
    {
        swapInHeapArray(arr, i, (i - 1) / 2);
        i = (i - 1) / 2;
    }
}


Node * getNodeInHeapArray(MinHeapArray * arr){
    return getMin(arr->heap_array[0]);
}

Node * popNodeInHeapArray(MinHeapArray * arr){
    if (arr->size == 0)
    {
        return NULL;
    }
    Node * tmp = extractMin(arr->heap_array[0]);
    // if(arr->heap_array[0]->size == 0){
    //     printf("ERROR!\n");
    // }
    
    return tmp;
}

MinHeap * popHeapInHeapArray(MinHeapArray * arr){
    if (arr->size == 0)
    {
        return NULL;
    }
    MinHeap * tmp = arr->heap_array[0];
    arr->heap_array[0] = arr->heap_array[arr->size - 1];
    arr->heap_array[arr->size - 1] = tmp;
    arr->size--;

    minHeapArrayHeapify(arr, 0);

    return arr->heap_array[arr->size];
}

void minHeapArrayHeapify(MinHeapArray * arr, int idx)
{
    // return ;
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

void initUnionSet()
{
    fa = (int *)malloc(maxn * sizeof(int));
    h = (int *)malloc(maxn * sizeof(int));
    sz = (int *)malloc(maxn * sizeof(int));
    for (int i = 0; i < maxn; ++i) {
        fa[i] = i;
        h[i] = 1;
        sz[i] = 1;
    }
}

void freeUnionSet() {
    free(fa);
    free(h);
}

int find(int x) {
    return x == fa[x] ? x : (fa[x] = find(fa[x]));
}

void unite(int x, int y) {
    x = find(x), y = find(y);
    if (h[x] < h[y]) {
        fa[x] = y;
        sz[x] = sz[y] = sz[x] + sz[y];
    } else {
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

    // return RETURN_OK;

    MinHeapArray heap_array = {maxn, 0, (MinHeap **)malloc(maxn * sizeof(MinHeap *))};
    struct timeval start, end;
    // gettimeofday(&start, NULL);
    MinHeap * heap = createMinHeap(maxn + 1);

    int selected_value_sum = 0;

    // 初始化当前头位置为输入的头状态
    int max_edge_num = 8*1024*1024/8; // 假设最多分配8MB的内存给节点
    int edge_per_node = max_edge_num/(input->ioVec.len + 1);
    if(edge_per_node > input->ioVec.len + 1){
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
            if(tmp_node.dis < min_node.dis){
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
    while (sz[0] != input->ioVec.len + 1) {
        if (sz[0] == input->ioVec.len + 1) {
            break;
        }
        
        int min_value = INT32_MAX;
        int min_heap_idx = -1;
        Node *node = extractMin(heap);
        

        if (nex[node->x] == 0 && ( node->x == 0 || (nex[node->y] != node->x)) && vis[node->y] == 0 && find(node->x) != find(node->y))
        {
            unite(node->x, node->y);
            nex[node->x] = node->y;
            vis[node->y] = 1;
            selected_value_sum += node->dis;
            set_num --;
            // printf("%d\n", set_num);
        }
        else{
            int target_id = node->y;
            // printf("target_id: %d, source_id: %d, %d, %d, %d, %d, %d, %d\n", target_id, node->x, nex[node->x], vis[node->x], vis[node->y], nex[node->y] ,find(node->x) ,find(node->y));
            HeadInfo status_tmp = {input->ioVec.ioArray[target_id-1].wrap, input->ioVec.ioArray[target_id-1].startLpos, HEAD_RW};
            Node min_node;
            min_node.dis = INT32_MAX;
            if(nex[0] == 0){
                Node tmp_node = {0, target_id, SeekTimeCalculate(&currentHead, &status_tmp)};
                min_node = tmp_node;
            }
            for (int source_id = 1; source_id < input->ioVec.len + 1; source_id++)
            {
                if (target_id == source_id || nex[source_id] != 0 || find(source_id) == find(target_id))
                    continue;
                HeadInfo status1 = {input->ioVec.ioArray[source_id-1].wrap, input->ioVec.ioArray[source_id - 1].endLpos, HEAD_RW};
                Node tmp_node = {source_id, target_id, SeekTimeCalculate(&status1, &status_tmp)};
                if(tmp_node.dis < min_node.dis){
                    min_node = tmp_node;
                }
            }
            insertHeap(heap, min_node);
        }
    }
    int now = nex[0], cnt = 0;
    while (cnt < input->ioVec.len) {
        output->sequence[cnt++] = now;
        now = nex[now];
    }
    destoryMinHeap(heap);

    // free(dis);
    return RETURN_OK;
}

int32_t p_scan(const InputParam *input, OutputParam *output) {
    // 参数：const InputParam *input, OutputParam *output, int partition_len, int *partitions, int p_num, int *scan_method
    // 当partitions为NULL时，后四个参数均可忽略，视为固定分区大小遍历搜索，并采用统一的SCAN1/SCAN2
    // 当partitions不为NULL时，后四个参数必须传入
    // partition_len为单位粒度，partitions为每个分区多少个单位大小，p_num为分区的数量，scan_method为每个分区采用的扫描方法
    return partition_scan_new(input, output, 0, NULL, 0, NULL);
    // int pnum = 4;
    // int partition_len = 5000;
    // int partitions[4] = {2, 4, 8, 133};
    // int scan_method[4] = {0, 1, 2, 1};
    // return partition_scan_new(input, output, partition_len, partitions, pnum, scan_method);
}

int32_t p_scan_t(const InputParam *input, OutputParam *output, int partition_len, int *partitions, int p_num, int *scan_method) {
    // 参数：const InputParam *input, OutputParam *output, int partition_len, int *partitions, int p_num, int *scan_method
    // 当partitions为NULL时，后四个参数均可忽略，视为固定分区大小遍历搜索，并采用统一的SCAN1/SCAN2
    // 当partitions不为NULL时，后四个参数必须传入
    // partition_len为单位粒度，partitions为每个分区多少个单位大小，p_num为分区的数量，scan_method为每个分区采用的扫描方法
    partition_scan_new(input, output, partition_len, partitions, p_num, scan_method);
    AccessTime accessTime = {0};
    TotalAccessTime(input, output, &accessTime);
    return accessTime.addressDuration;
}

int32_t partition_scan_new(const InputParam *input, OutputParam *output, int partition_len, int *partitions, int p_num, int *scan_method) {
    // 复制 IO 请求数组并按 lpos 排序
    IOUint *sortedIOs = (IOUint *)malloc(input->ioVec.len * sizeof(IOUint));
    for (uint32_t i = 0; i < input->ioVec.len; ++i) {
        sortedIOs[i] = input->ioVec.ioArray[i];
    }
    QuickSort(sortedIOs, input->ioVec.len);

    //----搜索最佳分割参数----
    int min_time = 0x3f3f3f3f, best_partition_size = 5000;
    // scan1：只按io请求的开始位置进行排序，可能会有同向掉头的情况
    // scan2：保证后一个请求的开始位置在前一个请求的结束位置之后，不会同向掉头，但可能需要扫描多次
    // mpscan*：对scan2进行优化，减少扫描次数
    int best_scan_method = 1;
    int *best_sequence = (int *)malloc(input->ioVec.len * sizeof(int));
    bool *vis = (bool *)malloc((input->ioVec.len + 1) * sizeof(bool));
    int partition_io_num[1000] = {0};    // 存储每个分区中的IO数量
    int partition_io_start[1000] = {0};  // 存储每个分区起始IO的索引

    // 按传入的参数进行分区
    if (partitions != NULL) {
        //----检查----
        int temp_partition_num = (MAX_LPOS + partition_len - 1) / partition_len;
        DEBUG("partition_len=%d, partition_num=%d\n", partition_len, p_num);
        printf("partitions=[");
        int tot = 0;
        for (int i = 0; i < p_num; i++) {
            // printf("partitions[%d]=%d,", i, partitions[i]);
            printf("%d,", partitions[i]);
            tot += partitions[i];
        }
        if (tot != temp_partition_num) {
            ERROR("tot=%d, partition_num should be %d\n", tot, temp_partition_num);
        }
        printf("]\n");
        printf("SCAN method=[");
        for (int i = 0; i < p_num; i++) {
            printf("%d,", scan_method[i]);
        }
        printf("]\n");
        //----检查----

        int partition_start_now = 0;
        int now = 0;
        int partition_threshold = partition_len * partitions[now];
        for (int i = 0; i < input->ioVec.len; i++) {
            // printf("\n");
            // DEBUG("i=%d startLpos=%d\n", i, sortedIOs[i].startLpos);
            if (sortedIOs[i].startLpos >= partition_start_now + partition_threshold) {
                // DEBUG("partition %d start at %d, io_num=%d\n", now, partition_start_now, partition_io_num[now]);
                while (sortedIOs[i].startLpos >= partition_start_now + partition_threshold) {
                    partition_start_now += partition_threshold;
                    now++;
                    partition_threshold = partition_len * partitions[now];
                }
                partition_io_start[now] = i;
            }
            // DEBUG("partition_start_now=%d now=%d\n", partition_start_now, now);
            partition_io_num[now]++;
        }

        for (int i = 0; i < input->ioVec.len + 1; i++) {
            vis[i] = 0;
        }
        HeadInfo head = input->headInfo;
        for (int i = 0; i <= now; i++) {
            if (partition_io_num[i] == 0)
                continue;
            // DEBUG("partition %d start at %d, io_num=%d\n", i, partition_io_start[i], partition_io_num[i]);
            _partition_scan_new(output, sortedIOs, vis, &head, partition_io_start[i], partition_io_num[i], scan_method[i]);
            head.wrap = sortedIOs[partition_io_start[i] + partition_io_num[i] - 1].wrap;
            head.lpos = sortedIOs[partition_io_start[i] + partition_io_num[i] - 1].endLpos;
        }
        // AccessTime accessTime = {0};
        // TotalAccessTime(input, output, &accessTime);
        // int time = accessTime.addressDuration;
    } else {
        // 按固定长度进行分区，对长度参数进行搜索
        for (int i = 5000; i <= 740000; i += 5000) {
            int partition_start_now = 0;  // 当前分区的起始位置
            int now = 0;                  // 当前分区索引
            int partition_threshold = i;  // 当前分区长度
            memset(partition_io_start, 0, sizeof(partition_io_start));
            memset(partition_io_num, 0, sizeof(partition_io_num));
            for (int j = 0; j < input->ioVec.len; j++) {
                // printf("\n");
                // DEBUG("j=%d startLpos=%d\n", j, sortedIOs[j].startLpos);
                if (sortedIOs[j].startLpos >= partition_start_now + partition_threshold) {
                    while (sortedIOs[j].startLpos >= partition_start_now + partition_threshold) {
                        partition_start_now += partition_threshold;
                        now++;
                    }
                    partition_io_start[now] = j;
                    // DEBUG("partition %d start at %d, io_num=%d\n", now, partition_start_now, partition_io_num[now]);
                }
                // DEBUG("partition_start_now=%d now=%d\n", partition_start_now, now);
                partition_io_num[now]++;
            }

            // DEBUG("partition_len=%d, partition_num=%d\n", i, now);
            // for (int j = 0; j <= now; j++)
            // {
            //     if (partition_io_num[j] != 0)
            //         DEBUG("partition %d start at %d, io_num=%d\n", j, partition_io_start[j], partition_io_num[j]);
            // }

            // 1. scan method: scan1
            for (int j = 0; j < input->ioVec.len + 1; j++) {
                vis[j] = 0;
            }
            HeadInfo head = input->headInfo;
            for (int j = 0; j <= now; j++) {
                // DEBUG("partition %d start at %d, io_num=%d\n", j, partition_io_start[j], partition_io_num[j]);
                if (partition_io_num[j] == 0)
                    continue;
                // for (int j = 0; j < input->ioVec.len; j++)
                //     printf("vis[%d]=%d\n", sortedIOs[j].id, vis[sortedIOs[j].id]);
                // printf("\n");
                _partition_scan_new(output, sortedIOs, vis, &head, partition_io_start[j], partition_io_num[j], 1);
                head.wrap = sortedIOs[partition_io_start[j] + partition_io_num[j] - 1].wrap;
                head.lpos = sortedIOs[partition_io_start[j] + partition_io_num[j] - 1].endLpos;
            }
            AccessTime accessTime = {0};
            TotalAccessTime(input, output, &accessTime);
            int time = accessTime.addressDuration;
            if (time <= min_time) {
                best_scan_method = 1;
                best_partition_size = i;
                min_time = time;
                for (int j = 0; j < input->ioVec.len; j++) {
                    best_sequence[j] = output->sequence[j];
                }
            }
            for (int j = 0; j < input->ioVec.len + 1; j++) {
                vis[j] = 0;
            }
            for (int j = 0; j < now; j++) {
                if (partition_io_num[j] == 0) {
                    continue;
                }
                _partition_scan_new(output, sortedIOs, vis, &head, partition_io_start[j], partition_io_num[j], 2);
                head.wrap = sortedIOs[partition_io_start[j] + partition_io_num[j] - 1].wrap;
                head.lpos = sortedIOs[partition_io_start[j] + partition_io_num[j] - 1].endLpos;
            }
            TotalAccessTime(input, output, &accessTime);
            time = accessTime.addressDuration;
            if (time <= min_time) {
                best_scan_method = 2;
                best_partition_size = i;
                min_time = time;
                for (int j = 0; j < input->ioVec.len; j++) {
                    best_sequence[j] = output->sequence[j];
                }
            }
        }
        printf("best_scan_method=%d best_partition_size=%d\n", best_scan_method, best_partition_size);
        for (int i = 0; i < input->ioVec.len; i++) {
            output->sequence[i] = best_sequence[i];
        }
    }
    free(best_sequence);
    free(sortedIOs);
}

int32_t _partition_scan_new(OutputParam *output, IOUint *sortedIOs, bool *vis, HeadInfo *head, int partition_start, int partition_len, const int scan_method) {
    uint32_t index = partition_start;
    HeadInfo currentHead = *head;
    int direction = 1;  // 扫描方向：1 表示从 BOT 向 EOT 扫描，-1 表示从 EOT 向 BOT 扫描
    // DEBUG("partition_start=%d, partition_len=%d\n", partition_start, partition_len);
    for (int i = partition_start; i < partition_start + partition_len; i++) {
        if (vis[sortedIOs[i].id]) {
            ERROR("vis[%d]=1\n", sortedIOs[i].id);
        }
    }
    if (scan_method > 2) {
        ERROR("unknown scan_method: %d\n", scan_method);
    }
    while (index < partition_start + partition_len) {
        if (scan_method == 0) {  // SORT
            output->sequence[index] = sortedIOs[index].id;
            index++;
            continue;
        }
        // DEBUG("index=%d, direction=%d\n", index, direction);
        if (direction == 1) {
            // 从 BOT 向 EOT 扫描
            for (uint32_t i = partition_start; i < partition_start + partition_len; ++i) {
                // DEBUG("i=%d, wrap=%d, vis=%d, startLpos=%d, currentHead.lpos=%d\n", i, sortedIOs[i].wrap, vis[sortedIOs[i].id], sortedIOs[i].startLpos, currentHead.lpos);
                // DEBUG("partition_start=%d, partition_len=%d\n", partition_start, partition_len);
                if (sortedIOs[i].wrap & 1 || vis[sortedIOs[i].id]) {
                    continue;
                }
                if (scan_method == 1) {  // SCAN1，两趟扫描
                    if (sortedIOs[i].startLpos >= currentHead.lpos) {
                        output->sequence[index++] = sortedIOs[i].id;
                        vis[sortedIOs[i].id] = 1;
                        currentHead.wrap = sortedIOs[i].wrap;
                        currentHead.lpos = sortedIOs[i].startLpos;
                    }
                } else if (scan_method == 2) {  // SCAN2，多趟扫描
                    if (sortedIOs[i].startLpos > currentHead.lpos || currentHead.lpos == 0) {
                        output->sequence[index++] = sortedIOs[i].id;
                        vis[sortedIOs[i].id] = 1;
                        currentHead.wrap = sortedIOs[i].wrap;
                        currentHead.lpos = sortedIOs[i].endLpos;
                    }
                }
            }
            direction = -1;  // 改变扫描方向
            // 赋值为最大值以保证下一轮扫描中大于当前位置的请求不会被忽略
            currentHead.lpos = MAX_LPOS + 1;
        } else {
            // 从 EOT 向 BOT 扫描
            for (int32_t i = partition_start + partition_len - 1; i >= partition_start; --i) {
                // DEBUG("i=%d, wrap=%d, vis=%d, startLpos=%d, currentHead.lpos=%d\n", i, sortedIOs[i].wrap, vis[sortedIOs[i].id], sortedIOs[i].startLpos, currentHead.lpos);
                if (!(sortedIOs[i].wrap & 1) || vis[sortedIOs[i].id]) {
                    continue;
                }
                if (scan_method == 1) {  // SCAN1，两趟扫描
                    if (sortedIOs[i].startLpos <= currentHead.lpos || currentHead.lpos == 0) {
                        output->sequence[index++] = sortedIOs[i].id;
                        vis[sortedIOs[i].id] = 1;
                        currentHead.wrap = sortedIOs[i].wrap;
                        currentHead.lpos = sortedIOs[i].startLpos;
                    }
                } else if (scan_method == 2) {  // SCAN2，多趟扫描
                    if (sortedIOs[i].startLpos < currentHead.lpos || currentHead.lpos == 0) {
                        output->sequence[index++] = sortedIOs[i].id;
                        vis[sortedIOs[i].id] = 1;
                        currentHead.wrap = sortedIOs[i].wrap;
                        currentHead.lpos = sortedIOs[i].endLpos;
                    }
                }
            }
            direction = 1;  // 改变扫描方向
            // 赋值为0以保证下一轮扫描中小于当前位置的请求不会被忽略
            currentHead.lpos = 0;
        }
    }
    return RETURN_OK;
}

int32_t _partition_mpscan(OutputParam *output, IOUint *sortedIOs, HeadInfo *head, bool *vis, int partition_start, int partition_len) {
    DEBUG("partition_start=%d, partition_len=%d\n", partition_start, partition_len);
    // 初始化当前头位置为输入的头状态
    HeadInfo currentHead = {0, 0, 1};

    // 扫描方向：1 表示从 BOT 向 EOT 扫描，-1 表示从 EOT 向 BOT 扫描
    int direction = 1;
    uint32_t index = 0;

    InputParam *cur_input = (InputParam *)malloc(sizeof(InputParam));
    cur_input->headInfo = *head;
    cur_input->ioVec.len = partition_len;
    cur_input->ioVec.ioArray = (IOUint *)malloc(partition_len * sizeof(IOUint));
    for (int i = 0; i < partition_len; i++) {
        cur_input->ioVec.ioArray[i] = sortedIOs[partition_start + i];
        // DEBUG("i=%d, id=%d, wrap=%d, startLpos=%d, endLpos=%d\n", i + partition_start, sortedIOs[partition_start + i].id, sortedIOs[partition_start + i].wrap, sortedIOs[partition_start + i].startLpos, sortedIOs[partition_start + i].endLpos);
    }
    OutputParam *cur_output = (OutputParam *)malloc(sizeof(OutputParam));
    cur_output->len = partition_len;
    cur_output->sequence = (uint32_t *)malloc(partition_len * sizeof(uint32_t));
    for (int i = 0; i < partition_len; i++) {
        cur_output->sequence[i] = sortedIOs[partition_start + i].id;
    }

    while (index < cur_input->ioVec.len) {
        if (direction == 1) {
            // printf("\nBOT->EOT: ");
            // 从 BOT 向 EOT 扫描，wrap 为偶数
            for (uint32_t i = 0; i < cur_input->ioVec.len; ++i) {
                if (sortedIOs[partition_start + i].wrap & 1 || vis[sortedIOs[partition_start + i].id]) {
                    continue;
                }
                if (sortedIOs[partition_start + i].startLpos >= currentHead.lpos) {
                    // printf("%d ", sortedIOs[partition_start + i].id);
                    cur_output->sequence[index++] = sortedIOs[partition_start + i].id;
                    vis[sortedIOs[partition_start + i].id] = 1;
                    currentHead.wrap = sortedIOs[partition_start + i].wrap;
                    currentHead.lpos = sortedIOs[partition_start + i].endLpos;
                    // currentHead.lpos = sortedIOs[partition_start+i].startLpos;
                }
            }
            direction = -1;  // 改变扫描方向
            currentHead.lpos = MAX_LPOS;
        } else {
            // printf("\nEOT->BOT: ");
            // 从 EOT 向 BOT 扫描
            for (int32_t i = cur_input->ioVec.len - 1; i >= 0; --i) {
                if (!(sortedIOs[partition_start + i].wrap & 1) || vis[sortedIOs[partition_start + i].id]) {
                    continue;
                }
                if (sortedIOs[partition_start + i].startLpos <= currentHead.lpos) {
                    // printf("%d ", sortedIOs[partition_start + i].id);
                    cur_output->sequence[index++] = sortedIOs[partition_start + i].id;
                    vis[sortedIOs[partition_start + i].id] = 1;
                    currentHead.wrap = sortedIOs[partition_start + i].wrap;
                    currentHead.lpos = sortedIOs[partition_start + i].endLpos;
                }
            }
            direction = 1;  // 改变扫描方向
            currentHead.lpos = 0;
        }
    }

    // 将最后一轮扫描插入前面
    // OutputParam *tmp;
    OutputParam *tmp = (OutputParam *)malloc(sizeof(OutputParam));
    tmp->len = cur_input->ioVec.len;
    tmp->sequence = (uint32_t *)malloc(cur_input->ioVec.len * sizeof(uint32_t));
    memcpy(tmp->sequence, cur_output->sequence, cur_input->ioVec.len * sizeof(int));
    AccessTime accessTime;
    DEBUG("cur_input->ioVec.len=%d\n", cur_input->ioVec.len);
    DEBUG("cur_output->len=%d\n", cur_output->len);
    for (int i = 0; i < cur_input->ioVec.len; i++) {
        DEBUG("cur_input->ioVec.ioArray[%d].id=%d\n", i, cur_input->ioVec.ioArray[i].id);
        DEBUG("cur_output->sequence[%d]=%d\n", i, cur_output->sequence[i]);
    }
    TotalAccessTime(cur_input, cur_output, &accessTime);
    DEBUG("111\n");
    while (true) {
        // 最后一轮扫描在 output 中的下标范围为 [idx+1, output->len - 1]
        int32_t io_len = 0, idx = cur_output->len - 1;
        while (idx >= 1 && (cur_input->ioVec.ioArray[cur_output->sequence[idx] - 1].wrap & 1) &&
               cur_input->ioVec.ioArray[cur_output->sequence[idx] - 1].startLpos < cur_input->ioVec.ioArray[output->sequence[idx - 1] - 1].startLpos) {  // 奇数
            idx--;
        }
        while (idx >= 1 && !(cur_input->ioVec.ioArray[cur_output->sequence[idx] - 1].wrap & 1) &&
               cur_input->ioVec.ioArray[cur_output->sequence[idx] - 1].startLpos > cur_input->ioVec.ioArray[output->sequence[idx - 1] - 1].startLpos) {  // 偶数
            idx--;
        }
        if (idx < 0)
            break;  // 当前已经是最后一轮扫描
        // printf("\n%d:", cur_output->sequence[idx]);

        // 遍历最后一轮的每个 IO
        for (int i = idx; i < cur_output->len; ++i) {
            // printf("\ni = %d , ", cur_output->sequence[i]);
            int32_t minTime = INT32_MAX;
            int32_t best_pos = -1;
            HeadInfo z = {cur_input->ioVec.ioArray[cur_output->sequence[i] - 1].wrap, cur_input->ioVec.ioArray[output->sequence[i] - 1].startLpos, HEAD_RW};

            // 寻找插入的最佳位置
            for (int j = 0; j < i - 1; ++j) {
                // if(cur_input->ioVec.ioArray[cur_output->sequence[j]-1].wrap != cur_input->ioVec.ioArray[output->sequence[i]-1].wrap)
                //     continue;
                HeadInfo x = {cur_input->ioVec.ioArray[cur_output->sequence[j] - 1].wrap, cur_input->ioVec.ioArray[output->sequence[j] - 1].startLpos, HEAD_RW};
                HeadInfo y = {cur_input->ioVec.ioArray[cur_output->sequence[j + 1] - 1].wrap, cur_input->ioVec.ioArray[output->sequence[j + 1] - 1].startLpos, HEAD_RW};
                int32_t seekTime = SeekTimeCalculate(&x, &z) + SeekTimeCalculate(&z, &y) - SeekTimeCalculate(&x, &y);
                // printf("seekTime = %d, minTime = %d ", seekTime, minTime);
                if (seekTime < minTime) {
                    best_pos = j;
                    minTime = seekTime;
                }
            }

            // printf("best_pos = %d , ", best_pos);

            // 将当前 IO 插入到 best_pos 后面
            for (int j = i; j > best_pos + 1; --j) {
                tmp->sequence[j] = tmp->sequence[j - 1];
            }
            tmp->sequence[best_pos + 1] = cur_output->sequence[i];  // 更新该处的 IO 序号
        }

        AccessTime tmpTime;
        TotalAccessTime(cur_input, tmp, &tmpTime);

        // printf("\ntmp[ ");
        // for (int i = 0; i < tmp->len; ++i)
        // {
        //     printf("%d ", tmp->sequence[i]);
        // }
        // printf("]\nout[ ");
        // for (int i = 0; i < output->len; ++i)
        // {
        //     printf("%d ", output->sequence[i]);
        // }
        // printf("]\ntmp: %d, output: %d\n", tmpTime.addressDuration, accessTime.addressDuration);

        if (tmpTime.addressDuration < accessTime.addressDuration) {
            accessTime.addressDuration = tmpTime.addressDuration;
            memcpy(cur_output->sequence, tmp->sequence, cur_input->ioVec.len * sizeof(int));
        } else {
            break;
        }
    }
    for (int i = 0; i < cur_input->ioVec.len; i++) {
        output->sequence[partition_start + i] = cur_output->sequence[i];
    }

    free(cur_input->ioVec.ioArray);
    free(cur_input);
    free(cur_output->sequence);
    free(cur_output);
    free(tmp->sequence);
    free(tmp);
    return RETURN_OK;
}

int32_t partition_scan(const InputParam *input, OutputParam *output) {
    // 复制 IO 请求数组并按 lpos 排序
    IOUint *sortedIOs = (IOUint *)malloc(input->ioVec.len * sizeof(IOUint));
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
        sortedIOs[i] = input->ioVec.ioArray[i];
    QuickSort(sortedIOs, input->ioVec.len);

    for (int i = 0; i < input->ioVec.len; i++) {
        // printf("%d ", sortedIOs[i].startLpos);
        if (i + 1 == input->ioVec.len)
            break;
        if (sortedIOs[i + 1].startLpos < sortedIOs[i].startLpos) {
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
    for (int i = 5000; i <= 740000; i += 5000) {
        int partition_start_now = 0;
        int now = 0;
        int partition_threshold = i;
        memset(partition_io_start, 0, sizeof(partition_io_start));
        memset(partition_io_num, 0, sizeof(partition_io_num));
        // 遍历所有请求，统计每个分区的起始请求和请求数量
        for (int j = 0; j < input->ioVec.len; j++) {
            if (sortedIOs[j].startLpos >= partition_start_now + partition_threshold) {
                while (sortedIOs[j].startLpos >= partition_start_now + partition_threshold) {
                    partition_start_now += partition_threshold;
                    now++;
                }
                partition_io_start[now] = j;
            }
            partition_io_num[now]++;
        }
        // 分区处理方法，SORT、SCAN1、SCAN2、MPScan*
        int scan_method[] = {0, 1, 2, 3};
        // int scan_method[] = {0, 1, 2};
        // int scan_method[] = {3};
        int method_num = sizeof(scan_method) / sizeof(scan_method[0]);
        // DEBUG("method_num=%d\n", method_num);
        for (int method_idx = 0; method_idx < method_num; method_idx++) {
            // DEBUG("partition_len=%d, method=%d\n", i, scan_method[method_idx]);
            // 重置 vis 数组
            for (int j = 0; j < input->ioVec.len + 1; j++)
                vis[j] = 0;
            HeadInfo head = input->headInfo;
            head.status = HEAD_RW;
            // 对每个分区进行处理
            for (int j = 0; j <= now; j++) {
                // 跳过空分区
                if (partition_io_num[j] == 0)
                    continue;
                if (scan_method[method_idx] < 3)
                    _partition_scan_new(output, sortedIOs, vis, &head, partition_io_start[j], partition_io_num[j], scan_method[method_idx]);
                else if (scan_method[method_idx] == 3) {
                    // _partition_mpscan(output, sortedIOs, &head, vis, partition_io_start[j], partition_io_num[j]);
                    MPScanPerPartition(input, output, sortedIOs, vis, &head, partition_io_start[j], partition_io_start[j] + partition_io_num[j]);
                }
                head.wrap = sortedIOs[partition_io_start[j] + partition_io_num[j] - 1].wrap;
                head.lpos = sortedIOs[partition_io_start[j] + partition_io_num[j] - 1].endLpos;
            }
            AccessTime accessTime = {0};
            // 计算当前分区方案下的总寻址时间
            TotalAccessTime(input, output, &accessTime);
            int time = accessTime.addressDuration;
            // 记录最小寻址时间的分区方案
            if (time <= min_time) {
                best_scan_method = scan_method[method_idx];
                best_partition_size = i;
                min_time = time;
                for (int j = 0; j < input->ioVec.len; j++)
                    best_sequence[j] = output->sequence[j];
            }
        }
    }
    printf("best_scan_method=%d best_partition_size=%d\n", best_scan_method, best_partition_size);
    for (int i = 0; i < input->ioVec.len; i++) {
        output->sequence[i] = best_sequence[i];
    }
    free(best_sequence);
    free(sortedIOs);
}

// 封装调度算法映射表
AlgorithmMap algorithms[] = {
    {"FCFS", IOScheduleAlgorithm},
    {"SORT", SORT},
    {"SCAN", SCAN},
    {"SCAN2", SCAN2},
    {"Nearest", NearestNeighborAlgorithm},
    {"SA", SimulatedAnnealing},
    {"TS", IOScheduleAlgorithm},  // TabuSearch
    {"HC", HillClimbing},
    {"GA", IOScheduleAlgorithm},  // GeneticAlgorithm
    {"merge", merge},
    {"partition_scan", partition_scan},
    {"partition_scan_new", p_scan},
    {"MPScan", MPScan},
    {"MPScanPartition", MPScanPartition}};

AlgorithmMap operator_optimizations[] = {
    {"SIMPLE", SimpleOperatorOptimization},
};

// 获取算法函数的封装函数
AlgorithmFunc get_algorithm_function(const char *algorithm) {
    int num_algorithms = sizeof(algorithms) / sizeof(algorithms[0]);
    for (int i = 0; i < num_algorithms; ++i) {
        if (strcmp(algorithms[i].name, algorithm) == 0) {
            return algorithms[i].func;
        }
    }
    return NULL;  // 找不到对应算法，返回 NULL
}
AlgorithmFunc get_operator_optimization_function(const char *operator_optimization) {
    int num_operator_optimizations = sizeof(operator_optimizations) / sizeof(operator_optimizations[0]);
    for (int i = 0; i < num_operator_optimizations; ++i) {
        if (strcmp(operator_optimizations[i].name, operator_optimization) == 0) {
            return operator_optimizations[i].func;
        }
    }
    return NULL;  // 找不到对应算法，返回 NULL
}

int32_t AlgorithmRun(const InputParam *input, OutputParam *output, char *algorithm, char *operator_optimization) {
    printf("algorithm=%s, operator_optimization=%s\n", algorithm, operator_optimization);
    AlgorithmFunc selected_algorithm = get_algorithm_function(algorithm);  // 获取对应的算法函数
    AlgorithmFunc selected_operator_optimization = get_operator_optimization_function(operator_optimization);
    if (!selected_algorithm) {
        printf("Invalid algorithm name: %s\n", algorithm);
        return RETURN_ERROR;  // 找不到对应的算法，返回错误
    }
    int32_t ret1 = selected_algorithm(input, output);
    if (ret1 != RETURN_OK) {
        printf("Algorithm %s execution failed.\n", algorithm);
        return RETURN_ERROR;
    }
    printf("Finished Seeking Algorithm\n");
    int32_t ret2 = selected_operator_optimization(input, output);
    if (ret2 != RETURN_OK) {
        printf("Operator Optimization %s execution failed.\n", algorithm);
        return RETURN_ERROR;
    }
    printf("Finished Operator Optimization\n");
    if (ret1 == RETURN_OK & ret2 == RETURN_OK) {
        // step1, step2都成功
        return RETURN_OK;
    }
    return RETURN_ERROR;
}
