#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "public.h"
#include "algorithm.h"
#include "logging.h"

// #define LOGLEVEL WARN_LEVEL

InputParam case1 = {
    .headInfo = {
        .wrap = 8,
        .lpos = 1000,
        .status = HEAD_STATIC},
    .ioVec = {.len = 5, .ioArray = (IOUint[5]){
                            {1, 4, 100, 150},
                            {2, 125, 58, 5},
                            {3, 9, 90, 30},
                            {4, 3, 29, 120},
                            {5, 6, 500, 1500},
                        }}};

OutputParam output1 = {
    .len = 5,
    .sequence = (uint32_t[5]){1, 2, 3, 4, 5}};

InputParam case1_new = {
    .headInfo = {
        .wrap = 8,
        .lpos = 1000,
        .status = HEAD_STATIC},
    .ioVec = {.len = 15, .ioArray = (IOUint[15]){{1, 244, 551525, 552079}, {2, 122, 187227, 187781}, {3, 155, 520928, 520374}, {4, 105, 462047, 461493}, {5, 242, 31892, 32446}, {6, 208, 394655, 395209}, {7, 255, 212911, 212357}, {8, 172, 520584, 521138}, {9, 208, 414564, 415118}, {10, 153, 682905, 682351}, {11, 51, 269102, 268548}, {12, 219, 485628, 485074}, {13, 246, 393485, 394039}, {14, 267, 997, 443}, {15, 88, 185046, 185600}}}};

OutputParam output1_new = {
    .len = 15,
    .sequence = (uint32_t[15]){1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}};

InputParam case2 = {
    .headInfo = {
        .wrap = 5,
        .lpos = 1000,
        .status = HEAD_STATIC},
    .ioVec = {.len = 10, .ioArray = (IOUint[10]){
                             {1, 4, 100, 150},
                             {2, 125, 58, 5},
                             {3, 9, 90, 30},
                             {4, 3, 29, 120},
                             {5, 6, 500, 1500},
                             {6, 4, 100, 150},
                             {7, 125, 58, 5},
                             {8, 9, 90, 30},
                             {9, 3, 29, 120},
                             {10, 6, 500, 1500},
                         }}};

OutputParam output2 = {
    .len = 10,
    .sequence = (uint32_t[10]){1, 2, 3, 4, 5, 6, 7, 8, 9, 10}};

InputParam case2_new = {
    .headInfo = {
        .wrap = 5,
        .lpos = 1000,
        .status = HEAD_STATIC},
    .ioVec = {.len = 10, .ioArray = (IOUint[10]){{1, 258, 299761, 300315}, {2, 78, 113833, 114387}, {3, 101, 408955, 408401}, {4, 62, 320443, 320997}, {5, 11, 164253, 163699}, {6, 235, 648483, 647929}, {7, 263, 191017, 190463}, {8, 58, 72549, 73103}, {9, 9, 414131, 413577}, {10, 38, 595403, 595957}}}};

OutputParam output2_new = {
    .len = 10,
    .sequence = (uint32_t[10]){1, 2, 3, 4, 5, 6, 7, 8, 9, 10}};

void interact()
{
    HeadInfo start;
    HeadInfo end;
    int32_t seekT = 0;
    int32_t beltW = 0;
    int32_t motorW = 0;
    uint32_t rwT;

    char input[100];
    while (1)
    {
        printf("请输入start.wrap, start.lpos, start.status (或输入'q'退出): ");
        fgets(input, sizeof(input), stdin);
        if (input[0] == 'q')
        {
            break;
        }
        sscanf(input, "%d %d %d", &start.wrap, &start.lpos, &start.status);

        printf("请输入end.wrap, end.lpos, end.status: ");
        fgets(input, sizeof(input), stdin);
        sscanf(input, "%d %d %d", &end.wrap, &end.lpos, &end.status);

        seekT = SeekTimeCalculate(&start, &end);   // 寻址时间
        beltW = BeltWearTimes(&start, &end, NULL); // 带体磨损
        motorW = MotorWearTimes(&start, &end);     // 电机磨损
        rwT = ReadTimeCalculate(abs(start.lpos - end.lpos));

        printf("seekT: %d ms, beltW: %d lpos, motorW: %d lpos\n", seekT, beltW, motorW);
        printf("rwT: %d ms\n", rwT);
    }
}

void PrintInputParam(const InputParam *input)
{
    printf("head info : [%d,%d,%d]\n", input->headInfo.wrap, input->headInfo.lpos, input->headInfo.status);
    printf("io count = %d\n", input->ioVec.len);
    printf("io array:\n");
    for (int i = 0; i < input->ioVec.len; i++)
    {
        printf("io [%d] : [%d,%d,%d,%d]\n",
               i + 1, input->ioVec.ioArray[i].id, input->ioVec.ioArray[i].wrap,
               input->ioVec.ioArray[i].startLpos, input->ioVec.ioArray[i].endLpos);
    }
}

void PrintMetrics(const KeyMetrics *metrics)
{
    printf("\nKey Metrics:\n");
    printf("\talgorithmRunningDuration:\t %f ms\n", metrics->algorithmRunningDuration);
    printf("\taddressingDuration:\t\t %u ms\n", metrics->addressingDuration);
    printf("\treadDuration:\t\t\t %u ms\n", metrics->readDuration);
    printf("\ttapeBeltWear:\t\t\t %u\n", metrics->tapeBeltWear);
    printf("\ttapeMotorWear:\t\t\t %u\n", metrics->tapeMotorWear);

    // for (int i = 0; i < MAX_LPOS; i++)
    // {
    //     if (metrics->lposPassTime[i] == 0)
    //         continue;
    //     printf("\tlposPassTime[%d]: %d\n", i, metrics->lposPassTime[i]);
    // }
}

void test(const InputParam *input, OutputParam *output)
{
    HeadInfo start;
    HeadInfo end;
    int32_t seekT = 0;
    int32_t beltW = 0;
    int32_t motorW = 0;
    uint32_t rwT;

    KeyMetrics result = {0};
    AccessTime accessTime = {0};
    TapeBeltSegWearInfo segWearInfo = {0};

    start.wrap = input->headInfo.wrap;
    start.lpos = input->headInfo.lpos;
    start.status = input->headInfo.status;

    for (int i = 0; i < output->len; i++)
    {
        int curIO = output->sequence[i] - 1;
        end.wrap = input->ioVec.ioArray[curIO].wrap;
        end.lpos = input->ioVec.ioArray[curIO].startLpos;
        end.status = HEAD_RW;

        INFO("seek from wrap %d, lpos %d to wrap %d, lpos %d\n",
             start.wrap, start.lpos, end.wrap, end.lpos);

        DEBUG("start.wrap: %d, start.lpos: %d, start.status: %d\n", start.wrap, start.lpos, start.status);
        DEBUG("end.wrap: %d, end.lpos: %d, end.status: %d\n", end.wrap, end.lpos, end.status);
        seekT = SeekTimeCalculate(&start, &end);   // 寻址时间
        beltW = BeltWearTimes(&start, &end, NULL); // 带体磨损
        motorW = MotorWearTimes(&start, &end);     // 电机磨损
        DEBUG("seekT: %d ms, beltW: %d lpos, motorW: %d lpos\n", seekT, beltW, motorW);

        result.addressingDuration += seekT;
        result.tapeBeltWear += beltW;
        result.tapeMotorWear += motorW;

        start.wrap = input->ioVec.ioArray[curIO].wrap;
        start.lpos = input->ioVec.ioArray[curIO].startLpos;
        start.status = HEAD_RW;
        end.wrap = input->ioVec.ioArray[curIO].wrap;
        end.lpos = input->ioVec.ioArray[curIO].endLpos;
        end.status = HEAD_RW;
        INFO("read from wrap %d, lpos %d to wrap %d, lpos %d\n",
             start.wrap, start.lpos, end.wrap, end.lpos);
        beltW = BeltWearTimes(&start, &end, NULL); // 带体磨损
        motorW = MotorWearTimes(&start, &end);     // 电机磨损
        DEBUG("beltW: %d lpos, motorW: %d lpos\n", beltW, motorW);

        result.tapeBeltWear += beltW;
        result.tapeMotorWear += motorW;

        rwT = ReadTimeCalculate(abs(start.lpos - end.lpos));
        DEBUG("rwT: %d ms\n", rwT);
        printf("\n");
        start = end;
    }
    printf("addressingDuration: %ld ms, tapeBeltWear: %ld lpos, tapeMotorWear: %ld lpos\n",
           result.addressingDuration, result.tapeBeltWear, result.tapeMotorWear);

    printf("\n");
    DEBUG("start tot Calculate:\n\n");
    // PrintInputParam(input);
    // printf("output sequence: [");
    // for (uint32_t i = 0; i < output->len; i++)
    // {
    //     printf("%d, ", output->sequence[i]);
    // }

    /* 访问时间 */
    TotalAccessTime(input, output, &accessTime);
    result.addressingDuration = accessTime.addressDuration;
    result.readDuration = accessTime.readDuration;

    /* 带体磨损 */
    result.tapeBeltWear = TotalTapeBeltWearTimes(input, output, &segWearInfo);
    memcpy(&result.lposPassTime, segWearInfo.segWear, sizeof(segWearInfo.segWear));

    /* 电机磨损 */
    result.tapeMotorWear = TotalMotorWearTimes(input, output);

    PrintMetrics(&result);

    printf("]\n");
}

void swap(uint32_t *a, uint32_t *b)
{
    uint32_t temp = *a;
    *a = *b;
    *b = temp;
}

#define MAX_RESULTS 100

uint64_t processed_count = 0;
KeyMetrics best_result[MAX_RESULTS];
OutputParam oracle_seq[MAX_RESULTS];

void permute(uint32_t *arr, int l, int r, InputParam *input, void (*process)(InputParam *, OutputParam *))
{
    if (l == r)
    {
        OutputParam output = {.len = r + 1, .sequence = arr};
        process(input, &output);
        processed_count++;
        // if (processed_count % 1000 == 0)
        // {
        //     DEBUG("processed_count: %ld\n", processed_count);
        //     printf("output sequence: [");
        //     for (uint32_t i = 0; i < oracle_seq[0].len; i++)
        //     {
        //         printf("%d, ", oracle_seq[0].sequence[i]);
        //     }
        //     PrintMetrics(&best_result[0]);
        // }
    }
    else
    {
        for (int i = l; i <= r; i++)
        {
            swap(&arr[l], &arr[i]);
            permute(arr, l + 1, r, input, process);
            swap(&arr[l], &arr[i]);
        }
    }
}

int GetMissingNumbers(uint32_t N, uint32_t *A, uint32_t size)
{
    // 创建一个标记数组，用于标记 1 到 N 是否出现在 A[] 中
    uint32_t *flags = (uint32_t *)calloc(N + 1, sizeof(uint32_t)); // 初始化为 0

    // 标记数组 A[] 中出现的数字
    for (uint32_t i = 0; i < size; i++)
    {
        if (A[i] >= 1 && A[i] <= N)
        {
            flags[A[i]] = 1;
        }
    }

    // 统计未出现的数字
    uint32_t count = 0;
    for (int i = 1; i <= N; i++)
    {
        if (flags[i] == 0)
        {
            count++;
        }
    }

    // 释放标记数组的内存
    free(flags);

    return count;
}

void update_best_result(OutputParam *output, KeyMetrics result)
{
    // printf("output sequence: [");
    // for (uint32_t j = 0; j < output->len; j++)
    // {
    //     printf("%d, ", output->sequence[j]);
    // }
    // printf("]\n");
    for (int i = 0; i < MAX_RESULTS; i++)
    {
        if (result.addressingDuration + result.readDuration <
            best_result[i].addressingDuration + best_result[i].readDuration)
        {
            for (int j = MAX_RESULTS - 1; j > i; j--)
            {
                best_result[j] = best_result[j - 1];
                oracle_seq[j] = oracle_seq[j - 1];
            }
            best_result[i] = result;
            oracle_seq[i].len = output->len;
            oracle_seq[i].sequence = (uint32_t *)malloc(output->len * sizeof(uint32_t));
            memcpy(oracle_seq[i].sequence, output->sequence, output->len * sizeof(uint32_t));
            break;
        }
        else if (result.addressingDuration + result.readDuration ==
                 best_result[i].addressingDuration + best_result[i].readDuration)
        {
            if (result.tapeBeltWear + result.tapeMotorWear <
                best_result[i].tapeBeltWear + best_result[i].tapeMotorWear)
            {
                for (int j = MAX_RESULTS - 1; j > i; j--)
                {
                    best_result[j] = best_result[j - 1];
                    oracle_seq[j] = oracle_seq[j - 1];
                }
                best_result[i] = result;
                oracle_seq[i].len = output->len;
                oracle_seq[i].sequence = (uint32_t *)malloc(output->len * sizeof(uint32_t));
                memcpy(oracle_seq[i].sequence, output->sequence, output->len * sizeof(uint32_t));
                break;
            }
        }
    }
}

void processOutput(InputParam *input, OutputParam *output)
{
    // 处理每个排列
    // printf("序列: ");
    // for (int i = 0; i < output->len; i++)
    // {
    //     printf("%d ", output->sequence[i]);
    // }
    // printf("\n");

    KeyMetrics result = {0};
    AccessTime accessTime = {0};
    TapeBeltSegWearInfo segWearInfo = {0};

    result.errorIOCount = GetMissingNumbers(input->ioVec.len, output->sequence, output->len);
    if (result.errorIOCount > 0)
    {
        printf("序列: ");
        for (int i = 0; i < output->len; i++)
        {
            printf("%d ", output->sequence[i]);
        }
        ERROR("errorIOCount: %d\n", result.errorIOCount);
    }

    /* 访问时间 */
    TotalAccessTime(input, output, &accessTime);
    result.addressingDuration = accessTime.addressDuration;
    result.readDuration = accessTime.readDuration;

    /* 带体磨损 */
    result.tapeBeltWear = TotalTapeBeltWearTimes(input, output, &segWearInfo);
    memcpy(&result.lposPassTime, segWearInfo.segWear, sizeof(segWearInfo.segWear));

    /* 电机磨损 */
    result.tapeMotorWear = TotalMotorWearTimes(input, output);

    // DEBUG("addressingDuration: %ld ms, tapeBeltWear: %ld lpos, tapeMotorWear: %ld lpos\n",
    //       result.addressingDuration, result.tapeBeltWear, result.tapeMotorWear);

    update_best_result(output, result);
}

void oracle(const InputParam *input, OutputParam *output)
{
    for (int i = 0; i < MAX_RESULTS; i++)
    {
        best_result[i].algorithmRunningDuration = 0x3f3f3f3f;
        best_result[i].addressingDuration = 0x3f3f3f3f;
        best_result[i].readDuration = 0x3f3f3f3f;
        best_result[i].tapeBeltWear = 0x3f3f3f3f;
        best_result[i].tapeMotorWear = 0x3f3f3f3f;
        best_result[i].errorIOCount = 0;
    }

    permute(output->sequence, 0, output->len - 1, input, processOutput);

    printf("best results:\n");
    for (int i = 0; i < MAX_RESULTS; i++)
    {
        printf("rank %d\n", i + 1);
        printf("output sequence: [");
        for (uint32_t j = 0; j < oracle_seq[i].len; j++)
        {
            printf("%d, ", oracle_seq[i].sequence[j]);
        }
        printf("]");
        PrintMetrics(&best_result[i]);
        printf("\n");
    }
}

int main()
{
    // freopen("../best_results_case2_new.txt", "w", stdout);

    // output1.sequence[0] = 1;
    // output1.sequence[1] = 5;
    // output1.sequence[2] = 3;
    // output1.sequence[3] = 4;
    // output1.sequence[4] = 2;
    // test(&case1, &output1);
    // test(&case1_new, &output1_new);
    // test(&case2, &output2);
    test(&case2_new, &output2_new);

    // oracle(&case2_new, &output2_new);
    // oracle(&case2, &output2);
    // oracle(&case2_new, &output2_new);
    // interact();
}