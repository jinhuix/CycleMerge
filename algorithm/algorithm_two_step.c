#include "public.h"
#include "logging.h"
#include "interface.h"
#include "operator_optimization.h"

/**
 * @brief  算法接口
 * @param  input            输入参数
 * @param  output           输出参数
 * @return int32_t          返回成功或者失败，RETURN_OK 或 RETURN_ERROR
 */
int32_t IOScheduleAlgorithm(const InputParam *input, OutputParam *output)
{
    int32_t ret;

    /* Step1：先入先出算法 */
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++)
    {
        output->sequence[i] = input->ioVec.ioArray[i].id;
    }

    /* 调用公共函数示例：调用电机寻址、带体磨损、电机磨损函数 */
    HeadInfo start = {input->ioVec.ioArray[0].wrap, input->ioVec.ioArray[0].endLpos, HEAD_RW};
    HeadInfo end = {input->ioVec.ioArray[1].wrap, input->ioVec.ioArray[1].endLpos, HEAD_RW};
    int32_t cost = 0;

    TapeBeltSegWearInfo segWearInfo = {0};
    for (uint32_t i = 0; i < 10000; i++)
    {
        cost = CostCalculate(&start, &end, &segWearInfo);
    }

    /* 调用加权接口函数 */
    uint32_t total_cost = TotalCostCalculate(input, output, &segWearInfo);

    return RETURN_OK;
}

int32_t AlgorithmRun(const InputParam *input, OutputParam *output)
{
    int32_t ret1 = RETURN_ERROR, ret2 = RETURN_ERROR;

    // step1 : 调度算法
    ret1 = IOScheduleAlgorithm(input, output);
    for (uint32_t i = 0; i < output->len; i++)
    {
        printf("%d ", output->sequence[i]);
    }
    printf("\n");
    if(ret1 == RETURN_OK){
        // step2: 优化算子
        ret2 = OperatorOptimization(input, output);
    }
    
    if(ret1 == RETURN_OK & ret2 == RETURN_OK){
        // step1, step2都成功
        return RETURN_OK;
    }
    return RETURN_ERROR;
}
