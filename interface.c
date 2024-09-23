#include <stdint.h>
#include <stdio.h>
#include "interface.h"


#define W1 1
#define W2 0 
#define W3 0

uint32_t CostCalculate(const HeadInfo *start, const HeadInfo *target, TapeBeltSegWearInfo *segWearInfo){
    int32_t seekT = 0;
    int32_t beltW = 0;
    int32_t motorW = 0;
    int32_t cost = 0;

    seekT = SeekTimeCalculate(&start, &target);   // 寻址时间
    beltW = BeltWearTimes(&start, &target, &segWearInfo); // 带体磨损
    motorW = MotorWearTimes(&start, &target);     // 电机磨损
    cost = W1*seekT + W2*beltW + W3*motorW;
    return cost;

}

uint32_t TotalCostCalculate(const InputParam *input, const OutputParam *output, TapeBeltSegWearInfo *segWearInfo){
    int32_t total_seekT = 0;
    int32_t total_beltW = 0;
    int32_t total_motorW = 0;
    int32_t total_cost = 0;
    AccessTime accessTime = {0};
    TotalAccessTime(input, output,  &accessTime);
    total_seekT = accessTime.addressDuration;
    total_beltW = TotalTapeBeltWearTimes(input, output, &segWearInfo);
    total_motorW = TotalMotorWearTimes(input, output);
    total_cost = W1*total_seekT + W2*total_beltW + W3*total_motorW;
    return total_cost;
}

