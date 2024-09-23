#include <stdint.h>
#include <stdio.h>
#include "public.h"

/**
 * @brief  寻道、带体磨损、电机磨碎的加权成本
 * @param  start            起始位置和状态
 * @param  target           目标位置和状态
 * @param  segWearInfo      带体的分段磨损信息，即记录磁头划过每个lpos的次数
 * @return uint32_t         寻道(ms)、带体磨损(次数)、电机磨碎(ms)的加权成本
 */
uint32_t CostCalculate(const HeadInfo *start, const HeadInfo *target, TapeBeltSegWearInfo *segWearInfo);

/**
 * @brief  统计电机磨损次数，即，启、停各计数1次，掉头计数2次，
 * @param  input            磁头和批量的IO信息
 * @param  output           排序后的IO序列
 * @return uint32_t         电机磨损次数
 */
uint32_t TotalCostCalculate(const InputParam *input, const OutputParam *output, TapeBeltSegWearInfo *segWearInfo);

