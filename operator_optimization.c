#include "public.h"

int32_t SimpleOperatorOptimization(const InputParam *input, OutputParam *output)
{
    /* Step2：通过算子优化，微调ouput的顺序，事例 */
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++)
    {
        output->sequence[i] = input->ioVec.ioArray[i].id;
    }
    return RETURN_OK;
}