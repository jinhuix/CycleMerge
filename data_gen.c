#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>

#define MAX_WRAP 280
#define MAX_LPOS 730994
#define MAX_IO_NUM 10000
#define MIN_IO_NUM 10

void generate_sample(int io_nums, const char *filename)
{
    // 初始化随机数生成器
    srand(time(NULL));

    // 打开文件进行写入
    FILE *file = fopen(filename, "w");
    if (file == NULL)
    {
        perror("Failed to open file");
        exit(EXIT_FAILURE);
    }

    // 随机生成head信息
    uint32_t head_wrap = rand() % MAX_WRAP;
    uint32_t head_lpos = rand() % MAX_LPOS;
    uint32_t head_status = 1; // 固定为1

    // 随机生成IO数量
    uint32_t io_count = MIN_IO_NUM + rand() % (MAX_IO_NUM - MIN_IO_NUM + 1);
    if (io_nums > 0)
        io_count = io_nums;

    // 打印head信息
    fprintf(file, "[\"head\":\"wrap\",\"lpos\",\"status\"]\n");
    fprintf(file, "[%u,%u,%u]\n", head_wrap, head_lpos, head_status);

    // 打印IO数量
    fprintf(file, "[\"io count\"]\n");
    fprintf(file, "[%u]\n", io_count);

    // 打印IO请求
    fprintf(file, "[\"io\":\"id\",\"wrap\",\"startLpos\",\"endLpos\"]\n");
    for (uint32_t i = 1; i <= io_count; ++i)
    {
        uint32_t io_wrap = rand() % MAX_WRAP;
        uint32_t min_len = 10, max_len = 2000;
        uint32_t io_startLpos = rand() % (MAX_LPOS - max_len);
        uint32_t data_lenth = rand() % max_len;
        uint32_t io_endLpos = io_startLpos + data_lenth < min_len ? min_len : data_lenth;

        if (io_wrap % 2 == 0)
        {
            // 偶数wrap，结束位置大于起始位置
            if (io_startLpos > io_endLpos)
            {
                uint32_t temp = io_startLpos;
                io_startLpos = io_endLpos;
                io_endLpos = temp;
            }
        }
        else
        {
            // 奇数wrap，起始位置大于结束位置
            if (io_startLpos < io_endLpos)
            {
                uint32_t temp = io_startLpos;
                io_startLpos = io_endLpos;
                io_endLpos = temp;
            }
        }

        fprintf(file, "[%u,%u,%u,%u]\n", i, io_wrap, io_startLpos, io_endLpos);
    }

    // 关闭文件
    fclose(file);
}

int main()
{
    printf("Please input the number of IO requests: ");
    int io_nums;
    char filename[256];
    scanf("%d", &io_nums);

    for (int i = 1; i <= 5; i++)
    {
        snprintf(filename, sizeof(filename), "../dataset/gen_case%u_io%u.txt", i, io_nums);
        printf("Generating %s...\n", filename);
        generate_sample(io_nums, filename);
    }

    return 0;
}