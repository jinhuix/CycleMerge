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

    // 存储已生成的IO请求
    uint32_t io_requests[io_count][4]; // [id, wrap, startLpos, endLpos]
    uint32_t generated_count = 0;

    for (uint32_t i = 1; i <= io_count; ++i)
    {
        uint32_t io_wrap, io_startLpos, io_endLpos, data_length;
        uint32_t min_len = 10, max_len = 2000;
        int valid = 0;

        while (!valid)
        {
            io_wrap = rand() % MAX_WRAP;
            io_startLpos = rand() % (MAX_LPOS - max_len);
            data_length = rand() % max_len;
            data_length = data_length < min_len ? min_len : data_length;
            io_endLpos = io_startLpos + data_length;

            if (io_wrap & 1)
            {
                // 奇数wrap，起始位置大于结束位置
                if (io_startLpos < io_endLpos)
                {
                    uint32_t temp = io_startLpos;
                    io_startLpos = io_endLpos;
                    io_endLpos = temp;
                }
            }

            // 检查是否与之前生成的IO请求有重叠
            valid = 1;
            for (uint32_t j = 0; j < generated_count; ++j)
            {
                if (io_requests[j][1] == io_wrap &&
                    ((io_startLpos >= io_requests[j][2] && io_startLpos <= io_requests[j][3]) ||
                     (io_endLpos >= io_requests[j][2] && io_endLpos <= io_requests[j][3]) ||
                     (io_startLpos <= io_requests[j][2] && io_endLpos >= io_requests[j][3])))
                {
                    valid = 0;
                    break;
                }
            }
        }

        // 存储生成的IO请求
        io_requests[generated_count][0] = i;
        io_requests[generated_count][1] = io_wrap;
        io_requests[generated_count][2] = io_startLpos;
        io_requests[generated_count][3] = io_endLpos;
        ++generated_count;

        if (io_wrap & 1)
        {
            // 奇数wrap，起始位置大于结束位置
            if (io_startLpos < io_endLpos)
            {
                uint32_t temp = io_startLpos;
                io_startLpos = io_endLpos;
                io_endLpos = temp;
            }
        }
        else
        {
            // 偶数wrap，起始位置小于结束位置
            if (io_startLpos > io_endLpos)
            {
                uint32_t temp = io_startLpos;
                io_startLpos = io_endLpos;
                io_endLpos = temp;
            }
        }

        // 打印IO请求
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
        snprintf(filename, sizeof(filename), "../dataset/gen/gen_case%u_io%u.txt", i, io_nums);
        printf("Generating %s...\n", filename);
        generate_sample(io_nums, filename);
    }

    return 0;
}