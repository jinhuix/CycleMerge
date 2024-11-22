#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <algorithm>
#include <iostream>
#include <random>
#include <vector>

#define MIN_WRAP 0
// #define MAX_WRAP 280
#define MAX_WRAP 279
#define MIN_LPOS 0
// #define MAX_LPOS 730994
#define MAX_LPOS 728994
#define MAX_IO_NUM 10000
#define MIN_IO_NUM 10

class Generator {
   public:
    int n;
    std::vector<int> wrap;
    // std::vector<double> lpos;
    std::vector<int> lpos;
    int last_wrap = -1, last_lpos = -1;

    std::mt19937 random;
    std::normal_distribution<double> normal_dist;
    std::uniform_int_distribution<int> uniform_int_dist1;
    std::uniform_int_distribution<int> uniform_int_dist2;

    int randomIntBetween(int lower, int upper) {
        std::uniform_int_distribution<int> dist(lower, upper);
        return dist(random);
    }

    double randomDoubleBetween(double lower, double upper) {
        std::uniform_real_distribution<double> dist(lower, upper);
        return dist(random);
    }

    void generateRandom() {
        for (int i = 0; i < n; ++i) {
            wrap[i] = randomIntBetween(MIN_WRAP, MAX_WRAP);
            lpos[i] = randomIntBetween(MIN_LPOS, MAX_LPOS);
        }
    }

    void generateSequentialOne(int &wrap, int &lpos, int io_nums) {
        if (last_wrap == MAX_WRAP && MAX_LPOS - last_lpos <= 2000) {
            generateRandomOne(wrap, lpos);
            return;
        }
        if (last_wrap == -1 || ((last_wrap % 2 == 0 && MAX_LPOS - last_lpos <= 2000) || (last_wrap % 2 == 1 && last_lpos <= 2000))) {
            wrap = last_wrap + 1;
            if (wrap & 1)
                last_lpos = MAX_LPOS;
            else
                last_lpos = 0;
        } else {
            wrap = last_wrap;
        }
        if (wrap & 1) {
            int ran_num = randomIntBetween(20000000 / io_nums, 50000000 / io_nums);
            if (last_lpos < ran_num)
                lpos = 0;
            else
                lpos = last_lpos - ran_num;
        } else {
            int ran_num = randomIntBetween(20000000 / io_nums, 50000000 / io_nums);
            if (last_lpos + ran_num > MAX_LPOS - 2000)
                lpos = MAX_LPOS - 2000;
            else
                lpos = last_lpos + ran_num;
        }
        last_wrap = wrap;
        last_lpos = lpos;
    }

    void generateRandomOne(int &wrap, int &lpos) {
        wrap = uniform_int_dist1(random);
        lpos = uniform_int_dist2(random);
    }

    void generateNormalOne(int &wrap, int &lpos) {
        wrap = randomIntBetween(MIN_WRAP, MAX_WRAP);
        lpos = normal_dist(random);
        while (lpos <= MIN_LPOS || lpos >= MAX_LPOS) {
            lpos = normal_dist(random);
        }
    }

    void generateGaussian() {
        // 正态分布，其均值为 MAX_D / 2，标准差为 MAX_D / 4
        std::normal_distribution<double> dist(MAX_LPOS / 2.0, MAX_LPOS / 4.0);
        for (int i = 0; i < n; ++i) {
            do {
                wrap[i] = randomIntBetween(MIN_WRAP, MAX_WRAP);
                lpos[i] = dist(random);
            } while (lpos[i] <= MIN_LPOS || lpos[i] >= MAX_LPOS);
        }
    }

    void shuffle() {
        for (int i = 0; i < n - 1; ++i) {
            int j = randomIntBetween(i, n - 1);
            std::swap(wrap[i], wrap[j]);
            std::swap(lpos[i], lpos[j]);
        }
    }

    void generateMixed() {
        int ind = 0;
        while (ind < n / 2) {
            int wrap_t = randomIntBetween(MIN_WRAP, MAX_WRAP);
            double lpos_t = randomDoubleBetween(MIN_LPOS, MAX_LPOS - 1);
            for (int delta = 0; delta < 10; ++delta) {
                if (ind < n) {
                    wrap[ind] = wrap_t;
                    lpos[ind] = lpos_t + delta / 10.0;
                    ++ind;
                }
            }
        }

        for (; ind < n; ++ind) {
            wrap[ind] = randomIntBetween(MIN_WRAP, MAX_WRAP);
            lpos[ind] = randomIntBetween(MIN_LPOS, MAX_LPOS);
        }

        shuffle();
    }

    void generate(long seed, int io_nums, int type = -1) {
        random.seed(seed);
        n = randomIntBetween(MIN_IO_NUM, MAX_IO_NUM);
        if (io_nums > 0)
            n = io_nums;
        wrap.resize(n);
        lpos.resize(n);

        if (type == -1)
            type = randomIntBetween(0, 2);
        switch (type) {
            case 0:
                generateRandom();
                uniform_int_dist1 = std::uniform_int_distribution<int>(MIN_WRAP, MAX_WRAP);
                uniform_int_dist2 = std::uniform_int_distribution<int>(MIN_LPOS, MAX_LPOS);
                break;
            case 1:
                generateGaussian();
                normal_dist = std::normal_distribution<double>(MAX_LPOS / 2.0, MAX_LPOS / 4.0);
                break;
            case 2:
                generateMixed();
                break;
            case 3:
                generateRandom();
                uniform_int_dist1 = std::uniform_int_distribution<int>(MIN_WRAP, MAX_WRAP);
                uniform_int_dist2 = std::uniform_int_distribution<int>(MIN_LPOS, MAX_LPOS);
                break;
        }
    }
};

void generate_sample(int io_nums, const char *filename, int type) {
    // 初始化随机数生成器
    srand(time(NULL));

    // 打开文件进行写入
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        perror("Failed to open file");
        exit(EXIT_FAILURE);
    }

    // 随机生成head信息
    uint32_t head_wrap = rand() % MAX_WRAP;
    uint32_t head_lpos = rand() % MAX_LPOS;
    uint32_t head_status = 1;  // 固定为1

    // 随机生成IO数量
    uint32_t io_count = MIN_IO_NUM + rand() % (MAX_IO_NUM - MIN_IO_NUM + 1);
    if (io_nums > 0)
        io_count = io_nums;
    Generator gen;
    gen.generate(59879, io_count, type);

    // 打印head信息
    fprintf(file, "[\"head\":\"wrap\",\"lpos\",\"status\"]\n");
    fprintf(file, "[%u,%u,%u]\n", head_wrap, head_lpos, head_status);

    // 打印IO数量
    fprintf(file, "[\"io count\"]\n");
    fprintf(file, "[%u]\n", io_count);

    // 打印IO请求
    fprintf(file, "[\"io\":\"id\",\"wrap\",\"startLpos\",\"endLpos\"]\n");

    // 存储已生成的IO请求
    uint32_t io_requests[io_count][4];  // [id, wrap, startLpos, endLpos]
    uint32_t generated_count = 0;

    for (uint32_t i = 1; i <= io_count; ++i) {
        int32_t io_wrap, io_startLpos, io_endLpos, data_length;
        int32_t min_len = 10, max_len = 2000;
        int valid = 0;

        while (!valid) {
            // io_wrap = rand() % MAX_WRAP;
            // io_startLpos = rand() % (MAX_LPOS - max_len);
            if (type == 0)
                gen.generateRandomOne(io_wrap, io_startLpos);
            else if (type == 1)
                gen.generateNormalOne(io_wrap, io_startLpos);
            else if (type == 2) {
                if (i > io_count / 2)
                    gen.generateRandomOne(io_wrap, io_startLpos);
                else
                    gen.generateNormalOne(io_wrap, io_startLpos);
            } else {
                gen.generateSequentialOne(io_wrap, io_startLpos, io_nums);
            }
            data_length = rand() % max_len;
            data_length = data_length < min_len ? min_len : data_length;
            io_endLpos = io_startLpos + data_length;

            if (io_wrap & 1) {
                // 奇数wrap，起始位置大于结束位置
                if (io_startLpos < io_endLpos) {
                    uint32_t temp = io_startLpos;
                    io_startLpos = io_endLpos;
                    io_endLpos = temp;
                }
            }

            // 检查是否与之前生成的IO请求有重叠
            valid = 1;
            for (uint32_t j = 0; j < generated_count; ++j) {
                if (io_requests[j][1] == io_wrap &&
                    ((io_startLpos >= io_requests[j][2] && io_startLpos <= io_requests[j][3]) ||
                     (io_endLpos >= io_requests[j][2] && io_endLpos <= io_requests[j][3]) ||
                     (io_startLpos <= io_requests[j][2] && io_endLpos >= io_requests[j][3]))) {
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

        if (io_wrap & 1) {
            // 奇数wrap，起始位置大于结束位置
            if (io_startLpos < io_endLpos) {
                uint32_t temp = io_startLpos;
                io_startLpos = io_endLpos;
                io_endLpos = temp;
            }
        } else {
            // 偶数wrap，起始位置小于结束位置
            if (io_startLpos > io_endLpos) {
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

int main() {
    printf("Please input the number of IO requests: ");
    int io_nums;
    char filename[256];
    scanf("%d", &io_nums);
    printf("Please input the io distribution type (0: random, 1: gaussian, 2: mixed, 3: sequential): ");
    int type;
    scanf("%d", &type);

    for (int i = 1; i <= 1; i++) {
        snprintf(filename, sizeof(filename), "../dataset/gen/gen_case%u_io%u_%s.txt",
                 i, io_nums, type == 0 ? "random" : type == 1 ? "gaussian"
                                                : type == 2   ? "mixed"
                                                              : "sequential");
        printf("Generating %s...\n", filename);
        generate_sample(io_nums, filename, type);
    }

    return 0;
}