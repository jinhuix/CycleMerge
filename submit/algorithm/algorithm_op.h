
#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "../public.h"
#include <sys/time.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        struct timeval start;
    } TimeRecord;

    static TimeRecord g_TimeRecord;

    void startRecordTime();

    int32_t getDurationMicroseconds();

    typedef struct
    {
        int id;
        struct req_node *next;
    } req_node;

    typedef struct
    {
        req_node *head;
        req_node *tail;
        int len;
    } req_list;

    typedef struct
    {
        int x, y, dis;
    } Node;

    typedef struct
    {
        Node *nodes;
        int size;
        int capacity;
    } MinHeap;

    typedef struct
    {
        int capacity;
        int size;
        MinHeap **heap_array;
    } MinHeapArray;

    typedef struct
    {
        const InputParam *input;
        OutputParam *output;
        AccessTime *accessTime;
    } ThreadArg;

    // 定义线程返回值结构体
    typedef struct
    {
        OutputParam *output;
        AccessTime accessTime;
    } ThreadResult;

    int32_t AlgorithmRun(const InputParam *input, OutputParam *output);

    void QuickSort(IOUint *a, int len);

    int32_t partition_scan(const InputParam *input, OutputParam *output);

    int32_t Sort_Scan_MPScan_Perpartition(OutputParam *output, IOUint *sortedIOs, bool *vis, HeadInfo *head, int partition_start, int partition_len, const int scan_method);

    int32_t MPScan_star_PerPartition(const InputParam *input, OutputParam *output, IOUint *sortedIOs, bool *vis, HeadInfo *head, int partition_start, int partition_end);

    int32_t MPScan(const InputParam *input, OutputParam *output);

    MinHeap *createMinHeap(int capacity);
    void swap(Node *a, Node *b);
    void heapify(MinHeap *heap, int idx);
    Node *extractMin(MinHeap *heap);
    Node *getMin(MinHeap *heap);
    void insertHeap(MinHeap *heap, Node node);
    void initUnionSet();
    void freeUnionSet();
    int find(int x);
    void unite(int x, int y);
    void destoryMinHeap(MinHeap *heap);
    int32_t merge(const InputParam *input, OutputParam *output);
    Node *randomExtractMin(MinHeap *heap);
    int32_t merge_random(const InputParam *input, OutputParam *output);
    void *merge_thread(void *arg);
    void *merge_random_thread(void *arg);
    void *partition_scan_merge_thread(void *arg);
    void *mp_scan_merge_random_thread(void *arg);
    int32_t SimulatedAnnealing(const InputParam *input, OutputParam *output);
    int32_t NearestNeighborAlgorithm(const InputParam *input, OutputParam *output);



#define INF 0x3f3f3f3f
uint32_t getCost(const HeadInfo *start, const HeadInfo *target);
int get_distance(int i, int j);
int min(int a, int b);
int max(int a, int b);
#ifndef _FIBONACCI_HEAP_H_
#define _FIBONACCI_HEAP_H_

typedef int Type;

typedef struct _FibonacciNode
{
    Type   key;                        // 关键字(键值)
    int degree;                        // 度数
    int value;                           //实际存储的值
    struct _FibonacciNode *left;    // 左兄弟
    struct _FibonacciNode *right;    // 右兄弟
    struct _FibonacciNode *child;    // 第一个孩子节点
    struct _FibonacciNode *parent;    // 父节点
    int marked;                       //是否被删除第1个孩子(1表示删除，0表示未删除)
}FibonacciNode, FibNode;

typedef struct _FibonacciHeap{
    int   keyNum;                    // 堆中节点的总数
    int   maxDegree;                // 最大度
    struct _FibonacciNode *min;        // 最小节点(某个最小堆的根节点)
    struct _FibonacciNode **cons;    // 最大度的内存区域
}FibonacciHeap, FibHeap;

// 创建Fibonacci堆
FibHeap* fib_heap_make();
// 新建键值为key的节点，并将其插入到斐波那契堆中
static FibNode* fib_heap_insert_key(FibHeap *heap, Type key, Type value);
// 删除键值为key的结点
void fib_heap_delete(FibHeap *heap, Type key);
// 移除最小节点
void fib_heap_extract_min(FibHeap *heap);
// 更新heap的中的oldkey为newkey
void fib_heap_update(FibHeap *heap, Type oldkey, Type newkey);
// 将h1, h2合并成一个堆，并返回合并后的堆
FibHeap* fib_heap_union(FibHeap *h1, FibHeap *h2);
// 在斐波那契堆heap中是否存在键值为key的节点；存在返回1，否则返回0。
int fib_heap_contains(FibHeap *heap, Type key);
// 获取最小节点对应的值(保存在pkey中)；成功返回1，失败返回0。
int fib_heap_get_min(FibHeap *heap, Type *pkey);
// 销毁斐波那契堆
void fib_heap_destroy(FibHeap *heap);
// 打印"斐波那契堆"
void fib_print(FibHeap *heap);
static void fib_node_destroy(FibNode *node);
static void fib_heap_decrease(FibHeap *heap, FibNode *node, Type key);
static void fib_heap_increase(FibHeap *heap, FibNode *node, Type key);

#endif

struct KM{
    // int match[MAXN], lx[MAXN],ly[MAXN], slack[MAXN], fa[MAXN*2];
    // bool visx[MAXN],visy[MAXN];
    int * match;
    int * lx;
    int * ly;
    bool * visx;
    bool * visy;
    bool * visit_node;
    int * fa;
    int * next;
    int * slack;
    int n, nx, ny;
    int need_n;
    FibHeap* slack_heap;
    FibNode** slack_nodes;
};

int kmGetDistance(int i, int j);
void kmInit(struct KM * cthis, int n, int nx, int ny);
void kmClear(struct KM * cthis);
// int kmFindpath(struct KM * cthis, int x);
void kmMain(struct KM * cthis);
void kmSolve(struct KM * cthis);

void cycleMergeLink(struct KM * cthis, int i, int j);
void cycleMergeFindMinimalMerge(struct KM * cthis, int * visited_start_p, int * visited_end_p, int * unvisited_start_p, int * unvisited_end_p);
void cycleMergeMain(struct KM * cthis);
 

#ifdef __cplusplus
}
#endif

#endif // ALGORITHM_H