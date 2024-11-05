
#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "../public.h"
#include <sys/time.h>

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

// 斐波那契堆节点结构
typedef struct FibNode {
    int key;           // 关键字值
    int value;         // 实际存储的值
    int degree;        // 子节点个数
    bool mark;         // 标记是否失去过子节点
    struct FibNode *parent;    // 父节点
    struct FibNode *child;     // 子节点
    struct FibNode *left;      // 左兄弟
    struct FibNode *right;     // 右兄弟
} FibNode;

// 斐波那契堆结构
typedef struct FibHeap {
    FibNode *min;      // 最小节点
    int n;             // 节点总数
} FibHeap;

FibNode* createFibNode(int key, int value);
FibHeap* createFibHeap();
void fibHeapLink(FibHeap* heap, FibNode* y, FibNode* x);
void consolidate(FibHeap* heap);
FibNode* fibHeapInsert(FibHeap* heap, int key, int value);
void fibHeapDecreaseKey(FibHeap* heap, FibNode* x, int new_key);
void cut(FibHeap* heap, FibNode* x, FibNode* y);
void cascadingCut(FibHeap* heap, FibNode* y);
FibNode* fibHeapExtractMin(FibHeap* heap);

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
int kmFindpath(struct KM * cthis, int x);
void kmMain(struct KM * cthis);
void kmSolve(struct KM * cthis);

void cycleMergeLink(struct KM * cthis, int i, int j);
void cycleMergeFindMinimalMerge(struct KM * cthis, int * visited_start_p, int * visited_end_p, int * unvisited_start_p, int * unvisited_end_p);
void cycleMergeMain(struct KM * cthis);
 

#ifdef __cplusplus
}
#endif

#endif // ALGORITHM_H