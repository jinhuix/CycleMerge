
#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "../public.h"
#include <sys/time.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct{
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

    typedef struct {
    const InputParam *input;
    OutputParam *output;
    AccessTime *accessTime;
    } ThreadArg;
    
    // 定义线程返回值结构体
    typedef struct {
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
#ifdef __cplusplus
}
#endif

#endif // ALGORITHM_H