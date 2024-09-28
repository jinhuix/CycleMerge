
#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "../public.h"

#ifdef __cplusplus
extern "C" {
#endif

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

typedef struct {
    int capacity;
    int size;
    MinHeap **heap_array;
} MinHeapArray;

int32_t AlgorithmRun(const InputParam *input, OutputParam *output);

void QuickSort(IOUint *a, int len);

int32_t partition_scan(const InputParam *input, OutputParam *output);

int32_t Sort_Scan_MPScan_Perpartition(OutputParam *output, IOUint *sortedIOs, bool *vis, HeadInfo *head, int partition_start, int partition_len, const int scan_method);

int32_t MPScan_star_PerPartition(const InputParam *input, OutputParam *output, IOUint *sortedIOs, bool *vis, HeadInfo *head, int partition_start, int partition_end);

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

#ifdef __cplusplus
}
#endif

#endif  // ALGORITHM_H