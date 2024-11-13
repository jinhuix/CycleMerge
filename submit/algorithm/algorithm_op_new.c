#include "algorithm_op.h"
#include "watchdog.h"
#include "hungarian_c.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <math.h>
#include <float.h>
#include <assert.h>
#include <signal.h>
#define MAX_TRIALS 200       // 最大尝试次数
#define NUM_EMPLOYED_BEES 40 // 雇佣蜂的数量
#define NUM_ONLOOKER_BEES 20 // 跟随蜂的数量
#define MAX_ITERATIONS 1000  // 最大迭代次数

#if 0
#define LOG2(x) ({ \
        unsigned int _i = 0; \
        __asm__("bsr %1, %0" : "=r" (_i) : "r" ((x))); \
        _i; })
#else   // 注意：通过gcc编译时，要添加 -lm 选项。
#define LOG2(x) ((log((double)(x))) / (log(2.0)))
#endif

const InputParam *g_input;
int * dist_martix = NULL;
CostWeights *cost_weight;

void initCostWeight(const InputParam *input, OutputParam *output) {
    // 执行SCAN算法，统计时间
    struct timeval start, end;
    gettimeofday(&start, NULL);
    SCAN(input, output);
    gettimeofday(&end, NULL);
    // 算法执行时间（ms）
    uint32_t b_at = ((end.tv_sec - start.tv_sec)*1000000 + (end.tv_usec - start.tv_usec)) / 1000.0;

    AccessTime accessTime = {0};
    TotalAccessTime(input, output, &accessTime);                            // 基线寻址时长
    cost_weight->b_st = accessTime.addressDuration;
    cost_weight->b_bw = TotalTapeBeltWearTimes(input, output, NULL);        // 带体磨损
    cost_weight->b_mw = TotalMotorWearTimes(input, output);                 // 电机磨损
    cost_weight->b_rt = cost_weight->b_st + accessTime.readDuration + b_at; // 基线读时延

    // 场景：高性能hdd
    cost_weight->alpha = 0.5 / cost_weight->b_rt;
    cost_weight->beta = 0.3 / cost_weight->b_bw;
    cost_weight->gamma = 0.2 / cost_weight->b_mw;
}

int32_t SCAN(const InputParam *input, OutputParam *output) {
    // 初始化输出参数
    output->len = input->ioVec.len;

    // 复制 IO 请求数组并按 lpos 排序
    IOUint *sortedIOs = (IOUint *)malloc(input->ioVec.len * sizeof(IOUint));
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
        sortedIOs[i] = input->ioVec.ioArray[i];
    QuickSort(sortedIOs, input->ioVec.len);

    // 初始化当前头位置为输入的头状态
    // HeadInfo currentHead = {input->headInfo.wrap, input->headInfo.lpos, input->headInfo.status};
    HeadInfo currentHead = {0, 0, 0};

    // 扫描方向：1 表示从 BOT 向 EOT 扫描，-1 表示从 EOT 向 BOT 扫描
    int direction = 1;
    uint32_t index = 0;

    bool vis[input->ioVec.len + 1];
    memset(vis, 0, sizeof(vis));
    int cnt = 0;
    while (index < input->ioVec.len) {
        if (direction == 1) {
            // 从 BOT 向 EOT 扫描
            for (uint32_t i = 0; i < input->ioVec.len; ++i) {
                if (sortedIOs[i].wrap & 1 || vis[sortedIOs[i].id])
                    continue;
                if (sortedIOs[i].startLpos >= currentHead.lpos) {
                    output->sequence[index++] = sortedIOs[i].id;
                    vis[sortedIOs[i].id] = 1;
                    currentHead.wrap = sortedIOs[i].wrap;
                    // currentHead.lpos = sortedIOs[i].endLpos;
                    currentHead.lpos = sortedIOs[i].startLpos;
                }
            }
            direction = -1; // 改变扫描方向
            currentHead.lpos = MAX_LPOS;
            cnt++;
        }
        else {
            // 从 EOT 向 BOT 扫描
            for (int32_t i = input->ioVec.len - 1; i >= 0; --i) {
                if (!(sortedIOs[i].wrap & 1) || vis[sortedIOs[i].id])
                    continue;
                if (sortedIOs[i].startLpos <= currentHead.lpos) {
                    output->sequence[index++] = sortedIOs[i].id;
                    vis[sortedIOs[i].id] = 1;
                    currentHead.wrap = sortedIOs[i].wrap;
                    // currentHead.lpos = sortedIOs[i].endLpos;
                    currentHead.lpos = sortedIOs[i].startLpos;
                }
            }
            direction = 1; // 改变扫描方向
            currentHead.lpos = 0;
        }
    }

    free(sortedIOs);
    // DEBUG("SCAN 扫描了来回%d趟\n", cnt);
    return RETURN_OK;
}

int _get_distance(int i, int j){
    if(i == j){
        return -INF;
    }

    HeadInfo currentHead, targetPos;
    

    if(i == 1){
        // head
        currentHead = g_input->headInfo;
    }
    else{
        currentHead.lpos = g_input->ioVec.ioArray[i-2].endLpos;
        currentHead.wrap = g_input->ioVec.ioArray[i-2].wrap;
        currentHead.status = HEAD_RW;
    }

    if(j == 1){
        // head
        targetPos = g_input->headInfo;
        return -0;
    }
    else{
        targetPos.lpos = g_input->ioVec.ioArray[j-2].startLpos;
        targetPos.wrap = g_input->ioVec.ioArray[j-2].wrap;
        targetPos.status = HEAD_RW;
    }

    // TotalTapeBeltWearTimes(&currentHead, &targetPos, NULL);  // 带体磨损
    return -getCost(&currentHead, &targetPos);
}

int kmGetDistance(int i, int j){
    if(dist_martix == NULL){
        dist_martix = (int *)malloc(sizeof(int)*(g_input->ioVec.len+2)*(g_input->ioVec.len+2));
        memset(dist_martix, 0, sizeof(int)*(g_input->ioVec.len+2)*(g_input->ioVec.len+2));

        for(int i=1;i<=g_input->ioVec.len + 1;i++){
            for(int j=1;j<=g_input->ioVec.len + 1;j++){
                ;
                dist_martix[i*(g_input->ioVec.len+2)+j] = _get_distance(i, j);
            }
        }
    }
    return dist_martix[i*(g_input->ioVec.len+2)+j];
    // return _get_distance(i, j);
}

int min(int a, int b)
{
    return a<b?a:b;
}

int max(int a, int b)
{
    return a>b?a:b;
}

void kmInit(struct KM * cthis, int n, int nx, int ny){
    int need_n = n + 100;
    cthis->match = (int *)malloc(sizeof(int)*(need_n));
    cthis->lx = (int *)malloc(sizeof(int)*(need_n));
    cthis->ly = (int *)malloc(sizeof(int)*(need_n));
    cthis->visx = (bool *)malloc(sizeof(bool)*(need_n));
    cthis->visy = (bool *)malloc(sizeof(bool)*(need_n));
    cthis->fa = (int *)malloc(sizeof(int) * 2 * need_n);
    cthis->slack = (int *)malloc(sizeof(bool)*(need_n));
    cthis->visit_node = (bool *)malloc(sizeof(bool)*(need_n));
    cthis->next = (int *)malloc(sizeof(int) * need_n);

    cthis->n = n;
    cthis->nx = nx;
    cthis->ny = ny;
    cthis->need_n = need_n;

    // 初始化斐波那契堆
    cthis->slack_heap = fib_heap_make();
    cthis->slack_nodes = (FibNode **)malloc((cthis->nx + 1) * sizeof(FibNode *));
}

void kmClear(struct KM * cthis){
    // 添加斐波那契堆相关的内存释放
    fib_heap_destroy(cthis->slack_heap);
    free(cthis->match);
    free(cthis->lx);
    free(cthis->ly);
    free(cthis->visx);
    free(cthis->visy);
    free(cthis->fa);
    free(cthis->slack);
    free(cthis->visit_node);
    free(cthis->next);
}

int kmFindpath(struct KM* cthis, int x) {
    int tempDelta;
    cthis->visx[x] = true;
    for(int y = 1; y <= cthis->ny; y++) {
        if(cthis->visy[y]) continue;
        
        tempDelta = cthis->lx[x] + cthis->ly[y] - kmGetDistance(x, y);
        
        if(tempDelta == 0) {
            cthis->visy[y] = true;
            cthis->fa[y + cthis->nx] = x;
            
            if(cthis->match[y] == -1) {
                return y + cthis->nx;
            }
            
            cthis->fa[cthis->match[y]] = y + cthis->nx;
            int res = kmFindpath(cthis, cthis->match[y]);
            if(res > 0) return res;
        }
        else {
            // 更新斐波那契堆中对应点的slack值
            int current_slack = cthis->slack_nodes[x]->key;
            if(current_slack > tempDelta) {
                fib_heap_decrease(cthis->slack_heap, cthis->slack_nodes[x], tempDelta);
            }
        }
    }
    return -1;
}

void kmMain(struct KM* cthis) {
    // 初始化slack值到斐波那契堆
    for (int i = 1; i <= cthis->ny; i++) {  // 应该使用ny而不是nx
        cthis->slack_nodes[i] = fib_heap_insert_key(cthis->slack_heap, INF, i);
    }
    for(int x = 1; x <= cthis->nx; ++x) {
        for (int i = 1; i <= cthis->ny; i++) {
            fib_heap_increase(cthis->slack_heap, cthis->slack_nodes[i], INF);  // 重置 slack 值为 INF
        }
        // 初始化各数组
        for(int i = 1; i <= cthis->nx + cthis->ny; i++) {
            cthis->fa[i] = -1;
        }
        memset(cthis->visx, false, sizeof(bool) * cthis->need_n);
        memset(cthis->visy, false, sizeof(bool) * cthis->need_n);
        int fir = 1;
        int leaf = -1;
        while(true) {
            if(fir == 1) {
                leaf = kmFindpath(cthis, x);
                fir = 0;
            }
            else {
                // 使用斐波那契堆找到最小slack值的顶点
                FibNode *minNode;
                while(true) {
                    minNode = cthis->slack_heap->min;
                    if(minNode == NULL)
                        break;
                    if(minNode->key > 0)
                        break;
                    int i = minNode->value;
                    //slack要重新清空，方以后接着用
                    fib_heap_extract_min(cthis->slack_heap);
                    fib_heap_insert_key(cthis->slack_heap, INF, i);
                    // fib_heap_increase(cthis->slack_heap, cthis->slack_nodes[i], INF);
                    leaf = kmFindpath(cthis, i);
                    if(leaf > 0) break;
                }
            }
            if(leaf > 0) {
                int p = leaf;
                while(p > 0) {
                    cthis->match[p-cthis->nx] = cthis->fa[p];
                    p = cthis->fa[cthis->fa[p]];
                }
                break;
            }
            else {
                // 未找到增广路，更新顶标
                int delta = cthis->slack_heap->min->key;
                // 更新标号和slack值
                for(int i = 1; i <= cthis->nx; ++i) {
                    if(cthis->visx[i]) {
                        cthis->lx[i] -= delta;
                        // 更新对应的slack值
                        fib_heap_decrease(cthis->slack_heap, cthis->slack_nodes[i], cthis->slack_nodes[i]->key - delta);
                    }
                }
                for(int j = 1; j <= cthis->ny; ++j) {
                    if(cthis->visy[j]) {
                        cthis->ly[j] += delta;
                    }
                }
            }        
        }
    }
}
 
void kmSolve(struct KM * cthis)
{
    memset(cthis->match,-1,sizeof(int) * cthis->need_n);
    memset(cthis->ly,0,sizeof(int) * cthis->need_n);
 
    for(int i=1;i<=cthis->nx;i++)
    {
        cthis->lx[i]=-INF;
        for(int j=1;j<=cthis->ny;j++)
            cthis->lx[i]=max(cthis->lx[i],kmGetDistance(i, j));
    }
    kmMain(cthis);
}

void cycleMergeLink(struct KM * cthis, int i, int j){
    cthis->next[i] = j;
    cthis->match[j] = i;
}

void cycleMergeFindMinimalMerge(struct KM * cthis, int * visited_start_p, int * visited_end_p, int * unvisited_start_p, int * unvisited_end_p){
    int visited_start = -1;
    int visited_end = -1;
    int unvisited_start = -1;
    int unvisited_end = -1;
    int max_cost = -INF;
    for(int i = 1; i <= cthis->n; i++){
        if(cthis->visit_node[i]){
            for(int j = 1; j <= cthis->n; j++){
                if(!cthis->visit_node[j]){
                    int this_cost = 0;
                    this_cost += kmGetDistance(i, j);
                    this_cost += kmGetDistance(cthis->match[j], cthis->next[i]);
                    this_cost -= kmGetDistance(i, cthis->next[i]);
                    this_cost -= kmGetDistance(cthis->match[j], j);
                    if(visited_start == -1 || this_cost > max_cost){
                        max_cost = this_cost;
                        visited_start = i;
                        visited_end = cthis->next[i];
                        unvisited_start = j;
                        unvisited_end = cthis->match[j];
                    }
                }
            }
        }
    }
    *visited_start_p = visited_start;
    *visited_end_p = visited_end;
    *unvisited_start_p = unvisited_start;
    *unvisited_end_p = unvisited_end;
}

void cycleMergeMain(struct KM * cthis){
    int path_length = 0;
    int cycle_cnt = 1;
    int last_end = -1;
    memset(cthis->visit_node, 0, sizeof(bool) * cthis->need_n);
    for(int i = 1; i <= cthis->ny; i++){
        int left = cthis->match[i];
        cthis->next[left] = i;
    }

    int now_node = 1;
    
    int visit_cnt = 0;
    while(visit_cnt != cthis->n){
        // assert(cthis->visit_node[now_node] == 0);
        cthis->visit_node[now_node] = 1;
        visit_cnt ++;
        if(visit_cnt == cthis->n){
            break;
        }

        int next = cthis->next[now_node];
        if(cthis->visit_node[next]){
            cycle_cnt ++;
            int visited_start, visited_end;
            int unvisited_start, unvisited_end;
            cycleMergeFindMinimalMerge(cthis, &visited_start, &visited_end, &unvisited_start, &unvisited_end);

            cycleMergeLink(cthis, visited_start, unvisited_start);
            cycleMergeLink(cthis, unvisited_end, visited_end);
            path_length += kmGetDistance(visited_start, unvisited_start);
            path_length += kmGetDistance(unvisited_end, visited_end);
            path_length -= kmGetDistance(visited_start, visited_end);
            now_node = unvisited_start;
        }
        else{
            path_length += kmGetDistance(now_node, next);
            now_node = next;
        }
    }
    printf("path length = %d, cycle_cnt = %d\n", path_length, cycle_cnt);
}

static FibNode *fib_heap_search(FibHeap *heap, Type key);

/*
 * 将node从双链表移除
 */
static void fib_node_remove(FibNode *node)
{
    node->left->right = node->right;
    node->right->left = node->left;
}

/*
 * 将"单个节点node"加入"链表root"之前
 *   a …… root
 *   a …… node …… root
 *
 * 注意： 此处node是单个节点，而root是双向链表
*/
static void fib_node_add(FibNode *node, FibNode *root)
{
    node->left        = root->left;
    root->left->right = node;
    node->right       = root;
    root->left        = node;
}

/*
 * 将双向链表b链接到双向链表a的后面
 *
 * 注意： 此处a和b都是双向链表
*/
static void fib_node_cat(FibNode *a, FibNode *b)
{
    FibNode *tmp;

    tmp            = a->right;
    a->right       = b->right;
    b->right->left = a;
    b->right       = tmp;
    tmp->left      = b;
}


/*
 * 创建斐波那契堆
 */
FibHeap* fib_heap_make()
{
    FibHeap* heap;

    heap = (FibHeap *) malloc(sizeof(FibHeap));
    if (heap == NULL)
    {
        printf("Error: make FibHeap failed\n");
        return NULL;
    }

    heap->keyNum = 0;
    heap->maxDegree = 0;
    heap->min = NULL;
    heap->cons = NULL;

    return heap;
}

/*
 * 创建斐波那契堆的节点
 */
static FibNode* fib_node_make(Type key, Type value)
{
    FibNode * node;

    node = (FibNode *) malloc(sizeof(FibNode));
    if (node == NULL)
    {
        printf("Error: make Node failed\n");
        return NULL;
    }
    node->key    = key;
    node->degree = 0;
    node->left   = node;
    node->right  = node;
    node->parent = NULL;
    node->child  = NULL;
    node->value = value;

    return node;
}

/*
 * 将节点node插入到斐波那契堆heap中
 */
static void fib_heap_insert_node(FibHeap *heap, FibNode *node)
{
    if (heap->keyNum == 0)
        heap->min = node;
    else
       {
        fib_node_add(node, heap->min);
        if (node->key < heap->min->key)
            heap->min = node;
    }
    heap->keyNum++;
}

/*
 * 新建键值为key的节点，并将其插入到斐波那契堆中
 */
static FibNode* fib_heap_insert_key(FibHeap *heap, Type key, Type value)
{
    FibNode *node;

    if (heap==NULL)
        return ;

    node = fib_node_make(key, value);
    if (node == NULL)
        return ;

    fib_heap_insert_node(heap, node);
    return node;
}

/*
 * 将h1, h2合并成一个堆，并返回合并后的堆
 */
FibHeap* fib_heap_union(FibHeap *h1, FibHeap *h2)
{
    FibHeap *tmp;

    if (h1==NULL)
        return h2;
    if (h2==NULL)
        return h1;

    // 以h1为"母"，将h2附加到h1上；下面是保证h1的度数大，尽可能的少操作。
    if(h2->maxDegree > h1->maxDegree)
    {
        tmp = h1;
        h1 = h2;
        h2 = tmp;
    }

    if((h1->min) == NULL)                // h1无"最小节点"
    {
        h1->min = h2->min;
        h1->keyNum = h2->keyNum;
        free(h2->cons);
        free(h2);
    }
    else if((h2->min) == NULL)           // h1有"最小节点" && h2无"最小节点"
    {
        free(h2->cons);
        free(h2);
    }                                   // h1有"最小节点" && h2有"最小节点"
    else
    {
        // 将"h2中根链表"添加到"h1"中
        fib_node_cat(h1->min, h2->min);
        if (h1->min->key > h2->min->key)
            h1->min = h2->min;
        h1->keyNum += h2->keyNum;
        free(h2->cons);
        free(h2);
    }

    return h1;
}

/*
 * 将"堆的最小结点"从根链表中移除，
 * 这意味着"将最小节点所属的树"从堆中移除!
 */
static FibNode *fib_heap_remove_min(FibHeap *heap)
{
    // assert(heap->min->degree >= 0);
    FibNode *min = heap->min;
    if (heap->min == min->right)
        heap->min = NULL;
    else
    {
        fib_node_remove(min);
        heap->min = min->right;
    }
    min->left = min->right = min;
    
    return min;
}

/*
 * 将node链接到root根结点
 */
static void fib_heap_link(FibHeap * heap, FibNode * node, FibNode *root)
{
    assert(heap != NULL);
    assert(node != NULL);
    assert(root != NULL);
    
    // 将 node 从双链表中移除
    fib_node_remove(node);
    
    // 将 node 设为 root 的孩子
    if (root->child == NULL) {
        root->child = node;
    } else {
        fib_node_add(node, root->child);
    }

    node->parent = root;
    root->degree++;
    node->marked = 0;
}

/*
 * 创建fib_heap_consolidate所需空间
 */
static void fib_heap_cons_make(FibHeap *heap) {
    if (!heap) return;

    // 计算新的最大度数
    int new_max_degree = LOG2(heap->keyNum) + 1;

    // 如果需要更大的空间
    if (new_max_degree > heap->maxDegree) {
        // 分配新内存
        FibNode **new_cons = (FibNode **)realloc(heap->cons, 
                                                sizeof(FibNode *) * (new_max_degree + 1));
        if (!new_cons) {
            fprintf(stderr, "Memory allocation failed in fib_heap_cons_make\n");
            return;  // 返回错误而不是退出
        }

        // 初始化新分配的内存区域
        for (int i = heap->maxDegree; i <= new_max_degree; i++) {
            new_cons[i] = NULL;
        }

        heap->cons = new_cons;
        heap->maxDegree = new_max_degree;
    }
}

/*
 * 合并斐波那契堆的根链表中左右相同度数的树
 */
static void fib_heap_consolidate(FibHeap *heap)
{
    int i, d, D;
    FibNode *x, *y, *tmp;

    fib_heap_cons_make(heap);//开辟哈希所用空间
    D = heap->maxDegree + 1;

    for (i = 0; i < D; i++)
        heap->cons[i] = NULL;

    // 合并相同度的根节点，使每个度数的树唯一
    while (heap->min != NULL)
    {
        if(heap->min->degree == -1)
            printf('dada\n');
        x = fib_heap_remove_min(heap);    // 取出堆中的最小树(最小节点所在的树)
        d = x->degree;                    // 获取最小树的度数
        
        // heap->cons[d] != NULL，意味着有两棵树(x和y)的"度数"相同。
        while (heap->cons[d] != NULL)
        {
            y = heap->cons[d];            // y是"与x的度数相同的树"
            if (x->key > y->key)        // 保证x的键值比y小
            {
                tmp = x;
                x = y;
                y = tmp;

            }
            fib_heap_link(heap, y, x);    // 将y链接到x中
            heap->cons[d] = NULL;
            d++;
        }
        heap->cons[d] = x;
    }
    heap->min = NULL;

    // 将heap->cons中的结点重新加到根表中
    for (i=0; i<D; i++)
    {
        if (heap->cons[i] != NULL)
        {
            if (heap->min == NULL)
                heap->min = heap->cons[i];
            else
            {
                fib_node_add(heap->cons[i], heap->min);
                if ((heap->cons[i])->key < heap->min->key)
                    heap->min = heap->cons[i];
            }
        }
    }
}

/*
 * 移除最小节点，并返回移除节点后的斐波那契堆
 */
FibNode* _fib_heap_extract_min(FibHeap *heap)
{
    if (heap==NULL || heap->min==NULL)
        return NULL;

    FibNode *child = NULL;
    FibNode *min = heap->min;
    // 将min每一个儿子(儿子和儿子的兄弟)都添加到"斐波那契堆的根链表"中
    while (min->child != NULL)
    {
        child = min->child;
        fib_node_remove(child);
        if (child->right == child)
            min->child = NULL;
        else
            min->child = child->right;

        fib_node_add(child, heap->min);
        child->parent = NULL;
    }
    // 将min从根链表中移除
    fib_node_remove(min);
    // 若min是堆中唯一节点，则设置堆的最小节点为NULL；
    // 否则，设置堆的最小节点为一个非空节点(min->right)，然后再进行调节。
    if (min->right == min)
        heap->min = NULL;
    else
    {
        heap->min = min->right;
        fib_heap_consolidate(heap);
    }
    heap->keyNum--;

    return min;
}

void fib_heap_extract_min(FibHeap *heap)
{
    FibNode *node;

    if (heap==NULL || heap->min==NULL)
        return ;

    node = _fib_heap_extract_min(heap);
    if (node!=NULL)
        free(node);
}

/*
 * 在斐波那契堆heap中是否存在键值为key的节点；存在返回1，否则返回0。
 */
int fib_heap_get_min(FibHeap *heap, Type *pkey)
{
    if (heap==NULL || heap->min==NULL || pkey==NULL)
        return 0;

    *pkey = heap->min->key;
    return 1;
}

/*
 * 修改度数
 */
static void renew_degree(FibNode *parent, int degree)
{
    printf("Before: Parent degree = %d, decrease by = %d\n", parent->degree, degree);
    parent->degree -= degree;
    printf("After: Parent degree = %d\n", parent->degree);
    if(parent->degree < 0)
        printf("wrong\n");
    if (parent-> parent != NULL)
        renew_degree(parent->parent, degree);
}

/*
 * 将node从父节点parent的子链接中剥离出来，
 * 并使node成为"堆的根链表"中的一员。
 */
static void fib_heap_cut(FibHeap *heap, FibNode *node, FibNode *parent)
{
    
    fib_node_remove(node);
    // printf("Cutting node with degree = %d from parent with degree = %d\n", node->degree, parent->degree);
    // renew_degree(parent, node->degree);
    parent->degree--;
    // node没有兄弟
    if (node == node->right)
        parent->child = NULL;
    else
        parent->child = node->right;
    node->parent = NULL;
    node->left = node->right = node;
    node->marked = 0;
    // 将"node所在树"添加到"根链表"中
    fib_node_add(node, heap->min);
    
}

/*
 * 对节点node进行"级联剪切"
 *
 * 级联剪切：如果减小后的结点破坏了最小堆性质，
 *     则把它切下来(即从所在双向链表中删除，并将
 *     其插入到由最小树根节点形成的双向链表中)，
 *     然后再从"被切节点的父节点"到所在树根节点递归执行级联剪枝
 */
static void fib_heap_cascading_cut(FibHeap *heap, FibNode *node)
{
    FibNode *parent = node->parent;
    if (parent == NULL)
        return ;

    if (node->marked == 0)
        node->marked = 1;
    else
    {
        fib_heap_cut(heap, node, parent);
        fib_heap_cascading_cut(heap, parent);
    }
}

/*
 * 将斐波那契堆heap中节点node的值减少为key
 */
static void fib_heap_decrease(FibHeap *heap, FibNode *node, Type key)
{
    FibNode *parent;

    if (heap==NULL || heap->min==NULL ||node==NULL)
        return ;

    if (key>=node->key)
    {
        printf("decrease failed: the new key(%d) is no smaller than current key(%d)\n", key, node->key);
        return ;
    }

    node->key = key;
    parent = node->parent;
    if (parent!=NULL && node->key < parent->key)
    {   
        // 将node从父节点parent中剥离出来，并将node添加到根链表中
        fib_heap_cut(heap, node, parent);
        fib_heap_cascading_cut(heap, parent);
    }
    // 更新最小节点
    if (node->key < heap->min->key)
        heap->min = node;
}

/*
 * 将斐波那契堆heap中节点node的值增加为key
 */
static void fib_heap_increase(FibHeap *heap, FibNode *node, Type key)
{
    FibNode *child, *parent, *right;

    if (heap==NULL || heap->min==NULL ||node==NULL)
        return ;

    if (key < node->key)
    {
        printf("increase failed: the new key(%d) is no greater than current key(%d)\n", key, node->key);
        return ;
    }

    // 将node每一个儿子(不包括孙子,重孙,...)都添加到"斐波那契堆的根链表"中
    while (node->child != NULL)
    {
        child = node->child;
        fib_node_remove(child);               // 将child从node的子链表中删除
        if (child->right == child)
            node->child = NULL;
        else
            node->child = child->right;

        fib_node_add(child, heap->min);       // 将child添加到根链表中
        child->parent = NULL;
    }
    node->degree = 0;
    node->key = key;

    // 如果node不在根链表中，
    //     则将node从父节点parent的子链接中剥离出来，
    //     并使node成为"堆的根链表"中的一员，
    //     然后进行"级联剪切"
    // 否则，则判断是否需要更新堆的最小节点
    parent = node->parent;
    if(parent != NULL)
    {
        fib_heap_cut(heap, node, parent);
        fib_heap_cascading_cut(heap, parent);
    }
    else if(heap->min == node)
    {
        right = node->right;
        while(right != node)
        {
            if(node->key > right->key)
                heap->min = right;
            right = right->right;
        }
    }
}

/*
 * 更新二项堆heap的节点node的键值为key
 */
void _fib_heap_update(FibHeap *heap, FibNode *node, Type key)
{
    if(key < node->key)
        fib_heap_decrease(heap, node, key);
    else if(key > node->key)
        fib_heap_increase(heap, node, key);
    else
        printf("No need to update!!!\n");
}

void fib_heap_update(FibHeap *heap, Type oldkey, Type newkey)
{
    FibNode *node;

    if (heap==NULL)
        return ;

    node = fib_heap_search(heap, oldkey);
    if (node!=NULL)
        _fib_heap_update(heap, node, newkey);
}

/*
 * 在最小堆root中查找键值为key的节点
 */
static FibNode* fib_node_search(FibNode *root, Type key)
{
    FibNode *t = root;    // 临时节点
    FibNode *p = NULL;    // 要查找的节点

    if (root==NULL)
        return root;

    do
    {
        if (t->key == key)
        {
            p = t;
            break;
        }
        else
        {
            if ((p = fib_node_search(t->child, key)) != NULL)
                break;
        }
        t = t->right;
    } while (t != root);

    return p;
}

/*
 * 在斐波那契堆heap中查找键值为key的节点
 */
static FibNode *fib_heap_search(FibHeap *heap, Type key)
{
    if (heap==NULL || heap->min==NULL)
        return NULL;

    return fib_node_search(heap->min, key);
}

/*
 * 在斐波那契堆heap中是否存在键值为key的节点。
 * 存在返回1，否则返回0。
 */
int fib_heap_contains(FibHeap *heap, Type key)
{
    return fib_heap_search(heap,key)!=NULL ? 1: 0;
}

/*
 * 删除结点node
 */
static void _fib_heap_delete(FibHeap *heap, FibNode *node)
{
    Type min = heap->min->key;
    fib_heap_decrease(heap, node, min-1);
    _fib_heap_extract_min(heap);
    free(node);
}

void fib_heap_delete(FibHeap *heap, Type key)
{
    FibNode *node;

    if (heap==NULL || heap->min==NULL)
        return ;

    node = fib_heap_search(heap, key);
    if (node==NULL)
        return ;

    _fib_heap_delete(heap, node);
}

/*
 * 销毁斐波那契堆
 */
static void fib_node_destroy(FibNode *node)
{
    FibNode *start = node;

    if(node == NULL)
        return;

    do {
        fib_node_destroy(node->child);
        // 销毁node，并将node指向下一个
        node = node->right;
        free(node->left);
    } while(node != start);
}

void fib_heap_destroy(FibHeap *heap)
{
    fib_node_destroy(heap->min);
    free(heap->cons);
    free(heap);
}

/*
 * 打印"斐波那契堆"
 *
 * 参数说明：
 *     node       -- 当前节点
 *     prev       -- 当前节点的前一个节点(父节点or兄弟节点)
 *     direction  --  1，表示当前节点是一个左孩子;
 *                    2，表示当前节点是一个兄弟节点。
 */
static void _fib_print(FibNode *node, FibNode *prev, int direction)
{
    FibonacciNode *start=node;

    if (node==NULL)
        return ;
    do
    {
        if (direction == 1)
            printf("%8d(%d) is %2d's child\n", node->key, node->degree, prev->key);
        else
            printf("%8d(%d) is %2d's next\n", node->key, node->degree, prev->key);

        if (node->child != NULL)
            _fib_print(node->child, node, 1);

        // 兄弟节点
        prev = node;
        node = node->right;
        direction = 2;
    } while(node != start);
}

void fib_print(FibHeap *heap)
{
    int i=0;
    FibonacciNode *p;

    if (heap==NULL || heap->min==NULL)
        return ;

    printf("== 斐波那契堆的详细信息: ==\n");
    p = heap->min;
    do {
        i++;
        printf("%2d. %4d(%d) is root\n", i, p->key, p->degree);

        _fib_print(p->child, p, 1);
        p = p->right;
    } while (p != heap->min);
    printf("\n");
}

void startRecordTime()
{
    gettimeofday(&g_TimeRecord.start, NULL);
}

int32_t getDurationMicroseconds()
{
    struct timeval end;
    gettimeofday(&end, NULL);
    return (end.tv_sec - g_TimeRecord.start.tv_sec) * 1000000 + end.tv_usec - g_TimeRecord.start.tv_usec;
}

// 计算总代价：寻址时长 + 带体磨损 + 电机磨损
double getTotalCost(const InputParam *input, OutputParam *output) {
    AccessTime accessTime = {0};
    TotalAccessTime(input, output, &accessTime);                    // 寻址时长
    uint32_t a_bw = TotalTapeBeltWearTimes(input, output, NULL);    // 带体磨损
    uint32_t a_mw = TotalMotorWearTimes(input, output);             // 电机磨损
    // 总代价
    double totalCost = cost_weight->alpha * accessTime.addressDuration + cost_weight->beta * a_bw + cost_weight->gamma * a_mw;
    return totalCost;
}

// 计算单次代价：寻址时长 + 带体磨损 + 电机磨损
double getCost(const HeadInfo *start, const HeadInfo *target) {
    uint32_t a_st = SeekTimeCalculate(start, target);
    uint32_t a_bw = BeltWearTimes(start, target, NULL);
    uint32_t a_mw = MotorWearTimes(start, target);
    double cost = cost_weight->alpha * a_st + cost_weight->beta * a_bw + cost_weight->gamma * a_mw;
    return cost;
}

void CheckBetterResults(const InputParam *input, OutputParam *output, int *best_sequence, uint32_t * min_cost_p, int * flag_p, int flag_v){
    pthread_mutex_lock(&watchDog_g.check_mutex); // 保护现场
    double total_cost = getTotalCost(input, output);
    printf("check if find better results!\n");
    if (total_cost < *min_cost_p) {
        printf("find better results!\n");
        *min_cost_p = total_cost;
        *flag_p = flag_v;
        memcpy(best_sequence, output->sequence, input->ioVec.len * sizeof(int));
    }
    pthread_mutex_unlock(&watchDog_g.check_mutex);
}

/**
 * @brief  算法接口
 * @param  input            输入参数
 * @param  output           输出参数
 * @return int32_t          返回成功或者失败，RETURN_OK 或 RETURN_ERROR
 */
int32_t IOScheduleAlgorithm(const InputParam *input, OutputParam *output) {
    // 计算代价权重
    initCostWeight(input, output);

    double min_cost = DBL_MAX;
    double total_cost = DBL_MAX;
    output->len = input->ioVec.len;
    OutputParam tmp_output;
    tmp_output.len = output->len;
    tmp_output.sequence = (int *)malloc(input->ioVec.len * sizeof(int));
    if (input->ioVec.len > 1000) {
        int flag = 1;
        partition_scan(input, &tmp_output); // 在算法内部还是只考虑寻址时长
        CheckBetterResults(input, &tmp_output, output->sequence, &min_cost, &flag, 0);

        // cycleMerge(input, &tmp_output);
        // CheckBetterResults(input, &tmp_output, output->sequence, &min_cost, &flag, 1);

        fastCycleMerge(input, &tmp_output);
        CheckBetterResults(input, &tmp_output, output->sequence, &min_cost, &flag, 2);

        // merge(input, &tmp_output);
        // CheckBetterResults(input, &tmp_output, output->sequence, &min_cost, &flag, 3);

        // merge_random(input, &tmp_output);
        // CheckBetterResults(input, &tmp_output, output->sequence, &min_cost, &flag, 4);
        printf("flag=%d\n", flag);
    }
    else {
        int flag = 3;
        partition_scan(input, &tmp_output); // 在算法内部还是只考虑寻址时长
        CheckBetterResults(input, &tmp_output, output->sequence, &min_cost, &flag, 5);
        
        MPScan(input, &tmp_output); // 在算法内部还是只考虑寻址时长
        CheckBetterResults(input, &tmp_output, output->sequence, &min_cost, &flag, 6);

        cycleMerge(input, &tmp_output);
        CheckBetterResults(input, &tmp_output, output->sequence, &min_cost, &flag, 7);

        merge(input, &tmp_output);
        CheckBetterResults(input, &tmp_output, output->sequence, &min_cost, &flag, 8);

        merge_random(input, &tmp_output);
        CheckBetterResults(input, &tmp_output, output->sequence, &min_cost, &flag, 9);

        NearestNeighborAlgorithm(input, &tmp_output);
        CheckBetterResults(input, &tmp_output, output->sequence, &min_cost, &flag, 10);

        printf("flag=%d\n", flag);
    }

    free(tmp_output.sequence);
    return 0;
}

void* _AlgorithmRunThread(void *arg)
{
    // signal(SIGUSR1, handle_signal);

    if (setjmp(jumpBuffer) == 0) {
        // 在函数开始时设置跳转点
        printf("Before longjmp\n");
        signal(SIGUSR1, handle_signal); // 注册信号处理函数
    } else {
        // 在触发信号处理后跳转到这里
        printf("After longjmp\n");
        signal(SIGUSR1, SIG_DFL); // 恢复默认的信号处理函数
        WatchDogWorkerFinish();
        return; // 直接结束函数运行，TODO: 能不能想个法子做一些内存管理
    }

    ThreadArg *thread_arg = (ThreadArg*)arg;
    int32_t ret, duration_us;

    startRecordTime();

    ret = IOScheduleAlgorithm(thread_arg->input, thread_arg->output);
    printf("cost before operator optimization: %ld\n", getTotalCost(thread_arg->input, thread_arg->output));
    // ret = operator_optimization(input, output);
    printf("cost afther operator optimization_op: %ld\n", getTotalCost(thread_arg->input, thread_arg->output));
    duration_us = getDurationMicroseconds();
    // printf("duration_us=%d\n", duration_us);
    
    WatchDogWorkerFinish();
    return NULL;
}


/**
 * @brief  算法运行的主入口
 * @param  input            输入参数
 * @param  output           输出参数
 * @return uint32_t          返回成功或者失败，RETURN_OK 或 RETURN_ERROR
 */
int32_t AlgorithmRun(const InputParam *input, OutputParam *output) {
    int32_t ret;
    pthread_t thread_id;
    RunThreadArg thread_arg;
    int threshold = 17*1000000, granularity = 1000000; // 17s, 1s

    startRecordTime();
    WatchDogInit();
    g_input = input;
    thread_arg.input = input;
    thread_arg.output = output;

    pthread_create(&thread_id, NULL, _AlgorithmRunThread, &thread_arg);
    pthread_join(thread_id, NULL);
    // WatchDogFunc(&thread_id, threshold, granularity);
    // sleep(1);
    return RETURN_OK;
}

int32_t operator_optimization(const InputParam *input, OutputParam *output)
{
    // SimpleOperatorOptimization(input, output);

    ArtificialBeeColony(input, output);
    return RETURN_OK;
}

int32_t SimpleOperatorOptimization(const InputParam *input, OutputParam *output)
{
    int32_t ret;

    int maxIterations = 1000000;

    uint32_t currentCost = getTotalCost(input, output); // 总代价
    uint32_t bestCost = currentCost;
    uint32_t bestSequence[input->ioVec.len];

    // 当前求到的初始解
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
        bestSequence[i] = output->sequence[i];

    // 用于存放邻域解
    OutputParam *temp_output;
    temp_output = (OutputParam *)malloc(sizeof(OutputParam));
    temp_output->len = input->ioVec.len;
    temp_output->sequence = (uint32_t *)malloc(input->ioVec.len * sizeof(uint32_t));

    uint32_t iteration = 0;
    uint32_t flag = 50000;

    while (iteration < flag)
    {
        // 生成邻域解
        for (uint32_t i = 0; i < input->ioVec.len; ++i)
            temp_output->sequence[i] = output->sequence[i];
        {
            // 将一个点插入另一个位置
            uint32_t from = rand() % input->ioVec.len;
            uint32_t to;
            to = rand() % input->ioVec.len;
            uint32_t temp = temp_output->sequence[from];
            if (from < to)
                for (uint32_t i = from; i < to; ++i)
                    temp_output->sequence[i] = temp_output->sequence[i + 1];
            else
                for (uint32_t i = from; i > to; --i)
                    temp_output->sequence[i] = temp_output->sequence[i - 1];
            temp_output->sequence[to] = temp;
        }

        // 计算邻域解的总时延
        uint32_t neighborCost = getTotalCost(input, temp_output);
        // 接受邻域解的条件，比当前解好或温度条件满足接受一个更劣的解
        if (neighborCost < currentCost)
        {
            printf("%ld iter, one operator optimization cost: %ld\n", iteration, neighborCost);
            // 接受邻域解
            for (uint32_t i = 0; i < input->ioVec.len; ++i)
                output->sequence[i] = temp_output->sequence[i];

            currentCost = neighborCost;

            // 若当前解比最优解好，则更新最优解
            if (currentCost < bestCost)
            {
                for (uint32_t i = 0; i < input->ioVec.len; ++i)
                    bestSequence[i] = output->sequence[i];
                bestCost = currentCost;
            }
        }

        ++iteration;
    }

    // 将最优解复制到输出参数
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++)
        output->sequence[i] = bestSequence[i];

    printf("accessTime after search:%d\n", bestCost);

    free(temp_output->sequence);
    free(temp_output);

    return RETURN_OK;
    return RETURN_OK;
}

int32_t SimulatedAnnealingRandom(const InputParam *input, OutputParam *output)
{
    int32_t ret;

    // 初始化参数
    double initialTemperature = 5000;
    double coolingRate = 0.995;
    double minTemperature = 1;
    int maxIterations = 10000000;

    uint32_t currentCost = getTotalCost(input, output); // 总代价
    uint32_t bestCost = currentCost;
    uint32_t bestSequence[input->ioVec.len];

    // 当前求到的初始解
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
        bestSequence[i] = output->sequence[i];

    // 用于存放邻域解
    OutputParam *temp_output;
    temp_output = (OutputParam *)malloc(sizeof(OutputParam));
    temp_output->len = input->ioVec.len;
    temp_output->sequence = (uint32_t *)malloc(input->ioVec.len * sizeof(uint32_t));

    // 模拟退火过程
    double temperature = initialTemperature;
    uint32_t iteration = 0;
    uint32_t flag = 100000;

    while (temperature > minTemperature && iteration < maxIterations)
    {
        // 生成邻域解
        for (uint32_t i = 0; i < input->ioVec.len; ++i)
            temp_output->sequence[i] = output->sequence[i];

        // double randValue = (double)rand() / RAND_MAX;
        // if (randValue < 0.33)
        // {
        //     // 交换两个点
        //     uint32_t i = rand() % input->ioVec.len;
        //     uint32_t j = rand() % input->ioVec.len;
        //     uint32_t temp = temp_output->sequence[i];
        //     temp_output->sequence[i] = temp_output->sequence[j];
        //     temp_output->sequence[j] = temp;
        // }
        // else if (randValue < 0.66)
        // {
        //     // 反转一段区间
        //     uint32_t start = rand() % input->ioVec.len;
        //     uint32_t end = rand() % input->ioVec.len;
        //     if (start > end)
        //     {
        //         uint32_t temp = start;
        //         start = end;
        //         end = temp;
        //     }
        //     while (start < end)
        //     {
        //         uint32_t temp = temp_output->sequence[start];
        //         temp_output->sequence[start] = temp_output->sequence[end];
        //         temp_output->sequence[end] = temp;
        //         start++;
        //         end--;
        //     }
        // }
        // else
        {
            // 将一个点插入另一个位置
            uint32_t from = rand() % input->ioVec.len;
            uint32_t to;
            to = rand() % input->ioVec.len;
            uint32_t temp = temp_output->sequence[from];
            if (from < to)
                for (uint32_t i = from; i < to; ++i)
                    temp_output->sequence[i] = temp_output->sequence[i + 1];
            else
                for (uint32_t i = from; i > to; --i)
                    temp_output->sequence[i] = temp_output->sequence[i - 1];
            temp_output->sequence[to] = temp;
        }

        // 计算邻域解的总时延
        uint32_t neighborCost = getTotalCost(input, temp_output);
        // 接受邻域解的条件，比当前解好或温度条件满足接受一个更劣的解
        if (neighborCost < currentCost || (iteration >= flag && ((double)rand() / RAND_MAX) < exp((currentCost - neighborCost) / temperature)))
        {
            // printf("%ld iter, one operator optimization cost: %ld\n", iteration, neighborCost);
            // 接受邻域解
            for (uint32_t i = 0; i < input->ioVec.len; ++i)
                output->sequence[i] = temp_output->sequence[i];

            currentCost = neighborCost;

            // 若当前解比最优解好，则更新最优解
            if (currentCost < bestCost)
            {
                if (currentCost * 1.0001 < bestCost)
                    flag = 0;
                for (uint32_t i = 0; i < input->ioVec.len; ++i)
                    bestSequence[i] = output->sequence[i];
                bestCost = currentCost;
            }
        }
        if (iteration >= flag)
        {
            // 降低温度
            temperature *= coolingRate;
        }
        ++iteration;
    }

    // 将最优解复制到输出参数
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++)
        output->sequence[i] = bestSequence[i];

    printf("accessTime after search:%d\n", bestCost);

    free(temp_output->sequence);
    free(temp_output);

    return RETURN_OK;
}

int32_t SimulatedAnnealing(const InputParam *input, OutputParam *output)
{
    int32_t ret;

    // 初始化参数
    double initialTemperature = 5000;
    double coolingRate = 0.995;
    double minTemperature = 1;
    int maxIterations = 10000000;

    uint32_t currentCost = getTotalCost(input, output); // 总代价
    uint32_t bestCost = currentCost;
    uint32_t bestSequence[input->ioVec.len];

    // 当前求到的初始解
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
        bestSequence[i] = output->sequence[i];

    // 用于存放邻域解
    OutputParam *temp_output;
    temp_output = (OutputParam *)malloc(sizeof(OutputParam));
    temp_output->len = input->ioVec.len;
    temp_output->sequence = (uint32_t *)malloc(input->ioVec.len * sizeof(uint32_t));

    // 模拟退火过程
    double temperature = initialTemperature;
    uint32_t iteration = 0;

    while (temperature > minTemperature && iteration < maxIterations)
    {
        // 生成邻域解
        for (uint32_t i = 0; i < input->ioVec.len; ++i)
            temp_output->sequence[i] = output->sequence[i];

        double randValue = (double)rand() / RAND_MAX;
        if (randValue < 0.33)
        {
            // 交换两个点
            uint32_t i = rand() % input->ioVec.len;
            uint32_t j = rand() % input->ioVec.len;
            uint32_t temp = temp_output->sequence[i];
            temp_output->sequence[i] = temp_output->sequence[j];
            temp_output->sequence[j] = temp;
        }
        else if (randValue < 0.66)
        {
            // 反转一段区间
            uint32_t start = rand() % input->ioVec.len;
            uint32_t end = rand() % input->ioVec.len;
            if (start > end)
            {
                uint32_t temp = start;
                start = end;
                end = temp;
            }
            while (start < end)
            {
                uint32_t temp = temp_output->sequence[start];
                temp_output->sequence[start] = temp_output->sequence[end];
                temp_output->sequence[end] = temp;
                start++;
                end--;
            }
        }
        else
        {
            // 将一个点插入另一个位置
            uint32_t from = rand() % input->ioVec.len;
            uint32_t to;
            to = rand() % input->ioVec.len;
            uint32_t temp = temp_output->sequence[from];
            if (from < to)
                for (uint32_t i = from; i < to; ++i)
                    temp_output->sequence[i] = temp_output->sequence[i + 1];
            else
                for (uint32_t i = from; i > to; --i)
                    temp_output->sequence[i] = temp_output->sequence[i - 1];
            temp_output->sequence[to] = temp;
        }

        // 计算邻域解的总时延
        uint32_t neighborCost = getTotalCost(input, temp_output);
        // 接受邻域解的条件，比当前解好或温度条件满足接受一个更劣的解
        double deltaCost = (double)(currentCost - neighborCost);
        if (neighborCost < currentCost || ((double)rand() / RAND_MAX) < exp(deltaCost / temperature))
        {
            // printf("%ld iter, one operator optimization cost: %ld\n", iteration, neighborCost);
            // 接受邻域解
            for (uint32_t i = 0; i < input->ioVec.len; ++i)
                output->sequence[i] = temp_output->sequence[i];

            currentCost = neighborCost;

            // 若当前解比最优解好，则更新最优解
            if (currentCost < bestCost)
            {
                for (uint32_t i = 0; i < input->ioVec.len; ++i)
                    bestSequence[i] = output->sequence[i];
                bestCost = currentCost;
            }
        }

        // 降低温度
        temperature *= coolingRate;
        ++iteration;
    }

    // 将最优解复制到输出参数
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++)
        output->sequence[i] = bestSequence[i];

    printf("accessTime after search:%d\n", bestCost);

    free(temp_output->sequence);
    free(temp_output);

    return RETURN_OK;
}

// 模拟蜂群
//  复制解
void copySolution(uint32_t *dest, const uint32_t *src, uint32_t len)
{
    for (uint32_t i = 0; i < len; ++i)
    {
        dest[i] = src[i];
    }
}

// 生成随机解
void generateRandomSolution(OutputParam *solution, uint32_t len)
{
    uint32_t i = rand() % len;
    uint32_t j = rand() % len;
    double randValue = (double)rand() / RAND_MAX;
    if (randValue < 0.33)
    {
        // 随机交换两个随机位置的元素
        uint32_t temp = solution->sequence[i];
        solution->sequence[i] = solution->sequence[j];
        solution->sequence[j] = temp;
    }
    else if (randValue < 0.66)
    {
        if (i > j)
        {
            uint32_t temp = i;
            i = j;
            j = temp;
        }
        while (i < j)
        {
            uint32_t temp = solution->sequence[i];
            solution->sequence[i] = solution->sequence[j];
            solution->sequence[j] = temp;
            i++;
            j--;
        }
    }
    else
    {
        uint32_t temp = solution->sequence[i];
        if (i < j)
            for (uint32_t k = i; k < j; ++k)
                solution->sequence[k] = solution->sequence[k + 1];
        else
            for (uint32_t k = i; k > j; --k)
                solution->sequence[k] = solution->sequence[k - 1];
        solution->sequence[j] = temp;
    }
}

// 生成邻域解（随机插入元素）
void generateNeighborSolution(OutputParam *neighbor, const OutputParam *current, uint32_t len)
{
    copySolution(neighbor->sequence, current->sequence, len);
    uint32_t i = rand() % len;
    uint32_t j = rand() % len;
    uint32_t temp = neighbor->sequence[i];
    if (i < j)
        for (uint32_t k = i; k < j; ++k)
            neighbor->sequence[k] = neighbor->sequence[k + 1];
    else
        for (uint32_t k = i; k > j; --k)
            neighbor->sequence[k] = neighbor->sequence[k - 1];
    neighbor->sequence[j] = temp;
}

// 人工蜂群算法
int32_t ArtificialBeeColony(const InputParam *input, OutputParam *output)
{
    int32_t ret;

    uint32_t len = input->ioVec.len;

    // 初始化蜜蜂群
    OutputParam employedBees[NUM_EMPLOYED_BEES];
    OutputParam bestSolution;
    bestSolution.len = len;
    bestSolution.sequence = (uint32_t *)malloc(len * sizeof(uint32_t));

    uint32_t employedBeesCost[NUM_EMPLOYED_BEES]; // 雇佣蜂对应的解的代价
    uint32_t bestCost = UINT32_MAX;               // 最好的解的代价
    int trials[NUM_EMPLOYED_BEES] = {0};          // 每个雇佣蜂的尝试次数

    // 初始化雇佣蜂
    for (int i = 0; i < NUM_EMPLOYED_BEES; ++i)
    {
        employedBees[i].len = len;
        employedBees[i].sequence = (uint32_t *)malloc(len * sizeof(uint32_t));
        for (uint32_t j = 0; j < input->ioVec.len; ++j)
            employedBees[i].sequence[j] = output->sequence[j];
        if (i != 0) // 第一只不随机
            generateRandomSolution(&employedBees[i], len);
        employedBeesCost[i] = getTotalCost(input, &employedBees[i]);
        printf("%ld emplyed bee initial cost: %ld\n", i, employedBeesCost[i]);
        // 更新最优解
        if (employedBeesCost[i] < bestCost)
        {
            bestCost = employedBeesCost[i];
            copySolution(bestSolution.sequence, employedBees[i].sequence, len);
        }
    }

    // 临时变量用于存储邻域解
    OutputParam neighbor;
    neighbor.len = len;
    neighbor.sequence = (uint32_t *)malloc(len * sizeof(uint32_t));

    int iteration = 0;
    while (iteration < MAX_ITERATIONS)
    {
        // 雇佣蜂阶段
        for (int i = 0; i < NUM_EMPLOYED_BEES; ++i)
        {
            generateNeighborSolution(&neighbor, &employedBees[i], len);
            uint32_t neighborCost = getTotalCost(input, &neighbor);

            // 如果邻域解更好，接受邻域解
            if (neighborCost < employedBeesCost[i])
            {
                employedBeesCost[i] = neighborCost;
                copySolution(employedBees[i].sequence, neighbor.sequence, len);
                trials[i] = 0; // 重置尝试次数
            }
            else
            {
                trials[i]++; // 尝试次数增加
            }

            // 更新最优解
            if (employedBeesCost[i] < bestCost)
            {
                bestCost = employedBeesCost[i];
                copySolution(bestSolution.sequence, employedBees[i].sequence, len);
                printf("%ld iter, one operator optimization cost: %ld\n", iteration, bestCost);
            }
        }

        // 计算选择概率（根据解的质量）
        double totalFitness = 0.0;
        double fitness[NUM_EMPLOYED_BEES];
        for (int i = 0; i < NUM_EMPLOYED_BEES; ++i)
        {
            fitness[i] = 1.0 / (1.0 + employedBeesCost[i]); // 代价越小，适应度越高
            totalFitness += fitness[i];
        }

        // 跟随蜂阶段
        for (int i = 0; i < NUM_ONLOOKER_BEES; ++i)
        {
            // 根据适应度选择解
            double r = (double)rand() / RAND_MAX * totalFitness;
            double cumulativeFitness = 0.0;
            int selectedBeeIndex = 0;
            for (int j = 0; j < NUM_EMPLOYED_BEES; ++j)
            {
                cumulativeFitness += fitness[j];
                if (cumulativeFitness >= r)
                {
                    selectedBeeIndex = j;
                    break;
                }
            }

            // 跟随蜂搜索邻域解
            generateNeighborSolution(&neighbor, &employedBees[selectedBeeIndex], len);
            uint32_t neighborCost = getTotalCost(input, &neighbor);

            // 如果邻域解更好，接受邻域解
            if (neighborCost < employedBeesCost[selectedBeeIndex])
            {
                employedBeesCost[selectedBeeIndex] = neighborCost;
                copySolution(employedBees[selectedBeeIndex].sequence, neighbor.sequence, len);
                trials[selectedBeeIndex] = 0; // 重置尝试次数
            }
            else
            {
                trials[selectedBeeIndex]++; // 尝试次数增加
            }

            // 更新最优解
            if (employedBeesCost[selectedBeeIndex] < bestCost)
            {
                bestCost = employedBeesCost[selectedBeeIndex];
                copySolution(bestSolution.sequence, employedBees[selectedBeeIndex].sequence, len);
                printf("%ld iter, one operator optimization cost: %ld\n", iteration, bestCost);
            }
        }

        // 侦查蜂阶段：检查是否有过多尝试次数的解，如果有则重新生成随机解
        for (int i = 0; i < NUM_EMPLOYED_BEES; ++i)
        {
            if (trials[i] > MAX_TRIALS)
            {
                generateRandomSolution(&employedBees[i], len);
                employedBeesCost[i] = getTotalCost(input, &employedBees[i]);
                trials[i] = 0; // 重置尝试次数
            }
        }

        iteration++;
    }

    // 将最优解复制到输出参数
    output->len = bestSolution.len;
    copySolution(output->sequence, bestSolution.sequence, bestSolution.len);

    printf("Final best cost: %u\n", bestCost);

    // 释放内存
    for (int i = 0; i < NUM_EMPLOYED_BEES; ++i)
    {
        free(employedBees[i].sequence);
    }
    free(neighbor.sequence);
    free(bestSolution.sequence);

    return 0;
}

// 快速排序
void QuickSort(IOUint *a, int len)
{
    int low = 0;
    int high = len - 1;
    int stack[high - low + 1];
    int top = -1;
    stack[++top] = low;
    stack[++top] = high;

    while (top >= 0)
    {
        // 从栈中弹出 high 和 low 值，表示当前需要排序的子数组的边界
        high = stack[top--];
        low = stack[top--];

        // 选择子数组的最后一个元素作为枢轴（pivot），并初始化变量 i 为 low - 1
        uint32_t pivot = a[high].startLpos;
        int i = low - 1;

        // 遍历当前子数组
        for (int j = low; j < high; ++j)
        {
            // 将所有小于枢轴的元素移到枢轴的左边
            if (a[j].startLpos < pivot)
            {
                ++i; // i 指向当前小于枢轴的元素的位置
                IOUint temp = a[i];
                a[i] = a[j];
                a[j] = temp;
            }
        }

        // 将枢轴元素放到正确的位置
        IOUint temp = a[i + 1];
        a[i + 1] = a[high];
        a[high] = temp;
        int pi = i + 1;

        // 根据枢轴的位置 pi，将左子数组和右子数组的边界压入栈中
        if (pi - 1 > low)
        {
            stack[++top] = low;
            stack[++top] = pi - 1;
        }
        if (pi + 1 < high)
        {
            stack[++top] = pi + 1;
            stack[++top] = high;
        }
    }
}

int32_t partition_scan(const InputParam *input, OutputParam *output)
{
    // 复制 IO 请求数组并按 lpos 排序
    IOUint *sortedIOs = (IOUint *)malloc(input->ioVec.len * sizeof(IOUint));
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
        sortedIOs[i] = input->ioVec.ioArray[i];
    QuickSort(sortedIOs, input->ioVec.len);

    for (int i = 0; i < input->ioVec.len; i++)
    {
        if (i + 1 == input->ioVec.len)
            break;
        if (sortedIOs[i + 1].startLpos < sortedIOs[i].startLpos)
        {
            printf("sort error!\n");
            abort();
        }
    }

    //----排序结束----

    //----搜索最佳分割参数----
    int min_time = 0x3f3f3f3f, best_partition_size = 5000;
    int best_scan_method = 1;
    int *best_sequence = (int *)malloc(input->ioVec.len * sizeof(int));
    bool *vis = (bool *)malloc((input->ioVec.len + 1) * sizeof(bool));
    int partition_io_num[1000] = {0};
    int partition_io_start[1000] = {0};

    // 按固定长度进行分区，对长度参数进行搜索
    for (int i = 5000; i <= 740000; i += 5000)
    {
        int partition_start_now = 0;
        int now = 0;
        int partition_threshold = i;
        memset(partition_io_start, 0, sizeof(partition_io_start));
        memset(partition_io_num, 0, sizeof(partition_io_num));
        // 遍历所有请求，统计每个分区的起始请求和请求数量
        for (int j = 0; j < input->ioVec.len; j++)
        {
            if (sortedIOs[j].startLpos >= partition_start_now + partition_threshold)
            {
                while (sortedIOs[j].startLpos >= partition_start_now + partition_threshold)
                {
                    partition_start_now += partition_threshold;
                    now++;
                }
                partition_io_start[now] = j;
            }
            partition_io_num[now]++;
        }
        // 分区处理方法，SORT、SCAN、MPScan、MPScan*
        int scan_method[] = {0, 1, 2, 3};
        // int scan_method[] = {0, 1, 2};
        // int scan_method[] = {3};
        int method_num = sizeof(scan_method) / sizeof(scan_method[0]);
        // DEBUG("method_num=%d\n", method_num);
        for (int method_idx = 0; method_idx < method_num; method_idx++)
        {
            // DEBUG("partition_len=%d, method=%d\n", i, scan_method[method_idx]);
            // 重置 vis 数组
            for (int j = 0; j < input->ioVec.len + 1; j++)
                vis[j] = 0;
            HeadInfo head = input->headInfo;
            head.status = HEAD_RW;
            // 对每个分区进行处理
            for (int j = 0; j <= now; j++)
            {
                // 跳过空分区
                if (partition_io_num[j] == 0)
                    continue;
                if (scan_method[method_idx] < 3)
                    Sort_Scan_MPScan_Perpartition(output, sortedIOs, vis, &head, partition_io_start[j], partition_io_num[j], scan_method[method_idx]);
                else if (scan_method[method_idx] == 3)
                {
                    MPScan_star_PerPartition(input, output, sortedIOs, vis, &head, partition_io_start[j], partition_io_start[j] + partition_io_num[j]);
                }
                head.wrap = sortedIOs[partition_io_start[j] + partition_io_num[j] - 1].wrap;
                head.lpos = sortedIOs[partition_io_start[j] + partition_io_num[j] - 1].endLpos;
            }
            AccessTime accessTime = {0};
            // 计算当前分区方案下的总寻址时间
            TotalAccessTime(input, output, &accessTime);
            int time = accessTime.addressDuration;
            // 记录最小寻址时间的分区方案
            if (time <= min_time)
            {
                best_scan_method = scan_method[method_idx];
                best_partition_size = i;
                min_time = time;
                for (int j = 0; j < input->ioVec.len; j++)
                    best_sequence[j] = output->sequence[j];
            }
        }
    }
    // printf("best_scan_method=%d best_partition_size=%d\n", best_scan_method, best_partition_size);
    for (int i = 0; i < input->ioVec.len; i++)
    {
        output->sequence[i] = best_sequence[i];
    }
    free(best_sequence);
    free(sortedIOs);
}

int32_t Sort_Scan_MPScan_Perpartition(OutputParam *output, IOUint *sortedIOs, bool *vis, HeadInfo *head, int partition_start, int partition_len, const int scan_method)
{
    uint32_t index = partition_start;
    HeadInfo currentHead = *head;
    int direction = 1; // 扫描方向：1 表示从 BOT 向 EOT 扫描，-1 表示从 EOT 向 BOT 扫描
    while (index < partition_start + partition_len)
    {
        if (scan_method == 0)
        { // SORT
            output->sequence[index] = sortedIOs[index].id;
            index++;
            continue;
        }
        if (direction == 1)
        {
            // 从 BOT 向 EOT 扫描
            for (uint32_t i = partition_start; i < partition_start + partition_len; ++i)
            {
                if (sortedIOs[i].wrap & 1 || vis[sortedIOs[i].id])
                {
                    continue;
                }
                if (scan_method == 1)
                { // SCAN1，两趟扫描
                    if (sortedIOs[i].startLpos >= currentHead.lpos)
                    {
                        output->sequence[index++] = sortedIOs[i].id;
                        vis[sortedIOs[i].id] = 1;
                        currentHead.wrap = sortedIOs[i].wrap;
                        currentHead.lpos = sortedIOs[i].startLpos;
                    }
                }
                else if (scan_method == 2)
                { // SCAN2，多趟扫描
                    if (sortedIOs[i].startLpos > currentHead.lpos || currentHead.lpos == 0)
                    {
                        output->sequence[index++] = sortedIOs[i].id;
                        vis[sortedIOs[i].id] = 1;
                        currentHead.wrap = sortedIOs[i].wrap;
                        currentHead.lpos = sortedIOs[i].endLpos;
                    }
                }
            }
            direction = -1; // 改变扫描方向
            // 赋值为最大值以保证下一轮扫描中大于当前位置的请求不会被忽略
            currentHead.lpos = MAX_LPOS + 1;
        }
        else
        {
            // 从 EOT 向 BOT 扫描
            for (int32_t i = partition_start + partition_len - 1; i >= partition_start; --i)
            {
                if (!(sortedIOs[i].wrap & 1) || vis[sortedIOs[i].id])
                {
                    continue;
                }
                if (scan_method == 1)
                { // SCAN1，两趟扫描
                    if (sortedIOs[i].startLpos <= currentHead.lpos || currentHead.lpos == 0)
                    {
                        output->sequence[index++] = sortedIOs[i].id;
                        vis[sortedIOs[i].id] = 1;
                        currentHead.wrap = sortedIOs[i].wrap;
                        currentHead.lpos = sortedIOs[i].startLpos;
                    }
                }
                else if (scan_method == 2)
                { // SCAN2，多趟扫描
                    if (sortedIOs[i].startLpos < currentHead.lpos || currentHead.lpos == 0)
                    {
                        output->sequence[index++] = sortedIOs[i].id;
                        vis[sortedIOs[i].id] = 1;
                        currentHead.wrap = sortedIOs[i].wrap;
                        currentHead.lpos = sortedIOs[i].endLpos;
                    }
                }
            }
            direction = 1; // 改变扫描方向
            // 赋值为0以保证下一轮扫描中小于当前位置的请求不会被忽略
            currentHead.lpos = 0;
        }
    }
    return RETURN_OK;
}

// 用MPScan处理每个分区内部
int32_t MPScan_star_PerPartition(const InputParam *input, OutputParam *output, IOUint *sortedIOs, bool *vis, HeadInfo *head, int partition_start, int partition_end)
{
    uint32_t index = partition_start;
    HeadInfo currentHead = *head;
    int direction = 1; // 扫描方向：1 表示从 BOT 向 EOT 扫描，-1 表示从 EOT 向 BOT 扫描

    while (index < partition_end)
    {
        if (direction == 1)
        {
            // 从 BOT 向 EOT 扫描
            for (uint32_t i = partition_start; i < partition_end; ++i)
            {
                if (sortedIOs[i].wrap & 1 || vis[sortedIOs[i].id])
                    continue;
                if (sortedIOs[i].startLpos > currentHead.lpos || currentHead.lpos == 0)
                {
                    output->sequence[index++] = sortedIOs[i].id;
                    vis[sortedIOs[i].id] = 1;
                    currentHead.wrap = sortedIOs[i].wrap;
                    currentHead.lpos = sortedIOs[i].endLpos;
                }
            }
            direction = -1;                  // 改变扫描方向
            currentHead.lpos = MAX_LPOS + 1; // 赋值为最大值以保证下一轮扫描中大于当前位置的请求不会被忽略
        }
        else
        {
            // 从 EOT 向 BOT 扫描
            for (int32_t i = partition_end - 1; i >= partition_start; --i)
            {
                if (!(sortedIOs[i].wrap & 1) || vis[sortedIOs[i].id])
                    continue;
                if (sortedIOs[i].startLpos < currentHead.lpos || currentHead.lpos == 0)
                {
                    output->sequence[index++] = sortedIOs[i].id;
                    vis[sortedIOs[i].id] = 1;
                    currentHead.wrap = sortedIOs[i].wrap;
                    currentHead.lpos = sortedIOs[i].endLpos;
                }
            }
            direction = 1;        // 改变扫描方向
            currentHead.lpos = 0; // 赋值为0以保证下一轮扫描中小于当前位置的请求不会被忽略
        }
    }

    // 将最后一轮扫描插入前面
    OutputParam *tmp = (OutputParam *)malloc(sizeof(OutputParam));
    tmp->len = input->ioVec.len;
    tmp->sequence = (uint32_t *)malloc(input->ioVec.len * sizeof(uint32_t));
    memcpy(tmp->sequence, output->sequence, input->ioVec.len * sizeof(int));
    AccessTime accessTime;
    TotalAccessTime(input, output, &accessTime);
    while (true)
    {
        // 最后一轮扫描在 output 中的下标范围为 [idx+1, output->len - 1]
        int32_t idx = partition_end - 1;
        if (input->ioVec.ioArray[output->sequence[idx] - 1].wrap & 1)
            while (idx > partition_start && (input->ioVec.ioArray[output->sequence[idx - 1] - 1].wrap & 1) &&
                   input->ioVec.ioArray[output->sequence[idx] - 1].startLpos < input->ioVec.ioArray[output->sequence[idx - 1] - 1].startLpos)
            { // 奇数
                idx--;
            }
        if (!(input->ioVec.ioArray[output->sequence[idx] - 1].wrap & 1))
            while (idx > partition_start && !(input->ioVec.ioArray[output->sequence[idx - 1] - 1].wrap & 1) &&
                   input->ioVec.ioArray[output->sequence[idx] - 1].startLpos > input->ioVec.ioArray[output->sequence[idx - 1] - 1].startLpos)
            { // 偶数
                idx--;
            }
        if (idx == partition_start)
            break; // 当前已经是最后一轮扫描

        // 遍历最后一轮的每个 IO
        for (int i = idx; i < partition_end; ++i)
        {
            int32_t minTime = INT32_MAX;
            int32_t best_pos = -1;
            HeadInfo z = {input->ioVec.ioArray[output->sequence[i] - 1].wrap, input->ioVec.ioArray[output->sequence[i] - 1].startLpos, HEAD_RW};

            // 寻找插入的最佳位置
            for (int j = partition_start; j < i - 1; ++j)
            {
                HeadInfo x = {input->ioVec.ioArray[output->sequence[j] - 1].wrap, input->ioVec.ioArray[output->sequence[j] - 1].startLpos, HEAD_RW};
                HeadInfo y = {input->ioVec.ioArray[output->sequence[j + 1] - 1].wrap, input->ioVec.ioArray[output->sequence[j + 1] - 1].startLpos, HEAD_RW};
                int32_t seekTime = SeekTimeCalculate(&x, &z) + SeekTimeCalculate(&z, &y) - SeekTimeCalculate(&x, &y);
                if (seekTime < minTime)
                {
                    best_pos = j;
                    minTime = seekTime;
                }
            }

            if (best_pos == -1)
            {
                continue;
            }
            // 将当前 IO 插入到 best_pos 后面
            for (int j = i; j > best_pos + 1; --j)
                tmp->sequence[j] = tmp->sequence[j - 1];
            tmp->sequence[best_pos + 1] = output->sequence[i]; // 更新该处的 IO 序号
        }

        AccessTime tmpTime;
        TotalAccessTime(input, tmp, &tmpTime);

        if (tmpTime.addressDuration < accessTime.addressDuration)
        {
            accessTime.addressDuration = tmpTime.addressDuration;
            memcpy(output->sequence, tmp->sequence, input->ioVec.len * sizeof(int));
        }
        else
        {
            break;
        }
    }

    free(tmp->sequence);
    free(tmp);

    return RETURN_OK;
}

int32_t SCAN2(const InputParam *input, OutputParam *output)
{
    // 初始化输出参数
    output->len = input->ioVec.len;

    // 复制 IO 请求数组并按 lpos 排序
    IOUint *sortedIOs = (IOUint *)malloc(input->ioVec.len * sizeof(IOUint));
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
    {
        sortedIOs[i] = input->ioVec.ioArray[i];
    }
    QuickSort(sortedIOs, input->ioVec.len);

    // 初始化当前头位置为输入的头状态
    HeadInfo currentHead = {input->headInfo.wrap, input->headInfo.lpos, input->headInfo.status};
    // HeadInfo currentHead = {0, 0, 0};

    // 扫描方向：1 表示从 BOT 向 EOT 扫描，-1 表示从 EOT 向 BOT 扫描
    int direction = 1;
    uint32_t index = 0;

    bool vis[input->ioVec.len + 1];
    memset(vis, 0, sizeof(vis));
    int last_cnt = 1;
    while (index < input->ioVec.len)
    {
        if (direction == 1)
        {
            int cur_cnt = 0;
            // 从 BOT 向 EOT 扫描
            for (uint32_t i = 0; i < input->ioVec.len; ++i)
            {
                if (sortedIOs[i].wrap & 1 || vis[sortedIOs[i].id])
                {
                    continue;
                }
                if (sortedIOs[i].startLpos >= currentHead.lpos)
                {
                    output->sequence[index++] = sortedIOs[i].id;
                    vis[sortedIOs[i].id] = 1;
                    cur_cnt++;
                    currentHead.wrap = sortedIOs[i].wrap;
                    currentHead.lpos = sortedIOs[i].endLpos;
                }
            }
            direction = -1; // 改变扫描方向
            currentHead.lpos = MAX_LPOS;
            last_cnt = cur_cnt;
        }
        else
        {
            int cur_cnt = 0;
            // 从 EOT 向 BOT 扫描
            for (int32_t i = input->ioVec.len - 1; i >= 0; --i)
            {
                if (!(sortedIOs[i].wrap & 1) || vis[sortedIOs[i].id])
                {
                    continue;
                }
                if (sortedIOs[i].startLpos <= currentHead.lpos)
                {
                    output->sequence[index++] = sortedIOs[i].id;
                    vis[sortedIOs[i].id] = 1;
                    cur_cnt++;
                    currentHead.wrap = sortedIOs[i].wrap;
                    currentHead.lpos = sortedIOs[i].endLpos;
                }
            }
            direction = 1; // 改变扫描方向
            currentHead.lpos = 0;
            last_cnt = cur_cnt;
        }
    }

    free(sortedIOs);
    return RETURN_OK;
}

int32_t MPScan(const InputParam *input, OutputParam *output)
{
    SCAN2(input, output);

    // 将最后一轮扫描插入前面
    OutputParam *tmp = (OutputParam *)malloc(sizeof(OutputParam));
    tmp->len = input->ioVec.len;
    tmp->sequence = (uint32_t *)malloc(input->ioVec.len * sizeof(uint32_t));
    memcpy(tmp->sequence, output->sequence, input->ioVec.len * sizeof(int));
    AccessTime accessTime;
    TotalAccessTime(input, output, &accessTime);
    while (true)
    {
        // 最后一轮扫描在 output 中的下标范围为 [idx+1, output->len - 1]
        int32_t io_len = 0, idx = output->len - 1;
        while (idx >= 1 && (input->ioVec.ioArray[output->sequence[idx] - 1].wrap & 1) &&
               input->ioVec.ioArray[output->sequence[idx] - 1].startLpos < input->ioVec.ioArray[output->sequence[idx - 1] - 1].startLpos)
        { // 奇数
            idx--;
        }
        while (idx >= 1 && !(input->ioVec.ioArray[output->sequence[idx] - 1].wrap & 1) &&
               input->ioVec.ioArray[output->sequence[idx] - 1].startLpos > input->ioVec.ioArray[output->sequence[idx - 1] - 1].startLpos)
        { // 偶数
            idx--;
        }
        if (idx < 0)
            break; // 当前已经是最后一轮扫描

        // printf("\n%d:", output->sequence[idx]);
        // 遍历最后一轮的每个 IO
        for (int i = idx; i < output->len; ++i)
        {
            // printf("\ni = %d , ", output->sequence[i]);
            int32_t minTime = INT32_MAX;
            int32_t best_pos = -1;
            HeadInfo z = {input->ioVec.ioArray[output->sequence[i] - 1].wrap, input->ioVec.ioArray[output->sequence[i] - 1].startLpos, HEAD_RW};

            // 寻找插入的最佳位置
            for (int j = 0; j < i - 1; ++j)
            {
                // if(input->ioVec.ioArray[output->sequence[j]-1].wrap != input->ioVec.ioArray[output->sequence[i]-1].wrap)
                //     continue;
                HeadInfo x = {input->ioVec.ioArray[output->sequence[j] - 1].wrap, input->ioVec.ioArray[output->sequence[j] - 1].startLpos, HEAD_RW};
                HeadInfo y = {input->ioVec.ioArray[output->sequence[j + 1] - 1].wrap, input->ioVec.ioArray[output->sequence[j + 1] - 1].startLpos, HEAD_RW};
                int32_t seekTime = SeekTimeCalculate(&x, &z) + SeekTimeCalculate(&z, &y) - SeekTimeCalculate(&x, &y);
                // printf("seekTime = %d, minTime = %d ", seekTime, minTime);
                if (seekTime < minTime)
                {
                    best_pos = j;
                    minTime = seekTime;
                }
            }

            // printf("best_pos = %d , ", best_pos);
            // 将当前 IO 插入到 best_pos 后面
            for (int j = i; j > best_pos + 1; --j)
            {
                tmp->sequence[j] = tmp->sequence[j - 1];
            }
            tmp->sequence[best_pos + 1] = output->sequence[i]; // 更新该处的 IO 序号
        }

        AccessTime tmpTime;
        TotalAccessTime(input, tmp, &tmpTime);

        // printf("\ntmp[ ");
        // for (int i = 0; i < tmp->len; ++i) {
        //     printf("%d ", tmp->sequence[i]);
        // }
        // printf("]\nout[ ");
        // for (int i = 0; i < output->len; ++i) {
        //     printf("%d ", output->sequence[i]);
        // }
        // printf("]\ntmp: %d, output: %d\n", tmpTime.addressDuration, accessTime.addressDuration);

        if (tmpTime.addressDuration < accessTime.addressDuration)
        {
            accessTime.addressDuration = tmpTime.addressDuration;
            memcpy(output->sequence, tmp->sequence, input->ioVec.len * sizeof(int));
        }
        else
        {
            break;
        }
    }

    return RETURN_OK;
}

MinHeap *createMinHeap(int capacity)
{
    MinHeap *heap = (MinHeap *)malloc(sizeof(MinHeap));
    heap->size = 0;
    heap->capacity = capacity;
    heap->nodes = (Node *)malloc(capacity * sizeof(Node));
    return heap;
}

void swap(Node *a, Node *b)
{
    Node temp = *a;
    *a = *b;
    *b = temp;
}

void heapify(MinHeap *heap, int idx)
{
    int smallest = idx;
    int left = 2 * idx + 1;
    int right = 2 * idx + 2;

    if (left < heap->size && heap->nodes[left].dis < heap->nodes[smallest].dis)
        smallest = left;

    if (right < heap->size && heap->nodes[right].dis < heap->nodes[smallest].dis)
        smallest = right;

    if (smallest != idx)
    {
        swap(&heap->nodes[idx], &heap->nodes[smallest]);
        heapify(heap, smallest);
    }
}

Node *extractMin(MinHeap *heap)
{ // 弹出最小值
    if (heap->size == 0)
    {
        return NULL;
    }

    swap(&heap->nodes[0], &heap->nodes[heap->size - 1]);
    heap->size--;
    heapify(heap, 0);

    return &heap->nodes[heap->size];
}

Node *getMin(MinHeap *heap) // 弹出最小值
{
    if (heap->size == 0)
    {
        return NULL;
    }

    return &heap->nodes[0];
}

void insertHeap(MinHeap *heap, Node node)
{
    if (heap->size == heap->capacity)
    {
        printf("Heap overflow\n");
        return;
    }

    heap->size++;
    int i = heap->size - 1;
    heap->nodes[i] = node;

    // 向上调整
    while (i != 0 && heap->nodes[(i - 1) / 2].dis > heap->nodes[i].dis)
    {
        swap(&heap->nodes[i], &heap->nodes[(i - 1) / 2]);
        i = (i - 1) / 2;
    }
}

int getValueInHeapArray(MinHeapArray *arr, int idx)
{
    return getMin(arr->heap_array[idx])->dis;
}

int getMinValueInHeapArray(MinHeapArray *arr)
{
    int value = getMin(arr->heap_array[0])->dis;
    return value;
}

void swapInHeapArray(MinHeapArray *arr, int idx1, int idx2)
{
    MinHeap *tmp = arr->heap_array[idx1];
    arr->heap_array[idx1] = arr->heap_array[idx2];
    arr->heap_array[idx2] = tmp;
}

void insertHeapInHeapArray(MinHeapArray *arr, MinHeap *heap)
{
    arr->heap_array[arr->size++] = heap;
    int i = arr->size - 1;
    while (i != 0 && getValueInHeapArray(arr, (i - 1) / 2) >= getValueInHeapArray(arr, i))
    {
        swapInHeapArray(arr, i, (i - 1) / 2);
        i = (i - 1) / 2;
    }
}

Node *getNodeInHeapArray(MinHeapArray *arr)
{
    return getMin(arr->heap_array[0]);
}

Node *popNodeInHeapArray(MinHeapArray *arr)
{
    if (arr->size == 0)
    {
        return NULL;
    }
    Node *tmp = extractMin(arr->heap_array[0]);
    minHeapArrayHeapify(arr, 0);
    return tmp;
}

MinHeap *popHeapInHeapArray(MinHeapArray *arr)
{
    if (arr->size == 0)
    {
        return NULL;
    }
    MinHeap *tmp = arr->heap_array[0];
    arr->heap_array[0] = arr->heap_array[arr->size - 1];
    arr->heap_array[arr->size - 1] = tmp;
    arr->size--;

    minHeapArrayHeapify(arr, 0);

    return arr->heap_array[arr->size];
}

void minHeapArrayHeapify(MinHeapArray *arr, int idx)
{
    int smallest = idx;
    int left = 2 * idx + 1;
    int right = 2 * idx + 2;

    if (left < arr->size && getValueInHeapArray(arr, left) < getValueInHeapArray(arr, smallest))
        smallest = left;

    if (right < arr->size && getValueInHeapArray(arr, right) < getValueInHeapArray(arr, smallest))
        smallest = right;

    if (smallest != idx)
    {
        swapInHeapArray(arr, idx, smallest);
        minHeapArrayHeapify(arr, smallest);
    }
}

const int maxn = 10001;
int *fa, *h, *sz;

void initUnionSet()
{
    fa = (int *)malloc(maxn * sizeof(int));
    h = (int *)malloc(maxn * sizeof(int));
    sz = (int *)malloc(maxn * sizeof(int));
    for (int i = 0; i < maxn; ++i)
    {
        fa[i] = i;
        h[i] = 1;
        sz[i] = 1;
    }
}

void freeUnionSet()
{
    free(fa);
    free(h);
}

int find(int x)
{
    return x == fa[x] ? x : (fa[x] = find(fa[x]));
}

void unite(int x, int y)
{
    x = find(x), y = find(y);
    if (h[x] < h[y])
    {
        fa[x] = y;
        sz[x] = sz[y] = sz[x] + sz[y];
    }
    else
    {
        fa[y] = x;
        if (h[x] == h[y])
            h[y]++;
        sz[x] = sz[y] = sz[x] + sz[y];
    }
}

void destoryMinHeap(MinHeap *heap)
{
    free(heap->nodes);
    free(heap);
    heap = NULL;
}

int32_t TailReinsert(const InputParam *input, OutputParam *output, int *pos, int pointnums)
{
    AccessTime accessTime;
    TotalAccessTime(input, output, &accessTime);
    uint32_t tapeBeltWear = TotalTapeBeltWearTimes(input, output, NULL); // 带体磨损
    uint32_t tapeMotorWear = TotalMotorWearTimes(input, output);         // 电机磨损

    uint32_t currentCost = accessTime.addressDuration + tapeBeltWear + tapeMotorWear; // 总代价
    uint32_t bestCost = currentCost;
    printf("cost before TailReinsert:%d\n", currentCost);

    // 将最后的点插入前面
    OutputParam *tmp = (OutputParam *)malloc(sizeof(OutputParam));
    tmp->len = input->ioVec.len;
    tmp->sequence = (uint32_t *)malloc(input->ioVec.len * sizeof(uint32_t));
    memcpy(tmp->sequence, output->sequence, input->ioVec.len * sizeof(int));

    uint32_t tail = pointnums - 1;
    while (tail)
    {
        int p_now = 0;
        for (int i = 0; i < output->len; i++)
            if (output->sequence[i] == pos[tail])
                p_now = i;

        printf("tail: %d, pos_now: %d\n", tail, p_now);

        // 最后一个 IO
        int32_t minCost = INT32_MAX;
        int32_t best_pos = -1;
        HeadInfo z = {input->ioVec.ioArray[output->sequence[p_now] - 1].wrap, input->ioVec.ioArray[output->sequence[p_now] - 1].startLpos, HEAD_RW};

        // 寻找插入的最佳位置，从前一个点的前面位置开始
        for (int j = 0; j < input->ioVec.len - 1; ++j)
        {
            if (j == p_now - 1)
                continue;
            HeadInfo x = {input->ioVec.ioArray[output->sequence[j] - 1].wrap, input->ioVec.ioArray[output->sequence[j] - 1].startLpos, HEAD_RW};
            HeadInfo y = {input->ioVec.ioArray[output->sequence[j + 1] - 1].wrap, input->ioVec.ioArray[output->sequence[j + 1] - 1].startLpos, HEAD_RW};
            int32_t cost = getCost(&x, &z) + getCost(&z, &y) - getCost(&x, &y);
            if (cost < minCost)
            {
                best_pos = j;
                minCost = cost;
            }
        }

        if (best_pos == -1)
        {
            tail--;
            continue;
        }

        // 将当前 IO 插入到 best_pos 后面
        if (best_pos > p_now)
        {
            for (int j = p_now; j <= best_pos; ++j)
                tmp->sequence[j] = tmp->sequence[j + 1];
            tmp->sequence[best_pos + 1] = output->sequence[p_now];
        }
        else
        {
            for (int j = p_now; j > best_pos + 1; --j)
                tmp->sequence[j] = tmp->sequence[j - 1];
            tmp->sequence[best_pos + 1] = output->sequence[p_now]; // 更新该处的 IO 序号
        }
        AccessTime tmpTime;
        TotalAccessTime(input, tmp, &tmpTime);
        tapeBeltWear = TotalTapeBeltWearTimes(input, tmp, NULL); // 带体磨损
        tapeMotorWear = TotalMotorWearTimes(input, tmp);         // 电机磨损

        currentCost = tmpTime.addressDuration + tapeBeltWear + tapeMotorWear; // 总代价
        printf("currentCost:%u\n", currentCost);
        if (currentCost < bestCost)
        {
            bestCost = currentCost;
            memcpy(output->sequence, tmp->sequence, input->ioVec.len * sizeof(int));
        }
        tail--;
    }

    printf("accessTime after TailReinsert:%d\n", bestCost);

    free(tmp->sequence);
    free(tmp);
}

int32_t cycleMerge(const InputParam *input, OutputParam *output){
    struct KM km;
    int n = input->ioVec.len + 1;
    kmInit(&km, n, n, n);
    kmSolve(&km);
    cycleMergeMain(&km);

    int cnt = 0;
    int now = 1;
    while (cnt < input->ioVec.len)
    {
        output->sequence[cnt++] = km.next[now] - 1;
        now = km.next[now];
    }
    kmClear(&km);
    if(dist_martix){
        free(dist_martix);
        dist_martix = NULL;
    }

    return RETURN_OK;
}

int32_t fastCycleMerge(const InputParam *input, OutputParam *output)
{
    int duration_us = getDurationMicroseconds();

    printf("[time] %s: start at %d\n", __func__, duration_us);
    struct KM km;
    int n = input->ioVec.len + 1;
    kmInit(&km, n, n, n);
    int match[maxn + 10];
    kmGetDistance(1,1);
    duration_us = getDurationMicroseconds();
    printf("[time] %s: finish dist matrix %d\n", __func__, duration_us);
    if(hungarianMinimumWeightPerfectMatchingDenseGraph_C(n, dist_martix, match)){
        duration_us = getDurationMicroseconds();
        printf("[time] %s: matching found at %d\n", __func__, duration_us);
        for (int i = 0; i < n; i++)
        {
            int left = i + 1;
            int right = match[i] + 1;
            // printf("%d, %d, %d\n", i, left, right);
            km.next[left] = right;
            km.match[right] = left;
        }
        cycleMergeMain(&km);
        duration_us = getDurationMicroseconds();
        printf("[time] %s: merge end at %d\n", __func__, duration_us);
    }
    

    int cnt = 0;
    int now = 1;
    while (cnt < input->ioVec.len)
    {
        output->sequence[cnt++] = km.next[now] - 1;
        now = km.next[now];
    }
    kmClear(&km);
    if (dist_martix)
    {
        free(dist_martix);
        dist_martix = NULL;
    }

    return RETURN_OK;
}

int32_t merge(const InputParam *input, OutputParam *output)
{
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
    {
        output->sequence[i] = input->ioVec.ioArray[i].id;
    }

    MinHeapArray heap_array = {maxn, 0, (MinHeap **)malloc(maxn * sizeof(MinHeap *))};
    MinHeap *heap = createMinHeap(maxn + 1);

    int selected_value_sum = 0;

    // 初始化当前头位置为输入的头状态
    int max_edge_num = 8 * 1024 * 1024 / 8; // 假设最多分配8MB的内存给节点
    int edge_per_node = max_edge_num / (input->ioVec.len + 1);
    if (edge_per_node > input->ioVec.len + 1)
    {
        edge_per_node = input->ioVec.len + 1;
    }
    HeadInfo currentHead = {input->headInfo.wrap, input->headInfo.lpos, input->headInfo.status};
    for (int i = 0; i < input->ioVec.len; i++)
    {
        HeadInfo status_tmp = {input->ioVec.ioArray[i].wrap, input->ioVec.ioArray[i].startLpos, HEAD_RW};
        Node min_node = {0, i + 1, getCost(&currentHead, &status_tmp)};

        for (int j = 0; j < input->ioVec.len; j++)
        {
            if (i == j)
                continue;
            HeadInfo status1 = {input->ioVec.ioArray[j].wrap, input->ioVec.ioArray[j].endLpos, HEAD_RW};

            Node tmp_node = {j + 1, i + 1, getCost(&status1, &status_tmp)};
            if (tmp_node.dis < min_node.dis)
            {
                min_node = tmp_node;
            }
        }
        insertHeap(heap, min_node);
    }

    int nex[maxn], vis[maxn];
    memset(nex, 0, sizeof(nex));
    memset(vis, 0, sizeof(vis));
    initUnionSet();
    int set_num = input->ioVec.len + 1;

    // 记录最后merge选取的最后10条边
    int lastpoints = 10;
    int *pos = (int *)malloc(lastpoints * sizeof(int));
    int tot_sz = 0;

    while (sz[0] != input->ioVec.len + 1)
    {
        if (sz[0] == input->ioVec.len + 1)
        {
            break;
        }

        int min_value = INT32_MAX;
        int min_heap_idx = -1;
        Node *node = extractMin(heap);

        if (nex[node->x] == 0 && (node->x == 0 || (nex[node->y] != node->x)) && vis[node->y] == 0 && find(node->x) != find(node->y))
        {
            // TODO: tail sort
            // tot_sz++;
            // // 记录最后merge选取的最后10条边
            // if (tot_sz > input->ioVec.len - lastpoints)
            // {
            //     pos[tot_sz + lastpoints - input->ioVec.len - 1] = node->y;
            //     printf("pos[%d]=%d\n", tot_sz + lastpoints - input->ioVec.len - 1, node->y);
            // }
            unite(node->x, node->y);
            nex[node->x] = node->y;
            vis[node->y] = 1;
            selected_value_sum += node->dis;
            set_num--;
        }
        else
        {
            int target_id = node->y;
            HeadInfo status_tmp = {input->ioVec.ioArray[target_id - 1].wrap, input->ioVec.ioArray[target_id - 1].startLpos, HEAD_RW};
            Node min_node;
            min_node.dis = INT32_MAX;
            if (nex[0] == 0)
            {
                Node tmp_node = {0, target_id, getCost(&currentHead, &status_tmp)};
                min_node = tmp_node;
            }
            for (int source_id = 1; source_id < input->ioVec.len + 1; source_id++)
            {
                if (target_id == source_id || nex[source_id] != 0 || find(source_id) == find(target_id))
                    continue;
                HeadInfo status1 = {input->ioVec.ioArray[source_id - 1].wrap, input->ioVec.ioArray[source_id - 1].endLpos, HEAD_RW};
                Node tmp_node = {source_id, target_id, getCost(&status1, &status_tmp)};
                if (tmp_node.dis < min_node.dis)
                {
                    min_node = tmp_node;
                }
            }
            insertHeap(heap, min_node);
        }
    }
    int now = nex[0], cnt = 0;
    while (cnt < input->ioVec.len)
    {
        output->sequence[cnt++] = now;
        now = nex[now];
    }
    destoryMinHeap(heap);

    // TailReinsert(input, output, pos, lastpoints);

    return RETURN_OK;
}

Node *randomExtractMin(MinHeap *heap)
{ // 弹出最小值
    if (heap->size == 0)
    {
        return NULL;
    }

    int random_choice = rand() % 100; // 生成 0 到 99 的随机数

    if (random_choice < 90)
    {
        // 90% 的概率选择堆中的最小节点（堆顶）
        swap(&heap->nodes[0], &heap->nodes[heap->size - 1]);
    }
    else
    {
        // 10% 的概率随机选择堆中的其他节点
        if (heap->size > 1)
        {
            int random_index = (rand() % (heap->size - 1) / 2) + 1;         // 生成 1 到 heap->size - 1 的随机索引
            swap(&heap->nodes[random_index], &heap->nodes[heap->size - 1]); // 将随机节点与最后一个节点交换
        }
        else
        {
            swap(&heap->nodes[0], &heap->nodes[heap->size - 1]);
        }
    }

    heap->size--;     // 减少堆大小
    heapify(heap, 0); // 对堆进行堆化，维护堆的特性

    return &heap->nodes[heap->size]; // 返回被弹出的节点
}

int32_t merge_random(const InputParam *input, OutputParam *output)
{
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
    {
        output->sequence[i] = input->ioVec.ioArray[i].id;
    }

    // return RETURN_OK;

    MinHeapArray heap_array = {maxn, 0, (MinHeap **)malloc(maxn * sizeof(MinHeap *))};
    MinHeap *heap = createMinHeap(maxn + 1);

    int selected_value_sum = 0;

    // 初始化当前头位置为输入的头状态
    int max_edge_num = 8 * 1024 * 1024 / 8; // 假设最多分配8MB的内存给节点
    int edge_per_node = max_edge_num / (input->ioVec.len + 1);
    if (edge_per_node > input->ioVec.len + 1)
    {
        edge_per_node = input->ioVec.len + 1;
    }
    HeadInfo currentHead = {input->headInfo.wrap, input->headInfo.lpos, input->headInfo.status};
    for (int i = 0; i < input->ioVec.len; i++)
    {
        HeadInfo status_tmp = {input->ioVec.ioArray[i].wrap, input->ioVec.ioArray[i].startLpos, HEAD_RW};
        Node min_node = {0, i + 1, getCost(&currentHead, &status_tmp)};

        for (int j = 0; j < input->ioVec.len; j++)
        {
            if (i == j)
                continue;
            HeadInfo status1 = {input->ioVec.ioArray[j].wrap, input->ioVec.ioArray[j].endLpos, HEAD_RW};

            Node tmp_node = {j + 1, i + 1, getCost(&status1, &status_tmp)};
            if (tmp_node.dis < min_node.dis)
            {
                min_node = tmp_node;
            }
        }
        insertHeap(heap, min_node);
    }

    int nex[maxn], vis[maxn];
    memset(nex, 0, sizeof(nex));
    memset(vis, 0, sizeof(vis));
    initUnionSet();
    int set_num = input->ioVec.len + 1;
    while (sz[0] != input->ioVec.len + 1)
    {
        if (sz[0] == input->ioVec.len + 1)
        {
            break;
        }

        int min_value = INT32_MAX;
        int min_heap_idx = -1;
        Node *node = randomExtractMin(heap);

        if (nex[node->x] == 0 && (node->x == 0 || (nex[node->y] != node->x)) && vis[node->y] == 0 && find(node->x) != find(node->y))
        {
            unite(node->x, node->y);
            nex[node->x] = node->y;
            vis[node->y] = 1;
            selected_value_sum += node->dis;
            set_num--;
            // printf("%d\n", set_num);
        }
        else
        {
            int target_id = node->y;
            // printf("target_id: %d, source_id: %d, %d, %d, %d, %d, %d, %d\n", target_id, node->x, nex[node->x], vis[node->x], vis[node->y], nex[node->y] ,find(node->x) ,find(node->y));
            HeadInfo status_tmp = {input->ioVec.ioArray[target_id - 1].wrap, input->ioVec.ioArray[target_id - 1].startLpos, HEAD_RW};
            Node min_node;
            min_node.dis = INT32_MAX;
            if (nex[0] == 0)
            {
                Node tmp_node = {0, target_id, getCost(&currentHead, &status_tmp)};
                min_node = tmp_node;
            }
            for (int source_id = 1; source_id < input->ioVec.len + 1; source_id++)
            {
                if (target_id == source_id || nex[source_id] != 0 || find(source_id) == find(target_id))
                    continue;
                HeadInfo status1 = {input->ioVec.ioArray[source_id - 1].wrap, input->ioVec.ioArray[source_id - 1].endLpos, HEAD_RW};
                Node tmp_node = {source_id, target_id, getCost(&status1, &status_tmp)};
                if (tmp_node.dis < min_node.dis)
                {
                    min_node = tmp_node;
                }
            }
            insertHeap(heap, min_node);
        }
    }
    int now = nex[0], cnt = 0;
    while (cnt < input->ioVec.len)
    {
        output->sequence[cnt++] = now;
        now = nex[now];
    }
    destoryMinHeap(heap);

    // free(dis);
    return RETURN_OK;
}

int32_t NearestNeighborAlgorithm(const InputParam *input, OutputParam *output)
{
    // 初始化输出参数
    output->len = input->ioVec.len;

    // 记录哪些请求已经被处理
    bool processed[input->ioVec.len];
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
    {
        processed[i] = false;
    }

    // 初始化当前头位置为输入的头状态
    HeadInfo currentHead = {input->headInfo.wrap, input->headInfo.lpos, input->headInfo.status};

    // 对于每一个请求，找到距离当前头位置最近的未处理请求
    for (uint32_t i = 0; i < input->ioVec.len; ++i)
    {
        int32_t minSeekTime = INT32_MAX;
        uint32_t nextRequestIndex = 0;

        for (uint32_t j = 0; j < input->ioVec.len; ++j)
        {
            if (processed[j])
                continue;

            HeadInfo nextHead = {input->ioVec.ioArray[j].wrap, input->ioVec.ioArray[j].startLpos, HEAD_RW};
            // int32_t seekTime = SeekTimeCalculate(&currentHead, &nextHead);
            int32_t seekTime = getCost(&currentHead, &nextHead);

            if (seekTime < minSeekTime)
            {
                minSeekTime = seekTime;
                nextRequestIndex = j;
            }
        }

        // 更新当前头位置为找到的最近请求的末尾位置
        currentHead.wrap = input->ioVec.ioArray[nextRequestIndex].wrap;
        currentHead.lpos = input->ioVec.ioArray[nextRequestIndex].endLpos;
        currentHead.status = HEAD_RW;

        // 将该请求标记为已处理
        processed[nextRequestIndex] = true;

        // 将该请求添加到输出序列中
        output->sequence[i] = input->ioVec.ioArray[nextRequestIndex].id;
    }

    return RETURN_OK;
}

// 包装 merge 函数的线程执行函数
// TODO：cost加上磨损
void *merge_thread(void *arg)
{
    ThreadArg *threadArg = (ThreadArg *)arg;

    // 创建线程返回值
    ThreadResult *result = (ThreadResult *)malloc(sizeof(ThreadResult));
    result->output = threadArg->output;

    merge(threadArg->input, result->output);
    TotalAccessTime(threadArg->input, result->output, &result->accessTime);

    pthread_exit(result); // 返回结果
}

// 包装 merge_random 函数的线程执行函数
// TODO：cost加上磨损
void *merge_random_thread(void *arg)
{
    ThreadArg *threadArg = (ThreadArg *)arg;

    // 创建线程返回值
    ThreadResult *result = (ThreadResult *)malloc(sizeof(ThreadResult));
    result->output = threadArg->output;

    merge_random(threadArg->input, result->output);
    TotalAccessTime(threadArg->input, result->output, &result->accessTime);

    pthread_exit(result); // 返回结果
}

// 线程1：执行 partition_scan 和 merge
// TODO：cost加上磨损
void *partition_scan_merge_thread(void *arg)
{
    int min_time = 0x3f3f3f3f;
    ThreadArg *threadArg = (ThreadArg *)arg;

    int *best_sequence = (int *)malloc(threadArg->input->ioVec.len * sizeof(int));
    if (!best_sequence)
        return NULL; // 内存分配失败，直接返回

    partition_scan(threadArg->input, threadArg->output);
    TotalAccessTime(threadArg->input, threadArg->output, threadArg->accessTime);
    if (threadArg->accessTime->addressDuration < min_time)
    {
        min_time = threadArg->accessTime->addressDuration;
        memcpy(best_sequence, threadArg->output->sequence, threadArg->input->ioVec.len * sizeof(int));
    }

    merge(threadArg->input, threadArg->output);
    TotalAccessTime(threadArg->input, threadArg->output, threadArg->accessTime);
    if (threadArg->accessTime->addressDuration < min_time)
    {
        min_time = threadArg->accessTime->addressDuration;
        memcpy(best_sequence, threadArg->output->sequence, threadArg->input->ioVec.len * sizeof(int));
    }

    memcpy(threadArg->output->sequence, best_sequence, threadArg->input->ioVec.len * sizeof(int));
    free(best_sequence);
    return NULL;
}

// 线程2：执行 MPScan 和 merge_random
// TODO：cost加上磨损
void *mp_scan_merge_random_thread(void *arg)
{
    int min_time = 0x3f3f3f3f;
    ThreadArg *threadArg = (ThreadArg *)arg;

    int *best_sequence = (int *)malloc(threadArg->input->ioVec.len * sizeof(int));
    if (!best_sequence)
        return NULL; // 内存分配失败，直接返回

    MPScan(threadArg->input, threadArg->output);
    TotalAccessTime(threadArg->input, threadArg->output, threadArg->accessTime);
    if (threadArg->accessTime->addressDuration < min_time)
    {
        min_time = threadArg->accessTime->addressDuration;
        memcpy(best_sequence, threadArg->output->sequence, threadArg->input->ioVec.len * sizeof(int));
    }

    merge_random(threadArg->input, threadArg->output);
    TotalAccessTime(threadArg->input, threadArg->output, threadArg->accessTime);
    if (threadArg->accessTime->addressDuration < min_time)
    {
        min_time = threadArg->accessTime->addressDuration;
        memcpy(best_sequence, threadArg->output->sequence, threadArg->input->ioVec.len * sizeof(int));
    }

    memcpy(threadArg->output->sequence, best_sequence, threadArg->input->ioVec.len * sizeof(int));
    free(best_sequence);
    return NULL;
}