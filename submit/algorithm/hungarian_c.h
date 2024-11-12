#ifndef HUNGARIAN_C_H
#define HUNGARIAN_C_H
#ifdef __cplusplus
extern "C"
{
#endif


#define INF 0x3f3f3f3f
int hungarianMinimumWeightPerfectMatchingDenseGraph_C(int n, int * dist_matrix, int * match);


#ifdef __cplusplus
}
#endif
#endif