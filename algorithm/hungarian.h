#ifndef HUNGARIAN_H
#define HUNGARIAN_H

#include <stdio.h>
#include <vector>
#include <memory>
#include <sys/time.h>
#include "../public.h"
#include "algorithm.h"


#define oo 0x3f3f3f3f
#define UNMATCHED (-1)

int quickSelect(int arr[], int low, int high, int k);

struct WeightedBipartiteEdge {
    int left;
    int right;
    int cost;

    WeightedBipartiteEdge() : left(), right(), cost() {}
    WeightedBipartiteEdge(int left, int right, int cost) : left(left), right(right), cost(cost) {}
};

struct LeftEdge
{
    uint16_t right;
    int cost;

    LeftEdge() : right(), cost() {}
    LeftEdge(int right, int cost) : right(right), cost(cost) {}

    const bool operator<(const LeftEdge &otherEdge) const
    {
        return right < otherEdge.right || (right == otherEdge.right && cost < otherEdge.cost);
    }
}__attribute__((packed));



struct Graph{
    int n;
    virtual int getEdgeCost(int i, int j) const = 0;

    virtual int getEdgeNumFromI(int i) const = 0;

    virtual LeftEdge getEdgeFromI(int i, int edgeIdx) const = 0;
};



struct AdjMatrix: public Graph{
    uint32_t * cost;

    AdjMatrix(InputParam *input){
        n = input->ioVec.len + 1;
        cost = (uint32_t *)malloc((n) * (n)*sizeof(uint32_t));
        // 初始化邻接矩阵

        HeadInfo src_pos, target_pos;
        for(int i = 0; i < n; i++){
            if(i == 0){
                src_pos = input->headInfo;
            }
            else{
                src_pos.lpos = input->ioVec.ioArray[i-1].endLpos;
                src_pos.status = HEAD_RW;
                src_pos.wrap = input->ioVec.ioArray[i-1].wrap;
            }

            for(int j = 0; j < n; j++){
                uint32_t cost_value = oo;
                if(j == i){
                    ;
                }
                else if(j == 0){
                    cost_value = 0;
                }
                else{
                    target_pos.lpos = input->ioVec.ioArray[j-1].startLpos;
                    target_pos.status = HEAD_RW;
                    target_pos.wrap = input->ioVec.ioArray[j-1].wrap;
                    cost_value = getCost(&src_pos, &target_pos);
                }
                if(cost_value > oo){
                    cost_value = oo;
                }
                // {
                //     printf("%d, ", cost_value);
                // }
                cost[i*(n) + j] = cost_value;
            }
            // printf("\n");
        }
    }

    ~AdjMatrix(){
        free(cost);
    }

    int getEdgeCost(int i, int j) const{
        if(cost[i*(n) + j] == oo) return oo;
        return cost[i*(n) + j];
    }

    int getEdgeNumFromI(int i) const{
        return n;
    }

    LeftEdge getEdgeFromI(int i, int edge_idx) const{
        int j = edge_idx;
        LeftEdge edge(j, cost[i*(n) + j]);
        return edge;
    }
};



struct AdjTable : public Graph
{
    int max_edge_per_node;
    const std::shared_ptr<std::vector<LeftEdge>[]> leftEdges;
    InputParam *input_ptr;

    AdjTable(InputParam *input, int max_edge_per_node):leftEdges(std::shared_ptr<std::vector<LeftEdge>[]>(new std::vector<LeftEdge>[input->ioVec.len + 1])){
        n = input->ioVec.len + 1;
        input_ptr = input;
        
        int cost_vec[n];
        int cost_vec_copy[n];
        int near_edge = 5;

        printf("%d\n", max_edge_per_node);


        HeadInfo src_pos, target_pos;
        for(int i = 0; i < n; i++){
            if(i == 0){
                src_pos = input->headInfo;
            }
            else{
                src_pos.lpos = input->ioVec.ioArray[i-1].endLpos;
                src_pos.status = HEAD_RW;
                src_pos.wrap = input->ioVec.ioArray[i-1].wrap;
            }

            for(int j = 0; j < n; j++){
                uint32_t cost_value = oo;
                if(j == i){
                    ;
                }
                else if(j == 0){
                    cost_value = 0;
                }
                else{
                    target_pos.lpos = input->ioVec.ioArray[j-1].startLpos;
                    target_pos.status = HEAD_RW;
                    target_pos.wrap = input->ioVec.ioArray[j-1].wrap;
                    cost_value = getCost(&src_pos, &target_pos);
                }
                if(cost_value > oo){
                    cost_value = oo;
                }
                cost_vec[j] = cost_value;
                cost_vec_copy[j] = cost_value;
            }

            int threshold = quickSelect(cost_vec_copy, 0, n-1, max_edge_per_node - 2*near_edge);

            for(int j = 0; j < n; j++){
                if(j == i) continue;
                if(cost_vec[j] <= threshold || (abs(i-j) < near_edge)){
                    // 加入开销最小的边。加入临近的边保证哈密顿回路的存在。
                    leftEdges[i].push_back(LeftEdge(j, cost_vec[j]));
                }
            }
        }// 结束构建邻接表

        printf("finish build adj table\n");
    }

    void SortAllEdges(){
        for(int i = 0; i < n; i++) {
            std::vector<LeftEdge>& edges = leftEdges[i];
            std::sort(edges.begin(), edges.end());
        }
    }

    int getEdgeNumFromI(int i) const{
        const auto & edges = leftEdges[i];
        return edges.size();
    }

    LeftEdge getEdgeFromI(int i, int edge_idx) const{
        const auto & edges = leftEdges[i];
        LeftEdge edge = edges[edge_idx];
        return edge;
    }

    int getEdgeCost(int i, int j) const {
        // const auto & edges = leftEdges[i];
        // int left = 0;
        // int right = edges.size() - 1;

        // while (left <= right) {
        //     int mid = left + (right - left) / 2;

        //     if (edges[mid].right == j) {
        //         return edges[mid].cost;
        //     } else if (edges[mid].right < j) {
        //         left = mid + 1;
        //     } else {
        //         right = mid - 1;
        //     }
        // }
        // return oo;

        HeadInfo src_pos, target_pos;
        if(i == 0){
            src_pos = input_ptr->headInfo;
        }
        else{
            src_pos.lpos = input_ptr->ioVec.ioArray[i-1].endLpos;
            src_pos.status = HEAD_RW;
            src_pos.wrap = input_ptr->ioVec.ioArray[i-1].wrap;
        }

        uint32_t cost_value = oo;
        if(j == i){
            ;
        }
        else if(j == 0){
            cost_value = 0;
        }
        else{
            target_pos.lpos = input_ptr->ioVec.ioArray[j-1].startLpos;
            target_pos.status = HEAD_RW;
            target_pos.wrap = input_ptr->ioVec.ioArray[j-1].wrap;
            cost_value = getCost(&src_pos, &target_pos);
        }

        return cost_value;
    }
};

struct MergePoint{
    int vis_start, vis_end, unv_start, unv_end;
    int cost;
    MergePoint(int _vis_start, int _vis_end, int _unv_start, int _unv_end, int _cost): vis_start(_vis_start), vis_end(_vis_end), unv_start(_unv_start), unv_end(_unv_end), cost(_cost) {}
};

// Given the number of nodes on each side of the bipartite graph and a list of edges, returns a minimum-weight perfect matching.
// If a matching is found, returns a length-n vector, giving the nodes on the right that the left nodes are matched to.
// If no matching exists, returns an empty vector.
// (Note: Edges with endpoints out of the range [0, n) are ignored.)
const std::vector<int> hungarianMinimumWeightPerfectMatching(int n, const std::vector<WeightedBipartiteEdge> allEdges);

const std::vector<int> hungarianMinimumWeightPerfectMatchingDenseGraph(const AdjMatrix & adjMatrix);


void localCycleMergeFindMinimalMerge(const Graph &adjMatrix, const std::vector<int> &visit_node, const std::vector<int> &next_vec, const std::vector<int> &prev_vec, int &visited_start, int &visited_end, int &unvisited_start, int &unvisited_end, std::vector<MergePoint> & minimal_merge_point);

std::vector<int> CycleMergeWithAdjMatrix(InputParam *input);
std::vector<int> CycleMergeWithAdjTable(InputParam *input);

std::vector<int> CycleMergeMain(const Graph &adjMatrix, std::vector<int> &prev_vec);



#endif // HUNGARIAN_H
