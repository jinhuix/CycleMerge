#ifndef HUNGARIAN_H
#define HUNGARIAN_H

#include <vector>
#include <sys/time.h>

struct WeightedBipartiteEdge {
    int left;
    int right;
    int cost;

    WeightedBipartiteEdge() : left(), right(), cost() {}
    WeightedBipartiteEdge(int left, int right, int cost) : left(left), right(right), cost(cost) {}
};

struct AdjMatrix{
    int n;
    const int * cost;

    AdjMatrix(int n, const int * cost): n(n), cost(cost){
        ;
    }

    int getCost(int i, int j) const{
        return -cost[(i+1) * (n + 1) + j + 1];
    }
};

// Given the number of nodes on each side of the bipartite graph and a list of edges, returns a minimum-weight perfect matching.
// If a matching is found, returns a length-n vector, giving the nodes on the right that the left nodes are matched to.
// If no matching exists, returns an empty vector.
// (Note: Edges with endpoints out of the range [0, n) are ignored.)
const std::vector<int> hungarianMinimumWeightPerfectMatching(int n, const std::vector<WeightedBipartiteEdge> allEdges);

const std::vector<int> hungarianMinimumWeightPerfectMatchingDenseGraph(const AdjMatrix & adjMatrix);

#endif // HUNGARIAN_H
