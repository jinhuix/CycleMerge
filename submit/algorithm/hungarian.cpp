// James Payor - December 2017
// MIT Licensed

#include <algorithm>
#include <vector>
#include <deque>
#include <cassert>
#include <limits>
#include <memory>
#include <iostream>

#include "hungarian_c.h"
#include "hungarian.h"

// Macros!
//
// fo(i, n):       foreach i in [0, n)
// range(i, a, b): foreach i in [a, b)
//
// (Note: end value is cached, so fo(i, function()) will only have function called once.)

#define fo(i, n) for (int i = 0, _n = (n); i < _n; ++i)
#define range(i, a, b) for (int i = (a), _n = (b); i < _n; ++i)


const std::vector<int> hungarianMinimumWeightPerfectMatching(const int n, const std::shared_ptr<std::vector<LeftEdge>[]> & leftEdges)
{
    // endregion Edge list initialization

    // These hold "potentials" for nodes on the left and nodes on the right, which reduce the costs of attached edges.
    // We maintain that every reduced cost, cost[i][j] - leftPotential[i] - leftPotential[j], is greater than zero.
    int leftPotential[n], rightPotential[n];

    // region Node potential initialization

    // Here, we seek good initial values for the node potentials.
    // Note: We're guaranteed by the above code that at every node on the left and right has at least one edge.

    // First, we raise the potentials on the left as high as we can for each node.
    // This guarantees each node on the left has at least one "tight" edge.

    fo(i, n)
    {
        const std::vector<LeftEdge> &edges = leftEdges[i];
        int smallestEdgeCost = edges[0].cost;
        range(edgeIndex, 1, edges.size())
        {
            if (edges[edgeIndex].cost < smallestEdgeCost)
            {
                smallestEdgeCost = edges[edgeIndex].cost;
            }
        }

        // Set node potential to the smallest incident edge cost.
        // This is as high as we can take it without creating an edge with zero reduced cost.
        leftPotential[i] = smallestEdgeCost;
    }

    // Second, we raise the potentials on the right as high as we can for each node.
    // We do the same as with the left, but this time take into account that costs are reduced
    // by the left potentials.
    // This guarantees that each node on the right has at least one "tight" edge.

    std::fill_n(rightPotential, n, oo);

    fo(i, n){
        fo(edgeIndex, leftEdges[i].size()){
            const LeftEdge &edge = leftEdges[i][edgeIndex];
            int reducedCost = edge.cost - leftPotential[i];
            if (rightPotential[edge.right] > reducedCost)
            {
                rightPotential[edge.right] = reducedCost;
            }
        }
    }

    // endregion Node potential initialization

    // Tracks how many edges for each left node are "tight".
    // Following initialization, we maintain the invariant that these are at the start of the node's edge list.
    int leftTightEdgesCount[n];
    std::fill_n(leftTightEdgesCount, n, 0);

    // region Tight edge initialization

    // Here we find all tight edges, defined as edges that have zero reduced cost.
    // We will be interested in the subgraph induced by the tight edges, so we partition the edge lists for
    // each left node accordingly, moving the tight edges to the start.

    fo(i, n)
    {
        std::vector<LeftEdge> &edges = leftEdges[i];
        int tightEdgeCount = 0;
        fo(edgeIndex, edges.size())
        {
            const LeftEdge &edge = edges[edgeIndex];
            int reducedCost = edge.cost - leftPotential[i] - rightPotential[edge.right];
            if (reducedCost == 0)
            {
                if (edgeIndex != tightEdgeCount)
                {
                    std::swap(edges[tightEdgeCount], edges[edgeIndex]);
                }
                ++tightEdgeCount;
            }
        }
        leftTightEdgesCount[i] = tightEdgeCount;
    }

    // endregion Tight edge initialization

    // Now we're ready to begin the inner loop.

    // We maintain an (initially empty) partial matching, in the subgraph of tight edges.
    int currentMatchingCardinality = 0;
    int leftMatchedTo[n], rightMatchedTo[n];
    std::fill_n(leftMatchedTo, n, UNMATCHED);
    std::fill_n(rightMatchedTo, n, UNMATCHED);

    // region Initial matching (speedup?)

    // Because we can, let's make all the trivial matches we can.
    fo(i, n)
    {
        const std::vector<LeftEdge> &edges = leftEdges[i];
        fo(edgeIndex, leftTightEdgesCount[i])
        {
            int j = edges[edgeIndex].right;
            if (rightMatchedTo[j] == UNMATCHED)
            {
                ++currentMatchingCardinality;
                rightMatchedTo[j] = i;
                leftMatchedTo[i] = j;
                break;
            }
        }
    }

    if (currentMatchingCardinality == n)
    {
        // Well, that's embarassing. We're already done!
        return std::vector<int>(leftMatchedTo, leftMatchedTo + n);
    }

    // endregion Initial matching (speedup?)

    // While an augmenting path exists, we add it to the matching.
    // When an augmenting path doesn't exist, we update the potentials so that an edge between the area
    // we can reach and the unreachable nodes on the right becomes tight, giving us another edge to explore.
    //
    // We proceed in this fashion until we can't find more augmenting paths or add edges.
    // At that point, we either have a min-weight perfect matching, or no matching exists.

    // region Inner loop state variables

    // One point of confusion is that we're going to cache the edges between the area we've explored
    // that are "almost tight", or rather are the closest to being tight.
    // This is necessary to achieve our O(N^3) runtime.
    //
    // rightMinimumSlack[j] gives the smallest amount of "slack" for an unreached node j on the right,
    // considering the edges between j and some node on the left in our explored area.
    //
    // rightMinimumSlackLeftNode[j] gives the node i with the corresponding edge.
    // rightMinimumSlackEdgeIndex[j] gives the edge index for node i.

    int rightMinimumSlack[n], rightMinimumSlackLeftNode[n], rightMinimumSlackEdgeIndex[n];

    std::deque<int> leftNodeQueue;
    bool leftSeen[n];
    int rightBacktrack[n];

    // Note: the above are all initialized at the start of the loop.

    // endregion Inner loop state variables

    while (currentMatchingCardinality < n)
    {

        // region Loop state initialization

        // Clear out slack caches.
        // Note: We need to clear the nodes so that we can notice when there aren't any edges available.
        std::fill_n(rightMinimumSlack, n, oo);
        std::fill_n(rightMinimumSlackLeftNode, n, UNMATCHED);

        // Clear the queue.
        leftNodeQueue.clear();

        // Mark everything "unseen".
        std::fill_n(leftSeen, n, false);
        std::fill_n(rightBacktrack, n, UNMATCHED);

        // endregion Loop state initialization

        int startingLeftNode = UNMATCHED;

        // region Find unmatched starting node

        // Find an unmatched left node to search outward from.
        // By heuristic, we pick the node with fewest tight edges, giving the BFS an easier time.
        // (The asymptotics don't care about this, but maybe it helps. Eh.)
        {
            int minimumTightEdges = oo;
            fo(i, n)
            {
                if (leftMatchedTo[i] == UNMATCHED && leftTightEdgesCount[i] < minimumTightEdges)
                {
                    minimumTightEdges = leftTightEdgesCount[i];
                    startingLeftNode = i;
                }
            }
        }

        // endregion Find unmatched starting node

        assert(startingLeftNode != UNMATCHED);

        assert(leftNodeQueue.empty());
        leftNodeQueue.push_back(startingLeftNode);
        leftSeen[startingLeftNode] = true;

        int endingRightNode = UNMATCHED;
        while (endingRightNode == UNMATCHED)
        {

            // region BFS until match found or no edges to follow

            while (endingRightNode == UNMATCHED && !leftNodeQueue.empty())
            {
                // Implementation note: this could just as easily be a DFS, but a BFS probably
                // has less edge flipping (by my guess), so we're using a BFS.

                const int i = leftNodeQueue.front();
                leftNodeQueue.pop_front();

                std::vector<LeftEdge> &edges = leftEdges[i];
                // Note: Some of the edges might not be tight anymore, hence the awful loop.
                for (int edgeIndex = 0; edgeIndex < leftTightEdgesCount[i]; ++edgeIndex)
                {
                    const LeftEdge &edge = edges[edgeIndex];
                    const int j = edge.right;

                    assert(edge.cost - leftPotential[i] - rightPotential[j] >= 0);
                    if (edge.cost > leftPotential[i] + rightPotential[j])
                    {
                        // This edge is loose now.
                        --leftTightEdgesCount[i];
                        std::swap(edges[edgeIndex], edges[leftTightEdgesCount[i]]);
                        --edgeIndex;
                        continue;
                    }

                    if (rightBacktrack[j] != UNMATCHED)
                    {
                        continue;
                    }

                    rightBacktrack[j] = i;
                    int matchedTo = rightMatchedTo[j];
                    if (matchedTo == UNMATCHED)
                    {
                        // Match found. This will terminate the loop.
                        endingRightNode = j;
                    }
                    else if (!leftSeen[matchedTo])
                    {
                        // No match found, but a new left node is reachable. Track how we got here and extend BFS queue.
                        leftSeen[matchedTo] = true;
                        leftNodeQueue.push_back(matchedTo);
                    }
                }

                // region Update cached slack values

                // The remaining edges may be to nodes that are unreachable.
                // We accordingly update the minimum slackness for nodes on the right.

                if (endingRightNode == UNMATCHED)
                {
                    const int potential = leftPotential[i];
                    range(edgeIndex, leftTightEdgesCount[i], edges.size())
                    {
                        const LeftEdge &edge = edges[edgeIndex];
                        int j = edge.right;

                        if (rightMatchedTo[j] == UNMATCHED || !leftSeen[rightMatchedTo[j]])
                        {
                            // This edge is to a node on the right that we haven't reached yet.

                            int reducedCost = edge.cost - potential - rightPotential[j];
                            assert(reducedCost >= 0);

                            if (reducedCost < rightMinimumSlack[j])
                            {
                                // There should be a better way to do this backtracking...
                                // One array instead of 3. But I can't think of something else. So it goes.
                                rightMinimumSlack[j] = reducedCost;
                                rightMinimumSlackLeftNode[j] = i;
                                rightMinimumSlackEdgeIndex[j] = edgeIndex;
                            }
                        }
                    }
                }

                // endregion Update cached slack values
            }

            // endregion BFS until match found or no edges to follow

            // region Update node potentials to add edges, if no match found

            if (endingRightNode == UNMATCHED)
            {
                // Out of nodes. Time to update some potentials.
                int minimumSlackRightNode = UNMATCHED;

                // region Find minimum slack node, or abort if none exists

                int minimumSlack = oo;
                fo(j, n)
                {
                    if (rightMatchedTo[j] == UNMATCHED || !leftSeen[rightMatchedTo[j]])
                    {
                        // This isn't a node reached by our BFS. Update minimum slack.
                        if (rightMinimumSlack[j] < minimumSlack)
                        {
                            minimumSlack = rightMinimumSlack[j];
                            minimumSlackRightNode = j;
                        }
                    }
                }

                if (minimumSlackRightNode == UNMATCHED || rightMinimumSlackLeftNode[minimumSlackRightNode] == UNMATCHED)
                {
                    // The caches are all empty. There was no option available.
                    // This means that the node the BFS started at, which is an unmatched left node, cannot reach the
                    // right - i.e. it will be impossible to find a perfect matching.

                    return std::vector<int>();
                }

                // endregion Find minimum slack node, or abort if none exists

                assert(minimumSlackRightNode != UNMATCHED);

                // Adjust potentials on left and right.
                fo(i, n)
                {
                    if (leftSeen[i])
                    {
                        leftPotential[i] += minimumSlack;
                        if (leftMatchedTo[i] != UNMATCHED)
                        {
                            rightPotential[leftMatchedTo[i]] -= minimumSlack;
                        }
                    }
                }

                // Downward-adjust slackness caches.
                fo(j, n)
                {
                    if (rightMatchedTo[j] == UNMATCHED || !leftSeen[rightMatchedTo[j]])
                    {
                        rightMinimumSlack[j] -= minimumSlack;

                        // If the slack hit zero, then we just found ourselves a new tight edge.
                        if (rightMinimumSlack[j] == 0)
                        {
                            const int i = rightMinimumSlackLeftNode[j];
                            const int edgeIndex = rightMinimumSlackEdgeIndex[j];

                            // region Update leftEdges[i] and leftTightEdgesCount[i]

                            // Move it in the relevant edge list.
                            if (edgeIndex != leftTightEdgesCount[i])
                            {
                                std::vector<LeftEdge> &edges = leftEdges[i];
                                std::swap(edges[edgeIndex], edges[leftTightEdgesCount[i]]);
                            }
                            ++leftTightEdgesCount[i];

                            // endregion Update leftEdges[i] and leftTightEdgesCount[i]

                            // If we haven't already encountered a match, we follow the edge and update the BFS queue.
                            // It's possible this edge leads to a match. If so, we'll carry on updating the tight edges,
                            // but won't follow them.
                            if (endingRightNode == UNMATCHED)
                            {
                                // We're contemplating the consequences of following (i, j), as we do in the BFS above.
                                rightBacktrack[j] = i;
                                int matchedTo = rightMatchedTo[j];
                                if (matchedTo == UNMATCHED)
                                {
                                    // Match found!
                                    endingRightNode = j;
                                }
                                else if (!leftSeen[matchedTo])
                                {
                                    // No match, but new left node found. Extend BFS queue.
                                    leftSeen[matchedTo] = true;
                                    leftNodeQueue.push_back(matchedTo);
                                }
                            }
                        }
                    }
                }
            }

            // endregion Update node potentials to add edges, if no match found
        }

        // At this point, we've found an augmenting path between startingLeftNode and endingRightNode.
        // We'll just use the backtracking info to update our match information.

        ++currentMatchingCardinality;

        // region Backtrack and flip augmenting path

        {
            int currentRightNode = endingRightNode;
            while (currentRightNode != UNMATCHED)
            {
                const int currentLeftNode = rightBacktrack[currentRightNode];
                const int nextRightNode = leftMatchedTo[currentLeftNode];

                rightMatchedTo[currentRightNode] = currentLeftNode;
                leftMatchedTo[currentLeftNode] = currentRightNode;

                currentRightNode = nextRightNode;
            }
        }

        // endregion Backtrack and flip augmenting path
    }

    // Oh look, we're done.
    return std::vector<int>(leftMatchedTo, leftMatchedTo + n);
}

const std::vector<int> hungarianMinimumWeightPerfectMatchingDenseGraph(const AdjMatrix &adjMatrix)
{
    printf("%s\n", __func__);
    // Edge lists for each left node.
    int n = adjMatrix.n;

    // These hold "potentials" for nodes on the left and nodes on the right, which reduce the costs of attached edges.
    // We maintain that every reduced cost, cost[i][j] - leftPotential[i] - leftPotential[j], is greater than zero.
    int leftPotential[n], rightPotential[n];

    // region Node potential initialization

    // Here, we seek good initial values for the node potentials.
    // Note: We're guaranteed by the above code that at every node on the left and right has at least one edge.

    // First, we raise the potentials on the left as high as we can for each node.
    // This guarantees each node on the left has at least one "tight" edge.

    fo(i, n)
    {
        // 使用邻接矩阵可以减少内存开销
        int smallestEdgeCost = oo;
        fo(j, n)
        {
            if (i == j)
                continue; // 可能不需要
            if (adjMatrix.getEdgeCost(i, j) < smallestEdgeCost)
            {
                smallestEdgeCost = adjMatrix.getEdgeCost(i, j);
            }
        }

        // Set node potential to the smallest incident edge cost.
        // This is as high as we can take it without creating an edge with zero reduced cost.
        leftPotential[i] = smallestEdgeCost;
    }

    // Second, we raise the potentials on the right as high as we can for each node.
    // We do the same as with the left, but this time take into account that costs are reduced
    // by the left potentials.
    // This guarantees that each node on the right has at least one "tight" edge.

    std::fill_n(rightPotential, n, oo);

    // 找到每列中，使得至少有一个0的最小reduce，可以通过记录哪些列已经有0了来优化，通过邻接矩阵优化
    fo(i, n)
    {
        fo(j, n)
        {
            if (i == j)
                continue;
            int reducedCost = adjMatrix.getEdgeCost(i, j) - leftPotential[i];
            if (rightPotential[j] > reducedCost)
            {
                rightPotential[j] = reducedCost;
            }
        }
    }

    // 此时保证每列至少有1个0
    // endregion Node potential initialization

    // Tracks how many edges for each left node are "tight".
    // Following initialization, we maintain the invariant that these are at the start of the node's edge list.
    int leftTightEdgesCount[n];
    std::fill_n(leftTightEdgesCount, n, 0);

    // Now we're ready to begin the inner loop.

    // We maintain an (initially empty) partial matching, in the subgraph of tight edges.
    int currentMatchingCardinality = 0;
    int leftMatchedTo[n], rightMatchedTo[n];
    std::fill_n(leftMatchedTo, n, UNMATCHED);
    std::fill_n(rightMatchedTo, n, UNMATCHED);

    // region Initial matching (speedup?)
    
    // Because we can, let's make all the trivial matches we can.
    fo(i, n)
    {
        fo(j, n)
        {
            if (i == j)
                continue;
            int reducedCost = adjMatrix.getEdgeCost(i, j) - leftPotential[i] - rightPotential[j];
            if (reducedCost == 0 && rightMatchedTo[j] == UNMATCHED)
            { // 判断能否加入初始matching
                ++currentMatchingCardinality;
                rightMatchedTo[j] = i;
                leftMatchedTo[i] = j;
                break;
            }
        }
    }
    // gxj: 一开始就可以直接确定的边
    printf("currentMatchingCardinality: %d\n", currentMatchingCardinality);

    if (currentMatchingCardinality == n)
    {
        // Well, that's embarassing. We're already done!
        return std::vector<int>(leftMatchedTo, leftMatchedTo + n);
    }

    // endregion Initial matching (speedup?)

    // While an augmenting path exists, we add it to the matching.
    // When an augmenting path doesn't exist, we update the potentials so that an edge between the area
    // we can reach and the unreachable nodes on the right becomes tight, giving us another edge to explore.
    //
    // We proceed in this fashion until we can't find more augmenting paths or add edges.
    // At that point, we either have a min-weight perfect matching, or no matching exists.

    // region Inner loop state variables

    // One point of confusion is that we're going to cache the edges between the area we've explored
    // that are "almost tight", or rather are the closest to being tight.
    // This is necessary to achieve our O(N^3) runtime.
    //
    // rightMinimumSlack[j] gives the smallest amount of "slack" for an unreached node j on the right,
    // considering the edges between j and some node on the left in our explored area.
    //
    // rightMinimumSlackLeftNode[j] gives the node i with the corresponding edge.
    // rightMinimumSlackEdgeIndex[j] gives the edge index for node i.

    int rightMinimumSlack[n], rightMinimumSlackLeftNode[n];

    std::deque<int> leftNodeQueue;
    bool leftSeen[n];
    int rightBacktrack[n];

    // Note: the above are all initialized at the start of the loop.

    // endregion Inner loop state variables

    while (currentMatchingCardinality < n)
    {

        // region Loop state initialization

        // Clear out slack caches.
        // Note: We need to clear the nodes so that we can notice when there aren't any edges available.
        std::fill_n(rightMinimumSlack, n, oo);
        std::fill_n(rightMinimumSlackLeftNode, n, UNMATCHED);

        // Clear the queue.
        leftNodeQueue.clear();

        // Mark everything "unseen".
        std::fill_n(leftSeen, n, false);
        std::fill_n(rightBacktrack, n, UNMATCHED);

        // endregion Loop state initialization

        int startingLeftNode = UNMATCHED;

        // region Find unmatched starting node

        // Find an unmatched left node to search outward from.
        // By heuristic, we pick the node with fewest tight edges, giving the BFS an easier time.
        // (The asymptotics don't care about this, but maybe it helps. Eh.)
        {
            int minimumTightEdges = oo;
            fo(i, n)
            {
                if (leftMatchedTo[i] == UNMATCHED)
                {
                    startingLeftNode = i;
                    break;
                }
            }
        }

        // endregion Find unmatched starting node

        assert(startingLeftNode != UNMATCHED);

        assert(leftNodeQueue.empty());
        leftNodeQueue.push_back(startingLeftNode);
        leftSeen[startingLeftNode] = true;

        int endingRightNode = UNMATCHED;
        while (endingRightNode == UNMATCHED)
        {

            // region BFS until match found or no edges to follow

            while (endingRightNode == UNMATCHED && !leftNodeQueue.empty())
            {
                // Implementation note: this could just as easily be a DFS, but a BFS probably
                // has less edge flipping (by my guess), so we're using a BFS.

                const int i = leftNodeQueue.front();
                leftNodeQueue.pop_front();

                // Note: Some of the edges might not be tight anymore, hence the awful loop.
                fo(j, n)
                {
                    if (i == j)
                        continue;
                    const int cost = adjMatrix.getEdgeCost(i, j);

                    assert(cost - leftPotential[i] - rightPotential[j] >= 0);
                    if (cost > leftPotential[i] + rightPotential[j])
                    {
                        // This edge is loose now.
                        // 不是tight的边，跳过
                        // --leftTightEdgesCount[i];
                        continue;
                    }

                    if (rightBacktrack[j] != UNMATCHED)
                    {
                        continue;
                    }

                    rightBacktrack[j] = i;
                    int matchedTo = rightMatchedTo[j];
                    if (matchedTo == UNMATCHED)
                    {
                        // Match found. This will terminate the loop.
                        endingRightNode = j;
                    }
                    else if (!leftSeen[matchedTo])
                    {
                        // No match found, but a new left node is reachable. Track how we got here and extend BFS queue.
                        leftSeen[matchedTo] = true;
                        leftNodeQueue.push_back(matchedTo);
                    }
                }

                // region Update cached slack values

                // The remaining edges may be to nodes that are unreachable.
                // We accordingly update the minimum slackness for nodes on the right.

                if (endingRightNode == UNMATCHED)
                {
                    const int potential = leftPotential[i];
                    fo(j, n)
                    {
                        if (i == j)
                            continue;
                        const int cost = adjMatrix.getEdgeCost(i, j);

                        if (rightMatchedTo[j] == UNMATCHED || !leftSeen[rightMatchedTo[j]])
                        {
                            // This edge is to a node on the right that we haven't reached yet.

                            int reducedCost = cost - potential - rightPotential[j];
                            assert(reducedCost >= 0);

                            if (reducedCost < rightMinimumSlack[j])
                            {
                                // There should be a better way to do this backtracking...
                                // One array instead of 3. But I can't think of something else. So it goes.
                                rightMinimumSlack[j] = reducedCost;
                                rightMinimumSlackLeftNode[j] = i;
                            }
                        }
                    }
                }

                // endregion Update cached slack values
            }

            // endregion BFS until match found or no edges to follow

            // region Update node potentials to add edges, if no match found

            if (endingRightNode == UNMATCHED)
            {
                // Out of nodes. Time to update some potentials.
                int minimumSlackRightNode = UNMATCHED;

                // region Find minimum slack node, or abort if none exists

                int minimumSlack = oo;
                fo(j, n)
                {
                    if (rightMatchedTo[j] == UNMATCHED || !leftSeen[rightMatchedTo[j]])
                    {
                        // This isn't a node reached by our BFS. Update minimum slack.
                        if (rightMinimumSlack[j] < minimumSlack)
                        {
                            minimumSlack = rightMinimumSlack[j];
                            minimumSlackRightNode = j;
                        }
                    }
                }

                if (minimumSlackRightNode == UNMATCHED || rightMinimumSlackLeftNode[minimumSlackRightNode] == UNMATCHED)
                {
                    // The caches are all empty. There was no option available.
                    // This means that the node the BFS started at, which is an unmatched left node, cannot reach the
                    // right - i.e. it will be impossible to find a perfect matching.

                    return std::vector<int>();
                }

                // endregion Find minimum slack node, or abort if none exists

                assert(minimumSlackRightNode != UNMATCHED);

                // Adjust potentials on left and right.
                fo(i, n)
                {
                    if (leftSeen[i])
                    {
                        leftPotential[i] += minimumSlack;
                        if (leftMatchedTo[i] != UNMATCHED)
                        {
                            rightPotential[leftMatchedTo[i]] -= minimumSlack;
                        }
                    }
                }

                // Downward-adjust slackness caches.
                fo(j, n)
                {
                    if (rightMatchedTo[j] == UNMATCHED || !leftSeen[rightMatchedTo[j]])
                    {
                        rightMinimumSlack[j] -= minimumSlack;

                        // If the slack hit zero, then we just found ourselves a new tight edge.
                        if (rightMinimumSlack[j] == 0)
                        {
                            const int i = rightMinimumSlackLeftNode[j];

                            // region Update leftEdges[i] and leftTightEdgesCount[i]

                            // ++leftTightEdgesCount[i];

                            // endregion Update leftEdges[i] and leftTightEdgesCount[i]

                            // If we haven't already encountered a match, we follow the edge and update the BFS queue.
                            // It's possible this edge leads to a match. If so, we'll carry on updating the tight edges,
                            // but won't follow them.
                            if (endingRightNode == UNMATCHED)
                            {
                                // We're contemplating the consequences of following (i, j), as we do in the BFS above.
                                rightBacktrack[j] = i;
                                int matchedTo = rightMatchedTo[j];
                                if (matchedTo == UNMATCHED)
                                {
                                    // Match found!
                                    endingRightNode = j;
                                }
                                else if (!leftSeen[matchedTo])
                                {
                                    // No match, but new left node found. Extend BFS queue.
                                    leftSeen[matchedTo] = true;
                                    leftNodeQueue.push_back(matchedTo);
                                }
                            }
                        }
                    }
                }
            }

            // endregion Update node potentials to add edges, if no match found
        }

        // At this point, we've found an augmenting path between startingLeftNode and endingRightNode.
        // We'll just use the backtracking info to update our match information.

        ++currentMatchingCardinality;

        // region Backtrack and flip augmenting path

        {
            int currentRightNode = endingRightNode;
            while (currentRightNode != UNMATCHED)
            {
                const int currentLeftNode = rightBacktrack[currentRightNode];
                const int nextRightNode = leftMatchedTo[currentLeftNode];

                rightMatchedTo[currentRightNode] = currentLeftNode;
                leftMatchedTo[currentLeftNode] = currentRightNode;

                currentRightNode = nextRightNode;
            }
        }

        // endregion Backtrack and flip augmenting path
    }

    // Oh look, we're done.
    return std::vector<int>(leftMatchedTo, leftMatchedTo + n);
}

int getMiddleValue(int a, int b, int c)
{
    if ((a >= b && a <= c) || (a >= c && a <= b))
    {
        return a;
    }
    else if ((b >= a && b <= c) || (b >= c && b <= a))
    {
        return b;
    }
    else
    {
        return c;
    }
}

int quickSelect(int arr[], int low, int high, int k)
{
    if (low <= high)
    {
        int pivot_a = arr[low];
        int pivot_b = arr[(low + high) / 2];
        int pivot_c = arr[high];
        int pivot = getMiddleValue(pivot_a, pivot_b, pivot_c);
        int i = low - 1;

        for (int j = low; j <= high - 1; j++)
        {
            if (arr[j] < pivot)
            {
                i++;
                int temp = arr[i];
                arr[i] = arr[j];
                arr[j] = temp;
            }
        }

        int temp = arr[i + 1];
        arr[i + 1] = arr[high];
        arr[high] = temp;

        int pivotIndex = i + 1;

        if (pivotIndex == k - 1)
        {
            return arr[pivotIndex];
        }
        else if (pivotIndex > k - 1)
        {
            return quickSelect(arr, low, pivotIndex - 1, k);
        }
        else
        {
            return quickSelect(arr, pivotIndex + 1, high, k);
        }
    }

    return -1; // 如果k超出数组范围，返回-1或其他适当的值
}

std::vector<int> CycleMergeMain(const Graph & graph, std::vector<int> &next_vec)
{
    int path_length = 0;
    int cycle_cnt = 1;
    int last_end = -1;
    std::vector<int> visit_node(next_vec.size(), 0);
    std::vector<int> prev_vec(next_vec.size(), -1);
    int matched_weight = 0;
    for (int i = 0; i < next_vec.size(); i++)
    {
        prev_vec[next_vec[i]] = i;
        matched_weight += graph.getEdgeCost(i, next_vec[i]);
        // printf(" %d -> %d: %d\n", i, next_vec[i], adjMatrix.getEdgeCost(i, next_vec[i]));
    }

    printf("matched_weight = %d\n", matched_weight);

    int now_node = 0;

    int visit_cnt = 0;
    while (visit_cnt != graph.n)
    {
        visit_node[now_node] = 1;
        visit_cnt++;
        if (visit_cnt == graph.n)
        {
            break;
        }

        int next = next_vec[now_node];
        if (visit_node[next])
        {
            cycle_cnt++;
            int visited_start, visited_end;
            int unvisited_start, unvisited_end;
            localCycleMergeFindMinimalMerge(graph, visit_node, next_vec, prev_vec, visited_start, visited_end, unvisited_start, unvisited_end);

            next_vec[visited_start] = unvisited_start;
            next_vec[unvisited_end] = visited_end;
            prev_vec[unvisited_start] = visited_start;
            prev_vec[visited_end] = unvisited_end;

            path_length += graph.getEdgeCost(visited_start, unvisited_start);
            path_length += graph.getEdgeCost(unvisited_end, visited_end);
            path_length -= graph.getEdgeCost(visited_start, visited_end);
            now_node = unvisited_start;
        }
        else
        {
            path_length += graph.getEdgeCost(now_node, next);
            now_node = next;
        }
    }
    std::vector<int> walk_order;
    int current_node = next_vec[0];
    while(current_node != 0){
        walk_order.push_back(current_node);
        current_node = next_vec[current_node];
    }
    printf("path length = %d, cycle_cnt = %d\n", path_length, cycle_cnt);

    return walk_order;
}


// std::vector<int> CycleMergeMain(const AdjMatrix &adjMatrix, std::vector<int> &next_vec)
// {
//     int path_length = 0;
//     int cycle_cnt = 1;
//     int last_end = -1;
//     std::vector<int> visit_node(adjMatrix.n, 0);
//     std::vector<int> prev_vec(next_vec.size(), -1);
//     int matched_weight = 0;
//     for (int i = 0; i < next_vec.size(); i++)
//     {
//         prev_vec[next_vec[i]] = i;
//         matched_weight += adjMatrix.getEdgeCost(i, next_vec[i]);
//         // printf(" %d -> %d: %d\n", i, next_vec[i], adjMatrix.getEdgeCost(i, next_vec[i]));
//     }

//     printf("matched_weight = %d\n", matched_weight);

//     int now_node = 0;

//     int visit_cnt = 0;
//     while (visit_cnt != adjMatrix.n)
//     {
//         visit_node[now_node] = 1;
//         visit_cnt++;
//         if (visit_cnt == adjMatrix.n)
//         {
//             break;
//         }

//         int next = next_vec[now_node];
//         if (visit_node[next])
//         {
//             cycle_cnt++;
//             int visited_start, visited_end;
//             int unvisited_start, unvisited_end;
//             localCycleMergeFindMinimalMerge(adjMatrix, visit_node, next_vec, prev_vec, visited_start, visited_end, unvisited_start, unvisited_end);

//             next_vec[visited_start] = unvisited_start;
//             next_vec[unvisited_end] = visited_end;
//             prev_vec[unvisited_start] = visited_start;
//             prev_vec[visited_end] = unvisited_end;

//             path_length += adjMatrix.getEdgeCost(visited_start, unvisited_start);
//             path_length += adjMatrix.getEdgeCost(unvisited_end, visited_end);
//             path_length -= adjMatrix.getEdgeCost(visited_start, visited_end);
//             now_node = unvisited_start;
//         }
//         else
//         {
//             path_length += adjMatrix.getEdgeCost(now_node, next);
//             now_node = next;
//         }
//     }
//     std::vector<int> walk_order;
//     int current_node = next_vec[0];
//     while(current_node != 0){
//         walk_order.push_back(current_node);
//         current_node = next_vec[current_node];
//     }
//     printf("path length = %d, cycle_cnt = %d\n", path_length, cycle_cnt);

//     return walk_order;
// }



std::vector<int> CycleMergeWithAdjMatrix(InputParam *input)
{
    printf("%s\n", __func__);
    AdjMatrix adjMatrix(input);
    std::vector<int> matching = hungarianMinimumWeightPerfectMatchingDenseGraph(adjMatrix);

    std::vector<int> &next = matching;

    printf("matching: %d\n", matching.size());

    std::vector<int> walk_order = CycleMergeMain(adjMatrix, next);

    return walk_order;
}

std::vector<int> CycleMergeWithAdjTable(InputParam *input, int max_edge_per_node){
    printf("%s\n", __func__);
    int n = input->ioVec.len + 1;

    AdjTable adjtable(input, max_edge_per_node);
    // for(int i = 0; i < n; i++){
    //     for(int edgeIdx = 0; edgeIdx < adjtable.leftEdges[i].size(); edgeIdx++){
    //         printf("%d %d %d\n", i, adjtable.leftEdges[i][edgeIdx].right, adjtable.leftEdges[i][edgeIdx].cost);
    //     }
    // }

    std::vector<int> matching = hungarianMinimumWeightPerfectMatching(n, adjtable.leftEdges);

    std::vector<int> &next = matching;

    adjtable.SortAllEdges();

    std::vector<int> walk_order = CycleMergeMain(adjtable, next);

    return walk_order;
}

void localCycleMergeFindMinimalMerge(const Graph &graph, const std::vector<int> &visit_node, const std::vector<int> &next_vec, const std::vector<int> &prev_vec, int &visited_start, int &visited_end, int &unvisited_start, int &unvisited_end)
{
    int min_cost = oo;
    int better_cnt = 0;
    for (int i = 0; i < graph.n; i++)
    {
        if (visit_node[i])
        {
            for (int j = 0; j < graph.n; j++)
            {
                if (!visit_node[j])
                {
                    int this_cost = 0;
                    this_cost += graph.getEdgeCost(i, j);
                    this_cost += graph.getEdgeCost(prev_vec[j], next_vec[i]);
                    this_cost -= graph.getEdgeCost(i, next_vec[i]);
                    this_cost -= graph.getEdgeCost(prev_vec[j], j);
                    if (visited_start == -1 || this_cost < min_cost)
                    {
                        min_cost = this_cost;
                        visited_start = i;
                        visited_end = next_vec[i];
                        unvisited_start = j;
                        unvisited_end = prev_vec[j];
                    }
                }
            }
        }
    }
}



void CycleMerge(InputParam *input, int * walk_order)
{
    int n = input->ioVec.len + 1;
    int max_edge_num = 120 * 10000; // 最多150w条边，大约需要12MB
    int edge_per_node = max_edge_num / n;

    

    if(edge_per_node > n){
        std::vector<int> walk_order_vec = CycleMergeWithAdjMatrix(input);
        printf("%d\n", walk_order_vec.size());
        assert(walk_order_vec.size() == n-1);

        for(int i = 0; i < walk_order_vec.size(); i++){
            walk_order[i] = walk_order_vec[i];
        }
    }
    else{
        std::vector<int> walk_order_vec = CycleMergeWithAdjTable(input, edge_per_node);
        assert(walk_order_vec.size() == n-1);
        for(int i = 0; i < walk_order_vec.size(); i++){
            walk_order[i] = walk_order_vec[i];
        }
    }
}

