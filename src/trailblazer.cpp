/*
* CS 106X Trailblazer
* Code to perform graph path searches
*
* Author: Elyas Esmaeili
*/

#include "trailblazer.h"
#include "pqueue.h"
#include "linkedlist.h"
#include <algorithm>  
#include <queue>

bool depthFirstSearchR(BasicGraph& graph, Vertex* current, Vertex* end);
Vector<Vertex*> shortestPathWeightConsidered(BasicGraph& graph, Vertex* start, Vertex* end, bool aStar=false);

Vector<Vertex*> createPath(Vertex* end);  
bool isCompeletelyVisited(BasicGraph& graph);

using namespace std;

Vector<Vertex*> depthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    Vector<Vertex*> path;

    depthFirstSearchR(graph, start, end);

    path = createPath(end);
	return path;
}

bool depthFirstSearchR(BasicGraph& graph, Vertex* current, Vertex* end)
{
    current->visited = true;
    current->setColor(GREEN);

	if (current == end) {
		return true;
	}

    for (auto& adjacentNode : graph.getNeighbors(current)) {
        if (!adjacentNode->visited) {
            adjacentNode->previous = current;
            if (depthFirstSearchR(graph, adjacentNode, end)) {
                return true;
			}
		}
	}
	
    current->setColor(GRAY);
	return false;
}

Vector<Vertex*> breadthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();

    Vector<Vertex*> path;
    queue<Vertex*> toExplore;
    toExplore.push(start);

    while (!toExplore.empty()) {
        Vertex* current = toExplore.front();
        toExplore.pop();
        current->setColor(GREEN);
        current->visited = true;

        if (current == end) {
			path = createPath(end);
			return path;
        }

        for (auto& adjacentNode : graph.getNeighbors(current)) {
            if (!adjacentNode->visited) {
                adjacentNode->previous = current;
                toExplore.push(adjacentNode);
                adjacentNode->setColor(YELLOW);
            }
        }
    }

    return path;
}

Vector<Vertex*> dijkstrasAlgorithm(BasicGraph& graph, Vertex* start, Vertex* end) {
	return shortestPathWeightConsidered(graph, start, end);
}

Vector<Vertex*> aStar(BasicGraph& graph, Vertex* start, Vertex* end) {
	return shortestPathWeightConsidered(graph, start, end, true);
}

Vector<Vertex*> shortestPathWeightConsidered(BasicGraph& graph, Vertex* start, Vertex* end, bool aStar)
{
	graph.resetData();
	Vector<Vertex*> path;
	PriorityQueue<Vertex*> pqueue;
	int heuristicValue = aStar ? heuristicFunction(start, end) : 0;

	start->setColor(YELLOW);
	pqueue.enqueue(start, heuristicFunction(start, end));

	while (!isCompeletelyVisited(graph)) {
		Vertex* lcNode = pqueue.dequeue();
		lcNode->setColor(GREEN);
		lcNode->visited = true;

		if (lcNode == end) {
			path = createPath(end);
			break;
		}

		for (auto& edge : lcNode->edges) {
			if (aStar) {
				heuristicValue = heuristicFunction(edge->finish, end);
			}
			if (edge->finish->getColor() == UNCOLORED) {
				edge->finish->setColor(YELLOW);
				edge->finish->previous = lcNode;
				edge->finish->cost = lcNode->cost + edge->cost;
				pqueue.enqueue(edge->finish, lcNode->cost + edge->cost + heuristicValue);
			}
			else if (edge->finish->getColor() == YELLOW) {
				if (edge->cost + lcNode->cost < edge->finish->cost) {
					edge->finish->previous = lcNode;
					edge->finish->cost = lcNode->cost + edge->cost;
					pqueue.changePriority(edge->finish, lcNode->cost + edge->cost + heuristicValue);
				}
			}
		}
	}
	return path;
}

Set<Edge*> kruskal(BasicGraph& graph) {
	Set<Edge*> mst;
	LinkedList<Set<Vertex*>> clusters;
	PriorityQueue<Edge*> edges;

	for (auto& vertex : graph.getVertexSet()) {
		Set<Vertex*> cluster; cluster.insert(vertex);
		clusters += cluster;
	}
	for (auto& edge : graph.getEdgeSet()) {
		edges.enqueue(edge, edge->cost);
	}

	while (clusters.size() != 1) {
		Edge* edge = edges.dequeue();

		int startCluster, finishCluster, index = 0;
        startCluster = finishCluster = -1;

			for (auto& cluster : clusters) {
				if (cluster.contains(edge->start)) {
					startCluster = index;
				}
				if (cluster.contains(edge->finish)) {
					finishCluster = index;
				}

				if ((startCluster != -1) && (startCluster == finishCluster)) {
					break;
				}
				index++;
			}

		if (startCluster != finishCluster) {
			clusters[startCluster] += clusters[finishCluster];
			clusters.remove(finishCluster);
			mst.insert(edge);
		}
	}
	return mst;
}

Vector<Vertex*> createPath(Vertex* end)
{
	Vector<Vertex*> path;

	Vertex* current = end;
	while (current != NULL) {
		path.push_back(current);
		current = current->previous;
	}

	reverse(path.begin(), path.end());
	return path;
}

bool isCompeletelyVisited(BasicGraph& graph)
{
	for (auto& node : graph.getVertexSet()) {
		if (!node->visited) {
			return false;
		}
	}
	return true;
}
