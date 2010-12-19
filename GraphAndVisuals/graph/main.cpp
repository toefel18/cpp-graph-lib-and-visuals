#include <iostream>
#include "vertex.h"
#include "edge.h"
#include "adjacencylist.h"
#include "dijkstraalgorithm.h"
#include "dijkstraalgorithmTester.h"

using namespace std;

int main(int argc, char** argv)
{
	using namespace graph;

	DijkstraAlgorithmTester dijkstraTester;
	dijkstraTester.runTests();



	AdjacencyList<> graph;

	typedef Vertex* point;
	
	point p1 = point(new Vertex("1")); 
	point p2 = point(new Vertex("2")); 
	point p3 = point(new Vertex("3"));	
	point p4 = point(new Vertex("4")); 
	point p5 = point(new Vertex("5")); 
	point p6 = point(new Vertex("6")); 
	point p7 = point(new Vertex("7"));
	point p8 = point(new Vertex("8"));
	point p9 = point(new Vertex("9"));

	graph.addVertex(p1);
	graph.addVertex(p2);
	graph.addVertex(p3);
	graph.addVertex(p4);
	graph.addVertex(p5);
	graph.addVertex(p6);
	graph.addVertex(p7);
	graph.addVertex(p8);
	graph.addVertex(p9);

	graph.addEdge(p1, p2, Edge(7));
	graph.addEdge(p1, p3, Edge(9));
	graph.addEdge(p1, p6, Edge(14));

	graph.addEdge(p2, p3, Edge(10));
	graph.addEdge(p2, p4, Edge(15));

	graph.addEdge(p3, p4, Edge(11));
	graph.addEdge(p3, p6, Edge(2));

	graph.addEdge(p4, p5, Edge(6));
	graph.addEdge(p6, p5, Edge(9));

	DijkstraAlgorithm<> algorithm(graph);

	DijkstraAlgorithm<>::DistanceMap shortestDistances = algorithm.getShortestDistances(p1);

	DijkstraAlgorithm<>::PathMap shortestPaths = algorithm.getShortestPaths(p1);

	DijkstraAlgorithm<>::Path shortestPathP5 = algorithm.getShortestPath(p1, p5);


	cout << endl << endl << "The paths from point" << p1->getName() << " to other nodes in the graph are: " << endl;
	for(DijkstraAlgorithm<>::DistanceMap::iterator i = shortestDistances.begin(); i != shortestDistances.end(); ++i)
	{
		cout << " - point" << i->first->getName() << " has a shortest path of " << i->second << endl;
	}

	cin.get();
	delete p1, p2, p3, p4, p5, p6;

	return 0;
}