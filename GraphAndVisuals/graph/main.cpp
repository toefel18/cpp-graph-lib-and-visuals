#include <iostream>
#include <tuple>
#include <utility>

#include "vertex.h"
#include "edge.h"
#include "adjacencylist.h"
#include "dijkstraalgorithm.h"
#include "dijkstraalgorithmtester.h"
#include "kruskalalgorithm.h"
#include "primalgorithm.h"
#include "astaralgorithmtester.h"

using namespace std;

void testDijkstra()
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

	graph.deleteAllVertices();
}

void testKruskal()
{
	using namespace graph;

	AdjacencyList<> graph;

	typedef Vertex* point;
	
	//models this graph: http://en.wikipedia.org/wiki/Kruskal%27s_algorithm#Example
	point pa = point(new Vertex("A")); 
	point pb = point(new Vertex("B")); 
	point pc = point(new Vertex("C"));	
	point pd = point(new Vertex("D"));	
	point pe = point(new Vertex("E"));	
	point pf = point(new Vertex("F"));
	point pg = point(new Vertex("G"));	
	
	graph.addVertex(pa);
	graph.addVertex(pb);
	graph.addVertex(pc);
	graph.addVertex(pd);
	graph.addVertex(pe);
	graph.addVertex(pf);
	graph.addVertex(pg);
	
	graph.addEdge(pa, pb, Edge(7));
	graph.addEdge(pa, pd, Edge(5));
	
	graph.addEdge(pb, pc, Edge(8));
	graph.addEdge(pb, pd, Edge(9));
	graph.addEdge(pb, pe, Edge(7));
	
	graph.addEdge(pc, pe, Edge(5));
	
	graph.addEdge(pd, pe, Edge(15));
	graph.addEdge(pd, pf, Edge(6));
	
	graph.addEdge(pe, pf, Edge(8));
	graph.addEdge(pe, pg, Edge(9));
	
	graph.addEdge(pf, pg, Edge(11));

	// Test algorithm.
	KruskalAlgorithm<> algorithm(graph);

	KruskalAlgorithm<>::EdgeMap minimumSpanningTree = algorithm.getMinimumSpanningTree();

	cout << endl << endl << "A minimum spanning tree (found using Kruskal's Algorithm) contains the following edges:" << endl;

	for(KruskalAlgorithm<>::EdgeMap::iterator edgeInfo = minimumSpanningTree.begin(), endEdgeInfo = minimumSpanningTree.end(); edgeInfo != endEdgeInfo; ++edgeInfo)
	{
		cout << " - edge with distance " << std::get<0>(*edgeInfo)->getDistance() << " from vertex " << std::get<1>(*edgeInfo)->getName() << " to vertex " << std::get<2>(*edgeInfo)->getName() << endl;
	}

	// Clean up.
	graph.deleteAllVertices();
}

void testPrim()
{
	using namespace graph;

	AdjacencyList<> graph;

	typedef Vertex* point;
	
	//models this graph: http://en.wikipedia.org/wiki/Prim%27s_algorithm#Example
	point pa = point(new Vertex("A")); 
	point pb = point(new Vertex("B")); 
	point pc = point(new Vertex("C"));	
	point pd = point(new Vertex("D"));	
	point pe = point(new Vertex("E"));	
	point pf = point(new Vertex("F"));
	point pg = point(new Vertex("G"));	
	
	graph.addVertex(pa);
	graph.addVertex(pb);
	graph.addVertex(pc);
	graph.addVertex(pd);
	graph.addVertex(pe);
	graph.addVertex(pf);
	graph.addVertex(pg);
	
	graph.addEdge(pa, pb, Edge(7));
	graph.addEdge(pa, pd, Edge(5));
	
	graph.addEdge(pb, pc, Edge(8));
	graph.addEdge(pb, pd, Edge(9));
	graph.addEdge(pb, pe, Edge(7));
	
	graph.addEdge(pc, pe, Edge(5));
	
	graph.addEdge(pd, pe, Edge(15));
	graph.addEdge(pd, pf, Edge(6));
	
	graph.addEdge(pe, pf, Edge(8));
	graph.addEdge(pe, pg, Edge(9));
	
	graph.addEdge(pf, pg, Edge(11));

	// Test algorithm.
	PrimAlgorithm<> algorithm(graph);

	PrimAlgorithm<>::EdgeMap minimumSpanningTree = algorithm.getMinimumSpanningTree(pd);

	cout << endl << endl << "A minimum spanning tree (found using Prim's Algorithm) contains the following edges:" << endl;

	for(KruskalAlgorithm<>::EdgeMap::iterator edgeInfo = minimumSpanningTree.begin(), endEdgeInfo = minimumSpanningTree.end(); edgeInfo != endEdgeInfo; ++edgeInfo)
	{
		cout << " - edge with distance " << std::get<0>(*edgeInfo)->getDistance() << " from vertex " << std::get<1>(*edgeInfo)->getName() << " to vertex " << std::get<2>(*edgeInfo)->getName() << endl;
	}

	// Clean up.
	graph.deleteAllVertices();
}

void testAStar()
{
	using namespace graph;
	AStarAlgorithmTester astarTester;
	astarTester.runTests();
}

int main(int argc, char** argv)
{
	testDijkstra();
	testAStar();
	testKruskal();
	testPrim();

	cout << "Press enter to close.." << endl;

	cin.get();

	return 0;
}