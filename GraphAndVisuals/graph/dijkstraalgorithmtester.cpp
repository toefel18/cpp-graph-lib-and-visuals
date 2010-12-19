#include "dijkstraalgorithmtester.h"
#include <iostream>
#include "vertex.h"
#include "edge.h"
#include "adjacencylist.h"
#include "dijkstraalgorithm.h"
#include <cassert>
#include <cmath>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace std;


/* 
* Test implementations
* Last updated by Christophe Hesters, 19-12-2010 
**/

namespace graph
{
	void DijkstraAlgorithmTester::runTests()
	{
		using std::cout;

		cout << "Testing DijkstraAlgorithm ... " << endl;
		
		testGetShortestDistances();
		testGetShortestPath();
		testGetShortestPaths();
		
		measureGetShortestDistances();
		measureGetShortestPath();
		measureGetShortestPaths();
		
		cout << "All tests passed!" << endl;
	}

	
	void DijkstraAlgorithmTester::testGetShortestDistances()
	{
		cout << "- Testing getShortestDistances() ... " << flush;

		AdjacencyList<Vertex, Edge> graph;

		setup(graph);
		
		DijkstraAlgorithm<> algorithm(graph);
		
		DijkstraAlgorithm<>::DistanceMap shortestPaths = algorithm.getShortestDistances(p1);
	
		assert(abs(shortestPaths[p2] - 7) < 0.0001);
		assert(abs(shortestPaths[p3] - 9) < 0.0001);
		assert(abs(shortestPaths[p4] - 20) < 0.0001);
		assert(abs(shortestPaths[p5] - 20) < 0.0001);
		assert(abs(shortestPaths[p6] - 11) < 0.0001);
	
		//todo, add more tests!

		tearDown(graph);
		
		cout << "Passed!" << endl;
	}

	void DijkstraAlgorithmTester::testGetShortestPath()
	{
		cout << "- Testing getShortestPath() ... " << flush;
		AdjacencyList<Vertex, Edge> graph;

		setup(graph);

		//implement here

		tearDown(graph);
		
		cout << "Passed!" << endl;
	}

	void DijkstraAlgorithmTester::testGetShortestPaths()
	{
		cout << "- Testing getShortestPaths() ... " << flush;
		AdjacencyList<Vertex, Edge> graph;

		setup(graph);

		//implement here

		tearDown(graph);
		
		cout << "Passed!" << endl;
	}
		
	void DijkstraAlgorithmTester::measureGetShortestPath()
	{
		cout << "- measuring getShortestPath() ... " << flush;
		AdjacencyList<Vertex, Edge> graph;

		setup(graph);

		//implement here

		
		tearDown(graph);
		
		cout << "done!" << endl;
	}

	void DijkstraAlgorithmTester::measureGetShortestPaths()
	{
		cout << "- measuring getShortestPaths() ... " << flush;	
		AdjacencyList<Vertex, Edge> graph;

		setup(graph);

		//implement here

		tearDown(graph);
		
		cout << "done!" << endl;
	}

	void DijkstraAlgorithmTester::measureGetShortestDistances()
	{
		cout << "- measuring getShortestDistances() ... " << flush;
		AdjacencyList<Vertex, Edge> graph;

		setup(graph);
		
		DijkstraAlgorithm<> algorithm(graph);

		boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::local_time();

		for(int i = 0; i < TestRuns; i++)
			DijkstraAlgorithm<>::DistanceMap shortestPaths = algorithm.getShortestDistances(p1);

		boost::posix_time::ptime endTime = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration duration = endTime.time_of_day() - startTime.time_of_day();

		cout << endl 
			 << "  - " << TestRuns << " iterations with " << endl 
			 << "  - " << graph.getNumVertices() << " vertices and " << graph.getNumEdges() << " edges" << endl
			 << "  - took " << duration << endl;
	
		tearDown(graph);
		
		cout << "done!" << endl;
	}

	
	template <typename Graph>
	void DijkstraAlgorithmTester::setup(Graph &graph)
	{
		//models this graph: http://upload.wikimedia.org/wikipedia/commons/4/45/Dijksta_Anim.gif
		p1 = point(new Vertex("1")); 
		p2 = point(new Vertex("2")); 
		p3 = point(new Vertex("3"));	
		p4 = point(new Vertex("4")); 
		p5 = point(new Vertex("5")); 
		p6 = point(new Vertex("6")); 
		
		assert(graph.getNumVertices() == 0);
		assert(graph.getNumEdges() == 0);

		graph.addVertex(p1);
		graph.addVertex(p2);
		graph.addVertex(p3);
		graph.addVertex(p4);
		graph.addVertex(p5);
		graph.addVertex(p6);

		assert(graph.getNumVertices() == 6);
		assert(graph.getNumEdges() == 0);

		graph.addEdge(p1, p2, Edge(7));
		graph.addEdge(p1, p3, Edge(9));
		graph.addEdge(p1, p6, Edge(14));

		assert(graph.getNumVertices() == 6);
		assert(graph.getNumEdges() == 6); // edes are undirected, so they are duplicated!

		graph.addEdge(p2, p3, Edge(10));
		graph.addEdge(p2, p4, Edge(15));

		graph.addEdge(p3, p4, Edge(11));
		graph.addEdge(p3, p6, Edge(2));

		graph.addEdge(p4, p5, Edge(6));
		graph.addEdge(p6, p5, Edge(9));

		assert(graph.getNumVertices() == 6);
		assert(graph.getNumEdges() == 18);
	}	

	template <typename Graph>
	void DijkstraAlgorithmTester::tearDown(Graph &graph)
	{
		graph.deleteAllVertices();
		//for(auto i = graph.vertices.begin(); i != graph.vertices.end(); ++i)
		//	delete (*i);
	}
}
