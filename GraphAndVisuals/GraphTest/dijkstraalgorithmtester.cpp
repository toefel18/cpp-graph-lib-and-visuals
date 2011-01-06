#include <iostream>
#include <cassert>
#include <cmath>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "../graph/vertex.h"
#include "../graph/edge.h"
#include "../graph/weightcomputer.h"
#include "../graph/adjacencylist.h"
#include "../graph/dijkstraalgorithm.h"

#include "dijkstraalgorithmtester.h"

using namespace std;

/* 
* Test implementations
* Last updated by Christophe Hesters, 19-12-2010 (tested on MSVC++ 10.0 and GCC 4.4 (with -std=gnu++0x))
*
* performance tests of 500000 iteration showed that GCC was faster by ~0.35 seconds at getShortestPaths()!
**/

//anonymous namespace limits access to file level!
namespace
{
	//custom edge
	class MyEdge
	{
		float speed;
		float distance;
	public:
		MyEdge(float distance, float speed)
			:speed(speed), distance(distance)
		{}

		float getSpeed() const {return speed;}
		float getDistance() const {return distance;}
	};

	//custom weight computer functor for MyEdge
	class MyWeightComputer
	{
	public:
		bool focusOnSpeed;
		static const int MAX_SPEED = 81;
		inline float operator()(const MyEdge &edge) const
		{
			if(focusOnSpeed)
				//because the lowest value gets higher priority, we substract speed from maxspeed!
				return MAX_SPEED - edge.getSpeed();
			else
				return edge.getDistance();
		};
	};
}


namespace graph
{
	void DijkstraAlgorithmTester::runTests()
	{
		using std::cout;

		cout << "Testing DijkstraAlgorithm ... " << endl;
		initVertices();

		testGetShortestDistances();
		testGetShortestPath();
		testGetShortestPaths();
		
		measureGetShortestDistances();
		measureGetShortestPath();
		measureGetShortestPaths();
		
		destroyVertices();
		cout << "All tests passed!" << endl;
	}

	
	void DijkstraAlgorithmTester::testGetShortestDistances()
	{
		cout << "- Testing getShortestDistances() ... " << flush;
		static const float precision = 0.0001f;

		AdjacencyList<Vertex, Edge> graph;

		setup(graph);
		
		DijkstraAlgorithm<> algorithm(graph);
		
		DijkstraAlgorithm<>::DistanceMap shortestDistances = algorithm.getShortestDistances(p1);
	
		assert(abs(shortestDistances[p2] - 7) < precision);
		assert(abs(shortestDistances[p3] - 9) < precision);
		assert(abs(shortestDistances[p4] - 20) < precision);
		assert(abs(shortestDistances[p5] - 20) < precision);
		assert(abs(shortestDistances[p6] - 11) < precision);
	
		//test generics!
		typedef AdjacencyList<Vertex, MyEdge> CustomGraph;
		typedef DijkstraAlgorithm<Vertex, MyEdge, MyWeightComputer, CustomGraph> CustomDijkstra;

		CustomGraph myGraph;
		//you do not need to add the vertices, that is also done by addEdge!

		setupCustom(myGraph);

		CustomDijkstra customAlgorithm(myGraph);

		MyWeightComputer computer;

		computer.focusOnSpeed = false;
		CustomDijkstra::DistanceMap distancePaths = customAlgorithm.getShortestDistances(p1, computer);
		assert(abs(distancePaths[p4] - 8) < precision);
		assert(abs(distancePaths[p2] - 7) < precision);
		assert(abs(distancePaths[p3] - 4) < precision);

		computer.focusOnSpeed = true; //this should alter the Dijkstra Algorithm outcome!
		CustomDijkstra::DistanceMap speedingPaths= customAlgorithm.getShortestDistances(p1, computer);
		assert(abs(speedingPaths[p4] - 41) < precision);
		assert(abs(speedingPaths[p2] - 1) < precision);
		assert(abs(speedingPaths[p3] - 30) < precision);
		
		cout << "Passed!" << endl;
	}

	void DijkstraAlgorithmTester::testGetShortestPath()
	{
		cout << "- Testing getShortestPath() ... " << flush;
		AdjacencyList<Vertex, Edge> graph;
		static const float precision = 0.0001f;
		setup(graph);
		
		DijkstraAlgorithm<> algorithm(graph);
		
		DijkstraAlgorithm<>::Path shortestPaths = algorithm.getShortestPath(p1, p5);
	
		DijkstraAlgorithm<>::Path::iterator it = shortestPaths.begin();
		assert(it->first == p3);
		assert(abs(it->second.getDistance() - 9) < precision);

		++it;
		assert(it->first == p6);
		assert(abs(it->second.getDistance() - 2) < precision);
		
		++it;
		assert(it->first == p5);
		assert(abs(it->second.getDistance() - 9) < precision);

		//test generics!
		typedef AdjacencyList<Vertex, MyEdge> CustomGraph;
		typedef DijkstraAlgorithm<Vertex, MyEdge, MyWeightComputer, CustomGraph> CustomDijkstra;

		CustomGraph myGraph;

		setupCustom(myGraph);

		CustomDijkstra customAlgorithm(myGraph);

		MyWeightComputer computer;

		computer.focusOnSpeed = false;
		CustomDijkstra::Path distPath = customAlgorithm.getShortestPath(p1, p4, computer);
		assert(distPath.begin()->first == p2);
		assert((++distPath.begin())->first == p4);

		computer.focusOnSpeed = true;
		CustomDijkstra::Path speedPath = customAlgorithm.getShortestPath(p1, p4, computer);
		assert(speedPath.begin()->first == p3);
		assert((++speedPath.begin())->first == p4);

		cout << "Passed!" << endl;
	}

	void DijkstraAlgorithmTester::testGetShortestPaths()
	{
		cout << "- Testing getShortestPaths() ... " << flush;
		AdjacencyList<Vertex, Edge> graph;
		static const float precision = 0.0001f;
		setup(graph);
		
		DijkstraAlgorithm<> algorithm(graph);
		
		DijkstraAlgorithm<>::PathMap shortestPaths = algorithm.getShortestPaths(p1);
	
		assert(shortestPaths.size() == 5);

		//to p5
		DijkstraAlgorithm<>::Path::iterator it = shortestPaths[p5].begin();
		assert(it->first == p3);
		assert(abs(it->second.getDistance() - 9) < precision);

		++it;
		assert(it->first == p6);
		assert(abs(it->second.getDistance() - 2) < precision);
		
		++it;
		assert(it->first == p5);
		assert(abs(it->second.getDistance() - 9) < precision);
		assert(++it == shortestPaths[p5].end());

		//to p4
		it = shortestPaths[p4].begin();
		assert(it->first == p3);
		assert(abs(it->second.getDistance() - 9) < precision);

		++it;
		assert(it->first == p4);
		assert(abs(it->second.getDistance() - 11) < precision);
		assert(++it == shortestPaths[p4].end());

		//to p2
		it = shortestPaths[p2].begin();
		assert(it->first == p2);
		assert(abs(it->second.getDistance() - 7) < precision);
		assert(++it == shortestPaths[p2].end());

		//test generics!
		typedef AdjacencyList<Vertex, MyEdge> CustomGraph;
		typedef DijkstraAlgorithm<Vertex, MyEdge, MyWeightComputer, CustomGraph> CustomDijkstra;

		CustomGraph myGraph;

		setupCustom(myGraph);

		CustomDijkstra customAlgorithm(myGraph);

		MyWeightComputer computer;

		computer.focusOnSpeed = false;
		CustomDijkstra::PathMap distPaths = customAlgorithm.getShortestPaths(p1, computer);
		
		CustomDijkstra::Path::iterator customIt = distPaths[p4].begin();
		assert(customIt->first == p2);
		assert(abs(customIt->second.getDistance() - 7) < precision);

		++customIt;
		assert(customIt->first == p4);
		assert(abs(customIt->second.getDistance() - 1) < precision);
		assert(++customIt == distPaths[p4].end());

		computer.focusOnSpeed = true;
		CustomDijkstra::PathMap speedPaths = customAlgorithm.getShortestPaths(p1, computer);
		customIt = speedPaths[p4].begin();

		assert(customIt->first == p3);
		assert(abs(customIt->second.getSpeed() - 51) < precision);

		++customIt;
		assert(customIt->first == p4);
		assert(abs(customIt->second.getSpeed() - 70) < precision);
		assert(++customIt == speedPaths[p4].end());

		cout << "Passed!" << endl;
	}
		
	void DijkstraAlgorithmTester::measureGetShortestPath()
	{
		cout << "- measuring getShortestPath() ... " << flush;
		AdjacencyList<Vertex, Edge> graph;

		setup(graph);
		
		DijkstraAlgorithm<> algorithm(graph);

		boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::local_time();

		//since p5 is the node most far (3 steps) away, it won't make too much difference
		//compared to getting all the shortest paths, but p6 is two steps away and that
		//shows the actualy performance increase in getting just a single path!
		for(int i = 0; i < TestRunsShortestPath; i++)
			DijkstraAlgorithm<>::Path shortestPaths = algorithm.getShortestPath(p1, p6);

		boost::posix_time::ptime endTime = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration duration = endTime.time_of_day() - startTime.time_of_day();

		cout << endl 
			 << "  - " << TestRunsShortestPath << " iterations with " << endl 
			 << "  - " << graph.getNumVertices() << " vertices and " << graph.getNumEdges() << " edges" << endl
			 << "  - took " << duration << endl;

		cout << "done!" << endl;
	}

	void DijkstraAlgorithmTester::measureGetShortestPaths()
	{
		cout << "- measuring getShortestPaths() ... " << flush;	
		AdjacencyList<Vertex, Edge> graph;

		setup(graph);

		DijkstraAlgorithm<> algorithm(graph);

		boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::local_time();

		for(int i = 0; i < TestRunsShortestPaths; i++)
			DijkstraAlgorithm<>::PathMap shortestPaths = algorithm.getShortestPaths(p1);

		boost::posix_time::ptime endTime = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration duration = endTime.time_of_day() - startTime.time_of_day();

		cout << endl 
			 << "  - " << TestRunsShortestPaths << " iterations with " << endl 
			 << "  - " << graph.getNumVertices() << " vertices and " << graph.getNumEdges() << " edges" << endl
			 << "  - took " << duration << endl;

		cout << "done!" << endl;
	}

	void DijkstraAlgorithmTester::measureGetShortestDistances()
	{
		cout << "- measuring getShortestDistances() ... " << flush;
		AdjacencyList<Vertex, Edge> graph;

		setup(graph);
		
		DijkstraAlgorithm<> algorithm(graph);

		boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::local_time();

		for(int i = 0; i < TestRunsDistances; i++)
			DijkstraAlgorithm<>::DistanceMap shortestPaths = algorithm.getShortestDistances(p1);

		boost::posix_time::ptime endTime = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration duration = endTime.time_of_day() - startTime.time_of_day();

		cout << endl 
			 << "  - " << TestRunsDistances << " iterations with " << endl 
			 << "  - " << graph.getNumVertices() << " vertices and " << graph.getNumEdges() << " edges" << endl
			 << "  - took " << duration << endl;
	
		cout << "done!" << endl;
	}

	
	template <typename Graph>
	void DijkstraAlgorithmTester::setup(Graph &graph)
	{
		//models this graph: http://upload.wikimedia.org/wikipedia/commons/4/45/Dijksta_Anim.gif
		
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
	void DijkstraAlgorithmTester::setupCustom(Graph &myGraph)
	{
		myGraph.addEdge(p1, p2, MyEdge(7, 80));
			assert(myGraph.getNumVertices() == 2);
			assert(myGraph.getNumEdges() == 2);
		
		myGraph.addEdge(p1, p3, MyEdge(4, 51));
			assert(myGraph.getNumVertices() == 3);
			assert(myGraph.getNumEdges() == 4);
		
		myGraph.addEdge(p3, p4, MyEdge(5, 70));
			assert(myGraph.getNumVertices() == 4);
			assert(myGraph.getNumEdges() == 6);
		
		myGraph.addEdge(p2, p4, MyEdge(1, 30));
			assert(myGraph.getNumVertices() == 4);
			assert(myGraph.getNumEdges() == 8);
	}	
	
	void DijkstraAlgorithmTester::initVertices()
	{
		p1 = point(new Vertex("1")); 
		p2 = point(new Vertex("2")); 
		p3 = point(new Vertex("3"));	
		p4 = point(new Vertex("4")); 
		p5 = point(new Vertex("5")); 
		p6 = point(new Vertex("6")); 
	}

	void DijkstraAlgorithmTester::destroyVertices()
	{
		delete p1;
		delete p2;
		delete p3;
		delete p4;
		delete p5;
		delete p6;
	}
}
