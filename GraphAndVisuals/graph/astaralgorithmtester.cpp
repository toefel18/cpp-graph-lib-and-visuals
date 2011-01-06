#include "astaralgorithmtester.h"
#include <iostream>
#include "vertex.h"
#include "edge.h"
#include "weightcomputer.h"
#include "heuristic.h"
#include "adjacencylist.h"
#include "astaralgorithm.h"
#include <cassert>
#include <cmath>
#include <boost/date_time/posix_time/posix_time.hpp>

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
	class MyVertex
	{
		std::string name;
		float _x, _y;
	public:
		/*!
		* \brief Constructs a vertex
		* 
		* \param name the name of the vertex
		*/
		MyVertex(std::string name, float x, float y)
			:name(name),_x(x),_y(y) {}
		MyVertex(float x, float y)
			:name(""),_x(x),_y(y) {}
		/*!
		* \brief Gets the name of the vertex
		* 
		* \returns the name of the vertex
		*/
		std::string getName() const {return name;}

		//
		inline float x() const {return _x;}
		inline float y() const {return _y;}
	};

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

	//heuristic between current and goal vertex
	class DistanceHeuristic
	{

	public:
		enum Mode{
			EUCLIDEAN,
			MANHATTAN,
			NONE,
		};

		DistanceHeuristic()
			:mode(EUCLIDEAN){}

		/*!
		* \brief Straight line heuristic that returns the straight line distance
		*		 between two vertices of type MyVertex 
		* 
		* \param current the current node
		* \param goal the end node
		* 
		* \returns straight line distance between the nodes
		*/
		inline float operator()(const MyVertex* current, const MyVertex* goal) const
		{
			if(mode == EUCLIDEAN)
			{
				float dx = goal->x() - current->x();
				float dy = goal->y() - current->y();
				return std::sqrt(dx*dx + dy*dy);
			} 
			else if (mode == MANHATTAN)
			{
				float dx = std::fabs(goal->x() - current->x());
				float dy = std::fabs(goal->y() - current->y());
				return dx + dy;
			} 
			else {
				return 0.0f;
			}
		};
		void setMode(Mode newMode){mode = newMode;}

	private:
		Mode mode;
	};
}

namespace graph
{
	void AStarAlgorithmTester::runTests()
	{
		using std::cout;

		cout << "Testing AStarAlgorithm ... " << endl;
		initVertices();

		testGetShortestPath();
		
		measureGetShortestPath();
		
		destroyVertices();
		cout << "All tests passed!" << endl;
	}

	void AStarAlgorithmTester::testGetShortestPath()
	{
		cout << "- Testing getShortestPath() ... " << flush;

		//test basic functionality with zero heuristic, equivalent to Dijkstra's algorithm!
		AdjacencyList<Vertex, Edge> graph;
		static const float precision = 0.0001f;
		setup(graph);
		
		AStarAlgorithm<> algorithm(graph);
		
		AStarAlgorithm<>::Path shortestPaths = algorithm.getShortestPath(p1, p5);
	
		AStarAlgorithm<>::Path::iterator it = shortestPaths.begin();
		assert(it->first == p3);
		assert(abs(it->second.getDistance() - 9) < precision);

		++it;
		assert(it->first == p6);
		assert(abs(it->second.getDistance() - 2) < precision);
		
		++it;
		assert(it->first == p5);
		assert(abs(it->second.getDistance() - 9) < precision);

		tearDown(graph);

		cout << "1 " << flush;

		//test generics!
		typedef AdjacencyList<Vertex, MyEdge> CustomGraph;
		typedef AStarAlgorithm<Vertex, MyEdge, MyWeightComputer, DefaultHeuristic<Vertex>, CustomGraph> CustomAStar;

		CustomGraph myGraph;

		setupCustom(myGraph);

		CustomAStar customAlgorithm(myGraph);

		MyWeightComputer computer;

		computer.focusOnSpeed = false;
		CustomAStar::Path distPath = customAlgorithm.getShortestPath(p1, p4, computer);
		assert(distPath.begin()->first == p2);
		assert((++distPath.begin())->first == p4);

		computer.focusOnSpeed = true;
		CustomAStar::Path speedPath = customAlgorithm.getShortestPath(p1, p4, computer);
		assert(speedPath.begin()->first == p3);
		assert((++speedPath.begin())->first == p4);

		tearDown(myGraph);
		
		cout << "2 " << flush;

		//test heuristics!
		typedef AdjacencyList<MyVertex, Edge> HeuristicGraph;
		typedef AStarAlgorithm<MyVertex, Edge, WeightComputer<Edge>, DistanceHeuristic, HeuristicGraph> HeuristicAStar;

		HeuristicGraph myHeuristicGraph;
		
		//we want to use these for pathfinding
		MyVertex *start,*end;
	
		setupHeuristicGraph(myHeuristicGraph, &start, &end);

		HeuristicAStar heuristicAlgo(myHeuristicGraph);
		HeuristicAStar::Path heuristicPath = heuristicAlgo.getShortestPath(start,end, WeightComputer<Edge>(), DistanceHeuristic());
		assert(heuristicPath.size() == 2);

		//heuristic works and is verified with global variable iteration count, 
		//there has to be found a cleaner way to test the heuristic! 
		//this verifiction code has been removed

		tearDown(myHeuristicGraph);
		
		cout << "3 " << flush;

		//ADD TEST THAT COMBINES HEURISTIC WITH WEIGHT COMPUTER, SELECT DIFFERENT WEIGHT SO TO SEE
		//WHICH ROUTE ASTAR SELECTS, THIS MIGHT NOT BE THE MOST OPTIMAL ONE!

		cout << "Passed!" << endl;
	}

	void AStarAlgorithmTester::measureGetShortestPath()
	{
		cout << "- measuring getShortestPath() ... " << flush;
		AdjacencyList<Vertex, Edge> graph;

		setup(graph);
		
		AStarAlgorithm<> algorithm(graph);

		boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::local_time();

		//since p5 is the node most far (3 steps) away, it won't make too much difference
		//compared to getting all the shortest paths, but p6 is two steps away and that
		//shows the actualy performance increase in getting just a single path!
		for(int i = 0; i < TestRunsShortestPath; i++)
			AStarAlgorithm<>::Path shortestPaths = algorithm.getShortestPath(p1, p6);

		boost::posix_time::ptime endTime = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration duration = endTime.time_of_day() - startTime.time_of_day();

		cout << endl 
			 << "  - " << TestRunsShortestPath << " iterations with " << endl 
			 << "  - " << graph.getNumVertices() << " vertices and " << graph.getNumEdges() << " edges" << endl
			 << "  - took " << duration << endl;

		tearDown(graph);
		
		cout << "done!" << endl;
	}
		
	template <typename Graph>
	void AStarAlgorithmTester::setup(Graph &graph)
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
	void AStarAlgorithmTester::setupCustom(Graph &myGraph)
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

	template <typename MyVertex, typename Graph>
	void AStarAlgorithmTester::setupHeuristicGraph(Graph &myGraph, MyVertex** start, MyVertex **end)
	{
		const float DEFAULT_EDGE_VAL = 1.0;
		const int ROWS = 3;
		const int COLS = 3;

		MyVertex * vertices[ROWS][COLS];

		for(int x = 0; x < ROWS; x++)
			for(int y = 0; y < COLS; y++)
				vertices[x][y] = new MyVertex((float)x, (float)y);
		
		//put 2 corners back in the caller vars
		*start = vertices[0][0];
		*end = vertices[ROWS-1][COLS-1];

		//add all horizontal edges
		for(int y = 0; y < COLS; y++)
		{
			myGraph.addEdge(vertices[0][y], vertices[1][y], Edge(DEFAULT_EDGE_VAL));
			myGraph.addEdge(vertices[1][y], vertices[2][y], Edge(DEFAULT_EDGE_VAL));
		}
		assert(myGraph.getNumEdges() == 12);

		//add all vertical edges
		for(int x = 0; x < ROWS; x++)
		{
			myGraph.addEdge(vertices[x][0], vertices[x][1], Edge(DEFAULT_EDGE_VAL));
			myGraph.addEdge(vertices[x][1], vertices[x][2], Edge(DEFAULT_EDGE_VAL));
		}
		assert(myGraph.getNumEdges() == 24);

		MyVertex *middle = vertices[1][1];
		myGraph.addEdge(middle, vertices[0][0], Edge(DEFAULT_EDGE_VAL));
		myGraph.addEdge(middle, vertices[0][2], Edge(DEFAULT_EDGE_VAL));
		myGraph.addEdge(middle, vertices[2][0], Edge(DEFAULT_EDGE_VAL));
		myGraph.addEdge(middle, vertices[2][2], Edge(DEFAULT_EDGE_VAL));

		assert(myGraph.getNumEdges() == 32);
		assert(myGraph.getNumVertices() == 9);
	}	

	template <typename Graph>
	void AStarAlgorithmTester::tearDown(Graph &graph)
	{
		//any teardown code needed
	}
	
	void AStarAlgorithmTester::initVertices()
	{
		p1 = point(new Vertex("1")); 
		p2 = point(new Vertex("2")); 
		p3 = point(new Vertex("3"));	
		p4 = point(new Vertex("4")); 
		p5 = point(new Vertex("5")); 
		p6 = point(new Vertex("6")); 
	}

	void AStarAlgorithmTester::destroyVertices()
	{
		delete p1;
		delete p2;
		delete p3;
		delete p4;
		delete p5;
		delete p6;
	}
}
