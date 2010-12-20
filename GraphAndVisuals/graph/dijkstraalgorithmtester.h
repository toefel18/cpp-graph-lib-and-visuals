#ifndef GRAPH_DIJKSTRA_ALGORITHM_TESTER_H
#define GRAPH_DIJKSTRA_ALGORITHM_TESTER_H


namespace graph
{
	class Vertex;

	/* 
	* DijkstraAlgorithm Tester
	* Tests the dijkstra algorithm
	*
	* Written by Christophe Hesters, 14-12-2010 (tested on MSVC++ 10.0 and GCC 4.4 (with -std=gnu++0x))
	**/
	class DijkstraAlgorithmTester
	{
	public:
		//increase these values in the release build to 500000 to get more relevant results!
		static const int TestRunsDistances = 5000;
		static const int TestRunsShortestPaths = 5000;
		static const int TestRunsShortestPath = 5000;

		typedef Vertex* point;

		void initVertices();
		void destroyVertices();

		template <typename Graph>
		void setup(Graph &graph);

		template <typename Graph>
		void setupCustom(Graph &graph);

		template <typename Graph>
		void tearDown(Graph &graph);
		
		void runTests();

		void testGetShortestDistances();
		void testGetShortestPath();
		void testGetShortestPaths();
		
		void measureGetShortestDistances();
		void measureGetShortestPath();
		void measureGetShortestPaths();
	private:
		point p1, p2, p3, p4, p5, p6;
	};

}
#endif //GRAPH_DIJKSTRA_ALGORITHM_TESTER_H
