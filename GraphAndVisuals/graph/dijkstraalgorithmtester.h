#ifndef GRAPH_DIJKSTRA_ALGORITHM_TESTER_H
#define GRAPH_DIJKSTRA_ALGORITHM_TESTER_H


namespace graph
{
	class Vertex;

	/* 
	* DijkstraAlgorithm Tester
	* Tests the dijkstra algorithm
	*
	* Written by Christophe Hesters, 14-12-2010 
	**/
	class DijkstraAlgorithmTester
	{
	public:
		static const int TestRuns = 100;

		typedef Vertex* point;

		template <typename Graph>
		void setup(Graph &graph);

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
