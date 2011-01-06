#ifndef GRAPH_ASTAR_ALGORITHM_TESTER_H
#define GRAPH_ASTAR_ALGORITHM_TESTER_H


namespace graph
{
	class Vertex;

	/*! 
	* \brief Tests the AStar algorithm
	*
	* \author Christophe Hesters
	* \date 14-12-2010 
	* \note tested on MSVC++ 10.0 
	*/
	class AStarAlgorithmTester
	{
	public:
		//increase these values in the release build to 500000 to get more relevant results!
		static const int TestRunsShortestPath = 5000;

		typedef Vertex* point;

		void initVertices();
		void destroyVertices();

		template <typename Graph>
		void setup(Graph &graph);

		template <typename Graph>
		void setupCustom(Graph &graph);

		template <typename MyVertex, typename Graph>
		void setupHeuristicGraph(Graph &graph, MyVertex** start, MyVertex **end);

		template <typename Graph>
		void tearDown(Graph &graph);
		
		void runTests();
		void testGetShortestPath();
		void measureGetShortestPath();
	private:
		point p1, p2, p3, p4, p5, p6;
	};

}
#endif //GRAPH_ASTAR_ALGORITHM_TESTER_H
