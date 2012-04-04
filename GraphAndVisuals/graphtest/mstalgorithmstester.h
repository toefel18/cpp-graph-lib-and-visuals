#ifndef GRAPH_MSTALGORITHMS_TESTER_H
#define GRAPH_MSTALGORITHMS_TESTER_H

#include <vector>
#ifdef WIN32
	#include <tuple>
#else
	#include <tr1/tuple>
#endif
#include <boost/date_time/posix_time/posix_time.hpp>

namespace graph
{
	/*! 
	* \brief Tests the Minumum Spanning Tree algorithms.
	*
	* \author Jeffrey Krist
	* \date 6-1-2011 
	* \note Tested on MSVC++ 10.0.
	*/
	class MSTAlgorithmsTester
	{
	public:
		static const int MaxVertices = 75;
		static const int MaxEdgesPerVertex = 4;
		static const int TestRuns = 12;

		typedef AdjacencyList<Vertex, Edge> Graph;
		typedef std::tr1::tuple<const Edge*, const Vertex*, const Vertex*> EdgeMapItem;
		typedef std::vector<EdgeMapItem> EdgeMap;
		
		void assertEqual(EdgeMap &edgeMapA, EdgeMap &edgeMapB);

		EdgeMap setupFixed(Graph &graph);
		void setupRandom(Graph &graph);
		
		void validateByEqual();
		void validateSingle();

		template <typename MSTAlgorithm>
		void validateSingle();
		
		void performanceTest();
		
		template<typename MSTAlgorithm>
		boost::posix_time::time_duration performanceTest(Graph &graph);
		
		void runTests();
	};

}
#endif //GRAPH_MSTALGORITHMS_TESTER_H
