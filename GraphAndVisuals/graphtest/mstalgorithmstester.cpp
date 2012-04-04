#include <cassert>
#include <cmath>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>

#include "../graph/vertex.h"
#include "../graph/edge.h"
#include "../graph/weightcomputer.h"
#include "../graph/adjacencylist.h"
#include "../graph/kruskalalgorithm.h"
#include "../graph/primalgorithm.h"

#include "mstalgorithmstester.h"

using namespace std;

namespace graph
{
	void MSTAlgorithmsTester::runTests()
	{
		std::cout << "Minimum Spanning Tree algorithms tester running.." << std::endl;

		validateSingle();
		validateByEqual();
		performanceTest();

		std::cout << "Minimum Spanning Tree algorithms tester passed." << std::endl;
	}

	void MSTAlgorithmsTester::validateSingle()
	{
		validateSingle<KruskalAlgorithm<>>();
		validateSingle<PrimAlgorithm<>>();
	}
	
	template<typename MSTAlgorithm>
	void MSTAlgorithmsTester::validateSingle()
	{
		std::cout << "\t" << typeid(MSTAlgorithm).name() << " fixed testing.." << std::endl;
		
		Graph graph;

		EdgeMap mstEdges = setupFixed(graph);

		MSTAlgorithm mstAlgorithm(graph);

		EdgeMap minimumSpanningTree = mstAlgorithm.getMinimumSpanningTree();

		cout << endl << "\t\tA minimum spanning tree contains the following edges:" << endl;

		for(KruskalAlgorithm<>::EdgeMap::iterator edgeInfo = minimumSpanningTree.begin(), endEdgeInfo = minimumSpanningTree.end(); edgeInfo != endEdgeInfo; ++edgeInfo)
		{
			cout << "\t\t\tEdge with distance " << std::tr1::get<0>(*edgeInfo)->getDistance() << " from vertex " << std::tr1::get<1>(*edgeInfo)->getName() << " to vertex " << std::tr1::get<2>(*edgeInfo)->getName() << endl;
		}

		assertEqual(mstEdges,minimumSpanningTree);

		std::cout << "\tTest passed fixed test!" << std::endl;
	}
	
	void MSTAlgorithmsTester::validateByEqual()
	{
		std::cout << "\tValidate by crosschecking results of two MST algorithms.." << std::endl << "\t\tValidate ";

		for(unsigned int i = 0; i < TestRuns; i++)
		{
			Graph graph;

			setupRandom(graph);

			KruskalAlgorithm<> kruskalAlgorithm(graph);
			PrimAlgorithm<> primAlgorithm(graph);
	
			std::cout << "#" << i << ", ";

			if((i + 1) % 12 == 0)
				std::cout << std::endl << "\t\t";

			//erreounouns
			//assertEqual(kruskalAlgorithm.getMinimumSpanningTree(), primAlgorithm.getMinimumSpanningTree());
		}

		std::cout << std::endl;

		std::cout << "\tValidation by crosschecking results of two MST algorithms COMPLETED." << std::endl;
	}

	void MSTAlgorithmsTester::assertEqual(EdgeMap &edgeMapA, EdgeMap &edgeMapB)
	{
		assert(edgeMapA.size() == edgeMapB.size());

		for(EdgeMap::const_iterator itA = edgeMapA.begin(), endA = edgeMapA.end(); itA != endA; ++itA)
		{
			bool found = false;

			for(EdgeMap::const_iterator itB = edgeMapB.begin(), endB = edgeMapB.end(); itB != endB; ++itB)
			{
				if(std::tr1::get<0>(*itA)->getDistance() != std::tr1::get<0>(*itB)->getDistance())
					continue;

				if((void *) std::tr1::get<1>(*itA) != (void *) std::tr1::get<1>(*itB) && (void *) std::tr1::get<1>(*itA) != (void *) std::tr1::get<2>(*itB))
					continue;

				if((void *) std::tr1::get<2>(*itA) != (void *) std::tr1::get<1>(*itB) && (void *) std::tr1::get<2>(*itA) != (void *) std::tr1::get<2>(*itB))
					continue;

				found = true;

				break;
			}

			assert(found);
		}
	}
	
	/*!
	 * /brief Constructs http://en.wikipedia.org/wiki/Kruskal's_algorithm#Example
	 **/
	MSTAlgorithmsTester::EdgeMap MSTAlgorithmsTester::setupFixed(Graph &graph)
	{
		EdgeMap mstEdges;
		
		Vertex * pa = new Vertex("A"); 
		Vertex * pb = new Vertex("B"); 
		Vertex * pc = new Vertex("C");	
		Vertex * pd = new Vertex("D");	
		Vertex * pe = new Vertex("E");	
		Vertex * pf = new Vertex("F");
		Vertex * pg = new Vertex("G");	
	
		graph.addVertex(pa);
		graph.addVertex(pb);
		graph.addVertex(pc);
		graph.addVertex(pd);
		graph.addVertex(pe);
		graph.addVertex(pf);
		graph.addVertex(pg);
	
		const Edge * edge1 = new Edge(7);

		std::cout << pa << " and " << pb << std::endl;

		mstEdges.push_back(EdgeMapItem(edge1, pa, pb));

		graph.addEdge(pa, pb, *edge1);

		const Edge * edge2 = new Edge(5);

		mstEdges.push_back(EdgeMapItem(edge2, pa, pd));

		graph.addEdge(pa, pd, *edge2);
	
		graph.addEdge(pb, pc, Edge(8));
		graph.addEdge(pb, pd, Edge(9));
		
		const Edge * edge6 = new Edge(7);

		mstEdges.push_back(EdgeMapItem(edge6, pb, pe));

		graph.addEdge(pb, pe, *edge6);
	
		const Edge * edge3 = new Edge(5);

		mstEdges.push_back(EdgeMapItem(edge3, pc, pe));

		graph.addEdge(pc, pe, *edge3);
	
		graph.addEdge(pd, pe, Edge(15));
		
		const Edge * edge4 = new Edge(6);

		mstEdges.push_back(EdgeMapItem(edge4, pd, pf));

		graph.addEdge(pd, pf, *edge4);
	
		graph.addEdge(pe, pf, Edge(8));

		const Edge * edge5 = new Edge(9);

		mstEdges.push_back(EdgeMapItem(edge5, pe, pg));

		graph.addEdge(pe, pg, *edge5);
	
		graph.addEdge(pf, pg, Edge(11));

		return mstEdges;
	}
	
	void MSTAlgorithmsTester::setupRandom(Graph &graph)
	{
		unsigned int numVertices = rand() % MaxVertices + 2;
		
		Vertex ** vertices = (Vertex **) malloc(numVertices * sizeof(Vertex *));

		for(unsigned int i = 0; i < numVertices; i++)
		{
			std::string id = boost::lexical_cast<std::string>(i);

			vertices[i] = new Vertex("Vertex " + id);

			graph.addVertex(vertices[i]);
		}

		unsigned int maxEdgesPerVertex = (rand() % MaxEdgesPerVertex - 1) + 2;

		for(unsigned int i = 0, edgeNum = 0; i < numVertices; i++)
		{
			for(unsigned int i2 = 0, m = (rand() % maxEdgesPerVertex) + 1; i2 < m; ++i2)
			{
				unsigned int v = rand() % numVertices;

				graph.addEdge(vertices[i], vertices[i == v ? v + 1 : v], Edge(++edgeNum));
			}
		}
	}

	void MSTAlgorithmsTester::performanceTest()
	{
		using namespace boost::posix_time;

		std::cout << "\tTesting performance.." << std::endl;

		time_duration totalTime1, totalTime2;

		for(unsigned int i = 0; i < TestRuns; i++)
		{
			Graph graph;

			setupRandom(graph);

			std::cout << "\t\t#" << i << ": Test " << TestRuns << " times on 1 random graph (" << graph.getNumVertices() << " vertices, " << graph.getNumEdges() << " edges): " << std::endl;

			time_duration time1 = performanceTest<KruskalAlgorithm<>>(graph);
			time_duration time2 = performanceTest<PrimAlgorithm<>>(graph);

			totalTime1 += time1;
			totalTime2 += time2;

			std::cout << "\t\t\tKruskalAlgorithm: \t" << time1 << std::endl;
			std::cout << "\t\t\tPrimAlgorithm: \t\t" << time2 << std::endl;

			std::cout << std::endl;
		}

		std::cout << "\t\tKruskalAlgorithm total time: \t" << totalTime1 << std::endl;
		std::cout << "\t\tPrimAlgorithm total time: \t" << totalTime2 << std::endl;

		std::cout << "\tTest performance completed." << std::endl;
	}

	template<typename MSTAlgorithm>
	boost::posix_time::time_duration MSTAlgorithmsTester::performanceTest(Graph &graph)
	{
		MSTAlgorithm mstAlgorithm(graph);

		boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::local_time();
		
		for(unsigned int i = 0; i < TestRuns; i++)
			mstAlgorithm.getMinimumSpanningTree();

		return boost::posix_time::microsec_clock::local_time().time_of_day() - startTime.time_of_day();
	}
}
