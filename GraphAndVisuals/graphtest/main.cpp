#include <iostream>
#ifdef __WIN32__
	#include <tuple>
#else
	#include <tr1/tuple>
#endif
#include <utility>

#include "../graph/vertex.h"
#include "../graph/edge.h"
#include "../graph/adjacencylist.h"
#include "../graph/dijkstraalgorithm.h"
//#include "../graph/kruskalalgorithm.h"
//#include "../graph/primalgorithm.h"

#include "dijkstraalgorithmtester.h"
#include "astaralgorithmtester.h"
#include "mstalgorithmstester.h"

using namespace std;

void testDijkstra()
{
	using namespace graph;

	DijkstraAlgorithmTester dijkstraTester;
	dijkstraTester.runTests();
}

void testAStar()
{
	using namespace graph;
	AStarAlgorithmTester astarTester;
	astarTester.runTests();
}


void testMSTAlgorithms()
{
	using namespace graph;

	MSTAlgorithmsTester mstAlgorithmsTester;

	mstAlgorithmsTester.runTests();
}

int main(int argc, char** argv)
{
	testDijkstra();
	testAStar();
	
	testMSTAlgorithms();

	system("pause");

	return 0;
}
