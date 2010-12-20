#ifndef GRAPH_WEIGHTCOMPUTER_H
#define GRAPH_WEIGHTCOMPUTER_H

#include "edge.h"

namespace graph
{
	/* 
	* Weight Computer
	* E = the edge type 
	* Can compute the distance between two points, it receives an edge and reads 
	* its properties to give a floating point representation for the distance!
	*
	* Written by Christophe Hesters, 13-12-2010 (tested on VC++ 10.0 and mingw GCC 4.4)
	**/
	template <class E = Edge>
	class WeightComputer
	{
	public:
		inline float operator()(const E &edge) const
		{
			return edge.getDistance();
		};
	};

}

#endif //GRAPH_WEIGHTCOMPUTER_H