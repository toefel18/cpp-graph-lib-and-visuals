#ifndef GRAPH_WEIGHTCOMPUTER_H
#define GRAPH_WEIGHTCOMPUTER_H

#include "edge.h"

namespace graph
{

	/*!
	* \brief Default functor to compute the weight of an edge in a graph.
	*		 it receives an edge and reads its properties to give a floating 
	*		 point representation for the distance!
	*
	* \tparam E an edge between to vertices in the graph.
	*
	* \author Christophe Hesters 
	* \date	13-12-2010 
	* \note tested on VC++ 10.0 and mingw GCC 4.4
	*/
	template <class E = Edge>
	class WeightComputer
	{
	public:

		/*!
		* \brief Computes the weight of the edge
		* 
		* \param edge the edge to compute the weight of
		* 
		* \returns the weight
		*/
		inline float operator()(const E &edge) const
		{
			return edge.getDistance();
		};
	};

}

#endif //GRAPH_WEIGHTCOMPUTER_H