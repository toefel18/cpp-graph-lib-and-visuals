#ifndef GRAPH_HEURISTIC_H
#define GRAPH_HEURISTIC_H

#include "vertex.h"

namespace graph
{

	/*!
	* \brief Default heuristic which does not actually provide
	*		 any meaningfull data because it does not now the context
	*		 but it provides an example implementation and returns 0
	*		 for all cases. All implementations should be admissible,
	*		 meaning that they may not overestimate.
	*
	* \tparam V the vertex type
	*
	* \author Christophe Hesters 
	* \date	06-1-2011 
	* \note tested on VC++ 10.0
	*/
	template <class V = Vertex>
	class DefaultHeuristic
	{
	public:

		/*!
		* \brief Default heuristic that returns zero for all cases.
		* 
		* \param current the current node
		* \param goal the target node
		* 
		* \returns heuristic function value between current and goal
		*/
		inline float operator()(const V* current, const V* goal) const
		{
			//returns nothing due to lack of context
			return 0.0f;
		};
	};

}

#endif //GRAPH_HEURISTIC_H