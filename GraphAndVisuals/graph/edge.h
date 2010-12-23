#ifndef GRAPH_EDGE_H
#define GRAPH_EDGE_H

namespace graph
{

	/*!
	* \brief Describes an edge between to vertices in a graph
	*
	* \author Christophe Hesters 
	* \date	12-12-2010 
	* \note tested on VC++ 10.0 and mingw GCC 4.4
	*/
	class Edge
	{
		float distance;

	public:
		/*!
		* \brief Constructs an edge
		* 
		* \param distance the distance, or weight, of this edge
		*/
		Edge(float distance)
			:distance(distance) {}

		/*!
		* \brief gets the distance property of this edge
		* 
		* \returns the distance
		*/
		float getDistance() const {return distance;}
	};
}
#endif //GRAPH_EDGE_H