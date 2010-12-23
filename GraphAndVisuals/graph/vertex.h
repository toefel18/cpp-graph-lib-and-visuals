#ifndef GRAPH_VERTEX_H
#define GRAPH_VERTEX_H

#include <string>

namespace graph
{

	/*!
	* \brief A point inside a graph
	*
	* \author Christophe Hesters 
	* \date	12-12-2010 
	* \note tested on VC++ 10.0 and mingw GCC 4.4
	*/
	class Vertex
	{
		std::string name;
	public:
	
		/*!
		* \brief Constructs a vertex
		* 
		* \param name the name of the vertex
		*/
		Vertex(std::string name)
			:name(name) {}

		/*!
		* \brief Gets the name of the vertex
		* 
		* \returns the name of the vertex
		*/
		std::string getName() const {return name;}
	};
}
#endif //GRAPH_VERTEX_H