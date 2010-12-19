#ifndef GRAPH_VERTEX_H
#define GRAPH_VERTEX_H

#include <string>

namespace graph
{
	/* 
	* Vertex
	* A point inside a graph
	*
	* Written by Christophe Hesters, 12-12-2010 (tested on VC++ 10.0 and mingw GCC 4.4)
	**/
	class Vertex
	{
		std::string name;
	public:
	
		Vertex(std::string name)
			:name(name) {}

		std::string getName() const {return name;}
	};
}
#endif //GRAPH_VERTEX_H