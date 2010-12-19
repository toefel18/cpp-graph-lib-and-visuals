#ifndef GRAPH_EDGE_H
#define GRAPH_EDGE_H

namespace graph
{
	/* 
	* Edge
	* Describes an edge between to vertices in a graph
	*
	* Written by Christophe Hesters, 12-12-2010 (tested on VC++ 10.0 and mingw GCC 4.4)
	**/
	class Edge
	{
		float distance;

	public:

		Edge(float distance)
			:distance(distance) {}

		float getDistance() const {return distance;}
	};

	
	/* 
	* Edge
	* an edge which includes speed, a different weight computer could select speed as the
	* paramter to compute for example the shortest path!
	*
	* Written by Christophe Hesters, 12-12-2010 (tested on VC++ 10.0 and mingw GCC 4.4)
	**/
	class ExtendedEdge : public Edge
	{
		float speed;

	public:
		ExtendedEdge(float distance, float speed = 0)
			:Edge(distance), speed(speed)
		{}

		float getSpeed() const {return speed;}
	};

}
#endif //GRAPH_EDGE_H