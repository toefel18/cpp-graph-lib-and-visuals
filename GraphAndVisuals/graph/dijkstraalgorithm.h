#ifndef GRAPH_DIJKSTRA_ALGORITHM_H
#define GRAPH_DIJKSTRA_ALGORITHM_H

#include <map>
#include <list>
#include <utility>
#include <queue>
#include <set>
#include <iterator>

#include "adjacencylist.h"
#include "weightcomputer.h"

namespace graph
{

	/* 
	* Dijkstra Shortest Paths algorithm 
	*
	* V represents a vertex in the graph. Internally we will only deal with pointers to V
	* E represents an edge between to vertices in the graph. E must be copy constructable!
	* Weight a functor with this signature: float operator()(const E &edge) const
	* Graph is the object that contains the graph! it must provide 'const NeighbourList& getNeighbours(V *from) const;'
	*
	* Written by Christophe Hesters, 14-12-2010 (tested on VC++ 10.0 and mingw GCC 4.4)
	**/
	template <class V = Vertex, class E = Edge, class Weight = WeightComputer<E>,  class Graph = AdjacencyList<V, E> >
	class DijkstraAlgorithm
	{	
	public:
	
		//compare functor, priorityqueue expects the less function
		template <class T> struct PairCompare{
			inline bool operator() (const T& link1, const T& link2) const {
				return link1.second > link2.second; //make sure the lowest compononents go first!
			}
		};

		/*Type declarations*/
		typedef float DistanceType;
		typedef std::pair<V*, DistanceType> Distance;
		typedef std::map<V*, DistanceType> ResultMap;
		typedef std::list< std::pair<V*, E> > Path;

		/*these types may be private*/
		typedef std::set<V*> VertexSet;
		typedef std::priority_queue<Distance,
					std::vector<Distance>,
					PairCompare<Distance> > PQueue;
	
		//Create a dijkstra algorithm object, linking to a graph
		DijkstraAlgorithm(const Graph &graph);
		~DijkstraAlgorithm();

		/** Finds all the shortest paths from the start vertex (Re-entrant!)
		*  start, the starting vertex
		*  weightComputer object which computes the weight
		*  returns a map which maps Vertices to their shortest path lengths
		*/
		ResultMap getShortestPaths(V* start, const Weight &weightComputer = Weight()) const;

		/** Finds all the shortest paths from the start vertex (Re-entrant!)
		*  start, the starting vertex
		*  weightComputer object which computes the weight
		*  returns a list with vertices and their corresponding edges from start to end!
		*/
		Path getShortestPath(V* start, V* end, const Weight &weightComputer = Weight()) const;

	protected:
		const Graph &graph;
		
		////utility function to add items that are not done to the priority queue
		//void addItemsToPQueue(PQueue &queue, 
		//	const typename Graph::NeighbourList &neighbours,
		//	const VertexSet &done,
		//	const Weight &weightComputer, 
		//	DistanceType weightOfCurrentNode) const;

		//utility function to add items that are not done to the priority queue
		void addItemsToPQueue(PQueue &queue, 
			typename Graph::ConstEdgeIterator neighboursBegin,
			typename Graph::ConstEdgeIterator neighboursEnd,
			const VertexSet &done,
			const Weight &weightComputer, 
			DistanceType weightOfCurrentNode) const;

	};

	template <class V, class E, class Weight, class Graph>
	DijkstraAlgorithm<V, E, Weight, Graph>::DijkstraAlgorithm(const Graph &graph)
		:graph(graph)
	{}

	template <class V, class E, class Weight, class Graph>
	DijkstraAlgorithm<V, E, Weight, Graph>::~DijkstraAlgorithm()
	{
	}

	////DEPRICATED, use iterator variant!
	//template <class V, class E, class Weight, class Graph>
	//inline void DijkstraAlgorithm<V, E, Weight, Graph>::addItemsToPQueue(
	//	PQueue &queue, 
	//	const typename Graph::NeighbourList &neighbours, 
	//	const VertexSet &done,
	//	const Weight &weightComputer, 
	//	DistanceType weightOfCurrentNode) const
	//{
	//	for(typename Graph::NeighbourList::const_iterator i = neighbours.begin(); i != neighbours.end(); ++i)
	//	{
	//		if(done.find(i->first) != done.end()) continue; 

	//		Distance distance(i->first, weightComputer(i->second) + weightOfCurrentNode);

	//		queue.push(distance);
	//	}
	//}


	template <class V, class E, class Weight, class Graph>
	inline void DijkstraAlgorithm<V, E, Weight, Graph>::addItemsToPQueue(
		PQueue &queue, 
		typename Graph::ConstEdgeIterator neighboursBegin,
		typename Graph::ConstEdgeIterator neighboursEnd,
		const VertexSet &done,
		const Weight &weightComputer, 
		DistanceType weightOfCurrentNode) const
	{
		//check if the iterator points to a list, if not, the vertex had no reachable neighbours!
		if(neighboursBegin.valid())
		{
			//just use the parameter as iteration
			for(; neighboursBegin != neighboursEnd; ++neighboursBegin)
			{
				if(done.find(neighboursBegin.vertex()) != done.end()) continue; 

				Distance distance(neighboursBegin.vertex(), weightComputer(neighboursBegin.edge()) + weightOfCurrentNode);

				queue.push(distance);
			}
		} 
	}

	template <class V, class E, class Weight, class Graph>
	typename DijkstraAlgorithm<V, E, Weight, Graph>::ResultMap 
		DijkstraAlgorithm<V, E, Weight, Graph>::getShortestPaths(V* start, const Weight& weightComputer) const
	{
		PQueue priorityItems;
		VertexSet done;
		ResultMap shortestPathTo;
		
		// do not process paths leading to the start vertex anymore
		done.insert(start);

		//add all neighbours to pqueue
		//addItemsToPQueue(priorityItems, graph.getNeighbours(start), done, weightComputer, 0);		
		addItemsToPQueue(priorityItems, 
			graph.neighboursBegin(start), 
			graph.neighboursEnd(start), 
			done, weightComputer, 0);		

		while(!priorityItems.empty())
		{
			//get the item with the highest priority and remove it from the queue
			Distance current(priorityItems.top());
			priorityItems.pop();

			//check if it's an item considered done! 
			if( done.find(current.first) != done.end() )
				//we already have the shortest path for this, skip this item
				continue;

			//we can add the result for that node to the result structure
			shortestPathTo[current.first] = current.second;

			//because the current item has the highest priority, we are done with that item!
			done.insert(current.first);
		
			//add all neighbours of the current item, pass the current nodes shortest path as current distance!
			//addItemsToPQueue(priorityItems, graph.getNeighbours(current.first), done, weightComputer, current.second);
			addItemsToPQueue(priorityItems, 
				graph.neighboursBegin(current.first), 
				graph.neighboursEnd(current.first), 
				done, weightComputer, current.second);	
		}

		return shortestPathTo;
	}
}

#endif //GRAPH_DIJKSTRA_ALGORITHM_H