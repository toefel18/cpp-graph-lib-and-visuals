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

	/*!
	* \brief Dijkstra Shortest Paths algorithm 
	*
	* \tparam V represents a vertex in the graph. Internally, only pointers to V are kept
	* \tparam E represents an edge between to vertices in the graph. E must be copy constructable!
	* \tparam Weight a functor with this signature: float operator()(const E &edge) const
	* \tparam Graph the object type that contains the graph! 
	*
	* \author Christophe Hesters 
	* \date	14-12-2010 
	* \note tested on VC++ 10.0 and mingw GCC 4.4
	*/
	template <class V = Vertex, class E = Edge, class Weight = WeightComputer<E>,  class Graph = AdjacencyList<V, E> >
	class DijkstraAlgorithm
	{	
	public:
	
		/*!
		 * \brief compare functor for pairs, comparing the second element to determine the order in pqueue
		 */
		template <class T> struct PairCompare{
			inline bool operator() (const T& link1, const T& link2) const {
				return link1.second > link2.second; //make sure the lowest compononents go first!
			}
		};

		/*Type declarations*/
		typedef float DistanceType;
		typedef std::pair<V*, DistanceType> Distance;
		typedef std::map<V*, DistanceType> DistanceMap;
		typedef std::set<V*> VertexSet;
		typedef std::priority_queue<Distance,
					std::vector<Distance>,
					PairCompare<Distance> > PQueue;
		
		/*Type declarations for tracing the complete path*/
		typedef std::pair<V*, E> EdgeInfo;
		typedef std::list< EdgeInfo > Path;
		typedef std::map<V*, Path> PathMap; //maps shortest path from start to V*

		/*!
		 * \brief a structure used internally to keep track of the complete paths
		 */
		struct DistancePathDescriptor {
			DistanceType distance; //the shortest distance from start node
			V* previous; //the node that put the current node in the pqueue
			V* current; //the current node
			const E* edge; //the edge descriptor, needs to be a pointer because of assignment!

			DistancePathDescriptor(DistanceType distance, V* previous, V* current, const E* edge)
				:distance(distance), previous(previous), current(current), edge(edge) {}
		};

		/*!
		 * \brief a functor to compare the order of DistancePathDescriptor structs in a pqueue
		 */
		struct DistancePathDescriptorCompare{
			inline bool operator() (const DistancePathDescriptor& desc1, const DistancePathDescriptor& desc2) const {
				return desc1.distance > desc2.distance; //make sure the lowest compononents go first!
			}
		};

		typedef std::priority_queue<DistancePathDescriptor,
					std::vector<DistancePathDescriptor>,
					DistancePathDescriptorCompare > PathDescPQueue;

	
		/*!
		 * \brief Constucts the algorithm for a given graph
		 *
		 * \param graph a reference to an instance of a Graph object
		 */
		DijkstraAlgorithm(const Graph &graph);

		~DijkstraAlgorithm();

		/*! 
		* \brief Finds all the shortest path lengths from the start 
		*		 vertex to all the other reachable vertices (Re-entrant!)
		*
		* \param start the starting vertex to compute 
		* \param weightComputer object which computes the weight (see testclass for instruction)
		*
		* \returns a map which maps Vertices to their shortest path lengths
		*/
		DistanceMap getShortestDistances(V* start, const Weight &weightComputer = Weight()) const;

		/*! 
		* \brief Finds the shortest paths for all nodes starting from the start vertex (Re-entrant!)
		*
		* \param start the starting vertex
		* \param weightComputer object which computes the weight (see testclass for instruction)
		*
		* \returns a map that maps all reachable vertices to a list with their paths relative from the start vertex!
		*/
		PathMap getShortestPaths(V* start, const Weight &weightComputer = Weight()) const;

		/*! 
		* \brief Finds the shortest paths from the start vertex to end vertex(Re-entrant!)
		*
		* \param start the starting vertex
		* \param end the destination vertex
		* \param weightComputer object which computes the weight (see testclass for instruction)
		*
		* \returns a list with the path from the start vertex to the end vertex!
		*/
		Path getShortestPath(V* start, V* end, const Weight &weightComputer = Weight()) const;

	protected:
		const Graph &graph;
	
		//utility function to add items that are not done to the priority queue
		void addItemsToPQueue(PQueue &queue, 
			typename Graph::ConstEdgeIterator neighboursBegin,
			typename Graph::ConstEdgeIterator neighboursEnd,
			const VertexSet &done,
			const Weight &weightComputer, 
			DistanceType weightOfCurrentNode) const;

		void addItemsToPathDescPQueue(PathDescPQueue &queue,
			V* addedBy,
			typename Graph::ConstEdgeIterator neighboursBegin,
			typename Graph::ConstEdgeIterator neighboursEnd,
			const VertexSet &done,
			const Weight &weightComputer, 
			DistanceType weightOfCurrentNode) const;

		/** Computes the shortest paths and stores the result in shortestPathTo
		* start is the starting point
		* end, if null all paths will be searched, if not null, the algorithm will stop if it reached that node!
		* weightComputer computes the weight of an edge
		*/
		void computeShortestPaths(PathMap& shortestPathTo, V* start, V* end, const Weight &weightComputer) const;
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
	typename DijkstraAlgorithm<V, E, Weight, Graph>::DistanceMap 
		DijkstraAlgorithm<V, E, Weight, Graph>::getShortestDistances(V* start, const Weight& weightComputer) const
	{
		PQueue priorityItems;
		VertexSet done;
		DistanceMap shortestPathTo;
		
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



	template <class V, class E, class Weight, class Graph>
	inline void DijkstraAlgorithm<V, E, Weight, Graph>::addItemsToPathDescPQueue(
		PathDescPQueue &queue, 
		V* addedBy,
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

				DistancePathDescriptor descriptor(weightComputer(neighboursBegin.edge()) + weightOfCurrentNode,
					addedBy, neighboursBegin.vertex(), &neighboursBegin.edge() );

				queue.push(descriptor);
			}
		} 
	}

	template <class V, class E, class Weight, class Graph>
	typename DijkstraAlgorithm<V, E, Weight, Graph>::PathMap 
		DijkstraAlgorithm<V, E, Weight, Graph>::getShortestPaths(V* start, const Weight &weightComputer) const
	{
		PathMap shortestPathTo;

		//0 will not stop the computation, and therefore compute all paths!
		computeShortestPaths(shortestPathTo, start, 0, weightComputer);

		return shortestPathTo;
	}

	template <class V, class E, class Weight, class Graph>
	void DijkstraAlgorithm<V, E, Weight, Graph>::computeShortestPaths(PathMap& shortestPathTo,
		V* start, V* end, const Weight &weightComputer) const
	{
		PathDescPQueue priorityItems;
		VertexSet done;
				
		// do not process paths leading to the start vertex anymore
		done.insert(start);

		//add all neighbours to pqueue
		//addItemsToPQueue(priorityItems, graph.getNeighbours(start), done, weightComputer, 0);		
		addItemsToPathDescPQueue(priorityItems, 
			start,
			graph.neighboursBegin(start), 
			graph.neighboursEnd(start), 
			done, weightComputer, 0);		

		while(!priorityItems.empty())
		{
			//get the item with the highest priority and remove it from the queue
			DistancePathDescriptor current(priorityItems.top());
			priorityItems.pop();

			//check if it's an item considered done! 
			if( done.find(current.current) != done.end() )
				//we already have the shortest path for this, skip this item
				continue;
			
			//create the shortest path to the current vertex
			Path& pathToCurrent = shortestPathTo[current.current];

			if(current.previous != start)
			{
				const Path& shortestPathOfPrevious = shortestPathTo[current.previous];

				//TODO mabybe it would be more efficient in time AND space, to include a reference
				//to the path list of the previous node instead of copying it's complete path!
				//(option: write an iterator that handles the referencing!)
				//IF YOU DO THIS OPTIMIZATION, ADAPT getShortestPath accordingly
				pathToCurrent.insert(pathToCurrent.end(), 
					shortestPathOfPrevious.begin(),
					shortestPathOfPrevious.end());
			}

			// add the last step in the path to the current node!
			pathToCurrent.push_back(EdgeInfo(current.current, *current.edge));

			//shortestPathTo[current.current] now contains the shortest path from start to current.current!

			//if this is the end vertex, STOP the computation
			if(current.current == end) return;

			//because the current item has the highest priority and 
			//we processed it, we are done with that item!
			done.insert(current.current);
		
			//add all neighbours of the current item, pass the current nodes shortest path as current distance!
			//addItemsToPQueue(priorityItems, graph.getNeighbours(current.first), done, weightComputer, current.second);
			addItemsToPathDescPQueue(priorityItems, 
				current.current,
				graph.neighboursBegin(current.current), 
				graph.neighboursEnd(current.current), 
				done, weightComputer, current.distance);	
		}
	}

	template <class V, class E, class Weight, class Graph>
	typename DijkstraAlgorithm<V, E, Weight, Graph>::Path 
		DijkstraAlgorithm<V, E, Weight, Graph>::getShortestPath(V* start, V* end, const Weight &weightComputer) const
	{
		PathMap shortestPathTo;

		computeShortestPaths(shortestPathTo, start, end, weightComputer);

		//if the iterator and optimization is used,(described in the 
		//computeShortestPaths method) then make sure here
		//that the shortest path returned does not contain references
		//to deleted lists ( which were referenced in the local shortestPathsTo)!
		return shortestPathTo[end];		
	}
}

#endif //GRAPH_DIJKSTRA_ALGORITHM_H