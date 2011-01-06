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
#include "heuristic.h"

namespace graph
{
	
	int iterationCount;

	/*!
	* \brief A* Shortest Paths algorithm 
	*
	* \tparam V represents a vertex in the graph. Internally, only pointers to V are kept
	* \tparam E represents an edge between to vertices in the graph. E must be copy constructable!
	* \tparam Weight a functor with this signature: float operator()(const E &edge) const
	* \tparam Heuristic a functor with this signature: float operator()(const V *current, const V *goal) const
	* \tparam Graph the object type that contains the graph! 
    *
	* \author Christophe Hesters 
	* \date	6-01-2011 
	* \note tested on VC++ 10.0
	*/
	template <class V = Vertex, 
		class E = Edge, 
		class Weight = WeightComputer<E>, 
		class Heuristic = DefaultHeuristic<V>, 
		class Graph = AdjacencyList<V, E> >
	class AStarAlgorithm
	{	
	public:

		/*Type declarations*/
		typedef float DistanceType;
		typedef std::set<V*> VertexSet;
		
		/*Type declarations for tracing the complete path*/
		typedef std::pair<V*, E> EdgeInfo;
		typedef std::list< EdgeInfo > Path;
		typedef std::map<V*, Path> PathMap; //maps shortest path from start to V*

		/*!
		 * \brief a structure used internally to keep track of the complete paths
		 */
		struct DistancePathDescriptor {
			DistanceType distance;			//the shortest distance from start node 
			DistanceType heuristicDistance; //the heuristic of the current node
			V* previous;					//the node that put the current node in the pqueue
			V* current;						//the current node
			const E* edge;					//the edge descriptor, needs to be a pointer because of assignment!

			DistancePathDescriptor(DistanceType distance, DistanceType heuristicDistance, V* previous, V* current, const E* edge)
				:distance(distance), heuristicDistance(heuristicDistance), previous(previous), current(current), edge(edge) {}
		};

		/*!
		 * \brief a functor to compare the order of DistancePathDescriptor structs in a pqueue
		 */
		struct DistancePathDescriptorCompare{
			inline bool operator() (const DistancePathDescriptor& desc1, const DistancePathDescriptor& desc2) const {
				//make sure the lowest compononents go first!
				return (desc1.distance + desc1.heuristicDistance) > (desc2.distance + desc2.heuristicDistance); 
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
		AStarAlgorithm(const Graph &graph);

		~AStarAlgorithm();


		/*! 
        * \brief Finds the shortest paths from the start vertex to end vertex
		*
		* \param start the starting vertex
		* \param end the destination vertex
		* \param weightComputer object which computes the weight (see testclass for instruction)
		* \param heuristic object which returns the heuristic value between current node and the goal node
        * \reentrant
		*
		* \returns a list with the path from the start vertex to the end vertex!
		*/
		Path getShortestPath(V* start, 
			V* end, 
			const Weight &weightComputer = Weight(),
			const Heuristic &heuristic = Heuristic()) const;

	protected:
		const Graph &graph;
	
		void addItemsToPathDescPQueue(PathDescPQueue &queue,
			V* addedBy,
			V* goal,
			typename Graph::ConstEdgeIterator neighboursBegin,
			typename Graph::ConstEdgeIterator neighboursEnd,
			const VertexSet &done,
			const Weight &weightComputer,
			const Heuristic &heuristic,
			DistanceType weightOfCurrentNode) const;
	};

	template <class V, class E, class Weight, class Heuristic, class Graph>
	AStarAlgorithm<V, E, Weight, Heuristic, Graph>::AStarAlgorithm(const Graph &graph)
		:graph(graph)
	{}

	template <class V, class E, class Weight, class Heuristic, class Graph>
	AStarAlgorithm<V, E, Weight, Heuristic, Graph>::~AStarAlgorithm()
	{
	}

	template <class V, class E, class Weight, class Heuristic, class Graph>
	inline void AStarAlgorithm<V, E, Weight, Heuristic, Graph>::addItemsToPathDescPQueue(
		PathDescPQueue &queue, 
		V* addedBy,
		V* goal,
		typename Graph::ConstEdgeIterator neighboursBegin,
		typename Graph::ConstEdgeIterator neighboursEnd,
		const VertexSet &done,
		const Weight &weightComputer, 
		const Heuristic &heuristic,
		DistanceType weightOfCurrentNode) const
	{
		//check if the iterator points to a list, if not, the vertex had no reachable neighbours!
		if(neighboursBegin.valid())
		{
			//just use the parameter as iteration
			for(; neighboursBegin != neighboursEnd; ++neighboursBegin)
			{
				if(done.find(neighboursBegin.vertex()) != done.end()) continue; 

				DistancePathDescriptor descriptor(
					weightComputer(neighboursBegin.edge()) + weightOfCurrentNode,
					heuristic(neighboursBegin.vertex(), goal),
					addedBy, neighboursBegin.vertex(), &neighboursBegin.edge() );

				queue.push(descriptor);
			}
		} 
	}

	template <class V, class E, class Weight, class Heuristic, class Graph>
	typename AStarAlgorithm<V, E, Weight, Heuristic, Graph>::Path 
		AStarAlgorithm<V, E, Weight, Heuristic, Graph>::getShortestPath(V* start, V* end, const Weight &weightComputer, const Heuristic &heuristic) const
	{
		PathMap shortestPathTo;
		PathDescPQueue priorityItems;
		VertexSet done;
		iterationCount = 0;
				
		// do not process paths leading to the start vertex anymore
		done.insert(start);

		//add all neighbours to pqueue
		//addItemsToPQueue(priorityItems, graph.getNeighbours(start), done, weightComputer, 0);		
		addItemsToPathDescPQueue(priorityItems, 
			start,
			end,
			graph.neighboursBegin(start), 
			graph.neighboursEnd(start), 
			done, weightComputer, heuristic, 0);		
	
		while(!priorityItems.empty())
		{
			graph::iterationCount++;
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
			if(current.current == end) 
				return shortestPathTo[end];

			//because the current item has the highest priority and 
			//we processed it, we are done with that item!
			done.insert(current.current);
		
			//add all neighbours of the current item, pass the current nodes shortest path as current distance!
			//addItemsToPQueue(priorityItems, graph.getNeighbours(current.first), done, weightComputer, current.second);
			addItemsToPathDescPQueue(priorityItems, 
				current.current,
				end,
				graph.neighboursBegin(current.current), 
				graph.neighboursEnd(current.current), 
				done, weightComputer, heuristic, current.distance);	
		}


		//if the iterator and optimization is used,(described in the 
		//computeShortestPaths method) then make sure here
		//that the shortest path returned does not contain references
		//to deleted lists ( which were referenced in the local shortestPathsTo)!
		return Path();		
	}
}

#endif //GRAPH_DIJKSTRA_ALGORITHM_H
