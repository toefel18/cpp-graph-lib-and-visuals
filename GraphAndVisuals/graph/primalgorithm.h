#ifndef GRAPH_PRIM_ALGORITHM_H
#define GRAPH_PRIM_ALGORITHM_H

#include <map>
#include <list>
#include <utility>
#include <set>
#include <iterator>

#include "../graph/adjacencylist.h"
#include "../graph/weightcomputer.h"
#include "../graph/vertex.h"
#include "../graph/edge.h"

namespace graph
{

	/*!
	* \brief Prim's Minimum Spanning Tree algorithm.
	*
	* \tparam V Represents a vertex in the graph. Internally, only pointers to V are kept
	* \tparam E Represents an edge between to vertices in the graph. E must be copy constructable!
	* \tparam Weight A functor with this signature: float operator()(const E &edge) const
	* \tparam Graph The object type that contains the graph! 
	*
	* \author Jeffrey Krist
	* \date	5-1-2011 
	* \note Tested on VC++ 10.0
	*/
	template <class V = Vertex, class E = Edge, class Weight = WeightComputer<E>,  class Graph = AdjacencyList<V, E> >
	class PrimAlgorithm
	{	
	public:
	
		struct NonDirectionalGraphException 
		{
		};
		
		/*!
		 * \brief A function to compare the order of EdgeInfo by distance.
		 */
		struct EdgeDistanceComparer
		{
			inline bool operator() (const Edge *edge1, const Edge *edge2) const {
				return edge1->getDistance() < edge2->getDistance();
			}
		};

		/*Type declarations*/
		typedef std::tuple<const E*, const V*, const V*> EdgeMapItem;
		typedef std::vector<EdgeMapItem> EdgeMap;

		/*!
		 * \brief Constucts the algorithm for a given graph
		 *
		 * \param graph a reference to an instance of a Graph object
		 */
		PrimAlgorithm(const Graph &graph);

		~PrimAlgorithm();

		/*! 
		* \brief Returns a Minimum Spanning Tree.
		*
		* \param weightComputer Object which computes the weight (see testclass for instruction)
		*
		* \returns A set of edges.
		*/
		EdgeMap getMinimumSpanningTree(V *firstVertex = 0) const;

	protected:
		const Graph &graph;
	};

	template <class V, class E, class Weight, class Graph>
	PrimAlgorithm<V, E, Weight, Graph>::PrimAlgorithm(const Graph &graph)
		:graph(graph)
	{}

	template <class V, class E, class Weight, class Graph>
	PrimAlgorithm<V, E, Weight, Graph>::~PrimAlgorithm()
	{
	}

	template <class V, class E, class Weight, class Graph>
	typename PrimAlgorithm<V, E, Weight, Graph>::EdgeMap
		PrimAlgorithm<V, E, Weight, Graph>::getMinimumSpanningTree(V *firstVertex) const
	{
		// Prim is only applyable on directional graphs.
		if(!graph.isDirectional())
			throw new NonDirectionalGraphException();

		typedef float DistanceType;

		// Use ordered balanced binary tree for fast lookups, to find reached vertices.
		typedef std::set<V*> ReachedVertices;
		typedef std::pair<V*, V*> VertexPair;
		
		// Use ordered balanced binary tree for fast best edge lookups (always root item) and to map edges to vertices.
		typedef std::multimap<const E*, VertexPair, EdgeDistanceComparer> Edge2vertices;

		Graph::ConstVertexSetItr vertices = graph.vertexBegin(), endVertices = graph.vertexEnd();

		// Abort if graph is empty.
		if(vertices == endVertices)
			return EdgeMap();

		// Select the first vertex with edges if none is given.
		for(; firstVertex == 0 && vertices != endVertices; ++vertices)
		{
			Graph::ConstEdgeIterator neighbours = graph.neighboursBegin(*vertices);

			if(neighbours != graph.neighboursEnd(*vertices))
				firstVertex = *vertices;
		}

		// Abort if no connected edges exists.
		if(firstVertex == 0)
			return EdgeMap();

		Edge2vertices possibleEdges;

		// Add edges of first vertex as possible edges.
		for(Graph::ConstEdgeIterator neighbour = graph.neighboursBegin(firstVertex), endNeighbours = graph.neighboursEnd(firstVertex);
			neighbour != endNeighbours; ++neighbour)
		{
			possibleEdges.insert(std::pair<const E*, VertexPair>(&(neighbour.edge()), VertexPair(firstVertex, neighbour.vertex())));
		}

		EdgeMap selectedEdges;
		ReachedVertices reachedVertices;

		// Mark first vertex as reached.
		reachedVertices.insert(reachedVertices.begin(), firstVertex);

		// While not all vertices are reached and there are possible edges.
		while(reachedVertices.size() != graph.getNumVertices() && possibleEdges.size() > 0)
		{
			// Get best edge.
			Edge2vertices::const_iterator bestEdgeIterator = possibleEdges.begin();

			const E* bestEdge = (*bestEdgeIterator).first;
			V* reachedVertex = std::get<1>((*bestEdgeIterator).second);

			// Store selected edge.
			selectedEdges.insert(selectedEdges.begin(), EdgeMapItem(bestEdge, std::get<0>((*bestEdgeIterator).second), reachedVertex));

			// Mark new vertex as reached.
			reachedVertices.insert(reachedVertices.begin(), reachedVertex);

			// Get edges from new reached vertex.
			ReachedVertices::const_iterator endReachedVertices = reachedVertices.end();

			for(Graph::ConstEdgeIterator neighbour = graph.neighboursBegin(reachedVertex), endNeighbours = graph.neighboursEnd(reachedVertex);
			neighbour != endNeighbours; ++neighbour)
			{
				V* newReachableVertex = neighbour.vertex();

				// Vertex of edge is not reached yet?
				if(reachedVertices.find(newReachableVertex) == endReachedVertices)
				{
					// Add possible edges, which are routing to non reached vertices.
					possibleEdges.insert(std::pair<const E*, VertexPair>(&(neighbour.edge()), VertexPair(reachedVertex, newReachableVertex)));
				}
				else
				{	
					// Remove possible edge from the new reached vertex, to avoid loops.
					Edge2vertices::const_iterator deleteEdge = possibleEdges.find(&(neighbour.edge()));

					if(deleteEdge != possibleEdges.end())
						possibleEdges.erase(deleteEdge);
				}
			}
		}

		// No MST exists if not all vertices are reached.
		if(reachedVertices.size() != graph.getNumVertices())
			return EdgeMap();

		return selectedEdges;
	}
}

#endif //GRAPH_PRIM_ALGORITHM_H