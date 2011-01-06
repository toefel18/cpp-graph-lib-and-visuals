#ifndef GRAPH_KRUSKAL_ALGORITHM_H
#define GRAPH_KRUSKAL_ALGORITHM_H

#include <map>
#include <list>
#include <utility>
#include <queue>
#include <set>
#include <iterator>
#include <hash_map>
#include <tuple>

#include "vertex.h"
#include "edge.h"
#include "adjacencylist.h"
#include "weightcomputer.h"

namespace graph
{

	/*!
	* \brief Kruskal's Minimum Spanning Tree algorithm.
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
	class KruskalAlgorithm
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
		KruskalAlgorithm(const Graph &graph);

		~KruskalAlgorithm();

		/*! 
		* \brief Returns a Minimum Spanning Tree.
		*
		* \param weightComputer Object which computes the weight (see testclass for instruction)
		*
		* \returns A set of edges.
		*/
		EdgeMap getMinimumSpanningTree() const;

	protected:
		const Graph &graph;
	};

	template <class V, class E, class Weight, class Graph>
	KruskalAlgorithm<V, E, Weight, Graph>::KruskalAlgorithm(const Graph &graph)
		:graph(graph)
	{}

	template <class V, class E, class Weight, class Graph>
	KruskalAlgorithm<V, E, Weight, Graph>::~KruskalAlgorithm()
	{
	}

	template <class V, class E, class Weight, class Graph>
	typename KruskalAlgorithm<V, E, Weight, Graph>::EdgeMap
		KruskalAlgorithm<V, E, Weight, Graph>::getMinimumSpanningTree() const
	{
		// Kruskal is only applyable on directional graphs.
		if(!graph.isDirectional())
			throw new NonDirectionalGraphException();

		typedef float DistanceType;

		// Use list for fast inserts/removes because the forest growes and shrinks fast.
		typedef std::list<V*> ForestGraph;
		typedef std::pair<V*, V*> VertexPair;
		 
		// Usa a ordered map to quickly find the best edge. This multimap seems to be quicked then others.
		typedef std::multimap<const E*, const E*, EdgeDistanceComparer> OrderedEdges;

		// Use a map with pretty constant key lookups, to map edges to vertices.
		typedef std::map<const E*, VertexPair> Edge2vertices;

		// Contains a forest (a set of trees), where each vertex in the graph is a separate graph,
		// may have some serieus add/removals.
		std::list<ForestGraph*> forest;

		// Map for speeding up vertex to forestgraph search.
		std::map<V*, ForestGraph*> vertices2ForestGraph;
		
		// Map for speeding up edge to it's vertices.
		Edge2vertices edge2vertices;

		Graph::ConstVertexSetItr vertices = graph.vertexBegin(), endVertices = graph.vertexEnd();

		// Abort if graph is empty.
		if(vertices == endVertices)
			return EdgeMap();

		// Add every vertex to a new graph in the forest and map edges to vertices.
		for(; vertices != endVertices; ++vertices)
		{
			// Create a new graph in the forest, containing one vertex.
			ForestGraph * forestGraph = new ForestGraph();

			forestGraph->insert(forestGraph->end(), *vertices);

			forest.insert(forest.end(), forestGraph);

			// Map vertice to forest graph.
			vertices2ForestGraph[*vertices] = forestGraph;

			// Get edges and map it to vertices. Due the map edges will be unique.
			for(Graph::ConstEdgeIterator neighbourVertices = graph.neighboursBegin(*vertices),
				endNeighbourVertices = graph.neighboursEnd(*vertices);
				neighbourVertices.valid() && neighbourVertices != endNeighbourVertices; ++neighbourVertices)
			{
				edge2vertices.insert(std::pair<const E*, VertexPair>(&(neighbourVertices.edge()), VertexPair(*vertices, neighbourVertices.vertex())));
			}
		}

		// Create a list containing all the edges in the graph, sorted by distance.
		OrderedEdges orderedEdges;

		for(Edge2vertices::const_iterator it = edge2vertices.begin(), end = edge2vertices.end(); it != end; ++it)
		{
			orderedEdges.insert(std::pair<const E*, const E*>((*it).first, (*it).first));
		}

		EdgeMap edgeMap;

		// While edges is nonempty and no single forest exsists (is not yet spanning).
		while(!orderedEdges.empty() && forest.size() > 1)
		{
			// Obtain and remove an edge with minimum weight from edges.
			OrderedEdges::const_iterator bestEdgeIterator = orderedEdges.begin();
		
			const E* minimalEdge = (*bestEdgeIterator).first;

			orderedEdges.erase(bestEdgeIterator);

			// Does the edge connects two different trees?
			VertexPair vertices = edge2vertices[minimalEdge];

			V* vertexA = std::get<0>(vertices);
			V* vertexB = std::get<1>(vertices);

			ForestGraph* forestGraphA = vertices2ForestGraph[vertexA];
			ForestGraph* forestGraphB = vertices2ForestGraph[vertexB];

			if((void *) forestGraphA != (void *) forestGraphB)
			{
				// Unite the two trees into a single tree and discard one.
				forestGraphA->insert(forestGraphA->end(), forestGraphB->begin(), forestGraphB->end());

				for(ForestGraph::const_iterator it = forestGraphB->cbegin(), end = forestGraphB->cend(); it != end; ++it) 
				{
					vertices2ForestGraph[*it] = forestGraphA;
				}

				forest.remove(forestGraphB);

				// Store edge since it is part of the MST.
				edgeMap.push_back(EdgeMapItem(minimalEdge, vertexA, vertexB));
			}
		}

		// No MST exists if forest does not contain a single graph.
		if(forest.size() != 1)
			return EdgeMap();

		return edgeMap;
	}
}

#endif //GRAPH_KRUSKAL_ALGORITHM_H