#ifndef ADJACENCYLIST_H
#define ADJACENCYLIST_H

#include <map>
#include <set>
#include <list>
#include <utility>
#include <iterator>
#include <stdexcept>

namespace graph
{
	/*!
	* \brief Const edge iterator, iterates over the edges, provides an abstraction for getting
	*		 the neighbour Vertices and the edge information! You can not adjust the graph trough this iterator!
	*
	* \tparam V the Vertex type
	* \tparam E the edge type
	* \tparam NeighbourList the type of the list that contains the neighbours of a vertex
	*
	* \author Christophe Hesters 
	* \date	19-12-2010 
	* \note tested on VC++ 10.0 and mingw GCC 4.4
	*/
	template <class V, class E, class NeighbourList>
	class ConstGraphEdgeIterator
	{
	protected:
		typename NeighbourList::const_iterator listIter;
		bool isValid;
	public:

		/*!
		* \brief Constructs a default invalid iterator (null iterator)
		*/
		ConstGraphEdgeIterator() :isValid(false){}

		/*!
		* \brief Constructs a valid iterator initialized with an iterator to 
		*		 the neighbour list
		* 
		* \param listIter begin iterator of a the neighbourlist
		*/
		ConstGraphEdgeIterator(const typename NeighbourList::const_iterator& listIter)
			:listIter(listIter), isValid(true){}

		/*!
		* \brief Copy constructor
		* 
		* \param other an other iterator of the same type
		*/
		ConstGraphEdgeIterator(const ConstGraphEdgeIterator& other)
			:listIter(other.listIter), isValid(other.isValid){}

		/*!
		* \brief Gets the vertex the iterator currently points to
		* 
		* \returns the current vertex
		*/
		V* vertex() const { return listIter->first; }

		/*!
		* \brief Gets the edge the iterator currently points to
		* 
		* \returns the current edge
		*/
		const E& edge() const { return listIter->second; }

		/*!
		* \brief check if this iterator is valid! (1 element past the end is also VALID!) 
		*		 invalid means: empty, there are NO elements, valid means, there are elements
		*		 to iterate!
		* 
		* \returns true on valid, false if invalid
		*/
		bool valid() const {return isValid;}

		/*!
		* \brief Increments this iterator (prefix notation)
		* \returns reference to this (which is an incremented iterator)
		*/
		ConstGraphEdgeIterator& operator++(){++listIter; return *this;}

		/*!
		* \brief Increments this iterator (postfix notation)
		* \returns copy of the iterator in its original (unincremented) state
		*/
		ConstGraphEdgeIterator operator++(int){
			ConstGraphEdgeIterator temp = *this;
			++listIter;
			return temp;
		}

		/*!
		* \brief Decrements this iterator (prefix notation)
		* \returns reference to this (which is a decremented iterator)
		*/
		ConstGraphEdgeIterator& operator--(){--listIter; return *this;}

		/*!
		* \brief Decrements this iterator (postfix notation)
		* \returns copy of the iterator in its original (undecrimented) state
		*/
		ConstGraphEdgeIterator operator--(int){				
			ConstGraphEdgeIterator temp = *this;
			--listIter;
			return temp;
		}

		/*!
		* \brief Compares this iterator to another for equalty
		*
		* \param other the other edge iterator
		* \todo review: should 2 invalid iterators be equal?
		* \returns true if the same, false otherwise (two invalid iterators are NOT equal!)
		*/
		bool operator==(const ConstGraphEdgeIterator& other) const{
			return (isValid && other.isValid) ? listIter == other.listIter : false;
		}

		/*!
		* \brief Compares this iterator to another for inequalty
		*
		* \param other the other edge iterator
		* \todo review: should 2 invalid iterators be equal?
		* \returns true if the different, false otherwise (two invalid iterators are NOT equal!)
		*/
		bool operator!=(const ConstGraphEdgeIterator& other) const{
			return (isValid && other.isValid) ? (listIter != other.listIter) : true;
		}
	};

	/*!
	* \brief Represents a graph in as an adjacency list
	*
	* \tparam V represents a vertex in the graph. Internally, only pointers to V are kept
	* \tparam E represents an edge between to vertices in the graph. E must be copy constructable!
	* \tparam NeighbourList the type of the list that contains the neighbours of a vertex
	*
	* \author Christophe Hesters 
	* \date	12-12-2010 
	* \note tested on VC++ 10.0 and mingw GCC 4.4
	*/
	template <class V = Vertex, class E = Edge>
	class AdjacencyList
	{
	public: 
		//These typedefs MUST be public and defined so other structures can use them
		typedef std::set<V*> VertexSet;
		//typedef typename VertexSet::iterator VertexSetItr;  //do not provide modifications (yet!)
		typedef typename VertexSet::const_iterator ConstVertexSetItr;
		typedef std::pair<V*, E> EdgeInfo;
		typedef std::list< EdgeInfo > NeighbourList;
		typedef std::map<V*, NeighbourList > Graph;
		typedef ConstGraphEdgeIterator<V, E, NeighbourList> ConstEdgeIterator;
		
		/*!
		* \brief Constructs an adjacency list graph object
		*/
		AdjacencyList();

		/*!
		* \brief Destructor, does not delete the contained vertices
		*	     that has to be done explicity trough the method 
		*		 deleteAllVertices() 
		*/
		~AdjacencyList();

		/*!
		* \brief Adds a vertex to the graph, but does not take ownership.
		*		 A call to this method is not necessary because addEdge
		*		 also puts vertices inside the graph if they aren't already
		*
		* \param vertex a point to include in the graph
		*/
		void addVertex(V *vertex);

		/*!
		* \brief Connects two vertices together with an edge
		*
		* \param from pointer to the start vertex (will be added to the graph if it is not in the graph already)
		* \param to pointer to the end end vertex (will be added to the graph if it is not in the graph already)
		* \param edge the edge descriptor (will be copied) 
		* \param directed if false we add an extra edge from ´to´ to ´from´.
		*		 this is default behaviour and creates an undirected graph
		*/
		void addEdge(V *from, V *to, const E& edge, bool directed = false);

		///* removes a vertex from the graph
 		//	* TODO: implement! but this requires Edge to provide operator== and !=
		//*  from and to are pointers to vertices 
		//*  edge describes the connection, matches will be deleted!
		//*  if directed = true, the the oposite direction will also be removed if exists!
		//*/
		//void removeEdge(V* from, V* to, const E& edge, bool directed = false);
				

		/*!
		* \brief Gets a const iterator to the begin of the list of neighbour
		*		 vertices reachable from the 'from' vertex, or an invalid  
		*	     iterator if there are no neighbours. 
		*
		* \note analogous to the stl containers .begin() members!
		*
		* \param from pointer to the vertex you want the neighbours from
		* 
		* \returns a constant edge iterator to the beginning of the list, or a null iterator (cannot modify the graph trough this iterator)
		*/
		ConstEdgeIterator neighboursBegin(V *from) const;

		/*!
		* \brief Gets a const iterator to the end of the list of neighbour 
		*		 vertices reachable from the 'from' vertex, or an invalid  
		*	     iterator if there are no neighbours. 
		*
		* \note analogous to the stl containers .end() members!
		*
		* \param from pointer to the vertex you want the neighbours from
		* 
		* \returns  a constant edge iterator to the end of the list, or a null iterator (cannot modify the graph trough this iterator)
		*/
		ConstEdgeIterator neighboursEnd(V *from) const;

		/*!
		* \brief Const iterator to the begin of the set of iterators
		*
		* \returns ConstVertexSetItr;
		*/
		ConstVertexSetItr vertexBegin() const {return vertices.begin();}

		/*!
		* \brief Const iterator to the 1 past the end of the set of iterators
		*
		* \returns ConstVertexSetItr;
		*/
		ConstVertexSetItr vertexEnd() const {return vertices.end();}

		/*!
		* \brief Gets the number of vertices in the graph.
		*
		* \returns number of vertices in the graph;
		*/
		int getNumVertices() const {return numVertices;}
		
		/*!
		* \brief Gets the number of edges in the graph.
		*
		* \returns number of edges in the graph;
		*/
		int getNumEdges() const {return numEdges;}

		/*!
		* \brief Deletes all the vertices in the graph
		*        by invoking the delete operator on them
		*		 (this is not done by the destructor
		*		  because this class does not take ownership!)
		*/
		void deleteAllVertices();
	
	protected:
		VertexSet vertices;

		Graph graph;

		int numVertices;
		int numEdges;

		friend class DijkstraAlgorithmTester;
	};


	template <class V, class E>
	AdjacencyList<V, E>::AdjacencyList()
		:numVertices(0), numEdges(0)
	{}


	template <class V, class E>
	AdjacencyList<V, E>::~AdjacencyList()
	{	}

	template <class V, class E>
	void AdjacencyList<V, E>::addVertex(V *vertex)
	{
		vertices.insert(vertex);
		++numVertices;
	}

	template <class V, class E>
	void AdjacencyList<V, E>::addEdge(V *from, V *to, const E& edge, bool directed)
	{
		//if those vertices are not in the vertices set, add them!
		if(vertices.find(from) == vertices.end())
		{
			vertices.insert(from);
			++numVertices;
		}

		if(vertices.find(to) == vertices.end())
		{
			vertices.insert(to);
			++numVertices;
		}

		//if from does not exist, it's created automatically!
		graph[from].push_back(EdgeInfo(to, edge));
		++numEdges;

		if(!directed)
		{
			graph[to].push_back(EdgeInfo(from, edge));
			++numEdges;
		}
	}

	//not yet implemented
	//template <class V, class E>
	//void AdjacencyList<V, E>::removeEdge(V* from, V* to, const E& edge, bool directed = false)
	//{
	//	//if those vertices are not in the vertices set, add them!
	//	typename Graph::iterator fromVertex = graph.find(from);
	//
	//	if(fromVertex != vertices.end())
	//	{
	//		typename NeighbourList::iterator edgeIterator = fromVertex->second.begin();
	//		//NOTE: edge type must be comparable! this requires operator== in Edge
	//	}
	//
	//	if(!directed)
	//	{
	//		typename Graph::iterator toVertex = graph.find(to);
	//
	//	}
	//}

	template <class V, class E>
	typename AdjacencyList<V, E>::ConstEdgeIterator AdjacencyList<V, E>::neighboursBegin(V *from) const
	{
		typename Graph::const_iterator i = graph.find(from);
		
		//if vertex has no neighbours, return invalid iterator!
		if(i == graph.end()) return ConstEdgeIterator();
			
		return ConstEdgeIterator(i->second.begin());
	}

	template <class V, class E>
	typename AdjacencyList<V, E>::ConstEdgeIterator AdjacencyList<V, E>::neighboursEnd(V *from) const
	{
		typename Graph::const_iterator i = graph.find(from);
		
		//if vertex has no neighbours, return invalid iterator!
		if(i == graph.end()) return ConstEdgeIterator();
			
		return ConstEdgeIterator(i->second.end());
	}

	template <class V, class E>
	void AdjacencyList<V, E>::deleteAllVertices()
	{
		for(ConstVertexSetItr i = vertices.begin(); i != vertices.end(); ++i)
		{
			delete (*i);
		}
	}
}

#endif