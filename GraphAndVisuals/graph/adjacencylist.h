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
	/**
	* Const Edge iterator, iterates over the edges, provides an abstraction for getting
	* the neighbour and the edge information! You can not adjust the graph trough this iterator!
	*
	* Written by Christophe Hesters, 19-12-2010 (tested on VC++ 10.0 and mingw GCC 4.4)
	*/
	template <class V, class E, class NeighbourList>
	class ConstGraphEdgeIterator
	{
	protected:
		typename NeighbourList::const_iterator listIter;
		bool isValid;
	public:
		//construct invalid iterator
		ConstGraphEdgeIterator() :isValid(false){}

		//construct a valid iterator 
		ConstGraphEdgeIterator(const typename NeighbourList::const_iterator& listIter)
			:listIter(listIter), isValid(true){}

		//copy constructor
		ConstGraphEdgeIterator(const ConstGraphEdgeIterator& other)
			:listIter(other.listIter), isValid(other.isValid){}

		//get the neighbour 
		V* vertex() const { return listIter->first; }

		//get the edge descriptor between the current node and the neighbour
		const E& edge() const { return listIter->second; }

		//check if this iterator is valid! (1 element past the end is also VALID!) 
		//invalid means: empty, there are NO elements, valid means, there are elements
		//to iterate!
		bool valid() const {return isValid;}

		//prefix operator
		ConstGraphEdgeIterator& operator++(){++listIter; return *this;}

		//postfix operator
		ConstGraphEdgeIterator operator++(int){
			ConstGraphEdgeIterator temp = *this;
			++listIter;
			return temp;
		}

		//prefix operator
		ConstGraphEdgeIterator& operator--(){--listIter; return *this;}

		//postfix operator
		ConstGraphEdgeIterator operator--(int){				
			ConstGraphEdgeIterator temp = *this;
			--listIter;
			return temp;
		}

		//equalty operator
		bool operator==(const ConstGraphEdgeIterator& other) const{
			return (isValid && other.isValid) ? listIter == other.listIter : false;
		}

		//inequalty operator
		bool operator!=(const ConstGraphEdgeIterator& other) const{
			return (isValid && other.isValid) ? (listIter != other.listIter) : true;
		}
	};

	/* 
	* Generic Adjacency List 
	* represents a graph in an adjacency list
	* V represents a vertex in the graph. Internally we will only deal with pointers to V
	* E represents an edge between to vertices in the graph. E must be copy constructable!
	*
	* Written by Christophe Hesters, 12-12-2010 (tested on VC++ 10.0 and mingw GCC 4.4)
	**/
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

		//Graph will boil down to:
		//std::map<V*, std::list< std::pair<V*, E> > >
		
		

		AdjacencyList();

		//does not delete the vertices!
		~AdjacencyList();

		/* adds a vertex to the graph, but does not take ownership!
		*
		*  vertex a point to include in the graph
		*/
		void addVertex(V *vertex);

		/* adds a vertex to the graph, but does not take ownership!
		*
		*  from and to are pointers to existing vertices (they will be added to the graph if unkown!)
		*  edge is an object that describes an edge, must be copy constructable
		*  directed, if false we add an extra edge from ´to´ to ´from´
		*/
		void addEdge(V *from, V *to, const E& edge, bool directed = false);

		///* removes a vertex from the graph
 		//	* TODO: implement! but this requires Edge to provide operator== and !=
		//*  from and to are pointers to vertices 
		//*  edge describes the connection, matches will be deleted!
		//*  if directed = true, the the oposite direction will also be removed if exists!
		//*/
		//void removeEdge(V* from, V* to, const E& edge, bool directed = false);

		//TODO, convert reference to pointer and return 0 if not exists?
		//TODO, implement iterator

		///* DEPRICATED USE ITERATORS!!!!
		//*  returns a list with all the vertices reachable from vertex from
		//*  
		//*  from is the vertex whose neighbours will be returned
		//*  returns a constant list with neighbours and corresponding edge descriptors!
		//*/
		//const NeighbourList& getNeighbours(V *from) const;
		
		/*
		* THESE ITERATORS ARE READ-ONLY! No modification provided yet!
		*/

		//begin iterator for the neighbours and their edges, if iterator.valid() == false, there are none!
		ConstEdgeIterator neighboursBegin(V *from) const;

		//end iterator for the neighbours and their edges, if iterator.valid() == false, there are none!
		ConstEdgeIterator neighboursEnd(V *from) const;

		//an iterator to the first of all the known vertices in the graph
		ConstVertexSetItr vertexBegin() const {return vertices.begin();}

		//an iterator to 1 past the end of all the known vertices in the graph
		ConstVertexSetItr vertexEnd() const {return vertices.end();}

		//returns the number of vertices in this graph
		int getNumVertices() const {return numVertices;}
		
		//returns the number of edges in this graph
		int getNumEdges() const {return numEdges;}

		//deletes all vertices in this set! (this is not done by the destructor automatically!)
		void deleteAllVertices();
	
	protected:
		VertexSet vertices;

		//std::map<V*, std::list< std::pair<V*, E> > > graph;
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


	//test if those vertices are in the list! if not add them?
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

	//DEPRICATED, USE ITERATORS!
	//template <class V, class E>
	//const typename AdjacencyList<V, E>::NeighbourList& AdjacencyList<V, E>::getNeighbours(V *from) const
	//{
	//	typename Graph::const_iterator i = graph.find(from);
	//	
	//	if(i == graph.end())
	//	{
	//		if(vertices.find(from) == vertices.end())
	//			throw std::logic_error("in getNeighbours(V*) -> vertex inside graph, but does not have any outgoing arcs!");
	//
	//		throw std::out_of_range("in getNeighbours(V*) -> vertex not inside the graph!");
	//	}
	//	return i->second;
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