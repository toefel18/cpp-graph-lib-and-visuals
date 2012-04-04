#ifndef GRAPHSCENE_H
#define GRAPHSCENE_H

#include <QtGui/QGraphicsScene>

class QGraphicsSceneMouseEvent;
class QGraphicsSceneContextMenuEvent;
class GuiVertex;
class EdgeWithPtr;

namespace graph{
	class Edge;

	template <class GuiVertex, class EdgeWithPtr>
	class AdjacencyList;
}

/*!
* \brief Scene that contains and builds a graph
*
* \author Christophe Hesters 
* \date	6-1-2011 
* \note tested on VC++ 10.0
*/
class GraphScene : public QGraphicsScene
{
	Q_OBJECT

public:
	typedef graph::AdjacencyList<GuiVertex, EdgeWithPtr> Graph;
	
	GraphScene(QObject *parent, Graph *graph);
	~GraphScene();

	enum EditMode {
		VERTEX_EDIT = 1,
//		EDGE_EDIT,
		SELECT
	};

public slots:
	void setVertexEditMode(){ editMode = VERTEX_EDIT; }
//	void setEdgeEditMode(){ editMode = EDGE_EDIT; }
	void setSelectMode(){ editMode = SELECT; }

	void addVertex(const QPointF &location);
	void addEdges();

	void runDijkstraSPF();
	void runDijkstraDistances();
	void runAStar();
	void runKruskal();
	void runPrim();
	
	//sets the current selected vertex as start vertex
	void setStartVertex();
	void setEndVertex();
	void clearStartEndVertex();

	void resetColorsAndText();

protected:

	virtual void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent);
	virtual void contextMenuEvent(QGraphicsSceneContextMenuEvent *contextMenuEvent);

	QColor defaultColor;
	QColor selectedColor;
	QColor startColor;
	QColor endColor;
	QColor pathEdgeColor;
	QColor pathVertexColor;

	GuiVertex *startVertex;
	GuiVertex *endVertex;

	EditMode editMode;
	Graph *graph;
};

#endif // GRAPHSCENE_H
