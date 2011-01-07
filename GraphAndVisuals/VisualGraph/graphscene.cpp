#include "graphscene.h"
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsSceneContextMenuEvent>
#include <QMenu>
#include <QAction>
#include "guivertex.h"
#include "guiedge.h"
#include <QInputDialog>
#include "../graph/adjacencylist.h"
#include "../graph/edge.h"
#include "../graph/dijkstraalgorithm.h"
#include "../graph/astaralgorithm.h"

#include "../graph/kruskalalgorithm.h"
#include "../graph/primalgorithm.h"

GraphScene::GraphScene(QObject *parent, Graph *graph)
	: QGraphicsScene(parent), editMode(VERTEX_EDIT), graph(graph)
{
	defaultColor = Qt::black;
	selectedColor = Qt::blue;
	startColor = Qt::red;
	endColor = Qt::green;
	pathVertexColor = Qt::yellow;
	pathEdgeColor = Qt::blue;
}

GraphScene::~GraphScene()
{

}

void GraphScene::mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
	QGraphicsScene::mousePressEvent(mouseEvent);
}

void GraphScene::contextMenuEvent(QGraphicsSceneContextMenuEvent *contextMenuEvent)
{
     QMenu menu;

//	 QAction *vertexEdit = menu.addAction("Set vertex edit mode");
//	 QAction *edgeEdit = menu.addAction("Set edge edit mode");
//	 QAction *selectEdit = menu.addAction("Set select mode");
	 
//	 connect(vertexEdit, SIGNAL(triggered(bool)), this, SLOT(setVertexEditMode()));
//	 connect(edgeEdit, SIGNAL(triggered(bool)), this, SLOT(setEdgeEditMode()));
//	 connect(selectEdit, SIGNAL(triggered(bool)), this, SLOT(setSelectMode()));
	 
	 QAction *addVertex;
	 QAction *addEdge;

	 QAction *runDijkstra;
	 QAction *runDijkstraDistances;
	 QAction *runAStar;
	 QAction *runKruskal;
	 QAction *runPrim;
	 
	 //if(editMode == VERTEX_EDIT)
	 //{
		 addVertex = new QAction("Add vertex", this);
		 menu.addAction(addVertex);
//		 menu.insertAction(vertexEdit, addVertex);
//		 vertexEdit->setEnabled(false);
	 //}
	 //else if (editMode == EDGE_EDIT)
	 //{
		 addEdge = new QAction("Add edge(s)", this);
		 menu.addAction(addEdge);
		 //menu.insertAction(vertexEdit, addEdge);
//		 edgeEdit->setEnabled(false);
	 //}
	 //else
	 //{
		 menu.addSeparator();

		 QMenu *submenu = new QMenu("Run Algorithms");
		 runDijkstra = submenu->addAction("Run Dijkstra's algoritm");
		 connect(runDijkstra, SIGNAL(triggered()), this, SLOT(runDijkstraSPF()));

		 runDijkstraDistances = submenu->addAction("Run Dijkstra's algorithm and get all distances");
		 connect(runDijkstraDistances, SIGNAL(triggered()), this, SLOT(runDijkstraDistances()));
		 
		 runAStar = submenu->addAction("Run A*");
		 connect(runAStar, SIGNAL(triggered()), this, SLOT(runAStar()));

		 runKruskal = submenu->addAction("Run Kruskal's MST algoritm");
		 connect(runKruskal, SIGNAL(triggered()), this, SLOT(runKruskal()));
		 
		 runPrim = submenu->addAction("Run Prim's MST aglorithm");
		 connect(runPrim, SIGNAL(triggered()), this, SLOT(runPrim()));

		 menu.addMenu(submenu);
//		 menu.insertMenu(vertexEdit, submenu);
//		 selectEdit->setEnabled(false);
//	 }

	 QAction *selectedAction = menu.exec(contextMenuEvent->screenPos());

	 if(selectedAction == addVertex)
	 {
		 QPointF pos = contextMenuEvent->scenePos();
		 this->addVertex(pos);
	 } 
	 else if (selectedAction == addEdge)
		 addEdges();
}

void GraphScene::runDijkstraSPF()
{
	resetColorsAndText();

	QList<QGraphicsItem*> items = selectedItems();

	if(items.size() == 2)
	{
		GuiVertex *start = dynamic_cast<GuiVertex*>(items[0]);
		GuiVertex *end = dynamic_cast<GuiVertex*>(items[1]);

		typedef graph::DijkstraAlgorithm<GuiVertex, EdgeWithPtr> Dijkstra;
		Dijkstra dijkstra(*graph);
		Dijkstra::Path path = dijkstra.getShortestPath(start, end);
		
		Dijkstra::Path::iterator itr = path.begin();

		start->setColor(startColor);

		for( ; itr != path.end(); ++itr)
		{
			itr->first->setColor(pathVertexColor);
			itr->second.getGuiEdge()->setColor(pathEdgeColor);
		}

		end->setColor(endColor);
	}
}

void GraphScene::runDijkstraDistances()
{
	resetColorsAndText();

	QList<QGraphicsItem*> items = selectedItems();

	if(items.size() == 1)
	{
		GuiVertex *start = dynamic_cast<GuiVertex*>(items[0]);

		typedef graph::DijkstraAlgorithm<GuiVertex, EdgeWithPtr> Dijkstra;
		Dijkstra dijkstra(*graph);
		Dijkstra::DistanceMap distances = dijkstra.getShortestDistances(start);
		
		Dijkstra::DistanceMap::iterator itr = distances.begin();

		start->setColor(startColor);

		for( ; itr != distances.end(); ++itr)
		{
			itr->first->setText(QString::number(itr->second, 'f', 1));
		}
	}
}

void GraphScene::runAStar()
{
	resetColorsAndText();

	QList<QGraphicsItem*> items = selectedItems();

	if(items.size() == 2)
	{
		GuiVertex *start = dynamic_cast<GuiVertex*>(items[0]);
		GuiVertex *end = dynamic_cast<GuiVertex*>(items[1]);
		
		typedef graph::AStarAlgorithm<GuiVertex, EdgeWithPtr> AStar;
		AStar aStar(*graph);
		AStar::Path path = aStar.getShortestPath(start, end);
		
		AStar::Path::iterator itr = path.begin();

		start->setColor(startColor);

		for( ; itr != path.end(); ++itr)
		{
			itr->first->setColor(pathVertexColor);
			itr->second.getGuiEdge()->setColor(pathEdgeColor);
		}

		end->setColor(endColor);
	}
}

void GraphScene::runKruskal()
{
	resetColorsAndText();
	typedef graph::KruskalAlgorithm<GuiVertex, EdgeWithPtr> Kruskal;
	Kruskal kruskal(*graph);

	Kruskal::EdgeMap edges = kruskal.getMinimumSpanningTree();		
	Kruskal::EdgeMap::iterator itr = edges.begin();

	for( ; itr != edges.end(); ++itr)
	{
		std::get<0>(*itr)->getGuiEdge()->setColor(pathEdgeColor);
	}
}

void GraphScene::runPrim()
{
	resetColorsAndText();
	typedef graph::PrimAlgorithm<GuiVertex, EdgeWithPtr> Prim;
	Prim prim(*graph);

	Prim::EdgeMap edges = prim.getMinimumSpanningTree();
	Prim::EdgeMap::iterator itr = edges.begin();

	for( ; itr != edges.end(); ++itr)
	{
		std::get<0>(*itr)->getGuiEdge()->setColor(pathEdgeColor);
	}
}

void GraphScene::resetColorsAndText()
{
	QList<QGraphicsItem*> graphicsItems = items();
	QList<QGraphicsItem*>::Iterator itr = graphicsItems.begin();

	for(; itr != graphicsItems.end(); ++itr)
	{
		GuiVertex *vertex = dynamic_cast<GuiVertex *>(*itr);
		if(vertex == 0)
		{
			GuiEdge *edge = dynamic_cast<GuiEdge *>(*itr);
			if(edge != 0)
				edge->setColor(Qt::black);
		} 
		else 
		{
			vertex->setColor(Qt::black);
			vertex->setText(QString(""));
		}
	}
}

void GraphScene::addVertex(const QPointF &location)
{
	GuiVertex *vertex = new GuiVertex;
	vertex->setPos(location);

	addItem(vertex);
	graph->addVertex(vertex);
}

void GraphScene::addEdges()
{
	QList<QGraphicsItem*> items = selectedItems();
	if (items.size() <= 1) return;

	bool ok;
	double weight = QInputDialog::getDouble(0, "Weight for all the edges", 
		"Enter the weight value for the edges about to be connected!",
		1.0, 0.0, 2147483647.0, 3, &ok);
	
	if(ok == false) return;

	for(int current = 0; current < items.size(); current++)
	{
		GuiVertex *currentVertex = dynamic_cast<GuiVertex *>(items[current]);
		if(currentVertex == 0)
			continue;

		for(int next = current+1; next < items.size(); next++)
		{
			GuiVertex *targetVertex = dynamic_cast<GuiVertex *>(items[next]);
			if(targetVertex == 0)
				continue;

			GuiEdge *guiEdge = new GuiEdge(currentVertex, targetVertex, (float)weight);
			addItem(guiEdge);
			graph->addEdge(currentVertex, targetVertex, guiEdge->getEdge());
		}
	}
}
