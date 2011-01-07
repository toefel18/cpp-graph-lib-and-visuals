#ifndef VISUALGRAPH_H
#define VISUALGRAPH_H

#include <QtGui/QMainWindow>
#include <QWidget>
#include "graphscene.h"
#include "../graph/adjacencylist.h"
#include "../graph/edge.h"
#include "guivertex.h"
#include "guiedge.h"

class QGraphicsView;
class QShortcut;

/*!
* \brief Visual Graph main window
*
* \author Christophe Hesters 
* \date	6-1-2011 
* \note tested on VC++ 10.0
*/
class VisualGraph : public QMainWindow
{
	Q_OBJECT

public:
	VisualGraph(QWidget *parent = 0, Qt::WFlags flags = 0);
	~VisualGraph();
	
	typedef graph::AdjacencyList<GuiVertex, EdgeWithPtr> Graph;

private:
	Graph graph;

	GraphScene *graphScene;

	void setupGUI();

	QWidget *mainWidget;
	QGraphicsView *graphicsView;

	QShortcut *addVertexShortcut;
	QShortcut *addEdgesShortcut;
	QShortcut *setStartVertexShortcut;
	QShortcut *setEndVertexShortcut;
	QShortcut *clearAllShortcut;

};

#endif // VISUALGRAPH_H
