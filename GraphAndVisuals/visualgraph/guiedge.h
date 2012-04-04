#ifndef GUIEDGE_H
#define GUIEDGE_H

#include <QtGui/QGraphicsItem>
#include "../graph/edge.h"

class GuiVertex;
class GuiEdge;

/*!
* \brief Leightweight Edge wrapper that contains a pointer
*		 to a graphical edge
*
* \author Christophe Hesters 
* \date	7-1-2011 
* \note tested on VC++ 10.0
*/
class EdgeWithPtr : public graph::Edge
{
public:
	EdgeWithPtr(float distance, GuiEdge *guiEdge)
		:graph::Edge(distance),guiEdge(guiEdge) {}

	GuiEdge *getGuiEdge() const {return guiEdge;}

protected:
	GuiEdge *guiEdge;
};

/*!
* \brief Graphical Edge class that connects two GuiVertex
*		 objects. It also displays the weight
*
* \author Christophe Hesters 
* \date	6-1-2011 
* \note tested on VC++ 10.0
*/
class GuiEdge : public QGraphicsItem
{
public:
	GuiEdge(GuiVertex *vertex1, GuiVertex *v2, float distance);
	~GuiEdge();

	void updateCoordinates();

	virtual QRectF boundingRect() const {return edgeBoundingRect;}
	virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

	void setColor(QColor color){this->color = color; update(edgeBoundingRect);}
	
	EdgeWithPtr getEdge(){return edge;}
	const GuiVertex* getVertex1(){return vertex1;}
	const GuiVertex* getVertex2(){return vertex2;}

protected:
	QColor color;
	QRectF edgeBoundingRect;

	QPointF p1;
	QPointF p2;

	QString weight;
	qreal weightWidth;
	qreal weightHeight;

	GuiVertex *vertex1;
	GuiVertex *vertex2;
	EdgeWithPtr edge;
};

#endif // GUIEDGE_H
