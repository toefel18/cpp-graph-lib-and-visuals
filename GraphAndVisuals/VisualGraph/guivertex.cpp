#include "guivertex.h"
#include <QPainter>
#include "guiedge.h"

GuiVertex::GuiVertex()
	: QGraphicsItem(0)
{
	color = Qt::black;
	textColor = Qt::white;
	qreal penWidth = 1;
	vertexBoundingRect = QRectF(-10 - penWidth / 2, -10 - penWidth / 2,
					20 + penWidth, 20 + penWidth);

	setFlags(ItemIsMovable | ItemIsSelectable | ItemSendsGeometryChanges);
	setZValue(1);
	innerText = QString("");
	innerTextHeight = 15;
}

GuiVertex::~GuiVertex()
{
	//delete attached edges?
}


void GuiVertex::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	const QBrush &oldBrush = painter->brush();
	const QPen &oldPen = painter->pen();

	if(isSelected())
		painter->setBrush(QBrush(Qt::blue, Qt::Dense2Pattern));
	else
		painter->setBrush(QBrush(color, Qt::SolidPattern));
	
	painter->drawEllipse(vertexBoundingRect);

	painter->setPen(textColor);
	painter->drawText(QPointF(vertexBoundingRect.x()+1, vertexBoundingRect.y()+innerTextHeight), innerText);

	painter->setBrush(oldBrush);
	painter->setPen(oldPen);
}

void GuiVertex::addEdge(GuiEdge *edge)
{
	edges.push_back(edge);
}

QVariant GuiVertex::itemChange(GraphicsItemChange change, const QVariant &value)
{
	if (change == ItemPositionChange && scene()) {
		for(EdgeList::iterator i = edges.begin(); i != edges.end(); ++i)
			(*i)->updateCoordinates();
	}

	return QGraphicsItem::itemChange(change, value);
}