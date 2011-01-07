#include "guiedge.h"
#include "guivertex.h"
#include <QPainter>
#include <QFontMetrics>
#include <algorithm>

GuiEdge::GuiEdge(GuiVertex *vertex1, GuiVertex *vertex2, float distance)
	:vertex1(vertex1), vertex2(vertex2), edge(distance, this)
{
	color = Qt::black;

	weight = QString::number(distance, 'f', 2);
	
	QFontMetrics *metrics = new QFontMetrics(QFont());
	QRect textRect = metrics->boundingRect(weight);
	
	weightWidth = (qreal)std::abs(textRect.x()) + std::abs(textRect.width()) + 13;
	weightHeight = (qreal)std::abs(textRect.y()) + std::abs(textRect.height()) + 13;

	delete metrics;

	vertex1->addEdge(this);
	vertex2->addEdge(this);
	
	updateCoordinates();
}

GuiEdge::~GuiEdge()
{

}

void GuiEdge::updateCoordinates()
{
	p1 = vertex1->scenePos();
	p2 = vertex2->scenePos();

	//locate real centers
	p1.setX(p1.x());// + (vertex1->boundingRect().width() / 2));
	p1.setY(p1.y());// + (vertex1->boundingRect().height() / 2));
	p2.setX(p2.x());// + (vertex2->boundingRect().width() / 2));
	p2.setY(p2.y());// + (vertex2->boundingRect().height() / 2));

	qreal halfPenWidth = 3; //nasty but to reduce artifcats of text
	
	prepareGeometryChange();

	edgeBoundingRect = QRectF( std::min(p1.x(), p2.x()) - halfPenWidth,
		std::min(p1.y(), p2.y()) - halfPenWidth,
		std::max(std::fabs(p2.x() - p1.x()) + halfPenWidth, weightWidth),
		std::max(std::fabs(p2.y() - p1.y()) + halfPenWidth, weightHeight ));
}

void GuiEdge::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	const QBrush &oldBrush = painter->brush();
	const QPen &oldPen = painter->pen();

	painter->setBrush(QBrush(color));
	QPen linePen(color);
	linePen.setWidthF(2.5);
	painter->setPen(linePen);
	painter->drawLine(p1, p2);
	
	float dispX = edgeBoundingRect.width() / 2;
	float dispY = edgeBoundingRect.height() / 2;

	painter->setPen(Qt::black);
	painter->drawText(edgeBoundingRect.x() + dispX,
		edgeBoundingRect.y() + dispY,
		weight);
	
	painter->setBrush(oldBrush);
	painter->setPen(oldPen);
}