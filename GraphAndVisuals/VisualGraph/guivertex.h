#ifndef GUIVERTEX_H
#define GUIVERTEX_H

#include <QGraphicsEllipseItem>
#include <vector>
#include <QColor>

class GuiEdge;

/*!
* \brief Graphical vertex that can be connected by edges
*
* \author Christophe Hesters 
* \date	6-1-2011 
* \note tested on VC++ 10.0
*/
class GuiVertex : public QGraphicsItem
{

public:
	GuiVertex();
	~GuiVertex();

	QColor getColor(){return color;}

	virtual QRectF boundingRect() const {return vertexBoundingRect;}
	virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

	void setColor(QColor color){this->color = color; update(vertexBoundingRect);}
	void setTextColor(QColor color){this->textColor = color; update(vertexBoundingRect);}
	void setText(QString text){this->innerText = text; update(vertexBoundingRect);}

	void addEdge(GuiEdge* edge);
	
protected:
	QColor color;
	QColor textColor;
	QRectF vertexBoundingRect;
	
	QString innerText;
	qreal innerTextHeight;

	virtual QVariant itemChange(GraphicsItemChange change, const QVariant &value);

	typedef std::vector<GuiEdge*> EdgeList;
	EdgeList edges;
};

#endif //GUIVERTEX_H
