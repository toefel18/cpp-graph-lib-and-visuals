#include "visualgraph.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGraphicsView>
#include <QPainter>

VisualGraph::VisualGraph(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	graphScene = new GraphScene(this, &graph);
	setupGUI();
	setWindowIcon(QIcon(":/VisualGraph/Resources/icon.png"));
	setMinimumSize(400, 400);
}

VisualGraph::~VisualGraph()
{
	delete graphScene;
}

void VisualGraph::setupGUI()
{
	mainWidget = new QWidget;
	QHBoxLayout *mainLayout = new QHBoxLayout;
	graphicsView = new QGraphicsView(graphScene);
	graphicsView->setRenderHint(QPainter::Antialiasing);

	mainLayout->addWidget(graphicsView);

	mainWidget->setLayout(mainLayout);
	setCentralWidget(mainWidget);
}