#include "visualgraph.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGraphicsView>
#include <QPainter>
#include <QShortcut>

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

//	addVertexShortcut = new QShortcut(tr("Ctrl+v"), this);
	addEdgesShortcut = new QShortcut(tr("e"), this);
	setStartVertexShortcut = new QShortcut(tr("s"), this);
	setEndVertexShortcut = new QShortcut(tr("t"), this);
	clearAllShortcut = new QShortcut(tr("c"), this);

	connect(addEdgesShortcut, SIGNAL(activated()), graphScene, SLOT(addEdges())); 
	connect(setStartVertexShortcut, SIGNAL(activated()), graphScene, SLOT(setStartVertex())); 
	connect(setEndVertexShortcut, SIGNAL(activated()), graphScene, SLOT(setEndVertex()));	
	connect(clearAllShortcut, SIGNAL(activated()), graphScene, SLOT(clearStartEndVertex())); 
	connect(clearAllShortcut, SIGNAL(activated()), graphScene, SLOT(resetColorsAndText())); 

	mainLayout->addWidget(graphicsView);

	mainWidget->setLayout(mainLayout);
	setCentralWidget(mainWidget);
}