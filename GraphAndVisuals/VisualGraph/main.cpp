#include "visualgraph.h"
#include <QtGui/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	VisualGraph w;
	w.show();
	return a.exec();
}
