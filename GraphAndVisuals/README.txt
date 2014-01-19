This project users QT4 by default. 
With minor changes, QT5 can be used. 

for Qt5 users:
 1. rename CMakeLists-qt5.txt to CMakeLists.txt 			(overwriting the existing CMakeLists.txt)
 2. rename graph/CMakeLists-qt5.txt to CMakeLists.txt 		(overwriting the existing CMakeLists.txt)
 3. rename graphtest/CMakeLists-qt5.txt to CMakeLists.txt 	(overwriting the existing CMakeLists.txt)
 4. rename visualgraph/CMakeLists-qt5.txt to CMakeLists.txt	(overwriting the existing CMakeLists.txt)
 
Run CMake..  

It will complain about invalid includes, this is because QT5 organises things differently. 
This is now illegal:
#include <QtGui/QMainWindow>
#include <QtGui/QWidget>

To fix the errors:
 1. open all source files and remove the QtGui/  prefix in the include directives. (simple find and replace)
 2. open visualgraph.h and visualgraph.cpp, replace Qt::WFlags with Qt::WindowFlags

Run CMake + build again. everything now works with Qt5.